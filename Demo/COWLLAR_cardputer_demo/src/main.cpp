/*
 * main.cpp — COWLLAR live activity classifier demo
 * -----------------------------------------------------------------------------
 * Target : M5Stack Cardputer Adv (ESP32-S3, built-in BMI270 6-axis IMU)
 * Stack  : Arduino / M5Unified / arduinoFFT
 *
 * What it does:
 *   Runs the collar's exact processing pipeline on the Cardputer's own IMU and
 *   shows the result live on screen:
 *
 *     BMI270 @ 25 Hz -> 128-sample sliding window -> 8 features
 *       -> decision tree -> REST / RUMINATE / GRAZE / WALK / ACTIVE
 *       -> per-unit EWMA baseline -> estrus-candidate flag
 *
 *   Same lane separation as the collar firmware: a timer callback does nothing
 *   but sample into a ring buffer (ISR lane); loop() does DSP + UI (supervisor
 *   lane).
 *
 * Demo script (how to perform each behaviour by hand):
 *   REST      — device flat on the table, hands off
 *   RUMINATE  — hold it and rock rhythmically, ~1–1.5 times per second
 *               (mimics the jaw-motion periodicity of chewing)
 *   GRAZE     — slow, irregular tilting and drifting (head-down foraging)
 *   WALK      — carry it while walking; your gait lands in the 1.8–3.5 Hz band
 *   ACTIVE    — shake vigorously; sustained shaking ~10 s trips the
 *               heat-candidate flag (watch the ALERT banner)
 *
 * Keys:
 *   D — toggle detail view (features + band energies)
 *   R — reset the EWMA baseline (re-prime)
 *   B — send a fake "BEEP" cue (buzzer demo, shock-free training talking point)
 * -----------------------------------------------------------------------------
 */
#include <M5Cardputer.h>   /* wraps M5Unified; provides Keyboard on the Adv (TCA8418) */
#include <arduinoFFT.h>
#include <math.h>
#include "demo_config.h"
#include "health_watch.h"

/* ============================== ISR LANE ================================== */
/* Ring buffer filled by a hardware timer at ACCEL_ODR_HZ. Keep it minimal.    */

struct Sample { float x, y, z; };

static volatile uint32_t g_wr = 0;              /* total samples written       */
static Sample g_ring[WINDOW_LEN];               /* ring, len == window         */
static hw_timer_t *g_timer = nullptr;
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool g_tick = false;

void IRAM_ATTR onSampleTick() {
    portENTER_CRITICAL_ISR(&g_mux);
    g_tick = true;                              /* sampling itself happens in  */
    portEXIT_CRITICAL_ISR(&g_mux);              /* loop(): M5.Imu is not       */
}                                               /* ISR-safe over I2C           */

static void pump_sampler() {
    /* Called every loop() pass; takes at most one sample per timer tick.      */
    bool due;
    portENTER_CRITICAL(&g_mux);
    due = g_tick; g_tick = false;
    portEXIT_CRITICAL(&g_mux);
    if (!due) return;

    float ax, ay, az;
    if (M5.Imu.getAccel(&ax, &ay, &az)) {
        g_ring[g_wr % WINDOW_LEN] = { ax, ay, az };
        g_wr++;
    }
}

/* =========================== SUPERVISOR LANE =============================== */

static float vReal[FFT_LEN], vImag[FFT_LEN];
static ArduinoFFT<float> FFT(vReal, vImag, FFT_LEN, (float)ACCEL_ODR_HZ);

/* --- feature extraction: identical math to the collar ---------------------- */
static void extract_features(const Sample *win, float *feat) {
    float mx = 0, my = 0, mz = 0;
    for (uint16_t i = 0; i < WINDOW_LEN; i++) { mx += win[i].x; my += win[i].y; mz += win[i].z; }
    mx /= WINDOW_LEN; my /= WINDOW_LEN; mz /= WINDOW_LEN;

    float vx = 0, vy = 0, vz = 0, sma = 0;
    static float mag[WINDOW_LEN];
    for (uint16_t i = 0; i < WINDOW_LEN; i++) {
        float dx = win[i].x - mx, dy = win[i].y - my, dz = win[i].z - mz;
        vx += dx * dx; vy += dy * dy; vz += dz * dz;
        /* SMA on DC-removed axes: raw axes carry the 1 g gravity bias, which
         * pins SMA >= ~1.0 at rest and locks the tree in ACTIVE (field-found
         * bug, first hardware run). */
        sma += fabsf(dx) + fabsf(dy) + fabsf(dz);
        mag[i] = sqrtf(win[i].x * win[i].x + win[i].y * win[i].y + win[i].z * win[i].z);
    }
    feat[0] = sqrtf(vx / WINDOW_LEN);
    feat[1] = sqrtf(vy / WINDOW_LEN);
    feat[2] = sqrtf(vz / WINDOW_LEN);
    feat[3] = sma / WINDOW_LEN;

    /* magnitude ZCR (DC-removed) */
    float mm = 0; for (uint16_t i = 0; i < WINDOW_LEN; i++) mm += mag[i];
    mm /= WINDOW_LEN;
    uint16_t zc = 0;
    for (uint16_t i = 1; i < WINDOW_LEN; i++)
        if ((mag[i] - mm) * (mag[i - 1] - mm) < 0.0f) zc++;
    feat[4] = (float)zc / WINDOW_LEN;

    /* spectrum of DC-removed magnitude */
    for (uint16_t i = 0; i < WINDOW_LEN; i++) { vReal[i] = mag[i] - mm; vImag[i] = 0.0f; }
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();

    const float bin_hz = (float)ACCEL_ODR_HZ / FFT_LEN;
    auto band_e = [&](float lo, float hi) {
        uint16_t klo = (uint16_t)(lo / bin_hz), khi = (uint16_t)(hi / bin_hz);
        float e = 0;
        for (uint16_t k = max<uint16_t>(klo, 1); k <= khi && k < SPEC_BINS; k++)
            e += vReal[k] * vReal[k];
        return e;
    };
    feat[5] = band_e(BAND_RUMIN_LO_HZ, BAND_RUMIN_HI_HZ);
    feat[6] = band_e(BAND_GAIT_LO_HZ,  BAND_GAIT_HI_HZ);

    float peak = 0; uint16_t pk = 1;
    for (uint16_t k = 1; k < SPEC_BINS; k++)
        if (vReal[k] > peak) { peak = vReal[k]; pk = k; }
    feat[7] = pk * bin_hz;
}

/* --- classifier: same tree structure as the collar ------------------------- */
static activity_t classify(const float *f) {
    if (f[3] < TH_REST_SMA)      return ACT_REST;
    if (f[3] > TH_ACTIVE_SMA)    return ACT_ACTIVE;
    if (f[6] > TH_WALK_GAIT_E)   return ACT_WALK;
    if (f[5] > TH_RUMIN_BAND_E)  return ACT_RUMINATE;
    return ACT_GRAZE;
}

/* --- estrus-candidate pre-flag: same EWMA mechanism as the collar ---------- */
struct Baseline {
    float mu = 0, var = 0.01f;
    bool primed = false;
    uint16_t run = 0;
    bool flagged = false;
    void update(float idx) {
        if (!primed) { mu = idx; var = 0.01f; primed = true; return; }
        float d = idx - mu;
        mu  += ESTRUS_EWMA_ALPHA * d;
        var  = (1.0f - ESTRUS_EWMA_ALPHA) * (var + ESTRUS_EWMA_ALPHA * d * d);
        float th = mu + ESTRUS_SIGMA_K * sqrtf(var);
        if (idx > th) { if (++run >= ESTRUS_SUSTAIN) { flagged = true; run = 0; } }
        else run = 0;
    }
    void reset() { primed = false; run = 0; flagged = false; }
};
static Baseline g_base;

/* ================================= UI ====================================== */

static const uint16_t ACT_COLOR[ACT_COUNT] = {
    0x7BEF /* rest: grey  */, 0x07E8 /* ruminate: green */,
    0xFE60 /* graze: amber*/, 0x04FF /* walk: blue      */,
    0xF981 /* active: red */
};

static uint8_t g_view = 0;              /* 0 main  1 features  2 health        */
static uint32_t g_epoch = 0;
static health_watch_t g_hw;

/* Rolling time budget for the on-screen bar (last block, live). */
static void draw_health(void) {
    auto &d = M5.Display;
    const char *names[5] = { "rest", "rumin", "graze", "walk", "actv" };
    const uint16_t cols[5] = { 0x7BEF, 0x07E8, 0xFE60, 0x04FF, 0xF981 };

    d.setTextDatum(top_left);
    d.setTextSize(1);
    d.setTextColor(TFT_DARKGREY, TFT_BLACK);
    d.setCursor(2, 30);
    d.printf("time budget  block %lu/%u\n",
             (unsigned long)g_hw.epochs, (unsigned)HW_EPOCHS_PER_BLOCK);

    uint16_t tot = g_hw.epochs ? g_hw.epochs : 1;
    int y = 42;
    for (int i = 0; i < 5; i++) {
        int w = (int)(140.0f * g_hw.cnt[i] / tot);
        d.fillRect(40, y, w, 6, cols[i]);
        d.drawRect(40, y, 140, 6, 0x2104);
        d.setCursor(2, y);
        d.setTextColor(cols[i], TFT_BLACK);
        d.print(names[i]);
        y += 9;
    }

    d.setCursor(2, 92);
    d.setTextColor(TFT_WHITE, TFT_BLACK);
    d.printf("z rumin %+.1f  z activ %+.1f\n", g_hw.z[0], g_hw.z[1]);
    d.printf("blocks %lu  run h%u/e%u\n", (unsigned long)g_hw.blocks,
             g_hw.run_health, g_hw.run_heat);
}

static void draw(activity_t act, const float *f) {
    auto &d = M5.Display;
    d.startWrite();
    d.fillScreen(TFT_BLACK);

    d.setTextDatum(top_left);
    d.setTextColor(TFT_DARKGREY, TFT_BLACK);
    d.setTextSize(1);
    d.setCursor(2, 2);
    d.printf("COWLLAR ep:%lu %s %s", (unsigned long)g_epoch,
             g_base.primed ? "base:ok" : "base:priming",
             HW_STATE_NAMES[g_hw.state]);

    /* main label */
    d.setTextDatum(middle_center);
    d.setTextColor(ACT_COLOR[act], TFT_BLACK);
    d.setTextSize(g_view ? 2 : 3);
    d.drawString(ACT_NAMES[act], d.width() / 2, g_view ? 16 : 56);

    /* alert banner. Health outranks heat: a sick cow is the more urgent call,
     * and the two conditions are mutually exclusive by construction anyway.   */
    if (g_hw.state == HW_HEALTH_WATCH) {
        uint16_t bg = 0xFB40;
        d.fillRect(0, d.height() - 16, d.width(), 16, bg);
        d.setTextColor(TFT_WHITE, bg);
        d.setTextSize(1);
        d.drawString("! HEALTH WATCH - rumination down, go look at her",
                     d.width() / 2, d.height() - 8);
    } else if (g_base.flagged || g_hw.state == HW_HEAT_WATCH) {
        d.fillRect(0, d.height() - 16, d.width(), 16, 0xF981);
        d.setTextColor(TFT_WHITE, 0xF981);
        d.setTextSize(1);
        d.drawString("! HEAT CANDIDATE -> would send LoRa event", d.width() / 2, d.height() - 8);
    }

    if (g_view == 2) {
        draw_health();
        d.endWrite();
        return;
    }

    if (g_view == 1) {
        d.setTextDatum(top_left);
        d.setTextSize(1);
        d.setTextColor(TFT_WHITE, TFT_BLACK);
        d.setCursor(2, 34);
        d.printf("SMA %.3f  ZCR %.2f  f0 %.1fHz\n", f[3], f[4], f[7]);
        d.printf("std x%.3f y%.3f z%.3f\n", f[0], f[1], f[2]);
        d.printf("E_rumin %.2f  E_gait %.2f\n", f[5], f[6]);
        /* band-energy bars */
        int w1 = min(120, (int)(f[5] * 60)), w2 = min(120, (int)(f[6] * 60));
        d.fillRect(2, 104, w1, 6, 0x07E8);
        d.fillRect(2, 114, w2, 6, 0x04FF);
        d.setCursor(126, 102); d.print("rumin");
        d.setCursor(126, 112); d.print("gait");
    } else {
        d.setTextDatum(top_left);
        d.setTextSize(1);
        d.setTextColor(TFT_DARKGREY, TFT_BLACK);
        d.setCursor(2, d.height() - 30);
        d.print("D:features H:health R:reset B:beep");
    }
    d.endWrite();
}

/* =============================== SETUP/LOOP ================================ */

void setup() {
    auto cfg = M5.config();
    M5Cardputer.begin(cfg, true);   /* true = init keyboard */
    M5.Display.setRotation(1);
    M5.Display.setBrightness(180);

    if (M5.Imu.getType() == m5::imu_none) {
        M5.Display.setTextColor(TFT_RED);
        M5.Display.drawString("IMU not found - update M5Unified", 4, 40);
        while (true) delay(1000);
    }

    /* 25 Hz sample timer: 80 MHz APB / 80 = 1 MHz tick; alarm every 40,000 µs */
    g_timer = timerBegin(0, 80, true);
    timerAttachInterrupt(g_timer, &onSampleTick, true);
    timerAlarmWrite(g_timer, 1000000UL / ACCEL_ODR_HZ, true);
    timerAlarmEnable(g_timer);

    hw_init(&g_hw);
    M5.Display.drawString("COWLLAR demo - collecting...", 4, 40);
}

void loop() {
    M5Cardputer.update();
    pump_sampler();

    /* keyboard (Adv: TCA8418 over I2C, wrapped by M5Cardputer.Keyboard) */
    if (M5Cardputer.Keyboard.isChange() && M5Cardputer.Keyboard.isPressed()) {
        if (M5Cardputer.Keyboard.isKeyPressed('d')) g_view = (g_view == 1) ? 0 : 1;
        if (M5Cardputer.Keyboard.isKeyPressed('h')) g_view = (g_view == 2) ? 0 : 2;
        if (M5Cardputer.Keyboard.isKeyPressed('r')) { g_base.reset(); hw_init(&g_hw); }
        if (M5Cardputer.Keyboard.isKeyPressed('b')) M5.Speaker.tone(2000, 120);
    }

    /* process every WINDOW_HOP new samples once the ring is full */
    static uint32_t last_proc = 0;
    if (g_wr >= WINDOW_LEN && (g_wr - last_proc) >= WINDOW_HOP) {
        last_proc = g_wr;

        /* snapshot ring in time order */
        static Sample win[WINDOW_LEN];
        uint32_t start = g_wr - WINDOW_LEN;
        for (uint16_t i = 0; i < WINDOW_LEN; i++)
            win[i] = g_ring[(start + i) % WINDOW_LEN];

        static float feat[N_FEATURES];
        extract_features(win, feat);
        activity_t act = classify(feat);

        float idx = (act == ACT_ACTIVE || act == ACT_WALK) ? 1.0f : 0.0f;
        g_base.update(idx);
        g_epoch++;

        /* Health watch runs on the same classifier output, one level up: it
         * cares about how the minutes were SPENT, not what is happening right
         * now. On the collar a block is an hour; here it is ~19 s.           */
        hw_push(&g_hw, (uint8_t)act);
        if (hw_block_ready(&g_hw)) {
            /* production: slot = hour of day from the RTC. Demo has one slot. */
            hw_close_block(&g_hw, 0);
        }

        draw(act, feat);

        /* clear the alert after a few epochs so the demo can repeat */
        static uint8_t alert_age = 0;
        if (g_base.flagged && ++alert_age > 6) { g_base.flagged = false; alert_age = 0; }
    }
}
