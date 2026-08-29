/*
 * main.cpp — COWLLAR field data logger (dual-IMU revision)
 * -----------------------------------------------------------------------------
 * Target : M5Stack Cardputer Adv (ESP32-S3, BMI270 internal, microSD)
 *          + Grove LSM6DS3TR-C on the HY2.0-4P port  <-- the collar's sensor
 *
 * PURPOSE
 *   Classifies nothing. Records raw IMU from BOTH sensors at 50 Hz together with
 *   a behaviour label a human presses while watching the cow. Offline,
 *   tools/tune_thresholds.py recomputes the collar's 8 features from this and
 *   fits the decision-tree constants.
 *
 *   The demo firmware's thresholds were fitted to a hand waving a Cardputer.
 *   They are honest placeholders. This firmware is how they stop being that.
 *
 * MOUNTING
 *   Tape the little Grove sensor exactly where the collar will sit and note its
 *   orientation. The Cardputer itself can ride anywhere convenient on the strap
 *   — that is the whole point of putting the sensor on a cable.
 *
 * KEYS
 *   1 LYING  2 STANDING  3 RUMINATE  4 GRAZE  5 WALK  6 DRINK  7 OTHER
 *   0 UNKNOWN (press the instant you are unsure)
 *   M  sync marker — press while the camera sees you, at start AND end
 *   N  new session   S  stop session   I  I2C rescan
 * -----------------------------------------------------------------------------
 */
#include <M5Cardputer.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "logger_config.h"

/* ============================== SAMPLER LANE =============================== */

static hw_timer_t   *g_timer = nullptr;
static portMUX_TYPE  g_mux   = portMUX_INITIALIZER_UNLOCKED;
static volatile bool g_tick  = false;

void IRAM_ATTR onSampleTick() {
    portENTER_CRITICAL_ISR(&g_mux);
    g_tick = true;                  /* both IMUs sit on I2C, which is not      */
    portEXIT_CRITICAL_ISR(&g_mux);  /* ISR-safe: the reads happen in loop()    */
}

/* ============================ EXTERNAL LSM6DS3 ============================== */

static int      g_extSda = -1, g_extScl = -1;
static uint8_t  g_lsmAddr = 0;
static bool     g_extOk = false;
static uint32_t g_extFails = 0;

static bool lsmWrite(uint8_t reg, uint8_t val) {
    Wire1.beginTransmission(g_lsmAddr);
    Wire1.write(reg); Wire1.write(val);
    return Wire1.endTransmission() == 0;
}

static bool lsmRead(uint8_t reg, uint8_t *buf, size_t n) {
    Wire1.beginTransmission(g_lsmAddr);
    Wire1.write(reg);
    if (Wire1.endTransmission(false) != 0) return false;
    if (Wire1.requestFrom((int)g_lsmAddr, (int)n) != (int)n) return false;
    for (size_t i = 0; i < n; i++) buf[i] = Wire1.read();
    return true;
}

/* Scan candidate Grove pin pairs and both I2C addresses. Prints everything it
 * finds — the scan output is the single most useful debugging tool here, since
 * "no sensor" and "wrong pins" and "wrong address" all look identical from the
 * outside. */
static bool extBegin(bool verbose) {
    struct Pair { int sda, scl; };
    const Pair cands[] = EXT_I2C_CANDIDATES;

    for (const Pair &p : cands) {
        Wire1.end();
        delay(5);
        if (!Wire1.begin(p.sda, p.scl, EXT_I2C_HZ)) continue;

        for (uint8_t addr : { (uint8_t)LSM_ADDR_A, (uint8_t)LSM_ADDR_B }) {
            g_lsmAddr = addr;
            uint8_t who = 0;
            if (!lsmRead(LSM_WHO_AM_I, &who, 1)) continue;
            if (who != LSM_ID_TRC && who != LSM_ID_DS3) {
                if (verbose)
                    Serial.printf("# SDA%d/SCL%d addr 0x%02x: WHO_AM_I=0x%02x (not LSM6DS3)\n",
                                  p.sda, p.scl, addr, who);
                continue;
            }
            lsmWrite(LSM_CTRL3_C, LSM_CTRL3_C_VAL);
            lsmWrite(LSM_CTRL1_XL, LSM_CTRL1_XL_VAL);
            lsmWrite(LSM_CTRL2_G,  LSM_CTRL2_G_VAL);
            g_extSda = p.sda; g_extScl = p.scl;
            Serial.printf("# LSM6DS3 found: SDA=%d SCL=%d addr=0x%02x WHO=0x%02x\n",
                          p.sda, p.scl, addr, who);
            return true;
        }
    }
    Serial.println("# no LSM6DS3 found on any candidate Grove pin pair");
    return false;
}

static bool extRead(float *ax, float *ay, float *az,
                    float *gx, float *gy, float *gz) {
    uint8_t b[12];
    /* CTRL3_C IF_INC is set, so gyro (0x22..0x27) and accel (0x28..0x2D) come
     * back in one contiguous burst — one transaction instead of two, which
     * halves the skew between the gyro and accel of the same sample. */
    if (!lsmRead(LSM_OUTX_L_G, b, 12)) { g_extFails++; return false; }
    auto i16 = [&](int i) { return (int16_t)((uint16_t)b[i + 1] << 8 | b[i]); };
    const float ga = LSM_ACC_MG_PER_LSB / 1000.0f;      /* -> g    */
    const float gg = LSM_GYR_MDPS_PER_LSB / 1000.0f;    /* -> dps  */
    *gx = i16(0) * gg; *gy = i16(2) * gg; *gz = i16(4)  * gg;
    *ax = i16(6) * ga; *ay = i16(8) * ga; *az = i16(10) * ga;
    return true;
}

/* ============================== LOGGER STATE =============================== */

static uint8_t  g_label      = LBL_UNKNOWN;
static bool     g_marker     = false;
static bool     g_running    = false;
static uint32_t g_samples    = 0;
static uint32_t g_missed     = 0;
static uint32_t g_last_us    = 0;
static uint32_t g_session_ms = 0;
static uint16_t g_dt_max     = 0;
static uint32_t g_label_secs[LBL_COUNT] = {0};

static File     g_file;
static bool     g_sd_ok      = false;
static int      g_sd_cs      = -1;
static char     g_fname[24]  = "-";
static bool     g_serial_mode = false;

static uint8_t  g_buf[WRITE_BLOCK_BYTES];
static size_t   g_buf_len = 0;
static uint32_t g_last_flush = 0;

/* Live magnitudes, for the on-screen sanity check. If the two bars do not move
 * together, something is wrong with a cable or a mount and you want to know NOW,
 * not during analysis three days later. */
static float g_iMag = 0, g_eMag = 0;

/* Edit per animal, or write it in the notebook and rename the files after. */
static char g_cow[16]   = "COW-01";
static char g_mount[16] = "NECK-TOP";

/* ============================== SD HANDLING ================================ */

static bool sdTryMount() {
    const int cands[] = SD_CS_CANDIDATES;
    SPI.begin(SD_SPI_SCK, SD_SPI_MISO, SD_SPI_MOSI, cands[0]);
    for (unsigned i = 0; i < sizeof(cands) / sizeof(cands[0]); i++) {
        if (SD.begin(cands[i], SPI, 20000000)) { g_sd_cs = cands[i]; return true; }
        SD.end();
        delay(20);
    }
    return false;
}

static void bufWrite(const void *p, size_t n) {
    if (g_buf_len + n > WRITE_BLOCK_BYTES) {
        g_file.write(g_buf, g_buf_len);
        g_buf_len = 0;
    }
    memcpy(g_buf + g_buf_len, p, n);
    g_buf_len += n;
}

static void bufFlush(bool sync) {
    if (g_buf_len) { g_file.write(g_buf, g_buf_len); g_buf_len = 0; }
    if (sync) g_file.flush();
}

static uint32_t rtcEpoch() {
    auto dt = M5.Rtc.getDateTime();
    if (dt.date.year < 2024) return 0;      /* unset clock: say 0, do not lie   */
    struct tm t = {};
    t.tm_year = dt.date.year - 1900; t.tm_mon = dt.date.month - 1;
    t.tm_mday = dt.date.date;
    t.tm_hour = dt.time.hours; t.tm_min = dt.time.minutes; t.tm_sec = dt.time.seconds;
    return (uint32_t)mktime(&t);
}

static bool sessionOpen() {
    if (!g_sd_ok) { g_serial_mode = true; g_running = true; g_session_ms = millis(); return true; }

    uint32_t id = 1;
    while (id < 10000) {
        snprintf(g_fname, sizeof(g_fname), "/COW%04lu.CLB", (unsigned long)id);
        if (!SD.exists(g_fname)) break;
        id++;
    }
    g_file = SD.open(g_fname, FILE_WRITE);
    if (!g_file) { g_serial_mode = true; g_running = true; g_session_ms = millis(); return false; }

    cowlog_hdr_t h = {};
    memcpy(h.magic, COWLOG_MAGIC, 8);
    h.odr_hz          = LOG_ODR_HZ;
    h.rec_bytes       = sizeof(cowlog_rec_t);
    h.acc_lsb_per_g   = ACC_LSB_PER_G;
    h.gyr_lsb_per_dps = GYR_LSB_PER_DPS;
    h.session_id      = id;
    h.start_epoch     = rtcEpoch();
    h.start_millis    = millis();
    h.sensors         = (uint8_t)(HDR_F_HAS_INT | (g_extOk ? HDR_F_HAS_EXT : 0));
    strncpy(h.int_part, "BMI270",      sizeof(h.int_part) - 1);
    strncpy(h.ext_part, "LSM6DS3TR-C", sizeof(h.ext_part) - 1);
    strncpy(h.mount, g_mount, sizeof(h.mount) - 1);
    strncpy(h.cow,   g_cow,   sizeof(h.cow) - 1);
    g_file.write((const uint8_t *)&h, sizeof(h));
    g_file.flush();

    g_samples = 0; g_missed = 0; g_dt_max = 0; g_last_us = 0; g_extFails = 0;
    memset((void *)g_label_secs, 0, sizeof(g_label_secs));
    g_session_ms = millis();
    g_running = true;
    return true;
}

static void sessionClose() {
    if (!g_running) return;
    g_running = false;
    if (!g_serial_mode && g_file) { bufFlush(true); g_file.close(); }
}

/* ============================== SAMPLING =================================== */

static inline int16_t q(float v, float scale) {
    long r = lroundf(v * scale);
    if (r >  32767) r =  32767;
    if (r < -32768) r = -32768;
    return (int16_t)r;
}

static void pumpSampler() {
    bool due;
    portENTER_CRITICAL(&g_mux);
    due = g_tick; g_tick = false;
    portEXIT_CRITICAL(&g_mux);
    if (!due || !g_running) return;

    float iax = 0, iay = 0, iaz = 0;
    bool iok = M5.Imu.getAccel(&iax, &iay, &iaz);

    float eax = 0, eay = 0, eaz = 0, egx = 0, egy = 0, egz = 0;
    bool eok = g_extOk && extRead(&eax, &eay, &eaz, &egx, &egy, &egz);

    if (!iok && !eok) return;       /* nothing to record; do not fabricate      */

    uint32_t now = micros();
    uint32_t dt_us = g_last_us ? (now - g_last_us) : 0;
    uint32_t dt_ms32 = dt_us / 1000u;
    uint16_t dt = (uint16_t)(dt_ms32 > 65535u ? 65535u : dt_ms32);
    g_last_us = now;
    if (dt > g_dt_max) g_dt_max = dt;
    if (dt > (2000u / LOG_ODR_HZ)) g_missed++;

    cowlog_rec_t r;
    r.iax = q(iax, ACC_LSB_PER_G); r.iay = q(iay, ACC_LSB_PER_G); r.iaz = q(iaz, ACC_LSB_PER_G);
    r.eax = q(eax, ACC_LSB_PER_G); r.eay = q(eay, ACC_LSB_PER_G); r.eaz = q(eaz, ACC_LSB_PER_G);
    r.egx = q(egx, GYR_LSB_PER_DPS); r.egy = q(egy, GYR_LSB_PER_DPS); r.egz = q(egz, GYR_LSB_PER_DPS);
    r.label = g_label;
    r.flags = (uint8_t)((g_marker ? FLAG_MARKER : 0) | (eok ? 0 : FLAG_EXT_FAIL));
    r.dt_ms = dt;
    r.seq   = (uint16_t)(g_samples & 0xFFFF);
    g_marker = false;

    g_iMag = sqrtf(iax*iax + iay*iay + iaz*iaz);
    g_eMag = sqrtf(eax*eax + eay*eay + eaz*eaz);

    if (g_serial_mode) {
        Serial.printf("%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%u\n",
                      (unsigned long)g_samples, r.iax, r.iay, r.iaz,
                      r.eax, r.eay, r.eaz, r.egx, r.egy, r.egz,
                      r.label, r.flags, r.dt_ms);
    } else {
        bufWrite(&r, sizeof(r));
    }

    g_samples++;
    if (g_samples % LOG_ODR_HZ == 0 && g_label < LBL_COUNT) g_label_secs[g_label]++;

    if (!g_serial_mode && millis() - g_last_flush > FLUSH_EVERY_MS) {
        bufFlush(true);
        g_last_flush = millis();
    }
}

/* ================================= UI ====================================== */

static const uint16_t LBL_COLOR[LBL_COUNT] = {
    0x7BEF, 0x421F, 0x05FF, 0x07E8, 0xFE60, 0x04FF, 0x07FF, 0xF981
};

static void draw() {
    auto &d = M5.Display;
    d.startWrite();
    d.fillScreen(TFT_BLACK);
    d.setTextSize(1);

    d.setTextDatum(top_left);
    d.setTextColor(TFT_DARKGREY, TFT_BLACK);
    d.setCursor(2, 2);
    d.printf("%s %s %s", g_cow, g_mount,
             g_serial_mode ? "SERIAL" : (g_sd_ok ? g_fname + 1 : "NO-SD"));

    d.setTextDatum(middle_center);
    d.setTextColor(LBL_COLOR[g_label], TFT_BLACK);
    d.setTextSize(3);
    d.drawString(LBL_NAMES[g_label], d.width() / 2, 30);

    uint32_t sec = g_running ? (millis() - g_session_ms) / 1000 : 0;
    d.setTextDatum(top_left);
    d.setTextSize(1);
    d.setTextColor(g_running ? TFT_WHITE : 0xF981, TFT_BLACK);
    d.setCursor(2, 50);
    d.printf("%s %02lu:%02lu n=%lu\n", g_running ? "REC" : "STOP",
             (unsigned long)(sec / 60), (unsigned long)(sec % 60),
             (unsigned long)g_samples);

    /* Two live magnitude bars. They should move TOGETHER. If the EXT bar is
     * flat, or wanders on its own, stop and fix the cable or the mount before
     * you record another minute. */
    auto bar = [&](int y, const char *tag, float mag, uint16_t col, bool ok) {
        d.setCursor(2, y);
        d.setTextColor(ok ? col : 0xF981, TFT_BLACK);
        d.print(tag);
        int w = (int)(fminf(mag, 2.0f) * 60.0f);
        d.drawRect(30, y, 122, 7, 0x2104);
        d.fillRect(31, y + 1, min(120, w), 5, ok ? col : 0xF981);
        d.setCursor(156, y);
        d.printf("%.2f", mag);
    };
    bar(62, "INT", g_iMag, 0x05FF, true);
    bar(72, "EXT", g_eMag, 0x07E8, g_extOk);

    d.setTextColor(g_missed || g_extFails ? 0xFE60 : TFT_DARKGREY, TFT_BLACK);
    d.setCursor(2, 84);
    d.printf("late%lu dt%ums xf%lu bat%d%%",
             (unsigned long)g_missed, g_dt_max, (unsigned long)g_extFails,
             M5.Power.getBatteryLevel());

    /* Per-label seconds: your live coverage check. If GRAZE has 900 s and
     * RUMINATE has 20 s, you do not have a dataset, you have a graze recording. */
    int y = 96;
    for (uint8_t i = 1; i < LBL_COUNT; i++) {
        if (i == 5) y = 96;
        int x = (i < 5) ? 2 : 120;
        d.setCursor(x, y);
        d.setTextColor(g_label_secs[i] ? LBL_COLOR[i] : 0x39E7, TFT_BLACK);
        d.printf("%.4s %lus", LBL_NAMES[i], (unsigned long)g_label_secs[i]);
        y += 10;
    }
    d.endWrite();
}

/* =============================== SETUP/LOOP ================================ */

void setup() {
    auto cfg = M5.config();
    M5Cardputer.begin(cfg, true);
    M5.Display.setRotation(1);
    M5.Display.setBrightness(140);
    M5.Display.setTextSize(1);
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.setCursor(2, 2);

    Serial.println("# COWLLAR dual-IMU logger");

    if (M5.Imu.getType() == m5::imu_none)
        M5.Display.printf("INT: none (update M5Unified)\n");
    else
        M5.Display.printf("INT: BMI270 ok\n");

    g_extOk = extBegin(true);
    M5.Display.setTextColor(g_extOk ? TFT_GREEN : 0xFE60, TFT_BLACK);
    if (g_extOk) M5.Display.printf("EXT: LSM6DS3 SDA%d SCL%d\n", g_extSda, g_extScl);
    else         M5.Display.printf("EXT: NOT FOUND - check Grove cable\n");

    g_sd_ok = sdTryMount();
    M5.Display.setTextColor(g_sd_ok ? TFT_GREEN : 0xFE60, TFT_BLACK);
    M5.Display.printf(g_sd_ok ? "SD : CS=%d\n" : "SD : none, CSV to USB\n", g_sd_cs);

    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.printf("cow=%s mount=%s\n", g_cow, g_mount);
    M5.Display.printf("N=start  I=rescan I2C\n");
    delay(2500);

    g_timer = timerBegin(0, 80, true);                  /* 1 MHz tick           */
    timerAttachInterrupt(g_timer, &onSampleTick, true);
    timerAlarmWrite(g_timer, 1000000UL / LOG_ODR_HZ, true);
    timerAlarmEnable(g_timer);
}

void loop() {
    M5Cardputer.update();
    pumpSampler();

    if (M5Cardputer.Keyboard.isChange() && M5Cardputer.Keyboard.isPressed()) {
        for (uint8_t i = 0; i < LBL_COUNT; i++)
            if (M5Cardputer.Keyboard.isKeyPressed((char)('0' + i))) {
                g_label = i;
                M5.Speaker.tone(1400 + 120 * i, 40);
            }
        if (M5Cardputer.Keyboard.isKeyPressed('m')) {
            g_marker = true;
            M5.Speaker.tone(3000, 200);                 /* audible on the video */
        }
        if (M5Cardputer.Keyboard.isKeyPressed('s')) { sessionClose(); M5.Speaker.tone(600, 300); }
        if (M5Cardputer.Keyboard.isKeyPressed('n')) { sessionClose(); sessionOpen(); M5.Speaker.tone(2400, 120); }
        if (M5Cardputer.Keyboard.isKeyPressed('i')) { g_extOk = extBegin(true); M5.Speaker.tone(1800, 80); }
    }

    static uint32_t lastDraw = 0;
    if (millis() - lastDraw > 250) { lastDraw = millis(); draw(); }
}
