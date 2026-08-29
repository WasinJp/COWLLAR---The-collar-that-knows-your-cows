/*
 * main.cpp — COWLLAR on-cow data-collection tag
 * -----------------------------------------------------------------------------
 * ESP32-S3 N16R8 + LSM6DS3TR-C + USB power bank.
 * Records raw IMU to internal flash. Nothing else.
 *
 * ARCHITECTURE
 *   core 0 : sampler task, high priority, 50 Hz, never blocks
 *   core 1 : writer task, drains the ring buffer to LittleFS
 *   loop() : LED status + USB command handling
 *
 *   The split exists because LittleFS writes stall for tens of milliseconds. A
 *   single-threaded design would drop samples during every flush, and the only
 *   evidence would be a dt_ms spike that nobody looks at until the data is
 *   already useless.
 *
 * OPERATION
 *   Power on  -> records automatically after a 3 s grace period
 *   LED green blink = recording      LED amber = nearly full
 *   LED red solid   = no IMU         LED red fast blink = OVERRUN, losing data
 *   LED blue        = idle / dump mode
 *
 * SYNC PROTOCOL — do not skip this
 *   Hold the tag against the Cardputer and TAP THEM TOGETHER THREE TIMES while
 *   pressing M on the Cardputer. Do it at the start AND at the end of every
 *   session. The taps appear as unmistakable spikes here; merge_labels.py uses
 *   the first pair for the offset and the second pair to VERIFY it.
 * -----------------------------------------------------------------------------
 */
#include <Arduino.h>
#include <Wire.h>
#include <FS.h>
#include <LittleFS.h>
#include <Adafruit_NeoPixel.h>
#include "tag_config.h"

/* ================================ STATE ==================================== */

static Adafruit_NeoPixel led(1, LED_PIN, NEO_GRB + NEO_KHZ800);

static tag_rec_t *g_ring = nullptr;
static uint32_t   g_ringN = 0;
static volatile uint32_t g_head = 0;     /* written by sampler  */
static volatile uint32_t g_tail = 0;     /* written by writer   */

static volatile bool     g_recording = false;
static volatile uint32_t g_samples = 0;
static volatile uint32_t g_overruns = 0;
static volatile uint32_t g_imuFails = 0;
static uint32_t g_written = 0;
static uint32_t g_sessionId = 0;
static char     g_fname[32] = "-";
static File     g_file;
static bool     g_imuOk = false;
static bool     g_fsOk = false;
static uint8_t  g_fullPct = 0;

/* Written on the enclosure in marker, and into every file header. */
static char g_tagId[8]  = "TAG-A";
static char g_cow[16]   = "COW-01";
static char g_mount[16] = "NECK-TOP";

static uint8_t g_lsmAddr = 0;
static int     g_sda = -1, g_scl = -1;

/* ============================== LSM6DS3 ==================================== */

static bool lsmWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(g_lsmAddr);
    Wire.write(reg); Wire.write(val);
    return Wire.endTransmission() == 0;
}

static bool lsmRead(uint8_t reg, uint8_t *b, size_t n) {
    Wire.beginTransmission(g_lsmAddr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)g_lsmAddr, (int)n) != (int)n) return false;
    for (size_t i = 0; i < n; i++) b[i] = Wire.read();
    return true;
}

/* Scan candidate pin pairs and both addresses, reporting everything found.
 * "Wrong pins", "wrong address" and "no sensor" are indistinguishable from the
 * outside, and telling them apart in a field costs an afternoon. */
static bool imuBegin() {
    struct P { int sda, scl; };
    const P cands[] = I2C_CANDIDATES;
    for (const P &p : cands) {
        Wire.end(); delay(5);
        if (!Wire.begin(p.sda, p.scl, I2C_HZ)) continue;
        for (uint8_t a : { (uint8_t)LSM_ADDR_A, (uint8_t)LSM_ADDR_B }) {
            g_lsmAddr = a;
            uint8_t who = 0;
            if (!lsmRead(LSM_WHO_AM_I, &who, 1)) continue;
            if (who != LSM_ID_TRC && who != LSM_ID_DS3) {
                Serial.printf("# SDA%d SCL%d 0x%02x -> WHO=0x%02x (not LSM6DS3)\n",
                              p.sda, p.scl, a, who);
                continue;
            }
            lsmWrite(LSM_CTRL3_C, LSM_CTRL3_C_VAL);
            lsmWrite(LSM_CTRL1_XL, LSM_CTRL1_XL_VAL);
            lsmWrite(LSM_CTRL2_G,  LSM_CTRL2_G_VAL);
            g_sda = p.sda; g_scl = p.scl;
            Serial.printf("# LSM6DS3 SDA=%d SCL=%d addr=0x%02x WHO=0x%02x\n",
                          p.sda, p.scl, a, who);
            return true;
        }
    }
    Serial.println("# LSM6DS3 NOT FOUND on any candidate pin pair");
    return false;
}

/* ============================== SAMPLER (core 0) =========================== */

static inline int16_t q(float v, float s) {
    long r = lroundf(v * s);
    return (int16_t)(r > 32767 ? 32767 : (r < -32768 ? -32768 : r));
}

static void samplerTask(void *) {
    TickType_t last = xTaskGetTickCount();
    uint32_t prev_ms = 0;
    const float ga = LSM_ACC_MG_PER_LSB / 1000.0f;
    const float gg = LSM_GYR_MDPS_PER_LSB / 1000.0f;

    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(TAG_PERIOD_MS));
        if (!g_recording) { prev_ms = 0; continue; }

        uint8_t b[12];
        bool ok = lsmRead(LSM_OUTX_L_G, b, 12);   /* IF_INC: gyro+accel in one
                                                     burst, so both belong to
                                                     the same instant           */
        if (!ok) g_imuFails++;

        auto i16 = [&](int i) { return (int16_t)((uint16_t)b[i+1] << 8 | b[i]); };

        uint32_t now = millis();
        uint8_t dt = prev_ms ? (uint8_t)min<uint32_t>(now - prev_ms, 255) : 0;
        prev_ms = now;

        uint32_t h = g_head, t = g_tail;
        if (((h + 1) % g_ringN) == t) {           /* ring full: the writer is
                                                     behind. Drop and SAY SO —
                                                     a silent gap is far worse
                                                     than a flagged one.        */
            g_overruns++;
            continue;
        }
        tag_rec_t *r = &g_ring[h];
        if (ok) {
            r->gx = q(i16(0) * gg, GYR_LSB_PER_DPS);
            r->gy = q(i16(2) * gg, GYR_LSB_PER_DPS);
            r->gz = q(i16(4) * gg, GYR_LSB_PER_DPS);
            r->ax = q(i16(6) * ga, ACC_LSB_PER_G);
            r->ay = q(i16(8) * ga, ACC_LSB_PER_G);
            r->az = q(i16(10) * ga, ACC_LSB_PER_G);
        } else {
            r->ax = r->ay = r->az = r->gx = r->gy = r->gz = 0;
        }
        r->flags = (uint8_t)((ok ? 0 : TF_IMU_FAIL) |
                             (g_overruns ? TF_OVERRUN : 0));
        r->dt_ms = dt;
        r->seq   = (uint16_t)(g_samples & 0xFFFF);
        g_head   = (h + 1) % g_ringN;
        g_samples++;
    }
}

/* ============================== WRITER (core 1) ============================ */

static bool sessionOpen() {
    if (!g_fsOk) return false;
    uint32_t id = 1;
    while (id < 10000) {
        snprintf(g_fname, sizeof(g_fname), REC_PREFIX "%04lu.bin", (unsigned long)id);
        if (!LittleFS.exists(g_fname)) break;
        id++;
    }
    /* create=true is mandatory on LittleFS: FS::open() defaults it to false and
     * refuses to open a file that does not exist yet. Without this the tag
     * recorded nothing at all. */
    g_file = LittleFS.open(g_fname, FILE_WRITE, true);
    if (!g_file) {
        Serial.printf("# CANNOT CREATE %s\n", g_fname);
        return false;
    }

    tag_hdr_t h = {};
    memcpy(h.magic, TAG_MAGIC, 8);
    h.odr_hz          = TAG_ODR_HZ;
    h.rec_bytes       = sizeof(tag_rec_t);
    h.acc_lsb_per_g   = ACC_LSB_PER_G;
    h.gyr_lsb_per_dps = GYR_LSB_PER_DPS;
    h.session_id      = id;
    h.boot_millis     = millis();
    strncpy(h.tag_id, g_tagId, sizeof(h.tag_id) - 1);
    strncpy(h.cow,    g_cow,   sizeof(h.cow) - 1);
    strncpy(h.mount,  g_mount, sizeof(h.mount) - 1);
    g_file.write((const uint8_t *)&h, sizeof(h));
    g_file.flush();

    g_sessionId = id;
    g_written = 0;
    g_head = g_tail = 0;
    g_samples = 0; g_overruns = 0; g_imuFails = 0;
    Serial.printf("# recording -> %s\n", g_fname);
    return true;
}

static void sessionClose() {
    g_recording = false;
    delay(2 * TAG_PERIOD_MS);
    if (g_file) { g_file.flush(); g_file.close(); }
    Serial.printf("# stopped. %lu samples, %lu overruns, %lu imu fails\n",
                  (unsigned long)g_samples, (unsigned long)g_overruns,
                  (unsigned long)g_imuFails);
}

static void writerTask(void *) {
    static uint8_t chunk[WRITE_CHUNK_BYTES];
    uint32_t lastFlush = millis();

    for (;;) {
        if (!g_recording || !g_file) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }

        size_t n = 0;
        while (n + sizeof(tag_rec_t) <= WRITE_CHUNK_BYTES && g_tail != g_head) {
            memcpy(chunk + n, &g_ring[g_tail], sizeof(tag_rec_t));
            n += sizeof(tag_rec_t);
            g_tail = (g_tail + 1) % g_ringN;
        }
        if (n) {
            g_file.write(chunk, n);
            g_written += n;
        }
        if (millis() - lastFlush > FLUSH_EVERY_MS) {
            g_file.flush();
            lastFlush = millis();
            size_t tot = LittleFS.totalBytes(), used = LittleFS.usedBytes();
            g_fullPct = tot ? (uint8_t)(used * 100 / tot) : 0;
            if (g_fullPct >= FULL_STOP_PCT) {
                Serial.println("# storage full — stopping cleanly");
                sessionClose();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(n ? 5 : 40));
    }
}

/* ================================ LED ====================================== */

static void setLed(uint8_t r, uint8_t g, uint8_t b) {
    led.setPixelColor(0, led.Color(r, g, b));
    led.show();
}

static void updateLed() {
    static uint32_t last = 0;
    static bool on = false;
    uint32_t period = 1000;
    uint8_t r = 0, g = 0, b = 0;

    if (!g_imuOk)            { setLed(60, 0, 0); return; }          /* solid red */
    if (!g_recording)        { setLed(0, 0, 40); return; }          /* blue idle */
    if (g_overruns)          { period = 120; r = 60; }              /* fast red  */
    else if (g_fullPct > 90) { period = 600; r = 50; g = 30; }      /* amber     */
    else                     { period = 1000; g = 50; }             /* green     */

    if (millis() - last > (on ? period / 8 : period)) {
        last = millis();
        on = !on;
        if (on) setLed(r, g, b); else setLed(0, 0, 0);
    }
}

/* ============================ USB COMMANDS ================================= */
/*
 *   ?         status
 *   L         list files
 *   D <name>  dump: "BEGIN <name> <size>", raw bytes, "END <sum32>"
 *   S         stop recording (dump_tag.py sends this first)
 *   G         go: start a new session
 *   X         erase all recordings
 */
static void handleCommand(String cmd) {
    cmd.trim();
    if (!cmd.length()) return;
    char c = toupper(cmd[0]);

    if (c == '?') {
        Serial.printf("STATUS tag=%s cow=%s mount=%s rec=%d file=%s "
                      "samples=%lu overrun=%lu imufail=%lu full=%u%% "
                      "imu=%d sda=%d scl=%d\n",
                      g_tagId, g_cow, g_mount, g_recording ? 1 : 0, g_fname,
                      (unsigned long)g_samples, (unsigned long)g_overruns,
                      (unsigned long)g_imuFails, g_fullPct,
                      g_imuOk ? 1 : 0, g_sda, g_scl);
    } else if (c == 'L') {
        File d = LittleFS.open("/");
        for (File f = d.openNextFile(); f; f = d.openNextFile())
            Serial.printf("FILE %s %u\n", f.name(), (unsigned)f.size());
        Serial.println("ENDLIST");
    } else if (c == 'S') {
        sessionClose();
        Serial.println("OK stopped");
    } else if (c == 'G') {
        if (!g_recording && sessionOpen()) { g_recording = true; Serial.println("OK recording"); }
        else Serial.println("ERR could not start");
    } else if (c == 'X') {
        sessionClose();
        /* Collect the names FIRST, then delete.
         * openNextFile() hands back a live File whose descriptor stays open
         * until it goes out of scope, and LittleFS refuses to unlink a path
         * that still has an open FD - which is why deleting inside the
         * iteration loop failed on every single file. */
        static String names[96];
        int cnt = 0;
        File d = LittleFS.open("/");
        for (File f = d.openNextFile(); f && cnt < 96; f = d.openNextFile()) {
            names[cnt++] = String("/") + f.name();
            f.close();
        }
        d.close();
        int ok = 0;
        for (int i = 0; i < cnt; i++)
            if (LittleFS.remove(names[i].c_str())) ok++;
        Serial.printf("OK erased %d of %d\n", ok, cnt);
        if (ok < cnt) Serial.println("# some survived - send F to format instead");
    } else if (c == 'F') {
        /* Nuclear option: reformat the partition. Guaranteed clean, and the
         * fallback when individual unlinks misbehave. */
        sessionClose();
        Serial.println("formatting, ~10 s...");
        Serial.flush();
        bool r = LittleFS.format();
        g_fsOk = LittleFS.begin(true, "/littlefs", 10, FS_PARTITION_LABEL);
        Serial.printf("%s (fs %s)\n", r ? "OK formatted" : "ERR format failed",
                      g_fsOk ? "remounted" : "MOUNT FAILED - power cycle");
    } else if (c == 'D') {
        int sp = cmd.indexOf(' ');
        if (sp < 0) { Serial.println("ERR usage: D <name>"); return; }
        String name = cmd.substring(sp + 1); name.trim();
        if (!name.startsWith("/")) name = "/" + name;
        File f = LittleFS.open(name, FILE_READ);
        if (!f) { Serial.println("ERR no such file"); return; }
        size_t sz = f.size();
        Serial.printf("BEGIN %s %u\n", name.c_str(), (unsigned)sz);
        Serial.flush();
        uint32_t sum = 0;
        uint8_t buf[512];
        while (f.available()) {
            size_t n = f.read(buf, sizeof(buf));
            for (size_t i = 0; i < n; i++) sum += buf[i];
            Serial.write(buf, n);
        }
        f.close();
        Serial.flush();
        Serial.printf("\nEND %lu\n", (unsigned long)sum);
    }
}

/* =============================== SETUP/LOOP ================================ */

void setup() {
    Serial.begin(115200);
    led.begin(); led.setBrightness(LED_BRIGHTNESS); setLed(0, 0, 40);

    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 1500) delay(10);
    Serial.println("\n# COWLLAR tag");

    /* Ring buffer: PSRAM if present. 1 MB is ~21 minutes of headroom, so a
     * flash stall would have to last twenty minutes before a sample is lost. */
    g_ringN = RING_RECORDS_PSRAM;
    g_ring = (tag_rec_t *)ps_malloc((size_t)g_ringN * sizeof(tag_rec_t));
    if (!g_ring) {
        g_ringN = RING_RECORDS_SRAM;
        g_ring = (tag_rec_t *)malloc((size_t)g_ringN * sizeof(tag_rec_t));
        Serial.println("# WARNING: no PSRAM, using small SRAM buffer");
    }
    Serial.printf("# ring %lu records (%.1f s of headroom)\n",
                  (unsigned long)g_ringN, (double)g_ringN / TAG_ODR_HZ);

    g_fsOk = LittleFS.begin(true, "/littlefs", 10, FS_PARTITION_LABEL);
    if (g_fsOk) {
        size_t tot = LittleFS.totalBytes(), used = LittleFS.usedBytes();
        g_fullPct = tot ? (uint8_t)(used * 100 / tot) : 0;
        Serial.printf("# fs %u/%u bytes (%u%%), ~%.1f h free\n",
                      (unsigned)used, (unsigned)tot, g_fullPct,
                      (double)(tot - used) / (sizeof(tag_rec_t) * TAG_ODR_HZ) / 3600.0);
    } else {
        Serial.println("# FS MOUNT FAILED — check partitions_16mb.csv is selected");
    }

    g_imuOk = imuBegin();

    xTaskCreatePinnedToCore(samplerTask, "sampler", 4096, nullptr, 5, nullptr, 0);
    xTaskCreatePinnedToCore(writerTask,  "writer",  8192, nullptr, 3, nullptr, 1);

    /* 3 s grace so a laptop can send S/D before recording starts. Unplugged in
     * a field, nothing arrives and it just begins — no button to press, and no
     * button for a cow to find. */
    Serial.println("# starting in 3 s (send any command to stay idle)");
    t0 = millis();
    while (millis() - t0 < 3000) {
        if (Serial.available()) { handleCommand(Serial.readStringUntil('\n')); return; }
        updateLed();
        delay(10);
    }
    if (g_imuOk && g_fsOk && sessionOpen()) g_recording = true;
}

void loop() {
    if (Serial.available()) handleCommand(Serial.readStringUntil('\n'));
    updateLed();
    delay(5);
}
