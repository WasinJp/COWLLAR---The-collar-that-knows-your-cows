/*
 * main.cpp — COWLLAR labeller (Cardputer Adv)
 * -----------------------------------------------------------------------------
 * Stays in your hands. Records nothing but WHAT YOU SAW and WHEN.
 * The tag on the cow records the motion. merge_labels.py joins them.
 *
 * Output: /LBL####.CSV on the microSD, plain text, human-readable.
 *   t_ms,event,code,name
 *
 * KEYS
 *   1 LYING  2 STANDING  3 RUMINATE  4 GRAZE  5 WALK  6 DRINK  7 OTHER
 *   0 UNKNOWN — press the instant you are unsure or lose sight of her.
 *               A guessed label does not merely add noise; it MOVES the fitted
 *               threshold. Unknown is free. Wrong is expensive.
 *   M  sync marker   N  new session   S  stop   U  undo last label
 *
 * THE TAP RITUAL — do not skip it, not once
 *   Hold the tag against this device. TAP THEM TOGETHER THREE TIMES while
 *   pressing M. Once at the start, once at the end. The taps show up as spikes
 *   in the tag's accelerometer; the first pair gives the clock offset and the
 *   second pair proves it was right.
 * -----------------------------------------------------------------------------
 */
#include <M5Cardputer.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <LittleFS.h>

#define SD_SCK 40
#define SD_MISO 39
#define SD_MOSI 14
#define SD_CS_CANDIDATES { 12, 11, 10, 5, 4 }

enum { L_UNKNOWN, L_LYING, L_STANDING, L_RUMINATE, L_GRAZE, L_WALK, L_DRINK, L_OTHER, L_COUNT };
static const char *NAMES[L_COUNT] = {
    "UNKNOWN", "LYING", "STANDING", "RUMINATE", "GRAZE", "WALK", "DRINK", "OTHER"
};
static const uint16_t COLORS[L_COUNT] = {
    0x7BEF, 0x421F, 0x05FF, 0x07E8, 0xFE60, 0x04FF, 0x07FF, 0xF981
};

/* Storage: SD if a card mounts, otherwise the Cardputer's own flash.
 *
 * This fallback is not a nicety. Label files are the ONLY record of what you
 * saw; the tag records motion but has no idea what the cow was doing. Serial-
 * only means powering off loses the entire session, and at a farm there is no
 * terminal attached. Internal flash always exists, so the labels always land
 * somewhere. Press P afterwards to print them out over USB. */
typedef enum { STORE_NONE = 0, STORE_SD, STORE_FLASH } store_t;
static store_t  g_store = STORE_NONE;
static bool     g_openFail = false;   /* file would not open - LOUD, not silent */
static uint32_t g_written = 0;        /* events actually committed to storage   */
static File     g_f;
static bool     g_sd = false;
static char     g_fname[24] = "-";
static bool     g_run = false;
static uint32_t g_t0 = 0;
static uint8_t  g_label = L_UNKNOWN;
static uint32_t g_labelStart = 0;
static uint32_t g_secs[L_COUNT] = {0};
static uint16_t g_marks = 0;
static uint16_t g_events = 0;

/* Last few events, shown on screen. Seeing what you just pressed is the only
 * way to catch a fat-fingered key before it becomes bad training data. */
#define HIST 4
static char g_hist[HIST][28];
static uint8_t g_histN = 0;

static void pushHist(const char *s) {
    for (int i = HIST - 1; i > 0; i--) memcpy(g_hist[i], g_hist[i-1], 28);
    strncpy(g_hist[0], s, 27); g_hist[0][27] = 0;
    if (g_histN < HIST) g_histN++;
}

static void emit(const char *event, int code, const char *name) {
    uint32_t t = millis() - g_t0;
    char line[64];
    snprintf(line, sizeof(line), "%lu,%s,%d,%s\n", (unsigned long)t, event, code, name);
    if (g_store != STORE_NONE && g_f) {
        g_f.print(line);
        g_f.flush();                 /* commit now: a label in RAM is a label lost */
        g_written++;
    } else {
        g_openFail = true;
    }   /* flush every event:
                                                           they are tiny and
                                                           losing one is fatal */
    Serial.print(line);
    char h[28];
    snprintf(h, sizeof(h), "%3lus %s", (unsigned long)(t / 1000), name);
    pushHist(h);
    g_events++;
}

static bool sdMount() {
    const int cs[] = SD_CS_CANDIDATES;
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, cs[0]);
    for (unsigned i = 0; i < sizeof(cs)/sizeof(cs[0]); i++) {
        if (SD.begin(cs[i], SPI, 20000000)) {
            Serial.printf("# SD mounted on CS=%d\n", cs[i]);
            return true;
        }
        SD.end(); delay(20);
    }
    Serial.println("# no SD (card must be FAT32, not exFAT) - using flash");
    return false;
}

static fs::FS *fsp() { return (g_store == STORE_SD) ? (fs::FS *)&SD : (fs::FS *)&LittleFS; }

/* Dump every stored label file to USB as plain text. Copy-paste out of the
 * serial monitor - no tooling needed, the files are only a few KB. */
static void dumpAll() {
    fs::FS *f = fsp();
    File d = f->open("/");
    Serial.println("===== BEGIN LABEL FILES =====");
    for (File e = d.openNextFile(); e; e = d.openNextFile()) {
        String n = String(e.name());
        if (n.indexOf("LBL") < 0) continue;
        Serial.printf("----- %s (%u bytes) -----\n", e.name(), (unsigned)e.size());
        while (e.available()) Serial.write(e.read());
        Serial.println();
    }
    Serial.println("===== END LABEL FILES =====");
}

static void sessionStart() {
    if (g_store != STORE_NONE) {
        fs::FS *f = fsp();
        uint32_t id = 1;
        while (id < 10000) {
            snprintf(g_fname, sizeof(g_fname), "/LBL%04lu.CSV", (unsigned long)id);
            if (!f->exists(g_fname)) break;
            id++;
        }
        /* The third argument is create=true and it is NOT optional on LittleFS:
         * FS::open() defaults create=false, and vfs_api refuses to open a file
         * that does not exist yet. Without it every session silently produced
         * nothing. */
        g_f = f->open(g_fname, FILE_WRITE, true);
        if (g_f) {
            g_f.print("t_ms,event,code,name\n");
            g_f.flush();
            g_openFail = false;
        } else {
            g_openFail = true;      /* screen goes red - see draw() */
            Serial.printf("# CANNOT CREATE %s\n", g_fname);
        }
    }
    g_t0 = millis(); g_run = true;
    g_label = L_UNKNOWN; g_labelStart = g_t0;
    memset(g_secs, 0, sizeof(g_secs));
    g_marks = 0; g_events = 0; g_written = 0; g_histN = 0;
    emit("START", 0, "session");
}

static void sessionStop() {
    if (!g_run) return;
    emit("STOP", 0, "session");
    g_run = false;
    if (g_f) g_f.close();
}

static void setLabel(uint8_t l) {
    if (!g_run || l >= L_COUNT) return;
    uint32_t now = millis();
    if (g_label < L_COUNT) g_secs[g_label] += (now - g_labelStart) / 1000;
    g_labelStart = now;
    g_label = l;
    emit("LABEL", l, NAMES[l]);
    M5.Speaker.tone(1400 + 120 * l, 40);
}

static void draw() {
    auto &d = M5.Display;
    d.startWrite();
    d.fillScreen(TFT_BLACK);
    d.setTextSize(1);
    d.setTextDatum(top_left);
    d.setTextColor(TFT_DARKGREY, TFT_BLACK);
    d.setCursor(2, 2);
    const char *st = (g_store == STORE_SD) ? "SD" : (g_store == STORE_FLASH ? "FLASH" : "NONE!");
    if (g_openFail) {
        d.fillRect(0, 0, d.width(), 10, TFT_RED);
        d.setTextColor(TFT_WHITE, TFT_RED);
        d.printf("!! NOT SAVING - STORAGE FAILED !!");
    } else {
        d.printf("%s %s sv:%u mk:%u", st,
                 g_store != STORE_NONE ? g_fname + 1 : "-",
                 (unsigned)g_written, g_marks);
    }

    d.setTextDatum(middle_center);
    d.setTextColor(COLORS[g_label], TFT_BLACK);
    d.setTextSize(3);
    d.drawString(NAMES[g_label], d.width() / 2, 30);

    uint32_t sec = g_run ? (millis() - g_t0) / 1000 : 0;
    uint32_t hold = g_run ? (millis() - g_labelStart) / 1000 : 0;
    d.setTextDatum(top_left);
    d.setTextSize(1);
    d.setTextColor(g_run ? TFT_WHITE : 0xF981, TFT_BLACK);
    d.setCursor(2, 50);
    d.printf("%s %02lu:%02lu   held %lus",
             g_run ? "REC" : "IDLE", (unsigned long)(sec/60), (unsigned long)(sec%60),
             (unsigned long)hold);

    /* recent events — catch a mis-key while it is still fixable */
    d.setTextColor(0x8410, TFT_BLACK);
    for (uint8_t i = 0; i < g_histN; i++) {
        d.setCursor(2, 64 + i * 9);
        d.print(g_hist[i]);
    }

    /* coverage: you do not have a dataset until every class has real minutes */
    int y = 64;
    for (uint8_t i = 1; i < L_COUNT; i++) {
        d.setCursor(128, y);
        d.setTextColor(g_secs[i] ? COLORS[i] : 0x39E7, TFT_BLACK);
        d.printf("%.4s %lu", NAMES[i], (unsigned long)g_secs[i]);
        y += 9;
    }

    if (g_run && g_marks == 0) {
        d.fillRect(0, d.height() - 12, d.width(), 12, 0xF981);
        d.setTextColor(TFT_WHITE, 0xF981);
        d.setTextDatum(middle_center);
        d.drawString("TAP x3 + press M to sync!", d.width() / 2, d.height() - 6);
    }
    d.endWrite();
}

void setup() {
    auto cfg = M5.config();
    M5Cardputer.begin(cfg, true);

    /* Serial.begin() is NOT optional here. With ARDUINO_USB_CDC_ON_BOOT the CDC
     * port is usually up already, but M5Cardputer.begin() can leave the Arduino
     * Serial object uninitialised - the symptom being that ESP-IDF's own [E]
     * log lines appear in the monitor while every Serial.print from our code
     * silently goes nowhere. That is exactly what made P look broken. */
    Serial.begin(115200);
    delay(300);
    Serial.println("\n# COWLLAR labeller ready");
    Serial.println("# events print here live; S then P dumps stored files");

    M5.Display.setRotation(1);
    M5.Display.setBrightness(140);
    M5.Display.setTextSize(1);
    M5.Display.setCursor(2, 2);
    g_sd = sdMount();
    if (g_sd) {
        g_store = STORE_SD;
    } else if (LittleFS.begin(true)) {          /* true = format if unformatted */
        g_store = STORE_FLASH;
        Serial.println("# labels -> internal flash. Press P to print them.");
    }
    M5.Display.setTextColor(g_store != STORE_NONE ? TFT_GREEN : TFT_RED, TFT_BLACK);
    M5.Display.printf(g_store == STORE_SD ? "storage: SD card\n"
                    : g_store == STORE_FLASH ? "storage: INTERNAL FLASH\n"
                    : "NO STORAGE - DO NOT USE\n");
    M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Display.printf("\nCOWLLAR labeller\n\n1-7 label  0 unknown\nM sync  N new  S stop  U undo\nP print files to USB\n");
    delay(2000);
    Serial.println("# COWLLAR labeller. t_ms,event,code,name");
}

void loop() {
    M5Cardputer.update();
    if (M5Cardputer.Keyboard.isChange() && M5Cardputer.Keyboard.isPressed()) {
        for (uint8_t i = 0; i < L_COUNT; i++)
            if (M5Cardputer.Keyboard.isKeyPressed('0' + i)) setLabel(i);
        if (M5Cardputer.Keyboard.isKeyPressed('m') && g_run) {
            g_marks++;
            emit("MARK", g_marks, "sync");
            M5.Speaker.tone(3000, 250);
        }
        if (M5Cardputer.Keyboard.isKeyPressed('n')) { sessionStop(); sessionStart(); }
        if (M5Cardputer.Keyboard.isKeyPressed('s')) { sessionStop(); M5.Speaker.tone(600, 300); }
        if (M5Cardputer.Keyboard.isKeyPressed('p')) {
            if (g_run) {
                /* Reading a file that is still open for writing is asking for
                 * trouble. Say so on the screen rather than doing nothing,
                 * which is indistinguishable from a broken key. */
                M5.Display.fillRect(0, 50, M5.Display.width(), 12, 0xF981);
                M5.Display.setTextColor(TFT_WHITE, 0xF981);
                M5.Display.setTextDatum(top_left);
                M5.Display.setCursor(2, 51);
                M5.Display.print("press S to stop first!");
                M5.Speaker.tone(400, 300);
                delay(1200);
            } else {
                M5.Display.fillRect(0, 50, M5.Display.width(), 12, 0x04FF);
                M5.Display.setTextColor(TFT_WHITE, 0x04FF);
                M5.Display.setTextDatum(top_left);
                M5.Display.setCursor(2, 51);
                M5.Display.print("dumping to USB...");
                dumpAll();
                M5.Speaker.tone(2000, 200);
                delay(600);
            }
        }
        if (M5Cardputer.Keyboard.isKeyPressed('u') && g_run) {
            /* Undo is a NOTE, not a deletion. The merge tool honours it. Never
             * rewrite history in a data file — an append-only log you can audit
             * beats a tidy one you cannot. */
            emit("UNDO", 0, "previous");
            M5.Speaker.tone(400, 150);
        }
    }
    static uint32_t last = 0;
    if (millis() - last > 250) { last = millis(); draw(); }
}
