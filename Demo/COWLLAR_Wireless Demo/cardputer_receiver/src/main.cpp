#include <Arduino.h>
/*
 * ============================================================
 *  HERD MONITOR RECEIVER  v2  (base station demo)
 *  Board: M5Stack Cardputer (ESP32-S3), esp32 core 3.x
 *  Library: M5Cardputer
 *
 *  v2: matches ear tag v2 packet (flickCount added). Shows a
 *  cyan "ear-flick transient rejected" banner when the tag
 *  filters a flick -- the live answer to the ear-motion question.
 *
 *  Keys: A = acknowledge alert   R = reset packet counter
 * ============================================================
 */

#include <M5Cardputer.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define ESPNOW_CHANNEL 1
#define STALE_MS       8000

// ---------- PACKET (must match tag v2 EXACTLY) -------------------------------
typedef struct __attribute__((packed)) {
  uint8_t  cowId;
  uint8_t  state;
  float    activityIndex;
  float    anomalyScore;
  uint8_t  alert;
  uint16_t flickCount;
  float    tempC;
  float    battV;
  uint32_t seq;
} TagPacket;

// ---------- SHARED STATE ------------------------------------------------------
volatile bool     havePacket = false;
TagPacket         lastPkt;
volatile int      lastRssi = 0;
volatile uint32_t lastRxMs = 0;
volatile uint32_t rxCount  = 0;
bool alertLatched = false;
bool alertAcked   = false;

uint16_t prevFlicks   = 0;
uint32_t flickToastMs = 0;      // show banner until this time

const char* STATE_NAMES[]   = {"RESTING", "RUMINATING", "GRAZING", "ACTIVE"};
const uint16_t STATE_COLORS[] = {0x867D, 0x07FF, 0x07E0, 0xFD20};

// ---------- ESP-NOW CALLBACK (core 3.x signature) -----------------------------
void onDataRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (len != sizeof(TagPacket)) return;
  memcpy((void*)&lastPkt, data, sizeof(TagPacket));
  if (info && info->rx_ctrl) lastRssi = info->rx_ctrl->rssi;
  lastRxMs = millis();
  rxCount++;
  havePacket = true;
  if (lastPkt.alert) alertLatched = true;
  else if (alertAcked) { alertLatched = false; alertAcked = false; }
}
/* esp32 core 2.x fallback signature (no RSSI):
   void onDataRecv(const uint8_t* mac, const uint8_t* data, int len) {...}  */

// ---------- UI ----------------------------------------------------------------
M5Canvas canvas(&M5Cardputer.Display);

void drawWaiting() {
  canvas.fillSprite(TFT_BLACK);
  canvas.setTextDatum(middle_center);
  canvas.setTextSize(2);
  canvas.setTextColor(TFT_DARKGREY);
  canvas.drawString("HERD MONITOR", canvas.width()/2, 40);
  canvas.setTextSize(1);
  canvas.setTextColor(TFT_WHITE);
  canvas.drawString("Waiting for ear tag...", canvas.width()/2, 70);
  canvas.drawString("ESP-NOW ch " + String(ESPNOW_CHANNEL), canvas.width()/2, 90);
  canvas.pushSprite(0, 0);
}

void drawAlert(const TagPacket& p) {
  static bool phase = false;
  phase = !phase;
  canvas.fillSprite(phase ? TFT_RED : TFT_BLACK);
  canvas.setTextDatum(middle_center);
  canvas.setTextSize(3);
  canvas.setTextColor(phase ? TFT_BLACK : TFT_RED);
  canvas.drawString("!! ALERT !!", canvas.width()/2, 28);
  canvas.setTextSize(2);
  canvas.setTextColor(phase ? TFT_BLACK : TFT_WHITE);
  char b[40];
  snprintf(b, sizeof(b), "COW-%02u", p.cowId);
  canvas.drawString(b, canvas.width()/2, 60);
  canvas.setTextSize(1);
  canvas.drawString("Abnormal behavior pattern", canvas.width()/2, 86);
  snprintf(b, sizeof(b), "anomaly z=%.1f", p.anomalyScore);
  canvas.drawString(b, canvas.width()/2, 100);
  canvas.drawString("[A] acknowledge", canvas.width()/2, 120);
  canvas.pushSprite(0, 0);
  if (phase) M5Cardputer.Speaker.tone(2400, 120);
}

void drawDashboard(const TagPacket& p, bool linkLost) {
  canvas.fillSprite(TFT_BLACK);
  canvas.setTextDatum(top_left);
  char b[48];

  // Header / flick toast
  canvas.setTextSize(1);
  if (millis() < flickToastMs) {
    canvas.fillRect(0, 0, canvas.width(), 16, TFT_CYAN);
    canvas.setTextColor(TFT_BLACK);
    canvas.drawString(" ear-flick transient rejected", 4, 4);
  } else {
    canvas.setTextColor(TFT_DARKGREY);
    canvas.drawString("HERD MONITOR - base station", 6, 4);
  }
  canvas.drawFastHLine(0, 16, canvas.width(), TFT_DARKGREY);

  // Cow ID + confirmed state
  canvas.setTextSize(2);
  canvas.setTextColor(TFT_WHITE);
  snprintf(b, sizeof(b), "COW-%02u", p.cowId);
  canvas.drawString(b, 6, 24);
  canvas.setTextColor(STATE_COLORS[p.state & 3]);
  canvas.drawString(STATE_NAMES[p.state & 3], 100, 24);

  // Activity bar
  canvas.setTextSize(1);
  canvas.setTextColor(TFT_WHITE);
  canvas.drawString("activity", 6, 50);
  int barMax = canvas.width() - 70;
  float norm = p.activityIndex / 0.8f; if (norm > 1) norm = 1;
  canvas.drawRect(60, 48, barMax, 10, TFT_DARKGREY);
  canvas.fillRect(60, 48, (int)(norm * barMax), 10, STATE_COLORS[p.state & 3]);

  // Telemetry
  snprintf(b, sizeof(b), "temp %.1fC  batt %.2fV  z=%.2f  flicks:%u",
           p.tempC, p.battV, p.anomalyScore, p.flickCount);
  canvas.drawString(b, 6, 66);

  // Link
  if (linkLost) {
    canvas.setTextColor(TFT_RED);
    canvas.drawString("LINK LOST - check tag / possible theft", 6, 82);
  } else {
    canvas.setTextColor(TFT_GREENYELLOW);
    snprintf(b, sizeof(b), "RSSI %d dBm  pkt #%lu  %lums ago",
             lastRssi, (unsigned long)p.seq,
             (unsigned long)(millis() - lastRxMs));
    canvas.drawString(b, 6, 82);
  }

  // Footer
  canvas.setTextColor(TFT_DARKGREY);
  canvas.drawFastHLine(0, 100, canvas.width(), TFT_DARKGREY);
  canvas.drawString("prod: LoRa uplink -> cloud -> LINE alert", 6, 108);
  canvas.drawString(String("rx: ") + rxCount + "  [A]=ack  [R]=reset", 6, 122);

  canvas.pushSprite(0, 0);
}

// ---------- SETUP --------------------------------------------------------------
void setup() {
  auto cfg = M5.config();
  M5Cardputer.begin(cfg, true);
  M5Cardputer.Display.setRotation(1);
  M5Cardputer.Display.setBrightness(180);
  canvas.createSprite(M5Cardputer.Display.width(), M5Cardputer.Display.height());

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) {
    M5Cardputer.Display.println("ESP-NOW init failed");
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onDataRecv);

  drawWaiting();
  M5Cardputer.Speaker.tone(1200, 80);
}

// ---------- LOOP ----------------------------------------------------------------
void loop() {
  M5Cardputer.update();

  if (M5Cardputer.Keyboard.isChange() && M5Cardputer.Keyboard.isPressed()) {
    auto ks = M5Cardputer.Keyboard.keysState();
    for (auto c : ks.word) {
      if (c == 'a' || c == 'A') { alertAcked = true; alertLatched = false;
                                  M5Cardputer.Speaker.tone(800, 60); }
      if (c == 'r' || c == 'R') rxCount = 0;
    }
  }

  if (!havePacket) { drawWaiting(); delay(150); return; }

  // New flick since last frame? Pop the banner + soft tick sound.
  if (lastPkt.flickCount != prevFlicks) {
    prevFlicks   = lastPkt.flickCount;
    flickToastMs = millis() + 1800;
    M5Cardputer.Speaker.tone(1500, 40);
  }

  bool linkLost = (millis() - lastRxMs) > STALE_MS;

  if (alertLatched && !linkLost) { drawAlert(lastPkt); delay(250); }
  else                           { drawDashboard(lastPkt, linkLost); delay(150); }
}
