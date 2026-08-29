#include <Arduino.h>
/*
 * ============================================================
 *  COW EAR TAG NODE  v3  (LSM6DS3 + ESP32-S3 edition)
 *  Board:  ESP32-S3 (any dev board)
 *  Sensor: LSM6DS3 / LSM6DS3TR-C on I2C  (gyro disabled = power)
 *  Radio:  ESP-NOW broadcast -> Cardputer receiver
 *
 *  Arduino IDE:
 *    - esp32 by Espressif (core 3.x), pick your ESP32-S3 board
 *    - Library: "SparkFun LSM6DS3 Breakout" (Library Manager)
 *
 *  SAME packet struct as receiver v2 -- Cardputer needs no reflash.
 *
 *  Pipeline: 25 Hz |accel| -> median/MAD robust stats -> ear-flick
 *  transient rejection -> behavior classification -> 2-window
 *  hysteresis -> per-animal rolling-baseline anomaly z-score.
 * ============================================================
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ---------- CONFIG ----------------------------------------------------------
#define SIMULATION_MODE 0        // 1 = demo without the IMU (injects fake flicks)
#define COW_ID          7
#define TX_INTERVAL_MS  2000
#define ESPNOW_CHANNEL  1

// I2C pins -- SET THESE FOR YOUR S3 BOARD's wiring
#define I2C_SDA_PIN     8
#define I2C_SCL_PIN     9
#define LSM6DS3_ADDR    0x6B     // 0x6A if SDO/SA0 pulled low

#define SAMPLE_HZ       25
#define WINDOW_SAMPLES  50       // 2 s window
#define HYST_WINDOWS    2
#define SPIKE_SIGMA     5.0f
#define MAX_FLICK_FRAC  0.20f

#if !SIMULATION_MODE
  #include <Wire.h>
  #include "SparkFunLSM6DS3.h"
  LSM6DS3 imu(I2C_MODE, LSM6DS3_ADDR);
#endif

// ---------- PACKET (matches receiver v2 EXACTLY) -----------------------------
typedef struct __attribute__((packed)) {
  uint8_t  cowId;
  uint8_t  state;          // 0=Resting 1=Ruminating 2=Grazing 3=Active
  float    activityIndex;
  float    anomalyScore;
  uint8_t  alert;
  uint16_t flickCount;
  float    tempC;
  float    battV;
  uint32_t seq;
} TagPacket;

static const uint8_t BROADCAST_ADDR[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// ---------- STATE -----------------------------------------------------------
float    win[WINDOW_SAMPLES];
uint32_t seqNum = 0;
uint16_t flickTotal = 0;

uint8_t displayedState = 0, candState = 0, candRepeats = 0;

float baseMean = 0, baseM2 = 0;
uint32_t baseN = 0;
const float ANOMALY_Z = 2.5f;
const uint32_t BASE_MIN_N = 8;

// ---------- SENSOR / SIM ----------------------------------------------------
float sampleAccelMag(int i) {
#if SIMULATION_MODE
  static float phase = 0;
  static int winNo = 0;
  if (i == 0) winNo++;
  phase += 0.13f;
  float v = 9.81f + 0.6f * sinf(phase) + 0.3f * sinf(phase * 2.7f)
            + (random(-100, 100)) / 300.0f;
  if ((winNo % 7 == 0) && i >= 20 && i < 24) v += 6.0f;   // fake ear flick
  return v;
#else
  (void)i;
  float x = imu.readFloatAccelX();
  float y = imu.readFloatAccelY();
  float z = imu.readFloatAccelZ();
  return 9.80665f * sqrtf(x * x + y * y + z * z);  // LSM6DS3 returns g -> m/s^2
#endif
}

float readTempC() {
#if SIMULATION_MODE
  return 38.5f + (random(-30, 30)) / 100.0f;
#else
  return imu.readTempC();
#endif
}

float readBattV() { return 3.95f + (random(-5, 5)) / 100.0f; }

// ---------- ROBUST STATS (unchanged from v2) ---------------------------------
static int cmpf(const void* a, const void* b) {
  float d = *(const float*)a - *(const float*)b;
  return (d > 0) - (d < 0);
}

float medianOf(float* src, int n) {
  static float tmp[WINDOW_SAMPLES];
  memcpy(tmp, src, n * sizeof(float));
  qsort(tmp, n, sizeof(float), cmpf);
  return (n & 1) ? tmp[n/2] : 0.5f * (tmp[n/2 - 1] + tmp[n/2]);
}

float computeRobustActivity(bool* flickDetected) {
  *flickDetected = false;
  float med = medianOf(win, WINDOW_SAMPLES);

  static float dev[WINDOW_SAMPLES];
  for (int i = 0; i < WINDOW_SAMPLES; i++) dev[i] = fabsf(win[i] - med);
  float mad = medianOf(dev, WINDOW_SAMPLES);
  float robustSigma = 1.4826f * mad + 1e-4f;

  int spiky = 0;
  for (int i = 0; i < WINDOW_SAMPLES; i++)
    if (fabsf(win[i] - med) > SPIKE_SIGMA * robustSigma) spiky++;

  if (spiky > 0 && spiky <= (int)(MAX_FLICK_FRAC * WINDOW_SAMPLES)) {
    for (int i = 0; i < WINDOW_SAMPLES; i++)
      if (fabsf(win[i] - med) > SPIKE_SIGMA * robustSigma) win[i] = med;
    *flickDetected = true;
  }

  float mean = 0;
  for (int i = 0; i < WINDOW_SAMPLES; i++) mean += win[i];
  mean /= WINDOW_SAMPLES;
  float var = 0;
  for (int i = 0; i < WINDOW_SAMPLES; i++) {
    float d = win[i] - mean;
    var += d * d;
  }
  return var / WINDOW_SAMPLES;
}

uint8_t classifyState(float ai) {
  if (ai < 0.01f) return 0;
  if (ai < 0.3f)  return 1;
  if (ai < 3.0f)   return 2;
  return 3;
}

uint8_t applyHysteresis(uint8_t candidate) {
  if (candidate == displayedState) { candRepeats = 0; return displayedState; }
  if (candidate == candState) {
    if (++candRepeats >= HYST_WINDOWS) { displayedState = candidate; candRepeats = 0; }
  } else { candState = candidate; candRepeats = 1; }
  return displayedState;
}

float updateAnomaly(float ai) {
  float z = 0;
  if (baseN >= BASE_MIN_N) {
    float sd = sqrtf(baseM2 / (baseN - 1)) + 1e-6f;
    z = fabsf(ai - baseMean) / sd;
  }
  if (baseN < BASE_MIN_N || z < ANOMALY_Z) {
    baseN++;
    float d = ai - baseMean;
    baseMean += d / baseN;
    baseM2   += d * (ai - baseMean);
  }
  return z;
}

// ---------- SETUP -----------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);

#if !SIMULATION_MODE
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // Configure BEFORE begin(): accel only, gyro off (battery story!)
  imu.settings.gyroEnabled        = 0;
  imu.settings.accelEnabled       = 1;
  imu.settings.accelRange         = 4;     // +/-4 g
  imu.settings.accelSampleRate    = 104;   // Hz (we decimate to 25 in loop)
  imu.settings.accelBandWidth     = 50;    // Hz anti-alias filter
  imu.settings.tempEnabled        = 1;

  if (imu.begin() != 0) {
    Serial.println("LSM6DS3 not found. Check SDA/SCL pins + addr (0x6B/0x6A),");
    Serial.println("or set SIMULATION_MODE 1 to demo without it.");
    while (true) delay(1000);
  }
  Serial.println("LSM6DS3 up (accel only, gyro off).");
#endif

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW init failed"); while (1) delay(1000); }
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST_ADDR, 6);
  peer.channel = ESPNOW_CHANNEL;
  esp_now_add_peer(&peer);

  Serial.printf("Ear tag v3 COW-%02d up, ESP-NOW ch %d\n", COW_ID, ESPNOW_CHANNEL);
}

// ---------- LOOP ------------------------------------------------------------
void loop() {
  uint32_t t0 = millis();

  for (int i = 0; i < WINDOW_SAMPLES; i++) {
    win[i] = sampleAccelMag(i);
    delay(1000 / SAMPLE_HZ);
  }

  bool flick = false;
  float ai = computeRobustActivity(&flick);
  if (flick) flickTotal++;

  uint8_t state = applyHysteresis(classifyState(ai));
  float z = updateAnomaly(ai);

  TagPacket pkt;
  pkt.cowId         = COW_ID;
  pkt.state         = state;
  pkt.activityIndex = ai;
  pkt.anomalyScore  = z;
  pkt.alert         = (baseN >= BASE_MIN_N && z >= ANOMALY_Z) ? 1 : 0;
  pkt.flickCount    = flickTotal;
  pkt.tempC         = readTempC();
  pkt.battV         = readBattV();
  pkt.seq           = seqNum++;

  esp_err_t r = esp_now_send(BROADCAST_ADDR, (uint8_t*)&pkt, sizeof(pkt));
  Serial.printf("seq=%lu st=%u ai=%.4f z=%.2f flicks=%u alert=%u (%s)\n",
                (unsigned long)pkt.seq, state, ai, z, flickTotal, pkt.alert,
                r == ESP_OK ? "sent" : "FAIL");

  uint32_t el = millis() - t0;
  if (el < TX_INTERVAL_MS) delay(TX_INTERVAL_MS - el);
}
