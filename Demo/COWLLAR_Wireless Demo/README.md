# Cow Ear Tag → Cardputer Demo (v2 — presentation day)

Ear tag senses motion, filters ear-flick transients, classifies behavior,
self-learns a baseline for anomaly alerts, and streams live to the Cardputer.

## Flash checklist (do in this order)

1. Arduino IDE → Boards Manager → **esp32 by Espressif (core 3.x)**.
2. Install libraries: **M5Cardputer** (+deps auto). If using a real MPU6050:
   **Adafruit MPU6050**.
3. Flash `cardputer_receiver.ino` → board "M5Cardputer".
4. Flash `ear_tag_node.ino` → your ESP32 tag board.
   - No IMU? Set `SIMULATION_MODE 1` (it even injects fake ear flicks so the
     filter banner still demos).
   - Non-default I2C pins (e.g. M5StickC Plus): `Wire.begin(32, 33)`.
5. **The packet struct changed in v2 — both devices must run v2.**

## 15-minute tuning session BEFORE the presentation

Hand-test and adjust these constants in the tag sketch:

- `classifyState()` thresholds (0.005 / 0.05 / 0.6): hold still, sway
  gently, shake — make sure each maps to the state you want on screen.
- `ANOMALY_Z` (2.5): if the alert trips too easily during normal handling,
  raise to 3.0; if the "hold still" alert takes too long, lower to 2.0.
- `SPIKE_SIGMA` (5.0): flick the tag sharply once with your finger — the
  cyan "ear-flick transient rejected" banner should pop and the state
  should NOT change. If flicks leak into ACTIVE, lower to 4.0.
- Let the tag run ~20 s after boot before demoing the alert (it needs
  8 windows to learn its baseline — that's `BASE_MIN_N`).

## Stage script (~75 seconds)

1. Power both. Dashboard live in ~2 s. Point at RSSI + packet counter:
   "real radio traffic, right now."
2. Sway the tag gently → GRAZING. Shake → ACTIVE (note it takes 2 windows —
   that's hysteresis, say the word).
3. **Flick it once, sharply.** Cyan banner: *ear-flick transient rejected*,
   flick counter increments, state unchanged. "Cows flap their ears
   constantly — the tag separates sub-second transients from sustained
   behavior using robust statistics, on-device."
4. Hold perfectly still 15–20 s → z climbs → red ALERT + beeper.
   "Each tag learns that individual cow's own normal — no labeled dataset
   required for this. In production this alert arrives as a LINE message."
5. Press A to acknowledge. Power off the tag → LINK LOST after 8 s:
   "and that's the theft alarm."

## Q&A prep — the three concerns, answered honestly

**Weight / ear droop.** Commercial ear-tag monitors run ~25–35 g and are
tolerated long-term by cattle. Our follower tag carries no GPS and a small
battery, so sub-30 g is a realistic target. Heavy components (GPS, bigger
battery) live on the anchor COLLAR — one per herd, neck-mounted where weight
is a non-issue. Farms can choose collar-only or collar+tags.

**Ear motion vs neck motion.** Ear data is noisier, but noise and signal
live at different time scales: flicks are sub-second transients; grazing/
rumination are sustained rhythmic patterns. Pipeline: median/MAD robust
statistics → transient rejection → classification → 2-window hysteresis.
You just watched it reject a flick live.

**No cow data yet (be honest, then pivot).** "Correct — behavior
classification thresholds ultimately need real cattle data. Our plan:
bootstrap from published cattle-accelerometer research, then calibrate on a
pilot herd, and we're transparent that this is a next step. But the core
alert feature is UNSUPERVISED — each tag baselines that individual animal,
so anomaly detection works from day one without labels. Labels improve
classification over time; alerts don't wait for them."

## Production mapping (say these numbers confidently)

| Demo today | Production |
|---|---|
| ESP-NOW, 2 s TX | LoRa, duty-cycled 5–15 min TX, deep sleep |
| Cardputer screen | Gateway → cloud → LINE Messaging API push |
| 1-D z-score | Mahalanobis over activity + rumination + RSSI-proximity |
| USB/LiPo | Small primary cell, multi-year target, <30 g tag |

## Troubleshooting

- No packets: same channel (1) on both? Both running v2 struct? >30 cm apart.
- Callback compile error: you're on esp32 core 2.x — use the 2.x signature
  commented in the receiver (no RSSI in 2.x).
- MPU6050 not found: check wiring/pins, or flip to SIMULATION_MODE 1 and
  demo anyway — the wireless link is still fully real.
