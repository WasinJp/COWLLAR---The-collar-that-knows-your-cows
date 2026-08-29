# COWLLAR — Live Activity Classifier Demo (Cardputer Adv)

Runs the collar's processing pipeline on the Cardputer Adv's **built-in BMI270 IMU**
and shows live behaviour classification on screen. This is the "same algorithm,
running right now" stage demo.

```
BMI270 @ 25 Hz → 128-sample sliding window → 8 features
  → decision tree → REST / RUMINATE / GRAZE / WALK / ACTIVE
  → EWMA baseline → heat-candidate flag (red banner)
```

## Build & flash

1. Install [PlatformIO](https://platformio.org/) (VS Code extension is easiest).
2. Open this folder, connect the Cardputer Adv via USB-C, power ON.
3. `pio run -t upload` — if the port isn't found, hold **BOOT (G0)** while
   plugging in, then retry.
4. Serial monitor (optional): `pio device monitor` @ 115200.

## Demo script — how to act like a cow

| Behaviour | How to perform it | What the classifier keys on |
|---|---|---|
| REST | Flat on the table, hands off | SMA ≈ 0 |
| RUMINATE | Rock rhythmically ~1–1.5×/s | Energy in 0.8–2.0 Hz band |
| GRAZE | Slow irregular tilt/drift | Mid SMA, no dominant band |
| WALK | Carry it while walking | Gait energy 1.8–3.5 Hz |
| ACTIVE | Shake vigorously | SMA above active threshold |
| **Heat flag** | Keep shaking ~10 s | Sustained σ-excess over baseline |

Keys: **D** detail view (features + band-energy bars) · **R** reset baseline ·
**B** buzzer beep (shock-free cue talking point).

Suggested stage flow: REST → detail view on → RUMINATE (point at the green
band bar rising) → WALK → shake until the red **HEAT CANDIDATE** banner trips →
"on the collar, this packet goes over LoRa mesh and arrives as a LINE message."

## What matches production vs. what's demo-tuned

| Aspect | Production collar | This demo | Why |
|---|---|---|---|
| ODR | 25 Hz | 25 Hz | identical |
| Window | 256 (10.24 s) | 128, hop 32 (1.28 s updates) | stage responsiveness |
| Features (8) | CMSIS-DSP | same math, plain C + arduinoFFT | platform |
| Tree structure | trained export (pending field data) | same structure, hand-tuned constants | Cardputer-in-hand ≠ collar-on-cow |
| Estrus mechanism | EWMA + kσ + sustain 18 | same, sustain 8 (~10 s) | audience shouldn't wait minutes |

If a judge asks "is this the real algorithm?" — the honest answer is yes for the
pipeline (features, tree structure, baseline mechanism) and "calibrated per
platform" for the constants; field data on Thai cattle produces the collar's
real constants. That answer is in the table above.

## Known caveats

- **M5Unified version:** BMI270/Cardputer-Adv support needs a recent M5Unified
  (pinned ^0.2.2). If the screen says "IMU not found", bump the version.
- **Keyboard:** uses the `M5Cardputer` library (the Adv's keyboard sits behind a
  TCA8418 I2C controller). If M5Cardputer@^1.1.0 predates full Adv support in
  your setup, update the library or check M5Stack's Cardputer-Adv examples.
- **The professor's cap changes the plan (good):** the CAP-LoRa-1262 carries an
  SX1262 **and** an ATGM336H GNSS — this one device can prototype the full
  ANCHOR role. Radio covers 868–923 MHz so AS923 works, but the bundled antenna
  is 868-tuned: fine on the bench, swap to a 923 MHz whip (~100–200 THB) before
  any range measurement. Cap pins: SX1262 SPI SCK G40 / MOSI G14 / MISO G39 /
  NSS G5 / RST G3 / IRQ G4 / BUSY G6; GNSS UART RX G13 / TX G15 (SPI shared
  with microSD — distinct CS, coexists fine).
- **This build has no LoRa.** The base Cardputer Adv has no LoRa radio — mesh
  needs the optional cap, and the common cap is **868 MHz (EU band), not
  AS923** — fine on the bench, no outdoor TX, no range claims from it.
- Timer/FFT sizes are all in `demo_config.h` — one file to retune.
