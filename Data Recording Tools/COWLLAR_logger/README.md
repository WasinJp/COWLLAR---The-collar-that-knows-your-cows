# COWLLAR — Field Data Logger (Cardputer Adv)

Separate firmware from the demo. It classifies nothing. It records raw IMU at
50 Hz plus the behaviour label a human presses while watching the cow, so the
demo's hand-fitted thresholds can be replaced with numbers fitted to actual
cattle.

```
BMI270 @ 50 Hz + gyro  ->  int16 records + your keypress label  ->  microSD
                                                                     |
                                          tools/decode_log.py  <-----+
                                          tools/tune_thresholds.py -> new constants
```

## Why 50 Hz and raw, when the collar runs 25 Hz and features

Because you get one shot at the farm. Raw 50 Hz can be decimated to 25 Hz
offline and recomputed into features as many times as you like, with different
window lengths, different bands, normalised or not. A log of features at 25 Hz
locks in every one of those choices permanently. Storage is 2.9 MB/hour — this
costs nothing and buys everything.

## Before you leave the lab

1. Flash it: `pio run -t upload`. Check the boot screen says **SD mounted on
   CS=nn**. If it says NO SD, the CS pin guess list in `logger_config.h` missed
   — find the right pin now, not at the farm. (Fallback works: it streams CSV
   over USB serial, but that means dragging a laptop into a field.)
2. Set the RTC once over USB so timestamps are real.
3. Edit `g_cow` and `g_mount` in `main.cpp` per animal, or just write them in the
   notebook and rename files afterwards.
4. **Rehearse the keypresses on a person.** Have a friend walk, stand, sit, and
   chew gum while you label them in real time. You will discover that labelling
   live is harder than it sounds — better to discover it before the cow.
5. Charge everything. Bring a power bank and a USB-C cable.

## Kit list

- Cardputer Adv + power bank + cable
- A pouch or phone armband, plus a webbing strap or velcro band for the neck
- Duct tape (it is always duct tape)
- **A phone on a small tripod** — video is the ground truth that rescues bad
  keypresses
- Notebook: cow tag, mount position, start wall-clock time, weather, feed state
- Spare microSD

## Mounting

Mount position changes the data more than almost anything else in this project.
A collar riding on top of the neck sees very different rumination signal than
one hanging under the jaw.

**Decide where the production collar will sit, then mount the logger exactly
there.** If you are not sure yet, record the same cow twice, once per position,
and let the data decide — that comparison is genuinely publishable-grade
evidence for a student project and takes one extra hour.

Note the orientation too (which face is up, which way the X axis points). Write
it in the notebook. Axis conventions are the single most common cause of "the
data looks nothing like last time".

## Session protocol

1. Attach, power on, press **N** (new session).
2. Start the video. Press **M** — the buzzer beeps and the device is visible in
   frame. That beep is your clock alignment. **Do it again at the end.**
3. Press the number key each time the behaviour changes. Press **0** the instant
   you are unsure or lose sight of the animal. Guessed labels are worse than no
   labels: they do not just add noise, they move the fitted threshold.
4. Watch the per-label seconds counter on screen. You do not have a dataset
   until every class has real minutes in it.
5. Press **S** to close the file cleanly before power off.

Keys: `1` lying `2` standing `3` ruminate `4` graze `5` walk `6` drink
`7` other `0` unknown · `M` marker · `N` new session · `S` stop

## How much data is enough

For fitting four threshold constants, aim for:

| Class | Minimum | Comfortable |
|---|---|---|
| RUMINATE | 20 min | 60 min |
| GRAZE | 20 min | 60 min |
| WALK | 10 min | 30 min |
| LYING + STANDING | 20 min | 60 min |

Across **at least 3 different cows** — thresholds fitted to one animal are that
animal's thresholds. And rumination happens mostly when cows are resting after
feeding, so time your visit accordingly; a mid-morning visit right after feeding
gets grazing and walking but you may sit a long time waiting for cud-chewing.
Ask the farmer when their cows usually ruminate. They will know instantly.

Realistically, 2 visits × 3 cows × 90 minutes gives a usable first fit.

## Working with the farmer

Ask permission explicitly, explain what the device does and does not do (it
records movement, it is not a camera, it does not shock the animal), and offer
to share what you find. Do not attach anything to an animal without the owner
present and agreeing. Keep sessions short and supervised — this is a
lab-built device on someone's livelihood.

Also ask, with the notebook open:

- Which cows have been sick recently, and how did you notice?
- How do you currently spot a cow in heat?
- What would actually be useful to know on your phone?

That last conversation is worth more to the project than the accelerometer data.
It tells you whether the thing you are building is the thing anyone wants.

## After the visit

```bash
python tools/decode_log.py  /path/COW0001.CLB  -o cow01.csv
python tools/tune_thresholds.py cow01.csv --odr 25 --window 256 --hop 64
```

The second script prints a per-class feature table, the suggested threshold
constants, and the confusion matrix you get with them. Paste the constants into
`demo_config.h` / `collar_config.h` — and keep the printed table in the repo,
because "our constants come from N minutes of labelled data on M cows" is a far
better answer to a judge than any number by itself.
