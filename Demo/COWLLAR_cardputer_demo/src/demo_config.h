/*
 * demo_config.h — COWLLAR live classifier demo (Cardputer Adv)
 * -----------------------------------------------------------------------------
 * Ported from collar_config.h (production). Single source of truth for the
 * demo build. Differences from production are marked [DEMO] with rationale —
 * be ready to explain each one on stage.
 * -----------------------------------------------------------------------------
 */
#ifndef DEMO_CONFIG_H
#define DEMO_CONFIG_H

#include <stdint.h>

/* ---- Acquisition ---------------------------------------------------------- */
#define ACCEL_ODR_HZ        25u     /* same as production                       */
#define WINDOW_LEN          128u    /* [DEMO] 256 in production.                */
/* Rationale: 256/25 Hz = 10.24 s per decision — correct for a cow, deadly for
 * a live demo. 128 -> 5.12 s windows, and with hop 32 the label refreshes
 * every 1.28 s. Band-energy resolution halves (0.195 Hz/bin) but the bands
 * are wide enough that classification is unaffected.                           */
#define WINDOW_HOP          32u     /* [DEMO] sliding window, snappy updates    */
#define FFT_LEN             WINDOW_LEN
#define SPEC_BINS           (FFT_LEN / 2u)

/* ---- Bands (identical to production) -------------------------------------- */
#define BAND_RUMIN_LO_HZ    0.8f    /* rumination / rhythmic chew               */
#define BAND_RUMIN_HI_HZ    2.0f
#define BAND_GAIT_LO_HZ     1.8f    /* walking gait                             */
#define BAND_GAIT_HI_HZ     3.5f

/* ---- Feature vector (identical layout to production) ----------------------- */
/* [0..2] per-axis std  [3] SMA  [4] magnitude ZCR                              */
/* [5] rumination-band E  [6] gait-band E  [7] dominant freq (Hz)               */
#define N_FEATURES          8u

/* ---- Activity classes ------------------------------------------------------ */
typedef enum {
    ACT_REST = 0,
    ACT_RUMINATE,
    ACT_GRAZE,
    ACT_WALK,
    ACT_ACTIVE,
    ACT_COUNT
} activity_t;

static const char *ACT_NAMES[ACT_COUNT] = {
    "REST", "RUMINATE", "GRAZE", "WALK", "ACTIVE"
};

/* ---- Decision-tree thresholds ---------------------------------------------- */
/* [DEMO] Calibrated from measured hand-held data, first run 2026-08-03.
 * The tree STRUCTURE is production; these constants are demo constants.
 *
 *   MEASURED (Cardputer Adv, held in hand):
 *     class      SMA     f0      E_rumin   E_gait
 *     REST       0.041   4.5Hz   0.01      0.02
 *     GRAZE      0.068   3.3Hz   0.16      0.34
 *     RUMINATE   0.149   0.8Hz   0.79      0.50
 *     WALK       0.181   3.3Hz   0.91      2.13
 *
 * RUN 2 (post gravity-fix, all five classes verified):
 *     class      SMA     f0       E_rumin   E_gait
 *     REST       0.002   10.9Hz   0.00      0.00
 *     RUMINATE   0.087   1.0Hz    1.09      0.74
 *     GRAZE      0.113   2.5Hz    0.15      0.32
 *     WALK       0.184   1.4Hz    28.01     5.04
 *     ACTIVE     0.669   3.5Hz    4.69      32.04
 *
 * NOTE (production refinement): band energies are UNNORMALISED, so they scale
 * with motion intensity, not spectral shape — see WALK's E_rumin 28.01 vs the
 * real RUMINATE 1.09 (walk gesture f0 1.4Hz falls inside the rumination band).
 * The tree still separates correctly because the gait gate fires first on
 * absolute energy, but for cow deployment normalise each band by total
 * spectral energy so the feature measures shape, with SMA carrying amplitude.
 *
 * Thresholds placed in the measured gaps, not guessed. Same procedure the
 * real collar needs against labelled cow data.                                 */
#define TH_REST_SMA         0.055f  /* gap: rest 0.041 | graze 0.068           */
#define TH_ACTIVE_SMA       0.40f   /* centred in gap: walk 0.184 | shake 0.669 */
#define TH_WALK_GAIT_E      1.20f   /* gap: rumin-leak 0.50 | walk 2.13        */
#define TH_RUMIN_BAND_E     0.35f   /* gap: graze 0.16 | rumin 0.79 — kept     */

/* ---- Estrus-candidate pre-flag (identical mechanism to production) ---------- */
#define ESTRUS_EWMA_ALPHA   0.05f
#define ESTRUS_SIGMA_K      2.5f
#define ESTRUS_SUSTAIN      8u      /* [DEMO] 18 in production; 8 hops ≈ 10 s
                                       so the audience sees the flag trip       */

/* ---- Health watch (health_watch.h) ----------------------------------------- *
 * The MECHANISM is production. Only the clock is compressed: a block is 15
 * epochs (~19 s) instead of an hour, and circadian slots are switched off
 * because a stage demo has no time of day. Say exactly this if asked — the
 * honest version is more impressive than pretending, and the compression ratio
 * is the only difference.
 *
 *   production : 1 block = 1 hour, 24 slots, sustain 4  -> alerts after ~4 h
 *   demo       : 1 block = ~19 s,  1 slot,   sustain 2  -> alerts after ~40 s
 * ---------------------------------------------------------------------------- */
#define HW_SLOTS            1u
#define HW_EPOCHS_PER_BLOCK 15u
#define HW_ALPHA            0.25f
#define HW_K                1.5f
#define HW_SUSTAIN          2u
#define HW_WARMUP           2u
#define HW_MIN_SIGMA        0.05f

#endif /* DEMO_CONFIG_H */
