/*
 * health_watch.h — COWLLAR two-sided behaviour-deviation monitor
 * -----------------------------------------------------------------------------
 * Answers the question "how would you know the cow is sick?"
 *
 * THE IDEA IN ONE PARAGRAPH
 *   You do not detect disease. You detect that an individual animal has stopped
 *   behaving like herself. The single most useful signal in dairy science is
 *   rumination time: it falls days before a farmer sees anything wrong, across
 *   mastitis, metritis, ketosis and lameness alike. Heat and illness both move
 *   the same two numbers, in opposite directions:
 *
 *       heat     : activity UP,   rumination DOWN,  lasts hours,  self-resolves
 *       illness  : activity DOWN, rumination DOWN,  lasts days,   gets worse
 *
 *   So the same machinery that flags heat flags illness — you just have to stop
 *   throwing away the sign of the deviation, and you have to measure over hours
 *   and days rather than seconds.
 *
 * WHAT THIS IS NOT
 *   Not a diagnosis. It cannot tell mastitis from ketosis from a stone in the
 *   hoof, and it should never be presented as if it could. It says "cow 14 has
 *   ruminated 30% less than her own normal for 8 hours — go look at her." The
 *   farmer and the vet do the rest. Published rumination-based alert systems run
 *   roughly 40-100% sensitivity depending on the disorder at ~85% specificity;
 *   claiming better than that from a student prototype invites exactly the
 *   question you cannot answer.
 *
 * MECHANISM
 *   1. Classify continuously (existing pipeline).
 *   2. Accumulate a TIME BUDGET per block: what fraction of the block was spent
 *      resting / ruminating / grazing / walking / active.
 *   3. Compare each fraction against this cow's own baseline FOR THE SAME TIME
 *      OF DAY. Cattle are strongly diurnal; comparing 03:00 against a
 *      whole-day average manufactures alarms every night.
 *   4. Robust z-score, two-sided, with a sustain requirement.
 *
 *   Baselines are per-animal and learned on-device, so no two collars share
 *   constants and a new cow needs no configuration — just a warm-up period
 *   (~3 days before the alerts mean anything; report LEARNING until then).
 * -----------------------------------------------------------------------------
 */
#ifndef HEALTH_WATCH_H
#define HEALTH_WATCH_H

#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>   /* header is C-clean, not only C++ */

/* ---- Tunables ---------------------------------------------------------------
 * Production values are for a real collar. The demo build overrides them so a
 * person on a stage can see the mechanism work in under a minute.            */
#ifndef HW_SLOTS
#define HW_SLOTS            24u     /* circadian slots per day (1 = disabled)  */
#endif
#ifndef HW_EPOCHS_PER_BLOCK
#define HW_EPOCHS_PER_BLOCK 3600u   /* epochs summed into one block            */
#endif
#ifndef HW_ALPHA
#define HW_ALPHA            0.10f   /* EWMA rate: ~10 blocks memory per slot   */
#endif
#ifndef HW_K
#define HW_K                2.0f    /* z threshold                              */
#endif
#ifndef HW_SUSTAIN
#define HW_SUSTAIN          4u      /* consecutive blocks before alerting       */
#endif
#ifndef HW_WARMUP
#define HW_WARMUP           3u      /* blocks per slot before a slot is trusted */
#endif
#ifndef HW_MIN_SIGMA
#define HW_MIN_SIGMA        0.02f   /* floor: a cow with a very regular routine
                                       has tiny sigma, and without a floor every
                                       ordinary day becomes a 6-sigma event    */
#endif

typedef enum {
    HW_LEARNING = 0,    /* not enough history yet — say so, do not alert       */
    HW_NORMAL,
    HW_HEAT_WATCH,      /* activity up + rumination down, short                */
    HW_HEALTH_WATCH,    /* rumination down + activity flat/down, sustained     */
    HW_STATE_COUNT
} hw_state_t;

static const char *HW_STATE_NAMES[HW_STATE_COUNT] = {
    "LEARNING", "NORMAL", "HEAT WATCH", "HEALTH WATCH"
};

/* One EWMA baseline per (slot, tracked fraction). */
typedef struct {
    float    mu, var;
    uint16_t n;
} hw_base_t;

typedef struct {
    /* accumulation within the current block */
    uint16_t  epochs;
    uint16_t  cnt[8];               /* per activity class                       */

    /* baselines: [slot][0]=rumination frac [1]=activity frac [2]=graze frac    */
    hw_base_t base[HW_SLOTS][3];

    /* last computed block */
    float     frac[3];
    float     z[3];
    uint16_t  run_health, run_heat;
    hw_state_t state;
    uint32_t  blocks;
} health_watch_t;

static inline void hw_init(health_watch_t *h) {
    memset(h, 0, sizeof(*h));
    h->state = HW_LEARNING;
}

/* Feed one classifier decision. cls indexes the activity_t enum:
 *   0 REST  1 RUMINATE  2 GRAZE  3 WALK  4 ACTIVE                             */
static inline void hw_push(health_watch_t *h, uint8_t cls) {
    if (cls < 8) h->cnt[cls]++;
    h->epochs++;
}

static inline bool hw_block_ready(const health_watch_t *h) {
    return h->epochs >= HW_EPOCHS_PER_BLOCK;
}

static inline float hw__z(const hw_base_t *b, float x) {
    if (b->n < HW_WARMUP) return 0.0f;      /* not enough history to judge     */
    float sd = sqrtf(b->var);
    if (sd < HW_MIN_SIGMA) sd = HW_MIN_SIGMA;
    return (x - b->mu) / sd;
}

static inline void hw__learn(hw_base_t *b, float x, float alpha) {
    if (b->n == 0) { b->mu = x; b->var = 0.0f; b->n = 1; return; }
    float d = x - b->mu;
    b->mu  += alpha * d;
    b->var  = (1.0f - alpha) * (b->var + alpha * d * d);
    if (b->n < 0xFFFF) b->n++;
}

/* THE BASELINE MUST NOT LEARN THE ILLNESS.
 *
 * This was a live bug, caught in host simulation before any hardware ran: with
 * a plain EWMA, a cow whose rumination drops and STAYS down raises the alert
 * for two blocks and then the baseline absorbs the new low value, z returns to
 * zero, and the alarm silently clears while the cow is still sick. The alarm
 * learns to accept the emergency. That failure is quiet, plausible-looking, and
 * exactly the kind that survives a demo and kills an animal.
 *
 * Fix: only fold a block into the baseline when it is unremarkable. A block
 * that is already flagged as deviant is evidence, not normality. The escape
 * hatch is HW_RELEARN — if a deviation persists far longer than any illness
 * episode should, it is probably a genuine change in the animal's routine
 * (new feed, new pen, season) and the baseline creeps toward it slowly.       */
#ifndef HW_RELEARN
#define HW_RELEARN          48u     /* blocks of sustained deviation before the
                                       baseline starts accepting the new level */
#endif

static inline void hw__learn_gated(hw_base_t *b, float x, float z, uint16_t run) {
    if (b->n < HW_WARMUP)        hw__learn(b, x, 1.0f / (b->n + 1));  /* seeding */
    else if (fabsf(z) <= HW_K)   hw__learn(b, x, HW_ALPHA);           /* normal  */
    else if (run > HW_RELEARN)   hw__learn(b, x, HW_ALPHA * 0.2f);    /* drift   */
    /* else: deviant block, baseline held */
}

/* Close the block. `slot` is the circadian slot (hour of day, or 0 if HW_SLOTS
 * is 1). Returns the new state. */
static inline hw_state_t hw_close_block(health_watch_t *h, uint8_t slot) {
    if (slot >= HW_SLOTS) slot = 0;
    float n = (float)(h->epochs ? h->epochs : 1);

    h->frac[0] = h->cnt[1] / n;                              /* rumination     */
    h->frac[1] = (h->cnt[3] + h->cnt[4]) / n;                /* walk + active  */
    h->frac[2] = h->cnt[2] / n;                              /* grazing        */

    /* 1. score against the existing baseline (never against itself) */
    for (int i = 0; i < 3; i++)
        h->z[i] = hw__z(&h->base[slot][i], h->frac[i]);

    bool warm = h->base[slot][0].n >= HW_WARMUP;

    /* Illness signature: rumination significantly below this cow's own normal
     * for this hour, while activity is NOT elevated. The activity condition is
     * what separates it from heat — without it, every heat event would also
     * raise a health alarm, and a farmer who gets two alerts for one event
     * stops trusting both. Depressed grazing is corroborating, not required:
     * a cow can go off her cud before she goes off her feed.                  */
    bool rum_down = h->z[0] < -HW_K;
    bool act_up   = h->z[1] >  HW_K;
    bool act_flat = h->z[1] <= 0.5f;

    if (warm && rum_down && act_flat) h->run_health++; else h->run_health = 0;
    if (warm && act_up)               h->run_heat++;   else h->run_heat   = 0;

    if (!warm)                              h->state = HW_LEARNING;
    else if (h->run_health >= HW_SUSTAIN)   h->state = HW_HEALTH_WATCH;
    else if (h->run_heat   >= HW_SUSTAIN)   h->state = HW_HEAT_WATCH;
    else                                    h->state = HW_NORMAL;

    /* 2. only now, and only if this block looked ordinary, learn from it */
    uint16_t run = (h->run_health > h->run_heat) ? h->run_health : h->run_heat;
    for (int i = 0; i < 3; i++)
        hw__learn_gated(&h->base[slot][i], h->frac[i], h->z[i], run);

    h->blocks++;
    h->epochs = 0;
    memset(h->cnt, 0, sizeof(h->cnt));
    return h->state;
}

/* Severity 0-3, for choosing how loudly to tell the farmer. */
static inline uint8_t hw_severity(const health_watch_t *h) {
    if (h->state != HW_HEALTH_WATCH) return 0;
    float m = -h->z[0];
    if (m > 4.0f && h->run_health >= 2 * HW_SUSTAIN) return 3;
    if (m > 3.0f) return 2;
    return 1;
}

#endif /* HEALTH_WATCH_H */
