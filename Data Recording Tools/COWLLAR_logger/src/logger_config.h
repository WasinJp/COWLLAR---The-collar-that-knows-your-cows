/*
 * logger_config.h — COWLLAR field data logger (dual-IMU revision)
 * -----------------------------------------------------------------------------
 * One rule governs this file: RECORD MORE THAN YOU THINK YOU NEED.
 * You can always throw data away offline. You cannot go back to the farm and
 * re-record at a higher rate, or with a sensor you left in the lab.
 *
 * WHY TWO IMUs
 *   The Cardputer's internal IMU is a Bosch BMI270. The collar will carry an
 *   ST LSM6DS3TR-C on the Grove port. Fitting thresholds on one part and
 *   deploying on the other means every constant needs re-checking anyway — so
 *   log BOTH at once, from the same window, under the same labels, and you get
 *   the transfer function for free instead of guessing at it later.
 *
 *   The external sensor is the one that matters: it is the collar's sensor, and
 *   on a Grove cable it can be taped exactly where the collar will sit while the
 *   81 g Cardputer rides somewhere convenient on the strap. Thresholds should be
 *   fitted to the EXTERNAL sensor. The internal one is the control.
 * -----------------------------------------------------------------------------
 */
#ifndef LOGGER_CONFIG_H
#define LOGGER_CONFIG_H

#include <stdint.h>

/* ---- Acquisition ----------------------------------------------------------- */
#define LOG_ODR_HZ          50u     /* 2x the collar's rate. Decimate offline.
                                       Lets us TEST whether 25 Hz is enough
                                       instead of assuming it.                  */

/* ---- Fixed-point scaling (int16 on the card, float offline) -----------------
 * Both sensors are read as floats in g / dps and stored on ONE common scale, so
 * the decoder needs no per-sensor special cases and the two streams are directly
 * comparable without unit arithmetic.                                          */
#define ACC_LSB_PER_G       4096.0f /* +/- 8 g range, 0.24 mg resolution         */
#define GYR_LSB_PER_DPS     16.4f   /* +/- ~1998 dps                             */

/* ---- External IMU: LSM6DS3TR-C on the Grove (HY2.0-4P) port ------------------
 * I2C address is 0x6A or 0x6B depending on the module's SDO pull. The firmware
 * tries both and prints what it found, because a wrong guess looks exactly like
 * a broken cable and will otherwise cost an afternoon.
 *
 * The Grove pins differ between Cardputer revisions, so the firmware scans a
 * candidate list and reports the pair that worked. Check the boot screen.     */
#define EXT_I2C_CANDIDATES  { {2,1}, {1,2}, {13,15}, {15,13}, {8,9}, {9,8} }
#define EXT_I2C_HZ          400000u

#define LSM_ADDR_A          0x6A
#define LSM_ADDR_B          0x6B
#define LSM_WHO_AM_I        0x0F
#define LSM_ID_TRC          0x6A    /* LSM6DS3TR-C */
#define LSM_ID_DS3          0x69    /* plain LSM6DS3 — also fine               */
#define LSM_CTRL1_XL        0x10
#define LSM_CTRL2_G         0x11
#define LSM_CTRL3_C         0x12
#define LSM_OUTX_L_G        0x22
#define LSM_OUTX_L_XL       0x28

/* 104 Hz, +/-4 g. Deliberately 2x our 50 Hz sample timer: oversampling the
 * sensor means every read returns fresh data instead of a value up to a whole
 * sample period stale, which would smear the phase relationship between the two
 * IMUs and quietly ruin the comparison this build exists to make.             */
#define LSM_CTRL1_XL_VAL    0x48    /* ODR 104 Hz | FS +/-4 g                   */
#define LSM_CTRL2_G_VAL     0x4C    /* ODR 104 Hz | FS +/-2000 dps              */
#define LSM_CTRL3_C_VAL     0x44    /* BDU (no torn samples) | IF_INC           */
#define LSM_ACC_MG_PER_LSB  0.122f  /* +/-4 g                                   */
#define LSM_GYR_MDPS_PER_LSB 70.0f  /* +/-2000 dps                              */

/* ---- Storage --------------------------------------------------------------- */
#define SD_SPI_SCK          40
#define SD_SPI_MISO         39
#define SD_SPI_MOSI         14
#define SD_CS_CANDIDATES    { 12, 11, 10, 5, 4 }

#define WRITE_BLOCK_BYTES   4096u
#define FLUSH_EVERY_MS      10000u  /* a dead battery costs <=10 s of data,
                                       not the whole session                    */

/* ---- Label set -------------------------------------------------------------- *
 * What a HUMAN presses while watching the cow. NOT the classifier's classes —
 * LYING and STANDING are separate here even though the collar lumps both into
 * REST. Record the finer distinction; you can always merge offline, and posture
 * matters for illness (sick cows lie more), so future-you will want it.
 *
 * Press the number key when the behaviour STARTS. It stays until you press
 * another. Press 0 the moment you are unsure or lose sight of the cow — an
 * honest UNKNOWN is worth far more than a guess, because a wrong label does not
 * merely add noise, it moves the fitted threshold.
 * ---------------------------------------------------------------------------- */
typedef enum {
    LBL_UNKNOWN = 0,
    LBL_LYING,          /* 1 - lying down, not chewing                          */
    LBL_STANDING,       /* 2 - standing still, not chewing                      */
    LBL_RUMINATE,       /* 3 - chewing cud (rhythmic jaw, head level/up)        */
    LBL_GRAZE,          /* 4 - head down, eating grass or at the trough         */
    LBL_WALK,           /* 5 - walking                                          */
    LBL_DRINK,          /* 6 - drinking                                         */
    LBL_OTHER,          /* 7 - scratching, mounting, running, handled           */
    LBL_COUNT
} label_t;

static const char *LBL_NAMES[LBL_COUNT] = {
    "UNKNOWN", "LYING", "STANDING", "RUMINATE", "GRAZE", "WALK", "DRINK", "OTHER"
};

/* ---- On-card record format --------------------------------------------------
 * File header, then a flat array of records. 24 bytes each.
 *   50 Hz * 24 B = 1.2 kB/s = 4.3 MB/hour = 104 MB/day.
 *   An 8 GB card holds ~2.5 months. Storage is never the limit here; battery
 *   and your patience for labelling are.
 * ---------------------------------------------------------------------------- */
#define COWLOG_MAGIC        "COWLOG03"      /* 8 bytes, no NUL                   */

#define HDR_F_HAS_INT       0x01
#define HDR_F_HAS_EXT       0x02

typedef struct __attribute__((packed)) {
    char     magic[8];      /* "COWLOG03"                                       */
    uint16_t odr_hz;
    uint16_t rec_bytes;     /* sizeof(cowlog_rec_t) — parser self-check         */
    float    acc_lsb_per_g;
    float    gyr_lsb_per_dps;
    uint32_t session_id;
    uint32_t start_epoch;   /* unix seconds if RTC valid, else 0                */
    uint32_t start_millis;
    uint8_t  sensors;       /* HDR_F_HAS_INT | HDR_F_HAS_EXT                    */
    char     int_part[11];  /* "BMI270"                                         */
    char     ext_part[12];  /* "LSM6DS3TR-C"                                    */
    char     mount[16];     /* where the EXTERNAL sensor was taped              */
    char     cow[16];       /* ear-tag number                                   */
    uint8_t  reserved[8];
} cowlog_hdr_t;             /* 96 bytes                                          */

typedef struct __attribute__((packed)) {
    int16_t  iax, iay, iaz; /* internal BMI270 accel   (0 if absent)            */
    int16_t  eax, eay, eaz; /* external LSM6DS3 accel  (0 if absent)            */
    int16_t  egx, egy, egz; /* external LSM6DS3 gyro   (0 if absent)            */
    uint8_t  label;
    uint8_t  flags;         /* bit0 sync marker, bit1 external read failed      */
    uint16_t dt_ms;         /* ms since previous sample — jitter evidence. If
                               this is not ~20 ms, the timing is lying to you
                               and every frequency feature is suspect.          */
    uint16_t seq;           /* low 16 bits of the sample counter — catches
                               dropped writes that dt_ms alone would not        */
} cowlog_rec_t;             /* 24 bytes                                          */

#define FLAG_MARKER         0x01
#define FLAG_EXT_FAIL       0x02

#endif /* LOGGER_CONFIG_H */
