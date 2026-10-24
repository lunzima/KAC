/* kac_encode.c -- KAC encoder.
 * Input is buffered until a chunk fills, then emitted; tier and channel form
 * are measured over that whole chunk unless both are pinned. Quantiser choices
 * use error against the unclamped reconstruction; kac_step clamps for output. */
#include "kac.h"

#include <string.h>

#define KAC_PROBE_POINTS 16u /* positions the scalefactor estimate is read at */
/* Widest scalefactor walk: how many of the sixteen are compared around the
 * estimate. Levels 0 to 2 use it and lower levels step down. Width changes which
 * local optimum the greedy search enters, not how much of the space is seen. */
#define KAC_PROBE_CAND 9u
#define KAC_STATE_FRESH 0
#define KAC_STATE_READY 1
#define KAC_STATE_FLUSHED 2

/* The -V quality ladder: rung n aims at kac_v_rung_mdb[n], with a terminus
 * entry after rung 9 for indices 9.000..9.999. On the eight-track reference set
 * (whole 60 s tracks, 44.1 kHz stereo, -q 3) T6 reaches a median 63.39 dB, so
 * rung 0 sits 0.39 dB below it; other material need not land on the rung. */
#define KAC_V_P5_MDB 63000
/* How far the achieved rate may sit from the goal before the loop corrects, in
 * thousandths. On the eight-track set the worst file is 0.287 % off and the
 * per-goal median 0.059-0.145 %; the delivered level moves at most 0.05 dB. */
#define KAC_ABR_TOL_PERMILLE 2
/* The smallest spread between measured tier readings that makes a block usable
 * as the ABR start. A block whose quietest reading is already at the no-error
 * sentinel is silence, and a flatter ladder names no tier. */
#define KAC_ABR_SEED_SPREAD 1000
/* Internal ABR startup rungs beyond the CLI -V ladder, as (rate, target) in
 * mdB: the eight-track delivered level less that track's end rung offset,
 * averaged on the visible ladder's 25 mdB rate / 250 mdB level grids. */
typedef struct {
    int32_t rate_md, target_mdb;
} kac_abr_rung;
static const kac_abr_rung kac_abr_low[] = {
    {1067, 23500}, {1333, 26000}, {1600, 28000}, {2000, 32000}};
static const kac_abr_rung kac_abr_high[] = {
    {6175, 63000}, {6400, 64000}, {7200, 67000}, {8000, 69250}};

/* The -V outer loop moves a level, so no slope is needed: a hundredth of the
 * shortfall per block, slower than ABR because a level is tracked rather than
 * integrated. */
#define KAC_V_STEP 100
/* Below its rung a file is corrected at once. Above it, no correction: the pick
 * took the cheapest tier that passes, so overshoot is bounded by one ladder
 * step, and a rate ceiling is what --abr is for. */
#define KAC_V_TOL_UP 100
/* Narrowest aim window for a block with no measured band spread. */
#define KAC_V_SLACK_MIN 300

/* Last integer rung; KAC_V_NTERM is the end of the ladder in thousandths and
 * sits below the next integer so that no eleventh rung is needed. */
#define KAC_V_NMAX 9
#define KAC_V_NTERM 9999

/* Rung targets in milli-dB, not evenly spaced: each is anchored on a measured
 * listening ladder whose steps are linear in listening while distortion is not.
 * Only anchors that keep the input sample rate are usable. Rung 0 is the top;
 * the terminus repeats the bottom anchor at a more optimistic percentile. */
static const int32_t kac_v_rung_mdb[KAC_V_NMAX + 2] = {
    KAC_V_P5_MDB, 58750, 55250, 51500, 49000, 45000, 42750, 40750, 37750, 36000, 33750};

/* Position of each entry above on the -V index, in thousandths. */
static const int16_t kac_v_rung_pos[KAC_V_NMAX + 2] = {0,    1000, 2000, 3000, 4000,       5000,
                                                       6000, 7000, 8000, 9000, KAC_V_NTERM};

/* Nominal rate of each rung, in milli-bit per sample: the median over the
 * calibration set, rounded to even steps. Achieved rate depends on material. */
static const int32_t kac_v_rate_mb[KAC_V_NMAX + 2] = {5950, 5250, 4650, 4125, 3650, 3225,
                                                      2875, 2625, 2425, 2300, 2175};

const char *kac_version(void)
{
    return "KAC 1.0";
}

void kac_init(kac_t *restrict g)
{
    uint8_t *p = (uint8_t *)g;
    size_t   i;
    for (i = 0; i < sizeof *g; i++)
        p[i] = 0;
    g->channels      = 2;
    g->samplerate    = 44100;
    g->tier          = KAC_TIER_AUTO;
    g->chunk_samples = KAC_CHUNK_SAMPLES;
    /* The searches the run gives up (KAC_QUALITY_DEFAULT, kac_internal.h) and the
     * form every block is sent in (KAC_MS_AUTO). */
    g->quality = KAC_QUALITY_DEFAULT;
    g->ms_mode = KAC_MS_AUTO;
    g->report  = 1;
    /* Default: per-block VBR at rung KAC_DEFAULT_V over the whole ladder. Any
     * selector clears it: a pinned tier or rate sets KAC_VBR_OFF, kac_set_vbr()
     * overwrites the rung and opens the window. */
    g->vbr_mode   = KAC_VBR_V;
    g->vbr_n      = KAC_DEFAULT_V * 1000;
    g->ladder     = 1;
    g->vbr_target = kac_v_rung_mdb[KAC_DEFAULT_V];
    g->tier_lo    = KAC_TIER_MIN;
    g->tier_hi    = KAC_TIER_MAX;
}

void kac_close(kac_t *restrict g)
{
    kac_init(g);
}

int kac_set_num_channels(kac_t *restrict g, int channels)
{
    if (!g || g->state != KAC_STATE_FRESH)
        return KAC_E_ARG;
    if (channels < 1 || channels > KAC_MAX_CHANNELS)
        return KAC_E_ARG;
    g->channels = channels;
    return 0;
}

int kac_set_in_samplerate(kac_t *restrict g, int hz)
{
    if (!g || g->state != KAC_STATE_FRESH || hz <= 0)
        return KAC_E_ARG;
    g->samplerate = hz;
    return 0;
}

/* A rate or level in thousandths, saturated before the cast. The public API
 * takes a double, and a value outside int32 would make the cast undefined. */
static int32_t kac_milli_from_double(double v)
{
    double x = v * 1000.0 + 0.5;

    if (x >= 2147483647.0)
        return INT32_MAX;
    if (x <= -2147483648.0)
        return INT32_MIN;
    return (int32_t)x;
}

int kac_set_brate(kac_t *restrict g, double bits_per_sample)
{
    if (!g || g->state != KAC_STATE_FRESH || bits_per_sample <= 0.0)
        return KAC_E_ARG;
    g->brate    = bits_per_sample;
    g->tier     = kac_tier_for_brate(bits_per_sample);
    g->tier_lo  = g->tier;
    g->tier_hi  = g->tier;
    g->vbr_mode = KAC_VBR_OFF; /* last selector wins, as with -t below */
    return 0;
}

int kac_set_tier(kac_t *restrict g, int tier)
{
    if (!g || g->state != KAC_STATE_FRESH)
        return KAC_E_ARG;
    if (tier < KAC_TIER_MIN || tier > KAC_TIER_MAX)
        return KAC_E_ARG;
    g->tier     = tier;
    g->tier_lo  = tier;
    g->tier_hi  = tier;
    g->brate    = 0.0;
    g->vbr_mode = KAC_VBR_OFF;
    return 0;
}

int kac_set_tier_limits(kac_t *restrict g, int lo, int hi)
{
    if (!g || g->state != KAC_STATE_FRESH)
        return KAC_E_ARG;
    if (lo < KAC_TIER_MIN || hi > KAC_TIER_MAX || lo > hi)
        return KAC_E_ARG;
    g->tier_lo = lo;
    g->tier_hi = hi;
    return 0;
}

int kac_set_init_state(kac_t *restrict g, const kac_snap *snaps)
{
    unsigned c;
    if (!g || !snaps || g->state != KAC_STATE_FRESH)
        return KAC_E_ARG;
    for (c = 0; c < (unsigned)KAC_MAX_CHANNELS; c++)
        g->init_snap[c] = snaps[c];
    g->init_snap_valid = 1;
    return 0;
}

int kac_get_state(const kac_t *restrict g, kac_snap *snaps)
{
    unsigned c;
    int      i;
    if (!g || !snaps)
        return KAC_E_ARG;
    for (c = 0; c < (unsigned)g->channels; c++) {
        for (i = 0; i < 4; i++) {
            snaps[c].h[i] = (int16_t)g->ch[c].h[i];
            snaps[c].w[i] = (int16_t)g->ch[c].w[i];
        }
    }
    return 0;
}

int kac_set_loop(kac_t *restrict g, uint32_t start, uint32_t end)
{
    if (!g || g->state != KAC_STATE_FRESH)
        return KAC_E_ARG;
    if (end <= start)
        return KAC_E_ARG;
    g->loop_start = start;
    g->loop_end   = end;
    return 0;
}

int kac_set_index(kac_t *restrict g, uint32_t stride, uint8_t *store, size_t bytes)
{
    if (!g || g->state != KAC_STATE_FRESH)
        return KAC_E_ARG;
    if (stride == 0 || !store || bytes == 0)
        return KAC_E_ARG;
    g->stride            = stride;
    g->index_store       = store;
    g->index_store_bytes = bytes;
    return 0;
}

int kac_set_index_offsets(kac_t *restrict g, uint32_t *store, size_t entries)
{
    if (!g || g->state != KAC_STATE_FRESH)
        return KAC_E_ARG;
    g->index_off         = store;
    g->index_off_entries = store ? entries : 0u;
    return 0;
}

/* The chunk quantum: a pinned tier uses its slice, a per-block run the least
 * count that is whole slices at every tier. */
static unsigned chunk_quantum(const kac_t *restrict g)
{
    return (g->vbr_mode == KAC_VBR_OFF && g->tier_lo == g->tier_hi) ? g->n : KAC_CHUNK_LCM;
}

/* The chunk nearest ms in whole quanta, at most half a millisecond past it.
 * Returns 0 when one quantum alone is already past the cap. */
static unsigned chunk_for_ms(const kac_t *restrict g, unsigned ms)
{
    const unsigned quantum = chunk_quantum(g);
    uint64_t       target  = (uint64_t)ms * (uint64_t)g->samplerate / 1000u;
    uint64_t       cap     = target + (uint64_t)g->samplerate / 2000u;
    uint64_t       k       = (target + quantum / 2u) / quantum;
    uint64_t       kmax    = KAC_CHUNK_SAMPLES / quantum;

    if (k < 1u)
        k = 1u;
    if (k > kmax)
        k = kmax;
    while (k > 1u && k * quantum > cap)
        k--;
    if (k * quantum > cap)
        return 0u;
    return (unsigned)(k * quantum);
}

int kac_set_stream(kac_t *restrict g, int enable)
{
    if (!g || g->state != KAC_STATE_READY)
        return KAC_E_ARG;
    /* Once anything has been emitted the handle has committed to one form and
     * cannot switch: a stream has no placeholder to remove and a file's
     * placeholder has already been spent or deliberately skipped. */
    if (g->prefix_done || g->total_in != 0u)
        return KAC_E_ARG;
    if (!enable) {
        g->stream        = 0;
        g->chunk_samples = kac_block_for_rate((unsigned)g->samplerate);
        return 0;
    }
    /* A stream is blocks with no length in front, so the two file-only things
     * are refused: a loop region needs the reader to hold the block it wraps
     * to, and the seek index is a trailer. */
    if (g->loop_end > g->loop_start || g->stride != 0)
        return KAC_E_ARG;
    g->stream = 1;
    return 0;
}

int kac_set_chunk_ms(kac_t *restrict g, unsigned ms)
{
    unsigned c;

    if (!g || g->state != KAC_STATE_READY)
        return KAC_E_ARG;
    if (!g->stream)
        return KAC_E_ARG;
    if (g->samplerate <= 0 || g->n == 0u)
        return KAC_E_ARG;
    if (g->total_in != 0u || g->blk_count != 0u || g->data_bytes != 0u)
        return KAC_E_ARG;
    c = chunk_for_ms(g, ms);
    if (c == 0u)
        return KAC_E_ARG;
    g->chunk_samples = c;
    return 0;
}

unsigned kac_get_chunk_samples(const kac_t *restrict g)
{
    return g ? g->chunk_samples : 0u;
}

/* Piecewise-linear read of an eleven-entry column at an index in thousandths.
 * Every step is one rung wide except the last, which is 0.999, so the terminal
 * entry is reached exactly at KAC_V_NTERM. Clamping is the caller's job. */
static int32_t kac_v_col_at(const int32_t *col, int n1000)
{
    int i;

    for (i = 0; i <= KAC_V_NMAX; i++) {
        int32_t a = kac_v_rung_pos[i];
        int32_t b = kac_v_rung_pos[i + 1];

        if (n1000 <= b) {
            if (n1000 <= a)
                return col[i];
            return col[i] + (col[i + 1] - col[i]) * (n1000 - a) / (b - a);
        }
    }
    return col[KAC_V_NMAX + 1];
}

static int32_t kac_v_tier_mdb1000(int n1000)
{
    return kac_v_col_at(kac_v_rung_mdb, n1000);
}

static int32_t kac_v_tier_mdb(int n)
{
    if (n < 0)
        n = 0;
    if (n > KAC_V_NMAX)
        n = KAC_V_NMAX;
    return kac_v_rung_mdb[n];
}

double kac_v_tier_psnr(int n)
{
    return (double)kac_v_tier_mdb(n) / 1000.0;
}

/* What a tier costs, in milli-bit per sample: its slice spends 64 bits over n
 * samples. */
static int32_t kac_tier_rate_mb(int t)
{
    return (int32_t)(64u * 1000u / kac_tiers[t - 1].n);
}

/* The two rate lookups over a span of tiers: the fixed-tier pin uses T2..T6,
 * and a per-block window uses the whole ladder. */
static int kac_tier_for_brate_span_mb(int32_t mbps, int lo, int hi)
{
    int t;

    for (t = hi; t > lo; t--) {
        if (mbps >= kac_tier_rate_mb(t))
            return t;
    }
    return lo;
}

static int kac_tier_ceil_brate_span_mb(int32_t mbps, int lo, int hi)
{
    int t;

    for (t = lo; t < hi; t++) {
        if (mbps <= kac_tier_rate_mb(t))
            return t;
    }
    return hi;
}

/* Highest pinnable tier whose rate is at most mbps; clamps to T2..T6. T1 and
 * T7 are block choices, not pinned ones. */
static int kac_tier_for_brate_mb(int32_t mbps)
{
    return kac_tier_for_brate_span_mb(mbps, KAC_TIER_FIXED_MIN, KAC_TIER_FIXED_MAX);
}

/* Mirror lookup: the lowest pinnable tier whose rate is at least mbps. The two
 * bounds can cross between tiers, which kac_set_tier_limits() refuses. */
static int kac_tier_ceil_brate_mb(int32_t mbps)
{
    return kac_tier_ceil_brate_span_mb(mbps, KAC_TIER_FIXED_MIN, KAC_TIER_FIXED_MAX);
}

/* The nominal rate of a rung, in milli-bit per sample. What reads it is the
 * tier a run starts on. */
static int32_t kac_v_nominal_rate_mb(int n1000)
{
    return kac_v_col_at(kac_v_rate_mb, n1000);
}

static unsigned kac_v_nominal_tier1000(int n1000)
{
    return (unsigned)kac_tier_for_brate_mb(kac_v_nominal_rate_mb(n1000));
}

unsigned kac_v_nominal_tier(int n)
{
    if (n < 0)
        n = 0;
    if (n > KAC_V_NMAX)
        n = KAC_V_NMAX;
    return kac_v_nominal_tier1000(n * 1000);
}

/* Which rung's nominal rate is the nearest fit for this goal rate: the first
 * rung from the top whose nominal rate the goal reaches, so a goal between two
 * rungs opens on the cheaper one. Answers in thousandths. */
static int kac_vbr_rung_for_rate_mb(int32_t mbps)
{
    int i;

    for (i = 0; i <= KAC_V_NMAX; i++) {
        if (mbps >= kac_v_rate_mb[i])
            return i * 1000;
    }
    return KAC_V_NTERM;
}

/* The ABR startup target for a goal rate: the visible ladder inside its span,
 * and the internal rungs beyond it. A goal past an end clamps to that end. */
static int32_t kac_abr_start_mdb(int32_t mbps)
{
    static const kac_abr_rung join_lo = {2175, 33750};
    static const kac_abr_rung join_hi = {5950, 63000};
    const kac_abr_rung       *a, *b;
    unsigned                  n, i;

    if (mbps <= kac_abr_low[0].rate_md)
        return kac_abr_low[0].target_mdb;
    if (mbps >= kac_abr_high[(sizeof kac_abr_high / sizeof kac_abr_high[0]) - 1].rate_md)
        return kac_abr_high[(sizeof kac_abr_high / sizeof kac_abr_high[0]) - 1].target_mdb;
    if (mbps < join_lo.rate_md) {
        a = kac_abr_low;
        n = (unsigned)(sizeof kac_abr_low / sizeof kac_abr_low[0]);
        b = &join_lo;
    } else if (mbps > join_hi.rate_md) {
        a = &join_hi;
        n = (unsigned)(sizeof kac_abr_high / sizeof kac_abr_high[0]);
        b = kac_abr_high;
    } else {
        return kac_v_tier_mdb1000(kac_vbr_rung_for_rate_mb(mbps));
    }

    if (mbps > join_hi.rate_md) {
        for (i = 0; i < n; i++) {
            if (mbps <= kac_abr_high[i].rate_md) {
                a = (i == 0) ? &join_hi : &kac_abr_high[i - 1];
                b = &kac_abr_high[i];
                break;
            }
        }
    } else {
        for (i = 0; i < n; i++) {
            if (mbps <= ((i + 1 < n) ? kac_abr_low[i + 1].rate_md : join_lo.rate_md)) {
                a = &kac_abr_low[i];
                b = (i + 1 < n) ? &kac_abr_low[i + 1] : &join_lo;
                break;
            }
        }
    }
    return a->target_mdb +
           (int32_t)(((int64_t)(mbps - a->rate_md) * (b->target_mdb - a->target_mdb)) /
                     (b->rate_md - a->rate_md));
}

int kac_set_vbr(kac_t *restrict g, int mode, double target)
{
    if (!g || g->state != KAC_STATE_FRESH)
        return KAC_E_ARG;
    if (mode != KAC_VBR_ABR && mode != KAC_VBR_V)
        return KAC_E_ARG;
    if (mode == KAC_VBR_ABR) {
        int32_t mbps;

        if (!(target > 0.0))
            return KAC_E_ARG;
        /* A goal outside the ladder is held at the nearest end rather than
         * refused; kac_get_vbr_target() reports the held value. */
        mbps = kac_milli_from_double(target);
        if (mbps > kac_tier_rate_mb(KAC_TIER_MAX))
            mbps = kac_tier_rate_mb(KAC_TIER_MAX);
        if (mbps < kac_tier_rate_mb(KAC_TIER_MIN))
            mbps = kac_tier_rate_mb(KAC_TIER_MIN);
        /* Seed the inner rule at the rung whose nominal rate matches the goal,
         * so the first block is aimed somewhere sensible before the outer loop
         * has anything to correct. */
        g->vbr_n = kac_vbr_rung_for_rate_mb(mbps);
        /* Inside the visible ladder the start is its rung; beyond it the
         * internal rungs carry the measured rate-to-level curve. */
        if (mbps > kac_v_rate_mb[0] || mbps < kac_v_rate_mb[KAC_V_NMAX + 1])
            g->vbr_psnr = kac_abr_start_mdb(mbps);
        else
            g->vbr_psnr = kac_v_tier_mdb1000(g->vbr_n);
        if (g->vbr_psnr > KAC_AIM_MAX_MDB)
            g->vbr_psnr = KAC_AIM_MAX_MDB;
        g->vbr_target = mbps; /* milli-bit/sample */
        g->ladder     = 0;
    } else {
        /* -V n: the index runs to KAC_V_NTERM thousandths with three decimals,
         * so -V 3.5 sits between rung 3 and rung 4 and -V 9.999 is the last
         * rung. Outside 0..9.999 is an argument error, not a clamp. */
        int n;

        if (target < 0.0 || target > (double)KAC_V_NTERM / 1000.0)
            return KAC_E_ARG;
        n             = kac_milli_from_double(target);
        g->vbr_n      = n;
        g->vbr_target = kac_v_tier_mdb1000(n);
        g->vbr_psnr   = g->vbr_target;
        g->ladder     = 1;
    }
    g->vbr_mode = mode;
    g->tier     = KAC_TIER_AUTO;
    g->brate    = 0.0;
    g->tier_lo  = KAC_TIER_MIN; /* a per-block selector opens the window again */
    g->tier_hi  = KAC_TIER_MAX;
    return 0;
}

int kac_set_vbr_target_psnr(kac_t *restrict g, double target_psnr)
{
    int32_t mdb;
    int     n, i;

    if (!g || g->state != KAC_STATE_FRESH)
        return KAC_E_ARG;
    if (!(target_psnr > 0.0) || target_psnr > (double)KAC_AIM_MAX_MDB / 1000.0)
        return KAC_E_ARG;

    /* Which rung this target stands for, so the nominal tier reads exactly as
     * it would for -V n. The rung saturates at the ends of the ladder, where
     * the target has no rung of its own; the target itself stays as given,
     * since it is what the outer loop steers on. */
    mdb = kac_milli_from_double(target_psnr);
    n   = KAC_V_NMAX + 1;
    for (i = 0; i <= KAC_V_NMAX + 1; i++) {
        if (mdb >= kac_v_rung_mdb[i]) {
            n = i;
            break;
        }
    }

    g->vbr_mode   = KAC_VBR_V;
    g->vbr_n      = (n <= KAC_V_NMAX) ? n * 1000 : KAC_V_NTERM;
    g->vbr_target = mdb;
    g->vbr_psnr   = mdb;
    g->ladder     = 0; /* an explicit level, not a rung */
    g->tier       = KAC_TIER_AUTO;
    g->brate      = 0.0;
    g->tier_lo    = KAC_TIER_MIN; /* a per-block selector opens the window again */
    g->tier_hi    = KAC_TIER_MAX;
    return 0;
}

/* Tier from bit rate: the rate is a pure function of the tier, so this is a
 * table lookup with no trial encoding. */
int kac_tier_for_brate(double bits_per_sample)
{
    return kac_tier_for_brate_mb(kac_milli_from_double(bits_per_sample));
}

int kac_tier_ceil_brate(double bits_per_sample)
{
    return kac_tier_ceil_brate_mb(kac_milli_from_double(bits_per_sample));
}

int kac_tier_for_brate_vbr(double bits_per_sample)
{
    return kac_tier_for_brate_span_mb(kac_milli_from_double(bits_per_sample), KAC_TIER_MIN,
                                      KAC_TIER_MAX);
}

int kac_tier_ceil_brate_vbr(double bits_per_sample)
{
    return kac_tier_ceil_brate_span_mb(kac_milli_from_double(bits_per_sample), KAC_TIER_MIN,
                                       KAC_TIER_MAX);
}

/* Row maxima of one tier's table. Every tier's row is filled once from
 * kac_init_params and never rebuilt: a block searches several tiers' rows, and
 * the handle copies used for measurement share them unchanged. */
static void rowmax_fill(kac_t *restrict g, unsigned ti)
{
    const int16_t *dqt = kac_pick_dqt(ti + 1u);
    unsigned       w   = kac_tiers[ti].w;
    int            s, k;

    for (s = 0; s < 16; s++) {
        int32_t mx = 0;
        for (k = 0; k < (1 << (int)w); k++) {
            int32_t v = dqt[((unsigned)s << w) + (unsigned)k];
            if (v < 0)
                v = -v;
            if (v > mx)
                mx = v;
        }
        g->rowmax[ti][s] = mx;
    }
}

/* Point the slice tables at one tier. Every block boundary calls this;
 * kac_init_params() calls it once so that the handle has tables to report
 * through before the first block. */
static int kac_use_tier(kac_t *restrict g, int tier)
{
    unsigned ti;

    if (tier < KAC_TIER_MIN || tier > KAC_TIER_MAX)
        return KAC_E_ARG;
    /* The tables always move and cur_tier records their tier: this runs on
     * handle copies whose tables may belong to another tier. */
    ti          = (unsigned)tier - 1u;
    g->w        = kac_tiers[ti].w;
    g->n        = kac_tiers[ti].n;
    g->dqt      = kac_pick_dqt((unsigned)tier);
    g->cur_tier = tier;
    return 0;
}

int kac_init_params(kac_t *restrict g)
{
    uint32_t k;
    unsigned ti;

    if (!g)
        return KAC_E_ARG;
    if (g->state != KAC_STATE_FRESH)
        return KAC_E_ARG;
    if (g->channels < 1 || g->channels > KAC_MAX_CHANNELS)
        return KAC_E_ARG;
    if (g->samplerate <= 0)
        return KAC_E_ARG;
    if (g->tier == KAC_TIER_AUTO) {
        if (g->vbr_mode != KAC_VBR_OFF) {
            /* A per-block run picks its tier per block; this is only the legal
             * tier kac_use_tier below needs to point the tables at. The header
             * takes block 0's own choice instead. */
            g->tier = (int)kac_v_nominal_tier1000(g->vbr_n);
        } else if (g->brate > 0.0) {
            g->tier    = kac_tier_for_brate(g->brate);
            g->tier_lo = g->tier;
            g->tier_hi = g->tier;
        } else {
            /* Nothing was selected at all: only a handle zeroed instead of
             * kac_init() reaches this arm, so it adopts the same default rather
             * than a second hard-coded tier. */
            g->vbr_mode   = KAC_VBR_V;
            g->vbr_n      = KAC_DEFAULT_V * 1000;
            g->vbr_target = kac_v_rung_mdb[KAC_DEFAULT_V];
            g->tier       = (int)kac_v_nominal_tier(KAC_DEFAULT_V);
            g->tier_lo    = KAC_TIER_MIN;
            g->tier_hi    = KAC_TIER_MAX;
        }
    }
    if (g->tier < KAC_TIER_MIN || g->tier > KAC_TIER_MAX)
        return KAC_E_ARG;
    if (g->tier_lo < KAC_TIER_MIN || g->tier_hi > KAC_TIER_MAX || g->tier_lo > g->tier_hi)
        return KAC_E_ARG;
    if (kac_use_tier(g, g->tier) != 0)
        return KAC_E_ARG;
    for (ti = 0; ti < KAC_TIER_MAX; ti++)
        rowmax_fill(g, ti);

    if (g->stride != 0) {
        /* KAC_SPEC.md Section 3.1, the IndexStride row: an index point is a
         * block boundary, so the stride is a multiple of the block length. */
        if (g->stride % kac_block_for_rate((unsigned)g->samplerate) != 0u)
            return KAC_E_ARG;
        if (!g->index_off)
            return KAC_E_ARG; /* it cannot be derived: KAC_SPEC.md Section 3.4 */
    }
    for (k = 0; k < KAC_MAX_CHANNELS; k++) {
        int i;
        kac_ch_init(&g->ch[k]);
        /* A caller-supplied initial state replaces the default one, so the
         * encoder and a decoder reading INIT_STATE start from the same place.
         * Weights are saturated the same way kac_snap_load() saturates them. */
        if (g->init_snap_valid) {
            for (i = 0; i < 4; i++) {
                g->ch[k].h[i] = g->init_snap[k].h[i];
                g->ch[k].w[i] = kac_sat_w(g->init_snap[k].w[i]);
            }
        }
    }
    g->loop_snap_valid = 0;
    g->fill            = 0;
    /* The file block at this rate; a stream keeps it as its default chunk. */
    g->chunk_samples = kac_block_for_rate((unsigned)g->samplerate);
    g->pos           = 0;
    g->total_in      = 0;
    g->prefix_done   = 0;
    g->data_bytes    = 0;
    g->blk_count     = 0;
    g->peak          = 0;
    g->sse           = 0;
    g->clip          = 0;
    g->pred_over     = 0;
    g->w_limit       = 0;
    {
        /* Resolve the quality level into search settings. The handle is public,
         * so an out-of-range level reads as the widest one. Only level 0 codes a
         * block per tier; the rest estimate, and every level corrects its reading
         * with its own row of kac_est_fix_mdb. The two brute settings are rows
         * past the last level, so one assignment covers them. */
        static const struct {
            unsigned w;
            int      est;
            int      form;
            unsigned lag;
            unsigned cand;
            int      plain;
            unsigned neighbor;
        } lv[KAC_QUALITY_MAX + 3] = {
            {8u, 0, KAC_FORM_SEP, KAC_LAG_WHOLE, KAC_PROBE_CAND, 0, KAC_SF_NEIGHBOR_PCT},
            {7u, 1, KAC_FORM_SEP, KAC_LAG_WHOLE, KAC_PROBE_CAND, 0, KAC_SF_NEIGHBOR_PCT},
            {6u, 1, KAC_FORM_SEP, KAC_LAG_WHOLE, KAC_PROBE_CAND, 0, KAC_SF_NEIGHBOR_PCT},
            {5u, 1, 8, KAC_LAG_WHOLE, 8u, 0, KAC_SF_NEIGHBOR_PCT},
            {4u, 1, 4, 4u, 8u, 0, KAC_SF_NEIGHBOR_PCT},
            {3u, 1, 2, 2u, 7u, 0, KAC_SF_NEIGHBOR_PCT},
            {2u, 1, 1, 1u, 6u, 0, KAC_SF_NEIGHBOR_PCT},
            {0u, 1, KAC_FORM_NONE, KAC_LAG_NONE, 6u, 0, KAC_SF_NEIGHBOR_PCT},
            {0u, 1, KAC_FORM_NONE, KAC_LAG_NONE, 5u, 0, KAC_SF_NEIGHBOR_PCT},
            {0u, 1, KAC_FORM_NONE, KAC_LAG_NONE, 4u, 0, KAC_SF_NEIGHBOR_PCT},
            {0u, 1, KAC_FORM_NONE, KAC_LAG_NONE, 0u, 1, KAC_SF_NEIGHBOR_PCT},
            {10u, 0, KAC_FORM_SEP, KAC_LAG_WHOLE, 12u, 0, 50u},
            {12u, 0, KAC_FORM_SEP, KAC_LAG_WHOLE, 16u, 0, 100u},
        };
        unsigned s;

        if (g->brute_level == 1u)
            s = KAC_QUALITY_MAX + 1u;
        else if (g->brute_level == 2u)
            s = KAC_QUALITY_MAX + 2u;
        else
            s = (g->quality >= 0 && g->quality <= KAC_QUALITY_MAX) ? (unsigned)g->quality : 0u;

        g->beam_w       = lv[s].w;
        g->probe_cand   = lv[s].cand;
        g->est_measure  = lv[s].est;
        g->form         = lv[s].form;
        g->beam_lag     = lv[s].lag;
        g->plain        = lv[s].plain;
        g->neighbor_pct = lv[s].neighbor;
    }
    g->state = KAC_STATE_READY;
    return 0;
}

static size_t preamble_bytes(const kac_t *g)
{
    size_t n = KAC_HDR_SIZE;
    if (g->init_snap_valid)
        n += (size_t)g->channels * KAC_SNAP_SIZE;
    if (g->loop_end > g->loop_start)
        n += (size_t)g->channels * KAC_SNAP_SIZE;
    return n;
}

/* Seek index entries a file of the samples fed so far carries: one per
 * IndexStride samples, a final partial stride still counted. */
static uint32_t index_entries(const kac_t *restrict g)
{
    if (g->stride == 0u || g->total_in == 0u)
        return 0u;
    return (g->total_in - 1u) / g->stride + 1u;
}

size_t kac_encode_bufsize(const kac_t *restrict g, uint32_t frames)
{
    size_t bound, span, blocks, nsl, chunk;

    if (!g || g->n == 0)
        return 0; /* not configured yet: there is no slice size to bound */
    /* One call carries over at most one chunk minus a sample. A chunk of L
     * samples costs one tier byte plus ceil(L/n) slices at the narrowest tier,
     * and a stream chunk may be shorter than the file grid. */
    chunk  = g->chunk_samples ? g->chunk_samples : kac_block_for_rate((unsigned)g->samplerate);
    span   = (size_t)frames + chunk;
    blocks = (span + chunk - 1u) / chunk + 1u;
    nsl    = span / kac_narrowest_n() + blocks;
    bound  = blocks + nsl * KAC_SLICE_SIZE * (size_t)g->channels +
            (size_t)KAC_SLICE_SIZE * (size_t)g->channels;
    if (!g->stream)
        bound += preamble_bytes(g);
    /* The flush emits the seek index as well as the tail block, so one bound
     * covers both. IndexStride >= the file block (Section 3.1), which keeps the
     * sum inside size_t for any accepted file. */
    if (g->stride != 0 && frames > 0) {
        uint64_t entries = ((uint64_t)frames - 1u) / g->stride + 1u;
        bound += (size_t)(entries * 4u);
        bound += (size_t)(entries * (uint64_t)g->channels * KAC_SNAP_SIZE);
    }
    return bound;
}

static inline unsigned nearest_level(const int16_t *restrict dq, unsigned np, int32_t a,
                                     int32_t base, int32_t step, uint32_t recip, int32_t half);

/* The two levels nearest to `ev`, their indices, and the nearest distance.
 * Once the nearest magnitude is known, the second is either the other sign of
 * that magnitude or a same-sign neighbour; the candidates run in index order,
 * so a tie still goes to the smaller index. */
static void nearest_two(const int16_t *restrict dq, unsigned nq, int64_t ev, int32_t base,
                        int32_t step, uint32_t recip, int32_t half, unsigned *restrict n1,
                        unsigned *restrict n2, int64_t *restrict d1)
{
    int64_t  e1 = -1, e2 = -1;
    int64_t  ae = ev < 0 ? -ev : ev;
    unsigned b1 = 0, b2 = 0, np = nq >> 1u, c[4], nc = 0, t;

    if (np <= 2u) {
        for (t = 0; t < nq; t++)
            c[nc++] = t;
    } else {
        unsigned j    = nearest_level(dq, np, (int32_t)ae, base, step, recip, half);
        unsigned sign = ev < 0 ? 1u : 0u;
        unsigned b    = 2u * j + sign;

        c[nc++] = b;
        c[nc++] = b ^ 1u;
        if (j > 0u)
            c[nc++] = 2u * (j - 1u) + sign;
        if (j + 1u < np)
            c[nc++] = 2u * (j + 1u) + sign;
        for (t = 1; t < nc; t++) {
            unsigned v = c[t], u = t;

            while (u > 0u && c[u - 1u] > v) {
                c[u] = c[u - 1u];
                u--;
            }
            c[u] = v;
        }
    }
    for (t = 0; t < nc; t++) {
        unsigned q  = c[t];
        int64_t  er = ev - (int64_t)dq[q];

        if (er < 0)
            er = -er;
        if (e1 < 0 || er < e1) {
            e2 = e1;
            b2 = b1;
            e1 = er;
            b1 = q;
        } else if (e2 < 0 || er < e2) {
            e2 = er;
            b2 = q;
        }
    }
    *n1 = b1;
    *n2 = b2;
    *d1 = e1 < 0 ? 0 : e1;
}

/* What one slice pass measured: error, reported counters, and the loop-point
 * snapshot. count_slice leaves sse zero because the beam already summed it. */
typedef struct {
    int64_t  sse;
    uint32_t clip;
    uint64_t wlim;
    uint64_t pred_over;
    kac_snap snap;
    int      got_snap;
    int      report;
} kac_slice_stat;

/* The per sample part of the counters, so the greedy coder and the replay
 * after the search count the same things the same way. */
static inline void kac_stat_step(const kac_ch *restrict e, int32_t s, kac_slice_stat *restrict st)
{
    unsigned k;

    if (!st->report)
        return;
    if (s == 32767 || s == -32768)
        st->clip++;
    for (k = 0; k < 4; k++)
        if (e->w[k] == KAC_W_LIM || e->w[k] == -KAC_W_LIM) {
            st->wlim++;
            break;
        }
}

/* The snapshot at one sample: the loop point is a sample and not a slice
 * boundary, so it is taken here rather than once per slice. */
static inline void kac_stat_snap(const kac_ch *restrict e, kac_slice_stat *restrict st)
{
    unsigned t;

    for (t = 0; t < 4; t++)
        st->snap.h[t] = (int16_t)e->h[t];
    for (t = 0; t < 4; t++)
        st->snap.w[t] = (int16_t)e->w[t];
    st->got_snap = 1;
}

/* The `width` cheapest of the paths just expanded, left in the beam's arrays,
 * with their count returned. A path the greedy symbols already beat is not
 * carried: the bound is the caller's own error. `csse[j] < 0` marks one already
 * taken, which is cheaper than sorting and does not care which tie wins. */
static unsigned keep_paths(kac_ch *restrict bp, int64_t *restrict bs, unsigned *restrict bn,
                           kac_ch *restrict cand, int64_t *restrict csse, unsigned *restrict cnode,
                           unsigned ncand, unsigned width, int64_t sse_cap)
{
    unsigned c, j;

    for (c = 0; c < width; c++) {
        unsigned bi = 0xffffffffu;
        int64_t  bv = 0;

        for (j = 0; j < ncand; j++) {
            if (csse[j] < 0)
                continue;
            if (bi == 0xffffffffu || csse[j] < bv) {
                bv = csse[j];
                bi = j;
            }
        }
        if (bi == 0xffffffffu)
            break;
        if (csse[bi] >= sse_cap)
            break;
        bp[c]    = cand[bi];
        bs[c]    = csse[bi];
        bn[c]    = cnode[bi];
        csse[bi] = -1;
    }
    return c;
}

/* Replay kept symbols through the decoder step to count counters and take the
 * loop-point snapshot. The beam does not count while searching. */
static void count_slice(const kac_ch *restrict e0, const int16_t *restrict dq,
                        const unsigned char *restrict qrs, unsigned len, int cap_rel,
                        kac_slice_stat *restrict st)
{
    kac_ch   e = *e0;
    unsigned i;

    st->sse       = 0;
    st->clip      = 0;
    st->wlim      = 0;
    st->pred_over = 0;
    st->got_snap  = 0;
    for (i = 0; i < len; i++) {
        int32_t pred;
        int32_t s;

        if ((int)i == cap_rel)
            kac_stat_snap(&e, st);
        pred = kac_predict(&e);
        if (st->report && (pred > 32767 || pred < -32768))
            st->pred_over++;
        s = kac_step_pred(&e, dq[qrs[i]], pred);
        kac_stat_step(&e, s, st);
    }
}

/* The interior of a scalefactor row: magnitudes form an arithmetic progression
 * from `base` in steps of `step`, and `recip` divides by that step. A row with
 * two interior levels or fewer is compared endpoint by endpoint and needs none
 * of it. Four passes read rows this way, so the arithmetic lives in one place. */
static void row_params(const int16_t *restrict dq, unsigned nq, int32_t *restrict step,
                       int32_t *restrict base, int32_t *restrict half, uint32_t *restrict recip)
{
    if (nq <= 4u) {
        *step  = 0;
        *base  = 0;
        *half  = 0;
        *recip = 0u;
        return;
    }
    *step  = (int32_t)dq[4] - dq[2];
    *base  = (int32_t)dq[2] - *step;
    *half  = (*step - 1) / 2;
    *recip = 0xFFFFFFFFu / (uint32_t)*step + 1u;
}

/* Bounded search along the residual path of one slice at the scalefactor the
 * greedy pass chose. Each sample extends every path into its two nearest levels;
 * the `width` least-error paths survive. Codes come from the trace, `end_out` is
 * the best path's state, and the caller keeps its greedy result unless this
 * returns less error. */
static int64_t refine_slice(const kac_ch *restrict e0, const int16_t *restrict x, unsigned len,
                            unsigned width, unsigned lag, const int16_t *restrict dq, unsigned w,
                            int cap_rel, int64_t sse_cap, unsigned char *restrict qrs,
                            kac_ch *restrict end_out, kac_slice_stat *restrict st)
{
    const unsigned nq = 1u << w;
    int32_t        step, base, half;
    uint32_t       recip;
    kac_ch         bp[KAC_BEAM_WIDTH];
    int64_t        bs[KAC_BEAM_WIDTH];
    unsigned       bn[KAC_BEAM_WIDTH];
    kac_ch         cand[KAC_BEAM_WIDTH * 2u];
    int64_t        csse[KAC_BEAM_WIDTH * 2u];
    unsigned       cnode[KAC_BEAM_WIDTH * 2u];
    /* Parent indices stay below KAC_BEAM_NODES (1441), so 16 bits hold them. */
    uint16_t      trp[KAC_BEAM_NODES];
    unsigned char trs[KAC_BEAM_NODES];
    unsigned      npath = 1u, tn = 1u, p, w2, i;

    row_params(dq, nq, &step, &base, &half, &recip);
    bp[0]  = *e0;
    bs[0]  = 0;
    bn[0]  = 0;
    trp[0] = 0u;
    trs[0] = 0u;
    for (i = 0; i < len; i++) {
        unsigned ncand = 0;

        for (p = 0; p < npath; p++) {
            int32_t  pred = kac_predict(&bp[p]);
            int64_t  ev   = (int64_t)x[i] - (int64_t)pred;
            int64_t  e1   = 0;
            unsigned n1 = 0u, n2 = 0u, nbr = 2u;

            nearest_two(dq, nq, ev, base, step, recip, half, &n1, &n2, &e1);
            /* Both levels are carried unless the nearest one is exact: there is
             * nothing for the second to improve on then. */
            if (e1 == 0)
                nbr = 1u;
            for (w2 = 0u; w2 < nbr; w2++) {
                unsigned sym = w2 ? n2 : n1;
                kac_ch   nx  = bp[p];
                int32_t  rec = kac_step_pred(&nx, dq[sym], pred);
                int64_t  de  = (int64_t)x[i] - (int64_t)rec;

                trp[tn]      = bn[p];
                trs[tn]      = (unsigned char)sym;
                cand[ncand]  = nx;
                csse[ncand]  = bs[p] + de * de;
                cnode[ncand] = tn;
                ncand++;
                tn++;
            }
        }
        npath = keep_paths(bp, bs, bn, cand, csse, cnode, ncand, width, sse_cap);
        if (npath == 0u)
            return -1;
        /* A lag drops every path but the best, so the ones after it re-widen
         * from one: less lookahead, and fewer states to carry. */
        if (lag != 0u && (i + 1u) % lag == 0u)
            npath = 1u;
    }
    {
        unsigned nd = bn[0];

        for (i = len; i > 0u; i--) {
            qrs[i - 1u] = trs[nd];
            nd          = trp[nd];
        }
    }
    *end_out = bp[0];
    count_slice(e0, dq, qrs, len, cap_rel, st);
    return bs[0];
}

/* Nearest magnitude index in one scalefactor row. Interior magnitudes form an
 * arithmetic progression, so rounding (a - base) / step with a 32-bit
 * reciprocal lands on the nearest interior index; the two endpoints are
 * compared separately. Ties go to the lower index. */
static inline unsigned nearest_level(const int16_t *restrict dq, unsigned np, int32_t a,
                                     int32_t base, int32_t step, uint32_t recip, int32_t half)
{
    int32_t  j, d, bd;
    unsigned best;

    if (a <= dq[0])
        return 0u;
    if (a >= dq[2u * (np - 1u)])
        return np - 1u;
    if (np <= 2u)
        return (a - dq[0] <= dq[2] - a) ? 0u : 1u;
    j = (int32_t)(((int64_t)(a - base + half) * recip) >> 32);
    if (j < 1)
        j = 1;
    else if (j > (int32_t)np - 2)
        j = (int32_t)np - 2;
    best = 0u;
    bd   = a - dq[0];
    d    = a - (base + step * j);
    if (d < 0)
        d = -d;
    if (d < bd) {
        bd   = d;
        best = (unsigned)j;
    }
    d = dq[2u * (np - 1u)] - a;
    if (d < bd)
        best = np - 1u;
    return best;
}

/* One scalefactor candidate over a whole slice. Returns 1 when the running
 * error exceeds sse_cap, in which case the candidate cannot win. */
static int trial_slice(kac_ch *restrict e, const int16_t *restrict x, unsigned len,
                       const int16_t *restrict dq, unsigned w, int cap_rel, int64_t sse_cap,
                       unsigned char *restrict qrs, kac_slice_stat *restrict st)
{
    const unsigned nq = 1u << w;
    const unsigned np = nq >> 1u;
    int32_t        step, base, half;
    uint32_t       recip = 0u;
    unsigned       i;

    row_params(dq, nq, &step, &base, &half, &recip);

    st->sse       = 0;
    st->clip      = 0;
    st->wlim      = 0;
    st->pred_over = 0;
    st->got_snap  = 0;
    for (i = 0; i < len; i++) {
        int32_t  pred, s;
        int64_t  ev;
        int32_t  a;
        unsigned j, best;

        if ((int)i == cap_rel)
            kac_stat_snap(e, st);
        pred = kac_predict(e);
        if (st->report && (pred > 32767 || pred < -32768))
            st->pred_over++;
        ev   = (int64_t)x[i] - (int64_t)pred;
        a    = (int32_t)(ev < 0 ? -ev : ev);
        j    = nearest_level(dq, np, a, base, step, recip, half);
        best = 2u * j + (ev < 0 ? 1u : 0u);

        s = kac_step_pred(e, dq[best], pred);
        {
            int64_t de = (int64_t)x[i] - s;
            st->sse += de * de;
        }
        kac_stat_step(e, s, st);
        qrs[i] = (unsigned char)best;
        if (st->sse > sse_cap)
            break;
    }
    return (i < len);
}

/* Envelope of |x - pred| read at `points` evenly spread positions. The whole
 * slice is walked because filter state depends on earlier samples. The probe
 * feeds back the unquantised residual, so it does not depend on earlier
 * candidates; the envelope starts at 1 because an exact match needs one level. */
static int32_t probe_slice(const kac_ch *restrict e0, const int16_t *restrict x, unsigned len,
                           unsigned points)
{
    kac_ch   t = *e0;
    int32_t  m = 1;
    unsigned i;

    if (points > len)
        points = len;
    for (i = 0; i < len; i++) {
        int32_t pred = kac_predict(&t);
        int32_t d    = (int32_t)x[i] - pred;
        int32_t a    = d < 0 ? -d : d;

        if ((i * points) % len < points && a > m)
            m = a;
        kac_step_pred(&t, d, pred);
    }
    return m;
}

static unsigned make_sf_order(const kac_t *g, int32_t probe_m, unsigned char *order)
{
    int s0 = 0, k, cnt = 0;
    while (s0 < 15 && g->rowmax[(unsigned)g->cur_tier - 1u][s0] < probe_m)
        s0++;
    order[cnt++] = (unsigned char)s0;
    if (g->probe_cand == 4u) {
        /* Four-shot: the two one-row neighbours follow in frequency order, and
         * the two-row slot goes to the side low tiers use: below T3 nearly all
         * of the few +-2 winners are on the low side. */
        int      side = (g->cur_tier <= KAC_TIER_MIN + 1) ? -2 : 2;
        int      extra[3];
        unsigned a;

        extra[0] = s0 - 1;
        extra[1] = s0 + 1;
        extra[2] = s0 + side;
        for (a = 0; a < 3u; a++) {
            int v = extra[a];

            if (v < 0)
                v = 0;
            if (v > 15)
                v = 15;
            for (k = 0; k < cnt; k++)
                if ((int)order[k] == v)
                    break;
            if (k == cnt && cnt < (int)g->probe_cand)
                order[cnt++] = (unsigned char)v;
        }
    }
    /* Walk outward from the estimate. The count is fixed so every slice costs
     * the same search, even when one side runs out. */
    for (k = 1; cnt < (int)g->probe_cand; k++) {
        int added = 0;
        int v;

        /* The lower row wins far more often than the higher one, so it is
         * tried first; a winner tightens the cap for everything after it. */
        v = s0 - k;
        if (v >= 0) {
            unsigned q;

            added = 1;
            for (q = 0; q < (unsigned)cnt; q++)
                if ((int)order[q] == v)
                    break;
            if (q == (unsigned)cnt && cnt < (int)g->probe_cand)
                order[cnt++] = (unsigned char)v;
        }
        v = s0 + k;
        if (v <= 15) {
            unsigned q;

            added = 1;
            for (q = 0; q < (unsigned)cnt; q++)
                if ((int)order[q] == v)
                    break;
            if (q == (unsigned)cnt && cnt < (int)g->probe_cand)
                order[cnt++] = (unsigned char)v;
        }
        if (!added)
            break;
    }
    return (unsigned)cnt;
}

/* scalefactor occupies bits 63..60, then n residuals of w bits, high first:
 * 4 + n*w <= 64, so residual i sits at 60 - w*(i+1) and the bits below the last
 * index, if the tier has any, are left zero here. */
static void pack_slice(uint8_t *restrict p, unsigned sf, const unsigned char *restrict qrs,
                       unsigned w, unsigned n)
{
    uint64_t v = (uint64_t)sf << 60;
    unsigned i;
    for (i = 0; i < n; i++)
        v |= (uint64_t)qrs[i] << (60u - w * (i + 1u));
    kac_put64(p, v);
}

/* The index point at this position, if any. Its offset and snapshot are written
 * by different functions, so the position derivation lives here once. */
static int index_point_at(const kac_t *restrict g, uint32_t *restrict k)
{
    if (g->stride == 0 || g->pos % g->stride != 0)
        return 0;
    *k = g->pos / g->stride;
    return 1;
}

/* Write the snapshot half of an index point, if this position has one. */
static int index_point_snap(kac_t *restrict g)
{
    uint32_t k;
    unsigned c;
    size_t   need;

    if (!index_point_at(g, &k))
        return 0;
    need = ((size_t)k * (size_t)g->channels + (size_t)g->channels) * KAC_SNAP_SIZE;
    if (!g->index_store || need > g->index_store_bytes)
        return KAC_E_INDEX;
    for (c = 0; c < (unsigned)g->channels; c++)
        kac_snap_store(g->index_store + ((size_t)k * g->channels + c) * KAC_SNAP_SIZE, &g->ch[c]);
    return 0;
}

/* What one channel's searches settled on for a slice. Both searches fill it, so
 * the commit writes back one result. */
typedef struct {
    int64_t       sse;
    unsigned      sf;
    uint32_t      clip;
    uint64_t      wlim;
    uint64_t      pred_over;
    kac_ch        end;
    kac_snap      snap;
    int           got_snap;
    unsigned char qrs[64];
} kac_slice_best;

/* Try each proposed scalefactor over the whole slice, pruning candidates that
 * cannot beat the running winner. A pruned candidate leaves -1 in cand_sse. The
 * local qrs keeps a partial candidate from overwriting the winner's symbols. */
static void search_sf_order(kac_t *restrict g, unsigned c, const int16_t *restrict x, unsigned len,
                            int cap_rel, const unsigned char *restrict order, unsigned cnt,
                            int64_t *restrict cand_sse, kac_slice_best *restrict best)
{
    unsigned char qrs[64];
    unsigned      i;

    best->sse = -1;
    for (i = 0; i < sizeof qrs; i++)
        qrs[i] = 0;
    for (i = 0; i < cnt; i++) {
        unsigned       sf = order[i];
        kac_ch         t  = g->ch[c];
        kac_slice_stat cand;
        int64_t        cap_sse = (best->sse < 0) ? ((int64_t)1 << 62) : best->sse;
        const int16_t *dqt     = g->dqt + ((size_t)sf << g->w);
        unsigned       j;

        cand.report = g->report;
        if (trial_slice(&t, x, len, dqt, g->w, cap_rel, cap_sse, qrs, &cand))
            continue;
        cand_sse[i] = cand.sse;
        if (best->sse < 0 || cand.sse < best->sse) {
            best->sse       = cand.sse;
            best->sf        = sf;
            best->end       = t;
            best->clip      = cand.clip;
            best->wlim      = cand.wlim;
            best->pred_over = cand.pred_over;
            if (cand.got_snap) {
                best->snap     = cand.snap;
                best->got_snap = 1;
            }
            for (j = 0; j < sizeof best->qrs; j++)
                best->qrs[j] = qrs[j];
        }
    }
}

/* Refine the winning scalefactor and, if one is within neighbor_pct, a runner-up
 * that could overturn it. Only candidates that ran are considered. A result can
 * only replace the greedy symbols with less error; if every path is pruned, the
 * greedy symbols stand. */
static void search_beam(kac_t *restrict g, unsigned c, const int16_t *restrict x, unsigned len,
                        int cap_rel, const unsigned char *restrict order, unsigned cnt,
                        const int64_t *restrict cand_sse, kac_slice_best *restrict best)
{
    unsigned char qrs[64];
    unsigned      try_sf[2];
    unsigned      ntry = 1, q2, t2;

    if (g->beam_w == 0u)
        return;
    try_sf[0] = best->sf;
    for (q2 = 0; q2 < cnt; q2++) {
        if (order[q2] == best->sf || cand_sse[q2] < 0)
            continue;
        if (cand_sse[q2] * 100 <= best->sse * (100 + (int64_t)g->neighbor_pct)) {
            try_sf[1] = order[q2];
            ntry      = 2;
            break;
        }
    }
    for (t2 = 0; t2 < ntry; t2++) {
        kac_ch         rend;
        kac_slice_stat r;
        int64_t        rsse;
        unsigned       j;

        for (j = 0; j < sizeof qrs; j++)
            qrs[j] = 0; /* a padded tail slice stays byte stable */
        r.report = g->report;
        rsse     = refine_slice(&g->ch[c], x, len, g->beam_w, g->beam_lag,
                                g->dqt + ((size_t)try_sf[t2] << g->w), g->w, cap_rel, best->sse, qrs,
                                &rend, &r);
        if (rsse >= 0 && rsse < best->sse) {
            best->sse       = rsse;
            best->sf        = try_sf[t2];
            best->end       = rend;
            best->clip      = r.clip;
            best->wlim      = r.wlim;
            best->pred_over = r.pred_over;
            if (r.got_snap) {
                best->snap     = r.snap;
                best->got_snap = 1;
            }
            for (j = 0; j < sizeof best->qrs; j++)
                best->qrs[j] = qrs[j];
        }
    }
}

static void best_reset(kac_slice_best *restrict b)
{
    unsigned i;

    b->sse       = -1;
    b->sf        = 0;
    b->clip      = 0;
    b->wlim      = 0;
    b->pred_over = 0;
    b->got_snap  = 0;
    for (i = 0; i < sizeof b->qrs; i++)
        b->qrs[i] = 0;
}

/* Commit one channel's search result: state, counters, loop snapshot, the
 * reconstruction a mid/side measurement needs, and the packed slice. */
static void channel_commit(kac_t *restrict g, unsigned c, const kac_slice_best *restrict best,
                           kac_ch e0, unsigned base, unsigned len, int recon, uint8_t *restrict out)
{
    unsigned j;

    g->ch[c] = best->end;
    g->sse += best->sse;
    g->clip += best->clip;
    g->w_limit += best->wlim;
    g->pred_over += best->pred_over;
    if (best->got_snap) {
        g->loop_snap[c] = best->snap;
        if (c + 1u == (unsigned)g->channels)
            g->loop_snap_valid = 1;
    }
    if (recon) {
        const int16_t *dq = g->dqt + ((size_t)best->sf << g->w);
        kac_ch         e  = e0;

        for (j = 0; j < len; j++)
            g->sbuf[c][base + j] = (int16_t)kac_step(&e, dq[best->qrs[j]]);
    }
    pack_slice(out, best->sf, best->qrs, g->w, g->n);
}

/* One channel of a slice: the two searches, the state they leave behind, the
 * reconstruction the decoder will produce where the block's form is mid and
 * side, and the symbols packed into the channel's 8 bytes. */
static void code_channel(kac_t *restrict g, unsigned c, unsigned base, unsigned len, int cap_rel,
                         int recon, uint8_t *restrict out)
{
    const int16_t *x = g->sbuf[c] + base;
    /* The ordering holds the level's own candidate count, at most
     * KAC_PROBE_CAND, and a slice at most 60 samples; both arrays below are
     * sized past what they take. */
    unsigned char  order[16];
    int64_t        cand_sse[16];
    kac_ch         e0 = g->ch[c]; /* the slice's entry state, for the write-back */
    kac_slice_best best;
    unsigned       i, cnt;

    best_reset(&best);
    for (i = 0; i < sizeof cand_sse / sizeof cand_sse[0]; i++)
        cand_sse[i] = -1;

    cnt = make_sf_order(g, probe_slice(&g->ch[c], x, len, KAC_PROBE_POINTS), order);
    search_sf_order(g, c, x, len, cap_rel, order, cnt, cand_sse, &best);
    search_beam(g, c, x, len, cap_rel, order, cnt, cand_sse, &best);
    channel_commit(g, c, &best, e0, base, len, recon, out);
}

/* One channel of a slice with no search: the row the residual envelope names,
 * then the nearest level at every sample through the decoder's own step. This
 * is the encoder reduced to the shape a differential coder has -- predict,
 * quantise the residual, carry the reconstruction -- with the row standing in
 * for the step size. The row is signalled rather than adapted, so unlike a
 * classic coder's step it need not follow from the previous sample. */
static void code_channel_plain(kac_t *restrict g, unsigned c, unsigned base, unsigned len,
                               int cap_rel, int recon, uint8_t *restrict out)
{
    const int16_t *x  = g->sbuf[c] + base;
    kac_ch         e0 = g->ch[c];
    kac_ch         t  = e0;
    kac_slice_stat st;
    kac_slice_best best;
    unsigned char  qrs[64];
    unsigned       sf, i;
    int32_t        env = probe_slice(&e0, x, len, KAC_PROBE_POINTS);
    const int16_t *dq;

    sf = 0;
    while (sf < 15 && g->rowmax[(unsigned)g->cur_tier - 1u][sf] < env)
        sf++;
    /* A padded tail slice stays byte stable. */
    for (i = 0; i < sizeof qrs; i++)
        qrs[i] = 0;
    dq        = g->dqt + ((size_t)sf << g->w);
    st.report = g->report;
    trial_slice(&t, x, len, dq, g->w, cap_rel, (int64_t)1 << 62, qrs, &st);

    best_reset(&best);
    best.sse       = st.sse;
    best.sf        = sf;
    best.end       = t;
    best.clip      = st.clip;
    best.wlim      = st.wlim;
    best.pred_over = st.pred_over;
    if (st.got_snap) {
        best.snap     = st.snap;
        best.got_snap = 1;
    }
    for (i = 0; i < sizeof best.qrs; i++)
        best.qrs[i] = qrs[i];
    channel_commit(g, c, &best, e0, base, len, recon, out);
}

/* One channel of a slice at a level that refines nothing: the rows the envelope
 * names are tried in order, each capped by the running best, and the winner is
 * committed. The search is search_sf_order's; what is gone is the candidate
 * array the beam reads and the beam call itself, both unreachable once the
 * width is zero. */
static void code_channel_rows(kac_t *restrict g, unsigned c, unsigned base, unsigned len,
                              int cap_rel, int recon, uint8_t *restrict out)
{
    const int16_t *x  = g->sbuf[c] + base;
    kac_ch         e0 = g->ch[c];
    unsigned char  order[16], qrs[64];
    kac_slice_stat cand;
    kac_slice_best best;
    unsigned       cnt, i, j;

    cnt = make_sf_order(g, probe_slice(&e0, x, len, KAC_PROBE_POINTS), order);
    best_reset(&best);
    for (i = 0; i < sizeof qrs; i++)
        qrs[i] = 0;
    for (i = 0; i < cnt; i++) {
        unsigned sf  = order[i];
        kac_ch   t   = e0;
        int64_t  cap = (best.sse < 0) ? ((int64_t)1 << 62) : best.sse;

        cand.report = g->report;
        if (trial_slice(&t, x, len, g->dqt + ((size_t)sf << g->w), g->w, cap_rel, cap, qrs, &cand))
            continue;
        if (best.sse < 0 || cand.sse < best.sse) {
            best.sse       = cand.sse;
            best.sf        = sf;
            best.end       = t;
            best.clip      = cand.clip;
            best.wlim      = cand.wlim;
            best.pred_over = cand.pred_over;
            if (cand.got_snap) {
                best.snap     = cand.snap;
                best.got_snap = 1;
            }
            for (j = 0; j < sizeof best.qrs; j++)
                best.qrs[j] = qrs[j];
        }
    }
    channel_commit(g, c, &best, e0, base, len, recon, out);
}

/* Code one slice of len samples; unused slots keep residual zero. With `recon`,
 * write the decoder's reconstruction back over those samples, which mid/side
 * measurement needs (see tier_measure). */
static int encode_slice(kac_t *restrict g, uint8_t *restrict out, unsigned base, unsigned len,
                        int recon)
{
    const uint32_t pos0 = g->pos;
    const unsigned n    = g->n;
    unsigned       c;
    int            cap_rel = -1;
    int            rc;

    rc = index_point_snap(g);
    if (rc != 0)
        return rc;
    if (g->loop_end > g->loop_start && g->loop_start >= pos0 && g->loop_start < pos0 + len)
        cap_rel = (int)(g->loop_start - pos0);

    for (c = 0; c < (unsigned)g->channels; c++) {
        uint8_t *dst = out + (size_t)c * KAC_SLICE_SIZE;

        if (g->plain)
            code_channel_plain(g, c, base, len, cap_rel, recon, dst);
        else if (g->beam_w == 0u)
            code_channel_rows(g, c, base, len, cap_rel, recon, dst);
        else
            code_channel(g, c, base, len, cap_rel, recon, dst);
    }
    g->pos += n;
    g->fill = 0;
    return 0;
}

/* Level in milli-dB from an error sum and sample count, matching
 * kac_get_psnr: 10*log10(peak^2 * samples / sse), built from integer logs so no
 * product can overflow. A file whose peak is still zero has no finite level;
 * KAC_MDB_MAX keeps it from tying every tier or producing a NaN slope. */
static int32_t vbr_level(const kac_t *restrict g, uint64_t sse, unsigned samples)
{
    if (sse == 0 || g->peak <= 0)
        return KAC_MDB_MAX;
    return 2 * kac_log10_mdb((uint64_t)g->peak) +
           kac_log10_mdb((uint64_t)samples * (uint64_t)g->channels) - kac_log10_mdb(sse);
}

/* Keep the `cap` smallest errors seen, each with the sample count it covers, in
 * ascending order. The error and its count travel together because what the
 * reading drops is both of them. */
static void keep_cut(int64_t *restrict sse, unsigned *restrict cnt, unsigned *restrict n,
                     unsigned cap, int64_t v, unsigned samples)
{
    unsigned i;

    if (*n < cap) {
        for (i = *n; i > 0u && sse[i - 1u] > v; i--) {
            sse[i] = sse[i - 1u];
            cnt[i] = cnt[i - 1u];
        }
        sse[i] = v;
        cnt[i] = samples;
        (*n)++;
    } else if (v < sse[cap - 1u]) {
        for (i = cap - 1u; i > 0u && sse[i - 1u] > v; i--) {
            sse[i] = sse[i - 1u];
            cnt[i] = cnt[i - 1u];
        }
        sse[i] = v;
        cnt[i] = samples;
    }
}

/* The mirror of keep_cut: the `cap` largest errors, descending, each with the
 * sample count it covers. */
static void keep_top(int64_t *restrict sse, unsigned *restrict cnt, unsigned *restrict n,
                     unsigned cap, int64_t v, unsigned samples)
{
    unsigned i;

    if (*n < cap) {
        for (i = *n; i > 0u && sse[i - 1u] < v; i--) {
            sse[i] = sse[i - 1u];
            cnt[i] = cnt[i - 1u];
        }
        sse[i] = v;
        cnt[i] = samples;
        (*n)++;
    } else if (v > sse[cap - 1u]) {
        for (i = cap - 1u; i > 0u && sse[i - 1u] < v; i--) {
            sse[i] = sse[i - 1u];
            cnt[i] = cnt[i - 1u];
        }
        sse[i] = v;
        cnt[i] = samples;
    }
}

/* The level a tier is judged on: the median of its weak 20 % band, middle band
 * and strong 20 % band. Both modes and the measurement window use it. */
static int32_t tier_level(const kac_t *restrict g, int t)
{
    return g->tier_psnr_mid[t];
}

/* How far the aim may rise, in milli-dB: from the middle band to the weak band.
 * This is also the cap on one upward block move. */
static int32_t aim_slack(const kac_t *restrict g, int tier)
{
    int32_t slack = 0;

    if (g->tier_band_lo[tier] != KAC_UNMEASURED && g->tier_psnr_mid[tier] != KAC_UNMEASURED)
        slack = g->tier_psnr_mid[tier] - g->tier_band_lo[tier];
    if (slack < KAC_V_SLACK_MIN)
        slack = KAC_V_SLACK_MIN;
    return slack;
}

/* How far the aim may fall, in milli-dB: from the middle band to the strong
 * band. This is also the cap on one downward block move. */
static int32_t aim_fall(const kac_t *restrict g, int tier)
{
    int32_t fall = 0;

    if (g->tier_band_hi[tier] != KAC_UNMEASURED && g->tier_psnr_mid[tier] != KAC_UNMEASURED)
        fall = g->tier_band_hi[tier] - g->tier_psnr_mid[tier];
    if (fall < KAC_V_SLACK_MIN)
        fall = KAC_V_SLACK_MIN;
    return fall;
}

/* Lowest tier in the window that may carry the block. T2..T6 are judged on
 * tier_level(); T1 only if its weakest gate share reaches the aim, and T7 only
 * if T6's strongest gate share misses it. A tier outside the window or
 * unmeasured is skipped, and the window floor is never crossed. */
static int vbr_pick(const kac_t *restrict g, int32_t aim)
{
    int t, hi = g->tier_hi;

    for (t = g->tier_lo; t < hi; t++) {
        int32_t level;

        if (g->tier_psnr[t] == KAC_UNMEASURED)
            continue;
        level = (t == KAC_TIER_MIN) ? g->tier_low[t] : tier_level(g, t);
        if (level >= aim)
            return t;
    }
    if (hi < KAC_TIER_MAX || g->tier_high[KAC_TIER_MAX - 1] < aim)
        return hi;
    return hi - 1;
}

/* Set the first ABR aim from the block's own ladder: the goal rate lies
 * between two nominal tiers, and the level at the same fractional position
 * between their readings is the aim. A flat ladder is silence and names no
 * tier, so the next block is tried. */
static void abr_seed(kac_t *restrict g)
{
    int32_t lo = KAC_MDB_MAX, hi = -KAC_MDB_MAX, aim;
    int     t, a = 0, b = 0, seen = 0;

    if (g->vbr_mode != KAC_VBR_ABR || g->abr_seeded)
        return;
    for (t = KAC_TIER_MIN; t <= KAC_TIER_MAX; t++) {
        int32_t p = g->tier_psnr[t];
        if (p == KAC_UNMEASURED)
            continue;
        if (p < lo)
            lo = p;
        if (p > hi)
            hi = p;
        seen++;
    }
    /* A block whose weakest reading is above every target names no useful aim. */
    if (seen < 2 || lo > KAC_AIM_MAX_MDB || hi - lo < KAC_ABR_SEED_SPREAD)
        return;
    for (t = KAC_TIER_MIN; t <= KAC_TIER_MAX; t++) {
        if (kac_tier_rate_mb(t) <= g->vbr_target)
            a = t;
        else {
            b = t;
            break;
        }
    }
    if (a == 0)
        a = b;
    if (b == 0)
        b = a;
    if (g->tier_psnr[a] == KAC_UNMEASURED || g->tier_psnr[b] == KAC_UNMEASURED)
        return;
    {
        int32_t ra = kac_tier_rate_mb(a), rb = kac_tier_rate_mb(b);
        int32_t la = (a == KAC_TIER_MIN) ? g->tier_low[a] : tier_level(g, a);
        int32_t lb = (b == KAC_TIER_MIN) ? g->tier_low[b] : tier_level(g, b);

        if (la == KAC_UNMEASURED || lb == KAC_UNMEASURED)
            return;
        aim = la;
        if (rb > ra)
            aim += (lb - la) * (g->vbr_target - ra) / (rb - ra);
    }
    g->vbr_psnr   = aim;
    g->abr_seeded = 1;
}

/* dB per bit/sample on this block: the secant across the permitted window,
 * from both endpoints' readings. ABR integrates, so scale error changes the
 * convergence speed, not the fixed point. */
static int32_t vbr_slope(const kac_t *restrict g)
{
    int     slow = g->tier_lo;
    int     fast = g->tier_hi;
    int32_t lv_s = (slow == KAC_TIER_MIN) ? g->tier_low[slow] : tier_level(g, slow);
    int32_t lv_f = (fast == KAC_TIER_MIN) ? g->tier_low[fast] : tier_level(g, fast);
    int32_t db   = lv_f - lv_s; /* milli-dB */
    /* the rate span in milli-bit/sample, so the quotient lands in milli-dB per
     * bit/sample and the outer loop can stay in integers */
    int32_t br =
        (int32_t)((64u * 1000u / kac_tiers[fast - 1].n) - (64u * 1000u / kac_tiers[slow - 1].n));

    if (db <= 0 || br <= 0)
        return 0; /* a block whose ladder does not rise has no slope to give */
    return db * 1000 / br;
}

/* Error between the caller's samples and the reconstructed output over a range,
 * plus rail-clipped output samples when `clip` is requested. Mid/side blocks are
 * merged back before comparison. */
static int64_t block_error(const kac_t *restrict orig, const kac_t *restrict rec, unsigned from,
                           unsigned len, uint32_t *restrict clip)
{
    int64_t  e = 0;
    unsigned c, k;

    if (clip)
        *clip = 0;
    for (c = 0; c < (unsigned)rec->channels; c++) {
        for (k = 0; k < len; k++) {
            int32_t d = (int32_t)orig->sbuf[c][from + k] - (int32_t)rec->sbuf[c][from + k];
            e += (int64_t)d * d;
            if (clip && (rec->sbuf[c][from + k] == 32767 || rec->sbuf[c][from + k] == -32768))
                (*clip)++;
        }
    }
    return e;
}

/* One measurement of a block: whole-block level, the middle band, the 20 %
 * outer bands, and the 5 % gates, all in milli-dB. An approximate pass reports
 * zero bytes. */
typedef struct {
    int32_t psnr;
    int32_t mid;
    int32_t band_lo, band_hi; /* the weakest and strongest KAC_BAND_PCT */
    int32_t low, high;        /* the weakest and strongest KAC_GATE_PCT */
    /* The coding behind the reading: its total error and clipping count, and
     * the slice bytes it wrote to the scratch buffer. A reading taken from an
     * approximate pass has no coding and reports zero bytes. */
    int64_t  sse;
    uint32_t clip;
    size_t   bytes;
} kac_reading;

/* Slice errors and sample counts for one ranking, with one entry past the band
 * so a band and its following boundary come from the same pass. */
typedef struct {
    int64_t  sse[KAC_BAND_KMAX + 1u];
    unsigned cnt[KAC_BAND_KMAX + 1u];
    unsigned n;
} kac_rank;

/* Candidate rule by quality: 1-2 walk the whole list everywhere, 3-4 at
 * T1/T2 (their long slices make probe ranking least reliable), 5-7 walk three,
 * 8 and above take the first. Zero means no cap. Level 0 codes blocks and
 * skips this. A negative `force` keeps this rule; a value at or above zero
 * replaces it, which is how a form measurement states its own effort. */
static unsigned est_cand_limit(const kac_t *restrict g, int tier, int force)
{
    if (force >= 0)
        return (unsigned)force;
    if (g->quality <= 2)
        return 0u;
    if (g->quality <= 4)
        return (tier <= KAC_TIER_MIN + 1) ? 0u : 3u;
    if (g->quality <= 7)
        return 3u;
    return 1u;
}

/* The same pass for two channels at once: the two LMS chains are independent,
 * so one loop can advance both and hide the recurrence latency. */
static void est_slice2(kac_t *restrict pr, unsigned at, unsigned len, int form_cand,
                       int64_t *restrict out0, int64_t *restrict out1)
{
    unsigned char order[2][64];
    unsigned      n = 1u << (pr->w - 1u);
    unsigned      cnt[2], limit, i, k, kmax, c;
    int64_t       best[2] = {INT64_MAX, INT64_MAX};
    kac_ch        e[2], end[2];

    cnt[0] = make_sf_order(pr, probe_slice(&pr->ch[0], pr->sbuf[0] + at, len, KAC_PROBE_POINTS),
                           order[0]);
    cnt[1] = make_sf_order(pr, probe_slice(&pr->ch[1], pr->sbuf[1] + at, len, KAC_PROBE_POINTS),
                           order[1]);
    limit  = est_cand_limit(pr, pr->cur_tier, form_cand);
    kmax   = cnt[0] > cnt[1] ? cnt[0] : cnt[1];
    end[0] = pr->ch[0];
    end[1] = pr->ch[1];
    for (k = 0; k < kmax && (limit == 0u || k < limit); k++) {
        const int16_t *dqt[2]   = {NULL, NULL};
        int32_t        step[2]  = {0, 0};
        int32_t        base[2]  = {0, 0};
        int32_t        half[2]  = {0, 0};
        uint32_t       recip[2] = {0u, 0u};
        int64_t        sse[2]   = {0, 0};
        int            act[2]   = {0, 0};
        int            full[2]  = {0, 0};

        for (c = 0; c < 2u; c++) {
            if (k >= cnt[c])
                continue;
            dqt[c] = pr->dqt + ((size_t)order[c][k] << pr->w);
            e[c]   = pr->ch[c];
            act[c] = 1;
            row_params(dqt[c], 2u * n, &step[c], &base[c], &half[c], &recip[c]);
        }
        for (i = 0; i < len && (act[0] || act[1]); i++) {
#define KAC_EST_STEP(C)                                                                            \
    do {                                                                                           \
        if (act[C]) {                                                                              \
            int32_t  pred = kac_predict(&e[C]);                                                    \
            int32_t  r    = (int32_t)pr->sbuf[C][at + i] - pred;                                   \
            int32_t  a    = r < 0 ? -r : r;                                                        \
            unsigned lo   = nearest_level(dqt[C], n, a, base[C], step[C], recip[C], half[C]);      \
            int32_t  dq   = r < 0 ? -(int32_t)dqt[C][2u * lo] : (int32_t)dqt[C][2u * lo];          \
            int32_t  s    = kac_clamp16(pred + dq);                                                \
            int32_t  err  = (int32_t)pr->sbuf[C][at + i] - s;                                      \
                                                                                                   \
            sse[C] += (int64_t)err * err;                                                          \
            if (sse[C] > best[C]) {                                                                \
                act[C] = 0;                                                                        \
            } else {                                                                               \
                kac_step_pred(&e[C], dq, pred);                                                    \
                if (i + 1u == len)                                                                 \
                    full[C] = 1;                                                                   \
            }                                                                                      \
        }                                                                                          \
    } while (0)
            KAC_EST_STEP(0);
            KAC_EST_STEP(1);
        }
#undef KAC_EST_STEP
        for (c = 0; c < 2u; c++)
            if (full[c] && sse[c] < best[c]) {
                best[c] = sse[c];
                end[c]  = e[c];
            }
    }
    /* The pass carries the winning candidate's state into the next slice. */
    pr->ch[0] = end[0];
    pr->ch[1] = end[1];
    *out0     = best[0];
    *out1     = best[1];
}

/* The approximate slice pass: the same predictor loop and scalefactor
 * candidates as a coding, without the beam, grid walk or packing. */
static int64_t est_slice(kac_t *restrict pr, unsigned c, unsigned at, unsigned len, int form_cand)
{
    unsigned char order[64];
    unsigned      n = 1u << (pr->w - 1u);
    kac_ch        e, best_end;
    unsigned      i, k, cnt, limit;
    int64_t       best = INT64_MAX;

    cnt =
        make_sf_order(pr, probe_slice(&pr->ch[c], pr->sbuf[c] + at, len, KAC_PROBE_POINTS), order);
    best_end = pr->ch[c];
    limit    = est_cand_limit(pr, pr->cur_tier, form_cand);
    for (k = 0; k < cnt && (limit == 0u || k < limit); k++) {
        const int16_t *dqt = pr->dqt + ((size_t)order[k] << pr->w);
        int64_t        sse = 0;
        int32_t        step, base, half;
        uint32_t       recip = 0u;

        row_params(dqt, 2u * n, &step, &base, &half, &recip);

        e = pr->ch[c];
        for (i = 0; i < len; i++) {
            int32_t  pred = kac_predict(&e);
            int32_t  r    = (int32_t)pr->sbuf[c][at + i] - pred;
            int32_t  a    = r < 0 ? -r : r;
            unsigned lo   = nearest_level(dqt, n, a, base, step, recip, half);
            int32_t  dq, s, err;

            dq  = r < 0 ? -(int32_t)dqt[2u * lo] : (int32_t)dqt[2u * lo];
            s   = kac_clamp16(pred + dq);
            err = (int32_t)pr->sbuf[c][at + i] - s;
            sse += (int64_t)err * err;
            if (sse > best)
                break; /* a candidate that cannot win costs no more of the slice */
            kac_step_pred(&e, dq, pred);
        }
        if (i == len && sse < best) {
            best     = sse;
            best_end = e;
        }
    }
    pr->ch[c] = best_end; /* the pass carries the state the next slice starts from */
    return best;
}

/* Whether tiers come from the approximate pass rather than real coding. */
static int est_reading(const kac_t *restrict g)
{
    return g->est_measure;
}

static const int32_t *est_fix_table(const kac_t *restrict g)
{
    int q = g->quality;

    if (g->brute_level != 0u)
        return kac_est_fix_mdb[KAC_QUALITY_MAX + 1];
    if (q < 0 || q > KAC_QUALITY_MAX)
        q = 0;
    return kac_est_fix_mdb[q];
}

/* Milli-dB this setting aims below the run's target; negative aims above. Zero
 * off the -V ladder, where the target is a promise and not a rung. */
static int32_t ladder_drop_mdb(const kac_t *restrict g)
{
    int q = g->quality;

    if (!g->ladder)
        return 0;
    if (g->brute_level != 0u)
        return -KAC_BRUTE_AIM_MDB;
    if (q < 0)
        q = 0;
    if (q > KAC_QUALITY_MAX)
        q = KAC_QUALITY_MAX;
    return kac_level_aim_mdb[q];
}

/* Column of kac_est_fix_mdb for a tier; a tier outside the format reads its
 * nearest end. */
static int fix_index(int tier)
{
    int ti = tier - KAC_TIER_MIN;

    if (ti < 0)
        ti = 0;
    if (ti > KAC_TIER_MAX - KAC_TIER_MIN)
        ti = KAC_TIER_MAX - KAC_TIER_MIN;
    return ti;
}

/* Add the level's per-tier correction to a reading, so it predicts what a block
 * actually delivers. Level 0 and the brute settings read by real coding, and the
 * gap to their own emission is per tier there too, so every level takes the
 * table. */
static int32_t measure_fix(const kac_t *restrict g, int tier)
{
    return est_fix_table(g)[fix_index(tier)];
}

/* Offset carried by a real coding's reading at an estimated level: measure_fix()
 * adds the level's per-tier correction to it, so the left/right baseline takes
 * the same one before the two forms are compared. */
static int32_t est_shortfall_mdb(const kac_t *restrict g, int tier, int estimated)
{
    return estimated ? est_fix_table(g)[fix_index(tier)] : 0;
}

/* Fold the weakest and strongest slice rankings into band and gate readings.
 * `fix` is added to every level; the middle takes the remaining slices. */
static void reading_fold(const kac_t *restrict g, const kac_rank *restrict weak,
                         const kac_rank *restrict strong, unsigned kband, unsigned kgate,
                         int64_t tot, unsigned len, int32_t fix, kac_reading *restrict rd)
{
    uint64_t wsse = 0, ssse = 0;
    unsigned wcnt = 0, scnt = 0, i;
    uint64_t gws = 0, gss = 0;
    unsigned gwc = 0, gsc = 0;
    int32_t  whole;

    for (i = 0; i < kband && i < weak->n; i++) {
        wsse += (uint64_t)weak->sse[i];
        wcnt += weak->cnt[i];
        if (i < kgate) {
            gws += (uint64_t)weak->sse[i];
            gwc += weak->cnt[i];
        }
    }
    for (i = 0; i < kband && i < strong->n; i++) {
        ssse += (uint64_t)strong->sse[i];
        scnt += strong->cnt[i];
        if (i < kgate) {
            gss += (uint64_t)strong->sse[i];
            gsc += strong->cnt[i];
        }
    }
    whole       = vbr_level(g, (uint64_t)tot, len);
    rd->low     = ((gwc == 0u) ? whole : vbr_level(g, gws, gwc)) + fix;
    rd->high    = ((gsc == 0u) ? whole : vbr_level(g, gss, gsc)) + fix;
    rd->band_lo = ((wcnt == 0u) ? whole : vbr_level(g, wsse, wcnt)) + fix;
    rd->band_hi = ((scnt == 0u) ? whole : vbr_level(g, ssse, scnt)) + fix;
    rd->psnr    = whole + fix;
    /* The middle band is the slices in neither of the other two: what is left
     * of the block once both ends are taken out. */
    rd->mid = (wcnt == 0u || scnt == 0u)
                  ? whole
                  : vbr_level(g, (uint64_t)tot - wsse - ssse, len - wcnt - scnt);
    rd->mid += fix;
}

/* Copy the handle for a probe, skipping alt_out: emission state is written
 * whole before it is read. */
static void probe_copy(kac_t *restrict pr, const kac_t *restrict g)
{
    memcpy(pr, g, offsetof(kac_t, alt_out));
}

/* Measure `tier` on this block and write its readings to `rd`. With `ms`, code
 * the pair as mid/side, merge each slice, and measure the decoder's output.
 * With `est_ms`, a forced mid/side run estimates the split form instead of
 * coding it, so no left/right data is consulted. The nominal ladder is not
 * used to predict the step: T2 to T1 varies with how quiet the material is,
 * and a single assumed step can oscillate between tiers. */
static int tier_measure(kac_t *restrict g, kac_t *restrict pr, int tier, int ms, int est_ms,
                        int form_cand, uint8_t *restrict scratch, unsigned len,
                        kac_reading *restrict rd)
{
    kac_rank weak, strong;
    size_t   at = 0;
    unsigned j, ns, kband, kgate;
    int      rc      = 0;
    int64_t  tot     = 0;
    uint32_t totclip = 0;

    probe_copy(pr, g);
    weak.n   = 0;
    strong.n = 0;
    if (kac_use_tier(pr, tier) != 0)
        return KAC_E_ARG;
    /* Measurements use the cheapest search; measure_fix() corrects them. */
    pr->beam_w = 0u;
    if (ms)
        kac_ms_split(pr->sbuf[0], pr->sbuf[1], len);
    /* Rank one entry past each band so the band and its boundary share a pass. */
    ns    = (len + pr->n - 1u) / pr->n;
    kband = (ns * KAC_BAND_PCT + 50u) / 100u;
    kgate = (ns * KAC_GATE_PCT + 50u) / 100u;
    if (kband > KAC_BAND_KMAX)
        kband = KAC_BAND_KMAX;
    if (kgate > kband)
        kgate = kband;

    for (j = 0; j < len && rc == 0; j += pr->n) {
        unsigned take    = len - j;
        int64_t  sse_at  = pr->sse;
        uint32_t clip_at = pr->clip;
        int64_t  e;
        if (take > pr->n)
            take = pr->n;
        /* Estimated tiers need no coding; the coded form is always measured for
         * real because it decides the block's bytes. A forced mid/side run
         * estimates after the split, so left/right samples never enter. */
        if (est_reading(g) && (!ms || est_ms)) {
            /* The two channels are independent chains, so one loop advances
             * both and hides the recurrence latency. */
            if (pr->channels == 2) {
                int64_t e0, e1;

                est_slice2(pr, j, take, form_cand, &e0, &e1);
                e = e0 + e1;
            } else {
                e = est_slice(pr, 0u, j, take, form_cand);
            }
            /* Merge turns (eM, eS) into (eM+eS, eM-eS): the squared errors add
             * to twice the split-domain sum. Merge clipping is not counted,
             * exactly as the left/right estimate does not count coding clip. */
            if (ms)
                e *= 2;
        } else {
            rc = encode_slice(pr, scratch + at, j, take, ms);
            at += (size_t)pr->channels * KAC_SLICE_SIZE;
            if (rc != 0)
                break;
            if (ms) {
                uint32_t c;
                kac_ms_merge(pr->sbuf[0] + j, pr->sbuf[1] + j, take);
                e = block_error(g, pr, j, take, &c);
                totclip += c;
            } else {
                e = pr->sse - sse_at;
                totclip += pr->clip - clip_at;
            }
        }
        tot += e;
        keep_cut(strong.sse, strong.cnt, &strong.n, kband + 1u, e, take);
        keep_top(weak.sse, weak.cnt, &weak.n, kband + 1u, e, take);
    }
    if (rc != 0)
        return rc;
    reading_fold(pr, &weak, &strong, kband, kgate, tot, len, measure_fix(g, tier), rd);
    rd->sse   = tot;
    rd->clip  = totclip;
    rd->bytes = at;
    return 0;
}

/* Measure `t` in left/right form and store the six readings. The channel form is
 * decided later, at the chosen tier, because measuring both forms everywhere is
 * too expensive. */
static int tier_measure_block(kac_t *restrict g, kac_t *restrict pr, int t, uint8_t *restrict out,
                              unsigned len)
{
    kac_reading rd;
    int         msm = (g->ms_mode == KAC_MS_MS && g->channels == 2);
    int         rc  = tier_measure(g, pr, t, msm, msm, -1, out, len, &rd);

    if (rc != 0)
        return rc;
    g->tier_psnr[t]     = rd.psnr;
    g->tier_psnr_mid[t] = rd.mid;
    g->tier_band_lo[t]  = rd.band_lo;
    g->tier_band_hi[t]  = rd.band_hi;
    g->tier_low[t]      = rd.low;
    g->tier_high[t]     = rd.high;
    return 0;
}

/* Code all slices at the current tier into `dst`, advancing `*bytes`. For
 * mid/side, merge and measure each slice as it is coded. */
static int encode_slices(kac_t *restrict g, const kac_t *restrict orig, uint8_t *restrict dst,
                         unsigned len, int ms, size_t *restrict bytes, int64_t *restrict sse,
                         uint32_t *restrict clip)
{
    const unsigned n = g->n;
    unsigned       i;

    for (i = 0; i < len; i += n) {
        unsigned take = len - i;
        int      rc;

        if (take > n)
            take = n;
        rc = encode_slice(g, dst + *bytes, i, take, ms);
        if (rc != 0)
            return rc;
        *bytes += (size_t)g->channels * KAC_SLICE_SIZE;
        if (ms) {
            uint32_t c;
            kac_ms_merge(g->sbuf[0] + i, g->sbuf[1] + i, take);
            *sse += block_error(orig, g, i, take, &c);
            *clip += c;
        }
    }
    return 0;
}

/* The one outer loop. Both modes move the same aim: -V lifts it when the file is
 * more than KAC_V_TOL_UP below its rung, and ABR turns the rate error into a
 * level step. A step is capped in its own direction by the block's bands; a
 * zero step and a zero peak move nothing. */
static void vbr_loop(kac_t *restrict g, uint64_t bytes, uint32_t pos0, unsigned len, int tier)
{
    int32_t step = 0;

    if (g->vbr_mode == KAC_VBR_ABR) {
        uint64_t den  = (uint64_t)len * (uint64_t)g->channels;
        int32_t  inst = (int32_t)((uint64_t)bytes * 8000u / den);
        int32_t  cum  = (int32_t)((g->data_bytes + (uint64_t)bytes) * 8000u /
                                ((uint64_t)(pos0 + len) * (uint64_t)g->channels));
        int32_t  ierr = g->vbr_target - inst;
        int32_t  cerr = g->vbr_target - cum;
        int32_t  tol  = g->vbr_target * KAC_ABR_TOL_PERMILLE / 1000;
        /* Out of tolerance the cumulative error counts twice: one block's
         * instantaneous error must not hold the file short of its target. */
        int32_t want = ((cerr > tol || cerr < -tol) ? ierr + 2 * cerr : ierr / 6 + cerr);

        if (want > tol || want < -tol || cerr > tol || cerr < -tol)
            step = (int32_t)((int64_t)want * vbr_slope(g) / 10000);
    } else {
        int32_t cum  = vbr_level(g, (uint64_t)g->sse, pos0 + len);
        int32_t tgt  = g->vbr_target - ladder_drop_mdb(g);
        int32_t over = cum - tgt;

        if (over < -KAC_V_TOL_UP)
            step = (tgt - cum) / KAC_V_STEP;
    }

    if (step != 0) {
        int32_t cap = (step >= 0) ? aim_slack(g, tier) : aim_fall(g, tier);

        if (step > cap)
            step = cap;
        else if (step < -cap)
            step = -cap;
        g->vbr_psnr += step;
        if (g->vbr_psnr > KAC_AIM_MAX_MDB)
            g->vbr_psnr = KAC_AIM_MAX_MDB;
        else if (g->vbr_psnr < 0)
            g->vbr_psnr = 0;
    }
}

/* Measure tiers around the previous block's tier, widening until one reading is
 * below the aim and one at or above it. At least two tiers are measured because
 * readings can invert, and the window stays inside tier_limits. */
static int measure_window(kac_t *restrict g, kac_t *restrict probe, uint8_t *restrict out,
                          unsigned len)
{
    int32_t aim = g->vbr_psnr - ladder_drop_mdb(g);
    int     t, lo, hi;

    for (t = KAC_TIER_MIN; t <= KAC_TIER_MAX; t++) {
        g->tier_psnr[t]     = KAC_UNMEASURED;
        g->tier_psnr_mid[t] = KAC_UNMEASURED;
        g->tier_band_lo[t]  = KAC_UNMEASURED;
        g->tier_band_hi[t]  = KAC_UNMEASURED;
        g->tier_low[t]      = KAC_UNMEASURED;
        g->tier_high[t]     = KAC_UNMEASURED;
    }

    /* The previous block's tier, or the seeded nominal tier on block 0, brought
     * inside the permitted span: a run whose limits moved under it, or whose
     * opening tier came from the nominal column, can leave it outside. */
    lo = g->cur_tier;
    if (lo < g->tier_lo)
        lo = g->tier_lo;
    if (lo > g->tier_hi)
        lo = g->tier_hi;
    if (lo < g->tier_hi) {
        hi = lo + 1;
    } else if (lo > g->tier_lo) {
        hi = lo;
        lo = lo - 1;
    } else {
        hi = lo;
    }

    for (;;) {
        for (t = lo; t <= hi; t++) {
            int rc;
            if (g->tier_psnr[t] != KAC_UNMEASURED)
                continue;
            rc = tier_measure_block(g, probe, t, out, len);
            if (rc != 0)
                return rc;
        }
        if ((lo == g->tier_lo || tier_level(g, lo) < aim) &&
            (hi == g->tier_hi || tier_level(g, hi) >= aim))
            break;
        if (lo > g->tier_lo && tier_level(g, lo) >= aim)
            lo--;
        if (hi < g->tier_hi && tier_level(g, hi) < aim)
            hi++;
    }

    /* vbr_slope() reads the two ends of the permitted span, so ABR measures
     * them however narrow the window turned out to be. A -V aim is a level and
     * needs only the window. */
    if (g->vbr_mode == KAC_VBR_ABR) {
        const int ends[2] = {g->tier_lo, g->tier_hi};
        int       j;
        for (j = 0; j < 2; j++) {
            int rc;
            if (g->tier_psnr[ends[j]] != KAC_UNMEASURED)
                continue;
            rc = tier_measure_block(g, probe, ends[j], out, len);
            if (rc != 0)
                return rc;
        }
    }
    return 0;
}

/* Choose left/right (0) or mid/side (1). `lr` is the block's reading in the
 * form it arrived in; mid/side is measured only at the chosen tier and must
 * beat that reading by kac_ms_margin_mdb[tier]. Mono has no second form. */
static int pick_form(kac_t *restrict g, kac_t *restrict probe, int tier, uint8_t *restrict out,
                     unsigned len, int32_t lr, int *restrict ms)
{
    *ms = 0;
    if (g->channels != 2)
        return 0;
    if (g->ms_mode == KAC_MS_MS) {
        /* Forced: the block is sent in the coded form, and there is nothing to
         * measure, because the measurement is what would have decided. */
        *ms = 1;
        return 0;
    }
    if (g->ms_mode == KAC_MS_AUTO && g->form != KAC_FORM_NONE) {
        kac_reading rd;
        int32_t     base;
        int         est = est_reading(g);
        /* The level's form column states what this measurement costs: a
         * positive value is the candidate rows the estimate walks, and
         * KAC_FORM_SEP is a real coding. An estimate is only usable when the
         * run measures by estimate at all. */
        int est_form  = (g->form >= 1 && g->form <= KAC_FORM_MAX_ROWS) && est;
        int form_cand = (g->form >= 1 && g->form <= KAC_FORM_MAX_ROWS) ? g->form : -1;
        /* The measurement writes its slices past the tier byte. The block is
         * coded again in the chosen form, so these bytes are scratch. */
        int rc = tier_measure(g, probe, tier, 1, est_form, form_cand, out + 1, len, &rd);

        if (rc != 0)
            return rc;
        /* Bring an estimated left/right reading up to a real coding before
         * comparing, or the margin would be measured from a low baseline. Two
         * estimates are already on one scale. */
        base = lr;
        if (!est_form)
            base += est_shortfall_mdb(g, tier, est);
        if (rd.psnr > base + kac_ms_margin_mdb[tier])
            *ms = 1;
    }
    return 0;
}

/* Emit [tier byte][slices]. Mid/side runs on the measurement scratch and
 * stages bytes in its alt_out; left/right writes directly and commits the
 * handle. */
static int emit_block(kac_t *restrict g, kac_t *restrict trial, uint8_t *restrict out, unsigned len,
                      int tier, int ms, uint32_t pos0, size_t *restrict bytes)
{
    /* Nothing has coded into the handle yet; totals still start at this block. */
    int64_t  sse0  = g->sse;
    uint32_t clip0 = g->clip;
    unsigned i;
    int64_t  ms_sse;
    uint32_t ms_clip;
    int      rc;

    *bytes = 1u;
    if (ms) {
        ms_sse  = 0;
        ms_clip = 0;
        probe_copy(trial, g);
        kac_ms_split(trial->sbuf[0], trial->sbuf[1], len);
        trial->alt_out[0] = (uint8_t)tier | KAC_BLOCK_MS;
        rc = encode_slices(trial, g, trial->alt_out, len, 1, bytes, &ms_sse, &ms_clip);
        if (rc != 0)
            return rc;
        trial->pos = pos0 + len;
        *g         = *trial;
        g->sse     = sse0 + ms_sse;
        g->clip    = clip0 + ms_clip;
        for (i = 0; i < *bytes; i++)
            out[i] = g->alt_out[i];
    } else {
        out[0] = (uint8_t)tier;
        rc     = encode_slices(g, NULL, out, len, 0, bytes, NULL, NULL);
        if (rc != 0)
            return rc;
        g->pos = pos0 + len;
    }
    return 0;
}

/* Encode one chunk: measure, pick tier and form, code, then run the outer loop.
 * Pinned tier with no form search skips measurement. Tier changes swap tables;
 * LMS state is continuous, so no extra signalling is needed. */
static int encode_block(kac_t *restrict g, uint8_t *restrict out, unsigned len,
                        size_t *restrict outbytes)
{
    int      tier;
    int      ms = 0;
    int      rc;
    uint32_t pos0;
    uint32_t k     = 0;
    size_t   bytes = 0;
    kac_t    probe; /* the tier measurement's working copy */

    if (len == 0 || len > KAC_CHUNK_SAMPLES)
        return KAC_E_ARG;

    pos0 = g->pos;
    if (index_point_at(g, &k)) {
        /* The tier byte precedes its slices, at a known data offset. */
        if (!g->index_off || (size_t)k >= g->index_off_entries)
            return KAC_E_INDEX;
        g->index_off[k] = (uint32_t)g->data_bytes + 1u; /* first slice, past the tier byte */
    }

    if (g->vbr_mode == KAC_VBR_OFF && (g->ms_mode != KAC_MS_AUTO || g->form == KAC_FORM_NONE)) {
        /* Pinned tier with neither tier nor form left to measure: the form is
         * forced by -m, or the level's form column is none. This is the
         * low-delay path. */
        tier = g->tier;
        ms   = (g->channels == 2 && g->ms_mode == KAC_MS_MS) ? 1 : 0;
        if (kac_use_tier(g, tier) != 0)
            return KAC_E_ARG;
    } else {
        rc = measure_window(g, &probe, out, len);
        if (rc != 0)
            return rc;
        abr_seed(g);
        tier = vbr_pick(g, g->vbr_psnr - ladder_drop_mdb(g));
        if (kac_use_tier(g, tier) != 0)
            return KAC_E_ARG;
        rc = pick_form(g, &probe, tier, out, len, g->tier_psnr[tier], &ms);
        if (rc != 0)
            return rc;
    }
    rc = emit_block(g, &probe, out, len, tier, ms, pos0, &bytes);
    if (rc != 0)
        return rc;

    /* Run the mode's outer loop after the per-block engine. */
    if (g->vbr_mode == KAC_VBR_V || g->vbr_mode == KAC_VBR_ABR)
        vbr_loop(g, bytes, pos0, len, tier);

    g->blk_count++;
    g->data_bytes += bytes;
    if (pos0 == 0u) {
        /* Header Tier and form are block 0's. */
        g->tier    = tier;
        g->blk0_ms = ms;
    }
    *outbytes = bytes;
    return 0;
}

static size_t emit_preamble_placeholder(kac_t *restrict g, uint8_t *restrict out)
{
    size_t p = preamble_bytes(g);
    size_t i;
    for (i = 0; i < p; i++)
        out[i] = 0;
    g->prefix_done = 1;
    return p;
}

/* Leading bytes: a file gets a placeholder the caller patches, a stream gets
 * none because kac_write_stream_preamble() supplies them. */
static size_t emit_prefix(kac_t *restrict g, uint8_t *restrict out)
{
    if (g->prefix_done)
        return 0;
    if (g->stream) {
        g->prefix_done = 1;
        return 0;
    }
    return emit_preamble_placeholder(g, out);
}

/* Take `take` interleaved samples into the block buffer at `fill`, and fold
 * them into the peak the reported PSNR is measured against. */
static void collect(kac_t *restrict g, const int16_t *restrict pcm, uint32_t take)
{
    const unsigned chn    = (unsigned)g->channels;
    const int      stereo = (chn == 2);
    uint32_t       i;

    for (i = 0; i < take; i++) {
        int32_t l               = pcm[(size_t)i * chn];
        int32_t r               = stereo ? pcm[(size_t)i * chn + 1] : 0;
        g->sbuf[0][g->fill + i] = (int16_t)l;
        if (stereo)
            g->sbuf[1][g->fill + i] = (int16_t)r;
        if (l < 0)
            l = -l;
        if (r < 0)
            r = -r;
        if (l > g->peak)
            g->peak = (int32_t)l;
        if (r > g->peak)
            g->peak = (int32_t)r;
    }
}

static int feed(kac_t *restrict g, const int16_t *restrict pcm, uint32_t frames,
                uint8_t *restrict out, size_t *restrict outlen)
{
    size_t written = 0;

    if (!g || !out || !outlen || !pcm)
        return KAC_E_ARG;
    if (g->state != KAC_STATE_READY)
        return KAC_E_ARG;
    written = emit_prefix(g, out);

    while (frames > 0) {
        unsigned space = g->chunk_samples - g->fill;
        uint32_t take  = (frames < space) ? frames : space;

        collect(g, pcm, take);
        pcm += (size_t)take * g->channels;
        frames -= take;
        g->fill += take;
        g->total_in += take;
        if (g->total_in > KAC_MAX_TOTAL)
            return KAC_E_TOTAL;
        if (g->fill == g->chunk_samples) {
            size_t got = 0;
            int    rc  = encode_block(g, out + written, g->chunk_samples, &got);

            if (rc != 0)
                return rc;
            written += got;
        }
    }
    *outlen = written;
    return (int)written;
}

int kac_encode_buffer_interleaved(kac_t *restrict g, const int16_t *restrict pcm, uint32_t frames,
                                  uint8_t *restrict out, size_t *restrict outlen)
{
    return feed(g, pcm, frames, out, outlen);
}

int kac_encode_flush(kac_t *restrict g, uint8_t *restrict out, size_t *restrict outlen)
{
    size_t written = 0;

    if (!g || !out || !outlen)
        return KAC_E_ARG;
    if (g->state != KAC_STATE_READY)
        return KAC_E_ARG;
    written = emit_prefix(g, out);
    if (g->total_in == 0)
        return KAC_E_TOTAL;
    if (g->loop_end > g->loop_start && (g->loop_start >= g->total_in || g->loop_end > g->total_in))
        return KAC_E_LOOP;

    if (g->fill > 0) {
        /* Only real samples are coded; zero residual fills the rest. A file's
         * reader never reaches it, a stream decodes it as tail padding. */
        size_t got = 0;
        int    rc  = encode_block(g, out + written, g->fill, &got);
        if (rc != 0)
            return rc;
        written += got;
        g->fill = 0;
    }

    /* A loop snapshot can only be validated after the block containing
     * LoopStart has been coded, which is past the trailing block. */
    if (g->loop_end > g->loop_start && !g->loop_snap_valid)
        return KAC_E_LOOP;

    if (g->stride != 0) {
        uint32_t entries = index_entries(g);
        uint32_t k;
        size_t   snap_bytes = (size_t)entries * g->channels * KAC_SNAP_SIZE;
        if (!g->index_store || snap_bytes > g->index_store_bytes)
            return KAC_E_INDEX;
        for (k = 0; k < entries; k++) {
            /* KAC_SPEC.md Section 3.4: the tier byte in front of each block fixes the
             * offsets, so they were recorded block by block while encoding. */
            if (!g->index_off || (size_t)k >= g->index_off_entries)
                return KAC_E_INDEX;
            kac_put32(out + written + (size_t)k * 4u, g->index_off[k]);
        }
        written += (size_t)entries * 4u;
        {
            size_t i;
            for (i = 0; i < snap_bytes; i++)
                out[written + i] = g->index_store[i];
        }
        written += snap_bytes;
    }
    g->state = KAC_STATE_FLUSHED;
    *outlen  = written;
    return (int)written;
}

/* 36-byte header shared by file and stream. A stream zeroes lengths, clears
 * SEEK/LOOP, puts its chunk length in IndexStride, and carries only INIT. */
static void write_header(const kac_t *restrict g, uint8_t *restrict buf, int stream)
{
    int looping = (!stream && g->loop_end > g->loop_start);
    /* The flag says the file may carry mid/side somewhere, so a decoder that
     * cannot decode it refuses the file. Block 0's actual form is in its byte. */
    int coded = (g->channels == 2 && g->ms_mode != KAC_MS_LR);
    buf[0]    = 'K';
    buf[1]    = 'A';
    buf[2]    = 'C';
    buf[3]    = 0;
    buf[4]    = (uint8_t)KAC_VERSION;
    buf[5] =
        (uint8_t)((g->init_snap_valid ? KAC_FLAG_INIT : 0u) | (looping ? KAC_FLAG_LOOP : 0u) |
                  ((!stream && g->stride != 0) ? KAC_FLAG_SEEK : 0u) | (coded ? KAC_FLAG_MS : 0u));
    buf[6] = (uint8_t)g->channels;
    buf[7] = (uint8_t)((unsigned)g->tier | (g->blk0_ms ? KAC_BLOCK_MS : 0u));
    kac_put32(buf + 8, (uint32_t)g->samplerate);
    kac_put32(buf + 12, stream ? 0u : g->total_in);
    kac_put32(buf + 16, looping ? g->loop_start : 0u);
    kac_put32(buf + 20, looping ? g->loop_end : 0u);
    /* DataBytes is the sum of the block walk, not a closed form. */
    kac_put32(buf + 24, stream ? 0u : (uint32_t)g->data_bytes);
    /* A stream states its chunk length here; a file states its seek stride. */
    kac_put32(buf + 28, stream ? g->chunk_samples : g->stride);
    kac_put32(buf + 32, stream ? 0u : index_entries(g));
}

/* Snapshot order follows the flags: INIT, then LOOP. A stream never has LOOP. */
static void write_snaps(const kac_t *restrict g, uint8_t *restrict buf, int stream)
{
    size_t   off = KAC_HDR_SIZE;
    unsigned c;
    if (g->init_snap_valid) {
        for (c = 0; c < (unsigned)g->channels; c++)
            kac_snap_write(buf + off + (size_t)c * KAC_SNAP_SIZE, &g->init_snap[c]);
        off += (size_t)g->channels * KAC_SNAP_SIZE;
    }
    if (!stream && g->loop_end > g->loop_start) {
        for (c = 0; c < (unsigned)g->channels; c++)
            kac_snap_write(buf + off + (size_t)c * KAC_SNAP_SIZE, &g->loop_snap[c]);
    }
}

int kac_write_preamble(const kac_t *restrict g, uint8_t *restrict buf, size_t *restrict len)
{
    if (!g || !buf || !len)
        return KAC_E_ARG;
    if (g->state != KAC_STATE_FLUSHED)
        return KAC_E_ARG;
    write_header(g, buf, 0);
    write_snaps(g, buf, 0);
    *len = preamble_bytes(g);
    return 0;
}

int kac_write_stream_preamble(const kac_t *restrict g, uint8_t *restrict buf, size_t *restrict len)
{
    if (!g || !buf || !len)
        return KAC_E_ARG;
    if (!g->stream || (g->state != KAC_STATE_READY && g->state != KAC_STATE_FLUSHED))
        return KAC_E_ARG;
    /* The header's Tier field is block 0's tier and form, and those are settled
     * only when block 0 has been coded. */
    if (g->blk_count == 0u)
        return KAC_E_ARG;
    write_header(g, buf, 1);
    write_snaps(g, buf, 1);
    *len = preamble_bytes(g);
    return 0;
}

unsigned kac_get_tier(const kac_t *restrict g)
{
    return (unsigned)g->tier;
}

int kac_set_quality(kac_t *restrict g, int level)
{
    if (!g || g->state != KAC_STATE_FRESH)
        return KAC_E_ARG;
    if (level < 0 || level > KAC_QUALITY_MAX)
        return KAC_E_ARG;
    g->quality = level;
    return 0;
}

int kac_set_report(kac_t *restrict g, int enable)
{
    if (!g || g->state != KAC_STATE_FRESH)
        return KAC_E_ARG;
    g->report = enable ? 1 : 0;
    return 0;
}

int kac_set_brute(kac_t *restrict g, unsigned level)
{
    if (!g || g->state != KAC_STATE_FRESH)
        return KAC_E_ARG;
    if (level > 2u)
        return KAC_E_ARG;
    g->brute_level = level;
    return 0;
}

int kac_set_ms_mode(kac_t *restrict g, int mode)
{
    if (!g || g->state != KAC_STATE_FRESH)
        return KAC_E_ARG;
    if (mode != KAC_MS_AUTO && mode != KAC_MS_LR && mode != KAC_MS_MS)
        return KAC_E_ARG;
    g->ms_mode = mode;
    return KAC_OK;
}

unsigned kac_get_quality(const kac_t *restrict g)
{
    return (unsigned)g->quality;
}
uint32_t kac_get_total(const kac_t *restrict g)
{
    return g->total_in;
}
double kac_get_brate(const kac_t *restrict g)
{
    if (!g || g->tier < KAC_TIER_MIN || g->tier > KAC_TIER_MAX)
        return 0.0; /* the tier is not fixed before kac_init_params */
    return 64.0 / (double)kac_tiers[(unsigned)g->tier - 1u].n;
}
double kac_get_psnr(const kac_t *restrict g)
{
    int32_t mdb;

    if (g->sse <= 0 || g->peak <= 0 || g->total_in == 0)
        return 99.0;
    mdb = 2 * kac_log10_mdb((uint64_t)g->peak) +
          kac_log10_mdb((uint64_t)g->total_in * (uint64_t)g->channels) -
          kac_log10_mdb((uint64_t)g->sse);
    if (mdb >= KAC_MDB_MAX)
        return 99.0;
    return (double)mdb / 1000.0;
}
double kac_get_clip_rate(const kac_t *restrict g)
{
    double n = (double)g->total_in * g->channels;
    return (n > 0.0) ? (double)g->clip / n : 0.0;
}
double kac_get_pred_over_rate(const kac_t *restrict g)
{
    double n = (double)g->total_in * g->channels;
    return (n > 0.0) ? (double)g->pred_over / n : 0.0;
}
double kac_get_w_limit_rate(const kac_t *restrict g)
{
    double n = (double)g->total_in * g->channels;
    return (n > 0.0) ? (double)g->w_limit / n : 0.0;
}

/* First report item: what the file actually cost. */
double kac_get_brate_achieved(const kac_t *restrict g)
{
    double ns;
    if (!g || g->n == 0 || g->total_in == 0)
        return 0.0;
    ns = (double)g->total_in * (double)g->channels;
    return (ns > 0.0) ? (double)g->data_bytes * 8.0 / ns : 0.0;
}

/* Second report item: how far the file sits from its nominal point, in the
 * mode's own unit (dB for -V, bit/sample for --abr). A quality target that
 * saturates at either end of the ladder shows up here and nowhere else. */
double kac_get_vbr_offset(const kac_t *restrict g)
{
    if (!g || g->vbr_mode == KAC_VBR_OFF)
        return 0.0;
    if (g->vbr_mode == KAC_VBR_V)
        return kac_get_psnr(g) - (double)g->vbr_target / 1000.0;
    return kac_get_brate_achieved(g) - (double)g->vbr_target / 1000.0;
}

int kac_get_vbr_mode(const kac_t *restrict g)
{
    return g ? g->vbr_mode : KAC_VBR_OFF;
}

double kac_get_vbr_target(const kac_t *restrict g)
{
    /* the field is milli-scaled; the caller's unit is not */
    return g ? (double)g->vbr_target / 1000.0 : 0.0;
}

unsigned kac_get_vbr_nominal_tier(const kac_t *restrict g)
{
    if (!g || g->vbr_mode == KAC_VBR_OFF)
        return 0u;
    if (g->vbr_mode == KAC_VBR_V)
        return kac_v_nominal_tier1000(g->vbr_n);
    /* vbr_target is in milli-bit per sample, the unit kac_tier_for_brate_mb
     * takes; kac_tier_for_brate() is the double-valued wrapper and would read
     * it as bit per sample. */
    return (unsigned)kac_tier_for_brate_mb(g->vbr_target);
}

uint32_t kac_get_data_bytes(const kac_t *restrict g)
{
    if (!g || g->n == 0)
        return 0u;
    return (uint32_t)g->data_bytes;
}
