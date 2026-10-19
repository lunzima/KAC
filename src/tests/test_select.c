/* test_select.c -- tier selection: bit rate is a pure lookup, the per-block
 * engine reaches a target PSNR, the window a run may give a block is a pair of
 * lookups too, and the default is the -V ladder's rung KAC_DEFAULT_V. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "kac.h"
#include "kac_test_util.h"

static void expect(int ok, const char *what, double got, double want)
{
    kac_tu_cases++;
    if (ok)
        return;
    kac_tu_fail++;
    fprintf(stderr, "FAIL: %s -> %.4f (want %.4f)\n", what, got, want);
}

static double tier_rate(int t)
{
    return 64.0 / (double)kac_tiers[t - 1].n;
}

static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* The block walk of KAC_SPEC.md Section 3.1, independent of the decoder's copy
 * so that agreement between the two is evidence rather than a tautology, and
 * counting each block's tier. The files here carry no snapshot, so the data
 * area starts at the header. Returns the block count, or 0 if the walk fails. */
static unsigned walk_blocks(const uint8_t *img, size_t len, unsigned hist[KAC_TIER_MAX + 1])
{
    const uint8_t *data  = img + KAC_HDR_SIZE;
    uint32_t       total = get32(img + 12), db = get32(img + 24), cov = 0, tb = 0, n = 0;
    unsigned       ch     = img[6], i;
    uint32_t       blkmax = kac_block_for_rate(get32(img + 8));

    for (i = 0; i <= (unsigned)KAC_TIER_MAX; i++)
        hist[i] = 0;
    if (len < KAC_HDR_SIZE || (uint64_t)KAC_HDR_SIZE + db > (uint64_t)len)
        return 0;
    while (cov < total) {
        uint32_t blk = total - cov;
        unsigned b, nn;
        if (blk > blkmax)
            blk = blkmax;
        if (tb >= db)
            return 0;
        b = data[tb] & KAC_TIER_MASK;
        if (b < (unsigned)KAC_TIER_MIN || b > (unsigned)KAC_TIER_MAX ||
            (data[tb] & KAC_TIER_SPARE) != 0u)
            return 0;
        hist[b]++;
        nn = kac_tiers[b - 1u].n;
        tb = tb + 1u + ((blk + nn - 1u) / nn) * KAC_SLICE_SIZE * ch;
        cov += blk;
        n++;
    }
    return (cov == total && tb == db) ? n : 0;
}

int main(void)
{
    const uint32_t frames = 88200;
    const unsigned ch     = 2;
    int16_t       *pcm;
    uint32_t       i;
    kac_t          g;

    pcm = (int16_t *)malloc((size_t)frames * ch * sizeof(int16_t));
    if (!pcm)
        return 1;
    for (i = 0; i < frames * ch; i++) {
        double t = (double)(i / ch) / 44100.0;
        double v = sin(2.0 * 3.14159265358979 * 440.0 * t) * 15000.0 +
                   sin(2.0 * 3.14159265358979 * 1319.0 * t) * 5000.0;
        pcm[i] = (int16_t)v;
    }

    printf("== bit rate to tier, no trial encode ==\n");
    {
        /* -b uses the floor lookup and -B the ceiling; between tiers they can
         * cross and leave no legal window tier. */
        double   targets[6];
        int      want[6];
        int      want_ceil[6];
        unsigned k;
        targets[0]   = 1.0;
        targets[1]   = 2.2;
        targets[2]   = 4.0;
        targets[3]   = 4.3;
        targets[4]   = 6.4;
        targets[5]   = 9.0;
        want[0]      = 2;
        want[1]      = 2;
        want[2]      = 3;
        want[3]      = 4;
        want[4]      = 6;
        want[5]      = 6;
        want_ceil[0] = 2;
        want_ceil[1] = 3;
        want_ceil[2] = 4;
        want_ceil[3] = 5;
        want_ceil[4] = 6;
        want_ceil[5] = 6;
        for (k = 0; k < 6; k++) {
            int    sel  = kac_tier_for_brate(targets[k]);
            int    selc = kac_tier_ceil_brate(targets[k]);
            double got  = tier_rate(sel);
            char   msg[64];
            sprintf(msg, "bit rate %.2f selects T%d", targets[k], want[k]);
            expect(sel == want[k] && (targets[k] - got <= 1.07 + 1e-9 ||
                                      sel == KAC_TIER_FIXED_MIN || sel == KAC_TIER_FIXED_MAX),
                   msg, (double)sel, (double)want[k]);
            expect(got <= targets[k] + 1e-9 || sel == 2, "rate does not exceed target", got,
                   targets[k]);
            sprintf(msg, "bit rate %.2f floors at T%d", targets[k], want_ceil[k]);
            expect(selc == want_ceil[k], msg, (double)selc, (double)want_ceil[k]);
            expect(selc >= sel, "the ceiling lookup never falls below the floor one", (double)selc,
                   (double)sel);
            printf("  ok  %.2f bit/sample -> T%d (%.4f) .. T%d\n", targets[k], sel, got, selc);
        }
        /* A rate at a tier's own rate is that tier from either side, and the two
         * ends of the span are the same tier from either side as well. */
        expect(kac_tier_ceil_brate(6.4) == 6 && kac_tier_for_brate(6.4) == 6,
               "6.400 is T6 from either side", (double)kac_tier_ceil_brate(6.4),
               (double)kac_tier_for_brate(6.4));
        expect(kac_tier_ceil_brate(1.0) == 2 && kac_tier_ceil_brate(99.0) == 6,
               "a rate past either end is held at that end", (double)kac_tier_ceil_brate(99.0),
               (double)kac_tier_ceil_brate(1.0));
    }

    printf("== target PSNR ==\n");
    {
        /* --psnr uses the same per-block engine as -V with a dB target. The
         * file must reach that level; the fixed-tier sweep is informational. */
        double   qs[3];
        unsigned k;
        qs[0] = 30.0;
        qs[1] = 45.0;
        qs[2] = 70.0;
        for (k = 0; k < 3; k++) {
            enc_opts o;
            uint8_t *image;
            size_t   len     = 0;
            int      sel_exh = KAC_TIER_MAX, t;
            double   got;
            unsigned hist[KAC_TIER_MAX + 1];

            /* the lowest tier whose whole-file PSNR reaches the target, or the
             * widest tier for "none does". T1 is the w = 1 one and no fixed
             * selector reaches it. */
            for (t = 2; t <= KAC_TIER_MAX; t++) {
                enc_opts oe;
                uint8_t *ie;
                size_t   le = 0;
                int      hit;
                enc_opts_default(&oe);
                oe.tier = t;
                ie      = enc_image(pcm, frames, ch, 44100, &oe, &le, &g);
                if (!ie)
                    break;
                hit = kac_get_psnr(&g) >= qs[k];
                free(ie);
                if (hit) {
                    sel_exh = t;
                    break;
                }
            }

            enc_opts_default(&o);
            o.tier = 0;
            o.psnr = qs[k];
            image  = enc_image(pcm, frames, ch, 44100, &o, &len, &g);
            expect(image != NULL, "quality encode", qs[k], 0.0);
            if (!image)
                continue;
            got = kac_get_psnr(&g);
            /* The data area is a sequence of blocks, each with its tier byte in
             * front of its slices: the walk of Section 3.1 closes on it. */
            expect(walk_blocks(image, len, hist) > 0, "the data area is a block sequence", qs[k],
                   0.0);
            expect(got + 0.5 >= qs[k], "target %.0f dB is reached", qs[k], got);
            printf("  ok  target %.0f dB -> %.2f dB at %.4f bit/sample "
                   "(a whole-file T%d reaches it too)\n",
                   qs[k], got, kac_get_brate_achieved(&g), sel_exh);
            free(image);
        }
    }

    printf("== the window a run may give a block ==\n");
    {
        /* kac_set_tier_limits is what the CLI's -b and -B fold into: the block
         * takes the cheapest tier that reaches the aim inside the window it was
         * given. The histogram comes off the emitted file, not the decisions. */
        static const struct {
            int    lo, hi;
            double target;
        } w[3] = {{4, 4, 2.0}, {3, 5, 2.0}, {5, 6, 8.0}};
        unsigned k;

        for (k = 0; k < 3; k++) {
            enc_opts o;
            uint8_t *image;
            size_t   len = 0;
            unsigned hist[KAC_TIER_MAX + 1], t, nb, out = 0;

            enc_opts_default(&o);
            o.tier       = 0;
            o.vbr_mode   = KAC_VBR_V;
            o.vbr_target = w[k].target;
            o.tier_lo    = w[k].lo;
            o.tier_hi    = w[k].hi;
            image        = enc_image(pcm, frames, ch, 44100, &o, &len, &g);
            expect(image != NULL, "a windowed run encodes", w[k].target, 0.0);
            if (!image)
                continue;
            nb = walk_blocks(image, len, hist);
            expect(nb > 0, "the windowed file's walk closes", w[k].target, 0.0);
            for (t = 1; t <= (unsigned)KAC_TIER_MAX; t++) {
                if (t < (unsigned)w[k].lo || t > (unsigned)w[k].hi) {
                    expect(hist[t] == 0, "no block is given a tier outside the window", t,
                           (double)hist[t]);
                } else {
                    out += hist[t];
                }
            }
            expect(out == nb, "every block is inside the window", (double)out, (double)nb);
            printf("  ok  window T%d..T%d at -V %.1f -> T1..T7 %u %u %u %u %u %u %u\n", w[k].lo,
                   w[k].hi, w[k].target, hist[1], hist[2], hist[3], hist[4], hist[5], hist[6],
                   hist[7]);
            free(image);
        }
        { /* A window is a pair of ends and nothing else: one tier, or the whole
           * ladder when nothing was set. */
            kac_t p;
            kac_init(&p);
            expect(kac_set_tier_limits(&p, 4, 3) == KAC_E_ARG, "an inverted window is refused", 0.0,
                   0.0);
            expect(kac_set_tier_limits(&p, 0, 4) == KAC_E_ARG, "T0 is refused", 0.0, 0.0);
            expect(kac_set_tier_limits(&p, 3, 8) == KAC_E_ARG, "T8 is refused", 0.0, 0.0);
            expect(kac_set_tier_limits(&p, 4, 4) == KAC_OK, "a one-tier window is accepted", 0.0,
                   0.0);
            /* A selector that pins the tier is a window of one; one that gives the
             * tier back to the ladder opens the window again. */
            expect(kac_set_tier(&p, 2) == KAC_OK && p.tier_lo == 2 && p.tier_hi == 2,
                   "pinning the tier pins the window", (double)p.tier_lo, (double)p.tier_hi);
            expect(kac_set_vbr(&p, KAC_VBR_V, 5.0) == KAC_OK && p.tier_lo == KAC_TIER_MIN &&
                       p.tier_hi == KAC_TIER_MAX,
                   "a per-block selector opens it again", (double)p.tier_hi, (double)p.tier_lo);
            expect(kac_set_brate(&p, 4.0) == KAC_OK && p.tier_lo == 3 && p.tier_hi == 3,
                   "a rate pins it to the tier it selects", (double)p.tier_lo, 3.0);
        }
    }

    kac_init(&g);
    kac_init_params(&g);
    /* The default is the -V ladder's rung KAC_DEFAULT_V (kac.h), not a pinned
     * tier: -V is what the CLI recommends and what a caller that selects nothing
     * gets, so the tier the handle opens on is that rung's nominal one. */
    expect(kac_get_vbr_mode(&g) == KAC_VBR_V, "no target -> the default mode is -V",
           (double)kac_get_vbr_mode(&g), (double)KAC_VBR_V);
    expect(kac_get_vbr_target(&g) == kac_v_tier_psnr(KAC_DEFAULT_V),
           "no target -> the default target is rung KAC_DEFAULT_V", kac_get_vbr_target(&g),
           kac_v_tier_psnr(KAC_DEFAULT_V));
    expect(kac_get_tier(&g) == kac_v_nominal_tier(KAC_DEFAULT_V),
           "no target -> the handle opens on that rung's nominal tier", (double)kac_get_tier(&g),
           (double)kac_v_nominal_tier(KAC_DEFAULT_V));
    printf("  ok  no target -> -V %d (chunked, opening tier T%u, target %.1f dB)\n", KAC_DEFAULT_V,
           kac_get_tier(&g), kac_v_tier_psnr(KAC_DEFAULT_V));

    /* A handle pinned to one tier, configured but fed nothing. Its data
     * area is zero bytes; the closed form of Section 3.1 carries its own `- 1`
     * and wraps on an empty file, so this is what pins the guard around it. */
    kac_init(&g);
    expect(kac_set_tier(&g, 3) == 0, "T3 selector", 0.0, 0.0);
    expect(kac_init_params(&g) == 0, "init params", 0.0, 0.0);
    expect(kac_get_data_bytes(&g) == 0u, "no samples fed -> no data bytes",
           (double)kac_get_data_bytes(&g), 0.0);
    expect(kac_get_brate_achieved(&g) == 0.0, "no samples fed -> no achieved rate",
           kac_get_brate_achieved(&g), 0.0);
    printf("  ok  unfed T3 handle -> data bytes %u, achieved rate %.4f\n", kac_get_data_bytes(&g),
           kac_get_brate_achieved(&g));

    free(pcm);
    printf("tier selection checks %d\n", kac_tu_cases);
    return kac_tu_report("test_select", "checks");
}
