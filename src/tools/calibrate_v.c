/* calibrate_v.c -- the target-PSNR scan that solves the -V ladder's endpoint P5.
 *
 *   calibrate_v tiers <music dir>              one pinned tier per block, T1..T7
 *   calibrate_v scan <music dir> <t0> <t1> <dt>
 *                                               per-block target-PSNR scan
 *   calibrate_v abr  <music dir> <bit/sample>  per-block average-rate scan
 *
 * CSV on stdout, progress on stderr:
 *
 *   track,mode,arg,psnr,rate,T1,T2,T3,T4,T5,T6,ms,blocks
 *
 * `arg` is the tier, the target PSNR or the target rate depending on the mode;
 * T1..T6 are the block tier histogram of the emitted file and `blocks` is how
 * many blocks that histogram counted.
 *
 * The material is the fixed calibration set, listed rather than globbed, so
 * that a track going missing is an error instead of a quietly different median.
 * kissrain is measured but is not part of the ladder's median set: it is
 * band-limited, the pipeline this format serves downsamples such material
 * before the encoder sees it, and on 44.1 kHz input the encoder spends bits on
 * an empty band. Whichever analysis takes a median over tracks drops its row.
 * Every row comes from kac_encode_buffer_interleaved through the same handle
 * setup the tests use (tests/kac_test_util.h), i.e. from the implementation
 * that ships.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "wav_io.h"
#include "../tests/kac_test_util.h"

#define NTRACKS 9

static const char *k_tracks[NTRACKS] = {"beatit",   "croatian", "daolang2002", "dayu",   "kissrain",
                                        "polumnia", "rockyou",  "timetosay",   "zuixuan"};

static uint32_t get32_le(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* The block walk of KAC_SPEC.md Section 3.1, so the histogram a rung is judged
 * on comes from the file the encoder actually emitted. Returns the block count,
 * or 0 if the walk does not close -- which would make the histogram
 * untrustworthy. */
static unsigned tier_hist(const uint8_t *img, size_t len, unsigned hist[KAC_TIER_MAX + 1])
{
    uint32_t       total, db, cov = 0, tb = 0;
    unsigned       ch, n = 0, i, b, nn;
    const uint8_t *data;

    for (i = 0; i < (unsigned)KAC_TIER_MAX + 1u; i++)
        hist[i] = 0;
    if (len < KAC_HDR_SIZE)
        return 0;
    total = get32_le(img + 12);
    db    = get32_le(img + 24);
    ch    = img[6];
    if ((uint64_t)KAC_HDR_SIZE + db > (uint64_t)len)
        return 0;
    data = img + KAC_HDR_SIZE;
    if (img[5] & KAC_FLAG_INIT)
        data += (size_t)ch * KAC_SNAP_SIZE;
    if (img[5] & KAC_FLAG_LOOP)
        data += (size_t)ch * KAC_SNAP_SIZE;
    while (cov < total) {
        uint32_t blk = total - cov;
        if (blk > KAC_CHUNK_SAMPLES)
            blk = KAC_CHUNK_SAMPLES;
        if (tb >= db)
            return 0;
        /* bit 3 of a tier byte is the block's channel form, so the tier is the
         * rest of the byte; the bits above it carry nothing. */
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

static long now_ms(void)
{
    return (long)(1000.0 * (double)clock() / (double)CLOCKS_PER_SEC);
}

/* One per-block encode of an already loaded track at one target. */
static int run_one(const kac_wav *w, double target, kac_t *gh, size_t *len, uint8_t **img, long *ms)
{
    enc_opts o;
    long     t0;

    enc_opts_default(&o);
    o.tier     = 0; /* -V owns the tier decision */
    o.vbr_mode = KAC_VBR_V;
    o.vbr_psnr = target;
    t0         = now_ms();
    *img       = enc_image(w->pcm, w->frames, w->channels, w->rate, &o, len, gh);
    *ms        = now_ms() - t0;
    return *img ? 0 : -1;
}

static void emit(const char *track, const char *mode, const char *arg, const kac_t *g,
                 const uint8_t *img, size_t len, long ms)
{
    unsigned hist[KAC_TIER_MAX + 1];
    if (!tier_hist(img, len, hist)) {
        fprintf(stderr, "calibrate_v: %s %s %s: block walk does not close\n", track, mode, arg);
        exit(1);
    }
    printf("%s,%s,%s,%.4f,%.4f,%u,%u,%u,%u,%u,%u,%ld,%u\n", track, mode, arg, kac_get_psnr(g),
           kac_get_brate_achieved(g), hist[1], hist[2], hist[3], hist[4], hist[5], hist[6], ms,
           g->blk_count);
}

int main(int argc, char **argv)
{
    kac_wav     w[NTRACKS];
    kac_t       g;
    const char *dir;
    const char *only;
    int         i, j;
    uint32_t    limit;

    /* --limit N truncates every track to its first N frames. The 10 s stereo
     * basis is 441 000 frames; a figure measured on 60 s
     * material must not be divided down to that basis, because the quotient
     * is a scaled number, not a measurement on the basis the table is
     * written in: the quotient is a number about neither basis. */
    limit = 0;
    only  = NULL;
    for (i = 1; i + 1 < argc; i++) {
        if (!strcmp(argv[i], "--track")) {
            only = argv[i + 1];
            for (j = i; j + 2 <= argc; j++)
                argv[j] = argv[j + 2];
            argc -= 2;
            i--;
            continue;
        }
        if (!strcmp(argv[i], "--limit")) {
            long v = atol(argv[i + 1]);
            if (v <= 0) {
                fprintf(stderr, "calibrate_v: bad --limit\n");
                return 2;
            }
            limit = (uint32_t)v;
            for (j = i; j + 2 <= argc; j++)
                argv[j] = argv[j + 2];
            argc -= 2;
            i--;
        }
    }
    if (argc < 3) {
        fprintf(stderr, "usage: calibrate_v [--limit N] tiers <music dir>\n"
                        "       calibrate_v [--limit N] scan <music dir> <t0> <t1> <dt>\n"
                        "       calibrate_v [--limit N] abr  <music dir> <bit/sample>\n");
        return 2;
    }
    dir = argv[2];
    if (!strcmp(argv[1], "scan") && argc >= 6 &&
        (!(atof(argv[5]) > 0.0) || atof(argv[4]) < atof(argv[3]))) {
        fprintf(stderr, "calibrate_v: bad range\n");
        return 2;
    }
    for (i = 0; i < NTRACKS; i++) {
        char path[1024];
        if (only && strcmp(only, k_tracks[i]))
            continue;
        snprintf(path, sizeof path, "%s/%s.wav", dir, k_tracks[i]);
        if (wav_read(path, &w[i]) != 0) {
            fprintf(stderr, "calibrate_v: cannot read %s\n", path);
            return 1;
        }
        if (w[i].rate != 44100u || w[i].channels != 2) {
            fprintf(stderr, "calibrate_v: %s is %u Hz / %u ch, want 44100 / 2\n", k_tracks[i],
                    w[i].rate, w[i].channels);
            return 1;
        }
        if (limit && w[i].frames > limit)
            w[i].frames = limit;
        fprintf(stderr, "calibrate_v: %-12s %u frames\n", k_tracks[i], w[i].frames);
    }

    printf("# track,mode,arg,psnr,rate,T1,T2,T3,T4,T5,T6,ms,blocks\n");

    if (!strcmp(argv[1], "tiers")) {
        int t;
        for (i = 0; i < NTRACKS; i++) {
            if (only && strcmp(only, k_tracks[i]))
                continue;
            for (t = 1; t <= KAC_TIER_MAX; t++) {
                enc_opts o;
                uint8_t *img;
                size_t   len = 0;
                long     t0  = now_ms(), ms;
                char     arg[8];

                enc_opts_default(&o);
                o.tier = t;
                img    = enc_image(w[i].pcm, w[i].frames, w[i].channels, w[i].rate, &o, &len, &g);
                ms     = now_ms() - t0;
                if (!img) {
                    fprintf(stderr, "calibrate_v: %s T%d failed\n", k_tracks[i], t);
                    return 1;
                }
                snprintf(arg, sizeof arg, "%d", t);
                emit(k_tracks[i], "tier", arg, &g, img, len, ms);
                free(img);
            }
            fprintf(stderr, "calibrate_v: %-12s done\n", k_tracks[i]);
        }
        return 0;
    }

    if (!strcmp(argv[1], "scan")) {
        double t0 = atof(argv[3]), t1 = atof(argv[4]), dt = atof(argv[5]);
        double t;
        for (i = 0; i < NTRACKS; i++) {
            if (only && strcmp(only, k_tracks[i]))
                continue;
            for (t = t0; t <= t1 + 1e-9; t += dt) {
                uint8_t *img;
                size_t   len = 0;
                long     ms;
                char     arg[32];
                if (run_one(&w[i], t, &g, &len, &img, &ms) != 0) {
                    fprintf(stderr, "calibrate_v: %s @ %.2f dB failed\n", k_tracks[i], t);
                    return 1;
                }
                snprintf(arg, sizeof arg, "%.2f", t);
                emit(k_tracks[i], "scan", arg, &g, img, len, ms);
                free(img);
            }
            fprintf(stderr, "calibrate_v: %-12s done\n", k_tracks[i]);
        }
        return 0;
    }

    if (!strcmp(argv[1], "abr")) {
        double rate = atof(argv[3]);
        if (!(rate > 0.0)) {
            fprintf(stderr, "calibrate_v: bad rate\n");
            return 2;
        }
        for (i = 0; i < NTRACKS; i++) {
            if (only && strcmp(only, k_tracks[i]))
                continue;
            enc_opts o;
            uint8_t *img;
            size_t   len = 0;
            long     t0, ms;
            char     arg[32];
            enc_opts_default(&o);
            o.tier       = 0;
            o.vbr_mode   = KAC_VBR_ABR;
            o.vbr_target = rate;
            t0           = now_ms();
            img          = enc_image(w[i].pcm, w[i].frames, w[i].channels, w[i].rate, &o, &len, &g);
            ms           = now_ms() - t0;
            if (!img) {
                fprintf(stderr, "calibrate_v: %s @ %.4f b/s failed\n", k_tracks[i], rate);
                return 1;
            }
            snprintf(arg, sizeof arg, "%.4f", rate);
            emit(k_tracks[i], "abr", arg, &g, img, len, ms);
            free(img);
        }
        return 0;
    }

    fprintf(stderr, "calibrate_v: unknown mode %s\n", argv[1]);
    return 2;
}
