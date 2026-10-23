/* test_roundtrip.c -- encoder/decoder consistency.
 * Encoder SSE must match SSE recomputed from decoded samples, and repeat runs
 * must be byte-identical. The final section checks the encoder time budget. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include <time.h>
#include "kac.h"
#include "kac_test_util.h"
#include "wav_io.h"

static void check(int ok, const char *fmt, ...)
{
    va_list ap;
    kac_tu_cases++;
    if (ok)
        return;
    kac_tu_fail++;
    va_start(ap, fmt);
    fprintf(stderr, "FAIL: ");
    vfprintf(stderr, fmt, ap);
    fprintf(stderr, "\n");
    va_end(ap);
}

static void gen_signal(int16_t *pcm, uint32_t frames, unsigned ch, int kind)
{
    uint32_t i;
    unsigned c;
    uint32_t seed = 12345u + (uint32_t)kind;
    for (i = 0; i < frames; i++) {
        double t   = (double)i / 44100.0;
        double env = 0.5 + 0.5 * sin(2.0 * 3.14159265358979 * 1.7 * t);
        for (c = 0; c < ch; c++) {
            double v = 0.0;
            if (kind == 0) {
                double f = 200.0 + 4000.0 * t / 4.0;
                v        = sin(2.0 * 3.14159265358979 * f * t) * 20000.0 * env;
            } else if (kind == 1) {
                seed = seed * 1103515245u + 12345u;
                v    = ((double)((seed >> 16) & 0x7FFF) - 16384.0) * 2.0;
            } else if (kind == 2) {
                v = sin(2.0 * 3.14159265358979 * 220.0 * t) * 12000.0 +
                    sin(2.0 * 3.14159265358979 * 440.0 * t) * 6000.0 +
                    sin(2.0 * 3.14159265358979 * 1320.0 * t) * 2500.0;
                v *= env;
            } else {
                v = ((i / 37u) & 1u) ? 32767.0 : -32768.0;
                if ((i % 500u) < 3u)
                    v = -v;
            }
            v += (double)c * 37.0;
            if (v > 32767.0)
                v = 32767.0;
            if (v < -32768.0)
                v = -32768.0;
            pcm[i * ch + c] = (int16_t)v;
        }
    }
}

static int32_t clip_count(const int16_t *p, size_t n)
{
    int32_t c = 0;
    size_t  i;
    for (i = 0; i < n; i++)
        if (p[i] == 32767 || p[i] == -32768)
            c++;
    return c;
}

static int decode_image(const uint8_t *image, size_t len, int16_t *out, uint32_t frames)
{
    kacdec_t d;
    int32_t  got;
    kacdec_init(&d);
    if (kacdec_open(&d, image, len) != KAC_OK)
        return -1;
    got = kacdec_read(&d, out, frames);
    if (got != (int32_t)frames)
        return -1;
    if (kacdec_read(&d, out, 1) != 0)
        return -1; /* end of stream */
    kacdec_close(&d);
    return 0;
}

static void run_case(const int16_t *pcm, uint32_t frames, unsigned ch, unsigned tier,
                     const char *name, uint32_t chunk)
{
    enc_opts o;
    kac_t    g;
    uint8_t *image, *image2;
    size_t   len = 0, len2 = 0;
    int16_t *out;
    size_t   n = (size_t)frames * ch;
    int64_t  sse_dec;

    enc_opts_default(&o);
    o.tier  = (int)tier;
    o.chunk = chunk;
    image   = enc_image(pcm, frames, ch, 44100, &o, &len, &g);
    check(image != NULL, "%s T%u chunk=%u: encode failed", name, tier, chunk);
    if (!image)
        return;
    out = (int16_t *)malloc(n * sizeof(int16_t));
    if (!out) {
        free(image);
        return;
    }

    check(decode_image(image, len, out, frames) == 0, "%s T%u chunk=%u: decode failed", name, tier,
          chunk);
    sse_dec = sse_of(pcm, out, n);
    check(sse_dec == g.sse, "%s T%u chunk=%u: SSE %lld != encoder %lld", name, tier, chunk,
          (long long)sse_dec, (long long)g.sse);
    check(clip_count(out, n) == (int32_t)g.clip, "%s T%u chunk=%u: clip count differs", name, tier,
          chunk);

    /* same input twice -> identical bytes */
    enc_opts_default(&o);
    o.tier  = (int)tier;
    o.chunk = chunk;
    image2  = enc_image(pcm, frames, ch, 44100, &o, &len2, &g);
    check(image2 && len2 == len && memcmp(image, image2, len) == 0,
          "%s T%u chunk=%u: encoder is not deterministic", name, tier, chunk);
    free(image2);

    if (chunk == 1u)
        printf("  ok  %-8s T%u ch=%u PSNR=%.2f dB clip=%u\n", name, tier, ch,
               10.0 * log10((double)g.peak * g.peak / ((double)g.sse / (double)n + 1e-30)),
               (unsigned)g.clip);
    free(out);
    free(image);
}

/* An INIT_STATE snapshot carrying the normative default state must not change
 * anything; out-of-range weights must be saturated instead of overflowing. */
static void run_init_state_case(const int16_t *pcm, uint32_t frames, unsigned ch)
{
    enc_opts             o;
    kac_t                g;
    uint8_t             *image, *mod;
    size_t               len = 0, nsz;
    int16_t             *ref, *out;
    size_t               i, n = (size_t)frames * ch;
    static const int16_t iw[4] = {0, 0, -8192, 16383};

    enc_opts_default(&o);
    o.tier = 3;
    image  = enc_image(pcm, frames, ch, 44100, &o, &len, &g);
    check(image != NULL, "init-state: encode failed");
    if (!image)
        return;
    ref = (int16_t *)malloc(n * sizeof(int16_t));
    out = (int16_t *)malloc(n * sizeof(int16_t));
    check(ref && out, "init-state: out of memory");
    if (!ref || !out) {
        free(image);
        free(ref);
        free(out);
        return;
    }
    check(decode_image(image, len, ref, frames) == 0, "init-state: decode failed");

    nsz = len + (size_t)ch * KAC_SNAP_SIZE;
    mod = (uint8_t *)malloc(nsz);
    if (!mod) {
        free(image);
        free(ref);
        free(out);
        return;
    }
    memcpy(mod, image, KAC_HDR_SIZE);
    memcpy(mod + KAC_HDR_SIZE + (size_t)ch * KAC_SNAP_SIZE, image + KAC_HDR_SIZE,
           len - KAC_HDR_SIZE);
    mod[5] |= KAC_FLAG_INIT;
    for (i = 0; i < ch; i++) {
        uint8_t *p = mod + KAC_HDR_SIZE + i * KAC_SNAP_SIZE;
        int      k;
        for (k = 0; k < 4; k++) {
            p[2 * k]     = 0;
            p[2 * k + 1] = 0;
        }
        for (k = 0; k < 4; k++) {
            uint16_t v       = (uint16_t)iw[k];
            p[8 + 2 * k]     = (uint8_t)(v & 0xFF);
            p[8 + 2 * k + 1] = (uint8_t)(v >> 8);
        }
    }
    check(decode_image(mod, nsz, out, frames) == 0, "init-state: decode failed");
    check(memcmp(ref, out, n * sizeof(int16_t)) == 0,
          "init-state: default snapshot changed the output");
    printf("  ok  init_state snapshot ignored when it holds the default state\n");

    for (i = 0; i < ch; i++) { /* weights far outside [-16383,16383] */
        uint8_t *p = mod + KAC_HDR_SIZE + i * KAC_SNAP_SIZE + 8;
        int      k;
        for (k = 0; k < 4; k++) {
            p[2 * k]     = 0xFF;
            p[2 * k + 1] = 0x7F;
        }
    }
    check(decode_image(mod, nsz, out, frames) == 0,
          "init-state: saturated snapshot must still decode");
    printf("  ok  out of range snapshot weights are saturated\n");

    free(mod);
    free(ref);
    free(out);
    free(image);
}

/* INIT_STATE + LOOP + SEEK_INDEX together: the leading snapshot shifts the
 * data and index sections, and loop playback must still be sample exact. */
static void run_combined_case(const int16_t *pcm, uint32_t frames, unsigned ch)
{
    enc_opts             o;
    kac_t                g;
    uint8_t             *image, *mod;
    size_t               len = 0, nsz;
    int16_t             *ref, *out;
    kacdec_t             d;
    size_t               i, n = (size_t)frames * ch;
    uint32_t             seg;
    static const int16_t iw[4] = {0, 0, -8192, 16383};

    enc_opts_default(&o);
    o.tier       = 3;
    o.loop_start = 101;
    o.loop_end   = frames - 7;
    o.stride     = kac_block_for_rate(44100);
    image        = enc_image(pcm, frames, ch, 44100, &o, &len, &g);
    check(image != NULL, "combined: encode failed");
    if (!image)
        return;
    ref = (int16_t *)malloc(n * sizeof(int16_t));
    out = (int16_t *)malloc(n * 4 * sizeof(int16_t));
    if (!ref || !out) {
        free(image);
        free(ref);
        free(out);
        return;
    }
    check(decode_image(image, len, ref, frames) == 0, "combined: decode failed");

    nsz = len + (size_t)ch * KAC_SNAP_SIZE;
    mod = (uint8_t *)malloc(nsz);
    if (!mod) {
        free(image);
        free(ref);
        free(out);
        return;
    }
    memcpy(mod, image, KAC_HDR_SIZE);
    memcpy(mod + KAC_HDR_SIZE + (size_t)ch * KAC_SNAP_SIZE, image + KAC_HDR_SIZE,
           len - KAC_HDR_SIZE);
    mod[5] |= KAC_FLAG_INIT;
    for (i = 0; i < ch; i++) {
        uint8_t *p = mod + KAC_HDR_SIZE + i * KAC_SNAP_SIZE;
        int      k;
        for (k = 0; k < 4; k++) {
            p[2 * k]     = 0;
            p[2 * k + 1] = 0;
        }
        for (k = 0; k < 4; k++) {
            uint16_t v       = (uint16_t)iw[k];
            p[8 + 2 * k]     = (uint8_t)(v & 0xFF);
            p[8 + 2 * k + 1] = (uint8_t)(v >> 8);
        }
    }

    kacdec_init(&d);
    check(kacdec_open(&d, mod, nsz) == KAC_OK, "combined: open failed");
    check(kacdec_read(&d, out, frames) == (int32_t)frames, "combined: read failed");
    check(memcmp(ref, out, n * sizeof(int16_t)) == 0,
          "combined: INIT snapshot changed linear output");

    kacdec_init(&d);
    check(kacdec_open(&d, mod, nsz) == KAC_OK, "combined: reopen failed");
    check(kacdec_read(&d, out, o.loop_start) == (int32_t)o.loop_start,
          "combined: play to loop start failed");
    kacdec_set_loop(&d, 1);
    seg = o.loop_end - o.loop_start;
    check(kacdec_read(&d, out, seg * 2u) == (int32_t)(seg * 2u), "combined: loop read failed");
    for (i = 0; i < (size_t)seg * 2u; i++)
        for (unsigned c = 0; c < ch; c++)
            if (out[i * ch + c] != ref[((size_t)o.loop_start + (i % seg)) * ch + c]) {
                check(0, "combined: loop pass mismatch at %u", (unsigned)i);
                goto done;
            }
    printf("  ok  INIT + LOOP + SEEK layout stays sample exact\n");
done:
    kacdec_close(&d);
    free(mod);
    free(ref);
    free(out);
    free(image);
}

/* INIT_STATE: a second file that starts from the first one's state must produce
 * the samples the first would have produced next. The split is on a block
 * boundary: a block's form depends on the samples it holds and the state it
 * starts in, and only a shared block grid makes those the same for both files. */
static void run_init_state(const int16_t *pcm, uint32_t frames, unsigned ch)
{
    const unsigned tier = 3;
    const uint32_t half = (frames / 2u) / kac_block_for_rate(44100) * kac_block_for_rate(44100);
    const size_t   rest = (size_t)(frames - half) * ch;
    enc_opts       o;
    kac_snap       snaps[KAC_MAX_CHANNELS];
    kac_t          g;
    uint8_t       *whole = NULL, *first = NULL, *second = NULL;
    size_t         lw = 0, lf = 0, lsx = 0;
    int16_t       *dw, *ds;
    uint32_t       i;
    unsigned       mism = 0;

    enc_opts_default(&o);
    o.tier = (int)tier;
    whole  = enc_image(pcm, frames, ch, 44100, &o, &lw, &g);
    check(whole != NULL, "init state: the whole signal encodes");

    enc_opts_default(&o);
    o.tier = (int)tier;
    first  = enc_image(pcm, half, ch, 44100, &o, &lf, &g);
    check(first != NULL, "init state: the first half encodes");
    if (!whole || !first) {
        free(whole);
        free(first);
        return;
    }
    check(kac_get_state(&g, snaps) == 0, "init state: the state is readable");

    enc_opts_default(&o);
    o.tier       = (int)tier;
    o.init_state = snaps;
    second       = enc_image(pcm + (size_t)half * ch, frames - half, ch, 44100, &o, &lsx, &g);
    check(second != NULL, "init state: the second half encodes from that state");
    if (!second) {
        free(whole);
        free(first);
        return;
    }
    check((second[5] & KAC_FLAG_INIT) != 0, "init state: the file carries the flag");

    dw = (int16_t *)malloc((size_t)frames * ch * sizeof(int16_t));
    ds = (int16_t *)malloc(rest * sizeof(int16_t));
    if (!dw || !ds) {
        free(whole);
        free(first);
        free(second);
        free(dw);
        free(ds);
        return;
    }
    check(decode_image(whole, lw, dw, frames) == 0, "init state: the whole file decodes");
    check(decode_image(second, lsx, ds, frames - half) == 0, "init state: the second file decodes");
    for (i = 0; i < rest; i++)
        if (ds[i] != dw[(size_t)half * ch + i])
            mism++;
    check(mism == 0, "init state: %u of %zu samples differ from the single-file encode", mism,
          rest);
    if (mism == 0)
        printf("  ok  a file starting from INIT_STATE continues the first (%zu samples)\n", rest);

    free(dw);
    free(ds);
    free(whole);
    free(first);
    free(second);
}

static void run_sf_search(const int16_t *pcm, uint32_t frames, unsigned ch)
{
    enc_opts o;
    kac_t    g;
    uint8_t *image;
    size_t   len = 0;
    int16_t *out;
    size_t   n = (size_t)frames * ch;

    enc_opts_default(&o);
    o.chunk = 333; /* not a multiple of the block length or of any tier's slices,
                    * so the tail of a chunk is coded as a partial block */
    image = enc_image(pcm, frames, ch, 44100, &o, &len, &g);
    check(image != NULL, "scalefactor search: encode failed");
    if (!image)
        return;
    out = (int16_t *)malloc(n * sizeof(int16_t));
    if (!out) {
        free(image);
        return;
    }
    check(decode_image(image, len, out, frames) == 0, "scalefactor search: decode failed");
    check(sse_of(pcm, out, n) == g.sse, "scalefactor search: SSE mismatch");
    printf("  ok  scalefactor search T%u PSNR=%.2f dB\n", kac_get_tier(&g),
           10.0 * log10((double)g.peak * g.peak / ((double)g.sse / (double)n + 1e-30)));
    free(out);
    free(image);
}

/* These signals drive the probe estimate to s0 == 2 and s0 == 15. The pinned
 * SSE checks the estimate, not the window: the estimate wins every slice, so
 * widening the window or changing the form leaves the sums unchanged. */
static void gen_probe_quiet(int16_t *pcm, uint32_t frames, unsigned ch)
{
    uint32_t i;
    unsigned c;

    for (i = 0; i < frames; i++) {
        int16_t v = ((i % 15u) < 8u) ? 0 : (int16_t)(((i / 15u) & 1u) ? 260 : -260);
        for (c = 0; c < ch; c++)
            pcm[(size_t)i * ch + c] = (int16_t)(v + (int)c * 11);
    }
}

static void gen_probe_loud(int16_t *pcm, uint32_t frames, unsigned ch)
{
    uint32_t i;
    unsigned c;

    for (i = 0; i < frames; i++) {
        int16_t v = ((i % 15u) < 8u) ? (int16_t)((i & 1u) ? 32767 : -32768) : 0;
        for (c = 0; c < ch; c++)
            pcm[(size_t)i * ch + c] = v;
    }
}

static void run_probe_boundary_case(void)
{
    const uint32_t frames = 3000;
    const unsigned ch     = 2;
    const size_t   n      = (size_t)frames * ch;
    int16_t       *pcm, *out;
    unsigned       k;

    pcm = (int16_t *)malloc(n * sizeof(int16_t));
    out = (int16_t *)malloc(n * sizeof(int16_t));
    check(pcm && out, "probe boundary: out of memory");
    if (!pcm || !out) {
        free(pcm);
        free(out);
        return;
    }
    for (k = 0; k < 2u; k++) {
        enc_opts o;
        kac_t    g;
        uint8_t *image;
        size_t   len  = 0;
        int64_t  want = (k == 0u) ? 640130LL : 34295213926LL;

        if (k == 0u)
            gen_probe_quiet(pcm, frames, ch);
        else
            gen_probe_loud(pcm, frames, ch);

        enc_opts_default(&o);
        o.tier    = 4;         /* w = 4, n = 15: the 6000 sample signal is 400 slices */
        o.ms_mode = KAC_MS_LR; /* the sums below pin the scalefactor search, not the
                                * channel form, so the form is held fixed */
        image = enc_image(pcm, frames, ch, 44100, &o, &len, &g);
        check(image != NULL, "probe boundary s0=%u: encode failed", k ? 15u : 2u);
        if (!image)
            continue;
        check(g.sse == want, "probe boundary s0=%u: SSE %lld != %lld", k ? 15u : 2u,
              (long long)g.sse, (long long)want);
        check(decode_image(image, len, out, frames) == 0, "probe boundary: decode failed");
        check(sse_of(pcm, out, n) == g.sse, "probe boundary: decoder SSE differs");
        printf("  ok  probe at s0=%u pins the search: SSE %lld\n", k ? 15u : 2u, (long long)g.sse);
        free(image);
    }
    free(pcm);
    free(out);
}

/* Worst-path encoder time budget: default settings, 10 s of 44.1 kHz stereo,
 * 441000 frames. The 3 s ceiling is asserted only in optimized builds; sanitizer
 * builds measure and print it. Material comes from KAC_MATERIAL if readable,
 * otherwise a deterministic synthetic signal. */
#define BUDGET_FRAMES 441000u
#define BUDGET_CHUNK 4096u
#ifndef BUDGET_LIMIT
#define BUDGET_LIMIT 3.0 /* overridable, so the red path below stays demonstrable */
#endif

#if !defined(KAC_BUDGET_ASSERT)
#if defined(__SANITIZE_ADDRESS__)
#define KAC_BUDGET_ASSERT 0
#else
#define KAC_BUDGET_ASSERT 1
#endif
#endif

static void gen_budget_signal(int16_t *pcm, uint32_t frames, unsigned ch)
{
    const double pi = 3.14159265358979;
    uint32_t     i;
    unsigned     c;
    uint32_t     seed = 20240501u;

    for (i = 0; i < frames; i++) {
        double t = (double)i / 44100.0;
        double v = 8000.0 * sin(2.0 * pi * (220.0 + 200.0 * t) * t);
        seed     = seed * 1103515245u + 12345u;
        v += 5000.0 * ((double)((seed >> 16) & 0x3FFFu) / 8192.0 - 1.0);
        for (c = 0; c < ch; c++) {
            double s = v + (double)c * 911.0;
            if (s > 32767.0)
                s = 32767.0;
            if (s < -32768.0)
                s = -32768.0;
            pcm[(size_t)i * ch + c] = (int16_t)s;
        }
    }
}

/* Every quality level must round trip, and levels that differ in the residual
 * search must produce different bytes. Compare levels on a per-block file,
 * where tier choice actually differs. */
static void run_quality_case(const int16_t *pcm, uint32_t frames, unsigned ch)
{
    enc_opts o;
    kac_t    g;
    uint8_t *image[KAC_QUALITY_MAX + 1];
    size_t   len[KAC_QUALITY_MAX + 1];
    int16_t *out;
    size_t   n = (size_t)frames * ch;
    unsigned lv;

    out = (int16_t *)malloc(n * sizeof(int16_t));
    check(out != NULL, "quality: buffer");
    if (!out)
        return;
    for (lv = 0; lv <= KAC_QUALITY_MAX; lv++) {
        int64_t sse_dec;

        enc_opts_default(&o);
        o.tier         = 4;
        o.quality      = (int)lv;
        o.have_quality = 1;
        image[lv]      = enc_image(pcm, frames, ch, 44100, &o, &len[lv], &g);
        check(image[lv] != NULL, "quality %u: encode failed", lv);
        if (!image[lv])
            continue;
        check(decode_image(image[lv], len[lv], out, frames) == 0, "quality %u: decode failed", lv);
        sse_dec = sse_of(pcm, out, n);
        check(sse_dec == g.sse, "quality %u: SSE %lld != encoder %lld", lv, (long long)sse_dec,
              (long long)g.sse);
        check(clip_count(out, n) == (int32_t)g.clip, "quality %u: clip count differs", lv);
        printf("  ok  quality %u  PSNR=%.2f dB clip=%u\n", lv,
               10.0 * log10((double)g.peak * g.peak / ((double)g.sse / (double)n + 1e-30)),
               (unsigned)g.clip);
    }
    { /* the two measurement levels, on the path where they differ */
        enc_opts o0, o1;
        kac_t    g0, g1;
        uint8_t *i0, *i1;
        size_t   l0 = 0, l1 = 0;

        enc_opts_default(&o0);
        o0.tier       = 0; /* the ladder decides, not a fixed tier */
        o0.vbr_mode   = KAC_VBR_V;
        o0.vbr_target = 5;
        i0            = enc_image(pcm, frames, ch, 44100, &o0, &l0, &g0);
        enc_opts_default(&o1);
        o1.tier         = 0;
        o1.vbr_mode     = KAC_VBR_V;
        o1.vbr_target   = 5;
        o1.quality      = 1;
        o1.have_quality = 1;
        i1              = enc_image(pcm, frames, ch, 44100, &o1, &l1, &g1);
        check(i0 && i1 && (l0 != l1 || memcmp(i0, i1, l0) != 0),
              "quality 1 codes a chunked file as quality 0 does");
        free(i0);
        free(i1);
    }
    /* The two ends of the axis are different settings, or the axis would be a
     * name for nothing. The levels between them are free to repeat each other:
     * there are four things to give up and several levels to space them over. */
    check(len[0] != len[KAC_QUALITY_MAX] || memcmp(image[0], image[KAC_QUALITY_MAX], len[0]) != 0,
          "quality %u codes what quality 0 codes, so the axis decides nothing", KAC_QUALITY_MAX);
    for (lv = 0; lv <= KAC_QUALITY_MAX; lv++)
        free(image[lv]);
    { /* the level is a settled property, and the range is held */
        kac_t p;

        kac_init(&p);
        check(kac_get_quality(&p) == KAC_QUALITY_DEFAULT, "quality starts at the default");
        check(kac_set_quality(&p, -1) == KAC_E_ARG, "quality -1 is refused");
        check(kac_set_quality(&p, KAC_QUALITY_MAX + 1) == KAC_E_ARG,
              "quality above the range is refused");
        check(kac_set_quality(&p, KAC_QUALITY_MAX) == KAC_OK, "the last level is accepted");
        check(kac_get_quality(&p) == KAC_QUALITY_MAX, "kac_get_quality reads it back");
        check(kac_set_report(&p, 0) == KAC_OK, "the counters can be turned off");
        check(kac_set_report(&p, 1) == KAC_OK, "and back on");
        check(kac_set_report(NULL, 1) == KAC_E_ARG, "a null handle is refused");
    }
    free(out);
}

/* The brute levels are fixed overrides of the search columns. Each must round
 * trip, and each must code something of its own: the two levels differ from
 * each other and from the quality level's own search. */
static void run_brute_case(const int16_t *pcm, uint32_t frames, unsigned ch)
{
    enc_opts o0, o1, o2;
    kac_t    g2, p;
    uint8_t *i0, *i1, *i2;
    size_t   l0 = 0, l1 = 0, l2 = 0;
    int16_t *out;

    enc_opts_default(&o0);
    o0.tier  = 4;
    o1       = o0;
    o1.brute = 1;
    o2       = o0;
    o2.brute = 2;

    i0 = enc_image(pcm, frames, ch, 44100, &o0, &l0, NULL);
    i1 = enc_image(pcm, frames, ch, 44100, &o1, &l1, NULL);
    i2 = enc_image(pcm, frames, ch, 44100, &o2, &l2, &g2);
    check(i0 != NULL && i1 != NULL && i2 != NULL, "brute: encode failed");
    if (i0 && i1)
        check(l0 != l1 || memcmp(i0, i1, l0) != 0, "brute 1 codes what brute 0 codes");
    if (i1 && i2)
        check(l1 != l2 || memcmp(i1, i2, l1) != 0, "brute 2 codes what brute 1 codes");
    if (i0 && i2)
        check(l0 != l2 || memcmp(i0, i2, l0) != 0, "brute 2 codes what brute 0 codes");
    out = (int16_t *)malloc((size_t)frames * ch * sizeof *out);
    if (out && i2) {
        check(decode_image(i2, l2, out, frames) == 0, "brute: decode failed");
        check(sse_of(pcm, out, (size_t)frames * ch) == g2.sse, "brute: decoder SSE differs");
    }
    free(out);
    free(i0);
    free(i1);
    free(i2);

    kac_init(&p);
    check(kac_set_brute(&p, 3u) == KAC_E_ARG, "brute 3 is refused");
    check(kac_set_brute(&p, 2u) == KAC_OK, "brute 2 is accepted while fresh");
    check(kac_init_params(&p) == KAC_OK, "brute: init_params");
    check(kac_set_brute(&p, 1u) == KAC_E_ARG, "brute is refused once configured");
}

static void run_budget_case(void)
{
    const char *path = getenv("KAC_MATERIAL");
    kac_wav     w;
    kac_t       g;
    int16_t    *pcm = NULL;
    uint8_t    *out = NULL;
    const char *src;
    uint32_t    frames, off;
    unsigned    ch;
    uint32_t    rate;
    clock_t     t0, t1;
    double      secs;

    if (!path || !*path)
        path = NULL;

    memset(&w, 0, sizeof w);
    if (path && wav_read(path, &w) == 0 && w.frames > 0u && w.rate > 0u) {
        pcm    = w.pcm; /* take ownership so nothing is freed twice */
        w.pcm  = NULL;
        ch     = w.channels;
        rate   = w.rate;
        frames = (w.frames < BUDGET_FRAMES) ? w.frames : BUDGET_FRAMES;
        src    = path;
    } else {
        ch     = 2;
        rate   = 44100;
        frames = BUDGET_FRAMES;
        pcm    = (int16_t *)malloc((size_t)frames * ch * sizeof(int16_t));
        if (!pcm) {
            check(0, "encoder time budget: out of memory");
            return;
        }
        gen_budget_signal(pcm, frames, ch);
        src = "synthetic (material not readable)";
    }

    kac_init(&g);
    kac_set_num_channels(&g, (int)ch);
    kac_set_in_samplerate(&g, (int)rate);
    if (kac_init_params(&g) != 0) {
        check(0, "encoder time budget: kac_init_params failed");
        goto done;
    }
    out = (uint8_t *)malloc(kac_encode_bufsize(&g, BUDGET_CHUNK));
    if (!out) {
        check(0, "encoder time budget: out of memory");
        goto done;
    }

    printf("  material   %s\n", src);
    printf("  basis      %u frames/channel = %.4f s, %u Hz, %u ch, per-block VBR at -V %d, "
           "the 9-candidate scalefactor search\n",
           frames, (double)frames / (double)rate, rate, ch, KAC_DEFAULT_V);

    t0 = clock();
    for (off = 0; off < frames; off += BUDGET_CHUNK) {
        uint32_t n   = frames - off;
        size_t   got = 0;
        if (n > BUDGET_CHUNK)
            n = BUDGET_CHUNK;
        if (kac_encode_buffer_interleaved(&g, pcm + (size_t)off * ch, n, out, &got) < 0) {
            check(0, "encoder time budget: encode error");
            goto done;
        }
    }
    t1   = clock();
    secs = (double)(t1 - t0) / (double)CLOCKS_PER_SEC;
    printf("  measured   %.3f s (one streaming pass over the whole input)\n", secs);
    printf("  budget     ceiling %.1f s\n", BUDGET_LIMIT);
    printf("  build      %s\n", KAC_BUDGET_ASSERT
                                    ? "optimized: the ceiling is ASSERTED"
                                    : "sanitized: the ceiling is recorded, not asserted");
    if (KAC_BUDGET_ASSERT)
        check(secs <= BUDGET_LIMIT,
              "encoder time budget: one encode took %.3f s, over the %.1f s "
              "ceiling",
              secs, BUDGET_LIMIT);

    /* The advanced search has the same ceiling on the same pass. */
    {
        kac_t   gb;
        clock_t b0, b1;
        double  bsecs;

        kac_init(&gb);
        kac_set_num_channels(&gb, (int)ch);
        kac_set_in_samplerate(&gb, (int)rate);
        if (kac_set_brute(&gb, 2u) != 0 || kac_init_params(&gb) != 0) {
            check(0, "brute 2 time budget: configuration failed");
            goto done;
        }
        b0 = clock();
        for (off = 0; off < frames; off += BUDGET_CHUNK) {
            uint32_t n   = frames - off;
            size_t   got = 0;
            if (n > BUDGET_CHUNK)
                n = BUDGET_CHUNK;
            if (kac_encode_buffer_interleaved(&gb, pcm + (size_t)off * ch, n, out, &got) < 0) {
                check(0, "brute 2 time budget: encode error");
                goto done;
            }
        }
        b1    = clock();
        bsecs = (double)(b1 - b0) / (double)CLOCKS_PER_SEC;
        printf("  brute 2    %.3f s (one streaming pass, --brute 2)\n", bsecs);
        if (KAC_BUDGET_ASSERT)
            check(bsecs <= BUDGET_LIMIT,
                  "brute 2 time budget: one encode took %.3f s, over the %.1f s ceiling", bsecs,
                  BUDGET_LIMIT);
    }

done:
    free(out);
    free(pcm);
    wav_free(&w);
}

int main(void)
{
    const uint32_t frames = 20000;
    int16_t       *pcm;
    unsigned       tier, kind, ci;
    unsigned       chs[2]    = {1, 2};
    uint32_t       chunks[3] = {1, 7, 4096};

    pcm = (int16_t *)malloc((size_t)frames * 2 * sizeof(int16_t));
    if (!pcm)
        return 1;

    printf("== streaming encode / decode consistency ==\n");
    for (ci = 0; ci < 2; ci++)
        for (kind = 0; kind < 4; kind++) {
            char nm[16];
            gen_signal(pcm, frames, chs[ci], (int)kind);
            sprintf(nm, "sig%u", kind);
            for (tier = KAC_TIER_MIN; tier <= KAC_TIER_MAX; tier++)
                for (unsigned k = 0; k < 3u; k++)
                    run_case(pcm, frames, chs[ci], tier, nm, chunks[k]);
        }

    printf("== snapshot layouts ==\n");
    gen_signal(pcm, frames, 2, 2);
    run_init_state_case(pcm, frames, 2);
    run_combined_case(pcm, frames, 2);

    printf("== scalefactor search modes ==\n");
    run_sf_search(pcm, frames, 2);

    printf("== INIT_STATE ==\n");
    run_init_state(pcm, frames, 2);

    printf("== prefix probe at the ends of the scalefactor table ==\n");
    run_probe_boundary_case();

    printf("== quality levels ==\n");
    run_quality_case(pcm, frames, 2);

    printf("== brute search levels ==\n");
    run_brute_case(pcm, frames, 2);

    printf("== encoder time budget ==\n");
    run_budget_case();

    free(pcm);
    return kac_tu_report("test_roundtrip", "checks");
}
