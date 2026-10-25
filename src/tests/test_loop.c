/* test_loop.c -- seamless looping: for every combination of an unaligned loop
 * start and a loop end that may fall inside a slice, three loop passes must
 * equal the linear decode of the same region, sample for sample. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "kac.h"
#include "kac.h"
#include "kac_test_util.h"

static void gen(int16_t *pcm, uint32_t frames, unsigned ch)
{
    uint32_t i;
    unsigned c;
    uint32_t seed = 777u;
    for (i = 0; i < frames; i++) {
        double t = (double)i / 44100.0;
        for (c = 0; c < ch; c++) {
            double v = sin(2.0 * 3.14159265358979 * 330.0 * t) * 18000.0 +
                       sin(2.0 * 3.14159265358979 * 990.0 * t) * 4000.0;
            seed = seed * 1103515245u + 12345u;
            v += (double)((int)((seed >> 16) & 0x3FF) - 512) * 0.5;
            if (v > 32767.0)
                v = 32767.0;
            if (v < -32768.0)
                v = -32768.0;
            pcm[i * ch + c] = (int16_t)v;
        }
    }
}

/* Read and drop n frames; used to reach the loop start without buffering. */
static int discard_frames(kacdec_t *d, uint32_t n)
{
    int16_t scratch[128 * KAC_MAX_CHANNELS];
    while (n > 0) {
        uint32_t take = (n > 128u) ? 128u : n;
        int32_t  got  = kacdec_read(d, scratch, take);
        if (got <= 0)
            return -1;
        n -= (uint32_t)got;
    }
    return 0;
}

static void run_loop_case(const int16_t *pcm, uint32_t frames, unsigned ch, unsigned tier,
                          uint32_t ls, uint32_t le, uint32_t chunk)
{
    enc_opts  o;
    kac_t     g;
    uint8_t  *image;
    size_t    len = 0;
    kacdec_t  d;
    int16_t  *ref, *out;
    uint32_t  seg = le - ls, pass, i;
    int32_t   got;
    int       mism  = 0;
    const int npass = 3;

    enc_opts_default(&o);
    o.tier       = (int)tier;
    o.loop_start = ls;
    o.loop_end   = le;
    o.chunk      = chunk;
    kac_tu_cases++;

    image = enc_image(pcm, frames, ch, 44100, &o, &len, &g);
    if (!image) {
        fprintf(stderr, "FAIL: encode T%u ls=%u le=%u\n", tier, ls, le);
        kac_tu_fail++;
        return;
    }
    ref = (int16_t *)malloc((size_t)frames * ch * sizeof(int16_t));
    out = (int16_t *)malloc((size_t)seg * npass * ch * sizeof(int16_t));
    if (!ref || !out) {
        free(image);
        free(ref);
        free(out);
        return;
    }

    kacdec_init(&d);
    got = kacdec_open(&d, image, len);
    if (got != KAC_OK) {
        fprintf(stderr, "FAIL: open %d\n", got);
        kac_tu_fail++;
        goto done;
    }
    if (!(d.flags & KAC_FLAG_LOOP)) {
        fprintf(stderr, "FAIL: no loop flag\n");
        kac_tu_fail++;
        goto done;
    }
    if (kacdec_read(&d, ref, frames) != (int32_t)frames) {
        fprintf(stderr, "FAIL: linear read\n");
        kac_tu_fail++;
        goto done;
    }

    kacdec_init(&d);
    if (kacdec_open(&d, image, len) != KAC_OK) {
        kac_tu_fail++;
        goto done;
    }
    if (discard_frames(&d, ls) != 0) {
        fprintf(stderr, "FAIL: read to loop start T%u ls=%u\n", tier, ls);
        kac_tu_fail++;
        goto done;
    }
    kacdec_set_loop(&d, 1);
    got = kacdec_read(&d, out, seg * (uint32_t)npass);
    if (got != (int32_t)(seg * (uint32_t)npass)) {
        fprintf(stderr, "FAIL: loop read %d want %u (T%u ls=%u le=%u)\n", got,
                seg * (uint32_t)npass, tier, ls, le);
        kac_tu_fail++;
        goto done;
    }
    for (pass = 0; pass < (uint32_t)npass; pass++)
        for (i = 0; i < seg; i++)
            for (unsigned c = 0; c < ch; c++) {
                int16_t gv = out[(pass * seg + i) * ch + c];
                int16_t ev = ref[(ls + i) * ch + c];
                if (gv != ev && mism < 5)
                    fprintf(stderr, "  pass=%u i=%u c=%u got=%d want=%d\n", pass, i, c, gv, ev);
                if (gv != ev)
                    mism++;
            }
    if (mism) {
        fprintf(stderr, "FAIL: T%u ls=%u le=%u mismatches %d\n", tier, ls, le, mism);
        kac_tu_fail++;
    }
done:
    kacdec_close(&d);
    free(ref);
    free(out);
    free(image);
}

int main(void)
{
    const uint32_t frames = 6000;
    int16_t       *pcm;
    unsigned       tier, ci, li, ki, ci2;
    unsigned       chs[2]    = {1, 2};
    uint32_t       chunks[2] = {7, 512};

    pcm = (int16_t *)malloc((size_t)frames * 2 * sizeof(int16_t));
    if (!pcm)
        return 1;
    gen(pcm, frames, 2);

    printf("== loop boundaries, unaligned start and end ==\n");
    for (ci = 0; ci < 2; ci++)
        for (tier = 1; tier <= 5; tier++) {
            unsigned n = kac_tiers[tier - 1].n;
            uint32_t ls_set[7], len_set[6];
            unsigned nls = 0, nlen = 0;

            ls_set[nls++]   = 0;
            ls_set[nls++]   = 1;
            ls_set[nls++]   = n - 1;
            ls_set[nls++]   = n;
            ls_set[nls++]   = n + 3;
            ls_set[nls++]   = 2 * n + 1;
            ls_set[nls++]   = 10u * n + (n > 4 ? 2u : 1u);
            len_set[nlen++] = 1;
            len_set[nlen++] = 3;
            len_set[nlen++] = n - 1;
            len_set[nlen++] = n;
            len_set[nlen++] = n + 1;
            len_set[nlen++] = 2 * n + 5;

            for (li = 0; li < nls; li++)
                for (ki = 0; ki < nlen; ki++) {
                    uint32_t ls = ls_set[li], le = ls + len_set[ki];
                    if (le > frames)
                        continue;
                    for (ci2 = 0; ci2 < 2u; ci2++)
                        run_loop_case(pcm, frames, chs[ci], tier, ls, le, chunks[ci2]);
                }
        }
    printf("loop combinations %d\n", kac_tu_cases);
    free(pcm);
    return kac_tu_report("test_loop", "cases");
}
