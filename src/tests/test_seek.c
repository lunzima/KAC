/* test_seek.c -- decoding after a seek must match the linear decode of the
 * same region; out of range seeks must be refused. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include "kac.h"
#include "kac_test_util.h"

static void expect(int ok, const char *fmt, ...)
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

int main(void)
{
    const uint32_t frames = 5000;
    const unsigned ch     = 2;
    int16_t       *pcm, *ref, *tail;
    enc_opts       o;
    kac_t          g;
    uint8_t       *image = NULL;
    size_t         len   = 0;
    kacdec_t       d;
    uint32_t       i, stride;
    unsigned       tier;
    int            rc;

    pcm  = (int16_t *)malloc((size_t)frames * ch * sizeof(int16_t));
    ref  = (int16_t *)malloc((size_t)frames * ch * sizeof(int16_t));
    tail = (int16_t *)malloc((size_t)frames * ch * sizeof(int16_t));
    if (!pcm || !ref || !tail)
        return 1;
    for (i = 0; i < frames * ch; i++)
        pcm[i] = (int16_t)((((i / 2u) * 6151u) % 65000u) - 32500);

    printf("== seek index ==\n");
    for (tier = 1; tier <= 5; tier++) {
        /* Densest legal index: one point per file block, where the index can
         * exceed the slice data and expose buffer-bound mistakes. */
        unsigned n = kac_tiers[tier - 1].n;
        stride     = kac_block_for_rate(44100);
        enc_opts_default(&o);
        o.tier       = (int)tier;
        o.stride     = stride;
        o.loop_start = 111;
        o.loop_end   = 4000;
        free(image);
        image = enc_image(pcm, frames, ch, 44100, &o, &len, &g);
        expect(image != NULL, "T%u encode", tier);
        if (!image)
            continue;

        kacdec_init(&d);
        rc = kacdec_open(&d, image, len);
        expect(rc == KAC_OK, "T%u open rc=%d", tier, rc);
        expect(kacdec_read(&d, ref, frames) == (int32_t)frames, "T%u full decode", tier);

        {
            uint32_t pts[8];
            unsigned np = 0, k;
            pts[np++]   = 0;
            pts[np++]   = 1;
            pts[np++]   = n;
            pts[np++]   = n + 3;
            pts[np++]   = 2u * stride + 1;
            pts[np++]   = 2u * stride + n - 1;
            pts[np++]   = frames / 2u;
            pts[np++]   = frames - 1;
            for (k = 0; k < np; k++) {
                uint32_t s   = pts[k];
                uint32_t rem = frames - s;
                uint32_t j;
                int      mism = 0;

                kacdec_init(&d);
                expect(kacdec_open(&d, image, len) == KAC_OK, "T%u reopen", tier);
                rc = kacdec_seek(&d, s);
                expect(rc == KAC_OK, "T%u seek(%u) rc=%d", tier, s, rc);
                if (rc != KAC_OK)
                    continue;
                expect(d.pos == s, "T%u seek(%u) left pos=%u", tier, s, d.pos);
                rc = kacdec_read(&d, tail, rem);
                expect(rc == (int)rem, "T%u read after seek(%u) rc=%d", tier, s, rc);
                if (rc != (int)rem)
                    continue;
                for (j = 0; j < rem * ch; j++)
                    if (tail[j] != ref[s * ch + j])
                        mism++;
                expect(mism == 0, "T%u seek(%u): %d/%u samples differ", tier, s, mism, rem * ch);
            }
        }

        kacdec_init(&d);
        kacdec_open(&d, image, len);
        rc = kacdec_seek(&d, frames);
        expect(rc == KAC_E_RANGE, "T%u seek(total) rc=%d", tier, rc);
        printf("  ok  T%u stride=%u\n", tier, stride);
    }

    free(pcm);
    free(ref);
    free(tail);
    free(image);
    printf("seek checks %d\n", kac_tu_cases);
    return kac_tu_report("test_seek", "checks");
}
