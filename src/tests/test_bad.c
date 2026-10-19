/* test_bad.c -- malformed files must be rejected, without undefined behaviour.
 * Every case runs under the sanitizers as well (see tests/run.sh). */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "kac.h"
#include "kac_test_util.h"

static size_t g_open_len = 0; /* lets a case shorten the image */

static void expect_rc(int got, int want, const char *name)
{
    kac_tu_cases++;
    if (got != want) {
        fprintf(stderr, "FAIL: %s -> rc=%d, want %d\n", name, got, want);
        kac_tu_fail++;
    }
}

static void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

int main(void)
{
    const uint32_t frames = 500;
    const unsigned ch     = 2;
    int16_t       *pcm, *out;
    enc_opts       o;
    kac_t          g;
    uint8_t       *base;
    size_t         len = 0;
    kacdec_t       d;
    uint32_t       i, data_bytes;
    int            rc;

    pcm = (int16_t *)malloc((size_t)frames * ch * sizeof(int16_t));
    out = (int16_t *)malloc((size_t)frames * ch * sizeof(int16_t));
    if (!pcm || !out)
        return 1;
    for (i = 0; i < frames * ch; i++)
        pcm[i] = (int16_t)(((i * 7919u) % 60000u) - 30000);

    enc_opts_default(&o);
    o.tier       = 3;
    o.loop_start = 77;
    o.loop_end   = 377;
    o.stride     = kac_block_for_rate(44100); /* the densest index there is */
    base         = enc_image(pcm, frames, ch, 44100, &o, &len, &g);
    if (!base) {
        fprintf(stderr, "base encode failed\n");
        return 1;
    }
    data_bytes = get32(base + 24);
    kacdec_init(&d);
    rc = kacdec_open(&d, base, len);
    if (rc != KAC_OK) {
        fprintf(stderr, "base file rejected rc=%d\n", rc);
        return 1;
    }

    printf("== malformed input (DataBytes=%u, entries=%u) ==\n", data_bytes, get32(base + 32));

#define MUTATE(name, want, ...)                                                                    \
    do {                                                                                           \
        uint8_t *m = (uint8_t *)malloc(len);                                                       \
        memcpy(m, base, len);                                                                      \
        g_open_len = len;                                                                          \
        __VA_ARGS__;                                                                               \
        expect_rc(kacdec_open(&d, m, g_open_len), want, name);                                     \
        free(m);                                                                                   \
    } while (0)

    MUTATE("bad magic", KAC_E_MAGIC, m[0] = 'X');
    MUTATE("magic not NUL padded", KAC_E_MAGIC, m[3] = 1);
    MUTATE("version 2", KAC_E_VERSION, m[4] = 2);
    /* bit3 is the M/S declaration (KAC_SPEC.md Section 3.1); bits 4-7 are reserved. */
    MUTATE("reserved flag bit5", KAC_E_FLAGS, m[5] |= 0x20);
    MUTATE("reserved flag bit7", KAC_E_FLAGS, m[5] |= 0x80);
    MUTATE("channels 3", KAC_E_CHANNEL, m[6] = 3);
    MUTATE("channels 0", KAC_E_CHANNEL, m[6] = 0);
    /* bit3 of the Tier field is the file's channel form, and the form has to be
     * declared in Flags before a file may carry it. */
    /* The base file is stereo and its form is its own, so the contradiction has to
     * be built here rather than assumed: clear the declaration and set the bit. */
    MUTATE("coded form without the Flags bit", KAC_E_FLAGS, m[5] &= (uint8_t)~KAC_FLAG_MS;
           m[7] |= KAC_BLOCK_MS);
    MUTATE("coded form bit4 in the Tier field", KAC_E_TIER, m[7] |= 0x10);
    /* T7 is the widest tier and is admitted, so the out-of-range fixture uses
     * 8; 0 and 255 still cover the other side of the rule. */
    MUTATE("tier 8", KAC_E_TIER, m[7] = KAC_TIER_MAX + 1);
    MUTATE("tier 0", KAC_E_TIER, m[7] = 0);
    MUTATE("tier 255", KAC_E_TIER, m[7] = 255);
    MUTATE("total samples 0", KAC_E_TOTAL, put32(m + 12, 0));
    MUTATE("total samples above the 2^28 bound", KAC_E_TOTAL, put32(m + 12, KAC_MAX_TOTAL + 1u));
    MUTATE("total samples at u32 max", KAC_E_TOTAL, put32(m + 12, 0xffffffffu));
    /* One byte more than the walk would need is also one byte more than the image
     * holds: a file states its own size, so the length rule sees it before the
     * block chain is read at all. */
    MUTATE("data bytes + 1", KAC_E_SIZE, put32(m + 24, data_bytes + 1));
    MUTATE("data bytes 0", KAC_E_DATA, put32(m + 24, 0));
    MUTATE("data bytes one slice short", KAC_E_DATA,
           put32(m + 24, data_bytes - KAC_SLICE_SIZE * KAC_MAX_CHANNELS));
    MUTATE("truncated by 1", KAC_E_SIZE, g_open_len = len - 1);
    MUTATE("header only, 35 bytes", KAC_E_SIZE, g_open_len = KAC_HDR_SIZE - 1u);
    MUTATE("truncated index", KAC_E_SIZE, g_open_len = len - 40);
    MUTATE("truncated loop snapshot", KAC_E_SIZE,
           g_open_len = len - KAC_SNAP_SIZE * KAC_MAX_CHANNELS);
    MUTATE("loop start == loop end", KAC_E_LOOP, put32(m + 16, 100); put32(m + 20, 100));
    MUTATE("loop start > loop end", KAC_E_LOOP, put32(m + 16, 300); put32(m + 20, 100));
    MUTATE("loop start >= total", KAC_E_LOOP, put32(m + 16, 500); put32(m + 20, 501));
    MUTATE("loop end > total", KAC_E_LOOP, put32(m + 20, 501));
    MUTATE("stride not a multiple of the block", KAC_E_INDEX, put32(m + 28, 61));
    MUTATE("stride 0 with SEEK flag", KAC_E_INDEX, put32(m + 28, 0));
    MUTATE("index entries + 1", KAC_E_INDEX, put32(m + 32, get32(m + 32) + 1));
    MUTATE("index entries - 1", KAC_E_INDEX, put32(m + 32, get32(m + 32) - 1));
    MUTATE("stride out of range", KAC_E_INDEX, put32(m + 28, 65535));
    MUTATE("no SEEK flag, stride set", KAC_E_INDEX, m[5] &= (uint8_t)~KAC_FLAG_SEEK);
    MUTATE("no SEEK flag, entries set", KAC_E_INDEX, m[5] &= (uint8_t)~KAC_FLAG_SEEK;
           put32(m + 28, 0));
#undef MUTATE

    { /* One channel has no pair, so the coded form cannot mean anything in a
       * mono file: a tier byte that carries it is refused however that file's
       * Flags read. */
        enc_opts om;
        uint8_t *mono;
        size_t   mlen = 0;

        enc_opts_default(&om);
        om.tier = 3;
        mono    = enc_image(pcm, frames, 1, 44100, &om, &mlen, &g);
        expect_rc(mono != NULL, 1, "mono file encodes");
        if (mono) {
            kacdec_init(&d);
            expect_rc(kacdec_open(&d, mono, mlen), KAC_OK, "mono file opens");
            mono[7] |= KAC_BLOCK_MS;
            kacdec_init(&d);
            expect_rc(kacdec_open(&d, mono, mlen), KAC_E_FLAGS,
                      "mono file carrying the coded form");
            free(mono);
        }
    }

    { /* T7 is the one tier that does not fill the slice -- 4 + 8*7 is 60 --
       * so four bits below the last index are padding. They are not payload,
       * and a byte string that decodes is one byte string, so a non-zero bit
       * there must be rejected rather than ignored. The base file is T3 and
       * fills its slices, so this encodes its own. */
        enc_opts o7;
        uint8_t *t7;
        size_t   l7  = 0;
        uint32_t off = (uint32_t)KAC_HDR_SIZE;

        enc_opts_default(&o7);
        o7.tier = KAC_TIER_MAX;
        t7      = enc_image(pcm, frames, ch, 44100, &o7, &l7, &g);
        expect_rc(t7 != NULL, 1, "T7 file encodes");
        if (t7) {
            if (t7[5] & KAC_FLAG_INIT)
                off += KAC_SNAP_SIZE * ch;
            if (t7[5] & KAC_FLAG_LOOP)
                off += KAC_SNAP_SIZE * ch;
            off += 1u; /* past block 0's tier byte, to its first slice */

            kacdec_init(&d);
            expect_rc(kacdec_open(&d, t7, l7), KAC_OK, "T7 file opens");
            expect_rc(kacdec_read(&d, out, frames), (int)frames, "T7 file decodes");

            t7[off] |= 0x01u; /* the lowest padding bit of slice 0, channel 0 */
            kacdec_init(&d);
            expect_rc(kacdec_open(&d, t7, l7), KAC_OK, "T7 file with padding opens");
            expect_rc(kacdec_read(&d, out, frames), KAC_E_DATA, "non-zero padding is rejected");
            free(t7);
        }
    }

    rc = kacdec_open(&d, base, len);
    expect_rc(rc, KAC_OK, "base file still opens");
    rc = kacdec_seek(&d, frames);
    expect_rc(rc, KAC_E_RANGE, "seek to total");
    kacdec_init(&d);
    kacdec_open(&d, base, len);
    rc = kacdec_read(&d, out, frames);
    expect_rc(rc, (int)frames, "read whole stream");
    rc = kacdec_read(&d, out, 1);
    expect_rc(rc, 0, "read at the end returns 0");
    kacdec_set_loop(&d, 1);
    rc = kacdec_read(&d, out, 1);
    expect_rc(rc, 1, "loop read wraps at the end");
    kacdec_init(&d);
    kacdec_open(&d, base, len);
    rc = kacdec_read(&d, out, frames + 1); /* a fixed block that overruns the end */
    expect_rc(rc, (int)frames, "an overrunning read is clamped to the stream");
    rc = kacdec_read(&d, out, frames + 1);
    expect_rc(rc, 0, "the next read reports the end of stream");

    { /* corrupt offset table */
        uint8_t *m   = (uint8_t *)malloc(len);
        size_t   tab = KAC_HDR_SIZE + (size_t)ch * KAC_SNAP_SIZE + data_bytes;
        memcpy(m, base, len);
        m[tab] = (uint8_t)(m[tab] + 8u);
        rc     = kacdec_open(&d, m, len);
        expect_rc(rc, KAC_OK, "corrupt offset table opens");
        if (rc == KAC_OK)
            expect_rc(kacdec_seek(&d, 0), KAC_E_INDEX, "corrupt offset table seek");
        free(m);
    }

    { /* files without index or loop */
        enc_opts o2;
        uint8_t *img;
        size_t   l2 = 0;

        enc_opts_default(&o2);
        o2.tier = 2;
        img     = enc_image(pcm, frames, ch, 44100, &o2, &l2, &g);
        kacdec_init(&d);
        expect_rc(kacdec_open(&d, img, l2), KAC_OK, "plain file opens");
        kacdec_set_loop(&d, 1);
        expect_rc(kacdec_read(&d, out, 1), KAC_E_NOLOOP, "loop read without LOOP flag");
        expect_rc(kacdec_seek(&d, 10), KAC_E_INDEX, "seek without SEEK flag");
        free(img);

        o2.loop_start = 10;
        o2.loop_end   = 100;
        img           = enc_image(pcm, frames, ch, 44100, &o2, &l2, &g);
        kacdec_init(&d);
        expect_rc(kacdec_open(&d, img, l2), KAC_OK, "loop without index opens");
        expect_rc(kacdec_seek(&d, 10), KAC_E_INDEX, "seek without index");
        free(img);
    }

    { /* Stripped layout: slices with no tier byte. The block walk refuses it
       * because one byte per block cannot land on the closed-form byte count.
       * A legal first byte isolates that failure from the tier range check. */
        enc_opts lr;
        uint8_t *full;
        size_t   flen = 0;

        enc_opts_default(&lr);
        lr.tier    = 3;
        lr.ms_mode = KAC_MS_LR; /* no block declares the coded form: the data bytes
                                 * are then readable as plain slices */
        full = enc_image(pcm, frames, ch, 44100, &lr, &flen, &g);
        expect_rc(full != NULL, 1, "left/right file for the stripped fixture");
        if (full) {
            uint32_t total = get32(full + 12);
            uint32_t db    = get32(full + 24);
            uint32_t cov = 0, tb = 0, ob = 0;
            uint8_t *plain = (uint8_t *)malloc(flen + 1u);
            size_t   plen  = 0;

            expect_rc(plain != NULL, 1, "stripped fixture buffer");
            if (plain) {
                memcpy(plain, full, KAC_HDR_SIZE);
                while (cov < total) {
                    uint32_t blk = total - cov;
                    unsigned nn;
                    uint32_t bytes;
                    if (blk > kac_block_for_rate(44100))
                        blk = kac_block_for_rate(44100);
                    nn    = kac_tiers[(full[KAC_HDR_SIZE + tb] & KAC_TIER_MASK) - 1u].n;
                    bytes = ((blk + nn - 1u) / nn) * KAC_SLICE_SIZE * ch;
                    memcpy(plain + KAC_HDR_SIZE + ob, full + KAC_HDR_SIZE + tb + 1u, bytes);
                    tb += 1u + bytes;
                    ob += bytes;
                    cov += blk;
                }
                expect_rc((tb == db) ? 1 : 0, 1, "the fixture's own walk closes");
                put32(plain + 24, ob); /* the closed form: no tier bytes to count */
                plain[KAC_HDR_SIZE] = (uint8_t)lr.tier;
                plen                = KAC_HDR_SIZE + ob;

                kacdec_init(&d);
                expect_rc(kacdec_open(&d, plain, plen), KAC_E_DATA,
                          "a data area with no tier bytes is refused by the block walk");
                free(plain);
            }
            free(full);
        }
    }

    kacdec_init(&d);
    expect_rc(kacdec_open(&d, base, 0), KAC_E_SIZE, "zero length");
    expect_rc(kacdec_open(&d, NULL, len), KAC_E_ARG, "NULL buffer");
    expect_rc(kacdec_open(NULL, base, len), KAC_E_ARG, "NULL handle");

    free(base);
    free(out);
    free(pcm);
    printf("malformed cases %d\n", kac_tu_cases);
    return kac_tu_report("test_bad", "cases");
}
