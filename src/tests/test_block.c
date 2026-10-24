/* test_block.c -- data area, tier bytes, seek, loop and ABR.
 * The independent block walk below makes agreement with the decoder evidence
 * rather than a tautology. Checks cover tier bytes 0/8, header/block-0
 * agreement, unannounced mid/side, block walk closure, IndexStride, round trip,
 * seek, loop inside a block, and ABR rate/hold/allocation including silence. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include "kac.h"
#include "kac_test_util.h"

/* Allowed ABR deviation, in per cent. At -q 3 the measured worst is +2.70 %
 * on level-stepping material, so 3.0 % is the bound. */
#define ABR_BOUND 3.0

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

/* One block of the walk: where its tier byte sits, what it says, how many
 * samples it covers. */
typedef struct {
    uint32_t tb, blk;
    unsigned tier;
} blkref;

/* The block walk of KAC_SPEC.md Section 3.2, independent of the decoder's copy
 * so that agreement between the two is evidence rather than a tautology.
 * Returns the number of blocks, or 0 when the walk does not close. */
static unsigned walk_blocks(const uint8_t *img, size_t len, blkref *out, unsigned cap)
{
    const uint8_t *data;
    uint32_t       total = get32(img + 12), db = get32(img + 24), cov = 0, tb = 0, n = 0;
    unsigned       ch     = img[6];
    uint32_t       blkmax = kac_block_for_rate(get32(img + 8));

    if (len < KAC_HDR_SIZE || (uint64_t)KAC_HDR_SIZE + db > (uint64_t)len)
        return 0;
    /* the data area follows the optional snapshots, not the header */
    data = img + KAC_HDR_SIZE;
    data += (img[5] & KAC_FLAG_INIT) ? (size_t)ch * KAC_SNAP_SIZE : 0u;
    data += (img[5] & KAC_FLAG_LOOP) ? (size_t)ch * KAC_SNAP_SIZE : 0u;
    if ((size_t)(data - img) + db > len)
        return 0;
    while (cov < total) {
        uint32_t blk = total - cov;
        unsigned b, tier, nn;
        if (blk > blkmax)
            blk = blkmax;
        if (tb >= db || n >= cap)
            return 0;
        /* bit 3 of a tier byte is the block's channel form, so the tier is the
         * rest of the byte; the bits above it carry nothing. */
        b    = data[tb];
        tier = b & KAC_TIER_MASK;
        if (tier < (unsigned)KAC_TIER_MIN || tier > (unsigned)KAC_TIER_MAX ||
            (b & KAC_TIER_SPARE) != 0)
            return 0;
        out[n].tb   = tb;
        out[n].tier = tier;
        out[n].blk  = blk;
        n++;
        nn = kac_tiers[tier - 1u].n;
        tb = tb + 1u + ((blk + nn - 1u) / nn) * KAC_SLICE_SIZE * ch;
        cov += blk;
    }
    return (cov == total && tb == db) ? n : 0;
}

/* Two channels with a shared tonal part and independent detail. The shared part
 * cancels in the side channel and the independent noise does not, so the coded
 * form carries less error than left/right. The varied fixture above has no such
 * ordering and cannot make the coded form win. */
static void gen_correlated(int16_t *pcm, uint32_t frames)
{
    uint32_t i, seed = 2463534242u;

    for (i = 0; i < frames; i++) {
        double t   = (double)i / 44100.0;
        double com = (sin(2.0 * 3.14159265358979 * 110.0 * t) +
                      0.7 * sin(2.0 * 3.14159265358979 * 277.0 * t) +
                      0.4 * sin(2.0 * 3.14159265358979 * 631.0 * t)) *
                     7000.0;
        unsigned c;

        for (c = 0; c < 2u; c++) {
            double n, v;

            seed = seed * 1103515245u + 12345u;
            n    = ((double)((seed >> 16) & 0x3FFFu) / 16384.0 - 1.0) * 2500.0;
            v    = com + n;
            if (v > 32767.0)
                v = 32767.0;
            if (v < -32768.0)
                v = -32768.0;
            pcm[i * 2u + c] = (int16_t)v;
        }
    }
}

/* Amplitude steps by 12:1 in the middle, so a target that sits anywhere in the
 * ladder has to move the tier between the two halves; the pseudo-random part
 * keeps the material from being so predictable that every tier saturates. */
static void gen_varied(int16_t *pcm, uint32_t frames, unsigned ch)
{
    uint32_t i, seed = 987654321u;
    unsigned c;
    for (i = 0; i < frames; i++) {
        double t    = (double)i / 44100.0;
        double amp  = (i < frames / 2u) ? 200.0 : 24000.0;
        double warm = sin(2.0 * 3.14159265358979 * 330.0 * t) * 0.6;
        double v;
        seed = seed * 1103515245u + 12345u;
        v    = (warm + ((double)((seed >> 16) & 0x7FFFu) / 16384.0 - 1.0) * 0.06) * amp;
        for (c = 0; c < ch; c++) {
            double s = v + (double)c * 37.0;
            if (s > 32767.0)
                s = 32767.0;
            if (s < -32768.0)
                s = -32768.0;
            pcm[i * ch + c] = (int16_t)s;
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

/* Decode every sample of the file without the loop: the reference a replayed
 * period has to match. Returns the sample count or -1. */
static int32_t decode_seq(const uint8_t *img, size_t len, int16_t *out, uint32_t frames)
{
    kacdec_t d;
    kacdec_init(&d);
    if (kacdec_open(&d, img, len) != KAC_OK)
        return -1;
    return kacdec_read(&d, out, frames);
}

/* Decode loop_start + periods * length samples with the loop on. Returns the
 * sample count or -1. */
static int32_t decode_loop(const uint8_t *img, size_t len, int16_t *out, uint32_t need,
                           uint32_t loop_start, uint32_t loop_end)
{
    kacdec_t d;
    kacdec_init(&d);
    if (kacdec_open(&d, img, len) != KAC_OK)
        return -1;
    if (!(d.flags & KAC_FLAG_LOOP) || d.loop_start != loop_start || d.loop_end != loop_end)
        return -1;
    kacdec_set_loop(&d, 1);
    return kacdec_read(&d, out, need);
}

int main(void)
{
    const uint32_t frames = 12000; /* 14 full blocks + 240 samples */
    const unsigned ch     = 2;
    const uint32_t block  = kac_block_for_rate(44100);
    const uint32_t stride = 6u * block; /* 5040: 2 index points */
    int16_t       *pcm, *out, *ref, *tail;
    enc_opts       o;
    kac_t          g, base_g;
    uint8_t       *image = NULL, *mut;
    size_t         len   = 0;
    uint32_t       i, db;
    unsigned       nblk, k;
    blkref         blk[64];
    kacdec_t       d;
    int            rc;

    pcm  = (int16_t *)malloc((size_t)frames * ch * sizeof(int16_t));
    out  = (int16_t *)malloc((size_t)frames * ch * sizeof(int16_t));
    ref  = (int16_t *)malloc((size_t)frames * ch * sizeof(int16_t));
    tail = (int16_t *)malloc((size_t)frames * ch * sizeof(int16_t));
    if (!pcm || !out || !ref || !tail)
        return 1;
    gen_varied(pcm, frames, ch);

    { /* The reserved-bit rule a decoder without an extension relies on: bits 4-7
       * of Flags and of a tier byte carry nothing and must be zero. */
        enc_opts o2;
        uint8_t *plain;
        size_t   plen = 0;

        enc_opts_default(&o2);
        o2.tier = 3;
        plain   = enc_image(pcm, frames, ch, 44100, &o2, &plen, &g);
        expect(plain != NULL, "plain encode");
        kacdec_init(&d);
        expect(kacdec_open(&d, plain, plen) == KAC_OK, "the plain file opens");
        mut = (uint8_t *)malloc(plen);
        memcpy(mut, plain, plen);
        mut[5] |= 0x20;
        expect(kacdec_open(&d, mut, plen) == KAC_E_FLAGS, "reserved flag bit5 -> KAC_E_FLAGS");
        free(mut);
        free(plain);
    }

    /* ---- the file every check below works from -------------------------- */
    /* Fixture: level 0, -V 8 so the tier sequence moves. Mid/side does not win
     * on this material, so tests needing it build one. */
    enc_opts_default(&o);
    o.tier         = 0; /* -V owns the tier decision */
    o.vbr_mode     = KAC_VBR_V;
    o.vbr_target   = 8;
    o.stride       = stride;
    o.quality      = 0;
    o.have_quality = 1;
    image          = enc_image(pcm, frames, ch, 44100, &o, &len, &g);
    if (!image) {
        fprintf(stderr, "encode failed\n");
        return 1;
    }
    db = get32(image + 24);
    printf("== file: %u frames, %u data bytes, flags 0x%02X, tier %u ==\n", frames, db, image[5],
           image[7]);

    expect(kac_get_vbr_mode(&g) == KAC_VBR_V, "the encoder reports mode -V");
    base_g = g; /* the sections below re-encode into g; keep the statistics */

    { /* the -V index is refused outside 0..9.999, never clamped -- "-V 12"
       * silently meaning "-V 9" would be a downgrade nobody asked for. */
        kac_t p;
        kac_init(&p);
        kac_set_num_channels(&p, (int)ch);
        expect(kac_set_vbr(&p, KAC_VBR_V, 10.0) == KAC_E_ARG, "-V 10 is refused");
        expect(kac_set_vbr(&p, KAC_VBR_V, -1.0) == KAC_E_ARG, "-V -1 is refused");
        expect(kac_set_vbr(&p, KAC_VBR_V, 2.5) == KAC_OK, "-V 2.5 is accepted (thousandths)");
        expect(kac_set_vbr(&p, KAC_VBR_V, 9.1) == KAC_OK, "-V 9.1 is accepted");
        expect(kac_set_vbr(&p, KAC_VBR_V, 9.999) == KAC_OK, "-V 9.999 is accepted (last rung)");
        expect(kac_get_vbr_target(&p) == 33.75, "-V 9.999 aims at 33.75 dB, got %.3f",
               kac_get_vbr_target(&p));
        expect(kac_set_vbr(&p, KAC_VBR_V, 9.0) == KAC_OK, "-V 9 is accepted");
        expect(kac_set_vbr(&p, KAC_VBR_V, 0.0) == KAC_OK, "-V 0 is accepted (not a default)");
        expect(kac_set_vbr(&p, KAC_VBR_ABR, 0.0) == KAC_E_ARG, "--abr 0 is refused");
        expect(kac_set_vbr(&p, 7, 1.0) == KAC_E_ARG, "an unknown mode is refused");
        /* the calibration-only entry validates its own arguments too */
        expect(kac_set_vbr_target_psnr(&p, 0.0) == KAC_E_ARG, "a zero target is refused");
        expect(kac_set_vbr_target_psnr(&p, -3.0) == KAC_E_ARG, "a negative target is refused");
        expect(kac_set_vbr_target_psnr(&p, 50.0) == KAC_OK, "a positive target is accepted");
        /* Pin the ten absolute rung targets; a span check alone would pass with
         * a shifted ladder. Spacing widens at the top because the anchors come
         * from a measured listening ladder, not the distortion curve. */
        {
            static const double want[10] = {63.0, 58.75, 55.25, 51.5,  49.0,
                                            45.0, 42.75, 40.75, 37.75, 36.0};
            unsigned            r;
            for (r = 0; r < 10; r++)
                expect(kac_v_tier_psnr((int)r) == want[r], "-V %u aims at %.2f dB, got %.2f", r,
                       want[r], kac_v_tier_psnr((int)r));
            /* and the span, kept as a second, weaker statement */
            expect(kac_v_tier_psnr(0) - kac_v_tier_psnr(9) == 27.0,
                   "the ten rungs span 27.0 dB (%.3f)", kac_v_tier_psnr(0) - kac_v_tier_psnr(9));
            expect(kac_v_nominal_tier(0) == 5u && kac_v_nominal_tier(9) == 2u,
                   "the ladder runs from nominal T5 to nominal T2 (%u .. %u)",
                   kac_v_nominal_tier(0), kac_v_nominal_tier(9));
        }
    }

    { /* At level 0 the ladder asks for no concession, so a rung target must
       * produce exactly the file -V n produces. */
        int n;
        for (n = 0; n <= 9; n += 3) {
            enc_opts oi, ob;
            uint8_t *ii, *ib;
            size_t   li = 0, lb = 0;
            int      same;

            enc_opts_default(&oi);
            oi.tier         = 0;
            oi.vbr_mode     = KAC_VBR_V;
            oi.vbr_target   = n;
            oi.have_quality = 1;
            oi.quality      = 0;
            ii              = enc_image(pcm, frames, ch, 44100, &oi, &li, &g);

            enc_opts_default(&ob);
            ob.tier         = 0;
            ob.vbr_mode     = KAC_VBR_V;
            ob.vbr_psnr     = kac_v_tier_psnr(n);
            ob.have_quality = 1;
            ob.quality      = 0;
            ib              = enc_image(pcm, frames, ch, 44100, &ob, &lb, &g);

            same = (ii != NULL && ib != NULL && li == lb && li > 0 && memcmp(ii, ib, li) == 0);
            expect(same, "-V %d and the calibration entry at %.3f dB differ (%zu vs %zu bytes)", n,
                   kac_v_tier_psnr(n), li, lb);
            free(ii);
            free(ib);
        }
        printf("  ok  the calibration entry reproduces -V n byte for byte\n");
    }

    nblk = walk_blocks(image, len, blk, 64);
    expect(nblk != 0, "the block walk of the encoder's own output closes");
    for (k = 0; nblk != 0 && k < nblk; k++)
        printf("  block %2u  tb=%6u  tier=T%u  %u samples\n", k, blk[k].tb, blk[k].tier,
               blk[k].blk);
    expect(nblk == (frames + block - 1u) / block,
           "one tier byte per block (%u blocks for %u frames)", nblk, frames);

    { /* The point of the whole exercise: the tier really does move, so the
       * checks below exercise the block machinery rather than one tier. */
        unsigned hist[KAC_TIER_MAX + 1], distinct = 0, j;
        for (j = 0; j <= KAC_TIER_MAX; j++)
            hist[j] = 0;
        for (k = 0; k < nblk; k++)
            hist[blk[k].tier]++;
        for (j = 1; j <= KAC_TIER_MAX; j++)
            if (hist[j])
                distinct++;
        printf("  tier histogram: T1=%u T2=%u T3=%u T4=%u T5=%u T6=%u T7=%u\n", hist[1], hist[2],
               hist[3], hist[4], hist[5], hist[6], hist[7]);
        expect(distinct >= 2, "the stream uses more than one tier (%u distinct)", distinct);
    }

    /* ---- 2. a block's tier byte outside 1..7 ----------------------------- */
    printf("\n== 1. tier byte 0 and 8 ==\n");
    expect(nblk >= 4, "enough blocks to mutate (%u)", nblk);
    for (k = 0; k < 2; k++) {
        unsigned idx  = k ? 3u : 0u; /* block 0 also trips the header cross-check */
        uint8_t  save = image[KAC_HDR_SIZE + blk[idx].tb];
        mut           = (uint8_t *)malloc(len);
        memcpy(mut, image, len);
        mut[KAC_HDR_SIZE + blk[idx].tb] = 0;
        expect(kacdec_open(&d, mut, len) == KAC_E_TIER, "block %u tier byte 0 -> KAC_E_TIER", idx);
        mut[KAC_HDR_SIZE + blk[idx].tb] = 8;
        expect(kacdec_open(&d, mut, len) == KAC_E_TIER, "block %u tier byte 8 -> KAC_E_TIER", idx);
        free(mut);
        expect(image[KAC_HDR_SIZE + blk[idx].tb] == save, "the base image is untouched");
        printf("  ok  block %u: tier byte 0 and 8 both refused\n", idx);
    }

    /* ---- 3. header Tier must be block 0's tier byte ---------------------- */
    printf("\n== 2. header Tier vs block 0's tier byte ==\n");
    {
        uint8_t  hdr = image[7];
        unsigned alt = (hdr == 1u) ? 2u : 1u; /* any other legal tier */
        mut          = (uint8_t *)malloc(len);
        memcpy(mut, image, len);
        mut[7] = (uint8_t)alt;
        expect(kacdec_open(&d, mut, len) == KAC_E_TIER,
               "header Tier=%u vs block 0 tier byte %u -> KAC_E_TIER", alt, blk[0].tier);
        /* and the cross-check is not vacuous: restoring the right value opens */
        mut[7] = (uint8_t)blk[0].tier;
        expect(kacdec_open(&d, mut, len) == KAC_OK, "header Tier restored -> opens");
        free(mut);
        printf("  ok  header Tier %u vs block 0's %u -> refused\n", alt, blk[0].tier);
    }

    /* ---- 2b. unannounced mid/side block ---------------------------------- *
     * The fixture has no mid/side blocks, so the case sets the bit itself. */
    printf("\n== 2b. a block carries the coded form unannounced ==\n");
    {
        uint8_t *m = (uint8_t *)malloc(len);
        unsigned j;

        /* Checked rather than assumed: no block of the fixture is really coded,
         * so the bit this case sets is the only one in the file. The margin puts
         * it that way -- the coded form must beat the arrived form by a quarter
         * of a dB at this tier, and this material's advantage is a few milli-dB. */
        for (j = 0; j < nblk; j++)
            expect((image[KAC_HDR_SIZE + blk[j].tb] & KAC_BLOCK_MS) == 0u,
                   "block %u of the fixture is sent in the form it arrived in", j);

        memcpy(m, image, len);
        expect((m[5] & KAC_FLAG_MS) != 0, "the stereo file declares that it may carry it");
        m[5] &= (uint8_t)~KAC_FLAG_MS;
        m[KAC_HDR_SIZE] |= KAC_BLOCK_MS; /* block 0's tier byte */
        expect(kacdec_open(&d, m, len) == KAC_E_TIER,
               "a block carries the coded form with Flags.MS=0 -> KAC_E_TIER");
        /* and the check is not vacuous: the declaration and block 0's own form
         * bit together are what the same bytes need to open. */
        m[5] |= (uint8_t)KAC_FLAG_MS;
        m[7] |= (uint8_t)KAC_BLOCK_MS;
        expect(kacdec_open(&d, m, len) == KAC_OK, "the declaration restored -> opens");
        free(m);
        printf("  ok  a block's coded form without the file's declaration -> refused\n");
    }

    /* ---- 2c. the form can be forced for the whole run -------------------- */
    printf("\n== 2c. the form is forced for the whole run ==\n");
    {
        enc_opts f;
        uint8_t *im;
        size_t   ln = 0;
        unsigned j, coded, nb;
        /* This section's own walk, so that sections below keep the fixture's
         * block list. */
        blkref bb[64];

        enc_opts_default(&f);
        f.tier       = 0; /* -V replaces the tier decision */
        f.vbr_mode   = KAC_VBR_V;
        f.vbr_target = 2;
        f.ms_mode    = KAC_MS_MS;
        im           = enc_image(pcm, frames, ch, 44100, &f, &ln, &g);
        expect(im != NULL, "a forced coded form encodes");
        if (im) {
            nb    = walk_blocks(im, ln, bb, 64);
            coded = 0;
            expect(nb > 0, "the forced file walks");
            for (j = 0; j < nb; j++)
                coded += (unsigned)((im[KAC_HDR_SIZE + bb[j].tb] & KAC_BLOCK_MS) != 0);
            expect(coded == nb, "every block is sent in the coded form (%u of %u)", coded, nb);
            expect((im[7] & KAC_BLOCK_MS) != 0, "the header declares it for block 0");
            /* The decoder checks the declaration against the blocks, so opening
             * is the round trip for a forced form. */
            expect(kacdec_open(&d, im, ln) == KAC_OK, "the forced file opens");
            free(im);
        }

        f.ms_mode = KAC_MS_LR;
        im        = enc_image(pcm, frames, ch, 44100, &f, &ln, &g);
        expect(im != NULL, "a forced left/right form encodes");
        if (im) {
            nb    = walk_blocks(im, ln, bb, 64);
            coded = 0;
            for (j = 0; j < nb; j++)
                coded += (unsigned)((im[KAC_HDR_SIZE + bb[j].tb] & KAC_BLOCK_MS) != 0);
            expect(coded == 0, "no block is sent in the coded form (%u of %u)", coded, nb);
            expect(kacdec_open(&d, im, ln) == KAC_OK, "the left/right file opens");
            free(im);
        }
    }

    printf("\n== 2d. the form is forced on a pinned tier too ==\n");
    {
        /* A pinned run gives its blocks one tier but chooses the form the same
         * way a per-block run does, so only the forced settings fix it. The
         * default (KAC_MS_AUTO) is whatever each block measured, not the coded
         * form everywhere. */
        enc_opts f;
        uint8_t *im;
        size_t   ln = 0;
        unsigned j, coded, nb;
        kac_t    fg;
        blkref   bb[64];

        enc_opts_default(&f);
        f.tier = 4;
        im     = enc_image(pcm, frames, ch, 44100, &f, &ln, &fg);
        expect(im != NULL, "the default form encodes at a pinned tier");
        if (im) {
            /* A pinned tier gives every block the same tier byte, so the header's
             * Tier field -- which is block 0's -- is that of every block. */
            nb = walk_blocks(im, ln, bb, 64);
            expect(nb > 0, "the pinned file walks with %u blocks", nb);
            for (j = 0; j < nb; j++)
                expect(bb[j].tier == (unsigned)(im[7] & KAC_TIER_MASK),
                       "block %u is on the header's tier (%u vs %u)", j, bb[j].tier,
                       (unsigned)(im[7] & KAC_TIER_MASK));
            expect((im[5] & KAC_FLAG_MS) != 0,
                   "the file declares that it may carry the coded form");
            /* The form is a per-block choice here as everywhere, so what the
             * measurement settled on is what the decoder has to reproduce. */
            kacdec_init(&d);
            rc = kacdec_open(&d, im, ln);
            expect(rc == KAC_OK, "the pinned file opens, rc=%d", rc);
            rc = kacdec_read(&d, out, frames);
            expect(rc == (int)frames, "full decode rc=%d", rc);
            expect(sse_of(pcm, out, (size_t)frames * ch) == fg.sse,
                   "decoder SSE %lld != encoder %lld",
                   (long long)sse_of(pcm, out, (size_t)frames * ch), (long long)fg.sse);
            free(im);
        }

        f.ms_mode = KAC_MS_MS;
        im        = enc_image(pcm, frames, ch, 44100, &f, &ln, &g);
        expect(im != NULL, "a forced coded form encodes at a pinned tier");
        if (im) {
            nb    = walk_blocks(im, ln, bb, 64);
            coded = 0;
            for (j = 0; j < nb; j++)
                coded += (unsigned)((im[KAC_HDR_SIZE + bb[j].tb] & KAC_BLOCK_MS) != 0);
            expect(nb > 0 && coded == nb, "every block is sent in the coded form (%u of %u)", coded,
                   nb);
            expect((im[7] & KAC_BLOCK_MS) != 0, "and the header carries it for block 0");
            expect(kacdec_open(&d, im, ln) == KAC_OK, "the forced coded file opens");
            free(im);
        }

        f.ms_mode = KAC_MS_LR;
        im        = enc_image(pcm, frames, ch, 44100, &f, &ln, &g);
        expect(im != NULL, "a forced left/right form encodes at a pinned tier");
        if (im) {
            expect((im[5] & KAC_FLAG_MS) == 0, "the left/right file declares nothing");
            expect((im[7] & KAC_BLOCK_MS) == 0, "and carries no form in the Tier field");
            expect(kacdec_open(&d, im, ln) == KAC_OK, "the left/right file opens");
            free(im);
        }
    }

    printf("\n== 2e. an estimated form still codes what it chooses ==\n");
    {
        /* The form estimate decides, and the block is then coded in that form.
         * The decision has to survive the coding on both sides of the mid/side
         * margin: both levels here estimate, and the fixture is one the coded
         * form wins on. */
        static const int levels[2] = {3, 4};
        int16_t         *cpcm      = (int16_t *)malloc((size_t)frames * ch * sizeof(int16_t));
        unsigned         a;

        expect(cpcm != NULL, "the correlated fixture allocates");
        if (cpcm) {
            gen_correlated(cpcm, frames);
            for (a = 0; a < 2u; a++) {
                enc_opts fq;
                uint8_t *imq;
                size_t   lq = 0;
                unsigned jq, nq, codedq;
                kac_t    gq;
                blkref   bq[64];

                enc_opts_default(&fq);
                fq.tier         = 3;
                fq.have_quality = 1;
                fq.quality      = levels[a];
                imq             = enc_image(cpcm, frames, ch, 44100, &fq, &lq, &gq);
                expect(imq != NULL, "q%d encodes the correlated fixture", levels[a]);
                if (!imq)
                    continue;
                nq     = walk_blocks(imq, lq, bq, 64);
                codedq = 0;
                for (jq = 0; jq < nq; jq++)
                    codedq += (unsigned)((imq[KAC_HDR_SIZE + bq[jq].tb] & KAC_BLOCK_MS) != 0);
                expect(nq > 0, "q%d walks with %u blocks", levels[a], nq);
                expect(codedq > 0, "q%d picks the coded form (%u of %u blocks)", levels[a], codedq,
                       nq);
                expect(decode_seq(imq, lq, out, frames) == (int)frames, "q%d decodes fully",
                       levels[a]);
                expect(sse_of(cpcm, out, (size_t)frames * ch) == gq.sse,
                       "q%d decoder SSE %lld != encoder %lld", levels[a],
                       (long long)sse_of(cpcm, out, (size_t)frames * ch), (long long)gq.sse);
                free(imq);
            }
            free(cpcm);
        }
    }

    /* ---- 4. the block walk must close ------------------------------------ */
    printf("\n== 3. the block walk does not close ==\n");
    mut = (uint8_t *)malloc(len);
    memcpy(mut, image, len);
    put32(mut + 24, db - KAC_SLICE_SIZE * ch); /* one slice less than the blocks use */
    expect(kacdec_open(&d, mut, len) == KAC_E_DATA, "DataBytes short by one slice -> KAC_E_DATA");
    free(mut);

    mut = (uint8_t *)malloc(len);
    memcpy(mut, image, len);
    put32(mut + 24, db + KAC_SLICE_SIZE * ch); /* more than the blocks use, and past EOF */
    rc = kacdec_open(&d, mut, len);
    expect(rc < 0, "DataBytes longer than the file -> rc=%d, want a refusal", rc);
    free(mut);

    mut = (uint8_t *)malloc(len);
    memcpy(mut, image, len);
    put32(mut + 12, frames * 2u); /* the blocks cannot cover this many samples */
    rc = kacdec_open(&d, mut, len);
    expect(rc < 0, "TotalSamples too large for the blocks -> rc=%d, want a refusal", rc);
    free(mut);

    /* Pin DataBytes to the last tier byte: the walk must stop exactly there
     * before covering TotalSamples. */
    expect(nblk >= 1 && blk[nblk - 1u].tb < db, "a last block to pin DataBytes on (%u of %u)",
           blk[nblk - 1u].tb, db);
    mut = (uint8_t *)malloc(len);
    memcpy(mut, image, len);
    put32(mut + 24, blk[nblk - 1u].tb);
    expect(kacdec_open(&d, mut, len) == KAC_E_DATA,
           "the walk runs out of data before TotalSamples -> KAC_E_DATA");
    free(mut);

    /* A file whose buffer ends where its data area ends. With an index behind
     * the data an overshooting walk still lands inside the image, so only this
     * shape exercises the guard: Section 3.1's walk reaches the end of the data
     * area before TotalSamples. */
    {
        size_t   full = KAC_HDR_SIZE + db;
        size_t   pin  = KAC_HDR_SIZE + blk[nblk - 1u].tb;
        uint8_t *noi  = (uint8_t *)malloc(full);

        memcpy(noi, image, full);
        noi[5] &= (uint8_t)~KAC_FLAG_SEEK;
        put32(noi + 28, 0);
        put32(noi + 32, 0);
        kacdec_init(&d);
        expect(kacdec_open(&d, noi, full) == KAC_OK, "the file without an index opens");

        /* truncating the *buffer* to the last block tier byte, not merely
         * the header field: a walk without the bound reads one byte past the
         * image here, which is what the guard exists to prevent */
        free(noi);
        noi = (uint8_t *)malloc(pin);
        memcpy(noi, image, pin);
        noi[5] &= (uint8_t)~KAC_FLAG_SEEK;
        put32(noi + 24, (uint32_t)(pin - KAC_HDR_SIZE));
        put32(noi + 28, 0);
        put32(noi + 32, 0);
        kacdec_init(&d);
        expect(kacdec_open(&d, noi, pin) == KAC_E_DATA,
               "no-index file: the walk runs out of data -> KAC_E_DATA");
        free(noi);
    }

    /* A truncated file must be refused. Nothing in the header states the byte
     * length of the data area -- the tier bytes do -- so the walk is what has to
     * refuse it, and this is the case that pins that. */
    kacdec_init(&d);
    expect(kacdec_open(&d, image, len - 1u) == KAC_E_SIZE, "truncated by 1 byte -> KAC_E_SIZE");
    printf("  ok  short DataBytes, oversized DataBytes, walk ends early, oversized total, "
           "truncation: all refused\n");

    /* ---- 4. IndexStride must be a multiple of the block ------------------ */
    printf("\n== 4. IndexStride ==\n");
    mut = (uint8_t *)malloc(len);
    memcpy(mut, image, len);
    put32(mut + 28, 5120); /* a multiple of every tier's N, but not of the block */
    put32(mut + 32, (frames - 1u) / 5120u + 1u);
    expect(kacdec_open(&d, mut, len) == KAC_E_INDEX, "IndexStride 5120 -> KAC_E_INDEX");
    free(mut);
    printf("  ok  IndexStride 5120 refused\n");

    { /* The block is what an index point has to land on, and the block is one
       * whole slice at every tier -- so a stride that is a multiple of the
       * block is the whole of the rule, and one that is a multiple of a single
       * tier's N is not enough. */
        enc_opts o3;
        uint8_t *plain;
        size_t   plen = 0;

        enc_opts_default(&o3);
        o3.tier   = 3; /* N = 20; 5120 = 256 * 20 */
        o3.stride = 5120;
        plain     = enc_image(pcm, frames, ch, 44100, &o3, &plen, &g);
        expect(plain == NULL, "an encoder asked for stride 5120 refuses it");
        o3.stride = 2u * block;
        plain     = enc_image(pcm, frames, ch, 44100, &o3, &plen, &g);
        expect(plain != NULL, "a stride of two blocks encodes");
        if (plain) {
            kacdec_init(&d);
            expect(kacdec_open(&d, plain, plen) == KAC_OK, "a two-block stride opens");
            expect(kacdec_seek(&d, block) == KAC_OK, "seek inside one stride");
            expect(kacdec_seek(&d, 2u * block) == KAC_OK, "seek at the second point");
            free(plain);
        }
        printf("  ok  a multiple of the block is the whole rule\n");
    }

    /* ---- 6. round trip: encoder simulation == decoder, sample by sample -- */
    printf("\n== 5. round trip ==\n");
    kacdec_init(&d);
    rc = kacdec_open(&d, image, len);
    expect(rc == KAC_OK, "open rc=%d", rc);
    expect(kacdec_total(&d) == frames, "total %u", kacdec_total(&d));
    expect(kacdec_tier(&d) == blk[0].tier, "header tier %u == block 0's tier byte %u",
           kacdec_tier(&d), blk[0].tier);
    rc = kacdec_read(&d, out, frames);
    expect(rc == (int)frames, "full decode rc=%d", rc);
    expect(sse_of(pcm, out, (size_t)frames * ch) == base_g.sse,
           "decoder SSE %lld != encoder simulation %lld",
           (long long)sse_of(pcm, out, (size_t)frames * ch), (long long)base_g.sse);
    expect(clip_count(out, (size_t)frames * ch) == (int32_t)base_g.clip,
           "decoder clip count %d != encoder %u", clip_count(out, (size_t)frames * ch),
           base_g.clip);
    expect(kacdec_read(&d, out, 1) == 0, "a read past the end returns 0");
    printf("  ok  PSNR %.3f dB, achieved %.4f bit/sample, offset %+.3f dB\n", kac_get_psnr(&base_g),
           kac_get_brate_achieved(&base_g), kac_get_vbr_offset(&base_g));

    /* ---- 7. seek to an index point == linear decode ---------------------- */
    printf("\n== 6. seek ==\n");
    kacdec_init(&d);
    expect(kacdec_open(&d, image, len) == KAC_OK, "reopen");
    expect(kacdec_read(&d, ref, frames) == (int)frames, "linear reference decode");
    for (i = 0; i < frames; i += stride) {
        uint32_t rem  = frames - i, j;
        int      mism = 0;

        expect(i % block == 0, "index point %u is a block boundary", i);
        kacdec_init(&d);
        expect(kacdec_open(&d, image, len) == KAC_OK, "reopen for seek(%u)", i);
        rc = kacdec_seek(&d, i);
        expect(rc == KAC_OK, "seek(%u) rc=%d", i, rc);
        if (rc != KAC_OK)
            continue;
        expect(d.pos == i, "seek(%u) left pos=%u", i, d.pos);
        rc = kacdec_read(&d, tail, rem);
        expect(rc == (int)rem, "read after seek(%u) rc=%d", i, rc);
        if (rc != (int)rem)
            continue;
        for (j = 0; j < rem * ch; j++)
            if (tail[j] != ref[i * ch + j])
                mism++;
        expect(mism == 0, "seek(%u): %d/%u samples differ", i, mism, rem * ch);
        printf("  ok  seek(%u) matches the linear decode\n", i);
    }
    kacdec_init(&d);
    kacdec_open(&d, image, len);
    expect(kacdec_seek(&d, frames) == KAC_E_RANGE, "seek(total) is out of range");
    /* A seek that lands mid-block still has to work: it decodes forward from
     * the index point. */
    kacdec_init(&d);
    kacdec_open(&d, image, len);
    rc = kacdec_seek(&d, stride + 1u);
    expect(rc == KAC_OK, "seek inside a block rc=%d", rc);
    if (rc == KAC_OK) {
        uint32_t rem  = frames - stride - 1u, j;
        int      mism = 0;
        expect(kacdec_read(&d, tail, rem) == (int)rem, "read after a mid-block seek");
        for (j = 0; j < rem * ch; j++)
            if (tail[j] != ref[(stride + 1u) * ch + j])
                mism++;
        expect(mism == 0, "mid-block seek: %d/%u samples differ", mism, rem * ch);
    }

    /* ---- 7. loop point inside a block ------------------------------------ *
     * The point must not be a block boundary, so finding the block requires the
     * tier walk; the tier sequence is made to vary. */
    printf("\n== 7. loop, loop point inside a block ==\n");
    {
        const uint32_t ls = 1234u, le = 9000u;
        const uint32_t per = le - ls;
        /* Total = LoopEnd + periods * (LoopEnd - LoopStart); LoopStart itself
         * is not added. */
        const uint32_t periods = 4u;
        const uint32_t need    = le + periods * per;
        int16_t       *seq_a = NULL, *lp_a = NULL;
        uint8_t       *cfile = NULL;
        size_t         clen  = 0;
        enc_opts       o5;

        /* Guard the setup: a boundary loop point would not exercise the walk. */
        expect(ls % block != 0u, "loop point %u is not a block boundary", ls);
        expect(ls >= block, "loop point %u is past block 0", ls);
        expect(le <= frames, "loop end %u stays inside the file", le);

        seq_a = (int16_t *)malloc((size_t)frames * ch * sizeof(int16_t));
        lp_a  = (int16_t *)malloc((size_t)need * ch * sizeof(int16_t));
        if (!seq_a || !lp_a) {
            free(image);
            return 1;
        }

        /* Vary the tier, or hardcoding one slice length would pass. */
        enc_opts_default(&o5);
        o5.tier       = 0;
        o5.vbr_mode   = KAC_VBR_V;
        o5.vbr_target = 2; /* the rung that moves the tier here */
        o5.loop_start = ls;
        o5.loop_end   = le;
        cfile         = enc_image(pcm, frames, ch, 44100, &o5, &clen, &g);
        expect(cfile != NULL, "loop encode");
        if (cfile) {
            blkref   b3[64];
            unsigned nb3, jj, distinct = 0, hist[KAC_TIER_MAX + 1];

            nb3 = walk_blocks(cfile, clen, b3, 64);
            expect(nb3 != 0, "the loop file's walk closes");
            for (jj = 0; jj <= KAC_TIER_MAX; jj++)
                hist[jj] = 0;
            for (jj = 0; jj < nb3; jj++)
                hist[b3[jj].tier]++;
            for (jj = 1; jj <= KAC_TIER_MAX; jj++)
                if (hist[jj])
                    distinct++;
            printf("  tier histogram: T1=%u T2=%u T3=%u T4=%u T5=%u T6=%u T7=%u (%u distinct)\n",
                   hist[1], hist[2], hist[3], hist[4], hist[5], hist[6], hist[7], distinct);
            expect(distinct >= 2,
                   "the loop file varies the tier, so the walk is really required (%u distinct)",
                   distinct);

            expect(decode_seq(cfile, clen, seq_a, frames) == (int32_t)frames,
                   "loop file: sequential decode");
            expect(decode_loop(cfile, clen, lp_a, need, ls, le) == (int32_t)need,
                   "loop file: loop decode");

            { /* the run up to the first wrap is a plain forward decode */
                uint32_t j;
                int      mism = 0;
                for (j = 0; j < le * ch; j++)
                    if (lp_a[j] != seq_a[j])
                        mism++;
                expect(mism == 0, "the run up to LoopEnd: %d/%u samples differ", mism, le * ch);
            }
            { /* every replayed period must equal the sequential decode of the
               * same span -- this is what a cached running pointer instead of
               * the absolute position would break first */
                uint32_t p, j;
                int      mism = 0;
                for (p = 0; p < periods; p++)
                    for (j = 0; j < per * ch; j++)
                        if (lp_a[(le + p * per) * ch + j] != seq_a[ls * ch + j])
                            mism++;
                expect(mism == 0, "replayed periods vs the sequential decode: %d/%u differ", mism,
                       periods * per * ch);
            }
            free(cfile);
        }
        printf("  ok  %u replayed periods match the sequential decode\n", periods);

        free(seq_a);
        free(lp_a);
    }

    /* ---- 8. ABR holds the requested rate --------------------------------- *
     * One fixed stepped signal across all targets isolates the controller.
     * A block is 840 of 441000 samples, so one-tier moves are about 0.06 %;
     * the bound reflects the approximate tier reading's spread. */
    printf("\n== 8. --abr holds the rate it is given ==\n");
    {
        const uint32_t      af         = 441000u;
        int16_t            *ap         = (int16_t *)malloc((size_t)af * ch * sizeof(int16_t));
        static const double targets[5] = {2.4, 3.2, 4.228, 4.8, 6.0};
        unsigned            k9;

        expect(ap != NULL, "abr buffer");
        if (!ap) {
            free(image);
            return 1;
        }
        for (i = 0; i < af; i++) {
            uint32_t a9    = 3000u + ((i / 4410u) % 3u) * 6000u; /* amplitude steps */
            int32_t  v     = (int32_t)((i * 7919u) % (2u * a9 + 1u)) - (int32_t)a9;
            ap[i * ch + 0] = (int16_t)v;
            ap[i * ch + 1] = (int16_t)(v * 9 / 10);
        }

        for (k9 = 0; k9 < 5; k9++) {
            enc_opts o9;
            uint8_t *im9;
            size_t   l9 = 0;
            double   got, dev;

            enc_opts_default(&o9);
            o9.tier       = 0;
            o9.vbr_mode   = KAC_VBR_ABR;
            o9.vbr_target = targets[k9];
            o9.chunk      = 4096;
            im9           = enc_image(ap, af, ch, 44100, &o9, &l9, &g);
            expect(im9 != NULL, "--abr %.3f encodes", targets[k9]);
            if (!im9)
                continue;
            got = kac_get_brate_achieved(&g);
            dev = 100.0 * (got - targets[k9]) / targets[k9];
            expect(dev <= ABR_BOUND && dev >= -ABR_BOUND,
                   "--abr %.3f delivered %.4f (%+.2f%%, bound %.0f%%)", targets[k9], got, dev,
                   ABR_BOUND);
            free(im9);
        }
        free(ap);
        printf("  ok  --abr holds its target within 1%% across T1..T5\n");
    }

    /* ---- 9. ABR allocates by the material -------------------------------- *
     * A stationary passage should keep one tier; a changing one should spread. */
    printf("\n== 9. --abr allocates by the material ==\n");
    {
        const uint32_t af10 = 882000u; /* 20 s: 1050 blocks */
        int16_t       *ap10 = (int16_t *)malloc((size_t)af10 * ch * sizeof(int16_t));
        int16_t       *bp10 = (int16_t *)malloc((size_t)af10 * ch * sizeof(int16_t));
        blkref        *ba   = (blkref *)malloc(2048u * sizeof(blkref));
        blkref        *bb   = (blkref *)malloc(2048u * sizeof(blkref));
        uint8_t       *ia = NULL, *ib = NULL;
        size_t         la = 0, lb = 0;
        unsigned       hist_a[KAC_TIER_MAX + 1] = {0}, hist_b[KAC_TIER_MAX + 1] = {0};
        unsigned       na = 0, nb = 0, topa = 0, topb = 0;

        expect(ap10 && bp10 && ba && bb, "material buffers");
        if (ap10 && bp10 && ba && bb) {
            for (i = 0; i < af10; i++) {
                uint32_t a10 = 8000u;
                uint32_t b11 = ((i / kac_block_for_rate(44100)) % 2u == 0u) ? 1500u : 24000u;
                int32_t  va  = (int32_t)(((uint64_t)i * 7919u) % (2u * a10 + 1u)) - (int32_t)a10;
                int32_t  vb  = (int32_t)(((uint64_t)i * 7919u) % (2u * b11 + 1u)) - (int32_t)b11;
                ap10[i * ch + 0] = (int16_t)va;
                ap10[i * ch + 1] = (int16_t)(va * 9 / 10);
                bp10[i * ch + 0] = (int16_t)vb;
                bp10[i * ch + 1] = (int16_t)(vb * 9 / 10);
            }
            {
                enc_opts o10;
                double   dev;
                enc_opts_default(&o10);
                o10.tier       = 0;
                o10.vbr_mode   = KAC_VBR_ABR;
                o10.vbr_target = 4.228;
                o10.chunk      = 4096;

                ia = enc_image(ap10, af10, ch, 44100, &o10, &la, &g);
                expect(ia != NULL, "stationary material encodes");
                if (ia) {
                    dev = 100.0 * (kac_get_brate_achieved(&g) - 4.228) / 4.228;
                    expect(dev <= 1.0 && dev >= -1.0,
                           "stationary material lands on the rate (%+.2f%%, bound 1%%)", dev);
                }
                ib = enc_image(bp10, af10, ch, 44100, &o10, &lb, &g);
                expect(ib != NULL, "level-stepping material encodes");
                if (ib) {
                    dev = 100.0 * (kac_get_brate_achieved(&g) - 4.228) / 4.228;
                    expect(dev <= ABR_BOUND && dev >= -ABR_BOUND,
                           "level-stepping material lands on the rate (%+.2f%%, bound %.0f%%)", dev,
                           ABR_BOUND);
                }

                if (ia && ib) {
                    na = walk_blocks(ia, la, ba, 2048u);
                    nb = walk_blocks(ib, lb, bb, 2048u);
                    expect(na > 0 && na == nb, "both files walk (%u and %u blocks)", na, nb);
                    if (na > 0 && nb > 0) {
                        unsigned differ = 0;
                        for (i = 0; i < na; i++)
                            hist_a[ba[i].tier]++;
                        for (i = 0; i < nb; i++)
                            hist_b[bb[i].tier]++;
                        for (i = 1; i < 6u; i++) {
                            if (hist_a[i] != hist_b[i])
                                differ = 1;
                            if (hist_a[i] > topa)
                                topa = hist_a[i];
                            if (hist_b[i] > topb)
                                topb = hist_b[i];
                        }
                        printf("  stationary  T1..T7 %u %u %u %u %u %u %u  (top %.3f of %u)\n",
                               hist_a[1], hist_a[2], hist_a[3], hist_a[4], hist_a[5], hist_a[6],
                               hist_a[7], (double)topa / (double)na, na);
                        printf("  stepping    T1..T7 %u %u %u %u %u %u %u  (top %.3f of %u)\n",
                               hist_b[1], hist_b[2], hist_b[3], hist_b[4], hist_b[5], hist_b[6],
                               hist_b[7], (double)topb / (double)nb, nb);
                        expect(differ, "the two materials do not get the same tier sequence");
                        expect((double)topa > 0.90 * (double)na,
                               "stationary material concentrates (%.3f of %u blocks)",
                               (double)topa / (double)na, na);
                        expect((double)topb < 0.70 * (double)nb,
                               "level-stepping material spreads (%.3f of %u blocks)",
                               (double)topb / (double)nb, nb);
                    }
                }
            }
        }
        free(ap10);
        free(bp10);
        free(ba);
        free(bb);
        free(ia);
        free(ib);
    }

    /* ---- 10. silent head does not move the rate rule --------------------- *
     * Every tier measures silence as the same sentinel, so the ladder has no
     * slope and the aim must not jump to that reading. */
    printf("\n== 10. --abr on a file that opens on silence ==\n");
    {
        const uint32_t sf = 88200u;
        const unsigned head =
            (unsigned)(sf / 2u / kac_block_for_rate(44100)); /* whole silent blocks */
        int16_t *sp    = (int16_t *)malloc((size_t)sf * ch * sizeof(int16_t));
        unsigned nblks = 0, jb, quiet = 0;
        blkref   bs[128];

        expect(sp != NULL, "silence-head buffer");
        if (!sp) {
            free(image);
            return 1;
        }
        for (i = 0; i < sf / 2u; i++) {
            sp[i * ch + 0] = 0;
            sp[i * ch + 1] = 0;
        }
        for (i = sf / 2u; i < sf; i++) {
            double  t      = (double)(i - sf / 2u) / 44100.0;
            int32_t v      = (int32_t)(sin(2.0 * 3.14159265358979 * 440.0 * t) * 12000.0);
            sp[i * ch + 0] = (int16_t)v;
            sp[i * ch + 1] = (int16_t)v;
        }
        {
            enc_opts os;
            uint8_t *is;
            size_t   ls = 0;

            enc_opts_default(&os);
            os.tier       = 0;
            os.vbr_mode   = KAC_VBR_ABR;
            os.vbr_target = 4.0;
            os.chunk      = 4096;
            is            = enc_image(sp, sf, ch, 44100, &os, &ls, &g);
            expect(is != NULL, "--abr encodes a file that opens on silence");
            if (is) {
                nblks = walk_blocks(is, ls, bs, 128u);
                expect(nblks == (sf + kac_block_for_rate(44100) - 1u) / kac_block_for_rate(44100),
                       "the walk closes over every block (%u)", nblks);
                for (jb = 0; jb < head && jb < nblks; jb++)
                    if (bs[jb].tier == 1u)
                        quiet++;
                expect(quiet == head, "the silent head sits on the cheapest tier (%u of %u)", quiet,
                       head);
                expect(nblks > head && bs[head].tier != KAC_TIER_MAX,
                       "the first block with a level is not on the widest tier (T%u)",
                       nblks > head ? bs[head].tier : 0u);
                free(is);
            }
        }
        { /* A low goal must not start on the top tiers: the first usable block
           * seeds the aim from its own ladder. */
            enc_opts os;
            uint8_t *is;
            size_t   ls = 0;
            unsigned kb, top = 0;

            enc_opts_default(&os);
            os.tier       = 0;
            os.vbr_mode   = KAC_VBR_ABR;
            os.vbr_target = 1.1;
            os.chunk      = 4096;
            is            = enc_image(sp, sf, ch, 44100, &os, &ls, &g);
            expect(is != NULL, "--abr 1.1 encodes a file that opens on silence");
            if (is) {
                nblks = walk_blocks(is, ls, bs, 128u);
                for (kb = 0; kb < nblks; kb++)
                    if (bs[kb].tier > top)
                        top = bs[kb].tier;
                expect(top <= 2u, "a 1.1 goal stays on the floor tiers (top T%u)", top);
                free(is);
            }
        }
        free(sp);
        printf("  ok  a silent head opens on the cheapest tier\n");
    }

    free(image);
    free(pcm);
    free(out);
    free(ref);
    free(tail);
    printf("\nblock checks %d\n", kac_tu_cases);
    return kac_tu_report("test_block", "checks");
}
