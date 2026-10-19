/* kac_test_util.h -- helper shared by the KAC tests: drive the streaming
 * encoder over a whole buffer and return the resulting file image. */
#ifndef KAC_TEST_UTIL_H
#define KAC_TEST_UTIL_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "kac.h"

#if defined(__GNUC__)
#define KAC_TU_UNUSED __attribute__((unused))
#else
#define KAC_TU_UNUSED
#endif

/* Shared tally, defined here so each single-translation-unit test has one copy.
 * `unit` names that test's cases in the verdict line. */
static int kac_tu_cases = 0;
static int kac_tu_fail  = 0;

static KAC_TU_UNUSED int kac_tu_report(const char *name, const char *unit)
{
    if (kac_tu_fail) {
        fprintf(stderr, "%d %s failed\n", kac_tu_fail, unit);
        return 1;
    }
    printf("%s: ALL PASS (%d %s)\n", name, kac_tu_cases, unit);
    return 0;
}

typedef struct {
    int             tier;       /* 0 = let the bit rate or the default decide */
    double          brate;      /* 0 = unused */
    double          psnr;       /* 0 = unused */
    int             vbr_mode;   /* KAC_VBR_OFF / KAC_VBR_ABR / KAC_VBR_V */
    double          vbr_target; /* ABR: bit/sample; -V: the quality index n */
    double          vbr_psnr;   /* > 0: use kac_set_vbr_target_psnr instead (calibration) */
    int             tier_lo;    /* 0 = the whole ladder */
    int             tier_hi;
    uint32_t        loop_start, loop_end;
    uint32_t        stride;
    uint32_t        chunk;        /* frames per encode call */
    int             quality;      /* the level, when have_quality says so */
    int             have_quality; /* a run that asks for nothing keeps the default */
    int             brute;        /* the advanced search level, 0 = off */
    int             ms_mode;      /* KAC_MS_AUTO, or a form forced for the whole run */
    const kac_snap *init_state;   /* NULL = the default initial state */
} enc_opts;

static KAC_TU_UNUSED void enc_opts_default(enc_opts *o)
{
    memset(o, 0, sizeof *o);
    o->tier     = 3;
    o->vbr_mode = KAC_VBR_OFF;
    o->chunk    = 4096;
}

/* Streaming encode of interleaved pcm; returns a malloc'd file image (caller
 * frees) or NULL, and fills gh with the encoder state for its statistics. */
static KAC_TU_UNUSED uint8_t *enc_image(const int16_t *pcm, uint32_t frames, unsigned ch,
                                        uint32_t rate, const enc_opts *o, size_t *len, kac_t *gh)
{
    kac_t     g;
    uint8_t  *image, *chunkbuf, *flushbuf, head[KAC_HDR_SIZE + 2u * KAC_SNAP_SIZE];
    size_t    cap, fcap, written = 0, headlen = 0, got;
    uint32_t  off, entries = 0;
    uint8_t  *index_store = NULL;
    uint32_t *index_off   = NULL;

    kac_init(&g);
    if (kac_set_num_channels(&g, (int)ch) != 0)
        return NULL;
    if (kac_set_in_samplerate(&g, (int)rate) != 0)
        return NULL;
    if (o->vbr_mode != KAC_VBR_OFF) {
        /* same exclusivity the CLI enforces: a tier selector would fix the tier
         * and leave the per-block measurement nothing to decide */
        if (o->tier > 0 || o->brate > 0.0 || o->psnr > 0.0)
            return NULL;
        if (o->vbr_psnr > 0.0) {
            if (kac_set_vbr_target_psnr(&g, o->vbr_psnr) != 0)
                return NULL;
        } else if (kac_set_vbr(&g, o->vbr_mode, o->vbr_target) != 0) {
            return NULL;
        }
    }
    if (o->tier > 0 && kac_set_tier(&g, o->tier) != 0)
        return NULL;
    if (o->ms_mode != KAC_MS_AUTO && kac_set_ms_mode(&g, o->ms_mode) != 0)
        return NULL;
    if (o->have_quality && kac_set_quality(&g, o->quality) != 0)
        return NULL;
    if (o->brute && kac_set_brute(&g, (unsigned)o->brute) != 0)
        return NULL;
    if (o->brate > 0.0 && kac_set_brate(&g, o->brate) != 0)
        return NULL;
    if (o->psnr > 0.0 && kac_set_vbr_target_psnr(&g, o->psnr) < 0)
        return NULL;
    if (o->loop_end > o->loop_start && kac_set_loop(&g, o->loop_start, o->loop_end) != 0)
        return NULL;
    if (o->init_state && kac_set_init_state(&g, o->init_state) != 0)
        return NULL;
    /* After every selector: each of them resets the window. */
    if (o->tier_lo > 0 && kac_set_tier_limits(&g, o->tier_lo, o->tier_hi) != 0)
        return NULL;
    if (o->stride != 0) {
        entries     = (frames - 1u) / o->stride + 1u;
        index_store = (uint8_t *)malloc((size_t)entries * ch * KAC_SNAP_SIZE);
        if (!index_store)
            return NULL;
        if (kac_set_index(&g, o->stride, index_store, (size_t)entries * ch * KAC_SNAP_SIZE) != 0) {
            free(index_store);
            return NULL;
        }
        /* The offsets cannot be derived (Section 3.4), so they are recorded. */
        index_off = (uint32_t *)malloc((size_t)entries * sizeof(uint32_t));
        if (!index_off || kac_set_index_offsets(&g, index_off, entries) != 0) {
            free(index_off);
            free(index_store);
            return NULL;
        }
    }
    if (kac_init_params(&g) != 0) {
        free(index_off);
        free(index_store);
        return NULL;
    }

    /* A slack ceiling on the whole image, not the per-call bound: every chunk
     * accumulates and then the flush. Two bytes per sample frame is at or above
     * any tier's rate (T7 is exactly two in stereo); the slack covers the rest. */
    cap      = (size_t)frames * 2u + 65536u + (size_t)entries * (4u + (size_t)ch * KAC_SNAP_SIZE);
    image    = (uint8_t *)malloc(cap);
    chunkbuf = (uint8_t *)malloc(kac_encode_bufsize(&g, o->chunk) + 64u);
    /* One buffer of the documented size carries the flush too (tail slice, offset
     * table and index snapshots). The +64 is slack so exceeding the bound is
     * observable; the check after the flush is what goes red. */
    fcap     = kac_encode_bufsize(&g, frames);
    flushbuf = (uint8_t *)malloc(fcap + 64u);
    if (!image || !chunkbuf || !flushbuf) {
        free(image);
        free(chunkbuf);
        free(flushbuf);
        free(index_off);
        free(index_store);
        return NULL;
    }

    for (off = 0; off < frames; off += o->chunk) {
        uint32_t n = frames - off;
        if (n > o->chunk)
            n = o->chunk;
        got = 0;
        if (kac_encode_buffer_interleaved(&g, pcm + (size_t)off * ch, n, chunkbuf, &got) < 0)
            goto fail;
        if (written + got > cap)
            goto fail;
        memcpy(image + written, chunkbuf, got);
        written += got;
    }
    got = 0;
    if (kac_encode_flush(&g, flushbuf, &got) < 0)
        goto fail;
    if (got > fcap) {
        /* kac_encode_bufsize's contract is an upper bound on the flush as well as
         * on one encode call; exceeding it is a bug in the bound, not in the
         * caller, which sized its buffer exactly as the header says to. */
        fprintf(stderr, "FAIL: kac_encode_bufsize(frames=%u) = %zu, flush wrote %zu\n", frames,
                fcap, got);
        goto fail;
    }
    if (written + got > cap)
        goto fail;
    memcpy(image + written, flushbuf, got);
    written += got;
    if (kac_write_preamble(&g, head, &headlen) != 0)
        goto fail;
    memcpy(image, head, headlen); /* replace the placeholder */

    free(chunkbuf);
    free(flushbuf);
    free(index_off);
    free(index_store);
    *len = written;
    if (gh)
        *gh = g;
    return image;
fail:
    free(image);
    free(chunkbuf);
    free(flushbuf);
    free(index_off);
    free(index_store);
    return NULL;
}

/* Encoder statistics recomputed from a decoded signal: PSNR is defined by the
 * sum of squared errors the encoder reports, so comparing this against
 * kac_get_psnr() cross-checks the reconstruction sample by sample. */
static KAC_TU_UNUSED int64_t sse_of(const int16_t *pcm, const int16_t *dec, size_t n)
{
    int64_t sse = 0;
    size_t  i;
    for (i = 0; i < n; i++) {
        int64_t e = (int64_t)pcm[i] - dec[i];
        sse += e * e;
    }
    return sse;
}

#endif /* KAC_TEST_UTIL_H */
