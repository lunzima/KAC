/* test_stream.c -- stream form: file data area without length fields.
 * Checks byte equality with the file data area, chunked decode, validation
 * differences from a file, and low-delay chunking including tier changes. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "kac.h"
#include "kac_test_util.h"

static void check(int ok, const char *what)
{
    kac_tu_cases++;
    if (ok)
        return;
    kac_tu_fail++;
    fprintf(stderr, "FAIL: %s\n", what);
}

static void check_i(long got, long want, const char *what)
{
    kac_tu_cases++;
    if (got == want)
        return;
    kac_tu_fail++;
    fprintf(stderr, "FAIL: %s -> %ld (want %ld)\n", what, got, want);
}

/* The pieces one stream encode call produces, kept apart so the decoder can be
 * fed exactly the way a sender would send them. */
typedef struct {
    uint8_t *body;
    size_t   bodylen;
    size_t   chunk[512]; /* start of each piece in body */
    unsigned nchunk;
    unsigned chunk_frames; /* samples the encoder was asked to emit per chunk */
    uint8_t  head[KAC_HDR_SIZE + 2u * KAC_SNAP_SIZE];
    size_t   headlen;
    kac_t    g;
} stream_enc;

static void stream_enc_free(stream_enc *s)
{
    free(s->body);
    s->body = NULL;
}

/* Feed chunk-sized calls and write the preamble after chunk 0, whose tier and
 * form it carries. switch_piece changes tier after that many chunks. */
static int stream_encode(stream_enc *s, const int16_t *pcm, uint32_t frames, unsigned ch,
                         uint32_t rate, int vbr_mode, double target, int tier, int ms_mode,
                         unsigned chunk_ms, unsigned switch_piece, int switch_tier,
                         const kac_snap *init)
{
    uint8_t *buf;
    uint32_t off;
    size_t   cap, written = 0, got;
    unsigned i;

    memset(s, 0, sizeof *s);
    kac_init(&s->g);
    if (kac_set_num_channels(&s->g, (int)ch) != 0)
        return -1;
    if (kac_set_in_samplerate(&s->g, (int)rate) != 0)
        return -1;
    if (vbr_mode != KAC_VBR_OFF) {
        if (kac_set_vbr(&s->g, vbr_mode, target) != 0)
            return -1;
    } else if (kac_set_tier(&s->g, tier) != 0) {
        return -1;
    }
    if (ms_mode != KAC_MS_AUTO && kac_set_ms_mode(&s->g, ms_mode) != 0)
        return -1;
    if (init && kac_set_init_state(&s->g, init) != 0)
        return -1;
    if (kac_init_params(&s->g) != 0)
        return -1;
    if (kac_set_stream(&s->g, 1) != 0)
        return -1;
    if (chunk_ms > 0u && kac_set_chunk_ms(&s->g, chunk_ms) != 0)
        return -1;
    s->chunk_frames = kac_get_chunk_samples(&s->g);

    cap     = (size_t)frames * 2u + 65536u;
    s->body = (uint8_t *)malloc(cap);
    buf     = (uint8_t *)malloc(kac_encode_bufsize(&s->g, s->chunk_frames) + 64u);
    if (!s->body || !buf) {
        free(buf);
        return -1;
    }

    for (off = 0; off < frames; off += s->chunk_frames) {
        uint32_t n = frames - off;
        if (n > s->chunk_frames)
            n = s->chunk_frames;
        got = 0;
        if (kac_encode_buffer_interleaved(&s->g, pcm + (size_t)off * ch, n, buf, &got) < 0) {
            free(buf);
            return -1;
        }
        if (got > 0u) {
            if (written + got > cap || s->nchunk >= 512u) {
                free(buf);
                return -1;
            }
            s->chunk[s->nchunk++] = written;
            memcpy(s->body + written, buf, got);
            written += got;
        }
        if (s->nchunk == 1u && kac_write_stream_preamble(&s->g, s->head, &s->headlen) != 0) {
            free(buf);
            return -1;
        }
        if (switch_piece != 0u && s->nchunk == switch_piece) {
            /* The next chunk runs on another tier. The chunk length is a
             * multiple of both tiers' N, so the framing stays whole. */
            s->g.tier    = switch_tier;
            s->g.tier_lo = switch_tier;
            s->g.tier_hi = switch_tier;
        }
    }
    got = 0;
    if (kac_encode_flush(&s->g, buf, &got) < 0) {
        free(buf);
        return -1;
    }
    if (got > 0u) {
        if (written + got > cap || s->nchunk >= 512u) {
            free(buf);
            return -1;
        }
        s->chunk[s->nchunk++] = written;
        memcpy(s->body + written, buf, got);
        written += got;
    }
    free(buf);
    s->bodylen = written;
    for (i = 0; i < s->nchunk; i++)
        check(s->chunk[i] < s->bodylen, "each piece starts inside the body");
    return 0;
}

/* Decode a stream the way a receiver does: feed a piece, drain it, feed the
 * next. Returns the sample count; dec must hold cap samples. */
static uint32_t stream_decode(const stream_enc *s, unsigned ch, int16_t *dec, uint32_t cap)
{
    kacdec_t d;
    uint32_t total = 0u;
    unsigned i;

    kacdec_init(&d);
    if (kacdec_stream_open(&d, s->head, s->headlen) != KAC_OK) {
        check(0, "kacdec_stream_open");
        return 0u;
    }
    for (i = 0; i < s->nchunk; i++) {
        size_t  end = (i + 1u < s->nchunk) ? s->chunk[i + 1u] : s->bodylen;
        int32_t got;

        if (kacdec_stream_feed(&d, s->body + s->chunk[i], end - s->chunk[i]) != KAC_OK) {
            check(0, "kacdec_stream_feed");
            kacdec_close(&d);
            return 0u;
        }
        while (total < cap && (got = kacdec_read(&d, dec + (size_t)total * ch, cap - total)) > 0)
            total += (uint32_t)got;
        check(total <= kacdec_total(&d), "the decoder never runs ahead of its input");
    }
    kacdec_close(&d);
    return total;
}

static uint32_t file_decode(const uint8_t *image, size_t len, unsigned ch, int16_t *dec)
{
    kacdec_t d;
    int32_t  got;
    uint32_t total = 0u;

    kacdec_init(&d);
    if (kacdec_open(&d, image, len) != KAC_OK) {
        check(0, "kacdec_open (reference)");
        return 0u;
    }
    while ((got = kacdec_read(&d, dec + (size_t)total * ch, 1024u)) > 0)
        total += (uint32_t)got;
    kacdec_close(&d);
    return total;
}

/* The largest N in the tier table, T1's. A stream's last chunk decodes at most
 * N - 1 samples past the sender's last one, and N is that chunk's own tier, so
 * the bound has to be the table's largest rather than any one tier's. */
static unsigned tier_nmax(void)
{
    unsigned t, nmax = 0u;
    for (t = 0u; t < (unsigned)KAC_TIER_MAX; t++)
        if (kac_tiers[t].n > nmax)
            nmax = kac_tiers[t].n;
    return nmax;
}

/* One full pass at one setting. */
static void run_case(const int16_t *pcm, uint32_t frames, unsigned ch, uint32_t rate, int vbr_mode,
                     double target, int tier, const kac_snap *init, const char *name)
{
    enc_opts   o;
    uint8_t   *file;
    size_t     filelen = 0, data_bytes;
    kac_t      fg;
    stream_enc s;
    int16_t   *ref, *dec;
    uint32_t   reftotal, streamtotal;
    char       msg[160];

    enc_opts_default(&o);
    if (vbr_mode != KAC_VBR_OFF) {
        o.tier       = 0;
        o.vbr_mode   = vbr_mode;
        o.vbr_target = target;
    } else {
        o.tier = tier;
    }
    o.init_state = init;
    file         = enc_image(pcm, frames, ch, rate, &o, &filelen, &fg);
    if (!file) {
        check(0, "reference encode");
        return;
    }
    data_bytes = kac_get_data_bytes(&fg);

    if (stream_encode(&s, pcm, frames, ch, rate, vbr_mode, target, tier, KAC_MS_AUTO, 0u, 0u, 0,
                      init) != 0) {
        check(0, "stream encode");
        free(file);
        return;
    }

    /* 1. The stream's blocks are the file's data area, byte for byte. Without a
     * seek index the data area is the tail of the image. */
    check_i((long)s.bodylen, (long)data_bytes, "the stream body is the file's data area");
    check(data_bytes <= filelen, "the data area is inside the image");
    check(data_bytes == 0u || memcmp(file + (filelen - data_bytes), s.body, s.bodylen) == 0,
          "the data area and the stream body are the same bytes");

    /* 2. Both decode to the same samples. */
    ref = (int16_t *)malloc(((size_t)frames + KAC_CHUNK_SAMPLES) * ch * sizeof(int16_t));
    dec = (int16_t *)malloc(((size_t)frames + KAC_CHUNK_SAMPLES) * ch * sizeof(int16_t));
    if (!ref || !dec) {
        check(0, "malloc");
        free(ref);
        free(dec);
        free(file);
        stream_enc_free(&s);
        return;
    }
    reftotal    = file_decode(file, filelen, ch, ref);
    streamtotal = stream_decode(&s, ch, dec, frames + KAC_CHUNK_SAMPLES);

    check_i((long)reftotal, (long)frames, "the file decode returns every sample");
    check(streamtotal >= reftotal, "the stream decodes at least as many samples");
    /* A stream's length is the sample count its bytes admit, so its last block
     * is the sender's remainder rounded up to that block's slice: at most one
     * slice short, with the slice set by that block's own tier. */
    check(streamtotal - reftotal < tier_nmax(), "the stream's overrun is under one slice");
    {
        uint32_t j;
        long     first = -1;
        for (j = 0; j < (uint32_t)reftotal * ch; j++)
            if (ref[j] != dec[j]) {
                first = (long)j;
                break;
            }
        snprintf(msg, sizeof msg,
                 "the stream decodes the same samples as the file "
                 "(first difference at %ld: %d vs %d)",
                 first, first < 0 ? 0 : ref[first], first < 0 ? 0 : dec[first]);
        check(first < 0, msg);
    }

    snprintf(msg, sizeof msg, "  %s: %u file samples, %u stream samples", name, reftotal,
             streamtotal);
    printf("%s\n", msg);

    /* 3. A stream is not a file. */
    {
        kacdec_t d;
        kacdec_init(&d);
        check(kacdec_open(&d, s.head, s.headlen) == KAC_E_TOTAL,
              "the whole-image open refuses a stream preamble");
    }

    free(ref);
    free(dec);
    free(file);
    stream_enc_free(&s);
}

/* One chunk's slice bytes, its tier byte removed. Returns the new output
 * length, or (size_t)-1 when the piece does not fit. */
static size_t strip_piece(const uint8_t *p, size_t len, uint8_t *out, size_t at, size_t cap)
{
    if (len < 1u || at > cap || len - 1u > cap - at)
        return (size_t)-1;
    memcpy(out + at, p + 1u, len - 1u);
    return at + len - 1u;
}

static uint32_t rd32(const unsigned char *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* The file's data area with every tier byte removed, walking the file
 * grid. Returns the length, or (size_t)-1 if the image does not walk. */
static size_t file_slices(const uint8_t *image, size_t image_len, uint8_t *out, size_t cap)
{
    const unsigned ch         = image[6];
    const uint32_t blk        = kac_block_for_rate(rd32(image + 8));
    uint32_t       total      = rd32(image + 12);
    uint32_t       data_bytes = rd32(image + 24);
    uint32_t       covered    = 0;
    size_t         pos        = KAC_HDR_SIZE;
    size_t         at         = 0u;
    size_t         start;

    if (image[5] & KAC_FLAG_INIT)
        pos += (size_t)ch * KAC_SNAP_SIZE;
    if (image[5] & KAC_FLAG_LOOP)
        pos += (size_t)ch * KAC_SNAP_SIZE;
    start = pos;
    while (covered < total) {
        unsigned t = image[pos] & KAC_TIER_MASK;
        unsigned n, len, nsl, bytes;

        if (t < KAC_TIER_MIN || t > KAC_TIER_MAX)
            return (size_t)-1;
        n   = kac_tiers[t - 1u].n;
        len = total - covered;
        if (len > blk)
            len = blk;
        nsl   = (len + n - 1u) / n;
        bytes = 1u + nsl * KAC_SLICE_SIZE * ch;
        at    = strip_piece(image + pos, bytes, out, at, cap);
        if (at == (size_t)-1)
            return at;
        pos += bytes;
        covered += len;
    }
    if (pos - start != data_bytes || pos > image_len)
        return (size_t)-1;
    return at;
}

/* A stream's slice bytes, its chunk tier bytes removed. */
static size_t stream_slices(const stream_enc *s, uint8_t *out, size_t cap)
{
    size_t   at = 0u;
    unsigned i;

    for (i = 0; i < s->nchunk; i++) {
        size_t end = (i + 1u < s->nchunk) ? s->chunk[i + 1u] : s->bodylen;

        at = strip_piece(s->body + s->chunk[i], end - s->chunk[i], out, at, cap);
        if (at == (size_t)-1)
            return at;
    }
    return at;
}

/* A fixed tier with a forced form, sent in short chunks. The slice sequence must
 * be the file path's, and both must decode to the same samples. */
static void run_chunk_case(const int16_t *pcm, uint32_t frames, unsigned ch, uint32_t rate,
                           int tier, int ms_mode, unsigned chunk_ms, unsigned want_chunk,
                           const char *name)
{
    enc_opts   o;
    uint8_t   *file, *fsl, *ssl;
    size_t     filelen = 0, flen, slen, cap;
    kac_t      fg;
    stream_enc s;
    int16_t   *ref, *dec;
    uint32_t   reftotal, streamtotal;
    char       msg[192];

    enc_opts_default(&o);
    o.tier    = tier;
    o.ms_mode = ms_mode;
    file      = enc_image(pcm, frames, ch, rate, &o, &filelen, &fg);
    if (!file) {
        check(0, "chunk case: reference encode");
        return;
    }
    if (stream_encode(&s, pcm, frames, ch, rate, KAC_VBR_OFF, 0.0, tier, ms_mode, chunk_ms, 0u, 0,
                      NULL) != 0) {
        check(0, "chunk case: stream encode");
        free(file);
        return;
    }
    snprintf(msg, sizeof msg, "%s: chunk is %u samples", name, want_chunk);
    check_i((long)s.chunk_frames, (long)want_chunk, msg);

    cap = (size_t)frames * 2u + 65536u;
    fsl = (uint8_t *)malloc(cap);
    ssl = (uint8_t *)malloc(cap);
    ref = (int16_t *)malloc(((size_t)frames + KAC_CHUNK_SAMPLES) * ch * sizeof(int16_t));
    dec = (int16_t *)malloc(((size_t)frames + KAC_CHUNK_SAMPLES) * ch * sizeof(int16_t));
    if (!fsl || !ssl || !ref || !dec) {
        check(0, "chunk case: malloc");
        goto out;
    }
    flen = file_slices(file, filelen, fsl, cap);
    slen = stream_slices(&s, ssl, cap);
    check(flen != (size_t)-1 && slen != (size_t)-1, "chunk case: both data areas walk");
    if (flen != (size_t)-1 && slen != (size_t)-1) {
        snprintf(msg, sizeof msg, "%s: the slices are the file path's (%zu vs %zu)", name, slen,
                 flen);
        check(slen == flen && memcmp(fsl, ssl, flen) == 0, msg);
    }

    reftotal    = file_decode(file, filelen, ch, ref);
    streamtotal = stream_decode(&s, ch, dec, frames + KAC_CHUNK_SAMPLES);
    snprintf(msg, sizeof msg, "%s: the file decode returns every sample", name);
    check_i((long)reftotal, (long)frames, msg);
    snprintf(msg, sizeof msg, "%s: the stream decode covers the file", name);
    check(streamtotal >= reftotal && streamtotal - reftotal < tier_nmax(), msg);
    if (streamtotal >= reftotal) {
        size_t j;
        long   first = -1;

        for (j = 0; j < (size_t)reftotal * ch; j++)
            if (ref[j] != dec[j]) {
                first = (long)j;
                break;
            }
        snprintf(msg, sizeof msg, "%s: the stream decodes the same samples (first at %ld)", name,
                 first);
        check(first < 0, msg);
    }
out:
    free(fsl);
    free(ssl);
    free(ref);
    free(dec);
    free(file);
    stream_enc_free(&s);
}

/* A tier change between chunks, against the same change on a 960-sample chunk: both
 * streams carry the same slices, so their decodes agree sample for sample. */
static void run_switch_case(const int16_t *pcm, uint32_t frames, unsigned ch, uint32_t rate,
                            const char *name)
{
    stream_enc a, b;
    uint8_t   *asl, *bsl;
    size_t     alen, blen, cap;
    int16_t   *da, *db;
    uint32_t   ta, tb;
    char       msg[192];

    /* 240 samples per chunk on T2 at 48 kHz, switching to T3 after 960
     * samples; the reference keeps the same tiers and switch point on a
     * 960-sample chunk. 240 is a multiple of both N's (30 and 20). */
    if (stream_encode(&a, pcm, frames, ch, rate, KAC_VBR_OFF, 0.0, 2, KAC_MS_MS, 5u, 4u, 3, NULL) !=
            0 ||
        stream_encode(&b, pcm, frames, ch, rate, KAC_VBR_OFF, 0.0, 2, KAC_MS_MS, 1000u, 1u, 3,
                      NULL) != 0) {
        check(0, "switch case: encode");
        return;
    }
    snprintf(msg, sizeof msg, "%s: short chunk is 240 samples", name);
    check_i((long)a.chunk_frames, 240, msg);
    snprintf(msg, sizeof msg, "%s: reference chunk is 960 samples", name);
    check_i((long)b.chunk_frames, KAC_CHUNK_SAMPLES, msg);
    /* Chunk 0's own byte and the first chunk after the switch. */
    check_i((long)(a.body[a.chunk[0]] & KAC_TIER_MASK), 2, "switch case: first chunk is T2");
    check_i((long)(a.body[a.chunk[4]] & KAC_TIER_MASK), 3, "switch case: later chunk is T3");

    cap = (size_t)frames * 2u + 65536u;
    asl = (uint8_t *)malloc(cap);
    bsl = (uint8_t *)malloc(cap);
    da  = (int16_t *)malloc(((size_t)frames + KAC_CHUNK_SAMPLES) * ch * sizeof(int16_t));
    db  = (int16_t *)malloc(((size_t)frames + KAC_CHUNK_SAMPLES) * ch * sizeof(int16_t));
    if (!asl || !bsl || !da || !db) {
        check(0, "switch case: malloc");
        goto out;
    }
    alen = stream_slices(&a, asl, cap);
    blen = stream_slices(&b, bsl, cap);
    snprintf(msg, sizeof msg, "%s: both slice sequences are the same length (%zu vs %zu)", name,
             alen, blen);
    check(alen != (size_t)-1 && alen == blen, msg);
    if (alen != (size_t)-1 && alen == blen) {
        snprintf(msg, sizeof msg, "%s: the tier change does not move a slice", name);
        check(memcmp(asl, bsl, alen) == 0, msg);
    }
    ta = stream_decode(&a, ch, da, frames + KAC_CHUNK_SAMPLES);
    tb = stream_decode(&b, ch, db, frames + KAC_CHUNK_SAMPLES);
    snprintf(msg, sizeof msg, "%s: both streams decode the same count", name);
    check_i((long)ta, (long)tb, msg);
    if (ta == tb) {
        snprintf(msg, sizeof msg, "%s: the tier change decodes the same samples", name);
        check(memcmp(da, db, (size_t)ta * ch * sizeof(int16_t)) == 0, msg);
    }
out:
    free(asl);
    free(bsl);
    free(da);
    free(db);
    stream_enc_free(&a);
    stream_enc_free(&b);
}

int main(void)
{
    const uint32_t frames = 5000u; /* five whole blocks and a remainder */
    const unsigned ch     = 2u;
    const uint32_t rate   = 44100u;
    int16_t       *pcm, *pcm1;
    uint32_t       i;
    kac_snap       snaps[KAC_MAX_CHANNELS];

    pcm  = (int16_t *)malloc((size_t)frames * ch * sizeof(int16_t));
    pcm1 = (int16_t *)malloc((size_t)frames * sizeof(int16_t));
    if (!pcm || !pcm1)
        return 1;
    for (i = 0; i < frames * ch; i++) {
        double t = (double)(i / ch) / 44100.0;
        double v = sin(2.0 * 3.14159265358979 * 220.0 * t) * 12000.0 +
                   sin(2.0 * 3.14159265358979 * 3000.0 * t) * 4000.0 * (1.0 - t / 0.2);
        pcm[i] = (int16_t)v;
    }
    for (i = 0; i < frames; i++) {
        double t = (double)i / 44100.0;
        pcm1[i]  = (int16_t)(sin(2.0 * 3.14159265358979 * 440.0 * t) * 14000.0);
    }

    printf("== block framing ==\n");
    run_case(pcm, frames, ch, rate, KAC_VBR_OFF, 0.0, 3, NULL, "fixed tier T3");
    run_case(pcm, frames, ch, rate, KAC_VBR_V, 6.0, 0, NULL, "per-block -V 6");
    run_case(pcm, frames, ch, rate, KAC_VBR_ABR, 3.2, 0, NULL, "per-block --abr 3.2");
    run_case(pcm1, frames, 1u, rate, KAC_VBR_V, 6.0, 0, NULL, "per-block -V 6 mono");
    run_case(pcm1, frames, 1u, rate, KAC_VBR_OFF, 0.0, 3, NULL, "fixed tier T3 mono");

    /* An initial state that came from somewhere else, so the preamble carries
     * INIT_STATE and both paths load it. */
    {
        enc_opts o;
        uint8_t *img;
        size_t   len = 0;
        kac_t    g;

        enc_opts_default(&o);
        o.tier = 3;
        img    = enc_image(pcm1, 2000u, 1u, rate, &o, &len, &g);
        if (!img || kac_get_state(&g, snaps) != 0) {
            check(0, "build an initial state");
        } else {
            run_case(pcm1, frames, 1u, rate, KAC_VBR_V, 6.0, 0, snaps,
                     "per-block -V 6, INIT_STATE");
            run_case(pcm1, frames, 1u, rate, KAC_VBR_OFF, 0.0, 3, snaps,
                     "fixed tier T3, INIT_STATE");
        }
        free(img);
    }

    printf("== low-delay chunks ==\n");
    /* A pinned tier plus a forced form codes each chunk once, so the chunk
     * boundary decides only where the tier byte goes. 1 slice at T2 (30
     * samples), k slices at T3 (5 ms = 11 slices = 220 samples), and mono. */
    run_chunk_case(pcm, frames, ch, rate, 2, KAC_MS_MS, 1u, 30u, "T2, 1-slice chunk");
    run_chunk_case(pcm, frames, ch, rate, 3, KAC_MS_MS, 5u, 220u, "T3, 5 ms chunk");
    run_chunk_case(pcm, frames, ch, rate, 3, KAC_MS_LR, 5u, 220u, "T3, 5 ms chunk, LR");
    run_chunk_case(pcm1, frames, 1u, rate, 3, KAC_MS_LR, 5u, 220u, "T3, 5 ms chunk, mono");
    run_chunk_case(pcm, frames, ch, 8000u, 3, KAC_MS_MS, 5u, 40u, "T3, 5 ms chunk at 8 kHz");
    run_switch_case(pcm, frames, ch, 48000u, "T2 to T3 between chunks");

    printf("== feed contract ==\n");
    {
        stream_enc s;
        kacdec_t   d;
        uint8_t    bad[64];
        int16_t    sink[8];
        int16_t    out2[2u * KAC_CHUNK_SAMPLES];
        size_t     first;

        if (stream_encode(&s, pcm, frames, ch, rate, KAC_VBR_V, 6.0, 0, KAC_MS_AUTO, 0u, 0u, 0,
                          NULL) != 0) {
            check(0, "stream encode (contract)");
        } else {
            first = (s.nchunk > 1u) ? s.chunk[1] : s.bodylen;

            /* A file handle is not fed. */
            kacdec_init(&d);
            check(kacdec_stream_feed(&d, s.body, 8u) == KAC_E_ARG,
                  "the feed path refuses a handle no stream opened");

            kacdec_init(&d);
            check(kacdec_stream_open(&d, s.head, s.headlen) == KAC_OK, "stream open");
            check(kacdec_stream_feed(&d, s.body, first) == KAC_OK, "first feed");
            check(kacdec_stream_feed(&d, s.body + first, 8u) == KAC_E_ARG,
                  "a second feed before the first is decoded is refused");

            /* A stream carries no loop region, so loop mode cannot wrap and the
             * refusal has to surface where the caller can see it. */
            kacdec_set_loop(&d, 1);
            check(kacdec_read(&d, sink, 4u) == KAC_E_NOLOOP,
                  "loop mode on a stream reports at the read");
            kacdec_close(&d);

            /* A failed open leaves an inert handle, not the previous stream. */
            kacdec_init(&d);
            kacdec_stream_open(&d, s.head, s.headlen);
            memcpy(bad, s.head, s.headlen < sizeof bad ? s.headlen : sizeof bad);
            bad[4] = 99u; /* Version */
            bad[6] = 0u;  /* Channels */
            check(kacdec_stream_open(&d, bad, s.headlen) == KAC_E_VERSION,
                  "a bad preamble is refused");
            check(kacdec_stream_feed(&d, s.body, first) == KAC_E_ARG,
                  "a failed open leaves the handle out of the stream path");
            check(kacdec_read(&d, sink, 4u) == 0, "a failed open decodes nothing");
            kacdec_close(&d);

            /* A piece that is not whole slices is refused by length. */
            kacdec_init(&d);
            kacdec_stream_open(&d, s.head, s.headlen);
            memcpy(bad, s.body, sizeof bad);
            check(kacdec_stream_feed(&d, bad, 9u) == KAC_E_SIZE,
                  "a piece that is not whole slices is refused");
            kacdec_close(&d);

            /* Two rules live on the same byte, and its position decides which
             * is reached: on chunk 0 any byte other than the header's tier is
             * refused first, so the range rule is not exercised there. Both are
             * checked, each off the other's ground. */
            {
                uint8_t  blk2[1537];
                unsigned hdr = s.head[7];
                size_t   l0  = first;
                size_t   l1  = ((s.nchunk > 2u) ? s.chunk[2] : s.bodylen) - s.chunk[1];

                if (l0 > sizeof blk2)
                    l0 = sizeof blk2;
                if (l1 > sizeof blk2)
                    l1 = sizeof blk2;

                kacdec_init(&d);
                kacdec_stream_open(&d, s.head, s.headlen);
                memcpy(blk2, s.body, l0);
                blk2[0] = (uint8_t)(hdr == 6u ? 5u : hdr + 1u); /* legal, not this one */
                check(kacdec_stream_feed(&d, blk2, l0) == KAC_E_TIER,
                      "chunk 0 must carry the header's tier");
                kacdec_close(&d);

                kacdec_init(&d);
                kacdec_stream_open(&d, s.head, s.headlen);
                kacdec_stream_feed(&d, s.body, first);
                while (kacdec_read(&d, out2, KAC_CHUNK_SAMPLES) > 0)
                    ; /* drain, so the next byte is not chunk 0's */
                memcpy(blk2, s.body + s.chunk[1], l1);
                blk2[0] = 0u;
                check(kacdec_stream_feed(&d, blk2, l1) == KAC_E_TIER,
                      "a tier byte below the range is refused off chunk 0");
                blk2[0] = 8u;
                check(kacdec_stream_feed(&d, blk2, l1) == KAC_E_TIER,
                      "a tier byte above the range is refused off chunk 0");
                kacdec_close(&d);
            }

            /* A stream preamble with SEEK or LOOP: the index is a trailer a
             * stream never reaches, and wrapping needs a block the decoder has
             * already given back. */
            {
                uint8_t head[KAC_HDR_SIZE + 2u * KAC_SNAP_SIZE];
                memcpy(head, s.head, s.headlen);
                head[5] = (uint8_t)(head[5] | KAC_FLAG_SEEK);
                kacdec_init(&d);
                check(kacdec_stream_open(&d, head, s.headlen) == KAC_E_FLAGS,
                      "SEEK is refused in a stream");
                head[5] = (uint8_t)(s.head[5] | KAC_FLAG_LOOP);
                kacdec_init(&d);
                check(kacdec_stream_open(&d, head, s.headlen) == KAC_E_LOOP,
                      "LOOP is refused in a stream");
                /* The chunk length is what frames the stream; zero or more than
                 * one block leaves nothing to frame with. */
                memcpy(head, s.head, s.headlen);
                head[28] = head[29] = head[30] = head[31] = 0u;
                kacdec_init(&d);
                check(kacdec_stream_open(&d, head, s.headlen) == KAC_E_INDEX,
                      "a stream needs a chunk length");
                head[28] = 0xC1u; /* 961 */
                head[29] = 0x03u;
                head[30] = head[31] = 0u;
                kacdec_init(&d);
                check(kacdec_stream_open(&d, head, s.headlen) == KAC_E_INDEX,
                      "a chunk longer than one block is refused");
            }
            stream_enc_free(&s);
        }
    }

    printf("== feed contract, one block ==\n");
    {
        /* At a pinned tier, check that a feed is limited to one block. */
        stream_enc     s;
        kacdec_t       d;
        int16_t        out[2u * KAC_CHUNK_SAMPLES];
        const unsigned n      = kac_tiers[3 - 1].n; /* T3: 20 samples per slice */
        const size_t   one    = 1u + (kac_block_for_rate(rate) / n) * KAC_SLICE_SIZE * ch;
        const size_t   maxone = 1u + (KAC_CHUNK_SAMPLES / n) * KAC_SLICE_SIZE * ch;

        if (stream_encode(&s, pcm, frames, ch, rate, KAC_VBR_OFF, 0.0, 3, KAC_MS_AUTO, 0u, 0u, 0,
                          NULL) != 0) {
            check(0, "stream encode (fixed tier)");
        } else if (s.nchunk < 2u || s.bodylen < one) {
            check(0, "a fixed tier stream is more than one block");
        } else {
            uint32_t stated = (uint32_t)s.head[28] | ((uint32_t)s.head[29] << 8) |
                              ((uint32_t)s.head[30] << 16) | ((uint32_t)s.head[31] << 24);

            check(stated == kac_block_for_rate(rate), "the preamble states the chunk length");
            kacdec_init(&d);
            check(kacdec_stream_open(&d, s.head, s.headlen) == KAC_OK, "fixed tier stream open");
            check(kacdec_stream_feed(&d, s.body, s.chunk[1]) == KAC_OK, "fixed tier first feed");
            /* Drain it, or the next feed is refused for the other reason. */
            while (kacdec_read(&d, out, KAC_CHUNK_SAMPLES) > 0)
                ;
            check(kacdec_stream_feed(&d, s.body + s.chunk[1], one) == KAC_OK,
                  "a whole block is a well formed piece");
            while (kacdec_read(&d, out, KAC_CHUNK_SAMPLES) > 0)
                ;
            /* One more slice is past the largest chunk a feed may carry. */
            check(kacdec_stream_feed(&d, s.body + s.chunk[1], maxone + KAC_SLICE_SIZE * ch) ==
                      KAC_E_SIZE,
                  "a piece past the largest chunk is refused");
            check(kacdec_stream_feed(&d, s.body + s.chunk[1], one + 1u) == KAC_E_SIZE,
                  "a piece that is not whole slices is refused");
            kacdec_close(&d);
        }
        stream_enc_free(&s);
    }

    printf("== encoder contract ==\n");
    {
        kac_t   g;
        uint8_t buf[16384];
        size_t  got = 0, bound;

        kac_init(&g);
        kac_init_params(&g);
        check(kac_set_stream(&g, 1) == 0, "a fresh handle takes the stream mode");
        bound = kac_encode_bufsize(&g, 960u);
        /* A stream states no length, so the encoder emits no placeholder: the
         * first call's output is data, and kac_encode_bufsize() bounds it
         * without counting a prefix. */
        check(kac_encode_buffer_interleaved(&g, pcm, 960u, buf, &got) == (int)got && got > 0u,
              "the first call emits a block and no prefix");
        check(got <= bound, "the block fits the documented bound");
        check(kac_write_stream_preamble(&g, buf, &got) == 0,
              "the preamble is available once block 0 is encoded");
        /* Both directions are refused once anything has been emitted: the handle
         * has committed to one form and neither form can be undone. */
        check(kac_set_stream(&g, 1) == KAC_E_ARG, "the mode cannot be set mid stream");
        check(kac_set_stream(&g, 0) == KAC_E_ARG, "the mode cannot be cleared mid stream");
        kac_close(&g);

        kac_init(&g);
        check(kac_set_loop(&g, 100u, 200u) == 0, "loop region set");
        kac_init_params(&g);
        check(kac_set_stream(&g, 1) == KAC_E_ARG, "a loop region is refused in a stream");
        kac_close(&g);

        kac_init(&g);
        kac_init_params(&g);
        check(kac_write_stream_preamble(&g, buf, &got) == KAC_E_ARG,
              "the preamble needs the stream mode");
        kac_close(&g);
    }

    /* The file block follows the sample rate: about 20 ms, a multiple of 120,
     * never past 20.5 ms or the 960-sample maximum. */
    check_i((long)kac_block_for_rate(0u), KAC_CHUNK_LCM, "rate 0 takes the minimum block");
    check_i((long)kac_block_for_rate(8000u), KAC_CHUNK_LCM, "8 kHz takes the minimum block");
    check_i((long)kac_block_for_rate(22050u), 360, "22.05 kHz takes 360 samples");
    check_i((long)kac_block_for_rate(32000u), 600, "32 kHz takes 600 samples");
    check_i((long)kac_block_for_rate(44100u), 840, "44.1 kHz takes 840 samples");
    check_i((long)kac_block_for_rate(48000u), KAC_CHUNK_SAMPLES, "48 kHz takes 960 samples");
    check_i((long)kac_block_for_rate(96000u), KAC_CHUNK_SAMPLES, "96 kHz is capped at 960");

    /* kac_set_chunk_ms(): stream only, before any bytes. */
    {
        kac_t   g;
        uint8_t buf[16384];
        size_t  got = 0;

        kac_init(&g);
        kac_set_tier(&g, 3);
        kac_init_params(&g);
        check(kac_set_chunk_ms(&g, 5u) == KAC_E_ARG, "chunk_ms needs the stream mode");
        check_i((long)kac_get_chunk_samples(&g), kac_block_for_rate(44100),
                "a file handle keeps the block");
        kac_close(&g);

        kac_init(&g);
        kac_init_params(&g);
        check(kac_set_stream(&g, 1) == 0, "stream opens on a per-block handle");
        check_i((long)kac_get_chunk_samples(&g), 840, "a per-block stream defaults to about 20 ms");
        check(kac_set_chunk_ms(&g, 5u) == 0, "a per-block stream takes a chunk length");
        check_i((long)kac_get_chunk_samples(&g), 240,
                "5 ms at 44.1 kHz is 240 samples, whole slices at every tier");
        kac_close(&g);

        kac_init(&g);
        kac_set_tier(&g, 3);
        kac_init_params(&g);
        check(kac_set_stream(&g, 1) == 0, "stream opens on pinned T3");
        check_i((long)kac_get_chunk_samples(&g), kac_block_for_rate(44100),
                "a pinned T3 stream defaults to the file block");
        kac_close(&g);

        kac_init(&g);
        kac_set_tier(&g, 2);
        kac_init_params(&g);
        check(kac_set_stream(&g, 1) == 0, "stream opens on T2");
        check(kac_set_chunk_ms(&g, 1u) == 0, "T2 takes a 1 ms chunk");
        check_i((long)kac_get_chunk_samples(&g), 30, "T2 1 ms is one 30-sample slice");
        kac_close(&g);

        kac_init(&g);
        kac_set_tier(&g, 1);
        kac_init_params(&g);
        check(kac_set_stream(&g, 1) == 0, "stream opens on T1");
        check(kac_set_chunk_ms(&g, 1u) == 0, "T1 rounds up to its slice");
        check_i((long)kac_get_chunk_samples(&g), 60, "T1 at 1 ms is one 60-sample slice");
        kac_close(&g);

        kac_init(&g);
        kac_set_in_samplerate(&g, 8000);
        kac_set_tier(&g, 1);
        kac_init_params(&g);
        check(kac_set_stream(&g, 1) == 0, "stream opens on T1 at 8 kHz");
        check(kac_set_chunk_ms(&g, 1u) == KAC_E_ARG, "T1's slice does not fit 1.5 ms at 8 kHz");
        check_i((long)kac_get_chunk_samples(&g), KAC_CHUNK_LCM,
                "a refused chunk leaves the stream default");
        kac_close(&g);

        kac_init(&g);
        kac_set_tier(&g, 3);
        kac_init_params(&g);
        check(kac_set_stream(&g, 1) == 0, "stream opens on T3");
        check(kac_set_chunk_ms(&g, 1000u) == 0, "a large ms is taken");
        check_i((long)kac_get_chunk_samples(&g), KAC_CHUNK_SAMPLES,
                "and clamps to the largest chunk");
        check(kac_encode_buffer_interleaved(&g, pcm, 960u, buf, &got) == (int)got,
              "a chunked stream encodes");
        check(kac_set_chunk_ms(&g, 5u) == KAC_E_ARG, "chunk_ms is refused once bytes exist");
        kac_close(&g);

        check(kac_get_chunk_samples(NULL) == 0u, "no handle, no chunk size");
    }

    free(pcm);
    free(pcm1);
    printf("stream checks %d\n", kac_tu_cases);
    return kac_tu_report("test_stream", "checks");
}
