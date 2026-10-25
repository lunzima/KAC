/* wav_io.h -- minimal WAV reader and writer (16 bit PCM, mono or stereo). */
#ifndef KAC_WAV_IO_H
#define KAC_WAV_IO_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#if defined(__GNUC__)
#define KAC_WAV_UNUSED __attribute__((unused))
#else
#define KAC_WAV_UNUSED
#endif

typedef struct {
    int16_t *pcm; /* interleaved */
    uint32_t frames;
    unsigned channels;
    uint32_t rate;
} kac_wav;

static KAC_WAV_UNUSED uint16_t wav_g16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | (uint16_t)((uint16_t)p[1] << 8));
}

static KAC_WAV_UNUSED uint32_t wav_g32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static KAC_WAV_UNUSED void wav_p16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static KAC_WAV_UNUSED void wav_p32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static KAC_WAV_UNUSED void wav_free(kac_wav *w)
{
    if (w) {
        free(w->pcm);
        w->pcm = NULL;
    }
}

/* Returns 0, or -1 for anything that is not 16 bit PCM in a RIFF/WAVE file. */
static KAC_WAV_UNUSED int wav_read(const char *path, kac_wav *w)
{
    FILE    *f;
    long     sz;
    uint8_t *b;
    uint32_t q, dlen = 0, rate = 0;
    uint8_t *data = NULL;
    unsigned ch   = 0;
    int      bits = 0, tag = 0;

    memset(w, 0, sizeof *w);
    f = fopen(path, "rb");
    if (!f)
        return -1;
    if (fseek(f, 0, SEEK_END)) {
        fclose(f);
        return -1;
    }
    sz = ftell(f);
    if (sz < 44 || fseek(f, 0, SEEK_SET)) {
        fclose(f);
        return -1;
    }
    b = (uint8_t *)malloc((size_t)sz);
    if (!b) {
        fclose(f);
        return -1;
    }
    if (fread(b, 1, (size_t)sz, f) != (size_t)sz) {
        free(b);
        fclose(f);
        return -1;
    }
    fclose(f);
    if (memcmp(b, "RIFF", 4) || memcmp(b + 8, "WAVE", 4)) {
        free(b);
        return -1;
    }
    for (q = 12; q + 8 <= (uint32_t)sz;) {
        uint32_t csz  = wav_g32(b + q + 4);
        uint8_t *body = b + q + 8;
        if ((uint64_t)q + 8 + csz > (uint32_t)sz)
            break;
        if (!memcmp(b + q, "fmt ", 4) && csz >= 16) {
            tag  = wav_g16(body);
            ch   = wav_g16(body + 2);
            rate = wav_g32(body + 4);
            bits = wav_g16(body + 14);
        } else if (!memcmp(b + q, "data", 4)) {
            data = body;
            dlen = csz;
        }
        q += 8 + csz + (csz & 1u);
    }
    if (!data || tag != 1 || bits != 16 || ch < 1 || ch > 2) {
        free(b);
        return -1;
    }
    w->frames   = dlen / (2u * ch);
    w->channels = ch;
    w->rate     = rate;
    w->pcm      = (int16_t *)malloc((size_t)w->frames * ch * sizeof(int16_t));
    if (!w->pcm) {
        free(b);
        return -1;
    }
    {
        uint32_t i, n = w->frames * ch;
        for (i = 0; i < n; i++)
            w->pcm[i] = (int16_t)wav_g16(data + 2 * i);
    }
    free(b);
    return 0;
}

static KAC_WAV_UNUSED int wav_write_header(FILE *f, uint32_t frames, unsigned ch, uint32_t rate)
{
    uint8_t  h[44];
    uint32_t dbytes = frames * ch * 2u;

    memcpy(h, "RIFF", 4);
    wav_p32(h + 4, 36u + dbytes);
    memcpy(h + 8, "WAVE", 4);
    memcpy(h + 12, "fmt ", 4);
    wav_p32(h + 16, 16);
    wav_p16(h + 20, 1);
    wav_p16(h + 22, (uint16_t)ch);
    wav_p32(h + 24, rate);
    wav_p32(h + 28, rate * ch * 2u);
    wav_p16(h + 32, (uint16_t)(ch * 2));
    wav_p16(h + 34, 16);
    memcpy(h + 36, "data", 4);
    wav_p32(h + 40, dbytes);
    return (fwrite(h, 1, 44, f) == 44) ? 0 : -1;
}

static KAC_WAV_UNUSED int wav_write_frames(FILE *f, const int16_t *pcm, uint32_t frames,
                                           unsigned ch)
{
    uint32_t i, n = frames * ch;
    uint8_t  buf[512];
    uint32_t k = 0;

    for (i = 0; i < n; i++) {
        wav_p16(buf + 2 * k, (uint16_t)pcm[i]);
        k++;
        if (k == 256u || i + 1u == n) {
            if (fwrite(buf, 2, k, f) != k)
                return -1;
            k = 0;
        }
    }
    return 0;
}

#endif /* KAC_WAV_IO_H */
