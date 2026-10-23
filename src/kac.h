/* kac.h -- public encoder and decoder interface. Handles are caller-owned and
 * the library never allocates. Encode calls omit the leading preamble;
 * kac_write_preamble() returns it to patch once lengths are known. */
#ifndef KAC_H
#define KAC_H

#include <stddef.h>
#include <stdint.h>
#include "kac_internal.h"

/* Channel form: as it arrived, or mid/side. AUTO measures each block. Forcing a
 * form skips the measurement. */
#define KAC_MS_AUTO 0
#define KAC_MS_LR 1
#define KAC_MS_MS 2

#define KAC_TIER_AUTO 0

/* Default configuration: rung 5 of the -V ladder. Pinning a tier or calling
 * kac_set_vbr() replaces it. */
#define KAC_DEFAULT_V 5

/* VBR modes. Both choose a tier per block: V targets a level, ABR steers the
 * level so the achieved rate converges. */
#define KAC_VBR_OFF 0
#define KAC_VBR_ABR 1 /* target bit rate in bit/sample, quality floats */
#define KAC_VBR_V 2   /* quality target -V n, bit rate floats; target = index n */

typedef struct {
    /* per-sample and per-slice state */
    kac_ch         ch[KAC_MAX_CHANNELS];
    int16_t        sbuf[KAC_MAX_CHANNELS][KAC_CHUNK_SAMPLES];
    unsigned       w, n;
    const int16_t *dqt;
    int32_t        rowmax[KAC_TIER_MAX][16];
    int            cur_tier;
    int            channels;
    unsigned       fill;
    uint32_t       pos, total_in;
    int            state;

    /* what searches this handle runs */
    int      quality;
    unsigned beam_w;
    unsigned probe_cand;
    int      ms_mode;
    int      est_measure;
    int      form;     /* form-measurement cost, KAC_FORM_* */
    unsigned beam_lag; /* KAC_LAG_*: how often the beam collapses */
    int      plain;    /* no search at all: one row, nearest level */
    int32_t  measure_fix;
    int      report; /* collect clip and limit counters */

    /* per-block control and readings */
    int      vbr_mode;
    int      vbr_n;
    int32_t  vbr_target, vbr_psnr;
    int      abr_seeded; /* the first usable block set vbr_psnr */
    int      tier_lo, tier_hi;
    int32_t  tier_psnr[KAC_TIER_MAX + 1];
    int32_t  tier_psnr_mid[KAC_TIER_MAX + 1];
    int32_t  tier_band_lo[KAC_TIER_MAX + 1];
    int32_t  tier_band_hi[KAC_TIER_MAX + 1];
    int32_t  tier_low[KAC_TIER_MAX + 1];
    int32_t  tier_high[KAC_TIER_MAX + 1];
    unsigned chunk_samples;
    uint64_t data_bytes;
    uint32_t blk_count;
    int      blk0_ms;
    int      prefix_done;

    /* running statistics */
    int32_t  peak;
    int64_t  sse;
    uint32_t clip;
    unsigned brute_level;
    unsigned neighbor_pct;
    uint64_t pred_over, w_limit;

    /* snapshots */
    kac_snap loop_snap[KAC_MAX_CHANNELS];
    int      loop_snap_valid;
    kac_snap init_snap[KAC_MAX_CHANNELS];
    int      init_snap_valid;

    /* configuration, valid before kac_init_params() */
    int       samplerate;
    double    brate;
    int       tier;
    uint32_t  loop_start, loop_end;
    uint32_t  stride;
    uint8_t  *index_store;
    size_t    index_store_bytes;
    uint32_t *index_off;
    size_t    index_off_entries;
    int       stream;

    /* staging bytes for the coded form; last because probe_copy() skips it */
    uint8_t alt_out[KAC_BLOCK_BYTES];
} kac_t;

typedef struct {
    /* per-sample state */
    kac_ch         ch[KAC_MAX_CHANNELS];
    const uint8_t *data;
    const int16_t *dqt;
    unsigned       w, n, shift;
    uint32_t       pos;
    uint32_t       total;
    uint32_t       loop_end;
    int            loop_en;
    uint8_t        channels;

    /* current block: tier byte, samples covered, byte base and form */
    uint32_t blk_start, blk_len, blk_off;
    int      ms;
    uint8_t  tier;

    /* header and stream state */
    uint32_t       sample_rate;
    uint32_t       loop_start;
    uint32_t       data_bytes;
    uint32_t       index_stride, index_entries;
    uint8_t        version, flags;
    const uint8_t *init_snap;
    const uint8_t *loop_snap;
    const uint8_t *idx_off;
    const uint8_t *idx_snap;
    uint32_t       loop_blk_start, loop_blk_off;
    int            stream;
} kacdec_t;

void kac_init(kac_t *restrict g);
void kac_close(kac_t *restrict g);

int kac_set_num_channels(kac_t *restrict g, int channels);
int kac_set_in_samplerate(kac_t *restrict g, int hz);
int kac_set_brate(kac_t *restrict g, double bits_per_sample);
/* Pin the tier for every block, 1..KAC_TIER_MAX. CLI --tier uses T2..T6; T1
 * and T7 are reachable for harnesses and low-delay streams. */
int kac_set_tier(kac_t *restrict g, int tier);
/* Restrict the per-block tier window, inclusive. A one-tier window behaves like
 * a pin but still reports a VBR mode. Any VBR/tier/rate selector resets it. */
int kac_set_tier_limits(kac_t *restrict g, int lo, int hi);
int kac_set_loop(kac_t *restrict g, uint32_t start, uint32_t end);

/* Search effort, 0..KAC_QUALITY_MAX. Level 0 is hardest; higher levels drop
 * searches and estimate tiers from one approximate pass. */
int kac_set_quality(kac_t *restrict g, int level);

/* Collect the clip, prediction-overflow and weight-limit counters (on by
 * default). Turning them off skips their per-sample bookkeeping; the PSNR and
 * achieved-rate reports stay exact, and the counters read zero. */
int kac_set_report(kac_t *restrict g, int enable);

/* Override the quality level with fixed harder settings (1 and 2); 0 is off.
 * When set, -q controls none of the search. */
int kac_set_brute(kac_t *restrict g, unsigned level);

/* Channel form: LR forces left/right, MS forces mid/side, AUTO measures and
 * requires the margin in kac_ms_margin_mdb[tier]. Mono has only one form. */
int      kac_set_ms_mode(kac_t *restrict g, int mode);
unsigned kac_get_quality(const kac_t *restrict g);

/* Use this LMS state as INIT_STATE. Without it, decoders use the normative
 * default. The array is copied and need not outlive the call. */
int kac_set_init_state(kac_t *restrict g, const kac_snap *snaps);

/* Current encoder state; passing it to another handle lets that file continue
 * this stream at the same tier and phase. */
int kac_get_state(const kac_t *restrict g, kac_snap *snaps);
int kac_set_index(kac_t *restrict g, uint32_t stride, uint8_t *store, size_t bytes);

/* Switch to stream output: encode calls emit blocks without the leading
 * placeholder. Refused after any emission, with a loop region, or with a seek
 * index. The default chunk is the file grid capped at 20.5 ms. */
int kac_set_stream(kac_t *restrict g, int enable);

/* Target stream chunk length in ms. A pinned tier uses whole slices; a
 * per-block run uses multiples of KAC_CHUNK_LCM. The count is the nearest
 * quantum, at most 0.5 ms over the target and no longer than the file grid. */
int kac_set_chunk_ms(kac_t *restrict g, unsigned ms);

/* Chunk length in samples, for framing feeds. */
unsigned kac_get_chunk_samples(const kac_t *restrict g);

/* Stream preamble, sent once before block 0. Must be called after block 0 is
 * coded because it carries block 0's tier and form. */
int kac_write_stream_preamble(const kac_t *restrict g, uint8_t *restrict buf, size_t *restrict len);

/* Set per-block VBR. ABR target is bit/sample; V target is the index 0..9.999. */
int kac_set_vbr(kac_t *restrict g, int mode, double target);

/* Record seek-index offsets. Byte offsets depend on the tier sequence and
 * cannot be derived; `entries` matches kac_set_index(). */
int kac_set_index_offsets(kac_t *restrict g, uint32_t *store, size_t entries);

/* Integer rung n target in dB and its nominal tier; n is clamped to 0..9. */
double   kac_v_tier_psnr(int n);
unsigned kac_v_nominal_tier(int n);

/* Set a PSNR target in dB, including values between rungs, up to 85 dB. At a
 * rung's exact target this produces the same file as kac_set_vbr(). */
int kac_set_vbr_target_psnr(kac_t *restrict g, double target_psnr);

/* Validate the configuration and fix the tier. Returns 0 or negative. */
int kac_init_params(kac_t *restrict g);

/* Highest pinnable tier whose rate is at most the target; clamps to T2..T6. */
int kac_tier_for_brate(double bits_per_sample);

/* Lowest pinnable tier whose rate is at least the target; clamps to T2..T6. */
int kac_tier_ceil_brate(double bits_per_sample);

/* The same lookups over T1..T7, for a per-block window. */
int kac_tier_for_brate_vbr(double bits_per_sample);
int kac_tier_ceil_brate_vbr(double bits_per_sample);

/* Upper bound for one encode call or a flush covering `frames`. Pass the whole
 * file to cover the seek index. The bound includes the measurement scratch
 * space, which can exceed the emitted bytes, so do not reduce it. */
size_t kac_encode_bufsize(const kac_t *restrict g, uint32_t frames);

/* Encode frames frames of interleaved PCM. There is one entry point. */
int kac_encode_buffer_interleaved(kac_t *restrict g, const int16_t *restrict pcm, uint32_t frames,
                                  uint8_t *restrict out, size_t *restrict outlen);

/* Emit the trailing partial slice and, when a seek index was requested, the
 * index section. Must be called once after the last frame. */
int kac_encode_flush(kac_t *restrict g, uint8_t *restrict out, size_t *restrict outlen);

/* Leading bytes of the file: 36 byte header, plus one snapshot per channel
 * when looping. Valid after kac_encode_flush(). */
int kac_write_preamble(const kac_t *restrict g, uint8_t *restrict buf, size_t *restrict len);

unsigned kac_get_tier(const kac_t *restrict g);
uint32_t kac_get_total(const kac_t *restrict g);
double   kac_get_brate(const kac_t *restrict g);
/* Report level in dB; the encoder computes it in milli-dB and divides once. */
double      kac_get_psnr(const kac_t *restrict g);
double      kac_get_clip_rate(const kac_t *restrict g);
double      kac_get_pred_over_rate(const kac_t *restrict g);
double      kac_get_w_limit_rate(const kac_t *restrict g);
const char *kac_version(void);

/* The two VBR report items. */
/* Achieved bit rate of the emitted data area: 8 * DataBytes / sample. For a run
 * pinned to one tier that is 64/N plus the tier byte, up to the final partial
 * block. */
double kac_get_brate_achieved(const kac_t *restrict g);
/* Signed deviation from the nominal point in the mode's own unit; 0 when off. */
double   kac_get_vbr_offset(const kac_t *restrict g);
int      kac_get_vbr_mode(const kac_t *restrict g);
double   kac_get_vbr_target(const kac_t *restrict g);
unsigned kac_get_vbr_nominal_tier(const kac_t *restrict g);
uint32_t kac_get_data_bytes(const kac_t *restrict g);

void kacdec_init(kacdec_t *restrict d);

/* Open a complete file image; a failed open leaves the handle inert. */
int kacdec_open(kacdec_t *restrict d, const uint8_t *restrict buf, size_t len);

/* Open a stream preamble and load its INIT_STATE. SEEK and LOOP are refused. */
int kacdec_stream_open(kacdec_t *restrict d, const uint8_t *restrict head, size_t len);

/* Supply the next chunk (tier byte plus slices). The buffer is borrowed until
 * the next feed; feeding before the previous chunk is consumed is refused. */
int kacdec_stream_feed(kacdec_t *restrict d, const uint8_t *restrict blk, size_t len);

/* Decode up to `frames`; returns frames produced, 0 at end. Loop mode wraps at
 * LoopEnd and reports KAC_E_NOLOOP if the file has no loop region. A stream
 * returns 0 when the fed chunks are exhausted. */
int32_t kacdec_read(kacdec_t *restrict d, int16_t *restrict out, uint32_t frames);

/* Enable loop wrapping; a source without a loop region reports KAC_E_NOLOOP
 * from kacdec_read(). */
void kacdec_set_loop(kacdec_t *restrict d, int enable);
int  kacdec_seek(kacdec_t *restrict d, uint32_t sample);
void kacdec_close(kacdec_t *restrict d);

uint32_t kacdec_sample_rate(const kacdec_t *d);
uint32_t kacdec_total(const kacdec_t *d);
unsigned kacdec_channels(const kacdec_t *d);
unsigned kacdec_tier(const kacdec_t *d);

#endif /* KAC_H */
