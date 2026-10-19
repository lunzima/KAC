# KAC Design Specification

Time-domain lossy audio format for wavetables and embedded playback.

## 1. Overview

KAC compresses audio in the time domain with a 4th-order LMS predictor and scalar residual
quantization. The decoder is a straight-line loop. The format supports arbitrary sample rates,
seamless looping, and gapless playback.

### 1.1 Conventions

The key words MUST, MUST NOT, SHOULD, and MAY are to be interpreted as described in RFC 2119.
"Reject a file" means that the decoder MUST treat it as invalid and MUST NOT output samples
from it. "Reject a seek" means that the seek fails and the open file stays usable.

All multi-byte fields are little-endian (Section 6.1). A slice is the 8-byte coded unit for one
channel. `N` is the number of samples in a slice and `w` is the residual width; both come from
the tier (Section 5.1).

In a file, a chunk holds `B(rate)` samples. Let `t = floor(rate * 20 / 1000)`. `B` is `t`
rounded to the nearest multiple of 120 (a half rounds up), then reduced by steps of 120 until
it is at most `t + floor(rate / 2000)`; the result is clamped to 120..960. So it is about
20 ms and never more than 20.5 ms; below about 5.9 kHz, where one 120-sample block is already
longer than 20.5 ms, `B` is 120. 120 is the least count that is a whole number of slices at
every tier, so a block boundary is always a slice boundary. A stream's chunk length is stated
in its header instead (Section 3.5).

| Property | Requirement |
|---|---|
| Decoder complexity | Fixed by the format: no floating point, no malloc, no variable-width reads. One fixed-width sliding-window read per residual (Section 3.2, Section 4.6) |
| Decoder state | LMS state 64 B (`ch[2]` = 2 * (4 int32 weights + 4 int32 history)) plus about 16 B of control fields = about 80 B (Section 4.1) |
| Bitrate | 7 tiers: 1.067 / 2.133 / 3.200 / 4.267 / 5.333 / 6.400 / 8.000 bit/channel*sample (Section 5.1) |
| Stream form | The same payload with no file length; the header states the chunk length, so a receiver needs no out-of-band framing (Section 3.5) |
| Algorithmic delay | 0 samples (purely causal, no lookahead), which enables gapless playback |
| Channels | 1 or 2 |
| Output | 16-bit PCM, interleaved |
| Sample rate | Any value. The rate is stored in the header; the decoder does not resample |

## 2. Hard constraints

### 2.1 The constraints

1. The sample rate is arbitrary. The file stores the rate, and the decoder outputs at that rate.
   The prediction gain of a 4-tap LMS depends on the rate, so the tiers of Section 5.1 are
   calibrated at 44.1 kHz. Another rate needs no format change but changes the quality a tier
   delivers.
2. The channel count is 1 or 2.
3. The output is 16-bit PCM.
4. Gapless playback is mandatory (Section 4.5).

## 3. Bitstream format

### 3.1 File header (fixed 36 bytes)

| Offset | Size | Field | Value |
|---:|---:|---|---|
| 0 | 4 | `Magic` | `4B 41 43 00` |
| 4 | 1 | `Version` | 1 |
| 5 | 1 | `Flags` | `LOOP`, `INIT_STATE`, `SEEK_INDEX`, `MS` in bits 0-3; bits 4-7 reserved |
| 6 | 1 | `Channels` | 1 or 2 |
| 7 | 1 | `Tier` | Chunk 0's tier in bits 0-2 and form in bit 3; bits 4-7 zero |
| 8 | 4 | `SampleRate` | Any u32 |
| 12 | 4 | `TotalSamples` | Exact sample count per channel, at most 2^28 |
| 16 | 4 | `LoopStart` | Loop start sample offset, when `Flags.LOOP=1` |
| 20 | 4 | `LoopEnd` | Loop end sample offset, exclusive, when `Flags.LOOP=1` |
| 24 | 4 | `DataBytes` | Size of the data area, every tier byte included |
| 28 | 4 | `IndexStride` | In a file, the seek index spacing; in a stream, the samples per chunk |
| 32 | 4 | `IndexEntries` | Number of seek index entries |

Field rules:

* `Magic` MUST be `4B 41 43 00` (`"KAC"` and a zero byte). Reject otherwise. `Version` MUST be 1.
* `Flags`: a set reserved bit MUST be rejected, not ignored. `MS=1` declares that a chunk may
  carry mid/side (Section 3.2). A decoder that does not implement that form MUST reject the file;
  it MUST NOT read the two coded channels as left and right.
* `Channels` MUST be 1 or 2. Any other value is rejected.
* `Tier`: bits 0-2 name chunk 0's tier (1-7). Any other value is rejected; it MUST NOT be
  ignored or replaced by a default. Bit 3 is chunk 0's form (0 = left/right, 1 = mid/side), and
  bits 4-7 MUST be zero. Bit 3 set with `Flags.MS=0` contradicts the header and MUST be
  rejected, like a reserved bit. The field MUST equal chunk 0's tier byte in both tier and
  form; a mismatch MUST be rejected.
* `SampleRate` may be any u32. Zero is legal and is not rejected; the decoder stores it and
  never resamples.
* `TotalSamples` MUST be at most 2^28 = 268 435 456; a larger value MUST be rejected
  (Section 4.4). The bound keeps `DataBytes`, the index, and the file length in signed 32-bit
  (Section 6.3). `TotalSamples = 0` is handled in Section 4.4.
* `DataBytes` covers the data area including every chunk's tier byte. No closed form gives it,
  because `N` varies per chunk. The decoder MUST walk the chunks: read each tier byte, take its
  `N`, and require (a) the sample counts to sum to `TotalSamples`, with the last chunk taking
  the remainder, (b) the bytes consumed to equal `DataBytes`, and (c) the data area not to run
  past the end of the file. A truncated file MUST be rejected; the decoder MUST NOT output the
  bytes that were read.
* `IndexStride`: with `SEEK_INDEX=1` it MUST be a positive multiple of the file block, because
  an index point must land on a chunk boundary and a chunk head is a slice head. With
  `SEEK_INDEX=0` it MUST be 0. A violation rejects the file.
* `IndexEntries`: with `SEEK_INDEX=1` it MUST equal `ceil(TotalSamples / IndexStride)`; reject
  otherwise (Section 3.4). With `SEEK_INDEX=0` it MUST be 0.

The rest of the layout is determined by `Flags`:

```
[init state snapshot]  Flags.INIT_STATE=1  ->  Channels * 16 bytes, at sample 0
[loop state snapshot]  Flags.LOOP=1        ->  Channels * 16 bytes, at sample LoopStart
[data area]            DataBytes bytes
[seek index]           Flags.SEEK_INDEX=1  ->  see Section 3.4
```

Illegal combinations MUST be rejected:

| Combination | Reason |
|---|---|
| `INIT_STATE=1` or `LOOP=1`, but the file cannot hold `Channels * 16` bytes at that position (past EOF or past the declared start of `DataBytes`) | The snapshot would be partial. The decoder MUST NOT read a partial snapshot and MUST NOT fall back to the default state |
| `LOOP=0` with a `LOOP_STATE` snapshot | No such file exists: the layout is fully determined by `Flags`. The decoder MUST NOT guess or infer a snapshot that `Flags` does not declare. Reading the first data byte as history is an implementation error |
| `LOOP=1` with out-of-range or empty `LoopStart`/`LoopEnd` | See Section 4.4 |
| `SEEK_INDEX=1` with `IndexStride=0`, or a file with `SEEK_INDEX=0` and a non-zero `IndexStride` or `IndexEntries` | See the `IndexStride` and `IndexEntries` rows above |
| A set reserved `Flags` bit | See the `Flags` row above |

### 3.2 Data area: chunks and slices

The data area is a sequence of chunks. Each chunk starts with a tier byte:

```
[tier byte][chunk 0 slices][tier byte][chunk 1 slices] ...
```

Chunk `m` covers samples `[B*m, B*(m + 1))`, where `B = B(rate)` (Section 1.1). The last chunk
takes the remainder, so the chunk lengths sum to `TotalSamples`. This grid belongs to the file
form. In a stream, the sender sets the chunk length and states it in the header; a chunk carries
1 to `960 / N` slices (Section 3.5).

The tier byte holds the tier in bits 0-2 (1 to 7; 0 or greater than 7 is rejected, with the same
no-ignore rule as the header `Tier` field), the chunk's channel form in bit 3, and zero in bits
4-7. The header `Tier` field equals chunk 0's tier byte in both tier and form.

Inside a chunk, slices are 8 bytes each and interleaved as
`ch0[slice0] ch1[slice0] ch0[slice1] ch1[slice1] ...`; a mono file has ch0 only. `N` comes from
the chunk's tier byte. One slice encodes `N` consecutive samples of one channel.

Bit layout (written into a `uint64` in big-endian order and stored little-endian in the file):

```
bit 63..60 : scalefactor (4 bit)
bit 59.. 0 : N residual indices, w bits each, MSB first
             4 + N*w <= 64; the bits below the last index are padding and MUST be zero
```

Every tier satisfies `4 + N*w <= 64`. Six tiers fill a slice exactly; T7 (w = 7, N = 8) leaves 4
padding bits. The next tier would cost more than 8 bit per sample, the rate of plain 8 bit PCM,
so the ladder stops at T7. A decoder MUST reject a slice whose padding is non-zero. The rule is
the same as for a reserved bit: a byte string must decode to exactly one sample string, and the
decoder MUST NOT ignore what is not payload.

Decoding runs once per slice:

```c
uint64_t v = kac_read_u64_le(p);             /* memcpy plus explicit byte order */
int sf = (int)(v >> 60);  v <<= 4;           /* 4-bit scalefactor */
const int32_t *dq = KAC_DQ + (sf << w);      /* rows are per sf, row width 1<<w */
bad |= v & (((uint64_t)1 << (60 - N*w)) - 1); /* padding; zero when 4+N*w == 64 */
for (i = 0; i < N; i++) {
    int qr = (int)(v >> shift);  v <<= w;    /* shift = 64 - w, constant per tier */
    out[i] = (int16_t)kac_step(&dec->ch[c], dq[qr]);   /* Section 4.2 */
}
if (bad) reject;
```

There is no bit carry, no variable-width read, and no second bitstream. Each slice needs one
64-bit load.

The file block is a multiple of every tier's `N` (at 44.1 kHz, B = 840: 840/60 = 14, 840/30 = 28,
840/20 = 42, 840/15 = 56, 840/12 = 70, 840/10 = 84, 840/8 = 105), so a chunk head is always a
slice head. One tier byte per chunk costs 8/(B*channels) bit/sample.

Locating a sample position. `N` varies per chunk, so a slice index cannot be derived from a
sample position; the closed form `slice_idx = pos / N` does not hold. To reach an arbitrary
position, such as an unindexed seek or the first entry into a loop at `LoopStart`, walk the
chunks from the start of the data area. Read one tier byte, take its `N`, advance past that
chunk by `ceil(L / N) * 8 * Channels` bytes, where
`L = min(B, TotalSamples - samples consumed so far)`, and repeat until the running sample
count covers the target. `B/N` is exact for a full chunk but not for the last one, so the
general form uses `ceil`. The walk reads tier bytes only and decodes no samples. A cached
absolute position is not a running bit pointer: Section 4.4 still forbids reusing the latter
across loop periods.

Coded channels (`Flags.MS=1` and a chunk's tier byte bit 3 set). Such a chunk carries mid and
side instead of left and right:

```
mid  = (L + R) >> 1          L = clamp16(mid + side)
side = (L - R) >> 1          R = clamp16(mid - side)
```

Both shifts are arithmetic and floor. Halves instead of a sum and a difference keep the pair
inside 16 bits: a full-scale sum or difference needs 17. `mid` carries the shared level and
`side` the difference, so nearly equal channels leave `side` small and its error small.

The form is per chunk and independent of the tier, so one file may carry both. A decoder MUST
read each chunk's bit before decoding it and MUST NOT carry the form into the next chunk.
Merging is `int16` arithmetic and adds no state; the output is the pair added and subtracted,
clamped to 16 bits.

The state a coded chunk leaves is the mid and side filters' state (Sections 3.3 and 3.4). A
snapshot taken inside a coded chunk holds that state and is restored into it; a decoder never
converts a snapshot between forms.

Halving rounds, so a pair returns within one count of the input pair. A file may declare `MS=1`
without setting any chunk's bit; its output equals `MS=0`. With `Channels=1` there is no pair,
so no chunk may set the bit.

### 3.3 LMS state snapshot (16 bytes per channel)

| Size | Field |
|---:|---|
| 8 | `history[4]`, one int16 each (little-endian) |
| 8 | `weights[4]`, one int16 each (little-endian), range `[-16383, 16383]` |

History samples are already `kac_clamp16` output, so int16 is lossless. Weights fit in int16
because the decoder saturates them to `[-16383, 16383]` on every step (Section 4.2), so an int16
field represents a weight at any instant exactly, and restore is bit-exact. An encoder that runs
the same `kac_step` agrees weight for weight, so the snapshot is exact by construction.

This is the normative default initial state, mandatory when `Flags.INIT_STATE = 0`:

```c
history[0..3] = 0;
weights[0] = 0;
weights[1] = 0;
weights[2] = -8192;                      /* linear extrapolator: pred ~= 2*h[3] - h[2] */
weights[3] =  16383;                     /* not 16384 */
```

* This is a linear extrapolation predictor, so predictions work from the first sample. An
  all-zero state is no slight penalty: `pred == 0` gives full-scale error immediately and can
  drive the weights, through the blindness mechanism of Section 4.2, into locking the output at
  full scale. The default is fixed; an implementer MUST NOT choose another.
* The upper bound is 16383, not 16384: `4 * 16384 * 32768 = 2^31`, while
  `INT32_MAX = 2^31 - 1`, so the worst case overflows by exactly 1. The +/-16383 headroom is
  131 071.
* The bound does not rely on the asymmetry of the negative end of `kac_clamp16`.

The decoder MUST apply `kac_sat_w` once to each of the four weights after reading any
`INIT_STATE`, `LOOP_STATE`, or `SEEK` snapshot. This is a correctness requirement: the
no-overflow proof of Section 4.2 relies on `|w| <= 16383`, and a malformed snapshot can carry
`w = 32767`. The `history` fields are used as-is; their magnitude is at most 32768.

Snapshots appear in three places, with the same semantics:

* the initial state at sample 0 (`Flags.INIT_STATE`), which lets a file start from an already
  converged state;
* the loop state at sample `LoopStart` (`Flags.LOOP`), the sole basis for seamless looping;
* the index state at sample `k * IndexStride` (`Flags.SEEK_INDEX`), for an exact seek.

Sample-point semantics (frozen): all three snapshots record the state at one specific sample
point, not at the start of the slice that contains that point. Section 4.4 relies on this to
support loop points at arbitrary offsets. Fields are in memory order: `history[0..3]` runs
oldest to newest (`history[3]` is the most recent), and `weights[0..3]` match the memory layout,
so the decoder reads them in byte order with no reordering.

### 3.4 Seek index (optional)

```
[u32 offset table]  IndexEntries entries; entry k = byte offset of the ch0 slice at index k,
                    the first slice of the chunk that holds sample k * IndexStride
                    (stereo is frozen: ch1 offset = entry + 8; the table stays IndexEntries
                     entries and is not expanded per channel)
[snapshot array]    IndexEntries * Channels * 16 bytes
```

Entry k corresponds to sample `k * IndexStride`. `IndexStride` is a multiple of the file block,
and the block is a multiple of every tier's `N`, so every index point falls on a chunk boundary
and a slice
boundary. The slice the table points at is a chunk's first slice, and its `N` and channel form
come from the tier byte immediately before it (Section 3.2). A byte offset from the table can
therefore be used directly, and the decoder never needs an offset within a slice.

The table is checked lazily: an entry is checked when a seek uses it, and only that entry, so a
file with a malformed entry elsewhere still opens. For entry `k` with value `e`, a seek is
rejected unless all three conditions hold:

* `1 <= e < DataBytes`. The byte at `e - 1` is the chunk's tier byte, so it must lie inside the
  data area and cannot be its first byte.
* The byte at `e - 1` passes the tier-byte rule of Section 3.2: tier 1-7, bits 4-7 zero, and bit
  3 set only when `Flags.MS=1` and `Channels=2`. It is not held to the header `Tier` field; that
  equality belongs to chunk 0 alone.
* The chunk fits inside the data area: `e + ceil(L / n) * 8 * Channels <= DataBytes`, where `n`
  is the `N` of that tier byte and `L = min(B, TotalSamples - k * IndexStride)` is the chunk's
  declared length.

In a well-formed file, an entry points at the true offset. A decoder is not required to verify
that: proving it means walking the whole chunk chain, while the three checks above are constant
work. A malformed table belongs with a corrupted slice -- it yields wrong samples and goes
unreported, because the data area has no sync word and the decoder neither detects nor recovers.

To seek to sample `S` when `Flags.SEEK_INDEX=1`:

```
Preconditions (checked first):
  SEEK_INDEX=1 and IndexStride=0                    -> reject (Section 3.1)
  IndexStride not congruent 0 (mod B)               -> reject (Section 3.1)
  IndexEntries != ceil(TotalSamples / IndexStride)  -> reject (Section 3.1)
  S >= TotalSamples                                 -> reject

k    = S / IndexStride                    floor division, once per seek
       S < TotalSamples and IndexEntries = ceil(TotalSamples/IndexStride), so
       k <= (TotalSamples-1)/IndexStride = IndexEntries - 1.
       The implementation MUST still assert k < IndexEntries before indexing [k],
       so a bypassed precondition cannot read past either array.
       Assertion failure -> reject that seek; never read out of bounds.
pos  = k * IndexStride
e    = [u32 offset table][k]              byte offset of the chunk's first ch0 slice
       e < 1 or e >= DataBytes                              -> reject
       the byte at e - 1 is not a legal tier byte           -> reject
       e + ceil(L/n) * 8 * Channels > DataBytes             -> reject
           L = min(B, TotalSamples - pos), n = that tier byte's N
jump to data byte e, load [snapshot array][k] (the state at pos),
       then apply kac_sat_w to each of the 4 weights (Section 3.3)
discard the next (S - pos) samples, then output normally from S
```

With `SEEK_INDEX=0` the file has no index, and seek is implementation-defined; normally it means
forward decoding from sample 0 only.

`IndexStride` SHOULD be a large multiple of the file block, such as 6 * B, for the same reason
the block is `B` (Section 3.2): it is a chunk boundary and a slice boundary in every tier.

An offset-table entry holds a byte offset into the data area. The whole file is bounded by
`TotalSamples <= 2^28` (Section 3.1), so every such offset is below `2^31` and a u32 field is
sufficient with margin.

The offset table is a required part of the format, not a redundant copy of the slice grid. The
byte offset of index `k` depends on the tiers of every chunk before it, which the length fields
of Section 3.1 do not give, so the decoder cannot recompute an entry from `k`. This variable
offset does not make slices variable-length: every slice is 8 bytes, and the tier decides only
the number of slices per chunk.

> The data area has no per-frame state header: the LMS state evolves continuously, and
> retransmitting it would waste 32 bytes per frame and shorten the prediction history. The price
> is no error isolation. One corrupted byte desynchronizes everything after it, and the decoder
> neither detects nor recovers, there being no sync word.
>
> The index supplies a resume point: a chunk's byte offset plus the state there. A caller can
> restart at a slice boundary once it knows that state. Isolation is thus a property of the index
> density and of the caller's decision to jump, not of the decoder. For offline-packed wavetable
> resources, this trade is acceptable.

### 3.5 Stream form

A stream carries the same payload as a file, sent without a file length; it states the length of
its chunks instead, so a receiver can frame it from the bytes. A stream is not a file. The rules
that require `TotalSamples` to be a non-zero exact count (Sections 3.1 and 4.4) belong to the file
form and do not apply to a stream. A decoder that implements only the file form has no obligation
to accept a stream. `TotalSamples = 0` MUST still be rejected as a file.

```
[header]                the 36 bytes of Section 3.1
[init state snapshot]   Flags.INIT_STATE=1 -> Channels * 16 bytes
[chunk 0] [chunk 1] ... the data area, chunk by chunk, with no index and no loop snapshot
```

| Item | Rule |
|---|---|
| `TotalSamples`, `DataBytes`, `IndexEntries` | Zero. The header states no file length; the receiver derives the sample count from the chunks |
| `IndexStride` | Samples in each chunk except the last. It MUST be 1..960 and MUST be a multiple of the `N` of every chunk it frames |
| `Flags.SEEK_INDEX` | MUST be 0. The index trails the data, and a stream has no end in which to receive it. A receiver MUST reject a stream preamble that sets it |
| `Flags.LOOP` | MUST be 0. Wrapping needs a chunk the receiver has already returned, so a receiver MUST reject a stream preamble that sets it |
| `Flags.MS` | 0 or 1. A sender sets it when its chunks may carry the coded form. The receiver reads each chunk's own bit, so the preamble declares what may follow, not what does |
| Header `Tier` | The first chunk's tier, as in Section 3.1 |
| Chunk framing | A chunk carries `IndexStride / N` slices, where `N` comes from its tier byte. The last chunk takes the remainder: 1 to `IndexStride` samples in whole slices |

Framing. The header states the chunk length, so a receiver finds every boundary in a byte
stream. For each chunk: read the tier byte, take its `N`, reject the chunk unless
`IndexStride % N == 0`, then read `1 + (IndexStride / N) * 8 * Channels` bytes (the tier byte
plus the slices). Every chunk except the last holds exactly `IndexStride` samples; the last
holds whatever remains, which MUST be the tier byte plus a whole number of slices. A transport
need not preserve message boundaries: the framing comes from the bytes. A receiver MUST reject a
stream whose `IndexStride` is 0 or greater than 960, a chunk whose `N` does not divide it, or a
final chunk that is not whole slices.

Sample count. The receiver derives it from the chunks: a chunk's tier byte gives its `N`, and
its slice count `nsl` gives `nsl * N` samples. A sender whose final slice holds fewer than `N`
samples pads it to the slice; the receiver decodes that padding as the tail, producing at most
`N - 1` extra samples. This is the padding Section 3.2 gives a final slice holding fewer than
`N` samples.

Correspondence. A stream whose chunks are all `B(rate)` samples long is exactly a file with its
length fields zeroed and `IndexStride` stating `B`: the same header, optional `INIT_STATE`
snapshot, and data area, in that order, with no index and no loop snapshot. Setting
`TotalSamples` and `DataBytes` to the exact counts and zeroing `IndexStride` recovers a file,
and the two decode identically over the first `TotalSamples` samples. A stream with shorter
chunks does not correspond to a file directly, because the file grid is `B` samples, but its
slices are the file form's, so re-chunking it yields a file.

## 4. Decoder specification

### 4.1 State

```c
typedef struct {
    int32_t w[4];       /* LMS weights */
    int32_t h[4];       /* sample history, most recent at h[3] */
} kac_ch;

typedef struct {
    unsigned channels;          /* 1 or 2 */
    unsigned tier;              /* 1..7 */
    unsigned w;                 /* residual bit width */
    unsigned N;                 /* samples per slice */
    unsigned shift;             /* 64 - w */
    kac_ch    ch[2];
} kacdec_t;
```

`ch[2]` is 64 bytes for stereo. The control fields `channels/tier/w/N/shift` add about 16
bytes, for about 80 bytes of decode state. That is not the size of the whole handle, which also
holds the parsed header fields and pointers into the file image. "64 bytes" in this
specification means the LMS state alone; "about 80 bytes" includes the control fields.

### 4.2 Per-sample step

```c
static inline int32_t kac_step(kac_ch *c, int32_t residual)
{
    int32_t pred = (c->w[0]*c->h[0] + c->w[1]*c->h[1]
                  + c->w[2]*c->h[2] + c->w[3]*c->h[3]) >> 13;
    int32_t s = pred + residual;
    s = s > 32767 ? 32767 : (s < -32768 ? -32768 : s);   /* output must be 16-bit */

    int32_t d = residual >> 4;
    /* the shift floors, so a small positive residual leaves d at zero while a
     * small negative one already gives -1: one side of the dead zone, closed */
    d += (d == 0 && residual != 0);
    c->w[0] = kac_sat_w(c->w[0] + ((c->h[0] < 0) ? -d : d));
    c->w[1] = kac_sat_w(c->w[1] + ((c->h[1] < 0) ? -d : d));
    c->w[2] = kac_sat_w(c->w[2] + ((c->h[2] < 0) ? -d : d));
    c->w[3] = kac_sat_w(c->w[3] + ((c->h[3] < 0) ? -d : d));

    c->h[0] = c->h[1]; c->h[1] = c->h[2]; c->h[2] = c->h[3]; c->h[3] = s;
    return s;
}
```

```c
/* Saturate a weight to [-16383, 16383]. This is the sufficient condition for the int32
 * accumulator not to overflow; the proof is below. */
static inline int32_t kac_sat_w(int32_t v)
{
    return v > 16383 ? 16383 : (v < -16383 ? -16383 : v);
}
```

This code has no function calls, no division, no floating point, no array indexing except one
quantizer-table load, and no channel test; channel tests live outside the slice.

Why weight saturation is mandatory. It is the sufficient condition against `2^31` overflow:

* History samples pass through `kac_clamp16`, so `|h_i| <= 32768`; note that
  `INT16_MIN = -32768` has magnitude 32768.
* With `|w_i| <= 16383`, the sum of the four products satisfies
  `|sum w_i*h_i| <= 4 * 16383 * 32768 = 2 147 352 576 < INT32_MAX = 2 147 483 647`, with
  131 071 to spare.
* Therefore `pred = sum >> 13` satisfies `|pred| <= 262 144`. Adding the residual (at most
  32 767 by the tables of Section 5.1) stays far inside int32, and `kac_clamp16` produces the
  output. The whole path has no signed overflow and no undefined behavior.

Without saturation it overflows. Take `|res| = 32767` (`d = 2047`, the T5 maximum of
Section 5.1). With the largest residual at every step and a same-sign history,
`|sum w_i*h_i|` passes `INT32_MAX` within a few samples, and the weights themselves overflow
int32 in under a minute of audio. Real signals are no excuse.

A useful side effect: the weights are always representable in int16, so the snapshot of Section
3.3 is 16 bytes per channel with bit-exact restore.

### 4.3 Channel handling

The channel loop sits outside the slice loop and never enters the sample loop. The channel index
and output stride are loop invariants:

```c
stop = loop_en ? dec->loop_end : dec->total;   /* end of the region being decoded */
pos  = 0;
while (pos < stop) {
    in    = pos % B;                           /* position within its chunk */
    if (in == 0)                               /* entering a chunk: its tier byte decides N,
                                                  shift, table and the chunk's byte base */
        load_chunk(dec, pos / B);
    sl  = in / dec->N;                         /* slice index within the chunk */
    off = in % dec->N;                         /* offset within that slice */
    seg = dec->N - off;                        /* rest of this slice */
    if (seg > B - in)
        seg = B - in;                          /* the chunk boundary truncates */
    if (seg > stop - pos)
        seg = stop - pos;                      /* the end of the region truncates */
    for (c = 0; c < dec->channels; c++)
        decode_segment(dec, c, sl, off, seg,
                       out + c + pos * dec->channels,
                       dec->channels /* output stride */);
    pos += seg;
}
```

Mono and stereo share the code; they differ only in the stride and the loop bound, tested once
per segment and never per sample.

This is the Section 3.2 walk with the slice grid restarted at each chunk. The tier byte is read
once per chunk, outside the sample loop. `sl` and `off` are relative to their chunk, and only
the chunk's byte base makes them a data-area position. A whole-file slice index does not exist,
because earlier chunks may carry any tiers.

`seg` is clamped by the slice, the chunk, and the region end, so the last file segment and the
segment ending at `LoopEnd` are truncated, not padded. The chunk clamp reaches past
`TotalSamples` in the last chunk, but `stop <= TotalSamples`, so the region clamp stops the walk.
This mirrors the exact count of Section 4.5: a final slice may be padded to `N`, and the decoder
stops at `TotalSamples`.

### 4.4 Seamless looping

`LoopStart` and `LoopEnd` are sample offsets and need not be slice-aligned. Quantizing loop
points to slice boundaries is unacceptable: a slice holds 10 to 60 samples, so the phase error
would be audible for a wavetable.

Snapshot semantics (frozen): the `LOOP_STATE` snapshot records the LMS state at the sample
`LoopStart`, not at the start of the slice that contains it. `LoopStart` may be any sample
offset, so the snapshot point generally does not coincide with a slice boundary.

To decode one loop period:

1. Find the chunk `q` that holds `LoopStart`. `N` varies per chunk, so
   `slice_idx = LoopStart / N` does not hold. Walk the chunks once, as in Section 3.2, reading
   each tier byte and advancing by `ceil(L/N)*8*Channels` bytes until the running sample count
   covers `LoopStart`. This gives the chunk's byte base and the in-slice offset
   `off = (LoopStart - B*q) % N` with `sidx = (LoopStart - B*q) / N`. Both divisions run
   once per period, never inside the sample loop. Cache this absolute position and reuse it
   every period.
2. Position the bit pointer absolutely. Let `data` point at the first data byte, after the
   header and any snapshots. For channel `c` in slice `m` of the chunk:
   `slice_base = chunk_base + 1 + (m * Channels + c) * 8`. Read one 64-bit word, take
   `sf = v >> 60`, then shift left by `4 + off*w`. Every slice recomputes `slice_base`; no
   running bit pointer carries over from the previous slice.
3. Decode. The state does not move for the skipped residuals: the snapshot already holds the
   state at `LoopStart`. Compute `cnt = min(N - off, LoopEnd - pos)` once per slice and decode
   `cnt` samples with the sliding-window read of Section 3.2.
4. Advance. When the walk leaves a chunk, add `1 + ceil(L/N)*8*Channels` to `chunk_base` and
   take `N` from the next tier byte. Every later slice repeats step 2 from its own
   `slice_base`.
5. Terminate when `pos >= LoopEnd`. Do not test `pos > LoopEnd`: `cnt` stops exactly at
   `LoopEnd`, so with `>` and `LoopEnd` on a slice boundary, `cnt` stays zero and the loop
   spins.
6. End the period with `pos = LoopStart`, reload the `LOOP_STATE` snapshot, and rerun step 2
   from the cached chunk position. Both are required: reloading the state without
   re-positioning decodes the next pass from the wrong bit.

Key points:

* The snapshot is bit-exact (Section 3.3), so every pass is identical sample for sample. That is
  what "seamless" means here.
* Skipping `off <= N-1` residuals costs one bit-pointer move, updates no state, emits nothing,
  needs no buffer, and happens once per period.
* In stereo, `pos` is a per-channel sample position, not an interleaved index. The output
  address is `out + c + pos * channels`, and `off` is a per-channel offset within the slice.
  This is frozen here.
* When `Flags.LOOP = 0`, `LoopStart` and `LoopEnd` are ignored, and the file plays out
  `TotalSamples` samples.

The decoder MUST reject these conditions:

| Condition | Why |
|---|---|
| `LoopStart == LoopEnd` | The loop region is empty: `cnt` is always zero, so a do/while or a `>` test spins. The region emits nothing. Either way the file is illegal |
| `LoopStart > LoopEnd` | `min(N-off, LoopEnd-pos)` goes negative, which is undefined behavior |
| `LoopStart >= TotalSamples` or `LoopEnd > TotalSamples` | The read would leave the data |
| `TotalSamples > 2^28` | Without the bound, `DataBytes` would eventually exceed u32, and a 32-bit target could not load the file |
| `Flags.MS=1` and the decoder does not implement the coded form | Reading coded channels as left and right would produce garbage (Section 3.1) |
| The header `Tier` field or any chunk's tier byte has bit 3 set while `Flags.MS=0` | The file carries a form it does not declare |
| `Channels=1` and a tier byte has bit 3 set | One channel has no pair, so the form has no meaning |
| A chunk's tier byte is 0 or greater than 7 | The chunk's `N` would be undefined |
| The header `Tier` field differs from chunk 0's tier byte, in tier or form | The two are redundant by Section 3.1; a mismatch means the file is corrupt or forged |
| `SEEK_INDEX=1` and `IndexStride` is not 0 (mod B) | An index point would not land on a chunk boundary |
| `SEEK_INDEX=1` and the sought offset entry has `e < 1` or `e >= DataBytes` | The byte at `e - 1` would be the chunk's tier byte, outside the data area. Reject that seek; the rest of the file is unaffected |
| `SEEK_INDEX=1` and the byte at `e - 1` fails the tier-byte rule (tier 0 or greater than 7, a non-zero bit 4-7, or bit 3 set with `Flags.MS=0` or `Channels=1`) | The entry does not point at a chunk, so its `N` and form are undefined. Reject that seek |
| `SEEK_INDEX=1` and `e + ceil(L/N)*8*Channels > DataBytes`, with `L = min(B, TotalSamples - k*IndexStride)` | The chunk does not fit in the data area. Reject that seek |
| The chunk walk gives `sum N != TotalSamples`, a byte total `!= DataBytes`, or a data area past EOF | The declared shape does not match the contents: truncation or forgery. This is the same walk that Section 3.2 uses to locate a sample position, so an unindexed seek or a first loop entry fails the same way. Do not fall back to `Flags.LOOP = 0` |
| `TotalSamples = 0` | Nothing to output. The chunk walk covers no samples, so `DataBytes = 0`, and the snapshot points, `IndexEntries`, and the seek index describe nothing. Every loop-point and seek computation is meaningless |

### 4.5 Gapless playback

The format guarantees gapless playback at five levels:

| Requirement | Mechanism |
|---|---|
| Zero algorithmic delay | LMS is purely causal: output k depends only on the previous 4 emitted samples. No warm-up or discarded leading samples are needed, and the format has no delay field |
| Exact sample count | `TotalSamples` is the exact per-channel count. A final slice is padded with any residual; zero padding is not guaranteed, because `2^w` is even and a sign-symmetric table has no zero codeword. The decoder outputs exactly `TotalSamples` samples |
| No state discontinuity between files | `Flags.INIT_STATE` carries the encoder-time initial state. Without it, the decoder uses the fixed default of Section 3.3, not all zeros: an all-zero state can lock the output at full scale through the blindness mechanism. Removing the start-up transient would require inverting the LMS, an encoder-side topic, so the format does not promise it |
| Seamless looping | The bit-exact snapshot of Section 4.4 plus loop points at arbitrary sample offsets |
| Snapshot bit-identical to continuous decoding | A snapshot is redundant. It MUST be bit-identical to the state obtained by continuous decoding from the header, or it is worse than useless |

Player side: decode each file in turn to `TotalSamples` samples and concatenate directly, with no
overlap, no gap, and no trimming of trailing samples.

> The format does not resample or shift pitch. A wavetable engine does transposition with its own
> phase accumulator and interpolation. The decoder always outputs raw-rate samples at the rate in
> the header.

### 4.6 Decoder prohibitions

The decoder MUST NOT use `float`, `double`, or `math.h`; `malloc` or `free`; sample-rate-dependent
tables; resampling; a separate bit-unpacking path; per-sample channel or loop-point tests (they
are loop invariants); variable-width bit reads; function-level mutable global state (all state
lives in `kacdec_t`, and the code is reentrant); 64-bit division; or a non-power-of-two division
inside the sample loop.

Mandatory:

* 4 weight saturations with `kac_sat_w()` per sample. The overflow proof of Section 4.2 depends
  on it, and omitting it is signed-overflow undefined behavior.
* One `kac_sat_w` per weight after reading any snapshot. All three load paths, `INIT_STATE`,
  `LOOP_STATE`, and `SEEK`, need it.
* The arithmetic right shift of `pred = sum >> 13`, with the compile-time assertion of
  Section 6.2.

## 5. Bitrate tiers

### 5.1 Tier table

The tier id is the width. `N*w` is 60 for every tier that fills the slice, so the bitrate is
exactly `(64/60)*w` and rises with the id:

| Tier | `w` | `N` | Bitrate (bit/channel*sample) | Quantization levels | Quantizer table size |
|---:|---:|---:|---:|---:|---:|
| T1 | 1 | 60 | 1.067 | 2 | 64 B |
| T2 | 2 | 30 | 2.133 | 4 | 128 B |
| T3 | 3 | 20 | 3.200 | 8 | 256 B |
| T4 | 4 | 15 | 4.267 | 16 | 512 B |
| T5 | 5 | 12 | 5.333 | 32 | 1 KB |
| T6 | 6 | 10 | 6.400 | 64 | 2 KB |
| T7 | 7 | 8 | 8.000 | 128 | 4 KB |

T7 is the last tier: `4 + 8*7` is 60 rather than 64, and the next tier would cost more than
8 bit/channel*sample, the rate of plain 8 bit PCM. Its 4 padding bits cost nothing that
matters, because the slice is 8 bytes either way.

* The tier is per chunk: each chunk carries its own tier byte, and the header `Tier` field gives
  chunk 0's (Section 3.2).
* The format has no compatibility obligation, so the magnitude rule below is a break, not a
  versioned change: `Version` stays 1 and no `Flags` bit is spent. A file is decoded with the
  tables of the revision it was written against; the `SHA-256` values below identify that
  revision.
* A tier's table is a `static const int16_t` array: 16 rows, one per scalefactor, of `2^w`
  values, indexed by `(sf << w) | qr` -- one address computation and one load. Magnitudes
  increase within a row and signs alternate symmetrically. The magnitudes follow the grid `q`
  below; T1 and T2 keep the odd ladder `M[j] = 2j+1`.

Four mandatory constraints on the tables. The generator MUST check all of them mechanically:

1. Exactly 16 rows of `2^w` entries. Within a row, the magnitude by `qr>>1` is strictly
   increasing: the `L = 2^(w-1)` magnitudes are all distinct, with no duplicate level. A
   duplicate throws away a quantization level.
2. Signs are exactly symmetric: `dqt[s][2j] = -dqt[s][2j+1]`.
3. `|dqt[s][qr]| <= 32767`, so every value fits in int16.
4. Tables map one-to-one onto tiers, are `static const`, and both ends use the same table.

`q` is deliberately non-uniform: denser at the bottom (`0.75 -> 2.5`) than in the interior
(step 2), and pinned at `2^w - 1` at the top so every row reaches full scale. T1 and T2 are
exempt and keep `M[j] = 2*j + 1`: T2 is 1 and 3, T1 is just 1 (`L = 1`). The grid measures worse
for T2, where `L = 2` makes the table the quantizer; T1 has `L = 1`, where amplitude shape is
vacuous.

Normative generation formula, which satisfies the four constraints:

```
SHA-256(gen_dqt.py output) = 8ae4fe14fe8c252fc020bcc0e2d80a3dee183edcffac3e7001a621d5b959f2c4
SHA-256(payload)           = 23ab4a206bd7eed10b589bbee7768efc28f9334e7f6be5591f91117e65b71e9e
```

The comparison object is the generator's output. The file hash also covers layout, such as line
wrapping, trailing comments, and column width, so a cosmetic reflow moves it. The payload hash
is computed over the table values alone and moves only when the data changes. Compare the
payload hash to decide whether a mismatch is layout or data.

```
L    = 2^(w-1)                                    levels per sign (T1:1, T2:2 ... T6:32, T7:64)
Mmax = 2^w - 1                                    table maximum
q[j] = 0.75                         j = 0          magnitude grid, generated per width
     = 2^w - 1                      j = L-1
     = 0.5 + j*step                 j = 1..L-2     step = floor( (2^w - 1 - 0.75) / (L-1) )
                                                   step = 2 for every L >= 4
K_S  = min( 2.75, log(32767 / Mmax) / log(16) )    per tier, keeps the step table under constraint 3
                                                   the w = 1 tier (T1) uses 3.0 as its ceiling
cap  = 32767 / Mmax                                (integer division)
S[s] = min( round_ties_away( (s+1)^K_S ), cap )    s = 0..15   step table (16 entries)

dqt[s][2j]   = +round_ties_away( S[s] * q[j] )     index qr = 2j   -> positive
dqt[s][2j+1] = -round_ties_away( S[s] * q[j] )     index qr = 2j+1 -> negative

The generator MUST assert every finished table entry-wise: constraint 1 (16 rows x 2^w
entries, strictly increasing magnitudes, L distinct magnitudes), constraint 2 (exact sign
symmetry), constraint 3 (|value| <= 32767), plus S[] non-decreasing. Any failure MUST abort.
Generating a table that violates a constraint is not allowed.
```

`round_ties_away(x)` rounds half away from zero.

## 6. Conventions the format depends on

### 6.1 Little-endian bitstream

The bitstream is little-endian. Reads use explicit byte assembly and depend on neither host byte
order nor alignment. Snapshot fields are read and written the same way, never by copying a
struct: padding bytes and alignment would then differ between implementations.

### 6.2 Arithmetic right shift

The arithmetic right shifts (`pred >> 13` and `residual >> 4`) rely on negative values shifting
arithmetically. In C99 this is implementation-defined, not undefined behavior. It is a platform
requirement here, satisfied by gcc, clang, and MSVC, and is pinned down with a C99 compile-time
assertion:

```c
typedef char kac_assert_arith_shift[((-1 >> 1) == -1) ? 1 : -1];
```

### 6.3 Field bounds

The `TotalSamples <= 2^28` bound of Section 3.1 keeps `DataBytes`, the index area, and the whole
file length inside signed 32-bit. A 32-bit target's `size_t` can therefore hold the file, and no
field of this format needs 64-bit arithmetic.

## 7. Format quick reference

```
KAC                        file header, 36 bytes
 + Magic 4B 41 43 00 | Ver=1 | Flags | Channels(1-2) | Tier(1-7, bit3 form)
   SampleRate(u32) | TotalSamples(u32) | LoopStart(u32) | LoopEnd(u32)
   DataBytes(u32) | IndexStride(u32) | IndexEntries(u32)

[INIT_STATE]  Channels * 16 B   state at sample 0
[LOOP_STATE]  Channels * 16 B   state at sample LoopStart   <- basis for seamless looping
[data]        [tier byte][B samples of slices][tier byte][...]   (a file's grid; a stream
              chunk may carry fewer slices, Section 3.5)
              within a chunk, 8 bytes per channel per slice: 4-bit sf + N residuals of w bits,
              4 + N*w <= 64
              (a tier byte is tier in bits 0-2, this chunk's channel form in bit 3, nothing
               else; a chunk in the coded form carries mid and side, Section 3.2)
[SEEK_INDEX]  u32 offset table + Channels*16 B snapshots * IndexEntries
              (IndexStride congruent 0 mod B, so index points land on chunk and slice
               boundaries)

stream        the same payload with TotalSamples/DataBytes/IndexEntries zero and IndexStride
              stating the chunk's sample count (1..960), no index and no LOOP: a receiver
              frames the bytes from that count, and the last chunk takes the remainder
              (Section 3.5)

tier       T1      T2      T3      T4      T5      T6      T7
w/N        1/60    2/30    3/20    4/15    5/12    6/10    7/8
bit/sample 1.067   2.133   3.200   4.267   5.333   6.400   8.000

weight range [-16383, 16383] (saturated every step, see 4.2; loading ANY snapshot requires
                              one kac_sat_w per weight, see 3.3 / 4.6)
control fields channels/tier/w/N/shift ~16 B -> kacdec_t stereo ~80 B (LMS state alone is 64 B)
```
