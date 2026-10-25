# KAC

Kunkun's Audio Codec: a time domain codec (4th order LMS prediction plus scalar residual
quantization) for wavetable and embedded playback. Zero algorithmic latency, gapless looping at
arbitrary sample offsets, 16 bit PCM output, C99, no external libraries.

The bit stream is specified in `doc/KAC_SPEC.md`. This file says how to build and use the code.

## Build

The sources sit in `src/`; run the commands below from there.

```sh
bash build.sh              # kac and the seven test programs, into build/
bash build.sh --sanitize   # plus ASan+UBSan copies
bash build.sh --clean
make check                 # full acceptance run (tests, sanitizers, audit, sizes)
```

Products land in `build/`, which is ignored by git. Flags are
`-std=c99 -Wall -Wextra -pedantic-errors -O3`; the tree builds without warnings.
`make format` (or `bash build.sh --format`) reformats the sources with
`CLANG_FORMAT`, which defaults to `clang-format-18`.

## Command line

`build/kac --longhelp` prints the following and exits. `--help` prints it
without the notes at the end, and `-?` is the same as `--longhelp`.

```
KAC 1.0

usage: kac [options] <infile> [outfile]          encode a WAV to KAC
       kac --decode [options] <infile> [outfile] decode a KAC to WAV
       outfile defaults to infile with .kac (encode) or .wav (--decode).

RECOMMENDED:
    kac in.wav out.kac            default, -V 5
    kac -V 0 in.wav out.kac       best quality
    kac -V 8 in.wav out.kac       --preset low
    kac -V 6.5 in.wav out.kac     --preset medium
    kac -V 5 in.wav out.kac       --preset standard
    kac -V 3 in.wav out.kac       --preset extreme
    kac -V 2 in.wav out.kac       --preset insane
    kac --tier 3 in.wav out.kac   one fixed tier

OPTIONS:

  Quality and rate:
    -b <rate>        bit/channel*sample: pinned tier, or the per-block
                     floor with a VBR mode
    -B <rate>        per-block ceiling; ignored when the tier is fixed
    --tier <2..6>    one tier for every block
    --psnr <dB>      VBR, target PSNR, bit rate floats
    -V <0..9.999>    VBR, quality target, bit rate floats;
                     0 = best, default 5
    --preset <name>  low|medium|standard|extreme|insane
    --abr <kbps>     VBR, average bit rate, quality floats
    --abr-bps <rate> the same in bit/channel*sample
    --lowdelay [tier] low-delay stream: tier (default 3),
                     chunks near 5 ms, at most 5.5 ms
    --chunk-ms <ms>  with --lowdelay: target chunk length, default 5

  Operational options:
    --loop <S,E>     loop region in samples
    --stride <N>     seek index every N samples (multiple of the block)
    --limit <N>      encode the first N samples per channel
    -m <s|j|f|m>     s = L/R, j = measured (default), f = mid/side,
                     m = mono
    --mono, -a       downmix to one channel
    -q <0..9>        algorithm quality, 0 = best, default 3
    -f               -q 7
    -h               -q 2
    --brute <1|2>    a harder search than any -q level
    --decode         decode instead of encode
    --loop-count <K> with --decode: play the loop K times

  Other:
    -S, --silent, --quiet
                     no banner, progress line or summary
    --help           this list
    --longhelp, -?   this list and the notes
    --version        version

NOTES:
  -V and --preset set the same rung; the later one wins, with a warning.
  --psnr and --abr/--abr-bps are separate modes and are refused with
  -V/--preset and with each other.
  -b is a rate: alone it pins a tier in T2..T6; with a VBR mode it is
  the floor and -B the ceiling, over T1..T7. --tier is refused with a
  VBR mode or -b.
  -m j measures the channel form per block; -m s and -m f force one.
  --lowdelay makes a stream: default T3, mid/side, -q 7, chunks near
  5 ms (at most 5.5 ms). All encoder options apply except --loop and
  --stride. The chunk length is in the stream header; the last chunk is
  padded, so the output can hold up to N-1 extra samples.
  --brute replaces the -q level: 1 and 2 are settings past -q 0.
  The banner, progress line and summary go to stderr; -S suppresses
  them. --mono is an encode option and is refused with --decode.
```

## Tiers a pinned run cannot name

Tier 1 and tier 7 are tiers of the format: the decoder accepts them in the header
and in a block, and `kac_set_tier()` takes either. But `--tier` and `-b` reach
only the five between them, and `--lowdelay` may name T1 or T7 on a pinned run.

T1 spends 1.067 bit/sample (`w = 1`, `n = 60`, `4 + 60*1 = 64` bits per slice),
and loses far more level than the next tier up on any material a run would be cut
from. T7 spends 8.000 (`w = 7`, `n = 8`, 64 bits per slice with four of them
spare), the same as plain 8-bit PCM. Neither is worth that where a block is merely
average, so the per-block rule holds them to a reading other than the block's
level: T1 only where even the weakest five per cent of the block reaches the aim,
T7 only where the tier below it misses the aim that far in from that tier's
strongest end. A block that fails either test takes the neighbouring tier
instead, T2 or T6. A run pinned with `--tier` or `-b` names a tier outright, and
its blocks take it.

## Choosing a tier per block

Choosing a tier per block costs a few times a pinned tier: the block is measured
at several tiers before one is picked, while `--tier` and `-b` measure it at one.
That is the difference between an encoder that fits the time available on a small
processor and one that does not, and the ratio does not get smaller on slower
hardware. On a processor without an FPU and with few general-purpose registers --
the class this format is aimed at -- encode with `--tier`, or with `-b`, which is
a table lookup; `-V`, `--abr` and the default belong on machines whose encoder
time is not scarce. The decoder is unaffected by the choice: it reads the same
slices, and decodes two orders of magnitude faster than real time.

## Encoder time budget

3 s for 10 s of 44.1 kHz stereo, asserted by `test_roundtrip` on the
configuration a run gets when it selects nothing. Wall clock depends on the
machine, so it is a ceiling and not a measurement, and that test prints the
measured figure beside it.

## Quality levels

The search can be given up in steps, and the steps are what a run pays for speed.
Level 0 runs every search this tree has, and measures a block at every tier in
the window it opens; the levels above it read those tiers off one approximate
pass instead. A run that selects nothing gets `-q 3`. Four settings make up a
level: the beam width, the candidate count, how often the beam collapses, and how
the channel form is measured:

| `-q` | Beam width | Candidates | Beam lag | Channel form |
|---:|---:|---:|---:|---|
| 0 | 8 | 9 | whole slice | coded |
| 1 | 7 | 9 | whole slice | coded |
| 2 | 6 | 9 | whole slice | coded |
| 3 | 5 | 8 | whole slice | estimated over 8 rows |
| 4 | 4 | 8 | 4 | estimated over 4 rows |
| 5 | 3 | 7 | 2 | estimated over 2 rows |
| 6 | 2 | 6 | 1 | estimated over 1 row |
| 7 | 0 | 6 | - | none: the block is sent in the form it arrived in |
| 8 | 0 | 5 | - | none: the block is sent in the form it arrived in |
| 9 | 0 | 4 | - | none: the block is sent in the form it arrived in |
| 10 | 0 | - | - | none, and no row search either |

A beam width of 0 removes the beam, not the row search: levels 7, 8 and 9 still
try candidate rows and take the nearest level per sample. Level 10 drops that
too, leaving one row from the residual envelope and the nearest level
everywhere. The command line stops at 9; level 10 is reachable through
`kac_set_quality()`. `--brute 1` and `--brute 2` replace the level with a setting
past level 0, so `-q` no longer changes any part of the search. A level is
settled when the encoder is initialised, `kac_set_quality()` being the same
setting for a caller.

A level does not choose the quality target: `-V`, `-b`, `--psnr` and `--abr` do.
A weaker level reaches the same target with more bits in less time, and each
level's reading of a tier carries its own measured offset, so the reading tracks
what that level actually delivers. On `-V` the brute settings aim above the rung
instead, by 0.3 dB: a harder search that met the rung exactly would turn its
extra work into rate rather than level.

A few common invocations:

```sh
build/kac music.wav music.kac                   # the default configuration
build/kac --tier 3 music.wav music.kac          # one tier for every block, T3
build/kac -b 6.4 music.wav music.kac            # the top tier, chosen by lookup
build/kac -V 5 -b 3.2 -B 4.3 music.wav m.kac    # per-block tiers between T3 and T4
build/kac --psnr 50 music.wav music.kac         # a target in dB, the same engine as -V
build/kac --loop 44100,88200 music.wav m.kac    # loop points, arbitrary offsets
build/kac --limit 441000 music.wav m10.kac      # first 10 s only
build/kac -V 3 music.wav m.kac                  # quality target, per-block tiers
build/kac --abr 320 --stride 5040 music.wav m.kac # 6 blocks at 44.1 kHz
build/kac --decode m.kac out.wav
build/kac --decode --loop-count 3 m.kac out.wav # three loop passes
```

Everything the tool prints goes to stderr, so stdout stays empty and can be redirected
freely. A run prints a banner, an `Encoding` / `Decoding` line, one line naming the configuration,
a progress display (a header printed once, then a status line rewritten in place with a carriage
return), and a summary. Times are CPU time and `play/CPU` is audio seconds per CPU second. `-S`
drops the whole display along with the banner and the summary.

## Library

Both handles are supplied by the caller and are safe to place on the stack; the library never
allocates.

```c
kac_t g;
kac_init(&g);
kac_set_num_channels(&g, 2);
kac_set_in_samplerate(&g, 44100);
kac_set_brate(&g, 4.27);                  /* or kac_set_tier(&g, 4), or kac_set_vbr(...) */
kac_init_params(&g);                      /* validates and fixes the tier */

while (frames_left) {
    kac_encode_buffer_interleaved(&g, pcm, n, out, &outlen);  /* bytes written */
    write(fd, out, outlen);
}
kac_encode_flush(&g, out, &outlen);       /* tail block plus optional index */
write(fd, out, outlen);
kac_write_preamble(&g, head, &headlen);   /* then overwrite file offset 0 */
lseek(fd, 0, SEEK_SET);
write(fd, head, headlen);
kac_close(&g);
```

`kac_encode_bufsize(&g, frames)` bounds one encode call and the flush. Pass the file's total
frame count: the flush writes the seek index as well as the tail block, and the index depends on
the total rather than on any one call, so sizing it from a single call's frame count leaves it
uncovered. A single buffer of that size is enough for the whole loop above.

A run's tier can also be bounded rather than fixed: `kac_set_tier_limits(&g, lo, hi)` is what the
command line's `-b` and `-B` fold into on the per-block path, and a block is then given the cheapest
tier that reaches the run's aim from inside that window. `kac_tier_for_brate()` and
`kac_tier_ceil_brate()` are the two lookups a rate going in is turned into a window's ends by.

`kac_get_state(&g, snaps)` reads the LMS state the encoder has reached, and
`kac_set_init_state(&g, snaps)` makes another handle start from it. A file encoded that way carries
`INIT_STATE`, so a decoder starts from the same state and the second file continues the first: its
samples are the ones the first would have produced next. Split on a block boundary -- a block's
channel form is chosen from the samples it holds and the state it starts in, so only a shared block
grid makes the same samples code the same way -- and both files must use the same tier.

The preamble carries
`TotalSamples`, so it can only be produced after the last frame: the encoder emits placeholder
bytes of the same length first, and `kac_write_preamble` returns the final contents to patch in at
offset 0. `kac.h` carries the full declaration set and the per-function detail.

```c
kacdec_t d;
kacdec_init(&d);
if (kacdec_open(&d, image, len) != KAC_OK) reject();
while ((got = kacdec_read(&d, out, frames)) > 0)
    write(fd, out, got * 2 * kacdec_channels(&d));   /* 0 marks the end */
kacdec_seek(&d, sample);        /* needs the seek index */
kacdec_set_loop(&d, 1);         /* wrap at LoopEnd instead of stopping */
kacdec_close(&d);
```

`kacdec_open` returns 0 or a negative `KAC_E_*` code; malformed input is rejected without any
undefined behaviour.

## Streaming

A file states its length, so its preamble can only be produced after the last frame. A live stream
cannot, so the same payload goes over the wire without them: `TotalSamples` and `DataBytes` are zero
and `IndexStride` states the chunk's sample count. A stream is not a file -- the specification
Section 3.5 -- and `kacdec_open` refusing it is the point.

```c
kac_t  g;
size_t outlen, headlen;
unsigned chunk;

kac_init(&g);
kac_set_num_channels(&g, 2);
kac_set_in_samplerate(&g, 48000);
kac_set_vbr(&g, KAC_VBR_V, 8);
kac_init_params(&g);
kac_set_stream(&g, 1);                 /* no leading placeholder */
chunk = kac_get_chunk_samples(&g);     /* about 20 ms by default */

/* One kac_encode_buffer_interleaved() call per chunk frames, so each call's
 * output is exactly one block of the stream. */
if (kac_encode_buffer_interleaved(&g, pcm, chunk, out, &outlen) > 0) {
    kac_write_stream_preamble(&g, head, &headlen);
    send(head, headlen);               /* before the blocks, not after them */
    send(out, outlen);
}
send(next blocks the same way);
kac_encode_flush(&g, out, &outlen);    /* the last block */
send(out, outlen);
```

`kac_write_stream_preamble()` follows the first block rather than preceding it: the header's `Tier`
field is block 0's tier and form, and the encoder fixes those only once block 0 has been coded -- so
the caller holds one block's bytes across the call. `kac_set_stream()` refuses a loop region and a
seek index; neither can exist in a stream. A stream's default chunk is the file block: about
20 ms, never more than 20.5 ms, or one quantum where even that is longer. `kac_set_chunk_ms()`
changes it, up to 960 samples, and the preamble states it.
The command line's `--lowdelay` writes such a stream, and `--decode` reads it without any
out-of-band chunk length.

```c
kacdec_t d;
int32_t  got;

kacdec_init(&d);
kacdec_stream_open(&d, head, headlen);     /* the preamble need not outlive this call */
for (;;) {
    size_t len = receive(&blk);
    if (len == 0) break;
    if (kacdec_stream_feed(&d, blk, len) != KAC_OK) reject();
    while ((got = kacdec_read(&d, out, frames)) > 0) play(out, got);
}
```

`blk` must stay valid until the next feed: the decoder holds no copy, so this is a borrow and not a
queue. On a stream `kacdec_read` returning 0 means the frames fed so far are exhausted, not that the
stream has ended -- feeding more moves it on.

A block's slice count is what gives it its sample count, so a stream's last block is padded up to its
slice and decodes at most `N - 1` samples past the sender's last one. The sender knows the exact
count; the same bytes with `TotalSamples` and `DataBytes` filled in and `IndexStride` zeroed are the
file that states it.

## Tests

From `src/`:

```sh
bash tests/run.sh          # functional tests, sanitizers, strict compile, review, footprint
bash tests/deadcode.sh     # unused code, leftover experiment switches, non-ASCII characters
```

`tests/run.sh` builds the tree first if the products are missing. The benchmark and timing
scripts (`tests/bench.sh`, `tests/timing.sh`) read their material from `$KAC_MATERIAL`;
a missing track is reported as an error rather than skipped. `tools/calibrate_v.c` is the
ladder-endpoint scan and is not part of `make check`.

## Files

```
src/kac.c              command line (encode and decode in one binary)
src/kac.h              public interface for both ends
src/kac_encode.c       encoder
src/kac_decode.c       decoder
src/kac_internal.h     tables, predict/step/sat, snapshot layout, byte helpers
src/kac_dqt.h          dequantization tables, generated by tools/gen_dqt.py
src/wav_io.h           WAV reader and writer
src/tools/gen_dqt.py   table generator
src/tools/check_dqt.py grid check for the generated tables
src/tools/calibrate_v.c  the -V ladder's endpoint scan (not part of make check)
src/tests/             test_roundtrip test_loop test_bad test_seek test_select test_block
                       test_stream
                       run.sh audit_objdump.py deadcode.sh bench.sh timing.sh
doc/KAC_SPEC.md        the bit stream specification
```
