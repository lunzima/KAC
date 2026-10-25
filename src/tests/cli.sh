#!/bin/bash
# cli.sh -- command line smoke tests: options, config line, header fields and
# help/README. Prints nothing on success.
set -u
BIN=${BIN:-build/kac}
TMP=$(mktemp -d) || exit 1
trap 'rm -rf "$TMP"' EXIT
WAV=$TMP/in.wav
FAILS=0

fail() {
    echo "FAIL: $*" >&2
    FAILS=$((FAILS + 1))
}

# A deterministic 0.2 s, 16-bit stereo WAV and its mono twin, so the script
# carries its material.
python3 - "$WAV" "$TMP/mono.wav" "$TMP/r2k.wav" <<'PY' || { echo "FAIL: could not write the test WAV" >&2; exit 1; }
import math
import struct
import sys
import wave
n = 8820
frames = bytearray()
for i in range(n):
    a = int(12000 * math.sin(2 * math.pi * 440 * i / 44100))
    b = int(9000 * math.sin(2 * math.pi * 660 * i / 44100))
    frames += struct.pack('<hh', a, b)
w = wave.open(sys.argv[1], 'wb')
w.setnchannels(2)
w.setsampwidth(2)
w.setframerate(44100)
w.writeframes(bytes(frames))
w.close()
mono = bytearray()
for i in range(n):
    v = int(12000 * math.sin(2 * math.pi * 440 * i / 44100))
    mono += struct.pack('<h', v)
w = wave.open(sys.argv[2], 'wb')
w.setnchannels(1)
w.setsampwidth(2)
w.setframerate(44100)
w.writeframes(bytes(mono))
w.close()
low = bytearray()
for i in range(1000):
    v = int(10000 * math.sin(2 * math.pi * 200 * i / 2000))
    low += struct.pack('<hh', v, v)
w = wave.open(sys.argv[3], 'wb')
w.setnchannels(2)
w.setsampwidth(2)
w.setframerate(2000)
w.writeframes(bytes(low))
w.close()
PY

# Config line of one run: the "Encoding as ..." line the encoder writes to stderr.
config_line() {
    "$BIN" "$@" "$WAV" "$TMP/o.kac" 2>&1 >/dev/null | grep -m1 '^Encoding as'
}

expect_config() {
    want=$1
    shift
    got=$(config_line "$@")
    case "$want" in
    !*)
        case "$got" in
        *"${want#!}"*) fail "config line must not contain '${want#!}': expected no match, actual '$got'" ;;
        esac
        ;;
    *)
        case "$got" in
        *"$want"*) ;;
        *) fail "config line: expected to contain '$want', actual '$got'" ;;
        esac
        ;;
    esac
}

# Section 1: what the config line says about the tier window.
expect_config '!tiers' -V 5
expect_config 'tiers T4..T7' -b 4.0 -V 5
expect_config 'tiers T1..T4' -B 4.3
expect_config 'tiers T1..T7' -b 1.0 -V 5
expect_config 'T2, 2.1333 bit/sample' -b 1.0
expect_config 'T4, 4.2667 bit/sample' -b 4.267 -B 6.4
expect_config '-B ignored: the tier is fixed' -b 4.267 -B 6.4
expect_config 'tiers T4..T4 (held: -B is under -b)' -V 5 -b 4.0 -B 4.1
expect_config '-q 3' -V 5
expect_config 'form measured' -V 5
expect_config 'left/right' -m s
expect_config 'mid/side' -m f
expect_config 'form as it arrives' -V 5 -q 9
expect_config '!left/right' --mono

# Header fields: Flags (byte 5), Channels (byte 6), Tier (byte 7).
header() {
    python3 - "$1" <<'PY'
import sys
b = open(sys.argv[1], 'rb').read(36)
print('%d %d %d %d' % (b[5], b[6], b[7], int.from_bytes(b[12:16], 'little')))
PY
}

encode() {
    "$BIN" -S "$@" "$WAV" "$TMP/h.kac" || fail "encode failed: $*"
}

encode -m f
read -r flags ch tier total <<<"$(header "$TMP/h.kac")"
if [ $((flags & 8)) -eq 0 ] || [ $((tier & 8)) -eq 0 ]; then
    fail "-m f: expected Flags.MS and the Tier form bit set, actual flags=$flags tier=$tier"
fi

encode -m s
read -r flags ch tier total <<<"$(header "$TMP/h.kac")"
if [ $((flags & 8)) -ne 0 ] || [ $((tier & 8)) -ne 0 ]; then
    fail "-m s: expected both form bits clear, actual flags=$flags tier=$tier"
fi

# Flags.MS means the file may carry mid/side somewhere; chunk 0 need not.
encode
read -r flags ch tier total <<<"$(header "$TMP/h.kac")"
if [ "$ch" -ne 2 ]; then
    fail "default: expected Channels 2, actual $ch"
fi
if [ $((flags & 8)) -eq 0 ]; then
    fail "default: expected Flags.MS for a measured stereo run, actual flags=$flags"
fi

encode --mono
read -r flags ch tier total <<<"$(header "$TMP/h.kac")"
if [ "$ch" -ne 1 ]; then
    fail "--mono: expected Channels 1, actual $ch"
fi

encode --tier 3
read -r flags ch tier total <<<"$(header "$TMP/h.kac")"
if [ $((tier & 7)) -ne 3 ]; then
    fail "--tier 3: expected the Tier field to name 3, actual $tier"
fi

# Section 3: option combinations that must be refused.
reject() {
    "$BIN" "$@" "$WAV" "$TMP/r.kac" >/dev/null 2>&1
    rc=$?
    if [ "$rc" -eq 0 ]; then
        fail "expected a non-zero exit, actual 0: kac $*"
    fi
}

reject --tier 3 -V 5
reject -V 5 --tier 3
reject --mono --decode
reject -V 12
reject --tier 1
reject --tier 7
reject --stride 5120
reject --stride 0
reject -B 0

# Section 4: encode then decode, and hold the sample count to the header.
frames_of() {
    python3 - "$1" <<'PY'
import sys
import wave
w = wave.open(sys.argv[1], 'rb')
print('%d %d' % (w.getnframes(), w.getnchannels()))
w.close()
PY
}

IFS=' ' read -r in_frames in_ch <<<"$(frames_of "$WAV")"

roundtrip() {
    "$BIN" -S "$@" "$WAV" "$TMP/rt.kac" || { fail "roundtrip encode failed: $*"; return; }
    "$BIN" -S --decode "$TMP/rt.kac" "$TMP/rt.wav" || { fail "roundtrip decode failed: $*"; return; }
    read -r flags ch tier total <<<"$(header "$TMP/rt.kac")"
    IFS=' ' read -r out_frames out_ch <<<"$(frames_of "$TMP/rt.wav")"
    if [ "$total" -ne "$in_frames" ]; then
        fail "roundtrip $*: TotalSamples $total, expected $in_frames"
    fi
    if [ "$out_frames" -ne "$in_frames" ]; then
        fail "roundtrip $*: decoded $out_frames samples, expected $in_frames"
    fi
    if [ "$out_ch" -ne "$ch" ]; then
        fail "roundtrip $*: decoded $out_ch channels, header says $ch"
    fi
}

roundtrip -V 5
roundtrip -b 4.0 -V 5
roundtrip --tier 3
roundtrip -m s
roundtrip --mono

# With only an input named, the output takes the input's name and the other
# extension.
cp "$WAV" "$TMP/der.wav"
"$BIN" -S "$TMP/der.wav" || fail "derived encode failed"
[ -f "$TMP/der.kac" ] || fail "derived encode did not write der.kac"
"$BIN" -S --decode "$TMP/der.kac" || fail "derived decode failed"
[ -f "$TMP/der.wav" ] || fail "derived decode did not write der.wav"

# The encoder is deterministic: the same valid command writes the same bytes.
"$BIN" -S -q 2 "$WAV" "$TMP/d1.kac" 2>/dev/null
"$BIN" -S -q 2 "$WAV" "$TMP/d2.kac" 2>/dev/null
cmp -s "$TMP/d1.kac" "$TMP/d2.kac" || fail "the same command wrote two different files"

# A short option takes its value attached or as the next word.
"$BIN" -S -q2 "$WAV" "$TMP/s1.kac"
"$BIN" -S -q 2 "$WAV" "$TMP/s2.kac"
cmp -s "$TMP/s1.kac" "$TMP/s2.kac" || fail "-q2 and -q 2 differ"
"$BIN" -S -V5 "$WAV" "$TMP/s1.kac"
"$BIN" -S -V 5 "$WAV" "$TMP/s2.kac"
cmp -s "$TMP/s1.kac" "$TMP/s2.kac" || fail "-V5 and -V 5 differ"
"$BIN" -S -b4.0 -V5 "$WAV" "$TMP/s1.kac"
"$BIN" -S -b 4.0 -V 5 "$WAV" "$TMP/s2.kac"
cmp -s "$TMP/s1.kac" "$TMP/s2.kac" || fail "-b4.0 -V5 and the spaced form differ"
"$BIN" -S -ms "$WAV" "$TMP/s1.kac"
"$BIN" -S -m s "$WAV" "$TMP/s2.kac"
cmp -s "$TMP/s1.kac" "$TMP/s2.kac" || fail "-ms and -m s differ"

# Section 5: invalid configurations exit 2 and start no encode.
refuse() {
    "$BIN" "$@" "$WAV" "$TMP/r.kac" >/dev/null 2>"$TMP/r.err"
    rc=$?
    if [ "$rc" -ne 2 ]; then
        fail "expected exit 2, actual $rc: kac $*"
    fi
    if grep -q "Encoding" "$TMP/r.err"; then
        fail "a refused run printed the encode banner: kac $*"
    fi
}

# Direction: every encoding option is refused with --decode, in either order.
refuse --decode -b 4
# The input is not overwritten by naming it as the output.
"$BIN" -S "$WAV" "$WAV" >/dev/null 2>"$TMP/e"
if [ $? -ne 2 ] || ! grep -q "same file" "$TMP/e"; then
    fail "input and output being the same file should be refused"
fi
# "-" is a conventional stdin/stdout spelling; this tool requires files.
"$BIN" -S - "$TMP/x.kac" >/dev/null 2>"$TMP/e"
if [ $? -ne 2 ] || ! grep -q "not supported" "$TMP/e"; then
    fail "stdin/stdout should be refused with a clear message"
fi
refuse --decode -B 5
refuse --decode -V 5
refuse -V 5 --decode
refuse --decode --preset low
refuse --decode --psnr 45
refuse --decode --abr 128
refuse --decode --abr-bps 3.0
refuse --decode --tier 3
refuse --decode -q 3
refuse --decode -f
refuse --decode -h
refuse --decode -m s
refuse --decode --stride 5760
refuse --decode --limit 100
refuse --decode --loop 10,20
refuse --loop-count 2

# The loop region: order, range, sign, wrap, and the encoded length.
refuse --loop 5,5
refuse --loop 6,3
refuse --loop -1,3
refuse --loop 0,4294967296
refuse --loop 3,
refuse --loop 100,999999

# A refusal names the option it refused, rather than only printing usage.
"$BIN" --bogus "$WAV" "$TMP/x.kac" >/dev/null 2>"$TMP/e"
if [ $? -ne 2 ] || ! grep -q "unknown option --bogus" "$TMP/e"; then
    fail "an unknown option should be named in the refusal"
fi
"$BIN" --lowdelay 8 "$WAV" "$TMP/x.kac" >/dev/null 2>"$TMP/e"
if [ $? -ne 2 ] || ! grep -q "tier from 1 to 7" "$TMP/e"; then
    fail "an out-of-range --lowdelay tier should be named in the refusal"
fi
"$BIN" --stride 5120 "$WAV" "$TMP/x.kac" >/dev/null 2>"$TMP/e"
if [ $? -ne 2 ] || ! grep -q "multiple of the .*block" "$TMP/e"; then
    fail "--stride should say what it requires"
fi

# Numbers: no trailing rubbish, no infinities or NaN, no sign, no wrap.
refuse --stride -1
refuse --limit -1
refuse --loop-count -1
refuse --limit 4294967296
refuse -b 50x
refuse -b inf
refuse -b nan
refuse -B 50x
refuse -B inf
refuse --psnr inf
refuse --psnr 50x

# Section 6: legal but surprising configurations warn and still run.
warned() {
    "$BIN" -S "$@" "$WAV" "$TMP/w.kac" >/dev/null 2>"$TMP/w.err"
    rc=$?
    if [ "$rc" -ne 0 ]; then
        fail "expected exit 0, actual $rc: kac $*"
    fi
    grep -q "warning:" "$TMP/w.err" || fail "expected a warning: kac $*"
}

warned -b 1.0
warned -b 8
warned -b 1.0 -V 5
warned -B 9 -V 5
warned -b 5 -B 4 -V 5
warned --tier 3 -B 5
warned -b 4 -B 5
warned --abr 50
warned --psnr 70
warned --psnr 20
warned -b 1e300
warned -B 1e300
refuse --psnr 1e300
refuse --psnr 86
warned --abr 4294967295
warned -q 3 -f
warned --preset low -V 5
warned -m s -a
warned --limit 0
warned --limit 9999999

# A mono input with -m f: the form has nothing to do.
"$BIN" -S -m f "$TMP/mono.wav" "$TMP/m.kac" >/dev/null 2>"$TMP/w.err"
if [ $? -ne 0 ] || ! grep -q "warning:" "$TMP/w.err"; then
    fail "mono input with -m f: expected exit 0 and a warning"
fi

# --loop-count 0 on a decode.
"$BIN" -S --loop 100,200 "$WAV" "$TMP/loop.kac" 2>/dev/null ||
    fail "could not build a loop region"
"$BIN" -S --decode --loop-count 0 "$TMP/loop.kac" "$TMP/loop.wav" >/dev/null 2>"$TMP/w.err"
if [ $? -ne 0 ] || ! grep -q "warning:" "$TMP/w.err"; then
    fail "--loop-count 0: expected exit 0 and a warning"
fi

# An ordinary run says nothing of the kind.
quiet() {
    "$BIN" -S "$@" "$WAV" "$TMP/q.kac" >/dev/null 2>"$TMP/q.err"
    if [ $? -ne 0 ] || grep -q "warning:" "$TMP/q.err"; then
        fail "expected a quiet valid run: kac $*"
    fi
}

quiet
quiet -V 5
quiet --tier 3
quiet -b 4 -V 5
quiet --abr 200
quiet --psnr 45
"$BIN" --psnr 45 "$WAV" "$TMP/ps.kac" >/dev/null 2>"$TMP/ps.err"
grep -q "^VBR --psnr 45.000 dB" "$TMP/ps.err" ||
    fail "the --psnr summary line should name --psnr and its dB target"
if grep -q "^VBR --abr" "$TMP/ps.err"; then
    fail "the --psnr summary line says --abr"
fi
quiet --limit 4000
quiet --loop 100,200
"$BIN" --quiet "$WAV" "$TMP/qq.kac" >/dev/null 2>"$TMP/qq.err" ||
    fail "--quiet should encode"
[ -s "$TMP/qq.err" ] && fail "--quiet should print nothing"

# Section 7: the low-delay stream.
"$BIN" --lowdelay "$WAV" "$TMP/ld.kac" >"$TMP/ld.out" 2>"$TMP/ld.err" ||
    fail "low-delay encode failed"
grep -q "T3, mid/side, chunk 220 samples" "$TMP/ld.err" ||
    fail "the config line should name T3, mid/side and the 220-sample chunk"
grep -q "chunk 220 samples, -q 7" "$TMP/ld.err" ||
    fail "the config line should name the low-delay quality level"
python3 - "$TMP/ld.kac" <<'PY' || fail "the output is not a stream"
import sys
b = open(sys.argv[1], 'rb').read(36)
assert b[:4] == b'KAC\0', 'magic'
assert int.from_bytes(b[12:16], 'little') == 0, 'TotalSamples'
assert int.from_bytes(b[24:28], 'little') == 0, 'DataBytes'
assert int.from_bytes(b[28:32], 'little') == 220, 'chunk samples in IndexStride'
assert int.from_bytes(b[32:36], 'little') == 0, 'IndexEntries'
PY
"$BIN" -S --decode "$TMP/ld.kac" "$TMP/ld.wav" ||
    fail "stream decode failed"
IFS=' ' read -r ld_frames _ <<<"$(frames_of "$TMP/ld.wav")"
if [ "$ld_frames" -ne "$in_frames" ]; then
    fail "stream decode returned $ld_frames frames, expected $in_frames"
fi
psnr=$(python3 - "$WAV" "$TMP/ld.wav" <<'PY'
import math
import struct
import sys
import wave

def read(path):
    w = wave.open(path, 'rb')
    n, ch = w.getnframes(), w.getnchannels()
    data = struct.unpack('<%dh' % (n * ch), w.readframes(n))
    w.close()
    return n, ch, data

n, ch, a = read(sys.argv[1])
m, ch2, b = read(sys.argv[2])
assert n == m and ch == ch2
sse = sum((x - y) * (x - y) for x, y in zip(a, b))
peak = max(abs(x) for x in a)
print('%.2f' % (10 * math.log10((peak * peak) / (sse / len(a)))))
PY
)
awk -v p="$psnr" 'BEGIN { exit !(p > 20) }' ||
    fail "low-delay roundtrip PSNR $psnr dB is below 20"

"$BIN" --lowdelay -m s "$WAV" "$TMP/ld_lr.kac" >/dev/null 2>"$TMP/ld.err" ||
    fail "low-delay -m s failed"
grep -q "left/right" "$TMP/ld.err" || fail "low-delay -m s should report left/right"

"$BIN" --lowdelay -m f "$TMP/mono.wav" "$TMP/ldmono.kac" >/dev/null 2>"$TMP/w.err"
if [ $? -ne 0 ] || ! grep -q "mono; -m f has no effect" "$TMP/w.err"; then
    fail "--lowdelay -m f on a mono input should warn"
fi

# A stream can carry every encoder setting the file form can, except a loop
# region and a seek index, which are file-only.
lowdelay_ok() {
    "$BIN" -S --lowdelay "$@" "$WAV" "$TMP/ldm.kac" ||
        { fail "--lowdelay $* should encode"; return; }
    "$BIN" -S --decode "$TMP/ldm.kac" "$TMP/ldm.wav" ||
        fail "--lowdelay $* should decode without a chunk option"
}
lowdelay_ok -V 5
"$BIN" --lowdelay -V 5 "$WAV" "$TMP/ldv.kac" >/dev/null 2>"$TMP/ldv.err" ||
    fail "--lowdelay -V 5 did not report its chunk"
grep -q "chunk 240 samples" "$TMP/ldv.err" ||
    fail "a per-block stream at 5 ms should use 240-sample chunks at 44.1 kHz"
lowdelay_ok --preset low
lowdelay_ok --psnr 45
lowdelay_ok --abr 128
lowdelay_ok --abr-bps 3
lowdelay_ok --tier 3
lowdelay_ok -b 4
lowdelay_ok -B 5
lowdelay_ok -m j
lowdelay_ok --brute 1
lowdelay_ok --brute 2
"$BIN" -S --lowdelay -q 0 "$WAV" "$TMP/ldq.kac" ||
    fail "--lowdelay -q 0 should encode"
refuse --lowdelay 3 -V 5
refuse --lowdelay 3 --tier 4
refuse --chunk-ms 5
refuse --lowdelay --loop 100,200
refuse --lowdelay --stride 5760

# A stream states its chunk count; a count that is not a whole number of the
# chunk's slices is an error.
python3 - "$TMP/ld.kac" "$TMP/bad.kac" <<'PY'
import sys
b = bytearray(open(sys.argv[1], 'rb').read())
b[28:32] = (210).to_bytes(4, 'little')
open(sys.argv[2], 'wb').write(b)
PY
"$BIN" -S --decode "$TMP/bad.kac" "$TMP/x.wav" >/dev/null 2>"$TMP/e"
if [ $? -ne 2 ] || ! grep -q "slices" "$TMP/e"; then
    fail "a chunk count that is not whole slices should exit 2"
fi

# --chunk-ms changes the chunk length the stream states.
"$BIN" --lowdelay --chunk-ms 10 "$WAV" "$TMP/ld10.kac" >/dev/null 2>"$TMP/ld.err" ||
    fail "--lowdelay --chunk-ms 10 failed"
grep -q "chunk 440 samples" "$TMP/ld.err" ||
    fail "--chunk-ms 10 should give a 440-sample chunk at 44.1 kHz"
"$BIN" -S --decode "$TMP/ld10.kac" "$TMP/ld10.wav" || fail "10 ms stream decode failed"

# A tier whose slice does not fit 5 ms at the input's rate.
"$BIN" --lowdelay 1 "$TMP/r2k.wav" "$TMP/x.kac" >/dev/null 2>"$TMP/e"
if [ $? -ne 2 ] || ! grep -q "5 ms chunk" "$TMP/e"; then
    fail "T1 at 2 kHz should exit 2"
fi

# Section 8: the brute search levels.
"$BIN" --tier 4 "$WAV" "$TMP/b0.kac" >/dev/null 2>"$TMP/b.err" || fail "brute: base encode failed"
"$BIN" --tier 4 --brute 1 "$WAV" "$TMP/b1.kac" >/dev/null 2>"$TMP/b.err" ||
    fail "brute 1: encode failed"
"$BIN" --tier 4 --brute 2 "$WAV" "$TMP/b2.kac" >/dev/null 2>"$TMP/b.err" ||
    fail "brute 2: encode failed"
grep -q "brute 2" "$TMP/b.err" || fail "the config line should name brute 2"
cmp -s "$TMP/b0.kac" "$TMP/b1.kac" && fail "brute 1 codes what the default does"
cmp -s "$TMP/b1.kac" "$TMP/b2.kac" && fail "brute 2 codes what brute 1 does"
# A brute setting is a level of its own: -q must not change any part of it.
"$BIN" -S -q 0 --brute 1 "$WAV" "$TMP/bq0.kac" || fail "brute 1 -q 0: encode failed"
"$BIN" -S -q 9 --brute 1 "$WAV" "$TMP/bq9.kac" || fail "brute 1 -q 9: encode failed"
cmp -s "$TMP/bq0.kac" "$TMP/bq9.kac" || fail "--brute 1 still changes with -q"
"$BIN" -S -q 0 --brute 2 "$WAV" "$TMP/bq0.kac" || fail "brute 2 -q 0: encode failed"
"$BIN" -S -q 9 --brute 2 "$WAV" "$TMP/bq9.kac" || fail "brute 2 -q 9: encode failed"
cmp -s "$TMP/bq0.kac" "$TMP/bq9.kac" || fail "--brute 2 still changes with -q"
"$BIN" -S --decode "$TMP/b2.kac" "$TMP/b2.wav" || fail "brute 2: decode failed"
IFS=' ' read -r b2_frames b2_ch <<<"$(frames_of "$TMP/b2.wav")"
if [ "$b2_frames" -ne "$in_frames" ]; then
    fail "brute 2: decoded $b2_frames frames, expected $in_frames"
fi
refuse --brute 0
refuse --brute 3
refuse --brute 99999999999
refuse --decode --brute 1

# Section 9: help exits cleanly, and --longhelp is the README block verbatim.
for opt in --help --usage --longhelp -? --version; do
    "$BIN" "$opt" >/dev/null 2>&1 || fail "$opt: expected exit 0"
done

README=README.md
[ -f "$README" ] || README=../README.md
awk '{ sub(/\r$/, "") }
     /prints the following/ { wait = 1; next }
     wait && /^```$/     { inside = 1; wait = 0; next }
     inside && /^```$/   { exit }
     inside              { print }' "$README" > "$TMP/readme_help"
"$BIN" --longhelp 2> "$TMP/kac_help" >/dev/null
if ! cmp -s "$TMP/readme_help" "$TMP/kac_help"; then
    fail "--longhelp differs from the README block:"
    diff -u "$TMP/readme_help" "$TMP/kac_help" >&2
fi

[ "$FAILS" -eq 0 ] || exit 1
exit 0
