#!/bin/bash
# bench.sh -- CBR table for the README: PSNR, sentinels, encode time, size.
# Optional argument truncates each track to that many seconds. Material is
# supplied through KAC_MATERIAL; missing material is an error.
cd "$(dirname "$0")/.." || exit 1
if [ -z "${KAC_MATERIAL:-}" ]; then
  echo "bench.sh: set KAC_MATERIAL to the directory holding the tracks" >&2
  exit 1
fi
W="$KAC_MATERIAL"
TRACKS="beatit20 croatian20 kissrain20 zuixuan3"
OUT=tests/bench.out
SECS="$1"
if [ -n "$SECS" ]; then LIMIT="--limit $((44100 * SECS))"; else LIMIT=""; fi

missing=""
for f in $TRACKS; do
  [ -f "$W/$f.wav" ] || missing="$missing $f.wav"
done
if [ -n "$missing" ]; then
  echo "bench.sh: missing material in $W:$missing" >&2
  echo "          set KAC_MATERIAL to the directory holding them" >&2
  exit 1
fi

{
  echo "# length: ${SECS:-full}${SECS:+ s}"
  echo "track | tier | bit/sample | PSNR dB | clip % | pred>32767 % | encode s | size B"
} | tee "$OUT"

for f in $TRACKS; do
  for t in 2 3 4 5 6; do
    line=$(./build/kac --tier "$t" $LIMIT "$W/$f.wav" /tmp/bench.kac 2>&1)
    psnr=$(echo "$line"   | sed -n 's/.*PSNR \([0-9.]*\) dB.*/\1/p')
    clip=$(echo "$line"   | sed -n 's/.*clip \([0-9.]*\) %.*/\1/p')
    pover=$(echo "$line"  | sed -n 's/.*32767 \([0-9.]*\) %.*/\1/p')
    rate=$(echo "$line"   | sed -n 's/.*T[0-9] \([0-9.]*\) bit\/sample.*/\1/p')
    size=$(stat -c %s /tmp/bench.kac)
    start=$(date +%s.%N)
    ./build/kac -S --tier "$t" $LIMIT "$W/$f.wav" /tmp/bench.kac
    end=$(date +%s.%N)
    secs=$(echo "$end $start" | awk '{printf "%.3f", $1 - $2}')
    printf "%-11s  T%s  %9s  %8s  %8s  %12s  %8s  %8s\n" \
           "$f" "$t" "$rate" "$psnr" "$clip" "$pover" "$secs" "$size" | tee -a "$OUT"
  done
done
