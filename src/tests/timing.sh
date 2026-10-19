#!/bin/bash
# timing.sh -- encode time per tier.
#
#   bash tests/timing.sh        full length (20 s tracks)
#   bash tests/timing.sh 10     first 10 s only: the number to compare with the
#                               10 s budget
#
# These tracks are the measurement basis of the timing figures and are not
# part of the delivered tree: point KAC_MATERIAL at the directory holding them.
# Missing material is an error, never a silent skip.
cd "$(dirname "$0")/.." || exit 1
if [ -z "${KAC_MATERIAL:-}" ]; then
    echo "timing.sh: set KAC_MATERIAL to the directory holding the tracks" >&2
    exit 1
fi
W="$KAC_MATERIAL"
TRACKS="beatit20 croatian20"
SECS="$1"

missing=""
for f in $TRACKS; do
    [ -f "$W/$f.wav" ] || missing="$missing $f.wav"
done
if [ -n "$missing" ]; then
    echo "timing.sh: missing material in $W:$missing" >&2
    echo "           set KAC_MATERIAL to the directory holding them" >&2
    exit 1
fi

if [ -n "$SECS" ]; then
    LIMIT="--limit $((44100 * SECS))"
    echo "== first $SECS s (--limit $((44100 * SECS)) samples per channel) =="
else
    LIMIT=""
    echo "== full length (beatit20/croatian20 = 20 s) =="
fi
for f in $TRACKS; do
  echo "-- $f"
  for t in 2 3 4 5 6; do
    start=$(date +%s.%N)
    ./build/kac -S --tier "$t" $LIMIT "$W/$f.wav" /tmp/timing.kac
    end=$(date +%s.%N)
    printf "  T%s  %s s\n" "$t" "$(echo "$end $start" | awk '{printf "%.3f", $1 - $2}')"
  done
done
