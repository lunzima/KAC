#!/bin/bash
# deadcode.sh -- check for dead code, leftover experiment switches and unused
# declarations before shipping.
cd "$(dirname "$0")/.." || exit 1

echo "=== 1. strict compile (unused, missing prototypes, shadowing) ==="
FLAGS="-std=c99 -Wall -Wextra -pedantic-errors -Wunused-function -Wunused-parameter
       -Wmissing-prototypes -Wstrict-prototypes -Wshadow -Wcast-qual -Wwrite-strings
       -O2 -I."
# shellcheck disable=SC2086
FLAGS=$(echo $FLAGS)
for f in kac_encode.c kac_decode.c kac.c tests/test_roundtrip.c tests/test_loop.c \
         tests/test_bad.c tests/test_seek.c tests/test_select.c tests/test_block.c \
         tests/test_stream.c tools/calibrate_v.c; do
    # shellcheck disable=SC2086
    if gcc $FLAGS -c "$f" -o /tmp/dc.o 2>/tmp/dc.log; then
        if [ -s /tmp/dc.log ]; then echo "  [$f] warnings:"; cat /tmp/dc.log;
        else echo "  [$f] clean"; fi
    else
        echo "  [$f] compile failed:"; cat /tmp/dc.log
    fi
done

echo
echo "=== 2. experiment switches and leftover markers (expect nothing) ==="
PAT='g_mant|g_Rmax|noclamp|TODO|FIXME|XXX|#if 0'
grep -nE "$PAT" kac_encode.c kac_decode.c kac_internal.h kac.h kac.c \
     wav_io.h Makefile build.sh 2>/dev/null || echo "  none"
# tools/gen_dqt.py intentionally reads variant switches from the environment;
# no other delivered file may call getenv.
grep -nE 'getenv|environ' kac_encode.c kac_decode.c kac_internal.h kac.h kac.c \
     wav_io.h 2>/dev/null || echo "  none (no environment read outside the generator)"

echo
echo "=== 3. non ASCII characters in the delivered tree ==="
# Every file, not a list of suffixes: the rule is about the delivered tree, and
# a suffix list would miss whatever carries none. Only the two directories that
# are not part of it are stepped over.
if bad=$(LC_ALL=C grep -rIl '[^[:print:][:space:]]' . --exclude-dir=.git --exclude-dir=build); then
    echo "  FAIL: non ASCII content found"
    printf '%s\n' "$bad" | head -20
else
    rc=$?
    if [ "$rc" -gt 1 ]; then
        echo "  FAIL: the scan failed (grep exit $rc)"
    else
        echo "  none"
    fi
fi

echo
echo "=== 4. static functions and their call counts ==="
for f in kac_encode.c kac_decode.c; do
    echo "-- $f"
    grep -oE '^static [A-Za-z_0-9 ]*\**([a-z_0-9]+)\(' "$f" |
        sed -E 's/.*[ *]([a-z_0-9]+)\($/\1/' | sort -u | while read -r fn; do
            n=$(grep -cE "[^A-Za-z_0-9]$fn *\(" "$f")
            printf "     %-20s %s occurrences\n" "$fn" "$n"
        done
done
