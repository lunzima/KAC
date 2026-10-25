#!/bin/bash
# build.sh -- build everything (C99).
#   bash build.sh              kac binary and the seven test programs
#   bash build.sh --sanitize   same, plus tests/*_asan with ASan+UBSan
#   bash build.sh --format     reformat the sources (CLANG_FORMAT, default
#                              clang-format-18)
#   bash build.sh --clean      remove build products
set -e
cd "$(dirname "$0")"

CFLAGS="-std=c99 -Wall -Wextra -pedantic-errors -O3"
LDLIBS="-lm"
TESTS="test_roundtrip test_loop test_bad test_seek test_select test_block test_stream"
BUILD="build"
CLANG_FORMAT="${CLANG_FORMAT:-clang-format-18}"
FMT_SRC="kac.c kac_encode.c kac_decode.c kac.h kac_internal.h kac_dqt.h wav_io.h
         tests/test_roundtrip.c tests/test_loop.c tests/test_bad.c
         tests/test_seek.c tests/test_select.c tests/test_block.c tests/test_stream.c
         tests/kac_test_util.h tools/calibrate_v.c"

if [ "$1" = "--clean" ]; then
    rm -rf "$BUILD" kac_dqt.h.tmp
    echo "cleaned"
    exit 0
fi

if [ "$1" = "--format" ]; then
    # shellcheck disable=SC2086
    "$CLANG_FORMAT" -i $FMT_SRC
    # shellcheck disable=SC2086
    "$CLANG_FORMAT" --dry-run --Werror $FMT_SRC
    echo "formatted; sources match the style"
    exit 0
fi

echo "[1/3] dequantization tables"
python3 tools/gen_dqt.py kac_dqt.h

mkdir -p "$BUILD"
echo "[2/3] kac binary and tests"
# shellcheck disable=SC2086
gcc $CFLAGS -I. -c kac_encode.c -o "$BUILD/kac_encode.o"
gcc $CFLAGS -I. -c kac_decode.c -o "$BUILD/kac_decode.o"
gcc $CFLAGS -I. -o "$BUILD/kac" kac.c "$BUILD/kac_encode.o" "$BUILD/kac_decode.o" $LDLIBS
for t in $TESTS; do
    gcc $CFLAGS -I. -o "$BUILD/$t" "tests/$t.c" "$BUILD/kac_encode.o" "$BUILD/kac_decode.o" $LDLIBS
done

if [ "$1" = "--sanitize" ]; then
    echo "[3/3] sanitizer copies"
    # shellcheck disable=SC2086
    for t in $TESTS; do
        gcc -std=c99 -Wall -Wextra -pedantic-errors -O1 -g \
            -fsanitize=undefined,address -fno-sanitize-recover=all \
            -I. -o "$BUILD/${t}_asan" "tests/$t.c" kac_encode.c kac_decode.c $LDLIBS
    done
    echo "done; run: bash tests/run.sh"
else
    echo "[3/3] sanitizer copies skipped (use --sanitize)"
    echo "done; run: bash tests/run.sh"
fi
