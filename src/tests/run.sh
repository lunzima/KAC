#!/bin/bash
# run.sh -- full acceptance run: functional tests, sanitizers, strict compile,
# implementation review, footprint record.
set -e
cd "$(dirname "$0")/.."
PASS=0
FAIL=0
TESTS="test_roundtrip test_loop test_bad test_seek test_select test_block test_stream"

# The audit and the size step read kac_decode.o, and step 1 runs the test
# binaries: build first if the tree has just been cleaned.
if [ ! -f build/kac_decode.o ] || [ ! -f build/kac_encode.o ] || [ ! -x build/test_roundtrip ] ||
   [ ! -x build/kac ]; then
    echo "(build products missing; running build.sh first)"
    bash build.sh >/dev/null || { echo "FAIL: build.sh failed"; exit 1; }
fi

step() { printf '\n==== %s ====\n' "$*"; }

step "1. functional tests (-O2)"
for t in $TESTS; do
    if "./build/$t" > "/tmp/$t.log" 2>&1; then
        tail -1 "/tmp/$t.log"
        PASS=$((PASS+1))
    else
        echo "FAIL: $t"
        tail -20 "/tmp/$t.log"
        FAIL=$((FAIL+1))
    fi
done

step "2. UBSan"
UB="-std=c99 -Wall -Wextra -pedantic-errors -O1 -g -fsanitize=undefined -fno-sanitize-recover=all"
for t in $TESTS; do
    gcc $UB -DKAC_BUDGET_ASSERT=0 -I. "tests/$t.c" kac_encode.c kac_decode.c -o "build/${t}_ubsan" -lm
    if "./build/${t}_ubsan" > "/tmp/${t}_ubsan.log" 2>&1; then
        echo "  ok  $t"
        PASS=$((PASS+1))
    else
        echo "FAIL: $t (UBSan)"
        grep -iE "runtime error|FAIL" "/tmp/${t}_ubsan.log" | head -10
        FAIL=$((FAIL+1))
    fi
done

step "3. ASan + UBSan"
AS="-std=c99 -Wall -Wextra -pedantic-errors -O1 -g -fsanitize=address,undefined -fno-sanitize-recover=all"
for t in $TESTS; do
    gcc $AS -DKAC_BUDGET_ASSERT=0 -I. "tests/$t.c" kac_encode.c kac_decode.c -o "build/${t}_asan" -lm
    if "./build/${t}_asan" > "/tmp/${t}_asan.log" 2>&1; then
        echo "  ok  $t"
        PASS=$((PASS+1))
    else
        echo "FAIL: $t (ASan)"
        grep -iE "ERROR|runtime error|FAIL" "/tmp/${t}_asan.log" | head -10
        FAIL=$((FAIL+1))
    fi
done

step "4. strict C99 compile"
# kac.c pulls in wav_io.h, so these three translation units cover every header.
if gcc -std=c99 -Wall -Wextra -pedantic-errors -O2 -I. -c kac_decode.c -o /tmp/strict_d.o 2>/tmp/strict.log &&
   gcc -std=c99 -Wall -Wextra -pedantic-errors -O2 -I. -c kac_encode.c -o /tmp/strict_e.o 2>>/tmp/strict.log &&
   gcc -std=c99 -Wall -Wextra -pedantic-errors -O2 -I. -c kac.c -o /tmp/strict_c.o 2>>/tmp/strict.log; then
    if [ -s /tmp/strict.log ]; then
        echo "FAIL: warnings"; cat /tmp/strict.log; FAIL=$((FAIL+1))
    else
        echo "  ok  no warnings"; PASS=$((PASS+1))
    fi
else
    echo "FAIL: compile error"; cat /tmp/strict.log; FAIL=$((FAIL+1))
fi

step "5. implementation review"
python3 tests/audit_objdump.py build/kac_decode.o && PASS=$((PASS+1)) || FAIL=$((FAIL+1))

step "6. code size and footprint (recorded, not asserted)"
# These are measurements of one implementation, not requirements of the format,
# so they are reported rather than turned into a pass/fail. The sample-loop
# figures are printed by the audit (step 5).
# --radix=d keeps this in plain decimal: parsing hex would need gawk's strtonum.
DEC_SIZE=$(nm --print-size --radix=d build/kac_decode.o | awk '/ kacdec_decode$/ {print $2 + 0}')
echo "  kacdec_decode  $DEC_SIZE B"
size -A build/kac_decode.o | awk '$1 == ".text" {printf "  .text total     %d B\n", $2}'
size -A build/kac_decode.o | awk '$1 == ".rodata" {printf "  .rodata tables  %d B\n", $2}'
cat > /tmp/kac_size.c <<'EOF'
#include <stdio.h>
#include "kac.h"
int main(void) {
    printf("  kac_ch %zu B, stereo LMS state %zu B, kacdec_t %zu B, kac_t %zu B\n",
           sizeof(kac_ch), 2 * sizeof(kac_ch), sizeof(kacdec_t), sizeof(kac_t));
    return 0;
}
EOF
gcc -std=c99 -I. /tmp/kac_size.c -o /tmp/kac_size && /tmp/kac_size

step "7. row 5 empirical leg: UBSan over the adversarial set"
# The overflow check must also show that both adversarial cases ran.
if ./build/test_roundtrip_ubsan > /tmp/adversarial.log 2>&1 &&
   grep -q "out of range snapshot weights are saturated" /tmp/adversarial.log &&
   grep -q "sig3" /tmp/adversarial.log; then
    echo "  ok  snapshot w=32767 and the full-scale residual stream, zero UBSan reports"
    PASS=$((PASS+1))
else
    echo "FAIL: the adversarial set did not run clean under UBSan"
    grep -iE "runtime error|FAIL" /tmp/adversarial.log | head -5
    FAIL=$((FAIL+1))
fi

step "8. command line smoke"
# build/kac is not covered by the test binaries; cli.sh checks its front end.
if bash tests/cli.sh; then
    echo "  ok  options, config line, header fields, help/README"
    PASS=$((PASS+1))
else
    FAIL=$((FAIL+1))
fi

printf '\n==== %d passed, %d failed ====\n' "$PASS" "$FAIL"
[ "$FAIL" -eq 0 ]
