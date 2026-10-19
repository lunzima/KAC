#!/usr/bin/env python3
"""Generate kac_dqt.h (KAC dequantization tables) and print its SHA-256.

Two tables decide a tier's quantizer: the amplitude ladder M[j] (the magnitudes
inside one scalefactor row) and the step ladder S[s] (how far apart the sixteen
scalefactor rows sit). Both are built here.

The build is switchable so a variant sweep can move one axis at a time; the
defaults reproduce the tree's committed table.

  KAC_DQT_AMP     sea   dqt[s][2j] = round(S[s] * q_j), the per-width grid
                  lin   M[j] = 2j + 1, the linear amplitude rule
  KAC_DQT_LADDER  pow   S[s] = round((s+1)^K_S) (the frozen tree)
                  log   constant ratio between the fixed endpoints
                  alaw  arithmetic at the bottom, constant ratio above it
  KAC_DQT_EXP     the exponent of the power ladder (default 2.75)
  KAC_KS_CAP      the K_S ceiling for the w = 1 tier only (default 3.0)
  KAC_SF          scalefactor rows per table (default 16)

T2 is frozen on both axes: L = 2 means the table IS the quantizer, so there is no
shape left to vary. T1 keeps the linear amplitude rule because the grid's value
for a one-magnitude row is a single number, and doubling it measures worse.

Usage:
  python3 tools/gen_dqt.py [output]      (default kac_dqt.h)
  python3 tools/gen_dqt.py --help
"""
import math
import os
import re
import sys
import hashlib

LIM = 32767
# The id IS the width. N * w = 60 for every tier that fills the 8 byte slice, so
# the rate 64 / N is exactly (64 / 60) * w: ordering by id is ordering by rate.
# T7 (w = 7, n = 8) is the one tier that does not fill the slice, because the
# slice after it would spend more than 8 bit per sample -- the rate of plain
# 8 bit PCM.
TIERS = [(1, 1, 60), (2, 2, 30), (3, 3, 20), (4, 4, 15), (5, 5, 12), (6, 6, 10),
         (7, 7, 8)]

AMP = os.environ.get("KAC_DQT_AMP", "sea")
LADDER = os.environ.get("KAC_DQT_LADDER", "pow")
EXP = float(os.environ.get("KAC_DQT_EXP", "2.75"))
# The w = 1 ceiling. Both ends of the sweep lose: at 2.75 the ladder is pinned
# at the top scalefactor, and at 3.75 the steps are too coarse for quiet
# material. 3.0 is the w = 1 study's pick.
KS_CAP_W1 = float(os.environ.get("KAC_KS_CAP", "3.0"))
# Scalefactor rows per table, i.e. the width of the slice's scalefactor field:
# 4 bit at 16, which is what the format writes. The knob exists so a variant
# sweep can move this one axis on its own; the default reproduces the tree.
SF = int(os.environ.get("KAC_SF", "16"))

# Two tiers keep the linear amplitudes, and one keeps the power ladder. Both
# exceptions are about the width -- so they are derived from it rather than
# written as ids, which a renumbering would move.
#
# w = 2 (L = 2) is frozen on both axes: the table IS the quantiser, so there is
# no shape left to vary.
#
# w = 1 (L = 1) has no amplitude shape to speak of -- the grid's value for that
# row is a single number, so applying it only doubles every step. Measured on
# the one track whose -V 9 file is almost entirely that tier: the linear rule
# beats the doubled one, so the exception is a measurement and not a
# convenience.
AMP_FROZEN = {t for (t, w, n) in TIERS if w <= 2}
LADDER_FROZEN = {t for (t, w, n) in TIERS if w == 2}
USAGE = __doc__

LIMIT_COL = 100        # the tree's clang-format column limit, honoured by the generator
ROW_INDENT = "    { "  # a row opens with four spaces, a brace and a space
CONT_INDENT = "      " # continuation lines: same width, so the grid never shifts
TRAILER = "  /* sf %d */"


def layout(dqt, ncol):
    """Physical lines of one tier, plus the column width used.

    One width W for every column of the tier (the widest value in it), a fixed
    number of elements per line, and breaks only between the (+M, -M) pairs of
    one magnitude. That makes column k of any line start at the very same
    offset, so the table reads as a regular grid."""
    w = max(len(str(v)) for row in dqt for v in row)
#budget : the widest trailer and the closing "}," stay on the last line
    avail = LIMIT_COL - len(TRAILER % (len(dqt) - 1)) - 2

    per_line = 2
    while per_line + 2 <= ncol:
        cand = per_line + 2
        if len(ROW_INDENT) + cand * w + 2 * (cand - 1) <= avail:
            per_line = cand
        else:
            break

    lines = []
    for s, row in enumerate(dqt):
        cells = [str(v).rjust(w) for v in row]
        body = []
        for i in range(0, ncol, per_line):
            prefix = ROW_INDENT if i == 0 else CONT_INDENT
            body.append(prefix + ", ".join(cells[i:i + per_line]))
        for k, text in enumerate(body):
            if k + 1 < len(body):
                lines.append(text + ",")
            else:
                lines.append(text + ("}" if s == len(dqt) - 1 else "},") +
                             (TRAILER % s))
    return lines, w, per_line


def check_layout(dqt, ncol, lines, w, per_line):
    """Assert the grid. Every element of a tier must sit on the same pitch, so
    slicing a line at multiples of the pitch yields clean numbers; each row
    carries the fixed number of elements per line, with a shorter last line, and
    only the last line of a row closes the brace and carries the trailer."""
    pitch = w + 2
    base = len(ROW_INDENT)
    nrow = len(dqt)
    assert len(lines) % nrow == 0, "line count is not a multiple of the rows"
    per_row = len(lines) // nrow
    for r in range(nrow):
        chunk = lines[r * per_row:(r + 1) * per_row]
        counts = []
        for k, line in enumerate(chunk):
            assert len(line) <= LIMIT_COL, "over %d columns: %r" % (LIMIT_COL, line)
            if k == 0:
                assert line.startswith(ROW_INDENT), "row does not open with a brace"
            else:
                assert line.startswith(CONT_INDENT) and "{" not in line, \
                    "continuation line is not plain: %r" % line
            n = 0
            while base + n * pitch + w <= len(line):
                field = line[base + n * pitch:base + n * pitch + w]
                if not re.fullmatch(r" *-?[0-9]+", field):
                    break
                n += 1
            assert n > 0, "line without elements: %r" % line
            tail = line[base + n * pitch - 2:]
            if k + 1 < len(chunk):
                assert tail == ",", "line does not end after its last element: %r" % line
            else:
                closed = "}" if r == nrow - 1 else "},"
                assert tail == closed + (TRAILER % r), \
                    "row does not close with its trailer: %r" % line
            counts.append(n)
        assert sum(counts) == ncol, "row %d has %d elements, want %d" % (r, sum(counts), ncol)
        assert counts[:-1] == [per_line] * (len(counts) - 1), \
            "row %d does not use a fixed fill: %s" % (r, counts)
        assert 0 < counts[-1] <= per_line, "row %d ends with %d elements" % (r, counts[-1])


def rta(x):
    """Round half away from zero."""
    return int(math.floor(x + 0.5)) if x >= 0 else -int(math.floor(-x + 0.5))


def kac_amplitudes(w):
    """The within-row magnitudes M[j] for bit width w: the frozen linear rule."""
    return [2 * j + 1 for j in range(1 << (w - 1))]


def sea_q(w):
    """The within-row amplitude grid for residual_bits = w, pre-rounding.

    start = 0.75, end = (1<<w)-1, and the interior rungs sit on a uniform grid of
    floor((end-start)/(steps-1)). Reading the grid back off a finished table row
    would be wrong: sf[0] = 1 rounds the whole row back to the odd ladder and the
    fractional offset is lost. w = 1 and w = 2 are the two rows special-cased."""
    if w == 1:
        return [2.0]
    if w == 2:
        return [1.115, 4.0]
    steps = 1 << (w - 1)
    start = 0.75
    end = float((1 << w) - 1)
    step_floor = math.floor((end - start) / (steps - 1))
    q = [0.5 + i * step_floor for i in range(steps)]
    q[0] = start
    q[steps - 1] = end
    return q


def amplitudes(w, t):
    if t in AMP_FROZEN:
        return kac_amplitudes(w)
    if AMP == "lin":
        return kac_amplitudes(w)
    assert AMP == "sea", AMP
    return sea_q(w)


def power_ladder(w, Mmax):
    """S[s] = round((s+1)^K_S) with K_S capped so every entry fits in int16."""
    K_S = min(KS_CAP_W1 if w == 1 else EXP, math.log(LIM / Mmax) / math.log(float(SF)))
    cap = LIM // Mmax
    S = []
    for s in range(SF):
        v = rta((s + 1) ** K_S)
        if v < 1:
            v = 1
        if v > cap:
            v = cap
        if s > 0 and v < S[s - 1]:
            v = S[s - 1]
        S.append(v)
    return S, K_S


def log_ladder(w, Mmax):
    """Constant ratio between the same endpoints: the shape the power law should
    have had. S[0] = 1 and S[15] = the power ladder's own S[15], so the span is
    unchanged and only the shape moves."""
    S0, K_S = power_ladder(w, Mmax)
    top = S0[SF - 1]
    S = [max(1, rta(top ** (s / float(SF - 1)))) for s in range(SF)]
    S[0] = 1
    S[SF - 1] = top
    return S, math.log(top) / float(SF - 1)


def alaw_ladder(w, Mmax):
    """A-law type: an arithmetic segment at the bottom, a constant ratio above.

    In A-law the quantiser step is constant while the compressor is linear and
    proportional to the signal after the breakpoint, so the step ladder is the
    inverse companding curve. The classic A = 87.6 puts the breakpoint at
    1/(1 + ln A) = 0.183 of the index range, which is s = 2.74 -> 3. The two
    pieces are joined at S[3] = 4 and the top is the power ladder's own S[15],
    so the span is unchanged."""
    S0, _ = power_ladder(w, Mmax)
    top = S0[SF - 1]
    brk = 3
    join = 1 + brk          # arithmetic from 1 by one per index
    if join >= top:
        return list(S0), 0.0
    ratio = (top / join) ** (1.0 / float(SF - 1 - brk))
    S = []
    for s in range(SF):
        if s <= brk:
            S.append(1 + s)
        else:
            S.append(max(S[-1] + 1, rta(join * ratio ** (s - brk))))
    S[0] = 1
    S[SF - 1] = top
    return S, ratio


def step_ladder(w, t, Mmax):
    if t in LADDER_FROZEN or LADDER == "pow":
        S, K = power_ladder(w, Mmax)
    elif LADDER == "log":
        S, K = log_ladder(w, Mmax)
    else:
        assert LADDER == "alaw", LADDER
        S, K = alaw_ladder(w, Mmax)
    return S, K


def build_table(w, t):
    """Return (M, S, K, dqt) for tier t of bit width w."""
    L = 1 << (w - 1)
    M = amplitudes(w, t)
    Mmax = max(M)
    S, K = step_ladder(w, t, Mmax)
    cap = LIM // Mmax
    for s in range(SF):
        if S[s] < 1:
            S[s] = 1
        if S[s] > cap:
            S[s] = cap
        if s > 0 and S[s] < S[s - 1]:
            S[s] = S[s - 1]
    dqt = []
    for s in range(SF):
        row = []
        for j in range(L):
#the amplitude shape is                 pre - rounding, so the
#product is rounded here and not by the ladder.
            v = rta(S[s] * M[j])
            row.append(v)              # qr = 2j
            row.append(-v)             # qr = 2j+1
        dqt.append(row)
    return M, S, K, dqt


def check(w, t, M, S, dqt):
    """Assert the five table constraints; a violation aborts generation."""
    L = 1 << (w - 1)
    nq = 1 << w
    tag = "T%d(w=%d)" % (t, w)
    assert all(S[i] <= S[i + 1] for i in range(15)), "%s: S not monotonic" % tag
    assert len(dqt) == SF, "%s: row count" % tag
    for s in range(SF):
        row = dqt[s]
        assert len(row) == nq, "%s: row %d width" % (tag, s)
        for j in range(L):
            assert row[2 * j] == -row[2 * j + 1], "%s: row %d symmetry" % (tag, s)
        mags = [abs(row[2 * j]) for j in range(L)]
        for j in range(L - 1):
            assert mags[j] < mags[j + 1], "%s: row %d not increasing" % (tag, s)
        assert len(set(mags)) == L, "%s: row %d has duplicate magnitudes" % (tag, s)
        for v in row:
            assert abs(v) <= LIM, "%s: row %d overflows int16" % (tag, s)
    for j in range(L - 1):
        assert M[j] < M[j + 1], "%s: M not increasing" % tag
    assert len(set(M)) == L, "%s: M has duplicates" % tag
    assert S[SF - 1] * max(M) <= LIM, "%s: S[15] * Mmax over int16" % tag


def payload_hash(tables):
    """SHA-256 over the table payload, int16 little-endian, tier/sf/qr order."""
    h = hashlib.sha256()
    for (t, w, n) in TIERS:
        for s in range(SF):
            for v in tables[w][s]:
                h.update(int(v).to_bytes(2, "little", signed=True))
    return h.hexdigest()


def main():
    args = sys.argv[1:]
    if args and args[0] in ("-h", "--help"):
        print(USAGE)
        return 0
    out = "kac_dqt.h"
    if args:
        if args[0].startswith("-"):
            sys.stderr.write("unknown option %s; see --help\n" % args[0])
            return 2
        if len(args) > 1:
            sys.stderr.write("too many arguments; see --help\n")
            return 2
        out = args[0]

    tables = {}
    print("variants: AMP=%s LADDER=%s EXP=%.3f KS_CAP_W1=%.3f" % (AMP, LADDER, EXP, KS_CAP_W1))
    print("tier  w  L   Mmax   K      S[SF-1] max|dqt|  max(d)")
    for (t, w, n) in TIERS:
        M, S, K, dqt = build_table(w, t)
        check(w, t, M, S, dqt)
        tables[w] = dqt
        mx = max(abs(v) for row in dqt for v in row)
        print("T%-3d  %-2d %-3d %-6d %-7.3f %-6d %-9d %-7d" %
              (t, w, len(M), M[-1], K, S[SF - 1], mx, mx >> 4))
    print("all five constraints hold")

    lines = []
    a = lines.append
    a("/* kac_dqt.h -- generated by tools/gen_dqt.py; do not edit by hand.")
    a(" * Amplitude table M[j] (within a scalefactor row) and step table S[s]")
    a(" * (between rows). Index: kac_dq_t<T>[(sf << w) | qr].")
    a(" * The generator lays the tables out as a regular grid (one column width")
    a(" * per tier, a fixed number of elements per line) and marks them")
    a(" * clang-format off, so regenerating this file reproduces it byte for")
    a(" * byte. The grid is asserted before the file is written. */")
    a("#ifndef KAC_DQT_H")
    a("#define KAC_DQT_H")
    a("")
    a("/* clang-format off */")
    a("")
    for (t, w, n) in TIERS:
        dqt = tables[w]
        ncol = 1 << w
        tbl, width, per_line = layout(dqt, ncol)
        check_layout(dqt, ncol, tbl, width, per_line)
        a("static const int16_t kac_dq_t%d[%d][%d] = {" % (t, SF, ncol))
        for line in tbl:
            a(line)
        a("};")
        a("")
    a("/* clang-format on */")
    a("")
    a("#endif /* KAC_DQT_H */")
    text = "\n".join(lines) + "\n"
# newline="\n" keeps the line ends as written: on Windows the default text mode
# would translate them to CRLF, and the header's own claim that regenerating
# this file reproduces it byte for byte (and the SHA-256 and byte count printed
# just below, both computed on `text`) would stop being true the moment it was
# regenerated there.
    with open(out, "w", newline="\n") as f:
        f.write(text)
    print("wrote %s (%d bytes)" % (out, len(text.encode())))
    print("SHA-256(%s) = %s" % (out, hashlib.sha256(text.encode()).hexdigest()))
    print("SHA-256(payload) = %s" % payload_hash(tables))
    return 0


if __name__ == "__main__":
    sys.exit(main())
