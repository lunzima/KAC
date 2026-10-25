#!/usr/bin/env python3
"""check_dqt.py -- independent layout check for the generated kac_dqt.h.

The tables must form a regular grid. Per tier, with W the width of the widest
value and pitch = W + 2:

  * the element area of every line starts at the same column (6): a row opens
    with "    { ", a continuation line with six spaces;
  * field k of any line occupies columns 6 + k*pitch .. + W and parses as a
    number, so the grid cannot drift between lines;
  * every line carries the same number of fields, the last line of a row may
    carry fewer;
  * no line exceeds the tree's 100 column limit.

usage: python3 tools/check_dqt.py [kac_dqt.h]
"""
import re
import sys

LIMIT_COL = 100
FIRST_COL = 6  # both "    { " and the continuation indent are six wide


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "kac_dqt.h"
    blocks = {}
    tier = None
    for line in open(path):
        m = re.match(r"static const int16_t kac_dq_t(\d)\[16\]\[(\d+)\]", line)
        if m:
            tier = m.group(1)
            blocks[tier] = []
            continue
        if tier and (line.lstrip().startswith("{") or line.startswith("      ")):
            blocks[tier].append(line.rstrip("\n"))

    bad = 0
    for t, rows in blocks.items():
        nums = [int(x) for line in rows for x in re.findall(r"-?[0-9]+", line)]
        width = max(len(str(v)) for v in nums)
        pitch = width + 2
        per_row = len(rows) // 16
        counts = []
        for line in rows:
            assert len(line) <= LIMIT_COL, "over %d columns: %r" % (LIMIT_COL, line)
            head = line[:FIRST_COL]
            if not (head == "    { " or head == " " * FIRST_COL):
                counts = None
                break
            k = 0
            while FIRST_COL + k * pitch + width <= len(line):
                field = line[FIRST_COL + k * pitch:FIRST_COL + k * pitch + width]
                if not re.fullmatch(r" *-?[0-9]+", field):
                    break
                k += 1
            if k == 0:
                counts = None
                break
            counts.append(k)
        if counts is None:
            print("T%s: element area does not start at column %d" % (t, FIRST_COL))
            bad += 1
            continue
        # every row must be filled with the same count, bar a shorter last line
        fills, tails = set(), set()
        for r in range(16):
            row = counts[r * per_row:(r + 1) * per_row]
            if per_row == 1:  # the whole row fits on one line
                fills.add(row[0])
                tails.add(row[0])
                continue
            fills |= set(row[:-1])
            tails.add(row[-1])
        ok = (len(fills) == 1 and
              all(0 < t <= max(fills) for t in tails))
        bad += 0 if ok else 1
        print("T%s: W=%d pitch=%d, first element column %d, %d lines (%d per row), "
              "fill %s, last line %s -> %s"
              % (t, width, pitch, FIRST_COL, len(rows), per_row, sorted(fills),
                 sorted(tails), "OK" if ok else "DRIFT"))
    print("grid check:", "OK" if bad == 0 else "%d tier(s) drifted" % bad)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
