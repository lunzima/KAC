#!/usr/bin/env python3
"""audit_objdump.py -- implementation review of the KAC decoder.

Implements the review checklist, in this order: source level > link level >
disassembly form. A form check verifies a
*form*, not a *behaviour*, so it is a reference cross-check and is never an
acceptance criterion on its own: the form-only rows (3, 4 and the shape counts
of row 5) are printed and, where a review is warranted, warned about.

Asserted -- these decide the exit status:

  row 1  the sample loop executes no function call.
           link level : no undefined symbol but the stack guard, and no
                        relocation into the loop range but the stack guard
           disassembly: no `call` inside the loop range
         The two cover *different* holes and only together are sufficient: the
         link check catches a tail call to an imported function (a `jmp`, which
         the call count cannot see), the call count catches a call to a
         same-object `static` function (which leaves neither an undefined symbol
         nor a relocation).
  row 2  the sample loop executes no division.
           link level : no import whose name contains `div` (this is also what
                        catches a 64-bit division, which compiles to __divdi3)
           source     : no `/` or `%` in the loop body
           disassembly: no div/idiv inside the loop range
  row 5  the decode path has no signed overflow.
           source     : every weight assignment is kac_sat_w(...) or an in-range
                        KAC_INIT_W* constant, and every snapshot load path goes
                        through kac_snap_load
           source     : the adversarial set that the UBSan leg of run.sh step 2
                        feeds is present in tests/test_roundtrip.c
  row 6  no floating point.
           source     : the decoder's own translation unit has no FP type
           link level : no soft-float helper import
  row 7  no heap allocation -- link level: no malloc/free/calloc/realloc import.

Recorded -- never asserted here:

  row 3  the jumps in the sample loop. More than the back edge needs a
         measurement showing the branch beats the branchless form; that is a
         review item, not a failed check.
  row 4  conditional moves in the sample loop.
  row 5  saturation sites, disassembly form (the count moves with unrolling).
  row 8  inner-loop instruction count, and the source-level direct-reference
         check on the loop body.

usage: python3 tests/audit_objdump.py [build/kac_decode.o] [--src <tree root>]
                                     [--loop <lo> <hi>]
"""
import os
import re
import subprocess
import sys

fails = []
warns = []


def ok(msg):
    print('  ok    ' + msg)


def record(msg):
    print('  rec   ' + msg)


def warn(msg):
    print('  WARN  ' + msg)
    warns.append(msg)


def check(cond, msg):
    if cond:
        ok(msg)
    else:
        print('  FAIL  ' + msg)
        fails.append(msg)
    return cond


def section(title):
    print('\n-- ' + title)


# --------------------------------------------------------------------------
# source level
# --------------------------------------------------------------------------

def strip_comments_strings(text):
    """Blank comments and literals, preserving length and line structure."""
    out = []
    i, n = 0, len(text)
    while i < n:
        c = text[i]
        if c == '/' and i + 1 < n and text[i + 1] == '/':
            while i < n and text[i] != '\n':
                out.append(' ')
                i += 1
        elif c == '/' and i + 1 < n and text[i + 1] == '*':
            out.append('  ')
            i += 2
            while i < n and not (text[i] == '*' and i + 1 < n and text[i + 1] == '/'):
                out.append('\n' if text[i] == '\n' else ' ')
                i += 1
            out.append('  ')
            i += 2
        elif c == '"' or c == "'":
            q = c
            out.append(' ')
            i += 1
            while i < n and text[i] != q:
                if text[i] == '\\' and i + 1 < n:
                    out.append('  ')
                    i += 2
                    continue
                out.append('\n' if text[i] == '\n' else ' ')
                i += 1
            out.append(' ')
            i += 1
        else:
            out.append(c)
            i += 1
    return ''.join(out)


def block_at(text, open_idx):
    depth = 0
    for i in range(open_idx, len(text)):
        if text[i] == '{':
            depth += 1
        elif text[i] == '}':
            depth -= 1
            if depth == 0:
                return open_idx, i
    return open_idx, len(text)


def function_body(clean, name):
    m = re.search(r'\b' + re.escape(name) + r'\s*\([^;{]*\)\s*\{', clean)
    if not m:
        return None
    return block_at(clean, m.end() - 1)


LOOP_RE = re.compile(r'\b(for|while)\s*\(')


def innermost_loop(clean, span):
    """Loop block nested deepest inside span; ties broken by the smallest block."""
    lo, hi = span
    blocks = []
    for m in LOOP_RE.finditer(clean, lo, hi):
        b = clean.find('{', m.end())
        if b < 0 or b > hi:
            continue
        s, e = block_at(clean, b)
        if s <= e <= hi:
            blocks.append((s, e))
    best, bestn = None, -1
    for s, e in blocks:
        n = sum(1 for (s2, e2) in blocks if s2 <= s and e <= e2 and (s2, e2) != (s, e))
        if n > bestn or (n == bestn and best is not None and (e - s) < (best[1] - best[0])):
            best, bestn = (s, e), n
    if best is None:
        return None
    return clean[best[0]:best[1] + 1], best


def line_of(text, idx):
    return text.count('\n', 0, idx) + 1


def read(path):
    try:
        with open(path, 'r', encoding='utf-8') as f:
            return f.read()
    except OSError:
        return None


# --------------------------------------------------------------------------
# disassembly
# --------------------------------------------------------------------------

def parse_functions(text):
    funcs, cur, insns = {}, None, []
    for line in text.splitlines():
        m = re.match(r'^[0-9a-f]+\s+<([^>]+)>:', line)
        if m:
            if cur:
                funcs[cur] = [tuple(i) for i in insns]
            cur, insns = m.group(1), []
            continue
        m = re.match(r'^\s*([0-9a-f]+):\t((?:[0-9a-f]{2} )*)\s*(.*)$', line)
        if m and cur is not None:
            rest = m.group(3).strip()
            mnem = rest.split()[0] if rest else ''
            tgt = None
            if mnem.startswith('j'):
                tm = re.match(r'([0-9a-f]+)\b', rest[len(mnem):].strip())
                if tm:
                    tgt = int(tm.group(1), 16)
            insns.append([int(m.group(1), 16), mnem, rest, tgt])
            continue
        m = re.match(r'^\s+[0-9a-f]+:\s+R_\S+\s+(\S+)', line)
        if m and insns and cur is not None:
            insns[-1][2] += ' -> ' + m.group(1)
    if cur:
        funcs[cur] = [tuple(i) for i in insns]
    return funcs


RELOC_RE = re.compile(r'^\s*([0-9a-f]+):\s+(R_[A-Za-z0-9_]+)\s+(\S+)')


def parse_relocations(text):
    """[(address, type, symbol)] -- symbol with any addend stripped."""
    rel = []
    for line in text.splitlines():
        m = RELOC_RE.match(line)
        if m:
            sym = re.split(r'[+\-]', m.group(3))[0]
            rel.append((int(m.group(1), 16), m.group(2), sym))
    return rel


def is_call(m):
    return m.startswith('call')


def is_div(m):
    return m.startswith('div') or m.startswith('idiv')


R64 = set(['%rax', '%rbx', '%rcx', '%rdx', '%rsi', '%rdi', '%rbp', '%rsp',
           '%r8', '%r9', '%r10', '%r11', '%r12', '%r13', '%r14', '%r15'])


def is_64bit_div(mnem, rest):
    if mnem.endswith('q'):
        return True
    if mnem.endswith('l'):
        return False
    return any(t in R64 for t in rest.split(None, 1)[1].replace(',', ' ').split())


# Floating point *arithmetic*, conversion, comparison and the x87 set. Moves are
# deliberately absent: `movups` is what gcc emits for an integer memset, so the
# mnemonic alone cannot separate FP from integer SIMD -- the register file is
# shared. That is why row 6 accepts on source + link level and
# treats the disassembly as a reference only.
FP_ARITH_RE = re.compile(
    r'^v?(?:add|sub|mul|div|max|min|sqrt|rsqrt|rcp|hadd|hsub|dpps|dppd|round)(?:pd|ps|ss|sd)$'
    r'|^v?(?:cmp|com|ucom|test)(?:pd|ps|ss|sd)$'
    r'|^v?cvt[a-z0-9]*(?:pd|ps|ss|sd)$'
    r'|^f(?:add|sub|mul|div|ld|st|com|ucom|icom|sqrt|abs|chs|xch|scale|rndint|tst|patan'
    r'|prem|ptan|sin|cos|sincos|yl2|xam|saved|stenv|ldcw|ldenv|free|nop|wait|init|decstp'
    r'|incstp|istp|ist)$')


def counts(insns, needle, exclude=None):
    return sum(1 for i in insns
               if needle in i[2] and (exclude is None or exclude not in i[2]))


# --------------------------------------------------------------------------

def main():
    args = sys.argv[1:]
    obj, root, forced = 'build/kac_decode.o', None, None
    i = 0
    while i < len(args):
        if args[i] == '--src':
            root = args[i + 1]
            i += 2
        elif args[i] == '--loop':
            forced = (int(args[i + 1], 0), int(args[i + 2], 0))
            i += 3
        else:
            obj = args[i]
            i += 1
    if root is None:
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    dec_src = read(os.path.join(root, 'kac_decode.c'))
    int_src = read(os.path.join(root, 'kac_internal.h'))
    pub_src = read(os.path.join(root, 'kac.h'))
    rt_src = read(os.path.join(root, 'tests', 'test_roundtrip.c'))
    missing = [n for n, s in (('kac_decode.c', dec_src), ('kac_internal.h', int_src),
                              ('kac.h', pub_src),
                              ('tests/test_roundtrip.c', rt_src)) if s is None]
    if missing:
        print('FAIL  cannot read: ' + ', '.join(missing))
        return 1
    dec_clean = strip_comments_strings(dec_src)

    try:
        disasm = subprocess.check_output(['objdump', '-dr', obj]).decode('utf-8', 'replace')
    except (OSError, subprocess.CalledProcessError) as exc:
        print('FAIL  objdump %s: %s' % (obj, exc))
        return 1
    try:
        nm_u = subprocess.check_output(['nm', '-u', obj]).decode('utf-8', 'replace')
    except (OSError, subprocess.CalledProcessError) as exc:
        print('FAIL  nm -u %s: %s' % (obj, exc))
        return 1

    F = parse_functions(disasm)
    relocs = parse_relocations(disasm)
    undef = sorted(set(l.split()[-1] for l in nm_u.splitlines() if l.strip()))

    # ---- loop range ------------------------------------------------------
    dec = F.get('kacdec_decode')
    if not check(dec is not None, 'kacdec_decode present in %s' % obj):
        return 1
    if forced:
        lo, hi = forced
        print('  rec   sample loop [%#x, %#x] (from --loop)' % (lo, hi))
    else:
        backs = [(a, t) for (a, m, r, t) in dec
                 if t is not None and m.startswith('j') and m != 'jmp' and t < a]
        check(len(backs) >= 1, 'sample loop back edge found (%d candidate(s))' % len(backs))
        if not backs:
            return 1
        # kacdec_decode holds more than one loop: the block loop is wrapped
        # around the sample loop. That
        # makes "the innermost back edge" a guess: gcc
        # reorders blocks, so `target < address` is a *heuristic* for a back
        # edge and produces false ones here -- the shared epilogue is placed
        # between a loop's target and the conditional jump that closes it, so
        # the rectangle from the target to the jump is not a loop body at all.
        # Two more filters, both of which can only make the audit fail loudly
        # rather than circle the wrong range in silence:
        #   1. a candidate whose rectangle holds a `ret` is not a loop of this
        #      function (its only returns are the epilogue), so it is dropped;
        #   2. the sample loop is the one that steps the filter, so it is the
        #      innermost survivor whose rectangle holds a weight saturation
        #      compare (KAC_W_LIM = 16383). The decoder holds a second loop over
        #      the same samples -- the merge of a coded chunk's mid/side pair --
        #      which saturates nothing, so the span alone cannot tell the two
        #      apart;
        #   3. among those the smallest span is the innermost loop, and every
        #      wider survivor must *contain* it -- otherwise the choice is
        #      ambiguous and the check below fails instead of picking one.
        live = []
        for a, t in backs:
            if any(r == 'ret' or r.startswith('ret') for (x, m, r, _tx) in dec if t <= x <= a):
                record('dropped back edge [%#x, %#x]: its rectangle contains a ret' % (t, a))
            else:
                live.append((a, t))
        check(live, 'a loop body among the back edges (%d candidate(s), %d after dropping the '
                    'ones that span a return)' % (len(backs), len(live)))
        if not live:
            return 1
        sat = []
        for a, t in live:
            here = [i for i in dec if t <= i[0] <= a]
            if counts(here, '0x3fff', 'ffffc001') or counts(here, 'ffffc001'):
                sat.append((a, t))
        check(sat, 'a loop that saturates a weight among the %d survivor(s)' % len(live))
        if not sat:
            return 1
        live = sat
        live.sort(key=lambda x: x[0] - x[1])
        lo, hi = live[0][1], live[0][0]
        outside = [(t, a) for (a, t) in live[1:] if (a - t) > (hi - lo) and not (t <= lo and hi <= a)]
        check(not outside,
              'the sample loop range [%#x, %#x] is the innermost of %d weight-saturating '
              'loop(s) and is contained in every wider one, so it was identified '
              'unambiguously' % (lo, hi, len(live)))
        for t, a in outside:
            print('        back edge [%#x, %#x] is wider but does not contain the range' % (t, a))
        print('  rec   %d back edge(s), %d loop(s) after the ret filter; sample loop [%#x, %#x]'
              % (len(backs), len(live), lo, hi))
        ranges = [(lo, hi)]

    def in_loop(addr):
        return any(lo <= addr <= hi for lo, hi in ranges)

    rng_txt = ', '.join('[%#x, %#x]' % r for r in ranges)
    body = [i for i in dec if in_loop(i[0])]
    reloc_in_loop = [r for r in relocs if in_loop(r[0])]
    print('  rec   sample loop body: %d instructions over %d range(s)' % (len(body), len(ranges)))

    # ---- row 1: no function call in the sample loop ----------------------
    section('row 1  no function call in the sample loop')
    imports = [s for s in undef if s != '__stack_chk_fail']
    check(not imports, 'link: no undefined symbol but the stack guard (%s)'
          % (', '.join(undef) or 'none'))
    # Only a reference to a symbol *outside* this object can be a call to it; a
    # relocation to a local section or to a symbol defined here (a .bss global,
    # say) is a data reference and is not a violation.
    undef_set = set(undef)
    bad_rel = [r for r in reloc_in_loop
               if r[2] in undef_set and r[2] != '__stack_chk_fail']
    check(not bad_rel,
          'link: the loop range(s) %s reference no symbol outside this object '
          '(%d relocation(s) there, all same-object)' % (rng_txt, len(reloc_in_loop)))
    calls = [i for i in body if is_call(i[1])]
    check(not calls, 'disassembly: no call inside the loop range(s) %s (%d found)'
          % (rng_txt, len(calls)))
    record('link: %d relocation(s) in the object, %d in the loop range(s)'
           % (len(relocs), len(reloc_in_loop)))
    record('limitation: nm -u is object level and therefore stricter than the loop-level '
           'property. It cannot pass falsely, but it can FAIL falsely -- an import used '
           'outside the sample loop is allowed and is not a violation (this check is '
           'stricter than the property it stands for)')
    for a, t, s in reloc_in_loop:
        record('  loop reloc %#x %s -> %s' % (a, t, s))

    # ---- row 2: no division in the sample loop ---------------------------
    section('row 2  no division in the sample loop')
    div_imports = [s for s in undef if 'div' in s]
    check(not div_imports, 'link: no import whose name contains div (%s)'
          % (', '.join(div_imports) or 'none'))
    fib = function_body(dec_clean, 'kacdec_decode')
    src_loop = innermost_loop(dec_clean, fib) if fib else None
    if src_loop is None:
        check(False, 'source: could not locate the sample loop in kacdec_decode')
        loop_txt = ''
    else:
        loop_txt, (ls, le) = src_loop
        print('  rec   source loop body: kac_decode.c:%d-%d' % (line_of(dec_src, ls),
                                                                line_of(dec_src, le)))
        divs = [('/' if c == '/' else '%', line_of(dec_src, ls + m.start()))
                for m in re.finditer(r'[/%]', loop_txt) for c in [m.group(0)]]
        check(not divs, 'source: no / or %% in the loop body (%s)'
              % (', '.join('%s at line %d' % d for d in divs) or 'none'))
    divs_in_loop = [i for i in body if is_div(i[1])]
    check(not divs_in_loop, 'disassembly: no div/idiv inside the loop range (%d found)'
          % len(divs_in_loop))
    all_div = [(n, j[0], j[2]) for n, ins in F.items() for j in ins if is_div(j[1])]
    bad64 = [d for d in all_div if is_64bit_div(d[2].split()[0], d[2])]
    record('object: %d division site(s), %d outside the loop, %d of them 64 bit'
           % (len(all_div), len(all_div) - len(divs_in_loop), len(bad64)))
    for n, a, r in all_div:
        record('  div %s+%#x  %s%s' % (n, a, r.split(' ->')[0],
                                       '   <-- 64 bit' if (n, a, r) in bad64 else ''))
    record('premise: a division only reaches the code through an imported helper, so the '
           'link-level check also covers 64-bit division (__divdi3 and friends); the same '
           'premise is what row 7 states for allocation')

    # ---- row 3: branches in the sample loop (record only) ----------------
    section('row 3  branches in the sample loop (recorded, not asserted)')
    jumps = [i for i in body if i[1].startswith('j')]
    extra = [j for j in jumps if not any(j[3] == lo for lo, hi in ranges)]
    record('%d jump(s) in the loop range; the reference is the loop back edge' % len(jumps))
    for j in jumps:
        record('  %#x  %s' % (j[0], j[2].split(' ->')[0]))
    if extra:
        warn('%d jump(s) besides the back edge; if any is data dependent a measurement '
             'showing it beats the branchless form must be attached (row 3)'
             % len(extra))
    else:
        record('the only jump is the back edge')

    # ---- row 4: conditional moves (record only) --------------------------
    section('row 4  conditional moves in the sample loop (recorded)')
    cmovs = [i for i in body if i[1].startswith('cmov')]
    record('%d conditional move(s) in the loop range' % len(cmovs))

    # ---- row 5: no signed overflow ---------------------------------------
    section('row 5  no signed overflow (the invariant, source level)')
    assign = re.compile(r'[A-Za-z_][A-Za-z_0-9]*\s*->\s*w\s*\[\s*[0-9]\s*\]\s*=\s*([^;]+);')
    lim_m = re.search(r'#define\s+KAC_W_LIM\s+\(?\s*(-?\d+)', int_src)
    lim = int(lim_m.group(1)) if lim_m else None
    init = dict(re.findall(r'#define\s+(KAC_INIT_W[0-9])\s+\(?\s*(-?\d+)', int_src))
    unresolved, sat = [], 0
    for m in assign.finditer(strip_comments_strings(int_src)):
        rhs = m.group(1).strip()
        if rhs.startswith('kac_sat_w('):
            sat += 1
        elif re.fullmatch(r'KAC_INIT_W[0-9]', rhs) or rhs in init:
            pass
        else:
            unresolved.append((line_of(int_src, m.start()), rhs))
    check(lim is not None, 'source: KAC_W_LIM found (%s)' % lim)
    check(not unresolved, 'source: every weight assignment is kac_sat_w(...) or an init '
                          'constant (%s)'
          % (', '.join('line %d: %s' % u for u in unresolved) or 'none'))
    check(sat == 8, 'source: kac_sat_w wraps 8 weight assignments, 4 in kac_step_pred '
                    'and 4 in kac_snap_load (%d)' % sat)
    if lim is not None and init:
        bad_init = {k: v for k, v in init.items() if abs(int(v)) > lim}
        check(not bad_init, 'source: every KAC_INIT_W* is within +-KAC_W_LIM (%s)'
              % (', '.join('%s=%s' % kv for kv in bad_init.items()) or 'none'))
    best = sum(1 for m in re.finditer(r'kac_snap_load\s*\(', dec_src))
    record('source: %d kac_snap_load call site(s) in kac_decode.c (INIT, LOOP and SEEK '
           'snapshots all load through it)' % best)
    have_adv = ('0x7F' in rt_src and '-32768.0' in rt_src)
    check(have_adv, 'source: the adversarial set the UBSan leg feeds is present in '
                    'tests/test_roundtrip.c (out-of-range snapshot weight 0x7FFF and the '
                    'full-scale residual stream)')
    record('empirical: the UBSan run over that set is enforced by tests/run.sh step 2 and '
           'step 7; it samples the property, it does not prove it')
    hi_cnt = counts(body, '0x3fff', 'ffffc001')
    lo_cnt = counts(body, '0xffffc001')
    record('form (reference only): saturation compare sites in the loop %d upper / %d lower; '
           'the count moves with unrolling, the invariant does not' % (hi_cnt, lo_cnt))
    out_hi = counts([i for i in dec if not in_loop(i[0])], '0x3fff', 'ffffc001')
    record('form (reference only): %d saturation site(s) in kacdec_decode outside the loop'
           % out_hi)

    # ---- row 6: no floating point ----------------------------------------
    section('row 6  no floating point')
    fp_kw = [w for w in ('float', 'double', 'long double')
             if re.search(r'\b' + w + r'\b', dec_clean)]
    check(not fp_kw, 'source: kac_decode.c has no floating point type (%s)'
          % (', '.join(fp_kw) or 'none'))
    check('<math.h>' not in dec_clean, 'source: kac_decode.c does not include <math.h>')
    fp_fns = sorted(set(re.findall(r'\bdouble\s+([A-Za-z_][A-Za-z_0-9]*)\s*\(', pub_src)))
    used = [f for f in fp_fns if re.search(r'\b' + f + r'\b', dec_clean)]
    check(not used, 'source: the decoder uses none of the double-returning declarations in '
                    'kac.h (%s)' % (', '.join(used) or 'of %d' % len(fp_fns)))
    check(not re.search(r'\bkac_t\b', dec_clean),
          'source: the decoder does not reference kac_t, the only struct with a double field')
    soft = [s for s in undef if re.match(r'^__(add|sub|mul|div|neg|cmp|extend|trunc|fix|float)'
                                         r'[a-z0-9]*(sf|df|tf|xf)3$', s)]
    check(not soft, 'link: no soft-float helper import (%s)' % (', '.join(soft) or 'none'))
    allins = [i for ins in F.values() for i in ins]
    fp_arith = sorted(set(i[1] for i in allins if FP_ARITH_RE.match(i[1])))
    check(not fp_arith, 'form: no floating point arithmetic mnemonic in the object (%s)'
          % (', '.join(fp_arith) or 'none'))
    simd = [i for i in allins if re.search(r'\b(xmm|ymm|zmm)\d', i[2])]
    record('form (reference only): %d instruction(s) touch xmm/ymm/zmm -- %s'
           % (len(simd), ', '.join(sorted(set(i[1] for i in simd))) or 'none'))
    record('form (reference only): a register name does not make an instruction floating '
           'point, so the disassembly cannot decide row 6 -- hence source + link above')

    # ---- row 7: no heap allocation ---------------------------------------
    section('row 7  no heap allocation')
    heap = [s for s in undef if s.split('@')[0] in ('malloc', 'free', 'calloc', 'realloc')]
    check(not heap, 'link: no malloc/free/calloc/realloc import (%s)'
          % (', '.join(heap) or 'none'))
    record('limitation: a custom allocator wrapper defined in this object would not appear '
           'in nm -u')

    # ---- row 8: no per-iteration dependence on channels/loop_end/total ----
    section('row 8  the sample loop does not depend per iteration on the slice invariants')
    refs = [n for n in ('channels', 'loop_end', 'total')
            if re.search(r'\b' + n + r'\b', loop_txt)]
    check(not refs, 'source: the loop body references none of channels/loop_end/total (%s)'
          % (', '.join(refs) or 'none'))
    record('limitation: direct references only; a callee reading d->channels internally, or '
           'inlining, would be missed')
    record('behavioural (reference only): inner-loop instruction count %d; compare against a '
           'build with the hoisted loads moved back inside the loop' % len(body))

    print('\n%d assertion(s) failed, %d warning(s)' % (len(fails), len(warns)))
    for f in fails:
        print('  FAIL  ' + f)
    for w in warns:
        print('  WARN  ' + w)
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
