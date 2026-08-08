#!/bin/bash
# Verify every hand-written extern in Src/mppt.c against the real definition.
#
#   ./test/check_externs.sh
#
# WHY
#   mppt.c reaches into AM32's globals. Most come from Inc/common.h, where the
#   compiler enforces agreement because main.c includes it too. A handful do
#   not appear in any shared header and are declared by hand in mppt.c - and a
#   hand-written extern is a promise the compiler cannot check across
#   translation units.
#
#   That is not hypothetical. zero_crosses was declared uint16_t here while
#   main.c defines it volatile uint32_t. The linker was happy. On a
#   little-endian target the low half read correctly for 13.7 seconds and then
#   wrapped, the duty collapsed from 1494 to 400 and the ESC pushed 2.7 A back
#   into the panel. See doc/MPPT.md 6.1.
#
#   So: find each extern's definition in the AM32 sources and compare types
#   textually. Not as good as a shared header, but it catches the exact class
#   of defect that already got through, and it runs in CI in under a second.
set -eu
cd "$(dirname "$0")/.."

python3 - <<'PY'
import re, sys, glob

src = open('Src/mppt.c', encoding='utf-8', errors='replace').read()

# Only file-scope externs, and only those NOT satisfied by a shared header.
externs = {}
for m in re.finditer(r'^extern\s+([^;]+?)\s*;\s*$', src, re.M):
    decl = ' '.join(m.group(1).split())
    name = re.findall(r'(\w+)\s*(?:\[[^\]]*\])?$', decl)[0]
    externs[name] = re.sub(r'\s*\b' + name + r'\b\s*(\[[^\]]*\])?$', '', decl).strip()

headers = ''
for h in ('Inc/common.h', 'Mcu/l431/Inc/phaseouts.h'):
    try:
        headers += open(h, encoding='utf-8', errors='replace').read()
    except OSError:
        pass

def norm(t):
    """Compare types modulo qualifier order and spelling noise."""
    toks = t.replace('*', ' * ').split()
    # `unsigned int` == `unsigned`, `signed char` != `char`, keep it simple
    toks = [x for x in toks if x != 'int' or len(toks) == 1]
    return ' '.join(sorted(toks))

sources = sorted(set(glob.glob('Src/*.c') + glob.glob('Mcu/*/Src/*.c')))
bad, unfound, ok = [], [], []

for name, want in sorted(externs.items()):
    found = []
    for f in sources:
        for ln in open(f, encoding='utf-8', errors='replace'):
            # file scope only: AM32 defines globals at column 0
            if ln.startswith(('extern', ' ', '\t', '#', '/', '*')):
                continue
            m = re.match(r'^([A-Za-z_][\w \t*]*?)\s+\*?\s*' + re.escape(name)
                         + r'\s*(?:\[[^\]]*\])?\s*(=|;)', ln)
            if m:
                found.append((f, ' '.join(m.group(1).split())))
    if not found:
        # a header may legitimately own it (common.h), then the compiler checks
        if re.search(r'\b' + re.escape(name) + r'\b', headers):
            ok.append((name, want, 'declared in a shared header - compiler-checked'))
        else:
            unfound.append((name, want))
        continue
    for f, got in found:
        if norm(got) != norm(want):
            bad.append((name, want, got, f))
        else:
            ok.append((name, want, f))

w = max(len(n) for n in externs) if externs else 8
for name, want, where in ok:
    print(f'  ok    {name:<{w}}  {want:<24}  {where}')
for name, want in unfound:
    print(f'  ?     {name:<{w}}  {want:<24}  no definition found - check by hand')
for name, want, got, f in bad:
    print(f'  BAD   {name:<{w}}  mppt.c says "{want}", {f} defines "{got}"')

print()
print(f'{len(ok)} checked, {len(bad)} mismatched, {len(unfound)} unresolved')
if bad:
    print('\nA type mismatch here is silent at link time and corrupts reads at '
          'run time. Fix mppt.c to match the definition.', file=sys.stderr)
    sys.exit(1)
if unfound:
    print('\nCould not locate a definition for the above. Either the variable '
          'moved or the matcher needs widening; do not ignore this.',
          file=sys.stderr)
    sys.exit(1)
PY
