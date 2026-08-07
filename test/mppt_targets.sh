#!/bin/bash
# List the AM32 board targets that MPPT can meaningfully be built for.
#
#   ./mppt_targets.sh            # "TARGET TRACKER" pairs, one per line
#   ./mppt_targets.sh --beta     # only the beta-capable targets
#   ./mppt_targets.sh --rpm      # only the rpm-fallback targets
#   ./mppt_targets.sh --report   # both lists plus exclusions, with reasons
#   ./mppt_targets.sh --mcu F051 # only that MCU family (used by CI's matrix)
#
# THE SELECTION RULE, AND WHY IT IS NOT "ALL TARGETS"
#
# Inc/targets.h ends with
#
#     #ifndef MILLIVOLT_PER_AMP
#     #define MILLIVOLT_PER_AMP 20
#     #endif
#
# so a board that never mentions the sense chain still compiles, and still
# reports a current - it is just a fabricated one. 140 of the 254 board
# blocks are in that position.
#
# That matters more for MPPT than for anything else in AM32. The beta
# regulator drives the setpoint from ln(I/V); handed a fabricated current it
# does not degrade gracefully, it walks vref into a clamp rail.
#
# So the board's OWN block defining MILLIVOLT_PER_AMP - somebody
# characterised a real sense chain - selects the tracker:
#
#   defines it        -> MPPT_TRACKER_BETA   (regulator, no dither, 99.9%)
#   inherits the 20   -> MPPT_TRACKER_RPM    (perturb-and-observe on
#                        e_com_time; needs no current at all, but dithers)
#
# That is the best signal targets.h offers; there is no explicit "has
# current sensing" flag.
#
# Also excluded: NXP MCXA parts, whose 16-bit ADC needs different current
# staging that mppt.h refuses outright rather than mis-scale by 16x.
set -e
cd "$(dirname "$0")/.."

python3 - "$@" <<'PY'
import re, sys
report = '--report' in sys.argv
only   = 'beta' if '--beta' in sys.argv else ('rpm' if '--rpm' in sys.argv else None)
mcu    = None
if '--mcu' in sys.argv:
    mcu = sys.argv[sys.argv.index('--mcu') + 1].upper()
src = open('Inc/targets.h', encoding='utf-8', errors='replace').read()
src = src.replace('\r\n', '\n').split('\n')

# Walk #ifdef/#endif nesting and attribute each #define to its innermost block.
blocks, stack = {}, []
for ln in src:
    m = re.match(r'\s*#if(?:def)?\s+([A-Za-z_][A-Za-z0-9_]*)\s*$', ln)
    if m:
        stack.append([m.group(1), set(), None]); continue
    if re.match(r'\s*#endif', ln):
        if stack:
            name, defs, fname = stack.pop()
            e = blocks.setdefault(name, [set(), None])
            e[0] |= defs
            if fname: e[1] = fname
        continue
    d = re.match(r'\s*#define\s+([A-Za-z_][A-Za-z0-9_]*)(.*)', ln)
    if d and stack:
        stack[-1][1].add(d.group(1))
        if d.group(1) == 'FILE_NAME':
            q = re.search(r'"([^"]+)"', d.group(2))
            if q: stack[-1][2] = q.group(1)

# The Makefile builds a target if its FILE_NAME contains _<MCU> (make/tools.mk
# get_targets). Mirror that exactly so we never emit a name make cannot build.
MCUS = ['E230','F031','F051','F415','F421','G071','L431','G431','V203','G031','A153']
eligible, excluded = [], []
for name, (defs, fname) in sorted(blocks.items()):
    if not fname:
        continue
    if not any('_'+m in fname for m in MCUS):
        excluded.append((fname, 'not a buildable make target')); continue
    if 'A153' in fname:
        excluded.append((fname, 'NXP MCXA - 16-bit ADC, current staging unsupported')); continue
    if mcu and ('_' + mcu) not in fname:
        continue
    tracker = 'BETA' if 'MILLIVOLT_PER_AMP' in defs else 'RPM'
    eligible.append((fname, tracker))

beta = [t for t, k in eligible if k == 'BETA']
rpm  = [t for t, k in eligible if k == 'RPM']

if report:
    print(f"MPPT_TRACKER_BETA - board declares its own MILLIVOLT_PER_AMP ({len(beta)}):")
    for t in beta: print("   ", t)
    print(f"\nMPPT_TRACKER_RPM - no current sense, rpm fallback ({len(rpm)}):")
    for t in rpm: print("   ", t)
    print(f"\nexcluded ({len(excluded)}):")
    reasons = {}
    for t, r in excluded: reasons.setdefault(r, []).append(t)
    for r, ts in sorted(reasons.items(), key=lambda x: -len(x[1])):
        print(f"    {len(ts):3d}  {r}")
elif only == 'beta':
    for t in beta: print(t)
elif only == 'rpm':
    for t in rpm: print(t)
else:
    for t, k in eligible: print(t, k)
PY
