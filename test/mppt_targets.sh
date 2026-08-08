#!/bin/bash
# List the AM32 board targets that MPPT can meaningfully be built for.
#
#   ./mppt_targets.sh            # "TARGET TRACKER" pairs, one per line
#   ./mppt_targets.sh --beta     # only the beta-capable targets
#   ./mppt_targets.sh --rpm      # only the rpm-fallback targets
#   ./mppt_targets.sh --report   # both lists plus exclusions, with reasons
#   ./mppt_targets.sh --excluded # "TARGET<tab>reason" for every board dropped
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
# staging that mppt.h refuses outright rather than mis-scale by 16x; and
# 4-in-1 / AIO boards, where four ESCs run four independent trackers against
# one shared solar bus and read each other's perturbations as their own.
set -e
cd "$(dirname "$0")/.."

python3 - "$@" <<'PY'
import re, sys
report = '--report' in sys.argv
show_excl = '--excluded' in sys.argv
only   = 'beta' if '--beta' in sys.argv else ('rpm' if '--rpm' in sys.argv else None)
mcu    = None
if '--mcu' in sys.argv:
    mcu = sys.argv[sys.argv.index('--mcu') + 1].upper()
src = open('Inc/targets.h', encoding='utf-8', errors='replace').read()
src = src.replace('\r\n', '\n').split('\n')

# Walk #ifdef/#endif nesting and attribute each #define to its innermost block.
# Every conditional must be PUSHED, not just the board blocks, because every
# one of them is POPPED by its #endif. targets.h has 52 #ifndef guards, many
# of them nested inside a board block; matching only `#ifdef NAME` meant the
# guard's #endif closed the BOARD, and every define after it - including the
# MILLIVOLT_PER_AMP that picks the tracker - was attributed to the wrong
# scope. Blocks that are not a plain `#ifdef NAME` get a null name and fold
# their defines back into the parent on close, which is what the preprocessor
# effectively does for a board that guards a define with #ifndef.
IFDEF = re.compile(r'\s*#\s*ifdef\s+([A-Za-z_]\w*)')
ANYIF = re.compile(r'\s*#\s*if')
ENDIF = re.compile(r'\s*#\s*endif')

blocks, stack = {}, []
for ln in src:
    if ENDIF.match(ln):
        if stack:
            name, defs, fname, dis = stack.pop()
            if name is not None:
                e = blocks.setdefault(name, [set(), None, False])
                e[0] |= defs
                if fname: e[1] = fname; e[2] = dis
            elif stack:
                stack[-1][1] |= defs
                if fname and stack[-1][2] is None:
                    stack[-1][2] = fname; stack[-1][3] = dis
        continue
    m = IFDEF.match(ln)
    if m:
        # trailing comments are common: `#ifdef SISKIN_11A_F051 // PB4 boot`
        stack.append([m.group(1), set(), None, False]); continue
    if ANYIF.match(ln):
        stack.append([None, set(), None, False]); continue
    d = re.match(r'\s*#define\s+([A-Za-z_][A-Za-z0-9_]*)(.*)', ln)
    if d and stack:
        stack[-1][1].add(d.group(1))
        if d.group(1) == 'FILE_NAME':
            q = re.search(r'"([^"]+)"', d.group(2))
            if q:
                stack[-1][2] = q.group(1)
                # get_targets in make/tools.mk drops any FILE_NAME line
                # carrying DISABLE_BUILD or //#, so `make <that target>`
                # simply does not exist. Honour the same markers rather than
                # maintaining a hand-written exclusion list here: upstream
                # will disable more boards over time and this stays correct.
                stack[-1][3] = ('DISABLE_BUILD' in ln) or ('//#' in ln)

# The Makefile builds a target if its FILE_NAME contains _<MCU> (make/tools.mk
# get_targets). Mirror that exactly so we never emit a name make cannot build.
MCUS = ['E230','F031','F051','F415','F421','G071','L431','G431','V203','G031','A153']

# Four-output hardware. Each of the four ESCs on a 4-in-1 runs its own copy of
# this firmware with its own tracker, and they all share one solar bus - so
# they perturb the same node and read each other's perturbations as their own
# result. AIO is the same hardware with a flight controller attached.
#
# Matched on FILE_NAME only. FIRMWARE_NAME is not usable for this: upstream has
# ORQA_F421 branded "TBSlu6s4in1", FLIPSKY_F421 branded "MAXKGO_4IN1" and
# GIPSY_F421 branded "Tekko32 4in1", which are copy-paste artefacts and say
# nothing about the board.
MULTI = re.compile(r'(^|_)(4IN1|AIO)(_|$)', re.I)

eligible, excluded = [], []
for name, (defs, fname, disabled) in sorted(blocks.items()):
    if not fname:
        continue
    if disabled:
        excluded.append((fname, 'DISABLE_BUILD in targets.h - upstream does '
                                'not build it either')); continue
    if not any('_'+m in fname for m in MCUS):
        excluded.append((fname, 'not a buildable make target')); continue
    if 'A153' in fname:
        excluded.append((fname, 'NXP MCXA - 16-bit ADC, current staging unsupported')); continue
    if MULTI.search(fname):
        excluded.append((fname, '4-in-1: four independent MPPT regulators on '
                                'one shared solar bus')); continue
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
elif show_excl:
    for t, r in sorted(excluded): print(f"{t}\t{r}")
elif only == 'beta':
    for t in beta: print(t)
elif only == 'rpm':
    for t in rpm: print(t)
else:
    for t, k in eligible: print(t, k)
PY
