#!/bin/bash
# Generate the release-notes index for a directory of built .hex files.
#
#   ./test/release_index.sh release owner/repo v1.0 > body.md
#
# WHY THIS EXISTS
#   A release carries ~247 hex. GitHub renders that as a flat asset list with
#   no search, so the practical question - "which file is for MY ESC?" -
#   is unanswerable unless you already know your MCU, which nobody buying an
#   ESC does. This emits one row per controller, grouped by leading letter and
#   sorted by name, with the MCU and tracker as information rather than as
#   something you must know up front to find your file.
#
#   Anything found in optnotes_*.txt is flagged: those were rebuilt at -Os to
#   fit and have had different codegen applied to timing-sensitive paths.
set -eu
DIR=${1:?usage: release_index.sh <dir> <owner/repo> <tag>}
REPO=${2:?}
TAG=${3:?}
cd "$(dirname "$0")/.."

python3 - "$DIR" "$REPO" "$TAG" <<'PY'
import glob, os, re, subprocess, sys
d, repo, tag = sys.argv[1], sys.argv[2], sys.argv[3]

# tracker per target, from the same source the build used
tracker = {}
for line in subprocess.run(['./test/mppt_targets.sh'], capture_output=True,
                           text=True, check=True).stdout.split('\n'):
    if line.strip():
        t, k = line.split()
        tracker[t] = k

# targets that needed -Os to fit
shrunk = {}
for f in glob.glob(os.path.join(d, 'optnotes_*.txt')):
    for line in open(f):
        if line.strip():
            shrunk[line.split()[0]] = line.strip().split(' ', 1)[1]

MCUS = ['F421', 'G071', 'F051', 'L431', 'F415', 'E230', 'G431', 'F031',
        'V203', 'G031']
rows = []
for path in glob.glob(os.path.join(d, '*.hex')):
    fn = os.path.basename(path)
    m = re.match(r'AM32_(.+)_[0-9]+\.[0-9]+\.hex$', fn)
    if not m:
        continue
    target = m.group(1)
    mcu = next((x for x in MCUS if '_' + x in '_' + target), '?')
    rows.append((target, mcu, tracker.get(target, '?'), fn))
rows.sort()

if not rows:
    sys.exit('no hex files found in ' + d)

url = f'https://github.com/{repo}/releases/download/{tag}'
out = []
out.append('## Find your controller')
out.append('')
out.append(f'{len(rows)} controllers. Expand your initial, or just press '
           '**Ctrl+F** and type your ESC name. You do not need to know which '
           'MCU your board uses - it is listed, not required.')
out.append('')
out.append('`beta` tracks the array with a current/voltage regulator and is '
           'the better tracker. `rpm` is the fallback for boards with no '
           'current sense, and dithers slightly. This is decided by the '
           'board, not by you.')
out.append('')

groups = {}
for r in rows:
    groups.setdefault(r[0][0].upper(), []).append(r)

for letter in sorted(groups):
    g = groups[letter]
    out.append(f'<details><summary><b>{letter}</b> &nbsp; ({len(g)} '
               f'controller{"s" if len(g) != 1 else ""})</summary>')
    out.append('')
    out.append('| Controller | MCU | Tracker | Download |')
    out.append('|---|---|---|---|')
    for target, mcu, trk, fn in g:
        note = ' ⚠️' if target in shrunk else ''
        out.append(f'| {target}{note} | {mcu} | `{trk.lower()}` | '
                   f'[{fn}]({url}/{fn}) |')
    out.append('')
    out.append('</details>')
    out.append('')

if shrunk:
    out.append('### ⚠️ Built at `-Os` to fit')
    out.append('')
    out.append('These parts have no room for MPPT at the `-O3` AM32 normally '
               'uses, so they were rebuilt at `-Os -fdata-sections`. They are '
               'functionally the same firmware, but the compiler made '
               'different choices on timing-sensitive paths. Bench them before '
               'you fly them.')
    out.append('')
    for t in sorted(shrunk):
        out.append(f'- `{t}` — {shrunk[t]}')
    out.append('')

# "where is my board?" answered here rather than left as a silent absence.
# The junk-name reason is skipped: those entries are not real boards.
exc = subprocess.run(['./test/mppt_targets.sh', '--excluded'],
                     capture_output=True, text=True, check=True).stdout
dropped = {}
for line in exc.split('\n'):
    if '\t' in line:
        t, r = line.split('\t', 1)
        if r != 'not a buildable make target':
            dropped.setdefault(r, []).append(t)

if dropped:
    out.append('## Boards deliberately not built')
    out.append('')
    for reason in sorted(dropped, key=lambda r: -len(dropped[r])):
        ts = sorted(dropped[reason])
        out.append(f'<details><summary>{reason} &nbsp; ({len(ts)})</summary>')
        out.append('')
        for t in ts:
            out.append(f'- `{t}`')
        out.append('')
        out.append('</details>')
        out.append('')

print('\n'.join(out))
PY
