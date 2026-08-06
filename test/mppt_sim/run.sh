#!/bin/bash
# Compiles the SHIPPING Src/mppt.c (no stubs in the algorithm) against a
# PV + bus-capacitor + BLDC + propeller plant model and flies a profile with
# a 70-degree bank manoeuvre and a 25 ms cloud edge.
#
#   ./run.sh                  # board config as committed, 3.4 s profile
#   ./run.sh 60               # 60 s soak
#   ARRAY=flight ./run.sh 60  # 12 V / 3 A array   - exercises the hill-climber
#   ARRAY=bench  ./run.sh 30  # 13.4 V / 120 mA    - exercises the FOCV path
#   EXTRA="-DSHADE_AT=5.0" ./run.sh 8    # force a bus collapse at t=5 s
#   ./run.sh coast            # directed test of the coast / unload path
#
# Board constants come from the REAL Inc/targets.h, so TARGET_VOLTAGE_DIVIDER,
# CURRENT_OFFSET, LOOP_FREQUENCY_HZ and every derived MPPT constant are exactly
# what the firmware will use. ARRAY= patches only the two nameplate lines, into
# a scratch copy of the header, so there is still exactly one definition of
# everything and the build stays -Werror clean.
#
# TWO THINGS THE HARNESS MUST KEEP GETTING RIGHT:
#
#  1. harness.c declares the AM32 globals with the types Src/main.c actually
#     uses. The original harness had zero_crosses as uint16_t where main.c has
#     volatile uint32_t, and that mismatch hid a real in-flight duty collapse.
#
#  2. HW_MVA is the TRUE sense-amp gain in mV/A. It is deliberately NOT
#     MILLIVOLT_PER_AMP: on EGAN_MPPT_L431 that constant is mis-set to make
#     DShot telemetry readable on a sub-amp panel. The plant must model the
#     hardware, not the firmware's belief about it.
set -e
cd "$(dirname "$0")"
ROOT=../..
T=${1:-3.4}
[ "$T" = "coast" ] && T=3.4
BOARD=${BOARD:-EGAN_MPPT_L431}
HW_MVA=${HW_MVA:-136}
OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT
mkdir -p "$OUT/inc"

case "${ARRAY:-board}" in
  flight) VOC=1200; ISC=300; PLANT="-DVOC_STC=12.0 -DISC_STC=3.0 -DVTH=0.672" ;;
  bench)  VOC=1340; ISC=12;  PLANT="-DVOC_STC=13.4 -DISC_STC=0.12 -DVTH=0.75" ;;
  *)      VOC=;     ISC=;    PLANT= ;;
esac

if [ -n "$VOC" ]; then
    sed -E "s/(#define MPPT_VOC_NOMINAL +)[0-9]+/\1$VOC/; \
            s/(#define MPPT_ARRAY_ISC +)[0-9]+/\1$ISC/" \
        $ROOT/Inc/targets.h > "$OUT/inc/targets.h"
else
    cp $ROOT/Inc/targets.h "$OUT/inc/targets.h"
fi
cp $ROOT/Inc/mppt.h "$OUT/inc/"

if [ "$1" = "coast" ]; then
    mkdir -p "$OUT/inc"; cp $ROOT/Inc/targets.h $ROOT/Inc/mppt.h "$OUT/inc/"
    gcc -std=gnu99 -O1 -g -fsanitize=undefined,address -DUSE_MAKE -D$BOARD \
        -I. -I"$OUT/inc" -o "$OUT/tc" $ROOT/Src/mppt.c harness.c test_coast.c -lm
    exec "$OUT/tc"
fi

CF="-std=gnu99 -O2 -DUSE_MAKE -D$BOARD -DHW_MVA=$HW_MVA $PLANT $EXTRA -I. -I$OUT/inc"

# common.h and phaseouts.h here are shims for headers that cannot compile on
# the host. Check every declaration they duplicate still matches the real
# thing, so a shim cannot silently drift the way the old harness did.
if ! grep -qE '^[[:space:]]*void[[:space:]]+allOff[[:space:]]*\([[:space:]]*(void)?[[:space:]]*\)' \
        $ROOT/Mcu/l431/Inc/phaseouts.h; then
    echo "ERROR: allOff() in Mcu/l431/Inc/phaseouts.h no longer matches the harness shim" >&2
    exit 1
fi
while read -r decl; do
    [ -z "$decl" ] && continue
    if ! grep -qF "$decl" $ROOT/Inc/common.h; then
        echo "ERROR: harness common.h declares '$decl' which is not in Inc/common.h" >&2
        exit 1
    fi
done <<EOF
extern char play_tone_flag;
extern int e_com_time;
EOF

# the module itself is held to AM32's own warning flags
gcc $CF -Wall -Wundef -Wextra -Werror -c $ROOT/Src/mppt.c -o "$OUT/mppt.o"
gcc $CF -DTSIM=$T -o "$OUT/sim" "$OUT/mppt.o" harness.c sim.c -lm
"$OUT/sim"
