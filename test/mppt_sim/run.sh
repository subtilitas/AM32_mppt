#!/bin/bash
# Compiles the SHIPPING Src/mppt.c - no stubbed algorithm - against a PV +
# bus-capacitor + BLDC + propeller plant, and flies a profile with a
# 70-degree bank manoeuvre and a 25 ms cloud edge.
#
#   ./run.sh                             # 3.4 s profile, board config as committed
#   ./run.sh 60                          # 60 s soak
#   G=0.25 ./run.sh 60                   # same panel at 25% irradiance
#   EXTRA="-DSHADE_AT=5.0" ./run.sh 8    # force a bus collapse at t=5 s
#   ./run.sh test                        # directed tests (test_directed.c)
#   TRACKER=RPM ./run.sh 60              # exercise the no-current-sense fallback
#
# ONE panel, with irradiance as the variable. There is deliberately no
# "small array" versus "large array" case: a large array in low light draws
# the same handful of ADC counts as a small array in full sun, so splitting
# them invents a distinction the hardware does not have.
#
# Board constants come from the REAL Inc/targets.h, so TARGET_VOLTAGE_DIVIDER,
# CURRENT_OFFSET, LOOP_FREQUENCY_HZ and every derived MPPT constant are exactly
# what the firmware will use.
#
# TWO THINGS THE HARNESS MUST KEEP GETTING RIGHT:
#
#  1. harness.c declares the AM32 globals with the types Src/main.c actually
#     uses. The original had zero_crosses as uint16_t where main.c has
#     volatile uint32_t, and that mismatch hid a real in-flight duty collapse.
#
#  2. HW_MVA is the TRUE sense-amp gain in mV/A. It is deliberately NOT
#     MILLIVOLT_PER_AMP: on EGAN_MPPT_L431 that constant is mis-set to make
#     DShot telemetry readable on a sub-amp panel. The plant must model the
#     hardware, not the firmware's belief about it.
#
#     Related, and the reason this warning exists twice: the plant must also
#     compute bus current from the APPLIED duty, not from AM32's unclamped
#     intent. Getting that wrong fed the firmware ~6x the real current and
#     made every absolute-current threshold meaningless.
set -e
cd "$(dirname "$0")"
ROOT=../..
T=${1:-3.4}
[ "$T" = "test" ] || [ "$T" = "coast" ] && T=3.4
BOARD=${BOARD:-EGAN_MPPT_L431}
HW_MVA=${HW_MVA:-136}
OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT
mkdir -p "$OUT/inc"

G=${G:-1.0}
PLANT="-DG_SCALE=$G"
cp $ROOT/Inc/targets.h "$OUT/inc/targets.h"
cp $ROOT/Inc/mppt.h "$OUT/inc/"

if [ "$1" = "test" ] || [ "$1" = "coast" ]; then
    mkdir -p "$OUT/inc"; cp $ROOT/Inc/targets.h $ROOT/Inc/mppt.h "$OUT/inc/"
    gcc -std=gnu99 -O1 -g -fsanitize=undefined,address -DUSE_MAKE -D$BOARD \
        -DMPPT_TRACKER=MPPT_TRACKER_${TRACKER:-BETA} \
        -I. -I"$OUT/inc" -o "$OUT/tc" $ROOT/Src/mppt.c harness.c test_directed.c -lm
    exec "$OUT/tc"
fi

TRK="-DMPPT_TRACKER=MPPT_TRACKER_${TRACKER:-BETA}"
CF="-std=gnu99 -O2 -DUSE_MAKE -D$BOARD -DHW_MVA=$HW_MVA $PLANT $TRK $EXTRA -I. -I$OUT/inc"

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
