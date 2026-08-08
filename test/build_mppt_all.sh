#!/bin/bash
# Cross-build every MPPT-eligible AM32 target with the tracker force-enabled.
#
#   ./test/build_mppt_all.sh          # build them all, summary table, non-zero on failure
#   ./test/build_mppt_all.sh -j4      # pass extra flags through to make
#   ./test/build_mppt_all.sh --mcu F051 -j4   # one MCU family (CI runs these in parallel)
#
# Requires the ARM toolchain: run `make arm_sdk_install` first.
#
# WHAT THIS IS
#   A portability and flash-budget gate. It proves the module compiles, links
#   and fits on every buildable board across all nine MCU families, with the
#   tracker each board can actually support: beta where there is a real
#   current sense chain, rpm perturb-and-observe where there is not. Flash overflow on the smaller parts is the failure mode
#   this is most likely to catch - AM32 is already tight on F051.
#
# WHAT THIS IS NOT
#   Validation. A green build does not mean the firmware is flight-ready on
#   that board: MPPT_BETA_MPP_Q8 and MPPT_BETA_VT are panel-specific, and
#   MPPT_V_COLLAPSE / MPPT_V_ABSOLUTE_MIN must come from the board's own
#   regulator dropout. Only EGAN_JUWI_L431 has been characterised.
#
# Target selection is derived from Inc/targets.h at run time by
# mppt_targets.sh, never hard-coded, so it stays correct as boards are added.
set -u
cd "$(dirname "$0")/.."
MCU_FILTER=""
EXTRA_MAKE_ARGS=()
while [ $# -gt 0 ]; do
    case "$1" in
        --mcu) MCU_FILTER="$2"; shift 2 ;;
        *)     EXTRA_MAKE_ARGS+=("$1"); shift ;;
    esac
done

# Locate arm-none-eabi-size (needed for the flash/RAM column; make itself uses
# its own relative ARM_SDK_PREFIX and does not consult PATH).
#
# Do NOT guess the depth. `make arm_sdk_install` unpacks to
#   tools/<os>/xpack-arm-none-eabi-gcc-<version>/bin
# which is two levels under tools/, and an earlier version of this script
# globbed tools/*/bin - one level. An unmatched glob stays literal in bash, so
# PATH silently gained the string "tools/*/bin", the check below failed, and
# the script exited 2 having built nothing. Search for the binary instead: the
# version number is pinned in make/tools.mk and will eventually move.
command -v arm-none-eabi-size >/dev/null 2>&1 || {
    sdk=$(find tools -type f -name arm-none-eabi-size -perm -u+x 2>/dev/null | head -1)
    [ -n "$sdk" ] && export PATH="$PATH:$(cd "$(dirname "$sdk")" && pwd)"
}
command -v arm-none-eabi-size >/dev/null 2>&1 || {
    echo "ERROR: arm-none-eabi toolchain not found under tools/ or on PATH." >&2
    echo "       Run 'make arm_sdk_install' first." >&2
    exit 2
}

if [ -n "$MCU_FILTER" ]; then
    mapfile -t PAIRS < <(./test/mppt_targets.sh --mcu "$MCU_FILTER")
    echo "MPPT-eligible ${MCU_FILTER} targets: ${#PAIRS[@]}"
else
    mapfile -t PAIRS < <(./test/mppt_targets.sh)
    echo "MPPT-eligible targets: ${#PAIRS[@]}"
fi
[ -n "$MCU_FILTER" ] || ./test/mppt_targets.sh --report | sed -n '/^excluded/,$p' | sed 's/^/  /'
echo

# An empty list is a broken mppt_targets.sh or a typo'd --mcu, not a clean run.
# Without this the loop below simply does not execute and the script exits 0,
# which is indistinguishable from success right up until the release is empty.
if [ "${#PAIRS[@]}" -eq 0 ]; then
    echo "ERROR: no MPPT-eligible targets${MCU_FILTER:+ for MCU '$MCU_FILTER'}." >&2
    echo "       Check ./test/mppt_targets.sh --report" >&2
    exit 2
fi

# One make invocation per target: slower than `make all`, but it gives a
# per-target verdict instead of stopping at the first broken board.
#
# THE -Os RETRY
#   AM32 compiles everything at -O3. On a 27 KB F051 that is the wrong trade
#   the moment MPPT's ~3.4 kB lands on top, and F051 is 49 of the 247 targets.
#   So a target that overflows flash is retried once at -Os -fdata-sections
#   instead of being abandoned. -fdata-sections is worth having on its own:
#   LDFLAGS already carries --gc-sections and CFLAGS already carries
#   -ffunction-sections, so read-only data is the one thing never collected.
#
#   Only targets that actually overflow are retried, so this cannot change
#   codegen on a board that already fit. Which boards needed it is recorded in
#   obj/optnotes_*.txt and surfaced in the release notes - -Os is different
#   codegen on timing-sensitive paths and those builds want bench validation.
mkdir -p obj
SIZE_FALLBACK="-Os -fdata-sections"
NOTES="obj/optnotes_${MCU_FILTER:-all}.txt"
: > "$NOTES"

pass=0; fail=0; shrunk=0; failed=()
row='%-34s %-5s %-4s %10s %10s\n'
printf "$row" TARGET TRACK OPT FLASH RAM
printf "$row" "----------------------------------" "-----" "----" "----------" "----------"
for pair in "${PAIRS[@]}"; do
    T=${pair%% *}; TRACKER=${pair##* }
    log=$(mktemp)
    CF="-DUSE_MPPT -DMPPT_TRACKER=MPPT_TRACKER_$TRACKER"
    opt="-O3"; built=0

    if make "$T" EXTRA_CFLAGS="$CF" ${EXTRA_MAKE_ARGS[@]+"${EXTRA_MAKE_ARGS[@]}"} >"$log" 2>&1; then
        built=1
    elif grep -qE "overflowed by|will not fit in region" "$log"; then
        over=$(grep -oE "overflowed by [0-9]+ bytes" "$log" | head -1)
        # a failed link can leave a partial .elf behind, and make would then
        # consider the target up to date and never apply the new flags
        rm -f obj/AM32_"$T"_[0-9]*.elf
        if make "$T" EXTRA_CFLAGS="$CF $SIZE_FALLBACK" \
                ${EXTRA_MAKE_ARGS[@]+"${EXTRA_MAKE_ARGS[@]}"} >"$log" 2>&1; then
            built=1; opt="-Os"; shrunk=$((shrunk + 1))
            echo "$T ${over:-overflowed flash} at -O3" >> "$NOTES"
        fi
    fi

    if [ "$built" = 1 ]; then
        # [0-9] anchors the version field. Without it AM32_TBS_12S_F415_*.elf
        # also matches AM32_TBS_12S_F415_CAN_*.elf - there are 8 such sibling
        # pairs, and the size column would report the wrong board.
        elf=$(ls -t obj/AM32_"$T"_[0-9]*.elf 2>/dev/null | head -1)
        if [ -n "$elf" ]; then
            read -r txt dat bss _ < <(arm-none-eabi-size "$elf" | tail -1)
            printf "$row" "$T" "$TRACKER" "$opt" "$((txt + dat))" "$((dat + bss))"
        else
            printf "$row" "$T" "$TRACKER" "$opt" "ok" "-"
        fi
        pass=$((pass + 1))
    else
        printf '%-34s %-5s %-4s %10s\n' "$T" "$TRACKER" "-" "FAIL"
        # surface the useful line: overflow, missing define, failed assertion
        grep -m3 -E "overflowed|region .* overflow|error:|static assertion" "$log" | sed 's/^/      /'
        failed+=("$T"); fail=$((fail + 1))
    fi
    rm -f "$log"
done

echo
hex=$(ls obj/*.hex 2>/dev/null | wc -l)
echo "built $pass ($shrunk needed -Os), failed $fail, hex in obj/: $hex"
if [ "$shrunk" -gt 0 ]; then
    echo "rebuilt at -Os to fit:"; sed 's/^/  /' "$NOTES"
fi
if [ "$fail" -gt 0 ]; then
    printf 'failed targets:\n'; printf '  %s\n' "${failed[@]}"
    exit 1
fi
# The .hex is what CI uploads and what ends up in a release, but the loop above
# only ever looks at the .elf. Say so out loud rather than handing the release
# job an empty directory.
if [ "$hex" -eq 0 ]; then
    echo "ERROR: $pass targets built but obj/ contains no .hex - nothing to release." >&2
    exit 3
fi
