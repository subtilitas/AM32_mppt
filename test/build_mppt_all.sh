#!/bin/bash
# Cross-build every MPPT-eligible AM32 target with the tracker force-enabled.
#
#   ./test/build_mppt_all.sh          # build them all, summary table, non-zero on failure
#   ./test/build_mppt_all.sh -j4      # pass extra flags through to make
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
#   regulator dropout. Only EGAN_MPPT_L431 has been characterised.
#
# Target selection is derived from Inc/targets.h at run time by
# mppt_targets.sh, never hard-coded, so it stays correct as boards are added.
set -u
cd "$(dirname "$0")/.."
EXTRA_MAKE_ARGS=("$@")

command -v arm-none-eabi-gcc >/dev/null 2>&1 || {
    if [ -d tools ]; then export PATH="$PATH:$(echo tools/*/bin | tr ' ' ':')"; fi
}
command -v arm-none-eabi-size >/dev/null 2>&1 || {
    echo "ERROR: arm-none-eabi toolchain not on PATH. Run 'make arm_sdk_install'." >&2
    exit 2
}

mapfile -t PAIRS < <(./test/mppt_targets.sh)
echo "MPPT-eligible targets: ${#PAIRS[@]}"
./test/mppt_targets.sh --report | sed -n '/^excluded/,$p' | sed 's/^/  /'
echo

# One make invocation per target: slower than `make all`, but it gives a
# per-target verdict instead of stopping at the first broken board.
mkdir -p obj
pass=0; fail=0; failed=()
printf '%-34s %-5s %10s %10s\n' TARGET TRACK FLASH RAM
printf '%-34s %-5s %10s %10s\n' "----------------------------------" "-----" "----------" "----------"
for pair in "${PAIRS[@]}"; do
    T=${pair%% *}; TRACKER=${pair##* }
    log=$(mktemp)
    CF="-DUSE_MPPT -DMPPT_TRACKER=MPPT_TRACKER_$TRACKER"
    if make "$T" EXTRA_CFLAGS="$CF" "${EXTRA_MAKE_ARGS[@]}" >"$log" 2>&1; then
        elf=$(ls -t obj/AM32_"$T"_*.elf 2>/dev/null | head -1)
        if [ -n "$elf" ]; then
            read -r txt dat bss _ < <(arm-none-eabi-size "$elf" | tail -1)
            printf '%-34s %-5s %10s %10s\n' "$T" "$TRACKER" "$((txt + dat))" "$((dat + bss))"
        else
            printf '%-34s %-5s %10s %10s\n' "$T" "$TRACKER" "ok" "-"
        fi
        pass=$((pass + 1))
    else
        printf '%-34s %-5s %10s\n' "$T" "$TRACKER" "FAIL"
        # surface the useful line: overflow, missing define, failed assertion
        grep -m3 -E "overflowed|region .* overflow|error:|static assertion" "$log" | sed 's/^/      /'
        failed+=("$T"); fail=$((fail + 1))
    fi
    rm -f "$log"
done

echo
echo "built $pass, failed $fail"
if [ "$fail" -gt 0 ]; then
    printf 'failed targets:\n'; printf '  %s\n' "${failed[@]}"
    exit 1
fi
