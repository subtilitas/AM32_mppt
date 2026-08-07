/*
 * Host-build shim for the per-MCU Mcu/<mcu>/Inc/phaseouts.h.
 *
 * mppt.c includes phaseouts.h for allOff(). The real header pulls in main.h
 * and the whole CMSIS/HAL tree, which will not compile on the host, so the
 * harness substitutes this.
 *
 * THE SIGNATURE BELOW MUST MATCH THE REAL HEADER. A harness that quietly
 * disagrees with the firmware about a declaration is precisely how the
 * zero_crosses uint16_t/uint32_t mismatch survived for as long as it did.
 * run.sh greps the real header to check allOff() is still there and still
 * takes no arguments; if you add a parameter, both places must change.
 */
#pragma once

void allOff(void);
