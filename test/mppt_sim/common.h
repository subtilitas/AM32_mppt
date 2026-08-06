/*
 * Host-build shim for Inc/common.h.
 *
 * The real header pulls in eeprom.h -> main.h -> the whole CMSIS/HAL tree,
 * which will not compile on the host. This declares only the subset mppt.c
 * actually uses from it.
 *
 * EVERY DECLARATION BELOW IS COPIED VERBATIM FROM Inc/common.h AND MUST STAY
 * THAT WAY. run.sh greps the real header to check. This is the same hazard
 * that let zero_crosses be declared uint16_t against a volatile uint32_t
 * definition for the whole life of the module: a harness that disagrees with
 * the firmware about a type does not fail, it quietly tests something else.
 */
#pragma once

#include <stdint.h>

extern char play_tone_flag;   /* Inc/common.h:23 */
extern int e_com_time;        /* Inc/common.h:48 */
