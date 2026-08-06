/*
 * AM32 globals the MPPT module consumes.
 *
 * EVERY TYPE HERE MUST MATCH ITS DEFINITION IN Src/main.c. These are separate
 * translation units in the real firmware, so nothing checks the agreement -
 * declaring zero_crosses as uint16_t when main.c has volatile uint32_t hid a
 * real in-flight duty collapse for the entire life of the module, because the
 * harness wrapped at the same point the firmware did.
 */
#include <stdint.h>

uint16_t ADC_raw_volts = 0, ADC_raw_current = 0, VOLTAGE_DIVIDER = 47;
volatile char armed = 0;
uint8_t running = 0;
uint16_t input = 0;
char prop_brake_active = 0;
volatile uint32_t zero_crosses = 0;      /* main.c:554 - uint32, uncapped */
char play_tone_flag = 0;                 /* main.c:411 */
int e_com_time = 65408;                  /* main.c:433 - electrical rev, us.
                                          * 65408 is AM32's own pre-spin-up
                                          * sentinel; start there so the rpm
                                          * tracker's validity check is
                                          * actually exercised. */

/* Stand-in for AM32's allOff(). The plant model reads h_all_off to decide
 * whether the motor is shorted through the low-side FETs (braking) or truly
 * floating (coasting) - the distinction the module exists to make. Cleared by
 * the plant each tick, so it reflects "was allOff called this tick". */
int h_all_off = 0;
void allOff(void) { h_all_off = 1; }
