#include "main.h"

#ifndef JETI_EX_TELEMETRY_H_
#define JETI_EX_TELEMETRY_H_

// Jeti Ex telemetry protocol (JETI Telemetry Protocol EN V1.06).
// Sensor frames are built into the shared aTxBuffer from kiss_telemetry.c
// and are sent out by the serial_telemetry driver using send_telem_DMA().

// EX data frame header
#define JETI_EX_SEPARATOR 0x7E
#define JETI_EX_MARKER 0x2F
#define JETI_EX_PKT_DATA 0x40
#define JETI_EX_PKT_TEXT 0x00

// 3rd party manufacturer id 0xA409, random device id 0x3276
#define JETI_EX_MANUFACTURER_LO 0x09
#define JETI_EX_MANUFACTURER_HI 0xA4
#define JETI_EX_DEVICE_LO 0x76
#define JETI_EX_DEVICE_HI 0x32

// sensor value data types
#define JETI_TYPE_14b 1 // int14_t
#define JETI_TYPE_22b 4 // int22_t

// sensor ids
enum {
    JETI_ID_TEMP = 1, // temperature, degrees Celsius
    JETI_ID_VOLT, // battery voltage, volts, 2 decimal places
    JETI_ID_CURR, // current, amps, 2 decimal places
    JETI_ID_CONS, // consumed capacity, mAh
    JETI_ID_RPM, // electrical rpm
    JETI_ID_COUNT
};

extern uint8_t aTxBuffer[49] __attribute__((aligned(4)));

// Builds the next Jeti Ex frame in aTxBuffer and returns its length in bytes.
// Sensor dictionary frames and value frames are alternated automatically.
uint8_t makeJetiTelemPackage(int8_t temp, uint16_t voltage, uint16_t current,
    uint16_t consumption, uint16_t e_rpm);

#endif
