/*
 * jeti_ex_telemetry.c
 *
 * Jeti Ex telemetry protocol frame builder, modelled after the KISS
 * telemetry module. Frames are written into the shared aTxBuffer and
 * transmitted by the serial_telemetry driver with send_telem_DMA().
 *
 * Protocol reference: JETI Telemetry Protocol EN V1.06 and
 * https://github.com/Sepp62/JetiExSensor
 */

#include "jeti_ex_telemetry.h"

#define JETI_SENSOR_NAME "AM32"

typedef struct {
    uint8_t id;
    const char* label;
    const char* unit;
    uint8_t data_type;
    uint8_t precision; // number of decimal places, 0-2
} jeti_sensor_t;

static const jeti_sensor_t jeti_sensors[] = {
    { JETI_ID_TEMP, "Temp", "\xF8""C", JETI_TYPE_14b, 0 },
    { JETI_ID_VOLT, "Voltage", "V", JETI_TYPE_22b, 2 },
    { JETI_ID_CURR, "Current", "A", JETI_TYPE_22b, 2 },
    { JETI_ID_CONS, "Capacity", "mAh", JETI_TYPE_22b, 0 },
    { JETI_ID_RPM, "RPM", "", JETI_TYPE_22b, 0 },
};

#define JETI_SENSOR_COUNT (sizeof(jeti_sensors) / sizeof(jeti_sensors[0]))

static uint8_t jeti_frame_count;

// CRC8, polynomial X^8 + X^2 + X + 1, from the Jeti EX protocol documentation
static uint8_t jeti_crc8(uint8_t* buf, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t c = 2; c < len; c++) {
        crc ^= buf[c];
        for (uint8_t i = 0; i < 8; i++) {
            crc = (crc & 0x80) ? (uint8_t)(0x07 ^ (crc << 1)) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static uint8_t jeti_str_len(const char* str)
{
    uint8_t len = 0;
    while (str[len] != '\0') {
        len++;
    }
    return len;
}

// Encode a sensor value in Jeti EX format, returns the number of bytes written
static uint8_t jeti_encode_value(uint8_t* buf, int32_t value, uint8_t data_type, uint8_t precision)
{
    uint8_t prec_bits = precision << 5;
    uint8_t sign = (value < 0) ? 0x80 : 0x00;
    uint32_t magnitude = (value < 0) ? (uint32_t)(-value) : (uint32_t)value;

    switch (data_type) {
    case JETI_TYPE_14b:
        if (magnitude > 0x1FFF) {
            magnitude = 0x1FFF;
        }
        buf[0] = magnitude & 0xFF;
        buf[1] = ((magnitude >> 8) & 0x1F) | sign | prec_bits;
        return 2;
    case JETI_TYPE_22b:
    default:
        if (magnitude > 0x1FFFFF) {
            magnitude = 0x1FFFFF;
        }
        buf[0] = magnitude & 0xFF;
        buf[1] = (magnitude >> 8) & 0xFF;
        buf[2] = ((magnitude >> 16) & 0x1F) | sign | prec_bits;
        return 3;
    }
}

// Text frame carrying the sensor device name (frame 0)
static uint8_t jeti_make_name_frame(uint8_t* buf)
{
    uint8_t n = 8;
    uint8_t name_len = jeti_str_len(JETI_SENSOR_NAME);

    buf[n++] = 0; // id 0 = device name
    buf[n++] = name_len << 3; // description length, no unit
    for (uint8_t i = 0; i < name_len; i++) {
        buf[n++] = JETI_SENSOR_NAME[i];
    }
    return n;
}

// Text frame carrying one sensor's dictionary entry (id, label and unit)
static uint8_t jeti_make_dict_frame(uint8_t* buf, uint8_t sensor_idx)
{
    uint8_t n = 8;
    const jeti_sensor_t* sensor = &jeti_sensors[sensor_idx];
    uint8_t label_len = jeti_str_len(sensor->label);
    uint8_t unit_len = jeti_str_len(sensor->unit);

    buf[n++] = sensor->id;
    buf[n++] = (label_len << 3) | unit_len; // 5 bit label length, 3 bit unit length
    for (uint8_t i = 0; i < label_len; i++) {
        buf[n++] = sensor->label[i];
    }
    for (uint8_t i = 0; i < unit_len; i++) {
        buf[n++] = sensor->unit[i];
    }
    return n;
}

// Data frame carrying all sensor values
static uint8_t jeti_make_value_frame(uint8_t* buf, int8_t temp, uint16_t voltage,
    uint16_t current, uint16_t consumption, uint16_t e_rpm)
{
    uint8_t n = 8;
    int32_t values[JETI_ID_COUNT];

    values[0] = 0; // id 0 is the device name, not sent as a value
    values[JETI_ID_TEMP] = temp;
    values[JETI_ID_VOLT] = voltage; // centivolts -> volts with 2 decimals
    values[JETI_ID_CURR] = current; // centiamps -> amps with 2 decimals
    values[JETI_ID_CONS] = consumption;
    values[JETI_ID_RPM] = (uint32_t)e_rpm * 100; // e_rpm is in hundreds

    for (uint8_t i = 0; i < JETI_SENSOR_COUNT; i++) {
        const jeti_sensor_t* sensor = &jeti_sensors[i];
        buf[n++] = (sensor->id << 4) | (sensor->data_type & 0x0F); // 4 bit id, 4 bit data type
        n += jeti_encode_value(&buf[n], values[sensor->id], sensor->data_type, sensor->precision);
    }
    return n;
}

uint8_t makeJetiTelemPackage(int8_t temp, uint16_t voltage, uint16_t current,
    uint16_t consumption, uint16_t e_rpm)
{
    uint8_t n;

    // Alternate the frame type: device name, then one dictionary frame per
    // sensor, then value frames until the dictionary repeats.
    if (jeti_frame_count == 0) {
        n = jeti_make_name_frame(aTxBuffer);
        aTxBuffer[2] = JETI_EX_PKT_TEXT;
    } else if ((jeti_frame_count % 2) == 0 && (jeti_frame_count / 2) <= JETI_SENSOR_COUNT) {
        n = jeti_make_dict_frame(aTxBuffer, (jeti_frame_count / 2) - 1);
        aTxBuffer[2] = JETI_EX_PKT_TEXT;
    } else {
        n = jeti_make_value_frame(aTxBuffer, temp, voltage, current, consumption, e_rpm);
        aTxBuffer[2] = JETI_EX_PKT_DATA;
    }
    jeti_frame_count++;

    aTxBuffer[0] = JETI_EX_SEPARATOR;
    aTxBuffer[1] = JETI_EX_MARKER;
    aTxBuffer[2] |= n - 2; // frame length without separator and marker
    aTxBuffer[3] = JETI_EX_MANUFACTURER_LO;
    aTxBuffer[4] = JETI_EX_MANUFACTURER_HI;
    aTxBuffer[5] = JETI_EX_DEVICE_LO;
    aTxBuffer[6] = JETI_EX_DEVICE_HI;
    aTxBuffer[7] = 0x00; // reserved

    aTxBuffer[n] = jeti_crc8(aTxBuffer, n);

    return n + 1;
}
