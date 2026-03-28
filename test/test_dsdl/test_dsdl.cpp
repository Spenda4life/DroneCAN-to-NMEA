/**
 * test/test_dsdl/test_dsdl.cpp
 *
 * Embedded unit tests for the DroneCAN DSDL decoders in dronecan_handler.cpp.
 * These tests run on the ESP32 target (pio test -e test_embedded).
 *
 * Strategy: build synthetic payloads using the same little-endian bit-packing
 * convention that libcanard's canardDecodeScalar uses, then feed them through
 * dronecan_test_inject() and verify the resulting g_sensors values.
 */

#include <Arduino.h>
#include <unity.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "dronecan_handler.h"

// ---------------------------------------------------------------------------
// Bit-level encode helper (matches libcanard little-endian bit order)
// ---------------------------------------------------------------------------

/**
 * Write `bit_len` bits of `value` into `buf` starting at `bit_offset`.
 * Bits are packed LSB-first within each byte — identical to canardDecodeScalar
 * convention so that what we write here is what the decoder will read back.
 */
static void set_bits(uint8_t* buf, uint32_t bit_offset, uint8_t bit_len, int64_t value) {
    for (uint8_t i = 0; i < bit_len; i++) {
        uint32_t abs_bit = bit_offset + i;
        uint32_t byte_idx = abs_bit / 8u;
        uint8_t  bit_idx  = (uint8_t)(abs_bit % 8u);
        if (value & (1LL << i)) {
            buf[byte_idx] |=  (uint8_t)(1u << bit_idx);
        } else {
            buf[byte_idx] &= (uint8_t)~(1u << bit_idx);
        }
    }
}

// Convenience wrapper for unsigned values.
static void set_bits_u(uint8_t* buf, uint32_t bit_offset, uint8_t bit_len, uint64_t value) {
    set_bits(buf, bit_offset, bit_len, (int64_t)value);
}

// ---------------------------------------------------------------------------
// Float16 encode helper
// ---------------------------------------------------------------------------

/**
 * Encode a float as IEEE 754 float16.
 * Only handles the normal number range needed for test values.
 * Uses the same bit layout that f16_to_float() in dronecan_handler.cpp decodes.
 */
static uint16_t float_to_f16(float f) {
    uint32_t bits;
    memcpy(&bits, &f, sizeof(bits));
    uint32_t sign     = (bits >> 31) & 0x1u;
    uint32_t exponent = (bits >> 23) & 0xFFu;
    uint32_t mantissa = bits & 0x7FFFFFu;

    if (exponent == 0 && mantissa == 0) {
        return (uint16_t)(sign << 15);  // zero
    }
    if (exponent == 0xFFu) {
        // Inf or NaN
        return (uint16_t)((sign << 15) | (0x1Fu << 10) | (mantissa ? 1 : 0));
    }

    int32_t exp16 = (int32_t)exponent - 127 + 15;
    if (exp16 >= 31) {
        // Overflow → infinity
        return (uint16_t)((sign << 15) | (0x1Fu << 10));
    }
    if (exp16 <= 0) {
        // Underflow → zero (subnormals not needed for these tests)
        return (uint16_t)(sign << 15);
    }
    uint16_t mantissa16 = (uint16_t)(mantissa >> 13);
    return (uint16_t)((sign << 15) | ((uint16_t)exp16 << 10) | mantissa16);
}

// ---------------------------------------------------------------------------
// setUp / tearDown
// ---------------------------------------------------------------------------

void setUp(void) {
    dronecan_init();
    memset(&g_sensors, 0, sizeof(g_sensors));
}

void tearDown(void) {}

// ---------------------------------------------------------------------------
// Fix2 decode tests  (DTID 1063)
// ---------------------------------------------------------------------------

/**
 * Synthesise a Fix2 payload representing Sydney, Australia:
 *   lat  = -33.8688°  (lat_deg_1e8 = -338688000)
 *   lon  = +151.2093° (lon_deg_1e8 = +1512093000)
 *   alt  = 50 m       (height_msl_mm = 50000)
 *   sats = 10
 *   fix  = 3 (3-D)
 *   gnss_time_standard = 2 (UTC)
 *   gnss_timestamp = 1700000000000000 µs (2023-11-14 22:13:20 UTC)
 *
 * Bit layout (from decode_fix2 in dronecan_handler.cpp):
 *   [ 0:39]  unused (e.g. uavcan_timestamp)
 *   [40:79]  gnss_timestamp   (40 bits, unsigned)
 *   [80:82]  gnss_time_standard (3 bits, unsigned)
 *   [83:87]  unused/reserved (5 bits to reach byte boundary at 88)
 *   [88:95]  num_leap_seconds (8 bits, unsigned)
 *   [96:132] longitude_deg_1e8 (37 bits, signed)
 *   [133:169] latitude_deg_1e8 (37 bits, signed)
 *   [170:196] unused (height_ellipsoid, 27 bits, but decoder reads height_msl at 197)
 *   [197:223] height_msl_mm (27 bits, signed)
 *   [224:239] ned_velocity[0] north (float16)
 *   [240:255] ned_velocity[1] east  (float16)
 *   [256:271] ned_velocity[2] down  (float16)
 *   [272:277] sats_used (6 bits)
 *   [280:283] status/fix_type (4 bits)
 *
 * Total: at least 284 bits → 36 bytes.
 */
static void build_fix2_payload(uint8_t* buf, size_t buf_size) {
    memset(buf, 0, buf_size);

    const uint64_t gnss_ts = 1700000000000000ULL;
    set_bits_u(buf, 40,  40, gnss_ts);

    set_bits_u(buf, 80,   3, 2u);    // gnss_time_standard = UTC
    set_bits_u(buf, 88,   8, 18u);   // num_leap_seconds = 18 (not used for UTC path)

    // longitude: +151.2093° → 1512093000
    set_bits(buf, 96,  37, (int64_t)1512093000LL);
    // latitude:  -33.8688° → -338688000
    set_bits(buf, 133, 37, (int64_t)-338688000LL);

    // height_msl_mm: 50000 (50 m)
    set_bits(buf, 197, 27, (int64_t)50000LL);

    // NED velocities — zero
    set_bits_u(buf, 224, 16, float_to_f16(0.0f));
    set_bits_u(buf, 240, 16, float_to_f16(0.0f));
    set_bits_u(buf, 256, 16, float_to_f16(0.0f));

    // sats_used = 10
    set_bits_u(buf, 272, 6, 10u);

    // fix_status = 3
    set_bits_u(buf, 280, 4, 3u);
}

void test_fix2_latitude(void) {
    uint8_t buf[40];
    build_fix2_payload(buf, sizeof(buf));
    dronecan_test_inject(1063, buf, sizeof(buf));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, -33.8688f, (float)g_sensors.lat_deg);
}

void test_fix2_longitude(void) {
    uint8_t buf[40];
    build_fix2_payload(buf, sizeof(buf));
    dronecan_test_inject(1063, buf, sizeof(buf));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 151.2093f, (float)g_sensors.lon_deg);
}

void test_fix2_altitude(void) {
    uint8_t buf[40];
    build_fix2_payload(buf, sizeof(buf));
    dronecan_test_inject(1063, buf, sizeof(buf));
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 50.0f, g_sensors.alt_m);
}

void test_fix2_num_sats(void) {
    uint8_t buf[40];
    build_fix2_payload(buf, sizeof(buf));
    dronecan_test_inject(1063, buf, sizeof(buf));
    TEST_ASSERT_EQUAL(10, g_sensors.num_sats);
}

void test_fix2_fix_type(void) {
    uint8_t buf[40];
    build_fix2_payload(buf, sizeof(buf));
    dronecan_test_inject(1063, buf, sizeof(buf));
    TEST_ASSERT_EQUAL(3, g_sensors.fix_type);
}

void test_fix2_fix_valid(void) {
    uint8_t buf[40];
    build_fix2_payload(buf, sizeof(buf));
    dronecan_test_inject(1063, buf, sizeof(buf));
    TEST_ASSERT_TRUE(g_sensors.fix_valid);
}

void test_fix2_timestamp_utc(void) {
    uint8_t buf[40];
    build_fix2_payload(buf, sizeof(buf));
    dronecan_test_inject(1063, buf, sizeof(buf));
    // gnss_time_standard=UTC → timestamp_usec == gnss_ts directly
    TEST_ASSERT_EQUAL_UINT64(1700000000000000ULL, g_sensors.timestamp_usec);
}

void test_fix2_no_fix_when_status_zero(void) {
    uint8_t buf[40];
    memset(buf, 0, sizeof(buf));
    // All zeros → fix_status = 0 → fix_valid = false
    dronecan_test_inject(1063, buf, sizeof(buf));
    TEST_ASSERT_FALSE(g_sensors.fix_valid);
    TEST_ASSERT_EQUAL(0, g_sensors.fix_type);
}

// ---------------------------------------------------------------------------
// Auxiliary DOP decode tests  (DTID 1062)
// ---------------------------------------------------------------------------

/**
 * Auxiliary payload layout (from decode_auxiliary):
 *   [0:15]  pdop (float16)
 *   [16:31] hdop (float16)
 *   [32:47] vdop (float16)
 */
static void build_auxiliary_payload(uint8_t* buf, float pdop, float hdop, float vdop) {
    memset(buf, 0, 6);
    set_bits_u(buf,  0, 16, float_to_f16(pdop));
    set_bits_u(buf, 16, 16, float_to_f16(hdop));
    set_bits_u(buf, 32, 16, float_to_f16(vdop));
}

void test_auxiliary_pdop(void) {
    uint8_t buf[6];
    build_auxiliary_payload(buf, 1.5f, 1.2f, 2.0f);
    dronecan_test_inject(1062, buf, sizeof(buf));
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 1.5f, g_sensors.pdop);
}

void test_auxiliary_hdop(void) {
    uint8_t buf[6];
    build_auxiliary_payload(buf, 1.5f, 1.2f, 2.0f);
    dronecan_test_inject(1062, buf, sizeof(buf));
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 1.2f, g_sensors.hdop);
}

void test_auxiliary_vdop(void) {
    uint8_t buf[6];
    build_auxiliary_payload(buf, 1.5f, 1.2f, 2.0f);
    dronecan_test_inject(1062, buf, sizeof(buf));
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 2.0f, g_sensors.vdop);
}

void test_auxiliary_known_f16_values(void) {
    // Use exact IEEE 754 float16 hex values placed directly
    // 1.5 = 0x3E00, 1.2 ≈ 0x3CCD, 2.0 = 0x4000
    uint8_t buf[6];
    memset(buf, 0, sizeof(buf));
    // pdop = 0x3E00 → bytes [0]=0x00, [1]=0x3E
    buf[0] = 0x00; buf[1] = 0x3E;
    // hdop = 0x3CCD → bytes [2]=0xCD, [3]=0x3C
    buf[2] = 0xCD; buf[3] = 0x3C;
    // vdop = 0x4000 → bytes [4]=0x00, [5]=0x40
    buf[4] = 0x00; buf[5] = 0x40;
    dronecan_test_inject(1062, buf, sizeof(buf));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.5f,  g_sensors.pdop);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.2f,  g_sensors.hdop);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 2.0f,  g_sensors.vdop);
}

// ---------------------------------------------------------------------------
// Magnetometer decode tests  (DTID 1001)
// ---------------------------------------------------------------------------

/**
 * MagneticFieldStrength2 payload layout (from decode_mag):
 *   [0:7]   sensor_id (8 bits, unused)
 *   [8:23]  mag_x (float16)
 *   [24:39] mag_y (float16)
 *   [40:55] mag_z (float16)
 */
static void build_mag_payload(uint8_t* buf, uint8_t sensor_id,
                               float mag_x, float mag_y, float mag_z) {
    memset(buf, 0, 7);
    set_bits_u(buf,  0,  8, sensor_id);
    set_bits_u(buf,  8, 16, float_to_f16(mag_x));
    set_bits_u(buf, 24, 16, float_to_f16(mag_y));
    set_bits_u(buf, 40, 16, float_to_f16(mag_z));
}

void test_mag_valid_set(void) {
    uint8_t buf[7];
    build_mag_payload(buf, 0, 0.3f, 0.1f, -0.5f);
    dronecan_test_inject(1001, buf, sizeof(buf));
    TEST_ASSERT_TRUE(g_sensors.mag_valid);
}

void test_mag_x_value(void) {
    uint8_t buf[7];
    build_mag_payload(buf, 0, 0.3f, 0.1f, -0.5f);
    dronecan_test_inject(1001, buf, sizeof(buf));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.3f, g_sensors.mag_x);
}

void test_mag_y_value(void) {
    uint8_t buf[7];
    build_mag_payload(buf, 0, 0.3f, 0.1f, -0.5f);
    dronecan_test_inject(1001, buf, sizeof(buf));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.1f, g_sensors.mag_y);
}

void test_mag_z_value(void) {
    uint8_t buf[7];
    build_mag_payload(buf, 0, 0.3f, 0.1f, -0.5f);
    dronecan_test_inject(1001, buf, sizeof(buf));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -0.5f, g_sensors.mag_z);
}

void test_mag_sensor_id_ignored(void) {
    // sensor_id should be skipped; mag values should still decode correctly
    uint8_t buf[7];
    build_mag_payload(buf, 42, 0.5f, -0.2f, 0.1f);
    dronecan_test_inject(1001, buf, sizeof(buf));
    TEST_ASSERT_TRUE(g_sensors.mag_valid);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.5f,  g_sensors.mag_x);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -0.2f, g_sensors.mag_y);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.1f,  g_sensors.mag_z);
}

void test_mag_known_f16_values(void) {
    // Use exact float16 hex values:
    // 0.3 ≈ 0x34CD → bytes 0xCD 0x34
    // 0.1 ≈ 0x2E66 → bytes 0x66 0x2E
    // -0.5 = 0xB800 → bytes 0x00 0xB8
    uint8_t buf[7];
    memset(buf, 0, sizeof(buf));
    buf[0] = 0;           // sensor_id = 0
    // mag_x at bits [8:23]: bytes 1,2
    buf[1] = 0xCD; buf[2] = 0x34;
    // mag_y at bits [24:39]: bytes 3,4
    buf[3] = 0x66; buf[4] = 0x2E;
    // mag_z at bits [40:55]: bytes 5,6
    buf[5] = 0x00; buf[6] = 0xB8;
    dronecan_test_inject(1001, buf, sizeof(buf));
    TEST_ASSERT_TRUE(g_sensors.mag_valid);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.3f,  g_sensors.mag_x);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.1f,  g_sensors.mag_y);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -0.5f, g_sensors.mag_z);
}

// ---------------------------------------------------------------------------
// Unknown DTID — should be silently ignored
// ---------------------------------------------------------------------------

void test_unknown_dtid_ignored(void) {
    // Inject a payload with an unregistered DTID; g_sensors must not change.
    uint8_t buf[8] = {0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD, 0xBE, 0xEF};
    g_sensors.fix_valid = false;
    dronecan_test_inject(9999, buf, sizeof(buf));
    TEST_ASSERT_FALSE(g_sensors.fix_valid);  // unchanged
}

// ---------------------------------------------------------------------------
// Arduino / Unity entry points
// ---------------------------------------------------------------------------

void setup(void) {
    delay(2000);   // allow USB-serial to connect before test output starts
    UNITY_BEGIN();

    // Fix2
    RUN_TEST(test_fix2_latitude);
    RUN_TEST(test_fix2_longitude);
    RUN_TEST(test_fix2_altitude);
    RUN_TEST(test_fix2_num_sats);
    RUN_TEST(test_fix2_fix_type);
    RUN_TEST(test_fix2_fix_valid);
    RUN_TEST(test_fix2_timestamp_utc);
    RUN_TEST(test_fix2_no_fix_when_status_zero);

    // Auxiliary
    RUN_TEST(test_auxiliary_pdop);
    RUN_TEST(test_auxiliary_hdop);
    RUN_TEST(test_auxiliary_vdop);
    RUN_TEST(test_auxiliary_known_f16_values);

    // Magnetometer
    RUN_TEST(test_mag_valid_set);
    RUN_TEST(test_mag_x_value);
    RUN_TEST(test_mag_y_value);
    RUN_TEST(test_mag_z_value);
    RUN_TEST(test_mag_sensor_id_ignored);
    RUN_TEST(test_mag_known_f16_values);

    // Edge cases
    RUN_TEST(test_unknown_dtid_ignored);

    UNITY_END();
}

void loop(void) {}
