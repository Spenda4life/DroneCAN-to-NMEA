/**
 * test/test_multiframe/test_multiframe.cpp
 *
 * Embedded integration tests for multi-frame DroneCAN transfer reassembly.
 * Run with: pio test -e test_embedded
 *
 * Fix2 messages are 36+ bytes and must be split across multiple CAN frames.
 * The existing DSDL tests bypass canard entirely (injecting via
 * dronecan_test_inject), so multi-frame reassembly through
 * canardHandleRxFrame() is never exercised. These tests close that gap.
 *
 * Strategy: manually construct CAN frames following the DroneCAN multi-frame
 * transport protocol (CRC, toggle bits, start/end markers) and feed them
 * through dronecan_handle_frame(). Verify g_sensors is correctly populated
 * after the final frame.
 */

#include <Arduino.h>
#include <unity.h>
#include <string.h>
#include <stdint.h>

#include "dronecan_handler.h"
#include "config.h"

extern "C" {
#include "canard.h"
}

// ---------------------------------------------------------------------------
// CRC-16/CCITT helpers (same algorithm as libcanard)
// ---------------------------------------------------------------------------

static uint16_t crc_add_byte(uint16_t crc, uint8_t byte) {
    crc ^= (uint16_t)((uint16_t)byte << 8U);
    for (uint8_t j = 0; j < 8; j++) {
        if (crc & 0x8000U) {
            crc = (uint16_t)((uint16_t)(crc << 1U) ^ 0x1021U);
        } else {
            crc = (uint16_t)(crc << 1U);
        }
    }
    return crc;
}

static uint16_t crc_add_signature(uint16_t crc, uint64_t signature) {
    for (uint16_t shift = 0; shift < 64; shift = (uint16_t)(shift + 8U)) {
        crc = crc_add_byte(crc, (uint8_t)(signature >> shift));
    }
    return crc;
}

static uint16_t crc_add(uint16_t crc, const uint8_t* data, size_t len) {
    while (len--) {
        crc = crc_add_byte(crc, *data++);
    }
    return crc;
}

// ---------------------------------------------------------------------------
// Bit-level encode helpers (same as test_dsdl.cpp)
// ---------------------------------------------------------------------------

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

static void set_bits_u(uint8_t* buf, uint32_t bit_offset, uint8_t bit_len, uint64_t value) {
    set_bits(buf, bit_offset, bit_len, (int64_t)value);
}

static uint16_t float_to_f16(float f) {
    uint32_t bits;
    memcpy(&bits, &f, sizeof(bits));
    uint32_t sign     = (bits >> 31) & 0x1u;
    uint32_t exponent = (bits >> 23) & 0xFFu;
    uint32_t mantissa = bits & 0x7FFFFFu;

    if (exponent == 0 && mantissa == 0) return (uint16_t)(sign << 15);
    if (exponent == 0xFFu) return (uint16_t)((sign << 15) | (0x1Fu << 10) | (mantissa ? 1 : 0));
    int32_t exp16 = (int32_t)exponent - 127 + 15;
    if (exp16 >= 31) return (uint16_t)((sign << 15) | (0x1Fu << 10));
    if (exp16 <= 0) return (uint16_t)(sign << 15);
    uint16_t mantissa16 = (uint16_t)(mantissa >> 13);
    return (uint16_t)((sign << 15) | ((uint16_t)exp16 << 10) | mantissa16);
}

// ---------------------------------------------------------------------------
// Multi-frame transfer builder
// ---------------------------------------------------------------------------

/**
 * Split a payload into multi-frame CAN frames per DroneCAN transport spec.
 * Returns the number of frames written into out_frames[].
 */
static int build_multiframe_transfer(
    uint16_t data_type_id,
    uint8_t  source_node_id,
    uint8_t  transfer_id,
    uint64_t data_type_signature,
    const uint8_t* payload,
    uint16_t payload_len,
    CanardCANFrame* out_frames,
    int max_frames)
{
    // Compute CRC over signature + payload
    uint16_t crc = crc_add_signature(0xFFFFU, data_type_signature);
    crc = crc_add(crc, payload, payload_len);

    // CAN ID for broadcast: priority(5) | data_type_id(16) | source_node_id(7)
    uint32_t can_id = ((uint32_t)CANARD_TRANSFER_PRIORITY_MEDIUM << 24U) |
                      ((uint32_t)data_type_id << 8U) |
                      (uint32_t)source_node_id;

    int frame_idx = 0;
    uint16_t data_index = 0;
    uint8_t toggle = 0;

    while (data_index < payload_len) {
        if (frame_idx >= max_frames) return -1;

        CanardCANFrame* f = &out_frames[frame_idx];
        memset(f, 0, sizeof(*f));
        f->id = can_id | CANARD_CAN_FRAME_EFF;

        uint8_t i = 0;
        if (data_index == 0) {
            // First frame: prepend CRC (little-endian)
            f->data[0] = (uint8_t)(crc & 0xFF);
            f->data[1] = (uint8_t)(crc >> 8);
            i = 2;
        }

        // Fill payload bytes (max 7 bytes per frame, 1 reserved for tail)
        for (; i < 7 && data_index < payload_len; i++, data_index++) {
            f->data[i] = payload[data_index];
        }

        // Tail byte
        uint8_t sot = (frame_idx == 0) ? 0x80 : 0x00;
        uint8_t eot = (data_index == payload_len) ? 0x40 : 0x00;
        f->data[i] = (uint8_t)(sot | eot | ((uint32_t)toggle << 5U) | (transfer_id & 0x1F));
        f->data_len = i + 1;

        frame_idx++;
        toggle ^= 1;
    }

    return frame_idx;
}

// ---------------------------------------------------------------------------
// Fix2 payload builder (same as test_dsdl.cpp)
// ---------------------------------------------------------------------------

static void build_fix2_payload(uint8_t* buf, size_t buf_size,
                                double lat, double lon, float alt,
                                uint8_t sats, uint8_t fix_type) {
    memset(buf, 0, buf_size);

    uint64_t gnss_ts = 1700000000000000ULL;
    set_bits_u(buf, 40, 40, gnss_ts);
    set_bits_u(buf, 80, 3, 2u);     // gnss_time_standard = UTC
    set_bits_u(buf, 88, 8, 18u);    // num_leap_seconds

    set_bits(buf, 96,  37, (int64_t)(lon * 1e8));
    set_bits(buf, 133, 37, (int64_t)(lat * 1e8));
    set_bits(buf, 197, 27, (int64_t)(alt * 1000.0));

    set_bits_u(buf, 224, 16, float_to_f16(0.0f));
    set_bits_u(buf, 240, 16, float_to_f16(0.0f));
    set_bits_u(buf, 256, 16, float_to_f16(0.0f));
    set_bits_u(buf, 272, 6, sats);
    set_bits_u(buf, 280, 4, fix_type);
}

// Fix2 data type signature (signature is skipped in should_accept_transfer,
// set to 0, but we need to match what canard expects for CRC validation).
// Since should_accept_transfer sets *out_data_type_signature = 0, we use 0.
static const uint64_t FIX2_SIGNATURE = 0;
static const uint16_t FIX2_DTID = 1063;

// ---------------------------------------------------------------------------
// setUp / tearDown
// ---------------------------------------------------------------------------

void setUp(void) {
    dronecan_init();
    memset(&g_sensors, 0, sizeof(g_sensors));
}

void tearDown(void) {}

// ---------------------------------------------------------------------------
// Test 1: Multi-frame Fix2 reassembly produces correct lat/lon/sats
// ---------------------------------------------------------------------------

void test_multiframe_fix2_reassembly(void) {
    uint8_t payload[40];
    build_fix2_payload(payload, sizeof(payload),
                       -33.8688, 151.2093, 50.0f, 10, 3);

    CanardCANFrame frames[16];
    int nframes = build_multiframe_transfer(
        FIX2_DTID, 42, 0, FIX2_SIGNATURE,
        payload, sizeof(payload), frames, 16);
    TEST_ASSERT_TRUE(nframes > 1);  // Must be multi-frame

    // Feed all frames through canard
    uint64_t ts = 1000000ULL;
    for (int i = 0; i < nframes; i++) {
        dronecan_handle_frame(&frames[i], ts);
        ts += 1000;
    }

    // Verify decoded values
    TEST_ASSERT_TRUE(g_sensors.fix_valid);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, -33.8688f, (float)g_sensors.lat_deg);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 151.2093f, (float)g_sensors.lon_deg);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 50.0f, g_sensors.alt_m);
    TEST_ASSERT_EQUAL(10, g_sensors.num_sats);
    TEST_ASSERT_EQUAL(3, g_sensors.fix_type);
}

// ---------------------------------------------------------------------------
// Test 2: Incomplete transfer (only first frame) does not update g_sensors
// ---------------------------------------------------------------------------

void test_multiframe_incomplete_transfer_ignored(void) {
    uint8_t payload[40];
    build_fix2_payload(payload, sizeof(payload),
                       -33.8688, 151.2093, 50.0f, 10, 3);

    CanardCANFrame frames[16];
    int nframes = build_multiframe_transfer(
        FIX2_DTID, 42, 1, FIX2_SIGNATURE,
        payload, sizeof(payload), frames, 16);
    TEST_ASSERT_TRUE(nframes > 1);

    // Feed only the first frame
    dronecan_handle_frame(&frames[0], 2000000ULL);

    // g_sensors should remain at initial state (no partial decode)
    TEST_ASSERT_FALSE(g_sensors.fix_valid);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, (float)g_sensors.lat_deg);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, (float)g_sensors.lon_deg);
}

// ---------------------------------------------------------------------------
// Test 3: Corrupted CRC causes transfer to be rejected
// ---------------------------------------------------------------------------

void test_multiframe_wrong_crc_rejected(void) {
    uint8_t payload[40];
    build_fix2_payload(payload, sizeof(payload),
                       -33.8688, 151.2093, 50.0f, 10, 3);

    CanardCANFrame frames[16];
    int nframes = build_multiframe_transfer(
        FIX2_DTID, 42, 2, FIX2_SIGNATURE,
        payload, sizeof(payload), frames, 16);
    TEST_ASSERT_TRUE(nframes > 1);

    // Corrupt the CRC in the first frame (bytes 0 and 1)
    frames[0].data[0] ^= 0xFF;
    frames[0].data[1] ^= 0xFF;

    // Feed all frames
    uint64_t ts = 3000000ULL;
    for (int i = 0; i < nframes; i++) {
        dronecan_handle_frame(&frames[i], ts);
        ts += 1000;
    }

    // g_sensors should remain untouched (canard rejects bad CRC)
    TEST_ASSERT_FALSE(g_sensors.fix_valid);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, (float)g_sensors.lat_deg);
}

// ---------------------------------------------------------------------------
// Arduino / Unity entry points
// ---------------------------------------------------------------------------

void setup(void) {
    delay(2000);
    UNITY_BEGIN();

    RUN_TEST(test_multiframe_fix2_reassembly);
    RUN_TEST(test_multiframe_incomplete_transfer_ignored);
    RUN_TEST(test_multiframe_wrong_crc_rejected);

    UNITY_END();
}

void loop(void) {}
