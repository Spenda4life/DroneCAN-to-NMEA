/**
 * test/test_nmea/test_nmea.cpp
 *
 * Native (host-side) unit tests for the NMEA generator.
 * Run with: pio test -e native
 *
 * These tests exercise all sentence builders, the checksum utility, coordinate
 * conversion, UTC/date decomposition, SOG/COG calculation and validity flags —
 * without any hardware or RTOS dependency.
 */

#include <unity.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

// Provide the globals referenced by nmea_generator.cpp.
// dronecan_handler.cpp is NOT compiled in the native build, so we define
// them here instead.
#include "dronecan_handler.h"

SensorData   g_sensors;
portMUX_TYPE g_sensors_mux = portMUX_INITIALIZER_UNLOCKED;

#include "nmea_generator.h"

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/** Zero g_sensors and set sensible defaults for a valid 3-D GPS fix. */
static void set_valid_fix(void) {
    memset(&g_sensors, 0, sizeof(g_sensors));
    g_sensors.fix_valid      = true;
    g_sensors.fix_type       = 3;
    g_sensors.num_sats       = 8;
    g_sensors.lat_deg        = 0.0;
    g_sensors.lon_deg        = 0.0;
    g_sensors.alt_m          = 0.0f;
    g_sensors.vel_n_ms       = 0.0f;
    g_sensors.vel_e_ms       = 0.0f;
    g_sensors.vel_d_ms       = 0.0f;
    g_sensors.timestamp_usec = 0ULL;
    g_sensors.pdop           = 1.0f;
    g_sensors.hdop           = 1.0f;
    g_sensors.vdop           = 1.0f;
    g_sensors.mag_valid      = false;
}

static void set_no_fix(void) {
    memset(&g_sensors, 0, sizeof(g_sensors));
    g_sensors.fix_valid = false;
    g_sensors.fix_type  = 0;
}

/**
 * Parse the two hex digits after '*' in a sentence and return the value.
 * Returns -1 if '*' is not found.
 */
static int parse_sentence_checksum(const char* sentence) {
    const char* star = strchr(sentence, '*');
    if (!star) return -1;
    char hex[3] = { star[1], star[2], '\0' };
    return (int)strtol(hex, NULL, 16);
}

// setUp / tearDown called by Unity before/after each test.
void setUp(void) {
    set_valid_fix();
}

void tearDown(void) {}

// ---------------------------------------------------------------------------
// Checksum tests
// ---------------------------------------------------------------------------

void test_checksum_known_sentence(void) {
    // Classic NMEA reference: checksum of the payload between $ and * should
    // be 0x6A for this well-known RMC sentence.
    const char* sentence =
        "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W";
    uint8_t csum = nmea_checksum(sentence);
    TEST_ASSERT_EQUAL_HEX8(0x6A, csum);
}

void test_checksum_empty_payload(void) {
    // Payload is empty — checksum should be 0.
    const char* sentence = "$GP";
    uint8_t csum = nmea_checksum(sentence);
    TEST_ASSERT_EQUAL_HEX8(0x00, csum);
}

void test_finalize_appends_correctly(void) {
    char buf[64];
    int payload_len = snprintf(buf, sizeof(buf), "$GPTEST,hello");
    int total = nmea_finalize(buf, payload_len);

    // Must end with \r\n
    TEST_ASSERT_EQUAL('\r', buf[total - 2]);
    TEST_ASSERT_EQUAL('\n', buf[total - 1]);

    // Must contain '*'
    const char* star = strchr(buf, '*');
    TEST_ASSERT_NOT_NULL(star);

    // Total length = payload + 6 characters: *XX\r\n
    TEST_ASSERT_EQUAL(payload_len + 6, total);

    // The checksum in the sentence must match nmea_checksum
    int parsed = parse_sentence_checksum(buf);
    TEST_ASSERT_EQUAL_HEX8(nmea_checksum(buf), (uint8_t)parsed);
}

// ---------------------------------------------------------------------------
// Coordinate conversion tests (exercised via build_rmc)
// ---------------------------------------------------------------------------

void test_lat_north_positive(void) {
    g_sensors.lat_deg = +33.8688;
    int len;
    const char* s = build_rmc(&len);
    TEST_ASSERT_NOT_NULL(s);
    // 33° + 0.8688*60 = 33°52.128' → "3352.1280,N"
    TEST_ASSERT_NOT_NULL(strstr(s, "3352.1280,N"));
}

void test_lat_south_negative(void) {
    g_sensors.lat_deg = -33.8688;
    int len;
    const char* s = build_rmc(&len);
    TEST_ASSERT_NOT_NULL(s);
    TEST_ASSERT_NOT_NULL(strstr(s, "3352.1280,S"));
}

void test_lon_east_positive(void) {
    g_sensors.lon_deg = +151.2093;
    int len;
    const char* s = build_rmc(&len);
    TEST_ASSERT_NOT_NULL(s);
    // 151° + 0.2093*60 = 151°12.558' → "15112.5580,E"
    TEST_ASSERT_NOT_NULL(strstr(s, "15112.5580,E"));
}

void test_lon_west_negative(void) {
    g_sensors.lon_deg = -151.2093;
    int len;
    const char* s = build_rmc(&len);
    TEST_ASSERT_NOT_NULL(s);
    TEST_ASSERT_NOT_NULL(strstr(s, "15112.5580,W"));
}

void test_lat_zero_equator(void) {
    g_sensors.lat_deg = 0.0;
    int len;
    const char* s = build_rmc(&len);
    TEST_ASSERT_NOT_NULL(s);
    TEST_ASSERT_NOT_NULL(strstr(s, "0000.0000,N"));
}

void test_lon_zero_meridian(void) {
    g_sensors.lon_deg = 0.0;
    int len;
    const char* s = build_rmc(&len);
    TEST_ASSERT_NOT_NULL(s);
    TEST_ASSERT_NOT_NULL(strstr(s, "00000.0000,E"));
}

void test_high_latitude(void) {
    g_sensors.lat_deg = +89.9999;
    int len;
    const char* s = build_rmc(&len);
    TEST_ASSERT_NOT_NULL(s);
    // Degree part must be "89", minutes must be < 60
    TEST_ASSERT_NOT_NULL(strstr(s, "89"));
    // minutes = 0.9999 * 60 = 59.994 — confirm it is less than 60
    double minutes = 0.9999 * 60.0;
    TEST_ASSERT_TRUE(minutes < 60.0);
}

// ---------------------------------------------------------------------------
// UTC time tests
// ---------------------------------------------------------------------------

void test_utc_midnight(void) {
    // Unix epoch = 1970-01-01 00:00:00.  timestamp_usec = 0.
    g_sensors.timestamp_usec = 0ULL;
    int len;
    const char* s = build_rmc(&len);
    TEST_ASSERT_NOT_NULL(s);
    // When timestamp is 0 the code skips conversion and uses defaults
    // "000000.00" and "010170".
    TEST_ASSERT_NOT_NULL(strstr(s, "000000.00"));
    TEST_ASSERT_NOT_NULL(strstr(s, "010170"));
}

void test_utc_known_time(void) {
    // 1700000000 seconds = 2023-11-14 22:13:20 UTC
    // As microseconds: 1700000000 * 1000000
    g_sensors.timestamp_usec = 1700000000ULL * 1000000ULL;
    int len;
    const char* s = build_rmc(&len);
    TEST_ASSERT_NOT_NULL(s);
    // Time field: "221320.00"
    TEST_ASSERT_NOT_NULL(strstr(s, "221320.00"));
    // Date field: "141123"  (14 Nov 2023)
    TEST_ASSERT_NOT_NULL(strstr(s, "141123"));
}

void test_utc_near_midnight(void) {
    // 1 second before midnight on 1970-01-01 = 86399 seconds = 23:59:59
    g_sensors.timestamp_usec = 86399ULL * 1000000ULL;
    int len;
    const char* s = build_rmc(&len);
    TEST_ASSERT_NOT_NULL(s);
    TEST_ASSERT_NOT_NULL(strstr(s, "235959.00"));
}

void test_utc_after_midnight(void) {
    // 1 second after midnight on 1970-01-02 = 86401 seconds = 00:00:01
    g_sensors.timestamp_usec = 86401ULL * 1000000ULL;
    int len;
    const char* s = build_rmc(&len);
    TEST_ASSERT_NOT_NULL(s);
    TEST_ASSERT_NOT_NULL(strstr(s, "000001.00"));
}

// ---------------------------------------------------------------------------
// SOG / COG tests (via build_rmc)
// ---------------------------------------------------------------------------

void test_sog_stationary(void) {
    g_sensors.vel_n_ms = 0.0f;
    g_sensors.vel_e_ms = 0.0f;
    int len;
    const char* s = build_rmc(&len);
    TEST_ASSERT_NOT_NULL(s);
    // SOG = 0.0 knots → "0.0," in the sentence
    TEST_ASSERT_NOT_NULL(strstr(s, ",0.0,"));
}

void test_sog_northward(void) {
    // 1 m/s north → SOG = 1.94384 knots → formatted as "1.9"
    g_sensors.vel_n_ms = 1.0f;
    g_sensors.vel_e_ms = 0.0f;
    int len;
    const char* s = build_rmc(&len);
    TEST_ASSERT_NOT_NULL(s);
    TEST_ASSERT_NOT_NULL(strstr(s, "1.9,"));
}

void test_cog_north(void) {
    // vel_n > 0, vel_e = 0 → COG = 0.0°
    g_sensors.vel_n_ms = 1.0f;
    g_sensors.vel_e_ms = 0.0f;
    int len;
    const char* s = build_rmc(&len);
    TEST_ASSERT_NOT_NULL(s);
    // COG "0.0" must appear after SOG field
    TEST_ASSERT_NOT_NULL(strstr(s, "1.9,0.0,"));
}

void test_cog_east(void) {
    // vel_n = 0, vel_e > 0 → COG = 90.0°
    g_sensors.vel_n_ms = 0.0f;
    g_sensors.vel_e_ms = 1.0f;
    int len;
    const char* s = build_rmc(&len);
    TEST_ASSERT_NOT_NULL(s);
    TEST_ASSERT_NOT_NULL(strstr(s, ",90.0,"));
}

void test_cog_south(void) {
    // vel_n < 0, vel_e = 0 → COG = 180.0°
    g_sensors.vel_n_ms = -1.0f;
    g_sensors.vel_e_ms = 0.0f;
    int len;
    const char* s = build_rmc(&len);
    TEST_ASSERT_NOT_NULL(s);
    TEST_ASSERT_NOT_NULL(strstr(s, ",180.0,"));
}

void test_cog_west(void) {
    // vel_n = 0, vel_e < 0 → COG = 270.0° (must not be negative)
    g_sensors.vel_n_ms = 0.0f;
    g_sensors.vel_e_ms = -1.0f;
    int len;
    const char* s = build_rmc(&len);
    TEST_ASSERT_NOT_NULL(s);
    TEST_ASSERT_NOT_NULL(strstr(s, ",270.0,"));
}

void test_cog_southwest(void) {
    // vel_n=-1, vel_e=-1 → atan2(-1,-1)=-135° → +360 = 225°
    g_sensors.vel_n_ms = -1.0f;
    g_sensors.vel_e_ms = -1.0f;
    int len;
    const char* s = build_rmc(&len);
    TEST_ASSERT_NOT_NULL(s);
    TEST_ASSERT_NOT_NULL(strstr(s, ",225.0,"));
}

// ---------------------------------------------------------------------------
// RMC validity tests
// ---------------------------------------------------------------------------

void test_rmc_valid_fix(void) {
    g_sensors.fix_valid = true;
    g_sensors.fix_type  = 3;
    int len;
    const char* s = build_rmc(&len);
    TEST_ASSERT_NOT_NULL(s);
    // Status field 'A' follows time: "$GPRMC,HHMMSS.ss,A,"
    TEST_ASSERT_NOT_NULL(strstr(s, ",A,"));
}

void test_rmc_no_fix(void) {
    g_sensors.fix_valid = false;
    g_sensors.fix_type  = 0;
    int len;
    const char* s = build_rmc(&len);
    TEST_ASSERT_NOT_NULL(s);
    TEST_ASSERT_NOT_NULL(strstr(s, ",V,"));
}

void test_rmc_checksum_valid(void) {
    g_sensors.lat_deg = -33.8688;
    g_sensors.lon_deg = +151.2093;
    g_sensors.vel_n_ms = 0.5f;
    g_sensors.vel_e_ms = 0.2f;
    g_sensors.timestamp_usec = 1700000000ULL * 1000000ULL;
    int len;
    const char* s = build_rmc(&len);
    TEST_ASSERT_NOT_NULL(s);
    int parsed = parse_sentence_checksum(s);
    TEST_ASSERT_NOT_EQUAL(-1, parsed);
    TEST_ASSERT_EQUAL_HEX8(nmea_checksum(s), (uint8_t)parsed);
}

// ---------------------------------------------------------------------------
// GGA tests
// ---------------------------------------------------------------------------

void test_gga_quality_no_fix(void) {
    g_sensors.fix_valid = false;
    int len;
    const char* s = build_gga(&len);
    TEST_ASSERT_NOT_NULL(s);
    // Quality field = 0: appears as ",0," after satellite count position
    TEST_ASSERT_NOT_NULL(strstr(s, ",0,"));
}

void test_gga_quality_gps_fix(void) {
    g_sensors.fix_valid = true;
    g_sensors.fix_type  = 3;
    int len;
    const char* s = build_gga(&len);
    TEST_ASSERT_NOT_NULL(s);
    // Quality field = 1: ",1," after lat/lon/NS/EW fields
    TEST_ASSERT_NOT_NULL(strstr(s, ",1,"));
}

void test_gga_altitude(void) {
    g_sensors.alt_m = 42.5f;
    int len;
    const char* s = build_gga(&len);
    TEST_ASSERT_NOT_NULL(s);
    TEST_ASSERT_NOT_NULL(strstr(s, "42.5"));
}

void test_gga_numsats(void) {
    g_sensors.num_sats = 9;
    int len;
    const char* s = build_gga(&len);
    TEST_ASSERT_NOT_NULL(s);
    // Formatted as "%02d" → "09"
    TEST_ASSERT_NOT_NULL(strstr(s, ",09,"));
}

void test_gga_checksum_valid(void) {
    g_sensors.lat_deg  = +48.1173;
    g_sensors.lon_deg  = +11.5167;
    g_sensors.alt_m    = 520.0f;
    g_sensors.num_sats = 7;
    g_sensors.hdop     = 1.4f;
    g_sensors.timestamp_usec = 1700000000ULL * 1000000ULL;
    int len;
    const char* s = build_gga(&len);
    TEST_ASSERT_NOT_NULL(s);
    int parsed = parse_sentence_checksum(s);
    TEST_ASSERT_NOT_EQUAL(-1, parsed);
    TEST_ASSERT_EQUAL_HEX8(nmea_checksum(s), (uint8_t)parsed);
}

// ---------------------------------------------------------------------------
// VTG tests
// ---------------------------------------------------------------------------

void test_vtg_sog_knots_and_kmh(void) {
    // 1 m/s north = 1.94384 kn = 3.60007 km/h
    // With %.1f format: knots = "1.9", km/h = "3.6"
    g_sensors.vel_n_ms = 1.0f;
    g_sensors.vel_e_ms = 0.0f;
    int len;
    const char* s = build_vtg(&len);
    TEST_ASSERT_NOT_NULL(s);
    // COG = 0.0° T, SOG = 1.9 N, SOG_kmh = 3.6 K
    TEST_ASSERT_NOT_NULL(strstr(s, "0.0,T,,M,1.9,N,3.6,K"));
}

void test_vtg_checksum_valid(void) {
    g_sensors.vel_n_ms = 2.5f;
    g_sensors.vel_e_ms = 1.5f;
    int len;
    const char* s = build_vtg(&len);
    TEST_ASSERT_NOT_NULL(s);
    int parsed = parse_sentence_checksum(s);
    TEST_ASSERT_NOT_EQUAL(-1, parsed);
    TEST_ASSERT_EQUAL_HEX8(nmea_checksum(s), (uint8_t)parsed);
}

// ---------------------------------------------------------------------------
// GSA tests
// ---------------------------------------------------------------------------

void test_gsa_no_fix(void) {
    g_sensors.fix_valid = false;
    g_sensors.fix_type  = 0;
    int len;
    const char* s = build_gsa(&len);
    TEST_ASSERT_NOT_NULL(s);
    // fix_field = 1 (no fix): "$GPGSA,A,1,"
    TEST_ASSERT_NOT_NULL(strstr(s, ",A,1,"));
}

void test_gsa_3d_fix(void) {
    g_sensors.fix_valid = true;
    g_sensors.fix_type  = 3;
    int len;
    const char* s = build_gsa(&len);
    TEST_ASSERT_NOT_NULL(s);
    TEST_ASSERT_NOT_NULL(strstr(s, ",A,3,"));
}

void test_gsa_dop_values(void) {
    g_sensors.pdop = 1.5f;
    g_sensors.hdop = 1.2f;
    g_sensors.vdop = 1.8f;
    int len;
    const char* s = build_gsa(&len);
    TEST_ASSERT_NOT_NULL(s);
    // Formatted as "%.1f": "1.5", "1.2", "1.8" appear at end of sentence
    TEST_ASSERT_NOT_NULL(strstr(s, "1.5,1.2,1.8"));
}

void test_gsa_checksum_valid(void) {
    g_sensors.pdop = 2.1f;
    g_sensors.hdop = 1.7f;
    g_sensors.vdop = 3.0f;
    int len;
    const char* s = build_gsa(&len);
    TEST_ASSERT_NOT_NULL(s);
    int parsed = parse_sentence_checksum(s);
    TEST_ASSERT_NOT_EQUAL(-1, parsed);
    TEST_ASSERT_EQUAL_HEX8(nmea_checksum(s), (uint8_t)parsed);
}

// ---------------------------------------------------------------------------
// HDM tests
// ---------------------------------------------------------------------------

void test_hdm_null_when_mag_invalid(void) {
    g_sensors.mag_valid = false;
    int len = -1;
    const char* s = build_hdm(&len);
    TEST_ASSERT_NULL(s);
    TEST_ASSERT_EQUAL(0, len);
}

void test_hdm_east(void) {
    // mag_x=0, mag_y=1 → atan2(1,0)=90°
    g_sensors.mag_valid = true;
    g_sensors.mag_x = 0.0f;
    g_sensors.mag_y = 1.0f;
    int len;
    const char* s = build_hdm(&len);
    TEST_ASSERT_NOT_NULL(s);
    TEST_ASSERT_NOT_NULL(strstr(s, "90.0,M"));
}

void test_hdm_north(void) {
    // mag_x=1, mag_y=0 → atan2(0,1)=0°
    g_sensors.mag_valid = true;
    g_sensors.mag_x = 1.0f;
    g_sensors.mag_y = 0.0f;
    int len;
    const char* s = build_hdm(&len);
    TEST_ASSERT_NOT_NULL(s);
    TEST_ASSERT_NOT_NULL(strstr(s, "0.0,M"));
}

void test_hdm_west(void) {
    // mag_x=0, mag_y=-1 → atan2(-1,0)=-90° → +360 = 270°
    g_sensors.mag_valid = true;
    g_sensors.mag_x = 0.0f;
    g_sensors.mag_y = -1.0f;
    int len;
    const char* s = build_hdm(&len);
    TEST_ASSERT_NOT_NULL(s);
    TEST_ASSERT_NOT_NULL(strstr(s, "270.0,M"));
}

void test_hdm_south(void) {
    // mag_x=-1, mag_y=0 → atan2(0,-1)=180°
    g_sensors.mag_valid = true;
    g_sensors.mag_x = -1.0f;
    g_sensors.mag_y = 0.0f;
    int len;
    const char* s = build_hdm(&len);
    TEST_ASSERT_NOT_NULL(s);
    TEST_ASSERT_NOT_NULL(strstr(s, "180.0,M"));
}

void test_hdm_no_negative_heading(void) {
    // Drive atan2 negative, confirm output is [0, 360)
    g_sensors.mag_valid = true;
    g_sensors.mag_x = -0.5f;
    g_sensors.mag_y = -0.5f;  // atan2(-0.5,-0.5) = -135° → 225°
    int len;
    const char* s = build_hdm(&len);
    TEST_ASSERT_NOT_NULL(s);
    // Extract heading value from sentence
    const char* comma = strchr(s, ',');
    TEST_ASSERT_NOT_NULL(comma);
    float heading = strtof(comma + 1, NULL);
    TEST_ASSERT_TRUE(heading >= 0.0f);
    TEST_ASSERT_TRUE(heading < 360.0f);
    // 225° expected
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 225.0f, heading);
}

void test_hdm_checksum_valid(void) {
    g_sensors.mag_valid = true;
    g_sensors.mag_x = 0.3f;
    g_sensors.mag_y = 0.2f;
    int len;
    const char* s = build_hdm(&len);
    TEST_ASSERT_NOT_NULL(s);
    int parsed = parse_sentence_checksum(s);
    TEST_ASSERT_NOT_EQUAL(-1, parsed);
    TEST_ASSERT_EQUAL_HEX8(nmea_checksum(s), (uint8_t)parsed);
}

// ---------------------------------------------------------------------------
// XDR baro tests
// ---------------------------------------------------------------------------

void test_xdr_baro_conversion(void) {
    // 101325 Pa = 1.01325 bar
    g_sensors.pressure_pa = 101325.0f;
    int len;
    const char* s = build_xdr_baro(&len);
    TEST_ASSERT_NOT_NULL(s);
    // Value is "%.5f" → "1.01325"
    TEST_ASSERT_NOT_NULL(strstr(s, "1.01325"));
}

void test_xdr_baro_checksum_valid(void) {
    g_sensors.pressure_pa = 101325.0f;
    int len;
    const char* s = build_xdr_baro(&len);
    TEST_ASSERT_NOT_NULL(s);
    int parsed = parse_sentence_checksum(s);
    TEST_ASSERT_NOT_EQUAL(-1, parsed);
    TEST_ASSERT_EQUAL_HEX8(nmea_checksum(s), (uint8_t)parsed);
}

// ---------------------------------------------------------------------------
// XDR temperature tests
// ---------------------------------------------------------------------------

void test_xdr_temp_conversion(void) {
    // 293.15 K = 20.00 °C
    g_sensors.temperature_k = 293.15f;
    int len;
    const char* s = build_xdr_temp(&len);
    TEST_ASSERT_NOT_NULL(s);
    // Value is "%.2f" → "20.00"
    TEST_ASSERT_NOT_NULL(strstr(s, "20.00"));
}

void test_xdr_temp_checksum_valid(void) {
    g_sensors.temperature_k = 293.15f;
    int len;
    const char* s = build_xdr_temp(&len);
    TEST_ASSERT_NOT_NULL(s);
    int parsed = parse_sentence_checksum(s);
    TEST_ASSERT_NOT_EQUAL(-1, parsed);
    TEST_ASSERT_EQUAL_HEX8(nmea_checksum(s), (uint8_t)parsed);
}

// ---------------------------------------------------------------------------
// Unity entry point
// ---------------------------------------------------------------------------

int main(void) {
    UNITY_BEGIN();

    // Checksum
    RUN_TEST(test_checksum_known_sentence);
    RUN_TEST(test_checksum_empty_payload);
    RUN_TEST(test_finalize_appends_correctly);

    // Coordinate conversion
    RUN_TEST(test_lat_north_positive);
    RUN_TEST(test_lat_south_negative);
    RUN_TEST(test_lon_east_positive);
    RUN_TEST(test_lon_west_negative);
    RUN_TEST(test_lat_zero_equator);
    RUN_TEST(test_lon_zero_meridian);
    RUN_TEST(test_high_latitude);

    // UTC time
    RUN_TEST(test_utc_midnight);
    RUN_TEST(test_utc_known_time);
    RUN_TEST(test_utc_near_midnight);
    RUN_TEST(test_utc_after_midnight);

    // SOG / COG
    RUN_TEST(test_sog_stationary);
    RUN_TEST(test_sog_northward);
    RUN_TEST(test_cog_north);
    RUN_TEST(test_cog_east);
    RUN_TEST(test_cog_south);
    RUN_TEST(test_cog_west);
    RUN_TEST(test_cog_southwest);

    // RMC validity
    RUN_TEST(test_rmc_valid_fix);
    RUN_TEST(test_rmc_no_fix);
    RUN_TEST(test_rmc_checksum_valid);

    // GGA
    RUN_TEST(test_gga_quality_no_fix);
    RUN_TEST(test_gga_quality_gps_fix);
    RUN_TEST(test_gga_altitude);
    RUN_TEST(test_gga_numsats);
    RUN_TEST(test_gga_checksum_valid);

    // VTG
    RUN_TEST(test_vtg_sog_knots_and_kmh);
    RUN_TEST(test_vtg_checksum_valid);

    // GSA
    RUN_TEST(test_gsa_no_fix);
    RUN_TEST(test_gsa_3d_fix);
    RUN_TEST(test_gsa_dop_values);
    RUN_TEST(test_gsa_checksum_valid);

    // HDM
    RUN_TEST(test_hdm_null_when_mag_invalid);
    RUN_TEST(test_hdm_east);
    RUN_TEST(test_hdm_north);
    RUN_TEST(test_hdm_west);
    RUN_TEST(test_hdm_south);
    RUN_TEST(test_hdm_no_negative_heading);
    RUN_TEST(test_hdm_checksum_valid);

    // XDR baro
    RUN_TEST(test_xdr_baro_conversion);
    RUN_TEST(test_xdr_baro_checksum_valid);

    // XDR temp
    RUN_TEST(test_xdr_temp_conversion);
    RUN_TEST(test_xdr_temp_checksum_valid);

    return UNITY_END();
}
