#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "canard.h"

// ---------------------------------------------------------------------------
// SensorData — shared state between canTask (writer) and outputTask (reader)
// ---------------------------------------------------------------------------
struct SensorData {
    // GPS Fix2
    bool     fix_valid;
    uint8_t  fix_type;        // 0=no fix, 2=2D, 3=3D
    uint8_t  num_sats;
    double   lat_deg;         // decimal degrees, positive = North
    double   lon_deg;         // decimal degrees, positive = East
    float    alt_m;           // altitude MSL
    float    vel_n_ms;        // NED velocity north (m/s)
    float    vel_e_ms;        // NED velocity east (m/s)
    float    vel_d_ms;        // NED velocity down (m/s)
    uint64_t timestamp_usec;  // microseconds since Unix epoch (UTC)

    // Auxiliary (DOP)
    float    pdop;
    float    hdop;
    float    vdop;

    // Magnetometer
    bool     mag_valid;
    float    mag_x;           // Gauss
    float    mag_y;
    float    mag_z;

    // Baro / Temp
    float    pressure_pa;
    float    temperature_k;

    // IMU
    float    accel_x, accel_y, accel_z;   // m/s^2
    float    gyro_x,  gyro_y,  gyro_z;    // rad/s

    // Staleness tracking
    uint32_t last_fix_ms;     // millis() at last Fix2 receive
    uint32_t last_mag_ms;
};

extern SensorData g_sensors;
extern portMUX_TYPE g_sensors_mux;

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

// Initialize the canard instance (node ID, memory pool, callbacks).
void dronecan_init(void);

// Feed a received CAN frame into the canard state machine.
void dronecan_handle_frame(const CanardCANFrame* frame, uint64_t timestamp_usec);

// Broadcast a uavcan.protocol.NodeStatus frame (call at 1 Hz).
void dronecan_send_node_status(void);

#ifdef UNIT_TEST
/**
 * Test injection hook — bypasses CAN framing and feeds a raw payload
 * directly into the DSDL decoder dispatch (on_reception).
 * Only compiled when UNIT_TEST is defined.
 */
void dronecan_test_inject(uint16_t data_type_id,
                          const uint8_t* payload,
                          uint16_t payload_len);
#endif

#ifdef __cplusplus
}
#endif
