#include "dronecan_handler.h"
#include "can_driver.h"
#include "config.h"

#include <string.h>
#include <math.h>
#include <Arduino.h>
#include <esp_log.h>

static const char* TAG = "dronecan";

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
SensorData g_sensors;
portMUX_TYPE g_sensors_mux = portMUX_INITIALIZER_UNLOCKED;

CanardInstance g_canard;
static uint8_t canard_memory_pool[4096];

// Transfer ID counters for TX transfers
static uint8_t node_status_transfer_id = 0;
static uint8_t alloc_transfer_id = 0;

// ---------------------------------------------------------------------------
// Dynamic Node ID Allocation state
// ---------------------------------------------------------------------------
static const uint16_t ALLOCATION_DTID = 1;
static const uint64_t ALLOCATION_SIGNATURE = 0x0B2A812620A11D40ULL;

static uint8_t  alloc_unique_id[16];   // Accumulated unique ID from allocatee
static uint8_t  alloc_unique_id_len;   // How many bytes accumulated so far
static uint32_t alloc_last_activity_ms; // Timeout tracking
static const uint32_t ALLOC_TIMEOUT_MS = 2000;

// ---------------------------------------------------------------------------
// Float16 conversion helper
// ---------------------------------------------------------------------------
static float f16_to_float(uint16_t h) {
    uint32_t sign     = (h >> 15) & 0x1U;
    uint32_t exponent = (h >> 10) & 0x1FU;
    uint32_t mantissa = h & 0x3FFU;
    uint32_t f;

    if (exponent == 0) {
        if (mantissa == 0) {
            f = sign << 31;
        } else {
            exponent = 1;
            while (!(mantissa & 0x400U)) {
                mantissa <<= 1;
                exponent--;
            }
            f = (sign << 31) | ((exponent + 112U) << 23) | ((mantissa & 0x3FFU) << 13);
        }
    } else if (exponent == 31) {
        f = (sign << 31) | (0xFFU << 23) | (mantissa << 13);
    } else {
        f = (sign << 31) | ((exponent + 112U) << 23) | (mantissa << 13);
    }

    float result;
    memcpy(&result, &f, sizeof(f));
    return result;
}

// ---------------------------------------------------------------------------
// DSDL decoders
// ---------------------------------------------------------------------------

// DTID 1063 — uavcan.equipment.gnss.Fix2
//
// Bit layout (from DSDL, uavcan.Timestamp = uint56):
//   timestamp.usec:        bit 0,   56 bits
//   gnss_timestamp.usec:   bit 56,  56 bits
//   gnss_time_standard:    bit 112, 3 bits
//   void13:                bit 115, 13 bits (reserved padding)
//   num_leap_seconds:      bit 128, 8 bits
//   longitude_deg_1e8:     bit 136, 37 bits (signed)
//   latitude_deg_1e8:      bit 173, 37 bits (signed)
//   height_ellipsoid_mm:   bit 210, 27 bits (signed)
//   height_msl_mm:         bit 237, 27 bits (signed)
//   ned_velocity[3]:       bit 264, 3×32 bits (float32!)
//   sats_used:             bit 360, 6 bits
//   status:                bit 366, 2 bits
//
static void decode_fix2(const CanardRxTransfer* transfer) {
    int64_t raw_s64 = 0;
    uint64_t raw_u64 = 0;

    // gnss_timestamp (bit 56, len 56)
    canardDecodeScalar(transfer, 56, 56, false, &raw_u64);
    uint64_t gnss_timestamp_usec = (uint64_t)raw_u64;

    // gnss_time_standard (bit 112, len 3)
    raw_u64 = 0;
    canardDecodeScalar(transfer, 112, 3, false, &raw_u64);
    uint8_t gnss_time_standard = (uint8_t)raw_u64;

    // num_leap_seconds (bit 128, len 8)
    raw_u64 = 0;
    canardDecodeScalar(transfer, 128, 8, false, &raw_u64);
    uint8_t num_leap_seconds = (uint8_t)raw_u64;

    // longitude_deg_1e8 (bit 136, len 37, signed)
    raw_s64 = 0;
    canardDecodeScalar(transfer, 136, 37, true, &raw_s64);
    double lon_deg = (double)raw_s64 / 1e8;

    // latitude_deg_1e8 (bit 173, len 37, signed)
    raw_s64 = 0;
    canardDecodeScalar(transfer, 173, 37, true, &raw_s64);
    double lat_deg = (double)raw_s64 / 1e8;

    // height_msl_mm (bit 237, len 27, signed)
    // Use int32_t — canardDecodeScalar sign-extends to 32 bits, not 64
    int32_t height_msl_mm = 0;
    canardDecodeScalar(transfer, 237, 27, true, &height_msl_mm);
    float alt_m = (float)height_msl_mm / 1000.0f;

    // ned_velocity[0] north (bit 264, len 32, float32)
    uint32_t raw_u32 = 0;
    canardDecodeScalar(transfer, 264, 32, false, &raw_u32);
    float vel_n;
    memcpy(&vel_n, &raw_u32, sizeof(vel_n));

    // ned_velocity[1] east (bit 296, len 32, float32)
    raw_u32 = 0;
    canardDecodeScalar(transfer, 296, 32, false, &raw_u32);
    float vel_e;
    memcpy(&vel_e, &raw_u32, sizeof(vel_e));

    // ned_velocity[2] down (bit 328, len 32, float32)
    raw_u32 = 0;
    canardDecodeScalar(transfer, 328, 32, false, &raw_u32);
    float vel_d;
    memcpy(&vel_d, &raw_u32, sizeof(vel_d));

    // sats_used (bit 360, len 6)
    raw_u64 = 0;
    canardDecodeScalar(transfer, 360, 6, false, &raw_u64);
    uint8_t num_sats = (uint8_t)raw_u64;

    // status (bit 366, len 2)
    raw_u64 = 0;
    canardDecodeScalar(transfer, 366, 2, false, &raw_u64);
    uint8_t fix_status = (uint8_t)raw_u64;

    // Convert gnss_timestamp to UTC microseconds since Unix epoch
    uint64_t utc_usec = 0;
    if (gnss_timestamp_usec > 0) {
        switch (gnss_time_standard) {
            case 2: // UTC — use directly
                utc_usec = gnss_timestamp_usec;
                break;
            case 3: // GPS — GPS time is ahead of UTC by leap seconds
                utc_usec = gnss_timestamp_usec - ((uint64_t)num_leap_seconds * 1000000ULL);
                break;
            case 1: // TAI — subtract 37 seconds (current TAI-UTC offset)
                utc_usec = (gnss_timestamp_usec > 37000000ULL)
                           ? (gnss_timestamp_usec - 37000000ULL)
                           : 0;
                break;
            case 0: // undefined — assume UTC
            default:
                utc_usec = gnss_timestamp_usec;
                break;
        }
    }

    portENTER_CRITICAL(&g_sensors_mux);
    g_sensors.lat_deg        = lat_deg;
    g_sensors.lon_deg        = lon_deg;
    g_sensors.alt_m          = alt_m;
    g_sensors.vel_n_ms       = vel_n;
    g_sensors.vel_e_ms       = vel_e;
    g_sensors.vel_d_ms       = vel_d;
    g_sensors.num_sats       = num_sats;
    g_sensors.fix_type       = fix_status;
    g_sensors.fix_valid      = (fix_status >= 2);
    g_sensors.timestamp_usec = utc_usec;
    g_sensors.last_fix_ms    = millis();
    portEXIT_CRITICAL(&g_sensors_mux);
}

// DTID 1061 — uavcan.equipment.gnss.Auxiliary
static void decode_auxiliary(const CanardRxTransfer* transfer) {
    uint64_t raw_u64 = 0;

    canardDecodeScalar(transfer, 0, 16, false, &raw_u64);
    float pdop = f16_to_float((uint16_t)raw_u64);

    raw_u64 = 0;
    canardDecodeScalar(transfer, 16, 16, false, &raw_u64);
    float hdop = f16_to_float((uint16_t)raw_u64);

    raw_u64 = 0;
    canardDecodeScalar(transfer, 32, 16, false, &raw_u64);
    float vdop = f16_to_float((uint16_t)raw_u64);

    portENTER_CRITICAL(&g_sensors_mux);
    g_sensors.pdop = pdop;
    g_sensors.hdop = hdop;
    g_sensors.vdop = vdop;
    portEXIT_CRITICAL(&g_sensors_mux);
}

// DTID 1002 — uavcan.equipment.ahrs.MagneticFieldStrength2
static void decode_mag(const CanardRxTransfer* transfer) {
    uint64_t raw_u64 = 0;

    // sensor_id (bit 0, len 8) — skip
    // mag_x (bit 8, len 16)
    raw_u64 = 0;
    canardDecodeScalar(transfer, 8, 16, false, &raw_u64);
    float mag_x = f16_to_float((uint16_t)raw_u64);

    // mag_y (bit 24, len 16)
    raw_u64 = 0;
    canardDecodeScalar(transfer, 24, 16, false, &raw_u64);
    float mag_y = f16_to_float((uint16_t)raw_u64);

    // mag_z (bit 40, len 16)
    raw_u64 = 0;
    canardDecodeScalar(transfer, 40, 16, false, &raw_u64);
    float mag_z = f16_to_float((uint16_t)raw_u64);

    portENTER_CRITICAL(&g_sensors_mux);
    g_sensors.mag_x      = mag_x;
    g_sensors.mag_y      = mag_y;
    g_sensors.mag_z      = mag_z;
    g_sensors.mag_valid  = true;
    g_sensors.last_mag_ms = millis();
    portEXIT_CRITICAL(&g_sensors_mux);
}

// DTID 1028 — uavcan.equipment.air_data.StaticPressure
static void decode_static_pressure(const CanardRxTransfer* transfer) {
    uint64_t raw_u64 = 0;
    canardDecodeScalar(transfer, 0, 32, false, &raw_u64);
    uint32_t raw32 = (uint32_t)raw_u64;
    float pressure_pa;
    memcpy(&pressure_pa, &raw32, sizeof(pressure_pa));

    portENTER_CRITICAL(&g_sensors_mux);
    g_sensors.pressure_pa = pressure_pa;
    portEXIT_CRITICAL(&g_sensors_mux);
}

// DTID 1029 — uavcan.equipment.air_data.StaticTemperature
static void decode_static_temperature(const CanardRxTransfer* transfer) {
    uint64_t raw_u64 = 0;
    canardDecodeScalar(transfer, 0, 16, false, &raw_u64);
    float temperature_k = f16_to_float((uint16_t)raw_u64);

    portENTER_CRITICAL(&g_sensors_mux);
    g_sensors.temperature_k = temperature_k;
    portEXIT_CRITICAL(&g_sensors_mux);
}

// DTID 1003 — uavcan.equipment.imu.RawIMU
static void decode_raw_imu(const CanardRxTransfer* transfer) {
    uint64_t raw_u64 = 0;

    // accelerometer_latest[0] (bit 176, len 16)
    raw_u64 = 0;
    canardDecodeScalar(transfer, 176, 16, false, &raw_u64);
    float accel_x = f16_to_float((uint16_t)raw_u64);

    // accelerometer_latest[1] (bit 192, len 16)
    raw_u64 = 0;
    canardDecodeScalar(transfer, 192, 16, false, &raw_u64);
    float accel_y = f16_to_float((uint16_t)raw_u64);

    // accelerometer_latest[2] (bit 208, len 16)
    raw_u64 = 0;
    canardDecodeScalar(transfer, 208, 16, false, &raw_u64);
    float accel_z = f16_to_float((uint16_t)raw_u64);

    // rate_gyro_latest[0] (bit 224, len 16)
    raw_u64 = 0;
    canardDecodeScalar(transfer, 224, 16, false, &raw_u64);
    float gyro_x = f16_to_float((uint16_t)raw_u64);

    // rate_gyro_latest[1] (bit 240, len 16)
    raw_u64 = 0;
    canardDecodeScalar(transfer, 240, 16, false, &raw_u64);
    float gyro_y = f16_to_float((uint16_t)raw_u64);

    // rate_gyro_latest[2] (bit 256, len 16)
    raw_u64 = 0;
    canardDecodeScalar(transfer, 256, 16, false, &raw_u64);
    float gyro_z = f16_to_float((uint16_t)raw_u64);

    portENTER_CRITICAL(&g_sensors_mux);
    g_sensors.accel_x = accel_x;
    g_sensors.accel_y = accel_y;
    g_sensors.accel_z = accel_z;
    g_sensors.gyro_x  = gyro_x;
    g_sensors.gyro_y  = gyro_y;
    g_sensors.gyro_z  = gyro_z;
    portEXIT_CRITICAL(&g_sensors_mux);
}

// ---------------------------------------------------------------------------
// Bit-level encode helper
// ---------------------------------------------------------------------------
static void encode_bits(uint8_t* buf, uint32_t bit_offset, uint8_t bit_len, uint64_t value) {
    for (uint8_t i = 0; i < bit_len; i++) {
        uint32_t abs_bit = bit_offset + i;
        uint32_t byte_idx = abs_bit / 8u;
        uint8_t  bit_idx  = (uint8_t)(abs_bit % 8u);
        if (value & (1ULL << i)) {
            buf[byte_idx] |= (uint8_t)(1u << bit_idx);
        } else {
            buf[byte_idx] &= (uint8_t)~(1u << bit_idx);
        }
    }
}

// ---------------------------------------------------------------------------
// TX queue drain helper
// ---------------------------------------------------------------------------
static void dronecan_tx_flush(void) {
    const CanardCANFrame* txf;
    while ((txf = canardPeekTxQueue(&g_canard)) != NULL) {
        can_transmit(txf);
        canardPopTxQueue(&g_canard);
    }
}

// ---------------------------------------------------------------------------
// Dynamic Node ID Allocator
//
// Wire format (tail-optimized, no length prefix for unique_id array):
//   Byte 0: node_id (bits 0-6) | first_part_of_unique_id (bit 7)
//   Bytes 1..N: raw unique_id bytes (length = payload_len - 1)
// ---------------------------------------------------------------------------
static void handle_allocation(const CanardRxTransfer* transfer) {
    // Only process anonymous transfers (from nodes without an ID)
    if (transfer->source_node_id != CANARD_BROADCAST_NODE_ID) return;

    if (transfer->payload_len < 1) return;

    // Decode byte 0: node_id (7 bits) | first_part (1 bit)
    uint64_t raw = 0;
    canardDecodeScalar(transfer, 0, 7, false, &raw);
    uint8_t preferred_node_id = (uint8_t)raw;
    (void)preferred_node_id;  // we assign our own ID

    raw = 0;
    canardDecodeScalar(transfer, 7, 1, false, &raw);
    bool first_part = (bool)raw;

    // Unique ID bytes follow directly after byte 0 (tail optimization)
    uint8_t uid_len = (uint8_t)(transfer->payload_len - 1);
    if (uid_len > 16) uid_len = 16;

    uint8_t uid_bytes[16];
    for (uint8_t i = 0; i < uid_len; i++) {
        raw = 0;
        canardDecodeScalar(transfer, 8 + i * 8, 8, false, &raw);
        uid_bytes[i] = (uint8_t)raw;
    }

    // Reset on first_part or timeout
    if (first_part) {
        alloc_unique_id_len = 0;
    }
    if (alloc_unique_id_len > 0 &&
        (millis() - alloc_last_activity_ms) > ALLOC_TIMEOUT_MS) {
        alloc_unique_id_len = 0;
    }
    alloc_last_activity_ms = millis();

    // Append received bytes
    for (uint8_t i = 0; i < uid_len && alloc_unique_id_len < 16; i++) {
        alloc_unique_id[alloc_unique_id_len++] = uid_bytes[i];
    }

    ESP_LOGI(TAG, "Alloc RX: first=%d rcvd=%d accum=%d", first_part, uid_len, alloc_unique_id_len);

    // Build response (same tail-optimized format):
    //   Byte 0: node_id | first_part=0
    //   Bytes 1..N: all accumulated unique_id bytes
    uint8_t resp[17];  // 1 header + up to 16 uid bytes
    memset(resp, 0, sizeof(resp));

    uint8_t resp_node_id = (alloc_unique_id_len >= 16) ? DRONECAN_ALLOC_NODE_ID : 0;
    resp[0] = (uint8_t)(resp_node_id & 0x7F);  // first_part = 0 (bit 7 = 0)
    memcpy(&resp[1], alloc_unique_id, alloc_unique_id_len);

    uint16_t resp_len = (uint16_t)(1 + alloc_unique_id_len);

    int16_t bc_res = canardBroadcast(&g_canard,
                    ALLOCATION_SIGNATURE,
                    ALLOCATION_DTID,
                    &alloc_transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    resp,
                    resp_len);
    if (bc_res <= 0) {
        ESP_LOGE(TAG, "Alloc broadcast failed: %d", bc_res);
    }
    dronecan_tx_flush();

    if (alloc_unique_id_len >= 16) {
        ESP_LOGI(TAG, "Allocated node ID %d", DRONECAN_ALLOC_NODE_ID);
        alloc_unique_id_len = 0;
    }
}

// ---------------------------------------------------------------------------
// Canard callbacks
// ---------------------------------------------------------------------------

static bool should_accept_transfer(const CanardInstance* ins,
                                   uint64_t* out_data_type_signature,
                                   uint16_t data_type_id,
                                   CanardTransferType transfer_type,
                                   uint8_t source_node_id) {
    (void)ins;
    (void)transfer_type;
    (void)source_node_id;

    switch (data_type_id) {
        case 1:    // dynamic_node_id.Allocation
            *out_data_type_signature = ALLOCATION_SIGNATURE;
            return true;
        case 1063: // Fix2
            *out_data_type_signature = 0xCA41E7000F37435FULL;
            return true;
        case 1061: // Auxiliary
            *out_data_type_signature = 0x9BE8BDC4C3DBBFD2ULL;
            return true;
        case 1002: // MagneticFieldStrength2
            *out_data_type_signature = 0xB6AC0C442430297EULL;
            return true;
        case 1028: // StaticPressure
            *out_data_type_signature = 0xCDC7C43412BDC89AULL;
            return true;
        case 1029: // StaticTemperature
            *out_data_type_signature = 0x49272A6477D96271ULL;
            return true;
        case 1003: // RawIMU
            *out_data_type_signature = 0x8280632C40E574B5ULL;
            return true;
        default:
            return false;
    }
}

static void on_reception(CanardInstance* ins,
                         CanardRxTransfer* transfer) {
    (void)ins;

    switch (transfer->data_type_id) {
        case 1:    handle_allocation(transfer);        break;
        case 1063: decode_fix2(transfer);              break;
        case 1061: decode_auxiliary(transfer);         break;
        case 1002: decode_mag(transfer);               break;
        case 1028: decode_static_pressure(transfer);   break;
        case 1029: decode_static_temperature(transfer); break;
        case 1003: decode_raw_imu(transfer);           break;
        default:   break;
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void dronecan_init(void) {
    memset(&g_sensors, 0, sizeof(g_sensors));
    alloc_unique_id_len = 0;
    alloc_last_activity_ms = 0;

    canardInit(&g_canard,
               canard_memory_pool,
               sizeof(canard_memory_pool),
               on_reception,
               should_accept_transfer,
               NULL);

    canardSetLocalNodeID(&g_canard, DRONECAN_NODE_ID);

    ESP_LOGI(TAG, "DroneCAN initialized (node ID=%d)", DRONECAN_NODE_ID);
}

void dronecan_handle_frame(const CanardCANFrame* frame, uint64_t timestamp_usec) {
    canardHandleRxFrame(&g_canard, frame, timestamp_usec);
}

void dronecan_send_node_status(void) {
    // uavcan.protocol.NodeStatus payload (7 bytes):
    //   uint32 uptime_sec         (4 bytes, little-endian)
    //   uint2  health = 0
    //   uint3  mode   = 0
    //   uint3  sub_mode = 0       — packed into 1 byte
    //   uint16 vendor_specific_status_code = 0 (2 bytes, little-endian)
    uint8_t payload[7];
    memset(payload, 0, sizeof(payload));

    uint32_t uptime_sec = millis() / 1000;
    payload[0] = (uint8_t)(uptime_sec & 0xFF);
    payload[1] = (uint8_t)((uptime_sec >> 8)  & 0xFF);
    payload[2] = (uint8_t)((uptime_sec >> 16) & 0xFF);
    payload[3] = (uint8_t)((uptime_sec >> 24) & 0xFF);
    // Byte 4: health(2) | mode(3) | sub_mode(3) — all zero = OK/OPERATIONAL
    payload[4] = 0x00;
    // Bytes 5-6: vendor_specific_status_code = 0
    payload[5] = 0x00;
    payload[6] = 0x00;

    // Data type signature for uavcan.protocol.NodeStatus
    static const uint64_t NODE_STATUS_DATA_TYPE_SIGNATURE = 0x0F0868D0073F4C60ULL;
    static const uint16_t NODE_STATUS_DATA_TYPE_ID        = 341;

    canardBroadcast(&g_canard,
                    NODE_STATUS_DATA_TYPE_SIGNATURE,
                    NODE_STATUS_DATA_TYPE_ID,
                    &node_status_transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    payload,
                    sizeof(payload));

    dronecan_tx_flush();
}

#ifdef UNIT_TEST
// ---------------------------------------------------------------------------
// Test injection — available only when UNIT_TEST is defined.
// Constructs a minimal CanardRxTransfer whose payload_head points at the
// supplied buffer and routes it through the same on_reception dispatcher
// used by the real canard callback.  This lets embedded unit tests verify
// DSDL decode logic without needing a live CAN bus.
// ---------------------------------------------------------------------------
void dronecan_test_inject(uint16_t data_type_id,
                          const uint8_t* payload,
                          uint16_t payload_len) {
    CanardRxTransfer transfer;
    memset(&transfer, 0, sizeof(transfer));
    transfer.data_type_id   = data_type_id;
    transfer.payload_head   = payload;
    transfer.payload_len    = payload_len;
    on_reception(&g_canard, &transfer);
}
#endif // UNIT_TEST
