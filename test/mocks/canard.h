#pragma once
/*
 * Mock canard.h for native (host) unit tests.
 * Provides enough type definitions so that dronecan_handler.h compiles.
 * The actual canard functions are NOT implemented here — they are not
 * called during native NMEA tests because dronecan_handler.cpp is excluded
 * from the native build (build_src_filter only includes nmea_generator.cpp).
 */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
#define CANARD_CAN_FRAME_EFF           0x80000000U   // Extended frame format flag
#define CANARD_CAN_EXT_ID_MASK         0x1FFFFFFFU
#define CANARD_TRANSFER_PRIORITY_LOW   30

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

typedef struct {
    uint32_t id;
    uint8_t  data[8];
    uint8_t  data_len;
} CanardCANFrame;

typedef struct CanardInstance {
    void* _placeholder;
} CanardInstance;

typedef enum {
    CanardTransferTypeBroadcast  = 0,
    CanardTransferTypeRequest    = 1,
    CanardTransferTypeResponse   = 2
} CanardTransferType;

typedef struct CanardRxTransfer {
    const struct CanardRxTransfer* next;
    const uint8_t*                 payload_head;
    uint16_t                       payload_len;
    uint16_t                       payload_middle_len;
    uint16_t                       data_type_id;
    uint8_t                        transfer_id;
    uint8_t                        payload_tail_len;
    uint8_t                        source_node_id;
    CanardTransferType             transfer_type;
} CanardRxTransfer;

typedef bool (*CanardShouldAcceptTransfer)(const CanardInstance*,
                                           uint64_t*,
                                           uint16_t,
                                           CanardTransferType,
                                           uint8_t);

typedef void (*CanardOnTransferReception)(CanardInstance*, CanardRxTransfer*);

// ---------------------------------------------------------------------------
// Function declarations — defined as no-ops / stubs in canard_mock.cpp
// (only included when UNIT_TEST is defined and dronecan_handler.cpp is built)
// ---------------------------------------------------------------------------
#ifdef __cplusplus
extern "C" {
#endif

void canardInit(CanardInstance* out_ins,
                void*           mem_arena,
                size_t          mem_arena_size,
                CanardOnTransferReception    on_reception,
                CanardShouldAcceptTransfer   should_accept,
                void*           user_reference);

void     canardSetLocalNodeID(CanardInstance* ins, uint8_t self_node_id);

int      canardHandleRxFrame(CanardInstance* ins,
                              const CanardCANFrame* frame,
                              uint64_t timestamp_usec);

/**
 * Decode a scalar field from a transfer payload.
 *
 * bit_offset   — bit position from start of payload
 * bit_length   — number of bits to read (1–64)
 * is_signed    — if true, sign-extend the result
 * out_value    — pointer to int64_t or uint64_t destination
 *
 * Reads little-endian bit order (LSB first within each byte), matching
 * the real libcanard implementation.
 */
void canardDecodeScalar(const CanardRxTransfer* transfer,
                        uint32_t                bit_offset,
                        uint8_t                 bit_length,
                        bool                    is_signed,
                        void*                   out_value);

int      canardBroadcast(CanardInstance*  ins,
                          uint64_t         data_type_signature,
                          uint16_t         data_type_id,
                          uint8_t*         inout_transfer_id,
                          uint8_t          priority,
                          const void*      payload,
                          uint16_t         payload_len);

const CanardCANFrame* canardPeekTxQueue(const CanardInstance* ins);

void     canardPopTxQueue(CanardInstance* ins);

#ifdef __cplusplus
}
#endif
