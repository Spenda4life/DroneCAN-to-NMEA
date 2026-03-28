#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "canard.h"

#ifdef __cplusplus
extern "C" {
#endif

// Set to true when TWAI bus-off is detected; cleared on recovery attempt
extern bool g_bus_off;

// Initialize TWAI peripheral at 1 Mbps on configured GPIO pins.
// Registers bus-off and bus-error alerts. Starts the driver.
void can_driver_init(void);

// Non-blocking receive. Returns true and fills out_frame if a frame is
// available in the TWAI RX queue; returns false if queue is empty.
bool can_receive(CanardCANFrame* out_frame);

// Non-blocking transmit. Converts CanardCANFrame to twai_message_t and
// calls twai_transmit with 0 timeout. Drops silently if queue is full.
void can_transmit(const CanardCANFrame* frame);

// Read TWAI alerts (non-blocking). If bus-off detected, initiates recovery.
// Returns true if any error alert fired.
bool can_check_alerts(void);

#ifdef __cplusplus
}
#endif
