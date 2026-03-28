#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Start WiFi SoftAP and TCP server.
// Initializes up to 4 simultaneous TCP client slots.
// If WIFI_UDP_BROADCAST is enabled, also initializes the UDP socket.
void wifi_output_init(void);

// Broadcast a sentence to all connected TCP clients and (if enabled) via UDP.
// Drops disconnected clients from their slots.
void wifi_output_broadcast(const char* sentence, int len);

// Accept any pending new TCP client connections into available slots.
// Call this periodically from the output task.
void wifi_output_tick(void);

#ifdef __cplusplus
}
#endif
