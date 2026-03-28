#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize UART2 for RS485 output at SERIAL_BAUD.
// Configures DE/RE pin as output, defaulting to receive mode (LOW).
void serial_output_init(void);

// Transmit a sentence over RS485.
// Asserts DE HIGH, waits 100 µs, writes data, flushes, waits 100 µs,
// then de-asserts DE LOW.
void serial_send_sentence(const char* sentence, int len);

#ifdef __cplusplus
}
#endif
