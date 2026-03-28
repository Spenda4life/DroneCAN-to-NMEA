#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// XOR checksum of all bytes between '$' and '*' (exclusive of both).
uint8_t nmea_checksum(const char* sentence);

// Appends "*XX\r\n" checksum footer to buf at payload_len position.
// Returns total length of the sentence including the footer.
int nmea_finalize(char* buf, int payload_len);

// Build NMEA sentences from current g_sensors state.
// Each function writes into an internal static buffer and returns a pointer
// to it. *out_len receives the total byte count (including \r\n).
// Returns NULL if the sentence should not be emitted (e.g. mag_valid false
// for HDM, or feature flag disabled).

const char* build_rmc(int* out_len);
const char* build_gga(int* out_len);
const char* build_vtg(int* out_len);
const char* build_gsa(int* out_len);
const char* build_hdm(int* out_len);
const char* build_xdr_baro(int* out_len);
const char* build_xdr_temp(int* out_len);

#ifdef __cplusplus
}
#endif
