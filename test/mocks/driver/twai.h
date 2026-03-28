#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t identifier;
    uint8_t  data[8];
    uint8_t  data_length_code;
    bool     extd;
    bool     rtr;
} twai_message_t;

typedef int twai_timing_config_t;
typedef int twai_filter_config_t;

typedef struct {
    int mode;
    int tx_io;
    int rx_io;
} twai_general_config_t;

#define TWAI_MODE_NORMAL      0
#define TWAI_ALERT_BUS_OFF    0x200
#define TWAI_ALERT_BUS_ERROR  0x004

#define TWAI_TIMING_CONFIG_1MBITS()       0
#define TWAI_FILTER_CONFIG_ACCEPT_ALL()   0
#define TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, mode) {(mode), (tx), (rx)}

static inline int twai_driver_install(const twai_general_config_t*, const twai_timing_config_t*, const twai_filter_config_t*) { return 0; }
static inline int twai_start()                                           { return 0; }
static inline int twai_stop()                                            { return 0; }
static inline int twai_driver_uninstall()                                { return 0; }
static inline int twai_receive(twai_message_t*, int)                     { return -1; }
static inline int twai_transmit(const twai_message_t*, int)              { return 0; }
static inline int twai_read_alerts(uint32_t* alerts, int)                { *alerts = 0; return 0; }
static inline int twai_initiate_recovery()                               { return 0; }
static inline int twai_alert_enabled(int, int)                           { return 0; }
