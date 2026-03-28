#include "can_driver.h"
#include "config.h"

#include <driver/twai.h>
#include <esp_log.h>
#include <string.h>

static const char* TAG = "can_driver";

bool g_bus_off = false;

void can_driver_init(void) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    g_config.rx_queue_len = 32;
    g_config.tx_queue_len = 16;

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();

    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TWAI driver install failed: %d", err);
        return;
    }

    err = twai_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TWAI start failed: %d", err);
        return;
    }

    // Register alerts for bus-off and bus-error conditions
    uint32_t alerts = TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_ERROR |
                      TWAI_ALERT_ERR_PASS | TWAI_ALERT_TX_FAILED;
    err = twai_reconfigure_alerts(alerts, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TWAI alert config failed: %d", err);
    }

    ESP_LOGI(TAG, "TWAI initialized at 1 Mbps (TX=GPIO%d, RX=GPIO%d)",
             (int)CAN_TX_PIN, (int)CAN_RX_PIN);
}

bool can_receive(CanardCANFrame* out_frame) {
    twai_message_t msg;
    esp_err_t err = twai_receive(&msg, 0);  // non-blocking
    if (err != ESP_OK) {
        return false;
    }

    // Convert twai_message_t to CanardCANFrame
    // DroneCAN uses 29-bit extended IDs exclusively
    out_frame->id = msg.identifier | CANARD_CAN_FRAME_EFF;

    // Canard does not use RTR frames; discard them
    if (msg.rtr) {
        return false;
    }

    out_frame->data_len = (uint8_t)msg.data_length_code;
    memcpy(out_frame->data, msg.data, msg.data_length_code);

    return true;
}

void can_transmit(const CanardCANFrame* frame) {
    twai_message_t msg;
    memset(&msg, 0, sizeof(msg));

    // Strip canard flags to get the raw 29-bit identifier
    msg.identifier = frame->id & CANARD_CAN_EXT_ID_MASK;
    msg.extd = 1;       // DroneCAN always uses extended (29-bit) IDs
    msg.rtr = 0;
    msg.data_length_code = frame->data_len;
    memcpy(msg.data, frame->data, frame->data_len);

    // Non-blocking: drop frame silently if TX queue is full
    twai_transmit(&msg, 0);
}

bool can_check_alerts(void) {
    uint32_t alerts = 0;
    esp_err_t err = twai_read_alerts(&alerts, 0);  // non-blocking
    if (err != ESP_OK || alerts == 0) {
        return false;
    }

    bool error_fired = false;

    if (alerts & TWAI_ALERT_BUS_OFF) {
        ESP_LOGE(TAG, "TWAI bus-off detected, initiating recovery");
        g_bus_off = true;
        error_fired = true;
        twai_initiate_recovery();
    }

    if (alerts & TWAI_ALERT_BUS_ERROR) {
        ESP_LOGW(TAG, "TWAI bus error alert");
        error_fired = true;
    }

    if (alerts & TWAI_ALERT_ERR_PASS) {
        ESP_LOGW(TAG, "TWAI error passive");
        error_fired = true;
    }

    if (alerts & TWAI_ALERT_TX_FAILED) {
        // TX failure is non-fatal; just note it
        error_fired = true;
    }

    // If recovery completed, clear bus-off flag
    twai_status_info_t status;
    if (twai_get_status_info(&status) == ESP_OK) {
        if (status.state == TWAI_STATE_RUNNING && g_bus_off) {
            g_bus_off = false;
            ESP_LOGI(TAG, "TWAI bus recovered");
        }
    }

    return error_fired;
}
