/**
 * test/test_heartbeat/test_heartbeat.cpp
 *
 * Embedded unit tests for the NodeStatus heartbeat broadcast.
 * Run with: pio test -e test_embedded
 *
 * Verifies that dronecan_send_node_status() produces correctly formed
 * 7-byte NodeStatus payloads with incrementing transfer IDs and uptime.
 *
 * Note: These tests inspect the canard TX queue before frames are popped.
 * Because dronecan_send_node_status() drains the queue after broadcast,
 * we instead test the payload by calling the canard broadcast directly
 * through the public API and verifying g_sensors state consistency.
 *
 * Strategy: call dronecan_send_node_status() which broadcasts and drains
 * the TX queue via can_transmit(). We use TWAI in NO_ACK (loopback) mode
 * to receive the transmitted frames and verify their contents.
 */

#include <Arduino.h>
#include <unity.h>
#include <string.h>
#include <driver/twai.h>

#include "dronecan_handler.h"
#include "can_driver.h"
#include "config.h"

extern "C" {
#include "canard.h"
}

// Access the canard instance to inspect state
extern CanardInstance g_canard;

// ---------------------------------------------------------------------------
// Helper: init TWAI in NO_ACK mode for loopback
// ---------------------------------------------------------------------------

static void twai_init_loopback(void) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NO_ACK);
    g_config.rx_queue_len = 32;
    g_config.tx_queue_len = 16;

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
}

static void twai_deinit(void) {
    twai_stop();
    twai_driver_uninstall();
}

// ---------------------------------------------------------------------------
// setUp / tearDown
// ---------------------------------------------------------------------------

void setUp(void) {
    // Initialize the DroneCAN handler (resets canard state)
    dronecan_init();
}

void tearDown(void) {}

// ---------------------------------------------------------------------------
// Test 1: NodeStatus payload is 7 bytes with health=0, mode=0
// ---------------------------------------------------------------------------

void test_node_status_payload_format(void) {
    twai_init_loopback();

    dronecan_send_node_status();
    vTaskDelay(pdMS_TO_TICKS(10));

    // Receive the transmitted frame via loopback
    // NodeStatus is 7 bytes, fits in a single CAN frame (8 bytes with tail byte)
    twai_message_t rx_msg;
    esp_err_t err = twai_receive(&rx_msg, pdMS_TO_TICKS(100));
    TEST_ASSERT_EQUAL(ESP_OK, err);

    // DroneCAN single-frame transfer: payload + 1 tail byte
    // NodeStatus = 7 payload bytes + 1 tail byte = 8 bytes total
    TEST_ASSERT_EQUAL(8, rx_msg.data_length_code);

    // Byte 4: health(2) | mode(3) | sub_mode(3) — all should be 0
    TEST_ASSERT_EQUAL_HEX8(0x00, rx_msg.data[4]);

    // Bytes 5-6: vendor_specific_status_code = 0
    TEST_ASSERT_EQUAL_HEX8(0x00, rx_msg.data[5]);
    TEST_ASSERT_EQUAL_HEX8(0x00, rx_msg.data[6]);

    twai_deinit();
}

// ---------------------------------------------------------------------------
// Test 2: Uptime field increments between calls
// ---------------------------------------------------------------------------

void test_node_status_uptime_increments(void) {
    twai_init_loopback();

    // First call
    dronecan_send_node_status();
    vTaskDelay(pdMS_TO_TICKS(10));

    twai_message_t rx1;
    esp_err_t err = twai_receive(&rx1, pdMS_TO_TICKS(100));
    TEST_ASSERT_EQUAL(ESP_OK, err);

    uint32_t uptime1 = (uint32_t)rx1.data[0] |
                       ((uint32_t)rx1.data[1] << 8) |
                       ((uint32_t)rx1.data[2] << 16) |
                       ((uint32_t)rx1.data[3] << 24);

    // Wait 1.5 seconds so uptime_sec changes
    vTaskDelay(pdMS_TO_TICKS(1500));

    // Second call
    dronecan_send_node_status();
    vTaskDelay(pdMS_TO_TICKS(10));

    twai_message_t rx2;
    err = twai_receive(&rx2, pdMS_TO_TICKS(100));
    TEST_ASSERT_EQUAL(ESP_OK, err);

    uint32_t uptime2 = (uint32_t)rx2.data[0] |
                       ((uint32_t)rx2.data[1] << 8) |
                       ((uint32_t)rx2.data[2] << 16) |
                       ((uint32_t)rx2.data[3] << 24);

    TEST_ASSERT_TRUE(uptime2 > uptime1);

    twai_deinit();
}

// ---------------------------------------------------------------------------
// Test 3: Transfer ID increments on successive calls
// ---------------------------------------------------------------------------

void test_node_status_transfer_id_increments(void) {
    twai_init_loopback();

    // First call
    dronecan_send_node_status();
    vTaskDelay(pdMS_TO_TICKS(10));

    twai_message_t rx1;
    esp_err_t err = twai_receive(&rx1, pdMS_TO_TICKS(100));
    TEST_ASSERT_EQUAL(ESP_OK, err);

    // Tail byte is the last byte of the frame: data[data_length_code - 1]
    // For single-frame: tail byte = start_of_transfer(1) | end_of_transfer(1) |
    //                   toggle(0) | transfer_id(5 bits)
    uint8_t tail1 = rx1.data[rx1.data_length_code - 1];
    uint8_t tid1 = tail1 & 0x1F;  // lower 5 bits

    // Second call
    dronecan_send_node_status();
    vTaskDelay(pdMS_TO_TICKS(10));

    twai_message_t rx2;
    err = twai_receive(&rx2, pdMS_TO_TICKS(100));
    TEST_ASSERT_EQUAL(ESP_OK, err);

    uint8_t tail2 = rx2.data[rx2.data_length_code - 1];
    uint8_t tid2 = tail2 & 0x1F;

    // Transfer ID should have incremented by 1 (modulo 32)
    TEST_ASSERT_EQUAL((tid1 + 1) & 0x1F, tid2);

    twai_deinit();
}

// ---------------------------------------------------------------------------
// Arduino / Unity entry points
// ---------------------------------------------------------------------------

void setup(void) {
    delay(2000);
    UNITY_BEGIN();

    RUN_TEST(test_node_status_payload_format);
    RUN_TEST(test_node_status_uptime_increments);
    RUN_TEST(test_node_status_transfer_id_increments);

    UNITY_END();
}

void loop(void) {}
