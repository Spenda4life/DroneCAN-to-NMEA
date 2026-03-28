/**
 * test/test_can_driver/test_can_driver.cpp
 *
 * Embedded unit tests for can_driver.cpp.
 * Run with: pio test -e test_embedded
 *
 * Uses TWAI_MODE_NO_ACK (self-test / loopback) so the ESP32 can transmit
 * and receive its own frames without a second CAN node on the bus.
 * A TJA1050 transceiver must still be wired to GPIO 4/5 for the TWAI
 * peripheral to function, but no remote node is required.
 */

#include <Arduino.h>
#include <unity.h>
#include <string.h>
#include <driver/twai.h>

#include "can_driver.h"
#include "config.h"

// We need access to canard frame flags
extern "C" {
#include "canard.h"
}

// ---------------------------------------------------------------------------
// Helper: initialize TWAI in NO_ACK (loopback) mode for self-test
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

// Helper: transmit a raw twai_message_t and wait briefly for loopback
static void loopback_transmit(const twai_message_t* msg) {
    ESP_ERROR_CHECK(twai_transmit(msg, pdMS_TO_TICKS(100)));
    vTaskDelay(pdMS_TO_TICKS(10)); // allow loopback to complete
}

// ---------------------------------------------------------------------------
// setUp / tearDown
// ---------------------------------------------------------------------------

void setUp(void) {
    g_bus_off = false;
}

void tearDown(void) {}

// ---------------------------------------------------------------------------
// Test 1: can_driver_init succeeds and TWAI is running
// ---------------------------------------------------------------------------

void test_can_driver_init_succeeds(void) {
    can_driver_init();

    twai_status_info_t status;
    esp_err_t err = twai_get_status_info(&status);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL(TWAI_STATE_RUNNING, status.state);

    // Clean up for subsequent tests
    twai_stop();
    twai_driver_uninstall();
}

// ---------------------------------------------------------------------------
// Test 2: can_receive returns false when RX queue is empty
// ---------------------------------------------------------------------------

void test_can_receive_returns_false_when_empty(void) {
    twai_init_loopback();

    CanardCANFrame frame;
    memset(&frame, 0, sizeof(frame));
    bool got = can_receive(&frame);
    TEST_ASSERT_FALSE(got);

    twai_deinit();
}

// ---------------------------------------------------------------------------
// Test 3: can_receive sets EFF flag on received extended-ID frame
// ---------------------------------------------------------------------------

void test_can_receive_sets_eff_flag(void) {
    twai_init_loopback();

    twai_message_t tx_msg;
    memset(&tx_msg, 0, sizeof(tx_msg));
    tx_msg.identifier = 0x12345;
    tx_msg.extd = 1;
    tx_msg.data_length_code = 1;
    tx_msg.data[0] = 0xAA;
    loopback_transmit(&tx_msg);

    CanardCANFrame rx_frame;
    bool got = can_receive(&rx_frame);
    TEST_ASSERT_TRUE(got);
    TEST_ASSERT_TRUE((rx_frame.id & CANARD_CAN_FRAME_EFF) != 0);

    twai_deinit();
}

// ---------------------------------------------------------------------------
// Test 4: can_receive copies data correctly
// ---------------------------------------------------------------------------

void test_can_receive_copies_data(void) {
    twai_init_loopback();

    twai_message_t tx_msg;
    memset(&tx_msg, 0, sizeof(tx_msg));
    tx_msg.identifier = 0x100;
    tx_msg.extd = 1;
    tx_msg.data_length_code = 5;
    tx_msg.data[0] = 0xDE;
    tx_msg.data[1] = 0xAD;
    tx_msg.data[2] = 0xBE;
    tx_msg.data[3] = 0xEF;
    tx_msg.data[4] = 0x42;
    loopback_transmit(&tx_msg);

    CanardCANFrame rx_frame;
    bool got = can_receive(&rx_frame);
    TEST_ASSERT_TRUE(got);
    TEST_ASSERT_EQUAL(5, rx_frame.data_len);
    TEST_ASSERT_EQUAL_HEX8(0xDE, rx_frame.data[0]);
    TEST_ASSERT_EQUAL_HEX8(0xAD, rx_frame.data[1]);
    TEST_ASSERT_EQUAL_HEX8(0xBE, rx_frame.data[2]);
    TEST_ASSERT_EQUAL_HEX8(0xEF, rx_frame.data[3]);
    TEST_ASSERT_EQUAL_HEX8(0x42, rx_frame.data[4]);

    twai_deinit();
}

// ---------------------------------------------------------------------------
// Test 5: can_receive discards RTR frames
// ---------------------------------------------------------------------------

void test_can_receive_discards_rtr(void) {
    twai_init_loopback();

    twai_message_t tx_msg;
    memset(&tx_msg, 0, sizeof(tx_msg));
    tx_msg.identifier = 0x200;
    tx_msg.extd = 1;
    tx_msg.rtr = 1;
    tx_msg.data_length_code = 0;
    loopback_transmit(&tx_msg);

    CanardCANFrame rx_frame;
    bool got = can_receive(&rx_frame);
    TEST_ASSERT_FALSE(got);

    twai_deinit();
}

// ---------------------------------------------------------------------------
// Test 6: can_transmit strips canard flags, sends raw 29-bit ID
// ---------------------------------------------------------------------------

void test_can_transmit_strips_canard_flags(void) {
    twai_init_loopback();

    CanardCANFrame tx_frame;
    memset(&tx_frame, 0, sizeof(tx_frame));
    tx_frame.id = 0x1ABCDEF | CANARD_CAN_FRAME_EFF;
    tx_frame.data_len = 2;
    tx_frame.data[0] = 0x11;
    tx_frame.data[1] = 0x22;

    can_transmit(&tx_frame);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Receive via raw TWAI to inspect the on-wire message
    twai_message_t rx_msg;
    esp_err_t err = twai_receive(&rx_msg, pdMS_TO_TICKS(100));
    TEST_ASSERT_EQUAL(ESP_OK, err);
    // The raw identifier should be the 29-bit value without canard flags
    TEST_ASSERT_EQUAL_HEX32(0x1ABCDEF, rx_msg.identifier);

    twai_deinit();
}

// ---------------------------------------------------------------------------
// Test 7: can_transmit sets extd=1 (29-bit extended ID)
// ---------------------------------------------------------------------------

void test_can_transmit_sets_extd(void) {
    twai_init_loopback();

    CanardCANFrame tx_frame;
    memset(&tx_frame, 0, sizeof(tx_frame));
    tx_frame.id = 0x300 | CANARD_CAN_FRAME_EFF;
    tx_frame.data_len = 1;
    tx_frame.data[0] = 0xFF;

    can_transmit(&tx_frame);
    vTaskDelay(pdMS_TO_TICKS(10));

    twai_message_t rx_msg;
    esp_err_t err = twai_receive(&rx_msg, pdMS_TO_TICKS(100));
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_TRUE(rx_msg.extd);

    twai_deinit();
}

// ---------------------------------------------------------------------------
// Test 8: can_check_alerts returns false on healthy bus
// ---------------------------------------------------------------------------

void test_can_check_alerts_no_error(void) {
    twai_init_loopback();

    // Configure alerts (same as can_driver_init does)
    uint32_t alerts = TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_ERROR |
                      TWAI_ALERT_ERR_PASS | TWAI_ALERT_TX_FAILED;
    twai_reconfigure_alerts(alerts, NULL);

    g_bus_off = false;
    bool error = can_check_alerts();
    TEST_ASSERT_FALSE(error);
    TEST_ASSERT_FALSE(g_bus_off);

    twai_deinit();
}

// ---------------------------------------------------------------------------
// Arduino / Unity entry points
// ---------------------------------------------------------------------------

void setup(void) {
    delay(2000);
    UNITY_BEGIN();

    RUN_TEST(test_can_driver_init_succeeds);
    RUN_TEST(test_can_receive_returns_false_when_empty);
    RUN_TEST(test_can_receive_sets_eff_flag);
    RUN_TEST(test_can_receive_copies_data);
    RUN_TEST(test_can_receive_discards_rtr);
    RUN_TEST(test_can_transmit_strips_canard_flags);
    RUN_TEST(test_can_transmit_sets_extd);
    RUN_TEST(test_can_check_alerts_no_error);

    UNITY_END();
}

void loop(void) {}
