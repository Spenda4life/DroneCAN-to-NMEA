#include <Arduino.h>
#include <math.h>
#include <esp_task_wdt.h>

#include "config.h"
#include "can_driver.h"
#include "dronecan_handler.h"
#include "nmea_generator.h"
#include "serial_output.h"
#include "wifi_output.h"

// ---------------------------------------------------------------------------
// LED status helper
// ---------------------------------------------------------------------------
// Pattern timing state
static uint32_t led_last_toggle_ms = 0;
static int      led_sos_step       = 0;
static uint32_t led_sos_next_ms    = 0;
static bool     led_state          = false;

// SOS timing (ms): S = 3 short, O = 3 long, S = 3 short
// dot=200ms on/off, dash=600ms on/off, letter gap=600ms off
static const uint16_t sos_pattern[] = {
    200, 200,   // S dot 1
    200, 200,   // S dot 2
    200, 600,   // S dot 3 + letter gap
    600, 200,   // O dash 1
    600, 200,   // O dash 2
    600, 600,   // O dash 3 + letter gap
    200, 200,   // S dot 1
    200, 200,   // S dot 2
    200, 1400,  // S dot 3 + word gap
};
static const int sos_pattern_len = (int)(sizeof(sos_pattern) / sizeof(sos_pattern[0]));

static void led_update(void) {
    uint32_t now = millis();

    if (g_bus_off) {
        // SOS pattern
        if (now >= led_sos_next_ms) {
            if (led_sos_step % 2 == 0) {
                // Turn ON phase
                led_state = true;
            } else {
                // Turn OFF phase
                led_state = false;
            }
            digitalWrite(LED_STATUS_PIN, led_state ? HIGH : LOW);
            led_sos_next_ms = now + sos_pattern[led_sos_step];
            led_sos_step = (led_sos_step + 1) % sos_pattern_len;
        }
        return;
    }

    // Read g_sensors state with spinlock
    bool fix_valid;
    bool can_active;
    portENTER_CRITICAL(&g_sensors_mux);
    fix_valid  = g_sensors.fix_valid;
    can_active = (g_sensors.last_fix_ms > 0) &&
                 ((millis() - g_sensors.last_fix_ms) < CAN_SILENCE_TIMEOUT_MS);
    portEXIT_CRITICAL(&g_sensors_mux);

    if (can_active && fix_valid) {
        // Solid ON: CAN active, GPS fix valid
        digitalWrite(LED_STATUS_PIN, HIGH);
    } else if (can_active && !fix_valid) {
        // 3 Hz blink: CAN active, no GPS fix (~167 ms per half-period)
        if (now - led_last_toggle_ms >= 167) {
            led_last_toggle_ms = now;
            led_state = !led_state;
            digitalWrite(LED_STATUS_PIN, led_state ? HIGH : LOW);
        }
    } else {
        // 1 Hz blink: WiFi up, waiting for CAN data (~500 ms per half-period)
        if (now - led_last_toggle_ms >= 500) {
            led_last_toggle_ms = now;
            led_state = !led_state;
            digitalWrite(LED_STATUS_PIN, led_state ? HIGH : LOW);
        }
    }
}

// ---------------------------------------------------------------------------
// CAN task (Core 0) — TWAI RX poll → dronecan handler → SensorData update
// ---------------------------------------------------------------------------
static void canTask(void* pvParameters) {
    (void)pvParameters;

    CanardCANFrame frame;
    uint32_t node_status_timer = 0;

    // Subscribe this task to the watchdog
    esp_task_wdt_add(NULL);

    while (true) {
        // Poll RX queue (non-blocking)
        if (can_receive(&frame)) {
            dronecan_handle_frame(&frame, (uint64_t)millis() * 1000ULL);
        }

        // Check for TWAI bus alerts (non-blocking)
        can_check_alerts();

        // Broadcast NodeStatus heartbeat at 1 Hz
        uint32_t now = millis();
        if (now - node_status_timer >= 1000) {
            node_status_timer = now;
            dronecan_send_node_status();
        }

        // Feed watchdog
        esp_task_wdt_reset();

        // Yield to other tasks briefly (1 tick = 1 ms at default config)
        vTaskDelay(1);
    }
}

// ---------------------------------------------------------------------------
// Output task (Core 1) — NMEA generation → WiFi → RS485
// ---------------------------------------------------------------------------
static void outputTask(void* pvParameters) {
    (void)pvParameters;

    // Calculate output intervals from Hz settings
    const uint32_t GPS_INTERVAL_MS = 1000 / GPS_OUTPUT_HZ;
    const uint32_t HDM_INTERVAL_MS = 1000 / HDM_OUTPUT_HZ;
    const uint32_t AUX_INTERVAL_MS = 1000 / AUX_OUTPUT_HZ;

    uint32_t gps_timer = 0;
    uint32_t hdm_timer = 0;
    uint32_t aux_timer = 0;

    // Subscribe this task to the watchdog
    esp_task_wdt_add(NULL);

    while (true) {
        uint32_t now = millis();

        // Accept new TCP clients
        wifi_output_tick();

        // --- CAN silence detection ---
        // If no Fix2 has been received within the timeout, invalidate the fix
        {
            uint32_t last_fix;
            portENTER_CRITICAL(&g_sensors_mux);
            last_fix = g_sensors.last_fix_ms;
            portEXIT_CRITICAL(&g_sensors_mux);

            if (last_fix > 0 && (now - last_fix) > CAN_SILENCE_TIMEOUT_MS) {
                portENTER_CRITICAL(&g_sensors_mux);
                g_sensors.fix_valid = false;
                portEXIT_CRITICAL(&g_sensors_mux);
            }
        }

        // --- GPS output cycle: RMC → GGA → VTG ---
        if (now - gps_timer >= GPS_INTERVAL_MS) {
            gps_timer = now;

#if EMIT_RMC
            {
                int len = 0;
                const char* s = build_rmc(&len);
                if (s && len > 0) {
                    serial_send_sentence(s, len);
                    wifi_output_broadcast(s, len);
                }
            }
#endif

#if EMIT_GGA
            {
                int len = 0;
                const char* s = build_gga(&len);
                if (s && len > 0) {
                    serial_send_sentence(s, len);
                    wifi_output_broadcast(s, len);
                }
            }
#endif

#if EMIT_VTG
            {
                int len = 0;
                const char* s = build_vtg(&len);
                if (s && len > 0) {
                    serial_send_sentence(s, len);
                    wifi_output_broadcast(s, len);
                }
            }
#endif
        }

        // --- HDM output cycle ---
#if EMIT_HDM
        if (now - hdm_timer >= HDM_INTERVAL_MS) {
            hdm_timer = now;

            int len = 0;
            const char* s = build_hdm(&len);
            if (s && len > 0) {
                serial_send_sentence(s, len);
                wifi_output_broadcast(s, len);
            }
        }
#endif

        // --- AUX output cycle: GSA, XDR baro, XDR temp ---
        if (now - aux_timer >= AUX_INTERVAL_MS) {
            aux_timer = now;

#if EMIT_GSA
            {
                int len = 0;
                const char* s = build_gsa(&len);
                if (s && len > 0) {
                    serial_send_sentence(s, len);
                    wifi_output_broadcast(s, len);
                }
            }
#endif

#if EMIT_XDR_BARO
            {
                int len = 0;
                const char* s = build_xdr_baro(&len);
                if (s && len > 0) {
                    serial_send_sentence(s, len);
                    wifi_output_broadcast(s, len);
                }
            }
#endif

#if EMIT_XDR_TEMP
            {
                int len = 0;
                const char* s = build_xdr_temp(&len);
                if (s && len > 0) {
                    serial_send_sentence(s, len);
                    wifi_output_broadcast(s, len);
                }
            }
#endif
        }

        // Update LED status indicator
        led_update();

        // Feed watchdog and yield
        esp_task_wdt_reset();
        vTaskDelay(1);
    }
}

// ---------------------------------------------------------------------------
// setup / loop
// ---------------------------------------------------------------------------
void setup(void) {
    Serial.begin(115200);

    // Configure status LED
    pinMode(LED_STATUS_PIN, OUTPUT);
    digitalWrite(LED_STATUS_PIN, LOW);

    // Initialize watchdog with 10-second timeout; panic on expiry
    esp_task_wdt_init(10, true);

    // Initialize all subsystems
    can_driver_init();
    dronecan_init();
    serial_output_init();
    wifi_output_init();

    // Spawn CAN task on Core 0 with 4 KB stack
    xTaskCreatePinnedToCore(
        canTask,      // function
        "canTask",    // name
        4096,         // stack size (bytes)
        NULL,         // parameter
        1,            // priority
        NULL,         // handle (not needed)
        0             // core ID
    );

    // Spawn output task on Core 1 with 8 KB stack
    xTaskCreatePinnedToCore(
        outputTask,   // function
        "outputTask", // name
        8192,         // stack size (bytes)
        NULL,         // parameter
        1,            // priority
        NULL,         // handle (not needed)
        1             // core ID
    );
}

void loop(void) {
    // All work is done in FreeRTOS tasks; loop() never runs.
    vTaskDelay(portMAX_DELAY);
}
