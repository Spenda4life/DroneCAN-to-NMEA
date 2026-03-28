#include "wifi_output.h"
#include "config.h"

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiServer.h>
#include <WiFiUdp.h>
#include <esp_log.h>

static const char* TAG = "wifi_output";

#define MAX_TCP_CLIENTS 4

static WiFiServer  tcp_server(WIFI_TCP_PORT);
static WiFiClient  clients[MAX_TCP_CLIENTS];

#if WIFI_UDP_BROADCAST
static WiFiUDP udp;
#endif

void wifi_output_init(void) {
    // Start as SoftAP with configured SSID and password
    WiFi.mode(WIFI_AP);
    bool ok = WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
    if (!ok) {
        ESP_LOGE(TAG, "SoftAP start failed");
    } else {
        ESP_LOGI(TAG, "SoftAP started: SSID=%s IP=%s",
                 WIFI_AP_SSID,
                 WiFi.softAPIP().toString().c_str());
    }

    // Start TCP server
    tcp_server.begin();
    tcp_server.setNoDelay(true);
    ESP_LOGI(TAG, "TCP server listening on port %d", WIFI_TCP_PORT);

#if WIFI_UDP_BROADCAST
    udp.begin(WIFI_TCP_PORT);
    ESP_LOGI(TAG, "UDP broadcast enabled on port %d", WIFI_TCP_PORT);
#endif
}

void wifi_output_tick(void) {
    // Accept any pending new connections
    WiFiClient new_client = tcp_server.available();
    if (new_client) {
        // Find an empty slot
        bool accepted = false;
        for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
            if (!clients[i] || !clients[i].connected()) {
                // Clear any stale client in this slot
                if (clients[i]) {
                    clients[i].stop();
                }
                clients[i] = new_client;
                accepted = true;
                ESP_LOGI(TAG, "New TCP client in slot %d: %s",
                         i, new_client.remoteIP().toString().c_str());
                break;
            }
        }
        if (!accepted) {
            // All slots full — reject the connection
            new_client.stop();
            ESP_LOGW(TAG, "TCP client rejected: all slots full");
        }
    }
}

void wifi_output_broadcast(const char* sentence, int len) {
    // Send to each connected TCP client; drop disconnected ones
    for (int i = 0; i < MAX_TCP_CLIENTS; i++) {
        if (clients[i]) {
            if (clients[i].connected()) {
                clients[i].write((const uint8_t*)sentence, len);
            } else {
                clients[i].stop();
                ESP_LOGI(TAG, "TCP client slot %d disconnected", i);
            }
        }
    }

#if WIFI_UDP_BROADCAST
    // Broadcast to subnet (255.255.255.255)
    udp.beginPacket("255.255.255.255", WIFI_TCP_PORT);
    udp.write((const uint8_t*)sentence, len);
    udp.endPacket();
#endif
}
