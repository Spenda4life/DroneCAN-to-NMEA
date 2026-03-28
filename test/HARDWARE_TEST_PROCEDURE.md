# Hardware Integration Smoke Test Procedure

**Project:** DroneCAN-to-NMEA Bridge (ESP32 / ARK GPS)
**Required equipment:** ESP32 DEVKIT V1, ARK GPS, TJA1050 transceiver, TTL-to-RS485 module, USB-to-RS485 adapter, laptop/phone for WiFi

---

## Step 1 -- Pre-flight Wiring Check

- [ ] CAN TX (GPIO 5) connected to TJA1050 TXD
- [ ] CAN RX (GPIO 4) connected to TJA1050 RXD
- [ ] TJA1050 CANH/CANL connected to ARK GPS CANH/CANL
- [ ] TJA1050 VCC = 5V, GND shared with ESP32
- [ ] UART2 TX (GPIO 17) connected to RS485 module DI
- [ ] UART2 RX (GPIO 16) connected to RS485 module RO
- [ ] RS485 DE+RE (GPIO 18) connected to RS485 module DE and RE (active HIGH = transmit)
- [ ] ARK GPS powered at 5V
- [ ] Record ARK GPS firmware version (visible in Mission Planner DroneCAN GUI): ___________

---

## Step 2 -- Flash and Boot

- [ ] Flash firmware: `pio run -e esp32doit-devkit-v1 -t upload`
- [ ] Open serial monitor: `pio device monitor`
- [ ] Verify log: "TWAI initialized at 1 Mbps"
- [ ] Verify log: "DroneCAN initialized (node ID=100)"
- [ ] Verify log: "SoftAP started: SSID=ESP32-GPS"
- [ ] Verify log: "TCP server listening on port 10110"

---

## Step 3 -- CAN Bus Validation

- [ ] Observe onboard LED (GPIO 2):
  - 1 Hz blink = WiFi up, waiting for CAN data
  - 3 Hz blink = CAN active, no GPS fix
  - Solid ON = CAN active, GPS fix valid
- [ ] Temporarily add debug print in `decode_fix2()` after spinlock:
  ```cpp
  Serial.printf("Fix2: lat=%.6f lon=%.6f sats=%d fix=%d\n",
                g_sensors.lat_deg, g_sensors.lon_deg,
                g_sensors.num_sats, g_sensors.fix_type);
  ```
- [ ] Verify lat/lon values are sane for your location
- [ ] Record `gnss_time_standard` value reported by ARK GPS: ___________
  (Add a `Serial.printf("time_std=%d\n", gnss_time_standard)` in `decode_fix2()`)
- [ ] If a second CAN device is on the bus (e.g., Cube Orange), verify node 100 appears as alive in Mission Planner DroneCAN GUI

---

## Step 4 -- WiFi / TCP Output

- [ ] Connect laptop or phone to WiFi SSID "ESP32-GPS" (password: `dronecan1`)
- [ ] Run: `nc 192.168.4.1 10110` (or use a TCP terminal app)
- [ ] Verify NMEA sentences stream: `$GPRMC`, `$GPGGA`, `$GPVTG`, `$GPGSA`
- [ ] Copy one `$GPRMC` sentence and verify checksum with an online checker (e.g., https://nmeachecksum.eqth.net)
- [ ] Verify coordinates match expected location
- [ ] Verify UTC time/date are current

Sample expected output:
```
$GPRMC,221320.00,A,3352.1280,S,15112.5580,E,0.0,0.0,141123,,,A*XX
$GPGGA,221320.00,3352.1280,S,15112.5580,E,1,10,1.0,50.0,M,,M,,*XX
$GPVTG,0.0,T,,M,0.0,N,0.0,K,A*XX
$GPGSA,A,3,,,,,,,,,,,,1.5,1.2,2.0*XX
```

---

## Step 5 -- RS485 Serial Output

- [ ] Connect USB-to-RS485 adapter to the RS485 output lines (A/B)
- [ ] Open terminal at 4800 baud, 8N1
- [ ] Verify same NMEA sentences appear on serial output
- [ ] Verify RS485 DE pin (GPIO 18) returns LOW between transmissions
  (measure with multimeter or oscilloscope if available)

---

## Step 6 -- GPS Loss Behavior

- [ ] Disconnect the ARK GPS CAN cable
- [ ] Within 5 seconds (`CAN_SILENCE_TIMEOUT_MS`), verify:
  - [ ] LED transitions to 1 Hz blink
  - [ ] `$GPRMC` status field changes from `A` to `V`
  - [ ] `$GPGGA` quality field changes to `0`
- [ ] Reconnect ARK GPS CAN cable
- [ ] Verify recovery:
  - [ ] LED returns to solid ON (or 3 Hz blink if fix is reacquiring)
  - [ ] `$GPRMC` status returns to `A`
  - [ ] `$GPGGA` quality returns to `1`

---

## Step 7 -- Bus Error Recovery (Optional)

**WARNING:** Only perform this test if you understand the risks. Shorting CAN lines can damage hardware if done incorrectly.

- [ ] Briefly short CANH to GND to induce bus-off condition
- [ ] Verify LED enters SOS pattern (3 short, 3 long, 3 short)
- [ ] Remove the short
- [ ] Verify LED recovers to normal pattern within a few seconds
- [ ] Verify NMEA output resumes on WiFi and RS485

---

## Results

| Step | Result | Notes |
|------|--------|-------|
| 1. Pre-flight | PASS / FAIL | |
| 2. Flash and Boot | PASS / FAIL | |
| 3. CAN Bus Validation | PASS / FAIL | |
| 4. WiFi / TCP Output | PASS / FAIL | |
| 5. RS485 Serial Output | PASS / FAIL | |
| 6. GPS Loss Behavior | PASS / FAIL | |
| 7. Bus Error Recovery | PASS / FAIL / SKIPPED | |

**Overall:** PASS / FAIL

**Tester:** ___________
**Date:** ___________
**ARK GPS firmware:** ___________
**gnss_time_standard value:** ___________

---

## Post-Test Actions

- [ ] Remove any temporary `Serial.printf` debug statements added in Step 3
- [ ] Document `gnss_time_standard` value in `CLAUDE.md` under DroneCAN Message Subscriptions
- [ ] Any failure blocks downstream integration (GX1600 connection, OpenPlotter, iPad apps)
