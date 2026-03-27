# CLAUDE.md — DroneCAN-to-NMEA Bridge

This file is the authoritative implementation guide for this project.
Read it fully before touching any source file.

---

## Project Purpose

Bridge an ARK GPS module (speaking DroneCAN at 1 Mbps) to NMEA 0183 consumers:
- **Wired RS485 serial** → Standard Horizon GX1600 VHF radio (GPS DATA IN port)
- **WiFi TCP** → iPad chartplotter apps + Raspberry Pi running OpenPlotter

The ESP32 is a full DroneCAN node (not a passive listener). It must participate on
the bus with a heartbeat, not just sniff frames.

---

## Hardware

| Component | Notes |
|---|---|
| ESP32 DEVKIT V1 DOIT (30-pin) | Target board |
| ARK GPS (Ublox M9N + BMM150 + BMP388 + ICM-42688-P) | DroneCAN source |
| TJA1050 CAN Bus Transceiver | CAN ↔ UART level conversion |
| TTL-to-RS485 converter module | UART2 → differential serial |
| Standard Horizon GX1600 VHF | Wired NMEA consumer (4800 baud) |
| iPad | WiFi TCP NMEA consumer |
| Raspberry Pi / OpenPlotter | WiFi TCP NMEA consumer |

### Pin Assignments

| Signal | GPIO |
|---|---|
| CAN TX → TJA1050 | GPIO 5 |
| CAN RX ← TJA1050 | GPIO 4 |
| UART2 TX → RS485 module DI | GPIO 17 |
| UART2 RX ← RS485 module RO | GPIO 16 |
| RS485 DE/RE enable (active HIGH = transmit) | GPIO 18 |
| Status LED | GPIO 2 (onboard) |

**Do not change pin assignments without updating `config.h` and this file.**

---

## Toolchain & Dependencies

- **Build system:** PlatformIO with Arduino framework for ESP32
- **`platformio.ini` target:** `esp32doit-devkit-v1`
- **CAN library:** `libcanard` (DroneCAN reference implementation)
  — clone into `lib/libcanard/` as a local dependency
  — source: https://github.com/dronecan/libcanard
  — use the `legacy-v0` branch (DroneCAN v0 / UAVCAN v0, which is what ARK GPS speaks)
- **NMEA generation:** hand-rolled (no library). See nmea_generator module below.
  Rationale: NMEA generation libraries are sparse and poorly maintained.
  The output format is simple enough that `snprintf` + XOR checksum covers everything cleanly.
- **WiFi/TCP:** Arduino `WiFi.h` + `WiFiServer.h` — no additional library needed
- **NVS (non-volatile storage):** Arduino `Preferences.h` — for persisting WiFi credentials

---

## Module Structure

```
src/
  main.cpp                ← setup(), loop(), FreeRTOS task wiring
  config.h                ← ALL pin defs, baud rates, timing, feature flags
  can_driver.cpp/.h       ← ESP32 TWAI peripheral init, frame RX/TX
  dronecan_handler.cpp/.h ← libcanard instance, transfer receivers, SensorData population
  nmea_generator.cpp/.h   ← sentence assembly, checksum, timing control
  serial_output.cpp/.h    ← UART2 init, RS485 DE toggling, sentence transmission
  wifi_output.cpp/.h      ← SoftAP + TCP server, multi-client management
lib/
  libcanard/              ← cloned from github (do not edit)
```

---

## config.h — All Tunables Live Here

```cpp
// --- CAN / DroneCAN ---
#define CAN_TX_PIN       GPIO_NUM_5
#define CAN_RX_PIN       GPIO_NUM_4
#define DRONECAN_NODE_ID 100          // pick any unused node ID 1–127

// --- RS485 Serial ---
#define UART2_TX_PIN     17
#define UART2_RX_PIN     16
#define RS485_DE_PIN     18
#define SERIAL_BAUD      4800         // GX1600 GPS IN expects 4800 by default

// --- WiFi ---
#define WIFI_AP_SSID     "ESP32-GPS"
#define WIFI_AP_PASS     "dronecan1"
#define WIFI_TCP_PORT    10110        // IANA assigned for NMEA-0183 over IP

// --- NMEA Sentence Enable Flags ---
#define EMIT_RMC         true   // Required by GX1600
#define EMIT_GGA         true   // Required by OpenPlotter
#define EMIT_VTG         true
#define EMIT_HDM         true   // Magnetic heading — requires magnetometer data
#define EMIT_GSA         true   // DOP values
#define EMIT_XDR_BARO    false  // Phase 2 — barometric pressure
#define EMIT_XDR_TEMP    false  // Phase 2 — air temperature

// --- Timing ---
#define GPS_OUTPUT_HZ    5      // Rate to emit RMC/GGA/VTG (match Fix2 rate)
#define HDM_OUTPUT_HZ    10     // Magnetic heading output rate
#define AUX_OUTPUT_HZ    1      // GSA / XDR output rate
#define CAN_SILENCE_TIMEOUT_MS 5000  // Emit void sentences after this
```

---

## DroneCAN Message Subscriptions

Register receivers for these Data Type IDs (DTIDs) in `dronecan_handler.cpp`:

| Message | DTID | Fields Used |
|---|---|---|
| `uavcan.equipment.gnss.Fix2` | 1063 | timestamp, lat, lon, alt, ned_velocity[3], fix_type, num_sats, pdop |
| `uavcan.equipment.gnss.Auxiliary` | 1062 | pdop, hdop, vdop |
| `uavcan.equipment.ahrs.MagneticFieldStrength2` | 1001 | magnetic_field[3] (x,y,z in Gauss) |
| `uavcan.equipment.air_data.StaticPressure` | 1028 | static_pressure (Pa) |
| `uavcan.equipment.air_data.StaticTemperature` | 1029 | static_temperature (K) |
| `uavcan.equipment.imu.RawIMU` | 1003 | accelerometer_latest[3], rate_gyro_latest[3] |
| `uavcan.protocol.NodeStatus` | 341 | (TX only — broadcast at 1 Hz as mandatory heartbeat) |

### SensorData Struct

Declare in `dronecan_handler.h`, updated from callbacks, read from nmea_generator:

```cpp
struct SensorData {
    // GPS Fix2
    bool     fix_valid;
    uint8_t  fix_type;        // 0=no fix, 2=2D, 3=3D
    uint8_t  num_sats;
    double   lat_deg;         // decimal degrees, positive = North
    double   lon_deg;         // decimal degrees, positive = East
    float    alt_m;           // altitude MSL
    float    vel_n_ms;        // NED velocity north (m/s)
    float    vel_e_ms;        // NED velocity east (m/s)
    float    vel_d_ms;        // NED velocity down (m/s)
    uint64_t timestamp_usec;  // microseconds since epoch (from Fix2)

    // Auxiliary (DOP)
    float    pdop;
    float    hdop;
    float    vdop;

    // Magnetometer
    bool     mag_valid;
    float    mag_x;           // Gauss
    float    mag_y;
    float    mag_z;

    // Baro / Temp
    float    pressure_pa;
    float    temperature_k;

    // IMU
    float    accel_x, accel_y, accel_z;   // m/s²
    float    gyro_x,  gyro_y,  gyro_z;    // rad/s

    // Staleness tracking
    uint32_t last_fix_ms;     // millis() at last Fix2 receive
    uint32_t last_mag_ms;
};

extern SensorData g_sensors;  // single global, protected by portENTER_CRITICAL
```

---

## NMEA Sentence Generation

All sentences go through a single helper:

```cpp
// Appends *XX\r\n checksum footer and writes into buf.
// Returns total length including footer.
int nmea_finalize(char* buf, int payload_len);

// XOR checksum of all bytes between $ and * (exclusive)
uint8_t nmea_checksum(const char* sentence);
```

### Sentence Formats

**`$GPRMC`** (emit at GPS_OUTPUT_HZ)
```
$GPRMC,HHMMSS.ss,A,DDMM.MMMM,N,DDDMM.MMMM,W,SSS.S,CCC.C,DDMMYY,,,A*XX\r\n
```
- Status field: `A` if fix_type >= 2, else `V`
- SOG: compute from `sqrt(vel_n² + vel_e²)` → convert m/s to knots (× 1.94384)
- COG: `atan2(vel_e, vel_n)` in degrees true
- Lat/Lon: convert decimal degrees to DDMM.MMMM format (integer degrees × 100 + fractional minutes)

**`$GPGGA`** (emit at GPS_OUTPUT_HZ)
```
$GPGGA,HHMMSS.ss,DDMM.MMMM,N,DDDMM.MMMM,W,Q,NN,H.H,AAA.A,M,,M,,*XX\r\n
```
- Q (fix quality): 0=invalid, 1=GPS fix, 2=DGPS

**`$GPVTG`** (emit at GPS_OUTPUT_HZ)
```
$GPVTG,CCC.C,T,,M,SSS.S,N,SSS.S,K,A*XX\r\n
```

**`$GPGSA`** (emit at AUX_OUTPUT_HZ)
```
$GPGSA,A,F,,,,,,,,,,,,P.P,H.H,V.V*XX\r\n
```
- F (fix): 1=no fix, 2=2D, 3=3D

**`$HCHDM`** (emit at HDM_OUTPUT_HZ — only if EMIT_HDM and mag_valid)
```
$HCHDM,HHH.H,M*XX\r\n
```
- Heading: `atan2(mag_y, mag_x)` converted to degrees 0–360
- Phase 2: apply tilt compensation using accel_x/y/z before atan2

**`$IIXDR`** (emit at AUX_OUTPUT_HZ — only if EMIT_XDR_BARO/TEMP)
```
$IIXDR,P,PPP.PPPPP,B,Barometer*XX\r\n   (pressure in bar = Pa / 100000)
$IIXDR,C,TTT.TT,C,AirTemp*XX\r\n        (temp in °C = K - 273.15)
```

---

## WiFi Output

- Boot as **SoftAP** with credentials from `config.h`
- If NVS contains saved STA credentials, attempt STA connection first with 10-second timeout; fall back to AP mode
- TCP server on `WIFI_TCP_PORT` (10110)
- Maintain a `WiFiClient clients[4]` array; check `client.connected()` before each write, drop and slot if disconnected
- **Also broadcast sentences as UDP to 255.255.255.255:10110** — this allows iPad apps (Navionics, iSailor) to auto-receive without manual IP entry. Make this a config flag `WIFI_UDP_BROADCAST`.

### OpenPlotter Connection
Add a "TCP Client" input in OpenPlotter's Signal K / NMEA0183 input panel pointing to the ESP32's AP IP (`192.168.4.1`) port `10110`.

---

## Serial RS485 Output

```cpp
// In serial_output.cpp
void serial_output_init() {
    Serial2.begin(SERIAL_BAUD, SERIAL_8N1, UART2_RX_PIN, UART2_TX_PIN);
    pinMode(RS485_DE_PIN, OUTPUT);
    digitalWrite(RS485_DE_PIN, LOW); // default = receive mode
}

void serial_send_sentence(const char* sentence, int len) {
    digitalWrite(RS485_DE_PIN, HIGH);
    delayMicroseconds(100);           // DE settling time
    Serial2.write(sentence, len);
    Serial2.flush();                  // wait for TX complete
    delayMicroseconds(100);
    digitalWrite(RS485_DE_PIN, LOW);
}
```

Transmit sentence order per update cycle: RMC → GGA → VTG → HDM.
The GX1600 only strictly needs RMC; confirm from the manual and adjust `EMIT_*` flags if needed.

---

## CAN Bus Robustness

- Register TWAI alerts for `TWAI_ALERT_BUS_OFF` and `TWAI_ALERT_BUS_ERROR`
- On `TWAI_ALERT_BUS_OFF`: call `twai_initiate_recovery()` automatically
- On CAN silence > `CAN_SILENCE_TIMEOUT_MS`: set `g_sensors.fix_valid = false` and emit RMC with status `V` so downstream devices know GPS is lost — do not just go silent
- Feed `esp_task_wdt` in the main loop; watchdog timeout = 10 seconds

---

## LED Status Codes (GPIO 2)

| Pattern | Meaning |
|---|---|
| Solid ON | CAN active, GPS fix valid |
| 1 Hz blink | WiFi up, waiting for CAN data |
| 3 Hz blink | CAN active, no GPS fix |
| SOS pattern | TWAI bus-off error |

---

## FreeRTOS Task Structure

Two tasks pinned to separate cores:

```
Core 0: canTask    — TWAI RX poll → canardHandleRxFrame → SensorData update
Core 1: outputTask — NMEA generation → WiFi TCP/UDP broadcast → RS485 serial TX
```

Shared state (`g_sensors`) protected with `portENTER_CRITICAL` / `portEXIT_CRITICAL` spinlock (fast writes from canTask, fast reads from outputTask — no blocking needed).

---

## Implementation Phases

Execute in order. Verify each phase before starting the next.

### Phase 1 — Scaffold
- [ ] Create `platformio.ini` for `esp32doit-devkit-v1`, Arduino framework
- [ ] Clone libcanard into `lib/libcanard/` (legacy-v0 branch)
- [ ] Create all `.cpp`/`.h` files with empty stubs
- [ ] Confirm project compiles clean with no source yet

### Phase 2 — CAN Driver
- [ ] Implement `can_driver.cpp`: TWAI init at 1 Mbps on GPIO 4/5
- [ ] Poll RX queue in a loop, return `twai_message_t`
- [ ] Implement TX path (needed for NodeStatus heartbeat)
- [ ] Verify with a logic analyzer or serial print of raw frame IDs

### Phase 3 — DroneCAN Handler
- [ ] Initialize canard instance with static memory pool (~4 KB)
- [ ] Register transfer receivers for DTIDs: 1063, 1062, 1001, 1028, 1029, 1003
- [ ] Implement DSDL deserialization for each (use ARK GPS DSDL definitions)
- [ ] Populate `g_sensors` in each callback
- [ ] Implement NodeStatus broadcast at 1 Hz
- [ ] Verify: serial-print lat/lon from Fix2 callback

### Phase 4 — NMEA Generator
- [ ] Implement `nmea_checksum()` and `nmea_finalize()`
- [ ] Implement `build_rmc()`, `build_gga()`, `build_vtg()`
- [ ] Implement `build_hdm()` (atan2 on mag_x/mag_y)
- [ ] Implement `build_gsa()`, `build_xdr()` (baro/temp)
- [ ] Verify: serial-print generated sentences, validate checksum with an online NMEA checker

### Phase 5 — Serial Output
- [ ] Implement `serial_output.cpp` with UART2 + RS485 DE toggling
- [ ] Transmit RMC + GGA at configured rate
- [ ] Verify with USB-serial adapter monitoring the RS485 line

### Phase 6 — WiFi Output
- [ ] Implement SoftAP + TCP server
- [ ] Implement multi-client management and sentence broadcast
- [ ] Add UDP broadcast option
- [ ] Verify: connect with a terminal (`nc 192.168.4.1 10110`) and observe NMEA stream
- [ ] Test with OpenPlotter and iPad app

### Phase 7 — Hardening
- [ ] Watchdog integration
- [ ] TWAI bus-off recovery
- [ ] GPS-loss void sentence emission
- [ ] LED status codes
- [ ] NVS credential storage

### Phase 8 (Future) — AHRS / Tilt Compensation
- [ ] Tilt-compensated magnetic heading using accel roll/pitch
- [ ] Dead-reckoning fill-in during GPS dropout
- [ ] Evaluate proprietary NMEA sentences for OpenPlotter AHRS input

---

## Key Constraints — Never Violate These

1. **Never block the canTask loop.** No `delay()`, no `Serial.print()`, no WiFi calls on Core 0.
2. **Always protect `g_sensors` with the spinlock** when reading or writing across task boundaries.
3. **Always transmit NodeStatus at 1 Hz.** The ARK GPS may go silent if it sees no live node on the bus.
4. **NMEA checksums must be correct.** Downstream devices silently discard sentences with bad checksums — this is a very hard bug to find.
5. **RS485 DE pin must go LOW after every transmission.** Leaving it HIGH will block the line.
6. **All tunables go in `config.h`.** No magic numbers in source files.

---

## Reference Links

- ARK GPS product page: https://arkelectron.com/product/ark-gps/
- ARK GPS GitHub (DSDL, schematics): https://github.com/ARK-Electronics/ARK_GPS
- DroneCAN spec: https://dronecan.github.io/
- libcanard (DroneCAN v0): https://github.com/dronecan/libcanard (legacy-v0 branch)
- NMEA 0183 sentence reference: https://www.nmea.org/content/STANDARDS/NMEA_0183_Standard
- Standard Horizon GX1600 manual: search "GX1600 owner's manual" for GPS DATA IN wiring and baud rate confirmation
- OpenPlotter NMEA input setup: https://openplotter.readthedocs.io
- IANA port 10110 (NMEA-0183): https://www.iana.org/assignments/service-names-port-numbers
