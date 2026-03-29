#include "nmea_generator.h"
#include "dronecan_handler.h"
#include "config.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <Arduino.h>

// ---------------------------------------------------------------------------
// Core helpers
// ---------------------------------------------------------------------------

uint8_t nmea_checksum(const char* sentence) {
    uint8_t csum = 0;
    // Skip leading '$'
    const char* p = sentence;
    if (*p == '$') {
        p++;
    }
    while (*p != '\0' && *p != '*') {
        csum ^= (uint8_t)*p;
        p++;
    }
    return csum;
}

int nmea_finalize(char* buf, int payload_len) {
    uint8_t csum = nmea_checksum(buf);
    int added = snprintf(buf + payload_len, 8, "*%02X\r\n", csum);
    return payload_len + added;
}

// Convert absolute decimal degrees to DDMM.MMMM string.
// dd_digits: 2 for latitude, 3 for longitude.
static void deg_to_ddmm(double deg_abs, int dd_digits, char* out, int out_size) {
    int degrees = (int)deg_abs;
    double minutes = (deg_abs - (double)degrees) * 60.0;
    if (dd_digits == 2) {
        snprintf(out, out_size, "%02d%07.4f", degrees, minutes);
    } else {
        snprintf(out, out_size, "%03d%07.4f", degrees, minutes);
    }
}

// UTC microseconds since Unix epoch → time-of-day components.
static void usec_to_utc(uint64_t usec, int* hh, int* mm, int* ss, int* ss_hundredths) {
    uint64_t total_sec  = usec / 1000000ULL;
    uint32_t time_of_day = (uint32_t)(total_sec % 86400ULL);
    *hh            = (int)(time_of_day / 3600);
    *mm            = (int)((time_of_day % 3600) / 60);
    *ss            = (int)(time_of_day % 60);
    *ss_hundredths = (int)((usec % 1000000ULL) / 10000ULL);
}

// UTC microseconds since Unix epoch → calendar date components (DD, MM, YY).
static void usec_to_date(uint64_t usec, int* dd, int* mon, int* yy) {
    uint32_t total_days = (uint32_t)(usec / (86400ULL * 1000000ULL));

    // Walk through years from 1970
    int year = 1970;
    while (true) {
        bool leap = ((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0);
        uint32_t days_in_year = leap ? 366 : 365;
        if (total_days < days_in_year) {
            break;
        }
        total_days -= days_in_year;
        year++;
    }

    static const int days_per_month[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    bool leap = ((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0);

    int month = 0;
    for (month = 0; month < 12; month++) {
        int dim = days_per_month[month];
        if (month == 1 && leap) {
            dim = 29;
        }
        if ((int)total_days < dim) {
            break;
        }
        total_days -= (uint32_t)dim;
    }

    *dd  = (int)total_days + 1;   // 1-based day
    *mon = month + 1;              // 1-based month
    *yy  = year % 100;             // 2-digit year
}

// ---------------------------------------------------------------------------
// Sentence builders
// ---------------------------------------------------------------------------

// Helper: compute SOG (knots) and COG (degrees true) from NED velocities.
static void compute_sog_cog(float vel_n, float vel_e, float* sog_knots, float* cog_deg) {
    *sog_knots = sqrtf(vel_n * vel_n + vel_e * vel_e) * 1.94384f;
    float cog = atan2f(vel_e, vel_n) * 180.0f / (float)M_PI;
    if (cog < 0.0f) {
        cog += 360.0f;
    }
    *cog_deg = cog;
}

const char* build_rmc(int* out_len) {
    static char buf[128];

    // Copy sensor state under spinlock
    SensorData s;
    portENTER_CRITICAL(&g_sensors_mux);
    s = g_sensors;
    portEXIT_CRITICAL(&g_sensors_mux);

    char time_str[16] = "000000.00";
    char date_str[8]  = "010170";
    if (s.timestamp_usec > 0) {
        int hh, mm, ss, hun;
        usec_to_utc(s.timestamp_usec, &hh, &mm, &ss, &hun);
        snprintf(time_str, sizeof(time_str), "%02d%02d%02d.%02d", hh, mm, ss, hun);
        int dd, mon, yy;
        usec_to_date(s.timestamp_usec, &dd, &mon, &yy);
        snprintf(date_str, sizeof(date_str), "%02d%02d%02d", dd, mon, yy);
    }

    char status = s.fix_valid ? 'A' : 'V';

    char lat_str[16] = "0000.0000";
    char lon_str[16] = "00000.0000";
    char ns = 'N';
    char ew = 'E';
    float sog = 0.0f;
    float cog = 0.0f;

    if (s.fix_valid) {
        ns = (s.lat_deg >= 0.0) ? 'N' : 'S';
        ew = (s.lon_deg >= 0.0) ? 'E' : 'W';
        deg_to_ddmm(fabs(s.lat_deg), 2, lat_str, sizeof(lat_str));
        deg_to_ddmm(fabs(s.lon_deg), 3, lon_str, sizeof(lon_str));
        compute_sog_cog(s.vel_n_ms, s.vel_e_ms, &sog, &cog);
    }

    // $GPRMC,HHMMSS.ss,A,DDMM.MMMM,N,DDDMM.MMMM,W,SSS.S,CCC.C,DDMMYY,,,A
    int n = snprintf(buf, sizeof(buf),
                     "$GPRMC,%s,%c,%s,%c,%s,%c,%.1f,%.1f,%s,,,A",
                     time_str, status,
                     lat_str, ns,
                     lon_str, ew,
                     sog, cog,
                     date_str);

    *out_len = nmea_finalize(buf, n);
    return buf;
}

const char* build_gga(int* out_len) {
    static char buf[128];

    SensorData s;
    portENTER_CRITICAL(&g_sensors_mux);
    s = g_sensors;
    portEXIT_CRITICAL(&g_sensors_mux);

    char time_str[16] = "000000.00";
    if (s.timestamp_usec > 0) {
        int hh, mm, ss, hun;
        usec_to_utc(s.timestamp_usec, &hh, &mm, &ss, &hun);
        snprintf(time_str, sizeof(time_str), "%02d%02d%02d.%02d", hh, mm, ss, hun);
    }

    char lat_str[16] = "0000.0000";
    char lon_str[16] = "00000.0000";
    char ns = 'N';
    char ew = 'E';
    int quality = 0;

    if (s.fix_valid) {
        ns = (s.lat_deg >= 0.0) ? 'N' : 'S';
        ew = (s.lon_deg >= 0.0) ? 'E' : 'W';
        deg_to_ddmm(fabs(s.lat_deg), 2, lat_str, sizeof(lat_str));
        deg_to_ddmm(fabs(s.lon_deg), 3, lon_str, sizeof(lon_str));
        quality = (s.fix_type >= 2) ? 1 : 0;
    }

    // $GPGGA,HHMMSS.ss,DDMM.MMMM,N,DDDMM.MMMM,W,Q,NN,H.H,AAA.A,M,,M,,
    int n = snprintf(buf, sizeof(buf),
                     "$GPGGA,%s,%s,%c,%s,%c,%d,%02d,%.1f,%.1f,M,,M,,",
                     time_str,
                     lat_str, ns,
                     lon_str, ew,
                     quality,
                     (int)s.num_sats,
                     (s.hdop > 0.0f ? s.hdop : 99.9f),
                     s.alt_m);

    *out_len = nmea_finalize(buf, n);
    return buf;
}

const char* build_vtg(int* out_len) {
    static char buf[128];

    SensorData s;
    portENTER_CRITICAL(&g_sensors_mux);
    s = g_sensors;
    portEXIT_CRITICAL(&g_sensors_mux);

    float sog = 0.0f;
    float cog = 0.0f;
    if (s.fix_valid) {
        compute_sog_cog(s.vel_n_ms, s.vel_e_ms, &sog, &cog);
    }

    float sog_kmh = sog * 1.852f;

    // $GPVTG,CCC.C,T,,M,SSS.S,N,KKK.K,K,A
    int n = snprintf(buf, sizeof(buf),
                     "$GPVTG,%.1f,T,,M,%.1f,N,%.1f,K,A",
                     cog, sog, sog_kmh);

    *out_len = nmea_finalize(buf, n);
    return buf;
}

const char* build_gsa(int* out_len) {
    static char buf[128];

    SensorData s;
    portENTER_CRITICAL(&g_sensors_mux);
    s = g_sensors;
    portEXIT_CRITICAL(&g_sensors_mux);

    // Fix field: 1=no fix, 2=2D, 3=3D
    int fix_field = s.fix_valid ? (int)s.fix_type : 1;
    if (fix_field < 1) fix_field = 1;
    if (fix_field > 3) fix_field = 3;

    float pdop = (s.pdop > 0.0f) ? s.pdop : 99.9f;
    float hdop = (s.hdop > 0.0f) ? s.hdop : 99.9f;
    float vdop = (s.vdop > 0.0f) ? s.vdop : 99.9f;

    // $GPGSA,A,F,,,,,,,,,,,,P.P,H.H,V.V
    int n = snprintf(buf, sizeof(buf),
                     "$GPGSA,A,%d,,,,,,,,,,,,%.1f,%.1f,%.1f",
                     fix_field, pdop, hdop, vdop);

    *out_len = nmea_finalize(buf, n);
    return buf;
}

const char* build_hdm(int* out_len) {
    static char buf[64];

    SensorData s;
    portENTER_CRITICAL(&g_sensors_mux);
    s = g_sensors;
    portEXIT_CRITICAL(&g_sensors_mux);

    if (!s.mag_valid) {
        *out_len = 0;
        return NULL;
    }

    float heading;
#if AHRS_TILT_COMP
    uint32_t now = millis();
    bool imu_fresh = (s.last_imu_ms > 0) && ((now - s.last_imu_ms) < IMU_STALE_MS);
    if (imu_fresh) {
        // Tilt-compensated heading using accelerometer roll/pitch
        float roll  = atan2f(s.accel_y, s.accel_z);
        float pitch = atan2f(-s.accel_x, sqrtf(s.accel_y * s.accel_y + s.accel_z * s.accel_z));

        float mag_x_comp = s.mag_x * cosf(pitch) + s.mag_z * sinf(pitch);
        float mag_y_comp = s.mag_x * sinf(roll) * sinf(pitch)
                         + s.mag_y * cosf(roll)
                         - s.mag_z * sinf(roll) * cosf(pitch);

        heading = atan2f(mag_y_comp, mag_x_comp) * 180.0f / (float)M_PI;
    } else
#endif
    {
        // Raw (untilted) magnetic heading
        heading = atan2f(s.mag_y, s.mag_x) * 180.0f / (float)M_PI;
    }

    if (heading < 0.0f) {
        heading += 360.0f;
    }

    // $HCHDM,HHH.H,M
    int n = snprintf(buf, sizeof(buf), "$HCHDM,%.1f,M", heading);

    *out_len = nmea_finalize(buf, n);
    return buf;
}

const char* build_xdr_baro(int* out_len) {
    static char buf[64];

    SensorData s;
    portENTER_CRITICAL(&g_sensors_mux);
    s = g_sensors;
    portEXIT_CRITICAL(&g_sensors_mux);

    // pressure in bar = Pa / 100000
    float pressure_bar = s.pressure_pa / 100000.0f;

    // $IIXDR,P,PPP.PPPPP,B,Barometer
    int n = snprintf(buf, sizeof(buf), "$IIXDR,P,%.5f,B,Barometer", pressure_bar);

    *out_len = nmea_finalize(buf, n);
    return buf;
}

const char* build_xdr_temp(int* out_len) {
    static char buf[64];

    SensorData s;
    portENTER_CRITICAL(&g_sensors_mux);
    s = g_sensors;
    portEXIT_CRITICAL(&g_sensors_mux);

    // temp in Celsius = K - 273.15
    float temp_c = s.temperature_k - 273.15f;

    // $IIXDR,C,TTT.TT,C,AirTemp
    int n = snprintf(buf, sizeof(buf), "$IIXDR,C,%.2f,C,AirTemp", temp_c);

    *out_len = nmea_finalize(buf, n);
    return buf;
}
