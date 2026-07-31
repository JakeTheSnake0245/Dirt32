/*
 * GpsUart — Quectel L76K GNSS driver for the Heltec WiFi LoRa 32 V4
 * plug-in module (NMEA-0183 over UART, 9600 baud).
 *
 * Pin map (per Heltec's official V4 example, LoRaWanGPSTime_lora_v4.ino):
 *   GPIO39  UART RX  (L76K TX -> CPU)   — NOTE: header silk names are from
 *   GPIO38  UART TX  (CPU -> L76K RX)     the module's perspective, not ours
 *   GPIO34  VGNSS_Ctrl power enable, ACTIVE LOW (LOW = GPS powered)
 *   GPIO42  reset, held HIGH (pull LOW > 100 ms to reset; never pulsed)
 *
 * The module is fully powered off between fixes — a buried node only takes
 * a fix opportunistically at heartbeat time (spec §5.2); position falls
 * back to the provisioned coordinates when no fix arrives in budget.
 *
 * Parses $GxRMC (position + UTC date/time) and $GxGGA (sats/HDOP), with
 * checksum verification. On a valid fix the ESP32 system clock is set
 * (settimeofday) so alert timestamps are real UTC between fixes.
 */
#pragma once
#include <Arduino.h>

struct GpsFixResult {
    bool     valid = false;
    int32_t  lat_e7 = 0;      /* degrees * 1e7, +N */
    int32_t  lon_e7 = 0;      /* degrees * 1e7, +E */
    uint32_t unix_time = 0;   /* UTC epoch seconds, 0 if unknown */
    uint8_t  sats = 0;
    uint16_t hdop_x10 = 0;
};

class GpsUart {
public:
    /* Powers the module ON and opens the UART. */
    void powerOn();
    /* Cuts module power and closes the UART (deep-sleep safe). */
    void powerOff();

    /* Block up to timeout_s waiting for a valid RMC fix with date.
       Caller must powerOn() first; call powerOff() after. */
    GpsFixResult acquire(uint16_t timeout_s);

    /* Stream raw NMEA to a console for `gpstest` debugging. */
    void passthrough(Stream &out, uint16_t seconds);

private:
    char   _line[120];
    size_t _len = 0;
    GpsFixResult _partial;    /* GGA fields merged into the RMC fix */

    bool feedChar(char c, GpsFixResult &out);   /* true when fix complete */
    bool parseLine(const char *line, GpsFixResult &out);
    static bool checksumOk(const char *line);
    static int32_t nmeaCoordE7(const char *field, char hemi);
    static uint32_t rmcToUnix(const char *utc, const char *date);
};
