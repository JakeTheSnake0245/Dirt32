#include "GpsUart.h"
#include <sys/time.h>

#ifndef PIN_GPS_RX
#define PIN_GPS_RX 38       /* L76K TX -> CPU */
#endif
#ifndef PIN_GPS_TX
#define PIN_GPS_TX 39       /* CPU -> L76K RX */
#endif
#ifndef PIN_GPS_EN
#define PIN_GPS_EN 34       /* active HIGH — enables VGNSS rail */
#endif
#ifndef PIN_GPS_RESET
#define PIN_GPS_RESET 42    /* LOW > 100 ms resets */
#endif
#ifndef GPS_BAUD
#define GPS_BAUD 9600
#endif

/* Heltec-documented method: Serial1 on RX=38 / TX=39, 9600 baud. */
static HardwareSerial &gpsSerial = Serial1;

void GpsUart::powerOn() {
    /* Heltec V4.3 documented GNSS method — nothing more:
     *   EN (GPIO34) HIGH  = VGNSS rail on
     *   RESET (GPIO42) HIGH = not in reset (no pulse; L76K self-starts)
     * L76K needs ~1 s after power-on before first NMEA sentence appears. */
    pinMode(PIN_GPS_EN, OUTPUT);
    pinMode(PIN_GPS_RESET, OUTPUT);
    digitalWrite(PIN_GPS_EN, HIGH);
    digitalWrite(PIN_GPS_RESET, HIGH);
    delay(1000);
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
    _len = 0;
    _partial = GpsFixResult{};
}

void GpsUart::powerOff() {
    gpsSerial.end();
    digitalWrite(PIN_GPS_EN, LOW);        /* cut VGNSS power rail */
}

bool GpsUart::checksumOk(const char *line) {
    if (line[0] != '$') return false;
    const char *star = strrchr(line, '*');
    if (!star || strlen(star) < 3) return false;
    uint8_t sum = 0;
    for (const char *p = line + 1; p < star; p++) sum ^= (uint8_t)*p;
    return sum == (uint8_t)strtoul(star + 1, NULL, 16);
}

/* NMEA lat: ddmm.mmmm  lon: dddmm.mmmm -> degrees * 1e7, signed by hemi. */
int32_t GpsUart::nmeaCoordE7(const char *field, char hemi) {
    if (!field[0]) return 0;
    double v = atof(field);
    int deg = (int)(v / 100.0);
    double minutes = v - deg * 100.0;
    double degrees = deg + minutes / 60.0;
    int32_t e7 = (int32_t)llround(degrees * 1e7);
    if (hemi == 'S' || hemi == 'W') e7 = -e7;
    return e7;
}

/* utc "hhmmss.ss", date "ddmmyy" -> unix epoch (days-from-civil). */
uint32_t GpsUart::rmcToUnix(const char *utc, const char *date) {
    if (strlen(utc) < 6 || strlen(date) < 6) return 0;
    int hh = (utc[0]-'0')*10 + (utc[1]-'0');
    int mi = (utc[2]-'0')*10 + (utc[3]-'0');
    int ss = (utc[4]-'0')*10 + (utc[5]-'0');
    int dd = (date[0]-'0')*10 + (date[1]-'0');
    int mo = (date[2]-'0')*10 + (date[3]-'0');
    int yy = 2000 + (date[4]-'0')*10 + (date[5]-'0');
    if (yy < 2024 || mo < 1 || mo > 12 || dd < 1 || dd > 31) return 0;
    /* Howard Hinnant days_from_civil */
    int y = yy - (mo <= 2);
    int era = y / 400;
    int yoe = y - era * 400;
    int doy = (153 * (mo + (mo > 2 ? -3 : 9)) + 2) / 5 + dd - 1;
    int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    long days = (long)era * 146097 + doe - 719468;
    return (uint32_t)(days * 86400L + hh * 3600 + mi * 60 + ss);
}

/* Split a validated NMEA sentence into fields (destructive). */
static int splitFields(char *s, char *fields[], int maxf) {
    int n = 0;
    char *p = s;
    while (n < maxf && p) {
        fields[n++] = p;
        p = strchr(p, ',');
        if (p) *p++ = '\0';
    }
    return n;
}

bool GpsUart::parseLine(const char *line, GpsFixResult &out) {
    if (!checksumOk(line)) return false;
    char buf[120];
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    char *star = strrchr(buf, '*');
    if (star) *star = '\0';
    char *f[16];
    int n = splitFields(buf, f, 16);
    if (n < 1 || strlen(f[0]) < 6) return false;
    const char *type = f[0] + 3;   /* skip $Gx */

    if (strncmp(type, "GGA", 3) == 0 && n >= 9) {
        _partial.sats = (uint8_t)atoi(f[7]);
        _partial.hdop_x10 = (uint16_t)(atof(f[8]) * 10);
        return false;
    }
    if (strncmp(type, "RMC", 3) == 0 && n >= 10) {
        if (f[2][0] != 'A') return false;    /* status: A = valid */
        out = _partial;                       /* carry sats/hdop from GGA */
        out.lat_e7 = nmeaCoordE7(f[3], f[4][0]);
        out.lon_e7 = nmeaCoordE7(f[5], f[6][0]);
        out.unix_time = rmcToUnix(f[1], f[9]);
        out.valid = (out.lat_e7 != 0 || out.lon_e7 != 0);
        return out.valid;
    }
    return false;
}

bool GpsUart::feedChar(char c, GpsFixResult &out) {
    if (c == '\r') return false;
    if (c == '\n') {
        _line[_len] = '\0';
        bool got = (_len > 10) && parseLine(_line, out);
        _len = 0;
        return got;
    }
    if (_len < sizeof(_line) - 1) _line[_len++] = c;
    else _len = 0;   /* garbage overflow: resync */
    return false;
}

GpsFixResult GpsUart::acquire(uint16_t timeout_s) {
    GpsFixResult fix;
    uint32_t deadline  = millis() + (uint32_t)timeout_s * 1000;
    uint32_t lastLog   = millis();
    uint32_t sentences = 0;   /* any NMEA sentence received, valid or not */

    while ((int32_t)(deadline - millis()) > 0) {
        while (gpsSerial.available()) {
            char c = (char)gpsSerial.read();
            if (c == '\n') sentences++;
            if (feedChar(c, fix)) {
                if (fix.unix_time > 0) {
                    struct timeval tv = { (time_t)fix.unix_time, 0 };
                    settimeofday(&tv, NULL);
                }
                Serial.printf("[gps] fix acquired in %lus — sats=%u\n",
                              (unsigned long)((millis() - (deadline - (uint32_t)timeout_s * 1000)) / 1000),
                              fix.sats);
                return fix;
            }
        }
        /* Progress heartbeat every 10 s so the user can see NMEA is flowing. */
        if (millis() - lastLog >= 10000) {
            uint32_t elapsed = (millis() - (deadline - (uint32_t)timeout_s * 1000)) / 1000;
            if (sentences > 0)
                Serial.printf("[gps] hunting... %lus elapsed, %lu sentences rx (no fix yet)\n",
                              (unsigned long)elapsed, (unsigned long)sentences);
            else
                Serial.printf("[gps] %lus elapsed — NO NMEA received (wiring/power issue?)\n",
                              (unsigned long)elapsed);
            lastLog = millis();
        }
        delay(5);
    }
    Serial.printf("[gps] timeout after %us — %lu sentences rx, no fix\n",
                  timeout_s, (unsigned long)sentences);
    return fix;   /* invalid — caller falls back to provisioned position */
}

void GpsUart::passthrough(Stream &out, uint16_t seconds) {
    uint32_t deadline = millis() + (uint32_t)seconds * 1000;
    while ((int32_t)(deadline - millis()) > 0) {
        while (gpsSerial.available()) out.write(gpsSerial.read());
        delay(2);
    }
}
