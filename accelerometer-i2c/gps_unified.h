#ifndef _GPS_UNIFIED_H_
#define _GPS_UNIFIED_H_


#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>

#ifndef PORTNAME
#define PORTNAME "/dev/ttyAMA0"
#endif

void serial_init(void);
void serial_config(void);
void serial_println(const char *, int);
void serial_readln(char *, int);
void serial_close(void);
#define _EMPTY 0x00
#define NMEA_GPRMC 0x01
#define NMEA_GPRMC_STR "$GPRMC"
#define NMEA_GPGGA 0x02
#define NMEA_GPGGA_STR "$GPGGA"
#define NMEA_UNKNOWN 0x00
#define _COMPLETED 0x03

#define NMEA_CHECKSUM_ERR 0x80
#define NMEA_MESSAGE_ERR 0xC0

struct gpgga {
    // Latitude eg: 4124.8963 (XXYY.ZZKK.. DEG, MIN, SEC.SS)
    double latitude;
    // Latitude eg: N
    char lat;
    // Longitude eg: 08151.6838 (XXXYY.ZZKK.. DEG, MIN, SEC.SS)
    double longitude;
    // Longitude eg: W
    char lon;
    // Quality 0, 1, 2
    uint8_t quality;
    // Number of satellites: 1,2,3,4,5...
    uint8_t satellites;
    // Altitude eg: 280.2 (Meters above mean sea level)
    double altitude;
};
typedef struct gpgga gpgga_t;

struct gprmc {
    double latitude;
    char lat;
    double longitude;
    char lon;
    double speed;
    double course;
};
typedef struct gprmc gprmc_t;

uint8_t nmea_get_message_type(const char *);
uint8_t nmea_valid_checksum(const char *);
void nmea_parse_gpgga(char *, gpgga_t *);
void nmea_parse_gprmc(char *, gprmc_t *);

struct location {
    double latitude;
    double longitude;
    double speed;
    double altitude;
    double course;
};
typedef struct location loc_t;

// Initialize device
extern void gps_init(void);
// Activate device
extern void gps_on(void);
// Get the actual location
extern void gps_location(loc_t *);


// Turn off device (low-power consumption)
extern void gps_off(void);

// -------------------------------------------------------------------------
// Internal functions
// -------------------------------------------------------------------------

// convert deg to decimal deg latitude, (N/S), longitude, (W/E)
void gps_convert_deg_to_dec(double *, char, double *, char);
double gps_deg_dec(double);

#endif