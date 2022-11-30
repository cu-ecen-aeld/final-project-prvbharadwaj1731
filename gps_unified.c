#include "gps_unified.h"
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <math.h>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>




int uart0_filestream = -1;

void serial_init(void)
{
    uart0_filestream = open(PORTNAME, O_RDWR | O_NOCTTY | O_NDELAY);

    if (uart0_filestream == -1)
    {
        //TODO error handling...
    }
}

void serial_config(void)
{
    struct termios options;
    tcgetattr(uart0_filestream, &options);
    options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(uart0_filestream, TCIFLUSH);
    tcsetattr(uart0_filestream, TCSANOW, &options);
}

void serial_println(const char *line, int len)
{
    if (uart0_filestream != -1) {
        char *cpstr = (char *)malloc((len+1) * sizeof(char));
        strcpy(cpstr, line);
        cpstr[len-1] = '\r';
        cpstr[len] = '\n';

        int count = write(uart0_filestream, cpstr, len+1);
        if (count < 0) {
            //TODO: handle errors...
        }
        free(cpstr);
    }
}

// Read a line from UART.
// Return a 0 len string in case of problems with UART
void serial_readln(char *buffer, int len)
{
    char c;
    char *b = buffer;
    int rx_length = -1;
    while(1) {
        rx_length = read(uart0_filestream, (void*)(&c), 1);

        if (rx_length <= 0) {
            //wait for messages
            sleep(1);
        } else {
            if (c == '\n') {
                *b++ = '\0';
                break;
            }
            *b++ = c;
        }
    }
}

void serial_close(void)
{
    close(uart0_filestream);
}
void nmea_parse_gpgga(char *nmea, gpgga_t *loc)
{
    char *p = nmea;

    p = strchr(p, ',')+1; //skip time

    p = strchr(p, ',')+1;
    loc->latitude = atof(p);

    p = strchr(p, ',')+1;
    switch (p[0]) {
        case 'N':
            loc->lat = 'N';
            break;
        case 'S':
            loc->lat = 'S';
            break;
        case ',':
            loc->lat = '\0';
            break;
    }

    p = strchr(p, ',')+1;
    loc->longitude = atof(p);

    p = strchr(p, ',')+1;
    switch (p[0]) {
        case 'W':
            loc->lon = 'W';
            break;
        case 'E':
            loc->lon = 'E';
            break;
        case ',':
            loc->lon = '\0';
            break;
    }

    p = strchr(p, ',')+1;
    loc->quality = (uint8_t)atoi(p);

    p = strchr(p, ',')+1;
    loc->satellites = (uint8_t)atoi(p);

    p = strchr(p, ',')+1;

    p = strchr(p, ',')+1;
    loc->altitude = atof(p);
}

void nmea_parse_gprmc(char *nmea, gprmc_t *loc)
{
    char *p = nmea;

    p = strchr(p, ',')+1; //skip time
    p = strchr(p, ',')+1; //skip status

    p = strchr(p, ',')+1;
    loc->latitude = atof(p);

    p = strchr(p, ',')+1;
    switch (p[0]) {
        case 'N':
            loc->lat = 'N';
            break;
        case 'S':
            loc->lat = 'S';
            break;
        case ',':
            loc->lat = '\0';
            break;
    }

    p = strchr(p, ',')+1;
    loc->longitude = atof(p);

    p = strchr(p, ',')+1;
    switch (p[0]) {
        case 'W':
            loc->lon = 'W';
            break;
        case 'E':
            loc->lon = 'E';
            break;
        case ',':
            loc->lon = '\0';
            break;
    }

    p = strchr(p, ',')+1;
    loc->speed = atof(p);

    p = strchr(p, ',')+1;
    loc->course = atof(p);
}

/**
 * Get the message type (GPGGA, GPRMC, etc..)
 *
 * This function filters out also wrong packages (invalid checksum)
 *
 * @param message The NMEA message
 * @return The type of message if it is valid
 */
uint8_t nmea_get_message_type(const char *message)
{
    uint8_t checksum = 0;
    if ((checksum = nmea_valid_checksum(message)) != _EMPTY) {
        return checksum;
    }

    if (strstr(message, NMEA_GPGGA_STR) != NULL) {
        return NMEA_GPGGA;
    }

    if (strstr(message, NMEA_GPRMC_STR) != NULL) {
        return NMEA_GPRMC;
    }

    return NMEA_UNKNOWN;
}

uint8_t nmea_valid_checksum(const char *message) {
    uint8_t checksum= (uint8_t)strtol(strchr(message, '*')+1, NULL, 16);

    char p;
    uint8_t sum = 0;
    ++message;
    while ((p = *message++) != '*') {
        sum ^= p;
    }

    if (sum != checksum) {
        return NMEA_CHECKSUM_ERR;
    }

    return _EMPTY;
}

extern void gps_init(void) {
    serial_init();
    serial_config();

    //Write commands
}

extern void gps_on(void) {
    //Write on
}

// Compute the GPS location using decimal scale
extern void gps_location(loc_t *coord) {
    uint8_t status = _EMPTY;
    while(status != _COMPLETED) {
        gpgga_t gpgga;
        gprmc_t gprmc;
        char buffer[256];

        serial_readln(buffer, 256);
        switch (nmea_get_message_type(buffer)) {
            case NMEA_GPGGA:
                nmea_parse_gpgga(buffer, &gpgga);

                gps_convert_deg_to_dec(&(gpgga.latitude), gpgga.lat, &(gpgga.longitude), gpgga.lon);

                coord->latitude = gpgga.latitude;
                coord->longitude = gpgga.longitude;
                coord->altitude = gpgga.altitude;

                status |= NMEA_GPGGA;
                break;
            case NMEA_GPRMC:
                nmea_parse_gprmc(buffer, &gprmc);

                coord->speed = gprmc.speed;
                coord->course = gprmc.course;

                status |= NMEA_GPRMC;
                break;
        }
    }
}

extern void gps_off(void) {
    //Write off
    serial_close();
}

// Convert lat e lon to decimals (from deg)
void gps_convert_deg_to_dec(double *latitude, char ns,  double *longitude, char we)
{
    double lat = (ns == 'N') ? *latitude : -1 * (*latitude);
    double lon = (we == 'E') ? *longitude : -1 * (*longitude);

    *latitude = gps_deg_dec(lat);
    *longitude = gps_deg_dec(lon);
}

double gps_deg_dec(double deg_point)
{
    double ddeg;
    double sec = modf(deg_point, &ddeg)*60;
    int deg = (int)(ddeg/100);
    int min = (int)(deg_point-(deg*100));

    double absdlat = round(deg * 1000000.);
    double absmlat = round(min * 1000000.);
    double absslat = round(sec * 1000000.);

    return round(absdlat + (absmlat/60) + (absslat/3600)) /1000000;
}
// int main(void) {
//     // Open
//     gps_init();

//     loc_t data;

//     while (1) {
//         gps_location(&data);

//         printf("%lf %lf\n", data.latitude, data.longitude);
//     }

//     return EXIT_SUCCESS;
// }
