/*
 * GPS sensor userspace code implementation
 * Reference: https://github.com/nghiaphamsg/BBB-linux-app/tree/master/41_UART_GPS
 *            https://www.tigoe.com/pcomp/code/Processing/127/#:
 * Modified by: Ganesh KM
 * using libgps : https://github.com/wdalmut/libgps.git
 */

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <syslog.h>
#include <errno.h>
#include "gps_unified.h"

int uart0_filestream = -1;

void serial_init(void)
{
    uart0_filestream = open(PORTNAME, O_RDWR | O_NOCTTY | O_NONBLOCK);// NOCTTY: not become the process's controlling terminal

    if (uart0_filestream == -1)
    {
    	syslog(LOG_ERR, "failed to open uart0:%d\n!!!", errno);
    }
}

void serial_config(void)
{
    struct termios config;

    tcgetattr(uart0_filestream, &config);

    //control mode :Baudrate| characacter size|ignore modem controllines| receiver mode enable
    config.c_cflag = B9600 | CS8 | CLOCAL | CREAD; 
    config.c_iflag = IGNPAR; //input mode - ignore framing and parity errors
    config.c_oflag = 0;//output mode
    config.c_lflag = 0; //local mode

    tcflush(uart0_filestream, TCIFLUSH);

    tcsetattr(uart0_filestream, TCSANOW, &config); //TCSANOW-change occurs immediately
}

// void serial_println(const char *line, int len)
// {
//     if (uart0_filestream != -1)
//     {
//         char *cpstr = (char *)malloc((len+1) * sizeof(char));
//         strcpy(cpstr, line);
//         cpstr[len-1] = '\r';
//         cpstr[len] = '\n';

//         int count = write(uart0_filestream, cpstr, len+1);
//         if (count < 0) {
//             //TODO: handle errors...
//         }
//         free(cpstr);
//     }
// }

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
        } 
        else 
        {
            if (c == '\n') 
            {
                *b++ = '\0';//Null
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

// $GPGGA,235317.000,4003.9039,N,10512.5793,W,1,08,1.6,1577.9,M,-20.7,M,,0000*5F
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
// $GPRMC,235316.000,A,4003.9040,N,10512.5792,W,0.09,144.75,141112,,*19
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
    if ((checksum = nmea_valid_checksum(message)) != _EMPTY)
    {
        return checksum;
    }
    // parse the message for NMEA_GPGGA
    if (strstr(message, NMEA_GPGGA_STR) != NULL)
    {
        return NMEA_GPGGA;
    }
    // Parse the message for NMEA_GPRMC
    if (strstr(message, NMEA_GPRMC_STR) != NULL) 
    {
        return NMEA_GPRMC;
    }

    return NMEA_UNKNOWN;
}

uint8_t nmea_valid_checksum(const char *message)
{
    // for checksum calculation refer * in the gps output which is in hex.
    uint8_t checksum= (uint8_t)strtol(strchr(message, '*')+1, NULL, 16);
    char p;
    uint8_t sum = 0;
    ++message;
    // XOR of all bytes between $ and * gives the checksum
    while ((p = *message++) != '*') 
    {
        sum ^= p;
    }
    if (sum != checksum) 
    {
        return NMEA_CHECKSUM_ERR;
    }

    return _EMPTY;
}

void gps_init(void) {
    serial_init();
    serial_config();
}

// Compute the GPS location using decimal scale
void gps_location(loc_t *coord) 
{
    uint8_t status = _EMPTY;
    while(status != _COMPLETED)
    {
        gpgga_t gpgga;
        gprmc_t gprmc;
        char buffer[256];

        serial_readln(buffer, 256);

        switch (nmea_get_message_type(buffer))
        {
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

void gps_off(void) {
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
int main(void)
{
    gps_init();
    loc_t data;
    while (1) 
    {
        gps_location(&data);
        printf("Location of the Crash: Latitude: %lf Longitude: %lf\n", data.latitude, data.longitude);
    }
    return EXIT_SUCCESS;
}