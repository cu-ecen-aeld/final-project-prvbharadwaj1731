/*
Author: Pranav Bharadwaj
Date: 11/12/2022
Description: User-space application attempting to communicate with MPU 6050 accelerometer over I2C-2 bus present on the 
Beaglebone Black. The address and bus location of the sensor is fixed.
*/

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>
#include <syslog.h>
#include <errno.h>

//for thingspeak
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h> 

#include <inttypes.h>
#include <string.h>
#include <math.h>

#include <linux/i2c-dev.h>
#include "i2c_utils.h"
#include <termios.h>

#include "gps_unified.h"
#include "ThingSpeakLinux.h"

#define MPU6050_PATH ("/dev/i2c-2")


//Sensor and register addresses
#define MPU6050_ADDR 0x68

#define PWR_MGMT1_ADDR  0x6B

#define ACCEL_XOUT_H    0x3B
#define ACCEL_XOUT_L    0x3C

#define ACCEL_YOUT_H    0x3D
#define ACCEL_YOUT_L    0x3E

#define ACCEL_ZOUT_H    0x3F
#define ACCEL_ZOUT_L    0x40

#define WHOMAI          0x75 //useful for debugging

char *WRITEAPI_KEY = "YH5AHTDY3OVH069F";


#define URL_THINGSPEAK             "api.thingspeak.com"
#define BEGIN_OF_HTTP_REQ          "GET /update?key="
#define END_OF_HTTP_REQ            "\r\n\r\n"
#define MAX_SIZE                   9999


//Globals
double x_axis_baseline_accel_g = -0.335; //global holding reference x-axis acceleration value in rest position
double crash_threshold_acceleration = 0.350; //Acceleration threshold for crash 


/*
Accelerometer specs:

z-axis of accelerometer is aligned with gravitional force direction. Monitorting x and y axes for sudden change in acceleration values would be sufficient to detect crash.
To normalize raw-values obtained from the accelerometer based on the sensor's sensitivity, calibration values are provided with the reference sheet.

MPU6050 Product Specification: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf

X-axis sensitivity factor = 2050
Y-axis sensitivity factor = 77
Z-axis sensitivity factor = 1947

Divide all values by 16384 (Max positive int16_t number) for normalized values
*/


/*ThingSpeak Code*/

//Function: Send some data to a ThingSpeak's channel
//Parameters:  - Number of fields (data to be sent)
//             - Array of integer values (all data that must be sent)
//             - ThingSpeaks's channel Key
//             - Size of ThingSpeak's channel key
//Return:      - 
int SendDataToThingSpeak(int FieldNo, float * FieldArray, char * Key, int SizeOfKey)
{
	int cleasockfd, n;
    struct sockaddr_in serv_addr;
    struct hostent *ServerTCP;
	int ReqStringSize;
	int i;
	char ReqString[MAX_SIZE];
	char BeginOfHTTPReq[]=BEGIN_OF_HTTP_REQ;
	char EndOfHTTPReq[]=END_OF_HTTP_REQ;
	char *ptReqString;
	
	if (FieldNo <=0){
		printf("PARAMS_ERROR is %d", PARAMS_ERROR);
		return PARAMS_ERROR;
	}

	if (FieldNo >= 1){
		printf("argument1 is %f\r\n", FieldArray[0]);
		printf("argument2 is %f\r\n", FieldArray[1]);
		printf("argument3 is %f\r\n", FieldArray[2]);
	}

    printf("setting up HTTP req\r\n");	
	
	//Setting up HTTP Req. string:
	bzero(&ReqString,sizeof(ReqString));
	sprintf(ReqString,"%s%s",BeginOfHTTPReq,Key);
	
	ptReqString = &ReqString[0]+(int)strlen(ReqString);
	for(i=1; i<= FieldNo; i++)
	{
		sprintf(ptReqString,"&field%d=%.2f",i,FieldArray[i-1]);
		ptReqString = &ReqString[0]+(int)strlen(ReqString);
	}
	
	sprintf(ptReqString,"%s",EndOfHTTPReq);
	//printf("%s",EndOfHTTPReq);
	//Connecting to ThingSpeak and sending data:
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
	//opening a socket
	if (sockfd < 0){
		printf("OPEN_SOCKET_ERROR is %d\r\n", OPEN_SOCKET_ERROR);
		return OPEN_SOCKET_ERROR;
	}

    printf("opening socket\r\n");
		
    bzero((char *) &serv_addr, sizeof(serv_addr));
    printf("bzero successful\r\n");
    serv_addr.sin_family = AF_INET;
	// Thingspeak IP : 184.106.153.149  statically assigned
	serv_addr.sin_addr.s_addr = inet_addr("184.106.153.149");
    printf("IP address assigned successfully\r\n");
    serv_addr.sin_port = htons(80);
    printf("port assigned successfully\r\n");
    
	//Step 4: connecting to ThingSpeak server (via HTTP port / port no. 80)
	if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0){
		printf("THINGSPEAK_CONNECTION_ERROR is %d", THINGSPEAK_CONNECTION_ERROR);
		return THINGSPEAK_CONNECTION_ERROR;
	}
	
    printf("Connected to ThingSpeak successfully\r\n");

	//sending data to ThingSpeak's channel
    write(sockfd,ReqString,strlen(ReqString));

    printf("Write to ThingSpeak channel successful\r\n");
		
	//close TCP connection
    close(sockfd);    
	
	//Complete
	printf("SEND_OK is %d", SEND_OK);
	return SEND_OK;
}





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





void main()
{
    //Declare variables to hold acceleration values in X, Y and Z axes.
    int16_t accel_x, accel_y, accel_z;
    int bytes_written; //track of number of bytes written to file
    double accel_x_change;
    double prev_accel_x = x_axis_baseline_accel_g;

    //Normalized acceleration values
    double g_x, g_y, g_z;

    //Array holding data to be sent to ThingSpeak
    float DataArray[3];

    //flag to set upon crash
    bool crash_flag = true; //default value is false

    gps_init();

    loc_t data;

    //Open accelerometer file
    int device_file, logfile, common_retval; 
    device_file = open(MPU6050_PATH, O_RDWR);
#ifdef VERBOSE_LOGGING
    printf("Attempting to open device file...\n");
#endif
    if(device_file < 0){        
        printf("Error occured while opening device file = %s. Exiting...\n", strerror(errno));
        exit(-1);
    }

#ifdef VERBOSE_LOGGING
    printf("Device file open.\n");
#endif

    //use ioctl to access physical device
    common_retval = ioctl(device_file, I2C_SLAVE, MPU6050_ADDR);
    if(common_retval < 0){
        printf("Error occured during ioctl call to get bus access to MPU6050. Exiting...\n");
        exit(-1);
    }

    //Wake up MPU6050 by writing 0 to PWR_MGMT1 register
    int8_t power_ret = i2c_smbus_read_byte_data(device_file, PWR_MGMT1_ADDR);
    i2c_smbus_write_byte_data(device_file, PWR_MGMT1_ADDR, ~(1 << 6)&power_ret);

    //In forever loop
    while(crash_flag){
        //Read acceleration in X-axis
        accel_x = (i2c_smbus_read_byte_data(device_file, ACCEL_XOUT_H) << 8) | (i2c_smbus_read_byte_data(device_file, ACCEL_XOUT_L));
        g_x = ((double)accel_x - 2050.0)/16384.0;

        //Read acceleration in Y-axis
        accel_y = (i2c_smbus_read_byte_data(device_file, ACCEL_YOUT_H) << 8) | (i2c_smbus_read_byte_data(device_file, ACCEL_YOUT_L));
        g_y = ((double)accel_y - 77.0)/16384.0;

        //Read acceleration in Z-axis
        accel_z = (i2c_smbus_read_byte_data(device_file, ACCEL_ZOUT_H) << 8) | (i2c_smbus_read_byte_data(device_file, ACCEL_ZOUT_L));
        g_z = ((double)accel_z - 1947.0)/16384.0;

#ifdef VERBOSE_LOGGING
        printf("g_x x-axis = %lf\n", g_x);
#endif

        accel_x_change = fabs(g_x - prev_accel_x);

#ifdef VERBOSE_LOGGING
        printf("Acceleration change = %lf\n", accel_x_change);
#endif

        prev_accel_x = g_x;

        if(accel_x_change >= crash_threshold_acceleration){            
            printf("Crash acceleration logged.\n");            
            
            //set crash flag
            crash_flag = false;
            
            gps_location(&data);
            printf("%lf \t%lf \t %lf\n",accel_x_change, data.latitude, data.longitude);

            DataArray[0] = (float)accel_x_change;
            DataArray[1] = (float)data.latitude;
            DataArray[2] = (float)data.longitude;

            
            //Sending data to ThingSpeak 
            //set a counter here to send data to cloud 3 times
            // for(int i=0;i<3;i++){
            //     SendDataToThingSpeak(4, &DataArray, WRITEAPI_KEY, sizeof(WRITEAPI_KEY));    
            // }  

            int return_code = SendDataToThingSpeak(3, &DataArray[0], WRITEAPI_KEY, sizeof(WRITEAPI_KEY));          
            printf("Return code from RESTful write = %s.\n", return_code);
        }

        sleep(0.5); //Read values every half second
    }

}