/*
Author: Pranav Bharadwaj
Date: 11/12/2022
Description: User-space application attempting to communicate with MPU 6050 accelerometer over I2C-2 bus present on the 
Beaglebone Black. The address and bus location of the sensor is fixed.
*/

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <stdlib.h>
#include <math.h>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <signal.h>

#include <linux/i2c-dev.h>
#include "i2c_utils.h"

#define MPU6050_PATH ("/dev/i2c-2")
#define LOGFILE_PATH ("/var/tmp/accel_data")


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

//Signal handler for crash detection and handling
void signal_handler(int sig)
{
    if(sig == SIGUSR1){
        printf("*****CAUTION*****\n");
        printf("Crash detected. Please take further action.\n");
    }
}

void main()
{
    //Declare variables to hold acceleration values in X, Y and Z axes.
    int16_t accel_x, accel_y, accel_z;
    int bytes_written; //track of number of bytes written to file
    double accel_x_change;
    double prev_accel_x = x_axis_baseline_accel_g;

    //Normalized acceleration values
    float g_x, g_y, g_z;

    //roll and pitch angles
    float roll, pitch;

    //pointer to write data into file
    char *write_ptr;

    //Open accelerometer file
    int device_file, logfile, common_retval; 
    device_file = open(MPU6050_PATH, O_RDWR);
    printf("Attempting to open device file...\n");
    if(device_file < 0){        
        printf("Error occured while opening device file = %s. Exiting...\n", strerror(errno));
        exit(-1);
    }

    printf("Device file open.\n");

    //Open logfile
    logfile = open(LOGFILE_PATH, O_CREAT | O_RDWR | O_TRUNC, 0777);
    if(logfile == -1){
        printf("Error occured while opening logfile = %s. Exiting...\n", strerror(errno));
        exit(-1);
    }

    printf("Logfile open.\n");

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
    while(1){
        //Read acceleration in X-axis
        accel_x = (i2c_smbus_read_byte_data(device_file, ACCEL_XOUT_H) << 8) | (i2c_smbus_read_byte_data(device_file, ACCEL_XOUT_L));
        g_x = ((double)accel_x - 2050.0)/16384.0;

        //Read acceleration in Y-axis
        accel_y = (i2c_smbus_read_byte_data(device_file, ACCEL_YOUT_H) << 8) | (i2c_smbus_read_byte_data(device_file, ACCEL_YOUT_L));
        g_y = ((double)accel_y - 77.0)/16384.0;

        //Read acceleration in Z-axis
        accel_z = (i2c_smbus_read_byte_data(device_file, ACCEL_ZOUT_H) << 8) | (i2c_smbus_read_byte_data(device_file, ACCEL_ZOUT_L));
        g_z = ((double)accel_z - 1947.0)/16384.0;


        //Calculating roll and pitch angles using accelerometer data
        // roll = (atan(accel_y / sqrt(pow(accel_x, 2) + pow(accel_z, 2)))*180/M_PI) - 0.58;
        // pitch = (atan(-1 * accel_x / sqrt(pow(accel_y, 2) + pow(accel_z, 2))) * 180/M_PI) + 1.58;


        printf("g_x x-axis = %lf\n", g_x);

        accel_x_change = fabs(g_x - prev_accel_x);

        // printf("x-axis acceleration change = %lf\n\n", accel_x_change);

        // printf("Roll = %lf degrees\n Pitch = %lf degrees\n\n", roll, pitch);

        prev_accel_x = g_x;

        if(accel_x_change >= crash_threshold_acceleration){
            *write_ptr = (char)g_x; //last recorded acceleration
            bytes_written = write(logfile, write_ptr, sizeof(g_x));
            if(bytes_written == -1){
                printf("Error occured writing acceleration values to logfile.\n");
                exit(-1);
            }
            printf("Crash acceleration logged.\n");
        }

        // printf("Normalized accceleration values in 3-axes given below:\n");
        // printf("X-axis = %lf\nY-axis = %lf\nZ-axis = %lf\n\n", g_x, g_y, g_z);

        // printf("Acceleration change in x-axis = %lf\n", accel_x_change);

        // printf("Acceleration change in 3-axes given below:\n");
        // printf("X-axis change = %d\nY-axis change = %d\nZ-axis change = %d\n\n", accel_x_change, accel_y_change, accel_z_change);

        sleep(1); //Read values every second
    }

}