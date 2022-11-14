/*
Author: Pranav Bharadwaj
Date: 11/12/2022
Description: User-space application attempting to communicate with MPU 6050 accelerometer over I2C-2 bus present on the 
Beaglebone Black. The address and bus location of the sensor is fixed.
*/



#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <stdlib.h>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <linux/i2c-dev.h>
#include "i2c_utils.h"

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


void main()
{
    //Declare variables to hold acceleration values in X, Y and Z axes.
    int16_t accel_x, accel_y, accel_z;

    //Open accelerometer file
    int device_file, common_retval; 
    device_file = open(MPU6050_PATH, O_RDWR);
    printf("Attempting to open device file...\n");
    if(device_file < 0){        
        printf("Error occured while opening device file = %s. Exiting...\n", strerror(errno));
        exit(-1);
    }

    printf("Device file open.\n");

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

        //Read acceleration in Y-axis
        accel_y = (i2c_smbus_read_byte_data(device_file, ACCEL_YOUT_H) << 8) | (i2c_smbus_read_byte_data(device_file, ACCEL_YOUT_L));

        //Read acceleration in Z-axis
        accel_z = (i2c_smbus_read_byte_data(device_file, ACCEL_ZOUT_H) << 8) | (i2c_smbus_read_byte_data(device_file, ACCEL_ZOUT_L));

        printf("Acceleration values in 3-axes given below:\n");
        printf("X-axis acceleration = %d\tY-axis acceleration = %d\tZ-axis acceleration = %d\n", (int)accel_x, (int)accel_y, (int)accel_z);
        sleep(1); //Read values every second
    }

}