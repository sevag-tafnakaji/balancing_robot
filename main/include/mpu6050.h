#ifndef MPU6050_H
#define MPU6050_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "driver/i2c.h"

static const char *mpu6050_tag = "MPU6050";

int mean_ax = 0, mean_ay = 0, mean_az = 0, mean_gx = 0, mean_gy = 0, mean_gz = 0;
int accel_x_offset = 0, accel_y_offset = 0, accel_z_offset = 0, gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0;

#define I2C_EXAMPLE_MASTER_SCL_IO 5         /*!< gpio number for I2C master clock, D1 */
#define I2C_EXAMPLE_MASTER_SDA_IO 4         /*!< gpio number for I2C master data, D2  */
#define I2C_EXAMPLE_MASTER_NUM I2C_NUM_0    /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE 0 /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE 0 /*!< I2C master do not need buffer */

#define MPU6050_SENSOR_ADDR 0x68   /*!< slave address for MPU6050 sensor */
#define MPU6050_CMD_START 0x41     /*!< Command to set measure mode */
#define MPU6050_WHO_AM_I 0x75      /*!< Command to read WHO_AM_I reg */
#define WRITE_BIT I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ   /*!< I2C master read */
#define ACK_CHECK_EN 0x1           /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0          /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                /*!< I2C ack value */
#define NACK_VAL 0x1               /*!< I2C nack value */
#define LAST_NACK_VAL 0x2          /*!< I2C last_nack value */
#define CONFIG_SEL_BIT 0x18        /*!< Bits that set scale values for accelerometer and gyroscope>*/

#define G_VALUE 9.80665f
// Values used during calibration of MPU6050
#define CALIBRATION_BUFFER_SIZE 100
#define ACCELERATION_DEADZONE 8
#define GYROSCOPE_DEADZONE 1
/**
 * Define the mpu6050 register address:
 */
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
#define PWR_MGMT_1 0x6B
#define WHO_AM_I 0x75 /*!< Command to read WHO_AM_I reg */

#endif // MPU6050_H
