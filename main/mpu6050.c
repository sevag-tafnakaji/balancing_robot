#include "mpu6050.h"

/**
 *
 * - read external MPU6050 sensor.
 * - Use one I2C port(master mode) to read or write the other I2C port(slave
 * mode) on one ESP8266 chip.
 *
 * Pin assignment:
 *
 * - master:
 *    GPIO4 is assigned as the data signal of i2c master port
 *    GPIO5 is assigned as the clock signal of i2c master port
 *
 * Connection:
 *
 * - connect sda/scl of sensor with GPIO4/GPIO5
 * - no need to add external pull-up resistors, driver will enable internal
 * pull-up resistors.
 *
 * Test items:
 *
 * - read the sensor data, if connected.
 */

/**
 * @brief i2c master initialization
 */
esp_err_t i2c_init() {
  int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
  conf.sda_pullup_en = 1;
  conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
  conf.scl_pullup_en = 1;
  conf.clk_stretch_tick =
      300; // 300 ticks, Clock stretch is about 210us, you can make changes
           // according to the actual situation.
  ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode));
  ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
  return ESP_OK;
}

/**
 * @brief code to write to mpu6050
 *
 * 1. send data
 * ___________________________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write reg_address + ack | write
 * data_len byte + ack  | stop |
 * --------|---------------------------|-------------------------|----------------------------|------|
 *
 * @param i2c_num I2C port number
 * @param reg_address slave reg address
 * @param data data to send
 * @param data_len data length
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
esp_err_t mpu6050_write(i2c_port_t i2c_num, uint8_t reg_address, uint8_t *data,
                        size_t data_len) {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, MPU6050_SENSOR_ADDR << 1 | WRITE_BIT,
                        ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
  i2c_master_write(cmd, data, data_len, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  return ret;
}

/**
 * @brief code to read mpu6050
 *
 * 1. send reg address
 * ______________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write reg_address + ack | stop |
 * --------|---------------------------|-------------------------|------|
 *
 * 2. read data
 * ___________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | read data_len byte + ack(last nack)  |
 * stop |
 * --------|---------------------------|--------------------------------------|------|
 *
 * @param i2c_num I2C port number
 * @param reg_address slave reg address
 * @param data data to read
 * @param data_len data length
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
esp_err_t mpu6050_read(i2c_port_t i2c_num, uint8_t reg_address, uint8_t *data,
                       size_t data_len) {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, MPU6050_SENSOR_ADDR << 1 | WRITE_BIT,
                        ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  if (ret != ESP_OK) {
    return ret;
  }

  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, MPU6050_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
  i2c_master_read(cmd, data, data_len, LAST_NACK_VAL);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  return ret;
}

esp_err_t mpu6050_init(i2c_port_t i2c_num) {
  uint8_t cmd_data;
  vTaskDelay(100 / portTICK_RATE_MS);
  i2c_init();
  cmd_data = 0x00; // reset mpu6050
  ESP_ERROR_CHECK(mpu6050_write(i2c_num, PWR_MGMT_1, &cmd_data, 1));
  cmd_data = 0x07; // Set the SMPRT_DIV
  ESP_ERROR_CHECK(mpu6050_write(i2c_num, SMPLRT_DIV, &cmd_data, 1));
  cmd_data = 0x06; // Set the Low Pass Filter
  ESP_ERROR_CHECK(mpu6050_write(i2c_num, CONFIG, &cmd_data, 1));
  cmd_data = 0x18; // Set the GYRO range, 0x18 = pm 2000 deg/s
  ESP_ERROR_CHECK(mpu6050_write(i2c_num, GYRO_CONFIG, &cmd_data, 1));
  cmd_data = 0x00; // Set the ACCEL range, 0x00 = pm 2g
  ESP_ERROR_CHECK(mpu6050_write(i2c_num, ACCEL_CONFIG, &cmd_data, 1));
  return ESP_OK;
}

float acc_scale_value() {
  uint8_t accel_range_bits;
  float accel_scale = 0.0f;
  mpu6050_read(I2C_EXAMPLE_MASTER_NUM, ACCEL_CONFIG, &accel_range_bits, 1);

  accel_range_bits = accel_range_bits & CONFIG_SEL_BIT;
  ESP_LOGD(mpu6050_tag, "accel scale bits: %d", (uint16_t)accel_range_bits);
  switch (accel_range_bits) {
  case 0:
    accel_scale = 2.0f * G_VALUE; // 2 ^ 14 for pm 2g
    break;
  case CONFIG_SEL_BIT:
    accel_scale = 4.0f * G_VALUE; // 2 ^ 13 for pm 4g
    break;
  case CONFIG_SEL_BIT + 1:
    accel_scale = 8.0f * G_VALUE; // 2 ^ 12 for pm 8g
    break;
  case CONFIG_SEL_BIT + 2:
    accel_scale = 16.0f * G_VALUE; // 2 ^ 11 for pm 16g
    break;
  }

  return accel_scale;
}

float gyro_scale_value() {
  uint8_t gyro_range_bits;
  float gyro_scale = 0;
  mpu6050_read(I2C_EXAMPLE_MASTER_NUM, GYRO_CONFIG, &gyro_range_bits, 1);

  gyro_range_bits = gyro_range_bits & CONFIG_SEL_BIT;
  ESP_LOGD(mpu6050_tag, "Gyro scale bits: %d", (uint16_t)gyro_range_bits);
  switch (gyro_range_bits) {
  case 0:
    gyro_scale = 250.0f; // for 250 deg/s
    break;
  case CONFIG_SEL_BIT:
    gyro_scale = 500.0f; // for 500 deg/s
    break;
  case CONFIG_SEL_BIT + 1:
    gyro_scale = 1000.0f; // for 1000 deg/s
    break;
  case CONFIG_SEL_BIT + 2:
    gyro_scale = 2000.0f; // for 2000 deg/s
    break;
  }

  return gyro_scale;
}

void mean_measurements() {
  int i = 0;
  int ignore_first_n = 10;
  uint8_t sensor_data[14];
  long buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0,
       buff_gz = 0;

  while (i < (CALIBRATION_BUFFER_SIZE + ignore_first_n + 1)) {
    memset(sensor_data, 0, 14);
    ESP_ERROR_CHECK(
        mpu6050_read(I2C_EXAMPLE_MASTER_NUM, ACCEL_XOUT_H, sensor_data, 14));
    if (i > ignore_first_n && i <= (CALIBRATION_BUFFER_SIZE + ignore_first_n)) {
      buff_ax +=
          ((int16_t)(sensor_data[0] << 8) | sensor_data[1]) + accel_x_offset;
      buff_ay +=
          ((int16_t)(sensor_data[2] << 8) | sensor_data[3]) + accel_y_offset;
      buff_az +=
          ((int16_t)(sensor_data[4] << 8) | sensor_data[5]) + accel_z_offset;
      buff_gx +=
          ((int16_t)(sensor_data[8] << 8) | sensor_data[9]) + gyro_x_offset;
      buff_gy +=
          ((int16_t)(sensor_data[10] << 8) | sensor_data[11]) + gyro_y_offset;
      buff_gz +=
          ((int16_t)(sensor_data[12] << 8) | sensor_data[13]) + gyro_z_offset;
    }

    if (i == (CALIBRATION_BUFFER_SIZE + ignore_first_n)) {
      mean_ax = buff_ax / CALIBRATION_BUFFER_SIZE;
      mean_ay = buff_ay / CALIBRATION_BUFFER_SIZE;
      mean_az = buff_az / CALIBRATION_BUFFER_SIZE;
      mean_gx = buff_gx / CALIBRATION_BUFFER_SIZE;
      mean_gy = buff_gy / CALIBRATION_BUFFER_SIZE;
      mean_gz = buff_gz / CALIBRATION_BUFFER_SIZE;
    }
    i++;
    vTaskDelay(2 / portTICK_RATE_MS);
  }
}

void calibrate_mpu() {
  ESP_LOGI(mpu6050_tag, "Beginning Calibration of MPU6050 IMU. Please Wait...");
  accel_x_offset = -mean_ax / 8;
  accel_y_offset = -mean_ay / 8;
  accel_z_offset = (16384 - mean_az) / 8;
  gyro_x_offset = -mean_gx / 4;
  gyro_y_offset = -mean_gy / 4;
  gyro_z_offset = -mean_gz / 4;

  while (1) {
    int ready = 0;
    mean_measurements();

    ESP_LOGD(mpu6050_tag, "MEAN: %d, %d, %d, %d, %d, %d", mean_ax, mean_ay,
             16384 - mean_az, mean_gx, mean_gy, mean_gz);
    ESP_LOGD(mpu6050_tag, "OFFSET: %d, %d, %d, %d, %d, %d", accel_x_offset,
             accel_y_offset, accel_z_offset, gyro_x_offset, gyro_y_offset,
             gyro_z_offset);

    if (abs(mean_ax) <= ACCELERATION_DEADZONE)
      ready++;
    else
      accel_x_offset = accel_x_offset - mean_ax / ACCELERATION_DEADZONE;

    if (abs(mean_ay) <= ACCELERATION_DEADZONE)
      ready++;
    else
      accel_y_offset = accel_y_offset - mean_ay / ACCELERATION_DEADZONE;

    if (abs(16384 - mean_az) <= ACCELERATION_DEADZONE)
      ready++;
    else
      accel_z_offset =
          accel_z_offset + (16384 - mean_az) / ACCELERATION_DEADZONE;

    if (abs(mean_gx) <= GYROSCOPE_DEADZONE)
      ready++;
    else
      gyro_x_offset = gyro_x_offset - mean_gx / (GYROSCOPE_DEADZONE + 1);

    if (abs(mean_gy) <= GYROSCOPE_DEADZONE)
      ready++;
    else
      gyro_y_offset = gyro_y_offset - mean_gy / (GYROSCOPE_DEADZONE + 1);

    if (abs(mean_gz) <= GYROSCOPE_DEADZONE)
      ready++;
    else
      gyro_z_offset = gyro_z_offset - mean_gz / (GYROSCOPE_DEADZONE + 1);

    if (ready == 6)
      break;
  }
  ESP_LOGI(mpu6050_tag, "Calibration Finished");
}

/**
 * TODO:
 *  Expand this to use en EKF filter rather than raw measurements.
 *  Split functionalities into separate functions.
 *  Reduce magic number usage
 */

void mpu6050_task(void *arg) {
  uint8_t sensor_data[14];
  float acc_scale;
  float gyro_scale;
  double accel_x;
  double accel_y;
  double accel_z;

  double gyro_x;
  double gyro_y;
  double gyro_z;

  portTickType xLastWakeTime;

  mpu6050_init(I2C_EXAMPLE_MASTER_NUM);

  calibrate_mpu();

  xLastWakeTime = xTaskGetTickCount();

  while (1) {

    memset(sensor_data, 0, 14);

    if (mpu6050_read(I2C_EXAMPLE_MASTER_NUM, ACCEL_XOUT_H, sensor_data, 14) ==
        ESP_OK) {
      ESP_LOGD(mpu6050_tag, "*******************\n");

      acc_scale = acc_scale_value();
      gyro_scale = gyro_scale_value();

      accel_x =
          ((int16_t)((sensor_data[0] << 8) | sensor_data[1]) + accel_x_offset) /
          32768.0 * acc_scale;
      accel_y =
          ((int16_t)((sensor_data[2] << 8) | sensor_data[3]) + accel_y_offset) /
          32768.0 * acc_scale;
      accel_z =
          ((int16_t)((sensor_data[4] << 8) | sensor_data[5]) + accel_z_offset) /
          32768.0 * acc_scale;
      gyro_x =
          ((int16_t)((sensor_data[8] << 8) | sensor_data[9]) + gyro_x_offset) /
          32768.0 * gyro_scale;
      gyro_y = ((int16_t)((sensor_data[10] << 8) | sensor_data[11]) +
                gyro_y_offset) /
               32768.0 * gyro_scale;
      gyro_z = ((int16_t)((sensor_data[12] << 8) | sensor_data[13]) +
                gyro_z_offset) /
               32768.0 * gyro_scale;

      ESP_LOGD(mpu6050_tag, "Accel scale: %d.%d, Gyro scale: %d.%d",
               (int)acc_scale, (int)(acc_scale * 100) % 100, (int)gyro_scale,
               (int)(gyro_scale * 100) % 100);

      ESP_LOGD(mpu6050_tag, " accel_x: %d.%d, accel_y: %d.%d, accel_z: %d.%d",
               (int)accel_x, (int)(fabs(accel_x) * 100) % 100, (int)accel_y,
               (int)(fabs(accel_y) * 100) % 100, (int)accel_z,
               (int)(fabs(accel_z) * 100) % 100);
      ESP_LOGD(mpu6050_tag, " gyro_x: %d.%d, gyro_y: %d.%d, gyro_z: %d.%d",
               (int)gyro_x, (int)(fabs(gyro_x) * 100) % 100, (int)gyro_y,
               (int)(fabs(gyro_y) * 100) % 100, (int)gyro_z,
               (int)(fabs(gyro_z) * 100) % 100);
    } else {
      ESP_LOGE(mpu6050_tag, "No ack, sensor not connected...skip...\n");
    }

    vTaskDelayUntil(&xLastWakeTime, 20 / portTICK_RATE_MS);
  }

  i2c_driver_delete(I2C_EXAMPLE_MASTER_NUM);
  vTaskDelete(NULL);
}
