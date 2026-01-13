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
 *    GPIO4 (D2) is assigned as the data signal of i2c master port
 *    GPIO5 (D1) is assigned as the clock signal of i2c master port
 *
 * Connection:
 *
 * - connect sda/scl of sensor with GPIO4/GPIO5
 * - no need to add external pull-up resistors, driver will enable internal
 * pull-up resistors.
 *
 * TODO: Figure out why using the %f flag in logging and printing doesn't work.
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
      300;  // 300 ticks, Clock stretch is about 210us, you can make changes
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
esp_err_t mpu6050_write(i2c_port_t i2c_num, uint8_t reg_address, uint8_t* data,
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
esp_err_t mpu6050_read(i2c_port_t i2c_num, uint8_t reg_address, uint8_t* data,
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
  cmd_data = 0x00;  // reset mpu6050
  ESP_ERROR_CHECK(mpu6050_write(i2c_num, PWR_MGMT_1, &cmd_data, 1));
  cmd_data = 0x07;  // Set the SMPRT_DIV
  ESP_ERROR_CHECK(mpu6050_write(i2c_num, SMPLRT_DIV, &cmd_data, 1));
  cmd_data = 0x06;  // Set the Low Pass Filter
  ESP_ERROR_CHECK(mpu6050_write(i2c_num, CONFIG, &cmd_data, 1));
  cmd_data = 0x18;  // Set the GYRO range, 0x18 = pm 2000 deg/s
  ESP_ERROR_CHECK(mpu6050_write(i2c_num, GYRO_CONFIG, &cmd_data, 1));
  cmd_data = 0x00;  // Set the ACCEL range, 0x00 = pm 2g
  ESP_ERROR_CHECK(mpu6050_write(i2c_num, ACCEL_CONFIG, &cmd_data, 1));
  return ESP_OK;
}

void acc_scale_value(sensorConfig_t* config) {
  uint8_t accel_range_bits;
  mpu6050_read(I2C_EXAMPLE_MASTER_NUM, ACCEL_CONFIG, &accel_range_bits, 1);

  accel_range_bits = accel_range_bits & CONFIG_SEL_BIT;
  ESP_LOGD(mpu6050_tag, "accel scale bits: %d", (uint16_t)accel_range_bits);
  switch (accel_range_bits) {
    case 0:
      config->scale_accel = 2.0f * G_VALUE;  // 2 ^ 14 for pm 2g
      break;
    case CONFIG_SEL_BIT:
      config->scale_accel = 4.0f * G_VALUE;  // 2 ^ 13 for pm 4g
      break;
    case CONFIG_SEL_BIT + 1:
      config->scale_accel = 8.0f * G_VALUE;  // 2 ^ 12 for pm 8g
      break;
    case CONFIG_SEL_BIT + 2:
      config->scale_accel = 16.0f * G_VALUE;  // 2 ^ 11 for pm 16g
      break;
  }
}

void gyro_scale_value(sensorConfig_t* config) {
  uint8_t gyro_range_bits;
  mpu6050_read(I2C_EXAMPLE_MASTER_NUM, GYRO_CONFIG, &gyro_range_bits, 1);

  gyro_range_bits = gyro_range_bits & CONFIG_SEL_BIT;
  ESP_LOGD(mpu6050_tag, "Gyro scale bits: %d", (uint16_t)gyro_range_bits);
  switch (gyro_range_bits) {
    case 0:
      config->scale_gyro = 250.0f;  // for 250 deg/s
      break;
    case CONFIG_SEL_BIT:
      config->scale_gyro = 500.0f;  // for 500 deg/s
      break;
    case CONFIG_SEL_BIT + 1:
      config->scale_gyro = 1000.0f;  // for 1000 deg/s
      break;
    case CONFIG_SEL_BIT + 2:
      config->scale_gyro = 2000.0f;  // for 2000 deg/s
      break;
  }
}

void read_raw_values(sensorData_t* dest, sensorConfig_t* config, bool scale) {
  uint8_t sensor_data[14];
  memset(sensor_data, 0, 14);

  if (mpu6050_read(I2C_EXAMPLE_MASTER_NUM, ACCEL_XOUT_H, sensor_data, 14) !=
      ESP_OK) {
    ESP_LOGE(mpu6050_tag, "Failed when attempting to read raw values");
    return;  // exit function if failed to extract raw values
  }
  ESP_LOGD(mpu6050_tag, "Raw data: %d, %d, %d, %d, %d, %d",
           ((int16_t)((sensor_data[0] << 8) | sensor_data[1]) +
            config->offset_accel.x),
           ((int16_t)((sensor_data[2] << 8) | sensor_data[3]) +
            config->offset_accel.y),
           ((int16_t)((sensor_data[4] << 8) | sensor_data[5]) +
            config->offset_accel.z),
           ((int16_t)((sensor_data[8] << 8) | sensor_data[9]) +
            config->offset_gyro.x),
           ((int16_t)((sensor_data[10] << 8) | sensor_data[11]) +
            config->offset_gyro.y),
           ((int16_t)((sensor_data[12] << 8) | sensor_data[13]) +
            config->offset_gyro.z));

  dest->accel.x = ((int16_t)((sensor_data[0] << 8) | sensor_data[1]) +
                   config->offset_accel.x);
  dest->accel.y = ((int16_t)((sensor_data[2] << 8) | sensor_data[3]) +
                   config->offset_accel.y);
  dest->accel.z = ((int16_t)((sensor_data[4] << 8) | sensor_data[5]) +
                   config->offset_accel.z);
  dest->gyro.x = ((int16_t)((sensor_data[8] << 8) | sensor_data[9]) +
                  config->offset_gyro.x);
  dest->gyro.y = ((int16_t)((sensor_data[10] << 8) | sensor_data[11]) +
                  config->offset_gyro.y);
  dest->gyro.z = ((int16_t)((sensor_data[12] << 8) | sensor_data[13]) +
                  config->offset_gyro.z);

  ESP_LOGD(mpu6050_tag, "dest: %d.%d, %d.%d, %d.%d, %d.%d, %d.%d, %d.%d",
           (int)dest->accel.x, (int)(fabs(dest->accel.x) * 100) % 100,
           (int)dest->accel.y, (int)(fabs(dest->accel.y) * 100) % 100,
           (int)(16384 - dest->accel.z),
           (int)(fabs(16384 - dest->accel.z) * 100) % 100, (int)dest->gyro.x,
           (int)(fabs(dest->gyro.x) * 100) % 100, (int)dest->gyro.y,
           (int)(fabs(dest->gyro.y) * 100) % 100, (int)dest->gyro.z,
           (int)(fabs(dest->gyro.z) * 100) % 100);

  if (scale) {
    acc_scale_value(config);
    gyro_scale_value(config);

    dest->accel.x *= config->scale_accel / 32768.0f;
    dest->accel.y *= config->scale_accel / 32768.0f;
    dest->accel.z *= config->scale_accel / 32768.0f;
    dest->gyro.x *= config->scale_gyro / 32768.0f;
    dest->gyro.y *= config->scale_gyro / 32768.0f;
    dest->gyro.z *= config->scale_gyro / 32768.0f;
  }
}

void mean_measurements() {
  int i = 0;
  int ignore_first_n = 10;
  // Only works when buffer is long int rather than floats
  long buff_ax = 0;
  long buff_ay = 0;
  long buff_az = 0;
  long buff_gx = 0;
  long buff_gy = 0;
  long buff_gz = 0;
  sensorData_t current_reading;

  while (i < (CALIBRATION_BUFFER_SIZE + ignore_first_n + 1)) {
    read_raw_values(&current_reading, &mpu6050_config, false);
    if (i > ignore_first_n && i <= (CALIBRATION_BUFFER_SIZE + ignore_first_n)) {
      buff_ax += current_reading.accel.x;
      buff_ay += current_reading.accel.y;
      buff_az += current_reading.accel.z;
      buff_gx += current_reading.gyro.x;
      buff_gy += current_reading.gyro.y;
      buff_gz += current_reading.gyro.z;
    }

    if (i == (CALIBRATION_BUFFER_SIZE + ignore_first_n)) {
      mean_values.accel.x = buff_ax / CALIBRATION_BUFFER_SIZE;
      mean_values.accel.y = buff_ay / CALIBRATION_BUFFER_SIZE;
      mean_values.accel.z = buff_az / CALIBRATION_BUFFER_SIZE;
      mean_values.gyro.x = buff_gx / CALIBRATION_BUFFER_SIZE;
      mean_values.gyro.y = buff_gy / CALIBRATION_BUFFER_SIZE;
      mean_values.gyro.z = buff_gz / CALIBRATION_BUFFER_SIZE;
    }
    i++;
    vTaskDelay(2 / portTICK_RATE_MS);
  }
}

void calibrate_mpu() {
  ESP_LOGI(mpu6050_tag, "Beginning Calibration of MPU6050 IMU. Please Wait...");
  mpu6050_config.offset_accel.x = -mean_values.accel.x / 8;
  mpu6050_config.offset_accel.y = -mean_values.accel.y / 8;
  mpu6050_config.offset_accel.z = (16384 - mean_values.accel.z) / 8;
  mpu6050_config.offset_gyro.x = -mean_values.gyro.x / 4;
  mpu6050_config.offset_gyro.y = -mean_values.gyro.y / 4;
  mpu6050_config.offset_gyro.z = -mean_values.gyro.z / 4;

  while (1) {
    int ready = 0;
    mean_measurements();

    ESP_LOGD(
        mpu6050_tag, "MEAN: %d.%d, %d.%d, %d.%d, %d.%d, %d.%d, %d.%d",
        (int)mean_values.accel.x, (int)(fabs(mean_values.accel.x) * 100) % 100,
        (int)mean_values.accel.y, (int)(fabs(mean_values.accel.y) * 100) % 100,
        (int)(16384 - mean_values.accel.z),
        (int)(fabs(16384 - mean_values.accel.z) * 100) % 100,
        (int)mean_values.gyro.x, (int)(fabs(mean_values.gyro.x) * 100) % 100,
        (int)mean_values.gyro.y, (int)(fabs(mean_values.gyro.y) * 100) % 100,
        (int)mean_values.gyro.z, (int)(fabs(mean_values.gyro.z) * 100) % 100);
    ESP_LOGD(mpu6050_tag, "OFFSET: %d, %d, %d, %d, %d, %d",
             mpu6050_config.offset_accel.x, mpu6050_config.offset_accel.y,
             mpu6050_config.offset_accel.z, mpu6050_config.offset_gyro.x,
             mpu6050_config.offset_gyro.y, mpu6050_config.offset_gyro.z);

    //  If mean values are low enough/within expected values, sensor is
    //  calibrated otherwise update the offset value to minimize the mean.
    if (abs(mean_values.accel.x) <= ACCELERATION_DEADZONE)
      ready++;
    else
      mpu6050_config.offset_accel.x =
          mpu6050_config.offset_accel.x -
          mean_values.accel.x / ACCELERATION_DEADZONE;

    if (abs(mean_values.accel.y) <= ACCELERATION_DEADZONE)
      ready++;
    else
      mpu6050_config.offset_accel.y =
          mpu6050_config.offset_accel.y -
          mean_values.accel.y / ACCELERATION_DEADZONE;

    if (abs(16384 - mean_values.accel.z) <= ACCELERATION_DEADZONE)
      ready++;
    else
      mpu6050_config.offset_accel.z =
          mpu6050_config.offset_accel.z +
          (16384 - mean_values.accel.z) / ACCELERATION_DEADZONE;

    if (abs(mean_values.gyro.x) <= GYROSCOPE_DEADZONE)
      ready++;
    else
      mpu6050_config.offset_gyro.x =
          mpu6050_config.offset_gyro.x -
          mean_values.gyro.x / (GYROSCOPE_DEADZONE + 1);

    if (abs(mean_values.gyro.y) <= GYROSCOPE_DEADZONE)
      ready++;
    else
      mpu6050_config.offset_gyro.y =
          mpu6050_config.offset_gyro.y -
          mean_values.gyro.y / (GYROSCOPE_DEADZONE + 1);

    if (abs(mean_values.gyro.z) <= GYROSCOPE_DEADZONE)
      ready++;
    else
      mpu6050_config.offset_gyro.z =
          mpu6050_config.offset_gyro.z -
          mean_values.gyro.z / (GYROSCOPE_DEADZONE + 1);

    if (ready == 6) break;
  }
  ESP_LOGI(mpu6050_tag, "Calibration Finished");
}

void mpu6050_task(void* arg) {
  portTickType xLastWakeTime;

  mpu6050_init(I2C_EXAMPLE_MASTER_NUM);

  calibrate_mpu();

  calibration_finished = true;

  xLastWakeTime = xTaskGetTickCount();

  while (1) {
    xSemaphoreTake(sensors_sem, portMAX_DELAY);
    read_raw_values(&raw_sensor_values, &mpu6050_config, true);
    xSemaphoreGive(sensors_sem);

    ESP_LOGD(mpu6050_tag, "Accel scale: %d.%d, Gyro scale: %d.%d",
             (int)mpu6050_config.scale_accel,
             (int)(fabs(mpu6050_config.scale_accel) * 100) % 100,
             (int)mpu6050_config.scale_gyro,
             (int)(fabs(mpu6050_config.scale_gyro) * 100) % 100);

    ESP_LOGD(mpu6050_tag,
             " accel_x: %d.%d, accel_y: %d.%d, accel_z: %d.%d, gyro_x: %d.%d, "
             "gyro_y: %d.%d, gyro_z: %d.%d",
             (int)raw_sensor_values.accel.x,
             (int)(fabs(raw_sensor_values.accel.x) * 100) % 100,
             (int)raw_sensor_values.accel.y,
             (int)(fabs(raw_sensor_values.accel.y) * 100) % 100,
             (int)(raw_sensor_values.accel.z),
             (int)(fabs(raw_sensor_values.accel.z) * 100) % 100,
             (int)raw_sensor_values.gyro.x,
             (int)(fabs(raw_sensor_values.gyro.x) * 100) % 100,
             (int)raw_sensor_values.gyro.y,
             (int)(fabs(raw_sensor_values.gyro.y) * 100) % 100,
             (int)raw_sensor_values.gyro.z,
             (int)(fabs(raw_sensor_values.gyro.z) * 100) % 100);

    vTaskDelayUntil(&xLastWakeTime, xSensorFrequency);
  }

  i2c_driver_delete(I2C_EXAMPLE_MASTER_NUM);
  vTaskDelete(NULL);
}
