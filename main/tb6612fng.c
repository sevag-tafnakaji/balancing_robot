
#include "tb6612fng.h"

static const char* tb6612fng_tag = "TB6612FNG";

// TODO: Implement state machine for direction transition and better duty cycle
// changing
esp_err_t read_from_motor_queue(motor_torque_t* dest) {
  /**
   * Return:
   *  ESP_OK in case writing to queue was successful
   *  ESP_ in case queue is full or timeout occured
   *
   */

  BaseType_t response =
      xQueueReceive(motor_torque_queue, dest, xMotorQueueRecieveBlockTime);

  if (response == pdTRUE)
    return ESP_OK;
  else
    return ESP_ERR_NO_MEM;
}

void set_motor_mode_A(int mode) {
  ESP_LOGD(tb6612fng_tag, "Setting motor A to mode %d", mode);
  //   ESP_ERROR_CHECK(gpio_set_level(A_IN_1, (uint32_t)(mode & 0x1)));
  //   ESP_ERROR_CHECK(gpio_set_level(A_IN_2, (uint32_t)(mode & 0x10)));
  curr_mode_A = mode;
  ESP_LOGD(tb6612fng_tag, "current mode for motor A %d", curr_mode_A);
}

void set_motor_mode_B(int mode) {
  ESP_LOGD(tb6612fng_tag, "Setting motor B to mode %d", mode);
  //   ESP_ERROR_CHECK(gpio_set_level(B_IN_1, (uint32_t)(mode & 0x1)));
  //   ESP_ERROR_CHECK(gpio_set_level(B_IN_2, (uint32_t)(mode & 0x10)));
  curr_mode_B = mode;
  ESP_LOGD(tb6612fng_tag, "current mode for motor B %d", curr_mode_B);
}

void set_motor_mode(int mode, char motor) {
  if (motor == 'A')
    set_motor_mode_A(mode);
  else if (motor == 'B')
    set_motor_mode_B(mode);
}

void set_both_motor_modes(int mode) {
  set_motor_mode(mode, 'A');
  set_motor_mode(mode, 'B');
}

void set_motor_duties(float duty_A, float duty_B) {
  uint32_t ui_duties[4] = {0, 0, 0, 0};
  update_motor_A_duty(duty_A, ui_duties);
  update_motor_B_duty(duty_B, ui_duties);

  // ESP_LOGI(tb6612fng_tag, "motor A mode: %d, motor B mode: %d", curr_mode_A,
  //          curr_mode_B);
  // ESP_LOGI(tb6612fng_tag, "motor A duties: [%d, %d], motor B duties: [%d,
  // %d]",
  //          ui_duties[0], ui_duties[1], ui_duties[2], ui_duties[3]);
  ESP_ERROR_CHECK(pwm_set_duties(ui_duties));
  ESP_ERROR_CHECK(pwm_start());
}

void update_motor_A_duty(float duty_A, uint32_t* duties) {
  if (fabsf(duty_A) < MINIMUM_DUTY) {
    duty_A = 0.0f;
  }

  if (duty_A > 0) {
    set_motor_mode_A(MOTOR_CCW);
  } else if (duty_A < 0) {
    set_motor_mode_A(MOTOR_CW);
  } else {
    set_motor_mode_A(MOTOR_SHORT_BREAK);
  }

  duty_A = fabs(duty_A);

  if (curr_mode_A != MOTOR_SHORT_BREAK) {
    int a_idx = (curr_mode_A == MOTOR_CW) ? 0 : 1;

    duties[a_idx] = PWM_PERIOD * duty_A;
  }

  // ESP_LOGI(tb6612fng_tag, "motor A mode: %d", curr_mode_A);
  // ESP_LOGI(tb6612fng_tag, "motor A duty: %d", (int)(duty_A * 100));
}

void update_motor_B_duty(float duty_B, uint32_t* duties) {
  if (fabsf(duty_B) < MINIMUM_DUTY) {
    duty_B = 0.0f;
  }
  if (duty_B > 0) {
    set_motor_mode_B(MOTOR_CCW);
  } else if (duty_B < 0) {
    set_motor_mode_B(MOTOR_CW);
  } else {
    set_motor_mode_B(MOTOR_SHORT_BREAK);
  }

  duty_B = fabs(duty_B);

  if (curr_mode_B != MOTOR_SHORT_BREAK) {
    int b_idx = (curr_mode_B == MOTOR_CW) ? 2 : 3;

    duties[b_idx] = PWM_PERIOD * duty_B;
  }

  // ESP_LOGI(tb6612fng_tag, "motor B mode: %d", curr_mode_B);
  // ESP_LOGI(tb6612fng_tag, "motor B duty: %d", (int)(duty_B * 100));
}

void initialise_pwms() {
  uint32_t pins[] = {A_IN_1, A_IN_2, B_IN_1, B_IN_2};
  uint32_t duties[] = {0, 0, 0, 0};

  ESP_ERROR_CHECK(pwm_init(PWM_PERIOD, duties, 4, pins));

  float phases[] = {0.0f, 0.0f, 0.0f, 0.0f};

  ESP_ERROR_CHECK(pwm_set_phases(phases));
  ESP_ERROR_CHECK(pwm_start());
}

void initialise_gpio() {
  gpio_config_t io_conf;
  // disable interrupt
  io_conf.intr_type = GPIO_INTR_DISABLE;
  // set as output mode
  io_conf.mode = GPIO_MODE_OUTPUT;
  // bit mask of the pins that you want to set,e.g.GPIO15/16
  io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
  // disable pull-down mode
  io_conf.pull_down_en = 0;
  // disable pull-up mode
  io_conf.pull_up_en = 0;
  // configure GPIO with the given settings
  ESP_ERROR_CHECK(gpio_config(&io_conf));
}

void initialise_driver() {
  // initialise GPIO pins
  initialise_gpio();

  // keep STBY on high during init
  gpio_set_level(STBY, 1u);

  // initialise PWM pins
  initialise_pwms();

  // Set initial motor modes
  set_both_motor_modes(MOTOR_SHORT_BREAK);
  pwm_stop(0x0);
}

float clamp_and_convert_torque_to_duty(float torque) {
  float torqueMin = -1.0f;
  float torqueMax = 1.0f;

  float clampedTorque = fminf(fmaxf(torqueMin, torque), torqueMax);

  // consider only the magnitude of the torque, otherwise 0 torque = 0.5 duty
  float noSignClampedTorque = fabsf(clampedTorque);
  float convertedDuty = (noSignClampedTorque) / (torqueMax);

  // update the converted duty to have the sign
  convertedDuty *= clampedTorque / noSignClampedTorque;

  return convertedDuty;
}

void set_torques() {
  float d_l = clamp_and_convert_torque_to_duty(motor_torques.T_left);
  float d_r = clamp_and_convert_torque_to_duty(motor_torques.T_right);

  set_motor_duties(d_l, d_r);
}

void driver_task(void* args) {
  initialise_driver();

  pwm_start();

  portTickType xLastWakeTime = xTaskGetTickCount();

  while (1) {
    // blocking action:
    if (read_from_motor_queue(&motor_torques) != ESP_OK) {
      ESP_LOGE(tb6612fng_tag,
               "Failed when attempting to read motor torques from queue");
      continue;
    }

    set_torques();

    vTaskDelayUntil(&xLastWakeTime, xDriverFrequency);
  }
}
