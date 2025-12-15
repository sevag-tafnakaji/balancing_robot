
#include "tb6612fng.h"

// TODO: Implement state machine for direction transition and better duty cycle
// changing

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

  if (curr_mode_A != MOTOR_SHORT_BREAK) {
    int a_idx = (curr_mode_A == MOTOR_CW) ? 0 : 1;

    ui_duties[a_idx] = PWM_PERIOD * duty_A;
  }

  if (curr_mode_B != MOTOR_SHORT_BREAK) {
    int b_idx = (curr_mode_B == MOTOR_CW) ? 2 : 3;

    ui_duties[b_idx] = PWM_PERIOD * duty_B;
  }

  uint32_t* pDuties = ui_duties;
  ESP_LOGD(tb6612fng_tag, "motor A mode: %d, motor B mode: %d", curr_mode_A,
           curr_mode_B);
  ESP_LOGD(tb6612fng_tag, "motor A duties: [%d, %d], motor B duties: [%d, %d]",
           ui_duties[0], ui_duties[1], ui_duties[2], ui_duties[3]);
  ESP_ERROR_CHECK(pwm_set_duties(pDuties));
  ESP_ERROR_CHECK(pwm_start());
}

void initialise_pwms() {
  //   uint32_t pins[] = {PWM_A, PWM_B};
  //   uint32_t duties[] = {40, 40};

  uint32_t pins[] = {A_IN_1, A_IN_2, B_IN_1, B_IN_2};
  uint32_t duties[] = {0, 0, 0, 0};

  //   ESP_ERROR_CHECK(pwm_init(PWM_PERIOD, duties, 2, pins));
  ESP_ERROR_CHECK(pwm_init(PWM_PERIOD, duties, 4, pins));

  // ESP_ERROR_CHECK(pwm_set_phase(1u, 0.0f));
  // initialise_pwm(PWM_PERIOD, 1.0f, 2u, PWM_B);
  // ESP_ERROR_CHECK(pwm_set_phase(2u, 0.0f));

  //   float phases[] = {0.0f, 0.0f};
  float phases[] = {0.0f, 0.0f, 0.0f, 0.0f};
  // float *pPhases = phases;

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

void driver_task(void* args) {
  initialise_driver();

  pwm_start();

  while (1) {
    ESP_LOGI(tb6612fng_tag, "Rotating slowly CW for 2 sec");
    set_both_motor_modes(MOTOR_CW);

    set_motor_duties(0.2, 0.2);

    vTaskDelay(2000 / portTICK_RATE_MS);

    // Stop for 1 second

    ESP_LOGI(tb6612fng_tag, "Stopping for 1 sec");

    set_both_motor_modes(MOTOR_SHORT_BREAK);

    set_motor_duties(0, 0);

    vTaskDelay(1000 / portTICK_RATE_MS);

    // Rotate CCW faster for 2 seconds

    ESP_LOGI(tb6612fng_tag, "Rotating quickly CCW for 2 sec");
    set_both_motor_modes(MOTOR_CCW);

    set_motor_duties(0.8, 0.8);

    vTaskDelay(2000 / portTICK_RATE_MS);

    // Stop for 1 second

    ESP_LOGI(tb6612fng_tag, "Stopping for 1 sec");

    set_both_motor_modes(MOTOR_SHORT_BREAK);

    set_motor_duties(0, 0);

    vTaskDelay(1000 / portTICK_RATE_MS);
  }
}
