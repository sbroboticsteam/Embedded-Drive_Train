/*-----------------------------------------------------------------------
-   File Name:		motor_control.c
-   Description:  Functionality for controlling the speed of our motors
-   Date:					8/24/2018
-   Name:					John Boccio
- Stony Brook University Robotics
-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------
- Private Includes
-----------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include "gpio.h"
#include "motor_control.h"
#include "tim.h"
/*-----------------------------------------------------------------------
- Private Defines & Macros
-----------------------------------------------------------------------*/
/** Specifies how often to take motor RPM measurements in ms **/
#define MTR_RPM_TIMESCALE	((uint16_t) 50)
/** One full encoder revolution goes from 0 to 1023 **/
#define ENCODER_TICKS_PER_REV		((uint16_t) 1024)
/*-----------------------------------------------------------------------
- Private Typedefs & Enumerations
-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------
- Private Variables
-----------------------------------------------------------------------*/
static motor_t motor_1;
static motor_t motor_2;
static motor_t motor_3;
static motor_t motor_4;
/*-----------------------------------------------------------------------
- Private Function Prototypes
-----------------------------------------------------------------------*/
static motor_t * get_mtr(mtr_id_t mtr_id);
static void update_mtr_rpm(mtr_id_t mtr_id);
static void pwm_on_off_helper(mtr_id_t mtr_id, mtr_status_t mtr_status);
static void encoder_on_off_helper(mtr_id_t mtr_id, mtr_status_t mtr_status);
/*-----------------------------------------------------------------------
- Private External References
-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------
- Public Functions
-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------
-   motors_init
-   Parameters:
-     void
-   Returns:
-     void
-   Description:
-    	Initializes all motor structs with their respective pwm timer,
-			encoder timer, and direction pin.
-----------------------------------------------------------------------*/
void motors_init(void) {
  memset(&motor_1, 0, sizeof(motor_t));
  motor_1.dir_ctrl.gpio_port = MTR1_DIR_GPIO_Port;
  motor_1.dir_ctrl.gpio_pin  = MTR1_DIR_Pin;
  motor_1.position.hencoder = &htim4;
  motor_1.position.channel = TIM_CHANNEL_1;
  motor_1.pwm.hpwm = &htim10;
  motor_1.pwm.channel = TIM_CHANNEL_1;

  memset(&motor_2, 0, sizeof(motor_2));
  motor_2.dir_ctrl.gpio_port = MTR2_DIR_GPIO_Port;
  motor_2.dir_ctrl.gpio_pin  = MTR2_DIR_Pin;
  motor_2.position.hencoder = &htim3;
  motor_2.position.channel = TIM_CHANNEL_2;
  motor_2.pwm.hpwm = &htim5;
  motor_2.pwm.channel = TIM_CHANNEL_2;

  memset(&motor_3, 0, sizeof(motor_t));
  motor_3.dir_ctrl.gpio_port = MTR2_DIR_GPIO_Port;
  motor_3.dir_ctrl.gpio_pin  = MTR2_DIR_Pin;
  motor_3.position.hencoder = &htim8;
  motor_3.position.channel = TIM_CHANNEL_2;
  motor_3.pwm.hpwm = &htim2;
  motor_3.pwm.channel = TIM_CHANNEL_4;

  memset(&motor_4, 0, sizeof(motor_t));
  motor_4.dir_ctrl.gpio_port = MTR2_DIR_GPIO_Port;
  motor_4.dir_ctrl.gpio_pin  = MTR2_DIR_Pin;
  motor_4.position.hencoder = &htim1;
  motor_4.position.channel = TIM_CHANNEL_2;
  motor_4.pwm.hpwm = &htim12;
  motor_4.pwm.channel = TIM_CHANNEL_2;
}

/*-----------------------------------------------------------------------
-   set_mtr_dir
-   Parameters:
-     mtr_id: Specify which motor to change direction of.
-			dir: Direction you want the motor to go.
-   Returns:
-     void
-   Description:
-    	Set the motor to go forward or backwards
-----------------------------------------------------------------------*/
void set_mtr_dir(mtr_id_t mtr_id, direction_t dir) {
  motor_t * motor = get_mtr(mtr_id);
  GPIO_PinState pin_state = (dir == FORWARD)? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(motor->dir_ctrl.gpio_port, motor->dir_ctrl.gpio_pin, pin_state);
}

/*-----------------------------------------------------------------------
-   set_mtr_dir
-   Parameters:
-     mtr_id: Specify which motor to get encoder count of.
-   Returns:
-     uint16_t: Motor's encoder count
-   Description:
-    	Get encoder count of a motor by reading CNT register of timer
-----------------------------------------------------------------------*/
uint16_t get_mtr_cnt(mtr_id_t mtr_id) {
  return (uint16_t) get_mtr(mtr_id)->position.hencoder->Instance->CNT;
}

/*-----------------------------------------------------------------------
-   get_mtr_rpm
-   Parameters:
-     mtr_id: Specify which motor to get rpm of.
-   Returns:
-     float: motor's rpm in m/s
-   Description:
-    	Get a motors rpm.
-----------------------------------------------------------------------*/
float get_mtr_rpm(mtr_id_t mtr_id) {
  return get_mtr(mtr_id)->position.rpm;
}

/*-----------------------------------------------------------------------
-   set_mtr_pwm
-   Parameters:
-     mtr_id: Specify which motor to set pwm of.
-			pwm: value between 0 and 1 to set duty cycle of pwm
-   Returns:
-     void
-   Description:
-    	Set the timers CCR register to a percentage of its ARR register.
-			Timer count 0 to CCR specifies PWM high, CCR to ARR specifies PWM
-			low.
-----------------------------------------------------------------------*/
void set_mtr_pwm(mtr_id_t mtr_id, float pwm) {
  motor_t * motor = get_mtr(mtr_id);

  if (pwm < 0) {
    pwm = 0;
  }
  else if (pwm > 1.0) {
    pwm = 1;
  }

  switch (motor->pwm.channel) {
  case TIM_CHANNEL_1: {
    motor->pwm.hpwm->Instance->CCR1 = (uint32_t) (motor->pwm.hpwm->Instance->ARR * pwm);
    break;
  }
  case TIM_CHANNEL_2: {
    motor->pwm.hpwm->Instance->CCR2 = (uint32_t) (motor->pwm.hpwm->Instance->ARR * pwm);
    break;
  }
  case TIM_CHANNEL_3: {
    motor->pwm.hpwm->Instance->CCR3 = (uint32_t) (motor->pwm.hpwm->Instance->ARR * pwm);
    break;
  }
  case TIM_CHANNEL_4: {
    motor->pwm.hpwm->Instance->CCR4 = (uint32_t) (motor->pwm.hpwm->Instance->ARR * pwm);
    break;
  }
  default:
    break;
  }
}

/*-----------------------------------------------------------------------
-   pwm_on_off
-   Parameters:
-     mtr_id: Specify which motor to turn pwm on/off.
-			mtr_status: turn on or off.
-   Returns:
-     void
-   Description:
-    	Turns the pwm timer on/off.
-----------------------------------------------------------------------*/
void pwm_on_off(mtr_id_t mtr_id, mtr_status_t mtr_status) {
  if (mtr_id == MTR_ALL) {
    mtr_id_t id;
    for (uint8_t i = 0; i < MTR_ALL; i++) {
      id = (mtr_id_t) (MTR1 + i);
      pwm_on_off_helper(id, mtr_status);
    }
  }
  else {
    pwm_on_off_helper(mtr_id, mtr_status);
  }
}

/*-----------------------------------------------------------------------
-   encoder_on_off
-   Parameters:
-     mtr_id: Specify which motor to turn encoder on/off.
-			mtr_status: turn on or off.
-   Returns:
-     void
-   Description:
-    	Turns the encoder timer on/off.
-----------------------------------------------------------------------*/
void encoder_on_off(mtr_id_t mtr_id, mtr_status_t mtr_status) {
  if (mtr_id == MTR_ALL) {
    mtr_id_t id;
    for (uint8_t i = 0; i < MTR_ALL; i++) {
      id = (mtr_id_t) (MTR1 + i);
      encoder_on_off_helper(id, mtr_status);
    }
  }
  else {
    encoder_on_off_helper(mtr_id, mtr_status);
  }
}

/*-----------------------------------------------------------------------
-   mtr_1ms_timeout
-   Parameters:
-     void
-   Returns:
-     void
-   Description:
-    	This function gets called every 1 ms by the SysTick timer. Once
-			MTR_RPM_TIMESCALE time has passed, we will update the rpm
-			of each of the motors.
-----------------------------------------------------------------------*/
void mtr_1ms_timeout(void) {
  static uint16_t time_passed = 0;

  if (time_passed++ == MTR_RPM_TIMESCALE) {
    mtr_id_t id;
    for (uint8_t i = 0; i < MTR_ALL; i++) {
      id = (mtr_id_t) i;
      update_mtr_rpm(id);
    }
    time_passed = 0;
  }
}

/*-----------------------------------------------------------------------
- Private Functions
-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------
-   get_mtr
-   Parameters:
-     mtr_id: Specify which motor struct to receive.
-   Returns:
-     motor_t *: Pointer to motor struct.
-   Description:
-    	Get the pointer to a motor struct
-----------------------------------------------------------------------*/
static motor_t * get_mtr(mtr_id_t mtr_id) {
  switch (mtr_id) {
  case MTR1:
    return &motor_1;
  case MTR2:
    return &motor_2;
  case MTR3:
    return &motor_3;
  case MTR4:
  default:
    return &motor_4;
  }
}

/*-----------------------------------------------------------------------
-   update_mtr_rpm
-   Parameters:
-     mtr_id: Specify which motor to update rpm of.
-   Returns:
-     void
-   Description:
-    	Updates the rpm of the motor based off of current and previous
-			encoder counts and the circumference of our wheels.
-----------------------------------------------------------------------*/
static void update_mtr_rpm(mtr_id_t mtr_id) {
  motor_t * motor = get_mtr(mtr_id);
  int16_t change_in_encoder;
  uint32_t current_count = motor->position.hencoder->Instance->CNT;
  uint32_t prev_count = motor->position.prev_encoder_cnt;
  // Stop encoder before reading so it doesn't change during calculations
  encoder_on_off(mtr_id, MTR_OFF);

  // Need to account for possible roll over, check if direction is reverse
  if (HAL_GPIO_ReadPin(motor->dir_ctrl.gpio_port, motor->dir_ctrl.gpio_pin) == GPIO_PIN_RESET) {
    // Check if we rolled over if CNT is greater than prev_encoder_cnt when going reverse
    if (current_count > prev_count) {
      change_in_encoder = prev_count + ENCODER_TICKS_PER_REV - current_count;
    }
    else {
      change_in_encoder = current_count - prev_count;
    }
  }
  // Direction is forward
  else {
    // Check if we rolled over if CNT is less than our prev_encoder_cnt when going forward
    if (motor->position.hencoder->Instance->CNT < motor->position.prev_encoder_cnt) {
      change_in_encoder = current_count + ENCODER_TICKS_PER_REV - prev_count;
    }
    else {
      change_in_encoder = current_count - prev_count;
    }
  }
  motor->position.prev_encoder_cnt = current_count;

  encoder_on_off(mtr_id, MTR_ON);
  float rotations =  change_in_encoder / (float)(ENCODER_TICKS_PER_REV - 1);
  // Rotations per ms multiplied by ms per min
  motor->position.rpm = (rotations / MTR_RPM_TIMESCALE) * 60000;
}

/** See pwm_on_off, helper function to write cleaner code **/
static void pwm_on_off_helper(mtr_id_t mtr_id, mtr_status_t mtr_status) {
  motor_t * mtr = get_mtr(mtr_id);
  if (mtr_status == MTR_OFF) {
    HAL_TIM_PWM_Stop(mtr->pwm.hpwm, mtr->pwm.channel);
  }
  else if (mtr_status == MTR_ON) {
    HAL_TIM_PWM_Start(mtr->pwm.hpwm, mtr->pwm.channel);
  }
}

/** See encoder_on_off, helper function to write cleaner code **/
static void encoder_on_off_helper(mtr_id_t mtr_id, mtr_status_t mtr_status) {
  motor_t * mtr = get_mtr(mtr_id);
  if (mtr_status == MTR_OFF) {
    HAL_TIM_Encoder_Stop(mtr->position.hencoder, mtr->position.channel);
  }
  else if (mtr_status == MTR_ON) {
    HAL_TIM_Encoder_Start(mtr->position.hencoder, mtr->position.channel);
  }
}
