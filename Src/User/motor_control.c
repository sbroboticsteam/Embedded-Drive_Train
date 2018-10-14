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
/** Specifies how often to take motor velocity measurements in ms **/
#define MTR_VELOCITY_TIMESCALE	((uint16_t) 500)
/** Wheel circumference to calculate velocity in meters **/
#define WHEEL_CIRCUMFERENCE			((float) 0.1)
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
static void update_mtr_velocity(mtr_id_t mtr_id);
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
	motor_3.position.channel = TIM_CHANNEL_1;
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
	HAL_GPIO_WritePin(motor->dir_ctrl.gpio_port, motor->dir_ctrl.gpio_pin, dir);
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
-   get_mtr_velocity
-   Parameters:
-     mtr_id: Specify which motor to get velocity of.
-   Returns:
-     float: motor's velocity in m/s
-   Description:
-    	Get a motors velocity.
-----------------------------------------------------------------------*/
float get_mtr_velocity(mtr_id_t mtr_id) {
	return get_mtr(mtr_id)->position.velocity;
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
		motor->pwm.hpwm->Instance->CCR1 = motor->pwm.hpwm->Instance->ARR * pwm;
		break;
	}
	case TIM_CHANNEL_2: {
		motor->pwm.hpwm->Instance->CCR2 = motor->pwm.hpwm->Instance->ARR * pwm;
		break;
	}
	case TIM_CHANNEL_3: {
		motor->pwm.hpwm->Instance->CCR3 = motor->pwm.hpwm->Instance->ARR * pwm;
		break;
	}
	case TIM_CHANNEL_4: {
		motor->pwm.hpwm->Instance->CCR4 = motor->pwm.hpwm->Instance->ARR * pwm;
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
		for (uint8_t i = 0; i < MTR_ALL; i++) {
			pwm_on_off_helper(MTR1 + i, mtr_status);
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
		for (uint8_t i = 0; i < MTR_ALL; i++) {
			encoder_on_off_helper(MTR1 + i, mtr_status);
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
-			MTR_VELOCITY_TIMESCALE time has passed, we will update the velocity
-			of each of the motors.
-----------------------------------------------------------------------*/
void mtr_1ms_timeout(void) {
	static uint16_t time_passed = 0;

	if (time_passed++ == MTR_VELOCITY_TIMESCALE) {
		for (uint8_t i = 0; i < MTR_ALL; i++) {
			update_mtr_velocity(i);
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
-   update_mtr_velocity
-   Parameters:
-     mtr_id: Specify which motor to update velocity of.
-   Returns:
-     void
-   Description:
-    	Updates the velocity of the motor based off of current and previous
-			encoder counts and the circumference of our wheels.
-----------------------------------------------------------------------*/
static void update_mtr_velocity(mtr_id_t mtr_id) {
	motor_t * motor = get_mtr(mtr_id);
	int16_t change_in_encoder = motor->position.hencoder->Instance->CNT - motor->position.prev_encoder_cnt;
	float percent_circum_moved = ENCODER_TICKS_PER_REV / change_in_encoder;
	motor->position.velocity = (percent_circum_moved * WHEEL_CIRCUMFERENCE) / MTR_VELOCITY_TIMESCALE;
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
