/*-----------------------------------------------------------------------
-   File Name:		motor_control.c
-   Description:  Functionality for controlling the speed of our motors
-   Date:					8/24/2018
-   Name:					John Boccio
- Stony Brook University Robotics
-----------------------------------------------------------------------*/

#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

/*-----------------------------------------------------------------------
- Public Includes
-----------------------------------------------------------------------*/
#include <stdint.h>
#include "stm32f446xx.h"
#include "tim.h"

/*-----------------------------------------------------------------------
- Public Defines & Macros
-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------
- Public Typedefs & Enumerations
-----------------------------------------------------------------------*/
typedef struct dir_ctrl {
	GPIO_TypeDef *	gpio_port;
	uint16_t 				gpio_pin;
} dir_ctrl_t;

typedef struct pwm {
	TIM_HandleTypeDef * hpwm;
	uint32_t channel;
} pwm_t;

typedef struct position {
	TIM_HandleTypeDef * hencoder;
	__IO uint32_t				prev_encoder_cnt;
	__IO float					rpm;
	uint32_t						channel;
} position_t;

typedef struct motor {
	dir_ctrl_t dir_ctrl;
	pwm_t pwm;
	position_t position;
} motor_t;

typedef enum direction {
	REVERSE = 0x00,
	FORWARD = 0x01,
} direction_t;

typedef enum mtr_status {
	MTR_OFF = 0x00,
	MTR_ON  = 0x01,
} mtr_status_t;

typedef enum mtr_id {
	MTR1 = 0,
	MTR2,
	MTR3,
	MTR4,
	MTR_ALL,
} mtr_id_t;

/*-----------------------------------------------------------------------
- Public External References
-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------
- Public Function Prototypes
-----------------------------------------------------------------------*/
void motors_init(void);
uint16_t get_mtr_cnt(mtr_id_t mtr_id);
void set_mtr_pwm(mtr_id_t mtr_id, float pwm);
void set_mtr_dir(mtr_id_t mtr_id, direction_t dir);
float get_mtr_rpm(mtr_id_t mtr_id);
void pwm_on_off(mtr_id_t mtr_id, mtr_status_t mtr_status);
void encoder_on_off(mtr_id_t mtr_id, mtr_status_t mtr_status);
void mtr_1ms_timeout(void);

#endif /* __MOTOR_CONTROL_H */
