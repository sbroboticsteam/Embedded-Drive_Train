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
#include "arm_math.h"
/*-----------------------------------------------------------------------
- Public Defines & Macros
-----------------------------------------------------------------------*/
/** Number of motors on the rover **/
#define MTR_COUNT                       ((uint8_t) 4)
/*-----------------------------------------------------------------------
- Public Typedefs & Enumerations
-----------------------------------------------------------------------*/
typedef enum direction {
  REVERSE = 0x00,
  FORWARD = 0x01,
} direction_t;

typedef enum mtr_num {
  MTR_0 = 0,
  MTR_1 = 1,
  MTR_2 = 2,
  MTR_3 = 3,
} mtr_num_t;
/*-----------------------------------------------------------------------
- Public External References
-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------
- Public Function Prototypes
-----------------------------------------------------------------------*/
void motor_task(void);
void motors_init(void);
uint16_t get_mtr_cnt(mtr_num_t mtr_num);
void set_mtr_pwm_dir(mtr_num_t mtr_num, uint32_t pwm, direction_t dir);
float get_mtr_rpm(mtr_num_t mtr_num);

#endif /* __MOTOR_CONTROL_H */
