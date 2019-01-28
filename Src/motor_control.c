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
#include <math.h>
#include "timing.h"
#include "gpio.h"
#include "motor_control.h"
#include "tim.h"

#ifdef DEBUG
#include <string.h>
#endif /* DEBUG */
/*-----------------------------------------------------------------------
- Private Defines & Macros
-----------------------------------------------------------------------*/
/** Specifies how often to take motor RPM measurements in ms **/
#define MTR_RPM_TIMESCALE	        ((uint16_t) 50)
/** One full encoder revolution goes from 0 to 2047 **/
#define ENCODER_TICKS_PER_REV		((uint16_t) 1024)
/*-----------------------------------------------------------------------
- Private Typedefs & Enumerations
-----------------------------------------------------------------------*/
typedef struct dir_ctrl {
  GPIO_TypeDef *        gpio_port;
  uint16_t 	        gpio_pin;
  direction_t           dir;
  direction_t           dir_goal;
} dir_ctrl_t;

typedef struct pwm {
  uint32_t      pwm_channel;
  uint32_t      pwm_val;
  uint32_t      pwm_goal;
  timer_t       pwm_timer;
} pwm_t;

typedef struct mtr_encoder {
  TIM_HandleTypeDef *   hencoder;
  __IO uint32_t         prev_encoder_cnt;
  uint32_t		channel;
} mtr_encoder_t;

typedef struct motor {
  dir_ctrl_t    dir_ctrl;
  pwm_t         pwm;
  mtr_encoder_t position;
  float         rpm;
} motor_t;
/*-----------------------------------------------------------------------
- Private Variables
-----------------------------------------------------------------------*/
static motor_t  motors[MTR_COUNT];
/** Ramp the pwm up/down every 50 ms by MTR_PWM_RAMP amount **/
static uint8_t  mtr_pwm_period = 50;
/** Amount to ramp the pwm up/down after MTR_PWM_PERIOD **/
static uint16_t mtr_pwm_ramp = 420;
/*-----------------------------------------------------------------------
- Private Function Prototypes
-----------------------------------------------------------------------*/
static void update_rpm(void);

static bool adjust_pwm(mtr_num_t mtr_num);
static void set_pwm(mtr_num_t mtr_num, uint32_t pwm);

static void set_dir(mtr_num_t mtr_num, direction_t dir);

static void motors_init(void);
/*-----------------------------------------------------------------------
- Private External References
-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------
- Public Functions
-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------
-   motor_task
-   Parameters:
-     void
-   Returns:
-     void
-   Description:
-    	The task that will be continously running to control the motors.
-----------------------------------------------------------------------*/
void motor_task(motor_state_t mtr_cmd) {
  static timer_t rpm_timer;

  switch (mtr_cmd) {
    case MOTORS_INIT: {
      /* Set up the pwm and encoder timers for the motors and start them */
      motors_init();

      /* Set the rpm timer to expire in MTR_RPM_TIMESCALE ms from now */
      set_timer(&rpm_timer, MTR_RPM_TIMESCALE);
      break;
    }
    case MOTORS_RUN: {
      /* Check if we should calculate new rpm values */
      if (timeout(&rpm_timer)) {
        /* Set the rpm timer to trigger again in MTR_RPM_TIMESCALE ms from now */
        set_timer(&rpm_timer, MTR_RPM_TIMESCALE);

        /* Calculate the new rpm values */
        update_rpm();

        /* Output of motor values for debugging purposes */
#ifdef DEBUG
        static uint8_t counter = 0;
        if (counter++ == 20) {
          printf("Motor 0 RPM: %4.5f, CNT: %d\n", motors[MTR_0].rpm, get_mtr_cnt(MTR_0));
          counter = 0;
        }
#endif /* DEBUG */
      }

      /* Check if we need to ramp up/down our pwm values */
      for (mtr_num_t mtr = MTR_0; mtr < MTR_COUNT; mtr++) {
        /* Check if the ramp pwm timer has timed out */
        if (timeout(&motors[mtr].pwm.pwm_timer)) {
          /* Slightly adjust the pwm of that motor, if it needs to be adjusted
             again, set the timer */
          if (adjust_pwm(mtr)) {
            stop_timer(&motors[mtr].pwm.pwm_timer);
          }
          else {
            set_timer(&motors[mtr].pwm.pwm_timer, mtr_pwm_period);
          }
        }
        /* else PWM is at the correct value */
      }

      break;
    }
    default: {
      break;
    }
  }
}

/*-----------------------------------------------------------------------
-   get_mtr_cnt
-   Parameters:
-     mtr_num: Specify which motor to get encoder count of.
-   Returns:
-     uint16_t: Motor's encoder count
-   Description:
-    	Get encoder count of a motor by reading CNT register of timer
-----------------------------------------------------------------------*/
uint16_t get_mtr_cnt(mtr_num_t mtr_num) {
  return motors[mtr_num].position.hencoder->Instance->CNT;
}

/*-----------------------------------------------------------------------
-   get_mtr_rpm
-   Parameters:
-     mtr_num: Specify which motor to get rpm of.
-   Returns:
-     float: motor's rpm
-   Description:
-     Get a motors rpm.
-----------------------------------------------------------------------*/
float get_mtr_rpm(mtr_num_t mtr_num) {
  return motors[mtr_num].rpm;
}

/*-----------------------------------------------------------------------
-   set_mtr_pwm_dir
-   Parameters:
-     mtr_num: Specify which motor to set the pwm and direction of
-     pwm: PWM value to set
-     dir: Direction to set
-   Returns:
-     void
-   Description:
-     Sets a PWM and direction goal for a motor and accelerates towards
-     that goal.
-----------------------------------------------------------------------*/
void set_mtr_pwm_dir(mtr_num_t mtr_num, uint32_t pwm, direction_t dir) {
  /* Check if the passed pwm parameter is invalid */
  if (pwm > htim2.Instance->ARR) {
    pwm = htim2.Instance->ARR;
  }

  /* Set the new desired pwm value and direction */
  /* These values are set as goals and we ramp to them using a configurable acceleration */
  motors[mtr_num].pwm.pwm_goal      = pwm;
  motors[mtr_num].dir_ctrl.dir_goal = dir;
  
  /* Begin the timer to ramp up the motor pwm */
  set_timer(&motors[mtr_num].pwm.pwm_timer, mtr_pwm_period);
}

/*-----------------------------------------------------------------------
- Private Functions
-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------
-   motors_init
-   Parameters:
-     void
-   Returns:
-     void
-   Description:
-    	Initializes all motor structs with their respective pwm timer,
-	encoder timer, and direction pin.
-----------------------------------------------------------------------*/
static void motors_init(void) {
  memset(motors, 0, sizeof(motors));

  /* Set the port and pin used to control the direction of the motor */
  motors[MTR_0].dir_ctrl.gpio_port = MTR0_DIR_GPIO_Port;
  motors[MTR_0].dir_ctrl.gpio_pin  = MTR0_DIR_Pin;
  /* Set the timer used to handle the encoder and its channel */
  motors[MTR_0].position.hencoder = &htim4;
  motors[MTR_0].position.channel = TIM_CHANNEL_1;
  /* Set the channel of the pwm timer used for the motor */
  motors[MTR_0].pwm.pwm_channel = TIM_CHANNEL_1;
  motors[MTR_0].pwm.pwm_val = 0;

  motors[MTR_1].dir_ctrl.gpio_port = MTR1_DIR_GPIO_Port;
  motors[MTR_1].dir_ctrl.gpio_pin  = MTR1_DIR_Pin;
  motors[MTR_1].position.hencoder = &htim3;
  motors[MTR_1].position.channel = TIM_CHANNEL_1;
  motors[MTR_1].pwm.pwm_channel = TIM_CHANNEL_2;
  motors[MTR_1].pwm.pwm_val = 0;

  motors[MTR_2].dir_ctrl.gpio_port = MTR2_DIR_GPIO_Port;
  motors[MTR_2].dir_ctrl.gpio_pin  = MTR2_DIR_Pin;
  motors[MTR_2].position.hencoder = &htim8;
  motors[MTR_2].position.channel = TIM_CHANNEL_1;
  motors[MTR_2].pwm.pwm_channel = TIM_CHANNEL_3;
  motors[MTR_2].pwm.pwm_val = 0;

  motors[MTR_3].dir_ctrl.gpio_port = MTR3_DIR_GPIO_Port;
  motors[MTR_3].dir_ctrl.gpio_pin  = MTR3_DIR_Pin;
  motors[MTR_3].position.hencoder = &htim1;
  motors[MTR_3].position.channel = TIM_CHANNEL_1;
  motors[MTR_3].pwm.pwm_channel = TIM_CHANNEL_4;
  motors[MTR_3].pwm.pwm_val = 0;

  /* Turn on PWM timers for all motors */
  for (mtr_num_t mtr = MTR_0; mtr < MTR_COUNT; mtr++) {
    HAL_TIM_PWM_Start(&htim2, motors[mtr].pwm.pwm_channel);
  }

  /* Turn on encoder timers for all motors */
  for (mtr_num_t mtr = MTR_0; mtr < MTR_COUNT; mtr++) {
    HAL_TIM_Encoder_Start(motors[mtr].position.hencoder, motors[mtr].position.channel);
  }
}

/*-----------------------------------------------------------------------
-   set_dir
-   Parameters:
-     mtr_num: Specify which motor to change direction of.
-     dir: Direction you want the motor to go.
-   Returns:
-     void
-   Description:
-     Set the motor to go forward or backwards
-----------------------------------------------------------------------*/
static void set_dir(mtr_num_t mtr_num, direction_t dir) {
  HAL_GPIO_WritePin(motors[mtr_num].dir_ctrl.gpio_port, motors[mtr_num].dir_ctrl.gpio_pin, (GPIO_PinState) dir);
  motors[mtr_num].dir_ctrl.dir = dir;
}

/*-----------------------------------------------------------------------
-   set_pwm
-   Parameters:
-     mtr_num: Specify which motor to set pwm of.
-     pwm: value between 0 and ARR to set the duty cycle
-   Returns:
-     void
-   Description:
-      Set the timers CCR register to the new pwm value. Timer count 0 to CCR 
-      specifies PWM high, CCR to ARR specifies PWM low.
-----------------------------------------------------------------------*/
static void set_pwm(mtr_num_t mtr_num, uint32_t pwm) {
  switch (motors[mtr_num].pwm.pwm_channel) {
    case TIM_CHANNEL_1: {
      htim2.Instance->CCR1 = pwm;
      break;
    }
    case TIM_CHANNEL_2: {
      htim2.Instance->CCR2 = pwm;
      break;
    }
    case TIM_CHANNEL_3: {
      htim2.Instance->CCR3 = pwm;
      break;
    }
    case TIM_CHANNEL_4: {
      htim2.Instance->CCR4 = pwm;
      break;
    }
    default: {
      break;
    }
  }
}

/*-----------------------------------------------------------------------
-   update_rpm
-   Parameters:
-     void
-   Returns:
-     void
-   Description:
-     Updates the rpm of all the motors based off of current and previous
-     encoder counts.
-----------------------------------------------------------------------*/
static void update_rpm(void) {
  for (mtr_num_t mtr = MTR_0; mtr < MTR_COUNT; mtr++) {
    int16_t change_in_encoder;
    uint32_t current_count = motors[mtr].position.hencoder->Instance->CNT;
    uint32_t prev_count = motors[mtr].position.prev_encoder_cnt;
    direction_t mtr_dir = (direction_t) HAL_GPIO_ReadPin(motors[mtr].dir_ctrl.gpio_port, motors[mtr].dir_ctrl.gpio_pin);

    /* Need to account for possible roll over, check if direction is reverse */
    if (mtr_dir == REVERSE) {
      /* Check if we rolled over if CNT is greater than prev_encoder_cnt when going reverse */
      if (current_count > prev_count) {
        change_in_encoder = current_count - (prev_count + ENCODER_TICKS_PER_REV);
      }
      else {
        change_in_encoder = prev_count - current_count;
      }
    }
    /* Direction is forward */
    else {
      /* Check if we rolled over if CNT is less than our prev_encoder_cnt when going forward */
      if (current_count < prev_count) {
        change_in_encoder = current_count + ENCODER_TICKS_PER_REV - prev_count;
      }
      else {
        change_in_encoder = current_count - prev_count;
      }
    }
    motors[mtr].position.prev_encoder_cnt = current_count;

    float rotations =  change_in_encoder / ((float)(ENCODER_TICKS_PER_REV - 1));
    /* Rotations per ms multiplied by ms per min */
    motors[mtr].rpm = (rotations / (float)MTR_RPM_TIMESCALE) * 60000;
  }
}

/*-----------------------------------------------------------------------
-   adjust_pwm
-   Parameters:
-     mtr_num: motor to adjust the pwm of
-   Returns:
-     bool: does it need to be adjusted again after this
-   Description:
-     Adjusts the pwm of a motor in small increments for a smooth ramp
-     up and ramp down.
-----------------------------------------------------------------------*/
static bool adjust_pwm(mtr_num_t mtr_num) {
  pwm_t *pwm           = &motors[mtr_num].pwm;
  dir_ctrl_t *dir_ctrl = &motors[mtr_num].dir_ctrl;
  
  /* Check if we are going in the intended direction */
  if (dir_ctrl->dir == dir_ctrl->dir_goal) {
    /* Ramp up */
    if (pwm->pwm_goal > pwm->pwm_val) {
      /* Check if we can increment by max pwm ramp value */
      if (pwm->pwm_goal - pwm->pwm_val >= mtr_pwm_ramp) {
        pwm->pwm_val += mtr_pwm_ramp;
      }
      /* Otherwise our pwm value has hit the goal */
      else {
        pwm->pwm_val = pwm->pwm_goal;
      }
    }
    /* Ramp down */
    else if (pwm->pwm_goal < pwm->pwm_val) {
      /* Check if we can decrement by max pwm ramp value */
      if (pwm->pwm_val - pwm->pwm_goal >= mtr_pwm_ramp) {
        pwm->pwm_val -= mtr_pwm_ramp;
      }
      /* Otherwise our pwm value has hit the goal */
      else {
        pwm->pwm_val = pwm->pwm_goal;
      }
    }
  }
  /* Our current direction is not our intended direction, we have to ramp down
     until our pwm is 0, then change direction and ramp up to the goal */
  else {
    uint32_t pwm_diff = pwm->pwm_val - mtr_pwm_ramp;
    /* If our pwm difference is 0, then all we need to do is change direction */
    if (pwm_diff == 0) {
      dir_ctrl->dir = dir_ctrl->dir_goal;
    }
    /* Check if we loop around when decrementing by the max amount */
    else if (pwm_diff > pwm->pwm_val) {
      /* Direction can be changed since we looped around */
      dir_ctrl->dir = dir_ctrl->dir_goal;
      
      /* Calculate the amount we can ramp up after changing directions */
      uint32_t ramp_left_over = mtr_pwm_ramp - pwm->pwm_val;
      /* Check if the left over amount allows us to reach our goal */
      if (ramp_left_over >= pwm->pwm_goal) {
        pwm->pwm_val = pwm->pwm_goal;
      }
      /* Otherwise we still have to ramp up to our goal */
      else {
        pwm->pwm_val = ramp_left_over;
      }
    }
    /* We can ramp down by max amount */
    else {
      pwm->pwm_val -= mtr_pwm_ramp;
    }
  }
        
  set_pwm(mtr_num, pwm->pwm_val);
  set_dir(mtr_num, dir_ctrl->dir);
  return (pwm->pwm_val == pwm->pwm_goal) && (dir_ctrl->dir == dir_ctrl->dir_goal);
}
