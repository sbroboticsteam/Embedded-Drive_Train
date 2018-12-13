/*-----------------------------------------------------------------------
-   File Name:		application.c
-   Description:	Main for our application code.
-   Date:					8/24/2018
-   Name:					John Boccio
- Stony Brook University Robotics
-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------
- Private Includes
-----------------------------------------------------------------------*/
#include <stdbool.h>
#include <string.h>
#include "application.h"
#include "motor_control.h"
#include "tim.h"
#include "gpio.h"
#include "usart.h"
/*-----------------------------------------------------------------------
- Private Defines & Macros
-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------
- Private Typedefs & Enumerations
-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------
- Private Variables
-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------
- Private External References
-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------
- Private Function Prototypes
-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------
- Public Functions
-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------
-   app_run
-   Parameters:
-     void
-   Returns:
-     void 
-   Description:
-    	Our main loop for the application code.
-----------------------------------------------------------------------*/
void app_run(void) {
  char printer[64];

  motors_init();
  float pwm_val = .5;
  direction_t change_dir = FORWARD;

  set_mtr_pwm(MTR1, pwm_val);
  set_mtr_dir(MTR1, change_dir);

  pwm_on_off(MTR_ALL, MTR_ON);
  encoder_on_off(MTR_ALL, MTR_ON);

  uint16_t counter = 0;
  while (1) {
    if (counter++ == 1000) {
      uint16_t rpm = (uint16_t)get_mtr_rpm(MTR1);
      sprintf(printer, "RPM: %d\n", rpm);
      HAL_UART_Transmit(&huart2, (uint8_t *)printer, strlen(printer), 0xFFFF);
    }
    set_mtr_dir(MTR1, change_dir);
    set_mtr_pwm(MTR1, pwm_val);
  }
}

/*-----------------------------------------------------------------------
- Private Functions
-----------------------------------------------------------------------*/
void HAL_SYSTICK_Callback(void) {
	// Called every 1 ms, use this as time base for updating motor velocity
	mtr_1ms_timeout();
}
