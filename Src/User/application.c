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
#define MAX_COMM_MSG_SIZE		((uint16_t) 256)

/*-----------------------------------------------------------------------
- Private Typedefs & Enumerations
-----------------------------------------------------------------------*/
typedef enum fsm_state {
	IDLE,
	CAN_PARSE,
	CAN_EXECUTE,
	CAN_SEND,
} fsm_state_t;

typedef struct comm_msg {
	uint8_t message[MAX_COMM_MSG_SIZE];
	uint16_t length;
	bool new_msg;
} comm_msg_t;
/*-----------------------------------------------------------------------
- Private Variables
-----------------------------------------------------------------------*/
//static fsm_state_t fsm_state;
//static comm_msg_t comm_msg = { 0 };
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
	char encoder_data[64];
	motors_init();
	float pwm_val = .5;
	direction_t change_dir = FORWARD;

	set_mtr_pwm(MTR1, pwm_val);
	set_mtr_dir(MTR1, change_dir);

	pwm_on_off(MTR_ALL, MTR_ON);
	encoder_on_off(MTR_ALL, MTR_ON);



	while (1) {
		float speed = get_mtr_velocity(MTR1);
		HAL_UART_Transmit(&huart2, (uint8_t*)encoder_data, strlen(encoder_data), 0xFFFF);
		set_mtr_dir(MTR1, change_dir);
		set_mtr_pwm(MTR1, pwm_val);
	}
//	while (1) {
//
//		/** Finite state machine **/
//		switch (fsm_state) {
//		case IDLE: {
//			if (comm_msg.new_msg) {
//				comm_msg.new_msg = false;
//				fsm_state = CAN_PARSE;
//			}
//			break;
//		}
//		case CAN_PARSE: {
//
//			fsm_state = CAN_EXECUTE;
//			break;
//		}
//		case CAN_EXECUTE: {
//
//			fsm_state = CAN_SEND;
//			break;
//		}
//		case CAN_SEND: {
//
//			fsm_state = IDLE;
//			break;
//		}
//		default: {
//			fsm_state = IDLE;
//			break;
//		}
//		}
//	}
}

/*-----------------------------------------------------------------------
- Private Functions
-----------------------------------------------------------------------*/
void HAL_SYSTICK_Callback(void) {
	// Called every 1 ms, use this as time base for updating motor velocity
	mtr_1ms_timeout();
}
