/*-----------------------------------------------------------------------
-   File Name: i2c_slave.c
-   Description: Functionality for this board to be a slave device
-   Date: 1/20/2019
-   Name: John Boccio
- Stony Brook University Robotics
-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------
- Private Includes
-----------------------------------------------------------------------*/
#include <stdint.h>
#include "i2c_slave.h"
#include "i2c.h"
#include "timing.h"
/*-----------------------------------------------------------------------
- Private Defines & Macros
-----------------------------------------------------------------------*/
/* Drive train i2c address */
#define I2C_OWN_ADDR            ((uint8_t) 1 << 1)
/* Max i2c receive size in bytes (Must be >= 5) */
#define SLAVE_RX_BUFF_SIZE      ((uint8_t) 20)
/* After receiving a message length, this is the max time we will wait for the
   actual message */
#define I2C_SLAVE_TIMEOUT       ((uint8_t) 100)
/*-----------------------------------------------------------------------
- Private Typedefs & Enumerations
-----------------------------------------------------------------------*/
typedef enum i2c_state {
  I2C_INIT,
  I2C_IDLE,
  I2C_MSG_LEN_RX,
  I2C_PROCESS_RX,
} i2c_state_t;

typedef enum resp_code {
  NACK_MSG_LEN = 0,
  ACK_MSG_LEN  = 1,
  NACK_MSG     = 2,
  ACK_MSG      = 3,
} resp_code_t;

typedef struct i2c_msg {
  bool msg_len_rx;
  bool msg_rx;
  uint8_t len;
  uint8_t buffer[SLAVE_RX_BUFF_SIZE];
  timer_t timer;
} i2c_msg_t;
/*-----------------------------------------------------------------------
- Private Variables
-----------------------------------------------------------------------*/
static i2c_msg_t i2c_msg = { .msg_len_rx = false, .timer.enabled = 0 };
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
-   i2c_slave_task
-   Parameters:
-     Param1
-   Returns:
-     void 
-   Description:
-     Describe here
-     
-----------------------------------------------------------------------*/
void i2c_slave_task(void) {
  static i2c_state_t i2c_state = I2C_INIT;
  
  switch (i2c_state) {
    case I2C_INIT: {
      /* Wait to receive a byte over i2c specifying the amount of incoming bytes */
      HAL_I2C_Slave_Receive_IT(&hi2c1, i2c_msg.buffer, 1);
      i2c_state = I2C_MSG_LEN_RX;
      break;
    }
    case I2C_MSG_LEN_RX: {
      if (i2c_msg.msg_len_rx) {
        /* Message length was received */
        i2c_msg.len = i2c_msg.buffer[0];
        
        /* Check if the length that the jetson send is valid, then send ACK_MSG_LEN
           and wait for the actual message */
        /* Minimum: 1 byte msg + 4 byte CRC */
        /* Maximum: (SLAVE_RX_BUFF_SIZE - 4) byte msg + 4 byte CRC */
        if (i2c_msg.len >= 5 && i2c_msg.len <= SLAVE_RX_BUFF_SIZE) 
        {
          /* Valid size, send the ACK on the message length */
          i2c_msg.buffer[0] = ACK_MSG_LEN;
          HAL_I2C_Slave_Transmit_IT(&hi2c1, i2c_msg.buffer, 1);
          
          /* If we don't receive the message within this time, wait for a message
           length again and abandon the one we were waiting for */
          set_timer(&i2c_msg.timer, I2C_SLAVE_TIMEOUT);   
          
          /* Receive the actual message */
          HAL_I2C_Slave_Receive_IT(&hi2c1, i2c_msg.buffer, i2c_msg.len);
          i2c_state = I2C_PROCESS_RX;
        }
        else {
          /* Invalid size, send the NACK on the message length */
          i2c_msg.buffer[0] = NACK_MSG_LEN;
          HAL_I2C_Slave_Transmit_IT(&hi2c1, i2c_msg.buffer, 1);

          i2c_msg.msg_len_rx = false;
          
          /* Wait to receive a byte over i2c specifying the amount of incoming bytes */
          HAL_I2C_Slave_Receive_IT(&hi2c1, i2c_msg.buffer, 1);
        }
        
        break;
      }
    }
    case I2C_PROCESS_RX: {
      if (i2c_msg.msg_rx) {
        
      }
      else if (timeout(&i2c_msg.timer)) {
        
      }
      break;
    }
    default: {
      /* Should never enter this, but if we do assume the worst and re-init */
      i2c_state = I2C_INIT;
      break;
    }
  }
}

void jetson_rx_handle(void) {
  /* Did we receive a message length or a message */
  if (!i2c_msg.msg_len_rx) {
    i2c_msg.msg_len_rx = true;
  }
  /* message length is marked as received so we must have received the actual message */
  else {
    i2c_msg.msg_rx = true;
  }
}

/*-----------------------------------------------------------------------
- Private Functions
-----------------------------------------------------------------------*/

