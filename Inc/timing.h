/*-----------------------------------------------------------------------
-   File Name:	  timing.h
-   Description:  Timing functionality for general timers
-   Date:	  1/14/2019
-   Name:	  John Boccio
- Stony Brook University Robotics
-----------------------------------------------------------------------*/

#ifndef __TIMING_H
#define __TIMING_H

/*-----------------------------------------------------------------------
- Public Includes
-----------------------------------------------------------------------*/
#include <stdbool.h>
#include "stm32f4xx_hal.h"
/*-----------------------------------------------------------------------
- Public Defines & Macros
-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------
- Public Typedefs & Enumerations
-----------------------------------------------------------------------*/
typedef struct {
  uint32_t expire_time;
  uint8_t enabled;
} timer_t;
/*-----------------------------------------------------------------------
- Public External References
-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------
- Public Function Prototypes
-----------------------------------------------------------------------*/
void set_timer(timer_t *timer, uint32_t duration);
void stop_timer(timer_t *timer);
bool timer_running(timer_t *timer);
bool timeout(timer_t *timer);

#endif /* __TIMING_H */
