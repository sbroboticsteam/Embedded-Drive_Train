/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define THROTTLE_L_Pin GPIO_PIN_0
#define THROTTLE_L_GPIO_Port GPIOC
#define THROTTLE_R_Pin GPIO_PIN_1
#define THROTTLE_R_GPIO_Port GPIOC
#define MTR0_PWM_Pin GPIO_PIN_0
#define MTR0_PWM_GPIO_Port GPIOA
#define MTR1_PWM_Pin GPIO_PIN_1
#define MTR1_PWM_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define MTR1_DIR_Pin GPIO_PIN_5
#define MTR1_DIR_GPIO_Port GPIOA
#define MTR1_A_Pin GPIO_PIN_6
#define MTR1_A_GPIO_Port GPIOA
#define MTR1_B_Pin GPIO_PIN_7
#define MTR1_B_GPIO_Port GPIOA
#define MTR3_PWM_Pin GPIO_PIN_2
#define MTR3_PWM_GPIO_Port GPIOB
#define MTR2_PWM_Pin GPIO_PIN_10
#define MTR2_PWM_GPIO_Port GPIOB
#define MTR2_DIR_Pin GPIO_PIN_15
#define MTR2_DIR_GPIO_Port GPIOB
#define MTR2_A_Pin GPIO_PIN_6
#define MTR2_A_GPIO_Port GPIOC
#define MTR2_B_Pin GPIO_PIN_7
#define MTR2_B_GPIO_Port GPIOC
#define MTR3_DIR_Pin GPIO_PIN_9
#define MTR3_DIR_GPIO_Port GPIOC
#define MTR3_A_Pin GPIO_PIN_8
#define MTR3_A_GPIO_Port GPIOA
#define MTR3_B_Pin GPIO_PIN_9
#define MTR3_B_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define MTR0_DIR_Pin GPIO_PIN_5
#define MTR0_DIR_GPIO_Port GPIOB
#define MTR0_B_Pin GPIO_PIN_6
#define MTR0_B_GPIO_Port GPIOB
#define MTR0_A_Pin GPIO_PIN_7
#define MTR0_A_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
