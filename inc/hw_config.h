/**
  ******************************************************************************
  * @file    hw_config.h
  * @author  MCD Application Team
  * @version V2.0.0
  * @date    11-July-2011
  * @brief   Target config file module.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HARDWARE_CONFIG_H
#define __HARDWARE_CONFIG_H

#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "gd32f4xx.h"

#include "board.h"

//#include "variables.h"
//#include "AnalisRS.h"

#define OPPSysTick_IRQn             0x01
#define xOPPRTC_IRQn                 0x0c  //not exist here
#define OPPOTG_FS_IRQn               0x07
#define OPPOTG_HS_IRQn               0x08
#define OPPUSART2_IRQn             0x04
#define OPPUSART1_IRQn             0x03
#define OPPUSART3_IRQn               0x02
#define OPPTIM7_IRQn               0x0a
#define OPPEXTI9_5_IRQn           0x0c

#define EiN()  __set_PRIMASK(0);
#define DiN()  __set_PRIMASK(1);

#define PWM_AUTORELOAD_DEF_1200 1200

/* Exported macros -----------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
 void LL_Init(void);
 void SystemClock_Config(void);
 void MX_GPIO_Init(void);
 void MX_IWDG_Init(void);
 void MX_USART2_UART_Init(void);
 void MX_USART1_UART_Init(void);
 void MX_USART0_UART_Init(void);
 void ADC012_Init(void);
  void MX_TIM3_Init(void);
  void MX_TIM4_Init(void);
  void MX_CRC_Init(void);
  void InitEncoderInterfaceTim2PA0PA1(void);

  void EnableBkpSram(void);

  void _Error_Handler(char *, int);

  #define Error_Handler() _Error_Handler(__FILE__, __LINE__)


#ifdef __cplusplus
}
#endif

#endif /* __HARDWARE_CONFIG_H */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
