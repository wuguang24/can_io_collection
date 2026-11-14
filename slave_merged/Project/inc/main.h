/**
  ******************************************************************************
  * @file    main.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    07-October-2011 
  * @brief   This file contains all the functions prototypes for the main.c 
  *          file.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "io.h"
#include "can_bus.h"
#include "can.h"
#include "io_ctrl.h"
#include "queue.h"
#include <string.h>
#include <stdio.h>
#include "aw_i2c.h"
#include "i2c.h"
#include "lcd_uart.h"
#include "flash.h"
void Time_Update(void);
void Delay(uint32_t nCount);
void delay_us(u16 us);
void BSP_Init(void);
uint32_t CPU_TS_TmrRd(void);
void CPU_TS_TmrInit(void);

void io_open_all(void);

#define  PORT_OFFSET    4

#define  S1_STATE            (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_1) &0x01)
#define  S2_STATE            (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_2) &0x01)
#define  S3_STATE            (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3) &0x01)
#define  S4_STATE            (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4) &0x01)
#define  S5_STATE            (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5) &0x01)
#define  S6_STATE            (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6) &0x01)
#define  S7_STATE            (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7) &0x01)
#define  S8_STATE            (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4) &0x01)

#define  IN_7_STATE            (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_9) &0x01)
#define  IN_6_STATE            (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_8)&0x01)
#define  IN_5_STATE            (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9)&0x01)
#define  IN_4_STATE            (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8)&0x01)
#define  IN_3_STATE            (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7)&0x01)
#define  IN_2_STATE            (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6)&0x01)
#define  IN_1_STATE            (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)&0x01)
#define  IN_0_STATE            (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)&0x01)

#define  OUT_7_(BitVal)         GPIO_WriteBit(GPIOC,GPIO_Pin_12,BitVal)
#define  OUT_6_(BitVal)         GPIO_WriteBit(GPIOD,GPIO_Pin_2,BitVal)
#define  OUT_5_(BitVal)         GPIO_WriteBit(GPIOB,GPIO_Pin_3,BitVal)
#define  OUT_4_(BitVal)         GPIO_WriteBit(GPIOB,GPIO_Pin_4,BitVal)
#define  OUT_3_(BitVal)         GPIO_WriteBit(GPIOB,GPIO_Pin_5,BitVal)
#define  OUT_2_(BitVal)         GPIO_WriteBit(GPIOB,GPIO_Pin_6,BitVal)
#define  OUT_1_(BitVal)         GPIO_WriteBit(GPIOB,GPIO_Pin_7,BitVal)
#define  OUT_0_(BitVal)         GPIO_WriteBit(GPIOB,GPIO_Pin_8,BitVal)


#define	 UART4_RX_485	        GPIO_ResetBits(GPIOA,GPIO_Pin_15)
#define	 UART4_TX_485	        GPIO_SetBits(GPIOA,GPIO_Pin_15)

#define    USART4_BUFF_LEN      50
#define    USART4_MOTO_LEN      20

#define    USART4_RECV_TIMMER   10
#define    USART4_SEND_TIMMER   10

extern  RCC_ClocksTypeDef RCC_Clocks;
extern  u8  Local_Station;
extern  u32 test_timing[];
#endif /* __MAIN_H */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

