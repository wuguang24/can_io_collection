/**
******************************************************************************
* @file    main.h
* @author  MCD Application Team
* @version V1.0.0
* @date    11/20/2009
* @brief   This file contains all the functions prototypes for the main.c
*          file.
******************************************************************************
* @copy
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

    /* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f107.h"
#include "fun.h"
#include "can_bus.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "flash.h"
#include "udpboardcast.h"
/* Exported function prototypes ----------------------------------------------*/
void Time_Update(void);
void Delay(uint32_t nCount);
void App_Lock(void);
void App_Unlock(void);
void CanIo_NotifyFromISR(long *pxHigherPriorityTaskWoken);
void Net_Notify(void);

/* ETHERNET errors */
#define  ETH_ERROR              ((uint32_t)0)
#define  ETH_SUCCESS            ((uint32_t)1)
#define  DP83848_PHY_ADDRESS       0x01

#define  MAX_SLAVE_NUM          96 /* 从机拨码站号 2~97 */
//#define  EACH_IP_PORT_OFFSET    1000//相邻ip的格口偏移量
//格口偏移: 按现场屏幕寻址规则配置（1422 仍用此宏，本文档不修改 1422 逻辑）
#define  PORT_OFFSET    1/* 相邻板卡格口偏移 */
#define BLOCK_GROUP_ONE 1
#define BLOCK_GROUP_TWO 2


/* 1426 已分段；768 单段 UDP，双缓冲*/
#define	TCP_RECEV_BUFF_SIZE		768
#define	CLIENT_DIS_CONNECT		0
#define	CLIENT_CONNECT_OK		1
#define	CLIENT_CONNECT_RECV		2
#define	CLIENT_RE_CONNECT		3

/*
 * USE_MAIN_H_IP — 与现场 V5 工程一致：
 * 打开后 flashReadNetPara() 用本文件 IP/DEST/端口覆盖 Boot Flash(0x0800F000) 网络区；
 * 关闭则完全使用 Flash 里保存的 IP（广播 3030 / 0x2F02 可改）。
 */
#define USE_MAIN_H_IP

#define DEST_IP_ADDR0   192
#define DEST_IP_ADDR1   168
#define DEST_IP_ADDR2   2
#define DEST_IP_ADDR3   100   /* 上位机/WCS */
#define DEST_PORT       9904

#define LOCAL_UDP_PORT  DEST_PORT
/* MAC ADDRESS: MAC_ADDR0:MAC_ADDR1:MAC_ADDR2:MAC_ADDR3:MAC_ADDR4:MAC_ADDR5 */
#define MAC_ADDR0   mac_addr[0]
#define MAC_ADDR1   mac_addr[1]
#define MAC_ADDR2   mac_addr[2]
#define MAC_ADDR3   mac_addr[3]
#define MAC_ADDR4   mac_addr[4]
#define MAC_ADDR5   mac_addr[5]
    /*Static IP ADDRESS: IP_ADDR0.IP_ADDR1.IP_ADDR2.IP_ADDR3 */
#define IP_ADDR0   192
#define IP_ADDR1   168
#define IP_ADDR2   2
#define IP_ADDR3   120   /* 板卡本机 */
    /*NETMASK*/
#define NETMASK_ADDR0   255
#define NETMASK_ADDR1   255
#define NETMASK_ADDR2   255
#define NETMASK_ADDR3   0
    /*Gateway Address*/
#define GW_ADDR0   192
#define GW_ADDR1   168
#define GW_ADDR2   2
#define GW_ADDR3   1

#define	 DIP1_STATE	        GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4)
#define	 DIP2_STATE	        GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5)
#define	 DIP3_STATE	        GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6)
#define	 DIP4_STATE	        GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7)
#define	 DIP5_STATE	        GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4)
#define	 DIP6_STATE	        GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0)
#define	 DIP7_STATE	        GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1)
#define	 DIP8_STATE	        GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_7)

#define	 LED_STATE		GPIO_ReadOutputDataBit(GPIOE, GPIO_Pin_2)
#define	 LED_ON			GPIO_ResetBits(GPIOE,GPIO_Pin_2)
#define	 LED_OFF		GPIO_SetBits(GPIOE,GPIO_Pin_2)

extern u8  heart_delay;
extern u8  ioCtrl_num;
extern u8  ext_host_flag;

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/

