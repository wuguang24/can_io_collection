#ifndef _I2C_H
#define _I2C_H

#include "stm32f10x.h"
#include <inttypes.h>

#define I2C_WR  0
#define I2C_RD  1

/* Open-drain bitbang: use BSRR/BRR/IDR for stable timing */
#define I2C_SCL_1()     do { GPIOB->BSRR = GPIO_Pin_10; } while(0)
#define I2C_SCL_0()     do { GPIOB->BRR  = GPIO_Pin_10; } while(0)
#define I2C_SDA_1()     do { GPIOB->BSRR = GPIO_Pin_11; } while(0)
#define I2C_SDA_0()     do { GPIOB->BRR  = GPIO_Pin_11; } while(0)
#define I2C_SDA_READ()  ((GPIOB->IDR & GPIO_Pin_11) != 0)
#define I2C_SCL_READ()  ((GPIOB->IDR & GPIO_Pin_10) != 0)

void delay_us(u16 us);
void i2c_Delay(void);
void i2c_Start(void);
void i2c_Stop(void);
void i2c_SendByte(u8 _ucByte);
u8 i2c_ReadByte(void);
u8 i2c_WaitAck(void);
void i2c_Ack(void);
void i2c_NAck(void);
u8 i2c_CheckDevice(u8 _Address);

#endif
