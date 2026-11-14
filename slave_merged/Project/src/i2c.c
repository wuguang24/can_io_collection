#include "main.h"
#include "i2c.h"

/*
 * Soft-I2C bit delay via DWT CYCCNT (enabled in BSP CPU_TS_TmrInit).
 * Target ~100 kHz: half-period ~5 us. Empty for-loops are unreliable under IAR optimize.
 */

#ifndef I2C_BIT_HALF_US
#define I2C_BIT_HALF_US   5u
#endif

#ifndef I2C_STRETCH_TIMEOUT_US
#define I2C_STRETCH_TIMEOUT_US  1000u
#endif

extern uint32_t CPU_TS_TmrRd(void);

static u32 i2c_cycles_per_us(void)
{
    u32 hz = RCC_Clocks.SYSCLK_Frequency;
    if(hz < 1000000u)
    {
        hz = 72000000u;
    }
    return hz / 1000000u;
}

void delay_us(u16 us)
{
    u32 cycles;
    u32 start;
    u32 cpu_us;

    if(us == 0)
    {
        return;
    }
    cpu_us = i2c_cycles_per_us();
    if(cpu_us == 0)
    {
        cpu_us = 72u;
    }
    cycles = cpu_us * (u32)us;
    start = CPU_TS_TmrRd();
    while((CPU_TS_TmrRd() - start) < cycles)
    {
    }
}

void i2c_Delay(void)
{
    delay_us(I2C_BIT_HALF_US);
}

/* Wait until SCL is released by slave (clock stretch), OD bus */
static void i2c_wait_scl_high(void)
{
    u32 start = CPU_TS_TmrRd();
    u32 limit = i2c_cycles_per_us() * I2C_STRETCH_TIMEOUT_US;

    I2C_SCL_1();
    while(!I2C_SCL_READ())
    {
        if((CPU_TS_TmrRd() - start) >= limit)
        {
            break;
        }
    }
}

void i2c_Start(void)
{
    I2C_SDA_1();
    I2C_SCL_1();
    i2c_Delay();
    I2C_SDA_0();
    i2c_Delay();
    I2C_SCL_0();
    i2c_Delay();
}

void i2c_Stop(void)
{
    I2C_SDA_0();
    i2c_Delay();
    i2c_wait_scl_high();
    i2c_Delay();
    I2C_SDA_1();
    i2c_Delay();
}

void i2c_SendByte(u8 _ucByte)
{
    u8 i;

    for(i = 0; i < 8; i++)
    {
        if(_ucByte & 0x80)
        {
            I2C_SDA_1();
        }
        else
        {
            I2C_SDA_0();
        }
        i2c_Delay();
        i2c_wait_scl_high();
        i2c_Delay();
        I2C_SCL_0();
        if(i == 7)
        {
            I2C_SDA_1();
        }
        _ucByte <<= 1;
        i2c_Delay();
    }
}

u8 i2c_ReadByte(void)
{
    u8 i;
    u8 value = 0;

    I2C_SDA_1();
    for(i = 0; i < 8; i++)
    {
        value <<= 1;
        i2c_wait_scl_high();
        i2c_Delay();
        if(I2C_SDA_READ())
        {
            value++;
        }
        I2C_SCL_0();
        i2c_Delay();
    }
    return value;
}

u8 i2c_WaitAck(void)
{
    u8 re;

    I2C_SDA_1();
    i2c_Delay();
    i2c_wait_scl_high();
    i2c_Delay();
    re = I2C_SDA_READ() ? 1 : 0;
    I2C_SCL_0();
    i2c_Delay();
    return re;
}

void i2c_Ack(void)
{
    I2C_SDA_0();
    i2c_Delay();
    i2c_wait_scl_high();
    i2c_Delay();
    I2C_SCL_0();
    i2c_Delay();
    I2C_SDA_1();
}

void i2c_NAck(void)
{
    I2C_SDA_1();
    i2c_Delay();
    i2c_wait_scl_high();
    i2c_Delay();
    I2C_SCL_0();
    i2c_Delay();
}

u8 i2c_CheckDevice(u8 _Address)
{
    u8 ucAck;

    i2c_Start();
    i2c_SendByte((u8)((_Address << 1) | I2C_WR));
    ucAck = i2c_WaitAck();
    i2c_Stop();
    return ucAck;
}
