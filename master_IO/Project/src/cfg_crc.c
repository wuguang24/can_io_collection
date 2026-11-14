#include "cfg_crc.h"

/* CRC-16/CCITT-FALSE */
u16 cfg_crc16_continue(u16 crc, const u8 *data, u16 len)
{
    u16 i, j;

    for(i = 0; i < len; i++)
    {
        crc ^= ((u16)data[i] << 8);
        for(j = 0; j < 8; j++)
        {
            if(crc & 0x8000)
            {
                crc = (u16)((crc << 1) ^ 0x1021);
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

u16 cfg_crc16(const u8 *data, u16 len)
{
    return cfg_crc16_continue(0xFFFF, data, len);
}
