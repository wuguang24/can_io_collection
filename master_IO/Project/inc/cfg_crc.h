#ifndef _CFG_CRC_H_
#define _CFG_CRC_H_

#include "stm32f10x.h"

u16 cfg_crc16(const u8 *data, u16 len);
u16 cfg_crc16_continue(u16 crc, const u8 *data, u16 len);

#endif
