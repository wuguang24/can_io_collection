#ifndef _FLASH_H
#define _FLASH_H

#include "main.h"

#define UserParaAddr        0x08028000
#define UserFuncAddr        (UserParaAddr  + 2)

#define TimeFlagAddr       (0x08028000 + 0x40)
#define TimeParaAddr       (TimeFlagAddr  + 2)

#define ScreenFlagAddr       (0x08028000 + 0x80)
#define ScreenParaAddr       (ScreenFlagAddr  + 2)

#define  FLASH_PARA1_NUM     32
#define  FLASH_PARA2_NUM     32
#define  FLASH_PARA3_NUM     (800 + 1)
#define FLASH_PARA_MAXNUM   (FLASH_PARA1_NUM + FLASH_PARA2_NUM + FLASH_PARA3_NUM)

#define FLASH_FLAG_1        0xA1
#define FLASH_FLAG_2        0xA2
#define FLASH_FLAG_3        0xA3
#define FLASH_FLAG_4        0xA4
/* 与 SLAVE_CFG_BANK_A 同址：新固件双区优先，无有效 bank 时才认 FLASH_FLAG_4 */
#define PinPageAddr         0x08028800
#define PIN_CFG_MAX_BYTES   256

#pragma pack (1)
typedef struct
{
    u16  data[FLASH_PARA_MAXNUM];
    u16  dataNum;
    u16  updateWait;
}Slave_Msg;
#pragma pack ()

extern Slave_Msg  slave_recv_para;

void flash_write_user_para(void); /* 仅屏幕等大页；FUNC/TIME/PIN 勿调用 */
void flash_upload_para(void);
void flash_read_para(void);
void vflash_read_startup_item_para(void);
u8 flash_write_pin_cfg(u8 *data, u16 len);

#endif
