#include "flash.h"

#include "string.h"

#include "pin_cfg.h"

#include "slave_cfg_store.h"

#include "can_bus.h"



Slave_Msg  slave_recv_para;



static void slave_bfunc_set_defaults(void)

{

    bfunc.use_mode = FUNC_PORT;

    bfunc.same_group_f2_sta = 0;

    bfunc.local_block_group = BLOCK_GROUP_ONE;

    bfunc.same_group_f1_total = 0;

    memset(bfunc.same_group_f1_group, 0, sizeof(bfunc.same_group_f1_group));

}



void vflash_read_startup_item_para(void)

{

    u16 flag3;

    u16 *pScr;

    static u8 first_screen = 0;

    u8  show_buf1[40] = {0x0D,0x01,0x10,0x10,0x50,0x00,0x02,0x04,0x00,0x01,0xFF,0x00,0x2B,0x63};

    u8  show_buf2[40] = {0x0D,0x02,0x10,0x10,0x50,0x00,0x02,0x04,0x00,0x02,0xFF,0x00,0xD4,0x27};



    io_para_init_config();

    pin_cfg_load_from_flash();



    if((slave_cfg_applied_mask() & SLAVE_CFG_APPLIED_TIME) == 0

       && *(u16 *)TimeFlagAddr == FLASH_FLAG_2)

    {

        memcpy(&timfunc, (const void *)TimeParaAddr, sizeof(timfunc));

        pin_cfg_on_time_updated();

    }

    if((slave_cfg_applied_mask() & SLAVE_CFG_APPLIED_FUNC) == 0)

    {

        if(*(u16 *)UserParaAddr == FLASH_FLAG_1)

        {

            memcpy(&bfunc, (const void *)UserFuncAddr, sizeof(bfunc));

        }

        else

        {

            slave_bfunc_set_defaults();

        }

    }

    (void)slave_cfg_migrate_from_flash_pages();



    flag3 = *(u16 *)(ScreenFlagAddr);

    if(flag3 == FLASH_FLAG_3)

    {

        pScr = (u16 *)(ScreenParaAddr);

        slave_recv_para.data[FLASH_PARA1_NUM + FLASH_PARA2_NUM] = FLASH_FLAG_3;

        memcpy(&scrData.scr_buf, pScr, sizeof(scrData.scr_buf));

        memcpy(&slave_recv_para.data[FLASH_PARA1_NUM + FLASH_PARA2_NUM + 1],

               &scrData.scr_buf, sizeof(scrData.scr_buf));

    }

    else if(first_screen == 0)

    {

        first_screen = 1;

        memcpy(&scrData.scr_buf[0], show_buf1, sizeof(show_buf1));

        memcpy(&scrData.scr_buf[4], show_buf2, sizeof(show_buf1));

    }



    slave_cfg_announce_pin();

}



void FlashWriteContinue(uint32_t address, u16* out_point, u32 len)

{

    u32 i = 0;



    FLASH_Unlock();

    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    if(FLASH_ErasePage(address) == FLASH_COMPLETE)

    {

        for(i = 0; i < len; i++)

        {

            FLASH_ProgramHalfWord(address + 2 * i, *(out_point + i));

        }

    }

    FLASH_Lock();

}



void FlashReadContinue(uint32_t address, u16* out_point, u32 len)

{

    u16* source;

    u32 i;



    source = (u16*)(address);

    for(i = 0; i < len; i++)

    {

        *(out_point + i) = *(source + i);

    }

}



void flash_upload_para(void)

{

    if(slave_recv_para.updateWait != 0)

    {

        slave_recv_para.updateWait--;

        if(slave_recv_para.updateWait == 0)

        {

            flash_write_user_para();

            vflash_read_startup_item_para();

        }

    }

}



void flash_write_user_para(void)

{

    FlashWriteContinue(UserParaAddr, slave_recv_para.data, FLASH_PARA_MAXNUM);

}



u8 flash_write_pin_cfg(u8 *data, u16 len)

{

    if(data == NULL || len == 0 || len > PIN_CFG_MAX_BYTES)

    {

        return 1;

    }

    return slave_cfg_commit_pin(data, len) ? 0 : 1;

}

