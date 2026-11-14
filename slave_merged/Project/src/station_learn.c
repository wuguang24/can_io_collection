#include "station_learn.h"
#include "can_bus.h"
#include "stm32f10x_bkp.h"
#include "stm32f10x_pwr.h"
#include "string.h"

extern u8 Local_Station;
extern u8 Local_Station1;
extern u8 Local_Station2;
extern u8 upstream_can;
extern u8 downstream_can;

static u8 s_dip_fixed;
static u8 s_learn_relay;
static u8 s_last_token;

static void stn_lrn_send_ext(u8 dst, const u8 *data, u8 len)
{
    sCanFrameExt canTxMsg;

    if(data == NULL || len == 0 || len > 8)
    {
        return;
    }
    canTxMsg.extId.func_id = CAN_FUNC_ID_STATION_LEARN;
    canTxMsg.extId.src_id = (Local_Station >= 2) ? Local_Station : 1;
    canTxMsg.extId.dst_id = dst;
    canTxMsg.extId.seg_num = 0;
    canTxMsg.extId.seg_polo = CAN_SEG_POLO_NONE;
    canTxMsg.data_len = len;
    memcpy(canTxMsg.data, data, len);
    if(upstream_can == 1)
    {
        vcanbus_addto_cansendqueue_one(canTxMsg);
    }
    else if(upstream_can == 2)
    {
        vcanbus_addto_cansendqueue_two(canTxMsg);
    }
    else
    {
        vcanbus_addto_cansendqueue_one(canTxMsg);
        vcanbus_addto_cansendqueue_two(canTxMsg);
    }
}

static void stn_lrn_forward(u8 can_channel, const u8 *data, u8 len)
{
    sCanFrameExt canTxMsg;
    u8 other;

    if(data == NULL || len == 0 || len > 8)
    {
        return;
    }
    if(upstream_can == 0 || downstream_can == 0)
    {
        return;
    }
    other = (can_channel == 0) ? downstream_can : upstream_can;
    canTxMsg.extId.func_id = CAN_FUNC_ID_STATION_LEARN;
    canTxMsg.extId.src_id = Local_Station;
    canTxMsg.extId.dst_id = 0x7F;
    canTxMsg.extId.seg_num = 0;
    canTxMsg.extId.seg_polo = CAN_SEG_POLO_NONE;
    canTxMsg.data_len = len;
    memcpy(canTxMsg.data, data, len);
    if(other == 1)
    {
        vcanbus_addto_cansendqueue_one(canTxMsg);
    }
    else
    {
        vcanbus_addto_cansendqueue_two(canTxMsg);
    }
}

void station_learn_save_station(u8 station)
{
    PWR_BackupAccessCmd(ENABLE);
    BKP_WriteBackupRegister(STN_LRN_BKP_DR_MAGIC, STN_LRN_BKP_MAGIC);
    BKP_WriteBackupRegister(STN_LRN_BKP_DR_STA, station);
    PWR_BackupAccessCmd(DISABLE);
}

void station_learn_init(void)
{
    s_dip_fixed = 0;
    s_learn_relay = 0;
    s_last_token = 0;
}

void station_learn_apply_dip(void)
{
    u16 magic;
    u16 sta;

    if(Local_Station1 != 0 || Local_Station2 != 0)
    {
        s_dip_fixed = 1;
        return;
    }
    s_dip_fixed = 0;
    PWR_BackupAccessCmd(ENABLE);
    magic = BKP_ReadBackupRegister(STN_LRN_BKP_DR_MAGIC);
    sta = BKP_ReadBackupRegister(STN_LRN_BKP_DR_STA);
    PWR_BackupAccessCmd(DISABLE);
    if(magic == STN_LRN_BKP_MAGIC && sta >= 2 && sta <= 97)
    {
        Local_Station = (u8)sta;
    }
    else
    {
        Local_Station = 0;
    }
}

u8 station_learn_on_frame(u8 can_channel, u8 src_id, u8 dst_id,
                            const u8 *data, u8 len)
{
    u8 buff[8];
    u8 station;

    if(data == NULL || len < 2)
    {
        return 0;
    }
    if(dst_id != Local_Station && dst_id != 0x7F)
    {
        return 0;
    }

    switch(data[0])
    {
    case STN_LRN_SUB_DISCOVER:
        if(s_dip_fixed)
        {
            return 0;
        }
        if(Local_Station != 0)
        {
            if(s_learn_relay)
            {
                stn_lrn_forward(can_channel, data, len);
            }
            return 1;
        }
        s_last_token = data[1];
        buff[0] = STN_LRN_SUB_ACK;
        buff[1] = data[1];
        buff[2] = 0;
        buff[3] = 0;
        stn_lrn_send_ext(1, buff, 4);
        return 1;

    case STN_LRN_SUB_ASSIGN:
        if(len < 3 || data[1] < 2)
        {
            return 0;
        }
        station = data[1];
        s_last_token = data[2];
        if(Local_Station != 0)
        {
            if(Local_Station != station)
            {
                if(s_learn_relay)
                {
                    stn_lrn_forward(can_channel, data, len);
                }
                return 1;
            }
        }
        else if(s_dip_fixed)
        {
            return 0;
        }
        else
        {
            Local_Station = station;
            station_learn_save_station(station);
        }
        buff[0] = STN_LRN_SUB_DONE;
        buff[1] = data[2];
        buff[2] = station;
        buff[3] = 0;
        stn_lrn_send_ext(1, buff, 4);
        return 1;

    case STN_LRN_SUB_PASS:
        if(Local_Station < 2 || data[1] != s_last_token)
        {
            return 0;
        }
        s_learn_relay = 1;
        return 1;

    default:
        break;
    }
    return 0;
}
