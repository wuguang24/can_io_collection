/*
 * 主机标准帧 IO（0x100/0x200/0x300）
 * 扩展帧 IAP 全部由 can2_bus.c / can_bus.c 处理，本文件不碰扩展帧。
 */
#include "stm32f10x_conf.h"
#include "stm32f10x_bkp.h"
#include "can.h"
#include "main.h"
#include "io_ctrl.h"

#define CAN_RECV_LEN 200
#define CAN_SEND_LEN 200

static can_frame_t can_recv_buf[CAN_CH_MAX][CAN_RECV_LEN];
static can_frame_t can_send_buf[CAN_CH_MAX][CAN_SEND_LEN];

queue_st can_recv_queue[CAN_CH_MAX];
queue_st can_send_queue[CAN_CH_MAX];
can_frame_t cur_send_frame[CAN_CH_MAX];
CAN_TypeDef* can_handle[CAN_CH_MAX] = {CAN1, CAN2};
u8 can_send_channel = CAN_CH_0;

static void can_msg_process(u8 ch, can_frame_t* frame);
static void can_send_queue_drain(u8 ch);

/* 与参考 can2_bus.c 一致：BKP=0x55 后复位进 Boot */
static void can_std_boot_jump(void)
{
    BKP_WriteBackupRegister(BKP_DR8, 0x55);
    NVIC_SystemReset();
}

static CAN_TypeDef* can_get_fwd_handle(u8 recv_ch)
{
    return (recv_ch == CAN_CH_0) ? CAN2 : CAN1;
}

u8 can_std_bridge_tx(u8 recv_ch, CanRxMsg* rx)
{
    CAN_TypeDef* fwd;
    u8 mb;

    fwd = can_get_fwd_handle(recv_ch);
    if((fwd->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) == 0)
    {
        return 0;
    }

    mb = CAN_Transmit(fwd, (CanTxMsg*)rx);
    return (mb != CAN_NO_MB) ? 1 : 0;
}

void can_init(void)
{
    u8 i;

    for(i = 0; i < CAN_CH_MAX; i++)
    {
        queue_init(&can_send_queue[i], (u8*)can_send_buf[i], CAN_SEND_LEN, sizeof(can_frame_t));
        queue_init(&can_recv_queue[i], (u8*)can_recv_buf[i], CAN_RECV_LEN, sizeof(can_frame_t));
    }
}

void can_send_frame(u8 ch, can_frame_t* frame)
{
    CanTxMsg txmsg;
    u8 i;

    if(can_handle[ch]->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2))
    {
        cur_send_frame[ch] = *frame;
        txmsg.StdId  = frame->id;
        txmsg.ExtId  = 0;
        txmsg.IDE    = CAN_ID_STD;
        txmsg.RTR    = CAN_RTR_DATA;
        txmsg.DLC    = frame->len > 8 ? 8 : frame->len;

        for(i = 0; i < txmsg.DLC; i++)
        {
            txmsg.Data[i] = frame->data[i];
        }

        CAN_Transmit(can_handle[ch], &txmsg);
    }
    else
    {
        queue_in(&can_send_queue[ch], (u8*)frame, 1);
    }
}

void can_recv_frame(u8 ch, CanRxMsg* RxMsg)
{
    u8 i;
    can_frame_t frame;

    frame.id = RxMsg->StdId;
    frame.len = RxMsg->DLC;
    for(i = 0; i < RxMsg->DLC; i++)
    {
        frame.data[i] = RxMsg->Data[i];
    }

    queue_in(&can_recv_queue[ch], (u8*)&frame, 1);
}

void can_recv_process(void)
{
    u8 i;
    u16 len;
    can_frame_t frame;

    for(i = 0; i < CAN_CH_MAX; i++)
    {
        while(queue_get_data_size(&can_recv_queue[i]) != 0)
        {
            len = 1;
            queue_out(&can_recv_queue[i], (u8*)&frame, &len);
            can_msg_process(i, &frame);
        }
    }
}

static void can_send_queue_drain(u8 ch)
{
    can_frame_t frm;
    u16 len;

    while((can_handle[ch]->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) != 0
          && queue_get_data_size(&can_send_queue[ch]) != 0)
    {
        len = 1;
        if(!queue_out(&can_send_queue[ch], (u8*)&frm, &len))
        {
            break;
        }
        can_send_frame(ch, &frm);
    }
}

static void can_msg_process(u8 ch, can_frame_t* frame)
{
    if(frame->id == S_M_CTRL_ID(Local_Station))
    {
        if(Local_Station < 2)
        {
            return;
        }
        if(frame->len >= 2)
        {
            io_ctrl_out = (u16)frame->data[0] + ((((u16)frame->data[1]) << 8) & 0xff00);
        }
        return;
    }

    switch(frame->id)
    {
    case M_HEAR_BEAT_ID:
        if(frame->len >= 1 && frame->data[0] == 0x01)
        {
            can_send_channel = (ch == CAN_CH_0) ? CAN_CH_0 : CAN_CH_1;
        }
        break;
    case M_BOOT_JUMP_CMD_ID:
        can_std_boot_jump();
        break;
    default:
        break;
    }
}

void can_send_process(void)
{
    static u16 data_cnt, heart_cnt;
    can_frame_t frame;
    static u8 io_in_old;

    if(io_in_old == io_ctrl_in)
    {
        if(data_cnt < 200)
        {
            data_cnt++;
        }
        else
        {
            data_cnt = 0;
            frame.id = S_IO_INFO_ID(Local_Station);
            frame.len = 1;
            frame.data[0] = io_ctrl_in;
            can_send_frame(can_send_channel, &frame);
        }
    }
    else
    {
        io_in_old = io_ctrl_in;
        data_cnt = 0;
        frame.id = S_IO_INFO_ID(Local_Station);
        frame.len = 1;
        frame.data[0] = io_ctrl_in;
        can_send_frame(can_send_channel, &frame);
    }

    if(heart_cnt < 3000)
    {
        heart_cnt++;
    }
    else
    {
        heart_cnt = 0;
        frame.id = S_HEART_INFO_ID(Local_Station);
        frame.len = 1;
        frame.data[0] = 0x01;
        can_send_frame(can_send_channel, &frame);
    }

    can_send_queue_drain(CAN_CH_0);
    can_send_queue_drain(CAN_CH_1);
}
