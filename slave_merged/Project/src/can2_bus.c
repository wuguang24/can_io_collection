#include "main.h"
#include "flash.h"
#include "io.h"
#include "pin_cfg.h"
#include "slave_cfg_store.h"
#include "port_cos.h"
#include "fault_led.h"
#include "station_learn.h"

static u16 slave_cfg_crc16(const u8 *data, u16 len)
{
    u16 crc = 0xFFFF;
    u16 i, j;
    for(i = 0; i < len; i++)
    {
        crc ^= ((u16)data[i] << 8);
        for(j = 0; j < 8; j++)
        {
            if(crc & 0x8000) crc = (u16)((crc << 1) ^ 0x1021);
            else crc <<= 1;
        }
    }
    return crc;
}

static void slave_send_cfg_ack(u8 kind, u8 result)
{
    u8 buff[4];
    buff[0] = kind;
    buff[1] = result;
    buff[2] = 0;
    buff[3] = 0;
    vcan_sendmsg_two(buff, 4, CAN_FUNC_ID_CFG_ACK, 1);
    vcan_sendmsg_one(buff, 4, CAN_FUNC_ID_CFG_ACK, 1);
}

/* 上电/无表时主动上报 PIN 状态：result 0=有表 1=无PIN */
void slave_cfg_announce_pin(void)
{
    slave_send_cfg_ack(3, (g_pin_ready && g_pin_n > 0) ? 0 : 1);
}

/* 无 PIN 时周期公告，避免上电帧丢失后主机一直不知情 */
void slave_cfg_announce_pin_periodic(void)
{
    if(g_pin_ready && g_pin_n > 0)
    {
        return;
    }
    slave_cfg_announce_pin();
}

// can msg
u32 port_enable_state = 0;
u32 port_print_state = 0;
u32 port_print_state_bk = 0;
u32 slot_full_state = 0;
u32 pack_full_state = 0;
u32 full_tier1_state = 0;
u32 full_tier2_state = 0;
u32 port_door_state = 0;
u16 port_block_state = 0;

u8  frame_count = 1;
u8  frame_count_bk = 0;

u16 manual_flag =0;
u16 manual_operation =0;

sAlarmCtrl alarm_ctrl[PORT_NUM];

u8 upstream_can = 0;
u8 downstream_can = 0;

#define  canframebuffsize_two   70
sCanFrameExt    canframequeuebuff_two[canframebuffsize_two];
sCanFrameQueue   canframequeue_two;

sCanRecv canRecv[2];


void vcan_sendmsg_two(u8* buff, u16 send_total_len, u8 type, u8 dst);
void vcan_sendmsg_one(u8* buff, u16 send_total_len, u8 type, u8 dst);

static u8 slave_boot_sum(const u8 *buf, u8 len)
{
    u8 s = 0;
    u8 i;
    for(i = 0; i < len; i++)
    {
        s = (u8)(s + buf[i]);
    }
    return s;
}

/* 回 ENTER_BOOT 应答 → 排空 EXT 转发队列 → 写 BKP 复位进 Boot */
static void slave_app_enter_boot(u8 can_channel)
{
    sCanFrameExt ack;
    u16 i;
    u32 tme = CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2;

    ack.extId.seg_polo = CAN_SEG_POLO_NONE;
    ack.extId.seg_num = 0;
    ack.extId.func_id = CAN_FUNC_ID_BOOT_MODE;
    ack.extId.src_id = Local_Station;
    ack.extId.dst_id = 1;
    ack.data_len = 5;
    ack.data[0] = 0x55;
    ack.data[1] = 0x55;
    ack.data[2] = Local_Station;
    ack.data[3] = DLOAD_CMD_ENTER_BOOT;
    ack.data[4] = slave_boot_sum(ack.data, 4);
    if(can_channel == 0)
    {
        vcanbus_addto_cansendqueue_one(ack);
    }
    else
    {
        vcanbus_addto_cansendqueue_two(ack);
    }

    for(i = 0; i < 800; i++)
    {
        vcan_sendframe_process_one();
        vcan_sendframe_process_two();
        if(canframequeue_one.front == canframequeue_one.rear
           && canframequeue_two.front == canframequeue_two.rear)
        {
            break;
        }
    }
    for(i = 0; i < 50000; i++)
    {
        if(((CAN1->TSR & tme) == tme) && ((CAN2->TSR & tme) == tme))
        {
            break;
        }
    }

    BKP_WriteBackupRegister(BKP_DR8, 0x55);
    NVIC_SystemReset();
}

void vcanbus_initcansendqueue_two(void)
{
    sCanFrameQueue* q = NULL;
    q = &canframequeue_two;

    q->queue = canframequeuebuff_two;
    q->front = q->rear = 0;
    q->maxsize = canframebuffsize_two;
}

void vcanbus_addto_cansendqueue_two(sCanFrameExt x)
{
    sCanFrameQueue* q = NULL;
    q = &canframequeue_two;

    if ((q->rear + 1) % q->maxsize == q->front)
    {
        return;
    }

    q->rear = (q->rear + 1) % q->maxsize;

    q->queue[q->rear] = x;
}


u8 u8canbus_send_twoframe_two(sCanFrameExt sTxMsg)
{
    CanTxMsg TxMessage;
    u16 i =0;

    TxMessage.ExtId = (sTxMsg.extId.src_id & 0x7F) | ((sTxMsg.extId.func_id & 0x1F) << 7) | ((sTxMsg.extId.seg_num & 0xFF) << 12) | ((sTxMsg.extId.seg_polo & 0x3) << 20) | ((sTxMsg.extId.dst_id & 0x7F) << 22);
    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = sTxMsg.data_len;
    for(i=0;i<sTxMsg.data_len;i++){
      TxMessage.Data[i] = sTxMsg.data[i];
    }

    return CAN_Transmit(CAN2, &TxMessage);
}

void slave_recv_process(u8 can_channel, u8 func_id)
{
    u8 i,addr;
    u16 *point;
    u16 para_addr;

    addr = can_channel;

    if (func_id == CAN_FUNC_ID_SCREEN_DATA)
    {
        scrData.type = canRecv[addr].recv_buff[0];//1.2.3.4
        scrData.port_io = canRecv[addr].recv_buff[1];

        scrData.scr_buf[(scrData.port_io - 1)*4 + (scrData.type - 1)][0] = canRecv[addr].recv_len - 1;

        for (i = 0; i < canRecv[addr].recv_len - 1; i++)
        {
            scrData.scr_buf[(scrData.port_io - 1)*4 + scrData.type - 1][i + 1] = canRecv[addr].recv_buff[i + 1];
        }

        //屏幕数据保存flash- T260413
        //保存flash
        para_addr = FLASH_PARA1_NUM + FLASH_PARA2_NUM;
        if(memcmp(&scrData.scr_buf, &slave_recv_para.data[para_addr + 1], (FLASH_PARA3_NUM - 1)) != 0)
        {
            memcpy(&slave_recv_para.data[para_addr + 1], &scrData.scr_buf, FLASH_PARA3_NUM - 1);
            slave_recv_para.data[para_addr] = FLASH_FLAG_3;
            slave_recv_para.updateWait = 3000;
        }

    }else if (func_id == CAN_FUNC_ID_INIT_FUNC_DATA)
    {
        if (canRecv[addr].recv_len != 2)
        {
            slave_send_cfg_ack(1, 2);
            return;
        }
        bfunc.use_mode = FUNC_PORT;
        bfunc.same_group_f2_sta = canRecv[addr].recv_buff[1];
        if (((canRecv[addr].recv_buff[0] >> 7) & 0x01) == 1)
        {
            bfunc.local_block_group = BLOCK_GROUP_TWO;
        }
        else
        {
            bfunc.local_block_group = BLOCK_GROUP_ONE;
        }
        bfunc.same_group_f1_total = 0;
        memset(bfunc.same_group_f1_group, 0, sizeof(bfunc.same_group_f1_group));

        if(!slave_cfg_commit_func((const u8 *)&bfunc))
        {
            slave_send_cfg_ack(1, 3);
            return;
        }
        slave_send_cfg_ack(1, 0);
    }else if (func_id == CAN_FUNC_ID_INIT_TIME_DATA)
    {
        u16 calc, rxcrc;
        point = (u16 *)&canRecv[addr].recv_buff[0];
        if(canRecv[addr].recv_len < 12)
        {
            slave_send_cfg_ack(2, 2);
            return;
        }
        calc = slave_cfg_crc16(canRecv[addr].recv_buff, 10);
        rxcrc = (u16)(canRecv[addr].recv_buff[10] | (canRecv[addr].recv_buff[11] << 8));
        if(calc != rxcrc)
        {
            slave_send_cfg_ack(2, 2);
            return;
        }
        timfunc.lock_filter_time = point[0];
        timfunc.full_filter_time = point[1];
        timfunc.block_filter_time = point[2];
        timfunc.door_filter_time = point[3];
        timfunc.solt_filter_time = point[4];
        pin_cfg_on_time_updated();

        if(!slave_cfg_commit_time((const u8 *)&timfunc))
        {
            slave_send_cfg_ack(2, 3);
            return;
        }
        slave_send_cfg_ack(2, 0);
    }
    else if (func_id == CAN_FUNC_ID_INIT_PIN_DATA)
    {
        u16 body_len;
        u16 calc, rxcrc;
        u8 result = 0;

        if(canRecv[addr].recv_len < 5)
        {
            slave_send_cfg_ack(3, 2);
            return;
        }
        body_len = (u16)(canRecv[addr].recv_len - 2);
        calc = slave_cfg_crc16(canRecv[addr].recv_buff, body_len);
        rxcrc = (u16)(canRecv[addr].recv_buff[body_len] | (canRecv[addr].recv_buff[body_len + 1] << 8));
        if(calc != rxcrc)
        {
            slave_send_cfg_ack(3, 2);
            return;
        }
        if(pin_cfg_apply(canRecv[addr].recv_buff, body_len) != 0)
        {
            slave_send_cfg_ack(3, 5);
            return;
        }
        port_cos_reset();
        if(!slave_cfg_commit_pin(canRecv[addr].recv_buff, canRecv[addr].recv_len))
        {
            result = 3;
        }
        else if(!(g_pin_ready && g_pin_n > 0))
        {
            /* 应用成功但表为空：告知主机无 PIN（主机若下发的就是空切片则视为预期） */
            result = 1;
        }
        /* 非空表写入成功：覆盖先前绑定，完成重绑 */
        slave_send_cfg_ack(3, result);
    }
}

void can_bus_frame_receive(CanRxMsg rxMsg, u8 can_channel)
{
    u16 i = 0;
    sCanFrameExt canTxMsg;


    canTxMsg.extId.dst_id = rxMsg.ExtId >> 22 & 0x7F;
    canTxMsg.extId.src_id = rxMsg.ExtId & 0x7F;
    canTxMsg.extId.func_id = (rxMsg.ExtId >> 7) & 0x1F;
    canTxMsg.extId.seg_num = (rxMsg.ExtId >> 12) & 0xFF;
    canTxMsg.extId.seg_polo = (rxMsg.ExtId >> 20) & 0x3;

    canTxMsg.data_len = rxMsg.DLC;

    for (i = 0; i < canTxMsg.data_len; i++) {
        canTxMsg.data[i] = rxMsg.Data[i];
    }

    fault_led_on_can_rx(canTxMsg.extId.src_id, canTxMsg.extId.dst_id);

    if(canTxMsg.extId.func_id == CAN_FUNC_ID_STATION_LEARN)
    {
        if(station_learn_on_frame(can_channel, canTxMsg.extId.src_id,
                                  canTxMsg.extId.dst_id,
                                  canTxMsg.data, canTxMsg.data_len))
        {
            return;
        }
    }

    if(upstream_can == 0 && canTxMsg.extId.func_id == CAN_FUNC_ID_PORT_STATE)
    {
        upstream_can = can_channel + 1;
        downstream_can = (can_channel == 0) ? 2 : 1;
    }
    
    if ((canTxMsg.extId.dst_id != Local_Station) || (canTxMsg.extId.dst_id == 0x7F))
    {
        if(upstream_can != 0 && downstream_can != 0)
        {
            if(can_channel == (upstream_can - 1))
            {
                if(downstream_can == 1)
                    vcanbus_addto_cansendqueue_one(canTxMsg);
                else
                    vcanbus_addto_cansendqueue_two(canTxMsg);
            }
            else
            {
                if(upstream_can == 1)
                    vcanbus_addto_cansendqueue_one(canTxMsg);
                else
                    vcanbus_addto_cansendqueue_two(canTxMsg);
            }
        }
        else
        {
            if(can_channel == 0)
                vcanbus_addto_cansendqueue_two(canTxMsg);
            else
                vcanbus_addto_cansendqueue_one(canTxMsg);
        }
    }
    
    if(canTxMsg.extId.dst_id == 0x7F){
        if (canTxMsg.extId.func_id == CAN_FUNC_ID_MANUAL_STATUS){
            if((canTxMsg.data[0]|(canTxMsg.data[1]<<8))==0){
            manual_flag = MANUAL_VALID;
            manual_operation = OPERATION_VALID;
            }
            else if((canTxMsg.data[0]|(canTxMsg.data[1]<<8))==1){
            manual_flag = MANUAL_INVAILD;
            manual_operation = OPERATION_VALID; 
            }
        }
        
    }

    if (canTxMsg.extId.dst_id != Local_Station && canTxMsg.extId.dst_id != 0x7F) return;
    
    if (canTxMsg.extId.func_id == CAN_FUNC_ID_BOOT_MODE)
    {
        /*
         * APP 只响应单帧 ENTER_BOOT（AA AA sta B0 ...）。
         * 擦写/传包同属 BOOT_MODE，必须留给已在 Boot 的程序处理；
         * 见帧就复位会导致：第一次只进 Boot、会话失败，第二次才能升成功。
         * 非本站帧已在上方入队转发；此处复位前再排空队列。
         */
        if(canTxMsg.extId.seg_polo == CAN_SEG_POLO_NONE
           && canTxMsg.data_len >= 4
           && canTxMsg.data[0] == 0xAA
           && canTxMsg.data[1] == 0xAA
           && canTxMsg.data[3] == DLOAD_CMD_ENTER_BOOT)
        {
            slave_app_enter_boot(can_channel);
        }
        return;
    }
    else if (canTxMsg.extId.func_id == CAN_FUNC_ID_PORT_STATE)
    {
        port_state_ctrl((u8*)(&rxMsg.Data[0]));

        /* 慢同步须始终回 COS CONF（主机判在线）；打印另发，不可互斥挡住 CONF */
        if(canTxMsg.data[0] & PORT_STATE_FLAG_SYNC)
        {
            port_cos_force_sync_reply();
        }
        if(port_print_state != 0)
        {
            can_bus_send_port_print_state();
        }
    }
    else if (canTxMsg.extId.func_id == CAN_FUNC_ID_PRINT_STATE)
    {
        frame_count++;
        port_print_state &= ~port_print_state_bk;
    }
    else if (canTxMsg.extId.func_id == CAN_FUNC_ID_LOCK_UNLOCK)
    {
        slave_recv_lock_unlock(canTxMsg.data, canTxMsg.data_len);
    }
    else if (canTxMsg.extId.func_id == CAN_FUNC_ID_ALARM_CTRL)
    {
        slave_recv_alarm_ctrl(canTxMsg.data, canTxMsg.data_len);
    }
    else if (canTxMsg.extId.func_id == CAN_FUNC_ID_SCREEN_DATA || canTxMsg.extId.func_id == CAN_FUNC_ID_INIT_FUNC_DATA || 
        canTxMsg.extId.func_id == CAN_FUNC_ID_INIT_TIME_DATA || canTxMsg.extId.func_id == CAN_FUNC_ID_INIT_PIN_DATA)
    {
        if (canTxMsg.extId.seg_polo == CAN_SEG_POLO_NONE)
        {
            memcpy(canRecv[can_channel].recv_buff, canTxMsg.data, canTxMsg.data_len);
            canRecv[can_channel].recv_len = canTxMsg.data_len;
            slave_recv_process(can_channel, canTxMsg.extId.func_id);

        }else if (canTxMsg.extId.seg_polo == CAN_SEG_POLO_FIRST){
            memcpy(canRecv[can_channel].recv_buff, canTxMsg.data, canTxMsg.data_len);

            canRecv[can_channel].g_SegPolo = CAN_SEG_POLO_FIRST;
            canRecv[can_channel].g_SegNum = canTxMsg.extId.seg_num;
            canRecv[can_channel].g_SegBytes = canTxMsg.data_len;


        }else if (canTxMsg.extId.seg_polo == CAN_SEG_POLO_MIDDLE){
            if (canRecv[can_channel].g_SegPolo == CAN_SEG_POLO_FIRST 
                &&  (canRecv[can_channel].g_SegNum + 1) == canTxMsg.extId.seg_num
                &&  (canRecv[can_channel].g_SegBytes + canTxMsg.data_len)<= CAN_RX_BUFF_SIZE)

            {
                canRecv[can_channel].g_SegNum = canTxMsg.extId.seg_num;
                memcpy(canRecv[can_channel].recv_buff + canRecv[can_channel].g_SegBytes, canTxMsg.data, canTxMsg.data_len);
                canRecv[can_channel].g_SegBytes += canTxMsg.data_len;
            }
        }else if (canTxMsg.extId.seg_polo == CAN_SEG_POLO_FINAL){
            if (canRecv[can_channel].g_SegPolo == CAN_SEG_POLO_FIRST
                &&  (canRecv[can_channel].g_SegNum + 1) == canTxMsg.extId.seg_num
                &&  (canRecv[can_channel].g_SegBytes + canTxMsg.data_len)<= CAN_RX_BUFF_SIZE)

            {
                memcpy(canRecv[can_channel].recv_buff + canRecv[can_channel].g_SegBytes, canTxMsg.data, canTxMsg.data_len);
                canRecv[can_channel].recv_len = canRecv[can_channel].g_SegBytes + canTxMsg.data_len;
                slave_recv_process(can_channel, canTxMsg.extId.func_id);

                canRecv[can_channel].g_SegPolo = CAN_SEG_POLO_NONE;
                canRecv[can_channel].g_SegNum = 0;
                canRecv[can_channel].g_SegBytes = 0;
            }
        }

    }

}


void vcan_sendmsg_two(u8* buff, u16 send_total_len, u8 type, u8 dst)
{
    sCanFrameExt canTxMsg;
    u16 send_len = 0;
    u16 i =0;

    canTxMsg.extId.func_id = type;
    canTxMsg.extId.src_id = Local_Station;
    canTxMsg.extId.dst_id = dst;

    if (send_total_len <= CAN_PACK_DATA_LEN)
    {
        canTxMsg.extId.seg_polo = CAN_SEG_POLO_NONE;
        canTxMsg.extId.seg_num = 0;
        canTxMsg.data_len = send_total_len;
        for(i=0;i<canTxMsg.data_len;i++){
          canTxMsg.data[i] = buff[i];
        }  
        vcanbus_addto_cansendqueue_two(canTxMsg);
    }
    else {
        canTxMsg.extId.seg_polo = CAN_SEG_POLO_FIRST;
        canTxMsg.extId.seg_num = 0;
        canTxMsg.data_len = CAN_PACK_DATA_LEN;
        for(i=0;i<canTxMsg.data_len;i++){
          canTxMsg.data[i] = buff[i];
        } 
        vcanbus_addto_cansendqueue_two(canTxMsg);
        send_len += CAN_PACK_DATA_LEN;
        while (1) {
            if (send_len + CAN_PACK_DATA_LEN < send_total_len)
            {
                canTxMsg.extId.seg_polo = CAN_SEG_POLO_MIDDLE;
                canTxMsg.extId.seg_num++;
                canTxMsg.data_len = CAN_PACK_DATA_LEN;
                for(i=0;i<canTxMsg.data_len;i++){
                  canTxMsg.data[i] = buff[i + send_len];
                }                 
                vcanbus_addto_cansendqueue_two(canTxMsg);
                send_len += CAN_PACK_DATA_LEN;
            }
            else
            {
                canTxMsg.extId.seg_polo = CAN_SEG_POLO_FINAL;
                canTxMsg.extId.seg_num++;
                canTxMsg.data_len = send_total_len - send_len;
                for(i=0;i<canTxMsg.data_len;i++){
                  canTxMsg.data[i] = buff[i + send_len];
                }   
                vcanbus_addto_cansendqueue_two(canTxMsg);
                break;
            }
        }
    }
}

void vcan_sendframe_process_two(void)
{
    sCanFrameQueue* q = &canframequeue_two;
    sCanFrameExt* canTxMsg = NULL;
    u16 front = 0;
    u8  tx_mailbox = 0;

    if (q->front == q->rear)
    {
        return;
    }

    front = q->front;
    front = (front + 1) % q->maxsize;
    canTxMsg = (sCanFrameExt*)(&(q->queue[front]));
    tx_mailbox = u8canbus_send_twoframe_two(*canTxMsg);
    if (tx_mailbox != CAN_NO_MB)
    {
        q->front = front;
    }
}


//从机发送心跳
void can_bus_send_slave_heart_state(void)
{
    sCanFrameExt canTxMsg;

    canTxMsg.extId.func_id = CAN_FUNC_ID_HEART_STATE;
    canTxMsg.extId.src_id = Local_Station;
    canTxMsg.extId.dst_id = 1;
    canTxMsg.extId.seg_num = 0;
    canTxMsg.extId.seg_polo = CAN_SEG_POLO_NONE;

    canTxMsg.data_len = 8;
    canTxMsg.data[0] = 0;
    canTxMsg.data[1] = 0;
    canTxMsg.data[2] = 0;
    canTxMsg.data[3] = 0;
    canTxMsg.data[4] = 0;
    canTxMsg.data[5] = 0;
    canTxMsg.data[6] = 0;
    canTxMsg.data[7] = 0;
    if(upstream_can == 1)
        vcanbus_addto_cansendqueue_one(canTxMsg);
    else if(upstream_can == 2)
        vcanbus_addto_cansendqueue_two(canTxMsg);
    else
    {
        vcanbus_addto_cansendqueue_one(canTxMsg);
        vcanbus_addto_cansendqueue_two(canTxMsg);
    }
}


void can_bus_send_port_print_state(void)
{
    sCanFrameExt canTxMsg;

    if (frame_count_bk != frame_count)
    {
        frame_count_bk = frame_count;
        port_print_state_bk = port_print_state;
    }

    canTxMsg.extId.func_id = CAN_FUNC_ID_PRINT_STATE;
    canTxMsg.extId.src_id = Local_Station;
    canTxMsg.extId.dst_id = 1;
    canTxMsg.extId.seg_num = 0;
    canTxMsg.extId.seg_polo = CAN_SEG_POLO_NONE;
    canTxMsg.data_len = 4;
    canTxMsg.data[0] = frame_count;
    canTxMsg.data[1] = port_print_state_bk & 0xFF;
    canTxMsg.data[2] = (port_print_state_bk >> 8) & 0xFF;
    canTxMsg.data[3] = (port_print_state_bk >> 16) & 0xFF;
    if(upstream_can == 1)
        vcanbus_addto_cansendqueue_one(canTxMsg);
    else if(upstream_can == 2)
        vcanbus_addto_cansendqueue_two(canTxMsg);
    else
    {
        vcanbus_addto_cansendqueue_one(canTxMsg);
        vcanbus_addto_cansendqueue_two(canTxMsg);
    }
}

/* 全口锁/解锁（急停与 0xFFFF 广播共用）→ 仅更新远程关格位图 */
static void slave_apply_all_lock(u8 lock_on)
{
    io_remote_lock_all(lock_on ? 1u : 0u);
}

/*
 * func_id=0x9：
 *  A) 急停子命令：data[0]=0xA1, data[1]=0正常/1急停（主机 0x142B 转发）
 *  B) 锁格：port_io(u16 LE) + lock_state；port_io=0xFFFF 为全口
 */
void slave_recv_lock_unlock(u8* data, u8 len)
{
    u16 port_io;
    u8  lock_state;
    u8  i;

    if(data == NULL || len < 2)
    {
        return;
    }

    /* 0x142B → 主机广播：子命令 0xA1 + emergency(0/1) */
    if(data[0] == (u8)CAN_FUNC_ID_CTRL1_CMD)
    {
        slave_apply_all_lock(data[1] ? 1u : 0u);
        return;
    }

    if(len < 3)
    {
        return;
    }

    port_io = (u16)(data[0] | (data[1] << 8));
    lock_state = data[2];

    if(port_io == 0xFFFF)
    {
        slave_apply_all_lock(lock_state == 0x01 ? 1u : 0u);
    }
    else
    {
        i = (u8)port_io;
        if(i < PORT_NUM)
        {
            io_remote_lock_port(i, (lock_state == 0x01) ? 1u : 0u);
        }
    }
}

//协议0x1414: 通用声光报警接收处理
void slave_recv_alarm_ctrl(u8* data, u8 len)
{
    u16 port_io;
    u16 duration;
    u8  alarm_type;
    u8  i;
    
    if(len < 5) return;
    
    port_io = data[0] | (data[1] << 8);
    duration = data[2] | (data[3] << 8);
    alarm_type = data[4];
    
    if(port_io == 0xFFFF)
    {
        for(i = 0; i < PORT_NUM; i++)
        {
            alarm_ctrl[i].duration = duration;
            alarm_ctrl[i].alarm_type = alarm_type;
            alarm_ctrl[i].active = 1;
            alarm_ctrl[i].tick = 0;
        }
    }
    else
    {
        i = port_io;
        if(i < PORT_NUM)
        {
            alarm_ctrl[i].duration = duration;
            alarm_ctrl[i].alarm_type = alarm_type;
            alarm_ctrl[i].active = 1;
            alarm_ctrl[i].tick = 0;
        }
    }
}

//协议0x1501: 下料落格校验光电触发上报
void slave_send_photo_trigger(u16 port_io)
{
    u8 buff[2];
    buff[0] = port_io & 0xFF;
    buff[1] = (port_io >> 8) & 0xFF;
    if(upstream_can == 1)
        vcan_sendmsg_one(buff, 2, CAN_FUNC_ID_PHOTO_TRIGGER, 1);
    else if(upstream_can == 2)
        vcan_sendmsg_two(buff, 2, CAN_FUNC_ID_PHOTO_TRIGGER, 1);
    else
    {
        vcan_sendmsg_two(buff, 2, CAN_FUNC_ID_PHOTO_TRIGGER, 1);
        vcan_sendmsg_one(buff, 2, CAN_FUNC_ID_PHOTO_TRIGGER, 1);
    }
}

//协议0x1411: 夹件光电报警上报
void slave_send_block_alarm(u16 port_io)
{
    u8 buff[2];
    buff[0] = port_io & 0xFF;
    buff[1] = (port_io >> 8) & 0xFF;
    if(upstream_can == 1)
        vcan_sendmsg_one(buff, 2, CAN_FUNC_ID_BLOCK_ALARM, 1);
    else if(upstream_can == 2)
        vcan_sendmsg_two(buff, 2, CAN_FUNC_ID_BLOCK_ALARM, 1);
    else
    {
        vcan_sendmsg_two(buff, 2, CAN_FUNC_ID_BLOCK_ALARM, 1);
        vcan_sendmsg_one(buff, 2, CAN_FUNC_ID_BLOCK_ALARM, 1);
    }
}

//声光报警定时处理，1ms调用
void slave_alarm_ctrl_process(void)
{
    u8 i;
    for(i = 0; i < PORT_NUM; i++)
    {
        if(alarm_ctrl[i].active)
        {
            alarm_ctrl[i].tick++;
            if(alarm_ctrl[i].tick >= alarm_ctrl[i].duration * 1000)
            {
                alarm_ctrl[i].active = 0;
                alarm_ctrl[i].tick = 0;
                alarm_ctrl[i].alarm_type = 0;
            }
        }
    }
}
