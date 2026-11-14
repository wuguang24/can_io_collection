#include "stm32f10x.h"
#include "stm32f107.h"
#include "main.h"
#include "io.h"
#include "cfg_table.h"
#include "cfg_logic.h"
#include "cfg_crc.h"
#include "cfg_can_ext.h"
#include "fun.h"
#include "station_learn.h"
#include "string.h"

/* 0x1402 组包缓冲；>256 时 fun.c 分片发送 */
u8 port_state_upload_buff[IOCTRL2WCS_UPLOAD_BODY_MAX];
static u16 s_upload_shadow_crc = 0;
static u16 s_upload_shadow_len = 0;
static u16 s_upload_heartbeat = 0;
#define UPLOAD_HEARTBEAT_TICKS  0u   /* 交叉带5.1：每500ms发1402；0=内容不变也发；>0同CRC降频 */
u16 g_port_state_upload_len = 0;
u8 print_state_upload_buff[8]; /* 0x1404：num(u16)+port(u16) */
u8 port_print_reply_state[MAX_SLAVE_NUM];
u32 port_close_state[MAX_SLAVE_NUM];
u8 port_close_flag[MAX_SLAVE_NUM];
u32 port_clear_state[MAX_SLAVE_NUM];
u8 port_clear_flag[MAX_SLAVE_NUM];
u8 close_cmd;
u8  last_close_cmd = 0;

sPortState portState[MAX_SLAVE_NUM];
u8 frame_count[MAX_SLAVE_NUM];
u8 poll_cnt = 0;

u8 slave_can_error_num = 0;
u8 upload_can_error[MAX_SLAVE_NUM] = {0};

sCanFrameExt    canframebuff[CAN_SEND_FRAME_LEN];
sCanFramequeue  canframequeue;

u8 full_tier_active = 0;

#define CAN_POLL_MS                 (5u)
#define PORT_STATE_STALE_MIN_MS     (500u)
#define PORT_STATE_STALE_SYNC_ROUNDS (4u)
#define COS_SYNC_DIV_TICKS          (COS_SYNC_PERIOD_MS / CAN_POLL_MS)

static u8  s_topo_poll_idx;
static u8  s_topo_sync_idx;
static u16 s_sync_div = 0;
static u8  s_port_rx_seq[MAX_SLAVE_NUM];
static sPortState s_port_trig[MAX_SLAVE_NUM];

static u16 s_io_ms_tick;
static u16 s_port_last_rx_ms[MAX_SLAVE_NUM];
static u8  s_port_ever_rx[MAX_SLAVE_NUM];

void can_bus_port_state_tick_1ms(void)
{
    s_io_ms_tick++;
}

static u8 can_bus_topo_station_count(void)
{
    u8 n;

    n = cfg_table_topo_station_count();
    if(n == 0 && ioCtrl_num > 0)
    {
        n = ioCtrl_num;
    }
    return n;
}

static u8 can_bus_sta_in_topo(u8 sta_idx)
{
    u8 station;
    u8 i, n;

    if(sta_idx >= MAX_SLAVE_NUM)
    {
        return 0;
    }
    station = (u8)(sta_idx + 2);
    n = cfg_table_topo_station_count();
    if(n == 0)
    {
        return (u8)(sta_idx < ioCtrl_num);
    }
    for(i = 0; i < n; i++)
    {
        if(cfg_table_topo_station_at(i) == station)
        {
            return 1;
        }
    }
    return 0;
}

u16 can_bus_port_state_stale_ms(void)
{
    u16 limit;
    u16 n;

    n = (u16)can_bus_topo_station_count();
    if(n == 0)
    {
        n = 1u;
    }
    limit = (u16)(n * COS_SYNC_PERIOD_MS * PORT_STATE_STALE_SYNC_ROUNDS);
    if(limit < PORT_STATE_STALE_MIN_MS)
    {
        limit = PORT_STATE_STALE_MIN_MS;
    }
    return limit;
}

/* COS 8B 载荷：[0]半满 [1]满 [2]卡 [3]门 [4]锁 [5]打印 [6]满包档1 [7]满包档2；seq 在 ExtID.seg_num */
static void cos_body_to_state(sPortState *ps, const u8 *body)
{
    if(ps == NULL || body == NULL)
    {
        return;
    }
    ps->port_half_full_state = body[0];
    ps->port_pack_full_state = body[1];
    ps->port_block_state    = body[2];
    ps->port_door_state     = body[3];
    ps->port_enable_state   = body[4];
    ps->port_print_state    = body[5];
    ps->port_full_tier1_state = body[6];
    ps->port_full_tier2_state = body[7];
}

static void can_bus_apply_cos_body(u8 sta_idx, const u8 *body)
{
    if(sta_idx >= MAX_SLAVE_NUM)
    {
        return;
    }
    cos_body_to_state(&portState[sta_idx], body);
}

static void can_bus_port_state_on_reply(u8 sta_idx)
{
    if(sta_idx >= MAX_SLAVE_NUM)
    {
        return;
    }
    s_port_last_rx_ms[sta_idx] = s_io_ms_tick;
    s_port_ever_rx[sta_idx] = 1;
}

static void can_bus_on_cos_confirm(u8 sta_idx, u8 seq, const u8 *body)
{
    if(sta_idx >= MAX_SLAVE_NUM || body == NULL)
    {
        return;
    }
    if(s_port_rx_seq[sta_idx] != 0 && seq != (u8)(s_port_rx_seq[sta_idx] + 1u)
       && seq != s_port_rx_seq[sta_idx])
    {
        /* 序号跳变：仍接受，但依赖慢同步兜底 */
    }
    s_port_rx_seq[sta_idx] = seq;
    can_bus_apply_cos_body(sta_idx, body);
    can_bus_port_state_on_reply(sta_idx);
    io_on_cos_confirm(sta_idx);
}

static void can_bus_send_port_cmd(u8 station, u8 sync)
{
    u8 buff[8];

    buff[0] = sync ? (u8)(0x01u | PORT_STATE_FLAG_SYNC) : 0x01u;
    buff[1] = 0;
    buff[2] = 0;
    buff[3] = 0;
    buff[4] = 0;
    buff[5] = 0;
    buff[6] = 0;
    buff[7] = 0;

    if(station >= 2 && port_close_flag[station - 2])
    {
        buff[1] = close_cmd;
        buff[2] = (u8)(port_close_state[station - 2] & 0xFFu);
        port_close_flag[station - 2] = 0;
        port_close_state[station - 2] = 0;
    }
    if(station >= 2 && port_clear_flag[station - 2])
    {
        buff[3] = (u8)(port_clear_state[station - 2] & 0xFFu);
        port_clear_flag[station - 2] = 0;
        port_clear_state[station - 2] = 0;
    }

    vcan_send_send_msg(buff, 8, CAN_FUNC_ID_PORT_STATE, station);
}

u8 can_bus_port_state_stale(u8 sta_idx)
{
    u16 age;

    if(sta_idx >= MAX_SLAVE_NUM)
    {
        return 1;
    }
    if(!can_bus_sta_in_topo(sta_idx))
    {
        return 1;
    }
    if(s_port_last_rx_ms[sta_idx] == 0)
    {
        return 1;
    }
    age = (u16)(s_io_ms_tick - s_port_last_rx_ms[sta_idx]);
    return (u8)(age > can_bus_port_state_stale_ms());
}

void can_bus_port_state_reset_all(void)
{
    memset(s_port_last_rx_ms, 0, sizeof(s_port_last_rx_ms));
    memset(s_port_ever_rx, 0, sizeof(s_port_ever_rx));
    memset(s_port_rx_seq, 0, sizeof(s_port_rx_seq));
    memset(s_port_trig, 0, sizeof(s_port_trig));
}

u8 can_bus_slave_cos_alive(u8 sta_idx)
{
    if(sta_idx >= MAX_SLAVE_NUM)
    {
        return 0;
    }
    if(!s_port_ever_rx[sta_idx])
    {
        return 0;
    }
    return (u8)(!can_bus_port_state_stale(sta_idx));
}

void can_bus_request_sync_all(void)
{
    u8 i, n, s;

    n = cfg_table_topo_station_count();
    if(n > 0)
    {
        for(i = 0; i < n; i++)
        {
            s = cfg_table_topo_station_at(i);
            if(s >= 2)
            {
                can_bus_send_port_cmd(s, 1);
            }
        }
        return;
    }
    if(ioCtrl_num == 0)
    {
        return;
    }
    for(s = 2; s <= (u8)(ioCtrl_num + 1); s++)
    {
        can_bus_send_port_cmd(s, 1);
    }
}

void vcan_bus_frame_queue_init(void)
{
    sCanFramequeue* q;

    q = &canframequeue;
    q->queue = canframebuff;

    q->front = q->rear = 0;

    q->maxsize = CAN_SEND_FRAME_LEN;
}

void vcan_addto_frame_queue(sCanFrameExt x)
{
    sCanFramequeue* q;

    q = &canframequeue;

    if ((q->rear + 1) % q->maxsize == q->front) {
        return;
    }

    q->rear = (q->rear + 1) % q->maxsize;

    q->queue[q->rear] = x;
}

u8 u8Is_feame_queue_free(void)
{
    sCanFramequeue* q;

    q = &canframequeue;

    if (q->front == q->rear) {
        return 1;
    }
    else {
        return 0;
    }
}


u8 u8can_bus_send_one_frame(sCanFrameExt sTxMsg)
{
    CanTxMsg TxMessage;

    TxMessage.ExtId = (sTxMsg.extId.src_id & 0x7F) | ((sTxMsg.extId.func_id & 0x1F) << 7) | ((sTxMsg.extId.seg_num & 0xFF) << 12) | ((sTxMsg.extId.seg_polo & 0x3) << 20) | ((sTxMsg.extId.dst_id & 0x7F) << 22);
    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = sTxMsg.data_len;
    memcpy(TxMessage.Data, sTxMsg.data, TxMessage.DLC);

    return CAN_Transmit(CAN1, &TxMessage);

}

void vcan_send_send_msg(u8* buff, u16 send_total_len, u8 type, u8 dst)
{
    sCanFrameExt canTxMsg;
    u16 send_len = 0;

    canTxMsg.extId.func_id = type;
    canTxMsg.extId.src_id =  1;
    canTxMsg.extId.dst_id = dst;

    if (send_total_len <= CAN_PACK_DATA_LEN)//不需要分段传输
    {
        canTxMsg.extId.seg_polo = CAN_SEG_POLO_NONE;
        canTxMsg.extId.seg_num = 0;
        canTxMsg.data_len = send_total_len;
        memcpy(canTxMsg.data, buff, canTxMsg.data_len);
        vcan_addto_frame_queue(canTxMsg);
    }
    else//需要分段传输
    {
        canTxMsg.extId.seg_polo = CAN_SEG_POLO_FIRST;
        canTxMsg.extId.seg_num = 0;
        canTxMsg.data_len = CAN_PACK_DATA_LEN;
        memcpy(canTxMsg.data, buff, canTxMsg.data_len);
        vcan_addto_frame_queue(canTxMsg);
        send_len += CAN_PACK_DATA_LEN;
        while (1) {
            if (send_len + CAN_PACK_DATA_LEN < send_total_len)
            {
                canTxMsg.extId.seg_polo = CAN_SEG_POLO_MIDDLE;
                canTxMsg.extId.seg_num++;
                canTxMsg.data_len = CAN_PACK_DATA_LEN;
                memcpy(canTxMsg.data, buff + send_len, canTxMsg.data_len);
                vcan_addto_frame_queue(canTxMsg);
                send_len += CAN_PACK_DATA_LEN;
            }
            else
            {
                canTxMsg.extId.seg_polo = CAN_SEG_POLO_FINAL;
                canTxMsg.extId.seg_num++;
                canTxMsg.data_len = send_total_len - send_len;
                memcpy(canTxMsg.data, buff + send_len, canTxMsg.data_len);
                vcan_addto_frame_queue(canTxMsg);
                break;
            }
        }
    }
}


void vcan_send_frame_process(void)
{
    sCanFramequeue* q = &canframequeue;
    sCanFrameExt* canTxMsg = NULL;
    u16 front;
    u8  tx_mailbox;

    if (q->front == q->rear)
    {
        return;
    }
    front = q->front;
    front = (front + 1) % q->maxsize;
    canTxMsg = (sCanFrameExt*)(&(q->queue[front]));
    tx_mailbox = u8can_bus_send_one_frame(*canTxMsg);
    if (tx_mailbox != CAN_NO_MB)
    {
        q->front = front;
    }

}

void can_bus_send_one_frame(sCanFrameExt sTxMsg)
{
    CanTxMsg TxMessage;
    u8 i;
    
    TxMessage.ExtId = (sTxMsg.extId.src_id & 0x7F) | ((sTxMsg.extId.func_id & 0x1F) << 7) | ((sTxMsg.extId.dst_id & 0x7F) << 22);
    TxMessage.IDE = CAN_ID_EXT;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = sTxMsg.data_len;
    
    for(i=0; i<TxMessage.DLC; i++)
    {
        TxMessage.Data[i] = sTxMsg.data[i];
    }
    
    CAN_Transmit(CAN1,&TxMessage);
}

void can_bus_send_print_response(u8 dst)
{
    u8 buff[20] = {0};

    vcan_send_send_msg(buff, 0, CAN_FUNC_ID_PRINT_STATE, dst);
}
static u8 can_bus_poll_station(u8 *idx, u8 sync_round)
{
    u8 n;
    u8 sta;

    n = cfg_table_topo_station_count();
    if(n == 0)
    {
        if(ioCtrl_num == 0)
        {
            return 0;
        }
        sta = (u8)(2u + (*idx));
        (*idx)++;
        if(*idx >= ioCtrl_num)
        {
            *idx = 0;
        }
        return sta;
    }
    if(*idx >= n)
    {
        *idx = 0;
    }
    sta = cfg_table_topo_station_at(*idx);
    (*idx)++;
    if(sta < 2)
    {
        return 0;
    }
    (void)sync_round;
    return sta;
}

void can_bus_frame_receive(CanRxMsg rxMsg)
{
    sCanFrameExtID extID;
    
    extID.func_id = (rxMsg.ExtId >> 7) & 0x1F;
    extID.src_id  = rxMsg.ExtId & 0x7F;
    extID.seg_num = (rxMsg.ExtId >> 12) & 0xFF;
    
    if(extID.func_id == CAN_FUNC_ID_STATION_LEARN)
    {
        if(rxMsg.DLC >= 2)
        {
            cfg_station_learn_on_rx(extID.src_id, rxMsg.Data, rxMsg.DLC);
        }
        return;
    }

    if(extID.src_id < 2) return;

    if(extID.func_id == CAN_FUNC_ID_PORT_COS_TRIG)
    {
        u8 sta = (u8)(extID.src_id - 2);
        if(rxMsg.DLC >= 8 && sta < MAX_SLAVE_NUM)
        {
            cos_body_to_state(&s_port_trig[sta], rxMsg.Data);
            io_on_cos_trig(sta, rxMsg.Data);
        }
    }
    else if(extID.func_id == CAN_FUNC_ID_PORT_COS_CONF)
    {
        u8 sta = (u8)(extID.src_id - 2);
        if(rxMsg.DLC >= 8 && sta < MAX_SLAVE_NUM)
        {
            can_bus_on_cos_confirm(sta, extID.seg_num, rxMsg.Data);
        }
    }
    else if(extID.func_id == CAN_FUNC_ID_CFG_ACK)
    {
        /* Data[0]=kind(2=TIME,3=PIN)  Data[1]=result(0=OK,1=无PIN,2=CRC,3=Flash,5=参数) */
        if(rxMsg.DLC >= 2)
        {
            cfg_can_on_cfg_ack(extID.src_id, rxMsg.Data[0], rxMsg.Data[1]);
        }
    }
    else if(extID.func_id == CAN_FUNC_ID_PRINT_STATE)
    {
        if(ext_host_flag != 0) return;
        if(rxMsg.Data[0] != frame_count[extID.src_id-2])
        {
            portState[extID.src_id-2].port_print_state |= ((rxMsg.Data[3]<<16)|(rxMsg.Data[2]<<8)|(rxMsg.Data[1]))&0xFFFFF;
            frame_count[extID.src_id-2] = rxMsg.Data[0];
            io_note_print_event(extID.src_id);
        }
        can_bus_send_print_response(extID.src_id);
    }
}

void can_bus_send_io_output(u8 station)
{
    CanTxMsg TxMessage;
    u16 out;
    u8 mailbox;
    u8 retry;

    if(station < 2 || (station - 2) >= MAX_SLAVE_NUM)
    {
        return;
    }
    /* 与从机 can.c：S_M_CTRL_ID(sta)=0x100+sta；物理 OUT0~7 各 2bit（关/开/快闪/慢闪） */
    out = io_port_out[station - 2];
    TxMessage.StdId = (u32)(0x100u + station);
    TxMessage.ExtId = 0;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 2;
    TxMessage.Data[0] = (u8)(out & 0xFFu);
    TxMessage.Data[1] = (u8)((out >> 8) & 0xFFu);

    /* 邮箱忙时短等再发，避免被 EXT 静默丢掉（约等满 3 个邮箱腾出） */
    for(retry = 0; retry < 200; retry++)
    {
        if((CAN1->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) != 0)
        {
            mailbox = CAN_Transmit(CAN1, &TxMessage);
            if(mailbox != CAN_NO_MB)
            {
                return;
            }
        }
    }
}

void can_bus_send_cycle_5ms(void)
{
    u8 station;

    station = can_bus_poll_station(&s_topo_poll_idx, 0);
    if(station < 2)
    {
        return;
    }

    /* 灯控 + 关格命令（不要求应答）；状态由从机 COS TRIG/CONF 上报 */
    can_bus_send_io_output(station);
    can_bus_send_port_cmd(station, 0);

    s_sync_div++;
    if(s_sync_div >= COS_SYNC_DIV_TICKS)
    {
        u8 sync_sta;

        s_sync_div = 0;
        sync_sta = can_bus_poll_station(&s_topo_sync_idx, 1);
        if(sync_sta >= 2)
        {
            can_bus_send_port_cmd(sync_sta, 1);
        }
    }

    if(poll_cnt != 0)
    {
        poll_cnt--;
    }
}

void can_bus_set_port_close_cmd(void)
{
    u16 packed;
    u16 num = 0;
    u8 sta_idx, bit;

    if (poll_cnt != 0) return;

    if (last_close_cmd == 0) {
        close_cmd = 1;
    }
    else {
        close_cmd = 0;
    }

    /* 队列元素：(sta_idx<<8)|local_bit — 与 io_commit_outputs / 1405 解析一致 */
    if (close_cmd == 0) {
        while (1) {
            packed = fun_get_msg_from_int_16_queue(&closemsgqueue);
            if (packed != INT_16_NULL) {
                sta_idx = (u8)(packed >> 8);
                bit = (u8)(packed & 0xFF);
                if(sta_idx < MAX_SLAVE_NUM && bit < 8)
                {
                    port_close_state[sta_idx] |= (0x1u << bit);
                    port_close_flag[sta_idx] = 1;
                    num++;
                }
            }
            else {
                break;
            }
        }
        if (num != 0) {
            poll_cnt = ioCtrl_num + 1;
        }
    }

    if (close_cmd == 1) {
        while (1) {
            packed = fun_get_msg_from_int_16_queue(&openmsgqueue);
            if (packed != INT_16_NULL) {
                sta_idx = (u8)(packed >> 8);
                bit = (u8)(packed & 0xFF);
                if(sta_idx < MAX_SLAVE_NUM && bit < 8)
                {
                    port_close_state[sta_idx] |= (0x1u << bit);
                    port_close_flag[sta_idx] = 1;
                    num++;
                }
            }
            else {
                break;
            }
        }
        if (num != 0) {
            poll_cnt = ioCtrl_num + 1;
        }
    }

    last_close_cmd = close_cmd;
}

void can_bus_set_port_clear_cmd(u8* point)
{
    u16 addrs_num;
    u16 *p_addrs;
    u16 i;
    u16 addr;
    u16 idx;
    u8 sta_idx, bit;

    addrs_num = (point[1]<<8)|point[0];
    p_addrs = (u16*)(point+2);

    for(i=0; i<addrs_num;i++)
    {
        addr = p_addrs[i];
        idx = cfg_find_port_index(addr);
        if(idx == 0xFFFF
           || unload_port_assign[idx].station < 2
           || unload_port_assign[idx].local_bit >= 8)
        {
            continue;
        }
        sta_idx = (u8)(unload_port_assign[idx].station - 2);
        bit = unload_port_assign[idx].local_bit;
        port_clear_state[sta_idx] |= (0x1u << bit);
        port_clear_flag[sta_idx] = 1;
    }
}

void can_bus_forward_screen_cmd(void)
{
    sScreenDate* sendScr;
    u8 buff[40] = {0};

    while (1) {
        if ((canframequeue.rear + 1) % canframequeue.maxsize == canframequeue.front) return;
        
        sendScr = fun_get_from_screen_queue(&screenqueue);
        if (sendScr != NULL) {    
            memcpy(&buff[1], sendScr->scr_buf, sendScr->total_len);
            buff[0] = sendScr->type;
            vcan_send_send_msg(buff, sendScr->total_len + 1, CAN_FUNC_ID_SCREEN_DATA, sendScr->board_num);
        }
        else {
            break;
        }
    }
}

void can_bus_set_port_print_reply(u8* point)
{
    u16 addrs_num;
    u16 *p_addrs;
    u16 i;
    u16 addr;
    
    addrs_num = (point[1]<<8)|point[0];
    p_addrs = (u16*)(point+2);
    
    for(i=0; i<addrs_num; i++)
    {
        addr = p_addrs[i];
        port_print_reply_state[addr/20] |= (0x1<<(addr%20));
    }
}

void can_bus_set_port_time_cmd(u8* point)
{
    u8 buff[12];
    u16 crc;

    (void)point;
    memcpy(buff, &g_cfg_time, 10);
    crc = cfg_crc16(buff, 10);
    buff[10] = (u8)(crc & 0xFF);
    buff[11] = (u8)(crc >> 8);
    vcan_send_send_msg(buff, 12, CAN_FUNC_ID_INIT_TIME_DATA, 0x7F);
}

///// send port state to wcs /////
/* Logic-layer upload: cross-belt 0x1402 = 4 groups only: close/full/block/door */
static u8 upload_wcs_port_bit(u16 port_idx)
{
    u8 b;
    if(port_idx >= g_cfg_port_num)
    {
        return 0;
    }
    b = unload_port_assign[port_idx].local_bit;
    return (b < 8) ? b : 0;
}

static u8 upload_port_station_stale(u16 port_idx)
{
    u8 sta_idx;

    if(port_idx >= g_cfg_port_num)
    {
        return 1;
    }
    if(unload_port_assign[port_idx].station < 2)
    {
        return 1;
    }
    sta_idx = (u8)(unload_port_assign[port_idx].station - 2);
    return can_bus_port_state_stale(sta_idx);
}

/* 向 1402 某组追加业务口地址；cap 为整包上限，返回 0 表示空间不足 */
static u8 upload_put_addr(u16 *len_io, u16 *num_io, u16 addr, u16 cap)
{
    u16 len = *len_io;
    u16 num = *num_io;
    u16 need;

    need = (u16)(len + 2 + (u16)(num + 1u) * 2u);
    if(need > cap)
    {
        return 0;
    }
    port_state_upload_buff[len + 2 + num * 2] = (u8)(addr & 0xFF);
    port_state_upload_buff[len + 3 + num * 2] = (u8)(addr >> 8);
    (*num_io)++;
    return 1;
}

static void upload_flush_group_count(u16 group_base, u16 num)
{
    port_state_upload_buff[group_base] = (u8)(num & 0xFF);
    port_state_upload_buff[group_base + 1] = (u8)(num >> 8);
}

static u8 upload2WCS_from_logic(void)
{
    u16 i, j;
    u16 num;
    u16 len;
    u16 addr;
    u8 sta_idx, bit;

    /* 0x1404 打印由 cfg_logic → print_alarm_process 即时上报 */

    /* 1) close_addr */
    num = 0;
    len = 0;
    for(i = 0; i < g_cfg_port_num; i++)
    {
        if(upload_port_station_stale(i))
        {
            continue;
        }
        if(unload_port[i].port_disable_stt || unload_port[i].lock_button.stt == IO_STT_ON)
        {
            if(!upload_put_addr(&len, &num, unload_port_assign[i].port_no,
                                IOCTRL2WCS_UPLOAD_BODY_MAX))
            {
                break;
            }
        }
    }
    upload_flush_group_count(0, num);
    len = (u16)(2 + num * 2);

    /* 2) full_addr */
    num = 0;
    for(i = 0; i < g_cfg_port_num; i++)
    {
        if(upload_port_station_stale(i))
        {
            continue;
        }
        if(unload_port[i].full_pack_photo.level == IO_LVL_ON)
        {
            if(!upload_put_addr(&len, &num, unload_port_assign[i].port_no,
                                IOCTRL2WCS_UPLOAD_BODY_MAX))
            {
                break;
            }
        }
    }
    upload_flush_group_count(len, num);
    len = (u16)(len + 2 + num * 2);

    /* 3) block_addr (+ jam_extra)；成员口由 cfg_rebuild_jam_skip 预标记 */
    num = 0;
    for(i = 0; i < g_cfg_port_num; i++)
    {
        if(upload_port_station_stale(i))
        {
            continue;
        }
        if(unload_port[i].clamp_pack_photo.level != IO_LVL_ON)
        {
            continue;
        }
        if(i < UNLOAD_PORT_NUM && g_cfg_jam_block_skip[i])
        {
            continue;
        }
        if(!upload_put_addr(&len, &num, unload_port_assign[i].port_no,
                            IOCTRL2WCS_UPLOAD_BODY_MAX))
        {
            break;
        }
        for(j = 0; j < unload_port_assign[i].jam_extra_num; j++)
        {
            if(!upload_put_addr(&len, &num, unload_port_assign[i].jam_extra_no[j],
                                IOCTRL2WCS_UPLOAD_BODY_MAX))
            {
                break;
            }
        }
    }
    upload_flush_group_count(len, num);
    len = (u16)(len + 2 + num * 2);

    /* 4) door_addr from slave door bitmap */
    num = 0;
    for(i = 0; i < g_cfg_port_num && num < CFG_BIZ_PORT_MAX; i++)
    {
        if(unload_port_assign[i].station < 2)
        {
            continue;
        }
        sta_idx = (u8)(unload_port_assign[i].station - 2);
        if(sta_idx >= MAX_SLAVE_NUM)
        {
            continue;
        }
        bit = upload_wcs_port_bit(i);
        if(can_bus_port_state_stale(sta_idx))
        {
            continue;
        }
        if((portState[sta_idx].port_door_state >> bit) & 1u)
        {
            addr = unload_port_assign[i].port_no;
            port_state_upload_buff[len + 2 + num * 2] = addr & 0xFF;
            port_state_upload_buff[len + 3 + num * 2] = (addr >> 8) & 0xFF;
            num++;
        }
    }
    port_state_upload_buff[len] = num & 0xFF;
    port_state_upload_buff[len + 1] = (num >> 8) & 0xFF;
    len = (u16)(len + 2 + num * 2);

    /* 可选扩展组：半满 / 满包档1 / 满包档2（142F bit0/bit2） */
    {
        u16 upf = cfg_upload_flags_effective();

        full_tier_active = (upf & CFG_UPLOAD_F_FULL_TIER) ? 1u : 0u;

        if(upf & CFG_UPLOAD_F_HALF)
        {
            num = 0;
            for(i = 0; i < g_cfg_port_num && num < CFG_BIZ_PORT_MAX; i++)
            {
                if(upload_port_station_stale(i))
                {
                    continue;
                }
                if(unload_port[i].half_full_photo.level == IO_LVL_ON)
                {
                    addr = unload_port_assign[i].port_no;
                    port_state_upload_buff[len + 2 + num * 2] = addr & 0xFF;
                    port_state_upload_buff[len + 3 + num * 2] = (addr >> 8) & 0xFF;
                    num++;
                }
            }
            port_state_upload_buff[len] = num & 0xFF;
            port_state_upload_buff[len + 1] = (num >> 8) & 0xFF;
            len = (u16)(len + 2 + num * 2);
        }

        if(upf & CFG_UPLOAD_F_FULL_TIER)
        {
            /* 5) full_tier1_addr */
            num = 0;
            for(i = 0; i < g_cfg_port_num && num < CFG_BIZ_PORT_MAX; i++)
            {
                if(unload_port_assign[i].station < 2)
                {
                    continue;
                }
                sta_idx = (u8)(unload_port_assign[i].station - 2);
                if(sta_idx >= MAX_SLAVE_NUM)
                {
                    continue;
                }
                bit = upload_wcs_port_bit(i);
                if(can_bus_port_state_stale(sta_idx))
                {
                    continue;
                }
                if((portState[sta_idx].port_full_tier1_state >> bit) & 1u)
                {
                    if(!upload_put_addr(&len, &num, unload_port_assign[i].port_no,
                                        IOCTRL2WCS_UPLOAD_BODY_MAX))
                    {
                        break;
                    }
                }
            }
            upload_flush_group_count(len, num);
            len = (u16)(len + 2 + num * 2);

            /* 6) full_tier2_addr */
            num = 0;
            for(i = 0; i < g_cfg_port_num && num < CFG_BIZ_PORT_MAX; i++)
            {
                if(unload_port_assign[i].station < 2)
                {
                    continue;
                }
                sta_idx = (u8)(unload_port_assign[i].station - 2);
                if(sta_idx >= MAX_SLAVE_NUM)
                {
                    continue;
                }
                bit = upload_wcs_port_bit(i);
                if(can_bus_port_state_stale(sta_idx))
                {
                    continue;
                }
                if((portState[sta_idx].port_full_tier2_state >> bit) & 1u)
                {
                    if(!upload_put_addr(&len, &num, unload_port_assign[i].port_no,
                                        IOCTRL2WCS_UPLOAD_BODY_MAX))
                    {
                        break;
                    }
                }
            }
            upload_flush_group_count(len, num);
            len = (u16)(len + 2 + num * 2);
        }
    }

    g_port_state_upload_len = len;

    if(fun_ioctrl2wcs_tx_busy())
    {
        return 0;
    }

    /* 无变化时按 UPLOAD_HEARTBEAT_TICKS 降频；0 则每周期仍发 1402 作 WCS 心跳 */
    {
        u16 body_crc = cfg_crc16(port_state_upload_buff, len);

        if(len == s_upload_shadow_len && body_crc == s_upload_shadow_crc)
        {
            if(s_upload_heartbeat < UPLOAD_HEARTBEAT_TICKS)
            {
                s_upload_heartbeat++;
                return 0;
            }
        }
        else
        {
            s_upload_shadow_crc = body_crc;
            s_upload_shadow_len = len;
            s_upload_heartbeat = 0;
        }
    }

    fun_ioctrl2wcs_prepare_tx(port_state_upload_buff, len);
    AddSendMsgToQueue(SEND_MSG_IOCTRL2WCS_CMD_TYPE);
    return 1;
}

void upload2WCS(void)
{
    if(ioCtrl_num == 0)
    {
        return;
    }
    if(g_cfg_host_ready == 0 || g_cfg_port_num == 0)
    {
        return;
    }
    (void)upload2WCS_from_logic();
}


void upload_slave_can_error_cmd(void)
{
        u8 i = 0;
        
        slave_can_error_num = 0;
        
        for(i = 0; i < ioCtrl_num; i++)
        {
            if(s_port_ever_rx[i] && can_bus_port_state_stale(i))
            {
                slave_can_error_num++;
                upload_can_error[slave_can_error_num - 1] = i + 2;
            }
        }

        /* 无离线从机时不上报，避免周期空包 1413(num=0) */
        if(slave_can_error_num == 0)
        {
            return;
        }

        AddSendMsgToQueue(SEND_MSG_SLAVE_CAN_ERROR_TYPE);      
}