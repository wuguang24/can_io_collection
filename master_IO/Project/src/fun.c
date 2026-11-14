#include "main.h"
#include "can_bus.h"
#include "TCPclient.h"
#include "cfg_table.h"
#include "cfg_logic.h"
#include "cfg_can_ext.h"
#include "cfg_store.h"
#include "cfg_crc.h"
#include "station_learn.h"
#include "io.h"
#include "FreeRTOS.h"
#include "task.h"

/* 0x1402 分片发送：指向 port_state_upload_buff，发送期间 upload 会因 tx_busy 停建包 */
static const u8 *s_1402_body_ptr;
static u16 s_1402_body_len;
static u8  s_1402_seg_total;
static u8  s_1402_seg_next;
static u8  s_1402_tx_active;

#define msgSendQueueSize  100
u16 msgQueueBuff[msgSendQueueSize];
MSG_SEND_QUEUE msgSendQueue;

#define closemsgQueueSize  128
u16 closequeuebuff[closemsgQueueSize];
sINT_16_QUEUE  closemsgqueue;

#define openmsgQueueSize  128
u16 openqueuebuff[openmsgQueueSize];
sINT_16_QUEUE  openmsgqueue;

#define screenQueueSize  30
sScreenDate scrbuff[screenQueueSize];
sScreenQueue  screenqueue;

sBD2PC_Connect_msg      pcconnectmsg;
/* ip_offest_* 定义在 cfg_table.c */

u8 product_type = 0x03;

static void fun_try_autosave_report(void)
{
    if(!cfg_store_try_autosave())
    {
        AddSendMsgToQueue(SEND_MSG_SLAVE_CFG_FAIL_TYPE);
    }
}

/* 与协议文档 5.1.7.1 / 连接版本对齐 */
static const char sds_validate_version[] = "chute_5.1.7.1_w16_261014";

#define SOFT_VERSION_LEN (sizeof(sds_validate_version) - 1U)

void vpcfun_init_sys_item(void)
{
    /* 连接握手版本 5.1.7.1（与协议修订、校验字符串一致） */
    pcconnectmsg.version[0] = 0x05;
    pcconnectmsg.version[1] = 0x01;
    pcconnectmsg.version[2] = 0x07;
    pcconnectmsg.version[3] = 0x01;
}

void fun_Init_INT_16_Queue(void)
{
    sINT_16_QUEUE* q;
    sScreenQueue* k;

    q = &closemsgqueue;
    q->maxsize = closemsgQueueSize;
    q->queue = closequeuebuff;
    q->front = q->rear = 0;

    q = &openmsgqueue;
    q->maxsize = openmsgQueueSize;
    q->queue = openqueuebuff;
    q->front = q->rear = 0;

    k = &screenqueue;
    k->maxsize = screenQueueSize;
    k->queue = scrbuff;
    k->front = k->rear = 0;
}

void initQueue(MSG_SEND_QUEUE *q, u16 ms)
{
    q->maxSize = ms;
    q->queue = msgQueueBuff;
    q->front = q->rear = 0;
} 
void enQueue(MSG_SEND_QUEUE *q, u16 x)
{
    taskENTER_CRITICAL();
    /* 队列满 */
    if((q->rear + 1) % q->maxSize == q->front)
    {
        taskEXIT_CRITICAL();
        return;
    }
    q->rear = (q->rear + 1) % q->maxSize;
    q->queue[q->rear] = x;
    taskEXIT_CRITICAL();
}
u16 outQueue(MSG_SEND_QUEUE *q)
{
    u16 v;

    taskENTER_CRITICAL();
    if(q->front == q->rear)
    {
        taskEXIT_CRITICAL();
        return 0;
    }
    q->front = (q->front + 1) % q->maxSize;
    v = q->queue[q->front];
    taskEXIT_CRITICAL();
    return v;
}
void InitSendMsgQueue(void)
{
    initQueue(&msgSendQueue, msgSendQueueSize);
}
void AddSendMsgToQueue(u16 msg)
{
    enQueue(&msgSendQueue, msg);
    Net_Notify();
}
u16 GetSendMsgFromQueue(void)
{
    return (outQueue(&msgSendQueue));
}


void fun_add_to_int_16_Queue(sINT_16_QUEUE* q, u16 x)
{
    //队列满
    if ((q->rear + 1) % q->maxsize == q->front)
    {
        return;
    }
    q->rear = (q->rear + 1) % q->maxsize; // 求出队尾的下一个位置
    q->queue[q->rear] = x; // 把x的值赋给新的队尾
}

u16 fun_get_msg_from_int_16_queue(sINT_16_QUEUE* q)
{
    if (q->front == q->rear)
    {
        return INT_16_NULL;
    }
    q->front = (q->front + 1) % q->maxsize; // 使队首指针指向下一个位置
    return q->queue[q->front]; // 返回队首元素
}

//
void fun_add_to_screen_Queue(sScreenQueue* q, sScreenDate x)
{
    //队列满
    if ((q->rear + 1) % q->maxsize == q->front)
    {
        return;
    }
    q->rear = (q->rear + 1) % q->maxsize; // 求出队尾的下一个位置
    q->queue[q->rear] = x; // 把x的值赋给新的队尾
}

sScreenDate* fun_get_from_screen_queue(sScreenQueue* q)
{
    if (q->front == q->rear)
    {
        return NULL;
    }
    q->front = (q->front + 1) % q->maxsize; // 使队首指针指向下一个位置
    return &q->queue[q->front]; // 返回队首元素
}


void fun_recv_port_close_msg(u8* point)
{
    u16 addrs_num;
    u16* p_addrs;
    u16 i;
    u16 addr;
    u8  value;
    u16 idx;
    u8 sta_idx, bit;

    value = point[0];
    addrs_num = (point[2] << 8) | point[1];
    p_addrs = (u16*)(point + 3);

    for(i = 0; i < addrs_num; i++)
    {
        addr = p_addrs[i];
        if(!(addr >= ip_offest_start && addr <= ip_offest_end))
        {
            continue;
        }

        idx = cfg_find_port_index(addr);
        if(idx == 0xFFFF
           || unload_port_assign[idx].station < 2
           || unload_port_assign[idx].local_bit >= 8)
        {
            continue;
        }
        sta_idx = (u8)(unload_port_assign[idx].station - 2);
        bit = unload_port_assign[idx].local_bit;

        if(value == 0)
        {
            fun_add_to_int_16_Queue(&closemsgqueue,
                (u16)(((u16)sta_idx << 8) | bit));
            unload_port[idx].port_disable_stt = 1;
            unload_port[idx].port_close_stt = 1;
            unload_port[idx].lock_button.stt = 1;
        }
        else
        {
            fun_add_to_int_16_Queue(&openmsgqueue,
                (u16)(((u16)sta_idx << 8) | bit));
            unload_port[idx].port_disable_stt = 0;
            unload_port[idx].port_close_stt = 0;
            unload_port[idx].port_clear_stt = 0;
            unload_port[idx].lock_button.stt = 0;
        }
    }
}
u16 mb_crc16(u8 *data,u16 len)
{
    u16 crcvalue = 0xFFFF;
    u8  i;
    
    while(len--) {
        crcvalue ^= *data++;
        for(i=0; i<8; i++){
            if(crcvalue & 0x0001){
                crcvalue = (crcvalue>>1)^0xA001;
            }
            else{
                crcvalue = (crcvalue>>1);
            }
        }
    }
    return  crcvalue;
}
//转发屏幕数据-T251230
void fun_recv_screen_msg(u8* point)
{
    u8  total_num = 0;
    sScreenDate tmp;
    u8 *p = point;
    u16 port_io;
    u16 i;
    u16 crc_data;

    total_num = *p;
    memset(&tmp, 0, sizeof(tmp));
    
    p += 1;
    for (i = 0; i < total_num; i++)
    {
        port_io = *(u16 *)p;
        if (port_io >= ip_offest_start && port_io <= ip_offest_end)  {
            port_io -= ip_offest_start;
            tmp.board_num = port_io /PORT_OFFSET + 2;//板卡地址起始2
            tmp.addr = port_io %PORT_OFFSET + 1;//屏幕地址起始1

            p += 2;
            tmp.type = *(p++);
            tmp.total_len = *(p++);//m帧长度
            memcpy(tmp.scr_buf, p, tmp.total_len);
            tmp.scr_buf[0] = tmp.addr;//io地址更新

            //CRC结果
            crc_data = mb_crc16(&tmp.scr_buf[0], tmp.total_len - 2);//更新crc
            tmp.scr_buf[tmp.total_len - 2] = crc_data &0xFF;
            tmp.scr_buf[tmp.total_len - 1] = (crc_data >>8)&0xFF;

            p = p + tmp.total_len;
            fun_add_to_screen_Queue(&screenqueue, tmp);              
        }
    
    }
}

//板卡功能配置-T2601（调用前须已 cfg_table_parse_init_func 成功）
void fun_recv_init_config_msg(u8* point, u16 payload_len)
{
    (void)point;
    (void)payload_len;

    cfg_set_flag(CFG_FLAG_1423);
    cfg_table_apply_io_ctrl_from_init();
    cfg_store_mark_runtime_dirty();

    if(cfg_table_topo_station_count() == 0)
    {
        return;
    }

    if(cfg_host_ready())
    {
        cfg_can_start_distribute();
    }
}

/* 0x1431：op=1 启动站号自学习；须已收 1423 拓扑。result 见 reply_recv_msg_station_learn_ack */
static u8 g_stn_learn_rx_result;

u8 fun_recv_station_learn_msg(u8 *payload, u16 payload_len)
{
    if(payload == NULL || payload_len < 1 || payload[0] != 0x01u)
    {
        return 1;
    }
    if(cfg_table_topo_station_count() == 0)
    {
        return 2;
    }
    if(cfg_station_learn_active())
    {
        return 3;
    }
    cfg_station_learn_start();
    return 0;
}

u8 recv_msg_check(u8 *point, u16 len)
{
    u16 i = 0;
    u8  sum = 0;

    if((point[0] != 0xAA) || (point[1] != 0xAA))
        return 0;
    if((point[6] | point[7] << 8) != len)
        return 0;  
    sum = point[9];
    for(i = 1 ; i < len - 9 ; i++)
    {
        sum ^= point[9 + i];
    }
    if(sum != point[8])
    {
        return 0;
    }

    return 1;
}
void recv_msg_process(u8 *point)
{
    MSG_HEAD_DATA *head = (MSG_HEAD_DATA *)point;
    u16 head_size_u16 = sizeof(MSG_HEAD_DATA);
    u8 *payload = point + head_size_u16;
    u16 payload_len = 0;

    if(head->MSG_LENGTH > head_size_u16)
    {
        payload_len = (u16)(head->MSG_LENGTH - head_size_u16);
    }

    switch (head->MSG_TYPE)
    {
    case RECV_MSG_BOOT_CMD_TYPE:
        /*
         * APP 收 0x1F01：先回 0x9F01 再进 Boot。
         * 立即复位会导致上位机约 5s 超时判失败，第 2 次才成功（此时已在 Boot）。
         */
        reply_recv_msg(&(tcp_client_list[0].tcp_send_buf[0]),
                       &(tcp_client_list[0].tcp_send_len),
                       REPLY_RECV_MSG_BOOT_CMD_TYPE);
        tcp_client_list[0].tcp_send_en = 1;
        udp_send_process(0);
        tcp_client_list[0].tcp_send_en = 0;
        {
            volatile u32 d;
            for(d = 0; d < 800000u; d++)
            {
                ; /* 给网口一点时间把应答发出去 */
            }
        }
        BKP_WriteBackupRegister(BKP_DR8, 0x55);
        NVIC_SystemReset();
        break;
    case RECV_REPLY_MSG_IOPRT2WCS_CMD_TYPE:
        can_bus_set_port_print_reply(payload);
        break;
    case RECV_MSG_PORT_CLOSE_CMD_TYPE:
        fun_recv_port_close_msg(payload);
        AddSendMsgToQueue(REPLAY_RECV_MSG_PORT_CLOSE_CMD_TYPE);
        break;

    case RECV_MSG_PORT_CLEAR_CMD_TYPE:
        can_bus_set_port_clear_cmd(payload);
        break;
    case RECV_MSG_SCREEN_DATA_TYPE://0x1422
        fun_recv_screen_msg(payload);
        AddSendMsgToQueue(REPLY_RECV_MSG_SCREEN_DATA_TYPE);
        break;
    case RECV_RECV_MSG_INIT_FUNC_CONFIG_TYPE://0x1423
        if(cfg_table_parse_init_func(payload, payload_len))
        {
            fun_recv_init_config_msg(payload, payload_len);
            AddSendMsgToQueue(REPLY_RECV_RECV_MSG_INIT_FUNC_CONFIG_TYPE);
        }
        break;
    case RECV_RECV_MSG_INIT_IP_OFFEST_TYPE://0x1424
        ip_offest_start = (u16)(payload[0] | (payload[1] << 8));
        ip_offest_end = (u16)(payload[2] | (payload[3] << 8));
        cfg_set_flag(CFG_FLAG_1424);
        cfg_store_mark_runtime_dirty();
        AddSendMsgToQueue(REPLY_RECV_RECV_MSG_INIT_IP_OFFEST_TYPE);
        if(cfg_host_ready())
        {
            cfg_can_start_distribute();
        }
        break;
    case RECV_RECV_MSG_INIT_TIME_TYPE://0x1425
        {
            u16 time_num = (u16)(payload[0] | (payload[1] << 8));
            if(time_num >= 5)
            {
                g_cfg_time.lock_filter_time = (u16)(payload[2] | (payload[3] << 8));
                g_cfg_time.full_filter_time = (u16)(payload[4] | (payload[5] << 8));
                g_cfg_time.block_filter_time = (u16)(payload[6] | (payload[7] << 8));
                g_cfg_time.door_filter_time = (u16)(payload[8] | (payload[9] << 8));
                g_cfg_time.solt_filter_time = (u16)(payload[10] | (payload[11] << 8));
                cfg_set_flag(CFG_FLAG_1425);
                cfg_store_mark_runtime_dirty();
            }
            can_bus_set_port_time_cmd(payload);
            AddSendMsgToQueue(REPLY_RECV_RECV_MSG_INIT_TIME_TYPE);
            if(cfg_host_ready())
            {
                cfg_can_start_distribute();
            }
        }
        break;
    case RECV_MSG_INIT_PIN_TYPE://0x1426
        {
            u16 field0 = (u16)(payload[0] | (payload[1] << 8));
            u16 ver = (u16)(payload[2] | (payload[3] << 8));
            u16 crc = (u16)(payload[4] | (payload[5] << 8));
            u16 seg_total = (u16)(payload[6] | (payload[7] << 8));
            u16 seg_index = (u16)(payload[8] | (payload[9] << 8));
            u16 seg_bytes = (u16)(payload[10] | (payload[11] << 8));
            u8 r;

            if(payload_len < 12)
            {
                g_cfg_pin_rx_last_result = CFG_PIN_RX_BAD_PARAM;
                AddSendMsgToQueue(REPLY_RECV_MSG_INIT_PIN_TYPE);
                break;
            }
            r = cfg_pin_rx_segment(field0, ver, crc, seg_total, seg_index, seg_bytes, payload + 12);
            g_cfg_pin_rx_last_result = r;
            /* 收齐成功或失败均答 9426+result；中间片不答 */
            if(r != CFG_PIN_RX_MORE)
            {
                AddSendMsgToQueue(REPLY_RECV_MSG_INIT_PIN_TYPE);
            }
        }
        break;
    case RECV_MSG_INIT_LOGIC_TYPE://0x1427
        if(cfg_logic_set_from_payload(payload, payload_len))
        {
            cfg_store_mark_runtime_dirty();
            AddSendMsgToQueue(REPLY_RECV_MSG_INIT_LOGIC_TYPE);
            if(cfg_host_ready() && !cfg_can_is_busy())
            {
                fun_try_autosave_report();
            }
        }
        break;
    case RECV_MSG_INIT_COMBO_TYPE://0x1428  func + 载荷
        if(payload_len >= 1
           && cfg_logic_set_by_func(payload[0], payload + 1, (u16)(payload_len - 1)))
        {
            cfg_store_mark_runtime_dirty();
            AddSendMsgToQueue(REPLY_RECV_MSG_INIT_COMBO_TYPE);
            /* 灯效只在主机：无 PIN 分发则立即固化 */
            if(cfg_host_ready() && !cfg_can_is_busy())
            {
                fun_try_autosave_report();
            }
        }
        break;
    case RECV_MSG_CFG_STORE_TYPE://0x142D
        if(cfg_store_handle_cmd(payload, payload_len))
        {
            AddSendMsgToQueue(REPLY_RECV_MSG_CFG_STORE_TYPE);
        }
        break;
    case RECV_MSG_JAM_FANOUT_TYPE://0x142E
        {
            u8 station, op, count;
            u16 ports[CFG_MAX_JAM_FANOUT];
            u8 i;

            if(payload_len < 3)
            {
                break;
            }
            station = payload[0];
            op = payload[1];
            count = payload[2];
            if((u16)(3 + count * 2) > payload_len || count > CFG_MAX_JAM_FANOUT)
            {
                break;
            }
            for(i = 0; i < count; i++)
            {
                ports[i] = (u16)(payload[3 + i * 2] | (payload[4 + i * 2] << 8));
            }
            if(cfg_jam_fanout_edit(station, op, count, ports))
            {
                cfg_store_mark_runtime_dirty();
                if(cfg_host_ready() && !cfg_can_is_busy())
                {
                    fun_try_autosave_report();
                }
                AddSendMsgToQueue(REPLY_RECV_MSG_JAM_FANOUT_TYPE);
            }
        }
        break;
    case RECV_MSG_STATION_LEARN_TYPE://0x1431
        g_stn_learn_rx_result = fun_recv_station_learn_msg(payload, payload_len);
        AddSendMsgToQueue(REPLY_RECV_MSG_STATION_LEARN_TYPE);
        break;
    case RECV_MSG_EXTIO_TYPE://0x142F 上报扩展 + 跨站 OUT
        if(cfg_extio_set_from_payload(payload, payload_len))
        {
            cfg_store_mark_runtime_dirty();
            if(cfg_host_ready() && !cfg_can_is_busy())
            {
                fun_try_autosave_report();
            }
            AddSendMsgToQueue(REPLY_RECV_MSG_EXTIO_TYPE);
        }
        break;
    case RECV_RECV_MSG_EMERGENCY_TYPE://0x142B
        {
            u8 buff[2];
            u8 len = 2;

            buff[0] = CAN_FUNC_ID_CTRL1_CMD;
            buff[1] = *payload;
            vcan_send_send_msg(buff, len, CAN_FUNC_ID_ENABLE_ALL_FULL, 0x7F);
        }
        AddSendMsgToQueue(REPLY_RECV_RECV_MSG_EMERGENCY_TYPE);
        break;
    default:
        break;
    }
}
void recv_message_from_sever(u8 *point, u16 *len)
{
    if(recv_msg_check(point, *len) == 0)
    {
        *len = 0;
        return;
    }
    recv_msg_process(point);
    *len = 0;
}
u8 fun_ioctrl2wcs_tx_busy(void)
{
    return s_1402_tx_active;
}

void fun_ioctrl2wcs_prepare_tx(const u8 *body, u16 body_len)
{
    u16 nseg;

    if(body == NULL || body_len == 0 || body_len > IOCTRL2WCS_UPLOAD_BODY_MAX)
    {
        return;
    }

    s_1402_body_ptr = body;
    s_1402_body_len = body_len;
    s_1402_seg_next = 0;
    s_1402_tx_active = 1;

    if(body_len <= IOCTRL2WCS_SEG_THRESHOLD)
    {
        s_1402_seg_total = 1;
    }
    else
    {
        nseg = (u16)((body_len + IOCTRL2WCS_SEG_CHUNK_MAX - 1u) / IOCTRL2WCS_SEG_CHUNK_MAX);
        if(nseg > 255u)
        {
            nseg = 255u;
        }
        s_1402_seg_total = (u8)nseg;
    }
}

void send_msg_ioctrl2wcs_cmd(u8 *buf, u16 *len, u16 type)
{
    u8 sum;
    u16 sendlen;
    u16 i;
    u16 body_len;
    u16 data_len;
    u16 chunk_off;
    u16 chunk_len;
    u16 body_crc;

    body_len = s_1402_body_len;
    if(body_len == 0)
    {
        port_state_upload_buff[0] = 0;
        port_state_upload_buff[1] = 0;
        s_1402_body_ptr = port_state_upload_buff;
        body_len = 2;
        s_1402_seg_total = 1;
        s_1402_seg_next = 0;
        s_1402_tx_active = 1;
    }
    if(s_1402_body_ptr == NULL)
    {
        *len = 0;
        s_1402_tx_active = 0;
        return;
    }

    buf[9] = type & 0xFF;
    buf[10] = (type >> 8) & 0xFF;

    if(body_len <= IOCTRL2WCS_SEG_THRESHOLD && s_1402_seg_total == 1)
    {
        /* 单帧格式：数据区≤256，close_num 起 */
        for(i = 0; i < body_len; i++)
        {
            buf[11 + i] = s_1402_body_ptr[i];
        }
        data_len = body_len;
        s_1402_tx_active = 0;
    }
    else
    {
        u16 hdr_off = 11;

        chunk_off = (u16)s_1402_seg_next * IOCTRL2WCS_SEG_CHUNK_MAX;
        if(chunk_off >= body_len)
        {
            s_1402_tx_active = 0;
            *len = 0;
            return;
        }
        chunk_len = body_len - chunk_off;
        if(chunk_len > IOCTRL2WCS_SEG_CHUNK_MAX)
        {
            chunk_len = IOCTRL2WCS_SEG_CHUNK_MAX;
        }

        body_crc = cfg_crc16(s_1402_body_ptr, body_len);

        buf[hdr_off + 0] = (u8)(IOCTRL2WCS_SEG_MAGIC & 0xFF);
        buf[hdr_off + 1] = (u8)(IOCTRL2WCS_SEG_MAGIC >> 8);
        buf[hdr_off + 2] = s_1402_seg_total;
        buf[hdr_off + 3] = s_1402_seg_next;
        buf[hdr_off + 4] = (u8)(body_len & 0xFF);
        buf[hdr_off + 5] = (u8)(body_len >> 8);
        buf[hdr_off + 6] = (u8)(body_crc & 0xFF);
        buf[hdr_off + 7] = (u8)(body_crc >> 8);
        buf[hdr_off + 8] = (u8)(chunk_off & 0xFF);
        buf[hdr_off + 9] = (u8)(chunk_off >> 8);
        buf[hdr_off + 10] = (u8)(chunk_len & 0xFF);
        buf[hdr_off + 11] = (u8)(chunk_len >> 8);
        memcpy(&buf[hdr_off + IOCTRL2WCS_SEG_HDR_SIZE], &s_1402_body_ptr[chunk_off], chunk_len);
        data_len = (u16)(IOCTRL2WCS_SEG_HDR_SIZE + chunk_len);

        s_1402_seg_next++;
        if(s_1402_seg_next >= s_1402_seg_total)
        {
            s_1402_tx_active = 0;
        }
        else
        {
            AddSendMsgToQueue(SEND_MSG_IOCTRL2WCS_CMD_TYPE);
        }
    }

    sendlen = (u16)(11 + data_len);
    sum = buf[9];
    for(i = 1; i < sendlen - 9; i++)
    {
        sum ^= buf[9 + i];
    }
    buf[0] = 0xAA;
    buf[1] = 0xAA;
    buf[2] = 0x01;
    buf[3] = 0x00;
    buf[4] = 0x00;
    buf[5] = 0x00;
    buf[6] = sendlen & 0xFF;
    buf[7] = (sendlen >> 8) & 0xFF;
    buf[8] = sum;

    *len = sendlen;
}

void send_msg_ioprt2wcs_cmd(u8 *buf, u16 *len, u16 type)
{
    u8 sum;
    u16 sendlen;
    u16 i;

    sendlen = 11 + 2 + (print_state_upload_buff[0]|(print_state_upload_buff[1]<<8)) * 2;
    buf[9] = type & 0xFF;
    buf[10] = (type >> 8) & 0xFF;
    for(i = 0; i < sendlen - 11 ; i++)
    {
        buf[11 + i] = print_state_upload_buff[i];
    }
    sum = buf[9];
    for(i = 1 ; i < sendlen - 9 ; i++)
    {
        sum ^= buf[9 + i];
    }
    buf[0] = 0xAA;
    buf[1] = 0xAA;
    buf[2] = 0x01;
    buf[3] = 0x00;
    buf[4] = 0x00;
    buf[5] = 0x00;
    buf[6] = sendlen & 0xFF;
    buf[7] = (sendlen >> 8) & 0xFF;
    buf[8] = sum;

    *len = sendlen;
}

void send_msg_slave_can_error_cmd(u8 *buf, u16 *len, u16 type)
{
    u8 sum;
    u16 sendlen;
    u16 i;
    
    buf[9] = type & 0xFF;
    buf[10] = (type >> 8) & 0xFF;
    
    sendlen = 11 + 1 + slave_can_error_num;
    
    for(i = 0 ; i < slave_can_error_num ; i++)
    {  
        buf[12 + i] = upload_can_error[i];
        upload_can_error[i] = 0;
    }
    
    buf[11] = slave_can_error_num;
    slave_can_error_num = 0;  
    
    sum = buf[9];
    for(i = 1 ; i < sendlen - 9 ; i++)
    {
        sum ^= buf[9 + i];
    }
    buf[0] = 0xAA;
    buf[1] = 0xAA;
    buf[2] = 0x01;
    buf[3] = 0x00;
    buf[4] = 0x00;
    buf[5] = 0x00;
    buf[6] = sendlen & 0xFF;
    buf[7] = (sendlen >> 8) & 0xFF;
    buf[8] = sum;

    *len = sendlen;
}

/* 0x142A：从机 PIN/配置异常 — num + (station, reason)*num
 * reason: 1超时 2CRC 3Flash 5参数 6无PIN 7孤儿站
 *         8~15 主机Flash保存失败(station=0)：8未细分 9未ready 10无样板站
 *         11打包失败 12堆不足 13擦写API失败 14读回校验失败 15异类站失败 */
void send_msg_slave_cfg_fail_cmd(u8 *buf, u16 *len, u16 type)
{
    u8 sum;
    u16 sendlen;
    u16 i;
    u8 stations[32];
    u8 reasons[32];
    u8 n = 0;

    cfg_can_fill_fail_list(stations, reasons, 32, &n);

    buf[9] = type & 0xFF;
    buf[10] = (type >> 8) & 0xFF;
    buf[11] = n;
    for(i = 0; i < n; i++)
    {
        buf[12 + i * 2] = stations[i];
        buf[13 + i * 2] = reasons[i];
    }
    sendlen = (u16)(11 + 1 + n * 2);

    sum = buf[9];
    for(i = 1; i < sendlen - 9; i++)
    {
        sum ^= buf[9 + i];
    }
    buf[0] = 0xAA;
    buf[1] = 0xAA;
    buf[2] = 0x01;
    buf[3] = 0x00;
    buf[4] = 0x00;
    buf[5] = 0x00;
    buf[6] = sendlen & 0xFF;
    buf[7] = (sendlen >> 8) & 0xFF;
    buf[8] = sum;
    *len = sendlen;
}


void vpcfun_send_msg_car_online_cmd(u8* buf, u16* len, u16 type)
{
    u8  sum;
    u16 sendlen;
    u16 i;

    sendlen = 11 + 4;
    buf[9] = type & 0xFF;
    buf[10] = (type >> 8) & 0xFF;
    buf[11] = pcconnectmsg.version[3] & 0xFF;
    buf[12] = pcconnectmsg.version[2] & 0xFF;
    buf[13] = pcconnectmsg.version[1] & 0xFF;
    buf[14] = pcconnectmsg.version[0] & 0xFF;

    sum = buf[9];
    for (i = 1; i < sendlen - 9; i++)
    {
        sum ^= buf[9 + i];
    }
    buf[0] = 0xAA;
    buf[1] = 0xAA;
    buf[2] = 0x01;
    buf[3] = 0x00;
    buf[4] = 0x00;
    buf[5] = 0x00;
    buf[6] = sendlen & 0xFF;
    buf[7] = (sendlen >> 8) & 0xFF;
    buf[8] = sum;
    *len = sendlen;
}


void vpcfun_send_validate_version_cmd(u8* buf, u16* len, u16 type)
{
    u8  sum;
    u16 sendlen;
    u16 i;

    buf[9] = type & 0xFF;
    buf[10] = (type >> 8) & 0xFF;

    buf[11] = product_type;
    buf[12] = SOFT_VERSION_LEN;

    sendlen = 11 + 1 + 1 + SOFT_VERSION_LEN;
    memcpy(&buf[13], sds_validate_version, SOFT_VERSION_LEN);

    sum = buf[9];
    for (i = 1; i < sendlen - 9; i++)
    {
        sum ^= buf[9 + i];
    }
    buf[0] = 0xAA;
    buf[1] = 0xAA;
    buf[2] = 0x01;
    buf[3] = 0x00;
    buf[4] = 0x00;
    buf[5] = 0x00;
    buf[6] = sendlen & 0xFF;
    buf[7] = (sendlen >> 8) & 0xFF;
    buf[8] = sum;
    *len = sendlen;
}

void send_msg_port_alarm_cmd(u8 *buf, u16 *len, u16 type, u16 port_no)
{
    u8 sum;
    u16 sendlen = 13;

    buf[9] = type & 0xFF;
    buf[10] = (type >> 8) & 0xFF;
    buf[11] = port_no & 0xFF;
    buf[12] = (port_no >> 8) & 0xFF;
    sum = buf[9] ^ buf[10] ^ buf[11] ^ buf[12];
    buf[0] = 0xAA;
    buf[1] = 0xAA;
    buf[2] = 0x01;
    buf[3] = 0x00;
    buf[4] = 0x00;
    buf[5] = 0x00;
    buf[6] = sendlen & 0xFF;
    buf[7] = (sendlen >> 8) & 0xFF;
    buf[8] = sum;
    *len = sendlen;
}

void unload_check_alarm_process(u8 port_index)
{
    u16 port_no;

    if(port_index >= g_cfg_port_num)
    {
        return;
    }
    port_no = unload_port_assign[port_index].port_no;
    send_msg_port_alarm_cmd(&(tcp_client_list[0].tcp_send_buf[0]),
                            &(tcp_client_list[0].tcp_send_len),
                            SEND_MSG_UNLOAD_CHECK_ALARM_TYPE, port_no);
    tcp_client_list[0].tcp_send_en = 1;
    udp_send_process(0);
}

void print_alarm_process(u8 port_index)
{
    u16 port_no;

    if(port_index >= g_cfg_port_num)
    {
        return;
    }
    /* 协议 0x1404: print_addr_num + print_addr[]，经统一组包发送 */
    port_no = unload_port_assign[port_index].port_no;
    print_state_upload_buff[0] = 1;
    print_state_upload_buff[1] = 0;
    print_state_upload_buff[2] = port_no & 0xFF;
    print_state_upload_buff[3] = (port_no >> 8) & 0xFF;
    AddSendMsgToQueue(SEND_MSG_IOPRT2WCS_CMD_TYPE);
}

void send_heart_msg(u8 *buf, u16 *len, u16 type)
{
    u8 sum = (type & 0xFF) ^ (type >> 8);

    buf[0] = 0xAA;
    buf[1] = 0xAA;
    buf[2] = 0x01;
    buf[3] = 0x00;
    buf[4] = 0x00;
    buf[5] = 0x00;
    buf[6] = 0x0B;
    buf[7] = 0x00;
    buf[8] = sum;
    buf[9] = type & 0xFF;
    buf[10] = (type >> 8) & 0xFF;

    *len = 11;
}

void reply_recv_msg(u8* buf, u16* len, u16 type)
{
    u8 sum = (type & 0xFF) ^ (type >> 8);

    buf[0] = 0xAA;
    buf[1] = 0xAA;
    buf[2] = 0x01;
    buf[3] = 0x00;
    buf[4] = 0x00;
    buf[5] = 0x00;
    buf[6] = 0x0B;
    buf[7] = 0x00;
    buf[8] = sum;
    buf[9] = type & 0xFF;
    buf[10] = (type >> 8) & 0xFF;

    *len = 11;
}

/* 0x9431：result — 0已启动 1参数 2无拓扑 3进行中 */
void reply_recv_msg_station_learn_ack(u8 *buf, u16 *len, u16 type)
{
    u8 result = g_stn_learn_rx_result;
    u8 sum;
    u16 i;

    buf[9] = type & 0xFF;
    buf[10] = (type >> 8) & 0xFF;
    buf[11] = result;
    sum = buf[9];
    for(i = 1; i < 3; i++)
    {
        sum ^= buf[9 + i];
    }
    buf[0] = 0xAA;
    buf[1] = 0xAA;
    buf[2] = 0x01;
    buf[3] = 0x00;
    buf[4] = 0x00;
    buf[5] = 0x00;
    buf[6] = 12;
    buf[7] = 0x00;
    buf[8] = sum;
    *len = 12;
}

/* 0x9426：result — 0成功；2参数 3序号 4溢出 5CRC/解析（与 CFG_PIN_RX_* 一致） */
void reply_recv_msg_pin_ack(u8 *buf, u16 *len, u16 type)
{
    u8 result = g_cfg_pin_rx_last_result;
    u8 sum;
    u16 i;

    /* 成功对 WCS 报 0；内部 CFG_PIN_RX_OK=1 → 对外 0 */
    if(result == CFG_PIN_RX_OK)
    {
        result = 0;
    }
    buf[9] = type & 0xFF;
    buf[10] = (type >> 8) & 0xFF;
    buf[11] = result;
    sum = buf[9];
    for(i = 1; i < 3; i++)
    {
        sum ^= buf[9 + i];
    }
    buf[0] = 0xAA;
    buf[1] = 0xAA;
    buf[2] = 0x01;
    buf[3] = 0x00;
    buf[4] = 0x00;
    buf[5] = 0x00;
    buf[6] = 12;
    buf[7] = 0x00;
    buf[8] = sum;
    *len = 12;
}

/* 0x1432：站号学习完成 summary + num + (station, result)*
 * summary: 0=全成功 1=有失败；result: 0=OK 1=超时 */
void send_msg_station_learn_done_cmd(u8 *buf, u16 *len, u16 type)
{
    u8 stations[CFG_MAX_BOARD];
    u8 results[CFG_MAX_BOARD];
    u8 summary;
    u8 n = 0;
    u8 sum;
    u16 sendlen;
    u16 i;

    cfg_station_learn_fill_report(&summary, stations, results, CFG_MAX_BOARD, &n);
    buf[9] = type & 0xFF;
    buf[10] = (type >> 8) & 0xFF;
    buf[11] = summary;
    buf[12] = n;
    for(i = 0; i < n; i++)
    {
        buf[13 + i * 2] = stations[i];
        buf[14 + i * 2] = results[i];
    }
    sendlen = (u16)(11 + 2 + n * 2);
    sum = buf[9];
    for(i = 1; i < sendlen - 9; i++)
    {
        sum ^= buf[9 + i];
    }
    buf[0] = 0xAA;
    buf[1] = 0xAA;
    buf[2] = 0x01;
    buf[3] = 0x00;
    buf[4] = 0x00;
    buf[5] = 0x00;
    buf[6] = sendlen & 0xFF;
    buf[7] = (sendlen >> 8) & 0xFF;
    buf[8] = sum;
    *len = sendlen;
}

/* 0x1430：V5扩展，交叉带协议无此项。num+(station,flags)*；全站0x03后停。
 * 须连续3次(1s)未全就位才周期上报；现场在线看交叉带5.1的0x1402 */
void send_msg_slave_ready_cmd(u8 *buf, u16 *len, u16 type)
{
    u8 stations[CFG_MAX_BOARD];
    u8 flags[CFG_MAX_BOARD];
    u8 n = 0;
    u8 sum;
    u16 sendlen;
    u16 i;

    cfg_can_fill_ready_list(stations, flags, CFG_MAX_BOARD, &n);
    buf[9] = type & 0xFF;
    buf[10] = (type >> 8) & 0xFF;
    buf[11] = n;
    for(i = 0; i < n; i++)
    {
        buf[12 + i * 2] = stations[i];
        buf[13 + i * 2] = flags[i];
    }
    sendlen = (u16)(11 + 1 + n * 2);
    sum = buf[9];
    for(i = 1; i < sendlen - 9; i++)
    {
        sum ^= buf[9 + i];
    }
    buf[0] = 0xAA;
    buf[1] = 0xAA;
    buf[2] = 0x01;
    buf[3] = 0x00;
    buf[4] = 0x00;
    buf[5] = 0x00;
    buf[6] = sendlen & 0xFF;
    buf[7] = (sendlen >> 8) & 0xFF;
    buf[8] = sum;
    *len = sendlen;
}

/* 0x1429：未就绪 + 缺配位图（bit0=1423 … bit3=1426；1=仍缺） */
void send_msg_conf_lose_cmd(u8 *buf, u16 *len, u16 type)
{
    u8 miss;
    u8 sum;
    u16 i;

    miss = (u8)((~g_cfg_flags) & CFG_FLAG_HOST_READY);
    buf[9] = type & 0xFF;
    buf[10] = (type >> 8) & 0xFF;
    buf[11] = miss;
    sum = buf[9];
    for(i = 1; i < 3; i++)
    {
        sum ^= buf[9 + i];
    }
    buf[0] = 0xAA;
    buf[1] = 0xAA;
    buf[2] = 0x01;
    buf[3] = 0x00;
    buf[4] = 0x00;
    buf[5] = 0x00;
    buf[6] = 12;
    buf[7] = 0x00;
    buf[8] = sum;
    *len = 12;
}
void send_message_to_sever(void)
{
    u16 msg_type;

    if(tcp_client_list[0].tcp_send_en == 0)//tcp发送成功或没在发送
    {
        msg_type = GetSendMsgFromQueue();
    }
    else//tcp正在发送
    {
        return;
    }
 
    switch(msg_type)
    {
    case SEND_MSG_IOCTRL2WCS_VERSION_TYPE:
        vpcfun_send_msg_car_online_cmd(&(tcp_client_list[0].tcp_send_buf[0]), &(tcp_client_list[0].tcp_send_len), msg_type);
        tcp_client_list[0].tcp_send_en = 1;
        break; 
    case SEND_MSG_VALIDATE_VERSION_TYPE:
        vpcfun_send_validate_version_cmd(&(tcp_client_list[0].tcp_send_buf[0]), &(tcp_client_list[0].tcp_send_len), msg_type);
        tcp_client_list[0].tcp_send_en = 1;
        break; 

    case SEND_MSG_IOCTRL2WCS_CMD_TYPE:
        send_msg_ioctrl2wcs_cmd(&(tcp_client_list[0].tcp_send_buf[0]), &(tcp_client_list[0].tcp_send_len), msg_type);
        tcp_client_list[0].tcp_send_en = 1;
        break;
    case SEND_MSG_IOPRT2WCS_CMD_TYPE:
        send_msg_ioprt2wcs_cmd(&(tcp_client_list[0].tcp_send_buf[0]), &(tcp_client_list[0].tcp_send_len), msg_type);
        tcp_client_list[0].tcp_send_en = 1;
        break;
    case REPLAY_RECV_MSG_PORT_CLOSE_CMD_TYPE:
        reply_recv_msg(&(tcp_client_list[0].tcp_send_buf[0]), &(tcp_client_list[0].tcp_send_len), msg_type);
        tcp_client_list[0].tcp_send_en = 1;
        break;
    case REPLY_RECV_MSG_PORT_CLEAR_CMD_TYPE:
    case REPLY_RECV_MSG_SCREEN_DATA_TYPE:
    case REPLY_RECV_RECV_MSG_INIT_FUNC_CONFIG_TYPE:
    case REPLY_RECV_RECV_MSG_INIT_IP_OFFEST_TYPE:
    case REPLY_RECV_RECV_MSG_INIT_TIME_TYPE:
    case REPLY_RECV_MSG_INIT_LOGIC_TYPE:
    case REPLY_RECV_MSG_INIT_COMBO_TYPE:
    case REPLY_RECV_MSG_CFG_STORE_TYPE:
    case REPLY_RECV_MSG_JAM_FANOUT_TYPE:
    case REPLY_RECV_MSG_EXTIO_TYPE:
    case REPLY_RECV_RECV_MSG_EMERGENCY_TYPE:
        reply_recv_msg(&(tcp_client_list[0].tcp_send_buf[0]), &(tcp_client_list[0].tcp_send_len), msg_type);
        tcp_client_list[0].tcp_send_en = 1;
        break;
    case REPLY_RECV_MSG_INIT_PIN_TYPE:
        reply_recv_msg_pin_ack(&(tcp_client_list[0].tcp_send_buf[0]),
                               &(tcp_client_list[0].tcp_send_len), msg_type);
        tcp_client_list[0].tcp_send_en = 1;
        break;
    case REPLY_RECV_MSG_STATION_LEARN_TYPE:
        reply_recv_msg_station_learn_ack(&(tcp_client_list[0].tcp_send_buf[0]),
                                         &(tcp_client_list[0].tcp_send_len), msg_type);
        tcp_client_list[0].tcp_send_en = 1;
        break;
    case SEND_MSG_CONF_LOSE_TYPE://0x1429
        send_msg_conf_lose_cmd(&(tcp_client_list[0].tcp_send_buf[0]),
                               &(tcp_client_list[0].tcp_send_len), msg_type);
        tcp_client_list[0].tcp_send_en = 1;
        break;
    case SEND_MSG_SLAVE_CAN_ERROR_TYPE:
        send_msg_slave_can_error_cmd(&(tcp_client_list[0].tcp_send_buf[0]), &(tcp_client_list[0].tcp_send_len), msg_type);
        tcp_client_list[0].tcp_send_en = 1;
        break;
    case SEND_MSG_SLAVE_CFG_FAIL_TYPE:
        send_msg_slave_cfg_fail_cmd(&(tcp_client_list[0].tcp_send_buf[0]), &(tcp_client_list[0].tcp_send_len), msg_type);
        tcp_client_list[0].tcp_send_en = 1;
        break;
    case SEND_MSG_SLAVE_READY_TYPE:
        send_msg_slave_ready_cmd(&(tcp_client_list[0].tcp_send_buf[0]), &(tcp_client_list[0].tcp_send_len), msg_type);
        tcp_client_list[0].tcp_send_en = 1;
        break;
    case SEND_MSG_STATION_LEARN_DONE_TYPE:
        send_msg_station_learn_done_cmd(&(tcp_client_list[0].tcp_send_buf[0]),
                                        &(tcp_client_list[0].tcp_send_len), msg_type);
        tcp_client_list[0].tcp_send_en = 1;
        break;
    default:
        if( tcp_client_list[0].tcp_client_statue == CLIENT_CONNECT_OK)
        {
            if( heart_delay == 0 )
            {
                send_heart_msg(&(tcp_client_list[0].tcp_send_buf[0]), &(tcp_client_list[0].tcp_send_len), HEART_MSG_TYPE);
                tcp_client_list[0].tcp_send_en = 1;
                heart_delay = HEART_DELAY;
            }
        }
        break;
    }
}
