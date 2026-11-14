#ifndef __CAN_BUS_H
#define __CAN_BUS_H
#include "main.h"

#define CAN_SEND_FRAME_LEN    64   /* EXT 待发队列深度；PIN 分发逐站 ACK，64 足够 */

#define CAN_PACK_DATA_LEN     8

#define ALL_BOARD             0x6F
#define PORT_STATE_FLAG_SYNC  0x80u   /* PORT_STATE data[0]：请求从机回 COS CONFIRM */
#define COS_SYNC_PERIOD_MS    500u    /* 每站慢同步间隔（与 main CAN_POLL 配合） */

#pragma pack (1)

enum
{
    CAN_FUNC_ID_HEART_STATE  = 0x1,
    CAN_FUNC_ID_PORT_STATE  = 0x2,
    CAN_FUNC_ID_PRINT_STATE = 0x3,
    /* 0x4：从机 CFG_ACK（kind,result）；主机解析，见 cfg_can_on_cfg_ack */
    CAN_FUNC_ID_SCADA_STATE = 0x4,
    CAN_FUNC_ID_CFG_ACK     = 0x4,
    CAN_FUNC_ID_STATION_LEARN     = 0x12, /* 站号自学习 DISCOVER/ASSIGN/PASS */
    CAN_FUNC_ID_PORT_COS_TRIG     = 0x10, /* 从机→主机：触发态位图 */
    CAN_FUNC_ID_PORT_COS_CONF     = 0x11, /* 从机→主机：确认态位图 */
    CAN_FUNC_ID_SCREEN_DATA  = 0x6,
    CAN_FUNC_ID_INIT_FUNC_DATA  = 0x7,
    CAN_FUNC_ID_INIT_TIME_DATA  = 0x8, /* 载荷 5×u16 + CRC16 LE，共 12B，与从机/GD 一致 */
    CAN_FUNC_ID_ENABLE_ALL_FULL = 0x9, /* 广播急停等；子命令区分 LOCK_UNLOCK */
    CAN_FUNC_ID_MANUAL_STATUS = 0xD,
    CAN_FUNC_ID_INIT_PIN_DATA = 0xE,  /* 站切片 PIN 表；进 Boot 只用 0xF */
    CAN_FUNC_ID_ENTER_BOOT  = 0xF,
};

enum//广播09的子funid
{
    CAN_FUNC_ID_CTRL1_CMD  = 0xA1, /* 全场急停子命令 */
    CAN_FUNC_ID_CTRL2_CMD  = 0xA2,
    CAN_FUNC_ID_CTRL3_CMD = 0xA3,
    CAN_FUNC_ID_CTRL4_CMD = 0xA4,
};

enum
{
    CAN_SEG_POLO_NONE   = 0,
    CAN_SEG_POLO_FIRST  = 1,
    CAN_SEG_POLO_MIDDLE = 2,
    CAN_SEG_POLO_FINAL  = 3
};
/*
|    28~22    |    21~20      |    19~12     |     11~7      |    6~0      |
|  dst_id(7)  |  seg_polo(2)  |  seg_num(8)  |  func_id(5)*  |  src_id(7)  |
|  *本主机打包用 func_id&0xF（低 4bit）；0x1~0xF 与从机一致
*/
typedef struct
{
    u8 seg_polo;
    u8 seg_num;
    u8 func_id;
    u8 src_id;
    u8 dst_id;
} sCanFrameExtID;

typedef struct
{
    sCanFrameExtID extId;
    u8 data_len;
    u8 data[8];
} sCanFrameExt;

typedef struct
{
    u32 port_enable_state;
    u32 port_print_state;
    u32 port_pack_full_state;
    u32 port_door_state;
    u16 port_block_state;
    u8  port_half_full_state; /* 从机 Data[0]：半满位图（与满包分离） */
    u8  port_full_tier1_state; /* 从机 Data[7]：满包档1 */
    u8  port_full_tier2_state; /* 从机 Data[8]：满包档2 */
} sPortState;




#pragma pack ()

typedef struct {
    sCanFrameExt* queue;
    u16 front, rear, len;
    u16 maxsize;
}sCanFramequeue;


extern u8 port_state_upload_buff[];
extern u16 g_port_state_upload_len;
extern u8 print_state_upload_buff[];
extern u8 close_cmd;

extern u8 slave_can_error_num;
extern u8 upload_can_error[];

extern u8 full_tier_active; /* 142F bit2：满包多档上报已启用 */
extern sPortState portState[];

/* COS CONFIRM 新鲜度：超时标 stale，0x1402 跳过离线站 */
void can_bus_port_state_tick_1ms(void);
u8  can_bus_port_state_stale(u8 sta_idx);
u16 can_bus_port_state_stale_ms(void);
void can_bus_port_state_reset_all(void);
void can_bus_request_sync_all(void);
u8  can_bus_slave_cos_alive(u8 sta_idx);

void can_bus_frame_receive(CanRxMsg rxMsg);
void can_bus_send_cycle_5ms(void); /* 5ms：灯控+下行命令；状态由 COS 事件上报 */
void can_bus_set_port_print_reply(u8* point);
void can_bus_set_port_close_cmd(void);
void can_bus_set_port_clear_cmd(u8* point);
void can_bus_forward_screen_cmd(void);
void can_bus_set_port_time_cmd(u8* point);
void can_bus_send_io_output(u8 station); /* STD 0x100+sta：物理 OUT0~7 各 2bit 模式 → 从机 io_ctrl_out */
void upload2WCS(void);
void upload_slave_can_error_cmd(void);

void vcan_bus_frame_queue_init(void);
void vcan_addto_frame_queue(sCanFrameExt x);
void vcan_send_send_msg(u8* buff, u16 send_total_len, u8 type, u8 dst);
void vcan_send_frame_process(void);
u8 u8Is_feame_queue_free(void);

#endif