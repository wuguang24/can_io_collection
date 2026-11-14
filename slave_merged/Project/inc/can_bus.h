#ifndef __CAN_BUS_H
#define __CAN_BUS_H
#pragma pack (1)

#define MANUAL_VALID   0
#define MANUAL_INVAILD 1

#define OPERATION_INVALID 0
#define OPERATION_VALID   1 

#define CAN_PACK_DATA_LEN 8
#define PORT_STATE_FLAG_SYNC          0x80u /* 主机 PORT_STATE data[0]：请求从机回 CONFIRM */
#define CAN_RX_DATA_PK_SIZE     256
#define CAN_RX_BUFF_SIZE        300
enum
{
    CAN_FUNC_ID_HEART_STATE = 0x1,
    CAN_FUNC_ID_PORT_STATE = 0x2,
    CAN_FUNC_ID_PRINT_STATE = 0x3,
    CAN_FUNC_ID_CFG_ACK = 0x4,           /* 与主机 CAN_FUNC_ID_CFG_ACK 对齐 */
    CAN_FUNC_ID_STATION_LEARN     = 0x12, /* 站号自学习 */
    CAN_FUNC_ID_PORT_COS_TRIG     = 0x10, /* 状态变化触发（短防抖位图） */
    CAN_FUNC_ID_PORT_COS_CONF     = 0x11, /* 状态确认（1425 滤波后位图） */
    CAN_FUNC_ID_SCREEN_DATA  = 0x6,
    CAN_FUNC_ID_INIT_FUNC_DATA  = 0x7,
    CAN_FUNC_ID_INIT_TIME_DATA  = 0x8,   /* 5×u16 + CRC16 LE */
    CAN_FUNC_ID_LOCK_UNLOCK  = 0x9,      /* 主机急停广播亦用 0x9 + 子命令 0xA1 */
    CAN_FUNC_ID_ALARM_CTRL   = 0xA,
    CAN_FUNC_ID_PHOTO_TRIGGER = 0xB,
    CAN_FUNC_ID_BLOCK_ALARM  = 0xC,
    CAN_FUNC_ID_MANUAL_STATUS = 0xD,
    CAN_FUNC_ID_INIT_PIN_DATA = 0xE,     /* 与主机一致；进 Boot 仅用 0xF */
    CAN_FUNC_ID_BOOT_MODE = 0xF,
    CAN_FUNC_ID_CTRL1_CMD = 0xA1         /* 急停载荷子命令（非 ExtID.func_id） */
};

/* 与从机 BOOT 一致：仅 ENTER_BOOT 让 APP 跳转，擦写/传包由 BOOT 处理 */
#define DLOAD_CMD_ENTER_BOOT    0xB0

enum
{
    CAN_SEG_POLO_NONE   = 0,
    CAN_SEG_POLO_FIRST  = 1,
    CAN_SEG_POLO_MIDDLE = 2,
    CAN_SEG_POLO_FINAL  = 3
};
/*
|    28~22    |    21~20      |    19~12     |     11~7      |    6~0      |    ExtID
|  dst_id(7)  |  seg_polo(2)  |  seg_num(8)  |  func_id(5)   |  src_id(7)  |
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
    u8  recv_buff[CAN_RX_BUFF_SIZE];
    u16 recv_len;
    u8  g_SegPolo;
    u8  g_SegNum;
    u16 g_SegBytes;
}sCanRecv;

#pragma pack (1)
typedef struct
{
    u16 duration;
    u8  alarm_type;
    u8  active;
    u16 tick;
}sAlarmCtrl;
#pragma pack ()

typedef struct
{
    sCanFrameExt* queue;
    u16 front, rear, len;
    u16 maxsize;
}sCanFrameQueue;

extern u32 port_enable_state;
extern u32 port_print_state;
extern u32 slot_full_state;
extern u32 pack_full_state;
extern u32 full_tier1_state;
extern u32 full_tier2_state;
extern u32 port_door_state;
extern u16 port_block_state;

extern u16 manual_flag;
extern u16 manual_operation;
extern sCanRecv canRecv[];
//CAN2
void vcanbus_initcansendqueue_two(void);
void vcanbus_addto_cansendqueue_two(sCanFrameExt x);
void vcanbus_framereceive_two(CanRxMsg rxMsg);
void vcan_sendmsg_two(u8* buff, u16 send_total_len, u8 type, u8 dst);
void vcan_sendframe_process_two(void);

void can_bus_send_slave_heart_state(void);
void can_bus_send_port_print_state(void);
void slave_cfg_announce_pin(void); /* 上电告知主机 PIN 是否就绪 */
void slave_cfg_announce_pin_periodic(void); /* 无 PIN 时周期公告 */
void slave_recv_lock_unlock(u8* data, u8 len);
void slave_recv_alarm_ctrl(u8* data, u8 len);
void slave_send_photo_trigger(u16 port_io);
void slave_send_block_alarm(u16 port_io);
void slave_alarm_ctrl_process(void);

extern sAlarmCtrl alarm_ctrl[];
extern u8 upstream_can;
extern u8 downstream_can;

//CAN1
extern sCanFrameQueue   canframequeue_one;
void vcanbus_initcansendqueue_one(void);
void vcanbus_addto_cansendqueue_one(sCanFrameExt x);
void vcanbus_framereceive_one(CanRxMsg rxMsg);
void vcan_sendmsg_one(u8* buff, u16 send_total_len, u8 type, u8 dst);
void vcan_sendframe_process_one(void);

void can_bus_frame_receive(CanRxMsg rxMsg, u8 can_channel);

#endif
