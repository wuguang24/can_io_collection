#ifndef _FUN_H
#define _FUN_H

#include "main.h"

/********命令定义**********/
#define		MSG_NULL_TYPE			        0x0000
/**********心跳命令***********/
#define         HEART_MSG_TYPE                          0x1150
/**********主动发送命令*********/
#define     SEND_MSG_ONLINE_CMD_TYPE                0x1403
#define     SEND_MSG_IOCTRL2WCS_CMD_TYPE            0x1402
#define     SEND_MSG_IOPRT2WCS_CMD_TYPE             0x1404
#define     SEND_MSG_SLAVE_CAN_ERROR_TYPE           0x1413
#define     SEND_MSG_IOCTRL2WCS_VERSION_TYPE        0x140B
#define     SEND_MSG_CONF_LOSE_TYPE                 0x1429
#define     SEND_MSG_SLAVE_CFG_FAIL_TYPE            0x142A /* 从机 PIN/配置失败：num+(sta,reason)* */
#define     SEND_MSG_SLAVE_READY_TYPE               0x1430 /* 拓扑从机就位：num+(sta,flags)* */
#define     SEND_MSG_STATION_LEARN_DONE_TYPE        0x1432 /* 站号学习完成：summary+num+(sta,rslt)* */
#define     SEND_MSG_VALIDATE_VERSION_TYPE          0x142C
#define     SEND_MSG_PHOTO_TRIG_TYPE                0x1501
#define     SEND_MSG_UNLOAD_CHECK_ALARM_TYPE        0x1501

/**********接收命令*********/
#define     RECV_MSG_ONLINE_CMD_TYPE                0x9403
#define     RECV_MSG_BOOT_CMD_TYPE                  0x1F01
#define     RECV_MSG_PORT_CLOSE_CMD_TYPE            0x1405
#define     RECV_MSG_PORT_CLEAR_CMD_TYPE            0x1406
#define     RECV_REPLY_MSG_IOPRT2WCS_CMD_TYPE       0x9404
#define     RECV_MSG_SCREEN_DATA_TYPE               0x1422
#define     RECV_RECV_MSG_INIT_FUNC_CONFIG_TYPE     0x1423
#define     RECV_RECV_MSG_INIT_IP_OFFEST_TYPE       0x1424
#define     RECV_RECV_MSG_INIT_TIME_TYPE            0x1425
#define     RECV_MSG_INIT_PIN_TYPE                      0x1426 /* 物理脚↔功能/逻辑通道绑定 */
#define     RECV_MSG_INIT_LOGIC_TYPE                    0x1427 /* 仅更新灯效矩阵；完整规则请用 0x1428 */
#define     RECV_MSG_INIT_COMBO_TYPE                    0x1428 /* 灯效：功能码1组合规则 / 2矩阵 */
#define     RECV_MSG_CFG_STORE_TYPE                     0x142D /* 本地模板/异类固化 */
#define     RECV_MSG_JAM_FANOUT_TYPE                    0x142E /* 夹件 jam_fanout 增删改 */
#define     RECV_MSG_EXTIO_TYPE                         0x142F /* 上报扩展+跨站公共OUT */
#define     RECV_MSG_STATION_LEARN_TYPE                 0x1431 /* 触发 CAN 站号自学习（拨码00从机） */
#define     RECV_RECV_MSG_EMERGENCY_TYPE            0x142B//WCS转发急停-T260811
/**********回复命令*********/
#define     REPLAY_RECV_MSG_PORT_CLOSE_CMD_TYPE           0x9405
#define     REPLY_RECV_MSG_PORT_CLEAR_CMD_TYPE            0x9406
#define     REPLY_RECV_MSG_SCREEN_DATA_TYPE               0x9422
#define     REPLY_RECV_RECV_MSG_INIT_FUNC_CONFIG_TYPE     0x9423
#define     REPLY_RECV_RECV_MSG_INIT_IP_OFFEST_TYPE       0x9424
#define     REPLY_RECV_RECV_MSG_INIT_TIME_TYPE            0x9425
#define     REPLY_RECV_MSG_INIT_PIN_TYPE                  0x9426
#define     REPLY_RECV_MSG_INIT_LOGIC_TYPE                0x9427
#define     REPLY_RECV_MSG_INIT_COMBO_TYPE                0x9428
#define     REPLY_RECV_MSG_CFG_STORE_TYPE                 0x942D
#define     REPLY_RECV_MSG_JAM_FANOUT_TYPE                0x942E
#define     REPLY_RECV_MSG_EXTIO_TYPE                     0x942F
#define     REPLY_RECV_MSG_STATION_LEARN_TYPE             0x9431
#define     REPLY_RECV_RECV_MSG_EMERGENCY_TYPE            0x942B
#define     REPLY_RECV_MSG_BOOT_CMD_TYPE                  0x9F01 /* 应答 0x1F01，进 Boot 前先回 */
#define     HEART_DELAY    5

#define     INT_16_NULL      0xFFFF


#define MAX_GROUP  20//最大组数
#define MAX_STATION_PER_GROUP 20//最大每组个数，20

#pragma pack (1) 
typedef struct {
	u8  MSG_TAG[2];
        u32 MSG_ID;
        u16 MSG_LENGTH;
	u8  MSG_CRC;
	u16 MSG_TYPE;
}MSG_HEAD_DATA;

typedef struct {
  
    u8   version[4];                       //版本号
    
}sBD2PC_Connect_msg;

typedef struct
{
    u8 board_num;
    u8 addr;
    u8 type;//1格口、2入袋件数、3格口流向、4目的地代码
    u8 total_len;
    u8 scr_buf[50];
} sScreenDate;

#pragma pack () 


typedef struct {
    u16* queue;
    u16 front, rear, len;
    u16 maxsize;
}sINT_16_QUEUE;

typedef struct {
        u16 *queue; /* 指向存储队列的数组空间 */
        u16 front, rear, len; /* 队首指针（下标），队尾指针（下标），队列长度变量 */
        u16 maxSize; /* queue数组长度 */
}MSG_SEND_QUEUE;

typedef struct
{
    sScreenDate *queue;
    u8 front, rear ,len;
    u16 maxsize;
} sScreenQueue;


/*BD INIT MSG*/
extern sBD2PC_Connect_msg           pcconnectmsg;
extern sINT_16_QUEUE    openmsgqueue;
extern sINT_16_QUEUE    closemsgqueue;
extern sScreenQueue     screenqueue;
extern u16 ip_offest_start;
extern u16 ip_offest_end;
void fun_Init_INT_16_Queue(void);
void InitSendMsgQueue(void);
void AddSendMsgToQueue(u16 msg);
u16 GetSendMsgFromQueue(void);
void recv_message_from_sever(u8 *point,u16 *len);
void send_message_to_sever(void);
void reply_recv_msg(u8* buf, u16* len, u16 type);
void send_heart_msg(u8 *buf, u16 *len, u16 type);
void vpcfun_init_sys_item(void);
u16 fun_get_msg_from_int_16_queue(sINT_16_QUEUE* q);
void fun_recv_screen_msg(u8* point);
sScreenDate* fun_get_from_screen_queue(sScreenQueue* q);
void fun_add_to_int_16_Queue(sINT_16_QUEUE* q, u16 x);
void unload_check_alarm_process(u8 port_index);
void print_alarm_process(u8 port_index);

/* 0x1402 上报：数据区>256 时分片；发送未完成前勿覆盖快照 */
#define IOCTRL2WCS_UPLOAD_BODY_MAX   2048
#define IOCTRL2WCS_SEG_THRESHOLD     256
#define IOCTRL2WCS_SEG_MAGIC         0xFFFE
#define IOCTRL2WCS_SEG_HDR_SIZE      12
#define IOCTRL2WCS_SEG_CHUNK_MAX     (IOCTRL2WCS_SEG_THRESHOLD - IOCTRL2WCS_SEG_HDR_SIZE)

u8   fun_ioctrl2wcs_tx_busy(void);
void fun_ioctrl2wcs_prepare_tx(const u8 *body, u16 body_len);
void send_msg_ioctrl2wcs_cmd(u8 *buf, u16 *len, u16 type);
#endif
