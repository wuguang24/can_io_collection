#ifndef _CAN_H_
#define _CAN_H_
#include "main.h"
#include "queue.h"

#define M_CTL_CMD_OFFSET    (0x000)
#define M_IO_CTRL_OFFSET    (0x100)
#define S_IO_INFO_OFFSET    (0x200)
#define S_IO_HEART_OFFSET   (0x300)

#define M_CTL_CMD_ID(x)      (M_CTL_CMD_OFFSET + x)
#define S_M_CTRL_ID(x)       (M_IO_CTRL_OFFSET + x)
#define S_IO_INFO_ID(x)      (S_IO_INFO_OFFSET + x)
#define S_HEART_INFO_ID(x)   (S_IO_HEART_OFFSET + x)

#define M_HEAR_BEAT_ID      (0x010)
#define M_BOOT_JUMP_CMD_ID  (0x011)
#define M_GET_VERSION_ID    (0x012)

typedef struct
{
    uint32_t id;
    uint8_t  data[8];
    uint8_t  len;
} can_frame_t;

typedef enum
{
    CAN_CH_0 = 0,
    CAN_CH_1,
    CAN_CH_MAX,
} can_ch_em;

extern queue_st can_recv_queue[CAN_CH_MAX];
extern queue_st can_send_queue[CAN_CH_MAX];
extern can_frame_t cur_send_frame[CAN_CH_MAX];
extern CAN_TypeDef* can_handle[CAN_CH_MAX];
extern u8 can_send_channel;

void can_init(void);
void can_send_frame(u8 ch, can_frame_t* frame);
void can_recv_frame(u8 ch, CanRxMsg* RxMsg);
void can_recv_process(void);
void can_send_process(void);
u8 can_std_bridge_tx(u8 recv_ch, CanRxMsg* rx);

#endif
