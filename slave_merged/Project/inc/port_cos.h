#ifndef _PORT_COS_H_
#define _PORT_COS_H_

#include "main.h"

#define PORT_COS_TRIG_DEBOUNCE_MS   (15u)
#define PORT_COS_CONFIRM_HOLD_MS    (25u)

/* COS 8B 载荷：[0]半满 [1]满 [2]卡 [3]门 [4]锁 [5]打印 [6]档1 [7]档2；seq 在 ExtID.seg_num */
void port_cos_pack(u8 seq, u8 *out8);
void port_cos_poll(void);
void port_cos_send_confirm(u8 seq);
void port_cos_force_sync_reply(void);
void port_cos_reset(void);

#endif
