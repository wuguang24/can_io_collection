#ifndef _STATION_LEARN_H_
#define _STATION_LEARN_H_

#include "stm32f10x.h"

#define STN_LRN_SUB_DISCOVER    0x01u
#define STN_LRN_SUB_ASSIGN      0x02u
#define STN_LRN_SUB_PASS        0x03u
#define STN_LRN_SUB_ACK         0x81u
#define STN_LRN_SUB_DONE        0x82u

#define STN_LRN_RSLT_OK         0u  /* 该站 DONE 应答正确 */
#define STN_LRN_RSLT_TIMEOUT    1u  /* 超时无应答 */
#define STN_LRN_MSG_DONE        0x1432u /* 学习完成主动上报（与 fun.h 一致） */

void cfg_station_learn_init(void);
void cfg_station_learn_start(void);
void cfg_station_learn_poll(void);
u8   cfg_station_learn_done(void);
u8   cfg_station_learn_active(void);
void cfg_station_learn_on_rx(u8 src_id, const u8 *data, u8 len);
void cfg_station_learn_fill_report(u8 *summary, u8 *stations, u8 *results,
                                   u8 max_n, u8 *out_n);

#endif
