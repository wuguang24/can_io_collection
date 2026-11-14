#ifndef _STATION_LEARN_H_
#define _STATION_LEARN_H_

#include "main.h"

#define STN_LRN_SUB_DISCOVER    0x01u
#define STN_LRN_SUB_ASSIGN      0x02u
#define STN_LRN_SUB_PASS        0x03u
#define STN_LRN_SUB_ACK         0x81u
#define STN_LRN_SUB_DONE        0x82u

#define STN_LRN_BKP_MAGIC       0xA55Au
#define STN_LRN_BKP_DR_MAGIC    BKP_DR5
#define STN_LRN_BKP_DR_STA      BKP_DR6

void station_learn_init(void);
void station_learn_apply_dip(void);
u8   station_learn_on_frame(u8 can_channel, u8 src_id, u8 dst_id,
                            const u8 *data, u8 len);
void station_learn_save_station(u8 station);

#endif
