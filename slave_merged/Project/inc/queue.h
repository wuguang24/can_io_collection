#ifndef _QUEUE_H_
#define _QUEUE_H_
#include "main.h"

typedef struct {
    u8* data;
    u16 front, rear;
    u16 data_size;
    u16 max_num;
    u16 buff_len;
} queue_st;

void queue_init(queue_st* p_queue, u8* buf, u16 data_num, u16 data_size);
u8 queue_in(queue_st* p_queue, u8* p_buf, u16 len);
u8 queue_out(queue_st* p_queue, u8* p_buf, u16* p_len);
u16 queue_get_data_size(queue_st* p_queue);

#endif
