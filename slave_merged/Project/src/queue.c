#include "main.h"
#include "queue.h"
#include "string.h"

void queue_init(queue_st* p_queue, u8* buf, u16 data_num, u16 data_size)
{
    memset(p_queue, 0x00, sizeof(queue_st));
    p_queue->buff_len = data_size * data_num;
    memset(buf, 0x00, p_queue->buff_len);
    p_queue->data = buf;
    p_queue->front = 0;
    p_queue->rear = 0;
    p_queue->max_num = data_num;
    p_queue->data_size = data_size;
}

u8 queue_in(queue_st* p_queue, u8* p_buf, u16 len)
{
    u16 cnt = 0;

    if((p_queue->rear + p_queue->data_size) % p_queue->buff_len == p_queue->front)
    {
        return 0;
    }

    for(; cnt < len; cnt++)
    {
        p_queue->rear = (p_queue->rear + p_queue->data_size) % p_queue->buff_len;
        memcpy(&p_queue->data[p_queue->rear], &p_buf[cnt * p_queue->data_size], p_queue->data_size);
    }
    return 1;
}

u8 queue_out(queue_st* p_queue, u8* p_buf, u16* p_len)
{
    u16 cnt = 0;

    for(; cnt < *p_len; cnt++)
    {
        if(p_queue->front != p_queue->rear)
        {
            p_queue->front = (p_queue->front + p_queue->data_size) % p_queue->buff_len;
            memcpy(&p_buf[cnt * p_queue->data_size], &p_queue->data[p_queue->front], p_queue->data_size);
        }
        else
        {
            break;
        }
    }
    *p_len = cnt;
    return 1;
}

u16 queue_get_data_size(queue_st* p_queue)
{
    if(p_queue->rear > p_queue->front)
    {
        return (p_queue->rear - p_queue->front) / p_queue->data_size;
    }
    else if(p_queue->front > p_queue->rear)
    {
        return (p_queue->buff_len - p_queue->front + p_queue->rear) / p_queue->data_size;
    }
    return 0;
}
