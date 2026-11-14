#ifndef IO_CTRL_H
#define IO_CTRL_H

typedef struct {
    u8  input_state;
    u8  input_middle_state;
    u8  level;
    u8  count_flag;
    u16 input_confirm_times;
    u16 up_cnt_value;
    u16 down_cnt_value;
} input_info_st;

extern u8 io_ctrl_in;
extern u16 io_ctrl_out;
void io_ctrl_process(void);
u8 io_ctrl_get(u8 index);
void io_ctrl_set(u8 index, BitAction BitVal);

#endif
