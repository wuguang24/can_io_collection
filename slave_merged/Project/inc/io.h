#ifndef IO_H
#define IO_H

/*
 * 从机哑 IO：物理 IN/OUT 由 pin_cfg + io_ctrl 驱动；
 * 灯效仅听主机 STD 0x100+sta（io_ctrl_out），不在本机演算。
 */

#define PORT_NUM            8

#define INPUT_TRIG_DOWN     0
#define INPUT_TRIG_UP       1
#define INPUT_TRIG_NULL     2

#define FUNC_PORT           0
#define BLOCK_GROUP_ONE     1
#define BLOCK_GROUP_TWO     2

#pragma pack(1)

/* 遗留滤波结构（InputPortScanProc 已空，保留类型供链接） */
typedef struct
{
    u8  input_state;
    u8  input_middle_state;
    u8  level;
    u8  count_flag;
    u16 input_confirm_times;
    u16 up_cnt_value;
    u16 down_cnt_value;
} sInput_Info;

/* 0x1423 FUNC 下发 → bfunc，上电从 slave_cfg 或 Flash 单页恢复 */
typedef struct
{
    u16 use_mode;
    u16 same_group_f2_sta;
    u16 local_block_group;
    u16 same_group_f1_total;
    u16 same_group_f1_group[20];
} sFuncConfig;

/* 0x1425 TIME → timfunc */
typedef struct
{
    u16 lock_filter_time;
    u16 full_filter_time;
    u16 block_filter_time;
    u16 door_filter_time;
    u16 solt_filter_time;
} sTimeConfig;

#pragma pack()

/*
 * 主机经 PORT_STATE / 0x9 下发的远程关格位图（bit0~7 对应本板 local_port）。
 * WritePortStateBit 将其 OR 进 port_enable_state 上报，与物理锁格键一致。
 */
extern u8 g_port_remote_lock;

extern sFuncConfig bfunc;
extern sTimeConfig timfunc;
extern sInput_Info loca;

void io_para_init_config(void);
void InputPortScanProc(void);
void OutPutCtrlProc(void);
void WritePortStateBit(void);
void port_state_ctrl(u8 *data);
void io_open_all(void);
void io_remote_lock_port(u8 port, u8 locked);
void io_remote_lock_all(u8 locked);
void input_filter(sInput_Info *state, u8 cur, u16 filterTime);

#endif
