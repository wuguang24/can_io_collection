#ifndef _IO_H_
#define _IO_H_
#include "main.h"
#include "cfg_profile.h"

/*
 * 全机业务口槽位上限（Σ 各站 local_port_count）；与从机数 MAX_SLAVE_NUM 无关。
 * 单站本地口仍 ≤8。默认 128，大线在 IAR CCDefines 加 CFG_BIZ_PORT_MAX=256。
 */
#define UNLOAD_PORT_NUM         CFG_BIZ_PORT_MAX
/* 夹件触发时 0x1402 额外上报 / 组内扩信号：由 1426 jam_fanout 配置 */
#define CFG_JAM_EXTRA_MAX       (12)
/* 同一功能多物理脚 OR：主脚在 assign.in_*，此处为附加脚（如满包 IN0|IN1|IN6） */
#define CFG_IN_OR_MAX           (3)

#define IO_LVL_OFF              (0)
#define IO_LVL_ON               (1)

#define IO_TRIG_DOWN            (0)
#define IO_TRIG_UP              (1)
#define IO_TRIG_NONE            (2)

#define IO_STT_OFF              (0)
#define IO_STT_ON               (1)

#define IO_OUT_MODE_OFF         (0)
#define IO_OUT_MODE_ON          (1)
#define IO_OUT_MODE_FAST        (2)
#define IO_OUT_MODE_SLOW        (3)

/* 输入端口枚举：从板物理 IN0~7（共 8 路） */
typedef enum
{
    INPUT_0 = 0,
    INPUT_1,
    INPUT_2,
    INPUT_3,
    INPUT_4,
    INPUT_5,
    INPUT_6,
    INPUT_7,
    INPUT_MAX,
    INPUT_INVALID,
} input_em;

/* 输出端口枚举：从板物理 OUT0~7（共 8 路）；与 cfg_logic 的 CH0~CH5 不是同一层 */
typedef enum
{
    OUTPUT_0 = 0,
    OUTPUT_1,
    OUTPUT_2,
    OUTPUT_3,
    OUTPUT_4,
    OUTPUT_5,
    OUTPUT_6,
    OUTPUT_7,
    OUTPUT_MAX,
    OUTPUT_INVALID,
} output_em;

/* 单路 DI 滤波状态（仅在对应从站 CAN 快照到达时按间隔 ms 推进） */
typedef struct {
    u32 level:1;          /* 稳定电平：IO_LVL_OFF / IO_LVL_ON */
    u32 trig:1;           /* 边沿：IO_TRIG_UP / IO_TRIG_DOWN / IO_TRIG_NONE */
    u32 stt:1;            /* 锁格等业务锁存（由 cfg_logic 置位） */
    u32 change_cnt:14;    /* 与 level 不一致的连续采样次数 */
    u32 continue_cnt:14;  /* 与 level 一致的连续采样次数 */
    u32 reserve:1;
} input_info_st;

/* 逻辑输出通道模式，最终经 set_output_state 写到从机物理 OUT */
typedef struct {
    u8 mode:2;            /* IO_OUT_MODE_OFF/ON/FAST/SLOW */
    u8 reserve:6;
} output_info_st;

/*
 * 单个业务格口的运行态（与 unload_port_assign 一一对应）。
 * 输入侧按功能语义命名；输出侧 *_light 存逻辑 CH0~CH5 的模式，alarm_* 对应 CH2~CH5。
 */
typedef struct
{
    input_info_st lock_button;         /* 锁格键（及 OR 附加脚） */
    input_info_st print_button;        /* 打印键 */
    input_info_st full_pack_photo;     /* 满包光电 */
    input_info_st half_full_photo;     /* 半满光电（上报扩展，本地可未绑脚） */
    input_info_st clamp_pack_photo;    /* 夹件光电 */
    input_info_st unload_check_photo;  /* 落格检测 */
    output_info_st lock_button_light;   /* CH0 锁格灯 */
    output_info_st print_button_light;  /* CH1 打印灯 */
    output_info_st alarm_red_light;     /* CH2 */
    output_info_st alarm_yellow_light;  /* CH3 */
    output_info_st alarm_green_light;   /* CH4 */
    output_info_st alarm_buzzer_light;  /* CH5 蜂鸣 */

    u8 port_disable_stt:1;  /* 1=格口禁用（锁格/上位机关闭等） */
    u8 port_print_stt:1;    /* 本周期发生打印事件，供 0x1402 上报 */
    u8 port_clamp_stt:1;    /* 夹件扩组标志 */
    u8 port_close_stt:1;    /* 上位机关闭命令锁存 */
    u8 port_clear_stt:1;    /* 清状态请求 */
    u8 reserve:3;
} unload_port_st;

/*
 * 格口 IO 绑定表 — cfg_table 根据 WCS 0x1426 展开，上电可由 cfg_store Flash 恢复。
 * in_* / out_*：从板物理脚 0~7；INPUT_INVALID/OUTPUT_INVALID 表示未绑定。
 * jam_extra_no：夹件时额外联动上报/灯效的邻格 port_no 列表（0x142E 可热改）。
 */
typedef struct
{
    u16 port_no;           /* 全机唯一格口号 */
    u16 station;           /* 从站号（≥2） */
    u8  local_bit;         /* 本站 8 路格口位图中的 bit，对齐从机 PIN local_port */
    u8 in_lock;
    u8 in_print;
    u8 in_full_pack;
    u8 in_half_full;
    u8 in_clamp_pack;
    u8 in_unload_check;
    u8 in_full_tier1;      /* 满包档 1（较低阈值）→ 0x1402 扩展组 */
    u8 in_full_tier2;      /* 满包档 2 */

    u8 out_lock_light;     /* 逻辑 CH0 绑定的物理 OUT */
    u8 out_print_light;    /* CH1 */
    u8 out_alarm_red;      /* CH2 */
    u8 out_alarm_yellow;   /* CH3 */
    u8 out_alarm_green;    /* CH4 */
    u8 out_alarm_buzzer;   /* CH5 */

    u8 jam_extra_num;
    u16 jam_extra_no[CFG_JAM_EXTRA_MAX];
} unload_port_assign_st;

#define CFG_PORT_F_LOCK_PRINT_MERGED  (1u << 0)

/* 附加 OR 脚：与 assign 主脚一起采样（对齐「多点位→同一功能」） */
extern u8  g_cfg_in_full_or[][CFG_IN_OR_MAX];
extern u8  g_cfg_in_full_or_n[];
extern u8  g_cfg_in_lock_or[][CFG_IN_OR_MAX];
extern u8  g_cfg_in_lock_or_n[];
extern u8  g_cfg_in_print_or[][CFG_IN_OR_MAX];
extern u8  g_cfg_in_print_or_n[];
/* full_or 首元快捷访问（与 g_cfg_in_full_or[i][0] 同步） */
extern u8  g_cfg_in_full2[];
/* 同功能第二物理 OUT（flags bit1 OR）：如双锁打键各带一盏灯 */
extern u8  g_cfg_out_lock2[];
extern u8  g_cfg_out_print2[];
extern u8  g_cfg_port_flags[];
extern u16 g_cfg_print_delay_ms[];
/* jam_fanout 扩到的夹件灯效信号（非光电本体） */
extern u8  g_io_clamp_fanout[];

extern u8 io_port_in[];
extern u16 io_port_out[];

extern unload_port_st unload_port[];
extern unload_port_assign_st unload_port_assign[];

void io_port_init(void);
u8 read_input_state(u8 station, u8 port);
/* 同站同脚取更高 mode（公共灯多格口合写，避免后写冲掉） */
void set_output_state(u8 station, u8 port, u8 mode);
void io_ctrl_proc(void);
void io_port_state_ctrl(u8 *data);
void io_open_all(void);

/* CAN 打印事件写入逻辑层输入 */
void io_note_print_event(u8 station);

/* COS 握手：CONFIRM 直写电平；TRIGGER 仅预显示（不上报 WCS） */
void io_on_cos_confirm(u8 sta_idx);
void io_on_cos_trig(u8 sta_idx, const u8 *body7);

#endif
