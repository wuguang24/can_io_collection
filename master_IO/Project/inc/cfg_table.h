#ifndef _CFG_TABLE_H_
#define _CFG_TABLE_H_

#include "io.h"

#define CFG_PORT_MAX            CFG_BIZ_PORT_MAX
#define CFG_STATION_MAX         99
#define CFG_PIN_REC_SIZE_V24    24u   /* 1426 记录：无 OR 掩码 */
#define CFG_PIN_REC_SIZE_OR_JAM4 27u  /* jam×4 + OR 掩码 */
#define CFG_PIN_REC_SIZE_JAM12  43u   /* jam×12 + OR 掩码（无满包档） */
#define CFG_PIN_REC_SIZE        45u   /* jam×12 + OR + tier1/tier2 脚 */
#define CFG_PIN_SLICE_HDR_V3    3u
#define CFG_PIN_SLICE_HDR_EXT   5u    /* ver+n+board_flags_u16 */

/* format2 每板 flags（下发从机切片头 + 主机保存） */
#define CFG_BOARD_F_NONE            0u
#define CFG_BOARD_F_PROX_MUX        (1u << 0)  /* phase1: IN2 选 IN0/IN1 */
#define CFG_BOARD_F_SIDE_GATE       (1u << 1)  /* phase1: 偶侧附加满包脚 */

#define CFG_KIND_FUNC           1
#define CFG_KIND_TIME           2
#define CFG_KIND_PIN            3

#define CFG_FAIL_TIMEOUT        1
#define CFG_FAIL_CRC            2
#define CFG_FAIL_FLASH          3
#define CFG_FAIL_SEG            4
#define CFG_FAIL_PARAM          5
#define CFG_FAIL_NO_PIN         6   /* 从机无有效 PIN 表 */
#define CFG_FAIL_ORPHAN         7   /* 1426 站号不在 1423 拓扑 */
#define CFG_FAIL_AUTOSAVE       8   /* 主机 Flash 保存失败（station=0，未细分时） */
#define CFG_FAIL_SAVE_NOT_READY 9   /* 未 ready 或 g_cfg_port_num=0 */
#define CFG_FAIL_SAVE_NO_STA    10  /* 样板站号 <2 */
#define CFG_FAIL_SAVE_PACK      11  /* pack_station / build_tpl 失败 */
#define CFG_FAIL_SAVE_HEAP      12  /* s_wr 堆分配失败 */
#define CFG_FAIL_SAVE_FLASH     13  /* Flash 擦除/编程 API 失败 */
#define CFG_FAIL_SAVE_VERIFY    14  /* 写后读回 CRC/seq 校验失败 */
#define CFG_FAIL_SAVE_EXCEPT    15  /* 异类站打包失败或超过 8 块 */

#define CFG_FLAG_1423           (1u << 0)
#define CFG_FLAG_1424           (1u << 1)
#define CFG_FLAG_1425           (1u << 2)
#define CFG_FLAG_1426           (1u << 3)
#define CFG_FLAG_HOST_READY     (CFG_FLAG_1423 | CFG_FLAG_1424 | CFG_FLAG_1425 | CFG_FLAG_1426)

/* 与 fun.h MAX_GROUP / MAX_STATION_PER_GROUP 对齐 */
#define CFG_MAX_GROUP           20
#define CFG_MAX_STATION_PER_GROUP 20
#define CFG_BLOCK_GROUP_ONE     1
#define CFG_BLOCK_GROUP_TWO     2

#define CFG_PIN_FMT_SLOT        1
#define CFG_PIN_FMT_BIND        2
#define CFG_PIN_FMT_BIND_MERGE  3   /* format2 编码，按站合并进运行表（不整表清空、不拓扑展开） */

#define CFG_IN_FUNC_NONE            0
#define CFG_IN_FUNC_LOCK            1
#define CFG_IN_FUNC_PRINT           2
#define CFG_IN_FUNC_LOCK_PRINT      3
#define CFG_IN_FUNC_FULL            4
#define CFG_IN_FUNC_HALF            5
#define CFG_IN_FUNC_CLAMP           6
#define CFG_IN_FUNC_UNLOAD_CHECK    7
#define CFG_IN_FUNC_IGNORE          8
#define CFG_IN_FUNC_FULL_TIER1      9   /* 满包档1（较低阈值）→ 1402 扩展组 */
#define CFG_IN_FUNC_FULL_TIER2      10  /* 满包档2（较高阈值）→ 1402 扩展组 */

/* 0x1426 输出功能码：表示「该物理脚驱动哪路逻辑通道」，脚号本身是 0~7 */
#define CFG_OUT_FUNC_NONE           0
#define CFG_OUT_FUNC_LOCK_LIGHT     1  /* → CH0 */
#define CFG_OUT_FUNC_PRINT_LIGHT    2  /* → CH1 */
#define CFG_OUT_FUNC_LOCK_PRINT_LIGHT 3 /* → CH0+CH1（合并键灯） */
#define CFG_OUT_FUNC_RED            4  /* → CH2 告警通道 */
#define CFG_OUT_FUNC_YELLOW         5  /* → CH3 */
#define CFG_OUT_FUNC_GREEN          6  /* → CH4 */
#define CFG_OUT_FUNC_BUZZER         7  /* → CH5 */

#define CFG_BIND_F_ACTIVE_LOW       (1u << 0)
/* 多脚 OR 进同一功能（满包/锁格/打印/锁打；输出同通道第二脚）；与 1428 信号 and/or/not 分层 */
#define CFG_BIND_F_OR_FULL          (1u << 1)
#define CFG_BIND_F_OR               CFG_BIND_F_OR_FULL
#define CFG_BIND_LOCAL_BOARD        0xFF  /* 板级：夹件 IN 或公共 OUT 绑到本站全部 local_port */

#define CFG_MAX_BOARD               96
#define CFG_MAX_BIND_PER_BOARD      16  /* 含板级公共 OUT，略放宽 */
#define CFG_MAX_LOCAL_PORT          8
#define CFG_MAX_JAM_FANOUT          12  /* 与 CFG_JAM_EXTRA_MAX 对齐；83~61 奇数组需 11 */
/*
 * 0x1426 format2 暂存：只放「样板 + 异类」原始板片段，其它站由主机 remap 展开后
 * 再按站透传 CAN PIN，无需把 15×8 全量塞进 RAM。
 *
 * 上位机「一块样板复制 N 站」：工程可有 N 板，UDP 仍只发 1 份样板（+异类）；
 * 主机不硬吃 N 份。超限 → CFG_PIN_RX_OVERFLOW，运行表不变。
 * 上限 ≈ 1 + 样板200 + 异类8×200；RX 与 Flash 拼包共用一块 workspace。
 */
#define CFG_BIND_BLOB_MAX           2048
extern u8 g_cfg_bind_workspace[CFG_BIND_BLOB_MAX];

/* 0x142F：上报扩展标志 + 跨站公共 OUT 路由 */
#define CFG_XOUT_MAX                16
#define CFG_UPLOAD_F_HALF           (1u << 0)  /* 同 CFG_REPORT_HALF_FULL */
#define CFG_UPLOAD_F_FULL_TIER      (1u << 2)  /* 同 CFG_REPORT_FULL_TIER：1402 追加满包档1/档2 */
#pragma pack(1)
typedef struct
{
    u8 station; /* 塔灯所在从站 ≥2 */
    u8 pin;     /* 物理 OUT0~7 */
    u8 src_ch;  /* 逻辑 CH0~5：取全机该通道 mode 最大值写入目标脚 */
    u8 flags;   /* 预留 */
} cfg_xout_route_t;
#pragma pack()

/* 0x142E：运行态编辑夹件 jam_fanout（不重发整包 1426） */
#define CFG_JAM_OP_REPLACE          1
#define CFG_JAM_OP_ADD              2
#define CFG_JAM_OP_REMOVE           3
#define CFG_JAM_OP_CLEAR            4

#pragma pack(1)
/* WCS 0x1426 单板 PIN 记录（format2 落盘 / cfg_store 样板片段） */
typedef struct
{
    u16 port_no;
    u8  station;
    u8  in_lock, in_print, in_full_pack, in_half_full, in_clamp_pack, in_unload_check;
    u8  out_lock_light, out_print_light, out_alarm_red, out_alarm_yellow, out_alarm_green, out_alarm_buzzer;
    u8  jam_extra_num;
    u16 jam_extra_no[CFG_MAX_JAM_FANOUT];
    u8  in_or_full_mask;   /* 附加满包脚位掩码（与主脚 OR，不含主脚） */
    u8  in_or_lock_mask;
    u8  in_or_print_mask;
    u8  in_full_tier1;
    u8  in_full_tier2;
} cfg_pin_rec_t;

/* 0x1426 绑定条目：物理脚 + 功能码 + 本板格口下标 */
typedef struct
{
    u8 pin;         /* IN/OUT 0~7 */
    u8 func;        /* CFG_IN_FUNC_* / CFG_OUT_FUNC_* */
    u8 local_port;  /* 0~7 或 CFG_BIND_LOCAL_BOARD(0xFF) 表板级公共脚 */
    u8 flags;       /* CFG_BIND_F_ACTIVE_LOW / CFG_BIND_F_OR_* */
} cfg_pin_bind_t;

/* 0x1423 拓扑中的从站节点 */
typedef struct
{
    u8 station;      /* 从站号 ≥2 */
    u8 block_group;  /* 卡包分组（0x1423 FUNC 遗留，与 jam 扩组独立） */
} cfg_slave_node_t;

/*
 * 0x1423 初始化拓扑 → g_cfg_init，决定全机从站列表与分组。
 * 上电 cfg_store 或 WCS 下发后用于 PIN 分发与 autosave 站号枚举。
 */
typedef struct
{
    u8  slave_num;       /* 从站总数 */
    u8  group_total;     /* 分组数 */
    u8  expand_master;   /* 扩展主机站号 */
    u8  reserved;
    struct
    {
        u8 each_total;
        cfg_slave_node_t each[CFG_MAX_STATION_PER_GROUP];
    } group[CFG_MAX_GROUP];
} cfg_init_conf_t;

/*
 * 0x1425 滤波时间（ms）→ g_cfg_time，io_handle 经 SYSTEM_TIME 换算为采样计数。
 * lock=锁格键  full=满包  block=夹件  solt=打印键  door=门磁（预留）
 */
typedef struct
{
    u16 lock_filter_time;
    u16 full_filter_time;
    u16 block_filter_time;
    u16 door_filter_time;
    u16 solt_filter_time;
} cfg_time_t;
#pragma pack()

extern unload_port_assign_st unload_port_assign[];
extern u16 g_cfg_port_num;
extern u16 g_cfg_ver;
extern u16 ip_offest_start;
extern u16 ip_offest_end;
extern cfg_time_t g_cfg_time;
extern cfg_init_conf_t g_cfg_init;
extern u8 g_cfg_flags;
extern u8 g_cfg_host_ready;

void cfg_table_init(void);
u8 cfg_table_apply_default(void);
u8 cfg_table_set_from_recs(const cfg_pin_rec_t *recs, u16 num, u16 ver, u16 crc);
u8 cfg_table_set_from_bind_blob(const u8 *blob, u16 blob_len, u16 ver, u16 crc);
u8 cfg_table_merge_from_bind_blob(const u8 *blob, u16 blob_len, u16 ver, u16 crc);
u16 cfg_table_calc_crc(void);
void cfg_table_build_station_slice(u8 station, u8 *out, u16 *out_len, u16 *out_crc);
u16 cfg_find_port_index(u16 port_no);
u8 cfg_table_parse_init_func(const u8 *payload, u16 payload_len);
void cfg_set_flag(u8 bit);
u8 cfg_host_ready(void);
/* 1423 权威：从 g_cfg_init 刷新 ioCtrl_num / ext_host_flag */
void cfg_table_apply_io_ctrl_from_init(void);
u8   cfg_table_topo_station_count(void);
u8   cfg_table_topo_station_at(u8 idx);

extern volatile u8 g_cfg_io_freeze;

#define CFG_PIN_RX_IDLE_MS      3000
/* cfg_pin_rx_segment 返回值 / 0x9426 result */
#define CFG_PIN_RX_MORE         0   /* 继续等后续片，不答 */
#define CFG_PIN_RX_OK           1   /* 收齐并已应用 */
#define CFG_PIN_RX_BAD_PARAM    2   /* 参数/格式非法 */
#define CFG_PIN_RX_SEQ          3   /* 片序号乱序或会话中断 */
#define CFG_PIN_RX_OVERFLOW     4   /* 暂存溢出 */
#define CFG_PIN_RX_PARSE        5   /* 收齐后 CRC/解析失败 */
u8 cfg_pin_rx_segment(u16 field0, u16 ver, u16 crc,
                      u16 seg_total, u16 seg_index, u16 seg_bytes, const u8 *data);
extern u8 g_cfg_pin_rx_last_result; /* 最近一次 1426 结果，供 0x9426 */
void cfg_pin_rx_tick_1ms(void);
void cfg_pin_rx_abort(void);
void cfg_runtime_hot_apply(void);

/*
 * 0x142E payload:
 *   station u8, op u8, count u8, ports[count] u16 LE
 * op: REPLACE/ADD/REMOVE/CLEAR — 改该站所有格口的 jam_extra（夹件连带上报）
 * 返回 1=成功
 */
u8 cfg_jam_fanout_edit(u8 station, u8 op, u8 count, const u16 *ports);

/* 0x142F：upload_flags u16 + xout_count u8 + routes[count]*4 */
u8 cfg_extio_set_from_payload(const u8 *payload, u16 len);
u16 cfg_upload_flags_effective(void); /* 142F | 1428.report_flags */

extern u8  g_cfg_in_full_or[][CFG_IN_OR_MAX];
extern u8  g_cfg_in_full_or_n[];
extern u8  g_cfg_in_lock_or[][CFG_IN_OR_MAX];
extern u8  g_cfg_in_lock_or_n[];
extern u8  g_cfg_in_print_or[][CFG_IN_OR_MAX];
extern u8  g_cfg_in_print_or_n[];
extern u8  g_cfg_in_full2[]; /* full_or 首元镜像，便于 1426 回写 OR 掩码 */
extern u8  g_cfg_out_lock2[];
extern u8  g_cfg_out_print2[];
extern u8  g_cfg_port_flags[];
extern u16 g_cfg_print_delay_ms[];
extern u16 g_cfg_upload_flags;
extern u8  g_cfg_xout_n;
extern cfg_xout_route_t g_cfg_xout[];
extern u16 g_cfg_board_flags[MAX_SLAVE_NUM];
extern u8  g_cfg_jam_block_skip[UNLOAD_PORT_NUM];

void cfg_rebuild_jam_skip(void);

#endif
