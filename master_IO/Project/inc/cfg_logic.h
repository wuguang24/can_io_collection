#ifndef _CFG_LOGIC_H_
#define _CFG_LOGIC_H_

#include "stm32f10x.h"

#define CFG_LM_OFF      0
#define CFG_LM_ON       1
#define CFG_LM_FAST     2
#define CFG_LM_SLOW     3

#define CFG_SIG_LOCKED  (1u << 0)
#define CFG_SIG_PRINT   (1u << 1)
#define CFG_SIG_FULL    (1u << 2)
#define CFG_SIG_HALF    (1u << 3)
#define CFG_SIG_CLAMP   (1u << 4)
#define CFG_SIG_CLOSED  (1u << 5)  /* WCS 0x1405 关口/远程锁 */

/*
 * 逻辑通道 CH0~CH5（共 6 路，≠ 从板物理 OUT0~7）：
 *   1428 规则写 out_ch=0..5 → 本模块算出 mode → io 再按 1426 写到物理脚。
 */
#define CFG_CH_LOCK     0
#define CFG_CH_PRINT    1
#define CFG_CH_OUT2     2
#define CFG_CH_OUT3     3
#define CFG_CH_OUT4     4
#define CFG_CH_OUT5     5
#define CFG_CH_RED      CFG_CH_OUT2
#define CFG_CH_YELLOW   CFG_CH_OUT3
#define CFG_CH_GREEN    CFG_CH_OUT4
#define CFG_CH_BUZZER   CFG_CH_OUT5
#define CFG_CH_NUM      6

#define CFG_BTN_TRIG    0
#define CFG_BTN_LEVEL   1

#define CFG_LOGIC_FUNC_COMBO    1
#define CFG_LOGIC_FUNC_MATRIX   2

#define CFG_PRINT_GATE_REQUIRE_LOCK   (1u << 0)
#define CFG_PRINT_GATE_SUPPRESS_ALARM (1u << 1)
#define CFG_REPORT_HALF_FULL          (1u << 0)  /* 1402 追加半满地址组 */
#define CFG_REPORT_OLB_PAD            (1u << 1)  /* 预留 */
#define CFG_REPORT_FULL_TIER          (1u << 2)  /* 1402 追加满包档1/档2 两组 */
#define CFG_COND_IDLE   0
#define CFG_COND_HALF   1
#define CFG_COND_FULL   2
#define CFG_COND_CLAMP  3
#define CFG_COND_NUM    4

/* 单条规则线格式长度（pack 后含 and/or/not 三个 u16） */
#define CFG_COMBO_RULE_MAX      32
#define CFG_COMBO_RULE_SIZE     10
#define CFG_LOGIC_HDR_SIZE      16
#define CFG_LOGIC_MATRIX_SIZE   (2 * CFG_COND_NUM * CFG_CH_NUM)

#pragma pack(1)
/* 0x1428 头（16B）+ 规则体；Flash 模板与运行时 g_cfg_logic.hdr 共用布局 */
typedef struct
{
    u16 profile_ver;
    u8  button_mode;       /* CFG_BTN_TRIG / CFG_BTN_LEVEL */
    u8  lock_print_mode;
    u8  print_gate;        /* CFG_PRINT_GATE_* */
    u8  cond_priority;     /* 多条件冲突时优先级 */
    u8  report_flags;      /* CFG_REPORT_* → 1402 扩展组 */
    u8  rule_count;        /* 0=仅矩阵/全灭 */
    u16 flash_fast_ms;     /* IO_OUT_MODE_FAST 周期 */
    u16 flash_slow_ms;
    u16 print_delay_ms;
    u16 release_filter_ms; /* DI 释放沿缩短滤波（ms） */
} cfg_logic_hdr_t;

/* 单条组合规则：out_ch=CH0~5，mode=CFG_LM_*，信号掩码见 CFG_SIG_* */
typedef struct
{
    u8  out_ch;
    u8  mode;
    u8  prio;
    u8  reserved;
    u16 and_mask;
    u16 or_mask;
    u16 not_mask;
} cfg_combo_rule_t;

typedef struct
{
    cfg_logic_hdr_t hdr;
    u8  matrix[2][CFG_COND_NUM][CFG_CH_NUM]; /* 0x1427 矩阵：[锁态][条件]→通道模式 */
    cfg_combo_rule_t rules[CFG_COMBO_RULE_MAX];
    u8  ready;
} cfg_logic_t;
#pragma pack()

extern cfg_logic_t g_cfg_logic;

void cfg_logic_init(void);
void cfg_logic_runtime_reset(void);
/* 空配置：无上位机规则时全灭；灯效仅由 0x1428/0x1427 灌入 */
void cfg_logic_load_empty(void);

u8 cfg_logic_set_from_payload(const u8 *payload, u16 len);
u8 cfg_combo_set_from_payload(const u8 *payload, u16 len);
/* func=CFG_LOGIC_FUNC_* ，payload 不含功能码本身 */
u8 cfg_logic_set_by_func(u8 func, const u8 *payload, u16 len);

void cfg_logic_apply_port(u8 i);

u16 cfg_logic_flash_fast_ms(void);
u16 cfg_logic_flash_slow_ms(void);
u16 cfg_logic_release_filter_ms(void);

#endif
