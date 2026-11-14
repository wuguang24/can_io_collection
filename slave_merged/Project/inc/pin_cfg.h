#ifndef _PIN_CFG_H_
#define _PIN_CFG_H_

#include "main.h"

#define PIN_CFG_PORT_MAX        8
#define PIN_CFG_JAM_MAX         12
#define PIN_CFG_REC_SIZE_V24    24u  /* 无 OR 掩码 */
#define PIN_CFG_REC_SIZE_OR_JAM4 27u /* jam×4 + OR 掩码 */
#define PIN_CFG_REC_SIZE_JAM12  43u  /* jam×12 + OR（无满包档脚） */
#define PIN_CFG_REC_SIZE        45u  /* 与主机 cfg_pin_rec_t 一致 */
#define PIN_CFG_SLICE_HDR_V3    3u
#define PIN_CFG_SLICE_HDR_EXT    5u  /* ver+n+board_flags_u16 */
#define PIN_CFG_PIN_MAX         8
#define PIN_CFG_INVALID         0xFFu

/* 与主机 cfg_table.h CFG_BOARD_F_* 对齐 */
#define PIN_BOARD_F_PROX_MUX        (1u << 0)  /* IN2 高→IN1满包，低→IN0满包 */
#define PIN_BOARD_F_SIDE_GATE       (1u << 1)  /* 偶数喷码格口 OR IN5 满包 */

#pragma pack(1)
/*
 * Same layout as host cfg_pin_rec_t.
 * in_* and out_* : physical pin 0..7 (0xFF=unbound).
 * out_alarm_* : 对应主机逻辑 CH2..CH5。
 * in_or_*_mask : 附加物理脚位图，与主脚 OR 后编入状态位图。
 * Slave only packs filtered DI into status bitmap; lights via host STD 0x100.
 */
typedef struct
{
    u16 port_no;
    u8  station;
    u8  in_lock, in_print, in_full_pack, in_half_full, in_clamp_pack, in_unload_check;
    u8  out_lock_light, out_print_light, out_alarm_red, out_alarm_yellow, out_alarm_green, out_alarm_buzzer;
    u8  jam_extra_num;
    u16 jam_extra_no[PIN_CFG_JAM_MAX];
    u8  in_or_full_mask;
    u8  in_or_lock_mask;
    u8  in_or_print_mask;
    u8  in_full_tier1;
    u8  in_full_tier2;
} pin_cfg_rec_t;
#pragma pack()

extern u8  g_pin_ready;
extern u8  g_pin_n;
extern u16 g_pin_ver;
extern u16 g_pin_board_flags;
extern pin_cfg_rec_t g_pin_rec[PIN_CFG_PORT_MAX];
extern u16 g_pin_filter_ms[PIN_CFG_PIN_MAX];

/* body = [ver_u16][n][flags_u16?][n x rec] without CRC; 0 = OK */
u8 pin_cfg_apply(const u8 *body, u16 body_len);
void pin_cfg_clear(void);
void pin_cfg_load_from_flash(void);
u16 pin_cfg_filter_for_pin(u8 pin);
void pin_cfg_on_time_updated(void);

#endif
