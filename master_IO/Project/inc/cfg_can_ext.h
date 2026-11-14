#ifndef _CFG_CAN_EXT_H_
#define _CFG_CAN_EXT_H_

#include "stm32f10x.h"

/*
 * F107 EXT 配置下发 + CFG_ACK(0x4) 闭环：
 * - TIME：广播 5×u16+CRC
 * - PIN：按 1423 拓扑站下发切片，等从机 kind=3 ACK；超时重试
 * - 重绑：WCS 再发 0x1426 → 主机 hot_apply 全量重推 PIN，从机覆盖 Flash
 * - 从机无 PIN：ACK result=1 → 记入失败表，UDP 0x142A；
 *   并对「运行表有格口」的站自动单站补发（2s 冷却）
 *
 * 从机 ACK result：0=OK  1=无PIN  2=CRC  3=Flash  5=参数
 */
void cfg_can_ext_init(void);
void cfg_can_ext_poll(void);                 /* 1ms：ACK 超时重试 / 收尾 */
void cfg_can_start_distribute(void);
void cfg_can_abort_distribute(void);
u8 cfg_can_is_busy(void);
u8 cfg_can_fail_count(void);
void cfg_can_fill_fail_list(u8 *stations, u8 *reasons, u8 max_n, u8 *out_n);
/* flags: bit0=PIN_OK bit1=COS在线；二者均有 = 0x03 表示该站就位 */
#define CFG_SLAVE_READY_PIN   0x01u
#define CFG_SLAVE_READY_COS   0x02u
#define CFG_SLAVE_READY_FULL  (CFG_SLAVE_READY_PIN | CFG_SLAVE_READY_COS)

void cfg_can_fill_ready_list(u8 *stations, u8 *flags, u8 max_n, u8 *out_n);
/* 拓扑内每站 flags==0x03 时返回 1（运行期心跳由 0x1402 承担，不再周期 1430） */
u8 cfg_can_topo_all_ready(void);

/* can_bus EXT 收包钩子 */
void cfg_can_on_cfg_ack(u8 station, u8 kind, u8 result);

/* 1423↔1426 站号一致性；返回孤儿格口所在站数（写入失败表 reason=ORPHAN） */
u8 cfg_can_check_station_consistency(void);

#endif
