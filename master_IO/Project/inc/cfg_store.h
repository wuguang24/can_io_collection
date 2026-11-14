#ifndef _CFG_STORE_H_
#define _CFG_STORE_H_

#include "stm32f10x.h"

/*
 * 主机本地配置 Flash（STM32F107 末 8KB，页 2KB）：
 *
 *   Bank0: TPL @ 0x0803C000  +  EXC @ 0x0803D000
 *   Bank1: TPL @ 0x0803E000  +  EXC @ 0x0803F000
 *
 * TPL = 1423 拓扑 + 1424 偏移 + 1425 滤波 + 样板 PIN 片段 + 1428 灯效 + 142F 扩展
 * EXC = 最多 8 块异类板的 PIN 覆盖
 *
 * 写入：autosave / 0x142D 选「非活跃 Bank」，TPL 与 EXC 同一 store_seq 成对提交；
 *       store_seq 在 tpl 头（reserved=0 视为 seq=0）。
 * 读取：上电 cfg_store_refresh_active() 取 store_seq 最大且 CRC 合法的一对。
 */
#define CFG_FLASH_PAGE_SIZE         2048u
#define CFG_FLASH_BANK0_TPL_ADDR    0x0803C000u
#define CFG_FLASH_BANK0_EXC_ADDR    0x0803D000u
#define CFG_FLASH_BANK1_TPL_ADDR    0x0803E000u
#define CFG_FLASH_BANK1_EXC_ADDR    0x0803F000u

#define CFG_STORE_MAGIC_TPL         0xC6F1u
#define CFG_STORE_MAGIC_EXC         0xC6E8u
#define CFG_STORE_EXCEPT_MAX        8
#define CFG_STORE_BOARD_BLOB_MAX    200

/* 0x142D op */
#define CFG_STORE_OP_SAVE_TPL       1
#define CFG_STORE_OP_SAVE_EXCEPT    2
#define CFG_STORE_OP_CLEAR_EXCEPT   3
#define CFG_STORE_OP_APPLY          4
#define CFG_STORE_OP_CLEAR_TPL      5

void cfg_store_refresh_active(void);
void cfg_store_boot_try_apply(void); /* main：仅 refresh+置位，不展开配置 */
void cfg_store_boot_finish(void);    /* CanIo 首圈：从 Flash 恢复并下发 PIN */
u8 cfg_store_save_template(u8 tpl_station);
u8 cfg_store_save_except(u8 station);
u8 cfg_store_clear_except(u8 station); /* 0xFF=全部 */
u8 cfg_store_clear_template(void);
u8 cfg_store_apply(void);
u8 cfg_store_handle_cmd(const u8 *payload, u16 payload_len);

/* WCS 热改配置后置位；PIN 全站 ACK 成功后自动写 Flash，掉电保持重绑结果 */
void cfg_store_mark_runtime_dirty(void);
u8 cfg_store_try_autosave(void); /* 1=已保存或无需保存  0=保存失败 */
extern u8 g_cfg_store_autosave_fail; /* 0=无；非0=最近保存失败原因(8~15)，142A station=0 */

#endif
