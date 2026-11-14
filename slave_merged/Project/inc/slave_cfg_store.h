#ifndef _SLAVE_CFG_STORE_H_
#define _SLAVE_CFG_STORE_H_

#include "main.h"

#define SLAVE_CFG_BANK_A_ADDR       0x08028800u
#define SLAVE_CFG_BANK_B_ADDR       0x08029000u
#define SLAVE_CFG_MAGIC             0xCF61u
#define SLAVE_CFG_VER               2u
#define SLAVE_CFG_PIN_MAX           260u
#define SLAVE_CFG_TIME_BYTES        10u
#define SLAVE_CFG_FUNC_BYTES        48u

#define SLAVE_CFG_APPLIED_PIN       (1u << 0)
#define SLAVE_CFG_APPLIED_TIME      (1u << 1)
#define SLAVE_CFG_APPLIED_FUNC      (1u << 2)

u8 slave_cfg_commit_pin(const u8 *pin_body, u16 pin_len);
u8 slave_cfg_commit_time(const u8 *time10);
u8 slave_cfg_commit_func(const u8 *func48);
u8 slave_cfg_boot_load(void);
u8 slave_cfg_migrate_from_flash_pages(void);
u8 slave_cfg_applied_mask(void);

#endif
