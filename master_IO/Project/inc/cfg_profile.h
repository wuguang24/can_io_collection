#ifndef _CFG_PROFILE_H_
#define _CFG_PROFILE_H_

/*
 * 主机 RAM / 容量编译档（IAR CCDefines 覆盖）：
 *   默认 128 — 常规现场 Σlpc≤128，与 64KB SRAM + FreeRTOS 12KB 堆更匹配
 *   CFG_BIZ_PORT_MAX=256 — 大线在工程选项追加该宏
 *
 * 与 FreeRTOS 关系：格口主表在 BSS；1426 解析缓冲、Flash 写页缓存在堆上临时申请。
 * 改档后请用 .map 核对剩余 RAM（F107 64KB 建议保持 128）。
 */
#ifndef CFG_BIZ_PORT_MAX
#define CFG_BIZ_PORT_MAX        128
#endif

#if (CFG_BIZ_PORT_MAX != 128) && (CFG_BIZ_PORT_MAX != 256)
#error "CFG_BIZ_PORT_MAX must be 128 or 256"
#endif

/* 1402 上报体上限：与端口档无关，shadow/分片固定 2KB */
#define CFG_UPLOAD_BODY_MAX     2048u

#endif /* _CFG_PROFILE_H_ */
