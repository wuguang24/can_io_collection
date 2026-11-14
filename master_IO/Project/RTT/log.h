#ifndef _LOG_H_
#define _LOG_H_

#include "cfg_rtt.h"

#define BDS_COLOR_TAG           "BDSCOL"
#define BDS_LOG_COLOR_BLACK     0x000000
#define BDS_LOG_COLOR_RED       0xFF3030
#define BDS_LOG_COLOR_GREEN     0x008B00
#define BDS_LOG_COLOR_BLUE      0x00008B
#define BDS_LOG_COLOR_VIOLET    0x9932CC

#if CFG_USE_RTT

#include "SEGGER_RTT.h"

#define rtt_init() \
    SEGGER_RTT_Init()

#define rtt_print_blue(format, ...) \
    SEGGER_RTT_printf(0, BDS_COLOR_TAG"(%d)"format, BDS_LOG_COLOR_BLUE, ##__VA_ARGS__)

#define rtt_print_red(format, ...) \
    SEGGER_RTT_printf(0, BDS_COLOR_TAG"(%d)"format, BDS_LOG_COLOR_RED, ##__VA_ARGS__)

#else

/* 关闭时：宏为空操作；同时 RTT .c 里另有 SEGGER_RTT_* 空桩防 Li005 */
#define rtt_init()                  ((void)0)
#define rtt_print_blue(...)         ((void)0)
#define rtt_print_red(...)          ((void)0)

#endif /* CFG_USE_RTT */

#endif /* _LOG_H_ */
