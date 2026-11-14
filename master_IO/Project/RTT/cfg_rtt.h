#ifndef _CFG_RTT_H_
#define _CFG_RTT_H_

/*
 * 工程级 RTT 总开关（只改这一处）
 *
 * 1 = 编译 SEGGER_RTT*.c，启用 rtt_print_* / RTT 缓冲（调试）
 * 0 = 不编译 RTT 实现与缓冲，打印为空操作（发布：省 Flash/RAM，减轻调度压力）
 */
#ifndef CFG_USE_RTT
#define CFG_USE_RTT                 0
#endif

#endif /* _CFG_RTT_H_ */
