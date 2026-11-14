#ifndef _FAULT_LED_H_
#define _FAULT_LED_H_

#include "main.h"

/*
 * AW20036 C1R1 状态灯故障码（现场无 WCS 时可读）：
 *   0 慢闪 1Hz     正常
 *   1  3闪+停      无 PIN / 空表
 *   3  4闪+停      PIN 站号与本机拨码不符
 *   4  5闪+停      上电后主机未通信
 *   5  常亮        拨码站号 < 2
 *   6  快闪 4Hz    双 CAN 长时间无收包
 */
#define FAULT_LED_CODE_OK           0u
#define FAULT_LED_CODE_NO_PIN       1u
#define FAULT_LED_CODE_PIN_STA      3u
#define FAULT_LED_CODE_NO_HOST      4u
#define FAULT_LED_CODE_STA_BAD      5u
#define FAULT_LED_CODE_CAN_SILENT   6u

#define FAULT_LED_HOST_GRACE_MS     (30000u)
#define FAULT_LED_CAN_SILENT_MS     (15000u)

void fault_led_init(void);
void fault_led_poll_1ms(void);
u8   fault_led_get_code(void);
void fault_led_on_can_rx(u8 src_id, u8 dst_id);

#endif
