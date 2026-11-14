#include "fault_led.h"
#include "pin_cfg.h"
#include "aw_i2c.h"

extern u8 Local_Station;

#define FAULT_GAP_MS        (2000u)
#define FAULT_FLASH_ON_MS   (150u)
#define FAULT_FLASH_OFF_MS  (150u)
#define FAULT_SLOW_MS       (500u)
#define FAULT_FAST_MS       (125u)

static u16 s_ms;
static u8  s_host_seen;
static u16 s_can_last_ms;
static u8  s_pat_ms;
static u8  s_pat_phase;   /* 0=间隔 1=亮 2=灭 */
static u8  s_pat_idx;
static u8  s_pat_target;

static u8 pin_station_mismatch(void)
{
    u8 i;

    if(!g_pin_ready || g_pin_n == 0)
    {
        return 0;
    }
    for(i = 0; i < g_pin_n; i++)
    {
        if(g_pin_rec[i].station != 0 && g_pin_rec[i].station != Local_Station)
        {
            return 1;
        }
    }
    return 0;
}

static u8 fault_led_resolve_code(void)
{
    if(Local_Station < 2)
    {
        return FAULT_LED_CODE_STA_BAD;
    }
    if(pin_station_mismatch())
    {
        return FAULT_LED_CODE_PIN_STA;
    }
    if(!g_pin_ready || g_pin_n == 0)
    {
        return FAULT_LED_CODE_NO_PIN;
    }
    if(s_ms > FAULT_LED_HOST_GRACE_MS && !s_host_seen)
    {
        return FAULT_LED_CODE_NO_HOST;
    }
    if(s_ms > FAULT_LED_CAN_SILENT_MS
       && (u16)(s_ms - s_can_last_ms) > FAULT_LED_CAN_SILENT_MS)
    {
        return FAULT_LED_CODE_CAN_SILENT;
    }
    return FAULT_LED_CODE_OK;
}

static u8 fault_code_flash_count(u8 code)
{
    switch(code)
    {
    case FAULT_LED_CODE_NO_PIN:     return 3u;
    case FAULT_LED_CODE_PIN_STA:      return 4u;
    case FAULT_LED_CODE_NO_HOST:      return 5u;
    default:                          return 0u;
    }
}

static void fault_led_pattern_reset(u8 code)
{
    s_pat_target = fault_code_flash_count(code);
    s_pat_phase = 0;
    s_pat_idx = 0;
    s_pat_ms = 0;
    aw1_user.led_statue = 0;
}

static void fault_led_pattern_tick(void)
{
    s_pat_ms++;
    switch(s_pat_phase)
    {
    case 0:
        aw1_user.led_statue = 0;
        if(s_pat_ms >= FAULT_GAP_MS)
        {
            s_pat_ms = 0;
            s_pat_idx = 0;
            s_pat_phase = 1;
        }
        break;
    case 1:
        aw1_user.led_statue = 1;
        if(s_pat_ms >= FAULT_FLASH_ON_MS)
        {
            s_pat_ms = 0;
            s_pat_phase = 2;
        }
        break;
    case 2:
        aw1_user.led_statue = 0;
        if(s_pat_ms >= FAULT_FLASH_OFF_MS)
        {
            s_pat_ms = 0;
            s_pat_idx++;
            if(s_pat_idx >= s_pat_target)
            {
                s_pat_phase = 0;
            }
            else
            {
                s_pat_phase = 1;
            }
        }
        break;
    default:
        s_pat_phase = 0;
        break;
    }
}

void fault_led_init(void)
{
    s_ms = 0;
    s_host_seen = 0;
    s_can_last_ms = 0;
    s_pat_ms = 0;
    s_pat_phase = 0;
    s_pat_idx = 0;
    s_pat_target = 0;
    aw1_user.led_statue = 0;
}

void fault_led_on_can_rx(u8 src_id, u8 dst_id)
{
    s_can_last_ms = s_ms;
    if(src_id == 1u
       && (dst_id == Local_Station || dst_id == 0x7Fu))
    {
        s_host_seen = 1;
    }
}

u8 fault_led_get_code(void)
{
    return fault_led_resolve_code();
}

void fault_led_poll_1ms(void)
{
    static u8 s_last_code = 0xFFu;
    static u16 s_slow_ms;
    u8 code;

    s_ms++;
    code = fault_led_resolve_code();
    if(code != s_last_code)
    {
        s_last_code = code;
        s_slow_ms = 0;
        fault_led_pattern_reset(code);
    }

    switch(code)
    {
    case FAULT_LED_CODE_OK:
        s_slow_ms++;
        if(s_slow_ms >= FAULT_SLOW_MS)
        {
            s_slow_ms = 0;
            aw1_user.led_statue = (u8)!aw1_user.led_statue;
        }
        break;
    case FAULT_LED_CODE_STA_BAD:
        aw1_user.led_statue = 1;
        break;
    case FAULT_LED_CODE_CAN_SILENT:
        s_slow_ms++;
        if(s_slow_ms >= FAULT_FAST_MS)
        {
            s_slow_ms = 0;
            aw1_user.led_statue = (u8)!aw1_user.led_statue;
        }
        break;
    default:
        fault_led_pattern_tick();
        break;
    }
}
