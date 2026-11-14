#include "main.h"
#include "io_ctrl.h"
#include "io.h"
#include "pin_cfg.h"
#include "port_cos.h"

/*
 * 从机哑 IO：读物理 IN0~7，按主机下发的 io_ctrl_out 驱动物理 OUT0~7。
 * io_ctrl_out 为 16bit（每脚 2bit：关/开/快闪/慢闪），与逻辑通道无关；通道映射只在主机。
 */
#define SYSTEM_CYCLE_MS                (1)
#define SYSTEM_CYCLE                   (SYSTEM_CYCLE_MS)
#define SYSTEM_TIME(x)                 (x/SYSTEM_CYCLE)
#define FLASH_PERIOD_FAST              (250)
#define FLASH_PERIOD_SLOW              (500)

u8 io_ctrl_in = 0;
u16 io_ctrl_out = 0;
static u8 output_state_fast = 0;
static u8 output_state_slow = 0;
static input_info_st in_port[PORT_NUM];

static void io_ctrl_input_filter(input_info_st *state, u8 cur, u16 filterTime);

u8 io_ctrl_get(u8 index)
{
    u8 inputSta;

    inputSta = (u8)((IN_7_STATE <<7) | (IN_6_STATE<< 6) | (IN_5_STATE<<5) | (IN_4_STATE<<4)
                  | (IN_3_STATE<<3) | (IN_2_STATE<<2) | (IN_1_STATE<<1) | IN_0_STATE);
    aw1_user.led_in = inputSta & 0xFF;
    if(index > 7) return 0;
    return (inputSta >> index) & 0x01;
}

void io_ctrl_set(u8 index, BitAction BitVal)
{
    if(index > 7) return;

    if(BitVal == Bit_RESET)
        aw1_user.led_out &= ~(0x01 << index);
    else
        aw1_user.led_out |= (0x01 << index);

    switch(index)
    {
    case 0: OUT_0_(BitVal); break;
    case 1: OUT_1_(BitVal); break;
    case 2: OUT_2_(BitVal); break;
    case 3: OUT_3_(BitVal); break;
    case 4: OUT_4_(BitVal); break;
    case 5: OUT_5_(BitVal); break;
    case 6: OUT_6_(BitVal); break;
    case 7: OUT_7_(BitVal); break;
    default: break;
    }
}

static void io_ctrl_input_filter(input_info_st *state, u8 cur, u16 filterTime)
{
    input_info_st *p = state;
    u8 input = cur;

    if(p->input_state != input && p->input_confirm_times == 0)
    {
        p->input_middle_state = input;
    }
    if(p->input_middle_state == input && p->input_middle_state != p->input_state)
    {
        p->input_confirm_times++;
        if(p->input_confirm_times >= filterTime)
        {
            p->input_state = p->input_middle_state;
            p->input_confirm_times = 0;
            if(p->input_state == 1)
            {
                p->level = INPUT_TRIG_UP;
                p->count_flag = 1;
            }
            else
            {
                p->level = INPUT_TRIG_DOWN;
                p->count_flag = 0;
            }
        }
    }
    else
    {
        p->input_middle_state = p->input_state;
        p->input_confirm_times = 0;
    }
}

void io_ctrl_process(void)
{
    u8 i;
    static u8 reg_fast = 0;
    static u8 reg_slow = 0;
    u16 out;
    u16 mode;

    reg_fast++;
    reg_slow++;
    if(reg_fast == SYSTEM_TIME(FLASH_PERIOD_FAST))
    {
        reg_fast = 0;
        output_state_fast = !output_state_fast;
    }
    if(reg_slow == SYSTEM_TIME(FLASH_PERIOD_SLOW))
    {
        reg_slow = 0;
        output_state_slow = !output_state_slow;
    }

    out = io_ctrl_out;
    for(i = 0; i < 8; i++)
    {
        mode = (out >> (i * 2)) & 0x03;
        if(mode == 0x00)
            io_ctrl_set(i, Bit_RESET);
        else if(mode == 0x01)
            io_ctrl_set(i, Bit_SET);
        else if(mode == 0x02)
            io_ctrl_set(i, output_state_fast ? Bit_SET : Bit_RESET);
        else if(mode == 0x03)
            io_ctrl_set(i, output_state_slow ? Bit_SET : Bit_RESET);
    }

    for(i = 0; i < 8; i++)
    {
        {
            u16 ft = pin_cfg_filter_for_pin(i);
            io_ctrl_input_filter(&in_port[i], io_ctrl_get(i), ft);
        }
        io_ctrl_in = (io_ctrl_in & ~(0x01 << i)) | ((in_port[i].input_state & 0x01) << i);
    }

    /* 滤波结果写入位图；COS 握手在 port_cos_poll 中发送 TRIG/CONF */
    WritePortStateBit();
    port_cos_poll();
}
