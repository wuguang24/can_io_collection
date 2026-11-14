#include "main.h"
#include "io.h"
#include "io_ctrl.h"
#include "can_bus.h"
#include "pin_cfg.h"

extern u32 full_tier1_state;
extern u32 full_tier2_state;

/*
 * WritePortStateBit：PIN 表驱动的物理 DI → 状态位图；
 * g_port_remote_lock 合并进 port_enable_state（主机远程关格）。
 */

u8 g_port_remote_lock = 0;

sFuncConfig bfunc;
sTimeConfig timfunc;
sInput_Info loca;

void io_remote_lock_port(u8 port, u8 locked)
{
    u8 mask;

    if(port >= PORT_NUM)
    {
        return;
    }
    mask = (u8)(1u << port);
    if(locked)
    {
        g_port_remote_lock |= mask;
    }
    else
    {
        g_port_remote_lock &= (u8)(~mask);
    }
}

void io_remote_lock_all(u8 locked)
{
    g_port_remote_lock = locked ? (u8)0xFFu : 0u;
}

void io_para_init_config(void)
{
    timfunc.lock_filter_time = 20;
    timfunc.full_filter_time = 2000;
    timfunc.block_filter_time = 4000;
    timfunc.door_filter_time = 100;
    timfunc.solt_filter_time = 1000;
    pin_cfg_on_time_updated();
}

void input_filter(sInput_Info *state, u8 cur, u16 filterTime)
{
    (void)state;
    (void)cur;
    (void)filterTime;
}

void InputPortScanProc(void) {}
void OutPutCtrlProc(void) {}

static u8 pin_level(u8 in, u8 pin)
{
    if(pin >= 8)
    {
        return 0;
    }
    return (u8)((in >> pin) & 1u);
}

static u8 pin_or_level(u8 in, u8 primary, u8 or_mask)
{
    if(pin_level(in, primary))
    {
        return 1;
    }
    return (u8)((in & or_mask) ? 1u : 0u);
}

static u8 slave_read_full_level(u8 in, const pin_cfg_rec_t *rec)
{
    u8 full;

    if(g_pin_board_flags & PIN_BOARD_F_PROX_MUX)
    {
        full = pin_level(in, pin_level(in, 2) ? 1u : 0u);
    }
    else
    {
        full = pin_or_level(in, rec->in_full_pack, rec->in_or_full_mask);
    }
    if((g_pin_board_flags & PIN_BOARD_F_SIDE_GATE)
       && ((rec->port_no & 1u) == 0u)
       && pin_level(in, 5))
    {
        full = 1;
    }
    return full;
}

void WritePortStateBit(void)
{
    u8 in = io_ctrl_in;
    u8 i;
    u32 en = 0, full = 0, half = 0, door = 0, print = 0;
    u32 tier1 = 0, tier2 = 0;
    u16 block = 0;

    if(!g_pin_ready || g_pin_n == 0)
    {
        port_enable_state = 0;
        pack_full_state   = 0;
        port_block_state  = 0;
        port_door_state   = 0;
        slot_full_state   = 0;
        port_print_state  = 0;
        full_tier1_state  = 0;
        full_tier2_state  = 0;
        return;
    }

    for(i = 0; i < g_pin_n && i < PORT_NUM; i++)
    {
        if(pin_or_level(in, g_pin_rec[i].in_lock, g_pin_rec[i].in_or_lock_mask))
        {
            en |= (1u << i);
        }
        if(slave_read_full_level(in, &g_pin_rec[i]))
        {
            full |= (1u << i);
        }
        if(pin_level(in, g_pin_rec[i].in_half_full))
        {
            half |= (1u << i);
        }
        if(g_pin_rec[i].in_full_tier1 < 8
           && pin_level(in, g_pin_rec[i].in_full_tier1))
        {
            tier1 |= (1u << i);
        }
        if(g_pin_rec[i].in_full_tier2 < 8
           && pin_level(in, g_pin_rec[i].in_full_tier2))
        {
            tier2 |= (1u << i);
        }
        if(pin_level(in, g_pin_rec[i].in_clamp_pack))
        {
            block |= (u16)(1u << i);
        }
        if(pin_level(in, g_pin_rec[i].in_unload_check))
        {
            door |= (1u << i);
        }
        if(pin_or_level(in, g_pin_rec[i].in_print, g_pin_rec[i].in_or_print_mask))
        {
            print |= (1u << i);
        }
    }
    en |= (u32)g_port_remote_lock;
    port_enable_state = en;
    pack_full_state   = full;
    port_block_state  = block;
    port_door_state   = door;
    slot_full_state   = half;
    port_print_state  = print;
    full_tier1_state  = tier1;
    full_tier2_state  = tier2;
}

/* 主机 PORT_STATE 轮询：data[1]=关/开命令，data[2..4]=格口位图 */
void port_state_ctrl(u8 *data)
{
    u8  close_cmd   = data[1];
    u32 close_state = (u32)data[2] | ((u32)data[3] << 8) | ((u32)data[4] << 16);
    u8  i;

    if(close_state == 0)
    {
        return;
    }
    for(i = 0; i < PORT_NUM; i++)
    {
        if((close_state >> i) & 0x1u)
        {
            io_remote_lock_port(i, (close_cmd == 0) ? 1u : 0u);
        }
    }
}

void io_open_all(void)
{
    io_remote_lock_all(0);
}
