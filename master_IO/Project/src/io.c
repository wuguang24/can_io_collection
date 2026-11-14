#include "io.h"
#include "cfg_table.h"
#include "cfg_logic.h"
#include "can_bus.h"
#include "fun.h"
#include "string.h"

extern void unload_check_alarm_process(u8 port_index);
extern void print_alarm_process(u8 port_index);
extern sINT_16_QUEUE closemsgqueue;
extern sINT_16_QUEUE openmsgqueue;
void fun_add_to_int_16_Queue(sINT_16_QUEUE *q, u16 x);

/*
 * io_handle 由 CanIo 任务每 1ms 调用；传感器电平由从机 COS CONFIRM 事件更新。
 *
 * 业务参数来源（勿在本文件硬编码）：
 *   滤波时间  → g_cfg_time（WCS 0x1425 / Flash 模板）
 *   灯闪周期  → g_cfg_logic.hdr（0x1428 / Flash 模板）
 *   状态上报  → main.c UPLOAD_PERIOD_MS（定时器周期）
 */
#define SYSTEM_CYCLE_MS                (1)
#define SYSTEM_CYCLE                   (SYSTEM_CYCLE_MS)
#define SYSTEM_TIME(x)                 ((x) / SYSTEM_CYCLE)

/* g_cfg_time.solt_filter_time==0 时的打印键滤波兜底（ms） */
#define FILTER_TIME_PRINT_DEFAULT      (50)


u8 io_port_in[MAX_SLAVE_NUM];
u16 io_port_out[MAX_SLAVE_NUM];


unload_port_st unload_port[UNLOAD_PORT_NUM];
static void io_user_logic_narrow(u8 i);
static u8 s_prev_disable[UNLOAD_PORT_NUM];
static u8 s_prev_inited = 0;

/* PRINT_STATE 帧：标记本周期打印位有事件，io_sample 按格口 bit 采样 */
void io_note_print_event(u8 station)
{
    if(station < 2 || (station - 2) >= MAX_SLAVE_NUM)
    {
        return;
    }
    portState[station - 2].port_print_state |= 1u;
}

static u8 io_local_bit_by_idx(u16 port_idx)
{
    u8 b;
    if(port_idx >= g_cfg_port_num)
    {
        return 0;
    }
    b = unload_port_assign[port_idx].local_bit;
    return (b < 8) ? b : 0;
}

/* 站级夹件扩组：仅由 1426 jam_fanout 驱动；无 jam 则不扩组 */
static u8 s_sta_clamp_expand[MAX_SLAVE_NUM];

static void io_refresh_clamp_expand_sta(void)
{
    u16 i, k, j;
    u8 head_sta, mem_sta, lbit, hblk;

    memset(s_sta_clamp_expand, 0, sizeof(s_sta_clamp_expand));

    for(i = 0; i < g_cfg_port_num; i++)
    {
        if(unload_port_assign[i].in_clamp_pack >= INPUT_MAX)
        {
            continue;
        }
        if(unload_port_assign[i].jam_extra_num == 0)
        {
            continue;
        }
        if(unload_port_assign[i].station < 2)
        {
            continue;
        }
        head_sta = (u8)(unload_port_assign[i].station - 2);
        if(head_sta >= MAX_SLAVE_NUM)
        {
            continue;
        }
        lbit = io_local_bit_by_idx(i);
        hblk = (u8)((portState[head_sta].port_block_state >> lbit) & 1u);
        if(!hblk)
        {
            continue;
        }
        s_sta_clamp_expand[head_sta] = 1;
        for(k = 0; k < unload_port_assign[i].jam_extra_num; k++)
        {
            j = cfg_find_port_index(unload_port_assign[i].jam_extra_no[k]);
            if(j >= g_cfg_port_num || unload_port_assign[j].station < 2)
            {
                continue;
            }
            mem_sta = (u8)(unload_port_assign[j].station - 2);
            if(mem_sta < MAX_SLAVE_NUM)
            {
                s_sta_clamp_expand[mem_sta] = 1;
            }
        }
    }
}

static u8 io_block_bit(u8 sta_idx, u8 bit)
{
    u8 blk = 0;
    if(sta_idx >= MAX_SLAVE_NUM)
    {
        return 0;
    }
    blk = (u8)((portState[sta_idx].port_block_state >> bit) & 1u);
    if(s_sta_clamp_expand[sta_idx])
    {
        blk = 1;
    }
    return blk;
}

/* 主脚 + OR 附加脚：任一有效即 1（多点位→同一功能，对齐 1428 信号层） */
static u8 io_read_pin_or(u8 station, u8 primary,
                         const u8 *or_pins, u8 or_n)
{
    u8 v = 0;
    u8 k;
    if(primary < INPUT_MAX)
    {
        v = read_input_state(station, primary);
    }
    if(or_pins != NULL)
    {
        for(k = 0; k < or_n && k < CFG_IN_OR_MAX; k++)
        {
            if(or_pins[k] < INPUT_MAX)
            {
                v |= read_input_state(station, or_pins[k]);
            }
        }
    }
    return v ? 1u : 0u;
}

/*
 * 读夹件：本口光电 | 若本口出现在某触发口 jam_fanout 中则并入该触发口光电。
 * 成员口未接线时仍可随触发口 jam 扩组；灯效另见 g_io_clamp_fanout。
 */
static u8 io_read_clamp_pin(u16 idx)
{
    u8 v = 0;
    u16 j, k;
    unload_port_assign_st *a = &unload_port_assign[idx];

    if(a->in_clamp_pack < INPUT_MAX)
    {
        v = read_input_state(a->station, a->in_clamp_pack);
    }
    for(j = 0; j < g_cfg_port_num; j++)
    {
        if(j == idx)
        {
            continue;
        }
        if(unload_port_assign[j].in_clamp_pack >= INPUT_MAX
           || unload_port_assign[j].jam_extra_num == 0)
        {
            continue;
        }
        for(k = 0; k < unload_port_assign[j].jam_extra_num; k++)
        {
            if(unload_port_assign[j].jam_extra_no[k] == a->port_no)
            {
                v |= read_input_state(unload_port_assign[j].station,
                                      unload_port_assign[j].in_clamp_pack);
                break;
            }
        }
    }
    return v ? 1u : 0u;
}

/*
 * jam_fanout 扩灯/信号：触发口夹件有效时，给连带格口打 fanout 标记，
 * 供 1428 夹件规则点灯；不改 clamp_pack_photo，避免 0x1402 重复上报。
 */
u8 g_io_clamp_fanout[UNLOAD_PORT_NUM];

static void io_expand_clamp_by_jam(void)
{
    u16 i, k, j;

    memset(g_io_clamp_fanout, 0, sizeof(g_io_clamp_fanout));
    for(i = 0; i < g_cfg_port_num; i++)
    {
        if(unload_port[i].clamp_pack_photo.level != IO_LVL_ON)
        {
            continue;
        }
        for(k = 0; k < unload_port_assign[i].jam_extra_num; k++)
        {
            j = cfg_find_port_index(unload_port_assign[i].jam_extra_no[k]);
            if(j < g_cfg_port_num)
            {
                g_io_clamp_fanout[j] = 1;
            }
        }
    }
}

/*
 * 本工程 CAN 上报的是格口状态位图；按 0x1426 点位表采样到 io_port_in，
 * 供后续滤波/组合逻辑使用。
 */
static void io_sample_inputs(void)
{
    u16 i;
    u8 sta_idx, bit, pin;
    u8 full, half, block, locked, printed;

    memset(io_port_in, 0, MAX_SLAVE_NUM);

    io_refresh_clamp_expand_sta();

    for(i = 0; i < g_cfg_port_num; i++)
    {
        if(unload_port_assign[i].station < 2)
        {
            continue;
        }
        sta_idx = (u8)(unload_port_assign[i].station - 2);
        if(sta_idx >= MAX_SLAVE_NUM)
        {
            continue;
        }
        bit = io_local_bit_by_idx(i);
        locked = (u8)((portState[sta_idx].port_enable_state >> bit) & 1u);
        full = (u8)((portState[sta_idx].port_pack_full_state >> bit) & 1u);
        block = io_block_bit(sta_idx, bit);
        printed = (u8)((portState[sta_idx].port_print_state >> bit) & 1u);
        /* 半满：从机 Data[0] 独立位图；满包多档由 1402 扩展组单独上报 */
        half = (u8)((portState[sta_idx].port_half_full_state >> bit) & 1u);
        (void)full_tier_active;

        pin = unload_port_assign[i].in_lock;
        if(pin < INPUT_MAX && locked)
        {
            io_port_in[sta_idx] |= (u8)(1u << pin);
        }
        {
            u8 k;
            for(k = 0; k < g_cfg_in_lock_or_n[i]; k++)
            {
                pin = g_cfg_in_lock_or[i][k];
                if(pin < INPUT_MAX && locked)
                {
                    io_port_in[sta_idx] |= (u8)(1u << pin);
                }
            }
        }
        pin = unload_port_assign[i].in_full_pack;
        if(pin < INPUT_MAX && full)
        {
            io_port_in[sta_idx] |= (u8)(1u << pin);
        }
        {
            u8 k;
            for(k = 0; k < g_cfg_in_full_or_n[i]; k++)
            {
                pin = g_cfg_in_full_or[i][k];
                if(pin < INPUT_MAX && full)
                {
                    io_port_in[sta_idx] |= (u8)(1u << pin);
                }
            }
            /* 无 OR 列表时回退到 full2 镜像 */
            pin = g_cfg_in_full2[i];
            if(pin < INPUT_MAX && full && g_cfg_in_full_or_n[i] == 0)
            {
                io_port_in[sta_idx] |= (u8)(1u << pin);
            }
        }
        pin = unload_port_assign[i].in_half_full;
        if(pin < INPUT_MAX && half)
        {
            io_port_in[sta_idx] |= (u8)(1u << pin);
        }
        pin = unload_port_assign[i].in_clamp_pack;
        if(pin < INPUT_MAX && block)
        {
            io_port_in[sta_idx] |= (u8)(1u << pin);
        }
        pin = unload_port_assign[i].in_print;
        if(pin < INPUT_MAX && printed)
        {
            io_port_in[sta_idx] |= (u8)(1u << pin);
        }
        {
            u8 k;
            for(k = 0; k < g_cfg_in_print_or_n[i]; k++)
            {
                pin = g_cfg_in_print_or[i][k];
                if(pin < INPUT_MAX && printed)
                {
                    io_port_in[sta_idx] |= (u8)(1u << pin);
                }
            }
        }
        /* 从机 PIN：unload_check 编入门位图；同步采样到落格脚 */
        pin = unload_port_assign[i].in_unload_check;
        if(pin < INPUT_MAX
           && ((portState[sta_idx].port_door_state >> bit) & 1u))
        {
            io_port_in[sta_idx] |= (u8)(1u << pin);
        }
    }
}

/* 主机逻辑算出的关格 → 队列下发；队列元素 = (sta_idx<<8)|local_bit */
static void io_commit_outputs(void)
{
    u16 i;
    u8 sta_idx, bit, locked;

    if(!s_prev_inited)
    {
        memset(s_prev_disable, 0xFF, sizeof(s_prev_disable));
        s_prev_inited = 1;
    }

    for(i = 0; i < g_cfg_port_num; i++)
    {
        u8 d = unload_port[i].port_disable_stt;
        if(s_prev_disable[i] == d)
        {
            continue;
        }
        s_prev_disable[i] = d;

        if(unload_port_assign[i].station < 2)
        {
            continue;
        }
        sta_idx = (u8)(unload_port_assign[i].station - 2);
        if(sta_idx >= MAX_SLAVE_NUM)
        {
            continue;
        }
        bit = io_local_bit_by_idx(i);
        locked = (u8)((portState[sta_idx].port_enable_state >> bit) & 1u);
        if(d == locked)
        {
            continue;
        }

        if(d)
        {
            fun_add_to_int_16_Queue(&closemsgqueue,
                (u16)(((u16)sta_idx << 8) | bit));
        }
        else
        {
            fun_add_to_int_16_Queue(&openmsgqueue,
                (u16)(((u16)sta_idx << 8) | bit));
        }
    }
}

/**
 * @brief  io模块初始化
 * 
 * @param[in]  void
 * 
 * @return void
 */
void io_port_init(void)
{
    u16 i, idx, n_saved;
    struct
    {
        u16 port_no;
        u8  close_stt;
        u8  lock_stt;
    } saved[UNLOAD_PORT_NUM];

    n_saved = 0;
    for(i = 0; i < g_cfg_port_num && i < UNLOAD_PORT_NUM; i++)
    {
        if(unload_port[i].port_close_stt || unload_port[i].lock_button.stt == IO_STT_ON)
        {
            saved[n_saved].port_no = unload_port_assign[i].port_no;
            saved[n_saved].close_stt = unload_port[i].port_close_stt;
            saved[n_saved].lock_stt = unload_port[i].lock_button.stt;
            n_saved++;
        }
    }

    memset(unload_port, 0x00, sizeof(unload_port));

    for(i = 0; i < n_saved; i++)
    {
        idx = cfg_find_port_index(saved[i].port_no);
        if(idx >= g_cfg_port_num)
        {
            continue;
        }
        unload_port[idx].port_close_stt = saved[i].close_stt;
        if(saved[i].close_stt)
        {
            unload_port[idx].lock_button.stt = saved[i].lock_stt;
        }
    }
}

/**
 * @brief  读取从机输入端口状态
 * 
 * @param[in]  station  从机站号（2-91）
 * @param[in]  port     输入端口号（0-7）
 * 
 * @return u8  输入状态：0-低电平，1-高电平
 * 
 * @note  逻辑说明：
 *         1. 从机输入状态存储在io_port_in[station-2]中（1字节包含8个输入）
 *         2. 通过位移和掩码提取指定port的状态位
 *         3. 例如：station=2, port=0 → io_port_in[0]的bit0
 */
u8 read_input_state(u8 station, u8 port)
{
    if(station < 2 || (station - 2) >= MAX_SLAVE_NUM || port >= INPUT_MAX)
    {
        return 0;
    }
    return (io_port_in[station-2] >> port) & 0x01;
}

/**
 * @brief  设置从机输出端口状态
 * 
 * @param[in]  station  从机站号（2-91）
 * @param[in]  port     输出端口号（0-7）
 * @param[in]  mode     输出模式：0-关闭，1-打开，2-快闪，3-慢闪
 * 
 * @return void
 * 
 * @note  逻辑说明：
 *         1. 每个输出端口占2位编码（00关闭、01打开、10快闪、11慢闪）
 *         2. io_port_out[station-2]是16位，可存储8个输出端口状态
 *         3. 先清除对应端口的2位，再写入新的模式值
 *         4. 例如：port=0占用bit0-1，port=1占用bit2-3
 */
void set_output_state(u8 station, u8 port, u8 mode)
{
    u8 old;
    u8 sta_idx;
    u16 shift;

    if(station < 2 || (station - 2) >= MAX_SLAVE_NUM || port >= OUTPUT_MAX)
    {
        return;
    }
    sta_idx = (u8)(station - 2);
    shift = (u16)(port * 2);
    old = (u8)((io_port_out[sta_idx] >> shift) & 0x03u);
    /* 多格口绑同一物理脚（公共红灯等）：保留更强模式，OFF=0 < ON=1 < 快闪 < 慢闪 */
    if(mode <= old)
    {
        return;
    }
    io_port_out[sta_idx] = (u16)((io_port_out[sta_idx] & (u16)(~(0x03u << shift)))
                                  | ((u16)mode << shift));
}

/**
 * @brief  滤波函数
 * 
 * @param[in]  State
 * @param[in]  cur
 * @param[in]  filterTime
 * 
 * @return void
 */
static void io_set_input_level(input_info_st *st, u8 on)
{
    u8 n = on ? IO_LVL_ON : IO_LVL_OFF;

    if(st->level != n)
    {
        st->trig = (n == IO_LVL_ON) ? IO_TRIG_UP : IO_TRIG_DOWN;
        st->level = n;
    }
    else
    {
        st->trig = IO_TRIG_NONE;
    }
    st->change_cnt = 0;
}

static void io_apply_port_from_confirm(u16 i)
{
    u8 sta_idx, bit;
    u8 locked, printed, full, half, block, door;

    if(unload_port_assign[i].station < 2)
    {
        return;
    }
    sta_idx = (u8)(unload_port_assign[i].station - 2);
    if(sta_idx >= MAX_SLAVE_NUM)
    {
        return;
    }
    bit = io_local_bit_by_idx(i);
    locked = (u8)((portState[sta_idx].port_enable_state >> bit) & 1u);
    printed = (u8)((portState[sta_idx].port_print_state >> bit) & 1u);
    full = (u8)((portState[sta_idx].port_pack_full_state >> bit) & 1u);
    half = (u8)((portState[sta_idx].port_half_full_state >> bit) & 1u);
    block = io_block_bit(sta_idx, bit);
    door = (u8)((portState[sta_idx].port_door_state >> bit) & 1u);

    if(unload_port_assign[i].in_lock < INPUT_MAX || g_cfg_in_lock_or_n[i] > 0)
    {
        io_set_input_level(&unload_port[i].lock_button, locked);
    }
    else
    {
        unload_port[i].lock_button.level = 0;
    }
    if(unload_port_assign[i].in_print < INPUT_MAX || g_cfg_in_print_or_n[i] > 0)
    {
        io_set_input_level(&unload_port[i].print_button, printed);
    }
    else
    {
        unload_port[i].print_button.level = 0;
    }
    if(unload_port_assign[i].in_full_pack < INPUT_MAX
       || g_cfg_in_full_or_n[i] > 0
       || g_cfg_in_full2[i] < INPUT_MAX)
    {
        io_set_input_level(&unload_port[i].full_pack_photo, full);
    }
    else
    {
        unload_port[i].full_pack_photo.level = 0;
    }
    if(unload_port_assign[i].in_half_full < INPUT_MAX)
    {
        io_set_input_level(&unload_port[i].half_full_photo, half);
    }
    else
    {
        unload_port[i].half_full_photo.level = 0;
    }
    if(unload_port_assign[i].in_clamp_pack < INPUT_MAX)
    {
        io_set_input_level(&unload_port[i].clamp_pack_photo, block);
        if(unload_port[i].clamp_pack_photo.trig == IO_TRIG_UP)
        {
            unload_port[i].clamp_pack_photo.trig = IO_TRIG_NONE;
        }
    }
    else
    {
        unload_port[i].clamp_pack_photo.level = 0;
    }
    if(unload_port_assign[i].in_unload_check < INPUT_MAX)
    {
        io_set_input_level(&unload_port[i].unload_check_photo, door);
        if(unload_port[i].unload_check_photo.trig == IO_TRIG_UP)
        {
            unload_port[i].unload_check_photo.trig = IO_TRIG_NONE;
            unload_check_alarm_process((u8)i);
        }
    }
    else
    {
        unload_port[i].unload_check_photo.level = 0;
    }
}

void io_on_cos_confirm(u8 sta_idx)
{
    u16 i;
    u8 station;

    if(sta_idx >= MAX_SLAVE_NUM)
    {
        return;
    }
    station = (u8)(sta_idx + 2);
    for(i = 0; i < g_cfg_port_num; i++)
    {
        if(unload_port_assign[i].station == station)
        {
            io_apply_port_from_confirm(i);
        }
    }
}

void io_on_cos_trig(u8 sta_idx, const u8 *body7)
{
    u16 i;
    u8 station;
    u8 bit;

    if(sta_idx >= MAX_SLAVE_NUM || body7 == NULL)
    {
        return;
    }
    station = (u8)(sta_idx + 2);
    for(i = 0; i < g_cfg_port_num; i++)
    {
        if(unload_port_assign[i].station != station)
        {
            continue;
        }
        bit = io_local_bit_by_idx(i);
        /* 触发态仅预置满包灯效提示，不上报 WCS（等 CONFIRM） */
        if(unload_port_assign[i].in_full_pack < INPUT_MAX
           || g_cfg_in_full_or_n[i] > 0
           || g_cfg_in_full2[i] < INPUT_MAX)
        {
            if(body7[1] & (u8)(1u << bit))
            {
                unload_port[i].full_pack_photo.level = IO_LVL_ON;
            }
        }
    }
}

/**
 * @brief  IO处理主函数（每1ms调用一次）
 * 
 * @param[in]  void
 * 
 * @return void
 * 
 * @note  传感器态由从机 COS CONFIRM 写入（从机 1425 滤波后）；主机不再二次滤波。
 *        TRIGGER 仅预置满包灯效；0x1402 与 stale 判定见 can_bus.c。
 *        灯效由 cfg_logic_apply_port → set_output_state 写从机 OUT。
 */
void io_handle(void)
{
    u16 i;

    /* 点位热切换窗口：不改输出，避免半配置驱动从机 */
    if(g_cfg_io_freeze)
    {
        return;
    }

    io_sample_inputs();

    /* 每周期先清灯位再按规则重写，避免未绑定脚残留上一周期模式 */
    memset(io_port_out, 0, sizeof(io_port_out));

    /* 夹件 jam_fanout：连带格口同步夹件信号，供灯效/逻辑使用 */
    io_expand_clamp_by_jam();

    for(i = 0; i < g_cfg_port_num; i++)
    {
        io_user_logic_narrow(i);

        /* 逻辑 CH0~CH5 的 mode → 1426 绑定的物理 OUT0~7（未绑则 set_output_state 直接返回） */
        set_output_state(unload_port_assign[i].station, unload_port_assign[i].out_lock_light, unload_port[i].lock_button_light.mode);
        set_output_state(unload_port_assign[i].station, unload_port_assign[i].out_print_light, unload_port[i].print_button_light.mode);
        if(g_cfg_out_lock2[i] < OUTPUT_MAX)
        {
            set_output_state(unload_port_assign[i].station, g_cfg_out_lock2[i],
                             unload_port[i].lock_button_light.mode);
        }
        if(g_cfg_out_print2[i] < OUTPUT_MAX)
        {
            set_output_state(unload_port_assign[i].station, g_cfg_out_print2[i],
                             unload_port[i].print_button_light.mode);
        }
        set_output_state(unload_port_assign[i].station, unload_port_assign[i].out_alarm_red, unload_port[i].alarm_red_light.mode);
        set_output_state(unload_port_assign[i].station, unload_port_assign[i].out_alarm_yellow, unload_port[i].alarm_yellow_light.mode);
        set_output_state(unload_port_assign[i].station, unload_port_assign[i].out_alarm_green, unload_port[i].alarm_green_light.mode);
        set_output_state(unload_port_assign[i].station, unload_port_assign[i].out_alarm_buzzer, unload_port[i].alarm_buzzer_light.mode);

        /* 跨站公共塔灯：把本口 CH mode 合写到 142F 目标站脚 */
        {
            u8 r;
            for(r = 0; r < g_cfg_xout_n; r++)
            {
                u8 mode = 0;
                switch(g_cfg_xout[r].src_ch)
                {
                case CFG_CH_LOCK:   mode = unload_port[i].lock_button_light.mode; break;
                case CFG_CH_PRINT:  mode = unload_port[i].print_button_light.mode; break;
                case CFG_CH_OUT2:   mode = unload_port[i].alarm_red_light.mode; break;
                case CFG_CH_OUT3:   mode = unload_port[i].alarm_yellow_light.mode; break;
                case CFG_CH_OUT4:   mode = unload_port[i].alarm_green_light.mode; break;
                case CFG_CH_OUT5:   mode = unload_port[i].alarm_buzzer_light.mode; break;
                default: break;
                }
                if(mode != 0)
                {
                    set_output_state(g_cfg_xout[r].station, g_cfg_xout[r].pin, mode);
                }
            }
        }
    }

    io_commit_outputs();
}

/** 格口灯效：0x1428 组合规则 / 矩阵，结果写入 unload_port[].*_light.mode */
static void io_user_logic_narrow(u8 i)
{
    cfg_logic_apply_port(i);
}

/**
 * @brief  端口输出处理
 * 
 * @param[in]  void
 * 
 * @return void
 */
void io_ctrl_proc(void)
{
    io_handle();
}

/**
 * @brief  上位机端口控制
 * 
 * @param[in]  void
 * 
 * @return void
 */
void io_port_state_ctrl(u8* data)
{
    u8  close_cmd   = data[1];
    u32 close_state = data[2]|(data[3]<<8)|(data[4]<<16);
    u8  i;
    
    if(close_state)
    {
        for(i = 0; i < g_cfg_port_num; i++)
        {
            if((close_state>>i)&0x1)
            {
                if(close_cmd == 0)//close
                {
                    unload_port[i].lock_button_light.mode = IO_OUT_MODE_ON;
                    unload_port[i].port_disable_stt = 1;
                }
                else//open
                {
                    unload_port[i].lock_button_light.mode = IO_OUT_MODE_OFF;
                    unload_port[i].port_disable_stt = 0;
                }
            }
        }
    }
}

/**
 * @brief  打开所有端口
 * 
 * @param[in]  void
 * 
 * @return void
 */
void io_open_all(void)
{
    u16 i;
    
    for(i = 0; i < g_cfg_port_num; i++)
    {

        unload_port[i].lock_button_light.mode = IO_OUT_MODE_OFF;
        unload_port[i].port_disable_stt = 0;
    }
}