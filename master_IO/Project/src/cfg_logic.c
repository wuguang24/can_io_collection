/*
 * cfg_logic.c — 输出组合逻辑调度（与或非规则 + 矩阵查表）
 *
 * 层次（勿与物理脚混淆）：
 *   - 从板硬件：IN0~7 / OUT0~7（共 8 入 8 出）
 *   - 0x1426 绑定层 OR：多物理脚合成一路功能信号（满包/锁/打印），再进入本模块
 *   - 本模块：6 路逻辑通道 CH0~CH5（规则算灭/亮/快闪/慢闪）
 *       CH0=锁格灯  CH1=打印灯  CH2~CH5=通用（宏名 OUT2..OUT5 是通道号，不是物理 OUT2~5）
 *   - 0x1426：把上述通道绑到任意物理 OUT0~7；未绑定的物理脚保持关
 *   - 0x1428/1427：and/or/not 组合「已合成」的逻辑信号 → 通道模式；无规则则全灭
 *   - 打印门控：若 1426 配了锁格输入，未锁格时打印通道强制灭；合并键可 print_after_lock 延时
 */
#include "cfg_logic.h"
#include "cfg_crc.h"
#include "io.h"
#include "string.h"

cfg_logic_t g_cfg_logic;

extern void print_alarm_process(u8 port_index);

static void fill_hdr_common(u16 ver)
{
    g_cfg_logic.hdr.profile_ver = ver;
    g_cfg_logic.hdr.button_mode = CFG_BTN_TRIG;
    g_cfg_logic.hdr.lock_print_mode = 0;
    /* 打印灯效/上报：必须已锁格才响应（上位机规则仍可配打印通道怎么亮） */
    g_cfg_logic.hdr.print_gate = (u8)(CFG_PRINT_GATE_REQUIRE_LOCK | CFG_PRINT_GATE_SUPPRESS_ALARM);
    g_cfg_logic.hdr.cond_priority = 0;
    g_cfg_logic.hdr.report_flags = 0;
    g_cfg_logic.hdr.rule_count = 0;
    g_cfg_logic.hdr.flash_fast_ms = 250;
    g_cfg_logic.hdr.flash_slow_ms = 500;
    g_cfg_logic.hdr.print_delay_ms = 0;
    g_cfg_logic.hdr.release_filter_ms = 20;
}

/* 无上位机规则：矩阵全灭、规则数0 → 现场无灯动作，便于排查 */
void cfg_logic_load_empty(void)
{
    memset(&g_cfg_logic, 0, sizeof(g_cfg_logic));
    fill_hdr_common(0);
    g_cfg_logic.ready = 1;
}

static u16 s_print_wait[UNLOAD_PORT_NUM];
static u8  s_was_locked[UNLOAD_PORT_NUM];

void cfg_logic_runtime_reset(void)
{
    memset(s_print_wait, 0, sizeof(s_print_wait));
    memset(s_was_locked, 0, sizeof(s_was_locked));
}

void cfg_logic_init(void)
{
    cfg_logic_load_empty();
    cfg_logic_runtime_reset();
}

u8 cfg_logic_set_from_payload(const u8 *payload, u16 len)
{
    u16 crc_rx, crc_calc;
    cfg_logic_hdr_t hdr;
    const u16 frame = (u16)(CFG_LOGIC_HDR_SIZE + CFG_LOGIC_MATRIX_SIZE + 2);

    if(payload == NULL || len < frame)
    {
        return 0;
    }
    crc_rx = (u16)(payload[CFG_LOGIC_HDR_SIZE + CFG_LOGIC_MATRIX_SIZE]
                   | (payload[CFG_LOGIC_HDR_SIZE + CFG_LOGIC_MATRIX_SIZE + 1] << 8));
    crc_calc = cfg_crc16(payload, (u16)(CFG_LOGIC_HDR_SIZE + CFG_LOGIC_MATRIX_SIZE));
    if(crc_calc != crc_rx)
    {
        return 0;
    }
    memcpy(&hdr, payload, sizeof(hdr));
    hdr.rule_count = 0; /* 1427 只更新矩阵，走矩阵查表模式 */
    g_cfg_logic.hdr = hdr;
    memcpy(g_cfg_logic.matrix, payload + CFG_LOGIC_HDR_SIZE, CFG_LOGIC_MATRIX_SIZE);
    g_cfg_logic.ready = 1;
    return 1;
}

u8 cfg_combo_set_from_payload(const u8 *payload, u16 len)
{
    cfg_logic_hdr_t hdr;
    u16 body;
    u16 crc_rx, crc_calc;
    u8 n;

    if(payload == NULL || len < (CFG_LOGIC_HDR_SIZE + 2))
    {
        return 0;
    }
    memcpy(&hdr, payload, sizeof(hdr));
    n = hdr.rule_count;
    if(n > CFG_COMBO_RULE_MAX)
    {
        return 0;
    }
    /* n==0：清空为无动作，便于现场关掉灯效规则 */
    body = (u16)(CFG_LOGIC_HDR_SIZE + (u16)n * (u16)CFG_COMBO_RULE_SIZE);
    if(len < (u16)(body + 2))
    {
        return 0;
    }
    crc_rx = (u16)(payload[body] | (payload[body + 1] << 8));
    crc_calc = cfg_crc16(payload, body);
    if(crc_calc != crc_rx)
    {
        return 0;
    }
    if(hdr.button_mode > CFG_BTN_LEVEL)
    {
        return 0;
    }
    if(hdr.release_filter_ms == 0)
    {
        hdr.release_filter_ms = 20;
    }
    if(hdr.flash_fast_ms == 0)
    {
        hdr.flash_fast_ms = 250;
    }
    if(hdr.flash_slow_ms == 0)
    {
        hdr.flash_slow_ms = 500;
    }

    g_cfg_logic.hdr = hdr;
    if(n > 0)
    {
        memcpy(g_cfg_logic.rules, payload + CFG_LOGIC_HDR_SIZE,
               (u16)n * (u16)CFG_COMBO_RULE_SIZE);
    }
    else
    {
        memset(g_cfg_logic.rules, 0, sizeof(g_cfg_logic.rules));
        memset(g_cfg_logic.matrix, 0, sizeof(g_cfg_logic.matrix));
    }
    g_cfg_logic.ready = 1;
    return 1;
}

u8 cfg_logic_set_by_func(u8 func, const u8 *payload, u16 len)
{
    if(func == CFG_LOGIC_FUNC_COMBO)
    {
        return cfg_combo_set_from_payload(payload, len);
    }
    if(func == CFG_LOGIC_FUNC_MATRIX)
    {
        return cfg_logic_set_from_payload(payload, len);
    }
    return 0;
}

static u16 build_sig(u8 i, u8 locked)
{
    u16 sig = 0;
    if(locked)
    {
        sig |= CFG_SIG_LOCKED;
    }
    if(unload_port[i].port_close_stt)
    {
        sig |= CFG_SIG_CLOSED;
        sig |= CFG_SIG_LOCKED; /* 关口视同已锁 */
    }
    if(unload_port[i].print_button.level == IO_LVL_ON)
    {
        sig |= CFG_SIG_PRINT;
    }
    if(unload_port[i].full_pack_photo.level == IO_LVL_ON)
    {
        sig |= CFG_SIG_FULL;
    }
    if(unload_port[i].half_full_photo.level == IO_LVL_ON)
    {
        sig |= CFG_SIG_HALF;
    }
    if(unload_port[i].clamp_pack_photo.level == IO_LVL_ON
       || g_io_clamp_fanout[i])
    {
        sig |= CFG_SIG_CLAMP;
    }
    return sig;
}

static u8 rule_match(const cfg_combo_rule_t *r, u16 sig)
{
    if((sig & r->and_mask) != r->and_mask)
    {
        return 0;
    }
    if(r->or_mask != 0 && (sig & r->or_mask) == 0)
    {
        return 0;
    }
    if((sig & r->not_mask) != 0)
    {
        return 0;
    }
    return 1;
}

/* 组合调度：每通道取最高 prio 命中规则的 mode */
static void apply_combo(u8 i, u16 sig, u8 modes[CFG_CH_NUM])
{
    u8 c, k;
    u8 best_prio[CFG_CH_NUM];
    u8 n = g_cfg_logic.hdr.rule_count;

    for(c = 0; c < CFG_CH_NUM; c++)
    {
        modes[c] = CFG_LM_OFF;
        best_prio[c] = 0;
    }

    for(k = 0; k < n; k++)
    {
        const cfg_combo_rule_t *r = &g_cfg_logic.rules[k];
        if(r->out_ch >= CFG_CH_NUM)
        {
            continue;
        }
        if(!rule_match(r, sig))
        {
            continue;
        }
        if(r->prio >= best_prio[r->out_ch])
        {
            best_prio[r->out_ch] = r->prio;
            modes[r->out_ch] = r->mode;
        }
    }
    (void)i;
}

static u8 pick_cond(u8 i)
{
    if(unload_port[i].clamp_pack_photo.level == IO_LVL_ON
       || g_io_clamp_fanout[i])
    {
        return CFG_COND_CLAMP;
    }
    if(unload_port[i].full_pack_photo.level == IO_LVL_ON)
    {
        return CFG_COND_FULL;
    }
    if(unload_port[i].half_full_photo.level == IO_LVL_ON)
    {
        return CFG_COND_HALF;
    }
    return CFG_COND_IDLE;
}

static void apply_matrix(u8 i, u8 locked, u8 modes[CFG_CH_NUM])
{
    u8 cond = pick_cond(i);
    const u8 *row = g_cfg_logic.matrix[locked][cond];
    u8 c;
    for(c = 0; c < CFG_CH_NUM; c++)
    {
        modes[c] = row[c];
    }
}

void cfg_logic_apply_port(u8 i)
{
    u8 locked;
    u8 modes[CFG_CH_NUM];
    u16 sig;
    u8 alarm;
    u8 has_lock_pin;
    u8 print_eff;
    u16 delay_ms;

    if(!g_cfg_logic.ready)
    {
        cfg_logic_load_empty();
    }

    has_lock_pin = (unload_port_assign[i].in_lock < INPUT_MAX
                    || g_cfg_in_lock_or_n[i] > 0) ? 1u : 0u;

    /* WCS 0x1405 关口：保持关闭，不被本地键电平模式冲掉 */
    if(unload_port[i].port_close_stt)
    {
        unload_port[i].lock_button.stt = IO_STT_ON;
        locked = 1;
    }
    else if(has_lock_pin)
    {
        if(g_cfg_logic.hdr.button_mode == CFG_BTN_LEVEL)
        {
            unload_port[i].lock_button.stt =
                (unload_port[i].lock_button.level == IO_LVL_ON) ? IO_STT_ON : IO_STT_OFF;
        }
        else if(unload_port[i].lock_button.trig == IO_TRIG_UP)
        {
            unload_port[i].lock_button.trig = IO_TRIG_NONE;
            unload_port[i].lock_button.stt =
                (unload_port[i].lock_button.stt == IO_STT_OFF) ? IO_STT_ON : IO_STT_OFF;
        }
        locked = (unload_port[i].lock_button.stt == IO_STT_ON) ? 1 : 0;
    }
    else
    {
        /* 无锁格脚：仍允许 1405 写入的 stt 驱动灯效，直到开格清掉 */
        if(unload_port[i].lock_button.stt == IO_STT_ON)
        {
            locked = 1;
        }
        else
        {
            locked = 0;
            unload_port[i].lock_button.trig = IO_TRIG_NONE;
        }
    }

    unload_port[i].port_disable_stt = locked ? 1 : 0;

    /*
     * print_after_lock：锁打同键/合并键时，先锁后延时再认打印。
     * 优先用 1426 板级 print_after_lock_ms，否则用 1428 hdr.print_delay_ms。
     */
    print_eff = (unload_port[i].print_button.level == IO_LVL_ON) ? 1u : 0u;
    delay_ms = g_cfg_print_delay_ms[i];
    if(delay_ms == 0)
    {
        delay_ms = g_cfg_logic.hdr.print_delay_ms;
    }
    if(delay_ms > 0
       && (g_cfg_port_flags[i] & CFG_PORT_F_LOCK_PRINT_MERGED)
       && has_lock_pin)
    {
        if(locked && !s_was_locked[i])
        {
            s_print_wait[i] = delay_ms;
        }
        if(!locked)
        {
            s_print_wait[i] = 0;
        }
        if(s_print_wait[i] > 0)
        {
            s_print_wait[i]--;
            print_eff = 0;
        }
    }
    else
    {
        s_print_wait[i] = 0;
    }
    s_was_locked[i] = locked;

    {
        u8 saved = unload_port[i].print_button.level;
        unload_port[i].print_button.level = print_eff ? IO_LVL_ON : IO_LVL_OFF;
        sig = build_sig(i, locked);
        unload_port[i].print_button.level = saved;
    }
    alarm = (sig & (CFG_SIG_CLAMP | CFG_SIG_FULL | CFG_SIG_HALF)) ? 1 : 0;

    /* 无上位机规则时全灭；矩阵由 1427 灌入后才有非零 */
    if(g_cfg_logic.hdr.rule_count > 0)
    {
        apply_combo(i, sig, modes);
    }
    else
    {
        apply_matrix(i, locked, modes);
    }

    /*
     * 有锁格点位或远程关口：未锁时打印灯强制灭。
     */
    if((has_lock_pin || unload_port[i].port_close_stt) && !locked)
    {
        modes[CFG_CH_PRINT] = CFG_LM_OFF;
    }

    /* modes[] → 运行态；随后 io 按 assign.out_* 物理脚写入 io_port_out */
    unload_port[i].lock_button_light.mode = modes[CFG_CH_LOCK];
    unload_port[i].print_button_light.mode = modes[CFG_CH_PRINT];
    unload_port[i].alarm_red_light.mode = modes[CFG_CH_OUT2];
    unload_port[i].alarm_yellow_light.mode = modes[CFG_CH_OUT3];
    unload_port[i].alarm_green_light.mode = modes[CFG_CH_OUT4];
    unload_port[i].alarm_buzzer_light.mode = modes[CFG_CH_OUT5];

    /*
     * 打印上报 0x1404：
     *   - 配置了锁格点位 → 必须已锁格，且按下打印键才上报
     *   - 未配置锁格点位 → 仅需按下打印键
     *   - 合并键 + print_after_lock：延时过后 print_eff 才为 1
     */
    if((has_lock_pin || unload_port[i].port_close_stt) && !locked)
    {
        unload_port[i].port_print_stt = 0;
        return;
    }
    if((g_cfg_logic.hdr.print_gate & CFG_PRINT_GATE_SUPPRESS_ALARM) && alarm)
    {
        return;
    }
    if(unload_port_assign[i].in_print >= INPUT_MAX
       && g_cfg_in_print_or_n[i] == 0)
    {
        unload_port[i].port_print_stt = 0;
        return;
    }
    if(print_eff)
    {
        if(unload_port[i].port_print_stt == 0)
        {
            unload_port[i].port_print_stt = 1;
            print_alarm_process(i);
        }
    }
    else
    {
        unload_port[i].port_print_stt = 0;
    }
}

u16 cfg_logic_flash_fast_ms(void)
{
    return g_cfg_logic.hdr.flash_fast_ms;
}

u16 cfg_logic_flash_slow_ms(void)
{
    return g_cfg_logic.hdr.flash_slow_ms;
}

u16 cfg_logic_release_filter_ms(void)
{
    return g_cfg_logic.hdr.release_filter_ms;
}
