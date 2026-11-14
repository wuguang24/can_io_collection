/*
 * 配置表 / 点位功能展开 / UDP 半包会话
 *
 * 0x1426：物理脚(0~7) ←→ 功能码（输入角色 / 输出承接的逻辑通道 CH0~CH5）
 * 展开结果写入 unload_port_assign[]；灯效模式由 cfg_logic(1428) 算出后再写物理 OUT。
 * 业务配置可固化到 Flash 末页（cfg_store）：模板 + 最多 8 块异类。
 * 上电优先尝试本地展开；WCS 仍可随时热更新 0x1423~1426。
 * 0x1426 支持运行中热导入；收包进暂存，CRC 通过后才热切换运行表。
 */
#include "cfg_table.h"
#include "cfg_crc.h"
#include "cfg_can_ext.h"
#include "cfg_store.h"
#include "cfg_logic.h"
#include "can_bus.h"
#include "io.h"
#include "main.h"
#include "string.h"

unload_port_assign_st unload_port_assign[UNLOAD_PORT_NUM];
u8  g_cfg_in_full_or[UNLOAD_PORT_NUM][CFG_IN_OR_MAX];
u8  g_cfg_in_full_or_n[UNLOAD_PORT_NUM];
u8  g_cfg_in_lock_or[UNLOAD_PORT_NUM][CFG_IN_OR_MAX];
u8  g_cfg_in_lock_or_n[UNLOAD_PORT_NUM];
u8  g_cfg_in_print_or[UNLOAD_PORT_NUM][CFG_IN_OR_MAX];
u8  g_cfg_in_print_or_n[UNLOAD_PORT_NUM];
u8  g_cfg_in_full2[UNLOAD_PORT_NUM]; /* full_or[i][0] 镜像，便于 1426 回写 */
u8  g_cfg_out_lock2[UNLOAD_PORT_NUM];  /* 锁灯第二物理脚（OR） */
u8  g_cfg_out_print2[UNLOAD_PORT_NUM]; /* 打印灯第二物理脚（OR） */
u8  g_cfg_port_flags[UNLOAD_PORT_NUM];
u16 g_cfg_print_delay_ms[UNLOAD_PORT_NUM];
u16 g_cfg_upload_flags = 0;
u8  g_cfg_xout_n = 0;
cfg_xout_route_t g_cfg_xout[CFG_XOUT_MAX];
u16 g_cfg_board_flags[MAX_SLAVE_NUM];
u8  g_cfg_jam_block_skip[UNLOAD_PORT_NUM];
u16 g_cfg_port_num = 0;
u16 g_cfg_ver = 0;
u16 ip_offest_start = 0;
u16 ip_offest_end = 1000;
cfg_time_t g_cfg_time = {50, 4000, 2000, 100, 1000};
cfg_init_conf_t g_cfg_init;
u8 g_cfg_flags = 0;
u8 g_cfg_host_ready = 0;

static void cfg_or_add(u8 *n, u8 *or_row, u8 pin)
{
    u8 k;
    if(n == NULL || or_row == NULL || pin > 7 || *n >= CFG_IN_OR_MAX)
    {
        return;
    }
    for(k = 0; k < *n; k++)
    {
        if(or_row[k] == pin)
        {
            return;
        }
    }
    or_row[*n] = pin;
    (*n)++;
}

static void cfg_sync_full2_mirror(u16 idx)
{
    if(idx >= UNLOAD_PORT_NUM)
    {
        return;
    }
    g_cfg_in_full2[idx] = (g_cfg_in_full_or_n[idx] > 0)
                          ? g_cfg_in_full_or[idx][0] : INPUT_INVALID;
}

static void cfg_clear_port_extra(void)
{
    memset(g_cfg_in_full_or, INPUT_INVALID, sizeof(g_cfg_in_full_or));
    memset(g_cfg_in_full_or_n, 0, sizeof(g_cfg_in_full_or_n));
    memset(g_cfg_in_lock_or, INPUT_INVALID, sizeof(g_cfg_in_lock_or));
    memset(g_cfg_in_lock_or_n, 0, sizeof(g_cfg_in_lock_or_n));
    memset(g_cfg_in_print_or, INPUT_INVALID, sizeof(g_cfg_in_print_or));
    memset(g_cfg_in_print_or_n, 0, sizeof(g_cfg_in_print_or_n));
    memset(g_cfg_in_full2, INPUT_INVALID, sizeof(g_cfg_in_full2));
    memset(g_cfg_out_lock2, OUTPUT_INVALID, sizeof(g_cfg_out_lock2));
    memset(g_cfg_out_print2, OUTPUT_INVALID, sizeof(g_cfg_out_print2));
    memset(g_cfg_port_flags, 0, sizeof(g_cfg_port_flags));
    memset(g_cfg_print_delay_ms, 0, sizeof(g_cfg_print_delay_ms));
}

/* 把 src 口槽（含 OR/第二脚等扩展）搬到 dst，供按站压缩用 */
static void cfg_move_port_slot(u16 dst, u16 src)
{
    if(dst == src)
    {
        return;
    }
    unload_port_assign[dst] = unload_port_assign[src];
    memcpy(g_cfg_in_full_or[dst], g_cfg_in_full_or[src], sizeof(g_cfg_in_full_or[0]));
    g_cfg_in_full_or_n[dst] = g_cfg_in_full_or_n[src];
    memcpy(g_cfg_in_lock_or[dst], g_cfg_in_lock_or[src], sizeof(g_cfg_in_lock_or[0]));
    g_cfg_in_lock_or_n[dst] = g_cfg_in_lock_or_n[src];
    memcpy(g_cfg_in_print_or[dst], g_cfg_in_print_or[src], sizeof(g_cfg_in_print_or[0]));
    g_cfg_in_print_or_n[dst] = g_cfg_in_print_or_n[src];
    g_cfg_in_full2[dst] = g_cfg_in_full2[src];
    g_cfg_out_lock2[dst] = g_cfg_out_lock2[src];
    g_cfg_out_print2[dst] = g_cfg_out_print2[src];
    g_cfg_port_flags[dst] = g_cfg_port_flags[src];
    g_cfg_print_delay_ms[dst] = g_cfg_print_delay_ms[src];
}

static void cfg_clear_one_port_extra(u16 idx)
{
    memset(g_cfg_in_full_or[idx], INPUT_INVALID, sizeof(g_cfg_in_full_or[0]));
    g_cfg_in_full_or_n[idx] = 0;
    memset(g_cfg_in_lock_or[idx], INPUT_INVALID, sizeof(g_cfg_in_lock_or[0]));
    g_cfg_in_lock_or_n[idx] = 0;
    memset(g_cfg_in_print_or[idx], INPUT_INVALID, sizeof(g_cfg_in_print_or[0]));
    g_cfg_in_print_or_n[idx] = 0;
    g_cfg_in_full2[idx] = INPUT_INVALID;
    g_cfg_out_lock2[idx] = OUTPUT_INVALID;
    g_cfg_out_print2[idx] = OUTPUT_INVALID;
    g_cfg_port_flags[idx] = 0;
    g_cfg_print_delay_ms[idx] = 0;
}

/* 从运行表去掉某站全部口，后面口前移；merge 前调用 */
static void cfg_drop_station_ports(u8 station)
{
    u16 r;
    u16 w = 0;
    u16 n = g_cfg_port_num;

    for(r = 0; r < n; r++)
    {
        if((u8)unload_port_assign[r].station == station)
        {
            continue;
        }
        if(w != r)
        {
            cfg_move_port_slot(w, r);
        }
        w++;
    }
    for(r = w; r < n; r++)
    {
        memset(&unload_port_assign[r], 0, sizeof(unload_port_assign[0]));
        assign_invalidate_ins_outs(&unload_port_assign[r]);
        cfg_clear_one_port_extra(r);
    }
    g_cfg_port_num = w;
}

static void rec_to_assign(const cfg_pin_rec_t *r, unload_port_assign_st *a);
static void cfg_or_mask_to_list(u8 mask, u8 primary, u8 *n, u8 *or_row);
static void cfg_pin_rec_from_bytes(cfg_pin_rec_t *dst, const u8 *src, u16 rec_size);
static void assign_invalidate_ins_outs(unload_port_assign_st *a);

void cfg_rebuild_jam_skip(void)
{
    u16 i, j, k;
    u16 pno;

    memset(g_cfg_jam_block_skip, 0, sizeof(g_cfg_jam_block_skip));
    for(i = 0; i < g_cfg_port_num; i++)
    {
        pno = unload_port_assign[i].port_no;
        for(j = 0; j < g_cfg_port_num; j++)
        {
            if(j == i)
            {
                continue;
            }
            for(k = 0; k < unload_port_assign[j].jam_extra_num; k++)
            {
                if(unload_port_assign[j].jam_extra_no[k] == pno)
                {
                    g_cfg_jam_block_skip[i] = 1;
                    break;
                }
            }
            if(g_cfg_jam_block_skip[i])
            {
                break;
            }
        }
    }
}

static u8 cfg_init_has_station(u8 station)
{
    u8 i, n;

    if(station < 2)
    {
        return 0;
    }
    n = cfg_table_topo_station_count();
    for(i = 0; i < n; i++)
    {
        if(cfg_table_topo_station_at(i) == station)
        {
            return 1;
        }
    }
    return 0;
}

static u8 cfg_recs_port_unique(const cfg_pin_rec_t *recs, u16 n)
{
    u16 i, j;

    /* 业务格口号允许为 0（现场 WCS/1402 点位常从 0 起）；仅要求全机唯一 */
    for(i = 0; i < n; i++)
    {
        for(j = i + 1; j < n; j++)
        {
            if(recs[i].port_no == recs[j].port_no)
            {
                return 0;
            }
        }
    }
    return 1;
}

/* 0x1426 原始字节流：port_no 唯一性（栈上 O(n^2)，避免 num×rec 堆分配） */
static u8 cfg_recs_raw_port_unique(const u8 *raw, u16 num, u16 rec_size)
{
    u16 i, j;

    if(raw == NULL || num == 0)
    {
        return 0;
    }
    for(i = 0; i < num; i++)
    {
        const u8 *ri = raw + (u32)i * rec_size;
        u16 pn = (u16)(ri[0] | (ri[1] << 8));

        for(j = i + 1; j < num; j++)
        {
            const u8 *rj = raw + (u32)j * rec_size;
            u16 pn2 = (u16)(rj[0] | (rj[1] << 8));

            if(pn == pn2)
            {
                return 0;
            }
        }
    }
    return 1;
}

static u8 cfg_port_no_seen(u16 port_no, const u16 *pending, u8 pending_n)
{
    u8 k;

    for(k = 0; k < pending_n; k++)
    {
        if(pending[k] == port_no)
        {
            return 0;
        }
    }
    return 1;
}

static u8 cfg_table_apply_recs(const cfg_pin_rec_t *recs, u16 num, u16 ver)
{
    u16 i;
    u8 sta_cnt[CFG_STATION_MAX];

    if(recs == NULL || num == 0 || num > UNLOAD_PORT_NUM)
    {
        return 0;
    }
    if(!cfg_recs_port_unique(recs, num))
    {
        return 0;
    }

    memset(sta_cnt, 0, sizeof(sta_cnt));
    for(i = 0; i < num; i++)
    {
        u8 sta;

        if(recs[i].station < 2 || (recs[i].station - 2) >= MAX_SLAVE_NUM)
        {
            return 0;
        }
        if(!cfg_init_has_station((u8)recs[i].station))
        {
            return 0;
        }
        rec_to_assign(&recs[i], &unload_port_assign[i]);
        sta = (u8)recs[i].station;
        unload_port_assign[i].local_bit = sta_cnt[sta];
        if(sta_cnt[sta] < 7)
        {
            sta_cnt[sta]++;
        }
    }
    for(; i < UNLOAD_PORT_NUM; i++)
    {
        memset(&unload_port_assign[i], 0, sizeof(unload_port_assign[i]));
        unload_port_assign[i].local_bit = 0xFF;
    }
    g_cfg_port_num = num;
    g_cfg_ver = ver;
    cfg_clear_port_extra();
    for(i = 0; i < num; i++)
    {
        cfg_or_mask_to_list(recs[i].in_or_full_mask, recs[i].in_full_pack,
                            &g_cfg_in_full_or_n[i], g_cfg_in_full_or[i]);
        cfg_or_mask_to_list(recs[i].in_or_lock_mask, recs[i].in_lock,
                            &g_cfg_in_lock_or_n[i], g_cfg_in_lock_or[i]);
        cfg_or_mask_to_list(recs[i].in_or_print_mask, recs[i].in_print,
                            &g_cfg_in_print_or_n[i], g_cfg_in_print_or[i]);
        cfg_sync_full2_mirror(i);
    }
    cfg_rebuild_jam_skip();
    return 1;
}

/* 逐条从 raw 解析并应用（单条 cfg_pin_rec_t 在栈上，无堆峰值） */
static u8 cfg_table_apply_recs_raw(const u8 *raw, u16 num, u16 ver, u16 rec_size)
{
    u16 i;
    u8 sta_cnt[CFG_STATION_MAX];
    cfg_pin_rec_t rec;

    if(raw == NULL || num == 0 || num > UNLOAD_PORT_NUM
       || rec_size < CFG_PIN_REC_SIZE_V24)
    {
        return 0;
    }
    if(!cfg_recs_raw_port_unique(raw, num, rec_size))
    {
        return 0;
    }

    memset(sta_cnt, 0, sizeof(sta_cnt));
    for(i = 0; i < num; i++)
    {
        u8 sta;

        cfg_pin_rec_from_bytes(&rec, raw + (u32)i * rec_size, rec_size);
        if(rec.station < 2 || (rec.station - 2) >= MAX_SLAVE_NUM)
        {
            return 0;
        }
        if(!cfg_init_has_station((u8)rec.station))
        {
            return 0;
        }
        rec_to_assign(&rec, &unload_port_assign[i]);
        sta = (u8)rec.station;
        unload_port_assign[i].local_bit = sta_cnt[sta];
        if(sta_cnt[sta] < 7)
        {
            sta_cnt[sta]++;
        }
    }
    for(; i < UNLOAD_PORT_NUM; i++)
    {
        memset(&unload_port_assign[i], 0, sizeof(unload_port_assign[i]));
        unload_port_assign[i].local_bit = 0xFF;
    }
    g_cfg_port_num = num;
    g_cfg_ver = ver;
    cfg_clear_port_extra();
    for(i = 0; i < num; i++)
    {
        cfg_pin_rec_from_bytes(&rec, raw + (u32)i * rec_size, rec_size);
        cfg_or_mask_to_list(rec.in_or_full_mask, rec.in_full_pack,
                            &g_cfg_in_full_or_n[i], g_cfg_in_full_or[i]);
        cfg_or_mask_to_list(rec.in_or_lock_mask, rec.in_lock,
                            &g_cfg_in_lock_or_n[i], g_cfg_in_lock_or[i]);
        cfg_or_mask_to_list(rec.in_or_print_mask, rec.in_print,
                            &g_cfg_in_print_or_n[i], g_cfg_in_print_or[i]);
        cfg_sync_full2_mirror(i);
    }
    cfg_rebuild_jam_skip();
    return 1;
}

static void assign_set_out_func_ex(unload_port_assign_st *a, u16 idx, u8 func, u8 pin, u8 fl)
{
    switch(func)
    {
    case CFG_OUT_FUNC_LOCK_LIGHT:
        if(a->out_lock_light >= OUTPUT_MAX)
        {
            a->out_lock_light = pin;
        }
        else if(fl & CFG_BIND_F_OR)
        {
            g_cfg_out_lock2[idx] = pin;
        }
        else
        {
            a->out_lock_light = pin;
        }
        break;
    case CFG_OUT_FUNC_PRINT_LIGHT:
        if(a->out_print_light >= OUTPUT_MAX)
        {
            a->out_print_light = pin;
        }
        else if(fl & CFG_BIND_F_OR)
        {
            g_cfg_out_print2[idx] = pin;
        }
        else
        {
            a->out_print_light = pin;
        }
        break;
    case CFG_OUT_FUNC_LOCK_PRINT_LIGHT:
        if(a->out_lock_light >= OUTPUT_MAX)
        {
            a->out_lock_light = pin;
            a->out_print_light = pin;
        }
        else if(fl & CFG_BIND_F_OR)
        {
            g_cfg_out_lock2[idx] = pin;
            g_cfg_out_print2[idx] = pin;
        }
        else
        {
            a->out_lock_light = pin;
            a->out_print_light = pin;
        }
        break;
    case CFG_OUT_FUNC_RED:
        a->out_alarm_red = pin;
        break;
    case CFG_OUT_FUNC_YELLOW:
        a->out_alarm_yellow = pin;
        break;
    case CFG_OUT_FUNC_GREEN:
        a->out_alarm_green = pin;
        break;
    case CFG_OUT_FUNC_BUZZER:
        a->out_alarm_buzzer = pin;
        break;
    default:
        break;
    }
}

static void assign_set_out_func(unload_port_assign_st *a, u8 func, u8 pin)
{
    /* 板级无 idx：不走第二脚 OR */
    switch(func)
    {
    case CFG_OUT_FUNC_LOCK_LIGHT:
        a->out_lock_light = pin;
        break;
    case CFG_OUT_FUNC_PRINT_LIGHT:
        a->out_print_light = pin;
        break;
    case CFG_OUT_FUNC_LOCK_PRINT_LIGHT:
        a->out_lock_light = pin;
        a->out_print_light = pin;
        break;
    case CFG_OUT_FUNC_RED:
        a->out_alarm_red = pin;
        break;
    case CFG_OUT_FUNC_YELLOW:
        a->out_alarm_yellow = pin;
        break;
    case CFG_OUT_FUNC_GREEN:
        a->out_alarm_green = pin;
        break;
    case CFG_OUT_FUNC_BUZZER:
        a->out_alarm_buzzer = pin;
        break;
    default:
        break;
    }
}

static void assign_apply_jam_list(unload_port_assign_st *a, const u16 *jam_ports, u8 jam_n)
{
    u8 k;
    a->jam_extra_num = 0;
    for(k = 0; k < jam_n && a->jam_extra_num < CFG_JAM_EXTRA_MAX; k++)
    {
        if(jam_ports[k] != a->port_no)
        {
            a->jam_extra_no[a->jam_extra_num] = jam_ports[k];
            a->jam_extra_num++;
        }
    }
}

static void assign_invalidate_ins_outs(unload_port_assign_st *a)
{
    a->local_bit = 0xFF;
    a->in_lock = INPUT_INVALID;
    a->in_print = INPUT_INVALID;
    a->in_full_pack = INPUT_INVALID;
    a->in_half_full = INPUT_INVALID;
    a->in_clamp_pack = INPUT_INVALID;
    a->in_unload_check = INPUT_INVALID;
    a->in_full_tier1 = INPUT_INVALID;
    a->in_full_tier2 = INPUT_INVALID;
    a->out_lock_light = OUTPUT_INVALID;
    a->out_print_light = OUTPUT_INVALID;
    a->out_alarm_red = OUTPUT_INVALID;
    a->out_alarm_yellow = OUTPUT_INVALID;
    a->out_alarm_green = OUTPUT_INVALID;
    a->out_alarm_buzzer = OUTPUT_INVALID;
    a->jam_extra_num = 0;
    memset(a->jam_extra_no, 0, sizeof(a->jam_extra_no));
}

/* OR 脚列表 → 从机位掩码（排除主脚） */
static u8 cfg_or_list_to_mask(const u8 *or_row, u8 or_n, u8 primary)
{
    u8 m = 0;
    u8 k;

    if(or_row == NULL)
    {
        return 0;
    }
    for(k = 0; k < or_n && k < CFG_IN_OR_MAX; k++)
    {
        if(or_row[k] < 8 && or_row[k] != primary)
        {
            m |= (u8)(1u << or_row[k]);
        }
    }
    return m;
}

static void cfg_or_mask_to_list(u8 mask, u8 primary, u8 *n, u8 *or_row)
{
    u8 pin;

    if(n == NULL || or_row == NULL)
    {
        return;
    }
    *n = 0;
    for(pin = 0; pin < 8 && *n < CFG_IN_OR_MAX; pin++)
    {
        if(pin == primary)
        {
            continue;
        }
        if(mask & (u8)(1u << pin))
        {
            or_row[*n] = pin;
            (*n)++;
        }
    }
}

static void cfg_pin_rec_from_bytes(cfg_pin_rec_t *dst, const u8 *src, u16 rec_size)
{
    memset(dst, 0, sizeof(*dst));
    dst->in_full_tier1 = INPUT_INVALID;
    dst->in_full_tier2 = INPUT_INVALID;
    if(src == NULL || rec_size < CFG_PIN_REC_SIZE_V24)
    {
        return;
    }
    memcpy(dst, src, CFG_PIN_REC_SIZE_V24);
    if(rec_size >= CFG_PIN_REC_SIZE_OR_JAM4)
    {
        u8 jam_n;
        dst->jam_extra_num = src[15];
        jam_n = dst->jam_extra_num;
        if(jam_n > 4)
        {
            jam_n = 4;
        }
        memcpy(dst->jam_extra_no, &src[16], (u16)jam_n * sizeof(u16));
        dst->in_or_full_mask = src[24];
        dst->in_or_lock_mask = src[25];
        dst->in_or_print_mask = src[26];
    }
    if(rec_size >= CFG_PIN_REC_SIZE_JAM12)
    {
        u8 jam_n;
        dst->jam_extra_num = src[15];
        jam_n = dst->jam_extra_num;
        if(jam_n > CFG_MAX_JAM_FANOUT)
        {
            jam_n = CFG_MAX_JAM_FANOUT;
        }
        memcpy(dst->jam_extra_no, &src[16], (u16)jam_n * sizeof(u16));
        dst->in_or_full_mask = src[40];
        dst->in_or_lock_mask = src[41];
        dst->in_or_print_mask = src[42];
        dst->in_full_tier1 = INPUT_INVALID;
        dst->in_full_tier2 = INPUT_INVALID;
        if(rec_size >= CFG_PIN_REC_SIZE)
        {
            dst->in_full_tier1 = src[43];
            dst->in_full_tier2 = src[44];
        }
    }
}

static u8 cfg_table_set_from_recs_raw(const u8 *raw, u16 num, u16 ver, u16 crc, u16 rec_size)
{
    u16 calc;

    if(raw == NULL || num == 0 || num > CFG_BIZ_PORT_MAX)
    {
        return 0;
    }
    calc = cfg_crc16(raw, (u16)(num * rec_size));
    if(calc != crc)
    {
        return 0;
    }
    return cfg_table_apply_recs_raw(raw, num, ver, rec_size);
}

static void rec_to_assign(const cfg_pin_rec_t *r, unload_port_assign_st *a)
{
    a->port_no = r->port_no;
    a->station = r->station;
    a->local_bit = 0xFF; /* set_from_recs 里按站重算 */
    a->in_lock = r->in_lock;
    a->in_print = r->in_print;
    a->in_full_pack = r->in_full_pack;
    a->in_half_full = r->in_half_full;
    a->in_clamp_pack = r->in_clamp_pack;
    a->in_unload_check = r->in_unload_check;
    a->in_full_tier1 = r->in_full_tier1;
    a->in_full_tier2 = r->in_full_tier2;
    a->out_lock_light = r->out_lock_light;
    a->out_print_light = r->out_print_light;
    a->out_alarm_red = r->out_alarm_red;
    a->out_alarm_yellow = r->out_alarm_yellow;
    a->out_alarm_green = r->out_alarm_green;
    a->out_alarm_buzzer = r->out_alarm_buzzer;
    a->jam_extra_num = r->jam_extra_num;
    if(a->jam_extra_num > CFG_JAM_EXTRA_MAX)
    {
        a->jam_extra_num = CFG_JAM_EXTRA_MAX;
    }
    memset(a->jam_extra_no, 0, sizeof(a->jam_extra_no));
    memcpy(a->jam_extra_no, r->jam_extra_no, (u16)a->jam_extra_num * sizeof(u16));
}

static void assign_to_rec(const unload_port_assign_st *a, cfg_pin_rec_t *r, u16 idx)
{
    r->port_no = a->port_no;
    r->station = (u8)a->station;
    r->in_lock = a->in_lock;
    r->in_print = a->in_print;
    r->in_full_pack = a->in_full_pack;
    r->in_half_full = a->in_half_full;
    r->in_clamp_pack = a->in_clamp_pack;
    r->in_unload_check = a->in_unload_check;
    r->in_full_tier1 = a->in_full_tier1;
    r->in_full_tier2 = a->in_full_tier2;
    r->out_lock_light = a->out_lock_light;
    r->out_print_light = a->out_print_light;
    r->out_alarm_red = a->out_alarm_red;
    r->out_alarm_yellow = a->out_alarm_yellow;
    r->out_alarm_green = a->out_alarm_green;
    r->out_alarm_buzzer = a->out_alarm_buzzer;
    r->jam_extra_num = a->jam_extra_num;
    if(r->jam_extra_num > CFG_MAX_JAM_FANOUT)
    {
        r->jam_extra_num = CFG_MAX_JAM_FANOUT;
    }
    memset(r->jam_extra_no, 0, sizeof(r->jam_extra_no));
    memcpy(r->jam_extra_no, a->jam_extra_no, (u16)r->jam_extra_num * sizeof(u16));
    r->in_or_full_mask = 0;
    r->in_or_lock_mask = 0;
    r->in_or_print_mask = 0;
    if(idx < UNLOAD_PORT_NUM)
    {
        r->in_or_full_mask = cfg_or_list_to_mask(g_cfg_in_full_or[idx],
                                                 g_cfg_in_full_or_n[idx],
                                                 a->in_full_pack);
        if(g_cfg_in_full2[idx] < 8 && g_cfg_in_full2[idx] != a->in_full_pack)
        {
            r->in_or_full_mask |= (u8)(1u << g_cfg_in_full2[idx]);
        }
        r->in_or_lock_mask = cfg_or_list_to_mask(g_cfg_in_lock_or[idx],
                                                 g_cfg_in_lock_or_n[idx],
                                                 a->in_lock);
        r->in_or_print_mask = cfg_or_list_to_mask(g_cfg_in_print_or[idx],
                                                  g_cfg_in_print_or_n[idx],
                                                  a->in_print);
    }
}

u8 cfg_table_apply_default(void)
{
    u16 i;

    /* 无编译期默认点位表：空槽等待 WCS 0x1426 */
    memset(unload_port_assign, 0, sizeof(unload_port_assign));
    cfg_clear_port_extra();
    memset(g_cfg_board_flags, 0, sizeof(g_cfg_board_flags));
    memset(g_cfg_jam_block_skip, 0, sizeof(g_cfg_jam_block_skip));
    g_cfg_upload_flags = 0;
    g_cfg_xout_n = 0;
    memset(g_cfg_xout, 0, sizeof(g_cfg_xout));
    for(i = 0; i < UNLOAD_PORT_NUM; i++)
    {
        assign_invalidate_ins_outs(&unload_port_assign[i]);
        g_cfg_in_full2[i] = INPUT_INVALID;
    }
    g_cfg_port_num = 0;
    g_cfg_ver = 0;
    g_cfg_host_ready = 0;
    g_cfg_flags = 0;
    return 1;
}

void cfg_table_init(void)
{
    (void)cfg_table_apply_default();
    memset(&g_cfg_init, 0, sizeof(g_cfg_init));
    ip_offest_start = 0;
    ip_offest_end = 1000;
    g_cfg_time.lock_filter_time = 50;
    g_cfg_time.full_filter_time = 4000;
    g_cfg_time.block_filter_time = 2000;
    g_cfg_time.door_filter_time = 100;
    g_cfg_time.solt_filter_time = 1000;
}

u16 cfg_table_calc_crc(void)
{
    /* 避免 UNLOAD_PORT_NUM×sizeof(rec) 压栈 */
    u16 crc = 0xFFFF;
    u16 i;
    cfg_pin_rec_t rec;

    for(i = 0; i < g_cfg_port_num; i++)
    {
        assign_to_rec(&unload_port_assign[i], &rec, i);
        crc = cfg_crc16_continue(crc, (const u8 *)&rec, (u16)sizeof(rec));
    }
    return crc;
}

u8 cfg_table_set_from_recs(const cfg_pin_rec_t *recs, u16 num, u16 ver, u16 crc)
{
    u16 calc;

    if(recs == NULL || num == 0 || num > UNLOAD_PORT_NUM)
    {
        return 0;
    }
    calc = cfg_crc16((const u8 *)recs, (u16)(num * sizeof(cfg_pin_rec_t)));
    if(calc != crc)
    {
        return 0;
    }
    return cfg_table_apply_recs(recs, num, ver);
}

/*
 * 0x1426 format2 blob (after CRC check of whole blob):
 *   board_count u8
 *   for each board:
 *     station u8
 *     local_port_count u8
 *     in_cnt u8, out_cnt u8
 *     flags u16, print_after_lock_ms u16
 *     jam_fanout_count u8
 *     port_map[local_port_count] u16 LE
 *     in_bind[in_cnt]: pin,func,local_port,flags
 *     out_bind[out_cnt]: pin,func,local_port,flags
 *     jam_fanout[jam_fanout_count] u16 LE
 */
/*
 * merge_mode=0：整表替换（format2），缺站按拓扑从样板展开
 * merge_mode=1：只替换 blob 内各站（format3），保留其它站，不做拓扑展开
 */
static u8 cfg_table_apply_bind_blob_ex(const u8 *blob, u16 blob_len, u16 ver, u16 crc,
                                       u8 merge_mode)
{
    const u8 *p;
    const u8 *end;
    u8 board_count;
    u8 b;
    u16 port_n = 0;
    u16 calc;

    if(blob == NULL || blob_len < 2)
    {
        return 0;
    }
    calc = cfg_crc16(blob, blob_len);
    if(calc != crc)
    {
        return 0;
    }

    if(merge_mode)
    {
        const u8 *sp = blob;
        const u8 *send = blob + blob_len;
        u8 bc;
        u8 bi;

        bc = *sp++;
        if(bc == 0 || bc > CFG_MAX_BOARD)
        {
            return 0;
        }
        for(bi = 0; bi < bc; bi++)
        {
            u8 station, lpc, in_cnt, out_cnt, jam_n;
            u16 skip;

            if((sp + 9) > send)
            {
                return 0;
            }
            station = sp[0];
            lpc = sp[1];
            in_cnt = sp[2];
            out_cnt = sp[3];
            jam_n = sp[8];
            sp += 9;
            skip = (u16)(lpc * 2u + in_cnt * 4u + out_cnt * 4u + jam_n * 2u);
            if((sp + skip) > send)
            {
                return 0;
            }
            cfg_drop_station_ports(station);
            if(station >= 2 && (station - 2) < MAX_SLAVE_NUM)
            {
                g_cfg_board_flags[station - 2] = 0; /* 稍后由板头重写 */
            }
            sp += skip;
        }
        port_n = g_cfg_port_num;
    }
    else
    {
        memset(unload_port_assign, 0, sizeof(unload_port_assign));
        cfg_clear_port_extra();
        memset(g_cfg_board_flags, 0, sizeof(g_cfg_board_flags));
        port_n = 0;
    }

    p = blob;
    end = blob + blob_len;
    board_count = *p++;
    if(board_count == 0 || board_count > CFG_MAX_BOARD)
    {
        return 0;
    }

    for(b = 0; b < board_count; b++)
    {
        u8 station, lpc, in_cnt, out_cnt, jam_n;
        u16 flags, print_ms;
        u16 port_map[CFG_MAX_LOCAL_PORT];
        cfg_pin_bind_t inb[CFG_MAX_BIND_PER_BOARD];
        cfg_pin_bind_t outb[CFG_MAX_BIND_PER_BOARD];
        u16 jam_ports[CFG_MAX_JAM_FANOUT];
        u8 i, lp;
        u16 base_idx;

        if((p + 8) > end)
        {
            return 0;
        }
        station = *p++;
        lpc = *p++;
        in_cnt = *p++;
        out_cnt = *p++;
        flags = (u16)(p[0] | (p[1] << 8)); p += 2;
        print_ms = (u16)(p[0] | (p[1] << 8)); p += 2;
        jam_n = *p++;
        if(station >= 2 && (station - 2) < MAX_SLAVE_NUM)
        {
            g_cfg_board_flags[station - 2] = flags;
        }

        if(station < 2 || (station - 2) >= MAX_SLAVE_NUM
           || lpc == 0 || lpc > CFG_MAX_LOCAL_PORT
           || in_cnt > CFG_MAX_BIND_PER_BOARD || out_cnt > CFG_MAX_BIND_PER_BOARD
           || jam_n > CFG_MAX_JAM_FANOUT
           || !cfg_init_has_station(station))
        {
            return 0;
        }
        if((p + lpc * 2 + in_cnt * 4 + out_cnt * 4 + jam_n * 2) > end)
        {
            return 0;
        }
        if((u16)(port_n + lpc) > CFG_BIZ_PORT_MAX)
        {
            return 0;
        }

        for(i = 0; i < lpc; i++)
        {
            u16 pn;
            u16 k;

            pn = (u16)(p[0] | (p[1] << 8));
            p += 2;
            if(!cfg_port_no_seen(pn, port_map, i))
            {
                return 0;
            }
            for(k = 0; k < port_n; k++)
            {
                if(unload_port_assign[k].port_no == pn)
                {
                    return 0;
                }
            }
            port_map[i] = pn;
        }
        for(i = 0; i < in_cnt; i++)
        {
            inb[i].pin = *p++;
            inb[i].func = *p++;
            inb[i].local_port = *p++;
            inb[i].flags = *p++;
        }
        for(i = 0; i < out_cnt; i++)
        {
            outb[i].pin = *p++;
            outb[i].func = *p++;
            outb[i].local_port = *p++;
            outb[i].flags = *p++;
        }
        for(i = 0; i < jam_n; i++)
        {
            jam_ports[i] = (u16)(p[0] | (p[1] << 8));
            p += 2;
        }

        base_idx = port_n;
        for(lp = 0; lp < lpc; lp++)
        {
            unload_port_assign_st *a = &unload_port_assign[base_idx + lp];
            a->port_no = port_map[lp];
            a->station = station;
            a->local_bit = lp;
            assign_invalidate_ins_outs(a);
            a->local_bit = lp; /* invalidate 会清掉，再写回 */
            g_cfg_in_full_or_n[base_idx + lp] = 0;
            g_cfg_in_lock_or_n[base_idx + lp] = 0;
            g_cfg_in_print_or_n[base_idx + lp] = 0;
            g_cfg_out_lock2[base_idx + lp] = OUTPUT_INVALID;
            g_cfg_out_print2[base_idx + lp] = OUTPUT_INVALID;
            g_cfg_in_full2[base_idx + lp] = INPUT_INVALID;
            g_cfg_port_flags[base_idx + lp] = 0;
            g_cfg_print_delay_ms[base_idx + lp] = print_ms;
        }

        for(i = 0; i < in_cnt; i++)
        {
            u8 pin = inb[i].pin;
            u8 func = inb[i].func;
            u8 loc = inb[i].local_port;
            u8 fl = inb[i].flags;

            if(pin > 7)
            {
                return 0;
            }
            if(loc == CFG_BIND_LOCAL_BOARD)
            {
                if(func == CFG_IN_FUNC_CLAMP)
                {
                    u8 j;
                    for(j = 0; j < lpc; j++)
                    {
                        unload_port_assign[base_idx + j].in_clamp_pack = pin;
                        assign_apply_jam_list(&unload_port_assign[base_idx + j],
                                             jam_ports, jam_n);
                    }
                }
                continue;
            }
            if(loc >= lpc)
            {
                return 0;
            }
            {
                unload_port_assign_st *a = &unload_port_assign[base_idx + loc];
                u16 idx = (u16)(base_idx + loc);
                switch(func)
                {
                case CFG_IN_FUNC_LOCK:
                    if(a->in_lock >= INPUT_MAX)
                    {
                        a->in_lock = pin;
                    }
                    else if(fl & CFG_BIND_F_OR)
                    {
                        cfg_or_add(&g_cfg_in_lock_or_n[idx], g_cfg_in_lock_or[idx], pin);
                    }
                    else
                    {
                        a->in_lock = pin;
                    }
                    break;
                case CFG_IN_FUNC_PRINT:
                    if(a->in_print >= INPUT_MAX)
                    {
                        a->in_print = pin;
                    }
                    else if(fl & CFG_BIND_F_OR)
                    {
                        cfg_or_add(&g_cfg_in_print_or_n[idx], g_cfg_in_print_or[idx], pin);
                    }
                    else
                    {
                        a->in_print = pin;
                    }
                    break;
                case CFG_IN_FUNC_LOCK_PRINT:
                    /* 多键同效：首键作主脚，其后带 OR 标志并入锁/打印 */
                    if(a->in_lock >= INPUT_MAX)
                    {
                        a->in_lock = pin;
                        a->in_print = pin;
                        g_cfg_port_flags[idx] |= CFG_PORT_F_LOCK_PRINT_MERGED;
                    }
                    else if(fl & CFG_BIND_F_OR)
                    {
                        cfg_or_add(&g_cfg_in_lock_or_n[idx], g_cfg_in_lock_or[idx], pin);
                        cfg_or_add(&g_cfg_in_print_or_n[idx], g_cfg_in_print_or[idx], pin);
                        g_cfg_port_flags[idx] |= CFG_PORT_F_LOCK_PRINT_MERGED;
                    }
                    else
                    {
                        a->in_lock = pin;
                        a->in_print = pin;
                        g_cfg_port_flags[idx] |= CFG_PORT_F_LOCK_PRINT_MERGED;
                    }
                    break;
                case CFG_IN_FUNC_FULL:
                    if(a->in_full_pack >= INPUT_MAX)
                    {
                        a->in_full_pack = pin;
                    }
                    else if(fl & CFG_BIND_F_OR)
                    {
                        cfg_or_add(&g_cfg_in_full_or_n[idx], g_cfg_in_full_or[idx], pin);
                        cfg_sync_full2_mirror(idx);
                    }
                    else
                    {
                        a->in_full_pack = pin;
                    }
                    break;
                case CFG_IN_FUNC_HALF:
                    a->in_half_full = pin;
                    break;
                case CFG_IN_FUNC_CLAMP:
                    a->in_clamp_pack = pin;
                    break;
                case CFG_IN_FUNC_UNLOAD_CHECK:
                    a->in_unload_check = pin;
                    break;
                case CFG_IN_FUNC_FULL_TIER1:
                    a->in_full_tier1 = pin;
                    break;
                case CFG_IN_FUNC_FULL_TIER2:
                    a->in_full_tier2 = pin;
                    break;
                default:
                    break;
                }
            }
        }

        for(i = 0; i < out_cnt; i++)
        {
            u8 pin = outb[i].pin;
            u8 func = outb[i].func;
            u8 loc = outb[i].local_port;
            u8 fl = outb[i].flags;

            if(pin > 7)
            {
                return 0;
            }
            /* 板级公共 OUT：本站全部格口承接同一物理脚（配合 set_output_state 合写） */
            if(loc == CFG_BIND_LOCAL_BOARD)
            {
                u8 j;
                for(j = 0; j < lpc; j++)
                {
                    assign_set_out_func(&unload_port_assign[base_idx + j], func, pin);
                }
                continue;
            }
            if(loc >= lpc)
            {
                return 0;
            }
            assign_set_out_func_ex(&unload_port_assign[base_idx + loc],
                                   (u16)(base_idx + loc), func, pin, fl);
        }

        /* 若有 jam 列表但夹件是按口绑的：仍灌到本站所有已绑夹件口 */
        if(jam_n > 0)
        {
            u8 j;
            for(j = 0; j < lpc; j++)
            {
                if(unload_port_assign[base_idx + j].in_clamp_pack < INPUT_MAX
                   && unload_port_assign[base_idx + j].jam_extra_num == 0)
                {
                    assign_apply_jam_list(&unload_port_assign[base_idx + j],
                                         jam_ports, jam_n);
                }
            }
        }

        port_n = (u16)(port_n + lpc);
    }

    if(p != end)
    {
        /* allow trailing padding 0xFF */
        while(p < end)
        {
            if(*p != 0xFF && *p != 0x00)
            {
                return 0;
            }
            p++;
        }
    }

    if(port_n == 0)
    {
        return 0;
    }

    /* 整表替换时：按 1423 拓扑把缺站从样板口复制并改格号；merge 不做展开 */
    if(!merge_mode)
    {
        u16 ii;
        u8 ti, topo_n, tpl_sta = 0;
        u8 tpl_idx[CFG_MAX_LOCAL_PORT];
        u8 tpl_lpc = 0;
        u16 ip0 = ip_offest_start;

        for(ii = 0; ii < port_n; ii++)
        {
            if(tpl_sta == 0)
            {
                tpl_sta = (u8)unload_port_assign[ii].station;
            }
        }
        for(ii = 0; ii < port_n && tpl_lpc < CFG_MAX_LOCAL_PORT; ii++)
        {
            if((u8)unload_port_assign[ii].station == tpl_sta)
            {
                tpl_idx[tpl_lpc++] = (u8)ii;
            }
        }
        topo_n = cfg_table_topo_station_count();
        for(ti = 0; ti < topo_n && tpl_lpc > 0; ti++)
        {
            u8 sta = cfg_table_topo_station_at(ti);
            u8 covered = 0;
            u8 lp;

            for(ii = 0; ii < port_n; ii++)
            {
                if((u8)unload_port_assign[ii].station == sta)
                {
                    covered = 1;
                    break;
                }
            }
            if(covered || sta < 2)
            {
                continue;
            }
            if((u16)(port_n + tpl_lpc) > CFG_BIZ_PORT_MAX)
            {
                return 0;
            }
            /* 板级 flags 随样板站复制 */
            if((sta - 2) < MAX_SLAVE_NUM && (tpl_sta - 2) < MAX_SLAVE_NUM)
            {
                g_cfg_board_flags[sta - 2] = g_cfg_board_flags[tpl_sta - 2];
            }
            for(lp = 0; lp < tpl_lpc; lp++)
            {
                u16 src_i = tpl_idx[lp];
                u16 dst_i = port_n;
                unload_port_assign_st *src = &unload_port_assign[src_i];
                unload_port_assign_st *dst = &unload_port_assign[dst_i];
                u8 k;

                *dst = *src;
                dst->station = sta;
                dst->local_bit = lp;
                dst->port_no = (u16)(ip0 + (u16)(sta - 2u) * (u16)tpl_lpc + lp);
                /* jam 连带格号：按样板相对下标重映射到本站 */
                for(k = 0; k < dst->jam_extra_num && k < CFG_JAM_EXTRA_MAX; k++)
                {
                    u16 old_pn = src->jam_extra_no[k];
                    u8 found = 0;
                    u8 j;
                    for(j = 0; j < tpl_lpc; j++)
                    {
                        if(unload_port_assign[tpl_idx[j]].port_no == old_pn)
                        {
                            dst->jam_extra_no[k] = (u16)(
                                ip0 + (u16)(sta - 2u) * (u16)tpl_lpc + j);
                            found = 1;
                            break;
                        }
                    }
                    if(!found && tpl_lpc > 0)
                    {
                        u16 base = unload_port_assign[tpl_idx[0]].port_no;
                        dst->jam_extra_no[k] = (u16)(
                            ip0 + (u16)(sta - 2u) * (u16)tpl_lpc
                            + (u16)(old_pn - base));
                    }
                }
                memcpy(g_cfg_in_full_or[dst_i], g_cfg_in_full_or[src_i],
                       sizeof(g_cfg_in_full_or[0]));
                g_cfg_in_full_or_n[dst_i] = g_cfg_in_full_or_n[src_i];
                memcpy(g_cfg_in_lock_or[dst_i], g_cfg_in_lock_or[src_i],
                       sizeof(g_cfg_in_lock_or[0]));
                g_cfg_in_lock_or_n[dst_i] = g_cfg_in_lock_or_n[src_i];
                memcpy(g_cfg_in_print_or[dst_i], g_cfg_in_print_or[src_i],
                       sizeof(g_cfg_in_print_or[0]));
                g_cfg_in_print_or_n[dst_i] = g_cfg_in_print_or_n[src_i];
                g_cfg_in_full2[dst_i] = g_cfg_in_full2[src_i];
                g_cfg_out_lock2[dst_i] = g_cfg_out_lock2[src_i];
                g_cfg_out_print2[dst_i] = g_cfg_out_print2[src_i];
                g_cfg_port_flags[dst_i] = g_cfg_port_flags[src_i];
                g_cfg_print_delay_ms[dst_i] = g_cfg_print_delay_ms[src_i];
                port_n++;
            }
        }
    }

    g_cfg_port_num = port_n;
    g_cfg_ver = ver;
    cfg_rebuild_jam_skip();
    return 1;
}

u8 cfg_table_set_from_bind_blob(const u8 *blob, u16 blob_len, u16 ver, u16 crc)
{
    return cfg_table_apply_bind_blob_ex(blob, blob_len, ver, crc, 0);
}

u8 cfg_table_merge_from_bind_blob(const u8 *blob, u16 blob_len, u16 ver, u16 crc)
{
    return cfg_table_apply_bind_blob_ex(blob, blob_len, ver, crc, 1);
}

void cfg_table_build_station_slice(u8 station, u8 *out, u16 *out_len, u16 *out_crc)
{
    cfg_pin_rec_t recs[8];
    u8 n = 0;
    u16 i;
    u16 crc;
    u16 flags = 0;
    u8 sta_idx;

    for(i = 0; i < g_cfg_port_num && n < 8; i++)
    {
        if((u8)unload_port_assign[i].station == station)
        {
            assign_to_rec(&unload_port_assign[i], &recs[n], i);
            n++;
        }
    }
    sta_idx = (station >= 2) ? (u8)(station - 2) : 0;
    if(sta_idx < MAX_SLAVE_NUM)
    {
        flags = g_cfg_board_flags[sta_idx];
    }
    out[0] = (u8)(g_cfg_ver & 0xFF);
    out[1] = (u8)(g_cfg_ver >> 8);
    out[2] = n;
    out[3] = (u8)(flags & 0xFF);
    out[4] = (u8)(flags >> 8);
    if(n > 0)
    {
        memcpy(&out[CFG_PIN_SLICE_HDR_EXT], recs, n * sizeof(cfg_pin_rec_t));
    }
    *out_len = (u16)(CFG_PIN_SLICE_HDR_EXT + n * sizeof(cfg_pin_rec_t));
    crc = cfg_crc16(out, *out_len);
    out[*out_len] = (u8)(crc & 0xFF);
    out[*out_len + 1] = (u8)(crc >> 8);
    *out_len += 2;
    *out_crc = crc;
}

u16 cfg_find_port_index(u16 port_no)
{
    u16 i;
    for(i = 0; i < g_cfg_port_num; i++)
    {
        if(unload_port_assign[i].port_no == port_no)
        {
            return i;
        }
    }
    return 0xFFFF;
}

u8 cfg_table_parse_init_func(const u8 *payload, u16 payload_len)
{
    u8 i, s;
    const u8 *p;
    const u8 *end;

    if(payload == NULL || payload_len < 3)
    {
        return 0;
    }

    p = payload;
    end = payload + payload_len;
    memset(&g_cfg_init, 0, sizeof(g_cfg_init));
    g_cfg_init.slave_num = *p++;
    g_cfg_init.group_total = *p++;
    g_cfg_init.expand_master = *p++;

    if(g_cfg_init.group_total > CFG_MAX_GROUP)
    {
        return 0;
    }

    for(i = 0; i < g_cfg_init.group_total; i++)
    {
        u8 cnt;
        if(p >= end)
        {
            return 0;
        }
        cnt = *p++;
        if(cnt == 0 || cnt > CFG_MAX_STATION_PER_GROUP)
        {
            return 0;
        }
        g_cfg_init.group[i].each_total = cnt;
        for(s = 0; s < cnt; s++)
        {
            if((p + 1) >= end)
            {
                return 0;
            }
            g_cfg_init.group[i].each[s].station = *p++;
            g_cfg_init.group[i].each[s].block_group = *p++;
        }
    }
    return 1;
}

void cfg_table_apply_io_ctrl_from_init(void)
{
    extern u8 ioCtrl_num;
    extern u8 ext_host_flag;

    if(g_cfg_init.slave_num != 0)
    {
        ioCtrl_num = g_cfg_init.slave_num;
        ext_host_flag = g_cfg_init.expand_master;
    }
    else
    {
        ioCtrl_num = 0;
    }
    if(ioCtrl_num > MAX_SLAVE_NUM)
    {
        ioCtrl_num = 0;
    }
}

static u8 cfg_table_topo_build(u8 *out, u8 max_n)
{
    u8 g, s, n, k;
    u8 sta;

    n = 0;
    if(out == NULL || max_n == 0)
    {
        return 0;
    }
    /* group_total=0：按 slave_num 生成连续站号 2..(slave_num+1) */
    if(g_cfg_init.group_total == 0 && g_cfg_init.slave_num > 0)
    {
        u8 sta_end = (u8)(g_cfg_init.slave_num + 1u);
        u8 sta;

        if(sta_end > 97u)
        {
            sta_end = 97u;
        }
        for(sta = 2; sta <= sta_end && n < max_n; sta++)
        {
            out[n++] = sta;
        }
        return n;
    }
    for(g = 0; g < g_cfg_init.group_total && g < CFG_MAX_GROUP; g++)
    {
        for(s = 0; s < g_cfg_init.group[g].each_total
                && s < CFG_MAX_STATION_PER_GROUP; s++)
        {
            sta = g_cfg_init.group[g].each[s].station;
            if(sta < 2)
            {
                continue;
            }
            for(k = 0; k < n; k++)
            {
                if(out[k] == sta)
                {
                    break;
                }
            }
            if(k < n)
            {
                continue;
            }
            if(n >= max_n)
            {
                return n;
            }
            out[n++] = sta;
        }
    }
    /* 按站号升序，与菊花链赋号顺序一致 */
    {
        u8 a, b, t;
        for(a = 0; a + 1u < n; a++)
        {
            for(b = (u8)(a + 1u); b < n; b++)
            {
                if(out[a] > out[b])
                {
                    t = out[a];
                    out[a] = out[b];
                    out[b] = t;
                }
            }
        }
    }
    return n;
}

u8 cfg_table_topo_station_count(void)
{
    u8 buf[CFG_MAX_BOARD];
    return cfg_table_topo_build(buf, CFG_MAX_BOARD);
}

u8 cfg_table_topo_station_at(u8 idx)
{
    u8 buf[CFG_MAX_BOARD];
    u8 n;

    n = cfg_table_topo_build(buf, CFG_MAX_BOARD);
    if(idx >= n)
    {
        return 0;
    }
    return buf[idx];
}

void cfg_set_flag(u8 bit)
{
    g_cfg_flags |= bit;
    g_cfg_host_ready = ((g_cfg_flags & CFG_FLAG_HOST_READY) == CFG_FLAG_HOST_READY) ? 1 : 0;
}

u8 cfg_host_ready(void)
{
    return g_cfg_host_ready;
}

/*============================================================================
 * 运行中热更新 / UDP 点位分段接收
 *
 * - 分段先写入暂存区，CRC 通过后才替换运行表（半包断网不影响当前配置）
 * - 要求 seg_index 连续 0..N-1；乱序/跳号/超时 → 丢弃暂存
 * - 成功落地：冻结 IO → 中止进行中的下发 → 清运行态 → 再分发 FUNC/TIME
 *============================================================================*/
volatile u8 g_cfg_io_freeze = 0;

typedef struct
{
    u8  active;       /* 1=正在收某次 1426 会话 */
    u16 field0;       /* fmt2=2 或 fmt1=port_num */
    u16 ver;
    u16 crc;
    u16 seg_total;
    u16 next_index;   /* 期望的下一片序号 */
    u16 got_bytes;
    u16 idle_ms;
} cfg_pin_rx_t;

/* 与 cfg_store 上电拼包共用，避免两份大数组撑爆 64KB SRAM */
u8 g_cfg_bind_workspace[CFG_BIND_BLOB_MAX];

static cfg_pin_rx_t s_pin_rx;
u8 g_cfg_pin_rx_last_result = CFG_PIN_RX_OK;

void cfg_pin_rx_abort(void)
{
    memset(&s_pin_rx, 0, sizeof(s_pin_rx));
}

void cfg_runtime_hot_apply(void)
{
    /* 配置切换窗口：暂停业务 IO，避免切换前后点位混用；随即按新表重推各站 PIN（重绑入口） */
    g_cfg_io_freeze = 1;
    cfg_can_abort_distribute();
    cfg_logic_runtime_reset();
    io_port_init();
    can_bus_port_state_reset_all();
    g_cfg_io_freeze = 0;

    if(cfg_host_ready())
    {
        cfg_can_start_distribute();
        can_bus_request_sync_all();
    }
}

void cfg_pin_rx_tick_1ms(void)
{
    if(!s_pin_rx.active)
    {
        return;
    }
    if(s_pin_rx.idle_ms < 0xFFFF)
    {
        s_pin_rx.idle_ms++;
    }
    /* 上位机断网/只发一半：超时丢弃暂存，运行表保持原样 */
    if(s_pin_rx.idle_ms >= CFG_PIN_RX_IDLE_MS)
    {
        cfg_pin_rx_abort();
    }
}

/*
 * 喂入一片 0x1426 数据段。
 * 返回: CFG_PIN_RX_*（0=继续等片不答；1=成功；≥2=失败原因，运行表未改）
 */
u8 cfg_pin_rx_segment(u16 field0, u16 ver, u16 crc,
                      u16 seg_total, u16 seg_index, u16 seg_bytes, const u8 *data)
{
    u16 fmt;

    if(data == NULL || seg_total == 0 || seg_index >= seg_total)
    {
        return CFG_PIN_RX_BAD_PARAM;
    }
    if(field0 == CFG_PIN_FMT_BIND || field0 == CFG_PIN_FMT_BIND_MERGE)
    {
        fmt = field0;
    }
    else if(field0 > 0 && field0 <= CFG_BIZ_PORT_MAX)
    {
        fmt = CFG_PIN_FMT_SLOT;
    }
    else
    {
        return CFG_PIN_RX_BAD_PARAM;
    }

    /* 新会话：seg0 或 参数变化 → 重建暂存（不碰运行表） */
    if(!s_pin_rx.active
       || seg_index == 0
       || s_pin_rx.field0 != field0
       || s_pin_rx.ver != ver
       || s_pin_rx.crc != crc
       || s_pin_rx.seg_total != seg_total)
    {
        if(seg_index != 0)
        {
            /* 半截会话中忽然换参数且不是从 0 开始 → 丢弃 */
            cfg_pin_rx_abort();
            return CFG_PIN_RX_SEQ;
        }
        /*
         * format1 全量口记录：最小也是 n×24B。若连最小都塞不进暂存，
         * 首片即拒（OVERFLOW），避免半包占满再失败。
         * format2 样板+异类：同接线多站由上位机压缩，本机 remap 展开，不硬吃 N 板。
         */
        if(fmt == CFG_PIN_FMT_SLOT
           && ((u32)field0 * (u32)CFG_PIN_REC_SIZE_V24 > (u32)CFG_BIND_BLOB_MAX))
        {
            cfg_pin_rx_abort();
            return CFG_PIN_RX_OVERFLOW;
        }
        cfg_pin_rx_abort();
        s_pin_rx.active = 1;
        s_pin_rx.field0 = field0;
        s_pin_rx.ver = ver;
        s_pin_rx.crc = crc;
        s_pin_rx.seg_total = seg_total;
        s_pin_rx.next_index = 0;
        s_pin_rx.got_bytes = 0;
        s_pin_rx.idle_ms = 0;
    }

    s_pin_rx.idle_ms = 0;

    /* 严格顺序，防止断网重连后乱序拼包 */
    if(seg_index != s_pin_rx.next_index)
    {
        cfg_pin_rx_abort();
        return CFG_PIN_RX_SEQ;
    }
    if((u32)s_pin_rx.got_bytes + seg_bytes > CFG_BIND_BLOB_MAX)
    {
        cfg_pin_rx_abort();
        return CFG_PIN_RX_OVERFLOW;
    }

    memcpy(&g_cfg_bind_workspace[s_pin_rx.got_bytes], data, seg_bytes);
    s_pin_rx.got_bytes = (u16)(s_pin_rx.got_bytes + seg_bytes);
    s_pin_rx.next_index++;

    if(s_pin_rx.next_index < s_pin_rx.seg_total)
    {
        return CFG_PIN_RX_MORE;
    }

    /* 收齐：校验并原子切换 */
    {
        u8 ok = 0;
        if(fmt == CFG_PIN_FMT_BIND)
        {
            ok = cfg_table_set_from_bind_blob(g_cfg_bind_workspace, s_pin_rx.got_bytes,
                                              s_pin_rx.ver, s_pin_rx.crc);
        }
        else if(fmt == CFG_PIN_FMT_BIND_MERGE)
        {
            ok = cfg_table_merge_from_bind_blob(g_cfg_bind_workspace, s_pin_rx.got_bytes,
                                                s_pin_rx.ver, s_pin_rx.crc);
        }
        else
        {
            u16 n = s_pin_rx.field0;
            u16 need45 = (u16)(n * CFG_PIN_REC_SIZE);
            u16 need43 = (u16)(n * CFG_PIN_REC_SIZE_JAM12);
            u16 need27 = (u16)(n * CFG_PIN_REC_SIZE_OR_JAM4);
            u16 need24 = (u16)(n * CFG_PIN_REC_SIZE_V24);

            if(s_pin_rx.got_bytes == need45)
            {
                ok = cfg_table_set_from_recs_raw(g_cfg_bind_workspace, n, s_pin_rx.ver,
                                                 s_pin_rx.crc, CFG_PIN_REC_SIZE);
            }
            else if(s_pin_rx.got_bytes == need43)
            {
                ok = cfg_table_set_from_recs_raw(g_cfg_bind_workspace, n, s_pin_rx.ver,
                                                 s_pin_rx.crc, CFG_PIN_REC_SIZE_JAM12);
            }
            else if(s_pin_rx.got_bytes == need27)
            {
                ok = cfg_table_set_from_recs_raw(g_cfg_bind_workspace, n, s_pin_rx.ver,
                                                 s_pin_rx.crc, CFG_PIN_REC_SIZE_OR_JAM4);
            }
            else if(s_pin_rx.got_bytes == need24)
            {
                ok = cfg_table_set_from_recs_raw(g_cfg_bind_workspace, n, s_pin_rx.ver,
                                                 s_pin_rx.crc, CFG_PIN_REC_SIZE_V24);
            }
        }
        cfg_pin_rx_abort();
        if(!ok)
        {
            return CFG_PIN_RX_PARSE;
        }
        cfg_set_flag(CFG_FLAG_1426);
        cfg_store_mark_runtime_dirty(); /* 重绑已进 RAM，待从机 ACK 后自动固化 */
        cfg_runtime_hot_apply();
        return CFG_PIN_RX_OK;
    }
}

u8 cfg_jam_fanout_edit(u8 station, u8 op, u8 count, const u16 *ports)
{
    u16 idx[CFG_MAX_LOCAL_PORT];
    u8 n = 0;
    u8 i, j, k;
    u16 list[CFG_JAM_EXTRA_MAX];
    u8 list_n;

    if(station < 2 || (station - 2) >= MAX_SLAVE_NUM)
    {
        return 0;
    }
    if(op != CFG_JAM_OP_CLEAR && count > 0 && ports == NULL)
    {
        return 0;
    }
    if(count > CFG_MAX_JAM_FANOUT)
    {
        return 0;
    }

    for(i = 0; i < g_cfg_port_num && n < CFG_MAX_LOCAL_PORT; i++)
    {
        if(unload_port_assign[i].station == station)
        {
            idx[n++] = i;
        }
    }
    if(n == 0)
    {
        return 0;
    }

    if(op == CFG_JAM_OP_CLEAR)
    {
        for(i = 0; i < n; i++)
        {
            unload_port_assign[idx[i]].jam_extra_num = 0;
        }
        return 1;
    }

    if(op == CFG_JAM_OP_REPLACE)
    {
        for(i = 0; i < n; i++)
        {
            assign_apply_jam_list(&unload_port_assign[idx[i]], ports, count);
        }
        return 1;
    }

    if(op == CFG_JAM_OP_ADD)
    {
        for(i = 0; i < n; i++)
        {
            unload_port_assign_st *a = &unload_port_assign[idx[i]];
            for(j = 0; j < count; j++)
            {
                u8 found = 0;
                if(ports[j] == a->port_no)
                {
                    continue;
                }
                for(k = 0; k < a->jam_extra_num; k++)
                {
                    if(a->jam_extra_no[k] == ports[j])
                    {
                        found = 1;
                        break;
                    }
                }
                if(!found && a->jam_extra_num < CFG_JAM_EXTRA_MAX)
                {
                    a->jam_extra_no[a->jam_extra_num] = ports[j];
                    a->jam_extra_num++;
                }
            }
        }
        return 1;
    }

    if(op == CFG_JAM_OP_REMOVE)
    {
        for(i = 0; i < n; i++)
        {
            unload_port_assign_st *a = &unload_port_assign[idx[i]];
            list_n = 0;
            for(k = 0; k < a->jam_extra_num; k++)
            {
                u8 drop = 0;
                for(j = 0; j < count; j++)
                {
                    if(a->jam_extra_no[k] == ports[j])
                    {
                        drop = 1;
                        break;
                    }
                }
                if(!drop && list_n < CFG_JAM_EXTRA_MAX)
                {
                    list[list_n++] = a->jam_extra_no[k];
                }
            }
            a->jam_extra_num = list_n;
            for(k = 0; k < list_n; k++)
            {
                a->jam_extra_no[k] = list[k];
            }
        }
        return 1;
    }

    return 0;
}

u8 cfg_extio_set_from_payload(const u8 *payload, u16 len)
{
    u16 flags;
    u8 n, i;
    cfg_xout_route_t tmp[CFG_XOUT_MAX];

    if(payload == NULL || len < 3)
    {
        return 0;
    }
    flags = (u16)(payload[0] | (payload[1] << 8));
    n = payload[2];
    if(n > CFG_XOUT_MAX)
    {
        return 0;
    }
    if(len < (u16)(3u + (u16)n * 4u))
    {
        return 0;
    }
    /* 先全部校验，失败不改动已有 upload_flags/xout */
    for(i = 0; i < n; i++)
    {
        const u8 *r = payload + 3 + i * 4;
        tmp[i].station = r[0];
        tmp[i].pin = r[1];
        tmp[i].src_ch = r[2];
        tmp[i].flags = r[3];
        if(tmp[i].station < 2
           || (tmp[i].station - 2) >= MAX_SLAVE_NUM
           || tmp[i].pin >= 8
           || tmp[i].src_ch >= 6)
        {
            return 0;
        }
    }
    g_cfg_upload_flags = flags;
    g_cfg_xout_n = n;
    for(i = 0; i < n; i++)
    {
        g_cfg_xout[i] = tmp[i];
    }
    return 1;
}

u16 cfg_upload_flags_effective(void)
{
    u16 f = g_cfg_upload_flags;
    if(g_cfg_logic.ready)
    {
        f = (u16)(f | g_cfg_logic.hdr.report_flags);
    }
    return f;
}

