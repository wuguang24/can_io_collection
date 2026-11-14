/*
 * 主机本地配置固化：
 *  - 模板页：拓扑(1423)+偏移(1424)+滤波(1425)+样板板卡 PIN 绑定(物理脚↔CH)
 *  - 异类页：最多 8 块板的 PIN 覆盖
 *  - 灯效：1428 规则随模板保存（逻辑通道 CH0~5；无规则则上电全灭）
 *  - 扩展：142F upload_flags + 跨站 xout 随模板保存
 * 上电若 Flash 有效且尚未被 WCS 灌满，则展开并下发从机。
 */
#include "cfg_store.h"
#include "cfg_table.h"
#include "cfg_crc.h"
#include "cfg_can_ext.h"
#include "cfg_logic.h"
#include "flash.h"
#include "main.h"
#include "stm32f10x_flash.h"
#include "string.h"
#include "FreeRTOS.h"

/* WCS 热更新后置 1；PIN 分发全成功则自动固化，避免重绑掉电丢失 */
static u8 s_runtime_dirty = 0;

#pragma pack(1)
typedef struct
{
    u16 magic;
    u16 ver;
    u16 crc;          /* 覆盖 store_seq~结构体尾（不含 magic/ver/crc） */
    u16 store_seq;    /* 双区序号；reserved=0 视为 seq=0 */
    cfg_init_conf_t init;
    u16 ip_start;
    u16 ip_end;
    cfg_time_t time;
    u8  tpl_station;  /* 样板站号 */
    u8  blob_len;
    u8  blob[CFG_STORE_BOARD_BLOB_MAX]; /* 单板 format2 片段(含 station 头) */
    u8  pad;
    /* 灯效规则随模板固化（无规则则 rule_count=0，上电保持全灭） */
    cfg_logic_hdr_t logic_hdr;
    u8  logic_rules[CFG_COMBO_RULE_MAX * CFG_COMBO_RULE_SIZE];
    /* 0x142F：上报扩展 + 跨站 OUT（随模板掉电保持） */
    u16 upload_flags;
    u8  xout_n;
    u8  xout_pad;
    u8  xout_raw[CFG_XOUT_MAX * 4];
} cfg_store_tpl_t;

typedef struct
{
    u8 used;
    u8 station;
    u8 blob_len;
    u8 pad;
    u8 blob[CFG_STORE_BOARD_BLOB_MAX];
} cfg_store_exc_slot_t;

typedef struct
{
    u16 magic;
    u16 crc;
    u8  count;
    u8  reserved[3];
    cfg_store_exc_slot_t slot[CFG_STORE_EXCEPT_MAX];
} cfg_store_exc_t;
#pragma pack()

/* TPL+EXC 须同时驻留：commit 成对写 Flash，不可用 union（memset exc 会擦掉 tpl） */
typedef struct
{
    cfg_store_tpl_t tpl;
    cfg_store_exc_t exc;
} cfg_store_wr_t;

static cfg_store_wr_t *s_wr;

static u8 cfg_store_wr_ensure(void)
{
    if(s_wr != NULL)
    {
        return 1;
    }
    s_wr = (cfg_store_wr_t *)pvPortMalloc(sizeof(cfg_store_wr_t));
    return (s_wr != NULL) ? 1u : 0u;
}

/* autosave / apply 工作区放 BSS，避免在 main 小栈或 CanIo 任务栈上开大数组 */
static u8  s_autosave_stations[CFG_MAX_BOARD];
static u8  s_autosave_blob[CFG_STORE_BOARD_BLOB_MAX];
static u8  s_apply_stations[CFG_MAX_BOARD];
static u8  s_boot_apply_pending = 0;

static u32 s_active_tpl_addr = CFG_FLASH_BANK1_TPL_ADDR;
static u32 s_active_exc_addr = CFG_FLASH_BANK1_EXC_ADDR;

static u8 cfg_flash_program(u32 addr, const void *src, u16 nbytes)
{
    const u8 *p = (const u8 *)src;
    u16 half_n;
    u16 i;
    u16 hw;

    if((nbytes == 0) || (nbytes > CFG_FLASH_PAGE_SIZE))
    {
        return 0;
    }
    half_n = (u16)((nbytes + 1u) / 2u);

    /* 擦写期间关中断并挂起调度，避免 ISR/任务取指与 Flash 操作冲突 */
    __disable_irq();
    vTaskSuspendAll();
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    if(FLASH_ErasePage(addr) != FLASH_COMPLETE)
    {
        FLASH_Lock();
        xTaskResumeAll();
        __enable_irq();
        return 0;
    }
    for(i = 0; i < half_n; i++)
    {
        u16 off = (u16)(i * 2u);
        hw = p[off];
        if((u16)(off + 1u) < nbytes)
        {
            hw = (u16)(hw | ((u16)p[off + 1u] << 8));
        }
        else
        {
            hw = (u16)(hw | 0xFF00u);
        }
        if(FLASH_ProgramHalfWord(addr + off, hw) != FLASH_COMPLETE)
        {
            FLASH_Lock();
            xTaskResumeAll();
            __enable_irq();
            return 0;
        }
    }
    FLASH_Lock();
    xTaskResumeAll();
    __enable_irq();
    return 1;
}

static u16 cfg_store_crc_buf(const u8 *p, u16 len)
{
    return cfg_crc16(p, len);
}

static u8 cfg_store_tpl_valid(const cfg_store_tpl_t *t)
{
    u16 crc;

    if(t == NULL || t->magic != CFG_STORE_MAGIC_TPL)
    {
        return 0;
    }
    if(t->blob_len == 0 || t->blob_len > CFG_STORE_BOARD_BLOB_MAX)
    {
        return 0;
    }
    if(t->init.group_total > CFG_MAX_GROUP)
    {
        return 0;
    }
    /* group_total=0 且 slave_num>0：与 1423 简化拓扑一致，允许落盘 */
    if(t->init.group_total == 0 && t->init.slave_num == 0)
    {
        return 0;
    }
    /* 必须跳过 magic+ver+crc，否则写入时 crc=0、读回校验时 crc≠0，永远失败 */
    crc = cfg_store_crc_buf((const u8 *)t + 6u, (u16)(sizeof(*t) - 6u));
    return (crc == t->crc) ? 1u : 0u;
}

static u8 cfg_store_exc_valid(const cfg_store_exc_t *e)
{
    u16 crc;

    if(e == NULL || e->magic != CFG_STORE_MAGIC_EXC)
    {
        return 0;
    }
    if(e->count > CFG_STORE_EXCEPT_MAX)
    {
        return 0;
    }
    crc = cfg_store_crc_buf((const u8 *)e + 4u, (u16)(sizeof(*e) - 4u));
    return (crc == e->crc) ? 1u : 0u;
}

static const cfg_store_tpl_t *cfg_store_tpl_at(u32 addr)
{
    return (const cfg_store_tpl_t *)addr;
}

static const cfg_store_exc_t *cfg_store_exc_at(u32 addr)
{
    return (const cfg_store_exc_t *)addr;
}

static const cfg_store_tpl_t *cfg_store_tpl_ptr(void)
{
    return cfg_store_tpl_at(s_active_tpl_addr);
}

static const cfg_store_exc_t *cfg_store_exc_ptr(void)
{
    return cfg_store_exc_at(s_active_exc_addr);
}

static u8 cfg_store_bank_pair_valid(u32 tpl_addr, u32 exc_addr)
{
    const cfg_store_tpl_t *t = cfg_store_tpl_at(tpl_addr);
    const cfg_store_exc_t *e = cfg_store_exc_at(exc_addr);

    if(!cfg_store_tpl_valid(t))
    {
        return 0;
    }
    /* 异类页可空（count=0）；有 count 则须 CRC 合法 */
    if(e->magic == CFG_STORE_MAGIC_EXC && e->count > 0)
    {
        return cfg_store_exc_valid(e) ? 1u : 0u;
    }
    return 1u;
}

void cfg_store_refresh_active(void)
{
    u32 banks[2][2] = {
        { CFG_FLASH_BANK0_TPL_ADDR, CFG_FLASH_BANK0_EXC_ADDR },
        { CFG_FLASH_BANK1_TPL_ADDR, CFG_FLASH_BANK1_EXC_ADDR }
    };
    u8 best_i = 0xFF;
    u16 best_seq = 0;
    u8 i;

    for(i = 0; i < 2; i++)
    {
        const cfg_store_tpl_t *t;

        if(!cfg_store_bank_pair_valid(banks[i][0], banks[i][1]))
        {
            continue;
        }
        t = cfg_store_tpl_at(banks[i][0]);
        if(best_i == 0xFF || t->store_seq >= best_seq)
        {
            best_i = i;
            best_seq = t->store_seq;
        }
    }
    if(best_i != 0xFF)
    {
        s_active_tpl_addr = banks[best_i][0];
        s_active_exc_addr = banks[best_i][1];
    }
    else
    {
        s_active_tpl_addr = CFG_FLASH_BANK1_TPL_ADDR;
        s_active_exc_addr = CFG_FLASH_BANK1_EXC_ADDR;
    }
}

static void cfg_store_inactive_addrs(u32 *tpl_addr, u32 *exc_addr)
{
    if(s_active_tpl_addr == CFG_FLASH_BANK0_TPL_ADDR)
    {
        *tpl_addr = CFG_FLASH_BANK1_TPL_ADDR;
        *exc_addr = CFG_FLASH_BANK1_EXC_ADDR;
    }
    else
    {
        *tpl_addr = CFG_FLASH_BANK0_TPL_ADDR;
        *exc_addr = CFG_FLASH_BANK0_EXC_ADDR;
    }
}

static u16 cfg_store_next_seq(void)
{
    const cfg_store_tpl_t *t = cfg_store_tpl_ptr();

    if(cfg_store_tpl_valid(t))
    {
        return (u16)(t->store_seq + 1u);
    }
    return 1u;
}

static u8 cfg_store_commit_pair(u32 tpl_addr, u32 exc_addr)
{
    const cfg_store_tpl_t *rt;
    const cfg_store_exc_t *re;

    if(!cfg_store_wr_ensure())
    {
        g_cfg_store_autosave_fail = CFG_FAIL_SAVE_HEAP;
        return 0;
    }
    if(!cfg_flash_program(tpl_addr, &s_wr->tpl, (u16)sizeof(s_wr->tpl)))
    {
        g_cfg_store_autosave_fail = CFG_FAIL_SAVE_FLASH;
        return 0;
    }
    if(!cfg_flash_program(exc_addr, &s_wr->exc, (u16)sizeof(s_wr->exc)))
    {
        g_cfg_store_autosave_fail = CFG_FAIL_SAVE_FLASH;
        return 0;
    }
    rt = cfg_store_tpl_at(tpl_addr);
    re = cfg_store_exc_at(exc_addr);
    if(!cfg_store_bank_pair_valid(tpl_addr, exc_addr)
       || rt->store_seq != s_wr->tpl.store_seq)
    {
        g_cfg_store_autosave_fail = CFG_FAIL_SAVE_VERIFY;
        return 0;
    }
    (void)re;
    s_active_tpl_addr = tpl_addr;
    s_active_exc_addr = exc_addr;
    return 1;
}

static void cfg_store_exc_load_active(void)
{
    const cfg_store_exc_t *pe = cfg_store_exc_ptr();

    if(!cfg_store_wr_ensure())
    {
        return;
    }
    if(cfg_store_exc_valid(pe))
    {
        memcpy(&s_wr->exc, pe, sizeof(s_wr->exc));
    }
    else
    {
        memset(&s_wr->exc, 0, sizeof(s_wr->exc));
        s_wr->exc.magic = CFG_STORE_MAGIC_EXC;
    }
}

/* 从运行表抽出一站 → format2 单板片段 */
static u8 cfg_store_pack_station(u8 station, u8 *out, u8 *out_len)
{
    u16 idx[CFG_MAX_LOCAL_PORT];
    u8  lpc = 0;
    u8  in_cnt = 0;
    u8  out_cnt = 0;
    u8  jam_n = 0;
    u16 jam_ports[CFG_MAX_JAM_FANOUT];
    cfg_pin_bind_t inb[CFG_MAX_BIND_PER_BOARD];
    cfg_pin_bind_t outb[CFG_MAX_BIND_PER_BOARD];
    u16 print_ms = 0;
    u8 i, j;
    u8 *p;
    u8 board_clamp_pin = INPUT_INVALID;
    u8 need_len;

    if(out == NULL || out_len == NULL || station < 2)
    {
        return 0;
    }

    for(i = 0; i < g_cfg_port_num && lpc < CFG_MAX_LOCAL_PORT; i++)
    {
        if(unload_port_assign[i].station == station)
        {
            idx[lpc++] = i;
        }
    }
    if(lpc == 0)
    {
        return 0;
    }
    print_ms = g_cfg_print_delay_ms[idx[0]];

    /* 板级卡包：各口 clamp 脚相同且 > 本口扇出 */
    {
        u8 pin0 = unload_port_assign[idx[0]].in_clamp_pack;
        u8 same = 1;
        for(i = 1; i < lpc; i++)
        {
            if(unload_port_assign[idx[i]].in_clamp_pack != pin0)
            {
                same = 0;
                break;
            }
        }
        if(same && pin0 < INPUT_MAX)
        {
            board_clamp_pin = pin0;
            if(in_cnt < CFG_MAX_BIND_PER_BOARD)
            {
                inb[in_cnt].pin = pin0;
                inb[in_cnt].func = CFG_IN_FUNC_CLAMP;
                inb[in_cnt].local_port = CFG_BIND_LOCAL_BOARD;
                inb[in_cnt].flags = 0;
                in_cnt++;
            }
            /* jam_fanout：以运行态 jam_extra 并集为准（支持 0x142E 删减后固化） */
            for(i = 0; i < lpc && jam_n < CFG_MAX_JAM_FANOUT; i++)
            {
                unload_port_assign_st *a = &unload_port_assign[idx[i]];
                for(j = 0; j < a->jam_extra_num && jam_n < CFG_MAX_JAM_FANOUT; j++)
                {
                    u16 pn = a->jam_extra_no[j];
                    u8 found = 0;
                    u8 k;
                    for(k = 0; k < jam_n; k++)
                    {
                        if(jam_ports[k] == pn)
                        {
                            found = 1;
                            break;
                        }
                    }
                    if(!found)
                    {
                        jam_ports[jam_n++] = pn;
                    }
                }
            }
        }
        else
        {
            for(i = 0; i < lpc && jam_n < CFG_MAX_JAM_FANOUT; i++)
            {
                unload_port_assign_st *a = &unload_port_assign[idx[i]];
                for(j = 0; j < a->jam_extra_num && jam_n < CFG_MAX_JAM_FANOUT; j++)
                {
                    u16 pn = a->jam_extra_no[j];
                    u8 found = 0;
                    u8 k;
                    for(k = 0; k < jam_n; k++)
                    {
                        if(jam_ports[k] == pn)
                        {
                            found = 1;
                            break;
                        }
                    }
                    if(!found)
                    {
                        jam_ports[jam_n++] = pn;
                    }
                }
            }
        }
    }

    for(i = 0; i < lpc; i++)
    {
        unload_port_assign_st *a = &unload_port_assign[idx[i]];
        u16 gi = idx[i];

        #define ADD_IN(_pin, _func, _fl) do { \
            if((_pin) < INPUT_MAX && in_cnt < CFG_MAX_BIND_PER_BOARD) { \
                inb[in_cnt].pin = (u8)(_pin); \
                inb[in_cnt].func = (u8)(_func); \
                inb[in_cnt].local_port = i; \
                inb[in_cnt].flags = (u8)(_fl); \
                in_cnt++; \
            } \
        } while(0)

        if(board_clamp_pin >= INPUT_MAX || a->in_clamp_pack != board_clamp_pin)
        {
            ADD_IN(a->in_clamp_pack, CFG_IN_FUNC_CLAMP, 0);
        }
        if((g_cfg_port_flags[gi] & CFG_PORT_F_LOCK_PRINT_MERGED)
           && a->in_lock < INPUT_MAX && a->in_lock == a->in_print)
        {
            ADD_IN(a->in_lock, CFG_IN_FUNC_LOCK_PRINT, 0);
            {
                u8 k;
                for(k = 0; k < g_cfg_in_lock_or_n[gi]; k++)
                {
                    ADD_IN(g_cfg_in_lock_or[gi][k], CFG_IN_FUNC_LOCK_PRINT, CFG_BIND_F_OR);
                }
            }
        }
        else
        {
            ADD_IN(a->in_lock, CFG_IN_FUNC_LOCK, 0);
            {
                u8 k;
                for(k = 0; k < g_cfg_in_lock_or_n[gi]; k++)
                {
                    ADD_IN(g_cfg_in_lock_or[gi][k], CFG_IN_FUNC_LOCK, CFG_BIND_F_OR);
                }
            }
            ADD_IN(a->in_print, CFG_IN_FUNC_PRINT, 0);
            {
                u8 k;
                for(k = 0; k < g_cfg_in_print_or_n[gi]; k++)
                {
                    ADD_IN(g_cfg_in_print_or[gi][k], CFG_IN_FUNC_PRINT, CFG_BIND_F_OR);
                }
            }
        }
        ADD_IN(a->in_full_pack, CFG_IN_FUNC_FULL, 0);
        {
            u8 k;
            for(k = 0; k < g_cfg_in_full_or_n[gi]; k++)
            {
                ADD_IN(g_cfg_in_full_or[gi][k], CFG_IN_FUNC_FULL, CFG_BIND_F_OR);
            }
            if(g_cfg_in_full_or_n[gi] == 0 && g_cfg_in_full2[gi] < INPUT_MAX)
            {
                ADD_IN(g_cfg_in_full2[gi], CFG_IN_FUNC_FULL, CFG_BIND_F_OR);
            }
        }
        ADD_IN(a->in_half_full, CFG_IN_FUNC_HALF, 0);
        ADD_IN(a->in_full_tier1, CFG_IN_FUNC_FULL_TIER1, 0);
        ADD_IN(a->in_full_tier2, CFG_IN_FUNC_FULL_TIER2, 0);
        ADD_IN(a->in_unload_check, CFG_IN_FUNC_UNLOAD_CHECK, 0);

        #undef ADD_IN
    }

    /* 板级公共 OUT：各口同脚则打成 local_port=0xFF，避免 blob 膨胀 */
    {
        u8 bo_pin[6];
        u8 bo_func[6];
        u8 bo_same[6];
        u8 bi;

        bo_pin[0] = unload_port_assign[idx[0]].out_lock_light;
        bo_func[0] = CFG_OUT_FUNC_LOCK_LIGHT;
        bo_pin[1] = unload_port_assign[idx[0]].out_print_light;
        bo_func[1] = CFG_OUT_FUNC_PRINT_LIGHT;
        bo_pin[2] = unload_port_assign[idx[0]].out_alarm_red;
        bo_func[2] = CFG_OUT_FUNC_RED;
        bo_pin[3] = unload_port_assign[idx[0]].out_alarm_yellow;
        bo_func[3] = CFG_OUT_FUNC_YELLOW;
        bo_pin[4] = unload_port_assign[idx[0]].out_alarm_green;
        bo_func[4] = CFG_OUT_FUNC_GREEN;
        bo_pin[5] = unload_port_assign[idx[0]].out_alarm_buzzer;
        bo_func[5] = CFG_OUT_FUNC_BUZZER;

        for(bi = 0; bi < 6; bi++)
        {
            bo_same[bi] = (bo_pin[bi] < OUTPUT_MAX) ? 1 : 0;
        }
        for(i = 1; i < lpc; i++)
        {
            unload_port_assign_st *a = &unload_port_assign[idx[i]];
            if(a->out_lock_light != bo_pin[0]) { bo_same[0] = 0; }
            if(a->out_print_light != bo_pin[1]) { bo_same[1] = 0; }
            if(a->out_alarm_red != bo_pin[2]) { bo_same[2] = 0; }
            if(a->out_alarm_yellow != bo_pin[3]) { bo_same[3] = 0; }
            if(a->out_alarm_green != bo_pin[4]) { bo_same[4] = 0; }
            if(a->out_alarm_buzzer != bo_pin[5]) { bo_same[5] = 0; }
        }
        for(bi = 0; bi < 6; bi++)
        {
            if(bo_same[bi] && out_cnt < CFG_MAX_BIND_PER_BOARD)
            {
                outb[out_cnt].pin = bo_pin[bi];
                outb[out_cnt].func = bo_func[bi];
                outb[out_cnt].local_port = CFG_BIND_LOCAL_BOARD;
                outb[out_cnt].flags = 0;
                out_cnt++;
            }
        }

        for(i = 0; i < lpc; i++)
        {
            unload_port_assign_st *a = &unload_port_assign[idx[i]];
            #define ADD_OUT_LOC(_pin, _func, _board_same) do { \
                if(!(_board_same) && (_pin) < OUTPUT_MAX && out_cnt < CFG_MAX_BIND_PER_BOARD) { \
                    outb[out_cnt].pin = (u8)(_pin); \
                    outb[out_cnt].func = (u8)(_func); \
                    outb[out_cnt].local_port = i; \
                    outb[out_cnt].flags = 0; \
                    out_cnt++; \
                } \
            } while(0)
            ADD_OUT_LOC(a->out_lock_light, CFG_OUT_FUNC_LOCK_LIGHT, bo_same[0]);
            ADD_OUT_LOC(a->out_print_light, CFG_OUT_FUNC_PRINT_LIGHT, bo_same[1]);
            if(g_cfg_out_lock2[idx[i]] < OUTPUT_MAX && out_cnt < CFG_MAX_BIND_PER_BOARD)
            {
                outb[out_cnt].pin = g_cfg_out_lock2[idx[i]];
                outb[out_cnt].func = CFG_OUT_FUNC_LOCK_LIGHT;
                outb[out_cnt].local_port = i;
                outb[out_cnt].flags = CFG_BIND_F_OR;
                out_cnt++;
            }
            if(g_cfg_out_print2[idx[i]] < OUTPUT_MAX && out_cnt < CFG_MAX_BIND_PER_BOARD)
            {
                outb[out_cnt].pin = g_cfg_out_print2[idx[i]];
                outb[out_cnt].func = CFG_OUT_FUNC_PRINT_LIGHT;
                outb[out_cnt].local_port = i;
                outb[out_cnt].flags = CFG_BIND_F_OR;
                out_cnt++;
            }
            ADD_OUT_LOC(a->out_alarm_red, CFG_OUT_FUNC_RED, bo_same[2]);
            ADD_OUT_LOC(a->out_alarm_yellow, CFG_OUT_FUNC_YELLOW, bo_same[3]);
            ADD_OUT_LOC(a->out_alarm_green, CFG_OUT_FUNC_GREEN, bo_same[4]);
            ADD_OUT_LOC(a->out_alarm_buzzer, CFG_OUT_FUNC_BUZZER, bo_same[5]);
            #undef ADD_OUT_LOC
        }
    }

    need_len = (u8)(9u + lpc * 2u + in_cnt * 4u + out_cnt * 4u + jam_n * 2u);
    if(need_len > CFG_STORE_BOARD_BLOB_MAX)
    {
        return 0;
    }

    {
        u16 bflags = 0;
        if(station >= 2 && (station - 2) < MAX_SLAVE_NUM)
        {
            bflags = g_cfg_board_flags[station - 2];
        }
        p = out;
        *p++ = station;
        *p++ = lpc;
        *p++ = in_cnt;
        *p++ = out_cnt;
        *p++ = (u8)(bflags & 0xFF);
        *p++ = (u8)(bflags >> 8);
    }
    *p++ = (u8)(print_ms & 0xFF);
    *p++ = (u8)(print_ms >> 8);
    *p++ = jam_n;
    for(i = 0; i < lpc; i++)
    {
        u16 pn = unload_port_assign[idx[i]].port_no;
        *p++ = (u8)(pn & 0xFF);
        *p++ = (u8)(pn >> 8);
    }
    for(i = 0; i < in_cnt; i++)
    {
        *p++ = inb[i].pin;
        *p++ = inb[i].func;
        *p++ = inb[i].local_port;
        *p++ = inb[i].flags;
    }
    for(i = 0; i < out_cnt; i++)
    {
        *p++ = outb[i].pin;
        *p++ = outb[i].func;
        *p++ = outb[i].local_port;
        *p++ = outb[i].flags;
    }
    for(i = 0; i < jam_n; i++)
    {
        *p++ = (u8)(jam_ports[i] & 0xFF);
        *p++ = (u8)(jam_ports[i] >> 8);
    }
    *out_len = need_len;
    return 1;
}

/* 把样板 blob 改写成目标站：站号 + 格号按 ip_start+(sta-2)*lpc+lp */
static u8 cfg_store_remap_board(const u8 *src, u8 src_len, u8 new_sta,
                                u16 ip_start, u8 *dst, u8 *dst_len)
{
    u8 lpc, in_cnt, out_cnt, jam_n;
    u8 i;
    u8 *p;
    const u8 *s;
    u16 need;

    if(src == NULL || src_len < 9 || dst == NULL || dst_len == NULL)
    {
        return 0;
    }
    s = src;
    lpc = s[1];
    in_cnt = s[2];
    out_cnt = s[3];
    jam_n = s[8];
    need = (u16)(9u + lpc * 2u + in_cnt * 4u + out_cnt * 4u + jam_n * 2u);
    if(need != src_len || need > CFG_STORE_BOARD_BLOB_MAX || lpc == 0 || lpc > CFG_MAX_LOCAL_PORT)
    {
        return 0;
    }

    memcpy(dst, src, src_len);
    dst[0] = new_sta;
    p = dst + 9;
    for(i = 0; i < lpc; i++)
    {
        u16 pn = (u16)(ip_start + (u16)(new_sta - 2u) * (u16)lpc + i);
        p[0] = (u8)(pn & 0xFF);
        p[1] = (u8)(pn >> 8);
        p += 2;
    }
    p = dst + 9 + lpc * 2 + in_cnt * 4 + out_cnt * 4;
    /* jam：按样板相对下标重映射（用原 jam 与原 port_map 对齐找 local index） */
    {
        const u8 *old_ports = src + 9;
        const u8 *old_jam = src + 9 + lpc * 2 + in_cnt * 4 + out_cnt * 4;
        for(i = 0; i < jam_n; i++)
        {
            u16 old_pn = (u16)(old_jam[0] | (old_jam[1] << 8));
            u8 lp;
            u8 found = 0;
            for(lp = 0; lp < lpc; lp++)
            {
                u16 op = (u16)(old_ports[lp * 2] | (old_ports[lp * 2 + 1] << 8));
                if(op == old_pn)
                {
                    u16 np = (u16)(ip_start + (u16)(new_sta - 2u) * (u16)lpc + lp);
                    p[0] = (u8)(np & 0xFF);
                    p[1] = (u8)(np >> 8);
                    found = 1;
                    break;
                }
            }
            if(!found)
            {
                /* 保持相对偏移：差样板站格号 */
                u16 base_old = (u16)(old_ports[0] | (old_ports[1] << 8));
                u16 delta = (u16)(old_pn - base_old);
                u16 np = (u16)(ip_start + (u16)(new_sta - 2u) * (u16)lpc + delta);
                p[0] = (u8)(np & 0xFF);
                p[1] = (u8)(np >> 8);
            }
            old_jam += 2;
            p += 2;
        }
    }
    *dst_len = (u8)need;
    return 1;
}

static const u8 *cfg_store_find_exc_blob(const cfg_store_exc_t *e, u8 station, u8 *blen)
{
    u8 i;
    if(e == NULL || !cfg_store_exc_valid(e))
    {
        return NULL;
    }
    for(i = 0; i < CFG_STORE_EXCEPT_MAX; i++)
    {
        if(e->slot[i].used && e->slot[i].station == station
           && e->slot[i].blob_len > 0)
        {
            *blen = e->slot[i].blob_len;
            return e->slot[i].blob;
        }
    }
    return NULL;
}

static u8 cfg_store_collect_stations(const cfg_init_conf_t *init, u8 *stas, u8 max_n)
{
    u8 n, i;

    (void)init;
    /* 与 1423 一致：group_total=0 时按 slave_num 展开站号 */
    n = cfg_table_topo_station_count();
    if(n > max_n)
    {
        n = max_n;
    }
    for(i = 0; i < n; i++)
    {
        stas[i] = cfg_table_topo_station_at(i);
    }
    return n;
}

u8 cfg_store_apply(void)
{
    const cfg_store_tpl_t *tpl;
    const cfg_store_exc_t *exc;

    u8 sn, i;
    u8 *blob;
    u8 *w;
    u16 wlen;
    u16 crc;

    cfg_store_refresh_active();
    tpl = cfg_store_tpl_ptr();
    if(!cfg_store_tpl_valid(tpl))
    {
        return 0;
    }
    exc = cfg_store_exc_ptr();

    memcpy(&g_cfg_init, &tpl->init, sizeof(g_cfg_init));
    cfg_table_apply_io_ctrl_from_init();
    ip_offest_start = tpl->ip_start;
    ip_offest_end = tpl->ip_end;
    g_cfg_time = tpl->time;

    sn = cfg_store_collect_stations(&tpl->init, s_apply_stations, CFG_MAX_BOARD);
    if(sn == 0)
    {
        return 0;
    }
    (void)sn;

    /*
     * 只拼「样板 + 异类」进 workspace；缺站由 cfg_table_set_from_bind_blob 内 remap 展开，
     * 再按站 CAN 透传 PIN。避免 15 板整包撑爆暂存。
     */
    cfg_pin_rx_abort();
    blob = g_cfg_bind_workspace;
    blob[0] = 0;
    w = &blob[1];
    wlen = 1;

    /* 样板板（保持样板站号与格号，展开逻辑会认第一块为样板） */
    if(tpl->blob_len == 0 || tpl->blob_len > CFG_STORE_BOARD_BLOB_MAX
       || (u16)(wlen + tpl->blob_len) > CFG_BIND_BLOB_MAX)
    {
        return 0;
    }
    memcpy(w, tpl->blob, tpl->blob_len);
    w += tpl->blob_len;
    wlen = (u16)(wlen + tpl->blob_len);
    blob[0]++;

    for(i = 0; i < CFG_STORE_EXCEPT_MAX; i++)
    {
        const u8 *src;
        u8 src_len;

        if(exc == NULL || !exc->slot[i].used || exc->slot[i].blob_len == 0)
        {
            continue;
        }
        src = exc->slot[i].blob;
        src_len = exc->slot[i].blob_len;
        if(src_len > CFG_STORE_BOARD_BLOB_MAX
           || (u16)(wlen + src_len) > CFG_BIND_BLOB_MAX)
        {
            return 0;
        }
        memcpy(w, src, src_len);
        w += src_len;
        wlen = (u16)(wlen + src_len);
        blob[0]++;
    }

    crc = cfg_crc16(blob, wlen);
    if(!cfg_table_set_from_bind_blob(blob, wlen, tpl->ver, crc))
    {
        return 0;
    }

    g_cfg_flags = 0;
    cfg_set_flag(CFG_FLAG_1423);
    cfg_set_flag(CFG_FLAG_1424);
    cfg_set_flag(CFG_FLAG_1425);
    cfg_set_flag(CFG_FLAG_1426);
    /* 恢复灯效：无规则则保持 empty（全灭） */
    {
        u8 n = tpl->logic_hdr.rule_count;
        g_cfg_logic.hdr = tpl->logic_hdr;
        if(n > 0 && n <= CFG_COMBO_RULE_MAX)
        {
            memcpy(g_cfg_logic.rules, tpl->logic_rules,
                   (u16)n * (u16)CFG_COMBO_RULE_SIZE);
        }
        else
        {
            g_cfg_logic.hdr.rule_count = 0;
            memset(g_cfg_logic.rules, 0, sizeof(g_cfg_logic.rules));
            memset(g_cfg_logic.matrix, 0, sizeof(g_cfg_logic.matrix));
        }
        if(g_cfg_logic.hdr.release_filter_ms == 0)
        {
            g_cfg_logic.hdr.release_filter_ms = 20;
        }
        if(g_cfg_logic.hdr.flash_fast_ms == 0)
        {
            g_cfg_logic.hdr.flash_fast_ms = 250;
        }
        if(g_cfg_logic.hdr.flash_slow_ms == 0)
        {
            g_cfg_logic.hdr.flash_slow_ms = 500;
        }
        g_cfg_logic.ready = 1;
    }
    /* 恢复 142F */
    g_cfg_upload_flags = tpl->upload_flags;
    g_cfg_xout_n = 0;
    if(tpl->xout_n > 0 && tpl->xout_n <= CFG_XOUT_MAX)
    {
        u8 i;
        g_cfg_xout_n = tpl->xout_n;
        for(i = 0; i < g_cfg_xout_n; i++)
        {
            const u8 *r = &tpl->xout_raw[i * 4];
            g_cfg_xout[i].station = r[0];
            g_cfg_xout[i].pin = r[1];
            g_cfg_xout[i].src_ch = r[2];
            g_cfg_xout[i].flags = r[3];
        }
    }
    cfg_runtime_hot_apply();
    s_runtime_dirty = 0; /* 上电/手动 apply：Flash 与运行表一致 */
    return 1;
}

static void cfg_store_tpl_recalc_crc(void)
{
    s_wr->tpl.crc = cfg_store_crc_buf((const u8 *)&s_wr->tpl + 6u, (u16)(sizeof(s_wr->tpl) - 6u));
}

static void cfg_store_exc_recalc_crc(void)
{
    s_wr->exc.crc = cfg_store_crc_buf((const u8 *)&s_wr->exc + 4u, (u16)(sizeof(s_wr->exc) - 4u));
}

static u8 cfg_store_commit_inactive(void)
{
    u32 tpl_addr;
    u32 exc_addr;

    cfg_store_inactive_addrs(&tpl_addr, &exc_addr);
    return cfg_store_commit_pair(tpl_addr, exc_addr);
}

/* 用当前运行表填充 s_wr->tpl（不含 store_seq / crc） */
static u8 cfg_store_build_tpl(u8 sta, const u8 *blob, u8 blen)
{
    u8 i;

    if(!cfg_store_wr_ensure())
    {
        return 0;
    }
    memset(&s_wr->tpl, 0, sizeof(s_wr->tpl));
    s_wr->tpl.magic = CFG_STORE_MAGIC_TPL;
    s_wr->tpl.ver = g_cfg_ver;
    memcpy(&s_wr->tpl.init, &g_cfg_init, sizeof(s_wr->tpl.init));
    s_wr->tpl.ip_start = ip_offest_start;
    s_wr->tpl.ip_end = ip_offest_end;
    s_wr->tpl.time = g_cfg_time;
    s_wr->tpl.tpl_station = sta;
    s_wr->tpl.blob_len = blen;
    memcpy(s_wr->tpl.blob, blob, blen);
    s_wr->tpl.logic_hdr = g_cfg_logic.hdr;
    if(g_cfg_logic.hdr.rule_count > 0
       && g_cfg_logic.hdr.rule_count <= CFG_COMBO_RULE_MAX)
    {
        memcpy(s_wr->tpl.logic_rules, g_cfg_logic.rules,
               (u16)g_cfg_logic.hdr.rule_count * (u16)CFG_COMBO_RULE_SIZE);
    }
    s_wr->tpl.upload_flags = g_cfg_upload_flags;
    s_wr->tpl.xout_n = (g_cfg_xout_n <= CFG_XOUT_MAX) ? g_cfg_xout_n : 0;
    for(i = 0; i < s_wr->tpl.xout_n; i++)
    {
        u8 *r = &s_wr->tpl.xout_raw[i * 4];
        r[0] = g_cfg_xout[i].station;
        r[1] = g_cfg_xout[i].pin;
        r[2] = g_cfg_xout[i].src_ch;
        r[3] = g_cfg_xout[i].flags;
    }
    return 1;
}

/* 异类页变更时复制活跃模板并递增 store_seq */
static u8 cfg_store_tpl_bump_from_active(void)
{
    const cfg_store_tpl_t *active = cfg_store_tpl_ptr();

    if(!cfg_store_tpl_valid(active))
    {
        return 0;
    }
    memcpy(&s_wr->tpl, active, sizeof(s_wr->tpl));
    s_wr->tpl.store_seq = cfg_store_next_seq();
    cfg_store_tpl_recalc_crc();
    return 1;
}

u8 cfg_store_save_template(u8 tpl_station)
{
    u8 blob[CFG_STORE_BOARD_BLOB_MAX];
    u8 blen;
    u8 sta = tpl_station;

    if(!cfg_host_ready() || g_cfg_port_num == 0)
    {
        g_cfg_store_autosave_fail = CFG_FAIL_SAVE_NOT_READY;
        return 0;
    }
    if(sta < 2)
    {
        sta = (u8)unload_port_assign[0].station;
    }
    if(!cfg_store_pack_station(sta, blob, &blen))
    {
        g_cfg_store_autosave_fail = CFG_FAIL_SAVE_PACK;
        return 0;
    }
    if(!cfg_store_build_tpl(sta, blob, blen))
    {
        g_cfg_store_autosave_fail = CFG_FAIL_SAVE_HEAP;
        return 0;
    }
    s_wr->tpl.store_seq = cfg_store_next_seq();
    cfg_store_tpl_recalc_crc();
    /* 手动 SAVE_TPL：以当前运行为准重建样板，同时清空异类页。
     * 异类由后续 SAVE_EXCEPT / autosave 再写入，避免旧 EXC 残留。 */
    memset(&s_wr->exc, 0, sizeof(s_wr->exc));
    s_wr->exc.magic = CFG_STORE_MAGIC_EXC;
    cfg_store_exc_recalc_crc();
    if(!cfg_store_commit_inactive())
    {
        if(g_cfg_store_autosave_fail == 0)
        {
            g_cfg_store_autosave_fail = CFG_FAIL_AUTOSAVE;
        }
        return 0;
    }
    g_cfg_store_autosave_fail = 0;
    return 1;
}

static u8 cfg_store_exc_wr_stage(u8 station, const u8 *blob, u8 blen)
{
    u8 i;
    u8 free_i = 0xFF;
    u8 hit = 0xFF;

    if(!cfg_store_wr_ensure())
    {
        return 0;
    }
    if(station < 2 || blob == NULL || blen == 0 || blen > CFG_STORE_BOARD_BLOB_MAX)
    {
        return 0;
    }

    for(i = 0; i < CFG_STORE_EXCEPT_MAX; i++)
    {
        if(s_wr->exc.slot[i].used && s_wr->exc.slot[i].station == station)
        {
            hit = i;
            break;
        }
        if(!s_wr->exc.slot[i].used && free_i == 0xFF)
        {
            free_i = i;
        }
    }
    if(hit == 0xFF)
    {
        if(free_i == 0xFF)
        {
            return 0;
        }
        hit = free_i;
        s_wr->exc.count++;
    }

    s_wr->exc.slot[hit].used = 1;
    s_wr->exc.slot[hit].station = station;
    s_wr->exc.slot[hit].blob_len = blen;
    s_wr->exc.slot[hit].pad = 0;
    memcpy(s_wr->exc.slot[hit].blob, blob, blen);
    s_wr->exc.magic = CFG_STORE_MAGIC_EXC;
    return 1;
}

u8 cfg_store_save_except(u8 station)
{
    u8 blob[CFG_STORE_BOARD_BLOB_MAX];
    u8 tpl_blob[CFG_STORE_BOARD_BLOB_MAX];
    u8 blen;
    u8 tpl_blen;
    u8 tpl_sta;

    if(station < 2 || !cfg_host_ready())
    {
        return 0;
    }
    if(!cfg_store_pack_station(station, blob, &blen))
    {
        return 0;
    }
    if(cfg_store_tpl_valid(cfg_store_tpl_ptr()))
    {
        if(!cfg_store_tpl_bump_from_active())
        {
            return 0;
        }
        cfg_store_exc_load_active();
    }
    else
    {
        tpl_sta = (u8)unload_port_assign[0].station;
        if(tpl_sta < 2 || !cfg_store_pack_station(tpl_sta, tpl_blob, &tpl_blen))
        {
            return 0;
        }
        if(!cfg_store_build_tpl(tpl_sta, tpl_blob, tpl_blen))
        {
            return 0;
        }
        s_wr->tpl.store_seq = cfg_store_next_seq();
        cfg_store_tpl_recalc_crc();
        memset(&s_wr->exc, 0, sizeof(s_wr->exc));
        s_wr->exc.magic = CFG_STORE_MAGIC_EXC;
    }
    if(!cfg_store_exc_wr_stage(station, blob, blen))
    {
        return 0;
    }
    cfg_store_exc_recalc_crc();
    return cfg_store_commit_inactive();
}

u8 cfg_store_clear_except(u8 station)
{
    u8 i;
    u8 n;

    /* 尚无有效样板时：异类页本就不存在，视为清除成功并应答 942D
     * （上位机常先发 CLEAR 再 SAVE_TPL；旧逻辑直接 return 0 → 无应答，联调误报超时） */
    if(!cfg_store_tpl_bump_from_active())
    {
        return 1;
    }
    cfg_store_exc_load_active();

    if(station == 0xFF)
    {
        memset(&s_wr->exc, 0, sizeof(s_wr->exc));
        s_wr->exc.magic = CFG_STORE_MAGIC_EXC;
    }
    else
    {
        for(i = 0; i < CFG_STORE_EXCEPT_MAX; i++)
        {
            if(s_wr->exc.slot[i].used && s_wr->exc.slot[i].station == station)
            {
                memset(&s_wr->exc.slot[i], 0, sizeof(s_wr->exc.slot[i]));
            }
        }
        n = 0;
        for(i = 0; i < CFG_STORE_EXCEPT_MAX; i++)
        {
            if(s_wr->exc.slot[i].used)
            {
                n++;
            }
        }
        s_wr->exc.count = n;
    }
    cfg_store_exc_recalc_crc();
    return cfg_store_commit_inactive();
}

u8 cfg_store_clear_template(void)
{
    u32 banks[2][2] = {
        { CFG_FLASH_BANK0_TPL_ADDR, CFG_FLASH_BANK0_EXC_ADDR },
        { CFG_FLASH_BANK1_TPL_ADDR, CFG_FLASH_BANK1_EXC_ADDR }
    };
    u8 i;

    if(!cfg_store_wr_ensure())
    {
        return 0;
    }
    memset(&s_wr->tpl, 0, sizeof(s_wr->tpl));
    memset(&s_wr->exc, 0, sizeof(s_wr->exc));
    for(i = 0; i < 2; i++)
    {
        if(!cfg_flash_program(banks[i][0], &s_wr->tpl, (u16)sizeof(s_wr->tpl)))
        {
            return 0;
        }
        if(!cfg_flash_program(banks[i][1], &s_wr->exc, (u16)sizeof(s_wr->exc)))
        {
            return 0;
        }
    }
    cfg_store_refresh_active();
    return 1;
}

void cfg_store_boot_try_apply(void)
{
    /* 调度未起：仅选 Bank，真正 apply 在 CanIo 首圈（栈/堆/CAN 均已就绪） */
    cfg_store_refresh_active();
    s_boot_apply_pending = cfg_store_tpl_valid(cfg_store_tpl_ptr()) ? 1u : 0u;
}

void cfg_store_boot_finish(void)
{
    if(s_boot_apply_pending == 0)
    {
        return;
    }
    s_boot_apply_pending = 0;
    (void)cfg_store_apply();
}

/*
 * 0x142D payload:
 *   op u8
 *   arg u8  (SAVE_TPL/SAVE_EXCEPT/CLEAR_EXCEPT 的站号；CLEAR_EXCEPT 用 0xFF 清全部)
 */
u8 cfg_store_handle_cmd(const u8 *payload, u16 payload_len)
{
    u8 op;
    u8 arg;
    u8 ok = 0;

    if(payload == NULL || payload_len < 1)
    {
        return 0;
    }
    op = payload[0];
    arg = (payload_len >= 2) ? payload[1] : 0;

    switch(op)
    {
    case CFG_STORE_OP_SAVE_TPL:
        ok = cfg_store_save_template(arg);
        break;
    case CFG_STORE_OP_SAVE_EXCEPT:
        ok = cfg_store_save_except(arg);
        break;
    case CFG_STORE_OP_CLEAR_EXCEPT:
        ok = cfg_store_clear_except(arg);
        break;
    case CFG_STORE_OP_APPLY:
        ok = cfg_store_apply();
        if(ok)
        {
            s_runtime_dirty = 0; /* Flash 已是权威源 */
        }
        return ok;
    case CFG_STORE_OP_CLEAR_TPL:
        ok = cfg_store_clear_template();
        break;
    default:
        return 0;
    }

    if(ok && (op == CFG_STORE_OP_SAVE_TPL || op == CFG_STORE_OP_SAVE_EXCEPT
              || op == CFG_STORE_OP_CLEAR_EXCEPT || op == CFG_STORE_OP_CLEAR_TPL))
    {
        s_runtime_dirty = 0;
    }
    return ok;
}

void cfg_store_mark_runtime_dirty(void)
{
    s_runtime_dirty = 1;
}

/*
 * PIN 全站下发成功后调用：把当前运行表写入模板(+异类站)。
 * 这样 0x1426 重绑后即使未再发 0x142D，掉电也能恢复。
 */
u8 g_cfg_store_autosave_fail = 0;

u8 cfg_store_try_autosave(void)
{
    const cfg_store_tpl_t *old;
    u8 sn, i;
    u8 tpl_sta;
    u8 blen;
    u16 p;
    u8 except_n = 0;
    u8 except_fail = 0;

    if(!s_runtime_dirty)
    {
        return 1;
    }
    if(!cfg_host_ready() || g_cfg_port_num == 0)
    {
        g_cfg_store_autosave_fail = CFG_FAIL_SAVE_NOT_READY;
        return 0;
    }

    old = cfg_store_tpl_ptr();
    if(cfg_store_tpl_valid(old) && old->tpl_station >= 2)
    {
        tpl_sta = old->tpl_station;
    }
    else
    {
        tpl_sta = (u8)unload_port_assign[0].station;
    }
    if(tpl_sta < 2)
    {
        g_cfg_store_autosave_fail = CFG_FAIL_SAVE_NO_STA;
        return 0;
    }

    if(!cfg_store_pack_station(tpl_sta, s_autosave_blob, &blen))
    {
        g_cfg_store_autosave_fail = CFG_FAIL_SAVE_PACK;
        return 0;
    }
    if(!cfg_store_build_tpl(tpl_sta, s_autosave_blob, blen))
    {
        g_cfg_store_autosave_fail = CFG_FAIL_SAVE_HEAP;
        return 0;
    }
    s_wr->tpl.store_seq = cfg_store_next_seq();
    cfg_store_tpl_recalc_crc();

    memset(&s_wr->exc, 0, sizeof(s_wr->exc));
    s_wr->exc.magic = CFG_STORE_MAGIC_EXC;

    sn = cfg_store_collect_stations(&g_cfg_init, s_autosave_stations, CFG_MAX_BOARD);
    for(i = 0; i < sn; i++)
    {
        u8 blen;

        if(s_autosave_stations[i] == tpl_sta)
        {
            continue;
        }
        for(p = 0; p < g_cfg_port_num; p++)
        {
            if((u8)unload_port_assign[p].station == s_autosave_stations[i])
            {
                if(except_n >= CFG_STORE_EXCEPT_MAX)
                {
                    except_fail = 1;
                    break;
                }
                if(!cfg_store_pack_station(s_autosave_stations[i], s_autosave_blob, &blen)
                   || !cfg_store_exc_wr_stage(s_autosave_stations[i], s_autosave_blob, blen))
                {
                    except_fail = 1;
                }
                else
                {
                    except_n++;
                }
                break;
            }
        }
        if(except_fail && except_n >= CFG_STORE_EXCEPT_MAX)
        {
            break;
        }
    }

    if(except_fail)
    {
        g_cfg_store_autosave_fail = CFG_FAIL_SAVE_EXCEPT;
        return 0;
    }
    cfg_store_exc_recalc_crc();
    if(!cfg_store_commit_inactive())
    {
        if(g_cfg_store_autosave_fail == 0)
        {
            g_cfg_store_autosave_fail = CFG_FAIL_AUTOSAVE;
        }
        return 0;
    }
    s_runtime_dirty = 0;
    g_cfg_store_autosave_fail = 0;
    return 1;
}
