/*
 * F107：配置下发 + CFG_ACK 闭环
 * - FUNC：cfg_can_start_distribute 内按拓扑逐站下发（格口板 len=2）
 * - TIME：5×u16 + CRC16 广播
 * - PIN：按站切片；等 ACK；超时重试；失败进列表供 0x142A
 * - 重绑：再次 0x1426 成功后 hot_apply → 全拓扑重发 PIN（从机覆盖 Flash）
 * - 无配置：上电/周期 ACK 或 PORT_STATE Data[6]=1 → 142A；并对「应有 PIN」的站自动补发
 */
#include "cfg_can_ext.h"
#include "cfg_table.h"
#include "cfg_crc.h"
#include "cfg_store.h"
#include "can_bus.h"
#include "fun.h"
#include "station_learn.h"
#include "string.h"

#define CFG_PIN_ACK_WAIT_MS     (800u)
#define CFG_PIN_RETRY_MAX       (3u)
#define CFG_PIN_RESEND_CD_10MS  (200u) /* 无 PIN 自动补发冷却 ≈2s（10ms 节拍） */

/* 按 sta_idx=station-2 跟踪 PIN 下发 */
static u8 s_in_topo[MAX_SLAVE_NUM];      /* 出现在 1423 */
static u8 s_expect_empty[MAX_SLAVE_NUM]; /* 本站 1426 无格口，允许空 PIN */
static u8 s_pin_pending[MAX_SLAVE_NUM];  /* 等待 ACK */
static u8 s_pin_retry[MAX_SLAVE_NUM];
static u8 s_pin_fail[MAX_SLAVE_NUM];     /* 0=无失败；非0=CFG_FAIL_* */
static u8 s_pin_ok[MAX_SLAVE_NUM];
static u8 s_resend_cd[MAX_SLAVE_NUM];    /* 无 PIN 补发冷却（10ms 单位） */
static u8 s_resend_div;                  /* 1ms→10ms 分频 */
static u8 s_busy;
static u16 s_wait_ms;
static u8 s_need_report;                 /* 失败集变化，促发 0x142A */

/* 通用线：每站独立格口板 FUNC=[station|g2, station] */
static void send_func_one_station(u8 station)
{
    u8 buff[2];
    u8 g, s;

    buff[0] = station;
    buff[1] = station;
    for(g = 0; g < g_cfg_init.group_total && g < CFG_MAX_GROUP; g++)
    {
        for(s = 0; s < g_cfg_init.group[g].each_total
                && s < CFG_MAX_STATION_PER_GROUP; s++)
        {
            if(g_cfg_init.group[g].each[s].station == station)
            {
                if(g_cfg_init.group[g].each[s].block_group == CFG_BLOCK_GROUP_TWO)
                {
                    buff[0] |= 0x80u;
                }
                goto send_func;
            }
        }
    }
send_func:
    vcan_send_send_msg(buff, 2, CAN_FUNC_ID_INIT_FUNC_DATA, station);
}

static void send_func_all_topo(void)
{
    u8 g, s, station;

    for(g = 0; g < g_cfg_init.group_total && g < CFG_MAX_GROUP; g++)
    {
        for(s = 0; s < g_cfg_init.group[g].each_total
                && s < CFG_MAX_STATION_PER_GROUP; s++)
        {
            station = g_cfg_init.group[g].each[s].station;
            if(station >= 2)
            {
                send_func_one_station(station);
            }
        }
    }
}

static void send_time_broadcast(void)
{
    u8 buff[12];
    u16 crc;

    memcpy(buff, &g_cfg_time, 10);
    crc = cfg_crc16(buff, 10);
    buff[10] = (u8)(crc & 0xFF);
    buff[11] = (u8)(crc >> 8);
    vcan_send_send_msg(buff, 12, CAN_FUNC_ID_INIT_TIME_DATA, 0x7F);
}

static void send_pin_one_station(u8 station)
{
    u8 buff[CFG_PIN_SLICE_HDR_EXT + 8 * CFG_PIN_REC_SIZE + 2];
    u16 len;
    u16 crc;
    u8 idx;
    u8 n;

    if(station < 2 || (station - 2) >= MAX_SLAVE_NUM)
    {
        return;
    }
    idx = (u8)(station - 2);
    cfg_table_build_station_slice(station, buff, &len, &crc);
    n = (len >= 5) ? buff[2] : 0;
    s_expect_empty[idx] = (n == 0) ? 1 : 0;
    if(len >= 5)
    {
        vcan_send_send_msg(buff, len, CAN_FUNC_ID_INIT_PIN_DATA, station);
    }
    (void)crc;
}

static void rebuild_topo_mask(void)
{
    u8 i, n, station;

    memset(s_in_topo, 0, sizeof(s_in_topo));
    n = cfg_table_topo_station_count();
    for(i = 0; i < n; i++)
    {
        station = cfg_table_topo_station_at(i);
        if(station >= 2 && (station - 2) < MAX_SLAVE_NUM)
        {
            s_in_topo[station - 2] = 1;
        }
    }
}

/* 当前运行表里该站是否有格口（应下发非空 PIN） */
static u8 station_has_ports(u8 station)
{
    u16 i;

    for(i = 0; i < g_cfg_port_num; i++)
    {
        if((u8)unload_port_assign[i].station == station)
        {
            return 1;
        }
    }
    return 0;
}

/*
 * 发现从机无 PIN 且主机侧该站应有绑表时：单站补发 PIN（不打断整网 busy 全量分发）。
 * 用于：从机后上电、Flash 丢失、重绑后个别站未跟上。
 */
static void kick_pin_resend_if_needed(u8 idx)
{
    u8 station;

    if(idx >= MAX_SLAVE_NUM || !cfg_host_ready() || s_busy)
    {
        return;
    }
    if(!s_in_topo[idx] || s_pin_pending[idx])
    {
        return;
    }
    station = (u8)(idx + 2);
    if(!station_has_ports(station))
    {
        s_expect_empty[idx] = 1;
        return;
    }
    if(s_resend_cd[idx] != 0)
    {
        return;
    }

    s_expect_empty[idx] = 0;
    s_pin_pending[idx] = 1;
    s_pin_retry[idx] = 0;
    s_pin_ok[idx] = 0;
    send_pin_one_station(station);
    s_busy = 1;
    s_wait_ms = 0;
    s_resend_cd[idx] = CFG_PIN_RESEND_CD_10MS;
}

static u8 map_ack_result(u8 result)
{
    if(result == 0)
    {
        return 0;
    }
    if(result == 1)
    {
        return CFG_FAIL_NO_PIN;
    }
    if(result == 2)
    {
        return CFG_FAIL_CRC;
    }
    if(result == 3)
    {
        return CFG_FAIL_FLASH;
    }
    if(result == 5)
    {
        return CFG_FAIL_PARAM;
    }
    return CFG_FAIL_PARAM;
}

u8 cfg_can_check_station_consistency(void)
{
    u16 i;
    u8 station;
    u8 orphan_n = 0;
    u8 idx;

    rebuild_topo_mask();

    for(i = 0; i < g_cfg_port_num; i++)
    {
        station = (u8)unload_port_assign[i].station;
        if(station < 2 || (station - 2) >= MAX_SLAVE_NUM)
        {
            continue;
        }
        idx = (u8)(station - 2);
        if(!s_in_topo[idx])
        {
            if(s_pin_fail[idx] != CFG_FAIL_ORPHAN)
            {
                s_pin_fail[idx] = CFG_FAIL_ORPHAN;
                s_pin_ok[idx] = 0;
                s_need_report = 1;
            }
            orphan_n++;
        }
    }
    return orphan_n;
}

void cfg_can_ext_init(void)
{
    memset(s_in_topo, 0, sizeof(s_in_topo));
    memset(s_expect_empty, 0, sizeof(s_expect_empty));
    memset(s_pin_pending, 0, sizeof(s_pin_pending));
    memset(s_pin_retry, 0, sizeof(s_pin_retry));
    memset(s_pin_fail, 0, sizeof(s_pin_fail));
    memset(s_pin_ok, 0, sizeof(s_pin_ok));
    memset(s_resend_cd, 0, sizeof(s_resend_cd));
    s_resend_div = 0;
    s_busy = 0;
    s_wait_ms = 0;
    s_need_report = 0;
    cfg_station_learn_init();
}

void cfg_can_abort_distribute(void)
{
    memset(s_pin_pending, 0, sizeof(s_pin_pending));
    s_busy = 0;
    s_wait_ms = 0;
}

u8 cfg_can_is_busy(void)
{
    return s_busy;
}

void cfg_can_start_distribute(void)
{
    u8 idx;

    if(!cfg_host_ready() || cfg_station_learn_active() || s_busy)
    {
        return;
    }

    rebuild_topo_mask();
    (void)cfg_can_check_station_consistency();

    /* 仅对拓扑内站清 pending/ok，保留孤儿失败标记 */
    for(idx = 0; idx < MAX_SLAVE_NUM; idx++)
    {
        if(s_in_topo[idx])
        {
            s_pin_pending[idx] = 1;
            s_pin_retry[idx] = 0;
            s_pin_ok[idx] = 0;
            if(s_pin_fail[idx] != CFG_FAIL_ORPHAN)
            {
                s_pin_fail[idx] = 0;
            }
        }
        else if(s_pin_fail[idx] != CFG_FAIL_ORPHAN)
        {
            s_pin_pending[idx] = 0;
            s_pin_ok[idx] = 0;
        }
    }

    send_func_all_topo();
    send_time_broadcast();
    for(idx = 0; idx < MAX_SLAVE_NUM; idx++)
    {
        if(s_pin_pending[idx])
        {
            send_pin_one_station((u8)(idx + 2));
        }
    }

    s_busy = 1;
    s_wait_ms = 0;
}

void cfg_can_on_cfg_ack(u8 station, u8 kind, u8 result)
{
    u8 idx;
    u8 mapped;

    if(station < 2 || (station - 2) >= MAX_SLAVE_NUM)
    {
        return;
    }
    idx = (u8)(station - 2);

    /* kind: 2=TIME  3=PIN；无 PIN 主动上报也用 kind=3 result=1 */
    if(kind != 3)
    {
        return;
    }

    mapped = map_ack_result(result);

    /*
     * 竞态：主机正在等本站 PIN ACK 时，从机上电公告(result=1)可能抢先到达。
     * 若本站应有非空 PIN，则忽略该公告，保持 pending 等超时重发，避免误结案。
     */
    if(s_pin_pending[idx] && mapped == CFG_FAIL_NO_PIN && !s_expect_empty[idx]
       && station_has_ports(station))
    {
        return;
    }

    s_pin_pending[idx] = 0;

    /* 主机下发的就是空切片：空 PIN 属预期，不当失败 */
    if(mapped == CFG_FAIL_NO_PIN && s_expect_empty[idx])
    {
        mapped = 0;
    }

    if(mapped == 0)
    {
        s_pin_ok[idx] = 1;
        if(s_pin_fail[idx] != 0 && s_pin_fail[idx] != CFG_FAIL_ORPHAN)
        {
            s_pin_fail[idx] = 0;
            s_need_report = 1;
        }
        else if(s_pin_fail[idx] != CFG_FAIL_ORPHAN)
        {
            s_pin_fail[idx] = 0;
        }
        return;
    }

    s_pin_ok[idx] = 0;
    if(s_pin_fail[idx] != mapped)
    {
        s_pin_fail[idx] = mapped;
        s_need_report = 1;
    }

    /* 无 PIN 主动公告：主机已就绪时尝试单站补发（重绑/换板后常见） */
    if(mapped == CFG_FAIL_NO_PIN)
    {
        rebuild_topo_mask();
        kick_pin_resend_if_needed(idx);
    }

    /* CRC/Flash/参数失败：若仍在本轮分发中，允许 poll 重试 */
    if(s_busy && s_in_topo[idx]
       && (mapped == CFG_FAIL_CRC || mapped == CFG_FAIL_FLASH || mapped == CFG_FAIL_PARAM))
    {
        if(s_pin_retry[idx] < CFG_PIN_RETRY_MAX)
        {
            s_pin_pending[idx] = 1;
        }
    }
}

void cfg_can_fill_ready_list(u8 *stations, u8 *flags, u8 max_n, u8 *out_n)
{
    u8 i, n, sta, idx, f;

    if(stations == NULL || flags == NULL || out_n == NULL)
    {
        return;
    }
    n = 0;
    for(i = 0; i < cfg_table_topo_station_count() && n < max_n; i++)
    {
        sta = cfg_table_topo_station_at(i);
        if(sta < 2 || (sta - 2) >= MAX_SLAVE_NUM)
        {
            continue;
        }
        idx = (u8)(sta - 2);
        f = 0;
        if(s_pin_ok[idx])
        {
            f |= 0x01u;
        }
        if(can_bus_slave_cos_alive(idx))
        {
            f |= 0x02u;
        }
        stations[n] = sta;
        flags[n] = f;
        n++;
    }
    *out_n = n;
}

u8 cfg_can_topo_all_ready(void)
{
    u8 stations[CFG_MAX_BOARD];
    u8 flags[CFG_MAX_BOARD];
    u8 n, i;

    if(!cfg_host_ready())
    {
        return 0;
    }
    cfg_can_fill_ready_list(stations, flags, CFG_MAX_BOARD, &n);
    if(n == 0)
    {
        return 0;
    }
    for(i = 0; i < n; i++)
    {
        if(flags[i] != CFG_SLAVE_READY_FULL)
        {
            return 0;
        }
    }
    return 1;
}

void cfg_can_ext_poll(void)
{
    u8 idx;
    u8 any_pending = 0;
    u8 resent = 0;

    cfg_station_learn_poll();

    if(++s_resend_div >= 10)
    {
        s_resend_div = 0;
        for(idx = 0; idx < MAX_SLAVE_NUM; idx++)
        {
            if(s_resend_cd[idx] > 0)
            {
                s_resend_cd[idx]--;
            }
        }
    }

    if(!s_busy)
    {
        if(s_need_report && cfg_host_ready())
        {
            s_need_report = 0;
            AddSendMsgToQueue(SEND_MSG_SLAVE_CFG_FAIL_TYPE);
        }
        return;
    }

    for(idx = 0; idx < MAX_SLAVE_NUM; idx++)
    {
        if(s_pin_pending[idx])
        {
            any_pending = 1;
            break;
        }
    }

    if(!any_pending)
    {
        s_busy = 0;
        s_wait_ms = 0;
        if(cfg_can_fail_count() > 0 || s_need_report)
        {
            s_need_report = 0;
            AddSendMsgToQueue(SEND_MSG_SLAVE_CFG_FAIL_TYPE);
        }
        else
        {
            /* 全站 PIN OK：把重绑后的运行表自动写入 Flash */
            if(!cfg_store_try_autosave())
            {
                AddSendMsgToQueue(SEND_MSG_SLAVE_CFG_FAIL_TYPE);
            }
            can_bus_request_sync_all();
        }
        return;
    }

    if(s_wait_ms < 0xFFFF)
    {
        s_wait_ms++;
    }
    if(s_wait_ms < CFG_PIN_ACK_WAIT_MS)
    {
        return;
    }
    s_wait_ms = 0;

    for(idx = 0; idx < MAX_SLAVE_NUM; idx++)
    {
        if(!s_pin_pending[idx])
        {
            continue;
        }
        if(s_pin_retry[idx] < CFG_PIN_RETRY_MAX)
        {
            s_pin_retry[idx]++;
            send_pin_one_station((u8)(idx + 2));
            resent = 1;
        }
        else
        {
            s_pin_pending[idx] = 0;
            s_pin_ok[idx] = 0;
            if(s_pin_fail[idx] == 0 || s_pin_fail[idx] == CFG_FAIL_NO_PIN)
            {
                s_pin_fail[idx] = CFG_FAIL_TIMEOUT;
            }
            s_need_report = 1;
        }
    }

    if(!resent)
    {
        s_busy = 0;
        if(s_need_report)
        {
            s_need_report = 0;
            AddSendMsgToQueue(SEND_MSG_SLAVE_CFG_FAIL_TYPE);
        }
        else if(cfg_can_fail_count() == 0)
        {
            if(!cfg_store_try_autosave())
            {
                AddSendMsgToQueue(SEND_MSG_SLAVE_CFG_FAIL_TYPE);
            }
        }
    }
}

u8 cfg_can_fail_count(void)
{
    u8 idx;
    u8 n = 0;

    for(idx = 0; idx < MAX_SLAVE_NUM; idx++)
    {
        if(s_pin_fail[idx] != 0)
        {
            n++;
        }
    }
    return n;
}

void cfg_can_fill_fail_list(u8 *stations, u8 *reasons, u8 max_n, u8 *out_n)
{
    u8 idx;
    u8 n = 0;

    if(out_n)
    {
        *out_n = 0;
    }
    if(stations == NULL || reasons == NULL || max_n == 0)
    {
        return;
    }

    for(idx = 0; idx < MAX_SLAVE_NUM && n < max_n; idx++)
    {
        if(s_pin_fail[idx] == 0)
        {
            continue;
        }
        stations[n] = (u8)(idx + 2);
        reasons[n] = s_pin_fail[idx];
        n++;
    }
    /* 主机 Flash 保存失败：station=0, reason=8~15（见 CFG_FAIL_SAVE_*） */
    if(g_cfg_store_autosave_fail != 0 && n < max_n)
    {
        stations[n] = 0;
        reasons[n] = g_cfg_store_autosave_fail;
        n++;
    }
    if(out_n)
    {
        *out_n = n;
    }
}
