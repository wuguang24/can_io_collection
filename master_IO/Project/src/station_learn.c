#include "station_learn.h"
#include "cfg_table.h"
#include "cfg_can_ext.h"
#include "can_bus.h"
#include "string.h"

extern void AddSendMsgToQueue(u16 msg);

#define STN_LRN_WAIT_MS         (400u)
#define STN_LRN_DISCOVER_MS     (80u)
#define STN_LRN_REPORT_MAX      CFG_MAX_BOARD

typedef enum
{
    STN_LRN_IDLE = 0,
    STN_LRN_WAIT_DONE,
    STN_LRN_DONE
} stn_lrn_state_t;

static stn_lrn_state_t s_state;
static u8  s_idx;
static u8  s_total;
static u8  s_token;
static u16 s_wait_ms;
static u8  s_pending_sta;
static u8  s_distribute_armed;
static u8  s_discover_sent;
static u8  s_rpt_sta[STN_LRN_REPORT_MAX];
static u8  s_rpt_rslt[STN_LRN_REPORT_MAX];
static u8  s_rpt_n;

static void stn_lrn_send_discover(u8 token)
{
    u8 buff[4];

    buff[0] = STN_LRN_SUB_DISCOVER;
    buff[1] = token;
    buff[2] = 0;
    buff[3] = 0;
    vcan_send_send_msg(buff, 4, CAN_FUNC_ID_STATION_LEARN, 0x7F);
}

static void stn_lrn_send_assign(u8 station, u8 token)
{
    u8 buff[4];

    buff[0] = STN_LRN_SUB_ASSIGN;
    buff[1] = station;
    buff[2] = token;
    buff[3] = 0;
    vcan_send_send_msg(buff, 4, CAN_FUNC_ID_STATION_LEARN, 0x7F);
}

static void stn_lrn_send_pass(u8 station, u8 token)
{
    u8 buff[4];

    buff[0] = STN_LRN_SUB_PASS;
    buff[1] = token;
    buff[2] = 0;
    buff[3] = 0;
    vcan_send_send_msg(buff, 4, CAN_FUNC_ID_STATION_LEARN, station);
}

static void stn_lrn_push_result(u8 station, u8 result)
{
    if(s_rpt_n >= STN_LRN_REPORT_MAX)
    {
        return;
    }
    s_rpt_sta[s_rpt_n] = station;
    s_rpt_rslt[s_rpt_n] = result;
    s_rpt_n++;
}

static void stn_lrn_try_finish(void)
{
    if(s_idx >= s_total)
    {
        s_state = STN_LRN_DONE;
        AddSendMsgToQueue(STN_LRN_MSG_DONE);
        if(s_distribute_armed && cfg_host_ready() && !cfg_can_is_busy())
        {
            s_distribute_armed = 0;
            cfg_can_start_distribute();
        }
    }
}

static void stn_lrn_begin_slot(void)
{
    if(s_idx >= s_total)
    {
        stn_lrn_try_finish();
        return;
    }
    s_pending_sta = cfg_table_topo_station_at(s_idx);
    if(s_pending_sta < 2)
    {
        s_idx++;
        stn_lrn_begin_slot();
        return;
    }
    s_token++;
    s_discover_sent = 0;
    stn_lrn_send_assign(s_pending_sta, s_token);
    s_state = STN_LRN_WAIT_DONE;
    s_wait_ms = 0;
}

void cfg_station_learn_init(void)
{
    s_state = STN_LRN_IDLE;
    s_idx = 0;
    s_total = 0;
    s_token = 0;
    s_wait_ms = 0;
    s_pending_sta = 0;
    s_distribute_armed = 0;
    s_discover_sent = 0;
    s_rpt_n = 0;
}

void cfg_station_learn_start(void)
{
    s_total = cfg_table_topo_station_count();
    s_idx = 0;
    s_wait_ms = 0;
    s_pending_sta = 0;
    s_distribute_armed = 1;
    s_discover_sent = 0;
    s_rpt_n = 0;

    if(s_total == 0)
    {
        s_state = STN_LRN_DONE;
        stn_lrn_try_finish();
        return;
    }
    stn_lrn_begin_slot();
}

u8 cfg_station_learn_done(void)
{
    return (u8)(s_state == STN_LRN_DONE);
}

u8 cfg_station_learn_active(void)
{
    return (u8)(s_state != STN_LRN_IDLE && s_state != STN_LRN_DONE);
}

void cfg_station_learn_fill_report(u8 *summary, u8 *stations, u8 *results,
                                   u8 max_n, u8 *out_n)
{
    u8 i;
    u8 n;
    u8 sum = 0;

    if(summary != NULL)
    {
        *summary = 0;
    }
    if(out_n != NULL)
    {
        *out_n = 0;
    }
    if(stations == NULL || results == NULL || out_n == NULL)
    {
        return;
    }
    n = s_rpt_n;
    if(n > max_n)
    {
        n = max_n;
    }
    for(i = 0; i < n; i++)
    {
        stations[i] = s_rpt_sta[i];
        results[i] = s_rpt_rslt[i];
        if(results[i] != STN_LRN_RSLT_OK)
        {
            sum = 1;
        }
    }
    *out_n = n;
    if(summary != NULL)
    {
        *summary = sum;
    }
}

void cfg_station_learn_on_rx(u8 src_id, const u8 *data, u8 len)
{
    if(data == NULL || len < 2 || s_state != STN_LRN_WAIT_DONE)
    {
        return;
    }
    if(data[0] == STN_LRN_SUB_ACK)
    {
        if(data[1] != s_token || !s_discover_sent)
        {
            return;
        }
        stn_lrn_send_assign(s_pending_sta, s_token);
        return;
    }
    if(data[0] != STN_LRN_SUB_DONE)
    {
        return;
    }
    if(data[1] != s_token || src_id != s_pending_sta)
    {
        return;
    }
    stn_lrn_send_pass(src_id, s_token);
    stn_lrn_push_result(s_pending_sta, STN_LRN_RSLT_OK);
    s_idx++;
    s_wait_ms = 0;
    if(s_idx >= s_total)
    {
        stn_lrn_try_finish();
        return;
    }
    stn_lrn_begin_slot();
}

void cfg_station_learn_poll(void)
{
    if(s_state != STN_LRN_WAIT_DONE)
    {
        return;
    }
    if(s_wait_ms < 0xFFFFu)
    {
        s_wait_ms++;
    }
    if(!s_discover_sent && s_wait_ms >= STN_LRN_DISCOVER_MS)
    {
        s_discover_sent = 1;
        stn_lrn_send_discover(s_token);
    }
    if(s_wait_ms < STN_LRN_WAIT_MS)
    {
        return;
    }
    s_wait_ms = 0;
    stn_lrn_push_result(s_pending_sta, STN_LRN_RSLT_TIMEOUT);
    s_idx++;
    if(s_idx >= s_total)
    {
        stn_lrn_try_finish();
        return;
    }
    stn_lrn_begin_slot();
}
