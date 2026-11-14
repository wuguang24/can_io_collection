#include "port_cos.h"
#include "can_bus.h"
#include "main.h"
#include "io.h"
#include "io_ctrl.h"
#include "pin_cfg.h"
#include "string.h"

extern u8 g_port_remote_lock;

extern u32 port_enable_state;
extern u32 port_print_state;
extern u32 slot_full_state;
extern u32 pack_full_state;
extern u32 full_tier1_state;
extern u32 full_tier2_state;
extern u32 port_door_state;
extern u16 port_block_state;
extern u8  upstream_can;

static u8  s_seq;
static u8  s_confirmed[8];
static u8  s_pending[8];
static u8  s_trig_sent;
static u8  s_wait_conf;
static u16 s_stable_ms;
static u16 s_early_cnt;
static u8  s_early[8];

static void cos_send(u8 func_id, u8 seq, const u8 *body)
{
    sCanFrameExt canTxMsg;
    u8 i;

    canTxMsg.extId.func_id = func_id;
    canTxMsg.extId.src_id = Local_Station;
    canTxMsg.extId.dst_id = 1;
    canTxMsg.extId.seg_num = seq;
    canTxMsg.extId.seg_polo = CAN_SEG_POLO_NONE;
    canTxMsg.data_len = 8;
    for(i = 0; i < 8; i++)
    {
        canTxMsg.data[i] = body[i];
    }

    if(upstream_can == 1)
    {
        vcanbus_addto_cansendqueue_one(canTxMsg);
    }
    else if(upstream_can == 2)
    {
        vcanbus_addto_cansendqueue_two(canTxMsg);
    }
    else
    {
        vcanbus_addto_cansendqueue_one(canTxMsg);
        vcanbus_addto_cansendqueue_two(canTxMsg);
    }
}

void port_cos_pack(u8 seq, u8 *out8)
{
    if(out8 == NULL)
    {
        return;
    }
    out8[0] = slot_full_state & 0xFFu;
    out8[1] = pack_full_state & 0xFFu;
    out8[2] = (u8)(port_block_state & 0xFFu);
    out8[3] = port_door_state & 0xFFu;
    out8[4] = port_enable_state & 0xFFu;
    out8[5] = port_print_state & 0xFFu;
    out8[6] = full_tier1_state & 0xFFu;
    out8[7] = full_tier2_state & 0xFFu;
    (void)seq;
}

static u8 cos_body_equal(const u8 *a, const u8 *b)
{
    u8 i;
    for(i = 0; i < 8; i++)
    {
        if(a[i] != b[i])
        {
            return 0;
        }
    }
    return 1;
}

static void cos_build_early(u8 *out8)
{
    u8 in = io_ctrl_in;
    u8 i;
    u32 en = 0, full = 0, half = 0, door = 0, print = 0, tier1 = 0, tier2 = 0;
    u16 block = 0;

    if(!g_pin_ready || g_pin_n == 0)
    {
        memset(out8, 0, 8);
        return;
    }

    for(i = 0; i < g_pin_n && i < PORT_NUM; i++)
    {
        u8 pin;
        if(g_pin_rec[i].in_lock < 8 && ((in >> g_pin_rec[i].in_lock) & 1u))
        {
            en |= (1u << i);
        }
        if(g_pin_rec[i].in_full_pack < 8 && ((in >> g_pin_rec[i].in_full_pack) & 1u))
        {
            full |= (1u << i);
        }
        pin = g_pin_rec[i].in_half_full;
        if(pin < 8 && ((in >> pin) & 1u))
        {
            half |= (1u << i);
        }
        pin = g_pin_rec[i].in_clamp_pack;
        if(pin < 8)
        {
            block |= (u16)( ((in >> pin) & 1u) << i );
        }
        pin = g_pin_rec[i].in_unload_check;
        if(pin < 8 && ((in >> pin) & 1u))
        {
            door |= (1u << i);
        }
        if(g_pin_rec[i].in_print < 8 && ((in >> g_pin_rec[i].in_print) & 1u))
        {
            print |= (1u << i);
        }
        pin = g_pin_rec[i].in_full_tier1;
        if(pin < 8 && ((in >> pin) & 1u))
        {
            tier1 |= (1u << i);
        }
        pin = g_pin_rec[i].in_full_tier2;
        if(pin < 8 && ((in >> pin) & 1u))
        {
            tier2 |= (1u << i);
        }
    }
    en |= (u32)g_port_remote_lock;
    out8[0] = (u8)(half & 0xFFu);
    out8[1] = (u8)(full & 0xFFu);
    out8[2] = (u8)(block & 0xFFu);
    out8[3] = (u8)(door & 0xFFu);
    out8[4] = (u8)(en & 0xFFu);
    out8[5] = (u8)(print & 0xFFu);
    out8[6] = (u8)(tier1 & 0xFFu);
    out8[7] = (u8)(tier2 & 0xFFu);
}

void port_cos_send_confirm(u8 seq)
{
    u8 body[8];
    port_cos_pack(seq, body);
    cos_send(CAN_FUNC_ID_PORT_COS_CONF, seq, body);
    memcpy(s_confirmed, body, 8);
    s_wait_conf = 0;
    s_trig_sent = 0;
    s_stable_ms = 0;
}

void port_cos_reset(void)
{
    memset(s_confirmed, 0, sizeof(s_confirmed));
    memset(s_pending, 0, sizeof(s_pending));
    memset(s_early, 0, sizeof(s_early));
    s_seq = 0;
    s_trig_sent = 0;
    s_wait_conf = 0;
    s_stable_ms = 0;
    s_early_cnt = 0;
}

void port_cos_force_sync_reply(void)
{
    if(s_wait_conf)
    {
        port_cos_send_confirm(s_seq);
    }
    else
    {
        u8 body[8];
        port_cos_pack(s_seq, body);
        if(!cos_body_equal(body, s_confirmed))
        {
            s_seq++;
            port_cos_send_confirm(s_seq);
        }
        else
        {
            port_cos_send_confirm(s_seq);
        }
    }
}

void port_cos_poll(void)
{
    u8 body[8];
    u8 early[8];

    if(!g_pin_ready || g_pin_n == 0)
    {
        return;
    }

    port_cos_pack(0, body);
    cos_build_early(early);

    if(!cos_body_equal(early, s_early))
    {
        memcpy(s_early, early, 8);
        s_early_cnt = 0;
    }
    else if(s_early_cnt < 0xFFFEu)
    {
        s_early_cnt++;
    }

    if(cos_body_equal(body, s_confirmed))
    {
        s_wait_conf = 0;
        s_trig_sent = 0;
        s_stable_ms = 0;
        return;
    }

    if(!s_trig_sent
       && !cos_body_equal(early, s_confirmed)
       && s_early_cnt >= PORT_COS_TRIG_DEBOUNCE_MS)
    {
        s_seq++;
        cos_send(CAN_FUNC_ID_PORT_COS_TRIG, s_seq, early);
        memcpy(s_pending, body, 8);
        s_trig_sent = 1;
        s_wait_conf = 1;
        s_stable_ms = 0;
    }

    if(s_wait_conf)
    {
        if(!cos_body_equal(body, s_pending))
        {
            memcpy(s_pending, body, 8);
            s_stable_ms = 0;
        }
        else if(s_stable_ms < 0xFFFEu)
        {
            s_stable_ms++;
        }

        if(s_stable_ms >= PORT_COS_CONFIRM_HOLD_MS)
        {
            port_cos_send_confirm(s_seq);
        }
    }
    else if(!s_trig_sent && !s_wait_conf)
    {
        memcpy(s_pending, body, 8);
        s_wait_conf = 1;
        s_stable_ms = 0;
        s_seq++;
    }
}
