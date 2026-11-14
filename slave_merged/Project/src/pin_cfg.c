#include "pin_cfg.h"
#include "io.h"
#include "flash.h"
#include "slave_cfg_store.h"
#include "string.h"

u8  g_pin_ready = 0;
u8  g_pin_n = 0;
u16 g_pin_ver = 0;
u16 g_pin_board_flags = 0;
pin_cfg_rec_t g_pin_rec[PIN_CFG_PORT_MAX];
u16 g_pin_filter_ms[PIN_CFG_PIN_MAX];

static u16 slave_cfg_crc16_local(const u8 *data, u16 len)
{
    u16 crc = 0xFFFF;
    u16 i, j;

    for(i = 0; i < len; i++)
    {
        crc ^= ((u16)data[i] << 8);
        for(j = 0; j < 8; j++)
        {
            if(crc & 0x8000)
            {
                crc = (u16)((crc << 1) ^ 0x1021);
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static u16 nz_or5(u16 ms)
{
    return ms ? ms : 5;
}

static void pin_cfg_apply_mask_filters(u8 mask, u16 ms)
{
    u8 pin;

    for(pin = 0; pin < PIN_CFG_PIN_MAX; pin++)
    {
        if(mask & (u8)(1u << pin))
        {
            g_pin_filter_ms[pin] = ms;
        }
    }
}

static void pin_cfg_rebuild_filter(void)
{
    u8 i, pin;
    u16 base;

    base = nz_or5(timfunc.lock_filter_time);
    for(i = 0; i < PIN_CFG_PIN_MAX; i++)
    {
        g_pin_filter_ms[i] = base;
    }

    if(!g_pin_ready || g_pin_n == 0)
    {
        return;
    }

    for(i = 0; i < g_pin_n && i < PIN_CFG_PORT_MAX; i++)
    {
        pin = g_pin_rec[i].in_lock;
        if(pin < PIN_CFG_PIN_MAX)
        {
            g_pin_filter_ms[pin] = nz_or5(timfunc.lock_filter_time);
        }
        pin_cfg_apply_mask_filters(g_pin_rec[i].in_or_lock_mask,
                                   nz_or5(timfunc.lock_filter_time));
        pin = g_pin_rec[i].in_print;
        if(pin < PIN_CFG_PIN_MAX)
        {
            g_pin_filter_ms[pin] = nz_or5(timfunc.solt_filter_time);
        }
        pin_cfg_apply_mask_filters(g_pin_rec[i].in_or_print_mask,
                                   nz_or5(timfunc.solt_filter_time));
        pin = g_pin_rec[i].in_full_pack;
        if(pin < PIN_CFG_PIN_MAX)
        {
            g_pin_filter_ms[pin] = nz_or5(timfunc.full_filter_time);
        }
        pin_cfg_apply_mask_filters(g_pin_rec[i].in_or_full_mask,
                                   nz_or5(timfunc.full_filter_time));
        pin = g_pin_rec[i].in_half_full;
        if(pin < PIN_CFG_PIN_MAX)
        {
            g_pin_filter_ms[pin] = nz_or5(timfunc.full_filter_time);
        }
        pin = g_pin_rec[i].in_clamp_pack;
        if(pin < PIN_CFG_PIN_MAX)
        {
            g_pin_filter_ms[pin] = nz_or5(timfunc.block_filter_time);
        }
        pin = g_pin_rec[i].in_unload_check;
        if(pin < PIN_CFG_PIN_MAX)
        {
            g_pin_filter_ms[pin] = nz_or5(
                timfunc.door_filter_time ? timfunc.door_filter_time
                                         : timfunc.solt_filter_time);
        }
    }
}

static u8 pin_cfg_detect_layout(const u8 *body, u16 body_len, u8 *hdr_out,
                                u8 *rec_size_out)
{
    u8 n;
    u16 payload;

    if(body == NULL || body_len < PIN_CFG_SLICE_HDR_V3 || hdr_out == NULL
       || rec_size_out == NULL)
    {
        return 1;
    }
    n = body[2];
    if(n > PIN_CFG_PORT_MAX)
    {
        return 1;
    }

    if(body_len >= PIN_CFG_SLICE_HDR_EXT)
    {
        payload = (u16)(body_len - PIN_CFG_SLICE_HDR_EXT);
        if(n > 0 && payload == (u16)n * PIN_CFG_REC_SIZE)
        {
            *hdr_out = PIN_CFG_SLICE_HDR_EXT;
            *rec_size_out = PIN_CFG_REC_SIZE;
            return 0;
        }
        if(n > 0 && payload == (u16)n * PIN_CFG_REC_SIZE_JAM12)
        {
            *hdr_out = PIN_CFG_SLICE_HDR_EXT;
            *rec_size_out = PIN_CFG_REC_SIZE_JAM12;
            return 0;
        }
        if(n == 0)
        {
            *hdr_out = PIN_CFG_SLICE_HDR_EXT;
            *rec_size_out = PIN_CFG_REC_SIZE;
            return 0;
        }
    }

    payload = (u16)(body_len - PIN_CFG_SLICE_HDR_V3);
    if(n == 0)
    {
        *hdr_out = PIN_CFG_SLICE_HDR_V3;
        *rec_size_out = PIN_CFG_REC_SIZE;
        return 0;
    }
    if(payload == (u16)n * PIN_CFG_REC_SIZE)
    {
        *hdr_out = PIN_CFG_SLICE_HDR_V3;
        *rec_size_out = PIN_CFG_REC_SIZE;
        return 0;
    }
    if(payload == (u16)n * PIN_CFG_REC_SIZE_OR_JAM4)
    {
        *hdr_out = PIN_CFG_SLICE_HDR_V3;
        *rec_size_out = PIN_CFG_REC_SIZE_OR_JAM4;
        return 0;
    }
    if(payload == (u16)n * PIN_CFG_REC_SIZE_V24)
    {
        *hdr_out = PIN_CFG_SLICE_HDR_V3;
        *rec_size_out = PIN_CFG_REC_SIZE_V24;
        return 0;
    }
    return 1;
}

static void pin_cfg_copy_one(pin_cfg_rec_t *dst, const u8 *src, u8 rec_size)
{
    u8 jam_n;

    memset(dst, 0, sizeof(*dst));
    dst->in_full_tier1 = PIN_CFG_INVALID;
    dst->in_full_tier2 = PIN_CFG_INVALID;
    if(rec_size < PIN_CFG_REC_SIZE_V24)
    {
        return;
    }
    memcpy(dst, src, PIN_CFG_REC_SIZE_V24);
    if(rec_size >= PIN_CFG_REC_SIZE_OR_JAM4)
    {
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
    if(rec_size >= PIN_CFG_REC_SIZE_JAM12)
    {
        dst->jam_extra_num = src[15];
        jam_n = dst->jam_extra_num;
        if(jam_n > PIN_CFG_JAM_MAX)
        {
            jam_n = PIN_CFG_JAM_MAX;
        }
        memcpy(dst->jam_extra_no, &src[16], (u16)jam_n * sizeof(u16));
        dst->in_or_full_mask = src[40];
        dst->in_or_lock_mask = src[41];
        dst->in_or_print_mask = src[42];
        dst->in_full_tier1 = PIN_CFG_INVALID;
        dst->in_full_tier2 = PIN_CFG_INVALID;
        if(rec_size >= PIN_CFG_REC_SIZE)
        {
            dst->in_full_tier1 = src[43];
            dst->in_full_tier2 = src[44];
        }
    }
}

void pin_cfg_clear(void)
{
    g_pin_ready = 0;
    g_pin_n = 0;
    g_pin_ver = 0;
    g_pin_board_flags = 0;
    memset(g_pin_rec, 0, sizeof(g_pin_rec));
    pin_cfg_rebuild_filter();
}

u8 pin_cfg_apply(const u8 *body, u16 body_len)
{
    u8 n;
    u8 i;
    u8 hdr;
    u8 rec_size;
    u8 rec_off;

    if(body == NULL || body_len < PIN_CFG_SLICE_HDR_V3)
    {
        return 1;
    }
    if(pin_cfg_detect_layout(body, body_len, &hdr, &rec_size) != 0)
    {
        return 1;
    }

    n = body[2];
    rec_off = hdr;
    if(hdr == PIN_CFG_SLICE_HDR_EXT)
    {
        g_pin_board_flags = (u16)(body[3] | ((u16)body[4] << 8));
    }
    else
    {
        g_pin_board_flags = 0;
    }

    if(n > 0)
    {
        for(i = 0; i < n; i++)
        {
            const u8 *rp = &body[rec_off] + (u16)i * rec_size;
            u8 sta = rp[2];

            if(sta != 0 && sta != Local_Station)
            {
                return 1;
            }
        }
    }

    g_pin_ver = (u16)(body[0] | ((u16)body[1] << 8));
    g_pin_n = n;
    if(n > 0)
    {
        for(i = 0; i < n; i++)
        {
            pin_cfg_copy_one(&g_pin_rec[i],
                             &body[rec_off] + (u16)i * rec_size,
                             rec_size);
        }
        for(; i < PIN_CFG_PORT_MAX; i++)
        {
            memset(&g_pin_rec[i], 0, sizeof(g_pin_rec[i]));
        }
        g_pin_ready = 1;
    }
    else
    {
        memset(g_pin_rec, 0, sizeof(g_pin_rec));
        g_pin_ready = 0;
    }
    pin_cfg_rebuild_filter();
    return 0;
}

void pin_cfg_load_from_flash(void)
{
    if(slave_cfg_boot_load())
    {
        return;
    }

    /* 单页 PIN（FLASH_FLAG_4 @ PinPageAddr） */
    u8 *rd;
    u8 n;
    u16 body_len;
    u16 total;
    u16 calc, rxcrc;
    u8 hdr;
    u8 rec_size;
    u8 t;
    static const u8 try_hdr[4] = {PIN_CFG_SLICE_HDR_EXT, PIN_CFG_SLICE_HDR_V3,
                                  PIN_CFG_SLICE_HDR_V3, PIN_CFG_SLICE_HDR_V3};
    static const u8 try_rec[4] = {PIN_CFG_REC_SIZE, PIN_CFG_REC_SIZE,
                                  PIN_CFG_REC_SIZE_OR_JAM4, PIN_CFG_REC_SIZE_V24};

    pin_cfg_clear();
    if(*(u16 *)PinPageAddr != FLASH_FLAG_4)
    {
        return;
    }
    rd = (u8 *)(PinPageAddr + 2);
    n = rd[2];
    if(n > PIN_CFG_PORT_MAX)
    {
        return;
    }

    for(t = 0; t < 4; t++)
    {
        hdr = try_hdr[t];
        rec_size = try_rec[t];
        body_len = (u16)hdr + (u16)n * (u16)rec_size;
        total = (u16)(body_len + 2u);
        if(total > PIN_CFG_MAX_BYTES)
        {
            continue;
        }
        calc = slave_cfg_crc16_local(rd, body_len);
        rxcrc = (u16)(rd[body_len] | ((u16)rd[body_len + 1] << 8));
        if(calc == rxcrc && pin_cfg_apply(rd, body_len) == 0)
        {
            return;
        }
    }
}

u16 pin_cfg_filter_for_pin(u8 pin)
{
    if(pin >= PIN_CFG_PIN_MAX)
    {
        return 5;
    }
    if(g_pin_filter_ms[pin] == 0)
    {
        return 5;
    }
    return g_pin_filter_ms[pin];
}

void pin_cfg_on_time_updated(void)
{
    pin_cfg_rebuild_filter();
}
