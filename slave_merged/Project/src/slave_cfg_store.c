#include "slave_cfg_store.h"

#include "pin_cfg.h"

#include "io.h"

#include "flash.h"

#include "stm32f10x_flash.h"

#include "string.h"



#pragma pack(1)

typedef struct

{

    u16 magic;

    u16 ver;

    u32 seq;

    u16 pin_len;

    u16 time_len;

    u16 func_len;

    u16 payload_crc;

} slave_cfg_hdr_t;



typedef struct

{

    slave_cfg_hdr_t hdr;

    u8 pin_data[SLAVE_CFG_PIN_MAX];

    u8 time_data[SLAVE_CFG_TIME_BYTES];

    u8 func_data[SLAVE_CFG_FUNC_BYTES];

} slave_cfg_bank_t;

#pragma pack()



static u8 s_applied_mask = 0;



u8 slave_cfg_applied_mask(void)

{

    return s_applied_mask;

}



static u16 slave_cfg_crc16(const u8 *data, u16 len)

{

    u16 crc = 0xFFFFu;

    u16 i, j;



    if(data == NULL)

    {

        return 0;

    }

    for(i = 0; i < len; i++)

    {

        crc ^= (u16)((u16)data[i] << 8);

        for(j = 0; j < 8; j++)

        {

            if(crc & 0x8000u)

            {

                crc = (u16)((crc << 1) ^ 0x1021u);

            }

            else

            {

                crc <<= 1;

            }

        }

    }

    return crc;

}



static u16 slave_cfg_payload_crc_v1(const u8 *pin, u16 pin_len,

                                    const u8 *time, u16 time_len)

{

    u16 crc = 0xFFFFu;



    if(pin_len > 0)

    {

        crc = slave_cfg_crc16(pin, pin_len);

    }

    if(time_len == SLAVE_CFG_TIME_BYTES && time != NULL)

    {

        u16 tc = slave_cfg_crc16(time, SLAVE_CFG_TIME_BYTES);

        crc = (pin_len > 0) ? (u16)(crc ^ tc) : tc;

    }

    return crc;

}



static u16 slave_cfg_payload_crc(const slave_cfg_bank_t *b)

{

    u16 crc;



    if(b == NULL)

    {

        return 0;

    }

    if(b->hdr.ver < SLAVE_CFG_VER)

    {

        return slave_cfg_payload_crc_v1(b->pin_data, b->hdr.pin_len,

                                      b->time_data, b->hdr.time_len);

    }

    crc = slave_cfg_payload_crc_v1(b->pin_data, b->hdr.pin_len,

                                   b->time_data, b->hdr.time_len);

    if(b->hdr.func_len == SLAVE_CFG_FUNC_BYTES)

    {

        u16 fc = slave_cfg_crc16(b->func_data, SLAVE_CFG_FUNC_BYTES);

        crc = (u16)(crc ^ fc);

    }

    return crc;

}



static u8 slave_cfg_pin_len_ok(u16 pin_len)

{

    return (pin_len == 0 || (pin_len >= 7u && pin_len <= SLAVE_CFG_PIN_MAX)) ? 1u : 0u;

}



static u8 slave_cfg_hdr_valid(const slave_cfg_hdr_t *h)

{

    u8 has_payload;



    if(h == NULL || h->magic != SLAVE_CFG_MAGIC)

    {

        return 0;

    }

    if(h->ver != 1u && h->ver != SLAVE_CFG_VER)

    {

        return 0;

    }

    if(!slave_cfg_pin_len_ok(h->pin_len))

    {

        return 0;

    }

    if(h->time_len != 0 && h->time_len != SLAVE_CFG_TIME_BYTES)

    {

        return 0;

    }

    if(h->ver >= SLAVE_CFG_VER)

    {

        if(h->func_len != 0 && h->func_len != SLAVE_CFG_FUNC_BYTES)

        {

            return 0;

        }

    }

    has_payload = (h->pin_len >= 7u)

                  || (h->time_len == SLAVE_CFG_TIME_BYTES)

                  || (h->ver >= SLAVE_CFG_VER && h->func_len == SLAVE_CFG_FUNC_BYTES);

    return has_payload ? 1u : 0u;

}



static u8 slave_cfg_bank_valid(const slave_cfg_bank_t *b)

{

    if(!slave_cfg_hdr_valid(&b->hdr))

    {

        return 0;

    }

    return (slave_cfg_payload_crc(b) == b->hdr.payload_crc) ? 1u : 0u;

}



static const slave_cfg_bank_t *slave_cfg_read_bank(u32 addr)

{

    return (const slave_cfg_bank_t *)addr;

}



static u8 slave_cfg_pick_best(const slave_cfg_bank_t **out)

{

    const slave_cfg_bank_t *a;

    const slave_cfg_bank_t *b;

    const slave_cfg_bank_t *best = NULL;



    a = slave_cfg_read_bank(SLAVE_CFG_BANK_A_ADDR);

    b = slave_cfg_read_bank(SLAVE_CFG_BANK_B_ADDR);

    if(slave_cfg_bank_valid(a))

    {

        best = a;

    }

    if(slave_cfg_bank_valid(b))

    {

        if(best == NULL || b->hdr.seq > best->hdr.seq)

        {

            best = b;

        }

    }

    *out = best;

    return (best != NULL) ? 1u : 0u;

}



static u32 slave_cfg_inactive_addr(void)

{

    const slave_cfg_bank_t *best = NULL;



    if(!slave_cfg_pick_best(&best))

    {

        return SLAVE_CFG_BANK_A_ADDR;

    }

    if((u32)best == SLAVE_CFG_BANK_A_ADDR)

    {

        return SLAVE_CFG_BANK_B_ADDR;

    }

    return SLAVE_CFG_BANK_A_ADDR;

}



static u32 slave_cfg_next_seq(void)

{

    const slave_cfg_bank_t *best = NULL;



    if(slave_cfg_pick_best(&best))

    {

        return best->hdr.seq + 1u;

    }

    return 1u;

}



static u8 slave_cfg_program_bank(u32 addr, const slave_cfg_bank_t *src)

{

    u16 half_n;

    u16 i;

    const u16 *p;



    if(src == NULL)

    {

        return 0;

    }

    half_n = (u16)((sizeof(slave_cfg_bank_t) + 1u) / 2u);

    p = (const u16 *)src;



    FLASH_Unlock();

    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    if(FLASH_ErasePage(addr) != FLASH_COMPLETE)

    {

        FLASH_Lock();

        return 0;

    }

    for(i = 0; i < half_n; i++)

    {

        if(FLASH_ProgramHalfWord(addr + (u32)i * 2u, p[i]) != FLASH_COMPLETE)

        {

            FLASH_Lock();

            return 0;

        }

    }

    FLASH_Lock();



    if(memcmp((const void *)addr, src, sizeof(slave_cfg_bank_t)) != 0)

    {

        return 0;

    }

    return slave_cfg_bank_valid((const slave_cfg_bank_t *)addr) ? 1u : 0u;

}



static u8 slave_cfg_apply_pin_blob(const u8 *rd, u16 pin_len)

{

    u16 body_len;

    u16 calc;

    u16 rxcrc;



    if(rd == NULL || pin_len < 7)

    {

        return 0;

    }

    body_len = (u16)(pin_len - 2u);

    calc = slave_cfg_crc16(rd, body_len);

    rxcrc = (u16)(rd[body_len] | ((u16)rd[body_len + 1] << 8));

    if(calc != rxcrc || pin_cfg_apply(rd, body_len) != 0)

    {

        return 0;

    }

    return 1;

}



static u8 slave_cfg_apply_bank(const slave_cfg_bank_t *b)

{

    u8 ok = 0;



    s_applied_mask = 0;

    if(b == NULL)

    {

        return 0;

    }

    if(b->hdr.pin_len >= 7u)

    {

        if(!slave_cfg_apply_pin_blob(b->pin_data, b->hdr.pin_len))

        {

            return 0;

        }

        s_applied_mask |= SLAVE_CFG_APPLIED_PIN;

        ok = 1;

    }

    if(b->hdr.time_len == SLAVE_CFG_TIME_BYTES)

    {

        memcpy(&timfunc, b->time_data, SLAVE_CFG_TIME_BYTES);

        pin_cfg_on_time_updated();

        s_applied_mask |= SLAVE_CFG_APPLIED_TIME;

        ok = 1;

    }

    if(b->hdr.ver >= SLAVE_CFG_VER && b->hdr.func_len == SLAVE_CFG_FUNC_BYTES)

    {

        memcpy(&bfunc, b->func_data, SLAVE_CFG_FUNC_BYTES);

        s_applied_mask |= SLAVE_CFG_APPLIED_FUNC;

        ok = 1;

    }

    return ok;

}



static void slave_cfg_merge_field(u8 *dst, u16 *dst_len, u16 dst_max,

                                  const u8 *src, u16 src_len,

                                  const slave_cfg_bank_t *best,

                                  const u8 *best_data, u16 best_len)

{

    if(src != NULL && src_len > 0)

    {

        if(src_len <= dst_max)

        {

            memcpy(dst, src, src_len);

            *dst_len = src_len;

        }

        return;

    }

    if(best != NULL && best_len > 0 && best_len <= dst_max)

    {

        memcpy(dst, best_data, best_len);

        *dst_len = best_len;

    }

}



static u8 slave_cfg_commit_merge(const u8 *pin_body, u16 pin_len,

                                 const u8 *time10, u16 time_len,

                                 const u8 *func48, u16 func_len)

{

    slave_cfg_bank_t wr;

    u32 addr;

    const slave_cfg_bank_t *prev = NULL;

    const slave_cfg_bank_t *best = NULL;



    if(pin_body != NULL && !slave_cfg_pin_len_ok(pin_len))

    {

        return 0;

    }

    if(time_len != 0 && time_len != SLAVE_CFG_TIME_BYTES)

    {

        return 0;

    }

    if(func48 != NULL && func_len != SLAVE_CFG_FUNC_BYTES)

    {

        return 0;

    }



    (void)slave_cfg_pick_best(&best);

    memset(&wr, 0xFF, sizeof(wr));

    wr.hdr.magic = SLAVE_CFG_MAGIC;

    wr.hdr.ver = SLAVE_CFG_VER;

    wr.hdr.seq = slave_cfg_next_seq();



    slave_cfg_merge_field(wr.pin_data, &wr.hdr.pin_len, SLAVE_CFG_PIN_MAX,

                          pin_body, pin_len, best,

                          best ? best->pin_data : NULL,

                          best ? best->hdr.pin_len : 0);

    slave_cfg_merge_field(wr.time_data, &wr.hdr.time_len, SLAVE_CFG_TIME_BYTES,

                          time10, time_len, best,

                          best ? best->time_data : NULL,

                          best ? best->hdr.time_len : 0);

    slave_cfg_merge_field(wr.func_data, &wr.hdr.func_len, SLAVE_CFG_FUNC_BYTES,

                          func48, func_len, best,

                          (best && best->hdr.ver >= SLAVE_CFG_VER) ? best->func_data : NULL,

                          (best && best->hdr.ver >= SLAVE_CFG_VER) ? best->hdr.func_len : 0);



    if(wr.hdr.pin_len < 7u

       && wr.hdr.time_len != SLAVE_CFG_TIME_BYTES

       && wr.hdr.func_len != SLAVE_CFG_FUNC_BYTES)

    {

        return 0;

    }



    wr.hdr.payload_crc = slave_cfg_payload_crc(&wr);

    addr = slave_cfg_inactive_addr();

    if(!slave_cfg_program_bank(addr, &wr))

    {

        return 0;

    }

    if(!slave_cfg_pick_best(&prev) || prev->hdr.seq != wr.hdr.seq)

    {

        return 0;

    }

    return 1;

}



u8 slave_cfg_boot_load(void)

{

    const slave_cfg_bank_t *best = NULL;



    if(!slave_cfg_pick_best(&best))

    {

        return 0;

    }

    return slave_cfg_apply_bank(best);

}



u8 slave_cfg_commit_pin(const u8 *pin_body, u16 pin_len)

{

    return slave_cfg_commit_merge(pin_body, pin_len,

                                  (const u8 *)&timfunc, SLAVE_CFG_TIME_BYTES,

                                  (const u8 *)&bfunc, SLAVE_CFG_FUNC_BYTES);

}



u8 slave_cfg_commit_time(const u8 *time10)

{

    if(time10 == NULL)

    {

        return 0;

    }

    return slave_cfg_commit_merge(NULL, 0, time10, SLAVE_CFG_TIME_BYTES,

                                  (const u8 *)&bfunc, SLAVE_CFG_FUNC_BYTES);

}



u8 slave_cfg_commit_func(const u8 *func48)

{

    if(func48 == NULL)

    {

        return 0;

    }

    return slave_cfg_commit_merge(NULL, 0,

                                  (const u8 *)&timfunc, SLAVE_CFG_TIME_BYTES,

                                  func48, SLAVE_CFG_FUNC_BYTES);

}



u8 slave_cfg_migrate_from_flash_pages(void)

{

    const slave_cfg_bank_t *best = NULL;

    u8 have_pin = 0;

    u8 have_time = 0;

    u8 have_func = 0;



    if(slave_cfg_pick_best(&best))

    {

        have_pin = (best->hdr.pin_len >= 7u) ? 1u : 0u;

        have_time = (best->hdr.time_len == SLAVE_CFG_TIME_BYTES) ? 1u : 0u;

        have_func = (best->hdr.ver >= SLAVE_CFG_VER

                     && best->hdr.func_len == SLAVE_CFG_FUNC_BYTES) ? 1u : 0u;

    }



    if(*(u16 *)TimeFlagAddr == FLASH_FLAG_2 && !have_time)

    {

        memcpy(&timfunc, (const void *)TimeParaAddr, sizeof(timfunc));

        have_time = 1;

    }

    if(*(u16 *)UserParaAddr == FLASH_FLAG_1 && !have_func)

    {

        memcpy(&bfunc, (const void *)UserFuncAddr, sizeof(bfunc));

        have_func = 1;

    }

    if(!have_pin && !have_time && !have_func)

    {

        return 0;

    }

    return slave_cfg_commit_merge(

        (have_pin && best != NULL) ? best->pin_data : NULL,

        (have_pin && best != NULL) ? best->hdr.pin_len : 0,

        have_time ? (const u8 *)&timfunc : NULL,

        have_time ? SLAVE_CFG_TIME_BYTES : 0,

        have_func ? (const u8 *)&bfunc : NULL,

        have_func ? SLAVE_CFG_FUNC_BYTES : 0);

}

