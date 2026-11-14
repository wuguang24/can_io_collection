#include "main.h"
#include "fault_led.h"
#include "station_learn.h"

#define SYSTEMTICK_PERIOD_MS  1

__IO uint32_t LocalTime = 0;
uint32_t timingdelay;
u16 sec_reg = 100;
u16 sec_flag = 0;
u8 msec_flag = 0;

u8 Local_Station = 0;
u8 Local_Station1 = 0;
u8 Local_Station2 = 0;

RCC_ClocksTypeDef RCC_Clocks;
u32 test_timing[20] = {0};

void sec_process(void)
{
    if(sec_flag == 1)
    {
        sec_flag = 0;
        /* 无 PIN：每秒公告一次，主机可 142A + 自动补发 */
        slave_cfg_announce_pin_periodic();
    }
}

void scan_dip_state(void)
{
    Local_Station1 = ((S4_STATE ? 8:0) + (S2_STATE ? 4:0) + (S3_STATE ? 2:0) + (S1_STATE ? 1:0));
    Local_Station2 = ((S8_STATE ? 8:0) + (S6_STATE ? 4:0) + (S7_STATE ? 2:0) + (S5_STATE ? 1:0));
    Local_Station = Local_Station1 * 10 + Local_Station2;
    station_learn_apply_dip();
}

void ms_hold_delay(void)
{
    u8 i;
    static u16 delay[3] = {0};

    for(i = 0; i < 3; i++)
    {
        if(aw1_user.recv_count_flag[i] == 1)
        {
            aw1_user.recv_count_flag[i] = 0;
            aw1_user.led_uart_can[i] = 1;
            delay[i] = 0;
        }

        if(aw1_user.led_uart_can[i] == 1)
        {
            if(++delay[i] >= 100)
            {
                delay[i] = 0;
                aw1_user.led_uart_can[i] = 0;
            }
        }
    }
}

void msec_process(void)
{
    if(msec_flag != 1) return;
    msec_flag = 0;

    scan_dip_state();
    io_ctrl_process();
    can_recv_process();
    can_send_process();
    flash_upload_para();
    fault_led_poll_1ms();
    ms_hold_delay();
    aw20036_led_all_ctrl();
}

int main(void)
{
    BSP_Init();
    aw2006_para_Init();
    vcanbus_initcansendqueue_two();
    vcanbus_initcansendqueue_one();
    can_init();
    aw20036_led_init(&aw1);
    fault_led_init();
    station_learn_init();
    scan_dip_state();
    vflash_read_startup_item_para();

    while(1)
    {
        msec_process();
        sec_process();
        slave_alarm_ctrl_process();
        vcan_sendframe_process_two();
        vcan_sendframe_process_one();
    }
}

void Delay(uint32_t nCount)
{
    timingdelay = LocalTime + nCount;
    while(timingdelay > LocalTime) {}
}

void Time_Update(void)
{
    LocalTime += SYSTEMTICK_PERIOD_MS;
    msec_flag = 1;

    if(sec_reg != 0)
    {
        sec_reg--;
        if(sec_reg == 0)
        {
            sec_reg = 1000;
            sec_flag = 1;
        }
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
    while(1) {}
}
#endif
