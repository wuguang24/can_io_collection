/*
 * F107 EXP FreeRTOS 调度
 *  CanIo (5): 1ms — flash/IO/cfg/CAN 轮询发送
 *  Net   (4): ETH/lwIP/UDP/WCS
 *  Maint (3): 1s — LED/心跳延时/版本/配置丢失
 *  Timer (2): 500ms 上报、5s CAN 故障
 */
#include "stm32_eth.h"
#include "netconf.h"
#include "main.h"
#include "TCPclient.h"
#include "udpboardcast.h"
#include "cfg_table.h"
#include "cfg_logic.h"
#include "cfg_can_ext.h"
#include "cfg_store.h"
#include "io.h"
#include "fun.h"
#include "can_bus.h"
#include "stm32f107.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "log.h"

#define TASK_CANIO_STACK      ( 256 )
#define TASK_NET_STACK        ( 1024 )
#define TASK_MAINT_STACK      ( 160 )

#define TASK_CANIO_PRIO       ( 5 )
#define TASK_MAINT_PRIO       ( 4 )   /* ?? Net?ETH/PHY ??? LED ?? */
#define TASK_NET_PRIO         ( 3 )

#define UPLOAD_PERIOD_MS      ( 500 )
#define CAN_ERR_PERIOD_MS     ( 5000 )
#define MAINT_PERIOD_MS       ( 1000 )
#define NET_IDLE_WAIT_MS      ( 2 )
#define CAN_POLL_PERIOD_MS    ( 5 )

__IO uint32_t LocalTime = 0;
uint32_t timingdelay;
u8  heart_delay = 0;
u8  ioCtrl_num = 0;
u8  ext_host_flag = 0;
u8  io_version_flag = 0;

static SemaphoreHandle_t xAppMutex = NULL;
static TaskHandle_t xCanIoTaskHandle = NULL;
static TaskHandle_t xNetTaskHandle = NULL;
static TimerHandle_t xTmrUpload = NULL;
static TimerHandle_t xTmrCanErr = NULL;

void App_Lock(void)
{
    if(xAppMutex != NULL)
    {
        (void)xSemaphoreTake(xAppMutex, portMAX_DELAY);
    }
}

void App_Unlock(void)
{
    if(xAppMutex != NULL)
    {
        (void)xSemaphoreGive(xAppMutex);
    }
}

void CanIo_NotifyFromISR(long *pxHigherPriorityTaskWoken)
{
    if(xCanIoTaskHandle != NULL)
    {
        vTaskNotifyGiveFromISR(xCanIoTaskHandle, (BaseType_t *)pxHigherPriorityTaskWoken);
    }
}

void Net_Notify(void)
{
    if(xNetTaskHandle != NULL)
    {
        (void)xTaskNotifyGive(xNetTaskHandle);
    }
}

void read_mac_addrs(void)
{
    u32 Mac_Code;
    u32 CpuID[3];

    CpuID[0] = *(u32 *)(0x1FFFF7E8);
    CpuID[1] = *(u32 *)(0x1FFFF7EC);
    CpuID[2] = *(u32 *)(0x1FFFF7F0);
    Mac_Code = (CpuID[0] >> 1) + (CpuID[1] >> 2) + (CpuID[2] >> 3);
    mac_addr[0] = 0x00;
    mac_addr[1] = 0x00;
    mac_addr[2] = (u8)((Mac_Code >> 24) & 0xFF);
    mac_addr[3] = (u8)((Mac_Code >> 16) & 0xFF);
    mac_addr[4] = (u8)((Mac_Code >> 8) & 0xFF);
    mac_addr[5] = (u8)(Mac_Code & 0xFF);
}

void scan_dip_state(void)
{
    u8 tmp, i;
    u8 dip_value = 0;

    tmp = 0;
    for(i = 0; i < 20; i++)
    {
        if(DIP1_STATE) tmp |= 0x01;
        if(DIP2_STATE) tmp |= 0x02;
        if(DIP3_STATE) tmp |= 0x04;
        if(DIP4_STATE) tmp |= 0x08;
        if(DIP5_STATE) tmp |= 0x10;
        if(DIP6_STATE) tmp |= 0x20;
        if(DIP7_STATE) tmp |= 0x40;
        if(dip_value != tmp)
        {
            dip_value = tmp;
            i = 0;
        }
    }
    /* 从机数量与站号以 1423/Flash 为准；拨码仅保留扩展主机位 */
    ext_host_flag = ((~dip_value) >> 6) & 0x1;
}

/* 1430：须连续 N 次(1s)检测到未全就位才上报，中间恢复则清零 */
#define CFG_1430_NOT_READY_STREAK   3u
static u8 s_1430_not_ready_streak;

static void sec_process(void)
{
    if(heart_delay != 0)
    {
        heart_delay--;
    }
    /* LED ?? CanIo 500ms ?????????????? */

    if(io_version_flag < 3)
    {
        io_version_flag++;
        AddSendMsgToQueue(SEND_MSG_IOCTRL2WCS_VERSION_TYPE);
        AddSendMsgToQueue(SEND_MSG_VALIDATE_VERSION_TYPE);
    }

    if(!cfg_host_ready())
    {
        s_1430_not_ready_streak = 0;
        AddSendMsgToQueue(SEND_MSG_CONF_LOSE_TYPE);
    }
    else if(cfg_can_fail_count() > 0 || g_cfg_store_autosave_fail)
    {
        s_1430_not_ready_streak = 0;
        /* 从机 PIN/站号异常，或主机 Flash autosave 失败：周期提醒 WCS */
        AddSendMsgToQueue(SEND_MSG_SLAVE_CFG_FAIL_TYPE);
    }
    else if(!cfg_can_topo_all_ready())
    {
        /* V5扩展1430：连续3s未全就位才发；全0x03或中途恢复则清零；在线看1402 */
        if(s_1430_not_ready_streak < 0xFFu)
        {
            s_1430_not_ready_streak++;
        }
        if(s_1430_not_ready_streak >= CFG_1430_NOT_READY_STREAK)
        {
            AddSendMsgToQueue(SEND_MSG_SLAVE_READY_TYPE);
        }
    }
    else
    {
        s_1430_not_ready_streak = 0;
    }
}

static void prvUploadTimerCb(TimerHandle_t xTimer)
{
    (void)xTimer;
    upload2WCS();
}

static void prvCanErrTimerCb(TimerHandle_t xTimer)
{
    (void)xTimer;
    upload_slave_can_error_cmd();
}

static void prvTaskCanIo(void *pvParameters)
{
    static u8 poll_div = 0;
    TickType_t last_led;

    (void)pvParameters;

    AppIrq_EnableCan();
    cfg_store_boot_finish();
    if(g_cfg_host_ready)
    {
        rtt_print_blue("cfg_store boot apply OK ports=%u\r\n", (unsigned)g_cfg_port_num);
    }
    last_led = xTaskGetTickCount();
    LED_OFF;
    rtt_print_blue("CanIo start\r\n");

    for(;;)
    {
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1));

        /* 1s ??????=HardFault??=CanIo ?? */
        if((xTaskGetTickCount() - last_led) >= 1000u)
        {
            last_led = xTaskGetTickCount();
            if(LED_STATE)
            {
                LED_ON;
            }
            else
            {
                LED_OFF;
            }
            rtt_print_blue("CanIo tick=%u led=%u eth=%u lwip=%u\r\n",
                           (unsigned)last_led,
                           (unsigned)LED_STATE,
                           (unsigned)EthInitStatus,
                           (unsigned)g_lwip_ready);
        }

        flashUploadProcess();
        flashResetProcess();

        cfg_pin_rx_tick_1ms();
        can_bus_port_state_tick_1ms();
        cfg_can_ext_poll();

        App_Lock();
        if(g_cfg_host_ready && g_cfg_port_num > 0)
        {
            io_ctrl_proc();
        }
        can_bus_set_port_close_cmd();
        can_bus_forward_screen_cmd();
        App_Unlock();

        /* 先轮询：STD 灯控抢邮箱，再把 EXT 入队；后排 vcan 发 EXT，避免灯帧被挤掉 */
        if(++poll_div >= CAN_POLL_PERIOD_MS)
        {
            poll_div = 0;
            if(ext_host_flag == 0)
            {
                can_bus_send_cycle_5ms();
            }
        }
        vcan_send_frame_process();
    }
}

static void prvTaskNet(void *pvParameters)
{
    (void)pvParameters;
    rtt_print_blue("Net start\r\n");

    for(;;)
    {
        LocalTime = xTaskGetTickCount();

        /* ETH_Init(PHY) ? LwIP(DMA/ETH_Start) ? ?? ETH IRQ */
        if(g_lwip_ready == 0)
        {
            if(EthInitStatus == 0)
            {
                rtt_print_blue("ETH_Init...\r\n");
                Ethernet_Configuration();
                rtt_print_blue("ETH_Init done status=%u\r\n", (unsigned)EthInitStatus);
            }
            if(EthInitStatus != 0)
            {
                rtt_print_blue("LwIP_Init...\r\n");
                LwIP_Init();
                UDP_server_init();
                rtt_print_blue("LwIP ready\r\n");
            }
            else
            {
                rtt_print_red("ETH_Init fail, retry\r\n");
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
        }

        LwIP_Periodic_Handle(LocalTime);
        App_Lock();
        udp_client_process();
        send_message_to_sever();
        App_Unlock();

        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(NET_IDLE_WAIT_MS));
    }
}

static void prvTaskMaint(void *pvParameters)
{
    TickType_t xLastWake = xTaskGetTickCount();

    (void)pvParameters;

    for(;;)
    {
        sec_process();
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(MAINT_PERIOD_MS));
    }
}

int main(void)
{
    u8 i;
    BaseType_t ok;

    System_Setup();
    rtt_init();
    rtt_print_blue("\r\n=== EXP F107 RTOS boot ===\r\n");
    rtt_print_blue("SysTick cleared, irq masked until scheduler\r\n");

    /* ??? 3 ?????? HardFault ?? */
    for(i = 0; i < 6; i++)
    {
        if(LED_STATE)
        {
            LED_ON;
        }
        else
        {
            LED_OFF;
        }
        {
            volatile u32 n = 400000u;
            while(n--) {}
        }
    }
    rtt_print_blue("GPIO blink done\r\n");

    InitSendMsgQueue();
    scan_dip_state();
    scan_dip_state();
    vpcfun_init_sys_item();
    fun_Init_INT_16_Queue();
    vcan_bus_frame_queue_init();
    cfg_table_init();
    cfg_logic_init();
    cfg_can_ext_init();
    io_port_init();

    {
        volatile u32 n = 2000u * 8000u;
        while(n--)
        {
        }
    }

    read_mac_addrs();
    flashReadNetPara();
    flashReadVersionPara();
    flashVersionInitCheck();

    /* Flash：main 仅选活跃 Bank；展开配置延后到 CanIo */
    cfg_store_boot_try_apply();

    xAppMutex = xSemaphoreCreateMutex();

    ok = pdPASS;
    if(xTaskCreate(prvTaskCanIo, "CanIo", TASK_CANIO_STACK, NULL, TASK_CANIO_PRIO, &xCanIoTaskHandle) != pdPASS)
    {
        ok = pdFAIL;
    }
    if(xTaskCreate(prvTaskNet,   "Net",   TASK_NET_STACK,   NULL, TASK_NET_PRIO,   &xNetTaskHandle) != pdPASS)
    {
        ok = pdFAIL;
    }
    if(xTaskCreate(prvTaskMaint, "Maint", TASK_MAINT_STACK, NULL, TASK_MAINT_PRIO, NULL) != pdPASS)
    {
        ok = pdFAIL;
    }

    xTmrUpload = xTimerCreate("Upd", pdMS_TO_TICKS(UPLOAD_PERIOD_MS), pdTRUE, NULL, prvUploadTimerCb);
    xTmrCanErr = xTimerCreate("CErr", pdMS_TO_TICKS(CAN_ERR_PERIOD_MS), pdTRUE, NULL, prvCanErrTimerCb);
    if(xTmrUpload != NULL)
    {
        (void)xTimerStart(xTmrUpload, 0);
    }
    if(xTmrCanErr != NULL)
    {
        (void)xTimerStart(xTmrCanErr, 0);
    }

    if(ok != pdPASS || xAppMutex == NULL)
    {
        rtt_print_red("task create FAIL\r\n");
        for(;;)
        {
            LED_ON;
            {
                volatile u32 n = 800000u;
                while(n--) {}
            }
            LED_OFF;
            {
                volatile u32 n = 800000u;
                while(n--) {}
            }
        }
    }

    rtt_print_blue("vTaskStartScheduler\r\n");
    vTaskStartScheduler();

    rtt_print_red("scheduler returned (heap?)\r\n");
    for(;;)
    {
        LED_ON;
        {
            volatile u32 n = 2000000u;
            while(n--) {}
        }
        LED_OFF;
        {
            volatile u32 n = 2000000u;
            while(n--) {}
        }
    }
}

void Delay(uint32_t nCount)
{
    /* ETH ? _eth_delay_(PHY_ResetDelay) ??????????(~1M)?
     * ????????? vTaskDelay?????????“??”????? */
    volatile uint32_t i;
    while(nCount--)
    {
        for(i = 0; i < 8; i++)
        {
        }
        /* ???????? Maint LED / CanIo ??? */
        if((nCount & 0x3FFu) == 0u)
        {
            if(xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
            {
                taskYIELD();
            }
        }
    }
}

void Time_Update(void)
{
    LocalTime = xTaskGetTickCount();
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
    while(1)
    {
    }
}
#endif
