#include "stm32f10x_it.h"
#include "stm32_eth.h"
#include "main.h"
#include "can.h"
#include "can_bus.h"

void NMI_Handler(void) {}

void HardFault_Handler(void) { NVIC_SystemReset(); }
void MemManage_Handler(void) { NVIC_SystemReset(); }
void BusFault_Handler(void)  { NVIC_SystemReset(); }
void UsageFault_Handler(void){ NVIC_SystemReset(); }

void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}

void SysTick_Handler(void)
{
    Time_Update();
}

void DMA2_Channel5_IRQHandler(void)
{
    if(DMA_GetFlagStatus(DMA2_FLAG_TC5) == SET)
    {
        DMA_ClearFlag(DMA2_FLAG_TC5);
        USART_ClearITPendingBit(UART4, USART_IT_TC);
        USART_ITConfig(UART4, USART_IT_TC, ENABLE);
    }
}

void UART4_IRQHandler(void)
{
    u16 curDataCnt;

    if(USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)
    {
        curDataCnt = UART4->SR;
        curDataCnt = UART4->DR;
        curDataCnt = DMA_GetCurrDataCounter(DMA2_Channel3);
        uart4_recv_len = UART4_DATA_MAX - curDataCnt;
        DMA_Cmd(DMA2_Channel3, DISABLE);
        DMA_SetCurrDataCounter(DMA2_Channel3, UART4_DATA_MAX);
        DMA_Cmd(DMA2_Channel3, ENABLE);
    }
    if(USART_GetITStatus(UART4, USART_IT_TC) != RESET)
    {
        USART_ClearFlag(UART4, USART_IT_TC);
        NVIC_ClearPendingIRQ(UART4_IRQn);
        USART_ITConfig(UART4, USART_IT_TC, DISABLE);
        UART4_RX_485;
        uart4_recv_len = 0;
    }
}

void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg RxMessage;

    while(CAN_MessagePending(CAN1, CAN_FIFO0))
    {
        CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
        aw1_user.recv_count_flag[1] = 1;
        if(RxMessage.IDE == CAN_ID_EXT)
        {
            can_bus_frame_receive(RxMessage, 0);
        }
        else
        {
            can_recv_frame(CAN_CH_0, &RxMessage);
            can_std_bridge_tx(CAN_CH_0, &RxMessage);
        }
    }
    CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
}

void CAN2_RX1_IRQHandler(void)
{
    CanRxMsg RxMessage;

    while(CAN_MessagePending(CAN2, CAN_FIFO1))
    {
        CAN_Receive(CAN2, CAN_FIFO1, &RxMessage);
        aw1_user.recv_count_flag[2] = 1;
        if(RxMessage.IDE == CAN_ID_EXT)
        {
            can_bus_frame_receive(RxMessage, 1);
        }
        else
        {
            can_recv_frame(CAN_CH_1, &RxMessage);
            can_std_bridge_tx(CAN_CH_1, &RxMessage);
        }
    }
    CAN_ClearITPendingBit(CAN2, CAN_IT_FMP1);
}
