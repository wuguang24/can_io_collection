#include "main.h"
#include "lcd_uart.h"


u8 uart4_send_buff[UART4_DATA_MAX];
u8 uart4_send_len;
u8 uart4_recv_buff[UART4_DATA_MAX];
u8 uart4_recv_len;
//
sScreenDate scrData;


void uart_send(u8 *buff, u8 len)
{
    uart4_send_len = len;
    UART4_TX_485;
    aw1_user.led_uart_can[0] = 1;

        USART_DMACmd(UART4, USART_DMAReq_Tx, DISABLE);
        DMA_Cmd(DMA2_Channel5, DISABLE);
        DMA_SetCurrDataCounter(DMA2_Channel5, uart4_send_len);  // 设置要发送的字节数目
        USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE);
        DMA_Cmd(DMA2_Channel5, ENABLE);             //开始DMA发送 
}

// void uart_send(u8 *buff, u8 len)
// {
//     uart4_send_len = len;
//     UART4_TX_485;
//     aw1_user.led_uart_can[0] = 1;

//     for(u16 i = 0 ; i < uart4_send_len ; i++) {
//         USART_ClearFlag(UART4, USART_FLAG_TC);
//         USART_SendData(UART4, buff[i]);
//         while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET);
//     }
            
//     UART4_RX_485;   
//     USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);

// }

u16 mb_crc16(u8 *data,u16 len)
{
    u16 crcvalue = 0xFFFF;
    u8  i;
    
    while(len--) {
        crcvalue ^= *data++;
        for(i=0; i<8; i++){
            if(crcvalue & 0x0001){
                crcvalue = (crcvalue>>1)^0xA001;
            }
            else{
                crcvalue = (crcvalue>>1);
            }
        }
    }
    return  crcvalue;
}

/* 重定向到 UART4-TX */
// int fputc(int ch, FILE *f)
// {
//     UART4_TX_485;
//     while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);
//     USART_SendData(UART4, (uint8_t)ch);
//     UART4_RX_485;
//     return ch;
// }

void uart_send_test(void)
{

    u8 data[30] = {0};
    u8 len = 8;
    static u8 num = 0;
    
    data[0] = num;
    data[1] = num;
    data[2] = num;
    data[3] = num;
    data[4] = num;
    data[5] = num;
    data[6] = num;
    data[7] = num++;
    
    memcpy(uart4_send_buff, data, len);
    uart_send(uart4_send_buff, len);
}

void usart_screen_show_process(void)
{

    //
    u8 i, len;
    u8* data;
    u16 crc_data;
    static u8 cur = 0;

    for ( i = cur; i < SCREEN_NUM_MAX * SCREEN_TYPE_MAX; i++)
    {
        cur = i + 1;
        if (cur >= SCREEN_NUM_MAX * SCREEN_TYPE_MAX)
        {
            cur = 0;
        }
        
        if (scrData.scr_buf[i][0] != 0 && scrData.scr_buf[i][0] < 50)//len != 0, 长度限制50
        {
            len = scrData.scr_buf[i][0];

            data = &scrData.scr_buf[i][1];
            // crc_data = mb_crc16(data, len - 2);//更新crc
            // *(data + len - 2) = crc_data &0xFF;
            // *(data + len - 1) = (crc_data >>8)&0xFF;
            

            memcpy(uart4_send_buff, data, len);
            uart_send(uart4_send_buff, len);
            break;
        }
    }
}