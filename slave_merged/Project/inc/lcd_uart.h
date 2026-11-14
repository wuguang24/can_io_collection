#ifndef  __LCD_UART_H_
#define  __LCD_UART_H_

#define UART4_DATA_MAX  50

#define WR_MUL          0x10
#define PORT_NUM_ADDR   0x1000
#define PORT_ADDR       0x1050

#define SCREEN_NUM_MAX      4
#define SCREEN_TYPE_MAX     4

typedef struct
{
    u8 scr_buf[16][50];
    u8 port_io;//板卡的格口addr，0~8
    u8 type;//1格口、2入袋件数、3格口流向、4目的地代码
    u8 total_len;
} sScreenDate;


extern u8 uart4_send_buff[];
extern u8 uart4_send_len;
extern u8 uart4_recv_buff[];
extern u8 uart4_recv_len;
extern sScreenDate scrData;

void usart_screen_show_process(void);
#endif