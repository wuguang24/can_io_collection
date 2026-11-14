#ifndef __TCPCLIENT_H__
#define __TCPCLIENT_H__

/* 仅 1 路 WCS UDP；再开一路会多占约 2KB RAM */
#define	TCP_CLIENT_MAX_NUM	1

struct tcp_client_table
{
    u8  tcp_recev_buf[TCP_RECEV_BUFF_SIZE];
    u8  tcp_send_buf[TCP_RECEV_BUFF_SIZE];
    u8  tcp_send_en;
    u16 tcp_recev_len;
    u16 tcp_send_len;
    u8  tcp_client_statue;
    u16 tcp_client_Delay_reg;
    struct udp_pcb *echoclient_pcb;
    u8  dest_ip[4];
    u16 dest_port;
    u8  connect_is_ok;
    u8  connect_dely;
};

extern struct tcp_client_table	tcp_client_list[]; 
extern void udp_send_process(u8 udp_offset);
extern void udp_echoclient_connect(u8 udp_offset);
extern void udp_client_process(void);

#endif 