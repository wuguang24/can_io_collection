/**
  ******************************************************************************
  * @file    tcp_echoclient.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    31-October-2011
  * @brief   tcp echoclient application using LwIP RAW API
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/udp.h"
#include "stm32f10x.h"
#include "memp.h"
#include "netconf.h"
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "TCPclient.h"
#include "log.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

struct tcp_client_table	  tcp_client_list[TCP_CLIENT_MAX_NUM];
extern u8 msg_send_en;

void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, struct ip_addr *addr, u16_t port);
void get_ip_para_process(u8 tcp_offset);

/**********************************************************

***********************************************************/
void creat_udp_connect_process(u8 udp_offset)
{
    tcp_client_list[udp_offset].echoclient_pcb = udp_new();
    tcp_client_list[udp_offset].tcp_client_statue = CLIENT_DIS_CONNECT;
    if (tcp_client_list[udp_offset].echoclient_pcb != NULL)
    {
        tcp_client_list[udp_offset].connect_is_ok = 1;
        tcp_client_list[udp_offset].connect_dely = 0;
        tcp_client_list[udp_offset].tcp_send_en = 0;
        tcp_client_list[udp_offset].tcp_send_len = 0;
    }
}
/***************************************************

**************************************************/
void udp_echoclient_connect(u8 udp_offset)
{
    //struct pbuf *p;
    struct ip_addr DestIPaddr;
    err_t err;

    /*assign destination IP address */
    get_ip_para_process(udp_offset);
    IP4_ADDR( &DestIPaddr, tcp_client_list[udp_offset].dest_ip[0], tcp_client_list[udp_offset].dest_ip[1], tcp_client_list[udp_offset].dest_ip[2], tcp_client_list[udp_offset].dest_ip[3] );
    udp_bind(tcp_client_list[udp_offset].echoclient_pcb, IP_ADDR_ANY, netPcParaLocal.rNet[0].localPort);
    /* configure destination IP address and port */
    err = udp_connect(tcp_client_list[udp_offset].echoclient_pcb, &DestIPaddr, tcp_client_list[udp_offset].dest_port);
    if (err == ERR_OK)
    {
        /* Set a receive callback for the upcb */
        udp_recv(tcp_client_list[udp_offset].echoclient_pcb, udp_receive_callback, NULL);
        tcp_client_list[udp_offset].tcp_client_statue = CLIENT_CONNECT_OK;
        tcp_client_list[udp_offset].connect_dely = 0;
        rtt_print_blue("UDP ok %d.%d.%d.%d:%u localPort=%u\r\n",
                       tcp_client_list[udp_offset].dest_ip[0],
                       tcp_client_list[udp_offset].dest_ip[1],
                       tcp_client_list[udp_offset].dest_ip[2],
                       tcp_client_list[udp_offset].dest_ip[3],
                       (unsigned)tcp_client_list[udp_offset].dest_port,
                       (unsigned)netPcParaLocal.rNet[0].localPort);
        /* ??????????? ARP/UDP */
        send_heart_msg(&(tcp_client_list[0].tcp_send_buf[0]),
                       &(tcp_client_list[0].tcp_send_len), HEART_MSG_TYPE);
        tcp_client_list[0].tcp_send_en = 1;
        heart_delay = HEART_DELAY;
    }
    else
    {
        rtt_print_red("UDP connect fail err=%d\r\n", (int)err);
    }
}
/**************************************************************************

***************************************************************************/
void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, struct ip_addr *addr, u16_t port)
{
    u8 i, j, tmp;
    u8_t *buf;
    u16 m;

    for(i = 0; i < TCP_CLIENT_MAX_NUM; i++)
    {
        if( upcb->remote_port == tcp_client_list[i].dest_port )
        {
            for(j = 0; j < 4; j++ )
            {
                tmp = upcb->remote_ip.addr  >> ( 8 * j );
                if( tcp_client_list[i].dest_ip[j] != tmp )
                {
                    break;
                }
            }
            if( ( tcp_client_list[i].tcp_client_statue == CLIENT_CONNECT_OK ) && (j == 4) )
            {
                if( p->tot_len <= TCP_RECEV_BUFF_SIZE )
                {
                    buf = ((u8_t *)p->payload);
                    for(m = 0; m < p->tot_len; m++)
                    {
                        tcp_client_list[i].tcp_recev_buf[tcp_client_list[i].tcp_recev_len + m] = *(buf + m);
                    }
                }
                tcp_client_list[i].tcp_recev_len += m;
                tcp_client_list[i].tcp_client_statue = CLIENT_CONNECT_RECV;
            }
        }
    }
    /* free pbuf */
    pbuf_free(p);

    /* free the UDP connection, so we can accept new clients */
    //udp_remove(upcb);
}
/******************************************************************

******************************************************************/
void  udp_send_process(u8 udp_offset)
{
    struct pbuf *p;
    err_t err;
    static u16 s_tx_ok = 0;
    static u16 s_tx_fail = 0;

    /* allocate pbuf from pool*/
    p = pbuf_alloc(PBUF_TRANSPORT, tcp_client_list[udp_offset].tcp_send_len, PBUF_POOL);
    if (p != NULL)
    {
        /* copy data to pbuf */
        pbuf_take(p, &(tcp_client_list[udp_offset].tcp_send_buf[0]), tcp_client_list[udp_offset].tcp_send_len);
        /* send udp data */
        err = udp_send(tcp_client_list[udp_offset].echoclient_pcb, p);
        if(err == ERR_OK)
        {
            s_tx_ok++;
            if((s_tx_ok <= 3u) || ((s_tx_ok % 20u) == 0u))
            {
                rtt_print_blue("UDP tx ok n=%u len=%u\r\n",
                               (unsigned)s_tx_ok,
                               (unsigned)tcp_client_list[udp_offset].tcp_send_len);
            }
        }
        else
        {
            s_tx_fail++;
            rtt_print_red("UDP tx err=%d fail=%u\r\n", (int)err, (unsigned)s_tx_fail);
        }

        /* free pbuf */
        pbuf_free(p);
    }
    else
    {
        s_tx_fail++;
        rtt_print_red("UDP tx pbuf NULL fail=%u\r\n", (unsigned)s_tx_fail);
    }
}
/**********************************************************************

***********************************************************************/
void   udp_client_process(void)
{
    if(tcp_client_list[0].tcp_client_statue == CLIENT_CONNECT_RECV)
    {
        recv_message_from_sever(&(tcp_client_list[0].tcp_recev_buf[0]), &(tcp_client_list[0].tcp_recev_len));
        tcp_client_list[0].tcp_client_statue = CLIENT_CONNECT_OK;
    }

    if( tcp_client_list[0].connect_is_ok == 0 )
    {
        creat_udp_connect_process(0);
    }
    else if( tcp_client_list[0].tcp_client_statue == CLIENT_DIS_CONNECT )
    {
        udp_echoclient_connect(0);
        /* connect ??? tcp_send_en?????????? */
        if(tcp_client_list[0].tcp_send_en == 1)
        {
            udp_send_process(0);
            tcp_client_list[0].tcp_send_en = 0;
        }
    }
    else if(tcp_client_list[0].tcp_client_statue == CLIENT_CONNECT_OK )
    {
        if(tcp_client_list[0].tcp_send_en == 1)
        {
            udp_send_process(0);
            tcp_client_list[0].tcp_send_en = 0;
            heart_delay = HEART_DELAY;
        }
    }
}

/***********************************************************

***************************************************************/
void get_ip_para_process(u8 tcp_offset)
{

        tcp_client_list[tcp_offset].dest_port = netPcParaLocal.rNet[tcp_offset].destPort;

        tcp_client_list[tcp_offset].dest_ip[0] = (netPcParaLocal.rNet[tcp_offset].destIP >> 24) & 0xFF;
        tcp_client_list[tcp_offset].dest_ip[1] = (netPcParaLocal.rNet[tcp_offset].destIP >> 16) & 0xFF;
        tcp_client_list[tcp_offset].dest_ip[2] = (netPcParaLocal.rNet[tcp_offset].destIP >> 8) & 0xFF;
        tcp_client_list[tcp_offset].dest_ip[3] = (netPcParaLocal.rNet[tcp_offset].destIP >> 0) & 0xFF;
}

void  udp_send_message(u8 *p_data, u16 len)
{
    u16 i;
    
    if( tcp_client_list[0].tcp_client_statue == CLIENT_CONNECT_OK )
    {
        for(i = 0; i < len; i++)
        {
            tcp_client_list[0].tcp_send_buf[i] = *(p_data + i);
        }
        tcp_client_list[0].tcp_send_len = len;
        udp_send_process(0);
    }
}
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
