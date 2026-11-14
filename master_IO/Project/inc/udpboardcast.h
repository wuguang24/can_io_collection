#ifndef _UDPBOARDCAST_H
#define _UDPBOARDCAST_H

#include "main.h"

#define     RECV_NET_PARA_TYPE                      0x2F01  //广播查询IP
#define     RECV_NET_SET_TYPE                       0x2F02  //广播设置IP


#define     REPLY_RECV_NET_PARA_TYPE                0xAF01 //广播回复IP查询
#define     REPLY_RECV_NET_SET_TYPE                 0xAF02 //广播回复设置IP

extern  uint8_t mac_addr[];

extern u8 updBoardRecvBuf[];
extern u8 udpBoardSendBuf[];
extern u16 udpSendLen;
extern u16 udpRecvLen;

//回复广播查询IP地址
void replyBroadInquireMsg(u8* buf, u16* len, u16 type);
void udpSendProcess(u8* buff, u16 len);
//需要初始化
void UDP_server_init(void);

#endif