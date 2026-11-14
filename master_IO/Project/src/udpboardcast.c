#include "udpboardcast.h"
#include "flash.h"


#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/udp.h"
#include "lwip/opt.h"

//广播内容
u8 updBoardRecvBuf[200];
u8 udpBoardSendBuf[200] = { 0 };
u16 udpSendLen = 0;
u16 udpRecvLen = 0;
struct udp_pcb* boardPcb;
u32 udpReplyFrame = 0;

uint8_t mac_addr[6];//={0,0,0,0,0,1};


void copyIpSetPara(sNetPara_t* net, u8* buff)
{
    u16 i = 0;
    u16* recvtemp = NULL;
    u16* nettemp = NULL;
    recvtemp = (u16*)(buff);
    nettemp = (u16*)(net);

    for (i = 0; i < NetUserParaLen; i++) {
        nettemp[i] = recvtemp[i];
    }
}

//回复广播查询IP地址
void replyBroadInquireMsg(u8* buf, u16* len, u16 type)
{
    u8 i;
    u8 sum = 0;
    u16* temp = 0;
    u16 j = 0;
    u16 num = 0;

    *len = 71;


    buf[0] = 0xAA;
    buf[1] = 0xAA;
    buf[2] = udpReplyFrame & 0xFF;
    buf[3] = (udpReplyFrame >> 8) & 0xFF;
    buf[4] = (udpReplyFrame >> 16) & 0xFF;
    buf[5] = (udpReplyFrame >> 24) & 0xFF;
    buf[9] = type & 0xFF;
    buf[10] = (type >> 8) & 0xFF;

    buf[11] = flashVersion.vendorID & 0xFF;
    buf[12] = (flashVersion.vendorID >> 8) & 0xFF;
    buf[13] = flashVersion.deviceID & 0xFF;
    buf[14] = (flashVersion.deviceID >> 8) & 0xFF;
    buf[15] = flashVersion.HwRevision & 0xFF;
    buf[16] = (flashVersion.HwRevision >> 8) & 0xFF;
    buf[17] = flashVersion.SwRevision & 0xFF;
    buf[18] = (flashVersion.SwRevision >> 8) & 0xFF;
    buf[19] = (flashVersion.SwRevision >> 16) & 0xFF;
    buf[20] = (flashVersion.SwRevision >> 24) & 0xFF;
    buf[21] = flashVersion.DeviceState & 0xFF;
    buf[22] = (flashVersion.DeviceState >> 8) & 0xFF;
    buf[23] = flashVersion.serialNO & 0xFF;
    buf[24] = (flashVersion.serialNO >> 8) & 0xFF;
    buf[25] = (flashVersion.serialNO >> 16) & 0xFF;
    buf[26] = (flashVersion.serialNO >> 24) & 0xFF;

    buf[27] = mac_addr[0];
    buf[28] = mac_addr[1];
    buf[29] = mac_addr[2];
    buf[30] = mac_addr[3];
    buf[31] = mac_addr[4];
    buf[32] = mac_addr[5];

    num = 7 + netPcParaLocal.connectNum * 4;

    temp = (u16*)(&(netPcParaLocal));
    j = 33;

    for (i = 0; i < num; i++) {
        buf[j] = *(temp + i) & 0xFF;
        j++;
        buf[j] = ((*(temp + i)) >> 8) & 0xFF;
        j++;
    }

    *len = 33 + num * 2;
    buf[6] = (*len) & 0xFF;
    buf[7] = ((*len) >> 8) & 0xFF;

    sum = buf[9];
    for (i = 1; i < (*len) - 9; i++)
    {
        sum ^= buf[9 + i];
    }
    buf[8] = sum;
}


//接收设置的IP地址
void recvSetIpMsg(u8* buf, u16* len)
{
    u16 i = 0;
    u8 mactemp[6] = { 0 };



    for (i = 0; i < 6; i++) {
        mactemp[i] = buf[i];
    }

    if ((mactemp[0] == 0) && (mactemp[1] == 0) && (mactemp[2] == 0) &&
        (mactemp[3] == 0) && (mactemp[4] == 0) && (mactemp[5] == 0)) {
        copyIpSetPara(&(netPcParaSet), &(buf[6]));
        flashWriteUpload = NETUPLOADTIME;
    }

    if ((mactemp[0] == mac_addr[0]) && (mactemp[1] == mac_addr[1]) && (mactemp[2] == mac_addr[2]) &&
        (mactemp[3] == mac_addr[3]) && (mactemp[4] == mac_addr[4]) && (mactemp[5] == mac_addr[5])) {
        copyIpSetPara(&(netPcParaSet), &(buf[6]));
        flashWriteUpload = NETUPLOADTIME;
    }
}


void udpSendProcess(u8* buff, u16 len)
{
    struct ip_addr destAddr; /* 获取远程主机 IP地址*/
    struct pbuf* ps;


    IP4_ADDR(&destAddr, 255, 255, 255, 255);

    ps = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_POOL);
    if (ps != NULL)
    {
        /* copy data to pbuf */
        pbuf_take(ps, buff, len);
        /* send udp data */
        udp_sendto(boardPcb, ps, &destAddr, BOARDIPPORT + 1); /*发送数据出去 */
        //            udp_send(g_pcb, p);

        pbuf_free(ps);
    }
}



void udp_server_recv(void* arg, struct udp_pcb* pcb, struct pbuf* p,
    struct ip_addr* addr, u16_t port)
{
    u8 destip[4];
    u16 len = 0;
    u16 i = 0;
    u16 type = 0;
    u8* tmp = 0;

    if (recv_msg_check(p->payload, p->len) == 0)
    {
        p->len = 0;
        return;
    }

    if (p->len >= 200) {
        return;
    }

    tmp = (u8*)(p->payload);
    udpRecvLen = p->len;

    for (i = 0; i < p->len; i++) {
        updBoardRecvBuf[i] = tmp[i];

    }
    udpReplyFrame = (updBoardRecvBuf[2]) | (updBoardRecvBuf[3] << 8) |
        (updBoardRecvBuf[4] << 16) | (updBoardRecvBuf[5] << 24);
    type = (updBoardRecvBuf[9]) | (updBoardRecvBuf[10] << 8);
    pbuf_free(p);

    switch (type) {
    case RECV_NET_PARA_TYPE:
        replyBroadInquireMsg(udpBoardSendBuf, &udpSendLen, REPLY_RECV_NET_PARA_TYPE);
        udpSendProcess(udpBoardSendBuf, udpSendLen);
        break;
    case RECV_NET_SET_TYPE:
        recvSetIpMsg(&(updBoardRecvBuf[11]), &udpRecvLen);
        break;
    default:
        break;
    }
}

void UDP_server_init(void)
{
    boardPcb = udp_new(); //申请udp控制块
    udp_bind(boardPcb, IP_ADDR_ANY, 3030);
    udp_recv(boardPcb, udp_server_recv, NULL);  /* 设置UDP段到时的回调函数 */
}