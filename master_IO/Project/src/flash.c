#include "flash.h"

//PC端网络参数
sNetPara_t netPcParaLocal;             //本地Ip
sNetPara_t netPcParaSet;               //将要设置的IP



//
u16 flashReadTmpBuff[NETUSERPARANUM];
//固件软件版本
sFlashVersion flashVersion;

//更新flash
u16 flashWriteUpload = 0;
//更新完以后重新启动网口
u16 flashResetNet = 0;




//flash读数据
void FlashReadContinue(uint32_t address, u16* out_point, u32 len)
{
	u16* source;
	u32 i;

	source = (u16*)(address);
	for (i = 0; i < len; i++)
	{
		*(out_point + i) = *(source + i);
	}
}

//flash写数据
void FlashWriteContinue(uint32_t address, u16* out_point, u32 len)
{
	u32 i = 0;

	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
	if (FLASH_ErasePage(address) == FLASH_COMPLETE)
	{
		for (i = 0; i < len; i++)
		{
			FLASH_ProgramHalfWord(address + 2 * i, *(out_point + i));
		}
	}
	FLASH_Lock();

}


/* 由 main.h 的 IP_ADDR/DEST_IP/GW/NETMASK 组装，与现场工程 LOCAL_IP_U32 写法一致 */
static void flash_apply_main_h_net_para(void)
{
	netPcParaLocal.localIp = ((IP_ADDR3) | (IP_ADDR2 << 8) | (IP_ADDR1 << 16) | (IP_ADDR0 << 24));
	netPcParaLocal.netMask = ((NETMASK_ADDR3) | (NETMASK_ADDR2 << 8) | (NETMASK_ADDR1 << 16) | (NETMASK_ADDR0 << 24));
	netPcParaLocal.gw = ((GW_ADDR3) | (GW_ADDR2 << 8) | (GW_ADDR1 << 16) | (GW_ADDR0 << 24));
	netPcParaLocal.connectNum = 1;
	netPcParaLocal.rNet[0].destIP = ((DEST_IP_ADDR3) | (DEST_IP_ADDR2 << 8) | (DEST_IP_ADDR1 << 16) | (DEST_IP_ADDR0 << 24));
	netPcParaLocal.rNet[0].localPort = DEST_PORT;
	netPcParaLocal.rNet[0].destPort = DEST_PORT;
}

//读取网络参数：先读 Boot Flash；USE_MAIN_H_IP 时再用 main.h 覆盖（LwIP 实际用 netPcParaLocal）
void flashReadNetPara(void)
{
	u16 temp = 0;
	u16 j = 0;
	u16 tembuf[40] = { 0 };
	u16 connnum = 0;
	u16 i = 0;

	temp = *(u16*)(NetUserParaAddress);

	//PC端网络地址
	if (temp == 0xAABB) {
		FlashReadContinue(NetIpStartAddress, (u16*)(&(tembuf[0])), (sizeof(sNetPara_t) / 2));
		//localIp
		netPcParaLocal.localIp = (tembuf[0]) | (tembuf[1] << 16);
		//netMask
		netPcParaLocal.netMask = (tembuf[2]) | (tembuf[3] << 16);
		//gw
		netPcParaLocal.gw = (tembuf[4]) | (tembuf[5] << 16);

		connnum = tembuf[6];
		i = 7;


		if ((connnum > 0) && (connnum <= 5)) {
			netPcParaLocal.connectNum = connnum;

			for (j = 0; j < connnum; j++) {
				netPcParaLocal.rNet[j].destIP = (tembuf[i] | (tembuf[i + 1] << 16));
				i += 2;
				netPcParaLocal.rNet[j].localPort = tembuf[i];
				i++;
				netPcParaLocal.rNet[j].destPort = tembuf[i];
				i++;
			}
		}
		else {
			flash_apply_main_h_net_para();
		}
	}
	else {
		flash_apply_main_h_net_para();
	}

#ifdef USE_MAIN_H_IP
	flash_apply_main_h_net_para();
#endif

	netPcParaSet = netPcParaLocal;
}

//不允许设置版本  读取版本参数
void flashReadVersionPara(void)
{
	u16 temp = 0;
	u16 tmpBuff[40] = { 0 };

	temp = *(u16*)(UserVersionFlag);

	if (temp == 0xAAAA) {
		FlashReadContinue(UserVersionPara, (u16*)(&(tmpBuff[0])), (sizeof(sFlashVersion) / 2));
		flashVersion.vendorID = tmpBuff[0];
		flashVersion.deviceID = tmpBuff[1];
		flashVersion.HwRevision = tmpBuff[2];
		flashVersion.SwRevision = tmpBuff[3];
		flashVersion.SwRevision = flashVersion.SwRevision | (tmpBuff[4] << 16);
		flashVersion.DeviceState = tmpBuff[5];
		flashVersion.serialNO = tmpBuff[6];
		flashVersion.serialNO = (flashVersion.serialNO) | (tmpBuff[7] << 16);
	}
	else {
		flashVersion.vendorID = 0x06F2;
		flashVersion.deviceID = (CARCONTFIRMWARE << 8);
		flashVersion.HwRevision = CARCONTSOFTWAREV2;
		flashVersion.SwRevision = 0;
		flashVersion.DeviceState &= ~(0x1);
		flashVersion.serialNO = 0;
	}
}

void flashWriteProcess(void)
{
	u16 j = 0;
	u16 i = 0;
	u16* temp = 0;


	FlashReadContinue(UserVersionPara, (u16*)(&flashReadTmpBuff), NETUSERPARANUM);

	temp = (u16*)(&flashVersion);

	flashReadTmpBuff[i] = 0xAAAA;
	i++;
	for (j = 0; j < UserVersionParalen; j++) {
		flashReadTmpBuff[i] = (temp[j]);
		i++;
	}

	flashReadTmpBuff[i] = 0xAABB;
	i++;
	temp = (u16*)(&netPcParaSet);

	for (j = 0; j < NetUserParaLen; j++) {
		flashReadTmpBuff[i] = (temp[j]);
		i++;
	}

	FlashWriteContinue(UserVersionFlag, flashReadTmpBuff, NETUSERPARANUM);

}

void flashUploadProcess(void)
{
	if (flashWriteUpload == 0) {
		return;
	}

	if (flashWriteUpload != 0) {
		flashWriteUpload--;
		if (flashWriteUpload == 0) {
			flashWriteProcess();
			flashReadNetPara();
			flashResetNet = NETRESETTIME;
			replyBroadInquireMsg(udpBoardSendBuf, &udpSendLen, REPLY_RECV_NET_SET_TYPE);
			udpSendProcess(udpBoardSendBuf, udpSendLen);
		}
	}
}

void flashResetProcess(void)
{
	if (flashResetNet == 0) {
		return;
	}

	if (flashResetNet != 0) {
		flashResetNet--;
		if (flashResetNet == 0) {
			NVIC_SystemReset();
		}
	}
}

void flashVersionInitCheck(void)
{
	u16 temp = 0;
	sFlashVersion flashInitVer;

	flashInitVer.vendorID = 0x06F2;
	flashInitVer.deviceID = (CARCONTFIRMWARE << 8);
	flashInitVer.HwRevision = CARCONTSOFTWAREV2;
	flashInitVer.SwRevision = 0;
	flashInitVer.DeviceState = IOTBOOTLAYER;
	flashInitVer.serialNO = 0;


	temp = *(u16*)(UserVersionFlag);
	if (temp != 0xAAAA) {
		flashVersion = flashInitVer;
		flashWriteProcess();

	}
	if (temp == 0xAAAA) {
		FlashReadContinue(UserVersionPara, (u16*)(&flashReadTmpBuff), NETUSERPARANUM);
		if ((flashReadTmpBuff[0] != 0x06F2) | ((flashReadTmpBuff[1] >> 8) != CARCONTFIRMWARE) |
			(flashReadTmpBuff[2] != IOTCONTSOFTWAREV2)) {
			flashVersion = flashInitVer;
			flashWriteProcess();
			flashReadNetPara();
			flashReadVersionPara();
		}
	}

}



