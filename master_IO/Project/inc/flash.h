#ifndef _FLASH_H
#define _FLASH_H

#include "main.h"

#define BOARDIPPORT                       3030

//不同boot程序需要选择设置合适的分区

#define UserVersionFlag                   0x800F000  //版本标志
#define UserVersionPara                   (UserVersionFlag + 2) //版本详细信息  
#define NetUserParaAddress                (UserVersionFlag + 18)  //网络标志区
#define NetIpStartAddress                 (UserVersionFlag + 20) //PC端网络详细数据起始区

#define UserFlagLen                       1
#define UserVersionParalen                8
#define NetUserParaLen                    27

#define NETUPLOADTIME                     30
#define NETRESETTIME                      30

#define NETCONNECTMAX                     5


//boot层
#define  IOTBOOTLAYER      0x0
//APP层
#define  IOTAPPLAYER       0x1

//firmware 固件版本

#define  CARCONTFIRMWARE   0x1          //车载控制器
#define  SPEEDCONTFIRMWARE 0x2          //速度控制器
#define  IOCONTFIRMWARE    0x3          //下料口控制器
#define  LOADCONTFIRMWARE  0x4          //供包台控制器
#define  BALANCONTFIRMWARE 0x5          //摆轮控制器
#define  STCONTFIRMWARE    0x6          //ST控制器
#define  IOTCONTFIRMWARE   0X7          //IOT控制器固件版本




//software 软件版本
#define  IOTCONTSOFTWAREV2   0x2        //IOT控制器软件版本V1
#define  CARCONTSOFTWAREV2   0x2        //车载控制器软件版本V1


#define  NETUSERPARANUM    200


#pragma pack (1) 


// boot app

typedef struct {
	u32 destIP;       //远程IP
	u16 localPort;    //本地端口
	u16 destPort;     //目的端口
}sRemoteNetPara;

//标志位0xaaaa
typedef struct {
	u32  localIp;                              //本地端口
	u32  netMask;                              //子网掩码
	u32  gw;                                   //网关 
	u16  connectNum;                           //连接数量
	sRemoteNetPara rNet[NETCONNECTMAX];        //远程Ip信息
}sNetPara_t;


//标志位0xBBBB
typedef struct {
	u16 vendorID;          //厂商标识   0x06F2
	u16 deviceID;          //设备标识  //bit8-15 主类别 bit0-7子类别
	u16 HwRevision;        //硬件版本
	u32 SwRevision;        //软件版本
	u16 DeviceState;       //bit0 0 boot状态 bit1 APP状态
	u32 serialNO;          //设备序列号
}sFlashVersion;

#pragma pack () 

//PC端网络参数
extern sNetPara_t netPcParaLocal;             //本地Ip
extern sNetPara_t netPcParaSet;               //将要设置的IP

extern sFlashVersion flashVersion;

extern u16 flashReadTmpBuff[];
extern u16 flashWriteTmpBuff[];
//更新flash
extern u16 flashWriteUpload;
//更新完以后重新启动网口
extern u16 flashResetNet;



//写参数
void flashWriteProcess(void);

//更新参数 1ms执行一次
void flashUploadProcess(void);
void flashResetProcess(void);

//初始化参数
void flashVersionInitCheck(void);

//读取网络参数  需要初始化时执行
void flashReadNetPara(void);
//不允许设置版本  读取版本参数
void flashReadVersionPara(void);

#endif