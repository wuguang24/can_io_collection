
#include "stm32f10x.h"
#include "i2c.h"
#include "aw_i2c.h"

#define AW_I2C_RETRIES 2
#define AW_I2C_RETRY_DELAY 1
#define AW_READ_CHIPID_RETRIES 2
#define AW_READ_CHIPID_RETRY_DELAY 1

//struct pinctrl *pinctrl_aw20036;
//struct pinctrl_state *aw20036_pins_cfg, *aw20036_rst_output0, *aw20036_rst_output1;
unsigned int aw20036debounce;

struct aw20036 aw1;
sAw20036_User_Cfg aw1_user;

/******************************************************
 *
 * aw20036 led parameter
 *
 ******************************************************/
#define AW20036_CFG_NAME_MAX        64
#define AW20036_IMAX_NAME_MAX       32

static char aw20036_imax_name[][AW20036_IMAX_NAME_MAX] = {
	{"AW20036_IMAX_10mA"},
	{"AW20036_IMAX_20mA"},
	{"AW20036_IMAX_30mA"},
	{"AW20036_IMAX_40mA"},
	{"AW20036_IMAX_60mA"},
	{"AW20036_IMAX_80mA"},
	{"AW20036_IMAX_120mA"},
	{"AW20036_IMAX_160mA"},
	{"AW20036_IMAX_3P3mA"},
	{"AW20036_IMAX_6P7mA"},
	{"AW20036_IMAX_10P0mA"},
	{"AW20036_IMAX_13P3mA"},
	{"AW20036_IMAX_20P0mA"},
	{"AW20036_IMAX_26P7mA"},
	{"AW20036_IMAX_40P0mA"},
	{"AW20036_IMAX_53P3mA"},
};

void aw2006_para_Init(void)
{
    struct aw20036 *aw20036 = &aw1;
    aw20036->imax = AW20036_IMAX_10mA;
    aw20036->rgbcolor = 0x3f;           //page1用来配置颜色值,范围0x00-0x3f
    aw20036->rgbbrightness = 0x3F;       //page2用来配置亮度值，范围0x00-0xff
}

/*
*********************************************************************************************************
*    函 数 名: i2c_write_byte_data
*    功能说明: 向指定地址写入若干数据
*    形    参：_usAddress : 起始地址
*             _usSize : 数据长度，单位为字节
*             _pWriteBuf : 存放读到的数据的缓冲区指针
*    返 回 值: 0 表示失败，1表示成功
*********************************************************************************************************
*/
    
u8 i2c_write_byte_data(uint16_t _usAddress, u8 *_pWriteBuf)
{
    uint16_t i,m;
    uint16_t usAddr;

    usAddr = _usAddress;    

    //　第０步：发停止信号，启动内部写操作　
    i2c_Stop();//1delay
    
    //通过检查器件应答的方式，判断内部写操作是否完成, 一般小于 10ms CLK频率为200KHz时，查询次数为30次左右
    for (m = 0; m < 1000; m++)
    {                
        //第1步：发起I2C总线启动信号 
        i2c_Start();//3delay
        
        //第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 
        i2c_SendByte(AW20036_ADDR | I2C_WR);    //此处是写指令//3*8delay
        
        //第3步：发送一个时钟，判断器件是否正确应答 ,0应答成功
        if (i2c_WaitAck() == 0)//3delay
        {
            break;
        }
    }
    if (m  == 1000)
    {
        goto cmd_fail;    //写超时
    }

    i2c_SendByte((u8)usAddr);//3*8delay
    
    //第5步：等待ACK 
    if (i2c_WaitAck() != 0)//3delay
    {
        goto cmd_fail;    //器件无应答 
    }
    
    //第6步：开始写入数据 
    i2c_SendByte(_pWriteBuf[0]);//3*8delay

    //第7步：发送ACK 
    if (i2c_WaitAck() != 0)//3delay
    {
        goto cmd_fail;    //器件无应答 
    }

    //命令执行成功，发送I2C总线停止信号 
    i2c_Stop();//1delay
    return 1;

cmd_fail: //命令执行失败后，切记发送停止信号，避免影响I2C总线上其他设备 
    //发送I2C总线停止信号 
    i2c_Stop();
    return 0;
}


/*
*********************************************************************************************************
*	函 数 名: i2c_write_byte_data
*	功能说明: 从指定地址处开始读取若干数据
*	形    参：_usAddress : 起始地址
*			 _usSize : 数据长度，单位为字节
*			 _pReadBuf : 存放读到的数据的缓冲区指针
*	返 回 值: 0 表示失败，1表示成功
*********************************************************************************************************
*/
u8 i2c_read_byte_data(uint16_t _usAddress, uint16_t _usSize, u8 *_pReadBuf)
{
	uint16_t i;

	//第1步：发起I2C总线启动信号
	i2c_Start();
	
	//第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读
	i2c_SendByte(AW20036_ADDR | I2C_WR);	//此处是写指令
	 
	//第3步：等待ACK
	if (i2c_WaitAck() != 0)
	{
		goto cmd_fail;	//器件无应答
	}

	//第4步：发送字节地址
	i2c_SendByte((u8)_usAddress);
	
	//第5步：等待ACK
	if (i2c_WaitAck() != 0)
	{
		goto cmd_fail;	//器件无应答
	}
	
	//第6步：重新启动I2C总线。前面的代码的目的向传送地址，下面开始读取数据
	i2c_Start();
	
	//第7步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读
	i2c_SendByte(AW20036_ADDR | I2C_RD);	//此处是读指令
	
	//第8步：发送ACK
	if (i2c_WaitAck() != 0)
	{
		goto cmd_fail;	//器件无应答
	}	
	
	//第9步：循环读取数据
	for (i = 0; i < _usSize; i++)
	{
		_pReadBuf[i] = i2c_ReadByte();	//读1个字节
		
		//每读完1个字节后，需要发送Ack， 最后一个字节不需要Ack，发Nack
		if (i != _usSize - 1)
		{
			i2c_Ack();	//中间字节读完后，CPU产生ACK信号(驱动SDA = 0)
		}
		else
		{
			i2c_NAck();	//最后1个字节读完后，CPU产生NACK信号(驱动SDA = 1)
		}
	}
	//发送I2C总线停止信号
	i2c_Stop();
	return 1;	//执行成功

cmd_fail: //命令执行失败后，切记发送停止信号，避免影响I2C总线上其他设备
	//发送I2C总线停止信号
	i2c_Stop();
	return 0;
}
/*
 *****************************************************
 *
 * aw20036 i2c write/read
 *
 *****************************************************
 */
static int
aw20036_i2c_write(struct aw20036 *aw20036, unsigned char reg_addr, unsigned char reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_write_byte_data(reg_addr, &reg_data);
		if (ret < 0){
			//pr_err("%s: i2c_write cnt=%d error=%d\n", __func__, cnt, ret);
		}else
			break;

		cnt++;
		//usleep_range(1000, 2000);
	}

	return ret;
}

static int
aw20036_i2c_read(struct aw20036 *aw20036, unsigned char reg_addr, unsigned char *reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_read_byte_data(reg_addr, 1, reg_data);
		if (ret < 0) {
			//pr_err("%s: i2c_read cnt=%d error=%d\n", __func__, cnt, ret);
		} else {
			*reg_data = ret;
			break;
		}
		cnt++;
		usleep_range(1000, 2000);
	}

	return ret;
}

static int aw20036_i2c_write_bits(struct aw20036 *aw20036,
				  unsigned char reg_addr, unsigned int mask,
				  unsigned char reg_data)
{
	unsigned char reg_val;

	aw20036_i2c_read(aw20036, reg_addr, &reg_val);
	reg_val &= mask;
	reg_val |= (reg_data & (~mask));
	aw20036_i2c_write(aw20036, reg_addr, reg_val);

	return 0;
}


/*
 *****************************************************
 *
 * led class dev
 *
 ******************************************************
 */
void aw20036_led_init(struct aw20036 *aw20036)
{
    u8 i;

    //ALL leds常亮模式
    aw20036_i2c_write(aw20036, REG_PAGE, AW20036_REG_PAGE0);//切换page进行配置
    aw20036_i2c_write(aw20036, REG_SWRST, 0x01);//软复位控制
    usleep_range(2000, 3000);
    aw20036_i2c_write(aw20036, REG_GCCR, AW20036_IMAX_53P3mA);
    aw20036_i2c_write(aw20036, REG_SIZE, 0x3F);//配置 SIZE，C1/C2/C3
    aw20036_i2c_write(aw20036, REG_WORK_MODE, 0x00);//芯片配置到active模式
    aw20036_i2c_write(aw20036, REG_PAGE, AW20036_REG_PAGE1);//切换到page1  
    for (i = 0; i < 0x23; i++)    
    aw20036_i2c_write(aw20036, 0x00+i, aw20036->rgbcolor);
    aw20036_i2c_write(aw20036, REG_PAGE, AW20036_REG_PAGE2);//切换到page2   
    for (i = 0; i < 0x23; i++)    
    aw20036_i2c_write(aw20036, 0x00+i, aw20036->rgbbrightness);

    Delay(3000);
    aw20036_i2c_write(aw20036, REG_PAGE, AW20036_REG_PAGE0);
    aw20036_i2c_write(aw20036, REG_GCCR, aw20036->imax);
    
    
    aw20036_i2c_write(&aw1, REG_PAGE, AW20036_REG_PAGE0);//切换page进行配置
}

//aw
void aw20036_led_one_ctrl(struct aw20036 *aw20036, u8 index, BitAction BitVal)
{
    u8 integer,remainder;
    u32 on_num = 0;

    if (index >= 36)return;

    integer = index /6;
    remainder = index %6;

    aw20036_i2c_write(aw20036, REG_PAGE, AW20036_REG_PAGE0);//切换page进行配置

    
    on_num |= (0x01 <<index);
    aw20036_i2c_write(aw20036, REG_LEDON0, on_num &0x3F);
    aw20036_i2c_write(aw20036, REG_LEDON1, (on_num >>6) &0x3F);
    aw20036_i2c_write(aw20036, REG_LEDON2, (on_num >>12) &0x3F);
    aw20036_i2c_write(aw20036, REG_LEDON3, (on_num >>18) &0x3F);
    aw20036_i2c_write(aw20036, REG_LEDON4, (on_num >>24) &0x3F);
    aw20036_i2c_write(aw20036, REG_LEDON5, (on_num >>30) &0x3F);
}


void aw20036_poll_test(void)
{
    static u8 test_led_num = 0;
    static u8 test_uart_send = 0;
    static u8 sec10 = 0;
    
    test_led_num++;
    if (test_led_num == 4)//C1R1-C1R4
        test_led_num = 12;//C2R1
    if (test_led_num == 20)
        test_led_num = 24;//C3R1
    if (test_led_num == 32){
        aw20036_led_init(&aw1);
        test_led_num = 0;
    }

    test_uart_send++;
    if(test_uart_send %1 == 0)
    {
        test_uart_send = 0;
        uart_send_test();
    }
    
    //out -test
    if(sec10 <= 8)
    {
        for(u8 i = 0; i<8; i++)
        {
            if (sec10 == i)
            {
                LightOutPut(i,Bit_SET);
            }else{
                LightOutPut(i,Bit_RESET);
            }
        }
    }
    
    sec10++;
    if (sec10 == 9)
    {
        for(u8 i = 0; i<8; i++)
        {
            LightOutPut(i,Bit_SET);
        }
    }
    
    if (sec10 == 10)
    {
        for(u8 i = 0; i<8; i++)
        {
            LightOutPut(i,Bit_RESET);
        }
    }
    if (sec10 == 11)
    {
        sec10 = 0;
    }
        
        
}

void aw20036_led_all_ctrl(void)
{
    static u8 cur_led[6] = {0};
    static u8 next_timing = 10;
    static u8 index = 1;
    u8 tmp,tmp1,tmp2,tmp3,tmp4;

    tmp1 = aw1_user.led_statue;
    tmp2 = aw1_user.led_uart_can[0];
    tmp3 = aw1_user.led_uart_can[1];
    tmp4 = aw1_user.led_uart_can[2];
    cur_led[0] = ((tmp4 <<3) |(tmp3 <<2) |(tmp2 <<1) | tmp1)&0xFF;

    cur_led[2] = aw1_user.led_in &0x3F;
    cur_led[3] = (aw1_user.led_in >>6) &0x03;
    cur_led[4] = aw1_user.led_out &0x3F;
    cur_led[5] = (aw1_user.led_out >>6) &0x03;

    if (next_timing > 0)
    {
        next_timing--;
    }else{
        next_timing = 10;
        switch (index)
        {
        case 1:
            aw20036_i2c_write(&aw1, REG_LEDON0, cur_led[0]);
            break;
        case 2:
            aw20036_i2c_write(&aw1, REG_LEDON2, cur_led[2]);
            break;
        case 3:
            aw20036_i2c_write(&aw1, REG_LEDON3, cur_led[3]);
            break;
        case 4:
            aw20036_i2c_write(&aw1, REG_LEDON4, cur_led[4]);
            break;
        case 5:
            aw20036_i2c_write(&aw1, REG_LEDON5, cur_led[5]);
            break;
        default:
            break;
        }
        index++;
        if (index > 5)
        {
            index = 1;
        }
        
    }

}