/*---------------------------------------------------------------------*/
/* --- STC MCU Limited ------------------------------------------------*/
/* --- STC 1T Series MCU Demo Programme -------------------------------*/
/* --- Mobile: (86)13922805190 ----------------------------------------*/
/* --- Fax: 86-0513-55012956,55012947,55012969 ------------------------*/
/* --- Tel: 86-0513-55012928,55012929,55012966 ------------------------*/
/* --- Web: www.STCMCU.com --------------------------------------------*/
/* --- Web: www.STCMCUDATA.com  ---------------------------------------*/
/* --- QQ:  800003751 -------------------------------------------------*/
/* 如果要在程序中使用此代码,请在程序中注明使用了STC的资料及程序        */
/*---------------------------------------------------------------------*/


/*************  功能说明    **************

本例程基于STC8H8K64U为主控芯片的实验箱8进行编写测试，STC8G、STC8H系列芯片可通用参考.

通过UART接口连接LIN收发器实现LIN总线信号收发测试例程.

UART1通过串口工具连接电脑.

UART2外接LIN收发器(TJA1020/1), 连接LIN总线.

将电脑串口发送的数据转发到LIN总线; 从LIN总线接收到的数据转发到电脑串口.

默认传输速率：9600波特率，发送LIN数据前切换波特率，发送13个显性间隔信号.

下载时, 选择时钟 22.1184MHz (用户可自行修改频率).

******************************************/

#include    "reg51.h"       //包含此头文件后，里面声明的寄存器不需要再手动输入，避免重复定义
#include    "intrins.h"

#define MAIN_Fosc       22118400L   //定义主时钟（精确计算波特率）

typedef     unsigned char   u8;
typedef     unsigned int    u16;
typedef     unsigned long   u32;

//手动输入声明"reg51.h"头文件里面没有定义的寄存器
sfr AUXR = 0x8E;
sfr S2CON = 0x9A;   //
sfr S2BUF = 0x9B;   //
sfr TH2  = 0xD6;
sfr TL2  = 0xD7;
sfr IE2   = 0xAF;
sfr INT_CLKO = 0x8F;
sfr P_SW1 = 0xA2;
sfr P_SW2 = 0xBA;

sfr P4   = 0xC0;
sfr P5   = 0xC8;
sfr P6   = 0xE8;
sfr P7   = 0xF8;
sfr P1M1 = 0x91;    //PxM1.n,PxM0.n     =00--->Standard,    01--->push-pull
sfr P1M0 = 0x92;    //                  =10--->pure input,  11--->open drain
sfr P0M1 = 0x93;
sfr P0M0 = 0x94;
sfr P2M1 = 0x95;
sfr P2M0 = 0x96;
sfr P3M1 = 0xB1;
sfr P3M0 = 0xB2;
sfr P4M1 = 0xB3;
sfr P4M0 = 0xB4;
sfr P5M1 = 0xC9;
sfr P5M0 = 0xCA;
sfr P6M1 = 0xCB;
sfr P6M0 = 0xCC;
sfr P7M1 = 0xE1;
sfr P7M0 = 0xE2;

sbit P00 = P0^0;
sbit P01 = P0^1;
sbit P02 = P0^2;
sbit P03 = P0^3;
sbit P04 = P0^4;
sbit P05 = P0^5;
sbit P06 = P0^6;
sbit P07 = P0^7;
sbit P10 = P1^0;
sbit P11 = P1^1;
sbit P12 = P1^2;
sbit P13 = P1^3;
sbit P14 = P1^4;
sbit P15 = P1^5;
sbit P16 = P1^6;
sbit P17 = P1^7;
sbit P20 = P2^0;
sbit P21 = P2^1;
sbit P22 = P2^2;
sbit P23 = P2^3;
sbit P24 = P2^4;
sbit P25 = P2^5;
sbit P26 = P2^6;
sbit P27 = P2^7;
sbit P30 = P3^0;
sbit P31 = P3^1;
sbit P32 = P3^2;
sbit P33 = P3^3;
sbit P34 = P3^4;
sbit P35 = P3^5;
sbit P36 = P3^6;
sbit P37 = P3^7;
sbit P40 = P4^0;
sbit P41 = P4^1;
sbit P42 = P4^2;
sbit P43 = P4^3;
sbit P44 = P4^4;
sbit P45 = P4^5;
sbit P46 = P4^6;
sbit P47 = P4^7;
sbit P50 = P5^0;
sbit P51 = P5^1;
sbit P52 = P5^2;
sbit P53 = P5^3;
sbit P54 = P5^4;
sbit P55 = P5^5;
sbit P56 = P5^6;
sbit P57 = P5^7;

sbit SLP_N  = P2^4;     //0: Sleep

// Serial Transmission Protocol
typedef struct {
    u8 header;
    u8 sensor_id;
    u8 checksum;
} Recv_t;

typedef struct {
    u8 header;
    unsigned int sensor_data;
    u8 checksum;
} Send_t;

/****************************** 用户定义宏 ***********************************/

#define Baudrate1           (65536UL - (MAIN_Fosc / 4) / 9600UL)
#define Baudrate2           (65536UL - (MAIN_Fosc / 4) / 9600UL)  //发送数据传输波特率

#define Baudrate_Break      (65536UL - (MAIN_Fosc / 4) / 6647UL)  //发送显性间隔信号波特率

#define UART1_BUF_LENGTH    32
#define UART2_BUF_LENGTH    32

#define LIN_ID    0x31

u8  TX1_Cnt;    //发送计数
u8  RX1_Cnt;    //接收计数
u8  TX2_Cnt;    //发送计数
u8  RX2_Cnt;    //接收计数
bit B_TX1_Busy; //发送忙标志
bit B_TX2_Busy; //发送忙标志
u8  RX1_TimeOut;
u8  RX2_TimeOut;

u8  xdata RX1_Buffer[UART1_BUF_LENGTH]; //接收缓冲
u8  xdata RX2_Buffer[UART2_BUF_LENGTH]; //接收缓冲

Recv_t xdata recv_data;
Send_t xdata send_data;

// void UART1_config(u8 brt);   // 选择波特率, 2: 使用Timer2做波特率, 其它值: 使用Timer1做波特率.
void UART2_config(u8 brt);   // 选择波特率, 2: 使用Timer2做波特率, 其它值: 无效.
void PrintString1(u8 *puts);
void delay_ms(u8 ms);
// void UART1_TxByte(u8 dat);
void UART2_TxByte(u8 dat);
void Lin_Send(u8 *puts);
void SetTimer2Baudraye(u16 dat);

u8 check_sum(int Mode, u8* data, size_t len) {
    unsigned char sum = 0, k;

    if (Mode == 1)  // Send
    for (k = 0; k < len; k++) {
        sum = sum ^ data[k];
    }

    if (Mode == 0)  // Recv
    for (k = 0; k < len; k++) {
        sum = sum ^ data[k];
    }
    return sum;
}

u8 recv_sensor_id(Recv_t* recv_data, u8* data)
{
    recv_data->header = data[0];
    recv_data->sensor_id = data[1];
    recv_data->checksum = data[2];
    if(recv_data->checksum == check_sum(0, data, 2))
    {
        return recv_data->sensor_id
    }
}

void send_sensor_value(unsigned int data, Send_t* send_data)
{
    send_data->header = 0x55;
    send_data->sensor_data = data;
    send_data->checksum = check_sum(1, &send_data->sensor_data, sizeof(send_data) - 1);
    memcpy(&RX2_Buffer, &send_data, sizeof(send_data));

    for (i=0; i < sizeof(send_data); i++)
        UART1_TxByte(RX2_Buffer[i]);
}

void set_pins_1(u8 number)
{
    // Clear bits 2, 3, 4, 5 in P1
    P1 &= ~(0x3C);

    // Set bits 2, 3, 4, 5 in P1 according to the number
    P1 |= (number << 2) & 0x3C;
}

void set_pins_2(u8 number)
{
    // Clear bits 3, 4, 5, 6 in P3
    P3 &= ~(0x78);

    // Set bits 3, 4, 5, 6 in P3 according to the number
    P3 |= (number << 3) & 0x78;
}

unsigned int read_adc_value(u8 channel)
{
    // Select ADC channel
    ADC_CONTR = (ADC_CONTR & 0xF8) | (channel & 0x07);

    // Start ADC conversion
    ADC_CONTR |= 0x08;

    // Wait for ADC conversion to complete
    while (!(ADC_CONTR & 0x10));

    // Clear ADC completion flag
    ADC_CONTR &= ~0x10;

    // Read ADC result
    return (ADC_RES << 2) | (ADC_RESL & 0x03);
}

unsigned int get_sensor_value(u8 sensor_id)
{
    if(sensor_id < 16){
        set_pins_1(sensor_id);
        return read_adc_value(6);
    } else {
        set_pins_2(sensor_id);
        return read_adc_value(7);
    }
}

void configure_pins(void)
{
    // Set P1.2, P1.3, P1.4, P1.5 to digital output
    P1M1 &= ~(0x3C); // Clear bits 2, 3, 4, 5 in P1M1
    P1M0 |= 0x3C;    // Set bits 2, 3, 4, 5 in P1M0

    // Set P3.3, P3.4, P3.5, P3.6 to digital output
    P3M1 &= ~(0x78); // Clear bits 3, 4, 5, 6 in P3M1
    P3M0 |= 0x78;    // Set bits 3, 4, 5, 6 in P3M0

    // Set P1.6, P1.7 to analog input
    P1M1 |= 0xC0;    // Set bits 6, 7 in P1M1
    P1M0 &= ~(0xC0); // Clear bits 6, 7 in P1M0

    // Disable digital function of P1.6, P1.7
    ADCCFG |= 0xC0;  // Set bits 6, 7 in ADCCFG

    // Configure ADC
    ADC_CONTR = 0x80; // Enable ADC and set ADC power on
    ADC_RES = 0;      // Clear ADC result register
    ADC_RESL = 0;     // Clear ADC result low byte register
}

//========================================================================
// 函数: void main(void)
// 描述: 主函数。
// 参数: none.
// 返回: none.
// 版本: VER1.0
// 日期: 2014-11-28
// 备注: 
//========================================================================
void main(void)
{
    u8 i;

    // P0M1 = 0x00;   P0M0 = 0x00;   //设置为准双向口
    // P1M1 = 0x00;   P1M0 = 0x00;   //设置为准双向口
    // P2M1 = 0x00;   P2M0 = 0x00;   //设置为准双向口
    // P3M1 = 0x00;   P3M0 = 0x00;   //设置为准双向口
    // P4M1 = 0x00;   P4M0 = 0x00;   //设置为准双向口
    // P5M1 = 0x00;   P5M0 = 0x00;   //设置为准双向口
    // P6M1 = 0x00;   P6M0 = 0x00;   //设置为准双向口
    // P7M1 = 0x00;   P7M0 = 0x00;   //设置为准双向口

    configure_pins();

    // UART1_config(1);    // 选择波特率, 2: 使用Timer2做波特率, 其它值: 使用Timer1做波特率.
    UART2_config(2);    // 选择波特率, 2: 使用Timer2做波特率, 其它值: 无效.
    EA = 1;             //允许全局中断
    SLP_N = 1;

    // PrintString1("STC8H8K64U UART1 Test Programme!\r\n");  //UART1发送一个字符串
    // PrintString1("STC8 Started\r\n");

    while (1)
    {
        delay_ms(1);
        // if(RX1_TimeOut > 0)
        // {
        //     if(--RX1_TimeOut == 0)  //超时,则串口接收结束
        //     {
        //         if(RX1_Cnt > 0)
        //         {
        //             recv(&recv_data, RX1_Buffer);
		// 			// Lin_Send(RX1_Buffer);  //将UART1收到的数据发送到LIN总线上
        //         }
        //         RX1_Cnt = 0;
        //     }
        // }

        if(RX2_TimeOut > 0)
        {
            if(--RX2_TimeOut == 0)  //超时,则串口接收结束
            {
                if(RX2_Cnt > 0)
                {
                    u8 sensor_id = recv_sensor_id(&recv_data, RX2_Buffer);
                    unsigned int sensor_value = get_sensor_value(sensor_id);
                    send_sensor_value(sensor_value, &send_data);
					// Lin_Send(RX1_Buffer);  //将UART1收到的数据发送到LIN总线上
                }
                RX2_Cnt = 0;
            }
        }
				
        // if(RX2_TimeOut > 0)
        // {
        //     if(--RX2_TimeOut == 0)  //超时,则串口接收结束
        //     {
        //         if(RX2_Cnt > 0)
        //         {
        //             for (i=0; i < RX2_Cnt; i++)     //遇到停止符0结束
        //             {
		// 				UART1_TxByte(RX2_Buffer[i]);  //从LIN总线收到的数据发送到UART1
        //             }
        //         }
        //         RX2_Cnt = 0;
        //     }
        // }
    }
}


//========================================================================
// 函数: void delay_ms(unsigned char ms)
// 描述: 延时函数。
// 参数: ms,要延时的ms数, 这里只支持1~255ms. 自动适应主时钟.
// 返回: none.
// 版本: VER1.0
// 日期: 2013-4-1
// 备注: 
//========================================================================
void delay_ms(u8 ms)
{
    u16 i;
    do{
        i = MAIN_Fosc / 13000;
        while(--i)    ;   //14T per loop
    }while(--ms);
}

//========================================================================
// 函数: u8 Lin_CheckPID(u8 id)
// 描述: ID码加上校验符，转成PID码。
// 参数: ID码.
// 返回: PID码.
// 版本: VER1.0
// 日期: 2020-12-2
// 备注: 
//========================================================================
u8 Lin_CheckPID(u8 id)
{
	u8 returnpid ;
	u8 P0 ;
	u8 P1 ;
	
	P0 = (((id)^(id>>1)^(id>>2)^(id>>4))&0x01)<<6 ;
	P1 = ((~((id>>1)^(id>>3)^(id>>4)^(id>>5)))&0x01)<<7 ;
	
	returnpid = id|P0|P1 ;
	
	return returnpid ;
}

//========================================================================
// 函数: u8 LINCalcChecksum(u8 *dat)
// 描述: 计算校验码。
// 参数: 数据场传输的数据.
// 返回: 校验码.
// 版本: VER1.0
// 日期: 2020-12-2
// 备注: 
//========================================================================
static u8 LINCalcChecksum(u8 *dat)
{
  u16 sum = 0;
  u8 i;

  for(i = 0; i < 8; i++)
  {
    sum += dat[i];
    if(sum & 0xFF00)
    {
      sum = (sum & 0x00FF) + 1;
    }
  }
  sum ^= 0x00FF;
  return (u8)sum;
}

//========================================================================
// 函数: void Lin_SendBreak(void)
// 描述: 发送显性间隔信号。
// 参数: none.
// 返回: none.
// 版本: VER1.0
// 日期: 2020-12-2
// 备注: 
//========================================================================
void Lin_SendBreak(void)
{
    SetTimer2Baudraye(Baudrate_Break);
    UART2_TxByte(0);
    SetTimer2Baudraye(Baudrate2);
}

//========================================================================
// 函数: void Lin_Send(u8 *puts)
// 描述: 发送LIN总线报文。
// 参数: 待发送的数据场内容.
// 返回: none.
// 版本: VER1.0
// 日期: 2020-12-2
// 备注: 
//========================================================================
void Lin_Send(u8 *puts)
{
    u8 i;

    Lin_SendBreak();			//Break
    UART2_TxByte(0x55);		//SYNC
    UART2_TxByte(Lin_CheckPID(LIN_ID));		//LIN ID
    for(i=0;i<8;i++)
    {
        UART2_TxByte(puts[i]);
    }
		UART2_TxByte(LINCalcChecksum(puts));
}

//========================================================================
// 函数: void UART1_TxByte(u8 dat)
// 描述: 发送一个字节.
// 参数: 无.
// 返回: 无.
// 版本: V1.0, 2014-6-30
//========================================================================
// void UART1_TxByte(u8 dat)
// {
//     SBUF = dat;
//     B_TX1_Busy = 1;
//     while(B_TX1_Busy);
// }

//========================================================================
// 函数: void UART2_TxByte(u8 dat)
// 描述: 发送一个字节.
// 参数: 无.
// 返回: 无.
// 版本: V1.0, 2014-6-30
//========================================================================
void UART2_TxByte(u8 dat)
{
    S2BUF = dat;
    B_TX2_Busy = 1;
    while(B_TX2_Busy);
}

//========================================================================
// 函数: void PrintString1(u8 *puts)
// 描述: 串口1发送字符串函数。
// 参数: puts:  字符串指针.
// 返回: none.
// 版本: VER1.0
// 日期: 2014-11-28
// 备注: 
//========================================================================
// void PrintString1(u8 *puts)
// {
//     for (; *puts != 0;  puts++)     //遇到停止符0结束
//     {
//         SBUF = *puts;
//         B_TX1_Busy = 1;
//         while(B_TX1_Busy);
//     }
// }

//========================================================================
// 函数: void PrintString2(u8 *puts)
// 描述: 串口2发送字符串函数。
// 参数: puts:  字符串指针.
// 返回: none.
// 版本: VER1.0
// 日期: 2014-11-28
// 备注: 
//========================================================================
//void PrintString2(u8 *puts)
//{
//    for (; *puts != 0;  puts++)     //遇到停止符0结束
//    {
//        S2BUF = *puts;
//        B_TX2_Busy = 1;
//        while(B_TX2_Busy);
//    }
//}

//========================================================================
// 函数: SetTimer2Baudraye(u16 dat)
// 描述: 设置Timer2做波特率发生器。
// 参数: dat: Timer2的重装值.
// 返回: none.
// 版本: VER1.0
// 日期: 2014-11-28
// 备注: 
//========================================================================
void SetTimer2Baudraye(u16 dat)
{
    AUXR &= ~(1<<4);    //Timer stop
    AUXR &= ~(1<<3);    //Timer2 set As Timer
    AUXR |=  (1<<2);    //Timer2 set as 1T mode
    TH2 = dat / 256;
    TL2 = dat % 256;
    IE2  &= ~(1<<2);    //禁止中断
    AUXR |=  (1<<4);    //Timer run enable
}

//========================================================================
// 函数: void UART1_config(u8 brt)
// 描述: UART1初始化函数。
// 参数: brt: 选择波特率, 2: 使用Timer2做波特率, 其它值: 使用Timer1做波特率.
// 返回: none.
// 版本: VER1.0
// 日期: 2014-11-28
// 备注: 
//========================================================================
// void UART1_config(u8 brt)
// {
//     /*********** 波特率使用定时器2 *****************/
//     if(brt == 2)
//     {
//         AUXR |= 0x01;       //S1 BRT Use Timer2;
//         SetTimer2Baudraye(Baudrate1);
//     }

//     /*********** 波特率使用定时器1 *****************/
//     else
//     {
//         TR1 = 0;
//         AUXR &= ~0x01;      //S1 BRT Use Timer1;
//         AUXR |=  (1<<6);    //Timer1 set as 1T mode
//         TMOD &= ~(1<<6);    //Timer1 set As Timer
//         TMOD &= ~0x30;      //Timer1_16bitAutoReload;
//         TH1 = (u8)(Baudrate1 / 256);
//         TL1 = (u8)(Baudrate1 % 256);
//         ET1 = 0;    //禁止中断
//         INT_CLKO &= ~0x02;  //不输出时钟
//         TR1  = 1;
//     }
//     /*************************************************/

//     SCON = (SCON & 0x3f) | 0x40;    //UART1模式, 0x00: 同步移位输出, 0x40: 8位数据,可变波特率, 0x80: 9位数据,固定波特率, 0xc0: 9位数据,可变波特率
// //  PS  = 1;    //高优先级中断
//     ES  = 1;    //允许中断
//     REN = 1;    //允许接收
//     P_SW1 &= 0x3f;
// //  P_SW1 |= 0x80;      //UART1 switch to, 0x00: P3.0 P3.1, 0x40: P3.6 P3.7, 0x80: P1.6 P1.7, 0xC0: P4.3 P4.4

//     B_TX1_Busy = 0;
//     TX1_Cnt = 0;
//     RX1_Cnt = 0;
// }

//========================================================================
// 函数: void UART2_config(u8 brt)
// 描述: UART2初始化函数。
// 参数: brt: 选择波特率, 2: 使用Timer2做波特率, 其它值: 无效.
// 返回: none.
// 版本: VER1.0
// 日期: 2014-11-28
// 备注: 
//========================================================================
void UART2_config(u8 brt)    // 选择波特率, 2: 使用Timer2做波特率, 其它值: 无效.
{
    if(brt == 2)
    {
        SetTimer2Baudraye(Baudrate2);

        S2CON &= ~(1<<7);   // 8位数据, 1位起始位, 1位停止位, 无校验
        IE2   |= 1;         //允许中断
        S2CON |= (1<<4);    //允许接收
        P_SW2 &= ~0x01; 
//        P_SW2 |= 1;         //UART2 switch to: 0: P1.0 P1.1,  1: P4.6 P4.7

        B_TX2_Busy = 0;
        TX2_Cnt = 0;
        RX2_Cnt = 0;
    }
}

//========================================================================
// 函数: void UART1_int (void) interrupt UART1_VECTOR
// 描述: UART1中断函数。
// 参数: nine.
// 返回: none.
// 版本: VER1.0
// 日期: 2014-11-28
// 备注: 
//========================================================================
// void UART1_int (void) interrupt 4
// {
//     if(RI)
//     {
//         RI = 0;
//         if(RX1_Cnt >= UART1_BUF_LENGTH) RX1_Cnt = 0;
//         RX1_Buffer[RX1_Cnt] = SBUF;
//         RX1_Cnt++;
//         RX1_TimeOut = 5;
//     }

//     if(TI)
//     {
//         TI = 0;
//         B_TX1_Busy = 0;
//     }
// }

//========================================================================
// 函数: void UART2_int (void) interrupt UART2_VECTOR
// 描述: UART2中断函数。
// 参数: nine.
// 返回: none.
// 版本: VER1.0
// 日期: 2014-11-28
// 备注: 
//========================================================================
void UART2_int (void) interrupt 8
{
    if((S2CON & 1) != 0)
    {
        S2CON &= ~1;    //Clear Rx flag
        if(RX2_Cnt >= UART2_BUF_LENGTH) RX2_Cnt = 0;
        RX2_Buffer[RX2_Cnt] = S2BUF;
        RX2_Cnt++;
        RX2_TimeOut = 5;
    }

    if((S2CON & 2) != 0)
    {
        S2CON &= ~2;    //Clear Tx flag
        B_TX2_Busy = 0;
    }
}
