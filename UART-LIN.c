/*---------------------------------------------------------------------*/
/* --- STC MCU Limited ------------------------------------------------*/
/* --- STC 1T Series MCU Demo Programme -------------------------------*/
/* --- Mobile: (86)13922805190 ----------------------------------------*/
/* --- Fax: 86-0513-55012956,55012947,55012969 ------------------------*/
/* --- Tel: 86-0513-55012928,55012929,55012966 ------------------------*/
/* --- Web: www.STCMCU.com --------------------------------------------*/
/* --- Web: www.STCMCUDATA.com  ---------------------------------------*/
/* --- QQ:  800003751 -------------------------------------------------*/
/* ���Ҫ�ڳ�����ʹ�ô˴���,���ڳ�����ע��ʹ����STC�����ϼ�����        */
/*---------------------------------------------------------------------*/


/*************  ����˵��    **************

�����̻���STC8H8K64UΪ����оƬ��ʵ����8���б�д���ԣ�STC8G��STC8Hϵ��оƬ��ͨ�òο�.

ͨ��UART�ӿ�����LIN�շ���ʵ��LIN�����ź��շ���������.

UART1ͨ�����ڹ������ӵ���.

UART2���LIN�շ���(TJA1020/1), ����LIN����.

�����Դ��ڷ��͵�����ת����LIN����; ��LIN���߽��յ�������ת�������Դ���.

Ĭ�ϴ������ʣ�9600�����ʣ�����LIN����ǰ�л������ʣ�����13�����Լ���ź�.

����ʱ, ѡ��ʱ�� 22.1184MHz (�û��������޸�Ƶ��).

******************************************/

#include    "reg51.h"       //������ͷ�ļ������������ļĴ�������Ҫ���ֶ����룬�����ظ�����
#include    "intrins.h"

#define MAIN_Fosc       22118400L   //������ʱ�ӣ���ȷ���㲨���ʣ�

typedef     unsigned char   u8;
typedef     unsigned int    u16;
typedef     unsigned long   u32;

//�ֶ���������"reg51.h"ͷ�ļ�����û�ж���ļĴ���
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
// typedef struct {
//     u8 header;
//     u8 sensor_id;
//     u8 checksum;
// } Recv_t;

typedef struct {
    u8 header;
    unsigned int sensor_data[32];
    u8 checksum;
} Send_t;

/****************************** �û������ ***********************************/

#define Baudrate1           (65536UL - (MAIN_Fosc / 4) / 9600UL)
#define Baudrate2           (65536UL - (MAIN_Fosc / 4) / 9600UL)  //�������ݴ��䲨����

#define Baudrate_Break      (65536UL - (MAIN_Fosc / 4) / 6647UL)  //�������Լ���źŲ�����

#define UART1_BUF_LENGTH    32
#define UART2_BUF_LENGTH    32

#define LIN_ID    0x31

u8  TX1_Cnt;    //���ͼ���
u8  RX1_Cnt;    //���ռ���
u8  TX2_Cnt;    //���ͼ���
u8  RX2_Cnt;    //���ռ���
bit B_TX1_Busy; //����æ��־
bit B_TX2_Busy; //����æ��־
u8  RX1_TimeOut;
u8  RX2_TimeOut;

u8  xdata RX1_Buffer[UART1_BUF_LENGTH]; //���ջ���
u8  xdata RX2_Buffer[UART2_BUF_LENGTH]; //���ջ���

Recv_t xdata recv_data;
Send_t xdata send_data;

// void UART1_config(u8 brt);   // ѡ������, 2: ʹ��Timer2��������, ����ֵ: ʹ��Timer1��������.
void UART2_config(u8 brt);   // ѡ������, 2: ʹ��Timer2��������, ����ֵ: ��Ч.
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

void send_sensor_value(unsigned int* data, Send_t* send_data)
{
    send_data->header = 0x55;
    send_data->sensor_data = data;
    send_data->checksum = check_sum(1, &send_data->sensor_data, sizeof(send_data) - 1);
    memcpy(&RX2_Buffer, &send_data, sizeof(send_data));

    // Set P5.5 to LOW
    P5 &= ~0x20; // Clear bit 5 in P5
    for (i=0; i < sizeof(send_data); i++)
        UART1_TxByte(RX2_Buffer[i]);
    // Set P5.5 to HIGH
    P5 |= 0x20; // Set bit 5 in P5
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

    if(channel == 6)
    {
        // Set P3.7 to LOW
        P3 &= ~0x80; // Clear bit 7 in P3
    }
    else
    {
        // Set P3.2 to LOW
    P3 &= ~0x04; // Clear bit 2 in P3
    }

    // Wait for ADC conversion to complete
    while (!(ADC_CONTR & 0x10));

    if(channel == 6)
    {
        // Set P3.7 to HIGH
        P3 |= 0x80; // Set bit 7 in P3
    }
    else
    {
        // Set P3.2 to HIGH
        P3 |= 0x04; // Set bit 2 in P3
    }

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

    // Set P5.5 to digital output
    P5M1 &= ~(0x20); // Clear bit 5 in P5M1
    P5M0 |= 0x20;    // Set bit 5 in P5M0

    // Set P3.7 to digital output
    P3M1 &= ~(0x80); // Clear bit 7 in P3M1
    P3M0 |= 0x80;    // Set bit 7 in P3M0

    // Set P3.2 to digital output
    P3M1 &= ~(0x04); // Clear bit 2 in P3M1
    P3M0 |= 0x04;    // Set bit 2 in P3M0

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
// ����: void main(void)
// ����: ��������
// ����: none.
// ����: none.
// �汾: VER1.0
// ����: 2014-11-28
// ��ע: 
//========================================================================
void main(void)
{
    u8 i;

    // P0M1 = 0x00;   P0M0 = 0x00;   //����Ϊ׼˫���
    // P1M1 = 0x00;   P1M0 = 0x00;   //����Ϊ׼˫���
    // P2M1 = 0x00;   P2M0 = 0x00;   //����Ϊ׼˫���
    // P3M1 = 0x00;   P3M0 = 0x00;   //����Ϊ׼˫���
    // P4M1 = 0x00;   P4M0 = 0x00;   //����Ϊ׼˫���
    // P5M1 = 0x00;   P5M0 = 0x00;   //����Ϊ׼˫���
    // P6M1 = 0x00;   P6M0 = 0x00;   //����Ϊ׼˫���
    // P7M1 = 0x00;   P7M0 = 0x00;   //����Ϊ׼˫���

    configure_pins();

    // UART1_config(1);    // ѡ������, 2: ʹ��Timer2��������, ����ֵ: ʹ��Timer1��������.
    UART2_config(2);    // ѡ������, 2: ʹ��Timer2��������, ����ֵ: ��Ч.
    EA = 1;             //����ȫ���ж�
    SLP_N = 1;

    // PrintString1("STC8H8K64U UART1 Test Programme!\r\n");  //UART1����һ���ַ���
    // PrintString1("STC8 Started\r\n");

    while (1)
    {

        delay_ms(1);
        // if(RX1_TimeOut > 0)
        // {
        //     if(--RX1_TimeOut == 0)  //��ʱ,�򴮿ڽ��ս���
        //     {
        //         if(RX1_Cnt > 0)
        //         {
        //             recv(&recv_data, RX1_Buffer);
		// 			// Lin_Send(RX1_Buffer);  //��UART1�յ������ݷ��͵�LIN������
        //         }
        //         RX1_Cnt = 0;
        //     }
        // }

        if(RX2_TimeOut > 0)
        {
            // if(--RX2_TimeOut == 0)  //��ʱ,�򴮿ڽ��ս���
            // {
            //     if(RX2_Cnt > 0)
            //     {
            //         u8 sensor_id = recv_sensor_id(&recv_data, RX2_Buffer);
            //         unsigned int sensor_value = get_sensor_value(sensor_id);
            //         send_sensor_value(sensor_value, &send_data);
			// 		// Lin_Send(RX1_Buffer);  //��UART1�յ������ݷ��͵�LIN������
            //     }
            //     RX2_Cnt = 0;
            // }
            unsigned int sensor_value[32];
            for(i=0; i < 32; i++)
                sensor_value[i] = get_sensor_value(i);
                
            send_sensor_value(sensor_value, &send_data);
        }
				
        // if(RX2_TimeOut > 0)
        // {
        //     if(--RX2_TimeOut == 0)  //��ʱ,�򴮿ڽ��ս���
        //     {
        //         if(RX2_Cnt > 0)
        //         {
        //             for (i=0; i < RX2_Cnt; i++)     //����ֹͣ��0����
        //             {
		// 				UART1_TxByte(RX2_Buffer[i]);  //��LIN�����յ������ݷ��͵�UART1
        //             }
        //         }
        //         RX2_Cnt = 0;
        //     }
        // }
    }
}


//========================================================================
// ����: void delay_ms(unsigned char ms)
// ����: ��ʱ������
// ����: ms,Ҫ��ʱ��ms��, ����ֻ֧��1~255ms. �Զ���Ӧ��ʱ��.
// ����: none.
// �汾: VER1.0
// ����: 2013-4-1
// ��ע: 
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
// ����: u8 Lin_CheckPID(u8 id)
// ����: ID�����У�����ת��PID�롣
// ����: ID��.
// ����: PID��.
// �汾: VER1.0
// ����: 2020-12-2
// ��ע: 
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
// ����: u8 LINCalcChecksum(u8 *dat)
// ����: ����У���롣
// ����: ���ݳ����������.
// ����: У����.
// �汾: VER1.0
// ����: 2020-12-2
// ��ע: 
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
// ����: void Lin_SendBreak(void)
// ����: �������Լ���źš�
// ����: none.
// ����: none.
// �汾: VER1.0
// ����: 2020-12-2
// ��ע: 
//========================================================================
void Lin_SendBreak(void)
{
    SetTimer2Baudraye(Baudrate_Break);
    UART2_TxByte(0);
    SetTimer2Baudraye(Baudrate2);
}

//========================================================================
// ����: void Lin_Send(u8 *puts)
// ����: ����LIN���߱��ġ�
// ����: �����͵����ݳ�����.
// ����: none.
// �汾: VER1.0
// ����: 2020-12-2
// ��ע: 
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
// ����: void UART1_TxByte(u8 dat)
// ����: ����һ���ֽ�.
// ����: ��.
// ����: ��.
// �汾: V1.0, 2014-6-30
//========================================================================
// void UART1_TxByte(u8 dat)
// {
//     SBUF = dat;
//     B_TX1_Busy = 1;
//     while(B_TX1_Busy);
// }

//========================================================================
// ����: void UART2_TxByte(u8 dat)
// ����: ����һ���ֽ�.
// ����: ��.
// ����: ��.
// �汾: V1.0, 2014-6-30
//========================================================================
void UART2_TxByte(u8 dat)
{
    S2BUF = dat;
    B_TX2_Busy = 1;
    while(B_TX2_Busy);
}

//========================================================================
// ����: void PrintString1(u8 *puts)
// ����: ����1�����ַ���������
// ����: puts:  �ַ���ָ��.
// ����: none.
// �汾: VER1.0
// ����: 2014-11-28
// ��ע: 
//========================================================================
// void PrintString1(u8 *puts)
// {
//     for (; *puts != 0;  puts++)     //����ֹͣ��0����
//     {
//         SBUF = *puts;
//         B_TX1_Busy = 1;
//         while(B_TX1_Busy);
//     }
// }

//========================================================================
// ����: void PrintString2(u8 *puts)
// ����: ����2�����ַ���������
// ����: puts:  �ַ���ָ��.
// ����: none.
// �汾: VER1.0
// ����: 2014-11-28
// ��ע: 
//========================================================================
//void PrintString2(u8 *puts)
//{
//    for (; *puts != 0;  puts++)     //����ֹͣ��0����
//    {
//        S2BUF = *puts;
//        B_TX2_Busy = 1;
//        while(B_TX2_Busy);
//    }
//}

//========================================================================
// ����: SetTimer2Baudraye(u16 dat)
// ����: ����Timer2�������ʷ�������
// ����: dat: Timer2����װֵ.
// ����: none.
// �汾: VER1.0
// ����: 2014-11-28
// ��ע: 
//========================================================================
void SetTimer2Baudraye(u16 dat)
{
    AUXR &= ~(1<<4);    //Timer stop
    AUXR &= ~(1<<3);    //Timer2 set As Timer
    AUXR |=  (1<<2);    //Timer2 set as 1T mode
    TH2 = dat / 256;
    TL2 = dat % 256;
    IE2  &= ~(1<<2);    //��ֹ�ж�
    AUXR |=  (1<<4);    //Timer run enable
}

//========================================================================
// ����: void UART1_config(u8 brt)
// ����: UART1��ʼ��������
// ����: brt: ѡ������, 2: ʹ��Timer2��������, ����ֵ: ʹ��Timer1��������.
// ����: none.
// �汾: VER1.0
// ����: 2014-11-28
// ��ע: 
//========================================================================
// void UART1_config(u8 brt)
// {
//     /*********** ������ʹ�ö�ʱ��2 *****************/
//     if(brt == 2)
//     {
//         AUXR |= 0x01;       //S1 BRT Use Timer2;
//         SetTimer2Baudraye(Baudrate1);
//     }

//     /*********** ������ʹ�ö�ʱ��1 *****************/
//     else
//     {
//         TR1 = 0;
//         AUXR &= ~0x01;      //S1 BRT Use Timer1;
//         AUXR |=  (1<<6);    //Timer1 set as 1T mode
//         TMOD &= ~(1<<6);    //Timer1 set As Timer
//         TMOD &= ~0x30;      //Timer1_16bitAutoReload;
//         TH1 = (u8)(Baudrate1 / 256);
//         TL1 = (u8)(Baudrate1 % 256);
//         ET1 = 0;    //��ֹ�ж�
//         INT_CLKO &= ~0x02;  //�����ʱ��
//         TR1  = 1;
//     }
//     /*************************************************/

//     SCON = (SCON & 0x3f) | 0x40;    //UART1ģʽ, 0x00: ͬ����λ���, 0x40: 8λ����,�ɱ䲨����, 0x80: 9λ����,�̶�������, 0xc0: 9λ����,�ɱ䲨����
// //  PS  = 1;    //�����ȼ��ж�
//     ES  = 1;    //�����ж�
//     REN = 1;    //��������
//     P_SW1 &= 0x3f;
// //  P_SW1 |= 0x80;      //UART1 switch to, 0x00: P3.0 P3.1, 0x40: P3.6 P3.7, 0x80: P1.6 P1.7, 0xC0: P4.3 P4.4

//     B_TX1_Busy = 0;
//     TX1_Cnt = 0;
//     RX1_Cnt = 0;
// }

//========================================================================
// ����: void UART2_config(u8 brt)
// ����: UART2��ʼ��������
// ����: brt: ѡ������, 2: ʹ��Timer2��������, ����ֵ: ��Ч.
// ����: none.
// �汾: VER1.0
// ����: 2014-11-28
// ��ע: 
//========================================================================
void UART2_config(u8 brt)    // ѡ������, 2: ʹ��Timer2��������, ����ֵ: ��Ч.
{
    if(brt == 2)
    {
        SetTimer2Baudraye(Baudrate2);

        S2CON &= ~(1<<7);   // 8λ����, 1λ��ʼλ, 1λֹͣλ, ��У��
        IE2   |= 1;         //�����ж�
        S2CON |= (1<<4);    //��������
        P_SW2 &= ~0x01; 
//        P_SW2 |= 1;         //UART2 switch to: 0: P1.0 P1.1,  1: P4.6 P4.7

        B_TX2_Busy = 0;
        TX2_Cnt = 0;
        RX2_Cnt = 0;
    }
}

//========================================================================
// ����: void UART1_int (void) interrupt UART1_VECTOR
// ����: UART1�жϺ�����
// ����: nine.
// ����: none.
// �汾: VER1.0
// ����: 2014-11-28
// ��ע: 
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
// ����: void UART2_int (void) interrupt UART2_VECTOR
// ����: UART2�жϺ�����
// ����: nine.
// ����: none.
// �汾: VER1.0
// ����: 2014-11-28
// ��ע: 
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
