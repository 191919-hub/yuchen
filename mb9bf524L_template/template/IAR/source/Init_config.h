#ifndef __ININT_CONFIG_H__
#define __ININT_CONFIG_H__

#define LCD_BUF_BYTS 16		// LCD现存所占空间




extern uchar LcdBuff[LCD_BUF_BYTS];
extern const unsigned char ht1621disp_Map_Wd[];
extern const unsigned char ht1621disp_Map_Id[];
extern uchar TmF1s;	
extern uchar TmF2s;
extern U8 RxState;
//extern unsigned char    RX_BUFF1[64];
//extern unsigned char    TX_BUFF1[64];
extern uchar SendDataNum;				// 发送数据长度
extern U8 RptAckJsFlg;			
extern U8 RptAckJs;			
extern U8 HtbAckJsFlg;			
extern U8 HtbAckJs;	
extern U8 r_send_time;		// 发送保存采集数据的时间间隔.1=20ms
extern uchar Sending;				// 数据正在发送
extern uchar BuzzEnable;	

void GPIO_Init(void);
void Main_ClkInit(void);
void MFS0_UART0_Init(void);
void MFS2_UART1_Init(void);
void MFS1_UART_Init(void);
void BT1_Init(void);
void BT2_Init(void);
void Beep_Init(void);
void PWM_Start(void);
void PWM_Stop(void);
unsigned int  ADC_GetValue(unsigned char  Channel);

void Hdware_Watchdog_Init(void);
#endif

