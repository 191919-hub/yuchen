#ifndef __ININT_CONFIG_H__
#define __ININT_CONFIG_H__

#define LCD_BUF_BYTS 16		// LCD?????????




extern unsigned char LcdBuff[LCD_BUF_BYTS];
extern const unsigned char ht1621disp_Map_Wd[];
extern const unsigned char ht1621disp_Map_Id[];
extern unsigned char TmF1s;	
extern unsigned char TmF2s;
//extern unsigned char    RX_BUFF1[64];
//extern unsigned char    TX_BUFF1[64];
extern unsigned char SendDataNum;				// ???????????
extern unsigned char Sending;				// ???????????
extern unsigned char BuzzEnable;	

void User_GPIO_Init(void);
void UART5_Wifi_Init(void);
void UART3_USB_Init(void);
void UART4_PwrBoard_Init(void);
void T16N0Init(void);
void PWM_Start(void);
void PWM_Stop(void);
void User_AD_Init(void);
void WdtInit(void);
#endif

