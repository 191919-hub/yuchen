#include "lib_wdt.h"
#include "lib_uart.h"
#include "lib_gpio.h"
#include "lib_adc.h"
#include "lib_scs.h"
#include "lib_timer.h"
#include "includes.h"
#include "Disp.H"
#include "Program_Cfg.h" //���������ã������ֹ����������������¶ȷֱ���
#include "Coupler.h"
#include "Init_config.h"


extern unsigned char g_1s_flag_Iot;

/*GPIO ��ʼ��*/
void User_GPIO_Init(void)
{
    GPIO_InitStruType x;

    /*�������*/
    x.GPIO_Signal = GPIO_Pin_Signal_Digital;
    x.GPIO_Func = GPIO_Func_0;
    x.GPIO_Direction = GPIO_Dir_Out;
    x.GPIO_PUEN = GPIO_PUE_Input_Disable;
    x.GPIO_PDEN = GPIO_PDE_Input_Disable;
    x.GPIO_OD = GPIO_ODE_Output_Disable;
    x.GPIO_DS = GPIO_DS_Output_Normal;

    GPIOA_ResetBit(GPIO_Pin_28);
    GPIO_Init(GPIOA, GPIO_Pin_28, &x);  //��ʾ DIO
    
    GPIOA_ResetBit(GPIO_Pin_27);
    GPIO_Init(GPIOA, GPIO_Pin_27, &x);  //��ʾ clk

    GPIOA_ResetBit(GPIO_Pin_26);
    GPIO_Init(GPIOA, GPIO_Pin_26, &x);  //��ʾ stb

    GPIOA_ResetBit(GPIO_Pin_12);
    GPIO_Init(GPIOA, GPIO_Pin_12, &x);  //485 re

    GPIOB_ResetBit(GPIO_Pin_4);
    GPIO_Init(GPIOB, GPIO_Pin_4, &x);  //E2P SCL1

    GPIOB_ResetBit(GPIO_Pin_5);
    GPIO_Init(GPIOB, GPIO_Pin_5, &x);  //E2P SDA1

    GPIOB_ResetBit(GPIO_Pin_9);
    GPIO_Init(GPIOB, GPIO_Pin_9, &x);  //light

    /*��������*/
    x.GPIO_Signal = GPIO_Pin_Signal_Digital;
    x.GPIO_Func = GPIO_Func_0;
    x.GPIO_Direction = GPIO_Dir_In;
    x.GPIO_PUEN = GPIO_PUE_Input_Enable;
    x.GPIO_PDEN = GPIO_PDE_Input_Disable;
    x.GPIO_OD = GPIO_ODE_Output_Disable;
    x.GPIO_DS = GPIO_DS_Output_Normal;

    GPIO_Init(GPIOB, GPIO_Pin_6, &x);  //�ſ���
    GPIO_Init(GPIOB, GPIO_Pin_7, &x);  //LED_ONLY

    /*��������*/
    x.GPIO_Signal = GPIO_Pin_Signal_Digital;
    x.GPIO_Func = GPIO_Func_0;
    x.GPIO_Direction = GPIO_Dir_In;
    x.GPIO_PUEN = GPIO_PUE_Input_Disable;
    x.GPIO_PDEN = GPIO_PDE_Input_Disable;
    x.GPIO_OD = GPIO_ODE_Output_Disable;
    x.GPIO_DS = GPIO_DS_Output_Normal;

    GPIO_Init(GPIOA, GPIO_Pin_1, &x);  //sw1
    GPIO_Init(GPIOA, GPIO_Pin_0, &x);  //sw2
    GPIO_Init(GPIOB, GPIO_Pin_13, &x);  //sw3
    GPIO_Init(GPIOB, GPIO_Pin_12, &x);  //sw4
    GPIO_Init(GPIOB, GPIO_Pin_11, &x);  //sw5
    GPIO_Init(GPIOB, GPIO_Pin_10, &x);  //sw6
    GPIO_Init(GPIOB, GPIO_Pin_8, &x);  //sw7

    /*AD����*/
    x.GPIO_Signal = GPIO_Pin_Signal_Analog;
    x.GPIO_Func = GPIO_Func_0;  //analog�����ò���.GPIO_Func�����ѡһ��
    x.GPIO_Direction = GPIO_Dir_In;
    x.GPIO_PUEN = GPIO_PUE_Input_Disable;
    x.GPIO_PDEN = GPIO_PDE_Input_Disable;
    x.GPIO_OD = GPIO_ODE_Output_Disable;
    x.GPIO_DS = GPIO_DS_Output_Normal;

    GPIO_Init(GPIOA, GPIO_Pin_4, &x);  //����ad

    /*������ pwm���*/
    x.GPIO_Signal = GPIO_Pin_Signal_Digital;
    x.GPIO_Func = GPIO_Func_2;  //BUZ����
    x.GPIO_Direction = GPIO_Dir_Out;
    x.GPIO_PUEN = GPIO_PUE_Input_Disable;
    x.GPIO_PDEN = GPIO_PDE_Input_Disable;
    x.GPIO_OD = GPIO_ODE_Output_Disable;
    x.GPIO_DS = GPIO_DS_Output_Normal;
    GPIO_Init(GPIOA,GPIO_Pin_31,&x);

	// if(BUZZ_LOW_SOUND)
	// 	GPIO->BUZC.BUZ_LOAD = 20000; 
	// else
		GPIO->BUZC.BUZ_LOAD = 917; 

    /*UART����*/
    x.GPIO_Signal = GPIO_Pin_Signal_Digital;
    x.GPIO_Func = GPIO_Func_2;
    x.GPIO_Direction = GPIO_Dir_In;
    x.GPIO_PUEN = GPIO_PUE_Input_Enable;
    x.GPIO_PDEN = GPIO_PDE_Input_Disable;
    x.GPIO_OD = GPIO_ODE_Output_Disable;
    x.GPIO_DS = GPIO_DS_Output_Normal;

    GPIO_Init(GPIOA, GPIO_Pin_10, &x);  //NET:RX    RXD4

    x.GPIO_Signal = GPIO_Pin_Signal_Digital;
    x.GPIO_Func = GPIO_Func_1;
    x.GPIO_Direction = GPIO_Dir_In;
    x.GPIO_PUEN = GPIO_PUE_Input_Enable;
    x.GPIO_PDEN = GPIO_PDE_Input_Disable;
    x.GPIO_OD = GPIO_ODE_Output_Disable;
    x.GPIO_DS = GPIO_DS_Output_Normal;

    GPIO_Init(GPIOA, GPIO_Pin_2, &x);  //NET:RX2    RXD5

    x.GPIO_Signal = GPIO_Pin_Signal_Digital;
    x.GPIO_Func = GPIO_Func_2;
    x.GPIO_Direction = GPIO_Dir_In;
    x.GPIO_PUEN = GPIO_PUE_Input_Enable;
    x.GPIO_PDEN = GPIO_PDE_Input_Disable;
    x.GPIO_OD = GPIO_ODE_Output_Disable;
    x.GPIO_DS = GPIO_DS_Output_Normal;

    GPIO_Init(GPIOA, GPIO_Pin_14, &x);  //NET:RX1    RXD3

    /*UART���*/
    x.GPIO_Signal = GPIO_Pin_Signal_Digital;
    x.GPIO_Func = GPIO_Func_2;
    x.GPIO_Direction = GPIO_Dir_Out;
    x.GPIO_PUEN = GPIO_PUE_Input_Disable;
    x.GPIO_PDEN = GPIO_PDE_Input_Disable;
    x.GPIO_OD = GPIO_ODE_Output_Disable;
    x.GPIO_DS = GPIO_DS_Output_Normal;

    GPIO_Init(GPIOA, GPIO_Pin_11, &x);  //net:tx  txd4

    x.GPIO_Signal = GPIO_Pin_Signal_Digital;
    x.GPIO_Func = GPIO_Func_1;
    x.GPIO_Direction = GPIO_Dir_Out;
    x.GPIO_PUEN = GPIO_PUE_Input_Disable;
    x.GPIO_PDEN = GPIO_PDE_Input_Disable;
    x.GPIO_OD = GPIO_ODE_Output_Disable;
    x.GPIO_DS = GPIO_DS_Output_Normal;

    GPIO_Init(GPIOA, GPIO_Pin_3, &x);  //net:tx2  txd5

    x.GPIO_Signal = GPIO_Pin_Signal_Digital;
    x.GPIO_Func = GPIO_Func_2;
    x.GPIO_Direction = GPIO_Dir_Out;
    x.GPIO_PUEN = GPIO_PUE_Input_Disable;
    x.GPIO_PDEN = GPIO_PDE_Input_Disable;
    x.GPIO_OD = GPIO_ODE_Output_Disable;
    x.GPIO_DS = GPIO_DS_Output_Normal;

    GPIO_Init(GPIOA, GPIO_Pin_13, &x);  //net:tx1  txd3

}

void User_AD_Init(void)
{
    ADC_InitStruType y;             
    
    y.ADC_ChS = ADC_CHS_AIN5;            //ͨ��:AIN2
    y.ADC_ClkS = ADC_ClkS_PCLK;          //ʱ�ӣ�PCLK
    y.ADC_ClkDiv = ADC_ClkDiv_32;        //Ԥ��Ƶ��1:32  ADCת��ʱ��Դһ��Ҫ���������ֲ���ADCת��ʱ��Դѡ���
    y.ADC_VrefP = ADC_VrefP_Vcc;        //����ο���ѹ��VDD
    y.ADC_SampS = ADC_SMPS_HARD;         //AD����ģʽѡ��Ӳ��
    y.ST = 7;
    y.ADC_VREFN = ENABLE;//DISABLE;
    y.ADC_VRBUF_EN = DISABLE;//ENABLE;
    ADC_Init(&y);                        //���սṹ��Ĳ�������ADC

    ADC_IE_Disable();                    //��ʹ���ж�
    ADC_Enable();                        //ʹ��ADC  
	Delay_100us(10);
}

///////////////Ӳ�����Ź�/////////////////////
void WdgHwCallback(void)
{

    //Hwwdg_Feed(0x55, 0xAA);  @20181017 CFJ
}

/***************************************************************
 ��  ����WDT��ʼ����ʼ��
 ����ֵ����
 ����ֵ����
***************************************************************/
void WdtInit(void)
{
	IWDT_InitStruType x;

    IWDT_RegUnLock();
    x.WDT_Tms = 5000;           //ι������
    x.WDT_IE = ENABLE;          //DISABLE;ENABLE
    x.WDT_Rst = ENABLE;
    x.WDT_Clock = WDT_CLOCK_PCLK;  //WDT_CLOCK_WDT;  WDT_CLOCK_PCLK
    IWDT_Init(&x);
    IWDT_Enable();
    IWDT_RegLock();
	
	NVIC_Init(NVIC_IWDT_IRQn, NVIC_Priority_0, ENABLE);
	
}

// void Clear_Watchdog(void)
// {
//     Hwwdg_Feed(0x55, 0xAA);
// }



////////////////BT1 1ms�ж�///////////////////////////////////////
//void BT1UnderflowIrqHandler(void)
/*********************************************************
������: void T16N0_IRQHandler(void)
��  ��: T16N0��ʱ�жϺ���
����ֵ: ��
���ֵ: ��
����ֵ: �� 
**********************************************************/
void T16N0_IRQHandler(void) //1ms��һ��
{
    t_onems++; //1ms
    R_NetTimeBase1ms++;

    t_1ms++;
    t_1ms_2++;
    t_j20ms++;
    R_HomeNetResponseTimeSecond++;
    if (t_j20ms >= 20)
    {
        t_j20ms = 0;
        t_20ms++;
    }
    if (t_1ms >= 2) //2ms
    {
        t_2ms++;
        t_twoms++;
        t_1ms = 0;

        if (g_Bus_Error_Time >= 30000) //תΪ����״̬��1min��δ�յ����ݣ��򱨹���
		{
			g_Sys_Erflag0_Comm = 1; //ͨѶ���� @20181025 CFJ
		}
		else
		{
			g_Bus_Error_Time++;
		}

        if (t_2ms >= 250)
        {
            t_halfsec++;
            t_2ms = 0;
            f_05s = 1; //0.5s
        }

        if (g_Txd_Time > 0)
        {
            g_Txd_Time--;
        }
        else
        {
            g_Txd_Time = 250;
            bUartSendStartFlag = 1;
        }
    }
    if (t_1ms_2 > 1000)
    {
        g_1s_flag_Iot = 1;
        t_1ms_2 = 0;
    }
    if (R_NetTimeBase1ms >= 100)
    {
        R_NetTimeBase1ms = 0;
        R_HomeNetResponseCheckTime100ms++;
    }
    WIFI_UART_IdleTimer(1);
    WIFI_UART_TimeOut_CntDown(1);
    if (CntDown_n_ms_1 > 0)
        CntDown_n_ms_1 --;
}

void T16N0Init(void) //
{
    TIM_BaseInitStruType x;

    x.TIM_ClkS = TIM_ClkS_PCLK;
    x.TIM_SYNC = DISABLE;
    x.TIM_EDGE = TIM_EDGE_Rise;
    x.TIM_Mode = TIM_Mode_TC0;
    T16Nx_BaseInit(T16N0,&x);
	
    T16Nx_SetPREMAT(T16N0,2);
    T16Nx_SetCNT(T16N0, 0);
    T16Nx_SetMAT0(T16N0,SystemCoreClock / 2000);
    NVIC_Init(NVIC_T16N0_IRQn,NVIC_Priority_0,ENABLE);
    T16Nx_MAT0ITConfig(T16N0,TIM_Clr_Int);
    T16Nx_ITConfig(T16N0,TIM_IT_MAT0,ENABLE);
    T16N0_Enable();
}

void PWM_Start(void)
{
#if FORBID_BUZZ //��ֹ��������

#else
    if(!FORBID_BUZZ)
        GPIO->BUZC.BUZEN = 1;
#endif
}
void PWM_Stop(void)
{
    GPIO->BUZC.BUZEN = 0;
}


/*���ͻ��������жϻص�*/
void Uart_PwrBoard_TxCallback(void)
{
    if (g_UART1_TXD_Counter > 29) //> TX_UART_BUF_LEN) //9����
    {
        g_UART1_TXD_Counter = 0;
        UART_ITConfig(UART4, UART_IT_TB, DISABLE);
        UART_ITConfig(UART4, UART_IT_TXIDLE, ENABLE);       //����������ж�
    }
    else
    {
        UART_SendByte(UART4, g_IDM_TX_ODM_Data[g_UART1_TXD_Counter]);
        g_UART1_TXD_Counter++;
    }
}

/*���������Ѿ����ͳ�����*/
void Uart_PwrBoard_TxDoneCallback(void)
{
    UART_ITConfig(UART4, UART_IT_TXIDLE, DISABLE);
    RS_485_STATUS_SET_LOW;
}

void Uart_PwrBoard_RxCallback(void) //��ʾ��������ذ��UART
{
    unsigned char rx_data; //volatile static uint8_t tmp_data=0;
    unsigned char UART1_RCV_DataBuf;
    unsigned char NCYC;


    rx_data = UART_RecByte(UART4);
    UART1_RCV_DataBuf = rx_data;
    switch (g_UART1_RCV_Counter)
    {
    case 0:
        if (UART1_RCV_DataBuf == 0xAA)
        {
            g_UART1_RCV_Counter = 1;
        }
        break;
    case 1:
        if (UART1_RCV_DataBuf == 0x55)
        {
            g_UART1_RCV_Counter = 2;
        }
        else
        {
            g_UART1_RCV_Counter = 0;
        }
        break;
    case 2:
        if (UART1_RCV_DataBuf == 0xE7)
        {
            g_UART1_RCV_Counter = 3;
        }
        else
        {
            g_UART1_RCV_Counter = 0;
        }
        break;
    case 3:
        if (UART1_RCV_DataBuf == 0x18)
        {
            g_UART1_RCV_Counter = 4;
            g_UART1_RCV_Cyc = 0;
        }
        else
        {
            g_UART1_RCV_Counter = 0;
        }
        break;
    case 4:
        Pannel_Uart1Buf[g_UART1_RCV_Cyc] = UART1_RCV_DataBuf;
        g_UART1_RCV_Cyc++;

        if (g_UART1_RCV_Cyc >= 26) //3)
        {
            NCYC = 0;
            for (g_UART1_RCV_Counter = 0; g_UART1_RCV_Counter <= 24; g_UART1_RCV_Counter++)
            {
                NCYC += Pannel_Uart1Buf[g_UART1_RCV_Counter];
            }
            g_UART1_RCV_Cyc--;
            if (NCYC == Pannel_Uart1Buf[g_UART1_RCV_Cyc])
            {
                if (Pannel_Uart1Buf[2] == 0x01)
                {
                    g_Sys_ReceiveDataType = 1;
                    for (g_UART1_RCV_Counter = 0; g_UART1_RCV_Counter <= 24; g_UART1_RCV_Counter++)
                    {
                        Pannel_Uart1Data[g_UART1_RCV_Counter] = Pannel_Uart1Buf[g_UART1_RCV_Counter];
                    }
                }
                else
                {
                    g_Sys_ReceiveDataType = 0;
                    for (g_UART1_RCV_Counter = 0; g_UART1_RCV_Counter <= 24; g_UART1_RCV_Counter++)
                    {
                        Pannel_Uart1Data[g_UART1_RCV_Counter] = Pannel_Uart1Buf[g_UART1_RCV_Counter];
                    }
                }
                g_Pannel_Comm_bRcv_Done = 1;
                //re_u2c1 = FALSE ;
            }
            else
            {
                g_UART1_RCV_Counter = 0; ////////////
            }
        }

        break;
    default:
        g_UART1_RCV_Cyc = 0;
        g_UART1_RCV_Counter = 0;
        break;
    }
}

void UART4_IRQHandler(void)
{
    if ((UART_GetITStatus(UART4, UART_IT_TB) != RESET) && (UART_GetFlagStatus(UART4, UART_FLAG_TB) != RESET))
    {
        Uart_PwrBoard_TxCallback();
    }

    if(UART_GetITStatus(UART4,UART_IT_TXIDLE) != RESET && UART_GetFlagStatus(UART4,UART_FLAG_TXIDLE) != RESET)
    {
        Uart_PwrBoard_TxDoneCallback();
    }

    if ((UART_GetITStatus(UART4,UART_IT_RB) != RESET) && (UART_GetFlagStatus(UART4, UART_FLAG_RB) != RESET))
    {
        Uart_PwrBoard_RxCallback();
    }
}

void UART4_PwrBoard_Init(void) //UART ��ʼ�� 9600 ���ذ������ͨѶ
{
    // stc_mfs_uart_config_t stcUartConfig; //UART1
    // stc_uart_irq_cb_t stcUart1IrqCb;

    // PDL_ZERO_STRUCT(stcUartConfig);
    // PDL_ZERO_STRUCT(stcUart1IrqCb);

    // SetPinFunc_SIN1_1();
    // SetPinFunc_SOT1_1();

    // stcUart1IrqCb.pfnTxIrqCb = Uart_PwrBoard_TxCallback;
    // stcUart1IrqCb.pfnRxIrqCb = Uart_PwrBoard_RxCallback;

    // stcUartConfig.enMode = UartNormal;
    // stcUartConfig.u32BaudRate = 9600; //2400;
    // stcUartConfig.enDataLength = UartEightBits;
    // stcUartConfig.enParity = UartParityNone;
    // stcUartConfig.enStopBit = UartOneStopBit;
    // stcUartConfig.enBitDirection = UartDataLsbFirst;
    // stcUartConfig.bInvertData = FALSE;
    // stcUartConfig.bHwFlow = FALSE;
    // stcUartConfig.pstcFifoConfig = NULL;
    // stcUartConfig.bUseExtClk = FALSE;
    // stcUartConfig.pstcIrqEn = NULL;
    // stcUartConfig.pstcIrqCb = &stcUart1IrqCb;
    // stcUartConfig.bTouchNvic = TRUE;

    // Mfs_Uart_Init(&UART1, &stcUartConfig);

    // Mfs_Uart_EnableFunc(&UART1, UartRx);
    // Mfs_Uart_EnableFunc(&UART1, UartTx); //ʹ�ܷ��ͽ���

    // Mfs_Uart_DisableIrq(&UART1, UartTxIrq); //�����ж���Ч @20181008 CFJ

    // Mfs_Uart_EnableIrq(&UART1, UartRxIrq); //ʹ�ܽ����ж�

    UART_InitStruType y;

    y.UART_StopBits = UART_StopBits_1;      //ֹͣλ��1
    y.UART_TxMode   = UART_DataMode_8;      //�������ݸ�ʽ��8λ����
    y.UART_TxPolar  = UART_Polar_Normal;    //���Ͷ˿ڼ��ԣ�����
    y.UART_RxMode   = UART_DataMode_8;      //�������ݸ�ʽ��8λ����
    y.UART_RxPolar  = UART_Polar_Normal;    //���ն˿ڼ��ԣ�����      
	y.UART_BaudRate = 9600;                 //������			
    y.UART_ClockSet = UART_Clock_1;         //ʱ��ѡ��Pclk
    UART_Init(UART4, &y);

    UART_TBIMConfig(UART4, UART_TRBIM_Byte);
    UART_RBIMConfig(UART4, UART_TRBIM_Byte);
    UART_ITConfig(UART4, UART_IT_RB, ENABLE);
    NVIC_Init(NVIC_UART4_IRQn, NVIC_Priority_1, ENABLE);

    UART4_TxEnable();
    UART4_RxEnable();
}


void UART5_IRQHandler(void)
{
    if ((UART_GetITStatus(UART5, UART_IT_TB) != RESET) && (UART_GetFlagStatus(UART5, UART_FLAG_TB) != RESET))
    {
        UART_Wifi_SEND_IRQHandler();
    }

    if ((UART_GetITStatus(UART5,UART_IT_RB) != RESET) && (UART_GetFlagStatus(UART5, UART_FLAG_RB) != RESET))
    {
        UART_Wifi_RECV_IRQHandler();
    }
}

void UART5_Wifi_Init(void) //��ʼ��
{

    UART_InitStruType y;

    y.UART_StopBits = UART_StopBits_1;      //ֹͣλ��1
    y.UART_TxMode   = UART_DataMode_8;      //�������ݸ�ʽ��8λ����
    y.UART_TxPolar  = UART_Polar_Normal;    //���Ͷ˿ڼ��ԣ�����
    y.UART_RxMode   = UART_DataMode_8;      //�������ݸ�ʽ��8λ����
    y.UART_RxPolar  = UART_Polar_Normal;    //���ն˿ڼ��ԣ�����      
	y.UART_BaudRate = 9600;                 //������			
    y.UART_ClockSet = UART_Clock_1;         //ʱ��ѡ��Pclk
    UART_Init(UART5, &y);

    UART_TBIMConfig(UART5, UART_TRBIM_Byte);
    UART_RBIMConfig(UART5, UART_TRBIM_Byte);
    UART_ITConfig(UART5, UART_IT_RB, ENABLE);
    NVIC_Init(NVIC_UART5_IRQn, NVIC_Priority_1, ENABLE);

    UART5_TxEnable();
    UART5_RxEnable();
}
//---------------------------------------------------------------------------//

void Uart_Usb_TxCallback(void);
void Uart_Usb_RxCallback(void);
/*********************************************************
������: void UART3_IRQHandler(void)
��  ��: UART�жϷ������
����ֵ: ��
���ֵ: ��
����ֵ: �� 
**********************************************************/
void UART3_IRQHandler(void)
{
    if ((UART_GetITStatus(UART3, UART_IT_TB) != RESET) && (UART_GetFlagStatus(UART3, UART_FLAG_TB) != RESET))
    {
        Uart_Usb_TxCallback();
    }

    if ((UART_GetITStatus(UART3,UART_IT_RB) != RESET) && (UART_GetFlagStatus(UART3, UART_FLAG_RB) != RESET))
    {
        Uart_Usb_RxCallback();
    }
}

void Uart_Usb_TxCallback(void)
{
    //unsigned char Tx_data;
    /////a = SCI1S1;                                    //�巢���ж�����λ�����ȶ�SCI1S1Ȼ��дSCI1D
    if (f_send_data)
    {
        if (f_send55)
        {
            f_send55 = 0;
            UART_SendByte(UART3, 0x55); //SCI1D = 0x55;
            if (r_sendr >= r_sendsum)
            {
                UART_ITConfig(UART3, UART_IT_TB, DISABLE);//Mfs_Uart_DisableIrq(UART3, UartTxIrq); //��Ч�����ж�//SCI1C2_TCIE = 0;                           //�رշ����ж�
                f_send_1ff = 0;
                f_send_2ff = 0;
                f_send_data = 0;
            }
        }
        else if (send_net[r_sendr] == 0xff)
        {
            f_send55 = 1;
            UART_SendByte(UART3, 0xff); //SCI1D = 0xff;
            r_sendr++;
        }
        else
        {
            UART_SendByte(UART3, send_net[r_sendr]); //SCI1D = send_net[r_sendr];
            r_sendr++;
            if (r_sendr >= r_sendsum)
            {
                UART_ITConfig(UART3, UART_IT_TB, DISABLE);//Mfs_Uart_DisableIrq(UART3, UartTxIrq); //��Ч�����ж�//SCI1C2_TCIE = 0;                           //�رշ���ģ��
                f_send55 = 0;
                f_send_1ff = 0;
                f_send_2ff = 0;
                f_send_data = 0;
            }
        }
    }
    else if (f_send_2ff)
    {
        f_send_data = 1;
        r_sendr = 0;
        UART_SendByte(UART3, r_sendsum); ////SCI1D = r_sendsum;
    }
    else if (f_send_1ff)
    {
        f_send_2ff = 1;
        UART_SendByte(UART3, 0xff); //SCI1D = 0xff;
    }
    else
    {
        f_send_1ff = 1;
        UART_SendByte(UART3, 0xff); //////SCI1D = 0xff;
    }
}

void Uart_Usb_RxCallback(void)
{
    unsigned char rx_data;

    /////a = SCI1S1;  //������ж�����λ�����ȶ�SCI1S1Ȼ���SCI1D
    rx_data = UART_RecByte(UART3);
    if (f_rec_over) //��������ţ���� f_rec_overΪ1���ٽ���Ϊ��Ч����
    {
        // a = SCI1D;
    }
    else if (f_rec_data)
    {
        if (f_rec55)
        {
            f_rec55 = 0;
            //a = SCI1D;
            if (rx_data == 0x55) //if(a==0x55)
            {
                r_rec55sum = r_rec55sum + 0x55;
            }
            else
            {
                f_rec_over = 0;
                f_rec_1ff = 0;
                f_rec_2ff = 0;
                f_rec_data = 0;
                r_rec55sum = 0;
            }
        }
        else
        {
            rec_net[r_receiver] = rx_data; // rec_net[r_receiver] = SCI1D; @20181130 CFJ
            if (rec_net[r_receiver] == 0xff)
            {
                f_rec55 = 1;
            }
            r_receiver++;
            if (r_receiver >= r_recsum)
            {
                f_rec_over = 1;
                f_rec_1ff = 0;
                f_rec_2ff = 0;
                f_rec_data = 0;
                f_rec55 = 0;
            }
        }
    }
    else if (f_rec_2ff)
    {
        r_recsum = rx_data; //r_recsum = SCI1D;
        if (r_recsum <= 20)
        {
            f_rec_data = 1;
            r_receiver = 0;
            t_net_rec = t_onems;
        }
        else
        {
            f_rec_data = 0;
            f_rec_2ff = 0;
            f_rec_1ff = 0;
            t_net_rec = t_onems;
        }
    }
    else if (f_rec_1ff)
    {
        if (rx_data == 0xff) //if(SCI1D==0xff)
        {
            f_rec_2ff = 1;
            r_receiver = 0;
            t_net_rec = t_onems;
        }
        else
        {
            f_rec_1ff = 0;
        }
    }
    else
    {
        if (rx_data == 0xff) //if(SCI1D==0xff)
        {
            f_rec_1ff = 1;
            t_net_rec = t_onems;
        }
    }
}

void UART3_USB_Init(void) //��ʼ�� 9600 USB ͨѶ��ʼ��
{
    // stc_mfs_uart_config_t stcUartConfig; //UART0
    // stc_uart_irq_cb_t stcUart2IrqCb;

    // PDL_ZERO_STRUCT(stcUartConfig);
    // PDL_ZERO_STRUCT(stcUart2IrqCb);

    // SetPinFunc_SIN2_2();
    // SetPinFunc_SOT2_2();

    // stcUart2IrqCb.pfnTxIrqCb = Uart_Usb_TxCallback;
    // stcUart2IrqCb.pfnRxIrqCb = Uart_Usb_RxCallback;

    // stcUartConfig.enMode = UartNormal;
    // stcUartConfig.u32BaudRate = 9600;
    // stcUartConfig.enDataLength = UartEightBits;
    // stcUartConfig.enParity = UartParityNone;
    // stcUartConfig.enStopBit = UartOneStopBit;
    // stcUartConfig.enBitDirection = UartDataLsbFirst;
    // stcUartConfig.bInvertData = FALSE;
    // stcUartConfig.bHwFlow = FALSE;
    // stcUartConfig.pstcFifoConfig = NULL;
    // stcUartConfig.bUseExtClk = FALSE;
    // stcUartConfig.pstcIrqEn = NULL;
    // stcUartConfig.pstcIrqCb = &stcUart2IrqCb;
    // stcUartConfig.bTouchNvic = TRUE;

    // Mfs_Uart_Init(&UART2, &stcUartConfig);

    // Mfs_Uart_EnableFunc(&UART2, UartRx); //ʹ�ܽ���
    // Mfs_Uart_EnableFunc(&UART2, UartTx); //ʹ�ܽ���

    // Mfs_Uart_EnableIrq(&UART2, UartRxIrq);  //ʹ�ܽ����ж�
    // Mfs_Uart_DisableIrq(&UART2, UartTxIrq); //��Ч�����ж�

    UART_InitStruType y;

    y.UART_StopBits = UART_StopBits_1;      //ֹͣλ��1
    y.UART_TxMode   = UART_DataMode_8;      //�������ݸ�ʽ��8λ����
    y.UART_TxPolar  = UART_Polar_Normal;    //���Ͷ˿ڼ��ԣ�����
    y.UART_RxMode   = UART_DataMode_8;      //�������ݸ�ʽ��8λ����
    y.UART_RxPolar  = UART_Polar_Normal;    //���ն˿ڼ��ԣ�����      
	y.UART_BaudRate = 9600;                 //������			
    y.UART_ClockSet = UART_Clock_1;         //ʱ��ѡ��Pclk
    UART_Init(UART3, &y);

    UART_TBIMConfig(UART3, UART_TRBIM_Byte);
    UART_RBIMConfig(UART3, UART_TRBIM_Byte);
    UART_ITConfig(UART3, UART_IT_RB, ENABLE);
    NVIC_Init(NVIC_UART3_IRQn, NVIC_Priority_1, ENABLE);

    UART3_TxEnable();
    UART3_RxEnable();
}



