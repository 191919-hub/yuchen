#include "lib_wdt.h"
#include "lib_uart.h"
#include "lib_gpio.h"
#include "lib_adc.h"
#include "lib_scs.h"
#include "lib_timer.h"
#include "includes.h"
#include "Disp.H"
#include "Program_Cfg.h" //程序功能配置，比如禁止蜂鸣器、控制冷藏温度分辨率
#include "Coupler.h"
#include "Init_config.h"


extern unsigned char g_1s_flag_Iot;

/*GPIO 初始化*/
void User_GPIO_Init(void)
{
    GPIO_InitStruType x;

    /*推挽输出*/
    x.GPIO_Signal = GPIO_Pin_Signal_Digital;
    x.GPIO_Func = GPIO_Func_0;
    x.GPIO_Direction = GPIO_Dir_Out;
    x.GPIO_PUEN = GPIO_PUE_Input_Disable;
    x.GPIO_PDEN = GPIO_PDE_Input_Disable;
    x.GPIO_OD = GPIO_ODE_Output_Disable;
    x.GPIO_DS = GPIO_DS_Output_Normal;

    GPIOA_ResetBit(GPIO_Pin_28);
    GPIO_Init(GPIOA, GPIO_Pin_28, &x);  //显示 DIO
    
    GPIOA_ResetBit(GPIO_Pin_27);
    GPIO_Init(GPIOA, GPIO_Pin_27, &x);  //显示 clk

    GPIOA_ResetBit(GPIO_Pin_26);
    GPIO_Init(GPIOA, GPIO_Pin_26, &x);  //显示 stb

    GPIOA_ResetBit(GPIO_Pin_12);
    GPIO_Init(GPIOA, GPIO_Pin_12, &x);  //485 re

    GPIOB_ResetBit(GPIO_Pin_4);
    GPIO_Init(GPIOB, GPIO_Pin_4, &x);  //E2P SCL1

    GPIOB_ResetBit(GPIO_Pin_5);
    GPIO_Init(GPIOB, GPIO_Pin_5, &x);  //E2P SDA1

    GPIOB_ResetBit(GPIO_Pin_9);
    GPIO_Init(GPIOB, GPIO_Pin_9, &x);  //light

    /*上拉输入*/
    x.GPIO_Signal = GPIO_Pin_Signal_Digital;
    x.GPIO_Func = GPIO_Func_0;
    x.GPIO_Direction = GPIO_Dir_In;
    x.GPIO_PUEN = GPIO_PUE_Input_Enable;
    x.GPIO_PDEN = GPIO_PDE_Input_Disable;
    x.GPIO_OD = GPIO_ODE_Output_Disable;
    x.GPIO_DS = GPIO_DS_Output_Normal;

    GPIO_Init(GPIOB, GPIO_Pin_6, &x);  //门开关
    GPIO_Init(GPIOB, GPIO_Pin_7, &x);  //LED_ONLY

    /*浮空输入*/
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

    /*AD输入*/
    x.GPIO_Signal = GPIO_Pin_Signal_Analog;
    x.GPIO_Func = GPIO_Func_0;  //analog功能用不到.GPIO_Func，随便选一个
    x.GPIO_Direction = GPIO_Dir_In;
    x.GPIO_PUEN = GPIO_PUE_Input_Disable;
    x.GPIO_PDEN = GPIO_PDE_Input_Disable;
    x.GPIO_OD = GPIO_ODE_Output_Disable;
    x.GPIO_DS = GPIO_DS_Output_Normal;

    GPIO_Init(GPIOA, GPIO_Pin_4, &x);  //环温ad

    /*蜂鸣器 pwm输出*/
    x.GPIO_Signal = GPIO_Pin_Signal_Digital;
    x.GPIO_Func = GPIO_Func_2;  //BUZ功能
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

    /*UART输入*/
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

    /*UART输出*/
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
    
    y.ADC_ChS = ADC_CHS_AIN5;            //通道:AIN2
    y.ADC_ClkS = ADC_ClkS_PCLK;          //时钟：PCLK
    y.ADC_ClkDiv = ADC_ClkDiv_32;        //预分频：1:32  ADC转换时钟源一定要符合数据手册中ADC转化时钟源选择表
    y.ADC_VrefP = ADC_VrefP_Vcc;        //正向参考电压：VDD
    y.ADC_SampS = ADC_SMPS_HARD;         //AD采样模式选择：硬件
    y.ST = 7;
    y.ADC_VREFN = ENABLE;//DISABLE;
    y.ADC_VRBUF_EN = DISABLE;//ENABLE;
    ADC_Init(&y);                        //按照结构体的参数配置ADC

    ADC_IE_Disable();                    //不使能中断
    ADC_Enable();                        //使能ADC  
	Delay_100us(10);
}

///////////////硬件看门狗/////////////////////
void WdgHwCallback(void)
{

    //Hwwdg_Feed(0x55, 0xAA);  @20181017 CFJ
}

/***************************************************************
 描  述：WDT初始化初始化
 输入值：无
 返回值：无
***************************************************************/
void WdtInit(void)
{
	IWDT_InitStruType x;

    IWDT_RegUnLock();
    x.WDT_Tms = 5000;           //喂狗周期
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



////////////////BT1 1ms中断///////////////////////////////////////
//void BT1UnderflowIrqHandler(void)
/*********************************************************
函数名: void T16N0_IRQHandler(void)
描  述: T16N0定时中断函数
输入值: 无
输出值: 无
返回值: 无 
**********************************************************/
void T16N0_IRQHandler(void) //1ms进一次
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

        if (g_Bus_Error_Time >= 30000) //转为接受状态，1min内未收到数据，则报故障
		{
			g_Sys_Erflag0_Comm = 1; //通讯错误 @20181025 CFJ
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
#if FORBID_BUZZ //禁止蜂鸣器响

#else
    if(!FORBID_BUZZ)
        GPIO->BUZC.BUZEN = 1;
#endif
}
void PWM_Stop(void)
{
    GPIO->BUZC.BUZEN = 0;
}


/*发送缓冲区空中断回调*/
void Uart_PwrBoard_TxCallback(void)
{
    if (g_UART1_TXD_Counter > 29) //> TX_UART_BUF_LEN) //9个数
    {
        g_UART1_TXD_Counter = 0;
        UART_ITConfig(UART4, UART_IT_TB, DISABLE);
        UART_ITConfig(UART4, UART_IT_TXIDLE, ENABLE);       //开发送完成中断
    }
    else
    {
        UART_SendByte(UART4, g_IDM_TX_ODM_Data[g_UART1_TXD_Counter]);
        g_UART1_TXD_Counter++;
    }
}

/*所有数据已经发送出来了*/
void Uart_PwrBoard_TxDoneCallback(void)
{
    UART_ITConfig(UART4, UART_IT_TXIDLE, DISABLE);
    RS_485_STATUS_SET_LOW;
}

void Uart_PwrBoard_RxCallback(void) //显示板接受主控板的UART
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

void UART4_PwrBoard_Init(void) //UART 初始化 9600 主控板与面板通讯
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
    // Mfs_Uart_EnableFunc(&UART1, UartTx); //使能发送接收

    // Mfs_Uart_DisableIrq(&UART1, UartTxIrq); //发送中断无效 @20181008 CFJ

    // Mfs_Uart_EnableIrq(&UART1, UartRxIrq); //使能接收中断

    UART_InitStruType y;

    y.UART_StopBits = UART_StopBits_1;      //停止位：1
    y.UART_TxMode   = UART_DataMode_8;      //发送数据格式：8位数据
    y.UART_TxPolar  = UART_Polar_Normal;    //发送端口极性：正常
    y.UART_RxMode   = UART_DataMode_8;      //接收数据格式：8位数据
    y.UART_RxPolar  = UART_Polar_Normal;    //接收端口极性：正常      
	y.UART_BaudRate = 9600;                 //波特率			
    y.UART_ClockSet = UART_Clock_1;         //时钟选择：Pclk
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

void UART5_Wifi_Init(void) //初始化
{
    // stc_mfs_uart_config_t stcUartConfig; //UART0
    // stc_uart_irq_cb_t stcUart0IrqCb;

    // PDL_ZERO_STRUCT(stcUartConfig);
    // PDL_ZERO_STRUCT(stcUart0IrqCb);

    // SetPinFunc_SIN0_1();
    // SetPinFunc_SOT0_1();

    // stcUart0IrqCb.pfnTxIrqCb = UART_Wifi_SEND_IRQHandler;
    // stcUart0IrqCb.pfnRxIrqCb = UART_Wifi_RECV_IRQHandler;

    // stcUartConfig.enMode = UartNormal;
    // stcUartConfig.u32BaudRate = 115200; //wifi模块115200，打印机9600
    // stcUartConfig.enDataLength = UartEightBits;
    // stcUartConfig.enParity = UartParityNone;
    // stcUartConfig.enStopBit = UartOneStopBit;
    // stcUartConfig.enBitDirection = UartDataLsbFirst;
    // stcUartConfig.bInvertData = FALSE;
    // stcUartConfig.bHwFlow = FALSE;
    // stcUartConfig.pstcFifoConfig = NULL;
    // stcUartConfig.bUseExtClk = FALSE;
    // stcUartConfig.pstcIrqEn = NULL;
    // stcUartConfig.pstcIrqCb = &stcUart0IrqCb;
    // stcUartConfig.bTouchNvic = TRUE;

    // Mfs_Uart_Init(&UART0, &stcUartConfig);

    // Mfs_Uart_EnableFunc(&UART0, UartRx); ////使能接收
    // Mfs_Uart_EnableFunc(&UART0, UartTx); //使能发送

    // Mfs_Uart_EnableIrq(&UART0, UartRxIrq);  //使能接受中断
    // Mfs_Uart_DisableIrq(&UART0, UartTxIrq); //无效发送中断

    UART_InitStruType y;

    y.UART_StopBits = UART_StopBits_1;      //停止位：1
    y.UART_TxMode   = UART_DataMode_8;      //发送数据格式：8位数据
    y.UART_TxPolar  = UART_Polar_Normal;    //发送端口极性：正常
    y.UART_RxMode   = UART_DataMode_8;      //接收数据格式：8位数据
    y.UART_RxPolar  = UART_Polar_Normal;    //接收端口极性：正常      
	y.UART_BaudRate = 115200;                 //波特率			
    y.UART_ClockSet = UART_Clock_1;         //时钟选择：Pclk
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
函数名: void UART3_IRQHandler(void)
描  述: UART中断服务程序
输入值: 无
输出值: 无
返回值: 无 
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
    /////a = SCI1S1;                                    //清发送中断请求位，首先读SCI1S1然后写SCI1D
    if (f_send_data)
    {
        if (f_send55)
        {
            f_send55 = 0;
            UART_SendByte(UART3, 0x55); //SCI1D = 0x55;
            if (r_sendr >= r_sendsum)
            {
                UART_ITConfig(UART3, UART_IT_TB, DISABLE);//Mfs_Uart_DisableIrq(UART3, UartTxIrq); //无效发送中断//SCI1C2_TCIE = 0;                           //关闭发送中断
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
                UART_ITConfig(UART3, UART_IT_TB, DISABLE);//Mfs_Uart_DisableIrq(UART3, UartTxIrq); //无效发送中断//SCI1C2_TCIE = 0;                           //关闭发送模块
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

    /////a = SCI1S1;  //清接收中断请求位，首先读SCI1S1然后读SCI1D
    rx_data = UART_RecByte(UART3);
    if (f_rec_over) //待处理符号，如果 f_rec_over为1，再接收为无效数据
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

void UART3_USB_Init(void) //初始化 9600 USB 通讯初始化
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

    // Mfs_Uart_EnableFunc(&UART2, UartRx); //使能接收
    // Mfs_Uart_EnableFunc(&UART2, UartTx); //使能接收

    // Mfs_Uart_EnableIrq(&UART2, UartRxIrq);  //使能接收中断
    // Mfs_Uart_DisableIrq(&UART2, UartTxIrq); //无效发送中断

    UART_InitStruType y;

    y.UART_StopBits = UART_StopBits_1;      //停止位：1
    y.UART_TxMode   = UART_DataMode_8;      //发送数据格式：8位数据
    y.UART_TxPolar  = UART_Polar_Normal;    //发送端口极性：正常
    y.UART_RxMode   = UART_DataMode_8;      //接收数据格式：8位数据
    y.UART_RxPolar  = UART_Polar_Normal;    //接收端口极性：正常      
	y.UART_BaudRate = 9600;                 //波特率			
    y.UART_ClockSet = UART_Clock_1;         //时钟选择：Pclk
    UART_Init(UART3, &y);

    UART_TBIMConfig(UART3, UART_TRBIM_Byte);
    UART_RBIMConfig(UART3, UART_TRBIM_Byte);
    UART_ITConfig(UART3, UART_IT_RB, ENABLE);
    NVIC_Init(NVIC_UART3_IRQn, NVIC_Priority_1, ENABLE);

    UART3_TxEnable();
    UART3_RxEnable();
}



