/*************************************************************************************
*更改记录：
//@20181016 CFJ
@20181120 TEST为了让冷藏 冷冻快速显示
@20181130 CFJ 更改如下，不在判断电池的低压报警功能，由底板判断；面板只判断底板传过来的故障
@20181130 CFJ 压缩机 风机等 更改了，因为丝印和单片机对不上，后面还要改回去。
@20181221 CFJ 事业部胡工将原来的NTC2改为NTC1 
@20181226 CFJ 增加当电压掉电时间持续长为3s，则认为掉电
@20190121 CFJ
@20190201 CFJ
@20190215 CFJ
@20190221 CFJ
@20190228 SW1退出市电电压校准   CFJ
后续更改记录参见版本记录.txt   王潘飞  2019-9-5                                                                                              
*/

/*
5A 11 22 33 44 55 66 77 88 99 AA BB CC DD EE FF 10 22 33 20 21 24 25 41
*/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/

#include "includes.h"
#include "Disp.H"
#include "logicsub.H"
#include "Program_Cfg.h" //程序功能配置，比如禁止蜂鸣器、控制冷藏温度分辨率
#include "Coupler.h"
#include "esp8266.h"
#include "lib_wdt.h"
#include "lib_scu.h"
#include "lib_uart.h"
#include "lib_scs.h"

BitType myflag1;
BitType myflag2;
BitType myflag3;
BitType myflag4;
BitType myflag5;
BitType myflag6;
BitType myflag7;
BitType myflag8;
BitType myflag9;
BitType myflag10;
BitType myflag11;
BitType myflag12;
BitType myflag13;
BitType myflag14;
BitType myflag15;
BitType myflag16;
BitType mylc_flag;
BitType g_PannelCommflag1;
BitType g_CommErrflag1;
uchar BuzzEnable;
unsigned char g_1s_flag_Iot = 0;
unsigned int Clock_Test;


/*****************************************************************/
void UART5_Init_As_Printer(void);
int32_t main(void)
{
    SystemInit();
    SCU_OpenXTAL();
    SCU_SysClkSelect(CLK_SEL_XTAL);
    SystemCoreClock = 7372800; //其他规格晶振在此处修改即可
    DeviceClockAllEnable();

    User_SysTickInit();

    User_GPIO_Init(); // 定义方向位

    User_AD_Init();
    
    T16N0Init();  //1ms 定时器初始化

    UART3_USB_Init(); //USB模块通讯初始化

    UART5_Wifi_Init(); //wifi模块或打印机通讯初始化

    UART4_PwrBoard_Init(); //UART 初始化 9600 主控板与面板通讯 初始化

    bUartSendStartFlag = 1;

    ReadE2();
    ReadCfgData(); //从flash中读取wifi物联的配置数据，比如SSID/密码/BE码等信息
    InitialUserRegister();
    WdtInit();
    f_First_PowerOnFlag = ON; // 首次上电这样蜂鸣器不叫 @21081120 CFJ //f_test_alarm=ON;//上电全显88,待事业部确认。@20181101 TEST  CFJ
    bSelfUartSendStartFlag = 1;
    l_Self_Detect(); //增加自检测试程序  @20190121 CFJ
    if (SelfCheckNoErrFlag)
    {
        BuzzBiBiBi();
    }
    else
    {
        BuzzBi();
    }
    g_Txd_Time = 250; //面板周期500ms发送数据  CFJ @20190121 CFJ

    for (;;)
    {
        Clock_Test ++;
        IWDT_Clear(); 
        BuzzPrg();
        PowerUpBuzzDelaylc();
        PowerUpBuzzDelayld();
        ad_convert(); 
        WriteToE2();
        IWDT_Clear(); 
        WriteE2();

        Pannel_Comm_Deal(); //接收完成数据处理  计算温度
        JudgeErrs();
        AlarmControl();
        KeyPress();
        IWDT_Clear(); 
        l_Pannel_DataTx();
        DealDispData(); //温度人为干预、数码管显示
        DataToLed();           //数据显示到LED
        Compressor_on_delay(); //上电设定延迟时间
        Compressor_delay_10sec();
        Lc_CompressorJudge();
        LdCompressorJudge();
        LnFan();
        Lc_lightProg(); //冷藏照明灯控制程序
        Nd_fan_Prog();
        Defrost_Prog();
        Door_Open(); ////wys11.03.19
        IWDT_Clear();
        SetTimeFrame();
        NetRecOver();
        ReceiveInitial();
        Time();
        Event_Log();         //记录开关门事件
        if (f_No_WifiModule) //wifi模块不存在，认为接的是打印机
        {
            if (f_No_WifiModule == 1)
            {
                f_No_WifiModule = 2;
                UART5_Init_As_Printer(); //调整波特率为9600
            }
            else if (u8_Send_Print_Time >= 2)
            {
                ReturnPrintFrame();
                u8_Send_Print_Time = 0;
            }
        }
        else
        {
            /*wifi发送和接收处理*/
            WIFI_RX_Data_Deal();
            if (g_1s_flag_Iot)
            {
                g_1s_flag_Iot = 0;
                WIFI_TX_Data();
            }
            /*wifi发送和接收处理*/
        }
    }
}

void UART5_Init_As_Printer(void) //调整波特率为9600
{
    UART_InitStruType y;

    y.UART_StopBits = UART_StopBits_1;      //停止位：1
    y.UART_TxMode   = UART_DataMode_8;      //发送数据格式：8位数据
    y.UART_TxPolar  = UART_Polar_Normal;    //发送端口极性：正常
    y.UART_RxMode   = UART_DataMode_8;      //接收数据格式：8位数据
    y.UART_RxPolar  = UART_Polar_Normal;    //接收端口极性：正常      
	y.UART_BaudRate = 9600;                 //波特率			
    y.UART_ClockSet = UART_Clock_1;         //时钟选择：Pclk
    UART_Init(UART5, &y);

    UART_TBIMConfig(UART5, UART_TRBIM_Byte);
    UART_RBIMConfig(UART5, UART_TRBIM_Byte);
    UART_ITConfig(UART5, UART_IT_RB, ENABLE);
    NVIC_Init(NVIC_UART5_IRQn, NVIC_Priority_1, ENABLE);

    UART5_TxEnable();
    UART5_RxEnable();
}