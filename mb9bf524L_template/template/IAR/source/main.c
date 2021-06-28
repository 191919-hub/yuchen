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
#include "pdl_header.h"
#include "includes.h"
#include "Disp.H"
#include "logicsub.H"
#include "Program_Cfg.h" //程序功能配置，比如禁止蜂鸣器、控制冷藏温度分辨率
#include "Coupler.h"
#include "esp8266.h"

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

void SetupMaxCore_CLK(void)
{
    uint32_t u32IoRegisterRead;
    FM_CRG->SCM_CTL = 0;
    FM_CRG->BSC_PSR = BSC_PSR_Val; /* set System Clock presacaler */
    FM_CRG->APBC0_PSR = 0x00;      /* set APB0 presacaler */
    FM_CRG->APBC1_PSR = 0x80;      /* set APB1 presacaler */
    FM_CRG->APBC2_PSR = 0x80;      /* set APB2 presacaler */
    FM_CRG->SWC_PSR = 0x00000003;  /* set SW Watchdog presacaler */
    FM_CRG->TTC_PSR = 0;           /* set Trace Clock presacaler */

    FM_CRG->CSW_TMR = 0x0000005C; /* set oscillation stabilization wait time */

    //if (SCM_CTL_Val & (1UL << 1)) {                    /* Main clock oscillator enabled ? */
    FM_CRG->SCM_CTL |= (1UL << 1); /* enable main oscillator */
    while (!(FM_CRG->SCM_STR & (1UL << 1)))
        ; /* wait for Main clock oscillation stable */
    //}
    SystemExternalClock = 4 * 1000000;
    FM_CRG->PSW_TMR = 0x00000000; /* set PLL stabilization wait time */

    FM_CRG->SCM_CTL |= (0x00000020 & 0xE0); /* Set Master Clock switch */

    do
    {
        u32IoRegisterRead = (FM_CRG->SCM_CTL & 0xE0);
    } while ((FM_CRG->SCM_STR & 0xE0) != u32IoRegisterRead); ///< wait for main clock stability
    SystemCoreClockUpdate();                                 ///< get the system core clock value                                                    ///< get the system core clock value
}

void SetupCR_CLK(void)
{
    if (0x000003FF != (FM_FLASH_IF->CRTRMM & 0x000003FF))
    {
        /* UnLock (MCR_FTRM) */
        FM_CRTRIM->MCR_RLR = 0x1ACCE554;
        /* Set MCR_FTRM */
        FM_CRTRIM->MCR_FTRM = FM_FLASH_IF->CRTRMM;
        /* Lock (MCR_FTRM) */
        FM_CRTRIM->MCR_RLR = 0x00000000;
    }
}

void Delay(uint i) //(uint32_t i)
{

    while (i--)
        ;
}

/*****************************************************************/
void UART0_Init_As_Printer(void);
int32_t main(void)
{

    Main_ClkInit();

    GPIO_Init(); // 定义方向位
    //1ms 定时器初始化
    BT1_Init();

    Beep_Init();

    MFS2_UART1_Init(); //USB通讯初始化

    MFS0_UART0_Init(); //wifi模块或打印机通讯初始化

    MFS1_UART_Init(); //UART 初始化 9600 主控板与面板通讯 初始化

    bUartSendStartFlag = 1;
    //g_Txd_Time = 250; @20190121 CFJ//面板周期500ms发送数据  CFJ
    //ReadE2();
    ReadE2();
    ReadCfgData(); //从flash中读取wifi物联的配置数据，比如SSID/密码/BE码等信息
    InitialUserRegister();
    Hdware_Watchdog_Init();
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
        Clear_Watchdog(); ///// __RESET_WATCHDOG();                           /* feeds the dog */
        BuzzPrg();
        PowerUpBuzzDelaylc();
        PowerUpBuzzDelayld();
        ad_convert(); //需检查下，是否正确 CFJ  ？？
        WriteToE2();
        Clear_Watchdog(); ////  __RESET_WATCHDOG();
        WriteE2();

        Pannel_Comm_Deal(); //DealRecData();//接收完成数据处理  计算温度
        JudgeErrs();
        AlarmControl();
        KeyPress();
        Clear_Watchdog(); //// __RESET_WATCHDOG();
        l_Pannel_DataTx();
        DealDispData(); //温度人为干预、数码管显示

        EheatConTrolP();       //HW_2019/4/24 10:35:28
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
        //report_judge();
        Clear_Watchdog(); ///__RESET_WATCHDOG();
        //NetResponseStateControl();
        SetTimeFrame();
        NetRecOver();
        //NetErrAlarm();
        ReceiveInitial();
        //Rec2Action();       //给打印机发送数据
        Time();
        Event_Log();         //记录开关门事件
        if (f_No_WifiModule) //wifi模块不存在，认为接的是打印机
        {
            if (f_No_WifiModule == 1)
            {
                f_No_WifiModule = 2;
                UART0_Init_As_Printer(); //调整波特率为9600
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

void UART0_Init_As_Printer(void) //调整波特率为9600
{
    stc_mfs_uart_config_t stcUartConfig; //UART0
    stc_uart_irq_cb_t stcUart0IrqCb;

    PDL_ZERO_STRUCT(stcUartConfig);
    PDL_ZERO_STRUCT(stcUart0IrqCb);

    SetPinFunc_SIN0_1();
    SetPinFunc_SOT0_1();

    stcUart0IrqCb.pfnTxIrqCb = UART0_SEND_IRQHandler;
    stcUart0IrqCb.pfnRxIrqCb = UART0_RECV_IRQHandler;

    stcUartConfig.enMode = UartNormal;
    stcUartConfig.u32BaudRate = 9600; //wifi模块115200，打印机9600
    stcUartConfig.enDataLength = UartEightBits;
    stcUartConfig.enParity = UartParityNone;
    stcUartConfig.enStopBit = UartOneStopBit;
    stcUartConfig.enBitDirection = UartDataLsbFirst;
    stcUartConfig.bInvertData = FALSE;
    stcUartConfig.bHwFlow = FALSE;
    stcUartConfig.pstcFifoConfig = NULL;
    stcUartConfig.bUseExtClk = FALSE;
    stcUartConfig.pstcIrqEn = NULL;
    stcUartConfig.pstcIrqCb = &stcUart0IrqCb;
    stcUartConfig.bTouchNvic = TRUE;

    Mfs_Uart_Init(&UART0, &stcUartConfig);

    Mfs_Uart_DisableFunc(&UART0, UartRx); ////关闭接收
    Mfs_Uart_EnableFunc(&UART0, UartTx);  //使能发送

    Mfs_Uart_DisableIrq(&UART0, UartRxIrq); //关闭接受中断
    Mfs_Uart_DisableIrq(&UART0, UartTxIrq); //无效发送中断
}