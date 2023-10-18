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
#include "systick.h"

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
BitType myflag17;
BitType myflag18;
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

    /*--TEST CFJ
    f_test_alarm=ON;
    f_first_ad=1;
    r_voltage=80;
    r_lcad=98;
    r16_ldad=53; 
    r_lcwd = CheckLcTable(r_lcad);
    r_ldsjwd = (tab_temperature[r16_ldad-36]);
    //RS_485_STATUS=1; //发送有效
    */
     //-----
        for(;;)  
        {        
		
            IWDT_Clear(); ///// __RESET_WATCHDOG();                           /* feeds the dog */
            BuzzPrg();//检测凤鸣的状态，例如是叫一声还是连续叫
            PowerUpBuzzDelaylc();//判断系统上电后的时间是否大于了报警延时设定值，一旦大于以后，这个函数失效 f_Powerupdelaylc(冷藏)一直有效
#ifdef  Need_Ld
			 PowerUpBuzzDelayld();//判断系统上电后的时间是否大于了报警延时设定值，一旦大于以后，这个函数失效 f_Powerupdelayld(冷冻)一直有效
#endif
			ad_convert(); //需检查下，是否正确 CFJ  ？？ 得到环温实际温度r_hwsjwd
            WriteToE2();//根据f_need_write_e2判断是不是需要写E2
            IWDT_Clear();////  __RESET_WATCHDOG(); 
            WriteE2();//根据函数WriteToE2中的f_e2prom判断需不需要写，如果函数WriteToE2中将f_e2prom置位则需要写，包括一些报警上下限等

			//g_Pannel_Comm_bRcv_Done=1代表收到主控板数据，无误。接收完成数据处理  计算温度，拿到ld lc温度，拿到一次AD值以后f_first_ad一直=1
            Pannel_Comm_Deal();//DealRecData();从uart1拿485的数据
            JudgeErrs();//判断各传感器是否故障
            AlarmControl();//蜂鸣器报警和远程报警
            KeyPress();
            IWDT_Clear();//// __RESET_WATCHDOG(); 
            l_Pannel_DataTx();//每500ms向底板发送一次数据，包含各种控制信息等
            DealDispData();  //温度人为干预、数码管显示 冷藏 冷冻温度以及两个led灯的显示
            
            EheatConTrolP(); //HW_2019/4/24 10:35:28根据冷藏现在的温度控制EheatConTrolP的赋值，可能是控制压缩机
            DataToLed();  //根据确定下来的要显示的数据，显示到LED
            Compressor_on_delay();   //上电压机设定延迟时间 t_yj_delay分钟以后f_compressor_on_dly = 1
            Compressor_delay_10sec();    
            Lc_CompressorJudge();//冷藏压机控制以及各种条件判断
 #ifdef  Need_Ld
            LdCompressorJudge();

            LnFan();//冷冻冷凝风机控制逻辑
 #endif
            Lc_lightProg();//冷藏照明灯控制程序 判断开门60ms后开灯，否则关灯
//            Nd_fan_Prog();//内胆风机(蒸发风机)的开或关判断，关上门后判断是不是允许，如果允许就开起来
 #ifdef  Need_Ld           
            Defrost_Prog();//除霜，只有冷冻才需要除霜
#endif	
//            Door_Open();////wys11.03.19  判断门是不是开着，如果开着15分钟以后就报警
            //report_judge(); 
            IWDT_Clear();///__RESET_WATCHDOG(); 
            NetResponseStateControl();//usb部分
            SetTimeFrame(); //根据串口2发送过来的数据设置时间日期 usb
            NetRecOver();//串口2接收完数据的处理程序 usb
            NetRecOver1();//UART0接收完数据的处理程序 物联
            NetErrAlarm();//通过串口2发送错误数据 
            ReceiveInitial();
						ReceiveInitial1();
            Rec2Action();   //uart0    打印机口      
            MachineTimeStatistics();//判断压机，内风机 冷凝风机的总运行时间
            Time(); //各个定时变量的判断赋值(定时器1的中断BT1UnderflowIrqHandler)，系统中根据这些赋值进行状态变化及切换         
						 
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
