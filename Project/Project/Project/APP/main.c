/*************************************************************************************
*���ļ�¼��
//@20181016 CFJ
@20181120 TESTΪ������� �䶳������ʾ
@20181130 CFJ �������£������жϵ�صĵ�ѹ�������ܣ��ɵװ��жϣ����ֻ�жϵװ崫�����Ĺ���
@20181130 CFJ ѹ���� ����� �����ˣ���Ϊ˿ӡ�͵�Ƭ���Բ��ϣ����滹Ҫ�Ļ�ȥ��
@20181221 CFJ ��ҵ��������ԭ����NTC2��ΪNTC1 
@20181226 CFJ ���ӵ���ѹ����ʱ�������Ϊ3s������Ϊ����
@20190121 CFJ
@20190201 CFJ
@20190215 CFJ
@20190221 CFJ
@20190228 SW1�˳��е��ѹУ׼   CFJ
�������ļ�¼�μ��汾��¼.txt   ���˷�  2019-9-5                                                                                              
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
#include "Program_Cfg.h" //���������ã������ֹ����������������¶ȷֱ���
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
    SystemCoreClock = 7372800; //����������ڴ˴��޸ļ���
    DeviceClockAllEnable();

    User_SysTickInit();

    User_GPIO_Init(); // ���巽��λ

    User_AD_Init();
    
    T16N0Init();  //1ms ��ʱ����ʼ��

    UART3_USB_Init(); //USBģ��ͨѶ��ʼ��

    UART5_Wifi_Init(); //wifiģ����ӡ��ͨѶ��ʼ��

    UART4_PwrBoard_Init(); //UART ��ʼ�� 9600 ���ذ������ͨѶ ��ʼ��

    bUartSendStartFlag = 1;

    ReadE2();
    ReadCfgData(); //��flash�ж�ȡwifi�������������ݣ�����SSID/����/BE�����Ϣ
    InitialUserRegister();
    WdtInit();
    f_First_PowerOnFlag = ON; // �״��ϵ��������������� @21081120 CFJ //f_test_alarm=ON;//�ϵ�ȫ��88,����ҵ��ȷ�ϡ�@20181101 TEST  CFJ
    bSelfUartSendStartFlag = 1;
    l_Self_Detect(); //�����Լ���Գ���  @20190121 CFJ
    if (SelfCheckNoErrFlag)
    {
        BuzzBiBiBi();
    }
    else
    {
        BuzzBi();
    }
    g_Txd_Time = 250; //�������500ms��������  CFJ @20190121 CFJ

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

        Pannel_Comm_Deal(); //����������ݴ���  �����¶�
        JudgeErrs();
        AlarmControl();
        KeyPress();
        IWDT_Clear(); 
        l_Pannel_DataTx();
        DealDispData(); //�¶���Ϊ��Ԥ���������ʾ
        DataToLed();           //������ʾ��LED
        Compressor_on_delay(); //�ϵ��趨�ӳ�ʱ��
        Compressor_delay_10sec();
        Lc_CompressorJudge();
        LdCompressorJudge();
        LnFan();
        Lc_lightProg(); //��������ƿ��Ƴ���
        Nd_fan_Prog();
        Defrost_Prog();
        Door_Open(); ////wys11.03.19
        IWDT_Clear();
        SetTimeFrame();
        NetRecOver();
        ReceiveInitial();
        Time();
        Event_Log();         //��¼�������¼�
        if (f_No_WifiModule) //wifiģ�鲻���ڣ���Ϊ�ӵ��Ǵ�ӡ��
        {
            if (f_No_WifiModule == 1)
            {
                f_No_WifiModule = 2;
                UART5_Init_As_Printer(); //����������Ϊ9600
            }
            else if (u8_Send_Print_Time >= 2)
            {
                ReturnPrintFrame();
                u8_Send_Print_Time = 0;
            }
        }
        else
        {
            /*wifi���ͺͽ��մ���*/
            WIFI_RX_Data_Deal();
            if (g_1s_flag_Iot)
            {
                g_1s_flag_Iot = 0;
                WIFI_TX_Data();
            }
            /*wifi���ͺͽ��մ���*/
        }
    }
}

void UART5_Init_As_Printer(void) //����������Ϊ9600
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