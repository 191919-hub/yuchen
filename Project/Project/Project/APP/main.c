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
    SystemCoreClock = 7372800; //����������ڴ˴��޸ļ���
    DeviceClockAllEnable();

    User_SysTickInit();

    User_GPIO_Init(); // ���巽��λ

    User_AD_Init();
    
    T16N0Init();  //1ms ��ʱ����ʼ��

    UART3_USB_Init(); //USBģ��ͨѶ��ʼ��

    UART5_Wifi_Init(); //��׿��ͨѶ��ʼ�� ԭ����wifi

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

    /*--TEST CFJ
    f_test_alarm=ON;
    f_first_ad=1;
    r_voltage=80;
    r_lcad=98;
    r16_ldad=53; 
    r_lcwd = CheckLcTable(r_lcad);
    r_ldsjwd = (tab_temperature[r16_ldad-36]);
    //RS_485_STATUS=1; //������Ч
    */
     //-----
        for(;;)
        {        
            IWDT_Clear(); ///// __RESET_WATCHDOG();                           /* feeds the dog */
            BuzzPrg();//��������״̬�������ǽ�һ������������
            PowerUpBuzzDelaylc();//�ж�ϵͳ�ϵ���ʱ���Ƿ�����˱�����ʱ�趨ֵ��һ�������Ժ��������ʧЧ f_Powerupdelaylc(���)һֱ��Ч
#ifdef  Need_Ld
						PowerUpBuzzDelayld();//�ж�ϵͳ�ϵ���ʱ���Ƿ�����˱�����ʱ�趨ֵ��һ�������Ժ��������ʧЧ f_Powerupdelayld(�䶳)һֱ��Ч
#endif
						ad_convert(); //�����£��Ƿ���ȷ CFJ  ���� �õ�����ʵ���¶�r_hwsjwd
            WriteToE2();//����f_need_write_e2�ж��ǲ�����ҪдE2
            IWDT_Clear();////  __RESET_WATCHDOG(); 
            WriteE2();//���ݺ���WriteToE2�е�f_e2prom�ж��費��Ҫд���������WriteToE2�н�f_e2prom��λ����Ҫд������һЩ���������޵�
						//g_Pannel_Comm_bRcv_Done=1�����յ����ذ����ݣ����󡣽���������ݴ���  �����¶ȣ��õ�ld lc�¶ȣ��õ�һ��ADֵ�Ժ�f_first_adһֱ=1
            Pannel_Comm_Deal();//DealRecData();��uart1��485������
            JudgeErrs();//�жϸ��������Ƿ����
            AlarmControl();//������������Զ�̱���
            KeyPress();
            IWDT_Clear();//// __RESET_WATCHDOG(); 
            l_Pannel_DataTx();//ÿ500ms��װ巢��һ�����ݣ��������ֿ�����Ϣ��
            DealDispData();  //�¶���Ϊ��Ԥ���������ʾ ��� �䶳�¶��Լ�����led�Ƶ���ʾ
            
            EheatConTrolP(); //HW_2019/4/24 10:35:28����������ڵ��¶ȿ���EheatConTrolP�ĸ�ֵ�������ǿ���ѹ����
            DataToLed();  //����ȷ��������Ҫ��ʾ�����ݣ���ʾ��LED
            Compressor_on_delay();   //�ϵ�ѹ���趨�ӳ�ʱ�� t_yj_delay�����Ժ�f_compressor_on_dly = 1
            Compressor_delay_10sec();    
            Lc_CompressorJudge();//���ѹ�������Լ����������ж�
 #ifdef  Need_Ld
            LdCompressorJudge();

            LnFan();//�䶳������������߼�
 #endif
            Lc_lightProg();//��������ƿ��Ƴ��� �жϿ���60ms�󿪵ƣ�����ص�
//            Nd_fan_Prog();//�ڵ����(�������)�Ŀ�����жϣ������ź��ж��ǲ��������������Ϳ�����
 #ifdef  Need_Ld           
            Defrost_Prog();//��˪��ֻ���䶳����Ҫ��˪
#endif	
//            Door_Open();////wys11.03.19  �ж����ǲ��ǿ��ţ��������15�����Ժ�ͱ���
            //report_judge(); 
            IWDT_Clear();///__RESET_WATCHDOG(); 
            NetResponseStateControl();//usb����
            SetTimeFrame(); //���ݴ���2���͹�������������ʱ������ usb
            NetRecOver();//����2���������ݵĴ������ usb
            NetRecOver1();//UART0���������ݵĴ������ ����
            NetErrAlarm();//ͨ������2���ʹ������� 
            ReceiveInitial();
						ReceiveInitial1();
            Rec2Action();   //uart0    ��ӡ����      
            MachineTimeStatistics();//�ж�ѹ�����ڷ�� ���������������ʱ��
            Time(); //������ʱ�������жϸ�ֵ(��ʱ��1���ж�BT1UnderflowIrqHandler)��ϵͳ�и�����Щ��ֵ����״̬�仯���л�         
						 
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
