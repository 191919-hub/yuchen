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
#include "pdl_header.h"
#include "includes.h"
#include "Disp.H"   
#include "logicsub.H" 
#include "Program_Cfg.h"  //���������ã������ֹ����������������¶ȷֱ���


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

void SetupMaxCore_CLK(void)
{
  uint32_t u32IoRegisterRead;
  FM_CRG->SCM_CTL = 0;
  FM_CRG->BSC_PSR   = BSC_PSR_Val;                /* set System Clock presacaler */
  FM_CRG->APBC0_PSR = 0x00;              /* set APB0 presacaler */
  FM_CRG->APBC1_PSR = 0x80;              /* set APB1 presacaler */
  FM_CRG->APBC2_PSR = 0x80;              /* set APB2 presacaler */
  FM_CRG->SWC_PSR   = 0x00000003;   /* set SW Watchdog presacaler */
  FM_CRG->TTC_PSR   = 0;                /* set Trace Clock presacaler */

  FM_CRG->CSW_TMR   = 0x0000005C;                /* set oscillation stabilization wait time */
  
  //if (SCM_CTL_Val & (1UL << 1)) {                    /* Main clock oscillator enabled ? */
    FM_CRG->SCM_CTL |= (1UL << 1);                /* enable main oscillator */ 
    while (!(FM_CRG->SCM_STR & (1UL << 1)));      /* wait for Main clock oscillation stable */
  //}
  SystemExternalClock = 4* 1000000;
  FM_CRG->PSW_TMR   = 0x00000000;                                             /* set PLL stabilization wait time */

  

  FM_CRG->SCM_CTL  |= (0x00000020 & 0xE0);                                     /* Set Master Clock switch */ 

  do                                              
  {                                               
    u32IoRegisterRead = (FM_CRG->SCM_CTL & 0xE0);                              
  }while ((FM_CRG->SCM_STR & 0xE0) != u32IoRegisterRead);                      ///< wait for main clock stability
  SystemCoreClockUpdate();                                                      ///< get the system core clock value                                                    ///< get the system core clock value
}

void SetupCR_CLK(void)
{
  if( 0x000003FF != (FM_FLASH_IF->CRTRMM & 0x000003FF) )
  {
    /* UnLock (MCR_FTRM) */
    FM_CRTRIM->MCR_RLR = 0x1ACCE554;
    /* Set MCR_FTRM */
    FM_CRTRIM->MCR_FTRM = FM_FLASH_IF->CRTRMM;
    /* Lock (MCR_FTRM) */
    FM_CRTRIM->MCR_RLR = 0x00000000;
  }
}


void Delay(uint i)//(uint32_t i)
{

  while(i--);


}

/*****************************************************************/

int32_t main(void)
{
   //  11  ��Ƿ������� ,22 �ı�Ƿ���������
   //static unsigned int ad_reslut=0;
   /* 
   bFM_GPIO_PDOR5_P2=0;
   bFM_GPIO_PCR5_P2=0;                                                          //0��������ֹ��    1������ʹ��
   bFM_GPIO_DDR5_P2=1u;                                                         //0����ʼ��Ϊ���룻1,��ʼ��Ϊ�����
   bFM_GPIO_PFR5_P2=0u;                                                         //GPIO
   bFM_GPIO_ADE_AN24=0u;                                                        //0 GPIO  1 ADC
   bFM_GPIO_ADE_AN25=1u;                                                        //��ȷ���ĸ�Ad �����Ե�
   bFM_GPIO_ADE_AN26=1u; 
   */
   
   Main_ClkInit();
  
   GPIO_Init();   // ���巽��λ
   //1ms ��ʱ����ʼ��
   BT1_Init();
   
   Beep_Init();
   
   //PWM_Start();
   
   //Delay(50000);
     
   //PWM_Stop();
   //���ڳ�ʼ��
   //TX1,RX1  9600 --->mfs2
   MFS2_UART1_Init(); //USBͨѶ��ʼ��
  // Mfs_Uart_SendData(&UART2, 0x55); //����0x55����
   // �����жϺ���  Uart1RxCallback()��
   
   //TX2,RX2 9600 ----mfs0
   MFS0_UART0_Init();  //��ӡ��ͨѶ��ʼ��
   //Mfs_Uart_SendData(&UART0, 0x55); //����0x55����
   
   //TX,RX  2400  ----mfs1
   MFS1_UART_Init(); //UART ��ʼ�� 9600 ���ذ������ͨѶ ��ʼ��
   //Mfs_Uart_SendData(&UART1, 0x55); //����0x55����
   
   //AD ��ȡ����
    //ad_reslut=ADC_GetValue(25); @20181008 CFJ
   // ad_reslut=ADC_GetValue(26); @20181008 CFJ
    
    /* 
    //Buzz   ��������黹����ģ���ϱ�д����
    BT2_Init();
    BuzzEnable=1; //@20181016 CFJ
    Delay(50000);
    BuzzEnable=0; //@20181016 CFJ
    //DisableInterrupts; @20180920 cfj 
   // ClearRam();        @20180920 cfj 
   ////// Initial();
    //EnableInterrupts;              @20180920 cfj                  //enable interrupts 
    */
    //bUartSendStartFlag=1;  //��忪ʼ���ͱ�־λ   CFJ 
    bUartSendStartFlag=1;
    //g_Txd_Time = 250; @20190121 CFJ//�������500ms��������  CFJ
    ReadE2();
    InitialUserRegister();
    Hdware_Watchdog_Init();
    f_First_PowerOnFlag=ON;// �״��ϵ��������������� @21081120 CFJ //f_test_alarm=ON;//�ϵ�ȫ��88,����ҵ��ȷ�ϡ�@20181101 TEST  CFJ 
    bSelfUartSendStartFlag=1;
    l_Self_Detect(); //�����Լ���Գ���  @20190121 CFJ
    if(SelfCheckNoErrFlag)
    {
    	  BuzzBiBiBi();
    }
    else
    {
    	  BuzzBi();
    } 
    g_Txd_Time = 250;//�������500ms��������  CFJ @20190121 CFJ
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
            Clear_Watchdog(); ///// __RESET_WATCHDOG();                           /* feeds the dog */
            BuzzPrg();
            PowerUpBuzzDelaylc();
            PowerUpBuzzDelayld();
            ad_convert(); //�����£��Ƿ���ȷ CFJ  ����
            WriteToE2();
            Clear_Watchdog();////  __RESET_WATCHDOG(); 
            WriteE2();
            
            Pannel_Comm_Deal();//DealRecData();//����������ݴ���  �����¶�
            JudgeErrs();
            AlarmControl();
            KeyPress();
            Clear_Watchdog();//// __RESET_WATCHDOG(); 
            l_Pannel_DataTx();
            DealDispData();  //�¶���Ϊ��Ԥ���������ʾ
            
            EheatConTrolP();            //HW_2019/4/24 10:35:28
            DataToLed();  //������ʾ��LED
            Compressor_on_delay();   //�ϵ��趨�ӳ�ʱ��
            Compressor_delay_10sec();    
            Lc_CompressorJudge();
            LdCompressorJudge();
            LnFan();
            Lc_lightProg();//��������ƿ��Ƴ��� 
            Nd_fan_Prog();
            Defrost_Prog();
            Door_Open();////wys11.03.19 
            //report_judge(); 
            Clear_Watchdog();///__RESET_WATCHDOG(); 
            //NetResponseStateControl();
            SetTimeFrame(); 
            NetRecOver();
            //NetErrAlarm();
            ReceiveInitial();
            Rec2Action();                     
            Time();          
        } 
}
