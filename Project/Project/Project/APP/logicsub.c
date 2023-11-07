/*
 ����.c�ļ�
*/
/****************************************/
/*                                      */
/*    Filename:       HYCD-282.c        */
/*    MCU:            JM60              */
/*    Frequency:      4.0M              */
/*    Date:           2010.4.1          */
/*    File Version:   VER 1.0           */
/*    Author:         SONGBINGZHAO      */
/*    Company:        hrkj              */
/*                                      */
/****************************************/
// 100526  zyj ������ڵ����ͣת
// 140915 �ۼ�6Сʱ��ͣ��13����
// 141016 �ۼ�6Сʱ��ͣ��18���ӣ���ѹ��ͣ��ִ��
//#include <hidef.h> /* for EnableInterrupts macro */
//#include "derivative.h" /* include peripheral declarations */
#include "Disp.H"
#include "logicsub.h"
//#include "pdl_header.h"
#include "math.h"
#include "Program_Cfg.h" //���������ã������ֹ����������������¶ȷֱ���
#include "Typedefine.h"
#include "includes.h"
#include "Coupler.h"
#include "esp8266.h"
#include "lib_adc.h"
#include "lib_uart.h"
#include "lib_wdt.h"
//#define nop asm("nop")

//uchar u8_Send_Print_Time;

//#pragma CODE_SEG DEFAULT

#define HEATONAD 2130		 //2.5��
#define HEATOFFAD 2110		 //3.0��
#define LCCOMPRTSTOREAD 1869 //7.8��

//�����������ػ�˪��������ʾ�¶�(����ĳ���¶Ȼ�˪)���������������(�ж�ѹ�������)
float r_lcwd_float = 0, r_lcgzwd_float = 0, r_lczt_float = 0, r_lcxswd_float = 0,r_lcHuaSHuangxswd_float=0,r_lcLengNingxswd_float=0,r_lcwd_float_c = 0;
unsigned char flag_Now_Disp_LCWD = 0, flag_Disp_LCWD_D = 0;
unsigned char Cnt_SetTime_Sended = 0;

unsigned int  lc_compressor_times=0;
unsigned int  lc_fan_times=0;
unsigned int  lc_nd_fan_times=0;

/***********************************************************
*Name         :  InitialUserRegister
*Function     :  Initial User's Register
*Input Value  :  None
*Output Value :  None
*Introduction :
***********************************************************/
void InitialUserRegister(void)
{
	F_StopResponse = 1;
	R_HomeNetResponseCheckTime = 10;
	u16_random = 0x00000001;
}
/**********************************************************/
void Time(void)
{

	if (f_2Ms_Flag) //2ms��־  //CFJ
	{
		f_2Ms_Flag = 0;
		/*
             if(g_Rec_Status_Flag)
             {
                 g_Bus_Rec_Time++;
                 if(g_Bus_Rec_Time>=250) //500ms
                 {
                    g_Bus_Rec_Time=0;
                    bUartSendStartFlag=1;
                    g_Txd_Time=0;
                    g_Rec_Status_Flag=0;
                    //Mfs_Uart_EnableFunc(&UART1, UartTx);    //ʹ�ܷ��ͽ���
                    //Mfs_Uart_DisableFunc(&UART1, UartRx); //����ʹ����Ч
                 }            
             }
             */
		if (g_Bus_Error_Time >= 30000) //תΪ����״̬��1min��δ�յ����ݣ��򱨹���
		{
			g_Sys_Erflag0_Comm = 1; //ͨѶ���� @20181025 CFJ
		}
		else
		{
			g_Bus_Error_Time++;
		}
	}

	if (f_05s)
	{
		if (!f_halfsec)
		{
			f_halfsec = 1;
			u8_Send_Print_Time++;
		}
		else
		{
			f_05s = 0;
			f_halfsec = 0;
		}
		
	}

	if (f_halfsec)
	{
		t_1min++;
		if (t_1min >= 120)
		{
			t_1min = 0;
			f_1min = 1;
		}
		
	}

	if (f_1min) //1min
	{
		if (!f_onemin)
		{
			f_onemin = 1;
		}
		else
		{
			f_1min = 0;
			f_onemin = 0;
		}
	}
	if (f_halfsec)
	{
		t_10s++;
		if (t_10s >= 20)
		{
			t_10s = 0;
			t_tens++;
			f_10s = 1;
		}
	}
	if (f_10s) //10s
	{
		if (!f_tens)
		{
			f_tens = 1;
		}
		else
		{
			f_10s = 0;
			f_tens = 0;
		}
	}
}
//********�ж��ϵ��������ӳٱ�������**********
void PowerUpBuzzDelaylc(void)
{
	if (f_Powerupdelaylc)
	{
		return;
	}
	if (f_onemin)
	{//��forѭ�����жϣ��������Ϊ����ӷ�תһ��u8_BuzzDlyMinlcһ���Ӽ�1
		u8_BuzzDlyMinlc++;
		if (u8_BuzzDlyMinlc >= 60)
		{
			u8_BuzzDlyMinlc = 0;
			u8_BuzzDlyHourlc++;
		}
	}

	if (u8_BuzzDlyHourlc >= u8_lc_bjycsj) //�ϵ籨����ʱ�����趨����ʱʱ��
	{
		f_Powerupdelaylc = 1;
	}
}
//**********************************************/
void PowerUpBuzzDelayld(void)
{
	if (f_Powerupdelayld)
	{
		return;
	}
	if (f_onemin)
	{
		u8_BuzzDlyMinld++;
		if (u8_BuzzDlyMinld >= 60)
		{
			u8_BuzzDlyMinld = 0;
			u8_BuzzDlyHourld++;
		}
	}

	if (u8_BuzzDlyHourld >= u8_ld_bjycsj)
	{
		f_Powerupdelayld = 1;
	}
}
//**********************************************
//********************����������****************
//**********************************************
void BuzzPrg(void)
{
	if (f_bi) //��һ��
	{
		if ((uchar)(t_onems - t_buzz) >= 200) //200ms
		{
			goto Tfmq;
		}
	}
	else if (f_bibibi) //������,��100ms,ͣ50ms
	{
		if (r_buzz == 0)
		{
			if ((unsigned char)(t_onems - t_buzz) >= 100) //100ms
			{
				r_buzz = 1;
				t_buzz = t_onems;
				goto Tfmq;
			}
		}
		else if (r_buzz == 1)
		{
			if ((unsigned char)(t_onems - t_buzz) >= 50) //50ms
			{
				r_buzz = 2;
				t_buzz = t_onems;
				PWM_Start();
				//TPM1SC = 0x00;  @20181008 CFJ
				//TPM1MOD = 0x0176;   @20181008 CFJ
				//(void)(TPM1C3SC == 0);

				//TPM1C3SC = 0x28;          @20181008 CFJ
				//TPM1C3V = 0xBB;
				//(void)(TPM1SC == 0);
				//TPM1SC = 0x0C;
			}
		}
		else if (r_buzz == 2)
		{
			if ((unsigned char)(t_onems - t_buzz) >= 100) //100ms
			{
				r_buzz = 3;
				t_buzz = t_onems;
				goto Tfmq;
			}
		}
		else if (r_buzz == 3)
		{
			if ((unsigned char)(t_onems - t_buzz) >= 50) //50ms
			{
				r_buzz = 4;
				t_buzz = t_onems;
				PWM_Start();
				//TPM1SC = 0x00;       @20181008 CFJ
				//TPM1MOD = 0x0176;
				//(void)(TPM1C3SC == 0);     @20181008 CFJ

				//TPM1C3SC = 0x28;
				//TPM1C3V = 0xBB;
				//(void)(TPM1SC == 0);
				//TPM1SC = 0x0C;
			}
		}
		else if (r_buzz == 4)
		{
			if ((unsigned char)(t_onems - t_buzz) >= 100) //100ms
			{
				r_buzz = 0;
				goto Tfmq1;
			}
		}
		else
		{
			goto Tfmq1;
		}
	}
	else
	{
		goto Tfmq1;
	}
	return;
Tfmq1:
	f_bibibi = 0;//������
Tfmq:
	f_bi = 0;//��һ��
	PWM_Stop();
	//TPM1SC = 0x00;    @20181008 CFJ
	//TPM1C3SC = 0x00;   @20181008 CFJ
}
//********************�����������**************
//**********************************************
void BuzzBi(void)
{
	f_bi = 1;
	t_buzz = t_onems;
	PWM_Start();
	//TPM1SC = 0x00;                               /* Stop and reset counter */ @20181008 CFJ
	// TPM1MOD = 0x0176;                            /* Period value setting */  @20181008 CFJ
	//(void)(TPM1C3SC == 0);                       /* Channel 0 int. flag clearing (first part) */  @20181008 CFJ
	/* TPM1C3SC: CH3F=0,CH3IE=0,MS3B=1,MS3A=0,ELS3B=1,ELS3A=0 */		 //@20181008 CFJ
																		 //TPM1C3SC = 0x28;                             /* Int. flag clearing (2nd part) and channel 0 contr. register setting */  @20181008 CFJ
																		 // TPM1C3V = 0xBB;                              /* Compare 0 value setting */   @20181008 CFJ
																		 //(void)(TPM1SC == 0);                         /* Overflow int. flag clearing (first part) */  @20181008 CFJ
	/* TPM1SC: TOF=0,TOIE=0,CPWMS=0,CLKSB=0,CLKSA=1,PS2=1,PS1=0,PS0=0 */ //@20181008 CFJ
																		 // TPM1SC = 0x0C;                               /* Int. flag clearing (2nd part) and timer control register se*/  @20181008 CFJ
}
//^^^^^^^^^^^^^^^^
void BuzzBiBiBi(void)
{
	f_bibibi = 1;
	r_buzz = 0;
	t_buzz = t_onems;
	PWM_Start();
	//TPM1SC = 0x00;                 @20181008 CFJ
	//TPM1MOD = 0x0176;             @20181008 CFJ
	//(void)(TPM1C3SC == 0);         @20181008 CFJ

	// TPM1C3SC = 0x28;               @20181008 CFJ
	// TPM1C3V = 0xBB;               @20181008 CFJ
	//(void)(TPM1SC == 0);          @20181008 CFJ

	//TPM1SC = 0x0C;              @20181008 CFJ
}
/****************************************************/
/*********************ADת������*********************/
/****************************************************/

/**
 * @brief adת��Ϊ�¶�
 *
 * 
 *
 * @param  Ad   12λad
 * @retval ʵ���¶�+38
 */
float CheckLcTable_12b(unsigned int Ad)
{
    unsigned int i = 0;
    float Temp;
    if ((float)Ad >= table_lc_12b[0][0])
    {
        return table_lc_12b[0][1] + 38; //-30��
    }
    else if ((float)Ad <= table_lc_12b[18][0])
    {
        return table_lc_12b[18][1] + 38; //60��
    }
    else
    {
        while ((float)Ad < table_lc_12b[i][0])
            i++;
        Temp = 38 + table_lc_12b[i - 1][1] + 5 * (Ad - table_lc_12b[i - 1][0]) / (table_lc_12b[i][0] - table_lc_12b[i - 1][0]); //�������¶�
    }
    return Temp;
}

unsigned char SingleAd(unsigned char channel);
unsigned char CheckLcTable(unsigned char ad_8b);
float CheckLcTable_12b(unsigned int ad_12b);
uint16_t ADCRead(ADC_TYPE_CHS chs);
/****************************************************/
void ad_convert(void)
{
	uchar adtemp;
    uchar DeltaAD;  //����ad��ֵ

	if ((unsigned char)(t_onems - t_ad) >= 200 || !f_first_ad)
	{
		t_ad = t_onems;
		adtemp = r_hwadtemp;
		r_hwadtemp = ADCRead(ADC_CHS_AIN5) / 16; 
		if (r_hwadtemp >= adtemp)
		{
			DeltaAD = r_hwadtemp - adtemp;
		}
		else
		{
			DeltaAD = adtemp - r_hwadtemp;
		}
		if (DeltaAD <= 5)
		{
			r_hwad = r_hwadtemp;
			r_hwsjwd = CheckLcTable(r_hwad);
		}
		else
		{
			return;
		}
	}
}

void ADCSetCHS(ADC_TYPE_CHS chs) ;
/*********************************************************
������: ADCRead
��  ��: ADC�������ݶ�ȡ(16�β���ƽ��ֵ)(�������)
����ֵ: ��
���ֵ: ��
����ֵ: ƽ��ֵ
**********************************************************/
uint16_t ADCRead(ADC_TYPE_CHS chs)
{
    uint8_t i;
    uint32_t max, min, sum, ad_temp;

    max = 0x00;
    min = 0x0FFF;
    sum = 0x00;

    ADCSetCHS(chs);

    for (i=0; i<18; i++)
    {	
		ADC_Start();
        while(ADC_GetFlagStatus(ADC_IF) == RESET);
        ad_temp = ADC_GetConvValue();
        ADC_ClearIFStatus(ADC_IF);
		Delay_100us(10);
        if (ad_temp > max)
            max = ad_temp;
        if (ad_temp < min)
            min = ad_temp;
        sum += ad_temp;
    }
    sum -= min;
    sum -= max;

    return sum >>4;
}

/*********************************************************
������: ADCSetCHS
��  ��: ADC���ò����ŵ�
����ֵ: ��
���ֵ: ��
����ֵ: ƽ��ֵ
**********************************************************/
void ADCSetCHS(ADC_TYPE_CHS chs)   
{
    ADC->CHS.CHS = chs;
}

unsigned char CheckLcTable(unsigned char ad_8b)
{
	if (ad_8b > 223)
	{
		return (8);
	}
	else if (ad_8b <= 23)
	{
		return (98);
	}
	else
	{
		return (table_lc[(unsigned char)(ad_8b - 24)]);
	}
}
/*********************************************************
������: ADC2Temper
��  ��: ADCת��Ϊ�¶�ֵ
����ֵ: ��
���ֵ: ��
����ֵ: ƽ��ֵ
**********************************************************/
unsigned short ADC2Temper_3840(uint16_t data)
{
    unsigned short  i,temp;

   for(i=0;i<160;i++)
    {
        if((table_ld_3840[i] > data && table_ld_3840[i+1] < data)
           || (table_ld_3840[i] == data))
        {
            temp = ((table_ld_3840[i] - data) * 10) / table_ld_3840_s[i];
			return((i - 40) * 10 + temp + 510);//��ƫ510����֤����������-40�ȶ�Ӧ����110 
        }
    }

	return 1740;//�д���ʱ��û�����ADֵ������1740,ʵ��Ϊ(1740-510)/10��
	//�õ�ʵ���¶ȵ�˼·���������ֵ<510�Ǹ�������������Ǹ�����
	//���統����ֵΪ400ʱ����Ӧ�¶�ӦΪ-(510-400)/10��(-11��),������ֵΪ600ʱ����Ӧ�¶�Ϊ+(600-510)/10��(+9��)
}


/****************************************************/
/********************��ʾ����************************/
/****************************************************/
void RuleForLdDisp(void);
void RuleForLcDisp(void);
void LdDisp(void);
void LcDisp(void);
void AlarmAndLockLed(void);
/**********************************************/
void DealDispData(void)
{
	RuleForLdDisp();//�䶳�������ʾ�߼�

#if (LC_DISP_RESOLUTION_0D1 || LC_RESOLUTION_0D1) //��ʾ�ֱ���0.1��
	RuleForLcDisp_0D1();						  //��ʾ�¶��˹���Ԥ��������ʵ���¶�  (�ֱ���1��)
#else
	RuleForLcDisp(); //��ʾ�¶��˹���Ԥ��������ʵ���¶�   (�ֱ���1��)
#endif
	LcDisp(); //��ÿ���������ʾ����ȷ������
	LdDisp();//����ط�Ҫ�ĳ�ֻ��ʾʪ�ȣ���������������
	AlarmAndLockLed();//����LED�ƵĴ���
}
/****************************************************/
/********************�䶳��ʾ����********************/
/****************************************************/
void RuleForLdDisp(void)
{
	if (!f_first_ad) //û�����1��AD�����ͷ���
	{
		return;
	}
#ifdef Need_Ld
	
	if (!f_ld_first_disp)
	{
		f_ld_first_disp = 1;//û�й��ϵĻ�����һֱ������
		r_ldgzwd = r_ldsjwd;
		t_ld_rule = t_halfsec;
	}
	//ʵ���¶�>�趨�¶�+1����ʵ���¶�<�趨�¶�
	else if ((r_ldsjwd > (unsigned char)(r_ldzt + 1)) || (r_ldsjwd < r_ldzt)) //
	{
		if ((unsigned char)(t_halfsec - t_ld_rule) >= 120) //60s HW 20190810
		{
			t_ld_rule = t_halfsec;
			if (r_ldsjwd > r_ldgzwd)
			{
				r_ldgzwd++;
			}
			else if (r_ldsjwd < r_ldgzwd)
			{
				r_ldgzwd--;
			}
		}
	}
	else
	{
		if ((r_ldgzwd > (unsigned char)(r_ldzt + 1)) || (r_ldgzwd < r_ldzt))
		{
			//u8_LdRule = t_tens; @20181130 CFJ
			if ((unsigned char)(t_halfsec - t_ld_rule) >= 120) //60s HW 20190810
			{
				t_ld_rule = t_halfsec;
				if (r_ldgzwd > r_ldzt)
				{
					r_ldgzwd--;
				}
				else if (r_ldgzwd < r_ldzt)
				{
					r_ldgzwd++;
				}
			}
		}
	}
	r_ldxswd = r_ldgzwd;
	//����˵������ط��ǵڶ�����ʾ����������������������������ʵ����ʾ�¶Ⱦ���r_ldxswd
	if ((r_hwsjwd >= 67) && (r_ldzt <= 170)) //�������¶ȴ���29�����趨�¶�С��-30��
	{
		r_ldxswd = r_ldxswd - 2; //��ʾֵ-2
	}
	else if (r_ldzt <= 170)
	{
		r_ldxswd = r_ldxswd - 1; //��ʾֵ-1
	}
  #else//��ʾʪ��ֵ
  if (!f_ld_first_disp)
  {//ϵͳ��һ������
		f_ld_first_disp = 1;//û�й��ϵĻ�����һֱ������
		t_ld_rule = t_halfsec;
  }
  if(LcHuaShuang_Flag){//��ǰ���ڻ�˪
     if(r_ldgzwd!=r_ldsjwd){
  	   if ((unsigned char)(t_halfsec - t_ld_rule) >= 120){ //60s HW 20190810
		t_ld_rule = t_halfsec;
		if (r_ldsjwd > r_ldgzwd)
		{
			r_ldgzwd++;
		}
		else if (r_ldsjwd < r_ldgzwd)
		{
			r_ldgzwd--;
		}
      }
    }else{
    	
    }
  }else{
		
		r_ldgzwd = r_ldsjwd;
		t_ld_rule = t_halfsec;
  }
  r_ldxswd=r_ldgzwd;
  #endif
}

/****************************************************/
/********************�����ʾ����********************/
/****************************************************/
//void RuleForLcDisp(void)
//{
//	if (!f_first_ad) //��δ�����¶�
//	{
//		return;
//	}
//	if (!f_lc_first_disp) //�״���ʾ
//	{
//		f_lc_first_disp = 1;
//		r_lcgzwd = r_lcwd;	 //��ʾʵ���¶�
//		t_lc_rule = t_halfsec; //������ʾˢ��ʱ���
//		u8_LcRule = t_tens;
//	}
//	else if ((r_lcwd > (uchar)(r_lczt + 2)) || (r_lcwd < (uchar)(r_lczt - 1))) //ʵ���¶ȸ��������¶ȳ���2��  ����  ���������¶ȳ���1��
//	{
//		{
//			//u8_LcRule = t_tens; @20181130 CFJ
//			if ((uchar)(t_halfsec - t_lc_rule) >= 120) //60�� @20181130 CFJ
//			{
//				t_lc_rule = t_halfsec;
//				if (r_lcwd > r_lcgzwd) //��ʾ�¶�<ʵ���¶�
//				{
//					r_lcgzwd++; //��ʾ�¶�++
//				}
//				else if (r_lcwd < r_lcgzwd)
//				{
//					r_lcgzwd--;
//				}
//			}
//		}
//	}
//	else
//	{
//		if (r_lcgzwd != r_lczt)
//		{
//			// t_lc_rule = t_halfsec;
//			if ((uchar)(t_halfsec - t_lc_rule) >= 120) //60S�� @20181120
//			{
//				t_lc_rule = t_halfsec;
//				if (r_lcgzwd > r_lczt) //��ʾ�¶�>ʵ���¶�
//				{
//					r_lcgzwd--; //��ʾ�¶�--
//				}
//				else if (r_lcgzwd < r_lczt)
//				{
//					r_lcgzwd++;
//				}
//			}
//		}
//	}
//	r_lcxswd = r_lcgzwd;
//}

/****************************************************/
/********************�����ʾ���򣨷ֱ���0.1�ȣ�********************/
/****************************************************/
void RuleForLcDisp_0D1(void)
{
	if (!f_first_ad) //��δ�����¶�
	{
		return;
	}
	if (!f_lc_first_disp) //�״���ʾ
	{
		f_lc_first_disp = 1;
		r_lcgzwd_float = r_lcwd_float; //��ʾʵ���¶�
		t_lc_rule = t_halfsec;		   //������ʾˢ��ʱ���
		u8_LcRule = t_tens;
	}
	else   //���䵽ʵ���¶�
	{
        if ((t_halfsec - t_lc_rule) >= 12) //6��
        {
            t_lc_rule = t_halfsec;

            if (fabs(r_lcwd_float - r_lcgzwd_float) < 0.1)
                r_lcgzwd_float = r_lcwd_float;
            else
            {
                if (r_lcwd_float > r_lcgzwd_float) //��ʾ�¶�<ʵ���¶�
                {
                    r_lcgzwd_float += 0.1; //��ʾ�¶�++
                }
                else if (r_lcwd_float < r_lcgzwd_float)
                {
                    r_lcgzwd_float -= 0.1;
                }
            }
        }
	}
	
	r_lcxswd_float = r_lcgzwd_float;
	r_lcgzwd = (unsigned char)floor(r_lcgzwd_float); //��Щ�ط�����Ҫ�õ�r_lcgzwd,����ѹ�����������б����Ĺ�����Ҫ�ж�r_lcgzwd�Ƿ���������¶ȣ�����ȡr_lcgzwd_float���������ָ�r_lcgzwd
}

/****************************************************/
/********************�����ʾ���򣨷ֱ���0.1�ȣ�********************/
/****************************************************/
//void RuleForLcDisp_0D2(void)
//{
//	if (!f_first_ad) //��δ�����¶�
//	{
//		return;
//	}
//	if (!f_lc_first_disp) //�״���ʾ
//	{
//		f_lc_first_disp = 1;
//		r_lcgzwd_float = r_lcwd_float; //��ʾʵ���¶�
//		t_lc_rule = t_halfsec;		   //������ʾˢ��ʱ���
//		u8_LcRule = t_tens;
//	}
//	else if ((r_lcwd_float > (r_lczt_float + 2)) || (r_lcwd_float < (r_lczt_float - 1))) //ʵ���¶ȸ��������¶ȳ���2��  ����  ���������¶ȳ���1��
//	{
//		{
//			//u8_LcRule = t_tens; @20181130 CFJ
//			if ((uchar)(t_halfsec - t_lc_rule) >= 12) //6��
//			{
//				t_lc_rule = t_halfsec;

//				if (fabs(r_lcwd_float - r_lcgzwd_float) < 0.1)
//					r_lcgzwd_float = r_lcwd_float;
//				else
//				{
//					if (r_lcwd_float > r_lcgzwd_float) //��ʾ�¶�<ʵ���¶�
//					{
//						r_lcgzwd_float += 0.1; //��ʾ�¶�++
//					}
//					else if (r_lcwd_float < r_lcgzwd_float)
//					{
//						r_lcgzwd_float -= 0.1;
//					}
//				}
//			}
//		}
//	}
//	else
//	{
//		if (r_lcgzwd_float != r_lczt_float) //����ʾ�¶� ������ �����¶�
//		{
//			// t_lc_rule = t_halfsec;
//			if ((uchar)(t_halfsec - t_lc_rule) >= 12) //6S
//			{
//				float xxx = 0;
//				t_lc_rule = t_halfsec;
//				xxx = fabs(r_lcgzwd_float - r_lczt_float);
//				if (xxx < 0.1)
//					r_lcgzwd_float = r_lczt_float;
//				else
//				{
//					if (r_lcgzwd_float > r_lczt_float) //��ʾ�¶�>ʵ���¶�
//					{
//						r_lcgzwd_float -= 0.1; //��ʾ�¶�--
//					}
//					else if (r_lcgzwd_float < r_lczt_float)
//					{
//						r_lcgzwd_float += 0.1;
//					}
//				}
//			}
//		}
//	}
//	r_lcxswd_float = r_lcgzwd_float;//������ʾʹ�õ�ֵ
//	r_lcgzwd = (unsigned char)floor(r_lcgzwd_float); //��Щ�ط�����Ҫ�õ�r_lcgzwd,����ѹ�����������б����Ĺ�����Ҫ�ж�r_lcgzwd�Ƿ���������¶ȣ�����ȡr_lcgzwd_float���������ָ�r_lcgzwd
//}

/****************************************************/
/********************�䶳��ʾ����********************/
/****************************************************/
void LdDisp(void)
{
	unsigned char r_bw=0, r_sw=0, r_gw=0;

	if (!f_first_ad)
	{
		return;
	}
	if (f_test_alarm || f_First_PowerOnFlag) //@20181120 CFJ
	{
		r_led11 = 0xff;
		r_led12 = 0xff;
		r_led13 = 0xff;
		return;
	}
	else if (r_set_state1 == SET_F0)
	{
		if (r_flash_bit == BIT3)
		{
			r_bw = DISP_NO; //����ʾ
			r_sw = DISP_F;
			if (t_halfsec & 0x01) //0b00000001)  @20181008 CFJ
			{
				r_gw = 0;
			}
			else
			{
				r_gw = DISP_NO;
			}
		}
	}
	else if (r_set_state1 == SET_F1)
	{
		if (r_flash_bit == BIT3)
		{
			r_bw = DISP_NO;
			r_sw = DISP_F;
			if (t_halfsec & 0x01) //0b00000001)  @20181008 CFJ
			{
				r_gw = 1;
			}
			else
			{
				r_gw = DISP_NO;
			}
		}
	}
	else if (r_set_state == SET_LD_L0)
	{
		if (r_flash_bit == BIT3)
		{
			r_bw = DISP_NO;
			r_sw = DISP_L;
			if (t_halfsec & 0x01) //0b00000001)  @20181008 CFJ
			{
				r_gw = 0;
			}
			else
			{
				r_gw = DISP_NO;
			}
		}
		else if (r_flash_bit == BIT2)
		{
			r_bw = DISP_NO;
			r_gw = 0;
			if (t_halfsec & 0x01) //0b00000001)  @20181008 CFJ
			{
				r_sw = DISP_L;
			}
			else
			{
				r_sw = DISP_NO;
			}
		}
	}
	else if (r_set_state == SET_LD_L1)
	{
		if (r_flash_bit == BIT3)
		{
			r_bw = DISP_NO;
			r_sw = DISP_L;
			if (t_halfsec & 0x01) //0b00000001)  @20181008 CFJ
			{
				r_gw = 1;
			}
			else
			{
				r_gw = DISP_NO;
			}
		}
	}
	else if (r_set_state == SET_LD_L2)
	{
		if (r_flash_bit == BIT3)
		{
			r_bw = DISP_NO;
			r_sw = DISP_L;
			if (t_halfsec & 0x01) //0b00000001)  @20181008 CFJ
			{
				r_gw = 2;
			}
			else
			{
				r_gw = DISP_NO;
			}
		}
	}
	else if (r_set_state == SET_LD_L3)
	{
		if (r_flash_bit == BIT3)
		{
			r_bw = DISP_NO;
			r_sw = DISP_L;
			if (t_halfsec & 0x01) //0b00000001)  @20181008 CFJ
			{
				r_gw = 3;
			}
			else
			{
				r_gw = DISP_NO;
			}
		}
	}
	else if (r_set_state == SET_LD_L4)
	{
		if (r_flash_bit == BIT3)
		{
			r_bw = DISP_NO;
			r_sw = DISP_L;
			if (t_halfsec & 0x01) //0b00000001)  @20181008 CFJ
			{
				r_gw = 4;
			}
			else
			{
				r_gw = DISP_NO;
			}
		}
	}
	else if (r_set_state == SET_LD_L5)
	{
		if (r_flash_bit == BIT3)
		{
			r_bw = DISP_NO;
			r_sw = DISP_L;
			if (t_halfsec & 0x01) //0b00000001)  @20181008 CFJ
			{
				r_gw = 5;
			}
			else
			{
				r_gw = DISP_NO;
			}
		}
	}
	else if (r_set_state == SET_LD || r_set_state == SET_LD_HIGH || r_set_state == SET_LD_LOW)
	{
		if(r_set_state == SET_LD_HIGH)//���ܵ�100
		{
			if (r_flash_bit == BIT1)
			{
				if (f_zf == ZHENG_SIGN)
				{
					if (t_halfsec & 0x01) //0b00000001)  @20181008 CFJ
					{
						r_bw = r_bwtj;
					}
					else
					{
						r_bw = DISP_NO;
					}
				}
				r_sw = r_swtj;
				r_gw = r_gwtj;
			}
			else if (r_flash_bit == BIT2)
			{
				if (f_zf == ZHENG_SIGN)
				{
					r_bw = r_bwtj;;
				}
				else
				{
					r_bw = DISP_FH;
				}
				if (t_halfsec & 0x01) //0b00000001)  @20181008 CFJ
				{
					r_sw = r_swtj;
				}
				else
				{
					r_sw = DISP_NO;
				}
				r_gw = r_gwtj;
			}
			else if (r_flash_bit == BIT3)
			{
				if (f_zf == ZHENG_SIGN)
				{
					r_bw = r_bwtj;;
				}
				else
				{
					r_bw = DISP_FH;
				}
				r_sw = r_swtj;
				if (t_halfsec & 0x01) //0b00000001)  @20181008 CFJ
				{
					r_gw = r_gwtj;
				}
				else
				{
					r_gw = DISP_NO;
				}
			}
		}
		else
		{
			if (r_flash_bit == BIT1)
			{
				if (f_zf == ZHENG_SIGN)
				{
					if (t_halfsec & 0x01) //0b00000001)  @20181008 CFJ
					{
						r_bw = DISP_ZH;
					}
					else
					{
						r_bw = DISP_NO;
					}
				}
				else
				{
					if (t_halfsec & 0x01) //0b00000001)  @20181008 CFJ
					{
						r_bw = DISP_FH;
					}
					else
					{
						r_bw = DISP_NO;
					}
				}
				r_sw = r_swtj;
				r_gw = r_gwtj;
			}
			else if (r_flash_bit == BIT2)
			{
				if (f_zf == ZHENG_SIGN)
				{
					r_bw = DISP_NO;
				}
				else
				{
					r_bw = DISP_FH;
				}
				if (t_halfsec & 0x01) //0b00000001)  @20181008 CFJ
				{
					r_sw = r_swtj;
				}
				else
				{
					r_sw = DISP_NO;
				}
				r_gw = r_gwtj;
			}
			else if (r_flash_bit == BIT3)
			{
				if (f_zf == ZHENG_SIGN)
				{
					r_bw = DISP_NO;
				}
				else
				{
					r_bw = DISP_FH;
				}
				r_sw = r_swtj;
				if (t_halfsec & 0x01) //0b00000001)  @20181008 CFJ
				{
					r_gw = r_gwtj;
				}
				else
				{
					r_gw = DISP_NO;
				}
			}
		}
	}
	else if (r_set_state == LD_CHK_VOL) //��ʾ�����ĵ�ѹ
	{
		if (r_voltage / 100 == 0)
		{
			r_bw = DISP_NO;
		}
		else
		{
			r_bw = r_voltage / 100;
		}
		r_sw = r_voltage % 100 / 10;
		r_gw = r_voltage % 10;
	}
	else if (r_set_state == LD_CHK_HW) //���´�����
	{
		if (f_hw_sensor_err) //���´�����������ʾE0
		{
			r_bw = DISP_NO;
			r_sw = DISP_E;
			r_gw = 0;
		}
		else if (r_hwsjwd < 38)
		{
			r_bw = DISP_FH;
			r_sw = (uchar)((38 - r_hwsjwd) / 10);
			r_gw = (uchar)((38 - r_hwsjwd) % 10);
		}
		else
		{
			r_bw = DISP_NO;
			r_sw = (uchar)((r_hwsjwd - 38) / 10);
			r_gw = (uchar)((r_hwsjwd - 38) % 10);
		}
	}
	else if (r_set_state == SET_LD_WDJZ) //ʪ��У׼
	{
		if (r_ldwdjzx >= 20)
		{
			r_bw = DISP_NO;
			r_sw = (r_ldwdjzx - 20)/10;
			r_gw = (r_ldwdjzx - 20)%10;
		}
		else
		{
			r_bw = DISP_FH;
			r_sw = (20 - r_ldwdjzx)/10;
			r_gw = (20 - r_ldwdjzx)%10;
		}
	}
	else if (r_set_state == SET_LD_BJYCSJ) //�䶳ѹ���ӳ�ʱ��
	{
		r_bw = DISP_NO;
		if (u8_ld_bjycsjx >= 10)
		{
			r_sw = 1;
		}
		else
		{
			r_sw = DISP_NO;
		}
		r_gw = u8_ld_bjycsjx % 10;
	}
	else if (r_set_state == SET_LDYJ) //��������ѹ����ͣ
	{
		r_bw = DISP_NO;
		r_sw = DISP_NO;
		if (t_halfsec & 0x01) //0b00000001)  @20181008 CFJ
		{
//			if ((flag_ajszyjkt & 0x01) == 0x01)
//				r_gw = 1;
//			else
//				r_gw = 0;
			r_gw = 1;

		}
		else
		{
			r_gw = DISP_NO;
		}
	}
	else if (r_set_state == SET_YEAR)
	{
		r_gw = DISP_NO;
		if (t_halfsec & 0x01) //0b00000001)  @20181008 CFJ
		{
			r_bw = u8_year / 10;
			r_sw = u8_year % 10;
		}
		else
		{
			r_bw = DISP_NO;
			r_sw = DISP_NO;
		}
	}
	else if (r_set_state == SET_MONTH)
	{
		r_gw = DISP_NO;
		if (t_halfsec & 0x01) //0b00000001)  @20181008 CFJ
		{
			r_bw = u8_month / 10;
			r_sw = u8_month % 10;
		}
		else
		{
			r_bw = DISP_NO;
			r_sw = DISP_NO;
		}
	}
	else if (r_set_state == SET_DAY)
	{
		r_gw = DISP_NO;
		if (t_halfsec & 0x01) //0b00000001)  @20181008 CFJ
		{
			r_bw = u8_day / 10;
			r_sw = u8_day % 10;
		}
		else
		{
			r_bw = DISP_NO;
			r_sw = DISP_NO;
		}
	}
	else if (r_set_state == SET_HOUR)
	{
		r_gw = DISP_NO;
		if (t_halfsec & 0x01) //0b00000001)  @20181008 CFJ
		{
			r_bw = u8_hour / 10;
			r_sw = u8_hour % 10;
		}
		else
		{
			r_bw = DISP_NO;
			r_sw = DISP_NO;
		}
	}
	else if (r_set_state == SET_MINUTE)
	{
		r_gw = DISP_NO;
		if (t_halfsec & 0x01) //0b00000001)  @20181008 CFJ
		{
			r_bw = u8_minute / 10;
			r_sw = u8_minute % 10;
		}
		else
		{
			r_bw = DISP_NO;
			r_sw = DISP_NO;
		}
	}
	else if (r_set_state == USB_Insert_Or_PullOut)
	{
		switch (u8_UsbState)
		{
		case 0:
			r_gw = DISP_b;
			r_sw = 5;
			r_bw = DISP_U;
			break;
		case 1:
			r_gw = DISP_Dot;
			r_sw = DISP_Dot;
			r_bw = DISP_Dot;
			break;
		case 2:
			r_gw = DISP_NO;
			r_sw = DISP_k;
			r_bw = DISP_o;
			break;
		default:
			r_gw = 0;
			r_sw = 0;
			r_bw = 0;
			break;
		}
	}
	else if (f_ld_DispErrs) //��ʾ����/����/�䶳SNR����
	{
		if (f_ld_high && f_ld_high_disp)
		{ //ʪ�ȸ� E9
			if ((unsigned char)(t_halfsec - t_ld_err_disp) >= 6) //3s
			{
				t_ld_err_disp = t_halfsec;
				f_ld_high_disp = 0;
			}
			r_bw = DISP_NO;
			r_sw = DISP_E;
			r_gw = 9;
		}
		else if (f_ld_low && f_ld_low_disp)
		{//ʪ�� ��E10
			if ((unsigned char)(t_halfsec - t_ld_err_disp) >= 6) //3s
			{
				t_ld_err_disp = t_halfsec;
				f_ld_low_disp = 0;
			}
			r_bw = DISP_E;
			r_sw = 1;
			r_gw = 0;
		}
		else
		{
			f_ld_DispErrs = 0;
			return;
		}
	}
//	else if (f_key_defrost && f_key_defrost_disp) //��˪ʱ,��ʾ3sDF,��ʾ5���¶�
//	{
//		if ((unsigned char)(t_halfsec - t_ld_err_disp) >= 6)
//		{
//			t_ld_err_disp = t_halfsec;
//			f_key_defrost_disp = 0;
//		}
//		r_bw = DISP_NO;
//		r_sw = DISP_D;
//		r_gw = DISP_F;
//	}
	else if (f_power_off && f_power_off_disp) //��
	{
		r_bw = DISP_NO;
		r_sw = DISP_NO;
		r_gw = DISP_NO;
	}
//	else if (f_ld_sensor_err)
//	{
//		if ((uchar)(t_halfsec - t_ld_err_disp) >= 10) //5s
//		{
//			f_key_defrost_disp = 1;
//			t_ld_err_disp = t_halfsec;
//		}
//		r_bw = DISP_NO;
//		r_sw = DISP_E;
//		r_gw = 6;
//	}
	else if (g_Sys_Erflag0_Comm) //@20181130 CFJ ����ͨѶ���ϱ�־
	{
		r_bw = DISP_NO;
		r_sw = DISP_E;
		r_gw = 4;
	}
	else
	{
		#ifdef Need_Ld
		if ((uchar)(t_halfsec - t_ld_err_disp) >= 10) //5s
		{
			f_key_defrost_disp = 1;
			t_ld_err_disp = t_halfsec;
		}
		
		if (r_ldxswd >= 200)
		{
			r_bw = DISP_NO;
			r_sw = (uchar)((r_ldxswd - 200) / 10);
			r_gw = (uchar)((r_ldxswd - 200) % 10);
		}
		else
		{
			r_bw = DISP_FH; //-
			r_sw = (uchar)((200 - r_ldxswd) / 10);
			r_gw = (uchar)((200 - r_ldxswd) % 10);
		}
		#else
			if(r_ldxswd>=100){//���Լ���ӵ�ʪ�ȣ�����ƫ��
				r_bw=1;
				r_sw=0;
				r_gw=0;
			}else{
				r_bw = DISP_NO;
				r_sw = (uchar)((r_ldxswd) / 10);
				r_gw = (uchar)((r_ldxswd) % 10);
			}
		
		#endif
	}

	r_led11 = table_led[r_bw];
	r_led12 = table_led[r_sw];
	r_led13 = table_led[r_gw];
}
/****************************************************/
/********************�����ʾ����********************/
/****************************************************/
void LcDisp(void) //��ÿ���������ʾ����ȷ������
{
	unsigned char r_bw = 0, r_sw = 0, r_gw = 0;

	flag_Now_Disp_LCWD = 0; //

	if (f_test_alarm || f_First_PowerOnFlag) //@20181120 CFJ    �״��ϵ�  ����  ���ˡ��������ԡ�����
	{
		r_led21 = 0xff;
		r_led22 = 0xff;
		r_led23 = 0xff;
		return;
	}
	if (!f_first_ad)
	{
		r_led21 = table_led[DISP_NO]; //�ʶ�ȫ��
		r_led22 = table_led[DISP_NO];
		r_led23 = table_led[DISP_NO];
		return;
	}
	if (r_set_state == SET_LC_L0) //  LO ��˸
	{
		if (r_flash_bit == BIT3)
		{
			r_bw = DISP_NO;
			r_sw = DISP_L;
			if (t_halfsec & 0x01) //0b00000001)  @20181008 CFJ
			{
				r_gw = 0;
			}
			else
			{
				r_gw = DISP_NO;
			}
		}
		else if (r_flash_bit == BIT2)
		{
			r_bw = DISP_NO;
			r_gw = 0;
			if (t_halfsec & 0x01) //0b00000001)  @20181008 CFJ
			{
				r_sw = DISP_L;
			}
			else
			{
				r_sw = DISP_NO;
			}
		}
	}
	else if (r_set_state == SET_LC_L1) // L1 ��˸
	{
		if (r_flash_bit == BIT3)
		{
			r_bw = DISP_NO;
			r_sw = DISP_L;
			if (t_halfsec & 0x01) //0b00000001)  @20181008 CFJ
			{
				r_gw = 1;
			}
			else
			{
				r_gw = DISP_NO;
			}
		}
	}
	else if (r_set_state == SET_LC_L2) // L2 ��˸
	{
		if (r_flash_bit == BIT3)
		{
			r_bw = DISP_NO;
			r_sw = DISP_L;
			if (t_halfsec & 0x01) //0b00000001)  @20181008 CFJ
			{
				r_gw = 2;
			}
			else
			{
				r_gw = DISP_NO;
			}
		}
	}
	else if (r_set_state == SET_LC_L3) // L3 ��˸
	{
		if (r_flash_bit == BIT3)
		{
			r_bw = DISP_NO;
			r_sw = DISP_L;
			if (t_halfsec & 0x01) //0b00000001)  @20181008 CFJ
			{
				r_gw = 3;
			}
			else
			{
				r_gw = DISP_NO;
			}
		}
	}
	else if (r_set_state == SET_LC_L4) // L4 ��˸
	{
		if (r_flash_bit == BIT3)
		{
			r_bw = DISP_NO;
			r_sw = DISP_L;
			if (t_halfsec & 0x01) //0b00000001)  @20181008 CFJ
			{
				r_gw = 4;
			}
			else
			{
				r_gw = DISP_NO;
			}
		}
	}
	else if (r_set_state == SET_LC_L5) // L5 ��˸
	{
		if (r_flash_bit == BIT3)
		{
			r_bw = DISP_NO;
			r_sw = DISP_L;
			if (t_halfsec & 0x01) //0b00000001)  @20181008 CFJ
			{
				r_gw = 5;
			}
			else
			{
				r_gw = DISP_NO;
			}
		}
	}
	else if (r_set_state == SET_LC_L6) // L6 ��˸
	{
		if (r_flash_bit == BIT3)
		{
			r_bw = DISP_NO;
			r_sw = DISP_L;
			if (t_halfsec & 0x01) //0b00000001)  @20181008 CFJ
			{
				r_gw = 6;
			}
			else
			{
				r_gw = DISP_NO;
			}
		}
	}
	else if (r_set_state == SET_LC_L7) // L7 ��˸
	{
		if (r_flash_bit == BIT3)
		{
			r_bw = DISP_NO;
			r_sw = DISP_L;
			if (t_halfsec & 0x01) //0b00000001)  @20181008 CFJ
			{
				r_gw = 7;
			}
			else
			{
				r_gw = DISP_NO;
			}
		}
	}
	else if (r_set_state == SET_LC_L8) // L8 ��˸
	{
		if (r_flash_bit == BIT3)
		{
			r_bw = DISP_NO;
			r_sw = DISP_L;
			if (t_halfsec & 0x01) //0b00000001)  @20181008 CFJ
			{
				r_gw = 8;
			}
			else
			{
				r_gw = DISP_NO;
			}
		}
	}
	else if (r_set_state == SET_LC || r_set_state == SET_LC_HIGH || r_set_state == SET_LC_LOW) // ������������¶ȡ����±���ֵ�����±���ֵ
	{
		if (r_flash_bit == BIT1)
		{
			if (f_zf == ZHENG_SIGN)
			{
				if (t_halfsec & 0x01) //0b00000001)  @20181008 CFJ
				{
					r_bw = DISP_ZH;
				}
				else
				{
					r_bw = DISP_NO;
				}
			}
			else
			{
				if (t_halfsec & 0x01) //if(t_halfsec&0b00000001)
				{
					r_bw = DISP_FH;
				}
				else
				{
					r_bw = DISP_NO;
				}
			}
			r_sw = r_swtj;
			r_gw = r_gwtj;
		}
		else if (r_flash_bit == BIT2)
		{
			if (f_zf == ZHENG_SIGN)
			{
				r_bw = DISP_NO;
			}
			else
			{
				r_bw = DISP_FH;
			}
			if (t_halfsec & 0x01) //if(t_halfsec&0b00000001)
			{
				r_sw = r_swtj;
			}
			else
			{
				r_sw = DISP_NO;
			}
			r_gw = r_gwtj;
		}
		else if (r_flash_bit == BIT3)
		{
			if (f_zf == ZHENG_SIGN)
			{
				r_bw = DISP_NO;
			}
			else
			{
				r_bw = DISP_FH;
			}
			r_sw = r_swtj;
			if (t_halfsec & 0x01) //if(t_halfsec&0b00000001)
			{
				r_gw = r_gwtj;
			}
			else
			{
				r_gw = DISP_NO;
			}
		}
	}
	else if (r_set_state == LC_CHK_VOL) //
	{
		if (r_voltage / 100 == 0)
		{
			r_bw = DISP_NO;
		}
		else
		{
			r_bw = r_voltage / 100;
		}
		r_sw = r_voltage % 100 / 10;
		r_gw = r_voltage % 10;
	}
	else if (r_set_state == LC_CHK_HW) //���´�����Ϣ  ����ʵ���¶�
	{
		if (f_hw_sensor_err)
		{
			r_bw = DISP_NO;
			r_sw = DISP_E;
			r_gw = 0;
		}
		else if (r_hwsjwd < 38)
		{
			r_bw = DISP_FH;
			r_sw = (uchar)((38 - r_hwsjwd) / 10);
			r_gw = (uchar)((38 - r_hwsjwd) % 10);
		}
		else
		{
			r_bw = DISP_NO;
			r_sw = (uchar)((r_hwsjwd - 38) / 10);
			r_gw = (uchar)((r_hwsjwd - 38) % 10);
		}
	}
	else if (r_set_state == LC_CHK_LCKZ) //E3 ��ؿ��ƴ�����ʵ���¶�
	{
		flag_Now_Disp_LCWD = 1;
		if (f_lc_sensor_err)
		{//�����ؿ��ƴ�����������
			r_bw = DISP_NO;
			r_sw = DISP_E;
			r_gw = 3;
		}
		else if (r_lcwd_float_c >= 38.0) //�¶�Ϊ��  �ֱ���Ϊ0.1
			{
				r_bw = (unsigned int)((r_lcwd_float_c - 38.0) * 10 + 0.5) / 100;		//+0.5��Ϊ��floatǿ��ת������ʱ����������
				r_sw = ((unsigned int)((r_lcwd_float_c - 38.0) * 10 + 0.5) % 100) / 10; //+0.5��Ϊ��floatǿ��ת������ʱ����������
				r_gw = (unsigned int)((r_lcwd_float_c - 38.0) * 10 + 0.5) % 10;
				flag_Disp_LCWD_D = 1;
				if (r_bw == 0)
				{
					r_bw = DISP_NO; //��λ��0����ʾ
				}
			}
			else if (r_lcwd_float_c <= 28.0) //�¶�<= -10��  //ֻ��ʾ��������
			{
				r_bw = DISP_FH; //-
				r_sw = ((unsigned int)(38 - r_lcwd_float_c) % 100) / 10;
				r_gw = (unsigned int)(38 - r_lcwd_float_c) % 10;
				flag_Disp_LCWD_D = 0;
			}
			else //   0  ~ -9.9��   ��ʾ1λС��
			{
				uchar temp_lc_x10 = (uchar)((38.0 - r_lcwd_float_c) * 10 + 0.5);
				r_bw = DISP_FH; //-
				if (temp_lc_x10 >= 100)
					temp_lc_x10 = 99;
				r_sw = temp_lc_x10 / 10; //+0.5��Ϊ��floatǿ��ת������ʱ����������
				r_gw = temp_lc_x10 % 10;
				flag_Disp_LCWD_D = 1;
			}
//			if (flag_Now_Disp_LCWD)
//			{
//				if (flag_Disp_LCWD_D)
//					r_led22 |= 0x20; //��ʾС����
//				else
//					r_led22 &= ~0x20; //�ر�С����
//			}
//			else
//				r_led22 &= ~0x20; //�ر�С����
	}
	else if (r_set_state == LC_CHK_LCHS) //��ؿ��ƴ�����ʵ���¶�
	{
		if (f_lcHs_sensor_err)//E2 �����ػ�˪������������
		{
			r_bw = DISP_NO;
			r_sw = DISP_E;
			r_gw = 2;
		}
		else if (r_lcHuaSHuangxswd_float < 510)
		{
			r_bw = DISP_FH;
			r_sw = ((int)(510 - r_lcHuaSHuangxswd_float)) / 100;
			r_gw = ((int)(510 - r_lcHuaSHuangxswd_float)-r_sw*100) / 10;
		}
		else
		{
			r_bw = DISP_NO;
			r_sw = ((int)(r_lcHuaSHuangxswd_float - 510)) / 100;
			r_gw = ((int)(r_lcHuaSHuangxswd_float - 510)-r_sw*100) / 10;
		}
	}
	else if (r_set_state == LC_CHK_LCLN) //��ؿ��ƴ�����ʵ���¶�
	{
		if (f_lcLn_sensor_err) //E6 �����ػ�˪������������
		{
			r_bw = DISP_NO;
			r_sw = DISP_E;
			r_gw = 6;
		}
		else if (r_lcLengNingxswd_float < 510)
		{
			r_bw = DISP_FH;
			r_sw = ((int)(510 - r_lcLengNingxswd_float)) / 100;
			r_gw = ((int)(510 - r_lcLengNingxswd_float)-r_sw*100) / 10;
		}
		else
		{
			r_bw = DISP_NO;
			r_sw = ((int)(r_lcLengNingxswd_float - 510)) / 100;
			r_gw = ((int)(r_lcLengNingxswd_float - 510)-r_sw*100)/ 10;
		}
	}
	else if (r_set_state == SET_LC_WDJZ)
	{
		flag_Now_Disp_LCWD = 1;
		flag_Disp_LCWD_D = 1;
		
		if (r_lcwdjzx >= 100)
		{
			r_bw = DISP_NO;
			r_sw = (r_lcwdjzx - 100)/10;
			r_gw = (r_lcwdjzx - 100)%10;
		}
		else
		{
			r_bw = DISP_FH;
			r_sw = (100 - r_lcwdjzx)/10;
			r_gw = (100 - r_lcwdjzx)%10;
		}
//		if (flag_Now_Disp_LCWD)
//			{
//				if (flag_Disp_LCWD_D)
//					r_led22 |= 0x20; //��ʾС����
//				else
//					r_led22 &= ~0x20; //�ر�С����
//			}
//			else
//				r_led22 &= ~0x20; //�ر�С����
	}
	else if (r_set_state == SET_LC_XSWDJZ)
	{
		flag_Now_Disp_LCWD = 1;
		flag_Disp_LCWD_D = 1;
		
		if (r_lcxswdjzx >= 100)
		{
			r_bw = DISP_NO;
			r_sw = (r_lcxswdjzx - 100)/10;
			r_gw = (r_lcxswdjzx - 100)%10;
		}
		else
		{
			r_bw = DISP_FH;
			r_sw = (100 - r_lcxswdjzx)/10;
			r_gw = (100 - r_lcxswdjzx)%10;
		}
//		if (flag_Now_Disp_LCWD)
//			{
//				if (flag_Disp_LCWD_D)
//					r_led22 |= 0x20; //��ʾС����
//				else
//					r_led22 &= ~0x20; //�ر�С����
//			}
//			else
//				r_led22 &= ~0x20; //�ر�С����
	}
	else if (r_set_state == SET_LC_BJYCSJ)
	{
		r_bw = DISP_NO;
		if (u8_lc_bjycsjx >= 10)
		{
			r_sw = 1;
		}
		else
		{
			r_sw = DISP_NO;
		}
		r_gw = u8_lc_bjycsjx % 10;
	}
	else if (r_set_state == SET_LCYJ_DELAY)
	{
		r_bw = DISP_NO;
		if (t_yj_delayx >= 10)
		{
			r_sw = 1;
		}
		else
		{
			r_sw = DISP_NO;
		}
		r_gw = t_yj_delayx % 10;
	}
	else if (r_set_state == SET_LCYJ)
	{
		r_bw = DISP_NO;
		r_sw = DISP_NO;
		if (t_halfsec & 0x01) //if(t_halfsec&0b00000001)
		{
//			if ((flag_ajszyjkt & 0x02) == 0x02)
//				r_gw = 1;
//			else
//				r_gw = 0;
			r_gw = 1;

		}
		else
		{
			r_gw = DISP_NO;
		}
	}
	else if (r_set_state == SET_YEAR)
	{
		r_gw = DISP_NO;
		r_bw = 1;
		r_sw = DISP_P;
	}
	else if (r_set_state == SET_MONTH)
	{
		r_gw = DISP_NO;
		r_bw = 2;
		r_sw = DISP_P;
	}
	else if (r_set_state == SET_DAY)
	{
		r_gw = DISP_NO;
		r_bw = 3;
		r_sw = DISP_P;
	}
	else if (r_set_state == SET_HOUR)
	{
		r_gw = DISP_NO;
		r_bw = 4;
		r_sw = DISP_P;
	}
	else if (r_set_state == SET_MINUTE)
	{
		r_gw = DISP_NO;
		r_bw = 5;
		r_sw = DISP_P;
	}
	else if (r_set_state == USB_Insert_Or_PullOut) //zyj  100621
	{
		switch (u8_UsbState)
		{
		case 0:
			r_gw = DISP_o;
			r_sw = DISP_N;
			r_bw = DISP_NO;
			break;
		case 1:
			r_gw = DISP_b;
			r_sw = 5;
			r_bw = DISP_U;
			break;
		case 2:
			r_gw = DISP_b;
			r_sw = 5;
			r_bw = DISP_U;
			break;
		default:
			break;
		}
	}
	else if (f_lc_DispErrs) //��ʾ�������
	{
		if (f_battery && f_BatteryErrDisp)
		{//��� E5
			if ((uchar)(t_halfsec - t_lc_err_disp) >= 6) //3s
			{
				t_lc_err_disp = t_halfsec;
				f_BatteryErrDisp = 0;
			}
			r_bw = DISP_NO;
			r_sw = DISP_E;
			r_gw = 5;
		}
		else if (f_lc_high && f_lc_high_disp)
		{//����¶ȸ� E9
			if ((unsigned char)(t_halfsec - t_lc_err_disp) >= 6) //3s
			{
				t_lc_err_disp = t_halfsec;
				f_lc_high_disp = 0;
			}
			r_bw = DISP_NO;
			r_sw = DISP_E;
			r_gw = 9;
		}
		else if (f_lc_low && f_lc_low_disp)
		{//����¶ȵ� E10
			if ((unsigned char)(t_halfsec - t_lc_err_disp) >= 6) //3s
			{
				t_lc_err_disp = t_halfsec;
				f_lc_low_disp = 0;
			}
			r_bw = DISP_E;
			r_sw = 1;
			r_gw = 0;
		}
		else if (f_hw_sensor_err && f_hw_sensor_err_disp)
		{ //���´��������� E0
			if ((unsigned char)(t_halfsec - t_lc_err_disp) >= 6) //3s
			{
				t_lc_err_disp = t_halfsec;
				f_hw_sensor_err_disp = 0;
			}
			r_bw = DISP_NO;
			r_sw = DISP_E;
			r_gw = 0;
		}
		else if (f_hw_high38 && f_hw_high_disp)
		{ // ���¸� E14
			if ((unsigned char)(t_halfsec - t_lc_err_disp) >= 6) //3s
			{
				t_lc_err_disp = t_halfsec;
				f_hw_high_disp = 0;
			}
			r_bw = DISP_E;
			r_sw = 1;
			r_gw = 4;
		}
		else if (f_lcXs_sensor_err && f_lcXs_err_disp)
		{//�����ʾ������ E1
			if ((unsigned char)(t_halfsec - t_lc_err_disp) >= 6) //3s
			{
				t_lc_err_disp = t_halfsec;
				f_lcXs_err_disp = 0;
			}
			r_bw = DISP_NO;
			r_sw = DISP_E;
			r_gw = 1;
		}
		else if (f_lcHs_sensor_err && f_lcHs_err_disp)
		{//��˪������ E2
			if ((unsigned char)(t_halfsec - t_lc_err_disp) >= 6) //3s
			{
				t_lc_err_disp = t_halfsec;
				f_lcHs_err_disp = 0;
			}
			r_bw = DISP_NO;
			r_sw = DISP_E;
			r_gw = 2;
		}
		else if (f_lc_sensor_err && f_lc_sensor_err_disp)
		{//��ؿ��ƴ����� E3
			if ((unsigned char)(t_halfsec - t_lc_err_disp) >= 6) //3s
			{
				t_lc_err_disp = t_halfsec;
				f_lc_sensor_err_disp = 0;
			}
			r_bw = DISP_NO;
			r_sw = DISP_E;
			r_gw = 3;
		}
		else if (f_lcLn_sensor_err && f_lcLn_err_disp)
		{//�������������� E6
			if ((unsigned char)(t_halfsec - t_lc_err_disp) >= 6) //3s
			{
				t_lc_err_disp = t_halfsec;
				f_lcLn_err_disp = 0;
			}
			r_bw = DISP_NO;
			r_sw = DISP_E;
			r_gw = 6;
		}
		else
		{
			f_lc_DispErrs = 0;
			return;
		}
	}
	else if (f_power_off && f_power_off_disp) //�����ܵͣ��ر���ʾ
	{
		if ((uchar)(t_halfsec - t_lc_err_disp) >= 60) //30s
		{
			t_lc_err_disp = t_halfsec;
			t_ld_err_disp = t_halfsec; //�� 2010 0506
			f_power_off_disp = 0;
		}
		r_bw = DISP_NO;
		r_sw = DISP_NO;
		r_gw = DISP_NO;
	}
	else if (f_lc_sensor_err) //��ؿ��ƴ���������  E3
	{
		if ((uchar)(t_halfsec - t_lc_err_disp) >= 10) //5s
		{
			t_lc_err_disp = t_halfsec;
			f_power_off_disp = 1;
		}
		r_bw = DISP_NO;
		r_sw = DISP_E;
		r_gw = 3;
	}

	else //��ʾʵ������¶�
	{
		flag_Now_Disp_LCWD = 1;
		if ((uchar)(t_halfsec - t_lc_err_disp) >= 10) //5s
		{
			t_lc_err_disp = t_halfsec;
			f_power_off_disp = 1; //�͵���ʱ�ر���ʾ    �ڵ͵���ʱ���ر�30s,��ʾ5s
		}

#if (LC_DISP_RESOLUTION_0D1 || LC_RESOLUTION_0D1) //�����ʾ�¶ȷֱ���Ϊ0.1��
		{

			if (r_lcxswd_float >= 38.0) //�¶�Ϊ��  �ֱ���Ϊ0.1
			{
				r_bw = (unsigned int)((r_lcxswd_float - 38.0) * 10 + 0.5) / 100;		//+0.5��Ϊ��floatǿ��ת������ʱ����������
				r_sw = ((unsigned int)((r_lcxswd_float - 38.0) * 10 + 0.5) % 100) / 10; //+0.5��Ϊ��floatǿ��ת������ʱ����������
				r_gw = (unsigned int)((r_lcxswd_float - 38.0) * 10 + 0.5) % 10;
				flag_Disp_LCWD_D = 1;
				if (r_bw == 0)
				{
					r_bw = DISP_NO; //��λ��0����ʾ
				}
			}
			else if (r_lcxswd_float <= 28.0) //�¶�<= -10��  //ֻ��ʾ��������
			{
				r_bw = DISP_FH; //-
				r_sw = ((unsigned int)(38 - r_lcxswd_float) % 100) / 10;
				r_gw = (unsigned int)(38 - r_lcxswd_float) % 10;
				flag_Disp_LCWD_D = 0;
			}
			else //   0  ~ -9.9��   ��ʾ1λС��
			{
				uchar temp_lc_x10 = (uchar)((38.0 - r_lcxswd_float) * 10 + 0.5);
				r_bw = DISP_FH; //-
				if (temp_lc_x10 >= 100)
					temp_lc_x10 = 99;
				r_sw = temp_lc_x10 / 10; //+0.5��Ϊ��floatǿ��ת������ʱ����������
				r_gw = temp_lc_x10 % 10;
				flag_Disp_LCWD_D = 1;
			}
		}
#else //�����ʾ�¶ȷֱ���Ϊ1��
		{
			if (r_lcxswd >= 38)
			{
				r_bw = DISP_NO;
				r_sw = (uchar)((r_lcxswd - 38) / 10);
				r_gw = (uchar)((r_lcxswd - 38) % 10);
			}
			else
			{
				r_bw = DISP_FH; //-
				r_sw = (uchar)((38 - r_lcxswd) / 10);
				r_gw = (uchar)((38 - r_lcxswd) % 10);
			}
		}
#endif
	}
	r_led21 = table_led[r_bw]; //0x44;//
	r_led22 = table_led[r_sw]; //0x44;//
	r_led23 = table_led[r_gw]; //0x44;//

	if (flag_Now_Disp_LCWD)
	{
		if (flag_Disp_LCWD_D)
			r_led22 |= 0x20; //��ʾС����
		else
			r_led22 &= ~0x20; //�ر�С����
	}
	else
		r_led22 &= ~0x20; //�ر�С����
}
/****************************************************/
/*******************alarm and lock LED***************/
/****************************************************/
void AlarmAndLockLed(void)
{
	//�����ƵĴ���
	if (f_lock || f_test_alarm)
	{
		r_led34 |= LOCK_LED_ON;
	}
	else
	{
		r_led34 &= LOCK_LED_OFF;
	}
	//�����ƵĴ���
	if (f_test_alarm || f_First_PowerOnFlag) //@20181120 CFJ
	{
		if ((uchar)(t_20ms - u8_test_alarm_time) >= 50) //50 1S  //25)      //0.5s @20181130 CFJ
		{
			u8_test_alarm_count++;
			u8_test_alarm_time = t_20ms;
			if (u8_test_alarm_count > 10) //12//����װ����7�Σ����Ը�Ϊ10 @20181130 CFJ                     //6s
			{
				u8_test_alarm_count = 0;
				f_test_alarm = OFF;
				f_First_PowerOnFlag = 0; //@20181120 CFJ
			}
		}
		r_led34 |= ALARM_LED_ON;
	}
	else if (f_battery)
	{
		r_led34 |= ALARM_LED_ON;
	}
	else if (f_hw_sensor_err || f_hw_high38 || f_ld_high || f_ld_low || f_power_off || f_lc_high || f_lc_low || f_lc_sensor_err || f_door_open||f_lcXs_sensor_err||f_lcHs_sensor_err||f_lcLn_sensor_err) ////////�ӿ��ű���wys11.03.19f_door_open
	{
		if (t_halfsec & 0x01) //if(t_halfsec&0b00000001) @20181008 CFJ
		{
			r_led34 |= ALARM_LED_ON;
		}
		else
		{
			r_led34 &= ALARM_LED_OFF;
		}
	}
	else
	{
		r_led34 &= ALARM_LED_OFF;
	}
}
/****************************************************/
/*******************��������*************************/
/****************************************************/
void BuzzAlarmControl(void);
void LongRangeAlarm(void);
/****************************************************/
void AlarmControl(void)
{
	BuzzAlarmControl();//���ݲ������Ʒ�������
	LongRangeAlarm();//Զ�̱���
}
/****************************************************/
void BuzzAlarmControl(void)
{
	if (f_test_alarm) //������ӷ�������
	{
		f_buzz_alarm = ON;
	}
	else if (f_stop_alarm) //����в�Ҫ������Ҫ��,ֹͣ����������
	{
		f_buzz_alarm = OFF;
		if (f_onemin)
		{
			t_stop_alarm++;
			if (t_stop_alarm >= 30)
			{
				f_stop_alarm = OFF;
			}
		}
	}
	else if (f_ld_high_buzz || f_ld_low_buzz || f_power_buzz  || f_lc_high_buzz || f_lc_low_buzz || f_lc_sensor_err||f_lcXs_sensor_err||f_lcHs_sensor_err||f_lcLn_sensor_err || f_dooropen_buzz) //||f_hw_sensor_err||f_battery)//20181101 TEST ����һ�����´��������Ϻ͵�ص�ѹ�ͱ��� CFJ /////wys11.03.19�ӿ��ű���
	{
		f_buzz_alarm = ON;
	}
	else
	{
		f_buzz_alarm = OFF;
	}
}
/********************************/
void LongRangeAlarm(void) //Զ�̱��������û��Զ�̱�����˵��
{
	if (f_ld_high_buzz || f_ld_low_buzz || f_lc_high_buzz || f_lc_low_buzz || f_power_buzz)
	{
		f_long_range_alarm = 1;
	}
	else
	{
		f_long_range_alarm = 0;
	}
}
/****************************************************/
/*******************����������ݴ���*****************/
/****************************************************/
void DealRecData(void)
{

}
/****************************************************/
/*****************Judge Errs*************************/
/****************************************************/
//void JudgeLdErr(void);
void JudgeLcErr(void);
void JudgeHwErr(void);
void JudgeHwHigh(void);
void JudgeLdHigh(void);
void JudgeLcLnHigh(void);
void JudgeLcHuaShuangLow(void);
void JudgeHumOver75(void);
void JudgeLcHigh(void);
void JudgeLdLow(void);
void JudgeLcLow(void);
void JudgeVoltageErr(void);
void JudgePowerOff(void);
void JudgeBattery(void);
void JudgeLcXsErr(void);//�����ʾ����������
void JudgeLcHsErr(void);//��ػ�˪����������
void JudgeLcLnErr(void);//�����������������
/*****************************************************/
void JudgeErrs(void)
{
	if (f_first_ad)
	{
	#ifdef Need_Ld
		//JudgeLdErr();
	#endif
		JudgeLcErr(); //��ؿ��ƴ���������
		JudgeHwErr();
		JudgeLcXsErr();//�����ʾ����������
		JudgeLcHsErr();//��ػ�˪����������
		JudgeLcLnErr();//�����������������
		JudgeHwHigh();
		
		JudgeLdHigh();
		JudgeLdLow();

		JudgeLcHigh();
		JudgeLcLow();
		JudgePowerOff();
		JudgeBattery();
		JudgeVoltageErr();
		JudgeLcLnHigh();//�ж������������Ƿ���
		JudgeLcHuaShuangLow();//�ж��Ƿ񵽴ﻯ˪����
		JudgeHumOver75();//�Ƿ�˪
	}
	// f_ld_sensor_err =1;
	// f_lc_sensor_err =1;
	// f_battery =      1;
	// f_hw_sensor_err =1;
	// f_hw_high38     =1;
	// f_ld_high =      1;
	// f_ld_low =       1;
	// f_lc_high =      1;
	// f_lc_low =       1;
	// f_power_off  =   1;
	// f_voltage_err =  1;
}
/**********************************/
//void JudgeLdErr(void) //�䶳����������
//{
//	if (r16_ldad > 1010 || r16_ldad < 15)
//	{
//		if (!f_ld_sensor_err) //��������䶳����������,����һ��,�Ժ��ٽ���
//		{
//			f_ld_sensor_err = 1;
//			f_stop_alarm = 0;
//		}
//	}
//	else if (f_ld_sensor_err)
//	{
//		f_ld_sensor_err = 0;
//		f_ld_first_disp = 0;
//	}
//}
/**********************************/
void JudgeLcErr(void) //��ؿ��ƴ���������
{
	if (r_lcad_12b > 250 * 16 || r_lcad_12b < 5 * 16)
	{
		if (!f_lc_sensor_err)
		{
			f_lc_sensor_err = 1;
			f_stop_alarm = 0;
		}
	}
	else if (f_lc_sensor_err)
	{
		f_lc_sensor_err = 0;
		f_lc_first_disp = 0;
	}
}
void JudgeLcXsErr(void)//�����ʾ����������
{
	if (r_lcXsad >=250 || r_lcXsad <=5)
	{
		if (!f_lcXs_sensor_err)
		{
			f_lcXs_sensor_err = 1;
			f_stop_alarm = 0;//���������ϱ���
		}
	}
	else if (f_lcXs_sensor_err)
	{
		f_lcXs_sensor_err = 0;

	}

}

void JudgeLcLnErr(void)//�����������������
{
	if (r16_lcLengNingad > 3817 || r16_lcLengNingad < 72)
	{
		if (!f_lcLn_sensor_err)
		{
			f_lcLn_sensor_err = 1;
			f_stop_alarm = 0;//���������ϱ���
		}
	}
	else if (f_lcLn_sensor_err)
	{
		f_lcLn_sensor_err = 0;

	}

}


void JudgeLcHsErr(void)//��ػ�˪����������
{
	if (r16_lcHuaShuangad > 3817 || r16_lcHuaShuangad < 72)
	{
		if (!f_lcHs_sensor_err)
		{
			f_lcHs_sensor_err = 1;
			f_stop_alarm = 0;//���������ϱ���
		}
	}
	else if (f_lcHs_sensor_err)
	{
		f_lcHs_sensor_err = 0;

	}

}

/**********************************/
void JudgeHwErr(void) //���´���������
{
	if (r_hwad > 240 || r_hwad < 5)
	{
		if (!f_hw_sensor_err)
		{
			f_hw_sensor_err = 1;
			f_stop_alarm = 0;//���������ϱ���
		}
	}
	else
	{
		f_hw_sensor_err = 0;
	}
}

/***********�ж�ʪ���ǲ��ǳ���75%�����������Ҫ���ó�ʪ����*/
void JudgeHumOver751(void)
{
	//���ﻹҪ���һ��ʪ�ȴ������ж��ǲ�������
 	if(f_lc_sensor_err!=1){//��ؿ��ƴ���������
	 	if((r_lcwd>=(2.5+38))&&(r_lcwd<=(7.5+38))){
			if(r_ldxswd>r_ldzt){//��ʾʪ��>75,��������ʾʪ�Ȼ�����ʵ��ʪ����Ҫ����//20200908          �ĳ����趨ֵ���ж�ʪ���Ƿ���ͣ 75
				if(r_lcwd>=38+4){//��ؿ��ƴ������¶�>=4�� 
					f_deHum_and_lcTemp=1;
					/********************************************
					ѹ�������������ͣ��ʱ�䲻�������ӣ������������Ӻ��������������ڷ��ͣ��
					ѹ���������������ڷ��ÿ���2���ӹ���1���ӣ�
					********************************************/
				}else{//С��4��
					f_deHum_and_lcTemp=2;
					/********************************************
					ѹ�������������ͣ��ʱ�䲻�������ӣ������������Ӻ��������������ڷ��ͣ��
					ѹ���������������ڷ��ÿ���2���ӹ���20���ӣ�
					********************************************/
				}
			
			}else if(r_ldxswd<=30){//����ʪ��<=30����ʪ
				f_deHum_and_lcTemp=0;
			}
	 	}else{
	 		f_deHum_and_lcTemp=0;//����ʪ
	 	}
 	}else{
 			f_deHum_and_lcTemp=0;//����ʪ
 	}
}

//�Ź������С��ͷҪ��ȡ������ʪ����,��˵���¶ȿ��Ʋ�ס�����ɸ���ʪ�й�ϵ
void JudgeHumOver75(void)
{
 	f_deHum_and_lcTemp=0;//����ʪ
}

/*************�жϻ��£������ݻ��������¼�********************/
void JudgeHwHigh(void) //���³�38
{
	if (f_hw_sensor_err)
	{
		f_hw_high38 = 0;
	}
	else if (r_hwsjwd > 76) //>38
	{
		if (!f_hw_high38)
		{
			f_hw_high38 = 1;
		}
	}
	else if (r_hwsjwd < 76)
	{
		f_hw_high38 = 0;
	}

	if (r_hwsjwd > (48+38) )//���³�48�ȣ�������±���
	{
		if (!Protect_HigHw_Temp)
		{
			Protect_HigHw_Temp = 1;
			f_lc_fan = OFF;//�ر�����������
			f_lc_compressor=OFF;//�ر����ѹ��
		}
	}else if(r_hwsjwd < (42+38) )
	{//����42���˳����±���
		Protect_HigHw_Temp=0;
	}
}
/***************************************************
�ж���ص������������¶ȣ��������¶ȶ���
****************************************************/
void JudgeLcLnHigh(void)
{
	if(f_lcLn_sensor_err==0){//����������û�й���
		//if(((r_lclnsjwd&0x80)>>7)!=1)
		if(r_lclnsjwd>0){
			if (r_lclnsjwd > LengNing_Protect_temp)//�����������ȳ���55�ȣ�������±���
			{
				if (!Protect_HigLn_Temp)
				{
					Protect_HigLn_Temp = 1;
					f_lc_fan = OFF;//�ر�����������
					f_lc_compressor=OFF;//�ر����ѹ��
				}
			}else if(r_lclnsjwd < LengNing_NotProtect_temp )
			{//����50���˳����±���
				Protect_HigLn_Temp=0;
			}
		}else{//�¶ȵ���0�ȣ�������ѹ��ͣ��
			Protect_HigLn_Temp=0;
		}
	}
}


/*****************************************************
�жϻ�˪�������¶ȣ��Ƿ񵽴ﻯ˪����
ѹ�����ۼ����г�����6Сʱ���ң���˪�������¶�<0.5��
*****************************************************/
void JudgeLcHuaShuangLow(void)
{
	if(f_lcCompSixHour){//ѹ�����Ѿ��ۼ�����6Сʱ
		if (r_lcHuaSHuangxswd_float <= 515)//��˪�������¶�ֵС��0.5�Ƚ��뻯˪����
		{
			if(f_lc_compressor == OFF){//ѹ������
				LcHuaShuang_Flag=1;
				f_lcCompSixHour=0;//����ѹ���ۼ�ͳ��ʱ��
			}

		}else
		{//�˳���˪������ѹ�������߼������жϰ�

		}
	}else{//�˳���˪����
		
	}
}
/**************************************************/
void JudgeLdHigh(void)//�䶳�¶��жϱ���
{
////#ifdef Need_Ld
//	//�䶳��������������ʪ������
//	if (!ldyj)
//	{
//		f_ld_high = 0;
//		f_ld_high_buzz = 0;
//		t_ld_high_buzz = t_tens;
//	}
//	//���±���,�����ϵ羭�ӳ�ʱ���FMQ����,�˺�15����FMQ����
//	else if (r_ldxswd > (r_ldzt + r_ld_high_alarm)) //��ʾ���趨�¶ȶ���ƫ200
//	{
//		f_ld_high = 1;
//		if (f_Powerupdelayld) //�����䶳�ϵ��ӳ�ʱ��+15min��FMQ����
//		{
//			if ((unsigned char)(t_tens - t_ld_high_buzz) >= 90) //15min
//			{
//				if (!f_ld_high_buzz)
//				{
//					f_ld_high_buzz = 1;
//					f_stop_alarm = 0;
//				}
//			}
//		}
//		else
//		{
//			t_ld_high_buzz = t_tens;
//		}
//	}
//	//���±�������
//	else if (r_ldxswd < (r_ldzt + r_ld_high_alarm))
//	{
//		f_ld_high = 0;
//		f_ld_high_buzz = 0;
//		t_ld_high_buzz = t_tens;
//	}
////#endif
}
/******************************/
void JudgeLcHigh(void)
{
	//��ش���������
	if (f_lc_sensor_err || (!lcyj))
	{
		f_lc_high = 0;
		f_lc_high_buzz = 0;
		t_lc_high_buzz = t_tens;
	}
#if (LC_DISP_RESOLUTION_0D1 || LC_RESOLUTION_0D1) //��ʾ�ֱ���0.1��
	else if (r_lcxswd_float > (r_lczt_float + r_lc_high_alarm))
#else
	else if (r_lcxswd > (r_lczt + r_lc_high_alarm))
#endif
	{
		f_lc_high = 1;
		if (f_Powerupdelaylc) ////if(f_Powerupdelaylc&&!f_door_switch) 11.02.23wys����״̬����15min����
		{
			if ((unsigned char)(t_tens - t_lc_high_buzz) >= 90) //15min
			{
				if (!f_lc_high_buzz)
				{
					f_lc_high_buzz = 1;
					f_stop_alarm = 0;
				}
			}
		}
		else
		{
			t_lc_high_buzz = t_tens;
		}
	}
#if (LC_DISP_RESOLUTION_0D1 || LC_RESOLUTION_0D1) //��ʾ�ֱ���0.1��
	else if (r_lcxswd_float < (r_lczt_float + r_lc_high_alarm))
#else
	else if (r_lcxswd < (r_lczt + r_lc_high_alarm))
#endif
	{
		f_lc_high = 0;
		f_lc_high_buzz = 0;
		t_lc_high_buzz = t_tens;
	}
}
/******************************/
void JudgeLdLow(void)//�䶳��ʾ�¶ȵ��±����жϣ�����ʪ�ȴ�������
{
////#ifdef Need_Ld
//	if (!ldyj)
//	{
//		f_ld_low = 0;
//		f_ld_low_buzz = 0;
//		t_ld_low_buzz = t_tens;
//	}
//	else if (r_ldxswd < (r_ldzt - r_ld_low_alarm))
//	{
//		f_ld_low = 1;
//		if (f_Powerupdelayld)
//		{
//			if ((unsigned char)(t_tens - t_ld_low_buzz) >= 90) //15min
//			{
//				if (!f_ld_low_buzz)
//				{
//					f_ld_low_buzz = 1;
//					f_stop_alarm = 0;
//				}
//			}
//		}
//		else
//		{
//			t_ld_low_buzz = t_tens;
//		}
//	}
//	else if (r_ldxswd > (r_ldzt - r_ld_low_alarm))
//	{
//		f_ld_low = 0;
//		f_ld_low_buzz = 0;
//		t_ld_low_buzz = t_tens;
//	}
////#endif
}
/******************************/
void JudgeLcLow(void)
{

	if (f_lc_sensor_err || (!lcyj))
	{
		f_lc_low = 0;
		f_lc_low_buzz = 0;
		t_lc_low_buzz = t_tens;
	}
#if (LC_DISP_RESOLUTION_0D1 || LC_RESOLUTION_0D1) //��ʾ�ֱ���0.1��
	else if ((r_lcxswd_float < (r_lczt_float - r_lc_low_alarm)) || (r_lcxswd_float < 38))
#else
	else if ((r_lcxswd < (r_lczt - r_lc_low_alarm)) || (r_lcxswd < 38))
#endif
	{
		f_lc_low = 1;
		if (f_Powerupdelaylc)
		{
			if ((unsigned char)(t_tens - t_lc_low_buzz) >= 90) //15min
			{
				if (!f_lc_low_buzz)
				{
					f_lc_low_buzz = 1;
					f_stop_alarm = 0;
				}
			}
		}
		else
		{
			t_lc_low_buzz = t_tens;
		}
	}
#if (LC_DISP_RESOLUTION_0D1 || LC_RESOLUTION_0D1) //��ʾ�ֱ���0.1��
	else if (r_lcxswd_float > (r_lczt_float - r_lc_low_alarm))
#else
	else if (r_lcxswd > (r_lczt - r_lc_low_alarm))
#endif
	{
		f_lc_low = 0;
		f_lc_low_buzz = 0;
		t_lc_low_buzz = t_tens;
	}
	//����Ƿ������±���
	if (f_lc_sensor_err || (!lcyj))
	{
		f_lc_low = 0;
		f_lc_low_buzz = 0;
		t_lc_low_Protect = t_tens;
	}else if(r_lcxswd_float < (2.5+38)){//��ʾ�¶�С��2.5�� ����ʾ������NTC2�ӿ�������
		if ((unsigned char)(t_tens - t_lc_low_Protect) >= 6) //1min
		{
			Protect_Low_Temp=1;//���±���
			f_lc_fan = OFF;//�ر�����������
			f_lc_compressor=OFF;//�ر����ѹ��
			f_nd_fan=ON;
		}
		
	}else{//����ʲôʱ��򿪣�����������ж�
		Protect_Low_Temp=0;
		t_lc_low_Protect=t_tens;
	}
}
/*********************************************************/
void JudgeVoltageErr(void)
{
	if (f_power_off)
	{
		f_voltage_err = 0;
		f_voltage_buzz = 0;
		t_voltage_buzz = t_halfsec;
	}
	//220V
	else if (f_voltage_slect == V220)
	{
		if (r_voltage < 196 || r_voltage > 238)
		{
			f_voltage_err = 1;
			if ((unsigned char)(t_halfsec - t_voltage_buzz) >= 120) //1min
			{
				if (!f_voltage_buzz)
				{
					f_voltage_buzz = 1;
					f_stop_alarm = 0;
				}
			}
		}
		else if (r_voltage > 196 && r_voltage < 238)
		{
			f_voltage_err = 0;
			f_voltage_buzz = 0;
			t_voltage_buzz = t_halfsec;
		}
	}
	//110V
	else
	{
		if (r_voltage < 99 || r_voltage > 122)
		{
			f_voltage_err = 1;
			if ((unsigned char)(t_halfsec - t_voltage_buzz) >= 120) //1min/////////////�������30s
			{
				if (!f_voltage_buzz)
				{
					f_voltage_buzz = 1;
					f_stop_alarm = 0;
				}
			}
		}
		else if (r_voltage > 99 && r_voltage < 122)
		{
			f_voltage_err = 0;
			f_voltage_buzz = 0;
			t_voltage_buzz = t_halfsec;
		}
	}
}
/*********************************/
void JudgePowerOff(void)
{

	if (r_voltage < 70)
	{
		//f_power_off = 1;
		if ((uchar)(t_halfsec - t_power_Off) >= 6) //�͵�ѹ����3s���򱨶ϵ� @20181226 CFJ
		{
			f_power_off = 1;
		}

		if ((uchar)(t_halfsec - t_power_buzz) >= 60) //30s
		{
			if (!f_power_buzz)
			{
				f_power_buzz = 1;
				f_stop_alarm = 0;
			}
		}
	}
	else
	{
		f_power_off = 0;
		f_power_buzz = 0;
		t_power_buzz = t_halfsec;
		t_power_Off = t_halfsec; //@20181226 CFJ �����ʱ��
	}
}
/*********************************/
void JudgeBattery(void)
{
	/* //@20181130 CFJ
		if(r_battery_ad<127)//10.43V
		{
			f_battery = 1;
		}
		else if(r_battery_ad>129)
		{
			f_battery = 0;
		}
		*/
}

/***********************************************************
*Name         :  DelayControl
*Function     :  Delay some time
*Input Value  :  Delay Cnt
*Output Value :  None
*Introduction :
***********************************************************/
void DelayControl(uchar R8_DelayCnt)
{
	for (; R8_DelayCnt > 0; R8_DelayCnt--)
	{
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
		nop();
	}
}

/****************************************************/
void DataToLed(void)
{
	if ((!f_first_ad) && SelfCheckFlag == 0) //if(!f_first_ad) //@20190221 CFJ
		return;
	if ((unsigned char)(t_onems - t_data_to_led) >= 125) //125ms
	{
		t_data_to_led = t_onems;
		LedBegin();
		LedWriteByte(0x00); //write,bank0,address0,address auto add

		LedWriteByte(r_led12); //�䶳�����  ��2λ
		LedWriteByte(r_led13); //�䶳�����  ��3λ
		LedWriteByte(r_led34); //����������ָʾ��

		LedWriteByte(r_led21); //��������  ��1λ ���������õ�����
		LedWriteByte(r_led22); //��������  ��2λ
		LedWriteByte(r_led23); //��������  ��3λ

		LedStop();
		LedBegin();
		LedWriteByte(0x28);	//(0b00101000);                       //write,fixed address,bank1,address 0  @20181008 CFJ
		LedWriteByte(r_led11); //�䶳�¶ȷ���

		LedStop();
		LedBegin();
		LedWriteByte(0x10); //(0b00010000);                       //write,bank2,address auto add,address0 @20181008 CFJ
		LedWriteByte(0xFD); //(0b11111101);     //@20181008 CFJ

		LedStop();
		LedBegin();
		LedWriteByte(0x0d);
		LedStop();
	}
}

/****************************************************/
void LedWriteByte(unsigned char led_data)
{
	unsigned char w_clk;
	for (w_clk = 8; w_clk != 0; w_clk--)
	{
		if (led_data & 0x01)
			LED_DIO_SET_HIGH;
		else
			LED_DIO_SET_LOW;
		led_data = led_data >> 1;
		LedSendScl(); //ack signal
	}
}
/****************************************************/
void LedBegin(void)
{
	DelayControl(4);
	LED_STB_SET_LOW;
	DelayControl(4);
}
/****************************************************/
void LedStop(void)
{
	DelayControl(4);
	LED_STB_SET_HIGH;
	DelayControl(4);
}

/****************************************************/
void LedSendScl(void)
{
	DelayControl(4);
	LED_CLK_SET_LOW;
	DelayControl(4);
	LED_CLK_SET_HIGH;
	DelayControl(4);
}

/****************************************************/
/*******************KEY******************************/
/****************************************************/
void ReadKey(void);
void KeyAction(void);
void KeyPutUp(void);
void KeyPutDown(void);
void KeyPut5s(void);
void AutoLock(void);
/****************************************************/
void KeyPress(void)
{
	if (f_first_ad)//�õ���lc ld�¶���ʹ�����в���
	{
		ReadKey();   //������ֵ
		KeyAction(); //��������
		AutoLock();//10S���Զ��洢����ֵ
	}
}
//^^^^^^^^^^^^^^^^^^^^^^^^^
void ReadKey(void)
{
	r_key = 0;
	if (KEY_SW1)//����
	{
		r_key = r_key | 0x01; //0b00000001; @20181008 CFJ
	}
	if (KEY_SW2)//λ���л�
	{
		r_key = r_key | 0x02; //0b00000010; @20181008 CFJ
	}
	if (KEY_SW3)//�¶ȵ���
	{
		r_key = r_key | 0x04; //0b00000100; @20181008 CFJ
	}
	if (KEY_SW4)//�����л�
	{
		r_key = r_key | 0x08; //0b00001000; @20181008 CFJ
	}
	if (KEY_SW5)//��������
	{
		r_key = r_key | 0x10; //0b00010000; @20181008 CFJ
	}
	if (KEY_SW6)//����ȡ��
	{
		r_key = r_key | 0x20; //0b00100000; @20181008 CFJ
	}
	if (KEY_SW7)//��˪
	{
		r_key = r_key | 0x40; //0b01000000; @20181008 CFJ
	}
}

//^^^^^^^^^^^^^^^^^^^^^^^^^
void KeyAction(void)
{
	if (r_key == 0) //û�а�������
	{
		r_keyz = 0;
		KeyPutUp(); //̧����Ч�İ�������
		r_sfkeyz = 0;//�洢������ֵ���ڱ�ĵط�ʹ�ã���keyputdown�����и�ֵ��Ҳ���ǰ��¸�ֵ��̧����ж�ֵ��ƽʱ����
		f_key_kp = 0;
		f_key_km = 0;
	}
	else //�а�������
	{
		if (!f_key_kp)//��f_key_kpû�а������µ��а�������ʱ��Ч
		{
			if (r_key != r_keyz)
			{//���°�����
				f_key_km = 0;
				r_keyz = r_key;
				t_key_dly = t_onems;
				t_key3s = t_halfsec;
			}
			else
			{//���º󣬵ڶ����жϣ���Чֵ>80ms
				if (!f_key_km)
				{
					if ((unsigned char)(t_onems - t_key_dly) >= 80)
					{
						f_key_km = 1;
						KeyPutDown(); //������Ч�İ�������,��r_sfkeyz��ֵ���Ա��ڱ�ĵط�ʹ��
					}
				}
				else
				{//����ʱ��̫���˳�����5S
					if ((unsigned char)(t_halfsec - t_key3s) >= 10)
					{
						KeyPut5s(); //����5����Ч�İ�������
					}
					/*if((unsigned char)(t_halfsec-t_key3s)>=6)
            {
                KeyPut3s();  //����3����Ч�İ�������
            }*/
				}
			}
		}
	}
}
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
void KeyPutUp(void) //����̧����Ч
{
	if (r_sfkeyz == KEY_UNLOCK) //λ���л�(����)
	{
		if (!f_lock) //���������
		{
			if (r_set_state == SET_LD || r_set_state == SET_LC)
			{
				r_flash_bit++;
				if (r_flash_bit > 3)
				{
					r_flash_bit = 2;
				}
				goto NotSaveKey2;
			}
			else if (r_set_state == SET_LD_HIGH || r_set_state == SET_LD_LOW ||
					 r_set_state == SET_LC_HIGH || r_set_state == SET_LC_LOW)
			{//��������ܵ���һλ��������״̬
				r_flash_bit++;
				if (r_flash_bit > 3)
				{
					r_flash_bit = 1;
				}
				goto NotSaveKey2;
			}
		}
	}
	else if (r_sfkeyz == KEY_SET) //�趨��
	{//���浱ǰ�趨ֵ
		if (!f_lock)
		{//����  
		
			if (r_set_state == SET_NC_LD) //�䶳��ǰ����
			{
				r_set_state = SET_LC_L0; //LD������ʾL0����
				r_flash_bit = BIT3;		 //��3λ��
				goto NotSaveKey2;
			}
			if (r_set_state == SET_NC_LC) //��ص�ǰ����״̬
			{
				r_set_state = SET_LC_L0; //Lc��ʾL0����
				r_flash_bit = BIT3;
				goto NotSaveKey2;
			}
			else if (r_set_state == SET_LD_L0) //�䶳L0-->�趨�¶�
			{
				r_set_state = SET_LD;
				r_flash_bit = BIT2;
//				f_zf = FU_SIGN;
				r_bwtj = (uchar)((r_ldzt) / 100);
				r_swtj = (uchar)((r_ldzt) / 10);
				r_gwtj = (uchar)((r_ldzt) % 10);
				goto NotSaveKey2;
			}
			else if (r_set_state == SET_LD_L1) //L1-->��ʪ�����趨ֵ
			{
				r_set_state = SET_LD_HIGH;
				r_flash_bit = BIT2;//�����м��Ǹ���ֵ
				#ifdef Need_ld
					if ((r_ldzt + r_ld_high_alarm) >= 200)
					{
						f_zf = ZHENG_SIGN;
						r_swtj = (uchar)((r_ldzt + r_ld_high_alarm - 200) / 10);
						r_gwtj = (uchar)((r_ldzt + r_ld_high_alarm - 200) % 10);
					}
					else
					{
						f_zf = FU_SIGN;
						r_swtj = (uchar)((200 - (r_ldzt + r_ld_high_alarm)) / 10);
						r_gwtj = (uchar)((200 - (r_ldzt + r_ld_high_alarm)) % 10);
					}
				#else
					f_zf = ZHENG_SIGN;
					r_bwtj = (uchar)((r_ldzt + r_ld_high_alarm ) / 100);
					r_swtj = (uchar)((r_ldzt + r_ld_high_alarm-100*r_bwtj ) / 10);
					r_gwtj = (uchar)((r_ldzt + r_ld_high_alarm ) % 10);
					//��Χ������0-100
					if ((r_ldzt + r_ld_high_alarm) > 100)
					{
						r_bwtj=1;
						r_swtj=0;
						r_gwtj=0;
						
					}
//					else if((r_ldzt + r_ld_high_alarm)<0)
//					{
//						r_bwtj=0;
//						r_swtj=0;
//						r_gwtj=0;
//					}
				#endif
				goto NotSaveKey2;
			}
			else if (r_set_state == SET_LD_L2) //L2-->���±����趨ֵ
			{
				r_set_state = SET_LD_LOW;
				r_flash_bit = BIT2;
				#ifdef Need_ld
					if ((r_ldzt - r_ld_low_alarm) >= 200)
					{
						f_zf = ZHENG_SIGN;
						r_swtj = (uchar)((r_ldzt - r_ld_low_alarm - 200) / 10);
						r_gwtj = (uchar)((r_ldzt - r_ld_low_alarm - 200) % 10);
					}
					else
					{
						f_zf = FU_SIGN;
						r_swtj = (uchar)((200 - (r_ldzt - r_ld_low_alarm)) / 10);
						r_gwtj = (uchar)((200 - (r_ldzt - r_ld_low_alarm)) % 10);
					}
				#else
					f_zf = ZHENG_SIGN;
					r_bwtj = (uchar)((r_ldzt - r_ld_low_alarm ) / 100);
					r_swtj = (uchar)((r_ldzt - r_ld_low_alarm-100*r_bwtj ) / 10);
					r_gwtj = (uchar)((r_ldzt - r_ld_low_alarm ) % 10);
				#endif
				goto NotSaveKey2;
			}
			else if (r_set_state == SET_LD_L3) //L3-->��ѹ��ѯ
			{
				r_set_state = LD_CHK_VOL;
				r_flash_bit = BIT3;
				goto NotSaveKey2;
			}
			else if (r_set_state == SET_LD_L4) //L4-->���²�ѯ
			{
				r_set_state = LD_CHK_HW;
				r_flash_bit = BIT3;
				goto NotSaveKey2;
			}
			else if (r_set_state == SET_LD_L5) //LDѹ����ͣ������
			{
				r_set_state = SET_LDYJ;
				r_flash_bit = BIT3;
				goto NotSaveKey2;
			}
			else if (r_set_state == LD_CHK_VOL || r_set_state == LD_CHK_HW)
			{
				r_set_state = SET_NC_LD;
				goto NotSaveKey2;
			}

			else if (r_set_state == SET_LC_L3)
			{
				r_set_state = LC_CHK_VOL;
				r_flash_bit = BIT3;
				goto NotSaveKey2;
			}
			else if (r_set_state == SET_LC_L4)
			{
				r_set_state = LC_CHK_HW;
				r_flash_bit = BIT3;
				goto NotSaveKey2;
			}
			else if (r_set_state == SET_LC_L5)
			{
				r_set_state = SET_LCYJ;
				r_flash_bit = BIT3;
				goto NotSaveKey2;
			}
			else if (r_set_state == SET_LC_L6)
			{
				r_set_state = LC_CHK_LCKZ;//��ؿ��ƴ������¶���ʾ
				r_flash_bit = BIT3;
				goto NotSaveKey2;
			}
			else if (r_set_state == SET_LC_L7)
			{
				r_set_state = LC_CHK_LCHS;//��ػ�˪�������¶���ʾ
				r_flash_bit = BIT3;
				goto NotSaveKey2;
			}
			else if (r_set_state == SET_LC_L8)
			{
				r_set_state = LC_CHK_LCLN;//��ػ�˪�������¶���ʾ
				r_flash_bit = BIT3;
				goto NotSaveKey2;
			}
			else if (r_set_state == LC_CHK_VOL || r_set_state == LC_CHK_HW||r_set_state == LC_CHK_LCKZ||r_set_state == LC_CHK_LCHS||r_set_state == LC_CHK_LCLN)
			{
				r_set_state = SET_NC_LC;
				goto NotSaveKey2;
			}
			else if (r_set_state == SET_LD) //ʪ���趨ֵ
			{
				if ((r_swtj * 10 + r_gwtj) > 0 && (r_swtj * 10 + r_gwtj) <= 100) //ʪ���趨ֵ��1-99֮��
				{
					r_ldzt = (r_swtj * 10 + r_gwtj);
					if((r_ldzt-r_ld_low_alarm)<1){r_ld_low_alarm=r_ldzt;}
					if((r_ldzt+r_ld_high_alarm)>100){r_ld_low_alarm=100-r_ldzt;}
					r_set_state = SET_NC_LD;
					goto SaveKey2;
				}
				else
				{
					goto SetErr;
				}
			}
			else if (r_set_state == SET_LD_HIGH) //�䶳���±�������
			{
				#ifdef Need_Ld
					if (f_zf == ZHENG_SIGN) //���±����������50
					{
						if ((r_swtj * 10 + r_gwtj) > 50)
						{
							goto SetErr;
						}
						else
						{
							r_ld_high_alarm = r_swtj * 10 + r_gwtj + 200 - r_ldzt;
							r_set_state = SET_NC_LD;
							goto SaveKey2;
						}
					}
					else //���±���������С>�趨�¶�+1
					{
						if ((200 - (r_swtj * 10 + r_gwtj)) < (r_ldzt + 1))
						{
							goto SetErr;
						}
						else
						{
							r_ld_high_alarm = 200 - (r_swtj * 10 + r_gwtj) - r_ldzt;
							r_set_state = SET_NC_LD;
							goto SaveKey2;
						}
					}
				#else
					if (f_zf == ZHENG_SIGN) //ʪ�ȱ����������100
					{
						if ((r_bwtj*100+r_swtj * 10 + r_gwtj) > 100)
						{
							goto SetErr;
						}
						else
						{
							if((r_bwtj*100+r_swtj * 10 + r_gwtj) < (r_ldzt+1))
							{//���±���ֵҪ���ڵ����趨ֵ+1
								goto SetErr;
							}
							r_ld_high_alarm = (r_bwtj*100+r_swtj * 10 + r_gwtj) - r_ldzt;//�õ���ֵ
							if(r_ld_high_alarm<1){r_ld_high_alarm=1;}
							r_set_state = SET_NC_LD;
							goto SaveKey2;
						}
					}
					else //���±���������С>�趨�¶�+1
					{
						goto SetErr;
					}
				#endif
			}
			else if (r_set_state == SET_LD_LOW) //LD�����趨
			{
				#ifdef Need_Ld
					if (f_zf == ZHENG_SIGN) //����
					{
						goto SetErr;
					}
					else //����
					{
						if ((200 - (r_swtj * 10 + r_gwtj)) > (r_ldzt - 1)) //>SET-1
						{
							goto SetErr;
						}
						else if ((200 - (r_swtj * 10 + r_gwtj)) < 140) //<-60
						{
							goto SetErr;
						}
						else
						{
							r_ld_low_alarm = r_ldzt - (200 - (r_swtj * 10 + r_gwtj));
							r_set_state = SET_NC_LD;
							goto SaveKey2;
						}
					}
				#else
					if (f_zf == ZHENG_SIGN) //����
					{
						if (((r_swtj * 10 + r_gwtj)) > (r_ldzt - 1)) //>SET-1
						{
							goto SetErr;
						}
//						else if (((r_swtj * 10 + r_gwtj)) < 0) //<-60
//						{
//							goto SetErr;
//						}
						else
						{
							r_ld_low_alarm = r_ldzt - ((r_swtj * 10 + r_gwtj));//�õ���ֵ
							if(r_ld_low_alarm<1){r_ld_low_alarm=1;}
//							if(r_ld_low_alarm<1)
//							{
//								goto SetErr;
//							}
							r_set_state = SET_NC_LD;
							goto SaveKey2;
						}
					}
					else //����
					{
						goto SetErr;
					}
				#endif
				
			}
			else if (r_set_state == SET_LC_L0) //L0-->�趨�¶�
			{
				r_set_state = SET_LC;
				r_flash_bit = BIT2;
				f_zf = ZHENG_SIGN;
				r_swtj = (uchar)((r_lczt - 38) / 10);
				r_gwtj = (uchar)((r_lczt - 38) % 10);
				goto NotSaveKey2;
			}
			else if (r_set_state == SET_LC_L1) //L1-->���±����趨ֵ
			{
				r_set_state = SET_LC_HIGH;
				r_flash_bit = BIT2;
				if ((r_lczt + r_lc_high_alarm) >= 38)
				{
					f_zf = ZHENG_SIGN;
					r_swtj = (uchar)((r_lczt + r_lc_high_alarm - 38) / 10);
					r_gwtj = (uchar)((r_lczt + r_lc_high_alarm - 38) % 10);
				}
				else
				{
					f_zf = FU_SIGN;
					r_swtj = (uchar)((38 - (r_lczt + r_lc_high_alarm)) / 10);
					r_gwtj = (uchar)((38 - (r_lczt + r_lc_high_alarm)) % 10);
				}
				goto NotSaveKey2;
			}
			else if (r_set_state == SET_LC_L2) //L2-->���±����趨
			{
				r_set_state = SET_LC_LOW;
				r_flash_bit = BIT2;
				if ((r_lczt - r_lc_low_alarm) >= 38) //�����¶� >= ���±����²�
				{
					if (r_lczt >= (LC_SET_TEMP_MIN + 38)) //�����¶�>=2��
					{
						f_zf = ZHENG_SIGN;
						r_swtj = (uchar)((r_lczt - r_lc_low_alarm - 38) / 10);
						r_gwtj = (uchar)((r_lczt - r_lc_low_alarm - 38) % 10);
					}
				}
				else //�����¶�< ���±����²�  �����±����¶�ֵΪ��
				{
					f_zf = FU_SIGN;
					r_swtj = (uchar)(-(r_lczt - r_lc_low_alarm - 38) / 10);
					r_gwtj = (uchar)(-(r_lczt - r_lc_low_alarm - 38) % 10);
				}
				goto NotSaveKey2;
			}
			else if (r_set_state == SET_LC) //����趨
			{
				//if(((r_swtj*10+r_gwtj)<=16)&&((r_swtj*10+r_gwtj)>=5))//20171106  if((r_swtj*10+r_gwtj)<=16)//����趨�¶�5~16֮��
				if (((r_swtj * 10 + r_gwtj) <= LC_SET_TEMP_MAX) && ((r_swtj * 10 + r_gwtj) >= LC_SET_TEMP_MIN)) //  2019-9-11 ���˷�
				{
					r_lczt = 38 + (r_swtj * 10 + r_gwtj);
					r_lczt_float = r_lczt;

					if ((r_lczt < (LC_SET_TEMP_MIN + 38)) || ((r_lczt - 38 - r_lc_low_alarm) < 0)) //���±���ֵΪ��
					{
						r_lc_low_alarm = r_lczt - 38; //���±���ֵΪ0
					}
					else if (r_lc_low_alarm < 1) //20180312
					{
						r_lc_low_alarm = 1;
					}
					r_set_state = SET_NC_LC;
					goto SaveKey2;
				}
				else
				{
					goto SetErr;
				}
			}
			else if (r_set_state == SET_LC_HIGH)
			{
				if (f_zf == ZHENG_SIGN)
				{
					if ((r_swtj * 10 + r_gwtj) > 50 || (38 + (r_swtj * 10 + r_gwtj)) < (r_lczt + 1))
					{
						goto SetErr;
					}
					else
					{
						r_lc_high_alarm = r_swtj * 10 + r_gwtj + 38 - r_lczt;
						r_set_state = SET_NC_LC;
						goto SaveKey2;
					}
				}
				else
				{
					goto SetErr;
				}
			}
			else if (r_set_state == SET_LC_LOW) //��ص��±����趨ֵ
			{
				if (f_zf == ZHENG_SIGN) //����
				{
					//if(((38+(r_swtj*10+r_gwtj))>(r_lczt-1))&& (r_lczt >= (LC_SET_TEMP_MIN+38) ) )
					if ((38 + (r_swtj * 10 + r_gwtj)) > (r_lczt - 1)) //��������ֵ���ڻ������������¶�
					{
						goto SetErr;
					}
					else
					{
						if (r_lczt >= (LC_SET_TEMP_MIN + 38))
						{
							r_lc_low_alarm = r_lczt - (38 + (r_swtj * 10 + r_gwtj));
						}
						else
						{
							r_lc_low_alarm = r_lczt - 38; //ʵ���ǵ��趨�¶�С�������Ϳ������¶�,���±���ֵǿ��Ϊ0��
						}
						r_set_state = SET_NC_LC;
						goto SaveKey2;
					}
				}
				else //����
				{
					//if ((r_swtj*10+r_gwtj)>=38)
					//{
					//goto SetErr;
					/*}
  		   	else if((38-(r_swtj*10+r_gwtj))>(r_lczt-5))
  		    {
  		      goto SetErr;
  		    }
  		    else
  		    {*/
					r_lc_low_alarm = r_lczt - 38; //����ʵ���趨�ĵ��±���ֵ����0��
					r_set_state = SET_NC_LC;
					goto SaveKey2;
					//}
				}
			}
			else if (r_set_state == SET_LD_WDJZ) //�䶳�¶�У׼
			{
				r_set_state = SET_NC_LC;
				r_ldwdjz = r_ldwdjzx;
				goto SaveKey2;
			}
			else if (r_set_state == SET_LC_WDJZ)
			{
				r_set_state = SET_NC_LC;
				r_lcwdjz = r_lcwdjzx;
				goto SaveKey2;
			}
			else if (r_set_state == SET_LC_XSWDJZ)
			{
				r_set_state = SET_NC_LC;
				r_lcxswdjz = r_lcxswdjzx;
				goto SaveKey2;
			}
			else if (r_set_state == SET_LCYJ_DELAY) //���ѹ���ӳ�ʱ��
			{
				r_set_state = SET_NC_LC;
				t_yj_delay = t_yj_delayx;
				goto SaveKey2;
			}
			else if (r_set_state == SET_LD_BJYCSJ) //�䶳�����ӳ�ʱ��
			{
				r_set_state = SET_NC_LD;
				u8_ld_bjycsj = u8_ld_bjycsjx;
				goto SaveKey2;
			}
			else if (r_set_state == SET_LC_BJYCSJ)
			{
				r_set_state = SET_NC_LC;
				u8_lc_bjycsj = u8_lc_bjycsjx;
				goto SaveKey2;
			}
			else if (r_set_state == SET_LDYJ) //�䶳ѹ��
			{
				r_set_state = SET_NC_LD;
				goto SaveKey2;
			}
			else if (r_set_state == SET_LCYJ)
			{
				r_set_state = SET_NC_LC;
				goto SaveKey2;
			}
			else if (r_set_state == SET_YEAR)
			{
				r_set_state = SET_MONTH;

				goto NotSaveKey2;
			}
			else if (r_set_state == SET_MONTH)
			{
				r_set_state = SET_DAY;
				goto NotSaveKey2;
			}
			else if (r_set_state == SET_DAY)
			{
				r_set_state = SET_HOUR;
				goto NotSaveKey2;
			}
			else if (r_set_state == SET_HOUR)
			{
				r_set_state = SET_MINUTE;
				goto NotSaveKey2;
			}
			else if (r_set_state == SET_MINUTE)
			{
				r_set_state = r1;
				ACKok = 0;
				Cnt_SetTime_Sended = 0;
				//  report_time=  1;
				//  t_report_time = t_20ms;
				//  u16_random_time = u16_random;
				goto NotSaveKey2;
			}
		}
	}
	else if (r_sfkeyz == KEY_TEST_ALARM) //��������
	{
		if (!f_lock)
		{
			f_test_alarm = ON;
			u8_test_alarm_count = 0;
			u8_test_alarm_time = t_20ms;
			goto NotSaveKey2;
		}
		else
		{
			f_ld_DispErrs = 1;
			t_ld_err_disp = t_halfsec;
			f_ld_low_disp = 1;
			f_ld_high_disp = 1;

			f_lc_DispErrs = 1;
			t_lc_err_disp = t_halfsec;
			f_BatteryErrDisp = 1;
			f_hw_sensor_err_disp = 1;
			f_hw_high_disp = 1;
			f_lc_low_disp = 1;
			f_lc_high_disp = 1;
			f_lcXs_err_disp=1;
			f_lcHs_err_disp=1;
			f_lc_sensor_err_disp=1;
			f_lcLn_err_disp=1;
			goto NotSaveKey2;
		}
	}
	else if (r_sfkeyz == KEY_DISABLE_BUZZ) //����
	{
		f_stop_alarm = ON;//ÿ��һ����������ôϵͳ������ͳ���´εı���ʱ��
		t_stop_alarm = 0;
		goto NotSaveKey2;
	}
	else if (r_sfkeyz == KEY_SWITCH) //�л���
	{
	#ifdef Need_Ld
		if (r_set_state == SET_LD_L0)
		{
			r_set_state = SET_LC_L0;
			r_flash_bit = BIT3;
			goto NotSaveKey2;
		}
		else if (r_set_state == SET_LC_L0)
		{
			r_set_state = SET_LD_L0;
			r_flash_bit = BIT3;
			goto NotSaveKey2;
		}
		else if (r_set_state == SET_LC_WDJZ)
		{
			r_set_state = SET_LD_WDJZ;
			r_lcwdjz = r_lcwdjzx;
			goto SaveKey2;
		}
		else if (r_set_state == SET_LC_XSWDJZ)
		{
			r_set_state = SET_LD_WDJZ;
			r_lcxswdjz = r_lcxswdjzx;
			goto SaveKey2;
		}
		else if (r_set_state == SET_LD_WDJZ)
		{
			r_set_state = SET_LC_WDJZ;
			r_ldwdjz = r_ldwdjzx;
			goto SaveKey2;
		}
		else if (r_set_state == SET_LC_BJYCSJ)
		{
			r_set_state = SET_LD_BJYCSJ;
			u8_lc_bjycsj = u8_lc_bjycsjx;
			goto SaveKey2;
		}
		else if (r_set_state == SET_LD_BJYCSJ)
		{
			r_set_state = SET_LC_BJYCSJ;
			u8_ld_bjycsj = u8_ld_bjycsjx;
			goto SaveKey2;
		}
		else if (r_set_state == SET_LDYJ)
		{
			r_set_state = SET_LCYJ;
			goto SaveKey2;
		}
		else if (r_set_state == SET_LCYJ)
		{
			r_set_state = SET_LDYJ;
			goto SaveKey2;
		}
	#else
		if (r_set_state == SET_LD_L0)
		{
			r_set_state = SET_LC_L0;
			r_flash_bit = BIT3;
			goto NotSaveKey2;
		}
//		else if (r_set_state == SET_LC_L0)
//		{
//			r_set_state = SET_LD_L0;
//			r_flash_bit = BIT3;
//			goto NotSaveKey2;
//		}
		else if (r_set_state == SET_LD_WDJZ)
		{
			r_set_state = SET_LC_WDJZ;
			r_ldwdjz = r_ldwdjzx;
			goto SaveKey2;
		}
		else if (r_set_state == SET_LC_WDJZ)
		{
			r_set_state = SET_LD_WDJZ;
			r_lcwdjz = r_lcwdjzx;
			goto SaveKey2;
		}
		else if (r_set_state == SET_LD_BJYCSJ)//�����ӳ�ʱ��
		{
			r_set_state = SET_LC_BJYCSJ;
			u8_ld_bjycsj = u8_ld_bjycsjx;
			goto SaveKey2;
		}
//		else if (r_set_state == SET_LC_BJYCSJ)
//		{
//			r_set_state = SET_LD_BJYCSJ;
//			u8_lc_bjycsj = u8_lc_bjycsjx;
//			goto SaveKey2;
//		}
		else if (r_set_state == SET_LDYJ)
		{
			r_set_state = SET_LCYJ;
			goto SaveKey2;
		}
	#endif
		
	}
	else if (r_sfkeyz == KEY_DEFROST) //��˪���������������ص�
	{
		if(f_LightStatusPin==1){//
			f_LightStatusPin=0;
		}else{
			f_LightStatusPin=1;
		}
		BuzzBi();
	}
	return;
SaveKey2:
	f_need_write_e2 = ON;
	t_write_e2 = t_halfsec;
NotSaveKey2:
	f_key_kp = 1;
	BuzzBi();
	return;
SetErr:
	f_key_kp = 1;
	BuzzBiBiBi();
}
//^^^^^^^^^^^^^^^^^^^^^^^^
void KeyPutDown(void)
{
	t_auto_lock = t_halfsec;
	r_sfkeyz = 0;
	if (r_keyz == KEY_UNLOCK)
	{//
		r_sfkeyz = KEY_UNLOCK;
	}
	else if (r_keyz == KEY_SET)
	{
		if (r_set_state1 == SET_F1) //SET_F1=2
		{
			r_set_state1 = SET_NC; //SET_NC=0
			f_lock = OFF;//����
			goto NotSaveKey;
		}
		else
		{
			r_sfkeyz = KEY_SET;
		}
	}
	else if (r_keyz == KEY_DISABLE_BUZZ)
	{
		r_sfkeyz = KEY_DISABLE_BUZZ;
	}
	else if (r_keyz == KEY_ADJUST)
	{//�������ж����н����ϵĵ�ǰ��ʾ״̬�������е���
		if (r_set_state1 == SET_F0)
		{
			r_set_state1 = SET_F1;
			r_flash_bit = BIT3;
			goto NotSaveKey;
		}
		else if (r_set_state == SET_LD_L0)
		{
			r_set_state = SET_LD_L1;
			r_flash_bit = BIT3;
			goto NotSaveKey;
		}
		else if (r_set_state == SET_LD_L1)
		{
			r_set_state = SET_LD_L2;
			r_flash_bit = BIT3;
			goto NotSaveKey;
		}
		else if (r_set_state == SET_LD_L2)
		{
			r_set_state = SET_LD_L0;//�Ź���ֻ����ʪ�ȵĸ߱���(L1)�͵ͱ���(L0)
			r_flash_bit = BIT3;
			goto NotSaveKey;
		}
		else if (r_set_state == SET_LD_L3)
		{
			r_set_state = SET_LD_L4;
			r_flash_bit = BIT3;
			goto NotSaveKey;
		}
		else if (r_set_state == SET_LD_L4)
		{
			r_set_state = SET_LD_L5;
			r_flash_bit = BIT3;
			goto NotSaveKey;
		}
		else if (r_set_state == SET_LD_L5)
		{
			r_set_state = SET_LD_L0;
			r_flash_bit = BIT3;
			goto NotSaveKey;
		}
		else if (r_set_state == SET_LC_L0)
		{
			r_set_state = SET_LC_L1;
			r_flash_bit = BIT3;
			goto NotSaveKey;
		}
		else if (r_set_state == SET_LC_L1)
		{
			r_set_state = SET_LC_L2;
			r_flash_bit = BIT3;
			goto NotSaveKey;
		}
		else if (r_set_state == SET_LC_L2)
		{
			r_set_state = SET_LC_L3;
			r_flash_bit = BIT3;
			goto NotSaveKey;
		}
		else if (r_set_state == SET_LC_L3)
		{
			r_set_state = SET_LC_L4;
			r_flash_bit = BIT3;
			goto NotSaveKey;
		}
		else if (r_set_state == SET_LC_L4)
		{
			r_set_state = SET_LC_L5;
			r_flash_bit = BIT3;
			goto NotSaveKey;
		}
		else if (r_set_state == SET_LC_L5)
		{
			r_set_state = SET_LC_L6;
			r_flash_bit = BIT3;
			goto NotSaveKey;
		}
		else if (r_set_state == SET_LC_L6)//��ʾ��ؿ��ƴ��������¶�
		{
			r_set_state = SET_LC_L7;
			r_flash_bit = BIT3;
			goto NotSaveKey;
		}
		else if (r_set_state == SET_LC_L7)//��ʾ��˪���������¶�
		{
			r_set_state = SET_LC_L8;
			r_flash_bit = BIT3;
			goto NotSaveKey;
		}
		else if (r_set_state == SET_LC_L8)//��ʾ�������������¶�
		{
			r_set_state = SET_LC_L0;
			r_flash_bit = BIT3;
			goto NotSaveKey;
		}
		else if (r_set_state == SET_LD || r_set_state == SET_LC)
		{
			if (r_flash_bit == 2) //�䶳���ʮλ����
			{
				r_swtj++;
				if (r_swtj > 9)
				{
					r_swtj = 0;
				}
			}
			else if (r_flash_bit == 3) //�䶳��ظ�λ����
			{
				r_gwtj++;
				if (r_gwtj > 9)
				{
					r_gwtj = 0;
				}
			}
			goto NotSaveKey;
		}
		else if (r_set_state == SET_LD_HIGH || r_set_state == SET_LC_HIGH)
		{
			if(r_set_state == SET_LD_HIGH )//�䶳(ʪ��)�������õ�100�����������õ����λ
			{
				if (r_flash_bit == 1) //�䶳(ʪ��)���±���ֵ���λ����
				{
					f_zf = 0;//���Ų���ʾ
					r_bwtj++;
					if (r_bwtj > 1)
					{
						r_bwtj = 0;
					}
				}
				else if (r_flash_bit == 2)
				{
					r_swtj++;
					if (r_swtj > 9)
					{
						r_swtj = 0;
					}
				}
				else if (r_flash_bit == 3)
				{
					r_gwtj++;
					if (r_gwtj > 9)
					{
						r_gwtj = 0;
					}
				}
			}
			else
			{
				if (r_flash_bit == 1) //�䶳��ظ��±���ֵ����λ����
				{
					if (f_zf)
					{
						f_zf = 0;
					}
					else
					{
						f_zf = 1;
					}
				}
				else if (r_flash_bit == 2)
				{
					r_swtj++;
					if (r_swtj > 9)
					{
						r_swtj = 0;
					}
				}
				else if (r_flash_bit == 3)
				{
					r_gwtj++;
					if (r_gwtj > 9)
					{
						r_gwtj = 0;
					}
				}
			}
			goto NotSaveKey;
		}
		else if (r_set_state == SET_LD_LOW || r_set_state == SET_LC_LOW)
		{
			if(r_set_state == SET_LD_LOW)//�䶳(ʪ��)�����趨
			{
				if (r_flash_bit == 1) //�䶳(ʪ��)���±���ֵ���λ����
				{
					f_zf = 0;//��λһֱ����Ҳ���ǺڵĲ���ʾ
					r_bwtj=0;//��λ��Ϊ0
					r_flash_bit=2;
				}
				if (r_flash_bit == 2)
				{
					r_swtj++;
					if (r_swtj > 9)
					{
						r_swtj = 0;
					}
				}
				else if (r_flash_bit == 3)
				{
					r_gwtj++;
					if (r_gwtj > 9)
					{
						r_gwtj = 0;
					}
				}
				
			}
			else
			{
				if (r_flash_bit == 1) //�䶳��ص��±���ֵ����λ����
				{
					if (f_zf)
					{
						f_zf = 0;
					}
					else
					{
						f_zf = 1;
					}
				}
				else if (r_flash_bit == 2)
				{
//					if (r_set_state == SET_LC_LOW)
//					{
						if (r_lczt >= (LC_SET_TEMP_MIN + 38)) //zyj  100604
						{
							r_swtj++;
							if (r_swtj > 9)
							{
								r_swtj = 0;
							}
						}
//					}
//					else
//					{
//						r_swtj++;
//						if (r_swtj > 9)
//						{
//							r_swtj = 0;
//						}
//					}
				}
				else if (r_flash_bit == 3)
				{
//					if (r_set_state == SET_LC_LOW)
//					{
						if (r_lczt >= (LC_SET_TEMP_MIN + 38)) //zyj  100604
						{
							r_gwtj++;
							if (r_gwtj > 9)
							{
								r_gwtj = 0;
							}
						}
//					}
//					else
//					{
//						r_gwtj++;
//						if (r_gwtj > 9)
//						{
//							r_gwtj = 0;
//						}
//					}
				}
			}
			goto NotSaveKey;
		}
		else if (r_set_state == SET_LDYJ)
		{
//			if ((flag_ajszyjkt & 0x01) == 0x01)
//				flag_ajszyjkt = flag_ajszyjkt & 0xfe;
//			else
//				flag_ajszyjkt = flag_ajszyjkt | 0x01;
			goto NotSaveKey;
		}
		else if (r_set_state == SET_LCYJ)
		{
//			if ((flag_ajszyjkt & 0x02) == 0x02)
//				flag_ajszyjkt = flag_ajszyjkt & 0xfd;
//			else
//				flag_ajszyjkt = flag_ajszyjkt | 0x02;
			goto NotSaveKey;
		}
		else if (r_set_state == SET_LD_WDJZ)
		{
			r_ldwdjzx++;
			if (r_ldwdjzx > 35)
			{//��ƫ����15
				r_ldwdjzx = 5;
			}
			goto NotSaveKey;
		}
		else if (r_set_state == SET_LC_WDJZ)
		{
			r_lcwdjzx++;
			if (r_lcwdjzx > 150)
			{
				r_lcwdjzx = 50;
			}
			goto NotSaveKey;
		}
		else if (r_set_state == SET_LC_XSWDJZ)
		{
			r_lcxswdjzx++;
			if (r_lcxswdjzx > 150)
			{
				r_lcxswdjzx = 50;
			}
			goto NotSaveKey;
		}
		else if (r_set_state == SET_LCYJ_DELAY)
		{
			t_yj_delayx++;
			if (t_yj_delayx > 10)
			{
				t_yj_delayx = 1;
			}
			goto NotSaveKey;
		}
		else if (r_set_state == SET_LD_BJYCSJ)
		{
			u8_ld_bjycsjx++;
			if (u8_ld_bjycsjx > 10)
			{
				u8_ld_bjycsjx = 0;
			}
			goto NotSaveKey;
		}
		else if (r_set_state == SET_LC_BJYCSJ)
		{
			u8_lc_bjycsjx++;
			if (u8_lc_bjycsjx > 10)
			{
				u8_lc_bjycsjx = 0;
			}
			goto NotSaveKey;
		}
		else if (r_set_state == SET_YEAR)
		{
			u8_year++;
			if (u8_year > 30)
			{
				u8_year = 10;
			}
			goto NotSaveKey;
		}
		else if (r_set_state == SET_MONTH)
		{
			u8_month++;
			if (u8_month > 12)
			{
				u8_month = 1;
			}
			goto NotSaveKey;
		}
		else if (r_set_state == SET_DAY)
		{
			u8_day++;
			if (u8_day > 31)
			{
				u8_day = 1;
			}
			goto NotSaveKey;
		}
		else if (r_set_state == SET_HOUR)
		{
			u8_hour++;
			if (u8_hour > 23)
			{
				u8_hour = 0;
			}
			goto NotSaveKey;
		}
		else if (r_set_state == SET_MINUTE)
		{
			u8_minute++;
			if (u8_minute > 59)
			{
				u8_minute = 0;
			}
			goto NotSaveKey;
		}
		else
		{
			r_sfkeyz = KEY_ADJUST;
		}
	}
	else if (r_keyz == KEY_TEST_ALARM)
	{
		r_sfkeyz = KEY_TEST_ALARM;
	}
	else if (r_keyz == KEY_DEFROST)
	{
		r_sfkeyz = KEY_DEFROST;
	}
	else if (r_keyz == KEY_SWITCH)
	{
		r_sfkeyz = KEY_SWITCH;
	}
	return;
NotSaveKey:
	f_key_kp = 1;
	BuzzBi();
}
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
void KeyPut5s(void)
{
	r_sfkeyz = 0;
	t_auto_lock = t_halfsec;
	if (r_keyz == KEY_UNLOCK) //����
	{
		if (f_lock)
		{
			r_set_state1 = SET_F0;
			r_flash_bit = BIT3;
			t_auto_lock = t_halfsec;
			goto NotSaveKey2;
		}
	}
	else if (r_keyz == KEY_DELAY) //���ѹ���ӳ�ʱ��
	{
		if (!f_lock)
		{
			r_set_state = SET_LCYJ_DELAY;
			t_yj_delayx = t_yj_delay;
			goto NotSaveKey2;
		}
	}
	else if (r_keyz == KEY_WDJZ) //�¶�У׼
	{
		if (!f_lock)
		{
			if (r_set_state == SET_NC_LD)
			{
				r_set_state = SET_LD_WDJZ;
				r_ldwdjzx = r_ldwdjz;
			}
			else
			{
				r_set_state = SET_LC_WDJZ;
				r_lcwdjzx = r_lcwdjz;
			}

			goto NotSaveKey2;
		}
	}
	else if (r_keyz == KEY_LCXSWDJZ) //�¶�У׼(��ʾ������)
	{
		if (!f_lock)
		{
			if (r_set_state == SET_NC_LD)
			{
				r_set_state = SET_LD_WDJZ;
				r_ldwdjzx = r_ldwdjz;
			}
			else
			{
				r_set_state = SET_LC_XSWDJZ;
				r_lcxswdjzx = r_lcxswdjz;
			}

			goto NotSaveKey2;
		}
	}
	else if (r_keyz == KEY_DEFROST) //��˪
	{
	//�ĳ����Զ���˪�����������Ū���˿�����ص�
//		if (!f_lock)
//		{
//			if (f_key_defrost)
//			{
//				f_key_defrost = OFF;
//			}
//			else
//			{
//				f_key_defrost = ON;
//				f_key_defrost_disp = 1;
//				t_ld_err_disp = t_halfsec;
//			}
//			f_compressor_on_dly = 1;
//			f_on_off_dly = 0;
//			goto NotSaveKey2;
//		}
	}
	else if (r_keyz == KEY_BJYCSJ) //FMQ�����ӳ�ʱ��  ����+����
	{
		if (!f_lock)
		{
			u8_ld_bjycsjx = u8_ld_bjycsj;
			u8_lc_bjycsjx = u8_lc_bjycsj;
			if (r_set_state == SET_NC_LD)
			{
				r_set_state = SET_LD_BJYCSJ;
			}
			else
			{
				r_set_state = SET_LC_BJYCSJ;
			}
			goto NotSaveKey2;
		}
	}
	else if (r_keyz == KEY_DATE)  //����+����+����
	{
		if (!f_lock)
		{
			if (r_set_state < 31)
			{
				r1 = r_set_state;
			}
			if (r_set_state <= 32) //zyj  100621
			{
				r_set_state = SET_YEAR;
			}
			goto NotSaveKey2;
		}
	}
	else if (r_keyz == KEY_UsbCheck) //usb��ѯ
	{
		if (!f_lock)
		{
			if (r_set_state < 31)
			{
				r1 = r_set_state;
			}
			if (r_set_state != 31)
			{
				r_set_state = USB_Insert_Or_PullOut;
			}
			goto NotSaveKey2;
		}
	}
	return;
NotSaveKey2:
	BuzzBi();
	f_key_kp = 1;
}
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
void AutoLock(void)
{
	if ((uchar)(t_halfsec - t_auto_lock) >= 20)
	{//�Ӱ��¿�ʼ���㣬10S�ӣ���������Ǵ洢���õ���˼
		f_lock = 1;//����
		if (r_set_state == SET_LD)//�����䶳�¶�
		{

			if ((r_bwtj*100+r_swtj * 10 + r_gwtj) > 0 && (r_bwtj*100+r_swtj * 10 + r_gwtj) <= 100)
			{
				r_ldzt = (r_bwtj*100+r_swtj * 10 + r_gwtj);
				if((r_ldzt-r_ld_low_alarm)<1){r_ld_low_alarm=r_ldzt;}
				if((r_ldzt+r_ld_high_alarm)>100){r_ld_low_alarm=100-r_ldzt;}
				f_need_write_e2 = ON;
				t_write_e2 = t_halfsec;//����t_write_e2���ֵ������Ϊ�˱���Ƶ��Щe2
			}
			else
			{
				BuzzBiBiBi();
			}
		}
		else if (r_set_state == SET_LD_HIGH)//�����䶳����ֵ��λ
		{
			#ifdef Need_Ld
				if (f_zf == ZHENG_SIGN)
				{
					if ((r_swtj * 10 + r_gwtj) > 50)
					{
						BuzzBiBiBi();
					}
					else
					{
						r_ld_high_alarm = r_swtj * 10 + r_gwtj + 200 - r_ldzt;
						f_need_write_e2 = ON;
						t_write_e2 = t_halfsec;
					}
				}
				else
				{
					if ((200 - (r_swtj * 10 + r_gwtj)) < (r_ldzt + 1))
					{
						BuzzBiBiBi();
					}
					else
					{
						r_ld_high_alarm = 200 - (r_swtj * 10 + r_gwtj) - r_ldzt;
						f_need_write_e2 = ON;
						t_write_e2 = t_halfsec;
					}
				}
			#else
				if (f_zf == ZHENG_SIGN)
				{
					if ((r_bwtj * 100+r_swtj * 10 + r_gwtj) > 100)
					{
						BuzzBiBiBi();
					}
					else
					{
						r_ld_high_alarm = r_bwtj * 100+r_swtj * 10 + r_gwtj- r_ldzt;
						if(r_ld_high_alarm<1){r_ld_high_alarm=1;}
						f_need_write_e2 = ON;
						t_write_e2 = t_halfsec;
					}
				}
				else
				{
						BuzzBiBiBi();
				}
			#endif
		}
		else if (r_set_state == SET_LD_LOW)//�����䶳����ֵ��λ
		{
			#ifdef Need_Ld
				if (f_zf == ZHENG_SIGN)
				{
					BuzzBiBiBi();
				}
				else
				{
					if ((200 - (r_swtj * 10 + r_gwtj)) < 140)
					{
						BuzzBiBiBi();
					}

					else if ((200 - (r_swtj * 10 + r_gwtj)) > (r_ldzt - 1))
					{
						BuzzBiBiBi();
					}
					else
					{
						r_ld_low_alarm = r_ldzt - (200 - (r_swtj * 10 + r_gwtj));
						f_need_write_e2 = ON;
						t_write_e2 = t_halfsec;
					}
				}
			#else
				if (f_zf == ZHENG_SIGN)
				{
//					if (((r_swtj * 10 + r_gwtj)) < 0)
//					{
//						BuzzBiBiBi();
//					}

//					else 
						if (((r_swtj * 10 + r_gwtj)) > (r_ldzt - 1))
					{
						BuzzBiBiBi();
					}
					else
					{
						r_ld_low_alarm = r_ldzt - ((r_swtj * 10 + r_gwtj));
						if(r_ld_low_alarm<1){r_ld_low_alarm=1;}
						f_need_write_e2 = ON;
						t_write_e2 = t_halfsec;
					}
				}
				else
				{
					BuzzBiBiBi();
				}
			#endif
		}
		else if (r_set_state == SET_LC)
		{//�������
			//if(((r_swtj*10+r_gwtj)<=16)&&((r_swtj*10+r_gwtj)>=5))//20171106  if((r_swtj*10+r_gwtj)<=16)
			if (((r_swtj * 10 + r_gwtj) <= LC_SET_TEMP_MAX) && ((r_swtj * 10 + r_gwtj) >= LC_SET_TEMP_MIN)) //  2019-9-11 ���˷�
			{
				r_lczt = 38 + (r_swtj * 10 + r_gwtj);
				r_lczt_float = r_lczt;
				if (r_lczt < (LC_SET_TEMP_MIN + 38))
				{
					r_lc_low_alarm = r_lczt - 38;
				}
				else
				{

					if (r_lc_low_alarm < 1) //20180312
					{
						r_lc_low_alarm = 1;
					}
				}
				f_need_write_e2 = ON;
				t_write_e2 = t_halfsec;
			}
			else
			{
				BuzzBiBiBi();
			}
		}
		else if (r_set_state == SET_LC_HIGH)
		{//������ر���ֵ��λ
			if (f_zf == ZHENG_SIGN)
			{
				if ((r_swtj * 10 + r_gwtj) > 50 || (38 + (r_swtj * 10 + r_gwtj)) < (r_lczt + 1))
				{
					BuzzBiBiBi();
				}
				else
				{
					r_lc_high_alarm = r_swtj * 10 + r_gwtj + 38 - r_lczt;
					f_need_write_e2 = ON;
					t_write_e2 = t_halfsec;
				}
			}
			else
			{
				BuzzBiBiBi();
			}
		}
		else if (r_set_state == SET_LC_LOW)
		{//������ر���ֵ��λ
			if (f_zf == ZHENG_SIGN)
			{
				if ((38 + (r_swtj * 10 + r_gwtj)) > (r_lczt - 1)) //���±���ֵ���ڻ���������¶�
				{
					BuzzBiBiBi();
				}
				else
				{
					if (r_lczt >= (LC_SET_TEMP_MIN + 38))
					{
						r_lc_low_alarm = r_lczt - (38 + (r_swtj * 10 + r_gwtj));
					}
					else
					{
						r_lc_low_alarm = r_lczt - 38;
					}
					f_need_write_e2 = ON;
					t_write_e2 = t_halfsec;
				}
			}
			else //��ֵǿ��Ϊ0
			{
				r_lc_low_alarm = r_lczt - 38;
				f_need_write_e2 = ON;
				t_write_e2 = t_halfsec;
			}
		}
		else if (r_set_state == SET_LD_WDJZ)
		{//�¶Ƚ���
			r_ldwdjz = r_ldwdjzx;
			f_need_write_e2 = ON;
			t_write_e2 = t_halfsec;
		}
		else if (r_set_state == SET_LC_WDJZ)
		{
			r_lcwdjz = r_lcwdjzx;
			f_need_write_e2 = ON;
			t_write_e2 = t_halfsec;
		}
		else if (r_set_state == SET_LC_XSWDJZ)
		{
			r_lcxswdjz = r_lcxswdjzx;
			f_need_write_e2 = ON;
			t_write_e2 = t_halfsec;
		}
		
		else if (r_set_state == SET_LCYJ_DELAY)
		{
			t_yj_delay = t_yj_delayx;
			f_need_write_e2 = ON;
			t_write_e2 = t_halfsec;
		}
		else if (r_set_state == SET_LD_BJYCSJ)
		{
			u8_ld_bjycsj = u8_ld_bjycsjx;
			f_need_write_e2 = ON;
			t_write_e2 = t_halfsec;
		}
		else if (r_set_state == SET_LC_BJYCSJ)
		{
			u8_lc_bjycsj = u8_lc_bjycsjx;
			f_need_write_e2 = ON;
			t_write_e2 = t_halfsec;
		}
		else if (r_set_state == SET_LDYJ)
		{
			f_need_write_e2 = ON;
			t_write_e2 = t_halfsec;
		}
		else if (r_set_state == SET_LCYJ)
		{
			f_need_write_e2 = ON;
			t_write_e2 = t_halfsec;
		}
		if (r_set_state == 31)
		{
			r_set_state = r1;
		}
		else if (r_set_state >= 32) //zyj 100621
		{
			r_set_state = r1;
			//  report_time =  1;
			//  t_report_time = t_20ms;
			//  u16_random_time = u16_random;
		}
		else if (r_set_state >= 15){
			r_set_state = SET_NC_LC;
		}
		else{
			r_set_state = SET_NC_LC;
		}
	}
}

/******************************************************/
/****************�ж�ѹ����ͣ����**********************/
/******************************************************/
void ldCompressorRun10Hour(void);
void ldCompRunOneHour(void);
/******************************************************/
void LdCompressorJudge(void) //�䶳ѹ���ж�
{
	unsigned int r16_on_ad;
	unsigned int r16_off_ad;

#if LD_COMP_RUN_MAX_1_HOUR
	ldCompRunOneHour();
#elif LD_COMP_RUN_MAX_10_HOUR
	ldCompressorRun10Hour();
#endif

	// ldCompressorRun10Hour();
	// ldCompRunOneHour();

	if (!f_first_ad)
		return;
	if ((uchar)(t_tens - t_on_off_dly) >= 6)
	{
		f_on_off_dly = 0; //��ͣ��ѹ���ӳ�1����
	}

	if (f_power_off)
	{
		f_compressor_on_dly = 0;
		t_yj_dly_time = 0;
		goto CompressorOff;
	}
	if (!ldyj)
	{
		goto CompressorOff;
	}

	if (f_key_defrost == ON)
	{
		goto CompressorOff;
	}
	if (f_ldCompressorProtect) //ѹ��10Сʱ����
	{
		goto CompressorOff;
	}
	if (f_ldComp1HourProtect) //ѹ��1Сʱ����
	{
		goto CompressorOff;
	}
//	if (f_ld_sensor_err)
//	{
//		if (f_onemin)
//		{
//			t_compressor++;
//		}
//		if (t_compressor < 30)
//		{
//			goto CompressorOn;
//		}
//		else if (t_compressor < 40)
//		{
//			goto CompressorOff;
//		}
//		else
//		{
//			t_compressor = 0;
//			goto CompressorOn;
//		}
//	}
	t_compressor = 0;
	r16_on_ad = tab_on[r_ldzt - 10];
	r16_off_ad = tab_off[r_ldzt - 10];
	if (f_compressor == ON)
	{
		if (r16_ldad > r16_off_ad)
			goto CompressorOff;
	}
	else
	{
		if (r16_ldad < r16_on_ad)
			goto CompressorOn;
	}
	return;

CompressorOn:
	if (f_compressor_on_dly && !f_on_off_dly)
	{
		if (!f_compressor && delay_10sec)
		{
			f_compressor = ON;
			f_on_off_dly = 1;
			t_on_off_dly = t_tens;
			t_yj_delay_10sec = 0;
			delay_10sec = 0;
		}
	}
	return;
CompressorOff:
	//if(!f_on_off_dly)
	{
		if (f_compressor)
		{
			f_compressor = OFF;
			f_on_off_dly = 1;
			t_on_off_dly = t_tens;
		}
	}
	return;
}
/************************************************/
void Compressor_on_delay(void) //�ϵ�ѹ���ӳٿ�ʱ��  �ɰ����趨
{
	if (!f_compressor_on_dly)
	{
		if (f_tens)//ÿ10Sִ��һ��
		{
			t_yj_dly_time++;
			if (t_yj_dly_time >= (t_yj_delay * 6))
			{
				f_compressor_on_dly = 1;//ֱ���´ζϵ�һֱ�����״̬
				t_yj_dly_time = 0;
			}
		}
	}
}
/**************************************************/
void Compressor_delay_10sec(void)
{
	if (!delay_10sec)
	{
		if (f_halfsec)
		{
			t_yj_delay_10sec++;
			if (t_yj_delay_10sec >= 20)
			{
				t_yj_delay_10sec = 0;
				delay_10sec = 1;
			}
		}
	}
}
/************�䶳ѹ����������10Сʱ����**************/
/************************************************/
void ldCompressorRun10Hour(void)
{
	if (!f_ldCompressorProtect)
	{
		if (!f_compressor)
		{
			u8_CompMin = 0;
			u8_CompHour = 0;
			return;
		}
		if (f_onemin)
		{
			if (++u8_CompMin >= 60)
			{
				u8_CompMin = 0;
				if (++u8_CompHour >= 10)
				{
					u8_CompHour = 0;
					f_ldCompressorProtect = 1;
					u8_CompStopTime = t_tens;
					// f_on_off_dly = 0;
				}
			}
		}
	}
	else
	{
		if ((unsigned char)(t_tens - u8_CompStopTime) >= 60) //10min
		{
			f_ldCompressorProtect = 0;
		}
	}
}

/************************************************/
/************�䶳ѹ����������1Сʱ����**************/
/************************************************/
void ldCompRunOneHour(void)
{
	if (!f_ldComp1HourProtect)
	{
		if (!f_compressor || (r_ldgzwd != r_ldzt))
		{
			u8_CompRunMin = 0;
			return;
		}
		if (f_onemin)
		{
			if (++u8_CompRunMin >= 60)
			{
				u8_CompRunMin = 0;
				f_ldComp1HourProtect = 1;
				u8_CompStop5min = t_tens;
				// f_on_off_dly = 0;
			}
		}
	}
	else
	{
		if ((uchar)(t_tens - u8_CompStop5min) >= 30) //5min
		{
			f_ldComp1HourProtect = 0;
		}
	}
}
/************************************************/
void lcCompressorRun10Hour(void);
void lcCompRunOneHour(void);
void lcCompressorAddup6Hour(void); // 140819
 void lcCompRunSixHour(void);

 /****************���ѹ�����Ƴ���****************/
void Lc_CompressorJudge(void)
{
	//lcCompressorRun10Hour();
	lcCompRunSixHour();//��˪ר�� ����ʱֹͣ��˪�߼�
//#if LC_COMP_RUN_MAX_1_HOUR
//	lcCompRunOneHour(); //����ϵͳ��ԱҪ������һСʱ�ģ�HW2019/7/29 9:50
//#elif LC_COMP_RUN_MAX_6_HOUR
//	lcCompressorAddup6Hour();
//#endif
	///lcCompRunOneHour();               ����ϵͳ��ԱҪ������һСʱ�ģ�HW2019/7/29 9:50
	//lcCompressorAddup6Hour();                       // 140819
	if (!f_first_ad)
		return;

	if ((uchar)(t_tens - t_lc_on_off_dly) >= 6) //1min   HW 20190814
	{
		f_lc_on_off_dly = 0;
	}
	if (f_power_off)
	{ // 140819
		f8_lcCompAddUp6HourProtect = 0;
		u16_lcComp_Addup_Minute = 0;
		f8_lcCompAddUp6HourTimer = 0;

		f_compressor_on_dly = 0;
		t_yj_dly_time = 0;
		goto Lc_CompressorOff;
	}
	if (!lcyj)
	{
		goto Lc_CompressorOff;
	}
//	if (f8_lcCompAddUp6HourProtect != 0) // ѹ���ۼ�6Сʱ����
//	{									 // 140819
//		goto Lc_CompressorOff;
//	}
//	if (f_lcCompressorProtect) //ѹ��30Сʱ����
//	{
//		goto Lc_CompressorOff;
//	}
//	if (f_lcComp1HourProtect) //ѹ��1Сʱ����
//	{
//		goto Lc_CompressorOff;
//	}
//	#ifdef Need_Ld//�䶳�Ļ���������
//	#else
//	if (f_key_defrost == ON)
//	{//����˪��ѹ�������������
//		goto Lc_CompressorOff;
//	}
//	#endif
	if (f_lc_sensor_err)
	{//��ؿ��ƴ��������ϣ����ݻ��¿���ѹ����
		//����С��16�ȣ�ѹ��������3���ӣ�ͣ��15����
		//16<���´�����<25��ѹ��������5���ӣ�ͣ��8����
		//25<���´�����<35��ѹ��������9���ӣ�ͣ��6����
		//35<���´�����<45��ѹ��������15���ӣ�ͣ��3����
		if (r_hwsjwd < (16+38) ){
			if (f_onemin)
			{
				t_lc_compressor++;
			}
			if (t_lc_compressor < 3)
			{
				f_defrost=OFF;
				goto Lc_CompressorOn;
			}
			else if (t_lc_compressor < 18)
			{
				f_defrost=ON;
				goto Lc_CompressorOff;
			}
			else
			{
				t_lc_compressor = 0;
				f_defrost=OFF;
				goto Lc_CompressorOn;
			}
		}else if((r_hwsjwd >= (16+38))&&(r_hwsjwd < (25+38))){
			if (f_onemin)
			{
				t_lc_compressor++;
			}
			if (t_lc_compressor < 5)
			{
				f_defrost=OFF;
				goto Lc_CompressorOn;
			}
			else if (t_lc_compressor < 13)
			{
				f_defrost=ON;
				goto Lc_CompressorOff;
			}
			else
			{
				t_lc_compressor = 0;
				f_defrost=OFF;
				goto Lc_CompressorOn;
			}
		}else if((r_hwsjwd >= (25+38))&&(r_hwsjwd < (35+38))){
			if (f_onemin)
			{
				t_lc_compressor++;
			}
			if (t_lc_compressor < 9)
			{	
				f_defrost=OFF;
				goto Lc_CompressorOn;
			}
			else if (t_lc_compressor < 15)
			{
				f_defrost=ON;
				goto Lc_CompressorOff;
			}
			else
			{
				t_lc_compressor = 0;
				f_defrost=OFF;
				goto Lc_CompressorOn;
			}
		}else if((r_hwsjwd >= (35+38))&&(r_hwsjwd < (45+38))){
			if (f_onemin)
			{
				t_lc_compressor++;
			}
			if (t_lc_compressor < 15)
			{
				f_defrost=OFF;
				goto Lc_CompressorOn;
			}
			else if (t_lc_compressor < 18)
			{
				f_defrost=ON;
				goto Lc_CompressorOff;
			}
			else
			{
				t_lc_compressor = 0;
				f_defrost=OFF;
				goto Lc_CompressorOn;
			}
			
		}else{//����45�� 
			if (f_onemin)
			{
				t_lc_compressor++;
			}
			if (t_lc_compressor < 15)
			{
				f_defrost=OFF;
				goto Lc_CompressorOn;
			}
			else if (t_lc_compressor < 18)
			{
				f_defrost=ON;
				goto Lc_CompressorOff;
			}
			else
			{
				t_lc_compressor = 0;
				f_defrost=OFF;
				goto Lc_CompressorOn;
			}
		}
	}else{
		t_lc_compressor = 0;
	}
	if(f_deHum_and_lcTemp==0){//��ǰû���ڳ�ʪ
		//�ж��Ƿ���Ҫ��˪
		if(LcHuaShuang_Flag){//��˪��������
			f_ln_fan=1;//��ָʾ��
			if (f_onemin)
			{
				u8_lcChuShuangTime++;
			}
			//1��	��˪ʱ��ﵽHUASHUANG_LASTED_TIME����
			if(u8_lcChuShuangTime>HUASHUANG_LASTED_TIME){//���1����
				LcHuaShuang_Flag=0;
				f_defrost=OFF;
				LcHuaShuang_Flag_after=1;
			}
			//2��	��˪�������¶�>5�ȣ�����ʱ��>1����
			if(r_lcHuaSHuangxswd_float>510+50){
				if (f_halfsec)
				{
					u8_lcChuShuangTime1++;
				}
				if(u8_lcChuShuangTime1>=120){
					u8_lcChuShuangTime1=0;
					LcHuaShuang_Flag=0;
					f_defrost=OFF;
					LcHuaShuang_Flag_after=1;
				}
			}else{//Ҫһֱ>5�Ȳ���
				u8_lcChuShuangTime1=0;
			}
			//3��	������ʾ�¶ȴ������¶�>6��
			if(r_lcwd_float>38+7){
					LcHuaShuang_Flag=0;
					f_defrost=OFF;
					LcHuaShuang_Flag_after=1;
			}
			//��������������û���㣬������˪
			if (f_halfsec)
			{
				u32_t_lcChuShuangTime1++;
			}
			if(u32_t_lcChuShuangTime1>=1800){//�ڷ������15���Ӻ�ͣ��
				//�ڷ��ͣ��
				f_nd_fan=OFF;
				u32_t_lcChuShuangTime1=1800;
			}else{//���뻯˪ʱ�������
				f_nd_fan=ON;
			}
			//����˿����
			f_defrost=ON;
			//ѹ����ͣ��
			goto Lc_CompressorOff;

		}else{//��ǰ����˪
			u8_lcChuShuangTime=0;
			u8_lcChuShuangTime1=0;
			u32_t_lcChuShuangTime1=0;
			f_defrost=OFF;
		}
		if(LcHuaShuang_Flag_after){//������˪��������ƺ����߼�
			//1��	��˪����ֹͣ����
			f_defrost=OFF;
			//2��	ѹ���������������ڷ��������
			if (f_halfsec)
			{
				u32_t_lcChuShuangTime2++;
				u32_t_lcChuShuangTime3++;
			}
			if(u32_t_lcChuShuangTime2>=240){//2���Ӻ󿪷��
				f_nd_fan=ON;
				LcHuaShuang_Flag_after=0;//�˳���˪�߼�,���������¶ȿ�����
				u32_t_lcChuShuangTime2=240;
			}else{//����ر�
				f_nd_fan=OFF;//�ر��������
			}
			if(u32_t_lcChuShuangTime3>=120){//1���Ӻ�ѹ��
                u32_t_lcChuShuangTime3=120;
				goto Lc_CompressorOn;
				
			}else{//����ر�
				goto Lc_CompressorOff;
			}
			

		}else{//�������е��߼�
		    f_ln_fan=0;
			u32_t_lcChuShuangTime2=0;
			u32_t_lcChuShuangTime3=0;
			t_lc_ChuShi_Compressor=0;
			f_nd_fan=ON;
			if (f_lc_compressor == ON)
			{//��ؿ��ƴ������¶�<=�����¶�               adֵԽС���¶�Խ��
				if (r_lcwd >= r_lczt){//��ؿ��ƴ������¶�>=�趨ֵ
					f_defrost=OFF;
					goto Lc_CompressorOn;
				}else{
					f_defrost=ON;
					goto Lc_CompressorOff;
				}
			}
			else
			{
				if (r_lcwd <= r_lczt){//��ؿ��ƴ������¶�<=�趨ֵ
					f_defrost=ON;
					goto Lc_CompressorOff;
				}else{
					f_defrost=OFF;
					goto Lc_CompressorOn;
				}
			}
		}
	}else{//��ʪ�߼�
		if(f_deHum_and_lcTemp==1){//ÿ��2���ӹ���1����
			if (f_halfsec)
			{
				t_lc_ChuShi_Compressor++;
			}
			if (t_lc_ChuShi_Compressor < 121)//61
				{
					//���ڷ��
					f_nd_fan=ON;
				}
				else if (t_lc_ChuShi_Compressor < 121+240)//361
				{
					//���ڷ��
					f_nd_fan=OFF;
				}
				else
				{
					t_lc_ChuShi_Compressor = 0;
					//���ڷ��
				}
			
		}else if(f_deHum_and_lcTemp==2){//ÿ��2���ӹ���20s
			if (f_halfsec)
			{
				t_lc_ChuShi_Compressor++;
			}
			if (t_lc_ChuShi_Compressor < 41)
			{
				//���ڷ��
				f_nd_fan=ON;
			}
			else if (t_lc_ChuShi_Compressor < 41+240)
			{
				//���ڷ��
				f_nd_fan=OFF;
			}
			else
			{
				t_lc_ChuShi_Compressor = 0;
				//���ڷ��
			}
		}
		goto Lc_CompressorOn;
		
	}
//	return;
Lc_CompressorOn:
	if (f_compressor_on_dly && !f_lc_on_off_dly)
	{
		if (!f_lc_compressor && delay_10sec)
		{
			
			if(Protect_HigHw_Temp==0&&Protect_HigLn_Temp==0&&Protect_Low_Temp==0){//���º��������¶ȶ�û���� Ҳû�е��±���
				f_lc_fan = ON;//����������
				f_lc_compressor = ON;//�����ѹ��
			}else{
				f_lc_fan = OFF;
				f_lc_compressor = OFF;
			}
			f_lc_on_off_dly = 1;
			t_lc_on_off_dly = t_tens;
			t_yj_delay_10sec = 0;
			delay_10sec = 0;
		}
	}
	return;
Lc_CompressorOff:
	if (!f_lc_on_off_dly)
	{
		if (f_lc_compressor)
		{
			if (1 == f8_lcCompAddUp6HourTimer)
			{//Ŀǰ��Զ�ò���ִ��
				f8_lcCompAddUp6HourTimer = 0;
				//
				f8_lcCompAddUp6HourProtect = 1;
				u8_lcCompAddUp6HourProtectTimer = t_tens;
				//f_lc_on_off_dly = 0;                          // ������5����
			}
			// 141017
			f_lc_compressor = OFF;
			f_lc_fan = OFF;
			f_lc_on_off_dly = 1;
			t_lc_on_off_dly = t_tens;
		}
	}
	return;
}
///////////////////////////////////////////////////

void lcCompressorAddup6Hour(void)
{
	if (0 == f8_lcCompAddUp6HourProtect)
	{
		if (1 == f8_lcCompAddUp6HourTimer)
		{
		}
		else if (f_lc_compressor)
		{
			if (f_onemin)
			{ // 140819
				u16_lcComp_Addup_Minute++;
				if (u16_lcComp_Addup_Minute >= 30 * 60) //����ϵͳ30HСʱ
				{
					f8_lcCompAddUp6HourTimer = 1;
					/*  141017        
             f8_lcCompAddUp6HourProtect = 1;
             u8_lcCompAddUp6HourProtectTimer = t_tens;            	
             f_lc_on_off_dly = 0;                          // ������5���� 
             */
				}
			}
		}
	}
	else
	{
		if ((unsigned char)(t_tens - u8_lcCompAddUp6HourProtectTimer) >= 180) //180*10/60=30min  HW
		{
			f8_lcCompAddUp6HourProtect = 0;
			u16_lcComp_Addup_Minute = 0;
		}
	}
}
/***************/
/************************************************/
// ѹ���ۼ�����30Сʱ 20190729
void lcCompressorRun10Hour(void)
{
	if (!f_lcCompressorProtect)
	{
		if (f_lc_compressor)
		{
			if (f_onemin)
			{
				if (++u8_lcCompMin >= 60)
				{
					u8_lcCompMin = 0;
					if (++u8_lcCompHour >= 30)
					{
						u8_lcCompHour = 0;
						f_lcCompressorProtect = 1;
						u8_lcCompStopTime = t_tens;
						// f_on_off_dly = 0;
					}
				}
			}
		}
	}
	else
	{
		if ((unsigned char)(t_tens - u8_lcCompStopTime) >= 180)
		{
			f_lcCompressorProtect = 0;
		}

//		if (r_lcad_12b >= LCCOMPRTSTOREAD) //���ѹ����������  HW
//		{
//			f_lcCompressorProtect = 0;
//		}
	}
}
/************���ѹ����������1Сʱ����**************/
//���¶ȴﵽ�趨�¶�,���ѹ��������������1Сʱ,ǿ��ͣ��5min
/***************************************************/
void lcCompRunOneHour(void)
{
	if (!f_lcComp1HourProtect)
	{
		if (!f_lc_compressor || (r_lcgzwd != r_lczt))
		{
			u8_lcCompRunMin = 0;
			return;
		}
		if (f_onemin)
		{
			if (++u8_lcCompRunMin >= 60)
			{
				u8_lcCompRunMin = 0;
				f_lcComp1HourProtect = 1;
				u8_lcCompStop5min = t_tens;
				//f_on_off_dly = 0;
			}
		}
	}
	else
	{
		if ((uchar)(t_tens - u8_lcCompStop5min) >= 30) //5min
		{
			f_lcComp1HourProtect = 0;
		}
	}
}

/************���ѹ����������6Сʱ����**************/
//��˪ר��ѹ���ۼ�����6H���һ�˪�������¶�<0.5��ʱ��Ҫ������˪�߼�
/***************************************************/
void lcCompRunSixHour(void)
{
	if (!f_lcCompSixHour)
	{
		if (f_lc_compressor )
		{
			if (f_onemin)
			{
				if (++u16_lcCompRunMinForHuaShuang >= 60*2)//�ۼ�����2h,���1���� //60*2 ��ʱΪ�˲��Ըĳ�20����
				{
					u16_lcCompRunMinForHuaShuang = 0;
					f_lcCompSixHour = 1;
					//f_on_off_dly = 0;
				}
			}
		}
	}
}


/********************�жϷ������*****************/
/*************************************************/
void LnFan(void)//�������
{
	if (f_power_off)
	{
		f_ln_fan = OFF;
	}
	else if (f_compressor == ON)
	{
		f_ln_fan = ON;
		t_fan2min = t_tens;
	}
	else
	{
		if ((unsigned char)(t_tens - t_fan2min) >= 12) //2min
		{
			f_ln_fan = OFF;
		}
	}
}

/********************�ж���������Ƴ���****************/
void Lc_lightProg(void)
{
	//������ڸĳ���Ч�ˣ�ͨ�������¼������ñ�־λf_LightStatusPin���Դﵽ�װ�����CN17����12V�ȵĿ����͹ر�
	if(f_LightStatusPin==1){//��˪��������
		//f_LightStatusPin = 1;
	}else{
		//f_LightStatusPin = 0;
	}
//	LED_OUT = 1;//û���ţ�����ͬ־˵������Ƴ���������
//	if (!DOOR_ONFLAG | !LED_ONFLAG) //if(!PTBD_PTBD7|!PTDD_PTDD0)�͵�ƽ��Ч
//	{
//		if ((uchar)(t_onems - t_lightms) >= 60)
//			LED_OUT = 1; //PTDD_PTDD2=1;   @20180920 cfj �ߵ�ƽ����
//		f_LightStatusPin = 1;
//	}
//	else
//	{
//		t_lightms = t_onems;
//		LED_OUT = 0; //PTDD_PTDD2=0;      @20180920 cfj
//		f_LightStatusPin = 0;
//	}
}
/********************�ڵ��������****************/
//void Nd_fan_Prog(void)
//{
//	if (DOOR_ONFLAG) //if(PTBD_PTBD7)    @20180920 cfj ��ع�����
//	{
//		f_door_switch = 0; // 100526  zyj
//		if (lcyj)//���ֵ��flag_ajszyjkt������flag_ajszyjkt��ֵ�ڰ����жϲ��ֽ��и�ֵ��Ҳ��E2�ж�ȡ������һ���������Ʒ����ͣ
//		{//�������������
//			if ((uchar)(t_onems - t_nd_fan) >= 60)
//			{
//				f_nd_fan = 1;//���ֵ�����װ壬ͨ���װ���Ʒ��
//			}
//		}
//		else
//		{
//			t_nd_fan = t_onems;
//			if(Protect_Low_Temp==0){//��ǰû�е��±���
//				f_nd_fan = 0;
//			}
//		}
//	}
//	else
//	{
//		f_door_switch = 1;//������
//		t_nd_fan = t_onems;
//		if(Protect_Low_Temp==0){//��ǰû�е��±���
//			f_nd_fan = 0;
//		}
//	}
//}
///***********���ű�������***************/ /////wys11.03.19
//void Door_Open(void)
//{
//	if (f_door_switch) //����
//	{
//		f_door_open = 1;
//		if ((unsigned char)(t_tens - t_door) >= 90) //15min
//		{
//			if (!f_dooropen_buzz)
//			{
//				f_dooropen_buzz = 1;
//				f_stop_alarm = 0;
//			}
//		}
//	}
//	else
//	{
//		f_door_open = 0;
//		f_dooropen_buzz = 0;
//		t_door = t_tens;
//	}
//}
/********************������˪����****************/
void Defrost_Prog(void)
{
	if (f_key_defrost)
	{
		f_defrost = 1;
	}
	else
	{
		f_defrost = 0;
	}
}
/***************���ͳ���*****************************/
/****************************************************/
/* @20181130 CFJ 
#if 0
void SendData(void)
{
  
  if(f_first_ad)       
  {
  	if(f_txbuzz)//������Ҫ�죬����׼����ʼ������һ֡
  	{
      f_txbuzz = 0;
      f_newframe = 0;
      f_senddata = 0;
      f_sendbit = 0;
      //C_send = 1; @20181008 CFJ
      f_gdp = 1;
      t_send = t_onems;
  	}
    else if(!f_newframe)//������һ֡���ȴ���һ֡����,�൱�ڳ�ʾ���˿�
    {
      if(f_gdp)
      {
        if((unsigned char)(t_onems - t_send) >= 2)
        {
          f_gdp = 0;
          //C_send = 0;
          t_send = t_onems;
        }    
      } 
      else
      {
        if((unsigned char)(t_onems - t_send) >= 2)
        {
          f_newframe = 1;
          f_gdp = 1;
          //C_send = 1;  @20181008 CFJ
          t_send = t_onems;
        }  
      }
    }
    else
    {
      if(f_senddata) 
      {
        if(f_sendbit)//���ڷ���һλ����
        {
          if(f_gdp)
          {
            if((unsigned char)(t_onems - t_send) >= 1)    //���͸ߵ�ƽ�ڼ�
            {
              f_gdp = 0;
             // C_send = 0;  @20181008 CFJ
              t_send = t_onems;
            }
          }
          else// ���͵͵�ƽ�ڼ�
          {
            if((!f_sbit && ((unsigned char)(t_onems - t_send) >= 2))
                || (f_sbit && ((unsigned char)(t_onems - t_send) >=6)))
            {
              f_sendbit = 0;
              f_gdp = 1;
              //C_send = 1; @20181008 CF
              t_send = t_onems; 
              r_bit--;
              if(r_bit <= 0)
              {
                r_byte++;
                if((!f_bp&&(r_byte>=C_byte))
                	 ||(f_bp&&(r_byte>=C_bp_byte)))
                {
                  f_senddata = 0;
                  f_newframe = 0;
                                                 //ˢ������
                  send[0] = 0xaa;
                  send[1] = flag_jqzt;
                  send[2] = 0;                
                  send[3] = 0;
                  send[4]= (unsigned char)(send[0]+send[1]+send[2]+send[3]);
                  if(f_bp_over)
                  {
                    f_bp_over = 0;
                    f_bp = 0;
                  }
                }
                else
                {
                  r_bit = C_bit;
                }
              }
            }
          }
        }
        else
        {
          if(send[r_byte] & 0x80)
          {
            f_sbit = 1;
          }
          else
          {
            f_sbit = 0;
          }
          send[r_byte] = send[r_byte] << 1;
          f_sendbit = 1;
          f_gdp = 1;
          //C_send = 1;  @20181008 CFJ
          t_send = t_onems;
        }
      }
      else
      {
        if(f_gdp)
        {
          if((unsigned char)(t_onems-t_send) >= 2)
          {
            f_gdp = 0;
            //C_send = 0; @20181008 CF
            t_send = t_onems;
          }
        }
        else if(!f_bp) 
        {
          if((unsigned char)(t_onems-t_send)>=10)
          {
                                                  //���յ���ʼ�ź�
            f_senddata = 1;
            f_gdp = 1;
            //C_send = 1; @20181008 CFJ
            t_send = t_onems;
            r_bit = C_bit;
            r_byte = 0;                           //Ҫ���͵��ֽ���
          }                    
        }
        else                                      //������֡Ϊ2ms�ߣ�20ms��
        {
          if((unsigned char)(t_onems-t_send)>=20)
          {
                                                  //���յ���ʼ�ź�
            f_senddata = 1;
            f_gdp = 1;
            //C_send = 1; @20181008 CFJ
            t_send = t_onems;
            send[0] = 1;
            r_bit = C_bit;
            r_byte = 0;                           //Ҫ���͵��ֽ���
            f_bp_over = 1;
          }                  
        }
      }
    }
  }
  else
  {
    t_send = t_onems;
  }    
}

void RecData(void)
{
  //if(!C_rec) @20181008 CF
  {
    f_ddp = 1;
  }
  //else @20181008 CF
  {
    if(f_ddp)
    {
      f_ddp = 0;
      if(f_startrec)                                        //1��ʼ������һ֡����
      {
        if((unsigned char)(t_onems - t_rec) >= 10)
        {
          r_rbit = C_rbit;
          r_rbyte = 0;
          t_rec = t_onems;
          f_recdata =1;
        } 
        else if(f_recdata)
        {
          if((unsigned char)(t_onems - t_rec) >=6)
          {
            rec[r_rbyte] = rec[r_rbyte] << 1;
            rec[r_rbyte] |= 0x01;
          }
          else if((unsigned char)(t_onems - t_rec) >=2)
          {
            rec[r_rbyte] = rec[r_rbyte] << 1;
            rec[r_rbyte] &= 0xfe;
          } 
          else
          {
            return;
          }
          t_rec = t_onems;
          r_rbit--;
          if(r_rbit <= 0)
          {
            r_rbyte++;
            if(r_rbyte >= C_rbyte)
            {
              f_startrec = 0;
              f_recdata = 0;
              if(rec[0] == 0x5a)
              {
               	if(rec[8]==(unsigned char)(rec[0]+rec[1]+rec[2]+rec[3]+rec[4]+rec[5]+rec[6]+rec[7]))
               	{
                  f_com_ok = 1;            		
                }
              }    
            }
            else
            {
              r_rbit = C_rbit;
            }
          } 
        }
      }
      else
      {
        f_startrec = 1;
        t_rec = t_onems;
      }
    }
  }
}
#endif
*/
/****************************************************/
/***************�����趨ֵ����***********************/
/****************************************************/
void WriteToE2(void)
{
	if (f_need_write_e2)
	{
		if ((uchar)(t_halfsec - t_write_e2) >= 4) //2s
		{
			f_need_write_e2 = 0;
			f_e2prom = 1;
			t_WriteE2=t_onems;
			WriteE2_num=0;
		}
	}
}

/****************************************************/
/**********II2 BUS***********************************/
/****************************************************/
/*need define in your .h file:f_e2prom,C_sda,C_scl,
  C_sdaddr,in,out*/

unsigned char ReadByte(void);
/*****************************************************/
void WriteE2(void)
{
	if (f_e2prom)
	{
		if((unsigned char)(t_onems-t_WriteE2)>=10){           //10ms
			
			WriteE2_num++;
			if(WriteE2_num == 1){
				Begin();
				WriteByte(0xA0); ///write chip addres 0xa0      ÿҳ64���ֽڣ���������д
				WriteByte(0x00); ///write data high addres

				WriteByte(r_ldzt);			///write data //byte1
				WriteByte(t_yj_delay);		//byte2
				WriteByte(r_ldwdjz);		//byte3
				WriteByte(r_ld_high_alarm); //byte4
				WriteByte(r_ld_low_alarm);  //byte5
				WriteByte(r_lczt);			//byte6
				WriteByte(r_lc_high_alarm); //byte7
				WriteByte(r_lc_low_alarm);  //byte8

				WriteByte(flag_ajszyjkt & 0x03); //write data  //byte9
				WriteByte(r_lcwdjz);			 //byte10
				WriteByte(u8_ld_bjycsj);		 //byte11
				WriteByte(u8_lc_bjycsj);		 //byte12
				WriteByte(0xAB);				 //byte13
				//���ѹ��
				WriteByte(lc_compressor_times>>24); //byte14
				WriteByte(lc_compressor_times>>16); //byte15
				WriteByte(lc_compressor_times>>8);//byte16
				C_SDA_SET_HIGH;
				Stop();
				t_WriteE2=t_onems;
			}else if(WriteE2_num == 2){
				//�ڶ�ҳ
				Begin();
				WriteByte(0xA0); ///write chip addres 0xa0      ÿҳ64���ֽڣ���������д
				WriteByte(0x10); ///write data high addres
				WriteByte(lc_compressor_times);//byte17
				//�������
				WriteByte(lc_fan_times>>24); //byte18
				WriteByte(lc_fan_times>>16); //byte19
				WriteByte(lc_fan_times>>8);//byte20
				WriteByte(lc_fan_times);//byte21
				//�������
				WriteByte(lc_nd_fan_times>>24); //byte22
				WriteByte(lc_nd_fan_times>>16); //byte23
				WriteByte(lc_nd_fan_times>>8);//byte24
				WriteByte(lc_nd_fan_times);//byte25
				WriteByte(r_lcxswdjz);//�����ʾ�¶Ⱦ;���ֵ byte26
				C_SDA_SET_HIGH;
				Stop();
				f_e2prom = 0;
			}	
		}
	}
}
//^^^^^^^^^^^^^^^^^^^^^^^^^^
void ReadE2(void)
{
	unsigned char temp;
	Begin();
	WriteByte(0xA0); ///write chip addres 0xa0
	WriteByte(0x00); ///write data addres
	Begin();
	WriteByte(0xA1); ///write chip addres 0xa0

	r_ldzt = ReadByte(); //byte1�䶳�趨
	Sendscl();

	t_yj_delay = ReadByte(); //byte2ѹ���ӳ�ʱ��
	t_yj_delayx = t_yj_delay;
	Sendscl();

	r_ldwdjz = ReadByte(); //byte3�䶳У׼ֵ
	r_ldwdjzx = r_ldwdjz;
	Sendscl();

	r_ld_high_alarm = ReadByte(); //byte4�䶳���±����趨�Ĳ�ֵ
	Sendscl();

	r_ld_low_alarm = ReadByte(); //byte5
	Sendscl();

	r_lczt = ReadByte(); //byte6//����趨�¶�
	r_lczt_float = r_lczt;
	Sendscl();

	r_lc_high_alarm = ReadByte(); //byte7
	Sendscl();

	r_lc_low_alarm = ReadByte(); //byte8 //��ص��±����趨�Ĳ�ֵ
	Sendscl();

	flag_ajszyjkt = ReadByte() & 0x03; //byte9��������ѹ����ͣ
	Sendscl();

	r_lcwdjz = ReadByte(); //byte10����¶�У׼ֵ
	r_lcwdjzx = r_lcwdjz;
	Sendscl();

	u8_ld_bjycsj = ReadByte(); //byte11�䶳�����ӳ�ʱ��
	u8_ld_bjycsjx = u8_ld_bjycsj;
	Sendscl();

	u8_lc_bjycsj = ReadByte(); //byte12
	u8_lc_bjycsjx = u8_lc_bjycsj;
	Sendscl();

//	temp = ReadByte(); //byte13
//	C_SDA_SET_HIGH;
//	Sendscl();
	temp = ReadByte(); //byte13
	Sendscl();
	//���ѹ��
	lc_compressor_times=0;
	lc_compressor_times |= (ReadByte()<<24); //byte14
	Sendscl();
	lc_compressor_times |= (ReadByte()<<16); //byte15
	Sendscl();
	lc_compressor_times |= (ReadByte()<<8); //byte16
	Sendscl();
	//�ڶ�ҳ
	lc_compressor_times |= ReadByte(); //byte17
	Sendscl();
	if(lc_compressor_times==0xffffffff)lc_compressor_times=0;
	//�������
	lc_fan_times=0;
	lc_fan_times |= (ReadByte()<<24); //byte18
	Sendscl();
	lc_fan_times |= (ReadByte()<<16); //byte19
	Sendscl();
	lc_fan_times |= (ReadByte()<<8); //byte20
	Sendscl();
	lc_fan_times |= ReadByte(); //byte21
	Sendscl();
	if(lc_fan_times==0xffffffff)lc_fan_times=0;
	//�������
	lc_nd_fan_times=0;
	lc_nd_fan_times |= (ReadByte()<<24); //byte22
	Sendscl();
	lc_nd_fan_times |= (ReadByte()<<16); //byte23
	Sendscl();
	lc_nd_fan_times |= (ReadByte()<<8); //byte24
	Sendscl();
	lc_nd_fan_times |= ReadByte(); //byte25
	if(lc_nd_fan_times==0xffffffff)lc_nd_fan_times=0;
	Sendscl();
	r_lcxswdjz=ReadByte();//byte26
	r_lcxswdjzx=r_lcxswdjz;
	Sendscl();
	C_SDA_SET_HIGH;
	Sendscl();
	Stop();

	r_ldzt_report = r_ldzt;
	r_lczt_report = r_lczt;
	ld_high_alarm_report = r_ld_high_alarm;
	ld_low_alarm_report = r_ld_low_alarm;
	lc_high_alarm_report = r_lc_high_alarm;
	lc_low_alarm_report = r_lc_low_alarm;
	if (temp != 0xAB)
	{
		r_ldzt = 60; //ʪ��Ĭ��60
		r_lczt = 43;  //5��C 43-38 ��ʼ�趨ֵ��5��
		r_lczt_float = r_lczt;
		t_yj_delayx = 1;
		t_yj_delay = 1;
		r_ldwdjzx = 10;
		r_ldwdjz = 20;//ʪ�Ⱦ�ƫ��λ��+-15
		r_ld_high_alarm = 5;
		r_ld_low_alarm = 5;
		r_lc_high_alarm = 3;
		r_lc_low_alarm = 3;
		r_lcwdjzx = 110;//���ƴ������¶�ƫ�����ϵ�2��
		r_lcwdjz = 110;//���ƴ������¶�ƫ�����ϵ�2��
		r_lcxswdjz=110;//��ʾ�������¶�ƫ�����ϵ�3��
		r_lcxswdjzx=110;//���ƴ������¶�ƫ�����ϵ�3��
		u8_ld_bjycsjx = 2; //�䶳�����ӳ�ʱ��,Сʱ
		u8_ld_bjycsj = 2;
		u8_lc_bjycsjx = 2;
		u8_lc_bjycsj = 2;
		flag_ajszyjkt = 0x03; //��������ѹ����ͣ��
		f_e2prom = 1;

		r_ldzt_report = 160;
		r_lczt_report = 43;
		ld_high_alarm_report = 5;
		ld_low_alarm_report = 5;
		lc_high_alarm_report = 3;
		lc_low_alarm_report = 3;
		lc_compressor_times=0;//���ѹ������ʱ��
		lc_fan_times=0;//�����������ʱ��
		lc_nd_fan_times=0;//�����������ʱ��
	}
}
//^^^^^^^^^^^^^^^^^
void WriteByte(unsigned char tem_dat)
{
	unsigned char w_clk;

	C_SDA_OUT; //1

	for (w_clk = 8; w_clk != 0; w_clk--)
	{
		DelayControl(4);
		if (tem_dat & 0x80)
			C_SDA_SET_HIGH;
		else
			C_SDA_SET_LOW;
		tem_dat = tem_dat << 1;
		DelayControl(4);
		Sendscl(); //ack signal
	}
	C_SDA_SET_HIGH;
	C_SDA_IN;
	w_clk = 0xFF;
	Sendscl();		//ack signal
	while (!C_SDA_STATUS) //while(!C_sda)
	{
		w_clk--;
		DelayControl(4);
		if (w_clk == 0)
			break;
	}
	C_SDA_OUT;
}
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
unsigned char ReadByte(void)
{
	unsigned char r_dat = 0, r_clk = 0;
	C_SDA_SET_HIGH;
	C_SDA_IN;

	for (r_clk = 8; r_clk > 0; r_clk--)
	{
		C_SCL_SET_HIGH;
		DelayControl(4);
		r_dat = r_dat << 1;

		if (C_SDA_STATUS) //if(C_sda)
			r_dat |= 0x01;
		else
			r_dat &= 0xfe;

		C_SCL_SET_LOW;
		DelayControl(4);
	}
	C_SDA_OUT;
	DelayControl(4);
	C_SDA_SET_LOW;

	return (r_dat);
}
/*****************************/
void Begin(void)
{
	C_SDA_OUT;
	C_SDA_SET_HIGH;
	DelayControl(4);
	C_SCL_SET_HIGH;
	DelayControl(4);
	C_SDA_SET_LOW;
	DelayControl(4);
	C_SCL_SET_LOW;
	DelayControl(4);
}
//--------------------------
void Stop(void)
{
	C_SDA_SET_LOW;
	DelayControl(4);
	C_SCL_SET_HIGH;
	DelayControl(4);
	C_SDA_SET_HIGH;
	DelayControl(4);
}
//---------------
void Sendscl(void)
{
	C_SCL_SET_HIGH;
	DelayControl(4);
	C_SCL_SET_LOW;
	DelayControl(4);
}
/*********************Timer Interrupt************/
/************************************************/
/* @20181130 CFJ  
void  isrVtpm2ovf(void) 
{
  //////(void)(TPM2SC==0);
 // //////TPM2SC_TOF =0; 
  u16_random++;
  if ( u16_random > 16383)
  u16_random = 1;
  t_onems++;                                     //1ms
  R_NetTimeBase1ms++;
  SendData();
  RecData(); //�������������� cfj
	t_1ms++;
	t_j20ms++;
	R_HomeNetResponseTimeSecond++;
	if(t_j20ms>=20)
  {
  	t_j20ms = 0;
  	t_20ms++;
  }
	if(t_1ms>=2)                                   //2ms
	{
	  t_2ms++;
	  t_twoms++;
	  t_1ms = 0;
	  if(t_2ms>=250)
  	{
	    t_halfsec++;
	    t_2ms = 0;
		  f_05s = 1;                                 //0.5s
	  }
	}
	if (R_NetTimeBase1ms >= 100)
	{
		R_NetTimeBase1ms = 0;
		R_HomeNetResponseCheckTime100ms++;
	}
}
*/
/************************************************/
/****************SCITransmit Interrupt***********/
/************************************************/
/*
void  SCITransmit(void) 
{
	uchar a;
	/////a = SCI1S1;                                    //�巢���ж�����λ�����ȶ�SCI1S1Ȼ��дSCI1D
	if(f_send_data)
	{
		if(f_send55)
		{
			f_send55 = 0;
			/////SCI1D = 0x55;
			if(r_sendr>=r_sendsum)
	    {
	  	  //////SCI1C2_TCIE = 0;                           //�رշ����ж�
	  	  f_send_1ff = 0;
	  	  f_send_2ff = 0;
	  	  f_send_data = 0;
	    }
		}
		else if(send_net[r_sendr]==0xff)
		{
			f_send55 = 1;
			//////SCI1D = 0xff;
			r_sendr++;
		}
		else
		{
			//////SCI1D = send_net[r_sendr];
	    r_sendr++;
	    if(r_sendr>=r_sendsum)
	    {
	  	  /////SCI1C2_TCIE = 0;                           //�رշ���ģ��
	  	  f_send55 = 0;
	  	  f_send_1ff = 0;
	  	  f_send_2ff = 0;
	  	  f_send_data = 0;
	    }
		}
	}
  else if(f_send_2ff)
  {
  	f_send_data = 1;
  	r_sendr = 0;
  	////SCI1D = r_sendsum;
  }
  else if(f_send_1ff)
  {
  	f_send_2ff = 1;
  	////SCI1D = 0xff;
  }
  else
  {
  	f_send_1ff = 1;
  	//////SCI1D = 0xff;
  }	
}  
*/
uchar u8_Rec2data[20];
uchar u8_Send2data[52];
uchar u8_Send2dataMemory[52];

uchar u8_Rec2_data_Num;
uchar u8_Send2_data_Num;
uchar u8_Send2_data_Num_Count;
uchar u8_Rec2_data_State;
uchar u8_Send2_data_State;



unsigned int lc_compressor_timesa,lc_nd_fan_timesa,lc_fan_timesa;
//������ѹ��������������������ʱ��ͳ��
void MachineTimeStatistics(void)
{
	if(f_power_off==1){//�ϵ�Ͳ�ͳ����
	}else{
		if(f_lc_compressor==ON){//ѹ��
			if(f_onemin){
				lc_compressor_timesa++;
			}
		}
		if(f_nd_fan==ON){//�������/�ڷ��
			if(f_onemin){
				lc_nd_fan_timesa++;
			}
		}
		if(f_lc_fan==ON){//�������
			if(f_onemin){
				lc_fan_timesa++;
			}
		}

		if((lc_compressor_timesa>=600) || (lc_nd_fan_timesa>=600) || (lc_fan_timesa>=600 )){//��һ������600���ӣ���дһ��eeprom
			
			lc_compressor_times+=lc_compressor_timesa;//ѹ��ʱ���ۼ�
			lc_nd_fan_times+=lc_nd_fan_timesa;//�������ʱ���ۼ�
			lc_fan_times+=lc_fan_timesa;//��������ۼ�
			lc_compressor_timesa=0;//����ͳ��ѹ��ʱ��
			lc_nd_fan_timesa=0;//����ͳ���������ʱ��
			lc_fan_timesa=0;//����ͳ���������ʱ��
			f_need_write_e2=ON;
		}
	}
}

void GetSum2(void)
{
//	uchar r_checksum = 0, count;

//	u8_Send2dataMemory[0] = 0x55;
//	u8_Send2dataMemory[1] = 0xFF;
//	u8_Send2dataMemory[2] = u8_Send2_data_Num;

//	r_checksum = u8_Send2_data_Num;

//	for (count = 0; count < u8_Send2_data_Num - 1; count++)
//	{
//		r_checksum = r_checksum + u8_Send2data[count];
//		u8_Send2dataMemory[count + 3] = u8_Send2data[count];
//	}

//	u8_Send2dataMemory[count + 3] = r_checksum;
//	u8_Send2_data_Num = u8_Send2_data_Num + 3;
	uchar r_checksum = 0, count;
	for (count = 1; count < 50 ; count++)
	{
		r_checksum += u8_Send2data[count];
	}
	u8_Send2data[50]=r_checksum;
	for(count=0;count<52;count++){
		u8_Send2dataMemory[count]=u8_Send2data[count];
	}
	u8_Send2_data_Num = 52;//Ҫ���͵����ݳ���
	u8_Send2_data_Num_Count = 0;
	u8_Send2_data_State = 1;
	
	SendData(u8_Send2dataMemory, u8_Send2_data_Num); 
}

//void GetSum2(void)
//{
//	uchar r_checksum = 0, count;
//	for (count = 1; count < 50 ; count++)
//	{
//		r_checksum += u8_Send2data[count];
//	}
//	u8_Send2data[50]=r_checksum;
//	for(count=0;count<52;count++){
//		u8_Send2dataMemory[count]=u8_Send2data[count];
//	}
//	u8_Send2_data_Num = 52;//Ҫ���͵����ݳ���
//	u8_Send2_data_Num_Count = 0;
//	u8_Send2_data_State = 1;
//	SendData(u8_Send2dataMemory, u8_Send2_data_Num); 
//										   /////////// SCI2C2_TCIE = 1;                                 //ʹ�ܷ���ģ��
//}

void ReturnPrintFrame(void)
{

	if (u8_Send2_data_State == 1)
	{//��һ֡���ݻ�û�������
		return;
	}

	u8_Send2data[0] = 0x69;
	u8_Send2data[1] = 0x00;//�����ϱ�;
	u8_Send2data[2] = 47;//���ݲ��ֳ���
	u8_Send2data[3]=2;//1:HYC-406,2:HYC-116
	if (r_lcxswd_float >= 38.0){//�¶�Ϊ��  �ֱ���Ϊ0.1
		//��ʾ�¶ȵ�8λ
		u8_Send2data[4]=(u16t)((r_lcxswd_float - 38.0) * 10 + 0.5);
		//��ʾ�¶ȸ�8λ
		u8_Send2data[5] =((u16t)((r_lcxswd_float - 38.0) * 10 + 0.5))>>8;

	}else if (r_lcxswd_float <= 28.0){//�¶�<= -10��  //ֻ��ʾ��������
		//��ʾ�¶ȵ�8λ
		u8_Send2data[4]=(-(u16t)((38 - r_lcxswd_float)*10))&0xff;
		//��ʾ�¶ȸ�8λ
		u8_Send2data[5] =(-(u16t)((38 - r_lcxswd_float)*10))>>8;

	}else{//   0  ~ -9.9��   ��ʾ1λС��
		u8_Send2data[4]=(-(u16t)((38 - r_lcxswd_float)*10+0.5))&0xff;
		u8_Send2data[5]=(-(u16t)((38 - r_lcxswd_float)*10+0.5))>>8;
	}
//	f_battery = 1; //��ع��ϱ�־
//	f_power_off = 1;//�ϵ籨��
//	f_lc_sensor_err = 1;//��ؿ��ƴ���������
//	f_lcXs_sensor_err = 1;//�����ʾ���������ϼ���HYC406�ϴ���������
//	f_lcHs_sensor_err=1;//��˪����������
//	f_lcLn_sensor_err = 1;//��������������
//	f_hw_high38 = 1;//���³�38�ȱ���
//	f_hw_sensor_err=1;//���´���������
//	g_Sys_Erflag0_Comm = 1; //ͨѶ���� @20181025 CFJ
//	f_lc_high = 1;//�����ʾ���������±���
//	f_lc_low = 1;//�����ʾ���������±���
//	Protect_HigLn_Temp = 1;//�������и���

	//����λ u8_Send2data[5]-u8_Send2data[7]
	u8_Send2data[10] = 0x00; 
	if(f_lc_high==1)u8_Send2data[10] |=1;
	if(f_lc_low==1)u8_Send2data[10] |=1<<1;
	if(f_power_off==1)u8_Send2data[10] |=1<<2;
	if(g_Sys_Erflag0_Comm==1)u8_Send2data[10] |=1<<3;
	if(f_lc_sensor_err==1)u8_Send2data[10] |=1<<4;
	if(f_battery==1)u8_Send2data[10] |=1<<5;
	//if(f_lcXs_sensor_err==1)u8_Send2data[10] |=1<<7; 20210531�ĳ���byte[12]��bit1λ
	//_Send2data[5] |= (f_lc_high|(f_lc_low<<1)|(f_power_off<<2)|(g_Sys_Erflag0_Comm<<3)|(f_lc_sensor_err<<4)|(f_battery<<5)|(f_lcXs_sensor_err<<7));

	
	u8_Send2data[11] = 0x00;
	if(f_hw_sensor_err==1)u8_Send2data[11] |=1<<1;
	if(f_lcHs_sensor_err==1)u8_Send2data[11] |=1<<2;
	if(f_hw_high38==1)u8_Send2data[11] |=1<<3;
	if(f_lcLn_sensor_err==1)u8_Send2data[11] |=1<<5;
	//u8_Send2data[6] |=(((f_hw_sensor_err&1)<<1)|((f_lcHs_sensor_err&1)<<2)|((f_hw_high38&1)<<3)|((f_lcLn_sensor_err&1)<<5));
		
	u8_Send2data[12] = 0x00;
	if(Protect_HigLn_Temp==1)u8_Send2data[12]|=1;
	if(f_lcXs_sensor_err==1)u8_Send2data[12] |=1<<1;//
	//u8_Send2data[7]|=Protect_HigLn_Temp;

	//Ԥ��
	u8_Send2data[13] = 0x00;
	if(f_ld_high==1) u8_Send2data[13]|=1<<6;//ʪ�ȸ�
	if(f_ld_low==1) u8_Send2data[13]|=1<<5;//ʪ�ȵ�
	//��ؿ�����
	u8_Send2data[14] = 0x00;
	//�䶳������
	u8_Send2data[15] = 0x00;
	//ʪ��
	u8_Send2data[16] = r_ldgzwd;
	if(f_power_off==1){//�ϵ� �ϵ�����ϱ�ȫ��
		//��������״̬--0����˪״̬1
		u8_Send2data[17] = 0;
		//ѹ��״̬
		u8_Send2data[18] = 0;
		//�������
		u8_Send2data[19] = 0;
		//�������
		u8_Send2data[20] = 0;
	}else{
		//��������״̬--0����˪״̬1
		if(LcHuaShuang_Flag==1){//��˪
			u8_Send2data[17] = 1;
		}else{
			u8_Send2data[17] = 0;
		}
		//ѹ��״̬
		if(f_lc_compressor == ON){//����
			u8_Send2data[18] = 1;
		}else{
			u8_Send2data[18] = 0;
		}
		//�������
		if(f_lc_fan == ON){//����
			u8_Send2data[19] = 1;
		}else{
			u8_Send2data[19] = 0;
		}
		//�������
		if(f_nd_fan == ON){//����
			u8_Send2data[20] = 1;
		}else{
			u8_Send2data[20] = 0;
		}
	}
	//���ѹ���ۼƹ���ʱ��
	u8_Send2data[21] = (lc_compressor_times+lc_compressor_timesa);
	u8_Send2data[22] = (lc_compressor_times+lc_compressor_timesa)>>8;
	u8_Send2data[23] = (lc_compressor_times+lc_compressor_timesa)>>16;
	u8_Send2data[24] = (lc_compressor_times+lc_compressor_timesa)>>24;
	//�����������ۼƹ���ʱ��
	u8_Send2data[25] = (lc_fan_times+lc_fan_timesa);
	u8_Send2data[26] = (lc_fan_times+lc_fan_timesa)>>8;
	u8_Send2data[27] = (lc_fan_times+lc_fan_timesa)>>16;
	u8_Send2data[28] = (lc_fan_times+lc_fan_timesa)>>24;
	//��������ۼƹ���ʱ��
	u8_Send2data[29] = (lc_nd_fan_times+lc_nd_fan_timesa);
	u8_Send2data[30] = (lc_nd_fan_times+lc_nd_fan_timesa)>>8;
	u8_Send2data[31] = (lc_nd_fan_times+lc_nd_fan_timesa)>>16;
	u8_Send2data[32] = (lc_nd_fan_times+lc_nd_fan_timesa)>>24;
	//Ԥ��
	u8_Send2data[33] = 0;
	u8_Send2data[34] = 0;
	//�䶳ѹ��״̬
	u8_Send2data[35] = 0;
	//�䶳�������װ
	u8_Send2data[36] = 0;
	//�䶳�������״̬
	u8_Send2data[37] = 0;
	//�䶳ѹ���ۼƹ���ʱ��
	u8_Send2data[38] = 0;
	u8_Send2data[39] = 0;
	u8_Send2data[40] = 0;
	u8_Send2data[41] = 0;
	//�䶳��������ۼƹ���ʱ��
	u8_Send2data[42] = 0;
	u8_Send2data[43] = 0;
	u8_Send2data[44] = 0;
	u8_Send2data[45] = 0;
	//�䶳��������ۼƹ���ʱ��
	u8_Send2data[46] = 0;
	u8_Send2data[47] = 0;
	u8_Send2data[48] = 0;
	u8_Send2data[49] = 0;
	//�ۼӺ�
	u8_Send2data[50] = 0;
	//֡β
	u8_Send2data[51] = 0x19;
	GetSum2();
}

void Rec2Action(void)
{
	if (u8_Send_Print_Time >= 2)
	{//ÿһ���Ӹ�һ�ѣ�ԭ�����ڴ�ӡ���ϣ�������������ʾ��ͨ��
		ReturnPrintFrame();
		u8_Send_Print_Time = 0;
	}
}
/*
void  SCI2Transmit(void)
{
	uchar Sendata ;

	///////Sendata = SCI2S1;  

   //// SCI2D = u8_Send2dataMemory[u8_Send2_data_Num_Count];
	if(++u8_Send2_data_Num_Count>=u8_Send2_data_Num)
	{
	  ///SCI2C2_TCIE = 0; 
	  u8_Send2_data_State=0;
	
	
	}

}

void  SCI2Receive(void) 
{

		uchar Recdata ;

		/////Recdata = SCI2S1;
		
		///////u8_Rec2data[u8_Rec2_data_Num++]=SCI2D;
		if(u8_Rec2_data_Num>=20)
		{
		u8_Rec2_data_State=1;
		u8_Rec2_data_Num=0;
		
		
		}

}
*/
/************************************************/
/*****************SCIReceive Interrupt***********/
/************************************************/
/*@20181130 CFJ
void SCIReceive(void) 
{
	uchar a;                                       //������ж�����λ�����ȶ�SCI1S1Ȼ���SCI1D
	/////a = SCI1S1;
	if(f_rec_over)                                 //��������ţ���� f_rec_overΪ1���ٽ���Ϊ��Ч����                    
	{		                                                                
	 //// a = SCI1D;		
	}
	else if(f_rec_data)
	{
		if(f_rec55)
		{
			f_rec55 = 0;
		//////	a = SCI1D;
			if(a==0x55)
			{
				r_rec55sum = r_rec55sum+0x55;
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
		 //// rec_net[r_receiver] = SCI1D;
		  if(rec_net[r_receiver]==0xff)
		  {
		  	f_rec55 = 1;
		  }
  	  r_receiver++;
  	  if(r_receiver>=r_recsum)
  	  {
  		  f_rec_over = 1;
  		  f_rec_1ff = 0;
  		  f_rec_2ff = 0;
  		  f_rec_data = 0;
  		  f_rec55 = 0;
  	  }
  	}
	}
  else if(f_rec_2ff)
  {
  	/////r_recsum = SCI1D;
  	if(r_recsum<=20)
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
  else if(f_rec_1ff)
  {
  	//////if(SCI1D==0xff)
  	////{
  	///	f_rec_2ff = 1;
  	///	r_receiver = 0;
  	/////	t_net_rec = t_onems;
  	///}
  	/////else
  	////{
  		///f_rec_1ff = 0;
  	////}
  }
  else
  {
  	///if(SCI1D==0xff)
  	///{
  		////f_rec_1ff = 1;
  		///t_net_rec = t_onems;
  	////}
  }
}
*/
/************************************************/
/*****************������ɴ������***************/
/************************************************/
void NetRecOver(void)
{
	uchar r_cheksum = 0;
	if (f_rec_over)
	{
		if(r_recsum != 0)
		{
			for (r_receiver = 0; r_receiver < (unsigned char)(r_recsum - 1); r_receiver++)
			{
				r_cheksum = r_cheksum + rec_net[r_receiver];
			}
			r_cheksum = r_cheksum + r_recsum + r_rec55sum;
			r_rec55sum = 0;
			if (r_cheksum == rec_net[r_receiver])
			{
				RecOkAction(); //���������Ч����
				f_rec_over = 0;
			}
			else
			{
				f_rec_over = 0;
			}
		}
		
	}
}

/************************************************/
/*****************UART0������ɴ������***************/
/************************************************/
void NetRecOver1(void)
{
	uchar r_cheksum = 0;
	if (f_rec_over1)
	{
		for (r_receiver1 = 0; r_receiver1 < (unsigned char)(r_recsum1+2); r_receiver1++)
		{
			r_cheksum = r_cheksum + rec_net1_ok[r_receiver1];
		}
		if (r_cheksum == rec_net1_ok[r_receiver1])
		{
			RecOkAction1(); //���������Ч����
			f_rec_over1 = 0;
		}
		else
		{
			f_rec_over1 = 0;
		}
	}
}
/************************************************/
/*****************UART0���������Ч����***************/
/************************************************/
void RecOkAction1(void)
{
	if(rec_net1_ok[0]==0x01){//�������
		f_stop_alarm = ON;
		t_stop_alarm = 0;
	}else if(rec_net1_ok[0]==0x02){//�ۼƹ���ʱ������
		if(rec_net1_ok[2]==0x02){
			if(rec_net1_ok[3]==1){//1�����������ѹ������ʱ��
				lc_compressor_times=0;
				lc_compressor_timesa=0;
			}
			if(rec_net1_ok[4]==1){//1����������������������ʱ��
				lc_fan_times=0;
				lc_fan_timesa=0;
			}
			if(rec_net1_ok[5]==1){//1����������������������ʱ��
				lc_nd_fan_times=0;
				lc_nd_fan_timesa=0;
			}
		}
		
		f_need_write_e2=ON;
	}
}

/************************************************/
/*****************���������Ч����***************/
/************************************************/
void RecOkAction(void)
{
	if (rec_net[6] == 1)
	{
		if (rec_net[7] == 0x4d)
		{
			if (rec_net[8] == 0x01) //4d 01 ��ѯ����
			{
				ReturnStateFrame();
			}
			else
			{
				ReturnVoidFrame();
			}
		}
		else if (rec_net[7] == 0x5d)
		{
			if (rec_net[8] == 0x01) //5d 01 �������
			{
				if (rec_net[10] >= 38 && rec_net[10] <= 54)
				{
					r_lczt = rec_net[10];
					r_lczt_float = r_lczt;
					if (r_lczt < 43)
					{
						r_lc_low_alarm = r_lczt - 38;
					}
					else
					{

						if (r_lc_low_alarm < 1) //20180312
						{
							r_lc_low_alarm = 1;
						}
					}
					f_need_write_e2 = ON;
					t_write_e2 = t_halfsec;
					BuzzBi();
					ReturnStateFrame();
				}
				else
				{
					ReturnVoidFrame();
				}
			}
			else if (rec_net[8] == 0x02) //5d 02 ������ظ���
			{
				if (rec_net[10] <= 50 && rec_net[10] >= r_lczt + 1)
				{
					r_lc_high_alarm = rec_net[10] - r_lczt;
					f_need_write_e2 = ON;
					t_write_e2 = t_halfsec;
					BuzzBi();
					ReturnStateFrame();
				}
				else
				{
					ReturnVoidFrame();
				}
			}
			else if (rec_net[8] == 0x03) //5d 03 ������ص���
			{
				if (rec_net[10] >= 38 && rec_net[10] <= r_lczt)
				{
					if (rec_net[10] <= r_lczt - 1)
					{
						r_lc_low_alarm = r_lczt - rec_net[10];
					}
					else
					{
						r_lc_low_alarm = r_lczt - 38;
					}
					f_need_write_e2 = ON;
					t_write_e2 = t_halfsec;
					BuzzBi();
					ReturnStateFrame();
				}
				else
				{
					ReturnVoidFrame();
				}
			}
			else if (rec_net[8] == 0x04) //5d 04 �����䶳
			{
				if (rec_net[10] >= 155 && rec_net[10] <= 190)
				{
					r_ldzt = rec_net[10];
					f_need_write_e2 = ON;
					t_write_e2 = t_halfsec;
					BuzzBi();
					ReturnStateFrame();
				}
				else
				{
					ReturnVoidFrame();
				}
			}
			else if (rec_net[8] == 0x05) //5d 05 �����䶳����
			{
				if (rec_net[10] <= 250 && rec_net[10] >= r_ldzt + 1)
				{
					r_ld_high_alarm = rec_net[10] - r_ldzt;
					f_need_write_e2 = ON;
					t_write_e2 = t_halfsec;
					BuzzBi();
					ReturnStateFrame();
				}
				else
				{
					ReturnVoidFrame();
				}
			}
			else if (rec_net[8] == 0x06) //5d 06 �����䶳������
			{
				if (rec_net[10] >= 11 && rec_net[10] <= r_ldzt - 1)
				{
					r_ld_low_alarm = r_ldzt - rec_net[10];
					#ifdef Need_LD
						if ((r_ldzt - r_ld_low_alarm) >= 200)
						{
							f_zf = ZHENG_SIGN;
							r_swtj = (uchar)((r_ldzt - r_ld_low_alarm - 200) / 10);
							r_gwtj = (uchar)((r_ldzt - r_ld_low_alarm - 200) % 10);
						}
						else
						{
							f_zf = FU_SIGN;
							r_swtj = (uchar)((200 - (r_ldzt - r_ld_low_alarm)) / 10);
							r_gwtj = (uchar)((200 - (r_ldzt - r_ld_low_alarm)) % 10);
						}
					#else
						f_zf = ZHENG_SIGN;
						r_swtj = (uchar)((r_ldzt - r_ld_low_alarm ) / 10);
						r_gwtj = (uchar)((r_ldzt - r_ld_low_alarm ) % 10);
					#endif
					f_need_write_e2 = ON;
					t_write_e2 = t_halfsec;
					BuzzBi();
					ReturnStateFrame();
				}
				else
				{
					ReturnVoidFrame();
				}
			}
			else if (rec_net[8] == 0x07) //5d 07 ��ؿ��Ƶ�λУ׼
			{
				if (rec_net[10] <= 15 && rec_net[10] >= 5)
				{
					r_lcwdjz = rec_net[10];
					r_lcwdjzx = r_lcwdjz;
					f_need_write_e2 = ON;
					t_write_e2 = t_halfsec;
					BuzzBi();
					ReturnStateFrame();
				}
				else
				{
					ReturnVoidFrame();
				}
			}
			else if (rec_net[8] == 0x08) //5d 08 �䶳��λУ׼
			{
				if (rec_net[10] <= 35 && rec_net[10] >= 5)
				{
					r_ldwdjz = rec_net[10];
					r_ldwdjzx = r_ldwdjz;
					f_need_write_e2 = ON;
					t_write_e2 = t_halfsec;
					BuzzBi();
					ReturnStateFrame();
				}
				else
				{
					ReturnVoidFrame();
				}
			}
			else if (rec_net[8] == 0x0a)
			{
				if (((rec_net[9] <= 30 && rec_net[9] >= 10) && (rec_net[10] <= 12 && rec_net[10] >= 1) &&
					 (rec_net[11] <= 31 && rec_net[11] >= 1) && (rec_net[12] <= 23) && (rec_net[13] <= 59)) &&
					(r_set_state < 32))
				{
					u8_year = rec_net[9];
					u8_month = rec_net[10];
					u8_day = rec_net[11];
					u8_hour = rec_net[12];
					u8_minute = rec_net[13];
					u8_year_copy = u8_year;
					u8_month_copy = u8_month;
					u8_day_copy = u8_day;
					u8_hour_copy = u8_hour;
					u8_minute_copy = u8_minute;
					ReturnAckFrame();
				}
				else
				{
					if ((!f_net_first_set_time) && ((rec_net[9] > 30 || rec_net[9] < 10) || (rec_net[10] > 12 || rec_net[10] < 1) ||
													(rec_net[11] > 31 || rec_net[11] < 1) || (rec_net[12] > 23) || (rec_net[13] > 59)))
					{
						u8_year = 10;
						u8_month = 1;
						u8_day = 1;
						u8_hour = 1;
						u8_minute = 1;
						u8_year_copy = u8_year;
						u8_month_copy = u8_month;
						u8_day_copy = u8_day;
						u8_hour_copy = u8_hour;
						u8_minute_copy = u8_minute;
					}
					ReturnVoidFrame();
				}
				if ((rec_net[15] & 0xc0) == 0xc0) //zyj  100621
				{
					/*if (!f_Usb_Copy_Start)
		               	 {
		               	 	 f_Usb_Copy_Start = 1;
		               	   //r1 = r_set_state;
		               	 }*/
					//r_set_state = USB_Insert_Or_PullOut;
					u8_UsbState = 1;
				}
				else if ((rec_net[15] & 0xc0) == 0x80)
				{
					//r_set_state = USB_Insert_Or_PullOut;
					//f_Usb_Copy_Done = 1;
					u8_UsbState = 2;
				}
				else if ((rec_net[15] & 0xc0) == 0)
				{
					u8_UsbState = 0;
					//f_Usb_Copy_Done = 1;
					/*if (f_Usb_Copy_Start)
			               	 {
			               	 	 f_Usb_Copy_Start = 0;
			               	 }*/
				}
				f_net_first_set_time = 1;
			}
			else
			{
				ReturnVoidFrame();
			}
		}
		else
		{
			ReturnVoidFrame();
		}
	}
	else if (rec_net[6] == 0x05)
	{
		if(Cnt_SetTime_Sended >= 3)  //����ʱ��������3��֮�󣬲������˱�־��λ
			ACKok = 1;																											 //lxx
		if ((rec_net[0] == (uchar)((u16_random_errcopy >> 8) & 0x3F)) && (rec_net[1] == (uchar)(u16_random_errcopy & 0xFF))) //Ҫ�жϷ��ص�֡���кŶԴ�//if ((rec_net[0]==(uchar) ((u16_random_errcopy >>8)& 0b00111111)) &&(rec_net[1]==(uchar) (u16_random_errcopy & 0b11111111)))  //Ҫ�жϷ��ص�֡���кŶԴ�
		{
			if (flag_err1_copy != 0 || flag_err2_copy != 0)
			{
				f_ack = 1;
				t_err200ms = t_onems;
				t_ack5s = t_halfsec;
			}
			else
			{
				R_VoidFrameCode = 0;
				ReturnVoidFrame();
			}
		}
		else if ((rec_net[0] == (((uchar)(u16_random_copy >> 8)) & 0x3F)) && (rec_net[1] == (((uchar)(u16_random_copy)) & 0xFF))) //Ҫ�жϷ��ص�֡���кŶԴ�//else if ((rec_net[0] == (((uchar)(u16_random_copy >> 8)) & 0b00111111)) && (rec_net[1] == (((uchar) (u16_random_copy)) & 0b11111111)))  //Ҫ�жϷ��ص�֡���кŶԴ�
		{
			if (u16_random_copy == u16_random_copyTemp)
			{
				F_ReceiveResponseAck = 1;
				F_FirstPowerRespinseDone = 1;
				F_ResponseFirstFrame = 0;
				F_NeedResponse = 0;
				//R_HomeNetResponseCheckTimeSecond = 0;
			}
			else
			{
				R_VoidFrameCode = 0;
				ReturnVoidFrame();
			}
		}
		else
		{
			R_VoidFrameCode = 0;
			ReturnVoidFrame();
		}
	}
	else if (rec_net[6] == 0x09)
	{
		if ((rec_net[0] == (uchar)((u16_random_errcopy >> 8) & 0x3F)) && (rec_net[1] == (uchar)(u16_random_errcopy & 0x00ff))) //if ((rec_net[0]==(uchar) ((u16_random_errcopy >>8)& 0b00111111)) &&(rec_net[1]==(uchar) (u16_random_errcopy & 0x00ff)))
		{
			if (flag_err1_copy != 0 || flag_err2_copy != 0)
			{
				f_stop_net_alarm = 1;
			}
			else
			{
				R_VoidFrameCode = 0;
				ReturnVoidFrame();
			}
		}
		else
		{
			R_VoidFrameCode = 0;
			ReturnVoidFrame();
		}
	}
	else if (rec_net[6] == 0x70)
	{
		ReturnIdentifycodeFrame();
	}
	else if (rec_net[6] == 0x7d)
	{
		R_HomeNetResponseCheckTime_H8 = rec_net[7];
		R_HomeNetResponseCheckTime_L8 = rec_net[8];
		if (R_HomeNetResponseCheckTime_H8 == 0 && R_HomeNetResponseCheckTime_L8 == 0)
		{
			F_StopResponse = 1;
			R_HomeNetResponseCheckTime = 0;
		}
		else
		{
			R_HomeNetResponseCheckTime100ms = 0;
			F_StopResponse = 0;
			R_HomeNetResponseCheckTime = ((uint)R_HomeNetResponseCheckTime_H8) << 8;
			R_HomeNetResponseCheckTime = R_HomeNetResponseCheckTime + R_HomeNetResponseCheckTime_L8;
			if (R_HomeNetResponseCheckTime < 10)
			{
				R_HomeNetResponseCheckTime = 10;
			}
		}
		ReturnAckFrame();
	}
	else if (rec_net[6] == 0x7e)
	{
		ReturnAckFrame();
	}
	else
	{
		R_VoidFrameCode = 0;
		ReturnVoidFrame();
	}
}
/************************************************/
/*******************����״̬֡����***************/
/************************************************/
void ReturnStateFrame(void)
{
	if (f_send_1ff || f_send_2ff || f_send_data)
	{
		return;
	}
	r_sendsum = 0x2d; //45;
	send_net[0] = rec_net[0];
	send_net[1] = rec_net[1];
	send_net[2] = rec_net[2];
	send_net[3] = rec_net[3];
	send_net[4] = rec_net[4];
	send_net[5] = rec_net[5];
	send_net[6] = 0x02;

	send_net[7] = 0x6d;
	send_net[8] = 0x01;

	send_net[9] = 0x00;
	send_net[10] = 0x00;

	send_net[11] = 0x00;
	send_net[12] = r_voltage; //��ѹ

	send_net[13] = 0x00;
	send_net[14] = 0x00;

	send_net[15] = 0x00;
	send_net[16] = 0x00;

	send_net[17] = 0x00;
	send_net[18] = 0x00;
	send_net[19] = 0x00;
	send_net[20] = 0x00;

#if (LC_DISP_RESOLUTION_0D1 || LC_RESOLUTION_0D1) //��ʾ�ֱ���0.1��
	r_lcxswd = (unsigned char)floor(r_lcxswd_float);
#else
	;
#endif
	send_net[21] = r_lcxswd; //�����ʾ�¶�
	send_net[22] = r_ldxswd;
	send_net[23] = r_lczt; //��ص�λ
	send_net[24] = r_ldzt; //�䶳��λ

	send_net[25] = r_lczt + r_lc_high_alarm; //����ұ�������
	send_net[26] = r_lczt - r_lc_low_alarm;  //����ұ�������
	send_net[27] = r_ldzt + r_ld_high_alarm; //�䶳��������
	send_net[28] = r_ldzt - r_ld_low_alarm;  //�䶳��������

	send_net[29] = u8_year;
	send_net[30] = u8_month;
	send_net[31] = u8_day;
	send_net[32] = u8_hour;
	send_net[33] = u8_minute;
	send_net[34] = 0x00;

	send_net[35] = r_hwsjwd; //����

	send_net[36] = r_lcwdjz;
	send_net[37] = r_ldwdjz;

	send_net[38] = flag_err2;
	send_net[39] = flag_err1;

	send_net[40] = 0x00;
	send_net[41] = 0x00;
	if (f_door_switch)
		send_net[41] = send_net[41] | 0x40;
	send_net[42] = 0x00;
	send_net[43] = 0x00;

	GetSum();
}
/****************����ʱ������֡******************/
/************************************************/
void SetTimeFrame(void)
{
	uchar i;
	if (ACKok == 1) //lxx
	{
		return;
	}
	if (!f_net_first_set_time)
	{
		return;
	}
	if (((u8_year != u8_year_copy) || (u8_month != u8_month_copy) ||
		 (u8_day != u8_day_copy) || (u8_hour != u8_hour_copy) ||
		 (u8_minute != u8_minute_copy)) &&
		(r_set_state < 32))
	{
		u8_year_copy = u8_year;
		u8_month_copy = u8_month;
		u8_day_copy = u8_day;
		u8_hour_copy = u8_hour;
		u8_minute_copy = u8_minute;
                
                u8_year_set = u8_year;
                u8_month_set = u8_month;
                u8_day_set=u8_day;
                u8_hour_set=u8_hour;
                u8_minute_set=u8_minute;    
                
		report_time = 1;
		t_report_time = t_20ms;
		u16_random_time = u16_random;
	}
	if (report_time)
	{
		if (f_send_1ff || f_send_2ff || f_send_data)
		{
			return;
		}
		if (!report_ack_time)
		{
			if ((uchar)(t_20ms - t_report_time) >= 200)
			{
				for (i = 0; i <= 49; i++)
				{
					send_net[i] = 0x00;
				}
				t_report_time = t_20ms;
				r_sendsum = 0x10;
				send_net[1] = (uchar)(u16_random_time & 0x00ff);
				send_net[0] = (uchar)((u16_random_time >> 8) & 0x00ff);
				send_net[2] = rec_net[2];
				send_net[3] = rec_net[3];
				send_net[4] = rec_net[4];
				send_net[5] = rec_net[5];

				send_net[6] = 0x01;
				send_net[7] = 0x5D;
				send_net[8] = 0x09;

				send_net[9] = u8_year_set;
				send_net[10] = u8_month_set;
				send_net[11] = u8_day_set;
				send_net[12] = u8_hour_set;
				send_net[13] = u8_minute_set;
				send_net[14] = 0x00;
				GetSum();
				if(Cnt_SetTime_Sended < 10)
					Cnt_SetTime_Sended ++;
			}
		}
		else
		{
			report_time = 0;
			report_ack_time = 0;
		}
	}
}
/************************************************/
/***************U-Home�ر�֡*****************/
/************************************************/
void NetResponseStateControl(void)
{
	if (F_StopResponse && F_FirstPowerRespinseDone)
	{
		R_HomeNetResponseTimeSecond = 0;
		R_HomeNetResponseCheckTime100ms = 0;
		F_ResponseFirstFrame = 0;
	}
	else
	{
		if (!F_FirstPowerResponse)
		{
			F_FirstPowerResponse = 1;
			u16_random_copyTemp = u16_random;
			u16_random_copy = u16_random_copyTemp;
			R_HomeNetResponseCheckTime100ms = 0;
			F_ResponseFirstFrame = 1;
			ResponseFirstPowerState();
		}
		else if (R_HomeNetResponseCheckTime100ms >= R_HomeNetResponseCheckTime)
		{
			if ((r_voltageCopy != r_voltage) ||
				(r_lcztCopy != r_lczt) ||
				(r_ldztCopy != r_ldzt) ||
				(r_lc_high_alarmCopy != r_lc_high_alarm) ||
				(r_lc_low_alarmCopy != r_lc_low_alarm) ||
				(r_ld_high_alarmCopy != r_ld_high_alarm) ||
				(r_ld_low_alarmCopy != r_ld_low_alarm))
			{
				r_voltageCopy = r_voltage;
				r_lcztCopy = r_lczt;
				r_ldztCopy = r_ldzt;
				r_lc_high_alarmCopy = r_lc_high_alarm;
				r_lc_low_alarmCopy = r_lc_low_alarm;
				r_ld_high_alarmCopy = r_ld_high_alarm;
				r_ld_low_alarmCopy = r_ld_low_alarm;
				F_NeedResponse = 1;
				F_ReceiveResponseAck = 0;
				u16_random_copyTemp = u16_random;
			}
			if ((!F_ReceiveResponseAck) && (F_NeedResponse))
			{
				if (!F_ResponseFirstFrame)
				{
					R_HomeNetCounter = 0;
					F_ResponseFirstFrame = 1;
					R_HomeNetResponseTimeSecond = 0;
					u16_random_copy = u16_random_copyTemp;
					ReturnReportFrame();
				}
			}
			R_HomeNetResponseCheckTime100ms = 0;
		}
		else
		{
			if ((!F_ReceiveResponseAck) && F_ResponseFirstFrame)
			{
				if (R_HomeNetCounter < 2)
				{
					if (R_HomeNetResponseTimeSecond >= 200) //zyj  100416 100506
					{
						R_HomeNetResponseTimeSecond = 0;
						//u16_random_copy = u16_random_copyTemp;
						if (!F_FirstPowerRespinseDone)
						{
							ResponseFirstPowerState();
						}
						else
						{
							ReturnReportFrame();
						}
						R_HomeNetCounter++;
					}
				}
				else
				{
					F_FirstPowerRespinseDone = 1;
					F_ResponseFirstFrame = 0;
					F_NeedResponse = 0;
				}
			}
		}
	}
}
/*******************�㱨֡����*******************/
/************************************************/
void ReturnReportFrame(void)
{
	if (f_send_1ff || f_send_2ff || f_send_data)
	{
		return;
	}
	r_sendsum = 0x2d; //45
					  // u16_random_copy = u16_random;
	send_net[1] = (uchar)(u16_random_copy & 0x00ff);
	send_net[0] = (uchar)((u16_random_copy >> 8) & 0x3F); //0b00111111);  @20181130 CFJ
	send_net[2] = 0x00;
	send_net[3] = 0x00;
	send_net[4] = 0x00;
	send_net[5] = 0x00;

	send_net[6] = 0x06;

	send_net[7] = 0x6d;
	send_net[8] = 0x01;

	send_net[9] = 0x00;
	send_net[10] = 0x00;

	send_net[11] = 0x00;
	send_net[12] = r_voltage; //��ѹ

	send_net[13] = 0x00;
	send_net[14] = 0x00;

	send_net[15] = 0x00;
	send_net[16] = 0x00;

	send_net[17] = 0x00;
	send_net[18] = 0x00;
	send_net[19] = 0x00;
	send_net[20] = 0x00;

	send_net[21] = r_lcxswd; //�����ʾ�¶�
	send_net[22] = r_ldxswd;
	send_net[23] = r_lczt; //��ص�λ
	send_net[24] = r_ldzt; //�䶳��λ

	send_net[25] = r_lczt + r_lc_high_alarm; //����ұ�������
	send_net[26] = r_lczt - r_lc_low_alarm;  //����ұ�������
	send_net[27] = r_ldzt + r_ld_high_alarm; //�䶳��������
	send_net[28] = r_ldzt - r_ld_low_alarm;  //�䶳��������

	send_net[29] = u8_year;
	send_net[30] = u8_month;
	send_net[31] = u8_day;
	send_net[32] = u8_hour;
	send_net[33] = u8_minute;
	send_net[34] = 0x00;

	send_net[35] = r_hwsjwd; //����

	send_net[36] = r_lcwdjz;
	send_net[37] = r_ldwdjz;

	send_net[38] = flag_err2;
	send_net[39] = flag_err1;

	send_net[40] = 0x00;
	send_net[41] = 0x00;
	if (f_door_switch)
		send_net[41] = send_net[41] | 0x40;
	send_net[42] = 0x00;
	send_net[43] = 0x00;
	GetSum();
}
/************************************************/
/************************************************/
/*******     �����ϵ�㱨״̬     ***************/
/************************************************/
void ResponseFirstPowerState(void)
{
	if (f_send_1ff || f_send_2ff || f_send_data)
	{
		return;
	}
	r_sendsum = 0x08;
	send_net[0] = (((uchar)(u16_random_copy >> 8)) & 0x3F); //@20181130 CFJ //0b00111111); //zyj  100504
	send_net[1] = (((uchar)(u16_random_copy)) & 0xFF);		//@20181130 CFJ //0b11111111);  //zyj  100504
	send_net[2] = rec_net[2];
	send_net[3] = rec_net[3];
	send_net[4] = rec_net[4];
	send_net[5] = rec_net[5];
	send_net[6] = 0x72;
	GetSum();
}
/****************************************************/
/******************�жϷ��ͻ㱨֡����****************/
//void r_voltage_report_judge(void);
//void r_lczt_report_judge(void);
//void r_ldzt_report_judge(void);
//void lc_high_alarm_report_judge(void);
//void lc_low_alarm_report_judge(void);
//void ld_high_alarm_report_judge(void);
//void ld_low_alarm_report_judge(void);
/****************************************************/
/*void report_judge(void) 
{   r_voltage_report_judge();
    r_lczt_report_judge(); 
    r_ldzt_report_judge();
    lc_high_alarm_report_judge(); 
    lc_low_alarm_report_judge();
    ld_high_alarm_report_judge(); 
    ld_low_alarm_report_judge();  
}*/
/****************************************************/
/*void r_voltage_report_judge(void) 
{  if(r_voltage != r_voltage_report) 
   {  if((uchar)(t_halfsec-t_voltage)>=120) 
      {      
        t_voltage = t_halfsec;
        r_voltage_report = r_voltage;     
        report = 1;
        t_report = t_20ms;   
        u16_random_copy = u16_random;
      }
   } 
   else
   {
      t_voltage = t_halfsec;
   }
}*/
/****************************************************/
/*void r_lczt_report_judge(void) 
{
  if(r_lczt!=r_lczt_report)
  {
    r_lczt_report = r_lczt;
    report = 1; 
    t_report = t_20ms;
    u16_random_copy = u16_random;  
  }
}*/
/****************************************************/
/*void r_ldzt_report_judge(void)
{
  if(r_ldzt!=r_ldzt_report)
  {    
    r_ldzt_report = r_ldzt;
    report = 1;
    t_report = t_20ms;
    u16_random_copy = u16_random; 
  }
}*/
/****************************************************/
/*void lc_high_alarm_report_judge(void) 
{
  if(r_lc_high_alarm != lc_high_alarm_report)
   {
     lc_high_alarm_report = r_lc_high_alarm;
     report = 1;
     t_report = t_20ms;
     u16_random_copy = u16_random;
   }
}*/

/****************************************************/
/*void lc_low_alarm_report_judge(void)
{
  if(r_lc_low_alarm != lc_low_alarm_report)
  {
     lc_low_alarm_report = r_lc_low_alarm;
     report = 1;
     t_report = t_20ms;
     u16_random_copy = u16_random; 
  }
}*/
/****************************************************/
/*void ld_high_alarm_report_judge(void)
{
  if(r_ld_high_alarm != ld_high_alarm_report)
  {
    ld_high_alarm_report = r_ld_high_alarm;
     report = 1;
     t_report = t_20ms;
     u16_random_copy = u16_random;
  }
}*/
/****************************************************/
/*void ld_low_alarm_report_judge(void) 
{
  if(r_ld_low_alarm != ld_low_alarm_report)
  {
     lc_low_alarm_report = r_lc_low_alarm;
     report = 1;
     t_report = t_20ms;
     u16_random_copy = u16_random; 
  }
}    */
/*******************���ϱ�������*********************/
/****************************************************/
void NetErrAlarm(void)
{
	if (flag_err1 == 0 && flag_err2 == 0)
	{
		flag_err1_copy = flag_err1;
		flag_err2_copy = flag_err2;
		f_stop_net_alarm = 0;
		f_ack = 0;
	}
	else
	{
		if (flag_err1 != flag_err1_copy || flag_err2 != flag_err2_copy)
		{
			flag_err1_copy = flag_err1;
			flag_err2_copy = flag_err2;
			t_err200ms = t_onems;
			t_ack5s = t_halfsec;
			f_stop_net_alarm = 0;
			f_ack = 0;
			u16_random_errcopy = u16_random;
		}
		else if (f_stop_net_alarm)
		{
			return;
		}
		else if (f_ack)
		{
			if ((uchar)(t_halfsec - t_ack5s) >= 10)
			{
				f_ack = 0;
			}
		}
		else if ((uchar)(t_onems - t_err200ms) >= 200)
		{
			t_err200ms = t_onems;
			t_ack5s = t_halfsec;
			r_sendsum = 0x0e; //14
							  // u16_random_errcopy = u16_random;
			send_net[1] = (uchar)(u16_random_errcopy & 0x00ff);
			send_net[0] = (uchar)((u16_random_errcopy >> 8) & 0x00ff);

			send_net[2] = 0;
			send_net[3] = 0;
			send_net[4] = 0;
			send_net[5] = 0;

			send_net[6] = 0x04;

			send_net[7] = 0x0f;
			send_net[8] = 0x5a;

			send_net[9] = 0;
			send_net[10] = 0;

			send_net[11] = flag_err2;
			send_net[12] = flag_err1;
			GetSum();
		}
	}
}
/************************************************/
/*******************������Чֵ����***************/
/************************************************/
void ReturnVoidFrame(void)
{
	r_sendsum = 0x0c;
	send_net[0] = rec_net[0];
	send_net[1] = rec_net[1];
	send_net[2] = rec_net[2];
	send_net[3] = rec_net[3];
	send_net[4] = rec_net[4];
	send_net[5] = rec_net[5];
	send_net[6] = 0x03;
	send_net[7] = 0x00;
	send_net[8] = 0x00;
	send_net[9] = 0x00;
	send_net[10] = R_VoidFrameCode;
	GetSum();
}
/*******************����ACK����******************/
/************************************************/
void ReturnAckFrame(void)
{
	r_sendsum = 0x08;
	send_net[0] = rec_net[0];
	send_net[1] = rec_net[1];
	send_net[2] = rec_net[2];
	send_net[3] = rec_net[3];
	send_net[4] = rec_net[4];
	send_net[5] = rec_net[5];
	send_net[6] = 0x05;
	GetSum();
}
/******************����ʶ�����ѯ֡����***********/
/************************************************/
void ReturnIdentifycodeFrame(void)
{
	r_sendsum = 0x18;
	send_net[0] = rec_net[0];
	send_net[1] = rec_net[1];
	send_net[2] = rec_net[2];
	send_net[3] = rec_net[3];
	send_net[4] = rec_net[4];
	send_net[5] = rec_net[5];
	send_net[6] = 0x71;

	send_net[7] = 0x01;
	send_net[8] = 0xc1;
	send_net[9] = 0x20;
	send_net[10] = 0x02;
	send_net[11] = 0x40;
	send_net[12] = 0x00;
	send_net[13] = 0x81;
	send_net[14] = 0xc0;
	send_net[15] = 0x40;
	send_net[16] = 0x81;
	send_net[17] = 0x03;
	send_net[18] = 0x35;
	send_net[19] = 0x6c;
	send_net[20] = 0x90;
	send_net[21] = 0x00;
	send_net[22] = 0x00;
	GetSum();
}
/************************************************/
/*******************����У��ͳ���***************/
/************************************************/
void GetSum(void)
{
	uchar r_checksum = 0;
	for (r_sendr = 0; r_sendr < (unsigned char)(r_sendsum - 1); r_sendr++)
	{
		if (send_net[r_sendr] == 0xff)
		{
			r_checksum = r_checksum + 0x55;
		}
		r_checksum = r_checksum + send_net[r_sendr];
	}
	r_checksum = r_checksum + r_sendsum;
	send_net[r_sendr] = r_checksum;
	UART_ITConfig(UART3, UART_IT_TB, ENABLE);  //ʹ�ܷ����ж�
}

/************************************************/
/*******************���ճ�ʼ������***************/
/************************************************/
void ReceiveInitial(void)
{
	if (f_rec_1ff || f_rec_2ff || f_rec_data)
	{
		if ((uchar)(t_onems - t_net_rec) >= 200) //200ms
		{
			f_rec_over = 0;
			f_rec_1ff = 0;
			f_rec_2ff = 0;
			f_rec_data = 0;
			f_rec55 = 0;
			r_rec55sum = 0;
		}
	}
}
/************************************************/
/*******************����uart0��ʼ������***************/
/************************************************/
void ReceiveInitial1(void)
{
	if (f_rec_169 || f_rec_201 || f_rec_data1)
	{
		if ((uchar)(t_onems - t_net_rec1) >= 200) //200ms
		{
			f_rec_over1 = 0;
			f_rec_169 = 0;
			f_rec_201 = 0;
			f_rec_data1 = 0;
		}
	}
}

/*********************************************
�������ƣ�void EheatConTrolP(void)
���������r_lcad  ����¶ȴ�������ADֵ
���������
�������ܣ�
��д���ڣ�2019/4/24 10:16:32
���ߣ�   HW
***************************************************/

void EheatConTrolP(void)//��˪����˿���������
{
	if (f_lc_sensor_err)//��ش�����NTC1����
	{
		Eheatcontrolpin = 0;//���Ƽ���˿
	}
//	else if (r_lcad_12b >= HEATONAD)
//	{
//		Eheatcontrolpin = 1;//
//	}
//	else if (r_lcad_12b <= HEATOFFAD)
//	{
//		Eheatcontrolpin = 0;
//	}
}
/************************************************/
void Pannel_Comm_Deal(void) //@CFJ ���ͨѶ����
{
	if (g_Pannel_Comm_bRcv_Done)
	{
		g_Pannel_Comm_bRcv_Done = 0;
		//g_Bus_Rec_Time=0;//��ȷ���������ݣ���2sʱ����0��
		//g_Txd_Time = 250 ;                   //500ms��ʱ��ʼ ����
		g_UART1_RCV_Cyc = 0;
		g_UART1_RCV_Counter = 0;

		g_Bus_Error_Time = 0; //1minʱ��
		g_Sys_Erflag0_Comm = 0;

		//g_Sys_Stateflag10_bFirst_Rcv_ODM = TURE ;
		//g_RX_LED_Time = 5 ;                       //����У�����ȷ ���յ���50ms

		///if(!g_Sys_Setflag4_bID_Only)
		//{
		l_ODM_Data_Parse();
		//}
		//bUartSendStartFlag = 1 ;//g_IDM_ODM_Comm_bPrase_Done = TURE ;
	}
}

float Check_Humi_Tab(unsigned int Ad)
{
  unsigned int i=0;
  float Temp;
  unsigned char Temp2;
  if(  (float)Ad >= Humi_Tab[20][0] )     
  {
		return Humi_Tab[20][1];  //100%
	} 
	else if( (float)Ad <= Humi_Tab[0][0])
	{
		return Humi_Tab[0][1] ;   //0%
	} 
	else 
	{
	   while( (i<21)&&((float)Ad > Humi_Tab[i][0]) ) 
	      i++;
		 if(i == 0)
			 i = 1;
	   Temp  = Humi_Tab[i-1][1] + 5*( Ad - Humi_Tab[i-1][0] )/( Humi_Tab[i][0] - Humi_Tab[i-1][0] ); //������
	   Temp2 = (unsigned char) floor( Temp + 0.5 );  //��������ȡ��
	}
  return Temp2;
}

////ʪ����ʾ����
///*
//��Сֵ40��
//40-65������ʵ����ֵ��ʾ�� 
//65-75����ֵ-5
//75-85����ֵ-10
//85-95����ֵ-20
//95���ϣ���ֵ-25
//*/
//static float Humidity_DisRule1(float * humidity)
//{
//	float value=0;
//	value = *humidity;
//	if(value<40){
//		value = 40;
//	}else if(value < 65){
//		;
//	}else if(value < 75){
//		value = value - 5;
//	}else if(value < 85){
//		value = value - 10;
//	}else if(value < 95){
//		value = value - 20;
//	}else if(value >= 95){
//		return value - 25;
//	}
//	return value;
//}

//�Ҷ������ʾ���򣬱�֤���Ա仯
static float Humidity_DisRule(float * humidity)
{
	float value=0;
	value = *humidity;
	if(value<40){
		value = 40;
	}else if(value <= 100){
		value = 40+(value-40)*(35/60.00);
	}else{
		value = 75;
	}
	return value;
}


//-----------------------------------------------------------------//
void l_ODM_Data_Parse(void) //��������������� CFJ
{
	unsigned char Lc_Temp, Lc_Temp_2, VoltageTemp;
	unsigned char batteryTemp;
	unsigned int g_HXJ_SD_AD;
	unsigned int lc_Hs_Temp=0;
	float g_HXJ_SD;
	if (g_Sys_ReceiveDataType) //��������֡����  CFJ
	{
		//��ѹ
		VoltageTemp = Pannel_Uart1Data[22] << 4;
		VoltageTemp |= Pannel_Uart1Data[23] >> 4;
		r_voltage_ad = VoltageTemp;
//		if ((r_voltage_ad >= 13) && (r_voltage_ad <= 254))
//		{
//			if (r_voltage_ad >= 30)
//			{
//				r_voltage = tab_voltage[r_voltage_ad - 13]; //VoltageDisplayValue1
//			}
//			else
//			{
//				r_voltage = 0;
//			}
//		}

		if(r_voltage_ad <= 30)
			r_voltage = 0;
		else if(r_voltage_ad <= 215)
			r_voltage = tab_voltage[r_voltage_ad-13]; 
		else
			r_voltage = 255;
		
		//���
		batteryTemp = (Pannel_Uart1Data[20] << 4);
		batteryTemp |= (Pannel_Uart1Data[21] >> 4);
		r_battery_ad = batteryTemp;
		//flag_rec1 = Pannel_Uart1Data[6];//��Ч
		if (!f_first_ad)
		{
			r_voltage_report = r_voltage; //r_voltage_report������Ч
		}
		if (Pannel_Uart1Data[24] & 0x07) //@20181130 CFJ ��ز巴 �̽� ��ѹ����
		{
			f_battery = 1; //��ع��ϱ�־
		}
		else
		{
			f_battery = 0;
		}
		f_Self_first_ad = 1;
	}
	else
	{
		#ifdef  Need_Ld
			//�䶳���¶ȴ����õ��װ��⵽���䶳�¶�ADֵ
			r16_ldad = Pannel_Uart1Data[13]; //��λ//rec[3]; ��Ӧ�װ�byte[17]
			r16_ldad = r16_ldad << 8;		 //r16_ldad<<8;
			r16_ldad = (r16_ldad & 0xff00) | Pannel_Uart1Data[14];
			r16_ldad >>= 2; //12λADת��Ϊ10λAD
			if ((r16_ldad <= 1010) && (r16_ldad >= 15))
			{
				//��������ڳ���û��ȫ�������䴦��,����Ӱ�������ȷ��
				if (SelfCheckFlag == 0)//��ǰ�����������У����Լ�ģʽ���Լ�ʱSelfCheckFlag=1
				{
					if ((r_ldwdjz >= 20) && (r16_ldad > 35)) //�¶�У��������&&�¶�С�ڵ���50 r_ldwdjz�������ͨ��ͨ�Ź�����Ӧ��Ҳ����ͨ������
					{
						r16_ldad = r16_ldad - (r_ldwdjz - 20) * 4; //У׼��ƫ10.�¶ȶ�Ӧ4��AD
					}
					else if ((r_ldwdjz < 20) && (r16_ldad < 990))
					{
						r16_ldad = r16_ldad + (20 - r_ldwdjz) * 4;
					}
				}
			}

			if (r16_ldad > 470)
			{
				r_ldsjwd = 140; //-60��
			}
			else if (r16_ldad < 36)
			{
				r_ldsjwd = 250; //50��
			}
			else
			{
				r_ldsjwd = (tab_temperature[r16_ldad - 36]);
			}
		#else//��ʾʪ��
			//�õ��װ��⵽��ʪ��ADֵ
			r16_ldad = Pannel_Uart1Data[17]; //��λ//rec[3]; ��Ӧ�װ�byte[21]
			r16_ldad = r16_ldad << 8;		 //r16_ldad<<8;
			r16_ldad = (r16_ldad & 0xff00) | Pannel_Uart1Data[18];
			//r16_ldad >>= 2; //12λADת��Ϊ10λAD
			g_HXJ_SD_AD = r16_ldad;
			if (g_HXJ_SD_AD > 4000)
			{
				g_HXJ_SD = 100; //100%
			}
			else if (g_HXJ_SD_AD < 100)
			{
				g_HXJ_SD = 0; //0%
			}
			else
			{
        		g_HXJ_SD = Check_Humi_Tab(g_HXJ_SD_AD);
			}

		    //�Ź�����������ʾ����
		    g_HXJ_SD=Humidity_DisRule(&g_HXJ_SD);
			
			r_ldsjwd = (u8t)g_HXJ_SD;

							//��������ڳ���û��ȫ�������䴦��,����Ӱ�������ȷ��
			if (SelfCheckFlag == 0)//��ǰ�����������У����Լ�ģʽ���Լ�ʱSelfCheckFlag=1
			{
				if ((r_ldwdjz >= 20)) //�¶�У��������&&�¶�С�ڵ���50 r_ldwdjz�������ͨ��ͨ�Ź�����Ӧ��Ҳ����ͨ������
				{
						r_ldsjwd=r_ldsjwd+(r_ldwdjz-20);
						if(r_ldsjwd>100){r_ldsjwd=100;}
				}
				else if ((r_ldwdjz < 20) && (r_ldwdjz>=5))
				{
						
//						if((r_ldsjwd+r_ldwdjz)<=20){
//							r_ldsjwd=0;
//						}else{
							r_ldsjwd=r_ldsjwd-(20-r_ldwdjz);
//						}
				}
			}
		#endif


		//����¶ȵĴ���,û���¶���ʾ��50~-60�ȵ�����
		Lc_Temp = Pannel_Uart1Data[5] << 4;  //NTC1��������ؿ��ƴ�����
		Lc_Temp |= Pannel_Uart1Data[6] >> 4;

		Lc_Temp_2 = Pannel_Uart1Data[7] << 4; //NTC2������ �����ʾ������
		Lc_Temp_2 |= Pannel_Uart1Data[8] >> 4; // 20190410  HYC-386��Ŀ�޸�ΪNTC2��Ϊ��ش�����HW
//		r_lcad = Lc_Temp;//���ƴ����� ���ADֵ ���ֵֻ���ڿ��Ƶ�ǰ�¶ȱ�������Ϊ5��ʱ������5.5�ȿ�ѹ��������4.5�ȹ�ѹ��
		
		
//		if (Lc_Temp >= 250 || Lc_Temp <= 10) //�¶ȴ���60 С��-20��Ϊ����,����NTC1��NTC2
//		{
				 //r_lcXsad = Lc_Temp_2;
				 r_lcad_8b=Lc_Temp_2;
         r_lcad_12b = Pannel_Uart1Data[7] * 256 + Pannel_Uart1Data[8];
//		}
//		else
//		{
			r_lcXsad = Lc_Temp;
      r_lcad_12b_c = Pannel_Uart1Data[5] * 256 + Pannel_Uart1Data[6];
//		}

        /*8λadƫ�ƴ���*/
		if ((r_lcad_8b < 250) && (r_lcad_8b > 5))
		{
			if (SelfCheckFlag == 0)
			{
				if ((r_lcwdjz >= 100) && (r_lcad_8b > 20)) //�¶�У��
				{
					r_lcad_8b = r_lcad_8b - ((r_lcwdjz - 100) / 10.0) * 3;
				}
				else if ((r_lcwdjz < 10) && (r_lcad_8b < 235))
				{
					r_lcad_8b = r_lcad_8b + ((100 - r_lcwdjz) / 10.0) * 3;
				}
			}
		}
		r_lcad_8b_c = CheckLcTable(r_lcXsad);   //�����¶�����ֻ���ڲ���ģʽ
		r_lcwd_float_c = (float)CheckLcTable_12b(r_lcad_12b_c) + (r_lcxswdjz - 100) / 10.0;;   //
		r_lcwd = CheckLcTable(r_lcad_8b);   //�����¶�����ֻ���ڲ���ģʽ
		r_lcwd_float = (float)CheckLcTable_12b(r_lcad_12b) + (r_lcwdjz - 100) / 10.0;
        if(r_lcwd_float < 8.0)
            r_lcwd_float = 8.0;
        if(r_lcwd_float > 98.0)
            r_lcwd_float = 98.0;
				

		//�û�˪��������ADֵ
//		lc_Hs_Temp=0;
		lc_Hs_Temp = Pannel_Uart1Data[9]; //��Ӧ�װ�byte[13]
		lc_Hs_Temp = lc_Hs_Temp << 8;		 //r16_ldad<<8;
		r16_lcHuaShuangad = (lc_Hs_Temp & 0xff00) | Pannel_Uart1Data[10];
		r_lcHuaSHuangxswd_float=ADC2Temper_3840(r16_lcHuaShuangad);//�õ��¶�0ֵ*10+510��ֵ
		if(r_lcHuaSHuangxswd_float>=(510+990)){r_lcHuaSHuangxswd_float=510+990;}
		//�������ֵ<510�Ǹ�������������Ǹ�����
		//���統����ֵΪ400ʱ����Ӧ�¶�ӦΪ-(510-400)/10��(-11��),������ֵΪ600ʱ����Ӧ�¶�Ϊ+(600-510)/10��(+9��)
		r_lcHuaShuangsjwd=((int)(r_lcHuaSHuangxswd_float-510))/10;
//		if(r_lcHuaSHuangxswd_float>510){//0������
//			r_lcHuaShuangsjwd=(int)((r_lcHuaSHuangxswd_float-510)/10);
//		}else{//
//			r_lcHuaShuangsjwd=-(int)((510-r_lcHuaSHuangxswd_float)/10);
//		}
		
		//��������������ADֵ
//		lc_Hs_Temp=0;
		lc_Hs_Temp = Pannel_Uart1Data[11]; //��Ӧ�װ�byte[13]
		lc_Hs_Temp = lc_Hs_Temp << 8;		 //r16_ldad<<8;
		r16_lcLengNingad = (lc_Hs_Temp & 0xff00) | Pannel_Uart1Data[12];
		r_lcLengNingxswd_float=ADC2Temper_3840(r16_lcLengNingad);
		if(r_lcLengNingxswd_float>=(510+990)){r_lcLengNingxswd_float=510+990;}
		r_lclnsjwd=((int)(r_lcLengNingxswd_float-510))/10;
//		if(r_lcLengNingxswd_float>510){//
//			r_lclnsjwd=(int)((r_lcLengNingxswd_float-510)/10);
//		}else{//
//			r_lclnsjwd=-(int)((510-r_lcLengNingxswd_float)/10);
//		}
		
/********************************************************************
�¶�У��������������صĿ��ƴ������ϣ��Ұ���������ʾ��������
		if ((r_lcad < 250) && (r_lcad > 5))
		{//��ؿ��ƴ������¶�������
			if (SelfCheckFlag == 0)
			{//��ǰΪ�����������У����Լ�ģʽ
				if ((r_lcwdjz >= 10) && (r_lcad > 20)) //�¶�У��
				{//�¶�1�ȴ�Լ��Ӧ3��ad��r_lcwdjz�ķ�Χ��5-15��������5��ƫ��
					r_lcad = r_lcad - (r_lcwdjz - 10) * 3;
				}
				else if ((r_lcwdjz < 10) && (r_lcad < 235))
				{
					r_lcad = r_lcad + (10 - r_lcwdjz) * 3;
				}
			}
		}
**********************************************************************/

//		if ((r_lcad < 250) && (r_lcad > 5))
//		{//��ؿ��ƴ������¶�������
//			if (SelfCheckFlag == 0)
//			{//��ǰΪ�����������У����Լ�ģʽ
//				if ((r_lcwdjz >= 10) && (r_lcad > 5)) //�¶�У��
//				{//�¶�1�ȴ�Լ��Ӧ3��ad��r_lcwdjz�ķ�Χ��5-15��������5��ƫ�� ADֵ���¶�Ϊ���ȹ�ϵ
//					r_lcwd = CheckLcTable(r_lcad) + (r_lcwdjz - 10);//ʹ��ADֵ��ȡ���¶�ֵ -24�Ժ���ƫ38Ϊ�¶�����ADֵ(116+24)����2��
//					//r_lcad = r_lcad - (r_lcwdjz - 10) * 3;
//				}
//				else if ((r_lcwdjz < 10) && (r_lcad < 250))
//				{
//					r_lcwd = CheckLcTable(r_lcad) - (10 - r_lcwdjz);//ʹ��ADֵ��ȡ���¶�ֵ -24�Ժ���ƫ38Ϊ�¶�����ADֵ(116+24)����2��

//					//r_lcad = r_lcad + (10 - r_lcwdjz) * 3;
//				}
//				if(r_lcwd<=8){r_lcwd = 8;}
//				if(r_lcwd>=98){r_lcwd = 98;}
//			}
//		}else{
//			r_lcwd = CheckLcTable(r_lcad);
//		}

//		if ((Lc_Temp_2 < 250) && (Lc_Temp_2 > 5))
//		{//�����ʾ�������¶�������
//			if (SelfCheckFlag == 0)
//			{//��ǰΪ�����������У����Լ�ģʽ
//				if ((r_lcxswdjz >= 10) && (Lc_Temp_2 > 5)) //�¶�У��
//				{//�¶�1�ȴ�Լ��Ӧ3��ad��r_lcwdjz�ķ�Χ��5-15��������5��ƫ�� ADֵ���¶�Ϊ���ȹ�ϵ
//					//Lc_Temp_2 = Lc_Temp_2 - (r_lcxswdjz - 10) * 3;
//					//r_lcwd_float = (float)CheckLcTable(Lc_Temp_2) + (r_lcxswdjz - 10);//jhw��ֻ��Ϊ��ʾʹ��
//					r_lcwd_float = (float)CheckLcTable_12b(r_lcad_12b) + (r_lcwdjz - 100) / 10.0;
//					if(r_lcwd_float < 8.0)
//							r_lcwd_float = 8.0;
//					if(r_lcwd_float > 98.0)
//							r_lcwd_float = 98.0;
//				}
//				else if ((r_lcxswdjz < 10) && (Lc_Temp_2 < 250))
//				{
//					r_lcwd_float = (float)CheckLcTable(Lc_Temp_2) - (10 - r_lcxswdjz);
//					//Lc_Temp_2 = Lc_Temp_2 + (10 - r_lcxswdjz) * 3;
//				}
//				if(r_lcwd_float<=8){r_lcwd_float=(float)8;}
//				if(r_lcwd_float>=98){r_lcwd_float=(float)98;}
//			}
//		}else{
//			r_lcwd_float = (float)CheckLcTable(Lc_Temp_2);
//		}
//		if (Lc_Temp_2 >= 250 || Lc_Temp_2 <= 5) //�¶ȴ���60 С��-30��Ϊ����,ʹ��ntc1���ƴ����� ���ϵ�ʱ���������//TODO
//		{
//			//Lc_Temp_2 = r_lcad;
//			r_lcwd_float=(float)r_lcwd;
//		}
		
//		//r_lcad = 210; // TEST ���� @20181025 CFJ
//		r_lcwd = CheckLcTable(r_lcad);//ʹ��ADֵ��ȡ���¶�ֵ -24�Ժ���ƫ38Ϊ�¶�����ADֵ(116+24)����2��
////		r_lcwd_float = (float)CheckLcTable(r_lcad);//������ӣ�Ϊ����LED����ʾС�������¶�(���¶�)��������������˳����ж�
//		r_lcwd_float = (float)CheckLcTable(Lc_Temp_2);//jhw��ֻ��Ϊ��ʾʹ��
	}
	f_first_ad = 1;
}

void l_Pannel_DataTx(void)
{
	if (bUartSendStartFlag) //&& (g_Txd_Time <= 0))ÿ500ms��һ�ѣ����Դ�巢һ������
	{
		bUartSendStartFlag = 0;
		l_Send_Data_Build();
	}
}



void l_Send_Data_Build(void)
{
	unsigned char NDegree, Sum, Temp;
	l_ClrIDMSendBuf();//��ո����еı�־λ
	Sum = 0;
	g_IDM_TX_ODM_Data[0] = 0xAA;

	g_IDM_TX_ODM_Data[1] = 0x55; //  ͷ��4���ֽ�

	g_IDM_TX_ODM_Data[2] = 0xE7;

	g_IDM_TX_ODM_Data[3] = 0x18;
	//----------------------------------//
	g_IDM_TX_ODM_Data[4] = 0xFF; //��ʾ���ַ

	g_IDM_TX_ODM_Data[5] = 0xB0; //I/O���ַ

	g_IDM_TX_ODM_Data[6] = 0x00; //����֡����
	//-----ϵͳ�趨���¶�-------//

	if (r_lczt >= 38) //����趨�¶� @20181130 CFJ
	{				  //���ڵ���0��
		Temp = (r_lczt - 38);
		g_IDM_TX_ODM_Data[7] = Temp;
	}
	else
	{ //0������
		Temp = (38 - r_lczt);
		g_IDM_TX_ODM_Data[7] = Temp;
		g_IDM_TX_ODM_Data[7] |= 0x80;
	}

	#ifdef Need_Ld
		if (r_ldzt >= 200) //�䶳�趨�¶�
		{				   //���ڵ���0��
			Temp = (r_ldzt - 200);
			g_IDM_TX_ODM_Data[8] = Temp;
		}
		else
		{ //0������
			Temp = (200 - r_ldzt);
			g_IDM_TX_ODM_Data[8] = Temp;
			g_IDM_TX_ODM_Data[8] |= 0x80;
		}
	#else
		g_IDM_TX_ODM_Data[8] = r_ldzt;
	#endif
	g_IDM_TX_ODM_Data[9] = 0x00; //Ԥ��δ��

	if (f_lc_fan) //��ص������������ѹ����һ���ڻ��������
	{
		g_IDM_TX_ODM_Data[10] |= 0x01;
	}

	if (f_ln_fan) //�䶳���
	{
		g_IDM_TX_ODM_Data[10] |= 0x02;
	}

	if (Eheatcontrolpin)
	{
		g_IDM_TX_ODM_Data[10] |= 0x04; //2019/4/24 HW  �����˿�Ŀ���
	}
	if (f_defrost) //��˪
	{
		g_IDM_TX_ODM_Data[10] |= 0x04;
	}

	if (f_nd_fan) //��ص��ڷ��/�����������ǻ������ѭ������
	{
		g_IDM_TX_ODM_Data[10] |= 0x08;
	}

	if (f_lc_compressor) //��ص�ѹ����
	{
		g_IDM_TX_ODM_Data[11] |= 0x01;
	}

	if (f_compressor) //�䶳ѹ��
	{
		g_IDM_TX_ODM_Data[11] |= 0x02;
	}

	if (f_LightStatusPin) //12V OUT3��Ϊ��ص����  -20190410-HW
	{
		g_IDM_TX_ODM_Data[12] |= 0x04;
	}

	if (f_long_range_alarm) //Զ�̱��� @20181130  CFJ
	{
		g_IDM_TX_ODM_Data[13] |= 0;
	}
	else
	{
		g_IDM_TX_ODM_Data[13] |= 0x01; //��Ϊ�װ��Զ�̱����ļ̵����ǳ��յġ���ҵ����ΰҪ���й���ʱ����0 ���޷� 1
	}

	g_IDM_TX_ODM_Data[14] = 0x00; //Ԥ��
	/*  ���Э��λ�ĸ��� ���±������ã��Ѻ���ҵ����ͨ�� @20181130 CFJ
    if(f_ld_high_buzz)  //Զ�̱���      
    {
        g_IDM_TX_ODM_Data[15]|=0x01;
    }    
    if(f_ld_low_buzz)
    {
    	  g_IDM_TX_ODM_Data[15]|=0x02;
    } 
    */
	if (f_power_buzz)
	{
		g_IDM_TX_ODM_Data[15] |= 0x04; //�ϵ籨��
	}
	/* @20181130 CFJ
    if(f_battery)
    {
    	 g_IDM_TX_ODM_Data[15]|=0x08; //��ص�����
    }
    */
	if (f_lc_sensor_err)
	{
		g_IDM_TX_ODM_Data[15] |= 0x40; //g_IDM_TX_ODM_Data[15]|=0x80; //��ҵ��Ҫ��NTC2��ΪNTC1��ش��������� @20181221 CFJ
	}

	//  g_IDM_TX_ODM_Data[16]|=0x00; ����δ��
//	if (f_ld_sensor_err)
//	{
//		g_IDM_TX_ODM_Data[17] |= 0x01; //�䶳����������
//	}
	if (f_hw_sensor_err)
	{
		g_IDM_TX_ODM_Data[17] |= 0x04; //���´���������
	}
	if (f_hw_high38)
	{
		g_IDM_TX_ODM_Data[17] |= 0x10; //���¹���
	}

	if (f_ld_high_buzz) //if(f_ld_high) @20181130 CFJ
	{
		g_IDM_TX_ODM_Data[18] |= 0x04; //�䶳���±��� v1.3Э��
	}
	if (f_ld_low_buzz) //if(f_ld_low) @20181130 CFJ
	{
		g_IDM_TX_ODM_Data[18] |= 0x08; //�䶳���±���
	}

	if (f_lc_high_buzz) //if(f_lc_high) @20181130 CFJ
	{
		g_IDM_TX_ODM_Data[18] |= 0x01; //��ظ��±���
	}

	if (f_lc_low_buzz) //if(f_lc_low) @20181130 CFJ
	{
		g_IDM_TX_ODM_Data[18] |= 0x02; //��ص��±���
	}

	if (f_buzz_alarm)
	{
		g_IDM_TX_ODM_Data[19] = 0x01; //��������
	}
	g_IDM_TX_ODM_Data[22] = r_lcad_8b_c;//��ؿ��ƴ������¶�NTC1
	g_IDM_TX_ODM_Data[23] = r16_ldad / 256;
	g_IDM_TX_ODM_Data[24] = r16_ldad % 256;
	g_IDM_TX_ODM_Data[25] = 0xFF; //ѹ����1

	g_IDM_TX_ODM_Data[26] = 0xFF; //ѹ����2

	/*
    if(!g_Sys_Erflag0_Comm)  //�޹��ϣ����ϵ�Ĭ�Ͽ���
    {
        g_IDM_TX_ODM_Data[2] |=0x80;
    }
    if(f_lc_compressor)  //���ѹ��
    {
        g_IDM_TX_ODM_Data[2] |=0x40;
    } 
    if(f_compressor)  //�䶳ѹ��
    {
        g_IDM_TX_ODM_Data[2] |=0x20;
    
    }
    if(f_lc_fan)  //��ط��
    {
        g_IDM_TX_ODM_Data[2] |=0x10;    
    }
    
    if(f_ln_fan)  //�䶳���
    {
        g_IDM_TX_ODM_Data[2] |=0x08;    
    }
    
    if(f_defrost)  //��˪
    {
        g_IDM_TX_ODM_Data[2] |=0x04;    
    }
    
    if(f_nd_fan)  //�ڷ��
    {
        g_IDM_TX_ODM_Data[2] |=0x02;    
    }
    
    if(g_Compressor_Frequency_Flag1)  //ѹ��1��Ƶ/��Ƶѡ���־
    {
        g_IDM_TX_ODM_Data[3] |=0x80;  //��Ƶ
    }
    else
    {   
         g_IDM_TX_ODM_Data[3]=0xFF;
    }
    if(g_Compressor_Frequency_Flag2)  //ѹ��2��Ƶ/��Ƶѡ���־
    {
        g_IDM_TX_ODM_Data[4] |=0x80;  //��Ƶ    
    }
    else
    {
        g_IDM_TX_ODM_Data[4]=0xFF;
    }
    
    if(f_long_range_alarm)  //Զ�̱���
    {
        g_IDM_TX_ODM_Data[5]|=0x80;
    }
    //����� Ĭ����
     g_IDM_TX_ODM_Data[5]|=0x40;
    
    if(f_buzz_alarm)
    {
        g_IDM_TX_ODM_Data[5]|=0x20;
    }
    
    //12v ���� 4·�����  D4 D3 D2 D1
    
    g_IDM_TX_ODM_Data[5]|=0x00;
    
    //Ԥ��
  
    g_IDM_TX_ODM_Data[6] = 0x00;
    g_IDM_TX_ODM_Data[7] = 0x00;
    
    //У����ֽ�
    g_IDM_TX_ODM_Data[8]=0;  
    */
	for (NDegree = 4; NDegree <= 28; NDegree++)
	{
		Sum += g_IDM_TX_ODM_Data[NDegree];
	}
	g_IDM_TX_ODM_Data[29] = Sum;
	g_UART1_TXD_Counter = 0;
	RS_485_STATUS_SET_HIGH; //������Ч
	g_Rec_Status_Flag = 0;
	UART_ITConfig(UART4, UART_IT_TB, ENABLE);
}
//------------------------------------------------------------------------//
void l_ClrIDMSendBuf(void)
{
	unsigned char NSendNum;
	for (NSendNum = 0; NSendNum <= 29; NSendNum++)
	{
		g_IDM_TX_ODM_Data[NSendNum] = 0;
	}
}
//---------------------------------------------------------------//
void l_Self_Detect(void)
{

	unsigned char l_ucTemp;
	unsigned char l_uci;
	//unsigned char SelfCheckFlag;
	unsigned char SelfCheckStep;
	unsigned char r_bw, r_sw, r_gw;
	//l_ucTemp = 0;
	l_uci = 5;
	SelfCheckFlag = 0;
	SelfCheckStep = 0;
	do
	{
		l_ucTemp = 0;
		while (l_uci != 0)
		{
			if (KEY_SW2 == 0) //(IO_SHORTTIME == 0)
			{
				if (SelfCheckFlag != 0) //����
				{
					l_ucTemp++;
					SelfCheckFlag = 0;
				}
			}
			else
			{
				if (SelfCheckFlag == 0)
				{
					l_ucTemp++;
					SelfCheckFlag = 1;
				}
			}
			l_uci--;
		}
		l_uci = 5;
	} while (l_ucTemp != 0);

	while (SelfCheckFlag != 0) //�����Լ�
	{
		/*
		    	 if((!f_first_ad)||(!f_Self_first_ad))
           {
		            t_Self_Times = t_20ms;
temp:		        IWDT_Clear();
				        l_Pannel_Self_DataTx();//�Լ�ʱ��巢������
				        Pannel_Comm_Deal();//������Ҫ�ж�PT100��NTC1�����ݡ�
				        if((unsigned char)(t_20ms-t_Self_Times)>=25)//ÿ2��������һ�����ݸ��װ塣
				        {
				        	 bSelfUartSendStartFlag=1;
				        	 t_Self_Times = t_20ms;		        	
				        }
				        if((!f_first_ad)||(!f_Self_first_ad)) //����صĵ��ѹ��PT100���ݶ��յ����������ж�
				        {
				        	 goto temp;
				        }
				    }
				    */
		/*
		        if(r16_ldad>1010||r16_ldad<15) //�䶳����������
						{
							if(!f_ld_sensor_err)//��������䶳����������,����һ��,�Ժ��ٽ���
							{
							  f_ld_sensor_err = 1;
							  f_stop_alarm = 0;
							}
						}
						//�����ж���ش��������ϣ�		
						//���������й���ʱ�����ٽ����Լ죬��ʾ���ϴ�������¶ȡ�
		        if(r_lcad>250||r_lcad<5)
						{
							if(!f_lc_sensor_err)
							{
							   f_lc_sensor_err = 1;
							   f_stop_alarm = 0;
							}
						}
		        */

		switch (SelfCheckStep)
		{
		case 0:
			BuzzBi(); //BUZZ( 1 );
			SelfCheckStep++;
			t_Self_Times = t_20ms; //SelfCheck1000ms = 10; //based on 100ms
			break;
		case 1:
			r_led11 = 0xFF; //�䶳 ȫ��
			r_led12 = 0xFF;
			r_led13 = 0xFF;

			r_led21 = 0xFF; //��� ȫ��
			r_led22 = 0xFF;
			r_led23 = 0xFF;
			// LCD267 = 0xFF;  //all led on
			// LCD245 = 0xFF;
			// LCD1621[0] = 0xFF;
			// LCD1621[1] = 0xFF;
			break;
		case 2:
			r_led11 = 0x00; //table_led[DISP_NO]; //�䶳 ȫ��
			r_led12 = 0x00; //table_led[DISP_NO];
			r_led13 = 0x00; //table_led[DISP_NO];

			r_led21 = 0x00; //table_led[DISP_NO]; //��� ȫ��
			r_led22 = 0x00; //table_led[DISP_NO];
			r_led23 = 0x00; //table_led[DISP_NO];
			//LCD267 = 0;        //alL  led off
			//LCD245 = 0;
			//LCD1621[1] = 0x00;
			//INT_LED595( 0 );
			break;
		case 3:						//��ʱ 09��
			r_led21 = table_led[8]; // ������������� ��ʾ8
			//LCD1621[0] = 0x80;
			//LCD245 = LCDTABG_EPK[9];
			//LCD267 = LCDTABG_EPK[0];
			//LCD1621[1] = 0xFF;
			break;
		case 4: //˯�� 18��
			r_led21 = table_led[DISP_NO];
			r_led22 = table_led[8];
			//LCD1621[0] = 0x02;
			//LCD245 = LCDTABG_EPK[8];
			//LCD267 = LCDTABG_EPK[1];
			break;
		case 5: //���е� ���� 27��
			r_led22 = table_led[DISP_NO];
			r_led23 = table_led[8];
			//LCD245 = LCDTABG_EPK[7];
			//LCD267 = LCDTABG_EPK[2];
			break;
		case 6: //������ ��ʪ36��
			r_led23 = table_led[DISP_NO];
			r_led11 = table_led[8];
			//LCD1621[0] = 0x00;
			//LCD245 = LCDTABG_EPK[6];
			// LCD267 = LCDTABG_EPK[3];
			break;
		case 7: //ȫ�� ��ʱ 45��
			r_led11 = table_led[DISP_NO];
			r_led12 = table_led[8];
			//LCD1621[0] = 0x00;
			//LCD245 = LCDTABG_EPK[5];
			//LCD267 = LCDTABG_EPK[4];
			break;
		case 8: //��ʾ�𲽽�������ε���
			r_led12 = table_led[DISP_NO];
			r_led13 = table_led[8];
			//LCD267 = 0;        //al  led off
			//LCD245 = 0;
			//LCD1621[1] = 0x00;
			//INT_LED595( 0 );
			//IO_STEPMOTOR_A = 1;
			break;
		case 9:
			r_led13 = table_led[DISP_NO];
			f_lc_fan = 1; //��ط��
			//IO_STEPMOTOR_A = 0;
			//IO_STEPMOTOR_B = 1;
			break;
		case 10:
			f_lc_fan = 0;
			f_ln_fan = 1; //�䶳���
			//IO_STEPMOTOR_B = 0;
			//IO_STEPMOTOR_C = 1;
			break;
		case 11: //��˪�̵���
			f_ln_fan = 0;
			f_defrost = 1;

			//IO_STEPMOTOR_C = 0;
			//IO_STEPMOTOR_D = 1;
			break;
		case 12:		   //
			f_defrost = 0; //��˪
			f_nd_fan = 1;
			break;
		case 13: //��ͨ �䶳�¶���ʾ
			f_nd_fan = 0;
			f_nd_AcLowfan = 1;
			//IO_COMPRESSOR = 0;
			//IO_4WAYVALVE = 1;
			break;
		case 14: //������
			f_nd_AcLowfan = 0;
			f_lc_compressor = 1; //���ѹ��

			//�䶳ѹ�����
			//IO_4WAYVALVE = 0;
			///IO_NEGATIVE_ULTRA = 1;
			break;
		case 15:				 //����¶���ʾ
			f_lc_compressor = 0; //���ѹ��
			f_compressor = 1;	//�䶳ѹ��

			//IO_NEGATIVE_ULTRA = 0;
			//IO_OUTFAN = 1;
			break;
		case 16:
			f_compressor = 0; //��ؿ��ƴ������¶���ʾ
			if (r_lcwd < 41 || r_lcwd > 47)
			{
				r_bw = DISP_NO;
				r_sw = DISP_E;
				r_gw = 3;
			}
			else
			{
//				if (r_lcwd >= 38)
//				{
					r_bw = DISP_NO;
					r_sw = (uchar)((r_lcwd - 38) / 10);
					r_gw = (uchar)((r_lcwd - 38) % 10);
//				}
//				else
//				{
//					r_bw = DISP_FH; //-
//					r_sw = (uchar)((38 - r_lcwd) / 10);
//					r_gw = (uchar)((38 - r_lcwd) % 10);
//				}
			}
			r_led21 = table_led[r_bw];
			r_led22 = table_led[r_sw];
			r_led23 = table_led[r_gw];

			//IO_OUTFAN = 0;
			//IO_ELECHEAT = 1;
			break;
		case 17: //�䶳�¶���ʾ
			r_led21 = table_led[DISP_NO];
			r_led22 = table_led[DISP_NO];
			r_led23 = table_led[DISP_NO];
			if (r_ldsjwd < 197 || r_ldsjwd > 203)
			{
				r_bw = DISP_NO;
				r_sw = DISP_E;
				r_gw = 6;
			}
			else
			{
				if (r_ldsjwd >= 200)
				{
					r_bw = DISP_NO;
					r_sw = (uchar)((r_ldsjwd - 200) / 10);
					r_gw = (uchar)((r_ldsjwd - 200) % 10);
				}
				else
				{
					r_bw = DISP_FH; //-
					r_sw = (uchar)((200 - r_ldsjwd) / 10);
					r_gw = (uchar)((200 - r_ldsjwd) % 10);
				}
			}
			r_led11 = table_led[r_bw];
			r_led12 = table_led[r_sw];
			r_led13 = table_led[r_gw];

			//IO_ELECHEAT = 0;
			//LedAllOn2s = 0;
			break;
		case 18: //ʵ�ʵ�ѹ��ʾ
			r_led11 = table_led[DISP_NO];
			r_led12 = table_led[DISP_NO];
			r_led13 = table_led[DISP_NO];
			if (r_voltage / 100 == 0)
			{
				r_bw = DISP_NO;
			}
			else
			{
				r_bw = r_voltage / 100;
			}
			r_sw = r_voltage % 100 / 10;
			r_gw = r_voltage % 10;

			r_led21 = table_led[r_bw];
			r_led22 = table_led[r_sw];
			r_led23 = table_led[r_gw];
			//BuzzBiBiBi();//BuzzBi();
			//FLAG_CON.Bit.b2 = 1;    //@140610
			//SETWIND( 3, FANWHIGH ); //@140610
			break; //@140610
		case 19:
			r_led11 = table_led[DISP_NO]; //�䶳 ȫ��
			r_led12 = table_led[DISP_NO];
			r_led13 = table_led[DISP_NO];

			r_led21 = table_led[DISP_NO]; //��� ȫ��
			r_led22 = table_led[DISP_NO];
			r_led23 = table_led[DISP_NO];
			SelfCheckStep++; //@140610
			if (r_lcwd < 41 || r_lcwd > 47 || r_ldsjwd < 197 || r_ldsjwd > 203)
			{
				;
			}
			else
			{
				SelfCheckNoErrFlag = 1;
			} //@140610
			//FLAG_CON.Bit.b2 = 0;    //@140610
			//CLOSEFAN();                 //@140610
			//BUZZ( 1 );
			//SelfCheckStep++;
			//LedAllOn2s = 4;
			break;
		case 20:
			//l_uci=5;
			break;
		default: //quit
				 //r_led11 = table_led[DISP_NO];
				 //r_led12 = table_led[DISP_NO];
				 //r_led13 = table_led[DISP_NO];
			if (r_voltage / 100 == 0)
			{
				r_bw = DISP_NO;
			}
			else
			{
				r_bw = r_voltage / 100;
			}
			r_sw = r_voltage % 100 / 10;
			r_gw = r_voltage % 10;

			r_led21 = table_led[r_bw];
			r_led22 = table_led[r_sw];
			r_led23 = table_led[r_gw];

			//r_led21 = table_led[DISP_NO];
			//r_led22 = table_led[DISP_NO];
			//r_led23 = table_led[DISP_NO];
			if (r_ldsjwd < 197 || r_ldsjwd > 203)
			{
				r_bw = DISP_NO;
				r_sw = DISP_E;
				r_gw = 6;
			}
			else
			{
				if (r_ldsjwd >= 200)
				{
					r_bw = DISP_NO;
					r_sw = (uchar)((r_ldsjwd - 200) / 10);
					r_gw = (uchar)((r_ldsjwd - 200) % 10);
				}
				else
				{
					r_bw = DISP_FH; //-
					r_sw = (uchar)((200 - r_ldsjwd) / 10);
					r_gw = (uchar)((200 - r_ldsjwd) % 10);
				}
			}
			r_led11 = table_led[r_bw];
			r_led12 = table_led[r_sw];
			r_led13 = table_led[r_gw];
			//SelfCheckFlag = 0; @20190228 �е�У׼����������˳��Լ�
			break;
		}
		BuzzPrg();
		IWDT_Clear();
		if ((uchar)(t_20ms - t_Self_Times) >= 50)
		{
			bSelfUartSendStartFlag = 1;
			t_Self_Times = t_20ms;
			if (SelfCheckStep <= 20)
			{
				SelfCheckStep++;
			}
			l_Pannel_Self_DataTx();
			IWDT_Clear();
			Pannel_Comm_Deal();
			IWDT_Clear();
		}
		//if ( SelfCheck1000ms == 0 )
		//{
		//    SelfCheck1000ms = 10;
		//    SelfCheckStep++;
		// }
		//BUZZ1();
		//g_Txd_Time=250; //�Լ������500ms�����·����ݡ�
		DataToLed();
		if (SelfCheckStep >= 21)
		{
			do //SW1�˳� @20190228 CFJ
			{
				l_ucTemp = 0;
				while (l_uci != 0)
				{
					if (KEY_SW1 == 0) //(IO_SHORTTIME == 0)
					{
						//if ( SelfCheckFlag != 0 ) //����
						//{
						//  l_ucTemp++;
						//SelfCheckFlag = 0;
						//}
					}
					else
					{
						if (SelfCheckFlag == 1)
						{
							l_ucTemp++;
							SelfCheckFlag = 0;
						}
					}
					l_uci--;
				}
				l_uci = 5;
			} while (l_ucTemp != 0);
		}
	}
}
//--------------------------------------------------------------//
void l_Self_Send_Data_Build(void)
{
	unsigned char NDegree, Sum, Temp;
	l_ClrIDMSendBuf();
	Sum = 0;
	g_IDM_TX_ODM_Data[0] = 0xAA;

	g_IDM_TX_ODM_Data[1] = 0x55; //  ͷ��4���ֽ�

	g_IDM_TX_ODM_Data[2] = 0xE7;

	g_IDM_TX_ODM_Data[3] = 0x18;
	//----------------------------------//
	g_IDM_TX_ODM_Data[4] = 0xFF; //��ʾ���ַ

	g_IDM_TX_ODM_Data[5] = 0xB0; //I/O���ַ

	g_IDM_TX_ODM_Data[6] = 0x00; //����֡����
	//-----ϵͳ�趨���¶�-------//

	if (r_lczt >= 38) //����趨�¶� @20181130 CFJ
	{				  //���ڵ���0��
		Temp = (r_lczt - 38);
		g_IDM_TX_ODM_Data[7] = Temp;
	}
	else
	{ //0������
		Temp = (38 - r_lczt);
		g_IDM_TX_ODM_Data[7] = Temp;
		g_IDM_TX_ODM_Data[7] |= 0x80;
	}
#ifdef Need_ld
	if (r_ldzt >= 200) //�䶳�趨�¶�
	{				   //���ڵ���0��
		Temp = (r_ldzt - 200);
		g_IDM_TX_ODM_Data[8] = Temp;
	}
	else
	{ //0������
		Temp = (200 - r_ldzt);
		g_IDM_TX_ODM_Data[8] = Temp;
		g_IDM_TX_ODM_Data[8] |= 0x80;
	}
#else
	g_IDM_TX_ODM_Data[8] = r_ldzt;
#endif
	g_IDM_TX_ODM_Data[9] = 0x00; //Ԥ��δ��

	if (f_lc_fan) //��ط��
	{
		g_IDM_TX_ODM_Data[10] |= 0x01;
	}

	if (f_ln_fan) //�䶳���
	{
		g_IDM_TX_ODM_Data[10] |= 0x02;
	}

	if (f_defrost) //��˪
	{
		g_IDM_TX_ODM_Data[10] |= 0x04;
	}

	if (f_nd_fan) //�ڷ�� �ſ��ع��ϵķ��
	{
		g_IDM_TX_ODM_Data[10] |= 0x08;
	}
	if (f_nd_AcLowfan)
	{
		g_IDM_TX_ODM_Data[10] |= 0x10; //@20190222 CFJ
	}
	if (f_lc_compressor) //���ѹ��
	{
		g_IDM_TX_ODM_Data[11] |= 0x01;
	}

	if (f_compressor) //�䶳ѹ��
	{
		g_IDM_TX_ODM_Data[11] |= 0x02;
	}

	g_IDM_TX_ODM_Data[12] = 0x00; //12V�������δ�õ����ڴ˷�0.

	if (f_long_range_alarm) //Զ�̱��� @20181130  CFJ
	{
		g_IDM_TX_ODM_Data[13] |= 0;
	}
	else
	{
		g_IDM_TX_ODM_Data[13] |= 0x01; //��Ϊ�װ��Զ�̱����ļ̵����ǳ��յġ���ҵ����ΰҪ���й���ʱ����0 ���޷� 1
	}

	g_IDM_TX_ODM_Data[14] = 0x00; //Ԥ��
	/*  ���Э��λ�ĸ��� ���±������ã��Ѻ���ҵ����ͨ�� @20181130 CFJ
    if(f_ld_high_buzz)  //Զ�̱���      
    {
        g_IDM_TX_ODM_Data[15]|=0x01;
    }    
    if(f_ld_low_buzz)
    {
    	  g_IDM_TX_ODM_Data[15]|=0x02;
    } 
    */
	if (f_power_buzz)
	{
		g_IDM_TX_ODM_Data[15] |= 0x00; //0x04; //�ϵ籨��
	}
	/* @20181130 CFJ
    if(f_battery)
    {
    	 g_IDM_TX_ODM_Data[15]|=0x08; //��ص�����
    }
    */
	if (f_lc_sensor_err)
	{
		g_IDM_TX_ODM_Data[15] |= 0x00; //0x40;//g_IDM_TX_ODM_Data[15]|=0x80; //��ҵ��Ҫ��NTC2��ΪNTC1��ش��������� @20181221 CFJ
	}

	//  g_IDM_TX_ODM_Data[16]|=0x00; ����δ��
//	if (f_ld_sensor_err)
//	{
//		g_IDM_TX_ODM_Data[17] |= 0x00; //0x01; //�䶳����������
//	}
	if (f_hw_sensor_err)
	{
		g_IDM_TX_ODM_Data[17] |= 0x00; //0x04; //���´���������
	}
	if (f_hw_high38)
	{
		g_IDM_TX_ODM_Data[17] |= 0x00; //0x10; //���¹���
	}

	if (f_ld_high_buzz) //if(f_ld_high) @20181130 CFJ
	{
		g_IDM_TX_ODM_Data[18] |= 0x00; //0x04; //�䶳���±��� v1.3Э��
	}
	if (f_ld_low_buzz) //if(f_ld_low) @20181130 CFJ
	{
		g_IDM_TX_ODM_Data[18] |= 0x00; //0x08; //�䶳���±���
	}

	if (f_lc_high_buzz) //if(f_lc_high) @20181130 CFJ
	{
		g_IDM_TX_ODM_Data[18] |= 0x00; //0x01; //��ظ��±���
	}

	if (f_lc_low_buzz) //if(f_lc_low) @20181130 CFJ
	{
		g_IDM_TX_ODM_Data[18] |= 0x00; //0x02; //��ص��±���
	}

	if (f_buzz_alarm)
	{
		g_IDM_TX_ODM_Data[19] = 0x00; //0x01;  //��������
	}

	g_IDM_TX_ODM_Data[25] = 0xFF; //ѹ����1

	g_IDM_TX_ODM_Data[26] = 0xFF; //ѹ����2

	/*
    if(!g_Sys_Erflag0_Comm)  //�޹��ϣ����ϵ�Ĭ�Ͽ���
    {
        g_IDM_TX_ODM_Data[2] |=0x80;
    }
    if(f_lc_compressor)  //���ѹ��
    {
        g_IDM_TX_ODM_Data[2] |=0x40;
    } 
    if(f_compressor)  //�䶳ѹ��
    {
        g_IDM_TX_ODM_Data[2] |=0x20;
    
    }
    if(f_lc_fan)  //��ط��
    {
        g_IDM_TX_ODM_Data[2] |=0x10;    
    }
    
    if(f_ln_fan)  //�䶳���
    {
        g_IDM_TX_ODM_Data[2] |=0x08;    
    }
    
    if(f_defrost)  //��˪
    {
        g_IDM_TX_ODM_Data[2] |=0x04;    
    }
    
    if(f_nd_fan)  //�ڷ��
    {
        g_IDM_TX_ODM_Data[2] |=0x02;    
    }
    
    if(g_Compressor_Frequency_Flag1)  //ѹ��1��Ƶ/��Ƶѡ���־
    {
        g_IDM_TX_ODM_Data[3] |=0x80;  //��Ƶ
    }
    else
    {   
         g_IDM_TX_ODM_Data[3]=0xFF;
    }
    if(g_Compressor_Frequency_Flag2)  //ѹ��2��Ƶ/��Ƶѡ���־
    {
        g_IDM_TX_ODM_Data[4] |=0x80;  //��Ƶ    
    }
    else
    {
        g_IDM_TX_ODM_Data[4]=0xFF;
    }
    
    if(f_long_range_alarm)  //Զ�̱���
    {
        g_IDM_TX_ODM_Data[5]|=0x80;
    }
    //����� Ĭ����
     g_IDM_TX_ODM_Data[5]|=0x40;
    
    if(f_buzz_alarm)
    {
        g_IDM_TX_ODM_Data[5]|=0x20;
    }
    
    //12v ���� 4·�����  D4 D3 D2 D1
    
    g_IDM_TX_ODM_Data[5]|=0x00;
    
    //Ԥ��
  
    g_IDM_TX_ODM_Data[6] = 0x00;
    g_IDM_TX_ODM_Data[7] = 0x00;
    
    //У����ֽ�
    g_IDM_TX_ODM_Data[8]=0;  
    */
	for (NDegree = 4; NDegree <= 28; NDegree++)
	{
		Sum += g_IDM_TX_ODM_Data[NDegree];
	}
	g_IDM_TX_ODM_Data[29] = Sum;
	g_UART1_TXD_Counter = 0;
	RS_485_STATUS_SET_HIGH; //������Ч
	g_Rec_Status_Flag = 0;
	UART_ITConfig(UART4, UART_IT_TB, ENABLE);//ʹ�ܷ����ж�
}

void l_Pannel_Self_DataTx(void)
{
	if (bSelfUartSendStartFlag) //@20190215 CFJ
	{
		bSelfUartSendStartFlag = 0;
		l_Self_Send_Data_Build();
	}
}
