/*
 控制.c文件
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
// 100526  zyj 风机开内胆风机停转
// 140915 累计6小时后停机13分钟
// 141016 累计6小时后停机18分钟，等压机停机执行
//#include <hidef.h> /* for EnableInterrupts macro */
//#include "derivative.h" /* include peripheral declarations */
#include "Disp.H"
#include "logicsub.h"
//#include "pdl_header.h"
#include "math.h"
#include "Program_Cfg.h" //程序功能配置，比如禁止蜂鸣器、控制冷藏温度分辨率
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

#define HEATONAD 2130		 //2.5℃
#define HEATOFFAD 2110		 //3.0℃
#define LCCOMPRTSTOREAD 1869 //7.8℃

//最后两个，冷藏化霜传感器显示温度(低于某个温度化霜)，冷藏冷凝传感器(判断压缩机脏堵)
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

	if (f_2Ms_Flag) //2ms标志  //CFJ
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
                    //Mfs_Uart_EnableFunc(&UART1, UartTx);    //使能发送接收
                    //Mfs_Uart_DisableFunc(&UART1, UartRx); //接受使能无效
                 }            
             }
             */
		if (g_Bus_Error_Time >= 30000) //转为接受状态，1min内未收到数据，则报故障
		{
			g_Sys_Erflag0_Comm = 1; //通讯错误 @20181025 CFJ
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
//********判断上电后蜂鸣器延迟报警程序**********
void PowerUpBuzzDelaylc(void)
{
	if (f_Powerupdelaylc)
	{
		return;
	}
	if (f_onemin)
	{//在for循环中判断，可以理解为半分钟翻转一次u8_BuzzDlyMinlc一分钟加1
		u8_BuzzDlyMinlc++;
		if (u8_BuzzDlyMinlc >= 60)
		{
			u8_BuzzDlyMinlc = 0;
			u8_BuzzDlyHourlc++;
		}
	}

	if (u8_BuzzDlyHourlc >= u8_lc_bjycsj) //上电报警计时大于设定的延时时间
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
//********************蜂鸣器程序****************
//**********************************************
void BuzzPrg(void)
{
	if (f_bi) //叫一声
	{
		if ((uchar)(t_onems - t_buzz) >= 200) //200ms
		{
			goto Tfmq;
		}
	}
	else if (f_bibibi) //连续叫,叫100ms,停50ms
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
	f_bibibi = 0;//连续叫
Tfmq:
	f_bi = 0;//叫一声
	PWM_Stop();
	//TPM1SC = 0x00;    @20181008 CFJ
	//TPM1C3SC = 0x00;   @20181008 CFJ
}
//********************蜂鸣器响程序**************
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
/*********************AD转换程序*********************/
/****************************************************/

/**
 * @brief ad转换为温度
 *
 * 
 *
 * @param  Ad   12位ad
 * @retval 实际温度+38
 */
float CheckLcTable_12b(unsigned int Ad)
{
    unsigned int i = 0;
    float Temp;
    if ((float)Ad >= table_lc_12b[0][0])
    {
        return table_lc_12b[0][1] + 38; //-30度
    }
    else if ((float)Ad <= table_lc_12b[18][0])
    {
        return table_lc_12b[18][1] + 38; //60度
    }
    else
    {
        while ((float)Ad < table_lc_12b[i][0])
            i++;
        Temp = 38 + table_lc_12b[i - 1][1] + 5 * (Ad - table_lc_12b[i - 1][0]) / (table_lc_12b[i][0] - table_lc_12b[i - 1][0]); //浮点型温度
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
    uchar DeltaAD;  //两次ad差值

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
函数名: ADCRead
描  述: ADC采样数据读取(16次采样平均值)(软件控制)
输入值: 无
输出值: 无
返回值: 平均值
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
函数名: ADCSetCHS
描  述: ADC设置采样信道
输入值: 无
输出值: 无
返回值: 平均值
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
函数名: ADC2Temper
描  述: ADC转换为温度值
输入值: 无
输出值: 无
返回值: 平均值
**********************************************************/
unsigned short ADC2Temper_3840(uint16_t data)
{
    unsigned short  i,temp;

   for(i=0;i<161;i++)
    {
        if((table_ld_3840[i] > data && table_ld_3840[i+1] < data)
           || (table_ld_3840[i] == data))
        {
            temp = ((table_ld_3840[i] - data) * 10) / table_ld_3840_s[i];
			return((i - 40) * 10 + temp + 510);//上偏510，保证不出负数即-40度对应的是110 
        }
    }

	return 1740;//有错误时，没有这个AD值，返回1740,实际为(1740-510)/10度
	//拿到实际温度的思路，如果返回值<510是个负数，否则就是个正数
	//例如当返回值为400时，对应温度应为-(510-400)/10度(-11度),当返回值为600时，对应温度为+(600-510)/10度(+9度)
}


/****************************************************/
/********************显示处理************************/
/****************************************************/
void RuleForLdDisp(void);
void RuleForLcDisp(void);
void LdDisp(void);
void LcDisp(void);
void AlarmAndLockLed(void);
/**********************************************/
void DealDispData(void)
{
	RuleForLdDisp();//冷冻数码管显示逻辑

#if (LC_DISP_RESOLUTION_0D1 || LC_RESOLUTION_0D1) //显示分辨率0.1度
	RuleForLcDisp_0D1();						  //显示温度人工干预，不采用实际温度  (分辨率1度)
#else
	RuleForLcDisp(); //显示温度人工干预，不采用实际温度   (分辨率1度)
#endif
	LcDisp(); //把每个数码管显示内容确定下来
	LdDisp();//这个地方要改成只显示湿度，不让他搞别的事情
	AlarmAndLockLed();//两个LED灯的处理
}
/****************************************************/
/********************冷冻显示规则********************/
/****************************************************/
void RuleForLdDisp(void)
{
	if (!f_first_ad) //没有完成1次AD采样就返回
	{
		return;
	}
#ifdef Need_Ld
	
	if (!f_ld_first_disp)
	{
		f_ld_first_disp = 1;//没有故障的话，就一直这样了
		r_ldgzwd = r_ldsjwd;
		t_ld_rule = t_halfsec;
	}
	//实际温度>设定温度+1或者实际温度<设定温度
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
	//老王说，这个地方是第二次显示矫正，可能在这个区间存在这个误差实际显示温度就是r_ldxswd
	if ((r_hwsjwd >= 67) && (r_ldzt <= 170)) //当环境温度大于29℃且设定温度小于-30℃
	{
		r_ldxswd = r_ldxswd - 2; //显示值-2
	}
	else if (r_ldzt <= 170)
	{
		r_ldxswd = r_ldxswd - 1; //显示值-1
	}
  #else//显示湿度值
  if (!f_ld_first_disp)
  {//系统第一次启动
		f_ld_first_disp = 1;//没有故障的话，就一直这样了
		t_ld_rule = t_halfsec;
  }
  if(LcHuaShuang_Flag){//当前正在化霜
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
/********************冷藏显示规则********************/
/****************************************************/
//void RuleForLcDisp(void)
//{
//	if (!f_first_ad) //还未采样温度
//	{
//		return;
//	}
//	if (!f_lc_first_disp) //首次显示
//	{
//		f_lc_first_disp = 1;
//		r_lcgzwd = r_lcwd;	 //显示实际温度
//		t_lc_rule = t_halfsec; //记下显示刷新时间点
//		u8_LcRule = t_tens;
//	}
//	else if ((r_lcwd > (uchar)(r_lczt + 2)) || (r_lcwd < (uchar)(r_lczt - 1))) //实际温度高于设置温度超过2度  或者  低于设置温度超过1度
//	{
//		{
//			//u8_LcRule = t_tens; @20181130 CFJ
//			if ((uchar)(t_halfsec - t_lc_rule) >= 120) //60秒 @20181130 CFJ
//			{
//				t_lc_rule = t_halfsec;
//				if (r_lcwd > r_lcgzwd) //显示温度<实际温度
//				{
//					r_lcgzwd++; //显示温度++
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
//			if ((uchar)(t_halfsec - t_lc_rule) >= 120) //60S钟 @20181120
//			{
//				t_lc_rule = t_halfsec;
//				if (r_lcgzwd > r_lczt) //显示温度>实际温度
//				{
//					r_lcgzwd--; //显示温度--
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
/********************冷藏显示规则（分辨率0.1度）********************/
/****************************************************/
void RuleForLcDisp_0D1(void)
{
	if (!f_first_ad) //还未采样温度
	{
		return;
	}
	if (!f_lc_first_disp) //首次显示
	{
		f_lc_first_disp = 1;
		r_lcgzwd_float = r_lcwd_float; //显示实际温度
		t_lc_rule = t_halfsec;		   //记下显示刷新时间点
		u8_LcRule = t_tens;
	}
	else   //渐变到实际温度
	{
        if ((t_halfsec - t_lc_rule) >= 12) //6秒
        {
            t_lc_rule = t_halfsec;

            if (fabs(r_lcwd_float - r_lcgzwd_float) < 0.1)
                r_lcgzwd_float = r_lcwd_float;
            else
            {
                if (r_lcwd_float > r_lcgzwd_float) //显示温度<实际温度
                {
                    r_lcgzwd_float += 0.1; //显示温度++
                }
                else if (r_lcwd_float < r_lcgzwd_float)
                {
                    r_lcgzwd_float -= 0.1;
                }
            }
        }
	}
	
	r_lcxswd_float = r_lcgzwd_float;
	r_lcgzwd = (unsigned char)floor(r_lcgzwd_float); //有些地方还需要用到r_lcgzwd,比如压缩机连续运行保护的功能需要判断r_lcgzwd是否等于设置温度，所以取r_lcgzwd_float得整数部分给r_lcgzwd
}

/****************************************************/
/********************冷藏显示规则（分辨率0.1度）********************/
/****************************************************/
//void RuleForLcDisp_0D2(void)
//{
//	if (!f_first_ad) //还未采样温度
//	{
//		return;
//	}
//	if (!f_lc_first_disp) //首次显示
//	{
//		f_lc_first_disp = 1;
//		r_lcgzwd_float = r_lcwd_float; //显示实际温度
//		t_lc_rule = t_halfsec;		   //记下显示刷新时间点
//		u8_LcRule = t_tens;
//	}
//	else if ((r_lcwd_float > (r_lczt_float + 2)) || (r_lcwd_float < (r_lczt_float - 1))) //实际温度高于设置温度超过2度  或者  低于设置温度超过1度
//	{
//		{
//			//u8_LcRule = t_tens; @20181130 CFJ
//			if ((uchar)(t_halfsec - t_lc_rule) >= 12) //6秒
//			{
//				t_lc_rule = t_halfsec;

//				if (fabs(r_lcwd_float - r_lcgzwd_float) < 0.1)
//					r_lcgzwd_float = r_lcwd_float;
//				else
//				{
//					if (r_lcwd_float > r_lcgzwd_float) //显示温度<实际温度
//					{
//						r_lcgzwd_float += 0.1; //显示温度++
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
//		if (r_lcgzwd_float != r_lczt_float) //拟显示温度 不等于 设置温度
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
//					if (r_lcgzwd_float > r_lczt_float) //显示温度>实际温度
//					{
//						r_lcgzwd_float -= 0.1; //显示温度--
//					}
//					else if (r_lcgzwd_float < r_lczt_float)
//					{
//						r_lcgzwd_float += 0.1;
//					}
//				}
//			}
//		}
//	}
//	r_lcxswd_float = r_lcgzwd_float;//最终显示使用的值
//	r_lcgzwd = (unsigned char)floor(r_lcgzwd_float); //有些地方还需要用到r_lcgzwd,比如压缩机连续运行保护的功能需要判断r_lcgzwd是否等于设置温度，所以取r_lcgzwd_float得整数部分给r_lcgzwd
//}

/****************************************************/
/********************冷冻显示处理********************/
/****************************************************/
void LdDisp(void)
{
	unsigned char r_bw, r_sw, r_gw;

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
			r_bw = DISP_NO; //不显示
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
		if(r_set_state == SET_LD_HIGH)//可能到100
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
	else if (r_set_state == LD_CHK_VOL) //显示测量的电压
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
	else if (r_set_state == LD_CHK_HW) //环温传感器
	{
		if (f_hw_sensor_err) //环温传感器故障显示E0
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
	else if (r_set_state == SET_LD_WDJZ) //湿度校准
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
	else if (r_set_state == SET_LD_BJYCSJ) //冷冻压机延迟时间
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
	else if (r_set_state == SET_LDYJ) //按键设置压机开停
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
			break;
		}
	}
	else if (f_ld_DispErrs) //显示高温/低温/冷冻SNR故障
	{
		if (f_ld_high && f_ld_high_disp)
		{ //湿度高 E9
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
		{//湿度 低E10
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
//	else if (f_key_defrost && f_key_defrost_disp) //除霜时,显示3sDF,显示5秒温度
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
	else if (f_power_off && f_power_off_disp) //加
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
	else if (g_Sys_Erflag0_Comm) //@20181130 CFJ 增加通讯故障标志
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
			if(r_ldxswd>=100){//我自己添加的湿度，不搞偏移
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
/********************冷藏显示处理********************/
/****************************************************/
void LcDisp(void) //把每个数码管显示内容确定下来
{
	unsigned char r_bw, r_sw, r_gw;

	flag_Now_Disp_LCWD = 0; //

	if (f_test_alarm || f_First_PowerOnFlag) //@20181120 CFJ    首次上电  或者  摁了“报警测试”按键
	{
		r_led21 = 0xff;
		r_led22 = 0xff;
		r_led23 = 0xff;
		return;
	}
	if (!f_first_ad)
	{
		r_led21 = table_led[DISP_NO]; //笔段全灭
		r_led22 = table_led[DISP_NO];
		r_led23 = table_led[DISP_NO];
		return;
	}
	if (r_set_state == SET_LC_L0) //  LO 闪烁
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
	else if (r_set_state == SET_LC_L1) // L1 闪烁
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
	else if (r_set_state == SET_LC_L2) // L2 闪烁
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
	else if (r_set_state == SET_LC_L3) // L3 闪烁
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
	else if (r_set_state == SET_LC_L4) // L4 闪烁
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
	else if (r_set_state == SET_LC_L5) // L5 闪烁
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
	else if (r_set_state == SET_LC_L6) // L6 闪烁
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
	else if (r_set_state == SET_LC_L7) // L7 闪烁
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
	else if (r_set_state == SET_LC_L8) // L8 闪烁
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
	else if (r_set_state == SET_LC || r_set_state == SET_LC_HIGH || r_set_state == SET_LC_LOW) // 设置冷藏设置温度、高温报警值、低温报警值
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
	else if (r_set_state == LC_CHK_HW) //环温错误信息  环境实际温度
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
	else if (r_set_state == LC_CHK_LCKZ) //E3 冷藏控制传感器实际温度
	{
		flag_Now_Disp_LCWD = 1;
		if (f_lc_sensor_err)
		{//如果冷藏控制传感器有问题
			r_bw = DISP_NO;
			r_sw = DISP_E;
			r_gw = 3;
		}
		if (r_lcwd_float_c >= 38.0) //温度为正  分辨率为0.1
			{
				r_bw = (unsigned int)((r_lcwd_float_c - 38.0) * 10 + 0.5) / 100;		//+0.5是为了float强制转换整数时，四舍五入
				r_sw = ((unsigned int)((r_lcwd_float_c - 38.0) * 10 + 0.5) % 100) / 10; //+0.5是为了float强制转换整数时，四舍五入
				r_gw = (unsigned int)((r_lcwd_float_c - 38.0) * 10 + 0.5) % 10;
				flag_Disp_LCWD_D = 1;
				if (r_bw == 0)
				{
					r_bw = DISP_NO; //百位的0不显示
				}
			}
			else if (r_lcwd_float_c <= 28.0) //温度<= -10度  //只显示整数部分
			{
				r_bw = DISP_FH; //-
				r_sw = ((unsigned int)(38 - r_lcwd_float_c) % 100) / 10;
				r_gw = (unsigned int)(38 - r_lcwd_float_c) % 10;
				flag_Disp_LCWD_D = 0;
			}
			else //   0  ~ -9.9度   显示1位小数
			{
				uchar temp_lc_x10 = (uchar)((38.0 - r_lcwd_float_c) * 10 + 0.5);
				r_bw = DISP_FH; //-
				if (temp_lc_x10 >= 100)
					temp_lc_x10 = 99;
				r_sw = temp_lc_x10 / 10; //+0.5是为了float强制转换整数时，四舍五入
				r_gw = temp_lc_x10 % 10;
				flag_Disp_LCWD_D = 1;
			}
			if (flag_Now_Disp_LCWD)
			{
				if (flag_Disp_LCWD_D)
					r_led22 |= 0x20; //显示小数点
				else
					r_led22 &= ~0x20; //关闭小数点
			}
			else
				r_led22 &= ~0x20; //关闭小数点
	}
	else if (r_set_state == LC_CHK_LCHS) //冷藏控制传感器实际温度
	{
		if (f_lcHs_sensor_err)//E2 如果冷藏化霜传感器有问题
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
	else if (r_set_state == LC_CHK_LCLN) //冷藏控制传感器实际温度
	{
		if (f_lcLn_sensor_err) //E6 如果冷藏化霜传感器有问题
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
		if (r_lcwdjzx >= 10)
		{
			r_bw = DISP_NO;
			r_sw = DISP_NO;
			r_gw = r_lcwdjzx - 10;
		}
		else
		{
			r_bw = DISP_NO;
			r_sw = DISP_FH;
			r_gw = 10 - r_lcwdjzx;
		}
	}
	else if (r_set_state == SET_LC_XSWDJZ)
	{
		if (r_lcxswdjzx >= 10)
		{
			r_bw = DISP_NO;
			r_sw = DISP_NO;
			r_gw = r_lcxswdjzx - 10;
		}
		else
		{
			r_bw = DISP_NO;
			r_sw = DISP_FH;
			r_gw = 10 - r_lcxswdjzx;
		}
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
	else if (f_lc_DispErrs) //显示错误代码
	{
		if (f_battery && f_BatteryErrDisp)
		{//电池 E5
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
		{//冷藏温度高 E9
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
		{//冷藏温度低 E10
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
		{ //环温传感器故障 E0
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
		{ // 环温高 E14
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
		{//冷藏显示传感器 E1
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
		{//化霜传感器 E2
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
		{//冷藏控制传感器 E3
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
		{//冷凝传感器故障 E6
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
	else if (f_power_off && f_power_off_disp) //电量很低，关闭显示
	{
		if ((uchar)(t_halfsec - t_lc_err_disp) >= 60) //30s
		{
			t_lc_err_disp = t_halfsec;
			t_ld_err_disp = t_halfsec; //加 2010 0506
			f_power_off_disp = 0;
		}
		r_bw = DISP_NO;
		r_sw = DISP_NO;
		r_gw = DISP_NO;
	}
	else if (f_lc_sensor_err) //冷藏控制传感器错误  E3
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

	else //显示实际冷藏温度
	{
		flag_Now_Disp_LCWD = 1;
		if ((uchar)(t_halfsec - t_lc_err_disp) >= 10) //5s
		{
			t_lc_err_disp = t_halfsec;
			f_power_off_disp = 1; //低电量时关闭显示    在低电量时，关闭30s,显示5s
		}

#if (LC_DISP_RESOLUTION_0D1 || LC_RESOLUTION_0D1) //冷藏显示温度分辨率为0.1度
		{

			if (r_lcxswd_float >= 38.0) //温度为正  分辨率为0.1
			{
				r_bw = (unsigned int)((r_lcxswd_float - 38.0) * 10 + 0.5) / 100;		//+0.5是为了float强制转换整数时，四舍五入
				r_sw = ((unsigned int)((r_lcxswd_float - 38.0) * 10 + 0.5) % 100) / 10; //+0.5是为了float强制转换整数时，四舍五入
				r_gw = (unsigned int)((r_lcxswd_float - 38.0) * 10 + 0.5) % 10;
				flag_Disp_LCWD_D = 1;
				if (r_bw == 0)
				{
					r_bw = DISP_NO; //百位的0不显示
				}
			}
			else if (r_lcxswd_float <= 28.0) //温度<= -10度  //只显示整数部分
			{
				r_bw = DISP_FH; //-
				r_sw = ((unsigned int)(38 - r_lcxswd_float) % 100) / 10;
				r_gw = (unsigned int)(38 - r_lcxswd_float) % 10;
				flag_Disp_LCWD_D = 0;
			}
			else //   0  ~ -9.9度   显示1位小数
			{
				uchar temp_lc_x10 = (uchar)((38.0 - r_lcxswd_float) * 10 + 0.5);
				r_bw = DISP_FH; //-
				if (temp_lc_x10 >= 100)
					temp_lc_x10 = 99;
				r_sw = temp_lc_x10 / 10; //+0.5是为了float强制转换整数时，四舍五入
				r_gw = temp_lc_x10 % 10;
				flag_Disp_LCWD_D = 1;
			}
		}
#else //冷藏显示温度分辨率为1度
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
			r_led22 |= 0x20; //显示小数点
		else
			r_led22 &= ~0x20; //关闭小数点
	}
	else
		r_led22 &= ~0x20; //关闭小数点
}
/****************************************************/
/*******************alarm and lock LED***************/
/****************************************************/
void AlarmAndLockLed(void)
{
	//锁定灯的处理
	if (f_lock || f_test_alarm)
	{
		r_led34 |= LOCK_LED_ON;
	}
	else
	{
		r_led34 &= LOCK_LED_OFF;
	}
	//报警灯的处理
	if (f_test_alarm || f_First_PowerOnFlag) //@20181120 CFJ
	{
		if ((uchar)(t_20ms - u8_test_alarm_time) >= 50) //50 1S  //25)      //0.5s @20181130 CFJ
		{
			u8_test_alarm_count++;
			u8_test_alarm_time = t_20ms;
			if (u8_test_alarm_count > 10) //12//否则底板会响7次，所以改为10 @20181130 CFJ                     //6s
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
	else if (f_hw_sensor_err || f_hw_high38 || f_ld_high || f_ld_low || f_power_off || f_lc_high || f_lc_low || f_lc_sensor_err || f_door_open||f_lcXs_sensor_err||f_lcHs_sensor_err||f_lcLn_sensor_err) ////////加开门报警wys11.03.19f_door_open
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
/*******************报警控制*************************/
/****************************************************/
void BuzzAlarmControl(void);
void LongRangeAlarm(void);
/****************************************************/
void AlarmControl(void)
{
	BuzzAlarmControl();//根据参数控制蜂鸣器叫
	LongRangeAlarm();//远程报警
}
/****************************************************/
void BuzzAlarmControl(void)
{
	if (f_test_alarm) //下面板子蜂鸣器叫
	{
		f_buzz_alarm = ON;
	}
	else if (f_stop_alarm) //如果有不要报警的要求,停止蜂鸣器报警
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
	else if (f_ld_high_buzz || f_ld_low_buzz || f_power_buzz  || f_lc_high_buzz || f_lc_low_buzz || f_lc_sensor_err||f_lcXs_sensor_err||f_lcHs_sensor_err||f_lcLn_sensor_err || f_dooropen_buzz) //||f_hw_sensor_err||f_battery)//20181101 TEST 加了一个环温传感器故障和电池电压低报警 CFJ /////wys11.03.19加开门报警
	{
		f_buzz_alarm = ON;
	}
	else
	{
		f_buzz_alarm = OFF;
	}
}
/********************************/
void LongRangeAlarm(void) //远程报警规格书没有远程报警的说明
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
/*******************接收完成数据处理*****************/
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
void JudgeLcXsErr(void);//冷藏显示传感器故障
void JudgeLcHsErr(void);//冷藏化霜传感器故障
void JudgeLcLnErr(void);//冷藏冷凝传感器故障
/*****************************************************/
void JudgeErrs(void)
{
	if (f_first_ad)
	{
	#ifdef Need_Ld
		//JudgeLdErr();
	#endif
		JudgeLcErr(); //冷藏控制传感器故障
		JudgeHwErr();
		JudgeLcXsErr();//冷藏显示传感器故障
		JudgeLcHsErr();//冷藏化霜传感器故障
		JudgeLcLnErr();//冷藏冷凝传感器故障
		JudgeHwHigh();
		
		JudgeLdHigh();
		JudgeLdLow();

		JudgeLcHigh();
		JudgeLcLow();
		JudgePowerOff();
		JudgeBattery();
		JudgeVoltageErr();
		JudgeLcLnHigh();//判断冷凝传感器是否超温
		JudgeLcHuaShuangLow();//判断是否到达化霜条件
		JudgeHumOver75();//是否化霜
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
//void JudgeLdErr(void) //冷冻传感器故障
//{
//	if (r16_ldad > 1010 || r16_ldad < 15)
//	{
//		if (!f_ld_sensor_err) //如果出现冷冻传感器故障,进入一次,以后不再进入
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
void JudgeLcErr(void) //冷藏控制传感器故障
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
void JudgeLcXsErr(void)//冷藏显示传感器故障
{
	if (r_lcXsad >=250 || r_lcXsad <=5)
	{
		if (!f_lcXs_sensor_err)
		{
			f_lcXs_sensor_err = 1;
			f_stop_alarm = 0;//有问题马上报警
		}
	}
	else if (f_lcXs_sensor_err)
	{
		f_lcXs_sensor_err = 0;

	}

}

void JudgeLcLnErr(void)//冷藏冷凝传感器故障
{
	if (r16_lcLengNingad > 3817 || r16_lcLengNingad < 72)
	{
		if (!f_lcLn_sensor_err)
		{
			f_lcLn_sensor_err = 1;
			f_stop_alarm = 0;//有问题马上报警
		}
	}
	else if (f_lcLn_sensor_err)
	{
		f_lcLn_sensor_err = 0;

	}

}


void JudgeLcHsErr(void)//冷藏化霜传感器故障
{
	if (r16_lcHuaShuangad > 3817 || r16_lcHuaShuangad < 72)
	{
		if (!f_lcHs_sensor_err)
		{
			f_lcHs_sensor_err = 1;
			f_stop_alarm = 0;//有问题马上报警
		}
	}
	else if (f_lcHs_sensor_err)
	{
		f_lcHs_sensor_err = 0;

	}

}

/**********************************/
void JudgeHwErr(void) //环温传感器故障
{
	if (r_hwad > 240 || r_hwad < 5)
	{
		if (!f_hw_sensor_err)
		{
			f_hw_sensor_err = 1;
			f_stop_alarm = 0;//有问题马上报警
		}
	}
	else
	{
		f_hw_sensor_err = 0;
	}
}

/***********判断湿度是不是超过75%，如果超了需要启用除湿功能*/
void JudgeHumOver751(void)
{
	//这里还要添加一个湿度传感器判断是不是正常
 	if(f_lc_sensor_err!=1){//冷藏控制传感器正常
	 	if((r_lcwd>=(2.5+38))&&(r_lcwd<=(7.5+38))){
			if(r_ldxswd>r_ldzt){//显示湿度>75,这里用显示湿度还是用实际湿度需要讨论//20200908          改成用设定值来判断湿度是否启停 75
				if(r_lcwd>=38+4){//冷藏控制传感器温度>=4度 
					f_deHum_and_lcTemp=1;
					/********************************************
					压缩机开机（如果停机时间不到三分钟，则满足三分钟后立即开机），内风机停机
					压缩机持续工作，内风机每间隔2分钟工作1分钟；
					********************************************/
				}else{//小于4度
					f_deHum_and_lcTemp=2;
					/********************************************
					压缩机开机（如果停机时间不到三分钟，则满足三分钟后立即开机），内风机停机
					压缩机持续工作，内风机每间隔2分钟工作20秒钟；
					********************************************/
				}
			
			}else if(r_ldxswd<=30){//箱内湿度<=30不除湿
				f_deHum_and_lcTemp=0;
			}
	 	}else{
	 		f_deHum_and_lcTemp=0;//不除湿
	 	}
 	}else{
 			f_deHum_and_lcTemp=0;//不除湿
 	}
}

//张国帆这个小老头要先取消掉除湿功能,他说是温度控制不住，怀疑跟除湿有关系
void JudgeHumOver75(void)
{
 	f_deHum_and_lcTemp=0;//不除湿
}

/*************判断环温，并根据环温驱动事件********************/
void JudgeHwHigh(void) //环温超38
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

	if (r_hwsjwd > (48+38) )//环温超48度，进入高温保护
	{
		if (!Protect_HigHw_Temp)
		{
			Protect_HigHw_Temp = 1;
			f_lc_fan = OFF;//关闭冷藏冷凝风机
			f_lc_compressor=OFF;//关闭冷藏压机
		}
	}else if(r_hwsjwd < (42+38) )
	{//低于42度退出高温保护
		Protect_HigHw_Temp=0;
	}
}
/***************************************************
判断冷藏的冷凝传感器温度，并根据温度动作
****************************************************/
void JudgeLcLnHigh(void)
{
	if(f_lcLn_sensor_err==0){//冷凝传感器没有故障
		//if(((r_lclnsjwd&0x80)>>7)!=1)
		if(r_lclnsjwd>0){
			if (r_lclnsjwd > LengNing_Protect_temp)//冷凝传感器度超温55度，进入高温保护
			{
				if (!Protect_HigLn_Temp)
				{
					Protect_HigLn_Temp = 1;
					f_lc_fan = OFF;//关闭冷藏冷凝风机
					f_lc_compressor=OFF;//关闭冷藏压机
				}
			}else if(r_lclnsjwd < LengNing_NotProtect_temp )
			{//低于50度退出高温保护
				Protect_HigLn_Temp=0;
			}
		}else{//温度低于0度，不进入压机停机
			Protect_HigLn_Temp=0;
		}
	}
}


/*****************************************************
判断化霜传感器温度，是否到达化霜条件
压缩机累计运行超过了6小时并且，化霜传感器温度<0.5度
*****************************************************/
void JudgeLcHuaShuangLow(void)
{
	if(f_lcCompSixHour){//压缩机已经累计运行6小时
		if (r_lcHuaSHuangxswd_float <= 515)//化霜传感器温度值小于0.5度进入化霜条件
		{
			if(f_lc_compressor == OFF){//压机关着
				LcHuaShuang_Flag=1;
				f_lcCompSixHour=0;//重置压机累计统计时间
			}

		}else
		{//退出化霜条件在压机控制逻辑里面判断吧

		}
	}else{//退出化霜条件
		
	}
}
/**************************************************/
void JudgeLdHigh(void)//冷冻温度判断报警
{
////#ifdef Need_Ld
//	//冷冻传感器错误，现在湿度来用
//	if (!ldyj)
//	{
//		f_ld_high = 0;
//		f_ld_high_buzz = 0;
//		t_ld_high_buzz = t_tens;
//	}
//	//高温报警,初次上电经延迟时间后FMQ报警,此后15分钟FMQ报警
//	else if (r_ldxswd > (r_ldzt + r_ld_high_alarm)) //显示和设定温度都上偏200
//	{
//		f_ld_high = 1;
//		if (f_Powerupdelayld) //初次冷冻上电延迟时间+15min后FMQ报警
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
//	//高温报警正常
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
	//冷藏传感器错误
	if (f_lc_sensor_err || (!lcyj))
	{
		f_lc_high = 0;
		f_lc_high_buzz = 0;
		t_lc_high_buzz = t_tens;
	}
#if (LC_DISP_RESOLUTION_0D1 || LC_RESOLUTION_0D1) //显示分辨率0.1度
	else if (r_lcxswd_float > (r_lczt_float + r_lc_high_alarm))
#else
	else if (r_lcxswd > (r_lczt + r_lc_high_alarm))
#endif
	{
		f_lc_high = 1;
		if (f_Powerupdelaylc) ////if(f_Powerupdelaylc&&!f_door_switch) 11.02.23wys开门状态高温15min报警
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
#if (LC_DISP_RESOLUTION_0D1 || LC_RESOLUTION_0D1) //显示分辨率0.1度
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
void JudgeLdLow(void)//冷冻显示温度低温报警判断，现在湿度传感器用
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
#if (LC_DISP_RESOLUTION_0D1 || LC_RESOLUTION_0D1) //显示分辨率0.1度
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
#if (LC_DISP_RESOLUTION_0D1 || LC_RESOLUTION_0D1) //显示分辨率0.1度
	else if (r_lcxswd_float > (r_lczt_float - r_lc_low_alarm))
#else
	else if (r_lcxswd > (r_lczt - r_lc_low_alarm))
#endif
	{
		f_lc_low = 0;
		f_lc_low_buzz = 0;
		t_lc_low_buzz = t_tens;
	}
	//检测是否进入低温保护
	if (f_lc_sensor_err || (!lcyj))
	{
		f_lc_low = 0;
		f_lc_low_buzz = 0;
		t_lc_low_Protect = t_tens;
	}else if(r_lcxswd_float < (2.5+38)){//显示温度小于2.5度 从显示传感器NTC2接口拿数据
		if ((unsigned char)(t_tens - t_lc_low_Protect) >= 6) //1min
		{
			Protect_Low_Temp=1;//低温保护
			f_lc_fan = OFF;//关闭冷藏冷凝风机
			f_lc_compressor=OFF;//关闭冷藏压机
			f_nd_fan=ON;
		}
		
	}else{//至于什么时候打开，看主程序的判断
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
			if ((unsigned char)(t_halfsec - t_voltage_buzz) >= 120) //1min/////////////规格书是30s
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
		if ((uchar)(t_halfsec - t_power_Off) >= 6) //低电压持续3s，则报断电 @20181226 CFJ
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
		t_power_Off = t_halfsec; //@20181226 CFJ 掉电的时间
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

		LedWriteByte(r_led12); //冷冻数码管  第2位
		LedWriteByte(r_led13); //冷冻数码管  第3位
		LedWriteByte(r_led34); //报警和锁定指示灯

		LedWriteByte(r_led21); //冷藏数码管  第1位 从数组中拿的数据
		LedWriteByte(r_led22); //冷藏数码管  第2位
		LedWriteByte(r_led23); //冷藏数码管  第3位

		LedStop();
		LedBegin();
		LedWriteByte(0x28);	//(0b00101000);                       //write,fixed address,bank1,address 0  @20181008 CFJ
		LedWriteByte(r_led11); //冷冻温度符号

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
	if (f_first_ad)//拿到了lc ld温度再使能所有操作
	{
		ReadKey();   //读按键值
		KeyAction(); //分析按键
		AutoLock();//10S后自动存储设置值
	}
}
//^^^^^^^^^^^^^^^^^^^^^^^^^
void ReadKey(void)
{
	r_key = 0;
	if (KEY_SW1)//设置
	{
		r_key = r_key | 0x01; //0b00000001; @20181008 CFJ
	}
	if (KEY_SW2)//位数切换
	{
		r_key = r_key | 0x02; //0b00000010; @20181008 CFJ
	}
	if (KEY_SW3)//温度调节
	{
		r_key = r_key | 0x04; //0b00000100; @20181008 CFJ
	}
	if (KEY_SW4)//温区切换
	{
		r_key = r_key | 0x08; //0b00001000; @20181008 CFJ
	}
	if (KEY_SW5)//报警测试
	{
		r_key = r_key | 0x10; //0b00010000; @20181008 CFJ
	}
	if (KEY_SW6)//蜂鸣取消
	{
		r_key = r_key | 0x20; //0b00100000; @20181008 CFJ
	}
	if (KEY_SW7)//除霜
	{
		r_key = r_key | 0x40; //0b01000000; @20181008 CFJ
	}
}

//^^^^^^^^^^^^^^^^^^^^^^^^^
void KeyAction(void)
{
	if (r_key == 0) //没有按键按下
	{
		r_keyz = 0;
		KeyPutUp(); //抬起有效的按键处理
		r_sfkeyz = 0;//存储按键的值，在别的地方使用，在keyputdown函数中赋值，也就是按下赋值，抬起后判断值，平时清零
		f_key_kp = 0;
		f_key_km = 0;
	}
	else //有按键按下
	{
		if (!f_key_kp)//从f_key_kp没有按键按下到有按键按下时有效
		{
			if (r_key != r_keyz)
			{//按下按键了
				f_key_km = 0;
				r_keyz = r_key;
				t_key_dly = t_onems;
				t_key3s = t_halfsec;
			}
			else
			{//按下后，第二次判断，有效值>80ms
				if (!f_key_km)
				{
					if ((unsigned char)(t_onems - t_key_dly) >= 80)
					{
						f_key_km = 1;
						KeyPutDown(); //按下有效的按键处理,给r_sfkeyz赋值，以便在别的地方使用
					}
				}
				else
				{//按下时间太长了超过了5S
					if ((unsigned char)(t_halfsec - t_key3s) >= 10)
					{
						KeyPut5s(); //按下5秒有效的按键处理
					}
					/*if((unsigned char)(t_halfsec-t_key3s)>=6)
            {
                KeyPut3s();  //按下3秒有效的按键处理
            }*/
				}
			}
		}
	}
}
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
void KeyPutUp(void) //按键抬起有效
{
	if (r_sfkeyz == KEY_UNLOCK) //位数切换(解锁)
	{
		if (!f_lock) //解锁情况下
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
			{//控制数码管的哪一位进入设置状态
				r_flash_bit++;
				if (r_flash_bit > 3)
				{
					r_flash_bit = 1;
				}
				goto NotSaveKey2;
			}
		}
	}
	else if (r_sfkeyz == KEY_SET) //设定键
	{//保存当前设定值
		if (!f_lock)
		{//解锁  
		
			if (r_set_state == SET_NC_LD) //冷冻当前调整
			{
				r_set_state = SET_LC_L0; //LD进入显示L0画面
				r_flash_bit = BIT3;		 //第3位闪
				goto NotSaveKey2;
			}
			if (r_set_state == SET_NC_LC) //冷藏当前调整状态
			{
				r_set_state = SET_LC_L0; //Lc显示L0画面
				r_flash_bit = BIT3;
				goto NotSaveKey2;
			}
			else if (r_set_state == SET_LD_L0) //冷冻L0-->设定温度
			{
				r_set_state = SET_LD;
				r_flash_bit = BIT2;
//				f_zf = FU_SIGN;
				r_bwtj = (uchar)((r_ldzt) / 100);
				r_swtj = (uchar)((r_ldzt) / 10);
				r_gwtj = (uchar)((r_ldzt) % 10);
				goto NotSaveKey2;
			}
			else if (r_set_state == SET_LD_L1) //L1-->高湿报警设定值
			{
				r_set_state = SET_LD_HIGH;
				r_flash_bit = BIT2;//设置中间那个数值
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
					//范围控制在0-100
					if ((r_ldzt + r_ld_high_alarm) > 100)
					{
						r_bwtj=1;
						r_swtj=0;
						r_gwtj=0;
						
					}
					else if((r_ldzt + r_ld_high_alarm)<0)
					{
						r_bwtj=0;
						r_swtj=0;
						r_gwtj=0;
					}
				#endif
				goto NotSaveKey2;
			}
			else if (r_set_state == SET_LD_L2) //L2-->低温报警设定值
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
			else if (r_set_state == SET_LD_L3) //L3-->电压查询
			{
				r_set_state = LD_CHK_VOL;
				r_flash_bit = BIT3;
				goto NotSaveKey2;
			}
			else if (r_set_state == SET_LD_L4) //L4-->环温查询
			{
				r_set_state = LD_CHK_HW;
				r_flash_bit = BIT3;
				goto NotSaveKey2;
			}
			else if (r_set_state == SET_LD_L5) //LD压缩机停机设置
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
				r_set_state = LC_CHK_LCKZ;//冷藏控制传感器温度显示
				r_flash_bit = BIT3;
				goto NotSaveKey2;
			}
			else if (r_set_state == SET_LC_L7)
			{
				r_set_state = LC_CHK_LCHS;//冷藏化霜传感器温度显示
				r_flash_bit = BIT3;
				goto NotSaveKey2;
			}
			else if (r_set_state == SET_LC_L8)
			{
				r_set_state = LC_CHK_LCLN;//冷藏化霜传感器温度显示
				r_flash_bit = BIT3;
				goto NotSaveKey2;
			}
			else if (r_set_state == LC_CHK_VOL || r_set_state == LC_CHK_HW||r_set_state == LC_CHK_LCKZ||r_set_state == LC_CHK_LCHS||r_set_state == LC_CHK_LCLN)
			{
				r_set_state = SET_NC_LC;
				goto NotSaveKey2;
			}
			else if (r_set_state == SET_LD) //湿度设定值
			{
				if ((r_swtj * 10 + r_gwtj) > 0 && (r_swtj * 10 + r_gwtj) <= 100) //湿度设定值在1-99之间
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
			else if (r_set_state == SET_LD_HIGH) //冷冻高温报警设置
			{
				#ifdef Need_Ld
					if (f_zf == ZHENG_SIGN) //高温报警设置最大50
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
					else //高温报警设置最小>设定温度+1
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
					if (f_zf == ZHENG_SIGN) //湿度报警设置最大100
					{
						if ((r_bwtj*100+r_swtj * 10 + r_gwtj) > 100)
						{
							goto SetErr;
						}
						else
						{
							if((r_bwtj*100+r_swtj * 10 + r_gwtj) < (r_ldzt+1))
							{//高温报警值要大于等于设定值+1
								goto SetErr;
							}
							r_ld_high_alarm = (r_bwtj*100+r_swtj * 10 + r_gwtj) - r_ldzt;//拿到差值
							if(r_ld_high_alarm<1){r_ld_high_alarm=1;}
							r_set_state = SET_NC_LD;
							goto SaveKey2;
						}
					}
					else //高温报警设置最小>设定温度+1
					{
						goto SetErr;
					}
				#endif
			}
			else if (r_set_state == SET_LD_LOW) //LD低温设定
			{
				#ifdef Need_Ld
					if (f_zf == ZHENG_SIGN) //正号
					{
						goto SetErr;
					}
					else //负号
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
					if (f_zf == ZHENG_SIGN) //正号
					{
						if (((r_swtj * 10 + r_gwtj)) > (r_ldzt - 1)) //>SET-1
						{
							goto SetErr;
						}
						else if (((r_swtj * 10 + r_gwtj)) < 0) //<-60
						{
							goto SetErr;
						}
						else
						{
							r_ld_low_alarm = r_ldzt - ((r_swtj * 10 + r_gwtj));//拿到差值
							if(r_ld_low_alarm<1){r_ld_low_alarm=1;}
//							if(r_ld_low_alarm<1)
//							{
//								goto SetErr;
//							}
							r_set_state = SET_NC_LD;
							goto SaveKey2;
						}
					}
					else //负号
					{
						goto SetErr;
					}
				#endif
				
			}
			else if (r_set_state == SET_LC_L0) //L0-->设定温度
			{
				r_set_state = SET_LC;
				r_flash_bit = BIT2;
				f_zf = ZHENG_SIGN;
				r_swtj = (uchar)((r_lczt - 38) / 10);
				r_gwtj = (uchar)((r_lczt - 38) % 10);
				goto NotSaveKey2;
			}
			else if (r_set_state == SET_LC_L1) //L1-->高温报警设定值
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
			else if (r_set_state == SET_LC_L2) //L2-->低温报警设定
			{
				r_set_state = SET_LC_LOW;
				r_flash_bit = BIT2;
				if ((r_lczt - r_lc_low_alarm) >= 38) //设置温度 >= 低温报警温差
				{
					if (r_lczt >= (LC_SET_TEMP_MIN + 38)) //设置温度>=2度
					{
						f_zf = ZHENG_SIGN;
						r_swtj = (uchar)((r_lczt - r_lc_low_alarm - 38) / 10);
						r_gwtj = (uchar)((r_lczt - r_lc_low_alarm - 38) % 10);
					}
				}
				else //设置温度< 低温报警温差  ，低温报警温度值为负
				{
					f_zf = FU_SIGN;
					r_swtj = (uchar)(-(r_lczt - r_lc_low_alarm - 38) / 10);
					r_gwtj = (uchar)(-(r_lczt - r_lc_low_alarm - 38) % 10);
				}
				goto NotSaveKey2;
			}
			else if (r_set_state == SET_LC) //冷藏设定
			{
				//if(((r_swtj*10+r_gwtj)<=16)&&((r_swtj*10+r_gwtj)>=5))//20171106  if((r_swtj*10+r_gwtj)<=16)//冷藏设定温度5~16之间
				if (((r_swtj * 10 + r_gwtj) <= LC_SET_TEMP_MAX) && ((r_swtj * 10 + r_gwtj) >= LC_SET_TEMP_MIN)) //  2019-9-11 王潘飞
				{
					r_lczt = 38 + (r_swtj * 10 + r_gwtj);
					r_lczt_float = r_lczt;

					if ((r_lczt < (LC_SET_TEMP_MIN + 38)) || ((r_lczt - 38 - r_lc_low_alarm) < 0)) //低温报警值为负
					{
						r_lc_low_alarm = r_lczt - 38; //低温报警值为0
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
			else if (r_set_state == SET_LC_LOW) //冷藏低温报警设定值
			{
				if (f_zf == ZHENG_SIGN) //正号
				{
					//if(((38+(r_swtj*10+r_gwtj))>(r_lczt-1))&& (r_lczt >= (LC_SET_TEMP_MIN+38) ) )
					if ((38 + (r_swtj * 10 + r_gwtj)) > (r_lczt - 1)) //低温设置值等于或大于冷藏设置温度
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
							r_lc_low_alarm = r_lczt - 38; //实际是当设定温度小于冷藏最低可设置温度,低温报警值强制为0度
						}
						r_set_state = SET_NC_LC;
						goto SaveKey2;
					}
				}
				else //负号
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
					r_lc_low_alarm = r_lczt - 38; //负号实际设定的低温报警值就是0度
					r_set_state = SET_NC_LC;
					goto SaveKey2;
					//}
				}
			}
			else if (r_set_state == SET_LD_WDJZ) //冷冻温度校准
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
			else if (r_set_state == SET_LCYJ_DELAY) //冷藏压机延迟时间
			{
				r_set_state = SET_NC_LC;
				t_yj_delay = t_yj_delayx;
				goto SaveKey2;
			}
			else if (r_set_state == SET_LD_BJYCSJ) //冷冻报警延迟时间
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
			else if (r_set_state == SET_LDYJ) //冷冻压机
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
	else if (r_sfkeyz == KEY_TEST_ALARM) //报警测试
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
	else if (r_sfkeyz == KEY_DISABLE_BUZZ) //消音
	{
		f_stop_alarm = ON;//每按一次消音，那么系统会重新统计下次的报警时间
		t_stop_alarm = 0;
		goto NotSaveKey2;
	}
	else if (r_sfkeyz == KEY_SWITCH) //切换键
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
		else if (r_set_state == SET_LD_BJYCSJ)//报警延迟时间
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
	else if (r_sfkeyz == KEY_DEFROST) //化霜键，现在用来开关灯
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
			f_lock = OFF;//解锁
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
	{//在这里判断所有界面上的当前显示状态，并进行调节
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
			r_set_state = SET_LD_L0;//张国帆只保留湿度的高报警(L1)和低报警(L0)
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
		else if (r_set_state == SET_LC_L6)//显示冷藏控制传感器的温度
		{
			r_set_state = SET_LC_L7;
			r_flash_bit = BIT3;
			goto NotSaveKey;
		}
		else if (r_set_state == SET_LC_L7)//显示化霜传感器的温度
		{
			r_set_state = SET_LC_L8;
			r_flash_bit = BIT3;
			goto NotSaveKey;
		}
		else if (r_set_state == SET_LC_L8)//显示冷凝传感器的温度
		{
			r_set_state = SET_LC_L0;
			r_flash_bit = BIT3;
			goto NotSaveKey;
		}
		else if (r_set_state == SET_LD || r_set_state == SET_LC)
		{
			if (r_flash_bit == 2) //冷冻冷藏十位调节
			{
				r_swtj++;
				if (r_swtj > 9)
				{
					r_swtj = 0;
				}
			}
			else if (r_flash_bit == 3) //冷冻冷藏个位调节
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
			if(r_set_state == SET_LD_HIGH )//冷冻(湿度)允许设置到100即，可以设置到最高位
			{
				if (r_flash_bit == 1) //冷冻(湿度)高温报警值最高位调节
				{
					f_zf = 0;//正号不显示
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
				if (r_flash_bit == 1) //冷冻冷藏高温报警值符号位调节
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
			if(r_set_state == SET_LD_LOW)//冷冻(湿度)下限设定
			{
				if (r_flash_bit == 1) //冷冻(湿度)低温报警值最高位调节
				{
					f_zf = 0;//首位一直是正也就是黑的不显示
					r_bwtj=0;//百位置为0
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
				if (r_flash_bit == 1) //冷冻冷藏低温报警值符号位调节
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
					if (r_set_state == SET_LC_LOW)
					{
						if (r_lczt >= (LC_SET_TEMP_MIN + 38)) //zyj  100604
						{
							r_swtj++;
							if (r_swtj > 9)
							{
								r_swtj = 0;
							}
						}
					}
					else
					{
						r_swtj++;
						if (r_swtj > 9)
						{
							r_swtj = 0;
						}
					}
				}
				else if (r_flash_bit == 3)
				{
					if (r_set_state == SET_LC_LOW)
					{
						if (r_lczt >= (LC_SET_TEMP_MIN + 38)) //zyj  100604
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
						r_gwtj++;
						if (r_gwtj > 9)
						{
							r_gwtj = 0;
						}
					}
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
			{//纠偏正负15
				r_ldwdjzx = 5;
			}
			goto NotSaveKey;
		}
		else if (r_set_state == SET_LC_WDJZ)
		{
			r_lcwdjzx++;
			if (r_lcwdjzx > 15)
			{
				r_lcwdjzx = 5;
			}
			goto NotSaveKey;
		}
		else if (r_set_state == SET_LC_XSWDJZ)
		{
			r_lcxswdjzx++;
			if (r_lcxswdjzx > 15)
			{
				r_lcxswdjzx = 5;
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
	if (r_keyz == KEY_UNLOCK) //解锁
	{
		if (f_lock)
		{
			r_set_state1 = SET_F0;
			r_flash_bit = BIT3;
			t_auto_lock = t_halfsec;
			goto NotSaveKey2;
		}
	}
	else if (r_keyz == KEY_DELAY) //冷藏压机延迟时间
	{
		if (!f_lock)
		{
			r_set_state = SET_LCYJ_DELAY;
			t_yj_delayx = t_yj_delay;
			goto NotSaveKey2;
		}
	}
	else if (r_keyz == KEY_WDJZ) //温度校准
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
	else if (r_keyz == KEY_LCXSWDJZ) //温度校准(显示传感器)
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
	else if (r_keyz == KEY_DEFROST) //化霜
	{
	//改成了自动化霜，把这个按键弄成了开关冷藏灯
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
	else if (r_keyz == KEY_BJYCSJ) //FMQ报警延迟时间  消音+设置
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
	else if (r_keyz == KEY_DATE)  //解锁+设置+调整
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
	else if (r_keyz == KEY_UsbCheck) //usb查询
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
	{//从按下开始计算，10S钟，这个可能是存储设置的意思
		f_lock = 1;//上锁
		if (r_set_state == SET_LD)//设置冷冻温度
		{

			if ((r_bwtj*100+r_swtj * 10 + r_gwtj) > 0 && (r_bwtj*100+r_swtj * 10 + r_gwtj) <= 100)
			{
				r_ldzt = (r_bwtj*100+r_swtj * 10 + r_gwtj);
				if((r_ldzt-r_ld_low_alarm)<1){r_ld_low_alarm=r_ldzt;}
				if((r_ldzt+r_ld_high_alarm)>100){r_ld_low_alarm=100-r_ldzt;}
				f_need_write_e2 = ON;
				t_write_e2 = t_halfsec;//设置t_write_e2这个值可能是为了避免频繁些e2
			}
			else
			{
				BuzzBiBiBi();
			}
		}
		else if (r_set_state == SET_LD_HIGH)//设置冷冻报警值高位
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
		else if (r_set_state == SET_LD_LOW)//设置冷冻报警值低位
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
					if (((r_swtj * 10 + r_gwtj)) < 0)
					{
						BuzzBiBiBi();
					}

					else if (((r_swtj * 10 + r_gwtj)) > (r_ldzt - 1))
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
		{//设置冷藏
			//if(((r_swtj*10+r_gwtj)<=16)&&((r_swtj*10+r_gwtj)>=5))//20171106  if((r_swtj*10+r_gwtj)<=16)
			if (((r_swtj * 10 + r_gwtj) <= LC_SET_TEMP_MAX) && ((r_swtj * 10 + r_gwtj) >= LC_SET_TEMP_MIN)) //  2019-9-11 王潘飞
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
		{//设置冷藏报警值高位
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
		{//设置冷藏报警值低位
			if (f_zf == ZHENG_SIGN)
			{
				if ((38 + (r_swtj * 10 + r_gwtj)) > (r_lczt - 1)) //低温报警值等于或大于设置温度
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
			else //负值强制为0
			{
				r_lc_low_alarm = r_lczt - 38;
				f_need_write_e2 = ON;
				t_write_e2 = t_halfsec;
			}
		}
		else if (r_set_state == SET_LD_WDJZ)
		{//温度矫正
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
/****************判断压机开停程序**********************/
/******************************************************/
void ldCompressorRun10Hour(void);
void ldCompRunOneHour(void);
/******************************************************/
void LdCompressorJudge(void) //冷冻压机判断
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
		f_on_off_dly = 0; //开停机压机延迟1分钟
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
	if (f_ldCompressorProtect) //压机10小时保护
	{
		goto CompressorOff;
	}
	if (f_ldComp1HourProtect) //压机1小时保护
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
void Compressor_on_delay(void) //上电压机延迟开时间  由按键设定
{
	if (!f_compressor_on_dly)
	{
		if (f_tens)//每10S执行一次
		{
			t_yj_dly_time++;
			if (t_yj_dly_time >= (t_yj_delay * 6))
			{
				f_compressor_on_dly = 1;//直到下次断电一直是这个状态
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
/************冷冻压机连续运行10小时程序**************/
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
/************冷冻压机连续运行1小时程序**************/
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

 /****************冷藏压机控制程序****************/
void Lc_CompressorJudge(void)
{
	//lcCompressorRun10Hour();
	lcCompRunSixHour();//化霜专用 先暂时停止化霜逻辑
//#if LC_COMP_RUN_MAX_1_HOUR
//	lcCompRunOneHour(); //按照系统人员要求屏蔽一小时的，HW2019/7/29 9:50
//#elif LC_COMP_RUN_MAX_6_HOUR
//	lcCompressorAddup6Hour();
//#endif
	///lcCompRunOneHour();               按照系统人员要求屏蔽一小时的，HW2019/7/29 9:50
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
//	if (f8_lcCompAddUp6HourProtect != 0) // 压机累计6小时保护
//	{									 // 140819
//		goto Lc_CompressorOff;
//	}
//	if (f_lcCompressorProtect) //压机30小时保护
//	{
//		goto Lc_CompressorOff;
//	}
//	if (f_lcComp1HourProtect) //压机1小时保护
//	{
//		goto Lc_CompressorOff;
//	}
//	#ifdef Need_Ld//冷冻的话，就算了
//	#else
//	if (f_key_defrost == ON)
//	{//开化霜关压机，关冷凝风机
//		goto Lc_CompressorOff;
//	}
//	#endif
	if (f_lc_sensor_err)
	{//冷藏控制传感器故障，根据环温控制压缩机
		//环温小于16度，压缩机运行3分钟，停机15分钟
		//16<环温传感器<25，压缩机运行5分钟，停机8分钟
		//25<环温传感器<35，压缩机运行9分钟，停机6分钟
		//35<环温传感器<45，压缩机运行15分钟，停机3分钟
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
			
		}else{//超过45度 
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
	if(f_deHum_and_lcTemp==0){//当前没有在除湿
		//判断是否需要化霜
		if(LcHuaShuang_Flag){//化霜条件到达
			f_ln_fan=1;//打开指示灯
			if (f_onemin)
			{
				u8_lcChuShuangTime++;
			}
			//1、	化霜时间达到HUASHUANG_LASTED_TIME分钟
			if(u8_lcChuShuangTime>HUASHUANG_LASTED_TIME){//误差1分钟
				LcHuaShuang_Flag=0;
				f_defrost=OFF;
				LcHuaShuang_Flag_after=1;
			}
			//2、	化霜传感器温度>5度，持续时间>1分钟
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
			}else{//要一直>5度才行
				u8_lcChuShuangTime1=0;
			}
			//3、	箱内显示温度传感器温度>6度
			if(r_lcwd_float>38+7){
					LcHuaShuang_Flag=0;
					f_defrost=OFF;
					LcHuaShuang_Flag_after=1;
			}
			//上面三个条件都没满足，正常化霜
			if (f_halfsec)
			{
				u32_t_lcChuShuangTime1++;
			}
			if(u32_t_lcChuShuangTime1>=1800){//内风机工作15分钟后停机
				//内风机停机
				f_nd_fan=OFF;
				u32_t_lcChuShuangTime1=1800;
			}else{//进入化霜时，开风机
				f_nd_fan=ON;
			}
			//加热丝工作
			f_defrost=ON;
			//压缩机停机
			goto Lc_CompressorOff;

		}else{//当前不化霜
			u8_lcChuShuangTime=0;
			u8_lcChuShuangTime1=0;
			u32_t_lcChuShuangTime1=0;
			f_defrost=OFF;
		}
		if(LcHuaShuang_Flag_after){//跳到化霜结束后的善后处理逻辑
			//1、	化霜加热停止工作
			f_defrost=OFF;
			//2、	压缩机开机工作，内风机不工作
			if (f_halfsec)
			{
				u32_t_lcChuShuangTime2++;
				u32_t_lcChuShuangTime3++;
			}
			if(u32_t_lcChuShuangTime2>=240){//2分钟后开风机
				f_nd_fan=ON;
				LcHuaShuang_Flag_after=0;//退出化霜逻辑,按照正常温度控制邹
				u32_t_lcChuShuangTime2=240;
			}else{//否则关闭
				f_nd_fan=OFF;//关闭蒸发风机
			}
			if(u32_t_lcChuShuangTime3>=120){//1分钟后开压机
                u32_t_lcChuShuangTime3=120;
				goto Lc_CompressorOn;
				
			}else{//否则关闭
				goto Lc_CompressorOff;
			}
			

		}else{//正常运行的逻辑
		    f_ln_fan=0;
			u32_t_lcChuShuangTime2=0;
			u32_t_lcChuShuangTime3=0;
			t_lc_ChuShi_Compressor=0;
			f_nd_fan=ON;
			if (f_lc_compressor == ON)
			{//冷藏控制传感器温度<=设置温度               ad值越小，温度越高
				if (r_lcwd >= r_lczt){//冷藏控制传感器温度>=设定值
					f_defrost=OFF;
					goto Lc_CompressorOn;
				}else{
					f_defrost=ON;
					goto Lc_CompressorOff;
				}
			}
			else
			{
				if (r_lcwd <= r_lczt){//冷藏控制传感器温度<=设定值
					f_defrost=ON;
					goto Lc_CompressorOff;
				}else{
					f_defrost=OFF;
					goto Lc_CompressorOn;
				}
			}
		}
	}else{//除湿逻辑
		if(f_deHum_and_lcTemp==1){//每隔2分钟工作1分钟
			if (f_halfsec)
			{
				t_lc_ChuShi_Compressor++;
			}
			if (t_lc_ChuShi_Compressor < 121)//61
				{
					//开内风机
					f_nd_fan=ON;
				}
				else if (t_lc_ChuShi_Compressor < 121+240)//361
				{
					//关内风机
					f_nd_fan=OFF;
				}
				else
				{
					t_lc_ChuShi_Compressor = 0;
					//开内风机
				}
			
		}else if(f_deHum_and_lcTemp==2){//每隔2分钟工作20s
			if (f_halfsec)
			{
				t_lc_ChuShi_Compressor++;
			}
			if (t_lc_ChuShi_Compressor < 41)
			{
				//开内风机
				f_nd_fan=ON;
			}
			else if (t_lc_ChuShi_Compressor < 41+240)
			{
				//关内风机
				f_nd_fan=OFF;
			}
			else
			{
				t_lc_ChuShi_Compressor = 0;
				//开内风机
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
			
			if(Protect_HigHw_Temp==0&&Protect_HigLn_Temp==0&&Protect_Low_Temp==0){//环温和冷凝器温度都没超温 也没有低温保护
				f_lc_fan = ON;//冷藏冷凝风机
				f_lc_compressor = ON;//；冷藏压机
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
			{//目前永远得不到执行
				f8_lcCompAddUp6HourTimer = 0;
				//
				f8_lcCompAddUp6HourProtect = 1;
				u8_lcCompAddUp6HourProtectTimer = t_tens;
				//f_lc_on_off_dly = 0;                          // 不计算5分钟
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
				if (u16_lcComp_Addup_Minute >= 30 * 60) //按照系统30H小时
				{
					f8_lcCompAddUp6HourTimer = 1;
					/*  141017        
             f8_lcCompAddUp6HourProtect = 1;
             u8_lcCompAddUp6HourProtectTimer = t_tens;            	
             f_lc_on_off_dly = 0;                          // 不计算5分钟 
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
// 压机累计运行30小时 20190729
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

//		if (r_lcad_12b >= LCCOMPRTSTOREAD) //冷藏压缩机开机点  HW
//		{
//			f_lcCompressorProtect = 0;
//		}
	}
}
/************冷藏压机连续运行1小时程序**************/
//当温度达到设定温度,冷藏压缩机的连续运行1小时,强制停机5min
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

/************冷藏压机连续运行6小时程序**************/
//化霜专用压机累计运行6H并且化霜传感器温度<0.5度时，要启动化霜逻辑
/***************************************************/
void lcCompRunSixHour(void)
{
	if (!f_lcCompSixHour)
	{
		if (f_lc_compressor )
		{
			if (f_onemin)
			{
				if (++u16_lcCompRunMinForHuaShuang >= 60*2)//累计运行2h,误差1分钟 //60*2 暂时为了测试改成20分钟
				{
					u16_lcCompRunMinForHuaShuang = 0;
					f_lcCompSixHour = 1;
					//f_on_off_dly = 0;
				}
			}
		}
	}
}


/********************判断风机程序*****************/
/*************************************************/
void LnFan(void)//冷凝风机
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

/********************判断冷藏照明灯程序****************/
void Lc_lightProg(void)
{
	//这个现在改成无效了，通过按键事件来设置标志位f_LightStatusPin，以达到底板上面CN17控制12V等的开启和关闭
	if(f_LightStatusPin==1){//化霜按键控制
		//f_LightStatusPin = 1;
	}else{
		//f_LightStatusPin = 0;
	}
//	LED_OUT = 1;//没有门，姜明同志说，这个灯常开就行了
//	if (!DOOR_ONFLAG | !LED_ONFLAG) //if(!PTBD_PTBD7|!PTDD_PTDD0)低电平有效
//	{
//		if ((uchar)(t_onems - t_lightms) >= 60)
//			LED_OUT = 1; //PTDD_PTDD2=1;   @20180920 cfj 高电平开灯
//		f_LightStatusPin = 1;
//	}
//	else
//	{
//		t_lightms = t_onems;
//		LED_OUT = 0; //PTDD_PTDD2=0;      @20180920 cfj
//		f_LightStatusPin = 0;
//	}
}
/********************内胆风机程序****************/
//void Nd_fan_Prog(void)
//{
//	if (DOOR_ONFLAG) //if(PTBD_PTBD7)    @20180920 cfj 冷藏关着门
//	{
//		f_door_switch = 0; // 100526  zyj
//		if (lcyj)//这个值从flag_ajszyjkt中来，flag_ajszyjkt的值在按键判断部分进行赋值，也从E2中读取，包含一个按键控制风机开停
//		{//如果允许风机开启
//			if ((uchar)(t_onems - t_nd_fan) >= 60)
//			{
//				f_nd_fan = 1;//这个值传给底板，通过底板控制风机
//			}
//		}
//		else
//		{
//			t_nd_fan = t_onems;
//			if(Protect_Low_Temp==0){//当前没有低温报警
//				f_nd_fan = 0;
//			}
//		}
//	}
//	else
//	{
//		f_door_switch = 1;//开着门
//		t_nd_fan = t_onems;
//		if(Protect_Low_Temp==0){//当前没有低温报警
//			f_nd_fan = 0;
//		}
//	}
//}
///***********开门报警程序***************/ /////wys11.03.19
//void Door_Open(void)
//{
//	if (f_door_switch) //开门
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
/********************按键除霜程序****************/
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
/***************发送程序*****************************/
/****************************************************/
/* @20181130 CFJ 
#if 0
void SendData(void)
{
  
  if(f_first_ad)       
  {
  	if(f_txbuzz)//蜂鸣器要响，立刻准备开始发送新一帧
  	{
      f_txbuzz = 0;
      f_newframe = 0;
      f_senddata = 0;
      f_sendbit = 0;
      //C_send = 1; @20181008 CFJ
      f_gdp = 1;
      t_send = t_onems;
  	}
    else if(!f_newframe)//不是新一帧，等待新一帧到来,相当于出示化端口
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
        if(f_sendbit)//正在发送一位数据
        {
          if(f_gdp)
          {
            if((unsigned char)(t_onems - t_send) >= 1)    //发送高电平期间
            {
              f_gdp = 0;
             // C_send = 0;  @20181008 CFJ
              t_send = t_onems;
            }
          }
          else// 发送低电平期间
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
                                                 //刷新数据
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
                                                  //接收到起始信号
            f_senddata = 1;
            f_gdp = 1;
            //C_send = 1; @20181008 CFJ
            t_send = t_onems;
            r_bit = C_bit;
            r_byte = 0;                           //要发送的字节数
          }                    
        }
        else                                      //蜂鸣器帧为2ms高，20ms低
        {
          if((unsigned char)(t_onems-t_send)>=20)
          {
                                                  //接收到起始信号
            f_senddata = 1;
            f_gdp = 1;
            //C_send = 1; @20181008 CFJ
            t_send = t_onems;
            send[0] = 1;
            r_bit = C_bit;
            r_byte = 0;                           //要发送的字节数
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
      if(f_startrec)                                        //1开始接收新一帧数据
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
/***************更新设定值程序***********************/
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
				WriteByte(0xA0); ///write chip addres 0xa0      每页64个字节，可以连续写
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
				//冷藏压机
				WriteByte(lc_compressor_times>>24); //byte14
				WriteByte(lc_compressor_times>>16); //byte15
				WriteByte(lc_compressor_times>>8);//byte16
				C_SDA_SET_HIGH;
				Stop();
				t_WriteE2=t_onems;
			}else if(WriteE2_num == 2){
				//第二页
				Begin();
				WriteByte(0xA0); ///write chip addres 0xa0      每页64个字节，可以连续写
				WriteByte(0x10); ///write data high addres
				WriteByte(lc_compressor_times);//byte17
				//冷凝风机
				WriteByte(lc_fan_times>>24); //byte18
				WriteByte(lc_fan_times>>16); //byte19
				WriteByte(lc_fan_times>>8);//byte20
				WriteByte(lc_fan_times);//byte21
				//蒸发风机
				WriteByte(lc_nd_fan_times>>24); //byte22
				WriteByte(lc_nd_fan_times>>16); //byte23
				WriteByte(lc_nd_fan_times>>8);//byte24
				WriteByte(lc_nd_fan_times);//byte25
				WriteByte(r_lcxswdjz);//冷藏显示温度就纠正值 byte26
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

	r_ldzt = ReadByte(); //byte1冷冻设定
	Sendscl();

	t_yj_delay = ReadByte(); //byte2压机延迟时间
	t_yj_delayx = t_yj_delay;
	Sendscl();

	r_ldwdjz = ReadByte(); //byte3冷冻校准值
	r_ldwdjzx = r_ldwdjz;
	Sendscl();

	r_ld_high_alarm = ReadByte(); //byte4冷冻高温报警设定的差值
	Sendscl();

	r_ld_low_alarm = ReadByte(); //byte5
	Sendscl();

	r_lczt = ReadByte(); //byte6//冷藏设定温度
	r_lczt_float = r_lczt;
	Sendscl();

	r_lc_high_alarm = ReadByte(); //byte7
	Sendscl();

	r_lc_low_alarm = ReadByte(); //byte8 //冷藏低温报警设定的差值
	Sendscl();

	flag_ajszyjkt = ReadByte() & 0x03; //byte9按键设置压机开停
	Sendscl();

	r_lcwdjz = ReadByte(); //byte10冷藏温度校准值
	r_lcwdjzx = r_lcwdjz;
	Sendscl();

	u8_ld_bjycsj = ReadByte(); //byte11冷冻报警延迟时间
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
	//冷藏压机
	lc_compressor_times=0;
	lc_compressor_times |= (ReadByte()<<24); //byte14
	Sendscl();
	lc_compressor_times |= (ReadByte()<<16); //byte15
	Sendscl();
	lc_compressor_times |= (ReadByte()<<8); //byte16
	Sendscl();
	//第二页
	lc_compressor_times |= ReadByte(); //byte17
	Sendscl();
	if(lc_compressor_times==0xffffffff)lc_compressor_times=0;
	//冷凝风机
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
	//蒸发风机
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
		r_ldzt = 60; //湿度默认60
		r_lczt = 43;  //5°C 43-38 初始设定值是5度
		r_lczt_float = r_lczt;
		t_yj_delayx = 1;
		t_yj_delay = 1;
		r_ldwdjzx = 10;
		r_ldwdjz = 20;//湿度纠偏中位数+-15
		r_ld_high_alarm = 5;
		r_ld_low_alarm = 5;
		r_lc_high_alarm = 3;
		r_lc_low_alarm = 3;
		r_lcwdjzx = 110;//控制传感器温度偏移量上调2度
		r_lcwdjz = 110;//控制传感器温度偏移量上调2度
		r_lcxswdjz=110;//显示传感器温度偏移量上调3度
		r_lcxswdjzx=110;//控制传感器温度偏移量上调3度
		u8_ld_bjycsjx = 2; //冷冻报警延迟时间,小时
		u8_ld_bjycsj = 2;
		u8_lc_bjycsjx = 2;
		u8_lc_bjycsj = 2;
		flag_ajszyjkt = 0x03; //按键设置压机开停用
		f_e2prom = 1;

		r_ldzt_report = 160;
		r_lczt_report = 43;
		ld_high_alarm_report = 5;
		ld_low_alarm_report = 5;
		lc_high_alarm_report = 3;
		lc_low_alarm_report = 3;
		lc_compressor_times=0;//冷藏压机运行时长
		lc_fan_times=0;//冷凝风机运行时长
		lc_nd_fan_times=0;//蒸发风机运行时长
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
	unsigned char r_dat, r_clk;
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
  RecData(); //面板接受主板数据 cfj
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
	/////a = SCI1S1;                                    //清发送中断请求位，首先读SCI1S1然后写SCI1D
	if(f_send_data)
	{
		if(f_send55)
		{
			f_send55 = 0;
			/////SCI1D = 0x55;
			if(r_sendr>=r_sendsum)
	    {
	  	  //////SCI1C2_TCIE = 0;                           //关闭发送中断
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
	  	  /////SCI1C2_TCIE = 0;                           //关闭发送模块
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
//机器上压机、冷凝风机、蒸发风机时间统计
void MachineTimeStatistics(void)
{
	if(f_power_off==1){//断电就不统计了
	}else{
		if(f_lc_compressor==ON){//压机
			if(f_onemin){
				lc_compressor_timesa++;
			}
		}
		if(f_nd_fan==ON){//蒸发风机/内风机
			if(f_onemin){
				lc_nd_fan_timesa++;
			}
		}
		if(f_lc_fan==ON){//冷凝风机
			if(f_onemin){
				lc_fan_timesa++;
			}
		}

		if((lc_compressor_timesa>=600) || (lc_nd_fan_timesa>=600) || (lc_fan_timesa>=600 )){//有一个大于600分钟，则写一次eeprom
			
			lc_compressor_times+=lc_compressor_timesa;//压机时间累加
			lc_nd_fan_times+=lc_nd_fan_timesa;//蒸发风机时间累加
			lc_fan_times+=lc_fan_timesa;//冷凝风机累加
			lc_compressor_timesa=0;//重新统计压机时间
			lc_nd_fan_timesa=0;//重新统计蒸发风机时间
			lc_fan_timesa=0;//重新统计冷凝风机时间
			f_need_write_e2=ON;
		}
	}
}

void GetSum2(void)
{
	uchar r_checksum = 0, count;

	u8_Send2dataMemory[0] = 0x55;
	u8_Send2dataMemory[1] = 0xFF;
	u8_Send2dataMemory[2] = u8_Send2_data_Num;

	r_checksum = u8_Send2_data_Num;

	for (count = 0; count < u8_Send2_data_Num - 1; count++)
	{
		r_checksum = r_checksum + u8_Send2data[count];
		u8_Send2dataMemory[count + 3] = u8_Send2data[count];
	}

	u8_Send2dataMemory[count + 3] = r_checksum;
	u8_Send2_data_Num = u8_Send2_data_Num + 3;

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
//	u8_Send2_data_Num = 52;//要发送的数据长度
//	u8_Send2_data_Num_Count = 0;
//	u8_Send2_data_State = 1;
//	SendData(u8_Send2dataMemory, u8_Send2_data_Num); 
//										   /////////// SCI2C2_TCIE = 1;                                 //使能发送模块
//}

void ReturnPrintFrame(void)
{

	if (u8_Send2_data_State == 1)
	{//上一帧数据还没发送完成
		return;
	}

	u8_Send2data[0] = 0x69;
	u8_Send2data[1] = 0x00;//数据上报;
	u8_Send2data[2] = 47;//数据部分长度
	u8_Send2data[3]=2;//1:HYC-406,2:HYC-116
	if (r_lcxswd_float >= 38.0){//温度为正  分辨率为0.1
		//显示温度低8位
		u8_Send2data[4]=(u16t)((r_lcxswd_float - 38.0) * 10 + 0.5);
		//显示温度高8位
		u8_Send2data[5] =((u16t)((r_lcxswd_float - 38.0) * 10 + 0.5))>>8;

	}else if (r_lcxswd_float <= 28.0){//温度<= -10度  //只显示整数部分
		//显示温度低8位
		u8_Send2data[4]=(-(u16t)((38 - r_lcxswd_float)*10))&0xff;
		//显示温度高8位
		u8_Send2data[5] =(-(u16t)((38 - r_lcxswd_float)*10))>>8;

	}else{//   0  ~ -9.9度   显示1位小数
		u8_Send2data[4]=(-(u16t)((38 - r_lcxswd_float)*10+0.5))&0xff;
		u8_Send2data[5]=(-(u16t)((38 - r_lcxswd_float)*10+0.5))>>8;
	}
//	f_battery = 1; //电池故障标志
//	f_power_off = 1;//断电报警
//	f_lc_sensor_err = 1;//冷藏控制传感器故障
//	f_lcXs_sensor_err = 1;//冷藏显示传感器故障兼容HYC406上传感器故障
//	f_lcHs_sensor_err=1;//化霜传感器故障
//	f_lcLn_sensor_err = 1;//冷凝传感器故障
//	f_hw_high38 = 1;//环温超38度报警
//	f_hw_sensor_err=1;//环温传感器故障
//	g_Sys_Erflag0_Comm = 1; //通讯错误 @20181025 CFJ
//	f_lc_high = 1;//冷藏显示传感器高温报警
//	f_lc_low = 1;//冷藏显示传感器低温报警
//	Protect_HigLn_Temp = 1;//冷凝传感高温

	//故障位 u8_Send2data[5]-u8_Send2data[7]
	u8_Send2data[10] = 0x00; 
	if(f_lc_high==1)u8_Send2data[10] |=1;
	if(f_lc_low==1)u8_Send2data[10] |=1<<1;
	if(f_power_off==1)u8_Send2data[10] |=1<<2;
	if(g_Sys_Erflag0_Comm==1)u8_Send2data[10] |=1<<3;
	if(f_lc_sensor_err==1)u8_Send2data[10] |=1<<4;
	if(f_battery==1)u8_Send2data[10] |=1<<5;
	//if(f_lcXs_sensor_err==1)u8_Send2data[10] |=1<<7; 20210531改成了byte[12]的bit1位
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

	//预留
	u8_Send2data[13] = 0x00;
	if(f_ld_high==1) u8_Send2data[13]|=1<<6;//湿度高
	if(f_ld_low==1) u8_Send2data[13]|=1<<5;//湿度低
	//冷藏开关门
	u8_Send2data[14] = 0x00;
	//冷冻开关门
	u8_Send2data[15] = 0x00;
	//湿度
	u8_Send2data[16] = r_ldgzwd;
	if(f_power_off==1){//断电 断电就是上报全关
		//正常控温状态--0，化霜状态1
		u8_Send2data[17] = 0;
		//压机状态
		u8_Send2data[18] = 0;
		//冷凝风机
		u8_Send2data[19] = 0;
		//蒸发风机
		u8_Send2data[20] = 0;
	}else{
		//正常控温状态--0，化霜状态1
		if(LcHuaShuang_Flag==1){//化霜
			u8_Send2data[17] = 1;
		}else{
			u8_Send2data[17] = 0;
		}
		//压机状态
		if(f_lc_compressor == ON){//开启
			u8_Send2data[18] = 1;
		}else{
			u8_Send2data[18] = 0;
		}
		//冷凝风机
		if(f_lc_fan == ON){//开启
			u8_Send2data[19] = 1;
		}else{
			u8_Send2data[19] = 0;
		}
		//蒸发风机
		if(f_nd_fan == ON){//开启
			u8_Send2data[20] = 1;
		}else{
			u8_Send2data[20] = 0;
		}
	}
	//冷藏压机累计工作时间
	u8_Send2data[21] = (lc_compressor_times+lc_compressor_timesa);
	u8_Send2data[22] = (lc_compressor_times+lc_compressor_timesa)>>8;
	u8_Send2data[23] = (lc_compressor_times+lc_compressor_timesa)>>16;
	u8_Send2data[24] = (lc_compressor_times+lc_compressor_timesa)>>24;
	//冷藏冷凝风机累计工作时间
	u8_Send2data[25] = (lc_fan_times+lc_fan_timesa);
	u8_Send2data[26] = (lc_fan_times+lc_fan_timesa)>>8;
	u8_Send2data[27] = (lc_fan_times+lc_fan_timesa)>>16;
	u8_Send2data[28] = (lc_fan_times+lc_fan_timesa)>>24;
	//蒸发风机累计工作时间
	u8_Send2data[29] = (lc_nd_fan_times+lc_nd_fan_timesa);
	u8_Send2data[30] = (lc_nd_fan_times+lc_nd_fan_timesa)>>8;
	u8_Send2data[31] = (lc_nd_fan_times+lc_nd_fan_timesa)>>16;
	u8_Send2data[32] = (lc_nd_fan_times+lc_nd_fan_timesa)>>24;
	//预留
	u8_Send2data[33] = 0;
	u8_Send2data[34] = 0;
	//冷冻压机状态
	u8_Send2data[35] = 0;
	//冷冻冷凝风机装
	u8_Send2data[36] = 0;
	//冷冻蒸发风机状态
	u8_Send2data[37] = 0;
	//冷冻压机累计工作时间
	u8_Send2data[38] = 0;
	u8_Send2data[39] = 0;
	u8_Send2data[40] = 0;
	u8_Send2data[41] = 0;
	//冷冻冷凝风机累计工作时间
	u8_Send2data[42] = 0;
	u8_Send2data[43] = 0;
	u8_Send2data[44] = 0;
	u8_Send2data[45] = 0;
	//冷冻蒸发风机累计工作时间
	u8_Send2data[46] = 0;
	u8_Send2data[47] = 0;
	u8_Send2data[48] = 0;
	u8_Send2data[49] = 0;
	//累加和
	u8_Send2data[50] = 0;
	//帧尾
	u8_Send2data[51] = 0x19;
	GetSum2();
}

void Rec2Action(void)
{
	if (u8_Send_Print_Time >= 2)
	{//每一秒钟搞一把，原来用在打印机上，现在用来和显示屏通信
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
	uchar a;                                       //清接收中断请求位，首先读SCI1S1然后读SCI1D
	/////a = SCI1S1;
	if(f_rec_over)                                 //待处理符号，如果 f_rec_over为1，再接收为无效数据                    
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
/*****************接收完成处理程序***************/
/************************************************/
void NetRecOver(void)
{
	uchar r_cheksum = 0;
	if (f_rec_over)
	{
		for (r_receiver = 0; r_receiver < (unsigned char)(r_recsum - 1); r_receiver++)
		{
			r_cheksum = r_cheksum + rec_net[r_receiver];
		}
		r_cheksum = r_cheksum + r_recsum + r_rec55sum;
		r_rec55sum = 0;
		if (r_cheksum == rec_net[r_receiver])
		{
			RecOkAction(); //接收完成生效程序
			f_rec_over = 0;
		}
		else
		{
			f_rec_over = 0;
		}
	}
}

/************************************************/
/*****************UART0接收完成处理程序***************/
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
			RecOkAction1(); //接收完成生效程序
			f_rec_over1 = 0;
		}
		else
		{
			f_rec_over1 = 0;
		}
	}
}
/************************************************/
/*****************UART0接收完成生效程序***************/
/************************************************/
void RecOkAction1(void)
{
	if(rec_net1_ok[0]==0x01){//解除报警
		f_stop_alarm = ON;
		t_stop_alarm = 0;
	}else if(rec_net1_ok[0]==0x02){//累计工作时间清零
		if(rec_net1_ok[2]==0x02){
			if(rec_net1_ok[3]==1){//1代表清零冷藏压机工作时间
				lc_compressor_times=0;
				lc_compressor_timesa=0;
			}
			if(rec_net1_ok[4]==1){//1代表清零冷藏冷凝风机工作时间
				lc_fan_times=0;
				lc_fan_timesa=0;
			}
			if(rec_net1_ok[5]==1){//1代表清零冷藏冷凝风机工作时间
				lc_nd_fan_times=0;
				lc_nd_fan_timesa=0;
			}
		}
		
		f_need_write_e2=ON;
	}
}

/************************************************/
/*****************接收完成生效程序***************/
/************************************************/
void RecOkAction(void)
{
	if (rec_net[6] == 1)
	{
		if (rec_net[7] == 0x4d)
		{
			if (rec_net[8] == 0x01) //4d 01 查询命令
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
			if (rec_net[8] == 0x01) //5d 01 设置冷藏
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
			else if (rec_net[8] == 0x02) //5d 02 设置冷藏高温
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
			else if (rec_net[8] == 0x03) //5d 03 设置冷藏低温
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
			else if (rec_net[8] == 0x04) //5d 04 设置冷冻
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
			else if (rec_net[8] == 0x05) //5d 05 设置冷冻高温
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
			else if (rec_net[8] == 0x06) //5d 06 设置冷冻低温温
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
			else if (rec_net[8] == 0x07) //5d 07 冷藏控制档位校准
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
			else if (rec_net[8] == 0x08) //5d 08 冷冻档位校准
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
		if(Cnt_SetTime_Sended >= 3)  //设置时间的命令发送3次之后，才允许将此标志置位
			ACKok = 1;																											 //lxx
		if ((rec_net[0] == (uchar)((u16_random_errcopy >> 8) & 0x3F)) && (rec_net[1] == (uchar)(u16_random_errcopy & 0xFF))) //要判断返回的帧序列号对错//if ((rec_net[0]==(uchar) ((u16_random_errcopy >>8)& 0b00111111)) &&(rec_net[1]==(uchar) (u16_random_errcopy & 0b11111111)))  //要判断返回的帧序列号对错
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
		else if ((rec_net[0] == (((uchar)(u16_random_copy >> 8)) & 0x3F)) && (rec_net[1] == (((uchar)(u16_random_copy)) & 0xFF))) //要判断返回的帧序列号对错//else if ((rec_net[0] == (((uchar)(u16_random_copy >> 8)) & 0b00111111)) && (rec_net[1] == (((uchar) (u16_random_copy)) & 0b11111111)))  //要判断返回的帧序列号对错
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
/*******************返回状态帧程序***************/
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
	send_net[12] = r_voltage; //电压

	send_net[13] = 0x00;
	send_net[14] = 0x00;

	send_net[15] = 0x00;
	send_net[16] = 0x00;

	send_net[17] = 0x00;
	send_net[18] = 0x00;
	send_net[19] = 0x00;
	send_net[20] = 0x00;

#if (LC_DISP_RESOLUTION_0D1 || LC_RESOLUTION_0D1) //显示分辨率0.1度
	r_lcxswd = (unsigned char)floor(r_lcxswd_float);
#else
	;
#endif
	send_net[21] = r_lcxswd; //冷藏显示温度
	send_net[22] = r_ldxswd;
	send_net[23] = r_lczt; //冷藏档位
	send_net[24] = r_ldzt; //冷冻档位

	send_net[25] = r_lczt + r_lc_high_alarm; //冷藏室报警上限
	send_net[26] = r_lczt - r_lc_low_alarm;  //冷藏室报警下限
	send_net[27] = r_ldzt + r_ld_high_alarm; //冷冻报警上限
	send_net[28] = r_ldzt - r_ld_low_alarm;  //冷冻报警下限

	send_net[29] = u8_year;
	send_net[30] = u8_month;
	send_net[31] = u8_day;
	send_net[32] = u8_hour;
	send_net[33] = u8_minute;
	send_net[34] = 0x00;

	send_net[35] = r_hwsjwd; //环温

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
/****************设置时间命令帧******************/
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
/***************U-Home回报帧*****************/
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
/*******************汇报帧程序*******************/
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
	send_net[12] = r_voltage; //电压

	send_net[13] = 0x00;
	send_net[14] = 0x00;

	send_net[15] = 0x00;
	send_net[16] = 0x00;

	send_net[17] = 0x00;
	send_net[18] = 0x00;
	send_net[19] = 0x00;
	send_net[20] = 0x00;

	send_net[21] = r_lcxswd; //冷藏显示温度
	send_net[22] = r_ldxswd;
	send_net[23] = r_lczt; //冷藏档位
	send_net[24] = r_ldzt; //冷冻档位

	send_net[25] = r_lczt + r_lc_high_alarm; //冷藏室报警上限
	send_net[26] = r_lczt - r_lc_low_alarm;  //冷藏室报警下限
	send_net[27] = r_ldzt + r_ld_high_alarm; //冷冻报警上限
	send_net[28] = r_ldzt - r_ld_low_alarm;  //冷冻报警下限

	send_net[29] = u8_year;
	send_net[30] = u8_month;
	send_net[31] = u8_day;
	send_net[32] = u8_hour;
	send_net[33] = u8_minute;
	send_net[34] = 0x00;

	send_net[35] = r_hwsjwd; //环温

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
/*******     初次上电汇报状态     ***************/
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
/******************判断发送汇报帧程序****************/
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
/*******************故障报警程序*********************/
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
/*******************返回无效值程序***************/
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
/*******************返回ACK程序******************/
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
/******************返回识别码查询帧程序***********/
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
/*******************计算校验和程序***************/
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
	UART_ITConfig(UART3, UART_IT_TB, ENABLE);  //使能发送中断
}

/************************************************/
/*******************接收初始化程序***************/
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
/*******************接收uart0初始化程序***************/
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
函数名称：void EheatConTrolP(void)
输入参数：r_lcad  冷藏温度传感器的AD值
输出参数：
函数功能：
编写日期：2019/4/24 10:16:32
编者：   HW
***************************************************/

void EheatConTrolP(void)//化霜加热丝在这里控制
{
	if (f_lc_sensor_err)//冷藏传感器NTC1错误
	{
		Eheatcontrolpin = 0;//控制加热丝
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
void Pannel_Comm_Deal(void) //@CFJ 面板通讯处理
{
	if (g_Pannel_Comm_bRcv_Done)
	{
		g_Pannel_Comm_bRcv_Done = 0;
		//g_Bus_Rec_Time=0;//正确接受完数据，则2s时间清0；
		//g_Txd_Time = 250 ;                   //500ms计时开始 发送
		g_UART1_RCV_Cyc = 0;
		g_UART1_RCV_Counter = 0;

		g_Bus_Error_Time = 0; //1min时间
		g_Sys_Erflag0_Comm = 0;

		//g_Sys_Stateflag10_bFirst_Rcv_ODM = TURE ;
		//g_RX_LED_Time = 5 ;                       //接收校验和正确 接收灯亮50ms

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
	   while( (float)Ad > Humi_Tab[i][0] ) 
	      i++;
	   Temp  = Humi_Tab[i-1][1] + 5*( Ad - Humi_Tab[i-1][0] )/( Humi_Tab[i][0] - Humi_Tab[i-1][0] ); //浮点型
	   Temp2 = (unsigned char) floor( Temp + 0.5 );  //四舍五入取整
	}
  return Temp2;
}

////湿度显示规则
///*
//最小值40，
//40-65，按照实际数值显示， 
//65-75，数值-5
//75-85，数值-10
//85-95，数值-20
//95以上，数值-25
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

//我定义的显示规则，保证线性变化
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
void l_ODM_Data_Parse(void) //接受主板解析数据 CFJ
{
	unsigned char Lc_Temp, Lc_Temp_2, VoltageTemp;
	unsigned char batteryTemp;
	unsigned int g_HXJ_SD_AD;
	unsigned int lc_Hs_Temp=0;
	float g_HXJ_SD;
	if (g_Sys_ReceiveDataType) //接受数据帧类型  CFJ
	{
		//电压
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
		
		//电池
		batteryTemp = (Pannel_Uart1Data[20] << 4);
		batteryTemp |= (Pannel_Uart1Data[21] >> 4);
		r_battery_ad = batteryTemp;
		//flag_rec1 = Pannel_Uart1Data[6];//无效
		if (!f_first_ad)
		{
			r_voltage_report = r_voltage; //r_voltage_report变量无效
		}
		if (Pannel_Uart1Data[24] & 0x07) //@20181130 CFJ 电池插反 短接 低压故障
		{
			f_battery = 1; //电池故障标志
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
			//冷冻的温度处理，拿到底板检测到的冷冻温度AD值
			r16_ldad = Pannel_Uart1Data[13]; //高位//rec[3]; 对应底板byte[17]
			r16_ldad = r16_ldad << 8;		 //r16_ldad<<8;
			r16_ldad = (r16_ldad & 0xff00) | Pannel_Uart1Data[14];
			r16_ldad >>= 2; //12位AD转换为10位AD
			if ((r16_ldad <= 1010) && (r16_ldad >= 15))
			{
				//这个括号内程序没有全部对区间处理,但不影响程序正确性
				if (SelfCheckFlag == 0)//当前程序正常运行，费自检模式，自检时SelfCheckFlag=1
				{
					if ((r_ldwdjz >= 20) && (r16_ldad > 35)) //温度校正是正数&&温度小于等于50 r_ldwdjz这个可以通过通信过来，应该也可以通过设置
					{
						r16_ldad = r16_ldad - (r_ldwdjz - 20) * 4; //校准上偏10.温度对应4个AD
					}
					else if ((r_ldwdjz < 20) && (r16_ldad < 990))
					{
						r16_ldad = r16_ldad + (20 - r_ldwdjz) * 4;
					}
				}
			}

			if (r16_ldad > 470)
			{
				r_ldsjwd = 140; //-60度
			}
			else if (r16_ldad < 36)
			{
				r_ldsjwd = 250; //50度
			}
			else
			{
				r_ldsjwd = (tab_temperature[r16_ldad - 36]);
			}
		#else//显示湿度
			//拿到底板检测到的湿度AD值
			r16_ldad = Pannel_Uart1Data[17]; //高位//rec[3]; 对应底板byte[21]
			r16_ldad = r16_ldad << 8;		 //r16_ldad<<8;
			r16_ldad = (r16_ldad & 0xff00) | Pannel_Uart1Data[18];
			//r16_ldad >>= 2; //12位AD转换为10位AD
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

		    //张国帆定义了显示规则
		    g_HXJ_SD=Humidity_DisRule(&g_HXJ_SD);
			
			r_ldsjwd = (u8t)g_HXJ_SD;

							//这个括号内程序没有全部对区间处理,但不影响程序正确性
			if (SelfCheckFlag == 0)//当前程序正常运行，费自检模式，自检时SelfCheckFlag=1
			{
				if ((r_ldwdjz >= 20)) //温度校正是正数&&温度小于等于50 r_ldwdjz这个可以通过通信过来，应该也可以通过设置
				{
						r_ldsjwd=r_ldsjwd+(r_ldwdjz-20);
						if(r_ldsjwd>100){r_ldsjwd=100;}
				}
				else if ((r_ldwdjz < 20) && (r_ldwdjz>=5))
				{
						
						if((r_ldsjwd-(20-r_ldwdjz))<=0){
							r_ldsjwd=0;
						}else{
							r_ldsjwd=r_ldsjwd-(20-r_ldwdjz);
						}
				}
			}
		#endif


		//冷藏温度的处理,没有温度显示在50~-60度的限制
		Lc_Temp = Pannel_Uart1Data[5] << 4;  //NTC1的数据冷藏控制传感器
		Lc_Temp |= Pannel_Uart1Data[6] >> 4;

		Lc_Temp_2 = Pannel_Uart1Data[7] << 4; //NTC2的数据 冷藏显示传感器
		Lc_Temp_2 |= Pannel_Uart1Data[8] >> 4; // 20190410  HYC-386项目修改为NTC2作为冷藏传感器HW
//		r_lcad = Lc_Temp;//控制传感器 冷藏AD值 这个值只用于控制当前温度比如设置为5度时，高于5.5度开压机。低于4.5度关压机
		
		
//		if (Lc_Temp >= 250 || Lc_Temp <= 10) //温度大于60 小于-20认为错误,兼容NTC1与NTC2
//		{
				 //r_lcXsad = Lc_Temp_2;
				 r_lcXsad=Lc_Temp_2;
         r_lcad_12b_c = Pannel_Uart1Data[7] * 256 + Pannel_Uart1Data[8];
//		}
//		else
//		{
			r_lcad_8b = Lc_Temp;
      r_lcad_12b = Pannel_Uart1Data[5] * 256 + Pannel_Uart1Data[6];
//		}

        /*8位ad偏移处理*/
		if ((r_lcad_8b < 250) && (r_lcad_8b > 5))
		{
			if (SelfCheckFlag == 0)
			{
				if ((r_lcwdjz >= 100) && (r_lcad_8b > 20)) //温度校正
				{
					r_lcad_8b = r_lcad_8b - ((r_lcwdjz - 100) / 10.0) * 3;
				}
				else if ((r_lcwdjz < 10) && (r_lcad_8b < 235))
				{
					r_lcad_8b = r_lcad_8b + ((100 - r_lcwdjz) / 10.0) * 3;
				}
			}
		}
		r_lcad_8b_c = CheckLcTable(r_lcXsad);   //整数温度现在只用于测试模式
		r_lcwd_float_c = (float)CheckLcTable_12b(r_lcad_12b_c) + (r_lcxswdjz - 100) / 10.0;;   //
		r_lcwd = CheckLcTable(r_lcad_8b);   //整数温度现在只用于测试模式
		r_lcwd_float = (float)CheckLcTable_12b(r_lcad_12b) + (r_lcwdjz - 100) / 10.0;
        if(r_lcwd_float < 8.0)
            r_lcwd_float = 8.0;
        if(r_lcwd_float > 98.0)
            r_lcwd_float = 98.0;
				

		//拿化霜传感器的AD值
		lc_Hs_Temp=0;
		lc_Hs_Temp = Pannel_Uart1Data[9]; //对应底板byte[13]
		lc_Hs_Temp = lc_Hs_Temp << 8;		 //r16_ldad<<8;
		r16_lcHuaShuangad = (lc_Hs_Temp & 0xff00) | Pannel_Uart1Data[10];
		r_lcHuaSHuangxswd_float=ADC2Temper_3840(r16_lcHuaShuangad);//拿到温度0值*10+510的值
		if(r_lcHuaSHuangxswd_float>=(510+990)){r_lcHuaSHuangxswd_float=510+990;}
		//如果返回值<510是个负数，否则就是个正数
		//例如当返回值为400时，对应温度应为-(510-400)/10度(-11度),当返回值为600时，对应温度为+(600-510)/10度(+9度)
		r_lcHuaShuangsjwd=((int)(r_lcHuaSHuangxswd_float-510))/10;
//		if(r_lcHuaSHuangxswd_float>510){//0度以上
//			r_lcHuaShuangsjwd=(int)((r_lcHuaSHuangxswd_float-510)/10);
//		}else{//
//			r_lcHuaShuangsjwd=-(int)((510-r_lcHuaSHuangxswd_float)/10);
//		}
		
		//拿冷凝传感器的AD值
		lc_Hs_Temp=0;
		lc_Hs_Temp = Pannel_Uart1Data[11]; //对应底板byte[13]
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
温度校正现在用在了冷藏的控制传感器上，我把它用在显示传感器上
		if ((r_lcad < 250) && (r_lcad > 5))
		{//冷藏控制传感器温度误差矫正
			if (SelfCheckFlag == 0)
			{//当前为正常程序运行，非自检模式
				if ((r_lcwdjz >= 10) && (r_lcad > 20)) //温度校正
				{//温度1度大约对应3个ad，r_lcwdjz的范围是5-15代表正负5度偏移
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
//		{//冷藏控制传感器温度误差矫正
//			if (SelfCheckFlag == 0)
//			{//当前为正常程序运行，非自检模式
//				if ((r_lcwdjz >= 10) && (r_lcad > 5)) //温度校正
//				{//温度1度大约对应3个ad，r_lcwdjz的范围是5-15代表正负5度偏移 AD值与温度为反比关系
//					r_lcwd = CheckLcTable(r_lcad) + (r_lcwdjz - 10);//使用AD值获取到温度值 -24以后上偏38为温度例如AD值(116+24)代表2度
//					//r_lcad = r_lcad - (r_lcwdjz - 10) * 3;
//				}
//				else if ((r_lcwdjz < 10) && (r_lcad < 250))
//				{
//					r_lcwd = CheckLcTable(r_lcad) - (10 - r_lcwdjz);//使用AD值获取到温度值 -24以后上偏38为温度例如AD值(116+24)代表2度

//					//r_lcad = r_lcad + (10 - r_lcwdjz) * 3;
//				}
//				if(r_lcwd<=8){r_lcwd = 8;}
//				if(r_lcwd>=98){r_lcwd = 98;}
//			}
//		}else{
//			r_lcwd = CheckLcTable(r_lcad);
//		}

//		if ((Lc_Temp_2 < 250) && (Lc_Temp_2 > 5))
//		{//冷藏显示传感器温度误差矫正
//			if (SelfCheckFlag == 0)
//			{//当前为正常程序运行，非自检模式
//				if ((r_lcxswdjz >= 10) && (Lc_Temp_2 > 5)) //温度校正
//				{//温度1度大约对应3个ad，r_lcwdjz的范围是5-15代表正负5度偏移 AD值与温度为反比关系
//					//Lc_Temp_2 = Lc_Temp_2 - (r_lcxswdjz - 10) * 3;
//					//r_lcwd_float = (float)CheckLcTable(Lc_Temp_2) + (r_lcxswdjz - 10);//jhw，只作为显示使用
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
//		if (Lc_Temp_2 >= 250 || Lc_Temp_2 <= 5) //温度大于60 小于-30认为错误,使用ntc1控制传感器 刚上电时有这个问题//TODO
//		{
//			//Lc_Temp_2 = r_lcad;
//			r_lcwd_float=(float)r_lcwd;
//		}
		
//		//r_lcad = 210; // TEST 测试 @20181025 CFJ
//		r_lcwd = CheckLcTable(r_lcad);//使用AD值获取到温度值 -24以后上偏38为温度例如AD值(116+24)代表2度
////		r_lcwd_float = (float)CheckLcTable(r_lcad);//老王添加，为了在LED上显示小数点后的温度(假温度)这个变量还用于了超温判断
//		r_lcwd_float = (float)CheckLcTable(Lc_Temp_2);//jhw，只作为显示使用
	}
	f_first_ad = 1;
}

void l_Pannel_DataTx(void)
{
	if (bUartSendStartFlag) //&& (g_Txd_Time <= 0))每500ms搞一把，向电源板发一次数据
	{
		bUartSendStartFlag = 0;
		l_Send_Data_Build();
	}
}



void l_Send_Data_Build(void)
{
	unsigned char NDegree, Sum, Temp;
	l_ClrIDMSendBuf();//清空给所有的标志位
	Sum = 0;
	g_IDM_TX_ODM_Data[0] = 0xAA;

	g_IDM_TX_ODM_Data[1] = 0x55; //  头码4个字节

	g_IDM_TX_ODM_Data[2] = 0xE7;

	g_IDM_TX_ODM_Data[3] = 0x18;
	//----------------------------------//
	g_IDM_TX_ODM_Data[4] = 0xFF; //显示板地址

	g_IDM_TX_ODM_Data[5] = 0xB0; //I/O板地址

	g_IDM_TX_ODM_Data[6] = 0x00; //数据帧类型
	//-----系统设定的温度-------//

	if (r_lczt >= 38) //冷藏设定温度 @20181130 CFJ
	{				  //大于等于0度
		Temp = (r_lczt - 38);
		g_IDM_TX_ODM_Data[7] = Temp;
	}
	else
	{ //0度以下
		Temp = (38 - r_lczt);
		g_IDM_TX_ODM_Data[7] = Temp;
		g_IDM_TX_ODM_Data[7] |= 0x80;
	}

	#ifdef Need_Ld
		if (r_ldzt >= 200) //冷冻设定温度
		{				   //大于等于0度
			Temp = (r_ldzt - 200);
			g_IDM_TX_ODM_Data[8] = Temp;
		}
		else
		{ //0度以下
			Temp = (200 - r_ldzt);
			g_IDM_TX_ODM_Data[8] = Temp;
			g_IDM_TX_ODM_Data[8] |= 0x80;
		}
	#else
		g_IDM_TX_ODM_Data[8] = r_ldzt;
	#endif
	g_IDM_TX_ODM_Data[9] = 0x00; //预留未用

	if (f_lc_fan) //冷藏的冷凝风机，和压缩机一起在机器外面搞
	{
		g_IDM_TX_ODM_Data[10] |= 0x01;
	}

	if (f_ln_fan) //冷冻风机
	{
		g_IDM_TX_ODM_Data[10] |= 0x02;
	}

	if (Eheatcontrolpin)
	{
		g_IDM_TX_ODM_Data[10] |= 0x04; //2019/4/24 HW  电加热丝的控制
	}
	if (f_defrost) //除霜
	{
		g_IDM_TX_ODM_Data[10] |= 0x04;
	}

	if (f_nd_fan) //冷藏的内风机/蒸发风机，在腔体内跑循环风用
	{
		g_IDM_TX_ODM_Data[10] |= 0x08;
	}

	if (f_lc_compressor) //冷藏的压缩机
	{
		g_IDM_TX_ODM_Data[11] |= 0x01;
	}

	if (f_compressor) //冷冻压机
	{
		g_IDM_TX_ODM_Data[11] |= 0x02;
	}

	if (f_LightStatusPin) //12V OUT3作为冷藏灯输出  -20190410-HW
	{
		g_IDM_TX_ODM_Data[12] |= 0x04;
	}

	if (f_long_range_alarm) //远程报警 @20181130  CFJ
	{
		g_IDM_TX_ODM_Data[13] |= 0;
	}
	else
	{
		g_IDM_TX_ODM_Data[13] |= 0x01; //因为底板的远程报警的继电器是常闭的。事业部胡伟要求有故障时，发0 ，无发 1
	}

	g_IDM_TX_ODM_Data[14] = 0x00; //预留
	/*  这个协议位的高温 低温报警不用，已和事业部沟通。 @20181130 CFJ
    if(f_ld_high_buzz)  //远程报警      
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
		g_IDM_TX_ODM_Data[15] |= 0x04; //断电报警
	}
	/* @20181130 CFJ
    if(f_battery)
    {
    	 g_IDM_TX_ODM_Data[15]|=0x08; //电池电量低
    }
    */
	if (f_lc_sensor_err)
	{
		g_IDM_TX_ODM_Data[15] |= 0x40; //g_IDM_TX_ODM_Data[15]|=0x80; //事业部要求NTC2改为NTC1冷藏传感器故障 @20181221 CFJ
	}

	//  g_IDM_TX_ODM_Data[16]|=0x00; 故障未用
//	if (f_ld_sensor_err)
//	{
//		g_IDM_TX_ODM_Data[17] |= 0x01; //冷冻传感器故障
//	}
	if (f_hw_sensor_err)
	{
		g_IDM_TX_ODM_Data[17] |= 0x04; //环温传感器故障
	}
	if (f_hw_high38)
	{
		g_IDM_TX_ODM_Data[17] |= 0x10; //环温过高
	}

	if (f_ld_high_buzz) //if(f_ld_high) @20181130 CFJ
	{
		g_IDM_TX_ODM_Data[18] |= 0x04; //冷冻高温报警 v1.3协议
	}
	if (f_ld_low_buzz) //if(f_ld_low) @20181130 CFJ
	{
		g_IDM_TX_ODM_Data[18] |= 0x08; //冷冻低温报警
	}

	if (f_lc_high_buzz) //if(f_lc_high) @20181130 CFJ
	{
		g_IDM_TX_ODM_Data[18] |= 0x01; //冷藏高温报警
	}

	if (f_lc_low_buzz) //if(f_lc_low) @20181130 CFJ
	{
		g_IDM_TX_ODM_Data[18] |= 0x02; //冷藏低温报警
	}

	if (f_buzz_alarm)
	{
		g_IDM_TX_ODM_Data[19] = 0x01; //蜂鸣报警
	}
	g_IDM_TX_ODM_Data[22] = r_lcad_8b_c;//冷藏控制传感器温度NTC1
	g_IDM_TX_ODM_Data[23] = r16_ldad / 256;
	g_IDM_TX_ODM_Data[24] = r16_ldad % 256;
	g_IDM_TX_ODM_Data[25] = 0xFF; //压缩机1

	g_IDM_TX_ODM_Data[26] = 0xFF; //压缩机2

	/*
    if(!g_Sys_Erflag0_Comm)  //无故障，则上电默认开机
    {
        g_IDM_TX_ODM_Data[2] |=0x80;
    }
    if(f_lc_compressor)  //冷藏压机
    {
        g_IDM_TX_ODM_Data[2] |=0x40;
    } 
    if(f_compressor)  //冷冻压机
    {
        g_IDM_TX_ODM_Data[2] |=0x20;
    
    }
    if(f_lc_fan)  //冷藏风机
    {
        g_IDM_TX_ODM_Data[2] |=0x10;    
    }
    
    if(f_ln_fan)  //冷冻风机
    {
        g_IDM_TX_ODM_Data[2] |=0x08;    
    }
    
    if(f_defrost)  //除霜
    {
        g_IDM_TX_ODM_Data[2] |=0x04;    
    }
    
    if(f_nd_fan)  //内风机
    {
        g_IDM_TX_ODM_Data[2] |=0x02;    
    }
    
    if(g_Compressor_Frequency_Flag1)  //压机1变频/定频选择标志
    {
        g_IDM_TX_ODM_Data[3] |=0x80;  //变频
    }
    else
    {   
         g_IDM_TX_ODM_Data[3]=0xFF;
    }
    if(g_Compressor_Frequency_Flag2)  //压机2变频/定频选择标志
    {
        g_IDM_TX_ODM_Data[4] |=0x80;  //变频    
    }
    else
    {
        g_IDM_TX_ODM_Data[4]=0xFF;
    }
    
    if(f_long_range_alarm)  //远程报警
    {
        g_IDM_TX_ODM_Data[5]|=0x80;
    }
    //后备输出 默认有
     g_IDM_TX_ODM_Data[5]|=0x40;
    
    if(f_buzz_alarm)
    {
        g_IDM_TX_ODM_Data[5]|=0x20;
    }
    
    //12v 负载 4路都输出  D4 D3 D2 D1
    
    g_IDM_TX_ODM_Data[5]|=0x00;
    
    //预留
  
    g_IDM_TX_ODM_Data[6] = 0x00;
    g_IDM_TX_ODM_Data[7] = 0x00;
    
    //校验和字节
    g_IDM_TX_ODM_Data[8]=0;  
    */
	for (NDegree = 4; NDegree <= 28; NDegree++)
	{
		Sum += g_IDM_TX_ODM_Data[NDegree];
	}
	g_IDM_TX_ODM_Data[29] = Sum;
	g_UART1_TXD_Counter = 0;
	RS_485_STATUS_SET_HIGH; //发送有效
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
	l_ucTemp = 0;
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
				if (SelfCheckFlag != 0) //错误
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

	while (SelfCheckFlag != 0) //进入自检
	{
		/*
		    	 if((!f_first_ad)||(!f_Self_first_ad))
           {
		            t_Self_Times = t_20ms;
temp:		        IWDT_Clear();
				        l_Pannel_Self_DataTx();//自检时面板发送数据
				        Pannel_Comm_Deal();//这里需要判断PT100和NTC1的数据。
				        if((unsigned char)(t_20ms-t_Self_Times)>=25)//每2秒主动发一次数据给底板。
				        {
				        	 bSelfUartSendStartFlag=1;
				        	 t_Self_Times = t_20ms;		        	
				        }
				        if((!f_first_ad)||(!f_Self_first_ad)) //当电池的电电压和PT100数据都收到，再往下判断
				        {
				        	 goto temp;
				        }
				    }
				    */
		/*
		        if(r16_ldad>1010||r16_ldad<15) //冷冻传感器故障
						{
							if(!f_ld_sensor_err)//如果出现冷冻传感器故障,进入一次,以后不再进入
							{
							  f_ld_sensor_err = 1;
							  f_stop_alarm = 0;
							}
						}
						//还需判断冷藏传感器故障，		
						//当传感器有故障时，则不再进入自检，显示故障代码或者温度。
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
			r_led11 = 0xFF; //冷冻 全显
			r_led12 = 0xFF;
			r_led13 = 0xFF;

			r_led21 = 0xFF; //冷藏 全显
			r_led22 = 0xFF;
			r_led23 = 0xFF;
			// LCD267 = 0xFF;  //all led on
			// LCD245 = 0xFF;
			// LCD1621[0] = 0xFF;
			// LCD1621[1] = 0xFF;
			break;
		case 2:
			r_led11 = 0x00; //table_led[DISP_NO]; //冷冻 全关
			r_led12 = 0x00; //table_led[DISP_NO];
			r_led13 = 0x00; //table_led[DISP_NO];

			r_led21 = 0x00; //table_led[DISP_NO]; //冷藏 全关
			r_led22 = 0x00; //table_led[DISP_NO];
			r_led23 = 0x00; //table_led[DISP_NO];
			//LCD267 = 0;        //alL  led off
			//LCD245 = 0;
			//LCD1621[1] = 0x00;
			//INT_LED595( 0 );
			break;
		case 3:						//定时 09度
			r_led21 = table_led[8]; // 冷藏数码管最左边 显示8
			//LCD1621[0] = 0x80;
			//LCD245 = LCDTABG_EPK[9];
			//LCD267 = LCDTABG_EPK[0];
			//LCD1621[1] = 0xFF;
			break;
		case 4: //睡眠 18度
			r_led21 = table_led[DISP_NO];
			r_led22 = table_led[8];
			//LCD1621[0] = 0x02;
			//LCD245 = LCDTABG_EPK[8];
			//LCD267 = LCDTABG_EPK[1];
			break;
		case 5: //运行灯 健康 27度
			r_led22 = table_led[DISP_NO];
			r_led23 = table_led[8];
			//LCD245 = LCDTABG_EPK[7];
			//LCD267 = LCDTABG_EPK[2];
			break;
		case 6: //健康灯 除湿36度
			r_led23 = table_led[DISP_NO];
			r_led11 = table_led[8];
			//LCD1621[0] = 0x00;
			//LCD245 = LCDTABG_EPK[6];
			// LCD267 = LCDTABG_EPK[3];
			break;
		case 7: //全灭 定时 45度
			r_led11 = table_led[DISP_NO];
			r_led12 = table_led[8];
			//LCD1621[0] = 0x00;
			//LCD245 = LCDTABG_EPK[5];
			//LCD267 = LCDTABG_EPK[4];
			break;
		case 8: //显示灭步进电机依次点亮
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
			f_lc_fan = 1; //冷藏风机
			//IO_STEPMOTOR_A = 0;
			//IO_STEPMOTOR_B = 1;
			break;
		case 10:
			f_lc_fan = 0;
			f_ln_fan = 1; //冷冻风机
			//IO_STEPMOTOR_B = 0;
			//IO_STEPMOTOR_C = 1;
			break;
		case 11: //除霜继电器
			f_ln_fan = 0;
			f_defrost = 1;

			//IO_STEPMOTOR_C = 0;
			//IO_STEPMOTOR_D = 1;
			break;
		case 12:		   //
			f_defrost = 0; //除霜
			f_nd_fan = 1;
			break;
		case 13: //四通 冷冻温度显示
			f_nd_fan = 0;
			f_nd_AcLowfan = 1;
			//IO_COMPRESSOR = 0;
			//IO_4WAYVALVE = 1;
			break;
		case 14: //负离子
			f_nd_AcLowfan = 0;
			f_lc_compressor = 1; //冷藏压机

			//冷冻压机输出
			//IO_4WAYVALVE = 0;
			///IO_NEGATIVE_ULTRA = 1;
			break;
		case 15:				 //冷藏温度显示
			f_lc_compressor = 0; //冷藏压机
			f_compressor = 1;	//冷冻压机

			//IO_NEGATIVE_ULTRA = 0;
			//IO_OUTFAN = 1;
			break;
		case 16:
			f_compressor = 0; //冷藏控制传感器温度显示
			if (r_lcwd < 41 || r_lcwd > 47)
			{
				r_bw = DISP_NO;
				r_sw = DISP_E;
				r_gw = 3;
			}
			else
			{
				if (r_lcwd >= 38)
				{
					r_bw = DISP_NO;
					r_sw = (uchar)((r_lcwd - 38) / 10);
					r_gw = (uchar)((r_lcwd - 38) % 10);
				}
				else
				{
					r_bw = DISP_FH; //-
					r_sw = (uchar)((38 - r_lcwd) / 10);
					r_gw = (uchar)((38 - r_lcwd) % 10);
				}
			}
			r_led21 = table_led[r_bw];
			r_led22 = table_led[r_sw];
			r_led23 = table_led[r_gw];

			//IO_OUTFAN = 0;
			//IO_ELECHEAT = 1;
			break;
		case 17: //冷冻温度显示
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
		case 18: //实际电压显示
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
			r_led11 = table_led[DISP_NO]; //冷冻 全关
			r_led12 = table_led[DISP_NO];
			r_led13 = table_led[DISP_NO];

			r_led21 = table_led[DISP_NO]; //冷藏 全关
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
			//SelfCheckFlag = 0; @20190228 市电校准完结束，才退出自检
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
		//g_Txd_Time=250; //自检结束后，500ms再重新发数据。
		DataToLed();
		if (SelfCheckStep >= 21)
		{
			do //SW1退出 @20190228 CFJ
			{
				l_ucTemp = 0;
				while (l_uci != 0)
				{
					if (KEY_SW1 == 0) //(IO_SHORTTIME == 0)
					{
						//if ( SelfCheckFlag != 0 ) //错误
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

	g_IDM_TX_ODM_Data[1] = 0x55; //  头码4个字节

	g_IDM_TX_ODM_Data[2] = 0xE7;

	g_IDM_TX_ODM_Data[3] = 0x18;
	//----------------------------------//
	g_IDM_TX_ODM_Data[4] = 0xFF; //显示板地址

	g_IDM_TX_ODM_Data[5] = 0xB0; //I/O板地址

	g_IDM_TX_ODM_Data[6] = 0x00; //数据帧类型
	//-----系统设定的温度-------//

	if (r_lczt >= 38) //冷藏设定温度 @20181130 CFJ
	{				  //大于等于0度
		Temp = (r_lczt - 38);
		g_IDM_TX_ODM_Data[7] = Temp;
	}
	else
	{ //0度以下
		Temp = (38 - r_lczt);
		g_IDM_TX_ODM_Data[7] = Temp;
		g_IDM_TX_ODM_Data[7] |= 0x80;
	}
#ifdef Need_ld
	if (r_ldzt >= 200) //冷冻设定温度
	{				   //大于等于0度
		Temp = (r_ldzt - 200);
		g_IDM_TX_ODM_Data[8] = Temp;
	}
	else
	{ //0度以下
		Temp = (200 - r_ldzt);
		g_IDM_TX_ODM_Data[8] = Temp;
		g_IDM_TX_ODM_Data[8] |= 0x80;
	}
#else
	g_IDM_TX_ODM_Data[8] = r_ldzt;
#endif
	g_IDM_TX_ODM_Data[9] = 0x00; //预留未用

	if (f_lc_fan) //冷藏风机
	{
		g_IDM_TX_ODM_Data[10] |= 0x01;
	}

	if (f_ln_fan) //冷冻风机
	{
		g_IDM_TX_ODM_Data[10] |= 0x02;
	}

	if (f_defrost) //除霜
	{
		g_IDM_TX_ODM_Data[10] |= 0x04;
	}

	if (f_nd_fan) //内风机 门开关关上的风机
	{
		g_IDM_TX_ODM_Data[10] |= 0x08;
	}
	if (f_nd_AcLowfan)
	{
		g_IDM_TX_ODM_Data[10] |= 0x10; //@20190222 CFJ
	}
	if (f_lc_compressor) //冷藏压机
	{
		g_IDM_TX_ODM_Data[11] |= 0x01;
	}

	if (f_compressor) //冷冻压机
	{
		g_IDM_TX_ODM_Data[11] |= 0x02;
	}

	g_IDM_TX_ODM_Data[12] = 0x00; //12V输出控制未用到，在此发0.

	if (f_long_range_alarm) //远程报警 @20181130  CFJ
	{
		g_IDM_TX_ODM_Data[13] |= 0;
	}
	else
	{
		g_IDM_TX_ODM_Data[13] |= 0x01; //因为底板的远程报警的继电器是常闭的。事业部胡伟要求有故障时，发0 ，无发 1
	}

	g_IDM_TX_ODM_Data[14] = 0x00; //预留
	/*  这个协议位的高温 低温报警不用，已和事业部沟通。 @20181130 CFJ
    if(f_ld_high_buzz)  //远程报警      
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
		g_IDM_TX_ODM_Data[15] |= 0x00; //0x04; //断电报警
	}
	/* @20181130 CFJ
    if(f_battery)
    {
    	 g_IDM_TX_ODM_Data[15]|=0x08; //电池电量低
    }
    */
	if (f_lc_sensor_err)
	{
		g_IDM_TX_ODM_Data[15] |= 0x00; //0x40;//g_IDM_TX_ODM_Data[15]|=0x80; //事业部要求NTC2改为NTC1冷藏传感器故障 @20181221 CFJ
	}

	//  g_IDM_TX_ODM_Data[16]|=0x00; 故障未用
//	if (f_ld_sensor_err)
//	{
//		g_IDM_TX_ODM_Data[17] |= 0x00; //0x01; //冷冻传感器故障
//	}
	if (f_hw_sensor_err)
	{
		g_IDM_TX_ODM_Data[17] |= 0x00; //0x04; //环温传感器故障
	}
	if (f_hw_high38)
	{
		g_IDM_TX_ODM_Data[17] |= 0x00; //0x10; //环温过高
	}

	if (f_ld_high_buzz) //if(f_ld_high) @20181130 CFJ
	{
		g_IDM_TX_ODM_Data[18] |= 0x00; //0x04; //冷冻高温报警 v1.3协议
	}
	if (f_ld_low_buzz) //if(f_ld_low) @20181130 CFJ
	{
		g_IDM_TX_ODM_Data[18] |= 0x00; //0x08; //冷冻低温报警
	}

	if (f_lc_high_buzz) //if(f_lc_high) @20181130 CFJ
	{
		g_IDM_TX_ODM_Data[18] |= 0x00; //0x01; //冷藏高温报警
	}

	if (f_lc_low_buzz) //if(f_lc_low) @20181130 CFJ
	{
		g_IDM_TX_ODM_Data[18] |= 0x00; //0x02; //冷藏低温报警
	}

	if (f_buzz_alarm)
	{
		g_IDM_TX_ODM_Data[19] = 0x00; //0x01;  //蜂鸣报警
	}

	g_IDM_TX_ODM_Data[25] = 0xFF; //压缩机1

	g_IDM_TX_ODM_Data[26] = 0xFF; //压缩机2

	/*
    if(!g_Sys_Erflag0_Comm)  //无故障，则上电默认开机
    {
        g_IDM_TX_ODM_Data[2] |=0x80;
    }
    if(f_lc_compressor)  //冷藏压机
    {
        g_IDM_TX_ODM_Data[2] |=0x40;
    } 
    if(f_compressor)  //冷冻压机
    {
        g_IDM_TX_ODM_Data[2] |=0x20;
    
    }
    if(f_lc_fan)  //冷藏风机
    {
        g_IDM_TX_ODM_Data[2] |=0x10;    
    }
    
    if(f_ln_fan)  //冷冻风机
    {
        g_IDM_TX_ODM_Data[2] |=0x08;    
    }
    
    if(f_defrost)  //除霜
    {
        g_IDM_TX_ODM_Data[2] |=0x04;    
    }
    
    if(f_nd_fan)  //内风机
    {
        g_IDM_TX_ODM_Data[2] |=0x02;    
    }
    
    if(g_Compressor_Frequency_Flag1)  //压机1变频/定频选择标志
    {
        g_IDM_TX_ODM_Data[3] |=0x80;  //变频
    }
    else
    {   
         g_IDM_TX_ODM_Data[3]=0xFF;
    }
    if(g_Compressor_Frequency_Flag2)  //压机2变频/定频选择标志
    {
        g_IDM_TX_ODM_Data[4] |=0x80;  //变频    
    }
    else
    {
        g_IDM_TX_ODM_Data[4]=0xFF;
    }
    
    if(f_long_range_alarm)  //远程报警
    {
        g_IDM_TX_ODM_Data[5]|=0x80;
    }
    //后备输出 默认有
     g_IDM_TX_ODM_Data[5]|=0x40;
    
    if(f_buzz_alarm)
    {
        g_IDM_TX_ODM_Data[5]|=0x20;
    }
    
    //12v 负载 4路都输出  D4 D3 D2 D1
    
    g_IDM_TX_ODM_Data[5]|=0x00;
    
    //预留
  
    g_IDM_TX_ODM_Data[6] = 0x00;
    g_IDM_TX_ODM_Data[7] = 0x00;
    
    //校验和字节
    g_IDM_TX_ODM_Data[8]=0;  
    */
	for (NDegree = 4; NDegree <= 28; NDegree++)
	{
		Sum += g_IDM_TX_ODM_Data[NDegree];
	}
	g_IDM_TX_ODM_Data[29] = Sum;
	g_UART1_TXD_Counter = 0;
	RS_485_STATUS_SET_HIGH; //发送有效
	g_Rec_Status_Flag = 0;
	UART_ITConfig(UART4, UART_IT_TB, ENABLE);//使能发送中断
}

void l_Pannel_Self_DataTx(void)
{
	if (bSelfUartSendStartFlag) //@20190215 CFJ
	{
		bSelfUartSendStartFlag = 0;
		l_Self_Send_Data_Build();
	}
}
