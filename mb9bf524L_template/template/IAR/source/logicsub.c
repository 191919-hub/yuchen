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
#include "pdl_header.h"
#include "math.h"
#include "Program_Cfg.h" //程序功能配置，比如禁止蜂鸣器、控制冷藏温度分辨率
#include "Coupler.h"
#include "esp8266.h"
//#define nop asm("nop")

//uchar u8_Send_Print_Time;

//#pragma CODE_SEG DEFAULT

#define HEATONAD 2130		 //2.5℃
#define HEATOFFAD 2110		 //3.0℃
#define LCCOMPRTSTOREAD 1869 //7.8℃

float r_lcwd_float = 0, r_lcgzwd_float = 0, r_lczt_float = 0, r_lcxswd_float = 0;
unsigned char flag_Now_Disp_LCWD = 0, flag_Disp_LCWD_D = 0;
unsigned char Cnt_SetTime_Sended = 0;
unsigned char f_Set_Time = 0;  //设置时间标志
unsigned char f_door_state = 0;  //门状态，0：关，1：开

/***********************************************/
#if 0
void Initial(void) 
{
  /* ### MC9S08JM60_64 "Cpu" init code ... */
  /*  PE initialization code after reset */
  /* Common initialization of the write once registers */
  /* SOPT1: COPT=3,STOPE=0 */
  SOPT1 = 0xD3;                                      
  /* SOPT2: COPCLKS=1,COPW=0,SPI1FE=1,SPI2FE=1,ACIC=0 */
  SOPT2 = 0x86;                                      
  /* SPMSC1: LVWF=0,LVWACK=0,LVWIE=0,LVDRE=1,LVDSE=1,LVDE=1,BGBE=0 */
  SPMSC1 = 0x1C;                                      
  /* SPMSC2: LVDV=0,LVWV=0,PPDF=0,PPDACK=0,PPDC=0 */
  SPMSC2 = 0x00;                                      
  /*  System clock initialization */
  MCGTRM = *(unsigned char*far)0xFFAF; /* Initialize MCGTRM register from a non volatile memory */
  MCGSC = *(unsigned char*far)0xFFAE;  /* Initialize MCGSC register from a non volatile memory */
  /* MCGC2: BDIV=0,RANGE=1,HGO=0,LP=0,EREFS=1,ERCLKEN=0,EREFSTEN=0 */
  MCGC2 = 0x24;                        /* Set MCGC2 register */
  /* MCGC1: CLKS=2,RDIV=7,IREFS=0,IRCLKEN=0,IREFSTEN=0 */
  MCGC1 = 0xB8;                        /* Set MCGC1 register */
  while(!MCGSC_OSCINIT) {              /* Wait until external reference is stable */
   SRS = 0x55;                         /* Reset watchdog counter write 55, AA */
   SRS = 0xAA;
  }
  while(MCGSC_IREFST) {                /* Wait until external reference is selected */
   SRS = 0x55;                         /* Reset watchdog counter write 55, AA */
   SRS = 0xAA;
  }
  while((MCGSC & 0x0C) != 0x08) {      /* Wait until external clock is selected as a bus clock reference */
   SRS = 0x55;                         /* Reset watchdog counter write 55, AA */
   SRS = 0xAA;
  }
  /* MCGC2: BDIV=0,RANGE=1,HGO=0,LP=1,EREFS=1,ERCLKEN=0,EREFSTEN=0 */
  MCGC2 = 0x2C;                        /* Set MCGC2 register */
  /* MCGC1: CLKS=2,RDIV=1,IREFS=0,IRCLKEN=0,IREFSTEN=0 */
  MCGC1 = 0x88;                        /* Set MCGC1 register */
  /* MCGC3: LOLIE=0,PLLS=1,CME=0,VDIV=6 */
  MCGC3 = 0x46;                        /* Set MCGC3 register */
  /* MCGC2: LP=0 */
  MCGC2 &= (unsigned char)~0x08;                     
  while(!MCGSC_PLLST) {                /* Wait until PLL is selected */
   SRS = 0x55;                         /* Reset watchdog counter write 55, AA */
   SRS = 0xAA;
  }
  while(!MCGSC_LOCK) {                 /* Wait until PLL is locked */
   SRS = 0x55;                         /* Reset watchdog counter write 55, AA */
   SRS = 0xAA;
  }
  /* MCGC1: CLKS=0,RDIV=1,IREFS=0,IRCLKEN=0,IREFSTEN=0 */
  MCGC1 = 0x08;                        /* Set MCGC1 register */
  while((MCGSC & 0x0C) != 0x0C) {      /* Wait until PLL clock is selected as a bus clock reference */
   SRS = 0x55;                         /* Reset watchdog counter write 55, AA */
   SRS = 0xAA;
  }
  
  /* Common initialization of the CPU registers */
  /* PTASE: PTASE5=1,PTASE4=1,PTASE3=1,PTASE2=1,PTASE1=1,PTASE0=1 */
  PTASE |= (unsigned char)0x3F;                               
  /* PTBSE: PTBSE7=1,PTBSE6=1,PTBSE5=1,PTBSE4=1,PTBSE3=1,PTBSE2=1,PTBSE1=1,PTBSE0=1 */
  PTBSE = 0xFF;                                      
  /* PTCSE: PTCSE6=1,PTCSE5=1,PTCSE4=1,PTCSE3=1,PTCSE2=1,PTCSE1=1,PTCSE0=1 */
  PTCSE |= (unsigned char)0x7F;                               
  /* PTDSE: PTDSE7=1,PTDSE6=1,PTDSE5=1,PTDSE4=1,PTDSE3=1,PTDSE2=1,PTDSE1=1,PTDSE0=1 */
  PTDSE = 0xFF;                                      
  /* PTESE: PTESE7=1,PTESE6=1,PTESE5=1,PTESE4=1,PTESE3=1,PTESE2=1,PTESE1=1,PTESE0=1 */
  PTESE = 0xFF;                                      
  /* PTFSE: PTFSE7=1,PTFSE6=1,PTFSE5=1,PTFSE4=1,PTFSE3=1,PTFSE2=1,PTFSE1=1,PTFSE0=1 */
  PTFSE = 0xFF;                                      
  /* PTGSE: PTGSE5=1,PTGSE4=1,PTGSE3=1,PTGSE2=1,PTGSE1=1,PTGSE0=1 */
  PTGSE |= (unsigned char)0x3F;                               
  /* PTADS: PTADS5=0,PTADS4=0,PTADS3=0,PTADS2=0,PTADS1=0,PTADS0=0 */
  PTADS = 0x00;                                      
  /* PTBDS: PTBDS7=0,PTBDS6=0,PTBDS5=0,PTBDS4=0,PTBDS3=0,PTBDS2=0,PTBDS1=0,PTBDS0=0 */
  PTBDS = 0x00;                                      
  /* PTCDS: PTCDS6=0,PTCDS5=0,PTCDS4=0,PTCDS3=0,PTCDS2=0,PTCDS1=0,PTCDS0=0 */
  PTCDS = 0x00;                                      
  /* PTDDS: PTDDS7=0,PTDDS6=0,PTDDS5=0,PTDDS4=0,PTDDS3=0,PTDDS2=0,PTDDS1=0,PTDDS0=0 */
  PTDDS = 0x00;                                      
  /* PTEDS: PTEDS7=0,PTEDS6=0,PTEDS5=0,PTEDS4=0,PTEDS3=0,PTEDS2=0,PTEDS1=0,PTEDS0=0 */
  PTEDS = 0x00;                                      
  /* PTFDS: PTFDS7=0,PTFDS6=0,PTFDS5=0,PTFDS4=0,PTFDS3=0,PTFDS2=0,PTFDS1=0,PTFDS0=0 */
  PTFDS = 0x00;                                      
  /* PTGDS: PTGDS5=0,PTGDS4=0,PTGDS3=0,PTGDS2=0,PTGDS1=0,PTGDS0=0 */
  PTGDS = 0x00;                                      
  /* ### Init_COP init code */
  SRS = 0x55;                          /* Clear WatchDog counter - first part */
  SRS = 0xAA;                          /* Clear WatchDog counter - second part */
 
  /* ### Init_TPM init code */
  /* TPM2SC: TOF=0,TOIE=0,CPWMS=0,CLKSB=0,CLKSA=0,PS2=0,PS1=0,PS0=0 */
  TPM2SC = 0x00;                       /* Stop and reset counter */
  TPM2MOD = 0x05DCU;                   /* Period value setting */
  (void)(TPM2SC == 0);                 /* Overflow int. flag clearing (first part) */
  /* TPM1SC: TOF=0,TOIE=1,CPWMS=0,CLKSB=0,CLKSA=1,PS2=1,PS1=0,PS0=0 */
  TPM2SC = 0x4C;                       /* Int. flag clearing (2nd part) and timer control register setting */
  /* ### */
  
  PTFD  = 0b11111111;
  PTFDD = 0b11100000;
  PTDD  = 0b00011001;
  PTDDD = 0b00000100;
  PTCD  = 0b00101111;
  PTCDD = 0b00101101;
  PTBD  = 0b11111000;
  PTBDD = 0b01111000; 
  PTGD  = 0b00000011;
  PTGDD = 0b00000011;                          
   
  /* ### Init_ADC init code */
  /* APCTL2: ADPC11=0,ADPC10=0,ADPC9=0,ADPC8=0 */
  APCTL2 = 0x04;                                      
  /* ADCCFG: ADLPC=0,ADIV1=1,ADIV0=1,ADLSMP=0,MODE1=0,MODE0=0,ADICLK1=0,ADICLK0=0 */
  ADCCFG = 0x60;                                      
  /* ADCSC2: ADACT=0,ADTRG=0,ACFE=0,ACFGT=0 */
  ADCSC2 = 0x00;                                      
  /* ADCCV: ADCV11=0,ADCV10=0,ADCV9=0,ADCV8=0,ADCV7=0,ADCV6=0,ADCV5=0,ADCV4=0,ADCV3=0,ADCV2=0,ADCV1=0,ADCV0=0 */
  ADCCV =  0x00;                                      
  /* ADCSC1: COCO=0,AIEN=0,ADCO=0,ADCH4=1,ADCH3=1,ADCH2=1,ADCH1=1,ADCH0=1 */
  ADCSC1 = 0x0A;                                                                                     


  SCI1C2 = 0x00;                       /* Disable the SCI1 module */
  (void)(SCI1S1 == 0);                 /* Dummy read of the SCI1S1 registr to clear flags */
  (void)(SCI1D == 10);                 /* Dummy read of the SCI1D registr to clear flags */
  /* SCI1S2: LBKDIF=1,RXEDGIF=1,RXINV=0,RWUID=0,BRK13=0,LBKDE=0,RAF=0 */
  SCI1S2 = 0xC0;                                      
  /* SCI1BDH: LBKDIE=0,RXEDGIE=0,SBR12=0,SBR11=0,SBR10=0,SBR9=0,SBR8=0 */
  SCI1BDH = 0x00;                                      
  /* SCI1BDL: SBR7=1,SBR6=0,SBR5=0,SBR4=1,SBR3=1,SBR2=1,SBR1=0,SBR0=0 */
  SCI1BDL = 0x9C;                                      
  /* SCI1C1: LOOPS=0,SCISWAI=0,RSRC=0,M=0,WAKE=0,ILT=0,PE=0,PT=0 */
  SCI1C1 = 0x00;                                      
  /* SCI1C3: R8=0,T8=0,TXDIR=1,TXINV=0,ORIE=0,NEIE=0,FEIE=0,PEIE=0 */
  SCI1C3 = 0x00;                                      
  /* SCI1C2: TIE=0,TCIE=0,RIE=1,ILIE=0,TE=1,RE=1,RWU=0,SBK=0 */
  SCI1C2 = 0x2c;  


  SCI2C2 = 0x00;                       /* Disable the SCI1 module */
  (void)(SCI2S1 == 0);                 /* Dummy read of the SCI1S1 registr to clear flags */
  (void)(SCI2D == 10);                 /* Dummy read of the SCI1D registr to clear flags */
  /* SCI1S2: LBKDIF=1,RXEDGIF=1,RXINV=0,RWUID=0,BRK13=0,LBKDE=0,RAF=0 */
  SCI2S2 = 0xC0;                                      
  /* SCI1BDH: LBKDIE=0,RXEDGIE=0,SBR12=0,SBR11=0,SBR10=0,SBR9=0,SBR8=0 */
  SCI2BDH = 0x00;                                      
  /* SCI1BDL: SBR7=1,SBR6=0,SBR5=0,SBR4=1,SBR3=1,SBR2=1,SBR1=0,SBR0=0 */
  SCI2BDL = 0x9C;                                      
  /* SCI1C1: LOOPS=0,SCISWAI=0,RSRC=0,M=0,WAKE=0,ILT=0,PE=0,PT=0 */
  SCI2C1 = 0x00;                                      
  /* SCI1C3: R8=0,T8=0,TXDIR=1,TXINV=0,ORIE=0,NEIE=0,FEIE=0,PEIE=0 */
  SCI2C3 = 0x00;                                      
  /* SCI1C2: TIE=0,TCIE=0,RIE=1,ILIE=0,TE=1,RE=1,RWU=0,SBK=0 */
  SCI2C2 = 0x2c;  


  
  /* ### */
} /*MCU_init*/
#endif
/****************************************************/
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
	{
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
	f_bibibi = 0;
Tfmq:
	f_bi = 0;
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
unsigned char SingleAd(unsigned char channel);
unsigned char CheckLcTable(unsigned char r_lcad);
/****************************************************/
void ad_convert(void)
{
	uchar adtemp;

	if ((unsigned char)(t_onems - t_ad) >= 200 || !f_first_ad)
	{
		t_ad = t_onems;
		adtemp = r_hwadtemp;
		r_hwadtemp = ADC_GetValue(22); //SingleAd(AD_CH10); @2018108 CFJ
		if (r_hwadtemp >= adtemp)
		{
			adtemp = r_hwadtemp - adtemp;
		}
		else
		{
			adtemp = adtemp - r_hwadtemp;
		}
		if (adtemp <= 5)
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
//^^^^^^^^^^^^^^^^^^
/*
unsigned char SingleAd(unsigned char channel)
{
	ADCSC1=channel;
	while(!ADCSC1_COCO)	
	{
		__NOP; //nop; @20181008 CFJ
		__NOP;//nop; @20181008 CFJ
	}
	return(ADCRL);	
}
*/
//^^^^^^^^^^^^^^^^^^
unsigned char CheckLcTable(unsigned char r_lcad)
{
	if (r_lcad > 223)
	{
		return (8);
	}
	else if (r_lcad <= 23)
	{
		return (98);
	}
	else
	{
		return (table_lc[(unsigned char)(r_lcad - 24)]);
	}
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
	RuleForLdDisp();

#if (LC_DISP_RESOLUTION_0D1 || LC_RESOLUTION_0D1) //显示分辨率0.1度
	RuleForLcDisp_0D1();						  //显示温度人工干预，不采用实际温度  (分辨率1度)
#else
	RuleForLcDisp(); //显示温度人工干预，不采用实际温度   (分辨率1度)
#endif

	LcDisp(); //把每个数码管显示内容确定下来
	LdDisp();
	AlarmAndLockLed();
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
	if (!f_ld_first_disp)
	{
		f_ld_first_disp = 1;
		r_ldgzwd = r_ldsjwd;
		t_ld_rule = t_halfsec;
	}
	//实际温度>设定温度+1或者实际温度<设定温度
	else if ((r_ldsjwd > (unsigned char)(r_ldzt + 1)) || (r_ldsjwd < r_ldzt)) //30s变化一度
	{

		if ((t_halfsec - t_ld_rule) >= 120) //60s HW 20190810
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
			if ((t_halfsec - t_ld_rule) >= 120) //60s HW 20190810
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

	if ((r_hwsjwd >= 67) && (r_ldzt <= 170)) //当环境温度大于29℃且设定温度小于-30℃
	{
		r_ldxswd = r_ldxswd - 2; //显示值-2
	}
	else if (r_ldzt <= 170)
	{
		r_ldxswd = r_ldxswd - 1; //显示值-1
	}

	/*if(r_ldwdjz>=10)//温度校正
  {
    r_ldxswd = r_ldgzwd+(r_ldwdjz-10);
  }
  else
  {
    r_ldxswd = r_ldgzwd-(10-r_ldwdjz);
  }*/
}

/****************************************************/
/********************冷藏显示规则********************/
/****************************************************/
void RuleForLcDisp(void)
{
	if (!f_first_ad) //还未采样温度
	{
		return;
	}
	if (!f_lc_first_disp) //首次显示
	{
		f_lc_first_disp = 1;
		r_lcgzwd = r_lcwd;	 //显示实际温度
		t_lc_rule = t_halfsec; //记下显示刷新时间点
		u8_LcRule = t_tens;
	}
	else if ((r_lcwd > (uchar)(r_lczt + 2)) || (r_lcwd < (uchar)(r_lczt - 1))) //实际温度高于设置温度超过2度  或者  低于设置温度超过1度
	{
		{
			//u8_LcRule = t_tens; @20181130 CFJ
			if ((t_halfsec - t_lc_rule) >= 120) //60秒 @20181130 CFJ
			{
				t_lc_rule = t_halfsec;
				if (r_lcwd > r_lcgzwd) //显示温度<实际温度
				{
					r_lcgzwd++; //显示温度++
				}
				else if (r_lcwd < r_lcgzwd)
				{
					r_lcgzwd--;
				}
			}
		}
	}
	else
	{
		if (r_lcgzwd != r_lczt)
		{
			// t_lc_rule = t_halfsec;
			if ((t_halfsec - t_lc_rule) >= 120) //60S钟 @20181120
			{
				t_lc_rule = t_halfsec;
				if (r_lcgzwd > r_lczt) //显示温度>实际温度
				{
					r_lcgzwd--; //显示温度--
				}
				else if (r_lcgzwd < r_lczt)
				{
					r_lcgzwd++;
				}
			}
		}
	}
	r_lcxswd = r_lcgzwd;
}

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
	else if ((r_lcwd_float > (r_lczt_float + 2)) || (r_lcwd_float < (r_lczt_float - 1))) //实际温度高于设置温度超过2度  或者  低于设置温度超过1度
	{
		{
			//u8_LcRule = t_tens; @20181130 CFJ
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
	}
	else
	{
		if (r_lcgzwd_float != r_lczt_float) //拟显示温度 不等于 设置温度
		{
			// t_lc_rule = t_halfsec;
			if ((t_halfsec - t_lc_rule) >= 12) //6S
			{
				t_lc_rule = t_halfsec;
				float xxx = fabs(r_lcgzwd_float - r_lczt_float);
				if (xxx < 0.1)
					r_lcgzwd_float = r_lczt_float;
				else
				{
					if (r_lcgzwd_float > r_lczt_float) //显示温度>实际温度
					{
						r_lcgzwd_float -= 0.1; //显示温度--
					}
					else if (r_lcgzwd_float < r_lczt_float)
					{
						r_lcgzwd_float += 0.1;
					}
				}
			}
		}
	}
	r_lcxswd_float = r_lcgzwd_float;
	r_lcgzwd = (unsigned char)floor(r_lcgzwd_float); //有些地方还需要用到r_lcgzwd,比如压缩机连续运行保护的功能需要判断r_lcgzwd是否等于设置温度，所以取r_lcgzwd_float得整数部分给r_lcgzwd
}

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
    if(f_disp_Wifi_Cfg)
	{
		if(f_disp_Wifi_Cfg == 1)
		{
			if (t_halfsec & 0x01) //0b00000001)  @20181008 CFJ
			{
				r_bw = 25;  //.
				r_sw = 25;	//.
				r_gw = 25;	//.
			}
			else
			{
				r_bw = DISP_NO;
				r_sw = DISP_NO;
				r_gw = DISP_NO;
			}
		}
		else if(f_disp_Wifi_Cfg == 2)
		{
			if (t_halfsec & 0x01) //0b00000001)  @20181008 CFJ
			{
				r_bw = DISP_C;
				r_sw = DISP_F;
				r_gw = 9;
			}
			else
			{
				r_bw = DISP_NO;
				r_sw = DISP_NO;
				r_gw = DISP_NO;
			}
		}
	}
	else if (f_test_alarm || f_First_PowerOnFlag) //@20181120 CFJ
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
	else if (r_set_state == SET_LD_WDJZ) //冷冻温度校准
	{
		if (r_ldwdjzx >= 10)
		{
			r_bw = DISP_NO;
			r_sw = DISP_NO;
			r_gw = r_ldwdjzx - 10;
		}
		else
		{
			r_bw = DISP_NO;
			r_sw = DISP_FH;
			r_gw = 10 - r_ldwdjzx;
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
			if ((flag_ajszyjkt & 0x01) == 0x01)
				r_gw = 1;
			else
				r_gw = 0;
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
		{
			if ((t_halfsec - t_ld_err_disp) >= 6) //3s
			{
				t_ld_err_disp = t_halfsec;
				f_ld_high_disp = 0;
			}
			r_bw = DISP_NO;
			r_sw = DISP_E;
			r_gw = 9;
		}
		else if (f_ld_low && f_ld_low_disp)
		{
			if ((t_halfsec - t_ld_err_disp) >= 6) //3s
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
	else if (f_key_defrost && f_key_defrost_disp) //除霜时,显示3sDF,显示5秒温度
	{
		if ((t_halfsec - t_ld_err_disp) >= 6)
		{
			t_ld_err_disp = t_halfsec;
			f_key_defrost_disp = 0;
		}
		r_bw = DISP_NO;
		r_sw = DISP_D;
		r_gw = DISP_F;
	}
	else if (f_power_off && f_power_off_disp) //加
	{
		r_bw = DISP_NO;
		r_sw = DISP_NO;
		r_gw = DISP_NO;
	}
	else if (f_ld_sensor_err)
	{
		if ((t_halfsec - t_ld_err_disp) >= 10) //5s
		{
			f_key_defrost_disp = 1;
			t_ld_err_disp = t_halfsec;
		}
		r_bw = DISP_NO;
		r_sw = DISP_E;
		r_gw = 6;
	}
	else if (g_Sys_Erflag0_Comm) //@20181130 CFJ 增加通讯故障标志
	{
		r_bw = DISP_NO;
		r_sw = DISP_E;
		r_gw = 4;
	}
	else
	{
		if ((t_halfsec - t_ld_err_disp) >= 10) //5s
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
			if ((flag_ajszyjkt & 0x02) == 0x02)
				r_gw = 1;
			else
				r_gw = 0;
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
		{
			if ((t_halfsec - t_lc_err_disp) >= 6) //3s
			{
				t_lc_err_disp = t_halfsec;
				f_BatteryErrDisp = 0;
			}
			r_bw = DISP_NO;
			r_sw = DISP_E;
			r_gw = 5;
		}
		else if (f_lc_high && f_lc_high_disp)
		{
			if ((t_halfsec - t_lc_err_disp) >= 6) //3s
			{
				t_lc_err_disp = t_halfsec;
				f_lc_high_disp = 0;
			}
			r_bw = DISP_NO;
			r_sw = DISP_E;
			r_gw = 9;
		}
		else if (f_lc_low && f_lc_low_disp)
		{
			if ((t_halfsec - t_lc_err_disp) >= 6) //3s
			{
				t_lc_err_disp = t_halfsec;
				f_lc_low_disp = 0;
			}
			r_bw = DISP_E;
			r_sw = 1;
			r_gw = 0;
		}
		else if (f_hw_sensor_err && f_hw_sensor_err_disp)
		{
			if ((t_halfsec - t_lc_err_disp) >= 6) //3s
			{
				t_lc_err_disp = t_halfsec;
				f_hw_sensor_err_disp = 0;
			}
			r_bw = DISP_NO;
			r_sw = DISP_E;
			r_gw = 0;
		}
		else if (f_hw_high38 && f_hw_high_disp)
		{
			if ((t_halfsec - t_lc_err_disp) >= 6) //3s
			{
				t_lc_err_disp = t_halfsec;
				f_hw_high_disp = 0;
			}
			r_bw = DISP_E;
			r_sw = 1;
			r_gw = 4;
		}
		else
		{
			f_lc_DispErrs = 0;
			return;
		}
	}
	else if (f_power_off && f_power_off_disp) //电量很低，关闭显示
	{
		if ((t_halfsec - t_lc_err_disp) >= 60) //30s
		{
			t_lc_err_disp = t_halfsec;
			t_ld_err_disp = t_halfsec; //加 2010 0506
			f_power_off_disp = 0;
		}
		r_bw = DISP_NO;
		r_sw = DISP_NO;
		r_gw = DISP_NO;
	}
	else if (f_lc_sensor_err) //冷藏传感器错误  E3
	{
		if ((t_halfsec - t_lc_err_disp) >= 10) //5s
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
		if ((t_halfsec - t_lc_err_disp) >= 10) //5s
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
				r_bw = DISP_FH; //-
				uchar temp_lc_x10 = (uchar)((38.0 - r_lcxswd_float) * 10 + 0.5);
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
	else if (f_ld_sensor_err || f_hw_sensor_err || f_hw_high38 || f_ld_high || f_ld_low || f_power_off || f_lc_high || f_lc_low || f_lc_sensor_err || f_door_open) ////////加开门报警wys11.03.19f_door_open
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
	BuzzAlarmControl();
	LongRangeAlarm();
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
	else if (f_ld_high_buzz || f_ld_low_buzz || f_power_buzz || f_ld_sensor_err || f_lc_high_buzz || f_lc_low_buzz || f_lc_sensor_err || f_dooropen_buzz) //||f_hw_sensor_err||f_battery)//20181101 TEST 加了一个环温传感器故障和电池电压低报警 CFJ /////wys11.03.19加开门报警
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
	/*
	if(!f_com_ok)
	return;
	f_com_ok = 0;
	//冷冻的温度处理
	r16_ldad = rec[3];
	r16_ldad = r16_ldad<<8;
	r16_ldad = (r16_ldad&0xff00)|rec[2];
	if ((r16_ldad <= 1010)&&(r16_ldad >= 15))
	{//这个括号内程序没有全部对区间处理,但不影响程序正确性
		if((r_ldwdjz>=10)&& (r16_ldad > 35))//温度校正是正数&&温度小于等于50
		{
			r16_ldad = r16_ldad-(r_ldwdjz-10)*4;//校准上偏10.温度对应4个AD
		}
		else if ((r_ldwdjz < 10)&& (r16_ldad < 990))
		{
			r16_ldad = r16_ldad+(10-r_ldwdjz)*4;
		}
	}
  	if(r16_ldad>470)     
  	{
		r_ldsjwd = 140;//-60度
	} 
	else if(r16_ldad<36)
	{
		r_ldsjwd = 250;//50度
	} 
	else													
	{
		r_ldsjwd = (tab_temperature[r16_ldad-36]);
	}
	//冷藏温度的处理,没有温度显示在50~-60度的限制 
	r_lcad = rec[1]; 
	if ((r_lcad < 250) && (r_lcad > 5))
	{
		if((r_lcwdjz>=10) && (r_lcad > 20))                  //温度校正
		{
			r_lcad = r_lcad-(r_lcwdjz-10)*3;
		}
		else if ((r_lcwdjz < 10) && (r_lcad < 235))
		{
			r_lcad = r_lcad+(10-r_lcwdjz)*3;
		} 
	}
	r_lcwd = CheckLcTable(r_lcad);
	//电压
	r_voltage_ad = rec[4];
	if((r_voltage_ad >= 13)&&(r_voltage_ad<= 254))
	{
		if(r_voltage_ad>=30)
		{
			r_voltage = tab_voltage[r_voltage_ad-13];    //VoltageDisplayValue1
		}
		else
		{
			r_voltage = 0;
		}
	}
	//电池
	r_battery_ad = rec[5];
	flag_rec1 = rec[6];//无效
	if (!f_first_ad)
	{ 
		r_voltage_report =r_voltage;//r_voltage_report变量无效
	}  
	f_first_ad = 1;
	*/
}
/****************************************************/
/*****************Judge Errs*************************/
/****************************************************/
void JudgeLdErr(void);
void JudgeLcErr(void);
void JudgeHwErr(void);
void JudgeHwHigh(void);
void JudgeLdHigh(void);
void JudgeLcHigh(void);
void JudgeLdLow(void);
void JudgeLcLow(void);
void JudgeVoltageErr(void);
void JudgePowerOff(void);
void JudgeBattery(void);
/*****************************************************/
void JudgeErrs(void)
{
	if (f_first_ad)
	{
		JudgeLdErr();
		JudgeLcErr(); //传感器故障
		JudgeHwErr();
		JudgeHwHigh();
		JudgeLdHigh();
		JudgeLdLow();
		JudgeLcHigh();
		JudgeLcLow();
		JudgePowerOff();
		JudgeBattery();
		JudgeVoltageErr();
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
void JudgeLdErr(void) //冷冻传感器故障
{
	if (r16_ldad > 1010 || r16_ldad < 15)
	{
		if (!f_ld_sensor_err) //如果出现冷冻传感器故障,进入一次,以后不再进入
		{
			f_ld_sensor_err = 1;
			f_stop_alarm = 0;
		}
	}
	else if (f_ld_sensor_err)
	{
		f_ld_sensor_err = 0;
		f_ld_first_disp = 0;
	}
}
/**********************************/
void JudgeLcErr(void) //冷藏传感器故障
{
	if (r_lcad > 250 || r_lcad < 5)
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
/**********************************/
void JudgeHwErr(void) //环温传感器故障
{
	if (r_hwad > 240 || r_hwad < 5)
	{
		if (!f_hw_sensor_err)
		{
			f_hw_sensor_err = 1;
		}
	}
	else
	{
		f_hw_sensor_err = 0;
	}
}
/*********************************/
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
}
/**************************************************/
void JudgeLdHigh(void)
{
	//冷冻传感器错误
	if (f_ld_sensor_err || (!ldyj))
	{
		f_ld_high = 0;
		f_ld_high_buzz = 0;
		t_ld_high_buzz = t_tens;
	}
	//高温报警,初次上电经延迟时间后FMQ报警,此后15分钟FMQ报警
	else if (r_ldxswd > (r_ldzt + r_ld_high_alarm)) //显示和设定温度都上偏200
	{
		f_ld_high = 1;
		if (f_Powerupdelayld) //初次冷冻上电延迟时间+15min后FMQ报警
		{
			if ((unsigned char)(t_tens - t_ld_high_buzz) >= 90) //15min
			{
				if (!f_ld_high_buzz)
				{
					f_ld_high_buzz = 1;
					f_stop_alarm = 0;
				}
			}
		}
		else
		{
			t_ld_high_buzz = t_tens;
		}
	}
	//高温报警正常
	else if (r_ldxswd < (r_ldzt + r_ld_high_alarm))
	{
		f_ld_high = 0;
		f_ld_high_buzz = 0;
		t_ld_high_buzz = t_tens;
	}
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
void JudgeLdLow(void)
{
	if (f_ld_sensor_err || (!ldyj))
	{
		f_ld_low = 0;
		f_ld_low_buzz = 0;
		t_ld_low_buzz = t_tens;
	}
	else if (r_ldxswd < (r_ldzt - r_ld_low_alarm))
	{
		f_ld_low = 1;
		if (f_Powerupdelayld)
		{
			if ((unsigned char)(t_tens - t_ld_low_buzz) >= 90) //15min
			{
				if (!f_ld_low_buzz)
				{
					f_ld_low_buzz = 1;
					f_stop_alarm = 0;
				}
			}
		}
		else
		{
			t_ld_low_buzz = t_tens;
		}
	}
	else if (r_ldxswd > (r_ldzt - r_ld_low_alarm))
	{
		f_ld_low = 0;
		f_ld_low_buzz = 0;
		t_ld_low_buzz = t_tens;
	}
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
			if ((t_halfsec - t_voltage_buzz) >= 120) //1min
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
			if ((t_halfsec - t_voltage_buzz) >= 120) //1min/////////////规格书是30s
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
		if ((t_halfsec - t_power_Off) >= 6) //低电压持续3s，则报断电 @20181226 CFJ
		{
			f_power_off = 1;
		}

		if ((t_halfsec - t_power_buzz) >= 60) //30s
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
    int i = 0;
	for (; R8_DelayCnt > 0; R8_DelayCnt--)
	{
        for(i = 0; i< 16; i++)  //由4MHz倍频到了64MHz,快了16倍，所以nop变为原来16倍
        {
            nop;
            nop;
            nop;
            nop;
            nop;
            nop;
            nop;
            nop;
            nop;
            nop;
            nop;
            nop;
            nop;
            nop;
            nop;
            nop;
            nop;
            nop;
            nop;
            nop;
        }
	}
}
/*****************Data to stled361s******************/
/****************************************************/
void LedWriteByte(unsigned char led_data);
void LedSendScl(void);
void LedBegin(void);
void LedStop(void);
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

		LedWriteByte(r_led21); //冷藏数码管  第1位
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
			LED_DIO = 1;
		else
			LED_DIO = 0;
		led_data = led_data >> 1;
		LedSendScl(); //ack signal
	}
}
/****************************************************/
void LedBegin(void)
{
	DelayControl(4);
	LED_STB = 0;
	DelayControl(4);
}
/****************************************************/
void LedStop(void)
{
	DelayControl(4);
	LED_STB = 1;
	DelayControl(4);
}

/****************************************************/
void LedSendScl(void)
{
	DelayControl(4);
	LED_CLK = 0;
	DelayControl(4);
	LED_CLK = 1;
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
	if (f_first_ad)
	{
		ReadKey();   //读按键值
		KeyAction(); //分析按键
		AutoLock();
	}
}
//^^^^^^^^^^^^^^^^^^^^^^^^^
void ReadKey(void)
{
	r_key = 0;
	if (KEY_SW1)
	{
		r_key = r_key | 0x01; //0b00000001; @20181008 CFJ
	}
	if (KEY_SW2)
	{
		r_key = r_key | 0x02; //0b00000010; @20181008 CFJ
	}
	if (KEY_SW3)
	{
		r_key = r_key | 0x04; //0b00000100; @20181008 CFJ
	}
	if (KEY_SW4)
	{
		r_key = r_key | 0x08; //0b00001000; @20181008 CFJ
	}
	if (KEY_SW5)
	{
		r_key = r_key | 0x10; //0b00010000; @20181008 CFJ
	}
	if (KEY_SW6)
	{
		r_key = r_key | 0x20; //0b00100000; @20181008 CFJ
	}
	if (KEY_SW7)
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
		r_sfkeyz = 0;
		f_key_kp = 0;
		f_key_km = 0;
	}
	else //有按键按下
	{
		if (!f_key_kp)
		{
			if (r_key != r_keyz)
			{
				f_key_km = 0;
				r_keyz = r_key;
				t_key_dly = t_onems;
				t_key3s = t_halfsec;
			}
			else
			{
				if (!f_key_km)
				{
					if ((unsigned char)(t_onems - t_key_dly) >= 80)
					{
						f_key_km = 1;
						KeyPutDown(); //按下有效的按键处理
					}
				}
				else
				{
					if ((t_halfsec - t_key3s) >= 10)
					{
						KeyPut5s(); //按下5秒有效的按键处理
					}
					/*if((t_halfsec-t_key3s)>=6)
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
			{
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
	{
		if (!f_lock)
		{
			if (r_set_state == SET_NC_LD) //冷冻当前调整
			{
				r_set_state = SET_LD_L0; //LD进入显示L0画面
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
				f_zf = FU_SIGN;
				r_swtj = (uchar)((200 - r_ldzt) / 10);
				r_gwtj = (uchar)((200 - r_ldzt) % 10);
				goto NotSaveKey2;
			}
			else if (r_set_state == SET_LD_L1) //L1-->高温报警设定值
			{
				r_set_state = SET_LD_HIGH;
				r_flash_bit = BIT2;
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
				goto NotSaveKey2;
			}
			else if (r_set_state == SET_LD_L2) //L2-->低温报警设定值
			{
				r_set_state = SET_LD_LOW;
				r_flash_bit = BIT2;
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
			else if (r_set_state == LC_CHK_VOL || r_set_state == LC_CHK_HW)
			{
				r_set_state = SET_NC_LC;
				goto NotSaveKey2;
			}
			else if (r_set_state == SET_LD) //冷冻设定温度
			{
				if ((r_swtj * 10 + r_gwtj) >= 10 && (r_swtj * 10 + r_gwtj) <= 45) //冷冻温度设定范围-10~-45之间
				{
					r_ldzt = 200 - (r_swtj * 10 + r_gwtj);
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
			}
			else if (r_set_state == SET_LD_LOW) //LD低温设定
			{
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
				r_set_state = SET_NC_LD;
				r_ldwdjz = r_ldwdjzx;
				goto SaveKey2;
			}
			else if (r_set_state == SET_LC_WDJZ)
			{
				r_set_state = SET_NC_LC;
				r_lcwdjz = r_lcwdjzx;
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
				u8_year_set = u8_year;
                u8_month_set = u8_month;
                u8_day_set=u8_day;
                u8_hour_set=u8_hour;
                u8_minute_set=u8_minute; 
				f_Set_Time = 1;
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
			goto NotSaveKey2;
		}
	}
	else if (r_sfkeyz == KEY_DISABLE_BUZZ) //消音
	{
        if(f_disp_Wifi_Cfg)		//短摁蜂鸣取消建，退出wifi配置模式
			f_Exit_WifiCfg = 1;
		f_stop_alarm = ON;
		t_stop_alarm = 0;
		goto NotSaveKey2;
	}
	else if (r_sfkeyz == KEY_SWITCH) //切换键
	{
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
	{
		r_sfkeyz = KEY_UNLOCK;
	}
	else if (r_keyz == KEY_SET)
	{
		if (r_set_state1 == SET_F1) //SET_F1=2
		{
			r_set_state1 = SET_NC; //SET_NC=0
			f_lock = OFF;
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
	{
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
			r_set_state = SET_LD_L3;
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
			r_set_state = SET_LC_L0;
			r_flash_bit = BIT3;
			goto NotSaveKey;
		}
		else if (r_set_state == SET_LD || r_set_state == SET_LC)
		{
			if (r_flash_bit == 2) //冷冻冷藏十位调节
			{
				r_swtj++;
                if(r_set_state == SET_LC)
                {
                    if (r_swtj > 1)
                    {
                        r_swtj = 0;
                    }
                }
				else if(r_set_state == SET_LD)  //冷冻十位最大4
                {
                    if (r_swtj > 4)
                    {
                        r_swtj = 0;
                    }
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
			goto NotSaveKey;
		}
		else if (r_set_state == SET_LD_LOW || r_set_state == SET_LC_LOW)
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
			goto NotSaveKey;
		}
		else if (r_set_state == SET_LDYJ)
		{
			if ((flag_ajszyjkt & 0x01) == 0x01)
				flag_ajszyjkt = flag_ajszyjkt & 0xfe;
			else
				flag_ajszyjkt = flag_ajszyjkt | 0x01;
			goto NotSaveKey;
		}
		else if (r_set_state == SET_LCYJ)
		{
			if ((flag_ajszyjkt & 0x02) == 0x02)
				flag_ajszyjkt = flag_ajszyjkt & 0xfd;
			else
				flag_ajszyjkt = flag_ajszyjkt | 0x02;
			goto NotSaveKey;
		}
		else if (r_set_state == SET_LD_WDJZ)
		{
			r_ldwdjzx++;
			if (r_ldwdjzx > 15)
			{
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
			if (u8_year > 50)
			{
				u8_year = 20;
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
	else if (r_keyz == KEY_DEFROST) //化霜
	{
		if (!f_lock)
		{
			if (f_key_defrost)
			{
				f_key_defrost = OFF;
			}
			else
			{
				f_key_defrost = ON;
				f_key_defrost_disp = 1;
				t_ld_err_disp = t_halfsec;
			}
			f_compressor_on_dly = 1;
			f_on_off_dly = 0;
			goto NotSaveKey2;
		}
	}
	else if (r_keyz == KEY_BJYCSJ) //FMQ报警延迟时间
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
	else if (r_keyz == KEY_DATE)
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
    else if (r_keyz == KEY_DISABLE_BUZZ) //长摁蜂鸣取消键，进入wifi配置模式
	{
		//if (!f_lock)
		{
			f_Enter_WifiCfg = 1;
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
	if ((t_halfsec - t_auto_lock) >= 20)
	{
		f_lock = 1;
        if (r_set_state1 == SET_F1 || r_set_state1 == SET_F0 )	  // 解锁过程中超时自动退出，正常显示温度   2021-1-15  wpf
		{
			r_set_state1 = SET_NC;
		}
		if (r_set_state == SET_LD)
		{
			if ((r_swtj * 10 + r_gwtj) >= 10 && (r_swtj * 10 + r_gwtj) <= 45)
			{
				r_ldzt = 200 - (r_swtj * 10 + r_gwtj);
				f_need_write_e2 = ON;
				t_write_e2 = t_halfsec;
			}
			else
			{
				BuzzBiBiBi();
			}
		}
		else if (r_set_state == SET_LD_HIGH)
		{
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
		}
		else if (r_set_state == SET_LD_LOW)
		{
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
		}
		else if (r_set_state == SET_LC)
		{
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
		{
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
		{
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
		{
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
		else if (r_set_state >= 15)
			r_set_state = SET_NC_LC;
		else
			r_set_state = SET_NC_LD;
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
	if (f_ld_sensor_err)
	{
		if (f_onemin)
		{
			t_compressor++;
		}
		if (t_compressor < 30)
		{
			goto CompressorOn;
		}
		else if (t_compressor < 40)
		{
			goto CompressorOff;
		}
		else
		{
			t_compressor = 0;
			goto CompressorOn;
		}
	}
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
		if (f_tens)
		{
			t_yj_dly_time++;
			if (t_yj_dly_time >= (t_yj_delay * 6))
			{
				f_compressor_on_dly = 1;
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
								   /****************冷藏压机控制程序****************/
void Lc_CompressorJudge(void)
{
	lcCompressorRun10Hour();
#if LC_COMP_RUN_MAX_1_HOUR
	lcCompRunOneHour(); //按照系统人员要求屏蔽一小时的，HW2019/7/29 9:50
#elif LC_COMP_RUN_MAX_6_HOUR
	lcCompressorAddup6Hour();
#endif
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
	if (f8_lcCompAddUp6HourProtect != 0) // 压机累计6小时保护
	{									 // 140819
		goto Lc_CompressorOff;
	}
	if (f_lcCompressorProtect) //压机10小时保护
	{
		goto Lc_CompressorOff;
	}
	if (f_lcComp1HourProtect) //压机1小时保护
	{
		goto Lc_CompressorOff;
	}
	if (f_lc_sensor_err)
	{
		if (f_onemin)
		{
			t_lc_compressor++;
		}
		if (t_lc_compressor < 15)
		{
			goto Lc_CompressorOn;
		}
		else if (t_lc_compressor < 30)
		{
			goto Lc_CompressorOff;
		}
		else
		{
			t_lc_compressor = 0;
			goto Lc_CompressorOn;
		}
	}
	t_lc_compressor = 0;
	if (f_lc_compressor == ON)
	{
		if (r_lcad >= table_lc_off[r_lczt - 38])
			goto Lc_CompressorOff;
	}
	else
	{
		if (r_lcad <= table_lc_on[r_lczt - 38])
			goto Lc_CompressorOn;
	}
	return;
Lc_CompressorOn:
	if (f_compressor_on_dly && !f_lc_on_off_dly)
	{
		if (!f_lc_compressor && delay_10sec)
		{
			f_lc_compressor = ON;
			f_lc_fan = ON;
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
			{
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

		if (r_lcad_12b >= LCCOMPRTSTOREAD) //冷藏压缩机开机点  HW
		{
			f_lcCompressorProtect = 0;
		}
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
/********************判断风机程序*****************/
/*************************************************/
void LnFan(void)
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
    static unsigned char t_open_ms = 0;
	if (!DOOR_ONFLAG | !LED_ONFLAG) //if(!PTBD_PTBD7|!PTDD_PTDD0)
	{
		if ((uchar)(t_onems - t_lightms) >= 60)
			LED_OUT = 1; //PTDD_PTDD2=1;   @20180920 cfj
		f_LightStatusPin = 1;
	}
	else
	{
		t_lightms = t_onems;
		LED_OUT = 0; //PTDD_PTDD2=0;      @20180920 cfj
		f_LightStatusPin = 0;
	}

    //检测门状态
    if(!DOOR_ONFLAG)
    {
        if ((uchar)(t_onems - t_open_ms) >= 60)
        {
            f_door_state = 1;
        }
    }
    else
    {
        t_open_ms = t_onems;
        f_door_state = 0;
    }
}
/********************内胆风机程序****************/
void Nd_fan_Prog(void)
{
	if (DOOR_ONFLAG) //if(PTBD_PTBD7)    @20180920 cfj
	{
		f_door_switch = 0; // 100526  zyj
		if (lcyj)
		{
			if ((uchar)(t_onems - t_nd_fan) >= 60)
			{
				f_nd_fan = 1;
			}
		}
		else
		{
			t_nd_fan = t_onems;
			f_nd_fan = 0;
		}
	}
	else
	{
		f_door_switch = 1;
		t_nd_fan = t_onems;
		f_nd_fan = 0;
	}
}
/***********开门报警程序***************/ /////wys11.03.19
void Door_Open(void)
{
	if (f_door_switch) //开门
	{
		f_door_open = 1;
		if ((unsigned char)(t_tens - t_door) >= 90) //15min
		{
			if (!f_dooropen_buzz)
			{
				f_dooropen_buzz = 1;
				f_stop_alarm = 0;
			}
		}
	}
	else
	{
		f_door_open = 0;
		f_dooropen_buzz = 0;
		t_door = t_tens;
	}
}
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

/****************************************************/
/***************更新设定值程序***********************/
/****************************************************/
void WriteToE2(void)
{
	if (f_need_write_e2)
	{
		if ((t_halfsec - t_write_e2) >= 4) //2s
		{
			f_need_write_e2 = 0;
			f_e2prom = 1;
		}
	}
}

/****************************************************/
/**********II2 BUS***********************************/
/****************************************************/
/*need define in your .h file:f_e2prom,C_sda,C_scl,
  C_sdaddr,in,out*/
void Begin(void);
void Stop(void);
void Sendscl(void);
void WriteByte(unsigned char tem_dat);
unsigned char ReadByte(void);
/*****************************************************/

//^^^^^^^^^^^^^^^^^^^^^^^^^^


void WriteE2(void)
{
	if (f_e2prom)
	{
		Begin();
		WriteByte(0xA0); ///write chip addres 0xa0      每页64个字节，可以连续写
		WriteByte(0x00); ///write data high addres
		WriteByte(0x00); ///write data low  addres

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
		WriteByte(0xAA);				 //byte13

		C_sda = 1;
		Stop();
		f_e2prom = 0;
	}
}
//^^^^^^^^^^^^^^^^^^^^^^^^^^
void ReadE2(void)
{
	unsigned char temp;
	Begin();
	WriteByte(0xA0); ///write chip addres 0xa0
	WriteByte(0x00); ///write data addres
	WriteByte(0x00);
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

	temp = ReadByte(); //byte13
	C_sda = 1;
	Sendscl();
	Stop();

	r_ldzt_report = r_ldzt;
	r_lczt_report = r_lczt;
	ld_high_alarm_report = r_ld_high_alarm;
	ld_low_alarm_report = r_ld_low_alarm;
	lc_high_alarm_report = r_lc_high_alarm;
	lc_low_alarm_report = r_lc_low_alarm;
	if (temp != 0xAA)
	{
		r_ldzt = 160; //-40
		r_lczt = 43;  //5°C
		r_lczt_float = r_lczt;
		t_yj_delayx = 0;
		t_yj_delay = 0;
		r_ldwdjzx = 10;
		r_ldwdjz = 10;
		r_ld_high_alarm = 5;
		r_ld_low_alarm = 5;
		r_lc_high_alarm = 3;
		r_lc_low_alarm = 3;
		r_lcwdjzx = 10;
		r_lcwdjz = 10;
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
	}
}
//^^^^^^^^^^^^^^^^^
void WriteByte(unsigned char tem_dat)
{
	unsigned char w_clk;

	C_sdaddr = out; //1

	for (w_clk = 8; w_clk != 0; w_clk--)
	{
		DelayControl(4);
		if (tem_dat & 0x80)
			C_sda = 1;
		else
			C_sda = 0;
		tem_dat = tem_dat << 1;
		DelayControl(4);
		Sendscl(); //ack signal
	}
	C_sda = 1;
	C_sdaddr = in;
	w_clk = 0xFF;
	Sendscl();		//ack signal
	while (!C_sda1) //while(!C_sda)
	{
		w_clk--;
		DelayControl(4);
		if (w_clk == 0)
			break;
	}
	C_sdaddr = out;
}
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
unsigned char ReadByte(void)
{
	unsigned char r_dat, r_clk;
	C_sda = 1;
	C_sdaddr = in;

	for (r_clk = 8; r_clk > 0; r_clk--)
	{
		C_scl = 1;
		DelayControl(4);
		r_dat = r_dat << 1;

		if (C_sda1) //if(C_sda)
			r_dat |= 0x01;
		else
			r_dat &= 0xfe;

		C_scl = 0;
		DelayControl(4);
	}
	C_sdaddr = out;
	DelayControl(4);
	C_sda = 0;

	return (r_dat);
}
/*****************************/
void Begin(void)
{
	C_sdaddr = out;
	C_sda = 1;
	DelayControl(4);
	C_scl = 1;
	DelayControl(4);
	C_sda = 0;
	DelayControl(4);
	C_scl = 0;
	DelayControl(4);
}
//--------------------------
void Stop(void)
{
	C_sda = 0;
	DelayControl(4);
	C_scl = 1;
	DelayControl(4);
	C_sda = 1;
	DelayControl(4);
}
//---------------
void Sendscl(void)
{
	C_scl = 1;
	DelayControl(4);
	C_scl = 0;
	DelayControl(4);
}

uchar u8_Rec2data[20];
uchar u8_Send2data[20];
uchar u8_Send2dataMemory[20];

uchar u8_Rec2_data_Num;
uchar u8_Send2_data_Num;
uchar u8_Send2_data_Num_Count;
uchar u8_Rec2_data_State;
uchar u8_Send2_data_State;

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

void ReturnPrintFrame(void)
{

	uchar data;
	// if (u8_Send2_data_State == 1)
	// {
	// 	return;
	// }

	u8_Send2_data_Num = 0x10;
	//45;
	//u8_Send2data[0] = 0x55;
	//u8_Send2data[1] = 0xFF;
	//u8_Send2data[2] = 0x10;
	u8_Send2data[0] = 0x0;
	u8_Send2data[1] = 0x0;
	u8_Send2data[2] = 0x0;
	u8_Send2data[3] = 0x0;
	u8_Send2data[4] = 0x0;
	u8_Send2data[5] = 0x01;
	u8_Send2data[6] = 0x02;
	u8_Send2data[7] = 0x4D;
	u8_Send2data[8] = 0x05;

	/*
  if(r_lcxswd>=38)
   {
	 r_bw = DISP_NO;
	 r_sw = (uchar)((r_lcxswd-38)/10);
	 r_gw = (uchar)((r_lcxswd-38)%10);
   } 
   else
   {
	 r_bw = DISP_FH;//-
	 r_sw = (uchar)((38-r_lcxswd)/10);
	 r_gw = (uchar)((38-r_lcxswd)%10);
   }
*/
#if (LC_DISP_RESOLUTION_0D1 || LC_RESOLUTION_0D1) //显示分辨率0.1度
	r_lcxswd = (unsigned char)floor(r_lcxswd_float);
#else
	;
#endif
	if (r_lcxswd >= 38)
	{
		data = (r_lcxswd - 38);

		u8_Send2data[9] = (data >> 4) & 0x0F;

		u8_Send2data[10] = (data << 4) & 0xF0;
	}
	else
	{
		data = (38 - r_lcxswd);

		u8_Send2data[9] = (data >> 4) & 0x0F;
		u8_Send2data[9] = u8_Send2data[9] | 0x80;

		u8_Send2data[10] = (data << 4) & 0xF0;
	}

	//u8_Send2data[9] = 0x02;//leng c
	//u8_Send2data[10] = 0x03;

	/*
  if(r_ldxswd>=200)
  {
	r_bw = DISP_NO;
	r_sw = (uchar)((r_ldxswd-200)/10);
	r_gw = (uchar)((r_ldxswd-200)%10);
  } 
  else
  {
	r_bw = DISP_FH;//-
	r_sw = (uchar)((200-r_ldxswd)/10);
	r_gw = (uchar)((200-r_ldxswd)%10);
  }
*/

	if (r_ldxswd >= 200)
	{

		data = (r_ldxswd - 200);
		u8_Send2data[11] = (data >> 4) & 0x0F;

		u8_Send2data[12] = (data << 4) & 0xF0;
	}
	else
	{
		data = (200 - r_ldxswd);

		u8_Send2data[11] = (data >> 4) & 0x0F;
		u8_Send2data[11] = u8_Send2data[11] | 0x80;

		u8_Send2data[12] = (data << 4) & 0xF0;
	}

	//u8_Send2data[11] = 0x01;//leng d
	// u8_Send2data[12] = 0x02;

	u8_Send2data[13] = 0;

	if (f_ld_sensor_err == 1)
	{
		u8_Send2data[13] |= 0x01;
	}
	if (f_lc_sensor_err == 1)
	{
		u8_Send2data[13] |= 0x02;
	}

	//u8_Send2data[13] = 0x04;
	u8_Send2data[14] = 0x00;

	GetSum2();
}

void Rec2Action(void)
{
	if (u8_Send_Print_Time >= 2)
	{
		ReturnPrintFrame();
		u8_Send_Print_Time = 0;
	}
}

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
			else if (rec_net[8] == 0x07) //5d 07 冷藏档位校准
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
				if (rec_net[10] <= 15 && rec_net[10] >= 5)
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
				if (((rec_net[9] <= 100 && rec_net[9] >= 0) && (rec_net[10] <= 12 && rec_net[10] >= 1) &&
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
					//第一次收到时间汇报帧，并且数据有误
					if ((!f_net_first_set_time) && ((rec_net[9] > 100 || rec_net[9] < 0) || (rec_net[10] > 12 || rec_net[10] < 1) ||
													(rec_net[11] > 31 || rec_net[11] < 1) || (rec_net[12] > 23) || (rec_net[13] > 59)))
					{
						u8_year = 20;
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
		
		
		ReturnVoidFrame();

	}
	else
	{
		
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
	// if (!f_net_first_set_time)   //还没接收到USB模块的时间
	// {
	// 	f_Set_Time = 0;
	// 	return;
	// }

	if (f_Set_Time)
	{
		if (f_send_1ff || f_send_2ff || f_send_data)
		{
			return;
		}

		{
			for (i = 0; i <= 49; i++)
			{
				send_net[i] = 0x00;
			}

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
			f_Set_Time = 0;
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
			if ((t_halfsec - t_ack5s) >= 10)
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
	Mfs_Uart_EnableIrq(&UART2, UartTxIrq); //使能发送中断//////SCI1C2_TCIE = 1;                                 //使能发送模块
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
/*********************************************
函数名称：void EheatConTrolP(void)
输入参数：r_lcad  冷藏温度传感器的AD值
输出参数：
函数功能：
编写日期：2019/4/24 10:16:32
编者：   HW
***************************************************/

void EheatConTrolP(void)
{
	if (f_lc_sensor_err)
	{
		Eheatcontrolpin = 0;
	}
	else if (r_lcad_12b >= HEATONAD)
	{
		Eheatcontrolpin = 1;
	}
	else if (r_lcad_12b <= HEATOFFAD)
	{
		Eheatcontrolpin = 0;
	}
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
//-----------------------------------------------------------------//
void l_ODM_Data_Parse(void) //接受主板解析数据 CFJ
{
	unsigned char Lc_Temp, Lc_Temp_2, VoltageTemp;
	unsigned char batteryTemp;
	if (g_Sys_ReceiveDataType) //接受数据帧类型  CFJ
	{
		//电压
		VoltageTemp = Pannel_Uart1Data[22] << 4;
		VoltageTemp |= Pannel_Uart1Data[23] >> 4;
		r_voltage_ad = VoltageTemp;
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
		//冷冻的温度处理
		r16_ldad = Pannel_Uart1Data[13]; //高位//rec[3];
		r16_ldad = r16_ldad << 8;		 //r16_ldad<<8;
		r16_ldad = (r16_ldad & 0xff00) | Pannel_Uart1Data[14];
		r16_ldad >>= 2; //12位AD转换为10位AD
		if ((r16_ldad <= 1010) && (r16_ldad >= 15))
		{
			//这个括号内程序没有全部对区间处理,但不影响程序正确性
			if (SelfCheckFlag == 0)
			{
				if ((r_ldwdjz >= 10) && (r16_ldad > 35)) //温度校正是正数&&温度小于等于50
				{
					r16_ldad = r16_ldad - (r_ldwdjz - 10) * 4; //校准上偏10.温度对应4个AD
				}
				else if ((r_ldwdjz < 10) && (r16_ldad < 990))
				{
					r16_ldad = r16_ldad + (10 - r_ldwdjz) * 4;
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

		//冷藏温度的处理,没有温度显示在50~-60度的限制
		Lc_Temp = Pannel_Uart1Data[5] << 4;
		Lc_Temp |= Pannel_Uart1Data[6] >> 4;

		Lc_Temp_2 = Pannel_Uart1Data[7] << 4;
		Lc_Temp_2 |= Pannel_Uart1Data[8] >> 4; // 20190410  HYC-386项目修改为NTC2作为冷藏传感器HW

		if (Lc_Temp >= 250 || Lc_Temp <= 10) //温度大于60 小于-20认为错误,兼容NTC1与NTC2
		{
			r_lcad = Lc_Temp_2;
		}
		else
		{
			r_lcad = Lc_Temp;
		}

		r_lcad_12b = Pannel_Uart1Data[7] * 256 + Pannel_Uart1Data[8];
		if ((r_lcad < 250) && (r_lcad > 5))
		{
			if (SelfCheckFlag == 0)
			{
				if ((r_lcwdjz >= 10) && (r_lcad > 20)) //温度校正
				{
					r_lcad = r_lcad - (r_lcwdjz - 10) * 3;
				}
				else if ((r_lcwdjz < 10) && (r_lcad < 235))
				{
					r_lcad = r_lcad + (10 - r_lcwdjz) * 3;
				}
			}
		}
		//r_lcad = 210; // TEST 测试 @20181025 CFJ
		r_lcwd = CheckLcTable(r_lcad);
		r_lcwd_float = (float)CheckLcTable(r_lcad);
	}
	f_first_ad = 1;
}

void l_Pannel_DataTx(void)
{
	if (bUartSendStartFlag) //&& (g_Txd_Time <= 0))
	{
		bUartSendStartFlag = 0;
		l_Send_Data_Build();
	}
}

void l_Send_Data_Build(void)
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

	g_IDM_TX_ODM_Data[9] = 0x00; //预留未用

	if (f_lc_fan) //冷藏风机
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

	if (f_nd_fan) //内风机 门开关关上的风机
	{
		g_IDM_TX_ODM_Data[10] |= 0x08;
	}

	if (f_lc_compressor) //冷藏压机
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
	if (f_ld_sensor_err)
	{
		g_IDM_TX_ODM_Data[17] |= 0x01; //冷冻传感器故障
	}
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
	g_IDM_TX_ODM_Data[22] = r_lcad;
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
	RS_485_STATUS = 1; //发送有效
	g_Rec_Status_Flag = 0;
	Mfs_Uart_EnableIrq(&UART1, UartTxIdleIrq); //UartTxIrq); //使能发送中断
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
temp:		        Clear_Watchdog();
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
			f_compressor = 0; //冷藏温度显示
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
		Clear_Watchdog();
		if ((uchar)(t_20ms - t_Self_Times) >= 50)
		{
			bSelfUartSendStartFlag = 1;
			t_Self_Times = t_20ms;
			if (SelfCheckStep <= 20)
			{
				SelfCheckStep++;
			}
			l_Pannel_Self_DataTx();
			Clear_Watchdog();
			Pannel_Comm_Deal();
			Clear_Watchdog();
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
	if (f_ld_sensor_err)
	{
		g_IDM_TX_ODM_Data[17] |= 0x00; //0x01; //冷冻传感器故障
	}
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
	RS_485_STATUS = 1; //发送有效
	g_Rec_Status_Flag = 0;
	Mfs_Uart_EnableIrq(&UART1, UartTxIdleIrq); //UartTxIrq); //使能发送中断
}

void l_Pannel_Self_DataTx(void)
{
	if (bSelfUartSendStartFlag) //@20190215 CFJ
	{
		bSelfUartSendStartFlag = 0;
		l_Self_Send_Data_Build();
	}
}