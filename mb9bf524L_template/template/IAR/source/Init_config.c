
#include "pdl_header.h"
#include "includes.h"
#include "Disp.H"
#include "Program_Cfg.h" //程序功能配置，比如禁止蜂鸣器、控制冷藏温度分辨率
#include "Coupler.h"

extern unsigned char g_1s_flag_Iot;
/*时钟配置*/
void Main_ClkInit(void)
{

    stc_clk_config_t stcClockConfig;

    do
    {
        PDL_ZERO_STRUCT(stcClockConfig);
        stcClockConfig.enBaseClkDiv = BaseClkDiv1;
        stcClockConfig.enAPB0Div = Apb0Div1;
        stcClockConfig.enAPB1Div = Apb1Div1;
        stcClockConfig.enAPB2Div = Apb2Div1;
        stcClockConfig.enMCOWaitTime = McoWaitExp117;
        stcClockConfig.enSCOWaitTime = ScoWaitExp10;
        stcClockConfig.enPLLOWaitTime = PlloWaitExp112;
        // PLLCLK = Main Osc * (PLLN / PLLK), please refer to Clock chapter in peripehral manual for K, N, M value
        stcClockConfig.u8PllK = 1; //
        stcClockConfig.u8PllN = 1;
        stcClockConfig.u8PllM = 1;

        /* Initialize clock */

        if (Ok != Clk_Init(&stcClockConfig))
        {
            while (1)
                ;
        }

        Clk_EnableMainClock(TRUE); //使能主时钟
        Clk_SetSource(ClkMain);    //切换到主时钟

    } while (0);
    SystemCoreClockUpdate();
}
/*GPIO 初始化*/
void GPIO_Init(void)
{

    // LCD control
    //P00输出0--CS

    bFM_GPIO_PDOR0_P0 = 0; //初始输出值
    bFM_GPIO_DDR0_P0 = 1u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR0_P0 = 0u; //GPIO

    //P01-- SWCLK

    //P02输出0--CS
    bFM_GPIO_PDOR0_P2 = 0; //初始输出值
    bFM_GPIO_DDR0_P2 = 1u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR0_P2 = 0u; //GPIO

    //P03--SWDIO

    //P04输出0--CS
    bFM_GPIO_PDOR0_P4 = 0; //初始输出值
    bFM_GPIO_DDR0_P4 = 1u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR0_P4 = 0u; //GPIO

    // P0A-----输入 BM4
    bFM_GPIO_PCR0_PA = 1;  //0，上拉禁止；    1，上拉使能
    bFM_GPIO_DDR0_PA = 0u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR0_PA = 0u; //GPIO
    bFM_GPIO_ADE_AN15 = 0u;
    /*
   //P04输出0--RD   
   bFM_GPIO_PDOR0_P4=0;                                                         //初始输出值                 
   bFM_GPIO_DDR0_P4=1u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_PFR0_P4=0u;                                                         //GPIO
   
   //P0B 输出 1--DATA
   bFM_GPIO_PDOR0_PB=1;                                                         //初始输出值
   bFM_GPIO_DDR0_PB=1u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_PFR0_PB=0u;                                                         //GPIO 
   bFM_GPIO_ADE_AN16=0u;
  */
    //P0B 输入  ---BM3
    //bFM_GPIO_PDOR0_PB=0;                                                         //初始输出值
    bFM_GPIO_PCR0_PB = 1;  //0，上拉禁止；    1，上拉使能
    bFM_GPIO_DDR0_PB = 0u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR0_PB = 0u; //GPIO
    bFM_GPIO_ADE_AN16 = 0u;

    //P0C 输入 ---BM2
    bFM_GPIO_PCR0_PC = 1;  //0，上拉禁止；    1，上拉使能
    bFM_GPIO_DDR0_PC = 0u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR0_PC = 0u; //GPIO
    bFM_GPIO_ADE_AN17 = 0u;

    //P0F输入 ----BM1
    bFM_GPIO_PCR0_PF = 1;  //0，上拉禁止；    1，上拉使能
    bFM_GPIO_DDR0_PF = 0u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR0_PF = 0u; //GPIO
    bFM_GPIO_ADE_AN18 = 0u;

    //---显示驱动定义----------//
    //// 输出   P60---DIO
    bFM_GPIO_PDOR6_P0 = 0; //初始输出值
    bFM_GPIO_DDR6_P0 = 1u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR6_P0 = 0u; //????????                                           //非GPIO,外部中断
    bFM_GPIO_ADE_AN21 = 0u;

    //   P61输出0---CLK
    bFM_GPIO_PDOR6_P1 = 0; //初始输出值
    bFM_GPIO_DDR6_P1 = 1u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR6_P1 = 0u; //GPIO
    bFM_GPIO_ADE_AN20 = 0u;

    //P62输出0 --STB
    bFM_GPIO_PDOR6_P2 = 0; //初始输出值
    bFM_GPIO_DDR6_P2 = 1u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR6_P2 = 0u; //GPIO
    bFM_GPIO_ADE_AN19 = 0u;

    //---USB---------//
    //   P80 输出 0
    bFM_GPIO_PDOR8_P0 = 0;
    bFM_GPIO_DDR8_P0 = 1u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR8_P0 = 0u; //GPIO

    //  P81 输出 1
    bFM_GPIO_PDOR8_P1 = 1;
    bFM_GPIO_DDR8_P1 = 1u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR8_P1 = 0u; //GPIO
                           //----------------------------------------------------------------------------------------------//
    //  P50 模拟输入----HW
    bFM_GPIO_PCR5_P0 = 0;   //0，上拉禁止；    1，上拉使能
    bFM_GPIO_DDR5_P0 = 0u;  //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR5_P0 = 1u;  //非GPIO，AD
    bFM_GPIO_ADE_AN22 = 1u; //AD通道选择
                            ///////////////////////////////////////////////////////////////////////////
                            //          P51 GPIO输出0
    bFM_GPIO_PDOR5_P1 = 0;  //输出 0
    bFM_GPIO_PCR5_P1 = 0;   //0，上拉禁止；    1，上拉使能
    bFM_GPIO_DDR5_P1 = 1u;  //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR5_P1 = 0u;  //GPIO
    bFM_GPIO_ADE_AN23 = 0u; //

    //           P52 GPIO输出0
    bFM_GPIO_PDOR5_P2 = 0; //输出 0
    bFM_GPIO_PCR5_P2 = 0;  //0，上拉禁止；    1，上拉使能
    bFM_GPIO_DDR5_P2 = 1u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR5_P2 = 0u; //GPIO
    bFM_GPIO_ADE_AN24 = 0u;
    //-------------------------------------------------------------------------//
    //        P30 GPIO输出0
    bFM_GPIO_PDOR3_P0 = 0;
    bFM_GPIO_PCR3_P0 = 0;   //0，上拉禁止；    1，上拉使能
    bFM_GPIO_DDR3_P0 = 1u;  //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR3_P0 = 0u;  //GPIO
    bFM_GPIO_ADE_AN25 = 0u; //AD通道选择

    //           P31 GPIO输出0---SCL
    bFM_GPIO_PDOR3_P1 = 0;
    bFM_GPIO_PCR3_P1 = 0;  //0，上拉禁止；    1，上拉使能
    bFM_GPIO_DDR3_P1 = 1u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR3_P1 = 0u; //GPIO
    bFM_GPIO_ADE_AN26 = 0u;

    //            P32 GPIO输出0---SDA
    bFM_GPIO_PDOR3_P2 = 0;
    bFM_GPIO_PCR3_P2 = 0;  //0，上拉禁止；    1，上拉使能
    bFM_GPIO_DDR3_P2 = 1u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR3_P2 = 0u; //GPIO

    //            P33 输入---DOOR
    bFM_GPIO_PCR3_P3 = 1;  //0，上拉禁止；    1，上拉使能
    bFM_GPIO_DDR3_P3 = 0u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR3_P3 = 0u; //GPIO

    //             P39 输入---LED ONLY
    bFM_GPIO_PCR3_P9 = 1;  //0，上拉禁止；    1，上拉使能
    bFM_GPIO_DDR3_P9 = 0u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR3_P9 = 0u; //GPIO

    //           P3A 输入---SW7
    bFM_GPIO_PCR3_PA = 1;  //0，上拉禁止；    1，上拉使能
    bFM_GPIO_DDR3_PA = 0u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR3_PA = 0u; //GPIO

    //           P3B 输出0----LIGHT 输出
    bFM_GPIO_PDOR3_PB = 0; //初始输出值
    bFM_GPIO_DDR3_PB = 1u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR3_PB = 0u; //GPIO

    //            P3C 输入----SW6
    bFM_GPIO_PCR3_PC = 1;  //0，上拉禁止；    1，上拉使能
    bFM_GPIO_DDR3_PC = 0u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR3_PC = 0u; //GPIO

    //            P3D 输入-----SW5
    bFM_GPIO_PCR3_PD = 1;  //0，上拉禁止；    1，上拉使能
    bFM_GPIO_DDR3_PD = 0u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR3_PD = 0u; //GPIO

    //            P3E 输入-----SW4
    bFM_GPIO_PCR3_PE = 1;  //0，上拉禁止；    1，上拉使能
    bFM_GPIO_DDR3_PE = 0u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR3_PE = 0u; //GPIO

    //           P3F 输入---SW3
    bFM_GPIO_PCR3_PF = 1;  //0，上拉禁止；    1，上拉使能
    bFM_GPIO_DDR3_PF = 0u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR3_PF = 0u; //GPIO

    //                 P46 输入---SW2
    bFM_GPIO_PCR4_P6 = 0;
    bFM_GPIO_PDOR4_P6 = 1; //0，上拉禁止；    1，上拉使能
    bFM_GPIO_DDR4_P6 = 0u;
    bFM_GPIO_SPSR_SUBXC0 = 0u;
    bFM_GPIO_SPSR_SUBXC1 = 0u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR4_P6 = 0u;     //GPIO

    //                 P47 输入---SW1
    bFM_GPIO_PCR4_P7 = 0;
    bFM_GPIO_PDOR4_P7 = 1; //0，上拉禁止；    1，上拉使能
    bFM_GPIO_DDR4_P7 = 0u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_SPSR_SUBXC0 = 0u;
    bFM_GPIO_SPSR_SUBXC1 = 0u;
    bFM_GPIO_PFR4_P7 = 0u; //GPIO

    //                  P49 输出0---预留
    bFM_GPIO_PDOR4_P9 = 0; //初始输出值
    bFM_GPIO_DDR4_P9 = 1u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR4_P9 = 0u; //GPIO
                           //                  P4A 输出0---预留
    bFM_GPIO_PDOR4_PA = 0; //初始输出值
    bFM_GPIO_DDR4_PA = 1u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR4_PA = 0u; //GPIO
                           //                 P4B 输出0---预留
    bFM_GPIO_PDOR4_PB = 0; //初始输出值
    bFM_GPIO_DDR4_PB = 1u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR4_PB = 0u; //GPIO

    //          P4C 输出0----预留
    bFM_GPIO_PDOR4_PC = 0; //初始输出值
    bFM_GPIO_DDR4_PC = 1u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR4_PC = 0u; //GPIO

    //          P4D 输出0--预留
    bFM_GPIO_PDOR4_PD = 0; //初始输出值
    bFM_GPIO_DDR4_PD = 1u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR4_PD = 0u; //GPIO

    //          P4E 输出0--预留
    bFM_GPIO_PDOR4_PE = 0; //初始输出值
    bFM_GPIO_DDR4_PE = 1u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR4_PE = 0u; //GPIO

    //          PE0 输出0--预留
    bFM_GPIO_PDORE_P0 = 0u; //初始输出值
    bFM_GPIO_DDRE_P0 = 1u;  //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFRE_P0 = 0u;  //GPIO

    //MD0------未定义；还需确认

    //         P10 输出0---485 RE
    bFM_GPIO_PDOR1_P0 = 0;
    bFM_GPIO_DDR1_P0 = 1u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR1_P0 = 0u; //GPIO
    bFM_GPIO_ADE_AN00 = 0u;

    //RX             P11 输入
    bFM_GPIO_PCR1_P1 = 1;  //0，上拉禁止；    1，上拉使能
    bFM_GPIO_DDR1_P1 = 0u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR1_P1 = 0u; //GPIO
    bFM_GPIO_ADE_AN01 = 0u;

    //TX            P12 输出 1
    bFM_GPIO_PDOR1_P2 = 1; //初始输出值
    bFM_GPIO_DDR1_P2 = 1u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR1_P2 = 0u; //GPIO
    bFM_GPIO_ADE_AN02 = 0u;

    //RX2             P14 输入
    bFM_GPIO_PCR1_P4 = 1;  //0，上拉禁止；    1，上拉使能
    bFM_GPIO_DDR1_P4 = 0u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR1_P4 = 0u; //GPIO
    bFM_GPIO_ADE_AN04 = 0u;

    //TX2           P15--- 输出 1
    bFM_GPIO_PDOR1_P5 = 1; //初始输出值
    bFM_GPIO_DDR1_P5 = 1u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR1_P5 = 0u; //GPIO
    bFM_GPIO_ADE_AN05 = 0u;

    //RX1              P17--- 输入
    bFM_GPIO_PCR1_P7 = 1;  //0，上拉禁止；    1，上拉使能
    bFM_GPIO_DDR1_P7 = 0u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR1_P7 = 0u; //GPIO
    bFM_GPIO_ADE_AN07 = 0u;

    //TX1              P18--- 输出 1
    bFM_GPIO_PDOR1_P8 = 1; //初始输出值
    bFM_GPIO_DDR1_P8 = 1u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR1_P8 = 0u; //GPIO
    bFM_GPIO_ADE_AN08 = 0u;

    //       P19 输出----0
    bFM_GPIO_PDOR1_P9 = 0; //初始输出值
    bFM_GPIO_DDR1_P9 = 1u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR1_P9 = 0u; //GPIO
    bFM_GPIO_ADE_AN09 = 0u;

    // BUZZ          P23 输出0
    bFM_GPIO_PDOR2_P3 = 0; //初始输出值
    bFM_GPIO_DDR2_P3 = 1u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR2_P3 = 0u; //GPIO
    bFM_GPIO_ADE_AN12 = 0u;

    // 未用            P22 输出 0
    bFM_GPIO_PDOR2_P2 = 0; //初始输出值
    bFM_GPIO_DDR2_P2 = 1u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR2_P2 = 0u; //GPIO
    bFM_GPIO_ADE_AN13 = 0u;

    //未用              P21输出
    bFM_GPIO_PDOR2_P1 = 0; //0，上拉禁止；    1，上拉使能
    bFM_GPIO_DDR2_P1 = 1u; //0，初始化为输入；1,初始化为输出。
    bFM_GPIO_PFR2_P1 = 0u; //GPIO
    bFM_GPIO_ADE_AN14 = 0u;

#if 0
   //            P3C 输出0----7797控制,用于31855供电
   bFM_GPIO_PDOR3_PC=0;                                                         //初始输出值
   bFM_GPIO_DDR3_PC=1u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_PFR3_PC=0u;                                                         //GPIO
////////////////////////////////////////////////////////////////////////////////
/*
   //           P50 GPIO输出0--buzz
   bFM_GPIO_PDOR5_P0=0;
   bFM_GPIO_PCR5_P0=0;                                                          //0，上拉禁止；    1，上拉使能
   bFM_GPIO_DDR5_P0=1u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_PFR5_P0=0u;                                                         //   GPIO
   bFM_GPIO_ADE_AN22=0u;                                                        //

   //          P51 GPIO输出0
   bFM_GPIO_PDOR5_P1=0;
   bFM_GPIO_PCR5_P1=0;                                                          //0，上拉禁止；    1，上拉使能
   bFM_GPIO_DDR5_P1=1u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_PFR5_P1=0u;                                                         //GPIO
   bFM_GPIO_ADE_AN23=0u;                                                        //
   //           P52 GPIO输出0---报警灯
   bFM_GPIO_PDOR5_P2=0;
   bFM_GPIO_PCR5_P2=0;                                                          //0，上拉禁止；    1，上拉使能
   bFM_GPIO_DDR5_P2=1u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_PFR5_P2=0u;                                                         //GPIO
   bFM_GPIO_ADE_AN24=0u;                                                        //
*/   
/////////////////////////////////////////////////////////////////////////////////
// 充电控制管脚
   //           P3A 输入---PG
   bFM_GPIO_PCR3_PA=1;                                                          //0，上拉禁止；    1，上拉使能                  
   bFM_GPIO_DDR3_PA=0u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_PFR3_PA=0u;                                                         //GPIO
   //             P3B 输出0----CE
   bFM_GPIO_PDOR3_PB=0;                                                         //初始输出值
   bFM_GPIO_DDR3_PB=1u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_PFR3_PB=0u;                                                         //GPIO
   
   //            P33 输入---state2
   bFM_GPIO_PCR3_P3=1;                                                          //0，上拉禁止；    1，上拉使能                  
   bFM_GPIO_DDR3_P3=0u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_PFR3_P3=0u;                                                         //GPIO
   
   //             P39 输入---state1
   bFM_GPIO_PCR3_P3=1;                                                          //0，上拉禁止；    1，上拉使能                  
   bFM_GPIO_DDR3_P3=0u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_PFR3_P3=0u;                                                         //GPIO
//////////////////////////////////////////////////////////////////////////
  /*
   // EE CTL
   //              P62输出1 --SCK
   bFM_GPIO_PDOR6_P2=1;                                                         //初始输出值
   bFM_GPIO_DDR6_P2=1u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_PFR6_P2=0u;                                                         //GPIO 
   bFM_GPIO_ADE_AN19=0u;
   
   //              P61输出1 --DATA     
   bFM_GPIO_PDOR6_P1=1;                                                         //初始输出值
   bFM_GPIO_DDR6_P1=1u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_PFR6_P1=0u;                                                         //GPIO 
    bFM_GPIO_ADE_AN20=0u; 
   */   
    ////////////////////////////////////////////////////////////////////////
  /*
   //            P32 输入---KEY
   bFM_GPIO_PCR3_P2=1;                                                          //0，上拉禁止；    1，上拉使能                  
   bFM_GPIO_DDR3_P2=0u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_PFR3_P2=0u;                                                         //GPIO
  */
   //////////////////////////////////////////////////////////////////////////
// SEN1 CTL
   //          P4D 输出--data
   bFM_GPIO_PDOR4_PD=0;               
   bFM_GPIO_DDR4_PD=1u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_PFR4_PD=0u;                                                         //GPIO
//   bFM_GPIO_ADE_AN00=0u;
/*
   bFM_GPIO_PCR4_PD=1;                                                          //0，上拉禁止；    1，上拉使能                  
   bFM_GPIO_DDR4_PD=0u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_PFR4_PD=0u;                                                         //GPIO
*/   
   
   //         P10 输出0---sck
   bFM_GPIO_PDOR1_P0=0;               
   bFM_GPIO_DDR1_P0=1u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_PFR1_P0=0u;                                                         //GPIO
   bFM_GPIO_ADE_AN00=0u;

/////////////////////////////////////////////////////////////////////////
 /*
   //          P51 GPIO输出0
   bFM_GPIO_PDOR5_P1=0;
   bFM_GPIO_PCR5_P1=0;                                                          //0，上拉禁止；    1，上拉使能
   bFM_GPIO_DDR5_P1=1u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_PFR5_P1=0u;                                                         //GPIO
   bFM_GPIO_ADE_AN23=0u;                                                        //
   
   //        P30 模拟输入
   bFM_GPIO_PCR3_P0=0;                                                          //0，上拉禁止；    1，上拉使能
   bFM_GPIO_DDR3_P0=0u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_PFR3_P0=1u;                                                         //非GPIO，AD
   bFM_GPIO_ADE_AN25=1u;                                                        //AD通道选择
   
   //           P31 模拟输入 
   bFM_GPIO_PCR3_P1=0;                                                          //0，上拉禁止；    1，上拉使能
   bFM_GPIO_DDR3_P1=0u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_PFR3_P1=1u;                                                         //非GPIO，AD
   bFM_GPIO_ADE_AN26=1u;                                                        //AD通道选择
   
   */
   
   /*
   //                  P46 输出0
   bFM_GPIO_PDOR4_P6=0;                                                         //初始输出值
   bFM_GPIO_DDR4_P6=1u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_SPSR_SUBXC0=0u;
   bFM_GPIO_SPSR_SUBXC1=0u; 
   bFM_GPIO_PFR4_P6=0u;                                                         //GPIO
   
   //                  P47 输出0
   bFM_GPIO_PDOR4_P7=0;                                                         //初始输出值
   bFM_GPIO_DDR4_P7=1u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_SPSR_SUBXC0=0u;
   bFM_GPIO_SPSR_SUBXC1=0u;
   bFM_GPIO_PFR4_P7=0u;                                                         //GPIO
   */
      
   //          P4C 输出0
   bFM_GPIO_PDOR4_PC=0;                                                         //初始输出值
   bFM_GPIO_DDR4_PC=1u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_PFR4_PC=0u;                                                         //GPIO
//   bFM_GPIO_PDOR4_PC=1;
//    bFM_GPIO_PDOR4_PC=0;
//	 bFM_GPIO_PDOR4_PC=1;
   
   //          P4E 输入
   bFM_GPIO_PCR4_PE=1;                                                          //0，上拉禁止；    1，上拉使能                  
   bFM_GPIO_DDR4_PE=0u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_PFR4_PE=0u;                                                         //GPIO

    //RX2             P11 输入
   bFM_GPIO_PCR1_P1=1;                                                          //0，上拉禁止；    1，上拉使能                  
   bFM_GPIO_DDR1_P1=0u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_PFR1_P1=0u;                                                         //GPIO
   bFM_GPIO_ADE_AN01=0u;
   
    //TX2             P12 输出 1
   bFM_GPIO_PDOR1_P2=1;                                                         //初始输出值
   bFM_GPIO_DDR1_P2=1u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_PFR1_P2=0u;                                                         //GPIO 
   bFM_GPIO_ADE_AN02=0u;

   //          P14 输出 0
   bFM_GPIO_PDOR1_P4=0;                                                         //初始输出值
   bFM_GPIO_DDR1_P4=1u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_PFR1_P4=0u;                                                         //GPIO 
   bFM_GPIO_ADE_AN04=0u;
   
   //         P15 输出 0
   bFM_GPIO_PDOR1_P5=0;                                                         //初始输出值
   bFM_GPIO_DDR1_P5=1u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_PFR1_P5=0u;                                                         //GPIO 
   bFM_GPIO_ADE_AN05=0u;
   
   //RX1              P17 输入
   bFM_GPIO_PCR1_P7=1;                                                          //0，上拉禁止；    1，上拉使能                  
   bFM_GPIO_DDR1_P7=0u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_PFR1_P7=0u;                                                         //GPIO
   bFM_GPIO_ADE_AN07=0u;
   
   //TX1              P18 输出 1
   bFM_GPIO_PDOR1_P8=1;                                                         //初始输出值
   bFM_GPIO_DDR1_P8=1u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_PFR1_P8=0u;                                                         //GPIO 
   bFM_GPIO_ADE_AN08=0u;
   
   //       P19 输出0
   bFM_GPIO_PDOR1_P9=0;                                                         //初始输出值
   bFM_GPIO_DDR1_P9=1u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_PFR1_P9=0u;                                                         //GPIO 
   bFM_GPIO_ADE_AN09=0u;
   
   //           P23 输出1
   bFM_GPIO_PDOR2_P3=1;                                                         //初始输出值
   bFM_GPIO_DDR2_P3=1u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_PFR2_P3=0u;                                                         //GPIO 
   bFM_GPIO_ADE_AN12=0u;
   
   //TX0              P22 输出 1
   bFM_GPIO_PDOR2_P2=1;                                                         //初始输出值
   bFM_GPIO_DDR2_P2=1u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_PFR2_P2=0u;                                                         //GPIO 
   bFM_GPIO_ADE_AN13=0u; 
    
   //RX0              P21输入
   bFM_GPIO_PCR2_P1=1;                                                          //0，上拉禁止；    1，上拉使能                  
   bFM_GPIO_DDR2_P1=0u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_PFR2_P1=0u;                                                         //GPIO
   bFM_GPIO_ADE_AN14=0u;
 /* 
   //         P02输出0      
   bFM_GPIO_PDOR0_P2=0;                                                         //初始输出值
   bFM_GPIO_DDR0_P2=1u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_PFR0_P2=0u;                                                         //GPIO 
   
   
   //           P0C输出0      
   bFM_GPIO_PDOR0_PC=0;                                                         //初始输出值
   bFM_GPIO_DDR0_PC=1u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_PFR0_PC=0u;                                                         //GPIO 
   bFM_GPIO_ADE_AN16=0u;
   
   //          P0F输出0      
   bFM_GPIO_PDOR0_PF=0;                                                         //初始输出值
   bFM_GPIO_DDR0_PF=1u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_PFR0_PF=0u;                                                         //GPIO 
   bFM_GPIO_ADE_AN18=0u;
 */  
   /*
   //        P60 外中断输入
   bFM_GPIO_PCR6_P0=1;                                                          //0，上拉禁止；    1，上拉使能
   bFM_GPIO_DDR6_P0=0u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_PFR6_P0=1u;                                                         //非GPIO,外部中断
   bFM_GPIO_ADE_AN20=0u; 
   
   //               P80 输出 1
   bFM_GPIO_PDOR8_P0=1;               
   bFM_GPIO_DDR8_P0=1u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_PFR8_P0=0u;                                                         //GPIO
   
   
   //               P81 输出 1
   bFM_GPIO_PDOR8_P1=1;               
   bFM_GPIO_DDR8_P1=1u;                                                         //0，初始化为输入；1,初始化为输出。
   bFM_GPIO_PFR8_P1=0u;                                                         //GPIO
   */
#endif
}

///////////////硬件看门狗/////////////////////
void WdgHwCallback(void)
{

    //Hwwdg_Feed(0x55, 0xAA);  @20181017 CFJ
}
void Hdware_Watchdog_Init(void)
{
    stc_hwwdg_config_t stcHwwdgConfig;
    stcHwwdgConfig.u32LoadValue = 100000; // Interval:1s (@CLKLC:100kHz)
    stcHwwdgConfig.bResetEnable = TRUE;   // Enables Hardware watchdog reset
    stcHwwdgConfig.pfnHwwdgIrqCb = WdgHwCallback;
    Hwwdg_Init(&stcHwwdgConfig);
    Hwwdg_Start();
}
void Clear_Watchdog(void)
{
    Hwwdg_Feed(0x55, 0xAA);
}

///////////////ADC0///////////////////////

unsigned int ADC_GetValue(unsigned char Channel)
{

    stc_adc_config_t stcConfig;
    stc_adc_scan_t stcScanCfg;
    uint32_t u32Data;
    // Clear structures
    PDL_ZERO_STRUCT(stcConfig);
    PDL_ZERO_STRUCT(stcScanCfg);

    stcScanCfg.u32ScanCannelSelect.u32AD_CHn = (1 << Channel);
    stcScanCfg.enScanMode = ScanSingleConversion;
    stcScanCfg.enScanTimerTrigger = AdcNoTimer;
    stcScanCfg.bScanTimerStartEnable = FALSE;
    stcScanCfg.u8ScanFifoDepth = 0;
    // For FM4 and FM0+, the calculation of sampling time and compare time is shown as following:
    // Sampling time = HCLK cycle * Frequency division ratio * {(ST set value + 1) * STX setting multiplier + 3}
    // At the following configuration:
    // Sampling time = 5ns * 5 * {(8+1)*8+3} = 1.875us (if HCLK = 200MHz)
    // Sampling time = 25ns * 5 * {(8+1)*8+3} = 9.375us (if HCLK = 40MHz)

    ////Sampling time = 0.125us * 2 * {(8+1)*8+3} = 18.75us (if HCLK = 8MHz)
    stcConfig.bLsbAlignment = TRUE;
    stcConfig.u32SamplingTimeSelect.u32AD_CHn = 0;
    stcConfig.enSamplingTimeN0 = Value8; // STX setting multiplier  8
    stcConfig.u8SamplingTime0 = 8u;      // ST value 0 : 8
    stcConfig.enSamplingTimeN1 = Value8; // STX setting multiplier 8
    stcConfig.u8SamplingTime1 = 8u;      // ST value 1 : 8
    stcConfig.u8ComparingClockDiv = 0u;  // Frequency division ratio:  0:Ratio 2, 1:Ratio 3, ...
    stcConfig.pstcScanInit = &stcScanCfg;

    if (Ok == Adc_Init(&ADC0, &stcConfig)) ///< Init ADC0
    {
        Adc_EnableWaitReady(&ADC0); ///< Enable ADC0 and wait for ready
    }

    Adc_SwTriggerScan(&ADC0);
    while (FALSE == Adc_GetIrqFlag(&ADC0, AdcScanIrq))
    {
    }
    Adc_ClrIrqFlag(&ADC0, AdcScanIrq);
    u32Data = Adc_ReadScanFifo(&ADC0);
    if (0xFFFFFFFF == u32Data || AdcFifoDataValid != Adc_GetScanDataValid(&ADC0, u32Data))
    {
        return 0; //数据无效
    }
    else
    {
        u32Data = Adc_GetScanData(&ADC0, u32Data);
        u32Data = u32Data >> 4;
        return u32Data;
    }
}

////////////////BT1 1ms中断///////////////////////////////////////
void BT1UnderflowIrqHandler(void)
{
    //static U8 tmp_key_time;

    //tmp_key_time++;

    //(void)(TPM2SC==0);
    //TPM2SC_TOF =0;
    u16_random++;
    if (u16_random > 16383)
        u16_random = 1;
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
        f_2Ms_Flag = 1;
        t_2ms++;
        t_twoms++;
        t_1ms = 0;

        if (t_2ms >= 250)
        {
            t_halfsec++;
            t_2ms = 0;
            f_05s = 1; //0.5s
        }
        /*
            if(SelfCheckFlag)   
            {
              
              
            }
            else
            {
               if(g_Txd_Time>0)
                     {
                        g_Txd_Time--;
                     }
                     else
                     {
                        g_Txd_Time=250;
                        bUartSendStartFlag=1;
                     }
              
            }
            */

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

void BT1_Init(void) //定时器1初始化，Reload 模式
{
    stc_bt_rt_config_t stcRtConfig;
    stc_rt_irq_en_t stcIrqEn;
    stc_rt_irq_cb_t stcIrqCb;

    PDL_ZERO_STRUCT(stcRtConfig);
    PDL_ZERO_STRUCT(stcIrqEn);
    PDL_ZERO_STRUCT(stcIrqCb);

    // Initialize interrupt
    stcRtConfig.pstcRtIrqEn = &stcIrqEn;
    stcRtConfig.pstcRtIrqCb = &stcIrqCb;
    // Set BT IO mode
    //  Bt_ConfigIOMode(&BT0, BtIoMode0);

    // Initialize BT
    stcRtConfig.enPres = RtPres1Div4; // BT_CLK_DIV_4: T = 1us @ PCLK = 4 MHz
    stcRtConfig.enSize = RtSize16Bit;
    stcRtConfig.enMode = RtReload;
    stcRtConfig.enExtTrig = RtExtTiggerDisable;
    stcRtConfig.enOutputPolarity = RtPolarityLow;
    // stcRtConfig.pstcRtIrqEn->bRtTrigIrq = 1;
    stcRtConfig.pstcRtIrqEn->bRtUnderflowIrq = 1;
    // stcRtConfig.pstcRtIrqCb->pfnRtTrigIrqCb = RtTrigIrqHandler ;
    stcRtConfig.pstcRtIrqCb->pfnRtUnderflowIrqCb = BT1UnderflowIrqHandler;
    stcRtConfig.bTouchNvic = TRUE;
    Bt_Rt_Init(&BT1, &stcRtConfig);

    // Write cycle value
    Bt_Rt_WriteCycleVal(&BT1, 999); // Cycle = T*(m+1)= 1000us @ T = 1 us
    Bt_Rt_EnableCount(&BT1);
    Bt_Rt_EnableSwTrig(&BT1);
}

////////////////BT2 125us中断///////////////////////////////////////
void BT2UnderflowIrqHandler(void)
{
    /*
   if(BuzzEnable)
   {     
      bFM_GPIO_PDOR5_P2=~bFM_GPIO_PDOR5_P2;
   }
   */
}

void BT2_Init(void) //定时器1初始化，Reload 模式
{

    stc_bt_rt_config_t stcRtConfig;
    stc_rt_irq_en_t stcIrqEn;
    stc_rt_irq_cb_t stcIrqCb;

    PDL_ZERO_STRUCT(stcRtConfig);
    PDL_ZERO_STRUCT(stcIrqEn);
    PDL_ZERO_STRUCT(stcIrqCb);

    // Initialize interrupt
    stcRtConfig.pstcRtIrqEn = &stcIrqEn;
    stcRtConfig.pstcRtIrqCb = &stcIrqCb;
    // Set BT IO mode
    //  Bt_ConfigIOMode(&BT0, BtIoMode0);

    // Initialize BT
    stcRtConfig.enPres = RtPres1Div4; // BT_CLK_DIV_4: T = 1/9us @ PCLK = 36 MHz
    stcRtConfig.enSize = RtSize16Bit;
    stcRtConfig.enMode = RtReload;
    stcRtConfig.enExtTrig = RtExtTiggerDisable;
    stcRtConfig.enOutputPolarity = RtPolarityLow;
    // stcRtConfig.pstcRtIrqEn->bRtTrigIrq = 1;                                  //
    stcRtConfig.pstcRtIrqEn->bRtUnderflowIrq = 1;
    // stcRtConfig.pstcRtIrqCb->pfnRtTrigIrqCb = RtTrigIrqHandler ;
    stcRtConfig.pstcRtIrqCb->pfnRtUnderflowIrqCb = BT2UnderflowIrqHandler;
    stcRtConfig.bTouchNvic = TRUE;
    Bt_Rt_Init(&BT2, &stcRtConfig);

    // Write cycle value
    Bt_Rt_WriteCycleVal(&BT2, 1124); //
    Bt_Rt_EnableCount(&BT2);
    Bt_Rt_EnableSwTrig(&BT2);
}

void Beep_Init(void)
{
    stc_bt_pwm_config_t stcPwmConfig;
    stc_pwm_irq_en_t stcIrqEn;
    stc_pwm_irq_cb_t stcIrqCb;

    SetPinFunc_TIOA7_1_OUT(); //设置PWM输出引脚
    // Clear structures
    PDL_ZERO_STRUCT(stcPwmConfig);
    PDL_ZERO_STRUCT(stcIrqEn);
    PDL_ZERO_STRUCT(stcIrqCb);

    // Initialize interrupt
    // stcPwmConfig.pstcPwmIrqEn = &stcIrqEn;
    //stcPwmConfig.pstcPwmIrqCb = &stcIrqCb;
    // Set requested I/O mode
    Bt_ConfigIOMode(&BT7, BtIoMode0);

    stcPwmConfig.enPres = PwmPresNone; // PWM clock = 4MHz
    stcPwmConfig.enMode = PwmContinuous;
    stcPwmConfig.enExtTrig = PwmExtTrigDisable;
    stcPwmConfig.enOutputMask = PwmOutputNormal;
    stcPwmConfig.enOutputPolarity = PwmPolarityLow;
    stcPwmConfig.enRestartEn = PwmRestartEnable;
    // stcPwmConfig.pstcPwmIrqEn->bPwmTrigIrq = 0;
    // stcPwmConfig.pstcPwmIrqEn->bPwmDutyMatchIrq = 0;
    // stcPwmConfig.pstcPwmIrqEn->bPwmUnderflowIrq = 0;
    // stcPwmConfig.pstcPwmIrqCb->pfnPwmTrigIrqCb = PwmTrigIrqHandler;
    // stcPwmConfig.pstcPwmIrqCb->pfnPwmDutyMatchIrqCb = PwmDutyMatchIrqHandler;
    // stcPwmConfig.pstcPwmIrqCb->pfnPwmUnderflowIrqCb = PwmUnderflowIrqHandler;
    //  stcPwmConfig.bTouchNvic = TRUE;
    Bt_Pwm_Init(&BT7, &stcPwmConfig);
    // Set cycle and duty value
    Bt_Pwm_WriteCycleVal(&BT7, 999); // Cycle = (1+m)*PWM clock cycle = 200us
    Bt_Pwm_WriteDutyVal(&BT7, 49);   // Duty = (1+m)*PWM clock cycle = 100us

    // Enable count operatoin
    // Bt_Pwm_EnableCount(&USER_BT);
}
void PWM_Start(void)
{
#if FORBID_BUZZ //禁止蜂鸣器响

#else
    Bt_Pwm_EnableCount(&BT7);
    Bt_Pwm_EnableSwTrig(&BT7);
#endif
}
void PWM_Stop(void)
{
    Bt_Pwm_DisableCount(&BT7);
}
////////////UART 初始化（MFS 4）///////////////

void UartTxCallback(void)
{
    if (g_UART1_TXD_Counter > 29) //> TX_UART_BUF_LEN) //9个数
    {
        Mfs_Uart_DisableIrq(&UART1, UartTxIdleIrq); //UartTxIrq);
        g_UART1_TXD_Counter = 0;
        //Delay(400);
        RS_485_STATUS = 0;
        g_Rec_Status_Flag = 1; //发送完，转为接受状态
    }
    else
    {
        Mfs_Uart_SendData(&UART1, g_IDM_TX_ODM_Data[g_UART1_TXD_Counter]);

        g_UART1_TXD_Counter++;
    }
}
void UartRxCallback(void) //显示板接受主控板的UART
{
    unsigned char rx_data; //volatile static uint8_t tmp_data=0;
    unsigned char UART1_RCV_DataBuf;
    unsigned char NCYC;
    if ((&UART1)->SSR_f.PE == 1 || (&UART1)->SSR_f.FRE == 1 || (&UART1)->SSR_f.ORE == 1) //如有错误则清除标志
    {
        (&UART1)->SSR_f.REC = 1;
    }

    rx_data = Mfs_Uart_ReceiveData(&UART1);
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

void MFS1_UART_Init(void) //UART 初始化 9600 主控板与面板通讯
{
    stc_mfs_uart_config_t stcUartConfig; //UART1
    stc_uart_irq_cb_t stcUart1IrqCb;

    PDL_ZERO_STRUCT(stcUartConfig);
    PDL_ZERO_STRUCT(stcUart1IrqCb);

    SetPinFunc_SIN1_1();
    SetPinFunc_SOT1_1();

    stcUart1IrqCb.pfnTxIrqCb = UartTxCallback;
    stcUart1IrqCb.pfnRxIrqCb = UartRxCallback;

    stcUartConfig.enMode = UartNormal;
    stcUartConfig.u32BaudRate = 9600; //2400;
    stcUartConfig.enDataLength = UartEightBits;
    stcUartConfig.enParity = UartParityNone;
    stcUartConfig.enStopBit = UartOneStopBit;
    stcUartConfig.enBitDirection = UartDataLsbFirst;
    stcUartConfig.bInvertData = FALSE;
    stcUartConfig.bHwFlow = FALSE;
    stcUartConfig.pstcFifoConfig = NULL;
    stcUartConfig.bUseExtClk = FALSE;
    stcUartConfig.pstcIrqEn = NULL;
    stcUartConfig.pstcIrqCb = &stcUart1IrqCb;
    stcUartConfig.bTouchNvic = TRUE;

    Mfs_Uart_Init(&UART1, &stcUartConfig);

    Mfs_Uart_EnableFunc(&UART1, UartRx);
    Mfs_Uart_EnableFunc(&UART1, UartTx); //使能发送接收

    Mfs_Uart_DisableIrq(&UART1, UartTxIrq); //发送中断无效 @20181008 CFJ

    Mfs_Uart_EnableIrq(&UART1, UartRxIrq); //使能接收中断

    // Mfs_Uart_EnableIrq(&UART1, UartRxIrq);
    // Mfs_Uart_EnableIrq(&UART1, UartTxIrq);  //使能发送接收中断
}
//-----------------------------------------------------------------------//
///////////UART0 初始化（MFS 0）///////////////

void Uart0TxCallback(void) //打印机 发送
{
    //volatile static uint8_t j=0;
    //uchar Sendata ;

    ///////Sendata = SCI2S1;

    //// SCI2D = u8_Send2dataMemory[u8_Send2_data_Num_Count];
    if (u8_Send2_data_Num_Count >= u8_Send2_data_Num) //if(++u8_Send2_data_Num_Count>=u8_Send2_data_Num)
    {
        Mfs_Uart_DisableIrq(&UART0, UartTxIrq); //无效发送中断///SCI2C2_TCIE = 0;
        u8_Send2_data_State = 0;
    }
    else
    {
        Mfs_Uart_SendData(&UART0, u8_Send2dataMemory[u8_Send2_data_Num_Count]); //g_IDM_TX_ODM_Data[g_UART1_TXD_Counter]); @20181130 CFJ
        u8_Send2_data_Num_Count++;
    }
}

void Uart0RxCallback(void)
{
    //
}

void MFS0_UART0_Init(void) //UART0 初始化 9600 打印机通讯 初始化
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
    stcUartConfig.u32BaudRate = 115200; //wifi模块115200，打印机9600
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

    Mfs_Uart_EnableFunc(&UART0, UartRx); ////使能接收
    Mfs_Uart_EnableFunc(&UART0, UartTx); //使能发送

    Mfs_Uart_EnableIrq(&UART0, UartRxIrq);  //使能接受中断
    Mfs_Uart_DisableIrq(&UART0, UartTxIrq); //无效发送中断
}
//---------------------------------------------------------------------------//

///////////UART1 初始化（MFS 2）///////////////

void Uart1TxCallback(void)
{
    //unsigned char Tx_data;
    /////a = SCI1S1;                                    //清发送中断请求位，首先读SCI1S1然后写SCI1D
    if (f_send_data)
    {
        if (f_send55)
        {
            f_send55 = 0;
            Mfs_Uart_SendData(&UART2, 0x55); //SCI1D = 0x55;
            if (r_sendr >= r_sendsum)
            {
                Mfs_Uart_DisableIrq(&UART2, UartTxIrq); //无效发送中断//SCI1C2_TCIE = 0;                           //关闭发送中断
                f_send_1ff = 0;
                f_send_2ff = 0;
                f_send_data = 0;
            }
        }
        else if (send_net[r_sendr] == 0xff)
        {
            f_send55 = 1;
            Mfs_Uart_SendData(&UART2, 0xff); //SCI1D = 0xff;
            r_sendr++;
        }
        else
        {
            Mfs_Uart_SendData(&UART2, send_net[r_sendr]); //SCI1D = send_net[r_sendr];
            r_sendr++;
            if (r_sendr >= r_sendsum)
            {
                Mfs_Uart_DisableIrq(&UART2, UartTxIrq); //无效发送中断//SCI1C2_TCIE = 0;                           //关闭发送模块
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
        Mfs_Uart_SendData(&UART2, r_sendsum); ////SCI1D = r_sendsum;
    }
    else if (f_send_1ff)
    {
        f_send_2ff = 1;
        Mfs_Uart_SendData(&UART2, 0xff); //SCI1D = 0xff;
    }
    else
    {
        f_send_1ff = 1;
        Mfs_Uart_SendData(&UART2, 0xff); //////SCI1D = 0xff;
    }
}

void Uart1RxCallback(void)
{
    unsigned char rx_data;
    if ((&UART2)->SSR_f.PE == 1 || (&UART2)->SSR_f.FRE == 1 || (&UART2)->SSR_f.ORE == 1) //如有错误则清除标志
    {
        (&UART2)->SSR_f.REC = 1;
    }

    /////a = SCI1S1;  //清接收中断请求位，首先读SCI1S1然后读SCI1D
    rx_data = Mfs_Uart_ReceiveData(&UART2);
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

void MFS2_UART1_Init(void) //UART2 初始化 9600 USB 通讯初始化
{
    stc_mfs_uart_config_t stcUartConfig; //UART0
    stc_uart_irq_cb_t stcUart2IrqCb;

    PDL_ZERO_STRUCT(stcUartConfig);
    PDL_ZERO_STRUCT(stcUart2IrqCb);

    SetPinFunc_SIN2_2();
    SetPinFunc_SOT2_2();

    stcUart2IrqCb.pfnTxIrqCb = Uart1TxCallback;
    stcUart2IrqCb.pfnRxIrqCb = Uart1RxCallback;

    stcUartConfig.enMode = UartNormal;
    stcUartConfig.u32BaudRate = 9600;
    stcUartConfig.enDataLength = UartEightBits;
    stcUartConfig.enParity = UartParityNone;
    stcUartConfig.enStopBit = UartOneStopBit;
    stcUartConfig.enBitDirection = UartDataLsbFirst;
    stcUartConfig.bInvertData = FALSE;
    stcUartConfig.bHwFlow = FALSE;
    stcUartConfig.pstcFifoConfig = NULL;
    stcUartConfig.bUseExtClk = FALSE;
    stcUartConfig.pstcIrqEn = NULL;
    stcUartConfig.pstcIrqCb = &stcUart2IrqCb;
    stcUartConfig.bTouchNvic = TRUE;

    Mfs_Uart_Init(&UART2, &stcUartConfig);

    Mfs_Uart_EnableFunc(&UART2, UartRx); //使能接收
    Mfs_Uart_EnableFunc(&UART2, UartTx); //使能接收

    Mfs_Uart_EnableIrq(&UART2, UartRxIrq);  //使能接收中断
    Mfs_Uart_DisableIrq(&UART2, UartTxIrq); //无效发送中断
}
