/*************************************************************************************
* Copyright (C) 2013-2015, Cypress Semiconductor Corporation. All rights reserved.    
*                                                                                     
* This software, including source code, documentation and related                     
* materials ( "Software" ), is owned by Cypress Semiconductor                         
* Corporation ( "Cypress" ) and is protected by and subject to worldwide              
* patent protection (United States and foreign), United States copyright              
* laws and international treaty provisions. Therefore, you may use this               
* Software only as provided in the license agreement accompanying the                 
* software package from which you obtained this Software ( "EULA" ).                  
* If no EULA applies, Cypress hereby grants you a personal, nonexclusive,             
* non-transferable license to copy, modify, and compile the                           
* Software source code solely for use in connection with Cypress's                    
* integrated circuit products. Any reproduction, modification, translation,           
* compilation, or representation of this Software except as specified                 
* above is prohibited without the express written permission of Cypress.              
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO                                
* WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING,                                
* BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED                                        
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A                                     
* PARTICULAR PURPOSE. Cypress reserves the right to make                              
* changes to the Software without notice. Cypress does not assume any                 
* liability arising out of the application or use of the Software or any              
* product or circuit described in the Software. Cypress does not                      
* authorize its products for use in any products where a malfunction or               
* failure of the Cypress product may reasonably be expected to result in              
* significant property damage, injury or death ( "High Risk Product" ). By            
* including Cypress's product in a High Risk Product, the manufacturer                
* of such system or application assumes all risk of such use and in doing             
* so agrees to indemnify Cypress against all liability.                               
*/
/******************************************************************************/
/** \file ppg.h
 **
 ** Headerfile for PPG functions
 **  
 ** History:
 **   - 2014-12-08  0.1  EZh  First version for FM universal PDL.
 **
 *****************************************************************************/

#ifndef __PPG_H__
#define __PPG_H__

#include "mcu.h"
#include "pdl_user.h"

#if (defined(PDL_PERIPHERAL_PPG_ACTIVE))

/**
 ******************************************************************************
 ** \defgroup PpgGroup Programmable Pulse Generator (PPG)
 **
 ** Provided functions of PPG module:
 ** 
 ** - Ppg_Init()
 ** - Ppg_DeInit()
 ** - Ppg_StartSoftwareTrig() 
 ** - Ppg_StopSoftwareTrig()
 ** - Ppg_SelGateLevel()
 ** - Ppg_ConfigUpCnt0()
 ** - Ppg_StartUpCnt0()
 ** - Ppg_GetUpCnt0Status()
 ** - Ppg_DisableTimerGen0StartTrig()
 ** - Ppg_ConfigUpCnt1()
 ** - Ppg_StartUpCnt1()
 ** - Ppg_GetUpCnt1Status()
 ** - Ppg_DisableTimerGen1StartTrig()
 ** - Ppg_ConfigUpCnt2()
 ** - Ppg_StartUpCnt2()
 ** - Ppg_GetUpCnt2Status()
 ** - Ppg_DisableTimerGen2StartTrig()
 ** - Ppg_EnableIrq()
 ** - Ppg_DisableIrq()
 ** - Ppg_GetIrqFlag()
 ** - Ppg_ClrIrqFlag()
 ** - Ppg_SetLevelWidth()
 ** - Ppg_InitIgbt()
 ** - Ppg_EnableIgbtMode()
 ** - Ppg_DisableIgbtMode()
 ** - Ppg_IrqHandler()
 ** 
 ** <b>How to use PPG module?</b>
 **
 ** Ppg_Init() must be used for configuration of a PPG couple channel with 
 ** a structure of the type #stc_ppg_config_t. Three ways can trigger the 
 ** PPG start:
 ** - Triggered by software
 ** - Triggered by up counter
 ** - Triggered by GATE signal from MFT
 **
 ** With Ppg_SetLevelWidth() the PPG low/high level width is set to the value
 ** given in the parameter Ppg_SetLevelWidth#u8LowWidth and 
 ** Ppg_SetLevelWidth#u8LowWidth. Ppg_GetUpCntxStatus() can get the operation
 ** status of up counter (x=0,1,2)
 **
 ** If use software to start PPG, calling Ppg_StartSoftwareTrig() will start PPG.
 **
 ** If use up counter to start PPG, initialize up counter with Ppg_ConfigUpCntx()
 ** with a structure of the type #stc_ppg_upcntx_config_t, start the up counter
 ** with Ppg_StartUpCntx() and if count value matchs with compare value, the 
 ** according PPG channel will start. 
 **
 ** If use GATE signal to trigger PPG, set the valid level with Ppg_SelGateLevel(),
 ** and if GATE signal from MFT becomes valid level, PPG will start.
 **
 ** With interrupt mode, when the interrupt occurs, the interrupt flag will
 ** be cleared and run into user interrupt callback function.
 **
 ** With polling mode, user can use Ppg_GetIrqFlag() to check if the 
 ** interrupt occurs, and clear the interrupt flag by Ppg_ClrIrqFlag().
 ** 
 ** When stopping the PPG, if PPG is triggered by software, use 
 ** Ppg_StopSoftwareTrig() to stop PPG output, if PPG is triggered by up counter,
 ** use Ppg_DisableTimerGen0StartTrig to stop PPG output, if PPG is triggered by
 ** GATE, set the GATE signal to invalid level in the MFT module.
 **
 ** IGBT mode is also supported by PPG. Ppg_InitIgbt() must be used for 
 ** configuration of IGBT mode with a structure of the type 
 ** #stc_ppg_igbt_config_t. 
 **
 ** Ppg_EnableIgbtMode() is used to enable IGBT mode and Ppg_DisableIgbtMode() 
 ** is used to disable IGBT mode.
 **
 ******************************************************************************/
//@{
/******************************************************************************
 * Global type definitions
 ******************************************************************************/
#define  PPG_CH0            0u
#define  PPG_CH1            1u
#define  PPG_CH2            2u
#define  PPG_CH3            3u
#define  PPG_CH4            4u
#define  PPG_CH5            5u
#define  PPG_CH6            6u
#define  PPG_CH7            7u
#define  PPG_CH8            8u
#define  PPG_CH9            9u
#define  PPG_CH10           10u
#define  PPG_CH11           11u
#define  PPG_CH12           12u
#define  PPG_CH13           13u
#define  PPG_CH14           14u
#define  PPG_CH15           15u
#define  PPG_CH16           16u
#define  PPG_CH17           17u
#define  PPG_CH18           18u
#define  PPG_CH19           19u
#define  PPG_CH20           20u
#define  PPG_CH21           21u
#define  PPG_CH22           22u
#define  PPG_CH23           23u

#define  PPG_COUPLE_CH01        0u
#define  PPG_COUPLE_CH23        2u
#define  PPG_COUPLE_CH45        4u
#define  PPG_COUPLE_CH67        6u
#define  PPG_COUPLE_CH89        8u
#define  PPG_COUPLE_CH1011      10u
#define  PPG_COUPLE_CH1213      12u
#define  PPG_COUPLE_CH1415      14u
#define  PPG_COUPLE_CH1617      16u
#define  PPG_COUPLE_CH1819      18u
#define  PPG_COUPLE_CH2021      20u
#define  PPG_COUPLE_CH2223      22u

#define  PPG_INT_MAX_CH     9u  // Including channel.0,2,4,8,10,12,16,18,20

#define  PPG_UP_COUNTER_CH0     0u
#define  PPG_UP_COUNTER_CH1     1u
#define  PPG_UP_COUNTER_CH2     2u

/**
 ******************************************************************************
 ** \brief Enumeration to define PPG operation mode
 ** \note only following combination of 4 channel is valid:
 **       Ch0_Ch1            Ch2_Ch3
 **       Ppg8Bit8Bit        Ppg8Bit8Bit
 **       Ppg8Bit8Bit        Ppg8Bit8Pres
 **       Ppg8Bit8Bit        Ppg16Bit
 **       Ppg8Bit8Pres       Ppg8Bit8Bit
 **       Ppg8Bit8Pres       Ppg8Bit8Pres
 **       Ppg8Bit8Pres       Ppg16Bit
 **       Ppg16Bit           Ppg8Bit8Bit
 **       Ppg16Bit           Ppg8Bit8Pres
 **       Ppg16Bit           Ppg16Bit
 **       Ppg16Bit16Pres     Ppg16Bit16Pres [Act as Prescaler]       
 **
 ******************************************************************************/
typedef enum en_ppg_opt_mode
{
    Ppg8Bit8Bit     = 0u,   ///< Even channel: 8bit PPG, odd channel: 8bit PPG
    Ppg8Bit8Pres    = 1u,   ///< Even channel: 8bit PPG, odd channel: 8bit prescaler
    Ppg16Bit        = 2u,   ///< 16bit PPG
    Ppg16Bit16Pres  = 3u,   ///< 16bit PPG + 16 prescaler (provided by next couple channel)
    
}en_ppg_opt_mode_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define PPG count clock prescaler
 ******************************************************************************/
typedef enum en_ppg_clock
{
    PpgPclkDiv1   = 0u,     ///< PPG count clock prescaler: 1
    PpgPclkDiv4   = 1u,     ///< PPG count clock prescaler: 1/4
    PpgPclkDiv16  = 2u,     ///< PPG count clock prescaler: 1/16
    PpgPclkDiv64  = 3u,     ///< PPG count clock prescaler: 1/64
  
}en_ppg_clock_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define PPG trigger
 ******************************************************************************/
typedef enum en_ppg_trig
{
    PpgSoftwareTrig   = 0u,     ///< Use software to trigger PPG
    PpgMftGateTrig    = 1u,     ///< Use GATE signal from MFT to trigger PPG
    PpgTimingGenTrig  = 2u,     ///< Use Timing Generator to trigger PPG
  
}en_ppg_trig_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define PPG output initial level
 ******************************************************************************/
typedef enum en_ppg_level
{
    PpgNormalLevel   = 0u,      ///< Initial level: Low
    PpgReverseLevel  = 1u,      ///< Initial level: High
 
}en_ppg_level_t;

/**
 ******************************************************************************
 ** \brief PPG interrupt mode
 ******************************************************************************/
typedef enum en_ppg_irq_mode
{
    PpgIrqHighOrLowUnderflow = 0u,   ///< Generate interrupt when high or low width count underflows
    PpgIrqHighUnderflow      = 1u,   ///< Generate interrupt when high width count underflows

}en_ppg_irq_mode_t;

/**
 ******************************************************************************
 ** \brief Structure to config PPG
 ******************************************************************************/
typedef struct stc_ppg_config
{
    en_ppg_opt_mode_t       enMode;       ///< PPG mode configuration
    en_ppg_clock_t          enEvenClock;  ///< Clock prescaler of even channel
    en_ppg_clock_t          enOddClock;   ///< Clock prescaler of odd channel
    en_ppg_level_t          enEvenLevel;  ///< Output level of even channel
    en_ppg_level_t          enOddLevel;   ///< Output level of odd channel
    en_ppg_trig_t           enTrig;       ///< PPG trigger mode configuration
    en_ppg_irq_mode_t       enIrqMode;    ///< PPG interrupt mode
    
    boolean_t               bIrqEnable;     ///< TRUE: enable IRQ, FALSE: don't enable IRQ
    func_ptr_t              pfnIrqCallback; ///< Pointer to PPG interrupt callback function
    boolean_t               bTouchNvic;     ///< TRUE: enable NVIC, FALSE: don't enable NVIC
    
}stc_ppg_config_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define valid level of GATE signal from MFT
 ******************************************************************************/
typedef enum en_ppg_gate_level
{
    PpgGateHighActive = 0u,  ///< GATE valid level: High
    PpgGateLowActive  = 1u,  ///< GATE valid level: Low
    
}en_ppg_gate_level_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define clock prescaler of up counter 
 **        in timing generator
 ******************************************************************************/
typedef enum en_ppg_upcnt_clk
{ 
    PpgUpCntPclkDiv2   = 0u,      ///< Up counter clock prescaler: 1/2
    PpgUpCntPclkDiv8   = 1u,      ///< Up counter clock prescaler: 1/8
    PpgUpCntPclkDiv32  = 2u,      ///< Up counter clock prescaler: 1/32
    PpgUpCntPclkDiv64  = 3u,      ///< Up counter clock prescaler: 1/64
    
}en_ppg_upcnt_clk_t;

/**
 ******************************************************************************
 ** \brief Up counter0 configuration
 ******************************************************************************/
typedef struct stc_ppg_upcnt0_config
{
    en_ppg_upcnt_clk_t   enClk;        ///< Up counter clock prescaler
    uint8_t              u8CmpValue0;  ///< Up counter compare value for Ch0
    uint8_t              u8CmpValue2;  ///< Up counter compare value for Ch2
    uint8_t              u8CmpValue4;  ///< Up counter compare value for Ch4
    uint8_t              u8CmpValue6;  ///< Up counter compare value for Ch6
    
}stc_ppg_upcnt0_config_t;

/**
 ******************************************************************************
 ** \brief Up counter0 stop channels selection
 ******************************************************************************/
typedef struct stc_timer0_gen_ch
{
    boolean_t            bPpgCh0;  ///< 1: select Ch.0, 0: not selected
    boolean_t            bPpgCh2;  ///< 1: select Ch.2, 0: not selected
    boolean_t            bPpgCh4;  ///< 1: select Ch.4, 0: not selected
    boolean_t            bPpgCh6;  ///< 1: select Ch.6, 0: not selected
    
}stc_timer0_gen_ch_t;

/**
 ******************************************************************************
 ** \brief Up counter1 configuration
 ******************************************************************************/
typedef struct stc_ppg_upcnt1_config
{
    en_ppg_upcnt_clk_t   enClk;            ///< Up counter clock prescaler
    uint8_t              u8CmpValue8;      ///< Up counter compare value for Ch8
    uint8_t              u8CmpValue10;     ///< Up counter compare value for Ch10
    uint8_t              u8CmpValue12;     ///< Up counter compare value for Ch12
    uint8_t              u8CmpValue14;     ///< Up counter compare value for Ch14
    
}stc_ppg_upcnt1_config_t;

/**
 ******************************************************************************
 ** \brief Up counter1 stop channels selection
 ******************************************************************************/
typedef struct stc_timer1_gen_ch
{
    boolean_t            bPpgCh8;    ///< 1: select Ch.8, 0: not selected
    boolean_t            bPpgCh10;   ///< 1: select Ch.10, 0: not selected
    boolean_t            bPpgCh12;   ///< 1: select Ch.12, 0: not selected
    boolean_t            bPpgCh14;   ///< 1: select Ch.14, 0: not selected
    
}stc_timer1_gen_ch_t;

/**
 ******************************************************************************
 ** \brief Up counter2 configuration
 ******************************************************************************/
typedef struct stc_ppg_upcnt2_config
{
    en_ppg_upcnt_clk_t   enClk;         ///< Up counter clock prescaler
    uint8_t              u8CmpValue16;  ///< Up counter compare value for Ch16
    uint8_t              u8CmpValue18;  ///< Up counter compare value for Ch18
    uint8_t              u8CmpValue20;  ///< Up counter compare value for Ch20
    uint8_t              u8CmpValue22;  ///< Up counter compare value for Ch22
    
}stc_ppg_upcnt2_config_t;

/**
 ******************************************************************************
 ** \brief Up counter2 stop channels selection
 ******************************************************************************/
typedef struct stc_timer2_gen_ch
{
    boolean_t            bPpgCh16;  ///< 1: select Ch.16, 0: not selected
    boolean_t            bPpgCh18;  ///< 1: select Ch.18, 0: not selected
    boolean_t            bPpgCh20;  ///< 1: select Ch.20, 0: not selected 
    boolean_t            bPpgCh22;  ///< 1: select Ch.22, 0: not selected 
    
}stc_timer2_gen_ch_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define IBGT prohibition mode
 ******************************************************************************/
typedef enum en_igbt_prohibition_mode
{
    IgbtNormalMode             = 0u,       ///< Normal mode
    IgbtStopProhibitionMode    = 1u,       ///< Stop prohibition mode in output active
    
}en_igbt_prohibition_mode_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define IBGT noise filter width
 ******************************************************************************/
typedef enum en_igbt_filter_width
{
    IgbtNoFilter     = 0u,             ///< No noise filter
    IgbtFilter4Pclk  = 1u,             ///< noise filter width: 4PCLK
    IgbtFilter8Pclk  = 2u,             ///< noise filter width: 8PCLK
    IgbtFilter16Pclk = 3u,             ///< noise filter width: 16PCLK 
    IgbtFilter32Pclk = 4u,             ///< noise filter width: 32PCLK
  
}en_igbt_filter_width_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define IBGT input/out level
 ******************************************************************************/
typedef enum en_igbt_level
{
    IgbtLevelNormal = 0u,          ///< Normal
    IgbtLevelInvert = 1u,          ///< Invert
    
}en_igbt_level_t;

/**
 ******************************************************************************
 ** \brief structure of PPG IGBT configuration
 ******************************************************************************/
typedef struct stc_ppg_igbt_config
{
    en_igbt_prohibition_mode_t  enMode;              ///< prohibition mode
    en_igbt_filter_width_t      enWidth;             ///< noise filter width
    en_igbt_level_t             enTrigInputLevel;    ///< Trigger input level
    en_igbt_level_t             enIgbt0OutputLevel;  ///< IGBT0 output level (PPG0)
    en_igbt_level_t             enIgbt1OutputLevel;  ///< IGBT1 output level (PPG4) 
    
}stc_ppg_igbt_config_t;

/**
 ******************************************************************************
 ** \brief PPG IRQ channel
 ******************************************************************************/
typedef enum en_ppg_irq_ch
{
    PpgCh024    = 0u,               ///< IRQ 36 for  PPG ch.0,2,4
    PpgCh81012  = 1u,               ///< IRQ 37 for  PPG ch.8,10,12
    PpgCh161820 = 2u,               ///< IRQ 38 for  PPG ch.16,18,20
    
}en_ppg_irq_ch_t;

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
/* 1. Init/De-Init */
en_result_t Ppg_Init( uint8_t u8CoupleCh, stc_ppg_config_t *pstcConfig);
en_result_t Ppg_DeInit(uint8_t u8CoupleCh, boolean_t bTouchNvic);
/* 2. Trigger configuration */
/* software */
en_result_t Ppg_StartSoftwareTrig(uint8_t u8Ch);
en_result_t Ppg_StopSoftwareTrig(uint8_t u8Ch);
/* MFT gate signal */
en_result_t Ppg_SelGateLevel(uint8_t u8EvenCh, en_ppg_gate_level_t enLevel);
/* Timing generator configuration */
en_result_t  Ppg_ConfigUpCnt0(stc_ppg_upcnt0_config_t* pstcConfig);
void  Ppg_StartUpCnt0(void);
en_stat_flag_t  Ppg_GetUpCnt0Status(void);
en_result_t  Ppg_DisableTimerGen0StartTrig(stc_timer0_gen_ch_t* pstcTimer0GenCh);

en_result_t  Ppg_ConfigUpCnt1(stc_ppg_upcnt1_config_t* pstcConfig);
void  Ppg_StartUpCnt1(void);
en_stat_flag_t  Ppg_GetUpCnt1Status(void);
en_result_t  Ppg_DisableTimerGen1StartTrig(stc_timer1_gen_ch_t* pstcTimer1GenCh);

en_result_t  Ppg_ConfigUpCnt2(stc_ppg_upcnt2_config_t* pstcConfig);
void  Ppg_StartUpCnt2(void);
en_stat_flag_t  Ppg_GetUpCnt2Status(void);
en_result_t  Ppg_DisableTimerGen2StartTrig(stc_timer2_gen_ch_t* pstcTimer2GenCh);
/* 3. Int */
#if (PDL_INTERRUPT_ENABLE_PPG == PDL_ON) 
en_result_t  Ppg_EnableIrq(uint8_t u8CoupleCh);
en_result_t  Ppg_DisableIrq(uint8_t u8CoupleCh);
#endif
en_irq_flag_t  Ppg_GetIrqFlag(uint8_t u8CoupleCh);
en_result_t  Ppg_ClrIrqFlag(uint8_t u8CoupleCh);
/* 4. H/L width set */
en_result_t  Ppg_SetLevelWidth(uint8_t u8Ch, uint8_t u8LowWidth, uint8_t u8HighWidth);
/* 5. IGBT mode */
en_result_t  Ppg_InitIgbt(stc_ppg_igbt_config_t* pstcPpgIgbt);
void Ppg_EnableIgbtMode(void);
void Ppg_DisableIgbtMode(void);
/* 6. IRQ handler */
void Ppg_IrqHandler(void);

#ifdef __cplusplus
}
#endif

//@}

#endif

#endif
