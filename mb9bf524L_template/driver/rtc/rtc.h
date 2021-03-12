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
/** \file rtc.h
 **
 ** Headerfile for RTC functions
 **  
 **
 ** History:
 **   - 2013-12-10   EZh     First verison for FM universal PDL.
 **
 ******************************************************************************/

#ifndef __RTC_H__
#define __RTC_H__

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "mcu.h"
#include "pdl_user.h"
#include "time.h"

#if (defined(PDL_PERIPHERAL_RTC_ACTIVE))

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/**
 ******************************************************************************
 ** \defgroup RtcGroup Real Time Clock (RTC)
 **
 ** Provided RTC module functions:
 **
 ** - Rtc_Init()
 ** - Rtc_DeInit()
 ** - Rtc_EnableIrq()
 ** - Rtc_DisableIrq()
 ** - Rtc_EnableFunc()
 ** - Rtc_DisableFunc()
 ** - Rtc_Reset()
 ** - Rtc_GetIrqFlag()
 ** - Rtc_ClrIrqFlag()
 ** - Rtc_GetStatus()  
 ** - Rtc_SetDateTime()
 ** - Rtc_ReadDateTime()
 ** - Rtc_SetAlarmDateTime()
 ** - Rtc_GetAlarmDateTime()
 ** - Rtc_SetTimerCycle()
 ** - Rtc_SetFreqCorrValue()
 ** - Rtc_SetDayOfWeek()
 **
 ** Rtc_Init() initializes an RTC instance according the configuration
 ** of the structure type #stc_rtc_config_t. Rtc_DeInit() de-initializes an
 ** RTC instance. 
 **
 ** Interrupts can be configured first in Rtc_Init(), after initialization,
 ** Rtc_EnableIrq() enables RTC interrupt sources selected by enumeration type
 ** #en_rtc_irq_sel_t. Rtc_DisableIrq() disables the RTC interrupt sources 
 ** selected by enumeration type #en_rtc_irq_sel_t.
 **
 ** Rtc_EnableFunc() enables RTC function selected by the parameter 
 ** Rtc_EnableFunc#enFunc and Rtc_DisableFunc() disables RTC function selected.
 **
 ** Rtc_Reset() resets RTC peripheral as well as RTC registers.
 **
 ** Rtc_GetStatus() gets the status selected by Rtc_GetStatus#enStatus .
 ** 
 ** Rtc_GetIrqFlag() gets the interrupt flag selected by enumeration type 
 ** #en_rtc_irq_sel_t and Rtc_ClrIrqFlag() clears the interrupt flag selected 
 ** by enumeration type #en_rtc_irq_sel_t
 **
 ** Rtc_SetDateTime() can change the time and date of RTC and Rtc_GetDateTime()
 ** gets current time and date. 
 **
 ** Rtc_SetAlarmDateTime() can change the alarm time and date, and
 ** Rtc_GetAlarmDateTime() gets the alarm time and date.
 **
 ** Rtc_SetTimerCycle() sets the cycle of RTC timer, note that the unit of cycle
 ** is second.
 **
 ** Rtc_SetFreqCorrValue() sets the frequency correction value of RTC. 
 **
 ** Rtc_SetDayOfWeek() sets the day of week based on time structure supplied. 
 ** 
 ******************************************************************************/
//@{

/**
 ******************************************************************************
 ** \brief Rtccallback function prototypes.
 ******************************************************************************/

/******************************************************************************
 * Global definitions
 ******************************************************************************/  
#define stc_rtcn_t   FM_RTC_TypeDef
#define RTC0       (*((volatile stc_rtcn_t *) FM_RTC_BASE))  
  
#define RTC_INSTANCE_COUNT                                \
        (uint8_t)(PDL_PERIPHERAL_ENABLE_RTC0 == PDL_ON)
  
/******************************************************************************/
/* Define RTC types                                                           */
/******************************************************************************/
#define PDL_RTC_WITHOUT_VBAT_TYPEA         0u  ///< The type A of RTC without VBAT domain
#define PDL_RTC_WITHOUT_VBAT_TYPEB         1u  ///< The type B of RTC without VBAT domain
#define PDL_RTC_VBAT_TYPEA                 2u  ///< The type A of RTC association with VBAT
#define PDL_RTC_VBAT_TYPEB                 3u  ///< The type B of RTC association with VBAT
  
/**
 ******************************************************************************
 ** \brief Extract RTC type according to a certain device
 ******************************************************************************/  
// FM3
#if (PDL_MCU_CORE == PDL_FM3_CORE) 
  #if (PDL_MCU_TYPE == PDL_FM3_TYPE3) || \
      (PDL_MCU_TYPE == PDL_FM3_TYPE4) || \
      (PDL_MCU_TYPE == PDL_FM3_TYPE5)
    #define PDL_RTC_TYPE       PDL_RTC_WITHOUT_VBAT_TYPEA
  #else
    #define PDL_RTC_TYPE       PDL_RTC_WITHOUT_VBAT_TYPEB      
  #endif      
// FM0+
#elif (PDL_MCU_CORE == PDL_FM0P_CORE)   
  #if (PDL_MCU_TYPE == PDL_FM0P_TYPE1) || (PDL_MCU_TYPE == PDL_FM0P_TYPE3)
    #define PDL_RTC_TYPE       PDL_RTC_WITHOUT_VBAT_TYPEB
  #elif (PDL_MCU_TYPE == PDL_FM0P_TYPE2)
    #define PDL_RTC_TYPE       PDL_RTC_VBAT_TYPEB    
  #endif
// FM4
#elif (PDL_MCU_CORE == PDL_FM4_CORE) // FM4
  #if (PDL_MCU_TYPE == PDL_FM4_TYPE1) ||  (PDL_MCU_TYPE == PDL_FM4_TYPE2) || (PDL_MCU_TYPE == PDL_FM4_TYPE6)
    #define PDL_RTC_TYPE       PDL_RTC_VBAT_TYPEA
  #elif (PDL_MCU_TYPE == PDL_FM4_TYPE3) ||  (PDL_MCU_TYPE == PDL_FM4_TYPE4)
    #define PDL_RTC_TYPE       PDL_RTC_VBAT_TYPEB
  #elif (PDL_MCU_TYPE == PDL_FM4_TYPE5)
    #define PDL_RTC_TYPE       PDL_RTC_WITHOUT_VBAT_TYPEB     
  #endif
#else
#error "Device core not found!"
#endif 
        
/**
 ******************************************************************************
 ** \brief Maximum Frequency Correction Value
 ******************************************************************************/  
#if (PDL_RTC_TYPE == PDL_RTC_WITHOUT_VBAT_TYPEA)   
#define RTC_MAX_FREQ_CORR_VALUE (0x7Fu)        
#else        
#define RTC_MAX_FREQ_CORR_VALUE (0x3FFu)
#endif
/**
 ******************************************************************************
 ** \brief Maximum Frequency Correction Cycle Setting Value
 ******************************************************************************/        
#define RTC_MAX_FREQ_CORR_CYCLE_SET_VALUE (0x3Fu)

/**
 ******************************************************************************
 ** \brief Maxmimum Timer Setting Value [unit: second]
 ******************************************************************************/  
#define RTC_MAX_TIMER_SET_VALUE (0x20000u)

/******************************************************************************
 * Global type definitions
 ******************************************************************************/
#if (PDL_RTC_TYPE == PDL_RTC_WITHOUT_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_WITHOUT_VBAT_TYPEB)        
/**
 ******************************************************************************
 ** \brief Real Time Clock clock selection
 ******************************************************************************/
typedef enum en_rtc_clk_sel
{
    RtcUseSubClk  = 0u,      ///< Use sub clock as RTC source clock
    RtcuseMainClk = 1u,      ///< Use main clock as RTC source clock
  
}en_rtc_clk_sel_t;
#endif

/**
 ******************************************************************************
 ** \brief RTCCO output selection
 **
 ** If user needs to output RTCCO, RTCCO pin function should also be enabled in 
 ** in GPIO macro.
 ******************************************************************************/
typedef enum en_rtc_rtcco_sel
{
    RtccoOutput2Hz = 0u,    ///< Output 2Hz signal at RTCCO pin
    RtccoOutput1Hz = 1u,    ///< Output 1Hz signal at RTCCO pin
    
}en_rtc_rtcco_sel_t;

/**
 ******************************************************************************
 ** \brief frequency correction configuration
 ******************************************************************************/
typedef struct stc_rtc_freq_corr
{ 
    uint16_t u16FreqCorrValue;  ///< Frequency correction value
#if (PDL_RTC_TYPE != PDL_RTC_WITHOUT_VBAT_TYPEA) 
    uint16_t u16FreqCorrCycle;  ///< Frequency correction cycle
#endif    
}stc_rtc_freq_corr_t;

/**
 ******************************************************************************
 ** \brief Real Time Clock functions
 ******************************************************************************/
typedef enum en_rtc_func
{
    RtcCount         = 0u,    ///< RTC counting
    RtcTimer         = 1u,    ///< RTC timer
    RtcFreqCorr      = 2u,    ///< RTC frequency correction module
    RtcAlarmYearEn   = 3u,    ///< Year comparison enable of RTC alarm
    RtcAlarmMonthEn  = 4u,    ///< Month comparison enable of RTC alarm
    RtcAlarmDayEn    = 5u,    ///< D comparison enable of RTC alarm
    RtcAlarmHourEn   = 6u,    ///< Hour comparison enable of RTC alarm 
    RtcAlarmMinEn    = 7u,    ///< Minute comparison enable of RTC alarm 
    
}en_rtc_func_t;

/**
 ******************************************************************************
 ** \brief Day of week name definitions 
 ******************************************************************************/
typedef enum en_rtc_day_of_week
{
    RtcSunday    = 0u,     ///< Sunday
    RtcMonday    = 1u,     ///< Monday
    RtcTuesday   = 2u,     ///< Tuesday
    RtcWednesday = 3u,     ///< Wednesday
    RtcThursday  = 4u,     ///< Thursday
    RtcFriday    = 5u,     ///< Friday
    RtcSaturday  = 6u,     ///< Saturday
} en_rtc_day_of_week_t;

/**
 ******************************************************************************
 ** \brief Month name definitions (not used in driver - to be used by
 **        user appliciation)
 ******************************************************************************/
typedef enum en_rtc_month
{
    RtcJanuary   = 1u,   ///< January
    RtcFebuary   = 2u,   ///< February 
    RtcMarch     = 3u,   ///< March
    RtcApril     = 4u,   ///< April
    RtcMay       = 5u,   ///< May 
    RtcJune      = 6u,   ///< June
    RtcJuly      = 7u,   ///< July
    RtcAugust    = 8u,   ///< August
    RtcSeptember = 9u,   ///< September
    RtcOctober   = 10u,  ///< October 
    RtcNovember  = 11u,  ///< November
    RtcDecember  = 12u,  ///< December
} en_rtc_month_t;
      
/**
 ******************************************************************************
 ** \brief Real Time Clock timer mode selection
 ******************************************************************************/
typedef enum en_rtc_time_mode
{
    RtcTimerOneshot = 0u,  ///< One-shot mode
    RtcTimerPeriod  = 1u,  ///< Periodical mode
    
}en_rtc_time_mode_t;

/**
 ******************************************************************************
 ** \brief Real Time Clock timer configuration structure
 ******************************************************************************/
typedef struct stc_rtc_timer
{
    en_rtc_time_mode_t       enMode;            ///< Timer mode
    uint32_t                 u32TimerCycle;     ///< The cycle [uint: second] of timer, after these seconds elapse, timer interrpt will occur
    
}stc_rtc_timer_t;

/**
 ******************************************************************************
 ** \brief Rtc Divider Ratio Setting
 ** 
 ** Divider Ration Settings for WTC. The enumarted values do not correspond to
 ** the bit patterns of WTDIV!
 ******************************************************************************/
typedef enum en_rtc_div_ratio
{
    RtcDivRatio1     = 0x00u,    ///< RIN clock is not divided
    RtcDivRatio2     = 0x01u,    ///< RIN clock is divided by 2
    RtcDivRatio4     = 0x02u,    ///< RIN clock is divided by 4
    RtcDivRatio8     = 0x03u,    ///< RIN clock is divided by 8
    RtcDivRatio16    = 0x04u,    ///< RIN clock is divided by 16
    RtcDivRatio32    = 0x05u,    ///< RIN clock is divided by 32
    RtcDivRatio64    = 0x06u,    ///< RIN clock is divided by 64
    RtcDivRatio128   = 0x07u,    ///< RIN clock is divided by 128
    RtcDivRatio256   = 0x08u,    ///< RIN clock is divided by 256
    RtcDivRatio512   = 0x09u,    ///< RIN clock is divided by 512
    RtcDivRatio1024  = 0x0Au,    ///< RIN clock is divided by 1024
    RtcDivRatio2048  = 0x0Bu,    ///< RIN clock is divided by 2048
    RtcDivRatio4096  = 0x0Cu,    ///< RIN clock is divided by 4096
    RtcDivRatio8192  = 0x0Du,    ///< RIN clock is divided by 8192
    RtcDivRatio16384 = 0x0Eu,    ///< RIN clock is divided by 16384
    RtcDivRatio32768 = 0x0Fu,    ///< RIN clock is divided by 32768
} en_rtc_div_ratio_t ;

/**
 ******************************************************************************
 ** \brief Interrupt configuration structure
 ******************************************************************************/
typedef struct stc_rtc_irq_en
{
    boolean_t   bHalfSecondIrq;        ///< TRUE:  0.5-Second interrupt enabled
                                       ///< FALSE: 0.5-Second interrupt disabled
    boolean_t   bOneSecondIrq;         ///< TRUE:  1-Second interrupt enabled
                                       ///< FALSE: 1-Second interrupt disabled
    boolean_t   bOneMinuteIrq;         ///< TRUE:  1-Minute interrupt enabled
                                       ///< FALSE: 1-Minute interrupt disabled
    boolean_t   bOneHourIrq;           ///< TRUE:  1-Hour interrupt enabled
                                       ///< FALSE: 1-Hour interrupt disabled
    boolean_t   bTimerIrq;             ///< TRUE:  Timer interrupt enabled
                                       ///< FALSE: Timer interrupt disabled  
    boolean_t   bAlarmIrq;             ///< TRUE:  Alarm interrupt enabled
                                       ///< FALSE: Alarm interrupt disabled
    boolean_t   bTimeRewriteErrorIrq;  ///< TRUE:  Time Rewrite Error interrupt enabled
                                       ///< FALSE: Time Rewrite Error interrupt disabled
} stc_rtc_irq_en_t;

/**
 ******************************************************************************
 ** \brief Interrupt selection  enumeration
 ******************************************************************************/
typedef enum en_rtc_irq_sel
{
    RtcHalfSecondIrq = 0u,   ///< 0.5-second count-up interrupt request  
    RtcOneSecondIrq  = 1u,   ///< 1-second count-up interrupt request   
    RtcOneMinuteIrq  = 2u,   ///< 1-Minute count-up interrupt request   
    RtcOneHourIrq    = 3u,   ///< 1-hour count-up interrupt request
    RtcTimerIrq      = 4u,   ///< Timer underflow interrupt request
    RtcAlarmIrq      = 5u,   ///< Alarm interrupt request
    RtcTimeRewriteErrorIrq = 6u, ///< Rewrite error interrupt request

} en_rtc_irq_sel_t;

/**
 ******************************************************************************
 ** \brief Real Time Clock status types
 ******************************************************************************/
typedef enum en_rtc_status
{
    RtcRunStatus             = 0u,  ///< RTC counter run status
    RtcTimerStatus           = 1u,  ///< RTC timer status
    
}en_rtc_status_t;    

/**
 ******************************************************************************
 ** \brief RTC callback functions structure
 ******************************************************************************/
typedef struct stc_rtc_irq_cb
{
    func_ptr_t               pfnTimeWrtErrIrqCb; ///< Callback function pointer for Timer writing error Interrupt
    func_ptr_t               pfnAlarmIrqCb;      ///< Callback function pointer for Alarm Interrupt
    func_ptr_t               pfnTimerIrqCb;      ///< Callback function pointer for Timer Interrupt
    func_ptr_t               pfnHalfSecondIrqCb; ///< Callback function pointer for 0.5-Second Interrupt
    func_ptr_t               pfnOneSecondIrqCb;  ///< Callback function pointer for One Second Interrupt
    func_ptr_t               pfnOneMinuteIrqCb;  ///< Callback function pointer for One Minute Interrupt
    func_ptr_t               pfnOneHourIrqCb;    ///< Callback function pointer for One Hour Interrupt
    
}stc_rtc_irq_cb_t, stc_rtc_intern_data_t;

/**
 ******************************************************************************
 ** \brief Real Time Clock Date and Time structure
 ******************************************************************************/
typedef struct stc_rtc_time
{
    uint8_t  u8Second;      ///< Second (Format 0...59)
    uint8_t  u8Minute;      ///< Minutes (Format 0...59)
    uint8_t  u8Hour;        ///< Hour (Format 0...23)
    uint8_t  u8Day;	        ///< Day (Format 1...31)
    uint8_t  u8DayOfWeek;   ///< Day of the week (Format 0...6)
    uint8_t  u8Month;       ///< Month (Format 1...12)
    uint16_t u16Year;       ///< Year (Format 1...99) + 2000
} stc_rtc_time_t;
    
/**
 ******************************************************************************
 ** \brief Real Time Clock Alarm structure
 ******************************************************************************/
typedef struct stc_rtc_alarm
{
    uint8_t  u8Minute; ///< Alarm minutes (Format 1...59)
    uint8_t  u8Hour;   ///< Alarm hour (Format 0...23)  
    uint8_t  u8Day;    ///< Alarm day (Format 1...31) 
    uint8_t  u8Month;  ///< Alarm month (Foramt 1...12)
    uint16_t u16Year;  ///< Alarm year (Format 1...99) + 2000  
} stc_rtc_alarm_t;

/**
 ******************************************************************************
 ** \brief Real Time Clock configuration
 ** 
 ** The RTC configuration settings
 ******************************************************************************/
typedef struct stc_rtc_config
{
#if (PDL_RTC_TYPE == PDL_RTC_WITHOUT_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_WITHOUT_VBAT_TYPEB)    
    en_rtc_clk_sel_t         enClkSel;             ///< TRUE:  RTC clock selection 
    uint32_t                 u32ClkPrescaler;      ///< RTC source clock prescaler
#endif    
    boolean_t                bEnSuboutDivider;     ///< TRUE:  Enable Divider for Divider Ratio
                                                   ///< FALSE: Disable Divider for Divider Ratio
    en_rtc_div_ratio_t       enDividerRatio;       ///< Divider ratio for SUBOUT
    en_rtc_rtcco_sel_t       enRtccoSel;           ///< Divider of CO signal, see #en_rtc_rtcco_sel_t for details
    
    stc_rtc_freq_corr_t*     pstcFreqCorrConfig;   ///< Pointer to frequency correction configuration, if it is set to NULL, frequency correction function will not be initialized.     
    stc_rtc_time_t*          pstcTimeDate;         ///< Pointer to time configuration structure, if it is set to NULL, time and date will not be initialized.      
    stc_rtc_alarm_t*         pstcAlarm;            ///< Pointer to alarm configuration structure, if it is set to NULL, alarm will not be initialized.   
    stc_rtc_timer_t*         pstcTimer;            ///< Pointer to timer configuration structure, if it is set to NULL, timer will not be initialized. 
#if (PDL_INTERRUPT_ENABLE_RTC0 == PDL_ON)        
    stc_rtc_irq_en_t*        pstcIrqEn;            ///< Pointer to interrupt enable structure, if it is set to NULL, no interrupts will not be enabled.
    stc_rtc_irq_cb_t*        pstcIrqCb;            ///< Pointer to interrupt callback function structure, if it is set to NULL, no interrupt callback functions will not be set.
    boolean_t                bTouchNvic;           ///< TRUE: enable NVIC, FALSE: don't enable NVIC
#endif    
    boolean_t                bRunNotInit;          ///< TRUE: don't initialize RTC when RTC is running, FALSE: Initialize RTC any time  
    
} stc_rtc_config_t;

/// Enumeration to define an index for each enabled RTC instance
typedef enum en_rtc_instance_index
{
  #if (PDL_PERIPHERAL_ENABLE_RTC0 == PDL_ON)
    RtcInstanceIndexRtc0,
  #endif
    RtcInstanceIndexMax
} en_rtc_instance_index_t;

/// RTC instance data type
typedef struct stc_rtc_instance_data
{
  volatile stc_rtcn_t*  pstcInstance;  ///< pointer to registers of an instance
  stc_rtc_intern_data_t stcInternData; ///< module internal data of instance
} stc_rtc_instance_data_t;

/******************************************************************************/
/* Global variable declarations ('extern', definition in C source)            */
/******************************************************************************/
/// Look-up table for all enabled RTC instances and their internal data
extern stc_rtc_instance_data_t m_astcRtcInstanceDataLut[RTC_INSTANCE_COUNT];

/******************************************************************************/
/* Global function prototypes (definition in C source)                        */
/******************************************************************************/
#if (PDL_INTERRUPT_ENABLE_RTC0 == PDL_ON)
void Rtc_IrqHandler(volatile stc_rtcn_t* pstcRtc, stc_rtc_intern_data_t* pstcRtcInternData);
en_result_t Rtc_EnableIrq(volatile stc_rtcn_t* pstcRtc, 
                          en_rtc_irq_sel_t enIrqSel);
en_result_t Rtc_DisableIrq(volatile stc_rtcn_t* pstcRtc, 
                           en_rtc_irq_sel_t enIrqSel);
#endif

/* Init/De-Init */
en_result_t Rtc_Init(volatile stc_rtcn_t* pstcRtc, 
                     stc_rtc_config_t*  pstcConfig);
en_result_t Rtc_DeInit(volatile stc_rtcn_t* pstcRtc, 
                       boolean_t bTouchNvic);

/* Function enable/disable */
en_result_t Rtc_EnableFunc(volatile stc_rtcn_t* pstcRtc, en_rtc_func_t enFunc);
en_result_t Rtc_DisableFunc(volatile stc_rtcn_t* pstcRtc, en_rtc_func_t enFunc);

/* Rtc Reset */
en_result_t Rtc_Reset(volatile stc_rtcn_t* pstcRtc);

/* Get/Clr RTC status */
boolean_t Rtc_GetIrqFlag(volatile stc_rtcn_t* pstcRtc, en_rtc_irq_sel_t enIrqSel);
en_result_t Rtc_ClrIrqFlag(volatile stc_rtcn_t* pstcRtc, en_rtc_irq_sel_t enIrqSel);
boolean_t Rtc_GetStatus(volatile stc_rtcn_t* pstcRtc, en_rtc_status_t enStatus);

/* Calendar/Alarm set/get */
en_result_t Rtc_SetDateTime(volatile stc_rtcn_t* pstcRtc, 
                            stc_rtc_time_t* pstcTimeDate, 
                            boolean_t bContinue, 
                            boolean_t bUpdateTime,
                            boolean_t bUpdateDate);
en_result_t Rtc_ReadDateTime(volatile stc_rtcn_t* pstcRtc, 
                             stc_rtc_time_t* pstcTimeDate);
en_result_t Rtc_SetAlarmDateTime(volatile stc_rtcn_t* pstcRtc, 
                                 stc_rtc_alarm_t* pstcAlarm,
                                 boolean_t bUpdateAlarmTime,
                                 boolean_t bUpdateAlarmDate);
en_result_t Rtc_GetAlarmDateTime(volatile stc_rtcn_t* pstcRtc, 
                                 stc_rtc_alarm_t* pstcAlarm);

/* Set timer cycle */
en_result_t Rtc_SetTimerCycle(volatile stc_rtcn_t* pstcRtc, 
                              uint32_t u32TimerCycle); 

/* Set frequency correction value */
en_result_t Rtc_SetFreqCorrValue(volatile stc_rtcn_t* pstcRtc, 
                                 uint16_t u16Value); 

/* Set day of week */
en_result_t Rtc_SetDayOfWeek(stc_rtc_time_t* pstcRtcTime);

/* Get raw time from RTC time  */
time_t Rtc_GetRawTime(stc_rtc_time_t* pstcRtcTime);

/* Set RTC time from raw time  */
en_result_t Rtc_SetLocalTime(stc_rtc_time_t* pstcRtcTime,
                             time_t          tRawTime);

//@} // RtcGroup

#ifdef __cplusplus
}
#endif

#endif /* #if (defined(PDL_PERIPHERAL_RTC_ACTIVE)) */

#endif /* __RTC_H__ */
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
