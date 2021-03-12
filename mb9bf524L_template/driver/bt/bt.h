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
/** \file bt.h
 **
 ** Headerfile for BT functions
 **
 ** History:
 **   - 2014-11-13  0.1  EZh First version for FM Universal PDL.
 **
 *****************************************************************************/
#ifndef __BT_H__
#define __BT_H__

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "mcu.h"
#include "pdl_user.h"

#if (defined(PDL_PERIPHERAL_BT_ACTIVE))

/**
 ******************************************************************************
 ** \defgroup BtGroup Base Timer (BT)
 **
 ** Provided BT module functions:
 **
 ** - Bt_ConfigIOMode()
 ** - Bt_Pwm_Init()
 ** - Bt_Pwm_EnableCount()
 ** - Bt_Pwm_DisableCount()
 ** - Bt_Pwm_EnableSwTrig()
 ** - Bt_Pwm_EnableIrq()
 ** - Bt_Pwm_DisableIrq()
 ** - Bt_Pwm_GetIrqFlag()
 ** - Bt_Pwm_ClrIrqFlag()
 ** - Bt_Pwm_WriteCycleVal()
 ** - Bt_Pwm_WriteDutyVal()
 ** - Bt_Pwm_ReadCurCnt()
 ** - Bt_Ppg_Init()
 ** - Bt_Ppg_EnableCount()
 ** - Bt_Ppg_DisableCount()
 ** - Bt_Ppg_EnableSwTrig()
 ** - Bt_Ppg_EnableIrq()
 ** - Bt_Ppg_DisableIrq()
 ** - Bt_Ppg_GetIrqFlag()
 ** - Bt_Ppg_ClrIrqFlag()
 ** - Bt_Ppg_WriteLowWidthVal()
 ** - Bt_Ppg_WriteHighWidthVal()
 ** - Bt_Ppg_ReadCurCnt()
 ** - Bt_Rt_Init()
 ** - Bt_Rt_EnableCount()
 ** - Bt_Rt_DisableCount()
 ** - Bt_Rt_EnableSwTrig()
 ** - Bt_Rt_EnableIrq()
 ** - Bt_Rt_DisableIrq()
 ** - Bt_Rt_GetIrqFlag()
 ** - Bt_Rt_ClrIrqFlag()
 ** - Bt_Rt_WriteCycleVal()
 ** - Bt_Rt_ReadCurCnt()
 ** - Bt_Pwc_Init()
 ** - Bt_Pwc_EnableCount()
 ** - Bt_Pwc_DisableCount()
 ** - Bt_Pwc_EnableIrq()
 ** - Bt_Pwc_DisableIrq()
 ** - Bt_Pwc_GetIrqFlag()
 ** - Bt_Pwc_ClrIrqFlag()
 ** - Bt_Pwc_GetErrorFlag()
 ** - Bt_Pwc_Get16BitMeasureData()
 ** - Bt_Pwc_Get32BitMeasureData()
 ** - Bt_SetSimultaneousStart()
 **
 ** 
 ** <b>How to use PWM timer function of BT module?</b>
 **
 ** Bt_ConfigIOMode() must be used to configure BT I/O mode to I/O mode 0 first.
 **
 ** Bt_Pwm_Init() is used to initialize PWM timer source clock, output polarity,
 ** operation mode and so on. Following operation mode can set:
 ** - Continuous mode
 ** - One-shot mode
 **
 ** The interrupts can be configured first in Bt_Pwm_Init(), after initialization, 
 ** a PWM interrupt can be enabled by the function Bt_Pwm_EnableIrq() and a PWM 
 ** interrupt can be disabled by the function Bt_Pwm_DisableIrq()
 **
 ** With Bt_Pwm_WriteCycleVal() the PWM cycle counter is set to the value
 ** given in the parameter Bt_Pwm_WriteCycleVal#u16Cycle.
 **
 ** With Bt_Pwm_WriteDutyVal() the PWM duty counter is set to the value
 ** given in the parameter Bt_Pwm_WriteDutyVal#u16Duty.
 **
 ** Notes that PWM can be only set to 16-bit mode, so above two parameters
 ** should be 16-bit.
 **
 ** Bt_Pwm_EnableCount() is used to enabe PWM counter.
 **
 ** After above setting is done, calling Bt_Pwm_EnableSwTrig() will start
 ** PWM timer.
 **
 ** With Bt_Pwm_ReadCurCnt() the current PWM count can be read when PWM timer
 ** is running.
 **
 ** With interrupt mode, when the interrupt occurs, the interrupt flag will
 ** be cleared and run into user interrupt callback function.
 **
 ** With polling mode, user can use Bt_Pwm_GetIrqFlag() to check if the
 ** interrupt occurs, and clear the interrupt flag by Bt_Pwm_ClrIrqFlag().
 **
 ** When stopping the PWM timer, use Bt_Pwm_DisableCount() to disable PWM
 ** counter.
 **
 ** <b>How to use PPG timer function of BT module?</b>
 **
 ** Bt_ConfigIOMode() must be used to configure BT I/O mode to I/O mode 0 first.
 **
 ** Bt_Ppg_Init() is used to initialize PPG timer source clock, output polarity,
 ** operation mode and so on. Following operation mode can set:
 ** - Continuous mode
 ** - One-shot mode
 **
 ** The interrupts can be configured first in Bt_Ppg_Init(), after initialization, 
 ** a PPG interrupt can be enabled by the function Bt_Ppg_EnableIrq().
 ** This function can set callback function for each channel too.
 **
 ** With Bt_Ppg_WriteLowWidthVal() the PPG low width is set to the value
 ** given in the parameter Bt_Ppg_WriteLowWidthVal#u16Cycle.
 **
 ** With Bt_Ppg_WriteHighWidthVal() the PPG high width is set to the value
 ** given in the parameter Bt_Ppg_WriteHighWidthVal#u16Cycle.
 **
 ** Notes that PPG can be only set to 16-bit mode, so above two parameters
 ** should be 16-bit.
 **
 ** Bt_Ppg_EnableCount() is used to enabe PPG counter.
 **
 ** After above setting is done, calling Bt_Ppg_EnableSwTrig() will start
 ** PPG timer.
 **
 ** With Bt_Ppg_ReadCurCnt() the current PPG count can be read when PPG timer
 ** is running.
 **
 ** With interrupt mode, when the interrupt occurs, the interrupt flag will
 ** be cleared and run into user interrupt callback function.
 **
 ** With polling mode, user can use Bt_Ppg_GetIrqFlag() to check if the
 ** interrupt occurs, and clear the interrupt flag by Bt_Ppg_ClrIrqFlag().
 **
 ** When stopping the PPG timer, use Bt_Ppg_DisableCount() to disable PPG
 ** counter and Bt_Ppg_DisableIrq() to disable PPG interrupt.
 **
 ** <b>How to use Reload Timer (RT) function of BT module?</b>
 **
 ** Bt_ConfigIOMode() must be used to configure BT I/O mode to I/O mode 0 first.
 **
 ** Bt_Rt_Init() is used to initialize RT source clock, output polarity,
 ** operation mode and so on. Following operation mode can set:
 ** - Reload mode
 ** - One-shot mode
 **
 ** The interrupts can be configured first in Bt_Rt_Init(), after initialization, 
 ** a RT interrupt can be enabled by the function Bt_Rt_EnableIrq().
 ** This function can set callback function for each channel too.
 **
 ** RT can be set to 16-bit mode or 32-bit mode. In 16-bit mode,
 ** Bt_Rt_WriteCycleVal() set the RT counter cycle of according channel.
 ** In the 32-bit mode, Bt_Rt_WriteCycleVal() with even channel register
 ** pointer as paremater set the low 16-bit cycle value and
 ** Bt_Rt_WriteCycleVal() with odd channel register pointer as paremater
 ** set the high 16-bit cycle value.
 **
 ** Bt_Rt_EnableCount() is used to enabe RT counter.
 **
 ** After above setting is done, calling Bt_Rt_EnableSwTrig() will start
 ** Reload Timer.
 **
 ** With Bt_Rt_ReadCurCnt() the current RT count can be read when Reload Timer
 ** is running.
 **
 ** With interrupt mode, when the interrupt occurs, the interrupt flag will
 ** be cleared and run into user interrupt callback function.
 **
 ** With polling mode, user can use Bt_Rt_GetIrqFlag() to check if the
 ** interrupt occurs, and clear the interrupt flag by Rt_Ppg_ClrIrqFlag().
 **
 ** When stopping the PPG timer, use Bt_Rt_DisableCount() to disable RT
 ** counter and Bt_Rt_DisableIrq() to disable RT interrupt.
 **
 ** <b>How to use PWC timer function of BT module?</b>
 **
 ** Bt_ConfigIOMode() must be used to configure BT I/O mode to I/O mode 0 first.
 **
 ** Bt_Pwc_Init() is used to initialize PWC timer source clock, measurement mode
 ** and so on.
 **
 ** Following measurement mode can set:
 ** - High pulse width measurement
 ** - Cycle measurement with rising edges
 ** - Cycle measurement with falling edges
 ** - Interval measurement between all edges
 ** - Low pulse width measurement
 **
 ** Following operation mode can be set:
 ** - Continuous mode
 ** - One-shot mode
 **
 ** The interrupts can be configured first in Bt_Pwc_Init(), after initialization, 
 ** a PWC interrupt can be enabled by the function Bt_Pwc_EnableIrq().
 ** This function can set callback function for each channel too.
 **
 ** Bt_Pwc_EnableCount() is used to enabe PWC counter.
 **
 ** After above setting is done, when the valid edge (1st) is detected, the
 ** measurement starts, and the valid edge (2nd) is detected, the measurement
 ** ends, the interrupt request flag is set at the same time.
 **
 ** PWC timer can be set to 16-bit mode and 32-bit mode. In the 16-bit mode,
 ** with Bt_Pwc_Get16BitMeasureData() the measured value can be read after
 ** measurement completes, in the 32-bit mode, with Bt_Pwc_Get32BitMeasureData()
 ** the measured value can be read.
 **
 ** With interrupt mode, when the interrupt occurs, the interrupt flag will
 ** be cleared and the code runs into user interrupt callback function.
 **
 ** With polling mode, user can use Bt_Pwc_GetIrqFlag() to check if the
 ** completion interrupt occurs, and clear the interrupt flag by
 ** Bt_Pwc_ClrIrqFlag(). Bt_Pwc_GetErrorFlag() is used to get the error flag
 ** of PWC measurement.
 **
 ** When stopping the PWC timer, use Bt_Pwc_DisableCount() to disable PWC
 ** counter and Bt_Pwc_DisableIrq() to disable PWC interrupt.
 **
 ******************************************************************************/

//@{

/******************************************************************************
 * Global definitions
 ******************************************************************************/
#define stc_btn_t  FM_BT_PWM_TypeDef

#define BT0       (*((volatile stc_btn_t *) FM_BT0_PWM_BASE))
#define BT1       (*((volatile stc_btn_t *) FM_BT1_PWM_BASE))
#define BT2       (*((volatile stc_btn_t *) FM_BT2_PWM_BASE))
#define BT3       (*((volatile stc_btn_t *) FM_BT3_PWM_BASE))
#define BT4       (*((volatile stc_btn_t *) FM_BT4_PWM_BASE))
#define BT5       (*((volatile stc_btn_t *) FM_BT5_PWM_BASE))
#define BT6       (*((volatile stc_btn_t *) FM_BT6_PWM_BASE))
#define BT7       (*((volatile stc_btn_t *) FM_BT7_PWM_BASE))
#if (PDL_MCU_CORE == PDL_FM3_CORE) || (PDL_MCU_CORE == PDL_FM4_CORE)
#define BT8       (*((volatile stc_btn_t *) FM_BT8_PWM_BASE))
#define BT9       (*((volatile stc_btn_t *) FM_BT9_PWM_BASE))
#define BT10      (*((volatile stc_btn_t *) FM_BT10_PWM_BASE))
#define BT11      (*((volatile stc_btn_t *) FM_BT11_PWM_BASE))
#define BT12      (*((volatile stc_btn_t *) FM_BT12_PWM_BASE))
#define BT13      (*((volatile stc_btn_t *) FM_BT13_PWM_BASE))
#define BT14      (*((volatile stc_btn_t *) FM_BT14_PWM_BASE))
#define BT15      (*((volatile stc_btn_t *) FM_BT15_PWM_BASE))
#endif

#define BT_INSTANCE_COUNT      ((PDL_PERIPHERAL_ENABLE_BT0 == PDL_ON) ? 1u : 0u)  + \
                                ((PDL_PERIPHERAL_ENABLE_BT1 == PDL_ON) ? 1u : 0u) + \
                                ((PDL_PERIPHERAL_ENABLE_BT2 == PDL_ON) ? 1u : 0u) + \
                                ((PDL_PERIPHERAL_ENABLE_BT3 == PDL_ON) ? 1u : 0u) + \
                                ((PDL_PERIPHERAL_ENABLE_BT4 == PDL_ON) ? 1u : 0u) + \
                                ((PDL_PERIPHERAL_ENABLE_BT5 == PDL_ON) ? 1u : 0u) + \
                                ((PDL_PERIPHERAL_ENABLE_BT6 == PDL_ON) ? 1u : 0u) + \
                                ((PDL_PERIPHERAL_ENABLE_BT7 == PDL_ON) ? 1u : 0u) + \
                                ((PDL_PERIPHERAL_ENABLE_BT8 == PDL_ON) ? 1u : 0u) + \
                                ((PDL_PERIPHERAL_ENABLE_BT9 == PDL_ON) ? 1u : 0u)  + \
                                ((PDL_PERIPHERAL_ENABLE_BT10 == PDL_ON) ? 1u : 0u) + \
                                ((PDL_PERIPHERAL_ENABLE_BT11 == PDL_ON) ? 1u : 0u) + \
                                ((PDL_PERIPHERAL_ENABLE_BT12 == PDL_ON) ? 1u : 0u) + \
                                ((PDL_PERIPHERAL_ENABLE_BT13 == PDL_ON) ? 1u : 0u) + \
                                ((PDL_PERIPHERAL_ENABLE_BT14 == PDL_ON) ? 1u : 0u) + \
                                ((PDL_PERIPHERAL_ENABLE_BT15 == PDL_ON) ? 1u : 0u)                               
                                                            

#if (PDL_INTERRUPT_ENABLE_BT0 == PDL_ON) ||  (PDL_INTERRUPT_ENABLE_BT1 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_BT2 == PDL_ON) ||  (PDL_INTERRUPT_ENABLE_BT3 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_BT4 == PDL_ON) ||  (PDL_INTERRUPT_ENABLE_BT5 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_BT6 == PDL_ON) ||  (PDL_INTERRUPT_ENABLE_BT7 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_BT8 == PDL_ON) ||  (PDL_INTERRUPT_ENABLE_BT9 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_BT10 == PDL_ON) ||  (PDL_INTERRUPT_ENABLE_BT11 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_BT12 == PDL_ON) ||  (PDL_INTERRUPT_ENABLE_BT13 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_BT14 == PDL_ON) ||  (PDL_INTERRUPT_ENABLE_BT15 == PDL_ON) 
    #define PDL_INTERRUPT_BT_ACTIVE    
#endif
       
                             
/******************************************************************************
 * Global type definitions
 ******************************************************************************/
/**
 ******************************************************************************
 ** \brief Enumeration to define an index for each BT instance
 ******************************************************************************/
typedef enum en_bt_instance_index
{
#if (PDL_PERIPHERAL_ENABLE_BT0 == PDL_ON)
    BtInstanceIndexBt0,      ///< Instance index of BT0
#endif
#if (PDL_PERIPHERAL_ENABLE_BT1 == PDL_ON)
    BtInstanceIndexBt1,      ///< Instance index of BT1
#endif
#if (PDL_PERIPHERAL_ENABLE_BT2 == PDL_ON)
    BtInstanceIndexBt2,      ///< Instance index of BT2
#endif
#if (PDL_PERIPHERAL_ENABLE_BT3 == PDL_ON)
    BtInstanceIndexBt3,      ///< Instance index of BT3
#endif
#if (PDL_PERIPHERAL_ENABLE_BT4 == PDL_ON)
    BtInstanceIndexBt4,      ///< Instance index of BT4
#endif
#if (PDL_PERIPHERAL_ENABLE_BT5 == PDL_ON)
    BtInstanceIndexBt5,      ///< Instance index of BT5
#endif
#if (PDL_PERIPHERAL_ENABLE_BT6 == PDL_ON)
    BtInstanceIndexBt6,      ///< Instance index of BT6
#endif
#if (PDL_PERIPHERAL_ENABLE_BT7 == PDL_ON)
    BtInstanceIndexBt7,      ///< Instance index of BT7
#endif
#if (PDL_PERIPHERAL_ENABLE_BT8 == PDL_ON)
    BtInstanceIndexBt8,      ///< Instance index of BT8
#endif
#if (PDL_PERIPHERAL_ENABLE_BT9 == PDL_ON)
    BtInstanceIndexBt9,      ///< Instance index of BT9
#endif
#if (PDL_PERIPHERAL_ENABLE_BT10 == PDL_ON)
    BtInstanceIndexBt10,      ///< Instance index of BT10
#endif
#if (PDL_PERIPHERAL_ENABLE_BT11 == PDL_ON)
    BtInstanceIndexBt11,      ///< Instance index of BT11
#endif
#if (PDL_PERIPHERAL_ENABLE_BT12 == PDL_ON)
    BtInstanceIndexBt12,      ///< Instance index of BT12
#endif
#if (PDL_PERIPHERAL_ENABLE_BT13 == PDL_ON)
    BtInstanceIndexBt13,      ///< Instance index of BT13
#endif
#if (PDL_PERIPHERAL_ENABLE_BT14 == PDL_ON)
    BtInstanceIndexBt14,      ///< Instance index of BT14
#endif
#if (PDL_PERIPHERAL_ENABLE_BT15 == PDL_ON)
    BtInstanceIndexBt15,      ///< Instance index of BT15
#endif
    BtInstanceIndexMax       ///< Maximum instance index
} en_bt_instance_index_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define BT IO mode
 ******************************************************************************/
typedef enum en_bt_io_mode
{
    BtIoMode0 = 0u,               ///< BT IO mode 0
    BtIoMode1 = 1u,               ///< BT IO mode 1
    BtIoMode2 = 2u,               ///< BT IO mode 2
    BtIoMode3 = 3u,               ///< BT IO mode 3
    BtIoMode4 = 4u,               ///< BT IO mode 4
    BtIoMode5 = 5u,               ///< BT IO mode 5
    BtIoMode6 = 6u,               ///< BT IO mode 6
    BtIoMode7 = 7u,               ///< BT IO mode 7
    BtIoMode8 = 8u,               ///< BT IO mode 8

}en_bt_io_mode_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define timer mode of BT
 ******************************************************************************/
typedef enum en_bt_timer_mode
{
    BtResetMode = 0u,              ///< Reset mode
    BtPwmMode   = 1u,              ///< PWM timer mode
    BtPpgMode   = 2u,              ///< PPG timer mode
    BtRtMode    = 3u,              ///< Reload timer mode
    BtPwcMode   = 4u,              ///< PWC timer mode

}en_bt_timer_mode_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define clock prescaler of PWM timer
 ******************************************************************************/
typedef enum en_pwm_clock_pres
{
    PwmPresNone            = 0u,       ///< PWM prescaler clock: no division
    PwmPres1Div4           = 1u,       ///< PWM prescaler clock: 1/4
    PwmPres1Div16          = 2u,       ///< PWM prescaler clock: 1/16
    PwmPres1Div128         = 3u,       ///< PWM prescaler clock: 1/128
    PwmPres1Div256         = 4u,       ///< PWM prescaler clock: 1/256
    PwmPres1ExtClkRising   = 5u,       ///< Use external clock and count at rising edge
    PwmPres1ExtClkFalling  = 6u,       ///< Use external clock and count at falling edge
    PwmPres1ExtClkBoth     = 7u,       ///< Use external clock and count at both edge
    PwmPres1Div512         = 8u,       ///< PWM prescaler clock: 1/512
    PwmPres1Div1024        = 9u,       ///< PWM prescaler clock: 1/1024
    PwmPres1Div2048        = 10u,      ///< PWM prescaler clock: 1/2048

}en_pwm_clock_pres_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define the configuration of PWM restart option
 ******************************************************************************/
typedef enum en_pwm_restart_enable
{
    PwmRestartDisable = 0u,       ///< Disable PWM restart
    PwmRestartEnable  = 1u,       ///< Enable PWM restart

}en_pwm_restart_enable_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define the configuration of PWM output mask
 ******************************************************************************/
typedef enum en_pwm_output_mask
{
    PwmOutputNormal = 0u,         ///< Output normal PWM wave
    PwmOutputMask   = 1u,         ///< Mask PWM wave

}en_pwm_output_mask_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define external trigger of PWM
 ******************************************************************************/
typedef enum en_pwm_ext_trig
{
    PwmExtTrigDisable  = 0u,      ///< Disable external trigger
    PwmExtTrigRising   = 1u,      ///< Enable external trigger with rising edge
    PwmExtTrigFalling  = 2u,      ///< Enable external trigger with falling edge
    PwmExtTrigBoth     = 3u,      ///< Enable external trigger with both edge

}en_pwm_ext_trig_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define PWM initial polarity
 ******************************************************************************/
typedef enum en_pwm_output_polarity
{
    PwmPolarityLow    = 0u,      ///< Initial polarity: low
    PwmPolarityHigh   = 1u,      ///< Initial polarity: high

}en_pwm_output_polarity_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define PWM timer mode
 ******************************************************************************/
typedef enum en_pwm_mode
{
    PwmContinuous = 0u,         ///< Continuous mode
    PwmOneshot    = 1u,         ///< One-shot mode

}en_pwm_mode_t;

/**
 ******************************************************************************
 ** \brief structure to enable the PWM interrupts
 ******************************************************************************/
typedef struct stc_pwm_irq_en
{
    boolean_t bPwmTrigIrq;        ///< Trigger interrupt selection
    boolean_t bPwmDutyMatchIrq;   ///< Duty match interrupt selection
    boolean_t bPwmUnderflowIrq;   ///< Underflow interrupt selection

} stc_pwm_irq_en_t;

/**
 ******************************************************************************
 ** \brief enumeration to select the PWM interrupt
 ******************************************************************************/
typedef enum en_pwm_irq_sel
{
    PwmTrigIrq          = 0u,   ///< PWM trigger interrupt
    PwmDutyMatchIrq     = 1u,   ///< PWM duty match interrupt
    PwmUnderflowIrq     = 2u,   ///< PWM underflow interrupt
    
} en_pwm_irq_sel_t;

/**
 ******************************************************************************
 ** \brief structure to set the PWM interrupt callback function
 ******************************************************************************/
typedef struct stc_pwm_int_cb
{
    func_ptr_t pfnPwmTrigIrqCb;      ///< Pointer to trigger interrupt callback function
    func_ptr_t pfnPwmDutyMatchIrqCb; ///< Pointer to duty match interrupt callback function
    func_ptr_t pfnPwmUnderflowIrqCb; ///< Pointer to underflow interrupt callback function

}stc_pwm_irq_cb_t;

/**
 ******************************************************************************
 ** \brief Structure of PWM configuration
 ******************************************************************************/
typedef struct stc_bt_pwm_config
{
    en_pwm_clock_pres_t         enPres;                ///< Clock prescaler
    en_pwm_restart_enable_t     enRestartEn;           ///< Restart enable setting
    en_pwm_output_mask_t        enOutputMask;          ///< PWM output mask setting
    en_pwm_ext_trig_t           enExtTrig;             ///< PWM external trigger setting
    en_pwm_output_polarity_t    enOutputPolarity;      ///< PWM output polarity setting
    en_pwm_mode_t               enMode;                ///< Continuous mode or one-shot mode
#if defined(PDL_INTERRUPT_BT_ACTIVE)    
    stc_pwm_irq_en_t*           pstcPwmIrqEn;          ///< Pointer to interrupt request enable setting structure of Base Timer with PWM mode
    stc_pwm_irq_cb_t*           pstcPwmIrqCb;          ///< Pointer to interrupt request callback functions structure of Base Timer with PWM mode
    boolean_t                   bTouchNvic;            ///< TRUE: enable NVIC, FALSE: don't enable NVIC
#endif
    
}stc_bt_pwm_config_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define clock prescaler of PPG timer
 ******************************************************************************/
typedef enum en_ppg_clock_pres
{
    PpgPresNone              = 0u,               ///< PPG prescaler clock: no division
    PpgPres1Div4             = 1u,               ///< PPG prescaler clock: 1/4
    PpgPres1Div16            = 2u,               ///< PPG prescaler clock: 1/16
    PpgPres1Div128           = 3u,               ///< PPG prescaler clock: 1/128
    PpgPres1Div256           = 4u,               ///< PPG prescaler clock: 1/256
    PpgPres1ExtClkRising     = 5u,               ///< Use external clock and count at rising edge
    PpgPres1ExtClkFalling    = 6u,               ///< Use external clock and count at falling edge
    PpgPres1ExtClkBoth       = 7u,               ///< Use external clock and count at both edge
    PpgPres1Div512           = 8u,               ///< PPG prescaler clock: 1/512
    PpgPres1Div1024          = 9u,               ///< PPG prescaler clock: 1/1024
    PpgPres1Div2048          = 10u,              ///< PPG prescaler clock: 1/2048

}en_ppg_clock_pres_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define the configuration of PPG restart option
 ******************************************************************************/
typedef enum en_ppg_restart_enable
{
    PpgRestartDisable = 0u,         ///< Disable PPG restart
    PpgRestartEnable  = 1u,         ///< Enable PPG restart

}en_ppg_restart_enable_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define the configuration of PPG output mask
 ******************************************************************************/
typedef enum en_ppg_output_mask
{
    PpgOutputNormal = 0u,           ///< Output normal PPG wave
    PpgOutputMask   = 1u,           ///< Mask PPG wave

}en_ppg_output_mask_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define external trigger of PPG
 ******************************************************************************/
typedef enum en_ppg_ext_trig
{
    PpgExtTrigDisable   = 0u,         ///< Disable external trigger
    PpgExtTrigRising    = 1u,         ///< Enable external trigger with rising edge
    PpgExtTrigFalling   = 2u,         ///< Enable external trigger with falling edge
    PpgExtTrigBoth      = 3u,         ///< Enable external trigger with both edge

}en_ppg_ext_trig_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define PWM initial polarity
 ******************************************************************************/
typedef enum en_ppg_output_polarity
{
    PpgPolarityLow   = 0u,              ///< Initial polarity: low
    PpgPolarityHigh  = 1u,              ///< Initial polarity: high

}en_ppg_output_polarity_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define PWM timer mode
 ******************************************************************************/
typedef enum en_ppg_mode
{
    PpgContinuous = 0u,                 ///< Continuous mode
    PpgOneshot    = 1u,                 ///< One-shot mode

}en_ppg_mode_t;

/**
 ******************************************************************************
 ** \brief structure to enable the PPG interrupts
 ******************************************************************************/
typedef struct stc_ppg_irq_en
{
    boolean_t bPpgTrigIrq;       ///< PPG trigger interrupt selection
    boolean_t bPpgUnderflowIrq;  ///< PPG underflow interrupt selection

} stc_ppg_irq_en_t;

/**
 ******************************************************************************
 ** \brief enumeration to select the PPG interrupt
 ******************************************************************************/
typedef enum en_ppg_irq_sel
{
    PpgTrigIrq      = 0u,     ///< PPG trigger interrupt 
    PpgUnderflowIrq = 1u,     ///< PPG underflow interrupt
    
} en_ppg_irq_sel_t;

/**
 ******************************************************************************
 ** \brief structure to set the PPG interrupt callback function
 ******************************************************************************/
typedef struct stc_ppg_int_cb
{
    func_ptr_t pfnPpgTrigIrqCb;      ///< Pointer to PPG trigger interrupt callback function
    func_ptr_t pfnPpgUnderflowIrqCb; ///< Pointer to PPG underflow interrupt callback function

}stc_ppg_irq_cb_t;

/**
 ******************************************************************************
 ** \brief Structure of PPG configuration
 ******************************************************************************/
typedef struct stc_bt_ppg_config
{
    en_ppg_clock_pres_t         enPres;             ///< Clock prescaler
    en_ppg_restart_enable_t     enRestartEn;        ///< Restart enable setting
    en_ppg_output_mask_t        enOutputMask;       ///< PPG output mask setting
    en_ppg_ext_trig_t           enExtTrig;          ///< PPG external trigger setting
    en_ppg_output_polarity_t    enOutputPolarity;   ///< PPG output polarity setting
    en_ppg_mode_t               enMode;             ///< Continuous mode or one-shot mode
#if defined(PDL_INTERRUPT_BT_ACTIVE)     
    stc_ppg_irq_en_t*           pstcPpgIrqEn;       ///< Pointer to interrupt request enable setting structure of Base Timer with PPG mode
    stc_ppg_irq_cb_t*           pstcPpgIrqCb;       ///< Pointer to interrupt request callback functions structure of Base Timer with PPG mode
    boolean_t                   bTouchNvic;         ///< TRUE: enable NVIC, FALSE: don't enable NVIC
#endif
}stc_bt_ppg_config_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define clock prescaler of Reload timer
 ******************************************************************************/
typedef enum en_rt_clock_pres
{
    RtPresNone              = 0u,           ///< Reload timer prescaler clock: no division
    RtPres1Div4             = 1u,           ///< Reload timer prescaler clock: 1/4
    RtPres1Div16            = 2u,           ///< Reload timer prescaler clock: 1/16
    RtPres1Div128           = 3u,           ///< Reload timer prescaler clock: 1/128
    RtPres1Div256           = 4u,           ///< Reload timer prescaler clock: 1/256
    RtPres1ExtClkRising     = 5u,           ///< Use external clock and count at rising edge
    RtPres1ExtClkFalling    = 6u,           ///< Use external clock and count at falling edge
    RtPres1ExtClkBoth       = 7u,           ///< Use external clock and count at both edge
    RtPres1Div512           = 8u,           ///< Reload timer prescaler clock: 1/512
    RtPres1Div1024          = 9u,           ///< Reload timer prescaler clock: 1/1024
    RtPres1Div2048          = 10u,          ///< Reload timer prescaler clock: 1/2048

}en_rt_clock_pres_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define external trigger of Reload timer
 ******************************************************************************/
typedef enum en_rt_ext_trigger
{
    RtExtTiggerDisable      = 0u,       ///< Disable external trigger
    RtExtTiggerRisingEdge   = 1u,       ///< Enable external trigger with rising edge
    RtExtTiggerFallingEdge  = 2u,       ///< Enable external trigger with falling edge
#if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE)      
    RtExtTiggerBothEdge     = 3u,       ///< Enable external trigger with both edge
    RtExtTiggerLowLevel     = 4u,       ///< Enable external trigger with low level
    RtExtTiggerHighLevel    = 5u,       ///< Enable external trigger with high level
#endif
}en_rt_ext_trigger_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define Reload timer output initial polarity
 ******************************************************************************/
typedef enum en_rt_output_polarity
{
    RtPolarityLow   = 0u,           ///< Initial polarity: low
    RtPolarityHigh  = 1u,           ///< Initial polarity: high

}en_rt_output_polarity_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define Reload timer mode
 ******************************************************************************/
typedef enum en_rt_mode
{
    RtReload    = 0u,               ///< Reload mode
    RtOneshot   = 1u,               ///< One-shot mode

}en_rt_mode_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define Reload timer size
 ******************************************************************************/
typedef enum en_rt_timer_size
{
    RtSize16Bit = 0u,             ///< 16-bit mode
    RtSize32Bit = 1u,             ///< 32-bit mode

}en_rt_timer_size_t;

/**
 ******************************************************************************
 ** \brief structure to enable the Reload timer interrupts
 ******************************************************************************/
typedef struct stc_rt_irq_en
{
    boolean_t bRtTrigIrq;        ///< Trigger interrupt selection
    boolean_t bRtUnderflowIrq;   ///< Underflow interrupt selection

} stc_rt_irq_en_t;

/**
 ******************************************************************************
 ** \brief enumeration to select the Reload timer interrupt
 ******************************************************************************/
typedef enum en_rt_irq_sel
{
    RtTrigIrq      = 0u,   ///< Reload Timer trigger interrupt 
    RtUnderflowIrq = 1u,   ///< Reload Timer underflow interrupt
    
} en_rt_irq_sel_t;

/**
 ******************************************************************************
 ** \brief structure to set the Reload timer interrupt callback function
 ******************************************************************************/
typedef struct stc_rt_irq_cb
{
    func_ptr_t pfnRtTrigIrqCb;      ///< Pointer to trigger interrupt callback function
    func_ptr_t pfnRtUnderflowIrqCb; ///< Pointer to underflow interrupt callback function

}stc_rt_irq_cb_t;

/**
 ******************************************************************************
 ** \brief Structure of Reload timer configuration
 ******************************************************************************/
typedef struct stc_bt_rt_config
{
    en_rt_clock_pres_t          enPres;                ///< Clock prescaler
    en_rt_timer_size_t          enSize;                ///< 16-bit or 32-bit
    en_rt_ext_trigger_t         enExtTrig;             ///< External trigger setting
    en_rt_output_polarity_t     enOutputPolarity;      ///< Output polarity setting
    en_rt_mode_t                enMode;                ///< Reload mode or one-shot mode
#if defined(PDL_INTERRUPT_BT_ACTIVE)      
    stc_rt_irq_en_t*            pstcRtIrqEn;        ///< Pointer to interrupt request enable setting structure of Base Timer with Reload TImer mode
    stc_rt_irq_cb_t*            pstcRtIrqCb;        ///< Pointer to interrupt request callback functions structure of Base Timer with Reload TImer mode
    boolean_t                   bTouchNvic;         ///< TRUE: enable NVIC, FALSE: don't enable NVIC
#endif
}stc_bt_rt_config_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define clock prescaler of PWC timer
 ******************************************************************************/
typedef enum en_pwc_clock_pres
{
    PwcPresNone     = 0u,        ///< PWC timer prescaler clock: no division
    PwcPres1Div4    = 1u,        ///< PWC timer prescaler clock: 1/4
    PwcPres1Div16   = 2u,        ///< PWC timer prescaler clock: 1/16
    PwcPres1Div128  = 3u,        ///< PWC timer prescaler clock: 1/128
    PwcPres1Div256  = 4u,        ///< PWC timer prescaler clock: 1/256
    PwcPres1Div512  = 8u,        ///< PWC timer prescaler clock: 1/512
    PwcPres1Div1024 = 9u,        ///< PWC timer prescaler clock: 1/1024
    PwcPres1Div2048 = 10u,       ///< PWC timer prescaler clock: 1/2048

}en_pwc_clock_pres_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define measure mode of PWC timer
 ******************************************************************************/
typedef enum en_pwc_measure_edge
{
    PwcMeasureRisingToFalling   = 0u,       ///< Measure between rising edge with falling edge
    PwcMeasureRisingToRising    = 1u,       ///< Measure between rising edge with rising edge
    PwcMeasureFallingToFalling  = 2u,       ///< Measure between falling edge with falling edge
    PwcMeasureEitherToEither    = 3u,       ///< Measure between either edge with either edge
    PwcMeasureFallingToRising   = 4u,       ///< Measure between falling edge with falling edge

}en_pwc_measure_edge_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define PWC timer mode
 ******************************************************************************/
typedef enum en_pwc_mode
{
    PwcContinuous = 0u,                 ///< Continuous mode
    PwcOneshot    = 1u,                 ///< One-shot mode

}en_pwc_mode_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define Reload timer size
 ******************************************************************************/
typedef enum en_pwc_timer_size
{
    PwcSize16Bit = 0u,               ///< 16-bit mode
    PwcSize32Bit = 1u,               ///< 32-bit mode

}en_pwc_timer_size_t;

/**
 ******************************************************************************
 ** \brief structure to enable the PWC timer interrupts
 ******************************************************************************/
typedef struct stc_pwc_irq_en
{
    boolean_t bPwcMeasureCompleteIrq;        ///< PWC measure completion IRQ
    boolean_t bPwcMeasureOverflowIrq;        ///< PWC measure overflow IRQ

}stc_pwc_irq_en_t;

/**
 ******************************************************************************
 ** \brief structure to select the PWC timer interrupt
 ******************************************************************************/
typedef enum en_pwc_irq_sel
{
    PwcMeasureCompleteIrq = 0u,    ///< PWC measure completion IRQ
    PwcMeasureOverflowIrq = 1u,    ///< PWC measure overflow IRQ
    
} en_pwc_irq_sel_t;

/**
 ******************************************************************************
 ** \brief structure to set the PWC timer interrupt callback function
 ******************************************************************************/
typedef struct stc_pwc_int_cb
{
    func_ptr_t pfnPwcMeasureCompleteIrqCb;      ///< Poiter to PWC measure completion callback function
    func_ptr_t pfnPwcMeasureOverflowIrqCb;      ///< Poiter to PWC measure overflow callback function
}stc_pwc_irq_cb_t;
 
/**
 ******************************************************************************
 ** \brief Structure of PWC timer configuration
 ******************************************************************************/
typedef struct stc_bt_pwc_config
{
    en_pwc_clock_pres_t          enPres;         ///< Clock prescaler
    en_pwc_timer_size_t          enSize;         ///< 16-bit or 32-bit
    en_pwc_measure_edge_t        enMeasureEdge;  ///< Measure mode setting
    en_pwc_mode_t                enMode;         ///< Continuous mode or one-shot mode
#if defined(PDL_INTERRUPT_BT_ACTIVE)     
    stc_pwc_irq_en_t*            pstcPwcIrqEn;       ///< Pointer to interrupt request enable setting structure of Base Timer with PWC mode
    stc_pwc_irq_cb_t*            pstcPwcIrqCb;       ///< Pointer to interrupt request callback functions structure of Base Timer with PWC mode
    boolean_t                    bTouchNvic;         ///< TRUE: enable NVIC, FALSE: don't enable NVIC
#endif    
}stc_bt_pwc_config_t;

/**
 ******************************************************************************
 ** \brief BT internal data
 ******************************************************************************/
typedef struct stc_bt_intern_data
{
    func_ptr_t  pfnBit0IrqCb;  ///< Pointer to callback function the interrupt reflected by bit0 of STC
    func_ptr_t  pfnBit1IrqCb;  ///< Pointer to callback function the interrupt reflected by bit1 of STC
    func_ptr_t  pfnBit2IrqCb;  ///< Pointer to callback function the interrupt reflected by bit2 of STC
} stc_bt_intern_data_t ;

/**
 ******************************************************************************
 ** \brief BT instance data
 ******************************************************************************/
typedef struct stc_bt_instance_data
{
    volatile stc_btn_t* pstcInstance;   ///< pointer to registers of an instance
    stc_bt_intern_data_t stcInternData; ///< module internal data of instance
} stc_bt_instance_data_t;

/******************************************************************************/
/* Global variable declarations ('extern', definition in C source)            */
/******************************************************************************/
/// Look-up table for all enabled BT instances and their internal data
extern stc_bt_instance_data_t m_astcBtInstanceDataLut[BT_INSTANCE_COUNT];

/******************************************************************************/
/* Global function prototypes (definition in C source)                        */
/******************************************************************************/
/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
/* 1. Configure BT IO mode  */
en_result_t Bt_ConfigIOMode(volatile stc_btn_t* pstcBt, en_bt_io_mode_t enIoMode);

#if (PDL_PERIPHERAL_ENABLE_BT_PWM_MODE == PDL_ON)
/* 2. Timer configuration */
/* 2.1 PWM timer */
/* Init */
en_result_t Bt_Pwm_Init(volatile stc_btn_t* pstcBt, stc_bt_pwm_config_t* pstcPwmConfig);
en_result_t Bt_Pwm_DeInit(volatile stc_btn_t* pstcBt, boolean_t bTouchNvic);
/* Func/Irq Enable/Disable */
en_result_t Bt_Pwm_EnableCount(volatile stc_btn_t* pstcBt);
en_result_t Bt_Pwm_DisableCount(volatile stc_btn_t* pstcBt);
en_result_t Bt_Pwm_EnableSwTrig(volatile stc_btn_t* pstcBt);
#if defined(PDL_INTERRUPT_BT_ACTIVE)
en_result_t Bt_Pwm_EnableIrq(volatile stc_btn_t* pstcBt, en_pwm_irq_sel_t enIrqSel);
en_result_t Bt_Pwm_DisableIrq(volatile stc_btn_t* pstcBt, en_pwm_irq_sel_t enIrqSel);
#endif
/* Interrupt flag read/clear */
en_irq_flag_t Bt_Pwm_GetIrqFlag(volatile stc_btn_t* pstcBt, en_pwm_irq_sel_t enIrqSel);
en_result_t Bt_Pwm_ClrIrqFlag(volatile stc_btn_t* pstcBt, en_pwm_irq_sel_t enIrqSel);
/* Write/read count */
en_result_t Bt_Pwm_WriteCycleVal(volatile stc_btn_t* pstcBt, uint16_t u16Cycle);
en_result_t Bt_Pwm_WriteDutyVal(volatile stc_btn_t* pstcBt, uint16_t u16Duty);
uint16_t Bt_Pwm_ReadCurCnt(volatile stc_btn_t* pstcBt);
#endif

#if (PDL_PERIPHERAL_ENABLE_BT_PPG_MODE == PDL_ON)
/* 2.2 PPG timer */
/* Init */
en_result_t Bt_Ppg_Init(volatile stc_btn_t* pstcBt, stc_bt_ppg_config_t* pstcPpgConfig);
en_result_t Bt_Ppg_DeInit(volatile stc_btn_t* pstcBt, boolean_t bTouchNvic);
/* Func/Int Enable/Disable */
en_result_t Bt_Ppg_EnableCount(volatile stc_btn_t* pstcBt);
en_result_t Bt_Ppg_DisableCount(volatile stc_btn_t* pstcBt);
en_result_t Bt_Ppg_EnableSwTrig(volatile stc_btn_t* pstcBt);
#if defined(PDL_INTERRUPT_BT_ACTIVE)
en_result_t Bt_Ppg_EnableIrq(volatile stc_btn_t* pstcBt, en_ppg_irq_sel_t enIrqSel);
en_result_t Bt_Ppg_DisableIrq(volatile stc_btn_t* pstcBt, en_ppg_irq_sel_t enIrqSel);
#endif
/* Interrupt flag read/clear */
en_irq_flag_t Bt_Ppg_GetIrqFlag(volatile stc_btn_t* pstcBt, en_ppg_irq_sel_t enIrqSel);
en_result_t Bt_Ppg_ClrIrqFlag(volatile stc_btn_t* pstcBt, en_ppg_irq_sel_t enIrqSel);
/* Write/read count */
en_result_t Bt_Ppg_WriteLowWidthVal(volatile stc_btn_t* pstcBt, uint16_t u16Val);
en_result_t Bt_Ppg_WriteHighWidthVal(volatile stc_btn_t* pstcBt, uint16_t u16Val);
uint16_t Bt_Ppg_ReadCurCnt(volatile stc_btn_t* pstcBt);
#endif

#if (PDL_PERIPHERAL_ENABLE_BT_RT_MODE == PDL_ON)
/* 2.3 RT timer */
/* Init */
en_result_t Bt_Rt_Init(volatile stc_btn_t* pstcBt, stc_bt_rt_config_t* pstcRtConfig);
en_result_t Bt_Rt_DeInit(volatile stc_btn_t* pstcBt, boolean_t bTouchNvic);
/* Func/Int Enable/Disable */
en_result_t Bt_Rt_EnableCount(volatile stc_btn_t* pstcBt);
en_result_t Bt_Rt_DisableCount(volatile stc_btn_t* pstcBt);
en_result_t Bt_Rt_EnableSwTrig(volatile stc_btn_t* pstcBt);
#if defined(PDL_INTERRUPT_BT_ACTIVE)
en_result_t Bt_Rt_EnableIrq(volatile stc_btn_t* pstcBt, en_rt_irq_sel_t enIrqSel);
en_result_t Bt_Rt_DisableIrq(volatile stc_btn_t* pstcBt, en_rt_irq_sel_t enIrqSel);
#endif
/* Interrupt flag read/clear */
en_irq_flag_t Bt_Rt_GetIrqFlag(volatile stc_btn_t* pstcBt, en_rt_irq_sel_t enIrqSel);
en_result_t Bt_Rt_ClrIrqFlag(volatile stc_btn_t* pstcBt, en_rt_irq_sel_t enIrqSel);
/* Write/read count */
en_result_t Bt_Rt_WriteCycleVal(volatile stc_btn_t* pstcBt, uint16_t u16Val);
uint16_t Bt_Rt_ReadCurCnt(volatile stc_btn_t* pstcBt);
#endif

#if (PDL_PERIPHERAL_ENABLE_BT_PWC_MODE == PDL_ON)
/* 2.4 PWC timer */
/* Init */
en_result_t Bt_Pwc_Init(volatile stc_btn_t* pstcBt, stc_bt_pwc_config_t* pstcPwcConfig);
en_result_t Bt_Pwc_DeInit(volatile stc_btn_t* pstcBt, boolean_t bTouchNvic);
/* Func/Int Enable/Disable */
en_result_t Bt_Pwc_EnableCount(volatile stc_btn_t* pstcBt);
en_result_t Bt_Pwc_DisableCount(volatile stc_btn_t* pstcBt);
#if defined(PDL_INTERRUPT_BT_ACTIVE)
en_result_t Bt_Pwc_EnableIrq(volatile stc_btn_t* pstcBt, en_pwc_irq_sel_t  enIrqSel);
en_result_t Bt_Pwc_DisableIrq(volatile stc_btn_t* pstcBt, en_pwc_irq_sel_t enIrqSel);
#endif
/* Interrupt flag read/clear */
en_irq_flag_t Bt_Pwc_GetIrqFlag(volatile stc_btn_t* pstcBt, en_pwc_irq_sel_t enIrqSel);
en_result_t Bt_Pwc_ClrIrqFlag(volatile stc_btn_t* pstcBt, en_pwc_irq_sel_t enIrqSel);
en_stat_flag_t Bt_Pwc_GetErrorFlag(volatile stc_btn_t* pstcBt);
/* Write/read count */
uint16_t Bt_Pwc_Get16BitMeasureData(volatile stc_btn_t* pstcBt);
uint32_t Bt_Pwc_Get32BitMeasureData(volatile stc_btn_t* pstcBt);
#endif

#if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE)
/* 4. Start/stop BT simultaneously  */
void Bt_SetSimultaneousStart(uint16_t u16Value);
#endif

/* 5. IRQ handler */
void Bt_IrqHandler( volatile stc_btn_t* pstcBt,
                    stc_bt_intern_data_t* pstcBtInternData) ;


#ifdef __cplusplus
}
#endif

//@} // BtGroup

#endif // #if (defined(PDL_PERIPHERAL_BT_ACTIVE))

#endif // #if  __BT_H__
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
