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
/** \file wc.h
 **
 ** Headerfile for WC functions
 **
 ** History:
 **   - 2014-11-17  0.1  EZh  First version for FM universal PDL.
 **
 ******************************************************************************/

#ifndef __WC_H__
#define __WC_H__

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "mcu.h"
#include "pdl_user.h"

#if (defined(PDL_PERIPHERAL_WC_ACTIVE))

/**
 ******************************************************************************
 ** \defgroup WcGroup Watch Counter (WC)
 **
 ** Provided functions of WC module:
 ** 
 ** - Wc_Pres_SelClk()
 ** - Wc_Pres_EnableDiv() 
 ** - Wc_Pres_DisableDiv()
 ** - Wc_Init()
 ** - Wc_EnableCount()
 ** - Wc_DisableCount()
 ** - Wc_EnableIrq()
 ** - Wc_DisableIrq()
 ** - Wc_WriteReloadVal()
 ** - Wc_ReadCurCnt()
 ** - Wc_ClearIrqFlag()
 ** - Wc_GetIrqFlag()
 ** - Wc_GetOperationFlag()
 **
 ** <b>How to use WC module?</b>
 **
 ** Before using WC, WC prescaler must be configured first. Use Wc_Pres_SelClk()
 ** to select input clock of prescaler. Following clocks can be selected:
 ** - Sub clock
 ** - Main clock
 ** - High-speed CR
 ** - CLKLC (divided by low speed CR)
 ** 
 ** Wc_Pres_EnableDiv() is used to enable watch counter prescaler. 
 ** Wc_Pres_DisableDiv() is used to disable watch counter prescaler. 
 **
 ** Wc_Init() must be used for configuration of watch counter with a structure 
 ** of the type #stc_wc_config_t.
 **
 ** A WC interrupt can be enabled by the function Wc_EnableIrq().
 **
 ** With Wc_WriteReloadVal() the WC reloader value is set to the value
 ** given in the parameter Wc_WriteReloadVal#u8Val.
 **
 ** After above setting, calling Wc_EnableCount() will start WC.
 **
 ** With Wc_ReadCurCnt() the current WC count can be read when WC is 
 ** counting. with Wc_GetOperationFlag() the current WC operation status
 ** can be read.
 **
 ** With interrupt mode, when the interrupt occurs, the interrupt flag will
 ** be cleared and run into user interrupt callback function.
 **
 ** With polling mode, user can use Wc_GetIrqFlag() to check if the 
 ** interrupt occurs, and clear the interrupt flag by Wc_ClearIrqFlag().
 ** 
 ** When stopping the WC, use Wc_DisableCount() to disable WC and 
 ** Wc_DisableIrq() to disable WC interrupt.
 **
 ******************************************************************************/

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/
#define stc_wcn_t   FM_WC_TypeDef
#define WC0         (*((stc_wcn_t *) FM_WC_BASE))

/**
 ******************************************************************************
 ** \brief Enumeration to define source clock of watch counter precaler
 ******************************************************************************/ 
typedef enum en_input_clk
{
    WcPresInClkSubOsc   = 0u,      ///< Watch counter prescaler source clock: sub oscillator
    WcPresInClkMainOsc  = 1u,      ///< Watch counter prescaler source clock: main oscillator
#if (PDL_MCU_TYPE == PDL_FM3_TYPE12) || (PDL_MCU_CORE == PDL_FM0P_CORE) || (PDL_MCU_CORE == PDL_FM4_CORE)    
    WcPresInClkHighCr   = 2u,      ///< Watch counter prescaler source clock: high-speed CR
    WcPresInClkLowCr    = 3u,      ///< Watch counter prescaler source clock: low-speed CR 
#endif
}en_input_clk_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define output clock of watch counter precaler
 ******************************************************************************/ 
typedef enum en_output_clk
{                                ///                                         WCCk3           WCCk2           WCCk1           WCCk0                         
    WcPresOutClkArray0 = 0u,     ///< Watch counter prescaler output array0: 2^15/src clock, 2^14/src clock, 2^13/src clock, 2^12/src clock
    WcPresOutClkArray1 = 1u,     ///< Watch counter prescaler output array0: 2^25/src clock, 2^24/src clock, 2^23/src clock, 2^22/src clock
#if (PDL_MCU_TYPE == PDL_FM3_TYPE12) || (PDL_MCU_CORE == PDL_FM0P_CORE) || (PDL_MCU_CORE == PDL_FM4_CORE)        
    WcPresOutClkArray2 = 2u,     ///< Watch counter prescaler output array0: 2^4/src clock, 2^3/src clock, 2^2/src clock, 2/src clock
    WcPresOutClkArray3 = 3u,     ///< Watch counter prescaler output array0: 2^8/src clock, 2^7/src clock, 2^6/src clock, 2^5/src clock
    WcPresOutClkArray4 = 4u,     ///< Watch counter prescaler output array0: 2^12/src clock, 2^11/src clock, 2^10/src clock, 2^9/src clock
    WcPresOutClkArray5 = 5u,     ///< Watch counter prescaler output array0: 2^19/src clock, 2^18/src clock, 2^17/src clock, 2^16/src clock
    WcPresOutClkArray6 = 6u,     ///< Watch counter prescaler output array0: 2^23/src clock, 2^22/src clock, 2^21/src clock, 2^20/src clock
#endif    
}en_output_clk_t;

/**
 ******************************************************************************
 ** \brief Structure to configure watch counter prescaler
 ******************************************************************************/
typedef struct stc_wc_pres_clk
{
    en_input_clk_t      enInputClk;   ///< Watch counter prescaler input clock setting
    en_output_clk_t     enOutputClk;  ///< Watch counter prescaler output clock setting
   
}stc_wc_pres_clk_t;


/**
 ******************************************************************************
 ** \brief Enumeration to set the clock of watch counter
 ******************************************************************************/
typedef enum en_wc_cnt_clk
{
    WcCntClkWcck0 = 0u,       ///< Watch counter source clock: WCCK0
    WcCntClkWcck1 = 1u,       ///< Watch counter source clock: WCCK1
    WcCntClkWcck2 = 2u,       ///< Watch counter source clock: WCCK2
    WcCntClkWcck3 = 3u,       ///< Watch counter source clock: WCCK3
    
}en_wc_cnt_clk_t;

/**
 ******************************************************************************
 ** \brief Structure to configure the watch counter
 ******************************************************************************/
typedef struct stc_wc_config
{
    en_wc_cnt_clk_t  enCntClk;      ///< Watch counter source clock setting
    uint8_t          u8ReloadValue; ///< Reload value 
#if (PDL_INTERRUPT_ENABLE_WC0 == PDL_ON)    
    boolean_t        bIrqEnable;     ///< TRUE: enable IRQ, FALSE: don't enable IRQ
    func_ptr_t       pfnIrqCallback; ///< Pointer to interrupt callback function
    boolean_t        bTouchNvic;     ///< TRUE: enable NVIC, FALSE: don't enable NVIC
#endif  
}stc_wc_config_t;


/******************************************************************************/
/* Global variable definitions ('extern')                                     */
/******************************************************************************/

/******************************************************************************/
/* Global function prototypes (definition in C source)                        */
/******************************************************************************/
/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/* 1. WC prescaler setting */
en_result_t Wc_Pres_SelClk(stc_wcn_t* pstcWc, stc_wc_pres_clk_t* pstcWcPresClk);
en_result_t Wc_Pres_EnableDiv(stc_wcn_t* pstcWc);
en_result_t Wc_Pres_DisableDiv(stc_wcn_t* pstcWc);
en_stat_flag_t Wc_Pres_GetDivStat(stc_wcn_t* pstcWc);
/* 2. WC setting */
/* Init/De-Init */
en_result_t Wc_Init(stc_wcn_t* pstcWc, stc_wc_config_t* pstcWcConfig);
en_result_t Wc_DeInit(stc_wcn_t* pstcWc, boolean_t bTouchNvic);
/* Func/Int enable/disable */
en_result_t Wc_EnableCount(stc_wcn_t* pstcWc);
en_result_t Wc_DisableCount(stc_wcn_t* pstcWc);
#if (PDL_INTERRUPT_ENABLE_WC0 == PDL_ON)
en_result_t Wc_EnableIrq(stc_wcn_t* pstcWc);
en_result_t Wc_DisableIrq(stc_wcn_t* pstcWc);
#endif
/* Count write/read */
en_result_t Wc_WriteReloadVal(stc_wcn_t* pstcWc, uint8_t u8Val);
uint8_t Wc_ReadCurCnt(stc_wcn_t* pstcWc);
/* Status clear */
en_result_t Wc_ClearIrqFlag(stc_wcn_t* pstcWc);
/* Status read */
en_irq_flag_t Wc_GetIrqFlag(stc_wcn_t* pstcWc);
en_stat_flag_t Wc_GetOperationFlag(stc_wcn_t* pstcWc);
/* 3. IRQ handler */
void Wc_IrqHandler(stc_wcn_t* pstcWc) ;

//@} // WcGroup

#ifdef __cplusplus
}
#endif

#endif // #if (defined(L3_PERIPHERAL_WC_ACTIVE))

#endif /* __WC_H__ */
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
