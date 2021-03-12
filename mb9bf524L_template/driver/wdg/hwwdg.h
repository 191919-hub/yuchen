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
/** \file hwwdg.h
 **
 ** Headerfile for Hardware Watchdog functions
 **  
 ** History:
 **   - 2014-11-02  1.0  EZh   First version for FM universal PDL.
 **
 ******************************************************************************/

#ifndef __HWWDG_H__
#define __HWWDG_H__

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "mcu.h"
#include "pdl_user.h"

#if (defined(PDL_PERIPHERAL_WDG_ACTIVE))

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/**
 ******************************************************************************
 ** \defgroup HwwdgGroup Hardware Watchdog (HWWDG)
 **
 ** Provided functions of HWWDG module:
 ** 
 ** - Hwwdg_Init()
 ** - Hwwdg_DeInit()
 ** - Hwwdg_Start()
 ** - Hwwdg_Stop()
 ** - Hwwdg_WriteWdgLoad()
 ** - Hwwdg_ReadWdgValue()
 ** - Hwwdg_Feed()
 ** - Hwwdg_QuickFeed()
 ** - Hwwdg_EnableDbgBrkWdgCtl()
 ** - Hwwdg_DisableDbgBrkWdgCtl()
 **
 ** This module has dedicated interrupt callback functions, in which the user 
 ** has to feed the Hardware Watchdog.
 **
 ** For the Hardware Watchdog Hwwdg_Init() is used for setting the interval
 ** time. Hwwdg_Feed() resets the Watchdog timer by a function call.
 ** Hwwdg_QuickFeed() does the same, but the code is inline expanded for
 ** e.g. time-critical polling loops.
 **
 ** Hwwdg_Init() sets the Hardware Watchdog interval.
 ** Hwwdg_DeInit() stops and disables the Hardware Watchdog, if two magic words
 ** as parameters are used.
 ** Hwwdg_Start() starts interruption and count of the Hardware Watchdog.
 ** Hwwdg_Stop() stops interruption and count of the Hardware Watchdog.
 ** Hwwdg_WriteWdgLoad() writes load value for the Hardware Watchdog.
 ** Hwwdg_ReadWdgValue() reads counter value of the Hardware Watchdog.
 ** Hwwdg_Feed() and Hwwdg_QuickFeed() do the same as their correspondig
 ** functions for the Software Watchdog, but here are two parameter needed,
 ** the 2nd one the inverted value of the 1st.
 ** Hwwdg_EnableDbgBrkWdgCtl() continues counting at the tool break. 
 ** Hwwdg_DisableDbgBrkWdgCtl() stops counting at the tool break.
 **
 ** \note 
 **    - The Hardware Watchdog shares its interrupt vector with the NMI.
 **    - The Hardware Watchdog is also switched off in System_Init() in
 **      system_fmx.c. Set the definition for HWWD_DISABLE to 0 in 
 **      system_fmx.h for letting the Hardware Watchdog running during
 **      start-up phase.
 **
 ******************************************************************************/
//@{

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/
/* for WDG_LCK */
#define HWWDG_REG_UNLOCK_1  (0x1ACCE551u)
#define HWWDG_REG_UNLOCK_2  (0xE5331AAEu)

/******************************************************************************
 * Global type definitions
 ******************************************************************************/

/**
 *****************************************************************************
 ** \brief Hardware Watchdog configuration
 *****************************************************************************/
typedef struct stc_hwwdg_config
{
    uint32_t   u32LoadValue;    ///< Timer interval
    boolean_t  bResetEnable;    ///< TRUE:  Enables Hardware watchdog reset
    func_ptr_t pfnHwwdgIrqCb;   ///< Pointer to hardware watchdog interrupt callback function
    
} stc_hwwdg_config_t;

/******************************************************************************/
/* Local type definitions ('typedef')                                         */
/******************************************************************************/

/******************************************************************************/
/* Global variable definitions ('extern')                                     */
/******************************************************************************/

/******************************************************************************/
/* Global function prototypes (definition in C source)                        */
/******************************************************************************/
/* Interrupt */
void HwwdgIrqHandler(void);

/* Setup(Initialize)/Disable(Un-initialize) */
en_result_t Hwwdg_Init(stc_hwwdg_config_t* pstcConfig);
en_result_t Hwwdg_DeInit(uint32_t u32MagicWord1,
                         uint32_t u32MagicWord2 );

/* Start/Stop */
en_result_t Hwwdg_Start(void);
void Hwwdg_Stop(void);

/* Write/Read for counter */
void Hwwdg_WriteWdgLoad(uint32_t u32LoadValue);
uint32_t Hwwdg_ReadWdgValue(void);

/* Feed watchdog */
void Hwwdg_Feed(uint8_t u8ClearPattern1, uint8_t u8ClearPattern2);

void Hwwdg_EnableDbgBrkWdgCtl(void);
void Hwwdg_DisableDbgBrkWdgCtl(void);

static void Hwwdg_QuickFeed(uint8_t u8ClearPattern1, uint8_t u8ClearPattern2);

/******************************************************************************/
/* Static inline functions                                                    */
/******************************************************************************/

/**
 ******************************************************************************
 ** \brief Quickly feed Hardware Watchdog (inline function)
 **
 ** This function feeds the Hardware Watchdog with the unlock, feed, and lock
 ** sequence as an inline function for quick execution in polling loops.
 ** Take care of the arbitrary values, because there are not checked for
 ** plausibility!
 **
 ** \param [in] u8ClearPattern1   Pattern of arbitrary value
 ** \param [in] u8ClearPattern2   Inverted arbitrary value
 ******************************************************************************/
static __INLINE void Hwwdg_QuickFeed(uint8_t u8ClearPattern1, uint8_t u8ClearPattern2)
{
    FM_HWWDT->WDG_LCK = HWWDG_REG_UNLOCK_1;/* Release Lock */
    FM_HWWDT->WDG_ICL = u8ClearPattern1;   /* Clear possible interrupt and reload value, step 1 */
    FM_HWWDT->WDG_ICL = u8ClearPattern2;   /* Clear possible interrupt and reload value, step 2 */
} /* Hwwdg_QuickFeed */

//@} // HwwdgGroup

#ifdef __cplusplus
}
#endif

#endif /* #if (defined(PDL_PERIPHERAL_WDG_ACTIVE)) */

#endif /* __HWWDG_H__ */
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
