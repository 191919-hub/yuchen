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
/** \file cr.h
 **
 ** Headerfile for CR functions
 **  
 **
 ** History:
 **   - 2014-11-18  0.1  EZh  First version for FM universal PDL.
 **
 ******************************************************************************/

#ifndef __CR_H__
#define __CR_H__

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "mcu.h"
#include "pdl_user.h"

#if (defined(PDL_PERIPHERAL_CR_ACTIVE))

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/**
 ******************************************************************************
 ** \defgroup CrGroup CR-trimming Function (CR)
 **
 ** Provided functions of CR module:
 ** 
 ** - Cr_SetFreqDiv()
 ** - Cr_SetTempTrimmingData()
 ** - Cr_GetTempTrimmingData()
 ** - Cr_SetFreqTrimmingData()
 ** - Cr_GetFreqTrimmingData() 
 **
 ** Cr_SetFreqDiv() is used to divde CR input signal, which outputs to base 
 ** timer from dividor output.
 **
 ** Cr_SetTempTrimmingData() sets the CR temperature trimming data, and 
 ** Cr_GetTempTrimmingData() gets the CR temperature trimming data.
 ** Not all devices has the temperature trimming register. 
 **
 ** Cr_SetFreqTrimmingData() sets the CR frequency trimming data, and 
 ** Cr_GetFreqTrimmingData() gets the CR frequency trimming data.  
 **
 ******************************************************************************/
//@{

/******************************************************************************
 * Global definitions
 ******************************************************************************/
  
/**
 ******************************************************************************
 ** \brief CR frequency division values
 **
 ** \note The division signal is output to internal Base Timer or Multi-Fucntion
 **       Timer. The division values of 64,128,256,512 are only supported by 
 **       FM3 TYPE3, TYPE7, FM4 and FM0+ products.
 ******************************************************************************/
typedef enum en_cr_freq_div
{  
    CrFreqDivBy4    = 0u,       ///< CR output (to Base Timer) prescaler: 4
    CrFreqDivBy8    = 1u,       ///< CR output (to Base Timer) prescaler: 8
    CrFreqDivBy16   = 2u,       ///< CR output (to Base Timer) prescaler: 16
    CrFreqDivBy32   = 3u,       ///< CR output (to Base Timer) prescaler: 32
#if (PDL_MCU_TYPE == PDL_FM3_TYPE3) || (PDL_MCU_TYPE == PDL_FM3_TYPE7) || \
    (PDL_MCU_CORE == PDL_FM0P_CORE) || (PDL_MCU_CORE == PDL_FM4_CORE)
    CrFreqDivBy64   = 4u,       ///< CR output (to Base Timer) prescaler: 64
    CrFreqDivBy128  = 5u,       ///< CR output (to Base Timer) prescaler: 128
    CrFreqDivBy256  = 6u,       ///< CR output (to Base Timer) prescaler: 256
    CrFreqDivBy512  = 7u,       ///< CR output (to Base Timer) prescaler: 512
#endif  
}en_cr_freq_div_t;
   
/******************************************************************************/
/* Global variable definitions ('extern')                                     */
/******************************************************************************/
/******************************************************************************/
/* Global function prototypes (definition in C source)                        */
/******************************************************************************/
en_result_t Cr_SetFreqDiv(en_cr_freq_div_t enCrDiv);
#if (PDL_MCU_TYPE == PDL_FM3_TYPE8)  || (PDL_MCU_TYPE == PDL_FM3_TYPE9)  || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE10) || (PDL_MCU_TYPE == PDL_FM3_TYPE11) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE12) || (PDL_MCU_CORE == PDL_FM0P_CORE)  || \
    (PDL_MCU_CORE == PDL_FM4_CORE)  
en_result_t Cr_SetTempTrimmingData(uint8_t u8Data);
uint8_t Cr_GetTempTrimmingData(void);
#endif
en_result_t Cr_SetFreqTrimmingData(uint16_t u16Data);
uint16_t Cr_GetFreqTrimmingData(void);

//@} // CrGroup

#ifdef __cplusplus
}
#endif

#endif /* #if (defined(PDL_PERIPHERAL_CR_ACTIVE)) */

#endif /* __CR_H__ */
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
