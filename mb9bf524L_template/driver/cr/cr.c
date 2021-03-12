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
/** \file cr.c
 **
 ** A detailed description is available at 
 ** @link CrGroup CR Module description @endlink
 **
 ** History:
 **   - 2014-11-18  0.1  EZh  First version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "cr.h"

#if (defined(PDL_PERIPHERAL_CR_ACTIVE))

/**
 ******************************************************************************
 ** \addtogroup CrGroup
 ******************************************************************************/
//@{

/******************************************************************************/
/* Local pre-processor symbols/macros ('#define')                             */
/******************************************************************************/
#define CR_REG_UNLOCK_CODE        0x1ACCE554u
#define CR_REG_LOCK_CODE          0u  
   
/******************************************************************************/
/* Global variable definitions (declared in header file with 'extern')        */
/******************************************************************************/

/******************************************************************************/
/* Local type definitions ('typedef')                                         */
/******************************************************************************/

/******************************************************************************/
/* Local function prototypes ('static')                                       */
/******************************************************************************/

/******************************************************************************/
/* Local variable definitions ('static')                                      */
/******************************************************************************/

/**
 ******************************************************************************
 ** \brief   Set the frequency division of CR output to Base timer
 **
 ** \param   [in]  enCrDiv            CR division
 ** \arg           CrFreqDivBy4
 ** \arg           CrFreqDivBy8
 ** \arg           CrFreqDivBy16
 ** \arg           CrFreqDivBy32
 ** \arg           CrFreqDivBy64
 ** \arg           CrFreqDivBy128
 ** \arg           CrFreqDivBy256
 ** \arg           CrFreqDivBy512
 **
 ** \retval  Ok                       Set the frequency division successfully
 ** \retval  ErrorInvalidParameter    enCrDiv > CrFreqDivBy512
 **
 ** \note The division values of 64,128,256,512 are only supported by FM3 TYPE3,
 **       TYPE7, FM4 and FM0+ products.
 **                                     
 ******************************************************************************/
en_result_t Cr_SetFreqDiv(en_cr_freq_div_t enCrDiv)
{
    switch (enCrDiv)
    {
        case CrFreqDivBy4:
            FM_CRTRIM->MCR_PSR = 0u;
            break;
        case CrFreqDivBy8:
            FM_CRTRIM->MCR_PSR = 1u;
            break;
        case CrFreqDivBy16:
            FM_CRTRIM->MCR_PSR = 2u;
            break;
        case CrFreqDivBy32:
            FM_CRTRIM->MCR_PSR = 3u;
            break;
#if (PDL_MCU_TYPE == PDL_FM3_TYPE3) || (PDL_MCU_TYPE == PDL_FM3_TYPE7) || \
    (PDL_MCU_CORE == PDL_FM0P_CORE) || (PDL_MCU_CORE == PDL_FM4_CORE)            
        case CrFreqDivBy64:
            FM_CRTRIM->MCR_PSR = 4u;
            break;
        case CrFreqDivBy128:
            FM_CRTRIM->MCR_PSR = 5u;
            break;
        case CrFreqDivBy256:
            FM_CRTRIM->MCR_PSR = 6u;
            break;
        case CrFreqDivBy512:
            FM_CRTRIM->MCR_PSR = 7u;
            break;
#endif
        default:
            return ErrorInvalidParameter;
    }
    
    return Ok;
}

#if (PDL_MCU_TYPE == PDL_FM3_TYPE8)  || (PDL_MCU_TYPE == PDL_FM3_TYPE9)  || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE10) || (PDL_MCU_TYPE == PDL_FM3_TYPE11) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE12) || (PDL_MCU_CORE == PDL_FM0P_CORE)  || \
    (PDL_MCU_CORE == PDL_FM4_CORE)  
/**
 ******************************************************************************
 ** \brief   Set CR temperature trimming register
 **
 ** \param   [in]  u8Data             temperature trimming value, only Bit[4:0]
 **                                   is valid.
 **
 ** \retval  Ok                       Set the frequency division successfully
 **                                     
 ******************************************************************************/
en_result_t Cr_SetTempTrimmingData(uint8_t u8Data)
{
    FM_CRTRIM->MCR_RLR = CR_REG_UNLOCK_CODE;
    FM_CRTRIM->MCR_TTRM = u8Data & 0x1Fu;
    FM_CRTRIM->MCR_RLR = CR_REG_LOCK_CODE;
    return Ok;
}

/**
 ******************************************************************************
 ** \brief   Get CR temperature trimming register      
 **
 ** \retval  temperature trimming value, only Bit[4:0] is valid.                       
 **                                     
 ******************************************************************************/
uint8_t Cr_GetTempTrimmingData(void)
{
    uint8_t u8CrTempData;
    u8CrTempData = FM_CRTRIM->MCR_TTRM & 0x1Fu;
    return u8CrTempData;
}

#endif

/**
 ******************************************************************************
 ** \brief   Set CR frequency trimming register
 **
 ** \param   [in]  u16Data             temperature trimming value, only Bit[9:0]
 **                                    is valid.
 **
 ** \retval  Ok                        Set CR frequency trimming successfully
 **
 ** \note The meaning of bit field of CR data is different among devices, see
 **       peripheral manual for the details.
 **                                     
 ******************************************************************************/
en_result_t Cr_SetFreqTrimmingData(uint16_t u16Data)
{
    FM_CRTRIM->MCR_RLR = CR_REG_UNLOCK_CODE;
#if (PDL_MCU_TYPE == PDL_FM3_TYPE1) || (PDL_MCU_TYPE == PDL_FM3_TYPE2) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE4) || (PDL_MCU_TYPE == PDL_FM3_TYPE5)   
    FM_CRTRIM->MCR_FTRM = u16Data & 0xFFu;
#else
    FM_CRTRIM->MCR_FTRM = u16Data & 0x3FFu;
#endif    
    FM_CRTRIM->MCR_RLR = CR_REG_LOCK_CODE;
    return Ok;
}

/**
 ******************************************************************************
 ** \brief   Get frequency trimming register
 **
 ** \retval  temperature trimming value, only Bit[9:0]
 **          is valid.
 **                                     
 ******************************************************************************/
uint16_t Cr_GetFreqTrimmingData(void)
{
    uint16_t u16CrFreqData;
#if (PDL_MCU_TYPE == PDL_FM3_TYPE1) || (PDL_MCU_TYPE == PDL_FM3_TYPE2) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE4) || (PDL_MCU_TYPE == PDL_FM3_TYPE5)       
    u16CrFreqData = FM_CRTRIM->MCR_FTRM & 0xFFu;
#else
    u16CrFreqData = FM_CRTRIM->MCR_FTRM & 0x3FFu;
#endif    
    return u16CrFreqData;
}

//@} // CrGroup

#endif /* #if (defined(PDL_PERIPHERAL_CR_ACTIVE)) */
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
