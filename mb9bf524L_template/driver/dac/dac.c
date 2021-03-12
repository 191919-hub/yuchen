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
/** \file dac.c
 **
 ** A detailed description is available at 
 ** @link DacGroup 12-Bit Digital Analog Converter description @endlink
 **
 ** History:
 **   - 2014-11-18  0.1  EZh  First version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "dac.h"

#if (defined(PDL_PERIPHERAL_DAC_ACTIVE))

/**
 ******************************************************************************
 ** \addtogroup DacGroup
 ******************************************************************************/
//@{

/******************************************************************************/
/* Local pre-processor symbols/macros ('#define')                             */
/******************************************************************************/

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

/******************************************************************************/
/* Function implementation - global ('extern') and local ('static')           */
/******************************************************************************/

/**
 ******************************************************************************
 ** \brief DAC Initialization
 **
 ** \param [in]  pstcDac           Pointer to DAC instance
 ** \param [in]  pstcConfig        Pointer to DAC configuration strucutre
 **
 ** \retval Ok                     DAC Instance sucessfully initialized
 ** \retval ErrorInvalidParameter  pstcDac == NULL or pstcConfig == NULL
 ******************************************************************************/
en_result_t Dac_Init( stc_dacn_t*       pstcDac,
                      stc_dac_config_t* pstcConfig
                    )
{
    // Check for NULL pointers
    if ((NULL  == pstcDac) ||
        (NULL  == pstcConfig)
       )
    {
      return ErrorInvalidParameter;
    }      
#if (PDL_MCU_CORE == PDL_FM4_CORE)     
    pstcDac->DACR0_f.DAC10 = (TRUE == pstcConfig->bDac12Bit0)        ? 0u : 1u ;
    pstcDac->DACR0_f.DAC10 = (TRUE == pstcConfig->bDac10RightAlign0) ? 1u : 0u ;
    pstcDac->DACR1_f.DAC10 = (TRUE == pstcConfig->bDac12Bit1)        ? 0u : 1u ;
    pstcDac->DACR1_f.DAC10 = (TRUE == pstcConfig->bDac10RightAlign1) ? 1u : 0u ;
#endif    
    return Ok;
} // Dac_Init

/**
 ******************************************************************************
 ** \brief DAC De-Initialization
 **
 ** \param [in]  pstcDac           Pointer to DAC instance
 **
 ** \retval Ok                     DAC Instance sucessfully de-initialized
 ** \retval ErrorInvalidParameter  pstcDac == NULL or pstcConfig == NULL
 ******************************************************************************/
en_result_t Dac_DeInit( stc_dacn_t* pstcDac )
{
    // Check for NULL pointers
    if (NULL == pstcDac)
    {
      return ErrorInvalidParameter;
    }      
     
    pstcDac->DACR0 = 0u;
    pstcDac->DADR0 = 0u;
    pstcDac->DACR1 = 0u;
    pstcDac->DADR1 = 0u;
    
    return Ok;
} // Dac_DeInit

/**
 ******************************************************************************
 ** \brief Set DAC0 12-bit value
 **
 ** \param [in]  pstcDac        Pointer to DAC instance
 ** \param [in]  u16DacValue    DAC0 value to be output
 **
 ** \retval Ok                  Value written.
 ******************************************************************************/
en_result_t Dac_SetValue0( stc_dacn_t* pstcDac,
                           uint16_t    u16DacValue
                         )
{
  pstcDac->DADR0_f.DA = u16DacValue;
  
  return Ok;
} // Dac_SetValue0


/**
 ******************************************************************************
 ** \brief Set DAC1 12-bit value
 **
 ** \param [in]  pstcDac        Pointer to DAC instance
 ** \param [in]  u16DacValue    DAC1 value to be output
 **
 ** \retval Ok                  Value written.
 ******************************************************************************/
en_result_t Dac_SetValue1( stc_dacn_t* pstcDac,
                           uint16_t    u16DacValue
                         )
{
  pstcDac->DADR1_f.DA = u16DacValue;
  
  return Ok;
} // Dac_SetValue1

/**
 ******************************************************************************
 ** \brief Enable DAC0
 **
 ** \param [in]  pstcDac        Pointer to DAC instance
 **
 ** \retval Ok                  DAC0 enabled
 ******************************************************************************/
en_result_t Dac_Enable0( stc_dacn_t* pstcDac )
{
  pstcDac->DACR0_f.DAE = 1u;
  
  return Ok;
} // Dac_Enable0

/**
 ******************************************************************************
 ** \brief Enable DAC1
 **
 ** \param [in]  pstcDac        Pointer to DAC instance
 **
 ** \retval Ok                  DAC1 enabled
 ******************************************************************************/
en_result_t Dac_Enable1( stc_dacn_t* pstcDac )
{
  pstcDac->DACR1_f.DAE = 1u;
  
  return Ok;
} // Dac_Enable1


/**
 ******************************************************************************
 ** \brief Disable DAC0
 **
 ** \param [in]  pstcDac        Pointer to DAC instance
 **
 ** \retval Ok                  DAC0 disabled
 ******************************************************************************/
en_result_t Dac_Disable0( stc_dacn_t* pstcDac )
{
  pstcDac->DACR0_f.DAE = 0u;
  
  return Ok;
} // Dac_Disable0

/**
 ******************************************************************************
 ** \brief Disable DAC1
 **
 ** \param [in]  pstcDac        Pointer to DAC instance
 **
 ** \retval Ok                  DAC1 disabled
 ******************************************************************************/
en_result_t Dac_Disable1( stc_dacn_t* pstcDac )
{
  pstcDac->DACR1_f.DAE = 0u;
  
  return Ok;
} // Dac_Disable1


//@} // DacGroup

#endif // #if (defined(PDL_PERIPHERAL_DAC_ACTIVE))

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
