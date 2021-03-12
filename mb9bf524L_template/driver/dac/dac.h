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
/** \file dac.h
 **
 ** Headerfile for DAC functions
 **  
 ** History:
 **   - 2014-11-18  0.1  EZh  First version for FM universal PDL.
 **
 ******************************************************************************/

#ifndef __DAC_H__
#define __DAC_H__

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_user.h"
#include "mcu.h"

#if (PDL_PERIPHERAL_DAC_AVAILABLE == PDL_OFF)
    #if (defined(PDL_PERIPHERAL_DAC_ACTIVE))
        #error "Don't enable DAC, as it is not available on this device"
    #endif
#endif

#if (defined(PDL_PERIPHERAL_DAC_ACTIVE))

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/**
 ******************************************************************************
 ** \defgroup DacGroup 10/12-Bit Digital Analog Converter (DAC)
 **
 ** Provided functions of DAC module:
 ** - Dac_Init()
 ** - Dac_DeInit()
 ** - Dac_SetValue0()
 ** - Dac_SetValue1()
 ** - Dac_Enable0()
 ** - Dac_Enable1()
 ** - Dac_Disable0()
 ** - Dac_Disable1()
 **
 ** With Dac_Init() the DAC is configured (10/12-Bit mode, 10-Bit data
 ** alignment). Each channel can be en- and disabled by e.g. Dac_Enable0(),
 ** Dac_Disable0(). Output values can be set easily by Dac_SetValue0() and
 ** Dac_SetValue1().
 **
 ******************************************************************************/
//@{

/******************************************************************************/
/* Global pre-processor symbols/macros ('#define')                            */
/******************************************************************************/
 
/******************************************************************************
 * Global type definitions
 ******************************************************************************/
#define stc_dacn_t   FM_DAC_TypeDef
#define DAC0  (*(( stc_dacn_t *) FM_DAC_BASE))
  

/**
 ******************************************************************************
 ** \brief DAC configuration.
 ******************************************************************************/
typedef struct stc_dac_config
{
#if (PDL_MCU_CORE == PDL_FM4_CORE)    
  boolean_t bDac12Bit0;            ///< Channel 0: TRUE: 12-Bit mode, FALSE: 10-Bit mode
  boolean_t bDac10RightAlign0;     ///< Channel 0: TRUE: 10-Bit data aligned to DA[9:0], FLASE: 10-Bit data aligned to DA[11:2]
  boolean_t bDac12Bit1;            ///< Channel 1: TRUE: 12-Bit mode, FALSE: 10-Bit mode
  boolean_t bDac10RightAlign1;     ///< Channel 1: TRUE: 10-Bit data aligned to DA[9:0], FLASE: 10-Bit data aligned to DA[11:2]
#endif 
  boolean_t bDummy;                ///< Only for avoid error when select other devices rather than FM4 devices
} stc_dac_config_t;

/******************************************************************************/
/* Global variable definitions ('extern')                                     */
/******************************************************************************/

/******************************************************************************/
/* Global function prototypes (definition in C source)                        */
/******************************************************************************/
en_result_t Dac_Init( stc_dacn_t*       pstcDac,
                      stc_dac_config_t* pstcConfig
                    );

en_result_t Dac_DeInit( stc_dacn_t* pstcDac );

en_result_t Dac_SetValue0( stc_dacn_t* pstcDac,
                           uint16_t    u16DacValue
                         );

en_result_t Dac_SetValue1( stc_dacn_t* pstcDac,
                           uint16_t    u16DacValue
                         );

en_result_t Dac_Enable0( stc_dacn_t* pstcDac );

en_result_t Dac_Enable1( stc_dacn_t* pstcDac );

en_result_t Dac_Disable0( stc_dacn_t* pstcDac );

en_result_t Dac_Disable1( stc_dacn_t* pstcDac );

//@} // DacGroup

#ifdef __cplusplus
}
#endif

#endif // #if (defined(PDL_PERIPHERAL_DAC_ACTIVE))

#endif /* __DAC_H__ */
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/

