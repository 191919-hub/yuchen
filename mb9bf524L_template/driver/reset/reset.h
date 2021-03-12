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
/** \file reset.h
 **
 ** Headerfile for RESET FACTOR functions
 **  
 ** History:
 **   - 2014-12-09  1.0  EZh  First version for FM universal PDL.
 **
 ******************************************************************************/

#ifndef __RESET_H__
#define __RESET_H__

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_user.h"
#include "mcu.h"

#if (defined(PDL_PERIPHERAL_RESET_ACTIVE))

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/**
 ******************************************************************************
 ** \defgroup ResetGroup Reset Factor (RESET)
 **
 ** Provided functions of RESET module:
 ** - Reset_GetCause()
 ** - Reset_GetStoredCause()
 **
 ** Reset_GetCause() reads the Reset Casue, clears the HW register, and stores
 ** the result in a global variable. This is needed, if the SystemInit function
 ** uses this function for retrieving the reset cause (for RTC, etc.), but the
 ** application itself also needs the reset cause.
 ** Reset_GetStoredCause() reads out the global reset cause variable, without
 ** touching the reset cause register. Reset_GetCause() must be called before.
 ** 
 ** This driver does not need any configuration structure, but uses a result
 ** structure #stc_reset_result_t, which has to be provided by the caller.
 **
 ******************************************************************************/
//@{

/******************************************************************************/
/* Global pre-processor symbols/macros ('#define')                            */
/******************************************************************************/
 
/******************************************************************************
 * Global type definitions
 ******************************************************************************/
  
/**
 ******************************************************************************
 ** \brief Datatype for Reset Cause return structure
 ******************************************************************************/
typedef struct stc_reset_result
{
  boolean_t    bPowerOn;              ///< TRUE: Power on reset occurred
  boolean_t    bInitx;                ///< TRUE: INITX (external) reset occurred
#if (PDL_MCU_TYPE == PDL_FM3_TYPE3) ||  (PDL_MCU_TYPE == PDL_FM3_TYPE7) 
  boolean_t    bLowVoltageDetection;  ///< TRUE: Low Voltage Detection reset ocucrred (only applicable for FM3 Type3 and 7, always FALSE otherwise)
#endif  
  boolean_t    bSoftwareWatchdog;     ///< TRUE: Software Watchdog reset occurred
  boolean_t    bHardwareWatchdog;     ///< TRUE: Hardware Watchdog reset occurred
  boolean_t    bClockSupervisor;      ///< TRUE: Clock Supervisor reset occurred
  boolean_t    bAnomalousFrequency;   ///< TRUE: Anomalous Frequency reset occurred
  boolean_t    bSoftware;             ///< TRUE: Software reset occurred
} stc_reset_result_t ;

/******************************************************************************/
/* Global variable definitions ('extern')                                     */
/******************************************************************************/

/******************************************************************************/
/* Global function prototypes (definition in C source)                        */
/******************************************************************************/

extern en_result_t Reset_GetCause( stc_reset_result_t* pstcResult ) ;
extern en_result_t Reset_GetStoredCause( stc_reset_result_t* pstcResult );

//@} // ResetGroup

#ifdef __cplusplus
}
#endif

#endif // #if (defined(PDL_PERIPHERAL_RESET_ACTIVE))

#endif /* __RESET_H__ */
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
