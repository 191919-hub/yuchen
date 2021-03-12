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
/** \file reset.c
 **
 ** A detailed description is available at 
 ** @link ResetGroup Reset Cause description @endlink
 **
 ** History:
 **   - 2014-12-09  1.0  EZh  First version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "reset.h"
#include <string.h>

#if (defined(PDL_PERIPHERAL_RESET_ACTIVE))

/**
 ******************************************************************************
 ** \addtogroup ResetGroup
 ******************************************************************************/
//@{

/******************************************************************************/
/* Local pre-processor symbols/macros ('#define')                             */
/******************************************************************************/

/******************************************************************************/
/* Global variable definitions (declared in header file with 'extern')        */
/******************************************************************************/
stc_reset_result_t stcStoredResetCause; ///< Global reset cause register
  

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
 ** \brief Get current Reset Cause Register contents
 **
 ** This function reads the Reset Cause Register and stores the cause bits
 ** in the result structure pointer.
 **
 ** \attention Calling this function clears all bits in the Reset Cause Register
 **            RST_STR! Reset_GetCause() should only be called after Start-up
 **            code!
 **
 ** \param [out]  pstcResult     Reset Cause result structure
 **
 ** \retval Ok                   Function done sucessfully
 **
 ******************************************************************************/
en_result_t Reset_GetCause(stc_reset_result_t* pstcResult)
{
  stc_crg_rst_str_field_t stcReadResetCause;
  uint16_t u16ResetCause;
  
  u16ResetCause = FM_CRG->RST_STR;
  stcReadResetCause = *(stc_crg_rst_str_field_t*)&u16ResetCause;

  // Check Power-on bit
  if (1u == stcReadResetCause.PONR)
  {
    pstcResult->bPowerOn = TRUE;
  }
  else
  {
    pstcResult->bPowerOn = FALSE;
  }
  
  // Check INITX/External Reset bit
  if (1u == stcReadResetCause.INITX)
  {
    pstcResult->bInitx = TRUE;
  }
  else
  {
    pstcResult->bInitx = FALSE;
  }

#if (PDL_MCU_TYPE == PDL_FM3_TYPE3) ||  (PDL_MCU_TYPE == PDL_FM3_TYPE7)   
  // Check Low Voltage Detection bit
  if (1u == stcReadResetCause.LVDH)
  {
    pstcResult->bLowVoltageDetection = TRUE;
  }
  else
  {
    pstcResult->bLowVoltageDetection = FALSE;
  }
#endif  
  
  // Check Software Watchdog bit
  if (1u == stcReadResetCause.SWDT)
  {
    pstcResult->bSoftwareWatchdog = TRUE;
  }
  else
  {
    pstcResult->bSoftwareWatchdog = FALSE;
  }
  
  // Check Hardware Watchdog bit
  if (1u == stcReadResetCause.HWDT)
  {
    pstcResult->bHardwareWatchdog = TRUE;
  }
  else
  {
    pstcResult->bHardwareWatchdog = FALSE;
  }
  
  // Check Clock Supervisor bit
  if (1u == stcReadResetCause.CSVR)
  {
    pstcResult->bClockSupervisor = TRUE;
  }
  else
  {
    pstcResult->bClockSupervisor = FALSE;
  }
  
  // Check Anomalous Frequency bit
  if (1u == stcReadResetCause.FCSR)
  {
    pstcResult->bAnomalousFrequency = TRUE;
  }
  else
  {
    pstcResult->bAnomalousFrequency = FALSE;
  }
  
  // Check Software Reset bit
  if (1u == stcReadResetCause.SRST)
  {
    pstcResult->bSoftware = TRUE;
  }
  else
  {
    pstcResult->bSoftware = FALSE;
  }

  // Copy reset cause structure argument contents to global reset cause structure
  memcpy(&stcStoredResetCause, pstcResult, sizeof(stcStoredResetCause));
  
  return Ok;
} // Reset_GetCause

/**
 ******************************************************************************
 ** \brief Get stored Reset Cause
 **
 ** This function reads the gloabal Reset Cause Variable
 **
 ** \pre Reset_GetCause() must be called before.
 **
 ** \param [out]  pstcResult     Reset Cause result structure
 **
 ** \retval Ok                   Function done sucessfully
 **
 ******************************************************************************/
en_result_t Reset_GetStoredCause( stc_reset_result_t* pstcResult )
{
  // Copy global reset cause structure contents to reset cause structure argument 
  memcpy(pstcResult, &stcStoredResetCause, sizeof(stcStoredResetCause));
  
  return Ok;
}

//@} // ResetGroup

#endif // #if (defined(PDL_PERIPHERAL_RESET_ACTIVE))

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
