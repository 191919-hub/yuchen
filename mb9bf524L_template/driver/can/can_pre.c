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
/** \file can_pre.c
 **
 ** A detailed description is available at
 ** @link CanPreGroup CAN Prescaler Module description @endlink
 **
 ** History:
 **   - 2014-08-28  1.0  AI   First version.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "can_pre.h"

#if (defined(PDL_PERIPHERAL_CANFD_ACTIVE))

/**
 ******************************************************************************
 ** \addtogroup CanPreGroup
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
/* Function implementation - global (no 'static') and local ('static')        */
/******************************************************************************/

/******************************************************************************/
/* Local Functions                                                            */
/******************************************************************************/

/******************************************************************************/
/* Global Functions                                                           */
/******************************************************************************/

/**
 *****************************************************************************
 ** \brief Initialize CAN prescaler
 **
 ** Setup CAN prescaler.
 **
 ** \param [in] enPrescale			Prescaler divisor.
 **
 ** \retval Ok                    	CAN prescaler has been successfully setup.
 ** \retval ErrorInvalidParameter 	Invalid divisor.
 *****************************************************************************/
en_result_t Canpre_Init( en_canfd_prescaler_t enPrescale )
{
	en_result_t	enResult;

	// Initialize result value (as 'Ok').
	enResult = Ok;

	// CAN dedicated clock prescaler.
	switch ( enPrescale )
	{
		case CanfdPreDiv11:
			FM_CANPRES->CANPRE = 0x00u;
			break;
		case CanfdPreDiv12:
			FM_CANPRES->CANPRE = 0x01u;
			break;
		case CanfdPreDiv14:
			FM_CANPRES->CANPRE = 0x02u;
			break;
		case CanfdPreDiv18:
			FM_CANPRES->CANPRE = 0x04u;
			break;
		case CanfdPreDiv23:
			FM_CANPRES->CANPRE = 0x08u;
			break;
		case CanfdPreDiv13:
			FM_CANPRES->CANPRE = 0x09u;
			break;
		case CanfdPreDiv16:
			FM_CANPRES->CANPRE = 0x0Au;
			break;
		case CanfdPreDiv112:
			FM_CANPRES->CANPRE = 0x0Bu;
			break;
		case CanfdPreDiv15:
			FM_CANPRES->CANPRE = 0x0Cu;
			break;
		case CanfdPreDiv110:
			FM_CANPRES->CANPRE = 0x0Eu;
			break;
		default:
			enResult = ErrorInvalidParameter;
			break;
	}

	// Return result.
	return enResult;
}

//@} // CanPreGroup

#endif // if (defined(PDL_PERIPHERAL_CANPRE_ACTIVE))

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
