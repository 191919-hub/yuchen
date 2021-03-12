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
/** \file can_pre.h
 **
 ** Headerfile for CAN prescaler functions
 **
 ** History:
 **   - 2014-08-28  1.0  AI   First version.
 **
 ******************************************************************************/

#ifndef __CAN_PRE_H__
#define __CAN_PRE_H__

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_user.h"
#include "mcu.h"

#if (defined(PDL_PERIPHERAL_CANFD_ACTIVE))

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/**
 ******************************************************************************
 ** \defgroup CanPreGroup Controller Area Network Prescaler (CAN Prescaler)
 **
 ** Provided functions of CAN prescaler module:
 ** - Canpre_Init() (Initialization of CAN prescaler)
 **
 ** <b>How to use this CAN prescaler module:</b>
 **
 ** This module is activated only when the CAN module is not active and the 
 ** CAN FD module is active.
 ** If the CAN module was activated, use Can_Init() API to initialize the CAN 
 ** prescaler instead.
 ******************************************************************************/
//@{

/******************************************************************************
 * Global definitions
 ******************************************************************************/

/******************************************************************************
 * Global type definitions
 ******************************************************************************/

/**
 ******************************************************************************
 ** \brief CAN Prescaler
 ******************************************************************************/
typedef enum en_canfd_prescaler
{
	CanfdPreDiv11  = 0,	// CAN prescaler clock: no division
	CanfdPreDiv12  = 1,   // CAN prescaler clock: 1/2
	CanfdPreDiv14  = 2,   // CAN prescaler clock: 1/4
	CanfdPreDiv18  = 3,   // CAN prescaler clock: 1/8
	CanfdPreDiv23  = 4,   // CAN prescaler clock: 2/3
	CanfdPreDiv13  = 5,   // CAN prescaler clock: 1/3
	CanfdPreDiv16  = 6,   // CAN prescaler clock: 1/6
	CanfdPreDiv112 = 7,   // CAN prescaler clock: 1/12
	CanfdPreDiv15  = 8,   // CAN prescaler clock: 1/5
	CanfdPreDiv110 = 9    // CAN prescaler clock: 1/10
} en_canfd_prescaler_t;


/******************************************************************************/
/* Global variable declarations ('extern', definition in C source)            */
/******************************************************************************/


/******************************************************************************/
/* Global function prototypes ('extern', definition in C source)              */
/******************************************************************************/

en_result_t Canpre_Init( en_canfd_prescaler_t enPrescale );


//@}
#ifdef __cplusplus
}
#endif

#endif // #if (defined(PDL_PERIPHERAL_CANPRE_ACTIVE))

#endif /* __CAN_PRE_H__ */
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
