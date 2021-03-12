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
/** \file pdl_device.h
 **
 ** Device and package headerfile for FM Peripheral Driver Library.
 **
 ** \note This file MUST only be included in pdl_user.h!
 **  
 ** History:
 **   - 2014-09-18  0.0.1  EZh   First version for universal PDL  
 **
 ******************************************************************************/

#ifndef  __PDL_DEVICE_H__
#define  __PDL_DEVICE_H__

/**
 ******************************************************************************
 ** \brief Global device series definition
 **
 ** See pdl.h line 125 for choice list.
 **
 ** \note This definition is used for GPIO settings, interrupt vector,
 **       enumeration handling, and Type(0, 1, ...) Device handling.
 ******************************************************************************/
#define PDL_MCU_SERIES     PDL_DEVICE_SERIES_MB9BF12X   // PDL_DEVICE_SERIES_MB9BF61X  // PDL_DEVICE_SERIES_S6E1C3X


/**
 ******************************************************************************
 ** \brief Global package definition
 **
 ** See pdl.h line 168 for choice list.
 **
 ** \note This definition is used for device package settings
 ******************************************************************************/
#define PDL_MCU_PACKAGE    PDL_DEVICE_PACKAGE_MB_L                //  PDL_DEVICE_PACKAGE_MB_S //PDL_DEVICE_PACKAGE_S6_D

#endif /* __PDL_DEVICE_H__ */

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
