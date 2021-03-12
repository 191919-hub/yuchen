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
/******************************************************************************
 ** \file   mcu.h
 **
 ** Header File for device dependent includes
 **
 ** History:
 ** 2011-11-25 V1.00 EH first version
 **
 ******************************************************************************/

#ifndef __MCU_H__
#define __MCU_H__

#include "pdl.h"

/**
 ******************************************************************************
 ** \brief MCU header file include
 ******************************************************************************/
#if (PDL_MCU_CORE == PDL_FM3_CORE)
    #include "fm3.h"
#elif (PDL_MCU_CORE == PDL_FM0P_CORE)
    #include "fm0p.h"
#elif (PDL_MCU_CORE == PDL_FM4_CORE)
    #include "fm4.h"
#else
    #error No valid MCU core found!
#endif

/**
 ******************************************************************************
 ** \brief MCU system start-up header file include
 **
 ******************************************************************************/
#if (PDL_MCU_CORE == PDL_FM3_CORE)
    #include "system_fm3.h"
#elif (PDL_MCU_CORE == PDL_FM0P_CORE)
    #include "system_fm0p.h"
#elif (PDL_MCU_CORE == PDL_FM4_CORE)
    #include "system_fm4.h"
#else
    #error No valid MCU core found!
#endif

#endif // #ifndef __MCU_H__
