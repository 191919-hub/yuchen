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
/** \file workflash.h
 **
 ** Headerfile for main flash functions
 **
 ** History:
 **   - 2015-01-20 1.0  KXi First version for FM Universal PDL.
 **
 *****************************************************************************/
#ifndef _WORK_FLASH_H_
#define _WORK_FLASH_H_

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "mcu.h"
#include "pdl_user.h"

#if defined(PDL_PERIPHERAL_WORK_FLASH_ACTIVE) 
/**
 ******************************************************************************
 ** \defgroup WorkFlashGroup Work Flash (WFlash)
 **
 ** Provided functions of Work Flash operation:
 ** - WFlash_ChipErase()
 ** - WFlash_SectorErase()
 ** - WFlash_WriteData32Bit() 
 ** - WFlash_WriteData16Bit()
 **
 ** <b>What is Work Flash</b>? 
 **
 ** Work Flash have independent area, which is sperated from Main Flash, 
 ** thus the Flash operation API can be called derectly from Main Flash. 
 **
 ** <b>How to use Work Flash module</b>? 
 **
 ** WFlash_ChipErase() can erase whole chip space of Work Flash.
 ** WFlash_SectorErase() can erase one selected sector.
 ** WFlash_WriteData32Bit() writes data into Flash area with word align.
 ** WFlash_WriteData16Bit() writes data into Flash area with half-word align.
 ** 
 ******************************************************************************/ 
//@{
/******************************************************************************
 * Global type definitions
 ******************************************************************************/ 
#define     WFLASH_CODE1                 0x0AA8u
#define     WFLASH_CODE2                 0x0554u

#define     WFLASH_RET_OK                0u
#define     WFLASH_RET_INVALID_PARA      1u
#define     WFLASH_RET_ABNORMAL          2u

#define     WFLASH_CHK_TOGG_NORMAL       0u
#define     WFLASH_CHK_TOGG_ABNORMAL     1u

#define     WFLASH_CHK_DPOL_NORMAL       0u
#define     WFLASH_CHK_DPOL_ABNORMAL     1u

#define     WFLASH_CHK_DPOL_MASK         (uint16_t)0x0080u
#define     WFLASH_CHK_TOGG_MASK         (uint16_t)0x0040u
#define     WFLASH_CHK_TLOV_MASK         (uint16_t)0x0020u
   
#define     WFLASH_BASE_ADDR             (0x200C0000u)
   
/******************************************************************************/
/* Global variable definitions ('extern')                                     */
/******************************************************************************/

/******************************************************************************/
/* Global function prototypes (definition in C source)                        */
/******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif
en_result_t WFlash_ChipErase(void);
en_result_t WFlash_SectorErase(uint16_t* pu16Addr);
en_result_t WFlash_WriteData32Bit(uint32_t*  pu32Addr, 
                                 uint32_t* pu32Data, 
                                 uint32_t u32Size);
en_result_t WFlash_WriteData16Bit(uint16_t*  pu16Addr, 
                                 uint16_t* pu16Data, 
                                 uint32_t u32Size);
#ifdef __cplusplus
}
#endif 


//@}

#endif /* PERIPHERAL_AVAILABLE_WORKFLASH */

#endif /* _WORKFLASH_FM3_H_ */
