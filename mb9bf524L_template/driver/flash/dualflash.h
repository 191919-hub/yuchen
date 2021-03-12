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
/** \file dualflash.h
 **
 ** Headerfile for main flash functions
 **
 ** History:
 **   - 2015-01-20 1.0  KXi First version for FM Universal PDL.
 **
 *****************************************************************************/
#ifndef _DUALFLASH_H_
#define _DUALFLASH_H_

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "mcu.h"
#include "pdl_user.h"

#if defined(PDL_PERIPHERAL_DUAL_FLASH_ACTIVE)

/**
 ******************************************************************************
 ** \defgroup DualFlashGroup Dual Flash (DFlash)
 **
 ** Provided functions of Dual Flash operation:
 ** 
 ** - DFlash_ChipErase()
 ** - DFlash_SectorErase()
 ** - DFlash_WriteData32Bit()
 ** - DFlash_WriteData16Bit() 
 ** - DFlash_WriteData8Bit() 
 **
 ** Dual Flash has two independent banks, which can be erase and written by 
 ** each other. Before using Daul Flash driver, make sure the Flash operation
 ** code is in different bank of the bank to be erase and written. 
 **
 ** In IAR environment, Flash code is located at 0x8000 by following way:
 **
 ** 1) First adding following line in the ICF file
 **
 ** place at address mem:0x00008000 { section .flashcode };
 **
 ** 2) Then adding the following line at each Flash operation API:
 **
 ** #pragma location=".flashcode"
 **
 ** In Keil environment, Flash code can be re-located by following way: 
 **
 ** - Define a ROM area in the "target"
 ** - Right click dual_flash.c
 ** - [Options for dual_flash.c]->[code/const]->select the ROM area defined.
 **
 ** DFlash_ChipErase() can erase whole chip space of dual Flash.
 **
 ** DFlash_SectorErase() can erase one selected sector.
 **
 ** DFlash_WriteData32Bit() writes data into Flash area with word align
 **
 ** DFlash_WriteData16Bit() writes data into Flash area with half word align
 **
 ** DFlash_WriteData8Bit() writes data into Flash area with byte align
 ** 
 ******************************************************************************/ 
//@{
/******************************************************************************/
/* Global pre-processor symbols/macros ('#define')                             */
/******************************************************************************/

#define     FLASH_LOW_BANK1_START_ADDR              (uint8_t*)(0x00000000ul)
#define     FLASH_LOW_BANK1_END_ADDR                (uint8_t*)(0x00003FFFul)
#define     FLASH_LOW_BANK2_START_ADDR              (uint8_t*)(0x00200000ul)
#define     FLASH_LOW_BANK2_END_ADDR                (uint8_t*)(0x00208000ul)
#define     FLASH_HIGH_BANK_START_ADDR              (uint8_t*)(0x00004000ul)
#define     FLASH_HIGH_BANK_END_ADDR                (uint8_t*)(0x00080000ul)

#define     DFLASH_RET_OK                0u
#define     DFLASH_RET_INVALID_PARA      1u
#define     DFLASH_RET_ABNORMAL          2u

#define     DFLASH_CHK_TOGG_NORMAL       0u
#define     DFLASH_CHK_TOGG_ABNORMAL     1u

#define     DFLASH_CHK_DPOL_NORMAL       0u
#define     DFLASH_CHK_DPOL_ABNORMAL     1u

#define     DFLASH_CHK_DPOL_MASK         (uint8_t)0x80u
#define     DFLASH_CHK_TOGG_MASK         (uint8_t)0x40u
#define     DFLASH_CHK_TLOV_MASK         (uint8_t)0x20u
/******************************************************************************/
/* Local type definitions ('typedef')                                         */
/******************************************************************************/

/******************************************************************************/
/* Global variable definitions ('extern')                                     */
/******************************************************************************/

/******************************************************************************/
/* Global function prototypes (definition in C source)                        */
/******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif
en_result_t DFlash_ChipErase(void);
en_result_t DFlash_SectorErase(uint16_t* pu16SecAddr);
en_result_t DFlash_WriteData32Bit( uint32_t*  pu32Addr, uint32_t* pu32Data,uint32_t u32Size );
en_result_t DFlash_WriteData16Bit( uint16_t*  pu16Addr, uint16_t* pu16Data, uint32_t u32Size );
en_result_t DFlash_WriteData8Bit( uint8_t*  pu8Addr, uint8_t* pu8Data, uint32_t u32Size );
#ifdef __cplusplus
}
#endif

//@}

#endif /* PERIPHERAL_AVAILABLE_DUALFLASH  */

#endif /* _DUALFLASH_H_  */
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
