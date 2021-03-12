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
/** \file mainflash.h
 **
 ** Headerfile for main flash functions
 **
 ** History:
 **   - 2015-01-20 1.0  KXi First version for FM Universal PDL.
 **
 *****************************************************************************/

#ifndef _MAINFLASH_H_
#define _MAINFLASH_H_

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "mcu.h"
#include "pdl_user.h"

#if defined(PDL_PERIPHERAL_MAIN_FLASH_ACTIVE) 

/**
 ******************************************************************************
 ** \defgroup MainFlashGroup Main Flash (MFlash)
 **
 ** Provided functions of Main Flash operation:
 ** 
 ** - MFlash_ChipErase()
 ** - MFlash_SectorErase()
 ** - MFlash_WriteData32Bit() 
 ** - MFlash_WriteData16Bit() 
 ** - MFlash_SetDualMode()
 ** - MFlash_SetRemapMode()
 ** - MFlash_DualSectorErase()
 ** - MFlash_DualWriteData16Bit()
 ** - MFlash_DualWriteData32Bit()
 **
 ** Before using the Main Flash operation APIs, please make sure the code is 
 ** operated in RAM area. 
 ** In the IAR environment, the key word "__ramfunc" makes sure that the code 
 ** is implemented from RAM. 
 ** In the Keil environment, user has to place the code to RAM area manually.
 ** 1) Right click main_flash.c
 ** 2) [Options for main_flash.c]->[code/const]->select a RAM area
 **
 ** MFlash_ChipErase() can erase whole chip space of Main Flash, whether CR 
 ** data remains after chip erase depends on the parameter MFlash_ChipErase#bCrRemain.
 ** MFlash_SectorErase() can erase one selected sector.
 ** MFlash_WriteData32Bit() writes data into Flash area with word align, as ECC is 
 ** equipped in the Flash module. Whether data verify and ECC check is done
 ** depends on the parameter MFlash_WriteData32Bit#bVerifyAndEccCheck.
 ** MFlash_WriteData16Bit() writes data into Flash area with half word align
 ** 
 ** For the TYPE3 of FM4 product, there are two mode: Main Flash mode and Dual
 ** Flash Mode, which can be set by MFlash_SetDualMode().
 ** In Main Flash mode, MFlash_Xxx() is used to operate whole Flash, but these
 ** APIs should be located at RAM area as described above.
 ** In Dual Flash mode, Flash is divided into Main Flash area and Dual Flash area,
 ** these two independent area can access to each other independently. 
 ** MFlash_DualXxx() is used to operate the Dual Flash mode, and place MFlash_DualXxx()
 ** in the Main Flash area. 
 ** MFlash_Xxx() is used to operate the Main Flash area, and place MFlash_Xxx() in
 ** the Dual Flash area (In the IAR IDE, "__ramfunc" before each MFlash_Xxx() 
 ** can be removed in this case)
 ** 
 ** 
 ******************************************************************************/ 
//@{

/******************************************************************************/
/* Local type definitions ('typedef')                                         */
/******************************************************************************/
   
/******************************************************************************
 * Global type definitions
 ******************************************************************************/
/**
 ******************************************************************************
 ** \brief write command
 ** 
 ** To Select write command sequence by MCU type
 ******************************************************************************/ 
#if (PDL_MCU_TYPE == PDL_FM3_TYPE3) || (PDL_MCU_TYPE == PDL_FM3_TYPE7) ||(PDL_MCU_CORE == PDL_FM4_CORE)
    #define     MFLASH_CODE1                 0x0AA8
    #define     MFLASH_CODE2                 0x0554   
#elif(PDL_MCU_TYPE == PDL_FM3_TYPE10) || (PDL_MCU_TYPE == PDL_FM3_TYPE11)
    #define     MFLASH_CODE1                 0x0AA8
    #define     MFLASH_CODE2                 0x0554  
#elif (PDL_MCU_CORE == PDL_FM0P_CORE)
#if (PDL_MCU_TYPE == PDL_FM0P_TYPE3)
    #define     MFLASH_CODE1                 0x0AA8
    #define     MFLASH_CODE2                 0x0544 
#else
    #define     MFLASH_CODE1                 0x0AA8
    #define     MFLASH_CODE2                 0x0554   
#endif
#else
    #define     MFLASH_CODE1                 0x0AA8
    #define     MFLASH_CODE2                 0x0554
#endif
/**
 ******************************************************************************
 ** \brief main flash check bit mask
 ** 
 ** To Set flash check hardware sequence flag bit mask
 ******************************************************************************/       
#define     MFLASH_CHK_DPOL_MASK         (uint16_t)0x0080
#define     MFLASH_CHK_TOGG_MASK         (uint16_t)0x0040
#define     MFLASH_CHK_TLOV_MASK         (uint16_t)0x0020
/**
 ******************************************************************************
 ** \brief CR address
 ** 
 ** To Set CR address
 ******************************************************************************/   
#define     CR_DATA_ADDR                 (0x00100004)
/**
 ******************************************************************************
 ** \brief state feedback
 ** 
 ** To feedback the flash operation state 
 ******************************************************************************/ 
typedef enum en_mflash_state
{
       MFLASH_RET_OK,                           ///< OK
       MFLASH_RET_INVALID_PARA,                 ///< parameter error
       MFLASH_RET_ABNORMAL,                     ///< operation fail
       MFLASH_RET_ECCERROR                      ///< flash ECC verify error
}en_mflash_state_t;   
/**
 ******************************************************************************
 ** \brief check toggle state
 ** 
 ** To feedback the flash toggle state 
 ******************************************************************************/ 
typedef enum en_mflash_toggle
{
       MFLASH_CHK_TOGG_NORMAL,                   ///< flash toggle normal
       MFLASH_CHK_TOGG_ABNORMAL                  ///< flash toggle abnormal
}en_mflash_toggle_t; 
/**
 ******************************************************************************
 ** \brief check toggle state
 ** 
 ** To feedback the flash toggle state 
 ******************************************************************************/ 
typedef enum en_mflash_datapoll
{
       MFLASH_CHK_DPOL_NORMAL,                   ///< data poll normal
       MFLASH_CHK_DPOL_ABNORMAL                  ///< data poll abnormal
}en_mflash_datapoll_t; 
/**
 ******************************************************************************
 ** \brief check toggle state
 ** 
 ** To feedback the flash toggle state 
 ******************************************************************************/ 
typedef enum en_mflash_eccbitstat
{
       MFLASH_ECC_NORMAL,                   ///< data poll normal
       MFLASH_ECC_ABNORMAL                  ///< data poll abnormal
}en_mflash_eccbitstat_t; 

/******************************************************************************/
/* Global variable definitions ('extern')                                     */
/******************************************************************************/

/******************************************************************************/
/* Global function prototypes (definition in C source)                        */
/******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif
en_result_t MFlash_ChipErase(boolean_t bCrRemain);
en_result_t MFlash_SectorErase(uint16_t* pu16SecAddr);
en_result_t MFlash_WriteData32Bit(uint32_t*  pu32WriteAddr, 
                                  uint32_t* pu32WriteData, 
                                  uint32_t u32Size, 
                                  boolean_t bVerifyAndEccCheck);
en_result_t MFlash_WriteData16Bit(uint16_t*  pu16WriteAddr, 
                                  uint16_t* pu16WriteData, 
                                  uint32_t u32Size);
en_result_t MFlash_WriteData16Bit_Fm0Type3CrSecureArea(uint16_t*  pu16WriteAddr, 
                                  uint16_t* pu16WriteData, 
                                  uint32_t u32Size);
#if (PDL_MCU_TYPE == PDL_FM4_TYPE3)
en_result_t MFlash_SetDualMode(boolean_t bDualMode);
en_result_t MFlash_SetRemapMode(boolean_t bRemapMode);
en_result_t MFlash_DualSectorErase(uint16_t* pu16SecAddr);
en_result_t MFlash_DualWriteData32Bit(uint32_t*  pu32WriteAddr,\
                                      uint32_t* pu32WriteData, \
                                      uint32_t u32Size, \
                                      boolean_t bVerifyAndEccCheck);
en_result_t MFlash_DualWriteData16Bit(uint16_t*  pu16WriteAddr, \
                                      uint16_t* pu16WriteData, \
                                      uint32_t u32Size);
#endif
#ifdef __cplusplus
}
#endif

//@}

#endif /* PERIPHERAL_AVAILABLE_MAINFLASH */

#endif /* _MAINFLASH_FM_H_ */
