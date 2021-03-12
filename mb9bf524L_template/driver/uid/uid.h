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
/** \file uid.h
 **
 ** Headerfile for unique ID functions
 **  
 ** History:
 **   - 2014-02-06  1.0  EZh  First version for FM universal PDL.
 **
 ******************************************************************************/

#ifndef __UID_H__
#define __UID_H__

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "mcu.h"
#include "pdl_user.h"

#if (defined(PDL_PERIPHERAL_UID_ACTIVE))

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
  
/**
 ******************************************************************************
 ** \defgroup UidGroup Unique ID Functions (UID)
 **
 ** Provided functions of UID module:
 ** - Uid_ReadUniqueId()
 ** - Uid_ReadUniqueId0()
 ** - Uid_ReadUniqueId1()
 ** - Uid_ReadUniqueId64()
 **
 ** Uid_ReadUniqueId() reads out the Unique ID to a pointered structure of the
 ** type #stc_unique_id_t. Uid_ReadUniqueId0() reads out the Unique ID 0
 ** register and aligns the result to the LSB. Uid_ReadUniqueId1() reads out the
 ** Unique ID 1 register. Both functions masks reserved bits to '0'.
 ** Uid_ReadUniqueId64() reads out the complete Unique ID registers aligns them
 ** to LSB and masks out reserved bits to '0'.
 **
 ******************************************************************************/
//@{

/******************************************************************************/
/* Global pre-processor symbols/macros ('#define')                            */
/******************************************************************************/

/******************************************************************************/
/* Global type definitions                                                    */
/******************************************************************************/

/**
 ******************************************************************************
 ** \brief PDL structure of Unique ID register set
 ******************************************************************************/  
 typedef struct stc_unique_id
 {
   uint32_t u32Uidr0;
   uint32_t u32Uidr1;
 } stc_unique_id_t;
 
/******************************************************************************/
/* Global variable definitions ('extern')                                     */
/******************************************************************************/ 
 
/******************************************************************************/
/* Global function prototypes (definition in C source)                        */
/******************************************************************************/

en_result_t Uid_ReadUniqueId(stc_unique_id_t *pstcUniqueId);
 
uint32_t Uid_ReadUniqueId0(void);
 
uint32_t Uid_ReadUniqueId1(void);

uint64_t Uid_ReadUniqueId64(void);
 
//@} // UidGroup

#ifdef __cplusplus
}
#endif

#endif // #if (defined(PDL_PERIPHERAL_UID_ACTIVE))

#endif /* __UID_H__ */

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
