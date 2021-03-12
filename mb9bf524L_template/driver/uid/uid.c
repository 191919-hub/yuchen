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
/** \file uid.c
 **
 ** A detailed description is available at 
 ** @link UidGroup Unique ID Module description @endlink
 **
 ** History:
 **   - 2014-02-06  1.0  EZh  First version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "uid.h"

#if (defined(PDL_PERIPHERAL_UID_ACTIVE))

/**
 ******************************************************************************
 ** \addtogroup UidGroup
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
 ** \brief Read Unique ID registers as is
 **
 ** This function reads out UIDR0 and UIDR1 as is without any shift to
 ** a pointered structure of the type stc_unique_id_t.
 ** Reserved bits are masked to '0'.
 **
 ** \param  [out] pstcUniqueId     Pointer to the Unique ID structure
 **
 ** \retval Ok                     Unique ID successfully written
 ** \retval ErrorInvalidParameter  pstcUniqueId == NULL
 ******************************************************************************/
en_result_t Uid_ReadUniqueId(stc_unique_id_t *pstcUniqueId)
{
  if (NULL == pstcUniqueId)
  {
    return ErrorInvalidParameter;
  }
  
  pstcUniqueId->u32Uidr0 = (0xFFFFFFF0ul & (FM_UNIQUE_ID->UIDR0));
  pstcUniqueId->u32Uidr1 = (0x00001FFFul & (FM_UNIQUE_ID->UIDR1));
  
  return Ok;
}
   
/**
 ******************************************************************************
 ** \brief Read Unique ID registers 0 and shifts it by 4 (LSB aligned)
 **
 ** This function reads out UIDR0 and aligns the value to the LSB by shifting
 ** by 4.
 **
 ** \return uint32_t                  Unique ID 0 >> 4
 ******************************************************************************/
uint32_t Uid_ReadUniqueId0(void)
{
  return ((FM_UNIQUE_ID->UIDR0) >> 4u);
}

/**
 ******************************************************************************
 ** \brief Read Unique ID registers 1 and masks the upper 19 bits to '0'
 **
 ** This function reads out UIDR1 and masks the upper 19 bits to '0'
 **
 ** \return uint32_t                  Unique ID 1 & 0x00001FFF
 ******************************************************************************/
uint32_t Uid_ReadUniqueId1(void)
{
  return ((FM_UNIQUE_ID->UIDR1) & 0x00001FFFul);
}

/**
 ******************************************************************************
 ** \brief Read Unique ID registers 0 and 1 and merge it LSB aligned to a 64 bit
 ** value
 **
 ** \return uint64_t                  Complete unique ID LSB aligned
 ******************************************************************************/
uint64_t Uid_ReadUniqueId64(void)
{
  uint64_t u64UniqueIdMsw;
  
  // Left shift by 60 bit in two steps
  u64UniqueIdMsw = (((FM_UNIQUE_ID->UIDR1) & 0x00001FFFul) << 16ul);
  u64UniqueIdMsw <<= 12ul;
  
  u64UniqueIdMsw |= ((FM_UNIQUE_ID->UIDR0) >> 4ul);
  return u64UniqueIdMsw;
}

//@} // UidGroup

#endif // #if (defined(PDL_PERIPHERAL_UID_ACTIVE))

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
