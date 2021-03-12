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
/** \file crc.h
 **
 ** Headerfile for CRC functions
 **  
 **
 ** History:
 **   - 2014-11-18  1.0  EZh   First version for FM universal PDL.
 **
 ******************************************************************************/

#ifndef __CRC_H__
#define __CRC_H__

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_user.h"
#include "mcu.h"

#if (PDL_PERIPHERAL_CRC_AVAILABLE == PDL_OFF)
    #if (defined(PDL_PERIPHERAL_CRC_ACTIVE))
        #error "Don't enable CRC, as it is not available on this device"
    #endif
#endif

#if (defined(PDL_PERIPHERAL_CRC_ACTIVE))

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/**
 ******************************************************************************
 ** \defgroup CrcGroup Crc Functions (CRC)
 **
 ** Provided functions of CRC module:
 ** 
 ** - Crc_Init()
 ** - Crc_DeInit()
 ** - Crc_Push8()
 ** - Crc_Push16()
 ** - Crc_Push32()
 ** - Crc_ReadResult()
 **
 ** Initialize the CRC with Crc_Init(). After this there are two ways to
 ** calculate the check result:
 ** - Providing data by CPU via Crc_Push8(), Crc_Push16(), or Crc_Push32()
 ** - Using DMA
 **
 ** The result can be read by calling Crc_ReadResult().
 **
 ** Be careful with the endian.
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
 ** \brief Crc mode
 ** 
 ** To select between CRC16 and CRC32
 ******************************************************************************/
typedef enum en_crc_mode
{
    Crc16  = 0,  ///< CCITT CRC16 standard
    Crc32  = 1   ///< IEEE-802.3 CRC32 Ethernet standard
} en_crc_mode_t;  

/**
 *****************************************************************************
 ** \brief Crc configuration
 *****************************************************************************/
typedef struct stc_crc_config
{
    en_crc_mode_t enMode;               ///< See description of #en_crc_mode_t
    boolean_t     bUseDma;              ///< TRUE: DMA usage, needs DMA driver
    boolean_t     bFinalXor;            ///< TRUE: CRC result as XOR value
    boolean_t     bResultLsbFirst;      ///< CRC result bit order: TRUE: LSB first, FALSE MSB first
    boolean_t     bResultLittleEndian;  ///< CRC result byte order: TRUE: Litte endian, FLASE: big endian
    boolean_t     bDataLsbFirst;        ///< CRC feed data bit order: TRUE: LSB first, FALSE MSB first
    boolean_t     bDataLittleEndian;    ///< CRC feed data byte order: TRUE: Litte endian, FLASE: big endian
    uint32_t      u32CrcInitValue;      ///< Initial value, upper 16 bit ignored using CRC16
} stc_crc_config_t;

/******************************************************************************/
/* Local type definitions ('typedef')                                         */
/******************************************************************************/

/******************************************************************************/
/* Global variable definitions ('extern')                                     */
/******************************************************************************/

/******************************************************************************/
/* Global function prototypes (definition in C source)                        */
/******************************************************************************/

/* Init/DeInit */
extern en_result_t Crc_Init(stc_crc_config_t* pstcConfig);
extern void Crc_DeInit(void);

/* Calucuration */
extern void Crc_Push8 (uint8_t   u8DataToPush);
extern void Crc_Push16(uint16_t u16DataToPush);
extern void Crc_Push32(uint32_t u32DataToPush);

/* Get result */
extern uint32_t Crc_ReadResult(void);


//@} // CrcGroup

#ifdef __cplusplus
}
#endif

#endif /* #if (defined(PDL_PERIPHERAL_CRC_ACTIVE)) */

#endif /* __CRC_H__ */
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
