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
/** \file pcrc.h
 **
 ** Headerfile for  Programmable CRC functions
 **  
 **
 ** History:
 **   - 2015-02-11  1.0  DHo   First version for FM universal PDL.
 **
 ******************************************************************************/

#ifndef __PCRC_H__
#define __PCRC_H__

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_user.h"
#include "mcu.h"

#if (defined(PDL_PERIPHERAL_PCRC_ACTIVE))

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/**
 ******************************************************************************
 ** \defgroup PcrcGroup Programmable CRC Controller (PCRC)
 **
 ** Provided functions of PCRC module:
 ** 
 ** -  Pcrc_Init()
 ** -  Pcrc_DeInit()
 ** -  Pcrc_SetInitialValue()
 ** -  Pcrc_SetFinalXorValue()
 ** -  Pcrc_SetOutputDataFormat()
 ** -  Pcrc_SetInputDataFormat()
 ** -  Pcrc_SetInputDataSize()
 ** -  Pcrc_SetCfgTest()
 ** -  Pcrc_GetIntRequestFlag()
 ** -  Pcrc_EnableIntRequest()
 ** -  Pcrc_DisableIntRequest()
 ** -  Pcrc_EnableDmaTx()
 ** -  Pcrc_DisableDmaTx()
 ** -  Pcrc_GetLockStatus()
 ** -  Pcrc_ClrIntRequest()
 ** -  Pcrc_WriteData()
 ** -  Pcrc_ReadResult()
 **.
 ** Pcrc_Init() initializes a Programmable CRC module according the configuration of the structure 
 ** type #stc_pcrc_config_t. 
 **
 ** Pcrc_DeInit() Re-Initialisation of a Programmable CRC module.
 **
 ** Pcrc_SetInitialValue() is used to set CRC computing initial value.
 **
 ** Pcrc_SetFinalXorValue() is used to set CRC computing result XOR value.
 **
 ** Pcrc_SetOutputDataFormat() sets CRC computing output data format selected by the
 ** structure type #en_crc_data_format_t.
 **
 ** Pcrc_SetInputDataFormat() sets CRC computing input data format selected by the
 ** structure type #en_crc_data_format_t.
 **
 ** Pcrc_SetInputDataSize()  sets CRC computing input data size selected by the
 ** structure type #en_crc_input_data_size_t.
 **
 ** Pcrc_SetCfgTest()  Sets TEST field of CRC Computing Configuration Register to access it
 **
 ** Pcrc_GetIntRequestFlag() gets CRC interrupt request flag.
 **
 ** Pcrc_EnableIntRequest() enable CRC interrupt request.
 **
 ** Pcrc_DisableIntRequest() disable CRC interrupt request.
 **
 ** Pcrc_EnableDmaTx() enables the transfer request of input data for DMA.
 **
 ** Pcrc_DisableDmaTx()  disables the transfer request of input data for DMA.
 ** 
 ** Pcrc_GetLockStatus()  gets CRC computing unit state.
 ** 
 ** Pcrc_ClrIntRequest() clear CRC interrupt request.
 **
 ** Pcrc_WriteData()  writes CRC computing input data..
 ** 
 ** Pcrc_ReadResult()  reads  CRC computing output data.
 **
 ******************************************************************************/
//@{


/******************************************************************************
 * Global definitions
 ******************************************************************************/

/******************************************************************************
 * Global type definitions
 ******************************************************************************/

/**
 *****************************************************************************
 ** \brief Programmable Crc data format
 *****************************************************************************/
typedef enum en_crc_data_format
{
    MsbFirstBigEndian    = 0u,   ///< MSB-first / Big endian is selected for output format conversion.
    MsbFirstLittleEndian = 1u,   ///< MSB-first / Little endian is selected for output format conversion.
    LsbFirstBigEndian    = 2u,   ///< LSB-first / Big endian is selected for output format conversion.
    LsbFirstLittleEndian = 3u,   ///< LSB-first / Little endian is selected for output format conversion.
} en_crc_data_format_t;  

/**
 *****************************************************************************
 ** \brief Programmable Crc input data size
 *****************************************************************************/
typedef enum en_crc_input_data_size
{
    InputData8Bit  = 0u,   ///<   8-bit input data size is selected.
    InputData16Bit = 1u,   ///<  16-bit input data size is selected.
    InputData24Bit = 2u,   ///<  24-bit input data size is selected.
    InputData32Bit = 3u,   ///<  32-bit input data size is selected.
} en_crc_input_data_size_t; 

/**
 *****************************************************************************
 ** \brief Programmable Crc configuration
 *****************************************************************************/
typedef struct stc_pcrc_config
{
    uint32_t      u32GeneratorPolynomial;     ///< CRC Computing Generator Polynomial
    en_crc_data_format_t enInputFormat;       ///< Input data format    
    en_crc_data_format_t enOutputFormat;      ///< Output data format  
    en_crc_input_data_size_t enInputDataSize; ///< Input data size 
    uint32_t      u32CrcInitValue;      ///< Initial value   
    uint32_t      u32FinalXorValue;     ///< Specifies the value of XOR (exclusive OR) for final results of CRC computing.   

    boolean_t  bUseDstc;          ///< TRUE: the transfer request of input data for DMA.    
#if (PDL_INTERRUPT_ENABLE_PCRC== PDL_ON)
    boolean_t  bIrqEn;            ///< TRUE: Interrupt request enable.
    func_ptr_t pfnIrqCb;  ///<  Interrupt request callback funciton
    boolean_t bTouchNvic;      
#endif        
} stc_pcrc_config_t;

/******************************************************************************/
/* Local type definitions ('typedef')                                         */
/******************************************************************************/

/******************************************************************************/
/* Global variable definitions ('extern')                                     */
/******************************************************************************/

/******************************************************************************/
/* Global function prototypes (definition in C source)                        */
/******************************************************************************/

#if (PDL_INTERRUPT_ENABLE_PCRC == PDL_ON)
void PcrcIrqHandler(void);
#endif
en_result_t Pcrc_Init(stc_pcrc_config_t* pstcConfig);
void Pcrc_DeInit(void); 
void Pcrc_SetInitialValue(uint32_t u32InitialValue);
void Pcrc_SetFinalXorValue(uint32_t u32FinalXorValue);
void Pcrc_SetOutputDataFormat(en_crc_data_format_t enOutputDataFormat); 
void Pcrc_SetInputDataFormat(en_crc_data_format_t enInputDataFormat); 
void Pcrc_SetInputDataSize(en_crc_input_data_size_t enInputDataSize); 
void Pcrc_SetCfgTest(uint8_t u8CfgTest); 
uint8_t Pcrc_GetIntRequestFlag(void); 
void Pcrc_EnableIntRequest(void); 
void Pcrc_DisableIntRequest(void);
void Pcrc_EnableDmaTx(void); 
void Pcrc_DisableDmaTx(void); 
uint8_t Pcrc_GetLockStatus(void); 
void Pcrc_ClrIntRequest(void); 
void Pcrc_WriteData(uint32_t u32WriteData);
uint32_t Pcrc_ReadResult(void);

#ifdef __cplusplus
}
#endif

//@}

#endif /* #if (defined(PDL_PERIPHERAL_PCRC_ACTIVE)) */

#endif /* __PCRC_H__ */
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
