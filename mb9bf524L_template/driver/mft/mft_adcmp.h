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
/** \file mft_adcmp.h
 **
 ** Headerfile for MFT's ADC Start Compare Unit functions
 **  
 ** History:
 **   - 2014-11-15  0.1  EZh First version for FM universal PDL.
 **
 ******************************************************************************/

#ifndef __MFT_ADCMP_H__
#define __MFT_ADCMP_H__

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "mcu.h"
#include "pdl_user.h"

#if (defined(PDL_PERIPHERAL_MFT_ADCMP_ACTIVE))

/**
 ******************************************************************************
 ** \defgroup AdcmpGroup ADC Start Compare Unit (ADCMP)
 **
 ** Provided functions of ADCMP module:
 ** 
 ** - Mft_Adcmp_Init()
 ** - Mft_Adcmp_EnableOperation() 
 ** - Mft_Adcmp_DisableOperation()
 ** - Mft_Adcmp_WriteAcmp()
 ** - Mft_Adcmp_ReadAcmp()
 ** - Mft_Adcmp_Init_Fm3()
 ** - Mft_Adcmp_EnableOperation_Fm3()
 ** - Mft_Adcmp_DisableOperation_Fm3()
 ** - Mft_Adcmp_WriteAccp_Fm3()
 ** - Mft_Adcmp_ReadAccp_Fm3()
 ** - Mft_Adcmp_WriteAccpdn_Fm3()
 ** - Mft_Adcmp_ReadAccpdn_Fm3()
 **
 ** <b>How to use ADCMP module?</b>
 ** 
 ** Two modes including normal mode and offset mode can be configured for 
 ** ADCMP. This module should be used with ADC. 
 **
 ** Before using ADCMP, a FRT used to connect with applying ADCMP must be 
 ** initialed. For how to configure FRT, see the description in the mft_frt.h. 
 ** In the offset mode, a OCU should also be used and initialed. For how to 
 ** configure OCU, see the description in the mft_ocu.h  
 **
 ** Mft_Adcmp_Init() must be used for configuration of a ADCMP channel with 
 ** a structure of the type #stc_mft_adcmp_config_t. 
 ** 
 ** With Mft_Adcmp_WriteAcmp() the ADC start compare value is set to the value
 ** given in the parameter Mft_Adcmp_WriteAcmp#u16AcmpVal. Whether the compare
 ** value is modified directly depends on buffer function. 
 **
 ** After above setting, calling Mft_Adcmp_EnableOperation() will start ADCMP.
 ** 
 ** When stopping the OCU, use Mft_Adcmp_DisableOperation() to disable 
 ** ADCMP.
 **
 ** In addition, the module is also compatible with FM3 usage.
 **
 ** <b>How to use ADCMP module with FM3 compatibility function?</b>
 ** 
 ** Before using ADCMP, a FRT used to connect with applying ADCMP must be 
 ** initialed. For how to configure FRT, see the description in the mft_frt.h. 
 **
 ** Mft_Adcmp_Init_Fm3() must be used for configuration of a ADCMP channel with 
 ** a structure of the type #stc_mft_adcmp_config_fm3_t. 
 **
 ** With Mft_Adcmp_WriteAccp_Fm3() the ADC start up compare value is set to the 
 ** value given in the parameter Mft_Adcmp_WriteAccp_Fm3#u16AccpVal. 
 ** With Mft_Adcmp_ReadAccp_Fm3() the ADC start up compare value is read.
 **
 ** With Mft_Adcmp_WriteAccpdn_Fm3() the ADC start down compare value is set 
 ** to the value given in the parameter Mft_Adcmp_WriteAccpdn_Fm3#u16AccpVal. 
 ** With Mft_Adcmp_ReadAccpdn_Fm3() the ADC start down compare value is read.
 **
 ** Mft_Adcmp_EnableOperation_Fm3() enables ADCMP operation and 
 ** Mft_Adcmp_DisableOperation_Fm3() disables ADCMP operation. 
 **
 ******************************************************************************/ 

/******************************************************************************/
/* Global pre-processor symbols/macros ('#define')                            */
/******************************************************************************/
#define  MFT_ADCMP_CH0             0u
#define  MFT_ADCMP_CH1             1u
#define  MFT_ADCMP_CH2             2u
#define  MFT_ADCMP_CH3             3u
#define  MFT_ADCMP_CH4             4u
#define  MFT_ADCMP_CH5             5u

#define  MFT_ADCMP_CH10            0u
#define  MFT_ADCMP_CH32            1u
#define  MFT_ADCMP_CH54            2u

#define  MFT_ADCMP_CH0_FM3         MFT_ADCMP_CH10  
#define  MFT_ADCMP_CH1_FM3         MFT_ADCMP_CH32
#define  MFT_ADCMP_CH2_FM3         MFT_ADCMP_CH54

#define  MFT_ADCMP_CH_MAX          5u
#define  MFT_ADCMP_CPCH_MAX        2u
/******************************************************************************
 * Global type definitions
 ******************************************************************************/
#define stc_mftn_adcmp_t   FM_MFT_ADCMP_TypeDef

#define MFT0_ADCMP  (*(volatile stc_mftn_adcmp_t *) FM_MFT0_ADCMP_BASE)
#define MFT1_ADCMP  (*(volatile stc_mftn_adcmp_t *) FM_MFT1_ADCMP_BASE)
#define MFT2_ADCMP  (*(volatile stc_mftn_adcmp_t *) FM_MFT2_ADCMP_BASE)

/**
 ******************************************************************************
 ** \brief Define Frt channel connect to Adcmp
 ******************************************************************************/  
typedef enum en_adcmp_frt
{
    Frt0ToAdcmp    = 0u, ///< connect Frt channel 0 to Icu
    Frt1ToAdcmp    = 1u, ///< connect Frt channel 1 to Icu
    Frt2ToAdcmp    = 2u, ///< connect Frt channel 2 to Icu
    
}en_adcmp_frt_t;

/**
 ******************************************************************************
 ** \brief Define Frt channel with FM3 mode connect to Adcmp
 ******************************************************************************/  
typedef enum en_adcmp_fm3_frt
{
#if (PDL_MCU_CORE == PDL_FM3_CORE)  
    Frt0ToAdcmpFm3    = 0u, ///< connect Frt channel 0 to Icu
#endif    
    Frt1ToAdcmpFm3    = 1u, ///< connect Frt channel 1 to Icu
    Frt2ToAdcmpFm3    = 2u, ///< connect Frt channel 2 to Icu
    
}en_adcmp_frt_fm3_t;

/**
 ******************************************************************************
 ** \brief Define Adcmp buffer type
 ******************************************************************************/ 
typedef enum en_adcmp_buf
{
    AdcmpBufDisable     = 0u, ///< disable Adcmp buffer function
    AdcmpBufFrtZero     = 1u, ///< transfer buffer of ACMP and ACMC when counter value of Frt connected= 0x0000
    AdcmpBufFrtPeak     = 2u, ///< transfer buffer of ACMP and ACMC when counter value of Frt connected= TCCP
    AdcmpBufFrtZeroPeak = 3u, ///< transfer buffer of ACMP and ACMC both when counter value of Frt connected= 0x0000 and TCCP
#if (PDL_MCU_CORE == PDL_FM4_CORE)
    #if (PDL_MCU_TYPE >= PDL_FM4_TYPE3)    
    AdcmpBufFrtZeroMszcZero = 4u,   ///< transfer buffer of ACMP and ACMC when counter value of Frt connected= 0x0000 and zero value detection mask counter value is 0
    AdcmpBufFrtPeakMspcZero = 5u,   ///< transfer buffer of ACMP and ACMC when counter value of Frt connected= peak and peak value detection mask counter value is 0
    AdcmpBufFrtZeroMszcZeroOrFrtPeakMspcZero = 6u,  ///< transfer buffer of ACMP and ACMC when counter value of Frt connected= 0x0000 and zero value detection mask counter value is 0 
                                                    ///< or counter value of Frt connected= peak and peak value detection mask counter value is 0
    #endif
#endif
}en_adcmp_buf_t;

/**
 ******************************************************************************
 ** \brief Define Mft Adcmp output start trig channel
 ******************************************************************************/ 
typedef enum en_adcmp_trig_channel
{
    AdcmpStartTrig0 = 0u, ///< Outputs ADC start trigger 0
    AdcmpStartTrig1 = 1u, ///< Outputs ADC start trigger 1
    AdcmpStartTrig2 = 2u, ///< Outputs ADC start trigger 2
    AdcmpStartTrig3 = 3u, ///< Outputs ADC start trigger 3
    AdcmpStartTrig4 = 4u, ///< Outputs ADC start trigger 4
    AdcmpStartTrig5 = 5u, ///< Outputs ADC start trigger 5
    AdcmpStartTrig6 = 6u, ///< Outputs ADC start trigger 6
    AdcmpStartTrig7 = 7u, ///< Outputs ADC start trigger 7
  
}en_adcmp_start_trig_t;

/**
 ******************************************************************************
 ** \brief Define Mft Adcmp trig mode
 ******************************************************************************/
typedef enum en_adcmp_trig_sel
{
    AdcmpTrigAdc0Scan = 0u, ///< AdcmpStartTrig0
    AdcmpTrigAdc0Prio = 1u, ///< AdcmpStartTrig1
    AdcmpTrigAdc1Scan = 2u, ///< AdcmpStartTrig2
    AdcmpTrigAdc1Prio = 3u, ///< AdcmpStartTrig3
    AdcmpTrigAdc2Scan = 4u, ///< AdcmpStartTrig4
    AdcmpTrigAdc2Prio = 5u, ///< AdcmpStartTrig5
    
}en_adcmp_trig_sel_t;

/**
 ******************************************************************************
 ** \brief Define Adcmp running mode
 ******************************************************************************/ 
typedef enum en_adcmp_mode
{
    AdcmpNormalMode = 0u, ///< select Adcmp Normal mode
    AdcmpOffsetMode = 1u, ///< select Adcmp Offset mode
    
}en_adcmp_mode_t;

/**
 ******************************************************************************
 ** \brief Define Occp channel
 ******************************************************************************/ 
typedef enum en_adcmp_occp_sel
{
    AdcmpSelOccp0 = 0u, ///< select Occp0 channel
    AdcmpSelOccp1 = 1u, ///< select Occp1 channel
  
}en_adcmp_occp_sel_t;

/**
 ******************************************************************************
 ** \brief Define Mft_adcmp configure parameters
 ******************************************************************************/ 
typedef struct stc_mft_adcmp_config
{
    en_adcmp_frt_t        enFrt;         ///< configure Adcmp Frt channel
    en_adcmp_buf_t        enBuf;         ///< configure Adcmp Buffer transfer type
    en_adcmp_trig_sel_t   enTrigSel;     ///< configure Adcmp Trigger type
    en_adcmp_mode_t       enMode;        ///< configure Adcmp Running mode
    en_adcmp_occp_sel_t   enOccpSel;     ///< select Adcmp Occp channel
#if (PDL_MCU_CORE == PDL_FM4_CORE)
    #if (PDL_MCU_TYPE >= PDL_FM4_TYPE3)   
    boolean_t             bCompareFrtZeroMaskCntVal;  ///< TRUE:  Comparison is performed with the FRT zero interrupt mask counter.
                                                      ///< FALSE: Comparison is not performed with the FRT zero interrupt mask counter.
    boolean_t             bCompareFrtPeakMaskCntVal;  ///< TRUE:  Comparison is performed with the FRT peak interrupt mask counter.
                                                      ///< FALSE: Comparison is not performed with the FRT peak interrupt mask counter.
    uint8_t               u8CompareVal;               ///< Specifies the AD conversion start time by the value to be compared with the FRT interrupt mask counter.
    #endif
#endif 
}stc_mft_adcmp_config_t;

/**
 ******************************************************************************
 ** \brief Define Mft_adcmp functions
 ******************************************************************************/ 
typedef struct stc_mft_adcmp_func
{
    boolean_t   bDownEn;  ///< Enable Adcmp Down function
    boolean_t   bPeakEn;  ///< Enable Adcmp Peak function
    boolean_t   bUpEn;    ///< Enable Adcmp Up function
    boolean_t   bZeroEn;  ///< Enable Adcmp Zero function
  
}stc_mft_adcmp_func_t;
/**
 ******************************************************************************
 ** \brief Define Mft_adcmp compatible fm3 mode
 ******************************************************************************/
typedef enum en_adcmp_mode_fm3
{
    AdcmpAccpUpAccpDownFm3   = 0u, ///< Fm3 compatible mode: Accp Up and Down
    AdcmpAccpUpFm3           = 1u, ///< Fm3 compatible mode: Accp Up
    AdcmpAccpDownFm3         = 2u, ///< Fm3 compatible mode: Accp Down
    AdcmpAccpUpAccpdnDownFm3 = 3u, ///< Fm3 compatible mode: Accp up adn Accpdn Down
    
}en_adcmp_mode_fm3_t;
/**
 ******************************************************************************
 ** \brief Define Mft_adcmp compatible fm3 configure parameters
 ******************************************************************************/
typedef struct stc_mft_adcmp_config_fm3
{
    en_adcmp_frt_fm3_t     enFrt;     ///< configure Adcmp Frt channel
    en_adcmp_mode_fm3_t    enMode;    ///< configure compatible Fm3 mode
    en_adcmp_buf_t         enBuf;     ///< configure Adcmp Buffer transfer type
#if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE)      
    en_adcmp_trig_sel_t    enTrigSel; ///< Select trig mode
#else
    boolean_t bAdcScanTriggerSel;     ///< FALSE: selects the start signal of ADCMP ch.x as ADC unitx scan conversion start signal.
                                      ///< TRUE: Selects the logic OR signal of FRT ch.0 to ch.2 start signal as ADC unitx scan conversion start signal.
    boolean_t bAdcPrioTriggerSel;     ///< FALSE: selects the start signal of ADCMP ch.x as ADC unitx priority conversion start signal.
                                      ///< TRUE: Selects the logic OR signal of FRT ch.0 to ch.2 start signal as ADC unitx priority conversion start signal.
#endif    
    
}stc_mft_adcmp_config_fm3_t;
/**
 ******************************************************************************
 ** \brief Global function prototypes (definition in C source)
 ******************************************************************************/
/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
//@{
#if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE)    
/* 1. FM4/FM0+ */
/* Init */
en_result_t Mft_Adcmp_Init (volatile stc_mftn_adcmp_t *pstcMftAdcmp,  
                            uint8_t u8Ch, 
                            stc_mft_adcmp_config_t *pstcConfig);
en_result_t Mft_Adcmp_DeInit(volatile stc_mftn_adcmp_t* pstcMftAdcmp, 
                             uint8_t u8Ch);
/* Func enable/disable */
en_result_t Mft_Adcmp_EnableOperation(volatile stc_mftn_adcmp_t *pstcMftAdcmp,  
                                      uint8_t u8Ch, 
                                      stc_mft_adcmp_func_t* pstcFunc);
en_result_t Mft_Adcmp_DisableOperation(volatile stc_mftn_adcmp_t *pstcMftAdcmp,  
                                       uint8_t u8Ch, 
                                       stc_mft_adcmp_func_t* pstcFunc);
/* Count write/read */
en_result_t Mft_Adcmp_WriteAcmp(volatile stc_mftn_adcmp_t *pstcMftAdcmp, uint8_t u8Ch, 
                                uint16_t u16AcmpVal);
uint16_t    Mft_Adcmp_ReadAcmp(volatile stc_mftn_adcmp_t *pstcMftAdcmp, uint8_t u8Ch);
#endif
/* 2. FM3/FM0+/FM4 */
/* Init/De-Init */
en_result_t Mft_Adcmp_Init_Fm3(volatile stc_mftn_adcmp_t *pstcMftAdcmp, 
                               uint8_t u8FM3Ch,
                               stc_mft_adcmp_config_fm3_t *pstcConfig);
en_result_t Mft_Adcmp_DeInit_Fm3(volatile stc_mftn_adcmp_t *pstcMftAdcmp, 
                                 uint8_t u8FM3Ch);
/* Mode configuration */
en_result_t Mft_Adcmp_EnableOperation_Fm3(volatile stc_mftn_adcmp_t *pstcMftAdcmp, uint8_t u8FM3Ch);
en_result_t Mft_Adcmp_DisableOperation_Fm3(volatile stc_mftn_adcmp_t *pstcMftAdcmp, uint8_t u8FM3Ch);
/* Count write/read */
en_result_t Mft_Adcmp_WriteAccp_Fm3(volatile stc_mftn_adcmp_t *pstcMftAdcmp, uint8_t u8FM3Ch, uint16_t u16AccpVal);
uint16_t Mft_Adcmp_ReadAccp_Fm3(volatile stc_mftn_adcmp_t *pstcMftAdcmp, uint8_t u8FM3Ch);
en_result_t Mft_Adcmp_WriteAccpdn_Fm3(volatile stc_mftn_adcmp_t *pstcMftAdcmp, uint8_t u8FM3Ch, uint16_t u16AccpdnVal);
uint16_t Mft_Adcmp_ReadAccpdn_Fm3(volatile stc_mftn_adcmp_t *pstcMftAdcmp, uint8_t u8FM3Ch);

//@}
#ifdef __cplusplus
}
#endif

#endif

#endif




/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
