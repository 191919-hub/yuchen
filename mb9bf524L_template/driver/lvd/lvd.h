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
/** \file lvd.h
 **
 ** Headerfile for LVD functions
 **  
 ** History:
 **   - 2014-12-03  1.0  EZh   First version for FM universal PDL.
 **
 ******************************************************************************/

#ifndef __LVD_H__
#define __LVD_H__

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "mcu.h"
#include "pdl_user.h"

#if (defined(PDL_PERIPHERAL_LVD_ACTIVE))

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/**
 ******************************************************************************
 ** \defgroup LvdGroup Low Voltage Detection (LVD)
 **
 ** Provided functions of LVD module:
 ** - Lvd_Init()
 ** - Lvd_DeInit()
 ** - Lvd_EnableIrqDetect()
 ** - Lvd_DisableIrqDetect()
 ** - Lvd_EnableReset()
 ** - Lvd_DisableReset()
 ** - Lvd_GetIrqStatus()
 ** - Lvd_ClrIrqStatus()
 ** - Lvd_GetIrqOperationStatus()
 ** - Lvd_GetResetOperationStatus() 
 **
 ** Lvd_Init() initialize the voltage threshold of both interrupt and reset 
 ** circuit.
 ** If a callback function is not specified when Lvd_Init() is called, NVIC is
 ** is not enabled. So user has to check interrupt by Lvd_GetIrqStatus().
 ** and clear interrupt by Lvd_ClearIrqStatus().
 ** 
 ** Lvd_DeInit() disables the LVD function. 
 ** 
 ** Lvd_EnableIrqDetect() enables LVD interrupt circuit and  
 ** Lvd_DisableIrqDetect() disable LVD interrupt circuit.
 ** 
 ** Lvd_EnableReset() enables LVD reset circuit and  
 ** Lvd_DisableReset() disable LVD reset circuit.
 ** 
 ** Lvd_GetIrqOperationStatus() is used for checking the operation status of
 ** LVD interrupt circuit.
 ** Lvd_GetResetOperationStatus() is used for checking the operation status of
 ** LVD reset circuit.
 **
 ******************************************************************************/
//@{

/******************************************************************************/
/* Global pre-processor symbols/macros ('#define')                            */
/******************************************************************************/
 
/******************************************************************************
 * Global type definitions
 ******************************************************************************/

/**
 ******************************************************************************
 ** \brief LVD ch.0 Interruption voltage settings for Low Voltage Detection
 ******************************************************************************/  
typedef enum en_lvd0_irq_detect_voltage
{
#if (PDL_MCU_TYPE == PDL_FM0P_TYPE1) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE9)  || (PDL_MCU_TYPE == PDL_FM3_TYPE10) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE11) || (PDL_MCU_TYPE == PDL_FM3_TYPE12)
    Lvd0IrqDetectVoltage280 = 3u,   ///< Interrupt when voltage is vicinity of 2.80 volts
    Lvd0IrqDetectVoltage300 = 4u,   ///< Interrupt when voltage is vicinity of 3.00 volts
    Lvd0IrqDetectVoltage320 = 5u,   ///< Interrupt when voltage is vicinity of 3.20 volts
    Lvd0IrqDetectVoltage360 = 6u,   ///< Interrupt when voltage is vicinity of 3.60 volts
    Lvd0IrqDetectVoltage370 = 7u,   ///< Interrupt when voltage is vicinity of 3.70 volts
    Lvd0IrqDetectVoltage400 = 8u,   ///< Interrupt when voltage is vicinity of 4.00 volts
    Lvd0IrqDetectVoltage410 = 9u,   ///< Interrupt when voltage is vicinity of 4.10 volts
    Lvd0IrqDetectVoltage420 = 10u,  ///< Interrupt when voltage is vicinity of 4.20 volts
    Lvd0IrqDetectVoltageMax,        ///< Max index of LVD interrupt voltage
#elif (PDL_MCU_TYPE == PDL_FM4_TYPE1) || (PDL_MCU_TYPE == PDL_FM4_TYPE2) || (PDL_MCU_TYPE == PDL_FM4_TYPE6)
    Lvd0IrqDetectVoltage280 = 7u,   ///< Interrupt when voltage is vicinity of 2.80 volts
    Lvd0IrqDetectVoltage300 = 4u,   ///< Interrupt when voltage is vicinity of 3.00 volts
    Lvd0IrqDetectVoltage320 = 12u,  ///< Interrupt when voltage is vicinity of 3.20 volts
    Lvd0IrqDetectVoltage360 = 15u,  ///< Interrupt when voltage is vicinity of 3.60 volts
    Lvd0IrqDetectVoltage370 = 14u,  ///< Interrupt when voltage is vicinity of 3.70 volts
    Lvd0IrqDetectVoltage400 = 9u,   ///< Interrupt when voltage is vicinity of 4.00 volts
    Lvd0IrqDetectVoltage410 = 8u,   ///< Interrupt when voltage is vicinity of 4.10 volts
    Lvd0IrqDetectVoltage420 = 24u,  ///< Interrupt when voltage is vicinity of 4.20 volts
    Lvd0IrqDetectVoltageMax,        ///< Max index of LVD interrupt voltage
#elif (PDL_MCU_TYPE == PDL_FM4_TYPE3) || (PDL_MCU_TYPE == PDL_FM4_TYPE4) || (PDL_MCU_TYPE == PDL_FM4_TYPE5) 
    Lvd0IrqDetectVoltage290 = 7u,   ///< Interrupt when voltage is vicinity of 2.90 volts
    Lvd0IrqDetectVoltage310 = 4u,   ///< Interrupt when voltage is vicinity of 3.10 volts
    Lvd0IrqDetectVoltage330 = 12u,  ///< Interrupt when voltage is vicinity of 3.30 volts
    Lvd0IrqDetectVoltage380 = 15u,  ///< Interrupt when voltage is vicinity of 3.80 volts
    Lvd0IrqDetectVoltage390 = 14u,  ///< Interrupt when voltage is vicinity of 3.90 volts
    Lvd0IrqDetectVoltage420 = 9u,   ///< Interrupt when voltage is vicinity of 4.20 volts
    Lvd0IrqDetectVoltage430 = 8u,   ///< Interrupt when voltage is vicinity of 4.30 volts
    Lvd0IrqDetectVoltage440 = 24u,  ///< Interrupt when voltage is vicinity of 4.40 volts   
    Lvd0IrqDetectVoltageMax,        ///< Max index of LVD interrupt voltage
#elif (PDL_MCU_TYPE == PDL_FM3_TYPE0) || (PDL_MCU_TYPE == PDL_FM3_TYPE1) || \
      (PDL_MCU_TYPE == PDL_FM3_TYPE2) || (PDL_MCU_TYPE == PDL_FM3_TYPE4) || \
      (PDL_MCU_TYPE == PDL_FM3_TYPE5)  
    Lvd0IrqDetectVoltage280 = 0u,   ///< Interrupt when voltage is vicinity of 2.80 volts
    Lvd0IrqDetectVoltage300 = 1u,   ///< Interrupt when voltage is vicinity of 3.00 volts
    Lvd0IrqDetectVoltage320 = 2u,   ///< Interrupt when voltage is vicinity of 3.20 volts
    Lvd0IrqDetectVoltage360 = 3u,   ///< Interrupt when voltage is vicinity of 3.60 volts
    Lvd0IrqDetectVoltage370 = 4u,   ///< Interrupt when voltage is vicinity of 3.70 volts
    Lvd0IrqDetectVoltage400 = 7u,   ///< Interrupt when voltage is vicinity of 4.00 volts
    Lvd0IrqDetectVoltage410 = 8u,   ///< Interrupt when voltage is vicinity of 4.10 volts
    Lvd0IrqDetectVoltage420 = 9u,   ///< Interrupt when voltage is vicinity of 4.20 volts    
    Lvd0IrqDetectVoltageMax,        ///< Max index of LVD interrupt voltage
#elif (PDL_MCU_TYPE == PDL_FM3_TYPE3) || (PDL_MCU_TYPE == PDL_FM3_TYPE7)
    Lvd0IrqDetectVoltage200 = 0u,   ///< Interrupt when voltage is vicinity of 2.00 volts
    Lvd0IrqDetectVoltage210 = 1u,   ///< Interrupt when voltage is vicinity of 2.10 volts  
    Lvd0IrqDetectVoltage220 = 2u,   ///< Interrupt when voltage is vicinity of 2.20 volts
    Lvd0IrqDetectVoltage230 = 3u,   ///< Interrupt when voltage is vicinity of 2.30 volts
    Lvd0IrqDetectVoltage240 = 4u,   ///< Interrupt when voltage is vicinity of 2.40 volts
    Lvd0IrqDetectVoltage250 = 5u,   ///< Interrupt when voltage is vicinity of 2.50 volts
    Lvd0IrqDetectVoltage260 = 6u,   ///< Interrupt when voltage is vicinity of 2.60 volts
    Lvd0IrqDetectVoltage280 = 7u,   ///< Interrupt when voltage is vicinity of 2.80 volts
    Lvd0IrqDetectVoltage300 = 8u,   ///< Interrupt when voltage is vicinity of 3.00 volts
    Lvd0IrqDetectVoltage320 = 9u,   ///< Interrupt when voltage is vicinity of 3.20 volts
    Lvd0IrqDetectVoltage360 = 10u,  ///< Interrupt when voltage is vicinity of 3.60 volts
    Lvd0IrqDetectVoltage370 = 11u,  ///< Interrupt when voltage is vicinity of 3.70 volts
    Lvd0IrqDetectVoltage400 = 12u,  ///< Interrupt when voltage is vicinity of 4.00 volts
    Lvd0IrqDetectVoltage410 = 13u,  ///< Interrupt when voltage is vicinity of 4.10 volts
    Lvd0IrqDetectVoltage420 = 14u,  ///< Interrupt when voltage is vicinity of 4.20 volts
    Lvd0IrqDetectVoltageMax,        ///< Max index of LVD interrupt voltage
#elif (PDL_MCU_TYPE == PDL_FM3_TYPE6) || (PDL_MCU_TYPE == PDL_FM3_TYPE8)
    Lvd0IrqDetectVoltage170 = 4u,   ///< Interrupt when voltage is vicinity of 1.70 volts
    Lvd0IrqDetectVoltage175 = 5u,   ///< Interrupt when voltage is vicinity of 1.75 volts  
    Lvd0IrqDetectVoltage180 = 6u,   ///< Interrupt when voltage is vicinity of 1.80 volts
    Lvd0IrqDetectVoltage185 = 7u,   ///< Interrupt when voltage is vicinity of 1.85 volts
    Lvd0IrqDetectVoltage190 = 8u,   ///< Interrupt when voltage is vicinity of 1.90 volts
    Lvd0IrqDetectVoltage195 = 9u,   ///< Interrupt when voltage is vicinity of 1.95 volts
    Lvd0IrqDetectVoltage200 = 10u,  ///< Interrupt when voltage is vicinity of 2.00 volts
    Lvd0IrqDetectVoltage205 = 11u,  ///< Interrupt when voltage is vicinity of 2.05 volts
    Lvd0IrqDetectVoltage250 = 12u,  ///< Interrupt when voltage is vicinity of 2.50 volts    
    Lvd0IrqDetectVoltage260 = 13u,  ///< Interrupt when voltage is vicinity of 2.60 volts
    Lvd0IrqDetectVoltage270 = 14u,  ///< Interrupt when voltage is vicinity of 2.70 volts
    Lvd0IrqDetectVoltage280 = 15u,  ///< Interrupt when voltage is vicinity of 2.80 volts
    Lvd0IrqDetectVoltage290 = 16u,  ///< Interrupt when voltage is vicinity of 2.90 volts
    Lvd0IrqDetectVoltage300 = 17u,  ///< Interrupt when voltage is vicinity of 3.00 volts
    Lvd0IrqDetectVoltage310 = 18u,  ///< Interrupt when voltage is vicinity of 3.10 volts
    Lvd0IrqDetectVoltage320 = 19u,  ///< Interrupt when voltage is vicinity of 3.20 volts
    Lvd0IrqDetectVoltageMax,        ///< Max index of LVD interrupt voltage
#elif (PDL_MCU_TYPE == PDL_FM0P_TYPE2) || (PDL_MCU_TYPE == PDL_FM0P_TYPE3)
    Lvd0IrqDetectVoltage150 = 0u,   ///< Interrupt when voltage is vicinity of 1.50 volts
    Lvd0IrqDetectVoltage155 = 1u,   ///< Interrupt when voltage is vicinity of 1.55 volts  
    Lvd0IrqDetectVoltage160 = 2u,   ///< Interrupt when voltage is vicinity of 1.60 volts
    Lvd0IrqDetectVoltage165 = 3u,   ///< Interrupt when voltage is vicinity of 1.65 volts
    Lvd0IrqDetectVoltage170 = 4u,   ///< Interrupt when voltage is vicinity of 1.70 volts
    Lvd0IrqDetectVoltage175 = 5u,   ///< Interrupt when voltage is vicinity of 1.75 volts  
    Lvd0IrqDetectVoltage180 = 6u,   ///< Interrupt when voltage is vicinity of 1.80 volts
    Lvd0IrqDetectVoltage185 = 7u,   ///< Interrupt when voltage is vicinity of 1.85 volts
    Lvd0IrqDetectVoltage190 = 8u,   ///< Interrupt when voltage is vicinity of 1.90 volts
    Lvd0IrqDetectVoltage195 = 9u,   ///< Interrupt when voltage is vicinity of 1.95 volts
    Lvd0IrqDetectVoltage200 = 10u,  ///< Interrupt when voltage is vicinity of 2.00 volts
    Lvd0IrqDetectVoltage205 = 11u,  ///< Interrupt when voltage is vicinity of 2.05 volts
    Lvd0IrqDetectVoltage250 = 12u,  ///< Interrupt when voltage is vicinity of 2.50 volts    
    Lvd0IrqDetectVoltage260 = 13u,  ///< Interrupt when voltage is vicinity of 2.60 volts
    Lvd0IrqDetectVoltage270 = 14u,  ///< Interrupt when voltage is vicinity of 2.70 volts
    Lvd0IrqDetectVoltage280 = 15u,  ///< Interrupt when voltage is vicinity of 2.80 volts
    Lvd0IrqDetectVoltage290 = 16u,  ///< Interrupt when voltage is vicinity of 2.90 volts
    Lvd0IrqDetectVoltage300 = 17u,  ///< Interrupt when voltage is vicinity of 3.00 volts
    Lvd0IrqDetectVoltage310 = 18u,  ///< Interrupt when voltage is vicinity of 3.10 volts
    Lvd0IrqDetectVoltage320 = 19u,  ///< Interrupt when voltage is vicinity of 3.20 volts
    Lvd0IrqDetectVoltageMax,        ///< Max index of LVD interrupt voltage
#endif      
} en_lvd0_irq_detect_voltage_t;

/**
 ******************************************************************************
 ** \brief LVD ch.0 Interruption release voltage settings for Low Voltage Detection
 ******************************************************************************/  
#if (PDL_MCU_TYPE == PDL_FM0P_TYPE2)
typedef enum en_lvd0_irq_release_voltage
{
    Lvd0IrqReleaseVoltage155 = 0u,   ///< Interrupt when voltage is vicinity of 1.55 volts  
    Lvd0IrqReleaseVoltage160 = 1u,   ///< Interrupt when voltage is vicinity of 1.60 volts
    Lvd0IrqReleaseVoltage165 = 2u,   ///< Interrupt when voltage is vicinity of 1.65 volts
    Lvd0IrqReleaseVoltage170 = 3u,   ///< Interrupt when voltage is vicinity of 1.70 volts
    Lvd0IrqReleaseVoltage175 = 4u,   ///< Interrupt when voltage is vicinity of 1.75 volts  
    Lvd0IrqReleaseVoltage180 = 5u,   ///< Interrupt when voltage is vicinity of 1.80 volts
    Lvd0IrqReleaseVoltage185 = 6u,   ///< Interrupt when voltage is vicinity of 1.85 volts
    Lvd0IrqReleaseVoltage190 = 7u,   ///< Interrupt when voltage is vicinity of 1.90 volts
    Lvd0IrqReleaseVoltage195 = 8u,   ///< Interrupt when voltage is vicinity of 1.95 volts
    Lvd0IrqReleaseVoltage200 = 9u,   ///< Interrupt when voltage is vicinity of 2.00 volts
    Lvd0IrqReleaseVoltage205 = 10u,  ///< Interrupt when voltage is vicinity of 2.05 volts
    Lvd0IrqReleaseVoltage250 = 11u,  ///< Interrupt when voltage is vicinity of 2.50 volts    
    Lvd0IrqReleaseVoltage260 = 12u,  ///< Interrupt when voltage is vicinity of 2.60 volts
    Lvd0IrqReleaseVoltage270 = 13u,  ///< Interrupt when voltage is vicinity of 2.70 volts
    Lvd0IrqReleaseVoltage280 = 14u,  ///< Interrupt when voltage is vicinity of 2.80 volts
    Lvd0IrqReleaseVoltage290 = 15u,  ///< Interrupt when voltage is vicinity of 2.90 volts
    Lvd0IrqReleaseVoltage300 = 16u,  ///< Interrupt when voltage is vicinity of 3.00 volts
    Lvd0IrqReleaseVoltage310 = 17u,  ///< Interrupt when voltage is vicinity of 3.10 volts
    Lvd0IrqReleaseVoltage320 = 18u,  ///< Interrupt when voltage is vicinity of 3.20 volts
    Lvd0IrqReleaseVoltage330 = 19u,  ///< Interrupt when voltage is vicinity of 3.20 volts
    Lvd0IrqReleaseVoltageMax,        ///< Max index of LVD interrupt voltage
    
}en_lvd0_irq_release_voltage_t;
#endif


#if (PDL_MCU_TYPE == PDL_FM0P_TYPE1) ||  \
    (PDL_MCU_TYPE == PDL_FM3_TYPE3) || (PDL_MCU_TYPE == PDL_FM3_TYPE7) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE6) || (PDL_MCU_TYPE == PDL_FM3_TYPE8) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE9) || (PDL_MCU_TYPE == PDL_FM3_TYPE10) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE11) || (PDL_MCU_TYPE == PDL_FM3_TYPE12) 
/**
 ******************************************************************************
 ** \brief LVD ch.0 Reset voltage settings for Low Voltage Detection
 ******************************************************************************/
typedef enum en_lvd_reset_voltage
{
#if (PDL_MCU_TYPE == PDL_FM0P_TYPE1) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE9)  || (PDL_MCU_TYPE == PDL_FM3_TYPE10) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE11) || (PDL_MCU_TYPE == PDL_FM3_TYPE12)
    LvdResetVoltage245 = 0u,   ///< Interrupt when voltage is vicinity of 2.45 volts
    LvdResetVoltage260 = 1u,   ///< Interrupt when voltage is vicinity of 2.60 volts
    LvdResetVoltage270 = 2u,   ///< Interrupt when voltage is vicinity of 2.70 volts
    LvdResetVoltage280 = 3u,   ///< Interrupt when voltage is vicinity of 2.80 volts
    LvdResetVoltage300 = 4u,   ///< Interrupt when voltage is vicinity of 3.00 volts
    LvdResetVoltage320 = 5u,   ///< Interrupt when voltage is vicinity of 3.20 volts
    LvdResetVoltage360 = 6u,   ///< Interrupt when voltage is vicinity of 3.60 volts
    LvdResetVoltage370 = 7u,   ///< Interrupt when voltage is vicinity of 3.70 volts
    LvdResetVoltage400 = 8u,   ///< Interrupt when voltage is vicinity of 4.00 volts
    LvdResetVoltage410 = 9u,   ///< Interrupt when voltage is vicinity of 4.10 volts
    LvdResetVoltage420 = 10u,  ///< Interrupt when voltage is vicinity of 4.20 volts
    LvdResetVoltageMax,        ///< Max index of LVD reset voltage
#elif (PDL_MCU_TYPE == PDL_FM3_TYPE3) || (PDL_MCU_TYPE == PDL_FM3_TYPE7)
    Lvd0ResetVoltage153 = 1u,   ///< Interrupt when voltage is vicinity of 1.53 volts
    Lvd0ResetVoltage193 = 4u,   ///< Interrupt when voltage is vicinity of 1.93 volts
    Lvd0ResetVoltageMax,        ///< Max index of LVD reset voltage
#elif (PDL_MCU_TYPE == PDL_FM3_TYPE6) || (PDL_MCU_TYPE == PDL_FM3_TYPE8)
    LvdResetVoltage150 = 0u,   ///< Interrupt when voltage is vicinity of 1.50 volts
    LvdResetVoltage155 = 1u,   ///< Interrupt when voltage is vicinity of 1.55 volts
    LvdResetVoltage160 = 2u,   ///< Interrupt when voltage is vicinity of 1.60 volts
    LvdResetVoltage165 = 3u,   ///< Interrupt when voltage is vicinity of 1.65 volts
    LvdResetVoltage170 = 4u,   ///< Interrupt when voltage is vicinity of 1.70 volts
    LvdResetVoltage175 = 5u,   ///< Interrupt when voltage is vicinity of 1.75 volts
    LvdResetVoltage180 = 6u,   ///< Interrupt when voltage is vicinity of 1.80 volts
    LvdResetVoltage185 = 7u,   ///< Interrupt when voltage is vicinity of 1.85 volts
    LvdResetVoltage190 = 8u,   ///< Interrupt when voltage is vicinity of 1.90 volts
    LvdResetVoltage195 = 9u,   ///< Interrupt when voltage is vicinity of 1.95 volts
    LvdResetVoltage200 = 10u,  ///< Interrupt when voltage is vicinity of 2.00 volts
    LvdResetVoltage205 = 11u,  ///< Interrupt when voltage is vicinity of 2.05 volts
    LvdResetVoltage250 = 12u,  ///< Interrupt when voltage is vicinity of 2.50 volts    
    LvdResetVoltage260 = 13u,  ///< Interrupt when voltage is vicinity of 2.60 volts    
    LvdResetVoltage270 = 14u,  ///< Interrupt when voltage is vicinity of 2.70 volts    
    LvdResetVoltage280 = 15u,  ///< Interrupt when voltage is vicinity of 2.80 volts    
    LvdResetVoltage290 = 16u,  ///< Interrupt when voltage is vicinity of 2.90 volts
    LvdResetVoltage300 = 17u,  ///< Interrupt when voltage is vicinity of 3.00 volts    
    LvdResetVoltage310 = 18u,  ///< Interrupt when voltage is vicinity of 3.10 volts 
    LvdResetVoltage320 = 19u,  ///< Interrupt when voltage is vicinity of 3.20 volts  
    LvdResetVoltageMax,        ///< Max index of LVD reset voltage
#endif  
    
}en_lvd_reset_voltage_t;
#endif

#if (PDL_MCU_TYPE == PDL_FM0P_TYPE2)
/**
 ******************************************************************************
 ** \brief LVD ch.1 Interruption voltage settings for Low Voltage Detection
 ******************************************************************************/  
typedef enum en_lvd1_irq_detect_voltage
{
    Lvd1IrqDetectVoltage150 = 0u,   ///< Interrupt when voltage is vicinity of 1.50 volts
    Lvd1IrqDetectVoltage155 = 1u,   ///< Interrupt when voltage is vicinity of 1.55 volts  
    Lvd1IrqDetectVoltage160 = 2u,   ///< Interrupt when voltage is vicinity of 1.60 volts
    Lvd1IrqDetectVoltage165 = 3u,   ///< Interrupt when voltage is vicinity of 1.65 volts
    Lvd1IrqDetectVoltage170 = 4u,   ///< Interrupt when voltage is vicinity of 1.70 volts
    Lvd1IrqDetectVoltage175 = 5u,   ///< Interrupt when voltage is vicinity of 1.75 volts  
    Lvd1IrqDetectVoltage180 = 6u,   ///< Interrupt when voltage is vicinity of 1.80 volts
    Lvd1IrqDetectVoltage185 = 7u,   ///< Interrupt when voltage is vicinity of 1.85 volts
    Lvd1IrqDetectVoltage190 = 8u,   ///< Interrupt when voltage is vicinity of 1.90 volts
    Lvd1IrqDetectVoltage195 = 9u,   ///< Interrupt when voltage is vicinity of 1.95 volts
    Lvd1IrqDetectVoltage200 = 10u,  ///< Interrupt when voltage is vicinity of 2.00 volts
    Lvd1IrqDetectVoltage205 = 11u,  ///< Interrupt when voltage is vicinity of 2.05 volts
    Lvd1IrqDetectVoltage250 = 12u,  ///< Interrupt when voltage is vicinity of 2.50 volts    
    Lvd1IrqDetectVoltage260 = 13u,  ///< Interrupt when voltage is vicinity of 2.60 volts
    Lvd1IrqDetectVoltage270 = 14u,  ///< Interrupt when voltage is vicinity of 2.70 volts
    Lvd1IrqDetectVoltage280 = 15u,  ///< Interrupt when voltage is vicinity of 2.80 volts
    Lvd1IrqDetectVoltage290 = 16u,  ///< Interrupt when voltage is vicinity of 2.90 volts
    Lvd1IrqDetectVoltage300 = 17u,  ///< Interrupt when voltage is vicinity of 3.00 volts
    Lvd1IrqDetectVoltage310 = 18u,  ///< Interrupt when voltage is vicinity of 3.10 volts
    Lvd1IrqDetectVoltage320 = 19u,  ///< Interrupt when voltage is vicinity of 3.20 volts
    Lvd1IrqDetectVoltageMax,        ///< Max index of LVD interrupt voltage
 
} en_lvd1_irq_detect_voltage_t;
#endif    

/**
 ******************************************************************************
 ** \brief LVD ch.1 Interruption release voltage settings for Low Voltage Detection
 ******************************************************************************/  
#if (PDL_MCU_TYPE == PDL_FM0P_TYPE2)
typedef enum en_lvd1_irq_release_voltage
{
    Lvd1IrqReleaseVoltage155 = 0u,   ///< Interrupt when voltage is vicinity of 1.55 volts  
    Lvd1IrqReleaseVoltage160 = 1u,   ///< Interrupt when voltage is vicinity of 1.60 volts
    Lvd1IrqReleaseVoltage165 = 2u,   ///< Interrupt when voltage is vicinity of 1.65 volts
    Lvd1IrqReleaseVoltage170 = 3u,   ///< Interrupt when voltage is vicinity of 1.70 volts
    Lvd1IrqReleaseVoltage175 = 4u,   ///< Interrupt when voltage is vicinity of 1.75 volts  
    Lvd1IrqReleaseVoltage180 = 5u,   ///< Interrupt when voltage is vicinity of 1.80 volts
    Lvd1IrqReleaseVoltage185 = 6u,   ///< Interrupt when voltage is vicinity of 1.85 volts
    Lvd1IrqReleaseVoltage190 = 7u,   ///< Interrupt when voltage is vicinity of 1.90 volts
    Lvd1IrqReleaseVoltage195 = 8u,   ///< Interrupt when voltage is vicinity of 1.95 volts
    Lvd1IrqReleaseVoltage200 = 9u,   ///< Interrupt when voltage is vicinity of 2.00 volts
    Lvd1IrqReleaseVoltage205 = 10u,  ///< Interrupt when voltage is vicinity of 2.05 volts
    Lvd1IrqReleaseVoltage250 = 11u,  ///< Interrupt when voltage is vicinity of 2.50 volts    
    Lvd1IrqReleaseVoltage260 = 12u,  ///< Interrupt when voltage is vicinity of 2.60 volts
    Lvd1IrqReleaseVoltage270 = 13u,  ///< Interrupt when voltage is vicinity of 2.70 volts
    Lvd1IrqReleaseVoltage280 = 14u,  ///< Interrupt when voltage is vicinity of 2.80 volts
    Lvd1IrqReleaseVoltage290 = 15u,  ///< Interrupt when voltage is vicinity of 2.90 volts
    Lvd1IrqReleaseVoltage300 = 16u,  ///< Interrupt when voltage is vicinity of 3.00 volts
    Lvd1IrqReleaseVoltage310 = 17u,  ///< Interrupt when voltage is vicinity of 3.10 volts
    Lvd1IrqReleaseVoltage320 = 18u,  ///< Interrupt when voltage is vicinity of 3.20 volts
    Lvd1IrqReleaseVoltage330 = 19u,  ///< Interrupt when voltage is vicinity of 3.20 volts
    Lvd1IrqReleaseVoltageMax,        ///< Max index of LVD interrupt voltage
    
}en_lvd1_irq_release_voltage_t;
#endif

/**
 ******************************************************************************
 ** \brief Low Voltage Detection internal data
 ******************************************************************************/ 
typedef struct stc_lvd_intern_data
{
    func_ptr_t  pfnIrqCallback[2];    ///< LVD interrupt callback function
} stc_lvd_intern_data_t ;

/**
 ******************************************************************************
 ** \brief Clock Supervisor configuration
 ** 
 ** The Clock Supervisor configuration settings
 ******************************************************************************/
typedef struct stc_lvd_config
{
    en_lvd0_irq_detect_voltage_t   enLvd0IrqDetectVoltage;     ///< LVD ch.0 interrupt detect voltage, see #en_lvd0_irq_detect_voltage_t for details.
#if (PDL_MCU_TYPE == PDL_FM0P_TYPE2)    
    en_lvd0_irq_release_voltage_t  enLvd0IrqReleaseVoltage;    ///< LVD ch.0 interrupt release voltage, see #en_lvd0_irq_release_voltage_t for details.
    boolean_t bLvd0ReleaseVoltageEnable;
#endif    
#if (PDL_MCU_TYPE == PDL_FM0P_TYPE1) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE3) || (PDL_MCU_TYPE == PDL_FM3_TYPE7) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE6) || (PDL_MCU_TYPE == PDL_FM3_TYPE8) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE9) || (PDL_MCU_TYPE == PDL_FM3_TYPE10) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE11) || (PDL_MCU_TYPE == PDL_FM3_TYPE12)     
    en_lvd_reset_voltage_t enLvdResetVoltage;  ///< LVD reset voltage, see #en_lvd_reset_voltage_t for details.
#endif      
#if (PDL_MCU_TYPE == PDL_FM0P_TYPE2)       
    en_lvd1_irq_detect_voltage_t  enLvd1IrqDetectVoltage;   ///< LVD ch.1 interrupt detect voltage, see #en_lvd1_irq_detect_voltage_t for details.
    en_lvd1_irq_release_voltage_t enLvd1IrqReleaseVoltage;  ///< LVD ch.1 interrupt release voltage, see #en_lvd1_irq_detect_voltage_t for details.
    boolean_t bLvd1ReleaseVoltageEnable;
#endif    
    func_ptr_t           pfnIrqCallback[2];   ///< LVD interrupt callback function
} stc_lvd_config_t;

/******************************************************************************/
/* Global variable definitions ('extern')                                     */
/******************************************************************************/

/******************************************************************************/
/* Global function prototypes (definition in C source)                        */
/******************************************************************************/
/* LVD Interupt IRQ */
void Lvd_IrqHandler(void);
/* Init/De-Init */
en_result_t Lvd_Init(stc_lvd_config_t* pstcConfig);
en_result_t Lvd_DeInit(void);
/* Enable/Disable reset/interrupt */
en_result_t Lvd_EnableIrqDetect(uint8_t u8Channel);
en_result_t Lvd_DisableIrqDetect(uint8_t u8Channel);
#if (PDL_MCU_TYPE == PDL_FM0P_TYPE1) || (PDL_MCU_TYPE == PDL_FM0P_TYPE2) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE3)  || (PDL_MCU_TYPE == PDL_FM3_TYPE7) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE6)  || (PDL_MCU_TYPE == PDL_FM3_TYPE8) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE9)  || (PDL_MCU_TYPE == PDL_FM3_TYPE10) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE11) || (PDL_MCU_TYPE == PDL_FM3_TYPE12)   
en_result_t Lvd_EnableReset(void);
en_result_t Lvd_DisableReset(void);
#endif
/* Enable/Disable reset/interrupt */
boolean_t Lvd_GetIrqStatus(uint8_t u8Channel);
en_result_t Lvd_ClrIrqStatus(uint8_t u8Channel);
/* Get interrupt/reset circuit status */
boolean_t Lvd_GetIrqOperationStatus(uint8_t u8Channel);
boolean_t Lvd_GetResetOperationStatus(void);
//@} // LvdGroup

#ifdef __cplusplus
}
#endif

#endif /* #if (defined(PDL_PERIPHERAL_LVD_ACTIVE)) */

#endif /* __LVD_H__ */
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
