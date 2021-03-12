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
/** \file pdl.h
 **
 ** Common headerfile for FM Peripheral Driver Library
 **
 ** History:
 **   - 2014-09-16  0.1  EZh  First version for FM universal PDL.
 **
 ******************************************************************************/

#ifndef __PDL_H__
#define __PDL_H__

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "base_types.h"

/* C binding of definitions if building with C++ compiler                     */
#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************/
/* Global pre-processor symbols/macros ('#define')                            */
/* Macro for initializing local structures to zero                            */
/******************************************************************************/
#define PDL_ZERO_STRUCT(x) pdl_memclr((uint8_t*)&(x), (uint32_t)(sizeof(x)))

/**
 ******************************************************************************
 ** All definitions needed for pdl_user.h are stated here
 ******************************************************************************/
#define PDL_ON            1u    ///< Switches a feature on
#define PDL_OFF           0u    ///< Switches a feature off

/**
 ******************************************************************************
 ** Global Device type definitions
 ** Note that an existing definition does not implicitly means full device
 ** type support of this library!
 ******************************************************************************/
// FM0+ device type  
#define PDL_FM0P_TYPE0  0u    ///< FM0+ device type0
#define PDL_FM0P_TYPE1  10u   ///< FM0+ device type1
#define PDL_FM0P_TYPE2  20u   ///< FM0+ device type2
#define PDL_FM0P_TYPE3  30u   ///< FM0+ device type3
#define PDL_FM0P_TYPE4  40u   ///< FM0+ device type4
#define PDL_FM0P_TYPE5  50u   ///< FM0+ device type5
#define PDL_FM0P_TYPE6  60u   ///< FM0+ device type6
#define PDL_FM0P_TYPE7  70u   ///< FM0+ device type7
#define PDL_FM0P_TYPE8  80u   ///< FM0+ device type8
#define PDL_FM0P_TYPE9  90u   ///< FM0+ device type9
// FM3 device type  
#define PDL_FM3_TYPE0   300u  ///< FM3 device type0
#define PDL_FM3_TYPE1   310u  ///< FM3 device type1
#define PDL_FM3_TYPE2   320u  ///< FM3 device type2
#define PDL_FM3_TYPE3   330u  ///< FM3 device type3
#define PDL_FM3_TYPE4   340u  ///< FM3 device type4
#define PDL_FM3_TYPE5   350u  ///< FM3 device type5
#define PDL_FM3_TYPE6   360u  ///< FM3 device type6
#define PDL_FM3_TYPE7   370u  ///< FM3 device type7
#define PDL_FM3_TYPE8   380u  ///< FM3 device type8
#define PDL_FM3_TYPE9   390u  ///< FM3 device type9
#define PDL_FM3_TYPE10  391u  ///< FM3 device type10
#define PDL_FM3_TYPE11  392u  ///< FM3 device type11
#define PDL_FM3_TYPE12  393u  ///< FM3 device type12
// FM4 device type  
#define PDL_FM4_TYPE0   400u  ///< FM4 device type0
#define PDL_FM4_TYPE1   410u  ///< FM4 device type1
#define PDL_FM4_TYPE2   420u  ///< FM4 device type2
#define PDL_FM4_TYPE3   430u  ///< FM4 device type3
#define PDL_FM4_TYPE4   440u  ///< FM4 device type4
#define PDL_FM4_TYPE5   450u  ///< FM4 device type5
#define PDL_FM4_TYPE6   460u  ///< FM4 device type6
#define PDL_FM4_TYPE7   470u  ///< FM4 device type7
#define PDL_FM4_TYPE8   480u  ///< FM4 device type8
#define PDL_FM4_TYPE9   490u  ///< FM4 device type9

/**
 ******************************************************************************
 ** Global Device Series List
 ******************************************************************************/
#define PDL_DEVICE_SERIES_MB9AF10X   0u
#define PDL_DEVICE_SERIES_MB9AF11X  10u
#define PDL_DEVICE_SERIES_MB9AF12X  11u 
#define PDL_DEVICE_SERIES_MB9AF13X  20u
#define PDL_DEVICE_SERIES_MB9AF14X  25u
#define PDL_DEVICE_SERIES_MB9AF15X  26u
#define PDL_DEVICE_SERIES_MB9AF31X  30u
#define PDL_DEVICE_SERIES_MB9AF34X  32u
#define PDL_DEVICE_SERIES_MB9AF42X  33u  
#define PDL_DEVICE_SERIES_MB9AFA3X  34u
#define PDL_DEVICE_SERIES_MB9AFA4X  36u
#define PDL_DEVICE_SERIES_MB9AFB4X  37u
#define PDL_DEVICE_SERIES_MB9BF10X  40u
#define PDL_DEVICE_SERIES_MB9BF11X  45u
#define PDL_DEVICE_SERIES_MB9BF12X  46u
#define PDL_DEVICE_SERIES_MB9BF16X  47u  
#define PDL_DEVICE_SERIES_MB9BF21X  55u
#define PDL_DEVICE_SERIES_MB9BF30X  60u
#define PDL_DEVICE_SERIES_MB9BF31X  65u
#define PDL_DEVICE_SERIES_MB9BF32X  66u
#define PDL_DEVICE_SERIES_MB9BF36X  67u  
#define PDL_DEVICE_SERIES_MB9BF40X  70u
#define PDL_DEVICE_SERIES_MB9BF41X  75u
#define PDL_DEVICE_SERIES_MB9BF42X  76u  
#define PDL_DEVICE_SERIES_MB9BF46X  77u    
#define PDL_DEVICE_SERIES_MB9BF50X  80u
#define PDL_DEVICE_SERIES_MB9BF51X  85u
#define PDL_DEVICE_SERIES_MB9BF52X  86u
#define PDL_DEVICE_SERIES_MB9BF56X  87u 
#define PDL_DEVICE_SERIES_MB9BF61X 100u
#define PDL_DEVICE_SERIES_MB9BFD1X 110u  
#define PDL_DEVICE_SERIES_S6E1A1X  120u
#define PDL_DEVICE_SERIES_S6E2C1X  125u   
#define PDL_DEVICE_SERIES_S6E2C2X  126u   
#define PDL_DEVICE_SERIES_S6E2C3X  127u   
#define PDL_DEVICE_SERIES_S6E2C4X  128u   
#define PDL_DEVICE_SERIES_S6E2C5X  129u    
#define PDL_DEVICE_SERIES_S6E2CCX  130u
#define PDL_DEVICE_SERIES_S6E2D3X  137u   
#define PDL_DEVICE_SERIES_S6E2D5X  138u   
#define PDL_DEVICE_SERIES_S6E2DFX  139u   
#define PDL_DEVICE_SERIES_S6E2DHX  140u
#define PDL_DEVICE_SERIES_S6E100X  150u
#define PDL_DEVICE_SERIES_S6E1B3X  155u   
#define PDL_DEVICE_SERIES_S6E1B7X  160u
#define PDL_DEVICE_SERIES_S6E1B8X  170u
#define PDL_DEVICE_SERIES_S6E2G2X  176u   
#define PDL_DEVICE_SERIES_S6E2G3X  177u     
#define PDL_DEVICE_SERIES_S6E2GKX  178u   
#define PDL_DEVICE_SERIES_S6E2GHX  179u    
#define PDL_DEVICE_SERIES_S6E2GMX  180u
#define PDL_DEVICE_SERIES_S6E2H1X  187u    
#define PDL_DEVICE_SERIES_S6E2H4X  188u    
#define PDL_DEVICE_SERIES_S6E2HEX  189u   
#define PDL_DEVICE_SERIES_S6E2HGX  190u
#define PDL_DEVICE_SERIES_S6E1C1X  195u    
#define PDL_DEVICE_SERIES_S6E1C3X  200u   
  
/**
 ******************************************************************************
 ** Global Device Package List
 ******************************************************************************/
// Original package definitions of Fujitsu device (Name: MBxxxx)  
#define PDL_DEVICE_PACKAGE_MB_J  0u
#define PDL_DEVICE_PACKAGE_MB_K  1u
#define PDL_DEVICE_PACKAGE_MB_L  2u
#define PDL_DEVICE_PACKAGE_MB_M  3u
#define PDL_DEVICE_PACKAGE_MB_N  4u
#define PDL_DEVICE_PACKAGE_MB_R  5u
#define PDL_DEVICE_PACKAGE_MB_S  6u
#define PDL_DEVICE_PACKAGE_MB_T  7u
  
// New package definitions of Spansion device (Name: S6xxxx)  
#define PDL_DEVICE_PACKAGE_S6_B  101u
#define PDL_DEVICE_PACKAGE_S6_C  102u
#define PDL_DEVICE_PACKAGE_S6_D  103u
#define PDL_DEVICE_PACKAGE_S6_E  104u
#define PDL_DEVICE_PACKAGE_S6_F  105u
#define PDL_DEVICE_PACKAGE_S6_G  106u
#define PDL_DEVICE_PACKAGE_S6_H  107u
#define PDL_DEVICE_PACKAGE_S6_J  108u
#define PDL_DEVICE_PACKAGE_S6_K  109u
#define PDL_DEVICE_PACKAGE_S6_L  110u

/**
 ******************************************************************************
 ** Global Device Core list
 ******************************************************************************/
#define PDL_FM0P_CORE            10u
#define PDL_FM3_CORE             20u
#define PDL_FM4_CORE             30u  
  
/**
 ******************************************************************************
 ** Global Device interrupt list
 ******************************************************************************/  
// FM3 interrupt type
#define PDL_FM3_INT_TYPE_A                      0u
#define PDL_FM3_INT_TYPE_B                      1u
#define PDL_FM3_INT_TYPE_C                      2u

// FM0+ interrupt type
#define PDL_FM0P_INT_TYPE_A                     10u
#define PDL_FM0P_INT_TYPE_B                     11u
#define PDL_FM0P_INT_TYPE_C                     12u      
      
// FM4 interrupt type
#define PDL_FM4_INT_TYPE_A                      20u
#define PDL_FM4_INT_TYPE_B                      21u
#define PDL_FM4_INT_TYPE_C                      22u    
  
/******************************************************************************/
/* User Device Setting Include file                                           */
/******************************************************************************/
#include "pdl_device.h"  // MUST be included here!

/**
 ******************************************************************************
 ** Device type extraction
 ******************************************************************************/        
#if (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AF10X)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_R)
    #define PDL_MCU_TYPE  PDL_FM3_TYPE0
  #else
    #error Device Package not found!
  #endif

#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AF11X)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_L) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_M) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N)
    #define PDL_MCU_TYPE PDL_FM3_TYPE1
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_K)
    #define PDL_MCU_TYPE PDL_FM3_TYPE5
  #else
    #error Device Package not found!
  #endif

#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AF12X)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_L)
    #define PDL_MCU_TYPE PDL_FM3_TYPE11
  #else
    #error Device Package not found!
  #endif          
          
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AF13X)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_K) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_L)
    #define PDL_MCU_TYPE PDL_FM3_TYPE3
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_M) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N)
    #define PDL_MCU_TYPE PDL_FM3_TYPE7
  #else
    #error Device Package not found!
  #endif

#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AF14X)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_L) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_M) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N)
    #define PDL_MCU_TYPE PDL_FM3_TYPE6
  #else
    #error Device Package not found!
  #endif

#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AF15X)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_R) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_M) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N)
    #define PDL_MCU_TYPE PDL_FM3_TYPE8
  #else
    #error Device Package not found!
  #endif

#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AF31X)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_L) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_M) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N)
    #define PDL_MCU_TYPE PDL_FM3_TYPE1
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_K)
    #define PDL_MCU_TYPE PDL_FM3_TYPE5
  #else
    #error Device Package not found!
  #endif

#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AF34X)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_L) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_M) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N)
    #define PDL_MCU_TYPE PDL_FM3_TYPE6
  #else
    #error Device Package not found!
  #endif

#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AF42X)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_L)
    #define PDL_MCU_TYPE PDL_FM3_TYPE11
  #else
    #error Device Package not found!
  #endif          
          
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AFA3X)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_L) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_M) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N)
    #define PDL_MCU_TYPE PDL_FM3_TYPE7
  #else
    #error Device Package not found!
  #endif

#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AFA4X)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_L) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_M) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N)
    #define PDL_MCU_TYPE PDL_FM3_TYPE6
  #else
    #error Device Package not found!
  #endif

#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AFB4X)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_L) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_M) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N)
    #define PDL_MCU_TYPE PDL_FM3_TYPE6
  #else
    #error Device Package not found!
  #endif

#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF10X)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_R)
    #define PDL_MCU_TYPE PDL_FM3_TYPE0
  #else
    #error Device Package not found!
  #endif

#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF11X)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_R)
    #define PDL_MCU_TYPE PDL_FM3_TYPE4
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_S) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_T)
    #define PDL_MCU_TYPE PDL_FM3_TYPE2
  #else
    #error Device Package not found!
  #endif

#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF12X)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_K) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_L) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_M)  
    #define PDL_MCU_TYPE PDL_FM3_TYPE9
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_J)
    #define PDL_MCU_TYPE PDL_FM3_TYPE10
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_T)
    #define PDL_MCU_TYPE PDL_FM3_TYPE12          
  #else        
    #error Device Package not found!
  #endif

#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF16X)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_R) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_M) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_L) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_K)  
    #define PDL_MCU_TYPE PDL_FM4_TYPE1
  #else        
    #error Device Package not found!
  #endif          
          
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF21X)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_S) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_T)
    #define PDL_MCU_TYPE PDL_FM3_TYPE2
  #else
    #error Device Package not found!
  #endif

#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF30X)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_R)
    #define PDL_MCU_TYPE PDL_FM3_TYPE0
  #else
    #error Device Package not found!
  #endif

#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF31X)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_R)
    #define PDL_MCU_TYPE PDL_FM3_TYPE4
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_S) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_T)
    #define PDL_MCU_TYPE PDL_FM3_TYPE2
  #else
    #error Device Package not found!
  #endif

#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF32X)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_K) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_L) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_M)  
    #define PDL_MCU_TYPE PDL_FM3_TYPE9
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_T)
    #define PDL_MCU_TYPE PDL_FM3_TYPE12          
  #else
   #error Device Package not found!
  #endif

#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF36X)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_R) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_M) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_L) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_K)    
    #define PDL_MCU_TYPE PDL_FM4_TYPE1
  #else        
    #error Device Package not found!
  #endif            
          
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF40X)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_R)
    #define PDL_MCU_TYPE PDL_FM3_TYPE0
  #else
    #error Device Package not found!
  #endif

#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF41X)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_R)
    #define PDL_MCU_TYPE PDL_FM3_TYPE4
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_S) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_T)
    #define PDL_MCU_TYPE PDL_FM3_TYPE2
  #else
    #error Device Package not found!
  #endif

#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF42X)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_T)
    #define PDL_MCU_TYPE PDL_FM3_TYPE12
  #else
    #error Device Package not found!
  #endif          

#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF46X)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_R) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_M) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_L) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_K)    
    #define PDL_MCU_TYPE PDL_FM4_TYPE1
  #else        
    #error Device Package not found!
  #endif 
          
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF50X)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_R)
    #define PDL_MCU_TYPE PDL_FM3_TYPE0
  #else
    #error Device Package not found!
  #endif

#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF51X)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_R)
    #define PDL_MCU_TYPE PDL_FM3_TYPE4
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_S) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_T)
    #define PDL_MCU_TYPE PDL_FM3_TYPE2
  #else
    #error Device Package not found!
  #endif

#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF52X)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_K) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_L) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_M)  
    #define PDL_MCU_TYPE PDL_FM3_TYPE9
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_T)
    #define PDL_MCU_TYPE PDL_FM3_TYPE12          
  #else
    #error Device Package not found!
  #endif

#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF56X)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_R) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_M) 
    #define PDL_MCU_TYPE PDL_FM4_TYPE1
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_L) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_K)    
    #define PDL_MCU_TYPE PDL_FM4_TYPE2      
  #else        
    #error Device Package not found!
  #endif 
          
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF61X)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_S) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_T)
    #define PDL_MCU_TYPE PDL_FM3_TYPE2
  #else
    #error Device Package not found!
  #endif

#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BFD1X)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_S) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_T)
    #define PDL_MCU_TYPE PDL_FM3_TYPE2
  #else
    #error Device Package not found!
  #endif

#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E1A1X)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_B) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_C)
    #define PDL_MCU_TYPE PDL_FM0P_TYPE1
  #else
    #error Device Package not found!
  #endif  
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2C1X)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_H) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_J) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_L)
    #define PDL_MCU_TYPE PDL_FM4_TYPE3
  #else 
    #error Device Package not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2C2X)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_H) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_J) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_L)
    #define PDL_MCU_TYPE PDL_FM4_TYPE3
  #else 
    #error Device Package not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2C3X)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_H) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_J) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_L)
    #define PDL_MCU_TYPE PDL_FM4_TYPE3
  #else 
    #error Device Package not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2C4X)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_H) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_J) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_L)
    #define PDL_MCU_TYPE PDL_FM4_TYPE3
  #else 
    #error Device Package not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2C5X)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_H) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_J) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_L)
    #define PDL_MCU_TYPE PDL_FM4_TYPE3
  #else 
    #error Device Package not found!
  #endif          
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2CCX)
  #if   (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_H) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_J) || \
        (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_L)
    #define PDL_MCU_TYPE PDL_FM4_TYPE3
  #else 
    #error Device Package not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2D3X)
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_J) || \
      (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_G)     
    #define PDL_MCU_TYPE PDL_FM4_TYPE4
  #else
    #error Device Package not found!    
  #endif          
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2D5X)
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_J) || \
      (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_G)     
    #define PDL_MCU_TYPE PDL_FM4_TYPE4
  #else
    #error Device Package not found!    
  #endif          
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2DFX)
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_J) || \
      (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_G)     
    #define PDL_MCU_TYPE PDL_FM4_TYPE4
  #else
    #error Device Package not found!    
  #endif           
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2DHX)
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_J) || \
      (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_G)     
    #define PDL_MCU_TYPE PDL_FM4_TYPE4
  #else
    #error Device Package not found!    
  #endif          
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E100X)
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_F)
    #define PDL_MCU_TYPE PDL_FM0P_TYPE2    
  #else
    #error Device Package not found!        
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E1B3X)
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_E) || \
      (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_F) || \
      (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_G)  
    #define PDL_MCU_TYPE PDL_FM0P_TYPE2    
  #else
    #error Device Package not found!        
  #endif        
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E1B7X)
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_E) || \
      (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_F) || \
      (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_G)  
    #define PDL_MCU_TYPE PDL_FM0P_TYPE2    
  #else
    #error Device Package not found!        
  #endif            
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E1B8X) 
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_E) || \
      (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_F) || \
      (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_G)   
    #define PDL_MCU_TYPE PDL_FM0P_TYPE2    
  #else
    #error Device Package not found!        
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2G2X)
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_H) || \
      (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_J)
    #define PDL_MCU_TYPE PDL_FM4_TYPE5
  #else
    #error Device Package not found!            
  #endif        
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2G3X)
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_H) || \
      (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_J)
    #define PDL_MCU_TYPE PDL_FM4_TYPE5
  #else
    #error Device Package not found!            
  #endif         
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2GKX)
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_H) || \
      (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_J)
    #define PDL_MCU_TYPE PDL_FM4_TYPE5
  #else
    #error Device Package not found!            
  #endif        
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2GHX)
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_H) || \
      (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_J)
    #define PDL_MCU_TYPE PDL_FM4_TYPE5
  #else
    #error Device Package not found!            
  #endif         
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2GMX)
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_H) || \
      (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_J)
    #define PDL_MCU_TYPE PDL_FM4_TYPE5
  #else
    #error Device Package not found!            
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2H1X)
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_E) || \
      (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_F) || \
      (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_G)  
    #define PDL_MCU_TYPE PDL_FM4_TYPE6
  #else
    #error Device Package not found!            
  #endif        
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2H4X)
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_E) || \
      (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_F) || \
      (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_G)  
    #define PDL_MCU_TYPE PDL_FM4_TYPE6
  #else
    #error Device Package not found!            
  #endif          
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2HEX)
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_E) || \
      (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_F) || \
      (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_G)  
    #define PDL_MCU_TYPE PDL_FM4_TYPE6
  #else
    #error Device Package not found!            
  #endif        
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2HGX)
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_E) || \
      (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_F) || \
      (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_G)  
    #define PDL_MCU_TYPE PDL_FM4_TYPE6
  #else
    #error Device Package not found!            
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E1C1X)
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_B) || \
      (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_C) || \
      (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_D)  
    #define PDL_MCU_TYPE PDL_FM0P_TYPE3
  #else
    #error Device Package not found!            
  #endif        
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E1C3X)
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_B) || \
      (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_C) || \
      (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_D)  
    #define PDL_MCU_TYPE PDL_FM0P_TYPE3
  #else
    #error Device Package not found!            
  #endif         
#else
  #error Device Series not found!
#endif        

/**
 ******************************************************************************
 ** Device core extraction
 ******************************************************************************/            
#if (PDL_MCU_TYPE == PDL_FM3_TYPE0) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE1) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE2) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE3) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE4) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE5) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE6) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE7) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE8) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE9) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE10) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE11) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE12)
    #define PDL_MCU_CORE    PDL_FM3_CORE
#elif (PDL_MCU_TYPE == PDL_FM0P_TYPE1) || \
      (PDL_MCU_TYPE == PDL_FM0P_TYPE2) || \
      (PDL_MCU_TYPE == PDL_FM0P_TYPE3)  
    #define PDL_MCU_CORE    PDL_FM0P_CORE
#elif (PDL_MCU_TYPE == PDL_FM4_TYPE1) || \
      (PDL_MCU_TYPE == PDL_FM4_TYPE2) || \
      (PDL_MCU_TYPE == PDL_FM4_TYPE3) || \
      (PDL_MCU_TYPE == PDL_FM4_TYPE4) || \
      (PDL_MCU_TYPE == PDL_FM4_TYPE5) || \
      (PDL_MCU_TYPE == PDL_FM4_TYPE6)  
    #define PDL_MCU_CORE    PDL_FM4_CORE  
#else
    #error Device type not found!     
#endif      
       
          
/**
 ******************************************************************************
 ** Default Interrupt Level (lowest priority, used for De-Init functions)
 ******************************************************************************/
#if (PDL_MCU_CORE == PDL_FM3_CORE) || (PDL_MCU_CORE == PDL_FM4_CORE)       
#define PDL_DEFAULT_INTERRUPT_LEVEL 0x0Fu
#else
#define PDL_DEFAULT_INTERRUPT_LEVEL 0x03u        
#endif        

/**
 ******************************************************************************
 ** PDL resource availability check
 ******************************************************************************/

// ADC
#define PDL_PERIPHERAL_ADC_AVAILABLE                          PDL_ON

// Base Timer
#define PDL_PERIPHERAL_BT_AVAILABLE                           PDL_ON

// Clock
#define PDL_PERIPHERAL_CLK_AVAILABLE                          PDL_ON

// CAN
#if (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BFD1X) ||  \
    (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF51X) ||  \
    (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF41X) ||  \
    (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF50X) ||  \
    (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF40X) || \
    (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF52X) || \
    (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF42X) || \
    (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AF42X)  
    #define PDL_PERIPHERAL_CAN_AVAILABLE                       PDL_ON   
#else
    #define PDL_PERIPHERAL_CAN_AVAILABLE                       PDL_OFF
#endif          

// CEC 
#if (PDL_MCU_TYPE == PDL_FM3_TYPE6) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE7) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE12)
    #define PDL_PERIPHERAL_RC_AVAILABLE                        PDL_ON
#else
    #define PDL_PERIPHERAL_RC_AVAILABLE                        PDL_OFF
#endif      
      
// CR Trimming
#define PDL_PERIPHERAL_CRTRIM_AVAILABLE                        PDL_ON

// CRC      
#if (PDL_MCU_TYPE == PDL_FM3_TYPE3) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE7) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE10) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE11) || \
    (PDL_MCU_TYPE == PDL_FM0P_TYPE1)  
    #define PDL_PERIPHERAL_CRC_AVAILABLE                       PDL_OFF  
#else
    #define PDL_PERIPHERAL_CRC_AVAILABLE                       PDL_ON
#endif      
      
// CSV
#define PDL_PERIPHERAL_CSV_AVAILABLE                           PDL_ON

// DAC
#define PDL_PERIPHERAL_DAC_AVAILABLE                           PDL_ON          
      
// DMA
#define PDL_PERIPHERAL_DMA_AVAILABLE                           PDL_ON      
      
// Dual Timer      
#if ((PDL_MCU_SERIES != PDL_DEVICE_SERIES_MB9AF13X) && (PDL_MCU_SERIES != PDL_DEVICE_SERIES_MB9AFA3X))        
    #define PDL_PERIPHERAL_DT_AVAILABLE                        PDL_ON
#else
    #define PDL_PERIPHERAL_DT_AVAILABLE                        PDL_OFF
#endif       
      
// External interrupt
#define PDL_PERIPHERAL_EXINT_AVAILABLE                         PDL_ON

// LVD
#define PDL_PERIPHERAL_LVD_AVAILABLE                           PDL_ON

// MFS
#define PDL_PERIPHERAL_MFS_AVAILABLE                           PDL_ON
      
// QPRC
#if (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AFB4X) || \
    (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AFA4X) || \
    (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AF34X) || \
    (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AF14X) || \
    (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AFA3X) || \
    (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AF13X)          
    #define PDL_PERIPHERAL_QPRC_AVAILABLE                      PDL_OFF  
#else
    #define PDL_PERIPHERAL_QPRC_AVAILABLE                      PDL_ON
#endif      

// VBAT 
#if (PDL_MCU_CORE == PDL_FM4_CORE)
  #if (PDL_MCU_TYPE == PDL_FM4_TYPE5)  
    #define PDL_PERIPHERAL_VBAT_AVAILABLE                      PDL_OFF
  #else
    #define PDL_PERIPHERAL_VBAT_AVAILABLE                      PDL_ON    
  #endif    
#elif (PDL_MCU_CORE == PDL_FM3_CORE)
    #define PDL_PERIPHERAL_VBAT_AVAILABLE                      PDL_OFF
#elif (PDL_MCU_CORE == PDL_FM0P_CORE)
  #if (PDL_MCU_TYPE == PDL_FM0P_TYPE1) || (PDL_MCU_TYPE == PDL_FM0P_TYPE3)
    #define PDL_PERIPHERAL_VBAT_AVAILABLE                      PDL_OFF      
  #else
    #define PDL_PERIPHERAL_VBAT_AVAILABLE                      PDL_ON  
  #endif 
#else
    #define PDL_PERIPHERAL_VBAT_AVAILABLE                      PDL_OFF    
#endif      
      
// RESET
#define PDL_PERIPHERAL_RESET_AVAILABLE                         PDL_ON
      
// Watch counter  
#if (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AFA3X) || \
    (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AF13X)       
    #define PDL_PERIPHERAL_WC_AVAILABLE                        PDL_OFF   
#else
    #define PDL_PERIPHERAL_WC_AVAILABLE                        PDL_ON
#endif      

// Hardware watchdog
#define PDL_PERIPHERAL_HWWDG_AVAILABLE                         PDL_ON

// Software watchdog
#define PDL_PERIPHERAL_SWWDG_AVAILABLE                         PDL_ON

// RTC
#if (PDL_MCU_TYPE == PDL_FM3_TYPE3) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE4) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE5) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE6) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE7) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE8) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE9) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE10) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE11) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE12) || \
    (PDL_MCU_CORE == PDL_FM0P_CORE)  || \
    (PDL_MCU_CORE == PDL_FM4_CORE)  
    #define PDL_PERIPHERAL_RTC_AVAILABLE                       PDL_ON
#else
    #define PDL_PERIPHERAL_RTC_AVAILABLE                       PDL_OFF  
#endif      
      
// LCD 
#if (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AFB4X) || \
    (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AFA4X) || \
    (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AFA3X)       
    #define PDL_PERIPHERAL_LCD_AVAILABLE                       PDL_ON
#else
    #define PDL_PERIPHERAL_LCD_AVAILABLE                       PDL_OFF      
#endif

// Unique ID
#if (PDL_MCU_TYPE == PDL_FM3_TYPE6) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE8) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE9) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE10)|| \
    (PDL_MCU_TYPE == PDL_FM3_TYPE11)|| \
    (PDL_MCU_TYPE == PDL_FM3_TYPE12)|| \
    (PDL_MCU_TYPE == PDL_DEVICE_FM0P_TYPE1)|| \
    (PDL_MCU_TYPE == PDL_DEVICE_FM4_TYPE4)       
    #define PDL_PERIPHERAL_UID_AVAILABLE                       PDL_ON
#else
    #define PDL_PERIPHERAL_UID_AVAILABLE                       PDL_OFF
#endif      

// USB
#if (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF50X) || \
    (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF30X) || \
    (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BFD1X) || \
    (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF61X) || \
    (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF51X) || \
    (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF31X) || \
    (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF21X) || \
    (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AFB4X) || \
    (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AF34X) || \
    (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AF31X) || \
    (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF52X) || \
    (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF32X)  
    #define PDL_PERIPHERAL_USB_AVAILABLE                       PDL_ON
#else
    #define PDL_PERIPHERAL_USB_AVAILABLE                       PDL_OFF  
#endif      

/**
 ******************************************************************************
 ** MCU Interrupt Type extraction
 ******************************************************************************/      
#if (PDL_MCU_TYPE == PDL_FM3_TYPE0) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE1) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE2) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE4) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE5) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE6) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE8) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE9) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE10) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE11) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE12)       
    #define PDL_MCU_INT_TYPE       PDL_FM3_INT_TYPE_A
#elif (PDL_MCU_TYPE == PDL_FM3_TYPE3) || \
      (PDL_MCU_TYPE == PDL_FM3_TYPE7)
    #define PDL_MCU_INT_TYPE       PDL_FM3_INT_TYPE_C
#elif (PDL_MCU_TYPE == PDL_FM0P_TYPE1) || \
      (PDL_MCU_TYPE == PDL_FM0P_TYPE2)   
    #define PDL_MCU_INT_TYPE       PDL_FM0P_INT_TYPE_A    
#elif (PDL_MCU_TYPE == PDL_FM0P_TYPE3)
    #define PDL_MCU_INT_TYPE       PDL_FM0P_INT_TYPE_C         
#elif (PDL_MCU_TYPE == PDL_FM4_TYPE1) || \
      (PDL_MCU_TYPE == PDL_FM4_TYPE2) || \
      (PDL_MCU_TYPE == PDL_FM4_TYPE3) || \
      (PDL_MCU_TYPE == PDL_FM4_TYPE4) || \
      (PDL_MCU_TYPE == PDL_FM4_TYPE5) || \
      (PDL_MCU_TYPE == PDL_FM4_TYPE6)  
    #define PDL_MCU_INT_TYPE       PDL_FM4_INT_TYPE_A     
#else
    #error MCU Type not found!
#endif

/**
 ******************************************************************************
 ** \brief IRQ name definition for all type MCUs
 ******************************************************************************/
#if (PDL_MCU_INT_TYPE == PDL_FM3_INT_TYPE_A)    
    #define CSV_IRQHandler(void)             IRQ000_Handler(void)  ///< CSV 
    #define SWDT_IRQHandler(void)            IRQ001_Handler(void)  ///< SW watchdog 
    #define LVD_IRQHandler(void)             IRQ002_Handler(void)  ///< LVD        
    #define MFT_WFG_IRQHandler(void)         IRQ003_Handler(void)  ///< MFT's WFG    
    #define INT0_7_IRQHandler(void)          IRQ004_Handler(void)  ///< Ext Int 0~7  
    #define INT8_31_IRQHandler(void)         IRQ005_Handler(void)  ///< Ext Int 8~31  
    #define DT_QPRC_IRQHandler(void)         IRQ006_Handler(void)  ///< DT & QRRC   
    #define MFS0_8_RX_IRQHandler(void)       IRQ007_Handler(void)  ///< MFS 0&8 RX   
    #define MFS0_8_TX_IRQHandler(void)       IRQ008_Handler(void)  ///< MFS 0&8 TX  
    #define MFS1_9_RX_IRQHandler(void)       IRQ009_Handler(void)  ///< MFS 1&9 RX  
    #define MFS1_9_TX_IRQHandler(void)       IRQ010_Handler(void) ///< MFS 1&9 TX  
    #define MFS2_10_RX_IRQHandler(void)      IRQ011_Handler(void) ///< MFS 2&10 RX  
    #define MFS2_10_TX_IRQHandler(void)      IRQ012_Handler(void) ///< MFS 2&10 TX  
    #define MFS3_11_RX_IRQHandler(void)      IRQ013_Handler(void) ///< MFS 3&11 RX  
    #define MFS3_11_TX_IRQHandler(void)      IRQ014_Handler(void) ///< MFS 3&11 TX  
    #define MFS4_12_RX_IRQHandler(void)      IRQ015_Handler(void) ///< MFS 4&12 RX  
    #define MFS4_12_TX_IRQHandler(void)      IRQ016_Handler(void) ///< MFS 4&12 TX  
    #define MFS5_13_RX_IRQHandler(void)      IRQ017_Handler(void) ///< MFS 5&13 RX 
    #define MFS5_13_TX_IRQHandler(void)      IRQ018_Handler(void) ///< MFS 5&13 TX  
    #define MFS6_14_RX_IRQHandler(void)      IRQ019_Handler(void) ///< MFS 6&14 RX 
    #define MFS6_14_TX_IRQHandler(void)      IRQ020_Handler(void) ///< MFS 6&14 TX 
    #define MFS7_15_RX_IRQHandler(void)      IRQ021_Handler(void) ///< MFS 7&15 RX 
    #define MFS7_15_TX_IRQHandler(void)      IRQ022_Handler(void) ///< MFS 7&15 TX
    #define PPG_IRQHandler(void)             IRQ023_Handler(void) ///< PPG 
    #define TIM_IRQHandler(void)             IRQ024_Handler(void) ///< MOSC,SOSC,PLL,WC,RTC 
    #define ADC0_IRQHandler(void)            IRQ025_Handler(void) ///< ADC0
    #define ADC1_IRQHandler(void)            IRQ026_Handler(void) ///< ADC1
    #define ADC2_LCD_IRQHandler(void)        IRQ027_Handler(void) ///< ADC2, LCD 
    #define MFT_FRT_IRQHandler(void)         IRQ028_Handler(void) ///< MFT's FRT 
    #define MFT_IPC_IRQHandler(void)         IRQ029_Handler(void) ///< MFT's IPC 
    #define MFT_OPC_IRQHandler(void)         IRQ030_Handler(void) ///< MFT's OPC 
    #define BT0_7_IRQHandler(void)           IRQ031_Handler(void) ///< BT0~7 
    #define ETHER0_CAN0_IRQHandler(void)     IRQ032_Handler(void) ///< CAN0, Ether0 
    #define ETHER1_CAN1_IRQHandler(void)     IRQ033_Handler(void) ///< CAN1, Ether1 
    #define USB0F_IRQHandler(void)           IRQ034_Handler(void) ///< USB0 Function 
    #define USB0_IRQHandler(void)            IRQ035_Handler(void) ///< USB0 Function&Host
    #define USB1F_HDMICEC0_IRQHandler(void)  IRQ036_Handler(void) ///< USB1 Function, CEC0
    #define USB1_HDMICEC1_IRQHandler(void)   IRQ037_Handler(void) ///< USB1 Function&Host, CEC1
    #define DMAC0_IRQHandler(void)           IRQ038_Handler(void) ///< DMAC0 
    #define DMAC1_IRQHandler(void)           IRQ039_Handler(void) ///< DMAC1  
    #define DMAC2_IRQHandler(void)           IRQ040_Handler(void) ///< DMAC2  
    #define DMAC3_IRQHandler(void)           IRQ041_Handler(void) ///< DMAC3  
    #define DMAC4_IRQHandler(void)           IRQ042_Handler(void) ///< DMAC4  
    #define DMAC5_IRQHandler(void)           IRQ043_Handler(void) ///< DMAC5  
    #define DMAC6_IRQHandler(void)           IRQ044_Handler(void) ///< DMAC6 
    #define DMAC7_IRQHandler(void)           IRQ045_Handler(void) ///< DMAC7 
    #define BT8_15_IRQHandler(void)          IRQ046_Handler(void) ///< BT8~15 
    #define FLASH_IRQHandler(void)           IRQ047_Handler(void) ///< Work Flash    
#elif (PDL_MCU_INT_TYPE == PDL_FM3_INT_TYPE_C)
    #define CSV_IRQHandler(void)              IRQ000_Handler(void)  ///< CSV 
    #define SWDT_IRQHandler(void)             IRQ001_Handler(void)  ///< SW watchdog 
    #define LVD_IRQHandler(void)              IRQ002_Handler(void)  ///< LVD       
    #define MFT_WFG_IRQHandler(void)          IRQ003_Handler(void)  ///< MFT's WFG  
    #define INT0_7_IRQHandler(void)           IRQ004_Handler(void)  ///< Ext Int 0~7
    #define INT8_31_IRQHandler(void)          IRQ005_Handler(void)  ///< Ext Int 8~31  
    #define MFS0_8_RX_IRQHandler(void)        IRQ006_Handler(void)  ///< MFS 0&8 RX 
    #define MFS0_8_TX_IRQHandler(void)        IRQ007_Handler(void)  ///< MFS 0&8 TX  
    #define MFS1_9_RX_IRQHandler(void)        IRQ008_Handler(void)  ///< MFS 1&9 RX 
    #define MFS1_9_TX_IRQHandler(void)        IRQ009_Handler(void)  ///< MFS 1&9 TX 
    #define MFS2_10_RX_IRQHandler(void)       IRQ010_Handler(void) ///< MFS 2&10 RX
    #define MFS2_10_TX_IRQHandler(void)       IRQ011_Handler(void) ///< MFS 2&10 TX
    #define MFS3_11_RX_IRQHandler(void)       IRQ012_Handler(void) ///< MFS 3&11 RX
    #define MFS3_11_TX_IRQHandler(void)       IRQ013_Handler(void) ///< MFS 3&11 TX
    #define MFS4_12_RX_IRQHandler(void)       IRQ014_Handler(void) ///< MFS 4&12 RX  
    #define MFS4_12_TX_IRQHandler(void)       IRQ015_Handler(void) ///< MFS 4&12 TX  
    #define MFS5_13_RX_IRQHandler(void)       IRQ016_Handler(void) ///< MFS 5&13 RX 
    #define MFS5_13_TX_IRQHandler(void)       IRQ017_Handler(void) ///< MFS 5&13 TX 
    #define MFS6_14_RX_IRQHandler(void)       IRQ018_Handler(void) ///< MFS 6&14 RX  
    #define MFS6_14_TX_IRQHandler(void)       IRQ019_Handler(void) ///< MFS 6&14 TX  
    #define MFS7_15_RX_IRQHandler(void)       IRQ020_Handler(void) ///< MFS 7&15 RX 
    #define MFS7_15_TX_IRQHandler(void)       IRQ021_Handler(void) ///< MFS 7&15 TX 
    #define PPG_IRQHandler(void)              IRQ022_Handler(void) ///< PPG
    #define TIM_IRQHandler(void)              IRQ023_Handler(void) ///< MOSC,SOSC,PLL,WC,RTC 
    #define ADC0_IRQHandler(void)             IRQ024_Handler(void) ///< ADC0
    #define MFT_FRT_IRQHandler(void)          IRQ025_Handler(void) ///< MFT's FRT 
    #define MFT_IPC_IRQHandler(void)          IRQ026_Handler(void) ///< MFT's IPC 
    #define MFT_OPC_IRQHandler(void)          IRQ027_Handler(void) ///< MFT's OPC
    #define BT0_7_IRQHandler(void)            IRQ028_Handler(void) ///< BT0~7      
    #define ADC2_LCD_IRQHandler(void)         IRQ029_Handler(void) ///< LCD 
    #define USB1F_HDMICEC0_IRQHandler(void)   IRQ030_Handler(void) ///< CEC0
    #define USB1_HDMICEC1_IRQHandler(void)    IRQ031_Handler(void) ///< CEC1
#elif (PDL_MCU_INT_TYPE == PDL_FM0P_INT_TYPE_A)
    #define CSV_IRQHandler(void)               IRQ000_Handler(void) ///< CSV
    #define SWDT_IRQHandler(void)              IRQ001_Handler(void) ///< SW watchdog
    #define LVD_IRQHandler(void)               IRQ002_Handler(void) ///< LVD
    #define MFT_WFG_IRQHandler(void)           IRQ003_Handler(void) ///< Interrupt Source Selection 3
    #define INT0_7_IRQHandler(void)            IRQ004_Handler(void) ///< Interrupt Source Selection 4
    #define INT8_31_IRQHandler(void)           IRQ005_Handler(void) ///< Interrupt Source Selection 5
    #define DT_QPRC_IRQHandler(void)           IRQ006_Handler(void) ///< Interrupt Source Selection 6
    #define MFS0_8_RX_IRQHandler(void)         IRQ007_Handler(void) ///< Interrupt Source Selection 7
    #define MFS0_8_TX_IRQHandler(void)         IRQ008_Handler(void) ///< Interrupt Source Selection 8
    #define MFS1_9_RX_IRQHandler(void)         IRQ009_Handler(void) ///< Interrupt Source Selection 9
    #define MFS1_9_TX_IRQHandler(void)         IRQ010_Handler(void) ///< Interrupt Source Selection 10
    #define MFS2_10_RX_IRQHandler(void)        IRQ011_Handler(void) ///< External Pin Interrupt Ch. 0
    #define MFS2_10_TX_IRQHandler(void)        IRQ012_Handler(void) ///< External Pin Interrupt Ch. 1
    #define MFS3_11_RX_IRQHandler(void)        IRQ013_Handler(void) ///< External Pin Interrupt Ch. 2
    #define MFS3_11_TX_IRQHandler(void)        IRQ014_Handler(void) ///< External Pin Interrupt Ch. 3
    #define MFS4_12_RX_IRQHandler(void)        IRQ015_Handler(void) ///< External Pin Interrupt Ch. 4
    #define MFS4_12_TX_IRQHandler(void)        IRQ016_Handler(void) ///< External Pin Interrupt Ch. 5
    #define MFS5_13_RX_I2SL0_RX_IRQHandler(void)        IRQ017_Handler(void) ///< External Pin Interrupt Ch. 6
    #define MFS5_13_TX_I2SL0_TX_IRQHandler(void)        IRQ018_Handler(void) ///< External Pin Interrupt Ch. 7
    #define MFS6_14_RX_DMA0_I2SL1_RX_IRQHandler(void)   IRQ019_Handler(void) ///< Quad Position & Revolution Counter Ch. 0
    #define MFS6_14_TX_DMA1_I2SL1_TX_IRQHandler(void)   IRQ020_Handler(void) ///< Quad Position & Revolution Counter Ch. 1
    #define MFS7_15_RX_DMA2_IRQHandler(void)   IRQ021_Handler(void) ///< MFT Unit 0 Waveform Generator / DTIF Ch. 0
    #define MFS7_15_TX_DMA3_IRQHandler(void)   IRQ022_Handler(void) ///< MFT Unit 1 Waveform Generator / DTIF Ch. 1
    #define PPG_IRQHandler(void)               IRQ023_Handler(void) ///< PPG/HDMI-CEC ch.0/IC card ch.0
    #define TIM_IRQHandler(void)               IRQ024_Handler(void) ///< Watch Counter/Real Time Counter/HDMI-CEC ch.1/IC card ch.1
    #define ADC0_IRQHandler(void)              IRQ025_Handler(void) ///< ADC Unit 0
    #define ADC1_IRQHandler(void)              IRQ026_Handler(void) ///< ADC Unit 1
    #define ADC2_LCD_IRQHandler(void)          IRQ027_Handler(void) ///< ADC Unit 2/LCD
    #define MFT_FRT_IRQHandler(void)           IRQ028_Handler(void) ///< MFT Unit 0/1/2 Free-Run Timer
    #define MFT_IPC_IRQHandler(void)           IRQ029_Handler(void) ///< MFT Unit 0/1/2 Input Capture Unit
    #define MFT_OPC_IRQHandler(void)           IRQ030_Handler(void) ///< MFT Unit 0/1/2 Output Compare Unit
    #define BT0_7_FLASH_IRQHandler(void)       IRQ031_Handler(void) ///< Base Timer ch.0~ch.7
#elif (PDL_MCU_INT_TYPE == PDL_FM0P_INT_TYPE_C)
    #define CSV_SWD_LVD_IRQHandler(void)       IRQ000_Handler(void) ///< CSV/SWD/LVD
    #define MFS0_IRQHandler(void)              IRQ001_Handler(void) ///< MFS ch.0
    #define MFS1_IRQHandler(void)              IRQ002_Handler(void) ///< MFS ch.1
//                                             IRQ003_Handler(void) ///< Reserved
    #define MFS3_IRQHandler(void)              IRQ004_Handler(void) ///< MFS ch.3
    #define MFS4_IRQHandler(void)              IRQ005_Handler(void) ///< MFS ch.4
//                                             IRQ006_Handler(void) ///< Reserved        
    #define MFS6_I2CSLAVE_IRQHandler(void)     IRQ007_Handler(void) ///< MFS ch.6/I2CSLAVE
    #define MFS7_IRQHandler(void)              IRQ008_Handler(void) ///< MFS ch.7
    #define ADC0_IRQHandler(void)              IRQ009_Handler(void) ///< ADC unit.0
    #define USB0F0_IRQHandler(void)            IRQ010_Handler(void) ///< USB0 function endpoint 1/2/3
    #define USB0F1_IRQHandler(void)            IRQ011_Handler(void) ///< USB0 function endpoint 4/5/0 in
    #define USB0F2_IRQHandler(void)            IRQ012_Handler(void) ///< USB0 function endpoint 0 out/SUSP/SOF/BRST/CONF/WKUP/SPK
    #define USB0H_IRQHandler(void)             IRQ013_Handler(void) ///< USB0 host
    #define OSC_IRQHandler(void)               IRQ014_Handler(void) ///< Main PLL/Main Osc/Sub clock
    #define TIM_IRQHandler(void)               IRQ015_Handler(void) ///< WC/RTC/DT        
    #define INT0_1_IRQHandler(void)            IRQ016_Handler(void) ///< External interrupt ch.0/1
    #define INT2_3_IRQHandler(void)            IRQ017_Handler(void) ///< External interrupt ch.2/3
    #define INT4_5_IRQHandler(void)            IRQ018_Handler(void) ///< External interrupt ch.4/5
    #define INT6_7_IRQHandler(void)            IRQ019_Handler(void) ///< External interrupt ch.6/7
    #define INT8_IRQHandler(void)              IRQ020_Handler(void) ///< External interrupt ch.8
//                                             IRQ021_Handler(void) ///< Reserved        
    #define INT12_13_IRQHandler(void)          IRQ022_Handler(void) ///< External interrupt ch.12/13
    #define INT15_IRQHandler(void)             IRQ023_Handler(void) ///< External interrupt ch.15        
    #define BT0_4_IRQHandler(void)             IRQ024_Handler(void) ///< Base timer ch.0/4
    #define BT1_5_IRQHandler(void)             IRQ025_Handler(void) ///< Base timer ch.1/5
    #define BT2_6_IRQHandler(void)             IRQ026_Handler(void) ///< Base timer ch.2/6
    #define BT3_7_IRQHandler(void)             IRQ027_Handler(void) ///< Base timer ch.3/7
    #define CEC_IRQHandler(void)               IRQ028_Handler(void) ///< CEC ch.0/1
    #define ICC1_FLASH_IRQHandler(void)        IRQ029_Handler(void) ///< ICC ch.1/Flash        
    #define DSTC_IRQHandler(void)              IRQ030_Handler(void) ///< DSTC
//                                             IRQ031_Handler(void) ///< Reserved
        
#elif (PDL_MCU_INT_TYPE == PDL_FM4_INT_TYPE_A)
    #define CSV_IRQHandler(void)               IRQ000_Handler(void) ///< CSV
    #define SWDT_IRQHandler(void)              IRQ001_Handler(void) ///< SW watchdog
    #define LVD_IRQHandler(void)               IRQ002_Handler(void) ///< LVD       
    #define IRQ003SEL_IRQHandler(void)         IRQ003_Handler(void) ///< Interrupt Source Selection 3
    #define IRQ004SEL_IRQHandler(void)         IRQ004_Handler(void) ///< Interrupt Source Selection 4
    #define IRQ005SEL_IRQ_Handler(void)        IRQ005_Handler(void) ///< Interrupt Source Selection 5
    #define IRQ006SEL_IRQHandler(void)         IRQ006_Handler(void) ///< Interrupt Source Selection 6
    #define IRQ007SEL_IRQHandler(void)         IRQ007_Handler(void) ///< Interrupt Source Selection 7
    #define IRQ008SEL_IRQHandler(void)         IRQ008_Handler(void) ///< Interrupt Source Selection 8
    #define IRQ009SEL_IRQHandler(void)         IRQ009_Handler(void) ///< Interrupt Source Selection 9
    #define IRQ010SEL_IRQHandler(void)         IRQ010_Handler(void) ///< Interrupt Source Selection 10
    #define EXINT0_IRQHandler(void)            IRQ011_Handler(void) ///< External Pin Interrupt Ch. 0
    #define EXINT1_IRQHandler(void)            IRQ012_Handler(void) ///< External Pin Interrupt Ch. 1
    #define EXINT2_IRQHandler(void)            IRQ013_Handler(void) ///< External Pin Interrupt Ch. 2
    #define EXINT3_IRQHandler(void)            IRQ014_Handler(void) ///< External Pin Interrupt Ch. 3
    #define EXINT4_IRQHandler(void)            IRQ015_Handler(void) ///< External Pin Interrupt Ch. 4
    #define EXINT5_IRQHandler(void)            IRQ016_Handler(void) ///< External Pin Interrupt Ch. 5
    #define EXINT6_IRQHandler(void)            IRQ017_Handler(void) ///< External Pin Interrupt Ch. 6
    #define EXINT7_IRQHandler(void)            IRQ018_Handler(void) ///< External Pin Interrupt Ch. 7
    #define QPRC0_IRQHandler(void)             IRQ019_Handler(void) ///< Quad Position & Revolution Counter Ch. 0
    #define QPRC1_IRQHandler(void)             IRQ020_Handler(void) ///< Quad Position & Revolution Counter Ch. 1
    #define WFG0_DTIF0_IRQHandler(void)        IRQ021_Handler(void) ///< MFT Unit 0 Waveform Generator / DTIF Ch. 0
    #define WFG1_DTIF1_IRQHandler(void)        IRQ022_Handler(void) ///< MFT Unit 1 Waveform Generator / DTIF Ch. 1
    #define WFG2_DTIF2_IRQHandler(void)        IRQ023_Handler(void) ///< MFT Unit 2 Waveform Generator / DTIF Ch. 2
    #define FRT0_PEAK_IRQHandler(void)         IRQ024_Handler(void) ///< MFT Unit 0 Free-Run Timer Peak Detection
    #define FRT0_ZERO_IRQHandler(void)         IRQ025_Handler(void) ///< MFT Unit 0 Free-Run Timer Zero Detection
    #define ICU0_IRQHandler(void)              IRQ026_Handler(void) ///< MFT Unit 0 Input Capture Unit
    #define OCU0_IRQHandler(void)              IRQ027_Handler(void) ///< MFT Unit 0 Output Compare Unit
    #define FRT1_PEAK_IRQHandler(void)         IRQ028_Handler(void) ///< MFT Unit 1 Free-Run Timer Peak Detection
    #define FRT1_ZERO_IRQHandler(void)         IRQ029_Handler(void) ///< MFT Unit 1 Free-Run Timer Zero Detection
    #define ICU1_IRQHandler(void)              IRQ030_Handler(void) ///< MFT Unit 1 Input Capture Unit
    #define OCU1_IRQHandler(void)              IRQ031_Handler(void) ///< MFT Unit 1 Output Compare Unit    
    #define FRT2_PEAK_IRQHandler(void)         IRQ032_Handler(void) ///< MFT Unit 2 Free-Run Timer Peak Detection
    #define FRT2_ZERO_IRQHandler(void)         IRQ033_Handler(void) ///< MFT Unit 2 Free-Run Timer Zero Detection
    #define ICU2_IRQHandler(void)              IRQ034_Handler(void) ///< MFT Unit 2 Input Capture Unit
    #define OCU2_IRQHandler(void)              IRQ035_Handler(void) ///< MFT Unit 2 Output Compare Unit
    #define PPG00_02_04_IRQHandler(void)       IRQ036_Handler(void) ///< Programmable Pulse Generator Ch. 0, 2, 4
    #define PPG08_10_12_IRQHandler(void)       IRQ037_Handler(void) ///< Programmable Pulse Generator Ch. 8, 10, 12
    #define PPG16_18_20_IRQHandler(void)       IRQ038_Handler(void) ///< Programmable Pulse Generator Ch. 16, 18, 20
    #define BT0_IRQHandler(void)               IRQ039_Handler(void) ///< Base Timer Ch. 0
    #define BT1_IRQHandler(void)               IRQ040_Handler(void) ///< Base Timer Ch. 1
    #define BT2_IRQHandler(void)               IRQ041_Handler(void) ///< Base Timer Ch. 2
    #define BT3_IRQHandler(void)               IRQ042_Handler(void) ///< Base Timer Ch. 3
    #define BT4_IRQHandler(void)               IRQ043_Handler(void) ///< Base Timer Ch. 4
    #define BT5_IRQHandler(void)               IRQ044_Handler(void) ///< Base Timer Ch. 5
    #define BT6_IRQHandler(void)               IRQ045_Handler(void) ///< Base Timer Ch. 6
    #define BT7_IRQHandler(void)               IRQ046_Handler(void) ///< Base Timer Ch. 7
    #define DT1_2_IRQHandler(void)             IRQ047_Handler(void) ///< Dual Timer Ch. 1, 2
    #define WC_IRQHandler(void)                IRQ048_Handler(void) ///< Watch Counter
    #define EXTBUS_ERR_IRQHandler(void)        IRQ049_Handler(void) ///< External Bus Error
    #define RTC_IRQHandler(void)               IRQ050_Handler(void) ///< Real Time Clock
    #define EXINT8_IRQHandler(void)            IRQ051_Handler(void) ///< External Pin Interrupt Ch. 8
    #define EXINT9_IRQHandler(void)            IRQ052_Handler(void) ///< External Pin Interrupt Ch. 9
    #define EXINT10_IRQHandler(void)           IRQ053_Handler(void) ///< External Pin Interrupt Ch. 10
    #define EXINT11_IRQHandler(void)           IRQ054_Handler(void) ///< External Pin Interrupt Ch. 11
    #define EXINT12_IRQHandler(void)           IRQ055_Handler(void) ///< External Pin Interrupt Ch. 12
    #define EXINT13_IRQHandler(void)           IRQ056_Handler(void) ///< External Pin Interrupt Ch. 13
    #define EXINT14_IRQHandler(void)           IRQ057_Handler(void) ///< External Pin Interrupt Ch. 14
    #define EXINT15_IRQHandler(void)           IRQ058_Handler(void) ///< External Pin Interrupt Ch. 15
    #define TIM_IRQHandler(void)               IRQ059_Handler(void) ///< OSC / SUB OSC / PLL / USB/ETHER-PLL
    #define MFS0_RX_IRQHandler(void)           IRQ060_Handler(void) ///< Multi Function Serial Reception Ch. 0
    #define MFS0_TX_IRQHandler(void)           IRQ061_Handler(void) ///< Multi Function Serial Reception/Status Ch. 0
    #define MFS1_RX_IRQHandler(void)           IRQ062_Handler(void) ///< Multi Function Serial Reception Ch. 1
    #define MFS1_TX_IRQHandler(void)           IRQ063_Handler(void) ///< Multi Function Serial Reception/Status Ch. 1
    #define MFS2_RX_IRQHandler(void)           IRQ064_Handler(void) ///< Multi Function Serial Reception Ch. 2
    #define MFS2_TX_IRQHandler(void)           IRQ065_Handler(void) ///< Multi Function Serial Reception/Status Ch. 2
    #define MFS3_RX_IRQHandler(void)           IRQ066_Handler(void) ///< Multi Function Serial Reception Ch. 3
    #define MFS3_TX_IRQHandler(void)           IRQ067_Handler(void) ///< Multi Function Serial Reception/Status Ch. 3
    #define MFS4_RX_IRQHandler(void)           IRQ068_Handler(void) ///< Multi Function Serial Reception Ch. 4
    #define MFS4_TX_IRQHandler(void)           IRQ069_Handler(void) ///< Multi Function Serial Reception/Status Ch. 4
    #define MFS5_RX_IRQHandler(void)           IRQ070_Handler(void) ///< Multi Function Serial Reception Ch. 5
    #define MFS5_TX_IRQHandler(void)           IRQ071_Handler(void) ///< Multi Function Serial Reception/Status Ch. 5
    #define MFS6_RX_IRQHandler(void)           IRQ072_Handler(void) ///< Multi Function Serial Reception Ch. 6
    #define MFS6_TX_IRQHandler(void)           IRQ073_Handler(void) ///< Multi Function Serial Reception/Status Ch. 6
    #define MFS7_RX_IRQHandler(void)           IRQ074_Handler(void) ///< Multi Function Serial Reception Ch. 7
    #define MFS7_TX_IRQHandler(void)           IRQ075_Handler(void) ///< Multi Function Serial Reception/Status Ch. 7
    #define ADC0_IRQHandler(void)              IRQ076_Handler(void) ///< Analog Digital Converter Unit 0
    #define ADC1_IRQHandler(void)              IRQ077_Handler(void) ///< Analog Digital Converter Unit 1
    #define USB0_IRQHandler(void)              IRQ078_Handler(void) ///< USB Ch. 0 (DRQ of endpoint 1 to 5) 
    #define USB0_HOST_IRQHandler(void)         IRQ079_Handler(void) ///< USB Ch. 0 (DRQ of endpoint 0) / USB Host (individual Status)
    #define CAN0_IRQHandler(void)              IRQ080_Handler(void) ///< CAN Ch. 0
    #define CAN1_IRQHandler(void)              IRQ081_Handler(void) ///< CAN Ch. 1
    #define ETHER0_IRQHandler(void)            IRQ082_Handler(void) ///< Ethernet Ch. 0
    #define DMAC0_IRQHandler(void)             IRQ083_Handler(void) ///< DMAC Ch. 0
    #define DMAC1_IRQHandler(void)             IRQ084_Handler(void) ///< DMAC Ch. 1
    #define DMAC2_IRQHandler(void)             IRQ085_Handler(void) ///< DMAC Ch. 2
    #define DMAC3_IRQHandler(void)             IRQ086_Handler(void) ///< DMAC Ch. 3
    #define DMAC4_IRQHandler(void)             IRQ087_Handler(void) ///< DMAC Ch. 4
    #define DMAC5_IRQHandler(void)             IRQ088_Handler(void) ///< DMAC Ch. 5
    #define DMAC6_IRQHandler(void)             IRQ089_Handler(void) ///< DMAC Ch. 6
    #define DMAC7_IRQHandler(void)             IRQ090_Handler(void) ///< DMAC Ch. 7
    #define DSTC_IRQHandler(void)              IRQ091_Handler(void) ///< DSTC SW Transfer Normal/Error
#if (PDL_MCU_TYPE == PDL_FM4_TYPE1) ||  \
    (PDL_MCU_TYPE == PDL_FM4_TYPE2) ||  \
    (PDL_MCU_TYPE == PDL_FM4_TYPE3) ||  \
    (PDL_MCU_TYPE == PDL_FM4_TYPE6)  
    #define EXINT16_17_18_19_IRQHandler(void)  IRQ092_Handler(void) ///< External Pin Interrupt Ch. 16, 17, 18, 19
    #define EXINT20_21_22_23_IRQHandler(void)  IRQ093_Handler(void) ///< External Pin Interrupt Ch. 20, 21, 22, 23
    #define EXINT24_25_26_27_IRQHandler(void)  IRQ094_Handler(void) ///< External Pin Interrupt Ch. 24, 25, 26, 27
    #define EXINT28_29_30_31_IRQHandler(void)  IRQ095_Handler(void) ///< External Pin Interrupt Ch. 28, 29, 30, 31
    #define QPRC2_IRQHandler(void)             IRQ096_Handler(void) ///< Quad Position & Revolution Counter Ch. 2
    #define QPRC3_IRQHandler(void)             IRQ097_Handler(void) ///< Quad Position & Revolution Counter Ch. 3
    #define BT8_IRQHandler(void)               IRQ098_Handler(void) ///< Base Timer Ch. 8
    #define BT9_IRQHandler(void)               IRQ099_Handler(void) ///< Base Timer Ch. 9
    #define BT10_IRQHandler(void)              IRQ100_Handler(void) ///< Base Timer Ch. 10
    #define BT11_IRQHandler(void)              IRQ101_Handler(void) ///< Base Timer Ch. 11
    #define BT12_13_14_15_IRQHandler(void)     IRQ102_Handler(void) ///< Base Timer Ch. 12 - 15
    #define MFS8_RX_IRQHandler(void)           IRQ103_Handler(void) ///< Multi Function Serial Reception Ch. 8
    #define MFS8_TX_IRQHandler(void)           IRQ104_Handler(void) ///< Multi Function Serial Reception/Status Ch. 8
    #define MFS9_RX_IRQHandler(void)           IRQ105_Handler(void) ///< Multi Function Serial Reception Ch. 9
    #define MFS9_TX_IRQHandler(void)           IRQ106_Handler(void) ///< Multi Function Serial Reception/Status Ch. 9
    #define MFS10_RX_IRQHandler(void)          IRQ107_Handler(void) ///< Multi Function Serial Reception Ch. 10
    #define MFS10_TX_IRQHandler(void)          IRQ108_Handler(void) ///< Multi Function Serial Reception/Status Ch. 10
    #define MFS11_RX_IRQHandler(void)          IRQ109_Handler(void) ///< Multi Function Serial Reception Ch. 11
    #define MFS11_TX_IRQHandler(void)          IRQ110_Handler(void) ///< Multi Function Serial Reception/Status Ch. 11
    #define ADC2_IRQHandler(void)              IRQ111_Handler(void) ///< Analog Digital Converter Unit 2
    #define DSTC_HWINT_NEW_IRQHandler(void)    IRQ112_Handler(void) ///< DSTC hardware interrupt ch.218 - 223
    #define USB1_IRQHandler(void)              IRQ113_Handler(void) ///< USB Ch. 1 (DRQ of endpoint 1 to 5) 
    #define USB1_HOST_IRQHandler(void)         IRQ114_Handler(void) ///< USB Ch. 1 (DRQ of endpoint 0) / USB Host (individual Status)
    #define HSSPI0_IRQHandler(void)            IRQ115_Handler(void) ///< Quad-SPI
//                                             IRQ116_Handler(void)      RESERVED 
    #define I2S_PCRC_IRQHandler(void)          IRQ117_Handler(void) ///< I2S and programmable-CRC
    #define SD_IRQHandler(void)                IRQ118_Handler(void) ///< SD Card
    #define FLASH_IRQHandler(void)             IRQ119_Handler(void) ///< Flash Interface
    #define MFS12_RX_IRQHandler(void)          IRQ120_Handler(void) ///< Multi Function Serial Reception Ch. 12
    #define MFS12_TX_IRQHandler(void)          IRQ121_Handler(void) ///< Multi Function Serial Reception/Status Ch. 12
    #define MFS13_RX_IRQHandler(void)          IRQ122_Handler(void) ///< Multi Function Serial Reception Ch. 13
    #define MFS13_TX_IRQHandler(void)          IRQ123_Handler(void) ///< Multi Function Serial Reception/Status Ch. 13
    #define MFS14_RX_IRQHandler(void)          IRQ124_Handler(void) ///< Multi Function Serial Reception Ch. 14
    #define MFS14_TX_IRQHandler(void)          IRQ125_Handler(void) ///< Multi Function Serial Reception/Status Ch. 14
    #define MFS15_RX_IRQHandler(void)          IRQ126_Handler(void) ///< Multi Function Serial Reception Ch. 15
    #define MFS15_TX_IRQHandler(void)          IRQ127_Handler(void) ///< Multi Function Serial Reception/Status Ch. 15    
#elif (PDL_MCU_TYPE == PDL_FM4_TYPE4)
    #define DSTC_HWINT_NEW_IRQHandler(void)     IRQ112_Handler(void) ///< DSTC hardware interrupt ch.120~125
    #define I2S_PCRC_IRQHandler(void)           IRQ117_Handler(void) ///< Programmable CRC/I2S
    #define SD_IRQHandler(void)                 IRQ118_Handler(void) ///< SD I/F
    #define FLASH_IRQHandler(void)              IRQ119_Handler(void) ///< Flash I/F 
    #define HSSPI0_RX_IRQHandler(void)          IRQ120_Handler(void) ///< High Speed Quad SPI reception interrupt
    #define HSSPI0_TX_IRQHandler(void)          IRQ121_Handler(void) ///< High Speed Quad SPI transmission interrupt
    #define HSSPI0_FAULT_IRQHandler(void)       IRQ122_Handler(void) ///< High Speed Quad SPI fault interrupt
    #define HBIF_IRQHandler(void)               IRQ123_Handler(void) ///< Hyper bus interface interrupt 
#elif (PDL_MCU_TYPE == PDL_FM4_TYPE5)
    #define EXINT16_17_18_19_IRQHandler(void)  IRQ092_Handler(void) ///< External Pin Interrupt Ch. 16, 17, 18, 19
    #define EXINT20_21_22_23_IRQHandler(void)  IRQ093_Handler(void) ///< External Pin Interrupt Ch. 20, 21, 22, 23
    #define EXINT24_25_26_27_IRQHandler(void)  IRQ094_Handler(void) ///< External Pin Interrupt Ch. 24, 25, 26, 27
    #define EXINT28_29_30_31_IRQHandler(void)  IRQ095_Handler(void) ///< External Pin Interrupt Ch. 28, 29, 30, 31
    #define BT8_IRQHandler(void)               IRQ098_Handler(void) ///< Base Timer Ch. 8
    #define BT9_IRQHandler(void)               IRQ099_Handler(void) ///< Base Timer Ch. 9
    #define BT10_IRQHandler(void)              IRQ100_Handler(void) ///< Base Timer Ch. 10
    #define BT11_IRQHandler(void)              IRQ101_Handler(void) ///< Base Timer Ch. 11
    #define BT12_13_14_15_IRQHandler(void)     IRQ102_Handler(void) ///< Base Timer Ch. 12 - 15
    #define MFS8_RX_IRQHandler(void)           IRQ103_Handler(void) ///< Multi Function Serial Reception Ch. 8
    #define MFS8_TX_IRQHandler(void)           IRQ104_Handler(void) ///< Multi Function Serial Reception/Status Ch. 8
    #define MFS9_RX_IRQHandler(void)           IRQ105_Handler(void) ///< Multi Function Serial Reception Ch. 9
    #define MFS9_TX_IRQHandler(void)           IRQ106_Handler(void) ///< Multi Function Serial Reception/Status Ch. 9
    #define ADC2_IRQHandler(void)              IRQ111_Handler(void) ///< Analog Digital Converter Unit 2
    #define USB1_IRQHandler(void)              IRQ113_Handler(void) ///< USB Ch. 1 (DRQ of endpoint 1 to 5) 
    #define USB1_HOST_IRQHandler(void)         IRQ114_Handler(void) ///< USB Ch. 1 (DRQ of endpoint 0) / USB Host (individual Status)  
    #define ICC0_ICC1_IRQHandler(void)         IRQ117_Handler(void) ///< ICC0/ICC1     
    #define SD_IRQHandler(void)                IRQ118_Handler(void) ///< SD Card 
    #define FLASH_IRQHandler(void)             IRQ119_Handler(void) ///< Flash Interface  
#else
#error Device type not found!      
#endif      
#else
    #error Interrupt Type not found!
#endif

/******************************************************************************/
/* Global type definitions ('typedef')                                        */
/******************************************************************************/
/**
 ******************************************************************************
 ** \brief Level
 **
 ** Specifies levels.
 **
 ******************************************************************************/
typedef enum en_level
{
    PdlLow      = 0u,  ///< Low level '0'
    PdlHigh     = 1u   ///< High level '1'
} en_level_t;

/**
 ******************************************************************************
 ** \brief Generic Flag Code
 **
 ** Specifies flags.
 **
 ******************************************************************************/
typedef enum en_flag
{
    PdlClr = 0u,       ///< Flag clr '0'
    PdlSet = 1u        ///< Flag set '1'
} en_stat_flag_t, en_irq_flag_t;
/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

/******************************************************************************
 * Global function prototypes
 ******************************************************************************/
extern void pdl_memclr(uint8_t* pu32Address, uint32_t u32Count);

/**
 ******************************************************************************
 ** This hook is part of wait loops.
 ******************************************************************************/
extern void PDL_WAIT_LOOP_HOOK(void);

#ifdef __cplusplus
}
#endif

#endif /* __PDL_H__ */

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/

