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
/** \file vbat.h
 **
 ** Headerfile for VBAT functions
 **  
 ** History:
 **   - 2014-12-13  0.1  EZh  First version for FM universal PDL.
 **
 ******************************************************************************/

#ifndef __VBAT_H__
#define __VBAT_H__

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "mcu.h"
#include "pdl_user.h"

#if (defined(PDL_PERIPHERAL_VBAT_ACTIVE))

#if (PDL_PERIPHERAL_VBAT_AVAILABLE == PDL_ON)

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
  
/**
 ******************************************************************************
 ** \defgroup VbatGroup VBAT Doamin (VBAT)
 ** 
 ** Provided functions of VBAT module:
 **
 ** - Vbat_Init()
 ** - Vbat_SetPinFunc_VREGCTL()
 ** - Vbat_SetPinFunc_VWAKEUP()
 ** - Vbat_SetPinFunc_X0A_X1A()
 ** - Vbat_InitGpioOutput()
 ** - Vbat_InitGpioInput() 
 ** - Vbat_PutPinP46() 
 ** - Vbat_GetPinP46() 
 ** - Vbat_PutPinP47() 
 ** - Vbat_GetPinP47()   
 ** - Vbat_PutPinP48() 
 ** - Vbat_GetPinP48()   
 ** - Vbat_PutPinP49() 
 ** - Vbat_GetPinP49()
 ** - Vbat_StartHibernation()  
 ** - Vbat_GetPowerOnFlag()
 ** - Vbat_GetWakeupFlag()
 ** - Vbat_ClrWakeupFlag()  
 **
 **  Vbat_Init() clears VBAT power on flag and set transfer clock division, 
 **  sub oscillator configuraiton, and boost time current.
 **
 **  Vbat_SetPinFunc_VREGCTL() set the pin function of P48/VREGCTL to VREGCTL.
 ** 
 **  Vbat_SetPinFunc_VWAKEUP() set the pin function of P49/VWAKEUP to VWAKEUP.
 ** 
 **  Vbat_SetPinFunc_X0A_X1A() set  the pin function of P46/X0A, P47/X0A to 
 **  X0A, X1A. Note that it is default setting for these two pins.
 ** 
 **  Vbat_InitGpioOutput() intializes a VBAT GPIO pin to output selected by 
 **  the parameter Vbat_InitGpioOutput#enGpio. 
 **
 **  Vbat_InitGpioInput() intializes a VBAT GPIO pin to input selected by 
 **  the parameter Vbat_InitGpioOutput#enGpio.   
 ** 
 **  Vbat_PutPinP46() set the output level of P46. Vbat_GetPinP46() get the 
 **  level of P46.
 **
 **  Vbat_PutPinP47() set the output level of P47. Vbat_GetPinP47() get the 
 **  level of P47.
 **
 **  Vbat_PutPinP48() set the output level of P48. Vbat_GetPinP48() get the 
 **  level of P48.
 **  
 **  Vbat_PutPinP49() set the output level of P49. Vbat_GetPinP49() get the 
 **  level of P49. 
 **
 **  Vbat_StartHibernation() start hibernation funciton.
 **
 **  Vbat_GetPowerOnFlag() reads the power flag of VBAT, which will be set when
 **  VBAT is power on.
 **
 **  Vbat_GetWakeupFlag() get the wakeup flag which will be set when wakeup from
 **  hibernation.
 **  
 **  Vbat_ClrWakeupFlag() is used to clear the wakeup flag.
 **
 ******************************************************************************/
//@{

/******************************************************************************/
/* Global pre-processor symbols/macros ('#define')      
 ******************************************************************************/

#if (PDL_MCU_TYPE == PDL_FM4_TYPE1) || \
    (PDL_MCU_TYPE == PDL_FM4_TYPE2) || \
    (PDL_MCU_TYPE == PDL_FM4_TYPE3)   
/**
 ******************************************************************************
 ** \brief Sustain and Boost Current
 ******************************************************************************/
typedef enum en_clk_current
{
    Clk0nA          = 0u,  ///< 0 nA sustain/boost, not allowed, if subclock is enabled
    Clk385nA        = 1u,  ///< 385 nA
    Clk445nA        = 2u,  ///< 445 nA, initial value for current sustain
    Clk510nA        = 3u,  ///< 510 nA, initial value for current boost
    ClkErrorCurrent = 4u,  ///< Errornous setting (for read-out function)
} en_clk_current_t;
#else
typedef enum en_clk_current
{
    ClkOff          = 0u,   ///< Clock is off
    Clk100nA        = 1u,   ///< 100 nA
    Clk140nA        = 2u,   ///< 140 nA
    Clk160nA        = 3u,   ///< 160 nA
    Clk200nA        = 4u,   ///< 200 nA
    Clk240nA        = 5u,   ///< 240 nA
    Clk280nA        = 6u,   ///< 280 nA
    Clk300nA        = 7u,   ///< 300 nA    
    Clk320nA        = 8u,   ///< 320 nA
    Clk400nA        = 9u,   ///< 400 nA
    Clk420nA        = 10u,  ///< 420 nA    
    Clk480nA        = 11u,  ///< 480 nA
    Clk500nA        = 12u,  ///< 560 nA    
    Clk560nA        = 13u,  ///< 640 nA
    Clk600nA        = 14u,  ///< 600 nA    
    Clk640nA        = 15u,  ///< 640 nA
    Clk700nA        = 16u,  ///< 700 nA    
    Clk720nA        = 17u,  ///< 720 nA 
    Clk800nA        = 18u,  ///< 800 nA
    Clk840nA        = 19u,  ///< 840 nA 
    Clk960nA        = 20u,  ///< 960 nA 
    Clk1000nA       = 21u,  ///< 1000 nA
    Clk1200nA       = 22u,  ///< 1200 nA
    Clk1400nA       = 23u,  ///< 1400 nA
    Clk1600nA       = 24u,  ///< 1600 nA 

} en_clk_current_t;
#endif

/**
 ******************************************************************************
 ** \brief Sustain and Boost Time
 ******************************************************************************/
typedef enum en_clk_boost_time
{
    ClkBoost50ms  = 0u,    ///< Boost time 50 ms (initial value)
    ClkBoost63ms  = 1u,    ///< Boost time 62.5 ms
    ClkBoost125ms = 2u,    ///< Boost time 125 ms
    ClkBoost250ms = 3u     ///< Boost time 250 ms
} en_clk_boost_time_t;

/**
 ******************************************************************************
 ** \brief VBAT configuration
 *******************************************************************************/
typedef struct stc_vbat_config
{
    uint8_t    u8ClkDiv;     ///< Transfer clock division of PREAD, PWRITE, BREAD, BWRITE
    boolean_t  bLinkSubClk;  ///< FALSE: The 32 kHz oscillation circuit operates independently as VBAT Dmain
                             ///< TRUE:  The 32 kHz oscillation circuit is linked with the clock control circuit.
    boolean_t  bVbatClkEn;   ///< FALSE: Stops the oscillation of VBAT  
                             ///< TRUE: Starts the oscillation of VBAT
    en_clk_current_t enSustainCurrent;  ///< Sustain current of sub clock 
    en_clk_current_t enBoostCurrent;    ///< Boost current of sub clock 
    en_clk_boost_time_t enClkBoostTime; ///< Boost time of sub clock 
    
}stc_vbat_config_t;

/**
 ******************************************************************************
 ** \brief VBAT GPIO selection
 *******************************************************************************/
typedef enum en_vbat_gpio
{
    VbatGpioP46 = 0u,  ///< P46 pin
    VbatGpioP47 = 1u,  ///< P47 pin
    VbatGpioP48 = 2u,  ///< P48 pin
    VbatGpioP49 = 3u,  ///< P49 pin
    
}en_vbat_gpio_t;

/******************************************************************************/
/* Global variable definitions ('extern')                                     */
/******************************************************************************/

/******************************************************************************/
/* Global function prototypes (definition in C source)                        */
/******************************************************************************/
/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
/* VBAT init */
en_result_t Vbat_Init(stc_vbat_config_t* pstcConfig);
/* Function pin setting */
void Vbat_SetPinFunc_VREGCTL(void);
void Vbat_SetPinFunc_VWAKEUP(void);
void Vbat_SetPinFunc_X0A_X1A(void);
/* GPIO setting */
en_result_t Vbat_InitGpioOutput(en_vbat_gpio_t enGpio, 
                                boolean_t bInitLevel, 
                                boolean_t bOpenDrain);
en_result_t Vbat_InitGpioInput(en_vbat_gpio_t enGpio, boolean_t bPullup);
void Vbat_PutPinP46(boolean_t bLevel);
void Vbat_PutPinP47(boolean_t bLevel);
void Vbat_PutPinP48(boolean_t bLevel);
void Vbat_PutPinP49(boolean_t bLevel);
boolean_t Vbat_GetPinP46(void);
boolean_t Vbat_GetPinP47(void);
boolean_t Vbat_GetPinP48(void);
boolean_t Vbat_GetPinP49(void);
/* Start hibernation function */
void Vbat_StartHibernation(void);
/* Get VBAT power on flag */
boolean_t Vbat_GetPowerOnFlag(void);
/* Get/clear hibernation wakeup flag */
boolean_t Vbat_GetWakeupFlag(void);
void Vbat_ClrWakeupFlag(void);

//@} // VbatGroup

#ifdef __cplusplus
}
#endif

#endif

#endif // #if (defined(PDL_PERIPHERAL_VBAT_ACTIVE))

#endif /* __LPM_H__ */
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
