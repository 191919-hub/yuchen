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
/** \file clk.h
 **
 ** Headerfile for CLK functions
 **  
 ** History:
 **   - 2014-11-14  1.0  EZh  First version for FM universal PDL.
 **
 ******************************************************************************/

#ifndef __CLK_H__
#define __CLK_H__

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "mcu.h"
#include "pdl_user.h"

#if (defined(PDL_PERIPHERAL_CLK_ACTIVE))

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
  
/**
 ******************************************************************************
 ** \defgroup ClkGroup Clock Functions (CLK)
 **  
 ** Provided CLK module functions:
 ** 
 ** - Clk_Init()
 ** - Clk_EnableHscr() 
 ** - Clk_DisableHscr()
 ** - Clk_EnableMainClock()
 ** - Clk_DisableMainClock()
 ** - Clk_EnableSubClock()
 ** - Clk_DisableSubClock()
 ** - Clk_EnablePllClock()
 ** - Clk_DisablePllClock()
 ** - Clk_SetSource()
 ** - Clk_PeripheralClockEnable()
 ** - Clk_PeripheralClockDisable()
 ** - Clk_PeripheralClockEnableAll()
 ** - Clk_PeripheralClockDisableAll()  
 ** - Clk_PeripheralGetClockState()
 ** - Clk_PeripheralSetReset()
 ** - Clk_PeripheralClearReset()
 **  
 ** Clk_Init() initializes the system clock according to the configuration
 ** of the structure type #stc_clk_config_t. This function provides another
 ** method to configure system clock beside the CMSIS way.
 **
 ** Clk_EnableHscr() enables the high speed CR and Clk_DisableHscr() disables
 ** high speed CR.
 **
 ** Clk_EnableMainClock() enables main clock and Clk_DisableMainClock() disables
 ** main clock.
 **
 ** Clk_EnableSubClock() enables sub clock and Clk_DisableSubClock() disables
 ** sub clock.
 **
 ** Clk_EnablePllClock() enables PLL clock and Clk_DisablePllClock() disables
 ** PLL clock.  
 **  
 ** Clk_SetSource() set the source clock for system clock according to the 
 ** the enumeration type #en_clk_source_t. Before calling this funciton, the
 ** used clock should be enabled.
 **
 ** Clk_PeripheralClockEnable() enables the clock supply to the peripheral 
 ** selected and Clk_PeripheralClockDisable() disables the clock supply to 
 ** the peripheral selected. At default status, the clock is supplied.
 ** Clk_PeripheralGetClockState() reads the clock status of the peripheral 
 ** selected.
 ** 
 ** Clk_PeripheralClockEnableAll() Enables the clock supply of all peripherals.
 ** Clk_PeripheralClockDisableAll() disables the clock supply of all 
 ** peripherals.
 **  
 ** Calling Clk_PeripheralSetReset() first and then Clk_PeripheralClearReset()
 ** can reset a peripheral.  
 **   
 ******************************************************************************/
//@{
  
/******************************************************************************
 * Global type definitions
 ******************************************************************************/
// PLL Input Clock Select (PINC) Bit Position
#define FM_PSW_TMR_PINC_BITPOS 0x10u
  
// Interrupt enable bitpositions in INT_ENR
#define FM_INT_ENR_PCSE_BITPOS 0x04u
#define FM_INT_ENR_SCSE_BITPOS 0x02u
#define FM_INT_ENR_MCSE_BITPOS 0x01u
  
// Interrupt status bitpositions in INT_STR
#define FM_INT_STR_PCSI_BITPOS 0x04u
#define FM_INT_STR_SCSI_BITPOS 0x02u
#define FM_INT_STR_MCSI_BITPOS 0x01u
  
// Interrupt ckear bitpositions in INT_CLR
#define FM_INT_CLR_PCSC_BITPOS 0x04u
#define FM_INT_CLR_SCSC_BITPOS 0x02u
#define FM_INT_CLR_MCSC_BITPOS 0x01u
  
/**
 ******************************************************************************
 ** \brief Clock Source
 ** 
 ** Differentiator for the different clock sources
 **
 ** \note The enumerated values do not correspond to the RCS/RCM bits of the
 **       clock control and status registers due to having upward compatibility,
 **       if this bit coding may change in future devices. The correct bit
 **       patterns are set by switch(en_clk_source)/case statements individually
 **       in the corresponding function(s).
 ******************************************************************************/
typedef enum en_clk_source
{
    ClkMain = 0u,  ///< Main Clock Oscillator
    ClkSub  = 1u,  ///< Sub Clock Oscillator
    ClkHsCr = 2u,  ///< High-Speed CR Clock Oscillator
    ClkLsCr = 3u,  ///< Low-Speed CR Clock Oscillator
    ClkPll  = 4u,  ///< PLL Clock
    ClkHsCrPll = 5u   ///< High-Speed CR PLL Clock
} en_clk_source_t;

/**
 ******************************************************************************
 ** \brief Base Clock Prescaler Settings
 ** 
 ** Enumeration of the dividers of the Base Clock (HCLK)
 ******************************************************************************/
typedef enum en_clk_baseclkdiv
{
    BaseClkDiv1  = 0u,  ///< HCLK Division 1/1
    BaseClkDiv2  = 1u,  ///< HCLK Division 1/2
    BaseClkDiv3  = 2u,  ///< HCLK Division 1/3
    BaseClkDiv4  = 3u,  ///< HCLK Division 1/4
    BaseClkDiv6  = 4u,  ///< HCLK Division 1/6
    BaseClkDiv8  = 5u,  ///< HCLK Division 1/8    
    BaseClkDiv16 = 6u,  ///< HCLK Division 1/16
    BaseClkErr   = 7u   ///< HCLK prohibited setting
} en_clk_baseclkdiv_t;

/**
 ******************************************************************************
 ** \brief APB0 Prescaler Settings
 ** 
 ** Enumeration of the dividers of the APB0 (PCLK0)
 ******************************************************************************/
typedef enum en_clk_apb0div
{
    Apb0Div1  = 0u,  ///< PCLK0 Division 1/1
    Apb0Div2  = 1u,  ///< PCLK0 Division 1/2
    Apb0Div4  = 2u,  ///< PCLK0 Division 1/4
    Apb0Div8  = 3u   ///< PCLK0 Division 1/8
} en_clk_apb0div_t;

/**
 ******************************************************************************
 ** \brief APB1 Prescaler Settings
 ** 
 ** Enumeration of the dividers of the APB0 (PCLK1)
 ******************************************************************************/
typedef enum en_clk_apb1div
{
    Apb1Div1  = 0u,  ///< PCLK1 Division 1/1
    Apb1Div2  = 1u,  ///< PCLK1 Division 1/2
    Apb1Div4  = 2u,  ///< PCLK1 Division 1/4
    Apb1Div8  = 3u   ///< PCLK1 Division 1/8
} en_clk_apb1div_t;

#if (PDL_MCU_CORE == PDL_FM3_CORE) || (PDL_MCU_CORE == PDL_FM4_CORE)    
/**
 ******************************************************************************
 ** \brief APB2 Prescaler Settings
 ** 
 ** Enumeration of the dividers of the APB0 (PCLK2)
 ******************************************************************************/
typedef enum en_clk_apb2div
{
    Apb2Div1  = 0u,  ///< PCLK2 Division 1/1
    Apb2Div2  = 1u,  ///< PCLK2 Division 1/2
    Apb2Div4  = 2u,  ///< PCLK2 Division 1/4
    Apb2Div8  = 3u   ///< PCLK2 Division 1/8
} en_clk_apb2div_t;
#endif

/**
 ******************************************************************************
 ** \brief Sub Clock oscillation stablilization wait time
 ** 
 ** Enumeration for the Sub Clock oscillation stabilization wait time settings
 ******************************************************************************/
typedef enum en_clk_scowaittime
{
    ScoWaitExp10  =  0u,  ///< 2^10 / F(CL) => ~10.3  ms
    ScoWaitExp11  =  1u,  ///< 2^11 / F(CL) => ~20.5  ms
    ScoWaitExp12  =  2u,  ///< 2^12 / F(CL) => ~41    ms
    ScoWaitExp13  =  3u,  ///< 2^13 / F(CL) => ~82    ms
    ScoWaitExp14  =  4u,  ///< 2^14 / F(CL) => ~164   ms
    ScoWaitExp15  =  5u,  ///< 2^15 / F(CL) => ~327   ms
    ScoWaitExp16  =  6u,  ///< 2^16 / F(CL) => ~655   ms
    ScoWaitExp17  =  7u,  ///< 2^17 / F(CL) => ~1.31  s      
    ScoWaitExp18  =  8u,  ///< 2^17 / F(CL) => ~2.62  s
    ScoWaitExp19  =  9u,  ///< 2^17 / F(CL) => ~5.24  s 
    ScoWaitExp20  = 10u,  ///< 2^17 / F(CL) => ~10.48 s
    ScoWaitExp21  = 11u,  ///< 2^17 / F(CL) => ~20.96 s 
    ScoWaitErr    = 12u   ///< Prohibited Setting
} en_clk_scowaittime_t;

/**
 ******************************************************************************
 ** \brief Main Clock oscillation stablilization wait time
 ** 
 ** Enumeration for the Main Clock oscillation stabilization wait time settings
 ******************************************************************************/
typedef enum en_clk_mcowaittime
{
    McoWaitExp11   = 0u,   ///< 2^1  / F(CH) => ~500  ns (F(CH) = 4 MHz)
    McoWaitExp15   = 1u,   ///< 2^5  / F(CH) => ~8    us (F(CH) = 4 MHz)
    McoWaitExp16   = 2u,   ///< 2^6  / F(CH) => ~16   us (F(CH) = 4 MHz)
    McoWaitExp17   = 3u,   ///< 2^7  / F(CH) => ~32   us (F(CH) = 4 MHz)
    McoWaitExp18   = 4u,   ///< 2^8  / F(CH) => ~64   us (F(CH) = 4 MHz)
    McoWaitExp19   = 5u,   ///< 2^9  / F(CH) => ~128  us (F(CH) = 4 MHz)
    McoWaitExp110  = 6u,   ///< 2^10 / F(CH) => ~256  us (F(CH) = 4 MHz)
    McoWaitExp111  = 7u,   ///< 2^11 / F(CH) => ~512  us (F(CH) = 4 MHz)
    McoWaitExp112  = 8u,   ///< 2^12 / F(CH) => ~1.0  ms (F(CH) = 4 MHz)
    McoWaitExp113  = 9u,   ///< 2^13 / F(CH) => ~2.0  ms (F(CH) = 4 MHz)
    McoWaitExp114  = 10u,  ///< 2^14 / F(CH) => ~4.0  ms (F(CH) = 4 MHz)
    McoWaitExp115  = 11u,  ///< 2^15 / F(CH) => ~8.0  ms (F(CH) = 4 MHz)
    McoWaitExp117  = 12u,  ///< 2^17 / F(CH) => ~33.0 ms (F(CH) = 4 MHz)
    McoWaitExp119  = 13u,  ///< 2^18 / F(CH) => ~131  ms (F(CH) = 4 MHz)
    McoWaitExp121  = 14u,  ///< 2^21 / F(CH) => ~524  ms (F(CH) = 4 MHz)
    McoWaitExp123  = 15u   ///< 2^23 / F(CH) => ~2.0  s  (F(CH) = 4 MHz)
} en_clk_mcowaittime_t;

/**
 ******************************************************************************
 ** \brief PLL Clock oscillation stablilization wait time
 ** 
 ** Enumeration for the PLL Clock oscillation stabilization wait time settings
 ******************************************************************************/
typedef enum en_clk_pllowaittime
{
    PlloWaitExp19   = 0u,   ///< 2^9  / F(CH) => ~128  us (F(CH) = 4 MHz)
    PlloWaitExp110  = 1u,   ///< 2^10 / F(CH) => ~256  us (F(CH) = 4 MHz)
    PlloWaitExp111  = 2u,   ///< 2^11 / F(CH) => ~512  us (F(CH) = 4 MHz)
    PlloWaitExp112  = 3u,   ///< 2^12 / F(CH) => ~1.02 ms (F(CH) = 4 MHz)
    PlloWaitExp113  = 4u,   ///< 2^13 / F(CH) => ~2.05 ms (F(CH) = 4 MHz)
    PlloWaitExp114  = 5u,   ///< 2^14 / F(CH) => ~4.10 ms (F(CH) = 4 MHz)
    PlloWaitExp115  = 6u,   ///< 2^15 / F(CH) => ~8.20 ms (F(CH) = 4 MHz)
    PlloWaitExp116  = 7u    ///< 2^16 / F(CH) => ~16.4 ms (F(CH) = 4 MHz)
} en_clk_pllowaittime_t;

/**
 ******************************************************************************
 ** \brief PLL SourceClock (PINC bit of PSW_TMR)
 ** 
 ** Enumeration for the PLL Clock Source.
 **
 ** \attention HS-RC source only available, if used device supports PLL-CLKHC
 **            setting! PINC bit availability is <b>not</b> checked in this
 **            driver!
 ******************************************************************************/
typedef enum en_clk_pll_src
{
    PllSrcClkMo = 0u,   ///< Use Main Clock as PLL source (always available, default)
    PllSrcClkHc = 123u  ///< Use HS-RC Clock as PLL source (only if available!)
} en_clk_pll_src_t;

/**
 ******************************************************************************
 ** \brief Datatype for holding internal data needed for CLK
 ******************************************************************************/
#if (PDL_INTERRUPT_ENABLE_CLK == PDL_ON)
typedef struct stc_clk_intern_data
{
    func_ptr_t            pfnPllStabCb;     ///< PLL stabilization callback function pointer
    func_ptr_t            pfnScoStabCb;     ///< Sub Clock stabilization callback function pointer
    func_ptr_t            pfnMcoStabCb;     ///< Main Clock stabilization callback function pointer
} stc_clk_intern_data_t ;
#endif

/**
 ******************************************************************************
 ** \brief Clock Gate peripheral enumerators
 **  
 ** \note Though all peripherals are listed here, it doesn't mean a certain  
 **       product has all these peripheral, please check the at "product lineup" 
 **       section in the product data sheet for which peripheral are available.
 ******************************************************************************/
typedef enum en_clk_gate_peripheral
{
    ClkGateGpio   = 0u,    ///< GPIO clock gate
    ClkGateExtif  = 1u,    ///< External bus clock gate
    ClkGateDma    = 2u,    ///< DMA clock gate
    ClkGateAdc0   = 3u,    ///< ADC0 clock gate
    ClkGateAdc1   = 4u,    ///< ADC1 clock gate
    ClkGateAdc2   = 5u,    ///< ADC2 clock gate
    ClkGateAdc3   = 6u,    ///< ADC3 clock gate
    ClkGateMfs0   = 7u,    ///< MFS0 clock gate
    ClkGateMfs1   = 8u,    ///< MFS1 clock gate
    ClkGateMfs2   = 9u,    ///< MFS2 clock gate
    ClkGateMfs3   = 10u,   ///< MFS3 clock gate
    ClkGateMfs4   = 11u,   ///< MFS4 clock gate
    ClkGateMfs5   = 12u,   ///< MFS5 clock gate
    ClkGateMfs6   = 13u,   ///< MFS6 clock gate
    ClkGateMfs7   = 14u,   ///< MFS7 clock gate
    ClkGateMfs8   = 15u,   ///< MFS8 clock gate
    ClkGateMfs9   = 16u,   ///< MFS9 clock gate
    ClkGateMfs10  = 17u,   ///< MFS10 clock gate
    ClkGateMfs11  = 18u,   ///< MFS11 clock gate
    ClkGateMfs12  = 19u,   ///< MFS12 clock gate
    ClkGateMfs13  = 20u,   ///< MFS13 clock gate
    ClkGateMfs14  = 21u,   ///< MFS14 clock gate
    ClkGateMfs15  = 22u,   ///< MFS15 clock gate
    ClkGateQprc0  = 23u,   ///< QPRC0 clock gate
    ClkGateQprc1  = 24u,   ///< QPRC1 clock gate
    ClkGateQprc2  = 25u,   ///< QPRC2 clock gate
    ClkGateQprc3  = 26u,   ///< QPRC3 clock gate
    ClkGateMft0   = 27u,   ///< MFT0, PPG0/2/4/6 clock gate
    ClkGateMft1   = 28u,   ///< MFT1, PPG8/10/12/14 clock gate
    ClkGateMft2   = 29u,   ///< MFT2, PPG16/18/20/22 clock gate
    ClkGateMft3   = 30u,   ///< MFT3, PPG24/26/28/30 clock gate
    ClkGateBt0    = 31u,   ///< BT0/1/2/3 clock gate
    ClkGateBt4    = 32u,   ///< BT4/5/6/7 clock gate
    ClkGateBt8    = 33u,   ///< BT8/9/10/11 clock gate
    ClkGateBt12   = 34u,   ///< BT12/13/14/15 clock gate
    ClkGateQspi   = 35u,   ///< Quad SPI clock gate
    ClkGateCec0   = 36u,   ///< HDMI-CEC0 clock gate
    ClkGateCec1   = 37u,   ///< HDMI-CEC1 clock gate
    ClkGateCrc    = 38u,   ///< CRC clock gate
    ClkGateI2s0   = 39u,   ///< I2S0 clock gate
    ClkGateI2s1   = 40u,   ///< I2S1 clock gate    
    ClkGateSd     = 41u,   ///< SD clock gate
    ClkGateCan0   = 42u,   ///< CAN0 clock gate
    ClkGateCan1   = 43u,   ///< CAN1 clock gate
    ClkGateCan2   = 44u,   ///< CAN2 clock gate
    ClkGateUsb0   = 45u,   ///< USB0 clock gate
    ClkGateUsb1   = 46u,   ///< USB1 clock gate
    ClkGateIcc0   = 47u,   ///< IC-Card interface 0
    ClkGateIcc1   = 48u,   ///< IC-Card interface 1
    ClkGateI2sl0  = 49u,   ///< I2S-Lite 0
    ClkGateI2sl1  = 50u,   ///< I2S-Lite 1
    
} en_clk_gate_peripheral_t;

/**
 ******************************************************************************
 ** \brief Reset peripheral enumerators
 **
 ** \note Though all peripherals are listed here, it doesn't mean a certain  
 **       product has all these peripheral, please check the at "product lineup" 
 **       section in the product data sheet for which peripheral are available. 
 ******************************************************************************/
typedef enum en_clk_reset_peripheral
{
    ClkResetExtif  = 1u,    ///< External bus Reset
    ClkResetDma    = 2u,    ///< DMA Reset
    ClkResetAdc0   = 3u,    ///< ADC0 Reset
    ClkResetAdc1   = 4u,    ///< ADC1 Reset
    ClkResetAdc2   = 5u,    ///< ADC2 Reset
    ClkResetAdc3   = 6u,    ///< ADC3 Reset
    ClkResetMfs0   = 7u,    ///< MFS0 Reset
    ClkResetMfs1   = 8u,    ///< MFS1 Reset
    ClkResetMfs2   = 9u,    ///< MFS2 Reset
    ClkResetMfs3   = 10u,   ///< MFS3 Reset
    ClkResetMfs4   = 11u,   ///< MFS4 Reset
    ClkResetMfs5   = 12u,   ///< MFS5 Reset
    ClkResetMfs6   = 13u,   ///< MFS6 Reset
    ClkResetMfs7   = 14u,   ///< MFS7 Reset
    ClkResetMfs8   = 15u,   ///< MFS8 Reset
    ClkResetMfs9   = 16u,   ///< MFS9 Reset
    ClkResetMfs10  = 17u,   ///< MFS10 Reset
    ClkResetMfs11  = 18u,   ///< MFS11 Reset
    ClkResetMfs12  = 19u,   ///< MFS12 Reset
    ClkResetMfs13  = 20u,   ///< MFS13 Reset
    ClkResetMfs14  = 21u,   ///< MFS14 Reset
    ClkResetMfs15  = 22u,   ///< MFS15 Reset
    ClkResetQprc0  = 23u,   ///< QPRC0 Reset
    ClkResetQprc1  = 24u,   ///< QPRC1 Reset
    ClkResetQprc2  = 25u,   ///< QPRC2 Reset
    ClkResetQprc3  = 26u,   ///< QPRC3 Reset
    ClkResetMft0   = 27u,   ///< MFT0, PPG0/2/4/6 Reset
    ClkResetMft1   = 28u,   ///< MFT1, PPG8/10/12/14 Reset
    ClkResetMft2   = 29u,   ///< MFT2, PPG16/18/20/22 Reset
    ClkResetMft3   = 30u,   ///< MFT3, PPG24/26/28/30 Reset
    ClkResetBt0    = 31u,   ///< BT0/1/2/3 Reset
    ClkResetBt4    = 32u,   ///< BT4/5/6/7 Reset
    ClkResetBt8    = 33u,   ///< BT8/9/19/11 Reset
    ClkResetBt12   = 34u,   ///< BT12/13/14/15 Reset
    ClkResetQspi   = 35u,   ///< Quad SPI Reset
    ClkResetCec0   = 36u,   ///< CEC0 Reset
    ClkResetCec1   = 37u,   ///< CEC1 Reset
    ClkResetCrc    = 38u,   ///< CRC Reset
    ClkResetI2s0   = 39u,   ///< I2S0 Reset
    ClkResetI2s1   = 40u,   ///< I2S1 Reset
    ClkResetSd     = 41u,   ///< SD Card I/F Reset
    ClkResetCan0   = 42u,   ///< CAN0 Reset
    ClkResetCan1   = 43u,   ///< CAN1 Reset
    ClkResetCan2   = 44u,   ///< CAN2 Reset
    ClkResetUsb0   = 45u,   ///< USB0 Reset
    ClkResetUsb1   = 46u,   ///< USB1 Reset   
    ClkResetIcc0   = 47u,   ///< IC-Card interface 0
    ClkResetIcc1   = 48u,   ///< IC-Card interface 1
    ClkResetI2sl0  = 49u,   ///< I2S-Lite 0
    ClkResetI2sl1  = 50u,   ///< I2S-Lite 1
    
} en_clk_reset_peripheral_t;

/**
 ******************************************************************************
 ** \brief Clock configuration
 ** 
 ** The Clock configuration settings
 ******************************************************************************/
typedef struct stc_clk_config
{   
    en_clk_baseclkdiv_t   enBaseClkDiv;     ///< See description of #en_clk_baseclkdiv_t
    en_clk_apb0div_t      enAPB0Div;        ///< See description of #en_clk_apb0div_t
    en_clk_apb1div_t      enAPB1Div;        ///< See description of #en_clk_apb1div_t
#if (PDL_MCU_CORE == PDL_FM3_CORE) || (PDL_MCU_CORE == PDL_FM4_CORE)  
    en_clk_apb2div_t      enAPB2Div;        ///< See description of #en_clk_apb2div_t 
#endif    
    boolean_t             bAPB1Disable;     ///< TRUE: Disables APB1 regardless of divider settings
#if (PDL_MCU_CORE == PDL_FM3_CORE) || (PDL_MCU_CORE == PDL_FM4_CORE)      
    boolean_t             bAPB2Disable;     ///< TRUE: Disables APB2 regardless of divider settings
#endif       
    en_clk_mcowaittime_t  enMCOWaitTime;    ///< See description of #en_clk_mcowaittime_t
    en_clk_scowaittime_t  enSCOWaitTime;    ///< See description of #en_clk_scowaittime_t
    en_clk_pllowaittime_t enPLLOWaitTime;   ///< See description of #en_clk_pllowaittime_t
    uint8_t               u8PllK;           ///< PLL input clock frequency division ratio, PLLK
    uint8_t               u8PllM;           ///< PLL VCO clock frequency division ratio, PLLM
    uint8_t               u8PllN;           ///< PLL feedback frequency division ration, PLLN
#if (PDL_INTERRUPT_ENABLE_CLK == PDL_ON)
    boolean_t             bPllIrq;          ///< TRUE: enable PLL oscillation stabilization completion interrupt
    boolean_t             bMcoIrq;          ///< TRUE: enable Main Clock oscillation stabilization completion interrupt
    boolean_t             bScoIrq;          ///< TRUE: enable Sub Clock oscillation stabilization completion interrupt
    func_ptr_t            pfnPllStabCb;     ///< PLL stabilization callback function pointer
    func_ptr_t            pfnMcoStabCb;     ///< Main Clock stabilization callback function pointer
    func_ptr_t            pfnScoStabCb;     ///< Sub Clock stabilization callback function pointer
#endif   
} stc_clk_config_t;

/******************************************************************************/
/* Global variable definitions ('extern')                                     */
/******************************************************************************/
#if (PDL_INTERRUPT_ENABLE_CLK == PDL_ON)
extern stc_clk_intern_data_t stcClkInternData;
#endif

/******************************************************************************/
/* Global function prototypes (definition in C source)                        */
/******************************************************************************/

#if (PDL_INTERRUPT_ENABLE_CLK == PDL_ON)
void Clk_IrqHandler(void) ;
#endif
/* Set clock dividor, wait time, interrupt */
en_result_t Clk_Init(stc_clk_config_t* pstcClk) ;
/* Clock enable/disable */
#if (PDL_MCU_CORE == PDL_FM0P_CORE) 
en_result_t Clk_EnableHscr(boolean_t bBlock);
en_result_t Clk_DisableHscr(void);
#endif
en_result_t Clk_EnableMainClock(boolean_t bBlock);
en_result_t Clk_DisableMainClock(void);
en_result_t Clk_EnableSubClock(boolean_t bBlock);
en_result_t Clk_DisableSubClock(void);
en_result_t Clk_EnablePllClock(boolean_t bBlock);
en_result_t Clk_DisablePllClock(void);
/* Set the source clock */
en_result_t Clk_SetSource(en_clk_source_t enSource);
#if (PDL_MCU_CORE == PDL_FM0P_CORE) || (PDL_MCU_CORE == PDL_FM4_CORE)  
/* Peripheral clock enable/disable/status read */
en_result_t Clk_PeripheralClockEnable(en_clk_gate_peripheral_t enPeripheral) ;
en_result_t Clk_PeripheralClockDisable(en_clk_gate_peripheral_t enPeripheral) ;
boolean_t Clk_PeripheralGetClockState(en_clk_gate_peripheral_t enPeripheral) ;
en_result_t Clk_PeripheralClockEnableAll(void) ;
en_result_t Clk_PeripheralClockDisableAll(void) ;
/* Peripheral set reset/release reset */
en_result_t Clk_PeripheralSetReset(en_clk_reset_peripheral_t enPeripheral) ;
en_result_t Clk_PeripheralClearReset(en_clk_reset_peripheral_t enPeripheral) ;
#endif

//@} // ClkGroup

#ifdef __cplusplus
}
#endif

#endif // #if (defined(PDL_PERIPHERAL_CLK_ACTIVE))

#endif /* __CLK_H__ */
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
