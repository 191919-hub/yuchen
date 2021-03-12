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
/** \file system_fm4.h
 **
 ** Headerfile for FM4 system parameters
 **
 ** History: 
 **   - 2014-09-18  0.0.1  EZh   First version for universal PDL  
 **
 ******************************************************************************/

#ifndef _SYSTEM_FM4_H_
#define _SYSTEM_FM4_H_

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl.h"  

#if (PDL_MCU_CORE == PDL_FM4_CORE)  

#ifdef __cplusplus
extern "C" {
#endif 

/******************************************************************************/
/* Global pre-processor symbols/macros ('define')                             */
/******************************************************************************/

/**
 ******************************************************************************
 ** \brief Clock Setup macro definition
 **
 ** - 0: CLOCK_SETTING_NONE  - User provides own clock setting in application
 ** - 1: CLOCK_SETTING_CMSIS - Clock setting done in system_mbabxxx.h like in
 **                            FM4 template projects
 ******************************************************************************/ 
#define CLOCK_SETTING_NONE  0u
#define CLOCK_SETTING_CMSIS 1u  
  
/******************************************************************************/
/*                                                                            */
/*                      START OF USER SETTINGS HERE                           */
/*                      ===========================                           */   
/*                                                                            */
/*                 All lines with '<<<' can be set by user.                   */
/*                                                                            */
/******************************************************************************/
#if (PDL_MCU_TYPE == PDL_FM4_TYPE1) || \
    (PDL_MCU_TYPE == PDL_FM4_TYPE2) || \
    (PDL_MCU_TYPE == PDL_FM4_TYPE6)  
#define CLOCK_SETUP    CLOCK_SETTING_CMSIS   // <<< Define clock setup macro here
#define __CLKMO        ( 4000000ul)   // <<< External   4MHz Crystal 
#define __CLKSO        (   32768ul)   // <<<  External  32KHz Crystal
#define __CLKHC        ( 4000000ul)         // Internal   4MHz CR Oscillator 
#define __CLKLC        (  100000ul)         // Internal 100KHz CR Oscillator    
#define SCM_CTL_Val           0x00000052ul    // <<< Define SCM_CTL here   
#define BSC_PSR_Val           0x00000000ul    // <<< Define BSC_PSR here
#define APBC0_PSR_Val         0x00000001UL     // <<< Define APBC0_PSR here
#define APBC1_PSR_Val         0x00000081ul    // <<< Define APBC1_PSR here 
#define APBC2_PSR_Val         0x00000081ul    // <<< Define APBC2_PSR here
#define SWC_PSR_Val           0x00000003ul    // <<< Define SWC_PSR here
#define LCR_PRSLD_Val           0x00000000ul    // <<< Define LCR_PRSLD
#define TTC_PSR_Val           0x00000000ul    // <<< Define TTC_PSR here  
#define CSW_TMR_Val           0x0000005Cul    // <<< Define CSW_TMR here  
#define PSW_TMR_Val           0x00000000ul    // <<< Define PSW_TMR here 
#define PLL_CTL1_Val          0x00000001ul    // <<< Define PLL_CTL1 here
#define PLL_CTL2_Val          0x00000027ul    // <<< Define PLL_CTL2 here
#define TRACE_BUFFER_ENABLE   1   // <<< Define Trace Buffer enable here  
#define HWWD_DISABLE          1   // <<< Define HW Watach dog enable here
#define CR_TRIM_SETUP         1   // <<< Define CR trimming at startup enable here

#define __PLLK         (((PLL_CTL1_Val >> 4ul) & 0x0Ful) + 1ul)
#define __PLLN         (((PLL_CTL2_Val     ) & 0x3Ful) + 1ul)  
#define __PLLM         (((PLL_CTL1_Val     ) & 0x0Ful) + 1ul) 

#define VB_CLK_DIV_Val     0x4Eu        // <<< Define VB_CLK_DIV here
#define WTOSCCNT_Val       0x02u        // <<< Define WTOSCCNT here
#define CCS_Val            0x08u        // <<< Define CCS here
#define CCB_Val            0x10u        // <<< Define CCS here 
#define BOOST_Val          0x03u        // <<< Define BOOST here
#define VBPFR_Val          0x1Cu        // <<< Define VBPFR here         

#elif (PDL_MCU_TYPE == PDL_FM4_TYPE3)
#define CLOCK_SETUP    CLOCK_SETTING_CMSIS    // <<< Define clock setup macro here
#define __CLKMO        ( 4000000ul)           // <<< External   4MHz Crystal 
#define __CLKSO        (   32768ul)           // <<<  External  32KHz Crystal
#define __CLKHC        ( 4000000ul)           // Internal   4MHz CR Oscillator 
#define __CLKLC        (  100000ul)           // Internal 100KHz CR Oscillator    
#define SCM_CTL_Val           0x00000052ul    // <<< Define SCM_CTL here   
#define BSC_PSR_Val           0x00000000ul    // <<< Define BSC_PSR here
#define APBC0_PSR_Val         0x00000001UL    // <<< Define APBC0_PSR here
#define APBC1_PSR_Val         0x00000081ul    // <<< Define APBC1_PSR here 
#define APBC2_PSR_Val         0x00000081ul    // <<< Define APBC2_PSR here
#define SWC_PSR_Val           0x00000003ul    // <<< Define SWC_PSR here
#define LCR_PRSLD_Val         0x00000000ul    // <<< Define LCR_PRSLD
#define TTC_PSR_Val           0x00000000ul    // <<< Define TTC_PSR here  
#define CSW_TMR_Val           0x0000005Cul    // <<< Define CSW_TMR here  
#define PSW_TMR_Val           0x00000000ul    // <<< Define PSW_TMR here 
#define PLL_CTL1_Val          0x00000001ul    // <<< Define PLL_CTL1 here
#define PLL_CTL2_Val          0x00000031ul    // <<< Define PLL_CTL2 here
#define TRACE_BUFFER_ENABLE   1   // <<< Define Trace Buffer enable here  
#define HWWD_DISABLE          1   // <<< Define HW Watach dog enable here
#define CR_TRIM_SETUP         1   // <<< Define CR trimming at startup enable here

#define __PLLK         (((PLL_CTL1_Val >> 4ul) & 0x0Ful) + 1ul)  
#define __PLLN         (((PLL_CTL2_Val     ) & 0x3Ful) + 1ul)  
#define __PLLM         (((PLL_CTL1_Val     ) & 0x0Ful) + 1ul) 
      
#define VB_CLK_DIV_Val     0x4Eu        // <<< Define VB_CLK_DIV here
#define WTOSCCNT_Val       0x02u        // <<< Define WTOSCCNT here
#define CCS_Val            0x08u        // <<< Define CCS here
#define CCB_Val            0x10u        // <<< Define CCS here   
#define BOOST_Val          0x03u        // <<< Define BOOST here      
#define VBPFR_Val          0x1Cu        // <<< Define VBPFR here         

#elif (PDL_MCU_TYPE == PDL_FM4_TYPE4)
#define CLOCK_SETUP    CLOCK_SETTING_CMSIS   // <<< Define clock setup macro here
#define __CLKMO        ( 4000000ul)   // <<< External   4MHz Crystal 
#define __CLKSO        (   32768ul)   // <<<  External  32KHz Crystal
#define __CLKHC        ( 4000000ul)         // Internal   4MHz CR Oscillator 
#define __CLKLC        (  100000ul)         // Internal 100KHz CR Oscillator    
#define SCM_CTL_Val           0x00000052ul    // <<< Define SCM_CTL here   
#define BSC_PSR_Val           0x00000000ul    // <<< Define BSC_PSR here
#define APBC0_PSR_Val         0x00000001UL     // <<< Define APBC0_PSR here
#define APBC1_PSR_Val         0x00000081ul    // <<< Define APBC1_PSR here 
#define APBC2_PSR_Val         0x00000081ul    // <<< Define APBC2_PSR here
#define SWC_PSR_Val           0x00000003ul    // <<< Define SWC_PSR here
#define LCR_PRSLD_Val         0x00000000ul    // <<< Define LCR_PRSLD
#define TTC_PSR_Val           0x00000000ul    // <<< Define TTC_PSR here  
#define CSW_TMR_Val           0x0000005Cul    // <<< Define CSW_TMR here  
#define PSW_TMR_Val           0x00000000ul    // <<< Define PSW_TMR here 
#define PLL_CTL1_Val          0x00000001ul    // <<< Define PLL_CTL1 here
#define PLL_CTL2_Val          0x00000027ul    // <<< Define PLL_CTL2 here
#define TRACE_BUFFER_ENABLE   1   // <<< Define Trace Buffer enable here  
#define HWWD_DISABLE          1   // <<< Define HW Watach dog enable here
#define CR_TRIM_SETUP         1   // <<< Define CR trimming at startup enable here

#define __PLLK         (((PLL_CTL1_Val >> 4ul) & 0x0Ful) + 1ul)
#define __PLLN         (((PLL_CTL2_Val     ) & 0x3Ful) + 1ul)  
#define __PLLM         (((PLL_CTL1_Val     ) & 0x0Ful) + 1ul) 
      
#define VB_CLK_DIV_Val     0x4Eu        // <<< Define VB_CLK_DIV here
#define WTOSCCNT_Val       0x02u        // <<< Define WTOSCCNT here
#define CCS_Val            0xCEu        // <<< Define CCS here
#define CCB_Val            0xCEu        // <<< Define CCS here 
#define BOOST_Val          0x03u        // <<< Define BOOST here 
#define VBPFR_Val          0x1Cu        // <<< Define VBPFR here         
      
#elif (PDL_MCU_TYPE == PDL_FM4_TYPE5)
#define CLOCK_SETUP    CLOCK_SETTING_CMSIS   // <<< Define clock setup macro here
#define __CLKMO        ( 4000000ul)   // <<< External   4MHz Crystal 
#define __CLKSO        (   32768ul)   // <<<  External  32KHz Crystal
#define __CLKHC        ( 4000000ul)         // Internal   4MHz CR Oscillator 
#define __CLKLC        (  100000ul)         // Internal 100KHz CR Oscillator    
#define SCM_CTL_Val           0x00000052ul    // <<< Define SCM_CTL here   
#define BSC_PSR_Val           0x00000000ul    // <<< Define BSC_PSR here
#define APBC0_PSR_Val         0x00000001UL     // <<< Define APBC0_PSR here
#define APBC1_PSR_Val         0x00000080ul    // <<< Define APBC1_PSR here 
#define APBC2_PSR_Val         0x00000081ul    // <<< Define APBC2_PSR here
#define SWC_PSR_Val           0x00000003ul    // <<< Define SWC_PSR here
#define LCR_PRSLD_Val         0x00000000ul    // <<< Define LCR_PRSLD
#define TTC_PSR_Val           0x00000000ul    // <<< Define TTC_PSR here  
#define CSW_TMR_Val           0x0000005Cul    // <<< Define CSW_TMR here  
#define PSW_TMR_Val           0x00000000ul    // <<< Define PSW_TMR here 
#define PLL_CTL1_Val          0x00000001ul    // <<< Define PLL_CTL1 here
#define PLL_CTL2_Val          0x0000002Cul    // <<< Define PLL_CTL2 here
#define TRACE_BUFFER_ENABLE   1   // <<< Define Trace Buffer enable here  
#define HWWD_DISABLE          1   // <<< Define HW Watach dog enable here
#define CR_TRIM_SETUP         1   // <<< Define CR trimming at startup enable here      

#define __PLLK         (((PLL_CTL1_Val >> 4ul) & 0x0Ful) + 1ul)
#define __PLLN         (((PLL_CTL2_Val     ) & 0x3Ful) + 1ul)  
#define __PLLM         (((PLL_CTL1_Val     ) & 0x0Ful) + 1ul) 
      
#else
#error "Device type not found!"  
#endif
  
/**
 ******************************************************************************
 ** \brief Calculate PLL output frequency from settings
 ******************************************************************************/
#define __PLLCLK       ((__CLKMO  * __PLLN) / __PLLK)
  
/**
 ******************************************************************************
 ** \brief Define Master Clock from settings
 ******************************************************************************/
#if   (((SCM_CTL_Val >> 5ul) & 0x07UL) == 0ul)
  #define __MASTERCLK     (__CLKHC)
#elif (((SCM_CTL_Val >> 5ul) & 0x07UL) == 1ul)
  #define __MASTERCLK     (__CLKMO)
#elif (((SCM_CTL_Val >> 5ul) & 0x07UL) == 2ul)
  #define __MASTERCLK     (__PLLCLK)
#elif (((SCM_CTL_Val >> 5ul) & 0x07UL) == 4ul)
  #define __MASTERCLK     (__CLKLC)
#elif (((SCM_CTL_Val >> 5ul) & 0x07UL) == 5ul)
  #define __MASTERCLK     (__CLKSO)
#else
  #define __MASTERCLK     (0UL)
#endif

/**
 ******************************************************************************
 ** \brief Define System Clock Frequency (Core Clock) from settings
 ******************************************************************************/
#if   ((BSC_PSR_Val & 0x07UL) == 0ul)
  #define __HCLK         (__MASTERCLK / 1ul)
#elif ((BSC_PSR_Val & 0x07UL) == 1ul)
  #define __HCLK         (__MASTERCLK / 2ul)
#elif ((BSC_PSR_Val & 0x07UL) == 2ul)
  #define __HCLK         (__MASTERCLK / 3ul)
#elif ((BSC_PSR_Val & 0x07UL) == 3ul)
  #define __HCLK         (__MASTERCLK / 4ul)
#elif ((BSC_PSR_Val & 0x07UL) == 4ul)
  #define __HCLK         (__MASTERCLK / 6ul)
#elif ((BSC_PSR_Val & 0x07UL) == 5ul)
  #define __HCLK         (__MASTERCLK / 8ul)
#elif ((BSC_PSR_Val & 0x07UL) == 6ul)
  #define __HCLK         (__MASTERCLK /16ul)
#else
  #define __HCLK         (0ul)
#endif  
  
/******************************************************************************/
/* Global function prototypes ('extern', definition in C source)              */
/******************************************************************************/

extern uint32_t SystemCoreClock;          // System Clock Frequency (Core Clock)
   
extern void SystemInit (void);            // Initialize the system

extern void SystemCoreClockUpdate (void); // Update SystemCoreClock variable

#ifdef __cplusplus
}
#endif

#endif

#endif /* _SYSTEM_FM4_H_ */
