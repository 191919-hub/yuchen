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
/** \file system_mb9abxxx.h
 **
 ** Headerfile for FM3 system parameters
 **
 ** History:
 **   - 2014-09-18  0.0.1  EZh   First version for universal PDL
 ******************************************************************************/

#ifndef _SYSTEM_FM3_H_
#define _SYSTEM_FM3_H_

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl.h"

#if (PDL_MCU_CORE == PDL_FM3_CORE)

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
 ** - 1: CLOCK_SETTING_CMSIS - Clock setting done in system_fmx.c like in
 **                            FM template projects
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
/*   Device dependent System Clock absolute maximum ranges:                   */
/*   Type 0:     80MHz                                                        */
/*   Type 1 & 5: 40MHz                                                        */
/*   Type 2 & 4: 144MHz                                                       */
/*   Type 3 & 7: 20MHz                                                        */
/*   Type 6 & 8 & 11: 40MHz                                                   */
/*   Type 9 & 10:     72MHz                                                   */
/*   Type 12:         60MHz                                                   */   
/*                                                                            */
/******************************************************************************/
#if (PDL_MCU_TYPE == PDL_FM3_TYPE0)
  #define CLOCK_SETUP           CLOCK_SETTING_CMSIS  // <<< Define clock setup macro here
  #define __CLKMO               (4000000UL)          // <<< External   4MHz Crystal
  #define __CLKSO               (32768UL)            // <<< External  32KHz Crystal
  #define __CLKHC               (4000000UL)          // Internal   4MHz CR Oscillator
  #define __CLKLC               (100000UL)           // Internal 100KHz CR Oscillator
  #define SCM_CTL_Val           0x00000052           // <<< Define SCM_CTL here
  #define BSC_PSR_Val           0x00000000           // <<< Define BSC_PSR here
  #define APBC0_PSR_Val         0x00000001           // <<< Define APBC0_PSR here
  #define APBC1_PSR_Val         0x00000081           // <<< Define APBC1_PSR here
  #define APBC2_PSR_Val         0x00000081           // <<< Define APBC2_PSR here
  #define SWC_PSR_Val           0x00000003           // <<< Define SWC_PSR here
  #define TTC_PSR_Val           0x00000000           // <<< Define TTC_PSR here
  #define CSW_TMR_Val           0x0000005C           // <<< Define CSW_TMR here
  #define PSW_TMR_Val           0x00000000           // <<< Define PSW_TMR here
  #define PLL_CTL1_Val          0x00000000           // <<< Define PLL_CTL1 here. Now K=1, M=1
  #define PLL_CTL2_Val          0x00000013           // <<< Define PLL_CTL2 here. Now N=19

  #define __PLLK         (((PLL_CTL1_Val >> 4) & 0x0F) + 1)
  #define __PLLN         (((PLL_CTL2_Val     ) & 0x1F) + 1)
  #define __PLLM         (((PLL_CTL1_Val     ) & 0x0Ful) + 1ul)

  #define HWWD_DISABLE          1   // <<< Define HW Watach dog enable here
  #define CR_TRIM_SETUP         1   // <<< Define CR trimming at startup enable here

#elif (PDL_MCU_TYPE == PDL_FM3_TYPE1) || (PDL_MCU_TYPE == PDL_FM3_TYPE5)
  #define CLOCK_SETUP           CLOCK_SETTING_CMSIS  // <<< Define clock setup macro here
  #define __CLKMO               (4000000UL)          // <<< External   4MHz Crystal
  #define __CLKSO               (32768UL)            // <<< External  32KHz Crystal
  #define __CLKHC               (4000000UL)          // Internal   4MHz CR Oscillator
  #define __CLKLC               (100000UL)           // Internal 100KHz CR Oscillator
  #define SCM_CTL_Val           0x00000052           // <<< Define SCM_CTL here
  #define BSC_PSR_Val           0x00000000           // <<< Define BSC_PSR here
  #define APBC0_PSR_Val         0x00000001           // <<< Define APBC0_PSR here
  #define APBC1_PSR_Val         0x00000081           // <<< Define APBC1_PSR here
  #define APBC2_PSR_Val         0x00000081           // <<< Define APBC2_PSR here
  #define SWC_PSR_Val           0x00000003           // <<< Define SWC_PSR here
  #define TTC_PSR_Val           0x00000000           // <<< Define TTC_PSR here
  #define CSW_TMR_Val           0x0000005C           // <<< Define CSW_TMR here
  #define PSW_TMR_Val           0x00000000           // <<< Define PSW_TMR here
  #define PLL_CTL1_Val          0x00000004           // <<< Define PLL_CTL1 here. Now K=1, M=5
  #define PLL_CTL2_Val          0x00000009           // <<< Define PLL_CTL2 here. Now N=10

  #define __PLLK         (((PLL_CTL1_Val >> 4) & 0x0F) + 1)
  #define __PLLN         (((PLL_CTL2_Val     ) & 0x3F) + 1)
  #define __PLLM         (((PLL_CTL1_Val     ) & 0x0Ful) + 1ul)

  #define HWWD_DISABLE          1   // <<< Define HW Watach dog enable here
  #define CR_TRIM_SETUP         1   // <<< Define CR trimming at startup enable here

#elif (PDL_MCU_TYPE == PDL_FM3_TYPE2) || (PDL_MCU_TYPE == PDL_FM3_TYPE4)
  #define CLOCK_SETUP           CLOCK_SETTING_CMSIS  // <<< Define clock setup macro here
  #define __CLKMO               (4000000UL)          // <<< External   4MHz Crystal
  #define __CLKSO               (32768UL)            // <<< External  32KHz Crystal
  #define __CLKHC               (4000000UL)          // Internal   4MHz CR Oscillator
  #define __CLKLC               (100000UL)           // Internal 100KHz CR Oscillator
  #define SCM_CTL_Val           0x00000052           // <<< Define SCM_CTL here
  #define BSC_PSR_Val           0x00000000           // <<< Define BSC_PSR here
  #define APBC0_PSR_Val         0x00000001           // <<< Define APBC0_PSR here
  #define APBC1_PSR_Val         0x00000082           // <<< Define APBC1_PSR here
  #define APBC2_PSR_Val         0x00000081           // <<< Define APBC2_PSR here
  #define SWC_PSR_Val           0x00000003           // <<< Define SWC_PSR here
  #define TTC_PSR_Val           0x00000000           // <<< Define TTC_PSR here
  #define CSW_TMR_Val           0x0000005C           // <<< Define CSW_TMR here
  #define PSW_TMR_Val           0x00000000           // <<< Define PSW_TMR here
  #define PLL_CTL1_Val          0x00000001           // <<< Define PLL_CTL1 here. Now K=1, M=2
  #define PLL_CTL2_Val          0x00000023           // <<< Define PLL_CTL2 here. Now N=36

  #define __PLLK         (((PLL_CTL1_Val >> 4) & 0x0F) + 1)
  #define __PLLN         (((PLL_CTL2_Val     ) & 0x3F) + 1)
  #define __PLLM         (((PLL_CTL1_Val     ) & 0x0Ful) + 1ul)

  #define HWWD_DISABLE          1   // <<< Define HW Watach dog enable here
  #define CR_TRIM_SETUP         1   // <<< Define CR trimming at startup enable here

#elif (PDL_MCU_TYPE == PDL_FM3_TYPE3) || (PDL_MCU_TYPE == PDL_FM3_TYPE7)
  #define CLOCK_SETUP           CLOCK_SETTING_CMSIS  // <<< Define clock setup macro here
  #define __CLKMO               (4000000UL)          // <<< External   4MHz Crystal
  #define __CLKSO               (32768UL)            // <<< External  32KHz Crystal
  #define __CLKHC               (4000000UL)          // Internal   4MHz CR Oscillator
  #define __CLKLC               (100000UL)           // Internal 100KHz CR Oscillator
  #define SCM_CTL_Val           0x00000052           // <<< Define SCM_CTL here
  #define BSC_PSR_Val           0x00000000           // <<< Define BSC_PSR here
  #define APBC0_PSR_Val         0x00000001           // <<< Define APBC0_PSR here
  #define APBC1_PSR_Val         0x00000081           // <<< Define APBC1_PSR here
  #define APBC2_PSR_Val         0x00000081           // <<< Define APBC2_PSR here
  #define SWC_PSR_Val           0x00000003           // <<< Define SWC_PSR here
  #define TTC_PSR_Val           0x00000000           // <<< Define TTC_PSR here
  #define CSW_TMR_Val           0x0000005C           // <<< Define CSW_TMR here
  #define PSW_TMR_Val           0x00000000           // <<< Define PSW_TMR here
  #define PLL_CTL1_Val          0x00000000           // <<< Define PLL_CTL1 here. Now K=1, M=2
  #define PLL_CTL2_Val          0x00000004           // <<< Define PLL_CTL2 here. Now N=36

  #define __PLLK         (((PLL_CTL1_Val >> 4) & 0x0F) + 1)
  #define __PLLN         (((PLL_CTL2_Val     ) & 0x3F) + 1)
  #define __PLLM         (((PLL_CTL1_Val     ) & 0x0Ful) + 1ul)

  #define HWWD_DISABLE          1   // <<< Define HW Watach dog enable here
  #define CR_TRIM_SETUP         1   // <<< Define CR trimming at startup enable here

#elif (PDL_MCU_TYPE == PDL_FM3_TYPE6) || (PDL_MCU_TYPE == PDL_FM3_TYPE8) || (PDL_MCU_TYPE == PDL_FM3_TYPE11)
  #define CLOCK_SETUP           CLOCK_SETTING_CMSIS  // <<< Define clock setup macro here
  #define __CLKMO               (4000000UL)          // <<< External   4MHz Crystal
  #define __CLKSO               (32768UL)            // <<< External  32KHz Crystal
  #define __CLKHC               (4000000UL)          // Internal   4MHz CR Oscillator
  #define __CLKLC               (100000UL)           // Internal 100KHz CR Oscillator
  #define SCM_CTL_Val           0x00000052           // <<< Define SCM_CTL here
  #define BSC_PSR_Val           0x00000000           // <<< Define BSC_PSR here
  #define APBC0_PSR_Val         0x00000001           // <<< Define APBC0_PSR here
  #define APBC1_PSR_Val         0x00000081           // <<< Define APBC1_PSR here
  #define APBC2_PSR_Val         0x00000081           // <<< Define APBC2_PSR here
  #define SWC_PSR_Val           0x00000003           // <<< Define SWC_PSR here
  #define TTC_PSR_Val           0x00000000           // <<< Define TTC_PSR here
  #define CSW_TMR_Val           0x0000005C           // <<< Define CSW_TMR here
  #define PSW_TMR_Val           0x00000000           // <<< Define PSW_TMR here
  #define PLL_CTL1_Val          0x00000001           // <<< Define PLL_CTL1 here. Now K=1, M=2
  #define PLL_CTL2_Val          0x00000009           // <<< Define PLL_CTL2 here. Now N=10

  #define __PLLK         (((PLL_CTL1_Val >> 4) & 0x0F) + 1)
  #define __PLLN         (((PLL_CTL2_Val     ) & 0x3F) + 1)
  #define __PLLM         (((PLL_CTL1_Val     ) & 0x0Ful) + 1ul)

  #define CLOCK_SETUP           CLOCK_SETTING_CMSIS  // <<< Define clock setup macro here
  #define HWWD_DISABLE          1   // <<< Define HW Watach dog enable here
  #define CR_TRIM_SETUP         1   // <<< Define CR trimming at startup enable here

#elif (PDL_MCU_TYPE == PDL_FM3_TYPE9) || (PDL_MCU_TYPE == PDL_FM3_TYPE10)
  //#define CLOCK_SETUP           CLOCK_SETTING_CMSIS  // <<< Define clock setup macro here
  #define __CLKMO               (4000000UL)          // <<< External   4MHz Crystal
  #define __CLKSO               (32768UL)            // <<< External  32KHz Crystal
  #define __CLKHC               (4000000UL)          // Internal   4MHz CR Oscillator
  #define __CLKLC               (100000UL)           // Internal 100KHz CR Oscillator
  #define SCM_CTL_Val           0x00000000          // <<< Define SCM_CTL here
  #define BSC_PSR_Val           0x00000000          // <<< Define BSC_PSR here
  #define APBC0_PSR_Val         0x00000001          // <<< Define APBC0_PSR here
  #define APBC1_PSR_Val         0x00000081          // <<< Define APBC1_PSR here
  #define APBC2_PSR_Val         0x00000080          // <<< Define APBC2_PSR here
  #define SWC_PSR_Val           0x00000003          // <<< Define SWC_PSR here
  #define TTC_PSR_Val           0x00000000          // <<< Define TTC_PSR here
  #define CSW_TMR_Val           0x0000005C          // <<< Define CSW_TMR here
  #define PSW_TMR_Val           0x00000000          // <<< Define PSW_TMR here
  #define PLL_CTL1_Val          0x00000001          // <<< Define PLL_CTL1 here. Now K=1, M=2
  #define PLL_CTL2_Val          0x000000011         // <<< Define PLL_CTL1 here. Now N=18

  #define __PLLK         (((PLL_CTL1_Val >> 4) & 0x0F) + 1)
  #define __PLLN         (((PLL_CTL2_Val     ) & 0x3F) + 1)
  #define __PLLM         (((PLL_CTL1_Val     ) & 0x0Ful) + 1ul)

  #define HWWD_DISABLE          1   // <<< Define HW Watach dog enable here
  #define CR_TRIM_SETUP         1   // <<< Define CR trimming at startup enable here

#elif (PDL_MCU_TYPE == PDL_FM3_TYPE12) 
  #define CLOCK_SETUP           CLOCK_SETTING_CMSIS  // <<< Define clock setup macro here
  #define __CLKMO               (4000000UL)          // <<< External   4MHz Crystal
  #define __CLKSO               (32768UL)            // <<< External  32KHz Crystal
  #define __CLKHC               (4000000UL)          // Internal   4MHz CR Oscillator
  #define __CLKLC               (100000UL)           // Internal 100KHz CR Oscillator
  #define SCM_CTL_Val           0x00000052           // <<< Define SCM_CTL here
  #define BSC_PSR_Val           0x00000000           // <<< Define BSC_PSR here
  #define APBC0_PSR_Val         0x00000001           // <<< Define APBC0_PSR here
  #define APBC1_PSR_Val         0x00000081           // <<< Define APBC1_PSR here
  #define APBC2_PSR_Val         0x00000081           // <<< Define APBC2_PSR here
  #define SWC_PSR_Val           0x00000003           // <<< Define SWC_PSR here
  #define TTC_PSR_Val           0x00000000           // <<< Define TTC_PSR here
  #define CSW_TMR_Val           0x0000005C           // <<< Define CSW_TMR here
  #define PSW_TMR_Val           0x00000000           // <<< Define PSW_TMR here
  #define PLL_CTL1_Val          0x00000001           // <<< Define PLL_CTL1 here. Now K=1, M=2
  #define PLL_CTL2_Val          0x0000000E           // <<< Define PLL_CTL1 here. Now N=15

  #define __PLLK         (((PLL_CTL1_Val >> 4) & 0x0F) + 1)
  #define __PLLN         (((PLL_CTL2_Val     ) & 0x3F) + 1)
  #define __PLLM         (((PLL_CTL1_Val     ) & 0x0Ful) + 1ul)

  #define HWWD_DISABLE          1   // <<< Define HW Watach dog enable here
  #define CR_TRIM_SETUP         1   // <<< Define CR trimming at startup enable here
#else
#error "Device type not found!"   
#endif

/**
 ******************************************************************************
 ** \brief Calculate PLL output frequency from settings
 ******************************************************************************/
#define __PLLCLK       ((__CLKMO  * __PLLN) / __PLLK) // PLL clock calculation

/**
 ******************************************************************************
 ** \brief Define Master Clock from settings
 ******************************************************************************/
#if   (((SCM_CTL_Val >> 5) & 0x07) == 0)
  #define __MASTERCLK     (__CLKHC)
#elif (((SCM_CTL_Val >> 5) & 0x07) == 1)
  #define __MASTERCLK     (__CLKMO)
#elif (((SCM_CTL_Val >> 5) & 0x07) == 2)
  #define __MASTERCLK     (__PLLCLK)
#elif (((SCM_CTL_Val >> 5) & 0x07) == 4)
  #define __MASTERCLK     (__CLKLC)
#elif (((SCM_CTL_Val >> 5) & 0x07) == 5)
  #define __MASTERCLK     (__CLKSO)
#else
  #define __MASTERCLK     (0UL)
#endif

/**
 ******************************************************************************
 ** \brief Define System Clock Frequency (Core Clock) from settings
 ******************************************************************************/
#if   ((BSC_PSR_Val & 0x07) == 0)
  #define __HCLK         (__MASTERCLK / 1)
#elif ((BSC_PSR_Val & 0x07) == 1)
  #define __HCLK         (__MASTERCLK / 2)
#elif ((BSC_PSR_Val & 0x07) == 2)
  #define __HCLK         (__MASTERCLK / 3)
#elif ((BSC_PSR_Val & 0x07) == 3)
  #define __HCLK         (__MASTERCLK / 4)
#elif ((BSC_PSR_Val & 0x07) == 4)
  #define __HCLK         (__MASTERCLK / 6)
#elif ((BSC_PSR_Val & 0x07) == 5)
  #define __HCLK         (__MASTERCLK / 8)
#elif ((BSC_PSR_Val & 0x07) == 6)
  #define __HCLK         (__MASTERCLK /16)
#else
  #define __HCLK         (0UL)
#endif

/******************************************************************************/
/* Global function prototypes ('extern', definition in C source)              */
/******************************************************************************/

extern uint32_t SystemCoreClock;          // System Clock Frequency (Core Clock)
extern uint32_t SystemExternalClock;

extern void SystemInit (void);            // Initialize the system

extern void SystemCoreClockUpdate (void); // Update SystemCoreClock variable

#ifdef __cplusplus
}
#endif

#endif

#endif /* _SYSTEM_FM3_H_ */
