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
#include "mcu.h"

/******************************************************************************/
/** \file system_fm3.c
 **
 ** FM3 system initialization functions
 ** All adjustments can be done in belonging header file.
 **
 ** History:
 **   - 2014-09-18  0.0.1  EZh   First version for universal PDL    
 **
 ******************************************************************************/
#if (PDL_MCU_CORE == PDL_FM3_CORE)  
 
/**
 ******************************************************************************
 ** System Clock Frequency (Core Clock) Variable according CMSIS
 ******************************************************************************/
uint32_t SystemCoreClock = __HCLK;
uint32_t SystemExternalClock = __CLKMO;
/**
 ******************************************************************************
 ** \brief  Update the System Core Clock with current core Clock retrieved from
 ** cpu registers.
 ** \param  none
 ** \return none
 ******************************************************************************/
void SystemCoreClockUpdate (void) {
  uint32_t masterClk;
  uint32_t u32RegisterRead; // Workaround variable for MISRA C rule conformance

  switch ((FM_CRG->SCM_CTL >> 5) & 0x07) {
    case 0:                                 /* internal High-speed Cr osc.    */
      masterClk = __CLKHC;
      break;

    case 1:                                 /* external main osc.             */
      masterClk = __CLKMO;
      break;

    case 2:                                 /* PLL clock                      */
  // Workaround for preventing MISRA C:1998 Rule 46 (MISRA C:2004 Rule 12.2)
  // violation:
  //   "Unordered accesses to a volatile location"
      u32RegisterRead = (__CLKMO  * (((FM_CRG->PLL_CTL2) & 0x1F) + 1));
      masterClk = (u32RegisterRead / (((FM_CRG->PLL_CTL1 >> 4) & 0x0F) + 1));
      break;

    case 4:                                 /* internal Low-speed CR osc.     */
      masterClk = __CLKLC;
      break;

    case 5:                                 /* external Sub osc.              */
      masterClk = __CLKSO;
      break;

    default:
      masterClk = 0Ul;
      break;
  }

  switch (FM_CRG->BSC_PSR & 0x07) {
    case 0:
      SystemCoreClock = masterClk;
      break;

    case 1:
      SystemCoreClock = masterClk / 2;
      break;

    case 2:
      SystemCoreClock = masterClk / 3;
      break;

    case 3:
      SystemCoreClock = masterClk / 4;
      break;

    case 4:
      SystemCoreClock = masterClk / 6;
      break;

    case 5:
      SystemCoreClock = masterClk /8;
      break;

    case 6:
      SystemCoreClock = masterClk /16;
      break;

    default:
      SystemCoreClock = 0Ul;
      break;
  }

}

/**
 ******************************************************************************
 ** \brief  Setup the microcontroller system. Initialize the System and update
 ** the SystemCoreClock variable.
 **
 ** \param  none
 ** \return none
 ******************************************************************************/
void SystemInit (void) {
#if (CLOCK_SETUP == CLOCK_SETTING_CMSIS)                     /* Clock Setup */
  static uint32_t u32IoRegisterRead;  // Workaround variable for MISRA C rule conformance
#endif  
  
#if (HWWD_DISABLE)                                 /* HW Watchdog Disable */
  FM_HWWDT->WDG_LCK = 0x1ACCE551;                 /* HW Watchdog Unlock */
  FM_HWWDT->WDG_LCK = 0xE5331AAE;
  FM_HWWDT->WDG_CTL = 0;                          /* HW Watchdog stop */
#endif

#if (CLOCK_SETUP == CLOCK_SETTING_CMSIS)          /* Clock Setup */
  FM_CRG->BSC_PSR   = BSC_PSR_Val;                /* set System Clock presacaler    不分频*/  
  FM_CRG->APBC0_PSR = APBC0_PSR_Val;              /* set APB0 presacaler  二分频*/
  FM_CRG->APBC1_PSR = APBC1_PSR_Val;              /* set APB1 presacaler 使能PCLK1输出，二分频 */
  FM_CRG->APBC2_PSR = APBC2_PSR_Val;              /* set APB2 presacaler 使能PCLK2输出，二分频*/
  FM_CRG->SWC_PSR   = SWC_PSR_Val | (1UL << 7);   /* set SW Watchdog presacaler 1/8 PLCK0 */
  FM_CRG->TTC_PSR   = TTC_PSR_Val;                /* set Trace Clock presacaler */

  FM_CRG->CSW_TMR   = CSW_TMR_Val;                /* set oscillation stabilization wait time  sub: 327ms  main: 33ms */
  
  if (SCM_CTL_Val & (1UL << 1)) {                    /* Main clock oscillator enabled ? */
    FM_CRG->SCM_CTL |= (1UL << 1);                /* enable main oscillator */ 
    while (!(FM_CRG->SCM_STR & (1UL << 1)));      /* wait for Main clock oscillation stable */
  }
  
  if (SCM_CTL_Val & (1UL << 3)) {                    /* Sub clock oscillator enabled ? */
    FM_CRG->SCM_CTL |= (1UL << 3);                /* enable sub oscillator */ 
    while (!(FM_CRG->SCM_STR & (1UL << 3)));      /* wait for Sub clock oscillation stable */
  }

  FM_CRG->PSW_TMR   = PSW_TMR_Val;                /* set PLL stabilization wait time */
  FM_CRG->PLL_CTL1  = PLL_CTL1_Val;               /* set PLLM and PLLK */
  FM_CRG->PLL_CTL2  = PLL_CTL2_Val;               /* set PLLN          */
  
  //if (SCM_CTL_Val & (1UL << 4)) {                    /* PLL enabled ? */
    //if ((SCM_CTL_Val & (1ul << 1u)) == 0u)  // if main clock disable, use high-speed CR for PLL
   // {
      //  FM_CRG->PSW_TMR_f.PINC = 1u;
    //}
    
    FM_CRG->SCM_CTL  |= (1UL << 4);               /* enable PLL */ 
    while (!(FM_CRG->SCM_STR & (1UL << 4)));      /* wait for PLL stable */
  //}

  FM_CRG->SCM_CTL  |= (SCM_CTL_Val & 0xE0);       /* Set Master Clock switch */ 
  
  // Workaround for preventing MISRA C:1998 Rule 46 (MISRA C:2004 Rule 12.2)
  // violations:
  //   "Unordered reads and writes to or from same location" and
  //   "Unordered accesses to a volatile location"
  do                                              
  {                                               
    u32IoRegisterRead = (FM_CRG->SCM_CTL & 0xE0); 
  }while ((FM_CRG->SCM_STR & 0xE0) != u32IoRegisterRead);
  
#elif (CLOCK_SETUP == CLOCK_SETTING_NONE)
  
  // user defined clock setting
  
#else
  #error Clock setup type unknown!
  
#endif // (CLOCK_SETUP == CLOCK_SETTING_CMSIS)
  
#if (CR_TRIM_SETUP)
  /* CR Trimming Data  */
  if( 0x000003FF != (FM_FLASH_IF->CRTRMM & 0x000003FF) )
  {
    /* UnLock (MCR_FTRM) */
    FM_CRTRIM->MCR_RLR = 0x1ACCE554;
    /* Set MCR_FTRM */
    FM_CRTRIM->MCR_FTRM = FM_FLASH_IF->CRTRMM;
    /* Lock (MCR_FTRM) */
    FM_CRTRIM->MCR_RLR = 0x00000000;
  }
#endif // (CR_TRIM_SETUP)
}

#endif 


