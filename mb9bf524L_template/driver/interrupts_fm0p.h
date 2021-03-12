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
/** \file interrupts.h
 **
 ** PDL Interrupt Handlers
 **
 ** History:
 **   - 2014-09-18  0.0.1  EZh   First version for universal PDL     
 **
 ******************************************************************************/
#ifndef __INTERRUPTS_FM0P_H__
#define __INTERRUPTS_FM0P_H__

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_user.h"

#if (PDL_MCU_CORE == PDL_FM0P_CORE)

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
  
/**
 ******************************************************************************
 ** \defgroup PdlInterrupts PDL Interrupt handling
 **
 ** Each device group of the FM0P MCUs may have individual interrupt service
 ** routine vector names. For this reason this file defines the device
 ** depenent vectors, if the corresponding resource is set to active in 
 ** pdl_user.h.
 **
 ** Interrupt and Callback flow in PDL:
 ** \image html Interrupt_flow.png
 **
 ******************************************************************************/
//@{

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/
#if (PDL_MCU_INT_TYPE == PDL_FM0P_INT_TYPE_A)
    #define PDL_IRQMON_FCS                     IRQ00MON
    #define PDL_IRQMON_SWT                     IRQ01MON
    #define PDL_IRQMON_LVD                     IRQ02MON
    #define PDL_IRQMON_MFT_WFG                 IRQ03MON
    #define PDL_IRQMON_EXINT0_7                IRQ04MON
    #define PDL_IRQMON_EXINT8_31               IRQ05MON
    #define PDL_IRQMON_DT_QPRC                 IRQ06MON
    #define PDL_IRQMON_MFS0_8_RX               IRQ07MON
    #define PDL_IRQMON_MFS0_8_TX               IRQ08MON
    #define PDL_IRQMON_MFS1_9_RX               IRQ09MON
    #define PDL_IRQMON_MFS1_9_TX               IRQ10MON
    #define PDL_IRQMON_MFS2_10_RX              IRQ11MON
    #define PDL_IRQMON_MFS2_10_TX              IRQ12MON
    #define PDL_IRQMON_MFS3_11_RX              IRQ13MON
    #define PDL_IRQMON_MFS3_11_TX              IRQ14MON  
    #define PDL_IRQMON_MFS4_12_RX              IRQ15MON
    #define PDL_IRQMON_MFS4_12_TX              IRQ16MON 
    #define PDL_IRQMON_MFS5_13_RX              IRQ17MON
    #define PDL_IRQMON_MFS5_13_TX              IRQ18MON 
    #define PDL_IRQMON_MFS6_14_RX_DMA0         IRQ19MON
    #define PDL_IRQMON_MFS6_14_TX_DMA1         IRQ20MON 
    #define PDL_IRQMON_MFS7_15_RX_DMA2         IRQ21MON
    #define PDL_IRQMON_MFS7_15_TX_DMA3         IRQ22MON
    #define PDL_IRQMON_PPG_RC0_ICC0_DSTC       IRQ23MON
    #define PDL_IRQMON_TIM                     IRQ24MON
    #define PDL_IRQMON_ADC0                    IRQ25MON
    #define PDL_IRQMON_ADC1                    IRQ26MON
    #define PDL_IRQMON_ADC2_LCD                IRQ27MON
    #define PDL_IRQMON_MFT_FRT                 IRQ28MON
    #define PDL_IRQMON_MFT_ICP                 IRQ29MON
    #define PDL_IRQMON_MFT_OPC                 IRQ30MON
    #define PDL_IRQMON_BT0_7_FLASH             IRQ31MON
#elif (PDL_MCU_INT_TYPE == PDL_FM0P_INT_TYPE_C)
    #define PDL_IRQMON_CSV_SWDT_LVD            IRQ00MON
    #define PDL_IRQMON_MFS0                    IRQ01MON
    #define PDL_IRQMON_MFS1                    IRQ02MON
//                                             IRQ03MON
    #define PDL_IRQMON_MFS3                    IRQ04MON
    #define PDL_IRQMON_MFS4                    IRQ05MON
//                                             IRQ06MON
    #define PDL_IRQMON_MFS6_SLAVE              IRQ07MON
    #define PDL_IRQMON_MFS7                    IRQ08MON
    #define PDL_IRQMON_ADC                     IRQ09MON
//                                             IRQ10MON
//                                             IRQ11MON
//                                             IRQ12MON
//                                             IRQ13MON
    #define PDL_IRQMON_OSC                     IRQ14MON  
    #define PDL_IRQMON_DT_RTC_WC               IRQ15MON
    #define PDL_IRQMON_INT0_1                  IRQ16MON 
    #define PDL_IRQMON_INT2_3                  IRQ17MON
    #define PDL_IRQMON_INT4_5                  IRQ18MON 
    #define PDL_IRQMON_INT6_7                  IRQ19MON
    #define PDL_IRQMON_INT8                    IRQ20MON 
//                                             IRQ21MON
    #define PDL_IRQMON_INT12_13                IRQ22MON
    #define PDL_IRQMON_INT15                   IRQ23MON
    #define PDL_IRQMON_BT0_4                   IRQ24MON
    #define PDL_IRQMON_BT1_5                   IRQ25MON
    #define PDL_IRQMON_BT2_6                   IRQ26MON
    #define PDL_IRQMON_BT3_7                   IRQ27MON
    #define PDL_IRQMON_CEC                     IRQ28MON
    #define PDL_IRQMON_ICC1_FLASH              IRQ29MON
    #define PDL_IRQMON_DSTC                    IRQ30MON
//                                             IRQ31MON
#else
    #error Interrupt type not found!
#endif  
  
  
/*****************************************************************************/
/* Check whether interrupt is enable when peripheral is inactive             */
/*****************************************************************************/
// Include adc.h if ADC is active and ADC interrupt is enabled.
#if (PDL_INTERRUPT_ENABLE_ADC0) || (PDL_INTERRUPT_ENABLE_ADC1) || (PDL_INTERRUPT_ENABLE_ADC2)
    #if defined(PDL_PERIPHERAL_ADC_ACTIVE)
        #include "adc\adc.h"
    #else
        #error Don't enable ADC interrupt when it is inactive!
    #endif
#endif    

// Include adc.h if BT is active and BT interrupt is enabled.
#if (PDL_INTERRUPT_ENABLE_BT0) || (PDL_INTERRUPT_ENABLE_BT1) || \
    (PDL_INTERRUPT_ENABLE_BT2) || (PDL_INTERRUPT_ENABLE_BT3) || \
    (PDL_INTERRUPT_ENABLE_BT4) || (PDL_INTERRUPT_ENABLE_BT5) || \
    (PDL_INTERRUPT_ENABLE_BT6) || (PDL_INTERRUPT_ENABLE_BT7)   
    #if defined(PDL_PERIPHERAL_BT_ACTIVE)
        #include "bt\bt.h"
    #else
        #error Don't enable BT interrupt when it is inactive!
    #endif
#endif  

// Include clk.h if clock is active and clock interrupt is enabled.      
#if (PDL_INTERRUPT_ENABLE_CLK == PDL_ON)   
    #if defined(PDL_PERIPHERAL_CLK_ACTIVE)
        #include "clk\clk.h"
    #else  
        #error Don't enable clock interrupt when it is inactive!
    #endif
#endif      

// Include csv.h if clock is active and clock interrupt is enabled.      
#if (PDL_INTERRUPT_ENABLE_CSV == PDL_ON)      
    #if defined(PDL_PERIPHERAL_CSV_ACTIVE)
        #include "csv\csv.h"
    #else 
        #error Don't enable CSV interrupt when it is inactive!
    #endif  
#endif      

// Include dma.h if DMA is active and DMA interrupt is enabled.      
#if (PDL_INTERRUPT_ENABLE_DMA0 == PDL_ON) || (PDL_INTERRUPT_ENABLE_DMA1 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_DMA2 == PDL_ON) || (PDL_INTERRUPT_ENABLE_DMA3 == PDL_ON)  
    #if defined(PDL_PERIPHERAL_DMA_ACTIVE)   
        #include "dma\dma.h"
    #else  
        #error Don't enable DMA interrupt when it is inactive!
    #endif
#endif      

// Include dstc.h if DSTC is active and DSTC interrupt is enabled.      
#if (PDL_INTERRUPT_ENABLE_DSTC == PDL_ON) 
    #if defined(PDL_PERIPHERAL_DSTC_ACTIVE)   
        #include "dstc\dstc.h"
    #else  
        #error Don't enable DSTC interrupt when it is inactive!
    #endif
#endif       
      
// Include dt.h if DT is active and DT interrupt is enabled.          
#if (PDL_INTERRUPT_ENABLE_DT0 == PDL_ON)         
    #if defined(PDL_PERIPHERAL_DT_ACTIVE)
        #include "dt\dt.h"
    #else
        #error Don't enable DT interrupt when it is inactive!
    #endif
#endif      

// Include exint.h if EXT INT is active and EXT INT is enabled.          
#if (PDL_INTERRUPT_ENABLE_EXINT0 == PDL_ON) || (PDL_INTERRUPT_ENABLE_EXINT1 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT2 == PDL_ON) || (PDL_INTERRUPT_ENABLE_EXINT3 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT4 == PDL_ON) || (PDL_INTERRUPT_ENABLE_EXINT5 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT6 == PDL_ON) || (PDL_INTERRUPT_ENABLE_EXINT7 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT8 == PDL_ON) || (PDL_INTERRUPT_ENABLE_EXINT9 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT10 == PDL_ON) || (PDL_INTERRUPT_ENABLE_EXINT11 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT12 == PDL_ON) || (PDL_INTERRUPT_ENABLE_EXINT13 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT14 == PDL_ON) || (PDL_INTERRUPT_ENABLE_EXINT15 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT16 == PDL_ON) || (PDL_INTERRUPT_ENABLE_EXINT17 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT18 == PDL_ON) || (PDL_INTERRUPT_ENABLE_EXINT19 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT20 == PDL_ON) || (PDL_INTERRUPT_ENABLE_EXINT21 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT22 == PDL_ON) || (PDL_INTERRUPT_ENABLE_EXINT23 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT24 == PDL_ON) || (PDL_INTERRUPT_ENABLE_EXINT25 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT26 == PDL_ON) || (PDL_INTERRUPT_ENABLE_EXINT27 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT28 == PDL_ON) || (PDL_INTERRUPT_ENABLE_EXINT29 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT30 == PDL_ON) || (PDL_INTERRUPT_ENABLE_EXINT31 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_NMI == PDL_ON)
    #if defined(PDL_PERIPHERAL_EXINT_ACTIVE) || defined (PDL_PERIPHERAL_NMI_ACTIVE)
        #include "exint\exint.h"
    #else
        #error Don't enable external interrupt when it is inactive!
    #endif
#endif      

// Include flash.h if Flash is active and Flash interrupt is enabled.  
#if (PDL_INTERRUPT_ENABLE_MAIN_FLASH == PDL_ON)      
    #if defined(PDL_PERIPHERAL_MAIN_FLASH_ACTIVE)
        #include "flash\mainflash.h"
    #else
        #error Don't enable Flash interrupt when it is inactive!
    #endif
#endif      

// Include i2cs.h if I2C Slave is active and I2C Slave interrupt is enabled.  
#if (PDL_INTERRUPT_ENABLE_I2CS0 == PDL_ON)   
    #if defined(PDL_PERIPHERAL_I2CS_ACTIVE)
        #include "i2cs\i2cs.h"
    #else
        #error Don't enable I2C Slave interrupt when it is inactive!
    #endif
#endif       
      
// Include i2sl.h if I2S_Lite is active and I2S_Lite interrupt is enabled.  
#if (PDL_INTERRUPT_ENABLE_I2SL0 == PDL_ON) || (PDL_INTERRUPT_ENABLE_I2SL1 == PDL_ON)      
    #if defined(PDL_PERIPHERAL_I2SL_ACTIVE)
        #include "i2sl\i2sl.h"
    #else
        #error Don't enable I2S-Lite interrupt when it is inactive!
    #endif
#endif       

// Include icc.h if ICC is active and ICC interrupt is enabled.  
#if (PDL_INTERRUPT_ENABLE_ICC0 == PDL_ON) || (PDL_INTERRUPT_ENABLE_ICC1 == PDL_ON)      
    #if defined(PDL_PERIPHERAL_ICC_ACTIVE)
        #include "icc\icc.h"
    #else
        #error Don't enable ICC interrupt when it is inactive!
    #endif
#endif    

// Include lcd.h if LCD is active and LCD interrupt is enabled.       
#if (PDL_INTERRUPT_ENABLE_LCD == PDL_ON)      
    #if defined(PDL_PERIPHERAL_LCD_ACTIVE)
        #include "lcd\lcd.h"
    #else  
        #error Don't enable LCD interrupt when it is inactive!
    #endif
#endif   

// Include lvd.h if LVD is active and LVD interrupt is enabled.       
#if (PDL_INTERRUPT_ENABLE_LVD == PDL_ON)      
    #if defined(PDL_PERIPHERAL_LVD_ACTIVE)
        #include "lvd\lvd.h"
    #else  
        #error Don't enable LVD interrupt when it is inactive!
    #endif
#endif      

// Include mfs.h if MFS is active and MFS interrupt is enabled.      
#if (PDL_INTERRUPT_ENABLE_MFS0 == PDL_ON) || (PDL_INTERRUPT_ENABLE_MFS1 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_MFS2 == PDL_ON) || (PDL_INTERRUPT_ENABLE_MFS3 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_MFS4 == PDL_ON) || (PDL_INTERRUPT_ENABLE_MFS5 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_MFS6 == PDL_ON) || (PDL_INTERRUPT_ENABLE_MFS7 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_MFS8 == PDL_ON) || (PDL_INTERRUPT_ENABLE_MFS9 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_MFS10 == PDL_ON) || (PDL_INTERRUPT_ENABLE_MFS11 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_MFS12 == PDL_ON) || (PDL_INTERRUPT_ENABLE_MFS13 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_MFS14 == PDL_ON) || (PDL_INTERRUPT_ENABLE_MFS15 == PDL_ON)  
    #if defined(PDL_PERIPHERAL_MFS_ACTIVE)
        #include "mfs\mfs.h"    
    #else
        #error Don't enable MFS interrupt when it is inactive!
    #endif
#endif      

// Include mft_frt.h if FRT is active and FRT interrupt is enabled.       
#if (PDL_INTERRUPT_ENABLE_MFT0_FRT == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_MFT1_FRT == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_MFT2_FRT == PDL_ON)  
    #if defined(PDL_PERIPHERAL_MFT_FRT_ACTIVE)  
        #include "mft\mft_frt.h"
    #else
        #error Don't enable MFT's FRT interrupt when it is inactive!
    #endif
#endif      

// Include mft_ocu.h if OCU is active and OCU interrupt is enabled.      
#if (PDL_INTERRUPT_ENABLE_MFT0_OCU == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_MFT1_OCU == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_MFT2_OCU == PDL_ON)
    #if defined(PDL_PERIPHERAL_MFT_OCU_ACTIVE)  
        #include "mft\mft_ocu.h"
    #else
        #error Don't enable MFT's OCU interrupt when it is inactive!
    #endif
#endif      

// Include mft_wfg.h if WFG is active and WFG interrupt is enabled.       
#if (PDL_INTERRUPT_ENABLE_MFT0_WFG == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_MFT1_WFG == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_MFT2_WFG == PDL_ON)
    #if defined(PDL_PERIPHERAL_MFT_WFG_ACTIVE)   
        #include "mft\mft_wfg.h"
    #else
        #error Don't enable MFT's WFG interrupt when it is inactive!
    #endif
#endif      

// Include mft_icu.h if ICU is active and ICU interrupt is enabled.      
#if (PDL_INTERRUPT_ENABLE_MFT0_ICU == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_MFT1_ICU == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_MFT2_ICU == PDL_ON)
    #if defined(PDL_PERIPHERAL_MFT_ICU_ACTIVE)
        #include "mft\mft_icu.h"
    #else
        #error Don't enable MFT's ICU interrupt when it is inactive!
    #endif
#endif      

// Include ppg.h if PPG is active and PPG interrupt is enabled.      
#if (PDL_INTERRUPT_ENABLE_PPG == PDL_ON)      
    #if defined(PDL_PERIPHERAL_PPG_ACTIVE)
        #include "ppg\ppg.h"  
    #else
        #error Don't enable PPG interrupt when it is inactive!
    #endif
#endif      

// Include qprc.h if QPRC is active and QPRC interrupt is enabled.         
#if (PDL_INTERRUPT_ENABLE_QPRC0 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_QPRC1 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_QPRC2 == PDL_ON)   
    #if defined(PDL_PERIPHERAL_QPRC_ACTIVE)
        #include "qprc\qprc.h"
    #else
        #error Don't enable QPRC interrupt when it is inactive!
    #endif
#endif  
      
// Include rc.h if RC is active and RC interrupt is enabled.        
#if (PDL_INTERRUPT_ENABLE_RC0 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_RC1 == PDL_ON)
    #if defined(PDL_PERIPHERAL_RC_ACTIVE)
        #include "rc\rc.h"
    #else
        #error Don't enable RC interrupt when it is inactive!
    #endif
#endif  
      
// Include rtc.h if RTC is active and RTC interrupt is enabled.        
#if (PDL_INTERRUPT_ENABLE_RTC0 == PDL_ON)      
    #if defined(PDL_PERIPHERAL_RTC_ACTIVE)
        #include "rtc\rtc.h"
    #else
        #error Don't enable RTC interrupt when it is inactive!
    #endif
#endif      

// Include wc.h if WC is active and WC interrupt is enabled.       
#if (PDL_INTERRUPT_ENABLE_WC0 == PDL_ON)         
    #if defined(PDL_PERIPHERAL_WC_ACTIVE)
        #include "wc\wc.h"
    #else
        #error Don't enable WC interrupt when it is inactive!
    #endif
#endif      

// Include wdg.h if watchdog is active and watchdog interrupt is enabled.      
#if (PDL_INTERRUPT_ENABLE_HWWDG == PDL_ON) || (PDL_INTERRUPT_ENABLE_SWWDG == PDL_ON)       
    #if defined(PDL_PERIPHERAL_WDG_ACTIVE)
        #include "wdg\wdg.h"
    #else
        #error Don't enable watchdog interrupt when it is inactive!
    #endif   
#endif      

//@} // PdlInterrupts
  
#ifdef __cplusplus
}
#endif  

#endif /* __CORTEX_M == 0x00 */
  
#endif /* __INTERRUPTS_H__ */

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
