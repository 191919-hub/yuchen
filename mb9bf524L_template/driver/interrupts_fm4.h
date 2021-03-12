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
 **   - 2013-03-21  0.0.1  MWi   First version.
 **   - 2013-08-20  0.0.2  Mwi   IRQMON bit patterns now complete until MFS15
 **   - 2014-11-20  0.0.3  EZh   Port to FM universal PDL
 **
 ******************************************************************************/
#ifndef __INTERRUPTS_FM4_H__
#define __INTERRUPTS_FM4_H__

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_user.h"

#if (PDL_MCU_CORE == PDL_FM4_CORE)

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
  
/**
 ******************************************************************************
 ** \defgroup PdlInterrupts PDL Interrupt handling
 **
 ** \brief Description of the PDL Interrupt handling
 **
 ** Each device group of the FM4 MCUs may have individual interrupt service
 ** routine vector names. For this reason this file defines the device
 ** depenent vectors, if it corresponding resource is set to active in pdl.h.
 **
 ** Interrupt and Callback flow in PDL:
 ** \image html Interrupt_flow.png
 **
 ******************************************************************************/
//@{
 
/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/
#if (PDL_MCU_INT_TYPE == PDL_FM4_INT_TYPE_A)
    #define PDL_IRQMON_FCS                     IRQ00MON
    #define PDL_IRQMON_SWT                     IRQ01MON
    #define PDL_IRQMON_LVD                     IRQ02MON
    #define PDL_IRQMON_EXINT0                  IRQ011MON
    #define PDL_IRQMON_EXINT1                  IRQ012MON    
    #define PDL_IRQMON_EXINT2                  IRQ013MON
    #define PDL_IRQMON_EXINT3                  IRQ014MON 
    #define PDL_IRQMON_EXINT4                  IRQ015MON
    #define PDL_IRQMON_EXINT5                  IRQ016MON 
    #define PDL_IRQMON_EXINT6                  IRQ017MON
    #define PDL_IRQMON_EXINT7                  IRQ018MON   
    #define PDL_IRQMON_QPRC0                   IRQ019MON 
    #define PDL_IRQMON_QPRC1                   IRQ020MON
    #define PDL_IRQMON_MFT0_WFG                IRQ021MON 
    #define PDL_IRQMON_MFT1_WFG                IRQ022MON 
    #define PDL_IRQMON_MFT2_WFG                IRQ023MON
    #define PDL_IRQMON_MFT0_FRT_PEAK           IRQ024MON
    #define PDL_IRQMON_MFT0_FRT_ZERO           IRQ025MON
    #define PDL_IRQMON_MFT0_ICU                IRQ026MON
    #define PDL_IRQMON_MFT0_OCU                IRQ027MON
    #define PDL_IRQMON_MFT1_FRT_PEAK           IRQ028MON
    #define PDL_IRQMON_MFT1_FRT_ZERO           IRQ029MON
    #define PDL_IRQMON_MFT1_ICU                IRQ030MON
    #define PDL_IRQMON_MFT1_OCU                IRQ031MON
    #define PDL_IRQMON_MFT2_FRT_PEAK           IRQ032MON
    #define PDL_IRQMON_MFT2_FRT_ZERO           IRQ033MON
    #define PDL_IRQMON_MFT2_ICU                IRQ034MON
    #define PDL_IRQMON_MFT2_OCU                IRQ035MON  
    #define PDL_IRQMON_PPG0_2_4                IRQ036MON  
    #define PDL_IRQMON_PPG6_8_10               IRQ037MON
    #define PDL_IRQMON_PPG16_18_20             IRQ038MON
    #define PDL_IRQMON_BT0                     IRQ039MON
    #define PDL_IRQMON_BT1                     IRQ040MON
    #define PDL_IRQMON_BT2                     IRQ041MON
    #define PDL_IRQMON_BT3                     IRQ042MON
    #define PDL_IRQMON_BT4                     IRQ043MON
    #define PDL_IRQMON_BT5                     IRQ044MON
    #define PDL_IRQMON_BT6                     IRQ045MON
    #define PDL_IRQMON_BT7                     IRQ046MON
    #define PDL_IRQMON_DT                      IRQ047MON
    #define PDL_IRQMON_WC                      IRQ048MON
    #define PDL_IRQMON_EXBUS                   IRQ049MON
    #define PDL_IRQMON_RTC                     IRQ050MON
    #define PDL_IRQMON_EXINT8                  IRQ051MON
    #define PDL_IRQMON_EXINT9                  IRQ052MON
    #define PDL_IRQMON_EXINT10                 IRQ053MON
    #define PDL_IRQMON_EXINT11                 IRQ054MON
    #define PDL_IRQMON_EXINT12                 IRQ055MON
    #define PDL_IRQMON_EXINT13                 IRQ056MON
    #define PDL_IRQMON_EXINT14                 IRQ057MON
    #define PDL_IRQMON_EXINT15                 IRQ058MON
    #define PDL_IRQMON_CLK                     IRQ059MON  
    #define PDL_IRQMON_MFS0_RX                 IRQ060MON 
    #define PDL_IRQMON_MFS0_TX                 IRQ061MON
    #define PDL_IRQMON_MFS1_RX                 IRQ062MON 
    #define PDL_IRQMON_MFS1_TX                 IRQ063MON 
    #define PDL_IRQMON_MFS2_RX                 IRQ064MON 
    #define PDL_IRQMON_MFS2_TX                 IRQ065MON 
    #define PDL_IRQMON_MFS3_RX                 IRQ066MON 
    #define PDL_IRQMON_MFS3_TX                 IRQ067MON 
    #define PDL_IRQMON_MFS4_RX                 IRQ068MON 
    #define PDL_IRQMON_MFS4_TX                 IRQ069MON 
    #define PDL_IRQMON_MFS5_RX                 IRQ070MON 
    #define PDL_IRQMON_MFS5_TX                 IRQ071MON 
    #define PDL_IRQMON_MFS6_RX                 IRQ072MON 
    #define PDL_IRQMON_MFS6_TX                 IRQ073MON
    #define PDL_IRQMON_MFS7_RX                 IRQ074MON 
    #define PDL_IRQMON_MFS7_TX                 IRQ075MON 
    #define PDL_IRQMON_ADC0                    IRQ076MON 
    #define PDL_IRQMON_ADC1                    IRQ077MON
    #define PDL_IRQMON_USBF0                   IRQ078MON
    #define PDL_IRQMON_USB0                    IRQ079MON
    #define PDL_IRQMON_CAN0                    IRQ080MON
    #define PDL_IRQMON_CAN1                    IRQ081MON
    #define PDL_IRQMON_ETH0                    IRQ082MON
    #define PDL_IRQMON_DMA0                    IRQ083MON
    #define PDL_IRQMON_DMA1                    IRQ084MON
    #define PDL_IRQMON_DMA2                    IRQ085MON  
    #define PDL_IRQMON_DMA3                    IRQ086MON
    #define PDL_IRQMON_DMA4                    IRQ087MON
    #define PDL_IRQMON_DMA5                    IRQ088MON
    #define PDL_IRQMON_DMA6                    IRQ089MON
    #define PDL_IRQMON_DMA7                    IRQ090MON
    #define PDL_IRQMON_DSTC                    IRQ091MON
    #define PDL_IRQMON_EXINT16_17_18_19        IRQ092MON
    #define PDL_IRQMON_EXINT20_21_22_23        IRQ093MON
    #define PDL_IRQMON_EXINT24_25_26_27        IRQ094MON
    #define PDL_IRQMON_EXINT28_29_30_31        IRQ095MON
    #define PDL_IRQMON_QPRC2                   IRQ096MON
    #define PDL_IRQMON_QPRC3                   IRQ097MON
    #define PDL_IRQMON_BT8                     IRQ098MON
    #define PDL_IRQMON_BT9                     IRQ099MON
    #define PDL_IRQMON_BT10                    IRQ100MON
    #define PDL_IRQMON_BT11                    IRQ101MON
    #define PDL_IRQMON_BT12_13_14_15           IRQ102MON
    #define PDL_IRQMON_MFS8_RX                 IRQ103MON
    #define PDL_IRQMON_MFS8_TX                 IRQ104MON
    #define PDL_IRQMON_MFS9_RX                 IRQ105MON
    #define PDL_IRQMON_MFS9_TX                 IRQ106MON
    #define PDL_IRQMON_MFS10_RX                IRQ107MON
    #define PDL_IRQMON_MFS10_TX                IRQ108MON
    #define PDL_IRQMON_MFS11_RX                IRQ109MON
    #define PDL_IRQMON_MFS11_TX                IRQ110MON
    #define PDL_IRQMON_ADC2                    IRQ111MON
    #define PDL_IRQMON_RSV0                    IRQ112MON
    #define PDL_IRQMON_USB1F                   IRQ113MON
    #define PDL_IRQMON_USB1                    IRQ114MON
    #define PDL_IRQMON_RSV1                    IRQ115MON
    #define PDL_IRQMON_RSV2                    IRQ116MON
    #define PDL_IRQMON_I2S_PCRC                IRQ117MON
    #define PDL_IRQMON_SD                      IRQ118MON
    #define PDL_IRQMON_FLASH                   IRQ119MON
    #define PDL_IRQMON_MFS12_RX                IRQ120MON
    #define PDL_IRQMON_MFS12_TX                IRQ121MON
    #define PDL_IRQMON_MFS13_RX                IRQ122MON
    #define PDL_IRQMON_MFS13_TX                IRQ123MON
    #define PDL_IRQMON_MFS14_RX                IRQ124MON
    #define PDL_IRQMON_MFS14_TX                IRQ125MON
    #define PDL_IRQMON_MFS15_RX                IRQ126MON
    #define PDL_IRQMON_MFS15_TX                IRQ127MON
  
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
    (PDL_INTERRUPT_ENABLE_BT6) || (PDL_INTERRUPT_ENABLE_BT7) || \
    (PDL_INTERRUPT_ENABLE_BT8) || (PDL_INTERRUPT_ENABLE_BT9) || \
    (PDL_INTERRUPT_ENABLE_BT10) || (PDL_INTERRUPT_ENABLE_BT11) || \
    (PDL_INTERRUPT_ENABLE_BT12) || (PDL_INTERRUPT_ENABLE_BT13) || \
    (PDL_INTERRUPT_ENABLE_BT14) || (PDL_INTERRUPT_ENABLE_BT15) 
    #if defined(PDL_PERIPHERAL_BT_ACTIVE)
        #include "bt\bt.h"
    #else
        #error Don't enable BT interrupt when it is inactive!
    #endif
#endif  

// Include can.h if CAN is active and CAN interrupt is enabled.      
#if (PDL_INTERRUPT_ENABLE_CAN0 == PDL_ON) || (PDL_INTERRUPT_ENABLE_CAN1 == PDL_ON)  
    #if defined(PDL_PERIPHERAL_CAN_ACTIVE)
        #include "can\can.h"
    #else  
        #error Don't enable CAN interrupt when it is inactive!
    #endif
#endif       
      
// Include can.h if CANFD is active and CANFD interrupt is enabled.      
#if (PDL_INTERRUPT_ENABLE_CANFD0 == PDL_ON) || (PDL_INTERRUPT_ENABLE_CANFD1 == PDL_ON)  
    #if defined(PDL_PERIPHERAL_CANFD_ACTIVE)
        #include "can\canfd.h"
    #else  
        #error Don't enable CANFD interrupt when it is inactive!
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
    (PDL_INTERRUPT_ENABLE_DMA2 == PDL_ON) || (PDL_INTERRUPT_ENABLE_DMA3 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_DMA4 == PDL_ON) || (PDL_INTERRUPT_ENABLE_DMA5 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_DMA6 == PDL_ON) || (PDL_INTERRUPT_ENABLE_DMA7 == PDL_ON) 
    #if defined(PDL_PERIPHERAL_DMA_ACTIVE)   
        #include "dma\dma.h"
    #else  
        #error Don't enable DMA interrupt when it is inactive!
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

// Include dstc.h if DSTC is active and DSTC interrupt is enabled.      
#if (PDL_INTERRUPT_ENABLE_DSTC == PDL_ON) 
    #if defined(PDL_PERIPHERAL_DSTC_ACTIVE)   
        #include "dstc\dstc.h"
    #else  
        #error Don't enable DSTC interrupt when it is inactive!
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
    #if defined(PDL_PERIPHERAL_EXINT_ACTIVE) || defined(PDL_PERIPHERAL_NMI_ACTIVE)
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

#if (PDL_INTERRUPT_ENABLE_DUAL_FLASH == PDL_ON)      
    #if defined(PDL_PERIPHERAL_DUAL_FLASH_ACTIVE)
        #include "flash\dualflash.h"
    #else
        #error Don't enable Flash interrupt when it is inactive!
    #endif
#endif 
      
#if (PDL_INTERRUPT_ENABLE_WORK_FLASH == PDL_ON)      
    #if defined(PDL_PERIPHERAL_WORK_FLASH_ACTIVE)
        #include "flash\workflash.h"
    #else
        #error Don't enable Flash interrupt when it is inactive!
    #endif
#endif       

// Include icc.h if ICC is active and I2S interrupt is enabled.            
#if (PDL_INTERRUPT_ENABLE_ICC0 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_ICC1 == PDL_ON)     
    #if defined(PDL_PERIPHERAL_ICC_ACTIVE)
        #include "icc\icc.h"
    #else  
        #error Don't enable ICC interrupt when it is inactive!
    #endif
#endif      
      
// Include i2s.h if I2S is active and I2S interrupt is enabled.            
#if (PDL_INTERRUPT_ENABLE_I2S0 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_I2S1 == PDL_ON)     
    #if defined(PDL_PERIPHERAL_I2S_ACTIVE)
        #include "i2s\i2s.h"
    #else  
        #error Don't enable I2S interrupt when it is inactive!
    #endif
#endif
      
// Include i2sl.h if I2SL is active and I2SL interrupt is enabled.            
#if (PDL_INTERRUPT_ENABLE_I2SL0 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_I2SL1 == PDL_ON)     
    #if defined(PDL_PERIPHERAL_I2SL_ACTIVE)
        #include "i2sl\i2sl.h"
    #else  
        #error Don't enable I2SL interrupt when it is inactive!
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

// Include pcrc.h if PCRC is active and PCRC interrupt is enabled.      
#if (PDL_INTERRUPT_ENABLE_PCRC == PDL_ON)      
    #if defined(PDL_PERIPHERAL_PCRC_ACTIVE)
        #include "pcrc\pcrc.h"  
    #else
        #error Don't enable PCRC interrupt when it is inactive!
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

// Include sdif.h if SDIF is active and SDIF interrupt is enabled.         
#if (PDL_INTERRUPT_ENABLE_SD0 == PDL_ON) 
    #if defined(PDL_PERIPHERAL_SD_ACTIVE)
        #include "sdif\sdif.h"
    #else
        #error Don't enable SDIF interrupt when it is inactive!
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

#endif

#endif /* __INTERRUPTS_H__ */

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
