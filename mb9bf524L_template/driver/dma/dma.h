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
/** \file dma.h
 **
 ** Headerfile for DMA functions
 **  
 ** History:
 **   - 2014-11-20  1.0  EZh  First version for FM uinversal PDL
 **
 ******************************************************************************/

#ifndef __DMA_H__
#define __DMA_H__

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "mcu.h"
#include "pdl_user.h"

#if (defined(PDL_PERIPHERAL_DMA_ACTIVE))

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/**
 ******************************************************************************
 ** \defgroup DmaGroup Direct Memory Access (DMA)
 **
 ** Provided functions of DMA module:
 ** 
 ** - Dma_InitChannel()
 ** - Dma_DeInitChannel() 
 ** - Dma_SetChannel() 
 ** - Dma_GetStopCause()
 ** - Dma_ClrStopCause()  
 ** - Dma_Enable()
 ** - Dma_Disable()
 ** - Dma_Pause()
 ** - Dma_Resume()
 ** - Dma_EnableIrq()
 ** - Dma_DisableIrq()
 ** - Dma_SetIrq10Selector()
 ** - Dma_SetIrq11Selector()
 ** - Dma_SetIrq24Selector()
 ** - Dma_SetIrq25Selector()  
 ** - Dma_SetIrq26Selector()
 ** - Dma_SetIrq27Selector()
 ** - Dma_SetIrq30Selector()
 ** - Dma_SetIrq31Selector()  
 **
 **  Dma_InitChannel() initializes a DMA channel with the structure type
 **  #stc_dma_config_t. 
 ** 
 **  Dma_InitChannel() resets a DMA channel to its initial status.
 **
 **  Dma_SetChannel() can enable/disable and triggers a channel by software. 
 **
 **  Dma_GetStopCause() gets the DMA completion cause and Dma_ClrStopCause() 
 **  clears the stop cause flags.
 **
 **  Dma_Enable() enables the entire DMA module and Dma_Disable() disables 
 **  entire DMA module.
 ** 
 **  Dma_Pause() pauses the entire DMA module and Dma_Resume() resumes 
 **  entire DMA module from pause status.
 **
 **  The interrupts can be configured in Dma_InitChannel(), after initialization,
 **  the interrrups can be enabled or disabled with Dma_EnableIrq() and 
 **  Dma_DisableIrq().
 **
 **  Dma_SetIrq10Selector() sets the source selection of IRQ10 which is used to
 **  trigger DMA.
 **
 **  Dma_SetIrq11Selector() sets the source selection of IRQ11 which is used to
 **  trigger DMA.
 ** 
 **  Dma_SetIrq24Selector() sets the source selection of IRQ24 which is used to
 **  trigger DMA.
 ** 
 **  Dma_SetIrq25Selector() sets the source selection of IRQ25 which is used to
 **  trigger DMA.
 **
 **  Dma_SetIrq26Selector() sets the source selection of IRQ26 which is used to
 **  trigger DMA.
 ** 
 **  Dma_SetIrq27Selector() sets the source selection of IRQ27 which is used to
 **  trigger DMA.
 **
 **  Dma_SetIrq30Selector() sets the source selection of IRQ30 which is used to
 **  trigger DMA.
 ** 
 **  Dma_SetIrq31Selector() sets the source selection of IRQ31 which is used to
 **  trigger DMA.  
 **  
 **  \note Only TYPE2 product of FM3 supports IRQ source selection for IRQ10,
 **  11, 24, 25, 26, 27, 30, 31.
 ** 
 ******************************************************************************/
//@{
  
/******************************************************************************/
/* Global pre-processor symbols/macros ('#define')                            */
/******************************************************************************/
#define PDL_DMA_CHANNELS        8u
#define PDL_DMA_INSTANCE_COUNT  (PDL_PERIPHERAL_ENABLE_DMA0 == PDL_ON) + \
                                (PDL_PERIPHERAL_ENABLE_DMA1 == PDL_ON) + \
                                (PDL_PERIPHERAL_ENABLE_DMA2 == PDL_ON) + \
                                (PDL_PERIPHERAL_ENABLE_DMA3 == PDL_ON) + \
                                (PDL_PERIPHERAL_ENABLE_DMA4 == PDL_ON) + \
                                (PDL_PERIPHERAL_ENABLE_DMA5 == PDL_ON) + \
                                (PDL_PERIPHERAL_ENABLE_DMA6 == PDL_ON) + \
                                (PDL_PERIPHERAL_ENABLE_DMA7 == PDL_ON) 
                             
// register address offset
#define DMA_DMACA_OFFSET 0x00ul
#define DMA_DMACB_OFFSET 0x04ul
#define DMA_DMASA_OFFSET 0x08ul
#define DMA_DMADA_OFFSET 0x0Cul
    
/******************************************************************************/
/* Local type definitions ('typedef')                                         */
/******************************************************************************/

/**
 ******************************************************************************
 ** \brief DMA transfer data width
 ******************************************************************************/
typedef enum en_dma_transfermode
{
    DmaBlockTransfer  = 0u,  ///< Block Transfer
    DmaBurstTransfer  = 1u,  ///< Burst transfer
    DmaDemandTransfer = 2u   ///< Demand transfer
} en_dma_transfermode_t ;

/**
 ******************************************************************************
 ** \brief DMA transfer data width
 ******************************************************************************/
typedef enum en_dma_transferwidth
{
    Dma8Bit  = 0u,  ///< 8 bit transfer via DMA
    Dma16Bit = 1u,  ///< 16 bit transfer via DMA
    Dma32Bit = 2u   ///< 32 bit transfer via DMA
} en_dma_transferwidth_t ;

#if (PDL_MCU_TYPE == PDL_FM3_TYPE2)
/**
 ******************************************************************************
 ** \brief DMA source selection of IRQ10
 **
 ** \note Only available for FM3 TYPE2 products
 ******************************************************************************/
typedef enum en_dma_irq10_sel
{
    DmaIrq10Bt4Irq0 = 0u,   ///< Base timer ch.4 Interrupt signal of IRQ0
    DmaIrq10Usb1Ep1 = 1u,   ///< USB ch.1 EP1 DRQ interrupt signal
    DmaIrq10Usb1Ep2 = 2u,   ///< USB ch.1 EP2 DRQ interrupt signal
    DmaIrq10Usb1Ep3 = 3u,   ///< USB ch.1 EP3 DRQ interrupt signal
    DmaIrq10Usb1Ep4 = 4u,   ///< USB ch.1 EP4 DRQ interrupt signal
    DmaIrq10Usb1Ep5 = 5u,   ///< USB ch.1 EP5 DRQ interrupt signal
    
}en_dma_irq10_sel_t;

/**
 ******************************************************************************
 ** \brief DMA source selection of IRQ11
 **
 ** \note Only available for FM3 TYPE2 products
 ******************************************************************************/
typedef enum en_dma_irq11_sel
{
    DmaIrq11Bt6Irq0 = 0u,   ///< Base timer ch.6 Interrupt signal of IRQ0
    DmaIrq11Usb1Ep1 = 1u,   ///< USB ch.1 EP1 DRQ interrupt signal
    DmaIrq11Usb1Ep2 = 2u,   ///< USB ch.1 EP2 DRQ interrupt signal
    DmaIrq11Usb1Ep3 = 3u,   ///< USB ch.1 EP3 DRQ interrupt signal
    DmaIrq11Usb1Ep4 = 4u,   ///< USB ch.1 EP4 DRQ interrupt signal
    DmaIrq11Usb1Ep5 = 5u,   ///< USB ch.1 EP5 DRQ interrupt signal
    
}en_dma_irq11_sel_t;

/**
 ******************************************************************************
 ** \brief DMA source selection of IRQ24
 **
 ** \note Only available for FM3 TYPE2 products
 ******************************************************************************/
typedef enum en_dma_irq24_sel
{
    DmaIrq24Mfs6Rx  = 0u,   ///< MFS ch.6 Receive interrupt signal
    DmaIrq24Usb1Ep1 = 1u,   ///< USB ch.1 EP1 DRQ interrupt signal
    DmaIrq24Usb1Ep2 = 2u,   ///< USB ch.1 EP2 DRQ interrupt signal
    DmaIrq24Usb1Ep3 = 3u,   ///< USB ch.1 EP3 DRQ interrupt signal
    DmaIrq24Usb1Ep4 = 4u,   ///< USB ch.1 EP4 DRQ interrupt signal
    DmaIrq24Usb1Ep5 = 5u,   ///< USB ch.1 EP5 DRQ interrupt signal
    
}en_dma_irq24_sel_t;

/**
 ******************************************************************************
 ** \brief DMA source selection of IRQ25
 **
 ** \note Only available for FM3 TYPE2 products
 ******************************************************************************/
typedef enum en_dma_irq25_sel
{
    DmaIrq25Mfs6Tx  = 0u,   ///< MFS ch.6 Send interrupt signal
    DmaIrq25Usb1Ep1 = 1u,   ///< USB ch.1 EP1 DRQ interrupt signal
    DmaIrq25Usb1Ep2 = 2u,   ///< USB ch.1 EP2 DRQ interrupt signal
    DmaIrq25Usb1Ep3 = 3u,   ///< USB ch.1 EP3 DRQ interrupt signal
    DmaIrq25Usb1Ep4 = 4u,   ///< USB ch.1 EP4 DRQ interrupt signal
    DmaIrq25Usb1Ep5 = 5u,   ///< USB ch.1 EP5 DRQ interrupt signal
    
}en_dma_irq25_sel_t;

/**
 ******************************************************************************
 ** \brief DMA source selection of IRQ26
 **
 ** \note Only available for FM3 TYPE2 products
 ******************************************************************************/
typedef enum en_dma_irq26_sel
{
    DmaIrq26Mfs7Rx  = 0u,   ///< MFS ch.7 Receive interrupt signal
    DmaIrq26Usb1Ep1 = 1u,   ///< USB ch.1 EP1 DRQ interrupt signal
    DmaIrq26Usb1Ep2 = 2u,   ///< USB ch.1 EP2 DRQ interrupt signal
    DmaIrq26Usb1Ep3 = 3u,   ///< USB ch.1 EP3 DRQ interrupt signal
    DmaIrq26Usb1Ep4 = 4u,   ///< USB ch.1 EP4 DRQ interrupt signal
    DmaIrq26Usb1Ep5 = 5u,   ///< USB ch.1 EP5 DRQ interrupt signal
    
}en_dma_irq26_sel_t;

/**
 ******************************************************************************
 ** \brief DMA source selection of IRQ27
 **
 ** \note Only available for FM3 TYPE2 products
 ******************************************************************************/
typedef enum en_dma_irq27_sel
{
    DmaIrq27Mfs7Tx  = 0u,   ///< MFS ch.7 Send interrupt signal
    DmaIrq27Usb1Ep1 = 1u,   ///< USB ch.1 EP1 DRQ interrupt signal
    DmaIrq27Usb1Ep2 = 2u,   ///< USB ch.1 EP2 DRQ interrupt signal
    DmaIrq27Usb1Ep3 = 3u,   ///< USB ch.1 EP3 DRQ interrupt signal
    DmaIrq27Usb1Ep4 = 4u,   ///< USB ch.1 EP4 DRQ interrupt signal
    DmaIrq27Usb1Ep5 = 5u,   ///< USB ch.1 EP5 DRQ interrupt signal
    
}en_dma_irq27_sel_t;

/**
 ******************************************************************************
 ** \brief DMA source selection of IRQ30
 **
 ** \note Only available for FM3 TYPE2 products
 ******************************************************************************/
typedef enum en_dma_irq30_sel
{
    DmaIrq30ExtIrq2  = 0u,   ///< External interrupt ch.2 signal
    DmaIrq30Usb1Ep1  = 1u,   ///< USB ch.1 EP1 DRQ interrupt signal
    DmaIrq30Usb1Ep2  = 2u,   ///< USB ch.1 EP2 DRQ interrupt signal
    DmaIrq30Usb1Ep3  = 3u,   ///< USB ch.1 EP3 DRQ interrupt signal
    DmaIrq30Usb1Ep4  = 4u,   ///< USB ch.1 EP4 DRQ interrupt signal
    DmaIrq30Usb1Ep5  = 5u,   ///< USB ch.1 EP5 DRQ interrupt signal
    
}en_dma_irq30_sel_t;

/**
 ******************************************************************************
 ** \brief DMA source selection of IRQ31
 **
 ** \note Only available for FM3 TYPE2 products
 ******************************************************************************/
typedef enum en_dma_irq31_sel
{
    DmaIrq31ExtIrq3  = 0u,   ///< External interrupt ch.3 signal
    DmaIrq31Usb1Ep1  = 1u,   ///< USB ch.1 EP1 DRQ interrupt signal
    DmaIrq31Usb1Ep2  = 2u,   ///< USB ch.1 EP2 DRQ interrupt signal
    DmaIrq31Usb1Ep3  = 3u,   ///< USB ch.1 EP3 DRQ interrupt signal
    DmaIrq31Usb1Ep4  = 4u,   ///< USB ch.1 EP4 DRQ interrupt signal
    DmaIrq31Usb1Ep5  = 5u,   ///< USB ch.1 EP5 DRQ interrupt signal
    
}en_dma_irq31_sel_t;
#endif

/**
 ******************************************************************************
 ** \brief DMA IDREQ number
 ******************************************************************************/
typedef enum en_dma_idreq
{
    Usb0Ep1        = 0u,   ///< Interrupt signal from EP1 DRQ of USB ch.0
    Usb0Ep2        = 1u,   ///< Interrupt signal from EP2 DRQ of USB ch.0
    Usb0Ep3        = 2u,   ///< Interrupt signal from EP3 DRQ of USB ch.0
    Usb0Ep4        = 3u,   ///< Interrupt signal from EP4 DRQ of USB ch.0
    Usb0Ep5        = 4u,   ///< Interrupt signal from EP5 DRQ of USB ch.0 
    Adc0           = 5u,   ///< Scan conversion interrupt signal from A/D converter unit0
    Adc1           = 6u,   ///< Scan conversion interrupt signal from A/D converter unit1
    Adc2           = 7u,   ///< Scan conversion interrupt signal from A/D converter unit2
    Bt0Irq0        = 8u,   ///< Interrupt signal from IRQ0 of base timer ch.0
    Bt2Irq0        = 9u,   ///< Interrupt signal from IRQ0 of base timer ch.2
    Bt4Irq0Sel10   = 10u,  ///< Default: Interrupt signal from IRQ0 of base timer ch.4, it can also be selected by Dma_SetIrq10Selector()
    Bt6Irq0Sel11   = 11u,  ///< Default: Interrupt signal from IRQ0 of base timer ch.6, it can also be selected by Dma_SetIrq11Selector()
    MfsRx0         = 12u,  ///< Receiving interrupt signal from MFS ch.0
    MfsTx0         = 13u,  ///< Sending interrupt signal from MFS ch.0   
    MfsRx1         = 14u,  ///< Receiving interrupt signal from MFS ch.1
    MfsTx1         = 15u,  ///< Sending interrupt signal from MFS ch.1
    MfsRx2         = 16u,  ///< Receiving interrupt signal from MFS ch.2
    MfsTx2         = 17u,  ///< Sending interrupt signal from MFS ch.2
    MfsRx3         = 18u,  ///< Receiving interrupt signal from MFS ch.3
    MfsTx3         = 19u,  ///< Sending interrupt signal from MFS ch.3 
    MfsRx4         = 20u,  ///< Receiving interrupt signal from MFS ch.4
    MfsTx4         = 21u,  ///< Sending interrupt signal from MFS ch.4 
    MfsRx5         = 22u,  ///< Receiving interrupt signal from MFS ch.5
    MfsTx5         = 23u,  ///< Sending interrupt signal from MFS ch.5 
    MfsRx6Sel24    = 24u,  ///< Default: Receiving interrupt signal from MFS ch.6, it can also be selected by Dma_SetIrq24Selector()
    MfsTx6Sel25    = 25u,  ///< Default: Sending interrupt signal from MFS ch.6, it can also be selected by Dma_SetIrq25Selector() 
    MfsRx7Sel26    = 26u,  ///< Default: Receiving interrupt signal from MFS ch.7, it can also be selected by Dma_SetIrq26Selector()
    MfsTx7Sel27    = 27u,  ///< Default: Sending interrupt signal from MFS ch.7, it can also be selected by Dma_SetIrq27Selector() 
    ExtIrq0        = 28u,  ///< Interrupt signal from external interrupt unit ch.0
    ExtIrq1        = 29u,  ///< Interrupt signal from external interrupt unit ch.1    
    ExtIrq2Sel30   = 30u,  ///< Default: Interrupt signal from external interrupt unit ch.2, it can also be selected by Dma_SetIrq30Selector()
    ExtIrq3Sel31   = 31u,  ///< Interrupt signal from external interrupt unit ch.3, it can also be selected by Dma_SetIrq31Selector() 
    Software       = 1234u ///< Software Demand (just a high number)    
} en_dma_idreq_t ;

/**
 ******************************************************************************
 ** \brief DMA stop cause
 ******************************************************************************/
typedef enum dma_stop_cause
{
    DmaStopByAddrOverflow        = 0u,  ///< DMA stops by address overflow
    DmaStopByTransferStopRqst    = 1u,  ///< DMA stops by transfer stop request
    DmaStopBySrcAccessError      = 2u,  ///< DMA stops by source access error
    DmaStopByDstAccessError      = 3u,  ///< DMA stops by destination access error
    DmaStopByTransferComplete    = 4u,  ///< DMA stops by transfer completion
    DmaStopByTransferPause       = 5u,  ///< DMA stops by transfer pause
    
}dma_stop_cause_t;

/**
 ******************************************************************************
 ** \brief DMA interrupt selection
 ******************************************************************************/
typedef struct stc_dma_irq_sel
{
    boolean_t bCompleteIrq;   ///< Select DMA transfer completion interrupt
    boolean_t bErrorIrq;      ///< Select DMA transfer error interrupt
    
}stc_dma_irq_sel_t, stc_dma_irq_en_t;

/**
 ******************************************************************************
 ** \brief DMA interrupt callback function
 ******************************************************************************/
typedef struct stc_dma_irq_cb
{
    func_ptr_t      pfnDmaCompletionIrqCb;         ///< Pointer to DMA transfer completion interrupt callback function
    func_ptr_arg1_t pfnDmaErrorIrqCb;              ///< Pointer to DMA transfer error interrupt callback function
    
}stc_dma_irq_cb_t;

/**
 ******************************************************************************
 ** \brief Datatype for holding internal data needed for DMA
 ******************************************************************************/
typedef struct stc_dma_intern_data
{
    func_ptr_t      apfnDmaIrqCompletionCb[PDL_DMA_INSTANCE_COUNT];   ///< DMA transfer completion callback pointer array
    func_ptr_arg1_t apfnDmaErrorIrqCb[PDL_DMA_INSTANCE_COUNT];        ///< DMA error callback pointer array

} stc_dma_intern_data_t;

/**
 ******************************************************************************
 ** \brief DMA configuration
 ******************************************************************************/
typedef struct stc_dma_config
{
    en_dma_idreq_t         enDmaIdrq ;             ///< ID Request number (see #en_dma_idreq_t for details)
    uint8_t                u8BlockCount ;          ///< Block counter
    uint16_t               u16TransferCount ;      ///< Transfer counter
    en_dma_transfermode_t  enTransferMode ;        ///< DMA transfer mode (see #en_dma_transfermode_t for details)
    en_dma_transferwidth_t enTransferWdith ;       ///< DMA transfer width (see #en_dma_transferwidth_t for details)
    uint32_t               u32SourceAddress;       ///< Source address
    uint32_t               u32DestinationAddress ; ///< Destination address
    boolean_t              bFixedSource ;          ///< TRUE = source address not increased
    boolean_t              bFixedDestination ;     ///< TRUE = destiantion address not increased
    boolean_t              bReloadCount ;          ///< TRUE = count is reloaded
    boolean_t              bReloadSource ;         ///< TRUE = source address is reloaded
    boolean_t              bReloadDestination ;    ///< TRUE = destination address is reloaded
    boolean_t              bEnableBitMask ;        ///< FALSE = Clear EB (bEnable) bit on completion (mandatory for transfer end!)
#if (PDL_INTERRUPT_ENABLE_DMA0 == PDL_ON) || (PDL_INTERRUPT_ENABLE_DMA1 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_DMA2 == PDL_ON) || (PDL_INTERRUPT_ENABLE_DMA3 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_DMA4 == PDL_ON) || (PDL_INTERRUPT_ENABLE_DMA5 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_DMA6 == PDL_ON) || (PDL_INTERRUPT_ENABLE_DMA7 == PDL_ON)   
    stc_dma_irq_en_t*      pstcIrqEn;              ///< Pointer to DMA interrupt enable structure
    stc_dma_irq_cb_t*      pstcIrqCb;              ///< Pointer to DMA interrupt callback function structure
    boolean_t              bTouchNvic;             ///< TRUE: enable NVIC, FALSE: disable NVIC
#endif    
    
} stc_dma_config_t ;

/******************************************************************************/
/* Global variable definitions ('extern')                                     */
/******************************************************************************/
extern stc_dma_intern_data_t stcDmaInternData;

/******************************************************************************/
/* Global function prototypes (definition in C source)                        */
/******************************************************************************/  
#if (PDL_INTERRUPT_ENABLE_DMA0 == PDL_ON) || (PDL_INTERRUPT_ENABLE_DMA1 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_DMA2 == PDL_ON) || (PDL_INTERRUPT_ENABLE_DMA3 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_DMA4 == PDL_ON) || (PDL_INTERRUPT_ENABLE_DMA5 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_DMA6 == PDL_ON) || (PDL_INTERRUPT_ENABLE_DMA7 == PDL_ON)  
void DmaIrqHandler(uint8_t u8DmaChannel) ;
en_result_t Dma_EnableIrq(uint8_t u8DmaChannel, 
                          stc_dma_irq_sel_t* pstcIrqSel);
en_result_t Dma_DisableIrq(uint8_t u8DmaChannel, 
                           stc_dma_irq_sel_t* pstcIrqSel);
#endif
/* Init/DeInit a DMA channel */
en_result_t Dma_InitChannel(uint8_t u8DmaChannel, volatile stc_dma_config_t* pstcConfig);
en_result_t Dma_DeInitChannel(uint8_t u8DmaChannel, boolean_t bTouchNvic);
/* Irq 10,11,24,25,26,27,30,31 Selection */
#if (PDL_MCU_TYPE == PDL_FM3_TYPE2)
en_result_t Dma_SetIrq10Selector(en_dma_irq10_sel_t enDmaIrq10Sel);
en_result_t Dma_SetIrq11Selector(en_dma_irq11_sel_t enDmaIrq11Sel);
en_result_t Dma_SetIrq24Selector(en_dma_irq24_sel_t enDmaIrq24Sel);
en_result_t Dma_SetIrq25Selector(en_dma_irq25_sel_t enDmaIrq25Sel);
en_result_t Dma_SetIrq26Selector(en_dma_irq26_sel_t enDmaIrq26Sel);
en_result_t Dma_SetIrq27Selector(en_dma_irq27_sel_t enDmaIrq27Sel);
en_result_t Dma_SetIrq30Selector(en_dma_irq30_sel_t enDmaIrq30Sel);
en_result_t Dma_SetIrq31Selector(en_dma_irq31_sel_t enDmaIrq30Sel);
#endif
/* Activate a DMA channel */
en_result_t Dma_SetChannel (uint8_t u8DmaChannel, 
                            boolean_t bEnable,
                            boolean_t bPause,
                            boolean_t bSoftwareTrigger) ;
/* Stop casue read/clear */
dma_stop_cause_t Dma_GetStopCause(uint8_t u8DmaChannel);
en_result_t Dma_ClrStopCause (uint8_t u8DmaChannel);
/* Globally enable/disable all channels */
en_result_t Dma_Enable(void) ;
en_result_t Dma_Disable(void) ;
/* Globally pause/resume all channels */
en_result_t Dma_Pause(void) ;
en_result_t Dma_Resume(void) ;


//@} // DmaGroup

#ifdef __cplusplus
}
#endif

#endif // #if (defined(PDL_PERIPHERAL_DMA_ACTIVE))

#endif /* __DMA_H__ */
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
