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
/*****************************************************************************/
/** \file i2s.c
 **
 ** I2S API functions header file.
 **
 ** History:
 ** - 2015-01-16  0.1  EZh   First version for FM universal PDL
 **
 *****************************************************************************/

#ifndef __I2S_H__
#define __I2S_H__

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include "mcu.h"
#include "pdl_user.h"

#if (defined(PDL_PERIPHERAL_I2S_ACTIVE))

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/*****************************************************************************/
/* Global pre-processor symbols/macros ('define')                            */
/*****************************************************************************/
#define stc_i2sn_t     FM_I2S_TypeDef 
  
#define I2S0       (*(volatile stc_i2sn_t *) FM_I2S0_BASE)
#define I2S1       (*(volatile stc_i2sn_t *) FM_I2S1_BASE)
  
#define I2S_INSTANCE_COUNT                                \
        (uint8_t)(PDL_PERIPHERAL_ENABLE_I2S0 == PDL_ON) + \
        (uint8_t)(PDL_PERIPHERAL_ENABLE_I2S1 == PDL_ON)
  
/**
 *****************************************************************************
 ** \defgroup I2sGroup Inter IC Sound (I2S)
 **
 ** \brief This section describes the interface for the I2S Unit (I2S)
 **
 ** Provided functions of I2S module:
 ** - I2s_InitClk()
 ** - I2s_DeInitClk()
 ** - I2s_EnablePll()
 ** - I2s_DisablePll()   
 ** - I2s_CheckPllReady()   
 ** - I2s_StartClk()
 ** - I2s_StopClk()   
 ** - I2s_Init()
 ** - I2s_DeInit()
 ** - I2s_Start()
 ** - I2s_Stop()  
 ** - I2s_GetStatus()
 ** - I2s_ClrStatus()
 ** - I2s_WriteTxFifo()
 ** - I2s_ReadRxFifo()
 ** - I2s_GetTxFifoNumber()
 ** - I2s_GetRxFifoNumber()   
 **
 ** I2s_InitClk() initializes I2S clock setting. I2s_DeInitClk() resets the 
 ** I2S clock setting.
 **  
 ** I2s_EnablePll() enables I2S PLL, I2s_DisablePll() disables I2S PLL. 
 ** I2s_CheckPllReady() checks if the PLL is stable.
 **
 ** I2s_StartClk() starts clock output to I2S macro, I2s_StopClk() stops
 ** clock output to I2S macro.
 **  
 ** I2s_Init() initializes an I2S instance according its configuration.
 ** I2s_DeInit() de-initializes an I2S instance and resets the hardware to
 ** initial values. 
 **  
 ** I2s_Start() starts I2S operation and I2s_Stop() stops the operation.
 **
 ** I2s_GetStatus() gets the I2s status according to the status type.
 ** I2s_ClrStatus() clears the I2s status according to the status type.
 **
 ** I2s_WriteTxFifo() writes the I2S TX FIFO by word and I2s_ReadRxFifo() 
 ** reads the I2S RX FIFO by word.
 **
 ** I2s_GetTxFifoNumber() gets the number of TX FIFO data count and 
 ** I2s_GetRxFifoNumber() gets the number of RX FIFO data count.  
 **  
 *****************************************************************************/
//@{

/******************************************************************************
 * Global definitions
 ******************************************************************************/

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/
typedef void (*i2s_func_ptr_t)(void);
typedef void (*i2s_error_func_ptr)(uint32_t);

/**
 *****************************************************************************
 ** \brief I2S PLL clock stabilization wait times (for 12.2 MHz input clock)
 *****************************************************************************/
typedef enum en_i2s_pll_wait_time
{
    I2sPllWait26us    = 0u,       ///< 26 us
    I2sPllWait53us    = 1u,       ///< 53 us
    I2sPllWait106us   = 2u,       ///< 106 us
    I2sPllWait213us   = 3u,       ///< 213 us
    I2sPllWait426us   = 4u,       ///< 426 us
    I2sPllWait853us   = 5u,       ///< 853 us
    I2sPllWait1ms70   = 6u,       ///< 1.70 ms
    I2sPllWait3ms41   = 7u        ///< 3.41 ms
} en_i2s_pll_wait_time_t;

/**
 *****************************************************************************
 ** \brief I2S ch.0 clock selection
 *****************************************************************************/
typedef enum en_i2s0_clk_in
{
    I2s0UsePllClk    = 0u,        ///< Use PLL clock for I2S clock
    I2s0UseI2sMclki  = 1u,        ///< Use MCLKI input for I2S clock
    
}en_i2s0_clk_in_t;

/**
 *****************************************************************************
 ** \brief I2S ch.1 clock selection
 *****************************************************************************/
typedef enum en_i2s1_clk_in
{
    I2s1UsePllClk    = 0u,        ///< Use PLL clock for I2S clock
    I2s1UseI2sMclki  = 1u,        ///< Use MCLKI input for I2S clock
    I2s1UseI2sMclki1 = 2u,        ///< Use MCLKI1 input for I2S clock
    
}en_i2s1_clk_in_t;

/**
 *****************************************************************************
 ** \brief I2S clock configuration
 **
 ** \note I2S ch.0 PLL clock output = (Fin * N * M / K)
 **       I2S ch.1 PLL clock output = (Fin * N * M1 / K)
 **       I2S ch.0 PLL clock =  I2S ch.0 PLL clock output / M
 **       I2S ch.1 PLL clock =  I2S ch.1 PLL clock output / M1
 **       I2S PLL clock output must be in a certain range, check it in the 
 **       product datasheet.
 *****************************************************************************/
typedef struct stc_i2s_clk_config
{
    en_i2s_pll_wait_time_t    enI2sPllWaitTime; ///< I2S PLL stabilization wait time, see #en_i2s_pll_wait_time_t for details
    uint8_t                   u8Pllk;           ///< K value of I2S PLL
    uint8_t                   u8Plln;           ///< N value of I2S PLL
    uint8_t                   u8Pllm;           ///< M value of I2S PLL
    uint8_t                   u8Pllm1;          ///< M1 value of I2S PLL
    en_i2s0_clk_in_t          enI2s0ClkIn;      ///< I2S ch.0 clock selection
    en_i2s1_clk_in_t          enI2s1ClkIn;      ///< I2S ch.1 clock selection  
} stc_i2s_clk_config_t;

/**
 *****************************************************************************
 ** \brief I2S start condition type (read, write)
 *****************************************************************************/
typedef enum en_i2s_start_condition
{
    I2sRead	    = 0u,                ///< I2S read 
    I2sWrite	= 1u                     ///< I2S write
} en_i2s_start_condition_t ;

/**
 *****************************************************************************
 ** \brief I2S packet receive completion timer
 *****************************************************************************/
typedef enum en_i2s_packet_timer
{
    NoOperation           = 0u,    ///< No operation
    Hclk54000Cycles       = 1u,    ///< Timeout cycle: 54000
    Hclk108000Cycles      = 2u,    ///< Timeout cycle: 108000
    Hclk216000Cycles      = 3u     ///< Timeout cycle: 216000
} en_i2s_packet_timer_t ;

/**
 *****************************************************************************
 ** \brief I2S sub frame configuration
 *****************************************************************************/
typedef struct stc_i2s_subframe_config
{
    uint8_t                     u8Snchn;                          ///< Sub Frame n Channel Numbers
    uint8_t                     u8Snchl;                          ///< Sub Frame n Channel Length
    uint8_t                     u8Snwdl;                          ///< Sub Frame n Word Length
} stc_i2s_subframe_config_t;

/**
 *****************************************************************************
 ** \brief I2S interrupt enable structure
 *****************************************************************************/
typedef struct stc_i2s_irq_en
{
    boolean_t                   bTxFifoUnderflow0Irq;       ///< TRUE: Enable TX-FIFO underflow 0 interrupt
    boolean_t                   bTxFifoUnderflow1Irq;       ///< TRUE: Enable TX-FIFO underflow 1 interrupt
    boolean_t                   bTxBlockSizeErrIrq;         ///< TRUE: Enable TX block size error interrupt
    boolean_t                   bFrameErrIrq;               ///< TRUE: Enable frame error interrupt
    boolean_t                   bTxFifoOverflowIrq;         ///< TRUE: Enable TX-FIFO overflow interrupt
    boolean_t                   bTxFifoIrq;                 ///< TRUE: Enable TX FIFO interrupt
    boolean_t                   bRxBlockSizeErrIrq;         ///< TRUE: Enable RX block size error interrupt
    boolean_t                   bRxFifoUnderflowIrq;        ///< TRUE: Enable RX-FIFO underflow interrupt
    boolean_t                   bRxFifoOverflowIrq;         ///< TRUE: Enable RX-FIFO overflow interrupt
    boolean_t                   bRxFifoIdleDetectIrq;       ///< TRUE: Enable RX FIFO idle detection interrupt
    boolean_t                   bRxFifoIrq;                 ///< TRUE: Enable RX FIFO interrupt  
    
}stc_i2s_irq_en_t;

/**
 *****************************************************************************
 ** \brief I2S interrupt enable structure
 *****************************************************************************/
typedef enum en_i2s_irq_sel
{
    I2sTxFifoUnderflow0Irq   = 0u,       ///< TX-FIFO underflow 0 interrupt
    I2sTxFifoUnderflow1Irq   = 1u,       ///< TX-FIFO underflow 1 interrupt
    I2sTxBlockSizeErrIrq     = 2u,       ///< TX block size error interrupt
    I2sFrameErrIrq           = 3u,       ///< Frame error interrupt
    I2sTxFifoOverflowIrq     = 4u,       ///< TX-FIFO overflow interrupt
    I2sTxFifoIrq             = 5u,       ///< TX FIFO interrupt
    I2sRxBlockSizeErrIrq     = 6u,       ///< RX block size error interrupt
    I2sRxFifoUnderflowIrq    = 7u,       ///< RX-FIFO underflow interrupt
    I2sRxFifoOverflowIrq     = 8u,       ///< RX-FIFO overflow interrupt
    I2sRxFifoIdleDetectIrq   = 9u,       ///< RX FIFO idle detection interrupt
    I2sRxFifoIrq             = 10u,      ///< RX FIFO interrupt  
    
}en_i2s_irq_sel_t;

/**
 *****************************************************************************
 ** \brief I2S status enumeration
 *****************************************************************************/   
typedef enum en_i2s_status
{
    I2sDmaTxChBlockSizeErr     = 0u,  ///< DMA transmit channel block size error
    I2sDmaRxChBlockSizeErr     = 1u,  ///< DMA receive channel block size error
    I2sFrameErr                = 2u,  ///< Frame error
    I2sTxFifoUnderflow1        = 3u,  ///< TX FIFO underflow interrupt flag 1
    I2sTxFifoUnderflow0        = 4u,  ///< TX FIFO underflow interrupt flag 0
    I2sTxFifoOverflow          = 5u,  ///< TX FIFO overflow interrupt flag 
    I2sRxFifoUnderflow         = 6u,  ///< RX FIFO underflow interrupt flag
    I2sRxFifoOverflow          = 7u,  ///< RX FIFO overflow interrupt flag
    I2sRxFifoFifoIdle          = 8u,  ///< RX FIFO idle interrupt flag
    I2sBusy                    = 9u,  ///< I2S transmit busy status
    I2sTxFifoMatchThreshold    = 10u, ///< TX FIFO interrupt flag
    I2sRxFifoMatchThreshold    = 11u, ///< RX FIFO interrupt flag
  
}en_i2s_status_t;
  
/**
 *****************************************************************************
 ** \brief I2S interrupt callback function structure
 *****************************************************************************/
typedef struct stc_i2s_irq_cb
{
    func_ptr_t                   pfnTxFifoUnderflow0IrqCb;       ///< TRUE: Enable TX-FIFO underflow 0 interrupt
    func_ptr_t                   pfnTxFifoUnderflow1IrqCb;       ///< TRUE: Enable TX-FIFO underflow 1 interrupt
    func_ptr_t                   pfnTxBlockSizeErrIrqCb;         ///< TRUE: Enable TX block size error interrupt
    func_ptr_t                   pfnFrameErrIrqCb;               ///< TRUE: Enable frame error interrupt
    func_ptr_t                   pfnTxFifoOverflowIrqCb;         ///< TRUE: Enable TX-FIFO overflow interrupt
    func_ptr_t                   pfnTxFifoIrqCb;                 ///< TRUE: Enable TX FIFO interrupt
    func_ptr_t                   pfnRxBlockSizeErrIrqCb;         ///< TRUE: Enable RX block size error interrupt
    func_ptr_t                   pfnRxFifoUnderflowIrqCb;        ///< TRUE: Enable RX-FIFO underflow interrupt
    func_ptr_t                   pfnRxFifoOverflowIrqCb;         ///< TRUE: Enable RX-FIFO overflow interrupt
    func_ptr_t                   pfnRxFifoIdleDetectIrqCb;       ///< TRUE: Enable RX FIFO idle detection interrupt
    func_ptr_t                   pfnRxFifoIrqCb;                 ///< TRUE: Enable RX FIFO interrupt  
    
}stc_i2s_irq_cb_t, stc_i2s_intern_data_t;

/**
 *****************************************************************************
 ** \brief I2S configuration
 *****************************************************************************/
typedef struct stc_i2s_config
{
    uint8_t                     u8ClockDiv;                       ///< Clock divider settings
    uint16_t                    u16OverheadBits;                  ///< Number of overhead bits
    boolean_t                   bMaskBit;                         ///< TRUE: Serial Ouput Data in invalid/empty case
    boolean_t                   bMasterMode;                      ///< TRUE: Master, FALSE: Slave
    boolean_t                   bSubFrame01;                      ///< TRUE: Sub Frame 0 and 1, FALSE: only Sub Frame 0
    boolean_t                   bFifoTwoWords;                    ///< TRUE: FIFO divided in two 16-bit words, FALSE: FIFO 32-Bit word
    boolean_t                   bBclkDivByMclk;                   ///< TRUE: Divide I2SMCLK and output as BLCK, FALSE: Divide HCLK and output as BCLK
    boolean_t                   bBitExtensionHigh;                ///< TRUE: Bit extension '1', FALSE: Bit extension '0'
    boolean_t                   bFreeRunMode;                     ///< TRUE: Frame sync signal is free running, FALSE: Frame sync signal in burst mode
    boolean_t                   bLsbFirst;                        ///< TRUE: Shift with LSB first, FALSE: Shift with MSB first
    boolean_t                   bSampleAtEnd;                     ///< TRUE: Sample at end of bit reception, FALSE: Sample at middle of bit reception
    boolean_t                   bClockpolarity;                   ///< TRUE: Data at falling edge of I2SCK, FALSE: Data ar risig edge of I2SCK
    boolean_t                   bWordSelectSamePhase;             ///< TRUE: I2SWS at same time of first bit, FALSE: I2SWS one block before first bit
    boolean_t                   bWordSelectLength;                ///< TRUE: I2SWS width one channel length, FALSE: I2SWS with one I2SCK pulse
    boolean_t                   bWordSelectPolarity;              ///< TRUE: I2SWS is '1', '0' when idle, FALSE: I2SWS is '0', '1' when idle
    stc_i2s_subframe_config_t   stcSubframe0;                     ///< Sub Frame 0 config
    stc_i2s_subframe_config_t   stcSubframe1;                     ///< Sub Frame 1 config
    uint32_t                    u32S0ch;                          ///< Sub Frame 0 Channel Enable
    uint32_t                    u32S1ch;                          ///< Sub Frame 1 Channel Enable
    uint8_t                     u8TxFifoThreshold;                ///< Treshold value of TX-FIFO for generating an interrupt
    uint8_t                     u8RxFifoThreshold;                ///< Treshold value of RX-FIFO for generating an interrupt
    en_i2s_packet_timer_t       enPacketReceiveCompletionTimer;   ///< see #en_i2s_packet_timer_t for details
    boolean_t                   bTxEnable;                        ///< TRUE: Transmitter enable, FALSE: Transmitter disable
    boolean_t                   bRxEnable;                        ///< TRUE: Receiver enable, FALSE: Receiver disable
    boolean_t                   bTxDmaEnable;                     ///< TRUE: Enable TX DMA
    boolean_t                   bRxDmaEnable;                     ///< TRUE: Enable RX DMA
#if (PDL_INTERRUPT_ENABLE_I2S0 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_I2S1 == PDL_ON)   
    stc_i2s_irq_en_t*           pstcIrqEn;                        ///< Pointer to interrupt enable structure
    stc_i2s_irq_cb_t*           pstcIrqCb;                        ///< Pointer to interrupt callback structure
    boolean_t                   bTouchNvic;                       ///< TRUE: Enable NVIC, FALSE: disable NVIC
#endif    
} stc_i2s_config_t;

/// Enumeration to define an index for each enabled I2S instance
typedef enum en_i2s_instance_index
{
  #if (PDL_PERIPHERAL_ENABLE_I2S0 == PDL_ON)
    I2sInstanceIndexI2s0,
  #endif
  #if (PDL_PERIPHERAL_ENABLE_I2S1 == PDL_ON)
    I2sInstanceIndexI2s1,
  #endif
    I2sInstanceIndexMax
} en_i2s_instance_index_t;

/// I2S instance data type
typedef struct stc_i2s_instance_data
{
  volatile stc_i2sn_t*  pstcInstance;  ///< pointer to registers of an instance
  stc_i2s_intern_data_t stcInternData; ///< module internal data of instance
} stc_i2s_instance_data_t;

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/
/// Look-up table for all enabled I2S instances and their internal data
extern stc_i2s_instance_data_t m_astcI2sInstanceDataLut[I2S_INSTANCE_COUNT];

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/
/* I2S interrupt */
#if (PDL_INTERRUPT_ENABLE_I2S0 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_I2S1 == PDL_ON) 
void I2sIrqHandler( volatile stc_i2sn_t*   pstcI2s,
                    stc_i2s_intern_data_t* pstcI2sInternData);      
en_result_t  I2s_EnableIrq(volatile stc_i2sn_t* pstcI2s,
                           en_i2s_irq_sel_t enIrqSel);
en_result_t  I2s_DisableIrq(volatile stc_i2sn_t* pstcI2s,
                            en_i2s_irq_sel_t enIrqSel);
#endif
/* I2S clock */
en_result_t I2s_InitClk(stc_i2s_clk_config_t* pstcConfig) ;
en_result_t I2s_DeInitClk(void) ;
en_result_t I2s_EnablePll(boolean_t bWaitPolling);
en_result_t I2s_CheckPllReady(void) ;
en_result_t I2s_DisablePll(void) ;
void I2s_StartClk(volatile stc_i2sn_t* pstcI2s) ;
void I2s_StopClk(volatile stc_i2sn_t* pstcI2s) ;
/* I2S */
en_result_t I2s_Init(volatile stc_i2sn_t* pstcI2s, stc_i2s_config_t* pstcConfig) ;
en_result_t I2s_DeInit(volatile stc_i2sn_t* pstcI2s, boolean_t bTouchNvic) ;
en_result_t I2s_Start(volatile stc_i2sn_t* pstcI2s) ;
en_result_t I2s_Stop(volatile stc_i2sn_t* pstcI2s) ;
en_result_t I2s_Reset(volatile stc_i2sn_t* pstcI2s);
en_result_t I2s_EnableTx(volatile stc_i2sn_t* pstcI2s);
en_result_t I2s_DisableTx(volatile stc_i2sn_t* pstcI2s);
en_result_t I2s_EnableRx(volatile stc_i2sn_t* pstcI2s);
en_result_t I2s_DisableRx(volatile stc_i2sn_t* pstcI2s);
boolean_t I2s_GetStatus(volatile stc_i2sn_t* pstcI2s, 
                        en_i2s_status_t enStatus);
en_result_t I2s_ClrStatus(volatile stc_i2sn_t* pstcI2s, 
                          en_i2s_status_t enStatus);
en_result_t I2s_WriteTxFifo(volatile stc_i2sn_t* pstcI2s, uint32_t u32Data);
uint32_t I2s_ReadRxFifo(volatile stc_i2sn_t* pstcI2s);
uint8_t I2s_GetTxFifoNumber(volatile stc_i2sn_t* pstcI2s);
uint8_t I2s_GetRxFifoNumber(volatile stc_i2sn_t* pstcI2s);

//@}
#ifdef __cplusplus
}
#endif

#endif

#endif /* __I2S_H__ */
/*****************************************************************************/
/* EOF (not truncated)                                                       */
/*****************************************************************************/
