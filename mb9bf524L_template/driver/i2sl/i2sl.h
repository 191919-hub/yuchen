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
/** \file i2sl.h
 **
 ** Headerfile for I2SL functions
 **
 ** History:
 **   - 2015-01-05  0.1  EZh  First version for FM universal PDL.
 **
 ******************************************************************************/

#ifndef __I2SL_H__
#define __I2SL_H__

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "mcu.h"
#include "pdl_user.h"

#if (defined(PDL_PERIPHERAL_I2SL_ACTIVE))

/**
 ******************************************************************************
 ** \defgroup I2slGroup I2S-Lite (I2SL)
 **
 ** Provided functions of I2SL module:
 **
 ** - I2sl_Init()
 ** - I2sl_DeInit()  
 ** - I2sl_Start()
 ** - I2sl_Stop()
 ** - I2sl_GetStatus()
 ** - I2sl_ClrError()
 ** - I2sl_WriteTxFifo()
 ** - I2sl_ReadRxFifo()
 ** - I2sl_ResetFifo()
 **
 ** I2sl_Init() is used for configuration of a I2S-Lite (I2SL)
 ** channel with a structure of the type #stc_i2sl_config_t.
 **
 ** I2sl_DeInit() is used to reset all registers of a I2SL channel.
 **
 ** I2sl_Start() starts output of I2S clock and I2sl_Stop() stops output of
 ** I2S clock.
 **
 ** I2sl_GetStatus() gets the status of I2S-Lite according to the status type
 ** #en_i2sl_status_t.
 **
 ** I2sl_ClrError() clears error of I2SL.
 **
 ** I2sl_WriteTxFifo() writes the transmission FIFO of I2SL and I2sl_ReadRxFifo()
 ** read the transmission FIFO of I2SL. I2sl_ResetFifo() resets FIFO.
 **
 ******************************************************************************/
//@{

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/*****************************************************************************/
/* Global pre-processor symbols/macros ('define')                            */
/*****************************************************************************/
#define stc_i2sln_t     FM_MFS_CSIO_TypeDef 
#define stc_i2slregn_t  FM_I2SL_TypeDef   
  
#if (PDL_MCU_CORE == PDL_FM0P_CORE)  
#if (PDL_MCU_TYPE == PDL_FM0P_TYPE3)   
#define I2SL0       (*(volatile stc_i2sln_t *) FM_MFS4_CSIO)
#define I2SL1       (*(volatile stc_i2sln_t *) FM_MFS6_CSIO)  
#else
#define I2SL0       (*(volatile stc_i2sln_t *) FM_MFS5_CSIO)
#define I2SL1       (*(volatile stc_i2sln_t *) FM_MFS6_CSIO)
#endif
#elif (PDL_MCU_CORE == PDL_FM4_CORE)
#if (PDL_MCU_TYPE == PDL_FM4_TYPE5)  
#define I2SL0       (*(volatile stc_i2sln_t *) FM_MFS1_CSIO)
#define I2SL1       (*(volatile stc_i2sln_t *) FM_MFS2_CSIO) // Dummy 
#endif  
#endif  
  
#define I2SLREG0    (*(volatile stc_i2slregn_t *) FM_I2SL0)
#define I2SLREG1    (*(volatile stc_i2slregn_t *) FM_I2SL1)  
  
#define I2SL_INSTANCE_COUNT                               \
        (uint8_t)(PDL_PERIPHERAL_ENABLE_I2SL0 == PDL_ON) + \
        (uint8_t)(PDL_PERIPHERAL_ENABLE_I2SL1 == PDL_ON)
 
 
/******************************************************************************
 * I2S-Lite input clock selection
 ******************************************************************************/
typedef enum en_i2sl_clk_in
{
    I2slUsePclk    = 0u,          ///< I2S-Lite use PCLK as the source clock of divider
    I2slUseExtClk  = 1u,          ///< I2S-Lite use external clock(input from pins) as the source clock

}en_i2sl_clk_in_t;

/******************************************************************************
 * I2S-Lite transfer mode
 ******************************************************************************/
typedef enum en_i2sl_mode
{
    I2slPhilipsMode          = 0u,  ///< I2S-Lite Philips standard
    I2slMsbJustifiedMode     = 1u,  ///< I2S-Lite MSB justified standard
    
}en_i2sl_mode_t;

/******************************************************************************
 * I2S-Lite frame length selection 
 ******************************************************************************/ 
typedef enum en_i2sl_frame_len
{
    I2slFrame16Bit = 0u, ///< 16 bits frame.
    I2slFrame32Bit = 1u, ///< 32 bits frame.
    
}en_i2sl_frame_len_t;

/******************************************************************************
 * I2S-Lite FIFO width selection 
 ******************************************************************************/ 
typedef enum en_i2sl_fifo_width
{
    I2slFifoWidth16Bit = 0u,  ///< FIFO 16-bit access
    I2slFifoWidth32Bit = 1u,  ///< FIFO 32-bit access
      
}en_i2sl_fifo_width_t;

/**
 ******************************************************************************
 ** \brief I2S-Lite FIFO Selection
 ******************************************************************************/
typedef enum en_i2sl_fifo_sel
{
    I2slTxFifo1RxFifo2 = 0u,          ///< Transmit FIFO:FIFO1, Received FIFO:FIFO2
    I2slTxFifo2RxFifo1 = 1u,          ///< Transmit FIFO:FIFO2, Received FIFO:FIFO1
} en_i2sl_fifo_sel_t;

/**
 ******************************************************************************
 ** \brief I2S-Lite interrupt callback function.
 ******************************************************************************/
typedef struct stc_i2sl_irq_cb
{
    func_ptr_t pfnRxFifoIrqCb;            ///< I2S-Lite RX FIFO interrupt callback function
    func_ptr_t pfnTxFifoIrqCb;            ///< I2S-Lite TX FIFO interrupt callback function
    func_ptr_t pfnRxFifoOverrunIrqCb;     ///< I2S-Lite RX FIFO overflow interrupt callback function

}stc_i2sl_irq_cb_t;

/**
 ******************************************************************************
 ** \brief I2S interrupt enable structure
 ******************************************************************************/
typedef struct stc_i2sl_irq_en
{
    boolean_t bRxFifoIrq;           ///< I2S-Lite RX interrupt
    boolean_t bTxFifoIrq;           ///< I2S-Lite TX FIFO interrupt
    
}stc_i2sl_irq_en_t;

/**
 ******************************************************************************
 ** \brief I2S interrupt enable structure
 ******************************************************************************/
typedef enum en_i2sl_irq_sel
{
    I2slRxFifoIrq = 0u,  ///< I2S-Lite RX interrupt
    I2slTxFifoIrq = 1u,  ///< I2S-Lite TX interrupt
    
}en_i2sl_irq_sel_t;

/**
 ******************************************************************************
 ** \brief i2s FIFO Number
 ******************************************************************************/
typedef enum en_i2sl_fifo
{
    I2slFifo1 = 0u,                   ///< FIFO No.1
    I2slFifo2 = 1u,                   ///< FIFO No.2
    
} en_i2sl_fifo_t;

/******************************************************************************
 * I2S-Lite configuration definitions
 ******************************************************************************/
 typedef struct stc_i2s_config
 {
    en_i2sl_clk_in_t    enClkIn;         ///< I2S-Lite clcok source selection
    boolean_t           bClkOutputEn;    ///< I2S-Lite enable clock output
    uint8_t             u8ClkInDiv;      ///< I2S-Lite clock division set,1=>1/2,2=>1/4.....
    en_i2sl_mode_t      enTransferMode;  ///< FALSE: I2S-Lite Philips standard, TRUE: MSB justified standard
    boolean_t           bDataOutput;     ///< FALSE: data is received via I2S-Lite, TRUE: data is transferred via I2S-Lite
    boolean_t           bI2sWsPolarity;  ///< FALSE: ¡°1¡± when idle, ¡°0¡± when left channel.
                                         ///< TRUE: ¡°0¡± when idle, ¡°1¡± when left channel. 
    boolean_t           bCkWsOutputEn;   ///< FALSE: I2SCK and I2SWS output disable.
                                         ///< TRUE: I2SCK and I2SWS output enable.
    en_i2sl_frame_len_t  enFrameLength;  ///< Frame length selection
    boolean_t            bMaskBitOutput; ///< The serial output data of the invalid transmit frames
    
    en_i2sl_fifo_sel_t   enFifoSel;    ///< FIFO selection, see #en_mfs_fifo_sel_t for details
    en_i2sl_fifo_width_t enFifoWidth;  ///< FIFO access width
    uint8_t u8ByteCount1;              ///< Transfer data count for FIFO1
    uint8_t u8ByteCount2;              ///< Transfer data count for FIFO2
    
    stc_i2sl_irq_en_t* pstcI2slIrqEn;  ///< Pointer to I2S-Lite interrupt enable structure
    stc_i2sl_irq_cb_t* pstcI2slIrqCb;  ///< Pointer to I2S-Lite interrupt callback functions
    boolean_t          bTouchNvic;     ///< FALSE: don't enable NVIC, TRUE: enable NVIC
    
 }stc_i2sl_config_t;

/******************************************************************************
 * I2S-Lite status definitions
 ******************************************************************************/ 
typedef enum en_i2sl_status
{
    I2slTxFifoEmpty    = 0u,  ///< TX FIFO status (empty or not)
    I2slRxFifoFull     = 1u,  ///< RX FIFO status (full or not)
    I2slRxFifoOverrun  = 2u,  ///< RX FIFO status (Overrun or not)
    I2slBusBusy        = 3u,  ///< I2S bus is transferring data or not
    I2slClockStatus    = 4u,  ///< I2S clock is running or not
    
}en_i2sl_status_t;
 
/**
 ******************************************************************************
 ** \brief I2S interrupt callback function
 ******************************************************************************/
typedef struct stc_i2sl_intern_data
{
    func_ptr_t pfnTxFifoIrqCb;            ///< I2S-Lite TX FIFO interrupt callback function
    func_ptr_t pfnRxFifoIrqCb;            ///< I2S-Lite RX FIFO interrupt callback function
    func_ptr_t pfnRxFifoOverrunIrqCb;     ///< I2S-Lite RX FIFO interrupt callback function
    
}stc_i2sl_intern_data_t;

/// MFS instance data type
typedef struct stc_i2sl_instance_data
{
    volatile stc_i2sln_t*   pstcInstance;         ///< Pointer to registers of an MFS instance
    volatile stc_i2slregn_t*   pstcInstanceReg;   ///< Pointer to registers of an I2S register instance
    stc_i2sl_intern_data_t stcInternData;         ///< Module internal data of instance
} stc_i2sl_instance_data_t;

/******************************************************************************/
/* Local type definitions ('typedef')                                         */
/******************************************************************************/
/// Enumeration to define an index for each enabled I2S-Lite instance
typedef enum en_i2sl_instance_index
{  
#if (PDL_PERIPHERAL_ENABLE_I2SL0 == PDL_ON)  
    I2slInstanceIndexI2sl0,
#endif    
#if (PDL_PERIPHERAL_ENABLE_I2SL1 == PDL_ON) 
    I2slInstanceIndexI2sl1,
#endif    
    I2slInstanceIndexMax,
    I2slInstanceIndexUnknown = 0xFFu,
    
} en_i2sl_instance_index_t;

/******************************************************************************/
/* Global variable definitions ('extern')                                     */
/******************************************************************************/
/// Look-up table for all enabled I2S-Lite instances and their internal data
extern stc_i2sl_instance_data_t m_astcI2slInstanceDataLut[I2SL_INSTANCE_COUNT];

/******************************************************************************/
/* Global function prototypes (definition in C source)                        */
/******************************************************************************/
#if (PDL_INTERRUPT_ENABLE_I2SL0 == PDL_ON) || (PDL_INTERRUPT_ENABLE_I2SL1 == PDL_ON)
void I2sl_TxIrqHandler( volatile stc_i2sln_t*   pstcI2sl,
                        stc_i2sl_intern_data_t* pstcI2slInternData);
void I2sl_RxIrqHandler( volatile stc_i2sln_t*   pstcI2sl,
                        stc_i2sl_intern_data_t* pstcI2slInternData);
en_result_t I2sl_EnableIrq(volatile stc_i2sln_t* pstcI2sl, 
                           en_i2sl_irq_sel_t enIrqSel);
en_result_t I2sl_DisableIrq(volatile stc_i2sln_t* pstcI2sl, 
                            en_i2sl_irq_sel_t enIrqSel);
#endif

// Init/De-init
en_result_t I2sl_Init( volatile stc_i2sln_t* pstcI2sl,
                       stc_i2sl_config_t*    pstcConfig );
en_result_t I2sl_DeInit( volatile stc_i2sln_t* pstcI2sl, 
                         boolean_t bTouchNvic);

// I2S start/stop
en_result_t I2sl_Start(volatile stc_i2sln_t* pstcI2sl);
en_result_t I2sl_Stop(volatile stc_i2sln_t* pstcI2sl);

// Enable/Disable clock out
en_result_t I2sl_EnableClockOut(volatile stc_i2sln_t* pstcI2sl);
en_result_t I2sl_DisableClockOut(volatile stc_i2sln_t* pstcI2sl);

// I2S get/clear status
boolean_t I2sl_GetStatus(volatile stc_i2sln_t* pstcI2sl, 
                         en_i2sl_status_t enStatus);
en_result_t I2sl_ClrError(volatile stc_i2sln_t* pstcI2sl);

// I2S FIFO read/write
en_result_t I2sl_WriteTxFifo(volatile stc_i2sln_t* pstcI2sl, 
                             uint32_t u32Data);
uint32_t I2sl_ReadRxFifo(volatile stc_i2sln_t* pstcI2sl);

// Read FIFO count
uint8_t I2sl_GetFifoCount(volatile stc_i2sln_t* pstcI2sl,
                          en_i2sl_fifo_t enFifo);
// Reset FIFO
en_result_t I2sl_ResetFifo(volatile stc_i2sln_t* pstcI2sl, en_i2sl_fifo_t enFifo);


#ifdef __cplusplus
}
#endif

//@}

#endif  //#if (defined(PDL_PERIPHERAL_I2SL_ACTIVE))
#endif  //#ifndef __I2SL_H__
/*****************************************************************************/
/* EOF (not truncated)                                                       */
/*****************************************************************************/
