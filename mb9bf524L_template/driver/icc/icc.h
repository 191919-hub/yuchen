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
/** \file icc.h
 **
 ** Headerfile for IC Card functions
 **  
 **
 ** History:
 **   - 2014-09-02  1.0  RQi   First version for FM universal PDL.
 **
 ******************************************************************************/

#ifndef __ICC_H__
#define __ICC_H__

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "mcu.h"
#include "pdl_user.h"

#if (defined(PDL_PERIPHERAL_ICC_ACTIVE))

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/**
 ******************************************************************************
 ** \defgroup IccGroup IC Card Interface (ICC)
 **
 ** Provided functions of ICC module:
 ** 
 ** - Icc_Init()
 ** - Icc_DeInit()
 ** - Icc_Enable()
 ** - Icc_Disable() 
 ** - Icc_EnableIrq()
 ** - Icc_DisableIrq()
 ** - Icc_SetBaudRate()
 ** - Icc_GetStatus()
 ** - Icc_GetIrqStatus()
 ** - Icc_SendData()
 ** - Icc_SendFifoData()
 ** - Icc_ReceiveData()
 ** - Icc_ReceiveFifoData()
 ** - Icc_ClearWriteFifo()
 ** - Icc_ClearReadFifo()
 ** - Icc_SetWriteFifoLevel()
 ** - Icc_SetReadFifoLevel()
 ** - Icc_GetWriteFifoCurCnt()
 ** - Icc_GetReadFifoCurCnt() 
 ** - Icc_SetVpenPinLevel()
 ** - Icc_SetVccPinLevel()
 ** - Icc_SetRstPinLevel()
 ** - Icc_SetClkPinLevel()  
 ** - Icc_SetDataPinLevel()    
 ** - Icc_SetIdleTimerCnt()
 ** - Icc_StartIdleTimer()
 ** - Icc_StopIdleTimer() 
 **
 ** Icc_Init() is used to initialize an ICC instance with
 ** parameter pstcConfig of type #stc_icc_config_t.
 **
 ** Icc_DeInit() is used to reset all ICC related configuration register.
 **  
 ** The interrupt can be configured in Icc_Init() first, after initialization,
 ** Icc_EnableIrq() enables ICC interrupt sources selected by parameter
 ** Icc_EnableIrq#enIrqSel. Icc_DisableIrq() disables the ICC interrupt 
 ** sources selected by parameter Icc_EnableIrq#enIrqSel.
 **
 ** Icc_SetBaudRate() provides a possibility to change the ICC baudrate.
 **
 ** Icc_GetStatus() gets the status selected by structure type #en_icc_status_t 
 ** and Icc_GetIrqStatus() gets the IRQ status selected by the structure type 
 ** #en_icc_irq_sel_t.
 **
 ** Icc_SendData() writes a byte data into ICC transfer buffer 
 ** Icc_SendFifoData() writes a byte data into ICC FIFO transfer buffer  
 ** Icc_ReceiveData() reads a byte data from ICC receive buffer.  
 ** Icc_ReceiveFifoData() reads a byte data from ICC FIFO receive buffer.  
 **  
 ** Icc_ClearWriteFifo() clear the write FIFO buffer of ICC.  
 **   
 ** Icc_ClearReadFifo() clear the read FIFO buffer of ICC.  
 **   
 ** Icc_SetWriteFifoLevel() sets the write FIFO level of ICC. 
 **
 ** Icc_SetReadFifoLevel() sets the read FIFO level of ICC.  
 **
 ** Icc_GetWriteFifoCurCnt() gets the current write FIFO count of ICC. 
 **
 ** Icc_GetReadFifoCurCnt() gets the current read FIFO count of ICC.  
 **  
 ** Icc_SetVpenPinLevel() is used to set the level of VPEN pin.
 ** 
 ** Icc_SetVccPinLevel() is used to set the level of VPEN pin.
 ** 
 ** Icc_SetRstPinLevel() is used to set the level of Reset pin.
 **
 ** Icc_SetClkPinLevel() is used to set the level of Clock pin.  
 **
 ** Icc_SetDataPinLevel() is used to set the level of Data pin.    
 ** 
 ** Icc_StartIdleTimer() is used to start idle timer and Icc_StopIdleTimer()
 ** is used to stop idle timer.
 ** 
 ** Icc_SetIdleTimerCnt() is used to set the count of Idle Timer.
 **  
 ******************************************************************************/
//@{

/******************************************************************************
 * Global type definitions
 ******************************************************************************/
#define stc_iccn_t FM_ICC_TypeDef

#define ICC0       (*((volatile stc_iccn_t *) FM_ICC0_BASE))
#define ICC1       (*((volatile stc_iccn_t *) FM_ICC1_BASE))

#define ICC_INSTANCE_COUNT                                \
        (uint8_t)(PDL_PERIPHERAL_ENABLE_ICC0 == PDL_ON) + \
        (uint8_t)(PDL_PERIPHERAL_ENABLE_ICC1 == PDL_ON)
            
/******************************************************************************
 * Global type definitions
 ******************************************************************************/
/**
 ******************************************************************************
 ** \brief ICC Idle timer source
 ******************************************************************************/
typedef enum en_icc_idletmrclk
{
    IccCardClk           = 0u,      ///< Idle timer is clocked by card clock(ICx_CLK)
    IccEtuClk            = 1u,      ///<  Idle timer is clocked by baud clock (ETU clock).
} en_icc_idletmrclk_t;

/**
 ******************************************************************************
 ** \brief ICC data direction
 ******************************************************************************/
typedef enum en_icc_data_dir
{
    IccDataLsbFirst = 0u,       ///< LSB first
    IccDataMsbFirst = 1u,       ///< MSB first
}en_icc_data_dir_t; 

/**
 ******************************************************************************
 ** \brief ICC transmit data format
 ******************************************************************************/
typedef enum en_icc_tx_data
{
    IccTx8Even2     = 0u,        ///< 8 bit with even parity and 2 stop bit
    IccTx8Odd2      = 1u,        ///< 8 bit with odd parity and 2 stop bit
    IccTx8None2     = 2u,        ///< 8 bit with none parity and 2 stop bit
}en_icc_tx_data_t; 

/**
 ******************************************************************************
 ** \brief ICC receive data format
 ******************************************************************************/
typedef enum en_icc_rx_data
{
    IccRx8Even1     = 0u,       ///< 8 bit with even parity and 2 stop bit
    IccRx8Odd1      = 1u,       ///< 8 bit with odd parity and 2 stop bit
    IccRx8None2     = 2u,       ///< 8 bit with none parity and 2 stop bit
    IccRx8None1     = 3u,       ///< 8 bit with none parity and 1 stop bit
    IccRx9None1     = 4u,       ///< 9 bit with none parity and 1 stop bit
}en_icc_rx_data_t; 

/**
 ******************************************************************************
 ** \brief ICC interrupt types
 ******************************************************************************/
typedef enum en_icc_irq_sel
{
    IccRxFullIrq          = 0u,             ///< Receive buffer full interrupt
    IccTxEmptyIrq         = 1u,             ///< Transmit buffer empty interrupt
    IccRxStartBitIrq      = 2u,             ///< Receive start bit detected interrupt
    IccCardEventIrq       = 3u,             ///< Card insert/pull out interrupt
    IccIdleTimerIrq       = 4u,             ///< Idle timer interrupt
    IccReadFifoIrq        = 5u,             ///< Receive FIFO interrupt
    IccWriteFifoIrq       = 6u,             ///< Transfer FIFO interrupt
    IccReadFifoOverrunIrq = 7u,             ///< Receive FIFO overrun interrupt

}en_icc_irq_sel_t;

/**
 ******************************************************************************
 ** \brief ICC interrupt enable structure
 ******************************************************************************/
typedef struct stc_icc_irq_en
{
    boolean_t bRxFullIrq;               ///< Receive buffer full interrupt
    boolean_t bTxEmptyIrq;              ///< Transmit buffer empty interrupt
    boolean_t bRxStartBitIrq;           ///< Receive start bit detected interrupt
    boolean_t bCardEventIrq;            ///< Card insert/pull out interrupt
    boolean_t bIdleTimerIrq;            ///< Idle timer interrupt
    boolean_t bReadFifoIrq;             ///< Receive FIFO interrupt
    boolean_t bWriteFifoIrq;            ///< Transfer FIFO interrupt
    boolean_t bReadFifoOverrunIrq;      ///< Receive FIFO overrun interrupt

}stc_icc_irq_en_t;


/**
 ******************************************************************************
 ** \brief ICC interrupt callback function
 ******************************************************************************/
typedef struct stc_icc_irq_cb
{
    func_ptr_t pfnRxFullIrqCb;          ///< Receive buffer full interrupt callback function pointer
    func_ptr_t pfnTxEmptyIrqCb;         ///< Transmit buffer empty interrupt callback function pointer
    func_ptr_t pfnRxStartBitIrqCb;      ///< Receive start bit detected interrupt callback function pointer
    func_ptr_t pfnCardEventIrqCb;       ///< Card insert/pull out interrupt callback function pointer
    func_ptr_t pfnIdleTimerIrqCb;       ///< Idle timer interrupt callback function pointer
    func_ptr_t pfnReadFifoIrqCb;        ///< Receive FIFO interrupt callback function pointer
    func_ptr_t pfnWriteFifoIrqCb;       ///< Transfer FIFO interrupt callback function pointer
    func_ptr_t pfnReadFifoOverrunIrqCb; ///< Receive FIFO overrun interrupt callback function pointer
    
}stc_icc_irq_cb_t, stc_icc_intern_data_t;

/**
 ******************************************************************************
 ** \brief ICC status types
 ******************************************************************************/
typedef enum en_icc_status
{
    IccTxEmpty              = 0u,   ///< Transmit buffer empty
    IccRxFull               = 1u,   ///< Receive buffer full
    IccRxAction             = 2u,   ///< Receive is in action
    IccTxAction             = 3u,   ///< Transmit is in action
    IccCardDetect           = 4u,   ///< Card insertion is detected
    IccCardEvent            = 5u,   ///< Card event is detected
    IccReceiveOverflow      = 6u,   ///< Receive buffer overflow
    IccIdleTimerRun         = 7u,   ///< Idle timer running
    IccReadFifoOverrun      = 8u,   ///< Read FIFO overflow
    IccReadFifoFull         = 9u,   ///< Read FIFO full
    IccWriteFifoEmpty       = 10u,  ///< Write FIFO empty
    IccRxStartErr           = 11u,  ///< Receive start bit error
    IccTxResend             = 12u,  ///< Trasmit detect resend
    IccRxResend             = 13u,  ///< Receive detect resend
  
}en_icc_status_t;

/**
 ******************************************************************************
 ** \brief ICC FIFO configuration.
 ******************************************************************************/
typedef struct stc_icc_fifo_config
{
    uint8_t     TxFifoLevel;                ///< Transfer data FIFO level
    uint8_t     RxFifoLevel;                ///< Receive data FIFO level
} stc_icc_fifo_config_t;

/**
 ******************************************************************************
 ** \brief ICC configuration structure
 ******************************************************************************/
typedef struct stc_icc_config
{
    uint16_t                u16ClkDiv;          ///< Clock division of card clock. (Card clock = PCLK/u16ClkDiv)
    uint16_t                u16BaudRate;        ///< Baud rate value, which is F/D. (1 ETU = (F/D) * (1/CardClock[Hz]))
    en_icc_tx_data_t        enTxDataFormat;     ///< Transmit data format
    en_icc_rx_data_t        enRxDataFormat;     ///< Receive data format
    boolean_t               bUseGuardTimer;     ///< TRUE: enable guard timer, FALSE: disable guard timer.
    uint8_t                 u8GuardTime;        ///< Guard time
    en_icc_idletmrclk_t     enIdleTimerSrcClk;  ///< Source clock of idle timer   
    uint16_t                u16IdleTimerCnt;    ///< Idle timer count
    en_icc_data_dir_t       enBitDirection;     ///< ICC data direction
    boolean_t               bInvertData;        ///< Inversion of the output data
    boolean_t               bDataResend;        ///< TRUE: data resend enabled, FALSE: data resend disabled
    boolean_t               bClkPinMode;        ///< TRUE: ICx_CLK pin controlled by software, FALSE: ICx_CLK pin controlled by hardware
    boolean_t               bDataPinMode;       ///< TRUE: ICx_DATA pin controlled by software, FALSE: ICx_DATA pin controlled by hardware
    boolean_t               bVpenPinOutputEn;   ///< TRUE: enable ICx_VPEN output, FALSE: dsiable ICx_VPEN output
    boolean_t               bVccPinOutputEn;    ///< TRUE: enable ICx_VCC output, FALSE: dsiable ICx_VCC output
    boolean_t               bRstPinOutputEn;    ///< TRUE: enable ICx_RST output, FALSE: dsiable ICx_RST output
    boolean_t               bDataPinOutputEn;   ///< TRUE: enable ICx_DATA output, FALSE: dsiable ICx_DATA output
    boolean_t               bClkPinOutputEn;    ///< TRUE: enable ICx_CLK output, FALSE: dsiable ICx_CLK output
    boolean_t               bVpenPinLevel;      ///< ICx_VPEN Pin level (Only valid when output is enabled)
    boolean_t               bVccPinLevel;       ///< ICx_VCC Pin level (Only valid when output is enabled)
    boolean_t               bRstPinLevel;       ///< ICx_RST Pin level (Only valid when output is enabled)
    boolean_t               bDataPinLevel;      ///< ICx_DATA Pin level (Only valid when output is enabled and Data pin mode is set to software control)
    boolean_t               bClkPinLevel;       ///< ICx_CLK Pin level (Only valid when output is enabled and Clock pin mode is set to software control)
    stc_icc_fifo_config_t   *pstcFifoConfig;    ///< Pointer to FIFO configuration structure, FIFO is not used when set to NULL
    
#if (PDL_INTERRUPT_ENABLE_ICC0 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_ICC1 == PDL_ON)
    stc_icc_irq_en_t        *pstcIrqEn;         ///< Pointer to ICC interrupt enable structure, if set to NULL, no interrupt enabled.
    stc_icc_irq_cb_t        *pstcIrqCb;         ///< Pointer to ICC interrupt callback functions structurei, f set to NULL, no interrupt callback initialized.
    boolean_t               bTouchNvic;         ///< TRUE: enable NVIC, FALSE: don't enable NVIC
#endif

} stc_icc_config_t;

/******************************************************************************/
/* Local type definitions ('typedef')                                         */
/******************************************************************************/
/// Enumeration to define an index for each enabled Icc instance
typedef enum en_icc_instance_index
{  
#if (PDL_PERIPHERAL_ENABLE_ICC0 == PDL_ON)  
    IccInstanceIndexIcc0,
#endif
#if (PDL_PERIPHERAL_ENABLE_ICC1 == PDL_ON)  
    IccInstanceIndexIcc1,
#endif
    IccInstanceIndexMax,
    IccInstanceIndexUnknown = 0xFFu,
} en_icc_instance_index_t;

/// ICC instance data type
typedef struct stc_icc_instance_data
{
    volatile stc_iccn_t*   pstcInstance;     ///< pointer to registers of an instance
    stc_icc_intern_data_t stcInternData;     ///< module internal data of instance
} stc_icc_instance_data_t;

/******************************************************************************/
/* Global variable definitions ('extern')                                     */
/******************************************************************************/
/// Look-up table for all enabled Icc instances and their internal data
extern stc_icc_instance_data_t m_astcIccInstanceDataLut[ICC_INSTANCE_COUNT];

/******************************************************************************/
/* Global function prototypes (definition in C source)                        */
/******************************************************************************/
#if (PDL_INTERRUPT_ENABLE_ICC0 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_ICC1 == PDL_ON)
// Interrupt
void Icc_IrqHandler(volatile stc_iccn_t*   pstcIcc, 
                    stc_icc_intern_data_t* pstcIccInternData);
en_result_t Icc_EnableIrq(volatile stc_iccn_t* pstcIcc, 
                             en_icc_irq_sel_t enIrqSel);
en_result_t Icc_DisableIrq(volatile stc_iccn_t* pstcIcc, 
                             en_icc_irq_sel_t enIrqSel);
#endif

// Init/De-Init
en_result_t Icc_Init(volatile stc_iccn_t*  pstcIcc,
                     stc_icc_config_t* pstcConfig);
en_result_t Icc_DeInit(volatile stc_iccn_t* pstcIcc, 
                         boolean_t bTouchNvic);

// Function enable/disable
en_result_t Icc_Enable(volatile stc_iccn_t* pstcIcc);
en_result_t Icc_Disable(volatile stc_iccn_t* pstcIcc);

// Baud rate
en_result_t Icc_SetBaudRate(volatile stc_iccn_t* pstcIcc,
                            uint16_t u16BaudRate);

// Status read
boolean_t Icc_GetStatus(volatile stc_iccn_t* pstcIcc, 
                        en_icc_status_t enStatus);
boolean_t Icc_GetIrqStatus(volatile stc_iccn_t* pstcIcc, 
                           en_icc_irq_sel_t enIrqSel);

// Data read/write
en_result_t Icc_SendData(volatile stc_iccn_t* pstcIcc, uint16_t Data);
en_result_t Icc_SendFifoData(volatile stc_iccn_t* pstcIcc, uint16_t Data);
uint16_t Icc_ReceiveData(volatile stc_iccn_t* pstcIcc);
uint16_t Icc_ReceiveFifoData(volatile stc_iccn_t* pstcIcc);

// FIFO 
en_result_t Icc_ClearWriteFifo (volatile stc_iccn_t* pstcIcc);
en_result_t Icc_ClearReadFifo (volatile stc_iccn_t* pstcIcc);
en_result_t Icc_SetWriteFifoLevel(volatile stc_iccn_t* pstcIcc, uint8_t u8Level);
en_result_t Icc_SetReadFifoLevel(volatile stc_iccn_t* pstcIcc, uint8_t u8Level);
uint8_t Icc_GetWriteFifoCurCnt(volatile stc_iccn_t* pstcIcc);
uint8_t Icc_GetReadFifoCurCnt(volatile stc_iccn_t* pstcIcc);

// IO
en_result_t Icc_SetVpenPinLevel (volatile stc_iccn_t* pstcIcc, 
                                 boolean_t bLevel);
en_result_t Icc_SetVccPinLevel (volatile stc_iccn_t* pstcIcc, 
                                boolean_t bLevel);
en_result_t Icc_SetRstPinLevel (volatile stc_iccn_t* pstcIcc, 
                                boolean_t bLevel);
en_result_t Icc_SetDataPinLevel (volatile stc_iccn_t* pstcIcc, 
                                 boolean_t bLevel);
en_result_t Icc_SetClkPinLevel (volatile stc_iccn_t* pstcIcc, 
                                boolean_t bLevel);

// Idle timer
en_result_t Icc_SetIdleTimerCnt(volatile stc_iccn_t* pstcIcc, uint16_t u16Cnt);
en_result_t Icc_StartIdleTimer(volatile stc_iccn_t* pstcIcc);
en_result_t Icc_StopIdleTimer(volatile stc_iccn_t* pstcIcc);

//@} // IccGroup

#ifdef __cplusplus
}
#endif

#endif /* #if (defined(PDL_PERIPHERAL_ICC_ACTIVE)) */

#endif /* __ICC_H__ */
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
