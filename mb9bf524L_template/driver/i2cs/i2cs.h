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
/** \file i2cs.h
 **
 ** Headerfile for I2C Slave functions
 **  
 **
 ** History:
 **   - 2015-06-28  1.0  EZh   First version for FM uinversal PDL.
 **
 ******************************************************************************/

#ifndef __I2CS_H__
#define __I2CS_H__

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "mcu.h"
#include "pdl_user.h"

#if (defined(PDL_PERIPHERAL_I2CS_ACTIVE))

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/**
 ******************************************************************************
 ** \defgroup I2csGroup I2C Slave (I2CS))
 **
 ** Provided functions of I2CS module:
 ** 
 ** - I2cs_Init()
 ** - I2cs_Init()
 ** - I2cs_EnableIrq()
 ** - I2cs_DisableIrq()  
 ** - I2cs_SendData()
 ** - I2cs_ReceiveData()
 ** - I2cs_ConfigAck()
 ** - I2cs_GetAck()
 ** - I2cs_GetStatus()
 ** - I2cs_ClrStatus()
 ** - I2cs_GetDataDir()     
 **
 ** I2cs_Init() is used to initialize an I2CS instance with
 ** parameter pstcConfig of type #stc_i2cs_config_t.
 **
 ** I2cs_DeInit() is used to reset all I2CS related register.
 ** So this function is used after initialization by I2cs_Init().
 **
 ** I2cs_EnableIrq() enables I2C interrupt sources selected by enumeration type
 ** #en_i2cs_irq_sel_t and I2cs_DisableIrq() disables the I2C interrupt 
 ** sources selected by enumeration type #en_i2cs_irq_sel_t.
 **
 ** I2cs_SendData() writes a byte data into I2CS transfer buffer and  
 ** I2cs_ReceiveData() reads a byte data from I2CS receive buffer. 
 **
 ** I2cs_ConfigAck() configures ACK signal when receive a data. 
 ** I2cs_GetAck() gets the ACK signal status after receiving a ACK.  
 **
 ** I2cs_GetStatus() gets the status selected by I2cs_GetStatus#enStatus
 ** and I2cs_ClrStatus() clears the I2C status selected, some status can
 ** only be cleared by hardware automatically.   
 **
 ** I2cs_GetDataDir() gets the data direction of I2CS in the slave mode.  
 **
 ******************************************************************************/
//@{

/******************************************************************************
 * Global type definitions
 ******************************************************************************/
#define stc_i2csn_t FM_I2CS_TypeDef 

#define I2CS0       (*((volatile stc_i2csn_t *) FM_I2CS0_BASE))
   
#define I2CS_INSTANCE_COUNT                                \
        (uint8_t)(PDL_PERIPHERAL_ENABLE_I2CS0 == PDL_ON) 
  
#define I2CS0_DATA_REG_ADDR   (uint32_t)(&FM_I2CS0->TDR)
    
/******************************************************************************
 * Global type definitions
 ******************************************************************************/

#define I2CS_IRQ_COND_RESTART_DETECTION        0u
#define I2CS_IRQ_COND_STOP_DETECTION           1u          
          
/**
 ******************************************************************************
 ** \brief I2CS ACK types
 ******************************************************************************/ 
typedef enum en_i2cs_ack
{
    I2csAck =  0u,   ///< I2CS normal ACK
    I2csNAck = 1u,   ///< I2CS NACK 

}en_i2cs_ack_t; 

/**
 ******************************************************************************
 ** \brief I2CS interrupt selection
 ******************************************************************************/
typedef enum en_i2cs_irq_sel
{
    I2csTxIrq          = 0u,          ///< I2CS TX interrupt
    I2csRxIrq          = 1u,          ///< I2CS RX interrupt
    I2csCondIrq        = 2u,          ///< I2CS condition detection interrupt
    I2csStatusIrq      = 3u,          ///< I2CS status interrupt
  
}en_i2cs_irq_sel_t;

/**
 ******************************************************************************
 ** \brief I2CS interrupt enable structure
 ******************************************************************************/
typedef struct stc_i2cs_irq_en
{
    boolean_t bTxIrq;         ///< I2CS TX interrupt
    boolean_t bRxIrq;         ///< I2CS RX interrupt
    boolean_t bCondIrq;       ///< I2CS condition interrupt
    boolean_t bStatusIrq;     ///< I2CS condition interrupt	
  
}stc_i2cs_irq_en_t;

/**
 ******************************************************************************
 ** \brief I2CS interrupt callback function
 ******************************************************************************/
typedef struct stc_i2cs_irq_cb
{
    func_ptr_t pfnTxIrqCb;              ///< I2CS TX interrupt callback function pointer
    func_ptr_t pfnRxIrqCb;              ///< I2CS RX interrupt callback function pointer
    func_ptr_arg1_t pfnCondIrqCb;       ///< I2CS Condition interrupt callback function pointer 
    func_ptr_t pfnStatusIrqCb;          ///< I2CS status interrupt callback function pointer	
    
}stc_i2cs_irq_cb_t;

/**
 ******************************************************************************
 ** \brief I2CS status types
 ******************************************************************************/
typedef enum en_i2cs_status
{
    I2csStatus              = 0u,   ///< I2CS status interrupt request flag
    I2csRxFull              = 1u,   ///< I2CS RX buffer full interrupt request flag
    I2csTxEmpty             = 2u,   ///< I2CS TX buffer empty interrupt request flag
    I2csFirstByteDetect     = 3u,   ///< I2CS First byte detection
    I2csReservedByteDetect  = 4u,   ///< I2CS reserved byte detection
	I2csActive              = 5u,   ///< I2CS active
	I2csRestartDetect       = 6u,   ///< I2CS re-start condition detection	
    I2csStopDetect          = 7u,   ///< I2CS stop condition detection
    I2csBusBusy             = 8u,   ///< I2CS Bus busy status
}en_i2cs_status_t;

/**
 ******************************************************************************
 ** \brief I2CS data direction
 ******************************************************************************/
typedef enum en_i2cs_data_dir
{
    I2csDataRx = 0u,  ///< Data from master to slave
    I2csDataTx = 1u,  ///< Data from slave to master
  
}en_i2cs_data_dir_t;

/**
 ******************************************************************************
 ** \brief I2CS configuration structure
 ******************************************************************************/
typedef struct stc_i2cs_config
{
    uint8_t   u8SlaveAddr;           ///< Slave address (This is effective on slave mode.
    uint8_t   u8SlaveMaskAddr;       ///< Slave Mask address (This is effective on slave mode.)
	boolean_t bEnableSlaveAddr;      ///< FLASE: disable slave address comparison, TRUE: enable salve address comparison
	boolean_t bEnableReservedAddr;   ///< FALSE: disable the detection of the reserved addresses (0000xxx or 1111xxx), TRUE: enable the detection of the reserved addresses (0000xxx or 1111xxx)
    boolean_t bWaitSelection;        ///< FALSE: generate interrupt after ACK, TRUE: generate interrupt before ACK
	boolean_t bEnableNoiseFilter;    ///< FALSE: disable noise filter, TRUE: enable noise filter
    uint8_t   u8SetupTime;           ///< Setup time between the I2C data line (SDA) and I2C clock line (SCL)  
	
#if (PDL_INTERRUPT_ENABLE_I2CS0 == PDL_ON)
    stc_i2cs_irq_en_t* pstcIrqEn;           ///< Pointer to I2CS interrupt enable structure, if set to NULL, no interrupt enabled.
    stc_i2cs_irq_cb_t* pstcIrqCb;           ///< Pointer to I2CS interrupt callback functions structurei, if set to NULL, no interrupt callback initialized.
    boolean_t bTouchNvic;                  ///< TRUE: enable NVIC, FALSE: don't enable NVIC
#endif    
} stc_i2cs_config_t;

/******************************************************************************/
/* Local type definitions ('typedef')                                         */
/******************************************************************************/
/// Enumeration to define an index for each enabled I2CS instance
typedef enum en_i2cs_instance_index
{  
#if (PDL_PERIPHERAL_ENABLE_I2CS0 == PDL_ON)  
    I2csInstanceIndexI2cs0,
#endif
    I2csInstanceIndexMax,    
    I2csInstanceIndexUnknown = 0xFFu,
    
} en_i2cs_instance_index_t;

/// I2CS module internal data, storing internal information for each enabled I2CS instance.
typedef struct stc_i2cs_intern_data
{
    func_ptr_t pfnTxIrqCb;              ///< I2CS TX interrupt callback function pointer
    func_ptr_t pfnRxIrqCb;              ///< I2CS RX interrupt callback function pointer
    func_ptr_arg1_t pfnCondIrqCb;       ///< I2CS Condition interrupt callback function pointer
    func_ptr_t pfnStatusIrqCb;          ///< I2CS status interrupt callback function pointer	

} stc_i2cs_intern_data_t;

/// I2CS instance data type
typedef struct stc_i2cs_instance_data
{
    volatile stc_i2csn_t*   pstcInstance;     ///< pointer to registers of an instance
    stc_i2cs_intern_data_t stcInternData;     ///< module internal data of instance
} stc_i2cs_instance_data_t;

/******************************************************************************/
/* Global variable definitions ('extern')                                     */
/******************************************************************************/

/// Look-up table for all enabled I2CS instances and their internal data
extern stc_i2cs_instance_data_t m_astcI2csInstanceDataLut[I2CS_INSTANCE_COUNT];
/******************************************************************************/
/* Global function prototypes (definition in C source)                        */
/******************************************************************************/
/* I2CS */
#if (PDL_INTERRUPT_ENABLE_I2CS0 == PDL_ON) 
// Interrupt
void I2csIrqHandlerTx(volatile stc_i2csn_t*   pstcI2cs, 
                      stc_i2cs_intern_data_t* pstcInternData);
void I2csIrqHandlerRx(volatile stc_i2csn_t*   pstcI2cs,  
                      stc_i2cs_intern_data_t* pstcInternData);
void I2csIrqHandlerStatus(volatile stc_i2csn_t*   pstcI2cs,   
                          stc_i2cs_intern_data_t* pstcInternData);
en_result_t I2cs_EnableIrq(volatile stc_i2csn_t*   pstcI2cs,   
                           en_i2cs_irq_sel_t enIrqSel);
en_result_t I2cs_DisableIrq(volatile stc_i2csn_t*   pstcI2cs,  
                            en_i2cs_irq_sel_t enIrqSel);
#endif
// Init/De-Init
en_result_t I2cs_Init(volatile stc_i2csn_t*   pstcI2cs,  
                      stc_i2cs_config_t* pstcConfig);
en_result_t I2cs_DeInit(volatile stc_i2csn_t*   pstcI2cs,   boolean_t bTouchNvic);

// Data read/write
en_result_t I2cs_SendData(volatile stc_i2csn_t*   pstcI2cs, uint8_t u8Data);
uint8_t I2cs_ReceiveData(volatile stc_i2csn_t*   pstcI2cs);

// ACK
en_result_t I2cs_ConfigAck(volatile stc_i2csn_t*   pstcI2cs, en_i2cs_ack_t enAck);
en_i2cs_ack_t I2cs_GetAck(volatile stc_i2csn_t*   pstcI2cs);

// Status read/clear
boolean_t I2cs_GetStatus(volatile stc_i2csn_t*   pstcI2cs, 
                         en_i2cs_status_t enStatus);
en_result_t I2cs_ClrStatus(volatile stc_i2csn_t*   pstcI2cs,
                           en_i2cs_status_t enStatus);

// Get Data direction
en_i2cs_data_dir_t I2cs_GetDataDir(volatile stc_i2csn_t*   pstcI2cs);


//@} // I2csGroup

#ifdef __cplusplus
}
#endif

#endif /* #if (defined(PDL_PERIPHERAL_I2CS_ACTIVE)) */

#endif /* __I2CS_H__ */
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
