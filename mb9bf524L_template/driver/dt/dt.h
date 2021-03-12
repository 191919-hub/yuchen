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
/** \file dt.h
 **
 ** Headerfile for DT functions
 **  
 **
 ** History:
 **   - 2014-12-01  0.1  EZh   First version for FM universal PDL.
 ******************************************************************************/

#ifndef __DT_H__
#define __DT_H__

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "mcu.h"
#include "pdl_user.h"

#if (defined(PDL_PERIPHERAL_DT_ACTIVE))

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/**
 ******************************************************************************
 ** \defgroup DtGroup Dual Timer (DT)
 **
 ** Provided functions of DT module:
 ** 
 ** - Dt_Init()
 ** - Dt_DeInit() 
 ** - Dt_EnableCount()
 ** - Dt_DisableCount()
 ** - Dt_EnableIrq()
 ** - Dt_DisableIrq()
 ** - Dt_GetIrqFlag()
 ** - Dt_GetMaskIrqFlag()
 ** - Dt_ClrIrqFlag()
 ** - Dt_WriteLoadVal()
 ** - Dt_WriteBgLoadVal()
 ** - Dt_ReadCurCntVal()
 **
 ** Dt_Init() must be used for configuration of a Dual Timer (DT) channel
 ** a structure with the type #stc_dt_channel_config_t.
 **
 ** The interrupts can be configured in Dt_Init(), after initialization, 
 ** the interrupt can be enabled or disabled with Dt_EnableIrq() and 
 ** Dt_DisableIrq().
 **
 ** A DT Channel can be enabled by the function Dt_EnableCount().
 ** Depending on the used mode, it is started in:
 ** - Free-run mode
 ** - Periodic mode
 ** - One-shot mode
 **
 ** With Dt_WriteLoadVal() the recent DT counter is set to the value
 ** given in the parameter Dt_WriteLoadVal#u32LoadVal.
 ** This works in each of the three operation modes.
 **
 ** With Dt_WriteBgLoadVal() the background reload value can be set,
 ** which is then set to the load value after the DT counter reaches the
 ** next 0.
 **
 ** With Dt_ReadCurCntVal() the current DT counter can be read.
 **
 ** Before deinitialization of the DT by Dt_DeInit(), it is recommended
 ** to disable all channels via Dt_DisableCount() and Dt_DisableIrq() before,
 ** to avoid a possible, unwanted interrupt.
 **
 ******************************************************************************/
//@{

/******************************************************************************
 * Global type definitions
 ******************************************************************************/
/**
 ******************************************************************************
 ** \brief Redefinition of DT register structure 
 ******************************************************************************/
typedef FM_DTIM_TypeDef stc_dtn_t;

/**
 ******************************************************************************
 ** \brief Dt mode
 ** 
 ** To select between Free-run, Periodic, and One-Shot mode
 ******************************************************************************/
typedef enum en_dt_mode
{
    DtFreeRun      = 0u,  ///< Free-running mode
    DtPeriodic     = 1u,  ///< Periodic mode
    DtOneShot      = 2u   ///< One-shot mode
} en_dt_mode_t;

/**
 ******************************************************************************
 ** \brief Dt Prescaler
 ** 
 ** To select clock divider
 ******************************************************************************/
typedef enum en_dt_prescaler
{
    DtPrescalerDiv1     = 0u,  ///< Prescaler divisor 1
    DtPrescalerDiv16    = 1u,  ///< Prescaler divisor 16
    DtPrescalerDiv256   = 2u   ///< Prescaler divisor 256
} en_dt_prescaler_t;

/**
 ******************************************************************************
 ** \brief Dt Counter Size
 ** 
 ** To select the size of the counter
 ******************************************************************************/
typedef enum en_dt_countersize
{
    DtCounterSize16     = 0u,  ///< 16 Bit counter size
    DtCounterSize32     = 1u   ///< 32 Bit counter size
} en_dt_countersize_t;

/**
 ******************************************************************************
 ** \brief Dt channel number
 ******************************************************************************/
typedef enum en_dt_channel
{
    DtChannel0     = 0u,  ///< channel 0
    DtChannel1     = 1u,  ///< channel 1
    DtMaxChannels  = 2u   ///< Number of channels
} en_dt_channel_t;

/**
 *****************************************************************************
 ** \brief Dt channel configuration
 ** 
 ** The DT configuration is done on a per channel basis
 *****************************************************************************/
typedef struct stc_dt_channel_config
{
    uint8_t u8Mode;           ///< Mode, see description of #en_dt_mode_t
    uint8_t u8PrescalerDiv;   ///< Prescaler, see description of #en_dt_prescaler_t
    uint8_t u8CounterSize;    ///< Counter size, see description of #en_dt_countersize_t
#if (PDL_INTERRUPT_ENABLE_DT0 == PDL_ON)    
    boolean_t  bIrqEnable;      ///< TRUE: enable interrupt, FALSE: disable interrupt
    func_ptr_t pfnIrqCallback;  ///< Pointer to interrupt callback function
    boolean_t  bTouchNvic;      ///< TRUE: enable MVIC, FALSE: don't enable NVIC
#endif    
} stc_dt_channel_config_t;

/******************************************************************************
 * Global definitions
 ******************************************************************************/
#define DT0     (*((volatile stc_dtn_t *) FM_DTIM_BASE))

/* for TimerPre of TimerXControl */
#define DT_PRE_TIMER_DIV_1      (0x00)
#define DT_PRE_TIMER_DIV_16     (0x01)
#define DT_PRE_TIMER_DIV_256    (0x02)

/******************************************************************************/
/* Local type definitions ('typedef')                                         */
/******************************************************************************/

/// Enumeration to define an index for each enabled Dual timer instance
typedef enum en_dt_instance_index
{
#if (PDL_PERIPHERAL_ENABLE_DT0 == PDL_ON)
    DtInstanceIndexDt0 = 0u,
#endif
    DtInstanceIndexMax
} en_dt_instance_index_t;

/// Datatype for holding internal data needed for DT
typedef struct stc_dt_intern_data
{
    /// Callback for interrupts of DT channel 0
    func_ptr_t  pfnIrqCallbackIntern[DtMaxChannels] ;
} stc_dt_intern_data_t ;

/// DT instance data type
typedef struct stc_dt_instance_data
{
    volatile stc_dtn_t*  pstcInstance;  ///< pointer to registers of an instance
    stc_dt_intern_data_t stcInternData; ///< module internal data of instance
} stc_dt_instance_data_t;

/******************************************************************************/
/* Global variable definitions ('extern')                                     */
/******************************************************************************/

/******************************************************************************/
/* Global function prototypes (definition in C source)                        */
/******************************************************************************/
#if (PDL_INTERRUPT_ENABLE_DT0 == PDL_ON)
/* External function  */
void DtIrqHandler(uint8_t u8Ch);
en_result_t Dt_EnableIrq (uint8_t u8Ch);
en_result_t Dt_DisableIrq(uint8_t u8Ch);
#endif

/* Init/Deinit */
en_result_t Dt_Init(stc_dt_channel_config_t* pstcConfig,
                    uint8_t                  u8Ch);
en_result_t Dt_DeInit(uint8_t u8Ch, boolean_t bTouchNvic);

/* Function Enable/Disable */
en_result_t Dt_EnableCount(uint8_t u8Ch);
en_result_t Dt_DisableCount(uint8_t u8Ch);

/* Get/Clr irq flag */
boolean_t Dt_GetIrqFlag(uint8_t u8Ch);
boolean_t Dt_GetMaskIrqFlag(uint8_t u8Ch);
en_result_t Dt_ClrIrqFlag(uint8_t u8Ch);

/* Write/Read count value */
en_result_t Dt_WriteLoadVal(uint32_t u32LoadVal,
                            uint8_t  u8Ch);
en_result_t Dt_WriteBgLoadVal(uint32_t u32BgLoadVal,
                              uint8_t  u8Ch);
uint32_t Dt_ReadCurCntVal(uint8_t u8Ch);

//@} // DtGroup

#ifdef __cplusplus
}
#endif

#endif /* #if (defined(PDL_PERIPHERAL_DT_ACTIVE)) */

#endif /* __DT_H__ */
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
