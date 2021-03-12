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
/** \file mft_frt.h
 **
 ** Headerfile for MFT's Free Run Timer functions
 **
 ** History:
 **   - 2014-12-21  0.1  EZh  First version.
 **   - 2014-01-16  0.2  EZh  Add comments about how to use FRT with APIs
 **
 *****************************************************************************/

#ifndef __MFT_FRT_H__
#define __MFT_FRT_H__

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "mcu.h"
#include "pdl_user.h"

#if (defined(PDL_PERIPHERAL_MFT_FRT_ACTIVE))

/**
 ******************************************************************************
 ** \defgroup FrtGroup Free-Run Timer (FRT)
 **
 ** Provided functions of FRT module:
 **
 ** - Mft_Frt_Init()
 ** - Mft_Frt_DeInit()  
 ** - Mft_Frt_SetMaskZeroTimes()
 ** - Mft_Frt_GetCurMaskZeroTimes()
 ** - Mft_Frt_SetMaskPeakTimes()
 ** - Mft_Frt_GetCurMaskPeakTimes()
 ** - Mft_Frt_Start()
 ** - Mft_Frt_Stop()
 ** - Mft_Frt_EnableIrq()
 ** - Mft_Frt_DisableIrq()
 ** - Mft_Frt_GetIrqFlag()
 ** - Mft_Frt_ClrIrqFlag()
 ** - Mft_Frt_SetCountCycle()
 ** - Mft_Frt_SetCountVal()
 ** - Mft_Frt_GetCurCount()
 **
 ** <b>How to use FRT module?</b>
 **
 ** Mft_Frt_Init() must be used for configuration of a Free-Run timer (FRT)
 ** channel with a structure of the type #stc_mft_frt_config_t.
 **
 ** Mft_Frt_SetMaskZeroTimes() is used to set the mask times of Zero match
 ** interrupt. Mft_Frt_GetCurMaskZeroTimes() is used to get the current mask 
 ** times of peak match interrupt.
 **
 ** Mft_Frt_SetMaskPeakTimes() is used to set the mask times of peak match
 ** interrupt. Mft_Frt_GetCurMaskPeakTimes() is used to get the current mask 
 ** times of peak match interrupt.
 **
 ** The interrupt can be configured with Mft_Frt_Init() first, after initialization, 
 ** The FRT interrupt can be enabled by the function Mft_Frt_EnableIrq() and 
 ** the FRT interrupt can be disabled by the function Mft_Frt_DisableIrq().  
 **
 ** With Mft_Frt_SetCountCycle() the FRT cycle is set to the value
 ** given in the parameter Mft_Frt_SetCountCycle#u16Cycle. And the initial
 ** count value can be modified by Mft_Frt_SetCountVal().
 **
 ** After above setting, calling Mft_Frt_Start() will start FRT.
 **
 ** With Mft_Frt_GetCurCount() the current FRT count can be read when FRT is
 ** running.
 **
 ** With interrupt mode, when the interrupt occurs, the interrupt flag will
 ** be cleared and run into user interrupt callback function.
 **
 ** With polling mode, user can use Mft_Frt_GetIrqFlag() to check if the
 ** interrupt occurs, and clear the interrupt flag by Mft_Frt_ClrIrqFlag().
 **
 ** When stopping the FRT, use Mft_Frt_Stop() to disable FRT and
 ** Mft_Frt_DisableIrq() to disable FRT interrupt.
 **
 ** Mft_Frt_SetSimultaneousStart() is used to start multi-channels FRT in the 
 ** same time.
 **
 ******************************************************************************/
//@{
/******************************************************************************
 * Global definitions
 ******************************************************************************/
#define stc_mftn_frt_t FM_MFT_FRT_TypeDef
#define MFT0_FRT       (*((volatile stc_mftn_frt_t *) FM_MFT0_FRT_BASE))
#define MFT1_FRT       (*((volatile stc_mftn_frt_t *) FM_MFT1_FRT_BASE))
#define MFT2_FRT       (*((volatile stc_mftn_frt_t *) FM_MFT2_FRT_BASE))

#define FRT_INSTANCE_COUNT (PDL_PERIPHERAL_ENABLE_MFT0_FRT == PDL_ON ? 1u : 0u) + \
                           (PDL_PERIPHERAL_ENABLE_MFT1_FRT == PDL_ON ? 1u : 0u) + \
                           (PDL_PERIPHERAL_ENABLE_MFT2_FRT == PDL_ON ? 1u : 0u)  

/**
 ******************************************************************************
 ** \brief MFT Frt channel definition
 ******************************************************************************/
#define  MFT_FRT_CH0             0u
#define  MFT_FRT_CH1             1u
#define  MFT_FRT_CH2             2u

#define  MFT_FRT_MAX_CH          3u
                                                          
/**
 ******************************************************************************
 ** \brief Enumeration to define an index for each FRT instance
 ******************************************************************************/
typedef enum en_frt_instance_index
{
#if (PDL_PERIPHERAL_ENABLE_MFT0_FRT == PDL_ON)  
    FrtInstanceIndexFrt0,      ///< Instance index of FRT0
#endif    
#if (PDL_PERIPHERAL_ENABLE_MFT1_FRT == PDL_ON) 
    FrtInstanceIndexFrt1,      ///< Instance index of FRT1
#endif
#if (PDL_PERIPHERAL_ENABLE_MFT2_FRT == PDL_ON) 
    FrtInstanceIndexFrt2,      ///< Instance index of FRT2
#endif
} en_frt_instance_index_t;

/**
 ******************************************************************************
 ** \brief Mft Clock Setting
 ******************************************************************************/
typedef enum en_mft_frt_clock
{
    FrtPclkDiv1   = 0u,    ///< FRT clock: PCLK
    FrtPclkDiv2   = 1u,    ///< FRT clock: PCLK/2
    FrtPclkDiv4   = 2u,    ///< FRT clock: PCLK/4
    FrtPclkDiv8   = 3u,    ///< FRT clock: PCLK/8
    FrtPclkDiv16  = 4u,    ///< FRT clock: PCLK/16
    FrtPclkDiv32  = 5u,    ///< FRT clock: PCLK/32
    FrtPclkDiv64  = 6u,    ///< FRT clock: PCLK/64
    FrtPclkDiv128 = 7u,    ///< FRT clock: PCLK/128
    FrtPclkDiv256 = 8u,    ///< FRT clock: PCLK/256
#if (PDL_MCU_CORE == PDL_FM0P_CORE) || (PDL_MCU_CORE == PDL_FM4_CORE)      
    FrtPclkDiv512 =  9u,    ///< FRT clock: PCLK/512
    FrtPclkDiv1024 = 10u,  ///< FRT clock: PCLK/1024
#endif    
} en_mft_frt_clock_t;

/**
 ******************************************************************************
 ** \brief FRT count mode
 ******************************************************************************/
typedef enum en_mft_frt_mode
{
    FrtUpCount     = 0u,    ///< FRT up-count mode
    FrtUpDownCount = 1u     ///< FRT up-/down-count mode
} en_mft_frt_mode_t;

/**
 ******************************************************************************
 ** \brief Enumeration of FRT interrupt selection
 ******************************************************************************/
typedef struct stc_frt_int_sel
{
    boolean_t bFrtZeroMatchIrq;       ///<  zero match interrupt selection
    boolean_t bFrtPeakMatchIrq;       ///<  peak match interrupt selection

}stc_frt_irq_sel_t, stc_frt_irq_en_t;

/**
 ******************************************************************************
 ** \brief Enumeration of FRT interrupt index
 ******************************************************************************/
typedef enum   en_mft_frt_int
{
    enFrtZeroMatchIrq = 0u,           ///<  zero match interrupt index
    enFrtPeakMatchIrq = 1u,           ///<  peak match interrupt index

}en_mft_frt_irq_t;

/**
 ******************************************************************************
 ** \brief FRT interrupt callback function
 ******************************************************************************/
typedef struct stc_frt_int_cb
{
    func_ptr_t  pfnFrtZeroIrqCb; ///< match zero interrupt callback function
    func_ptr_t  pfnFrtPeakIrqCb; ///< match peak interrupt callback function

}stc_frt_irq_cb_t;

/**
 ******************************************************************************
 ** \brief FRT configure
 ******************************************************************************/
typedef struct stc_mft_frt_config
{
    en_mft_frt_clock_t  enFrtClockDiv;  ///< FRT clock divide
    en_mft_frt_mode_t   enFrtMode;      ///< FRT count mode
    boolean_t           bEnBuffer;      ///< enable buffer
    boolean_t           bEnExtClock;    ///< enable external clock
#if (PDL_MCU_CORE == PDL_FM3_CORE)    
    boolean_t           bTriggerAdc0;    ///< TRUE: Output AD conversion start signal to ADC uint 0 upon FRT zreo match event
                                         ///< FALSE: Don't output AD conversion start signal to ADC uint 0 upon FRT zreo match event
    boolean_t           bTriggerAdc1;    ///< TRUE: Output AD conversion start signal to ADC uint 1 upon FRT zreo match event
                                         ///< FALSE: Don't output AD conversion start signal to ADC uint 1 upon FRT zreo match event
    boolean_t           bTriggerAdc2;    ///< TRUE: Output AD conversion start signal to ADC uint 2 upon FRT zreo match event
                                         ///< FALSE: Don't output AD conversion start signal to ADC uint 2 upon FRT zreo match event
#endif
    stc_frt_irq_en_t*   pstcIrqEn;      ///< Pointer to FRT interrupt enable structure
    stc_frt_irq_cb_t*   pstcIrqCb;      ///< Pointer to FRT interrupt callback functions structure
    boolean_t           bTouchNvic;     ///< TRUE: Enable NVIC, FALSE: don't enable NVIC
    
}stc_mft_frt_config_t;

/**
 ******************************************************************************
 ** \brief Enumeration of FRT channels with offset mode
 ******************************************************************************/
typedef enum en_frt_offset_ch
{
    FrtOffsetCh1 = 0u, ///<  FRT ch.1 with offset mode
    FrtOffsetCh2 = 1u, ///<  FRT ch.2 with offset mode

}en_frt_offset_ch_t;    
    
/**
 ******************************************************************************
 ** \brief FRT instance internal data, storing internal
 **        information for each enabled FRT instance.
 ******************************************************************************/
typedef struct stc_mft_frt_intern_data
{
    func_ptr_t  pfnFrt0PeakIrqCb;  ///< Callback function pointer of FRT0 peak detection interrupt
    func_ptr_t  pfnFrt0ZeroIrqCb;  ///< Callback function pointer of FRT0 zero detection interrupt
    func_ptr_t  pfnFrt1PeakIrqCb;  ///< Callback function pointer of FRT1 peak detection interrupt
    func_ptr_t  pfnFrt1ZeroIrqCb;  ///< Callback function pointer of FRT1 zero detection interrupt
    func_ptr_t  pfnFrt2PeakIrqCb;  ///< Callback function pointer of FRT2 peak detection interrupt
    func_ptr_t  pfnFrt2ZeroIrqCb;  ///< Callback function pointer of FRT2 zero detection interrupt    

}stc_mft_frt_intern_data_t;

/**
 ******************************************************************************
 ** \brief FRT instance data type
 ******************************************************************************/
typedef struct stc_mft_frt_instance_data
{
    volatile stc_mftn_frt_t*  pstcInstance;  ///< pointer to registers of an instance
    stc_mft_frt_intern_data_t stcInternData; ///< module internal data of instance
} stc_mft_frt_instance_data_t;

/******************************************************************************/
/* Global variable declarations ('extern', definition in C source)            */
/******************************************************************************/
/// Look-up table for all enabled FRT of MFT  instances and their internal data
extern stc_mft_frt_instance_data_t m_astcMftFrtInstanceDataLut[FRT_INSTANCE_COUNT];

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
/******************************************************************************/
/* Global function prototypes ('extern', definition in C source)              */
/******************************************************************************/
/* Init */
en_result_t Mft_Frt_Init(volatile stc_mftn_frt_t*pstcMft,
                         uint8_t u8Ch, stc_mft_frt_config_t*  pstcFrtConfig);
en_result_t Mft_Frt_DeInit(volatile stc_mftn_frt_t*pstcMft, uint8_t u8Ch, 
                           boolean_t bTouchNvic);
#if (PDL_MCU_CORE == PDL_FM0P_CORE) || (PDL_MCU_CORE == PDL_FM4_CORE)    
/* Mask zero/peak write/read */
en_result_t Mft_Frt_SetMaskZeroTimes(volatile stc_mftn_frt_t*pstcMft, 
                                     uint8_t u8Ch, 
                                     uint8_t u8Times);
uint8_t Mft_Frt_GetCurMaskZeroTimes(volatile stc_mftn_frt_t*pstcMft, uint8_t u8Ch);
en_result_t Mft_Frt_SetMaskPeakTimes(volatile stc_mftn_frt_t*pstcMft, 
                                     uint8_t u8Ch, 
                                     uint8_t u8Times);
uint8_t Mft_Frt_GetCurMaskPeakTimes(volatile stc_mftn_frt_t*pstcMft, uint8_t u8Ch);
#endif

#if (PDL_MCU_CORE == PDL_FM4_CORE) 
#if (PDL_MCU_TYPE >= PDL_FM4_TYPE3)
en_result_t  Mft_Frt_SetOffsetMode(volatile stc_mftn_frt_t*pstcMft, 
                                   uint8_t u8Ch, 
                                   boolean_t bOffsetMode);
#endif
#endif
/* Func/Int enable/disable */
en_result_t Mft_Frt_Start(volatile stc_mftn_frt_t*pstcMft, uint8_t u8Ch);
en_result_t Mft_Frt_Stop(volatile stc_mftn_frt_t*pstcMft, uint8_t u8Ch);
#if (PDL_INTERRUPT_ENABLE_MFT0_FRT == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_MFT1_FRT == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_MFT2_FRT == PDL_ON)
en_result_t Mft_Frt_EnableIrq(volatile stc_mftn_frt_t*pstcMft,
                              uint8_t u8Ch,
                              stc_frt_irq_sel_t* pstcIrqSel);
en_result_t Mft_Frt_DisableIrq(volatile stc_mftn_frt_t*pstcMft,
                               uint8_t u8Ch,
                               stc_frt_irq_sel_t* pstcIrqSel);
#endif

/* Status get/clear */
en_irq_flag_t Mft_Frt_GetIrqFlag(volatile stc_mftn_frt_t*pstcMft,
                                 uint8_t u8Ch,
                                 en_mft_frt_irq_t enIrqType);
en_result_t Mft_Frt_ClrIrqFlag(volatile stc_mftn_frt_t*pstcMft,
                               uint8_t u8Ch,
                               en_mft_frt_irq_t enIrqType);

/* Count write/read */
en_result_t Mft_Frt_SetCountCycle(volatile stc_mftn_frt_t*pstcMft, uint8_t u8Ch, uint16_t u16Cycle);
en_result_t Mft_Frt_SetCountVal(volatile stc_mftn_frt_t*pstcMft, uint8_t u8Ch,uint16_t u16Count);
uint16_t Mft_Frt_GetCurCount(volatile stc_mftn_frt_t*pstcMft, uint8_t u8Ch);

#if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE)
/* Simultaneous Start */
en_result_t Mft_Frt_SetSimultaneousStart(uint16_t u16Stop, uint16_t u16Clr);
#endif

/* IRQ handler */
void Mft_Frt_IrqHandler(volatile stc_mftn_frt_t*pstcMft, stc_mft_frt_intern_data_t* pstcMftFrtInternData) ;

#ifdef __cplusplus
}
#endif

//@}

#endif // #if (defined(PDL_PERIPHERAL_MFT_FRT_ACTIVE))

#endif /* __MFT_H__ */
