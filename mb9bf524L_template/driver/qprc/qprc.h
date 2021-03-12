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
/** \file qprc.h
 **
 ** Headerfile for QPRC functions
 **
 ** History:
 **   - 2014-12-11  0.1  EZh  First version for FM universal PDL.
 **
 *****************************************************************************/
#ifndef __QPRC_H__
#define __QPRC_H__

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "mcu.h"
#include "pdl_user.h"

#if (defined(PDL_PERIPHERAL_QPRC_ACTIVE))

/**
 ******************************************************************************
 ** \defgroup QprcGroup Quadrature Position/Revolution Counter (QPRC)
 **
 ** Provided functions of QPRC module:
 ** 
 ** - Qprc_Init()
 ** - Qprc_StopPcCount() 
 ** - Qprc_RestartPcCount()
 ** - Qprc_SetPcCount()
 ** - Qprc_GetPcCount()
 ** - Qprc_SetRcCount()
 ** - Qprc_GetRcCount()
 ** - Qprc_SetPcCompareValue()
 ** - Qprc_GetPcCompareValue()
 ** - Qprc_SetPcRcCompareValue()
 ** - Qprc_GetPcRcCompareValue()
 ** - Qprc_ConfigPcMode()
 ** - Qprc_ConfigRcMode()
 ** - Qprc_EnableIrq()
 ** - Qprc_DisableIrq()
 ** - Qprc_GetIrqFlag()
 ** - Qprc_ClrIrqFlag()
 ** - Qprc_GetPcOfUfDir()
 ** - Qprc_GetPcDir()
 **
 **<b>How to use Position Count (PC) and Revolution Counter (RC)? </b>
 **
 ** Qprc_Init() must be used for configuration of a QPRC channel with a 
 ** structure of the type #stc_qprc_config_t.
 **
 ** The interrupts can be configured in Qprc_Init(), after initialization, 
 ** a QPRC interrupt can be enabled or disabled by the function Qprc_EnableIrq()
 ** and Qprc_DisableIrq().
 **
 ** With Qprc_SetPcCompareValue() the PC compare value is set to the value
 ** given in the parameter Qprc_SetPcCompareValue#u16PcValue. And PC compare 
 ** value is read by Qprc_GetPcCompareValue().
 **
 ** With Qprc_SetPcRcCompareValue() the PC and RC compare value is set to 
 ** the value given in the parameter Qprc_SetPcRcCompareValue#u16PcRcValue. 
 ** And PC and RC compare value is read by Qprc_GetPcRcCompareValue(). Whether
 ** PC and RC compare value compares to PC count and RC count depends on the 
 ** setting in paramter of the Qprc_Init().
 ** 
 ** The initial PC count value can be set by Qprc_SetPcCount(). And current
 ** PC count can be read by Qprc_GetPcCount()
 **
 ** The initial RC count value can be set by Qprc_SetRcCount(). And current
 ** RC count can be read by Qprc_GetRcCount()
 **
 ** The maximum PC count value can be set by Qprc_SetPcMaxValue(). And 
 ** the maximum PC count value can be read by Qprc_GetPcMaxValue(). If PC
 ** count exceeds this value, a PC overflow interrupt flag will be set.
 **
 ** After above setting, Qprc_ConfigPcMode() can start PC with following mode:
 ** - Up-down count mode(PC mode 1)
 ** - Phase different count mode (PC mode 2)
 ** - Directional count mode (PC mode 3)
 **
 ** Qprc_ConfigRcMode() can start RC with following mode:
 ** - ZIN trigger mode(RC mode 1)
 ** - PC underflow or overflow detection trigger mode (RC mode 2)
 ** - ZIN or PC underflow or overflow detection trigger mode(RC mode 3)
 **
 ** With interrupt mode, when the interrupt occurs, the interrupt flag will
 ** be cleared and run into user interrupt callback function.
 **
 ** With polling mode, user can use Qprc_GetIrqFlag() to check if the 
 ** interrupt occurs, and clear the interrupt flag by Qprc_ClrIrqFlag().
 **
 ** Qprc_GetPcOfUfDir() can get the PC direction after a PC overflow or  
 ** underflow occurs.
 **
 ** Qprc_GetPcDir() can get the current PC direction when PC is counting.
 **
 ** Qprc_StopPcCount() can stop PC when PC is counting. And Qprc_RestartPcCount()
 ** will restart PC when PC is in stop status.
 ** 
 ** When stopping the QPRC, disable PC by using Qprc_ConfigPcMode() to set PC 
 ** to PC mode0 and disable RC by using Qprc_ConfigRcMode() to set RC to RC 
 ** mode0. Use QPRC_DisableIrq() to disable QPRC interrupt.
 **
 ******************************************************************************/

//@{

/******************************************************************************
 * Global definitions
 ******************************************************************************/
#define stc_qprcn_t     FM_QPRC_TypeDef

#define QPRC0       (*((volatile stc_qprcn_t *) FM_QPRC0_BASE))
#define QPRC1       (*((volatile stc_qprcn_t *) FM_QPRC1_BASE))
#define QPRC2       (*((volatile stc_qprcn_t *) FM_QPRC2_BASE))
#define QPRC3       (*((volatile stc_qprcn_t *) FM_QPRC3_BASE))

#if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE)  
#define stc_qprc_nfn_t  FM_QPRC_NF_TypeDef   
   
#define QPRCNF0       (*((volatile stc_qprc_nfn_t *) FM_QPRC0_NF_BASE))
#define QPRCNF1       (*((volatile stc_qprc_nfn_t *) FM_QPRC1_NF_BASE))
#define QPRCNF2       (*((volatile stc_qprc_nfn_t *) FM_QPRC2_NF_BASE))
#define QPRCNF3       (*((volatile stc_qprc_nfn_t *) FM_QPRC3_NF_BASE))
#endif   

#define QPRC_INSTANCE_COUNT     (PDL_PERIPHERAL_ENABLE_QPRC0 == PDL_ON ? 1u : 0u) + \
                                (PDL_PERIPHERAL_ENABLE_QPRC1 == PDL_ON ? 1u : 0u) + \
                                (PDL_PERIPHERAL_ENABLE_QPRC2 == PDL_ON ? 1u : 0u) + \
                                (PDL_PERIPHERAL_ENABLE_QPRC3 == PDL_ON ? 1u : 0u) 																

#define QPRC_PC_OVERFLOW_INT       0u
#define QPRC_PC_UNDERFLOW_INT      1u
#define QPRC_PC_ZERO_INDEX_INT     2u
   
/**
 ******************************************************************************
 ** \brief Enumeration to define an index for each QPRC instance
 ******************************************************************************/
typedef enum en_qprc_instance_index
{
#if (PDL_PERIPHERAL_ENABLE_QPRC0 == PDL_ON)  
    QprcInstanceIndexQprc0 = 0u,  ///< Instance index of QPRC0
#endif
#if (PDL_PERIPHERAL_ENABLE_QPRC1 == PDL_ON) 
    QprcInstanceIndexQprc1,      ///< Instance index of QPRC1
#endif
#if (PDL_PERIPHERAL_ENABLE_QPRC2 == PDL_ON) 
    QprcInstanceIndexQprc2,      ///< Instance index of QPRC2
#endif
#if (PDL_PERIPHERAL_ENABLE_QPRC3 == PDL_ON) 
    QprcInstanceIndexQprc3,      ///< Instance index of QPRC2
#endif	
} en_qprc_instance_index_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define position counter mode
 ******************************************************************************/
typedef enum en_qprc_pcmode
{
    QprcPcMode0 = 0u,  ///< PC_Mode0: Disable position counter
    QprcPcMode1 = 1u,  ///< PC_Mode1: Increments with AIN active edge and decrements with BIN active edge
    QprcPcMode2 = 2u,  ///< PC_Mode2: Phase difference count mode: Counts up if AIN is leading BIN, down if BIN leading.
    QprcPcMode3 = 3u,  ///< PC_Mode3: Directional count mode: Counts up/down with BIN active edge and AIN level
} en_qprc_pcmode_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define revolution counter mode
 ******************************************************************************/
typedef enum en_qprc_rcmode
{
    QprcRcMode0 = 0u,  ///< RC_Mode0: Disable revolution counter
    QprcRcMode1 = 1u,  ///< RC_Mode1: Up/down count of revolution counter with ZIN active edge
    QprcRcMode2 = 2u,  ///< RC_Mode2: Up/down count of revolution counter on overflow or underflow in position count match
    QprcRcMode3 = 3u,  ///< RC_Mode3: Up/down count of revolution counter on overflow or underflow in position count match and ZIN active edge
} en_qprc_rcmode_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define ZIN valid edge
 ******************************************************************************/
typedef enum en_qprc_zinedge
{
    QprcZinDisable     = 0u,  ///< Disables edge and level detection
    QprcZinFallingEdge = 1u,  ///< ZIN active at falling edge
    QprcZinRisingEdge  = 2u,  ///< ZIN active at rising edge
    QprcZinBothEdges   = 3u,  ///< ZIN active at falling or rising edge
    QprcZinLowLevel    = 4u,  ///< ZIN active at low level detected
    QprcZinHighLevel   = 5u,  ///< ZIN active at high level detected
} en_qprc_zinedge_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define BIN valid edge
 ******************************************************************************/
typedef enum en_qprc_binedge
{
    QprcBinDisable     = 0u,  ///< Disables edge detection
    QprcBinFallingEdge = 1u,  ///< BIN active at falling edge
    QprcBinRisingEdge  = 2u,  ///< BIN active at rising edge
    QprcBinBothEdges   = 3u,  ///< BIN active at falling or rising edge
} en_qprc_binedge_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define AIN valid edge
 ******************************************************************************/
typedef enum en_qprc_ainedge
{
    QprcAinDisable     = 0u,  ///< Disables edge detection
    QprcAinFallingEdge = 1u,  ///< AIN active at falling edge
    QprcAinRisingEdge  = 2u,  ///< AIN active at rising edge
    QprcAinBothEdges   = 3u,  ///< AIN active at falling or rising edge
} en_qprc_ainedge_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define reset mask times of position counter
 ******************************************************************************/
typedef enum en_qprc_pcresetmask
{
    QprcResetMaskDisable  = 0u,  ///< No reset mask
    QprcResetMask2Times   = 1u,  ///< The position counter reset or a revolution counter count-up or -down events are ignored until the position counter changes twice
    QprcResetMask4Times   = 2u,  ///< The position counter reset or a revolution counter count-up or -down events are ignored until the position counter changes four times
    QprcResetMask8Times   = 3u,  ///< The position counter reset or a revolution counter count-up or -down events are ignored until the position counter changes eight times
} en_qprc_pcresetmask_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define compare object of QPRCR register
 ******************************************************************************/
typedef enum en_qprc_compmode
{
    QprcCompareWithPosition   = 0u,  ///< Compares the value of the QPRC Position and Revolution Counter Compare Register (QPRCR) with that of the position counter.
    QprcCompareWithRevolution = 1u,  ///< Compares the value of the QPRC Position and Revolution Counter Compare Register (QPRCR) with that of the revolution counter.
} en_qprc_compmode_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define QPRC interrupt type
 ******************************************************************************/
typedef enum en_qprc_irq_sel
{
    QprcPcOfUfZeroIrq       = 0u,    ///< Overflow, underflow, zero match interrupt of position counter
    QprcPcMatchIrq          = 1u,    ///< PC match interrupt of position counter
    QprcPcRcMatchIrq        = 2u,    ///< PC and RC match interrupt
    QprcPcMatchRcMatchIrq   = 3u,    ///< PC match and RC match interrupt
    QprcPcCountInvertIrq    = 4u,    ///< PC invert interrupt
    QprcRcOutrangeIrq       = 5u,    ///< RC outrange interrupt

}en_qprc_irq_sel_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define QPRC interrupt selection
 ******************************************************************************/
typedef struct stc_qprc_irq_en
{
    boolean_t    bQprcPcOfUfZeroIrq;      ///< Overflow, underflow, zero match interrupt of position counter, 1: selected, 0: not selected
    boolean_t    bQprcPcMatchIrq;         ///< PC match interrupt of position counter, 1: selected, 0: not selected
    boolean_t    bQprcPcRcMatchIrq;       ///< PC and RC match interrupt, 1: selected, 0: not selected
    boolean_t    bQprcPcMatchRcMatchIrq;  ///< PC match and RC match interrupt, 1: selected, 0: not selected
    boolean_t    bQprcPcCountInvertIrq;   ///< PC invert interrupt, 1: selected, 0: not selected
    boolean_t    bQprcRcOutrangeIrq;      ///< RC outrange interrupt, 1: selected, 0: not selected

}stc_qprc_irq_en_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define QPRC interrupt callback function
 ******************************************************************************/
typedef struct stc_qprc_irq_cb
{
    func_ptr_arg1_t   pfnPcOfUfZeroIrqCb;      ///< Overflow, underflow, zero match interrupt callback function of position counter
    func_ptr_t        pfnPcMatchIrqCb;         ///< PC match interrupt callback function of position counter
    func_ptr_t        pfnPcRcMatchIrqCb;       ///< PC and RC match interrupt callback function
    func_ptr_t        pfnPcMatchRcMatchIrqCb;  ///< PC match and RC match interrupt callback function
    func_ptr_t        pfnPcCountInvertIrqCb;   ///< PC invert interrupt callback function
    func_ptr_t        pfnRcOutrangeIrqCb;      ///< RC outrange interrupt callback function

}stc_qprc_irq_cb_t, stc_qprc_intern_data_t;

/**
 ******************************************************************************
 ** \brief Enumeration to define QPRC filter width
 ******************************************************************************/
typedef enum en_qprc_filter_width
{
    QprcNoFilter             = 0u,     ///< No filter
    QprcFilterWidth4Pclk     = 1u,     ///< QPRC filter width: 4 PCLK
    QprcFilterWidth8Pclk     = 2u,     ///< QPRC filter width: 8 PCLK
    QprcFilterWidth16Pclk    = 3u,     ///< QPRC filter width: 16 PCLK
    QprcFilterWidth32Pclk    = 4u,     ///< QPRC filter width: 32 PCLK
    QprcFilterWidth64Pclk    = 5u,     ///< QPRC filter width: 64 PCLK
    QprcFilterWidth128Pclk   = 6u,     ///< QPRC filter width: 128 PCLK
    QprcFilterWidth256Pclk   = 7u,     ///< QPRC filter width: 256 PCLK

}en_qprc_filter_width_t;

/**
 ******************************************************************************
 ** \brief Structure to configuration QPRC filter
 ******************************************************************************/
typedef struct stc_qprc_filter
{
    boolean_t               bInputMask;     ///< Input mask setting
    boolean_t               bInputInvert;   ///< Input invert setting
    en_qprc_filter_width_t  enWidth;        ///< QPRC filter width setting

}stc_qprc_filter_t;

/**
 ******************************************************************************
 ** \brief Structure to configuration QPRC
 ******************************************************************************/
typedef struct stc_qprc_config
{
    boolean_t             bSwapAinBin;                    ///< TRUE: Swap AIN and BIN inputs
    en_qprc_compmode_t    enCompareMode;                  ///< Description see #en_qprc_compmode_t
    en_qprc_zinedge_t     enZinEdge;                      ///< Detection mode of the ZIN pin
    en_qprc_binedge_t     enBinEdge;                      ///< Detection mode of the BIN pin
    en_qprc_ainedge_t     enAinEdge;                      ///< Detection mode of the AIN pin
    en_qprc_pcresetmask_t enPcResetMask;                  ///< Position counter reset mask
    boolean_t             b8KValue;                       ///< TRUE: Outrange mode from 0 to 0x7FFF, FALSE: Outrange mode from 0 to 0xFFFF:
#if (PDL_MCU_TYPE >= PDL_FM4_TYPE3) 
    boolean_t             bPhaseEdge;                     ///< Only meaningful for 1 -time frequency multiplication of PC_Mode2. TRUE: The QPCR is counted up or down by both edge (rising edge and falling edge).FALSE: The QPCR is counted up or down by the same edge (rising edge or falling edge).
#endif    
#if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE)
    stc_qprc_filter_t     stcAinFilter;                   ///< AIN noise filter configuration
    stc_qprc_filter_t     stcBinFilter;                   ///< BIN noise filter configuration
    stc_qprc_filter_t     stcCinFilter;                   ///< CIN noise filter configuration
#endif
    stc_qprc_irq_en_t*    pstcIrqEn;                      ///< Pointer to interrupt enable structure
    stc_qprc_irq_cb_t*    pstcIrqCb;                      ///< Pointer to interrupt callback function structure    
    boolean_t             bTouchNvic;                     ///< TRUE: enable NVIC, FALSE: don't enable NVIC
    
} stc_qprc_config_t;

/**
 ******************************************************************************
 ** \brief Structure of QPRC instance data
 ******************************************************************************/
typedef struct stc_qprc_instance_data
{
    volatile stc_qprcn_t*  pstcInstance;       ///< pointer to registers of an instance
#if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE)    
    volatile stc_qprc_nfn_t* pstcInstanceNf;   ///< pointer to registers of a QPRC-NF instance
#endif    
    stc_qprc_intern_data_t stcInternData;      ///< module internal data of instance
} stc_qprc_instance_data_t;

/******************************************************************************/
/* Global variable definitions ('extern')                                     */
/******************************************************************************/

/// Look-up table for all enabled MFS instances and their internal data
extern stc_qprc_instance_data_t m_astcQprcInstanceDataLut[QPRC_INSTANCE_COUNT];

/******************************************************************************/
/* Global function prototypes (definition in C source)                        */
/******************************************************************************/
/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/* QPRC init */
en_result_t Qprc_Init( volatile stc_qprcn_t* pstcQprc, stc_qprc_config_t*  pstcConfig );
en_result_t Qprc_DeInit( volatile stc_qprcn_t* pstcQprc, boolean_t bTouchNvic );
/* Stop/Restart Pc Count */
en_result_t Qprc_StopPcCount(volatile stc_qprcn_t *pstcQprc);
en_result_t Qprc_RestartPcCount(volatile stc_qprcn_t *pstcQprc);
/* Count write/read */
en_result_t Qprc_SetPcCount ( volatile stc_qprcn_t* pstcQprc, uint16_t u16PcValue ) ;
uint16_t Qprc_GetPcCount ( volatile stc_qprcn_t* pstcQprc );
en_result_t Qprc_SetRcCount ( volatile stc_qprcn_t* pstcQprc, uint16_t u16RcValue );
uint16_t Qprc_GetRcCount ( volatile stc_qprcn_t* pstcQprc );
en_result_t Qprc_SetPcMaxValue( volatile stc_qprcn_t* pstcQprc, uint16_t u16PcMaxValue ) ;
uint16_t Qprc_GetPcMaxValue(volatile stc_qprcn_t *pstcQprc);
en_result_t Qprc_SetPcCompareValue( volatile stc_qprcn_t* pstcQprc, uint16_t u16PcValue ) ;
uint16_t Qprc_GetPcCompareValue( volatile stc_qprcn_t *pstcQprc);
en_result_t Qprc_SetPcRcCompareValue( volatile stc_qprcn_t* pstcQprc, uint16_t u16PcRcValue ) ;
uint16_t Qprc_GetPcRcCompareValue(volatile stc_qprcn_t *pstcQprc);
/* Mode configuration */
en_result_t Qprc_ConfigPcMode( volatile stc_qprcn_t* pstcQprc, en_qprc_pcmode_t enMode );
en_result_t Qprc_ConfigRcMode( volatile stc_qprcn_t* pstcQprc, en_qprc_rcmode_t enMode );
/* Interrupt configuration */
#if (PDL_INTERRUPT_ENABLE_QPRC0 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_QPRC1 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_QPRC2 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_QPRC3 == PDL_ON)
en_result_t Qprc_EnableIrq( volatile stc_qprcn_t* pstcQprc, en_qprc_irq_sel_t enIrqSel);
en_result_t Qprc_DisableIrq( volatile stc_qprcn_t* pstcQprc, en_qprc_irq_sel_t enIrqSel );
#endif
en_irq_flag_t Qprc_GetIrqFlag( volatile stc_qprcn_t* pstcQprc, en_qprc_irq_sel_t enIrqSel );
en_result_t Qprc_ClrIrqFlag( volatile stc_qprcn_t *pstcQprc, en_qprc_irq_sel_t enIrqSel );
/* status */
en_stat_flag_t Qprc_GetPcOfUfDir( volatile stc_qprcn_t* pstcQprc );
en_stat_flag_t Qprc_GetPcDir( volatile stc_qprcn_t* pstcQprc );
/* IRQ handler */
void Qprc_IrqHandler ( volatile stc_qprcn_t* pstcQprc,
                       stc_qprc_intern_data_t* pstcQprcInternData );
#ifdef __cplusplus
}
#endif

//@} // QprcGroup

#endif // #if (defined(PDL_PERIPHERAL_QPRC_ACTIVE))

#endif /* __QPRC_H__ */
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
