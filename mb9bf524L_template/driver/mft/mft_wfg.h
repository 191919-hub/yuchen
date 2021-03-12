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
/** \file mft_wfg.h
 **
 ** Headerfile for MFT's Waveform Generator functions
 **  
 ** History:
 **   - 2014-12-03  0.1  EZh  First version for FM uinversal PDL
 **
 *****************************************************************************/

#ifndef __MFT_WFG_H__
#define __MFT_WFG_H__

#include "mcu.h"
#include "pdl_user.h"

#if (defined(PDL_PERIPHERAL_MFT_WFG_ACTIVE))

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
  
/**
 ******************************************************************************
 ** \defgroup WfgGroup Waveform Generator (WFG)
 **
 ** Provided functions of WFG module:
 ** 
 ** - Mft_Wfg_Init()
 ** - Mft_Wfg_DeInit() 
 ** - Mft_Wfg_StartTimer()
 ** - Mft_Wfg_StopTimer() 
 ** - Mft_Wfg_WriteTimerCountCycle()
 ** - Mft_Wfg_GetTimerIrqFlag()
 ** - Mft_Wfg_ClrTimerIrqFlag()
 ** - Mft_Wfg_WriteTimerCountCycle()
 ** - Mft_Wfg_Nzcl_Init()
 ** - Mft_Wfg_Nzcl_DeInit()
 ** - Mft_Wfg_Nzcl_SwTiggerDtif()
 ** - Mft_Wfg_Nzcl_GetDigitalFilterIrqFlag() 
 ** - Mft_Wfg_Nzcl_ClrDigitalFilterIrqFlag() 
 ** - Mft_Wfg_Nzcl_GetAnalogFilterIrqFlag()
 ** - Mft_Wfg_Nzcl_ClrAnalogFilterIrqFlag()
 **
 ** <b>The WFG can be configured to following mode</b>:
 ** - Through mode
 ** - RT-PPG mode
 ** - Timer-PPG mode
 ** - RT-dead timer mode
 ** - PPG-dead timer mode 
 **
 ** <b>how to operate WFG with Through mode (one of usages)?</b>
 **   
 ** Before using WFG, the FRT used to connect with applying OCU must be 
 ** initialed first and OCU must be initialed then. 
 **
 ** For how to configure FRT, see the description in the mft_frt.h
 **
 ** For how to configure OCU, see the description in the mft_ocu.h
 **  
 ** Configure the WFG structure of the type #stc_wfg_config_t:
 ** - enMode = Through mode  
 ** - enGtenBits = b'00
 ** - enPselBits = b'00
 ** - enPgenBits = b'00  
 ** - enDmodBit = b'0
 **
 ** Call Mft_Wfg_Init() with the structure configued. 
 **  
 ** Enable OCU operation and RT signal will output to RTO pin directly.
 ** Changing the value of control bits will influence RTO output. 
 **
 **<b>how to operate WFG with RT-PPG mode (one of usages)? </b>
 **  
 ** Before using WFG, the FRT used to connect with applying OCU must be 
 ** initialized first and OCU must be initialed then, PPG should be initialized
 ** at the following.  
 **
 ** For how to configure FRT, see the description in the mft_frt.h
 **
 ** For how to configure OCU, see the description in the mft_ocu.h  
 **
 ** For how to configure PPG, see the description in the ppg.h
 **
 ** Configure the WFG structure of the type #stc_wfg_config_t:
 ** - enMode = RT-PPG mode  
 ** - enGtenBits = b'00 
 ** - enPselBits = b'00 (input PPG ch.0 to WFG0)
 ** - enPgenBits = b'11  
 ** - enDmodBit = b'0  
 **
 ** Call Mft_Wfg_Init() with the structure configued. 
 **  
 ** Start PPG0 and enable OCU operation. 
 **  
 ** In this case RTO0 will outputs the logic AND signal of RT0 signal and PPG0,
 ** RTO1 will outputs the logic AND signal of RT1 signal and PPG0.
 ** Changing the value of control bits (GTEN, PSEL, PGEN, DMOF) will influence 
 ** RTO output.  
 ** 
 ** <b>how to operate WFG with Timer-PPG mode (one of usages)? </b>
 ** 
 ** Before using WFG, the FRT used to connect with applying OCU must be 
 ** initialized first and OCU must be initialed then, PPG and WFG timer 
 ** should be initialized at the following.  
 **
 ** For how to configure FRT, see the description in the mft_frt.h
 **
 ** For how to configure OCU, see the description in the mft_ocu.h  
 **
 ** For how to configure PPG, see the description in the ppg.h 
 **
 ** For how to initialize WFG timer, see the description at the following.
 ** 
 ** Configure the WFG structure of the type #stc_wfg_config_t:
 ** - enGtenBits = b'00 
 ** - enPselBits = b'00 (input PPG ch.0 to WFG10)
 ** - enPgenBits = b'11  
 ** - enDmodBit = b'0    
 **  
 ** Configure WFG to Timer-PPG mode with Mft_Wfg_ConfigMode()   
 **  
 ** Start PPG0, enable OCU operation and start WFG timer.   
 **
 ** In this case RTO0 will outputs the logic AND signal of WFG timer flag 0
 ** and PPG0, RTO1 will outputs the logic AND signal of WFG timer flag 1 
 ** and PPG0. 
 ** Changing the value of control bits will influence RTO output.  
 **
 ** <b>how to operate WFG with RT-dead timer mode (one of usages)?</b>
 ** 
 ** Before using WFG, the FRT used to connect with applying OCU must be 
 ** initialized first and OCU must be initialed then, WFG timer 
 ** should be initialized at the following.  
 **
 ** For how to configure FRT, see the description in the mft_frt.h
 **
 ** For how to configure OCU, see the description in the mft_ocu.h  
 **
 ** For how to initialize WFG timer, see the description at the following.
 ** 
 ** Configure WFG to RT-dead timer mode with Mft_Wfg_ConfigMode()  
 **  
 ** Configure the WFG structure of the type #stc_wfg_config_t:
 ** - enMode = RT-dead timer mode   
 ** - enClk  = xxx  
 ** - enGtenBits = b'00 
 ** - enPselBits = b'00 
 ** - enPgenBits = b'00  
 ** - enDmodBit = b'0    
 **
 ** Call Mft_Wfg_Init() with the structure configued. 
 **  
 ** Enable OCU operation and start WFG timer.   
 **
 ** In this case, RT(1) will trigger the WFG timer to start, WFG will output
 ** the generated non-overlap signal.  
 ** Changing the value of control bits will influence RTO output.  
 **  
 ** <b>how to operate WFG with PPG-dead timer mode (one of usages)?</b>
 ** 
 ** Before using WFG, WFG timer should be initialized at the following.  
 ** For how to initialize WFG timer, see the description at the following.
 ** 
 ** Configure WFG to PPG-dead timer mode with Mft_Wfg_ConfigMode()  
 **  
 ** Configure the WFG structure of the type #stc_wfg_config_t:
 ** - enMode = PPG-dead timer mode   
 ** - enClk  = xxx  
 ** - enGtenBits = b'00 
 ** - enPselBits = b'00 (input PPG0 to WFG10)
 ** - enPgenBits = b'00  
 ** - enDmodBit = b'0    
 **  
 ** Call Mft_Wfg_Init() with the structure configued.    
 **  
 ** Start WFG timer.   
 **
 ** In this case, PPG0 will trigger the WFG timer to start, WFG will output
 ** the generated non-overlap singal.  
 ** Changing the value of control bits will influence RTO output.   
 ** 
 ** <b>How to use the NZCL (used for motor emergency shutdown) ? </b>
 **
 ** Mft_Wfg_Nzcl_Init() must be used for configuration of the WFG NZCL
 ** with a structure of the type #stc_wfg_nzcl_config_t. 
 **  
 ** Both analog noise filter and digital noise filter can be used to filter the
 ** signal input from DTTIX pin.
 **  
 ** Mft_Wfg_Nzcl_SwTiggerDtif() is used to trigger the DTIF interrupt by 
 ** software, regardless of DTTIX pin input status.    
 **
 ** With interrupt mode, when the interrupt occurs, it will run into user 
 ** interrupt callback function set with the Mft_Wfg_Nzcl_Init().
 **
 ** With polling mode, user can use Mft_Wfg_Nzcl_GetDigitalFilterIrqFlag() to 
 ** check if the interrupt of digital noise filter occurs, and clear the 
 ** interrupt flag by Mft_Wfg_Nzcl_ClrDigitalFilterIrqFlag(). user can use 
 ** Mft_Wfg_Nzcl_GetAnalogFilterIrqFlag() to check if the interrupt of analog 
 ** noise filter occurs, and clear the interrupt flag by 
 ** Mft_Wfg_Nzcl_ClrAnalogFilterIrqFlag(). 
 ** 
 ** When stopping the WFG NZCL, use Mft_Wfg_Nzcl_DeInit() to disable WFG NZCL
 ** operation as well as WFG NZCL interrupt. 
 **  
 ******************************************************************************/    
//@{  
/******************************************************************************
 * Global definitions
 ******************************************************************************/
#define stc_mftn_wfg_t FM_MFT_WFG_TypeDef

#define MFT0_WFG       (*((volatile stc_mftn_wfg_t *) FM_MFT0_WFG_BASE))
#define MFT1_WFG       (*((volatile stc_mftn_wfg_t *) FM_MFT1_WFG_BASE))
#define MFT2_WFG       (*((volatile stc_mftn_wfg_t *) FM_MFT2_WFG_BASE))

#define  MFT_WFG_CH0             0u
#define  MFT_WFG_CH1             1u
#define  MFT_WFG_CH2             2u
#define  MFT_WFG_CH3             3u 
#define  MFT_WFG_CH4             4u
#define  MFT_WFG_CH5             5u

#define  MFT_WFG_CH10            0u
#define  MFT_WFG_CH32            1u
#define  MFT_WFG_CH54            2u
#define  MFT_WFG_MAXCH           3u
  
#define WFG_INSTANCE_COUNT       (PDL_PERIPHERAL_ENABLE_MFT0_WFG == PDL_ON ? 1u : 0u) + \
                                 (PDL_PERIPHERAL_ENABLE_MFT1_WFG == PDL_ON ? 1u : 0u) + \
                                 (PDL_PERIPHERAL_ENABLE_MFT2_WFG == PDL_ON ? 1u : 0u)  

/**
 ******************************************************************************
 ** \brief Enumeration to define an index for each WFG instance
 ******************************************************************************/   
typedef enum en_wfg_instance_index
{
#if (PDL_PERIPHERAL_ENABLE_MFT0_WFG == PDL_ON)  
    WfgInstanceIndexWfg0 = 0u, ///< Instance index of WFG0
#endif
#if (PDL_PERIPHERAL_ENABLE_MFT1_WFG == PDL_ON)
    WfgInstanceIndexWfg1,      ///< Instance index of WFG1
#endif 
#if (PDL_PERIPHERAL_ENABLE_MFT2_WFG == PDL_ON)
    WfgInstanceIndexWfg2,      ///< Instance index of WFG2
#endif    

} en_wfg_instance_index_t;  
  
/******************************************************************************
 * \brief WFG operation mode
 ******************************************************************************/
typedef enum en_mft_wfg_mode
{
    WfgThroughMode                 = 0u,           ///< through mode
    WfgRtPpgMode                   = 1u,           ///< RT-PPG mode
    WfgTimerPpgMode                = 2u,           ///< Timer-PPG mode
    WfgRtDeadTimerMode             = 4u,           ///< RT-dead timer mode
    WfgRtDeadTimerFilterMode       = 5u,           ///< RT-dead timer filter mode
    WfgPpgDeadTimerFilterMode      = 6u,           ///< PPG-dead timer filter mode
    WfgPpgDeadTimerMode            = 7u,           ///< PPG-dead timer mode
    
}en_mft_wfg_mode_t;

/******************************************************************************
 * \brief Enumeration of GTEN bits setting
 ******************************************************************************/
typedef enum en_gten_bits
{
    GtenBits00B = 0u,         ///< GTEN == b'00
    GtenBits01B = 1u,         ///< GTEN == b'01
    GtenBits10B = 2u,         ///< GTEN == b'10 
    GtenBits11B = 3u,         ///< GTEN == b'11
  
}en_gten_bits_t;

/******************************************************************************
 * \brief Enumeration of PSEL bits setting
 ******************************************************************************/
typedef enum en_psel_bits
{
    PselBits00B = 0u,    ///< PSEL == b'00
    PselBits01B = 1u,    ///< PSEL == b'01
    PselBits10B = 2u,    ///< PSEL == b'10
    PselBits11B = 3u,    ///< PSEL == b'11
  
}en_psel_bits_t;

/******************************************************************************
 * \brief Enumeration of PGEN bits setting
 ******************************************************************************/
typedef enum en_pgen_bits
{
    PgenBits00B  = 0u,       ///< PGEN == b'00
    PgenBits01B  = 1u,       ///< PGEN == b'01 
    PgenBits10B  = 2u,       ///< PGEN == b'10
    PgenBits11B  = 3u,       ///< PGEN == b'11
    
}en_pgen_bits_t;

/******************************************************************************
 * \brief Enumeration of DMOD bit setting
 ******************************************************************************/
typedef enum en_dmod_bits
{
    DmodBits00B = 0u,     ///< output RTO1 and RTO0 signals without changing the level
    DmodBits01B = 1u,     ///< output both RTO1 and RTO0 signals reversed
#if (PDL_MCU_CORE == PDL_FM4_CORE) ||  (PDL_MCU_CORE == PDL_FM0P_CORE)   
    DmodBits10B = 2u,     ///< Outputs the RTO(0) signal reversed, outputs the RTO(1) signal without changing the level.
    DmodBits11B = 3u,     ///< Outputs the RTO(0) signal without changing the level, Outputs the RTO(1 ) signal reversed.
#endif    
}en_dmod_bits_t;

/******************************************************************************
 * \brief Enumeration to set count clock prescaler
 ******************************************************************************/
typedef enum en_wfg_timer_clock
{
    WfgPlckDiv1    = 0u,     ///< WFG timer clock prescaler: None
    WfgPlckDiv2    = 1u,     ///< WFG timer clock prescaler: 1/2
    WfgPlckDiv4    = 2u,     ///< WFG timer clock prescaler: 1/4
    WfgPlckDiv8    = 3u,     ///< WFG timer clock prescaler: 1/8
    WfgPlckDiv16   = 4u,     ///< WFG timer clock prescaler: 1/16
    WfgPlckDiv32   = 5u,     ///< WFG timer clock prescaler: 1/32
    WfgPlckDiv64   = 6u,     ///< WFG timer clock prescaler: 1/64
#if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE)     
    WfgPlckDiv128  = 7u,     ///< WFG timer clock prescaler: 1/128
#endif  
}en_wfg_timer_clock_t;

/******************************************************************************
 * \brief Configuration structure of waveform generator
 ******************************************************************************/
typedef struct stc_wfg_config
{
    en_mft_wfg_mode_t enMode;    ///< select the output condition
    en_gten_bits_t   enGtenBits; ///< select the output condition
    en_psel_bits_t   enPselBits; ///< select the PPG timer unit
    en_pgen_bits_t   enPgenBits; ///< how to reflect the CH_PPG signal
    en_dmod_bits_t   enDmodBits; ///< polarity for RTO0 and RTO1 signal output
    
    en_wfg_timer_clock_t enClk;  ///< clock division of WFG timer

#if (PDL_INTERRUPT_ENABLE_MFT0_WFG == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_MFT1_WFG == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_MFT2_WFG == PDL_ON)      
    #if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE)    
    boolean_t   bWfgimerIrqMask;      ///< TRUE: mask timer interrupt, FALSE: don't mask timer interrupt
    #endif    
    func_ptr_t  pfnWfgTimerIrqCallback;  ///< Poitner to WFG timer callback function
    boolean_t   bTouchNvic;              ///< TRUE: enable WFG NVIC, FALSE, don't enable WFG NVIC
#endif    
    
}stc_wfg_config_t;

/******************************************************************************
 * \brief WFG internal data
 ******************************************************************************/
typedef struct stc_mft_wfg_intern_data
{
    func_ptr_t  pfnWfg10TimerIrqCallback;   ///< Callback function pointer of WFG10 timer interrupt callback  
    func_ptr_t  pfnWfg32TimerIrqCallback;   ///< Callback function pointer of WFG32 timer interrupt callback  
    func_ptr_t  pfnWfg54TimerIrqCallback;   ///< Callback function pointer of WFG54 timer interrupt callback  
    func_ptr_t  pfnDtifAnalogFilterIrqCallback;    ///< Callback function pointer of analog filter interrupt callback 
    func_ptr_t  pfnDtifDigtalFilterIrqCallback;    ///< Callback function pointer of analog filter interrupt callback 
}stc_mft_wfg_intern_data_t;

/******************************************************************************
 * brief noise canceling width for a digital noise-canceler
 ******************************************************************************/
typedef enum en_nzcl_filter_width
{
    NzlcNoFilter,      ///< no noise-canceling
    NzlcWidth4Cycle,   ///< 4 PCLK cycles
    NzlcWidth8Cycle,   ///< 8 PCLK cycles
    NzlcWidth16Cycle,  ///< 16 PCLK cycles
    NzlcWidth32Cycle,  ///< 32 PCLK cycles
#if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE)       
    NzlcWidth64Cycle,  ///< 64 PCLK cycles
    NzlcWidth128Cycle, ///< 128 PCLK cycles
    #if (PDL_MCU_TYPE >= PDL_FM4_TYPE3)
    NzlcWidth256Cycle, ///< 256 PCLK cycles
    #endif
#endif    
    
}en_nzcl_filter_width_t;

/******************************************************************************
 * brief NZCL configure
 ******************************************************************************/
typedef struct stc_wfg_nzcl_config
{
    boolean_t               bEnDigitalFilter;      ///< enable digital filter
    en_nzcl_filter_width_t  enDigitalFilterWidth;  ///< digital filter width
#if(PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE)    
    boolean_t               bEnAnalogFilter;       ///< enable analog filter
    #if(PDL_MCU_TYPE >= PDL_FM4_TYPE3)
    boolean_t               bHoldRto;              ///< TRUE: hold RTO when DTIF interrupt occurs, FALSE: don't hold RTO when DTIF interrupt occurs
    #endif
#endif    
    boolean_t               bSwitchToGpio;         ///< switch to GPIO   

#if (PDL_INTERRUPT_ENABLE_MFT0_WFG == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_MFT1_WFG == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_MFT2_WFG == PDL_ON)      
    #if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE)    
    boolean_t  bDtifDigitalFilterIrqMask;          ///< Digital filter interrupt selection  
    boolean_t  bDtifAnalogFilterIrqMask;           ///< Analog filter interrupt selection
    #endif      
    
    func_ptr_t  pfnDtifDigtalFilterIrqCallback;    ///< Pointer to digital filter interrupt callback function
    #if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE)  
    func_ptr_t  pfnDtifAnalogFilterIrqCallback;    ///< Pointer to analog filter interrupt callback function
    #endif
    
    boolean_t               bTouchNvic;            ///< TRUE: 
#endif
    
}stc_wfg_nzcl_config_t;

/******************************************************************************
 * brief structure of WFG instance data
 ******************************************************************************/
typedef struct stc_mft_wfg_instance_data
{
  volatile stc_mftn_wfg_t*  pstcInstance;  ///< pointer to registers of an instance
  stc_mft_wfg_intern_data_t stcInternData; ///< module internal data of instance
} stc_mft_wfg_instance_data_t;

/******************************************************************************/
/* Global variable declarations ('extern', definition in C source)            */
/******************************************************************************/
/// Look-up table for all enabled WFG  instances and their internal data
extern stc_mft_wfg_instance_data_t m_astcMftWfgInstanceDataLut[WFG_INSTANCE_COUNT];

/******************************************************************************/
/* Global function prototypes ('extern', definition in C source)              */
/******************************************************************************/
/* 1. WFG function configuration */
en_result_t Mft_Wfg_Init(volatile stc_mftn_wfg_t* pstcWfg,
		                 uint8_t u8CoupleCh, 
                         stc_wfg_config_t* pstcConfig);
en_result_t Mft_Wfg_DeInit(volatile stc_mftn_wfg_t* pstcWfg, 
                           uint8_t u8CoupleCh, 
                           boolean_t bTouchNvic);

en_result_t Mft_Wfg_StartTimer(volatile stc_mftn_wfg_t* pstcWfg, uint8_t u8CoupleCh);
en_result_t Mft_Wfg_StopTimer(volatile stc_mftn_wfg_t* pstcWfg, uint8_t u8CoupleCh);
en_irq_flag_t Mft_Wfg_GetTimerIrqFlag(volatile stc_mftn_wfg_t* pstcWfg, uint8_t u8CoupleCh);
en_result_t Mft_Wfg_ClrTimerIrqFlag(volatile stc_mftn_wfg_t* pstcWfg, uint8_t u8CoupleCh);
en_result_t Mft_Wfg_WriteTimerCountCycle(volatile stc_mftn_wfg_t* pstcWfg,
                                         uint8_t u8CoupleCh, 
                                         uint16_t u16CycleA, 
                                         uint16_t u16CycleB);
#if(PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE) 
en_result_t Mft_Wfg_SetFilterCountValue( volatile stc_mftn_wfg_t* pstcWfg,
                                         uint8_t u8CoupleCh, uint16_t u16Count);
#endif
/* 2. WFG NZCL configuration */
en_result_t Mft_Wfg_Nzcl_Init(volatile stc_mftn_wfg_t* pstcWfg,
                              stc_wfg_nzcl_config_t* pstcNzclConfig,
                              boolean_t bTouchNvic);
en_result_t Mft_Wfg_Nzcl_DeInit(volatile stc_mftn_wfg_t* pstcWfg, 
                                boolean_t bTouchNvic);
en_result_t Mft_Wfg_Nzcl_SwTiggerDtif(volatile stc_mftn_wfg_t* pstcWfg);
en_irq_flag_t Mft_Wfg_Nzcl_GetDigitalFilterIrqFlag(volatile stc_mftn_wfg_t* pstcWfg);
en_result_t Mft_Wfg_Nzcl_ClrDigitalFilterIrqFlag(volatile stc_mftn_wfg_t* pstcWfg);
#if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE)  
en_irq_flag_t Mft_Wfg_Nzcl_GetAnalogFilterIrqFlag(volatile stc_mftn_wfg_t* pstcWfg);
en_result_t Mft_Wfg_NzclClrAnalogFilterIrqFlag(volatile stc_mftn_wfg_t* pstcWfg);
#endif

/* 3. IRQ handler */
void Mft_Wfg_IrqHandler(volatile  stc_mftn_wfg_t* pstcWfg,
                        stc_mft_wfg_intern_data_t* pstcMftWfgInternData) ;

#ifdef __cplusplus
}
#endif

//@}

#endif // #if (defined(PDL_PERIPHERAL_ADC_ACTIVE))
#endif
