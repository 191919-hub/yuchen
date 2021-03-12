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
/** \file mft_ocu.h
 **
 ** Headerfile for MFT's Output Compare Unit functions
 **  
 ** History:
 **   - 2014-11-21  0.1  EZh  First version for FM uiniversal PDL.
 **
 *****************************************************************************/

#ifndef __MFT_OCU_H__
#define __MFT_OCU_H__
/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "mcu.h"
#include "pdl_user.h"

#if (defined(PDL_PERIPHERAL_MFT_OCU_ACTIVE))
  
/**
 ******************************************************************************
 ** \defgroup OcuGroup Output Compare Unit (OCU)
 **
 ** Provided functions of OCU module:
 ** 
 ** - Mft_Ocu_Init()
 ** - Mft_Ocu_DeInit()
 ** - Mft_Ocu_SetEvenChCompareMode() 
 ** - Mft_Ocu_SetOddChCompareMode()
 ** - Mft_Ocu_SetCompareMode_Fm3()
 ** - Mft_Ocu_EnableOperation()
 ** - Mft_Ocu_DisableOperation()
 ** - Mft_Ocu_EnableIrq()
 ** - Mft_Ocu_DisableIrq()
 ** - Mft_Ocu_WriteOccp()
 ** - Mft_Ocu_ReadOccp()
 ** - Mft_Ocu_GetIrqFlag()
 ** - Mft_Ocu_ClrIrqFlag()
 ** - Mft_Ocu_IrqHandler()
 **
 ** <b>How to use OCU module?</b>
 ** 
 ** Before using OCU, a FRT used to connect with applying OCU must be initialed.
 ** For how to configure FRT, see the description in the mft_frt.h   
 **
 ** Mft_Ocu_Init() must be used for configuration of an Output Compare Unit (OCU) 
 ** channel with a structure of the type #stc_mft_ocu_config_t. A FRT should be
 ** connected with applying OCU with this function.
 **
 ** For FM0+ and FM4 products:
 ** Mft_Ocu_SetEvenChCompareMode() is used to set the compare function/mode of 
 ** even channel with the structure type #stc_ocu_even_compare_config_t.
 ** Mft_Ocu_SetOddChCompareMode() is used to set the compare function/mode of 
 ** odd channel with the structure type #stc_ocu_odd_compare_config_t.
 **
 ** For all devices:
 ** Mft_Ocu_SetCompareMode_Fm3() can also be used to configure mode with FM3-
 ** compatible mode.
 ** 
 ** With Mft_Ocu_WriteOccp() the OCU compare value is set to the value
 ** given in the parameter Mft_Ocu_WriteOccp#u16Cycle. Whether the compare
 ** value is modified directly depends on buffer function. 
 **
 ** The interrupts can be configured in Mft_Ocu_Init(), after intialization, 
 ** The interrupt can be enabled or disabled by the function 
 ** Mft_Ocu_EnableIrq() and Mft_Ocu_DisableIrq().
 **
 ** After above setting, calling Mft_Ocu_EnableOperation() will start OCU.
 **
 ** With interrupt mode, the interrupt occurs when FRT count match OCU compare
 ** value, the interrupt flag will be cleared and run into user 
 ** interrupt callback function.
 **
 ** With polling mode, user can use Mft_Ocu_GetIrqFlag() to check if the 
 ** interrupt occurs, and clear the interrupt flag by Mft_Ocu_ClrIrqFlag().
 ** 
 ** When stopping the OCU, use Mft_Ocu_DisableOperation() to disable OCU and 
 ** Mft_Ocu_DisableIrq() to disable OCU interrupt.
 **
 ******************************************************************************/  
//@{ 
/******************************************************************************
 * Global definitions
 ******************************************************************************/
#define stc_mftn_ocu_t FM_MFT_OCU_TypeDef

#define MFT0_OCU       (*((volatile stc_mftn_ocu_t *) FM_MFT0_OCU_BASE))
#define MFT1_OCU       (*((volatile stc_mftn_ocu_t *) FM_MFT1_OCU_BASE))
#define MFT2_OCU       (*((volatile stc_mftn_ocu_t *) FM_MFT2_OCU_BASE))

#define  MFT_OCU_CH0             0u
#define  MFT_OCU_CH1             1u
#define  MFT_OCU_CH2             2u
#define  MFT_OCU_CH3             3u 
#define  MFT_OCU_CH4             4u
#define  MFT_OCU_CH5             5u
#define  MFT_OCU_MAXCH           6u

#define  MFT_OCU_CH10            0u
#define  MFT_OCU_CH32            1u
#define  MFT_OCU_CH54            2u

#define  OCU_INSTANCE_COUNT       (PDL_PERIPHERAL_ENABLE_MFT0_OCU == PDL_ON ? 1u :0u) + \
                                  (PDL_PERIPHERAL_ENABLE_MFT1_OCU == PDL_ON ? 1u :0u) + \
                                  (PDL_PERIPHERAL_ENABLE_MFT2_OCU == PDL_ON ? 1u :0u)  

/**
 ******************************************************************************
 ** \brief Enumeration to define an index for each OCU instance
 ******************************************************************************/   
typedef enum en_ocu_instance_index
{
#if (PDL_PERIPHERAL_ENABLE_MFT0_OCU == PDL_ON)  
    OcuInstanceIndexOcu0 = 0u,  ///< Instance index of OCU0
#endif
#if (PDL_PERIPHERAL_ENABLE_MFT1_OCU == PDL_ON) 
    OcuInstanceIndexOcu1,      ///< Instance index of OCU1
#endif 
#if (PDL_PERIPHERAL_ENABLE_MFT2_OCU == PDL_ON) 
    OcuInstanceIndexOcu2,      ///< Instance index of OCU2
#endif    

} en_ocu_instance_index_t;  
  
/**
 ******************************************************************************
 ** \brief select the FRT to be connected to OCU
 ******************************************************************************/
typedef enum en_mft_ocu_frt
{
    Frt0ToOcu   = 0u,       ///< connect FRT0 to OCU
    Frt1ToOcu   = 1u,       ///< connect FRT1 to OCU
    Frt2ToOcu   = 2u,       ///< connect FRT2 to OCU

}en_mft_ocu_frt_t;

/**
 ******************************************************************************
 ** \brief output level of the RT pin state
 ******************************************************************************/
typedef enum en_ocu_rt_out_state
{
    RtLowLevel  = 0u,         ///< output low level to RT pin
    RtHighLevel = 1u,         ///< output high level to RT pin
}en_ocu_rt_out_state_t;

/**
 ******************************************************************************
 ** \brief buffer register function of OCCP
 ******************************************************************************/
typedef enum en_ocu_occp_buf
{
    OccpBufDisable           = 0u,       ///< disable the buffer function
    OccpBufTrsfByFrtZero     = 1u,       ///< buffer transfer when counter value is 0x0000
    OccpBufTrsfByFrtPeak     = 2u,       ///< buffer transfer when counter value is TCCP
#if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE)     
    OccpBufTrsfByFrtZeroPeak = 3u,       ///< buffer transfer when the value is both 0 and TCCP
#endif    
#if (PDL_MCU_CORE == PDL_FM4_CORE)
    #if(PDL_MCU_TYPE >= PDL_FM4_TYPE3)
    OccpBufTrsfByFrtZeroMszcZero = 4u,       ///< buffer transfer when counter value is 0x0000 and zero value detection mask counter value is 0
    OccpBufTrsfByFrtPeakMspcZero = 5u,       ///< buffer transfer when counter value is TCCP and peak value detection mask counter value is 0
    OccpBufTrsfByFrtZeroMszcZeroOrFrtPeakMspcZero = 6u, ///< buffer transfer when counter value is 0x0000 and zero value detection mask counter value is 0 or                                                        
                                                        ///< when counter value is TCCP and peak value detection mask counter value is 0
    #endif
#endif
}en_ocu_occp_buf_t;

#if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE)   
/**
 ******************************************************************************
 ** \brief buffer register function of OCSE
 ******************************************************************************/
typedef enum en_ocu_ocse_buf
{
    OcseBufDisable           = 0u,       ///< disable the buffer function
    OcseBufTrsfByFrtZero     = 1u,       ///< buffer transfer when counter value is 0x0000
    OcseBufTrsfByFrtPeak     = 2u,       ///< buffer transfer when counter value is TCCP
    OcseBufTrsfByFrtZeroPeak = 3u,       ///< buffer transfer when the value is both 0 and TCCP
#if (PDL_MCU_CORE == PDL_FM4_CORE)
    #if(PDL_MCU_TYPE >= PDL_FM4_TYPE3)
    OcseBufTrsfByFrtZeroMszcZero     = 4u,  ///< buffer transfer when FRT counter value is 0x0000 and zero value detection mask counter value is 0
    OcseBufTrsfByFrtPeakMspcZero     = 5u,  ///< buffer transfer when FRT counter value is TCCP and peak value detection mask counter value is 0    
    OcseBufTrsfByFrtZeroMszcZeroOrFrtPeakMspcZero = 6u,  ///< buffer transfer when FRT counter value is 0x0000 and zero value detection mask counter value is 0 or
                                                         ///< when FRT counter value is TCCP and peak value detection mask counter value is 0 
    #endif
#endif
}en_ocu_ocse_buf_t;
#endif

/**
 ******************************************************************************
 ** \brief OCU configure
 ******************************************************************************/
typedef struct stc_mft_ocu_config
{
    en_mft_ocu_frt_t    enFrtConnect;  ///< select the FRT to be connected to OCU
    en_ocu_occp_buf_t   enOccpBufMode; ///< buffer register function of OCCP
#if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE)   
    en_ocu_ocse_buf_t   enOcseBufMode; ///< buffer register function of OCSE
    boolean_t           bFm4;          ///< TRUE: set the compare mode of used channels with Mft_Ocu_SetEvenChCompareMode() and Mft_Ocu_SetOddChCompareMode()  
                                       ///< FALSE: set the compare mode of used channels with Mft_Ocu_SetCompareMode_Fm3() 
#endif    
    en_ocu_rt_out_state_t enPinState;  ///< RT output level state

#if (PDL_INTERRUPT_ENABLE_MFT0_OCU == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_MFT1_OCU == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_MFT2_OCU == PDL_ON)    
    boolean_t  bIrqEnable;             ///< TRUE: enable OCU interrupt, FALSE: disable OCU interrupt
    func_ptr_t pfnIrqCallback;         ///< Pointer to OCU interrupt callback function.
    boolean_t  bTouchNvic;             ///< TRUE: enable NVIC, FALSE: disable NVIC
#endif    
    
}stc_mft_ocu_config_t;

#if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE) 
/**
 ******************************************************************************
 ** \brief RT output status
 ******************************************************************************/
typedef enum en_rt_status
{
    RtOutputHold     = 0u,              ///< RT output hold
    RtOutputHigh     = 1u,              ///< RT output high 
    RtOutputLow      = 2u,              ///< RT output low  
    RtOutputReverse  = 3u,              ///< RT output reverse  
  
}en_rt_even_status_t, en_rt_odd_status_t;

/**
 ******************************************************************************
 ** \brief The condition for IOP flag set
 ******************************************************************************/
typedef enum en_iop_flag_set_condition
{
    IopFlagHold = 0u,           ///< IOP flag hold
    IopFlagSet  = 1u,           ///< IOP flag set
  
}en_iop_flag_even_t, en_iop_flag_odd_t;

/**
 ******************************************************************************
 ** \brief The compare configuration of even OCU channel
 ******************************************************************************/
typedef struct stc_ocu_even_compare_config
{
    en_rt_even_status_t enFrtZeroEvenMatchEvenChRtStatus;    ///< Even channel's RT output status when even channel match occurs at the condition of FRT count=0x0000      
    en_rt_even_status_t enFrtZeroEvenNotMatchEvenChRtStatus; ///< Even channel's RT output status when even channel match doesn't occur at the condition of FRT count=0x0000    
    
    en_rt_even_status_t enFrtUpCntEvenMatchEvenChRtStatus;   ///< Even channel's RT output status when even channel match occurs at the condition of FRT is counting up     
    
    en_rt_even_status_t enFrtPeakEvenMatchEvenChRtStatus;  ///< Even channel's RT output status when even channel match occurs at the condition of FRT count=Peak
    en_rt_even_status_t enFrtPeakEvenNotMatchEvenChStatus; ///< Even channel's RT output status when even channel match doesn't occur at the condition of FRT count=Peak
    
    en_rt_even_status_t enFrtDownCntEvenMatchEvenChRtStatus;    ///< Even channel's RT output status when even channel match occurs at the condition of FRT is counting down 
    
    en_iop_flag_even_t enIopFlagWhenFrtZeroEvenMatch;    ///< Even channel OCU's IOP flag status when even channel match occurs at the condition of FRT count=0x0000
    en_iop_flag_even_t enIopFlagWhenFrtUpCntEvenMatch;   ///< Even channel OCU's IOP flag status when even channel match occurs at the condition of FRT is counting up
    en_iop_flag_even_t enIopFlagWhenFrtPeakEvenMatch;    ///< Even channel OCU's IOP flag status when even channel match occurs at the condition of FRT count=Peak
    en_iop_flag_even_t enIopFlagWhenFrtDownCntEvenMatch; ///< Even channel OCU's IOP flag status when even channel match occurs at the condition of FRT is counting down
    
#if (PDL_MCU_CORE == PDL_FM4_CORE)
    #if(PDL_MCU_TYPE >= PDL_FM4_TYPE3)
    boolean_t bFrtZeroPeakExtendMatchCondition;          ///< TRUE: The condition of that OCCP <= FRT counter value will be treated as comparison match
                                                         ///< FALSE: The condition of that OCCP > FRT counter value will be treated as comparison not match
                                                         
    #endif
#endif
    
}stc_ocu_even_compare_config_t;

/**
 ******************************************************************************
 ** \brief The compare configuration of odd OCU channel
 ******************************************************************************/
typedef struct stc_ocu_odd_compare_config
{
    en_rt_odd_status_t enFrtZeroOddMatchEvenMatchOddChRtStatus;       ///< Odd channel's RT output status when even channel and odd channel match occurs at the condition of FRT count=0x0000   
    en_rt_odd_status_t enFrtZeroOddMatchEvenNotMatchOddChRtStatus;    ///< Odd channel's RT output status when even channel not match and odd channel match occurs at the condition of FRT count=0x0000 
    en_rt_odd_status_t enFrtZeroOddNotMatchEvenMatchOddChRtStatus;    ///< Odd channel's RT output status when even channel match and odd channel not match occurs at the condition of FRT count=0x0000 
    en_rt_odd_status_t enFrtZeroOddNotMatchEvenNotMatchOddChRtStatus; ///< Odd channel's RT output status when even channel not match and odd channel not match occurs at the condition of FRT count=0x0000
    
    en_rt_odd_status_t enFrtUpCntOddMatchEvenMatchOddChRtStatus;      ///< Odd channel's RT output status when even channel and odd channel match occurs at the condition of FRT is counting up 
    en_rt_odd_status_t enFrtUpCntOddMatchEvenNotMatchOddChRtStatus;   ///< Odd channel's RT output status when even channel not match and odd channel match occurs at the condition of FRT is counting up
    en_rt_odd_status_t enFrtUpCntOddNotMatchEvenMatchOddChRtStatus;   ///< Odd channel's RT output status when even channel match and odd channel not match occurs at the condition of FRT is counting up
    
    en_rt_odd_status_t enFrtPeakOddMatchEvenMatchOddChRtStatus;      ///< Odd channel's RT output status when even channel and odd channel match occurs at the condition of FRT count=Peak
    en_rt_odd_status_t enFrtPeakOddMatchEvenNotMatchOddChRtStatus;   ///< Odd channel's RT output status when even channel not match and odd channel match occurs at the condition of FRT count=Peak
    en_rt_odd_status_t enFrtPeakOddNotMatchEvenMatchOddChRtStatus;   ///< Odd channel's RT output status when even channel match and odd channel not match occurs at the condition of FRT count=Peak
    en_rt_odd_status_t enFrtPeakOddNotMatchEvenNotMatchOddChRtStatus;///< Odd channel's RT output status when even channel not match and odd channel not match occurs at the condition of FRT count=Peak
    
    en_rt_odd_status_t enFrtDownOddMatchEvenMatchOddChRtStatus;      ///< Odd channel's RT output status when even channel and odd channel match occurs at the condition of FRT is counting down
    en_rt_odd_status_t enFrtDownOddMatchEvenNotMatchOddChRtStatus;   ///< Odd channel's RT output status when even channel not match and odd channel match occurs at the condition of FRT is counting down
    en_rt_odd_status_t enFrtDownOddNotMatchEvenMatchOddChRtStatus;   ///< Odd channel's RT output status when even channel match and odd channel not match occurs at the condition of FRT is coutning down
    
    en_iop_flag_odd_t enIopFlagWhenFrtZeroOddMatch;    ///< Odd channel OCU's IOP flag status when Odd channel match occurs at the condition of FRT count=0x0000
    en_iop_flag_odd_t enIopFlagWhenFrtUpCntOddMatch;   ///< Odd channel OCU's IOP flag status when Odd channel match occurs at the condition of FRT is counting up
    en_iop_flag_odd_t enIopFlagWhenFrtPeakOddMatch;    ///< Odd channel OCU's IOP flag status when Odd channel match occurs at the condition of FRT count=Peak
    en_iop_flag_odd_t enIopFlagWhenFrtDownCntOddMatch; ///< Odd channel OCU's IOP flag status when Odd channel match occurs at the condition of FRT is counting down

#if (PDL_MCU_CORE == PDL_FM4_CORE)
    #if(PDL_MCU_TYPE >= PDL_FM4_TYPE3)
    boolean_t bFrtZeroPeakExtendMatchCondition;          ///< TRUE: The condition of that OCCP <= FRT counter value will be treated as comparison match
                                                         ///< FALSE: The condition of that OCCP > FRT counter value will be treated as comparison not match
                                                         
    #endif
#endif
    
}stc_ocu_odd_compare_config_t;

#endif

/**
 ******************************************************************************
 ** \brief The compare mode of OCU couple channels (FM3 mode)
 **
 ** \note The OCU channel with 1 change mode or 2 change mode must connect 
 **       with FRT with up count mode.
 **       The OCU channel with active high mode or active low mode must  
 **       connect with FRT with up/down count mode.
 ******************************************************************************/
typedef enum en_ocu_compare_mode 
{
    OcuOdd1ChangeEven1Change    = 0u,    ///< Odd OCU channel is set to 1 change mode, even OCU channel is set to 1 change mode.
    OcuOdd2ChangeEven1Change    = 1u,    ///< Odd OCU channel is set to 2 change mode, even OCU channel is set to 1 change mode.
    OcuOdd1ChangeEvenActiveHigh = 2u,    ///< Odd OCU channel is set to 1 change mode, even OCU channel is set to active high mode.
    OcuOddActiveHighEven1Change = 3u,    ///< Odd OCU channel is set to active high mode, even OCU channel is set to 1 change mode.
    OcuOddActiveLowEven1Change  = 4u,    ///< Odd OCU channel is set to active low mode, even OCU channel is set to 1 change mode.
    OcuOddActiveHighEvenActiveHigh = 5u, ///< Odd OCU channel is set to active high mode, even OCU channel is set to active high mode.
    OcuOddActiveLowEvenActiveLow   = 6u, ///< Odd OCU channel is set to active low mode, even OCU channel is set to active low mode.
    
}en_ocu_compare_mode_t;

/**
 ******************************************************************************
 ** \brief Structure of OCU internal data
 ******************************************************************************/
typedef struct stc_mft_ocu_intern_data
{
    func_ptr_t  pfnOcu0IrqCallback;  ///< Callback function pointer of OCU0 interrupt 
    func_ptr_t  pfnOcu1IrqCallback;  ///< Callback function pointer of OCU1 interrupt 
    func_ptr_t  pfnOcu2IrqCallback;  ///< Callback function pointer of OCU2 interrupt 
    func_ptr_t  pfnOcu3IrqCallback;  ///< Callback function pointer of OCU3 interrupt 
    func_ptr_t  pfnOcu4IrqCallback;  ///< Callback function pointer of OCU4 interrupt 
    func_ptr_t  pfnOcu5IrqCallback;  ///< Callback function pointer of OCU5 interrupt 
}stc_mft_ocu_intern_data_t;

/**
 ******************************************************************************
 ** \brief OCU instance data type
 ******************************************************************************/
typedef struct stc_mft_ocu_instance_data
{
  volatile stc_mftn_ocu_t*  pstcInstance;  ///< pointer to registers of an instance
  stc_mft_ocu_intern_data_t stcInternData; ///< module internal data of instance
} stc_mft_ocu_instance_data_t;

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
/******************************************************************************/
/* Global variable declarations ('extern', definition in C source)            */
/******************************************************************************/
/// Look-up table for all enabled OCU  instances and their internal data
extern stc_mft_ocu_instance_data_t m_astcMftOcuInstanceDataLut[OCU_INSTANCE_COUNT];

/******************************************************************************/
/* Global function prototypes ('extern', definition in C source)              */
/******************************************************************************/
/* Init */
en_result_t Mft_Ocu_Init( volatile stc_mftn_ocu_t* pstcOcu, 
                          uint8_t u8Ch, 
                          stc_mft_ocu_config_t*  pstcOcuConfig);
en_result_t Mft_Ocu_DeInit( volatile stc_mftn_ocu_t* pstcOcu, 
                            uint8_t u8Ch,
                            boolean_t bTouchNvic);
#if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE)
/* Compare configuration */
en_result_t Mft_Ocu_SetEvenChCompareMode(volatile stc_mftn_ocu_t* pstcOcu,
                                         uint8_t EvenCh,
                                         stc_ocu_even_compare_config_t* pstcConfig);
en_result_t Mft_Ocu_SetOddChCompareMode (volatile stc_mftn_ocu_t* pstcOcu,
                                         uint8_t OddCh,
                                         stc_ocu_odd_compare_config_t* pstcConfig);
#endif
en_result_t Mft_Ocu_SetCompareMode_Fm3(volatile stc_mftn_ocu_t* pstcOcu,
                                       uint8_t u8CoupleCh,
                                       en_ocu_compare_mode_t enCompareMode);

/* Func/Int enable/disable */
en_result_t Mft_Ocu_EnableOperation(volatile stc_mftn_ocu_t* pstcOcu, uint8_t u8Ch);
en_result_t Mft_Ocu_DisableOperation(volatile stc_mftn_ocu_t* pstcOcu, uint8_t u8Ch);
#if (PDL_INTERRUPT_ENABLE_MFT0_OCU == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_MFT1_OCU == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_MFT2_OCU == PDL_ON)
en_result_t Mft_Ocu_EnableIrq(volatile stc_mftn_ocu_t* pstcOcu,
                              uint8_t u8Ch,func_ptr_t  pfnCallback);
en_result_t Mft_Ocu_DisableIrq(volatile stc_mftn_ocu_t* pstcOcu,uint8_t u8Ch);
#endif

/* Count write/read */
en_result_t Mft_Ocu_WriteOccp(volatile stc_mftn_ocu_t* pstcOcu, 
                              uint8_t u8Ch, uint16_t u16Occp);
uint16_t Mft_Ocu_ReadOccp(volatile stc_mftn_ocu_t* pstcOcu, uint8_t u8Ch);

/* Status read/clear */
en_irq_flag_t Mft_Ocu_GetIrqFlag(volatile stc_mftn_ocu_t* pstcOcu, uint8_t u8Ch);
en_result_t Mft_Ocu_ClrIrqFlag(volatile stc_mftn_ocu_t* pstcOcu, uint8_t u8Ch);

/* Get RT pin level */
en_ocu_rt_out_state_t Mft_Ocu_GetRtPinLevel(volatile stc_mftn_ocu_t* pstcOcu, 
                                            uint8_t u8Ch);

/* IRQ handler */
void Mft_Ocu_IrqHandler( volatile stc_mftn_ocu_t* pstcOcu,
                         stc_mft_ocu_intern_data_t* pstcMftOcuInternData) ;


#ifdef __cplusplus
}
#endif

//@}

#endif // #if (defined(PDL_PERIPHERAL_ADC_ACTIVE))

#endif
