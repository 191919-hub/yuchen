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
/** \file mft_icu.h
 **
 ** Header file for MFT's Input Capture Unit functions
 **  
 ** History:
 **   - 2014-11-21  0.1  Edison Zhang  First version for FM uinversal PDL.
 **
 *****************************************************************************/

#ifndef __MFT_ICU_H__
#define __MFT_ICU_H__

#include "mcu.h"
#include "pdl_user.h"   
   
#if (defined(PDL_PERIPHERAL_MFT_ICU_ACTIVE))

/**
 ******************************************************************************
 ** \defgroup IcuGroup Input Capture Unit (ICU)
 **
 ** Provided functions of ICU module:
 ** 
 ** - Mft_Icu_SelFrt()
 ** - Mft_Icu_ConfigDetectMode() 
 ** - Mft_Icu_EnableIrq()
 ** - Mft_Icu_DisableIrq()
 ** - Mft_Icu_GetIrqFlag()
 ** - Mft_Icu_ClrIrqFlag()
 ** - Mft_Icu_GetLastEdge()
 ** - Mft_Icu_GetCaptureData()
 ** - Mft_Icu_IrqHandler()
 **
 ** <b>How to use ICU module?</b>
 ** 
 ** Before using OCU, a FRT used to connect with applying OCU must be initialed.
 ** For how to configure FRT, see the description in the mft_frt.h. With 
 ** Mft_Icu_SelFrt(), a FRT can be connected with OCU.
 **
 ** A ICU interrupt can be enabled by the function Mft_Icu_EnableIrq().
 ** This function can set callback function for each channel too.
 **
 ** After above setting, calling Mft_Icu_ConfigDetectMode() will select a 
 ** detection mode and start ICU operation at the same time. Following detection
 ** mode cna be select:
 ** - Disable
 ** - Rising edge detection mode
 ** - Falling edge detection mode
 ** - Both edge detection mode
 **
 ** With interrupt mode, the interrupt occurs when valid edge is detected, the 
 ** interrupt flag will be cleared and run into user interrupt callback 
 ** function. In the callback function, the capture value can be read with
 ** Mft_Icu_GetCaptureData().
 **
 ** With polling mode, user can use Mft_Icu_GetIrqFlag() to check if the 
 ** interrupt occurs, and clear the interrupt flag by Mft_Icu_ClrIrqFlag().
 ** when interrupt is detected, the capture value can be read with
 ** Mft_Icu_GetCaptureData().
 **
 ** When the valid edge is detected, Mft_Icu_GetLastEdge() can get the edge
 ** information of last valid edge.
 ** 
 ** When stopping the ICU, use Mft_Icu_ConfigDetectMode() to disable ICU and 
 ** Mft_Icu_DisableIrq() to disable ICU interrupt.
 **
 ******************************************************************************/  
//@{
/******************************************************************************
 * Global type definitions
 ******************************************************************************/
#define stc_mftn_icu_t   FM_MFT_ICU_TypeDef
#define MFT0_ICU  (*((volatile stc_mftn_icu_t *) FM_MFT0_ICU_BASE))
#define MFT1_ICU  (*((volatile stc_mftn_icu_t *) FM_MFT1_ICU_BASE))
#define MFT2_ICU  (*((volatile stc_mftn_icu_t *) FM_MFT2_ICU_BASE))   

#define  MFT_ICU_CH0        0u
#define  MFT_ICU_CH1        1u
#define  MFT_ICU_CH2        2u
#define  MFT_ICU_CH3        3u

#define  MFT_ICU_CHx_MAX    3u

#define MFT_ICU_INSTANCE_COUNT \
        (uint8_t)(PDL_PERIPHERAL_ENABLE_MFT0_ICU == PDL_ON) + \
        (uint8_t)(PDL_PERIPHERAL_ENABLE_MFT1_ICU == PDL_ON) + \
        (uint8_t)(PDL_PERIPHERAL_ENABLE_MFT2_ICU == PDL_ON)

/**
 ******************************************************************************
 ** \brief Enumeration to define an index for each ICU instance
 ******************************************************************************/   
typedef enum en_icu_instance_index
{
#if (PDL_PERIPHERAL_ENABLE_MFT0_ICU == PDL_ON)   
    IcuInstanceIndexIcu0,              ///< Instance index of ICU0
#endif
#if (PDL_PERIPHERAL_ENABLE_MFT1_ICU == PDL_ON)   
    IcuInstanceIndexIcu1,              ///< Instance index of ICU1
#endif
#if (PDL_PERIPHERAL_ENABLE_MFT2_ICU == PDL_ON)
    IcuInstanceIndexIcu2,              ///< Instance index of ICU2
#endif
} en_icu_instance_index_t;  
   
/**
 ******************************************************************************
 ** \brief Frt channel
 ** 
 ** To select FRT channel to connect to ICU
 ******************************************************************************/
typedef enum en_mft_icu_frt
{
    Frt0ToIcu    = 0u,                      ///< connect Frt channel 0 to Icu
    Frt1ToIcu    = 1u,                      ///< connect Frt channel 1 to Icu
    Frt2ToIcu    = 2u,                      ///< connect Frt channel 2 to Icu
    IcuFrtToExt0 = 3u,                      ///< connect extern Frt channel 0 to Icu
    IcuFrtToExt1 = 4u,                      ///< connect extern Frt channel 1 to Icu
  
}en_mft_icu_frt_t;

/**
 ******************************************************************************
 ** \brief Icu mode
 ** 
 ** To select Icu edge detection mode
 ******************************************************************************/
typedef enum en_mft_icu_mode
{
    IcuDisable       = 0u,                  ///< disable Icu edge detection
    IcuRisingDetect  = 1u,                  ///< detect Icu rising edge
    IcuFallingDetect = 2u,                  ///< detect Icu falling edge
    IcuBothDetect    = 3u,                  ///< detect Icu rising/falling edge

}en_mft_icu_mode_t;

/**
 ******************************************************************************
 ** \brief Icu edge
 ** 
 ** To select Icu edge
 ******************************************************************************/
typedef enum en_icu_edge
{
    IcuFallingEdge = 0u,                    ///< select Icu falling edge
    IcuRisingEdge  = 1u,                    ///< select Icu rising edge
    
}en_mft_icu_edge_t;

/**
 ******************************************************************************
 ** \brief Icu instance internal data,
 **        storing internal information for each enabled Icu instance
 ******************************************************************************/
typedef struct stc_mft_icu_intern_data
{
    func_ptr_t  pfnIcu0IrqCb;               ///< Callback function pointer of ICU0 interrupt
    func_ptr_t  pfnIcu1IrqCb;               ///< Callback function pointer of ICU1 interrupt
    func_ptr_t  pfnIcu2IrqCb;               ///< Callback function pointer of ICU2 interrupt
    func_ptr_t  pfnIcu3IrqCb;               ///< Callback function pointer of ICU3 interrupt
    
}stc_mft_icu_intern_data_t;
/**
 ******************************************************************************
 ** \brief Mft_icu instance data type
 ******************************************************************************/
typedef struct stc_mft_icu_instance_data
{
    volatile stc_mftn_icu_t*  pstcInstance; ///< pointer to registers of an instance
    stc_mft_icu_intern_data_t stcInternData;///< module internal data of instance
    
}stc_mft_icu_instance_data_t;
/******************************************************************************/
/* Global variable definitions ('extern')                                     */
/******************************************************************************/

/** Look-up table for all enabled MFT_ICU instances and their internal data */
extern stc_mft_icu_instance_data_t m_astcMftIcuInstanceDataLut[MFT_ICU_INSTANCE_COUNT];

/** C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
/** Clock */
en_result_t Mft_Icu_SelFrt(volatile stc_mftn_icu_t*pstcMft, uint8_t u8Ch, en_mft_icu_frt_t enFrt);
/** mode set */
en_result_t Mft_Icu_ConfigDetectMode(volatile stc_mftn_icu_t*pstcMft, uint8_t u8Ch, en_mft_icu_mode_t enMode);
/** Interrupt */
#if (PDL_INTERRUPT_ENABLE_MFT0_ICU == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_MFT1_ICU == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_MFT2_ICU == PDL_ON)
en_result_t Mft_Icu_EnableIrq(volatile stc_mftn_icu_t*pstcMft, 
                              uint8_t u8Ch, 
                              func_ptr_t pfnIrqCb,
                              boolean_t bTouchNvic);
en_result_t Mft_Icu_DisableIrq(volatile stc_mftn_icu_t*pstcMft, 
                               uint8_t u8Ch,
                               boolean_t bTouchNvic);
#endif
en_irq_flag_t Mft_Icu_GetIrqFlag(volatile stc_mftn_icu_t*pstcMft, uint8_t u8Ch);
en_result_t Mft_Icu_ClrIrqFlag(volatile stc_mftn_icu_t*pstcMft, uint8_t u8Ch);
/** Status read/write */
en_mft_icu_edge_t Mft_Icu_GetLastEdge(volatile stc_mftn_icu_t*pstcMft, uint8_t u8Ch);
/** Count read */
uint16_t Mft_Icu_GetCaptureData(volatile stc_mftn_icu_t*pstcMft, uint8_t u8Ch);
/** IRQ handler */
void Mft_Icu_IrqHandler(volatile stc_mftn_icu_t*pstcMft, stc_mft_icu_intern_data_t* pstcMftIcuInternData) ;

#ifdef __cplusplus
}
#endif

//@}

#endif

#endif
