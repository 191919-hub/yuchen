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
/** \file exint.h
 **
 ** Headerfile for EXINT functions
 **  
 ** History:
 **   - 2013-08-16  1.0  MWi  First version.
 **   - 2013-10-25  1.1  MWi  PDL_ITERRUPT_ENABLE_x corrected
 **
 ******************************************************************************/

#ifndef __EXINT_H__
#define __EXINT_H__

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_user.h"
#include "mcu.h"

#if (defined(PDL_PERIPHERAL_EXINT_ACTIVE)) || \
    (defined(PDL_PERIPHERAL_NMI_ACTIVE))


/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
  
/**
 ******************************************************************************
 ** \defgroup ExintGroup External Interrupts (EXINT)
 **
 ** Provided functions of EXINT module:
 ** 
 ** - Exint_Init()
 ** - Exint_DeInit()
 ** - Exint_EnableChannel()
 ** - Exint_DisableChannel()
 ** - Exint_SetDetectMode()
 ** - Exint_GetDetectMode()
 ** - Exint_Nmi_Init()
 ** - Exint_Nmi_DeInit()
 **
 ** \brief How to use External Interrupts
 **
 ** With Exint_Init() the external interrupts are enabled as given in the
 ** configuration. Note, that both external interrupt groups (0-7 and 8-15)
 ** can be enabled individually.
 **
 ** If a single channel may be disabled and reenabled during runtime the
 ** functions Exint_DisableChannel() and Exint_EnableChannel() can be
 ** used.
 **
 ** Exint_SetDetectMode() is used to change the detect mode of single channel.
 ** And Exint_GetDetectMode gets the detect mode of single channel.
 ** 
 ** The external interrupts are disabled globally by Exint_DeInit().
 **
 ** Each channel provides an individually callback function which is handled
 ** in the interrupt service routine.
 **
 ** Exint_Nmi_Init() "initializes" the None-Maskable Interrupt, which means
 ** that the callback function is set to the dedicated internal data.
 **
 ******************************************************************************/
//@{
  
/******************************************************************************/
/* Global pre-processor symbols/macros ('#define')                            */
/******************************************************************************/
#define PDL_EXINT_CHANNELS       (32u)  
#define PDL_EXINT_INSTANCE_COUNT (uint8_t)(PDL_INTERRUPT_ENABLE_EXINT0  == PDL_ON) + \
                                 (uint8_t)(PDL_INTERRUPT_ENABLE_EXINT1  == PDL_ON) + \
                                 (uint8_t)(PDL_INTERRUPT_ENABLE_EXINT2  == PDL_ON) + \
                                 (uint8_t)(PDL_INTERRUPT_ENABLE_EXINT3  == PDL_ON) + \
                                 (uint8_t)(PDL_INTERRUPT_ENABLE_EXINT4  == PDL_ON) + \
                                 (uint8_t)(PDL_INTERRUPT_ENABLE_EXINT5  == PDL_ON) + \
                                 (uint8_t)(PDL_INTERRUPT_ENABLE_EXINT6  == PDL_ON) + \
                                 (uint8_t)(PDL_INTERRUPT_ENABLE_EXINT7  == PDL_ON) + \
                                 (uint8_t)(PDL_INTERRUPT_ENABLE_EXINT8  == PDL_ON) + \
                                 (uint8_t)(PDL_INTERRUPT_ENABLE_EXINT9  == PDL_ON) + \
                                 (uint8_t)(PDL_INTERRUPT_ENABLE_EXINT10 == PDL_ON) + \
                                 (uint8_t)(PDL_INTERRUPT_ENABLE_EXINT11 == PDL_ON) + \
                                 (uint8_t)(PDL_INTERRUPT_ENABLE_EXINT12 == PDL_ON) + \
                                 (uint8_t)(PDL_INTERRUPT_ENABLE_EXINT13 == PDL_ON) + \
                                 (uint8_t)(PDL_INTERRUPT_ENABLE_EXINT14 == PDL_ON) + \
                                 (uint8_t)(PDL_INTERRUPT_ENABLE_EXINT15 == PDL_ON) + \
                                 (uint8_t)(PDL_INTERRUPT_ENABLE_EXINT16 == PDL_ON) + \
                                 (uint8_t)(PDL_INTERRUPT_ENABLE_EXINT17 == PDL_ON) + \
                                 (uint8_t)(PDL_INTERRUPT_ENABLE_EXINT18 == PDL_ON) + \
                                 (uint8_t)(PDL_INTERRUPT_ENABLE_EXINT19 == PDL_ON) + \
                                 (uint8_t)(PDL_INTERRUPT_ENABLE_EXINT20 == PDL_ON) + \
                                 (uint8_t)(PDL_INTERRUPT_ENABLE_EXINT21 == PDL_ON) + \
                                 (uint8_t)(PDL_INTERRUPT_ENABLE_EXINT22 == PDL_ON) + \
                                 (uint8_t)(PDL_INTERRUPT_ENABLE_EXINT23 == PDL_ON) + \
                                 (uint8_t)(PDL_INTERRUPT_ENABLE_EXINT24 == PDL_ON) + \
                                 (uint8_t)(PDL_INTERRUPT_ENABLE_EXINT25 == PDL_ON) + \
                                 (uint8_t)(PDL_INTERRUPT_ENABLE_EXINT26 == PDL_ON) + \
                                 (uint8_t)(PDL_INTERRUPT_ENABLE_EXINT27 == PDL_ON) + \
                                 (uint8_t)(PDL_INTERRUPT_ENABLE_EXINT28 == PDL_ON) + \
                                 (uint8_t)(PDL_INTERRUPT_ENABLE_EXINT29 == PDL_ON) + \
                                 (uint8_t)(PDL_INTERRUPT_ENABLE_EXINT30 == PDL_ON) + \
                                 (uint8_t)(PDL_INTERRUPT_ENABLE_EXINT31 == PDL_ON) 

/******************************************************************************/
/* Local type definitions ('typedef')                                         */
/******************************************************************************/
  
typedef enum en_exint_level
{
  ExIntLowLevel     = 0u,  ///< "L" level detection
  ExIntHighLevel    = 1u,  ///< "H" level detection
  ExIntRisingEdge   = 2u,  ///< Rising edge detection
  ExIntFallingEdge  = 3u,  ///< Falling edge detection
#if (PDL_MCU_TYPE == PDL_FM4_TYPE5) ||  (PDL_MCU_TYPE == PDL_FM4_TYPE6) || (PDL_MCU_TYPE == PDL_FM0P_TYPE3)
  ExIntBothEdge     = 4u,  ///< Both edge detection  
#endif    
} en_exint_level_t;

/**
 ******************************************************************************
 ** \brief EXINT configuration
 ******************************************************************************/

#if (defined(PDL_PERIPHERAL_EXINT_ACTIVE))
/**
 *****************************************************************************
 ** \brief External Interrupt configuration
 ** 
 ** The EXINT configuration
 *****************************************************************************/
typedef struct stc_exint_config
{
  boolean_t         abEnable[PDL_EXINT_INSTANCE_COUNT];           ///< TRUE: External Interrupt enable
  en_exint_level_t  aenLevel[PDL_EXINT_INSTANCE_COUNT];           ///< level detection, see #en_exint_level_t for details
  func_ptr_t        apfnExintCallback[PDL_EXINT_INSTANCE_COUNT];  ///< Callback pointers  
  boolean_t         bTouchNvic;      ///< TRUE: Enable NVIC, FALSE: don't enable NVIC, 
  
} stc_exint_config_t;

/**
 ******************************************************************************
 ** \brief Datatype for holding internal data needed for EXINT
 ******************************************************************************/
typedef struct stc_exint_intern_data
{
  func_ptr_t    apfnExintCallback[PDL_EXINT_INSTANCE_COUNT];  ///< Callback pointer array
} stc_exint_intern_data_t ;
#endif

#if (defined(PDL_PERIPHERAL_NMI_ACTIVE))
/**
 *****************************************************************************
 ** \brief NMI configuration
 ** 
 ** The NMI configuration
 *****************************************************************************/
typedef struct stc_exint_nmi_config
{ 
  func_ptr_t        pfnNmiCallback;   ///< NMI Callback pointers
} stc_exint_nmi_config_t;

/**
 ******************************************************************************
 ** \brief Datatype for holding internal data needed for NMI
 ******************************************************************************/
typedef struct stc_exint_nmi_intern_data
{
  func_ptr_t    pfnNmiCallback;  ///< Callback pointer
} stc_exint_nmi_intern_data_t ;
#endif

/// Enumeration to define an index for each enabled external interrupt instance
typedef enum en_exint_instance_index
{
  #if (PDL_PERIPHERAL_ENABLE_EXINT0 == PDL_ON)
    ExintInstanceIndexExint0,
  #endif
  #if (PDL_PERIPHERAL_ENABLE_EXINT1 == PDL_ON)
    ExintInstanceIndexExint1,
  #endif
  #if (PDL_PERIPHERAL_ENABLE_EXINT2 == PDL_ON)
    ExintInstanceIndexExint2,
  #endif
  #if (PDL_PERIPHERAL_ENABLE_EXINT3 == PDL_ON)
    ExintInstanceIndexExint3,
  #endif
  #if (PDL_PERIPHERAL_ENABLE_EXINT4 == PDL_ON)
    ExintInstanceIndexExint4,
  #endif
  #if (PDL_PERIPHERAL_ENABLE_EXINT5 == PDL_ON)
    ExintInstanceIndexExint5,
  #endif
  #if (PDL_PERIPHERAL_ENABLE_EXINT6 == PDL_ON)
    ExintInstanceIndexExint6,
  #endif
  #if (PDL_PERIPHERAL_ENABLE_EXINT7 == PDL_ON)
    ExintInstanceIndexExint7,
  #endif
  #if (PDL_PERIPHERAL_ENABLE_EXINT8 == PDL_ON)
    ExintInstanceIndexExint8,
  #endif
  #if (PDL_PERIPHERAL_ENABLE_EXINT9 == PDL_ON)
    ExintInstanceIndexExint9,
  #endif
  #if (PDL_PERIPHERAL_ENABLE_EXINT10 == PDL_ON)
    ExintInstanceIndexExint10,
  #endif
  #if (PDL_PERIPHERAL_ENABLE_EXINT11 == PDL_ON)
    ExintInstanceIndexExint11,
  #endif
  #if (PDL_PERIPHERAL_ENABLE_EXINT12 == PDL_ON)
    ExintInstanceIndexExint12,
  #endif
  #if (PDL_PERIPHERAL_ENABLE_EXINT13 == PDL_ON)
    ExintInstanceIndexExint13,
  #endif
  #if (PDL_PERIPHERAL_ENABLE_EXINT14 == PDL_ON)
    ExintInstanceIndexExint14,
  #endif
  #if (PDL_PERIPHERAL_ENABLE_EXINT15 == PDL_ON)
    ExintInstanceIndexExint15,
  #endif
  #if (PDL_PERIPHERAL_ENABLE_EXINT16 == PDL_ON)
    ExintInstanceIndexExint16,
  #endif
  #if (PDL_PERIPHERAL_ENABLE_EXINT17 == PDL_ON)
    ExintInstanceIndexExint17,
  #endif
  #if (PDL_PERIPHERAL_ENABLE_EXINT18 == PDL_ON)
    ExintInstanceIndexExint18,
  #endif
  #if (PDL_PERIPHERAL_ENABLE_EXINT19 == PDL_ON)
    ExintInstanceIndexExint19,
  #endif
  #if (PDL_PERIPHERAL_ENABLE_EXINT20 == PDL_ON)
    ExintInstanceIndexExint20,
  #endif
  #if (PDL_PERIPHERAL_ENABLE_EXINT21 == PDL_ON)
    ExintInstanceIndexExint21,
  #endif
  #if (PDL_PERIPHERAL_ENABLE_EXINT22 == PDL_ON)
    ExintInstanceIndexExint22,
  #endif
  #if (PDL_PERIPHERAL_ENABLE_EXINT23 == PDL_ON)
    ExintInstanceIndexExint23,
  #endif
  #if (PDL_PERIPHERAL_ENABLE_EXINT24 == PDL_ON)
    ExintInstanceIndexExint24,
  #endif
  #if (PDL_PERIPHERAL_ENABLE_EXINT25 == PDL_ON)
    ExintInstanceIndexExint25,
  #endif
  #if (PDL_PERIPHERAL_ENABLE_EXINT26 == PDL_ON)
    ExintInstanceIndexExint26,
  #endif
  #if (PDL_PERIPHERAL_ENABLE_EXINT27 == PDL_ON)
    ExintInstanceIndexExint27,
  #endif
  #if (PDL_PERIPHERAL_ENABLE_EXINT28 == PDL_ON)
    ExintInstanceIndexExint28,
  #endif
  #if (PDL_PERIPHERAL_ENABLE_EXINT29 == PDL_ON)
    ExintInstanceIndexExint29,
  #endif
  #if (PDL_PERIPHERAL_ENABLE_EXINT30 == PDL_ON)
    ExintInstanceIndexExint30,
  #endif
  #if (PDL_PERIPHERAL_ENABLE_EXINT31 == PDL_ON)
    ExintInstanceIndexExint31,
  #endif      
    ExintInstanceIndexMax
} en_exint_instance_index_t;

/******************************************************************************/
/* Global variable definitions ('extern')                                     */
/******************************************************************************/
#if (defined(PDL_PERIPHERAL_EXINT_ACTIVE))
extern stc_exint_intern_data_t stcExintInternData;
#endif
#if (defined(PDL_PERIPHERAL_NMI_ACTIVE))
extern stc_exint_nmi_intern_data_t stcExintNMIInternData;
#endif
/******************************************************************************/
/* Global function prototypes (definition in C source)                        */
/******************************************************************************/
#if (defined(PDL_PERIPHERAL_EXINT_ACTIVE))
void Exint_IrqHandler(uint8_t u8Channel) ;

en_result_t Exint_Init(stc_exint_config_t* pstcConfig) ;

en_result_t Exint_DeInit(void) ;

en_result_t Exint_EnableChannel(uint8_t u8Channel) ;

en_result_t Exint_DisableChannel(uint8_t u8Channel) ;

en_result_t Exint_SetDetectMode(uint8_t u8Channel, en_exint_level_t enLevel);

en_exint_level_t Exint_GetDetectMode(uint8_t u8Channel);
#endif
#if (defined(PDL_PERIPHERAL_NMI_ACTIVE))
void Exint_Nmi_IrqHandler(void) ;

en_result_t Exint_Nmi_Init(stc_exint_nmi_config_t* pstcConfig) ;

en_result_t Exint_Nmi_DeInit(stc_exint_nmi_config_t* pstcConfig) ;
#endif 

//@} // ExintGroup

#ifdef __cplusplus
}
#endif

#endif // #if (defined(PDL_PERIPHERAL_EXINT_ACTIVE))

#endif /* __EXINT_H__ */
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
