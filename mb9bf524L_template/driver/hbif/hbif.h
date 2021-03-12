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
/*****************************************************************************/
/** \file hbif.h
 **
 ** HBIF API functions header file.
 **
 ** History:
 ** - 2015-01-28  0.1  EZh   First version for FM universal PDL
 **
 *****************************************************************************/

#ifndef __HBIF_H__
#define __HBIF_H__

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include "mcu.h"
#include "pdl_user.h"

#if (defined(PDL_PERIPHERAL_HBIF_ACTIVE))

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/*****************************************************************************/
/* Global pre-processor symbols/macros ('define')                            */
/*****************************************************************************/
  
/**
 *****************************************************************************
 ** \defgroup HbifGroup HyperBus Interface (HBIF)
 **
 ** \brief This section describes the interface for the HyperBus
 **
 ** Provided functions of HBIF module:
 ** - Hbif_Init()
 ** - Hbif_DeInit()
 ** - Hbif_GetStatus()
 ** - Hbif_GetInterruptFlag()   
 ** - Hbif_SetWriteProtection() 
 ** - Hbif_SetGpo0Level()
 ** - Hbif_SetGpo1Level()
 **
 ** Hbif_Init() initializes HBIF configuration. I2s_DeInitClk() resets the 
 ** HBIF registers.
 **  
 ** Hbif_GetStatus() gets the status of HyberBus during data transaction.
 **
 ** Hbif_GetInterruptFlag() gets the interrupt flag of HyberBus.
 **
 ** Hbif_SetWriteProtection() sets write protection to external Hyper Flash.
 **
 ** Hbif_SetGpo0Level() sets the level of GPO0 pin and Hbif_SetGpo1Level() sets
 ** the level of GPO1 pin.
 **  
 *****************************************************************************/
//@{

/******************************************************************************
 * Global definitions
 ******************************************************************************/

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/
/**
 *****************************************************************************
 ** \brief HyperBus interface status types
 *****************************************************************************/
typedef enum en_hbif_status
{
    HbifRstoDuringWrite           = 0u,  ///< RSTO Error in Write Transaction
    HbifTransactionErrDuringWrite = 1u,  ///< Transaction Error in Write Transaction
    HbifDecodeErrDuringWrite      = 2u,  ///< Decode Error in Write Transaction
    HbifWriteActive               = 3u,  ///< Write Transaction Active
    HbifRdsStallDuringRead        = 4u,  ///< RDS Stall Error in Read Transaction
    HbifRstoDuringRead            = 5u,  ///< RSTO Error in Read Transaction
    HbifTransactionErrDuringRead  = 6u,  ///< Transaction Error in Read Transaction
    HbifDecodeErrDuringRead       = 7u,  ///< Decode Error in Read Transaction
    HbifReadActive                = 8u,  ///< Read Transaction Active
    
}en_hbif_status_t;
   
/**
 *****************************************************************************
 ** \brief Wrap size types
 *****************************************************************************/
typedef enum en_hbif_wrap_size
{
    HbifWrap64Bytes = 1u, ///< Wrap bust length: 64 bytes
    HbifWrap16Bytes = 2u, ///< Wrap bust length: 16 bytes
    HbifWrap32Bytes = 3u, ///< Wrap bust length: 32 bytes
    
}en_hbif_wrap_size_t;
  
/**
 *****************************************************************************
 ** \brief Chip Select High Cycle Between Operations
 *****************************************************************************/
typedef enum en_hbif_cshi_cycle_t
{
    HbifCshi15Clk = 0u,   ///< Insert 1.5 clock to the chip select high period 
    HbifCshi25Clk = 1u,   ///< Insert 2.5 clock to the chip select high period 
    HbifCshi35Clk = 2u,   ///< Insert 3.5 clock to the chip select high period 
    HbifCshi45Clk = 3u,   ///< Insert 4.5 clock to the chip select high period 
    HbifCshi55Clk = 4u,   ///< Insert 5.5 clock to the chip select high period 
    HbifCshi65Clk = 5u,   ///< Insert 6.5 clock to the chip select high period
    HbifCshi75Clk = 6u,   ///< Insert 7.5 clock to the chip select high period
    HbifCshi85Clk = 7u,   ///< Insert 8.5 clock to the chip select high period
    HbifCshi95Clk = 8u,   ///< Insert 9.5 clock to the chip select high period
    HbifCshi105Clk = 9u,  ///< Insert 10.5 clock to the chip select high period
    HbifCshi115Clk = 10u, ///< Insert 11.5 clock to the chip select high period
    HbifCshi125Clk = 11u, ///< Insert 12.5 clock to the chip select high period
    HbifCshi135Clk = 12u, ///< Insert 13.5 clock to the chip select high period
    HbifCshi145Clk = 13u, ///< Insert 14.5 clock to the chip select high period
    HbifCshi155Clk = 14u, ///< Insert 15.5 clock to the chip select high period
    HbifCshi165Clk = 15u, ///< Insert 16.5 clock to the chip select high period
    
}en_hbif_rcshi_cycle_t, en_hbif_wcshi_cycle_t;

/**
 *****************************************************************************
 ** \brief  Chip Select Setup To Next CK Rising Edge
 *****************************************************************************/
typedef enum en_hbif_css_cycle_t
{  
    HbifCcs1Clk = 0u,    ///<  inserts the 1 cycles, between the falling edge of chip select and the rising edge of first CK.
    HbifCcs2Clk = 1u,    ///<  inserts the 2 cycles, between the falling edge of chip select and the rising edge of first CK.
    HbifCcs3Clk = 2u,    ///<  inserts the 3 cycles, between the falling edge of chip select and the rising edge of first CK.
    HbifCcs4Clk = 3u,    ///<  inserts the 4 cycles, between the falling edge of chip select and the rising edge of first CK.
    HbifCcs5Clk = 4u,    ///<  inserts the 5 cycles, between the falling edge of chip select and the rising edge of first CK.
    HbifCcs6Clk = 5u,    ///<  inserts the 6 cycles, between the falling edge of chip select and the rising edge of first CK.
    HbifCcs7Clk = 6u,    ///<  inserts the 7 cycles, between the falling edge of chip select and the rising edge of first CK.
    HbifCcs8Clk = 7u,    ///<  inserts the 8 cycles, between the falling edge of chip select and the rising edge of first CK.
    HbifCcs9Clk = 8u,    ///<  inserts the 9 cycles, between the falling edge of chip select and the rising edge of first CK.
    HbifCcs10Clk = 9u,   ///<  inserts the 10 cycles, between the falling edge of chip select and the rising edge of first CK.
    HbifCcs11Clk = 10u,  ///<  inserts the 11 cycles, between the falling edge of chip select and the rising edge of first CK.
    HbifCcs12Clk = 11u,  ///<  inserts the 12 cycles, between the falling edge of chip select and the rising edge of first CK.
    HbifCcs13Clk = 12u,  ///<  inserts the 13 cycles, between the falling edge of chip select and the rising edge of first CK.
    HbifCcs14Clk = 13u,  ///<  inserts the 14 cycles, between the falling edge of chip select and the rising edge of first CK.
    HbifCcs15Clk = 14u,  ///<  inserts the 15 cycles, between the falling edge of chip select and the rising edge of first CK.
    HbifCcs16Clk = 15u,  ///<  inserts the 16 cycles, between the falling edge of chip select and the rising edge of first CK.
    
}en_hbif_rcss_cycle_t, en_hbif_wcss_cycle_t;

/**
 *****************************************************************************
 ** \brief  Chip Select Setup To Next CK Rising Edge
 *****************************************************************************/
typedef enum en_hbif_csh_cycle_t
{
    HbifCsh1Clk = 0u,    ///<  inserts the 1 cycles, between the falling edge of last CK and the rising edge of chip select.
    HbifCsh2Clk = 1u,    ///<  inserts the 2 cycles, between the falling edge of last CK and the rising edge of chip select.
    HbifCsh3Clk = 2u,    ///<  inserts the 3 cycles, between the falling edge of last CK and the rising edge of chip select.
    HbifCsh4Clk = 3u,    ///<  inserts the 4 cycles, between the falling edge of last CK and the rising edge of chip select.
    HbifCsh5Clk = 4u,    ///<  inserts the 5 cycles, between the falling edge of last CK and the rising edge of chip select.
    HbifCsh6Clk = 5u,    ///<  inserts the 6 cycles, between the falling edge of last CK and the rising edge of chip select.
    HbifCsh7Clk = 6u,    ///<  inserts the 7 cycles, between the falling edge of last CK and the rising edge of chip select.
    HbifCsh8Clk = 7u,    ///<  inserts the 8 cycles, between the falling edge of last CK and the rising edge of chip select.
    HbifCsh9Clk = 8u,    ///<  inserts the 9 cycles, between the falling edge of last CK and the rising edge of chip select.
    HbifCsh10Clk = 9u,   ///<  inserts the 10 cycles, between the falling edge of last CK and the rising edge of chip select.
    HbifCsh11Clk = 10u,  ///<  inserts the 11 cycles, between the falling edge of last CK and the rising edge of chip select.
    HbifCsh12Clk = 11u,  ///<  inserts the 12 cycles, between the falling edge of last CK and the rising edge of chip select.
    HbifCsh13Clk = 12u,  ///<  inserts the 13 cycles, between the falling edge of last CK and the rising edge of chip select.
    HbifCsh14Clk = 13u,  ///<  inserts the 14 cycles, between the falling edge of last CK and the rising edge of chip select.
    HbifCsh15Clk = 14u,  ///<  inserts the 15 cycles, between the falling edge of last CK and the rising edge of chip select.
    HbifCsh16Clk = 15u,  ///<  inserts the 16 cycles, between the falling edge of last CK and the rising edge of chip select.
    
}en_hbif_rcsh_cycle_t, en_hbif_wcsh_cycle_t;

/**
 *****************************************************************************
 ** \brief   Latency Cycle for HyperRAM mode
 *****************************************************************************/
typedef enum en_hbif_ltcy_cycle
{
    HbRamLatency4Clk = 0u,
    HbRamLatency5Clk = 1u,
    HbRamLatency6Clk = 2u,
    
}en_hbif_ltcy_cycle_t;

/**
 *****************************************************************************
 ** \brief Hyber Flash memory conguration structure
 *****************************************************************************/
typedef struct stc_hbif_mem_config
{
    uint32_t                    u32MemBaseAddress;               ///< The base address for memory space 
    boolean_t                   bMergeContinuousRead;            ///< TRUE: wrap transaction and subsequent continuous transaction can be merged
                                                                 ///< FALSE: wrap transaction and subsequent continuous transaction can't be merged 
    boolean_t                   bSupportAsymmetryCache;          ///< TRUE: asymmetry cache system support
                                                                 ///< FALSE: no asymmetry cache system support 
    boolean_t                   bRegisterSpace;                  ///< TRUE: access register space, FALSE: access memory space
    boolean_t                   bHyperRamDevice;                 ///< TRUE: connect with HyperRAM memory, FALSE: connect with HyperFlash memory
    en_hbif_wrap_size_t         enWrapBustsize;                  ///< Wrapped burst size type 
    en_hbif_rcshi_cycle_t       enReadCsHighCycle;               ///< Before the read access, this setting inserts the CK cycles to the chip select high period.
    en_hbif_wcshi_cycle_t       enWriteCsHighCycle;              ///< Before the write access, this setting inserts the CK cycles to the chip select high period.
    en_hbif_rcss_cycle_t        enReadCsNextClkCycle;            ///< In the read access, this setting inserts the CK cycles, between the falling edge of chip select and the rising edge of first CK.
    en_hbif_wcss_cycle_t        enWriteCsNextClkCycle;           ///< In the write access, this setting inserts the CK cycles, between the falling edge of chip select and the rising edge of first CK.
    en_hbif_rcsh_cycle_t        enReadCsHoldCycle;               ///< In the read access, this setting inserts the CK cycles, between the falling edge of last CK and the rising edge of chip select.
    en_hbif_wcsh_cycle_t        enWriteCsHoldCycle;              ///< In the write access, this setting inserts the CK cycles, between the falling edge of last CK and the rising edge of chip select
    en_hbif_ltcy_cycle_t        enHyperRamLatencyCycle;          ///< Latency Cycle for HyperRAM mode, ignored when the connected device is HyperFlash
    boolean_t                   bGpoOutputLevel;                 ///< TRUE: output high from GPO; FALSE: output low from GPO
    
}stc_hbif_mem_config_t;

/**
 *****************************************************************************
 ** \brief I2S configuration
 *****************************************************************************/
typedef struct stc_hbif_config
{
    stc_hbif_mem_config_t       *pstcMem0Config;                  ///< Pointer to memory space 0 configuration
    stc_hbif_mem_config_t       *pstcMem1Config;                  ///< Pointer to memory space 1 configuration
        
#if (PDL_INTERRUPT_ENABLE_HBIF == PDL_ON) 
    boolean_t                   bEnableInterrupt;                 ///< TRUE: enable hyper bus interrupt, FALSE: disable interrupt
    boolean_t                   bInterruptPolarity;               ///< TRUE: IENOn signal is active high, FALSE: IENOn signal is active low
    func_ptr_t                  pfnIrqCb;                         ///< Pointer to interrupt callback structure
    boolean_t                   bTouchNvic;                       ///< TRUE: Enable NVIC, FALSE: disable NVIC
#endif    
    
} stc_hbif_config_t;

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/
en_result_t Hbif_Init(stc_hbif_config_t* pstcConfig) ;
en_result_t Hbif_DeInit(boolean_t bTouchNvic) ;
boolean_t Hbif_GetStatus(en_hbif_status_t enStatus);
boolean_t Hbif_GetInterruptFlag(void);
en_result_t Hbif_SetWriteProtection(boolean_t bEnableWp);
en_result_t Hbif_SetGpo0Level(boolean_t bLevel);
en_result_t Hbif_SetGpo1Level(boolean_t bLevel);

//@}
#ifdef __cplusplus
}
#endif

#endif

#endif /* __I2S_H__ */
/*****************************************************************************/
/* EOF (not truncated)                                                       */
/*****************************************************************************/
