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
/** \file pcrc.c
 **
 ** A detailed description is available at 
 ** @link PcrcGroup PCRC Module description @endlink
 **
 ** History:
 **   - 2015-02-11  1.0  DHo   First version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pcrc.h"

#if (defined(PDL_PERIPHERAL_PCRC_ACTIVE))

/**
 ******************************************************************************
 ** \addtogroup PcrcGroup
 ******************************************************************************/
//@{

/******************************************************************************/
/* Local pre-processor symbols/macros ('#define')                             */
/******************************************************************************/

/******************************************************************************/
/* Global variable definitions (declared in header file with 'extern')        */
/******************************************************************************/

/******************************************************************************/
/* Local type definitions ('typedef')                                         */
/******************************************************************************/

/******************************************************************************/
/* Local function prototypes ('static')                                       */
/******************************************************************************/
#if (PDL_INTERRUPT_ENABLE_PCRC == PDL_ON)
/**
 ******************************************************************************
 ** \brief Device dependent initialization of interrupts according CMSIS with
 **        level defined in pdl_user.h
 **
 ******************************************************************************/
static void PcrcInitNvic(void); 

/**
 ******************************************************************************
 ** \brief Device dependent de-initialization of interrupts according CMSIS 
 **
 ******************************************************************************/
static void PcrcDeInitNvic(void); 
#endif

/******************************************************************************/
/* Local variable definitions ('static')                                      */
/******************************************************************************/
#if (PDL_INTERRUPT_ENABLE_PCRC == PDL_ON)
static func_ptr_t m_pfnPcrcIrqCb = NULL;
#endif

/******************************************************************************/
/* Function implementation - global ('extern') and local ('static')           */
/******************************************************************************/

#if (PDL_INTERRUPT_ENABLE_PCRC == PDL_ON)
/**
 ******************************************************************************
 ** \brief Device dependent initialization of interrupts according CMSIS with
 **        level defined in pdl.h
 **
 ******************************************************************************/
static void PcrcInitNvic(void)
{    
    NVIC_ClearPendingIRQ(PRGCRC_I2S_IRQn);
    NVIC_EnableIRQ(PRGCRC_I2S_IRQn);
    NVIC_SetPriority(PRGCRC_I2S_IRQn, PDL_IRQ_LEVEL_I2S_PCRC);

    return;
} 

/**
 ******************************************************************************
 ** \brief Device dependent de-initialization of interrupts according CMSIS 
 **
 ******************************************************************************/
static void PcrcDeInitNvic(void)
{ 
    NVIC_ClearPendingIRQ(PRGCRC_I2S_IRQn);
    NVIC_DisableIRQ(PRGCRC_I2S_IRQn);
    NVIC_SetPriority(PRGCRC_I2S_IRQn, PDL_DEFAULT_INTERRUPT_LEVEL);

    return;
} 

/**
 ******************************************************************************
 ** \brief ISR of the PCRC module
 ** 
 ******************************************************************************/
void PcrcIrqHandler(void)
{    
    if(FM_PCRC->PRGCRC_CFG_f.CIRQ)
    {
        FM_PCRC->PRGCRC_CFG_f.CIRQCLR = 1u;
        if(NULL != m_pfnPcrcIrqCb)
        {
            m_pfnPcrcIrqCb(); 
        }
    }

    return;
}
#endif

/**
 ******************************************************************************
 ** \brief Initialisation of a Programmable CRC module.
 **
 ** \param pstcConfig    CRC module configuration 
 **
 ** \retval Ok                 Initializiation of Programmable CRC module successfully 
 **                            done.
 ** \retval ErrorInvalidParameter  If one of following cases match:
 **                                - pstcConfig == NULL,
 **                                - parameter out of range
 ******************************************************************************/
en_result_t Pcrc_Init(stc_pcrc_config_t* pstcConfig)
{  
    en_result_t enResult;
    stc_pcrc_prgcrc_cfg_field_t PRGCRC_CFG_f;

    PDL_ZERO_STRUCT(PRGCRC_CFG_f);

    /* Check for valid pointers and configrue parameters */
    if ((NULL == pstcConfig) ||
        (pstcConfig->enInputFormat > LsbFirstLittleEndian) ||
        (pstcConfig->enOutputFormat > LsbFirstLittleEndian) ||
        (pstcConfig->enInputDataSize > InputData32Bit))
    {
        enResult = ErrorInvalidParameter;
    }
    else
    {
        PRGCRC_CFG_f.FI = pstcConfig->enInputFormat;
        PRGCRC_CFG_f.FO = pstcConfig->enOutputFormat;
        PRGCRC_CFG_f.SZ = pstcConfig->enInputDataSize;
        PRGCRC_CFG_f.TEST = 0x20u;              //Always writes "100000" to perform write access to this register.

        /* Update hardware */
        FM_PCRC->PRGCRC_POLY = pstcConfig->u32GeneratorPolynomial;
        FM_PCRC->PRGCRC_SEED = pstcConfig->u32CrcInitValue;
        FM_PCRC->PRGCRC_FXOR = pstcConfig->u32FinalXorValue;
        FM_PCRC->PRGCRC_CFG_f = PRGCRC_CFG_f;
        FM_PCRC->PRGCRC_CFG_f.CDEN = ((pstcConfig->bUseDstc == TRUE) ? 1u : 0u);

#if (PDL_INTERRUPT_ENABLE_PCRC== PDL_ON)
        m_pfnPcrcIrqCb = pstcConfig->pfnIrqCb;
        FM_PCRC->PRGCRC_CFG_f.CIRQCLR = 1u;
        FM_PCRC->PRGCRC_CFG_f.CIEN = ((pstcConfig->bIrqEn == TRUE) ? 1u : 0u);
         // Initialize NVIC
        if(TRUE == pstcConfig->bTouchNvic)
        {
            PcrcInitNvic();
        }
#endif        
        enResult = Ok;
    }

    return (enResult);
}

/**
 ******************************************************************************
 ** \brief Re-Initialisation of a Programmable CRC module.
 ******************************************************************************/
void Pcrc_DeInit(void)
{
#if (PDL_INTERRUPT_ENABLE_PCRC == PDL_ON)
    PcrcDeInitNvic();
#endif
    /* clear hardware */
    FM_PCRC->PRGCRC_WR = 0u;
    FM_PCRC->PRGCRC_POLY = 0u;
    FM_PCRC->PRGCRC_SEED = 0u;
    FM_PCRC->PRGCRC_FXOR = 0u;
    FM_PCRC->PRGCRC_CFG = 0u;

    return;
} 

/**
 ******************************************************************************
 ** \brief Set CRC computing initial value.
 **
 ** \param  u32InitialValue: CRC initial value
 ** 
 ******************************************************************************/
void Pcrc_SetInitialValue(uint32_t u32InitialValue)
{
    FM_PCRC->PRGCRC_SEED = u32InitialValue;

    return;
}

/**
 ******************************************************************************
 ** \brief Set CRC computing result XOR value.
 **
 ** \param  u32FinalXorValue:  result XOR value
 ** 
 ******************************************************************************/
void Pcrc_SetFinalXorValue(uint32_t u32FinalXorValue)
{
    FM_PCRC->PRGCRC_FXOR = u32FinalXorValue;

    return;
}

/**
 ******************************************************************************
 ** \brief Set CRC output data Format.
 **
 ** \param  enOutputDataFormat:  CRC output data Format
 ** 
 ******************************************************************************/
void Pcrc_SetOutputDataFormat(en_crc_data_format_t enOutputDataFormat)
{
    FM_PCRC->PRGCRC_CFG_f.FO = enOutputDataFormat;

    return;
} 

/**
 ******************************************************************************
 ** \brief Set CRC input data Format.
 **
 ** \param  enInputDataFormat:  CRC input data Format
 ** 
 ******************************************************************************/
void Pcrc_SetInputDataFormat(en_crc_data_format_t enInputDataFormat)
{
    FM_PCRC->PRGCRC_CFG_f.FI = enInputDataFormat;

    return;
} 

/**
 ******************************************************************************
 ** \brief Set CRC input data size.
 **
 ** \param  enInputDataSize:  CRC input data size
 ** 
 ******************************************************************************/
void Pcrc_SetInputDataSize(en_crc_input_data_size_t enInputDataSize)
{
    FM_PCRC->PRGCRC_CFG_f.SZ = enInputDataSize;

    return;
} 

/**
 ******************************************************************************
 ** \brief Set  TEST field of CRC Computing Configuration Register 
 **
 ** \param  u8CfgTest:  TEST field value. 
 **                                      The correct operation is not achieved if perform writing values other 
 **                                      than "100000" to the TEST[5:0] register
 ** 
 ******************************************************************************/
void Pcrc_SetCfgTest(uint8_t u8CfgTest)
{
    FM_PCRC->PRGCRC_CFG_f.TEST = u8CfgTest;

    return;
} 

/**
 ******************************************************************************
 ** \brief Get CRC interrupt request flag.
 **
 ** \retval PRGCRC Interrupt request flag
 ** 
 ******************************************************************************/
uint8_t Pcrc_GetIntRequestFlag(void)
{
    return (FM_PCRC->PRGCRC_CFG_f.CIRQ);
} 

/**
 ******************************************************************************
 ** \brief Enable CRC interrupt request.
 ******************************************************************************/
void Pcrc_EnableIntRequest(void)
{
    FM_PCRC->PRGCRC_CFG_f.CIEN = 1u;

    return;
} 

/**
 ******************************************************************************
 ** \brief Disable CRC interrupt request.
 ******************************************************************************/
void Pcrc_DisableIntRequest(void)
{
    FM_PCRC->PRGCRC_CFG_f.CIEN = 0u;

    return;
}

/**
 ******************************************************************************
 ** \brief Allows the transfer request of input data for DMA.
 ******************************************************************************/
void Pcrc_EnableDmaTx(void)
{
    FM_PCRC->PRGCRC_CFG_f.CDEN = 1u;

    return;
} 

/**
 ******************************************************************************
 ** \brief Disable the transfer request of input data for DMA.
 ******************************************************************************/
void Pcrc_DisableDmaTx(void)
{
    FM_PCRC->PRGCRC_CFG_f.CDEN = 0u;

    return;
} 

/**
 ******************************************************************************
 ** \brief Get CRC computing unit state.
 **
 ** \retval  1 : Indicates the CRC computing unit is in busy state
 ** \retval  0 : Indicates the CRC computing unit is in idle state.
 ** 
 ******************************************************************************/
uint8_t Pcrc_GetLockStatus(void)
{
    return ((FM_PCRC->PRGCRC_CFG_f.LOCK == 1u) ? 1u : 0u);
} 

/**
 ******************************************************************************
 ** \brief Clear RGCRC Interrupt request.
 ******************************************************************************/
void Pcrc_ClrIntRequest(void)
{
    /* Caluculate CRC (Push 8bit data) */
    FM_PCRC->PRGCRC_CFG_f.CIRQCLR = 1u;

    return;
} 

/**
 ******************************************************************************
 ** \brief Write computing input data.
 **
 ** \param  u32WriteData: input data value
 **
 ******************************************************************************/
void Pcrc_WriteData(uint32_t u32WriteData)
{
    FM_PCRC->PRGCRC_WR = u32WriteData;

    return;
}

/**
 ******************************************************************************
 ** \brief Get computing output data.
 **
 ** \retval output data value
 ** 
 ******************************************************************************/
uint32_t Pcrc_ReadResult(void)
{
    return (FM_PCRC->PRGCRC_RD);
}

//@} // CrcGroup

#endif /* #if (defined(PDL_PERIPHERAL_PCRC_ACTIVE)) */
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
