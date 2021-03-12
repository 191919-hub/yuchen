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
/** \file i2sl.c
 **
 ** A detailed description is available at 
 ** @link I2slGroup I2S-Lite Module description @endlink
 **
 ** History:
 **   - 2015-01-05  0.1  EZh  First version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "i2sl.h"

#if (defined(PDL_PERIPHERAL_I2SL_ACTIVE))

/**
 ******************************************************************************
 ** \addtogroup I2slGroup
 ******************************************************************************/
//@{   

/******************************************************************************/
/* Local pre-processor symbols/macros ('#define')                             */
/******************************************************************************/


/******************************************************************************/
/* Global variable definitions (declared in header file with 'extern')        */
/******************************************************************************/

/// Look-up table for all enabled I2s instances and their internal data
stc_i2sl_instance_data_t m_astcI2slInstanceDataLut[I2SL_INSTANCE_COUNT] =
{
#if (PDL_PERIPHERAL_ENABLE_I2SL0 == PDL_ON)
    {
        &I2SL0,
        &I2SLREG0,
        0u,
    },
#endif
#if (PDL_PERIPHERAL_ENABLE_I2SL1 == PDL_ON)
    {
        &I2SL1,
        &I2SLREG1,
        0u,
    },
#endif
};

/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/


/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/

/******************************************************************************/
/* Local Functions                                                            */
/******************************************************************************/

/**
 ******************************************************************************
 ** \brief Return the internal data for a certain i2s instance.
 **
 ** \param pstcI2sl Pointer to i2s instance
 **
 ** \return Pointer to internal data or NULL if instance is not enabled (or not known)
 **
 ******************************************************************************/
static stc_i2sl_intern_data_t* I2slGetInternDataPtr(volatile stc_i2sln_t* pstcI2sl) 
{
    uint8_t u8Instance;
   
    for (u8Instance = 0u; u8Instance < I2SL_INSTANCE_COUNT; u8Instance++)
    {
        if (pstcI2sl == m_astcI2slInstanceDataLut[u8Instance].pstcInstance)
        {
            return &m_astcI2slInstanceDataLut[u8Instance].stcInternData;
        }
    }

    return NULL;
}

/**
 ******************************************************************************
 ** \brief Return the internal data for a certain i2s register instance.
 **
 ** \param pstcI2sl Pointer to i2s instance
 **
 ** \return Pointer to internal data or NULL if instance is not enabled (or not known)
 **
 ******************************************************************************/
static volatile stc_i2slregn_t* I2slGetRegPtr(volatile stc_i2sln_t* pstcI2sl) 
{
    uint8_t u8Instance;
   
    for (u8Instance = 0u; u8Instance < I2SL_INSTANCE_COUNT; u8Instance++)
    {
        if (pstcI2sl == m_astcI2slInstanceDataLut[u8Instance].pstcInstance)
        {
            return m_astcI2slInstanceDataLut[u8Instance].pstcInstanceReg;
        }
    }

    return NULL;
}

#if (PDL_INTERRUPT_ENABLE_I2SL0 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_I2SL1 == PDL_ON)

/**
 ******************************************************************************
 ** \brief i2s instance transmiting interrupt service routine
 **
 ** I2S instance interrupt service routine, clear interrupt cause and
 **
 ** \param  pstcI2sl              Pointer to I2S instance
 ** \param  pstcI2slInternData    Pointer to I2S intern data
 **        
 ******************************************************************************/
void I2sl_TxIrqHandler( volatile stc_i2sln_t*   pstcI2sl,
                        stc_i2sl_intern_data_t* pstcI2slInternData)
{
    if ((NULL != pstcI2sl) && (NULL != pstcI2slInternData))
    {
        if(1u == pstcI2sl->FCR1_f.FDRQ)
        {
            if (NULL != pstcI2slInternData->pfnTxFifoIrqCb)
            {
                pstcI2slInternData->pfnTxFifoIrqCb();
            }
            
            pstcI2sl->FCR1_f.FDRQ = 0u;
        }
    }
    return;
} 

/**
 ******************************************************************************
 ** \brief i2s instance receiving interrupt service routine
 **
 ** I2S instance interrupt service routine, clear interrupt cause and
 **
 ** \param  pstcI2sl              Pointer to I2S instance
 ** \param  pstcI2slInternData    Pointer to I2S intern data
 **        
 ******************************************************************************/
void I2sl_RxIrqHandler( volatile stc_i2sln_t*   pstcI2sl,
                        stc_i2sl_intern_data_t* pstcI2slInternData)
{
    if ((NULL != pstcI2sl) && (NULL != pstcI2slInternData))
    {
        if(1u == pstcI2sl->SSR_f.ORE)
        {
            if (NULL != pstcI2slInternData->pfnRxFifoOverrunIrqCb)
            {
                pstcI2slInternData->pfnRxFifoOverrunIrqCb();
            }
            
            pstcI2sl->SSR_f.REC = 1u; // Clear error
        }
        else if (1u == pstcI2sl->SSR_f.RDRF)
        {
            if (NULL != pstcI2slInternData->pfnRxFifoIrqCb)
            {
                pstcI2slInternData->pfnRxFifoIrqCb();
            }
        }
        
    }
}

/**
 ******************************************************************************
 ** \brief Set NVIC Interrupt depending on I2S instance
 **
 ** \param pstcI2sl Pointer to I2S instance
 ** \param bRxIrq   TRUE: RX interrupt, FALSE: TX interrupt
 **
 ******************************************************************************/
static void I2slInitNvic(volatile stc_i2sln_t* pstcI2sl, boolean_t  bRxIrq)
{
#if (PDL_INTERRUPT_ENABLE_I2SL0 == PDL_ON)  
    #if (PDL_MCU_CORE == PDL_FM0P_CORE)
    #if (PDL_MCU_TYPE == PDL_FM0P_TYPE3)
    NVIC_ClearPendingIRQ(MFS4_IRQn);
    NVIC_EnableIRQ(MFS4_IRQn);
    NVIC_SetPriority(MFS4_IRQn, PDL_IRQ_LEVEL_MFS4);   
    #else
    if((volatile stc_i2sln_t*)(&I2SL0) == pstcI2sl)
    {
        if(TRUE == bRxIrq)
        {
            NVIC_ClearPendingIRQ(MFS5_13_RX_IRQn);
            NVIC_EnableIRQ(MFS5_13_RX_IRQn);
            NVIC_SetPriority(MFS5_13_RX_IRQn, PDL_IRQ_LEVEL_MFS5_13_RX);   
        }
        else
        {
            NVIC_ClearPendingIRQ(MFS5_13_TX_IRQn);
            NVIC_EnableIRQ(MFS5_13_TX_IRQn);
            NVIC_SetPriority(MFS5_13_TX_IRQn, PDL_IRQ_LEVEL_MFS5_13_TX); 
        }
    }
    #endif
    #elif (PDL_MCU_CORE == PDL_FM4_CORE)
    #if (PDL_MCU_TYPE == PDL_FM4_TYPE5)  
    if((volatile stc_i2sln_t*)(&I2SL0) == pstcI2sl)
    {
        if(TRUE == bRxIrq)
        {
            NVIC_ClearPendingIRQ(MFS1_RX_IRQn);
            NVIC_EnableIRQ(MFS1_RX_IRQn);
            NVIC_SetPriority(MFS1_RX_IRQn, PDL_IRQ_LEVEL_MFS1_RX);   
        }
        else
        {
            NVIC_ClearPendingIRQ(MFS1_TX_IRQn);
            NVIC_EnableIRQ(MFS1_TX_IRQn);
            NVIC_SetPriority(MFS1_TX_IRQn, PDL_IRQ_LEVEL_MFS1_TX); 
        }
    }
    #endif
    #endif
#endif    
#if (PDL_INTERRUPT_ENABLE_I2SL1 == PDL_ON)  
    #if (PDL_MCU_CORE == PDL_FM0P_CORE)
    #if (PDL_MCU_TYPE == PDL_FM0P_TYPE3)
    NVIC_ClearPendingIRQ(MFS6_I2CS_IRQn);
    NVIC_EnableIRQ(MFS6_I2CS_IRQn);
    NVIC_SetPriority(MFS6_I2CS_IRQn, PDL_IRQ_LEVEL_MFS6_I2CS0);
    #else
    if ((volatile stc_i2sln_t*)(&I2SL1) == pstcI2sl)
    {
        if(TRUE == bRxIrq)
        {
            NVIC_ClearPendingIRQ(MFS6_14_RX_DMA0_IRQn);
            NVIC_EnableIRQ(MFS6_14_RX_DMA0_IRQn);
            NVIC_SetPriority(MFS6_14_RX_DMA0_IRQn, PDL_IRQ_LEVEL_MFS6_14_RX_DMA0);   
        }
        else
        {
            NVIC_ClearPendingIRQ(MFS6_14_TX_DMA1_IRQn);
            NVIC_EnableIRQ(MFS6_14_TX_DMA1_IRQn);
            NVIC_SetPriority(MFS6_14_TX_DMA1_IRQn, PDL_IRQ_LEVEL_MFS6_14_TX_DMA1); 
        }
    }
#endif 
#endif 
#endif 
}
/**
 ******************************************************************************
 ** \brief Clear NVIC Interrupt depending on I2S instance
 **
 ** \param pstcI2sl Pointer to I2S instance
 ** \param bRxIrq   TRUE: RX interrupt, FALSE: TX interrupt
 **
 ******************************************************************************/
static void I2slDeInitNvic(volatile stc_i2sln_t* pstcI2sl, boolean_t  bRxIrq)
{
    /* Only when all of I2S interrupt sources are disable, disable I2S IRQ */
#if (PDL_INTERRUPT_ENABLE_I2SL0 == PDL_ON) 
    #if (PDL_MCU_CORE == PDL_FM0P_CORE)
    #if (PDL_MCU_TYPE == PDL_FM0P_TYPE3)
    if ((volatile stc_i2sln_t*)(&I2SL0) == pstcI2sl)
    {
        NVIC_ClearPendingIRQ(MFS4_IRQn);
        NVIC_DisableIRQ(MFS4_IRQn);
        NVIC_SetPriority(MFS4_IRQn, PDL_DEFAULT_INTERRUPT_LEVEL);  
    }
    #else
    if ((volatile stc_i2sln_t*)(&I2SL0) == pstcI2sl)
    {
        if(TRUE == bRxIrq)
        {
            NVIC_ClearPendingIRQ(MFS5_13_RX_IRQn);
            NVIC_DisableIRQ(MFS5_13_RX_IRQn);
            NVIC_SetPriority(MFS5_13_RX_IRQn, PDL_DEFAULT_INTERRUPT_LEVEL);   
        }
        else
        {
            NVIC_ClearPendingIRQ(MFS5_13_TX_IRQn);
            NVIC_DisableIRQ(MFS5_13_TX_IRQn);
            NVIC_SetPriority(MFS5_13_TX_IRQn, PDL_DEFAULT_INTERRUPT_LEVEL);   
        }
    }
    #endif
    #elif (PDL_MCU_CORE == PDL_FM4_CORE)
    #if (PDL_MCU_TYPE == PDL_FM4_TYPE5) 
    if ((volatile stc_i2sln_t*)(&I2SL0) == pstcI2sl)
    {
        if(TRUE == bRxIrq)
        {
            NVIC_ClearPendingIRQ(MFS1_RX_IRQn);
            NVIC_DisableIRQ(MFS1_RX_IRQn);
            NVIC_SetPriority(MFS1_RX_IRQn, PDL_DEFAULT_INTERRUPT_LEVEL);   
        }
        else
        {
            NVIC_ClearPendingIRQ(MFS1_TX_IRQn);
            NVIC_DisableIRQ(MFS1_TX_IRQn);
            NVIC_SetPriority(MFS1_TX_IRQn, PDL_DEFAULT_INTERRUPT_LEVEL);   
        }
    }
    #endif
    #endif
#endif 
#if (PDL_INTERRUPT_ENABLE_I2SL1 == PDL_ON) 
    #if (PDL_MCU_CORE == PDL_FM0P_CORE)
    #if (PDL_MCU_TYPE == PDL_FM0P_TYPE3)
    if ((volatile stc_i2sln_t*)(&I2SL1) == pstcI2sl)
    {
        NVIC_ClearPendingIRQ(MFS6_I2CS_IRQn);
        NVIC_DisableIRQ(MFS6_I2CS_IRQn);
        NVIC_SetPriority(MFS6_I2CS_IRQn, PDL_DEFAULT_INTERRUPT_LEVEL);  
    }
    #else
    if ((volatile stc_i2sln_t*)(&I2SL1) == pstcI2sl)
    {
        if(TRUE == bRxIrq)
        {
            NVIC_ClearPendingIRQ(MFS6_14_RX_DMA0_IRQn);
            NVIC_DisableIRQ(MFS6_14_RX_DMA0_IRQn);
            NVIC_SetPriority(MFS6_14_RX_DMA0_IRQn, PDL_DEFAULT_INTERRUPT_LEVEL);   
        }
        else
        {
            NVIC_ClearPendingIRQ(MFS6_14_TX_DMA1_IRQn);
            NVIC_DisableIRQ(MFS6_14_TX_DMA1_IRQn);
            NVIC_SetPriority(MFS6_14_TX_DMA1_IRQn, PDL_DEFAULT_INTERRUPT_LEVEL);   
        }
    }
#endif 
#endif 
#endif 
}


/**
 ******************************************************************************
 ** \brief Enable I2S interrupts
 **
 ** \param [in] pstcI2sl   Pointer to I2S instance   
 ** \param [in] enIrqSel   I2S interrupt type
 ** 
 ** \retval Ok                    Interrupt has been enabled 
 ** \retval ErrorInvalidParameter If one of following conditions are met:
 **                               - pstcI2sl == NULL
 **                               - Other invalid configuration                   
 ** 
 ******************************************************************************/
en_result_t I2sl_EnableIrq(volatile stc_i2sln_t* pstcI2sl, 
                           en_i2sl_irq_sel_t enIrqSel)
{    
    if (NULL == pstcI2sl)
    {
        return (ErrorInvalidParameter);
    }
    
    // Enable interrupt
    switch (enIrqSel)
    {
        case I2slRxFifoIrq:
            pstcI2sl->SCR_f.RIE = 1u;
            break;
        case I2slTxFifoIrq:
            pstcI2sl->FCR1_f.FTIE = 1u;
            break;
        default:
            return (ErrorInvalidParameter);
    }
    
    return Ok;
}

/**
 ******************************************************************************
 ** \brief Disable I2S interrupt
 **
 ** \param [in] pstcI2sl    Pointer to I2S instance   
 ** \param [in] enIrqSel    I2S interrupt type
 ** 
 ** \retval Ok                    Interrupt has been disabled
 ** \retval ErrorInvalidParameter If one of following conditions are met:
 **                               - pstcI2slo == NULL
 **                               - pstcInt == NULL                
 ** 
 ******************************************************************************/
en_result_t I2sl_DisableIrq(volatile stc_i2sln_t* pstcI2sl, 
                            en_i2sl_irq_sel_t enIrqSel)
{    
    if (NULL == pstcI2sl)
    {
        return (ErrorInvalidParameter);
    }
    
    // Enable interrupt
    switch (enIrqSel)
    {
        case I2slRxFifoIrq:
            pstcI2sl->SCR_f.RIE = 0u;
            break;
        case I2slTxFifoIrq:
            pstcI2sl->FCR1_f.FTIE = 0u;
            break;
        default:
            return (ErrorInvalidParameter);
    }
    
    return Ok;
}

#endif // #if (PDL_INTERRUPT_ENABLE_I2SL0 == PDL_ON) || (PDL_INTERRUPT_ENABLE_I2SL1 == PDL_ON)

/******************************************************************************/
/* Global Functions                                                           */
/******************************************************************************/
/**
 ******************************************************************************
 ** \brief Initialize I2S
 **
 ** This function initializes an I2S module
 **
 ** \param [in]  pstcI2sl          I2S instance
 ** \param [in]  pstcConfig        I2S module configuration 
 **
 ** \retval Ok                    I2S initialized normally
 ** \retval ErrorInvalidParameter if one of following cases matchs:
 **                               - pstcI2sl == NULL
 **                               - pstcConfig == NULL
 **                               - Other invalid configuration
 ******************************************************************************/
en_result_t I2sl_Init( volatile stc_i2sln_t* pstcI2sl,
                       stc_i2sl_config_t*    pstcConfig )
{    
    stc_i2sl_intern_data_t* pstcI2slInternData;
    volatile stc_i2slregn_t*         pstcI2sReg;
    
    /* Check for valid pointer and get pointer to internal data struct ... */
    pstcI2slInternData = I2slGetInternDataPtr(pstcI2sl);
    pstcI2sReg = I2slGetRegPtr(pstcI2sl);
    
    if((NULL == pstcConfig) || (NULL == pstcI2slInternData) || (NULL == pstcI2sReg))
    {
        return ErrorInvalidParameter;
    }
    
    // Enable I2S mode
    pstcI2sReg->CNTLREG_f.I2SEN = 1u;
    
    // Configure I2S input clock
    switch (pstcConfig->enClkIn)
    {
        case I2slUsePclk:
            pstcI2sReg->I2SCLK_f.MCKIE = 0u;
            break;
        case I2slUseExtClk:
            pstcI2sReg->I2SCLK_f.MCKIE = 1u;
            break;
        default:
            return ErrorInvalidParameter;
    }
    
    pstcI2sReg->I2SCLK_f.I2SDIV = pstcConfig->u8ClkInDiv;
    
    // Set I2S clock output or not
    pstcI2sReg->I2SCLK_f.MCKOE = ((pstcConfig->bClkOutputEn == TRUE) ? 1u : 0u);
    
    // Set I2S transfer mode
    switch (pstcConfig->enTransferMode)
    {
        case I2slPhilipsMode:
            pstcI2sReg->CNTLREG_f.I2SMOD = 0u;
            break;
        case I2slMsbJustifiedMode:
            pstcI2sReg->CNTLREG_f.I2SMOD = 1u;
            break;
        default:
            return ErrorInvalidParameter;
    }
    
    // Set I2SWS polarity
    pstcI2sReg->CNTLREG_f.FSPL = ((pstcConfig->bI2sWsPolarity == TRUE) ? 1u : 0u);
    
    // Set i2SWS and I2SCK pins output or not
    pstcI2sReg->CNTLREG_f.CKOE = ((pstcConfig->bCkWsOutputEn == TRUE) ? 1u : 0u);
    
    // Set mask bit replacing invalid frame
    pstcI2sReg->CNTLREG_f.MSKB = ((pstcConfig->bMaskBitOutput == TRUE) ? 1u : 0u);
    
    // Set corresponding MFS ch with CSIO mode, slave mode
    pstcI2sl->SMR_f.MD = 2u; 
    pstcI2sl->SCR_f.MS = 1u;
    
    // Set data length and frame length
    switch (pstcConfig->enFrameLength)
    {
        case I2slFrame16Bit:
            pstcI2sReg->CNTLREG_f.FRAML = 0u;
            break;
        case I2slFrame32Bit:
            pstcI2sReg->CNTLREG_f.FRAML = 1u;
            break;
        default:
            return ErrorInvalidParameter;
    }
    
    pstcI2sl->ESCR_f.L = 7u;
    pstcI2sl->ESCR_f.L3 = 1u;
       
    // Data direction is always MSB in I2S mode
    pstcI2sl->SMR_f.BDS = TRUE;   
    
    // Enable SOE 
    pstcI2sl->SMR_f.SOE = TRUE;  
        
    // Enable receiving or transmiting
    if(TRUE == pstcConfig->bDataOutput)
    {
        pstcI2sl->SCR_f.RXE = 0u;
        pstcI2sl->SCR_f.TXE = 1u;
    }
    else
    {
        pstcI2sl->SCR_f.RXE = 1u;
        pstcI2sl->SCR_f.TXE = 0u;
    }
    
    // Reset FIFO 
    pstcI2sl->FCR0_f.FCL1 = 1u;
    pstcI2sl->FCR0_f.FCL2 = 1u;
        
    // Selection TX and RX FIFO
    if(pstcConfig->enFifoSel == I2slTxFifo1RxFifo2)
    {
        pstcI2sl->FCR1_f.FSEL = 0u;
    }
    else
    {
        pstcI2sl->FCR1_f.FSEL = 1u;
    }
    
    // Set FIFO access length
    switch (pstcConfig->enFifoWidth)
    {
        case I2slFifoWidth16Bit:
            pstcI2sl->SSR_f.AWC = 0u;
            break;
        case I2slFifoWidth32Bit:
            pstcI2sl->SSR_f.AWC = 1u;
            break;
        default:
            return ErrorInvalidParameter;
    }
    
    // Set FIFO count 
    pstcI2sl->FBYTE1 = pstcConfig->u8ByteCount1;
    pstcI2sl->FBYTE2 = pstcConfig->u8ByteCount2;
    
    // Enable FIFO  
    pstcI2sl->FCR0_f.FE1 = 1u;
    pstcI2sl->FCR0_f.FE2 = 1u;

#if (PDL_INTERRUPT_ENABLE_I2SL0 == PDL_ON) || (PDL_INTERRUPT_ENABLE_I2SL1 == PDL_ON)    
    // Configure interrupts 
    if(NULL != pstcConfig->pstcI2slIrqEn)
    {
        if(TRUE == pstcConfig->pstcI2slIrqEn->bTxFifoIrq)
        {
            pstcI2sl->FCR1_f.FTIE = 1u;
        }
        
        if(TRUE == pstcConfig->pstcI2slIrqEn->bRxFifoIrq)
        {
            pstcI2sl->SCR_f.RIE = 1u;
        }
    }
    
    // Configure interrupt callbacks
    if(NULL != pstcConfig->pstcI2slIrqCb)
    {
        pstcI2slInternData->pfnTxFifoIrqCb = pstcConfig->pstcI2slIrqCb->pfnTxFifoIrqCb;
        pstcI2slInternData->pfnRxFifoIrqCb = pstcConfig->pstcI2slIrqCb->pfnRxFifoIrqCb;
        pstcI2slInternData->pfnRxFifoOverrunIrqCb = pstcConfig->pstcI2slIrqCb->pfnRxFifoOverrunIrqCb;
    }
    
    if(TRUE == pstcConfig->bTouchNvic)
    {
        I2slInitNvic(pstcI2sl, TRUE);
        I2slInitNvic(pstcI2sl, FALSE);
    }
#endif 
    
    // Generate software reset
    pstcI2sReg->I2SRST = 0xA5u;
    
    return Ok;
}

/**
 ******************************************************************************
 ** \brief Deinitialisation I2S
 **
 ** This function initializes an I2S module
 **
 ** \param [in]  pstcI2sl          I2S instance
 ** \param [in]  bTouchNvic        Touch NVIC or not
 **
 ** \retval Ok                    I2S initialized normally
 ** \retval ErrorInvalidParameter if one of following cases matchs:
 **                               - pstcI2sl == NULL
 **                               - Other invalid configuration
 ******************************************************************************/
en_result_t I2sl_DeInit( volatile stc_i2sln_t* pstcI2sl, boolean_t bTouchNvic)
{
    volatile stc_i2slregn_t*         pstcI2sReg;
    
    /* Check for valid pointer and get pointer to internal data struct ... */
    pstcI2sReg = I2slGetRegPtr(pstcI2sl);
  
    if((NULL == pstcI2sl) || (NULL == pstcI2sReg))
    {
        return ErrorInvalidParameter;
    }
    
    // First of all, stop I2S 
    pstcI2sReg->CNTLREG = 0u;
    
    // Serial Mode Register clear all bits (valid in any mode) 
    pstcI2sl->SMR = 0u;
       
    // Reset Mfs receive and transmit bit to default 
    // and clear all error flags 
    pstcI2sl->SCR = 0u;

    // Clear reception Errors 
    pstcI2sl->SSR_f.REC = TRUE;
       
    // Reset all other used register to default value 
    pstcI2sl->SSR  = 0u;
    pstcI2sl->ESCR = 0u;
    pstcI2sl->FCR0 = 0u;
    pstcI2sl->FCR1 = 0u;

#if (PDL_INTERRUPT_ENABLE_I2SL0 == PDL_ON) || (PDL_INTERRUPT_ENABLE_I2SL1 == PDL_ON)        
    if (TRUE == bTouchNvic)
    {
        I2slDeInitNvic(pstcI2sl, TRUE);
        I2slDeInitNvic(pstcI2sl, FALSE);
    }
#endif    
            
    return Ok;
}

/**
 ******************************************************************************
 ** \brief start I2S, enable i2s clock generate
 **
 ** \param [in] pstcI2sl     Pointer to I2SL instance   
 **
 ** \retval Ok Error is cleared 
 ** \retval ErrorInvalidParameter pstcI2sl = NULL
 ** 
 ******************************************************************************/
en_result_t I2sl_Start(volatile stc_i2sln_t* pstcI2sl)
{
    volatile stc_i2slregn_t*         pstcI2sReg;
    
    /* Check for valid pointer and get pointer to internal data struct ... */
    pstcI2sReg = I2slGetRegPtr(pstcI2sl);
  
    if((NULL == pstcI2sl) || (NULL == pstcI2sReg))
    {
        return ErrorInvalidParameter;
    }
  
    pstcI2sReg->CNTLREG_f.I2SRUN = 1u;
    
    return Ok;
}

/**
 ******************************************************************************
 ** \brief stop I2S, disable i2s clock generate
 **
 ** \param [in] pstcI2sl     Pointer to I2SL instance   
 **
 ** \retval Ok Error is cleared 
 ** \retval ErrorInvalidParameter pstcI2sl = NULL
 ** 
 ******************************************************************************/
en_result_t I2sl_Stop(volatile stc_i2sln_t* pstcI2sl)
{
    volatile stc_i2slregn_t*         pstcI2sReg;
    
    /* Check for valid pointer and get pointer to internal data struct ... */
    pstcI2sReg = I2slGetRegPtr(pstcI2sl);
  
    if((NULL == pstcI2sl) || (NULL == pstcI2sReg))
    {
        return ErrorInvalidParameter;
    }
    
    pstcI2sReg->CNTLREG_f.I2SRUN = 0u;
    
    return Ok;
}

/**
 ******************************************************************************
 ** \brief Enable I2S Clock out
 **
 ** \param [in] pstcI2sl     Pointer to I2SL instance   
 **
 ** \retval Ok Error is cleared 
 ** \retval ErrorInvalidParameter pstcI2sl = NULL
 ** 
 ******************************************************************************/
en_result_t I2sl_EnableClockOut(volatile stc_i2sln_t* pstcI2sl)
{
    volatile stc_i2slregn_t*         pstcI2sReg;
    
    /* Check for valid pointer and get pointer to internal data struct ... */
    pstcI2sReg = I2slGetRegPtr(pstcI2sl);
  
    if((NULL == pstcI2sl) || (NULL == pstcI2sReg))
    {
        return ErrorInvalidParameter;
    }
    
    pstcI2sReg->I2SCLK_f.MCKOE = 1;
    
    return Ok;
}

/**
 ******************************************************************************
 ** \brief Disable I2S Clock out
 **
 ** \param [in] pstcI2sl     Pointer to I2SL instance   
 **
 ** \retval Ok Error is cleared 
 ** \retval ErrorInvalidParameter pstcI2sl = NULL
 ** 
 ******************************************************************************/
en_result_t I2sl_DisableClockOut(volatile stc_i2sln_t* pstcI2sl)
{
    volatile stc_i2slregn_t*         pstcI2sReg;
    
    /* Check for valid pointer and get pointer to internal data struct ... */
    pstcI2sReg = I2slGetRegPtr(pstcI2sl);
  
    if((NULL == pstcI2sl) || (NULL == pstcI2sReg))
    {
        return ErrorInvalidParameter;
    }
    
    pstcI2sReg->I2SCLK_f.MCKOE = 0;
    
    return Ok;
}

/**
 ******************************************************************************
 ** \brief Get status of I2S-Lite according to status type
 **
 ** \param [in] pstcI2sl     Pointer to I2SL instance   
 ** \param [in] enStatus     I2SL status type
 ** \arg   I2slRxFifoFull    I2SL receive FIFO full
 ** \arg   I2slTxFifoEmpty   I2SL tranfer FIFO empty
 ** \arg   I2slRxFifoOverrun I2SL receive FIFO overrun
 ** \arg   I2slBusBusy       I2SL bus status
 ** \arg   I2slClockStatus   I2SL clock status
 ** 
 ** \retval FALSE                 If one of following conditions are met:  
 **                               - I2SL receive FIFO data count don't meet with FBYTE [enStatus = I2slRxFifoFull]
 **                               - I2SL tranfer FIFO is not empty [enStatus = I2slTxFifoEmpty]
 **                               - I2SL receive FIFO don't overrun [enStatus = UartTxIdle]
 **                               - I2SL bus idle [enStatus = I2slBusBusy] 
 **                               - I2SL clock stops [enStatus = I2slClockStatus] 
 ** \retval TRUE                 If one of following conditions are met:  
 **                               - I2SL receive FIFO data count meet with FBYTE [enStatus = I2slRxFifoFull]
 **                               - I2SL tranfer FIFO is empty [enStatus = I2slTxFifoEmpty]
 **                               - I2SL receive FIFO overruns [enStatus = UartTxIdle]
 **                               - I2SL bus busy [enStatus = I2slBusBusy] 
 **                               - I2SL clock is running [enStatus = I2slClockStatus]  
 ** 
 ******************************************************************************/
boolean_t I2sl_GetStatus(volatile stc_i2sln_t* pstcI2sl, 
                         en_i2sl_status_t enStatus)
{
    volatile stc_i2slregn_t*         pstcI2sReg;
    boolean_t bResult = FALSE;
    
    /* Check for valid pointer and get pointer to internal data struct ... */
    pstcI2sReg = I2slGetRegPtr(pstcI2sl);
    
    switch(enStatus)
    {
        case I2slRxFifoOverrun:
            bResult = (1u == pstcI2sl->SSR_f.ORE) ? TRUE : FALSE;
            break;  
        case I2slRxFifoFull:
            bResult = (1u == pstcI2sl->SSR_f.RDRF) ? TRUE : FALSE;
            break;  
        case I2slTxFifoEmpty:
            bResult = (1u == pstcI2sl->FCR1_f.FDRQ) ? TRUE : FALSE;
            break;  
        case I2slBusBusy:
            bResult = (1u == pstcI2sReg->I2SST_f.BUSY) ? TRUE : FALSE;
            break;  
        case I2slClockStatus:
            bResult = (1u == pstcI2sReg->I2SST_f.CKSTP) ? TRUE : FALSE;
            break;  
        default:
            break;  
    }
    
    return bResult;
}
 
/**
 ******************************************************************************
 ** \brief Clear overrun error flag
 **
 ** \param [in] pstcI2sl     Pointer to I2SL instance   
 **
 ** \retval Ok Error is cleared 
 ** \retval ErrorInvalidParameter pstcI2sl = NULL
 ** 
 ******************************************************************************/
en_result_t I2sl_ClrError(volatile stc_i2sln_t* pstcI2sl)
{
    if(NULL == pstcI2sl)
    {
        return ErrorInvalidParameter;
    }
    
    pstcI2sl->SSR_f.REC = 1u;
    
    return Ok;
}

/**
 ******************************************************************************
 ** \brief Write I2S data buffer
 **
 ** \param [in] pstcI2sl   Pointer to I2S instance   
 ** \param [in] u32Data    Send data
 ** 
 ** \retval Ok                    Data has been successfully sent
 ** \retval ErrorInvalidParameter If one of following conditions are met:
 **                               - pstcI2sl == NULL           
 ** 
 ******************************************************************************/
en_result_t I2sl_WriteTxFifo(volatile stc_i2sln_t* pstcI2sl, 
                             uint32_t u32Data)
{
    volatile uint32_t u32TdrAddr, *pu32TdrAddr;
  
    if(NULL == pstcI2sl)
    {
        return ErrorInvalidParameter;
    }
    
    u32TdrAddr = (uint32_t)&pstcI2sl->TDR;
    
    pu32TdrAddr =  (uint32_t*)u32TdrAddr; 
    
    *pu32TdrAddr = u32Data; // Access TX FIFO with 32-bit width
    
    return Ok;
}

/**
 ******************************************************************************
 ** \brief read I2S data buffer
 **
 ** \param [in] pstcI2sl   Pointer to I2S instance   
 ** 
 ** \retval Receive data          
 ** 
 ******************************************************************************/
uint32_t I2sl_ReadRxFifo(volatile stc_i2sln_t* pstcI2sl)
{
    volatile uint32_t u32RdrAddr, *pu32RdrAddr;
  
    u32RdrAddr = (uint32_t)&pstcI2sl->RDR;
    
    pu32RdrAddr =  (uint32_t*)u32RdrAddr; // Access RX FIFO with 32-bit width
    
    return *pu32RdrAddr;
}

/**
 ******************************************************************************
 ** \brief Reset I2S FIFO 
 **
 ** \param [in] pstcI2sl   Pointer to I2S instance   
 ** \param [in] enFifo     FIFO1 or FIFO2
 ** 
 ** \retval Ok                    FIFO has been successfully reset
 ** \retval ErrorInvalidParameter If one of following conditions are met:
 **                               - pstcCsio == NULL           
 ** 
 ******************************************************************************/
en_result_t I2sl_ResetFifo (volatile stc_i2sln_t* pstcI2sl,
                            en_i2sl_fifo_t enFifo)
{
    if(NULL == pstcI2sl)
    {
        return ErrorInvalidParameter;
    }
    
    switch (enFifo)
    {
        case I2slFifo1:
            pstcI2sl->FCR0_f.FCL1 = 1u;
            break;
        case I2slFifo2:
            pstcI2sl->FCR0_f.FCL2 = 1u;
            break;
        default:
            return ErrorInvalidParameter;
    }
    
    return Ok;
}

/**
 ******************************************************************************
 ** \brief set I2S FIFO  count
 **
 ** \param [in] pstcI2sl   Pointer to I2S instance   
 ** \param [in] enFifo     FIFO1 or FIFO2
 ** \param [in] u8Count    FIFO count
 ** 
 ** \retval Ok                    FIFO has been successfully reset
 ** \retval ErrorInvalidParameter If one of following conditions are met:
 **                               - pstcCsio == NULL           
 ** 
 ******************************************************************************/
en_result_t I2sl_SetFifoCount(volatile stc_i2sln_t* pstcI2sl,
                              en_i2sl_fifo_t enFifo,
                              uint8_t u8Count)
{
    if(NULL == pstcI2sl)
    {
        return ErrorInvalidParameter;
    }
    
    switch (enFifo)
    {
        case I2slFifo1:
            pstcI2sl->FBYTE1 = u8Count;
            break;
        case I2slFifo2:
            pstcI2sl->FBYTE1 = u8Count;
            break;
        default:
            return ErrorInvalidParameter;
    }
   
    return Ok;
}

/**
 ******************************************************************************
 ** \brief Get I2S FIFO  count
 **
 ** \param [in] pstcI2sl   Pointer to I2S instance   
 ** \param [in] enFifo     FIFO1 or FIFO2
 ** 
 ** \retval FIFO count       
 ** 
 ** This function gets the current data count in selected FIFO.
 **
 ** \note 0xFF will be return value if FIFO index is error.
 ******************************************************************************/
uint8_t I2sl_GetFifoCount(volatile stc_i2sln_t* pstcI2sl,
                          en_i2sl_fifo_t enFifo)
{
    uint8_t u8FifoCnt = 0xFFu;
    
    if(I2slFifo1 == enFifo)
    {
        u8FifoCnt = pstcI2sl->FBYTE1 ;
    }
    else
    {
        u8FifoCnt = pstcI2sl->FBYTE2;
    }
    
    return u8FifoCnt;
}

//@} // I2slGroup

#endif // #if (defined(PDL_PERIPHERAL_I2SL_ACTIVE))
/*****************************************************************************/
/* EOF (not truncated)                                                       */
/*****************************************************************************/
