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
/** \file dt.c
 **
 ** A detailed description is available at 
 ** @link DtGroup DT Module description @endlink
 **
 ** History:
 **   - 2014-04-01  1.0  EZh   First version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "dt.h"

#if (defined(PDL_PERIPHERAL_DT_ACTIVE))

/**
 ******************************************************************************
 ** \addtogroup DtGroup
 ******************************************************************************/
//@{

/******************************************************************************/
/* Local pre-processor symbols/macros ('#define')                             */
/******************************************************************************/

/******************************************************************************/
/* Global variable definitions (declared in header file with 'extern')        */
/******************************************************************************/

/// Look-up table for all enabled DT instances and their internal data
static stc_dt_instance_data_t m_astcDtInstanceDataLut[DtInstanceIndexMax] =
{
#if (PDL_PERIPHERAL_ENABLE_DT0 == PDL_ON)
    {
        &DT0,   /* pstcInstance */
        0u      /* stcInternData (not initialized yet) */
    }
#endif
};

/******************************************************************************/
/* Local type definitions ('typedef')                                         */
/******************************************************************************/


/******************************************************************************/
/* Local function prototypes ('static')                                       */
/******************************************************************************/
static stc_dt_intern_data_t* DtGetInternDataPtr(volatile stc_dtn_t** ppstcDt,
                                                uint8_t              u8Ch);
#if (PDL_INTERRUPT_ENABLE_DT0 == PDL_ON)
static void DtInitNvic(void);
static void DtDeInitNvic(void);
#endif

/******************************************************************************/
/* Local variable definitions ('static')                                      */
/******************************************************************************/

/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/

/**
 ******************************************************************************
 ** \brief Check pointer to Dual Timer instance and enable channel
 **
 ** \param [in,out] ppstcDt  Pointer of pointer to Dual Timer instance register area
 ** \param [in]     u8Ch     Channel number
 **
 ** \retval Pointer to internal data or NULL if instance is not enabled.
 **         (or channel is invalid)
 **
 ******************************************************************************/
static stc_dt_intern_data_t* DtGetInternDataPtr(volatile stc_dtn_t** ppstcDt,
                                                uint8_t              u8Ch)
{
    stc_dt_intern_data_t* pstcDtInternData = NULL;
    uint32_t              u32Instance;

    /* check for channel */
    if ((NULL != ppstcDt)
    &&  (NULL != *ppstcDt)
    &&  (DtMaxChannels > u8Ch)
       )
    {
        /* Get ptr to internal data struct ... */
        for (u32Instance = 0; u32Instance < (uint32_t)DtInstanceIndexMax; u32Instance++)
        {
            if (*ppstcDt == m_astcDtInstanceDataLut[u32Instance].pstcInstance)
            {
                /* Set actual address of register list of current channel */
                *ppstcDt = &((*ppstcDt)[u8Ch]);
                pstcDtInternData = &m_astcDtInstanceDataLut[u32Instance].stcInternData;
                break;
            }
        }
    }

    return (pstcDtInternData);
} /* DtGetInternDataPtr */

#if (PDL_INTERRUPT_ENABLE_DT0 == PDL_ON)
/**
 ******************************************************************************
 ** \brief Device dependent initialization of interrupts according CMSIS with
 **        level defined in pdl.h
 **
 ******************************************************************************/
static void DtInitNvic(void)
{    
#if (PDL_MCU_CORE == PDL_FM4_CORE)
    NVIC_ClearPendingIRQ(DT1_2_IRQn);
    NVIC_EnableIRQ(DT1_2_IRQn);
    NVIC_SetPriority(DT1_2_IRQn, PDL_IRQ_LEVEL_DT0);
#elif (PDL_MCU_CORE == PDL_FM3_CORE)
    NVIC_ClearPendingIRQ(DTIM_QDU_IRQn);
    NVIC_EnableIRQ(DTIM_QDU_IRQn);
    NVIC_SetPriority(DTIM_QDU_IRQn, PDL_IRQ_LEVEL_DT_QPRC);
#else
#if (PDL_MCU_TYPE == PDL_FM0P_TYPE3)
    NVIC_ClearPendingIRQ(DTIM_RTC_WC_IRQn);
    NVIC_EnableIRQ(DTIM_RTC_WC_IRQn);
    NVIC_SetPriority(DTIM_RTC_WC_IRQn, PDL_IRQ_LEVEL_DT_RTC_WC);
#else
    NVIC_ClearPendingIRQ(DTIM_QDU_IRQn);
    NVIC_EnableIRQ(DTIM_QDU_IRQn);
    NVIC_SetPriority(DTIM_QDU_IRQn, PDL_IRQ_LEVEL_DT_QPRC);
#endif
#endif
} /* Dt_InitIrq */

/**
 ******************************************************************************
 ** \brief Device dependent de-initialization of interrupts according CMSIS 
 **
 ******************************************************************************/
static void DtDeInitNvic(void)
{
#if (PDL_MCU_CORE == PDL_FM4_CORE)
    NVIC_ClearPendingIRQ(DT1_2_IRQn);
    NVIC_DisableIRQ(DT1_2_IRQn);
    NVIC_SetPriority(DT1_2_IRQn, PDL_DEFAULT_INTERRUPT_LEVEL);
#elif (PDL_MCU_CORE == PDL_FM3_CORE)    
    NVIC_ClearPendingIRQ(DTIM_QDU_IRQn);
    NVIC_DisableIRQ(DTIM_QDU_IRQn);
    NVIC_SetPriority(DTIM_QDU_IRQn, PDL_DEFAULT_INTERRUPT_LEVEL);
#else
#if (PDL_MCU_TYPE == PDL_FM0P_TYPE3)
    NVIC_ClearPendingIRQ(DTIM_RTC_WC_IRQn);
    NVIC_DisableIRQ(DTIM_RTC_WC_IRQn);
    NVIC_SetPriority(DTIM_RTC_WC_IRQn, PDL_DEFAULT_INTERRUPT_LEVEL);
#else    
    NVIC_ClearPendingIRQ(DTIM_QDU_IRQn);
    NVIC_DisableIRQ(DTIM_QDU_IRQn);
    NVIC_SetPriority(DTIM_QDU_IRQn, PDL_DEFAULT_INTERRUPT_LEVEL);
#endif  
#endif  
} /* Dt_DeInitIrq */

/**
 *****************************************************************************
 ** \brief ISR callback for DT (channel 0 and 1)
 **
 ** This callbacks are called by the global DT ISR whenever an DT triggers an
 ** interrupt (on channel 0 and/or 1). It calls the callback functions that
 ** has been given during Dt initialization (see Dt_Init() and
 ** #stc_dt_channel_config_t) for each channel individually. If the pointer
 ** to a callback function is NULL, no call is performed.
 **
 ** The active interrupt request flags are cleared by the ISR
 **
 ** \param [in]  u8Ch             Channel number
 **
 *****************************************************************************/
void DtIrqHandler(uint8_t u8Ch)
{
    volatile stc_dtn_t*   pstcDt;
    stc_dt_intern_data_t* pstcDtInternData;

    pstcDt = &DT0;
    /* Get actual address of register list of current channel */
    pstcDt = (volatile stc_dtn_t*)(&((pstcDt)[u8Ch]));
    pstcDtInternData = &m_astcDtInstanceDataLut[0].stcInternData;

    if (TRUE == pstcDt->TIMERXRIS_f.TIMERXRIS)  /* Timer 0 interrupt? */
    {
        pstcDt->TIMERXINTCLR = 0u;               /* Clear interrupt */

        /* Check for callback function pointer */
        if (NULL != pstcDtInternData->pfnIrqCallbackIntern[u8Ch])
        {
            pstcDtInternData->pfnIrqCallbackIntern[u8Ch]() ;
        }
    }
} /* DtIrqHandler */

#endif

/**
 *****************************************************************************
 ** \brief Initialize DT
 **
 ** This function initializes the specified channel of Dual Timer.
 **
 ** \param [in]  pstcConfig       Dual timer configuration
 ** \param [in]  u8Ch             Channel number
 **
 ** \retval Ok                    Process successfully done.
 ** \retval ErrorInvalidParameter If one of following conditions are met:
 **             - pstcConfig == NULL
 **             - u8Ch >= DtMaxChannels
 **             - pstcDtInternData == NULL (invalid or disabled DT unit
 **                                         (PDL_PERIPHERAL_ENABLE_DT))
 **             - one or more enumerated values in pstcDt out of enumaration
 **
 *****************************************************************************/
en_result_t Dt_Init(stc_dt_channel_config_t* pstcConfig,
                    uint8_t                  u8Ch
                   )
{
    en_result_t                    enResult;
    /* Pointer to Dual Timer instance register area */
    volatile stc_dtn_t*            pstcDt;
    /* Pointer to internal data */
    stc_dt_intern_data_t*          pstcDtInternData;
    stc_dtim_timerXcontrol_field_t stcTIMERXCTRL = { 0u }; /* Preset to zero */

    pstcDt = &DT0;
    /*-------- Initialize internal data -------*/
    /* Get pointer to internal data structure and check channel... */
    pstcDtInternData = DtGetInternDataPtr(&pstcDt, u8Ch);
    /* ... and check for NULL */
    if ((NULL == pstcDtInternData)
    ||  (NULL == pstcConfig)
       )
    {
        enResult = ErrorInvalidParameter;
    }
    else
    {
        enResult = Ok;
        /*-------- Configure the DT timer -------*/
        pstcDt->TIMERXCONTROL = 0u;              /* Disable everything */

        /* Set Timer Mode */
        switch (pstcConfig->u8Mode)
        {
            /* Free run mode */
            case DtFreeRun:
                stcTIMERXCTRL.TIMERMODE = FALSE;
                stcTIMERXCTRL.ONESHOT   = FALSE;
                break;
            /* Periodic mode */
            case DtPeriodic:
                stcTIMERXCTRL.TIMERMODE = TRUE;
                stcTIMERXCTRL.ONESHOT   = FALSE;
                break;
            /* One shot mode */
            case DtOneShot:
                stcTIMERXCTRL.TIMERMODE = FALSE;
                stcTIMERXCTRL.ONESHOT   = TRUE;
                break;
            default:
                enResult = ErrorInvalidParameter;
                break;
        }

        /* Set Prescaler */
        switch (pstcConfig->u8PrescalerDiv)
        {
            /* Clock/1 */
            case DtPrescalerDiv1:
                stcTIMERXCTRL.TIMERPRE = DT_PRE_TIMER_DIV_1;
                break;
            /* Clock/16 */
            case DtPrescalerDiv16:
                stcTIMERXCTRL.TIMERPRE = DT_PRE_TIMER_DIV_16;
                break;
            /* Clock/256 */
            case DtPrescalerDiv256:
                stcTIMERXCTRL.TIMERPRE = DT_PRE_TIMER_DIV_256;
                break;
            default:
                enResult = ErrorInvalidParameter;
                break;
        }

        /* Set Counter Size */
        switch (pstcConfig->u8CounterSize)
        {
            /* 16bit */
            case DtCounterSize16:
                stcTIMERXCTRL.TIMERSIZE = FALSE;
                break;
            /* 32bit */
            case DtCounterSize32:
                stcTIMERXCTRL.TIMERSIZE = TRUE;
                break;
            default:
                enResult = ErrorInvalidParameter;
                break;
        }
#if (PDL_INTERRUPT_ENABLE_DT0 == PDL_ON)        
        /* Set interrupt */
        if(pstcConfig->bIrqEnable == TRUE)
        {
            stcTIMERXCTRL.INTENABLE = 1u;
        }
        
        /* Set interrupt callback function */
        pstcDtInternData->pfnIrqCallbackIntern[u8Ch] = pstcConfig->pfnIrqCallback;
        
        /* Set NVIC */
        if(pstcConfig->bTouchNvic == TRUE)
        {
            DtInitNvic();
        }
#endif        
    }
    
    if (Ok == enResult)
    {
        /* Set control register */
        pstcDt->TIMERXCONTROL_f = stcTIMERXCTRL;
    }
    
    return (enResult);
} /* Dt_Init */

/**
 *****************************************************************************
 ** \brief De-Initialize DT
 **
 ** This function de-initializes the specified channel of Dual Timer.
 ** Dt-DeInit() accesses the DT hardware register. They are reset.
 **
 ** \param [in]  u8Ch             Channel number
 ** \param [in]  bTouchNvic       Touch NVIC or not
 **
 ** \retval Ok                    Process successfully done.
 ** \retval ErrorInvalidParameter If one of following conditions are met:
 **             - pstcDt == NULL
 **             - u8Ch >= DtMaxChannels
 **             - pstcDtInternData == NULL (invalid or disabled DT unit
 **                                         (PDL_PERIPHERAL_ENABLE_DT)) 
 **
 *****************************************************************************/
en_result_t Dt_DeInit(uint8_t u8Ch, boolean_t bTouchNvic)
{
    en_result_t           enResult;
    /* Pointer to Dual Timer instance register area */
    volatile stc_dtn_t*   pstcDt;
    /* Pointer to internal data */
    stc_dt_intern_data_t* pstcDtInternData;

    pstcDt = &DT0;
    /*-------- Initialize internal data -------*/
    /* Get pointer to internal data structure and check channel... */
    pstcDtInternData = DtGetInternDataPtr(&pstcDt, u8Ch);
    /* ... and check for NULL */
    if (NULL == pstcDtInternData)
    {
        enResult = ErrorInvalidParameter;
    }
    else
    {
        /* Clear all registers */
        pstcDt->TIMERXCONTROL = 0u;

        pstcDt->TIMERXLOAD    = 0u;
        pstcDt->TIMERXINTCLR  = 1u;
        pstcDt->TIMERXBGLOAD  = 0u;

#if (PDL_INTERRUPT_ENABLE_DT0 == PDL_ON)          
        /* Clear callback */
        pstcDtInternData->pfnIrqCallbackIntern[u8Ch] = NULL;
        
        if(bTouchNvic == TRUE)
        {
            DtDeInitNvic();
        }
#endif        
        enResult = Ok;
    }

    return (enResult);
} /* Dt_DeInit */

/**
 *****************************************************************************
 ** \brief Enable Timer Counter
 **
 ** This function enables the timer counter.
 **
 ** \param [in]  u8Ch             Channel number
 **
 ** \retval Ok                    Process successfully done.
 ** \retval ErrorInvalidParameter If one of following conditions are met:
 **             - u8Ch >= DtMaxChannels
 **             - pstcDtInternData == NULL (invalid or disabled DT unit
 **                                         (PDL_PERIPHERAL_ENABLE_DT)) 
 **
 *****************************************************************************/
en_result_t Dt_EnableCount(uint8_t u8Ch)
{
    en_result_t           enResult;
    /* Pointer to Dual Timer instance register area */
    volatile stc_dtn_t*   pstcDt;
    /* Pointer to internal data */
    stc_dt_intern_data_t* pstcDtInternData;

    enResult = ErrorInvalidParameter;

    pstcDt = &DT0;
    /* Get pointer to internal data structure and check channel... */
    pstcDtInternData = DtGetInternDataPtr(&pstcDt, u8Ch);
    /* ... and check for NULL */
    if (NULL != pstcDtInternData)
    {
        /* Enable timer counter */
        pstcDt->TIMERXCONTROL_f.TIMEREN = TRUE;
        enResult = Ok;
    }

    return (enResult);
} /* Dt_EnableCount */

/**
 *****************************************************************************
 ** \brief Disable Timer Counter
 **
 ** This function disables the timer counter.
 **
 ** \param [in]  u8Ch             Channel number
 **
 ** \retval Ok                    Process successfully done.
 ** \retval ErrorInvalidParameter If one of following conditions are met:
 **             - u8Ch >= DtMaxChannels
 **             - pstcDtInternData == NULL (invalid or disabled DT unit
 **                                         (PDL_PERIPHERAL_ENABLE_DT)) 
 **
 *****************************************************************************/
en_result_t Dt_DisableCount(uint8_t u8Ch)
{
    en_result_t           enResult;
    /* Pointer to Dual Timer instance register area */
    volatile stc_dtn_t*   pstcDt;
    /* Pointer to internal data */
    stc_dt_intern_data_t* pstcDtInternData;

    enResult = ErrorInvalidParameter;

    pstcDt = &DT0;
    /* Get pointer to internal data structure and check channel... */
    pstcDtInternData = DtGetInternDataPtr(&pstcDt, u8Ch);
    /* ... and check for NULL */
    if (NULL != pstcDtInternData)
    {
        /* Disable timer counter */
        pstcDt->TIMERXCONTROL_f.TIMEREN = FALSE;
        enResult = Ok;
    }

    return (enResult);
} /* Dt_DisableCount */

#if (PDL_INTERRUPT_ENABLE_DT0 == PDL_ON)
/**
 *****************************************************************************
 ** \brief Enable Interrupt
 **
 ** This function enables the interruption.
 **
 ** \param [in]  u8Ch             Channel number
 **
 ** \retval Ok                    Process successfully done.
 ** \retval ErrorInvalidParameter If one of following conditions are met:
 **             - u8Ch >= DtMaxChannels
 **             - pstcDtInternData == NULL (invalid or disabled DT unit
 **                                         (PDL_PERIPHERAL_ENABLE_DT)) 
 **
 *****************************************************************************/
en_result_t Dt_EnableIrq(uint8_t          u8Ch)
{
    en_result_t           enResult;
    /* Pointer to Dual Timer instance register area */
    volatile stc_dtn_t*   pstcDt;
    /* Pointer to internal data */
    stc_dt_intern_data_t* pstcDtInternData;

    enResult = ErrorInvalidParameter;

    pstcDt = &DT0;
    /* Get pointer to internal data structure and check channel... */
    pstcDtInternData = DtGetInternDataPtr(&pstcDt, u8Ch);
    /* ... and check for NULL */
    if (NULL != pstcDtInternData)
    {
        /* Enable interrupt */
        pstcDt->TIMERXCONTROL_f.INTENABLE = TRUE;

        enResult = Ok;
    }

    return (enResult);
} /* Dt_EnableIrq */

/**
 *****************************************************************************
 ** \brief Disable Interrupt
 **
 ** This function disables the interruption.
 **
 ** \param [in]  u8Ch             Channel number
 **
 ** \retval Ok                    Process successfully done.
 ** \retval ErrorInvalidParameter If one of following conditions are met:
 **             - u8Ch >= DtMaxChannels
 **             - pstcDtInternData == NULL (invalid or disabled DT unit
 **                                         (PDL_PERIPHERAL_ENABLE_DT)) 
 **
 *****************************************************************************/
en_result_t Dt_DisableIrq(uint8_t u8Ch)
{
    en_result_t           enResult;
    /* Pointer to Dual Timer instance register area */
    volatile stc_dtn_t*   pstcDt;
    /* Pointer to internal data */
    stc_dt_intern_data_t* pstcDtInternData;

    enResult = ErrorInvalidParameter;

    pstcDt = &DT0;
    /* Get pointer to internal data structure and check channel... */
    pstcDtInternData = DtGetInternDataPtr(&pstcDt, u8Ch);
    /* ... and check for NULL */
    if (NULL != pstcDtInternData)
    {
        /* Disable interrupt */
        pstcDt->TIMERXCONTROL_f.INTENABLE = FALSE;

        enResult = Ok;
    }

    return (enResult);
} /* Dt_DisableIrq */
#endif

/**
 ******************************************************************************
 ** \brief Get interrupt status
 ** The Function can return the interrupt status (TimerXRIS)
 **
 ** \param [in]     u8Ch          Channel number
 **
 ** \retval boolean_t:the interrupt status
 **
 ******************************************************************************/
boolean_t Dt_GetIrqFlag(uint8_t u8Ch)
{
    /* Pointer to Dual Timer instance register area */
    volatile stc_dtn_t*   pstcDt;
    boolean_t             bRetVal = FALSE;
    /* Pointer to internal data */
    stc_dt_intern_data_t* pstcDtInternData;

    pstcDt = &DT0;
    /* Get pointer to internal data structure and check channel... */
    pstcDtInternData = DtGetInternDataPtr(&pstcDt, u8Ch);
    /* ... and check for NULL */
    if (NULL != pstcDtInternData)
    {
        /* Check the interrupt status */
        if (TRUE == pstcDt->TIMERXRIS_f.TIMERXRIS)
        {
            bRetVal = TRUE;
        }
    }

    return (bRetVal);
} /* Dt_GetIrqFlag */

/**
 ******************************************************************************
 ** \brief Get mask interrupt status
 ** The Function can return the mask interrupt status (TimerXMIS)
 **
 ** \param [in]     u8Ch          Channel number
 **
 ** \retval boolean_t:the mask interrupt status
 **
 ******************************************************************************/
boolean_t Dt_GetMaskIrqFlag(uint8_t u8Ch)
{
    /* Pointer to Dual Timer instance register area */
    volatile stc_dtn_t*   pstcDt;
    boolean_t             bRetVal = FALSE;
    /* Pointer to internal data */
    stc_dt_intern_data_t* pstcDtInternData;

    pstcDt = &DT0;
    /* Get pointer to internal data structure and check channel... */
    pstcDtInternData = DtGetInternDataPtr(&pstcDt, u8Ch);
    /* ... and check for NULL */
    if (NULL != pstcDtInternData)
    {
        /* Check the mask interrupt status */
        if (TRUE == pstcDt->TIMERXMIS_f.TIMERXMIS)
        {
            bRetVal = TRUE;
        }
    }

    return (bRetVal);
} /* Dt_GetMaskIrqFlag */

/**
 ******************************************************************************
 ** \brief Clear interrupt status
 ** The Function clears the interrupt status
 **
 ** \param [in]  u8Ch           Channel number
 **
 ** \retval Ok                        Process successfully done.
 ** \retval ErrorInvalidParameter     If one of following conditions are met:
 **             - u8Ch >= DtMaxChannels
 **             - pstcDtInternData == NULL (invalid or disabled DT unit
 **                                         (PDL_PERIPHERAL_ENABLE_DT)) 
 **
 ******************************************************************************/
en_result_t Dt_ClrIrqFlag(uint8_t u8Ch)
{
    en_result_t           enResult;
    /* Pointer to Dual Timer instance register area */
    volatile stc_dtn_t*   pstcDt;
    /* Pointer to internal data */
    stc_dt_intern_data_t* pstcDtInternData;

    enResult = ErrorInvalidParameter;

    pstcDt = &DT0;
    /* Get pointer to internal data structure and check channel... */
    pstcDtInternData = DtGetInternDataPtr(&pstcDt, u8Ch);
    /* ... and check for NULL */
    if (NULL != pstcDtInternData)
    {
        /* Clear the interrupt status */
        pstcDt->TIMERXINTCLR = 1;
        enResult = Ok;
    }

    return (enResult);
} /* Dt_ClrIrqFlag */

/**
 ******************************************************************************
 ** \brief Write load value
 ** The Function writes the load value to load register
 **
 ** \param [in]  u32LoadVal     Load value to set to load register
 ** \param [in]  u8Ch           Channel number
 **
 ** \retval Ok                        Process successfully done.
 ** \retval ErrorInvalidParameter     If one of following conditions are met:
 **             - u8Ch >= DtMaxChannels
 **             - pstcDtInternData == NULL (invalid or disabled DT unit
 **                                         (PDL_PERIPHERAL_ENABLE_DT)) 
 **
 ******************************************************************************/
en_result_t Dt_WriteLoadVal(uint32_t u32LoadVal,
                            uint8_t  u8Ch
                           )
{
    en_result_t           enResult;
    /* Pointer to Dual Timer instance register area */
    volatile stc_dtn_t*   pstcDt;
    /* Pointer to internal data */
    stc_dt_intern_data_t* pstcDtInternData;

    enResult = ErrorInvalidParameter;

    pstcDt = &DT0;
    /* Get pointer to internal data structure and check channel... */
    pstcDtInternData = DtGetInternDataPtr(&pstcDt, u8Ch);
    /* ... and check for NULL */
    if (NULL != pstcDtInternData)
    {
        /* 16bit mode */
        if (FALSE == pstcDt->TIMERXCONTROL_f.TIMERSIZE)
        {
            u32LoadVal &= 0x0000FFFFu;
        }
        /* Write load value to register */
        pstcDt->TIMERXLOAD = u32LoadVal;
        enResult = Ok;
    }

    return (enResult);
} /* Dt_WriteLoadVal */

/**
 ******************************************************************************
 ** \brief Write back-ground load value
 ** The Function writes the load value to back-ground load register
 **
 ** \param [in]  u32BgLoadVal   Load value to set to back-ground load register
 ** \param [in]  u8Ch           Channel number
 **
 ** \retval Ok                        Process successfully done.
 ** \retval ErrorInvalidParameter     If one of following conditions are met:
 **             - u8Ch >= DtMaxChannels
 **             - pstcDtInternData == NULL (invalid or disabled DT unit
 **                                         (PDL_PERIPHERAL_ENABLE_DT)) 
 **
 ******************************************************************************/
en_result_t Dt_WriteBgLoadVal(uint32_t u32BgLoadVal,
                              uint8_t  u8Ch
                             )
{
    en_result_t           enResult;
    /* Pointer to Dual Timer instance register area */
    volatile stc_dtn_t*   pstcDt;
    /* Pointer to internal data */
    stc_dt_intern_data_t* pstcDtInternData;

    enResult = ErrorInvalidParameter;

    pstcDt = &DT0;
    /* Get pointer to internal data structure and check channel... */
    pstcDtInternData = DtGetInternDataPtr(&pstcDt, u8Ch);
    /* ... and check for NULL */
    if (NULL != pstcDtInternData)
    {
        /* 16bit mode */
        if (FALSE == pstcDt->TIMERXCONTROL_f.TIMERSIZE)
        {
            u32BgLoadVal &= 0x0000FFFFu;
        }
        /* Write back-ground load value to register */
        pstcDt->TIMERXBGLOAD = u32BgLoadVal;
        enResult = Ok;
    }

    return (enResult);
} /* Dt_WriteBgLoadVal */

/**
 ******************************************************************************
 ** \brief Read current count value
 ** The Function reads the value from value register
 **
 ** \param [in]  u8Ch           Channel number
 **
 ** \retval uint32_t:current counter value
 **
 ******************************************************************************/
uint32_t Dt_ReadCurCntVal(uint8_t u8Ch)
{
    /* Pointer to Dual Timer instance register area */
    volatile stc_dtn_t*   pstcDt;
    uint32_t              u32DtValue = 0;
    /* Pointer to internal data */
    stc_dt_intern_data_t* pstcDtInternData;

    pstcDt = &DT0;
    /* Get pointer to internal data structure and check channel... */
    pstcDtInternData = DtGetInternDataPtr(&pstcDt, u8Ch);
    /* ... and check for NULL */
    if (NULL != pstcDtInternData)
    {
        /* Read current count value */
        u32DtValue = pstcDt->TIMERXVALUE;
        /* 16bit mode */
        if (FALSE == pstcDt->TIMERXCONTROL_f.TIMERSIZE)
        {
            u32DtValue &= 0x0000FFFFu;
        }
    }

    return (u32DtValue);
} /* Dt_ReadCurCntVal */

//@} // DtGroup

#endif /* #if (defined(PDL_PERIPHERAL_DT_ACTIVE)) */

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/

