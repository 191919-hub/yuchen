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
/** \file rtc.c
 **
 ** A detailed description is available at 
 ** @link RtcGroup RTC Module description @endlink
 **
 ** History:
 **   - 2014-12-04  1.0  EZh   First version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "rtc.h"

#if (defined(PDL_PERIPHERAL_RTC_ACTIVE))

/**
 ******************************************************************************
 ** \addtogroup RtcGroup
 ******************************************************************************/
//@{
   
/******************************************************************************/
/* Local pre-processor symbols/macros ('#define')                             */
/******************************************************************************/   
/* Timeout count */
#define RTC_TIMEOUT              (SystemCoreClock/10u)
/* Error code for mktime */
#define RTC_ERR                 (-1)
/* convert DEC to BCD */
#define DecToBcd(x)           ((((x)/10)<<4) + ((x)%10))
/* convert BCD to DEC */
#define BcdToDec(x)           ((((x)>>4)*10) + ((x)&0x0F))

/* WTCR20 bit mapping */
#define RTC_WTCR20_PWRITE       (0x20u)
#define RTC_WTCR20_PREAD        (0x10u)
#define RTC_WTCR20_BWRITE       (0x08u)
#define RTC_WTCR20_BREAD        (0x04u)
#define RTC_WTCR20_CWRITE       (0x02u)
#define RTC_WTCR20_CREAD        (0x01u)   
   
/******************************************************************************/
/* Global variable definitions (declared in header file with 'extern')        */
/******************************************************************************/
/// Look-up table for all enabled RTC instances and their internal data
stc_rtc_instance_data_t m_astcRtcInstanceDataLut[RTC_INSTANCE_COUNT] =
{
#if (PDL_PERIPHERAL_ENABLE_RTC0 == PDL_ON)
    { 
        &RTC0,  // pstcInstance
        0u          
    },
#endif
};   
   
 
/******************************************************************************/
/* Local type definitions ('typedef')                                         */
/******************************************************************************/

/******************************************************************************/
/* Local function prototypes ('static')                                       */
/******************************************************************************/
static void RtcToTm( stc_rtc_time_t* pstcRtcTime,
                     struct tm*      pstcTime);
static void RtcDisableNvic(void);
static void RtcRestoreNvic(void);

#if (PDL_INTERRUPT_ENABLE_RTC0 == PDL_ON)
static void RtcInitNvic(volatile stc_rtcn_t* pstcRtc);
static void RtcDeInitNvic(volatile stc_rtcn_t* pstcRtc);
#endif

/******************************************************************************/
/* Local variable definitions ('static')                                      */
/******************************************************************************/
static uint32_t u32NvicData;

/**
 ******************************************************************************
 ** \brief Return the internal data for a certain RTC instance.
 **
 ** \param pstcRtc Pointer to RTC instance
 **
 ** \return Pointer to internal data or NULL if instance is not enabled (or not known)
 **
 ******************************************************************************/
static stc_rtc_intern_data_t* RtcGetInternDataPtr(volatile stc_rtcn_t* pstcRtc) 
{
    uint8_t u8Instance;
   
    for (u8Instance = 0u; u8Instance < RTC_INSTANCE_COUNT; u8Instance++)
    {
        if (pstcRtc == m_astcRtcInstanceDataLut[u8Instance].pstcInstance)
        {
            return &m_astcRtcInstanceDataLut[u8Instance].stcInternData;
        }
    }

    return NULL;
}

#if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB)  
/**
 ******************************************************************************
 ** \brief Wait to complete writing transmission
 **
 ** \param [in]  pstcRtc         Pointer to RTC instance
 **
 ** \retval  Ok             Success to complete transmission
 ** \retval  ErrorTimeout   Timeout to complete transmission
 **
 ******************************************************************************/
static en_result_t RtcWaitTransComplete(volatile stc_rtcn_t* pstcRtc)
{
    en_result_t enResult = Ok;
    uint32_t    u32WaitCompleteTimeout;

    u32WaitCompleteTimeout = RTC_TIMEOUT;
    while ((TRUE == pstcRtc->WTCR10_f.TRANS)    /* Wait until transmission is completed */
    &&     (0u != u32WaitCompleteTimeout)        /* If transmission is not completed even if it passes for a long time... */
          )
    {
        PDL_WAIT_LOOP_HOOK();
        u32WaitCompleteTimeout--;
    }

    if (0u == u32WaitCompleteTimeout)
    {
        enResult = ErrorTimeout;
    }

    return (enResult);
} /* RtcWaitTransComplete */   

/**
 ******************************************************************************
 ** \brief Transfer to/from VBAT domain and wait to complete transmission
 **
 ** \param [in] pstcRtc     Pointer to RTC instance
 ** \param [in] u8Control   Control to transmission
 **
 ** \retval  Ok             Success to complete transmission
 ** \retval  ErrorTimeout   Timeout to complete transmission
 **
 ******************************************************************************/
static en_result_t RtcTransWithVbat(volatile stc_rtcn_t* pstcRtc, 
                                    uint8_t u8Control)
{
    en_result_t enResult;

    /* Transmit to or from VBAT domain */
    FM_RTC->WTCR20 = u8Control;
    /* Wait to complete transmission */
    enResult = RtcWaitTransComplete(pstcRtc);

    return (enResult);
} /* RtcTransToVbat */
#endif

/**
 ******************************************************************************
 ** \brief Disable RTC NVIC and save original value
 ******************************************************************************/
static void RtcDisableNvic(void)
{
#if (PDL_MCU_CORE == PDL_FM4_CORE)
    u32NvicData = NVIC->ISER[(uint32_t)((int32_t)RTC_IRQn) >> 5u];
    NVIC->ICER[((uint32_t)(RTC_IRQn) >> 5u)] = (1ul << ((uint32_t)(RTC_IRQn) & 0x1Fu));
#elif (PDL_MCU_CORE == PDL_FM0P_CORE)
  #if (PDL_MCU_TYPE == PDL_FM0P_TYPE3)
    u32NvicData = NVIC->ISER[(uint32_t)((int32_t)DTIM_RTC_WC_IRQn) >> 5u];
    NVIC->ICER[((uint32_t)(DTIM_RTC_WC_IRQn) >> 5u)] = (1ul << ((uint32_t)(DTIM_RTC_WC_IRQn) & 0x1Fu));
  #else  
    u32NvicData = NVIC->ISER[(uint32_t)((int32_t)ICC1_HDMICEC1_OSC_PLL_WC_RTC_IRQn) >> 5u];
    NVIC->ICER[((uint32_t)(ICC1_HDMICEC1_OSC_PLL_WC_RTC_IRQn) >> 5u)] = (1ul << ((uint32_t)(ICC1_HDMICEC1_OSC_PLL_WC_RTC_IRQn) & 0x1Fu));
  #endif    
#else
    u32NvicData = NVIC->ISER[(uint32_t)((int32_t)OSC_PLL_WC_RTC_IRQn) >> 5u];
    NVIC->ICER[((uint32_t)(OSC_PLL_WC_RTC_IRQn) >> 5u)] = (1ul << ((uint32_t)(OSC_PLL_WC_RTC_IRQn) & 0x1Fu));
#endif    
}

/**
 ******************************************************************************
 ** \brief Restore RTC NVIC
 ******************************************************************************/
static void RtcRestoreNvic(void)
{
#if (PDL_MCU_CORE == PDL_FM4_CORE)  
    NVIC->ISER[(uint32_t)((int32_t)RTC_IRQn) >> 5u] = u32NvicData;
#elif (PDL_MCU_CORE == PDL_FM0P_CORE)
  #if (PDL_MCU_TYPE == PDL_FM0P_TYPE3)
    NVIC->ISER[(uint32_t)((int32_t)DTIM_RTC_WC_IRQn) >> 5u] = u32NvicData;
  #else  
    NVIC->ISER[(uint32_t)((int32_t)ICC1_HDMICEC1_OSC_PLL_WC_RTC_IRQn) >> 5u] = u32NvicData;
  #endif    
#else
    NVIC->ISER[(uint32_t)((int32_t)OSC_PLL_WC_RTC_IRQn) >> 5u] = u32NvicData;
#endif    
}


/**
 ******************************************************************************
 ** \brief Convert RTC time structure to time.h tm structure
 **
 ** \param [in]  pstcRtcTime    RTC Time structure
 ** \param [out] pstcTime       tm Time structure
 **
 ******************************************************************************/
static void RtcToTm( stc_rtc_time_t* pstcRtcTime,
                     struct tm*      pstcTime)
{
    pstcTime->tm_year  = (int)(pstcRtcTime->u16Year - 1900u);
    pstcTime->tm_mon   = (int)(pstcRtcTime->u8Month - 1u);
    pstcTime->tm_mday  = (int)(pstcRtcTime->u8Day);
    pstcTime->tm_sec   = (int)(pstcRtcTime->u8Second);
    pstcTime->tm_min   = (int)(pstcRtcTime->u8Minute);
    pstcTime->tm_hour  = (int)(pstcRtcTime->u8Hour);
    pstcTime->tm_isdst = 0;
} /* RtcToTm */

#if (PDL_INTERRUPT_ENABLE_RTC0 == PDL_ON)
/**
 ******************************************************************************
 ** \brief Device dependent initialization of interrupts according CMSIS with
 **        level defined in pdl_user.h
 **
 ** \param [in] pstcRtc     Pointer to RTC instance
 **
 ******************************************************************************/
static void RtcInitNvic(volatile stc_rtcn_t* pstcRtc)
{
#if (PDL_MCU_CORE == PDL_FM4_CORE)
    NVIC_ClearPendingIRQ(RTC_IRQn);
    NVIC_EnableIRQ(RTC_IRQn);
    NVIC_SetPriority(RTC_IRQn, PDL_IRQ_LEVEL_RTC0);
#elif (PDL_MCU_CORE == PDL_FM0P_CORE)
  #if (PDL_MCU_TYPE == PDL_FM0P_TYPE3)
    NVIC_ClearPendingIRQ(DTIM_RTC_WC_IRQn);
    NVIC_EnableIRQ(DTIM_RTC_WC_IRQn);
    NVIC_SetPriority(DTIM_RTC_WC_IRQn, PDL_IRQ_LEVEL_DT_RTC_WC);
  #else  
    NVIC_ClearPendingIRQ(ICC1_HDMICEC1_OSC_PLL_WC_RTC_IRQn);
    NVIC_EnableIRQ(ICC1_HDMICEC1_OSC_PLL_WC_RTC_IRQn);
    NVIC_SetPriority(ICC1_HDMICEC1_OSC_PLL_WC_RTC_IRQn, PDL_IRQ_LEVEL_CLK_WC_RTC_CEC1_ICC1);
  #endif  
#else  
    NVIC_ClearPendingIRQ(OSC_PLL_WC_RTC_IRQn);
    NVIC_EnableIRQ(OSC_PLL_WC_RTC_IRQn);
    NVIC_SetPriority(OSC_PLL_WC_RTC_IRQn, PDL_IRQ_LEVEL_CLK_WC_RTC);
#endif    

} /* RtcInitIrq */

/**
 ******************************************************************************
 ** \brief Device dependent de-initialization of interrupts according CMSIS with
 **        level defined in pdl.h
 **
 ** \param [in] pstcRtc     Pointer to RTC instance
 **
 ******************************************************************************/
static void RtcDeInitNvic(volatile stc_rtcn_t* pstcRtc)
{
#if (PDL_MCU_CORE == PDL_FM4_CORE)
    NVIC_ClearPendingIRQ(RTC_IRQn);
    NVIC_DisableIRQ(RTC_IRQn);
    NVIC_SetPriority(RTC_IRQn, PDL_DEFAULT_INTERRUPT_LEVEL);
#elif (PDL_MCU_CORE == PDL_FM0P_CORE)
  #if (PDL_MCU_TYPE == PDL_FM0P_TYPE3)
    NVIC_ClearPendingIRQ(DTIM_RTC_WC_IRQn);
    NVIC_DisableIRQ(DTIM_RTC_WC_IRQn);
    NVIC_SetPriority(DTIM_RTC_WC_IRQn, PDL_DEFAULT_INTERRUPT_LEVEL);
  #else
    NVIC_ClearPendingIRQ(ICC1_HDMICEC1_OSC_PLL_WC_RTC_IRQn);
    NVIC_DisableIRQ(ICC1_HDMICEC1_OSC_PLL_WC_RTC_IRQn);
    NVIC_SetPriority(ICC1_HDMICEC1_OSC_PLL_WC_RTC_IRQn, PDL_DEFAULT_INTERRUPT_LEVEL);
  #endif    
#else  
    NVIC_ClearPendingIRQ(OSC_PLL_WC_RTC_IRQn);
    NVIC_DisableIRQ(OSC_PLL_WC_RTC_IRQn);
    NVIC_SetPriority(OSC_PLL_WC_RTC_IRQn, PDL_DEFAULT_INTERRUPT_LEVEL);
#endif    
} /* RtcDeInitIrq */

/**
 ******************************************************************************
 ** \brief RTC  interrupt service routine.
 **
 ** \param [in] pstcRtc     Pointer to RTC instance
 ** \param [in] pstcRtcInternData  Pointer to RTC intern data
 ******************************************************************************/
void Rtc_IrqHandler(volatile stc_rtcn_t* pstcRtc, stc_rtc_intern_data_t* pstcRtcInternData)
{
    stc_rtc_wtcr12_field_t stcWtcr12;
    stc_rtc_wtcr11_field_t stcWtcr11;
    stc_rtc_time_t stcTime;
    boolean_t bAlarmMatch = TRUE;
    
#if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB)     
    // Recall data from VBAT domain to RTC count registers
    if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_CREAD))
    {
        return;
    }
#endif  
    /* Read interrupt flags */
    stcWtcr12 = pstcRtc->WTCR12_f; 
    /* Read alarm enable bits */
    stcWtcr11 = pstcRtc->WTCR11_f;

    /* Timer writing error Interrupt? */
    if (TRUE == stcWtcr12.INTERI)
    {
        /* Clear INTERI Flag */
        stcWtcr12.INTERI = FALSE;

        if (NULL != pstcRtcInternData->pfnTimeWrtErrIrqCb)
        {
            /* Callback */
            pstcRtcInternData->pfnTimeWrtErrIrqCb();
        }
    }

    /* Alarm Completion? */
    if (TRUE == stcWtcr12.INTALI)
    {
        /* Clear INTALI Flag */
        stcWtcr12.INTALI = FALSE;
        
        Rtc_ReadDateTime(pstcRtc, &stcTime);
        
        if(stcWtcr11.MIEN)
        {
            if(pstcRtc->ALMIR != DecToBcd(stcTime.u8Minute)) bAlarmMatch = FALSE;
        }
        if(stcWtcr11.HEN)
        {
            if(pstcRtc->ALHR != DecToBcd(stcTime.u8Hour)) bAlarmMatch = FALSE;
        }
        if(stcWtcr11.DEN)
        {
            if(pstcRtc->ALDR != DecToBcd(stcTime.u8Day)) bAlarmMatch = FALSE;
        }
        if(stcWtcr11.MOEN)
        {
            if(pstcRtc->ALMOR != DecToBcd(stcTime.u8Month)) bAlarmMatch = FALSE;
        }
        if(stcWtcr11.YEN)
        {
            if(pstcRtc->ALYR != DecToBcd(stcTime.u16Year - 2000u)) bAlarmMatch = FALSE;
        }

        if(bAlarmMatch == TRUE)
        {
            if (NULL != pstcRtcInternData->pfnAlarmIrqCb)
            {
                /* Callback */
                pstcRtcInternData->pfnAlarmIrqCb();
            }
        }
    }

    /* Timer Interrupt? */
    if (TRUE == stcWtcr12.INTTMI)
    {
        /* Clear INTTMI Flag */
        stcWtcr12.INTTMI = FALSE;

        if (NULL != pstcRtcInternData->pfnTimerIrqCb)
        {
            /* Callback */
            pstcRtcInternData->pfnTimerIrqCb();
        }
    }

    /* 0.5-Second Interrupt? */
    if (TRUE == stcWtcr12.INTSSI)
    {
        /* Clear INTSSI Flag */
        stcWtcr12.INTSSI = FALSE;

        if (NULL != pstcRtcInternData->pfnHalfSecondIrqCb)
        {
            /* Callback */
            pstcRtcInternData->pfnHalfSecondIrqCb();
        }
    }

    /* One Second Interrupt? */
    if (TRUE == stcWtcr12.INTSI)
    {
        /* Clear INTSI Flag */
        stcWtcr12.INTSI = FALSE;

        if (NULL != pstcRtcInternData->pfnOneSecondIrqCb)
        {
            /* Callback */
            pstcRtcInternData->pfnOneSecondIrqCb();
        }
    }

    /* One Minute Interrupt? */
    if (TRUE == stcWtcr12.INTMI)
    {
        /* Clear INTMI Flag */
        stcWtcr12.INTMI = FALSE;

        if (NULL != pstcRtcInternData->pfnOneMinuteIrqCb)
        {
            /* Callback */
            pstcRtcInternData->pfnOneMinuteIrqCb();
        }
    }

    /* One Hour Interrupt? */
    if (TRUE == stcWtcr12.INTHI)
    {
        /* Clear INTHI Flag */
        stcWtcr12.INTHI = FALSE;

        if (NULL != pstcRtcInternData->pfnOneHourIrqCb)
        {
            /* Callback */
            pstcRtcInternData->pfnOneHourIrqCb();
        }
    }

    pstcRtc->WTCR12_f = stcWtcr12;
    
#if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB)        
    // Save RTC count registers to VBAT domain
    if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_CWRITE))
    {
        return;
    }
#endif       
    
} /* RtcIrqHandler */

/**
 ******************************************************************************
 ** \brief Enable RTC (and Timer) Interrupt
 **
 ** \param [in]  pstcRtc         Pointer to RTC instance
 ** \param [in]  enIrqSel        RTC interrupt type
 **
 ** \retval Ok                    Interrupt selected is enabled
 ** \retval ErrorInvalidParameter If one of following conditions are met:
 **                               - pstcRtc == NULL
 **                               - enIrqSel out of range
 **
 ******************************************************************************/
en_result_t Rtc_EnableIrq(volatile stc_rtcn_t* pstcRtc, 
                          en_rtc_irq_sel_t enIrqSel)
{
    /* Check for valid pointer */
    if ((NULL == pstcRtc))
    {
        return ErrorInvalidParameter;
    }
    
#if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB)     
    // Recall data from VBAT domain to RTC count registers
    if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_CREAD))
    {
        return ErrorTimeout;
    }
#endif 
    
    switch (enIrqSel)
    {
        case RtcHalfSecondIrq:
            pstcRtc->WTCR13_f.INTSSIE = 1u;
            break;
        case RtcOneSecondIrq:
            pstcRtc->WTCR13_f.INTSIE = 1u;
            break;
        case RtcOneMinuteIrq:
            pstcRtc->WTCR13_f.INTMIE = 1u;
            break;
        case RtcOneHourIrq:
            pstcRtc->WTCR13_f.INTHIE = 1u;
            break;
        case RtcTimerIrq:
            pstcRtc->WTCR13_f.INTTMIE = 1u;
            break;
        case RtcAlarmIrq:
            pstcRtc->WTCR13_f.INTALIE = 1u;
            break;
        case RtcTimeRewriteErrorIrq:
            pstcRtc->WTCR13_f.INTERIE = 1u;
            break;
        default:
            return ErrorInvalidParameter;
    }

#if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB)        
    // Save RTC count registers to VBAT domain
    if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_CWRITE))
    {
        return ErrorTimeout;
    }
#endif      
    
    return (Ok);
} /* Rtc_EnableDisableInterrupts */

/**
 ******************************************************************************
 ** \brief Disable RTC (and Timer) Interrupt
 **
 ** \param [in]  pstcRtc         Pointer to RTC instance
 ** \param [in]  enIrqSel        RTC interrupt type
 **
 ** \retval Ok                    Interrupt selected is disabled
 ** \retval ErrorInvalidParameter If one of following conditions are met:
 **                               - pstcRtc == NULL
 **                               - enIrqSel out of range
 **
 ******************************************************************************/
en_result_t Rtc_DisableIrq(volatile stc_rtcn_t* pstcRtc, 
                           en_rtc_irq_sel_t enIrqSel)
{
    /* Check for valid pointer */
    if ((NULL == pstcRtc))
    {
        return ErrorInvalidParameter;
    }
    
#if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB)     
    // Recall data from VBAT domain to RTC count registers
    if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_CREAD))
    {
        return ErrorTimeout;
    }
#endif     
    
    switch (enIrqSel)
    {
        case RtcHalfSecondIrq:
            pstcRtc->WTCR13_f.INTSSIE = 0u;
            break;
        case RtcOneSecondIrq:
            pstcRtc->WTCR13_f.INTSIE = 0u;
            break;
        case RtcOneMinuteIrq:
            pstcRtc->WTCR13_f.INTMIE = 0u;
            break;
        case RtcOneHourIrq:
            pstcRtc->WTCR13_f.INTHIE = 0u;
            break;
        case RtcTimerIrq:
            pstcRtc->WTCR13_f.INTTMIE = 0u;
            break;
        case RtcAlarmIrq:
            pstcRtc->WTCR13_f.INTALIE = 0u;
            break;
        case RtcTimeRewriteErrorIrq:
            pstcRtc->WTCR13_f.INTERIE = 0u;
            break;
        default:
            return ErrorInvalidParameter;
    }

#if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB)        
    // Save RTC count registers to VBAT domain
    if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_CWRITE))
    {
        return ErrorTimeout;
    }
#endif       
    
    return (Ok);
} /* Rtc_EnableDisableInterrupts */

#endif

/**
 ******************************************************************************
 ** \brief Initialize RTC
 **
 ** This function initializes the RTC module 
 **
 ** \param [in] pstcRtc           Pointer to RTC instance
 ** \param [in] pstcConfig        Pointer to RTC configuration structure
 **
 ** \retval Ok                    RTC initialization completed normally
 ** \retval ErrorInvalidParameter If one of following conditions are met:
 **                               - pstcConfig == NULL
 **                               - Invalid value of a pstcConfig's element
 **
 ******************************************************************************/
en_result_t Rtc_Init(volatile stc_rtcn_t* pstcRtc , 
                     stc_rtc_config_t*    pstcConfig)
{
    // Pointer to internal data
    stc_rtc_intern_data_t* pstcRtcInternData ; 
    uint32_t u32Timeout = RTC_TIMEOUT;
    uint32_t u32Count;
    
    if((NULL == pstcConfig) || (NULL == pstcRtc)) 
    {
        return ErrorInvalidParameter;
    }
    
    /* Get pointer to internal data structure ... */
    pstcRtcInternData = RtcGetInternDataPtr( pstcRtc ) ;
    
    /* Check for instance available or not */
    if(NULL == pstcRtcInternData)
    {
        return ErrorInvalidParameter ;
    }
    
#if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB)
    /* Recall the data from VBAT to RTC clock registers of VBAT */
    if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_PREAD))
    {
        return ErrorTimeout;
    }
    
    /* Recall the data from VBAT to RTC registers of VBAT */
    if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_CREAD))
    {
        return ErrorTimeout;
    }
#endif
    
    if((TRUE == pstcConfig->bRunNotInit) && (1u == FM_RTC->WTCR10_f.ST)) /* RTC is running ? */
    {
        /*
         * If bRunNotInit is TRUE, don't do initialization RTC is aready running. 
         * However the interrupt callback and NVIC still need updating.
         */
        /* Clear all flags */
        pstcRtc->WTCR12  = 0x00u;
        
        /* if WTCR21 is cleared, re-configure RTC timer */
        if (0u == pstcRtc->WTCR21)
        {
            if(NULL != pstcConfig->pstcTimer)
            {
                switch (pstcConfig->pstcTimer->enMode)
                {
                    case RtcTimerOneshot:
                        pstcRtc->WTCR21_f.TMEN = 0u;
                        break;
                    case RtcTimerPeriod:
                        pstcRtc->WTCR21_f.TMEN = 1u;
                        break;
                    default:
                        return ErrorInvalidParameter;
                }
      
                if(pstcConfig->pstcTimer->u32TimerCycle > RTC_MAX_TIMER_SET_VALUE)
        {
                    return ErrorInvalidParameter;
                }
                
                u32Count = (pstcConfig->pstcTimer->u32TimerCycle*2u) - 1u;
                
                pstcRtc->WTTR0 = (uint8_t)u32Count;
                pstcRtc->WTTR1 = (uint8_t)(u32Count >> 8u);
                pstcRtc->WTTR2 = (uint8_t)(u32Count >> 16u);
            }
        }
        
    #if (PDL_INTERRUPT_ENABLE_RTC0 == PDL_ON)    
        /* if WTCR3 is cleared , re-configure interrupt */
        if(0u == pstcRtc->WTCR13)
        {
            if (NULL != pstcConfig->pstcIrqEn)
            {
                if(TRUE == pstcConfig->pstcIrqEn->bTimeRewriteErrorIrq)
                {
                    pstcRtc->WTCR13_f.INTERIE = 1u;
                }
                
                if(TRUE == pstcConfig->pstcIrqEn->bAlarmIrq)
                {
                    pstcRtc->WTCR13_f.INTALIE = 1u;
                }
                
                if(TRUE == pstcConfig->pstcIrqEn->bTimerIrq)
                {
                    pstcRtc->WTCR13_f.INTTMIE = 1u;
                }
                
                if(TRUE == pstcConfig->pstcIrqEn->bOneHourIrq)
                {
                    pstcRtc->WTCR13_f.INTHIE = 1u;
                }
                
                if(TRUE == pstcConfig->pstcIrqEn->bOneMinuteIrq)
                {
                    pstcRtc->WTCR13_f.INTMIE = 1u;
                }
                
                if(TRUE == pstcConfig->pstcIrqEn->bOneSecondIrq)
                {
                    pstcRtc->WTCR13_f.INTSIE = 1u;
                }
                
                if(TRUE == pstcConfig->pstcIrqEn->bHalfSecondIrq)
                {
                    pstcRtc->WTCR13_f.INTSSIE = 1u;
                }
            }       
        }
        /* Configure interrupt callback functions */
        if (NULL != pstcConfig->pstcIrqCb)
        {
            pstcRtcInternData->pfnTimeWrtErrIrqCb = pstcConfig->pstcIrqCb->pfnTimeWrtErrIrqCb;
            pstcRtcInternData->pfnAlarmIrqCb = pstcConfig->pstcIrqCb->pfnAlarmIrqCb;
            pstcRtcInternData->pfnTimerIrqCb = pstcConfig->pstcIrqCb->pfnTimerIrqCb;
            pstcRtcInternData->pfnOneHourIrqCb = pstcConfig->pstcIrqCb->pfnOneHourIrqCb;
            pstcRtcInternData->pfnOneMinuteIrqCb = pstcConfig->pstcIrqCb->pfnOneMinuteIrqCb;
            pstcRtcInternData->pfnOneSecondIrqCb = pstcConfig->pstcIrqCb->pfnOneSecondIrqCb;
            pstcRtcInternData->pfnHalfSecondIrqCb = pstcConfig->pstcIrqCb->pfnHalfSecondIrqCb;
        }
        
        /* Configure NVIC */
        if (TRUE == pstcConfig->bTouchNvic)
        {
            RtcInitNvic(pstcRtc);
        }
    #endif  
        return Ok;
    }
      
    /* For initialization: Force all bits of WTCR1 and WTCR2 (inclusive ST bit) */
    /*   to '0'  */
    pstcRtc->WTCR10 = 0u;
    pstcRtc->WTCR11 = 0u;
    pstcRtc->WTCR12 = 0u;
    pstcRtc->WTCR13 = 0u;
    pstcRtc->WTCR20 = 0u;
    pstcRtc->WTCR21 = 0u;
    
#if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB) 
    // Save RTC register data from buffer to VBAT domain (load WTCR1, WTCR2 to clear RTC)
    if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_CWRITE))
    {
        return ErrorTimeout;
    }
#endif    
    
#if (PDL_RTC_TYPE == PDL_RTC_WITHOUT_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_WITHOUT_VBAT_TYPEB)   
    /* Select source clock for RTC */
    switch (pstcConfig->enClkSel)
    {
        case RtcUseSubClk:
            /* Select sub clock as source clock */
            pstcRtc->WTCLKS_f.WTCLKS = 0u;
            /* Wait until sub clock stable */
            while(1)
            {
                if(0x02u == pstcRtc->WTCLKM_f.WTCLKM)
                {
                    break;
                }
                else
                {
                    u32Timeout--;
                    if(0u == u32Timeout)
                    {
                        return ErrorTimeout;
                    }
                }
            }
            
            pstcRtc->WTBR = (pstcConfig->u32ClkPrescaler/4u) - 1u; /* set 0.5s as sub-second interval */
            break;
        case RtcuseMainClk:
            /* Select main clock as source clock */
            pstcRtc->WTCLKS_f.WTCLKS = 1u;
            /* Wait until main clock stable */
            while(1)
            {
                if(0x03u == pstcRtc->WTCLKM_f.WTCLKM)
                {
                    break;
                }
                else
                {
                    u32Timeout--;
                    if(0u == u32Timeout)
                    {
                        return ErrorTimeout;
                    }
                }
            }
            pstcRtc->WTBR = (pstcConfig->u32ClkPrescaler/4u) - 1u; /* set 0.5s as sub-second interval */
            break;  
        default:
            return ErrorInvalidParameter;
    }
#endif    
       
    if(pstcConfig->bEnSuboutDivider == TRUE)
    {
        /* Disable divider */
        pstcRtc->WTDIVEN_f.WTDIVEN = 0u;
        
        /* Set Divider ratio */
        switch (pstcConfig->enDividerRatio)
        {
            case RtcDivRatio1:
            case RtcDivRatio2:
            case RtcDivRatio4:
            case RtcDivRatio8:
            case RtcDivRatio16:
            case RtcDivRatio32:
            case RtcDivRatio64:
            case RtcDivRatio128:
            case RtcDivRatio256:
            case RtcDivRatio512:
            case RtcDivRatio1024:
            case RtcDivRatio2048:
            case RtcDivRatio4096:
            case RtcDivRatio8192:
            case RtcDivRatio16384:
            case RtcDivRatio32768:
                pstcRtc->WTDIV = pstcConfig->enDividerRatio;
                break;
            default:
                return  ErrorInvalidParameter;
        }
        
        /*  Enable divider */
        u32Timeout = RTC_TIMEOUT;
        pstcRtc->WTDIVEN_f.WTDIVEN = 1u;
        
    #if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB) 
        // Save RTC clock register data from buffer to VBAT domain (load WTDIVEN)
        if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_PWRITE))
        {
            return ErrorTimeout;
        }
    #endif
        
        while(TRUE != pstcRtc->WTDIVEN_f.WTDIVRDY)
        {
        #if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB)
            /* Recall the data from VBAT to RTC clock registers of VBAT */
            if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_PREAD))
            {
                return ErrorTimeout;
            }
        #endif

            u32Timeout--;
            if(0u == u32Timeout)
            {
                return ErrorInvalidParameter;
            }
        }
    }
    
    /* Initial frequency correction module */
    if(NULL != pstcConfig->pstcFreqCorrConfig)
    {
        if(pstcConfig->pstcFreqCorrConfig->u16FreqCorrValue > RTC_MAX_FREQ_CORR_VALUE)
        {
            return ErrorInvalidParameter;
        }
        pstcRtc->WTCAL0 = (uint8_t)pstcConfig->pstcFreqCorrConfig->u16FreqCorrValue ;
          
    #if (PDL_RTC_TYPE != PDL_RTC_WITHOUT_VBAT_TYPEA) 
        pstcRtc->WTCAL1 = (uint8_t)(pstcConfig->pstcFreqCorrConfig->u16FreqCorrValue >> 8u) ;
        if(pstcConfig->pstcFreqCorrConfig->u16FreqCorrCycle > RTC_MAX_FREQ_CORR_CYCLE_SET_VALUE)
        {
            return ErrorInvalidParameter;
        }
        pstcRtc->WTCALPRD = pstcConfig->pstcFreqCorrConfig->u16FreqCorrCycle - 1;
    #endif    
    }
    
    /* Configure RTCCO output */
    switch (pstcConfig->enRtccoSel)
    {
        case RtccoOutput2Hz:
            pstcRtc->WTCOSEL_f.WTCOSEL = 0u; 
            break;
        case RtccoOutput1Hz:
            pstcRtc->WTCOSEL_f.WTCOSEL = 1u;
            break;
        default:
            return ErrorInvalidParameter;
    }
    
    /* Initialize time and date */
    if(NULL != pstcConfig->pstcTimeDate)
    {
        pstcRtc->WTSR  = DecToBcd(pstcConfig->pstcTimeDate->u8Second);
        pstcRtc->WTMIR = DecToBcd(pstcConfig->pstcTimeDate->u8Minute);
        pstcRtc->WTHR  = DecToBcd(pstcConfig->pstcTimeDate->u8Hour);
        pstcRtc->WTDR  = DecToBcd(pstcConfig->pstcTimeDate->u8Day);
        pstcRtc->WTMOR = DecToBcd(pstcConfig->pstcTimeDate->u8Month);
        pstcRtc->WTYR  = DecToBcd(pstcConfig->pstcTimeDate->u16Year - 2000u);
        pstcRtc->WTDW  = DecToBcd(pstcConfig->pstcTimeDate->u8DayOfWeek);
    }
    
    /* Initialize Alarm */
    if(NULL != pstcConfig->pstcAlarm)
    {
        pstcRtc->ALMIR = DecToBcd(pstcConfig->pstcAlarm->u8Minute);
        pstcRtc->ALHR  = DecToBcd(pstcConfig->pstcAlarm->u8Hour);
        pstcRtc->ALDR  = DecToBcd(pstcConfig->pstcAlarm->u8Day);
        pstcRtc->ALMOR = DecToBcd(pstcConfig->pstcAlarm->u8Month);
        pstcRtc->ALYR  = DecToBcd(pstcConfig->pstcAlarm->u16Year - 2000u);
    }
    
    if(NULL != pstcConfig->pstcTimer)
    {
        switch (pstcConfig->pstcTimer->enMode)
        {
            case RtcTimerOneshot:
                pstcRtc->WTCR21_f.TMEN = 0u;
                break;
            case RtcTimerPeriod:
                pstcRtc->WTCR21_f.TMEN = 1u;
                break;
            default:
                return ErrorInvalidParameter;
        }
        
        if(pstcConfig->pstcTimer->u32TimerCycle > RTC_MAX_TIMER_SET_VALUE)
        {
            return ErrorInvalidParameter;
        }
        
        u32Count = (pstcConfig->pstcTimer->u32TimerCycle*2u) - 1u;
        
        pstcRtc->WTTR0 = (uint8_t)u32Count;
        pstcRtc->WTTR1 = (uint8_t)(u32Count >> 8u);
        pstcRtc->WTTR2 = (uint8_t)(u32Count >> 16u);
    }

#if (PDL_INTERRUPT_ENABLE_RTC0 == PDL_ON)    
    /* Configure interrupt */
    if (NULL != pstcConfig->pstcIrqEn)
    {
        if(TRUE == pstcConfig->pstcIrqEn->bTimeRewriteErrorIrq)
        {
            pstcRtc->WTCR13_f.INTERIE = 1u;
        }
        
        if(TRUE == pstcConfig->pstcIrqEn->bAlarmIrq)
        {
            pstcRtc->WTCR13_f.INTALIE = 1u;
        }
        
        if(TRUE == pstcConfig->pstcIrqEn->bTimerIrq)
        {
            pstcRtc->WTCR13_f.INTTMIE = 1u;
        }
        
        if(TRUE == pstcConfig->pstcIrqEn->bOneHourIrq)
        {
            pstcRtc->WTCR13_f.INTHIE = 1u;
        }
        
        if(TRUE == pstcConfig->pstcIrqEn->bOneMinuteIrq)
        {
            pstcRtc->WTCR13_f.INTMIE = 1u;
        }
        
        if(TRUE == pstcConfig->pstcIrqEn->bOneSecondIrq)
        {
            pstcRtc->WTCR13_f.INTSIE = 1u;
        }
        
        if(TRUE == pstcConfig->pstcIrqEn->bHalfSecondIrq)
        {
            pstcRtc->WTCR13_f.INTSSIE = 1u;
        }
    }
    
    /* Configure interrupt callback functions */
    if (NULL != pstcConfig->pstcIrqCb)
    {
        pstcRtcInternData->pfnTimeWrtErrIrqCb = pstcConfig->pstcIrqCb->pfnTimeWrtErrIrqCb;
        pstcRtcInternData->pfnAlarmIrqCb = pstcConfig->pstcIrqCb->pfnAlarmIrqCb;
        pstcRtcInternData->pfnTimerIrqCb = pstcConfig->pstcIrqCb->pfnTimerIrqCb;
        pstcRtcInternData->pfnOneHourIrqCb = pstcConfig->pstcIrqCb->pfnOneHourIrqCb;
        pstcRtcInternData->pfnOneMinuteIrqCb = pstcConfig->pstcIrqCb->pfnOneMinuteIrqCb;
        pstcRtcInternData->pfnOneSecondIrqCb = pstcConfig->pstcIrqCb->pfnOneSecondIrqCb;
        pstcRtcInternData->pfnHalfSecondIrqCb = pstcConfig->pstcIrqCb->pfnHalfSecondIrqCb;
    }
    
    /* Configure NVIC */
    if (TRUE == pstcConfig->bTouchNvic)
    {
        RtcInitNvic(pstcRtc);
    }
#endif    

#if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB) 
    // Save RTC clock register data from buffer to VBAT domain
    if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_PWRITE))
    {
        return ErrorTimeout;
    }
    // Save RTC register data from buffer to VBAT domain
    if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_CWRITE))
    {
        return ErrorTimeout;
    }
#endif
    
    return (Ok);
} /* Rtc_Init */

/**
 ******************************************************************************
 ** \brief De-Initialize RTC
 **
 ** \param [in] pstcRtc            Pointer to RTC instance
 ** \param [in] bTouchNvic         Touch NVIC or not
 **
 ** This function stops and resets the RTC module and its interrupts.
 **
 ** \retval  Ok                     RTC de-initialization normally
 **
 ******************************************************************************/
en_result_t Rtc_DeInit(volatile stc_rtcn_t* pstcRtc, boolean_t bTouchNvic)
{
    /* Set all registers of RTC instance to '0' */
    pstcRtc->WTCR10 = 0u;
    pstcRtc->WTCR11 = 0u;
    pstcRtc->WTCR12 = 0u;
    pstcRtc->WTCR13 = 0u;
    pstcRtc->WTCR20 = 0u;
    pstcRtc->WTCR21 = 0u;
    pstcRtc->WTCAL0    = 0u;
#if (PDL_RTC_TYPE != PDL_RTC_WITHOUT_VBAT_TYPEA)    
    pstcRtc->WTCAL1    = 0u;
#endif    
    pstcRtc->WTCALEN  = 0u;
    pstcRtc->WTDIV    = 0u;
    pstcRtc->WTDIVEN  = 0u;
    pstcRtc->WTCALPRD = 0u;
    pstcRtc->WTCOSEL  = 0u;
    pstcRtc->WTSR     = 0u;
    pstcRtc->WTMIR    = 0u;
    pstcRtc->WTHR     = 0u;
    pstcRtc->WTDR     = 0u;
    pstcRtc->WTDW     = 0u;
    pstcRtc->WTMOR    = 0u;
    pstcRtc->WTYR     = 0u;
    pstcRtc->ALMIR    = 0u;
    pstcRtc->ALHR     = 0u;
    pstcRtc->ALDR     = 0u;
    pstcRtc->ALMOR    = 0u;
    pstcRtc->ALYR     = 0u;
        
#if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB) 
    // Save RTC clock register data from buffer to VBAT domain
    if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_PWRITE))
    {
        return ErrorTimeout;
    }
    // Save RTC register data from buffer to VBAT domain
    if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_CWRITE))
    {
        return ErrorTimeout;
    }
#endif    

#if (PDL_INTERRUPT_ENABLE_RTC0 == PDL_ON)      
    if(TRUE == bTouchNvic)
    {
        RtcDeInitNvic(pstcRtc);
    }
#endif    
    
    return (Ok);
} /* Rtc_DeInit */


/**
 ******************************************************************************
 ** \brief Enable functions of RTC
 **
 ** These functions includes RTC counting, Timer, Alarm compariron and frequency
 ** correction module.
 **
 ** \param [in] pstcRtc      Pointer to RTC instance
 ** \param enFunc  Function types
 ** \arg   RtcCount          RTC counting
 ** \arg   RtcTimer          RTC timer
 ** \arg   RtcFreqCorr       RTC frequency correction module
 ** \arg   RtcAlarmYearEn    Year comparison enable of RTC alarm
 ** \arg   RtcAlarmMonthEn   Month comparison enable of RTC alarm
 ** \arg   RtcAlarmDayEn     Day comparison enable of RTC alarm
 ** \arg   RtcAlarmHourEn    Hour comparison enable of RTC alarm 
 ** \arg   RtcAlarmMinEn     Minute comparison enable of RTC alarm
 **
 ** \retval Ok                    RTC functions enabled normally
 ** \retval ErrorInvalidParameter If one of following conditions are met:
 **                               - Invalid value of enFunc 
 **
 ******************************************************************************/
en_result_t Rtc_EnableFunc(volatile stc_rtcn_t* pstcRtc, en_rtc_func_t enFunc)
{
    // Check parameter
    if(NULL == pstcRtc)
    {
        return ErrorInvalidParameter;
    }

#if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB)     
    // Recall data from VBAT domain to RTC count registers
    if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_CREAD))
    {
        return ErrorTimeout;
    }
    
    // Recall data from VBAT domain to RTC clock registers
    if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_PREAD))
    {
        return ErrorTimeout;
    }
#endif  
    
    switch (enFunc)
    {
        case RtcCount:
            pstcRtc->WTCR10_f.ST = 1u;
            break;
        case RtcTimer:
            pstcRtc->WTCR21_f.TMST = 1u;
            break;
        case RtcFreqCorr:
            pstcRtc->WTCALEN_f.WTCALEN = 1u;
            break;
        case RtcAlarmYearEn:
            pstcRtc->WTCR11_f.YEN = 1u;
            break;
        case RtcAlarmMonthEn:
            pstcRtc->WTCR11_f.MOEN = 1u;
            break;
        case RtcAlarmDayEn:
            pstcRtc->WTCR11_f.DEN = 1u;
            break;
        case RtcAlarmHourEn:
            pstcRtc->WTCR11_f.HEN = 1u;
            break;
        case RtcAlarmMinEn:
            pstcRtc->WTCR11_f.MIEN = 1u;
            break;
        default:
            return ErrorInvalidParameter;
    }
    
#if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB)        
    // Save RTC count registers to VBAT domain
    if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_CWRITE))
    {
        return ErrorTimeout;
    }
    
    // Save RTC clock registers to VBAT domain
    if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_PWRITE))
    {
        return ErrorTimeout;
    }
#endif    
    
    return Ok;
}
       
/**
 ******************************************************************************
 ** \brief Disable functions of RTC
 **
 ** These functions includes RTC counting, Timer, Alarm compariron and frequency
 ** correction module.
 **
 ** \param [in] pstcRtc      Pointer to RTC instance
 ** \param enFunc  Function types
 ** \arg   RtcCount          RTC counting
 ** \arg   RtcTimer          RTC timer
 ** \arg   RtcFreqCorr       RTC frequency correction module
 ** \arg   RtcAlarmYearEn    Year comparison enable of RTC alarm
 ** \arg   RtcAlarmMonthEn   Month comparison enable of RTC alarm
 ** \arg   RtcAlarmDayEn     Day comparison enable of RTC alarm
 ** \arg   RtcAlarmHourEn    Hour comparison enable of RTC alarm 
 ** \arg   RtcAlarmMinEn     Minute comparison enable of RTC alarm
 **
 ** \retval Ok                    RTC functions disabled normally
 ** \retval ErrorInvalidParameter If one of following conditions are met:
 **                               - Invalid value of enFunc 
 **
 ******************************************************************************/
en_result_t Rtc_DisableFunc(volatile stc_rtcn_t* pstcRtc, en_rtc_func_t enFunc)
{
    // Check parameter
    if(NULL == pstcRtc)
    {
        return ErrorInvalidParameter;
    }
    
#if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB)     
    // Recall data from VBAT domain to RTC count registers
    if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_CREAD))
    {
        return ErrorTimeout;
    }
#endif     
  
    switch (enFunc)
    {
        case RtcCount:
            pstcRtc->WTCR10_f.ST = 0u;
            break;
        case RtcTimer:
            pstcRtc->WTCR21_f.TMST = 0u;
            break;
        case RtcFreqCorr:
            pstcRtc->WTCALEN_f.WTCALEN = 0u;
            break;
        case RtcAlarmYearEn:
            pstcRtc->WTCR11_f.YEN = 0u;
            break;
        case RtcAlarmMonthEn:
            pstcRtc->WTCR11_f.MOEN = 0u;
            break;
        case RtcAlarmDayEn:
            pstcRtc->WTCR11_f.DEN = 0u;
            break;
        case RtcAlarmHourEn:
            pstcRtc->WTCR11_f.HEN = 0u;
            break;
        case RtcAlarmMinEn:
            pstcRtc->WTCR11_f.MIEN = 0u;
            break;
        default:
            return ErrorInvalidParameter;
    }

#if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB)        
    // Save RTC count registers to VBAT domain
    if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_CWRITE))
    {
        return ErrorTimeout;
    }
#endif 
    
    return Ok;
}

/**
 ******************************************************************************
 ** \brief Get RTC interrupt flag according to interrupt type
 **
 ** \param [in] pstcRtc            Pointer to RTC instance
 ** \param [in] enIrqSel           RTC interrupt type
 ** \arg   RtcRewriteError         RTC rewrite error interrupt flag
 ** \arg   RtcAlarmIrq             RTC alarm interrupt flag
 ** \arg   RtcTimerUnderFlow       RTC Timer underflow interrupt flag
 ** \arg   RtcOneHourFlag          1-hour count-up interrupt flag
 ** \arg   RtcOneMiniteFlag        1-Minute count-up interrupt flag
 ** \arg   RtcOneSecondFlag        1-second count-up interrupt flag
 ** \arg   RtcHalfSecondFlag       0.5-second count-up interrupt flag
 ** 
 ** \retval FALSE                 If one of following conditions are met:
 **                               - No RTC rewrite error [enStatus = RtcRewriteError]
 **                               - No element of alarm matchs [enStatus = RtcAlarmElementMatch]     
 **                               - RTC timer underflow doesn't occurs [enStatus = RtcTimerUnderFlow]
 **                               - 1-hour count-up flag is not set [enStatus = RtcOneHourFlag]
 **                               - 1-minute count-up flag is not set [enStatus = RtcOneMiniteFlag]
 **                               - 1-second count-up flag is not set [enStatus = RtcOneSecondFlag]
 **                               - 0.5-second count-up flag is not set [enStatus = RtcHalfSecondFlag]
 **                               - RTC is running [enStatus = RtcRunStatus]
 **                               - RTC timer is running [enStatus = RtcTimerStatus]
 ** \retval TRUE                 If one of following conditions are met:
 **                               - RTC rewrite error occurs [enStatus = RtcRewriteError]
 **                               - One of an element of alarm matchs [enStatus = RtcAlarmElementMatch]     
 **                               - RTC timer underflow doesn't occurs [enStatus = RtcTimerUnderFlow]
 **                               - 1-hour count-up flag is set [enStatus = RtcOneHourFlag]
 **                               - 1-minute count-up flag is set [enStatus = RtcOneMiniteFlag]
 **                               - 1-second count-up flag is set [enStatus = RtcOneSecondFlag]
 **                               - 0.5-second count-up flag is set [enStatus = RtcHalfSecondFlag]
 ** 
 ******************************************************************************/
boolean_t Rtc_GetIrqFlag(volatile stc_rtcn_t* pstcRtc, en_rtc_irq_sel_t enIrqSel)
{
    boolean_t bRet = FALSE;
  
    /* Check parameter */
    if (NULL == pstcRtc)
    {
        return FALSE;
    }
  
#if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB)     
    // Recall data from VBAT domain to RTC count registers
    if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_CREAD))
    {
        return FALSE;
    }
#endif      
    
    switch (enIrqSel)
    {
        case RtcHalfSecondIrq:
            (pstcRtc->WTCR12_f.INTSSI == 1u) ? (bRet = TRUE) : (bRet = FALSE);
            break;
        case RtcOneSecondIrq:
            (pstcRtc->WTCR12_f.INTSI == 1u) ? (bRet = TRUE) : (bRet = FALSE);
            break;
        case RtcOneMinuteIrq:
            (pstcRtc->WTCR12_f.INTMI == 1u) ? (bRet = TRUE) : (bRet = FALSE);
            break;
        case RtcOneHourIrq:
            (pstcRtc->WTCR12_f.INTHI == 1u) ? (bRet = TRUE) : (bRet = FALSE);
            break;
        case RtcTimerIrq:
            (pstcRtc->WTCR12_f.INTTMI == 1u) ? (bRet = TRUE) : (bRet = FALSE);
            break;
        case RtcAlarmIrq:
            (pstcRtc->WTCR12_f.INTALI == 1u) ? (bRet = TRUE) : (bRet = FALSE);
            break;
        case RtcTimeRewriteErrorIrq:
            (pstcRtc->WTCR12_f.INTERI == 1u) ? (bRet = TRUE) : (bRet = FALSE);
            break;
        default:
            break;
    }
    
    return bRet;
}

/**
 ******************************************************************************
 ** \brief Clear RTC interrupt flag according to interrupt type
 **
 ** \param [in] pstcRtc            Pointer to RTC instance
 ** \param [in] enIrqSel           RTC interrupt type
 ** \arg   RtcRewriteError         RTC rewrite error interrupt flag
 ** \arg   RtcAlarmIrq             RTC alarm interrupt flag
 ** \arg   RtcTimerUnderFlow       RTC Timer underflow interrupt flag
 ** \arg   RtcOneHourFlag          1-hour count-up interrupt flag
 ** \arg   RtcOneMiniteFlag        1-Minute count-up interrupt flag
 ** \arg   RtcOneSecondFlag        1-second count-up interrupt flag
 ** \arg   RtcHalfSecondFlag       0.5-second count-up interrupt flag
 ** 
 ** \retval Ok                    RTC interrupt flag is cleared normally
 ** \retval ErrorInvalidParameter If one of following conditions are met:
 **                               - Invalid value of enStatus 
 ** 
 ******************************************************************************/
en_result_t Rtc_ClrIrqFlag(volatile stc_rtcn_t* pstcRtc, en_rtc_irq_sel_t enIrqSel)
{      
    if (NULL == pstcRtc)
    {
        return ErrorInvalidParameter;
    }

#if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB)     
    // Recall data from VBAT domain to RTC count registers
    if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_CREAD))
    {
        return ErrorTimeout;
    }
#endif  
    
    switch (enIrqSel)
    {
        case RtcHalfSecondIrq:
            pstcRtc->WTCR12_f.INTSSI = 0u;
            break;
        case RtcOneSecondIrq:
            pstcRtc->WTCR12_f.INTSI = 0u;
            break;
        case RtcOneMinuteIrq:
            pstcRtc->WTCR12_f.INTMI = 0u;
            break;
        case RtcOneHourIrq:
            pstcRtc->WTCR12_f.INTHI = 0u;
            break;
        case RtcTimerIrq:
            pstcRtc->WTCR12_f.INTTMI = 0u;
            break;
        case RtcAlarmIrq:
            pstcRtc->WTCR12_f.INTALI = 0u;
            break;
        case RtcTimeRewriteErrorIrq:
            pstcRtc->WTCR12_f.INTERI = 0u;
            break;
        default:
            return ErrorInvalidParameter;
    }

#if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB)        
    // Save RTC count registers to VBAT domain
    if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_CWRITE))
    {
        return ErrorTimeout;
    }
#endif 
    
    return Ok;
}

/**
 ******************************************************************************
 ** \brief Get status of RTC according to status type
 **
 ** \param [in] pstcRtc            Pointer to RTC instance
 ** \param [in] enStatus           RTC status
 ** \arg   RtcRunStatus            RTC counter run status
 ** \arg   RtcTimerStatus          RTC timer status
 ** 
 ** \retval FALSE                 If one of following conditions are met:
 **                               - RTC is running [enStatus = RtcRunStatus]
 **                               - RTC timer is running [enStatus = RtcTimerStatus]
 ** \retval TRUE                 If one of following conditions are met:
 **                               - RTC counter stops [enStatus = RtcRunStatus]
 **                               - RTC timer stops [enStatus = RtcTimerStatus]
 ** 
 ******************************************************************************/
boolean_t Rtc_GetStatus(volatile stc_rtcn_t* pstcRtc, en_rtc_status_t enStatus)
{
    boolean_t bRet = FALSE;

#if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB)     
    // Recall data from VBAT domain to RTC count registers
    if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_CREAD))
    {
        return ErrorTimeout;
    }
#endif
    
    switch(enStatus)
    {
        case RtcRunStatus:
            (pstcRtc->WTCR10_f.RUN == 1u) ? (bRet = TRUE) : (bRet = FALSE);
            break;
        case RtcTimerStatus:
            (pstcRtc->WTCR21_f.TMRUN == 1u) ? (bRet = TRUE) : (bRet = FALSE);
            break;
        default:
            break;  
    }
    
    return bRet;
}
        
/**
 ******************************************************************************
 ** \brief Reset RTC
 **
 ** \param [in] pstcRtc            Pointer to RTC instance
 ******************************************************************************/
en_result_t Rtc_Reset(volatile stc_rtcn_t* pstcRtc)
{
#if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB)     
    // Recall data from VBAT domain to RTC count registers
    if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_CREAD))
    {
        return ErrorTimeout;
    }
#endif
  
    pstcRtc->WTCR10_f.SRST = 1;

#if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB)        
    // Save RTC count registers to VBAT domain
    if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_CWRITE))
    {
        return ErrorTimeout;
    }
#endif     
    
    return Ok;
}
        
/**
 ******************************************************************************
 ** \brief Set time and date after RTC is running
 **
 ** \param [in] pstcRtc           Pointer to RTC instance
 ** \param [in] pstcTimeDate      Time and date structure
 ** \param [in] bContinue         RTC counting continues or not
 ** \arg  FALSE     RTC 1-second counting reset during setting procedure
 ** \arg  TRUE      RTC 1-second continues during setting procedure
 ** \param [in] bUpdateTime       Update time or not
 ** \arg  FALSE     Don't update time
 ** \arg  TRUE      Update time
 ** \param [in] bUpdateDate       Update date or not
 ** \arg  FALSE     Don't update date
 ** \arg  TRUE      Update date
 ** 
 ** \retval Ok                    RTC time and date is set normally
 ** \retval ErrorInvalidParameter If one of following conditions are met:
 **                               - pstcTimeDate == NULL 
 ** \retval ErrorInvalidMode      RTC is running
 ** \retval ErrorTimeout          RTC rewrite timeout            
 ** 
 ******************************************************************************/
en_result_t Rtc_SetDateTime(volatile stc_rtcn_t* pstcRtc, 
                            stc_rtc_time_t* pstcTimeDate, 
                            boolean_t bContinue,
                            boolean_t bUpdateTime,
                            boolean_t bUpdateDate)
{
    uint32_t u32TimeOut;
    stc_rtc_wtcr13_field_t stcWtcr13;
  
    if((NULL == pstcTimeDate) || (NULL == pstcRtc))
    {
        return ErrorInvalidParameter;  
    }
    
#if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB)     
    // Recall data from VBAT domain to RTC count registers (load WTCR10)
    if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_CREAD))
    {
        return ErrorTimeout;
    }
#endif    
    
     /* Don't set time and date with this function before RTC starts */
    if(0u == pstcRtc->WTCR10_f.RUN)
    {
        return ErrorInvalidMode;
    }

    if(1u == pstcRtc->WTCR10_f.BUSY)
    {
        /*  Wait until rewrite idle */
        u32TimeOut = RTC_TIMEOUT;
        while(1)
        {
            if(TRUE == bContinue)
            {
                if((0u == pstcRtc->WTCR10_f.BUSY) && (0u == pstcRtc->WTCR10_f.SCST))
                {
                    break;
                }
            }
            else
            {
                if((0u == pstcRtc->WTCR10_f.BUSY) && (0u == pstcRtc->WTCR10_f.SCRST))
                {
                    break;
                }
            }
            u32TimeOut--;
            if(0u == u32TimeOut)
            {
                return ErrorTimeout;
            }

        #if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB)     
            // Recall data from VBAT domain to RTC count registers (load WTCR10)
            if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_CREAD))
            {
                return ErrorTimeout;
            }
        #endif             
        }       
    }
             
    /* Backup Interrupt Settings */
    stcWtcr13 = pstcRtc->WTCR13_f;
    
    /* Disable RTC NVIC */
    RtcDisableNvic();
    
    /* Disable interrupt */
    pstcRtc->WTCR13 = 0u;
      
    /* Clear every second interrupt */
    pstcRtc->WTCR12_f.INTSI = 0u;
    
    /* Enable every second interrupt */
    pstcRtc->WTCR13_f.INTSIE = 1u;
                    
#if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB)        
    // Save RTC count registers to VBAT domain (Write WTCR10, WTCR13)
    if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_CWRITE))
    {
        return ErrorTimeout;
    }
#endif  

    /* Wait to occur interrupt */
    while (1u != pstcRtc->WTCR12_f.INTSI)
    {
    #if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB)     
        // Recall data from VBAT domain to RTC count registers (load WTCR12)
        if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_CREAD))
        {
            return ErrorTimeout;
        }
    #endif  
    }    
    
    /* Clear every second interrupt */
    pstcRtc->WTCR12_f.INTSI = 0u;
    
    if(TRUE == bContinue)
    {
        /* Stop 1 second output */
        pstcRtc->WTCR10_f.SCST = 1u;
    }
    else
    {
        /* Reset 1 second counter */
        pstcRtc->WTCR10_f.SCRST = 1u;
    }
    
    /* Set time and date */
    if (TRUE == bUpdateTime)
    {
        pstcRtc->WTSR  = DecToBcd(pstcTimeDate->u8Second);
        pstcRtc->WTMIR = DecToBcd(pstcTimeDate->u8Minute);
        pstcRtc->WTHR  = DecToBcd(pstcTimeDate->u8Hour);
    }
    
    if (TRUE == bUpdateDate)
    {
        pstcRtc->WTDR  = DecToBcd(pstcTimeDate->u8Day);
        pstcRtc->WTMOR = DecToBcd(pstcTimeDate->u8Month);
        pstcRtc->WTYR  = DecToBcd(pstcTimeDate->u16Year - 2000u);
        pstcRtc->WTDW  = DecToBcd(pstcTimeDate->u8DayOfWeek);
    }

#if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB)        
    // Save RTC count registers to VBAT domain (Write time registes, WTCR10, WTCR12)
    if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_CWRITE))
    {
        return ErrorTimeout;
    }
#endif   
    
    if(TRUE == bContinue)
    {
        /* Refresh the time and date */
        pstcRtc->WTCR10_f.SCST = 0u;
    }
    else
    {
        /* Refresh the time and date  */
        pstcRtc->WTCR10_f.SCRST = 0u;
    }
    
    /* Retrieve Interrupt Settings */
    pstcRtc->WTCR13_f = stcWtcr13;
    
#if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB)        
    // Save RTC count registers to VBAT domain (Write WTCR10, WTCR13)
    if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_CWRITE))
    {
        return ErrorTimeout;
    }
#endif      
    
    /* Retrieve NVIC */
    RtcRestoreNvic();    
    
    /* Wait until the transfer from time and date registers to counters end */
    u32TimeOut = RTC_TIMEOUT;
    while(1)
    {
        if(TRUE == bContinue)
        {
            if((0u == pstcRtc->WTCR10_f.BUSY) && (0u == pstcRtc->WTCR10_f.SCST))
            {
                break;
            }
        }
        else
        {
            if((0u == pstcRtc->WTCR10_f.BUSY) && (0u == pstcRtc->WTCR10_f.SCRST))
            {
                break;
            }
        }
        u32TimeOut--;
        if(0u == u32TimeOut)
        {
            return ErrorTimeout;
        }
        
    #if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB)     
        // Recall data from VBAT domain to RTC count registers (Load WTCR10)
        if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_CREAD))
        {
            return ErrorTimeout;
        }
    #endif      
    }    
                 
    return Ok;            
}
                
/**
 ******************************************************************************
 ** \brief Read RTC time and date 
 **
 ** \param [in] pstcRtc           Pointer to RTC instance
 ** \param [in] pstcTimeDate      Time and date structure
 ** 
 ** \retval Ok                    RTC time and date is read normally
 ** \retval ErrorInvalidParameter If one of following conditions are met:
 **                               - pstcTimeDate == NULL 
 ** \retval ErrorTimeout          Time and date read timeout
 ** 
 ******************************************************************************/
en_result_t Rtc_ReadDateTime(volatile stc_rtcn_t* pstcRtc, stc_rtc_time_t* pstcTimeDate)
{
#if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB)    
#else  
    uint32_t u32TimeOut;
#endif    
    uint8_t  u8DayOfWeek, u8BcdSec, u8BcdMin, u8BcdHour, u8Day, u8Month, u16Year = 0u;
  
    if(pstcTimeDate == NULL)
    {
        return ErrorInvalidParameter;
    }
    
#if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB)     
    // Recall data from VBAT domain to RTC count registers
    if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_CREAD))
    {
        return ErrorTimeout;
    }
#else    
    /* Start read */
    pstcRtc->WTCR20_f.CREAD = 1u;
    
    /* Wait until read finish */
    u32TimeOut = RTC_TIMEOUT;
    while(1)
    {
        if(0u == FM_RTC->WTCR20_f.CREAD)
        {
            break;
        }
        u32TimeOut --;
        if(0u == u32TimeOut)
        {
            RtcRestoreNvic(); 
            return ErrorTimeout;
        }
        
    }
#endif      
    /* Read time and date */    
    u8BcdSec  = pstcRtc->WTSR;
    u8BcdMin  = pstcRtc->WTMIR;
    u8BcdHour = pstcRtc->WTHR;
    u8Day     = pstcRtc->WTDR;
    u8Month   = pstcRtc->WTMOR;
    u16Year    = pstcRtc->WTYR;
    u8DayOfWeek = pstcRtc->WTDW;
    
    pstcTimeDate->u8Second = BcdToDec(u8BcdSec);
    pstcTimeDate->u8Minute = BcdToDec(u8BcdMin);
    pstcTimeDate->u8Hour   = BcdToDec(u8BcdHour);
    pstcTimeDate->u8Day    = BcdToDec(u8Day);
    pstcTimeDate->u8Month  = BcdToDec(u8Month);
    pstcTimeDate->u16Year  = BcdToDec(u16Year) + 2000u;
    pstcTimeDate->u8DayOfWeek = BcdToDec(u8DayOfWeek);
    
    return Ok;
}

/**
 ******************************************************************************
 ** \brief Set time and date information of Alarm
 **
 ** \param [in] pstcRtc           Pointer to RTC instance
 ** \param [in] pstcAlarm         Time and date structure 
 ** \param [in] bUpdateAlarmTime  Update alarm time or not
 ** \param [in] bUpdateAlarmDate  Update alarm date or not

 ** \retval Ok                    RTC Alarm time and date is set normally
 ** \retval ErrorInvalidParameter If one of following conditions are met:
 **                               - pstcAlarm == NULL 
 ** 
 ******************************************************************************/
en_result_t Rtc_SetAlarmDateTime(volatile stc_rtcn_t* pstcRtc, 
                                 stc_rtc_alarm_t* pstcAlarm,
                                 boolean_t bUpdateAlarmTime,
                                 boolean_t bUpdateAlarmDate)
{
    if(NULL == pstcAlarm)
    {
        return ErrorInvalidParameter;
    }
  
#if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB)     
    // Recall data from VBAT domain to RTC count registers
    if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_CREAD))
    {
        return ErrorTimeout;
    }
#endif  
    
    pstcRtc->ALMIR = DecToBcd(pstcAlarm->u8Minute);
    pstcRtc->ALHR  = DecToBcd(pstcAlarm->u8Hour);
    pstcRtc->ALDR  = DecToBcd(pstcAlarm->u8Day);
    pstcRtc->ALMOR = DecToBcd(pstcAlarm->u8Month);
    pstcRtc->ALYR  = DecToBcd(pstcAlarm->u16Year - 2000u);
    
#if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB)        
    // Save RTC count registers to VBAT domain
    if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_CWRITE))
    {
        return ErrorTimeout;
    }
#endif      
    
    return Ok;
}
                
/**
 ******************************************************************************
 ** \brief Get time and date information of Alarm
 **
 ** \param [in] pstcRtc           Pointer to RTC instance
 ** \param [in] pstcAlarm         Time and date structure 
 ** \retval Ok                    RTC Alarm time and date is read normally
 ** \retval ErrorInvalidParameter If one of following conditions are met:
 **                               - pstcAlarm == NULL 
 ** 
 ******************************************************************************/
en_result_t Rtc_GetAlarmDateTime(volatile stc_rtcn_t* pstcRtc, 
                                 stc_rtc_alarm_t* pstcAlarm)
{
    uint8_t  u8BcdMin, u8BcdHour, u8Day, u8Month, u8Year;
    if(NULL == pstcAlarm)
    {
        return ErrorInvalidParameter;
    }
    
#if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB)     
    // Recall data from VBAT domain to RTC count registers
    if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_CREAD))
    {
        return ErrorTimeout;
    }
#endif      
    
    u8BcdMin  = pstcRtc->ALMIR;
    u8BcdHour = pstcRtc->ALHR;
    u8Day     = pstcRtc->ALDR;
    u8Month   = pstcRtc->ALMOR;
    u8Year    = pstcRtc->ALYR;
    
    pstcAlarm->u8Minute = BcdToDec(u8BcdMin);
    pstcAlarm->u8Hour   = BcdToDec(u8BcdHour);
    pstcAlarm->u8Day    = BcdToDec(u8Day);
    pstcAlarm->u8Month  = BcdToDec(u8Month);
    pstcAlarm->u16Year   = BcdToDec(u8Year) + 2000u;
    
    return Ok;
}
                
/**
 ******************************************************************************
 ** \brief Set Day of the Week
 **
 ** This function calculates the day of the week from YY-MM-DD in the Time
 ** structure. It uses mktime of time.h library.
 **
 ** \param  [in,out] pstcRtcTime     RTC Time structure
 **
 ** \retval Ok                       Internal data has been setup
 ** \retval ErrorInvalidParameter    pstcRtcTime == NULL or mktime failed
 **
 ******************************************************************************/
en_result_t Rtc_SetDayOfWeek(stc_rtc_time_t* pstcRtcTime)
{
    en_result_t enResult;
    struct tm stcTime;

    enResult = ErrorInvalidParameter;

    /* Check for NULL pointer */
    if (NULL != pstcRtcTime)
    {
        enResult = Ok;
        /* Convert RTC time structure to time.h tm structure */
        RtcToTm(pstcRtcTime, &stcTime);

        /* Calculated raw time (UNIX time) is error */
        if ((time_t)RTC_ERR == mktime(&stcTime))
        {
            enResult = ErrorInvalidParameter;
        }

        if (Ok == enResult)
        {
            /* Set calculated the day of week */
            pstcRtcTime->u8DayOfWeek = (uint8_t)(stcTime.tm_wday);
        }
    }

    return (enResult);
} /* Rtc_SetDayOfWeek */
                
/**
 ******************************************************************************
 ** \brief Set cycle of RTC timer
 **
 ** \param [in] pstcRtc           Pointer to RTC instance
 ** \param [in] u32TimerCycle     Timer cycle
 ** \retval Ok                    RTC timer cycle is set normally
 ** \retval ErrorInvalidParameter If one of following conditions are met:
 **                               - u32TimerCycle > RTC_MAX_TIMER_SET_VALUE
 ** 
 ** \note The unit of parameter u32TimerCycle is second.
 ** 
 ******************************************************************************/
en_result_t Rtc_SetTimerCycle(volatile stc_rtcn_t* pstcRtc, uint32_t u32TimerCycle)
{
    uint32_t u32Count;
  
    if(u32TimerCycle > RTC_MAX_TIMER_SET_VALUE)
    {
        return ErrorInvalidParameter;
    }

#if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB) 
    // Recall data from VBAT domain to RTC count registers
    if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_CREAD))
    {
        return ErrorTimeout;
    }
#endif       
    
    u32Count = (u32TimerCycle*2) - 1u;
    
    pstcRtc->WTTR0 = (uint8_t)u32Count;
    pstcRtc->WTTR1 = (uint8_t)(u32Count >> 8u);
    pstcRtc->WTTR2 = (uint8_t)(u32Count >> 16u);

#if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB)        
    // Save RTC count registers to VBAT domain
    if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_CWRITE))
    {
        return ErrorTimeout;
    }
#endif    
    
    return Ok;
}
       
/**
 ******************************************************************************
 ** \brief Set calibration value of RTC frequency correction module
 **
 ** \param [in] pstcRtc           Pointer to RTC instance
 ** \param [in] u16Value          Calibration value
 ** \retval Ok                    RTC timer cycle is set normally
 ** \retval ErrorInvalidParameter If one of following conditions are met:
 **                               - u32TimerCycle > RTC_MAX_FREQ_CORR_VALUE
 ** 
 ******************************************************************************/
en_result_t Rtc_SetFreqCorrValue(volatile stc_rtcn_t* pstcRtc, uint16_t u16Value)
{
    if(u16Value > RTC_MAX_FREQ_CORR_VALUE)
    {
        return ErrorInvalidParameter;
    }

#if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB)     
    // Recall data from VBAT domain to RTC count registers
    if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_PREAD))
    {
        return ErrorTimeout;
    }
#endif      
    
    pstcRtc->WTCAL0 = (uint8_t)u16Value;
#if (PDL_RTC_TYPE != PDL_RTC_WITHOUT_VBAT_TYPEA)     
    pstcRtc->WTCAL1 = (uint8_t)(u16Value >> 8u);
#endif    
#if (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEA) || (PDL_RTC_TYPE == PDL_RTC_VBAT_TYPEB)        
    // Save RTC count registers to VBAT domain
    if(Ok != RtcTransWithVbat(pstcRtc, RTC_WTCR20_PWRITE))
    {
        return ErrorTimeout;
    }
#endif 
    
    return Ok;
}
                                    
/**
 ******************************************************************************
 ** \brief Get raw time
 **
 ** This function calculates the "raw" time ('UNIX time').
 **
 ** \param [in] pstcRtcTime     Pointer to RTC Time structure
 **
 ** \return     Calculated time or '-1' on error
 **
 ******************************************************************************/
time_t Rtc_GetRawTime(stc_rtc_time_t* pstcRtcTime)
{
    time_t uiTime;
    struct tm stcTime;

    /* Check for NULL pointer */
    if (NULL == pstcRtcTime)
    {
        uiTime = (time_t)RTC_ERR;
    }
    else
    {
        /* Convert RTC time structure to time.h tm structure */
        RtcToTm(pstcRtcTime, &stcTime);
        uiTime = mktime(&stcTime);
    }

    /* Return raw time (UNIX time) */
    return (uiTime);
} /* Rtc_GetRawTime */

/**
 ******************************************************************************
 ** \brief Sets the RTC time structure from raw time
 **
 ** This function calculates from the RTC time structure "raw" time
 ** ('UNIX time').
 **
 ** \param [out] pstcRtcTime         RTC Time structure
 ** \param [in]  tRawTime            "Raw" time
 **
 ** \retval Ok                       Internal data has been setup
 ** \retval ErrorInvalidParameter    pstcRtcTime == NULL or localtime failed
 **
 ******************************************************************************/
en_result_t Rtc_SetLocalTime(stc_rtc_time_t* pstcRtcTime,
                             time_t          tRawTime)
{
    en_result_t enResult;
    struct tm* pstcTime;

    enResult = ErrorInvalidParameter;

    /* Check for NULL pointer */
    if (NULL != pstcRtcTime)
    {
        /* Get the pointer which converted to RTC time structure from raw time */
        pstcTime = localtime((const time_t*) &tRawTime);

        /* Un-success */
        if (NULL == pstcTime)
        {
            enResult = ErrorInvalidParameter;
        }
        else
        {
            pstcRtcTime->u16Year      = (uint16_t)(pstcTime->tm_year + 1900u);
            pstcRtcTime->u8Month     = (uint8_t)(pstcTime->tm_mon + 1u);
            pstcRtcTime->u8Day       = (uint8_t)(pstcTime->tm_mday);
            pstcRtcTime->u8Second    = (uint8_t)(pstcTime->tm_sec);
            pstcRtcTime->u8Minute    = (uint8_t)(pstcTime->tm_min);
            pstcRtcTime->u8Hour      = (uint8_t)(pstcTime->tm_hour);
            pstcRtcTime->u8DayOfWeek = (uint8_t)(pstcTime->tm_wday);
            enResult = Ok;
        }
    }

    return (enResult);
} /* Rtc_SetTime */


//@} // RtcGroup

#endif /* #if (defined(PDL_PERIPHERAL_RTC_ACTIVE)) */
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
