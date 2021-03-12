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
/** \file adc.h
 **
 ** Headerfile for ADC functions
 **
 ** History:
 **   - 2014-11-08  1.0  EZh First version for FM Universal PDL.
 **
 ******************************************************************************/

#ifndef __ADC_H__
#define __ADC_H__

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "mcu.h"
#include "pdl_user.h"

#if (defined(PDL_PERIPHERAL_ADC_ACTIVE))

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/**
 ******************************************************************************
 ** \defgroup AdcGroup 12-Bit Analog Digital Converter (ADC)
 **
 ** Provided functions of ADC module:
 ** - Adc_Init()
 ** - Adc_DeInit()
 ** - Adc_Enable()
 ** - Adc_Ready()
 ** - Adc_EnableWaitReady()
 ** - Adc_Disable()
 ** - Adc_SwTriggerScan()
 ** - Adc_StopScanRepeat()
 ** - Adc_SwTriggerPrio()
 ** - Adc_ForceStop()
 ** - Adc_EnableIrq()
 ** - Adc_DisableIrq()
 ** - Adc_GetIrqFlag()
 ** - Adc_ClrIrqFlag()
 ** - Adc_GetStatus()
 ** - Adc_ReadScanFifo()
 ** - Adc_ClrScanFifo()
 ** - Adc_GetScanChannel()
 ** - Adc_GetScanDataValid()
 ** - Adc_GetScanDataCause() 
 ** - Adc_ReadPrioFifo()
 ** - Adc_ClrPrioFifo()
 ** - Adc_GetPrioChannel()
 ** - Adc_GetPrioDataValid()
 ** - Adc_GetPrioDataCause()  
 ** 
 ** Adc_Init() initializes an ADC instance according the configuration
 ** of the structure type #stc_adc_config_t. Adc_DeInit() de-initializes an
 ** ADC instance.
 **
 ** For using the ADC instnace Adc_Enable() can be used first to enable ADC
 ** operation.
 **
 ** The interrupt can be configured in Adc_Init() first, after initialization,
 ** Adc_EnableIrq() is used to enable ADC interrupt selected by the structure
 ** type #stc_adc_irq_sel_t. Adc_DisableIrq() is used to disable ADC interrupt 
 ** also selected by the structure type #stc_adc_irq_sel_t.
 **
 ** Adc_SwTriggerScan() and Adc_SwTriggerPrio() can be used for software trigger
 ** start for scan and priority conversion.
 **  
 ** Adc_StopScanRepeat() stops ADC scan conversion with repeat mode.  
 **
 ** Adc_ForceStop() stops both ADC scan and priority conversion forcely.
 **
 ** Adc_GetIrqFlag() can get the interrupt flag in the interrupt-polling mode
 ** and Adc_ClrIrqFlag() can clear the interrupt flag. The interrupt type can
 ** be selected by the enumeration type #en_adc_irq_t.
 **
 ** Adc_GetStatus() return the status of ADC selected by the structure
 ** type #en_adc_status_t.
 **
 ** Adc_ClrScanFifo() and Adc_ClrPrioFifo() clear the FIFOs.
 **
 ** Adc_ReadScanFifo() and Adc_ReadPrioFifo() read out the FIFOs.
 **
 ** Adc_GetScanChannel() and Adc_GetPrioChannel() extracts channel information 
 ** from the FIFO data.
 **
 ** Adc_GetScanDataValid() and Adc_GetPrioDataValid() extracts data valid 
 ** information from the FIFO data.
 **
 ** Adc_GetScanDataCause() and Adc_GetPrioDataCause() extracts trigger cause
 ** information from the FIFO data.   
 **
 ** \note The callback arguments for the scan and priority result point to
 ** the according FIFO registers of the recent instance. The application must
 ** take care of emptying these FIFOs. Either the FIFO depth of the
 ** configuration can be used for counting or the function
 ** Adc_GetStatus() can be used to check if the FIFO is empty or not.
 **
 ******************************************************************************/
//@{

/******************************************************************************
 * Global definitions
 ******************************************************************************/
#define stc_adcn_t   FM_ADC_TypeDef
#define ADC0       (*((volatile stc_adcn_t *) FM_ADC0_BASE))
#define ADC1       (*((volatile stc_adcn_t *) FM_ADC1_BASE))
#define ADC2       (*((volatile stc_adcn_t *) FM_ADC2_BASE))

#define PDL_ADC_READY_WAIT_COUNT 1000000u        ///< Time-out counter value for ADC ready bit polling

#define ADC_INSTANCE_COUNT                                \
        (uint8_t)(PDL_PERIPHERAL_ENABLE_ADC0 == PDL_ON) + \
        (uint8_t)(PDL_PERIPHERAL_ENABLE_ADC1 == PDL_ON) + \
        (uint8_t)(PDL_PERIPHERAL_ENABLE_ADC2 == PDL_ON)

#define ADC0_SCAN_FIFO_ADDR     (uint32_t)(&FM_ADC0->SCFD)
#define ADC1_SCAN_FIFO_ADDR     (uint32_t)(&FM_ADC1->SCFD)
#define ADC2_SCAN_FIFO_ADDR     (uint32_t)(&FM_ADC2->SCFD)

// ADC types        
#define PDL_ADC_A              0u
#define PDL_ADC_B              1u

// Check the ADC type according to a certain device            
#if (PDL_MCU_TYPE == PDL_FM3_TYPE0) || (PDL_MCU_TYPE == PDL_FM3_TYPE1) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE2) || (PDL_MCU_TYPE == PDL_FM3_TYPE4) || \
    (PDL_MCU_TYPE == PDL_FM3_TYPE5)
#define PDL_ADC_TYPE     PDL_ADC_A
#else
#define PDL_ADC_TYPE     PDL_ADC_B      
#endif
          
/******************************************************************************
 * Global type definitions
 ******************************************************************************/
typedef void (*func_ptr_adc_parg32_t)(volatile uint32_t* pu32Argument);

/**
 ******************************************************************************
 ** \brief Scan Conversion Mode
 ******************************************************************************/
typedef enum en_adc_scan_mode
{
    ScanSingleConversion  = 0u,        ///< Single mode of scan conversion
    ScanRepeatConversion  = 1u         ///< Repeat mode of scan conversion
} en_adc_scan_mode_t;

/**
 ******************************************************************************
 ** \brief Priority Conversion Stage Count
 ******************************************************************************/
typedef enum en_adc_sample_time_n
{
    Value1        = 0u,    ///< Set value * 1
    Value4        = 1u,    ///< Set value * 4
    Value8        = 2u,    ///< Set value * 8
    Value16       = 3u,    ///< Set value * 16
    Value32       = 4u,    ///< Set value * 32
    Value64       = 5u,    ///< Set value * 64
    Value128      = 6u,    ///< Set value * 128
    Value256      = 7u,    ///< Set value * 256
} en_adc_sample_time_n_t;

#if (PDL_ADC_TYPE == PDL_ADC_A)
#if((PDL_MCU_TYPE != PDL_FM3_TYPE0)) 
/**
 ******************************************************************************
 ** \brief The basic cycle selection of enable state transition
 **
 ** \note This setting is only available for TYPE 0,1,2,5 of FM3 products.
 **       The final enable state transition time is also related with comparing
 **       clock division setting, see peripheral manual for details.
 ******************************************************************************/
typedef enum en_adc_enable_cycle
{
    AdcEnableCycle36 = 0u, ///< Set basic cycle to 36
    AdcEnableCycle20 = 1u, ///< Set basic cycle to 20
    AdcEnableCycle9  = 2u, ///< Set basic cycle to 9
    AdcEnableCycle44 = 3u, ///< Set basic cycle to 44
    
}en_adc_enable_cycle_t;
#endif    
#endif

/**
 ******************************************************************************
 ** \brief ADC scan/priority conversion timer trigger selection
 ** \note Do not change the numbering.
 ******************************************************************************/
typedef enum en_adc_timer_select
{
    AdcNoTimer =  0u,   ///< No selected trigger
    AdcMft     =  1u,   ///< Trigger by Multifunction Timer
    AdcBt0     =  2u,   ///< Trigger with Base Timer 0
    AdcBt1     =  3u,   ///< Trigger with Base Timer 1
    AdcBt2     =  4u,   ///< Trigger with Base Timer 2
    AdcBt3     =  5u,   ///< Trigger with Base Timer 3
    AdcBt4     =  6u,   ///< Trigger with Base Timer 4
    AdcBt5     =  7u,   ///< Trigger with Base Timer 5
    AdcBt6     =  8u,   ///< Trigger with Base Timer 6
    AdcBt7     =  9u,   ///< Trigger with Base Timer 7

} en_adc_timer_select_t;

/**
 ******************************************************************************
 ** \brief ADC Channel list with same sort order as the ER32 registers
 ******************************************************************************/
typedef struct stc_ad_channel_list
{
    union 
    {
        uint32_t u32AD_CHn;
        struct stcChannelBits
        {
            uint32_t AD_CH_0  : 1 ;
            uint32_t AD_CH_1  : 1 ;
            uint32_t AD_CH_2  : 1 ;
            uint32_t AD_CH_3  : 1 ;
            uint32_t AD_CH_4  : 1 ;
            uint32_t AD_CH_5  : 1 ;
            uint32_t AD_CH_6  : 1 ;
            uint32_t AD_CH_7  : 1 ;
            uint32_t AD_CH_8  : 1 ;
            uint32_t AD_CH_9  : 1 ;
            uint32_t AD_CH_10 : 1 ;
            uint32_t AD_CH_11 : 1 ;
            uint32_t AD_CH_12 : 1 ;
            uint32_t AD_CH_13 : 1 ;
            uint32_t AD_CH_14 : 1 ;
            uint32_t AD_CH_15 : 1 ;
            uint32_t AD_CH_16 : 1 ;
            uint32_t AD_CH_17 : 1 ;
            uint32_t AD_CH_18 : 1 ;
            uint32_t AD_CH_19 : 1 ;
            uint32_t AD_CH_20 : 1 ;
            uint32_t AD_CH_21 : 1 ;
            uint32_t AD_CH_22 : 1 ;
            uint32_t AD_CH_23 : 1 ;
            uint32_t AD_CH_24 : 1 ;
            uint32_t AD_CH_25 : 1 ;
            uint32_t AD_CH_26 : 1 ;
            uint32_t AD_CH_27 : 1 ;
            uint32_t AD_CH_28 : 1 ;
            uint32_t AD_CH_29 : 1 ;
            uint32_t AD_CH_30 : 1 ;
            uint32_t AD_CH_31 : 1 ;
        };
    };
} stc_ad_channel_list_t ;

/**
 ******************************************************************************
 ** \brief Scan Conversion configuration
 ******************************************************************************/
typedef struct stc_adc_scan
{
    en_adc_scan_mode_t            enScanMode;                     ///< Conversion Mode. See #en_adc_scan_mode_t for details
    stc_ad_channel_list_t         u32ScanCannelSelect;            ///< Selects scan channels for ADC (1 = active)
    boolean_t                     bScanTimerStartEnable;          ///< TRUE: Triggers Scan Conversion by Timer
    en_adc_timer_select_t         enScanTimerTrigger;             ///< See #en_adc_timer_select_t for details
    uint8_t                       u8ScanFifoDepth;                ///< Depth of the Scan Conversion FIFO 0...15

}stc_adc_scan_t;

/**
 ******************************************************************************
 ** \brief Priority Conversion configuration
 ******************************************************************************/
typedef struct stc_adc_prio
{
    boolean_t                     bPrioExtTrigStartEnable;        ///< TRUE: Triggers Prioritys Conversion on falling edge of external signal
    boolean_t                     bPrioTimerStartEnable;          ///< TRUE: Triggers Priority Conversion by Timer
    en_adc_timer_select_t         enPrioTimerTrigger;             ///< See #en_adc_timer_select_t for details
    uint8_t                       u8PrioFifoDepth;                ///< Depth of the Priority Conversion FIFO 0...3
    uint8_t                       u8PrioLevel1AnalogChSel;        ///< Priority Level 1 Analog Channel Selector Ch. 0...7
    uint8_t                       u8PrioLevel2AnalogChSel;        ///< Priority Level 2 Analog Channel Selector Ch. 0...31

}stc_adc_prio_t;

/**
 ******************************************************************************
 ** \brief Comparison configuration
 ******************************************************************************/
typedef struct stc_adc_comapre
{
    uint16_t                      u16CompareValue;                ///< ADC Comparison Value (CMPD)
    boolean_t                     bCompareAllChannels;            ///< TRUE: Compare all selected Channels, FALSE: Compare CCH-Channel
    uint8_t                       u8CompareChannel;               ///< CCH-Channel to be compared, if selected Ch. 0...31
    boolean_t                     bCompIrqEqualGreater;           ///< TRUE: Generate Interrupt, if CMPD most significant 10 bis >= current ADC value
                                                                  ///< FALSE: Generate Interrupt, if CMPD most significant 10 bis < current ADC value
}stc_adc_comapre_t;

#if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE) 
/**
 ******************************************************************************
 ** \brief Range comparison configuration
 ******************************************************************************/
typedef struct stc_adc_range_compare
{
    uint16_t                      u16UpperLimitRangeValue;        ///< Upper Limit Value for Range Comparison
    uint16_t                      u16LowerLimitRangeValue;        ///< Lower limit Value for Range Comparison
    uint8_t                       u8RangeCountValue;              ///< Range Count Value 1...7
    boolean_t                     bWithinRange;                   ///< TRUE: Value within Range, FALSE: Value out of Range
    boolean_t                     bRangeCompareAllChannels;       ///< TRUE: Range compare all selected Channels, FALSE: Compare WCCH-Channel
    uint8_t                       u8RangeComapreChannel;          ///< WCCH-Channel to be Range compared, if selected Ch. 0...31

}stc_adc_range_compare_t;
#endif

/**
 ******************************************************************************
 ** \brief ADC interrupt selection.
 ******************************************************************************/
typedef struct stc_adc_irq_sel
{
    boolean_t                     bScanIrq;                 ///< TRUE: Enable Scan Conversion Interrupt
    boolean_t                     bPrioIrq;                 ///< TRUE: Enable Priority Conversion Interrupt
    boolean_t                     bFifoOverrunIrq;          ///< TRUE: Enable FIFO Overrun Interrupt
    boolean_t                     bComparisonIrq;           ///< TRUE: Enable Conversion Comparison Interrupt
#if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE)       
    boolean_t                     bRangeComparisonIrq;      ///< TRUE: Enable Range Comparison Interrupt
#endif
}stc_adc_irq_sel_t, stc_adc_irq_en_t;

/**
 ******************************************************************************
 ** \brief ADC interrupt callback function & internal data structure.
 ******************************************************************************/
typedef struct stc_adc_irq_cb
{
    func_ptr_adc_parg32_t   pfnScanIrqCb;              ///< Scan conversion interrupt callback pointer
    func_ptr_adc_parg32_t   pfnPrioIrqCb;              ///< Priority conversion interrupt callback
    func_ptr_t              pfnScanErrIrqCb;           ///< Scan FIFO overrun error callback pointer
    func_ptr_t              pfnPrioErrIrqCb;           ///< Priority FIFO overrun error callback pointer
    func_ptr_t              pfnComparisonIrqCb;        ///< Comparison interrupt callback pointer
#if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE)     
    func_ptr_t              pfnRangeComparisonIrqCb;   ///< Range condition interrupt callback pointer
#endif
    
}stc_adc_irq_cb_t, stc_adc_intern_data_t;

/**
 ******************************************************************************
 ** \brief ADC configuration.
 ******************************************************************************/
typedef struct stc_adc_config
{
    boolean_t                     bLsbAlignment;                  ///< TURE: Result is LSB aligned
    stc_ad_channel_list_t         u32SamplingTimeSelect;          ///< Selects channels for Sampling time 0 or 1 setting
    en_adc_sample_time_n_t        enSamplingTimeN0;               ///< Sampling Time N0 Time Setting Bits, see #en_adc_sample_time_n_t for details
    uint8_t                       u8SamplingTime0;                ///< Sampling Time 0 0...15
    en_adc_sample_time_n_t        enSamplingTimeN1;               ///< Sampling Time N1 Time Setting Bits, see #en_adc_sample_time_n_t for details
    uint8_t                       u8SamplingTime1;                ///< Sampling Time 1 0...15
    uint8_t                       u8ComparingClockDiv;            ///< Multiplier of N, see Peripheral Manual for Details!
#if (PDL_ADC_TYPE == PDL_ADC_A)
    #if((PDL_MCU_TYPE != PDL_FM3_TYPE0))  
    en_adc_enable_cycle_t         enEnableCycle;                  ///< Set the basic cycle of operation enable state transitions, see Peripheral Manual for Details! 
    #endif    
#elif (PDL_ADC_TYPE == PDL_ADC_B)    
    uint8_t                       u8EnableTime;                   ///< Enable Time
#endif
    stc_adc_scan_t*               pstcScanInit;                   ///< Pointer to scan coversion configuration structure, scan coversion won't be intialized when it is set to NULL.
    stc_adc_prio_t*               pstcPrioInit;                   ///< Pointer to priority coversion configuration structure, priority coversion won't be intialized when it is set to NULL.
    stc_adc_comapre_t*            pstcComparisonInit;             ///< Pointer to comparison structure, comparison function won't be intialized when it is set to NULL.
#if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE)   
    stc_adc_range_compare_t*      pstcRangeComparisonInit;        ///< Pointer to range comparison structure, range comparison function won't be intialized when it is set to NULL.
#endif
    
    stc_adc_irq_en_t*             pstcIrqEn;                      ///< Pointer to ADC interrupts enable structure
    stc_adc_irq_cb_t*             pstcIrqCb;                      ///< Pointer to the structure of ADC interrupt request callback functions  
    boolean_t                     bTouchNvic;                     ///< TRUE: enable NVIC, FALSE: don't enable NVIC
    
} stc_adc_config_t;

/**
 ******************************************************************************
 ** \brief FIFO valid data result (Scan, Priority Conversion)
 ******************************************************************************/
typedef enum en_adc_fifo_data_valid
{
    AdcFifoDataInvalid  = 0u,    ///< FIFO data invalid
    AdcFifoDataValid    = 1u     ///< FIFO data valid
} en_adc_fifo_data_valid_t;

/**
 ******************************************************************************
 ** \brief FIFO data start cause
 ******************************************************************************/
typedef enum en_adc_fifo_start_cause
{
    AdcFifoSoftwareStart   = 0u,    ///< FIFO data software start cause (Scan & Prio2 Conversion)
    AdcFifoTimerStart      = 1u,    ///< FIFO data timer start cause (Scan & Prio2 Conversion)
    AdcFifoExternalTrigger = 2u,    ///< FIFO data external trigger cause (Prio1 Conversion)
    AdcFifoErrorStart      = 3u     ///< FIFO data caused by unknown factor (Scan & Prio2 Conversion)
} en_adc_fifo_start_cause_t;

/**
 ******************************************************************************
 ** \brief ADC status list
 ******************************************************************************/
typedef enum en_adc_status
{
    ScanFifoEmptyStatus         = 0u,  ///< Scan FIFO empty status
    ScanFifoFullStatus          = 1u,  ///< Scan FIFO full status
    PrioFifoEmptyStatus         = 2u,  ///< Priority FIFO empty status
    PrioFifoFullStatus          = 3u,  ///< Priority FIFO full status
    Prio2PendingStatus          = 4u,  ///< Priority conversion (level 2) pending status
    PrioStatus                  = 5u,  ///< Priority conversion status
    ScanStatus                  = 6u,  ///< Scan conversion status
#if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE)     
    RangeThresholdExcessFlag    = 7u,  ///< Range comparison threshold exceed status
#endif

}en_adc_status_t;

/**
 ******************************************************************************
 ** \brief ADC interrupt type
 ******************************************************************************/
typedef enum en_adc_irq
{
    AdcScanIrq                = 0u,   ///< Scan conversion interrupt request
    AdcPrioIrq                = 1u,   ///< Priority conversion interrupt request
    AdcScanFifoOverrunIrq     = 2u,   ///< Scan FIFO overrun interrupt request
    AdcPrioFifoOverrunIrq     = 3u,   ///< Priority FIFO overrun interrupt request
    AdcComparisonIrq          = 4u,   ///< ADC comparison interrupt request
#if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE)     
    AdcRangeComparisonIrq     = 5u,   ///< ADC range comparison interrupt request
#endif
}en_adc_irq_t;

/// Enumeration to define an index for each enabled ADC instance
typedef enum en_adc_instance_index
{
  #if (PDL_PERIPHERAL_ENABLE_ADC0 == PDL_ON)
    AdcInstanceIndexAdc0,
  #endif
  #if (PDL_PERIPHERAL_ENABLE_ADC1 == PDL_ON)
    AdcInstanceIndexAdc1,
  #endif
  #if (PDL_PERIPHERAL_ENABLE_ADC2 == PDL_ON)
    AdcInstanceIndexAdc2,
  #endif
    AdcInstanceIndexMax
} en_adc_instance_index_t;

/// ADC instance data type
typedef struct stc_adc_instance_data
{
  volatile stc_adcn_t*  pstcInstance;  ///< pointer to registers of an instance
  stc_adc_intern_data_t stcInternData; ///< module internal data of instance
} stc_adc_instance_data_t;

/******************************************************************************/
/* Global variable declarations ('extern', definition in C source)            */
/******************************************************************************/

/// Look-up table for all enabled ADC instances and their internal data
extern stc_adc_instance_data_t m_astcAdcInstanceDataLut[ADC_INSTANCE_COUNT];

/******************************************************************************/
/* Global function prototypes ('extern', definition in C source)              */
/******************************************************************************/
/* Init/De-Init */
en_result_t               Adc_Init( volatile stc_adcn_t* pstcAdc,
                                    stc_adc_config_t*    pstcConfig );
en_result_t               Adc_DeInit( volatile stc_adcn_t* pstcAdc, 
                                      boolean_t bTouchNvic);

#if (PDL_INTERRUPT_ENABLE_ADC0 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_ADC1 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_ADC2 == PDL_ON)
/* Int Enable/Disable */
en_result_t               Adc_EnableIrq(volatile stc_adcn_t* pstcAdc,
                                        stc_adc_irq_sel_t* pstcIrqSel);
en_result_t               Adc_DisableIrq(volatile stc_adcn_t* pstcAdc,
                                         stc_adc_irq_sel_t* pstcIrqSel);
#endif
/* Int Flag get/clear */
boolean_t                 Adc_GetIrqFlag(volatile stc_adcn_t* pstcAdc,
                                         en_adc_irq_t enIrq);
en_result_t               Adc_ClrIrqFlag(volatile stc_adcn_t* pstcAdc,
                                         en_adc_irq_t enIrq);

/* ADC enable/disable */
en_result_t               Adc_Enable( volatile stc_adcn_t* pstcAdc );
en_result_t               Adc_Ready( volatile stc_adcn_t* pstcAdc );
en_result_t               Adc_EnableWaitReady( volatile stc_adcn_t* pstcAdc );
en_result_t               Adc_Disable( volatile stc_adcn_t* pstcAdc );

/* ADC conversion start/stop */
en_result_t               Adc_SwTriggerScan( volatile stc_adcn_t* pstcAdc );
en_result_t               Adc_StopScanRepeat( volatile stc_adcn_t* pstcAdc );
en_result_t               Adc_StartScanRepeat( volatile stc_adcn_t* pstcAdc );
en_result_t               Adc_SwTriggerPrio( volatile stc_adcn_t* pstcAdc );
en_result_t               Adc_ForceStop( volatile stc_adcn_t* pstcAdc );
/* Status */
boolean_t                 Adc_GetStatus(volatile stc_adcn_t* pstcAdc,
                                        en_adc_status_t enStatus);
/* FIFO read/clear */
uint32_t                  Adc_ReadScanFifo( volatile stc_adcn_t* pstcAdc );
en_result_t               Adc_ClrScanFifo(volatile stc_adcn_t* pstcAdc);
uint8_t                   Adc_GetScanChannel( volatile stc_adcn_t* pstcAdc, 
                                              uint32_t u32FifoData );
en_adc_fifo_data_valid_t  Adc_GetScanDataValid( volatile stc_adcn_t* pstcAdc,  
                                                uint32_t u32FifoData );
en_adc_fifo_start_cause_t Adc_GetScanDataCause( volatile stc_adcn_t* pstcAdc, 
                                                uint32_t u32FifoData );
uint16_t                  Adc_GetScanData( volatile stc_adcn_t* pstcAdc, 
                                           uint32_t u32FifoData );

uint32_t                  Adc_ReadPrioFifo( volatile stc_adcn_t* pstcAdc );
en_result_t               Adc_ClrPrioFifo(volatile stc_adcn_t* pstcAdc);
uint8_t                   Adc_GetPrioChannel(volatile stc_adcn_t* pstcAdc, 
                                             uint32_t u32FifoData );
en_adc_fifo_data_valid_t  Adc_GetPrioDataValid(volatile stc_adcn_t* pstcAdc,
                                               uint32_t u32FifoData );
en_adc_fifo_start_cause_t Adc_GetPrioDataCause(volatile stc_adcn_t* pstcAdc,
                                               uint32_t u32FifoData );
uint16_t                  Adc_GetPrioData(volatile stc_adcn_t* pstcAdc,
                                          uint32_t u32FifoData );

/* ADC IRQ */
void                      AdcIrqHandler( volatile stc_adcn_t*   pstcAdc,
                                         stc_adc_intern_data_t* pstcAdcInternData );
//@}
#ifdef __cplusplus
}
#endif

#endif // #if (defined(PDL_PERIPHERAL_ADC_ACTIVE))

#endif /* __ADC_H__ */
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
