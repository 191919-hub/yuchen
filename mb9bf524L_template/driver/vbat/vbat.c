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
/** \file vbat.c
 **
 ** A detailed description is available at 
 ** @link VbatGroup VBAT description @endlink
 **
 ** History:
 **   - 2014-12-08  1.0  EZh  First version for FM Universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "vbat.h"

#if (defined(PDL_PERIPHERAL_VBAT_ACTIVE))

#if (PDL_PERIPHERAL_VBAT_AVAILABLE == PDL_ON)
/**
 ******************************************************************************
 ** \addtogroup VbatGroup
 ******************************************************************************/
//@{

/******************************************************************************/
/* Local pre-processor symbols/macros ('#define')                             */
/******************************************************************************/
/******************************************************************************/
/* Global variable definitions (declared in header file with 'extern')        */
/******************************************************************************/
/******************************************************************************/
/* Local function prototypes ('static')                                       */
/******************************************************************************/
/******************************************************************************/
/* Local variable definitions ('static')                                      */
/******************************************************************************/
/******************************************************************************/
/* Function implementation - global ('extern') and local ('static')           */
/******************************************************************************/
/******************************************************************************/
/* Local Functions                                                            */
/******************************************************************************/
/******************************************************************************/
/* Global Functions                                                           */
/******************************************************************************/

/**
 ******************************************************************************
 ** \brief Initialize VBAT
 **
 ** This function initializes an VBAT module
 **
 ** \param [in]  pstcConfig       VBAT module configuration 
 **
 ** \retval Ok                    VBAT initialized normally
 ** \retval ErrorInvalidParameter If one of following cases matchs: 
 **                               - pstcConfig == NULL
 **                               - Other invalid configuration
 ******************************************************************************/
en_result_t Vbat_Init(stc_vbat_config_t* pstcConfig)
{
    // Check parameter
    if (NULL == pstcConfig)
    {
        return ErrorInvalidParameter;
    }
    
    //  Clear VBAT power on flag
    FM_RTC->VDET = 0u;
    
    // Configure transfer clock
    if ((0u == pstcConfig->u8ClkDiv) ||  (0xFFu == pstcConfig->u8ClkDiv))
    {
        return ErrorInvalidParameter;
    }  
    FM_RTC->VB_CLKDIV = pstcConfig->u8ClkDiv;
    
    // Configure sustain/boost current 
#if (PDL_MCU_TYPE == PDL_FM4_TYPE1) || \
    (PDL_MCU_TYPE == PDL_FM4_TYPE2) || \
    (PDL_MCU_TYPE == PDL_FM4_TYPE3)      
    switch (pstcConfig->enSustainCurrent)
    {
        case Clk0nA:
            FM_RTC->CCS = 0x00u;
            break;
        case Clk385nA:
            FM_RTC->CCS = 0x04u;
            break;
        case Clk445nA:
            FM_RTC->CCS = 0x08u;
            break;
        case Clk510nA:
            FM_RTC->CCS = 0x10u;
            break;
        default:
            return ErrorInvalidParameter;
    }
    
    switch (pstcConfig->enBoostCurrent)
    {
        case Clk0nA:
            FM_RTC->CCB = 0x00u;
            break;
        case Clk385nA:
            FM_RTC->CCB = 0x04u;
            break;
        case Clk445nA:
            FM_RTC->CCB = 0x08u;
            break;
        case Clk510nA:
            FM_RTC->CCB = 0x10u;
            break;
        default:
            return ErrorInvalidParameter;
    }
#else
    switch (pstcConfig->enSustainCurrent)
    {
        case ClkOff:
            FM_RTC->CCS_f.CDRVB = 0u;
            FM_RTC->CCS_f.CDRVA = 0u;
            break;
        case Clk100nA:
            FM_RTC->CCS_f.CDRVB = 1u;
            FM_RTC->CCS_f.CDRVA = 0u;
            break;
        case Clk140nA:
            FM_RTC->CCS_f.CDRVB = 2u;
            FM_RTC->CCS_f.CDRVA = 0u;
            break;
        case Clk160nA:
            FM_RTC->CCS_f.CDRVB = 3u;
            FM_RTC->CCS_f.CDRVA = 0u;
            break;
        case Clk200nA:
            FM_RTC->CCS_f.CDRVB = 4u;
            FM_RTC->CCS_f.CDRVA = 0u;
            break;
        case Clk240nA:
            FM_RTC->CCS_f.CDRVB = 5u;
            FM_RTC->CCS_f.CDRVA = 0u;
            break;
        case Clk280nA:
            FM_RTC->CCS_f.CDRVB = 6u;
            FM_RTC->CCS_f.CDRVA = 0u;
            break;
        case Clk300nA:
            FM_RTC->CCS_f.CDRVB = 1u;
            FM_RTC->CCS_f.CDRVA = 1u;
            break;
        case Clk320nA:
            FM_RTC->CCS_f.CDRVB = 7u;
            FM_RTC->CCS_f.CDRVA = 0u;
            break;
        case Clk400nA:
            FM_RTC->CCS_f.CDRVB = 4u;
            FM_RTC->CCS_f.CDRVA = 1u;
            break;
        case Clk420nA:
            FM_RTC->CCS_f.CDRVB = 2u;
            FM_RTC->CCS_f.CDRVA = 2u;
            break;
        case Clk480nA:
            FM_RTC->CCS_f.CDRVB = 3u;
            FM_RTC->CCS_f.CDRVA = 2u;
            break;
        case Clk500nA:
            FM_RTC->CCS_f.CDRVB = 1u;
            FM_RTC->CCS_f.CDRVA = 3u;
            break;
        case Clk560nA:
            FM_RTC->CCS_f.CDRVB = 6u;
            FM_RTC->CCS_f.CDRVA = 1u;
            break;
        case Clk600nA:
            FM_RTC->CCS_f.CDRVB = 4u;
            FM_RTC->CCS_f.CDRVA = 2u;
            break;
        case Clk640nA:
            FM_RTC->CCS_f.CDRVB = 7u;
            FM_RTC->CCS_f.CDRVA = 1u;
            break;
        case Clk700nA:
            FM_RTC->CCS_f.CDRVB = 2u;
            FM_RTC->CCS_f.CDRVA = 3u;
            break;
        case Clk720nA:
            FM_RTC->CCS_f.CDRVB = 5u;
            FM_RTC->CCS_f.CDRVA = 2u;
            break;
        case Clk800nA:
            FM_RTC->CCS_f.CDRVB = 3u;
            FM_RTC->CCS_f.CDRVA = 3u;
            break;
        case Clk840nA:
            FM_RTC->CCS_f.CDRVB = 6u;
            FM_RTC->CCS_f.CDRVA = 2u;
            break;
        case Clk960nA:
            FM_RTC->CCS_f.CDRVB = 7u;
            FM_RTC->CCS_f.CDRVA = 2u;
            break;
        case Clk1000nA:
            FM_RTC->CCS_f.CDRVB = 4u;
            FM_RTC->CCS_f.CDRVA = 3u;
            break;
        case Clk1200nA:
            FM_RTC->CCS_f.CDRVB = 5u;
            FM_RTC->CCS_f.CDRVA = 3u;
            break;
        case Clk1400nA:
            FM_RTC->CCS_f.CDRVB = 6u;
            FM_RTC->CCS_f.CDRVA = 2u;
            break; 
        case Clk1600nA:
            FM_RTC->CCS_f.CDRVB = 7u;
            FM_RTC->CCS_f.CDRVA = 3u;
            break;               
        default:
            return ErrorInvalidParameter;
    }
    
    switch (pstcConfig->enBoostCurrent)
    {
        case ClkOff:
            FM_RTC->CCB_f.CDRVB = 0u;
            FM_RTC->CCB_f.CDRVA = 0u;
            break;
        case Clk100nA:
            FM_RTC->CCB_f.CDRVB = 1u;
            FM_RTC->CCB_f.CDRVA = 0u;
            break;
        case Clk140nA:
            FM_RTC->CCB_f.CDRVB = 2u;
            FM_RTC->CCB_f.CDRVA = 0u;
            break;
        case Clk160nA:
            FM_RTC->CCB_f.CDRVB = 3u;
            FM_RTC->CCB_f.CDRVA = 0u;
            break;
        case Clk200nA:
            FM_RTC->CCB_f.CDRVB = 4u;
            FM_RTC->CCB_f.CDRVA = 0u;
            break;
        case Clk240nA:
            FM_RTC->CCB_f.CDRVB = 5u;
            FM_RTC->CCB_f.CDRVA = 0u;
            break;
        case Clk280nA:
            FM_RTC->CCB_f.CDRVB = 6u;
            FM_RTC->CCB_f.CDRVA = 0u;
            break;
        case Clk300nA:
            FM_RTC->CCB_f.CDRVB = 1u;
            FM_RTC->CCB_f.CDRVA = 1u;
            break;
        case Clk320nA:
            FM_RTC->CCB_f.CDRVB = 7u;
            FM_RTC->CCB_f.CDRVA = 0u;
            break;
        case Clk400nA:
            FM_RTC->CCB_f.CDRVB = 4u;
            FM_RTC->CCB_f.CDRVA = 1u;
            break;
        case Clk420nA:
            FM_RTC->CCB_f.CDRVB = 2u;
            FM_RTC->CCB_f.CDRVA = 2u;
            break;
        case Clk480nA:
            FM_RTC->CCB_f.CDRVB = 3u;
            FM_RTC->CCB_f.CDRVA = 2u;
            break;
        case Clk500nA:
            FM_RTC->CCB_f.CDRVB = 1u;
            FM_RTC->CCB_f.CDRVA = 3u;
            break;
        case Clk560nA:
            FM_RTC->CCB_f.CDRVB = 6u;
            FM_RTC->CCB_f.CDRVA = 1u;
            break;
        case Clk600nA:
            FM_RTC->CCB_f.CDRVB = 4u;
            FM_RTC->CCB_f.CDRVA = 2u;
            break;
        case Clk640nA:
            FM_RTC->CCB_f.CDRVB = 7u;
            FM_RTC->CCB_f.CDRVA = 1u;
            break;
        case Clk700nA:
            FM_RTC->CCB_f.CDRVB = 2u;
            FM_RTC->CCB_f.CDRVA = 3u;
            break;
        case Clk720nA:
            FM_RTC->CCB_f.CDRVB = 5u;
            FM_RTC->CCB_f.CDRVA = 2u;
            break;
        case Clk800nA:
            FM_RTC->CCB_f.CDRVB = 3u;
            FM_RTC->CCB_f.CDRVA = 3u;
            break;
        case Clk840nA:
            FM_RTC->CCB_f.CDRVB = 6u;
            FM_RTC->CCB_f.CDRVA = 2u;
            break;
        case Clk960nA:
            FM_RTC->CCB_f.CDRVB = 7u;
            FM_RTC->CCB_f.CDRVA = 2u;
            break;
        case Clk1000nA:
            FM_RTC->CCB_f.CDRVB = 4u;
            FM_RTC->CCB_f.CDRVA = 3u;
            break;
        case Clk1200nA:
            FM_RTC->CCB_f.CDRVB = 5u;
            FM_RTC->CCB_f.CDRVA = 3u;
            break;
        case Clk1400nA:
            FM_RTC->CCB_f.CDRVB = 6u;
            FM_RTC->CCB_f.CDRVA = 2u;
            break; 
        case Clk1600nA:
            FM_RTC->CCB_f.CDRVB = 7u;
            FM_RTC->CCB_f.CDRVA = 3u;
            break;               
        default:
            return ErrorInvalidParameter;
    }
#endif    
    
    // Set boost time
    switch (pstcConfig->enClkBoostTime)
    {
        case ClkBoost50ms:
            FM_RTC->BOOST = 0u;
            break;
        case ClkBoost63ms:
            FM_RTC->BOOST = 1u;
            break;
        case ClkBoost125ms:
            FM_RTC->BOOST = 2u;
            break;
        case ClkBoost250ms:
            FM_RTC->BOOST = 3u;
            break;
        default:
            return ErrorInvalidParameter;
    }
    
    // Configure whether to link clock to clock control module
    FM_RTC->WTOSCCNT_f.SOSCNTL = ((pstcConfig->bLinkSubClk == TRUE) ? 1u : 0u);
    
    // Configure whether to enable sub clock in the VBAT domain
    FM_RTC->WTOSCCNT_f.SOSCEX = ((pstcConfig->bVbatClkEn == TRUE) ? 0u : 1u);
    
    // Transmit to VBAT domain 
    FM_RTC->WTCR20_f.PWRITE = 1u;

    // Wait to complete transmission
    while(0u != FM_RTC->WTCR10_f.TRANS)
    {}
    
    return Ok;
}

/**
 ******************************************************************************
 ** \brief Configure pin funciton of P48/VREGCTL to VREGCTL
 ******************************************************************************/
void Vbat_SetPinFunc_VREGCTL(void)
{
    // Recall from VBAT domain 
    FM_RTC->WTCR20_f.PREAD = 1u;

    // Wait to complete transmission
    while(0u != FM_RTC->WTCR10_f.TRANS)
    {}
    
    // Set function register
    FM_RTC->VBPFR_f.VPFR0 = 1u;
    
    // Trasnmit to VBAT domain 
    FM_RTC->WTCR20_f.PWRITE = 1u;

    // Wait to complete transmission
    while(0u != FM_RTC->WTCR10_f.TRANS)
    {}
    
    return;
}

/**
 ******************************************************************************
 ** \brief Configure pin funciton of P49/VWAKEUP to VWAKEUP
 ******************************************************************************/
void Vbat_SetPinFunc_VWAKEUP(void)
{
    // Recall from VBAT domain 
    FM_RTC->WTCR20_f.PREAD = 1;

    // Wait to complete transmission
    while(0u != FM_RTC->WTCR10_f.TRANS)
    {}
    
    // Set function register
    FM_RTC->VBPFR_f.VPFR1 = 1u;
    
    // Trasnmit to VBAT domain 
    FM_RTC->WTCR20_f.PWRITE = 1;

    // Wait to complete transmission
    while(0u != FM_RTC->WTCR10_f.TRANS)
    {}
    
    return;
}

/**
 ******************************************************************************
 ** \brief Configure pin funciton of P46/X0A, P47/X1A to X0A, X1A
 ******************************************************************************/
void Vbat_SetPinFunc_X0A_X1A(void)
{
    // Recall from VBAT domain 
    FM_RTC->WTCR20_f.PREAD = 1;

    // Wait to complete transmission
    while(0u != FM_RTC->WTCR10_f.TRANS)
    {}
    
    // Set function register
    FM_RTC->VBPFR_f.SPSR0 = 1u;
    FM_RTC->VBPFR_f.SPSR1 = 0u;
    FM_RTC->VBPFR_f.VPFR2 = 1u;
    FM_RTC->VBPFR_f.VPFR3 = 1u;
    
    // Trasnmit to VBAT domain 
    FM_RTC->WTCR20_f.PWRITE = 1;

    // Wait to complete transmission
    while(0u != FM_RTC->WTCR10_f.TRANS)
    {}
    
    return;
}

/**
 ******************************************************************************
 ** \brief Initialize VBAT GPIO output
 **
 ** This function initializes an VBAT GPIO output
 **
 ** \param [in]  enGpio       VBAT GPIO list
 ** \param [in]  bInitLevel   GPIO output initial level
 ** \param [in]  bOpenDrain   GPIO open drain or not
 **
 ** \retval Ok                    VBAT initialized normally
 ** \retval ErrorInvalidParameter enGpio out of range
 ** 
 ** \note Open drain funciton is only available for P48 and P49.
 **
 ******************************************************************************/
en_result_t Vbat_InitGpioOutput(en_vbat_gpio_t enGpio, 
                                boolean_t bInitLevel,
                                boolean_t bOpenDrain)
{
    // Recall from VBAT domain 
    FM_RTC->WTCR20_f.PREAD = 1;

    // Wait to complete transmission
    while(0u != FM_RTC->WTCR10_f.TRANS)
    {}
  
    // Set GPIO registers
    switch (enGpio)
    {
        case VbatGpioP46:
            FM_RTC->VBPFR_f.SPSR0 = 0u;
            FM_RTC->VBPFR_f.SPSR1 = 0u;
            FM_RTC->VBPFR_f.VPFR3 = 0u;
            FM_RTC->VBDDR_f.VDDR3 = 1u;
            FM_RTC->VBDOR_f.VDOR3 = ((1u == bInitLevel) ? 1u : 0u);
            break;
        case VbatGpioP47:
            FM_RTC->VBPFR_f.SPSR0 = 0u;
            FM_RTC->VBPFR_f.SPSR1 = 0u;
            FM_RTC->VBPFR_f.VPFR2 = 0u;
            FM_RTC->VBDDR_f.VDDR2 = 1u;     
            FM_RTC->VBDOR_f.VDOR2 = ((1u == bInitLevel) ? 1u : 0u);
            break;
        case VbatGpioP48:
            FM_RTC->VBPFR_f.VPFR0 = 0u;
            FM_RTC->VBDDR_f.VDDR0 = 1u;
            FM_RTC->VBDOR_f.VDOR0 = ((1u == bInitLevel) ? 1u : 0u);
            FM_RTC->VBPZR_f.VPZR0 = ((1u == bOpenDrain) ? 1u : 0u);
            break;
        case VbatGpioP49:
            FM_RTC->VBPFR_f.VPFR1 = 0u;
            FM_RTC->VBDDR_f.VDDR1 = 1u;
            FM_RTC->VBDOR_f.VDOR1 = ((1u == bInitLevel) ? 1u : 0u);
            FM_RTC->VBPZR_f.VPZR1 = ((1u == bOpenDrain) ? 1u : 0u);
            break;
        default:
            return ErrorInvalidParameter;
    }
    
    // Trasnmit to VBAT domain 
    FM_RTC->WTCR20_f.PWRITE = 1;

    // Wait to complete transmission
    while(0u != FM_RTC->WTCR10_f.TRANS)
    {}
    
    return Ok;
}

/**
 ******************************************************************************
 ** \brief Initialize VBAT GPIO input
 **
 ** This function initializes an VBAT GPIO input
 **
 ** \param [in]  enGpio       VBAT GPIO list
 ** \param [in]  bPullup      GPIO pullup register connected or not
 **
 ** \retval Ok                    VBAT initialized normally
 ** \retval ErrorInvalidParameter enGpio out of range
 **
 ******************************************************************************/
en_result_t Vbat_InitGpioInput(en_vbat_gpio_t enGpio, boolean_t bPullup)
{
    // Recall from VBAT domain 
    FM_RTC->WTCR20_f.PREAD = 1;

    // Wait to complete transmission
    while(0u != FM_RTC->WTCR10_f.TRANS)
    {}
  
    // Set GPIO registers
    switch (enGpio)
    {
        case VbatGpioP46:
            FM_RTC->VBPFR_f.SPSR0 = 0u;
            FM_RTC->VBPFR_f.SPSR1 = 0u;
            FM_RTC->VBPFR_f.VPFR3 = 0u;
            FM_RTC->VBDDR_f.VDDR3 = 0u;
            FM_RTC->VBPCR_f.VPCR3 = ((1u == bPullup) ? 1u : 0u);
            break;
        case VbatGpioP47:
            FM_RTC->VBPFR_f.SPSR0 = 0u;
            FM_RTC->VBPFR_f.SPSR1 = 0u;
            FM_RTC->VBPFR_f.VPFR2 = 0u;
            FM_RTC->VBDDR_f.VDDR2 = 0u;     
            FM_RTC->VBPCR_f.VPCR2 = ((1u == bPullup) ? 1u : 0u);
            break;
        case VbatGpioP48:
            FM_RTC->VBPFR_f.VPFR0 = 0u;
            FM_RTC->VBDDR_f.VDDR0 = 0u;
            FM_RTC->VBPCR_f.VPCR0 = ((1u == bPullup) ? 1u : 0u);
            break;
        case VbatGpioP49:
            FM_RTC->VBPFR_f.VPFR1 = 0u;
            FM_RTC->VBDDR_f.VDDR1 = 0u;
            FM_RTC->VBPCR_f.VPCR1 = ((1u == bPullup) ? 1u : 0u);
            break;
        default:
            return ErrorInvalidParameter;
    }
    
    // Trasnmit to VBAT domain 
    FM_RTC->WTCR20_f.PWRITE = 1;

    // Wait to complete transmission
    while(0u != FM_RTC->WTCR10_f.TRANS)
    {}
    
    return Ok;
}

/**
 ******************************************************************************
 ** \brief Set the output level of P46
 **
 ** \param [in]  bLevel       Output level
 **
 ******************************************************************************/
void Vbat_PutPinP46(boolean_t bLevel)
{
    // Recall from VBAT domain 
    FM_RTC->WTCR20_f.PREAD = 1;

    // Wait to complete transmission
    while(0u != FM_RTC->WTCR10_f.TRANS)
    {}
    
    // Set output register
    FM_RTC->VBDOR_f.VDOR3 = ((1u == bLevel) ? 1u : 0u);
    
    // Trasnmit to VBAT domain 
    FM_RTC->WTCR20_f.PWRITE = 1;

    // Wait to complete transmission
    while(0u != FM_RTC->WTCR10_f.TRANS)
    {}
    
    return;
}

/**
 ******************************************************************************
 ** \brief Set the output level of P47
 **
 ** \param [in]  bLevel       Output level
 **
 ******************************************************************************/
void Vbat_PutPinP47(boolean_t bLevel)
{
    // Recall from VBAT domain 
    FM_RTC->WTCR20_f.PREAD = 1;

    // Wait to complete transmission
    while(0u != FM_RTC->WTCR10_f.TRANS)
    {}
    
    // Set output register
    FM_RTC->VBDOR_f.VDOR2 = ((1u == bLevel) ? 1u : 0u);
    
    // Trasnmit to VBAT domain 
    FM_RTC->WTCR20_f.PWRITE = 1;

    // Wait to complete transmission
    while(0u != FM_RTC->WTCR10_f.TRANS)
    {}
    
    return;
}

/**
 ******************************************************************************
 ** \brief Set the output level of P48
 **
 ** \param [in]  bLevel       Output level
 **
 ******************************************************************************/
void Vbat_PutPinP48(boolean_t bLevel)
{
    // Recall from VBAT domain 
    FM_RTC->WTCR20_f.PREAD = 1;

    // Wait to complete transmission
    while(0u != FM_RTC->WTCR10_f.TRANS)
    {}
    
    // Set output register
    FM_RTC->VBDOR_f.VDOR0 = ((1u == bLevel) ? 1u : 0u);
    
    // Trasnmit to VBAT domain 
    FM_RTC->WTCR20_f.PWRITE = 1;

    // Wait to complete transmission
    while(0u != FM_RTC->WTCR10_f.TRANS)
    {}
    
    return;
}

/**
 ******************************************************************************
 ** \brief Set the output level of P49
 **
 ** \param [in]  bLevel       Output level
 **
 ******************************************************************************/
void Vbat_PutPinP49(boolean_t bLevel)
{
    // Recall from VBAT domain 
    FM_RTC->WTCR20_f.PREAD = 1;

    // Wait to complete transmission
    while(0u != FM_RTC->WTCR10_f.TRANS)
    {}
    
    // Set output register
    FM_RTC->VBDOR_f.VDOR1 = ((1u == bLevel) ? 1u : 0u);
    
    // Trasnmit to VBAT domain 
    FM_RTC->WTCR20_f.PWRITE = 1;

    // Wait to complete transmission
    while(0u != FM_RTC->WTCR10_f.TRANS)
    {}
    
    return;
}

/**
 ******************************************************************************
 ** \brief Get the input level of P46
 **
 ** \retval TRUE              High level
 ** \retval FALSE             Low level
 **
 ******************************************************************************/
boolean_t Vbat_GetPinP46(void)
{
    boolean_t bRet;
  
    // Recall from VBAT domain 
    FM_RTC->WTCR20_f.PREAD = 1u;

    // Wait to complete transmission
    while(0u != FM_RTC->WTCR10_f.TRANS)
    {}
    
    bRet = ((1u == FM_RTC->VBDIR_f.VDIR3) ? 1u : 0u) ;
    
    return bRet;
}

/**
 ******************************************************************************
 ** \brief Get the input level of P47
 **
 ** \retval TRUE              High level
 ** \retval FALSE             Low level
 **
 ******************************************************************************/
boolean_t Vbat_GetPinP47(void)
{
    boolean_t bRet;
  
    // Recall from VBAT domain 
    FM_RTC->WTCR20_f.PREAD = 1u;

    // Wait to complete transmission
    while(0u != FM_RTC->WTCR10_f.TRANS)
    {}
    
    bRet = ((1u == FM_RTC->VBDIR_f.VDIR2) ? 1u : 0u) ;
    
    return bRet;
}

/**
 ******************************************************************************
 ** \brief Get the input level of P48
 **
 ** \retval TRUE              High level
 ** \retval FALSE             Low level
 **
 ******************************************************************************/
boolean_t Vbat_GetPinP48(void)
{
    boolean_t bRet;
  
    // Recall from VBAT domain 
    FM_RTC->WTCR20_f.PREAD = 1u;

    // Wait to complete transmission
    while(0u != FM_RTC->WTCR10_f.TRANS)
    {}
    
    bRet = ((1u == FM_RTC->VBDIR_f.VDIR0) ? 1u : 0u) ;
    
    return bRet;
}

/**
 ******************************************************************************
 ** \brief Get the input level of P49
 **
 ** \retval TRUE              High level
 ** \retval FALSE             Low level
 **
 ******************************************************************************/
boolean_t Vbat_GetPinP49(void)
{
    boolean_t bRet;
  
    // Recall from VBAT domain 
    FM_RTC->WTCR20_f.PREAD = 1u;

    // Wait to complete transmission
    while(0u != FM_RTC->WTCR10_f.TRANS)
    {}
    
    bRet = ((1u == FM_RTC->VBDIR_f.VDIR1) ? 1u : 0u) ;
    
    return bRet;
}

/**
 ******************************************************************************
 ** \brief Start hibernation function of VBAT
 ******************************************************************************/
void Vbat_StartHibernation(void)
{
    FM_RTC->HIBRST = 1u;
    
    return;
}

/**
 ******************************************************************************
 ** \brief Get power on flag of VBAT
 **
 ** \retval TRUE Power on flag set
 ** \retval FALSE Power on flag clear
 ******************************************************************************/
boolean_t Vbat_GetPowerOnFlag(void)
{
    boolean_t bRet;
    
    bRet = (1u == FM_RTC->VDET_f.PON) ? 1u : 0u;
    
    return bRet;
}

/**
 ******************************************************************************
 ** \brief Get wakeup flag of VBAT
 **
 ** \retval TRUE wakeup flag set
 ** \retval FALSE wakeup flag clear
 ******************************************************************************/
boolean_t Vbat_GetWakeupFlag(void)
{
    boolean_t bRet;
    
    bRet = (1u == FM_RTC->EWKUP_f.WUP0) ? 1u : 0u;
    
    return bRet;
}

/**
 ******************************************************************************
 ** \brief Clear wakeup flag of VBAT
 ******************************************************************************/
void Vbat_ClrWakeupFlag(void)
{
    FM_RTC->EWKUP_f.WUP0 = 0u;
}

//@} // VbatGroup

#endif

#endif // #if (defined(PDL_PERIPHERAL_VBAT_ACTIVE))

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
