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
/** \file clk.c
 **
 ** A detailed description is available at 
 ** @link ClkGroup Clock Module description @endlink
 **
 ** History:
 **   - 2014-11-14  1.0  EZh  First version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "clk.h"

#if (defined(PDL_PERIPHERAL_CLK_ACTIVE))

/**
 ******************************************************************************
 ** \addtogroup ClkGroup
 ******************************************************************************/
//@{

/******************************************************************************/
/* Local pre-processor symbols/macros ('#define')                             */
/******************************************************************************/

/******************************************************************************/
/* Global variable definitions (declared in header file with 'extern')        */
/******************************************************************************/
#if (PDL_INTERRUPT_ENABLE_CLK == PDL_ON)
stc_clk_intern_data_t stcClkInternData;
#endif

/******************************************************************************/
/* Local type definitions ('typedef')                                         */
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

/**
 ******************************************************************************
 ** \brief Clock Stabilization Interrupt Handler
 ******************************************************************************/
#if (PDL_INTERRUPT_ENABLE_CLK == PDL_ON)
void Clk_IrqHandler(void)
{
  uint8_t u8IntStrReadOut;
  
  u8IntStrReadOut = FM_CRG->INT_STR;
  
  // PLL stabilization ready?
  if (0u != (u8IntStrReadOut & FM_INT_STR_PCSI_BITPOS))
  {
    FM_CRG->INT_CLR |= FM_INT_CLR_PCSC_BITPOS;  // Clear Irq
    
    // PLL ready callback if defined
    if (NULL != stcClkInternData.pfnPllStabCb)
    {
      stcClkInternData.pfnPllStabCb();
    }
  }

  // Sub Clock stabilization ready?  
  if (0u != (u8IntStrReadOut & FM_INT_STR_SCSI_BITPOS))
  {
    FM_CRG->INT_CLR |= FM_INT_CLR_SCSC_BITPOS;  // Clear Irq
    
    // Sub Clock ready callback if defined
    if (NULL != stcClkInternData.pfnScoStabCb)
    {
      stcClkInternData.pfnScoStabCb();
    }
  }

  // Main Clock stabilization ready?  
  if (0u != (u8IntStrReadOut & FM_INT_STR_MCSI_BITPOS))
  {
    FM_CRG->INT_CLR |= FM_INT_CLR_MCSC_BITPOS;  // Clear Irq
    
    // Main Clock ready callback if defined
    if (NULL != stcClkInternData.pfnMcoStabCb)
    {
      stcClkInternData.pfnMcoStabCb();
    }
  }
}
#endif

/**
 ******************************************************************************
 ** \brief Initialize system clock according to user configuration
 **
 ** Set the definition CLOCK_SETUP to "CLOCK_SETTING_NONE" when using this
 ** function to initialize system clock.
 **
 ** \param [in]  pstcClk         Pointer to clock configuration structure
 **
 ** \retval Ok    Clock initialized normally
 ** \retval ErrorInvalidParameter    The paramter is set to error range    
 ******************************************************************************/
en_result_t Clk_Init(stc_clk_config_t* pstcClk) 
{
    if(pstcClk == NULL)
    {
        return ErrorInvalidParameter;
    }
    
    /* Set base clock dividor */
    switch(pstcClk->enBaseClkDiv)
    {
        case BaseClkDiv1:
            FM_CRG->BSC_PSR_f.BSR = 0u;
            break;
        case BaseClkDiv2:
            FM_CRG->BSC_PSR_f.BSR = 1u;
            break;    
        case BaseClkDiv3:
            FM_CRG->BSC_PSR_f.BSR = 2u;
            break;
        case BaseClkDiv4:
            FM_CRG->BSC_PSR_f.BSR = 3u;
            break;
        case BaseClkDiv6:
            FM_CRG->BSC_PSR_f.BSR = 4u;
            break;
        case BaseClkDiv8:
            FM_CRG->BSC_PSR_f.BSR = 5u;
            break;
        case BaseClkDiv16:
            FM_CRG->BSC_PSR_f.BSR = 6u;
            break;    
        default:
            return ErrorInvalidParameter;
            
    }
    
    /* Set APB0 bus clock dividor */
    switch(pstcClk->enAPB0Div)
    {
        case Apb0Div1:
            FM_CRG->APBC0_PSR_f.APBC0 = 0u;
            break;
        case Apb0Div2:
            FM_CRG->APBC0_PSR_f.APBC0 = 1u;
            break;
        case Apb0Div4:
            FM_CRG->APBC0_PSR_f.APBC0 = 2u;
            break;
        case Apb0Div8:
            FM_CRG->APBC0_PSR_f.APBC0 = 3u;
            break;    
        default:
            return ErrorInvalidParameter;
    }
    
    /* Set APB1 bus clock dividor */
    switch(pstcClk->enAPB1Div)
    {
        case Apb1Div1:
            FM_CRG->APBC1_PSR_f.APBC1 = 0u;
            break;
        case Apb1Div2:
            FM_CRG->APBC1_PSR_f.APBC1 = 1u;
            break;
        case Apb1Div4:
            FM_CRG->APBC1_PSR_f.APBC1 = 2u;
            break;
        case Apb1Div8:
            FM_CRG->APBC1_PSR_f.APBC1 = 3u;
            break;    
        default:
            return ErrorInvalidParameter;
    }
    
    if(TRUE == pstcClk->bAPB1Disable)
    {
        FM_CRG->APBC1_PSR_f.APBC1EN = 0;
    }
    
    /* Configure stability wait time */
    FM_CRG->CSW_TMR_f.MOWT = pstcClk->enMCOWaitTime;
    FM_CRG->CSW_TMR_f.SOWT = pstcClk->enSCOWaitTime;
    FM_CRG->PSW_TMR_f.POWT = pstcClk->enPLLOWaitTime;
    
#if (PDL_INTERRUPT_ENABLE_CLK == PDL_ON)    
    /* Configure interrupt */
    if(TRUE == pstcClk->bMcoIrq)
    {
        if(NULL == pstcClk->pfnMcoStabCb)
        {
            return ErrorInvalidParameter;
        }
        
        FM_CRG->INT_ENR_f.MCSE = 1;
        stcClkInternData.pfnMcoStabCb = pstcClk->pfnMcoStabCb;
    }
    
    if(TRUE == pstcClk->bScoIrq)
    {
        if(NULL == pstcClk->pfnScoStabCb)
        {
            return ErrorInvalidParameter;
        }
        
        FM_CRG->INT_ENR_f.SCSE = 1;
        stcClkInternData.pfnScoStabCb = pstcClk->pfnScoStabCb;
    }
    
    if(TRUE == pstcClk->bPllIrq)
    {
        if(NULL == pstcClk->pfnPllStabCb)
        {
            return ErrorInvalidParameter;
        }
        
        FM_CRG->INT_ENR_f.PCSE = 1;
        stcClkInternData.pfnPllStabCb = pstcClk->pfnPllStabCb;
    }
    
    #if (PDL_MCU_CORE == PDL_FM3_CORE)
    NVIC_ClearPendingIRQ(OSC_PLL_WC_RTC_IRQn);
    NVIC_EnableIRQ(OSC_PLL_WC_RTC_IRQn);
    NVIC_SetPriority(OSC_PLL_WC_RTC_IRQn, PDL_IRQ_LEVEL_CLK_WC_RTC);
    #elif (PDL_MCU_CORE == PDL_FM4_CORE)
    NVIC_ClearPendingIRQ(TIM_IRQn);
    NVIC_EnableIRQ(TIM_IRQn);
    NVIC_SetPriority(TIM_IRQn, PDL_IRQ_LEVEL_CLK);
    #else
    #if (PDL_MCU_TYPE == PDL_FM0P_TYPE3)
    NVIC_ClearPendingIRQ(CLK_IRQn);
    NVIC_EnableIRQ(CLK_IRQn);
    NVIC_SetPriority(CLK_IRQn, PDL_IRQ_LEVEL_CLK);
    #else
    NVIC_ClearPendingIRQ(ICC1_HDMICEC1_OSC_PLL_WC_RTC_IRQn);
    NVIC_EnableIRQ(ICC1_HDMICEC1_OSC_PLL_WC_RTC_IRQn);
    NVIC_SetPriority(ICC1_HDMICEC1_OSC_PLL_WC_RTC_IRQn, PDL_IRQ_LEVEL_CLK_WC_RTC_CEC1_ICC1);
    #endif
    #endif    
    
#endif    
    
    /* Set PLL K, M, N */
    FM_CRG->PLL_CTL1_f.PLLK = pstcClk->u8PllK - 1u;
    FM_CRG->PLL_CTL1_f.PLLM = pstcClk->u8PllM - 1u;
    FM_CRG->PLL_CTL2 = pstcClk->u8PllN - 1u;
    
    return Ok;
}

#if (PDL_MCU_CORE == PDL_FM0P_CORE)   
/**
 ******************************************************************************
 ** \brief Enable high speed CR
 **
 ** This function easily enables the high speed CR. No configuration is needed.
 **
 ** \param bBlock    Wait until CR stability or not
 ** \arg   FALSE     Return immediately after enable high speed CR
 ** \arg   TRUE      Wait until CR stability after enable high speed CR
 **
 ** \retval Ok                high speed CR enabled
 ******************************************************************************/
en_result_t Clk_EnableHscr(boolean_t bBlock)
{
  FM_CRG->SCM_CTL_f.HCRE = 1u;
  
  if(TRUE == bBlock)
  {
    while(FM_CRG->SCM_STR_f.HCRDY != 1u);
  }

  return Ok;
} // Clk_EnableMainClock

/**
 ******************************************************************************
 ** \brief Disable Main Clock
 **
 ** This function easily disables the Main Clock. No configuration is needed.
 **
 ** \retval Ok                Main Clock disabled
 ******************************************************************************/
en_result_t Clk_DisableHscr(void)
{
  FM_CRG->SCM_CTL_f.HCRE = 0u;

  return Ok;
} // Clk_DisableMainClock
#endif

/**
 ******************************************************************************
 ** \brief Enable Main Clock and wait until it is stable
 **
 ** This function easily enables the Main Clock. No configuration is needed.
 ** 
 ** \param bBlock    Wait until Main Clock stability or not
 ** \arg   FALSE     Return immediately after enable Main Clock
 ** \arg   TRUE      Wait until Main Clock stability after enable Main Clock
 **
 ** \retval Ok                Main Clock enabled
 ******************************************************************************/
en_result_t Clk_EnableMainClock(boolean_t bBlock)
{
  FM_CRG->SCM_CTL_f.MOSCE = 1u;
  
  if(TRUE == bBlock)
  {
    while(1u != FM_CRG->SCM_STR_f.MORDY);
  }

  return Ok;
} // Clk_EnableMainClock

/**
 ******************************************************************************
 ** \brief Disable Main Clock
 **
 ** This function easily disables the Main Clock. No configuration is needed.
 **
 ** \retval Ok                Main Clock disabled
 ******************************************************************************/
en_result_t Clk_DisableMainClock(void)
{
  FM_CRG->SCM_CTL_f.MOSCE = 0u;

  return Ok;
} // Clk_DisableMainClock


/**
 ******************************************************************************
 ** \brief Enable Sub Clock
 **
 ** This function easily enables the Sub Clock. No configuration is needed.
 **
 ** \param bBlock    Wait until Sub Clock stability or not
 ** \arg   FALSE     Return immediately after enable Sub Clock
 ** \arg   TRUE      Wait until Sub Clock stability after enable Sub Clock
 **
 ** \retval Ok                Sub Clock enabled
 ******************************************************************************/
en_result_t Clk_EnableSubClock(boolean_t bBlock)
{
  FM_CRG->SCM_CTL_f.SOSCE = 1u;
  
  if(TRUE == bBlock)
  {
    while(1u != FM_CRG->SCM_STR_f.SORDY);
  }

  return Ok;
} // Clk_EnableSubClock

/**
 ******************************************************************************
 ** \brief Disable Sub Clock
 **
 ** This function easily disables the Sub Clock. No configuration is needed.
 **
 ** \retval Ok                Sub Clock disabled
 ******************************************************************************/
en_result_t Clk_DisableSubClock(void)
{
  FM_CRG->SCM_CTL_f.SOSCE = 0u;

  return Ok;
} // Clk_DisableSubClock


/**
 ******************************************************************************
 ** \brief Enable PLL Clock
 **
 ** This function easily enables the PLL Clock. No configuration is needed.
 **
 ** \param bBlock    Wait until PLL Clock stability or not
 ** \arg   FALSE     Return immediately after enable PLL Clock
 ** \arg   TRUE      Wait until PLL Clock stability after enable PLL Clock
 **
 ** \retval Ok                PLL Clock enabled
 ******************************************************************************/
en_result_t Clk_EnablePllClock(boolean_t bBlock)
{
  FM_CRG->SCM_CTL_f.PLLE = 1u;
  
  if(bBlock == TRUE)
  {
    while(1u != FM_CRG->SCM_STR_f.PLRDY);
  }

  return Ok;
} // Clk_EnableSubClock

/**
 ******************************************************************************
 ** \brief Disable PLL Clock
 **
 ** This function easily disables the PLL Clock. No configuration is needed.
 **
 ** \retval Ok                PLL Clock disabled
 ******************************************************************************/
en_result_t Clk_DisablePllClock(void)
{
  FM_CRG->SCM_CTL_f.PLLE = 0u;

  return Ok;
} // Clk_DisableSubClock

/**
 ******************************************************************************
 ** \brief Set Clock Source
 **
 ** This function sets the clock source and performs transition, if wanted.
 **
 ** \param [in]  enSource        System source clock
 ** \arg         ClkMain         Set Main Clock as system source clock
 ** \arg         ClkSub          Set Sub Clock as system source clock
 ** \arg         ClkHsCr         Set High-speed CR as system source clock
 ** \arg         ClkLsCr         Set Low-speed CR as system source clock
 ** \arg         ClkPll          Set Main PLL clock as system source clock
 ** \arg         ClkHsCrPll      Set High-speed CR PLL clock as system source clock
 ** 
 ** \retval Ok                     Clock source set
 ** \retval ErrorInvalidParameter  pstcConfig == NULL or Illegal mode
 ** \retval ErrorInvalidMode       Clock setting not possible
 ******************************************************************************/
en_result_t Clk_SetSource(en_clk_source_t enSource)
{  
  uint8_t u8Rcs, u8Rcm;
  switch(enSource)
  {
    case ClkMain:
      if ((TRUE != FM_CRG->SCM_CTL_f.MOSCE) ||  // Main Oscillator ready?
          (TRUE != FM_CRG->SCM_STR_f.MORDY))
      {
        return ErrorInvalidMode ;
      }
      FM_CRG->SCM_CTL_f.RCS = 0x1u;
      break;
    case ClkSub:
      if ((TRUE != FM_CRG->SCM_CTL_f.SOSCE) ||  // Sub Oscillator ready?
          (TRUE != FM_CRG->SCM_STR_f.SORDY))
      {
        return ErrorInvalidMode ;
      }
      FM_CRG->SCM_CTL_f.RCS = 0x5u;
      break;
    case ClkHsCr:                                // Always possible
      FM_CRG->SCM_CTL_f.RCS = 0x0u;
      break;     
    case ClkLsCr:                                // Always possible
      FM_CRG->SCM_CTL_f.RCS = 0x4u;
      break;
    case ClkHsCrPll:  
      FM_CRG->PSW_TMR_f.PINC = 1u;
      FM_CRG->SCM_CTL_f.RCS = 0x2u;
      break;
    case ClkPll:                           
      if ((TRUE != FM_CRG->SCM_STR_f.MORDY) ||  // PLL ready?
          (TRUE != FM_CRG->SCM_STR_f.PLRDY))
      {
        return ErrorInvalidMode ;
      }
      
      FM_CRG->PSW_TMR_f.PINC = 0u;
      FM_CRG->SCM_CTL_f.RCS = 0x2u;
      break;
    default:
      return ErrorInvalidParameter ;
  }

  /* Wait until switch stable */
  while(1)
  {
    u8Rcs = FM_CRG->SCM_CTL_f.RCS;
    u8Rcm = FM_CRG->SCM_STR_f.RCM;
    if(u8Rcs == u8Rcm)
    {
        break;
    }
  }

  return Ok;
} // Clk_SetSource

#if (PDL_MCU_CORE == PDL_FM0P_CORE) || (PDL_MCU_CORE == PDL_FM4_CORE)
/**
 ******************************************************************************
 ** \brief Enables the clock gate of a peripheral
 **
 ** This function sets the corresponding bit in the CKENn register to enable
 ** the clock of a peripheral.
 **
 ** \param  enPeripheral           Enumerator of a peripheral, see
 **                                #en_clk_gate_peripheral_t for details
 **
 ** \retval Ok                     Peripheral clock enabled
 ** \retval ErrorInvalidParameter  Peripheral enumerator does not exist
 ******************************************************************************/
en_result_t Clk_PeripheralClockEnable(en_clk_gate_peripheral_t enPeripheral)
{
  switch (enPeripheral)
  {
    case ClkGateGpio:
      FM_PCG->CKEN0_f.GIOCK = 1u;
      break;     
    case ClkGateDma:
      FM_PCG->CKEN0_f.DMACK = 1u;
      break;
    case ClkGateAdc0:
      FM_PCG->CKEN0_f.ADCCK0 = 1u;
      break;
    case ClkGateAdc1:
      FM_PCG->CKEN0_f.ADCCK1 = 1u;
      break;
    case ClkGateAdc2:
      FM_PCG->CKEN0_f.ADCCK2 = 1u;
      break;
    case ClkGateAdc3:
      FM_PCG->CKEN0_f.ADCCK3 = 1u;
      break;
    case ClkGateMfs0:
      FM_PCG->CKEN0_f.MFSCK0 = 1u;
      break;
    case ClkGateMfs1:
      FM_PCG->CKEN0_f.MFSCK1 = 1u;
      break;
    case ClkGateMfs2:
      FM_PCG->CKEN0_f.MFSCK2 = 1u;
      break;
    case ClkGateMfs3:
      FM_PCG->CKEN0_f.MFSCK3 = 1u;
      break;
    case ClkGateMfs4:
      FM_PCG->CKEN0_f.MFSCK4 = 1u;
      break;
    case ClkGateMfs5:
      FM_PCG->CKEN0_f.MFSCK5 = 1u;
      break;
    case ClkGateMfs6:
      FM_PCG->CKEN0_f.MFSCK6 = 1u;
      break;
    case ClkGateMfs7:
      FM_PCG->CKEN0_f.MFSCK7 = 1u;
      break;
    case ClkGateMfs8:
      FM_PCG->CKEN0_f.MFSCK8 = 1u;
      break;
    case ClkGateMfs9:
      FM_PCG->CKEN0_f.MFSCK9 = 1u;
      break;
    case ClkGateMfs10:
      FM_PCG->CKEN0_f.MFSCK10 = 1u;
      break;
    case ClkGateMfs11:
      FM_PCG->CKEN0_f.MFSCK11 = 1u;
      break;
    case ClkGateMfs12:
      FM_PCG->CKEN0_f.MFSCK12 = 1u;
      break;
    case ClkGateMfs13:
      FM_PCG->CKEN0_f.MFSCK13 = 1u;
      break;
    case ClkGateMfs14:
      FM_PCG->CKEN0_f.MFSCK14 = 1u;
      break;
    case ClkGateMfs15:
      FM_PCG->CKEN0_f.MFSCK15 = 1u;
      break;
    case ClkGateQprc0:
      FM_PCG->CKEN1_f.QDUCK0 = 1u;
      break;
    case ClkGateQprc1:
      FM_PCG->CKEN1_f.QDUCK1 = 1u;
      break;
    case ClkGateQprc2:
      FM_PCG->CKEN1_f.QDUCK2 = 1u;
      break;
    case ClkGateQprc3:
      FM_PCG->CKEN1_f.QDUCK3 = 1u;
      break;
    case ClkGateMft0:
      FM_PCG->CKEN1_f.MFTCK0 = 1u;
      break;
    case ClkGateMft1:
      FM_PCG->CKEN1_f.MFTCK1 = 1u;
      break;
    case ClkGateMft2:
      FM_PCG->CKEN1_f.MFTCK2 = 1u;
      break;
    case ClkGateMft3:
      FM_PCG->CKEN1_f.MFTCK3 = 1u;
      break;
    case ClkGateBt0:
      FM_PCG->CKEN1_f.BTMCK0 = 1u;
      break;
    case ClkGateBt4:
      FM_PCG->CKEN1_f.BTMCK1 = 1u;
      break;
    case ClkGateBt8:
      FM_PCG->CKEN1_f.BTMCK2 = 1u;
      break;
    case ClkGateBt12:
      FM_PCG->CKEN1_f.BTMCK3 = 1u;
      break;
#if (PDL_MCU_CORE == PDL_FM4_CORE)  
    case ClkGateExtif:
      FM_PCG->CKEN0_f.EXBCK = 1u;
      break;      
    case ClkGateUsb0:
      FM_PCG->CKEN2_f.USBCK0 = 1u;
      break;
    case ClkGateUsb1:
      FM_PCG->CKEN2_f.USBCK1 = 1u;
      break;    
    case ClkGateCan0:
      FM_PCG->CKEN2_f.CANCK0 = 1u;
      break;
    case ClkGateCan1:
      FM_PCG->CKEN2_f.CANCK1 = 1u;
      break;      
    case ClkGateCan2:
      FM_PCG->CKEN2_f.CANCK2 = 1u;
      break;
    case ClkGateSd:
      FM_PCG->CKEN2_f.SDCCK = 1u;
      break; 
    case ClkGateI2s0:
      FM_PCG->CKEN2_f.I2SCK0 = 1u;
      break;
    case ClkGateI2s1:
      FM_PCG->CKEN2_f.I2SCK1 = 1u;
      break;      
    case ClkGateCrc:
      FM_PCG->CKEN2_f.PCRCCK = 1u;
      break;
    case ClkGateQspi:
      FM_PCG->CKEN2_f.QSPICK = 1u;
      break; 
#endif
#if (PDL_MCU_CORE == PDL_FM0P_CORE)  
    case ClkGateCec0:
    case ClkGateCec1:
      FM_PCG->CKEN2_f.CECCK = 1u;
      break;  
#endif
#if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE)  
    case ClkGateIcc0:
      FM_PCG->CKEN2_f.ICCCK0 = 1u;
      break;
    case ClkGateIcc1:
      FM_PCG->CKEN2_f.ICCCK1 = 1u;
      break;
    case ClkGateI2sl0:
      FM_PCG->CKEN2_f.IISCCK0 = 1u;
      break;
    case ClkGateI2sl1:
      FM_PCG->CKEN2_f.IISCCK1 = 1u;
      break;      
#endif      
    default:
      return ErrorInvalidParameter;
  }
  
  return Ok;
} // Clk_PeripheralClockDisable
  
/**
 ******************************************************************************
 ** \brief Read the clock gate state of a peripheral
 **
 ** This function reads out the corresponding bit in the CKENn register.
 **
 ** \param  enPeripheral           Enumerator of a peripheral, see
 **                                #en_clk_gate_peripheral_t for details
 **
 ** \retval TRUE                   Peripheral clock enabled
 ** \retval FALSE                  Peripheral clock not enabled, peripheral
 **                                not existing
 ******************************************************************************/
boolean_t Clk_PeripheralGetClockState(en_clk_gate_peripheral_t enPeripheral)
{
  switch (enPeripheral)
  {
    case ClkGateGpio:
      return ((1u == FM_PCG->CKEN0_f.GIOCK) ? TRUE : FALSE);    
    case ClkGateDma:
      return ((1u == FM_PCG->CKEN0_f.DMACK) ? TRUE : FALSE);
    case ClkGateAdc0:
      return ((1u == FM_PCG->CKEN0_f.ADCCK0) ? TRUE : FALSE);
    case ClkGateAdc1:
      return ((1u == FM_PCG->CKEN0_f.ADCCK1) ? TRUE : FALSE);
    case ClkGateAdc2:
      return ((1u == FM_PCG->CKEN0_f.ADCCK2) ? TRUE : FALSE);
    case ClkGateAdc3:
      return ((1u == FM_PCG->CKEN0_f.ADCCK3) ? TRUE : FALSE);
    case ClkGateMfs0:
      return ((1u == FM_PCG->CKEN0_f.MFSCK0) ? TRUE : FALSE);
    case ClkGateMfs1:
      return ((1u == FM_PCG->CKEN0_f.MFSCK1) ? TRUE : FALSE);
    case ClkGateMfs2:
      return ((1u == FM_PCG->CKEN0_f.MFSCK2) ? TRUE : FALSE);
    case ClkGateMfs3:
      return ((1u == FM_PCG->CKEN0_f.MFSCK3) ? TRUE : FALSE);
    case ClkGateMfs4:
      return ((1u == FM_PCG->CKEN0_f.MFSCK4) ? TRUE : FALSE);
    case ClkGateMfs5:
      return ((1u == FM_PCG->CKEN0_f.MFSCK5) ? TRUE : FALSE);
    case ClkGateMfs6:
      return ((1u == FM_PCG->CKEN0_f.MFSCK6) ? TRUE : FALSE);
    case ClkGateMfs7:
      return ((1u == FM_PCG->CKEN0_f.MFSCK7) ? TRUE : FALSE);
    case ClkGateMfs8:
      return ((1u == FM_PCG->CKEN0_f.MFSCK8) ? TRUE : FALSE);
    case ClkGateMfs9:
      return ((1u == FM_PCG->CKEN0_f.MFSCK9) ? TRUE : FALSE);
    case ClkGateMfs10:
      return ((1u ==  FM_PCG->CKEN0_f.MFSCK10) ? TRUE : FALSE);
    case ClkGateMfs11:
      return ((1u == FM_PCG->CKEN0_f.MFSCK11) ? TRUE : FALSE);
    case ClkGateMfs12:
      return ((1u == FM_PCG->CKEN0_f.MFSCK12) ? TRUE : FALSE);
    case ClkGateMfs13:
      return ((1u == FM_PCG->CKEN0_f.MFSCK13) ? TRUE : FALSE);
    case ClkGateMfs14:
      return ((1u == FM_PCG->CKEN0_f.MFSCK14) ? TRUE : FALSE);
    case ClkGateMfs15:
      return ((1u == FM_PCG->CKEN0_f.MFSCK15) ? TRUE : FALSE);
    case ClkGateQprc0:
      return ((1u == FM_PCG->CKEN1_f.QDUCK0) ? TRUE : FALSE);
    case ClkGateQprc1:
      return ((1u == FM_PCG->CKEN1_f.QDUCK1) ? TRUE : FALSE);
    case ClkGateQprc2:
      return ((1u == FM_PCG->CKEN1_f.QDUCK2) ? TRUE : FALSE);
    case ClkGateQprc3:
      return ((1u == FM_PCG->CKEN1_f.QDUCK3) ? TRUE : FALSE);
    case ClkGateMft0:
      return ((1u == FM_PCG->CKEN1_f.MFTCK0) ? TRUE : FALSE);
    case ClkGateMft1:
      return ((1u == FM_PCG->CKEN1_f.MFTCK1) ? TRUE : FALSE);
    case ClkGateMft2:
      return ((1u == FM_PCG->CKEN1_f.MFTCK2) ? TRUE : FALSE);
    case ClkGateMft3:
      return ((1u == FM_PCG->CKEN1_f.MFTCK3) ? TRUE : FALSE);
    case ClkGateBt0:
      return ((1u == FM_PCG->CKEN1_f.BTMCK0) ? TRUE : FALSE);
    case ClkGateBt4:
      return ((1u == FM_PCG->CKEN1_f.BTMCK1) ? TRUE : FALSE);
    case ClkGateBt8:
      return ((1u == FM_PCG->CKEN1_f.BTMCK2) ? TRUE : FALSE);
    case ClkGateBt12:  
      return ((1u == FM_PCG->CKEN1_f.BTMCK3) ? TRUE : FALSE);
#if (PDL_MCU_CORE == PDL_FM4_CORE) 
    case ClkGateExtif:
      return ((1u == FM_PCG->CKEN0_f.EXBCK) ? TRUE : FALSE);    
    case ClkGateUsb0:
      return ((1u == FM_PCG->CKEN2_f.USBCK0) ? TRUE : FALSE);
    case ClkGateUsb1:
      return ((1u == FM_PCG->CKEN2_f.USBCK1) ? TRUE : FALSE);    
    case ClkGateCan0:
      return ((1u == FM_PCG->CKEN2_f.CANCK0) ? TRUE : FALSE);
    case ClkGateCan1:
      return ((1u == FM_PCG->CKEN2_f.CANCK1) ? TRUE : FALSE);   
    case ClkGateCan2:
      return ((1u == FM_PCG->CKEN2_f.CANCK2) ? TRUE : FALSE);      
    case ClkGateSd:
      return ((1u == FM_PCG->CKEN2_f.SDCCK) ? TRUE : FALSE);
    case ClkGateI2s0:
      return ((1u == FM_PCG->CKEN2_f.I2SCK0) ? TRUE : FALSE);
    case ClkGateI2s1:
      return ((1u == FM_PCG->CKEN2_f.I2SCK1) ? TRUE : FALSE);
    case ClkGateCrc:
      return ((1u == FM_PCG->CKEN2_f.PCRCCK) ? TRUE : FALSE);
    case ClkGateQspi:
      return ((1u == FM_PCG->CKEN2_f.QSPICK) ? TRUE : FALSE);     
#endif
#if (PDL_MCU_CORE == PDL_FM0P_CORE)
    case ClkGateCec0:
    case ClkGateCec1:
      return ((1u == FM_PCG->CKEN2_f.CECCK) ? TRUE : FALSE);   
#endif      
#if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE)
    case ClkGateIcc0:
      return ((1u == FM_PCG->CKEN2_f.ICCCK0) ? TRUE : FALSE);  
    case ClkGateIcc1:
      return ((1u == FM_PCG->CKEN2_f.ICCCK1) ? TRUE : FALSE);  
    case ClkGateI2sl0:
      return ((1u == FM_PCG->CKEN2_f.IISCCK0) ? TRUE : FALSE);  
    case ClkGateI2sl1:
      return ((1u == FM_PCG->CKEN2_f.IISCCK1) ? TRUE : FALSE);    
#endif      
    default:
      break;
  }
  
  return FALSE; // Peripheral not found -> always FALSE
} // Clk_PeripheralGetClockState

/**
 ******************************************************************************
 ** \brief Disables the clock gate of a peripheral
 **
 ** This function clears the corresponding bit in the CKENn register to enable
 ** the clock of a peripheral.
 **
 ** \param  enPeripheral           Enumerator of a peripheral, see
 **                                #en_clk_gate_peripheral_t for details
 **
 ** \retval Ok                     Peripheral clock disabled
 ** \retval ErrorInvalidParameter  Peripheral enumerator does not exist
 ******************************************************************************/
en_result_t Clk_PeripheralClockDisable(en_clk_gate_peripheral_t enPeripheral)
{
  switch (enPeripheral)
  {
    case ClkGateGpio:
      FM_PCG->CKEN0_f.GIOCK = 0u;
      break;    
    case ClkGateDma:
      FM_PCG->CKEN0_f.DMACK = 0u;
      break;
    case ClkGateAdc0:
      FM_PCG->CKEN0_f.ADCCK0 = 0u;
      break;
    case ClkGateAdc1:
      FM_PCG->CKEN0_f.ADCCK1 = 0u;
      break;
    case ClkGateAdc2:
      FM_PCG->CKEN0_f.ADCCK2 = 0u;
      break;
    case ClkGateAdc3:
      FM_PCG->CKEN0_f.ADCCK3 = 0u;
      break;
    case ClkGateMfs0:
      FM_PCG->CKEN0_f.MFSCK0 = 0u;
      break;
    case ClkGateMfs1:
      FM_PCG->CKEN0_f.MFSCK1 = 0u;
      break;
    case ClkGateMfs2:
      FM_PCG->CKEN0_f.MFSCK2 = 0u;
      break;
    case ClkGateMfs3:
      FM_PCG->CKEN0_f.MFSCK3 = 0u;
      break;
    case ClkGateMfs4:
      FM_PCG->CKEN0_f.MFSCK4 = 0u;
      break;
    case ClkGateMfs5:
      FM_PCG->CKEN0_f.MFSCK5 = 0u;
      break;
    case ClkGateMfs6:
      FM_PCG->CKEN0_f.MFSCK6 = 0u;
      break;
    case ClkGateMfs7:
      FM_PCG->CKEN0_f.MFSCK7 = 0u;
      break;
    case ClkGateMfs8:
      FM_PCG->CKEN0_f.MFSCK8 = 0u;
      break;
    case ClkGateMfs9:
      FM_PCG->CKEN0_f.MFSCK9 = 0u;
      break;
    case ClkGateMfs10:
      FM_PCG->CKEN0_f.MFSCK10 = 0u;
      break;
    case ClkGateMfs11:
      FM_PCG->CKEN0_f.MFSCK11 = 0u;
      break;
    case ClkGateMfs12:
      FM_PCG->CKEN0_f.MFSCK12 = 0u;
      break;
    case ClkGateMfs13:
      FM_PCG->CKEN0_f.MFSCK13 = 0u;
      break;
    case ClkGateMfs14:
      FM_PCG->CKEN0_f.MFSCK14 = 0u;
      break;
    case ClkGateMfs15:
      FM_PCG->CKEN0_f.MFSCK15 = 0u;
      break;
    case ClkGateQprc0:
      FM_PCG->CKEN1_f.QDUCK0 = 0u;
      break;
    case ClkGateQprc1:
      FM_PCG->CKEN1_f.QDUCK1 = 0u;
      break;
    case ClkGateQprc2:
      FM_PCG->CKEN1_f.QDUCK2 = 0u;
      break;
    case ClkGateQprc3:
      FM_PCG->CKEN1_f.QDUCK3 = 0u;
      break;
    case ClkGateMft0:
      FM_PCG->CKEN1_f.MFTCK0 = 0u;
      break;
    case ClkGateMft1:
      FM_PCG->CKEN1_f.MFTCK1 = 0u;
      break;
    case ClkGateMft2:
      FM_PCG->CKEN1_f.MFTCK2 = 0u;
      break;
    case ClkGateMft3:
      FM_PCG->CKEN1_f.MFTCK3 = 0u;
      break;
    case ClkGateBt0:
      FM_PCG->CKEN1_f.BTMCK0 = 0u;
      break;
    case ClkGateBt4:
      FM_PCG->CKEN1_f.BTMCK1 = 0u;
      break;
    case ClkGateBt8:
      FM_PCG->CKEN1_f.BTMCK2 = 0u;
      break;
    case ClkGateBt12:
      FM_PCG->CKEN1_f.BTMCK3 = 0u;
      break;
#if (PDL_MCU_CORE == PDL_FM4_CORE)
    case ClkGateExtif:
      FM_PCG->CKEN0_f.EXBCK = 0u;
      break;  
    case ClkGateUsb0:
      FM_PCG->CKEN2_f.USBCK0 = 0u;
      break;
    case ClkGateUsb1:
      FM_PCG->CKEN2_f.USBCK1 = 0u;
      break;   
    case ClkGateCan0:
      FM_PCG->CKEN2_f.CANCK0 = 0u;
      break;
    case ClkGateCan1:
      FM_PCG->CKEN2_f.CANCK1 = 0u;
      break;  
    case ClkGateCan2:
      FM_PCG->CKEN2_f.CANCK2 = 0u;
      break;
    case ClkGateSd:
      FM_PCG->CKEN2_f.SDCCK = 0u;
      break; 
    case ClkGateI2s0:
      FM_PCG->CKEN2_f.I2SCK0 = 0u;
      break;
    case ClkGateI2s1:
      FM_PCG->CKEN2_f.I2SCK1 = 0u;
      break;      
    case ClkGateCrc:
      FM_PCG->CKEN2_f.PCRCCK = 0u;
      break;
    case ClkGateQspi:
      FM_PCG->CKEN2_f.QSPICK = 0u;
      break;
#endif
#if (PDL_MCU_CORE == PDL_FM0P_CORE)
    case ClkGateCec0:
    case ClkGateCec1:
      FM_PCG->CKEN2_f.CECCK = 0u;
      break;  
#endif      
#if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE)
    case ClkGateIcc0:
      FM_PCG->CKEN2_f.ICCCK0 = 0u;
      break;
    case ClkGateIcc1:
      FM_PCG->CKEN2_f.ICCCK1 = 0u;
      break;
    case ClkGateI2sl0:
      FM_PCG->CKEN2_f.IISCCK0 = 0u;
      break;
    case ClkGateI2sl1:
      FM_PCG->CKEN2_f.IISCCK1 = 0u;
      break;      
#endif        
    default:
      return ErrorInvalidParameter;
  }
  
  return Ok;
} // Clk_PeripheralClockDisable

/**
 ******************************************************************************
 ** \brief Enable the clock gate of all peripherals
 **
 ** This function enable the clock of all peripherals.
 **
 ** \retval Ok                     All peripheral clock enabled
 ******************************************************************************/
en_result_t Clk_PeripheralClockEnableAll(void) 
{
    FM_PCG->CKEN0 = 0xFFFFFFFFu;
    FM_PCG->CKEN1 = 0xFFFFFFFFu;
    FM_PCG->CKEN2 = 0xFFFFFFFFu;
    
    return Ok;
}

/**
 ******************************************************************************
 ** \brief Disable the clock gate of all peripherals
 **
 ** This function disables the clock of all peripherals.
 **
 ** \retval Ok                     All peripheral clock disabled
 ******************************************************************************/
en_result_t Clk_PeripheralClockDisableAll(void) 
{
    FM_PCG->CKEN0 = 0u;
    FM_PCG->CKEN1 = 0u;
    FM_PCG->CKEN2 = 0u;
    
    return Ok;
}

/**
 ******************************************************************************
 ** \brief Set reset bit a peripheral
 **
 ** This function sets the corresponding bit in the MRSTn register to set
 ** a peripheral in reset state.
 **
 ** \param  enPeripheral           Enumerator of a peripheral, see
 **                                #en_clk_reset_peripheral_t for details
 **
 ** \retval Ok                     Peripheral clock enabled
 ** \retval ErrorInvalidParameter  Peripheral enumerator does not exist
 ******************************************************************************/
en_result_t Clk_PeripheralSetReset(en_clk_reset_peripheral_t enPeripheral)
{
  switch (enPeripheral)
  {
    case ClkResetDma:
      FM_PCG->MRST0_f.DMARST = 1u;
      break;
    case ClkResetAdc0:
      FM_PCG->MRST0_f.ADCRST0 = 1u;
      break;
    case ClkResetAdc1:
      FM_PCG->MRST0_f.ADCRST1 = 1u;
      break;
    case ClkResetAdc2:
      FM_PCG->MRST0_f.ADCRST2 = 1u;
      break;
    case ClkResetAdc3:
      FM_PCG->MRST0_f.ADCRST3 = 1u;
      break;
    case ClkResetMfs0:
      FM_PCG->MRST0_f.MFSRST0 = 1u;
      break;
    case ClkResetMfs1:
      FM_PCG->MRST0_f.MFSRST1 = 1u;
      break;
    case ClkResetMfs2:
      FM_PCG->MRST0_f.MFSRST2 = 1u;
      break;
    case ClkResetMfs3:
      FM_PCG->MRST0_f.MFSRST3 = 1u;
      break;
    case ClkResetMfs4:
      FM_PCG->MRST0_f.MFSRST4 = 1u;
      break;
    case ClkResetMfs5:
      FM_PCG->MRST0_f.MFSRST5 = 1u;
      break;
    case ClkResetMfs6:
      FM_PCG->MRST0_f.MFSRST6 = 1u;
      break;
    case ClkResetMfs7:
      FM_PCG->MRST0_f.MFSRST7 = 1u;
      break;
    case ClkResetMfs8:
      FM_PCG->MRST0_f.MFSRST8 = 1u;
      break;
    case ClkResetMfs9:
      FM_PCG->MRST0_f.MFSRST9 = 1u;
      break;
    case ClkResetMfs10:
      FM_PCG->MRST0_f.MFSRST10 = 1u;
      break;
    case ClkResetMfs11:
      FM_PCG->MRST0_f.MFSRST11 = 1u;
      break;
    case ClkResetMfs12:
      FM_PCG->MRST0_f.MFSRST12 = 1u;
      break;
    case ClkResetMfs13:
      FM_PCG->MRST0_f.MFSRST13 = 1u;
      break;
    case ClkResetMfs14:
      FM_PCG->MRST0_f.MFSRST14 = 1u;
      break;
    case ClkResetMfs15:
      FM_PCG->MRST0_f.MFSRST15 = 1u;
      break;
    case ClkResetQprc0:
      FM_PCG->MRST1_f.QDURST0 = 1u;
      break;
    case ClkResetQprc1:
      FM_PCG->MRST1_f.QDURST1 = 1u;
      break;
    case ClkResetQprc2:
      FM_PCG->MRST1_f.QDURST2 = 1u;
      break;
    case ClkResetQprc3:
      FM_PCG->MRST1_f.QDURST3 = 1u;
      break;
    case ClkResetMft0:
      FM_PCG->MRST1_f.MFTRST0 = 1u;
      break;
    case ClkResetMft1:
      FM_PCG->MRST1_f.MFTRST1 = 1u;
      break;
    case ClkResetMft2:
      FM_PCG->MRST1_f.MFTRST2 = 1u;
      break;
    case ClkResetMft3:
      FM_PCG->MRST1_f.MFTRST3 = 1u;
      break;
    case ClkResetBt0:
      FM_PCG->MRST1_f.BTMRST0 = 1u;
      break;
    case ClkResetBt4:
      FM_PCG->MRST1_f.BTMRST1 = 1u;
      break;
    case ClkResetBt8:
      FM_PCG->MRST1_f.BTMRST2 = 1u;
      break;
    case ClkResetBt12:
      FM_PCG->MRST1_f.BTMRST3 = 1u;
      break;
#if (PDL_MCU_CORE == PDL_FM4_CORE) 
    case ClkResetExtif:
      FM_PCG->MRST0_f.EXBRST = 1u;  
      break;    
    case ClkResetUsb0:
      FM_PCG->MRST2_f.USBRST0 = 1u;
      break;
    case ClkResetUsb1:
      FM_PCG->MRST2_f.USBRST1 = 1u;
      break;    
    case ClkResetCan2:
      FM_PCG->MRST2_f.CANRST2 = 1u;
      break;
    case ClkResetSd:
      FM_PCG->MRST2_f.SDCRST = 1u;
      break;
    case ClkResetI2s0:
      FM_PCG->MRST2_f.I2SRST0 = 1u;
      break;
    case ClkResetI2s1:
      FM_PCG->MRST2_f.I2SRST1 = 1u;
      break;
    case ClkResetCrc:
      FM_PCG->MRST2_f.PCRCRST = 1u;
      break;
    case ClkResetQspi:
      FM_PCG->MRST2_f.QSPIRST = 1u;
      break;
    case ClkResetCan0:
      FM_PCG->MRST2_f.CANRST0 = 1u;
      break;
    case ClkResetCan1:
      FM_PCG->MRST2_f.CANRST1 = 1u;
      break;      
#endif      
#if (PDL_MCU_CORE == PDL_FM0P_CORE)      
    case ClkResetCec0:
    case ClkResetCec1:
      FM_PCG->MRST2_f.CECRST = 1u;
      break;
#endif      
#if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE)         
    case ClkResetIcc0:
      FM_PCG->MRST2_f.ICCRST0 = 1u;
      break;
    case ClkResetIcc1:
      FM_PCG->MRST2_f.ICCRST1 = 1u;
      break;
    case ClkResetI2sl0:
      FM_PCG->MRST2_f.IISCRST0 = 1u;
      break;
    case ClkResetI2sl1:
      FM_PCG->MRST2_f.IISCRST1 = 1u;
      break;      
#endif      
    default:
      return ErrorInvalidParameter;
  }
  
  return Ok;
} // Clk_PeripheralSetReset

/**
 ******************************************************************************
 ** \brief Clear reset bit a peripheral
 **
 ** This function clears the corresponding bit in the MRSTn register to release
 ** a peripheral from reset state.
 **
 ** \param  enPeripheral           Enumerator of a peripheral, see
 **                                #en_clk_reset_peripheral_t for details
 **
 ** \retval Ok                     Peripheral clock enabled
 ** \retval ErrorInvalidParameter  Peripheral enumerator does not exist
 ******************************************************************************/
en_result_t Clk_PeripheralClearReset(en_clk_reset_peripheral_t enPeripheral)
{
  switch (enPeripheral)
  {   
    case ClkResetDma:
      FM_PCG->MRST0_f.DMARST = 0u;
      break;
    case ClkResetAdc0:
      FM_PCG->MRST0_f.ADCRST0 = 0u;
      break;
    case ClkResetAdc1:
      FM_PCG->MRST0_f.ADCRST1 = 0u;
      break;
    case ClkResetAdc2:
      FM_PCG->MRST0_f.ADCRST2 = 0u;
      break;
    case ClkResetAdc3:
      FM_PCG->MRST0_f.ADCRST3 = 0u;
      break;
    case ClkResetMfs0:
      FM_PCG->MRST0_f.MFSRST0 = 0u;
      break;
    case ClkResetMfs1:
      FM_PCG->MRST0_f.MFSRST1 = 0u;
      break;
    case ClkResetMfs2:
      FM_PCG->MRST0_f.MFSRST2 = 0u;
      break;
    case ClkResetMfs3:
      FM_PCG->MRST0_f.MFSRST3 = 0u;
      break;
    case ClkResetMfs4:
      FM_PCG->MRST0_f.MFSRST4 = 0u;
      break;
    case ClkResetMfs5:
      FM_PCG->MRST0_f.MFSRST5 = 0u;
      break;
    case ClkResetMfs6:
      FM_PCG->MRST0_f.MFSRST6 = 0u;
      break;
    case ClkResetMfs7:
      FM_PCG->MRST0_f.MFSRST7 = 0u;
      break;
    case ClkResetMfs8:
      FM_PCG->MRST0_f.MFSRST8 = 0u;
      break;
    case ClkResetMfs9:
      FM_PCG->MRST0_f.MFSRST9 = 0u;
      break;
    case ClkResetMfs10:
      FM_PCG->MRST0_f.MFSRST10 = 0u;
      break;
    case ClkResetMfs11:
      FM_PCG->MRST0_f.MFSRST11 = 0u;
      break;
    case ClkResetMfs12:
      FM_PCG->MRST0_f.MFSRST12 = 0u;
      break;
    case ClkResetMfs13:
      FM_PCG->MRST0_f.MFSRST13 = 0u;
      break;
    case ClkResetMfs14:
      FM_PCG->MRST0_f.MFSRST14 = 0u;
      break;
    case ClkResetMfs15:
      FM_PCG->MRST0_f.MFSRST15 = 0u;
      break;
    case ClkResetQprc0:
      FM_PCG->MRST1_f.QDURST0 = 0u;
      break;
    case ClkResetQprc1:
      FM_PCG->MRST1_f.QDURST1 = 0u;
      break;
    case ClkResetQprc2:
      FM_PCG->MRST1_f.QDURST2 = 0u;
      break;
    case ClkResetQprc3:
      FM_PCG->MRST1_f.QDURST3 = 0u;
      break;
    case ClkResetMft0:
      FM_PCG->MRST1_f.MFTRST0 = 0u;
      break;
    case ClkResetMft1:
      FM_PCG->MRST1_f.MFTRST1 = 0u;
      break;
    case ClkResetMft2:
      FM_PCG->MRST1_f.MFTRST2 = 0u;
      break;
    case ClkResetMft3:
      FM_PCG->MRST1_f.MFTRST3 = 0u;
      break;
    case ClkResetBt0:
      FM_PCG->MRST1_f.BTMRST0 = 0u;
      break;
    case ClkResetBt4:
      FM_PCG->MRST1_f.BTMRST1 = 0u;
      break;
    case ClkResetBt8:
      FM_PCG->MRST1_f.BTMRST2 = 0u;
      break;
    case ClkResetBt12:
      FM_PCG->MRST1_f.BTMRST3 = 0u;
      break;
#if (PDL_MCU_CORE == PDL_FM4_CORE)
    case ClkResetExtif:
      FM_PCG->MRST0_f.EXBRST = 0u;
      break;  
    case ClkResetUsb0:
      FM_PCG->MRST2_f.USBRST0 = 0u;
      break;
    case ClkResetUsb1:
      FM_PCG->MRST2_f.USBRST1 = 0u;
      break;     
    case ClkResetCan0:
      FM_PCG->MRST2_f.CANRST0 = 0u;
      break;
    case ClkResetCan1:
      FM_PCG->MRST2_f.CANRST1 = 0u;
      break;
    case ClkResetCan2:
      FM_PCG->MRST2_f.CANRST2 = 0u;
      break;
    case ClkResetSd:
      FM_PCG->MRST2_f.SDCRST = 0u;
      break;
    case ClkResetI2s0:
      FM_PCG->MRST2_f.I2SRST0 = 0u;
      break;
    case ClkResetI2s1:
      FM_PCG->MRST2_f.I2SRST1 = 0u;
      break;
    case ClkResetCrc:
      FM_PCG->MRST2_f.PCRCRST = 0u;
      break;
    case ClkResetQspi:
      FM_PCG->MRST2_f.QSPIRST = 0u;
      break;
#endif
#if (PDL_MCU_CORE == PDL_FM0P_CORE)
    case ClkResetCec0:
    case ClkResetCec1:
      FM_PCG->MRST2_f.CECRST = 0u;
      break;  
#endif      
#if (PDL_MCU_CORE == PDL_FM4_CORE) || (PDL_MCU_CORE == PDL_FM0P_CORE)
    case ClkResetIcc0:
      FM_PCG->MRST2_f.ICCRST0 = 0u;
      break;
    case ClkResetIcc1:
      FM_PCG->MRST2_f.ICCRST1 = 0u;
      break;
    case ClkResetI2sl0:
      FM_PCG->MRST2_f.IISCRST0 = 0u;
      break;
    case ClkResetI2sl1:
      FM_PCG->MRST2_f.IISCRST1 = 0u;
      break;        
#endif      
    default:
      return ErrorInvalidParameter;
  }
  
  return Ok;
} // Clk_PeripheralClearReset

#endif

//@} // ClkGroup

#endif // #if (defined(PDL_PERIPHERAL_ENABLE_CLK))

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
