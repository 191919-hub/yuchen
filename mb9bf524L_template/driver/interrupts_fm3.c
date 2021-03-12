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
/** \file interrupts.c
 **
 ** PDL Interrupt Handler
 **
 ** A detailed description is available at 
 ** @link PdlInterrupts PDL Interrupt handling description @endlink
 **
 ** History:
 **   - 2014-09-18  0.0.1  EZh   First version for universal PDL    
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "interrupts_fm3.h"

#if (PDL_MCU_CORE == PDL_FM3_CORE)
/******************************************************************************/
/*********************************** NMI **************************************/
/******************************************************************************/
#if (PDL_INTERRUPT_ENABLE_NMI == PDL_ON) || (PDL_INTERRUPT_ENABLE_HWWDG == PDL_ON)
void NMI_Handler(void)
{
#if (PDL_INTERRUPT_ENABLE_NMI == PDL_ON)
    if(1u == bFM_INTREQ_EXC02MON_NMI)
    {
        Exint_Nmi_IrqHandler();
    }
#endif
#if (PDL_INTERRUPT_ENABLE_HWWDG == PDL_ON)
    if (1u == bFM_INTREQ_EXC02MON_HWINT)
    {
        HwwdgIrqHandler();
    }
#endif
}
#endif

/******************************************************************************/
/******************************* SW watchdog **********************************/
/******************************************************************************/
#if (PDL_INTERRUPT_ENABLE_SWWDG == PDL_ON)
void SWDT_IRQHandler(void)
{
    SwwdgIrqHandler();
}
#endif

/******************************************************************************/
/*********************************** ADC **************************************/
/******************************************************************************/
#if (PDL_INTERRUPT_ENABLE_ADC0 == PDL_ON) 
void ADC0_IRQHandler(void)
{
    AdcIrqHandler((volatile stc_adcn_t*)&ADC0, &(m_astcAdcInstanceDataLut[AdcInstanceIndexAdc0].stcInternData));
}
#endif

#if (PDL_INTERRUPT_ENABLE_ADC1 == PDL_ON) 
void ADC1_IRQHandler(void)
{
    AdcIrqHandler((volatile stc_adcn_t*)&ADC1, &(m_astcAdcInstanceDataLut[AdcInstanceIndexAdc1].stcInternData));
}
#endif

#if (PDL_INTERRUPT_ENABLE_LCD == PDL_ON) || (PDL_INTERRUPT_ENABLE_ADC2 == PDL_ON) 
void ADC2_LCD_IRQHandler(void)
{
#if (PDL_INTERRUPT_ENABLE_LCD == PDL_ON)
    Lcd_IrqHandler();
#endif
#if (PDL_INTERRUPT_ENABLE_ADC2 == PDL_ON)   
    AdcIrqHandler((volatile stc_adcn_t*)&ADC2, &(m_astcAdcInstanceDataLut[AdcInstanceIndexAdc2].stcInternData));
#endif    
}
#endif

/******************************************************************************/
/*********************************** BT ***************************************/
/******************************************************************************/
#if (PDL_INTERRUPT_ENABLE_BT0 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_BT1 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_BT2 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_BT3 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_BT4 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_BT5 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_BT6 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_BT7 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_BT8 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_BT9 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_BT10 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_BT11 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_BT12 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_BT13 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_BT14 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_BT15 == PDL_ON) 
/**
 ******************************************************************************
 ** \brief BT ch.0~ch.7 IRQ handler
 ******************************************************************************/
void BT0_7_IRQHandler(void)
{
    uint32_t u32IrqMon = FM_INTREQ->PDL_IRQMON_BT0_7;
  
#if (PDL_INTERRUPT_ENABLE_BT0 == PDL_ON) 
    if(0u != (u32IrqMon & 0x00000003ul))
    {
        Bt_IrqHandler((volatile stc_btn_t*)&BT0, &m_astcBtInstanceDataLut[BtInstanceIndexBt0].stcInternData);
    }
#endif     
#if (PDL_INTERRUPT_ENABLE_BT1 == PDL_ON) 
    if(0u != (u32IrqMon & 0x0000000Cul))
    {
        Bt_IrqHandler((volatile stc_btn_t*)&BT1, &m_astcBtInstanceDataLut[BtInstanceIndexBt1].stcInternData);
    }
#endif     
#if (PDL_INTERRUPT_ENABLE_BT2 == PDL_ON) 
    if(0u != (u32IrqMon & 0x00000030ul))
    {
        Bt_IrqHandler((volatile stc_btn_t*)&BT2, &m_astcBtInstanceDataLut[BtInstanceIndexBt2].stcInternData);
    }
#endif 
#if (PDL_INTERRUPT_ENABLE_BT3 == PDL_ON) 
    if(0u != (u32IrqMon & 0x000000C0ul))
    {
        Bt_IrqHandler((volatile stc_btn_t*)&BT3, &m_astcBtInstanceDataLut[BtInstanceIndexBt3].stcInternData);
    }
#endif    
#if (PDL_INTERRUPT_ENABLE_BT4 == PDL_ON) 
    if(0u != (u32IrqMon & 0x00000300ul))
    {
        Bt_IrqHandler((volatile stc_btn_t*)&BT4, &m_astcBtInstanceDataLut[BtInstanceIndexBt4].stcInternData);
    }
#endif
#if (PDL_INTERRUPT_ENABLE_BT5 == PDL_ON) 
    if(0u != (u32IrqMon & 0x00000C00ul))
    {
        Bt_IrqHandler((volatile stc_btn_t*)&BT5, &m_astcBtInstanceDataLut[BtInstanceIndexBt5].stcInternData);
    }
#endif  
#if (PDL_INTERRUPT_ENABLE_BT6 == PDL_ON) 
    if(0u != (u32IrqMon & 0x00003000ul))
    {
        Bt_IrqHandler((volatile stc_btn_t*)&BT6, &m_astcBtInstanceDataLut[BtInstanceIndexBt6].stcInternData);
    }
#endif
#if (PDL_INTERRUPT_ENABLE_BT7 == PDL_ON) 
    if(0u != (u32IrqMon & 0x0000C000ul))
    {
        Bt_IrqHandler((volatile stc_btn_t*)&BT7, &m_astcBtInstanceDataLut[BtInstanceIndexBt7].stcInternData);
    }
#endif   
}
#endif

#if (PDL_INTERRUPT_ENABLE_BT8  == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_BT9  == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_BT10 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_BT11 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_BT12 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_BT13 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_BT14 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_BT15 == PDL_ON)
/**
 ******************************************************************************
 ** \brief BT ch.8~ch.15 IRQ handler
 ******************************************************************************/
void BT8_15_IRQHandler(void)
{
    uint32_t u32IrqMon = FM_INTREQ->PDL_IRQMON_BT8_15;
  
#if (PDL_INTERRUPT_ENABLE_BT8 == PDL_ON) 
    if(0u != (u32IrqMon & 0x00000003ul))
    {
        Bt_IrqHandler((volatile stc_btn_t*)&BT8, &m_astcBtInstanceDataLut[BtInstanceIndexBt8].stcInternData);
    }
#endif     
#if (PDL_INTERRUPT_ENABLE_BT9 == PDL_ON) 
    if(0u != (u32IrqMon & 0x0000000Cul))
    {
        Bt_IrqHandler((volatile stc_btn_t*)&BT9, &m_astcBtInstanceDataLut[BtInstanceIndexBt9].stcInternData);
    }
#endif     
#if (PDL_INTERRUPT_ENABLE_BT10 == PDL_ON) 
    if(0u != (u32IrqMon & 0x00000030ul))
    {
        Bt_IrqHandler((volatile stc_btn_t*)&BT10, &m_astcBtInstanceDataLut[BtInstanceIndexBt10].stcInternData);
    }
#endif 
#if (PDL_INTERRUPT_ENABLE_BT11 == PDL_ON) 
    if(0u != (u32IrqMon & 0x000000C0ul))
    {
        Bt_IrqHandler((volatile stc_btn_t*)&BT11, &m_astcBtInstanceDataLut[BtInstanceIndexBt11].stcInternData);
    }
#endif    
#if (PDL_INTERRUPT_ENABLE_BT12 == PDL_ON) 
    if(0u != (u32IrqMon & 0x00000300ul))
    {
        Bt_IrqHandler((volatile stc_btn_t*)&BT12, &m_astcBtInstanceDataLut[BtInstanceIndexBt12].stcInternData);
    }
#endif
#if (PDL_INTERRUPT_ENABLE_BT13 == PDL_ON) 
    if(0u != (u32IrqMon & 0x00000C00ul))
    {
        Bt_IrqHandler((volatile stc_btn_t*)&BT13, &m_astcBtInstanceDataLut[BtInstanceIndexBt13].stcInternData);
    }
#endif  
#if (PDL_INTERRUPT_ENABLE_BT14 == PDL_ON) 
    if(0u != (u32IrqMon & 0x00003000ul))
    {
        Bt_IrqHandler((volatile stc_btn_t*)&BT14, &m_astcBtInstanceDataLut[BtInstanceIndexBt14].stcInternData);
    }
#endif
#if (PDL_INTERRUPT_ENABLE_BT15 == PDL_ON) 
    if(0u != (u32IrqMon & 0x0000C000ul))
    {
        Bt_IrqHandler((volatile stc_btn_t*)&BT15, &m_astcBtInstanceDataLut[BtInstanceIndexBt15].stcInternData);
    }
#endif   
}
      
#endif

/******************************************************************************/
/*********************************** CAN **************************************/
/******************************************************************************/
#if (PDL_INTERRUPT_ENABLE_CAN0 == PDL_ON)
/**
 ******************************************************************************
 ** \brief CAN ch.0 IRQ handler
 ******************************************************************************/
void ETHER0_CAN0_IRQHandler(void)
{
    CanIrqHandler((stc_cann_t*)&CAN0, &(m_astcCanInstanceDataLut[CanInstanceIndexCan0].stcInternData));
}
#endif

#if (PDL_INTERRUPT_ENABLE_CAN1 == PDL_ON)
/**
 ******************************************************************************
 ** \brief CAN ch.1 IRQ handler
 ******************************************************************************/
void ETHER1_CAN1_IRQHandler(void)
{
    CanIrqHandler((stc_cann_t*)&CAN1, &(m_astcCanInstanceDataLut[CanInstanceIndexCan1].stcInternData));
}
#endif

/******************************************************************************/
/*********************************** CLK/WC/RTC *******************************/
/******************************************************************************/
#if (PDL_INTERRUPT_ENABLE_CLK == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_WC0 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_RTC0 == PDL_ON)
void TIM_IRQHandler(void)
{
    uint32_t u32IrqMon = FM_INTREQ->PDL_IRQMON_TIM;
  
#if (PDL_INTERRUPT_ENABLE_RTC0 == PDL_ON)
    if(0u != (u32IrqMon & 0x00000020ul))
    {  
    Rtc_IrqHandler((stc_rtcn_t*)&RTC0, &(m_astcRtcInstanceDataLut[RtcInstanceIndexRtc0].stcInternData));
    }
#endif
#if (PDL_INTERRUPT_ENABLE_WC0 == PDL_ON)  
    if(0u != (u32IrqMon & 0x00000010ul))
    {  
    Wc_IrqHandler(&WC0);
    }
#endif    
#if (PDL_INTERRUPT_ENABLE_CLK == PDL_ON)
    if(0u != (u32IrqMon & 0x00000007ul))
    {  
    Clk_IrqHandler();
    }
#endif    
}
#endif

/******************************************************************************/
/*********************************** CSV **************************************/
/******************************************************************************/
#if (PDL_INTERRUPT_ENABLE_CSV == PDL_ON)
void CSV_IRQHandler(void)
{
    Csv_IrqHandler();
}
#endif

/******************************************************************************/
/*********************************** DMA **************************************/
/******************************************************************************/
#if (PDL_INTERRUPT_ENABLE_DMA0 == PDL_ON)
/**
 ******************************************************************************
 ** \brief DMA ch.0 IRQ handler
 ******************************************************************************/
void DMAC0_IRQHandler(void)
{
    DmaIrqHandler(0u);
}
#endif

#if (PDL_INTERRUPT_ENABLE_DMA1 == PDL_ON)
/**
 ******************************************************************************
 ** \brief DMA ch.1 IRQ handler
 ******************************************************************************/
void DMAC1_IRQHandler(void)
{
    DmaIrqHandler(1u);
}
#endif

#if (PDL_INTERRUPT_ENABLE_DMA2 == PDL_ON)
/**
 ******************************************************************************
 ** \brief DMA ch.2 IRQ handler
 ******************************************************************************/
void DMAC2_IRQHandler(void)
{
    DmaIrqHandler(2u);
}
#endif

#if (PDL_INTERRUPT_ENABLE_DMA3 == PDL_ON)
/**
 ******************************************************************************
 ** \brief DMA ch.3 IRQ handler
 ******************************************************************************/
void DMAC3_IRQHandler(void)
{
    DmaIrqHandler(3u);
}
#endif

#if (PDL_INTERRUPT_ENABLE_DMA4 == PDL_ON)
/**
 ******************************************************************************
 ** \brief DMA ch.4 IRQ handler
 ******************************************************************************/
void DMAC4_IRQHandler(void)
{
    DmaIrqHandler(4u);
}
#endif

#if (PDL_INTERRUPT_ENABLE_DMA5 == PDL_ON)
/**
 ******************************************************************************
 ** \brief DMA ch.5 IRQ handler
 ******************************************************************************/
void DMAC5_IRQHandler(void)
{
    DmaIrqHandler(5u);
}
#endif

#if (PDL_INTERRUPT_ENABLE_DMA6 == PDL_ON)
/**
 ******************************************************************************
 ** \brief DMA ch.6 IRQ handler
 ******************************************************************************/
void DMAC6_IRQHandler(void)
{
    DmaIrqHandler(6u);
}
#endif

#if (PDL_INTERRUPT_ENABLE_DMA7 == PDL_ON)
/**
 ******************************************************************************
 ** \brief DMA ch.7 IRQ handler
 ******************************************************************************/
void DMAC7_IRQHandler(void)
{
    DmaIrqHandler(7u);
}
#endif

/******************************************************************************/
/******************************* Dual Timer/QPRC ******************************/
/******************************************************************************/
#if (PDL_INTERRUPT_ENABLE_DT0 == PDL_ON) || (PDL_INTERRUPT_ENABLE_QPRC0 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_QPRC1 == PDL_ON) || (PDL_INTERRUPT_ENABLE_QPRC2 == PDL_ON)
void DT_QPRC_IRQHandler(void)
{
    uint32_t u32IrqMon = FM_INTREQ->PDL_IRQMON_DT_QPRC;
    
#if (PDL_INTERRUPT_ENABLE_DT0 == PDL_ON)
    if (0u != (u32IrqMon & 0x00000001ul))
    {
        DtIrqHandler(DtChannel0);
    }
    if (0u != (u32IrqMon & 0x00000002ul))
    {
        DtIrqHandler(DtChannel1);
    }
#endif    
#if (PDL_INTERRUPT_ENABLE_QPRC0 == PDL_ON)
    if(0u != (u32IrqMon & 0x000000FCul))
    {
        Qprc_IrqHandler((volatile stc_qprcn_t*)&QPRC0, &m_astcQprcInstanceDataLut[QprcInstanceIndexQprc0].stcInternData);
    }
#endif  
#if (PDL_INTERRUPT_ENABLE_QPRC1 == PDL_ON)
    if(0u != (u32IrqMon & 0x00003F00ul))
    {
        Qprc_IrqHandler((volatile stc_qprcn_t*)&QPRC1, &m_astcQprcInstanceDataLut[QprcInstanceIndexQprc1].stcInternData);
    }
#endif      
#if (PDL_INTERRUPT_ENABLE_QPRC2 == PDL_ON)    
    if(0u != (u32IrqMon & 0x000FC000ul))
    {
        Qprc_IrqHandler((volatile stc_qprcn_t*)&QPRC2, &m_astcQprcInstanceDataLut[QprcInstanceIndexQprc2].stcInternData);
    }
#endif    
}
#endif

/******************************************************************************/
/*********************************** LVD **************************************/
/******************************************************************************/
#if (PDL_INTERRUPT_ENABLE_LVD == PDL_ON)
/**
 ******************************************************************************
 ** \brief LVD IRQ handler
 ******************************************************************************/
void LVD_IRQHandler(void)
{
    Lvd_IrqHandler();
}
#endif // #if (PDL_PERIPHERAL_ENABLE_LVD == PDL_ON)

/******************************************************************************/
/********************************* EXINT **************************************/
/******************************************************************************/
#if (PDL_INTERRUPT_ENABLE_EXINT0 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT1 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT2 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT3 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT4 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT5 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT6 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT7 == PDL_ON)   
/**
 ******************************************************************************
 ** \brief EXINT ch.0 ~ ch.7 IRQ handler
 ******************************************************************************/
void INT0_7_IRQHandler(void)
{ 
    uint32_t u32IrqMon = FM_INTREQ->PDL_IRQMON_EXINT0_7;
  
#if(PDL_INTERRUPT_ENABLE_EXINT0 == PDL_ON)
    if(0u != (u32IrqMon & 0x00000001u))
    {
        Exint_IrqHandler(0u);
    }
#endif  
#if(PDL_INTERRUPT_ENABLE_EXINT1 == PDL_ON)
    if(0u != (u32IrqMon & 0x00000002u))
    {
        Exint_IrqHandler(1u);
    }
#endif 
#if(PDL_INTERRUPT_ENABLE_EXINT2 == PDL_ON)
    if(0u != (u32IrqMon & 0x00000004u))
    {
        Exint_IrqHandler(2u);
    }
#endif 
#if(PDL_INTERRUPT_ENABLE_EXINT3 == PDL_ON)
    if(0u != (u32IrqMon & 0x00000008u))
    {
        Exint_IrqHandler(3u);
    }
#endif 
#if(PDL_INTERRUPT_ENABLE_EXINT4 == PDL_ON)
    if(0u != (u32IrqMon & 0x00000010u))
    {
        Exint_IrqHandler(4u);
    }
#endif 
#if(PDL_INTERRUPT_ENABLE_EXINT5 == PDL_ON)
    if(0u != (u32IrqMon & 0x00000020u))
    {
        Exint_IrqHandler(5u);
    }
#endif 
#if(PDL_INTERRUPT_ENABLE_EXINT6 == PDL_ON)
    if(0u != (u32IrqMon & 0x00000040u))
    {
        Exint_IrqHandler(6u);
    }
#endif 
#if(PDL_INTERRUPT_ENABLE_EXINT7 == PDL_ON)
    if(0u != (u32IrqMon & 0x00000080u))
    {
        Exint_IrqHandler(7u);
    }
#endif     
    
}
#endif

#if (PDL_INTERRUPT_ENABLE_EXINT8 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT9 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT10 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT11 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT12 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT13 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT14 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT15 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT16 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT17 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT18 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT19 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT20 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT21 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT22 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT23 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT24 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT25 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT26 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT27 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT28 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT29 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT30 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_EXINT31 == PDL_ON)     
/**
 ******************************************************************************
 ** \brief EXINT ch.8 ~ ch.31 IRQ handler
 ******************************************************************************/
void INT8_31_IRQHandler(void)
{ 
    uint32_t u32IrqMon = FM_INTREQ->PDL_IRQMON_EXINT8_31;
    
#if(PDL_INTERRUPT_ENABLE_EXINT8 == PDL_ON)
    if(0u != (u32IrqMon & 0x00000001u))
    {
        Exint_IrqHandler(8u);
    }
#endif  
#if(PDL_INTERRUPT_ENABLE_EXINT9 == PDL_ON)
    if(0u != (u32IrqMon & 0x00000002u))
    {
        Exint_IrqHandler(9u);
    }
#endif 
#if(PDL_INTERRUPT_ENABLE_EXINT10 == PDL_ON)
    if(0u != (u32IrqMon & 0x00000004u))
    {
        Exint_IrqHandler(10u);
    }
#endif 
#if(PDL_INTERRUPT_ENABLE_EXINT11 == PDL_ON)
    if(0u != (u32IrqMon & 0x00000008u))
    {
        Exint_IrqHandler(11u);
    }
#endif 
#if(PDL_INTERRUPT_ENABLE_EXINT12 == PDL_ON)
    if(0u != (u32IrqMon & 0x00000010u))
    {
        Exint_IrqHandler(12u);
    }
#endif 
#if(PDL_INTERRUPT_ENABLE_EXINT13 == PDL_ON)
    if(0u != (u32IrqMon & 0x00000020u))
    {
        Exint_IrqHandler(13u);
    }
#endif 
#if(PDL_INTERRUPT_ENABLE_EXINT14 == PDL_ON)
    if(0u != (u32IrqMon & 0x00000040u))
    {
        Exint_IrqHandler(14u);
    }
#endif 
#if(PDL_INTERRUPT_ENABLE_EXINT15 == PDL_ON)
    if(0u != (u32IrqMon & 0x00000080u))
    {
        Exint_IrqHandler(15u);
    }
#endif     
#if(PDL_INTERRUPT_ENABLE_EXINT16 == PDL_ON)
    if(0u != (u32IrqMon & 0x00000100u))
    {
        Exint_IrqHandler(16u);
    }
#endif  
#if(PDL_INTERRUPT_ENABLE_EXINT17 == PDL_ON)
    if(0u != (u32IrqMon & 0x00000200u))
    {
        Exint_IrqHandler(17u);
    }
#endif 
#if(PDL_INTERRUPT_ENABLE_EXINT18 == PDL_ON)
    if(0u != (u32IrqMon & 0x00000400u))
    {
        Exint_IrqHandler(18u);
    }
#endif 
#if(PDL_INTERRUPT_ENABLE_EXINT19 == PDL_ON)
    if(0u != (u32IrqMon & 0x00000800u))
    {
        Exint_IrqHandler(19u);
    }
#endif 
#if(PDL_INTERRUPT_ENABLE_EXINT20 == PDL_ON)
    if(0u != (u32IrqMon & 0x00001000u))
    {
        Exint_IrqHandler(20u);
    }
#endif 
#if(PDL_INTERRUPT_ENABLE_EXINT21 == PDL_ON)
    if(0u != (u32IrqMon & 0x00002000u))
    {
        Exint_IrqHandler(21u);
    }
#endif 
#if(PDL_INTERRUPT_ENABLE_EXINT22 == PDL_ON)
    if(0u != (u32IrqMon & 0x00004000u))
    {
        Exint_IrqHandler(22u);
    }
#endif 
#if(PDL_INTERRUPT_ENABLE_EXINT23 == PDL_ON)
    if(0u != (u32IrqMon & 0x00008000u))
    {
        Exint_IrqHandler(23u);
    }
#endif
#if(PDL_INTERRUPT_ENABLE_EXINT24 == PDL_ON)
    if(0u != (u32IrqMon & 0x00010000u))
    {
        Exint_IrqHandler(24u);
    }
#endif  
#if(PDL_INTERRUPT_ENABLE_EXINT25 == PDL_ON)
    if(0u != (u32IrqMon & 0x00020000u))
    {
        Exint_IrqHandler(25u);
    }
#endif 
#if(PDL_INTERRUPT_ENABLE_EXINT26 == PDL_ON)
    if(0u != (u32IrqMon & 0x00040000u))
    {
        Exint_IrqHandler(26u);
    }
#endif 
#if(PDL_INTERRUPT_ENABLE_EXINT27 == PDL_ON)
    if(0u != (u32IrqMon & 0x00080000u))
    {
        Exint_IrqHandler(27u);
    }
#endif 
#if(PDL_INTERRUPT_ENABLE_EXINT28 == PDL_ON)
    if(0u != (u32IrqMon & 0x00100000u))
    {
        Exint_IrqHandler(28u);
    }
#endif 
#if(PDL_INTERRUPT_ENABLE_EXINT29 == PDL_ON)
    if(0u != (u32IrqMon & 0x00200000u))
    {
        Exint_IrqHandler(29u);
    }
#endif 
#if(PDL_INTERRUPT_ENABLE_EXINT30 == PDL_ON)
    if(0u != (u32IrqMon & 0x00400000u))
    {
        Exint_IrqHandler(30u);
    }
#endif 
#if(PDL_INTERRUPT_ENABLE_EXINT31 == PDL_ON)
    if(0u != (u32IrqMon & 0x00800000u))
    {
        Exint_IrqHandler(31u);
    }
#endif       
}
#endif

/******************************************************************************/
/*********************************** Flash*************************************/
/******************************************************************************/
#if (PDL_INTERRUPT_ENABLE_FLASH == PDL_ON)
void FLASH_IRQHandler(void)
{
    //WFlash_IrqHandler();
}
#endif

/******************************************************************************/
/*********************************** MFT **************************************/
/******************************************************************************/
#if (PDL_INTERRUPT_ENABLE_MFT0_FRT == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_MFT1_FRT == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_MFT2_FRT == PDL_ON)  
/**
 ******************************************************************************
 ** \brief MFT0's FRT peak match IRQ handler
 ******************************************************************************/
void MFT_FRT_IRQHandler(void)
{ 
    uint32_t u32IrqMon = FM_INTREQ->PDL_IRQMON_MFT_FRT;
    
#if (PDL_INTERRUPT_ENABLE_MFT0_FRT == PDL_ON)    
    if(0u != (u32IrqMon & 0x0000003Fu))
    {
        Mft_Frt_IrqHandler((volatile stc_mftn_frt_t*)&MFT0_FRT, &m_astcMftFrtInstanceDataLut[FrtInstanceIndexFrt0].stcInternData);
    }
#endif   
#if (PDL_INTERRUPT_ENABLE_MFT1_FRT == PDL_ON)    
    if(0u != (u32IrqMon & 0x00000FC0u))
    {
        Mft_Frt_IrqHandler((volatile stc_mftn_frt_t*)&MFT1_FRT, &m_astcMftFrtInstanceDataLut[FrtInstanceIndexFrt1].stcInternData);
    }
#endif     
#if (PDL_INTERRUPT_ENABLE_MFT2_FRT == PDL_ON)    
    if(0u != (u32IrqMon & 0x0003F000u))
    {
        Mft_Frt_IrqHandler((volatile stc_mftn_frt_t*)&MFT2_FRT, &m_astcMftFrtInstanceDataLut[FrtInstanceIndexFrt2].stcInternData);
    }
#endif      
}
#endif

#if (PDL_INTERRUPT_ENABLE_MFT0_OCU == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_MFT1_OCU == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_MFT2_OCU == PDL_ON)  
/**
 ******************************************************************************
 ** \brief MFT0's OCU IRQ handler
 ******************************************************************************/
void MFT_OPC_IRQHandler(void)
{
    uint32_t u32IrqMon = FM_INTREQ->PDL_IRQMON_MFT_OPC;
    
#if (PDL_INTERRUPT_ENABLE_MFT0_OCU == PDL_ON)    
    if(0u != (u32IrqMon & 0x0000003Fu))
    {
        Mft_Ocu_IrqHandler((volatile stc_mftn_ocu_t*)&MFT0_OCU, &m_astcMftOcuInstanceDataLut[OcuInstanceIndexOcu0].stcInternData);
    }
#endif  
#if (PDL_INTERRUPT_ENABLE_MFT1_OCU == PDL_ON)    
    if(0u != (u32IrqMon & 0x00000FC0u))
    {
        Mft_Ocu_IrqHandler((volatile stc_mftn_ocu_t*)&MFT1_OCU, &m_astcMftOcuInstanceDataLut[OcuInstanceIndexOcu1].stcInternData);
    }
#endif     
#if (PDL_INTERRUPT_ENABLE_MFT2_OCU == PDL_ON)    
    if(0u != (u32IrqMon & 0x0003F000u))
    {
        Mft_Ocu_IrqHandler((volatile stc_mftn_ocu_t*)&MFT2_OCU, &m_astcMftOcuInstanceDataLut[OcuInstanceIndexOcu2].stcInternData);
    }
#endif   
}
#endif

#if (PDL_INTERRUPT_ENABLE_MFT0_WFG == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_MFT1_WFG == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_MFT2_WFG == PDL_ON)  
/**
 ******************************************************************************
 ** \brief MFT0's WFG IRQ handler
 ******************************************************************************/
void MFT_WFG_IRQHandler(void)
{
    uint32_t u32IrqMon = FM_INTREQ->PDL_IRQMON_MFT_WFG;
      
#if (PDL_INTERRUPT_ENABLE_MFT0_WFG == PDL_ON)
    if(0u != (u32IrqMon & 0x0000000Fu))
    {
        Mft_Wfg_IrqHandler((volatile stc_mftn_wfg_t*)&MFT0_WFG, &m_astcMftWfgInstanceDataLut[WfgInstanceIndexWfg0].stcInternData);
    }
#endif
#if (PDL_INTERRUPT_ENABLE_MFT1_WFG == PDL_ON)
    if(0u != (u32IrqMon & 0x000000F0u))
    {
        Mft_Wfg_IrqHandler((volatile stc_mftn_wfg_t*)&MFT1_WFG, &m_astcMftWfgInstanceDataLut[WfgInstanceIndexWfg1].stcInternData);
    }
#endif    
#if (PDL_INTERRUPT_ENABLE_MFT2_WFG == PDL_ON)
    if(0u != (u32IrqMon & 0x00000F00u))
    {
        Mft_Wfg_IrqHandler((volatile stc_mftn_wfg_t*)&MFT2_WFG, &m_astcMftWfgInstanceDataLut[WfgInstanceIndexWfg2].stcInternData);
    }
#endif     
}
#endif

#if (PDL_INTERRUPT_ENABLE_MFT0_ICU == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_MFT1_ICU == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_MFT2_ICU == PDL_ON)
/**
 ******************************************************************************
 ** \brief MFT0's ICU IRQ handler
 ******************************************************************************/
void MFT_IPC_IRQHandler(void)
{
    uint32_t u32IrqMon = FM_INTREQ->PDL_IRQMON_MFT_ICP;
    
#if (PDL_INTERRUPT_ENABLE_MFT0_ICU == PDL_ON)  
    if(0u != (u32IrqMon & 0x0000000Fu))
    {
        Mft_Icu_IrqHandler((volatile stc_mftn_icu_t*)&MFT0_ICU, &m_astcMftIcuInstanceDataLut[IcuInstanceIndexIcu0].stcInternData);
    }
#endif    
#if (PDL_INTERRUPT_ENABLE_MFT1_ICU == PDL_ON)  
    if(0u != (u32IrqMon & 0x000000F0u))
    {
        Mft_Icu_IrqHandler((volatile stc_mftn_icu_t*)&MFT1_ICU, &m_astcMftIcuInstanceDataLut[IcuInstanceIndexIcu1].stcInternData);
    }
#endif    
#if (PDL_INTERRUPT_ENABLE_MFT2_ICU == PDL_ON)  
    if(0u != (u32IrqMon & 0x00000F00u))
    {
        Mft_Icu_IrqHandler((volatile stc_mftn_icu_t*)&MFT2_ICU, &m_astcMftIcuInstanceDataLut[IcuInstanceIndexIcu2].stcInternData);
    }
#endif      
}
#endif

/******************************************************************************/
/*********************************** PPG **************************************/
/******************************************************************************/
#if (PDL_INTERRUPT_ENABLE_PPG == PDL_ON)
/**
 ******************************************************************************
 ** \brief PPG ch.0,2,4,8,10,12,16,18,20 IRQ handler
 ******************************************************************************
 */
void PPG_IRQHandler(void)
{
    Ppg_IrqHandler();
}
#endif

/******************************************************************************/
/*********************************** MFS **************************************/
/******************************************************************************/

#if (PDL_INTERRUPT_ENABLE_MFS0 == PDL_ON) || (PDL_INTERRUPT_ENABLE_MFS8 == PDL_ON)
/**
 ******************************************************************************
 ** \brief MFS ch.0/8 RX IRQ handler
 ******************************************************************************
 */
void MFS0_8_RX_IRQHandler(void) 
{
    uint32_t u32IrqMon = FM_INTREQ->PDL_IRQMON_MFS0_8_RX;
  
#if (PDL_INTERRUPT_ENABLE_MFS0 == PDL_ON)    
    if(0u != (u32IrqMon & 0x00000001u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs0].stcInternData.enMode)
        {
            case MfsUartMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_UART_MODE == PDL_ON)   
                MfsUartIrqHandlerRx((stc_mfsn_uart_t*)&UART0, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs0].stcInternData));
            #endif
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)   
                MfsCsioIrqHandlerRx((stc_mfsn_csio_t*)&CSIO0, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs0].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)   
                MfsI2cIrqHandlerRx((stc_mfsn_i2c_t*)&I2C0, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs0].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)  
                MfsLinIrqHandlerRx((stc_mfsn_lin_t*)&LIN0, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs0].stcInternData));
            #endif
                break;
            default:
                break;
        
        }
    }
#endif
#if (PDL_INTERRUPT_ENABLE_MFS8 == PDL_ON)        
    if(0u != (u32IrqMon & 0x00000002u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs8].stcInternData.enMode)
        {
            case MfsUartMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_UART_MODE == PDL_ON)    
                MfsUartIrqHandlerRx((stc_mfsn_uart_t*)&UART8, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs8].stcInternData));
            #endif
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)  
                MfsCsioIrqHandlerRx((stc_mfsn_csio_t*)&CSIO8, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs8].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)   
                MfsI2cIrqHandlerRx((stc_mfsn_i2c_t*)&I2C8, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs8].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)    
                MfsLinIrqHandlerRx((stc_mfsn_lin_t*)&LIN8, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs8].stcInternData));
            #endif
                break;
            default:
                break;
        
        }
    }
#endif    
}

/**
 ******************************************************************************
 ** \brief MFS ch.0/8 TX IRQ handler
 ******************************************************************************/
void MFS0_8_TX_IRQHandler(void)
{
    uint32_t u32IrqMon = FM_INTREQ->PDL_IRQMON_MFS0_8_TX;
  
#if (PDL_INTERRUPT_ENABLE_MFS0 == PDL_ON)    
    if(0u != (u32IrqMon & 0x00000001u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs0].stcInternData.enMode)
        {
            case MfsUartMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_UART_MODE == PDL_ON)   
                MfsUartIrqHandlerTx((stc_mfsn_uart_t*)&UART0, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs0].stcInternData));
            #endif
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)   
                MfsCsioIrqHandlerTx((stc_mfsn_csio_t*)&CSIO0, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs0].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)   
                MfsI2cIrqHandlerTx((stc_mfsn_i2c_t*)&I2C0, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs0].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)  
                MfsLinIrqHandlerTx((stc_mfsn_lin_t*)&LIN0, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs0].stcInternData));
            #endif
                break;
            default:
                break;
        
        }
    }
    
    if(0u != (u32IrqMon & 0x00000002u))
    {
       switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs0].stcInternData.enMode)
        {
            case MfsUartMode:
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)   
                MfsCsioIrqHandlerStatus((stc_mfsn_csio_t*)&CSIO0, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs0].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)     
                MfsI2cIrqHandlerStatus((stc_mfsn_i2c_t*)&I2C0, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs0].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)    
                MfsLinIrqHandlerStatus((stc_mfsn_lin_t*)&LIN0, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs0].stcInternData));
            #endif
                break;
            default:
                break;
        
        }
    }
#endif
#if (PDL_INTERRUPT_ENABLE_MFS8 == PDL_ON)        
    if(0u != (u32IrqMon & 0x00000004u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs8].stcInternData.enMode)
        {
            case MfsUartMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_UART_MODE == PDL_ON)     
                MfsUartIrqHandlerTx((stc_mfsn_uart_t*)&UART8, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs8].stcInternData));
            #endif
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)    
                MfsCsioIrqHandlerTx((stc_mfsn_csio_t*)&CSIO8, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs8].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)   
                MfsI2cIrqHandlerTx((stc_mfsn_i2c_t*)&I2C8, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs8].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)  
                MfsLinIrqHandlerTx((stc_mfsn_lin_t*)&LIN8, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs8].stcInternData));
            #endif
                break;
            default:
                break;
        
        }
    }
    
    if(0u != (u32IrqMon & 0x00000008u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs8].stcInternData.enMode)
        {
            case MfsUartMode:
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)     
                MfsCsioIrqHandlerStatus((stc_mfsn_csio_t*)&CSIO8, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs8].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)   
                MfsI2cIrqHandlerStatus((stc_mfsn_i2c_t*)&I2C8, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs8].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)    
                MfsLinIrqHandlerStatus((stc_mfsn_lin_t*)&LIN8, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs8].stcInternData));
            #endif
                break;
            default:
                break;
        
        } 
    }
#endif    
}
#endif // #if (PDL_INTERRUPT_ENABLE_MFS0 == PDL_ON) || (PDL_INTERRUPT_ENABLE_MFS8 == PDL_ON)

#if (PDL_INTERRUPT_ENABLE_MFS1 == PDL_ON) || (PDL_INTERRUPT_ENABLE_MFS9 == PDL_ON)
/**
 ******************************************************************************
 ** \brief MFS ch.1/9 RX IRQ handler
 ******************************************************************************/
void MFS1_9_RX_IRQHandler(void)
{
    uint32_t u32IrqMon = FM_INTREQ->PDL_IRQMON_MFS1_9_RX;  
  
#if (PDL_INTERRUPT_ENABLE_MFS1 == PDL_ON)    
    if(0u != (u32IrqMon & 0x00000001u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs1].stcInternData.enMode)
        {
            case MfsUartMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_UART_MODE == PDL_ON)   
                MfsUartIrqHandlerRx((stc_mfsn_uart_t*)&UART1, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs1].stcInternData));
            #endif
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)     
                MfsCsioIrqHandlerRx((stc_mfsn_csio_t*)&CSIO1, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs1].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)     
                MfsI2cIrqHandlerRx((stc_mfsn_i2c_t*)&I2C1, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs1].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)  
                MfsLinIrqHandlerRx((stc_mfsn_lin_t*)&LIN1, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs1].stcInternData));
            #endif
                break;
            default:
                break;
        
        }
    }
#endif
#if (PDL_INTERRUPT_ENABLE_MFS9 == PDL_ON)        
    if(0u != (u32IrqMon & 0x00000002u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs9].stcInternData.enMode)
        {
            case MfsUartMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_UART_MODE == PDL_ON)    
                MfsUartIrqHandlerRx((stc_mfsn_uart_t*)&UART9, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs9].stcInternData));
            #endif
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)  
                MfsCsioIrqHandlerRx((stc_mfsn_csio_t*)&CSIO9, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs9].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)   
                MfsI2cIrqHandlerRx((stc_mfsn_i2c_t*)&I2C9, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs9].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)  
                MfsLinIrqHandlerRx((stc_mfsn_lin_t*)&LIN9, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs9].stcInternData));
            #endif
                break;
            default:
                break;
        
        }
    }
#endif    
}

/**
 ******************************************************************************
 ** \brief MFS ch.1/9 TX IRQ handler
 ******************************************************************************/
void MFS1_9_TX_IRQHandler(void)
{
    uint32_t u32IrqMon = FM_INTREQ->PDL_IRQMON_MFS1_9_TX;  
  
#if (PDL_INTERRUPT_ENABLE_MFS1 == PDL_ON)    
    if(0u != (u32IrqMon & 0x00000001u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs1].stcInternData.enMode)
        {
            case MfsUartMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_UART_MODE == PDL_ON)  
                MfsUartIrqHandlerTx((stc_mfsn_uart_t*)&UART1, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs1].stcInternData));
            #endif
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)   
                MfsCsioIrqHandlerTx((stc_mfsn_csio_t*)&CSIO1, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs1].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)  
                MfsI2cIrqHandlerTx((stc_mfsn_i2c_t*)&I2C1, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs1].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)    
                MfsLinIrqHandlerTx((stc_mfsn_lin_t*)&LIN1, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs1].stcInternData));
            #endif
                break;
            default:
                break;
        
        }
    }
    
    if(0u != (u32IrqMon & 0x00000002u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs1].stcInternData.enMode)
        {
            case MfsUartMode:
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)     
                MfsCsioIrqHandlerStatus((stc_mfsn_csio_t*)&CSIO1, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs1].stcInternData));
            #endif    
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)    
                MfsI2cIrqHandlerStatus((stc_mfsn_i2c_t*)&I2C1, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs1].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)  
                MfsLinIrqHandlerStatus((stc_mfsn_lin_t*)&LIN1, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs1].stcInternData));
            #endif
                break;
            default:
                break;
        
        } 
    }
#endif
#if (PDL_INTERRUPT_ENABLE_MFS9 == PDL_ON)        
    if(0u != (u32IrqMon & 0x00000004u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs9].stcInternData.enMode)
        {
            case MfsUartMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_UART_MODE == PDL_ON)    
                MfsUartIrqHandlerTx((stc_mfsn_uart_t*)&UART9, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs9].stcInternData));
            #endif
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)    
                MfsCsioIrqHandlerTx((stc_mfsn_csio_t*)&CSIO9, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs9].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)   
                MfsI2cIrqHandlerTx((stc_mfsn_i2c_t*)&I2C9, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs9].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)    
                MfsLinIrqHandlerTx((stc_mfsn_lin_t*)&LIN9, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs9].stcInternData));
            #endif
                break;
            default:
                break;
        
        }
    }
    
    if(0u != (u32IrqMon & 0x00000008u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs9].stcInternData.enMode)
        {
            case MfsUartMode:
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)  
                MfsCsioIrqHandlerStatus((stc_mfsn_csio_t*)&CSIO9, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs9].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)    
                MfsI2cIrqHandlerStatus((stc_mfsn_i2c_t*)&I2C9, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs9].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)   
                MfsLinIrqHandlerStatus((stc_mfsn_lin_t*)&LIN9, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs9].stcInternData));
            #endif
                break;
            default:
                break;
        
        }
    }
#endif    
}
#endif // #if (PDL_INTERRUPT_ENABLE_MFS1 == PDL_ON) || (PDL_INTERRUPT_ENABLE_MFS9 == PDL_ON)

#if (PDL_INTERRUPT_ENABLE_MFS2 == PDL_ON) || (PDL_INTERRUPT_ENABLE_MFS10 == PDL_ON)
/**
 ******************************************************************************
 ** \brief MFS ch.2/10 RX IRQ handler
 ******************************************************************************/
void MFS2_10_RX_IRQHandler(void)
{
    uint32_t u32IrqMon = FM_INTREQ->PDL_IRQMON_MFS2_10_RX;  
  
#if (PDL_INTERRUPT_ENABLE_MFS2 == PDL_ON)    
    if(0u != u32IrqMon & (0x00000001u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs2].stcInternData.enMode)
        {
            case MfsUartMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_UART_MODE == PDL_ON)  
                MfsUartIrqHandlerRx((stc_mfsn_uart_t*)&UART2, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs2].stcInternData));
            #endif
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)    
                MfsCsioIrqHandlerRx((stc_mfsn_csio_t*)&CSIO2, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs2].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)   
                MfsI2cIrqHandlerRx((stc_mfsn_i2c_t*)&I2C2, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs2].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)     
                MfsLinIrqHandlerRx((stc_mfsn_lin_t*)&LIN2, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs2].stcInternData));
            #endif
                break;
            default:
                break;
        
        } 
    }
#endif
#if (PDL_INTERRUPT_ENABLE_MFS10 == PDL_ON)        
    if(0u != u32IrqMon & (0x00000002u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs10].stcInternData.enMode)
        {
            case MfsUartMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_UART_MODE == PDL_ON)  
                MfsUartIrqHandlerRx((stc_mfsn_uart_t*)&UART10, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs10].stcInternData));
            #endif
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)   
                MfsCsioIrqHandlerRx((stc_mfsn_csio_t*)&CSIO10, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs10].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)  
                MfsI2cIrqHandlerRx((stc_mfsn_i2c_t*)&I2C10, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs10].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)  
                MfsLinIrqHandlerRx((stc_mfsn_lin_t*)&LIN10, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs10].stcInternData));
            #endif
                break;
            default:
                break;
        
        } 
    }
#endif    
}

/**
 ******************************************************************************
 ** \brief MFS ch.2/10 TX IRQ handler
 ******************************************************************************/
void MFS2_10_TX_IRQHandler(void)
{
    uint32_t u32IrqMon = FM_INTREQ->PDL_IRQMON_MFS2_10_TX;  
  
#if (PDL_INTERRUPT_ENABLE_MFS2 == PDL_ON)    
    if(0u != (u32IrqMon & 0x00000001u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs2].stcInternData.enMode)
        {
            case MfsUartMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_UART_MODE == PDL_ON)    
                MfsUartIrqHandlerTx((stc_mfsn_uart_t*)&UART2, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs2].stcInternData));
            #endif
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)    
                MfsCsioIrqHandlerTx((stc_mfsn_csio_t*)&CSIO2, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs2].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)    
                MfsI2cIrqHandlerTx((stc_mfsn_i2c_t*)&I2C2, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs2].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)   
                MfsLinIrqHandlerTx((stc_mfsn_lin_t*)&LIN2, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs2].stcInternData));
            #endif
                break;
            default:
                break;
        
        }
    }
    
    if(0u != (u32IrqMon & 0x00000002u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs2].stcInternData.enMode)
        {
            case MfsUartMode:
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)    
                MfsCsioIrqHandlerStatus((stc_mfsn_csio_t*)&CSIO2, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs2].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)   
                MfsI2cIrqHandlerStatus((stc_mfsn_i2c_t*)&I2C2, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs2].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)     
                MfsLinIrqHandlerStatus((stc_mfsn_lin_t*)&LIN2, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs2].stcInternData));
            #endif
                break;
            default:
                break;
        
        } 
    }
#endif
#if (PDL_INTERRUPT_ENABLE_MFS10 == PDL_ON)        
    if(0u != (u32IrqMon & 0x00000004u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs10].stcInternData.enMode)
        {
            case MfsUartMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_UART_MODE == PDL_ON)   
                MfsUartIrqHandlerTx((stc_mfsn_uart_t*)&UART10, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs10].stcInternData));
            #endif
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)  
                MfsCsioIrqHandlerTx((stc_mfsn_csio_t*)&CSIO10, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs10].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)     
                MfsI2cIrqHandlerTx((stc_mfsn_i2c_t*)&I2C10, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs10].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)     
                MfsLinIrqHandlerTx((stc_mfsn_lin_t*)&LIN10, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs10].stcInternData));
            #endif
                break;
            default:
                break;
        
        }
    }
    
    if(0u != (u32IrqMon & 0x00000008u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs10].stcInternData.enMode)
        {
            case MfsUartMode:
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)    
                MfsCsioIrqHandlerStatus((stc_mfsn_csio_t*)&CSIO10, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs10].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)   
                MfsI2cIrqHandlerStatus((stc_mfsn_i2c_t*)&I2C10, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs10].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)   
                MfsLinIrqHandlerStatus((stc_mfsn_lin_t*)&LIN10, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs10].stcInternData));
            #endif
                break;
            default:
                break;
        
        }
    }
#endif    
}
#endif // #if (PDL_INTERRUPT_ENABLE_MFS2 == PDL_ON) || (PDL_INTERRUPT_ENABLE_MFS10 == PDL_ON)

#if (PDL_INTERRUPT_ENABLE_MFS3 == PDL_ON) || (PDL_INTERRUPT_ENABLE_MFS11 == PDL_ON)
/**
 ******************************************************************************
 ** \brief MFS ch.3/11 RX IRQ handler
 ******************************************************************************/
void MFS3_11_RX_IRQHandler(void)
{
    uint32_t u32IrqMon = FM_INTREQ->PDL_IRQMON_MFS3_11_RX;  
  
#if (PDL_INTERRUPT_ENABLE_MFS3 == PDL_ON)    
    if(0u != (u32IrqMon & 0x00000001u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs3].stcInternData.enMode)
        {
            case MfsUartMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_UART_MODE == PDL_ON)     
                MfsUartIrqHandlerRx((stc_mfsn_uart_t*)&UART3, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs3].stcInternData));
            #endif
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)    
                MfsCsioIrqHandlerRx((stc_mfsn_csio_t*)&CSIO3, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs3].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)  
                MfsI2cIrqHandlerRx((stc_mfsn_i2c_t*)&I2C3, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs3].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)  
                MfsLinIrqHandlerRx((stc_mfsn_lin_t*)&LIN3, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs3].stcInternData));
            #endif    
                break;
            default:
                break;
        
        } 
    }
#endif
#if (PDL_INTERRUPT_ENABLE_MFS11 == PDL_ON)        
    if(0u != (u32IrqMon & 0x00000002u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs11].stcInternData.enMode)
        {
            case MfsUartMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_UART_MODE == PDL_ON)  
                MfsUartIrqHandlerRx((stc_mfsn_uart_t*)&UART11, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs11].stcInternData));
            #endif
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)      
                MfsCsioIrqHandlerRx((stc_mfsn_csio_t*)&CSIO11, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs11].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)    
                MfsI2cIrqHandlerRx((stc_mfsn_i2c_t*)&I2C11, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs11].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)  
                MfsLinIrqHandlerRx((stc_mfsn_lin_t*)&LIN11, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs11].stcInternData));
            #endif
                break;
            default:
                break;
        
        } 
    }
#endif    
}

/**
 ******************************************************************************
 ** \brief MFS ch.3/11 TX IRQ handler
 ******************************************************************************/
void MFS3_11_TX_IRQHandler(void)
{
    uint32_t u32IrqMon = FM_INTREQ->PDL_IRQMON_MFS3_11_TX;  
  
#if (PDL_INTERRUPT_ENABLE_MFS3 == PDL_ON)    
    if(0u != (u32IrqMon & 0x00000001u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs3].stcInternData.enMode)
        {
            case MfsUartMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_UART_MODE == PDL_ON)    
                MfsUartIrqHandlerTx((stc_mfsn_uart_t*)&UART3, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs3].stcInternData));
            #endif
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)  
                MfsCsioIrqHandlerTx((stc_mfsn_csio_t*)&CSIO3, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs3].stcInternData));
            #endif    
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)  
                MfsI2cIrqHandlerTx((stc_mfsn_i2c_t*)&I2C3, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs3].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)   
                MfsLinIrqHandlerTx((stc_mfsn_lin_t*)&LIN3, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs3].stcInternData));
            #endif
                break;
            default:
                break;
        
        }
    }
    if(0u != (u32IrqMon & 0x00000002u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs3].stcInternData.enMode)
        {
            case MfsUartMode:
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)   
                MfsCsioIrqHandlerStatus((stc_mfsn_csio_t*)&CSIO3, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs3].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)    
                MfsI2cIrqHandlerStatus((stc_mfsn_i2c_t*)&I2C3, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs3].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)   
                MfsLinIrqHandlerStatus((stc_mfsn_lin_t*)&LIN3, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs3].stcInternData));
            #endif
                break;
            default:
                break;
        
        }
    }
#endif
#if (PDL_INTERRUPT_ENABLE_MFS11 == PDL_ON)        
    if(0u != (u32IrqMon & 0x00000004u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs11].stcInternData.enMode)
        {
            case MfsUartMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_UART_MODE == PDL_ON)    
                MfsUartIrqHandlerTx((stc_mfsn_uart_t*)&UART11, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs11].stcInternData));
            #endif
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)  
                MfsCsioIrqHandlerTx((stc_mfsn_csio_t*)&CSIO11, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs11].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)  
                MfsI2cIrqHandlerTx((stc_mfsn_i2c_t*)&I2C11, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs11].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)     
                MfsLinIrqHandlerTx((stc_mfsn_lin_t*)&LIN11, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs11].stcInternData));
            #endif
                break;
            default:
                break;
        
        }
    }
    if(0u != (u32IrqMon & 0x00000008u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs11].stcInternData.enMode)
        {
            case MfsUartMode:
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)    
                MfsCsioIrqHandlerStatus((stc_mfsn_csio_t*)&CSIO11, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs11].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)    
                MfsI2cIrqHandlerStatus((stc_mfsn_i2c_t*)&I2C11, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs11].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)   
                MfsLinIrqHandlerStatus((stc_mfsn_lin_t*)&LIN11, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs11].stcInternData));
            #endif
                break;
            default:
                break;
        
        }
    }
#endif    
}
#endif // #if (PDL_INTERRUPT_ENABLE_MFS3 == PDL_ON) || (PDL_INTERRUPT_ENABLE_MFS11 == PDL_ON)

#if (PDL_INTERRUPT_ENABLE_MFS4 == PDL_ON) || (PDL_INTERRUPT_ENABLE_MFS12 == PDL_ON)
/**
 ******************************************************************************
 ** \brief MFS ch.4/12 RX IRQ handler
 ******************************************************************************/
void MFS4_12_RX_IRQHandler(void)
{
    uint32_t u32IrqMon = FM_INTREQ->PDL_IRQMON_MFS4_12_RX;  
  
#if (PDL_INTERRUPT_ENABLE_MFS4 == PDL_ON)    
    if(0u != (u32IrqMon & 0x00000001u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs4].stcInternData.enMode)
        {
            case MfsUartMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_UART_MODE == PDL_ON)    
                MfsUartIrqHandlerRx((stc_mfsn_uart_t*)&UART4, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs4].stcInternData));
            #endif
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)  
                MfsCsioIrqHandlerRx((stc_mfsn_csio_t*)&CSIO4, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs4].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)   
                MfsI2cIrqHandlerRx((stc_mfsn_i2c_t*)&I2C4, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs4].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)    
                MfsLinIrqHandlerRx((stc_mfsn_lin_t*)&LIN4, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs4].stcInternData));
            #endif
                break;
            default:
                break;
        
        } 
    }
#endif
#if (PDL_INTERRUPT_ENABLE_MFS12 == PDL_ON)        
    if(0u != (u32IrqMon & 0x00000002u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs12].stcInternData.enMode)
        {
            case MfsUartMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_UART_MODE == PDL_ON)  
                MfsUartIrqHandlerRx((stc_mfsn_uart_t*)&UART12, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs12].stcInternData));
            #endif
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)    
                MfsCsioIrqHandlerRx((stc_mfsn_csio_t*)&CSIO12, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs12].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)     
                MfsI2cIrqHandlerRx((stc_mfsn_i2c_t*)&I2C12, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs12].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)    
                MfsLinIrqHandlerRx((stc_mfsn_lin_t*)&LIN12, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs12].stcInternData));
            #endif
                break;
            default:
                break;
        
        }  
    }
#endif    
}

/**
 ******************************************************************************
 ** \brief MFS ch.4/12 TX IRQ handler
 ******************************************************************************/
void MFS4_12_TX_IRQHandler(void)
{
    uint32_t u32IrqMon = FM_INTREQ->PDL_IRQMON_MFS4_12_TX;  
  
#if (PDL_INTERRUPT_ENABLE_MFS4 == PDL_ON)    
    if(0u != (u32IrqMon & 0x00000001u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs4].stcInternData.enMode)
        {
            case MfsUartMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_UART_MODE == PDL_ON)   
                MfsUartIrqHandlerTx((stc_mfsn_uart_t*)&UART4, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs4].stcInternData));
            #endif
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)  
                MfsCsioIrqHandlerTx((stc_mfsn_csio_t*)&CSIO4, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs4].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)  
                MfsI2cIrqHandlerTx((stc_mfsn_i2c_t*)&I2C4, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs4].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)      
                MfsLinIrqHandlerTx((stc_mfsn_lin_t*)&LIN4, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs4].stcInternData));
            #endif
                break;
            default:
                break;
        }
    }
    if(0u != (u32IrqMon & 0x00000002u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs4].stcInternData.enMode)
        {
            case MfsUartMode:
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)  
                MfsCsioIrqHandlerStatus((stc_mfsn_csio_t*)&CSIO4, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs4].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)  
                MfsI2cIrqHandlerStatus((stc_mfsn_i2c_t*)&I2C4, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs4].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)  
                MfsLinIrqHandlerStatus((stc_mfsn_lin_t*)&LIN4, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs4].stcInternData));
            #endif
                break;
            default:
                break;
        
        }
    }
#endif
#if (PDL_INTERRUPT_ENABLE_MFS12 == PDL_ON)        
    if(0u != (u32IrqMon & 0x00000004u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs12].stcInternData.enMode)
        {
            case MfsUartMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_UART_MODE == PDL_ON)   
                MfsUartIrqHandlerTx((stc_mfsn_uart_t*)&UART12, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs12].stcInternData));
            #endif
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)    
                MfsCsioIrqHandlerTx((stc_mfsn_csio_t*)&CSIO12, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs12].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)   
                MfsI2cIrqHandlerTx((stc_mfsn_i2c_t*)&I2C12, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs12].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)    
                MfsLinIrqHandlerTx((stc_mfsn_lin_t*)&LIN12, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs12].stcInternData));
            #endif
                break;
            default:
                break;
        }
    }
    if(0u != (u32IrqMon & 0x00000008u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs12].stcInternData.enMode)
        {
            case MfsUartMode:
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)  
                MfsCsioIrqHandlerStatus((stc_mfsn_csio_t*)&CSIO12, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs12].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)   
                MfsI2cIrqHandlerStatus((stc_mfsn_i2c_t*)&I2C12, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs12].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)      
                MfsLinIrqHandlerStatus((stc_mfsn_lin_t*)&LIN12, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs12].stcInternData));
            #endif
                break;
            default:
                break;
        
        }
    }
#endif    
}
#endif // #if (PDL_INTERRUPT_ENABLE_MFS4 == PDL_ON) || (PDL_INTERRUPT_ENABLE_MFS12 == PDL_ON)

#if (PDL_INTERRUPT_ENABLE_MFS5 == PDL_ON) || (PDL_INTERRUPT_ENABLE_MFS13 == PDL_ON)
/**
 ******************************************************************************
 ** \brief MFS ch.5/13 RX IRQ handler
 ******************************************************************************/
void MFS5_13_RX_IRQHandler(void)
{
    uint32_t u32IrqMon = FM_INTREQ->PDL_IRQMON_MFS5_13_RX;  
  
#if (PDL_INTERRUPT_ENABLE_MFS5 == PDL_ON)    
    if(0u != (u32IrqMon & 0x00000001u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs5].stcInternData.enMode)
        {
            case MfsUartMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_UART_MODE == PDL_ON)     
                MfsUartIrqHandlerRx((stc_mfsn_uart_t*)&UART5, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs5].stcInternData));
            #endif
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)    
                MfsCsioIrqHandlerRx((stc_mfsn_csio_t*)&CSIO5, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs5].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)   
                MfsI2cIrqHandlerRx((stc_mfsn_i2c_t*)&I2C5, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs5].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)  
                MfsLinIrqHandlerRx((stc_mfsn_lin_t*)&LIN5, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs5].stcInternData));
            #endif
                break;
            default:
                break;
        
        }  
    }
#endif
#if (PDL_INTERRUPT_ENABLE_MFS13 == PDL_ON)        
    if(0u != (u32IrqMon & 0x00000002u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs13].stcInternData.enMode)
        {
            case MfsUartMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_UART_MODE == PDL_ON)     
                MfsUartIrqHandlerRx((stc_mfsn_uart_t*)&UART13, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs13].stcInternData));
            #endif
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)    
                MfsCsioIrqHandlerRx((stc_mfsn_csio_t*)&CSIO13, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs13].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)   
                MfsI2cIrqHandlerRx((stc_mfsn_i2c_t*)&I2C13, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs13].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)  
                MfsLinIrqHandlerRx((stc_mfsn_lin_t*)&LIN13, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs13].stcInternData));
            #endif
                break;
            default:
                break;
        
        }  
    }
#endif    
}

/**
 ******************************************************************************
 ** \brief MFS ch.5/13 TX IRQ handler
 ******************************************************************************/
void MFS5_13_TX_IRQHandler(void)
{
    uint32_t u32IrqMon = FM_INTREQ->PDL_IRQMON_MFS5_13_TX;  
  
#if (PDL_INTERRUPT_ENABLE_MFS5 == PDL_ON)    
    if(0u != (u32IrqMon & 0x00000001u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs5].stcInternData.enMode)
        {
            case MfsUartMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_UART_MODE == PDL_ON)    
                MfsUartIrqHandlerTx((stc_mfsn_uart_t*)&UART5, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs5].stcInternData));
            #endif
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)      
                MfsCsioIrqHandlerTx((stc_mfsn_csio_t*)&CSIO5, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs5].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)   
                MfsI2cIrqHandlerTx((stc_mfsn_i2c_t*)&I2C5, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs5].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)   
                MfsLinIrqHandlerTx((stc_mfsn_lin_t*)&LIN5, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs5].stcInternData));
            #endif
                break;
            default:
                break;
        }
    }
    if(0u != (u32IrqMon & 0x00000002u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs5].stcInternData.enMode)
        {
            case MfsUartMode:
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)   
                MfsCsioIrqHandlerStatus((stc_mfsn_csio_t*)&CSIO5, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs5].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)   
                MfsI2cIrqHandlerStatus((stc_mfsn_i2c_t*)&I2C5, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs5].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)   
                MfsLinIrqHandlerStatus((stc_mfsn_lin_t*)&LIN5, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs5].stcInternData));
            #endif
                break;
            default:
                break;
        
        } 
    }
#endif
#if (PDL_INTERRUPT_ENABLE_MFS13 == PDL_ON)        
    if(0u != (u32IrqMon & 0x00000004u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs13].stcInternData.enMode)
        {
            case MfsUartMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_UART_MODE == PDL_ON)  
                MfsUartIrqHandlerTx((stc_mfsn_uart_t*)&UART13, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs13].stcInternData));
            #endif
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)    
                MfsCsioIrqHandlerTx((stc_mfsn_csio_t*)&CSIO13, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs13].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)  
                MfsI2cIrqHandlerTx((stc_mfsn_i2c_t*)&I2C13, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs13].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)  
                MfsLinIrqHandlerTx((stc_mfsn_lin_t*)&LIN13, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs13].stcInternData));
            #endif
                break;
            default:
                break;
        }
    }
    if(0u != (u32IrqMon & 0x00000008u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs13].stcInternData.enMode)
        {
            case MfsUartMode:
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON) 
                MfsCsioIrqHandlerStatus((stc_mfsn_csio_t*)&CSIO13, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs13].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)  
                MfsI2cIrqHandlerStatus((stc_mfsn_i2c_t*)&I2C13, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs13].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)    
                MfsLinIrqHandlerStatus((stc_mfsn_lin_t*)&LIN13, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs13].stcInternData));
            #endif
                break;
            default:
                break;
        
        }
    }
#endif    
}
#endif // #if (PDL_INTERRUPT_ENABLE_MFS5 == PDL_ON) || (PDL_INTERRUPT_ENABLE_MFS13 == PDL_ON)

#if (PDL_INTERRUPT_ENABLE_MFS6 == PDL_ON) || (PDL_INTERRUPT_ENABLE_MFS14 == PDL_ON) 
/**
 ******************************************************************************
 ** \brief MFS ch.6/14 RX IRQ handler
 ******************************************************************************/
void MFS6_14_RX_IRQHandler(void)
{
    uint32_t u32IrqMon = FM_INTREQ->PDL_IRQMON_MFS6_14_RX;  
  
#if (PDL_INTERRUPT_ENABLE_MFS6 == PDL_ON)    
    if(0u != (u32IrqMon & 0x00000001u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs6].stcInternData.enMode)
        {
            case MfsUartMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_UART_MODE == PDL_ON)   
                MfsUartIrqHandlerRx((stc_mfsn_uart_t*)&UART6, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs6].stcInternData));
            #endif
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)   
                MfsCsioIrqHandlerRx((stc_mfsn_csio_t*)&CSIO6, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs6].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)   
                MfsI2cIrqHandlerRx((stc_mfsn_i2c_t*)&I2C6, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs6].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)  
                MfsLinIrqHandlerRx((stc_mfsn_lin_t*)&LIN6, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs6].stcInternData));
            #endif
                break;
            default:
                break;
        
        }  
    }
#endif
#if (PDL_INTERRUPT_ENABLE_MFS14 == PDL_ON)        
    if(0u != (u32IrqMon & 0x00000002u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs14].stcInternData.enMode)
        {
            case MfsUartMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_UART_MODE == PDL_ON)   
                MfsUartIrqHandlerRx((stc_mfsn_uart_t*)&UART14, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs14].stcInternData));
            #endif
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)  
                MfsCsioIrqHandlerRx((stc_mfsn_csio_t*)&CSIO14, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs14].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)   
                MfsI2cIrqHandlerRx((stc_mfsn_i2c_t*)&I2C14, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs14].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)    
                MfsLinIrqHandlerRx((stc_mfsn_lin_t*)&LIN14, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs14].stcInternData));
            #endif
                break;
            default:
                break;
        
        }  
    }
#endif    

}

/**
 ******************************************************************************
 ** \brief MFS ch.6/14 TX, DMA ch. 1 IRQ handler
 ******************************************************************************/
void MFS6_14_TX_IRQHandler(void)
{
    uint32_t u32IrqMon = FM_INTREQ->PDL_IRQMON_MFS6_14_TX;  
  
#if (PDL_INTERRUPT_ENABLE_MFS6 == PDL_ON)    
    if(0u != (u32IrqMon & 0x00000001u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs6].stcInternData.enMode)
        {
            case MfsUartMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_UART_MODE == PDL_ON)   
                MfsUartIrqHandlerTx((stc_mfsn_uart_t*)&UART6, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs6].stcInternData));
            #endif
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)  
                MfsCsioIrqHandlerTx((stc_mfsn_csio_t*)&CSIO6, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs6].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)  
                MfsI2cIrqHandlerTx((stc_mfsn_i2c_t*)&I2C6, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs6].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)     
                MfsLinIrqHandlerTx((stc_mfsn_lin_t*)&LIN6, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs6].stcInternData));
            #endif
                break;
            default:
                break;
        }
    }
    if(0u != (u32IrqMon & 0x00000002u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs6].stcInternData.enMode)
        {
            case MfsUartMode:
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)   
                MfsCsioIrqHandlerStatus((stc_mfsn_csio_t*)&CSIO6, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs6].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)  
                MfsI2cIrqHandlerStatus((stc_mfsn_i2c_t*)&I2C6, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs6].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)   
                MfsLinIrqHandlerStatus((stc_mfsn_lin_t*)&LIN6, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs6].stcInternData));
            #endif
                break;
            default:
                break;
        
        } 
    }
#endif
#if (PDL_INTERRUPT_ENABLE_MFS14 == PDL_ON)        
    if(0u != (u32IrqMon & 0x00000004u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs14].stcInternData.enMode)
        {
            case MfsUartMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_UART_MODE == PDL_ON)    
                MfsUartIrqHandlerTx((stc_mfsn_uart_t*)&UART14, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs14].stcInternData));
            #endif
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)  
                MfsCsioIrqHandlerTx((stc_mfsn_csio_t*)&CSIO14, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs14].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)  
                MfsI2cIrqHandlerTx((stc_mfsn_i2c_t*)&I2C14, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs14].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)  
                MfsLinIrqHandlerTx((stc_mfsn_lin_t*)&LIN14, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs14].stcInternData));
            #endif
                break;
            default:
                break;
        }
    }
    if(0u != (u32IrqMon & 0x00000008u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs14].stcInternData.enMode)
        {
            case MfsUartMode:
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)   
                MfsCsioIrqHandlerStatus((stc_mfsn_csio_t*)&CSIO14, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs14].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)    
                MfsI2cIrqHandlerStatus((stc_mfsn_i2c_t*)&I2C14, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs14].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)  
                MfsLinIrqHandlerStatus((stc_mfsn_lin_t*)&LIN14, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs14].stcInternData));
            #endif
                break;
            default:
                break;
        
        }
    }
#endif     
    
}
#endif // #if (PDL_INTERRUPT_ENABLE_MFS6 == PDL_ON) || (PDL_INTERRUPT_ENABLE_MFS14 == PDL_ON)

#if (PDL_INTERRUPT_ENABLE_MFS7 == PDL_ON) || (PDL_INTERRUPT_ENABLE_MFS15 == PDL_ON) 

/**
 ******************************************************************************
 ** \brief MFS ch.7/15 RX, DMA ch. 2 IRQ handler
 ******************************************************************************/
void MFS7_15_RX_IRQHandler(void)
{
    uint32_t u32IrqMon = FM_INTREQ->PDL_IRQMON_MFS7_15_RX;  
  
#if (PDL_INTERRUPT_ENABLE_MFS7 == PDL_ON)    
    if(0u != (u32IrqMon & 0x00000001u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs7].stcInternData.enMode)
        {
            case MfsUartMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_UART_MODE == PDL_ON)   
                MfsUartIrqHandlerRx((stc_mfsn_uart_t*)&UART7, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs7].stcInternData));
            #endif
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)   
                MfsCsioIrqHandlerRx((stc_mfsn_csio_t*)&CSIO7, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs7].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)    
                MfsI2cIrqHandlerRx((stc_mfsn_i2c_t*)&I2C7, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs7].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)    
                MfsLinIrqHandlerRx((stc_mfsn_lin_t*)&LIN7, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs7].stcInternData));
            #endif
                break;
            default:
                break;
        
        }   
    }
#endif
#if (PDL_INTERRUPT_ENABLE_MFS15 == PDL_ON)        
    if(0u != (u32IrqMon & 0x00000002u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs15].stcInternData.enMode)
        {
            case MfsUartMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_UART_MODE == PDL_ON)   
                MfsUartIrqHandlerRx((stc_mfsn_uart_t*)&UART15, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs15].stcInternData));
            #endif
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)   
                MfsCsioIrqHandlerRx((stc_mfsn_csio_t*)&CSIO15, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs15].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)     
                MfsI2cIrqHandlerRx((stc_mfsn_i2c_t*)&I2C15, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs15].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)    
                MfsLinIrqHandlerRx((stc_mfsn_lin_t*)&LIN15, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs15].stcInternData));
            #endif
                break;
            default:
                break;
        
        }   
    }
#endif    
}

/**
 ******************************************************************************
 ** \brief MFS ch.7/15 TX, DMA ch. 3 IRQ handler
 ******************************************************************************/
void MFS7_15_TX_IRQHandler(void)
{
    uint32_t u32IrqMon = FM_INTREQ->PDL_IRQMON_MFS7_15_TX;
  
#if (PDL_INTERRUPT_ENABLE_MFS7 == PDL_ON)    
    if(0u != (u32IrqMon & 0x00000001u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs7].stcInternData.enMode)
        {
            case MfsUartMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_UART_MODE == PDL_ON)   
                MfsUartIrqHandlerTx((stc_mfsn_uart_t*)&UART7, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs7].stcInternData));
            #endif
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)   
                MfsCsioIrqHandlerTx((stc_mfsn_csio_t*)&CSIO7, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs7].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)    
                MfsI2cIrqHandlerTx((stc_mfsn_i2c_t*)&I2C7, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs7].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)    
                MfsLinIrqHandlerTx((stc_mfsn_lin_t*)&LIN7, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs7].stcInternData));
            #endif
                break;
            default:
                break;
        }
    }
    
    if(0u != (u32IrqMon & 0x00000002u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs7].stcInternData.enMode)
        {
            case MfsUartMode:
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)  
                MfsCsioIrqHandlerStatus((stc_mfsn_csio_t*)&CSIO7, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs7].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)      
                MfsI2cIrqHandlerStatus((stc_mfsn_i2c_t*)&I2C7, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs7].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)  
                MfsLinIrqHandlerStatus((stc_mfsn_lin_t*)&LIN7, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs7].stcInternData));
            #endif
                break;
            default:
                break;
        
        }
    }
#endif
#if (PDL_INTERRUPT_ENABLE_MFS15 == PDL_ON)        
    if(0u != (u32IrqMon & 0x00000004u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs15].stcInternData.enMode)
        {
            case MfsUartMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_UART_MODE == PDL_ON)  
                MfsUartIrqHandlerTx((stc_mfsn_uart_t*)&UART15, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs15].stcInternData));
            #endif
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)  
                MfsCsioIrqHandlerTx((stc_mfsn_csio_t*)&CSIO15, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs15].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)   
                MfsI2cIrqHandlerTx((stc_mfsn_i2c_t*)&I2C15, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs15].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)  
                MfsLinIrqHandlerTx((stc_mfsn_lin_t*)&LIN15, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs15].stcInternData));
            #endif
                break;
            default:
                break;
        }
    }
    if(0u != (u32IrqMon & 0x00000008u))
    {
        switch(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs15].stcInternData.enMode)
        {
            case MfsUartMode:
                break;
            case MfsCsioMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_CSIO_MODE == PDL_ON)    
                MfsCsioIrqHandlerStatus((stc_mfsn_csio_t*)&CSIO15, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs15].stcInternData));
            #endif
                break;          
            case MfsI2cMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_I2C_MODE == PDL_ON)   
                MfsI2cIrqHandlerStatus((stc_mfsn_i2c_t*)&I2C15, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs15].stcInternData));
            #endif
                break;          
            case MfsLinMode:
            #if (PDL_PERIPHERAL_ENABLE_MFS_LIN_MODE == PDL_ON)  
                MfsLinIrqHandlerStatus((stc_mfsn_lin_t*)&LIN15, &(m_astcMfsInstanceDataLut[MfsInstanceIndexMfs15].stcInternData));
            #endif
                break;
            default:
                break;
        
        }
    }
#endif    
    
}
#endif // #if (PDL_INTERRUPT_ENABLE_MFS7 == PDL_ON) || (PDL_INTERRUPT_ENABLE_MFS15 == PDL_ON)

/******************************************************************************/
/*********************************** USB0**************************************/
/******************************************************************************/
#if (PDL_INTERRUPT_ENABLE_USB0 == PDL_ON)
/**
 ******************************************************************************
 ** \brief USB0 function IRQ handler
 ******************************************************************************/
void USB0F_IRQHandler(void)
{
    //Usbf_IrqHandler(0);
}
/**
 ******************************************************************************
 ** \brief USB ch.0 function & host IRQ handler
 ******************************************************************************/
void USB0_IRQHandler(void)
{
    //Usb_IrqHandler(0);
}
#endif

/******************************************************************************/
/*********************************** USB1 & CEC *******************************/
/******************************************************************************/
#if (PDL_INTERRUPT_ENABLE_USB1 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_RC0 == PDL_ON) 
/**
 ******************************************************************************
 ** \brief USB ch.1 function, HDMI-CEC ch.0 IRQ handler
 ******************************************************************************/      
void USB1F_HDMICEC0_IRQHandler(void)
{
    uint32_t u32IrqMon = FM_INTREQ->PDL_IRQMON_USBF1_CEC0;

#if (PDL_INTERRUPT_ENABLE_USB1 == PDL_ON)
    if(0u != (u32IrqMon & 0x0000001Fu))
    {
        //Usbf_IrqHandler(1);
    }
#endif  
  
#if (PDL_INTERRUPT_ENABLE_RC0 == PDL_ON)
    if(0u != (u32IrqMon & 0x00000020u))
    {
        RcIrqHandler((volatile stc_rcn_t*)&RC0, &(m_astcRcInstanceDataLut[RcInstanceIndexRc0].stcInternData));
    }
#endif  
}
#endif

#if (PDL_INTERRUPT_ENABLE_USB1 == PDL_ON) || \
    (PDL_INTERRUPT_ENABLE_RC1 == PDL_ON) 
/**
 ******************************************************************************
 ** \brief USB ch.1 function & host, HDMI-CEC ch.1 IRQ handler
 ******************************************************************************/         
void USB1_HDMICEC1_IRQHandler(void)
{
    uint32_t u32IrqMon = FM_INTREQ->PDL_IRQMON_USB1_CEC1;
    
#if (PDL_INTERRUPT_ENABLE_USB1 == PDL_ON)
    if(0u != (u32IrqMon & 0x0000001Fu))
    {
        //Usb_IrqHandler(1);
    }
#endif  
  
#if (PDL_INTERRUPT_ENABLE_RC1 == PDL_ON)
    if(0u != (u32IrqMon & 0x00000020u))
    {
        RcIrqHandler((volatile stc_rcn_t*)&RC1, &(m_astcRcInstanceDataLut[RcInstanceIndexRc1].stcInternData));
    }
#endif  
}
#endif

#endif 
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
