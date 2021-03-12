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
/** \file lcd.h
 **
 ** Headerfile for LCD functions
 **  
 ** History:
 **   - 2015-01-23  1.0  DHo   First version for FM universal PDL.
 **
 ******************************************************************************/

#ifndef __LCD_H__
#define __LCD_H__

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "mcu.h"
#include "pdl_user.h"

#if (defined(PDL_PERIPHERAL_LCD_ACTIVE))

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/**
 ******************************************************************************
 ** \defgroup LcdGroup LCD controller (LCD)
 **
 ** Provided functions of LCD module:  
 ** 
 ** -  Lcd_Init()
 ** -  Lcd_DeInit()
 ** -  Lcd_ConfInputIoMode()
 ** -  Lcd_SetDispMode()
 ** -  Lcd_EnableBlankDisp()
 ** -  Lcd_DisableBlankDisp()
 ** -  Lcd_WriteRAMBits()
 ** -  Lcd_WriteRAMBit()
 ** -  Lcd_WriteRAMByte()
 ** -  Lcd_ReadRAMByte();
 ** -  Lcd_FillWholeRam()
 ** -  Lcd_ClrWholeRam()
 ** -  Lcd_SetBinkInterval()
 ** -  Lcd_SetBlinkDot()
 ** -  Lcd_EnableIrq()
 ** -  Lcd_DisableIrq()
 ** -  Lcd_ClrIrqFlag()
 ** -  Lcd_GetIrqFlag()
 ** 
 ** Lcd_Init() initializes an LCD instance according the configuration of the structure 
 ** type #stc_lcd_config_t. 
 **
 ** Lcd_DeInit() de-initializes an LCD instance.
 **
 ** Lcd_ConfInputIoMode() is used to config LCD input IO mode selected by the structure
 ** type #en_lcd_input_io_mode_t.
 **
 ** Lcd_SetDispMode() is used to  config LCD display mode  selected by the
 ** structure type #en_lcd_disp_mode_t
 **
 ** Lcd_EnableBlankDisp() can be used to display blank.
 **
 ** Lcd_DisableBlankDisp() can be used to display data stored in LCDRAM.
 **
 ** Lcd_WriteRAMBits()  sets multi-bits in a byte of LCD RAM.
 **
 ** Lcd_WriteRAMBit()  sets LCDRAM 1 bit n a byte of LCD RAM.
 **
 ** Lcd_WriteRAMByte() writes byte of LCD RAM
 **
 ** Lcd_ReadRAMByte() reads byte from LCD RAM.
 **
 ** Lcd_FillWholeRam() fills whole LCD RAM to display all.
 **
 ** Lcd_ClrWholeRam() clears whole LCDRAM.
 **
 ** Lcd_SetBinkInterval()  selects blink interval time.
 ** 
 ** Lcd_SetBlinkDot()  selects blink dot.
 ** 
 ** Lcd_EnableIrq() enables lcd interrupt.
 **
 ** Lcd_DisableIrq()  disables lcd interrupt.
 ** 
 ** Lcd_ClrIrqFlag()  clears interrupt flag.
 ** 
 ** Lcd_GetIrqFlag() gets interrupt flag. 
 **
 ******************************************************************************/
//@{


/******************************************************************************/
/* Global pre-processor symbols/macros ('#define')                            */
/******************************************************************************/
 
/******************************************************************************
 * Global type definitions
 ******************************************************************************/
 
/**
 ******************************************************************************
 ** \brief Select divider resitors to create LCD drive power 
 ******************************************************************************/
typedef enum en_lcd_div_mode
{
    LcdcExtRes = 0u,   ///<  External divider resistors are used to create LCD drive power.
    LcdcIntRes = 1u,   ///<  Internal divider resistors are used to create LCD drive power.
}en_lcd_div_mode_t;

/**
 ******************************************************************************
 ** \brief Divider resistor value selection
 ******************************************************************************/
typedef enum en_lcd_disp_mode
{
    LcdStop = 0u,              ///<  LCD controller stops display operations.
    Lcd_4Com_1Div2Duty = 1u,   ///< 4 COM mode, 1/2 duty
    Lcd_4Com_1Div3Duty = 2u,   ///< 4 COM mode, 1/3 duty
    Lcd_4Com_1Div4Duty = 3u,   ///< 4 COM mode, 1/4 duty
    Lcd_8Com_1Div8Duty = 4u,   ///< 8 COM mode, 1/8 duty
}en_lcd_disp_mode_t;

/**
 ******************************************************************************
 ** \brief Divider resistor value selection
 ******************************************************************************/
typedef enum en_lcd_div_res_val
{
    LcdDivRes100K = 0u,   ///< 100 k¦¸ resistors are selected.
    LcdDivRes10K  = 1u,   ///<  100 k¦¸ resistors are selected.
}en_lcd_div_res_val_t;

/**
 ******************************************************************************
 ** \brief 8 COM mode bias selection
 ******************************************************************************/
typedef enum en_lcd_8com_bias
{
    Lcd1Div3Bias = 0u,   ///< 1/3 bias is selected in 8 COM mode.
    Lcd1Div4Bias = 1u,   ///< 1/4 bias is selected in 8 COM mode.
}en_lcd_8com_bias_t;

/**
 ******************************************************************************
 ** \brief Source clock selection
 ******************************************************************************/
typedef enum en_lcd_src_clk_sel
{
    LcdClkSelSubClk = 0u,   ///< Sub-clock is selected for LCDC source clock.
    LcdClkSelPclk   = 1u,   ///< PCLK is selected for LCDC source clock.
}en_lcd_src_clk_sel_t;

/**
 ******************************************************************************
 ** \brief Select lcd source clock and configure clock division ratio 
 ******************************************************************************/
typedef struct stc_lcd_clk_config
{
    en_lcd_src_clk_sel_t enSrcClk;  ///< Set LCDC source clock.
    uint32_t u32DivVal;             ///<  These bits set LCDC clock division ratio (1 to 2097153).
}stc_lcd_clk_config_t;

/**
 ******************************************************************************
 ** \brief VVx selection bit
 **
 ** \note As VV4 pin cannot be used as GPIO when LCD controller is 
 **        selected (LCDCC1:VSEL="1"), be sure to write "1" to VE4 bit.
 ******************************************************************************/
typedef struct stc_lcd_vv_sel
{
    union unVvSel
    {
        uint8_t u8Vv;
        struct stcVvBits
        {            
            uint8_t VE0  : 1 ;
            uint8_t VE1  : 1 ;
            uint8_t VE2  : 1 ;
            uint8_t VE3  : 1 ;
            uint8_t VE4  : 1 ;
            uint8_t RESERVED0  : 3;
        };
    };
} stc_lcd_vv_sel_t ;

/**
 ******************************************************************************
 ** \brief These bits control I/O port status and analog switches for SEG outputs.
 ******************************************************************************/
typedef struct stc_lcd_seg_sel1
{
    union unSegSel1
    {
        uint32_t u32SegSel1;
        struct stcSegSel1Bits
        {
            uint32_t SEG00  : 1 ;
            uint32_t SEG01  : 1 ;
            uint32_t SEG02  : 1 ;
            uint32_t SEG03  : 1 ;
            uint32_t SEG04  : 1 ;
            uint32_t SEG05  : 1 ;
            uint32_t SEG06  : 1 ;
            uint32_t SEG07  : 1 ;
            uint32_t SEG08  : 1 ;
            uint32_t SEG09  : 1 ;
            uint32_t SEG10  : 1 ;
            uint32_t SEG11  : 1 ;
            uint32_t SEG12  : 1 ;
            uint32_t SEG13  : 1 ;
            uint32_t SEG14  : 1 ;
            uint32_t SEG15  : 1 ;
            uint32_t SEG16  : 1 ;
            uint32_t SEG17  : 1 ;
            uint32_t SEG18  : 1 ;
            uint32_t SEG19  : 1 ;
            uint32_t SEG20  : 1 ;
            uint32_t SEG21  : 1 ;
            uint32_t SEG22  : 1 ;
            uint32_t SEG23  : 1 ;
            uint32_t SEG24  : 1 ;
            uint32_t SEG25  : 1 ;
            uint32_t SEG26  : 1 ;
            uint32_t SEG27  : 1 ;
            uint32_t SEG28  : 1 ;
            uint32_t SEG29  : 1 ;
            uint32_t SEG30  : 1 ;
            uint32_t SEG31  : 1 ;
        };
    };
} stc_lcd_seg_sel1_t ;

/**
 ******************************************************************************
 ** \brief These bits control I/O port status and analog switches for SEG outputs.
 ******************************************************************************/
typedef struct stc_lcd_seg_sel2
{
    union unSegSel2
    {
        uint32_t u32SegSel2;
        struct stcSegSel2Bits
        {
            uint32_t SEG32  : 1 ;
            uint32_t SEG33  : 1 ;
            uint32_t SEG34  : 1 ;
            uint32_t SEG35  : 1 ;
            uint32_t SEG36  : 1 ;
            uint32_t SEG37  : 1 ;
            uint32_t SEG38  : 1 ;
            uint32_t SEG39  : 1 ;
            uint32_t RESERVED0  : 24;
        };
    };
} stc_lcd_seg_sel2_t ;

/**
 ******************************************************************************
 ** \brief These bits control I/O port status and analog switches for COM outputs.
 ******************************************************************************/
typedef struct stc_lcd_com_sel
{
    union unComSel
    {
        uint8_t u8ComSel;
        struct stcComSelBits
        {
            uint32_t COM0  : 1 ;
            uint32_t COM1  : 1 ;
            uint32_t COM2  : 1 ;
            uint32_t COM3  : 1 ;
            uint32_t COM4  : 1 ;
            uint32_t COM5  : 1;
            uint32_t COM6  : 1 ;
            uint32_t COM7  : 1 ;
        };
    };
} stc_lcd_com_sel_t;

/**
 ******************************************************************************
 ** \brief controls I/O ports shared by COM and SEG.
 ******************************************************************************/
typedef enum en_lcd_input_io_mode
{
    LcdInputIoCutoff  = 0u,   ///< Input from I/O port is cut off.
    LcdInputIoConnect = 1u,   ///< Input from I/O port is not cut off.

}en_lcd_input_io_mode_t;

/**
 ******************************************************************************
 ** \brief Blink interval
 ******************************************************************************/
typedef enum en_lcd_blink_interval
{
    LcdBlinkIntHalfSecond = 0u,   ///< If sub-clock is 32.768 [kHz], the interval becomes 0.5 [s].
    LcdBlinkIntOneSecond  = 1u,   ///< If sub-clock is 32.768 [kHz], the interval becomes 1.0 [s].

}en_lcd_blink_interval_t;

/**
 ******************************************************************************
 ** \brief Blink interval
 ******************************************************************************/
typedef enum en_lcd_ram_bit_operation
{
    LcdBitClr = 0u,   ///< Clear lcd ram bit.
    LcdBitSet = 1u,   ///< Set lcd ram bit.

}en_lcd_ram_bit_operation_t;

/**
 ******************************************************************************
 ** \brief LCD blink function enumeration
 ******************************************************************************/
typedef enum en_lcd_blink_operation
{   
    LcdBlinkOff = 0u,       ///< LCD blink on
    LcdBlinkOn  = 1u,       ///< LCD blink off
}en_lcd_blink_operation_t;

/**
 ******************************************************************************
 ** \brief LCD blink SEG COM dot enumeration
 ******************************************************************************/
typedef enum en_lcd_blink_dot_t
{   
    LCDC_BlinkDotOff = 0u,        ///< LCD blink on
/***4COM*****************/
    LCDC_Blik4COMS3C3,            ///< lcdc blink for SEG3-COM3 
    LCDC_Blik4COMS3C2,            ///< lcdc blink for SEG3-COM2 
    LCDC_Blik4COMS3C1,            ///< lcdc blink for SEG3-COM1 
    LCDC_Blik4COMS3C0,            ///< lcdc blink for SEG3-COM0 
    LCDC_Blik4COMS2C3,            ///< lcdc blink for SEG2-COM3 
    LCDC_Blik4COMS2C2,            ///< lcdc blink for SEG2-COM2 
    LCDC_Blik4COMS2C1,            ///< lcdc blink for SEG2-COM1 
    LCDC_Blik4COMS2C0,            ///< lcdc blink for SEG2-COM0 
    LCDC_Blik4COMS1C3,            ///< lcdc blink for SEG1-COM3 
    LCDC_Blik4COMS1C2,            ///< lcdc blink for SEG1-COM2 
    LCDC_Blik4COMS1C1,            ///< lcdc blink for SEG1-COM1
    LCDC_Blik4COMS1C0,            ///< lcdc blink for SEG1-COM0
    LCDC_Blik4COMS0C3,            ///< lcdc blink for SEG0-COM3
    LCDC_Blik4COMS0C2,            ///< lcdc blink for SEG0-COM2
    LCDC_Blik4COMS0C1,            ///< lcdc blink for SEG0-COM1
    LCDC_Blik4COMS0C0,            ///< lcdc blink for SEG0-COM0
/***8COM*****************/    
    LCDC_Blik8COMS1C7,            ///< lcdc blink for SEG1-COM7 
    LCDC_Blik8COMS1C6,            ///< lcdc blink for SEG1-COM6 
    LCDC_Blik8COMS1C5,            ///< lcdc blink for SEG1-COM5 
    LCDC_Blik8COMS1C4,            ///< lcdc blink for SEG1-COM4 
    LCDC_Blik8COMS1C3,            ///< lcdc blink for SEG1-COM3 
    LCDC_Blik8COMS1C2,            ///< lcdc blink for SEG1-COM2 
    LCDC_Blik8COMS1C1,            ///< lcdc blink for SEG1-COM1 
    LCDC_Blik8COMS1C0,            ///< lcdc blink for SEG1-COM0 
    LCDC_Blik8COMS0C7,            ///< lcdc blink for SEG0-COM7 
    LCDC_Blik8COMS0C6,            ///< lcdc blink for SEG0-COM6 
    LCDC_Blik8COMS0C5,            ///< lcdc blink for SEG0-COM5
    LCDC_Blik8COMS0C4,            ///< lcdc blink for SEG0-COM4
    LCDC_Blik8COMS0C3,            ///< lcdc blink for SEG0-COM3
    LCDC_Blik8COMS0C2,            ///< lcdc blink for SEG0-COM2
    LCDC_Blik8COMS0C1,            ///< lcdc blink for SEG0-COM1
    LCDC_Blik8COMS0C0,            ///< lcdc blink for SEG0-COM0
}en_lcd_blink_dot_t;

/**
 ******************************************************************************
 ** \brief LCD booster function
 ******************************************************************************/
typedef struct stc_lcd_booster
{
    union unBooster
    {
        uint16_t u16Booster;
        struct stcBoosterBits
        {
            uint16_t BSTOPT     : 1;///< booster function could be available
            uint16_t BSTPD      : 1;///< Control booster power
            uint16_t RESERVED0  : 1;
            uint16_t CENSEL     : 1;///< Control booster C1/C0 pin funciton 
            uint16_t RESERVED1  : 4;
            uint16_t BTRF       : 4;///< Booster fine setting bits for reference voltage of VV1
            uint16_t BTRC       : 2;///< Booster coarse setting bits for reference voltage of VV1
        };
    };
}stc_lcd_booster_t;

/**
 ******************************************************************************
 ** \brief Clock Supervisor configuration
 ** 
 ** The Clock Supervisor configuration settings
 ******************************************************************************/
typedef struct stc_lcd_config
{
    boolean_t bTimerMod;                ///< FALSE: sops running in timer mode. TRUE:run in timer mode.
    en_lcd_div_mode_t enDivMode;        ///< LCD drive power control, see #en_lcd_div_mode_t for details
    en_lcd_disp_mode_t enDispMode;      ///< LCD controller display mode selection, see #en_lcd_disp_mode_t for details
    en_lcd_div_res_val_t enDivResVal;   ///< Divider resistor value selection, see #en_lcd_div_res_val_t for details
    en_lcd_8com_bias_t en8ComBias;      ///< 8 COM mode bias selection, see #en_lcd_8com_bias_t for details
    boolean_t bEnDispRevs;              ///< Reverse display control. FALSE:Display is not reversed.  TRUE:Display is reversed.
    boolean_t bEnBlankDisp;             ///< TRUE:Blank is displayed independent of data stored in LCDRAM
    en_lcd_input_io_mode_t enInputIoMode;   ///< I/O port input control,  see #en_lcd_input_io_mode_t for details
    stc_lcd_clk_config_t stcClkConf;  ///< LCD clock control,  see #stc_lcd_clk_config_t for details
    stc_lcd_vv_sel_t stcVvSel;        ///< VVx control,  see #stc_lcd_vv_sel_t for details
    stc_lcd_seg_sel1_t stcSegSel1;    ///< Segment output pins (SEG00 to SEG31) control,  see #stc_lcd_seg_sel1_t for details
    stc_lcd_seg_sel2_t stcSegSel2;    ///< Segment output pins (SEG32 to SEG39) control,  see #stc_lcd_seg_sel2_t for details
    stc_lcd_com_sel_t stcComSel;      ///< COM output pins (COM0 to COM7) control,  see #stc_lcd_com_sel_t for details
#if (PDL_MCU_CORE == PDL_FM0P_CORE)
    boolean_t   bEnBooster;           ///< Enable booster function
    stc_lcd_booster_t stcBooster;     ///< Configure LCD booster functionl,  see #stc_lcd_booster_t for details
#endif

#if (PDL_INTERRUPT_ENABLE_LCD == PDL_ON)
    boolean_t  bIrqEn;                ///< Enable LCD interrupt
    func_ptr_t pfnIrqCb;              ///< Pointer to LCD interrupt callback function
    boolean_t bTouchNvic;             ///< TRUE: enable NVIC, FALSE: don't enable NVIC
#endif    
} stc_lcd_config_t;


/******************************************************************************/
/* Global variable definitions ('extern')                                     */
/******************************************************************************/

/******************************************************************************/
/* Global function prototypes (definition in C source)                        */
/******************************************************************************/
#if (PDL_INTERRUPT_ENABLE_LCD == PDL_ON)
void Lcd_IrqHandler(void);
#endif
en_result_t Lcd_Init(stc_lcd_config_t* pstcConfig);
void Lcd_DeInit( void);
en_result_t Lcd_ConfInputIoMode(en_lcd_input_io_mode_t enInputIoMode);
void Lcd_SetDispMode(en_lcd_disp_mode_t enDispMode);
void Lcd_EnableBlankDisp(void);
void Lcd_DisableBlankDisp(void);
en_result_t Lcd_WriteRAMBits(uint8_t u8RAMIndex, uint8_t u8Bitsmap,
                             boolean_t bBit);
en_result_t Lcd_WriteRAMBit(uint8_t u8RAMIndex, uint8_t u8BitIndex, 
                            boolean_t bBit);
en_result_t Lcd_WriteRAMByte(uint8_t u8RAMIndex, uint8_t u8DataByte);  
en_result_t Lcd_ReadRAMByte(uint8_t u8RAMIndex,uint8_t *pu8DataByte);  
void Lcd_FillWholeRam(void);
void Lcd_ClrWholeRam(void);
en_result_t Lcd_SetBinkInterval(en_lcd_blink_interval_t enBlinkInterval);
en_result_t Lcd_SetBlinkDot(en_lcd_blink_dot_t enBlinkDot, 
                            en_lcd_blink_operation_t enBlinkOp);

/* 
interrupt Setting */
#if (PDL_INTERRUPT_ENABLE_LCD == PDL_ON)
void Lcd_EnableIrq(void);
void Lcd_DisableIrq(void);
#endif
void Lcd_ClrIrqFlag(void);
boolean_t Lcd_GetIrqFlag(void);

#ifdef __cplusplus
}
#endif

//@}

#endif /* #if (defined(PDL_PERIPHERAL_LCD_ACTIVE)) */

#endif /* __LCD_H__ */
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
