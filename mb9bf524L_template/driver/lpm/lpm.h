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
/** \file lpm.h
 **
 ** Headerfile for LPM functions
 **  
 ** History:
 **   - 2014-12-13  0.1  EZh  First version for FM universal PDL.
 **
 ******************************************************************************/

#ifndef __LPM_H__
#define __LPM_H__

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "mcu.h"
#include "pdl_user.h"

#if (defined(PDL_PERIPHERAL_LPM_ACTIVE))

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
  
/**
 ******************************************************************************
 ** \defgroup LpmGroup Low Power Modes Functions (LPM)
 ** 
 ** Provided functions of LPM module:
 **
 ** - Lpm_GoToStandByMode()
 ** - Lpm_ConfigDeepStbRetCause()
 ** - Lpm_ReadDeepStbRetCause()
 ** - Lpm_SetWkupPinLevel()
 ** - Lpm_ConfigSubClk()
 ** - Lpm_ConfigDeepStbRAMRetention()
 ** - Lpm_WriteBackupReg()
 ** - Lpm_ReadBackupReg() 
 **
 **  Lpm_GoToStandByMode() enters the low power consumption selected by 
 **  the parameter Lpm_GoToStandByMode#enMode. Since MCU enters into low
 **  power consumption mode after calling this funciton, this function will
 **  not return until MCU is waken up. There are also different between waken up
 **  from Standby mode and deep standby mode. When MCU is waken up from standby
 **  mode, the program of MCU will run after this function, but when MCU is 
 **  waken up from deep standby mode, the program of MCU will run from the reset
 **  vector.
 **
 **  Lpm_ConfigDeepStbRetCause() configures the return causes of deep standby
 **  mode and Lpm_ReadDeepStbRetCause() reads the return causes after MCU is 
 **  waken up. Note that Wakeup from WKUP0 pin is always enabled.
 ** 
 **  Lpm_SetWkupPinLevel() is used to set the WKUP pin active level.  
 ** 
 **  Lpm_ConfigSubClk() configures whether to supply Sub clock to RTC and CEC 
 **  in deep standby mode. Not all FM product has this function. Please 
 **  confirm it in the hardware manual.
 **
 **  Lpm_ConfigDeepStbRAMRetention() configures whether to retain RAM content
 **  after entering deep standby mode. Not all FM product has this function. 
 **  Plesae comfirm it in the hardware manual.
 **
 **  Lpm_WriteBackupRegisters() writes all backup registers, which can 
 **  retain in the deep standby mode. and Lpm_ReadBackupRegisters() reads 
 **  all backup regisers after waken up from deep standby mode. 
 **  
 **  Lpm_Writeu8DataBackupRegister() writes a backup regiser with 8-bit 
 **  alignment, which can retain in the deep standby mode, and 
 **  Lpm_Readu8DataBackupRegister() reads a backup regiser with 8-bit 
 **  alignment after waken up from deep standby mode.    
 **
 **  Lpm_Writeu16DataBackupRegister() writes a backup regiser with 16-bit 
 **  alignment, which can retain in the deep standby mode, and 
 **  Lpm_Readu16DataBackupRegister() reads a backup regiser with 16-bit 
 **  alignment after waken up from deep standby mode. 
 **
 **  Lpm_Writeu32DataBackupRegister() writes a backup regiser with 32-bit 
 **  alignment, which can retain in the deep standby mode, and 
 **  Lpm_Readu32DataBackupRegister() reads a backup regiser with 32-bit 
 **  alignment after waken up from deep standby mode.        
 **  
 **  Up to 32 byte backup registers can be for FM product, see product data 
 **  sheet for details.
 **
 ** \note The deep standby mode may not be available for some of FM product.
 **       Please refer to the hardware manual for the function availability.  
 **
 ******************************************************************************/
//@{

/******************************************************************************/
/* Global pre-processor symbols/macros ('#define')      
 ******************************************************************************/
#if (PDL_PERIPHERAL_VBAT_AVAILABLE == PDL_ON)   
#define PDL_BACK_UP_REGISTERS       (32u)   
#else
#define PDL_BACK_UP_REGISTERS       (16u)     
#endif   
   
#define WIFSR_WRTCI                 (1ul<<0u)
#define WIFSR_WLVDI                 (1ul<<1u)
#define WIFSR_WUI0                  (1ul<<2u)
#define WIFSR_WUI1                  (1ul<<3u)
#define WIFSR_WUI2                  (1ul<<4u)
#define WIFSR_WUI3                  (1ul<<5u)
#define WIFSR_WUI4                  (1ul<<6u)
#define WIFSR_WUI5                  (1ul<<7u)
#define WIFSR_WCEC0I                (1ul<<8u)
#define WIFSR_WCEC1I                (1ul<<9u)
#define WIFSR_WUI6                  (1ul<<10u)
#define WIFSR_WUI7                  (1ul<<11u)
#define WIFSR_WUI8                  (1ul<<12u)
#define WIFSR_WUI9                  (1ul<<13u)
#define WIFSR_WUI10                 (1ul<<14u)
#define WIFSR_WUI11                 (1ul<<15u)
  
#define WRFSR_WINITX                 (1ul<<0u)
#define WRFSR_WLVDH                  (1ul<<1u)  
     
/**
 ******************************************************************************
 ** \brief define standby mode type
 ******************************************************************************/
typedef enum en_lpm_mode
{
    StbSleepMode    = 0u,    //!< standby sleep mode
    StbTimerMode    = 1u,    //!< standby timer mode    
    StbStopMode     = 2u,    //!< standby stop mode     
    StbRtcMode      = 3u,    //!< standby RTC mode 
    DeepStbRtcMode  = 4u,    //!< deep standby RTC mode
    DeepStbStopMode = 5u,    //!< deep standby stop mode      
    
} en_lpm_mode_t;
 
/**
 ******************************************************************************
 ** \brief Deep standby mode return cause
 *******************************************************************************/
typedef struct stc_ret_cause
{
    boolean_t   bRtcEn;     //!< Return from RTC interrupt enable
    boolean_t   bLvdEn;     //!< Return from LVD interrupt enable
    boolean_t   bWakeup1En; //!< Return from Wakeup1 pin enable
    boolean_t   bWakeup2En; //!< Return from Wakeup2 pin enable
    boolean_t   bWakeup3En; //!< Return from Wakeup3 pin enable
    boolean_t   bWakeup4En; //!< Return from Wakeup4 pin enable
    boolean_t   bWakeup5En; //!< Return from Wakeup5 pin enable
    boolean_t   bCec0En;    //!< Return from CEC0 interrupt enable
    boolean_t   bCec1En;    //!< Return from CEC1 interrupt enable
#if (PDL_MCU_TYPE == PDL_FM0P_TYPE2)        
    boolean_t   bWakeup6En; //!< Return from Wakeup6 pin enable
    boolean_t   bWakeup7En; //!< Return from Wakeup7 pin enable
    boolean_t   bWakeup8En; //!< Return from Wakeup8 pin enable
    boolean_t   bWakeup9En; //!< Return from Wakeup9 pin enable
    boolean_t   bWakeup10En; //!< Return from Wakeup10 pin enable
    boolean_t   bWakeup11En; //!< Return from Wakeup10 pin enable
#endif    
}stc_dstb_ret_cause_t;

/**
 ******************************************************************************
 ** \brief define deep standby mode return cause flag
 ******************************************************************************/
typedef enum en_ret_cause_flag
{
    DeepStbNoFlag    = 0u,     ///< No flag set
    DeepStbInitx     = 1u,     ///< return from INITX input reset
    DeepStbLvdReset  = 2u,     ///< return from LVD reset
    DeepStbRtcInt    = 3u,     ///< return from RTC interrupt
    DeepStbLvdInt    = 4u,     ///< return from LVD interrupt
    DeepStbWkupPin0  = 5u,     ///< return from Wkup pin1 detection
    DeepStbWkupPin1  = 6u,     ///< return from Wkup pin1 detection
    DeepStbWkupPin2  = 7u,     ///< return from Wkup pin2 detection
    DeepStbWkupPin3  = 8u,     ///< return from Wkup pin3 detection
    DeepStbWkupPin4  = 9u,     ///< return from Wkup pin4 detection
    DeepStbWkupPin5  = 10u,    ///< return from Wkup pin5 detection
    DeepStbCec0      = 11u,    ///< return from Wkup CEC1 reception interrupt
    DeepStbCec1      = 12u,    ///< return from Wkup CEC2 reception interrupt
#if (PDL_MCU_TYPE == PDL_FM0P_TYPE2)    
    DeepStbWkupPin6  = 13u,    ///< return from Wkup pin6 detection
    DeepStbWkupPin7  = 14u,    ///< return from Wkup pin7 detection
    DeepStbWkupPin8  = 15u,    ///< return from Wkup pin8 detection
    DeepStbWkupPin9  = 16u,    ///< return from Wkup pin9 detection
    DeepStbWkupPin10  = 17u,   ///< return from Wkup pin10 detection
    DeepStbWkupPin11  = 18u,   ///< return from Wkup pin11 detection
#endif    
} en_dstb_ret_cause_t;

/**
******************************************************************************
 ** \brief define wkup pin index
******************************************************************************/
typedef enum en_dstb_wkup_pin
{
    WkupPin1 = 0u,      ///< index of wkup pin 1
    WkupPin2 = 1u,      ///< index of wkup pin 2
    WkupPin3 = 2u,      ///< index of wkup pin 3
    WkupPin4 = 3u,      ///< index of wkup pin 4
    WkupPin5 = 4u,      ///< index of wkup pin 5
#if (PDL_MCU_TYPE == PDL_FM0P_TYPE2)    
    WkupPin6 = 5u,      ///< index of wkup pin 6
    WkupPin7 = 6u,      ///< index of wkup pin 7
    WkupPin8 = 7u,      ///< index of wkup pin 8
    WkupPin9 = 8u,      ///< index of wkup pin 9
    WkupPin10 = 9u,     ///< index of wkup pin 10
    WkupPin11 = 10u,    ///< index of wkup pin 11   
#endif    
    WkupPinMax,
} en_dstb_wkup_pin_t;

/**
 ******************************************************************************
 ** \brief Valid level of wakeup pin
 ******************************************************************************/
typedef enum en_wkup_valid_level
{
    WkupLowLevelValid   = 0u,  ///< Set low level as valid level
    WkupHighLevelValid  = 1u,  ///< Set high level as valid level
    
}en_wkup_valid_level_t;

/**
 ******************************************************************************
 ** \brief Internal voltage selection in deep standby mode
 ******************************************************************************/
typedef enum en_lpm_internal_voltage
{
    LpmInternalVoltage120 = 0u, ///< Internal voltage is 1.20v in deepstandby mode. 
    LpmInternalVoltage105 = 1u, ///< Internal voltage is 1.05v in deepstandby mode. 
    LpmInternalVoltage110 = 2u, ///< Internal voltage is 1.10v in deepstandby mode. 
    
}en_lpm_internal_voltage_t;

/**
 ******************************************************************************
 ** \brief Main oscillator selection types
 ******************************************************************************/
typedef enum en_lpm_main_osc
{
    LpmMainOsc4M         = 0u, ///< Support 4MHz main oscillator in deep standby mode, has the lowest power consumption
    LpmMainOsc4M8M       = 1u, ///< Support 4MHz and 8MHz main oscillator in deep standby mode, has the middle power consumption
    LpmMainOsc4M8M16M    = 2u, ///< Support 4MHz, 8MHz, and 16MHz main oscillator in deep standby mode, has higher power consumption
    LpmMainOsc48M        = 3u, ///< Support 48MHz main oscillator in deep standby mode, has the highest power consumption
    
}en_lpm_main_osc_t;

/**
 ******************************************************************************
 ** \brief backup register index
 *******************************************************************************/
typedef enum en_dstb_bakup_reg
{
    BackupReg1 = 1u,      ///< index of backup register 1
    BackupReg2 = 2u,      ///< index of backup register 2
    BackupReg3 = 3u,      ///< index of backup register 3
    BackupReg4 = 4u,      ///< index of backup register 4
    BackupReg5 = 5u,      ///< index of backup register 5
    BackupReg6 = 6u,      ///< index of backup register 6
    BackupReg7 = 7u,      ///< index of backup register 7
    BackupReg8 = 8u,      ///< index of backup register 8
    BackupReg9 = 9u,      ///< index of backup register 9
    BackupReg10 = 10u,    ///< index of backup register 10
    BackupReg11 = 11u,    ///< index of backup register 11
    BackupReg12 = 12u,    ///< index of backup register 12
    BackupReg13 = 13u,    ///< index of backup register 13
    BackupReg14 = 14u,    ///< index of backup register 14
    BackupReg15 = 15u,    ///< index of backup register 15
    BackupReg16 = 16u,    ///< index of backup register 16
    
} en_dstb_bakup_reg_t;

/**
 ******************************************************************************
 ** \brief backup register index
 *******************************************************************************/
typedef struct stc_backupreg
{
    uint8_t u8BREG00;
    uint8_t u8BREG01;
    uint8_t u8BREG02;
    uint8_t u8BREG03;
    uint8_t u8BREG04;
    uint8_t u8BREG05;
    uint8_t u8BREG06;
    uint8_t u8BREG07;
    uint8_t u8BREG08;
    uint8_t u8BREG09;
    uint8_t u8BREG0A;
    uint8_t u8BREG0B;
    uint8_t u8BREG0C;
    uint8_t u8BREG0D;
    uint8_t u8BREG0E;
    uint8_t u8BREG0F;
    uint8_t u8BREG10;
    uint8_t u8BREG11;
    uint8_t u8BREG12;
    uint8_t u8BREG13;
    uint8_t u8BREG14;
    uint8_t u8BREG15;
    uint8_t u8BREG16;
    uint8_t u8BREG17;
    uint8_t u8BREG18;
    uint8_t u8BREG19;
    uint8_t u8BREG1A;
    uint8_t u8BREG1B;
    uint8_t u8BREG1C;
    uint8_t u8BREG1D;
    uint8_t u8BREG1E;
    uint8_t u8BREG1F;
} stc_backupreg_t;

/******************************************************************************/
/* Global variable definitions ('extern')                                     */
/******************************************************************************/

/******************************************************************************/
/* Global function prototypes (definition in C source)                        */
/******************************************************************************/
void Lpm_GoToStandByMode(en_lpm_mode_t enMode, boolean_t bIoRemain);
en_result_t Lpm_ConfigDeepStbRetCause(stc_dstb_ret_cause_t* pstcCause);
en_dstb_ret_cause_t Lpm_ReadDeepStbRetCause(void);
en_result_t Lpm_SetWkupPinLevel(en_dstb_wkup_pin_t enPinIndex, en_wkup_valid_level_t enLevel);
en_result_t Lpm_ConfigSubClk(boolean_t bSupplyCec, boolean_t bSupplyRtc);
en_result_t Lpm_ConfigDeepStbRAMRetention(boolean_t bRamRetain);
#if (PDL_MCU_TYPE == PDL_FM0P_TYPE2)
en_result_t Lpm_ConfigInternalVoltage(en_lpm_internal_voltage_t enVoltage);
en_result_t Lpm_SelMainOscTypes(en_lpm_main_osc_t enMainOsc);
en_result_t Lpm_ConfigFlashPower(boolean_t bPowerOn);
#endif
en_result_t Lpm_ReadBackupRegisters(stc_backupreg_t* stcBackUpReg) ;
en_result_t Lpm_WriteBackupRegisters(stc_backupreg_t* stcBackUpReg) ;
en_result_t Lpm_Readu8DataBackupRegister(uint8_t u8AddressOffset, uint8_t* u8Data) ;
en_result_t Lpm_Writeu8DataBackupRegister(uint8_t u8AddressOffset, uint8_t u8Data) ;
en_result_t Lpm_Readu16DataBackupRegister(uint8_t u8AddressOffset, uint16_t* u16Data) ;
en_result_t Lpm_Writeu16DataBackupRegister(uint8_t u8AddressOffset, uint16_t u16Data) ;
en_result_t Lpm_Readu32DataBackupRegister(uint8_t u8AddressOffset, uint32_t* u32Data) ;
en_result_t Lpm_Writeu32DataBackupRegister(uint8_t u8AddressOffset, uint32_t u32Data) ;

//@} // LpmGroup

#ifdef __cplusplus
}
#endif

#endif // #if (defined(PDL_PERIPHERAL_LPM_ACTIVE))

#endif /* __LPM_H__ */
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
