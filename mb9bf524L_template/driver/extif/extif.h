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
/** \file extif.h
 **
 ** Headerfile for EXTIF functions
 **  
 ** History:
 **   - 2013-04-16  1.0  MWi  First version.
 **   - 2015-01-31  1.1  EZh  Port to FM universal PDL frame.
 **
 ******************************************************************************/

#ifndef __EXTIF_H__
#define __EXTIF_H__

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_user.h"
#include "mcu.h"

#if (defined(PDL_PERIPHERAL_EXTIF_ACTIVE))

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
  
/**
 ******************************************************************************
 ** \defgroup ExtifGroup External Bus Interface Functions (EXTIF)
 **
 ** Provided functions of EXTIF module:
 ** - Extif_InitArea()
 ** - Extif_ReadErrorStatus()
 ** - Extif_ReadErrorAddress()
 ** - Extif_ClearErrorStatus()
 ** - Extif_CheckSdcmdReady()
 ** - Extif_SetSdramCommand()
 **
 ** Extif_InitArea() initializes an area according the user configuration.
 **
 ** Extif_ReadErrorStatus() reads out the error status of the Error Status
 ** Register (EST). 
 ** 
 ** Extif_ReadErrorAddress() reads out the address were the error occurred. 
 ** 
 ** Extif_ClearErrorStatus() clears the error status.
 ** 
 ** Extif_CheckSdcmdReady() checks, if it is allowed to set a SDRAM command.
 ** 
 ** Extif_SetSdramCommand() set the SDRAM command (calls
 ** Extif_CheckSdcmdReady() ).
 **
 ******************************************************************************/
//@{
  
/******************************************************************************/
/* Global pre-processor symbols/macros ('#define')                            */
/******************************************************************************/ 
#define EXTIF          (*((stc_extif_arrays_t *) FM_EXBUS_BASE))
   
/******************************************************************************
 * NAND Flash access macros (intern)
 ******************************************************************************/
#define EXITIF_NAND_ALE_OFFSET   0x00003000ul  ///< Address offset for MNALE assert until write
#define EXITIF_NAND_ADDR_OFFSET  0x00002000ul  ///< Address offset for address cycle (+ MNALE)
#define EXITIF_NAND_CMD_OFFSET   0x00001000ul  ///< Address offset for command cycle (+ MNCLE)
#define EXITIF_NAND_DATA_OFFSET  0x00000000ul  ///< Address offset for data read/write
  
/******************************************************************************
 * NAND Flash access macros (user)
 ******************************************************************************/
#define Extif_Nand_SetCommand(base, cmd)  {*(volatile unsigned char*)((base) + EXITIF_NAND_CMD_OFFSET)  = (unsigned char)(cmd);}  ///< Sets a NAND command cycle
#define Extif_Nand_SetAddress(base, addr) {*(volatile unsigned char*)((base) + EXITIF_NAND_ADDR_OFFSET) = (unsigned char)(addr);} ///< Sets a NAND address cycle
#define Extif_Nand_ReadData(base)         (*(volatile unsigned char*)((base) + EXITIF_NAND_DATA_OFFSET))                          ///< Reads NAND data
#define Extif_Nand_WriteData(base, data)  {*(volatile unsigned char*)((base) + EXITIF_NAND_DATA_OFFSET) = (unsigned char)(data);} ///< Writes NAND data
#define Extif_Nand_ClearAle(base)         {*(volatile unsigned char*)((base) + EXITIF_NAND_ALE_OFFSET)  = (unsigned char)0;}      ///< De-asserts MNALE signal
   
/******************************************************************************
 * Global type definitions
 ******************************************************************************/

/**
 ******************************************************************************
 ** \brief Definition of MODE, TIM, AREA, ATIM register arrays
 ** 
 ** To select the data bus width
 ******************************************************************************/
typedef struct stc_extif_arrays
{
    union {
        uint32_t                au32MODE[8];
        stc_exbus_mode_field_t  astcMODE[8];
    };  
    union {
        uint32_t                au32TIM[8];
        stc_exbus_tim_field_t   astcTIM[8];
    };   
    union {
        uint32_t                au32AREA[8];
        stc_exbus_area_field_t  astcAREA[8];
    };   
    union {
        uint32_t                au32ATIM[8];
        stc_exbus_atim_field_t  astcATIM[8];
    };
} stc_extif_arrays_t;

/**
 ******************************************************************************
 ** \brief Extif data width
 ** 
 ** To select the data bus width
 ******************************************************************************/
typedef enum en_extif_width
{
    Extif8Bit   = 0u,  ///< 8 Bit mode
    Extif16Bit  = 1u,  ///< 16 Bit  mode
#if (PDL_MCU_TYPE == PDL_FM4_TYPE3)    
    Extif32Bit  = 2u,  ///< 32 Bit  mode
#endif    
} en_extif_width_t;

/**
 ******************************************************************************
 ** \brief Extif cycle defintion
 ** 
 ** To select the bus mode of the interface.
 **
 ** \note Do not change enumeration number. The numbers are taken for
 **       calculating the corresponding bitfield!
 ******************************************************************************/
typedef enum en_extif_cycle
{ 
    Extif0Cycle   = 0u,  ///<  0 cycles
    Extif1Cycle   = 1u,  ///<  1 cycle
    Extif2Cycle   = 2u,  ///<  2 cycles
    Extif3Cycle   = 3u,  ///<  3 cycles
    Extif4Cycle   = 4u,  ///<  4 cycles
    Extif5Cycle   = 5u,  ///<  5 cycles
    Extif6Cycle   = 6u,  ///<  6 cycles
    Extif7Cycle   = 7u,  ///<  7 cycles
    Extif8Cycle   = 8u,  ///<  8 cycles
    Extif9Cycle   = 9u,  ///<  9 cycles
    Extif10Cycle  = 10u, ///< 10 cycles
    Extif11Cycle  = 11u, ///< 11 cycles
    Extif12Cycle  = 12u, ///< 12 cycles
    Extif13Cycle  = 13u, ///< 13 cycles
    Extif14Cycle  = 14u, ///< 14 cycles
    Extif15Cycle  = 15u, ///< 15 cycles
    Extif16Cycle  = 16u, ///< 16 cycles
    ExtifDisabled = 17u  ///< Setting disabled
} en_extif_cycle_t;

/**
 ******************************************************************************
 ** \brief SDRAM cycle defintion
 ** 
 ** To select the bus mode of the interface.
 **
 ** \note Do not change enumeration number. The numbers are taken for
 **       calculating the corresponding bitfield!
 ******************************************************************************/
typedef enum en_sdram_cycle
{ 
    Sdram1Cycle   = 0u,  ///<  1 cycle
    Sdram2Cycle   = 1u,  ///<  2 cycles
    Sdram3Cycle   = 2u,  ///<  3 cycles
    Sdram4Cycle   = 3u,  ///<  4 cycles
    Sdram5Cycle   = 4u,  ///<  5 cycles
    Sdram6Cycle   = 5u,  ///<  6 cycles
    Sdram7Cycle   = 6u,  ///<  7 cycles
    Sdram8Cycle   = 7u,  ///<  8 cycles
    Sdram9Cycle   = 8u,  ///<  9 cycles
    Sdram10Cycle  = 9u, ///< 10 cycles
    Sdram11Cycle  = 10u, ///< 11 cycles
    Sdram12Cycle  = 11u, ///< 12 cycles
    Sdram13Cycle  = 12u, ///< 13 cycles
    Sdram14Cycle  = 13u, ///< 14 cycles
    Sdram15Cycle  = 14u, ///< 15 cycles
    Sdram16Cycle  = 15u, ///< 16 cycles
    SdramDisabled = 16u  ///< Setting disabled
} en_sdram_cycle_t;

/**
 ******************************************************************************
 ** \brief Extif mask setup (area size)
 ** 
 ** To select the Mask Setup value for the address area per chip select.
 **
 ** \note Do not change enumeration number. The numbers are taken for
 **       calculating the corresponding bitfield!
 ******************************************************************************/
typedef enum en_extif_mask
{
    Extif1MB   = 0u,  ///< Area 1 MByte 
    Extif2MB   = 1u,  ///< Area 2 MByte 
    Extif4MB   = 2u,  ///< Area 4 MByte 
    Extif8MB   = 3u,  ///< Area 8 MByte 
    Extif16MB  = 4u,  ///< Area 16 MByte 
    Extif32MB  = 5u,  ///< Area 32 MByte 
    Extif64MB  = 6u,  ///< Area 64 MByte 
    Extif128MB = 7u   ///< Area 128 MByte 
} en_extif_mask_t;

/**
 ******************************************************************************
 ** \brief Extif CAS address select
 ** 
 ** Select the address for the Column Address Select
 **
 ** \note Do not change enumeration number. The numbers are taken for
 **       calculating the corresponding bitfield!
 ******************************************************************************/
typedef enum en_extif_cas
{
    ExtifCas16Bit = 0u,  ///< MAD[9:0] = Internal address [10:1], 16-Bit width
    ExtifCas32Bit = 1u   ///< MAD[9:0] = Internal address [11:2], 32-Bit width  
} en_extif_cas_t;

/**
 ******************************************************************************
 ** \brief Extif RAS address select
 ** 
 ** Select the address for the Row Address Select
 **
 ** \note Do not change enumeration number. The numbers are taken for
 **       calculating the corresponding bitfield!
 ******************************************************************************/
typedef enum en_extif_ras
{
    ExtifRas_19_6  = 0u,   ///< MAD[13:0] = Internal address [19:6]
    ExtifRas_20_7  = 1u,   ///< MAD[13:0] = Internal address [20:7]
    ExtifRas_21_8  = 2u,   ///< MAD[13:0] = Internal address [21:8]
    ExtifRas_22_9  = 3u,   ///< MAD[13:0] = Internal address [22:9]
    ExtifRas_23_10 = 4u,   ///< MAD[13:0] = Internal address [23:10]
    ExtifRas_24_11 = 5u,   ///< MAD[13:0] = Internal address [24:11]
    ExtifRas_25_12 = 6u,   ///< MAD[13:0] = Internal address [25:12]
} en_extif_ras_t;

/**
 ******************************************************************************
 ** \brief Extif BAS address select
 ** 
 ** Select the address for the Bank Address Select
 **
 ** \note Do not change enumeration number. The numbers are taken for
 **       calculating the corresponding bitfield!
 ******************************************************************************/
typedef enum en_extif_bas
{
    ExtifBas_20_19 = 0u,   ///< MAD[13:0] = Internal address [20:19]
    ExtifBas_21_20 = 1u,   ///< MAD[13:0] = Internal address [21:20]
    ExtifBas_22_21 = 2u,   ///< MAD[13:0] = Internal address [22:21]
    ExtifBas_23_22 = 3u,   ///< MAD[13:0] = Internal address [23:22]
    ExtifBas_24_23 = 4u,   ///< MAD[13:0] = Internal address [24:23]
    ExtifBas_25_24 = 5u,   ///< MAD[13:0] = Internal address [25:24]
    ExtifBas_26_25 = 6u,   ///< MAD[13:0] = Internal address [26:25]
} en_extif_bas_t;

typedef struct stc_extif_sdram_config
{
    boolean_t         bSdramEnable;              ///< TRUE: Enables SDRAM functionality (only possible for area 8)
    boolean_t         bSdramPowerDownMode;       ///< TRUE: Enables SDRAM Power Down Mode (only possible for area 8)
    boolean_t         bSdramRefreshOff;          ///< TRUE: Disables Refresh Function (only possible for area 8)
    en_extif_cas_t    enCasel;                   ///< Column Address Select, see #en_extif_cas_t for details (only possible for area 8)
    en_extif_ras_t    enRasel;                   ///< Row Address Select, see #en_extif_ras_t for details (only possible for area 8)
    en_extif_bas_t    enBasel;                   ///< Bank Address Select, see #en_extif_bas_t for details (only possible for area 8)
    uint16_t          u16RefreshCount;           ///< Refresh Count in Cycles (only possible for area 8)
    uint8_t           u8RefreshNumber;           ///< Number of Refreshs (only possible for area 8)
    boolean_t         bPreRefreshEnable;         ///< TRUE: Enables Refresh (only possible for area 8)
    uint16_t          u16PowerDownCount;         ///< Power Down Count in Cycles (only possible for area 8)
    en_sdram_cycle_t  enSdramCasLatencyCycle;    ///< Latency of CAS in Cycles (only possible for area 8)
    en_sdram_cycle_t  enSdramRasCycle;           ///< RAS Cycles (only possible for area 8)
    en_sdram_cycle_t  enSdramRasPrechargeCycle;  ///< RAS Precharge Cycles (only possible for area 8)
    en_sdram_cycle_t  enSdramRasCasDelayCycle;   ///< RAS CAS Delay Cycles (only possible for area 8)
    en_sdram_cycle_t  enSdramRasActiveCycle;     ///< RAS Active Cycles (only possible for area 8)
    en_sdram_cycle_t  enSdramRefreshCycle;       ///< Refresh Cycles (only possible for area 8)
    en_sdram_cycle_t  enSdramPrechargeCycle;     ///< Data-in to Precharge Lead Time in Cycles (only possible for area 8)
    boolean_t         bSdramErrorInterruptEnable; ///< TRUE: Enables SDRAM error interrupt (only possible for area 8)
    func_ptr_t        pfnSdramErrorCallback;     ///< Pointer to SDRAM Error Callback Function (only possible for area 8)
   
}stc_extif_sdram_config_t;

/**
 *****************************************************************************
 ** \brief Extif area configuration
 ** 
 ** The WDG configuration is done on a per area (Chip select) basis
 *****************************************************************************/
typedef struct stc_extif_area_config
{
    en_extif_width_t  enWidth;                   ///< 8, 16 bit data bus width. See description of #en_extif_width_t
    boolean_t         bReadByteMask;             ///< TRUE: Read Byte Mask enable
    boolean_t         bWriteEnableOff;           ///< TRUE: Write enable disabled
    boolean_t         bNandFlash;                ///< TRUE: NAND Flash bus enable, FLASE: NOR Flash/SRAM bus enable
    boolean_t         bPageAccess;               ///< TRUE: NOR Flash memory page access mode enabled
#if (PDL_MCU_TYPE != PDL_FM3_TYPE0)    
    boolean_t         bRdyOn;                    ///< TRUE: RDY mode enabled
    boolean_t         bStopDataOutAtFirstIdle;   ///< TRUE: Stop to write data output at first idle cycle, FALSE: Extends to write data output to the last idle cycle
    boolean_t         bMultiplexMode;            ///< TRUE: Multiplex mode
    boolean_t         bAleInvert;                ///< TRUE: Invert ALE signal (negative polarity)
    boolean_t         bAddrOnDataLinesOff;       ///< TRUE: Do not output address to data lines (Hi-Z during ALC cycle period)
    boolean_t         bMpxcsOff;                 ///< TRUE: Do not assert MCSX in ALC cycle period
    boolean_t         bMoexWidthAsFradc;         ///< TRUE: MOEX width is set with FRADC, FALSE: MOEX width is set with RACC-RADC
    boolean_t         bMclkoutEnable;            ///< TRUE: Enables MCLKOUT pin
    uint8_t           u8MclkDivision;            ///< Division ratio for MCLK (1 ... 16 div)
#endif    
    en_extif_cycle_t  enReadAccessCycle;         ///< Read Access Cycle timing
    en_extif_cycle_t  enReadAddressSetupCycle;   ///< Read Address Setup Cycle timing
    en_extif_cycle_t  enFirstReadAddressCycle;   ///< First Read Address Cycle timing
    en_extif_cycle_t  enReadIdleCycle;           ///< Read Idle Cycle timing
    en_extif_cycle_t  enWriteAccessCycle;        ///< Write Access Cycle timing
    en_extif_cycle_t  enWriteAddressSetupCycle;  ///< Write Address Setup Cycle timing
    en_extif_cycle_t  enWriteEnableCycle;        ///< Write Enable Cycle timing
    en_extif_cycle_t  enWriteIdleCycle;          ///< Write Idle Cycle timing
    uint8_t           u8AreaAddress;             ///< Address bits [27:20]
    en_extif_mask_t   enAreaMask;                ///< See description of #en_extif_mask_t
    en_extif_cycle_t  enAddressLatchCycle;       ///< Address Latch Cycles
    en_extif_cycle_t  enAddressLatchSetupCycle;  ///< Address Latch Enable Setup Cycles
    en_extif_cycle_t  enAddressLatchWidthCycle;  ///< Address Latch Enable Width Cycles
#if (PDL_MCU_CORE == PDL_FM4_CORE)
    boolean_t         bPrecedReadContinuousWrite; ///< TRUE: Enables preceding read and continuous write request
    stc_extif_sdram_config_t* pExtifSdramConfig;  ///< Pointer to the configuration of SDRAM I/F of external bus interface 
#if (PDL_MCU_TYPE == PDL_FM4_TYPE4)    
    stc_extif_sdram_config_t* pGdcSdramConfig;    ///< Pointer to the configuration of SDRAM I/F of GDC 
#endif    
    boolean_t         bSramFlashErrorInterruptEnable; ///< TRUE: Enables SRAM/Flash Error Interrupt 
    func_ptr_t        pfnSramFlashErrorCallback; ///< Pointer to SRAM/Flash Error Callback Function
    
#endif     
} stc_extif_area_config_t;
  
/// EXTIF Error Interrupt Callback Pointers
typedef struct stc_extif_intern_data
{
  boolean_t         bSdramErrorInterruptEnable;        ///< TRUE: Enables SDRAM error interrupt
  boolean_t         bSramFlashErrorInterruptEnable;    ///< TRUE: Enables SRAM/Flash Error Interrupt
#if (PDL_MCU_TYPE == PDL_FM4_TYPE4)    
  boolean_t         bGdcSdramErrorInterruptEnable;     ///< TRUE: Enables GDC SDRAM Error Interrupt
#endif  
  func_ptr_t        pfnSdramErrorCallback;          ///< Pointer to SDRAM Error Callback Function
  func_ptr_t        pfnSramFlashErrorCallback;      ///< Pointer to SRAM/Flash Error Callback Function
#if (PDL_MCU_TYPE == PDL_FM4_TYPE4)      
  func_ptr_t        pfnGdcSdramErrorCallback;       ///< Pointer to GDC SDRAM Error Callback Function
#endif  
} stc_extif_intern_data_t;

/******************************************************************************/
/* Global function prototypes ('extern', definition in C source)              */
/******************************************************************************/

void        Extif_IrqHandler( void );

en_result_t Extif_InitArea( uint8_t                  u8Area,
                            stc_extif_area_config_t* pstcConfig );

en_result_t Extif_ReadErrorStatus( void );

uint32_t    Extif_ReadErrorAddress( void );

en_result_t Extif_ClearErrorStatus( void );

en_result_t Extif_CheckSdcmdReady( void );

en_result_t Extif_SetSdramCommand( uint16_t  u16Address,
                                   boolean_t bMsdwex,
                                   boolean_t bMcasx,
                                   boolean_t bMrasx,
                                   boolean_t bMcsx8,
                                   boolean_t bMadcke
                                 );

//@} // ExtifGroup

#ifdef __cplusplus
}
#endif

#endif // #if (defined(PDL_PERIPHERAL_EXTIF_ACTIVE))

#endif /* __EXTIF_H__ */
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
