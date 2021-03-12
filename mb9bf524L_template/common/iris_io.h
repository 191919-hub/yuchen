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
/* IRIS SDL2/MDL2 Registers Declaration                                       */
/*                                                        2014-05-09 14:00:00 */
/******************************************************************************/

#ifndef _IRIS_H_
#define _IRIS_H_

#ifdef __cplusplus
extern "C" {
#endif 

#ifndef __IO
#define __IO volatile
#endif
#include "stdint.h"

/******************************************************************************/
/*                Device Specific Peripheral Registers structures             */
/******************************************************************************/

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif


/******************************************************************************
 * IRIS_SDL2 register bit fields
 ******************************************************************************/
/* ----[ SubSysCtrl ]-------------------------------------------------------------------- */
/* LockUnlock */
typedef struct SubSysCtrl_LockUnlock {
    __IO  uint32_t LockUnlock                              : 32;    /* W    */  /* 0x0 */
} SubSysCtrl_LockUnlock_t;

/* LockStatus */
typedef struct SubSysCtrl_LockStatus {
    __IO  uint32_t LockStatus                              :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t PrivilegeStatus                         :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t FreezeStatus                            :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} SubSysCtrl_LockStatus_t;

/* IPIdentifier */
typedef struct SubSysCtrl_IPIdentifier {
          uint32_t RESERVED_01                             :  4;
    __IO  uint32_t DesignDeliveryID                        :  4;    /* RW   */  /* 0x1 */
    __IO  uint32_t DesignMaturityLevel                     :  4;    /* RW   */  /* 0x4 */
    __IO  uint32_t IPEvolution                             :  4;    /* RW   */  /* 0x1 */
    __IO  uint32_t IPFeatureSet                            :  4;    /* RW   */  /* 0x4 */
    __IO  uint32_t IPApplication                           :  4;    /* RW   */  /* 0x4 */
    __IO  uint32_t IPConfiguration                         :  4;    /* RW   */  /* 0x2 */
    __IO  uint32_t IPFamily                                :  4;    /* RW   */  /* 0x2 */
} SubSysCtrl_IPIdentifier_t;

/* ConfigClockControl */
typedef struct SubSysCtrl_ConfigClockControl {
    __IO  uint32_t ConfigClockSelect                       :  3;    /* RW   */  /* 0x1 */
          uint32_t RESERVED_02                             : 29;
} SubSysCtrl_ConfigClockControl_t;

/* ConfigMemorySelect */
typedef struct SubSysCtrl_ConfigMemorySelect {
    __IO  uint32_t ConfigMemorySelect                      :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             : 31;
} SubSysCtrl_ConfigMemorySelect_t;

/* ConfigVramRemap */
typedef struct SubSysCtrl_ConfigVramRemap {
    __IO  uint32_t ConfigVramRemap                         :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             : 31;
} SubSysCtrl_ConfigVramRemap_t;

/* ConfigPanicSwitch */
typedef struct SubSysCtrl_ConfigPanicSwitch {
    __IO  uint32_t ConfigPanicSwitch                       :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             : 31;
} SubSysCtrl_ConfigPanicSwitch_t;

/* dsp_LockUnlock */
typedef struct SubSysCtrl_dsp_LockUnlock {
    __IO  uint32_t dsp_LockUnlock                          : 32;    /* W    */  /* 0x0 */
} SubSysCtrl_dsp_LockUnlock_t;

/* dsp_LockStatus */
typedef struct SubSysCtrl_dsp_LockStatus {
    __IO  uint32_t dsp_LockStatus                          :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t dsp_PrivilegeStatus                     :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t dsp_FreezeStatus                        :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} SubSysCtrl_dsp_LockStatus_t;

/* dsp0_ClockDivider */
typedef struct SubSysCtrl_dsp0_ClockDivider {
          uint32_t RESERVED_01                             :  8;
    __IO  uint32_t dsp0_ClockDivider                       : 16;    /* RW   */  /* 0x41e0 */
          uint32_t RESERVED_03                             :  8;
} SubSysCtrl_dsp0_ClockDivider_t;

/* dsp0_DomainControl */
typedef struct SubSysCtrl_dsp0_DomainControl {
    __IO  uint32_t dsp0_ClockEnable                        :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             : 15;
    __IO  uint32_t dsp0_SoftwareReset                      :  1;    /* RW   */  /* 0x1 */
          uint32_t RESERVED_04                             : 15;
} SubSysCtrl_dsp0_DomainControl_t;

/* dsp0_ClockShift */
typedef struct SubSysCtrl_dsp0_ClockShift {
    __IO  uint32_t dsp0_ClockInvert                        :  1;    /* RW   */  /* 0x1 */
          uint32_t RESERVED_02                             : 15;
    __IO  uint32_t dsp0_ClockOffset                        :  8;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_04                             :  8;
} SubSysCtrl_dsp0_ClockShift_t;

/* dsp0_LineEndControl */
typedef struct SubSysCtrl_dsp0_LineEndControl {
    __IO  uint32_t LineEnd_Delay                           :  7;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             :  9;
    __IO  uint32_t LineEnd_Enable                          :  1;    /* RW   */  /* 0x1 */
          uint32_t RESERVED_04                             : 15;
} SubSysCtrl_dsp0_LineEndControl_t;

/* dsp0_PowerEnControl */
typedef struct SubSysCtrl_dsp0_PowerEnControl {
    __IO  uint32_t Power_Enable                            :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             : 31;
} SubSysCtrl_dsp0_PowerEnControl_t;

/* SDRAMC_ClockDivider */
typedef struct SubSysCtrl_SDRAMC_ClockDivider {
          uint32_t RESERVED_01                             :  8;
    __IO  uint32_t SDRAMC_ClockDivider                     : 16;    /* RW   */  /* 0x400 */
          uint32_t RESERVED_03                             :  8;
} SubSysCtrl_SDRAMC_ClockDivider_t;

/* SDRAMC_DomainControl */
typedef struct SubSysCtrl_SDRAMC_DomainControl {
    __IO  uint32_t SDRAMC_ClockEnable                      :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             : 15;
    __IO  uint32_t SDRAMC_SoftwareReset                    :  1;    /* RW   */  /* 0x1 */
          uint32_t RESERVED_04                             : 15;
} SubSysCtrl_SDRAMC_DomainControl_t;

/* HSSPIC_ClockDivider */
typedef struct SubSysCtrl_HSSPIC_ClockDivider {
          uint32_t RESERVED_01                             :  8;
    __IO  uint32_t HSSPIC_ClockDivider                     : 16;    /* RW   */  /* 0x400 */
          uint32_t RESERVED_03                             :  8;
} SubSysCtrl_HSSPIC_ClockDivider_t;

/* HSSPIC_DomainControl */
typedef struct SubSysCtrl_HSSPIC_DomainControl {
    __IO  uint32_t HSSPIC_ClockEnable                      :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             : 15;
    __IO  uint32_t HSSPIC_SoftwareReset                    :  1;    /* RW   */  /* 0x1 */
          uint32_t RESERVED_04                             : 15;
} SubSysCtrl_HSSPIC_DomainControl_t;

/* RPCC_ClockDivider */
typedef struct SubSysCtrl_RPCC_ClockDivider {
    __IO  uint32_t RPCC_ClockDivider                       :  3;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             : 29;
} SubSysCtrl_RPCC_ClockDivider_t;

/* RPCC_DomainControl */
typedef struct SubSysCtrl_RPCC_DomainControl {
    __IO  uint32_t RPCC_ClockEnable                        :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             : 31;
} SubSysCtrl_RPCC_DomainControl_t;

/* AXI_ClockDivider */
typedef struct SubSysCtrl_AXI_ClockDivider {
          uint32_t RESERVED_01                             :  8;
    __IO  uint32_t AXIClockSelect                          : 16;    /* RW   */  /* 0x0400 */
          uint32_t RESERVED_03                             :  8;
} SubSysCtrl_AXI_ClockDivider_t;

/* vram_LockUnlock */
typedef struct SubSysCtrl_vram_LockUnlock {
    __IO  uint32_t vram_LockUnlock                         : 32;    /* W    */  /* 0x0 */
} SubSysCtrl_vram_LockUnlock_t;

/* vram_LockStatus */
typedef struct SubSysCtrl_vram_LockStatus {
    __IO  uint32_t vram_LockStatus                         :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t vram_PrivilegeStatus                    :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t vram_FreezeStatus                       :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} SubSysCtrl_vram_LockStatus_t;

/* vram_arbiter_priority */
typedef struct SubSysCtrl_vram_arbiter_priority {
    __IO  uint32_t vram_priority_s0_write                  :  2;    /* RW   */  /* 0x0 */
    __IO  uint32_t vram_priority_s0_read                   :  2;    /* RW   */  /* 0x0 */
    __IO  uint32_t vram_priority_s1_read                   :  2;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_04                             : 26;
} SubSysCtrl_vram_arbiter_priority_t;

/******************************************************************************
 * IRIS_MDL2 register bit fields
 ******************************************************************************/
/* ----[ ComCtrl ]----------------------------------------------------------------------- */
/* IPIdentifier */
typedef struct ComCtrl_IPIdentifier {
          uint32_t RESERVED_01                             :  4;
    __IO  uint32_t DesignDeliveryID                        :  4;    /* RW   */  /* 0x1 */
    __IO  uint32_t DesignMaturityLevel                     :  4;    /* RW   */  /* 0x4 */
    __IO  uint32_t IPEvolution                             :  4;    /* RW   */  /* 0x2 */
    __IO  uint32_t IPFeatureSet                            :  4;    /* RW   */  /* 0x2 */
    __IO  uint32_t IPApplication                           :  4;    /* RW   */  /* 0x2 */
    __IO  uint32_t IPConfiguration                         :  4;    /* RW   */  /* 0x1 */
    __IO  uint32_t IPFamily                                :  4;    /* RW   */  /* 0x2 */
} ComCtrl_IPIdentifier_t;

/* LockUnlock */
typedef struct ComCtrl_LockUnlock {
    __IO  uint32_t LockUnlock                              : 32;    /* W    */  /* X */
} ComCtrl_LockUnlock_t;

/* LockStatus */
typedef struct ComCtrl_LockStatus {
    __IO  uint32_t LockStatus                              :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t PrivilegeStatus                         :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t FreezeStatus                            :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} ComCtrl_LockStatus_t;

/* UserInterruptMask0 */
typedef struct ComCtrl_UserInterruptMask0 {
    __IO  uint32_t UserInterruptMask0                      : 29;    /* RW   */  /* 0x1fffffff */
          uint32_t RESERVED_02                             :  3;
} ComCtrl_UserInterruptMask0_t;

/* InterruptEnable0 */
typedef struct ComCtrl_InterruptEnable0 {
    __IO  uint32_t InterruptEnable0                        : 29;    /* RWX  */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
} ComCtrl_InterruptEnable0_t;

/* InterruptPreset0 */
typedef struct ComCtrl_InterruptPreset0 {
    __IO  uint32_t InterruptPreset0                        : 29;    /* W1P  */  /* X */
          uint32_t RESERVED_02                             :  3;
} ComCtrl_InterruptPreset0_t;

/* InterruptClear0 */
typedef struct ComCtrl_InterruptClear0 {
    __IO  uint32_t InterruptClear0                         : 29;    /* W1P  */  /* X */
          uint32_t RESERVED_02                             :  3;
} ComCtrl_InterruptClear0_t;

/* InterruptStatus0 */
typedef struct ComCtrl_InterruptStatus0 {
    __IO  uint32_t InterruptStatus0                        : 29;    /* R    */  /* X */
          uint32_t RESERVED_02                             :  3;
} ComCtrl_InterruptStatus0_t;

/* UserInterruptEnable0 */
typedef struct ComCtrl_UserInterruptEnable0 {
    __IO  uint32_t UserInterruptEnable0                    : 29;    /* RWX  */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
} ComCtrl_UserInterruptEnable0_t;

/* UserInterruptPreset0 */
typedef struct ComCtrl_UserInterruptPreset0 {
    __IO  uint32_t UserInterruptPreset0                    : 29;    /* W1P  */  /* X */
          uint32_t RESERVED_02                             :  3;
} ComCtrl_UserInterruptPreset0_t;

/* UserInterruptClear0 */
typedef struct ComCtrl_UserInterruptClear0 {
    __IO  uint32_t UserInterruptClear0                     : 29;    /* W1P  */  /* X */
          uint32_t RESERVED_02                             :  3;
} ComCtrl_UserInterruptClear0_t;

/* UserInterruptStatus0 */
typedef struct ComCtrl_UserInterruptStatus0 {
    __IO  uint32_t UserInterruptStatus0                    : 29;    /* R    */  /* X */
          uint32_t RESERVED_02                             :  3;
} ComCtrl_UserInterruptStatus0_t;

/* GeneralPurpose */
typedef struct ComCtrl_GeneralPurpose {
    __IO  uint32_t GeneralPurpose                          : 32;    /* RW   */  /* 0x0 */
} ComCtrl_GeneralPurpose_t;

/* ----[ CmdSeq ]------------------------------------------------------------------------ */
/* HIF */
typedef struct CmdSeq_HIF {
    __IO  uint32_t HIF                                     : 32;    /* W    */  /* X */
} CmdSeq_HIF_t;

/* LockUnlockHIF */
typedef struct CmdSeq_LockUnlockHIF {
    __IO  uint32_t LockUnlockHIF                           : 32;    /* W    */  /* X */
} CmdSeq_LockUnlockHIF_t;

/* LockStatusHIF */
typedef struct CmdSeq_LockStatusHIF {
    __IO  uint32_t LockStatusHIF                           :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t PrivilegeStatusHIF                      :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t FreezeStatusHIF                         :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} CmdSeq_LockStatusHIF_t;

/* LockUnlock */
typedef struct CmdSeq_LockUnlock {
    __IO  uint32_t LockUnlock                              : 32;    /* W    */  /* X */
} CmdSeq_LockUnlock_t;

/* LockStatus */
typedef struct CmdSeq_LockStatus {
    __IO  uint32_t LockStatus                              :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t PrivilegeStatus                         :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t FreezeStatus                            :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} CmdSeq_LockStatus_t;

/* BufferAddress */
typedef struct CmdSeq_BufferAddress {
    __IO  uint32_t Local                                   :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             :  4;
    __IO  uint32_t Addr                                    : 27;    /* RW   */  /* 0x0 */
} CmdSeq_BufferAddress_t;

/* BufferSize */
typedef struct CmdSeq_BufferSize {
          uint32_t RESERVED_01                             :  3;
    __IO  uint32_t Size                                    : 13;    /* RW   */  /* 0x10 */
          uint32_t RESERVED_03                             : 16;
} CmdSeq_BufferSize_t;

/* WatermarkControl */
typedef struct CmdSeq_WatermarkControl {
    __IO  uint32_t LowWM                                   : 16;    /* RW   */  /* 0x20 */
    __IO  uint32_t HighWM                                  : 16;    /* RW   */  /* 0x60 */
} CmdSeq_WatermarkControl_t;

/* Control */
typedef struct CmdSeq_Control {
    __IO  uint32_t ClrAxiw                                 :  1;    /* W1P  */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
    __IO  uint32_t ClrRbuf                                 :  1;    /* W1P  */  /* 0x0 */
    __IO  uint32_t ClrCmdBuf                               :  1;    /* W1P  */  /* 0x0 */
          uint32_t RESERVED_05                             : 27;
    __IO  uint32_t Clear                                   :  1;    /* W1P  */  /* 0x0 */
} CmdSeq_Control_t;

/* Status */
typedef struct CmdSeq_Status {
    __IO  uint32_t FIFOSpace                               : 17;    /* R    */  /* 0x80 */
          uint32_t RESERVED_02                             :  7;
    __IO  uint32_t FIFOEmpty                               :  1;    /* R    */  /* 0x1 */
    __IO  uint32_t FIFOFull                                :  1;    /* R    */  /* 0x0 */
    __IO  uint32_t FIFOWMState                             :  1;    /* R    */  /* 0x0 */
    __IO  uint32_t Watchdog                                :  1;    /* R    */  /* 0x0 */
    __IO  uint32_t ReadBusy                                :  1;    /* R    */  /* 0x0 */
    __IO  uint32_t WriteBusy                               :  1;    /* R    */  /* 0x0 */
    __IO  uint32_t Idle                                    :  1;    /* R    */  /* 0x1 */
    __IO  uint32_t ErrorHalt                               :  1;    /* R    */  /* 0x0 */
} CmdSeq_Status_t;

/* ----[ PixEngCfg ]--------------------------------------------------------------------- */
/* SafetyLockUnlock */
typedef struct PixEngCfg_SafetyLockUnlock {
    __IO  uint32_t SafetyLockUnlock                        : 32;    /* W    */  /* X */
} PixEngCfg_SafetyLockUnlock_t;

/* SafetyLockStatus */
typedef struct PixEngCfg_SafetyLockStatus {
    __IO  uint32_t SafetyLockStatus                        :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t SafetyPrivilegeStatus                   :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t SafetyFreezeStatus                      :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} PixEngCfg_SafetyLockStatus_t;

/* store9_SafetyMask */
typedef struct PixEngCfg_store9_SafetyMask {
    __IO  uint32_t store9_SafetyMask                       : 14;    /* RW   */  /* 0x3fff */
          uint32_t RESERVED_02                             : 18;
} PixEngCfg_store9_SafetyMask_t;

/* extdst0_SafetyMask */
typedef struct PixEngCfg_extdst0_SafetyMask {
    __IO  uint32_t extdst0_SafetyMask                      : 14;    /* RW   */  /* 0x3fff */
          uint32_t RESERVED_02                             : 18;
} PixEngCfg_extdst0_SafetyMask_t;

/* extdst4_SafetyMask */
typedef struct PixEngCfg_extdst4_SafetyMask {
    __IO  uint32_t extdst4_SafetyMask                      : 14;    /* RW   */  /* 0x3fff */
          uint32_t RESERVED_02                             : 18;
} PixEngCfg_extdst4_SafetyMask_t;

/* fetchdecode9_LockUnlock */
typedef struct PixEngCfg_fetchdecode9_LockUnlock {
    __IO  uint32_t fetchdecode9_LockUnlock                 : 32;    /* W    */  /* X */
} PixEngCfg_fetchdecode9_LockUnlock_t;

/* fetchdecode9_LockStatus */
typedef struct PixEngCfg_fetchdecode9_LockStatus {
    __IO  uint32_t fetchdecode9_LockStatus                 :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t fetchdecode9_PrivilegeStatus            :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t fetchdecode9_FreezeStatus               :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} PixEngCfg_fetchdecode9_LockStatus_t;

/* fetchdecode9_Dynamic */
typedef struct PixEngCfg_fetchdecode9_Dynamic {
    __IO  uint32_t fetchdecode9_src_sel                    :  5;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             : 27;
} PixEngCfg_fetchdecode9_Dynamic_t;

/* fetchdecode9_Status */
typedef struct PixEngCfg_fetchdecode9_Status {
          uint32_t RESERVED_01                             : 16;
    __IO  uint32_t fetchdecode9_sel                        :  2;    /* R    */  /* X */
          uint32_t RESERVED_03                             : 14;
} PixEngCfg_fetchdecode9_Status_t;

/* fetchrot9_LockUnlock */
typedef struct PixEngCfg_fetchrot9_LockUnlock {
    __IO  uint32_t fetchrot9_LockUnlock                    : 32;    /* W    */  /* X */
} PixEngCfg_fetchrot9_LockUnlock_t;

/* fetchrot9_LockStatus */
typedef struct PixEngCfg_fetchrot9_LockStatus {
    __IO  uint32_t fetchrot9_LockStatus                    :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t fetchrot9_PrivilegeStatus               :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t fetchrot9_FreezeStatus                  :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} PixEngCfg_fetchrot9_LockStatus_t;

/* fetchrot9_Dynamic */
typedef struct PixEngCfg_fetchrot9_Dynamic {
    __IO  uint32_t fetchrot9_src_sel                       :  5;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             : 27;
} PixEngCfg_fetchrot9_Dynamic_t;

/* fetchrot9_Status */
typedef struct PixEngCfg_fetchrot9_Status {
          uint32_t RESERVED_01                             : 16;
    __IO  uint32_t fetchrot9_sel                           :  2;    /* R    */  /* X */
          uint32_t RESERVED_03                             : 14;
} PixEngCfg_fetchrot9_Status_t;

/* fetcheco9_LockUnlock */
typedef struct PixEngCfg_fetcheco9_LockUnlock {
    __IO  uint32_t fetcheco9_LockUnlock                    : 32;    /* W    */  /* X */
} PixEngCfg_fetcheco9_LockUnlock_t;

/* fetcheco9_LockStatus */
typedef struct PixEngCfg_fetcheco9_LockStatus {
    __IO  uint32_t fetcheco9_LockStatus                    :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t fetcheco9_PrivilegeStatus               :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t fetcheco9_FreezeStatus                  :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} PixEngCfg_fetcheco9_LockStatus_t;

/* fetcheco9_Status */
typedef struct PixEngCfg_fetcheco9_Status {
          uint32_t RESERVED_01                             : 16;
    __IO  uint32_t fetcheco9_sel                           :  2;    /* R    */  /* X */
          uint32_t RESERVED_03                             : 14;
} PixEngCfg_fetcheco9_Status_t;

/* rop9_LockUnlock */
typedef struct PixEngCfg_rop9_LockUnlock {
    __IO  uint32_t rop9_LockUnlock                         : 32;    /* W    */  /* X */
} PixEngCfg_rop9_LockUnlock_t;

/* rop9_LockStatus */
typedef struct PixEngCfg_rop9_LockStatus {
    __IO  uint32_t rop9_LockStatus                         :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t rop9_PrivilegeStatus                    :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t rop9_FreezeStatus                       :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} PixEngCfg_rop9_LockStatus_t;

/* rop9_Dynamic */
typedef struct PixEngCfg_rop9_Dynamic {
    __IO  uint32_t rop9_prim_sel                           :  5;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t rop9_sec_sel                            :  5;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t rop9_tert_sel                           :  5;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_06                             :  3;
    __IO  uint32_t rop9_clken                              :  2;    /* RWS  */  /* 0x1 */
          uint32_t RESERVED_08                             :  6;
} PixEngCfg_rop9_Dynamic_t;

/* rop9_Status */
typedef struct PixEngCfg_rop9_Status {
          uint32_t RESERVED_01                             : 16;
    __IO  uint32_t rop9_sel                                :  2;    /* R    */  /* X */
          uint32_t RESERVED_03                             : 14;
} PixEngCfg_rop9_Status_t;

/* clut9_LockUnlock */
typedef struct PixEngCfg_clut9_LockUnlock {
    __IO  uint32_t clut9_LockUnlock                        : 32;    /* W    */  /* X */
} PixEngCfg_clut9_LockUnlock_t;

/* clut9_LockStatus */
typedef struct PixEngCfg_clut9_LockStatus {
    __IO  uint32_t clut9_LockStatus                        :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t clut9_PrivilegeStatus                   :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t clut9_FreezeStatus                      :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} PixEngCfg_clut9_LockStatus_t;

/* clut9_Dynamic */
typedef struct PixEngCfg_clut9_Dynamic {
    __IO  uint32_t clut9_src_sel                           :  5;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             : 27;
} PixEngCfg_clut9_Dynamic_t;

/* clut9_Status */
typedef struct PixEngCfg_clut9_Status {
          uint32_t RESERVED_01                             : 16;
    __IO  uint32_t clut9_sel                               :  2;    /* R    */  /* X */
          uint32_t RESERVED_03                             : 14;
} PixEngCfg_clut9_Status_t;

/* matrix9_LockUnlock */
typedef struct PixEngCfg_matrix9_LockUnlock {
    __IO  uint32_t matrix9_LockUnlock                      : 32;    /* W    */  /* X */
} PixEngCfg_matrix9_LockUnlock_t;

/* matrix9_LockStatus */
typedef struct PixEngCfg_matrix9_LockStatus {
    __IO  uint32_t matrix9_LockStatus                      :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t matrix9_PrivilegeStatus                 :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t matrix9_FreezeStatus                    :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} PixEngCfg_matrix9_LockStatus_t;

/* matrix9_Dynamic */
typedef struct PixEngCfg_matrix9_Dynamic {
    __IO  uint32_t matrix9_src_sel                         :  5;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             : 19;
    __IO  uint32_t matrix9_clken                           :  2;    /* RWS  */  /* 0x1 */
          uint32_t RESERVED_04                             :  6;
} PixEngCfg_matrix9_Dynamic_t;

/* matrix9_Status */
typedef struct PixEngCfg_matrix9_Status {
          uint32_t RESERVED_01                             : 16;
    __IO  uint32_t matrix9_sel                             :  2;    /* R    */  /* X */
          uint32_t RESERVED_03                             : 14;
} PixEngCfg_matrix9_Status_t;

/* blitblend9_LockUnlock */
typedef struct PixEngCfg_blitblend9_LockUnlock {
    __IO  uint32_t blitblend9_LockUnlock                   : 32;    /* W    */  /* X */
} PixEngCfg_blitblend9_LockUnlock_t;

/* blitblend9_LockStatus */
typedef struct PixEngCfg_blitblend9_LockStatus {
    __IO  uint32_t blitblend9_LockStatus                   :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t blitblend9_PrivilegeStatus              :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t blitblend9_FreezeStatus                 :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} PixEngCfg_blitblend9_LockStatus_t;

/* blitblend9_Dynamic */
typedef struct PixEngCfg_blitblend9_Dynamic {
    __IO  uint32_t blitblend9_prim_sel                     :  5;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t blitblend9_sec_sel                      :  5;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             : 11;
    __IO  uint32_t blitblend9_clken                        :  2;    /* RWS  */  /* 0x1 */
          uint32_t RESERVED_06                             :  6;
} PixEngCfg_blitblend9_Dynamic_t;

/* blitblend9_Status */
typedef struct PixEngCfg_blitblend9_Status {
          uint32_t RESERVED_01                             : 16;
    __IO  uint32_t blitblend9_sel                          :  2;    /* R    */  /* X */
          uint32_t RESERVED_03                             : 14;
} PixEngCfg_blitblend9_Status_t;

/* store9_LockUnlock */
typedef struct PixEngCfg_store9_LockUnlock {
    __IO  uint32_t store9_LockUnlock                       : 32;    /* W    */  /* X */
} PixEngCfg_store9_LockUnlock_t;

/* store9_LockStatus */
typedef struct PixEngCfg_store9_LockStatus {
    __IO  uint32_t store9_LockStatus                       :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t store9_PrivilegeStatus                  :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t store9_FreezeStatus                     :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} PixEngCfg_store9_LockStatus_t;

/* store9_Static */
typedef struct PixEngCfg_store9_Static {
    __IO  uint32_t store9_ShdEn                            :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t store9_powerdown                        :  1;    /* RW   */  /* 0x1 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t store9_Sync_Mode                        :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_06                             :  2;
    __IO  uint32_t store9_SW_Reset                         :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_08                             :  4;
    __IO  uint32_t store9_div                              :  8;    /* RW   */  /* 0x80 */
          uint32_t RESERVED_10                             :  8;
} PixEngCfg_store9_Static_t;

/* store9_Dynamic */
typedef struct PixEngCfg_store9_Dynamic {
    __IO  uint32_t store9_src_sel                          :  5;    /* RWS  */  /* 0x1 */
          uint32_t RESERVED_02                             : 27;
} PixEngCfg_store9_Dynamic_t;

/* store9_Request */
typedef struct PixEngCfg_store9_Request {
    __IO  uint32_t store9_sel_ShdLdReq                     :  1;    /* RWX  */  /* 0x0 */
    __IO  uint32_t store9_ShdLdReq                         :  8;    /* RWX  */  /* 0x0 */
          uint32_t RESERVED_03                             : 23;
} PixEngCfg_store9_Request_t;

/* store9_Trigger */
typedef struct PixEngCfg_store9_Trigger {
    __IO  uint32_t store9_Sync_Trigger                     :  1;    /* W1P  */  /* X */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t store9_trigger_sequence_complete        :  1;    /* W1P  */  /* X */
          uint32_t RESERVED_04                             : 27;
} PixEngCfg_store9_Trigger_t;

/* store9_Status */
typedef struct PixEngCfg_store9_Status {
    __IO  uint32_t store9_pipeline_status                  :  2;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  6;
    __IO  uint32_t store9_sync_busy                        :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             : 23;
} PixEngCfg_store9_Status_t;

/* constframe0_LockUnlock */
typedef struct PixEngCfg_constframe0_LockUnlock {
    __IO  uint32_t constframe0_LockUnlock                  : 32;    /* W    */  /* X */
} PixEngCfg_constframe0_LockUnlock_t;

/* constframe0_LockStatus */
typedef struct PixEngCfg_constframe0_LockStatus {
    __IO  uint32_t constframe0_LockStatus                  :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t constframe0_PrivilegeStatus             :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t constframe0_FreezeStatus                :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} PixEngCfg_constframe0_LockStatus_t;

/* constframe0_Status */
typedef struct PixEngCfg_constframe0_Status {
          uint32_t RESERVED_01                             : 16;
    __IO  uint32_t constframe0_sel                         :  2;    /* R    */  /* X */
          uint32_t RESERVED_03                             : 14;
} PixEngCfg_constframe0_Status_t;

/* extdst0_LockUnlock */
typedef struct PixEngCfg_extdst0_LockUnlock {
    __IO  uint32_t extdst0_LockUnlock                      : 32;    /* W    */  /* X */
} PixEngCfg_extdst0_LockUnlock_t;

/* extdst0_LockStatus */
typedef struct PixEngCfg_extdst0_LockStatus {
    __IO  uint32_t extdst0_LockStatus                      :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t extdst0_PrivilegeStatus                 :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t extdst0_FreezeStatus                    :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} PixEngCfg_extdst0_LockStatus_t;

/* extdst0_Static */
typedef struct PixEngCfg_extdst0_Static {
    __IO  uint32_t extdst0_ShdEn                           :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t extdst0_powerdown                       :  1;    /* RW   */  /* 0x1 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t extdst0_Sync_Mode                       :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_06                             :  2;
    __IO  uint32_t extdst0_SW_Reset                        :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_08                             :  4;
    __IO  uint32_t extdst0_div                             :  8;    /* RW   */  /* 0x80 */
          uint32_t RESERVED_10                             :  8;
} PixEngCfg_extdst0_Static_t;

/* extdst0_Dynamic */
typedef struct PixEngCfg_extdst0_Dynamic {
    __IO  uint32_t extdst0_src_sel                         :  5;    /* RWS  */  /* 0xf */
          uint32_t RESERVED_02                             : 27;
} PixEngCfg_extdst0_Dynamic_t;

/* extdst0_Request */
typedef struct PixEngCfg_extdst0_Request {
    __IO  uint32_t extdst0_sel_ShdLdReq                    :  1;    /* RWX  */  /* 0x0 */
    __IO  uint32_t extdst0_ShdLdReq                        :  8;    /* RWX  */  /* 0x0 */
          uint32_t RESERVED_03                             : 23;
} PixEngCfg_extdst0_Request_t;

/* extdst0_Trigger */
typedef struct PixEngCfg_extdst0_Trigger {
    __IO  uint32_t extdst0_Sync_Trigger                    :  1;    /* W1P  */  /* X */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t extdst0_trigger_sequence_complete       :  1;    /* W1P  */  /* X */
          uint32_t RESERVED_04                             : 27;
} PixEngCfg_extdst0_Trigger_t;

/* extdst0_Status */
typedef struct PixEngCfg_extdst0_Status {
    __IO  uint32_t extdst0_pipeline_status                 :  2;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  6;
    __IO  uint32_t extdst0_sync_busy                       :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             : 23;
} PixEngCfg_extdst0_Status_t;

/* constframe4_LockUnlock */
typedef struct PixEngCfg_constframe4_LockUnlock {
    __IO  uint32_t constframe4_LockUnlock                  : 32;    /* W    */  /* X */
} PixEngCfg_constframe4_LockUnlock_t;

/* constframe4_LockStatus */
typedef struct PixEngCfg_constframe4_LockStatus {
    __IO  uint32_t constframe4_LockStatus                  :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t constframe4_PrivilegeStatus             :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t constframe4_FreezeStatus                :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} PixEngCfg_constframe4_LockStatus_t;

/* constframe4_Status */
typedef struct PixEngCfg_constframe4_Status {
          uint32_t RESERVED_01                             : 16;
    __IO  uint32_t constframe4_sel                         :  2;    /* R    */  /* X */
          uint32_t RESERVED_03                             : 14;
} PixEngCfg_constframe4_Status_t;

/* extdst4_LockUnlock */
typedef struct PixEngCfg_extdst4_LockUnlock {
    __IO  uint32_t extdst4_LockUnlock                      : 32;    /* W    */  /* X */
} PixEngCfg_extdst4_LockUnlock_t;

/* extdst4_LockStatus */
typedef struct PixEngCfg_extdst4_LockStatus {
    __IO  uint32_t extdst4_LockStatus                      :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t extdst4_PrivilegeStatus                 :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t extdst4_FreezeStatus                    :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} PixEngCfg_extdst4_LockStatus_t;

/* extdst4_Static */
typedef struct PixEngCfg_extdst4_Static {
    __IO  uint32_t extdst4_ShdEn                           :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t extdst4_powerdown                       :  1;    /* RW   */  /* 0x1 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t extdst4_Sync_Mode                       :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_06                             :  2;
    __IO  uint32_t extdst4_SW_Reset                        :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_08                             :  4;
    __IO  uint32_t extdst4_div                             :  8;    /* RW   */  /* 0x80 */
          uint32_t RESERVED_10                             :  8;
} PixEngCfg_extdst4_Static_t;

/* extdst4_Dynamic */
typedef struct PixEngCfg_extdst4_Dynamic {
    __IO  uint32_t extdst4_src_sel                         :  5;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             : 27;
} PixEngCfg_extdst4_Dynamic_t;

/* extdst4_Request */
typedef struct PixEngCfg_extdst4_Request {
    __IO  uint32_t extdst4_sel_ShdLdReq                    :  1;    /* RWX  */  /* 0x0 */
    __IO  uint32_t extdst4_ShdLdReq                        :  8;    /* RWX  */  /* 0x0 */
          uint32_t RESERVED_03                             : 23;
} PixEngCfg_extdst4_Request_t;

/* extdst4_Trigger */
typedef struct PixEngCfg_extdst4_Trigger {
    __IO  uint32_t extdst4_Sync_Trigger                    :  1;    /* W1P  */  /* X */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t extdst4_trigger_sequence_complete       :  1;    /* W1P  */  /* X */
          uint32_t RESERVED_04                             : 27;
} PixEngCfg_extdst4_Trigger_t;

/* extdst4_Status */
typedef struct PixEngCfg_extdst4_Status {
    __IO  uint32_t extdst4_pipeline_status                 :  2;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  6;
    __IO  uint32_t extdst4_sync_busy                       :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             : 23;
} PixEngCfg_extdst4_Status_t;

/* fetchdecode0_LockUnlock */
typedef struct PixEngCfg_fetchdecode0_LockUnlock {
    __IO  uint32_t fetchdecode0_LockUnlock                 : 32;    /* W    */  /* X */
} PixEngCfg_fetchdecode0_LockUnlock_t;

/* fetchdecode0_LockStatus */
typedef struct PixEngCfg_fetchdecode0_LockStatus {
    __IO  uint32_t fetchdecode0_LockStatus                 :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t etchdecode0_PrivilegeStatus             :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t fetchdecode0_FreezeStatus               :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} PixEngCfg_fetchdecode0_LockStatus_t;

/* fetchdecode0_Dynamic */
typedef struct PixEngCfg_fetchdecode0_Dynamic {
    __IO  uint32_t fetchdecode0_src_sel                    :  5;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             : 27;
} PixEngCfg_fetchdecode0_Dynamic_t;

/* fetchdecode0_Status */
typedef struct PixEngCfg_fetchdecode0_Status {
          uint32_t RESERVED_01                             : 16;
    __IO  uint32_t fetchdecode0_sel                        :  2;    /* R    */  /* X */
          uint32_t RESERVED_03                             : 14;
} PixEngCfg_fetchdecode0_Status_t;

/* fetchlayer0_LockUnlock */
typedef struct PixEngCfg_fetchlayer0_LockUnlock {
    __IO  uint32_t fetchlayer0_LockUnlock                  : 32;    /* W    */  /* X */
} PixEngCfg_fetchlayer0_LockUnlock_t;

/* fetchlayer0_LockStatus */
typedef struct PixEngCfg_fetchlayer0_LockStatus {
    __IO  uint32_t fetchlayer0_LockStatus                  :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t fetchlayer0_PrivilegeStatus             :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t fetchlayer0_FreezeStatus                :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} PixEngCfg_fetchlayer0_LockStatus_t;

/* fetchlayer0_Status */
typedef struct PixEngCfg_fetchlayer0_Status {
          uint32_t RESERVED_01                             : 16;
    __IO  uint32_t fetchlayer0_sel                         :  2;    /* R    */  /* X */
          uint32_t RESERVED_03                             : 14;
} PixEngCfg_fetchlayer0_Status_t;

/* layerblend1_LockUnlock */
typedef struct PixEngCfg_layerblend1_LockUnlock {
    __IO  uint32_t layerblend1_LockUnlock                  : 32;    /* W    */  /* X */
} PixEngCfg_layerblend1_LockUnlock_t;

/* layerblend1_LockStatus */
typedef struct PixEngCfg_layerblend1_LockStatus {
    __IO  uint32_t layerblend1_LockStatus                  :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t layerblend1_PrivilegeStatus             :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t layerblend1_FreezeStatus                :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} PixEngCfg_layerblend1_LockStatus_t;

/* layerblend1_Dynamic */
typedef struct PixEngCfg_layerblend1_Dynamic {
    __IO  uint32_t layerblend1_prim_sel                    :  5;    /* RWS  */  /* 0x9 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t layerblend1_sec_sel                     :  5;    /* RWS  */  /* 0xe */
          uint32_t RESERVED_04                             : 11;
    __IO  uint32_t layerblend1_clken                       :  2;    /* RWS  */  /* 0x1 */
          uint32_t RESERVED_06                             :  6;
} PixEngCfg_layerblend1_Dynamic_t;

/* layerblend1_Status */
typedef struct PixEngCfg_layerblend1_Status {
          uint32_t RESERVED_01                             : 16;
    __IO  uint32_t layerblend1_sel                         :  2;    /* R    */  /* X */
          uint32_t RESERVED_03                             : 14;
} PixEngCfg_layerblend1_Status_t;

/* layerblend2_LockUnlock */
typedef struct PixEngCfg_layerblend2_LockUnlock {
    __IO  uint32_t layerblend2_LockUnlock                  : 32;    /* W    */  /* X */
} PixEngCfg_layerblend2_LockUnlock_t;

/* layerblend2_LockStatus */
typedef struct PixEngCfg_layerblend2_LockStatus {
    __IO  uint32_t layerblend2_LockStatus                  :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t layerblend2_PrivilegeStatus             :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t layerblend2_FreezeStatus                :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} PixEngCfg_layerblend2_LockStatus_t;

/* layerblend2_Dynamic */
typedef struct PixEngCfg_layerblend2_Dynamic {
    __IO  uint32_t layerblend2_prim_sel                    :  5;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t layerblend2_sec_sel                     :  5;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             : 11;
    __IO  uint32_t layerblend2_clken                       :  2;    /* RWS  */  /* 0x1 */
          uint32_t RESERVED_06                             :  6;
} PixEngCfg_layerblend2_Dynamic_t;

/* layerblend2_Status */
typedef struct PixEngCfg_layerblend2_Status {
          uint32_t RESERVED_01                             : 16;
    __IO  uint32_t layerblend2_sel                         :  2;    /* R    */  /* X */
          uint32_t RESERVED_03                             : 14;
} PixEngCfg_layerblend2_Status_t;

/* extsrc8_LockUnlock */
typedef struct PixEngCfg_extsrc8_LockUnlock {
    __IO  uint32_t extsrc8_LockUnlock                      : 32;    /* W    */  /* X */
} PixEngCfg_extsrc8_LockUnlock_t;

/* extsrc8_LockStatus */
typedef struct PixEngCfg_extsrc8_LockStatus {
    __IO  uint32_t extsrc8_LockStatus                      :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t extsrc8_PrivilegeStatus                 :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t extsrc8_FreezeStatus                    :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} PixEngCfg_extsrc8_LockStatus_t;

/* extsrc8_Status */
typedef struct PixEngCfg_extsrc8_Status {
          uint32_t RESERVED_01                             : 16;
    __IO  uint32_t extsrc8_sel                             :  2;    /* R    */  /* X */
          uint32_t RESERVED_03                             : 14;
} PixEngCfg_extsrc8_Status_t;

/* ----[ DisEngCfg ]--------------------------------------------------------------------- */
/* LockUnlock0 */
typedef struct DisEngCfg_LockUnlock0 {
    __IO  uint32_t LockUnlock0                             : 32;    /* W    */  /* X */
} DisEngCfg_LockUnlock0_t;

/* LockStatus0 */
typedef struct DisEngCfg_LockStatus0 {
    __IO  uint32_t LockStatus0                             :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t PrivilegeStatus0                        :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t FreezeStatus0                           :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} DisEngCfg_LockStatus0_t;

/* ClockCtrl0 */
typedef struct DisEngCfg_ClockCtrl0 {
    __IO  uint32_t DspClkDivide0                           :  1;    /* RW   */  /* 0x1 */
          uint32_t RESERVED_02                             : 31;
} DisEngCfg_ClockCtrl0_t;

/* PolarityCtrl0 */
typedef struct DisEngCfg_PolarityCtrl0 {
    __IO  uint32_t PolHs0                                  :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t PolVs0                                  :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t PolEn0                                  :  1;    /* RW   */  /* 0x1 */
    __IO  uint32_t PixInv0                                 :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_05                             : 28;
} DisEngCfg_PolarityCtrl0_t;

/* SrcSelect0 */
typedef struct DisEngCfg_SrcSelect0 {
    __IO  uint32_t sig_select0                             :  2;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             : 30;
} DisEngCfg_SrcSelect0_t;

/* ----[ Fetch ]------------------------------------------------------------------------- */
/* LockUnlock */
typedef struct Fetch_LockUnlock {
    __IO  uint32_t LockUnlock                              : 32;    /* W    */  /* X */
} Fetch_LockUnlock_t;

/* LockStatus */
typedef struct Fetch_LockStatus {
    __IO  uint32_t LockStatus                              :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t PrivilegeStatus                         :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t FreezeStatus                            :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} Fetch_LockStatus_t;

/* StaticControl */
typedef struct Fetch_StaticControl {
    __IO  uint32_t ShdEn                                   :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             : 15;
    __IO  uint32_t BaseAddressAutoUpdate                   :  8;    /* RW   */  /* 0x0 */
    __IO  uint32_t ShdLdReqSticky                          :  8;    /* RW   */  /* 0xff */
} Fetch_StaticControl_t;

/* BurstBufferManagement */
typedef struct Fetch_BurstBufferManagement {
    __IO  uint32_t SetNumBuffers                           :  8;    /* RWS  */  /* 0x4 */
    __IO  uint32_t SetBurstLength                          :  5;    /* RWS  */  /* 0x2 */
          uint32_t RESERVED_03                             : 18;
    __IO  uint32_t LineMode                                :  1;    /* RWS  */  /* 0x0 */
} Fetch_BurstBufferManagement_t;

/* RingBufStartAddr0 */
typedef struct Fetch_RingBufStartAddr0 {
    __IO  uint32_t RingBufStartAddr0                       : 32;    /* RWS  */  /* 0x0 */
} Fetch_RingBufStartAddr0_t;

/* RingBufWrapAddr0 */
typedef struct Fetch_RingBufWrapAddr0 {
    __IO  uint32_t RingBufWrapAddr0                        : 32;    /* RWS  */  /* 0x0 */
} Fetch_RingBufWrapAddr0_t;

/* FrameProperties0 */
typedef struct Fetch_FrameProperties0 {
    __IO  uint32_t FieldId0                                :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             : 31;
} Fetch_FrameProperties0_t;

/* BaseAddress0 */
typedef struct Fetch_BaseAddress0 {
    __IO  uint32_t BaseAddress0                            : 32;    /* RWS  */  /* 0x0 */
} Fetch_BaseAddress0_t;

/* SourceBufferAttributes0 */
typedef struct Fetch_SourceBufferAttributes0 {
    __IO  uint32_t Stride0                                 : 16;    /* RWS  */  /* 0x4ff */
    __IO  uint32_t BitsPerPixel0                           :  6;    /* RWS  */  /* 0x20 */
          uint32_t RESERVED_03                             : 10;
} Fetch_SourceBufferAttributes0_t;

/* SourceBufferDimension0 */
typedef struct Fetch_SourceBufferDimension0 {
    __IO  uint32_t LineWidth0                              : 14;    /* RWS  */  /* 0x3fff */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t LineCount0                              : 14;    /* RWS  */  /* 0x3fff */
          uint32_t RESERVED_04                             :  2;
} Fetch_SourceBufferDimension0_t;

/* ColorComponentBits0 */
typedef struct Fetch_ColorComponentBits0 {
    __IO  uint32_t ComponentBitsAlpha0                     :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_02                             :  4;
    __IO  uint32_t ComponentBitsBlue0                      :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_04                             :  4;
    __IO  uint32_t ComponentBitsGreen0                     :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_06                             :  4;
    __IO  uint32_t ComponentBitsRed0                       :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_08                             :  3;
    __IO  uint32_t ITUFormat0                              :  1;    /* RWS  */  /* 0x0 */
} Fetch_ColorComponentBits0_t;

/* ColorComponentShift0 */
typedef struct Fetch_ColorComponentShift0 {
    __IO  uint32_t ComponentShiftAlpha0                    :  5;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t ComponentShiftBlue0                     :  5;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t ComponentShiftGreen0                    :  5;    /* RWS  */  /* 0x10 */
          uint32_t RESERVED_06                             :  3;
    __IO  uint32_t ComponentShiftRed0                      :  5;    /* RWS  */  /* 0x18 */
          uint32_t RESERVED_08                             :  3;
} Fetch_ColorComponentShift0_t;

/* LayerOffset0 */
typedef struct Fetch_LayerOffset0 {
    __IO  uint32_t LayerXOffset0                           : 15;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
    __IO  uint32_t LayerYOffset0                           : 15;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  1;
} Fetch_LayerOffset0_t;

/* ClipWindowOffset0 */
typedef struct Fetch_ClipWindowOffset0 {
    __IO  uint32_t ClipWindowXOffset0                      : 15;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
    __IO  uint32_t ClipWindowYOffset0                      : 15;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  1;
} Fetch_ClipWindowOffset0_t;

/* ClipWindowDimensions0 */
typedef struct Fetch_ClipWindowDimensions0 {
    __IO  uint32_t ClipWindowWidth0                        : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t ClipWindowHeight0                       : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  2;
} Fetch_ClipWindowDimensions0_t;

/* ConstantColor0 */
typedef struct Fetch_ConstantColor0 {
    __IO  uint32_t ConstantAlpha0                          :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t ConstantBlue0                           :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t ConstantGreen0                          :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t ConstantRed0                            :  8;    /* RWS  */  /* 0x0 */
} Fetch_ConstantColor0_t;

/* LayerProperty0 */
typedef struct Fetch_LayerProperty0 {
    __IO  uint32_t PaletteEnable0                          :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t TileMode0                               :  2;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  2;
    __IO  uint32_t AlphaSrcEnable0                         :  1;    /* RWS  */  /* 0x1 */
    __IO  uint32_t AlphaConstEnable0                       :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t AlphaMaskEnable0                        :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t AlphaTransEnable0                       :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RGBAlphaSrcEnable0                      :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RGBAlphaConstEnable0                    :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RGBAlphaMaskEnable0                     :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RGBAlphaTransEnable0                    :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t PremulConstRGB0                         :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t YUVConversionMode0                      :  2;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_15                             :  1;
    __IO  uint32_t GammaRemoveEnable0                      :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_17                             :  9;
    __IO  uint32_t ClipWindowEnable0                       :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t SourceBufferEnable0                     :  1;    /* RWS  */  /* 0x1 */
} Fetch_LayerProperty0_t;

/* BaseAddress1 */
typedef struct Fetch_BaseAddress1 {
    __IO  uint32_t BaseAddress1                            : 32;    /* RWS  */  /* 0x0 */
} Fetch_BaseAddress1_t;

/* SourceBufferAttributes1 */
typedef struct Fetch_SourceBufferAttributes1 {
    __IO  uint32_t Stride1                                 : 16;    /* RWS  */  /* 0x3 */
    __IO  uint32_t BitsPerPixel1                           :  6;    /* RWS  */  /* 0x20 */
          uint32_t RESERVED_03                             : 10;
} Fetch_SourceBufferAttributes1_t;

/* SourceBufferDimension1 */
typedef struct Fetch_SourceBufferDimension1 {
    __IO  uint32_t LineWidth1                              : 14;    /* RWS  */  /* 0x3fff */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t LineCount1                              : 14;    /* RWS  */  /* 0x3fff */
          uint32_t RESERVED_04                             :  2;
} Fetch_SourceBufferDimension1_t;

/* ColorComponentBits1 */
typedef struct Fetch_ColorComponentBits1 {
    __IO  uint32_t ComponentBitsAlpha1                     :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_02                             :  4;
    __IO  uint32_t ComponentBitsBlue1                      :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_04                             :  4;
    __IO  uint32_t ComponentBitsGreen1                     :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_06                             :  4;
    __IO  uint32_t ComponentBitsRed1                       :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_08                             :  3;
    __IO  uint32_t ITUFormat1                              :  1;    /* RWS  */  /* 0x0 */
} Fetch_ColorComponentBits1_t;

/* ColorComponentShift1 */
typedef struct Fetch_ColorComponentShift1 {
    __IO  uint32_t ComponentShiftAlpha1                    :  5;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t ComponentShiftBlue1                     :  5;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t ComponentShiftGreen1                    :  5;    /* RWS  */  /* 0x10 */
          uint32_t RESERVED_06                             :  3;
    __IO  uint32_t ComponentShiftRed1                      :  5;    /* RWS  */  /* 0x18 */
          uint32_t RESERVED_08                             :  3;
} Fetch_ColorComponentShift1_t;

/* LayerOffset1 */
typedef struct Fetch_LayerOffset1 {
    __IO  uint32_t LayerXOffset1                           : 15;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
    __IO  uint32_t LayerYOffset1                           : 15;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  1;
} Fetch_LayerOffset1_t;

/* ClipWindowOffset1 */
typedef struct Fetch_ClipWindowOffset1 {
    __IO  uint32_t ClipWindowXOffset1                      : 15;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
    __IO  uint32_t ClipWindowYOffset1                      : 15;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  1;
} Fetch_ClipWindowOffset1_t;

/* ClipWindowDimensions1 */
typedef struct Fetch_ClipWindowDimensions1 {
    __IO  uint32_t ClipWindowWidth1                        : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t ClipWindowHeight1                       : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  2;
} Fetch_ClipWindowDimensions1_t;

/* ConstantColor1 */
typedef struct Fetch_ConstantColor1 {
    __IO  uint32_t ConstantAlpha1                          :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t ConstantBlue1                           :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t ConstantGreen1                          :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t ConstantRed1                            :  8;    /* RWS  */  /* 0x0 */
} Fetch_ConstantColor1_t;

/* LayerProperty1 */
typedef struct Fetch_LayerProperty1 {
    __IO  uint32_t PaletteEnable1                          :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t TileMode1                               :  2;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  2;
    __IO  uint32_t AlphaSrcEnable1                         :  1;    /* RWS  */  /* 0x1 */
    __IO  uint32_t AlphaConstEnable1                       :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t AlphaMaskEnable1                        :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t AlphaTransEnable1                       :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RGBAlphaSrcEnable1                      :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RGBAlphaConstEnable1                    :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RGBAlphaMaskEnable1                     :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RGBAlphaTransEnable1                    :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t PremulConstRGB1                         :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t YUVConversionMode1                      :  2;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_15                             :  1;
    __IO  uint32_t GammaRemoveEnable1                      :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_17                             :  9;
    __IO  uint32_t ClipWindowEnable1                       :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t SourceBufferEnable1                     :  1;    /* RWS  */  /* 0x0 */
} Fetch_LayerProperty1_t;

/* BaseAddress2 */
typedef struct Fetch_BaseAddress2 {
    __IO  uint32_t BaseAddress2                            : 32;    /* RWS  */  /* 0x0 */
} Fetch_BaseAddress2_t;

/* SourceBufferAttributes2 */
typedef struct Fetch_SourceBufferAttributes2 {
    __IO  uint32_t Stride2                                 : 16;    /* RWS  */  /* 0x3 */
    __IO  uint32_t BitsPerPixel2                           :  6;    /* RWS  */  /* 0x20 */
          uint32_t RESERVED_03                             : 10;
} Fetch_SourceBufferAttributes2_t;

/* SourceBufferDimension2 */
typedef struct Fetch_SourceBufferDimension2 {
    __IO  uint32_t LineWidth2                              : 14;    /* RWS  */  /* 0x3fff */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t LineCount2                              : 14;    /* RWS  */  /* 0x3fff */
          uint32_t RESERVED_04                             :  2;
} Fetch_SourceBufferDimension2_t;

/* ColorComponentBits2 */
typedef struct Fetch_ColorComponentBits2 {
    __IO  uint32_t ComponentBitsAlpha2                     :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_02                             :  4;
    __IO  uint32_t ComponentBitsBlue2                      :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_04                             :  4;
    __IO  uint32_t ComponentBitsGreen2                     :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_06                             :  4;
    __IO  uint32_t ComponentBitsRed2                       :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_08                             :  3;
    __IO  uint32_t ITUFormat2                              :  1;    /* RWS  */  /* 0x0 */
} Fetch_ColorComponentBits2_t;

/* ColorComponentShift2 */
typedef struct Fetch_ColorComponentShift2 {
    __IO  uint32_t ComponentShiftAlpha2                    :  5;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t ComponentShiftBlue2                     :  5;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t ComponentShiftGreen2                    :  5;    /* RWS  */  /* 0x10 */
          uint32_t RESERVED_06                             :  3;
    __IO  uint32_t ComponentShiftRed2                      :  5;    /* RWS  */  /* 0x18 */
          uint32_t RESERVED_08                             :  3;
} Fetch_ColorComponentShift2_t;

/* LayerOffset2 */
typedef struct Fetch_LayerOffset2 {
    __IO  uint32_t LayerXOffset2                           : 15;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
    __IO  uint32_t LayerYOffset2                           : 15;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  1;
} Fetch_LayerOffset2_t;

/* ClipWindowOffset2 */
typedef struct Fetch_ClipWindowOffset2 {
    __IO  uint32_t ClipWindowXOffset2                      : 15;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
    __IO  uint32_t ClipWindowYOffset2                      : 15;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  1;
} Fetch_ClipWindowOffset2_t;

/* ClipWindowDimensions2 */
typedef struct Fetch_ClipWindowDimensions2 {
    __IO  uint32_t ClipWindowWidth2                        : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t ClipWindowHeight2                       : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  2;
} Fetch_ClipWindowDimensions2_t;

/* ConstantColor2 */
typedef struct Fetch_ConstantColor2 {
    __IO  uint32_t ConstantAlpha2                          :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t ConstantBlue2                           :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t ConstantGreen2                          :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t ConstantRed2                            :  8;    /* RWS  */  /* 0x0 */
} Fetch_ConstantColor2_t;

/* LayerProperty2 */
typedef struct Fetch_LayerProperty2 {
    __IO  uint32_t PaletteEnable2                          :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t TileMode2                               :  2;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  2;
    __IO  uint32_t AlphaSrcEnable2                         :  1;    /* RWS  */  /* 0x1 */
    __IO  uint32_t AlphaConstEnable2                       :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t AlphaMaskEnable2                        :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t AlphaTransEnable2                       :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RGBAlphaSrcEnable2                      :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RGBAlphaConstEnable2                    :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RGBAlphaMaskEnable2                     :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RGBAlphaTransEnable2                    :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t PremulConstRGB2                         :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t YUVConversionMode2                      :  2;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_15                             :  1;
    __IO  uint32_t GammaRemoveEnable2                      :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_17                             :  9;
    __IO  uint32_t ClipWindowEnable2                       :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t SourceBufferEnable2                     :  1;    /* RWS  */  /* 0x0 */
} Fetch_LayerProperty2_t;

/* BaseAddress3 */
typedef struct Fetch_BaseAddress3 {
    __IO  uint32_t BaseAddress3                            : 32;    /* RWS  */  /* 0x0 */
} Fetch_BaseAddress3_t;

/* SourceBufferAttributes3 */
typedef struct Fetch_SourceBufferAttributes3 {
    __IO  uint32_t Stride3                                 : 16;    /* RWS  */  /* 0x3 */
    __IO  uint32_t BitsPerPixel3                           :  6;    /* RWS  */  /* 0x20 */
          uint32_t RESERVED_03                             : 10;
} Fetch_SourceBufferAttributes3_t;

/* SourceBufferDimension3 */
typedef struct Fetch_SourceBufferDimension3 {
    __IO  uint32_t LineWidth3                              : 14;    /* RWS  */  /* 0x3fff */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t LineCount3                              : 14;    /* RWS  */  /* 0x3fff */
          uint32_t RESERVED_04                             :  2;
} Fetch_SourceBufferDimension3_t;

/* ColorComponentBits3 */
typedef struct Fetch_ColorComponentBits3 {
    __IO  uint32_t ComponentBitsAlpha3                     :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_02                             :  4;
    __IO  uint32_t ComponentBitsBlue3                      :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_04                             :  4;
    __IO  uint32_t ComponentBitsGreen3                     :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_06                             :  4;
    __IO  uint32_t ComponentBitsRed3                       :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_08                             :  3;
    __IO  uint32_t ITUFormat3                              :  1;    /* RWS  */  /* 0x0 */
} Fetch_ColorComponentBits3_t;

/* ColorComponentShift3 */
typedef struct Fetch_ColorComponentShift3 {
    __IO  uint32_t ComponentShiftAlpha3                    :  5;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t ComponentShiftBlue3                     :  5;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t ComponentShiftGreen3                    :  5;    /* RWS  */  /* 0x10 */
          uint32_t RESERVED_06                             :  3;
    __IO  uint32_t ComponentShiftRed3                      :  5;    /* RWS  */  /* 0x18 */
          uint32_t RESERVED_08                             :  3;
} Fetch_ColorComponentShift3_t;

/* LayerOffset3 */
typedef struct Fetch_LayerOffset3 {
    __IO  uint32_t LayerXOffset3                           : 15;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
    __IO  uint32_t LayerYOffset3                           : 15;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  1;
} Fetch_LayerOffset3_t;

/* ClipWindowOffset3 */
typedef struct Fetch_ClipWindowOffset3 {
    __IO  uint32_t ClipWindowXOffset3                      : 15;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
    __IO  uint32_t ClipWindowYOffset3                      : 15;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  1;
} Fetch_ClipWindowOffset3_t;

/* ClipWindowDimensions3 */
typedef struct Fetch_ClipWindowDimensions3 {
    __IO  uint32_t ClipWindowWidth3                        : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t ClipWindowHeight3                       : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  2;
} Fetch_ClipWindowDimensions3_t;

/* ConstantColor3 */
typedef struct Fetch_ConstantColor3 {
    __IO  uint32_t ConstantAlpha3                          :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t ConstantBlue3                           :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t ConstantGreen3                          :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t ConstantRed3                            :  8;    /* RWS  */  /* 0x0 */
} Fetch_ConstantColor3_t;

/* LayerProperty3 */
typedef struct Fetch_LayerProperty3 {
    __IO  uint32_t PaletteEnable3                          :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t TileMode3                               :  2;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  2;
    __IO  uint32_t AlphaSrcEnable3                         :  1;    /* RWS  */  /* 0x1 */
    __IO  uint32_t AlphaConstEnable3                       :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t AlphaMaskEnable3                        :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t AlphaTransEnable3                       :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RGBAlphaSrcEnable3                      :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RGBAlphaConstEnable3                    :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RGBAlphaMaskEnable3                     :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RGBAlphaTransEnable3                    :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t PremulConstRGB3                         :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t YUVConversionMode3                      :  2;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_15                             :  1;
    __IO  uint32_t GammaRemoveEnable3                      :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_17                             :  9;
    __IO  uint32_t ClipWindowEnable3                       :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t SourceBufferEnable3                     :  1;    /* RWS  */  /* 0x0 */
} Fetch_LayerProperty3_t;

/* BaseAddress4 */
typedef struct Fetch_BaseAddress4 {
    __IO  uint32_t BaseAddress4                            : 32;    /* RWS  */  /* 0x0 */
} Fetch_BaseAddress4_t;

/* SourceBufferAttributes4 */
typedef struct Fetch_SourceBufferAttributes4 {
    __IO  uint32_t Stride4                                 : 16;    /* RWS  */  /* 0x3 */
    __IO  uint32_t BitsPerPixel4                           :  6;    /* RWS  */  /* 0x20 */
          uint32_t RESERVED_03                             : 10;
} Fetch_SourceBufferAttributes4_t;

/* SourceBufferDimension4 */
typedef struct Fetch_SourceBufferDimension4 {
    __IO  uint32_t LineWidth4                              : 14;    /* RWS  */  /* 0x3fff */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t LineCount4                              : 14;    /* RWS  */  /* 0x3fff */
          uint32_t RESERVED_04                             :  2;
} Fetch_SourceBufferDimension4_t;

/* ColorComponentBits4 */
typedef struct Fetch_ColorComponentBits4 {
    __IO  uint32_t ComponentBitsAlpha4                     :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_02                             :  4;
    __IO  uint32_t ComponentBitsBlue4                      :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_04                             :  4;
    __IO  uint32_t ComponentBitsGreen4                     :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_06                             :  4;
    __IO  uint32_t ComponentBitsRed4                       :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_08                             :  3;
    __IO  uint32_t ITUFormat4                              :  1;    /* RWS  */  /* 0x0 */
} Fetch_ColorComponentBits4_t;

/* ColorComponentShift4 */
typedef struct Fetch_ColorComponentShift4 {
    __IO  uint32_t ComponentShiftAlpha4                    :  5;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t ComponentShiftBlue4                     :  5;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t ComponentShiftGreen4                    :  5;    /* RWS  */  /* 0x10 */
          uint32_t RESERVED_06                             :  3;
    __IO  uint32_t ComponentShiftRed4                      :  5;    /* RWS  */  /* 0x18 */
          uint32_t RESERVED_08                             :  3;
} Fetch_ColorComponentShift4_t;

/* LayerOffset4 */
typedef struct Fetch_LayerOffset4 {
    __IO  uint32_t LayerXOffset4                           : 15;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
    __IO  uint32_t LayerYOffset4                           : 15;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  1;
} Fetch_LayerOffset4_t;

/* ClipWindowOffset4 */
typedef struct Fetch_ClipWindowOffset4 {
    __IO  uint32_t ClipWindowXOffset4                      : 15;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
    __IO  uint32_t ClipWindowYOffset4                      : 15;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  1;
} Fetch_ClipWindowOffset4_t;

/* ClipWindowDimensions4 */
typedef struct Fetch_ClipWindowDimensions4 {
    __IO  uint32_t ClipWindowWidth4                        : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t ClipWindowHeight4                       : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  2;
} Fetch_ClipWindowDimensions4_t;

/* ConstantColor4 */
typedef struct Fetch_ConstantColor4 {
    __IO  uint32_t ConstantAlpha4                          :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t ConstantBlue4                           :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t ConstantGreen4                          :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t ConstantRed4                            :  8;    /* RWS  */  /* 0x0 */
} Fetch_ConstantColor4_t;

/* LayerProperty4 */
typedef struct Fetch_LayerProperty4 {
    __IO  uint32_t PaletteEnable4                          :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t TileMode4                               :  2;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  2;
    __IO  uint32_t AlphaSrcEnable4                         :  1;    /* RWS  */  /* 0x1 */
    __IO  uint32_t AlphaConstEnable4                       :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t AlphaMaskEnable4                        :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t AlphaTransEnable4                       :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RGBAlphaSrcEnable4                      :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RGBAlphaConstEnable4                    :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RGBAlphaMaskEnable4                     :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RGBAlphaTransEnable4                    :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t PremulConstRGB4                         :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t YUVConversionMode4                      :  2;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_15                             :  1;
    __IO  uint32_t GammaRemoveEnable4                      :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_17                             :  9;
    __IO  uint32_t ClipWindowEnable4                       :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t SourceBufferEnable4                     :  1;    /* RWS  */  /* 0x0 */
} Fetch_LayerProperty4_t;

/* BaseAddress5 */
typedef struct Fetch_BaseAddress5 {
    __IO  uint32_t BaseAddress5                            : 32;    /* RWS  */  /* 0x0 */
} Fetch_BaseAddress5_t;

/* SourceBufferAttributes5 */
typedef struct Fetch_SourceBufferAttributes5 {
    __IO  uint32_t Stride5                                 : 16;    /* RWS  */  /* 0x3 */
    __IO  uint32_t BitsPerPixel5                           :  6;    /* RWS  */  /* 0x20 */
          uint32_t RESERVED_03                             : 10;
} Fetch_SourceBufferAttributes5_t;

/* SourceBufferDimension5 */
typedef struct Fetch_SourceBufferDimension5 {
    __IO  uint32_t LineWidth5                              : 14;    /* RWS  */  /* 0x3fff */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t LineCount5                              : 14;    /* RWS  */  /* 0x3fff */
          uint32_t RESERVED_04                             :  2;
} Fetch_SourceBufferDimension5_t;

/* ColorComponentBits5 */
typedef struct Fetch_ColorComponentBits5 {
    __IO  uint32_t ComponentBitsAlpha5                     :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_02                             :  4;
    __IO  uint32_t ComponentBitsBlue5                      :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_04                             :  4;
    __IO  uint32_t ComponentBitsGreen5                     :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_06                             :  4;
    __IO  uint32_t ComponentBitsRed5                       :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_08                             :  3;
    __IO  uint32_t ITUFormat5                              :  1;    /* RWS  */  /* 0x0 */
} Fetch_ColorComponentBits5_t;

/* ColorComponentShift5 */
typedef struct Fetch_ColorComponentShift5 {
    __IO  uint32_t ComponentShiftAlpha5                    :  5;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t ComponentShiftBlue5                     :  5;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t ComponentShiftGreen5                    :  5;    /* RWS  */  /* 0x10 */
          uint32_t RESERVED_06                             :  3;
    __IO  uint32_t ComponentShiftRed5                      :  5;    /* RWS  */  /* 0x18 */
          uint32_t RESERVED_08                             :  3;
} Fetch_ColorComponentShift5_t;

/* LayerOffset5 */
typedef struct Fetch_LayerOffset5 {
    __IO  uint32_t LayerXOffset5                           : 15;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
    __IO  uint32_t LayerYOffset5                           : 15;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  1;
} Fetch_LayerOffset5_t;

/* ClipWindowOffset5 */
typedef struct Fetch_ClipWindowOffset5 {
    __IO  uint32_t ClipWindowXOffset5                      : 15;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
    __IO  uint32_t ClipWindowYOffset5                      : 15;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  1;
} Fetch_ClipWindowOffset5_t;

/* ClipWindowDimensions5 */
typedef struct Fetch_ClipWindowDimensions5 {
    __IO  uint32_t ClipWindowWidth5                        : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t ClipWindowHeight5                       : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  2;
} Fetch_ClipWindowDimensions5_t;

/* ConstantColor5 */
typedef struct Fetch_ConstantColor5 {
    __IO  uint32_t ConstantAlpha5                          :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t ConstantBlue5                           :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t ConstantGreen5                          :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t ConstantRed5                            :  8;    /* RWS  */  /* 0x0 */
} Fetch_ConstantColor5_t;

/* LayerProperty5 */
typedef struct Fetch_LayerProperty5 {
    __IO  uint32_t PaletteEnable5                          :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t TileMode5                               :  2;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  2;
    __IO  uint32_t AlphaSrcEnable5                         :  1;    /* RWS  */  /* 0x1 */
    __IO  uint32_t AlphaConstEnable5                       :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t AlphaMaskEnable5                        :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t AlphaTransEnable5                       :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RGBAlphaSrcEnable5                      :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RGBAlphaConstEnable5                    :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RGBAlphaMaskEnable5                     :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RGBAlphaTransEnable5                    :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t PremulConstRGB5                         :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t YUVConversionMode5                      :  2;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_15                             :  1;
    __IO  uint32_t GammaRemoveEnable5                      :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_17                             :  9;
    __IO  uint32_t ClipWindowEnable5                       :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t SourceBufferEnable5                     :  1;    /* RWS  */  /* 0x0 */
} Fetch_LayerProperty5_t;

/* BaseAddress6 */
typedef struct Fetch_BaseAddress6 {
    __IO  uint32_t BaseAddress6                            : 32;    /* RWS  */  /* 0x0 */
} Fetch_BaseAddress6_t;

/* SourceBufferAttributes6 */
typedef struct Fetch_SourceBufferAttributes6 {
    __IO  uint32_t Stride6                                 : 16;    /* RWS  */  /* 0x3 */
    __IO  uint32_t BitsPerPixel6                           :  6;    /* RWS  */  /* 0x20 */
          uint32_t RESERVED_03                             : 10;
} Fetch_SourceBufferAttributes6_t;

/* SourceBufferDimension6 */
typedef struct Fetch_SourceBufferDimension6 {
    __IO  uint32_t LineWidth6                              : 14;    /* RWS  */  /* 0x3fff */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t LineCount6                              : 14;    /* RWS  */  /* 0x3fff */
          uint32_t RESERVED_04                             :  2;
} Fetch_SourceBufferDimension6_t;

/* ColorComponentBits6 */
typedef struct Fetch_ColorComponentBits6 {
    __IO  uint32_t ComponentBitsAlpha6                     :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_02                             :  4;
    __IO  uint32_t ComponentBitsBlue6                      :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_04                             :  4;
    __IO  uint32_t ComponentBitsGreen6                     :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_06                             :  4;
    __IO  uint32_t ComponentBitsRed6                       :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_08                             :  3;
    __IO  uint32_t ITUFormat6                              :  1;    /* RWS  */  /* 0x0 */
} Fetch_ColorComponentBits6_t;

/* ColorComponentShift6 */
typedef struct Fetch_ColorComponentShift6 {
    __IO  uint32_t ComponentShiftAlpha6                    :  5;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t ComponentShiftBlue6                     :  5;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t ComponentShiftGreen6                    :  5;    /* RWS  */  /* 0x10 */
          uint32_t RESERVED_06                             :  3;
    __IO  uint32_t ComponentShiftRed6                      :  5;    /* RWS  */  /* 0x18 */
          uint32_t RESERVED_08                             :  3;
} Fetch_ColorComponentShift6_t;

/* LayerOffset6 */
typedef struct Fetch_LayerOffset6 {
    __IO  uint32_t LayerXOffset6                           : 15;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
    __IO  uint32_t LayerYOffset6                           : 15;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  1;
} Fetch_LayerOffset6_t;

/* ClipWindowOffset6 */
typedef struct Fetch_ClipWindowOffset6 {
    __IO  uint32_t ClipWindowXOffset6                      : 15;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
    __IO  uint32_t ClipWindowYOffset6                      : 15;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  1;
} Fetch_ClipWindowOffset6_t;

/* ClipWindowDimensions6 */
typedef struct Fetch_ClipWindowDimensions6 {
    __IO  uint32_t ClipWindowWidth6                        : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t ClipWindowHeight6                       : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  2;
} Fetch_ClipWindowDimensions6_t;

/* ConstantColor6 */
typedef struct Fetch_ConstantColor6 {
    __IO  uint32_t ConstantAlpha6                          :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t ConstantBlue6                           :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t ConstantGreen6                          :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t ConstantRed6                            :  8;    /* RWS  */  /* 0x0 */
} Fetch_ConstantColor6_t;

/* LayerProperty6 */
typedef struct Fetch_LayerProperty6 {
    __IO  uint32_t PaletteEnable6                          :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t TileMode6                               :  2;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  2;
    __IO  uint32_t AlphaSrcEnable6                         :  1;    /* RWS  */  /* 0x1 */
    __IO  uint32_t AlphaConstEnable6                       :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t AlphaMaskEnable6                        :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t AlphaTransEnable6                       :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RGBAlphaSrcEnable6                      :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RGBAlphaConstEnable6                    :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RGBAlphaMaskEnable6                     :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RGBAlphaTransEnable6                    :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t PremulConstRGB6                         :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t YUVConversionMode6                      :  2;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_15                             :  1;
    __IO  uint32_t GammaRemoveEnable6                      :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_17                             :  9;
    __IO  uint32_t ClipWindowEnable6                       :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t SourceBufferEnable6                     :  1;    /* RWS  */  /* 0x0 */
} Fetch_LayerProperty6_t;

/* BaseAddress7 */
typedef struct Fetch_BaseAddress7 {
    __IO  uint32_t BaseAddress7                            : 32;    /* RWS  */  /* 0x0 */
} Fetch_BaseAddress7_t;

/* SourceBufferAttributes7 */
typedef struct Fetch_SourceBufferAttributes7 {
    __IO  uint32_t Stride7                                 : 16;    /* RWS  */  /* 0x3 */
    __IO  uint32_t BitsPerPixel7                           :  6;    /* RWS  */  /* 0x20 */
          uint32_t RESERVED_03                             : 10;
} Fetch_SourceBufferAttributes7_t;

/* SourceBufferDimension7 */
typedef struct Fetch_SourceBufferDimension7 {
    __IO  uint32_t LineWidth7                              : 14;    /* RWS  */  /* 0x3fff */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t LineCount7                              : 14;    /* RWS  */  /* 0x3fff */
          uint32_t RESERVED_04                             :  2;
} Fetch_SourceBufferDimension7_t;

/* ColorComponentBits7 */
typedef struct Fetch_ColorComponentBits7 {
    __IO  uint32_t ComponentBitsAlpha7                     :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_02                             :  4;
    __IO  uint32_t ComponentBitsBlue7                      :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_04                             :  4;
    __IO  uint32_t ComponentBitsGreen7                     :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_06                             :  4;
    __IO  uint32_t ComponentBitsRed7                       :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_08                             :  3;
    __IO  uint32_t ITUFormat7                              :  1;    /* RWS  */  /* 0x0 */
} Fetch_ColorComponentBits7_t;

/* ColorComponentShift7 */
typedef struct Fetch_ColorComponentShift7 {
    __IO  uint32_t ComponentShiftAlpha7                    :  5;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t ComponentShiftBlue7                     :  5;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t ComponentShiftGreen7                    :  5;    /* RWS  */  /* 0x10 */
          uint32_t RESERVED_06                             :  3;
    __IO  uint32_t ComponentShiftRed7                      :  5;    /* RWS  */  /* 0x18 */
          uint32_t RESERVED_08                             :  3;
} Fetch_ColorComponentShift7_t;

/* LayerOffset7 */
typedef struct Fetch_LayerOffset7 {
    __IO  uint32_t LayerXOffset7                           : 15;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
    __IO  uint32_t LayerYOffset7                           : 15;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  1;
} Fetch_LayerOffset7_t;

/* ClipWindowOffset7 */
typedef struct Fetch_ClipWindowOffset7 {
    __IO  uint32_t ClipWindowXOffset7                      : 15;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
    __IO  uint32_t ClipWindowYOffset7                      : 15;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  1;
} Fetch_ClipWindowOffset7_t;

/* ClipWindowDimensions7 */
typedef struct Fetch_ClipWindowDimensions7 {
    __IO  uint32_t ClipWindowWidth7                        : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t ClipWindowHeight7                       : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  2;
} Fetch_ClipWindowDimensions7_t;

/* ConstantColor7 */
typedef struct Fetch_ConstantColor7 {
    __IO  uint32_t ConstantAlpha7                          :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t ConstantBlue7                           :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t ConstantGreen7                          :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t ConstantRed7                            :  8;    /* RWS  */  /* 0x0 */
} Fetch_ConstantColor7_t;

/* LayerProperty7 */
typedef struct Fetch_LayerProperty7 {
    __IO  uint32_t PaletteEnable7                          :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t TileMode7                               :  2;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  2;
    __IO  uint32_t AlphaSrcEnable7                         :  1;    /* RWS  */  /* 0x1 */
    __IO  uint32_t AlphaConstEnable7                       :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t AlphaMaskEnable7                        :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t AlphaTransEnable7                       :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RGBAlphaSrcEnable7                      :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RGBAlphaConstEnable7                    :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RGBAlphaMaskEnable7                     :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RGBAlphaTransEnable7                    :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t PremulConstRGB7                         :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t YUVConversionMode7                      :  2;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_15                             :  1;
    __IO  uint32_t GammaRemoveEnable7                      :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_17                             :  9;
    __IO  uint32_t ClipWindowEnable7                       :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t SourceBufferEnable7                     :  1;    /* RWS  */  /* 0x0 */
} Fetch_LayerProperty7_t;

/* FrameDimensions */
typedef struct Fetch_FrameDimensions {
    __IO  uint32_t FrameWidth                              : 14;    /* RWS  */  /* 0x13f */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t FrameHeight                             : 14;    /* RWS  */  /* 0xef */
          uint32_t RESERVED_04                             :  1;
    __IO  uint32_t EmptyFrame                              :  1;    /* RWS  */  /* 0x0 */
} Fetch_FrameDimensions_t;

/* FrameResampling */
typedef struct Fetch_FrameResampling {
    __IO  uint32_t StartX                                  :  6;    /* RWS  */  /* 0x0 */
    __IO  uint32_t StartY                                  :  6;    /* RWS  */  /* 0x0 */
    __IO  uint32_t DeltaX                                  :  6;    /* RWS  */  /* 0x4 */
    __IO  uint32_t DeltaY                                  :  6;    /* RWS  */  /* 0x4 */
    __IO  uint32_t SwapDirection                           :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_06                             :  7;
} Fetch_FrameResampling_t;

/* WarpControl */
typedef struct Fetch_WarpControl {
    __IO  uint32_t WarpBitsPerPixel                        :  6;    /* RWS  */  /* 0x20 */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t WarpCoordinateMode                      :  2;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  2;
    __IO  uint32_t WarpSymmetricOffset                     :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_06                             : 19;
} Fetch_WarpControl_t;

/* PerspStartX */
typedef struct Fetch_PerspStartX {
    __IO  uint32_t PerspStartX                             : 32;    /* RWS  */  /* 0x0 */
} Fetch_PerspStartX_t;

/* PerspStartY */
typedef struct Fetch_PerspStartY {
    __IO  uint32_t PerspStartY                             : 32;    /* RWS  */  /* 0x0 */
} Fetch_PerspStartY_t;

/* PerspStartW */
typedef struct Fetch_PerspStartW {
    __IO  uint32_t PerspStartW                             : 32;    /* RWS  */  /* 0x3F800000 */
} Fetch_PerspStartW_t;

/* PerspStartXX */
typedef struct Fetch_PerspStartXX {
    __IO  uint32_t PerspStartXX                            : 32;    /* RWS  */  /* 0x3F800000 */
} Fetch_PerspStartXX_t;

/* PerspStartXY */
typedef struct Fetch_PerspStartXY {
    __IO  uint32_t PerspStartXY                            : 32;    /* RWS  */  /* 0x0 */
} Fetch_PerspStartXY_t;

/* PerspStartXW */
typedef struct Fetch_PerspStartXW {
    __IO  uint32_t PerspStartXW                            : 32;    /* RWS  */  /* 0x0 */
} Fetch_PerspStartXW_t;

/* PerspStartYX */
typedef struct Fetch_PerspStartYX {
    __IO  uint32_t PerspStartYX                            : 32;    /* RWS  */  /* 0x0 */
} Fetch_PerspStartYX_t;

/* PerspStartYY */
typedef struct Fetch_PerspStartYY {
    __IO  uint32_t PerspStartYY                            : 32;    /* RWS  */  /* 0x3F800000 */
} Fetch_PerspStartYY_t;

/* PerspStartYW */
typedef struct Fetch_PerspStartYW {
    __IO  uint32_t PerspStartYW                            : 32;    /* RWS  */  /* 0x0 */
} Fetch_PerspStartYW_t;

/* AffineStartX */
typedef struct Fetch_AffineStartX {
          uint32_t RESERVED_01                             : 11;
    __IO  uint32_t AffineStartX                            : 21;    /* RWS  */  /* 0x0 */
} Fetch_AffineStartX_t;

/* AffineStartY */
typedef struct Fetch_AffineStartY {
          uint32_t RESERVED_01                             : 11;
    __IO  uint32_t AffineStartY                            : 21;    /* RWS  */  /* 0x0 */
} Fetch_AffineStartY_t;

/* AffineDeltaXX */
typedef struct Fetch_AffineDeltaXX {
    __IO  uint32_t AffineDeltaXX                           : 25;    /* RWS  */  /* 0x40000 */
          uint32_t RESERVED_02                             :  7;
} Fetch_AffineDeltaXX_t;

/* AffineDeltaXY */
typedef struct Fetch_AffineDeltaXY {
    __IO  uint32_t AffineDeltaXY                           : 25;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  7;
} Fetch_AffineDeltaXY_t;

/* AffineDeltaYX */
typedef struct Fetch_AffineDeltaYX {
    __IO  uint32_t AffineDeltaYX                           : 25;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  7;
} Fetch_AffineDeltaYX_t;

/* AffineDeltaYY */
typedef struct Fetch_AffineDeltaYY {
    __IO  uint32_t AffineDeltaYY                           : 25;    /* RWS  */  /* 0x40000 */
          uint32_t RESERVED_02                             :  7;
} Fetch_AffineDeltaYY_t;

/* ArbStartX */
typedef struct Fetch_ArbStartX {
    __IO  uint32_t ArbStartX                               : 21;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             : 11;
} Fetch_ArbStartX_t;

/* ArbStartY */
typedef struct Fetch_ArbStartY {
    __IO  uint32_t ArbStartY                               : 21;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             : 11;
} Fetch_ArbStartY_t;

/* ArbDelta */
typedef struct Fetch_ArbDelta {
    __IO  uint32_t ArbDeltaXX                              :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t ArbDeltaXY                              :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t ArbDeltaYX                              :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t ArbDeltaYY                              :  8;    /* RWS  */  /* 0x0 */
} Fetch_ArbDelta_t;

/* FIRPositions */
typedef struct Fetch_FIRPositions {
    __IO  uint32_t FIR0Position                            :  4;    /* RWS  */  /* 0x5 */
    __IO  uint32_t FIR1Position                            :  4;    /* RWS  */  /* 0x6 */
    __IO  uint32_t FIR2Position                            :  4;    /* RWS  */  /* 0x9 */
    __IO  uint32_t FIR3Position                            :  4;    /* RWS  */  /* 0xa */
          uint32_t RESERVED_05                             : 16;
} Fetch_FIRPositions_t;

/* FIRCoefficients */
typedef struct Fetch_FIRCoefficients {
    __IO  uint32_t FIR0Coefficient                         :  8;    /* RWS  */  /* 0x20 */
    __IO  uint32_t FIR1Coefficient                         :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t FIR2Coefficient                         :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t FIR3Coefficient                         :  8;    /* RWS  */  /* 0x0 */
} Fetch_FIRCoefficients_t;

/* DecodeControl */
typedef struct Fetch_DecodeControl {
    __IO  uint32_t CompressionMode                         :  2;    /* RWS  */  /* 0x1 */
          uint32_t RESERVED_02                             : 13;
    __IO  uint32_t RLADEndianness                          :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RLADCompBitsRed                         :  4;    /* RWS  */  /* 0x8 */
    __IO  uint32_t RLADCompBitsGreen                       :  4;    /* RWS  */  /* 0x8 */
    __IO  uint32_t RLADCompBitsBlue                        :  4;    /* RWS  */  /* 0x8 */
    __IO  uint32_t RLADCompBitsAlpha                       :  4;    /* RWS  */  /* 0x8 */
} Fetch_DecodeControl_t;

/* SourceBufferLength */
typedef struct Fetch_SourceBufferLength {
    __IO  uint32_t RLEWords                                : 29;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
} Fetch_SourceBufferLength_t;

/* Control */
typedef struct Fetch_Control {
    __IO  uint32_t RasterMode                              :  3;    /* RWS  */  /* 0x0 */
    __IO  uint32_t InputSelect                             :  2;    /* RWS  */  /* 0x0 */
    __IO  uint32_t YUV422UpsamplingMode                    :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  1;
    __IO  uint32_t RawPixel                                :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t PaletteIdxWidth                         :  3;    /* RWS  */  /* 0x7 */
          uint32_t RESERVED_07                             :  5;
    __IO  uint32_t ClipColor                               :  1;    /* RWS  */  /* 0x1 */
    __IO  uint32_t ClipLayer                               :  3;    /* RWS  */  /* 0x0 */
    __IO  uint32_t FilterMode                              :  3;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_11                             :  9;
} Fetch_Control_t;

/* TriggerEnable */
typedef struct Fetch_TriggerEnable {
    __IO  uint32_t ShdLdReq                                :  8;    /* RWX  */  /* 0x0 */
          uint32_t RESERVED_02                             : 24;
} Fetch_TriggerEnable_t;

/* ControlTrigger */
typedef struct Fetch_ControlTrigger {
    __IO  uint32_t ShdTokGen                               :  1;    /* W1P  */  /* X */
          uint32_t RESERVED_02                             : 31;
} Fetch_ControlTrigger_t;

/* Start */
typedef struct Fetch_Start {
    __IO  uint32_t Start                                   :  1;    /* W1P  */  /* X */
          uint32_t RESERVED_02                             : 31;
} Fetch_Start_t;

/* FetchType */
typedef struct Fetch_FetchType {
    __IO  uint32_t FetchType                               :  4;    /* R    */  /* X */
          uint32_t RESERVED_02                             : 28;
} Fetch_FetchType_t;

/* DecoderStatus */
typedef struct Fetch_DecoderStatus {
    __IO  uint32_t BufferTooSmall                          :  1;    /* RW1C */  /* 0x0 */
    __IO  uint32_t BufferTooLarge                          :  1;    /* RW1C */  /* 0x0 */
          uint32_t RESERVED_03                             : 30;
} Fetch_DecoderStatus_t;

/* ReadAddress0 */
typedef struct Fetch_ReadAddress0 {
    __IO  uint32_t ReadAddress0                            : 32;    /* R    */  /* 0x0 */
} Fetch_ReadAddress0_t;

/* BurstBufferProperties */
typedef struct Fetch_BurstBufferProperties {
    __IO  uint32_t ManagedBurstBuffers                     :  8;    /* R    */  /* X */
    __IO  uint32_t BurstLengthForMaxBuffers                :  5;    /* R    */  /* X */
          uint32_t RESERVED_03                             : 19;
} Fetch_BurstBufferProperties_t;

/* Status */
typedef struct Fetch_Status {
    __IO  uint32_t WriteTimeout                            :  1;    /* RW1C */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t ReadTimeout                             :  1;    /* RW1C */  /* 0x0 */
          uint32_t RESERVED_04                             : 27;
} Fetch_Status_t;

/* ColorPalette */
typedef struct Fetch_ColorPalette {
    __IO  uint32_t ColorPalette                            : 24;    /* RWS  */  /* X */
          uint32_t RESERVED_02                             :  8;
} Fetch_ColorPalette_t;

/* ----[ ConstFrame ]-------------------------------------------------------------------- */
/* LockUnlock */
typedef struct ConstFrame_LockUnlock {
    __IO  uint32_t LockUnlock                              : 32;    /* W    */  /* X */
} ConstFrame_LockUnlock_t;

/* LockStatus */
typedef struct ConstFrame_LockStatus {
    __IO  uint32_t LockStatus                              :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t PrivilegeStatus                         :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t FreezeStatus                            :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} ConstFrame_LockStatus_t;

/* StaticControl */
typedef struct ConstFrame_StaticControl {
    __IO  uint32_t ShdEn                                   :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             : 31;
} ConstFrame_StaticControl_t;

/* FrameDimensions */
typedef struct ConstFrame_FrameDimensions {
    __IO  uint32_t FrameWidth                              : 14;    /* RWS  */  /* 0x13f */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t FrameHeight                             : 14;    /* RWS  */  /* 0xef */
          uint32_t RESERVED_04                             :  1;
    __IO  uint32_t EmptyFrame                              :  1;    /* RWS  */  /* 0x0 */
} ConstFrame_FrameDimensions_t;

/* ConstantColor */
typedef struct ConstFrame_ConstantColor {
    __IO  uint32_t ConstantAlpha                           :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t ConstantBlue                            :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t ConstantGreen                           :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t ConstantRed                             :  8;    /* RWS  */  /* 0x0 */
} ConstFrame_ConstantColor_t;

/* ControlTrigger */
typedef struct ConstFrame_ControlTrigger {
    __IO  uint32_t ShdTokGen                               :  1;    /* W1P  */  /* X */
          uint32_t RESERVED_02                             : 31;
} ConstFrame_ControlTrigger_t;

/* Start */
typedef struct ConstFrame_Start {
    __IO  uint32_t Start                                   :  1;    /* W1P  */  /* X */
          uint32_t RESERVED_02                             : 31;
} ConstFrame_Start_t;

/* ----[ Store ]------------------------------------------------------------------------- */
/* LockUnlock */
typedef struct Store_LockUnlock {
    __IO  uint32_t LockUnlock                              : 32;    /* W    */  /* X */
} Store_LockUnlock_t;

/* LockStatus */
typedef struct Store_LockStatus {
    __IO  uint32_t LockStatus                              :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t PrivilegeStatus                         :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t FreezeStatus                            :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} Store_LockStatus_t;

/* StaticControl */
typedef struct Store_StaticControl {
    __IO  uint32_t ShdEn                                   :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             :  7;
    __IO  uint32_t BaseAddressAutoUpdate                   :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_04                             : 23;
} Store_StaticControl_t;

/* BurstBufferManagement */
typedef struct Store_BurstBufferManagement {
          uint32_t RESERVED_01                             :  8;
    __IO  uint32_t SetBurstLength                          :  5;    /* RWS  */  /* 0x4 */
          uint32_t RESERVED_03                             : 19;
} Store_BurstBufferManagement_t;

/* RingBufStartAddr */
typedef struct Store_RingBufStartAddr {
    __IO  uint32_t RingBufStartAddr                        : 32;    /* RWS  */  /* 0x0 */
} Store_RingBufStartAddr_t;

/* RingBufWrapAddr */
typedef struct Store_RingBufWrapAddr {
    __IO  uint32_t RingBufWrapAddr                         : 32;    /* RWS  */  /* 0x0 */
} Store_RingBufWrapAddr_t;

/* BaseAddress */
typedef struct Store_BaseAddress {
    __IO  uint32_t BaseAddress                             : 32;    /* RWS  */  /* 0x0 */
} Store_BaseAddress_t;

/* DestinationBufferAttributes */
typedef struct Store_DestinationBufferAttributes {
    __IO  uint32_t Stride                                  : 17;    /* RWS  */  /* 0x4ff */
          uint32_t RESERVED_02                             :  7;
    __IO  uint32_t BitsPerPixel                            :  7;    /* RWS  */  /* 0x20 */
          uint32_t RESERVED_04                             :  1;
} Store_DestinationBufferAttributes_t;

/* DestinationBufferDimension */
typedef struct Store_DestinationBufferDimension {
    __IO  uint32_t LineWidth                               : 14;    /* RWS  */  /* 0x3fff */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t LineCount                               : 14;    /* RWS  */  /* 0x3fff */
          uint32_t RESERVED_04                             :  2;
} Store_DestinationBufferDimension_t;

/* FrameOffset */
typedef struct Store_FrameOffset {
    __IO  uint32_t FrameXOffset                            : 15;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
    __IO  uint32_t FrameYOffset                            : 15;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  1;
} Store_FrameOffset_t;

/* ColorComponentBits */
typedef struct Store_ColorComponentBits {
    __IO  uint32_t ComponentBitsAlpha                      :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_02                             :  4;
    __IO  uint32_t ComponentBitsBlue                       :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_04                             :  4;
    __IO  uint32_t ComponentBitsGreen                      :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_06                             :  4;
    __IO  uint32_t ComponentBitsRed                        :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_08                             :  4;
} Store_ColorComponentBits_t;

/* ColorComponentShift */
typedef struct Store_ColorComponentShift {
    __IO  uint32_t ComponentShiftAlpha                     :  5;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t ComponentShiftBlue                      :  5;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t ComponentShiftGreen                     :  5;    /* RWS  */  /* 0x10 */
          uint32_t RESERVED_06                             :  3;
    __IO  uint32_t ComponentShiftRed                       :  5;    /* RWS  */  /* 0x18 */
          uint32_t RESERVED_08                             :  3;
} Store_ColorComponentShift_t;

/* Control */
typedef struct Store_Control {
    __IO  uint32_t ColorDitherEnable                       :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t AlphaDitherEnable                       :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_03                             :  2;
    __IO  uint32_t DitherOffset                            :  4;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_05                             :  4;
    __IO  uint32_t GammaApplyEnable                        :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_07                             :  3;
    __IO  uint32_t YUVConversionMode                       :  2;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RasterMode                              :  2;    /* RWS  */  /* 0x0 */
    __IO  uint32_t YUV422DownsamplingMode                  :  2;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_11                             : 10;
} Store_Control_t;

/* EncodeControl */
typedef struct Store_EncodeControl {
    __IO  uint32_t CompressionMode                         :  1;    /* RWS  */  /* 0x1 */
          uint32_t RESERVED_02                             : 15;
    __IO  uint32_t RLADCompBitsRed                         :  4;    /* RWS  */  /* 0x8 */
    __IO  uint32_t RLADCompBitsGreen                       :  4;    /* RWS  */  /* 0x8 */
    __IO  uint32_t RLADCompBitsBlue                        :  4;    /* RWS  */  /* 0x8 */
    __IO  uint32_t RLADCompBitsAlpha                       :  4;    /* RWS  */  /* 0x8 */
} Store_EncodeControl_t;

/* DestinationBufferLength */
typedef struct Store_DestinationBufferLength {
    __IO  uint32_t RLEWordsMax                             : 29;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
} Store_DestinationBufferLength_t;

/* Start */
typedef struct Store_Start {
    __IO  uint32_t Start                                   :  1;    /* W1P  */  /* X */
          uint32_t RESERVED_02                             : 31;
} Store_Start_t;

/* EncoderStatus */
typedef struct Store_EncoderStatus {
    __IO  uint32_t RLEWords                                : 29;    /* R    */  /* 0x1fffffff */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t BufferTooSmall                          :  1;    /* RW1C */  /* 0x0 */
} Store_EncoderStatus_t;

/* WriteAddress */
typedef struct Store_WriteAddress {
    __IO  uint32_t WriteAddress                            : 32;    /* R    */  /* 0x0 */
} Store_WriteAddress_t;

/* FrameProperties */
typedef struct Store_FrameProperties {
    __IO  uint32_t FieldId                                 :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             : 31;
} Store_FrameProperties_t;

/* BurstBufferProperties */
typedef struct Store_BurstBufferProperties {
          uint32_t RESERVED_01                             :  8;
    __IO  uint32_t MaxBurstLength                          :  5;    /* R    */  /* X */
          uint32_t RESERVED_03                             : 19;
} Store_BurstBufferProperties_t;

/* LastControlWord */
typedef struct Store_LastControlWord {
    __IO  uint32_t L_VAL                                   : 32;    /* R    */  /* 0x0 */
} Store_LastControlWord_t;

/* PerfCounter */
typedef struct Store_PerfCounter {
    __IO  uint32_t PerfResult                              : 32;    /* R    */  /* 0x0 */
} Store_PerfCounter_t;

/* ----[ ExtSrc ]------------------------------------------------------------------------ */
/* LockUnlock */
typedef struct ExtSrc_LockUnlock {
    __IO  uint32_t LockUnlock                              : 32;    /* W    */  /* X */
} ExtSrc_LockUnlock_t;

/* LockStatus */
typedef struct ExtSrc_LockStatus {
    __IO  uint32_t LockStatus                              :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t PrivilegeStatus                         :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t FreezeStatus                            :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} ExtSrc_LockStatus_t;

/* StaticControl */
typedef struct ExtSrc_StaticControl {
    __IO  uint32_t ShdEn                                   :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             :  7;
    __IO  uint32_t StartSel                                :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_04                             : 23;
} ExtSrc_StaticControl_t;

/* ClipWindowOffset */
typedef struct ExtSrc_ClipWindowOffset {
    __IO  uint32_t ClipWindowXOffset                       : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t ClipWindowYOffset                       : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  2;
} ExtSrc_ClipWindowOffset_t;

/* ClipWindowDimension */
typedef struct ExtSrc_ClipWindowDimension {
    __IO  uint32_t ClipWindowWidth                         : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t ClipWindowHeight                        : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  2;
} ExtSrc_ClipWindowDimension_t;

/* ColorComponentBits */
typedef struct ExtSrc_ColorComponentBits {
    __IO  uint32_t ComponentBitsAlpha                      :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_02                             :  4;
    __IO  uint32_t ComponentBitsBlue                       :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_04                             :  4;
    __IO  uint32_t ComponentBitsGreen                      :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_06                             :  4;
    __IO  uint32_t ComponentBitsRed                        :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_08                             :  3;
    __IO  uint32_t ITUFormat                               :  1;    /* RWS  */  /* 0x0 */
} ExtSrc_ColorComponentBits_t;

/* ColorComponentShift */
typedef struct ExtSrc_ColorComponentShift {
    __IO  uint32_t ComponentShiftAlpha                     :  6;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t ComponentShiftBlue                      :  6;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_04                             :  2;
    __IO  uint32_t ComponentShiftGreen                     :  6;    /* RWS  */  /* 0x10 */
          uint32_t RESERVED_06                             :  2;
    __IO  uint32_t ComponentShiftRed                       :  6;    /* RWS  */  /* 0x18 */
          uint32_t RESERVED_08                             :  2;
} ExtSrc_ColorComponentShift_t;

/* ConstantColor */
typedef struct ExtSrc_ConstantColor {
    __IO  uint32_t ConstantAlpha                           :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t ConstantBlue                            :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t ConstantGreen                           :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t ConstantRed                             :  8;    /* RWS  */  /* 0x0 */
} ExtSrc_ConstantColor_t;

/* Control */
typedef struct ExtSrc_Control {
    __IO  uint32_t ClipWindowEnable                        :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t RasterMode                              :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t YUV422UpsamplingMode                    :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t YUVConversionMode                       :  2;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RGBAlphaSrcEnable                       :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RGBAlphaConstEnable                     :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RGBAlphaTransEnable                     :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t PremulConstRGB                          :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t AlphaSrcEnable                          :  1;    /* RWS  */  /* 0x1 */
    __IO  uint32_t AlphaConstEnable                        :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t AlphaTransEnable                        :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_13                             :  1;
    __IO  uint32_t GammaRemoveEnable                       :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_15                             : 15;
} ExtSrc_Control_t;

/* ControlTrigger */
typedef struct ExtSrc_ControlTrigger {
    __IO  uint32_t ShdTokGen                               :  1;    /* W1P  */  /* X */
          uint32_t RESERVED_02                             : 31;
} ExtSrc_ControlTrigger_t;

/* Start */
typedef struct ExtSrc_Start {
    __IO  uint32_t Start                                   :  1;    /* W1P  */  /* 0x0 */
          uint32_t RESERVED_02                             : 31;
} ExtSrc_Start_t;

/* ----[ ExtDst ]------------------------------------------------------------------------ */
/* LockUnlock */
typedef struct ExtDst_LockUnlock {
    __IO  uint32_t LockUnlock                              : 32;    /* W    */  /* X */
} ExtDst_LockUnlock_t;

/* LockStatus */
typedef struct ExtDst_LockStatus {
    __IO  uint32_t LockStatus                              :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t PrivilegeStatus                         :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t FreezeStatus                            :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} ExtDst_LockStatus_t;

/* StaticControl */
typedef struct ExtDst_StaticControl {
    __IO  uint32_t ShdEn                                   :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             :  7;
    __IO  uint32_t KICK_MODE                               :  1;    /* RW   */  /* 0x1 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t PerfCountMode                           :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_06                             : 19;
} ExtDst_StaticControl_t;

/* Control */
typedef struct ExtDst_Control {
    __IO  uint32_t GammaApplyEnable                        :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             : 31;
} ExtDst_Control_t;

/* SoftwareKick */
typedef struct ExtDst_SoftwareKick {
    __IO  uint32_t KICK                                    :  1;    /* W1P  */  /* 0x0 */
          uint32_t RESERVED_02                             : 31;
} ExtDst_SoftwareKick_t;

/* Status */
typedef struct ExtDst_Status {
    __IO  uint32_t CNT_ERR_STS                             :  1;    /* RW1C */  /* 0x0 */
          uint32_t RESERVED_02                             : 31;
} ExtDst_Status_t;

/* ControlWord */
typedef struct ExtDst_ControlWord {
    __IO  uint32_t CW_VAL                                  : 32;    /* R    */  /* X */
} ExtDst_ControlWord_t;

/* CurPixelCnt */
typedef struct ExtDst_CurPixelCnt {
    __IO  uint32_t C_XVAL                                  : 16;    /* R    */  /* X */
    __IO  uint32_t C_YVAL                                  : 16;    /* R    */  /* X */
} ExtDst_CurPixelCnt_t;

/* LastPixelCnt */
typedef struct ExtDst_LastPixelCnt {
    __IO  uint32_t L_XVAL                                  : 16;    /* R    */  /* X */
    __IO  uint32_t L_YVAL                                  : 16;    /* R    */  /* X */
} ExtDst_LastPixelCnt_t;

/* PerfCounter */
typedef struct ExtDst_PerfCounter {
    __IO  uint32_t PerfResult                              : 32;    /* R    */  /* 0x0 */
} ExtDst_PerfCounter_t;

/* ----[ Matrix ]------------------------------------------------------------------------ */
/* LockUnlock */
typedef struct Matrix_LockUnlock {
    __IO  uint32_t LockUnlock                              : 32;    /* W    */  /* X */
} Matrix_LockUnlock_t;

/* LockStatus */
typedef struct Matrix_LockStatus {
    __IO  uint32_t LockStatus                              :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t PrivilegeStatus                         :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t FreezeStatus                            :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} Matrix_LockStatus_t;

/* StaticControl */
typedef struct Matrix_StaticControl {
    __IO  uint32_t ShdEn                                   :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             : 31;
} Matrix_StaticControl_t;

/* Control */
typedef struct Matrix_Control {
    __IO  uint32_t MODE                                    :  2;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t AlphaMask                               :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t AlphaInvert                             :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_05                             : 26;
} Matrix_Control_t;

/* Red0 */
typedef struct Matrix_Red0 {
    __IO  uint32_t A11                                     : 13;    /* RWS  */  /* 0x400 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t A12                                     : 13;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
} Matrix_Red0_t;

/* Red1 */
typedef struct Matrix_Red1 {
    __IO  uint32_t A13                                     : 13;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t A14                                     : 13;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
} Matrix_Red1_t;

/* Green0 */
typedef struct Matrix_Green0 {
    __IO  uint32_t A21                                     : 13;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t A22                                     : 13;    /* RWS  */  /* 0x400 */
          uint32_t RESERVED_04                             :  3;
} Matrix_Green0_t;

/* Green1 */
typedef struct Matrix_Green1 {
    __IO  uint32_t A23                                     : 13;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t A24                                     : 13;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
} Matrix_Green1_t;

/* Blue0 */
typedef struct Matrix_Blue0 {
    __IO  uint32_t A31                                     : 13;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t A32                                     : 13;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
} Matrix_Blue0_t;

/* Blue1 */
typedef struct Matrix_Blue1 {
    __IO  uint32_t A33                                     : 13;    /* RWS  */  /* 0x400 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t A34                                     : 13;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
} Matrix_Blue1_t;

/* Alpha0 */
typedef struct Matrix_Alpha0 {
    __IO  uint32_t A41                                     : 13;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t A42                                     : 13;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
} Matrix_Alpha0_t;

/* Alpha1 */
typedef struct Matrix_Alpha1 {
    __IO  uint32_t A43                                     : 13;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t A44                                     : 13;    /* RWS  */  /* 0x400 */
          uint32_t RESERVED_04                             :  3;
} Matrix_Alpha1_t;

/* OffsetVector0 */
typedef struct Matrix_OffsetVector0 {
    __IO  uint32_t C1                                      : 13;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t C2                                      : 13;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
} Matrix_OffsetVector0_t;

/* OffsetVector1 */
typedef struct Matrix_OffsetVector1 {
    __IO  uint32_t C3                                      : 13;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t C4                                      : 13;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
} Matrix_OffsetVector1_t;

/* ----[ CLuT ]-------------------------------------------------------------------------- */
/* LockUnlock */
typedef struct CLuT_LockUnlock {
    __IO  uint32_t LockUnlock                              : 32;    /* W    */  /* X */
} CLuT_LockUnlock_t;

/* LockStatus */
typedef struct CLuT_LockStatus {
    __IO  uint32_t LockStatus                              :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t PrivilegeStatus                         :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t FreezeStatus                            :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} CLuT_LockStatus_t;

/* StaticControl */
typedef struct CLuT_StaticControl {
    __IO  uint32_t ShdEn                                   :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             : 31;
} CLuT_StaticControl_t;

/* UnshadowedControl */
typedef struct CLuT_UnshadowedControl {
    __IO  uint32_t B_EN                                    :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t G_EN                                    :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t R_EN                                    :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_04                             : 29;
} CLuT_UnshadowedControl_t;

/* Control */
typedef struct CLuT_Control {
    __IO  uint32_t MODE                                    :  2;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t COL_8BIT                                :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t AlphaMask                               :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t AlphaInvert                             :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_06                             :  1;
    __IO  uint32_t IDX_BITS                                :  4;    /* RWS  */  /* 0x8 */
          uint32_t RESERVED_08                             : 20;
} CLuT_Control_t;

/* Status */
typedef struct CLuT_Status {
    __IO  uint32_t WRITE_TIMEOUT                           :  1;    /* RW1C */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t READ_TIMEOUT                            :  1;    /* RW1C */  /* 0x0 */
          uint32_t RESERVED_04                             : 27;
} CLuT_Status_t;

/* LastControlWord */
typedef struct CLuT_LastControlWord {
    __IO  uint32_t L_VAL                                   : 32;    /* R    */  /* X */
} CLuT_LastControlWord_t;

/* LUT */
typedef struct CLuT_LUT {
    __IO  uint32_t BLUE                                    : 10;    /* RW   */  /* X */
    __IO  uint32_t GREEN                                   : 10;    /* RW   */  /* X */
    __IO  uint32_t RED                                     : 10;    /* RW   */  /* X */
          uint32_t RESERVED_04                             :  2;
} CLuT_LUT_t;

/* ----[ GammaCor ]---------------------------------------------------------------------- */
/* LockUnlock */
typedef struct GammaCor_LockUnlock {
    __IO  uint32_t LockUnlock                              : 32;    /* W    */  /* X */
} GammaCor_LockUnlock_t;

/* LockStatus */
typedef struct GammaCor_LockStatus {
    __IO  uint32_t LockStatus                              :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t PrivilegeStatus                         :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t FreezeStatus                            :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} GammaCor_LockStatus_t;

/* StaticControl */
typedef struct GammaCor_StaticControl {
    __IO  uint32_t ShdEn                                   :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t BlueWriteEnable                         :  1;    /* RW   */  /* 0x1 */
    __IO  uint32_t GreenWriteEnable                        :  1;    /* RW   */  /* 0x1 */
    __IO  uint32_t RedWriteEnable                          :  1;    /* RW   */  /* 0x1 */
          uint32_t RESERVED_05                             : 28;
} GammaCor_StaticControl_t;

/* LutStart */
typedef struct GammaCor_LutStart {
    __IO  uint32_t StartBlue                               : 10;    /* W    */  /* X */
    __IO  uint32_t StartGreen                              : 10;    /* W    */  /* X */
    __IO  uint32_t StartRed                                : 10;    /* W    */  /* X */
          uint32_t RESERVED_04                             :  2;
} GammaCor_LutStart_t;

/* LutDeltas */
typedef struct GammaCor_LutDeltas {
    __IO  uint32_t DeltaBlue                               : 10;    /* W    */  /* 0x0 */
    __IO  uint32_t DeltaGreen                              : 10;    /* W    */  /* 0x0 */
    __IO  uint32_t DeltaRed                                : 10;    /* W    */  /* 0x0 */
          uint32_t RESERVED_04                             :  2;
} GammaCor_LutDeltas_t;

/* Control */
typedef struct GammaCor_Control {
    __IO  uint32_t Mode                                    :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t AlphaMask                               :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t AlphaInvert                             :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_05                             : 26;
} GammaCor_Control_t;

/* ----[ Dither ]------------------------------------------------------------------------ */
/* LockUnlock */
typedef struct Dither_LockUnlock {
    __IO  uint32_t LockUnlock                              : 32;    /* W    */  /* X */
} Dither_LockUnlock_t;

/* LockStatus */
typedef struct Dither_LockStatus {
    __IO  uint32_t LockStatus                              :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t PrivilegeStatus                         :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t FreezeStatus                            :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} Dither_LockStatus_t;

/* Control */
typedef struct Dither_Control {
    __IO  uint32_t mode                                    :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             : 31;
} Dither_Control_t;

/* DitherControl */
typedef struct Dither_DitherControl {
    __IO  uint32_t blue_range_select                       :  3;    /* RW   */  /* 0x2 */
          uint32_t RESERVED_02                             :  1;
    __IO  uint32_t green_range_select                      :  3;    /* RW   */  /* 0x2 */
          uint32_t RESERVED_04                             :  1;
    __IO  uint32_t red_range_select                        :  3;    /* RW   */  /* 0x2 */
          uint32_t RESERVED_06                             :  5;
    __IO  uint32_t offset_select                           :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_08                             :  3;
    __IO  uint32_t algo_select                             :  2;    /* RW   */  /* 0x3 */
          uint32_t RESERVED_10                             :  2;
    __IO  uint32_t alpha_mode                              :  2;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_12                             :  6;
} Dither_DitherControl_t;

/* Release */
typedef struct Dither_Release {
    __IO  uint32_t subversion                              :  8;    /* R    */  /* 0x0 */
    __IO  uint32_t version                                 :  8;    /* R    */  /* 0x0 */
          uint32_t RESERVED_03                             : 16;
} Dither_Release_t;

/* ----[ LayerBlend ]-------------------------------------------------------------------- */
/* LockUnlock */
typedef struct LayerBlend_LockUnlock {
    __IO  uint32_t LockUnlock                              : 32;    /* W    */  /* X */
} LayerBlend_LockUnlock_t;

/* LockStatus */
typedef struct LayerBlend_LockStatus {
    __IO  uint32_t LockStatus                              :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t PrivilegeStatus                         :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t FreezeStatus                            :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} LayerBlend_LockStatus_t;

/* StaticControl */
typedef struct LayerBlend_StaticControl {
    __IO  uint32_t ShdEn                                   :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t ShdLdSel                                :  2;    /* RW   */  /* 0x2 */
    __IO  uint32_t ShdTokSel                               :  2;    /* RW   */  /* 0x2 */
          uint32_t RESERVED_04                             : 27;
} LayerBlend_StaticControl_t;

/* Control */
typedef struct LayerBlend_Control {
    __IO  uint32_t MODE                                    :  1;    /* RWS  */  /* 0x1 */
          uint32_t RESERVED_02                             :  1;
    __IO  uint32_t AlphaMaskEnable                         :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  1;
    __IO  uint32_t AlphaMaskMode                           :  3;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_06                             :  1;
    __IO  uint32_t SecLowPassEn                            :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t SecReplicateEn                          :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t SecEvenRowEvenColDis                    :  4;    /* RWS  */  /* 0x0 */
    __IO  uint32_t SecEvenRowOddColDis                     :  4;    /* RWS  */  /* 0x0 */
    __IO  uint32_t SecOddRowEvenColDis                     :  4;    /* RWS  */  /* 0x0 */
    __IO  uint32_t SecOddRowOddColDis                      :  4;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_13                             :  6;
} LayerBlend_Control_t;

/* BlendControl */
typedef struct LayerBlend_BlendControl {
    __IO  uint32_t PRIM_C_BLD_FUNC                         :  3;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
    __IO  uint32_t SEC_C_BLD_FUNC                          :  3;    /* RWS  */  /* 0x1 */
          uint32_t RESERVED_04                             :  1;
    __IO  uint32_t PRIM_A_BLD_FUNC                         :  3;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_06                             :  1;
    __IO  uint32_t SEC_A_BLD_FUNC                          :  3;    /* RWS  */  /* 0x1 */
          uint32_t RESERVED_08                             :  1;
    __IO  uint32_t BlendAlpha                              :  8;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_10                             :  8;
} LayerBlend_BlendControl_t;

/* Position */
typedef struct LayerBlend_Position {
    __IO  uint32_t XPOS                                    : 16;    /* RWS  */  /* 0x0 */
    __IO  uint32_t YPOS                                    : 16;    /* RWS  */  /* 0x0 */
} LayerBlend_Position_t;

/* ----[ BlitBlend ]--------------------------------------------------------------------- */
/* LockUnlock */
typedef struct BlitBlend_LockUnlock {
    __IO  uint32_t LockUnlock                              : 32;    /* W    */  /* X */
} BlitBlend_LockUnlock_t;

/* LockStatus */
typedef struct BlitBlend_LockStatus {
    __IO  uint32_t LockStatus                              :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t PrivilegeStatus                         :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t FreezeStatus                            :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} BlitBlend_LockStatus_t;

/* StaticControl */
typedef struct BlitBlend_StaticControl {
    __IO  uint32_t ShdEn                                   :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             : 31;
} BlitBlend_StaticControl_t;

/* Control */
typedef struct BlitBlend_Control {
    __IO  uint32_t Mode                                    :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             : 31;
} BlitBlend_Control_t;

/* NeutralBorder */
typedef struct BlitBlend_NeutralBorder {
    __IO  uint32_t NeutralBorderMode                       :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  7;
    __IO  uint32_t NeutralBorderLeft                       :  3;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  1;
    __IO  uint32_t NeutralBorderRight                      :  3;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_06                             : 17;
} BlitBlend_NeutralBorder_t;

/* ConstantColor */
typedef struct BlitBlend_ConstantColor {
    __IO  uint32_t ConstantAlpha                           :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t ConstantBlue                            :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t ConstantGreen                           :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t ConstantRed                             :  8;    /* RWS  */  /* 0x0 */
} BlitBlend_ConstantColor_t;

/* ColorRedBlendFunction */
typedef struct BlitBlend_ColorRedBlendFunction {
    __IO  uint32_t BlendFuncColorRedSrc                    : 16;    /* RWS  */  /* 0x300 */
    __IO  uint32_t BlendFuncColorRedDst                    : 16;    /* RWS  */  /* 0x300 */
} BlitBlend_ColorRedBlendFunction_t;

/* ColorGreenBlendFunction */
typedef struct BlitBlend_ColorGreenBlendFunction {
    __IO  uint32_t BlendFuncColorGreenSrc                  : 16;    /* RWS  */  /* 0x300 */
    __IO  uint32_t BlendFuncColorGreenDst                  : 16;    /* RWS  */  /* 0x300 */
} BlitBlend_ColorGreenBlendFunction_t;

/* ColorBlueBlendFunction */
typedef struct BlitBlend_ColorBlueBlendFunction {
    __IO  uint32_t BlendFuncColorBlueSrc                   : 16;    /* RWS  */  /* 0x300 */
    __IO  uint32_t BlendFuncColorBlueDst                   : 16;    /* RWS  */  /* 0x300 */
} BlitBlend_ColorBlueBlendFunction_t;

/* AlphaBlendFunction */
typedef struct BlitBlend_AlphaBlendFunction {
    __IO  uint32_t BlendFuncAlphaSrc                       : 16;    /* RWS  */  /* 0x300 */
    __IO  uint32_t BlendFuncAlphaDst                       : 16;    /* RWS  */  /* 0x300 */
} BlitBlend_AlphaBlendFunction_t;

/* BlendMode1 */
typedef struct BlitBlend_BlendMode1 {
    __IO  uint32_t BlendModeColorRed                       : 16;    /* RWS  */  /* 0x8006 */
    __IO  uint32_t BlendModeColorGreen                     : 16;    /* RWS  */  /* 0x8006 */
} BlitBlend_BlendMode1_t;

/* BlendMode2 */
typedef struct BlitBlend_BlendMode2 {
    __IO  uint32_t BlendModeColorBlue                      : 16;    /* RWS  */  /* 0x8006 */
    __IO  uint32_t BlendModeAlpha                          : 16;    /* RWS  */  /* 0x8006 */
} BlitBlend_BlendMode2_t;

/* DirectSetup */
typedef struct BlitBlend_DirectSetup {
    __IO  uint32_t ColorDebug                              : 10;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  6;
    __IO  uint32_t AlphaDebug                              : 10;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  6;
} BlitBlend_DirectSetup_t;

/* PrimControlWord */
typedef struct BlitBlend_PrimControlWord {
    __IO  uint32_t P_VAL                                   : 32;    /* R    */  /* X */
} BlitBlend_PrimControlWord_t;

/* SecControlWord */
typedef struct BlitBlend_SecControlWord {
    __IO  uint32_t S_VAL                                   : 32;    /* R    */  /* X */
} BlitBlend_SecControlWord_t;

/* ----[ ROp ]--------------------------------------------------------------------------- */
/* LockUnlock */
typedef struct ROp_LockUnlock {
    __IO  uint32_t LockUnlock                              : 32;    /* W    */  /* X */
} ROp_LockUnlock_t;

/* LockStatus */
typedef struct ROp_LockStatus {
    __IO  uint32_t LockStatus                              :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t PrivilegeStatus                         :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t FreezeStatus                            :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} ROp_LockStatus_t;

/* StaticControl */
typedef struct ROp_StaticControl {
    __IO  uint32_t ShdEn                                   :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             : 31;
} ROp_StaticControl_t;

/* Control */
typedef struct ROp_Control {
    __IO  uint32_t Mode                                    :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t AlphaMode                               :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t BlueMode                                :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t GreenMode                               :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t RedMode                                 :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t PrimDiv2                                :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t SecDiv2                                 :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t TertDiv2                                :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_10                             : 21;
} ROp_Control_t;

/* RasterOperationIndices */
typedef struct ROp_RasterOperationIndices {
    __IO  uint32_t OpIndexAlpha                            :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t OpIndexBlue                             :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t OpIndexGreen                            :  8;    /* RWS  */  /* 0x0 */
    __IO  uint32_t OpIndexRed                              :  8;    /* RWS  */  /* 0x0 */
} ROp_RasterOperationIndices_t;

/* PrimControlWord */
typedef struct ROp_PrimControlWord {
    __IO  uint32_t P_VAL                                   : 32;    /* R    */  /* X */
} ROp_PrimControlWord_t;

/* SecControlWord */
typedef struct ROp_SecControlWord {
    __IO  uint32_t S_VAL                                   : 32;    /* R    */  /* X */
} ROp_SecControlWord_t;

/* TertControlWord */
typedef struct ROp_TertControlWord {
    __IO  uint32_t T_VAL                                   : 32;    /* R    */  /* X */
} ROp_TertControlWord_t;

/* ----[ FrameGen ]---------------------------------------------------------------------- */
/* LockUnlock */
typedef struct FrameGen_LockUnlock {
    __IO  uint32_t LockUnlock                              : 32;    /* W    */  /* X */
} FrameGen_LockUnlock_t;

/* LockStatus */
typedef struct FrameGen_LockStatus {
    __IO  uint32_t LockStatus                              :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t PrivilegeStatus                         :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t FreezeStatus                            :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} FrameGen_LockStatus_t;

/* FgStCtrl */
typedef struct FrameGen_FgStCtrl {
    __IO  uint32_t ShdEn                                   :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t FgSyncMode                              :  2;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_03                             : 29;
} FrameGen_FgStCtrl_t;

/* HtCfg1 */
typedef struct FrameGen_HtCfg1 {
    __IO  uint32_t Hact                                    : 14;    /* RW   */  /* 0x140 */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t Htotal                                  : 14;    /* RW   */  /* 0x18f */
          uint32_t RESERVED_04                             :  2;
} FrameGen_HtCfg1_t;

/* HtCfg2 */
typedef struct FrameGen_HtCfg2 {
    __IO  uint32_t Hsync                                   : 14;    /* RW   */  /* 0x1f */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t Hsbp                                    : 14;    /* RW   */  /* 0x47 */
          uint32_t RESERVED_04                             :  1;
    __IO  uint32_t HsEn                                    :  1;    /* RW   */  /* 0x1 */
} FrameGen_HtCfg2_t;

/* VtCfg1 */
typedef struct FrameGen_VtCfg1 {
    __IO  uint32_t Vact                                    : 14;    /* RW   */  /* 0xf0 */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t Vtotal                                  : 14;    /* RW   */  /* 0xfc */
          uint32_t RESERVED_04                             :  2;
} FrameGen_VtCfg1_t;

/* VtCfg2 */
typedef struct FrameGen_VtCfg2 {
    __IO  uint32_t Vsync                                   : 14;    /* RW   */  /* 0x3 */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t Vsbp                                    : 14;    /* RW   */  /* 0x9 */
          uint32_t RESERVED_04                             :  1;
    __IO  uint32_t VsEn                                    :  1;    /* RW   */  /* 0x1 */
} FrameGen_VtCfg2_t;

/* Int0Config */
typedef struct FrameGen_Int0Config {
    __IO  uint32_t Int0Col                                 : 14;    /* RWX  */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
    __IO  uint32_t Int0HsEn                                :  1;    /* RWX  */  /* 0x0 */
    __IO  uint32_t Int0Row                                 : 14;    /* RWX  */  /* 0x0 */
          uint32_t RESERVED_05                             :  1;
    __IO  uint32_t Int0En                                  :  1;    /* RWX  */  /* 0x0 */
} FrameGen_Int0Config_t;

/* Int1Config */
typedef struct FrameGen_Int1Config {
    __IO  uint32_t Int1Col                                 : 14;    /* RWX  */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
    __IO  uint32_t Int1HsEn                                :  1;    /* RWX  */  /* 0x0 */
    __IO  uint32_t Int1Row                                 : 14;    /* RWX  */  /* 0x0 */
          uint32_t RESERVED_05                             :  1;
    __IO  uint32_t Int1En                                  :  1;    /* RWX  */  /* 0x0 */
} FrameGen_Int1Config_t;

/* Int2Config */
typedef struct FrameGen_Int2Config {
    __IO  uint32_t Int2Col                                 : 14;    /* RWX  */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
    __IO  uint32_t Int2HsEn                                :  1;    /* RWX  */  /* 0x0 */
    __IO  uint32_t Int2Row                                 : 14;    /* RWX  */  /* 0x0 */
          uint32_t RESERVED_05                             :  1;
    __IO  uint32_t Int2En                                  :  1;    /* RWX  */  /* 0x0 */
} FrameGen_Int2Config_t;

/* Int3Config */
typedef struct FrameGen_Int3Config {
    __IO  uint32_t Int3Col                                 : 14;    /* RWX  */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
    __IO  uint32_t Int3HsEn                                :  1;    /* RWX  */  /* 0x0 */
    __IO  uint32_t Int3Row                                 : 14;    /* RWX  */  /* 0x0 */
          uint32_t RESERVED_05                             :  1;
    __IO  uint32_t Int3En                                  :  1;    /* RWX  */  /* 0x0 */
} FrameGen_Int3Config_t;

/* PKickConfig */
typedef struct FrameGen_PKickConfig {
    __IO  uint32_t PKickCol                                : 14;    /* RW   */  /* 0x140 */
          uint32_t RESERVED_02                             :  1;
    __IO  uint32_t PKickInt0En                             :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t PKickRow                                : 14;    /* RW   */  /* 0xf0 */
          uint32_t RESERVED_05                             :  1;
    __IO  uint32_t PKickEn                                 :  1;    /* RW   */  /* 0x0 */
} FrameGen_PKickConfig_t;

/* SKickConfig */
typedef struct FrameGen_SKickConfig {
    __IO  uint32_t SKickCol                                : 14;    /* RW   */  /* 0x140 */
          uint32_t RESERVED_02                             :  1;
    __IO  uint32_t SKickInt1En                             :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t SKickRow                                : 14;    /* RW   */  /* 0xf0 */
    __IO  uint32_t SKickTrig                               :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t SKickEn                                 :  1;    /* RW   */  /* 0x0 */
} FrameGen_SKickConfig_t;

/* SecStatConfig */
typedef struct FrameGen_SecStatConfig {
    __IO  uint32_t LevGoodFrames                           :  4;    /* RW   */  /* 0x2 */
    __IO  uint32_t LevBadFrames                            :  4;    /* RW   */  /* 0x1 */
    __IO  uint32_t LevSkewInRange                          :  4;    /* RW   */  /* 0x1 */
          uint32_t RESERVED_04                             : 20;
} FrameGen_SecStatConfig_t;

/* FgSRCR1 */
typedef struct FrameGen_FgSRCR1 {
    __IO  uint32_t SREn                                    :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t SRMode                                  :  2;    /* RW   */  /* 0x0 */
    __IO  uint32_t SRAdj                                   :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t SREven                                  :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t SRFastSync                              :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t SRQAlign                                :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t SRQVal                                  :  2;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_08                             :  7;
    __IO  uint32_t SRDbgDisp                               :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t SREpOff                                 :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_11                             : 14;
} FrameGen_FgSRCR1_t;

/* FgSRCR2 */
typedef struct FrameGen_FgSRCR2 {
    __IO  uint32_t HTotalMin                               : 14;    /* RW   */  /* 0x188 */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t HTotalMax                               : 14;    /* RW   */  /* 0x1b7 */
          uint32_t RESERVED_04                             :  2;
} FrameGen_FgSRCR2_t;

/* FgSRCR3 */
typedef struct FrameGen_FgSRCR3 {
    __IO  uint32_t VTotalMin                               : 14;    /* RW   */  /* 0xfb */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t VTotalMax                               : 14;    /* RW   */  /* 0x115 */
          uint32_t RESERVED_04                             :  2;
} FrameGen_FgSRCR3_t;

/* FgSRCR4 */
typedef struct FrameGen_FgSRCR4 {
    __IO  uint32_t TargetSkew                              : 29;    /* RW   */  /* 0xc8 */
          uint32_t RESERVED_02                             :  3;
} FrameGen_FgSRCR4_t;

/* FgSRCR5 */
typedef struct FrameGen_FgSRCR5 {
    __IO  uint32_t SyncRangeLow                            : 29;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
} FrameGen_FgSRCR5_t;

/* FgSRCR6 */
typedef struct FrameGen_FgSRCR6 {
    __IO  uint32_t SyncRangeHigh                           : 29;    /* RW   */  /* 0x190 */
          uint32_t RESERVED_02                             :  3;
} FrameGen_FgSRCR6_t;

/* FgKSDR */
typedef struct FrameGen_FgKSDR {
    __IO  uint32_t PCntCplMax                              :  3;    /* RW   */  /* 0x2 */
          uint32_t RESERVED_02                             : 13;
    __IO  uint32_t SCntCplMax                              :  3;    /* RW   */  /* 0x2 */
          uint32_t RESERVED_04                             : 13;
} FrameGen_FgKSDR_t;

/* PaCfg */
typedef struct FrameGen_PaCfg {
    __IO  uint32_t Pstartx                                 : 14;    /* RWS  */  /* 0x1 */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t Pstarty                                 : 14;    /* RWS  */  /* 0x1 */
          uint32_t RESERVED_04                             :  2;
} FrameGen_PaCfg_t;

/* SaCfg */
typedef struct FrameGen_SaCfg {
    __IO  uint32_t Sstartx                                 : 14;    /* RWS  */  /* 0x1 */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t Sstarty                                 : 14;    /* RWS  */  /* 0x1 */
          uint32_t RESERVED_04                             :  2;
} FrameGen_SaCfg_t;

/* FgInCtrl */
typedef struct FrameGen_FgInCtrl {
    __IO  uint32_t FgDm                                    :  3;    /* RWS  */  /* 0x6 */
    __IO  uint32_t EnPrimAlpha                             :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t EnSecAlpha                              :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             : 27;
} FrameGen_FgInCtrl_t;

/* FgInCtrlPanic */
typedef struct FrameGen_FgInCtrlPanic {
    __IO  uint32_t FgDmPanic                               :  3;    /* RWS  */  /* 0x0 */
    __IO  uint32_t EnPrimAlphaPanic                        :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t EnSecAlphaPanic                         :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             : 27;
} FrameGen_FgInCtrlPanic_t;

/* FgCCR */
typedef struct FrameGen_FgCCR {
    __IO  uint32_t CcBlue                                  :  8;    /* RWS  */  /* 0xff */
    __IO  uint32_t CcGreen                                 :  8;    /* RWS  */  /* 0xff */
    __IO  uint32_t CcRed                                   :  8;    /* RWS  */  /* 0xff */
    __IO  uint32_t CcAlpha                                 :  1;    /* RWS  */  /* 0x1 */
          uint32_t RESERVED_05                             :  7;
} FrameGen_FgCCR_t;

/* FgEnable */
typedef struct FrameGen_FgEnable {
    __IO  uint32_t FgEn                                    :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             : 31;
} FrameGen_FgEnable_t;

/* FgSlr */
typedef struct FrameGen_FgSlr {
    __IO  uint32_t ShdTokGen                               :  1;    /* W1P  */  /* 0x0 */
          uint32_t RESERVED_02                             : 31;
} FrameGen_FgSlr_t;

/* FgEnSts */
typedef struct FrameGen_FgEnSts {
    __IO  uint32_t EnSts                                   :  1;    /* R    */  /* 0x0 */
    __IO  uint32_t PanicStat                               :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_03                             : 30;
} FrameGen_FgEnSts_t;

/* FgTimeStamp */
typedef struct FrameGen_FgTimeStamp {
    __IO  uint32_t LineIndex                               : 14;    /* R    */  /* 0x0 */
    __IO  uint32_t FrameIndex                              : 18;    /* R    */  /* 0x0 */
} FrameGen_FgTimeStamp_t;

/* FgChStat */
typedef struct FrameGen_FgChStat {
    __IO  uint32_t PFifoEmpty                              :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  7;
    __IO  uint32_t PrimSyncStat                            :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  7;
    __IO  uint32_t SFifoEmpty                              :  1;    /* R    */  /* 0x0 */
    __IO  uint32_t SkewRangeErr                            :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_07                             :  6;
    __IO  uint32_t SecSyncStat                             :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_09                             :  7;
} FrameGen_FgChStat_t;

/* FgChStatClr */
typedef struct FrameGen_FgChStatClr {
    __IO  uint32_t ClrPrimStat                             :  1;    /* W1P  */  /* 0x0 */
          uint32_t RESERVED_02                             : 15;
    __IO  uint32_t ClrSecStat                              :  1;    /* W1P  */  /* 0x0 */
          uint32_t RESERVED_04                             : 15;
} FrameGen_FgChStatClr_t;

/* FgSkewMon */
typedef struct FrameGen_FgSkewMon {
    __IO  uint32_t SkewMon                                 : 29;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
} FrameGen_FgSkewMon_t;

/* FgSFifoMin */
typedef struct FrameGen_FgSFifoMin {
    __IO  uint32_t SFifoMin                                : 11;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             : 21;
} FrameGen_FgSFifoMin_t;

/* FgSFifoMax */
typedef struct FrameGen_FgSFifoMax {
    __IO  uint32_t SFifoMax                                : 11;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             : 21;
} FrameGen_FgSFifoMax_t;

/* FgSFifoFillClr */
typedef struct FrameGen_FgSFifoFillClr {
    __IO  uint32_t SFifoFillClr                            :  1;    /* W1P  */  /* 0x0 */
          uint32_t RESERVED_02                             : 31;
} FrameGen_FgSFifoFillClr_t;

/* FgSrEpD */
typedef struct FrameGen_FgSrEpD {
    __IO  uint32_t EpVal                                   : 29;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
} FrameGen_FgSrEpD_t;

/* FgSrFtD */
typedef struct FrameGen_FgSrFtD {
    __IO  uint32_t FrTot                                   : 28;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  4;
} FrameGen_FgSrFtD_t;

/* ----[ TCon ]-------------------------------------------------------------------------- */
/* SSqCnts */
typedef struct TCon_SSqCnts {
    __IO  uint32_t SSQCNTS_SEQY                            : 15;    /* R    */  /* X */
    __IO  uint32_t SSQCNTS_FIELD                           :  1;    /* R    */  /* X */
    __IO  uint32_t SSQCNTS_SEQX                            : 15;    /* R    */  /* X */
    __IO  uint32_t SSQCNTS_OUT                             :  1;    /* R    */  /* X */
} TCon_SSqCnts_t;

/* LockUnlock */
typedef struct TCon_LockUnlock {
    __IO  uint32_t LockUnlock                              : 32;    /* W    */  /* X */
} TCon_LockUnlock_t;

/* LockStatus */
typedef struct TCon_LockStatus {
    __IO  uint32_t LockStatus                              :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t PrivilegeStatus                         :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t FreezeStatus                            :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} TCon_LockStatus_t;

/* SSqCycle */
typedef struct TCon_SSqCycle {
    __IO  uint32_t SSQCYCLE                                :  6;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             : 26;
} TCon_SSqCycle_t;

/* SWreset */
typedef struct TCon_SWreset {
    __IO  uint32_t SWreset                                 :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t EnResetWord                             : 12;    /* RW   */  /* 0x41 */
    __IO  uint32_t ResetWordEnd                            :  8;    /* RW   */  /* 0xc0 */
    __IO  uint32_t ResetWordStart                          :  8;    /* RW   */  /* 0x3f */
} TCon_SWreset_t;

/* TCON_CTRL */
typedef struct TCon_TCON_CTRL {
    __IO  uint32_t ChannelMode                             :  2;    /* RW   */  /* 0x0 */
    __IO  uint32_t tcon_sync                               :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t Bypass                                  :  1;    /* RW   */  /* 0x1 */
    __IO  uint32_t Inv_Ctrl                                :  4;    /* RW   */  /* 0x0 */
    __IO  uint32_t EnLVDS                                  :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t LVDSMode                                :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t LVDS_Balance                            :  1;    /* RW   */  /* 0x1 */
    __IO  uint32_t LVDS_CLOCK_INV                          :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t MiniLVDS_OpCode                         :  3;    /* RW   */  /* 0x1 */
    __IO  uint32_t DUAL_SWAP                               :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t SplitPosition                           : 14;    /* RW   */  /* 0x140 */
          uint32_t RESERVED_12                             :  2;
} TCon_TCON_CTRL_t;

/* RSDSInvCtrl */
typedef struct TCon_RSDSInvCtrl {
    __IO  uint32_t RSDS_Inv                                : 12;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             :  4;
    __IO  uint32_t RSDS_Inv_Dual                           : 12;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_04                             :  4;
} TCon_RSDSInvCtrl_t;

/* MapBit3_0 */
typedef struct TCon_MapBit3_0 {
    __IO  uint32_t MapBit0                                 :  5;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t MapBit1                                 :  5;    /* RW   */  /* 0x1 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t MapBit2                                 :  5;    /* RW   */  /* 0x2 */
          uint32_t RESERVED_06                             :  3;
    __IO  uint32_t MapBit3                                 :  5;    /* RW   */  /* 0x3 */
          uint32_t RESERVED_08                             :  3;
} TCon_MapBit3_0_t;

/* MapBit7_4 */
typedef struct TCon_MapBit7_4 {
    __IO  uint32_t MapBit4                                 :  5;    /* RW   */  /* 0x4 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t MapBit5                                 :  5;    /* RW   */  /* 0x5 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t MapBit6                                 :  5;    /* RW   */  /* 0x6 */
          uint32_t RESERVED_06                             :  3;
    __IO  uint32_t MapBit7                                 :  5;    /* RW   */  /* 0x7 */
          uint32_t RESERVED_08                             :  3;
} TCon_MapBit7_4_t;

/* MapBit11_8 */
typedef struct TCon_MapBit11_8 {
    __IO  uint32_t MapBit8                                 :  5;    /* RW   */  /* 0x8 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t MapBit9                                 :  5;    /* RW   */  /* 0x9 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t MapBit10                                :  5;    /* RW   */  /* 0xa */
          uint32_t RESERVED_06                             :  3;
    __IO  uint32_t MapBit11                                :  5;    /* RW   */  /* 0xb */
          uint32_t RESERVED_08                             :  3;
} TCon_MapBit11_8_t;

/* MapBit15_12 */
typedef struct TCon_MapBit15_12 {
    __IO  uint32_t MapBit12                                :  5;    /* RW   */  /* 0xc */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t MapBit13                                :  5;    /* RW   */  /* 0xd */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t MapBit14                                :  5;    /* RW   */  /* 0xe */
          uint32_t RESERVED_06                             :  3;
    __IO  uint32_t MapBit15                                :  5;    /* RW   */  /* 0xf */
          uint32_t RESERVED_08                             :  3;
} TCon_MapBit15_12_t;

/* MapBit19_16 */
typedef struct TCon_MapBit19_16 {
    __IO  uint32_t MapBit16                                :  5;    /* RW   */  /* 0x10 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t MapBit17                                :  5;    /* RW   */  /* 0x11 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t MapBit18                                :  5;    /* RW   */  /* 0x12 */
          uint32_t RESERVED_06                             :  3;
    __IO  uint32_t MapBit19                                :  5;    /* RW   */  /* 0x13 */
          uint32_t RESERVED_08                             :  3;
} TCon_MapBit19_16_t;

/* MapBit23_20 */
typedef struct TCon_MapBit23_20 {
    __IO  uint32_t MapBit20                                :  5;    /* RW   */  /* 0x14 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t MapBit21                                :  5;    /* RW   */  /* 0x15 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t MapBit22                                :  5;    /* RW   */  /* 0x16 */
          uint32_t RESERVED_06                             :  3;
    __IO  uint32_t MapBit23                                :  5;    /* RW   */  /* 0x17 */
          uint32_t RESERVED_08                             :  3;
} TCon_MapBit23_20_t;

/* MapBit27_24 */
typedef struct TCon_MapBit27_24 {
    __IO  uint32_t MapBit24                                :  5;    /* RW   */  /* 0x18 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t MapBit25                                :  5;    /* RW   */  /* 0x19 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t MapBit26                                :  5;    /* RW   */  /* 0x1a */
          uint32_t RESERVED_06                             :  3;
    __IO  uint32_t MapBit27                                :  5;    /* RW   */  /* 0x1b */
          uint32_t RESERVED_08                             :  3;
} TCon_MapBit27_24_t;

/* MapBit3_0_Dual */
typedef struct TCon_MapBit3_0_Dual {
    __IO  uint32_t MapBit0_Dual                            :  5;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t MapBit1_Dual                            :  5;    /* RW   */  /* 0x1 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t MapBit2_Dual                            :  5;    /* RW   */  /* 0x2 */
          uint32_t RESERVED_06                             :  3;
    __IO  uint32_t MapBit3_Dual                            :  5;    /* RW   */  /* 0x3 */
          uint32_t RESERVED_08                             :  3;
} TCon_MapBit3_0_Dual_t;

/* MapBit7_4_Dual */
typedef struct TCon_MapBit7_4_Dual {
    __IO  uint32_t MapBit4_Dual                            :  5;    /* RW   */  /* 0x4 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t MapBit5_Dual                            :  5;    /* RW   */  /* 0x5 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t MapBit6_Dual                            :  5;    /* RW   */  /* 0x6 */
          uint32_t RESERVED_06                             :  3;
    __IO  uint32_t MapBit7_Dual                            :  5;    /* RW   */  /* 0x7 */
          uint32_t RESERVED_08                             :  3;
} TCon_MapBit7_4_Dual_t;

/* MapBit11_8_Dual */
typedef struct TCon_MapBit11_8_Dual {
    __IO  uint32_t MapBit8_Dual                            :  5;    /* RW   */  /* 0x8 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t MapBit9_Dual                            :  5;    /* RW   */  /* 0x9 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t MapBit10_Dual                           :  5;    /* RW   */  /* 0xa */
          uint32_t RESERVED_06                             :  3;
    __IO  uint32_t MapBit11_Dual                           :  5;    /* RW   */  /* 0xb */
          uint32_t RESERVED_08                             :  3;
} TCon_MapBit11_8_Dual_t;

/* MapBit15_12_Dual */
typedef struct TCon_MapBit15_12_Dual {
    __IO  uint32_t MapBit12_Dual                           :  5;    /* RW   */  /* 0xc */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t MapBit13_Dual                           :  5;    /* RW   */  /* 0xd */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t MapBit14_Dual                           :  5;    /* RW   */  /* 0xe */
          uint32_t RESERVED_06                             :  3;
    __IO  uint32_t MapBit15_Dual                           :  5;    /* RW   */  /* 0xf */
          uint32_t RESERVED_08                             :  3;
} TCon_MapBit15_12_Dual_t;

/* MapBit19_16_Dual */
typedef struct TCon_MapBit19_16_Dual {
    __IO  uint32_t MapBit16_Dual                           :  5;    /* RW   */  /* 0x10 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t MapBit17_Dual                           :  5;    /* RW   */  /* 0x11 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t MapBit18_Dual                           :  5;    /* RW   */  /* 0x12 */
          uint32_t RESERVED_06                             :  3;
    __IO  uint32_t MapBit19_Dual                           :  5;    /* RW   */  /* 0x13 */
          uint32_t RESERVED_08                             :  3;
} TCon_MapBit19_16_Dual_t;

/* MapBit23_20_Dual */
typedef struct TCon_MapBit23_20_Dual {
    __IO  uint32_t MapBit20_Dual                           :  5;    /* RW   */  /* 0x14 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t MapBit21_Dual                           :  5;    /* RW   */  /* 0x15 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t MapBit22_Dual                           :  5;    /* RW   */  /* 0x16 */
          uint32_t RESERVED_06                             :  3;
    __IO  uint32_t MapBit23_Dual                           :  5;    /* RW   */  /* 0x17 */
          uint32_t RESERVED_08                             :  3;
} TCon_MapBit23_20_Dual_t;

/* MapBit27_24_Dual */
typedef struct TCon_MapBit27_24_Dual {
    __IO  uint32_t MapBit24_Dual                           :  5;    /* RW   */  /* 0x18 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t MapBit25_Dual                           :  5;    /* RW   */  /* 0x19 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t MapBit26_Dual                           :  5;    /* RW   */  /* 0x1a */
          uint32_t RESERVED_06                             :  3;
    __IO  uint32_t MapBit27_Dual                           :  5;    /* RW   */  /* 0x1b */
          uint32_t RESERVED_08                             :  3;
} TCon_MapBit27_24_Dual_t;

/* SPG0PosOn */
typedef struct TCon_SPG0PosOn {
    __IO  uint32_t SPGPSON_Y0                              : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSON_FIELD0                          :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSON_X0                              : 15;    /* RW   */  /* 0x148 */
    __IO  uint32_t SPGPSON_TOGGLE0                         :  1;    /* RW   */  /* 0x0 */
} TCon_SPG0PosOn_t;

/* SPG0MaskOn */
typedef struct TCon_SPG0MaskOn {
    __IO  uint32_t SPGMKON0                                : 31;    /* RW   */  /* 0xffff */
          uint32_t RESERVED_02                             :  1;
} TCon_SPG0MaskOn_t;

/* SPG0PosOff */
typedef struct TCon_SPG0PosOff {
    __IO  uint32_t SPGPSOFF_Y0                             : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSOFF_FIELD0                         :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSOFF_X0                             : 15;    /* RW   */  /* 0x168 */
    __IO  uint32_t SPGPSOFF_TOGGLE0                        :  1;    /* RW   */  /* 0x0 */
} TCon_SPG0PosOff_t;

/* SPG0MaskOff */
typedef struct TCon_SPG0MaskOff {
    __IO  uint32_t SPGMKOFF0                               : 31;    /* RW   */  /* 0xffff */
          uint32_t RESERVED_02                             :  1;
} TCon_SPG0MaskOff_t;

/* SPG1PosOn */
typedef struct TCon_SPG1PosOn {
    __IO  uint32_t SPGPSON_Y1                              : 15;    /* RW   */  /* 0xf3 */
    __IO  uint32_t SPGPSON_FIELD1                          :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSON_X1                              : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSON_TOGGLE1                         :  1;    /* RW   */  /* 0x0 */
} TCon_SPG1PosOn_t;

/* SPG1MaskOn */
typedef struct TCon_SPG1MaskOn {
    __IO  uint32_t SPGMKON1                                : 31;    /* RW   */  /* 0x7fff0000 */
          uint32_t RESERVED_02                             :  1;
} TCon_SPG1MaskOn_t;

/* SPG1PosOff */
typedef struct TCon_SPG1PosOff {
    __IO  uint32_t SPGPSOFF_Y1                             : 15;    /* RW   */  /* 0xf7 */
    __IO  uint32_t SPGPSOFF_FIELD1                         :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSOFF_X1                             : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSOFF_TOGGLE1                        :  1;    /* RW   */  /* 0x0 */
} TCon_SPG1PosOff_t;

/* SPG1MaskOff */
typedef struct TCon_SPG1MaskOff {
    __IO  uint32_t SPGMKOFF1                               : 31;    /* RW   */  /* 0x7fff0000 */
          uint32_t RESERVED_02                             :  1;
} TCon_SPG1MaskOff_t;

/* SPG2PosOn */
typedef struct TCon_SPG2PosOn {
    __IO  uint32_t SPGPSON_Y2                              : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSON_FIELD2                          :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSON_X2                              : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSON_TOGGLE2                         :  1;    /* RW   */  /* 0x0 */
} TCon_SPG2PosOn_t;

/* SPG2MaskOn */
typedef struct TCon_SPG2MaskOn {
    __IO  uint32_t SPGMKON2                                : 31;    /* RW   */  /* 0xffff */
          uint32_t RESERVED_02                             :  1;
} TCon_SPG2MaskOn_t;

/* SPG2PosOff */
typedef struct TCon_SPG2PosOff {
    __IO  uint32_t SPGPSOFF_Y2                             : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSOFF_FIELD2                         :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSOFF_X2                             : 15;    /* RW   */  /* 0x140 */
    __IO  uint32_t SPGPSOFF_TOGGLE2                        :  1;    /* RW   */  /* 0x0 */
} TCon_SPG2PosOff_t;

/* SPG2MaskOff */
typedef struct TCon_SPG2MaskOff {
    __IO  uint32_t SPGMKOFF2                               : 31;    /* RW   */  /* 0xffff */
          uint32_t RESERVED_02                             :  1;
} TCon_SPG2MaskOff_t;

/* SPG3PosOn */
typedef struct TCon_SPG3PosOn {
    __IO  uint32_t SPGPSON_Y3                              : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSON_FIELD3                          :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSON_X3                              : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSON_TOGGLE3                         :  1;    /* RW   */  /* 0x0 */
} TCon_SPG3PosOn_t;

/* SPG3MaskOn */
typedef struct TCon_SPG3MaskOn {
    __IO  uint32_t SPGMKON3                                : 31;    /* RW   */  /* 0x7fff0000 */
          uint32_t RESERVED_02                             :  1;
} TCon_SPG3MaskOn_t;

/* SPG3PosOff */
typedef struct TCon_SPG3PosOff {
    __IO  uint32_t SPGPSOFF_Y3                             : 15;    /* RW   */  /* 0xf0 */
    __IO  uint32_t SPGPSOFF_FIELD3                         :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSOFF_X3                             : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSOFF_TOGGLE3                        :  1;    /* RW   */  /* 0x0 */
} TCon_SPG3PosOff_t;

/* SPG3MaskOff */
typedef struct TCon_SPG3MaskOff {
    __IO  uint32_t SPGMKOFF3                               : 31;    /* RW   */  /* 0x7fff0000 */
          uint32_t RESERVED_02                             :  1;
} TCon_SPG3MaskOff_t;

/* SPG4PosOn */
typedef struct TCon_SPG4PosOn {
    __IO  uint32_t SPGPSON_Y4                              : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSON_FIELD4                          :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSON_X4                              : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSON_TOGGLE4                         :  1;    /* RW   */  /* 0x0 */
} TCon_SPG4PosOn_t;

/* SPG4MaskOn */
typedef struct TCon_SPG4MaskOn {
    __IO  uint32_t SPGMKON4                                : 31;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
} TCon_SPG4MaskOn_t;

/* SPG4PosOff */
typedef struct TCon_SPG4PosOff {
    __IO  uint32_t SPGPSOFF_Y4                             : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSOFF_FIELD4                         :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSOFF_X4                             : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSOFF_TOGGLE4                        :  1;    /* RW   */  /* 0x0 */
} TCon_SPG4PosOff_t;

/* SPG4MaskOff */
typedef struct TCon_SPG4MaskOff {
    __IO  uint32_t SPGMKOFF4                               : 31;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
} TCon_SPG4MaskOff_t;

/* SPG5PosOn */
typedef struct TCon_SPG5PosOn {
    __IO  uint32_t SPGPSON_Y5                              : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSON_FIELD5                          :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSON_X5                              : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSON_TOGGLE5                         :  1;    /* RW   */  /* 0x0 */
} TCon_SPG5PosOn_t;

/* SPG5MaskOn */
typedef struct TCon_SPG5MaskOn {
    __IO  uint32_t SPGMKON5                                : 31;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
} TCon_SPG5MaskOn_t;

/* SPG5PosOff */
typedef struct TCon_SPG5PosOff {
    __IO  uint32_t SPGPSOFF_Y5                             : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSOFF_FIELD5                         :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSOFF_X5                             : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSOFF_TOGGLE5                        :  1;    /* RW   */  /* 0x0 */
} TCon_SPG5PosOff_t;

/* SPG5MaskOff */
typedef struct TCon_SPG5MaskOff {
    __IO  uint32_t SPGMKOFF5                               : 31;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
} TCon_SPG5MaskOff_t;

/* SPG6PosOn */
typedef struct TCon_SPG6PosOn {
    __IO  uint32_t SPGPSON_Y6                              : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSON_FIELD6                          :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSON_X6                              : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSON_TOGGLE6                         :  1;    /* RW   */  /* 0x0 */
} TCon_SPG6PosOn_t;

/* SPG6MaskOn */
typedef struct TCon_SPG6MaskOn {
    __IO  uint32_t SPGMKON6                                : 31;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
} TCon_SPG6MaskOn_t;

/* SPG6PosOff */
typedef struct TCon_SPG6PosOff {
    __IO  uint32_t SPGPSOFF_Y6                             : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSOFF_FIELD6                         :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSOFF_X6                             : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSOFF_TOGGLE6                        :  1;    /* RW   */  /* 0x0 */
} TCon_SPG6PosOff_t;

/* SPG6MaskOff */
typedef struct TCon_SPG6MaskOff {
    __IO  uint32_t SPGMKOFF6                               : 31;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
} TCon_SPG6MaskOff_t;

/* SPG7PosOn */
typedef struct TCon_SPG7PosOn {
    __IO  uint32_t SPGPSON_Y7                              : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSON_FIELD7                          :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSON_X7                              : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSON_TOGGLE7                         :  1;    /* RW   */  /* 0x0 */
} TCon_SPG7PosOn_t;

/* SPG7MaskOn */
typedef struct TCon_SPG7MaskOn {
    __IO  uint32_t SPGMKON7                                : 31;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
} TCon_SPG7MaskOn_t;

/* SPG7PosOff */
typedef struct TCon_SPG7PosOff {
    __IO  uint32_t SPGPSOFF_Y7                             : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSOFF_FIELD7                         :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSOFF_X7                             : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSOFF_TOGGLE7                        :  1;    /* RW   */  /* 0x0 */
} TCon_SPG7PosOff_t;

/* SPG7MaskOff */
typedef struct TCon_SPG7MaskOff {
    __IO  uint32_t SPGMKOFF7                               : 31;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
} TCon_SPG7MaskOff_t;

/* SPG8PosOn */
typedef struct TCon_SPG8PosOn {
    __IO  uint32_t SPGPSON_Y8                              : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSON_FIELD8                          :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSON_X8                              : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSON_TOGGLE8                         :  1;    /* RW   */  /* 0x0 */
} TCon_SPG8PosOn_t;

/* SPG8MaskOn */
typedef struct TCon_SPG8MaskOn {
    __IO  uint32_t SPGMKON8                                : 31;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
} TCon_SPG8MaskOn_t;

/* SPG8PosOff */
typedef struct TCon_SPG8PosOff {
    __IO  uint32_t SPGPSOFF_Y8                             : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSOFF_FIELD8                         :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSOFF_X8                             : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSOFF_TOGGLE8                        :  1;    /* RW   */  /* 0x0 */
} TCon_SPG8PosOff_t;

/* SPG8MaskOff */
typedef struct TCon_SPG8MaskOff {
    __IO  uint32_t SPGMKOFF8                               : 31;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
} TCon_SPG8MaskOff_t;

/* SPG9PosOn */
typedef struct TCon_SPG9PosOn {
    __IO  uint32_t SPGPSON_Y9                              : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSON_FIELD9                          :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSON_X9                              : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSON_TOGGLE9                         :  1;    /* RW   */  /* 0x0 */
} TCon_SPG9PosOn_t;

/* SPG9MaskOn */
typedef struct TCon_SPG9MaskOn {
    __IO  uint32_t SPGMKON9                                : 31;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
} TCon_SPG9MaskOn_t;

/* SPG9PosOff */
typedef struct TCon_SPG9PosOff {
    __IO  uint32_t SPGPSOFF_Y9                             : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSOFF_FIELD9                         :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSOFF_X9                             : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSOFF_TOGGLE9                        :  1;    /* RW   */  /* 0x0 */
} TCon_SPG9PosOff_t;

/* SPG9MaskOff */
typedef struct TCon_SPG9MaskOff {
    __IO  uint32_t SPGMKOFF9                               : 31;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
} TCon_SPG9MaskOff_t;

/* SPG10PosOn */
typedef struct TCon_SPG10PosOn {
    __IO  uint32_t SPGPSON_Y10                             : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSON_FIELD10                         :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSON_X10                             : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSON_TOGGLE10                        :  1;    /* RW   */  /* 0x0 */
} TCon_SPG10PosOn_t;

/* SPG10MaskOn */
typedef struct TCon_SPG10MaskOn {
    __IO  uint32_t SPGMKON10                               : 31;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
} TCon_SPG10MaskOn_t;

/* SPG10PosOff */
typedef struct TCon_SPG10PosOff {
    __IO  uint32_t SPGPSOFF_Y10                            : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSOFF_FIELD10                        :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSOFF_X10                            : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSOFF_TOGGLE10                       :  1;    /* RW   */  /* 0x0 */
} TCon_SPG10PosOff_t;

/* SPG10MaskOff */
typedef struct TCon_SPG10MaskOff {
    __IO  uint32_t SPGMKOFF10                              : 31;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
} TCon_SPG10MaskOff_t;

/* SPG11PosOn */
typedef struct TCon_SPG11PosOn {
    __IO  uint32_t SPGPSON_Y11                             : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSON_FIELD11                         :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSON_X11                             : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSON_TOGGLE11                        :  1;    /* RW   */  /* 0x0 */
} TCon_SPG11PosOn_t;

/* SPG11MaskOn */
typedef struct TCon_SPG11MaskOn {
    __IO  uint32_t SPGMKON11                               : 31;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
} TCon_SPG11MaskOn_t;

/* SPG11PosOff */
typedef struct TCon_SPG11PosOff {
    __IO  uint32_t SPGPSOFF_Y11                            : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSOFF_FIELD11                        :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSOFF_X11                            : 15;    /* RW   */  /* 0x0 */
    __IO  uint32_t SPGPSOFF_TOGGLE11                       :  1;    /* RW   */  /* 0x0 */
} TCon_SPG11PosOff_t;

/* SPG11MaskOff */
typedef struct TCon_SPG11MaskOff {
    __IO  uint32_t SPGMKOFF11                              : 31;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             :  1;
} TCon_SPG11MaskOff_t;

/* SMx0Sigs */
typedef struct TCon_SMx0Sigs {
    __IO  uint32_t SMX0SIGS_S0                             :  3;    /* RW   */  /* 0x2 */
    __IO  uint32_t SMX0SIGS_S1                             :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX0SIGS_S2                             :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX0SIGS_S3                             :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX0SIGS_S4                             :  3;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_06                             : 17;
} TCon_SMx0Sigs_t;

/* SMx0FctTable */
typedef struct TCon_SMx0FctTable {
    __IO  uint32_t SMXFCT0                                 : 32;    /* RW   */  /* 0x1 */
} TCon_SMx0FctTable_t;

/* SMx1Sigs */
typedef struct TCon_SMx1Sigs {
    __IO  uint32_t SMX1SIGS_S0                             :  3;    /* RW   */  /* 0x3 */
    __IO  uint32_t SMX1SIGS_S1                             :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX1SIGS_S2                             :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX1SIGS_S3                             :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX1SIGS_S4                             :  3;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_06                             : 17;
} TCon_SMx1Sigs_t;

/* SMx1FctTable */
typedef struct TCon_SMx1FctTable {
    __IO  uint32_t SMXFCT1                                 : 32;    /* RW   */  /* 0x1 */
} TCon_SMx1FctTable_t;

/* SMx2Sigs */
typedef struct TCon_SMx2Sigs {
    __IO  uint32_t SMX2SIGS_S0                             :  3;    /* RW   */  /* 0x4 */
    __IO  uint32_t SMX2SIGS_S1                             :  3;    /* RW   */  /* 0x5 */
    __IO  uint32_t SMX2SIGS_S2                             :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX2SIGS_S3                             :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX2SIGS_S4                             :  3;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_06                             : 17;
} TCon_SMx2Sigs_t;

/* SMx2FctTable */
typedef struct TCon_SMx2FctTable {
    __IO  uint32_t SMXFCT2                                 : 32;    /* RW   */  /* 0x8 */
} TCon_SMx2FctTable_t;

/* SMx3Sigs */
typedef struct TCon_SMx3Sigs {
    __IO  uint32_t SMX3SIGS_S0                             :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX3SIGS_S1                             :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX3SIGS_S2                             :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX3SIGS_S3                             :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX3SIGS_S4                             :  3;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_06                             : 17;
} TCon_SMx3Sigs_t;

/* SMx3FctTable */
typedef struct TCon_SMx3FctTable {
    __IO  uint32_t SMXFCT3                                 : 32;    /* RW   */  /* 0x0 */
} TCon_SMx3FctTable_t;

/* SMx4Sigs */
typedef struct TCon_SMx4Sigs {
    __IO  uint32_t SMX4SIGS_S0                             :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX4SIGS_S1                             :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX4SIGS_S2                             :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX4SIGS_S3                             :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX4SIGS_S4                             :  3;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_06                             : 17;
} TCon_SMx4Sigs_t;

/* SMx4FctTable */
typedef struct TCon_SMx4FctTable {
    __IO  uint32_t SMXFCT4                                 : 32;    /* RW   */  /* 0x0 */
} TCon_SMx4FctTable_t;

/* SMx5Sigs */
typedef struct TCon_SMx5Sigs {
    __IO  uint32_t SMX5SIGS_S0                             :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX5SIGS_S1                             :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX5SIGS_S2                             :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX5SIGS_S3                             :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX5SIGS_S4                             :  3;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_06                             : 17;
} TCon_SMx5Sigs_t;

/* SMx5FctTable */
typedef struct TCon_SMx5FctTable {
    __IO  uint32_t SMXFCT5                                 : 32;    /* RW   */  /* 0x0 */
} TCon_SMx5FctTable_t;

/* SMx6Sigs */
typedef struct TCon_SMx6Sigs {
    __IO  uint32_t SMX6SIGS_S0                             :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX6SIGS_S1                             :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX6SIGS_S2                             :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX6SIGS_S3                             :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX6SIGS_S4                             :  3;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_06                             : 17;
} TCon_SMx6Sigs_t;

/* SMx6FctTable */
typedef struct TCon_SMx6FctTable {
    __IO  uint32_t SMXFCT6                                 : 32;    /* RW   */  /* 0x0 */
} TCon_SMx6FctTable_t;

/* SMx7Sigs */
typedef struct TCon_SMx7Sigs {
    __IO  uint32_t SMX7SIGS_S0                             :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX7SIGS_S1                             :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX7SIGS_S2                             :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX7SIGS_S3                             :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX7SIGS_S4                             :  3;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_06                             : 17;
} TCon_SMx7Sigs_t;

/* SMx7FctTable */
typedef struct TCon_SMx7FctTable {
    __IO  uint32_t SMXFCT7                                 : 32;    /* RW   */  /* 0x0 */
} TCon_SMx7FctTable_t;

/* SMx8Sigs */
typedef struct TCon_SMx8Sigs {
    __IO  uint32_t SMX8SIGS_S0                             :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX8SIGS_S1                             :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX8SIGS_S2                             :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX8SIGS_S3                             :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX8SIGS_S4                             :  3;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_06                             : 17;
} TCon_SMx8Sigs_t;

/* SMx8FctTable */
typedef struct TCon_SMx8FctTable {
    __IO  uint32_t SMXFCT8                                 : 32;    /* RW   */  /* 0x0 */
} TCon_SMx8FctTable_t;

/* SMx9Sigs */
typedef struct TCon_SMx9Sigs {
    __IO  uint32_t SMX9SIGS_S0                             :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX9SIGS_S1                             :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX9SIGS_S2                             :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX9SIGS_S3                             :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX9SIGS_S4                             :  3;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_06                             : 17;
} TCon_SMx9Sigs_t;

/* SMx9FctTable */
typedef struct TCon_SMx9FctTable {
    __IO  uint32_t SMXFCT9                                 : 32;    /* RW   */  /* 0x0 */
} TCon_SMx9FctTable_t;

/* SMx10Sigs */
typedef struct TCon_SMx10Sigs {
    __IO  uint32_t SMX10SIGS_S0                            :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX10SIGS_S1                            :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX10SIGS_S2                            :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX10SIGS_S3                            :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX10SIGS_S4                            :  3;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_06                             : 17;
} TCon_SMx10Sigs_t;

/* SMx10FctTable */
typedef struct TCon_SMx10FctTable {
    __IO  uint32_t SMXFCT10                                : 32;    /* RW   */  /* 0x0 */
} TCon_SMx10FctTable_t;

/* SMx11Sigs */
typedef struct TCon_SMx11Sigs {
    __IO  uint32_t SMX11SIGS_S0                            :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX11SIGS_S1                            :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX11SIGS_S2                            :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX11SIGS_S3                            :  3;    /* RW   */  /* 0x0 */
    __IO  uint32_t SMX11SIGS_S4                            :  3;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_06                             : 17;
} TCon_SMx11Sigs_t;

/* SMx11FctTable */
typedef struct TCon_SMx11FctTable {
    __IO  uint32_t SMXFCT11                                : 32;    /* RW   */  /* 0x0 */
} TCon_SMx11FctTable_t;

/* ----[ Sig ]--------------------------------------------------------------------------- */
/* LockUnlock */
typedef struct Sig_LockUnlock {
    __IO  uint32_t LockUnlock                              : 32;    /* W    */  /* X */
} Sig_LockUnlock_t;

/* LockStatus */
typedef struct Sig_LockStatus {
    __IO  uint32_t LockStatus                              :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t PrivilegeStatus                         :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t FreezeStatus                            :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_06                             : 23;
} Sig_LockStatus_t;

/* StaticControl */
typedef struct Sig_StaticControl {
    __IO  uint32_t ShdEn                                   :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             :  3;
    __IO  uint32_t ShdLdSel                                :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_04                             : 11;
    __IO  uint32_t ErrThres                                :  8;    /* RW   */  /* 0x0 */
    __IO  uint32_t ErrThresReset                           :  8;    /* RW   */  /* 0x8 */
} Sig_StaticControl_t;

/* PanicColor */
typedef struct Sig_PanicColor {
          uint32_t RESERVED_01                             :  7;
    __IO  uint32_t PanicAlpha                              :  1;    /* RW   */  /* 0x0 */
    __IO  uint32_t PanicBlue                               :  8;    /* RW   */  /* 0x0 */
    __IO  uint32_t PanicGreen                              :  8;    /* RW   */  /* 0x0 */
    __IO  uint32_t PanicRed                                :  8;    /* RW   */  /* 0x0 */
} Sig_PanicColor_t;

/* EvalControl0 */
typedef struct Sig_EvalControl0 {
    __IO  uint32_t EnEvalWin0                              :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t EnCRC0                                  :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_03                             :  6;
    __IO  uint32_t AlphaMask0                              :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t AlphaInv0                               :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_06                             :  6;
    __IO  uint32_t EnLocalPanic0                           :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t EnGlobalPanic0                          :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_09                             : 14;
} Sig_EvalControl0_t;

/* EvalUpperLeft0 */
typedef struct Sig_EvalUpperLeft0 {
    __IO  uint32_t XEvalUpperLeft0                         : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t YEvalUpperLeft0                         : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  2;
} Sig_EvalUpperLeft0_t;

/* EvalLowerRight0 */
typedef struct Sig_EvalLowerRight0 {
    __IO  uint32_t XEvalLowerRight0                        : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t YEvalLowerRight0                        : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  2;
} Sig_EvalLowerRight0_t;

/* SigCRCRedRef0 */
typedef struct Sig_SigCRCRedRef0 {
    __IO  uint32_t SigCRCRedRef0                           : 32;    /* RWS  */  /* 0xffffffff */
} Sig_SigCRCRedRef0_t;

/* SigCRCGreenRef0 */
typedef struct Sig_SigCRCGreenRef0 {
    __IO  uint32_t SigCRCGreenRef0                         : 32;    /* RWS  */  /* 0xffffffff */
} Sig_SigCRCGreenRef0_t;

/* SigCRCBlueRef0 */
typedef struct Sig_SigCRCBlueRef0 {
    __IO  uint32_t SigCRCBlueRef0                          : 32;    /* RWS  */  /* 0xffffffff */
} Sig_SigCRCBlueRef0_t;

/* SigCRCRed0 */
typedef struct Sig_SigCRCRed0 {
    __IO  uint32_t SigCRCRed0                              : 32;    /* R    */  /* 0xffffffff */
} Sig_SigCRCRed0_t;

/* SigCRCGreen0 */
typedef struct Sig_SigCRCGreen0 {
    __IO  uint32_t SigCRCGreen0                            : 32;    /* R    */  /* 0xffffffff */
} Sig_SigCRCGreen0_t;

/* SigCRCBlue0 */
typedef struct Sig_SigCRCBlue0 {
    __IO  uint32_t SigCRCBlue0                             : 32;    /* R    */  /* 0xffffffff */
} Sig_SigCRCBlue0_t;

/* EvalControl1 */
typedef struct Sig_EvalControl1 {
    __IO  uint32_t EnEvalWin1                              :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t EnCRC1                                  :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_03                             :  6;
    __IO  uint32_t AlphaMask1                              :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t AlphaInv1                               :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_06                             :  6;
    __IO  uint32_t EnLocalPanic1                           :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t EnGlobalPanic1                          :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_09                             : 14;
} Sig_EvalControl1_t;

/* EvalUpperLeft1 */
typedef struct Sig_EvalUpperLeft1 {
    __IO  uint32_t XEvalUpperLeft1                         : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t YEvalUpperLeft1                         : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  2;
} Sig_EvalUpperLeft1_t;

/* EvalLowerRight1 */
typedef struct Sig_EvalLowerRight1 {
    __IO  uint32_t XEvalLowerRight1                        : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t YEvalLowerRight1                        : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  2;
} Sig_EvalLowerRight1_t;

/* SigCRCRedRef1 */
typedef struct Sig_SigCRCRedRef1 {
    __IO  uint32_t SigCRCRedRef1                           : 32;    /* RWS  */  /* 0xffffffff */
} Sig_SigCRCRedRef1_t;

/* SigCRCGreenRef1 */
typedef struct Sig_SigCRCGreenRef1 {
    __IO  uint32_t SigCRCGreenRef1                         : 32;    /* RWS  */  /* 0xffffffff */
} Sig_SigCRCGreenRef1_t;

/* SigCRCBlueRef1 */
typedef struct Sig_SigCRCBlueRef1 {
    __IO  uint32_t SigCRCBlueRef1                          : 32;    /* RWS  */  /* 0xffffffff */
} Sig_SigCRCBlueRef1_t;

/* SigCRCRed1 */
typedef struct Sig_SigCRCRed1 {
    __IO  uint32_t SigCRCRed1                              : 32;    /* R    */  /* 0xffffffff */
} Sig_SigCRCRed1_t;

/* SigCRCGreen1 */
typedef struct Sig_SigCRCGreen1 {
    __IO  uint32_t SigCRCGreen1                            : 32;    /* R    */  /* 0xffffffff */
} Sig_SigCRCGreen1_t;

/* SigCRCBlue1 */
typedef struct Sig_SigCRCBlue1 {
    __IO  uint32_t SigCRCBlue1                             : 32;    /* R    */  /* 0xffffffff */
} Sig_SigCRCBlue1_t;

/* EvalControl2 */
typedef struct Sig_EvalControl2 {
    __IO  uint32_t EnEvalWin2                              :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t EnCRC2                                  :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_03                             :  6;
    __IO  uint32_t AlphaMask2                              :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t AlphaInv2                               :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_06                             :  6;
    __IO  uint32_t EnLocalPanic2                           :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t EnGlobalPanic2                          :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_09                             : 14;
} Sig_EvalControl2_t;

/* EvalUpperLeft2 */
typedef struct Sig_EvalUpperLeft2 {
    __IO  uint32_t XEvalUpperLeft2                         : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t YEvalUpperLeft2                         : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  2;
} Sig_EvalUpperLeft2_t;

/* EvalLowerRight2 */
typedef struct Sig_EvalLowerRight2 {
    __IO  uint32_t XEvalLowerRight2                        : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t YEvalLowerRight2                        : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  2;
} Sig_EvalLowerRight2_t;

/* SigCRCRedRef2 */
typedef struct Sig_SigCRCRedRef2 {
    __IO  uint32_t SigCRCRedRef2                           : 32;    /* RWS  */  /* 0xffffffff */
} Sig_SigCRCRedRef2_t;

/* SigCRCGreenRef2 */
typedef struct Sig_SigCRCGreenRef2 {
    __IO  uint32_t SigCRCGreenRef2                         : 32;    /* RWS  */  /* 0xffffffff */
} Sig_SigCRCGreenRef2_t;

/* SigCRCBlueRef2 */
typedef struct Sig_SigCRCBlueRef2 {
    __IO  uint32_t SigCRCBlueRef2                          : 32;    /* RWS  */  /* 0xffffffff */
} Sig_SigCRCBlueRef2_t;

/* SigCRCRed2 */
typedef struct Sig_SigCRCRed2 {
    __IO  uint32_t SigCRCRed2                              : 32;    /* R    */  /* 0xffffffff */
} Sig_SigCRCRed2_t;

/* SigCRCGreen2 */
typedef struct Sig_SigCRCGreen2 {
    __IO  uint32_t SigCRCGreen2                            : 32;    /* R    */  /* 0xffffffff */
} Sig_SigCRCGreen2_t;

/* SigCRCBlue2 */
typedef struct Sig_SigCRCBlue2 {
    __IO  uint32_t SigCRCBlue2                             : 32;    /* R    */  /* 0xffffffff */
} Sig_SigCRCBlue2_t;

/* EvalControl3 */
typedef struct Sig_EvalControl3 {
    __IO  uint32_t EnEvalWin3                              :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t EnCRC3                                  :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_03                             :  6;
    __IO  uint32_t AlphaMask3                              :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t AlphaInv3                               :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_06                             :  6;
    __IO  uint32_t EnLocalPanic3                           :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t EnGlobalPanic3                          :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_09                             : 14;
} Sig_EvalControl3_t;

/* EvalUpperLeft3 */
typedef struct Sig_EvalUpperLeft3 {
    __IO  uint32_t XEvalUpperLeft3                         : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t YEvalUpperLeft3                         : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  2;
} Sig_EvalUpperLeft3_t;

/* EvalLowerRight3 */
typedef struct Sig_EvalLowerRight3 {
    __IO  uint32_t XEvalLowerRight3                        : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t YEvalLowerRight3                        : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  2;
} Sig_EvalLowerRight3_t;

/* SigCRCRedRef3 */
typedef struct Sig_SigCRCRedRef3 {
    __IO  uint32_t SigCRCRedRef3                           : 32;    /* RWS  */  /* 0xffffffff */
} Sig_SigCRCRedRef3_t;

/* SigCRCGreenRef3 */
typedef struct Sig_SigCRCGreenRef3 {
    __IO  uint32_t SigCRCGreenRef3                         : 32;    /* RWS  */  /* 0xffffffff */
} Sig_SigCRCGreenRef3_t;

/* SigCRCBlueRef3 */
typedef struct Sig_SigCRCBlueRef3 {
    __IO  uint32_t SigCRCBlueRef3                          : 32;    /* RWS  */  /* 0xffffffff */
} Sig_SigCRCBlueRef3_t;

/* SigCRCRed3 */
typedef struct Sig_SigCRCRed3 {
    __IO  uint32_t SigCRCRed3                              : 32;    /* R    */  /* 0xffffffff */
} Sig_SigCRCRed3_t;

/* SigCRCGreen3 */
typedef struct Sig_SigCRCGreen3 {
    __IO  uint32_t SigCRCGreen3                            : 32;    /* R    */  /* 0xffffffff */
} Sig_SigCRCGreen3_t;

/* SigCRCBlue3 */
typedef struct Sig_SigCRCBlue3 {
    __IO  uint32_t SigCRCBlue3                             : 32;    /* R    */  /* 0xffffffff */
} Sig_SigCRCBlue3_t;

/* EvalControl4 */
typedef struct Sig_EvalControl4 {
    __IO  uint32_t EnEvalWin4                              :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t EnCRC4                                  :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_03                             :  6;
    __IO  uint32_t AlphaMask4                              :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t AlphaInv4                               :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_06                             :  6;
    __IO  uint32_t EnLocalPanic4                           :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t EnGlobalPanic4                          :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_09                             : 14;
} Sig_EvalControl4_t;

/* EvalUpperLeft4 */
typedef struct Sig_EvalUpperLeft4 {
    __IO  uint32_t XEvalUpperLeft4                         : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t YEvalUpperLeft4                         : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  2;
} Sig_EvalUpperLeft4_t;

/* EvalLowerRight4 */
typedef struct Sig_EvalLowerRight4 {
    __IO  uint32_t XEvalLowerRight4                        : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t YEvalLowerRight4                        : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  2;
} Sig_EvalLowerRight4_t;

/* SigCRCRedRef4 */
typedef struct Sig_SigCRCRedRef4 {
    __IO  uint32_t SigCRCRedRef4                           : 32;    /* RWS  */  /* 0xffffffff */
} Sig_SigCRCRedRef4_t;

/* SigCRCGreenRef4 */
typedef struct Sig_SigCRCGreenRef4 {
    __IO  uint32_t SigCRCGreenRef4                         : 32;    /* RWS  */  /* 0xffffffff */
} Sig_SigCRCGreenRef4_t;

/* SigCRCBlueRef4 */
typedef struct Sig_SigCRCBlueRef4 {
    __IO  uint32_t SigCRCBlueRef4                          : 32;    /* RWS  */  /* 0xffffffff */
} Sig_SigCRCBlueRef4_t;

/* SigCRCRed4 */
typedef struct Sig_SigCRCRed4 {
    __IO  uint32_t SigCRCRed4                              : 32;    /* R    */  /* 0xffffffff */
} Sig_SigCRCRed4_t;

/* SigCRCGreen4 */
typedef struct Sig_SigCRCGreen4 {
    __IO  uint32_t SigCRCGreen4                            : 32;    /* R    */  /* 0xffffffff */
} Sig_SigCRCGreen4_t;

/* SigCRCBlue4 */
typedef struct Sig_SigCRCBlue4 {
    __IO  uint32_t SigCRCBlue4                             : 32;    /* R    */  /* 0xffffffff */
} Sig_SigCRCBlue4_t;

/* EvalControl5 */
typedef struct Sig_EvalControl5 {
    __IO  uint32_t EnEvalWin5                              :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t EnCRC5                                  :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_03                             :  6;
    __IO  uint32_t AlphaMask5                              :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t AlphaInv5                               :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_06                             :  6;
    __IO  uint32_t EnLocalPanic5                           :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t EnGlobalPanic5                          :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_09                             : 14;
} Sig_EvalControl5_t;

/* EvalUpperLeft5 */
typedef struct Sig_EvalUpperLeft5 {
    __IO  uint32_t XEvalUpperLeft5                         : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t YEvalUpperLeft5                         : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  2;
} Sig_EvalUpperLeft5_t;

/* EvalLowerRight5 */
typedef struct Sig_EvalLowerRight5 {
    __IO  uint32_t XEvalLowerRight5                        : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t YEvalLowerRight5                        : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  2;
} Sig_EvalLowerRight5_t;

/* SigCRCRedRef5 */
typedef struct Sig_SigCRCRedRef5 {
    __IO  uint32_t SigCRCRedRef5                           : 32;    /* RWS  */  /* 0xffffffff */
} Sig_SigCRCRedRef5_t;

/* SigCRCGreenRef5 */
typedef struct Sig_SigCRCGreenRef5 {
    __IO  uint32_t SigCRCGreenRef5                         : 32;    /* RWS  */  /* 0xffffffff */
} Sig_SigCRCGreenRef5_t;

/* SigCRCBlueRef5 */
typedef struct Sig_SigCRCBlueRef5 {
    __IO  uint32_t SigCRCBlueRef5                          : 32;    /* RWS  */  /* 0xffffffff */
} Sig_SigCRCBlueRef5_t;

/* SigCRCRed5 */
typedef struct Sig_SigCRCRed5 {
    __IO  uint32_t SigCRCRed5                              : 32;    /* R    */  /* 0xffffffff */
} Sig_SigCRCRed5_t;

/* SigCRCGreen5 */
typedef struct Sig_SigCRCGreen5 {
    __IO  uint32_t SigCRCGreen5                            : 32;    /* R    */  /* 0xffffffff */
} Sig_SigCRCGreen5_t;

/* SigCRCBlue5 */
typedef struct Sig_SigCRCBlue5 {
    __IO  uint32_t SigCRCBlue5                             : 32;    /* R    */  /* 0xffffffff */
} Sig_SigCRCBlue5_t;

/* EvalControl6 */
typedef struct Sig_EvalControl6 {
    __IO  uint32_t EnEvalWin6                              :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t EnCRC6                                  :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_03                             :  6;
    __IO  uint32_t AlphaMask6                              :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t AlphaInv6                               :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_06                             :  6;
    __IO  uint32_t EnLocalPanic6                           :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t EnGlobalPanic6                          :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_09                             : 14;
} Sig_EvalControl6_t;

/* EvalUpperLeft6 */
typedef struct Sig_EvalUpperLeft6 {
    __IO  uint32_t XEvalUpperLeft6                         : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t YEvalUpperLeft6                         : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  2;
} Sig_EvalUpperLeft6_t;

/* EvalLowerRight6 */
typedef struct Sig_EvalLowerRight6 {
    __IO  uint32_t XEvalLowerRight6                        : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t YEvalLowerRight6                        : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  2;
} Sig_EvalLowerRight6_t;

/* SigCRCRedRef6 */
typedef struct Sig_SigCRCRedRef6 {
    __IO  uint32_t SigCRCRedRef6                           : 32;    /* RWS  */  /* 0xffffffff */
} Sig_SigCRCRedRef6_t;

/* SigCRCGreenRef6 */
typedef struct Sig_SigCRCGreenRef6 {
    __IO  uint32_t SigCRCGreenRef6                         : 32;    /* RWS  */  /* 0xffffffff */
} Sig_SigCRCGreenRef6_t;

/* SigCRCBlueRef6 */
typedef struct Sig_SigCRCBlueRef6 {
    __IO  uint32_t SigCRCBlueRef6                          : 32;    /* RWS  */  /* 0xffffffff */
} Sig_SigCRCBlueRef6_t;

/* SigCRCRed6 */
typedef struct Sig_SigCRCRed6 {
    __IO  uint32_t SigCRCRed6                              : 32;    /* R    */  /* 0xffffffff */
} Sig_SigCRCRed6_t;

/* SigCRCGreen6 */
typedef struct Sig_SigCRCGreen6 {
    __IO  uint32_t SigCRCGreen6                            : 32;    /* R    */  /* 0xffffffff */
} Sig_SigCRCGreen6_t;

/* SigCRCBlue6 */
typedef struct Sig_SigCRCBlue6 {
    __IO  uint32_t SigCRCBlue6                             : 32;    /* R    */  /* 0xffffffff */
} Sig_SigCRCBlue6_t;

/* EvalControl7 */
typedef struct Sig_EvalControl7 {
    __IO  uint32_t EnEvalWin7                              :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t EnCRC7                                  :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_03                             :  6;
    __IO  uint32_t AlphaMask7                              :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t AlphaInv7                               :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_06                             :  6;
    __IO  uint32_t EnLocalPanic7                           :  1;    /* RWS  */  /* 0x0 */
    __IO  uint32_t EnGlobalPanic7                          :  1;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_09                             : 14;
} Sig_EvalControl7_t;

/* EvalUpperLeft7 */
typedef struct Sig_EvalUpperLeft7 {
    __IO  uint32_t XEvalUpperLeft7                         : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t YEvalUpperLeft7                         : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  2;
} Sig_EvalUpperLeft7_t;

/* EvalLowerRight7 */
typedef struct Sig_EvalLowerRight7 {
    __IO  uint32_t XEvalLowerRight7                        : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_02                             :  2;
    __IO  uint32_t YEvalLowerRight7                        : 14;    /* RWS  */  /* 0x0 */
          uint32_t RESERVED_04                             :  2;
} Sig_EvalLowerRight7_t;

/* SigCRCRedRef7 */
typedef struct Sig_SigCRCRedRef7 {
    __IO  uint32_t SigCRCRedRef7                           : 32;    /* RWS  */  /* 0xffffffff */
} Sig_SigCRCRedRef7_t;

/* SigCRCGreenRef7 */
typedef struct Sig_SigCRCGreenRef7 {
    __IO  uint32_t SigCRCGreenRef7                         : 32;    /* RWS  */  /* 0xffffffff */
} Sig_SigCRCGreenRef7_t;

/* SigCRCBlueRef7 */
typedef struct Sig_SigCRCBlueRef7 {
    __IO  uint32_t SigCRCBlueRef7                          : 32;    /* RWS  */  /* 0xffffffff */
} Sig_SigCRCBlueRef7_t;

/* SigCRCRed7 */
typedef struct Sig_SigCRCRed7 {
    __IO  uint32_t SigCRCRed7                              : 32;    /* R    */  /* 0xffffffff */
} Sig_SigCRCRed7_t;

/* SigCRCGreen7 */
typedef struct Sig_SigCRCGreen7 {
    __IO  uint32_t SigCRCGreen7                            : 32;    /* R    */  /* 0xffffffff */
} Sig_SigCRCGreen7_t;

/* SigCRCBlue7 */
typedef struct Sig_SigCRCBlue7 {
    __IO  uint32_t SigCRCBlue7                             : 32;    /* R    */  /* 0xffffffff */
} Sig_SigCRCBlue7_t;

/* ShadowLoad */
typedef struct Sig_ShadowLoad {
    __IO  uint32_t ShdLdReq                                :  8;    /* RWX  */  /* 0x0 */
          uint32_t RESERVED_02                             : 24;
} Sig_ShadowLoad_t;

/* ContinuousMode */
typedef struct Sig_ContinuousMode {
    __IO  uint32_t EnCont                                  :  1;    /* RW   */  /* 0x0 */
          uint32_t RESERVED_02                             : 31;
} Sig_ContinuousMode_t;

/* SoftwareKick */
typedef struct Sig_SoftwareKick {
    __IO  uint32_t Kick                                    :  1;    /* W1P  */  /* 0x0 */
          uint32_t RESERVED_02                             : 31;
} Sig_SoftwareKick_t;

/* Status */
typedef struct Sig_Status {
    __IO  uint32_t StsSigError                             :  8;    /* R    */  /* 0x0 */
          uint32_t RESERVED_02                             :  8;
    __IO  uint32_t StsSigValid                             :  1;    /* R    */  /* 0x0 */
          uint32_t RESERVED_04                             :  3;
    __IO  uint32_t StsSigIdle                              :  1;    /* R    */  /* 0x1 */
          uint32_t RESERVED_06                             : 11;
} Sig_Status_t;

/******************************************************************************
 * IRIS_SDL2 registers
 ******************************************************************************/
/* ----[ SubSysCtrl ]-------------------------------------------------------------------- */
typedef struct {
    /* 3.1.2.1 LockUnlock */
    union {
        __IO  uint32_t                              LockUnlock;
        SubSysCtrl_LockUnlock_t                     LockUnlock_f;
    };

    /* 3.1.2.2 LockStatus */
    union {
        __IO  uint32_t                              LockStatus;
        SubSysCtrl_LockStatus_t                     LockStatus_f;
    };

    /* 3.1.2.3 IPIdentifier */
    union {
        __IO  uint32_t                              IPIdentifier;
        SubSysCtrl_IPIdentifier_t                   IPIdentifier_f;
    };

    /* 3.1.2.4 ConfigClockControl */
    union {
        __IO  uint32_t                              ConfigClockControl;
        SubSysCtrl_ConfigClockControl_t             ConfigClockControl_f;
    };

    uint32_t                                        RESERVED_0010[4];

    /* 3.1.2.5 ConfigMemorySelect */
    union {
        __IO  uint32_t                              ConfigMemorySelect;
        SubSysCtrl_ConfigMemorySelect_t             ConfigMemorySelect_f;
    };

    /* 3.1.2.6 ConfigVramRemap */
    union {
        __IO  uint32_t                              ConfigVramRemap;
        SubSysCtrl_ConfigVramRemap_t                ConfigVramRemap_f;
    };

    /* 3.1.2.7 ConfigPanicSwitch */
    union {
        __IO  uint32_t                              ConfigPanicSwitch;
        SubSysCtrl_ConfigPanicSwitch_t              ConfigPanicSwitch_f;
    };

    /* 3.1.2.19 AXI_ClockDivider */
    union {
        __IO  uint32_t                              AXI_ClockDivider;
        SubSysCtrl_AXI_ClockDivider_t               AXI_ClockDivider_f;
    };

    uint32_t                                        RESERVED_002C[4];

    /* 3.1.2.8 dsp_LockUnlock */
    union {
        __IO  uint32_t                              dsp_LockUnlock;
        SubSysCtrl_dsp_LockUnlock_t                 dsp_LockUnlock_f;
    };

    /* 3.1.2.9 LockStatus */
    union {
        __IO  uint32_t                              dsp_LockStatus;
        SubSysCtrl_dsp_LockStatus_t                 dsp_LockStatus_f;
    };

    /* 3.1.2.10 dsp0_ClockDivider */
    union {
        __IO  uint32_t                              dsp0_ClockDivider;
        SubSysCtrl_dsp0_ClockDivider_t              dsp0_ClockDivider_f;
    };

    /* 3.1.2.11 dsp0_DomainControl */
    union {
        __IO  uint32_t                              dsp0_DomainControl;
        SubSysCtrl_dsp0_DomainControl_t             dsp0_DomainControl_f;
    };

    /* 3.1.2.12 dsp0_ClockShift */
    union {
        __IO  uint32_t                              dsp0_ClockShift;
        SubSysCtrl_dsp0_ClockShift_t                dsp0_ClockShift_f;
    };

    /* 3.1.2.13 dsp0_LineEndControl */
    union {
        __IO  uint32_t                              dsp0_LineEndControl;
        SubSysCtrl_dsp0_LineEndControl_t            dsp0_LineEndControl_f;
    };

    /* 3.1.2.14 dsp0_PowerEnControl */
    union {
        __IO  uint32_t                              dsp0_PowerEnControl;
        SubSysCtrl_dsp0_PowerEnControl_t            dsp0_PowerEnControl_f;
    };

    uint32_t                                        RESERVED_0054[7];

    /* 3.1.2.13 SDRAMC_ClockDivider */
    union {
        __IO  uint32_t                              SDRAMC_ClockDivider;
        SubSysCtrl_SDRAMC_ClockDivider_t            SDRAMC_ClockDivider_f;
    };

    /* 3.1.2.14 SDRAMC_DomainControl */
    union {
        __IO  uint32_t                              SDRAMC_DomainControl;
        SubSysCtrl_SDRAMC_DomainControl_t           SDRAMC_DomainControl_f;
    };

    /* 3.1.2.15 HSSPIC_ClockDivider */
    union {
        __IO  uint32_t                              HSSPIC_ClockDivider;
        SubSysCtrl_HSSPIC_ClockDivider_t            HSSPIC_ClockDivider_f;
    };

    /* 3.1.2.16 HSSPIC_DomainControl */
    union {
        __IO  uint32_t                              HSSPIC_DomainControl;
        SubSysCtrl_HSSPIC_DomainControl_t           HSSPIC_DomainControl_f;
    };

    /* 3.1.2.17 RPCC_ClockDivider */
    union {
        __IO  uint32_t                              RPCC_ClockDivider;
        SubSysCtrl_RPCC_ClockDivider_t              RPCC_ClockDivider_f;
    };

    /* 3.1.2.18 RPCC_DomainControl */
    union {
        __IO  uint32_t                              RPCC_DomainControl;
        SubSysCtrl_RPCC_DomainControl_t             RPCC_DomainControl_f;
    };

    uint32_t                                        RESERVED_0094[28];

    /* 3.1.2.20 vram_LockUnlock */
    union {
        __IO  uint32_t                              vram_LockUnlock;
        SubSysCtrl_vram_LockUnlock_t                vram_LockUnlock_f;
    };

    /* 3.1.2.21 vram_LockStatus */
    union {
        __IO  uint32_t                              vram_LockStatus;
        SubSysCtrl_vram_LockStatus_t                vram_LockStatus_f;
    };

    uint32_t                                        RESERVED_0108[16];

    /* 3.1.2.22 vram_arbiter_priority */
    union {
        __IO  uint32_t                              vram_arbiter_priority;
        SubSysCtrl_vram_arbiter_priority_t          vram_arbiter_priority_f;
    };

} IRIS_SDL2_SubSysCtrl_TypeDef;
/******************************************************************************
 * IRIS_MDL2 registers
 ******************************************************************************/
/* ----[ ComCtrl ]----------------------------------------------------------------------- */
typedef struct {
    /* 3.6.2.1 IPIdentifier */
    union {
        __IO  uint32_t                              IPIdentifier;
        ComCtrl_IPIdentifier_t                      IPIdentifier_f;
    };

    uint32_t                                        RESERVED_0004[7];

    /* 3.6.2.2 LockUnlock */
    union {
        __IO  uint32_t                              LockUnlock;
        ComCtrl_LockUnlock_t                        LockUnlock_f;
    };

    /* 3.6.2.3 LockStatus */
    union {
        __IO  uint32_t                              LockStatus;
        ComCtrl_LockStatus_t                        LockStatus_f;
    };

    /* 3.6.2.4 UserInterruptMask0 */
    union {
        __IO  uint32_t                              UserInterruptMask0;
        ComCtrl_UserInterruptMask0_t                UserInterruptMask0_f;
    };

    /* 3.6.2.5 InterruptEnable0 */
    union {
        __IO  uint32_t                              InterruptEnable0;
        ComCtrl_InterruptEnable0_t                  InterruptEnable0_f;
    };

    /* 3.6.2.6 InterruptPreset0 */
    union {
        __IO  uint32_t                              InterruptPreset0;
        ComCtrl_InterruptPreset0_t                  InterruptPreset0_f;
    };

    /* 3.6.2.7 InterruptClear0 */
    union {
        __IO  uint32_t                              InterruptClear0;
        ComCtrl_InterruptClear0_t                   InterruptClear0_f;
    };

    /* 3.6.2.8 InterruptStatus0 */
    union {
        __IO  uint32_t                              InterruptStatus0;
        ComCtrl_InterruptStatus0_t                  InterruptStatus0_f;
    };

    uint32_t                                        RESERVED_003C;

    /* 3.6.2.9 UserInterruptEnable0 */
    union {
        __IO  uint32_t                              UserInterruptEnable0;
        ComCtrl_UserInterruptEnable0_t              UserInterruptEnable0_f;
    };

    /* 3.6.2.10 UserInterruptPreset0 */
    union {
        __IO  uint32_t                              UserInterruptPreset0;
        ComCtrl_UserInterruptPreset0_t              UserInterruptPreset0_f;
    };

    /* 3.6.2.11 UserInterruptClear0 */
    union {
        __IO  uint32_t                              UserInterruptClear0;
        ComCtrl_UserInterruptClear0_t               UserInterruptClear0_f;
    };

    /* 3.6.2.12 UserInterruptStatus0 */
    union {
        __IO  uint32_t                              UserInterruptStatus0;
        ComCtrl_UserInterruptStatus0_t              UserInterruptStatus0_f;
    };

    uint32_t                                        RESERVED_0050[12];

    /* 3.6.2.13 GeneralPurpose[0..31] */
    union {
        __IO  uint32_t                              GeneralPurpose[32];
        ComCtrl_GeneralPurpose_t                    GeneralPurpose_f[32];
    };

} IRIS_MDL2_ComCtrl_TypeDef;


/* ----[ CmdSeq ]------------------------------------------------------------------------ */
typedef struct {
    /* 3.6.3.1 HIF[0..63] */
    union {
        __IO  uint32_t                              HIF[64];
        CmdSeq_HIF_t                                HIF_f[64];
    };

    /* 3.6.3.2 LockUnlockHIF */
    union {
        __IO  uint32_t                              LockUnlockHIF;
        CmdSeq_LockUnlockHIF_t                      LockUnlockHIF_f;
    };

    /* 3.6.3.3 LockStatusHIF */
    union {
        __IO  uint32_t                              LockStatusHIF;
        CmdSeq_LockStatusHIF_t                      LockStatusHIF_f;
    };

    uint32_t                                        RESERVED_0108[30];

    /* 3.6.3.4 LockUnlock */
    union {
        __IO  uint32_t                              LockUnlock;
        CmdSeq_LockUnlock_t                         LockUnlock_f;
    };

    /* 3.6.3.5 LockStatus */
    union {
        __IO  uint32_t                              LockStatus;
        CmdSeq_LockStatus_t                         LockStatus_f;
    };

    /* 3.6.3.6 BufferAddress */
    union {
        __IO  uint32_t                              BufferAddress;
        CmdSeq_BufferAddress_t                      BufferAddress_f;
    };

    /* 3.6.3.7 BufferSize */
    union {
        __IO  uint32_t                              BufferSize;
        CmdSeq_BufferSize_t                         BufferSize_f;
    };

    /* 3.6.3.8 WatermarkControl */
    union {
        __IO  uint32_t                              WatermarkControl;
        CmdSeq_WatermarkControl_t                   WatermarkControl_f;
    };

    /* 3.6.3.9 Control */
    union {
        __IO  uint32_t                              Control;
        CmdSeq_Control_t                            Control_f;
    };

    /* 3.6.3.10 Status */
    union {
        __IO  uint32_t                              Status;
        CmdSeq_Status_t                             Status_f;
    };

} IRIS_MDL2_CmdSeq_TypeDef;


/* ----[ PixEngCfg ]--------------------------------------------------------------------- */
typedef struct {
    /* 3.6.4.1 SafetyLockUnlock */
    union {
        __IO  uint32_t                              SafetyLockUnlock;
        PixEngCfg_SafetyLockUnlock_t                SafetyLockUnlock_f;
    };

    /* 3.6.4.2 SafetyLockStatus */
    union {
        __IO  uint32_t                              SafetyLockStatus;
        PixEngCfg_SafetyLockStatus_t                SafetyLockStatus_f;
    };

    /* 3.6.4.3 store9_SafetyMask */
    union {
        __IO  uint32_t                              store9_SafetyMask;
        PixEngCfg_store9_SafetyMask_t               store9_SafetyMask_f;
    };

    /* 3.6.4.4 extdst0_SafetyMask */
    union {
        __IO  uint32_t                              extdst0_SafetyMask;
        PixEngCfg_extdst0_SafetyMask_t              extdst0_SafetyMask_f;
    };

    /* 3.6.4.5 extdst4_SafetyMask */
    union {
        __IO  uint32_t                              extdst4_SafetyMask;
        PixEngCfg_extdst4_SafetyMask_t              extdst4_SafetyMask_f;
    };

    uint32_t                                        RESERVED_0014[3];

    /* 3.6.4.6 fetchdecode9_LockUnlock */
    union {
        __IO  uint32_t                              fetchdecode9_LockUnlock;
        PixEngCfg_fetchdecode9_LockUnlock_t         fetchdecode9_LockUnlock_f;
    };

    /* 3.6.4.7 fetchdecode9_LockStatus */
    union {
        __IO  uint32_t                              fetchdecode9_LockStatus;
        PixEngCfg_fetchdecode9_LockStatus_t         fetchdecode9_LockStatus_f;
    };

    /* 3.6.4.8 fetchdecode9_Dynamic */
    union {
        __IO  uint32_t                              fetchdecode9_Dynamic;
        PixEngCfg_fetchdecode9_Dynamic_t            fetchdecode9_Dynamic_f;
    };

    /* 3.6.4.9 fetchdecode9_Status */
    union {
        __IO  uint32_t                              fetchdecode9_Status;
        PixEngCfg_fetchdecode9_Status_t             fetchdecode9_Status_f;
    };

    uint32_t                                        RESERVED_0030[4];

    /* 3.6.4.10 fetchrot9_LockUnlock */
    union {
        __IO  uint32_t                              fetchrot9_LockUnlock;
        PixEngCfg_fetchrot9_LockUnlock_t            fetchrot9_LockUnlock_f;
    };

    /* 3.6.4.11 fetchrot9_LockStatus */
    union {
        __IO  uint32_t                              fetchrot9_LockStatus;
        PixEngCfg_fetchrot9_LockStatus_t            fetchrot9_LockStatus_f;
    };

    /* 3.6.4.12 fetchrot9_Dynamic */
    union {
        __IO  uint32_t                              fetchrot9_Dynamic;
        PixEngCfg_fetchrot9_Dynamic_t               fetchrot9_Dynamic_f;
    };

    /* 3.6.4.13 fetchrot9_Status */
    union {
        __IO  uint32_t                              fetchrot9_Status;
        PixEngCfg_fetchrot9_Status_t                fetchrot9_Status_f;
    };

    /* 3.6.4.14 fetcheco9_LockUnlock */
    union {
        __IO  uint32_t                              fetcheco9_LockUnlock;
        PixEngCfg_fetcheco9_LockUnlock_t            fetcheco9_LockUnlock_f;
    };

    /* 3.6.4.15 fetcheco9_LockStatus */
    union {
        __IO  uint32_t                              fetcheco9_LockStatus;
        PixEngCfg_fetcheco9_LockStatus_t            fetcheco9_LockStatus_f;
    };

    /* 3.6.4.16 fetcheco9_Status */
    union {
        __IO  uint32_t                              fetcheco9_Status;
        PixEngCfg_fetcheco9_Status_t                fetcheco9_Status_f;
    };

    uint32_t                                        RESERVED_005C;

    /* 3.6.4.17 rop9_LockUnlock */
    union {
        __IO  uint32_t                              rop9_LockUnlock;
        PixEngCfg_rop9_LockUnlock_t                 rop9_LockUnlock_f;
    };

    /* 3.6.4.18 rop9_LockStatus */
    union {
        __IO  uint32_t                              rop9_LockStatus;
        PixEngCfg_rop9_LockStatus_t                 rop9_LockStatus_f;
    };

    /* 3.6.4.19 rop9_Dynamic */
    union {
        __IO  uint32_t                              rop9_Dynamic;
        PixEngCfg_rop9_Dynamic_t                    rop9_Dynamic_f;
    };

    /* 3.6.4.20 rop9_Status */
    union {
        __IO  uint32_t                              rop9_Status;
        PixEngCfg_rop9_Status_t                     rop9_Status_f;
    };

    uint32_t                                        RESERVED_0070[4];

    /* 3.6.4.21 clut9_LockUnlock */
    union {
        __IO  uint32_t                              clut9_LockUnlock;
        PixEngCfg_clut9_LockUnlock_t                clut9_LockUnlock_f;
    };

    /* 3.6.4.22 clut9_LockStatus */
    union {
        __IO  uint32_t                              clut9_LockStatus;
        PixEngCfg_clut9_LockStatus_t                clut9_LockStatus_f;
    };

    /* 3.6.4.23 clut9_Dynamic */
    union {
        __IO  uint32_t                              clut9_Dynamic;
        PixEngCfg_clut9_Dynamic_t                   clut9_Dynamic_f;
    };

    /* 3.6.4.24 clut9_Status */
    union {
        __IO  uint32_t                              clut9_Status;
        PixEngCfg_clut9_Status_t                    clut9_Status_f;
    };

    uint32_t                                        RESERVED_0090[4];

    /* 3.6.4.25 matrix9_LockUnlock */
    union {
        __IO  uint32_t                              matrix9_LockUnlock;
        PixEngCfg_matrix9_LockUnlock_t              matrix9_LockUnlock_f;
    };

    /* 3.6.4.26 matrix9_LockStatus */
    union {
        __IO  uint32_t                              matrix9_LockStatus;
        PixEngCfg_matrix9_LockStatus_t              matrix9_LockStatus_f;
    };

    /* 3.6.4.27 matrix9_Dynamic */
    union {
        __IO  uint32_t                              matrix9_Dynamic;
        PixEngCfg_matrix9_Dynamic_t                 matrix9_Dynamic_f;
    };

    /* 3.6.4.28 matrix9_Status */
    union {
        __IO  uint32_t                              matrix9_Status;
        PixEngCfg_matrix9_Status_t                  matrix9_Status_f;
    };

    uint32_t                                        RESERVED_00B0[4];

    /* 3.6.4.29 blitblend9_LockUnlock */
    union {
        __IO  uint32_t                              blitblend9_LockUnlock;
        PixEngCfg_blitblend9_LockUnlock_t           blitblend9_LockUnlock_f;
    };

    /* 3.6.4.30 blitblend9_LockStatus */
    union {
        __IO  uint32_t                              blitblend9_LockStatus;
        PixEngCfg_blitblend9_LockStatus_t           blitblend9_LockStatus_f;
    };

    /* 3.6.4.31 blitblend9_Dynamic */
    union {
        __IO  uint32_t                              blitblend9_Dynamic;
        PixEngCfg_blitblend9_Dynamic_t              blitblend9_Dynamic_f;
    };

    /* 3.6.4.32 blitblend9_Status */
    union {
        __IO  uint32_t                              blitblend9_Status;
        PixEngCfg_blitblend9_Status_t               blitblend9_Status_f;
    };

    uint32_t                                        RESERVED_00D0[4];

    /* 3.6.4.33 store9_LockUnlock */
    union {
        __IO  uint32_t                              store9_LockUnlock;
        PixEngCfg_store9_LockUnlock_t               store9_LockUnlock_f;
    };

    /* 3.6.4.34 store9_LockStatus */
    union {
        __IO  uint32_t                              store9_LockStatus;
        PixEngCfg_store9_LockStatus_t               store9_LockStatus_f;
    };

    /* 3.6.4.35 store9_Static */
    union {
        __IO  uint32_t                              store9_Static;
        PixEngCfg_store9_Static_t                   store9_Static_f;
    };

    /* 3.6.4.36 store9_Dynamic */
    union {
        __IO  uint32_t                              store9_Dynamic;
        PixEngCfg_store9_Dynamic_t                  store9_Dynamic_f;
    };

    /* 3.6.4.37 store9_Request */
    union {
        __IO  uint32_t                              store9_Request;
        PixEngCfg_store9_Request_t                  store9_Request_f;
    };

    /* 3.6.4.38 store9_Trigger */
    union {
        __IO  uint32_t                              store9_Trigger;
        PixEngCfg_store9_Trigger_t                  store9_Trigger_f;
    };

    /* 3.6.4.39 store9_Status */
    union {
        __IO  uint32_t                              store9_Status;
        PixEngCfg_store9_Status_t                   store9_Status_f;
    };

    uint32_t                                        RESERVED_00FC;

    /* 3.6.4.40 constframe0_LockUnlock */
    union {
        __IO  uint32_t                              constframe0_LockUnlock;
        PixEngCfg_constframe0_LockUnlock_t          constframe0_LockUnlock_f;
    };

    /* 3.6.4.41 constframe0_LockStatus */
    union {
        __IO  uint32_t                              constframe0_LockStatus;
        PixEngCfg_constframe0_LockStatus_t          constframe0_LockStatus_f;
    };

    /* 3.6.4.42 constframe0_Status */
    union {
        __IO  uint32_t                              constframe0_Status;
        PixEngCfg_constframe0_Status_t              constframe0_Status_f;
    };

    uint32_t                                        RESERVED_010C[5];

    /* 3.6.4.43 extdst0_LockUnlock */
    union {
        __IO  uint32_t                              extdst0_LockUnlock;
        PixEngCfg_extdst0_LockUnlock_t              extdst0_LockUnlock_f;
    };

    /* 3.6.4.44 extdst0_LockStatus */
    union {
        __IO  uint32_t                              extdst0_LockStatus;
        PixEngCfg_extdst0_LockStatus_t              extdst0_LockStatus_f;
    };

    /* 3.6.4.45 extdst0_Static */
    union {
        __IO  uint32_t                              extdst0_Static;
        PixEngCfg_extdst0_Static_t                  extdst0_Static_f;
    };

    /* 3.6.4.46 extdst0_Dynamic */
    union {
        __IO  uint32_t                              extdst0_Dynamic;
        PixEngCfg_extdst0_Dynamic_t                 extdst0_Dynamic_f;
    };

    /* 3.6.4.47 extdst0_Request */
    union {
        __IO  uint32_t                              extdst0_Request;
        PixEngCfg_extdst0_Request_t                 extdst0_Request_f;
    };

    /* 3.6.4.48 extdst0_Trigger */
    union {
        __IO  uint32_t                              extdst0_Trigger;
        PixEngCfg_extdst0_Trigger_t                 extdst0_Trigger_f;
    };

    /* 3.6.4.49 extdst0_Status */
    union {
        __IO  uint32_t                              extdst0_Status;
        PixEngCfg_extdst0_Status_t                  extdst0_Status_f;
    };

    uint32_t                                        RESERVED_013C;

    /* 3.6.4.50 constframe4_LockUnlock */
    union {
        __IO  uint32_t                              constframe4_LockUnlock;
        PixEngCfg_constframe4_LockUnlock_t          constframe4_LockUnlock_f;
    };

    /* 3.6.4.51 constframe4_LockStatus */
    union {
        __IO  uint32_t                              constframe4_LockStatus;
        PixEngCfg_constframe4_LockStatus_t          constframe4_LockStatus_f;
    };

    /* 3.6.4.52 constframe4_Status */
    union {
        __IO  uint32_t                              constframe4_Status;
        PixEngCfg_constframe4_Status_t              constframe4_Status_f;
    };

    uint32_t                                        RESERVED_014C[5];

    /* 3.6.4.53 extdst4_LockUnlock */
    union {
        __IO  uint32_t                              extdst4_LockUnlock;
        PixEngCfg_extdst4_LockUnlock_t              extdst4_LockUnlock_f;
    };

    /* 3.6.4.54 extdst4_LockStatus */
    union {
        __IO  uint32_t                              extdst4_LockStatus;
        PixEngCfg_extdst4_LockStatus_t              extdst4_LockStatus_f;
    };

    /* 3.6.4.55 extdst4_Static */
    union {
        __IO  uint32_t                              extdst4_Static;
        PixEngCfg_extdst4_Static_t                  extdst4_Static_f;
    };

    /* 3.6.4.56 extdst4_Dynamic */
    union {
        __IO  uint32_t                              extdst4_Dynamic;
        PixEngCfg_extdst4_Dynamic_t                 extdst4_Dynamic_f;
    };

    /* 3.6.4.57 extdst4_Request */
    union {
        __IO  uint32_t                              extdst4_Request;
        PixEngCfg_extdst4_Request_t                 extdst4_Request_f;
    };

    /* 3.6.4.58 extdst4_Trigger */
    union {
        __IO  uint32_t                              extdst4_Trigger;
        PixEngCfg_extdst4_Trigger_t                 extdst4_Trigger_f;
    };

    /* 3.6.4.59 extdst4_Status */
    union {
        __IO  uint32_t                              extdst4_Status;
        PixEngCfg_extdst4_Status_t                  extdst4_Status_f;
    };

    uint32_t                                        RESERVED_017C;

    /* 3.6.4.60 fetchdecode0_LockUnlock */
    union {
        __IO  uint32_t                              fetchdecode0_LockUnlock;
        PixEngCfg_fetchdecode0_LockUnlock_t         fetchdecode0_LockUnlock_f;
    };

    /* 3.6.4.61 fetchdecode0_LockStatus */
    union {
        __IO  uint32_t                              fetchdecode0_LockStatus;
        PixEngCfg_fetchdecode0_LockStatus_t         fetchdecode0_LockStatus_f;
    };

    /* 3.6.4.62 fetchdecode0_Dynamic */
    union {
        __IO  uint32_t                              fetchdecode0_Dynamic;
        PixEngCfg_fetchdecode0_Dynamic_t            fetchdecode0_Dynamic_f;
    };

    /* 3.6.4.63 fetchdecode0_Status */
    union {
        __IO  uint32_t                              fetchdecode0_Status;
        PixEngCfg_fetchdecode0_Status_t             fetchdecode0_Status_f;
    };

    /* 3.6.4.64 fetchlayer0_LockUnlock */
    union {
        __IO  uint32_t                              fetchlayer0_LockUnlock;
        PixEngCfg_fetchlayer0_LockUnlock_t          fetchlayer0_LockUnlock_f;
    };

    /* 3.6.4.65 fetchlayer0_LockStatus */
    union {
        __IO  uint32_t                              fetchlayer0_LockStatus;
        PixEngCfg_fetchlayer0_LockStatus_t          fetchlayer0_LockStatus_f;
    };

    /* 3.6.4.66 fetchlayer0_Status */
    union {
        __IO  uint32_t                              fetchlayer0_Status;
        PixEngCfg_fetchlayer0_Status_t              fetchlayer0_Status_f;
    };

    uint32_t                                        RESERVED_019C;

    /* 3.6.4.67 layerblend1_LockUnlock */
    union {
        __IO  uint32_t                              layerblend1_LockUnlock;
        PixEngCfg_layerblend1_LockUnlock_t          layerblend1_LockUnlock_f;
    };

    /* 3.6.4.68 layerblend1_LockStatus */
    union {
        __IO  uint32_t                              layerblend1_LockStatus;
        PixEngCfg_layerblend1_LockStatus_t          layerblend1_LockStatus_f;
    };

    /* 3.6.4.69 layerblend1_Dynamic */
    union {
        __IO  uint32_t                              layerblend1_Dynamic;
        PixEngCfg_layerblend1_Dynamic_t             layerblend1_Dynamic_f;
    };

    /* 3.6.4.70 layerblend1_Status */
    union {
        __IO  uint32_t                              layerblend1_Status;
        PixEngCfg_layerblend1_Status_t              layerblend1_Status_f;
    };

    uint32_t                                        RESERVED_01B0[4];

    /* 3.6.4.71 layerblend2_LockUnlock */
    union {
        __IO  uint32_t                              layerblend2_LockUnlock;
        PixEngCfg_layerblend2_LockUnlock_t          layerblend2_LockUnlock_f;
    };

    /* 3.6.4.72 layerblend2_LockStatus */
    union {
        __IO  uint32_t                              layerblend2_LockStatus;
        PixEngCfg_layerblend2_LockStatus_t          layerblend2_LockStatus_f;
    };

    /* 3.6.4.73 layerblend2_Dynamic */
    union {
        __IO  uint32_t                              layerblend2_Dynamic;
        PixEngCfg_layerblend2_Dynamic_t             layerblend2_Dynamic_f;
    };

    /* 3.6.4.74 layerblend2_Status */
    union {
        __IO  uint32_t                              layerblend2_Status;
        PixEngCfg_layerblend2_Status_t              layerblend2_Status_f;
    };

    /* 3.6.4.75 extsrc8_LockUnlock */
    union {
        __IO  uint32_t                              extsrc8_LockUnlock;
        PixEngCfg_extsrc8_LockUnlock_t              extsrc8_LockUnlock_f;
    };

    /* 3.6.4.76 extsrc8_LockStatus */
    union {
        __IO  uint32_t                              extsrc8_LockStatus;
        PixEngCfg_extsrc8_LockStatus_t              extsrc8_LockStatus_f;
    };

    /* 3.6.4.77 extsrc8_Status */
    union {
        __IO  uint32_t                              extsrc8_Status;
        PixEngCfg_extsrc8_Status_t                  extsrc8_Status_f;
    };

} IRIS_MDL2_PixEngCfg_TypeDef;


/* ----[ FetchDecodeL9 ]----------------------------------------------------------------- */
typedef struct {
    /* 3.6.6.1 LockUnlock */
    union {
        __IO  uint32_t                              LockUnlock;
        Fetch_LockUnlock_t                          LockUnlock_f;
    };

    /* 3.6.6.2 LockStatus */
    union {
        __IO  uint32_t                              LockStatus;
        Fetch_LockStatus_t                          LockStatus_f;
    };

    /* 3.6.6.3 StaticControl */
    union {
        __IO  uint32_t                              StaticControl;
        Fetch_StaticControl_t                       StaticControl_f;
    };

    /* 3.6.6.4 BurstBufferManagement */
    union {
        __IO  uint32_t                              BurstBufferManagement;
        Fetch_BurstBufferManagement_t               BurstBufferManagement_f;
    };

    /* 3.6.6.5 RingBufStartAddr0 */
    union {
        __IO  uint32_t                              RingBufStartAddr0;
        Fetch_RingBufStartAddr0_t                   RingBufStartAddr0_f;
    };

    /* 3.6.6.6 RingBufWrapAddr0 */
    union {
        __IO  uint32_t                              RingBufWrapAddr0;
        Fetch_RingBufWrapAddr0_t                    RingBufWrapAddr0_f;
    };

    /* 3.6.6.7 FrameProperties0 */
    union {
        __IO  uint32_t                              FrameProperties0;
        Fetch_FrameProperties0_t                    FrameProperties0_f;
    };

    /* 3.6.6.8 BaseAddress0 */
    union {
        __IO  uint32_t                              BaseAddress0;
        Fetch_BaseAddress0_t                        BaseAddress0_f;
    };

    /* 3.6.6.9 SourceBufferAttributes0 */
    union {
        __IO  uint32_t                              SourceBufferAttributes0;
        Fetch_SourceBufferAttributes0_t             SourceBufferAttributes0_f;
    };

    /* 3.6.6.10 SourceBufferDimension0 */
    union {
        __IO  uint32_t                              SourceBufferDimension0;
        Fetch_SourceBufferDimension0_t              SourceBufferDimension0_f;
    };

    /* 3.6.6.11 ColorComponentBits0 */
    union {
        __IO  uint32_t                              ColorComponentBits0;
        Fetch_ColorComponentBits0_t                 ColorComponentBits0_f;
    };

    /* 3.6.6.12 ColorComponentShift0 */
    union {
        __IO  uint32_t                              ColorComponentShift0;
        Fetch_ColorComponentShift0_t                ColorComponentShift0_f;
    };

    /* 3.6.6.13 LayerOffset0 */
    union {
        __IO  uint32_t                              LayerOffset0;
        Fetch_LayerOffset0_t                        LayerOffset0_f;
    };

    /* 3.6.6.14 ClipWindowOffset0 */
    union {
        __IO  uint32_t                              ClipWindowOffset0;
        Fetch_ClipWindowOffset0_t                   ClipWindowOffset0_f;
    };

    /* 3.6.6.15 ClipWindowDimensions0 */
    union {
        __IO  uint32_t                              ClipWindowDimensions0;
        Fetch_ClipWindowDimensions0_t               ClipWindowDimensions0_f;
    };

    /* 3.6.6.16 ConstantColor0 */
    union {
        __IO  uint32_t                              ConstantColor0;
        Fetch_ConstantColor0_t                      ConstantColor0_f;
    };

    /* 3.6.6.17 LayerProperty0 */
    union {
        __IO  uint32_t                              LayerProperty0;
        Fetch_LayerProperty0_t                      LayerProperty0_f;
    };

    /* 3.6.6.88 FrameDimensions */
    union {
        __IO  uint32_t                              FrameDimensions;
        Fetch_FrameDimensions_t                     FrameDimensions_f;
    };

    /* 3.6.6.89 FrameResampling */
    union {
        __IO  uint32_t                              FrameResampling;
        Fetch_FrameResampling_t                     FrameResampling_f;
    };

    /* 3.6.6.111 DecodeControl */
    union {
        __IO  uint32_t                              DecodeControl;
        Fetch_DecodeControl_t                       DecodeControl_f;
    };

    /* 3.6.6.112 SourceBufferLength */
    union {
        __IO  uint32_t                              SourceBufferLength;
        Fetch_SourceBufferLength_t                  SourceBufferLength_f;
    };

    /* 3.6.6.113 Control */
    union {
        __IO  uint32_t                              Control;
        Fetch_Control_t                             Control_f;
    };

    /* 3.6.6.115 ControlTrigger */
    union {
        __IO  uint32_t                              ControlTrigger;
        Fetch_ControlTrigger_t                      ControlTrigger_f;
    };

    /* 3.6.6.116 Start */
    union {
        __IO  uint32_t                              Start;
        Fetch_Start_t                               Start_f;
    };

    /* 3.6.6.117 FetchType */
    union {
        __IO  uint32_t                              FetchType;
        Fetch_FetchType_t                           FetchType_f;
    };

    /* 3.6.6.118 DecoderStatus */
    union {
        __IO  uint32_t                              DecoderStatus;
        Fetch_DecoderStatus_t                       DecoderStatus_f;
    };

    /* 3.6.6.119 ReadAddress0 */
    union {
        __IO  uint32_t                              ReadAddress0;
        Fetch_ReadAddress0_t                        ReadAddress0_f;
    };

    /* 3.6.6.120 BurstBufferProperties */
    union {
        __IO  uint32_t                              BurstBufferProperties;
        Fetch_BurstBufferProperties_t               BurstBufferProperties_f;
    };

    /* 3.6.6.121 Status */
    union {
        __IO  uint32_t                              Status;
        Fetch_Status_t                              Status_f;
    };

    uint32_t                                        RESERVED_0074[227];

    /* 3.6.6.122 ColorPalette[0..255] */
    union {
        __IO  uint32_t                              ColorPalette[256];
        Fetch_ColorPalette_t                        ColorPalette_f[256];
    };

} IRIS_MDL2_FetchDecodeL9_TypeDef;


/* ----[ FetchRotL9 ]-------------------------------------------------------------------- */
typedef struct {
    /* 3.6.6.1 LockUnlock */
    union {
        __IO  uint32_t                              LockUnlock;
        Fetch_LockUnlock_t                          LockUnlock_f;
    };

    /* 3.6.6.2 LockStatus */
    union {
        __IO  uint32_t                              LockStatus;
        Fetch_LockStatus_t                          LockStatus_f;
    };

    /* 3.6.6.3 StaticControl */
    union {
        __IO  uint32_t                              StaticControl;
        Fetch_StaticControl_t                       StaticControl_f;
    };

    /* 3.6.6.4 BurstBufferManagement */
    union {
        __IO  uint32_t                              BurstBufferManagement;
        Fetch_BurstBufferManagement_t               BurstBufferManagement_f;
    };

    /* 3.6.6.8 BaseAddress0 */
    union {
        __IO  uint32_t                              BaseAddress0;
        Fetch_BaseAddress0_t                        BaseAddress0_f;
    };

    /* 3.6.6.9 SourceBufferAttributes0 */
    union {
        __IO  uint32_t                              SourceBufferAttributes0;
        Fetch_SourceBufferAttributes0_t             SourceBufferAttributes0_f;
    };

    /* 3.6.6.10 SourceBufferDimension0 */
    union {
        __IO  uint32_t                              SourceBufferDimension0;
        Fetch_SourceBufferDimension0_t              SourceBufferDimension0_f;
    };

    /* 3.6.6.11 ColorComponentBits0 */
    union {
        __IO  uint32_t                              ColorComponentBits0;
        Fetch_ColorComponentBits0_t                 ColorComponentBits0_f;
    };

    /* 3.6.6.12 ColorComponentShift0 */
    union {
        __IO  uint32_t                              ColorComponentShift0;
        Fetch_ColorComponentShift0_t                ColorComponentShift0_f;
    };

    /* 3.6.6.13 LayerOffset0 */
    union {
        __IO  uint32_t                              LayerOffset0;
        Fetch_LayerOffset0_t                        LayerOffset0_f;
    };

    /* 3.6.6.14 ClipWindowOffset0 */
    union {
        __IO  uint32_t                              ClipWindowOffset0;
        Fetch_ClipWindowOffset0_t                   ClipWindowOffset0_f;
    };

    /* 3.6.6.15 ClipWindowDimensions0 */
    union {
        __IO  uint32_t                              ClipWindowDimensions0;
        Fetch_ClipWindowDimensions0_t               ClipWindowDimensions0_f;
    };

    /* 3.6.6.16 ConstantColor0 */
    union {
        __IO  uint32_t                              ConstantColor0;
        Fetch_ConstantColor0_t                      ConstantColor0_f;
    };

    /* 3.6.6.17 LayerProperty0 */
    union {
        __IO  uint32_t                              LayerProperty0;
        Fetch_LayerProperty0_t                      LayerProperty0_f;
    };

    /* 3.6.6.88 FrameDimensions */
    union {
        __IO  uint32_t                              FrameDimensions;
        Fetch_FrameDimensions_t                     FrameDimensions_f;
    };

    /* 3.6.6.89 FrameResampling */
    union {
        __IO  uint32_t                              FrameResampling;
        Fetch_FrameResampling_t                     FrameResampling_f;
    };

    /* 3.6.6.100 AffineStartX */
    union {
        __IO  uint32_t                              AffineStartX;
        Fetch_AffineStartX_t                        AffineStartX_f;
    };

    /* 3.6.6.101 AffineStartY */
    union {
        __IO  uint32_t                              AffineStartY;
        Fetch_AffineStartY_t                        AffineStartY_f;
    };

    /* 3.6.6.102 AffineDeltaXX */
    union {
        __IO  uint32_t                              AffineDeltaXX;
        Fetch_AffineDeltaXX_t                       AffineDeltaXX_f;
    };

    /* 3.6.6.103 AffineDeltaXY */
    union {
        __IO  uint32_t                              AffineDeltaXY;
        Fetch_AffineDeltaXY_t                       AffineDeltaXY_f;
    };

    /* 3.6.6.104 AffineDeltaYX */
    union {
        __IO  uint32_t                              AffineDeltaYX;
        Fetch_AffineDeltaYX_t                       AffineDeltaYX_f;
    };

    /* 3.6.6.105 AffineDeltaYY */
    union {
        __IO  uint32_t                              AffineDeltaYY;
        Fetch_AffineDeltaYY_t                       AffineDeltaYY_f;
    };

    /* 3.6.6.109 FIRPositions */
    union {
        __IO  uint32_t                              FIRPositions;
        Fetch_FIRPositions_t                        FIRPositions_f;
    };

    /* 3.6.6.110 FIRCoefficients */
    union {
        __IO  uint32_t                              FIRCoefficients;
        Fetch_FIRCoefficients_t                     FIRCoefficients_f;
    };

    /* 3.6.6.113 Control */
    union {
        __IO  uint32_t                              Control;
        Fetch_Control_t                             Control_f;
    };

    /* 3.6.6.115 ControlTrigger */
    union {
        __IO  uint32_t                              ControlTrigger;
        Fetch_ControlTrigger_t                      ControlTrigger_f;
    };

    /* 3.6.6.116 Start */
    union {
        __IO  uint32_t                              Start;
        Fetch_Start_t                               Start_f;
    };

    /* 3.6.6.117 FetchType */
    union {
        __IO  uint32_t                              FetchType;
        Fetch_FetchType_t                           FetchType_f;
    };

    /* 3.6.6.120 BurstBufferProperties */
    union {
        __IO  uint32_t                              BurstBufferProperties;
        Fetch_BurstBufferProperties_t               BurstBufferProperties_f;
    };

} IRIS_MDL2_FetchRotL9_TypeDef;


/* ----[ FetchEcoL9 ]-------------------------------------------------------------------- */
typedef struct {
    /* 3.6.6.1 LockUnlock */
    union {
        __IO  uint32_t                              LockUnlock;
        Fetch_LockUnlock_t                          LockUnlock_f;
    };

    /* 3.6.6.2 LockStatus */
    union {
        __IO  uint32_t                              LockStatus;
        Fetch_LockStatus_t                          LockStatus_f;
    };

    /* 3.6.6.3 StaticControl */
    union {
        __IO  uint32_t                              StaticControl;
        Fetch_StaticControl_t                       StaticControl_f;
    };

    /* 3.6.6.4 BurstBufferManagement */
    union {
        __IO  uint32_t                              BurstBufferManagement;
        Fetch_BurstBufferManagement_t               BurstBufferManagement_f;
    };

    /* 3.6.6.8 BaseAddress0 */
    union {
        __IO  uint32_t                              BaseAddress0;
        Fetch_BaseAddress0_t                        BaseAddress0_f;
    };

    /* 3.6.6.9 SourceBufferAttributes0 */
    union {
        __IO  uint32_t                              SourceBufferAttributes0;
        Fetch_SourceBufferAttributes0_t             SourceBufferAttributes0_f;
    };

    /* 3.6.6.10 SourceBufferDimension0 */
    union {
        __IO  uint32_t                              SourceBufferDimension0;
        Fetch_SourceBufferDimension0_t              SourceBufferDimension0_f;
    };

    /* 3.6.6.11 ColorComponentBits0 */
    union {
        __IO  uint32_t                              ColorComponentBits0;
        Fetch_ColorComponentBits0_t                 ColorComponentBits0_f;
    };

    /* 3.6.6.12 ColorComponentShift0 */
    union {
        __IO  uint32_t                              ColorComponentShift0;
        Fetch_ColorComponentShift0_t                ColorComponentShift0_f;
    };

    /* 3.6.6.13 LayerOffset0 */
    union {
        __IO  uint32_t                              LayerOffset0;
        Fetch_LayerOffset0_t                        LayerOffset0_f;
    };

    /* 3.6.6.14 ClipWindowOffset0 */
    union {
        __IO  uint32_t                              ClipWindowOffset0;
        Fetch_ClipWindowOffset0_t                   ClipWindowOffset0_f;
    };

    /* 3.6.6.15 ClipWindowDimensions0 */
    union {
        __IO  uint32_t                              ClipWindowDimensions0;
        Fetch_ClipWindowDimensions0_t               ClipWindowDimensions0_f;
    };

    /* 3.6.6.16 ConstantColor0 */
    union {
        __IO  uint32_t                              ConstantColor0;
        Fetch_ConstantColor0_t                      ConstantColor0_f;
    };

    /* 3.6.6.17 LayerProperty0 */
    union {
        __IO  uint32_t                              LayerProperty0;
        Fetch_LayerProperty0_t                      LayerProperty0_f;
    };

    /* 3.6.6.88 FrameDimensions */
    union {
        __IO  uint32_t                              FrameDimensions;
        Fetch_FrameDimensions_t                     FrameDimensions_f;
    };

    /* 3.6.6.89 FrameResampling */
    union {
        __IO  uint32_t                              FrameResampling;
        Fetch_FrameResampling_t                     FrameResampling_f;
    };

    /* 3.6.6.113 Control */
    union {
        __IO  uint32_t                              Control;
        Fetch_Control_t                             Control_f;
    };

    /* 3.6.6.115 ControlTrigger */
    union {
        __IO  uint32_t                              ControlTrigger;
        Fetch_ControlTrigger_t                      ControlTrigger_f;
    };

    /* 3.6.6.116 Start */
    union {
        __IO  uint32_t                              Start;
        Fetch_Start_t                               Start_f;
    };

    /* 3.6.6.117 FetchType */
    union {
        __IO  uint32_t                              FetchType;
        Fetch_FetchType_t                           FetchType_f;
    };

    /* 3.6.6.120 BurstBufferProperties */
    union {
        __IO  uint32_t                              BurstBufferProperties;
        Fetch_BurstBufferProperties_t               BurstBufferProperties_f;
    };

} IRIS_MDL2_FetchEcoL9_TypeDef;


/* ----[ ROp9 ]-------------------------------------------------------------------------- */
typedef struct {
    /* 3.6.17.1 LockUnlock */
    union {
        __IO  uint32_t                              LockUnlock;
        ROp_LockUnlock_t                            LockUnlock_f;
    };

    /* 3.6.17.2 LockStatus */
    union {
        __IO  uint32_t                              LockStatus;
        ROp_LockStatus_t                            LockStatus_f;
    };

    /* 3.6.17.3 StaticControl */
    union {
        __IO  uint32_t                              StaticControl;
        ROp_StaticControl_t                         StaticControl_f;
    };

    /* 3.6.17.4 Control */
    union {
        __IO  uint32_t                              Control;
        ROp_Control_t                               Control_f;
    };

    /* 3.6.17.5 RasterOperationIndices */
    union {
        __IO  uint32_t                              RasterOperationIndices;
        ROp_RasterOperationIndices_t                RasterOperationIndices_f;
    };

    /* 3.6.17.6 PrimControlWord */
    union {
        __IO  uint32_t                              PrimControlWord;
        ROp_PrimControlWord_t                       PrimControlWord_f;
    };

    /* 3.6.17.7 SecControlWord */
    union {
        __IO  uint32_t                              SecControlWord;
        ROp_SecControlWord_t                        SecControlWord_f;
    };

    /* 3.6.17.8 TertControlWord */
    union {
        __IO  uint32_t                              TertControlWord;
        ROp_TertControlWord_t                       TertControlWord_f;
    };

} IRIS_MDL2_ROp9_TypeDef;


/* ----[ CLuTL9 ]------------------------------------------------------------------------ */
typedef struct {
    /* 3.6.12.1 LockUnlock */
    union {
        __IO  uint32_t                              LockUnlock;
        CLuT_LockUnlock_t                           LockUnlock_f;
    };

    /* 3.6.12.2 LockStatus */
    union {
        __IO  uint32_t                              LockStatus;
        CLuT_LockStatus_t                           LockStatus_f;
    };

    /* 3.6.12.3 StaticControl */
    union {
        __IO  uint32_t                              StaticControl;
        CLuT_StaticControl_t                        StaticControl_f;
    };

    /* 3.6.12.4 UnshadowedControl */
    union {
        __IO  uint32_t                              UnshadowedControl;
        CLuT_UnshadowedControl_t                    UnshadowedControl_f;
    };

    /* 3.6.12.5 Control */
    union {
        __IO  uint32_t                              Control;
        CLuT_Control_t                              Control_f;
    };

    /* 3.6.12.6 Status */
    union {
        __IO  uint32_t                              Status;
        CLuT_Status_t                               Status_f;
    };

    /* 3.6.12.7 LastControlWord */
    union {
        __IO  uint32_t                              LastControlWord;
        CLuT_LastControlWord_t                      LastControlWord_f;
    };

    uint32_t                                        RESERVED_001C[249];

    /* 3.6.12.8 LUT[0..255] */
    union {
        __IO  uint32_t                              LUT[256];
        CLuT_LUT_t                                  LUT_f[256];
    };

} IRIS_MDL2_CLuTL9_TypeDef;


/* ----[ MatrixL9 ]---------------------------------------------------------------------- */
typedef struct {
    /* 3.6.11.1 LockUnlock */
    union {
        __IO  uint32_t                              LockUnlock;
        Matrix_LockUnlock_t                         LockUnlock_f;
    };

    /* 3.6.11.2 LockStatus */
    union {
        __IO  uint32_t                              LockStatus;
        Matrix_LockStatus_t                         LockStatus_f;
    };

    /* 3.6.11.3 StaticControl */
    union {
        __IO  uint32_t                              StaticControl;
        Matrix_StaticControl_t                      StaticControl_f;
    };

    /* 3.6.11.4 Control */
    union {
        __IO  uint32_t                              Control;
        Matrix_Control_t                            Control_f;
    };

    /* 3.6.11.5 Red0 */
    union {
        __IO  uint32_t                              Red0;
        Matrix_Red0_t                               Red0_f;
    };

    /* 3.6.11.6 Red1 */
    union {
        __IO  uint32_t                              Red1;
        Matrix_Red1_t                               Red1_f;
    };

    /* 3.6.11.7 Green0 */
    union {
        __IO  uint32_t                              Green0;
        Matrix_Green0_t                             Green0_f;
    };

    /* 3.6.11.8 Green1 */
    union {
        __IO  uint32_t                              Green1;
        Matrix_Green1_t                             Green1_f;
    };

    /* 3.6.11.9 Blue0 */
    union {
        __IO  uint32_t                              Blue0;
        Matrix_Blue0_t                              Blue0_f;
    };

    /* 3.6.11.10 Blue1 */
    union {
        __IO  uint32_t                              Blue1;
        Matrix_Blue1_t                              Blue1_f;
    };

    /* 3.6.11.13 OffsetVector0 */
    union {
        __IO  uint32_t                              OffsetVector0;
        Matrix_OffsetVector0_t                      OffsetVector0_f;
    };

    /* 3.6.11.14 OffsetVector1 */
    union {
        __IO  uint32_t                              OffsetVector1;
        Matrix_OffsetVector1_t                      OffsetVector1_f;
    };

} IRIS_MDL2_MatrixL9_TypeDef;


/* ----[ BlitBlendL9 ]------------------------------------------------------------------- */
typedef struct {
    /* 3.6.16.1 LockUnlock */
    union {
        __IO  uint32_t                              LockUnlock;
        BlitBlend_LockUnlock_t                      LockUnlock_f;
    };

    /* 3.6.16.2 LockStatus */
    union {
        __IO  uint32_t                              LockStatus;
        BlitBlend_LockStatus_t                      LockStatus_f;
    };

    /* 3.6.16.3 StaticControl */
    union {
        __IO  uint32_t                              StaticControl;
        BlitBlend_StaticControl_t                   StaticControl_f;
    };

    /* 3.6.16.4 Control */
    union {
        __IO  uint32_t                              Control;
        BlitBlend_Control_t                         Control_f;
    };

    /* 3.6.16.5 NeutralBorder */
    union {
        __IO  uint32_t                              NeutralBorder;
        BlitBlend_NeutralBorder_t                   NeutralBorder_f;
    };

    /* 3.6.16.6 ConstantColor */
    union {
        __IO  uint32_t                              ConstantColor;
        BlitBlend_ConstantColor_t                   ConstantColor_f;
    };

    /* 3.6.16.7 ColorRedBlendFunction */
    union {
        __IO  uint32_t                              ColorRedBlendFunction;
        BlitBlend_ColorRedBlendFunction_t           ColorRedBlendFunction_f;
    };

    /* 3.6.16.8 ColorGreenBlendFunction */
    union {
        __IO  uint32_t                              ColorGreenBlendFunction;
        BlitBlend_ColorGreenBlendFunction_t         ColorGreenBlendFunction_f;
    };

    /* 3.6.16.9 ColorBlueBlendFunction */
    union {
        __IO  uint32_t                              ColorBlueBlendFunction;
        BlitBlend_ColorBlueBlendFunction_t          ColorBlueBlendFunction_f;
    };

    /* 3.6.16.10 AlphaBlendFunction */
    union {
        __IO  uint32_t                              AlphaBlendFunction;
        BlitBlend_AlphaBlendFunction_t              AlphaBlendFunction_f;
    };

    /* 3.6.16.11 BlendMode1 */
    union {
        __IO  uint32_t                              BlendMode1;
        BlitBlend_BlendMode1_t                      BlendMode1_f;
    };

    /* 3.6.16.12 BlendMode2 */
    union {
        __IO  uint32_t                              BlendMode2;
        BlitBlend_BlendMode2_t                      BlendMode2_f;
    };

    /* 3.6.16.13 DirectSetup */
    union {
        __IO  uint32_t                              DirectSetup;
        BlitBlend_DirectSetup_t                     DirectSetup_f;
    };

    /* 3.6.16.14 PrimControlWord */
    union {
        __IO  uint32_t                              PrimControlWord;
        BlitBlend_PrimControlWord_t                 PrimControlWord_f;
    };

    /* 3.6.16.15 SecControlWord */
    union {
        __IO  uint32_t                              SecControlWord;
        BlitBlend_SecControlWord_t                  SecControlWord_f;
    };

} IRIS_MDL2_BlitBlendL9_TypeDef;


/* ----[ StoreL9 ]----------------------------------------------------------------------- */
typedef struct {
    /* 3.6.8.1 LockUnlock */
    union {
        __IO  uint32_t                              LockUnlock;
        Store_LockUnlock_t                          LockUnlock_f;
    };

    /* 3.6.8.2 LockStatus */
    union {
        __IO  uint32_t                              LockStatus;
        Store_LockStatus_t                          LockStatus_f;
    };

    /* 3.6.8.3 StaticControl */
    union {
        __IO  uint32_t                              StaticControl;
        Store_StaticControl_t                       StaticControl_f;
    };

    /* 3.6.8.4 BurstBufferManagement */
    union {
        __IO  uint32_t                              BurstBufferManagement;
        Store_BurstBufferManagement_t               BurstBufferManagement_f;
    };

    /* 3.6.8.5 RingBufStartAddr */
    union {
        __IO  uint32_t                              RingBufStartAddr;
        Store_RingBufStartAddr_t                    RingBufStartAddr_f;
    };

    /* 3.6.8.6 RingBufWrapAddr */
    union {
        __IO  uint32_t                              RingBufWrapAddr;
        Store_RingBufWrapAddr_t                     RingBufWrapAddr_f;
    };

    /* 3.6.8.7 BaseAddress */
    union {
        __IO  uint32_t                              BaseAddress;
        Store_BaseAddress_t                         BaseAddress_f;
    };

    /* 3.6.8.8 DestinationBufferAttributes */
    union {
        __IO  uint32_t                              DestinationBufferAttributes;
        Store_DestinationBufferAttributes_t         DestinationBufferAttributes_f;
    };

    /* 3.6.8.9 DestinationBufferDimension */
    union {
        __IO  uint32_t                              DestinationBufferDimension;
        Store_DestinationBufferDimension_t          DestinationBufferDimension_f;
    };

    /* 3.6.8.10 FrameOffset */
    union {
        __IO  uint32_t                              FrameOffset;
        Store_FrameOffset_t                         FrameOffset_f;
    };

    /* 3.6.8.11 ColorComponentBits */
    union {
        __IO  uint32_t                              ColorComponentBits;
        Store_ColorComponentBits_t                  ColorComponentBits_f;
    };

    /* 3.6.8.12 ColorComponentShift */
    union {
        __IO  uint32_t                              ColorComponentShift;
        Store_ColorComponentShift_t                 ColorComponentShift_f;
    };

    /* 3.6.8.13 Control */
    union {
        __IO  uint32_t                              Control;
        Store_Control_t                             Control_f;
    };

    /* 3.6.8.16 Start */
    union {
        __IO  uint32_t                              Start;
        Store_Start_t                               Start_f;
    };

    /* 3.6.8.18 WriteAddress */
    union {
        __IO  uint32_t                              WriteAddress;
        Store_WriteAddress_t                        WriteAddress_f;
    };

    /* 3.6.8.19 FrameProperties */
    union {
        __IO  uint32_t                              FrameProperties;
        Store_FrameProperties_t                     FrameProperties_f;
    };

    /* 3.6.8.20 BurstBufferProperties */
    union {
        __IO  uint32_t                              BurstBufferProperties;
        Store_BurstBufferProperties_t               BurstBufferProperties_f;
    };

    /* 3.6.8.21 LastControlWord */
    union {
        __IO  uint32_t                              LastControlWord;
        Store_LastControlWord_t                     LastControlWord_f;
    };

    /* 3.6.8.22 PerfCounter */
    union {
        __IO  uint32_t                              PerfCounter;
        Store_PerfCounter_t                         PerfCounter_f;
    };

} IRIS_MDL2_StoreL9_TypeDef;


/* ----[ ConstFrameL0 ]------------------------------------------------------------------ */
typedef struct {
    /* 3.6.7.1 LockUnlock */
    union {
        __IO  uint32_t                              LockUnlock;
        ConstFrame_LockUnlock_t                     LockUnlock_f;
    };

    /* 3.6.7.2 LockStatus */
    union {
        __IO  uint32_t                              LockStatus;
        ConstFrame_LockStatus_t                     LockStatus_f;
    };

    /* 3.6.7.3 StaticControl */
    union {
        __IO  uint32_t                              StaticControl;
        ConstFrame_StaticControl_t                  StaticControl_f;
    };

    /* 3.6.7.4 FrameDimensions */
    union {
        __IO  uint32_t                              FrameDimensions;
        ConstFrame_FrameDimensions_t                FrameDimensions_f;
    };

    /* 3.6.7.5 ConstantColor */
    union {
        __IO  uint32_t                              ConstantColor;
        ConstFrame_ConstantColor_t                  ConstantColor_f;
    };

    /* 3.6.7.6 ControlTrigger */
    union {
        __IO  uint32_t                              ControlTrigger;
        ConstFrame_ControlTrigger_t                 ControlTrigger_f;
    };

    /* 3.6.7.7 Start */
    union {
        __IO  uint32_t                              Start;
        ConstFrame_Start_t                          Start_f;
    };

} IRIS_MDL2_ConstFrameL0_TypeDef;


/* ----[ ExtDstL0 ]---------------------------------------------------------------------- */
typedef struct {
    /* 3.6.10.1 LockUnlock */
    union {
        __IO  uint32_t                              LockUnlock;
        ExtDst_LockUnlock_t                         LockUnlock_f;
    };

    /* 3.6.10.2 LockStatus */
    union {
        __IO  uint32_t                              LockStatus;
        ExtDst_LockStatus_t                         LockStatus_f;
    };

    /* 3.6.10.3 StaticControl */
    union {
        __IO  uint32_t                              StaticControl;
        ExtDst_StaticControl_t                      StaticControl_f;
    };

    /* 3.6.10.5 SoftwareKick */
    union {
        __IO  uint32_t                              SoftwareKick;
        ExtDst_SoftwareKick_t                       SoftwareKick_f;
    };

    /* 3.6.10.6 Status */
    union {
        __IO  uint32_t                              Status;
        ExtDst_Status_t                             Status_f;
    };

    /* 3.6.10.7 ControlWord */
    union {
        __IO  uint32_t                              ControlWord;
        ExtDst_ControlWord_t                        ControlWord_f;
    };

    /* 3.6.10.8 CurPixelCnt */
    union {
        __IO  uint32_t                              CurPixelCnt;
        ExtDst_CurPixelCnt_t                        CurPixelCnt_f;
    };

    /* 3.6.10.9 LastPixelCnt */
    union {
        __IO  uint32_t                              LastPixelCnt;
        ExtDst_LastPixelCnt_t                       LastPixelCnt_f;
    };

    /* 3.6.10.10 PerfCounter */
    union {
        __IO  uint32_t                              PerfCounter;
        ExtDst_PerfCounter_t                        PerfCounter_f;
    };

} IRIS_MDL2_ExtDstL0_TypeDef;


/* ----[ ConstFrameL4 ]------------------------------------------------------------------ */
typedef struct {
    /* 3.6.7.1 LockUnlock */
    union {
        __IO  uint32_t                              LockUnlock;
        ConstFrame_LockUnlock_t                     LockUnlock_f;
    };

    /* 3.6.7.2 LockStatus */
    union {
        __IO  uint32_t                              LockStatus;
        ConstFrame_LockStatus_t                     LockStatus_f;
    };

    /* 3.6.7.3 StaticControl */
    union {
        __IO  uint32_t                              StaticControl;
        ConstFrame_StaticControl_t                  StaticControl_f;
    };

    /* 3.6.7.4 FrameDimensions */
    union {
        __IO  uint32_t                              FrameDimensions;
        ConstFrame_FrameDimensions_t                FrameDimensions_f;
    };

    /* 3.6.7.5 ConstantColor */
    union {
        __IO  uint32_t                              ConstantColor;
        ConstFrame_ConstantColor_t                  ConstantColor_f;
    };

    /* 3.6.7.6 ControlTrigger */
    union {
        __IO  uint32_t                              ControlTrigger;
        ConstFrame_ControlTrigger_t                 ControlTrigger_f;
    };

    /* 3.6.7.7 Start */
    union {
        __IO  uint32_t                              Start;
        ConstFrame_Start_t                          Start_f;
    };

} IRIS_MDL2_ConstFrameL4_TypeDef;


/* ----[ ExtDstL4 ]---------------------------------------------------------------------- */
typedef struct {
    /* 3.6.10.1 LockUnlock */
    union {
        __IO  uint32_t                              LockUnlock;
        ExtDst_LockUnlock_t                         LockUnlock_f;
    };

    /* 3.6.10.2 LockStatus */
    union {
        __IO  uint32_t                              LockStatus;
        ExtDst_LockStatus_t                         LockStatus_f;
    };

    /* 3.6.10.3 StaticControl */
    union {
        __IO  uint32_t                              StaticControl;
        ExtDst_StaticControl_t                      StaticControl_f;
    };

    /* 3.6.10.5 SoftwareKick */
    union {
        __IO  uint32_t                              SoftwareKick;
        ExtDst_SoftwareKick_t                       SoftwareKick_f;
    };

    /* 3.6.10.6 Status */
    union {
        __IO  uint32_t                              Status;
        ExtDst_Status_t                             Status_f;
    };

    /* 3.6.10.7 ControlWord */
    union {
        __IO  uint32_t                              ControlWord;
        ExtDst_ControlWord_t                        ControlWord_f;
    };

    /* 3.6.10.8 CurPixelCnt */
    union {
        __IO  uint32_t                              CurPixelCnt;
        ExtDst_CurPixelCnt_t                        CurPixelCnt_f;
    };

    /* 3.6.10.9 LastPixelCnt */
    union {
        __IO  uint32_t                              LastPixelCnt;
        ExtDst_LastPixelCnt_t                       LastPixelCnt_f;
    };

    /* 3.6.10.10 PerfCounter */
    union {
        __IO  uint32_t                              PerfCounter;
        ExtDst_PerfCounter_t                        PerfCounter_f;
    };

} IRIS_MDL2_ExtDstL4_TypeDef;


/* ----[ FetchDecodeL0 ]----------------------------------------------------------------- */
typedef struct {
    /* 3.6.6.1 LockUnlock */
    union {
        __IO  uint32_t                              LockUnlock;
        Fetch_LockUnlock_t                          LockUnlock_f;
    };

    /* 3.6.6.2 LockStatus */
    union {
        __IO  uint32_t                              LockStatus;
        Fetch_LockStatus_t                          LockStatus_f;
    };

    /* 3.6.6.3 StaticControl */
    union {
        __IO  uint32_t                              StaticControl;
        Fetch_StaticControl_t                       StaticControl_f;
    };

    /* 3.6.6.4 BurstBufferManagement */
    union {
        __IO  uint32_t                              BurstBufferManagement;
        Fetch_BurstBufferManagement_t               BurstBufferManagement_f;
    };

    /* 3.6.6.5 RingBufStartAddr0 */
    union {
        __IO  uint32_t                              RingBufStartAddr0;
        Fetch_RingBufStartAddr0_t                   RingBufStartAddr0_f;
    };

    /* 3.6.6.6 RingBufWrapAddr0 */
    union {
        __IO  uint32_t                              RingBufWrapAddr0;
        Fetch_RingBufWrapAddr0_t                    RingBufWrapAddr0_f;
    };

    /* 3.6.6.7 FrameProperties0 */
    union {
        __IO  uint32_t                              FrameProperties0;
        Fetch_FrameProperties0_t                    FrameProperties0_f;
    };

    /* 3.6.6.8 BaseAddress0 */
    union {
        __IO  uint32_t                              BaseAddress0;
        Fetch_BaseAddress0_t                        BaseAddress0_f;
    };

    /* 3.6.6.9 SourceBufferAttributes0 */
    union {
        __IO  uint32_t                              SourceBufferAttributes0;
        Fetch_SourceBufferAttributes0_t             SourceBufferAttributes0_f;
    };

    /* 3.6.6.10 SourceBufferDimension0 */
    union {
        __IO  uint32_t                              SourceBufferDimension0;
        Fetch_SourceBufferDimension0_t              SourceBufferDimension0_f;
    };

    /* 3.6.6.11 ColorComponentBits0 */
    union {
        __IO  uint32_t                              ColorComponentBits0;
        Fetch_ColorComponentBits0_t                 ColorComponentBits0_f;
    };

    /* 3.6.6.12 ColorComponentShift0 */
    union {
        __IO  uint32_t                              ColorComponentShift0;
        Fetch_ColorComponentShift0_t                ColorComponentShift0_f;
    };

    /* 3.6.6.13 LayerOffset0 */
    union {
        __IO  uint32_t                              LayerOffset0;
        Fetch_LayerOffset0_t                        LayerOffset0_f;
    };

    /* 3.6.6.14 ClipWindowOffset0 */
    union {
        __IO  uint32_t                              ClipWindowOffset0;
        Fetch_ClipWindowOffset0_t                   ClipWindowOffset0_f;
    };

    /* 3.6.6.15 ClipWindowDimensions0 */
    union {
        __IO  uint32_t                              ClipWindowDimensions0;
        Fetch_ClipWindowDimensions0_t               ClipWindowDimensions0_f;
    };

    /* 3.6.6.16 ConstantColor0 */
    union {
        __IO  uint32_t                              ConstantColor0;
        Fetch_ConstantColor0_t                      ConstantColor0_f;
    };

    /* 3.6.6.17 LayerProperty0 */
    union {
        __IO  uint32_t                              LayerProperty0;
        Fetch_LayerProperty0_t                      LayerProperty0_f;
    };

    /* 3.6.6.88 FrameDimensions */
    union {
        __IO  uint32_t                              FrameDimensions;
        Fetch_FrameDimensions_t                     FrameDimensions_f;
    };

    /* 3.6.6.89 FrameResampling */
    union {
        __IO  uint32_t                              FrameResampling;
        Fetch_FrameResampling_t                     FrameResampling_f;
    };

    /* 3.6.6.111 DecodeControl */
    union {
        __IO  uint32_t                              DecodeControl;
        Fetch_DecodeControl_t                       DecodeControl_f;
    };

    /* 3.6.6.112 SourceBufferLength */
    union {
        __IO  uint32_t                              SourceBufferLength;
        Fetch_SourceBufferLength_t                  SourceBufferLength_f;
    };

    /* 3.6.6.113 Control */
    union {
        __IO  uint32_t                              Control;
        Fetch_Control_t                             Control_f;
    };

    /* 3.6.6.115 ControlTrigger */
    union {
        __IO  uint32_t                              ControlTrigger;
        Fetch_ControlTrigger_t                      ControlTrigger_f;
    };

    /* 3.6.6.116 Start */
    union {
        __IO  uint32_t                              Start;
        Fetch_Start_t                               Start_f;
    };

    /* 3.6.6.117 FetchType */
    union {
        __IO  uint32_t                              FetchType;
        Fetch_FetchType_t                           FetchType_f;
    };

    /* 3.6.6.118 DecoderStatus */
    union {
        __IO  uint32_t                              DecoderStatus;
        Fetch_DecoderStatus_t                       DecoderStatus_f;
    };

    /* 3.6.6.119 ReadAddress0 */
    union {
        __IO  uint32_t                              ReadAddress0;
        Fetch_ReadAddress0_t                        ReadAddress0_f;
    };

    /* 3.6.6.120 BurstBufferProperties */
    union {
        __IO  uint32_t                              BurstBufferProperties;
        Fetch_BurstBufferProperties_t               BurstBufferProperties_f;
    };

    /* 3.6.6.121 Status */
    union {
        __IO  uint32_t                              Status;
        Fetch_Status_t                              Status_f;
    };

    uint32_t                                        RESERVED_0074[227];

    /* 3.6.6.122 ColorPalette[0..255] */
    union {
        __IO  uint32_t                              ColorPalette[256];
        Fetch_ColorPalette_t                        ColorPalette_f[256];
    };

} IRIS_MDL2_FetchDecodeL0_TypeDef;


/* ----[ FetchLayerL0 ]------------------------------------------------------------------ */
typedef struct {
    /* 3.6.6.1 LockUnlock */
    union {
        __IO  uint32_t                              LockUnlock;
        Fetch_LockUnlock_t                          LockUnlock_f;
    };

    /* 3.6.6.2 LockStatus */
    union {
        __IO  uint32_t                              LockStatus;
        Fetch_LockStatus_t                          LockStatus_f;
    };

    /* 3.6.6.3 StaticControl */
    union {
        __IO  uint32_t                              StaticControl;
        Fetch_StaticControl_t                       StaticControl_f;
    };

    /* 3.6.6.4 BurstBufferManagement */
    union {
        __IO  uint32_t                              BurstBufferManagement;
        Fetch_BurstBufferManagement_t               BurstBufferManagement_f;
    };

    /* 3.6.6.8 BaseAddress0 */
    union {
        __IO  uint32_t                              BaseAddress0;
        Fetch_BaseAddress0_t                        BaseAddress0_f;
    };

    /* 3.6.6.9 SourceBufferAttributes0 */
    union {
        __IO  uint32_t                              SourceBufferAttributes0;
        Fetch_SourceBufferAttributes0_t             SourceBufferAttributes0_f;
    };

    /* 3.6.6.10 SourceBufferDimension0 */
    union {
        __IO  uint32_t                              SourceBufferDimension0;
        Fetch_SourceBufferDimension0_t              SourceBufferDimension0_f;
    };

    /* 3.6.6.11 ColorComponentBits0 */
    union {
        __IO  uint32_t                              ColorComponentBits0;
        Fetch_ColorComponentBits0_t                 ColorComponentBits0_f;
    };

    /* 3.6.6.12 ColorComponentShift0 */
    union {
        __IO  uint32_t                              ColorComponentShift0;
        Fetch_ColorComponentShift0_t                ColorComponentShift0_f;
    };

    /* 3.6.6.13 LayerOffset0 */
    union {
        __IO  uint32_t                              LayerOffset0;
        Fetch_LayerOffset0_t                        LayerOffset0_f;
    };

    /* 3.6.6.14 ClipWindowOffset0 */
    union {
        __IO  uint32_t                              ClipWindowOffset0;
        Fetch_ClipWindowOffset0_t                   ClipWindowOffset0_f;
    };

    /* 3.6.6.15 ClipWindowDimensions0 */
    union {
        __IO  uint32_t                              ClipWindowDimensions0;
        Fetch_ClipWindowDimensions0_t               ClipWindowDimensions0_f;
    };

    /* 3.6.6.16 ConstantColor0 */
    union {
        __IO  uint32_t                              ConstantColor0;
        Fetch_ConstantColor0_t                      ConstantColor0_f;
    };

    /* 3.6.6.17 LayerProperty0 */
    union {
        __IO  uint32_t                              LayerProperty0;
        Fetch_LayerProperty0_t                      LayerProperty0_f;
    };

    /* 3.6.6.18 BaseAddress1 */
    union {
        __IO  uint32_t                              BaseAddress1;
        Fetch_BaseAddress1_t                        BaseAddress1_f;
    };

    /* 3.6.6.19 SourceBufferAttributes1 */
    union {
        __IO  uint32_t                              SourceBufferAttributes1;
        Fetch_SourceBufferAttributes1_t             SourceBufferAttributes1_f;
    };

    /* 3.6.6.20 SourceBufferDimension1 */
    union {
        __IO  uint32_t                              SourceBufferDimension1;
        Fetch_SourceBufferDimension1_t              SourceBufferDimension1_f;
    };

    /* 3.6.6.21 ColorComponentBits1 */
    union {
        __IO  uint32_t                              ColorComponentBits1;
        Fetch_ColorComponentBits1_t                 ColorComponentBits1_f;
    };

    /* 3.6.6.22 ColorComponentShift1 */
    union {
        __IO  uint32_t                              ColorComponentShift1;
        Fetch_ColorComponentShift1_t                ColorComponentShift1_f;
    };

    /* 3.6.6.23 LayerOffset1 */
    union {
        __IO  uint32_t                              LayerOffset1;
        Fetch_LayerOffset1_t                        LayerOffset1_f;
    };

    /* 3.6.6.24 ClipWindowOffset1 */
    union {
        __IO  uint32_t                              ClipWindowOffset1;
        Fetch_ClipWindowOffset1_t                   ClipWindowOffset1_f;
    };

    /* 3.6.6.25 ClipWindowDimensions1 */
    union {
        __IO  uint32_t                              ClipWindowDimensions1;
        Fetch_ClipWindowDimensions1_t               ClipWindowDimensions1_f;
    };

    /* 3.6.6.26 ConstantColor1 */
    union {
        __IO  uint32_t                              ConstantColor1;
        Fetch_ConstantColor1_t                      ConstantColor1_f;
    };

    /* 3.6.6.27 LayerProperty1 */
    union {
        __IO  uint32_t                              LayerProperty1;
        Fetch_LayerProperty1_t                      LayerProperty1_f;
    };

    /* 3.6.6.28 BaseAddress2 */
    union {
        __IO  uint32_t                              BaseAddress2;
        Fetch_BaseAddress2_t                        BaseAddress2_f;
    };

    /* 3.6.6.29 SourceBufferAttributes2 */
    union {
        __IO  uint32_t                              SourceBufferAttributes2;
        Fetch_SourceBufferAttributes2_t             SourceBufferAttributes2_f;
    };

    /* 3.6.6.30 SourceBufferDimension2 */
    union {
        __IO  uint32_t                              SourceBufferDimension2;
        Fetch_SourceBufferDimension2_t              SourceBufferDimension2_f;
    };

    /* 3.6.6.31 ColorComponentBits2 */
    union {
        __IO  uint32_t                              ColorComponentBits2;
        Fetch_ColorComponentBits2_t                 ColorComponentBits2_f;
    };

    /* 3.6.6.32 ColorComponentShift2 */
    union {
        __IO  uint32_t                              ColorComponentShift2;
        Fetch_ColorComponentShift2_t                ColorComponentShift2_f;
    };

    /* 3.6.6.33 LayerOffset2 */
    union {
        __IO  uint32_t                              LayerOffset2;
        Fetch_LayerOffset2_t                        LayerOffset2_f;
    };

    /* 3.6.6.34 ClipWindowOffset2 */
    union {
        __IO  uint32_t                              ClipWindowOffset2;
        Fetch_ClipWindowOffset2_t                   ClipWindowOffset2_f;
    };

    /* 3.6.6.35 ClipWindowDimensions2 */
    union {
        __IO  uint32_t                              ClipWindowDimensions2;
        Fetch_ClipWindowDimensions2_t               ClipWindowDimensions2_f;
    };

    /* 3.6.6.36 ConstantColor2 */
    union {
        __IO  uint32_t                              ConstantColor2;
        Fetch_ConstantColor2_t                      ConstantColor2_f;
    };

    /* 3.6.6.37 LayerProperty2 */
    union {
        __IO  uint32_t                              LayerProperty2;
        Fetch_LayerProperty2_t                      LayerProperty2_f;
    };

    /* 3.6.6.38 BaseAddress3 */
    union {
        __IO  uint32_t                              BaseAddress3;
        Fetch_BaseAddress3_t                        BaseAddress3_f;
    };

    /* 3.6.6.39 SourceBufferAttributes3 */
    union {
        __IO  uint32_t                              SourceBufferAttributes3;
        Fetch_SourceBufferAttributes3_t             SourceBufferAttributes3_f;
    };

    /* 3.6.6.40 SourceBufferDimension3 */
    union {
        __IO  uint32_t                              SourceBufferDimension3;
        Fetch_SourceBufferDimension3_t              SourceBufferDimension3_f;
    };

    /* 3.6.6.41 ColorComponentBits3 */
    union {
        __IO  uint32_t                              ColorComponentBits3;
        Fetch_ColorComponentBits3_t                 ColorComponentBits3_f;
    };

    /* 3.6.6.42 ColorComponentShift3 */
    union {
        __IO  uint32_t                              ColorComponentShift3;
        Fetch_ColorComponentShift3_t                ColorComponentShift3_f;
    };

    /* 3.6.6.43 LayerOffset3 */
    union {
        __IO  uint32_t                              LayerOffset3;
        Fetch_LayerOffset3_t                        LayerOffset3_f;
    };

    /* 3.6.6.44 ClipWindowOffset3 */
    union {
        __IO  uint32_t                              ClipWindowOffset3;
        Fetch_ClipWindowOffset3_t                   ClipWindowOffset3_f;
    };

    /* 3.6.6.45 ClipWindowDimensions3 */
    union {
        __IO  uint32_t                              ClipWindowDimensions3;
        Fetch_ClipWindowDimensions3_t               ClipWindowDimensions3_f;
    };

    /* 3.6.6.46 ConstantColor3 */
    union {
        __IO  uint32_t                              ConstantColor3;
        Fetch_ConstantColor3_t                      ConstantColor3_f;
    };

    /* 3.6.6.47 LayerProperty3 */
    union {
        __IO  uint32_t                              LayerProperty3;
        Fetch_LayerProperty3_t                      LayerProperty3_f;
    };

    /* 3.6.6.48 BaseAddress4 */
    union {
        __IO  uint32_t                              BaseAddress4;
        Fetch_BaseAddress4_t                        BaseAddress4_f;
    };

    /* 3.6.6.49 SourceBufferAttributes4 */
    union {
        __IO  uint32_t                              SourceBufferAttributes4;
        Fetch_SourceBufferAttributes4_t             SourceBufferAttributes4_f;
    };

    /* 3.6.6.50 SourceBufferDimension4 */
    union {
        __IO  uint32_t                              SourceBufferDimension4;
        Fetch_SourceBufferDimension4_t              SourceBufferDimension4_f;
    };

    /* 3.6.6.51 ColorComponentBits4 */
    union {
        __IO  uint32_t                              ColorComponentBits4;
        Fetch_ColorComponentBits4_t                 ColorComponentBits4_f;
    };

    /* 3.6.6.52 ColorComponentShift4 */
    union {
        __IO  uint32_t                              ColorComponentShift4;
        Fetch_ColorComponentShift4_t                ColorComponentShift4_f;
    };

    /* 3.6.6.53 LayerOffset4 */
    union {
        __IO  uint32_t                              LayerOffset4;
        Fetch_LayerOffset4_t                        LayerOffset4_f;
    };

    /* 3.6.6.54 ClipWindowOffset4 */
    union {
        __IO  uint32_t                              ClipWindowOffset4;
        Fetch_ClipWindowOffset4_t                   ClipWindowOffset4_f;
    };

    /* 3.6.6.55 ClipWindowDimensions4 */
    union {
        __IO  uint32_t                              ClipWindowDimensions4;
        Fetch_ClipWindowDimensions4_t               ClipWindowDimensions4_f;
    };

    /* 3.6.6.56 ConstantColor4 */
    union {
        __IO  uint32_t                              ConstantColor4;
        Fetch_ConstantColor4_t                      ConstantColor4_f;
    };

    /* 3.6.6.57 LayerProperty4 */
    union {
        __IO  uint32_t                              LayerProperty4;
        Fetch_LayerProperty4_t                      LayerProperty4_f;
    };

    /* 3.6.6.58 BaseAddress5 */
    union {
        __IO  uint32_t                              BaseAddress5;
        Fetch_BaseAddress5_t                        BaseAddress5_f;
    };

    /* 3.6.6.59 SourceBufferAttributes5 */
    union {
        __IO  uint32_t                              SourceBufferAttributes5;
        Fetch_SourceBufferAttributes5_t             SourceBufferAttributes5_f;
    };

    /* 3.6.6.60 SourceBufferDimension5 */
    union {
        __IO  uint32_t                              SourceBufferDimension5;
        Fetch_SourceBufferDimension5_t              SourceBufferDimension5_f;
    };

    /* 3.6.6.61 ColorComponentBits5 */
    union {
        __IO  uint32_t                              ColorComponentBits5;
        Fetch_ColorComponentBits5_t                 ColorComponentBits5_f;
    };

    /* 3.6.6.62 ColorComponentShift5 */
    union {
        __IO  uint32_t                              ColorComponentShift5;
        Fetch_ColorComponentShift5_t                ColorComponentShift5_f;
    };

    /* 3.6.6.63 LayerOffset5 */
    union {
        __IO  uint32_t                              LayerOffset5;
        Fetch_LayerOffset5_t                        LayerOffset5_f;
    };

    /* 3.6.6.64 ClipWindowOffset5 */
    union {
        __IO  uint32_t                              ClipWindowOffset5;
        Fetch_ClipWindowOffset5_t                   ClipWindowOffset5_f;
    };

    /* 3.6.6.65 ClipWindowDimensions5 */
    union {
        __IO  uint32_t                              ClipWindowDimensions5;
        Fetch_ClipWindowDimensions5_t               ClipWindowDimensions5_f;
    };

    /* 3.6.6.66 ConstantColor5 */
    union {
        __IO  uint32_t                              ConstantColor5;
        Fetch_ConstantColor5_t                      ConstantColor5_f;
    };

    /* 3.6.6.67 LayerProperty5 */
    union {
        __IO  uint32_t                              LayerProperty5;
        Fetch_LayerProperty5_t                      LayerProperty5_f;
    };

    /* 3.6.6.68 BaseAddress6 */
    union {
        __IO  uint32_t                              BaseAddress6;
        Fetch_BaseAddress6_t                        BaseAddress6_f;
    };

    /* 3.6.6.69 SourceBufferAttributes6 */
    union {
        __IO  uint32_t                              SourceBufferAttributes6;
        Fetch_SourceBufferAttributes6_t             SourceBufferAttributes6_f;
    };

    /* 3.6.6.70 SourceBufferDimension6 */
    union {
        __IO  uint32_t                              SourceBufferDimension6;
        Fetch_SourceBufferDimension6_t              SourceBufferDimension6_f;
    };

    /* 3.6.6.71 ColorComponentBits6 */
    union {
        __IO  uint32_t                              ColorComponentBits6;
        Fetch_ColorComponentBits6_t                 ColorComponentBits6_f;
    };

    /* 3.6.6.72 ColorComponentShift6 */
    union {
        __IO  uint32_t                              ColorComponentShift6;
        Fetch_ColorComponentShift6_t                ColorComponentShift6_f;
    };

    /* 3.6.6.73 LayerOffset6 */
    union {
        __IO  uint32_t                              LayerOffset6;
        Fetch_LayerOffset6_t                        LayerOffset6_f;
    };

    /* 3.6.6.74 ClipWindowOffset6 */
    union {
        __IO  uint32_t                              ClipWindowOffset6;
        Fetch_ClipWindowOffset6_t                   ClipWindowOffset6_f;
    };

    /* 3.6.6.75 ClipWindowDimensions6 */
    union {
        __IO  uint32_t                              ClipWindowDimensions6;
        Fetch_ClipWindowDimensions6_t               ClipWindowDimensions6_f;
    };

    /* 3.6.6.76 ConstantColor6 */
    union {
        __IO  uint32_t                              ConstantColor6;
        Fetch_ConstantColor6_t                      ConstantColor6_f;
    };

    /* 3.6.6.77 LayerProperty6 */
    union {
        __IO  uint32_t                              LayerProperty6;
        Fetch_LayerProperty6_t                      LayerProperty6_f;
    };

    /* 3.6.6.78 BaseAddress7 */
    union {
        __IO  uint32_t                              BaseAddress7;
        Fetch_BaseAddress7_t                        BaseAddress7_f;
    };

    /* 3.6.6.79 SourceBufferAttributes7 */
    union {
        __IO  uint32_t                              SourceBufferAttributes7;
        Fetch_SourceBufferAttributes7_t             SourceBufferAttributes7_f;
    };

    /* 3.6.6.80 SourceBufferDimension7 */
    union {
        __IO  uint32_t                              SourceBufferDimension7;
        Fetch_SourceBufferDimension7_t              SourceBufferDimension7_f;
    };

    /* 3.6.6.81 ColorComponentBits7 */
    union {
        __IO  uint32_t                              ColorComponentBits7;
        Fetch_ColorComponentBits7_t                 ColorComponentBits7_f;
    };

    /* 3.6.6.82 ColorComponentShift7 */
    union {
        __IO  uint32_t                              ColorComponentShift7;
        Fetch_ColorComponentShift7_t                ColorComponentShift7_f;
    };

    /* 3.6.6.83 LayerOffset7 */
    union {
        __IO  uint32_t                              LayerOffset7;
        Fetch_LayerOffset7_t                        LayerOffset7_f;
    };

    /* 3.6.6.84 ClipWindowOffset7 */
    union {
        __IO  uint32_t                              ClipWindowOffset7;
        Fetch_ClipWindowOffset7_t                   ClipWindowOffset7_f;
    };

    /* 3.6.6.85 ClipWindowDimensions7 */
    union {
        __IO  uint32_t                              ClipWindowDimensions7;
        Fetch_ClipWindowDimensions7_t               ClipWindowDimensions7_f;
    };

    /* 3.6.6.86 ConstantColor7 */
    union {
        __IO  uint32_t                              ConstantColor7;
        Fetch_ConstantColor7_t                      ConstantColor7_f;
    };

    /* 3.6.6.87 LayerProperty7 */
    union {
        __IO  uint32_t                              LayerProperty7;
        Fetch_LayerProperty7_t                      LayerProperty7_f;
    };

    /* 3.6.6.88 FrameDimensions */
    union {
        __IO  uint32_t                              FrameDimensions;
        Fetch_FrameDimensions_t                     FrameDimensions_f;
    };

    /* 3.6.6.89 FrameResampling */
    union {
        __IO  uint32_t                              FrameResampling;
        Fetch_FrameResampling_t                     FrameResampling_f;
    };

    /* 3.6.6.113 Control */
    union {
        __IO  uint32_t                              Control;
        Fetch_Control_t                             Control_f;
    };

    /* 3.6.6.114 TriggerEnable */
    union {
        __IO  uint32_t                              TriggerEnable;
        Fetch_TriggerEnable_t                       TriggerEnable_f;
    };

    /* 3.6.6.115 ControlTrigger */
    union {
        __IO  uint32_t                              ControlTrigger;
        Fetch_ControlTrigger_t                      ControlTrigger_f;
    };

    /* 3.6.6.116 Start */
    union {
        __IO  uint32_t                              Start;
        Fetch_Start_t                               Start_f;
    };

    /* 3.6.6.117 FetchType */
    union {
        __IO  uint32_t                              FetchType;
        Fetch_FetchType_t                           FetchType_f;
    };

    /* 3.6.6.120 BurstBufferProperties */
    union {
        __IO  uint32_t                              BurstBufferProperties;
        Fetch_BurstBufferProperties_t               BurstBufferProperties_f;
    };

    /* 3.6.6.121 Status */
    union {
        __IO  uint32_t                              Status;
        Fetch_Status_t                              Status_f;
    };

    uint32_t                                        RESERVED_0174[163];

    /* 3.6.6.122 ColorPalette[0..255] */
    union {
        __IO  uint32_t                              ColorPalette[256];
        Fetch_ColorPalette_t                        ColorPalette_f[256];
    };

} IRIS_MDL2_FetchLayerL0_TypeDef;


/* ----[ LayerBlendL1 ]------------------------------------------------------------------ */
typedef struct {
    /* 3.6.15.1 LockUnlock */
    union {
        __IO  uint32_t                              LockUnlock;
        LayerBlend_LockUnlock_t                     LockUnlock_f;
    };

    /* 3.6.15.2 LockStatus */
    union {
        __IO  uint32_t                              LockStatus;
        LayerBlend_LockStatus_t                     LockStatus_f;
    };

    /* 3.6.15.3 StaticControl */
    union {
        __IO  uint32_t                              StaticControl;
        LayerBlend_StaticControl_t                  StaticControl_f;
    };

    /* 3.6.15.4 Control */
    union {
        __IO  uint32_t                              Control;
        LayerBlend_Control_t                        Control_f;
    };

    /* 3.6.15.5 BlendControl */
    union {
        __IO  uint32_t                              BlendControl;
        LayerBlend_BlendControl_t                   BlendControl_f;
    };

    /* 3.6.15.6 Position */
    union {
        __IO  uint32_t                              Position;
        LayerBlend_Position_t                       Position_f;
    };

} IRIS_MDL2_LayerBlendL1_TypeDef;


/* ----[ LayerBlendL2 ]------------------------------------------------------------------ */
typedef struct {
    /* 3.6.15.1 LockUnlock */
    union {
        __IO  uint32_t                              LockUnlock;
        LayerBlend_LockUnlock_t                     LockUnlock_f;
    };

    /* 3.6.15.2 LockStatus */
    union {
        __IO  uint32_t                              LockStatus;
        LayerBlend_LockStatus_t                     LockStatus_f;
    };

    /* 3.6.15.3 StaticControl */
    union {
        __IO  uint32_t                              StaticControl;
        LayerBlend_StaticControl_t                  StaticControl_f;
    };

    /* 3.6.15.4 Control */
    union {
        __IO  uint32_t                              Control;
        LayerBlend_Control_t                        Control_f;
    };

    /* 3.6.15.5 BlendControl */
    union {
        __IO  uint32_t                              BlendControl;
        LayerBlend_BlendControl_t                   BlendControl_f;
    };

    /* 3.6.15.6 Position */
    union {
        __IO  uint32_t                              Position;
        LayerBlend_Position_t                       Position_f;
    };

} IRIS_MDL2_LayerBlendL2_TypeDef;


/* ----[ ExtSrcL8 ]---------------------------------------------------------------------- */
typedef struct {
    /* 3.6.9.1 LockUnlock */
    union {
        __IO  uint32_t                              LockUnlock;
        ExtSrc_LockUnlock_t                         LockUnlock_f;
    };

    /* 3.6.9.2 LockStatus */
    union {
        __IO  uint32_t                              LockStatus;
        ExtSrc_LockStatus_t                         LockStatus_f;
    };

    /* 3.6.9.3 StaticControl */
    union {
        __IO  uint32_t                              StaticControl;
        ExtSrc_StaticControl_t                      StaticControl_f;
    };

    /* 3.6.9.4 ClipWindowOffset */
    union {
        __IO  uint32_t                              ClipWindowOffset;
        ExtSrc_ClipWindowOffset_t                   ClipWindowOffset_f;
    };

    /* 3.6.9.5 ClipWindowDimension */
    union {
        __IO  uint32_t                              ClipWindowDimension;
        ExtSrc_ClipWindowDimension_t                ClipWindowDimension_f;
    };

    /* 3.6.9.6 ColorComponentBits */
    union {
        __IO  uint32_t                              ColorComponentBits;
        ExtSrc_ColorComponentBits_t                 ColorComponentBits_f;
    };

    /* 3.6.9.7 ColorComponentShift */
    union {
        __IO  uint32_t                              ColorComponentShift;
        ExtSrc_ColorComponentShift_t                ColorComponentShift_f;
    };

    /* 3.6.9.8 ConstantColor */
    union {
        __IO  uint32_t                              ConstantColor;
        ExtSrc_ConstantColor_t                      ConstantColor_f;
    };

    /* 3.6.9.9 Control */
    union {
        __IO  uint32_t                              Control;
        ExtSrc_Control_t                            Control_f;
    };

    /* 3.6.9.10 ControlTrigger */
    union {
        __IO  uint32_t                              ControlTrigger;
        ExtSrc_ControlTrigger_t                     ControlTrigger_f;
    };

    /* 3.6.9.11 Start */
    union {
        __IO  uint32_t                              Start;
        ExtSrc_Start_t                              Start_f;
    };

} IRIS_MDL2_ExtSrcL8_TypeDef;


/* ----[ DisEngCfg ]--------------------------------------------------------------------- */
typedef struct {
    /* 3.6.5.1 LockUnlock0 */
    union {
        __IO  uint32_t                              LockUnlock0;
        DisEngCfg_LockUnlock0_t                     LockUnlock0_f;
    };

    /* 3.6.5.2 LockStatus0 */
    union {
        __IO  uint32_t                              LockStatus0;
        DisEngCfg_LockStatus0_t                     LockStatus0_f;
    };

    /* 3.6.5.3 ClockCtrl0 */
    union {
        __IO  uint32_t                              ClockCtrl0;
        DisEngCfg_ClockCtrl0_t                      ClockCtrl0_f;
    };

    /* 3.6.5.4 PolarityCtrl0 */
    union {
        __IO  uint32_t                              PolarityCtrl0;
        DisEngCfg_PolarityCtrl0_t                   PolarityCtrl0_f;
    };

    /* 3.6.5.5 SrcSelect0 */
    union {
        __IO  uint32_t                              SrcSelect0;
        DisEngCfg_SrcSelect0_t                      SrcSelect0_f;
    };

} IRIS_MDL2_DisEngCfg_TypeDef;


/* ----[ FrameGenL0 ]-------------------------------------------------------------------- */
typedef struct {
    /* 3.6.18.1 LockUnlock */
    union {
        __IO  uint32_t                              LockUnlock;
        FrameGen_LockUnlock_t                       LockUnlock_f;
    };

    /* 3.6.18.2 LockStatus */
    union {
        __IO  uint32_t                              LockStatus;
        FrameGen_LockStatus_t                       LockStatus_f;
    };

    /* 3.6.18.3 FgStCtrl */
    union {
        __IO  uint32_t                              FgStCtrl;
        FrameGen_FgStCtrl_t                         FgStCtrl_f;
    };

    /* 3.6.18.4 HtCfg1 */
    union {
        __IO  uint32_t                              HtCfg1;
        FrameGen_HtCfg1_t                           HtCfg1_f;
    };

    /* 3.6.18.5 HtCfg2 */
    union {
        __IO  uint32_t                              HtCfg2;
        FrameGen_HtCfg2_t                           HtCfg2_f;
    };

    /* 3.6.18.6 VtCfg1 */
    union {
        __IO  uint32_t                              VtCfg1;
        FrameGen_VtCfg1_t                           VtCfg1_f;
    };

    /* 3.6.18.7 VtCfg2 */
    union {
        __IO  uint32_t                              VtCfg2;
        FrameGen_VtCfg2_t                           VtCfg2_f;
    };

    /* 3.6.18.8 Int0Config */
    union {
        __IO  uint32_t                              Int0Config;
        FrameGen_Int0Config_t                       Int0Config_f;
    };

    /* 3.6.18.9 Int1Config */
    union {
        __IO  uint32_t                              Int1Config;
        FrameGen_Int1Config_t                       Int1Config_f;
    };

    /* 3.6.18.10 Int2Config */
    union {
        __IO  uint32_t                              Int2Config;
        FrameGen_Int2Config_t                       Int2Config_f;
    };

    /* 3.6.18.11 Int3Config */
    union {
        __IO  uint32_t                              Int3Config;
        FrameGen_Int3Config_t                       Int3Config_f;
    };

    /* 3.6.18.12 PKickConfig */
    union {
        __IO  uint32_t                              PKickConfig;
        FrameGen_PKickConfig_t                      PKickConfig_f;
    };

    /* 3.6.18.13 SKickConfig */
    union {
        __IO  uint32_t                              SKickConfig;
        FrameGen_SKickConfig_t                      SKickConfig_f;
    };

    /* 3.6.18.14 SecStatConfig */
    union {
        __IO  uint32_t                              SecStatConfig;
        FrameGen_SecStatConfig_t                    SecStatConfig_f;
    };

    /* 3.6.18.21 FgKSDR */
    union {
        __IO  uint32_t                              FgKSDR;
        FrameGen_FgKSDR_t                           FgKSDR_f;
    };

    /* 3.6.18.22 PaCfg */
    union {
        __IO  uint32_t                              PaCfg;
        FrameGen_PaCfg_t                            PaCfg_f;
    };

    /* 3.6.18.23 SaCfg */
    union {
        __IO  uint32_t                              SaCfg;
        FrameGen_SaCfg_t                            SaCfg_f;
    };

    /* 3.6.18.24 FgInCtrl */
    union {
        __IO  uint32_t                              FgInCtrl;
        FrameGen_FgInCtrl_t                         FgInCtrl_f;
    };

    /* 3.6.18.25 FgInCtrlPanic */
    union {
        __IO  uint32_t                              FgInCtrlPanic;
        FrameGen_FgInCtrlPanic_t                    FgInCtrlPanic_f;
    };

    /* 3.6.18.26 FgCCR */
    union {
        __IO  uint32_t                              FgCCR;
        FrameGen_FgCCR_t                            FgCCR_f;
    };

    /* 3.6.18.27 FgEnable */
    union {
        __IO  uint32_t                              FgEnable;
        FrameGen_FgEnable_t                         FgEnable_f;
    };

    /* 3.6.18.28 FgSlr */
    union {
        __IO  uint32_t                              FgSlr;
        FrameGen_FgSlr_t                            FgSlr_f;
    };

    /* 3.6.18.29 FgEnSts */
    union {
        __IO  uint32_t                              FgEnSts;
        FrameGen_FgEnSts_t                          FgEnSts_f;
    };

    /* 3.6.18.30 FgTimeStamp */
    union {
        __IO  uint32_t                              FgTimeStamp;
        FrameGen_FgTimeStamp_t                      FgTimeStamp_f;
    };

    /* 3.6.18.31 FgChStat */
    union {
        __IO  uint32_t                              FgChStat;
        FrameGen_FgChStat_t                         FgChStat_f;
    };

    /* 3.6.18.32 FgChStatClr */
    union {
        __IO  uint32_t                              FgChStatClr;
        FrameGen_FgChStatClr_t                      FgChStatClr_f;
    };

} IRIS_MDL2_FrameGenL0_TypeDef;


/* ----[ GammaCor0 ]--------------------------------------------------------------------- */
typedef struct {
    /* 3.6.13.1 LockUnlock */
    union {
        __IO  uint32_t                              LockUnlock;
        GammaCor_LockUnlock_t                       LockUnlock_f;
    };

    /* 3.6.13.2 LockStatus */
    union {
        __IO  uint32_t                              LockStatus;
        GammaCor_LockStatus_t                       LockStatus_f;
    };

    /* 3.6.13.3 StaticControl */
    union {
        __IO  uint32_t                              StaticControl;
        GammaCor_StaticControl_t                    StaticControl_f;
    };

    /* 3.6.13.4 LutStart */
    union {
        __IO  uint32_t                              LutStart;
        GammaCor_LutStart_t                         LutStart_f;
    };

    /* 3.6.13.5 LutDeltas */
    union {
        __IO  uint32_t                              LutDeltas;
        GammaCor_LutDeltas_t                        LutDeltas_f;
    };

    /* 3.6.13.6 Control */
    union {
        __IO  uint32_t                              Control;
        GammaCor_Control_t                          Control_f;
    };

} IRIS_MDL2_GammaCor0_TypeDef;


/* ----[ Dither0 ]----------------------------------------------------------------------- */
typedef struct {
    /* 3.6.14.1 LockUnlock */
    union {
        __IO  uint32_t                              LockUnlock;
        Dither_LockUnlock_t                         LockUnlock_f;
    };

    /* 3.6.14.2 LockStatus */
    union {
        __IO  uint32_t                              LockStatus;
        Dither_LockStatus_t                         LockStatus_f;
    };

    /* 3.6.14.3 Control */
    union {
        __IO  uint32_t                              Control;
        Dither_Control_t                            Control_f;
    };

    /* 3.6.14.4 DitherControl */
    union {
        __IO  uint32_t                              DitherControl;
        Dither_DitherControl_t                      DitherControl_f;
    };

    /* 3.6.14.5 Release */
    union {
        __IO  uint32_t                              Release;
        Dither_Release_t                            Release_f;
    };

} IRIS_MDL2_Dither0_TypeDef;


/* ----[ TCon0 ]------------------------------------------------------------------------- */
typedef struct {
    /* 3.6.19.1 SSqCnts[0..63] */
    union {
        __IO  uint32_t                              SSqCnts[64];
        TCon_SSqCnts_t                              SSqCnts_f[64];
    };

    uint32_t                                        RESERVED_0100[192];

    /* 3.6.19.2 LockUnlock */
    union {
        __IO  uint32_t                              LockUnlock;
        TCon_LockUnlock_t                           LockUnlock_f;
    };

    /* 3.6.19.3 LockStatus */
    union {
        __IO  uint32_t                              LockStatus;
        TCon_LockStatus_t                           LockStatus_f;
    };

    /* 3.6.19.4 SSqCycle */
    union {
        __IO  uint32_t                              SSqCycle;
        TCon_SSqCycle_t                             SSqCycle_f;
    };

    /* 3.6.19.5 SWreset */
    union {
        __IO  uint32_t                              SWreset;
        TCon_SWreset_t                              SWreset_f;
    };

    /* 3.6.19.6 TCON_CTRL */
    union {
        __IO  uint32_t                              TCON_CTRL;
        TCon_TCON_CTRL_t                            TCON_CTRL_f;
    };

    /* 3.6.19.7 RSDSInvCtrl */
    union {
        __IO  uint32_t                              RSDSInvCtrl;
        TCon_RSDSInvCtrl_t                          RSDSInvCtrl_f;
    };

    /* 3.6.19.8 MapBit3_0 */
    union {
        __IO  uint32_t                              MapBit3_0;
        TCon_MapBit3_0_t                            MapBit3_0_f;
    };

    /* 3.6.19.9 MapBit7_4 */
    union {
        __IO  uint32_t                              MapBit7_4;
        TCon_MapBit7_4_t                            MapBit7_4_f;
    };

    /* 3.6.19.10 MapBit11_8 */
    union {
        __IO  uint32_t                              MapBit11_8;
        TCon_MapBit11_8_t                           MapBit11_8_f;
    };

    /* 3.6.19.11 MapBit15_12 */
    union {
        __IO  uint32_t                              MapBit15_12;
        TCon_MapBit15_12_t                          MapBit15_12_f;
    };

    /* 3.6.19.12 MapBit19_16 */
    union {
        __IO  uint32_t                              MapBit19_16;
        TCon_MapBit19_16_t                          MapBit19_16_f;
    };

    /* 3.6.19.13 MapBit23_20 */
    union {
        __IO  uint32_t                              MapBit23_20;
        TCon_MapBit23_20_t                          MapBit23_20_f;
    };

    /* 3.6.19.14 MapBit27_24 */
    union {
        __IO  uint32_t                              MapBit27_24;
        TCon_MapBit27_24_t                          MapBit27_24_f;
    };

    /* 3.6.19.15 MapBit3_0_Dual */
    union {
        __IO  uint32_t                              MapBit3_0_Dual;
        TCon_MapBit3_0_Dual_t                       MapBit3_0_Dual_f;
    };

    /* 3.6.19.16 MapBit7_4_Dual */
    union {
        __IO  uint32_t                              MapBit7_4_Dual;
        TCon_MapBit7_4_Dual_t                       MapBit7_4_Dual_f;
    };

    /* 3.6.19.17 MapBit11_8_Dual */
    union {
        __IO  uint32_t                              MapBit11_8_Dual;
        TCon_MapBit11_8_Dual_t                      MapBit11_8_Dual_f;
    };

    /* 3.6.19.18 MapBit15_12_Dual */
    union {
        __IO  uint32_t                              MapBit15_12_Dual;
        TCon_MapBit15_12_Dual_t                     MapBit15_12_Dual_f;
    };

    /* 3.6.19.19 MapBit19_16_Dual */
    union {
        __IO  uint32_t                              MapBit19_16_Dual;
        TCon_MapBit19_16_Dual_t                     MapBit19_16_Dual_f;
    };

    /* 3.6.19.20 MapBit23_20_Dual */
    union {
        __IO  uint32_t                              MapBit23_20_Dual;
        TCon_MapBit23_20_Dual_t                     MapBit23_20_Dual_f;
    };

    /* 3.6.19.21 MapBit27_24_Dual */
    union {
        __IO  uint32_t                              MapBit27_24_Dual;
        TCon_MapBit27_24_Dual_t                     MapBit27_24_Dual_f;
    };

    /* 3.6.19.22 SPG0PosOn */
    union {
        __IO  uint32_t                              SPG0PosOn;
        TCon_SPG0PosOn_t                            SPG0PosOn_f;
    };

    /* 3.6.19.23 SPG0MaskOn */
    union {
        __IO  uint32_t                              SPG0MaskOn;
        TCon_SPG0MaskOn_t                           SPG0MaskOn_f;
    };

    /* 3.6.19.24 SPG0PosOff */
    union {
        __IO  uint32_t                              SPG0PosOff;
        TCon_SPG0PosOff_t                           SPG0PosOff_f;
    };

    /* 3.6.19.25 SPG0MaskOff */
    union {
        __IO  uint32_t                              SPG0MaskOff;
        TCon_SPG0MaskOff_t                          SPG0MaskOff_f;
    };

    /* 3.6.19.26 SPG1PosOn */
    union {
        __IO  uint32_t                              SPG1PosOn;
        TCon_SPG1PosOn_t                            SPG1PosOn_f;
    };

    /* 3.6.19.27 SPG1MaskOn */
    union {
        __IO  uint32_t                              SPG1MaskOn;
        TCon_SPG1MaskOn_t                           SPG1MaskOn_f;
    };

    /* 3.6.19.28 SPG1PosOff */
    union {
        __IO  uint32_t                              SPG1PosOff;
        TCon_SPG1PosOff_t                           SPG1PosOff_f;
    };

    /* 3.6.19.29 SPG1MaskOff */
    union {
        __IO  uint32_t                              SPG1MaskOff;
        TCon_SPG1MaskOff_t                          SPG1MaskOff_f;
    };

    /* 3.6.19.30 SPG2PosOn */
    union {
        __IO  uint32_t                              SPG2PosOn;
        TCon_SPG2PosOn_t                            SPG2PosOn_f;
    };

    /* 3.6.19.31 SPG2MaskOn */
    union {
        __IO  uint32_t                              SPG2MaskOn;
        TCon_SPG2MaskOn_t                           SPG2MaskOn_f;
    };

    /* 3.6.19.32 SPG2PosOff */
    union {
        __IO  uint32_t                              SPG2PosOff;
        TCon_SPG2PosOff_t                           SPG2PosOff_f;
    };

    /* 3.6.19.33 SPG2MaskOff */
    union {
        __IO  uint32_t                              SPG2MaskOff;
        TCon_SPG2MaskOff_t                          SPG2MaskOff_f;
    };

    /* 3.6.19.34 SPG3PosOn */
    union {
        __IO  uint32_t                              SPG3PosOn;
        TCon_SPG3PosOn_t                            SPG3PosOn_f;
    };

    /* 3.6.19.35 SPG3MaskOn */
    union {
        __IO  uint32_t                              SPG3MaskOn;
        TCon_SPG3MaskOn_t                           SPG3MaskOn_f;
    };

    /* 3.6.19.36 SPG3PosOff */
    union {
        __IO  uint32_t                              SPG3PosOff;
        TCon_SPG3PosOff_t                           SPG3PosOff_f;
    };

    /* 3.6.19.37 SPG3MaskOff */
    union {
        __IO  uint32_t                              SPG3MaskOff;
        TCon_SPG3MaskOff_t                          SPG3MaskOff_f;
    };

    /* 3.6.19.38 SPG4PosOn */
    union {
        __IO  uint32_t                              SPG4PosOn;
        TCon_SPG4PosOn_t                            SPG4PosOn_f;
    };

    /* 3.6.19.39 SPG4MaskOn */
    union {
        __IO  uint32_t                              SPG4MaskOn;
        TCon_SPG4MaskOn_t                           SPG4MaskOn_f;
    };

    /* 3.6.19.40 SPG4PosOff */
    union {
        __IO  uint32_t                              SPG4PosOff;
        TCon_SPG4PosOff_t                           SPG4PosOff_f;
    };

    /* 3.6.19.41 SPG4MaskOff */
    union {
        __IO  uint32_t                              SPG4MaskOff;
        TCon_SPG4MaskOff_t                          SPG4MaskOff_f;
    };

    /* 3.6.19.42 SPG5PosOn */
    union {
        __IO  uint32_t                              SPG5PosOn;
        TCon_SPG5PosOn_t                            SPG5PosOn_f;
    };

    /* 3.6.19.43 SPG5MaskOn */
    union {
        __IO  uint32_t                              SPG5MaskOn;
        TCon_SPG5MaskOn_t                           SPG5MaskOn_f;
    };

    /* 3.6.19.44 SPG5PosOff */
    union {
        __IO  uint32_t                              SPG5PosOff;
        TCon_SPG5PosOff_t                           SPG5PosOff_f;
    };

    /* 3.6.19.45 SPG5MaskOff */
    union {
        __IO  uint32_t                              SPG5MaskOff;
        TCon_SPG5MaskOff_t                          SPG5MaskOff_f;
    };

    /* 3.6.19.46 SPG6PosOn */
    union {
        __IO  uint32_t                              SPG6PosOn;
        TCon_SPG6PosOn_t                            SPG6PosOn_f;
    };

    /* 3.6.19.47 SPG6MaskOn */
    union {
        __IO  uint32_t                              SPG6MaskOn;
        TCon_SPG6MaskOn_t                           SPG6MaskOn_f;
    };

    /* 3.6.19.48 SPG6PosOff */
    union {
        __IO  uint32_t                              SPG6PosOff;
        TCon_SPG6PosOff_t                           SPG6PosOff_f;
    };

    /* 3.6.19.49 SPG6MaskOff */
    union {
        __IO  uint32_t                              SPG6MaskOff;
        TCon_SPG6MaskOff_t                          SPG6MaskOff_f;
    };

    /* 3.6.19.50 SPG7PosOn */
    union {
        __IO  uint32_t                              SPG7PosOn;
        TCon_SPG7PosOn_t                            SPG7PosOn_f;
    };

    /* 3.6.19.51 SPG7MaskOn */
    union {
        __IO  uint32_t                              SPG7MaskOn;
        TCon_SPG7MaskOn_t                           SPG7MaskOn_f;
    };

    /* 3.6.19.52 SPG7PosOff */
    union {
        __IO  uint32_t                              SPG7PosOff;
        TCon_SPG7PosOff_t                           SPG7PosOff_f;
    };

    /* 3.6.19.53 SPG7MaskOff */
    union {
        __IO  uint32_t                              SPG7MaskOff;
        TCon_SPG7MaskOff_t                          SPG7MaskOff_f;
    };

    /* 3.6.19.54 SPG8PosOn */
    union {
        __IO  uint32_t                              SPG8PosOn;
        TCon_SPG8PosOn_t                            SPG8PosOn_f;
    };

    /* 3.6.19.55 SPG8MaskOn */
    union {
        __IO  uint32_t                              SPG8MaskOn;
        TCon_SPG8MaskOn_t                           SPG8MaskOn_f;
    };

    /* 3.6.19.56 SPG8PosOff */
    union {
        __IO  uint32_t                              SPG8PosOff;
        TCon_SPG8PosOff_t                           SPG8PosOff_f;
    };

    /* 3.6.19.57 SPG8MaskOff */
    union {
        __IO  uint32_t                              SPG8MaskOff;
        TCon_SPG8MaskOff_t                          SPG8MaskOff_f;
    };

    /* 3.6.19.58 SPG9PosOn */
    union {
        __IO  uint32_t                              SPG9PosOn;
        TCon_SPG9PosOn_t                            SPG9PosOn_f;
    };

    /* 3.6.19.59 SPG9MaskOn */
    union {
        __IO  uint32_t                              SPG9MaskOn;
        TCon_SPG9MaskOn_t                           SPG9MaskOn_f;
    };

    /* 3.6.19.60 SPG9PosOff */
    union {
        __IO  uint32_t                              SPG9PosOff;
        TCon_SPG9PosOff_t                           SPG9PosOff_f;
    };

    /* 3.6.19.61 SPG9MaskOff */
    union {
        __IO  uint32_t                              SPG9MaskOff;
        TCon_SPG9MaskOff_t                          SPG9MaskOff_f;
    };

    /* 3.6.19.62 SPG10PosOn */
    union {
        __IO  uint32_t                              SPG10PosOn;
        TCon_SPG10PosOn_t                           SPG10PosOn_f;
    };

    /* 3.6.19.63 SPG10MaskOn */
    union {
        __IO  uint32_t                              SPG10MaskOn;
        TCon_SPG10MaskOn_t                          SPG10MaskOn_f;
    };

    /* 3.6.19.64 SPG10PosOff */
    union {
        __IO  uint32_t                              SPG10PosOff;
        TCon_SPG10PosOff_t                          SPG10PosOff_f;
    };

    /* 3.6.19.65 SPG10MaskOff */
    union {
        __IO  uint32_t                              SPG10MaskOff;
        TCon_SPG10MaskOff_t                         SPG10MaskOff_f;
    };

    /* 3.6.19.66 SPG11PosOn */
    union {
        __IO  uint32_t                              SPG11PosOn;
        TCon_SPG11PosOn_t                           SPG11PosOn_f;
    };

    /* 3.6.19.67 SPG11MaskOn */
    union {
        __IO  uint32_t                              SPG11MaskOn;
        TCon_SPG11MaskOn_t                          SPG11MaskOn_f;
    };

    /* 3.6.19.68 SPG11PosOff */
    union {
        __IO  uint32_t                              SPG11PosOff;
        TCon_SPG11PosOff_t                          SPG11PosOff_f;
    };

    /* 3.6.19.69 SPG11MaskOff */
    union {
        __IO  uint32_t                              SPG11MaskOff;
        TCon_SPG11MaskOff_t                         SPG11MaskOff_f;
    };

    /* 3.6.19.70 SMx0Sigs */
    union {
        __IO  uint32_t                              SMx0Sigs;
        TCon_SMx0Sigs_t                             SMx0Sigs_f;
    };

    /* 3.6.19.71 SMx0FctTable */
    union {
        __IO  uint32_t                              SMx0FctTable;
        TCon_SMx0FctTable_t                         SMx0FctTable_f;
    };

    /* 3.6.19.72 SMx1Sigs */
    union {
        __IO  uint32_t                              SMx1Sigs;
        TCon_SMx1Sigs_t                             SMx1Sigs_f;
    };

    /* 3.6.19.73 SMx1FctTable */
    union {
        __IO  uint32_t                              SMx1FctTable;
        TCon_SMx1FctTable_t                         SMx1FctTable_f;
    };

    /* 3.6.19.74 SMx2Sigs */
    union {
        __IO  uint32_t                              SMx2Sigs;
        TCon_SMx2Sigs_t                             SMx2Sigs_f;
    };

    /* 3.6.19.75 SMx2FctTable */
    union {
        __IO  uint32_t                              SMx2FctTable;
        TCon_SMx2FctTable_t                         SMx2FctTable_f;
    };

    /* 3.6.19.76 SMx3Sigs */
    union {
        __IO  uint32_t                              SMx3Sigs;
        TCon_SMx3Sigs_t                             SMx3Sigs_f;
    };

    /* 3.6.19.77 SMx3FctTable */
    union {
        __IO  uint32_t                              SMx3FctTable;
        TCon_SMx3FctTable_t                         SMx3FctTable_f;
    };

    /* 3.6.19.78 SMx4Sigs */
    union {
        __IO  uint32_t                              SMx4Sigs;
        TCon_SMx4Sigs_t                             SMx4Sigs_f;
    };

    /* 3.6.19.79 SMx4FctTable */
    union {
        __IO  uint32_t                              SMx4FctTable;
        TCon_SMx4FctTable_t                         SMx4FctTable_f;
    };

    /* 3.6.19.80 SMx5Sigs */
    union {
        __IO  uint32_t                              SMx5Sigs;
        TCon_SMx5Sigs_t                             SMx5Sigs_f;
    };

    /* 3.6.19.81 SMx5FctTable */
    union {
        __IO  uint32_t                              SMx5FctTable;
        TCon_SMx5FctTable_t                         SMx5FctTable_f;
    };

    /* 3.6.19.82 SMx6Sigs */
    union {
        __IO  uint32_t                              SMx6Sigs;
        TCon_SMx6Sigs_t                             SMx6Sigs_f;
    };

    /* 3.6.19.83 SMx6FctTable */
    union {
        __IO  uint32_t                              SMx6FctTable;
        TCon_SMx6FctTable_t                         SMx6FctTable_f;
    };

    /* 3.6.19.84 SMx7Sigs */
    union {
        __IO  uint32_t                              SMx7Sigs;
        TCon_SMx7Sigs_t                             SMx7Sigs_f;
    };

    /* 3.6.19.85 SMx7FctTable */
    union {
        __IO  uint32_t                              SMx7FctTable;
        TCon_SMx7FctTable_t                         SMx7FctTable_f;
    };

    /* 3.6.19.86 SMx8Sigs */
    union {
        __IO  uint32_t                              SMx8Sigs;
        TCon_SMx8Sigs_t                             SMx8Sigs_f;
    };

    /* 3.6.19.87 SMx8FctTable */
    union {
        __IO  uint32_t                              SMx8FctTable;
        TCon_SMx8FctTable_t                         SMx8FctTable_f;
    };

    /* 3.6.19.88 SMx9Sigs */
    union {
        __IO  uint32_t                              SMx9Sigs;
        TCon_SMx9Sigs_t                             SMx9Sigs_f;
    };

    /* 3.6.19.89 SMx9FctTable */
    union {
        __IO  uint32_t                              SMx9FctTable;
        TCon_SMx9FctTable_t                         SMx9FctTable_f;
    };

    /* 3.6.19.90 SMx10Sigs */
    union {
        __IO  uint32_t                              SMx10Sigs;
        TCon_SMx10Sigs_t                            SMx10Sigs_f;
    };

    /* 3.6.19.91 SMx10FctTable */
    union {
        __IO  uint32_t                              SMx10FctTable;
        TCon_SMx10FctTable_t                        SMx10FctTable_f;
    };

    /* 3.6.19.92 SMx11Sigs */
    union {
        __IO  uint32_t                              SMx11Sigs;
        TCon_SMx11Sigs_t                            SMx11Sigs_f;
    };

    /* 3.6.19.93 SMx11FctTable */
    union {
        __IO  uint32_t                              SMx11FctTable;
        TCon_SMx11FctTable_t                        SMx11FctTable_f;
    };

} IRIS_MDL2_TCon0_TypeDef;


/* ----[ SigL0 ]------------------------------------------------------------------------- */
typedef struct {
    /* 3.6.20.1 LockUnlock */
    union {
        __IO  uint32_t                              LockUnlock;
        Sig_LockUnlock_t                            LockUnlock_f;
    };

    /* 3.6.20.2 LockStatus */
    union {
        __IO  uint32_t                              LockStatus;
        Sig_LockStatus_t                            LockStatus_f;
    };

    /* 3.6.20.3 StaticControl */
    union {
        __IO  uint32_t                              StaticControl;
        Sig_StaticControl_t                         StaticControl_f;
    };

    /* 3.6.20.4 PanicColor */
    union {
        __IO  uint32_t                              PanicColor;
        Sig_PanicColor_t                            PanicColor_f;
    };

    /* 3.6.20.5 EvalControl0 */
    union {
        __IO  uint32_t                              EvalControl0;
        Sig_EvalControl0_t                          EvalControl0_f;
    };

    /* 3.6.20.6 EvalUpperLeft0 */
    union {
        __IO  uint32_t                              EvalUpperLeft0;
        Sig_EvalUpperLeft0_t                        EvalUpperLeft0_f;
    };

    /* 3.6.20.7 EvalLowerRight0 */
    union {
        __IO  uint32_t                              EvalLowerRight0;
        Sig_EvalLowerRight0_t                       EvalLowerRight0_f;
    };

    /* 3.6.20.8 SigCRCRedRef0 */
    union {
        __IO  uint32_t                              SigCRCRedRef0;
        Sig_SigCRCRedRef0_t                         SigCRCRedRef0_f;
    };

    /* 3.6.20.9 SigCRCGreenRef0 */
    union {
        __IO  uint32_t                              SigCRCGreenRef0;
        Sig_SigCRCGreenRef0_t                       SigCRCGreenRef0_f;
    };

    /* 3.6.20.10 SigCRCBlueRef0 */
    union {
        __IO  uint32_t                              SigCRCBlueRef0;
        Sig_SigCRCBlueRef0_t                        SigCRCBlueRef0_f;
    };

    /* 3.6.20.11 SigCRCRed0 */
    union {
        __IO  uint32_t                              SigCRCRed0;
        Sig_SigCRCRed0_t                            SigCRCRed0_f;
    };

    /* 3.6.20.12 SigCRCGreen0 */
    union {
        __IO  uint32_t                              SigCRCGreen0;
        Sig_SigCRCGreen0_t                          SigCRCGreen0_f;
    };

    /* 3.6.20.13 SigCRCBlue0 */
    union {
        __IO  uint32_t                              SigCRCBlue0;
        Sig_SigCRCBlue0_t                           SigCRCBlue0_f;
    };

    /* 3.6.20.14 EvalControl1 */
    union {
        __IO  uint32_t                              EvalControl1;
        Sig_EvalControl1_t                          EvalControl1_f;
    };

    /* 3.6.20.15 EvalUpperLeft1 */
    union {
        __IO  uint32_t                              EvalUpperLeft1;
        Sig_EvalUpperLeft1_t                        EvalUpperLeft1_f;
    };

    /* 3.6.20.16 EvalLowerRight1 */
    union {
        __IO  uint32_t                              EvalLowerRight1;
        Sig_EvalLowerRight1_t                       EvalLowerRight1_f;
    };

    /* 3.6.20.17 SigCRCRedRef1 */
    union {
        __IO  uint32_t                              SigCRCRedRef1;
        Sig_SigCRCRedRef1_t                         SigCRCRedRef1_f;
    };

    /* 3.6.20.18 SigCRCGreenRef1 */
    union {
        __IO  uint32_t                              SigCRCGreenRef1;
        Sig_SigCRCGreenRef1_t                       SigCRCGreenRef1_f;
    };

    /* 3.6.20.19 SigCRCBlueRef1 */
    union {
        __IO  uint32_t                              SigCRCBlueRef1;
        Sig_SigCRCBlueRef1_t                        SigCRCBlueRef1_f;
    };

    /* 3.6.20.20 SigCRCRed1 */
    union {
        __IO  uint32_t                              SigCRCRed1;
        Sig_SigCRCRed1_t                            SigCRCRed1_f;
    };

    /* 3.6.20.21 SigCRCGreen1 */
    union {
        __IO  uint32_t                              SigCRCGreen1;
        Sig_SigCRCGreen1_t                          SigCRCGreen1_f;
    };

    /* 3.6.20.22 SigCRCBlue1 */
    union {
        __IO  uint32_t                              SigCRCBlue1;
        Sig_SigCRCBlue1_t                           SigCRCBlue1_f;
    };

    /* 3.6.20.77 ShadowLoad */
    union {
        __IO  uint32_t                              ShadowLoad;
        Sig_ShadowLoad_t                            ShadowLoad_f;
    };

    /* 3.6.20.78 ContinuousMode */
    union {
        __IO  uint32_t                              ContinuousMode;
        Sig_ContinuousMode_t                        ContinuousMode_f;
    };

    /* 3.6.20.79 SoftwareKick */
    union {
        __IO  uint32_t                              SoftwareKick;
        Sig_SoftwareKick_t                          SoftwareKick_f;
    };

    /* 3.6.20.80 Status */
    union {
        __IO  uint32_t                              Status;
        Sig_Status_t                                Status_f;
    };

} IRIS_MDL2_SigL0_TypeDef;
/******************************************************************************
 * Peripheral memory map
 ******************************************************************************/

#ifndef FM4_PERIPH_BASE2
#define FM4_PERIPH_BASE2        (0xD0000000UL)
#endif

#define IRIS_SDL2_BASE          (FM4_PERIPH_BASE2 + 0x00A00000UL)  /* Iris subsystem control. */
#define IRIS_MDL2_BASE          (FM4_PERIPH_BASE2 + 0x00A10000UL)  /* Iris-MDL2 configuration. (Blit, Display, CmdSeq) (256KB) */

/******************************************************************************
 * Peripheral declaration
 ******************************************************************************/
#define IRIS_SDL2_SubSysCtrl    ((IRIS_SDL2_SubSysCtrl_TypeDef      *) (IRIS_SDL2_BASE + 0x0000))

#define IRIS_MDL2_ComCtrl       ((IRIS_MDL2_ComCtrl_TypeDef         *) (IRIS_MDL2_BASE + 0x0000))
#define IRIS_MDL2_CmdSeq        ((IRIS_MDL2_CmdSeq_TypeDef          *) (IRIS_MDL2_BASE + 0x0400))
#define IRIS_MDL2_PixEngCfg     ((IRIS_MDL2_PixEngCfg_TypeDef       *) (IRIS_MDL2_BASE + 0x0800))
#define IRIS_MDL2_FetchDecodeL9 ((IRIS_MDL2_FetchDecodeL9_TypeDef   *) (IRIS_MDL2_BASE + 0x0c00))
#define IRIS_MDL2_FetchRotL9    ((IRIS_MDL2_FetchRotL9_TypeDef      *) (IRIS_MDL2_BASE + 0x1400))
#define IRIS_MDL2_FetchEcoL9    ((IRIS_MDL2_FetchEcoL9_TypeDef      *) (IRIS_MDL2_BASE + 0x1800))
#define IRIS_MDL2_ROp9          ((IRIS_MDL2_ROp9_TypeDef            *) (IRIS_MDL2_BASE + 0x1c00))
#define IRIS_MDL2_CLuTL9        ((IRIS_MDL2_CLuTL9_TypeDef          *) (IRIS_MDL2_BASE + 0x2000))
#define IRIS_MDL2_MatrixL9      ((IRIS_MDL2_MatrixL9_TypeDef        *) (IRIS_MDL2_BASE + 0x2800))
#define IRIS_MDL2_BlitBlendL9   ((IRIS_MDL2_BlitBlendL9_TypeDef     *) (IRIS_MDL2_BASE + 0x2c00))
#define IRIS_MDL2_StoreL9       ((IRIS_MDL2_StoreL9_TypeDef         *) (IRIS_MDL2_BASE + 0x3000))
#define IRIS_MDL2_ConstFrameL0  ((IRIS_MDL2_ConstFrameL0_TypeDef    *) (IRIS_MDL2_BASE + 0x3400))
#define IRIS_MDL2_ExtDstL0      ((IRIS_MDL2_ExtDstL0_TypeDef        *) (IRIS_MDL2_BASE + 0x3800))
#define IRIS_MDL2_ConstFrameL4  ((IRIS_MDL2_ConstFrameL4_TypeDef    *) (IRIS_MDL2_BASE + 0x3c00))
#define IRIS_MDL2_ExtDstL4      ((IRIS_MDL2_ExtDstL4_TypeDef        *) (IRIS_MDL2_BASE + 0x4000))
#define IRIS_MDL2_FetchDecodeL0 ((IRIS_MDL2_FetchDecodeL0_TypeDef   *) (IRIS_MDL2_BASE + 0x4400))
#define IRIS_MDL2_FetchLayerL0  ((IRIS_MDL2_FetchLayerL0_TypeDef    *) (IRIS_MDL2_BASE + 0x4c00))
#define IRIS_MDL2_LayerBlendL1  ((IRIS_MDL2_LayerBlendL1_TypeDef    *) (IRIS_MDL2_BASE + 0x5400))
#define IRIS_MDL2_LayerBlendL2  ((IRIS_MDL2_LayerBlendL2_TypeDef    *) (IRIS_MDL2_BASE + 0x5800))
#define IRIS_MDL2_ExtSrcL8      ((IRIS_MDL2_ExtSrcL8_TypeDef        *) (IRIS_MDL2_BASE + 0x5c00))
#define IRIS_MDL2_DisEngCfg     ((IRIS_MDL2_DisEngCfg_TypeDef       *) (IRIS_MDL2_BASE + 0x6000))
#define IRIS_MDL2_FrameGenL0    ((IRIS_MDL2_FrameGenL0_TypeDef      *) (IRIS_MDL2_BASE + 0x6400))
#define IRIS_MDL2_GammaCor0     ((IRIS_MDL2_GammaCor0_TypeDef       *) (IRIS_MDL2_BASE + 0x6800))
#define IRIS_MDL2_Dither0       ((IRIS_MDL2_Dither0_TypeDef         *) (IRIS_MDL2_BASE + 0x6c00))
#define IRIS_MDL2_TCon0         ((IRIS_MDL2_TCon0_TypeDef           *) (IRIS_MDL2_BASE + 0x7000))
#define IRIS_MDL2_SigL0         ((IRIS_MDL2_SigL0_TypeDef           *) (IRIS_MDL2_BASE + 0x7800))


#ifdef __cplusplus
}
#endif

#endif /* _IRIS_H_ */
