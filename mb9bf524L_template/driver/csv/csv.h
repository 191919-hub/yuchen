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
/** \file csv.h
 **
 ** Headerfile for CSV functions
 **  
 **
 ** History:
 **   - 2014-11-18  1.0  EZh  First version for FM universal PDL.
 **
 ******************************************************************************/
 
#ifndef _CSV_H_
#define _CSV_H_

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "mcu.h"
#include "pdl_user.h"

#if (defined(PDL_PERIPHERAL_CSV_ACTIVE))

/**
 ******************************************************************************
 ** \defgroup CsvGroup Clock Supervisor Functions (CSV)
 **
 ** Provided functions of CSV module:
 ** 
 ** - Csv_EnableMainCsv()
 ** - Csv_DisableMainCsv()
 ** - Csv_EnableSubCsv()
 ** - Csv_DisableSubCsv()
 ** - Csv_GetCsvFailCause()  
 ** - Csv_EnableFcs() 
 ** - Csv_DisableFcs() 
 ** - Csv_EnableFcsReset() 
 ** - Csv_DisableFcsReset() 
 ** - Csv_EnableFcsIrq() 
 ** - Csv_DisableFcsIrq() 
 ** - Csv_ClrFcsIrqFlag() 
 ** - Csv_GetFcsIrqFlag() 
 ** - Csv_SetFcsCrDiv()
 ** - Csv_SetFcsDetectRange()
 ** - Csv_GetFcsDetectCount()  
 **
 ** <b>What is CSV?</b>
 **
 ** The CSV module includes CSV and FCS function. CSV uses the high and low
 ** CR to monitor main and sub clock, if the abnormal state is detected, a 
 ** reset occurs. FCS uses high-speed CR to monitor the main clock, a range 
 ** can be set in advance. If the monitor cycle exeeds the range, either 
 ** interrupt or reset will occurs.
 **  
 ** <b>How to use CSV module with the APIs provided?</b>
 **
 ** First, Enable the CSV function with Csv_EnableMainCsv() or Csv_EnableSubCsv()
 **
 ** When the abnormal status is detected, a CSV reset occurs, then read the 
 ** the CSV failure cause by Csv_GetCsvFailCause().
 **
 ** Disable the CSV function with Csv_DisableMainCsv() or Csv_DisableSubCsv()
 **  
 ** <b>How to use FCS module with the APIs provided?</b>
 **
 ** First, set the CR dividor with Csv_SetFcsCrDiv() and set the expected range
 ** of main clok cycle with Csv_SetFcsDetectRange().
 **
 ** Second, enable the FCS interrupt with Csv_EnableFcsInt() or enable FCS
 ** reset with Csv_EnableFcsReset(). 
 **
 ** Then start FCS function with Csv_EnableFcs().
 **
 ** When abnormal frequency is detected, an interrupt occurs when FCS interrupt
 ** is enabled, then read the interrupt flag by Csv_GetFcsIrqFlag() and 
 ** clear the interrupt flag by Csv_ClrFcsIrqFlag().
 **
 ** When abnormal frequency is detected, a reset issues when FCS reset is
 ** enabled.
 **
 ** When abnormal frequency is detected, current main clock cycle can be read
 ** by Csv_GetFcsDetectCount().
 **
 ** Disable FCS by Csv_DisableFcs(), disable FCS reset by Csv_DisableFcsReset()
 ** and disable FCS interrupt by Csv_DisableFcsInt().
 **
 ******************************************************************************/
//@{
/******************************************************************************
 * Global type definitions
 ******************************************************************************/
/**
 ******************************************************************************
 ** \brief Enumeration to define an index for each BT instance
 ******************************************************************************/     
typedef enum en_fcs_cr_div
{
    FcsCrDiv256 = 5u,        ///< 1/256 frequency of high-speed CR oscillation   
    FcsCrDiv512 = 6u,        ///< 1/512 frequency of high-speed CR oscillation   
    FcsCrDiv1024 = 7u,       ///< 1/1024 frequency of high-speed CR oscillation
    
}en_fcs_cr_div_t;

/**
 ******************************************************************************
 ** \brief Structure of CSV status
 ******************************************************************************/
typedef struct stc_csv_status
{
    boolean_t bCsvMainClockStatus;
    boolean_t bCsvSubClockStatus;
    
}stc_csv_status_t;

/**
 ******************************************************************************
 ** \brief FCS interrupt callback function type
 ******************************************************************************/
typedef void fn_fcs_int_callback(void);

/******************************************************************************/
/* Global function prototypes (definition in C source)                        */
/******************************************************************************/
/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C" {
#endif
/* CSV */  
void Csv_EnableMainCsv(void);
void Csv_DisableMainCsv(void);
void Csv_EnableSubCsv(void);
void Csv_DisableSubCsv(void);
uint8_t Csv_GetCsvFailCause(stc_csv_status_t* pstcCsvStatus);
/* FCS */  
void Csv_EnableFcs(void);
void Csv_DisableFcs(void);
void Csv_EnableFcsReset(void);
void Csv_DisableFcsReset(void);
#if (PDL_INTERRUPT_ENABLE_CSV == PDL_ON) 
en_result_t Csv_EnableFcsIrq(fn_fcs_int_callback* pfnIrqCb);
void Csv_DisableFcsIrq(void);
#endif
void Csv_ClrFcsIrqFlag(void);
uint8_t Csv_GetFcsIrqFlag(void);
en_result_t Csv_SetFcsCrDiv(en_fcs_cr_div_t enDiv) ;
void Csv_SetFcsDetectRange(uint16_t u16LowerVal, uint16_t u16UpperVal);
uint16_t Csv_GetFcsDetectCount(void);
/* IRQ */
void Csv_IrqHandler(void);
#ifdef __cplusplus
}
#endif

//@}

#endif

#endif /* _CLOCK_FM3_H_ */
/*****************************************************************************/
/* END OF FILE */
