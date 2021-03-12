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
/************************************************************************************/
/** \file gpio_mb9bf16xk.h
 **
 ** Headerfile for MB9BF16XK GPIO functions, included in gpio.h
 **
 ** History:
 **   - 2014-03-25  1.0  MWi    First version.
 **   - 2014-10-21  1.1  MWi    Port extension renamed from _Pxy to _Py
 **   - 2014-12-25  1.2  EZh    Correct AN pin definition
 **
 ******************************************************************************/

#ifndef __GPIO_MB9BF16XK_H__
#define __GPIO_MB9BF16XK_H__

#include <stdint.h>

/******************************************************************************
   GPIO
*******************************************************************************/

/*---- GPIO bit P00 ----*/

#define GPIO1PIN_P00_GET             ( bFM_GPIO_PDIR0_P0 )

#define GPIO1PIN_P00_PUT(v)          ( bFM_GPIO_PDOR0_P0=(v) )

#define GPIO1PIN_P00_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P00_INITIN(v) \
                                                        : GPIO1PIN_P00_INITOUT(v) )

#define GPIO1PIN_P00_INITIN(v)       do{ bFM_GPIO_PCR0_P0=(v).bPullup; \
                                         bFM_GPIO_DDR0_P0=0u; \
                                         bFM_GPIO_PFR0_P0=0u; }while(0u)

#define GPIO1PIN_P00_INITOUT(v)      do{ bFM_GPIO_PDOR0_P0=(v).bInitVal; \
                                         bFM_GPIO_DDR0_P0=1u; \
                                         bFM_GPIO_PFR0_P0=0u; }while(0u)


/*---- GPIO bit NP00 ----*/
#define GPIO1PIN_NP00_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR0_P0)) )

#define GPIO1PIN_NP00_PUT(v)         ( bFM_GPIO_PDOR0_P0=(uint32_t)(!(v)) )

#define GPIO1PIN_NP00_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP00_INITIN(v) \
                                                        : GPIO1PIN_NP00_INITOUT(v) )

#define GPIO1PIN_NP00_INITIN(v)      do{ bFM_GPIO_PCR0_P0=(v).bPullup; \
                                         bFM_GPIO_DDR0_P0=0u; \
                                         bFM_GPIO_PFR0_P0=0u; }while(0u)

#define GPIO1PIN_NP00_INITOUT(v)     do{ bFM_GPIO_PDOR0_P0=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR0_P0=1u; \
                                         bFM_GPIO_PFR0_P0=0u; }while(0u)


/*---- GPIO bit P01 ----*/

#define GPIO1PIN_P01_GET             ( bFM_GPIO_PDIR0_P1 )

#define GPIO1PIN_P01_PUT(v)          ( bFM_GPIO_PDOR0_P1=(v) )

#define GPIO1PIN_P01_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P01_INITIN(v) \
                                                        : GPIO1PIN_P01_INITOUT(v) )

#define GPIO1PIN_P01_INITIN(v)       do{ bFM_GPIO_PCR0_P1=(v).bPullup; \
                                         bFM_GPIO_DDR0_P1=0u; \
                                         bFM_GPIO_PFR0_P1=0u; }while(0u)

#define GPIO1PIN_P01_INITOUT(v)      do{ bFM_GPIO_PDOR0_P1=(v).bInitVal; \
                                         bFM_GPIO_DDR0_P1=1u; \
                                         bFM_GPIO_PFR0_P1=0u; }while(0u)


/*---- GPIO bit NP01 ----*/
#define GPIO1PIN_NP01_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR0_P1)) )

#define GPIO1PIN_NP01_PUT(v)         ( bFM_GPIO_PDOR0_P1=(uint32_t)(!(v)) )

#define GPIO1PIN_NP01_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP01_INITIN(v) \
                                                        : GPIO1PIN_NP01_INITOUT(v) )

#define GPIO1PIN_NP01_INITIN(v)      do{ bFM_GPIO_PCR0_P1=(v).bPullup; \
                                         bFM_GPIO_DDR0_P1=0u; \
                                         bFM_GPIO_PFR0_P1=0u; }while(0u)

#define GPIO1PIN_NP01_INITOUT(v)     do{ bFM_GPIO_PDOR0_P1=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR0_P1=1u; \
                                         bFM_GPIO_PFR0_P1=0u; }while(0u)


/*---- GPIO bit P02 ----*/

#define GPIO1PIN_P02_GET             ( bFM_GPIO_PDIR0_P2 )

#define GPIO1PIN_P02_PUT(v)          ( bFM_GPIO_PDOR0_P2=(v) )

#define GPIO1PIN_P02_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P02_INITIN(v) \
                                                        : GPIO1PIN_P02_INITOUT(v) )

#define GPIO1PIN_P02_INITIN(v)       do{ bFM_GPIO_PCR0_P2=(v).bPullup; \
                                         bFM_GPIO_DDR0_P2=0u; \
                                         bFM_GPIO_PFR0_P2=0u; }while(0u)

#define GPIO1PIN_P02_INITOUT(v)      do{ bFM_GPIO_PDOR0_P2=(v).bInitVal; \
                                         bFM_GPIO_DDR0_P2=1u; \
                                         bFM_GPIO_PFR0_P2=0u; }while(0u)


/*---- GPIO bit NP02 ----*/
#define GPIO1PIN_NP02_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR0_P2)) )

#define GPIO1PIN_NP02_PUT(v)         ( bFM_GPIO_PDOR0_P2=(uint32_t)(!(v)) )

#define GPIO1PIN_NP02_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP02_INITIN(v) \
                                                        : GPIO1PIN_NP02_INITOUT(v) )

#define GPIO1PIN_NP02_INITIN(v)      do{ bFM_GPIO_PCR0_P2=(v).bPullup; \
                                         bFM_GPIO_DDR0_P2=0u; \
                                         bFM_GPIO_PFR0_P2=0u; }while(0u)

#define GPIO1PIN_NP02_INITOUT(v)     do{ bFM_GPIO_PDOR0_P2=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR0_P2=1u; \
                                         bFM_GPIO_PFR0_P2=0u; }while(0u)


/*---- GPIO bit P03 ----*/

#define GPIO1PIN_P03_GET             ( bFM_GPIO_PDIR0_P3 )

#define GPIO1PIN_P03_PUT(v)          ( bFM_GPIO_PDOR0_P3=(v) )

#define GPIO1PIN_P03_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P03_INITIN(v) \
                                                        : GPIO1PIN_P03_INITOUT(v) )

#define GPIO1PIN_P03_INITIN(v)       do{ bFM_GPIO_PCR0_P3=(v).bPullup; \
                                         bFM_GPIO_DDR0_P3=0u; \
                                         bFM_GPIO_PFR0_P3=0u; }while(0u)

#define GPIO1PIN_P03_INITOUT(v)      do{ bFM_GPIO_PDOR0_P3=(v).bInitVal; \
                                         bFM_GPIO_DDR0_P3=1u; \
                                         bFM_GPIO_PFR0_P3=0u; }while(0u)


/*---- GPIO bit NP03 ----*/
#define GPIO1PIN_NP03_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR0_P3)) )

#define GPIO1PIN_NP03_PUT(v)         ( bFM_GPIO_PDOR0_P3=(uint32_t)(!(v)) )

#define GPIO1PIN_NP03_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP03_INITIN(v) \
                                                        : GPIO1PIN_NP03_INITOUT(v) )

#define GPIO1PIN_NP03_INITIN(v)      do{ bFM_GPIO_PCR0_P3=(v).bPullup; \
                                         bFM_GPIO_DDR0_P3=0u; \
                                         bFM_GPIO_PFR0_P3=0u; }while(0u)

#define GPIO1PIN_NP03_INITOUT(v)     do{ bFM_GPIO_PDOR0_P3=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR0_P3=1u; \
                                         bFM_GPIO_PFR0_P3=0u; }while(0u)


/*---- GPIO bit P04 ----*/

#define GPIO1PIN_P04_GET             ( bFM_GPIO_PDIR0_P4 )

#define GPIO1PIN_P04_PUT(v)          ( bFM_GPIO_PDOR0_P4=(v) )

#define GPIO1PIN_P04_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P04_INITIN(v) \
                                                        : GPIO1PIN_P04_INITOUT(v) )

#define GPIO1PIN_P04_INITIN(v)       do{ bFM_GPIO_PCR0_P4=(v).bPullup; \
                                         bFM_GPIO_DDR0_P4=0u; \
                                         bFM_GPIO_PFR0_P4=0u; }while(0u)

#define GPIO1PIN_P04_INITOUT(v)      do{ bFM_GPIO_PDOR0_P4=(v).bInitVal; \
                                         bFM_GPIO_DDR0_P4=1u; \
                                         bFM_GPIO_PFR0_P4=0u; }while(0u)


/*---- GPIO bit NP04 ----*/
#define GPIO1PIN_NP04_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR0_P4)) )

#define GPIO1PIN_NP04_PUT(v)         ( bFM_GPIO_PDOR0_P4=(uint32_t)(!(v)) )

#define GPIO1PIN_NP04_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP04_INITIN(v) \
                                                        : GPIO1PIN_NP04_INITOUT(v) )

#define GPIO1PIN_NP04_INITIN(v)      do{ bFM_GPIO_PCR0_P4=(v).bPullup; \
                                         bFM_GPIO_DDR0_P4=0u; \
                                         bFM_GPIO_PFR0_P4=0u; }while(0u)

#define GPIO1PIN_NP04_INITOUT(v)     do{ bFM_GPIO_PDOR0_P4=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR0_P4=1u; \
                                         bFM_GPIO_PFR0_P4=0u; }while(0u)


/*---- GPIO bit P10 ----*/

#define GPIO1PIN_P10_GET             ( bFM_GPIO_PDIR1_P0 )

#define GPIO1PIN_P10_PUT(v)          ( bFM_GPIO_PDOR1_P0=(v) )

#define GPIO1PIN_P10_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P10_INITIN(v) \
                                                        : GPIO1PIN_P10_INITOUT(v) )

#define GPIO1PIN_P10_INITIN(v)       do{ bFM_GPIO_PCR1_P0=(v).bPullup; \
                                         bFM_GPIO_DDR1_P0=0u; \
                                         bFM_GPIO_PFR1_P0=0u; }while(0u)

#define GPIO1PIN_P10_INITOUT(v)      do{ bFM_GPIO_PDOR1_P0=(v).bInitVal; \
                                         bFM_GPIO_DDR1_P0=1u; \
                                         bFM_GPIO_PFR1_P0=0u; }while(0u)


/*---- GPIO bit NP10 ----*/
#define GPIO1PIN_NP10_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR1_P0)) )

#define GPIO1PIN_NP10_PUT(v)         ( bFM_GPIO_PDOR1_P0=(uint32_t)(!(v)) )

#define GPIO1PIN_NP10_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP10_INITIN(v) \
                                                        : GPIO1PIN_NP10_INITOUT(v) )

#define GPIO1PIN_NP10_INITIN(v)      do{ bFM_GPIO_PCR1_P0=(v).bPullup; \
                                         bFM_GPIO_DDR1_P0=0u; \
                                         bFM_GPIO_PFR1_P0=0u; }while(0u)

#define GPIO1PIN_NP10_INITOUT(v)     do{ bFM_GPIO_PDOR1_P0=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR1_P0=1u; \
                                         bFM_GPIO_PFR1_P0=0u; }while(0u)


/*---- GPIO bit P11 ----*/

#define GPIO1PIN_P11_GET             ( bFM_GPIO_PDIR1_P1 )

#define GPIO1PIN_P11_PUT(v)          ( bFM_GPIO_PDOR1_P1=(v) )

#define GPIO1PIN_P11_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P11_INITIN(v) \
                                                        : GPIO1PIN_P11_INITOUT(v) )

#define GPIO1PIN_P11_INITIN(v)       do{ bFM_GPIO_PCR1_P1=(v).bPullup; \
                                         bFM_GPIO_DDR1_P1=0u; \
                                         bFM_GPIO_PFR1_P1=0u; }while(0u)

#define GPIO1PIN_P11_INITOUT(v)      do{ bFM_GPIO_PDOR1_P1=(v).bInitVal; \
                                         bFM_GPIO_DDR1_P1=1u; \
                                         bFM_GPIO_PFR1_P1=0u; }while(0u)


/*---- GPIO bit NP11 ----*/
#define GPIO1PIN_NP11_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR1_P1)) )

#define GPIO1PIN_NP11_PUT(v)         ( bFM_GPIO_PDOR1_P1=(uint32_t)(!(v)) )

#define GPIO1PIN_NP11_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP11_INITIN(v) \
                                                        : GPIO1PIN_NP11_INITOUT(v) )

#define GPIO1PIN_NP11_INITIN(v)      do{ bFM_GPIO_PCR1_P1=(v).bPullup; \
                                         bFM_GPIO_DDR1_P1=0u; \
                                         bFM_GPIO_PFR1_P1=0u; }while(0u)

#define GPIO1PIN_NP11_INITOUT(v)     do{ bFM_GPIO_PDOR1_P1=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR1_P1=1u; \
                                         bFM_GPIO_PFR1_P1=0u; }while(0u)


/*---- GPIO bit P12 ----*/

#define GPIO1PIN_P12_GET             ( bFM_GPIO_PDIR1_P2 )

#define GPIO1PIN_P12_PUT(v)          ( bFM_GPIO_PDOR1_P2=(v) )

#define GPIO1PIN_P12_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P12_INITIN(v) \
                                                        : GPIO1PIN_P12_INITOUT(v) )

#define GPIO1PIN_P12_INITIN(v)       do{ bFM_GPIO_PCR1_P2=(v).bPullup; \
                                         bFM_GPIO_DDR1_P2=0u; \
                                         bFM_GPIO_PFR1_P2=0u; \
                                         bFM_GPIO_ADE_AN00=0u; }while(0u)


#define GPIO1PIN_P12_INITOUT(v)      do{ bFM_GPIO_PDOR1_P2=(v).bInitVal; \
                                         bFM_GPIO_DDR1_P2=1u; \
                                         bFM_GPIO_PFR1_P2=0u; \
                                         bFM_GPIO_ADE_AN00=0u; }while(0u)


/*---- GPIO bit NP12 ----*/
#define GPIO1PIN_NP12_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR1_P2)) )

#define GPIO1PIN_NP12_PUT(v)         ( bFM_GPIO_PDOR1_P2=(uint32_t)(!(v)) )

#define GPIO1PIN_NP12_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP12_INITIN(v) \
                                                        : GPIO1PIN_NP12_INITOUT(v) )

#define GPIO1PIN_NP12_INITIN(v)      do{ bFM_GPIO_PCR1_P2=(v).bPullup; \
                                         bFM_GPIO_DDR1_P2=0u; \
                                         bFM_GPIO_PFR1_P2=0u; \
                                         bFM_GPIO_ADE_AN00=0u; }while(0u)


#define GPIO1PIN_NP12_INITOUT(v)     do{ bFM_GPIO_PDOR1_P2=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR1_P2=1u; \
                                         bFM_GPIO_PFR1_P2=0u; \
                                         bFM_GPIO_ADE_AN00=0u; }while(0u)


/*---- GPIO bit P13 ----*/

#define GPIO1PIN_P13_GET             ( bFM_GPIO_PDIR1_P3 )

#define GPIO1PIN_P13_PUT(v)          ( bFM_GPIO_PDOR1_P3=(v) )

#define GPIO1PIN_P13_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P13_INITIN(v) \
                                                        : GPIO1PIN_P13_INITOUT(v) )

#define GPIO1PIN_P13_INITIN(v)       do{ bFM_GPIO_PCR1_P3=(v).bPullup; \
                                         bFM_GPIO_DDR1_P3=0u; \
                                         bFM_GPIO_PFR1_P3=0u; \
                                         bFM_GPIO_ADE_AN01=0u; }while(0u)


#define GPIO1PIN_P13_INITOUT(v)      do{ bFM_GPIO_PDOR1_P3=(v).bInitVal; \
                                         bFM_GPIO_DDR1_P3=1u; \
                                         bFM_GPIO_PFR1_P3=0u; \
                                         bFM_GPIO_ADE_AN01=0u; }while(0u)


/*---- GPIO bit NP13 ----*/
#define GPIO1PIN_NP13_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR1_P3)) )

#define GPIO1PIN_NP13_PUT(v)         ( bFM_GPIO_PDOR1_P3=(uint32_t)(!(v)) )

#define GPIO1PIN_NP13_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP13_INITIN(v) \
                                                        : GPIO1PIN_NP13_INITOUT(v) )

#define GPIO1PIN_NP13_INITIN(v)      do{ bFM_GPIO_PCR1_P3=(v).bPullup; \
                                         bFM_GPIO_DDR1_P3=0u; \
                                         bFM_GPIO_PFR1_P3=0u; \
                                         bFM_GPIO_ADE_AN01=0u; }while(0u)


#define GPIO1PIN_NP13_INITOUT(v)     do{ bFM_GPIO_PDOR1_P3=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR1_P3=1u; \
                                         bFM_GPIO_PFR1_P3=0u; \
                                         bFM_GPIO_ADE_AN01=0u; }while(0u)


/*---- GPIO bit P20 ----*/

#define GPIO1PIN_P20_GET             ( bFM_GPIO_PDIR2_P0 )

#define GPIO1PIN_P20_PUT(v)          ( bFM_GPIO_PDOR2_P0=(v) )

#define GPIO1PIN_P20_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P20_INITIN(v) \
                                                        : GPIO1PIN_P20_INITOUT(v) )

#define GPIO1PIN_P20_INITIN(v)       do{ bFM_GPIO_PCR2_P0=(v).bPullup; \
                                         bFM_GPIO_DDR2_P0=0u; \
                                         bFM_GPIO_PFR2_P0=0u; \
                                         bFM_GPIO_ADE_AN02=0u; }while(0u)


#define GPIO1PIN_P20_INITOUT(v)      do{ bFM_GPIO_PDOR2_P0=(v).bInitVal; \
                                         bFM_GPIO_DDR2_P0=1u; \
                                         bFM_GPIO_PFR2_P0=0u; \
                                         bFM_GPIO_ADE_AN02=0u; }while(0u)


/*---- GPIO bit NP20 ----*/
#define GPIO1PIN_NP20_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR2_P0)) )

#define GPIO1PIN_NP20_PUT(v)         ( bFM_GPIO_PDOR2_P0=(uint32_t)(!(v)) )

#define GPIO1PIN_NP20_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP20_INITIN(v) \
                                                        : GPIO1PIN_NP20_INITOUT(v) )

#define GPIO1PIN_NP20_INITIN(v)      do{ bFM_GPIO_PCR2_P0=(v).bPullup; \
                                         bFM_GPIO_DDR2_P0=0u; \
                                         bFM_GPIO_PFR2_P0=0u; \
                                         bFM_GPIO_ADE_AN02=0u; }while(0u)


#define GPIO1PIN_NP20_INITOUT(v)     do{ bFM_GPIO_PDOR2_P0=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR2_P0=1u; \
                                         bFM_GPIO_PFR2_P0=0u; \
                                         bFM_GPIO_ADE_AN02=0u; }while(0u)


/*---- GPIO bit P21 ----*/

#define GPIO1PIN_P21_GET             ( bFM_GPIO_PDIR2_P1 )

#define GPIO1PIN_P21_PUT(v)          ( bFM_GPIO_PDOR2_P1=(v) )

#define GPIO1PIN_P21_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P21_INITIN(v) \
                                                        : GPIO1PIN_P21_INITOUT(v) )

#define GPIO1PIN_P21_INITIN(v)       do{ bFM_GPIO_PCR2_P1=(v).bPullup; \
                                         bFM_GPIO_DDR2_P1=0u; \
                                         bFM_GPIO_PFR2_P1=0u; \
                                         bFM_GPIO_ADE_AN03=0u; }while(0u)


#define GPIO1PIN_P21_INITOUT(v)      do{ bFM_GPIO_PDOR2_P1=(v).bInitVal; \
                                         bFM_GPIO_DDR2_P1=1u; \
                                         bFM_GPIO_PFR2_P1=0u; \
                                         bFM_GPIO_ADE_AN03=0u; }while(0u)


/*---- GPIO bit NP21 ----*/
#define GPIO1PIN_NP21_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR2_P1)) )

#define GPIO1PIN_NP21_PUT(v)         ( bFM_GPIO_PDOR2_P1=(uint32_t)(!(v)) )

#define GPIO1PIN_NP21_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP21_INITIN(v) \
                                                        : GPIO1PIN_NP21_INITOUT(v) )

#define GPIO1PIN_NP21_INITIN(v)      do{ bFM_GPIO_PCR2_P1=(v).bPullup; \
                                         bFM_GPIO_DDR2_P1=0u; \
                                         bFM_GPIO_PFR2_P1=0u; \
                                         bFM_GPIO_ADE_AN03=0u; }while(0u)


#define GPIO1PIN_NP21_INITOUT(v)     do{ bFM_GPIO_PDOR2_P1=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR2_P1=1u; \
                                         bFM_GPIO_PFR2_P1=0u; \
                                         bFM_GPIO_ADE_AN03=0u; }while(0u)


/*---- GPIO bit P22 ----*/

#define GPIO1PIN_P22_GET             ( bFM_GPIO_PDIR2_P2 )

#define GPIO1PIN_P22_PUT(v)          ( bFM_GPIO_PDOR2_P2=(v) )

#define GPIO1PIN_P22_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P22_INITIN(v) \
                                                        : GPIO1PIN_P22_INITOUT(v) )

#define GPIO1PIN_P22_INITIN(v)       do{ bFM_GPIO_PCR2_P2=(v).bPullup; \
                                         bFM_GPIO_DDR2_P2=0u; \
                                         bFM_GPIO_PFR2_P2=0u; \
                                         bFM_GPIO_ADE_AN04=0u; }while(0u)


#define GPIO1PIN_P22_INITOUT(v)      do{ bFM_GPIO_PDOR2_P2=(v).bInitVal; \
                                         bFM_GPIO_DDR2_P2=1u; \
                                         bFM_GPIO_PFR2_P2=0u; \
                                         bFM_GPIO_ADE_AN04=0u; }while(0u)


/*---- GPIO bit NP22 ----*/
#define GPIO1PIN_NP22_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR2_P2)) )

#define GPIO1PIN_NP22_PUT(v)         ( bFM_GPIO_PDOR2_P2=(uint32_t)(!(v)) )

#define GPIO1PIN_NP22_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP22_INITIN(v) \
                                                        : GPIO1PIN_NP22_INITOUT(v) )

#define GPIO1PIN_NP22_INITIN(v)      do{ bFM_GPIO_PCR2_P2=(v).bPullup; \
                                         bFM_GPIO_DDR2_P2=0u; \
                                         bFM_GPIO_PFR2_P2=0u; \
                                         bFM_GPIO_ADE_AN04=0u; }while(0u)


#define GPIO1PIN_NP22_INITOUT(v)     do{ bFM_GPIO_PDOR2_P2=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR2_P2=1u; \
                                         bFM_GPIO_PFR2_P2=0u; \
                                         bFM_GPIO_ADE_AN04=0u; }while(0u)


/*---- GPIO bit P23 ----*/

#define GPIO1PIN_P23_GET             ( bFM_GPIO_PDIR2_P3 )

#define GPIO1PIN_P23_PUT(v)          ( bFM_GPIO_PDOR2_P3=(v) )

#define GPIO1PIN_P23_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P23_INITIN(v) \
                                                        : GPIO1PIN_P23_INITOUT(v) )

#define GPIO1PIN_P23_INITIN(v)       do{ bFM_GPIO_PCR2_P3=(v).bPullup; \
                                         bFM_GPIO_DDR2_P3=0u; \
                                         bFM_GPIO_PFR2_P3=0u; \
                                         bFM_GPIO_ADE_AN05=0u; }while(0u)


#define GPIO1PIN_P23_INITOUT(v)      do{ bFM_GPIO_PDOR2_P3=(v).bInitVal; \
                                         bFM_GPIO_DDR2_P3=1u; \
                                         bFM_GPIO_PFR2_P3=0u; \
                                         bFM_GPIO_ADE_AN05=0u; }while(0u)


/*---- GPIO bit NP23 ----*/
#define GPIO1PIN_NP23_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR2_P3)) )

#define GPIO1PIN_NP23_PUT(v)         ( bFM_GPIO_PDOR2_P3=(uint32_t)(!(v)) )

#define GPIO1PIN_NP23_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP23_INITIN(v) \
                                                        : GPIO1PIN_NP23_INITOUT(v) )

#define GPIO1PIN_NP23_INITIN(v)      do{ bFM_GPIO_PCR2_P3=(v).bPullup; \
                                         bFM_GPIO_DDR2_P3=0u; \
                                         bFM_GPIO_PFR2_P3=0u; \
                                         bFM_GPIO_ADE_AN05=0u; }while(0u)


#define GPIO1PIN_NP23_INITOUT(v)     do{ bFM_GPIO_PDOR2_P3=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR2_P3=1u; \
                                         bFM_GPIO_PFR2_P3=0u; \
                                         bFM_GPIO_ADE_AN05=0u; }while(0u)


/*---- GPIO bit P30 ----*/

#define GPIO1PIN_P30_GET             ( bFM_GPIO_PDIR3_P0 )

#define GPIO1PIN_P30_PUT(v)          ( bFM_GPIO_PDOR3_P0=(v) )

#define GPIO1PIN_P30_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P30_INITIN(v) \
                                                        : GPIO1PIN_P30_INITOUT(v) )

#define GPIO1PIN_P30_INITIN(v)       do{ bFM_GPIO_PCR3_P0=(v).bPullup; \
                                         bFM_GPIO_DDR3_P0=0u; \
                                         bFM_GPIO_PFR3_P0=0u; }while(0u)

#define GPIO1PIN_P30_INITOUT(v)      do{ bFM_GPIO_PDOR3_P0=(v).bInitVal; \
                                         bFM_GPIO_DDR3_P0=1u; \
                                         bFM_GPIO_PFR3_P0=0u; }while(0u)


/*---- GPIO bit NP30 ----*/
#define GPIO1PIN_NP30_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR3_P0)) )

#define GPIO1PIN_NP30_PUT(v)         ( bFM_GPIO_PDOR3_P0=(uint32_t)(!(v)) )

#define GPIO1PIN_NP30_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP30_INITIN(v) \
                                                        : GPIO1PIN_NP30_INITOUT(v) )

#define GPIO1PIN_NP30_INITIN(v)      do{ bFM_GPIO_PCR3_P0=(v).bPullup; \
                                         bFM_GPIO_DDR3_P0=0u; \
                                         bFM_GPIO_PFR3_P0=0u; }while(0u)

#define GPIO1PIN_NP30_INITOUT(v)     do{ bFM_GPIO_PDOR3_P0=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR3_P0=1u; \
                                         bFM_GPIO_PFR3_P0=0u; }while(0u)


/*---- GPIO bit P31 ----*/

#define GPIO1PIN_P31_GET             ( bFM_GPIO_PDIR3_P1 )

#define GPIO1PIN_P31_PUT(v)          ( bFM_GPIO_PDOR3_P1=(v) )

#define GPIO1PIN_P31_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P31_INITIN(v) \
                                                        : GPIO1PIN_P31_INITOUT(v) )

#define GPIO1PIN_P31_INITIN(v)       do{ bFM_GPIO_PCR3_P1=(v).bPullup; \
                                         bFM_GPIO_DDR3_P1=0u; \
                                         bFM_GPIO_PFR3_P1=0u; }while(0u)

#define GPIO1PIN_P31_INITOUT(v)      do{ bFM_GPIO_PDOR3_P1=(v).bInitVal; \
                                         bFM_GPIO_DDR3_P1=1u; \
                                         bFM_GPIO_PFR3_P1=0u; }while(0u)


/*---- GPIO bit NP31 ----*/
#define GPIO1PIN_NP31_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR3_P1)) )

#define GPIO1PIN_NP31_PUT(v)         ( bFM_GPIO_PDOR3_P1=(uint32_t)(!(v)) )

#define GPIO1PIN_NP31_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP31_INITIN(v) \
                                                        : GPIO1PIN_NP31_INITOUT(v) )

#define GPIO1PIN_NP31_INITIN(v)      do{ bFM_GPIO_PCR3_P1=(v).bPullup; \
                                         bFM_GPIO_DDR3_P1=0u; \
                                         bFM_GPIO_PFR3_P1=0u; }while(0u)

#define GPIO1PIN_NP31_INITOUT(v)     do{ bFM_GPIO_PDOR3_P1=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR3_P1=1u; \
                                         bFM_GPIO_PFR3_P1=0u; }while(0u)


/*---- GPIO bit P32 ----*/

#define GPIO1PIN_P32_GET             ( bFM_GPIO_PDIR3_P2 )

#define GPIO1PIN_P32_PUT(v)          ( bFM_GPIO_PDOR3_P2=(v) )

#define GPIO1PIN_P32_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P32_INITIN(v) \
                                                        : GPIO1PIN_P32_INITOUT(v) )

#define GPIO1PIN_P32_INITIN(v)       do{ bFM_GPIO_PCR3_P2=(v).bPullup; \
                                         bFM_GPIO_DDR3_P2=0u; \
                                         bFM_GPIO_PFR3_P2=0u; }while(0u)

#define GPIO1PIN_P32_INITOUT(v)      do{ bFM_GPIO_PDOR3_P2=(v).bInitVal; \
                                         bFM_GPIO_DDR3_P2=1u; \
                                         bFM_GPIO_PFR3_P2=0u; }while(0u)


/*---- GPIO bit NP32 ----*/
#define GPIO1PIN_NP32_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR3_P2)) )

#define GPIO1PIN_NP32_PUT(v)         ( bFM_GPIO_PDOR3_P2=(uint32_t)(!(v)) )

#define GPIO1PIN_NP32_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP32_INITIN(v) \
                                                        : GPIO1PIN_NP32_INITOUT(v) )

#define GPIO1PIN_NP32_INITIN(v)      do{ bFM_GPIO_PCR3_P2=(v).bPullup; \
                                         bFM_GPIO_DDR3_P2=0u; \
                                         bFM_GPIO_PFR3_P2=0u; }while(0u)

#define GPIO1PIN_NP32_INITOUT(v)     do{ bFM_GPIO_PDOR3_P2=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR3_P2=1u; \
                                         bFM_GPIO_PFR3_P2=0u; }while(0u)


/*---- GPIO bit P33 ----*/

#define GPIO1PIN_P33_GET             ( bFM_GPIO_PDIR3_P3 )

#define GPIO1PIN_P33_PUT(v)          ( bFM_GPIO_PDOR3_P3=(v) )

#define GPIO1PIN_P33_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P33_INITIN(v) \
                                                        : GPIO1PIN_P33_INITOUT(v) )

#define GPIO1PIN_P33_INITIN(v)       do{ bFM_GPIO_PCR3_P3=(v).bPullup; \
                                         bFM_GPIO_DDR3_P3=0u; \
                                         bFM_GPIO_PFR3_P3=0u; }while(0u)

#define GPIO1PIN_P33_INITOUT(v)      do{ bFM_GPIO_PDOR3_P3=(v).bInitVal; \
                                         bFM_GPIO_DDR3_P3=1u; \
                                         bFM_GPIO_PFR3_P3=0u; }while(0u)


/*---- GPIO bit NP33 ----*/
#define GPIO1PIN_NP33_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR3_P3)) )

#define GPIO1PIN_NP33_PUT(v)         ( bFM_GPIO_PDOR3_P3=(uint32_t)(!(v)) )

#define GPIO1PIN_NP33_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP33_INITIN(v) \
                                                        : GPIO1PIN_NP33_INITOUT(v) )

#define GPIO1PIN_NP33_INITIN(v)      do{ bFM_GPIO_PCR3_P3=(v).bPullup; \
                                         bFM_GPIO_DDR3_P3=0u; \
                                         bFM_GPIO_PFR3_P3=0u; }while(0u)

#define GPIO1PIN_NP33_INITOUT(v)     do{ bFM_GPIO_PDOR3_P3=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR3_P3=1u; \
                                         bFM_GPIO_PFR3_P3=0u; }while(0u)


/*---- GPIO bit P34 ----*/

#define GPIO1PIN_P34_GET             ( bFM_GPIO_PDIR3_P4 )

#define GPIO1PIN_P34_PUT(v)          ( bFM_GPIO_PDOR3_P4=(v) )

#define GPIO1PIN_P34_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P34_INITIN(v) \
                                                        : GPIO1PIN_P34_INITOUT(v) )

#define GPIO1PIN_P34_INITIN(v)       do{ bFM_GPIO_PCR3_P4=(v).bPullup; \
                                         bFM_GPIO_DDR3_P4=0u; \
                                         bFM_GPIO_PFR3_P4=0u; }while(0u)

#define GPIO1PIN_P34_INITOUT(v)      do{ bFM_GPIO_PDOR3_P4=(v).bInitVal; \
                                         bFM_GPIO_DDR3_P4=1u; \
                                         bFM_GPIO_PFR3_P4=0u; }while(0u)


/*---- GPIO bit NP34 ----*/
#define GPIO1PIN_NP34_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR3_P4)) )

#define GPIO1PIN_NP34_PUT(v)         ( bFM_GPIO_PDOR3_P4=(uint32_t)(!(v)) )

#define GPIO1PIN_NP34_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP34_INITIN(v) \
                                                        : GPIO1PIN_NP34_INITOUT(v) )

#define GPIO1PIN_NP34_INITIN(v)      do{ bFM_GPIO_PCR3_P4=(v).bPullup; \
                                         bFM_GPIO_DDR3_P4=0u; \
                                         bFM_GPIO_PFR3_P4=0u; }while(0u)

#define GPIO1PIN_NP34_INITOUT(v)     do{ bFM_GPIO_PDOR3_P4=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR3_P4=1u; \
                                         bFM_GPIO_PFR3_P4=0u; }while(0u)


/*---- GPIO bit P35 ----*/

#define GPIO1PIN_P35_GET             ( bFM_GPIO_PDIR3_P5 )

#define GPIO1PIN_P35_PUT(v)          ( bFM_GPIO_PDOR3_P5=(v) )

#define GPIO1PIN_P35_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P35_INITIN(v) \
                                                        : GPIO1PIN_P35_INITOUT(v) )

#define GPIO1PIN_P35_INITIN(v)       do{ bFM_GPIO_PCR3_P5=(v).bPullup; \
                                         bFM_GPIO_DDR3_P5=0u; \
                                         bFM_GPIO_PFR3_P5=0u; }while(0u)

#define GPIO1PIN_P35_INITOUT(v)      do{ bFM_GPIO_PDOR3_P5=(v).bInitVal; \
                                         bFM_GPIO_DDR3_P5=1u; \
                                         bFM_GPIO_PFR3_P5=0u; }while(0u)


/*---- GPIO bit NP35 ----*/
#define GPIO1PIN_NP35_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR3_P5)) )

#define GPIO1PIN_NP35_PUT(v)         ( bFM_GPIO_PDOR3_P5=(uint32_t)(!(v)) )

#define GPIO1PIN_NP35_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP35_INITIN(v) \
                                                        : GPIO1PIN_NP35_INITOUT(v) )

#define GPIO1PIN_NP35_INITIN(v)      do{ bFM_GPIO_PCR3_P5=(v).bPullup; \
                                         bFM_GPIO_DDR3_P5=0u; \
                                         bFM_GPIO_PFR3_P5=0u; }while(0u)

#define GPIO1PIN_NP35_INITOUT(v)     do{ bFM_GPIO_PDOR3_P5=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR3_P5=1u; \
                                         bFM_GPIO_PFR3_P5=0u; }while(0u)


/*---- GPIO bit P46 ----*/

/******************************************************************************
   Note, This GPIO pin configration macros will access registers in VBAT domain.
   Please take care if the configration does not conflict with your VBAT resources.
   Especially, sub oscillator should not be enabled if P46/P47 is used as VBAT GPIO.
   You may need to change the settings of sub oscillator in system_xxxx.h .
*******************************************************************************/

#define GPIO1PIN_P46_GET             ( bFM_RTC_VBDIR_VDIR3 )

#define GPIO1PIN_P46_PUT(v)          do{ FM_RTC->WTCR20_f.PREAD = 1u; \
                                         while(0u != FM_RTC->WTCR10_f.TRANS){} \
                                         bFM_RTC_VBDOR_VDOR3=(v) \
                                         FM_RTC->WTCR20_f.PWRITE = 1u; \
                                         while(0u != FM_RTC->WTCR10_f.TRANS){} \
                                       }while(0u)

#define GPIO1PIN_P46_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P46_INITIN(v) \
                                                        : GPIO1PIN_P46_INITOUT(v) )

#define GPIO1PIN_P46_INITIN(v)       do{ bFM_RTC_VDET_PON = 0u; \
                                         FM_RTC->WTCR20_f.PREAD = 1u; \
                                         while(0u != FM_RTC->WTCR10_f.TRANS){} \
                                         bFM_RTC_VBPCR_VPCR3=(v).bPullup; \
                                         bFM_RTC_VBDDR_VDDR3=0u; \
                                         PINRELOC_SET_EPFR( FM_RTC->VBPFR, 4u, 2u, 0x00u ); \
                                         bFM_RTC_VBPFR_VPFR3=0u; \
                                         FM_RTC->WTCR20_f.PWRITE = 1u; \
                                         while(0u != FM_RTC->WTCR10_f.TRANS){} \
                                       }while(0u)

#define GPIO1PIN_P46_INITOUT(v)      do{ bFM_RTC_VDET_PON = 0u; \
                                         FM_RTC->WTCR20_f.PREAD = 1u; \
                                         while(0u != FM_RTC->WTCR10_f.TRANS){} \
                                         bFM_RTC_VBDOR_VDOR3=(v).bInitVal; \
                                         bFM_RTC_VBDDR_VDDR3=1u; \
                                         PINRELOC_SET_EPFR( FM_RTC->VBPFR, 4u, 2u, 0x00u ); \
                                         bFM_RTC_VBPFR_VPFR3=0u; \
                                         FM_RTC->WTCR20_f.PWRITE = 1u; \
                                         while(0u != FM_RTC->WTCR10_f.TRANS){} \
                                       }while(0u)

/*---- GPIO bit NP46 ----*/
#define GPIO1PIN_NP46_GET            ( (uint32_t)(!(uint32_t)(bFM_RTC_VBPDIR_VDIR3)) )

#define GPIO1PIN_NP46_PUT(v)         do{ FM_RTC->WTCR20_f.PREAD = 1u; \
                                         while(0u != FM_RTC->WTCR10_f.TRANS){} \
                                         bFM_RTC_VBDOR_VDOR3=(uint32_t)(!(v)) \
                                         FM_RTC->WTCR20_f.PWRITE = 1u; \
                                         while(0u != FM_RTC->WTCR10_f.TRANS){} \
                                       }while(0u)

#define GPIO1PIN_NP46_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP46_INITIN(v) \
                                                        : GPIO1PIN_NP46_INITOUT(v) )

#define GPIO1PIN_NP46_INITIN(v)      do{ bFM_RTC_VDET_PON = 0u; \
                                         FM_RTC->WTCR20_f.PREAD = 1u; \
                                         while(0u != FM_RTC->WTCR10_f.TRANS){} \
                                         bFM_RTC_VBPCR_VPCR3=(v).bPullup; \
                                         bFM_RTC_VBDDR_VDDR3=0u; \
                                         PINRELOC_SET_EPFR( FM_RTC->VBPFR, 4u, 2u, 0x00u ); \
                                         bFM_RTC_VBPFR_VPFR3=0u; \
                                         FM_RTC->WTCR20_f.PWRITE = 1u; \
                                         while(0u != FM_RTC->WTCR10_f.TRANS){} \
                                       }while(0u)

#define GPIO1PIN_NP46_INITOUT(v)     do{ bFM_RTC_VDET_PON = 0u; \
                                         FM_RTC->WTCR20_f.PREAD = 1u; \
                                         while(0u != FM_RTC->WTCR10_f.TRANS){} \
                                         bFM_RTC_VBDOR_VDOR3=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_RTC_VBDDR_VDDR3=1u; \
                                         PINRELOC_SET_EPFR( FM_RTC->VBPFR, 4u, 2u, 0x00u ); \
                                         bFM_RTC_VBPFR_VPFR3=0u; \
                                         FM_RTC->WTCR20_f.PWRITE = 1u; \
                                         while(0u != FM_RTC->WTCR10_f.TRANS){} \
                                       }while(0u)

/*---- GPIO bit P47 ----*/

/******************************************************************************
   Note, This GPIO pin configration macros will access registers in VBAT domain.
   Please take care if the configration does not conflict with your VBAT resources.
   Especially, sub oscillator should not be enabled if P46/P47 is used as VBAT GPIO.
   You may need to change the settings of sub oscillator in system_xxxx.h .
*******************************************************************************/

#define GPIO1PIN_P47_GET             ( bFM_RTC_VBDIR_VDIR2 )

#define GPIO1PIN_P47_PUT(v)          do{ FM_RTC->WTCR20_f.PREAD = 1u; \
                                         while(0u != FM_RTC->WTCR10_f.TRANS){} \
                                         bFM_RTC_VBDOR_VDOR2=(v) \
                                         FM_RTC->WTCR20_f.PWRITE = 1u; \
                                         while(0u != FM_RTC->WTCR10_f.TRANS){} \
                                       }while(0u)

#define GPIO1PIN_P47_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P47_INITIN(v) \
                                                        : GPIO1PIN_P47_INITOUT(v) )

#define GPIO1PIN_P47_INITIN(v)       do{ bFM_RTC_VDET_PON = 0u; \
                                         FM_RTC->WTCR20_f.PREAD = 1u; \
                                         while(0u != FM_RTC->WTCR10_f.TRANS){} \
                                         bFM_RTC_VBPCR_VPCR2=(v).bPullup; \
                                         bFM_RTC_VBDDR_VDDR2=0u; \
                                         PINRELOC_SET_EPFR( FM_RTC->VBPFR, 4u, 2u, 0x00u ); \
                                         bFM_RTC_VBPFR_VPFR2=0u; \
                                         FM_RTC->WTCR20_f.PWRITE = 1u; \
                                         while(0u != FM_RTC->WTCR10_f.TRANS){} \
                                       }while(0u)

#define GPIO1PIN_P47_INITOUT(v)      do{ bFM_RTC_VDET_PON = 0u; \
                                         FM_RTC->WTCR20_f.PREAD = 1u; \
                                         while(0u != FM_RTC->WTCR10_f.TRANS){} \
                                         bFM_RTC_VBDOR_VDOR2=(v).bInitVal; \
                                         bFM_RTC_VBDDR_VDDR2=1u; \
                                         PINRELOC_SET_EPFR( FM_RTC->VBPFR, 4u, 2u, 0x00u ); \
                                         bFM_RTC_VBPFR_VPFR2=0u; \
                                         FM_RTC->WTCR20_f.PWRITE = 1u; \
                                         while(0u != FM_RTC->WTCR10_f.TRANS){} \
                                       }while(0u)

/*---- GPIO bit NP47 ----*/
#define GPIO1PIN_NP47_GET            ( (uint32_t)(!(uint32_t)(bFM_RTC_VBPDIR_VDIR2)) )

#define GPIO1PIN_NP47_PUT(v)         do{ FM_RTC->WTCR20_f.PREAD = 1u; \
                                         while(0u != FM_RTC->WTCR10_f.TRANS){} \
                                         bFM_RTC_VBDOR_VDOR2=(uint32_t)(!(v)) \
                                         FM_RTC->WTCR20_f.PWRITE = 1u; \
                                         while(0u != FM_RTC->WTCR10_f.TRANS){} \
                                       }while(0u)

#define GPIO1PIN_NP47_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP47_INITIN(v) \
                                                        : GPIO1PIN_NP47_INITOUT(v) )

#define GPIO1PIN_NP47_INITIN(v)      do{ bFM_RTC_VDET_PON = 0u; \
                                         FM_RTC->WTCR20_f.PREAD = 1u; \
                                         while(0u != FM_RTC->WTCR10_f.TRANS){} \
                                         bFM_RTC_VBPCR_VPCR2=(v).bPullup; \
                                         bFM_RTC_VBDDR_VDDR2=0u; \
                                         PINRELOC_SET_EPFR( FM_RTC->VBPFR, 4u, 2u, 0x00u ); \
                                         bFM_RTC_VBPFR_VPFR2=0u; \
                                         FM_RTC->WTCR20_f.PWRITE = 1u; \
                                         while(0u != FM_RTC->WTCR10_f.TRANS){} \
                                       }while(0u)

#define GPIO1PIN_NP47_INITOUT(v)     do{ bFM_RTC_VDET_PON = 0u; \
                                         FM_RTC->WTCR20_f.PREAD = 1u; \
                                         while(0u != FM_RTC->WTCR10_f.TRANS){} \
                                         bFM_RTC_VBDOR_VDOR2=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_RTC_VBDDR_VDDR2=1u; \
                                         PINRELOC_SET_EPFR( FM_RTC->VBPFR, 4u, 2u, 0x00u ); \
                                         bFM_RTC_VBPFR_VPFR2=0u; \
                                         FM_RTC->WTCR20_f.PWRITE = 1u; \
                                         while(0u != FM_RTC->WTCR10_f.TRANS){} \
                                       }while(0u)

/*---- GPIO bit P54 ----*/

#define GPIO1PIN_P54_GET             ( bFM_GPIO_PDIR5_P4 )

#define GPIO1PIN_P54_PUT(v)          ( bFM_GPIO_PDOR5_P4=(v) )

#define GPIO1PIN_P54_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P54_INITIN(v) \
                                                        : GPIO1PIN_P54_INITOUT(v) )

#define GPIO1PIN_P54_INITIN(v)       do{ bFM_GPIO_PCR5_P4=(v).bPullup; \
                                         bFM_GPIO_DDR5_P4=0u; \
                                         bFM_GPIO_PFR5_P4=0u; }while(0u)

#define GPIO1PIN_P54_INITOUT(v)      do{ bFM_GPIO_PDOR5_P4=(v).bInitVal; \
                                         bFM_GPIO_DDR5_P4=1u; \
                                         bFM_GPIO_PFR5_P4=0u; }while(0u)


/*---- GPIO bit NP54 ----*/
#define GPIO1PIN_NP54_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR5_P4)) )

#define GPIO1PIN_NP54_PUT(v)         ( bFM_GPIO_PDOR5_P4=(uint32_t)(!(v)) )

#define GPIO1PIN_NP54_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP54_INITIN(v) \
                                                        : GPIO1PIN_NP54_INITOUT(v) )

#define GPIO1PIN_NP54_INITIN(v)      do{ bFM_GPIO_PCR5_P4=(v).bPullup; \
                                         bFM_GPIO_DDR5_P4=0u; \
                                         bFM_GPIO_PFR5_P4=0u; }while(0u)

#define GPIO1PIN_NP54_INITOUT(v)     do{ bFM_GPIO_PDOR5_P4=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR5_P4=1u; \
                                         bFM_GPIO_PFR5_P4=0u; }while(0u)


/*---- GPIO bit P55 ----*/

#define GPIO1PIN_P55_GET             ( bFM_GPIO_PDIR5_P5 )

#define GPIO1PIN_P55_PUT(v)          ( bFM_GPIO_PDOR5_P5=(v) )

#define GPIO1PIN_P55_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P55_INITIN(v) \
                                                        : GPIO1PIN_P55_INITOUT(v) )

#define GPIO1PIN_P55_INITIN(v)       do{ bFM_GPIO_PCR5_P5=(v).bPullup; \
                                         bFM_GPIO_DDR5_P5=0u; \
                                         bFM_GPIO_PFR5_P5=0u; }while(0u)

#define GPIO1PIN_P55_INITOUT(v)      do{ bFM_GPIO_PDOR5_P5=(v).bInitVal; \
                                         bFM_GPIO_DDR5_P5=1u; \
                                         bFM_GPIO_PFR5_P5=0u; }while(0u)


/*---- GPIO bit NP55 ----*/
#define GPIO1PIN_NP55_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR5_P5)) )

#define GPIO1PIN_NP55_PUT(v)         ( bFM_GPIO_PDOR5_P5=(uint32_t)(!(v)) )

#define GPIO1PIN_NP55_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP55_INITIN(v) \
                                                        : GPIO1PIN_NP55_INITOUT(v) )

#define GPIO1PIN_NP55_INITIN(v)      do{ bFM_GPIO_PCR5_P5=(v).bPullup; \
                                         bFM_GPIO_DDR5_P5=0u; \
                                         bFM_GPIO_PFR5_P5=0u; }while(0u)

#define GPIO1PIN_NP55_INITOUT(v)     do{ bFM_GPIO_PDOR5_P5=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR5_P5=1u; \
                                         bFM_GPIO_PFR5_P5=0u; }while(0u)


/*---- GPIO bit P56 ----*/

#define GPIO1PIN_P56_GET             ( bFM_GPIO_PDIR5_P6 )

#define GPIO1PIN_P56_PUT(v)          ( bFM_GPIO_PDOR5_P6=(v) )

#define GPIO1PIN_P56_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P56_INITIN(v) \
                                                        : GPIO1PIN_P56_INITOUT(v) )

#define GPIO1PIN_P56_INITIN(v)       do{ bFM_GPIO_PCR5_P6=(v).bPullup; \
                                         bFM_GPIO_DDR5_P6=0u; \
                                         bFM_GPIO_PFR5_P6=0u; }while(0u)

#define GPIO1PIN_P56_INITOUT(v)      do{ bFM_GPIO_PDOR5_P6=(v).bInitVal; \
                                         bFM_GPIO_DDR5_P6=1u; \
                                         bFM_GPIO_PFR5_P6=0u; }while(0u)


/*---- GPIO bit NP56 ----*/
#define GPIO1PIN_NP56_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR5_P6)) )

#define GPIO1PIN_NP56_PUT(v)         ( bFM_GPIO_PDOR5_P6=(uint32_t)(!(v)) )

#define GPIO1PIN_NP56_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP56_INITIN(v) \
                                                        : GPIO1PIN_NP56_INITOUT(v) )

#define GPIO1PIN_NP56_INITIN(v)      do{ bFM_GPIO_PCR5_P6=(v).bPullup; \
                                         bFM_GPIO_DDR5_P6=0u; \
                                         bFM_GPIO_PFR5_P6=0u; }while(0u)

#define GPIO1PIN_NP56_INITOUT(v)     do{ bFM_GPIO_PDOR5_P6=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR5_P6=1u; \
                                         bFM_GPIO_PFR5_P6=0u; }while(0u)


/*---- GPIO bit P57 ----*/

#define GPIO1PIN_P57_GET             ( bFM_GPIO_PDIR5_P7 )

#define GPIO1PIN_P57_PUT(v)          ( bFM_GPIO_PDOR5_P7=(v) )

#define GPIO1PIN_P57_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P57_INITIN(v) \
                                                        : GPIO1PIN_P57_INITOUT(v) )

#define GPIO1PIN_P57_INITIN(v)       do{ bFM_GPIO_PCR5_P7=(v).bPullup; \
                                         bFM_GPIO_DDR5_P7=0u; \
                                         bFM_GPIO_PFR5_P7=0u; }while(0u)

#define GPIO1PIN_P57_INITOUT(v)      do{ bFM_GPIO_PDOR5_P7=(v).bInitVal; \
                                         bFM_GPIO_DDR5_P7=1u; \
                                         bFM_GPIO_PFR5_P7=0u; }while(0u)


/*---- GPIO bit NP57 ----*/
#define GPIO1PIN_NP57_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR5_P7)) )

#define GPIO1PIN_NP57_PUT(v)         ( bFM_GPIO_PDOR5_P7=(uint32_t)(!(v)) )

#define GPIO1PIN_NP57_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP57_INITIN(v) \
                                                        : GPIO1PIN_NP57_INITOUT(v) )

#define GPIO1PIN_NP57_INITIN(v)      do{ bFM_GPIO_PCR5_P7=(v).bPullup; \
                                         bFM_GPIO_DDR5_P7=0u; \
                                         bFM_GPIO_PFR5_P7=0u; }while(0u)

#define GPIO1PIN_NP57_INITOUT(v)     do{ bFM_GPIO_PDOR5_P7=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR5_P7=1u; \
                                         bFM_GPIO_PFR5_P7=0u; }while(0u)


/*---- GPIO bit P60 ----*/

#define GPIO1PIN_P60_GET             ( bFM_GPIO_PDIR6_P0 )

#define GPIO1PIN_P60_PUT(v)          ( bFM_GPIO_PDOR6_P0=(v) )

#define GPIO1PIN_P60_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P60_INITIN(v) \
                                                        : GPIO1PIN_P60_INITOUT(v) )

#define GPIO1PIN_P60_INITIN(v)       do{ bFM_GPIO_PCR6_P0=(v).bPullup; \
                                         bFM_GPIO_DDR6_P0=0u; \
                                         bFM_GPIO_PFR6_P0=0u; }while(0u)

#define GPIO1PIN_P60_INITOUT(v)      do{ bFM_GPIO_PDOR6_P0=(v).bInitVal; \
                                         bFM_GPIO_DDR6_P0=1u; \
                                         bFM_GPIO_PFR6_P0=0u; }while(0u)


/*---- GPIO bit NP60 ----*/
#define GPIO1PIN_NP60_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR6_P0)) )

#define GPIO1PIN_NP60_PUT(v)         ( bFM_GPIO_PDOR6_P0=(uint32_t)(!(v)) )

#define GPIO1PIN_NP60_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP60_INITIN(v) \
                                                        : GPIO1PIN_NP60_INITOUT(v) )

#define GPIO1PIN_NP60_INITIN(v)      do{ bFM_GPIO_PCR6_P0=(v).bPullup; \
                                         bFM_GPIO_DDR6_P0=0u; \
                                         bFM_GPIO_PFR6_P0=0u; }while(0u)

#define GPIO1PIN_NP60_INITOUT(v)     do{ bFM_GPIO_PDOR6_P0=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR6_P0=1u; \
                                         bFM_GPIO_PFR6_P0=0u; }while(0u)


/*---- GPIO bit P61 ----*/

#define GPIO1PIN_P61_GET             ( bFM_GPIO_PDIR6_P1 )

#define GPIO1PIN_P61_PUT(v)          ( bFM_GPIO_PDOR6_P1=(v) )

#define GPIO1PIN_P61_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P61_INITIN(v) \
                                                        : GPIO1PIN_P61_INITOUT(v) )

#define GPIO1PIN_P61_INITIN(v)       do{ bFM_GPIO_PCR6_P1=(v).bPullup; \
                                         bFM_GPIO_DDR6_P1=0u; \
                                         bFM_GPIO_PFR6_P1=0u; \
                                         bFM_GPIO_ADE_AN14=0u; }while(0u)


#define GPIO1PIN_P61_INITOUT(v)      do{ bFM_GPIO_PDOR6_P1=(v).bInitVal; \
                                         bFM_GPIO_DDR6_P1=1u; \
                                         bFM_GPIO_PFR6_P1=0u; \
                                         bFM_GPIO_ADE_AN14=0u; }while(0u)


/*---- GPIO bit NP61 ----*/
#define GPIO1PIN_NP61_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR6_P1)) )

#define GPIO1PIN_NP61_PUT(v)         ( bFM_GPIO_PDOR6_P1=(uint32_t)(!(v)) )

#define GPIO1PIN_NP61_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP61_INITIN(v) \
                                                        : GPIO1PIN_NP61_INITOUT(v) )

#define GPIO1PIN_NP61_INITIN(v)      do{ bFM_GPIO_PCR6_P1=(v).bPullup; \
                                         bFM_GPIO_DDR6_P1=0u; \
                                         bFM_GPIO_PFR6_P1=0u; \
                                         bFM_GPIO_ADE_AN14=0u; }while(0u)


#define GPIO1PIN_NP61_INITOUT(v)     do{ bFM_GPIO_PDOR6_P1=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR6_P1=1u; \
                                         bFM_GPIO_PFR6_P1=0u; \
                                         bFM_GPIO_ADE_AN14=0u; }while(0u)


/*---- GPIO bit P66 ----*/

#define GPIO1PIN_P66_GET             ( bFM_GPIO_PDIR6_P6 )

#define GPIO1PIN_P66_PUT(v)          ( bFM_GPIO_PDOR6_P6=(v) )

#define GPIO1PIN_P66_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P66_INITIN(v) \
                                                        : GPIO1PIN_P66_INITOUT(v) )

#define GPIO1PIN_P66_INITIN(v)       do{ bFM_GPIO_PCR6_P6=(v).bPullup; \
                                         bFM_GPIO_DDR6_P6=0u; \
                                         bFM_GPIO_PFR6_P6=0u; \
                                         bFM_GPIO_ADE_AN09=0u; }while(0u)


#define GPIO1PIN_P66_INITOUT(v)      do{ bFM_GPIO_PDOR6_P6=(v).bInitVal; \
                                         bFM_GPIO_DDR6_P6=1u; \
                                         bFM_GPIO_PFR6_P6=0u; \
                                         bFM_GPIO_ADE_AN09=0u; }while(0u)


/*---- GPIO bit NP66 ----*/
#define GPIO1PIN_NP66_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR6_P6)) )

#define GPIO1PIN_NP66_PUT(v)         ( bFM_GPIO_PDOR6_P6=(uint32_t)(!(v)) )

#define GPIO1PIN_NP66_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP66_INITIN(v) \
                                                        : GPIO1PIN_NP66_INITOUT(v) )

#define GPIO1PIN_NP66_INITIN(v)      do{ bFM_GPIO_PCR6_P6=(v).bPullup; \
                                         bFM_GPIO_DDR6_P6=0u; \
                                         bFM_GPIO_PFR6_P6=0u; \
                                         bFM_GPIO_ADE_AN09=0u; }while(0u)


#define GPIO1PIN_NP66_INITOUT(v)     do{ bFM_GPIO_PDOR6_P6=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR6_P6=1u; \
                                         bFM_GPIO_PFR6_P6=0u; \
                                         bFM_GPIO_ADE_AN09=0u; }while(0u)


/*---- GPIO bit P80 ----*/

#define GPIO1PIN_P80_GET             ( bFM_GPIO_PDIR8_P0 )

#define GPIO1PIN_P80_PUT(v)          ( bFM_GPIO_PDOR8_P0=(v) )

#define GPIO1PIN_P80_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P80_INITIN(v) \
                                                        : GPIO1PIN_P80_INITOUT(v) )

#define GPIO1PIN_P80_INITIN(v)       do{ bFM_GPIO_PCR8_P0=(v).bPullup; \
                                         bFM_GPIO_DDR8_P0=0u; \
                                         bFM_GPIO_PFR8_P0=0u; }while(0u)

#define GPIO1PIN_P80_INITOUT(v)      do{ bFM_GPIO_PDOR8_P0=(v).bInitVal; \
                                         bFM_GPIO_DDR8_P0=1u; \
                                         bFM_GPIO_PFR8_P0=0u; }while(0u)


/*---- GPIO bit NP80 ----*/
#define GPIO1PIN_NP80_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR8_P0)) )

#define GPIO1PIN_NP80_PUT(v)         ( bFM_GPIO_PDOR8_P0=(uint32_t)(!(v)) )

#define GPIO1PIN_NP80_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP80_INITIN(v) \
                                                        : GPIO1PIN_NP80_INITOUT(v) )

#define GPIO1PIN_NP80_INITIN(v)      do{ bFM_GPIO_PCR8_P0=(v).bPullup; \
                                         bFM_GPIO_DDR8_P0=0u; \
                                         bFM_GPIO_PFR8_P0=0u; }while(0u)

#define GPIO1PIN_NP80_INITOUT(v)     do{ bFM_GPIO_PDOR8_P0=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR8_P0=1u; \
                                         bFM_GPIO_PFR8_P0=0u; }while(0u)


/*---- GPIO bit P81 ----*/

#define GPIO1PIN_P81_GET             ( bFM_GPIO_PDIR8_P1 )

#define GPIO1PIN_P81_PUT(v)          ( bFM_GPIO_PDOR8_P1=(v) )

#define GPIO1PIN_P81_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P81_INITIN(v) \
                                                        : GPIO1PIN_P81_INITOUT(v) )

#define GPIO1PIN_P81_INITIN(v)       do{ bFM_GPIO_PCR8_P1=(v).bPullup; \
                                         bFM_GPIO_DDR8_P1=0u; \
                                         bFM_GPIO_PFR8_P1=0u; }while(0u)

#define GPIO1PIN_P81_INITOUT(v)      do{ bFM_GPIO_PDOR8_P1=(v).bInitVal; \
                                         bFM_GPIO_DDR8_P1=1u; \
                                         bFM_GPIO_PFR8_P1=0u; }while(0u)


/*---- GPIO bit NP81 ----*/
#define GPIO1PIN_NP81_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR8_P1)) )

#define GPIO1PIN_NP81_PUT(v)         ( bFM_GPIO_PDOR8_P1=(uint32_t)(!(v)) )

#define GPIO1PIN_NP81_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP81_INITIN(v) \
                                                        : GPIO1PIN_NP81_INITOUT(v) )

#define GPIO1PIN_NP81_INITIN(v)      do{ bFM_GPIO_PCR8_P1=(v).bPullup; \
                                         bFM_GPIO_DDR8_P1=0u; \
                                         bFM_GPIO_PFR8_P1=0u; }while(0u)

#define GPIO1PIN_NP81_INITOUT(v)     do{ bFM_GPIO_PDOR8_P1=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR8_P1=1u; \
                                         bFM_GPIO_PFR8_P1=0u; }while(0u)


/*---- GPIO bit PE0 ----*/

#define GPIO1PIN_PE0_GET             ( bFM_GPIO_PDIRE_P0 )

#define GPIO1PIN_PE0_PUT(v)          ( bFM_GPIO_PDORE_P0=(v) )

#define GPIO1PIN_PE0_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_PE0_INITIN(v) \
                                                        : GPIO1PIN_PE0_INITOUT(v) )

#define GPIO1PIN_PE0_INITIN(v)       do{ bFM_GPIO_PCRE_P0=(v).bPullup; \
                                         bFM_GPIO_DDRE_P0=0u; \
                                         bFM_GPIO_PFRE_P0=0u; }while(0u)

#define GPIO1PIN_PE0_INITOUT(v)      do{ bFM_GPIO_PDORE_P0=(v).bInitVal; \
                                         bFM_GPIO_DDRE_P0=1u; \
                                         bFM_GPIO_PFRE_P0=0u; }while(0u)


/*---- GPIO bit NPE0 ----*/
#define GPIO1PIN_NPE0_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIRE_P0)) )

#define GPIO1PIN_NPE0_PUT(v)         ( bFM_GPIO_PDORE_P0=(uint32_t)(!(v)) )

#define GPIO1PIN_NPE0_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NPE0_INITIN(v) \
                                                        : GPIO1PIN_NPE0_INITOUT(v) )

#define GPIO1PIN_NPE0_INITIN(v)      do{ bFM_GPIO_PCRE_P0=(v).bPullup; \
                                         bFM_GPIO_DDRE_P0=0u; \
                                         bFM_GPIO_PFRE_P0=0u; }while(0u)

#define GPIO1PIN_NPE0_INITOUT(v)     do{ bFM_GPIO_PDORE_P0=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDRE_P0=1u; \
                                         bFM_GPIO_PFRE_P0=0u; }while(0u)


/*---- GPIO bit PE2 ----*/

#define GPIO1PIN_PE2_GET             ( bFM_GPIO_PDIRE_P2 )

#define GPIO1PIN_PE2_PUT(v)          ( bFM_GPIO_PDORE_P2=(v) )

#define GPIO1PIN_PE2_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_PE2_INITIN(v) \
                                                        : GPIO1PIN_PE2_INITOUT(v) )

#define GPIO1PIN_PE2_INITIN(v)       do{ bFM_GPIO_PCRE_P2=(v).bPullup; \
                                         bFM_GPIO_DDRE_P2=0u; \
/*--- Notice: This function will configure both of PE2/X0 and PE3/X1 as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_GPIO->SPSR, 2u, 2u, 0x00u ); \
                                         bFM_GPIO_PFRE_P2=0u; }while(0u)

#define GPIO1PIN_PE2_INITOUT(v)      do{ bFM_GPIO_PDORE_P2=(v).bInitVal; \
                                         bFM_GPIO_DDRE_P2=1u; \
/*--- Notice: This function will configure both of PE2/X0 and PE3/X1 as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_GPIO->SPSR, 2u, 2u, 0x00u ); \
                                         bFM_GPIO_PFRE_P2=0u; }while(0u)


/*---- GPIO bit NPE2 ----*/
#define GPIO1PIN_NPE2_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIRE_P2)) )

#define GPIO1PIN_NPE2_PUT(v)         ( bFM_GPIO_PDORE_P2=(uint32_t)(!(v)) )

#define GPIO1PIN_NPE2_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NPE2_INITIN(v) \
                                                        : GPIO1PIN_NPE2_INITOUT(v) )

#define GPIO1PIN_NPE2_INITIN(v)      do{ bFM_GPIO_PCRE_P2=(v).bPullup; \
                                         bFM_GPIO_DDRE_P2=0u; \
/*--- Notice: This function will configure both of PE2/X0 and PE3/X1 as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_GPIO->SPSR, 2u, 2u, 0x00u ); \
                                         bFM_GPIO_PFRE_P2=0u; }while(0u)

#define GPIO1PIN_NPE2_INITOUT(v)     do{ bFM_GPIO_PDORE_P2=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDRE_P2=1u; \
/*--- Notice: This function will configure both of PE2/X0 and PE3/X1 as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_GPIO->SPSR, 2u, 2u, 0x00u ); \
                                         bFM_GPIO_PFRE_P2=0u; }while(0u)


/*---- GPIO bit PE3 ----*/

#define GPIO1PIN_PE3_GET             ( bFM_GPIO_PDIRE_P3 )

#define GPIO1PIN_PE3_PUT(v)          ( bFM_GPIO_PDORE_P3=(v) )

#define GPIO1PIN_PE3_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_PE3_INITIN(v) \
                                                        : GPIO1PIN_PE3_INITOUT(v) )

#define GPIO1PIN_PE3_INITIN(v)       do{ bFM_GPIO_PCRE_P3=(v).bPullup; \
                                         bFM_GPIO_DDRE_P3=0u; \
/*--- Notice: This function will configure both of PE2/X0 and PE3/X1 as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_GPIO->SPSR, 2u, 2u, 0x00u ); \
                                         bFM_GPIO_PFRE_P3=0u; }while(0u)

#define GPIO1PIN_PE3_INITOUT(v)      do{ bFM_GPIO_PDORE_P3=(v).bInitVal; \
                                         bFM_GPIO_DDRE_P3=1u; \
/*--- Notice: This function will configure both of PE2/X0 and PE3/X1 as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_GPIO->SPSR, 2u, 2u, 0x00u ); \
                                         bFM_GPIO_PFRE_P3=0u; }while(0u)


/*---- GPIO bit NPE3 ----*/
#define GPIO1PIN_NPE3_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIRE_P3)) )

#define GPIO1PIN_NPE3_PUT(v)         ( bFM_GPIO_PDORE_P3=(uint32_t)(!(v)) )

#define GPIO1PIN_NPE3_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NPE3_INITIN(v) \
                                                        : GPIO1PIN_NPE3_INITOUT(v) )

#define GPIO1PIN_NPE3_INITIN(v)      do{ bFM_GPIO_PCRE_P3=(v).bPullup; \
                                         bFM_GPIO_DDRE_P3=0u; \
/*--- Notice: This function will configure both of PE2/X0 and PE3/X1 as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_GPIO->SPSR, 2u, 2u, 0x00u ); \
                                         bFM_GPIO_PFRE_P3=0u; }while(0u)

#define GPIO1PIN_NPE3_INITOUT(v)     do{ bFM_GPIO_PDORE_P3=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDRE_P3=1u; \
/*--- Notice: This function will configure both of PE2/X0 and PE3/X1 as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_GPIO->SPSR, 2u, 2u, 0x00u ); \
                                         bFM_GPIO_PFRE_P3=0u; }while(0u)



/******************************************************************************
   PIN RELOCATION
*******************************************************************************/

/*--- ADTG_0 for ADC#0 ---*/
#define SetPinFunc_ADTG_0_ADC0(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 12u, 4u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P7 = 1u; }while(0u)

/*--- ADTG_0 for ADC#1 ---*/
#define SetPinFunc_ADTG_0_ADC1(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 16u, 4u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P7 = 1u; }while(0u)

/*--- ADTG_0 for ADC#2 ---*/
#define SetPinFunc_ADTG_0_ADC2(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 20u, 4u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P7 = 1u; }while(0u)

/*--- ADTG_2 for ADC#0 ---*/
#define SetPinFunc_ADTG_2_ADC0(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 12u, 4u, 0x03u ); \
                                                 bFM_GPIO_PFR1_P1 = 1u; }while(0u)

/*--- ADTG_2 for ADC#1 ---*/
#define SetPinFunc_ADTG_2_ADC1(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 16u, 4u, 0x03u ); \
                                                 bFM_GPIO_PFR1_P1 = 1u; }while(0u)

/*--- ADTG_2 for ADC#2 ---*/
#define SetPinFunc_ADTG_2_ADC2(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 20u, 4u, 0x03u ); \
                                                 bFM_GPIO_PFR1_P1 = 1u; }while(0u)

/*--- ADTG_3 for ADC#0 ---*/
#define SetPinFunc_ADTG_3_ADC0(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 12u, 4u, 0x04u ); \
                                                 bFM_GPIO_PFR2_P1 = 1u; \
                                                 bFM_GPIO_ADE_AN03 = 0u; }while(0u)

/*--- ADTG_3 for ADC#1 ---*/
#define SetPinFunc_ADTG_3_ADC1(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 16u, 4u, 0x04u ); \
                                                 bFM_GPIO_PFR2_P1 = 1u; \
                                                 bFM_GPIO_ADE_AN03 = 0u; }while(0u)

/*--- ADTG_3 for ADC#2 ---*/
#define SetPinFunc_ADTG_3_ADC2(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 20u, 4u, 0x04u ); \
                                                 bFM_GPIO_PFR2_P1 = 1u; \
                                                 bFM_GPIO_ADE_AN03 = 0u; }while(0u)

/*--- ADTG_5 for ADC#0 ---*/
#define SetPinFunc_ADTG_5_ADC0(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 12u, 4u, 0x06u ); \
                                                 bFM_GPIO_PFR6_P1 = 1u; \
                                                 bFM_GPIO_ADE_AN14 = 0u; }while(0u)

/*--- ADTG_5 for ADC#1 ---*/
#define SetPinFunc_ADTG_5_ADC1(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 16u, 4u, 0x06u ); \
                                                 bFM_GPIO_PFR6_P1 = 1u; \
                                                 bFM_GPIO_ADE_AN14 = 0u; }while(0u)

/*--- ADTG_5 for ADC#2 ---*/
#define SetPinFunc_ADTG_5_ADC2(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 20u, 4u, 0x06u ); \
                                                 bFM_GPIO_PFR6_P1 = 1u; \
                                                 bFM_GPIO_ADE_AN14 = 0u; }while(0u)

/*--- AIN0_1 ---*/
#define SetPinFunc_AIN0_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 0u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P0 = 1u; }while(0u)

/*--- AIN0_2 ---*/
#define SetPinFunc_AIN0_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 0u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR1_P0 = 1u; }while(0u)

/*--- BIN0_1 ---*/
#define SetPinFunc_BIN0_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 2u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P1 = 1u; }while(0u)

/*--- BIN0_2 ---*/
#define SetPinFunc_BIN0_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 2u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR1_P1 = 1u; }while(0u)

/*--- CROUT_1 ---*/
#define SetPinFunc_CROUT_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR00, 1u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR6_P6 = 1u; \
                                                 bFM_GPIO_ADE_AN09 = 0u; }while(0u)

/*--- DA0 ---*/
#define SetPinFunc_DA0(dummy)                do{ }while(0u)

/*--- DA1 ---*/
#define SetPinFunc_DA1(dummy)                do{ }while(0u)

/*--- DTTI0X_0 ---*/
#define SetPinFunc_DTTI0X_0(dummy)           do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 16u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P7 = 1u; }while(0u)

/*--- FRCK0_0 ---*/
#define SetPinFunc_FRCK0_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 18u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P6 = 1u; }while(0u)

/*--- IC00_1 ---*/
#define SetPinFunc_IC00_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 20u, 3u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P3 = 1u; \
                                                 bFM_GPIO_ADE_AN01 = 0u; }while(0u)

/*--- IC00_2 ---*/
#define SetPinFunc_IC00_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 20u, 3u, 0x03u ); \
                                                 bFM_GPIO_PFR6_P1 = 1u; \
                                                 bFM_GPIO_ADE_AN14 = 0u; }while(0u)

/*--- IC00_LSYN0 ---*/
#define SetPinFunc_IC00_LSYN0(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 20u, 3u, 0x04u ); \
                                                 }while(0)

/*--- IC00_LSYN4 ---*/
#define SetPinFunc_IC00_LSYN4(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 20u, 3u, 0x05u ); \
                                                 }while(0)

/*--- IC01_1 ---*/
#define SetPinFunc_IC01_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 23u, 3u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P2 = 1u; \
                                                 bFM_GPIO_ADE_AN00 = 0u; }while(0u)

/*--- IC01_2 ---*/
#define SetPinFunc_IC01_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 23u, 3u, 0x03u ); \
                                                 bFM_GPIO_PFR6_P0 = 1u; }while(0u)

/*--- IC01_LSYN1 ---*/
#define SetPinFunc_IC01_LSYN1(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 23u, 3u, 0x04u ); \
                                                 }while(0)

/*--- IC01_LSYN5 ---*/
#define SetPinFunc_IC01_LSYN5(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 23u, 3u, 0x05u ); \
                                                 }while(0)

/*--- IC02_0 ---*/
#define SetPinFunc_IC02_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 26u, 3u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P4 = 1u; }while(0u)

/*--- IC02_1 ---*/
#define SetPinFunc_IC02_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 26u, 3u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P1 = 1u; }while(0u)

/*--- IC02_LSYN2 ---*/
#define SetPinFunc_IC02_LSYN2(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 26u, 3u, 0x04u ); \
                                                 }while(0)

/*--- IC02_LSYN6 ---*/
#define SetPinFunc_IC02_LSYN6(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 26u, 3u, 0x05u ); \
                                                 }while(0)

/*--- IC03_0 ---*/
#define SetPinFunc_IC03_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 29u, 3u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P5 = 1u; }while(0u)

/*--- IC03_1 ---*/
#define SetPinFunc_IC03_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 29u, 3u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P0 = 1u; }while(0u)

/*--- IC03_LSYN3 ---*/
#define SetPinFunc_IC03_LSYN3(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 29u, 3u, 0x04u ); \
                                                 }while(0)

/*--- IC03_LSYN7 ---*/
#define SetPinFunc_IC03_LSYN7(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 29u, 3u, 0x05u ); \
                                                 }while(0)

/*--- INT00_1 ---*/
#define SetPinFunc_INT00_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 0u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P0 = 1u; }while(0u)

/*--- INT01_1 ---*/
#define SetPinFunc_INT01_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 2u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P1 = 1u; }while(0u)

/*--- INT02_1 ---*/
#define SetPinFunc_INT02_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 4u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P2 = 1u; \
                                                 bFM_GPIO_ADE_AN00 = 0u; }while(0u)

/*--- INT03_1 ---*/
#define SetPinFunc_INT03_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 6u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P0 = 1u; \
                                                 bFM_GPIO_ADE_AN02 = 0u; }while(0u)

/*--- INT04_0 ---*/
#define SetPinFunc_INT04_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 8u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P4 = 1u; }while(0u)

/*--- INT04_1 ---*/
#define SetPinFunc_INT04_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 8u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P1 = 1u; \
                                                 bFM_GPIO_ADE_AN03 = 0u; }while(0u)

/*--- INT05_0 ---*/
#define SetPinFunc_INT05_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 10u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P5 = 1u; }while(0u)

/*--- INT05_1 ---*/
#define SetPinFunc_INT05_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 10u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P2 = 1u; \
                                                 bFM_GPIO_ADE_AN04 = 0u; }while(0u)

/*--- INT06_0 ---*/
#define SetPinFunc_INT06_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 12u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P6 = 1u; }while(0u)

/*--- INT06_1 ---*/
#define SetPinFunc_INT06_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 12u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P3 = 1u; \
                                                 bFM_GPIO_ADE_AN05 = 0u; }while(0u)

/*--- INT07_0 ---*/
#define SetPinFunc_INT07_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 14u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P7 = 1u; }while(0u)

/*--- INT08_0 ---*/
#define SetPinFunc_INT08_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 16u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_P0 = 1u; }while(0u)

/*--- INT09_0 ---*/
#define SetPinFunc_INT09_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 18u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_P1 = 1u; }while(0u)

/*--- INT10_0 ---*/
#define SetPinFunc_INT10_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 20u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_P2 = 1u; }while(0u)

/*--- INT10_1 ---*/
#define SetPinFunc_INT10_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 20u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR6_P6 = 1u; \
                                                 bFM_GPIO_ADE_AN09 = 0u; }while(0u)

/*--- INT11_0 ---*/
#define SetPinFunc_INT11_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 22u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_P3 = 1u; }while(0u)

/*--- INT12_0 ---*/
#define SetPinFunc_INT12_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 24u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_P4 = 1u; }while(0u)

/*--- INT13_0 ---*/
#define SetPinFunc_INT13_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 26u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_P5 = 1u; }while(0u)

/*--- INT15_1 ---*/
#define SetPinFunc_INT15_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 30u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR6_P1 = 1u; \
                                                 bFM_GPIO_ADE_AN14 = 0u; }while(0u)

/*--- MD1 ---*/
#define SetPinFunc_MD1(dummy)                do{ }while(0u)

/*--- NMIX ---*/
#define SetPinFunc_NMIX(dummy)               do{ bFM_GPIO_EPFR00_NMIS = 1u; \
                                                 bFM_GPIO_PFR6_P0 = 1u; }while(0u)

/*--- RTCCO_1 ---*/
#define SetPinFunc_RTCCO_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR00, 4u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P0 = 1u; \
                                                 bFM_GPIO_ADE_AN02 = 0u; }while(0u)

/*--- RTO00_0 ---*/
#define SetPinFunc_RTO00_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 0u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_P0 = 1u; }while(0u)

/*--- RTO01_0 ---*/
#define SetPinFunc_RTO01_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 2u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_P1 = 1u; }while(0u)

/*--- RTO02_0 ---*/
#define SetPinFunc_RTO02_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 4u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_P2 = 1u; }while(0u)

/*--- RTO03_0 ---*/
#define SetPinFunc_RTO03_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 6u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_P3 = 1u; }while(0u)

/*--- RTO04_0 ---*/
#define SetPinFunc_RTO04_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 8u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_P4 = 1u; }while(0u)

/*--- RTO05_0 ---*/
#define SetPinFunc_RTO05_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 10u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_P5 = 1u; }while(0u)

/*--- SCK0_0 ---*/
#define SetPinFunc_SCK0_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 8u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR2_P3 = 1u; \
                                                 bFM_GPIO_ADE_AN05 = 0u; }while(0u)

/*--- SCK1_1 ---*/
#define SetPinFunc_SCK1_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 14u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR6_P0 = 1u; }while(0u)

/*--- SCK2_0 ---*/
#define SetPinFunc_SCK2_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 20u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_P2 = 1u; }while(0u)

/*--- SCK3_0 ---*/
#define SetPinFunc_SCK3_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 26u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P7 = 1u; }while(0u)

/*--- SCK4_1 ---*/
#define SetPinFunc_SCK4_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 8u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P5 = 1u; }while(0u)

/*--- SCK6_0 ---*/
#define SetPinFunc_SCK6_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 20u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR1_P2 = 1u; \
                                                 bFM_GPIO_ADE_AN00 = 0u; }while(0u)

/*--- SCS6_0 ---*/
#define SetPinFunc_SCS6_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR16, 0u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR1_P1 = 1u; }while(0u)

/*--- SIN0_0 ---*/
#define SetPinFunc_SIN0_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 4u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR2_P1 = 1u; \
                                                 bFM_GPIO_ADE_AN03 = 0u; }while(0u)

/*--- SIN2_0 ---*/
#define SetPinFunc_SIN2_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 16u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_P0 = 1u; }while(0u)

/*--- SIN3_0 ---*/
#define SetPinFunc_SIN3_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 22u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P5 = 1u; }while(0u)

/*--- SIN4_1 ---*/
#define SetPinFunc_SIN4_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 4u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P3 = 1u; }while(0u)

/*--- SIN6_0 ---*/
#define SetPinFunc_SIN6_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 16u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR2_P0 = 1u; \
                                                 bFM_GPIO_ADE_AN02 = 0u; }while(0u)

/*--- SOT0_0 ---*/
#define SetPinFunc_SOT0_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 6u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR2_P2 = 1u; \
                                                 bFM_GPIO_ADE_AN04 = 0u; }while(0u)

/*--- SOT1_1 ---*/
#define SetPinFunc_SOT1_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 12u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR6_P1 = 1u; \
                                                 bFM_GPIO_ADE_AN14 = 0u; }while(0u)

/*--- SOT2_0 ---*/
#define SetPinFunc_SOT2_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 18u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_P1 = 1u; }while(0u)

/*--- SOT3_0 ---*/
#define SetPinFunc_SOT3_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 24u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P6 = 1u; }while(0u)

/*--- SOT4_1 ---*/
#define SetPinFunc_SOT4_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 6u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P4 = 1u; }while(0u)

/*--- SOT6_0 ---*/
#define SetPinFunc_SOT6_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 18u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR1_P3 = 1u; \
                                                 bFM_GPIO_ADE_AN01 = 0u; }while(0u)

/*--- SUBOUT_1 ---*/
#define SetPinFunc_SUBOUT_1(dummy)           do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR00, 6u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P0 = 1u; \
                                                 bFM_GPIO_ADE_AN02 = 0u; }while(0u)

/*--- TIOA0_1_OUT ---*/
#define SetPinFunc_TIOA0_1_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 2u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P0 = 1u; \
                                                 bFM_GPIO_ADE_AN02 = 0u; }while(0u)

/*--- TIOA1_1_IN ---*/
#define SetPinFunc_TIOA1_1_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 8u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P2 = 1u; \
                                                 bFM_GPIO_ADE_AN04 = 0u; }while(0u)

/*--- TIOA1_1_OUT ---*/
#define SetPinFunc_TIOA1_1_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 10u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P2 = 1u; \
                                                 bFM_GPIO_ADE_AN04 = 0u; }while(0u)

/*--- TIOA2_0_OUT ---*/
#define SetPinFunc_TIOA2_0_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 18u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P4 = 1u; }while(0u)

/*--- TIOA3_0_IN ---*/
#define SetPinFunc_TIOA3_0_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 24u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P6 = 1u; }while(0u)

/*--- TIOA3_0_OUT ---*/
#define SetPinFunc_TIOA3_0_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 26u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P6 = 1u; }while(0u)

/*--- TIOA4_0_OUT ---*/
#define SetPinFunc_TIOA4_0_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 2u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_P0 = 1u; }while(0u)

/*--- TIOA4_1_OUT ---*/
#define SetPinFunc_TIOA4_1_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 2u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P0 = 1u; }while(0u)

/*--- TIOA5_0_IN ---*/
#define SetPinFunc_TIOA5_0_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 8u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_P2 = 1u; }while(0u)

/*--- TIOA5_0_OUT ---*/
#define SetPinFunc_TIOA5_0_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 10u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_P2 = 1u; }while(0u)

/*--- TIOA6_0_OUT ---*/
#define SetPinFunc_TIOA6_0_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 18u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_P4 = 1u; }while(0u)

/*--- TIOA7_1_IN ---*/
#define SetPinFunc_TIOA7_1_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 24u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P2 = 1u; \
                                                 bFM_GPIO_ADE_AN00 = 0u; }while(0u)

/*--- TIOA7_1_OUT ---*/
#define SetPinFunc_TIOA7_1_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 26u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P2 = 1u; \
                                                 bFM_GPIO_ADE_AN00 = 0u; }while(0u)

/*--- TIOB0_1_IN ---*/
#define SetPinFunc_TIOB0_1_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 4u, 3u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P1 = 1u; \
                                                 bFM_GPIO_ADE_AN03 = 0u; }while(0u)

/*--- TIOB1_1_IN ---*/
#define SetPinFunc_TIOB1_1_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 12u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P3 = 1u; \
                                                 bFM_GPIO_ADE_AN05 = 0u; }while(0u)

/*--- TIOB2_0_IN ---*/
#define SetPinFunc_TIOB2_0_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 20u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P5 = 1u; }while(0u)

/*--- TIOB3_0_IN ---*/
#define SetPinFunc_TIOB3_0_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 28u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P7 = 1u; }while(0u)

/*--- TIOB4_0_IN ---*/
#define SetPinFunc_TIOB4_0_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 4u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_P1 = 1u; }while(0u)

/*--- TIOB4_1_IN ---*/
#define SetPinFunc_TIOB4_1_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 4u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P1 = 1u; }while(0u)

/*--- TIOB5_0_IN ---*/
#define SetPinFunc_TIOB5_0_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 12u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_P3 = 1u; }while(0u)

/*--- TIOB6_0_IN ---*/
#define SetPinFunc_TIOB6_0_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 20u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_P5 = 1u; }while(0u)

/*--- TIOB7_1_IN ---*/
#define SetPinFunc_TIOB7_1_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 28u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P3 = 1u; \
                                                 bFM_GPIO_ADE_AN01 = 0u; }while(0u)

/*--- WKUP0 ---*/
#define SetPinFunc_WKUP0(dummy)              do{ }while(0u)

/*--- WKUP1 ---*/
#define SetPinFunc_WKUP1(dummy)              do{ }while(0u)

/*--- WKUP2 ---*/
#define SetPinFunc_WKUP2(dummy)              do{ }while(0u)

/*--- X0 ---*/
/*--- Notice: This function will configure X0 and X1 as clock pins. ---*/
#define SetPinFunc_X0(dummy)                 do{ PINRELOC_SET_EPFR( FM_GPIO->SPSR, 2u, 2u, 0x01u ); }while(0u)

/*--- X0A ---*/
/*--- Notice: This function will configure X0A and X1A as clock pins. ---*/
#define SetPinFunc_X0A(dummy)                do{ \
                                                 FM_RTC->WTCR20_f.PREAD = 1u; \
                                                 while(0u != FM_RTC->WTCR10_f.TRANS){} \
                                                 PINRELOC_SET_EPFR( FM_RTC->VBPFR, 4u, 2u, 0x01u ); \
                                                 bFM_RTC_VBPFR_VPFR2 = 1u; \
                                                 bFM_RTC_VBPFR_VPFR3 = 1u; \
                                                 FM_RTC->WTCR20_f.PWRITE = 1u; \
                                                 while(0u != FM_RTC->WTCR10_f.TRANS){} \
                                               }while(0u)

/*--- X1 ---*/
/*--- Notice: This function will configure X0 and X1 as clock pins. ---*/
#define SetPinFunc_X1(dummy)                 do{ PINRELOC_SET_EPFR( FM_GPIO->SPSR, 2u, 2u, 0x01u ); }while(0u)

/*--- X1A ---*/
/*--- Notice: This function will configure X0A and X1A as clock pins. ---*/
#define SetPinFunc_X1A(dummy)                do{ \
                                                 FM_RTC->WTCR20_f.PREAD = 1u; \
                                                 while(0u != FM_RTC->WTCR10_f.TRANS){} \
                                                 PINRELOC_SET_EPFR( FM_RTC->VBPFR, 4u, 2u, 0x01u ); \
                                                 bFM_RTC_VBPFR_VPFR2 = 1u; \
                                                 bFM_RTC_VBPFR_VPFR3 = 1u; \
                                                 FM_RTC->WTCR20_f.PWRITE = 1u; \
                                                 while(0u != FM_RTC->WTCR10_f.TRANS){} \
                                               }while(0u)

/*--- ZIN0_1 ---*/
#define SetPinFunc_ZIN0_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 4u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P2 = 1u; }while(0u)

/*--- ZIN0_2 ---*/
#define SetPinFunc_ZIN0_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 4u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR1_P2 = 1u; \
                                                 bFM_GPIO_ADE_AN00 = 0u; }while(0u)


/******************************************************************************
   ANALOG PINS
*******************************************************************************/

/*--- AN00 ---*/
#define SetPinFunc_AN00(dummy)               do{ bFM_GPIO_ADE_AN00 = 1u; }while(0u)

/*--- AN01 ---*/
#define SetPinFunc_AN01(dummy)               do{ bFM_GPIO_ADE_AN01 = 1u; }while(0u)

/*--- AN02 ---*/
#define SetPinFunc_AN02(dummy)               do{ bFM_GPIO_ADE_AN02 = 1u; }while(0u)

/*--- AN03 ---*/
#define SetPinFunc_AN03(dummy)               do{ bFM_GPIO_ADE_AN03 = 1u; }while(0u)

/*--- AN04 ---*/
#define SetPinFunc_AN04(dummy)               do{ bFM_GPIO_ADE_AN04 = 1u; }while(0u)

/*--- AN05 ---*/
#define SetPinFunc_AN05(dummy)               do{ bFM_GPIO_ADE_AN05 = 1u; }while(0u)

/*--- AN09 ---*/
#define SetPinFunc_AN09(dummy)               do{ bFM_GPIO_ADE_AN09 = 1u; }while(0u)

/*--- AN14 ---*/
#define SetPinFunc_AN14(dummy)               do{ bFM_GPIO_ADE_AN14 = 1u; }while(0u)


#endif // #ifndef __GPIO_MB9BF16XK_H__


/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/

