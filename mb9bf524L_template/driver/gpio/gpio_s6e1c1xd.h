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
/** \file gpio_s6e1c1xd.h
 **
 ** Headerfile for S6E1C1XD GPIO functions, included in gpio.h
 **
 ** History:
 **   - 2015-06-26  1.0  Edison Zhang    First version
 **
 ******************************************************************************/

#ifndef __GPIO_S6E1C1XD_H__
#define __GPIO_S6E1C1XD_H__

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


/*---- GPIO bit P05 ----*/

#define GPIO1PIN_P05_GET             ( bFM_GPIO_PDIR0_P5 )

#define GPIO1PIN_P05_PUT(v)          ( bFM_GPIO_PDOR0_P5=(v) )

#define GPIO1PIN_P05_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P05_INITIN(v) \
                                                        : GPIO1PIN_P05_INITOUT(v) )

#define GPIO1PIN_P05_INITIN(v)       do{ bFM_GPIO_PCR0_P5=(v).bPullup; \
                                         bFM_GPIO_DDR0_P5=0u; \
                                         bFM_GPIO_PFR0_P5=0u; }while(0u)

#define GPIO1PIN_P05_INITOUT(v)      do{ bFM_GPIO_PDOR0_P5=(v).bInitVal; \
                                         bFM_GPIO_DDR0_P5=1u; \
                                         bFM_GPIO_PFR0_P5=0u; }while(0u)


/*---- GPIO bit NP05 ----*/
#define GPIO1PIN_NP05_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR0_P5)) )

#define GPIO1PIN_NP05_PUT(v)         ( bFM_GPIO_PDOR0_P5=(uint32_t)(!(v)) )

#define GPIO1PIN_NP05_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP05_INITIN(v) \
                                                        : GPIO1PIN_NP05_INITOUT(v) )

#define GPIO1PIN_NP05_INITIN(v)      do{ bFM_GPIO_PCR0_P5=(v).bPullup; \
                                         bFM_GPIO_DDR0_P5=0u; \
                                         bFM_GPIO_PFR0_P5=0u; }while(0u)

#define GPIO1PIN_NP05_INITOUT(v)     do{ bFM_GPIO_PDOR0_P5=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR0_P5=1u; \
                                         bFM_GPIO_PFR0_P5=0u; }while(0u)


/*---- GPIO bit P0B ----*/

#define GPIO1PIN_P0B_GET             ( bFM_GPIO_PDIR0_PB )

#define GPIO1PIN_P0B_PUT(v)          ( bFM_GPIO_PDOR0_PB=(v) )

#define GPIO1PIN_P0B_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P0B_INITIN(v) \
                                                        : GPIO1PIN_P0B_INITOUT(v) )

#define GPIO1PIN_P0B_INITIN(v)       do{ bFM_GPIO_PCR0_PB=(v).bPullup; \
                                         bFM_GPIO_DDR0_PB=0u; \
                                         bFM_GPIO_PFR0_PB=0u; }while(0u)

#define GPIO1PIN_P0B_INITOUT(v)      do{ bFM_GPIO_PDOR0_PB=(v).bInitVal; \
                                         bFM_GPIO_DDR0_PB=1u; \
                                         bFM_GPIO_PFR0_PB=0u; }while(0u)


/*---- GPIO bit NP0B ----*/
#define GPIO1PIN_NP0B_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR0_PB)) )

#define GPIO1PIN_NP0B_PUT(v)         ( bFM_GPIO_PDOR0_PB=(uint32_t)(!(v)) )

#define GPIO1PIN_NP0B_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP0B_INITIN(v) \
                                                        : GPIO1PIN_NP0B_INITOUT(v) )

#define GPIO1PIN_NP0B_INITIN(v)      do{ bFM_GPIO_PCR0_PB=(v).bPullup; \
                                         bFM_GPIO_DDR0_PB=0u; \
                                         bFM_GPIO_PFR0_PB=0u; }while(0u)

#define GPIO1PIN_NP0B_INITOUT(v)     do{ bFM_GPIO_PDOR0_PB=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR0_PB=1u; \
                                         bFM_GPIO_PFR0_PB=0u; }while(0u)


/*---- GPIO bit P0C ----*/

#define GPIO1PIN_P0C_GET             ( bFM_GPIO_PDIR0_PC )

#define GPIO1PIN_P0C_PUT(v)          ( bFM_GPIO_PDOR0_PC=(v) )

#define GPIO1PIN_P0C_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P0C_INITIN(v) \
                                                        : GPIO1PIN_P0C_INITOUT(v) )

#define GPIO1PIN_P0C_INITIN(v)       do{ bFM_GPIO_PCR0_PC=(v).bPullup; \
                                         bFM_GPIO_DDR0_PC=0u; \
                                         bFM_GPIO_PFR0_PC=0u; }while(0u)

#define GPIO1PIN_P0C_INITOUT(v)      do{ bFM_GPIO_PDOR0_PC=(v).bInitVal; \
                                         bFM_GPIO_DDR0_PC=1u; \
                                         bFM_GPIO_PFR0_PC=0u; }while(0u)


/*---- GPIO bit NP0C ----*/
#define GPIO1PIN_NP0C_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR0_PC)) )

#define GPIO1PIN_NP0C_PUT(v)         ( bFM_GPIO_PDOR0_PC=(uint32_t)(!(v)) )

#define GPIO1PIN_NP0C_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP0C_INITIN(v) \
                                                        : GPIO1PIN_NP0C_INITOUT(v) )

#define GPIO1PIN_NP0C_INITIN(v)      do{ bFM_GPIO_PCR0_PC=(v).bPullup; \
                                         bFM_GPIO_DDR0_PC=0u; \
                                         bFM_GPIO_PFR0_PC=0u; }while(0u)

#define GPIO1PIN_NP0C_INITOUT(v)     do{ bFM_GPIO_PDOR0_PC=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR0_PC=1u; \
                                         bFM_GPIO_PFR0_PC=0u; }while(0u)


/*---- GPIO bit P0F ----*/

#define GPIO1PIN_P0F_GET             ( bFM_GPIO_PDIR0_PF )

#define GPIO1PIN_P0F_PUT(v)          ( bFM_GPIO_PDOR0_PF=(v) )

#define GPIO1PIN_P0F_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P0F_INITIN(v) \
                                                        : GPIO1PIN_P0F_INITOUT(v) )

#define GPIO1PIN_P0F_INITIN(v)       do{ bFM_GPIO_PCR0_PF=(v).bPullup; \
                                         bFM_GPIO_DDR0_PF=0u; \
                                         bFM_GPIO_PFR0_PF=0u; }while(0u)

#define GPIO1PIN_P0F_INITOUT(v)      do{ bFM_GPIO_PDOR0_PF=(v).bInitVal; \
                                         bFM_GPIO_DDR0_PF=1u; \
                                         bFM_GPIO_PFR0_PF=0u; }while(0u)


/*---- GPIO bit NP0F ----*/
#define GPIO1PIN_NP0F_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR0_PF)) )

#define GPIO1PIN_NP0F_PUT(v)         ( bFM_GPIO_PDOR0_PF=(uint32_t)(!(v)) )

#define GPIO1PIN_NP0F_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP0F_INITIN(v) \
                                                        : GPIO1PIN_NP0F_INITOUT(v) )

#define GPIO1PIN_NP0F_INITIN(v)      do{ bFM_GPIO_PCR0_PF=(v).bPullup; \
                                         bFM_GPIO_DDR0_PF=0u; \
                                         bFM_GPIO_PFR0_PF=0u; }while(0u)

#define GPIO1PIN_NP0F_INITOUT(v)     do{ bFM_GPIO_PDOR0_PF=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR0_PF=1u; \
                                         bFM_GPIO_PFR0_PF=0u; }while(0u)


/*---- GPIO bit P10 ----*/

#define GPIO1PIN_P10_GET             ( bFM_GPIO_PDIR1_P0 )

#define GPIO1PIN_P10_PUT(v)          ( bFM_GPIO_PDOR1_P0=(v) )

#define GPIO1PIN_P10_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P10_INITIN(v) \
                                                        : GPIO1PIN_P10_INITOUT(v) )

#define GPIO1PIN_P10_INITIN(v)       do{ bFM_GPIO_PCR1_P0=(v).bPullup; \
                                         bFM_GPIO_DDR1_P0=0u; \
                                         bFM_GPIO_PFR1_P0=0u; \
                                         bFM_GPIO_ADE_AN00=0u; }while(0u)


#define GPIO1PIN_P10_INITOUT(v)      do{ bFM_GPIO_PDOR1_P0=(v).bInitVal; \
                                         bFM_GPIO_DDR1_P0=1u; \
                                         bFM_GPIO_PFR1_P0=0u; \
                                         bFM_GPIO_ADE_AN00=0u; }while(0u)


/*---- GPIO bit NP10 ----*/
#define GPIO1PIN_NP10_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR1_P0)) )

#define GPIO1PIN_NP10_PUT(v)         ( bFM_GPIO_PDOR1_P0=(uint32_t)(!(v)) )

#define GPIO1PIN_NP10_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP10_INITIN(v) \
                                                        : GPIO1PIN_NP10_INITOUT(v) )

#define GPIO1PIN_NP10_INITIN(v)      do{ bFM_GPIO_PCR1_P0=(v).bPullup; \
                                         bFM_GPIO_DDR1_P0=0u; \
                                         bFM_GPIO_PFR1_P0=0u; \
                                         bFM_GPIO_ADE_AN00=0u; }while(0u)


#define GPIO1PIN_NP10_INITOUT(v)     do{ bFM_GPIO_PDOR1_P0=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR1_P0=1u; \
                                         bFM_GPIO_PFR1_P0=0u; \
                                         bFM_GPIO_ADE_AN00=0u; }while(0u)


/*---- GPIO bit P11 ----*/

#define GPIO1PIN_P11_GET             ( bFM_GPIO_PDIR1_P1 )

#define GPIO1PIN_P11_PUT(v)          ( bFM_GPIO_PDOR1_P1=(v) )

#define GPIO1PIN_P11_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P11_INITIN(v) \
                                                        : GPIO1PIN_P11_INITOUT(v) )

#define GPIO1PIN_P11_INITIN(v)       do{ bFM_GPIO_PCR1_P1=(v).bPullup; \
                                         bFM_GPIO_DDR1_P1=0u; \
                                         bFM_GPIO_PFR1_P1=0u; \
                                         bFM_GPIO_ADE_AN01=0u; }while(0u)


#define GPIO1PIN_P11_INITOUT(v)      do{ bFM_GPIO_PDOR1_P1=(v).bInitVal; \
                                         bFM_GPIO_DDR1_P1=1u; \
                                         bFM_GPIO_PFR1_P1=0u; \
                                         bFM_GPIO_ADE_AN01=0u; }while(0u)


/*---- GPIO bit NP11 ----*/
#define GPIO1PIN_NP11_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR1_P1)) )

#define GPIO1PIN_NP11_PUT(v)         ( bFM_GPIO_PDOR1_P1=(uint32_t)(!(v)) )

#define GPIO1PIN_NP11_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP11_INITIN(v) \
                                                        : GPIO1PIN_NP11_INITOUT(v) )

#define GPIO1PIN_NP11_INITIN(v)      do{ bFM_GPIO_PCR1_P1=(v).bPullup; \
                                         bFM_GPIO_DDR1_P1=0u; \
                                         bFM_GPIO_PFR1_P1=0u; \
                                         bFM_GPIO_ADE_AN01=0u; }while(0u)


#define GPIO1PIN_NP11_INITOUT(v)     do{ bFM_GPIO_PDOR1_P1=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR1_P1=1u; \
                                         bFM_GPIO_PFR1_P1=0u; \
                                         bFM_GPIO_ADE_AN01=0u; }while(0u)


/*---- GPIO bit P12 ----*/

#define GPIO1PIN_P12_GET             ( bFM_GPIO_PDIR1_P2 )

#define GPIO1PIN_P12_PUT(v)          ( bFM_GPIO_PDOR1_P2=(v) )

#define GPIO1PIN_P12_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P12_INITIN(v) \
                                                        : GPIO1PIN_P12_INITOUT(v) )

#define GPIO1PIN_P12_INITIN(v)       do{ bFM_GPIO_PCR1_P2=(v).bPullup; \
                                         bFM_GPIO_DDR1_P2=0u; \
                                         bFM_GPIO_PFR1_P2=0u; \
                                         bFM_GPIO_ADE_AN02=0u; }while(0u)


#define GPIO1PIN_P12_INITOUT(v)      do{ bFM_GPIO_PDOR1_P2=(v).bInitVal; \
                                         bFM_GPIO_DDR1_P2=1u; \
                                         bFM_GPIO_PFR1_P2=0u; \
                                         bFM_GPIO_ADE_AN02=0u; }while(0u)


/*---- GPIO bit NP12 ----*/
#define GPIO1PIN_NP12_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR1_P2)) )

#define GPIO1PIN_NP12_PUT(v)         ( bFM_GPIO_PDOR1_P2=(uint32_t)(!(v)) )

#define GPIO1PIN_NP12_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP12_INITIN(v) \
                                                        : GPIO1PIN_NP12_INITOUT(v) )

#define GPIO1PIN_NP12_INITIN(v)      do{ bFM_GPIO_PCR1_P2=(v).bPullup; \
                                         bFM_GPIO_DDR1_P2=0u; \
                                         bFM_GPIO_PFR1_P2=0u; \
                                         bFM_GPIO_ADE_AN02=0u; }while(0u)


#define GPIO1PIN_NP12_INITOUT(v)     do{ bFM_GPIO_PDOR1_P2=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR1_P2=1u; \
                                         bFM_GPIO_PFR1_P2=0u; \
                                         bFM_GPIO_ADE_AN02=0u; }while(0u)


/*---- GPIO bit P13 ----*/

#define GPIO1PIN_P13_GET             ( bFM_GPIO_PDIR1_P3 )

#define GPIO1PIN_P13_PUT(v)          ( bFM_GPIO_PDOR1_P3=(v) )

#define GPIO1PIN_P13_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P13_INITIN(v) \
                                                        : GPIO1PIN_P13_INITOUT(v) )

#define GPIO1PIN_P13_INITIN(v)       do{ bFM_GPIO_PCR1_P3=(v).bPullup; \
                                         bFM_GPIO_DDR1_P3=0u; \
                                         bFM_GPIO_PFR1_P3=0u; \
                                         bFM_GPIO_ADE_AN03=0u; }while(0u)


#define GPIO1PIN_P13_INITOUT(v)      do{ bFM_GPIO_PDOR1_P3=(v).bInitVal; \
                                         bFM_GPIO_DDR1_P3=1u; \
                                         bFM_GPIO_PFR1_P3=0u; \
                                         bFM_GPIO_ADE_AN03=0u; }while(0u)


/*---- GPIO bit NP13 ----*/
#define GPIO1PIN_NP13_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR1_P3)) )

#define GPIO1PIN_NP13_PUT(v)         ( bFM_GPIO_PDOR1_P3=(uint32_t)(!(v)) )

#define GPIO1PIN_NP13_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP13_INITIN(v) \
                                                        : GPIO1PIN_NP13_INITOUT(v) )

#define GPIO1PIN_NP13_INITIN(v)      do{ bFM_GPIO_PCR1_P3=(v).bPullup; \
                                         bFM_GPIO_DDR1_P3=0u; \
                                         bFM_GPIO_PFR1_P3=0u; \
                                         bFM_GPIO_ADE_AN03=0u; }while(0u)


#define GPIO1PIN_NP13_INITOUT(v)     do{ bFM_GPIO_PDOR1_P3=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR1_P3=1u; \
                                         bFM_GPIO_PFR1_P3=0u; \
                                         bFM_GPIO_ADE_AN03=0u; }while(0u)


/*---- GPIO bit P14 ----*/

#define GPIO1PIN_P14_GET             ( bFM_GPIO_PDIR1_P4 )

#define GPIO1PIN_P14_PUT(v)          ( bFM_GPIO_PDOR1_P4=(v) )

#define GPIO1PIN_P14_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P14_INITIN(v) \
                                                        : GPIO1PIN_P14_INITOUT(v) )

#define GPIO1PIN_P14_INITIN(v)       do{ bFM_GPIO_PCR1_P4=(v).bPullup; \
                                         bFM_GPIO_DDR1_P4=0u; \
                                         bFM_GPIO_PFR1_P4=0u; \
                                         bFM_GPIO_ADE_AN04=0u; }while(0u)


#define GPIO1PIN_P14_INITOUT(v)      do{ bFM_GPIO_PDOR1_P4=(v).bInitVal; \
                                         bFM_GPIO_DDR1_P4=1u; \
                                         bFM_GPIO_PFR1_P4=0u; \
                                         bFM_GPIO_ADE_AN04=0u; }while(0u)


/*---- GPIO bit NP14 ----*/
#define GPIO1PIN_NP14_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR1_P4)) )

#define GPIO1PIN_NP14_PUT(v)         ( bFM_GPIO_PDOR1_P4=(uint32_t)(!(v)) )

#define GPIO1PIN_NP14_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP14_INITIN(v) \
                                                        : GPIO1PIN_NP14_INITOUT(v) )

#define GPIO1PIN_NP14_INITIN(v)      do{ bFM_GPIO_PCR1_P4=(v).bPullup; \
                                         bFM_GPIO_DDR1_P4=0u; \
                                         bFM_GPIO_PFR1_P4=0u; \
                                         bFM_GPIO_ADE_AN04=0u; }while(0u)


#define GPIO1PIN_NP14_INITOUT(v)     do{ bFM_GPIO_PDOR1_P4=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR1_P4=1u; \
                                         bFM_GPIO_PFR1_P4=0u; \
                                         bFM_GPIO_ADE_AN04=0u; }while(0u)


/*---- GPIO bit P15 ----*/

#define GPIO1PIN_P15_GET             ( bFM_GPIO_PDIR1_P5 )

#define GPIO1PIN_P15_PUT(v)          ( bFM_GPIO_PDOR1_P5=(v) )

#define GPIO1PIN_P15_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P15_INITIN(v) \
                                                        : GPIO1PIN_P15_INITOUT(v) )

#define GPIO1PIN_P15_INITIN(v)       do{ bFM_GPIO_PCR1_P5=(v).bPullup; \
                                         bFM_GPIO_DDR1_P5=0u; \
                                         bFM_GPIO_PFR1_P5=0u; \
                                         bFM_GPIO_ADE_AN05=0u; }while(0u)


#define GPIO1PIN_P15_INITOUT(v)      do{ bFM_GPIO_PDOR1_P5=(v).bInitVal; \
                                         bFM_GPIO_DDR1_P5=1u; \
                                         bFM_GPIO_PFR1_P5=0u; \
                                         bFM_GPIO_ADE_AN05=0u; }while(0u)


/*---- GPIO bit NP15 ----*/
#define GPIO1PIN_NP15_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR1_P5)) )

#define GPIO1PIN_NP15_PUT(v)         ( bFM_GPIO_PDOR1_P5=(uint32_t)(!(v)) )

#define GPIO1PIN_NP15_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP15_INITIN(v) \
                                                        : GPIO1PIN_NP15_INITOUT(v) )

#define GPIO1PIN_NP15_INITIN(v)      do{ bFM_GPIO_PCR1_P5=(v).bPullup; \
                                         bFM_GPIO_DDR1_P5=0u; \
                                         bFM_GPIO_PFR1_P5=0u; \
                                         bFM_GPIO_ADE_AN05=0u; }while(0u)


#define GPIO1PIN_NP15_INITOUT(v)     do{ bFM_GPIO_PDOR1_P5=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR1_P5=1u; \
                                         bFM_GPIO_PFR1_P5=0u; \
                                         bFM_GPIO_ADE_AN05=0u; }while(0u)


/*---- GPIO bit P1A ----*/

#define GPIO1PIN_P1A_GET             ( bFM_GPIO_PDIR1_PA )

#define GPIO1PIN_P1A_PUT(v)          ( bFM_GPIO_PDOR1_PA=(v) )

#define GPIO1PIN_P1A_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P1A_INITIN(v) \
                                                        : GPIO1PIN_P1A_INITOUT(v) )

#define GPIO1PIN_P1A_INITIN(v)       do{ bFM_GPIO_PCR1_PA=(v).bPullup; \
                                         bFM_GPIO_DDR1_PA=0u; \
                                         bFM_GPIO_PFR1_PA=0u; }while(0u)

#define GPIO1PIN_P1A_INITOUT(v)      do{ bFM_GPIO_PDOR1_PA=(v).bInitVal; \
                                         bFM_GPIO_DDR1_PA=1u; \
                                         bFM_GPIO_PFR1_PA=0u; }while(0u)


/*---- GPIO bit NP1A ----*/
#define GPIO1PIN_NP1A_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR1_PA)) )

#define GPIO1PIN_NP1A_PUT(v)         ( bFM_GPIO_PDOR1_PA=(uint32_t)(!(v)) )

#define GPIO1PIN_NP1A_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP1A_INITIN(v) \
                                                        : GPIO1PIN_NP1A_INITOUT(v) )

#define GPIO1PIN_NP1A_INITIN(v)      do{ bFM_GPIO_PCR1_PA=(v).bPullup; \
                                         bFM_GPIO_DDR1_PA=0u; \
                                         bFM_GPIO_PFR1_PA=0u; }while(0u)

#define GPIO1PIN_NP1A_INITOUT(v)     do{ bFM_GPIO_PDOR1_PA=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR1_PA=1u; \
                                         bFM_GPIO_PFR1_PA=0u; }while(0u)


/*---- GPIO bit P1B ----*/

#define GPIO1PIN_P1B_GET             ( bFM_GPIO_PDIR1_PB )

#define GPIO1PIN_P1B_PUT(v)          ( bFM_GPIO_PDOR1_PB=(v) )

#define GPIO1PIN_P1B_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P1B_INITIN(v) \
                                                        : GPIO1PIN_P1B_INITOUT(v) )

#define GPIO1PIN_P1B_INITIN(v)       do{ bFM_GPIO_PCR1_PB=(v).bPullup; \
                                         bFM_GPIO_DDR1_PB=0u; \
                                         bFM_GPIO_PFR1_PB=0u; }while(0u)

#define GPIO1PIN_P1B_INITOUT(v)      do{ bFM_GPIO_PDOR1_PB=(v).bInitVal; \
                                         bFM_GPIO_DDR1_PB=1u; \
                                         bFM_GPIO_PFR1_PB=0u; }while(0u)


/*---- GPIO bit NP1B ----*/
#define GPIO1PIN_NP1B_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR1_PB)) )

#define GPIO1PIN_NP1B_PUT(v)         ( bFM_GPIO_PDOR1_PB=(uint32_t)(!(v)) )

#define GPIO1PIN_NP1B_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP1B_INITIN(v) \
                                                        : GPIO1PIN_NP1B_INITOUT(v) )

#define GPIO1PIN_NP1B_INITIN(v)      do{ bFM_GPIO_PCR1_PB=(v).bPullup; \
                                         bFM_GPIO_DDR1_PB=0u; \
                                         bFM_GPIO_PFR1_PB=0u; }while(0u)

#define GPIO1PIN_NP1B_INITOUT(v)     do{ bFM_GPIO_PDOR1_PB=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR1_PB=1u; \
                                         bFM_GPIO_PFR1_PB=0u; }while(0u)


/*---- GPIO bit P1C ----*/

#define GPIO1PIN_P1C_GET             ( bFM_GPIO_PDIR1_PC )

#define GPIO1PIN_P1C_PUT(v)          ( bFM_GPIO_PDOR1_PC=(v) )

#define GPIO1PIN_P1C_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P1C_INITIN(v) \
                                                        : GPIO1PIN_P1C_INITOUT(v) )

#define GPIO1PIN_P1C_INITIN(v)       do{ bFM_GPIO_PCR1_PC=(v).bPullup; \
                                         bFM_GPIO_DDR1_PC=0u; \
                                         bFM_GPIO_PFR1_PC=0u; }while(0u)

#define GPIO1PIN_P1C_INITOUT(v)      do{ bFM_GPIO_PDOR1_PC=(v).bInitVal; \
                                         bFM_GPIO_DDR1_PC=1u; \
                                         bFM_GPIO_PFR1_PC=0u; }while(0u)


/*---- GPIO bit NP1C ----*/
#define GPIO1PIN_NP1C_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR1_PC)) )

#define GPIO1PIN_NP1C_PUT(v)         ( bFM_GPIO_PDOR1_PC=(uint32_t)(!(v)) )

#define GPIO1PIN_NP1C_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP1C_INITIN(v) \
                                                        : GPIO1PIN_NP1C_INITOUT(v) )

#define GPIO1PIN_NP1C_INITIN(v)      do{ bFM_GPIO_PCR1_PC=(v).bPullup; \
                                         bFM_GPIO_DDR1_PC=0u; \
                                         bFM_GPIO_PFR1_PC=0u; }while(0u)

#define GPIO1PIN_NP1C_INITOUT(v)     do{ bFM_GPIO_PDOR1_PC=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR1_PC=1u; \
                                         bFM_GPIO_PFR1_PC=0u; }while(0u)


/*---- GPIO bit P1D ----*/

#define GPIO1PIN_P1D_GET             ( bFM_GPIO_PDIR1_PD )

#define GPIO1PIN_P1D_PUT(v)          ( bFM_GPIO_PDOR1_PD=(v) )

#define GPIO1PIN_P1D_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P1D_INITIN(v) \
                                                        : GPIO1PIN_P1D_INITOUT(v) )

#define GPIO1PIN_P1D_INITIN(v)       do{ bFM_GPIO_PCR1_PD=(v).bPullup; \
                                         bFM_GPIO_DDR1_PD=0u; \
                                         bFM_GPIO_PFR1_PD=0u; }while(0u)

#define GPIO1PIN_P1D_INITOUT(v)      do{ bFM_GPIO_PDOR1_PD=(v).bInitVal; \
                                         bFM_GPIO_DDR1_PD=1u; \
                                         bFM_GPIO_PFR1_PD=0u; }while(0u)


/*---- GPIO bit NP1D ----*/
#define GPIO1PIN_NP1D_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR1_PD)) )

#define GPIO1PIN_NP1D_PUT(v)         ( bFM_GPIO_PDOR1_PD=(uint32_t)(!(v)) )

#define GPIO1PIN_NP1D_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP1D_INITIN(v) \
                                                        : GPIO1PIN_NP1D_INITOUT(v) )

#define GPIO1PIN_NP1D_INITIN(v)      do{ bFM_GPIO_PCR1_PD=(v).bPullup; \
                                         bFM_GPIO_DDR1_PD=0u; \
                                         bFM_GPIO_PFR1_PD=0u; }while(0u)

#define GPIO1PIN_NP1D_INITOUT(v)     do{ bFM_GPIO_PDOR1_PD=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR1_PD=1u; \
                                         bFM_GPIO_PFR1_PD=0u; }while(0u)


/*---- GPIO bit P1E ----*/

#define GPIO1PIN_P1E_GET             ( bFM_GPIO_PDIR1_PE )

#define GPIO1PIN_P1E_PUT(v)          ( bFM_GPIO_PDOR1_PE=(v) )

#define GPIO1PIN_P1E_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P1E_INITIN(v) \
                                                        : GPIO1PIN_P1E_INITOUT(v) )

#define GPIO1PIN_P1E_INITIN(v)       do{ bFM_GPIO_PCR1_PE=(v).bPullup; \
                                         bFM_GPIO_DDR1_PE=0u; \
                                         bFM_GPIO_PFR1_PE=0u; }while(0u)

#define GPIO1PIN_P1E_INITOUT(v)      do{ bFM_GPIO_PDOR1_PE=(v).bInitVal; \
                                         bFM_GPIO_DDR1_PE=1u; \
                                         bFM_GPIO_PFR1_PE=0u; }while(0u)


/*---- GPIO bit NP1E ----*/
#define GPIO1PIN_NP1E_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR1_PE)) )

#define GPIO1PIN_NP1E_PUT(v)         ( bFM_GPIO_PDOR1_PE=(uint32_t)(!(v)) )

#define GPIO1PIN_NP1E_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP1E_INITIN(v) \
                                                        : GPIO1PIN_NP1E_INITOUT(v) )

#define GPIO1PIN_NP1E_INITIN(v)      do{ bFM_GPIO_PCR1_PE=(v).bPullup; \
                                         bFM_GPIO_DDR1_PE=0u; \
                                         bFM_GPIO_PFR1_PE=0u; }while(0u)

#define GPIO1PIN_NP1E_INITOUT(v)     do{ bFM_GPIO_PDOR1_PE=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR1_PE=1u; \
                                         bFM_GPIO_PFR1_PE=0u; }while(0u)


/*---- GPIO bit P1F ----*/

#define GPIO1PIN_P1F_GET             ( bFM_GPIO_PDIR1_PF )

#define GPIO1PIN_P1F_PUT(v)          ( bFM_GPIO_PDOR1_PF=(v) )

#define GPIO1PIN_P1F_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P1F_INITIN(v) \
                                                        : GPIO1PIN_P1F_INITOUT(v) )

#define GPIO1PIN_P1F_INITIN(v)       do{ bFM_GPIO_PCR1_PF=(v).bPullup; \
                                         bFM_GPIO_DDR1_PF=0u; \
                                         bFM_GPIO_PFR1_PF=0u; }while(0u)

#define GPIO1PIN_P1F_INITOUT(v)      do{ bFM_GPIO_PDOR1_PF=(v).bInitVal; \
                                         bFM_GPIO_DDR1_PF=1u; \
                                         bFM_GPIO_PFR1_PF=0u; }while(0u)


/*---- GPIO bit NP1F ----*/
#define GPIO1PIN_NP1F_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR1_PF)) )

#define GPIO1PIN_NP1F_PUT(v)         ( bFM_GPIO_PDOR1_PF=(uint32_t)(!(v)) )

#define GPIO1PIN_NP1F_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP1F_INITIN(v) \
                                                        : GPIO1PIN_NP1F_INITOUT(v) )

#define GPIO1PIN_NP1F_INITIN(v)      do{ bFM_GPIO_PCR1_PF=(v).bPullup; \
                                         bFM_GPIO_DDR1_PF=0u; \
                                         bFM_GPIO_PFR1_PF=0u; }while(0u)

#define GPIO1PIN_NP1F_INITOUT(v)     do{ bFM_GPIO_PDOR1_PF=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR1_PF=1u; \
                                         bFM_GPIO_PFR1_PF=0u; }while(0u)


/*---- GPIO bit P21 ----*/

#define GPIO1PIN_P21_GET             ( bFM_GPIO_PDIR2_P1 )

#define GPIO1PIN_P21_PUT(v)          ( bFM_GPIO_PDOR2_P1=(v) )

#define GPIO1PIN_P21_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P21_INITIN(v) \
                                                        : GPIO1PIN_P21_INITOUT(v) )

#define GPIO1PIN_P21_INITIN(v)       do{ bFM_GPIO_PCR2_P1=(v).bPullup; \
                                         bFM_GPIO_DDR2_P1=0u; \
                                         bFM_GPIO_PFR2_P1=0u; }while(0u)

#define GPIO1PIN_P21_INITOUT(v)      do{ bFM_GPIO_PDOR2_P1=(v).bInitVal; \
                                         bFM_GPIO_DDR2_P1=1u; \
                                         bFM_GPIO_PFR2_P1=0u; }while(0u)


/*---- GPIO bit NP21 ----*/
#define GPIO1PIN_NP21_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR2_P1)) )

#define GPIO1PIN_NP21_PUT(v)         ( bFM_GPIO_PDOR2_P1=(uint32_t)(!(v)) )

#define GPIO1PIN_NP21_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP21_INITIN(v) \
                                                        : GPIO1PIN_NP21_INITOUT(v) )

#define GPIO1PIN_NP21_INITIN(v)      do{ bFM_GPIO_PCR2_P1=(v).bPullup; \
                                         bFM_GPIO_DDR2_P1=0u; \
                                         bFM_GPIO_PFR2_P1=0u; }while(0u)

#define GPIO1PIN_NP21_INITOUT(v)     do{ bFM_GPIO_PDOR2_P1=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR2_P1=1u; \
                                         bFM_GPIO_PFR2_P1=0u; }while(0u)


/*---- GPIO bit P22 ----*/

#define GPIO1PIN_P22_GET             ( bFM_GPIO_PDIR2_P2 )

#define GPIO1PIN_P22_PUT(v)          ( bFM_GPIO_PDOR2_P2=(v) )

#define GPIO1PIN_P22_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P22_INITIN(v) \
                                                        : GPIO1PIN_P22_INITOUT(v) )

#define GPIO1PIN_P22_INITIN(v)       do{ bFM_GPIO_PCR2_P2=(v).bPullup; \
                                         bFM_GPIO_DDR2_P2=0u; \
                                         bFM_GPIO_PFR2_P2=0u; \
                                         bFM_GPIO_ADE_AN07=0u; }while(0u)


#define GPIO1PIN_P22_INITOUT(v)      do{ bFM_GPIO_PDOR2_P2=(v).bInitVal; \
                                         bFM_GPIO_DDR2_P2=1u; \
                                         bFM_GPIO_PFR2_P2=0u; \
                                         bFM_GPIO_ADE_AN07=0u; }while(0u)


/*---- GPIO bit NP22 ----*/
#define GPIO1PIN_NP22_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR2_P2)) )

#define GPIO1PIN_NP22_PUT(v)         ( bFM_GPIO_PDOR2_P2=(uint32_t)(!(v)) )

#define GPIO1PIN_NP22_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP22_INITIN(v) \
                                                        : GPIO1PIN_NP22_INITOUT(v) )

#define GPIO1PIN_NP22_INITIN(v)      do{ bFM_GPIO_PCR2_P2=(v).bPullup; \
                                         bFM_GPIO_DDR2_P2=0u; \
                                         bFM_GPIO_PFR2_P2=0u; \
                                         bFM_GPIO_ADE_AN07=0u; }while(0u)


#define GPIO1PIN_NP22_INITOUT(v)     do{ bFM_GPIO_PDOR2_P2=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR2_P2=1u; \
                                         bFM_GPIO_PFR2_P2=0u; \
                                         bFM_GPIO_ADE_AN07=0u; }while(0u)


/*---- GPIO bit P23 ----*/

#define GPIO1PIN_P23_GET             ( bFM_GPIO_PDIR2_P3 )

#define GPIO1PIN_P23_PUT(v)          ( bFM_GPIO_PDOR2_P3=(v) )

#define GPIO1PIN_P23_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P23_INITIN(v) \
                                                        : GPIO1PIN_P23_INITOUT(v) )

#define GPIO1PIN_P23_INITIN(v)       do{ bFM_GPIO_PCR2_P3=(v).bPullup; \
                                         bFM_GPIO_DDR2_P3=0u; \
                                         bFM_GPIO_PFR2_P3=0u; \
                                         bFM_GPIO_ADE_AN06=0u; }while(0u)


#define GPIO1PIN_P23_INITOUT(v)      do{ bFM_GPIO_PDOR2_P3=(v).bInitVal; \
                                         bFM_GPIO_DDR2_P3=1u; \
                                         bFM_GPIO_PFR2_P3=0u; \
                                         bFM_GPIO_ADE_AN06=0u; }while(0u)


/*---- GPIO bit NP23 ----*/
#define GPIO1PIN_NP23_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR2_P3)) )

#define GPIO1PIN_NP23_PUT(v)         ( bFM_GPIO_PDOR2_P3=(uint32_t)(!(v)) )

#define GPIO1PIN_NP23_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP23_INITIN(v) \
                                                        : GPIO1PIN_NP23_INITOUT(v) )

#define GPIO1PIN_NP23_INITIN(v)      do{ bFM_GPIO_PCR2_P3=(v).bPullup; \
                                         bFM_GPIO_DDR2_P3=0u; \
                                         bFM_GPIO_PFR2_P3=0u; \
                                         bFM_GPIO_ADE_AN06=0u; }while(0u)


#define GPIO1PIN_NP23_INITOUT(v)     do{ bFM_GPIO_PDOR2_P3=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR2_P3=1u; \
                                         bFM_GPIO_PFR2_P3=0u; \
                                         bFM_GPIO_ADE_AN06=0u; }while(0u)


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


/*---- GPIO bit P3A ----*/

#define GPIO1PIN_P3A_GET             ( bFM_GPIO_PDIR3_PA )

#define GPIO1PIN_P3A_PUT(v)          ( bFM_GPIO_PDOR3_PA=(v) )

#define GPIO1PIN_P3A_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P3A_INITIN(v) \
                                                        : GPIO1PIN_P3A_INITOUT(v) )

#define GPIO1PIN_P3A_INITIN(v)       do{ bFM_GPIO_PCR3_PA=(v).bPullup; \
                                         bFM_GPIO_DDR3_PA=0u; \
                                         bFM_GPIO_PFR3_PA=0u; }while(0u)

#define GPIO1PIN_P3A_INITOUT(v)      do{ bFM_GPIO_PDOR3_PA=(v).bInitVal; \
                                         bFM_GPIO_DDR3_PA=1u; \
                                         bFM_GPIO_PFR3_PA=0u; }while(0u)


/*---- GPIO bit NP3A ----*/
#define GPIO1PIN_NP3A_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR3_PA)) )

#define GPIO1PIN_NP3A_PUT(v)         ( bFM_GPIO_PDOR3_PA=(uint32_t)(!(v)) )

#define GPIO1PIN_NP3A_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP3A_INITIN(v) \
                                                        : GPIO1PIN_NP3A_INITOUT(v) )

#define GPIO1PIN_NP3A_INITIN(v)      do{ bFM_GPIO_PCR3_PA=(v).bPullup; \
                                         bFM_GPIO_DDR3_PA=0u; \
                                         bFM_GPIO_PFR3_PA=0u; }while(0u)

#define GPIO1PIN_NP3A_INITOUT(v)     do{ bFM_GPIO_PDOR3_PA=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR3_PA=1u; \
                                         bFM_GPIO_PFR3_PA=0u; }while(0u)


/*---- GPIO bit P3B ----*/

#define GPIO1PIN_P3B_GET             ( bFM_GPIO_PDIR3_PB )

#define GPIO1PIN_P3B_PUT(v)          ( bFM_GPIO_PDOR3_PB=(v) )

#define GPIO1PIN_P3B_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P3B_INITIN(v) \
                                                        : GPIO1PIN_P3B_INITOUT(v) )

#define GPIO1PIN_P3B_INITIN(v)       do{ bFM_GPIO_PCR3_PB=(v).bPullup; \
                                         bFM_GPIO_DDR3_PB=0u; \
                                         bFM_GPIO_PFR3_PB=0u; }while(0u)

#define GPIO1PIN_P3B_INITOUT(v)      do{ bFM_GPIO_PDOR3_PB=(v).bInitVal; \
                                         bFM_GPIO_DDR3_PB=1u; \
                                         bFM_GPIO_PFR3_PB=0u; }while(0u)


/*---- GPIO bit NP3B ----*/
#define GPIO1PIN_NP3B_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR3_PB)) )

#define GPIO1PIN_NP3B_PUT(v)         ( bFM_GPIO_PDOR3_PB=(uint32_t)(!(v)) )

#define GPIO1PIN_NP3B_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP3B_INITIN(v) \
                                                        : GPIO1PIN_NP3B_INITOUT(v) )

#define GPIO1PIN_NP3B_INITIN(v)      do{ bFM_GPIO_PCR3_PB=(v).bPullup; \
                                         bFM_GPIO_DDR3_PB=0u; \
                                         bFM_GPIO_PFR3_PB=0u; }while(0u)

#define GPIO1PIN_NP3B_INITOUT(v)     do{ bFM_GPIO_PDOR3_PB=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR3_PB=1u; \
                                         bFM_GPIO_PFR3_PB=0u; }while(0u)


/*---- GPIO bit P3C ----*/

#define GPIO1PIN_P3C_GET             ( bFM_GPIO_PDIR3_PC )

#define GPIO1PIN_P3C_PUT(v)          ( bFM_GPIO_PDOR3_PC=(v) )

#define GPIO1PIN_P3C_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P3C_INITIN(v) \
                                                        : GPIO1PIN_P3C_INITOUT(v) )

#define GPIO1PIN_P3C_INITIN(v)       do{ bFM_GPIO_PCR3_PC=(v).bPullup; \
                                         bFM_GPIO_DDR3_PC=0u; \
                                         bFM_GPIO_PFR3_PC=0u; }while(0u)

#define GPIO1PIN_P3C_INITOUT(v)      do{ bFM_GPIO_PDOR3_PC=(v).bInitVal; \
                                         bFM_GPIO_DDR3_PC=1u; \
                                         bFM_GPIO_PFR3_PC=0u; }while(0u)


/*---- GPIO bit NP3C ----*/
#define GPIO1PIN_NP3C_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR3_PC)) )

#define GPIO1PIN_NP3C_PUT(v)         ( bFM_GPIO_PDOR3_PC=(uint32_t)(!(v)) )

#define GPIO1PIN_NP3C_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP3C_INITIN(v) \
                                                        : GPIO1PIN_NP3C_INITOUT(v) )

#define GPIO1PIN_NP3C_INITIN(v)      do{ bFM_GPIO_PCR3_PC=(v).bPullup; \
                                         bFM_GPIO_DDR3_PC=0u; \
                                         bFM_GPIO_PFR3_PC=0u; }while(0u)

#define GPIO1PIN_NP3C_INITOUT(v)     do{ bFM_GPIO_PDOR3_PC=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR3_PC=1u; \
                                         bFM_GPIO_PFR3_PC=0u; }while(0u)


/*---- GPIO bit P3D ----*/

#define GPIO1PIN_P3D_GET             ( bFM_GPIO_PDIR3_PD )

#define GPIO1PIN_P3D_PUT(v)          ( bFM_GPIO_PDOR3_PD=(v) )

#define GPIO1PIN_P3D_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P3D_INITIN(v) \
                                                        : GPIO1PIN_P3D_INITOUT(v) )

#define GPIO1PIN_P3D_INITIN(v)       do{ bFM_GPIO_PCR3_PD=(v).bPullup; \
                                         bFM_GPIO_DDR3_PD=0u; \
                                         bFM_GPIO_PFR3_PD=0u; }while(0u)

#define GPIO1PIN_P3D_INITOUT(v)      do{ bFM_GPIO_PDOR3_PD=(v).bInitVal; \
                                         bFM_GPIO_DDR3_PD=1u; \
                                         bFM_GPIO_PFR3_PD=0u; }while(0u)


/*---- GPIO bit NP3D ----*/
#define GPIO1PIN_NP3D_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR3_PD)) )

#define GPIO1PIN_NP3D_PUT(v)         ( bFM_GPIO_PDOR3_PD=(uint32_t)(!(v)) )

#define GPIO1PIN_NP3D_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP3D_INITIN(v) \
                                                        : GPIO1PIN_NP3D_INITOUT(v) )

#define GPIO1PIN_NP3D_INITIN(v)      do{ bFM_GPIO_PCR3_PD=(v).bPullup; \
                                         bFM_GPIO_DDR3_PD=0u; \
                                         bFM_GPIO_PFR3_PD=0u; }while(0u)

#define GPIO1PIN_NP3D_INITOUT(v)     do{ bFM_GPIO_PDOR3_PD=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR3_PD=1u; \
                                         bFM_GPIO_PFR3_PD=0u; }while(0u)


/*---- GPIO bit P3E ----*/

#define GPIO1PIN_P3E_GET             ( bFM_GPIO_PDIR3_PE )

#define GPIO1PIN_P3E_PUT(v)          ( bFM_GPIO_PDOR3_PE=(v) )

#define GPIO1PIN_P3E_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P3E_INITIN(v) \
                                                        : GPIO1PIN_P3E_INITOUT(v) )

#define GPIO1PIN_P3E_INITIN(v)       do{ bFM_GPIO_PCR3_PE=(v).bPullup; \
                                         bFM_GPIO_DDR3_PE=0u; \
                                         bFM_GPIO_PFR3_PE=0u; }while(0u)

#define GPIO1PIN_P3E_INITOUT(v)      do{ bFM_GPIO_PDOR3_PE=(v).bInitVal; \
                                         bFM_GPIO_DDR3_PE=1u; \
                                         bFM_GPIO_PFR3_PE=0u; }while(0u)


/*---- GPIO bit NP3E ----*/
#define GPIO1PIN_NP3E_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR3_PE)) )

#define GPIO1PIN_NP3E_PUT(v)         ( bFM_GPIO_PDOR3_PE=(uint32_t)(!(v)) )

#define GPIO1PIN_NP3E_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP3E_INITIN(v) \
                                                        : GPIO1PIN_NP3E_INITOUT(v) )

#define GPIO1PIN_NP3E_INITIN(v)      do{ bFM_GPIO_PCR3_PE=(v).bPullup; \
                                         bFM_GPIO_DDR3_PE=0u; \
                                         bFM_GPIO_PFR3_PE=0u; }while(0u)

#define GPIO1PIN_NP3E_INITOUT(v)     do{ bFM_GPIO_PDOR3_PE=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR3_PE=1u; \
                                         bFM_GPIO_PFR3_PE=0u; }while(0u)


/*---- GPIO bit P3F ----*/

#define GPIO1PIN_P3F_GET             ( bFM_GPIO_PDIR3_PF )

#define GPIO1PIN_P3F_PUT(v)          ( bFM_GPIO_PDOR3_PF=(v) )

#define GPIO1PIN_P3F_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P3F_INITIN(v) \
                                                        : GPIO1PIN_P3F_INITOUT(v) )

#define GPIO1PIN_P3F_INITIN(v)       do{ bFM_GPIO_PCR3_PF=(v).bPullup; \
                                         bFM_GPIO_DDR3_PF=0u; \
                                         bFM_GPIO_PFR3_PF=0u; }while(0u)

#define GPIO1PIN_P3F_INITOUT(v)      do{ bFM_GPIO_PDOR3_PF=(v).bInitVal; \
                                         bFM_GPIO_DDR3_PF=1u; \
                                         bFM_GPIO_PFR3_PF=0u; }while(0u)


/*---- GPIO bit NP3F ----*/
#define GPIO1PIN_NP3F_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR3_PF)) )

#define GPIO1PIN_NP3F_PUT(v)         ( bFM_GPIO_PDOR3_PF=(uint32_t)(!(v)) )

#define GPIO1PIN_NP3F_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP3F_INITIN(v) \
                                                        : GPIO1PIN_NP3F_INITOUT(v) )

#define GPIO1PIN_NP3F_INITIN(v)      do{ bFM_GPIO_PCR3_PF=(v).bPullup; \
                                         bFM_GPIO_DDR3_PF=0u; \
                                         bFM_GPIO_PFR3_PF=0u; }while(0u)

#define GPIO1PIN_NP3F_INITOUT(v)     do{ bFM_GPIO_PDOR3_PF=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR3_PF=1u; \
                                         bFM_GPIO_PFR3_PF=0u; }while(0u)


/*---- GPIO bit P40 ----*/

#define GPIO1PIN_P40_GET             ( bFM_GPIO_PDIR4_P0 )

#define GPIO1PIN_P40_PUT(v)          ( bFM_GPIO_PDOR4_P0=(v) )

#define GPIO1PIN_P40_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P40_INITIN(v) \
                                                        : GPIO1PIN_P40_INITOUT(v) )

#define GPIO1PIN_P40_INITIN(v)       do{ bFM_GPIO_PCR4_P0=(v).bPullup; \
                                         bFM_GPIO_DDR4_P0=0u; \
                                         bFM_GPIO_PFR4_P0=0u; }while(0u)

#define GPIO1PIN_P40_INITOUT(v)      do{ bFM_GPIO_PDOR4_P0=(v).bInitVal; \
                                         bFM_GPIO_DDR4_P0=1u; \
                                         bFM_GPIO_PFR4_P0=0u; }while(0u)


/*---- GPIO bit NP40 ----*/
#define GPIO1PIN_NP40_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR4_P0)) )

#define GPIO1PIN_NP40_PUT(v)         ( bFM_GPIO_PDOR4_P0=(uint32_t)(!(v)) )

#define GPIO1PIN_NP40_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP40_INITIN(v) \
                                                        : GPIO1PIN_NP40_INITOUT(v) )

#define GPIO1PIN_NP40_INITIN(v)      do{ bFM_GPIO_PCR4_P0=(v).bPullup; \
                                         bFM_GPIO_DDR4_P0=0u; \
                                         bFM_GPIO_PFR4_P0=0u; }while(0u)

#define GPIO1PIN_NP40_INITOUT(v)     do{ bFM_GPIO_PDOR4_P0=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR4_P0=1u; \
                                         bFM_GPIO_PFR4_P0=0u; }while(0u)


/*---- GPIO bit P41 ----*/

#define GPIO1PIN_P41_GET             ( bFM_GPIO_PDIR4_P1 )

#define GPIO1PIN_P41_PUT(v)          ( bFM_GPIO_PDOR4_P1=(v) )

#define GPIO1PIN_P41_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P41_INITIN(v) \
                                                        : GPIO1PIN_P41_INITOUT(v) )

#define GPIO1PIN_P41_INITIN(v)       do{ bFM_GPIO_PCR4_P1=(v).bPullup; \
                                         bFM_GPIO_DDR4_P1=0u; \
                                         bFM_GPIO_PFR4_P1=0u; }while(0u)

#define GPIO1PIN_P41_INITOUT(v)      do{ bFM_GPIO_PDOR4_P1=(v).bInitVal; \
                                         bFM_GPIO_DDR4_P1=1u; \
                                         bFM_GPIO_PFR4_P1=0u; }while(0u)


/*---- GPIO bit NP41 ----*/
#define GPIO1PIN_NP41_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR4_P1)) )

#define GPIO1PIN_NP41_PUT(v)         ( bFM_GPIO_PDOR4_P1=(uint32_t)(!(v)) )

#define GPIO1PIN_NP41_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP41_INITIN(v) \
                                                        : GPIO1PIN_NP41_INITOUT(v) )

#define GPIO1PIN_NP41_INITIN(v)      do{ bFM_GPIO_PCR4_P1=(v).bPullup; \
                                         bFM_GPIO_DDR4_P1=0u; \
                                         bFM_GPIO_PFR4_P1=0u; }while(0u)

#define GPIO1PIN_NP41_INITOUT(v)     do{ bFM_GPIO_PDOR4_P1=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR4_P1=1u; \
                                         bFM_GPIO_PFR4_P1=0u; }while(0u)


/*---- GPIO bit P42 ----*/

#define GPIO1PIN_P42_GET             ( bFM_GPIO_PDIR4_P2 )

#define GPIO1PIN_P42_PUT(v)          ( bFM_GPIO_PDOR4_P2=(v) )

#define GPIO1PIN_P42_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P42_INITIN(v) \
                                                        : GPIO1PIN_P42_INITOUT(v) )

#define GPIO1PIN_P42_INITIN(v)       do{ bFM_GPIO_PCR4_P2=(v).bPullup; \
                                         bFM_GPIO_DDR4_P2=0u; \
                                         bFM_GPIO_PFR4_P2=0u; }while(0u)

#define GPIO1PIN_P42_INITOUT(v)      do{ bFM_GPIO_PDOR4_P2=(v).bInitVal; \
                                         bFM_GPIO_DDR4_P2=1u; \
                                         bFM_GPIO_PFR4_P2=0u; }while(0u)


/*---- GPIO bit NP42 ----*/
#define GPIO1PIN_NP42_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR4_P2)) )

#define GPIO1PIN_NP42_PUT(v)         ( bFM_GPIO_PDOR4_P2=(uint32_t)(!(v)) )

#define GPIO1PIN_NP42_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP42_INITIN(v) \
                                                        : GPIO1PIN_NP42_INITOUT(v) )

#define GPIO1PIN_NP42_INITIN(v)      do{ bFM_GPIO_PCR4_P2=(v).bPullup; \
                                         bFM_GPIO_DDR4_P2=0u; \
                                         bFM_GPIO_PFR4_P2=0u; }while(0u)

#define GPIO1PIN_NP42_INITOUT(v)     do{ bFM_GPIO_PDOR4_P2=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR4_P2=1u; \
                                         bFM_GPIO_PFR4_P2=0u; }while(0u)


/*---- GPIO bit P43 ----*/

#define GPIO1PIN_P43_GET             ( bFM_GPIO_PDIR4_P3 )

#define GPIO1PIN_P43_PUT(v)          ( bFM_GPIO_PDOR4_P3=(v) )

#define GPIO1PIN_P43_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P43_INITIN(v) \
                                                        : GPIO1PIN_P43_INITOUT(v) )

#define GPIO1PIN_P43_INITIN(v)       do{ bFM_GPIO_PCR4_P3=(v).bPullup; \
                                         bFM_GPIO_DDR4_P3=0u; \
                                         bFM_GPIO_PFR4_P3=0u; }while(0u)

#define GPIO1PIN_P43_INITOUT(v)      do{ bFM_GPIO_PDOR4_P3=(v).bInitVal; \
                                         bFM_GPIO_DDR4_P3=1u; \
                                         bFM_GPIO_PFR4_P3=0u; }while(0u)


/*---- GPIO bit NP43 ----*/
#define GPIO1PIN_NP43_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR4_P3)) )

#define GPIO1PIN_NP43_PUT(v)         ( bFM_GPIO_PDOR4_P3=(uint32_t)(!(v)) )

#define GPIO1PIN_NP43_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP43_INITIN(v) \
                                                        : GPIO1PIN_NP43_INITOUT(v) )

#define GPIO1PIN_NP43_INITIN(v)      do{ bFM_GPIO_PCR4_P3=(v).bPullup; \
                                         bFM_GPIO_DDR4_P3=0u; \
                                         bFM_GPIO_PFR4_P3=0u; }while(0u)

#define GPIO1PIN_NP43_INITOUT(v)     do{ bFM_GPIO_PDOR4_P3=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR4_P3=1u; \
                                         bFM_GPIO_PFR4_P3=0u; }while(0u)


/*---- GPIO bit P46 ----*/

#define GPIO1PIN_P46_GET             ( bFM_GPIO_PDIR4_P6 )

#define GPIO1PIN_P46_PUT(v)          ( bFM_GPIO_PDOR4_P6=(v) )

#define GPIO1PIN_P46_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P46_INITIN(v) \
                                                        : GPIO1PIN_P46_INITOUT(v) )

#define GPIO1PIN_P46_INITIN(v)       do{ bFM_GPIO_PCR4_P6=(v).bPullup; \
                                         bFM_GPIO_DDR4_P6=0u; \
                                         bFM_GPIO_PFR4_P6=0u; }while(0u)

#define GPIO1PIN_P46_INITOUT(v)      do{ bFM_GPIO_PDOR4_P6=(v).bInitVal; \
                                         bFM_GPIO_DDR4_P6=1u; \
/*--- Notice: This function will configure both of P46/X0A and P47/X1A as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_DS->SUBOSC_CTL, 0u, 2u, 0x00u ); \
                                         bFM_GPIO_PFR4_P6=0u; }while(0u)


/*---- GPIO bit NP46 ----*/
#define GPIO1PIN_NP46_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR4_P6)) )

#define GPIO1PIN_NP46_PUT(v)         ( bFM_GPIO_PDOR4_P6=(uint32_t)(!(v)) )

#define GPIO1PIN_NP46_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP46_INITIN(v) \
                                                        : GPIO1PIN_NP46_INITOUT(v) )

#define GPIO1PIN_NP46_INITIN(v)      do{ bFM_GPIO_PCR4_P6=(v).bPullup; \
                                         bFM_GPIO_DDR4_P6=0u; \
/*--- Notice: This function will configure both of P46/X0A and P47/X1A as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_DS->SUBOSC_CTL, 0u, 2u, 0x00u ); \
                                         bFM_GPIO_PFR4_P6=0u; }while(0u)

#define GPIO1PIN_NP46_INITOUT(v)     do{ bFM_GPIO_PDOR4_P6=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR4_P6=1u; \
/*--- Notice: This function will configure both of P46/X0A and P47/X1A as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_DS->SUBOSC_CTL, 0u, 2u, 0x00u ); \
                                         bFM_GPIO_PFR4_P6=0u; }while(0u)


/*---- GPIO bit P47 ----*/

#define GPIO1PIN_P47_GET             ( bFM_GPIO_PDIR4_P7 )

#define GPIO1PIN_P47_PUT(v)          ( bFM_GPIO_PDOR4_P7=(v) )

#define GPIO1PIN_P47_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P47_INITIN(v) \
                                                        : GPIO1PIN_P47_INITOUT(v) )

#define GPIO1PIN_P47_INITIN(v)       do{ bFM_GPIO_PCR4_P7=(v).bPullup; \
                                         bFM_GPIO_DDR4_P7=0u; \
                                         bFM_GPIO_PFR4_P7=0u; }while(0u)

#define GPIO1PIN_P47_INITOUT(v)      do{ bFM_GPIO_PDOR4_P7=(v).bInitVal; \
                                         bFM_GPIO_DDR4_P7=1u; \
/*--- Notice: This function will configure both of P46/X0A and P47/X1A as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_DS->SUBOSC_CTL, 0u, 2u, 0x00u ); \
                                         bFM_GPIO_PFR4_P7=0u; }while(0u)


/*---- GPIO bit NP47 ----*/
#define GPIO1PIN_NP47_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR4_P7)) )

#define GPIO1PIN_NP47_PUT(v)         ( bFM_GPIO_PDOR4_P7=(uint32_t)(!(v)) )

#define GPIO1PIN_NP47_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP47_INITIN(v) \
                                                        : GPIO1PIN_NP47_INITOUT(v) )

#define GPIO1PIN_NP47_INITIN(v)      do{ bFM_GPIO_PCR4_P7=(v).bPullup; \
                                         bFM_GPIO_DDR4_P7=0u; \
/*--- Notice: This function will configure both of P46/X0A and P47/X1A as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_DS->SUBOSC_CTL, 0u, 2u, 0x00u ); \
                                         bFM_GPIO_PFR4_P7=0u; }while(0u)

#define GPIO1PIN_NP47_INITOUT(v)     do{ bFM_GPIO_PDOR4_P7=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR4_P7=1u; \
/*--- Notice: This function will configure both of P46/X0A and P47/X1A as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_DS->SUBOSC_CTL, 0u, 2u, 0x00u ); \
                                         bFM_GPIO_PFR4_P7=0u; }while(0u)


/*---- GPIO bit P4C ----*/

#define GPIO1PIN_P4C_GET             ( bFM_GPIO_PDIR4_PC )

#define GPIO1PIN_P4C_PUT(v)          ( bFM_GPIO_PDOR4_PC=(v) )

#define GPIO1PIN_P4C_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P4C_INITIN(v) \
                                                        : GPIO1PIN_P4C_INITOUT(v) )

#define GPIO1PIN_P4C_INITIN(v)       do{ bFM_GPIO_PCR4_PC=(v).bPullup; \
                                         bFM_GPIO_DDR4_PC=0u; \
                                         bFM_GPIO_PFR4_PC=0u; }while(0u)

#define GPIO1PIN_P4C_INITOUT(v)      do{ bFM_GPIO_PDOR4_PC=(v).bInitVal; \
                                         bFM_GPIO_DDR4_PC=1u; \
                                         bFM_GPIO_PFR4_PC=0u; }while(0u)


/*---- GPIO bit NP4C ----*/
#define GPIO1PIN_NP4C_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR4_PC)) )

#define GPIO1PIN_NP4C_PUT(v)         ( bFM_GPIO_PDOR4_PC=(uint32_t)(!(v)) )

#define GPIO1PIN_NP4C_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP4C_INITIN(v) \
                                                        : GPIO1PIN_NP4C_INITOUT(v) )

#define GPIO1PIN_NP4C_INITIN(v)      do{ bFM_GPIO_PCR4_PC=(v).bPullup; \
                                         bFM_GPIO_DDR4_PC=0u; \
                                         bFM_GPIO_PFR4_PC=0u; }while(0u)

#define GPIO1PIN_NP4C_INITOUT(v)     do{ bFM_GPIO_PDOR4_PC=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR4_PC=1u; \
                                         bFM_GPIO_PFR4_PC=0u; }while(0u)


/*---- GPIO bit P4D ----*/

#define GPIO1PIN_P4D_GET             ( bFM_GPIO_PDIR4_PD )

#define GPIO1PIN_P4D_PUT(v)          ( bFM_GPIO_PDOR4_PD=(v) )

#define GPIO1PIN_P4D_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P4D_INITIN(v) \
                                                        : GPIO1PIN_P4D_INITOUT(v) )

#define GPIO1PIN_P4D_INITIN(v)       do{ bFM_GPIO_PCR4_PD=(v).bPullup; \
                                         bFM_GPIO_DDR4_PD=0u; \
                                         bFM_GPIO_PFR4_PD=0u; }while(0u)

#define GPIO1PIN_P4D_INITOUT(v)      do{ bFM_GPIO_PDOR4_PD=(v).bInitVal; \
                                         bFM_GPIO_DDR4_PD=1u; \
                                         bFM_GPIO_PFR4_PD=0u; }while(0u)


/*---- GPIO bit NP4D ----*/
#define GPIO1PIN_NP4D_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR4_PD)) )

#define GPIO1PIN_NP4D_PUT(v)         ( bFM_GPIO_PDOR4_PD=(uint32_t)(!(v)) )

#define GPIO1PIN_NP4D_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP4D_INITIN(v) \
                                                        : GPIO1PIN_NP4D_INITOUT(v) )

#define GPIO1PIN_NP4D_INITIN(v)      do{ bFM_GPIO_PCR4_PD=(v).bPullup; \
                                         bFM_GPIO_DDR4_PD=0u; \
                                         bFM_GPIO_PFR4_PD=0u; }while(0u)

#define GPIO1PIN_NP4D_INITOUT(v)     do{ bFM_GPIO_PDOR4_PD=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR4_PD=1u; \
                                         bFM_GPIO_PFR4_PD=0u; }while(0u)


/*---- GPIO bit P4E ----*/

#define GPIO1PIN_P4E_GET             ( bFM_GPIO_PDIR4_PE )

#define GPIO1PIN_P4E_PUT(v)          ( bFM_GPIO_PDOR4_PE=(v) )

#define GPIO1PIN_P4E_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P4E_INITIN(v) \
                                                        : GPIO1PIN_P4E_INITOUT(v) )

#define GPIO1PIN_P4E_INITIN(v)       do{ bFM_GPIO_PCR4_PE=(v).bPullup; \
                                         bFM_GPIO_DDR4_PE=0u; \
                                         bFM_GPIO_PFR4_PE=0u; }while(0u)

#define GPIO1PIN_P4E_INITOUT(v)      do{ bFM_GPIO_PDOR4_PE=(v).bInitVal; \
                                         bFM_GPIO_DDR4_PE=1u; \
                                         bFM_GPIO_PFR4_PE=0u; }while(0u)


/*---- GPIO bit NP4E ----*/
#define GPIO1PIN_NP4E_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR4_PE)) )

#define GPIO1PIN_NP4E_PUT(v)         ( bFM_GPIO_PDOR4_PE=(uint32_t)(!(v)) )

#define GPIO1PIN_NP4E_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP4E_INITIN(v) \
                                                        : GPIO1PIN_NP4E_INITOUT(v) )

#define GPIO1PIN_NP4E_INITIN(v)      do{ bFM_GPIO_PCR4_PE=(v).bPullup; \
                                         bFM_GPIO_DDR4_PE=0u; \
                                         bFM_GPIO_PFR4_PE=0u; }while(0u)

#define GPIO1PIN_NP4E_INITOUT(v)     do{ bFM_GPIO_PDOR4_PE=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR4_PE=1u; \
                                         bFM_GPIO_PFR4_PE=0u; }while(0u)


/*---- GPIO bit P50 ----*/

#define GPIO1PIN_P50_GET             ( bFM_GPIO_PDIR5_P0 )

#define GPIO1PIN_P50_PUT(v)          ( bFM_GPIO_PDOR5_P0=(v) )

#define GPIO1PIN_P50_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P50_INITIN(v) \
                                                        : GPIO1PIN_P50_INITOUT(v) )

#define GPIO1PIN_P50_INITIN(v)       do{ bFM_GPIO_PCR5_P0=(v).bPullup; \
                                         bFM_GPIO_DDR5_P0=0u; \
                                         bFM_GPIO_PFR5_P0=0u; }while(0u)

#define GPIO1PIN_P50_INITOUT(v)      do{ bFM_GPIO_PDOR5_P0=(v).bInitVal; \
                                         bFM_GPIO_DDR5_P0=1u; \
                                         bFM_GPIO_PFR5_P0=0u; }while(0u)


/*---- GPIO bit NP50 ----*/
#define GPIO1PIN_NP50_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR5_P0)) )

#define GPIO1PIN_NP50_PUT(v)         ( bFM_GPIO_PDOR5_P0=(uint32_t)(!(v)) )

#define GPIO1PIN_NP50_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP50_INITIN(v) \
                                                        : GPIO1PIN_NP50_INITOUT(v) )

#define GPIO1PIN_NP50_INITIN(v)      do{ bFM_GPIO_PCR5_P0=(v).bPullup; \
                                         bFM_GPIO_DDR5_P0=0u; \
                                         bFM_GPIO_PFR5_P0=0u; }while(0u)

#define GPIO1PIN_NP50_INITOUT(v)     do{ bFM_GPIO_PDOR5_P0=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR5_P0=1u; \
                                         bFM_GPIO_PFR5_P0=0u; }while(0u)


/*---- GPIO bit P51 ----*/

#define GPIO1PIN_P51_GET             ( bFM_GPIO_PDIR5_P1 )

#define GPIO1PIN_P51_PUT(v)          ( bFM_GPIO_PDOR5_P1=(v) )

#define GPIO1PIN_P51_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P51_INITIN(v) \
                                                        : GPIO1PIN_P51_INITOUT(v) )

#define GPIO1PIN_P51_INITIN(v)       do{ bFM_GPIO_PCR5_P1=(v).bPullup; \
                                         bFM_GPIO_DDR5_P1=0u; \
                                         bFM_GPIO_PFR5_P1=0u; }while(0u)

#define GPIO1PIN_P51_INITOUT(v)      do{ bFM_GPIO_PDOR5_P1=(v).bInitVal; \
                                         bFM_GPIO_DDR5_P1=1u; \
                                         bFM_GPIO_PFR5_P1=0u; }while(0u)


/*---- GPIO bit NP51 ----*/
#define GPIO1PIN_NP51_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR5_P1)) )

#define GPIO1PIN_NP51_PUT(v)         ( bFM_GPIO_PDOR5_P1=(uint32_t)(!(v)) )

#define GPIO1PIN_NP51_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP51_INITIN(v) \
                                                        : GPIO1PIN_NP51_INITOUT(v) )

#define GPIO1PIN_NP51_INITIN(v)      do{ bFM_GPIO_PCR5_P1=(v).bPullup; \
                                         bFM_GPIO_DDR5_P1=0u; \
                                         bFM_GPIO_PFR5_P1=0u; }while(0u)

#define GPIO1PIN_NP51_INITOUT(v)     do{ bFM_GPIO_PDOR5_P1=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR5_P1=1u; \
                                         bFM_GPIO_PFR5_P1=0u; }while(0u)


/*---- GPIO bit P52 ----*/

#define GPIO1PIN_P52_GET             ( bFM_GPIO_PDIR5_P2 )

#define GPIO1PIN_P52_PUT(v)          ( bFM_GPIO_PDOR5_P2=(v) )

#define GPIO1PIN_P52_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P52_INITIN(v) \
                                                        : GPIO1PIN_P52_INITOUT(v) )

#define GPIO1PIN_P52_INITIN(v)       do{ bFM_GPIO_PCR5_P2=(v).bPullup; \
                                         bFM_GPIO_DDR5_P2=0u; \
                                         bFM_GPIO_PFR5_P2=0u; }while(0u)

#define GPIO1PIN_P52_INITOUT(v)      do{ bFM_GPIO_PDOR5_P2=(v).bInitVal; \
                                         bFM_GPIO_DDR5_P2=1u; \
                                         bFM_GPIO_PFR5_P2=0u; }while(0u)


/*---- GPIO bit NP52 ----*/
#define GPIO1PIN_NP52_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR5_P2)) )

#define GPIO1PIN_NP52_PUT(v)         ( bFM_GPIO_PDOR5_P2=(uint32_t)(!(v)) )

#define GPIO1PIN_NP52_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP52_INITIN(v) \
                                                        : GPIO1PIN_NP52_INITOUT(v) )

#define GPIO1PIN_NP52_INITIN(v)      do{ bFM_GPIO_PCR5_P2=(v).bPullup; \
                                         bFM_GPIO_DDR5_P2=0u; \
                                         bFM_GPIO_PFR5_P2=0u; }while(0u)

#define GPIO1PIN_NP52_INITOUT(v)     do{ bFM_GPIO_PDOR5_P2=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR5_P2=1u; \
                                         bFM_GPIO_PFR5_P2=0u; }while(0u)


/*---- GPIO bit P53 ----*/

#define GPIO1PIN_P53_GET             ( bFM_GPIO_PDIR5_P3 )

#define GPIO1PIN_P53_PUT(v)          ( bFM_GPIO_PDOR5_P3=(v) )

#define GPIO1PIN_P53_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P53_INITIN(v) \
                                                        : GPIO1PIN_P53_INITOUT(v) )

#define GPIO1PIN_P53_INITIN(v)       do{ bFM_GPIO_PCR5_P3=(v).bPullup; \
                                         bFM_GPIO_DDR5_P3=0u; \
                                         bFM_GPIO_PFR5_P3=0u; }while(0u)

#define GPIO1PIN_P53_INITOUT(v)      do{ bFM_GPIO_PDOR5_P3=(v).bInitVal; \
                                         bFM_GPIO_DDR5_P3=1u; \
                                         bFM_GPIO_PFR5_P3=0u; }while(0u)


/*---- GPIO bit NP53 ----*/
#define GPIO1PIN_NP53_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR5_P3)) )

#define GPIO1PIN_NP53_PUT(v)         ( bFM_GPIO_PDOR5_P3=(uint32_t)(!(v)) )

#define GPIO1PIN_NP53_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP53_INITIN(v) \
                                                        : GPIO1PIN_NP53_INITOUT(v) )

#define GPIO1PIN_NP53_INITIN(v)      do{ bFM_GPIO_PCR5_P3=(v).bPullup; \
                                         bFM_GPIO_DDR5_P3=0u; \
                                         bFM_GPIO_PFR5_P3=0u; }while(0u)

#define GPIO1PIN_NP53_INITOUT(v)     do{ bFM_GPIO_PDOR5_P3=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR5_P3=1u; \
                                         bFM_GPIO_PFR5_P3=0u; }while(0u)


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
                                         bFM_GPIO_PFR6_P1=0u; }while(0u)

#define GPIO1PIN_P61_INITOUT(v)      do{ bFM_GPIO_PDOR6_P1=(v).bInitVal; \
                                         bFM_GPIO_DDR6_P1=1u; \
                                         bFM_GPIO_PFR6_P1=0u; }while(0u)


/*---- GPIO bit NP61 ----*/
#define GPIO1PIN_NP61_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR6_P1)) )

#define GPIO1PIN_NP61_PUT(v)         ( bFM_GPIO_PDOR6_P1=(uint32_t)(!(v)) )

#define GPIO1PIN_NP61_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP61_INITIN(v) \
                                                        : GPIO1PIN_NP61_INITOUT(v) )

#define GPIO1PIN_NP61_INITIN(v)      do{ bFM_GPIO_PCR6_P1=(v).bPullup; \
                                         bFM_GPIO_DDR6_P1=0u; \
                                         bFM_GPIO_PFR6_P1=0u; }while(0u)

#define GPIO1PIN_NP61_INITOUT(v)     do{ bFM_GPIO_PDOR6_P1=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR6_P1=1u; \
                                         bFM_GPIO_PFR6_P1=0u; }while(0u)


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
   Fast GPIO
*******************************************************************************/

/*---- Fast GPIO bit P00 ----*/

#define FGPIO1PIN_P00_GET             ( FM_FASTIO->FPDIR0_f.P0 )

#define FGPIO1PIN_P00_PUT(v)          ( FM_FASTIO->FPDOR0_f.P0=(v) )

#define FGPIO1PIN_P00_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P00_INITIN(v) \
                                                         : FGPIO1PIN_P00_INITOUT(v) )

#define FGPIO1PIN_P00_INITIN(v)       do{ bFM_GPIO_PCR0_P0=(v).bPullup; \
                                          bFM_GPIO_DDR0_P0=0u; \
                                          bFM_GPIO_PFR0_P0=0u; }while(0)

#define FGPIO1PIN_P00_INITOUT(v)      do{ FM_FASTIO->FPDOR0_f.P0=(v).bInitVal; \
                                          bFM_GPIO_DDR0_P0=1u; \
                                          bFM_GPIO_PFR0_P0=0u; }while(0)


/*---- Fast GPIO bit NP00 ----*/
#define FGPIO1PIN_NP00_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR0_f.P0)) )

#define FGPIO1PIN_NP00_PUT(v)         ( FM_FASTIO->FPDOR0_f.P0=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP00_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP00_INITIN(v) \
                                                         : FGPIO1PIN_NP00_INITOUT(v) )

#define FGPIO1PIN_NP00_INITIN(v)      do{ bFM_GPIO_PCR0_P0=(v).bPullup; \
                                          bFM_GPIO_DDR0_P0=0u; \
                                          bFM_GPIO_PFR0_P0=0u; }while(0)

#define FGPIO1PIN_NP00_INITOUT(v)     do{ bFM_GPIO_PDOR0_P0=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR0_P0=1u; \
                                          bFM_GPIO_PFR0_P0=0u; }while(0)


/*---- Fast GPIO bit P01 ----*/

#define FGPIO1PIN_P01_GET             ( FM_FASTIO->FPDIR0_f.P1 )

#define FGPIO1PIN_P01_PUT(v)          ( FM_FASTIO->FPDOR0_f.P1=(v) )

#define FGPIO1PIN_P01_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P01_INITIN(v) \
                                                         : FGPIO1PIN_P01_INITOUT(v) )

#define FGPIO1PIN_P01_INITIN(v)       do{ bFM_GPIO_PCR0_P1=(v).bPullup; \
                                          bFM_GPIO_DDR0_P1=0u; \
                                          bFM_GPIO_PFR0_P1=0u; }while(0)

#define FGPIO1PIN_P01_INITOUT(v)      do{ FM_FASTIO->FPDOR0_f.P1=(v).bInitVal; \
                                          bFM_GPIO_DDR0_P1=1u; \
                                          bFM_GPIO_PFR0_P1=0u; }while(0)


/*---- Fast GPIO bit NP01 ----*/
#define FGPIO1PIN_NP01_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR0_f.P1)) )

#define FGPIO1PIN_NP01_PUT(v)         ( FM_FASTIO->FPDOR0_f.P1=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP01_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP01_INITIN(v) \
                                                         : FGPIO1PIN_NP01_INITOUT(v) )

#define FGPIO1PIN_NP01_INITIN(v)      do{ bFM_GPIO_PCR0_P1=(v).bPullup; \
                                          bFM_GPIO_DDR0_P1=0u; \
                                          bFM_GPIO_PFR0_P1=0u; }while(0)

#define FGPIO1PIN_NP01_INITOUT(v)     do{ bFM_GPIO_PDOR0_P1=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR0_P1=1u; \
                                          bFM_GPIO_PFR0_P1=0u; }while(0)


/*---- Fast GPIO bit P02 ----*/

#define FGPIO1PIN_P02_GET             ( FM_FASTIO->FPDIR0_f.P2 )

#define FGPIO1PIN_P02_PUT(v)          ( FM_FASTIO->FPDOR0_f.P2=(v) )

#define FGPIO1PIN_P02_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P02_INITIN(v) \
                                                         : FGPIO1PIN_P02_INITOUT(v) )

#define FGPIO1PIN_P02_INITIN(v)       do{ bFM_GPIO_PCR0_P2=(v).bPullup; \
                                          bFM_GPIO_DDR0_P2=0u; \
                                          bFM_GPIO_PFR0_P2=0u; }while(0)

#define FGPIO1PIN_P02_INITOUT(v)      do{ FM_FASTIO->FPDOR0_f.P2=(v).bInitVal; \
                                          bFM_GPIO_DDR0_P2=1u; \
                                          bFM_GPIO_PFR0_P2=0u; }while(0)


/*---- Fast GPIO bit NP02 ----*/
#define FGPIO1PIN_NP02_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR0_f.P2)) )

#define FGPIO1PIN_NP02_PUT(v)         ( FM_FASTIO->FPDOR0_f.P2=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP02_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP02_INITIN(v) \
                                                         : FGPIO1PIN_NP02_INITOUT(v) )

#define FGPIO1PIN_NP02_INITIN(v)      do{ bFM_GPIO_PCR0_P2=(v).bPullup; \
                                          bFM_GPIO_DDR0_P2=0u; \
                                          bFM_GPIO_PFR0_P2=0u; }while(0)

#define FGPIO1PIN_NP02_INITOUT(v)     do{ bFM_GPIO_PDOR0_P2=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR0_P2=1u; \
                                          bFM_GPIO_PFR0_P2=0u; }while(0)


/*---- Fast GPIO bit P03 ----*/

#define FGPIO1PIN_P03_GET             ( FM_FASTIO->FPDIR0_f.P3 )

#define FGPIO1PIN_P03_PUT(v)          ( FM_FASTIO->FPDOR0_f.P3=(v) )

#define FGPIO1PIN_P03_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P03_INITIN(v) \
                                                         : FGPIO1PIN_P03_INITOUT(v) )

#define FGPIO1PIN_P03_INITIN(v)       do{ bFM_GPIO_PCR0_P3=(v).bPullup; \
                                          bFM_GPIO_DDR0_P3=0u; \
                                          bFM_GPIO_PFR0_P3=0u; }while(0)

#define FGPIO1PIN_P03_INITOUT(v)      do{ FM_FASTIO->FPDOR0_f.P3=(v).bInitVal; \
                                          bFM_GPIO_DDR0_P3=1u; \
                                          bFM_GPIO_PFR0_P3=0u; }while(0)


/*---- Fast GPIO bit NP03 ----*/
#define FGPIO1PIN_NP03_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR0_f.P3)) )

#define FGPIO1PIN_NP03_PUT(v)         ( FM_FASTIO->FPDOR0_f.P3=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP03_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP03_INITIN(v) \
                                                         : FGPIO1PIN_NP03_INITOUT(v) )

#define FGPIO1PIN_NP03_INITIN(v)      do{ bFM_GPIO_PCR0_P3=(v).bPullup; \
                                          bFM_GPIO_DDR0_P3=0u; \
                                          bFM_GPIO_PFR0_P3=0u; }while(0)

#define FGPIO1PIN_NP03_INITOUT(v)     do{ bFM_GPIO_PDOR0_P3=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR0_P3=1u; \
                                          bFM_GPIO_PFR0_P3=0u; }while(0)


/*---- Fast GPIO bit P05 ----*/

#define FGPIO1PIN_P05_GET             ( FM_FASTIO->FPDIR0_f.P5 )

#define FGPIO1PIN_P05_PUT(v)          ( FM_FASTIO->FPDOR0_f.P5=(v) )

#define FGPIO1PIN_P05_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P05_INITIN(v) \
                                                         : FGPIO1PIN_P05_INITOUT(v) )

#define FGPIO1PIN_P05_INITIN(v)       do{ bFM_GPIO_PCR0_P5=(v).bPullup; \
                                          bFM_GPIO_DDR0_P5=0u; \
                                          bFM_GPIO_PFR0_P5=0u; }while(0)

#define FGPIO1PIN_P05_INITOUT(v)      do{ FM_FASTIO->FPDOR0_f.P5=(v).bInitVal; \
                                          bFM_GPIO_DDR0_P5=1u; \
                                          bFM_GPIO_PFR0_P5=0u; }while(0)


/*---- Fast GPIO bit NP05 ----*/
#define FGPIO1PIN_NP05_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR0_f.P5)) )

#define FGPIO1PIN_NP05_PUT(v)         ( FM_FASTIO->FPDOR0_f.P5=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP05_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP05_INITIN(v) \
                                                         : FGPIO1PIN_NP05_INITOUT(v) )

#define FGPIO1PIN_NP05_INITIN(v)      do{ bFM_GPIO_PCR0_P5=(v).bPullup; \
                                          bFM_GPIO_DDR0_P5=0u; \
                                          bFM_GPIO_PFR0_P5=0u; }while(0)

#define FGPIO1PIN_NP05_INITOUT(v)     do{ bFM_GPIO_PDOR0_P5=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR0_P5=1u; \
                                          bFM_GPIO_PFR0_P5=0u; }while(0)


/*---- Fast GPIO bit P0B ----*/

#define FGPIO1PIN_P0B_GET             ( FM_FASTIO->FPDIR0_f.PB )

#define FGPIO1PIN_P0B_PUT(v)          ( FM_FASTIO->FPDOR0_f.PB=(v) )

#define FGPIO1PIN_P0B_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P0B_INITIN(v) \
                                                         : FGPIO1PIN_P0B_INITOUT(v) )

#define FGPIO1PIN_P0B_INITIN(v)       do{ bFM_GPIO_PCR0_PB=(v).bPullup; \
                                          bFM_GPIO_DDR0_PB=0u; \
                                          bFM_GPIO_PFR0_PB=0u; }while(0)

#define FGPIO1PIN_P0B_INITOUT(v)      do{ FM_FASTIO->FPDOR0_f.PB=(v).bInitVal; \
                                          bFM_GPIO_DDR0_PB=1u; \
                                          bFM_GPIO_PFR0_PB=0u; }while(0)


/*---- Fast GPIO bit NP0B ----*/
#define FGPIO1PIN_NP0B_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR0_f.PB)) )

#define FGPIO1PIN_NP0B_PUT(v)         ( FM_FASTIO->FPDOR0_f.PB=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP0B_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP0B_INITIN(v) \
                                                         : FGPIO1PIN_NP0B_INITOUT(v) )

#define FGPIO1PIN_NP0B_INITIN(v)      do{ bFM_GPIO_PCR0_PB=(v).bPullup; \
                                          bFM_GPIO_DDR0_PB=0u; \
                                          bFM_GPIO_PFR0_PB=0u; }while(0)

#define FGPIO1PIN_NP0B_INITOUT(v)     do{ bFM_GPIO_PDOR0_PB=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR0_PB=1u; \
                                          bFM_GPIO_PFR0_PB=0u; }while(0)


/*---- Fast GPIO bit P0C ----*/

#define FGPIO1PIN_P0C_GET             ( FM_FASTIO->FPDIR0_f.PC )

#define FGPIO1PIN_P0C_PUT(v)          ( FM_FASTIO->FPDOR0_f.PC=(v) )

#define FGPIO1PIN_P0C_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P0C_INITIN(v) \
                                                         : FGPIO1PIN_P0C_INITOUT(v) )

#define FGPIO1PIN_P0C_INITIN(v)       do{ bFM_GPIO_PCR0_PC=(v).bPullup; \
                                          bFM_GPIO_DDR0_PC=0u; \
                                          bFM_GPIO_PFR0_PC=0u; }while(0)

#define FGPIO1PIN_P0C_INITOUT(v)      do{ FM_FASTIO->FPDOR0_f.PC=(v).bInitVal; \
                                          bFM_GPIO_DDR0_PC=1u; \
                                          bFM_GPIO_PFR0_PC=0u; }while(0)


/*---- Fast GPIO bit NP0C ----*/
#define FGPIO1PIN_NP0C_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR0_f.PC)) )

#define FGPIO1PIN_NP0C_PUT(v)         ( FM_FASTIO->FPDOR0_f.PC=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP0C_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP0C_INITIN(v) \
                                                         : FGPIO1PIN_NP0C_INITOUT(v) )

#define FGPIO1PIN_NP0C_INITIN(v)      do{ bFM_GPIO_PCR0_PC=(v).bPullup; \
                                          bFM_GPIO_DDR0_PC=0u; \
                                          bFM_GPIO_PFR0_PC=0u; }while(0)

#define FGPIO1PIN_NP0C_INITOUT(v)     do{ bFM_GPIO_PDOR0_PC=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR0_PC=1u; \
                                          bFM_GPIO_PFR0_PC=0u; }while(0)


/*---- Fast GPIO bit P0F ----*/

#define FGPIO1PIN_P0F_GET             ( FM_FASTIO->FPDIR0_f.PF )

#define FGPIO1PIN_P0F_PUT(v)          ( FM_FASTIO->FPDOR0_f.PF=(v) )

#define FGPIO1PIN_P0F_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P0F_INITIN(v) \
                                                         : FGPIO1PIN_P0F_INITOUT(v) )

#define FGPIO1PIN_P0F_INITIN(v)       do{ bFM_GPIO_PCR0_PF=(v).bPullup; \
                                          bFM_GPIO_DDR0_PF=0u; \
                                          bFM_GPIO_PFR0_PF=0u; }while(0)

#define FGPIO1PIN_P0F_INITOUT(v)      do{ FM_FASTIO->FPDOR0_f.PF=(v).bInitVal; \
                                          bFM_GPIO_DDR0_PF=1u; \
                                          bFM_GPIO_PFR0_PF=0u; }while(0)


/*---- Fast GPIO bit NP0F ----*/
#define FGPIO1PIN_NP0F_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR0_f.PF)) )

#define FGPIO1PIN_NP0F_PUT(v)         ( FM_FASTIO->FPDOR0_f.PF=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP0F_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP0F_INITIN(v) \
                                                         : FGPIO1PIN_NP0F_INITOUT(v) )

#define FGPIO1PIN_NP0F_INITIN(v)      do{ bFM_GPIO_PCR0_PF=(v).bPullup; \
                                          bFM_GPIO_DDR0_PF=0u; \
                                          bFM_GPIO_PFR0_PF=0u; }while(0)

#define FGPIO1PIN_NP0F_INITOUT(v)     do{ bFM_GPIO_PDOR0_PF=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR0_PF=1u; \
                                          bFM_GPIO_PFR0_PF=0u; }while(0)


/*---- Fast GPIO bit P10 ----*/

#define FGPIO1PIN_P10_GET             ( FM_FASTIO->FPDIR1_f.P0 )

#define FGPIO1PIN_P10_PUT(v)          ( FM_FASTIO->FPDOR1_f.P0=(v) )

#define FGPIO1PIN_P10_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P10_INITIN(v) \
                                                         : FGPIO1PIN_P10_INITOUT(v) )

#define FGPIO1PIN_P10_INITIN(v)       do{ bFM_GPIO_PCR1_P0=(v).bPullup; \
                                          bFM_GPIO_DDR1_P0=0u; \
                                          bFM_GPIO_PFR1_P0=0u; \
                                          bFM_GPIO_ADE_AN00=0u; }while(0)


#define FGPIO1PIN_P10_INITOUT(v)      do{ FM_FASTIO->FPDOR1_f.P0=(v).bInitVal; \
                                          bFM_GPIO_DDR1_P0=1u; \
                                          bFM_GPIO_PFR1_P0=0u; \
                                          bFM_GPIO_ADE_AN00=0u; }while(0)


/*---- Fast GPIO bit NP10 ----*/
#define FGPIO1PIN_NP10_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR1_f.P0)) )

#define FGPIO1PIN_NP10_PUT(v)         ( FM_FASTIO->FPDOR1_f.P0=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP10_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP10_INITIN(v) \
                                                         : FGPIO1PIN_NP10_INITOUT(v) )

#define FGPIO1PIN_NP10_INITIN(v)      do{ bFM_GPIO_PCR1_P0=(v).bPullup; \
                                          bFM_GPIO_DDR1_P0=0u; \
                                          bFM_GPIO_PFR1_P0=0u; \
                                          bFM_GPIO_ADE_AN00=0u; }while(0)


#define FGPIO1PIN_NP10_INITOUT(v)     do{ bFM_GPIO_PDOR1_P0=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR1_P0=1u; \
                                          bFM_GPIO_PFR1_P0=0u; \
                                          bFM_GPIO_ADE_AN00=0u; }while(0)


/*---- Fast GPIO bit P11 ----*/

#define FGPIO1PIN_P11_GET             ( FM_FASTIO->FPDIR1_f.P1 )

#define FGPIO1PIN_P11_PUT(v)          ( FM_FASTIO->FPDOR1_f.P1=(v) )

#define FGPIO1PIN_P11_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P11_INITIN(v) \
                                                         : FGPIO1PIN_P11_INITOUT(v) )

#define FGPIO1PIN_P11_INITIN(v)       do{ bFM_GPIO_PCR1_P1=(v).bPullup; \
                                          bFM_GPIO_DDR1_P1=0u; \
                                          bFM_GPIO_PFR1_P1=0u; \
                                          bFM_GPIO_ADE_AN01=0u; }while(0)


#define FGPIO1PIN_P11_INITOUT(v)      do{ FM_FASTIO->FPDOR1_f.P1=(v).bInitVal; \
                                          bFM_GPIO_DDR1_P1=1u; \
                                          bFM_GPIO_PFR1_P1=0u; \
                                          bFM_GPIO_ADE_AN01=0u; }while(0)


/*---- Fast GPIO bit NP11 ----*/
#define FGPIO1PIN_NP11_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR1_f.P1)) )

#define FGPIO1PIN_NP11_PUT(v)         ( FM_FASTIO->FPDOR1_f.P1=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP11_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP11_INITIN(v) \
                                                         : FGPIO1PIN_NP11_INITOUT(v) )

#define FGPIO1PIN_NP11_INITIN(v)      do{ bFM_GPIO_PCR1_P1=(v).bPullup; \
                                          bFM_GPIO_DDR1_P1=0u; \
                                          bFM_GPIO_PFR1_P1=0u; \
                                          bFM_GPIO_ADE_AN01=0u; }while(0)


#define FGPIO1PIN_NP11_INITOUT(v)     do{ bFM_GPIO_PDOR1_P1=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR1_P1=1u; \
                                          bFM_GPIO_PFR1_P1=0u; \
                                          bFM_GPIO_ADE_AN01=0u; }while(0)


/*---- Fast GPIO bit P12 ----*/

#define FGPIO1PIN_P12_GET             ( FM_FASTIO->FPDIR1_f.P2 )

#define FGPIO1PIN_P12_PUT(v)          ( FM_FASTIO->FPDOR1_f.P2=(v) )

#define FGPIO1PIN_P12_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P12_INITIN(v) \
                                                         : FGPIO1PIN_P12_INITOUT(v) )

#define FGPIO1PIN_P12_INITIN(v)       do{ bFM_GPIO_PCR1_P2=(v).bPullup; \
                                          bFM_GPIO_DDR1_P2=0u; \
                                          bFM_GPIO_PFR1_P2=0u; \
                                          bFM_GPIO_ADE_AN02=0u; }while(0)


#define FGPIO1PIN_P12_INITOUT(v)      do{ FM_FASTIO->FPDOR1_f.P2=(v).bInitVal; \
                                          bFM_GPIO_DDR1_P2=1u; \
                                          bFM_GPIO_PFR1_P2=0u; \
                                          bFM_GPIO_ADE_AN02=0u; }while(0)


/*---- Fast GPIO bit NP12 ----*/
#define FGPIO1PIN_NP12_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR1_f.P2)) )

#define FGPIO1PIN_NP12_PUT(v)         ( FM_FASTIO->FPDOR1_f.P2=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP12_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP12_INITIN(v) \
                                                         : FGPIO1PIN_NP12_INITOUT(v) )

#define FGPIO1PIN_NP12_INITIN(v)      do{ bFM_GPIO_PCR1_P2=(v).bPullup; \
                                          bFM_GPIO_DDR1_P2=0u; \
                                          bFM_GPIO_PFR1_P2=0u; \
                                          bFM_GPIO_ADE_AN02=0u; }while(0)


#define FGPIO1PIN_NP12_INITOUT(v)     do{ bFM_GPIO_PDOR1_P2=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR1_P2=1u; \
                                          bFM_GPIO_PFR1_P2=0u; \
                                          bFM_GPIO_ADE_AN02=0u; }while(0)


/*---- Fast GPIO bit P13 ----*/

#define FGPIO1PIN_P13_GET             ( FM_FASTIO->FPDIR1_f.P3 )

#define FGPIO1PIN_P13_PUT(v)          ( FM_FASTIO->FPDOR1_f.P3=(v) )

#define FGPIO1PIN_P13_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P13_INITIN(v) \
                                                         : FGPIO1PIN_P13_INITOUT(v) )

#define FGPIO1PIN_P13_INITIN(v)       do{ bFM_GPIO_PCR1_P3=(v).bPullup; \
                                          bFM_GPIO_DDR1_P3=0u; \
                                          bFM_GPIO_PFR1_P3=0u; \
                                          bFM_GPIO_ADE_AN03=0u; }while(0)


#define FGPIO1PIN_P13_INITOUT(v)      do{ FM_FASTIO->FPDOR1_f.P3=(v).bInitVal; \
                                          bFM_GPIO_DDR1_P3=1u; \
                                          bFM_GPIO_PFR1_P3=0u; \
                                          bFM_GPIO_ADE_AN03=0u; }while(0)


/*---- Fast GPIO bit NP13 ----*/
#define FGPIO1PIN_NP13_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR1_f.P3)) )

#define FGPIO1PIN_NP13_PUT(v)         ( FM_FASTIO->FPDOR1_f.P3=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP13_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP13_INITIN(v) \
                                                         : FGPIO1PIN_NP13_INITOUT(v) )

#define FGPIO1PIN_NP13_INITIN(v)      do{ bFM_GPIO_PCR1_P3=(v).bPullup; \
                                          bFM_GPIO_DDR1_P3=0u; \
                                          bFM_GPIO_PFR1_P3=0u; \
                                          bFM_GPIO_ADE_AN03=0u; }while(0)


#define FGPIO1PIN_NP13_INITOUT(v)     do{ bFM_GPIO_PDOR1_P3=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR1_P3=1u; \
                                          bFM_GPIO_PFR1_P3=0u; \
                                          bFM_GPIO_ADE_AN03=0u; }while(0)


/*---- Fast GPIO bit P14 ----*/

#define FGPIO1PIN_P14_GET             ( FM_FASTIO->FPDIR1_f.P4 )

#define FGPIO1PIN_P14_PUT(v)          ( FM_FASTIO->FPDOR1_f.P4=(v) )

#define FGPIO1PIN_P14_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P14_INITIN(v) \
                                                         : FGPIO1PIN_P14_INITOUT(v) )

#define FGPIO1PIN_P14_INITIN(v)       do{ bFM_GPIO_PCR1_P4=(v).bPullup; \
                                          bFM_GPIO_DDR1_P4=0u; \
                                          bFM_GPIO_PFR1_P4=0u; \
                                          bFM_GPIO_ADE_AN04=0u; }while(0)


#define FGPIO1PIN_P14_INITOUT(v)      do{ FM_FASTIO->FPDOR1_f.P4=(v).bInitVal; \
                                          bFM_GPIO_DDR1_P4=1u; \
                                          bFM_GPIO_PFR1_P4=0u; \
                                          bFM_GPIO_ADE_AN04=0u; }while(0)


/*---- Fast GPIO bit NP14 ----*/
#define FGPIO1PIN_NP14_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR1_f.P4)) )

#define FGPIO1PIN_NP14_PUT(v)         ( FM_FASTIO->FPDOR1_f.P4=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP14_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP14_INITIN(v) \
                                                         : FGPIO1PIN_NP14_INITOUT(v) )

#define FGPIO1PIN_NP14_INITIN(v)      do{ bFM_GPIO_PCR1_P4=(v).bPullup; \
                                          bFM_GPIO_DDR1_P4=0u; \
                                          bFM_GPIO_PFR1_P4=0u; \
                                          bFM_GPIO_ADE_AN04=0u; }while(0)


#define FGPIO1PIN_NP14_INITOUT(v)     do{ bFM_GPIO_PDOR1_P4=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR1_P4=1u; \
                                          bFM_GPIO_PFR1_P4=0u; \
                                          bFM_GPIO_ADE_AN04=0u; }while(0)


/*---- Fast GPIO bit P15 ----*/

#define FGPIO1PIN_P15_GET             ( FM_FASTIO->FPDIR1_f.P5 )

#define FGPIO1PIN_P15_PUT(v)          ( FM_FASTIO->FPDOR1_f.P5=(v) )

#define FGPIO1PIN_P15_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P15_INITIN(v) \
                                                         : FGPIO1PIN_P15_INITOUT(v) )

#define FGPIO1PIN_P15_INITIN(v)       do{ bFM_GPIO_PCR1_P5=(v).bPullup; \
                                          bFM_GPIO_DDR1_P5=0u; \
                                          bFM_GPIO_PFR1_P5=0u; \
                                          bFM_GPIO_ADE_AN05=0u; }while(0)


#define FGPIO1PIN_P15_INITOUT(v)      do{ FM_FASTIO->FPDOR1_f.P5=(v).bInitVal; \
                                          bFM_GPIO_DDR1_P5=1u; \
                                          bFM_GPIO_PFR1_P5=0u; \
                                          bFM_GPIO_ADE_AN05=0u; }while(0)


/*---- Fast GPIO bit NP15 ----*/
#define FGPIO1PIN_NP15_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR1_f.P5)) )

#define FGPIO1PIN_NP15_PUT(v)         ( FM_FASTIO->FPDOR1_f.P5=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP15_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP15_INITIN(v) \
                                                         : FGPIO1PIN_NP15_INITOUT(v) )

#define FGPIO1PIN_NP15_INITIN(v)      do{ bFM_GPIO_PCR1_P5=(v).bPullup; \
                                          bFM_GPIO_DDR1_P5=0u; \
                                          bFM_GPIO_PFR1_P5=0u; \
                                          bFM_GPIO_ADE_AN05=0u; }while(0)


#define FGPIO1PIN_NP15_INITOUT(v)     do{ bFM_GPIO_PDOR1_P5=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR1_P5=1u; \
                                          bFM_GPIO_PFR1_P5=0u; \
                                          bFM_GPIO_ADE_AN05=0u; }while(0)


/*---- Fast GPIO bit P1A ----*/

#define FGPIO1PIN_P1A_GET             ( FM_FASTIO->FPDIR1_f.PA )

#define FGPIO1PIN_P1A_PUT(v)          ( FM_FASTIO->FPDOR1_f.PA=(v) )

#define FGPIO1PIN_P1A_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P1A_INITIN(v) \
                                                         : FGPIO1PIN_P1A_INITOUT(v) )

#define FGPIO1PIN_P1A_INITIN(v)       do{ bFM_GPIO_PCR1_PA=(v).bPullup; \
                                          bFM_GPIO_DDR1_PA=0u; \
                                          bFM_GPIO_PFR1_PA=0u; }while(0)

#define FGPIO1PIN_P1A_INITOUT(v)      do{ FM_FASTIO->FPDOR1_f.PA=(v).bInitVal; \
                                          bFM_GPIO_DDR1_PA=1u; \
                                          bFM_GPIO_PFR1_PA=0u; }while(0)


/*---- Fast GPIO bit NP1A ----*/
#define FGPIO1PIN_NP1A_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR1_f.PA)) )

#define FGPIO1PIN_NP1A_PUT(v)         ( FM_FASTIO->FPDOR1_f.PA=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP1A_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP1A_INITIN(v) \
                                                         : FGPIO1PIN_NP1A_INITOUT(v) )

#define FGPIO1PIN_NP1A_INITIN(v)      do{ bFM_GPIO_PCR1_PA=(v).bPullup; \
                                          bFM_GPIO_DDR1_PA=0u; \
                                          bFM_GPIO_PFR1_PA=0u; }while(0)

#define FGPIO1PIN_NP1A_INITOUT(v)     do{ bFM_GPIO_PDOR1_PA=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR1_PA=1u; \
                                          bFM_GPIO_PFR1_PA=0u; }while(0)


/*---- Fast GPIO bit P1B ----*/

#define FGPIO1PIN_P1B_GET             ( FM_FASTIO->FPDIR1_f.PB )

#define FGPIO1PIN_P1B_PUT(v)          ( FM_FASTIO->FPDOR1_f.PB=(v) )

#define FGPIO1PIN_P1B_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P1B_INITIN(v) \
                                                         : FGPIO1PIN_P1B_INITOUT(v) )

#define FGPIO1PIN_P1B_INITIN(v)       do{ bFM_GPIO_PCR1_PB=(v).bPullup; \
                                          bFM_GPIO_DDR1_PB=0u; \
                                          bFM_GPIO_PFR1_PB=0u; }while(0)

#define FGPIO1PIN_P1B_INITOUT(v)      do{ FM_FASTIO->FPDOR1_f.PB=(v).bInitVal; \
                                          bFM_GPIO_DDR1_PB=1u; \
                                          bFM_GPIO_PFR1_PB=0u; }while(0)


/*---- Fast GPIO bit NP1B ----*/
#define FGPIO1PIN_NP1B_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR1_f.PB)) )

#define FGPIO1PIN_NP1B_PUT(v)         ( FM_FASTIO->FPDOR1_f.PB=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP1B_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP1B_INITIN(v) \
                                                         : FGPIO1PIN_NP1B_INITOUT(v) )

#define FGPIO1PIN_NP1B_INITIN(v)      do{ bFM_GPIO_PCR1_PB=(v).bPullup; \
                                          bFM_GPIO_DDR1_PB=0u; \
                                          bFM_GPIO_PFR1_PB=0u; }while(0)

#define FGPIO1PIN_NP1B_INITOUT(v)     do{ bFM_GPIO_PDOR1_PB=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR1_PB=1u; \
                                          bFM_GPIO_PFR1_PB=0u; }while(0)


/*---- Fast GPIO bit P1C ----*/

#define FGPIO1PIN_P1C_GET             ( FM_FASTIO->FPDIR1_f.PC )

#define FGPIO1PIN_P1C_PUT(v)          ( FM_FASTIO->FPDOR1_f.PC=(v) )

#define FGPIO1PIN_P1C_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P1C_INITIN(v) \
                                                         : FGPIO1PIN_P1C_INITOUT(v) )

#define FGPIO1PIN_P1C_INITIN(v)       do{ bFM_GPIO_PCR1_PC=(v).bPullup; \
                                          bFM_GPIO_DDR1_PC=0u; \
                                          bFM_GPIO_PFR1_PC=0u; }while(0)

#define FGPIO1PIN_P1C_INITOUT(v)      do{ FM_FASTIO->FPDOR1_f.PC=(v).bInitVal; \
                                          bFM_GPIO_DDR1_PC=1u; \
                                          bFM_GPIO_PFR1_PC=0u; }while(0)


/*---- Fast GPIO bit NP1C ----*/
#define FGPIO1PIN_NP1C_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR1_f.PC)) )

#define FGPIO1PIN_NP1C_PUT(v)         ( FM_FASTIO->FPDOR1_f.PC=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP1C_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP1C_INITIN(v) \
                                                         : FGPIO1PIN_NP1C_INITOUT(v) )

#define FGPIO1PIN_NP1C_INITIN(v)      do{ bFM_GPIO_PCR1_PC=(v).bPullup; \
                                          bFM_GPIO_DDR1_PC=0u; \
                                          bFM_GPIO_PFR1_PC=0u; }while(0)

#define FGPIO1PIN_NP1C_INITOUT(v)     do{ bFM_GPIO_PDOR1_PC=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR1_PC=1u; \
                                          bFM_GPIO_PFR1_PC=0u; }while(0)


/*---- Fast GPIO bit P1D ----*/

#define FGPIO1PIN_P1D_GET             ( FM_FASTIO->FPDIR1_f.PD )

#define FGPIO1PIN_P1D_PUT(v)          ( FM_FASTIO->FPDOR1_f.PD=(v) )

#define FGPIO1PIN_P1D_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P1D_INITIN(v) \
                                                         : FGPIO1PIN_P1D_INITOUT(v) )

#define FGPIO1PIN_P1D_INITIN(v)       do{ bFM_GPIO_PCR1_PD=(v).bPullup; \
                                          bFM_GPIO_DDR1_PD=0u; \
                                          bFM_GPIO_PFR1_PD=0u; }while(0)

#define FGPIO1PIN_P1D_INITOUT(v)      do{ FM_FASTIO->FPDOR1_f.PD=(v).bInitVal; \
                                          bFM_GPIO_DDR1_PD=1u; \
                                          bFM_GPIO_PFR1_PD=0u; }while(0)


/*---- Fast GPIO bit NP1D ----*/
#define FGPIO1PIN_NP1D_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR1_f.PD)) )

#define FGPIO1PIN_NP1D_PUT(v)         ( FM_FASTIO->FPDOR1_f.PD=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP1D_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP1D_INITIN(v) \
                                                         : FGPIO1PIN_NP1D_INITOUT(v) )

#define FGPIO1PIN_NP1D_INITIN(v)      do{ bFM_GPIO_PCR1_PD=(v).bPullup; \
                                          bFM_GPIO_DDR1_PD=0u; \
                                          bFM_GPIO_PFR1_PD=0u; }while(0)

#define FGPIO1PIN_NP1D_INITOUT(v)     do{ bFM_GPIO_PDOR1_PD=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR1_PD=1u; \
                                          bFM_GPIO_PFR1_PD=0u; }while(0)


/*---- Fast GPIO bit P1E ----*/

#define FGPIO1PIN_P1E_GET             ( FM_FASTIO->FPDIR1_f.PE )

#define FGPIO1PIN_P1E_PUT(v)          ( FM_FASTIO->FPDOR1_f.PE=(v) )

#define FGPIO1PIN_P1E_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P1E_INITIN(v) \
                                                         : FGPIO1PIN_P1E_INITOUT(v) )

#define FGPIO1PIN_P1E_INITIN(v)       do{ bFM_GPIO_PCR1_PE=(v).bPullup; \
                                          bFM_GPIO_DDR1_PE=0u; \
                                          bFM_GPIO_PFR1_PE=0u; }while(0)

#define FGPIO1PIN_P1E_INITOUT(v)      do{ FM_FASTIO->FPDOR1_f.PE=(v).bInitVal; \
                                          bFM_GPIO_DDR1_PE=1u; \
                                          bFM_GPIO_PFR1_PE=0u; }while(0)


/*---- Fast GPIO bit NP1E ----*/
#define FGPIO1PIN_NP1E_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR1_f.PE)) )

#define FGPIO1PIN_NP1E_PUT(v)         ( FM_FASTIO->FPDOR1_f.PE=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP1E_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP1E_INITIN(v) \
                                                         : FGPIO1PIN_NP1E_INITOUT(v) )

#define FGPIO1PIN_NP1E_INITIN(v)      do{ bFM_GPIO_PCR1_PE=(v).bPullup; \
                                          bFM_GPIO_DDR1_PE=0u; \
                                          bFM_GPIO_PFR1_PE=0u; }while(0)

#define FGPIO1PIN_NP1E_INITOUT(v)     do{ bFM_GPIO_PDOR1_PE=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR1_PE=1u; \
                                          bFM_GPIO_PFR1_PE=0u; }while(0)


/*---- Fast GPIO bit P1F ----*/

#define FGPIO1PIN_P1F_GET             ( FM_FASTIO->FPDIR1_f.PF )

#define FGPIO1PIN_P1F_PUT(v)          ( FM_FASTIO->FPDOR1_f.PF=(v) )

#define FGPIO1PIN_P1F_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P1F_INITIN(v) \
                                                         : FGPIO1PIN_P1F_INITOUT(v) )

#define FGPIO1PIN_P1F_INITIN(v)       do{ bFM_GPIO_PCR1_PF=(v).bPullup; \
                                          bFM_GPIO_DDR1_PF=0u; \
                                          bFM_GPIO_PFR1_PF=0u; }while(0)

#define FGPIO1PIN_P1F_INITOUT(v)      do{ FM_FASTIO->FPDOR1_f.PF=(v).bInitVal; \
                                          bFM_GPIO_DDR1_PF=1u; \
                                          bFM_GPIO_PFR1_PF=0u; }while(0)


/*---- Fast GPIO bit NP1F ----*/
#define FGPIO1PIN_NP1F_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR1_f.PF)) )

#define FGPIO1PIN_NP1F_PUT(v)         ( FM_FASTIO->FPDOR1_f.PF=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP1F_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP1F_INITIN(v) \
                                                         : FGPIO1PIN_NP1F_INITOUT(v) )

#define FGPIO1PIN_NP1F_INITIN(v)      do{ bFM_GPIO_PCR1_PF=(v).bPullup; \
                                          bFM_GPIO_DDR1_PF=0u; \
                                          bFM_GPIO_PFR1_PF=0u; }while(0)

#define FGPIO1PIN_NP1F_INITOUT(v)     do{ bFM_GPIO_PDOR1_PF=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR1_PF=1u; \
                                          bFM_GPIO_PFR1_PF=0u; }while(0)


/*---- Fast GPIO bit P21 ----*/

#define FGPIO1PIN_P21_GET             ( FM_FASTIO->FPDIR2_f.P1 )

#define FGPIO1PIN_P21_PUT(v)          ( FM_FASTIO->FPDOR2_f.P1=(v) )

#define FGPIO1PIN_P21_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P21_INITIN(v) \
                                                         : FGPIO1PIN_P21_INITOUT(v) )

#define FGPIO1PIN_P21_INITIN(v)       do{ bFM_GPIO_PCR2_P1=(v).bPullup; \
                                          bFM_GPIO_DDR2_P1=0u; \
                                          bFM_GPIO_PFR2_P1=0u; }while(0)

#define FGPIO1PIN_P21_INITOUT(v)      do{ FM_FASTIO->FPDOR2_f.P1=(v).bInitVal; \
                                          bFM_GPIO_DDR2_P1=1u; \
                                          bFM_GPIO_PFR2_P1=0u; }while(0)


/*---- Fast GPIO bit NP21 ----*/
#define FGPIO1PIN_NP21_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR2_f.P1)) )

#define FGPIO1PIN_NP21_PUT(v)         ( FM_FASTIO->FPDOR2_f.P1=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP21_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP21_INITIN(v) \
                                                         : FGPIO1PIN_NP21_INITOUT(v) )

#define FGPIO1PIN_NP21_INITIN(v)      do{ bFM_GPIO_PCR2_P1=(v).bPullup; \
                                          bFM_GPIO_DDR2_P1=0u; \
                                          bFM_GPIO_PFR2_P1=0u; }while(0)

#define FGPIO1PIN_NP21_INITOUT(v)     do{ bFM_GPIO_PDOR2_P1=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR2_P1=1u; \
                                          bFM_GPIO_PFR2_P1=0u; }while(0)


/*---- Fast GPIO bit P22 ----*/

#define FGPIO1PIN_P22_GET             ( FM_FASTIO->FPDIR2_f.P2 )

#define FGPIO1PIN_P22_PUT(v)          ( FM_FASTIO->FPDOR2_f.P2=(v) )

#define FGPIO1PIN_P22_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P22_INITIN(v) \
                                                         : FGPIO1PIN_P22_INITOUT(v) )

#define FGPIO1PIN_P22_INITIN(v)       do{ bFM_GPIO_PCR2_P2=(v).bPullup; \
                                          bFM_GPIO_DDR2_P2=0u; \
                                          bFM_GPIO_PFR2_P2=0u; \
                                          bFM_GPIO_ADE_AN07=0u; }while(0)


#define FGPIO1PIN_P22_INITOUT(v)      do{ FM_FASTIO->FPDOR2_f.P2=(v).bInitVal; \
                                          bFM_GPIO_DDR2_P2=1u; \
                                          bFM_GPIO_PFR2_P2=0u; \
                                          bFM_GPIO_ADE_AN07=0u; }while(0)


/*---- Fast GPIO bit NP22 ----*/
#define FGPIO1PIN_NP22_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR2_f.P2)) )

#define FGPIO1PIN_NP22_PUT(v)         ( FM_FASTIO->FPDOR2_f.P2=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP22_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP22_INITIN(v) \
                                                         : FGPIO1PIN_NP22_INITOUT(v) )

#define FGPIO1PIN_NP22_INITIN(v)      do{ bFM_GPIO_PCR2_P2=(v).bPullup; \
                                          bFM_GPIO_DDR2_P2=0u; \
                                          bFM_GPIO_PFR2_P2=0u; \
                                          bFM_GPIO_ADE_AN07=0u; }while(0)


#define FGPIO1PIN_NP22_INITOUT(v)     do{ bFM_GPIO_PDOR2_P2=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR2_P2=1u; \
                                          bFM_GPIO_PFR2_P2=0u; \
                                          bFM_GPIO_ADE_AN07=0u; }while(0)


/*---- Fast GPIO bit P23 ----*/

#define FGPIO1PIN_P23_GET             ( FM_FASTIO->FPDIR2_f.P3 )

#define FGPIO1PIN_P23_PUT(v)          ( FM_FASTIO->FPDOR2_f.P3=(v) )

#define FGPIO1PIN_P23_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P23_INITIN(v) \
                                                         : FGPIO1PIN_P23_INITOUT(v) )

#define FGPIO1PIN_P23_INITIN(v)       do{ bFM_GPIO_PCR2_P3=(v).bPullup; \
                                          bFM_GPIO_DDR2_P3=0u; \
                                          bFM_GPIO_PFR2_P3=0u; \
                                          bFM_GPIO_ADE_AN06=0u; }while(0)


#define FGPIO1PIN_P23_INITOUT(v)      do{ FM_FASTIO->FPDOR2_f.P3=(v).bInitVal; \
                                          bFM_GPIO_DDR2_P3=1u; \
                                          bFM_GPIO_PFR2_P3=0u; \
                                          bFM_GPIO_ADE_AN06=0u; }while(0)


/*---- Fast GPIO bit NP23 ----*/
#define FGPIO1PIN_NP23_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR2_f.P3)) )

#define FGPIO1PIN_NP23_PUT(v)         ( FM_FASTIO->FPDOR2_f.P3=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP23_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP23_INITIN(v) \
                                                         : FGPIO1PIN_NP23_INITOUT(v) )

#define FGPIO1PIN_NP23_INITIN(v)      do{ bFM_GPIO_PCR2_P3=(v).bPullup; \
                                          bFM_GPIO_DDR2_P3=0u; \
                                          bFM_GPIO_PFR2_P3=0u; \
                                          bFM_GPIO_ADE_AN06=0u; }while(0)


#define FGPIO1PIN_NP23_INITOUT(v)     do{ bFM_GPIO_PDOR2_P3=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR2_P3=1u; \
                                          bFM_GPIO_PFR2_P3=0u; \
                                          bFM_GPIO_ADE_AN06=0u; }while(0)


/*---- Fast GPIO bit P30 ----*/

#define FGPIO1PIN_P30_GET             ( FM_FASTIO->FPDIR3_f.P0 )

#define FGPIO1PIN_P30_PUT(v)          ( FM_FASTIO->FPDOR3_f.P0=(v) )

#define FGPIO1PIN_P30_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P30_INITIN(v) \
                                                         : FGPIO1PIN_P30_INITOUT(v) )

#define FGPIO1PIN_P30_INITIN(v)       do{ bFM_GPIO_PCR3_P0=(v).bPullup; \
                                          bFM_GPIO_DDR3_P0=0u; \
                                          bFM_GPIO_PFR3_P0=0u; }while(0)

#define FGPIO1PIN_P30_INITOUT(v)      do{ FM_FASTIO->FPDOR3_f.P0=(v).bInitVal; \
                                          bFM_GPIO_DDR3_P0=1u; \
                                          bFM_GPIO_PFR3_P0=0u; }while(0)


/*---- Fast GPIO bit NP30 ----*/
#define FGPIO1PIN_NP30_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR3_f.P0)) )

#define FGPIO1PIN_NP30_PUT(v)         ( FM_FASTIO->FPDOR3_f.P0=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP30_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP30_INITIN(v) \
                                                         : FGPIO1PIN_NP30_INITOUT(v) )

#define FGPIO1PIN_NP30_INITIN(v)      do{ bFM_GPIO_PCR3_P0=(v).bPullup; \
                                          bFM_GPIO_DDR3_P0=0u; \
                                          bFM_GPIO_PFR3_P0=0u; }while(0)

#define FGPIO1PIN_NP30_INITOUT(v)     do{ bFM_GPIO_PDOR3_P0=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR3_P0=1u; \
                                          bFM_GPIO_PFR3_P0=0u; }while(0)


/*---- Fast GPIO bit P31 ----*/

#define FGPIO1PIN_P31_GET             ( FM_FASTIO->FPDIR3_f.P1 )

#define FGPIO1PIN_P31_PUT(v)          ( FM_FASTIO->FPDOR3_f.P1=(v) )

#define FGPIO1PIN_P31_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P31_INITIN(v) \
                                                         : FGPIO1PIN_P31_INITOUT(v) )

#define FGPIO1PIN_P31_INITIN(v)       do{ bFM_GPIO_PCR3_P1=(v).bPullup; \
                                          bFM_GPIO_DDR3_P1=0u; \
                                          bFM_GPIO_PFR3_P1=0u; }while(0)

#define FGPIO1PIN_P31_INITOUT(v)      do{ FM_FASTIO->FPDOR3_f.P1=(v).bInitVal; \
                                          bFM_GPIO_DDR3_P1=1u; \
                                          bFM_GPIO_PFR3_P1=0u; }while(0)


/*---- Fast GPIO bit NP31 ----*/
#define FGPIO1PIN_NP31_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR3_f.P1)) )

#define FGPIO1PIN_NP31_PUT(v)         ( FM_FASTIO->FPDOR3_f.P1=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP31_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP31_INITIN(v) \
                                                         : FGPIO1PIN_NP31_INITOUT(v) )

#define FGPIO1PIN_NP31_INITIN(v)      do{ bFM_GPIO_PCR3_P1=(v).bPullup; \
                                          bFM_GPIO_DDR3_P1=0u; \
                                          bFM_GPIO_PFR3_P1=0u; }while(0)

#define FGPIO1PIN_NP31_INITOUT(v)     do{ bFM_GPIO_PDOR3_P1=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR3_P1=1u; \
                                          bFM_GPIO_PFR3_P1=0u; }while(0)


/*---- Fast GPIO bit P32 ----*/

#define FGPIO1PIN_P32_GET             ( FM_FASTIO->FPDIR3_f.P2 )

#define FGPIO1PIN_P32_PUT(v)          ( FM_FASTIO->FPDOR3_f.P2=(v) )

#define FGPIO1PIN_P32_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P32_INITIN(v) \
                                                         : FGPIO1PIN_P32_INITOUT(v) )

#define FGPIO1PIN_P32_INITIN(v)       do{ bFM_GPIO_PCR3_P2=(v).bPullup; \
                                          bFM_GPIO_DDR3_P2=0u; \
                                          bFM_GPIO_PFR3_P2=0u; }while(0)

#define FGPIO1PIN_P32_INITOUT(v)      do{ FM_FASTIO->FPDOR3_f.P2=(v).bInitVal; \
                                          bFM_GPIO_DDR3_P2=1u; \
                                          bFM_GPIO_PFR3_P2=0u; }while(0)


/*---- Fast GPIO bit NP32 ----*/
#define FGPIO1PIN_NP32_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR3_f.P2)) )

#define FGPIO1PIN_NP32_PUT(v)         ( FM_FASTIO->FPDOR3_f.P2=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP32_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP32_INITIN(v) \
                                                         : FGPIO1PIN_NP32_INITOUT(v) )

#define FGPIO1PIN_NP32_INITIN(v)      do{ bFM_GPIO_PCR3_P2=(v).bPullup; \
                                          bFM_GPIO_DDR3_P2=0u; \
                                          bFM_GPIO_PFR3_P2=0u; }while(0)

#define FGPIO1PIN_NP32_INITOUT(v)     do{ bFM_GPIO_PDOR3_P2=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR3_P2=1u; \
                                          bFM_GPIO_PFR3_P2=0u; }while(0)


/*---- Fast GPIO bit P33 ----*/

#define FGPIO1PIN_P33_GET             ( FM_FASTIO->FPDIR3_f.P3 )

#define FGPIO1PIN_P33_PUT(v)          ( FM_FASTIO->FPDOR3_f.P3=(v) )

#define FGPIO1PIN_P33_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P33_INITIN(v) \
                                                         : FGPIO1PIN_P33_INITOUT(v) )

#define FGPIO1PIN_P33_INITIN(v)       do{ bFM_GPIO_PCR3_P3=(v).bPullup; \
                                          bFM_GPIO_DDR3_P3=0u; \
                                          bFM_GPIO_PFR3_P3=0u; }while(0)

#define FGPIO1PIN_P33_INITOUT(v)      do{ FM_FASTIO->FPDOR3_f.P3=(v).bInitVal; \
                                          bFM_GPIO_DDR3_P3=1u; \
                                          bFM_GPIO_PFR3_P3=0u; }while(0)


/*---- Fast GPIO bit NP33 ----*/
#define FGPIO1PIN_NP33_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR3_f.P3)) )

#define FGPIO1PIN_NP33_PUT(v)         ( FM_FASTIO->FPDOR3_f.P3=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP33_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP33_INITIN(v) \
                                                         : FGPIO1PIN_NP33_INITOUT(v) )

#define FGPIO1PIN_NP33_INITIN(v)      do{ bFM_GPIO_PCR3_P3=(v).bPullup; \
                                          bFM_GPIO_DDR3_P3=0u; \
                                          bFM_GPIO_PFR3_P3=0u; }while(0)

#define FGPIO1PIN_NP33_INITOUT(v)     do{ bFM_GPIO_PDOR3_P3=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR3_P3=1u; \
                                          bFM_GPIO_PFR3_P3=0u; }while(0)


/*---- Fast GPIO bit P34 ----*/

#define FGPIO1PIN_P34_GET             ( FM_FASTIO->FPDIR3_f.P4 )

#define FGPIO1PIN_P34_PUT(v)          ( FM_FASTIO->FPDOR3_f.P4=(v) )

#define FGPIO1PIN_P34_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P34_INITIN(v) \
                                                         : FGPIO1PIN_P34_INITOUT(v) )

#define FGPIO1PIN_P34_INITIN(v)       do{ bFM_GPIO_PCR3_P4=(v).bPullup; \
                                          bFM_GPIO_DDR3_P4=0u; \
                                          bFM_GPIO_PFR3_P4=0u; }while(0)

#define FGPIO1PIN_P34_INITOUT(v)      do{ FM_FASTIO->FPDOR3_f.P4=(v).bInitVal; \
                                          bFM_GPIO_DDR3_P4=1u; \
                                          bFM_GPIO_PFR3_P4=0u; }while(0)


/*---- Fast GPIO bit NP34 ----*/
#define FGPIO1PIN_NP34_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR3_f.P4)) )

#define FGPIO1PIN_NP34_PUT(v)         ( FM_FASTIO->FPDOR3_f.P4=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP34_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP34_INITIN(v) \
                                                         : FGPIO1PIN_NP34_INITOUT(v) )

#define FGPIO1PIN_NP34_INITIN(v)      do{ bFM_GPIO_PCR3_P4=(v).bPullup; \
                                          bFM_GPIO_DDR3_P4=0u; \
                                          bFM_GPIO_PFR3_P4=0u; }while(0)

#define FGPIO1PIN_NP34_INITOUT(v)     do{ bFM_GPIO_PDOR3_P4=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR3_P4=1u; \
                                          bFM_GPIO_PFR3_P4=0u; }while(0)


/*---- Fast GPIO bit P35 ----*/

#define FGPIO1PIN_P35_GET             ( FM_FASTIO->FPDIR3_f.P5 )

#define FGPIO1PIN_P35_PUT(v)          ( FM_FASTIO->FPDOR3_f.P5=(v) )

#define FGPIO1PIN_P35_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P35_INITIN(v) \
                                                         : FGPIO1PIN_P35_INITOUT(v) )

#define FGPIO1PIN_P35_INITIN(v)       do{ bFM_GPIO_PCR3_P5=(v).bPullup; \
                                          bFM_GPIO_DDR3_P5=0u; \
                                          bFM_GPIO_PFR3_P5=0u; }while(0)

#define FGPIO1PIN_P35_INITOUT(v)      do{ FM_FASTIO->FPDOR3_f.P5=(v).bInitVal; \
                                          bFM_GPIO_DDR3_P5=1u; \
                                          bFM_GPIO_PFR3_P5=0u; }while(0)


/*---- Fast GPIO bit NP35 ----*/
#define FGPIO1PIN_NP35_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR3_f.P5)) )

#define FGPIO1PIN_NP35_PUT(v)         ( FM_FASTIO->FPDOR3_f.P5=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP35_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP35_INITIN(v) \
                                                         : FGPIO1PIN_NP35_INITOUT(v) )

#define FGPIO1PIN_NP35_INITIN(v)      do{ bFM_GPIO_PCR3_P5=(v).bPullup; \
                                          bFM_GPIO_DDR3_P5=0u; \
                                          bFM_GPIO_PFR3_P5=0u; }while(0)

#define FGPIO1PIN_NP35_INITOUT(v)     do{ bFM_GPIO_PDOR3_P5=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR3_P5=1u; \
                                          bFM_GPIO_PFR3_P5=0u; }while(0)


/*---- Fast GPIO bit P3A ----*/

#define FGPIO1PIN_P3A_GET             ( FM_FASTIO->FPDIR3_f.PA )

#define FGPIO1PIN_P3A_PUT(v)          ( FM_FASTIO->FPDOR3_f.PA=(v) )

#define FGPIO1PIN_P3A_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P3A_INITIN(v) \
                                                         : FGPIO1PIN_P3A_INITOUT(v) )

#define FGPIO1PIN_P3A_INITIN(v)       do{ bFM_GPIO_PCR3_PA=(v).bPullup; \
                                          bFM_GPIO_DDR3_PA=0u; \
                                          bFM_GPIO_PFR3_PA=0u; }while(0)

#define FGPIO1PIN_P3A_INITOUT(v)      do{ FM_FASTIO->FPDOR3_f.PA=(v).bInitVal; \
                                          bFM_GPIO_DDR3_PA=1u; \
                                          bFM_GPIO_PFR3_PA=0u; }while(0)


/*---- Fast GPIO bit NP3A ----*/
#define FGPIO1PIN_NP3A_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR3_f.PA)) )

#define FGPIO1PIN_NP3A_PUT(v)         ( FM_FASTIO->FPDOR3_f.PA=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP3A_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP3A_INITIN(v) \
                                                         : FGPIO1PIN_NP3A_INITOUT(v) )

#define FGPIO1PIN_NP3A_INITIN(v)      do{ bFM_GPIO_PCR3_PA=(v).bPullup; \
                                          bFM_GPIO_DDR3_PA=0u; \
                                          bFM_GPIO_PFR3_PA=0u; }while(0)

#define FGPIO1PIN_NP3A_INITOUT(v)     do{ bFM_GPIO_PDOR3_PA=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR3_PA=1u; \
                                          bFM_GPIO_PFR3_PA=0u; }while(0)


/*---- Fast GPIO bit P3B ----*/

#define FGPIO1PIN_P3B_GET             ( FM_FASTIO->FPDIR3_f.PB )

#define FGPIO1PIN_P3B_PUT(v)          ( FM_FASTIO->FPDOR3_f.PB=(v) )

#define FGPIO1PIN_P3B_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P3B_INITIN(v) \
                                                         : FGPIO1PIN_P3B_INITOUT(v) )

#define FGPIO1PIN_P3B_INITIN(v)       do{ bFM_GPIO_PCR3_PB=(v).bPullup; \
                                          bFM_GPIO_DDR3_PB=0u; \
                                          bFM_GPIO_PFR3_PB=0u; }while(0)

#define FGPIO1PIN_P3B_INITOUT(v)      do{ FM_FASTIO->FPDOR3_f.PB=(v).bInitVal; \
                                          bFM_GPIO_DDR3_PB=1u; \
                                          bFM_GPIO_PFR3_PB=0u; }while(0)


/*---- Fast GPIO bit NP3B ----*/
#define FGPIO1PIN_NP3B_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR3_f.PB)) )

#define FGPIO1PIN_NP3B_PUT(v)         ( FM_FASTIO->FPDOR3_f.PB=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP3B_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP3B_INITIN(v) \
                                                         : FGPIO1PIN_NP3B_INITOUT(v) )

#define FGPIO1PIN_NP3B_INITIN(v)      do{ bFM_GPIO_PCR3_PB=(v).bPullup; \
                                          bFM_GPIO_DDR3_PB=0u; \
                                          bFM_GPIO_PFR3_PB=0u; }while(0)

#define FGPIO1PIN_NP3B_INITOUT(v)     do{ bFM_GPIO_PDOR3_PB=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR3_PB=1u; \
                                          bFM_GPIO_PFR3_PB=0u; }while(0)


/*---- Fast GPIO bit P3C ----*/

#define FGPIO1PIN_P3C_GET             ( FM_FASTIO->FPDIR3_f.PC )

#define FGPIO1PIN_P3C_PUT(v)          ( FM_FASTIO->FPDOR3_f.PC=(v) )

#define FGPIO1PIN_P3C_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P3C_INITIN(v) \
                                                         : FGPIO1PIN_P3C_INITOUT(v) )

#define FGPIO1PIN_P3C_INITIN(v)       do{ bFM_GPIO_PCR3_PC=(v).bPullup; \
                                          bFM_GPIO_DDR3_PC=0u; \
                                          bFM_GPIO_PFR3_PC=0u; }while(0)

#define FGPIO1PIN_P3C_INITOUT(v)      do{ FM_FASTIO->FPDOR3_f.PC=(v).bInitVal; \
                                          bFM_GPIO_DDR3_PC=1u; \
                                          bFM_GPIO_PFR3_PC=0u; }while(0)


/*---- Fast GPIO bit NP3C ----*/
#define FGPIO1PIN_NP3C_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR3_f.PC)) )

#define FGPIO1PIN_NP3C_PUT(v)         ( FM_FASTIO->FPDOR3_f.PC=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP3C_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP3C_INITIN(v) \
                                                         : FGPIO1PIN_NP3C_INITOUT(v) )

#define FGPIO1PIN_NP3C_INITIN(v)      do{ bFM_GPIO_PCR3_PC=(v).bPullup; \
                                          bFM_GPIO_DDR3_PC=0u; \
                                          bFM_GPIO_PFR3_PC=0u; }while(0)

#define FGPIO1PIN_NP3C_INITOUT(v)     do{ bFM_GPIO_PDOR3_PC=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR3_PC=1u; \
                                          bFM_GPIO_PFR3_PC=0u; }while(0)


/*---- Fast GPIO bit P3D ----*/

#define FGPIO1PIN_P3D_GET             ( FM_FASTIO->FPDIR3_f.PD )

#define FGPIO1PIN_P3D_PUT(v)          ( FM_FASTIO->FPDOR3_f.PD=(v) )

#define FGPIO1PIN_P3D_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P3D_INITIN(v) \
                                                         : FGPIO1PIN_P3D_INITOUT(v) )

#define FGPIO1PIN_P3D_INITIN(v)       do{ bFM_GPIO_PCR3_PD=(v).bPullup; \
                                          bFM_GPIO_DDR3_PD=0u; \
                                          bFM_GPIO_PFR3_PD=0u; }while(0)

#define FGPIO1PIN_P3D_INITOUT(v)      do{ FM_FASTIO->FPDOR3_f.PD=(v).bInitVal; \
                                          bFM_GPIO_DDR3_PD=1u; \
                                          bFM_GPIO_PFR3_PD=0u; }while(0)


/*---- Fast GPIO bit NP3D ----*/
#define FGPIO1PIN_NP3D_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR3_f.PD)) )

#define FGPIO1PIN_NP3D_PUT(v)         ( FM_FASTIO->FPDOR3_f.PD=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP3D_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP3D_INITIN(v) \
                                                         : FGPIO1PIN_NP3D_INITOUT(v) )

#define FGPIO1PIN_NP3D_INITIN(v)      do{ bFM_GPIO_PCR3_PD=(v).bPullup; \
                                          bFM_GPIO_DDR3_PD=0u; \
                                          bFM_GPIO_PFR3_PD=0u; }while(0)

#define FGPIO1PIN_NP3D_INITOUT(v)     do{ bFM_GPIO_PDOR3_PD=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR3_PD=1u; \
                                          bFM_GPIO_PFR3_PD=0u; }while(0)


/*---- Fast GPIO bit P3E ----*/

#define FGPIO1PIN_P3E_GET             ( FM_FASTIO->FPDIR3_f.PE )

#define FGPIO1PIN_P3E_PUT(v)          ( FM_FASTIO->FPDOR3_f.PE=(v) )

#define FGPIO1PIN_P3E_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P3E_INITIN(v) \
                                                         : FGPIO1PIN_P3E_INITOUT(v) )

#define FGPIO1PIN_P3E_INITIN(v)       do{ bFM_GPIO_PCR3_PE=(v).bPullup; \
                                          bFM_GPIO_DDR3_PE=0u; \
                                          bFM_GPIO_PFR3_PE=0u; }while(0)

#define FGPIO1PIN_P3E_INITOUT(v)      do{ FM_FASTIO->FPDOR3_f.PE=(v).bInitVal; \
                                          bFM_GPIO_DDR3_PE=1u; \
                                          bFM_GPIO_PFR3_PE=0u; }while(0)


/*---- Fast GPIO bit NP3E ----*/
#define FGPIO1PIN_NP3E_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR3_f.PE)) )

#define FGPIO1PIN_NP3E_PUT(v)         ( FM_FASTIO->FPDOR3_f.PE=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP3E_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP3E_INITIN(v) \
                                                         : FGPIO1PIN_NP3E_INITOUT(v) )

#define FGPIO1PIN_NP3E_INITIN(v)      do{ bFM_GPIO_PCR3_PE=(v).bPullup; \
                                          bFM_GPIO_DDR3_PE=0u; \
                                          bFM_GPIO_PFR3_PE=0u; }while(0)

#define FGPIO1PIN_NP3E_INITOUT(v)     do{ bFM_GPIO_PDOR3_PE=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR3_PE=1u; \
                                          bFM_GPIO_PFR3_PE=0u; }while(0)


/*---- Fast GPIO bit P3F ----*/

#define FGPIO1PIN_P3F_GET             ( FM_FASTIO->FPDIR3_f.PF )

#define FGPIO1PIN_P3F_PUT(v)          ( FM_FASTIO->FPDOR3_f.PF=(v) )

#define FGPIO1PIN_P3F_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P3F_INITIN(v) \
                                                         : FGPIO1PIN_P3F_INITOUT(v) )

#define FGPIO1PIN_P3F_INITIN(v)       do{ bFM_GPIO_PCR3_PF=(v).bPullup; \
                                          bFM_GPIO_DDR3_PF=0u; \
                                          bFM_GPIO_PFR3_PF=0u; }while(0)

#define FGPIO1PIN_P3F_INITOUT(v)      do{ FM_FASTIO->FPDOR3_f.PF=(v).bInitVal; \
                                          bFM_GPIO_DDR3_PF=1u; \
                                          bFM_GPIO_PFR3_PF=0u; }while(0)


/*---- Fast GPIO bit NP3F ----*/
#define FGPIO1PIN_NP3F_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR3_f.PF)) )

#define FGPIO1PIN_NP3F_PUT(v)         ( FM_FASTIO->FPDOR3_f.PF=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP3F_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP3F_INITIN(v) \
                                                         : FGPIO1PIN_NP3F_INITOUT(v) )

#define FGPIO1PIN_NP3F_INITIN(v)      do{ bFM_GPIO_PCR3_PF=(v).bPullup; \
                                          bFM_GPIO_DDR3_PF=0u; \
                                          bFM_GPIO_PFR3_PF=0u; }while(0)

#define FGPIO1PIN_NP3F_INITOUT(v)     do{ bFM_GPIO_PDOR3_PF=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR3_PF=1u; \
                                          bFM_GPIO_PFR3_PF=0u; }while(0)


/*---- Fast GPIO bit P40 ----*/

#define FGPIO1PIN_P40_GET             ( FM_FASTIO->FPDIR4_f.P0 )

#define FGPIO1PIN_P40_PUT(v)          ( FM_FASTIO->FPDOR4_f.P0=(v) )

#define FGPIO1PIN_P40_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P40_INITIN(v) \
                                                         : FGPIO1PIN_P40_INITOUT(v) )

#define FGPIO1PIN_P40_INITIN(v)       do{ bFM_GPIO_PCR4_P0=(v).bPullup; \
                                          bFM_GPIO_DDR4_P0=0u; \
                                          bFM_GPIO_PFR4_P0=0u; }while(0)

#define FGPIO1PIN_P40_INITOUT(v)      do{ FM_FASTIO->FPDOR4_f.P0=(v).bInitVal; \
                                          bFM_GPIO_DDR4_P0=1u; \
                                          bFM_GPIO_PFR4_P0=0u; }while(0)


/*---- Fast GPIO bit NP40 ----*/
#define FGPIO1PIN_NP40_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR4_f.P0)) )

#define FGPIO1PIN_NP40_PUT(v)         ( FM_FASTIO->FPDOR4_f.P0=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP40_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP40_INITIN(v) \
                                                         : FGPIO1PIN_NP40_INITOUT(v) )

#define FGPIO1PIN_NP40_INITIN(v)      do{ bFM_GPIO_PCR4_P0=(v).bPullup; \
                                          bFM_GPIO_DDR4_P0=0u; \
                                          bFM_GPIO_PFR4_P0=0u; }while(0)

#define FGPIO1PIN_NP40_INITOUT(v)     do{ bFM_GPIO_PDOR4_P0=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR4_P0=1u; \
                                          bFM_GPIO_PFR4_P0=0u; }while(0)


/*---- Fast GPIO bit P41 ----*/

#define FGPIO1PIN_P41_GET             ( FM_FASTIO->FPDIR4_f.P1 )

#define FGPIO1PIN_P41_PUT(v)          ( FM_FASTIO->FPDOR4_f.P1=(v) )

#define FGPIO1PIN_P41_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P41_INITIN(v) \
                                                         : FGPIO1PIN_P41_INITOUT(v) )

#define FGPIO1PIN_P41_INITIN(v)       do{ bFM_GPIO_PCR4_P1=(v).bPullup; \
                                          bFM_GPIO_DDR4_P1=0u; \
                                          bFM_GPIO_PFR4_P1=0u; }while(0)

#define FGPIO1PIN_P41_INITOUT(v)      do{ FM_FASTIO->FPDOR4_f.P1=(v).bInitVal; \
                                          bFM_GPIO_DDR4_P1=1u; \
                                          bFM_GPIO_PFR4_P1=0u; }while(0)


/*---- Fast GPIO bit NP41 ----*/
#define FGPIO1PIN_NP41_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR4_f.P1)) )

#define FGPIO1PIN_NP41_PUT(v)         ( FM_FASTIO->FPDOR4_f.P1=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP41_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP41_INITIN(v) \
                                                         : FGPIO1PIN_NP41_INITOUT(v) )

#define FGPIO1PIN_NP41_INITIN(v)      do{ bFM_GPIO_PCR4_P1=(v).bPullup; \
                                          bFM_GPIO_DDR4_P1=0u; \
                                          bFM_GPIO_PFR4_P1=0u; }while(0)

#define FGPIO1PIN_NP41_INITOUT(v)     do{ bFM_GPIO_PDOR4_P1=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR4_P1=1u; \
                                          bFM_GPIO_PFR4_P1=0u; }while(0)


/*---- Fast GPIO bit P42 ----*/

#define FGPIO1PIN_P42_GET             ( FM_FASTIO->FPDIR4_f.P2 )

#define FGPIO1PIN_P42_PUT(v)          ( FM_FASTIO->FPDOR4_f.P2=(v) )

#define FGPIO1PIN_P42_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P42_INITIN(v) \
                                                         : FGPIO1PIN_P42_INITOUT(v) )

#define FGPIO1PIN_P42_INITIN(v)       do{ bFM_GPIO_PCR4_P2=(v).bPullup; \
                                          bFM_GPIO_DDR4_P2=0u; \
                                          bFM_GPIO_PFR4_P2=0u; }while(0)

#define FGPIO1PIN_P42_INITOUT(v)      do{ FM_FASTIO->FPDOR4_f.P2=(v).bInitVal; \
                                          bFM_GPIO_DDR4_P2=1u; \
                                          bFM_GPIO_PFR4_P2=0u; }while(0)


/*---- Fast GPIO bit NP42 ----*/
#define FGPIO1PIN_NP42_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR4_f.P2)) )

#define FGPIO1PIN_NP42_PUT(v)         ( FM_FASTIO->FPDOR4_f.P2=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP42_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP42_INITIN(v) \
                                                         : FGPIO1PIN_NP42_INITOUT(v) )

#define FGPIO1PIN_NP42_INITIN(v)      do{ bFM_GPIO_PCR4_P2=(v).bPullup; \
                                          bFM_GPIO_DDR4_P2=0u; \
                                          bFM_GPIO_PFR4_P2=0u; }while(0)

#define FGPIO1PIN_NP42_INITOUT(v)     do{ bFM_GPIO_PDOR4_P2=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR4_P2=1u; \
                                          bFM_GPIO_PFR4_P2=0u; }while(0)


/*---- Fast GPIO bit P43 ----*/

#define FGPIO1PIN_P43_GET             ( FM_FASTIO->FPDIR4_f.P3 )

#define FGPIO1PIN_P43_PUT(v)          ( FM_FASTIO->FPDOR4_f.P3=(v) )

#define FGPIO1PIN_P43_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P43_INITIN(v) \
                                                         : FGPIO1PIN_P43_INITOUT(v) )

#define FGPIO1PIN_P43_INITIN(v)       do{ bFM_GPIO_PCR4_P3=(v).bPullup; \
                                          bFM_GPIO_DDR4_P3=0u; \
                                          bFM_GPIO_PFR4_P3=0u; }while(0)

#define FGPIO1PIN_P43_INITOUT(v)      do{ FM_FASTIO->FPDOR4_f.P3=(v).bInitVal; \
                                          bFM_GPIO_DDR4_P3=1u; \
                                          bFM_GPIO_PFR4_P3=0u; }while(0)


/*---- Fast GPIO bit NP43 ----*/
#define FGPIO1PIN_NP43_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR4_f.P3)) )

#define FGPIO1PIN_NP43_PUT(v)         ( FM_FASTIO->FPDOR4_f.P3=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP43_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP43_INITIN(v) \
                                                         : FGPIO1PIN_NP43_INITOUT(v) )

#define FGPIO1PIN_NP43_INITIN(v)      do{ bFM_GPIO_PCR4_P3=(v).bPullup; \
                                          bFM_GPIO_DDR4_P3=0u; \
                                          bFM_GPIO_PFR4_P3=0u; }while(0)

#define FGPIO1PIN_NP43_INITOUT(v)     do{ bFM_GPIO_PDOR4_P3=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR4_P3=1u; \
                                          bFM_GPIO_PFR4_P3=0u; }while(0)


/*---- Fast GPIO bit P46 ----*/

#define FGPIO1PIN_P46_GET             ( FM_FASTIO->FPDIR4_f.P6 )

#define FGPIO1PIN_P46_PUT(v)          ( FM_FASTIO->FPDOR4_f.P6=(v) )

#define FGPIO1PIN_P46_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P46_INITIN(v) \
                                                         : FGPIO1PIN_P46_INITOUT(v) )

#define FGPIO1PIN_P46_INITIN(v)       do{ bFM_GPIO_PCR4_P6=(v).bPullup; \
                                          bFM_GPIO_DDR4_P6=0u; \
/*--- Notice: This function will configure both of P46/X0A and P47/X1A as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_DS->SUBOSC_CTL, 0u, 2u, 0x00u ); \
                                          bFM_GPIO_PFR4_P6=0u; }while(0)

#define FGPIO1PIN_P46_INITOUT(v)      do{ FM_FASTIO->FPDOR4_f.P6=(v).bInitVal; \
                                          bFM_GPIO_DDR4_P6=1u; \
/*--- Notice: This function will configure both of P46/X0A and P47/X1A as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_DS->SUBOSC_CTL, 0u, 2u, 0x00u ); \
                                          bFM_GPIO_PFR4_P6=0u; }while(0)


/*---- Fast GPIO bit NP46 ----*/
#define FGPIO1PIN_NP46_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR4_f.P6)) )

#define FGPIO1PIN_NP46_PUT(v)         ( FM_FASTIO->FPDOR4_f.P6=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP46_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP46_INITIN(v) \
                                                         : FGPIO1PIN_NP46_INITOUT(v) )

#define FGPIO1PIN_NP46_INITIN(v)      do{ bFM_GPIO_PCR4_P6=(v).bPullup; \
                                          bFM_GPIO_DDR4_P6=0u; \
/*--- Notice: This function will configure both of P46/X0A and P47/X1A as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_DS->SUBOSC_CTL, 0u, 2u, 0x00u ); \
                                          bFM_GPIO_PFR4_P6=0u; }while(0)

#define FGPIO1PIN_NP46_INITOUT(v)     do{ bFM_GPIO_PDOR4_P6=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR4_P6=1u; \
/*--- Notice: This function will configure both of P46/X0A and P47/X1A as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_DS->SUBOSC_CTL, 0u, 2u, 0x00u ); \
                                          bFM_GPIO_PFR4_P6=0u; }while(0)


/*---- Fast GPIO bit P47 ----*/

#define FGPIO1PIN_P47_GET             ( FM_FASTIO->FPDIR4_f.P7 )

#define FGPIO1PIN_P47_PUT(v)          ( FM_FASTIO->FPDOR4_f.P7=(v) )

#define FGPIO1PIN_P47_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P47_INITIN(v) \
                                                         : FGPIO1PIN_P47_INITOUT(v) )

#define FGPIO1PIN_P47_INITIN(v)       do{ bFM_GPIO_PCR4_P7=(v).bPullup; \
                                          bFM_GPIO_DDR4_P7=0u; \
/*--- Notice: This function will configure both of P46/X0A and P47/X1A as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_DS->SUBOSC_CTL, 0u, 2u, 0x00u ); \
                                          bFM_GPIO_PFR4_P7=0u; }while(0)

#define FGPIO1PIN_P47_INITOUT(v)      do{ FM_FASTIO->FPDOR4_f.P7=(v).bInitVal; \
                                          bFM_GPIO_DDR4_P7=1u; \
/*--- Notice: This function will configure both of P46/X0A and P47/X1A as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_DS->SUBOSC_CTL, 0u, 2u, 0x00u ); \
                                          bFM_GPIO_PFR4_P7=0u; }while(0)


/*---- Fast GPIO bit NP47 ----*/
#define FGPIO1PIN_NP47_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR4_f.P7)) )

#define FGPIO1PIN_NP47_PUT(v)         ( FM_FASTIO->FPDOR4_f.P7=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP47_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP47_INITIN(v) \
                                                         : FGPIO1PIN_NP47_INITOUT(v) )

#define FGPIO1PIN_NP47_INITIN(v)      do{ bFM_GPIO_PCR4_P7=(v).bPullup; \
                                          bFM_GPIO_DDR4_P7=0u; \
/*--- Notice: This function will configure both of P46/X0A and P47/X1A as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_DS->SUBOSC_CTL, 0u, 2u, 0x00u ); \
                                          bFM_GPIO_PFR4_P7=0u; }while(0)

#define FGPIO1PIN_NP47_INITOUT(v)     do{ bFM_GPIO_PDOR4_P7=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR4_P7=1u; \
/*--- Notice: This function will configure both of P46/X0A and P47/X1A as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_DS->SUBOSC_CTL, 0u, 2u, 0x00u ); \
                                          bFM_GPIO_PFR4_P7=0u; }while(0)


/*---- Fast GPIO bit P4C ----*/

#define FGPIO1PIN_P4C_GET             ( FM_FASTIO->FPDIR4_f.PC )

#define FGPIO1PIN_P4C_PUT(v)          ( FM_FASTIO->FPDOR4_f.PC=(v) )

#define FGPIO1PIN_P4C_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P4C_INITIN(v) \
                                                         : FGPIO1PIN_P4C_INITOUT(v) )

#define FGPIO1PIN_P4C_INITIN(v)       do{ bFM_GPIO_PCR4_PC=(v).bPullup; \
                                          bFM_GPIO_DDR4_PC=0u; \
                                          bFM_GPIO_PFR4_PC=0u; }while(0)

#define FGPIO1PIN_P4C_INITOUT(v)      do{ FM_FASTIO->FPDOR4_f.PC=(v).bInitVal; \
                                          bFM_GPIO_DDR4_PC=1u; \
                                          bFM_GPIO_PFR4_PC=0u; }while(0)


/*---- Fast GPIO bit NP4C ----*/
#define FGPIO1PIN_NP4C_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR4_f.PC)) )

#define FGPIO1PIN_NP4C_PUT(v)         ( FM_FASTIO->FPDOR4_f.PC=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP4C_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP4C_INITIN(v) \
                                                         : FGPIO1PIN_NP4C_INITOUT(v) )

#define FGPIO1PIN_NP4C_INITIN(v)      do{ bFM_GPIO_PCR4_PC=(v).bPullup; \
                                          bFM_GPIO_DDR4_PC=0u; \
                                          bFM_GPIO_PFR4_PC=0u; }while(0)

#define FGPIO1PIN_NP4C_INITOUT(v)     do{ bFM_GPIO_PDOR4_PC=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR4_PC=1u; \
                                          bFM_GPIO_PFR4_PC=0u; }while(0)


/*---- Fast GPIO bit P4D ----*/

#define FGPIO1PIN_P4D_GET             ( FM_FASTIO->FPDIR4_f.PD )

#define FGPIO1PIN_P4D_PUT(v)          ( FM_FASTIO->FPDOR4_f.PD=(v) )

#define FGPIO1PIN_P4D_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P4D_INITIN(v) \
                                                         : FGPIO1PIN_P4D_INITOUT(v) )

#define FGPIO1PIN_P4D_INITIN(v)       do{ bFM_GPIO_PCR4_PD=(v).bPullup; \
                                          bFM_GPIO_DDR4_PD=0u; \
                                          bFM_GPIO_PFR4_PD=0u; }while(0)

#define FGPIO1PIN_P4D_INITOUT(v)      do{ FM_FASTIO->FPDOR4_f.PD=(v).bInitVal; \
                                          bFM_GPIO_DDR4_PD=1u; \
                                          bFM_GPIO_PFR4_PD=0u; }while(0)


/*---- Fast GPIO bit NP4D ----*/
#define FGPIO1PIN_NP4D_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR4_f.PD)) )

#define FGPIO1PIN_NP4D_PUT(v)         ( FM_FASTIO->FPDOR4_f.PD=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP4D_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP4D_INITIN(v) \
                                                         : FGPIO1PIN_NP4D_INITOUT(v) )

#define FGPIO1PIN_NP4D_INITIN(v)      do{ bFM_GPIO_PCR4_PD=(v).bPullup; \
                                          bFM_GPIO_DDR4_PD=0u; \
                                          bFM_GPIO_PFR4_PD=0u; }while(0)

#define FGPIO1PIN_NP4D_INITOUT(v)     do{ bFM_GPIO_PDOR4_PD=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR4_PD=1u; \
                                          bFM_GPIO_PFR4_PD=0u; }while(0)


/*---- Fast GPIO bit P4E ----*/

#define FGPIO1PIN_P4E_GET             ( FM_FASTIO->FPDIR4_f.PE )

#define FGPIO1PIN_P4E_PUT(v)          ( FM_FASTIO->FPDOR4_f.PE=(v) )

#define FGPIO1PIN_P4E_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P4E_INITIN(v) \
                                                         : FGPIO1PIN_P4E_INITOUT(v) )

#define FGPIO1PIN_P4E_INITIN(v)       do{ bFM_GPIO_PCR4_PE=(v).bPullup; \
                                          bFM_GPIO_DDR4_PE=0u; \
                                          bFM_GPIO_PFR4_PE=0u; }while(0)

#define FGPIO1PIN_P4E_INITOUT(v)      do{ FM_FASTIO->FPDOR4_f.PE=(v).bInitVal; \
                                          bFM_GPIO_DDR4_PE=1u; \
                                          bFM_GPIO_PFR4_PE=0u; }while(0)


/*---- Fast GPIO bit NP4E ----*/
#define FGPIO1PIN_NP4E_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR4_f.PE)) )

#define FGPIO1PIN_NP4E_PUT(v)         ( FM_FASTIO->FPDOR4_f.PE=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP4E_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP4E_INITIN(v) \
                                                         : FGPIO1PIN_NP4E_INITOUT(v) )

#define FGPIO1PIN_NP4E_INITIN(v)      do{ bFM_GPIO_PCR4_PE=(v).bPullup; \
                                          bFM_GPIO_DDR4_PE=0u; \
                                          bFM_GPIO_PFR4_PE=0u; }while(0)

#define FGPIO1PIN_NP4E_INITOUT(v)     do{ bFM_GPIO_PDOR4_PE=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR4_PE=1u; \
                                          bFM_GPIO_PFR4_PE=0u; }while(0)


/*---- Fast GPIO bit P50 ----*/

#define FGPIO1PIN_P50_GET             ( FM_FASTIO->FPDIR5_f.P0 )

#define FGPIO1PIN_P50_PUT(v)          ( FM_FASTIO->FPDOR5_f.P0=(v) )

#define FGPIO1PIN_P50_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P50_INITIN(v) \
                                                         : FGPIO1PIN_P50_INITOUT(v) )

#define FGPIO1PIN_P50_INITIN(v)       do{ bFM_GPIO_PCR5_P0=(v).bPullup; \
                                          bFM_GPIO_DDR5_P0=0u; \
                                          bFM_GPIO_PFR5_P0=0u; }while(0)

#define FGPIO1PIN_P50_INITOUT(v)      do{ FM_FASTIO->FPDOR5_f.P0=(v).bInitVal; \
                                          bFM_GPIO_DDR5_P0=1u; \
                                          bFM_GPIO_PFR5_P0=0u; }while(0)


/*---- Fast GPIO bit NP50 ----*/
#define FGPIO1PIN_NP50_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR5_f.P0)) )

#define FGPIO1PIN_NP50_PUT(v)         ( FM_FASTIO->FPDOR5_f.P0=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP50_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP50_INITIN(v) \
                                                         : FGPIO1PIN_NP50_INITOUT(v) )

#define FGPIO1PIN_NP50_INITIN(v)      do{ bFM_GPIO_PCR5_P0=(v).bPullup; \
                                          bFM_GPIO_DDR5_P0=0u; \
                                          bFM_GPIO_PFR5_P0=0u; }while(0)

#define FGPIO1PIN_NP50_INITOUT(v)     do{ bFM_GPIO_PDOR5_P0=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR5_P0=1u; \
                                          bFM_GPIO_PFR5_P0=0u; }while(0)


/*---- Fast GPIO bit P51 ----*/

#define FGPIO1PIN_P51_GET             ( FM_FASTIO->FPDIR5_f.P1 )

#define FGPIO1PIN_P51_PUT(v)          ( FM_FASTIO->FPDOR5_f.P1=(v) )

#define FGPIO1PIN_P51_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P51_INITIN(v) \
                                                         : FGPIO1PIN_P51_INITOUT(v) )

#define FGPIO1PIN_P51_INITIN(v)       do{ bFM_GPIO_PCR5_P1=(v).bPullup; \
                                          bFM_GPIO_DDR5_P1=0u; \
                                          bFM_GPIO_PFR5_P1=0u; }while(0)

#define FGPIO1PIN_P51_INITOUT(v)      do{ FM_FASTIO->FPDOR5_f.P1=(v).bInitVal; \
                                          bFM_GPIO_DDR5_P1=1u; \
                                          bFM_GPIO_PFR5_P1=0u; }while(0)


/*---- Fast GPIO bit NP51 ----*/
#define FGPIO1PIN_NP51_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR5_f.P1)) )

#define FGPIO1PIN_NP51_PUT(v)         ( FM_FASTIO->FPDOR5_f.P1=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP51_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP51_INITIN(v) \
                                                         : FGPIO1PIN_NP51_INITOUT(v) )

#define FGPIO1PIN_NP51_INITIN(v)      do{ bFM_GPIO_PCR5_P1=(v).bPullup; \
                                          bFM_GPIO_DDR5_P1=0u; \
                                          bFM_GPIO_PFR5_P1=0u; }while(0)

#define FGPIO1PIN_NP51_INITOUT(v)     do{ bFM_GPIO_PDOR5_P1=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR5_P1=1u; \
                                          bFM_GPIO_PFR5_P1=0u; }while(0)


/*---- Fast GPIO bit P52 ----*/

#define FGPIO1PIN_P52_GET             ( FM_FASTIO->FPDIR5_f.P2 )

#define FGPIO1PIN_P52_PUT(v)          ( FM_FASTIO->FPDOR5_f.P2=(v) )

#define FGPIO1PIN_P52_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P52_INITIN(v) \
                                                         : FGPIO1PIN_P52_INITOUT(v) )

#define FGPIO1PIN_P52_INITIN(v)       do{ bFM_GPIO_PCR5_P2=(v).bPullup; \
                                          bFM_GPIO_DDR5_P2=0u; \
                                          bFM_GPIO_PFR5_P2=0u; }while(0)

#define FGPIO1PIN_P52_INITOUT(v)      do{ FM_FASTIO->FPDOR5_f.P2=(v).bInitVal; \
                                          bFM_GPIO_DDR5_P2=1u; \
                                          bFM_GPIO_PFR5_P2=0u; }while(0)


/*---- Fast GPIO bit NP52 ----*/
#define FGPIO1PIN_NP52_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR5_f.P2)) )

#define FGPIO1PIN_NP52_PUT(v)         ( FM_FASTIO->FPDOR5_f.P2=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP52_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP52_INITIN(v) \
                                                         : FGPIO1PIN_NP52_INITOUT(v) )

#define FGPIO1PIN_NP52_INITIN(v)      do{ bFM_GPIO_PCR5_P2=(v).bPullup; \
                                          bFM_GPIO_DDR5_P2=0u; \
                                          bFM_GPIO_PFR5_P2=0u; }while(0)

#define FGPIO1PIN_NP52_INITOUT(v)     do{ bFM_GPIO_PDOR5_P2=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR5_P2=1u; \
                                          bFM_GPIO_PFR5_P2=0u; }while(0)


/*---- Fast GPIO bit P53 ----*/

#define FGPIO1PIN_P53_GET             ( FM_FASTIO->FPDIR5_f.P3 )

#define FGPIO1PIN_P53_PUT(v)          ( FM_FASTIO->FPDOR5_f.P3=(v) )

#define FGPIO1PIN_P53_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P53_INITIN(v) \
                                                         : FGPIO1PIN_P53_INITOUT(v) )

#define FGPIO1PIN_P53_INITIN(v)       do{ bFM_GPIO_PCR5_P3=(v).bPullup; \
                                          bFM_GPIO_DDR5_P3=0u; \
                                          bFM_GPIO_PFR5_P3=0u; }while(0)

#define FGPIO1PIN_P53_INITOUT(v)      do{ FM_FASTIO->FPDOR5_f.P3=(v).bInitVal; \
                                          bFM_GPIO_DDR5_P3=1u; \
                                          bFM_GPIO_PFR5_P3=0u; }while(0)


/*---- Fast GPIO bit NP53 ----*/
#define FGPIO1PIN_NP53_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR5_f.P3)) )

#define FGPIO1PIN_NP53_PUT(v)         ( FM_FASTIO->FPDOR5_f.P3=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP53_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP53_INITIN(v) \
                                                         : FGPIO1PIN_NP53_INITOUT(v) )

#define FGPIO1PIN_NP53_INITIN(v)      do{ bFM_GPIO_PCR5_P3=(v).bPullup; \
                                          bFM_GPIO_DDR5_P3=0u; \
                                          bFM_GPIO_PFR5_P3=0u; }while(0)

#define FGPIO1PIN_NP53_INITOUT(v)     do{ bFM_GPIO_PDOR5_P3=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR5_P3=1u; \
                                          bFM_GPIO_PFR5_P3=0u; }while(0)


/*---- Fast GPIO bit P60 ----*/

#define FGPIO1PIN_P60_GET             ( FM_FASTIO->FPDIR6_f.P0 )

#define FGPIO1PIN_P60_PUT(v)          ( FM_FASTIO->FPDOR6_f.P0=(v) )

#define FGPIO1PIN_P60_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P60_INITIN(v) \
                                                         : FGPIO1PIN_P60_INITOUT(v) )

#define FGPIO1PIN_P60_INITIN(v)       do{ bFM_GPIO_PCR6_P0=(v).bPullup; \
                                          bFM_GPIO_DDR6_P0=0u; \
                                          bFM_GPIO_PFR6_P0=0u; }while(0)

#define FGPIO1PIN_P60_INITOUT(v)      do{ FM_FASTIO->FPDOR6_f.P0=(v).bInitVal; \
                                          bFM_GPIO_DDR6_P0=1u; \
                                          bFM_GPIO_PFR6_P0=0u; }while(0)


/*---- Fast GPIO bit NP60 ----*/
#define FGPIO1PIN_NP60_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR6_f.P0)) )

#define FGPIO1PIN_NP60_PUT(v)         ( FM_FASTIO->FPDOR6_f.P0=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP60_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP60_INITIN(v) \
                                                         : FGPIO1PIN_NP60_INITOUT(v) )

#define FGPIO1PIN_NP60_INITIN(v)      do{ bFM_GPIO_PCR6_P0=(v).bPullup; \
                                          bFM_GPIO_DDR6_P0=0u; \
                                          bFM_GPIO_PFR6_P0=0u; }while(0)

#define FGPIO1PIN_NP60_INITOUT(v)     do{ bFM_GPIO_PDOR6_P0=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR6_P0=1u; \
                                          bFM_GPIO_PFR6_P0=0u; }while(0)


/*---- Fast GPIO bit P61 ----*/

#define FGPIO1PIN_P61_GET             ( FM_FASTIO->FPDIR6_f.P1 )

#define FGPIO1PIN_P61_PUT(v)          ( FM_FASTIO->FPDOR6_f.P1=(v) )

#define FGPIO1PIN_P61_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P61_INITIN(v) \
                                                         : FGPIO1PIN_P61_INITOUT(v) )

#define FGPIO1PIN_P61_INITIN(v)       do{ bFM_GPIO_PCR6_P1=(v).bPullup; \
                                          bFM_GPIO_DDR6_P1=0u; \
                                          bFM_GPIO_PFR6_P1=0u; }while(0)

#define FGPIO1PIN_P61_INITOUT(v)      do{ FM_FASTIO->FPDOR6_f.P1=(v).bInitVal; \
                                          bFM_GPIO_DDR6_P1=1u; \
                                          bFM_GPIO_PFR6_P1=0u; }while(0)


/*---- Fast GPIO bit NP61 ----*/
#define FGPIO1PIN_NP61_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR6_f.P1)) )

#define FGPIO1PIN_NP61_PUT(v)         ( FM_FASTIO->FPDOR6_f.P1=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP61_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP61_INITIN(v) \
                                                         : FGPIO1PIN_NP61_INITOUT(v) )

#define FGPIO1PIN_NP61_INITIN(v)      do{ bFM_GPIO_PCR6_P1=(v).bPullup; \
                                          bFM_GPIO_DDR6_P1=0u; \
                                          bFM_GPIO_PFR6_P1=0u; }while(0)

#define FGPIO1PIN_NP61_INITOUT(v)     do{ bFM_GPIO_PDOR6_P1=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR6_P1=1u; \
                                          bFM_GPIO_PFR6_P1=0u; }while(0)


/*---- Fast GPIO bit P80 ----*/

#define FGPIO1PIN_P80_GET             ( FM_FASTIO->FPDIR8_f.P0 )

#define FGPIO1PIN_P80_PUT(v)          ( FM_FASTIO->FPDOR8_f.P0=(v) )

#define FGPIO1PIN_P80_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P80_INITIN(v) \
                                                         : FGPIO1PIN_P80_INITOUT(v) )

#define FGPIO1PIN_P80_INITIN(v)       do{ bFM_GPIO_PCR8_P0=(v).bPullup; \
                                          bFM_GPIO_DDR8_P0=0u; \
                                          bFM_GPIO_PFR8_P0=0u; }while(0)

#define FGPIO1PIN_P80_INITOUT(v)      do{ FM_FASTIO->FPDOR8_f.P0=(v).bInitVal; \
                                          bFM_GPIO_DDR8_P0=1u; \
                                          bFM_GPIO_PFR8_P0=0u; }while(0)


/*---- Fast GPIO bit NP80 ----*/
#define FGPIO1PIN_NP80_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR8_f.P0)) )

#define FGPIO1PIN_NP80_PUT(v)         ( FM_FASTIO->FPDOR8_f.P0=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP80_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP80_INITIN(v) \
                                                         : FGPIO1PIN_NP80_INITOUT(v) )

#define FGPIO1PIN_NP80_INITIN(v)      do{ bFM_GPIO_PCR8_P0=(v).bPullup; \
                                          bFM_GPIO_DDR8_P0=0u; \
                                          bFM_GPIO_PFR8_P0=0u; }while(0)

#define FGPIO1PIN_NP80_INITOUT(v)     do{ bFM_GPIO_PDOR8_P0=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR8_P0=1u; \
                                          bFM_GPIO_PFR8_P0=0u; }while(0)


/*---- Fast GPIO bit P81 ----*/

#define FGPIO1PIN_P81_GET             ( FM_FASTIO->FPDIR8_f.P1 )

#define FGPIO1PIN_P81_PUT(v)          ( FM_FASTIO->FPDOR8_f.P1=(v) )

#define FGPIO1PIN_P81_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P81_INITIN(v) \
                                                         : FGPIO1PIN_P81_INITOUT(v) )

#define FGPIO1PIN_P81_INITIN(v)       do{ bFM_GPIO_PCR8_P1=(v).bPullup; \
                                          bFM_GPIO_DDR8_P1=0u; \
                                          bFM_GPIO_PFR8_P1=0u; }while(0)

#define FGPIO1PIN_P81_INITOUT(v)      do{ FM_FASTIO->FPDOR8_f.P1=(v).bInitVal; \
                                          bFM_GPIO_DDR8_P1=1u; \
                                          bFM_GPIO_PFR8_P1=0u; }while(0)


/*---- Fast GPIO bit NP81 ----*/
#define FGPIO1PIN_NP81_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR8_f.P1)) )

#define FGPIO1PIN_NP81_PUT(v)         ( FM_FASTIO->FPDOR8_f.P1=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP81_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP81_INITIN(v) \
                                                         : FGPIO1PIN_NP81_INITOUT(v) )

#define FGPIO1PIN_NP81_INITIN(v)      do{ bFM_GPIO_PCR8_P1=(v).bPullup; \
                                          bFM_GPIO_DDR8_P1=0u; \
                                          bFM_GPIO_PFR8_P1=0u; }while(0)

#define FGPIO1PIN_NP81_INITOUT(v)     do{ bFM_GPIO_PDOR8_P1=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR8_P1=1u; \
                                          bFM_GPIO_PFR8_P1=0u; }while(0)


/*---- Fast GPIO bit PE2 ----*/

#define FGPIO1PIN_PE2_GET             ( FM_FASTIO->FPDIRE_f.P2 )

#define FGPIO1PIN_PE2_PUT(v)          ( FM_FASTIO->FPDORE_f.P2=(v) )

#define FGPIO1PIN_PE2_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_PE2_INITIN(v) \
                                                         : FGPIO1PIN_PE2_INITOUT(v) )

#define FGPIO1PIN_PE2_INITIN(v)       do{ bFM_GPIO_PCRE_P2=(v).bPullup; \
                                          bFM_GPIO_DDRE_P2=0u; \
/*--- Notice: This function will configure both of PE2/X0 and PE3/X1 as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_GPIO->SPSR, 2u, 2u, 0x00u ); \
                                          bFM_GPIO_PFRE_P2=0u; }while(0)

#define FGPIO1PIN_PE2_INITOUT(v)      do{ FM_FASTIO->FPDORE_f.P2=(v).bInitVal; \
                                          bFM_GPIO_DDRE_P2=1u; \
/*--- Notice: This function will configure both of PE2/X0 and PE3/X1 as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_GPIO->SPSR, 2u, 2u, 0x00u ); \
                                          bFM_GPIO_PFRE_P2=0u; }while(0)


/*---- Fast GPIO bit NPE2 ----*/
#define FGPIO1PIN_NPE2_GET            ( (uint8_t)(!(FM_FASTIO->FPDIRE_f.P2)) )

#define FGPIO1PIN_NPE2_PUT(v)         ( FM_FASTIO->FPDORE_f.P2=(uint8_t)(!(v)) )

#define FGPIO1PIN_NPE2_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NPE2_INITIN(v) \
                                                         : FGPIO1PIN_NPE2_INITOUT(v) )

#define FGPIO1PIN_NPE2_INITIN(v)      do{ bFM_GPIO_PCRE_P2=(v).bPullup; \
                                          bFM_GPIO_DDRE_P2=0u; \
/*--- Notice: This function will configure both of PE2/X0 and PE3/X1 as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_GPIO->SPSR, 2u, 2u, 0x00u ); \
                                          bFM_GPIO_PFRE_P2=0u; }while(0)

#define FGPIO1PIN_NPE2_INITOUT(v)     do{ bFM_GPIO_PDORE_P2=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDRE_P2=1u; \
/*--- Notice: This function will configure both of PE2/X0 and PE3/X1 as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_GPIO->SPSR, 2u, 2u, 0x00u ); \
                                          bFM_GPIO_PFRE_P2=0u; }while(0)


/*---- Fast GPIO bit PE3 ----*/

#define FGPIO1PIN_PE3_GET             ( FM_FASTIO->FPDIRE_f.P3 )

#define FGPIO1PIN_PE3_PUT(v)          ( FM_FASTIO->FPDORE_f.P3=(v) )

#define FGPIO1PIN_PE3_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_PE3_INITIN(v) \
                                                         : FGPIO1PIN_PE3_INITOUT(v) )

#define FGPIO1PIN_PE3_INITIN(v)       do{ bFM_GPIO_PCRE_P3=(v).bPullup; \
                                          bFM_GPIO_DDRE_P3=0u; \
/*--- Notice: This function will configure both of PE2/X0 and PE3/X1 as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_GPIO->SPSR, 2u, 2u, 0x00u ); \
                                          bFM_GPIO_PFRE_P3=0u; }while(0)

#define FGPIO1PIN_PE3_INITOUT(v)      do{ FM_FASTIO->FPDORE_f.P3=(v).bInitVal; \
                                          bFM_GPIO_DDRE_P3=1u; \
/*--- Notice: This function will configure both of PE2/X0 and PE3/X1 as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_GPIO->SPSR, 2u, 2u, 0x00u ); \
                                          bFM_GPIO_PFRE_P3=0u; }while(0)


/*---- Fast GPIO bit NPE3 ----*/
#define FGPIO1PIN_NPE3_GET            ( (uint8_t)(!(FM_FASTIO->FPDIRE_f.P3)) )

#define FGPIO1PIN_NPE3_PUT(v)         ( FM_FASTIO->FPDORE_f.P3=(uint8_t)(!(v)) )

#define FGPIO1PIN_NPE3_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NPE3_INITIN(v) \
                                                         : FGPIO1PIN_NPE3_INITOUT(v) )

#define FGPIO1PIN_NPE3_INITIN(v)      do{ bFM_GPIO_PCRE_P3=(v).bPullup; \
                                          bFM_GPIO_DDRE_P3=0u; \
/*--- Notice: This function will configure both of PE2/X0 and PE3/X1 as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_GPIO->SPSR, 2u, 2u, 0x00u ); \
                                          bFM_GPIO_PFRE_P3=0u; }while(0)

#define FGPIO1PIN_NPE3_INITOUT(v)     do{ bFM_GPIO_PDORE_P3=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDRE_P3=1u; \
/*--- Notice: This function will configure both of PE2/X0 and PE3/X1 as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_GPIO->SPSR, 2u, 2u, 0x00u ); \
                                          bFM_GPIO_PFRE_P3=0u; }while(0)



/******************************************************************************
   PIN RELOCATION
*******************************************************************************/

/*--- ADTG_5 for ADC#0 ---*/
#define SetPinFunc_ADTG_5_ADC0(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 12u, 4u, 0x06u ); \
                                                 bFM_GPIO_PFR1_PF = 1u; }while(0)

/*--- ADTG_5 for ADC#1 ---*/
#define SetPinFunc_ADTG_5_ADC1(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 16u, 4u, 0x06u ); \
                                                 bFM_GPIO_PFR1_PF = 1u; }while(0)

/*--- ADTG_5 for ADC#2 ---*/
#define SetPinFunc_ADTG_5_ADC2(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 20u, 4u, 0x06u ); \
                                                 bFM_GPIO_PFR1_PF = 1u; }while(0)

/*--- ADTG_6 for ADC#0 ---*/
#define SetPinFunc_ADTG_6_ADC0(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 12u, 4u, 0x07u ); \
                                                 bFM_GPIO_PFR3_P3 = 1u; }while(0)

/*--- ADTG_6 for ADC#1 ---*/
#define SetPinFunc_ADTG_6_ADC1(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 16u, 4u, 0x07u ); \
                                                 bFM_GPIO_PFR3_P3 = 1u; }while(0)

/*--- ADTG_6 for ADC#2 ---*/
#define SetPinFunc_ADTG_6_ADC2(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 20u, 4u, 0x07u ); \
                                                 bFM_GPIO_PFR3_P3 = 1u; }while(0)

/*--- ADTG_7 for ADC#0 ---*/
#define SetPinFunc_ADTG_7_ADC0(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 12u, 4u, 0x08u ); \
                                                 bFM_GPIO_PFR4_P3 = 1u; }while(0)

/*--- ADTG_7 for ADC#1 ---*/
#define SetPinFunc_ADTG_7_ADC1(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 16u, 4u, 0x08u ); \
                                                 bFM_GPIO_PFR4_P3 = 1u; }while(0)

/*--- ADTG_7 for ADC#2 ---*/
#define SetPinFunc_ADTG_7_ADC2(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 20u, 4u, 0x08u ); \
                                                 bFM_GPIO_PFR4_P3 = 1u; }while(0)

/*--- CEC0_0 ---*/
#define SetPinFunc_CEC0_0(dummy)             do{ PINRELOC_SET_EPFR( FM_DS->CEC_CTL, 0u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR1_PA = 1u; }while(0)

/*--- CEC1_0 ---*/
#define SetPinFunc_CEC1_0(dummy)             do{ PINRELOC_SET_EPFR( FM_DS->CEC_CTL, 2u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR6_P0 = 1u; }while(0)

/*--- CROUT_1 ---*/
#define SetPinFunc_CROUT_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR00, 1u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR0_PF = 1u; }while(0)

/*--- CTS4_1 ---*/
#define SetPinFunc_CTS4_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 2u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_PD = 1u; }while(0)

/*--- IC1_CIN_0 ---*/
#define SetPinFunc_IC1_CIN_0(dummy)          do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR33, 16u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_PA = 1u; }while(0)

/*--- IC1_CLK_0 ---*/
#define SetPinFunc_IC1_CLK_0(dummy)          do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR33, 26u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_PF = 1u; }while(0)

/*--- IC1_DATA_0 ---*/
#define SetPinFunc_IC1_DATA_0(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR33, 18u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_PB = 1u; }while(0)

/*--- IC1_RST_0 ---*/
#define SetPinFunc_IC1_RST_0(dummy)          do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR33, 20u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_PC = 1u; }while(0)

/*--- IC1_VCC_0 ---*/
#define SetPinFunc_IC1_VCC_0(dummy)          do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR33, 24u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_PE = 1u; }while(0)

/*--- IC1_VPEN_0 ---*/
#define SetPinFunc_IC1_VPEN_0(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR33, 22u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_PD = 1u; }while(0)

/*--- INT00_0 ---*/
#define SetPinFunc_INT00_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 0u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P0 = 1u; }while(0)

/*--- INT00_1 ---*/
#define SetPinFunc_INT00_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 0u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR0_P5 = 1u; }while(0)

/*--- INT01_0 ---*/
#define SetPinFunc_INT01_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 2u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P1 = 1u; }while(0)

/*--- INT02_0 ---*/
#define SetPinFunc_INT02_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 4u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P2 = 1u; }while(0)

/*--- INT02_1 ---*/
#define SetPinFunc_INT02_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 4u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P1 = 1u; \
                                                 bFM_GPIO_ADE_AN01 = 0u; }while(0)

/*--- INT03_0 ---*/
#define SetPinFunc_INT03_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 6u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_PA = 1u; }while(0)

/*--- INT03_1 ---*/
#define SetPinFunc_INT03_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 6u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P4 = 1u; \
                                                 bFM_GPIO_ADE_AN04 = 0u; }while(0)

/*--- INT03_2 ---*/
#define SetPinFunc_INT03_2(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 6u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR3_P0 = 1u; }while(0)

/*--- INT04_0 ---*/
#define SetPinFunc_INT04_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 8u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_P3 = 1u; }while(0)

/*--- INT04_2 ---*/
#define SetPinFunc_INT04_2(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 8u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR3_P1 = 1u; }while(0)

/*--- INT05_1 ---*/
#define SetPinFunc_INT05_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 10u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_PA = 1u; }while(0)

/*--- INT05_2 ---*/
#define SetPinFunc_INT05_2(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 10u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR3_P2 = 1u; }while(0)

/*--- INT06_1 ---*/
#define SetPinFunc_INT06_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 12u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P1 = 1u; }while(0)

/*--- INT06_2 ---*/
#define SetPinFunc_INT06_2(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 12u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR4_PE = 1u; }while(0)

/*--- INT07_2 ---*/
#define SetPinFunc_INT07_2(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 14u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR5_P3 = 1u; }while(0)

/*--- INT08_1 ---*/
#define SetPinFunc_INT08_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 16u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P5 = 1u; }while(0)

/*--- INT12_1 ---*/
#define SetPinFunc_INT12_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 24u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR4_P0 = 1u; }while(0)

/*--- INT13_1 ---*/
#define SetPinFunc_INT13_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 26u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR4_P1 = 1u; }while(0)

/*--- INT15_1 ---*/
#define SetPinFunc_INT15_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 30u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR6_P0 = 1u; }while(0)

/*--- MD1 ---*/
#define SetPinFunc_MD1(dummy)                do{ }while(0u)

/*--- MI2SCK4_1 ---*/
#define SetPinFunc_MI2SCK4_1(dummy)          do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR37, 4u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_PC = 1u; }while(0)

/*--- MI2SCK6_1 ---*/
#define SetPinFunc_MI2SCK6_1(dummy)          do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR38, 4u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P1 = 1u; }while(0)

/*--- MI2SDI4_1 ---*/
#define SetPinFunc_MI2SDI4_1(dummy)          do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR37, 8u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_PA = 1u; }while(0)

/*--- MI2SDI6_1 ---*/
#define SetPinFunc_MI2SDI6_1(dummy)          do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR38, 8u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P3 = 1u; }while(0)

/*--- MI2SDO4_1 ---*/
#define SetPinFunc_MI2SDO4_1(dummy)          do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR37, 10u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_PB = 1u; }while(0)

/*--- MI2SDO6_1 ---*/
#define SetPinFunc_MI2SDO6_1(dummy)          do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR38, 10u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P2 = 1u; }while(0)

/*--- MI2SMCK4_1_IN ---*/
#define SetPinFunc_MI2SMCK4_1_IN(dummy)      do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR37, 0u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_PE = 1u; }while(0)

/*--- MI2SMCK4_1_OUT ---*/
#define SetPinFunc_MI2SMCK4_1_OUT(dummy)     do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR37, 2u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_PE = 1u; }while(0)

/*--- MI2SMCK6_1_IN ---*/
#define SetPinFunc_MI2SMCK6_1_IN(dummy)      do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR38, 0u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P4 = 1u; }while(0)

/*--- MI2SMCK6_1_OUT ---*/
#define SetPinFunc_MI2SMCK6_1_OUT(dummy)     do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR38, 2u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P4 = 1u; }while(0)

/*--- MI2SWS4_1 ---*/
#define SetPinFunc_MI2SWS4_1(dummy)          do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR37, 6u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_PD = 1u; }while(0)

/*--- MI2SWS6_1 ---*/
#define SetPinFunc_MI2SWS6_1(dummy)          do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR38, 6u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P0 = 1u; }while(0)

/*--- NMIX ---*/
#define SetPinFunc_NMIX(dummy)               do{ bFM_GPIO_EPFR00_NMIS = 1; \
                                                 bFM_GPIO_PFR0_PF = 1u; }while(0)

/*--- RTCCO_0 ---*/
#define SetPinFunc_RTCCO_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR00, 4u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR0_PF = 1u; }while(0)

/*--- RTCCO_1 ---*/
#define SetPinFunc_RTCCO_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR00, 4u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P3 = 1u; \
                                                 bFM_GPIO_ADE_AN03 = 0u; }while(0)

/*--- RTCCO_2 ---*/
#define SetPinFunc_RTCCO_2(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR00, 4u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR3_PA = 1u; }while(0)

/*--- RTS4_1 ---*/
#define SetPinFunc_RTS4_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 0u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_PE = 1u; }while(0)

/*--- SCK0_0 ---*/
#define SetPinFunc_SCK0_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 8u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR2_P3 = 1u; \
                                                 bFM_GPIO_ADE_AN06 = 0u; }while(0)

/*--- SCK1_1 ---*/
#define SetPinFunc_SCK1_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 14u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P3 = 1u; \
                                                 bFM_GPIO_ADE_AN03 = 0u; }while(0)

/*--- SCK3_1 ---*/
#define SetPinFunc_SCK3_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 26u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR5_P2 = 1u; }while(0)

/*--- SCK4_1 ---*/
#define SetPinFunc_SCK4_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 8u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_PC = 1u; }while(0)

/*--- SCK6_1 ---*/
#define SetPinFunc_SCK6_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 20u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P1 = 1u; }while(0)

/*--- SCK7_1 ---*/
#define SetPinFunc_SCK7_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 26u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR4_PC = 1u; }while(0)

/*--- SCS10_1 ---*/
#define SetPinFunc_SCS10_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR22, 4u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P4 = 1u; \
                                                 bFM_GPIO_ADE_AN04 = 0u; }while(0)

/*--- SCS11_1 ---*/
#define SetPinFunc_SCS11_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR22, 6u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P5 = 1u; \
                                                 bFM_GPIO_ADE_AN05 = 0u; }while(0)

/*--- SCS60_1 ---*/
#define SetPinFunc_SCS60_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR23, 0u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P0 = 1u; }while(0)

/*--- SCS61_1 ---*/
#define SetPinFunc_SCS61_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR23, 2u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P4 = 1u; }while(0)

/*--- SCS62_1 ---*/
#define SetPinFunc_SCS62_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR23, 4u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P5 = 1u; }while(0)

/*--- SI2CSCL6_1 ---*/
#define SetPinFunc_SI2CSCL6_1(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR31, 24u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P1 = 1u; }while(0)

/*--- SI2CSDA6_1 ---*/
#define SetPinFunc_SI2CSDA6_1(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR31, 26u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P2 = 1u; }while(0)

/*--- SIN0_0 ---*/
#define SetPinFunc_SIN0_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 4u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR0_P3 = 1u; }while(0)

/*--- SIN0_1 ---*/
#define SetPinFunc_SIN0_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 4u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P4 = 1u; \
                                                 bFM_GPIO_ADE_AN04 = 0u; }while(0)

/*--- SIN1_1 ---*/
#define SetPinFunc_SIN1_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 10u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P1 = 1u; \
                                                 bFM_GPIO_ADE_AN01 = 0u; }while(0)

/*--- SIN3_1 ---*/
#define SetPinFunc_SIN3_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 22u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR5_P0 = 1u; }while(0)

/*--- SIN4_1 ---*/
#define SetPinFunc_SIN4_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 4u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_PA = 1u; }while(0)

/*--- SIN6_1 ---*/
#define SetPinFunc_SIN6_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 16u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P3 = 1u; }while(0)

/*--- SIN7_1 ---*/
#define SetPinFunc_SIN7_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 22u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR4_PE = 1u; }while(0)

/*--- SOT0_0 ---*/
#define SetPinFunc_SOT0_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 6u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR0_P1 = 1u; }while(0)

/*--- SOT0_1 ---*/
#define SetPinFunc_SOT0_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 6u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P5 = 1u; \
                                                 bFM_GPIO_ADE_AN05 = 0u; }while(0)

/*--- SOT1_1 ---*/
#define SetPinFunc_SOT1_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 12u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P2 = 1u; \
                                                 bFM_GPIO_ADE_AN02 = 0u; }while(0)

/*--- SOT3_1 ---*/
#define SetPinFunc_SOT3_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 24u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR5_P1 = 1u; }while(0)

/*--- SOT4_1 ---*/
#define SetPinFunc_SOT4_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 6u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_PB = 1u; }while(0)

/*--- SOT6_1 ---*/
#define SetPinFunc_SOT6_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 18u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P2 = 1u; }while(0)

/*--- SOT7_1 ---*/
#define SetPinFunc_SOT7_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 24u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR4_PD = 1u; }while(0)

/*--- SUBOUT_0 ---*/
#define SetPinFunc_SUBOUT_0(dummy)           do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR00, 6u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR0_PF = 1u; }while(0)

/*--- SUBOUT_1 ---*/
#define SetPinFunc_SUBOUT_1(dummy)           do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR00, 6u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P3 = 1u; \
                                                 bFM_GPIO_ADE_AN03 = 0u; }while(0)

/*--- SUBOUT_2 ---*/
#define SetPinFunc_SUBOUT_2(dummy)           do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR00, 6u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR3_PA = 1u; }while(0)

/*--- SWCLK ---*/
#define SetPinFunc_SWCLK(dummy)              do{ bFM_GPIO_EPFR00_SWDEN = 1; \
                                                 bFM_GPIO_PFR0_P1 = 1u; }while(0)

/*--- SWDIO ---*/
#define SetPinFunc_SWDIO(dummy)              do{ bFM_GPIO_EPFR00_SWDEN = 1; \
                                                 bFM_GPIO_PFR0_P3 = 1u; }while(0)

/*--- TIOA0_0_OUT ---*/
#define SetPinFunc_TIOA0_0_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 2u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR4_P0 = 1u; }while(0)

/*--- TIOA0_1_OUT ---*/
#define SetPinFunc_TIOA0_1_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 2u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_PA = 1u; }while(0)

/*--- TIOA1_0_IN ---*/
#define SetPinFunc_TIOA1_0_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 8u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR4_P1 = 1u; }while(0)

/*--- TIOA1_0_OUT ---*/
#define SetPinFunc_TIOA1_0_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 10u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR4_P1 = 1u; }while(0)

/*--- TIOA1_1_IN ---*/
#define SetPinFunc_TIOA1_1_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 8u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_PB = 1u; }while(0)

/*--- TIOA1_1_OUT ---*/
#define SetPinFunc_TIOA1_1_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 10u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_PB = 1u; }while(0)

/*--- TIOA1_2_IN ---*/
#define SetPinFunc_TIOA1_2_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 8u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR5_P3 = 1u; }while(0)

/*--- TIOA1_2_OUT ---*/
#define SetPinFunc_TIOA1_2_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 10u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR5_P3 = 1u; }while(0)

/*--- TIOA2_0_OUT ---*/
#define SetPinFunc_TIOA2_0_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 18u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR4_P2 = 1u; }while(0)

/*--- TIOA2_1_OUT ---*/
#define SetPinFunc_TIOA2_1_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 18u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_PC = 1u; }while(0)

/*--- TIOA2_2_OUT ---*/
#define SetPinFunc_TIOA2_2_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 18u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR6_P0 = 1u; }while(0)

/*--- TIOA3_0_IN ---*/
#define SetPinFunc_TIOA3_0_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 24u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR4_P3 = 1u; }while(0)

/*--- TIOA3_0_OUT ---*/
#define SetPinFunc_TIOA3_0_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 26u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR4_P3 = 1u; }while(0)

/*--- TIOA3_1_IN ---*/
#define SetPinFunc_TIOA3_1_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 24u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_PD = 1u; }while(0)

/*--- TIOA3_1_OUT ---*/
#define SetPinFunc_TIOA3_1_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 26u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_PD = 1u; }while(0)

/*--- TIOA4_1_OUT ---*/
#define SetPinFunc_TIOA4_1_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 2u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_PE = 1u; }while(0)

/*--- TIOA5_1_IN ---*/
#define SetPinFunc_TIOA5_1_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 8u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_PF = 1u; }while(0)

/*--- TIOA5_1_OUT ---*/
#define SetPinFunc_TIOA5_1_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 10u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_PF = 1u; }while(0)

/*--- TIOA5_2_IN ---*/
#define SetPinFunc_TIOA5_2_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 8u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR0_P5 = 1u; }while(0)

/*--- TIOA5_2_OUT ---*/
#define SetPinFunc_TIOA5_2_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 10u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR0_P5 = 1u; }while(0)

/*--- TIOA6_1_OUT ---*/
#define SetPinFunc_TIOA6_1_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 18u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR0_PC = 1u; }while(0)

/*--- TIOA7_1_IN ---*/
#define SetPinFunc_TIOA7_1_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 24u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P3 = 1u; \
                                                 bFM_GPIO_ADE_AN06 = 0u; }while(0)

/*--- TIOA7_1_OUT ---*/
#define SetPinFunc_TIOA7_1_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 26u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P3 = 1u; \
                                                 bFM_GPIO_ADE_AN06 = 0u; }while(0)

/*--- TIOB0_1_IN ---*/
#define SetPinFunc_TIOB0_1_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 4u, 3u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P0 = 1u; }while(0)

/*--- TIOB2_1_IN ---*/
#define SetPinFunc_TIOB2_1_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 20u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P2 = 1u; }while(0)

/*--- TIOB2_2_IN ---*/
#define SetPinFunc_TIOB2_2_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 20u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR6_P1 = 1u; }while(0)

/*--- TIOB3_0_IN ---*/
#define SetPinFunc_TIOB3_0_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 28u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR4_PC = 1u; }while(0)

/*--- TIOB4_1_IN ---*/
#define SetPinFunc_TIOB4_1_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 4u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P4 = 1u; }while(0)

/*--- TIOB5_1_IN ---*/
#define SetPinFunc_TIOB5_1_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 12u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P5 = 1u; }while(0)

/*--- TIOB6_1_IN ---*/
#define SetPinFunc_TIOB6_1_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 20u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR0_PB = 1u; }while(0)

/*--- TIOB7_0_IN ---*/
#define SetPinFunc_TIOB7_0_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 28u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR0_P3 = 1u; }while(0)

/*--- TIOB7_1_IN ---*/
#define SetPinFunc_TIOB7_1_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 28u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P2 = 1u; \
                                                 bFM_GPIO_ADE_AN07 = 0u; }while(0)

/*--- UDM0 ---*/
#define SetPinFunc_UDM0(dummy)               do{ bFM_GPIO_SPSR_USB0C = 1u; }while(0u)

/*--- UDP0 ---*/
#define SetPinFunc_UDP0(dummy)               do{ bFM_GPIO_SPSR_USB0C = 1u; }while(0u)

/*--- UHCONX0 ---*/
#define SetPinFunc_UHCONX0(dummy)            do{ bFM_GPIO_EPFR00_USBP0E = 1; \
                                                 bFM_GPIO_PFR6_P1 = 1u; }while(0)

/*--- WKUP0 ---*/
#define SetPinFunc_WKUP0(dummy)              do{ }while(0u)

/*--- WKUP1 ---*/
#define SetPinFunc_WKUP1(dummy)              do{ }while(0u)

/*--- WKUP2 ---*/
#define SetPinFunc_WKUP2(dummy)              do{ }while(0u)

/*--- WKUP3 ---*/
#define SetPinFunc_WKUP3(dummy)              do{ }while(0u)

/*--- WKUP4 ---*/
#define SetPinFunc_WKUP4(dummy)              do{ }while(0u)

/*--- WKUP5 ---*/
#define SetPinFunc_WKUP5(dummy)              do{ }while(0u)

/*--- WKUP6 ---*/
#define SetPinFunc_WKUP6(dummy)              do{ }while(0u)

/*--- WKUP7 ---*/
#define SetPinFunc_WKUP7(dummy)              do{ }while(0u)

/*--- X0 ---*/
/*--- Notice: This function will configure X0 and X1 as clock pins. ---*/
#define SetPinFunc_X0(dummy)                 do{ PINRELOC_SET_EPFR( FM_GPIO->SPSR, 2u, 2u, 0x01u ); }while(0u)

/*--- X0A ---*/
/*--- Notice: This function will configure X0A and X1A as clock pins. ---*/
#define SetPinFunc_X0A(dummy)                do{ PINRELOC_SET_EPFR( FM_DS->SUBOSC_CTL, 0u, 2u, 0x01u ); }while(0u)

/*--- X1 ---*/
/*--- Notice: This function will configure X0 and X1 as clock pins. ---*/
#define SetPinFunc_X1(dummy)                 do{ PINRELOC_SET_EPFR( FM_GPIO->SPSR, 2u, 2u, 0x01u ); }while(0u)

/*--- X1A ---*/
/*--- Notice: This function will configure X0A and X1A as clock pins. ---*/
#define SetPinFunc_X1A(dummy)                do{ PINRELOC_SET_EPFR( FM_DS->SUBOSC_CTL, 0u, 2u, 0x01u ); }while(0u)


/******************************************************************************
   ANALOG PINS
*******************************************************************************/

/*--- AN00 ---*/
#define SetPinFunc_AN00(dummy)               ( bFM_GPIO_ADE_AN00 = 1 )

/*--- AN01 ---*/
#define SetPinFunc_AN01(dummy)               ( bFM_GPIO_ADE_AN01 = 1 )

/*--- AN02 ---*/
#define SetPinFunc_AN02(dummy)               ( bFM_GPIO_ADE_AN02 = 1 )

/*--- AN03 ---*/
#define SetPinFunc_AN03(dummy)               ( bFM_GPIO_ADE_AN03 = 1 )

/*--- AN04 ---*/
#define SetPinFunc_AN04(dummy)               ( bFM_GPIO_ADE_AN04 = 1 )

/*--- AN05 ---*/
#define SetPinFunc_AN05(dummy)               ( bFM_GPIO_ADE_AN05 = 1 )

/*--- AN06 ---*/
#define SetPinFunc_AN06(dummy)               ( bFM_GPIO_ADE_AN06 = 1 )

/*--- AN07 ---*/
#define SetPinFunc_AN07(dummy)               ( bFM_GPIO_ADE_AN07 = 1 )


#endif // #ifndef __GPIO_S6E1C1XD_H__


/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/

