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
/** \file gpio_s6e1a1xb.h
 **
 ** Headerfile for S6E1A1XB GPIO functions, included in gpio.h
 **
 ** History:
 **   - 2013-12-04  1.0  Edison Zhang    First version
 **   - 2014-05-14  1.1  Edison Zhang	 Update P46/P47, PE2/PE3 initialization
 **
 ******************************************************************************/

#ifndef __GPIO_S6E1A1XB_H__
#define __GPIO_S6E1A1XB_H__

#include <stdint.h>

/******************************************************************************
   GPIO
*******************************************************************************/

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


/*---- GPIO bit P46 ----*/

#define GPIO1PIN_P46_GET             ( bFM_GPIO_PDIR4_P6 )

#define GPIO1PIN_P46_PUT(v)          ( bFM_GPIO_PDOR4_P6=(v) )

#define GPIO1PIN_P46_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P46_INITIN(v) \
                                                        : GPIO1PIN_P46_INITOUT(v) )

#define GPIO1PIN_P46_INITIN(v)       do{ bFM_GPIO_PCR4_P6=(v).bPullup; \
                                         bFM_GPIO_DDR4_P6=0u; \
/*--- Notice: This function will configure both of P46/X0A and P47/X1A as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_GPIO->SPSR, 0u, 2u, 0x00u ); \
                                         bFM_GPIO_PFR4_P6=0u; }while(0u)

#define GPIO1PIN_P46_INITOUT(v)      do{ bFM_GPIO_PDOR4_P6=(v).bInitVal; \
                                         bFM_GPIO_DDR4_P6=1u; \
/*--- Notice: This function will configure both of P46/X0A and P47/X1A as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_GPIO->SPSR, 0u, 2u, 0x00u ); \
                                         bFM_GPIO_PFR4_P6=0u; }while(0u)


/*---- GPIO bit NP46 ----*/
#define GPIO1PIN_NP46_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR4_P6)) )

#define GPIO1PIN_NP46_PUT(v)         ( bFM_GPIO_PDOR4_P6=(uint32_t)(!(v)) )

#define GPIO1PIN_NP46_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP46_INITIN(v) \
                                                        : GPIO1PIN_NP46_INITOUT(v) )

#define GPIO1PIN_NP46_INITIN(v)      do{ bFM_GPIO_PCR4_P6=(v).bPullup; \
                                         bFM_GPIO_DDR4_P6=0u; \
/*--- Notice: This function will configure both of P46/X0A and P47/X1A as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_GPIO->SPSR, 0u, 2u, 0x00u ); \
                                         bFM_GPIO_PFR4_P6=0u; }while(0u)

#define GPIO1PIN_NP46_INITOUT(v)     do{ bFM_GPIO_PDOR4_P6=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR4_P6=1u; \
/*--- Notice: This function will configure both of P46/X0A and P47/X1A as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_GPIO->SPSR, 0u, 2u, 0x00u ); \
                                         bFM_GPIO_PFR4_P6=0u; }while(0u)


/*---- GPIO bit P47 ----*/

#define GPIO1PIN_P47_GET             ( bFM_GPIO_PDIR4_P7 )

#define GPIO1PIN_P47_PUT(v)          ( bFM_GPIO_PDOR4_P7=(v) )

#define GPIO1PIN_P47_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P47_INITIN(v) \
                                                        : GPIO1PIN_P47_INITOUT(v) )

#define GPIO1PIN_P47_INITIN(v)       do{ bFM_GPIO_PCR4_P7=(v).bPullup; \
                                         bFM_GPIO_DDR4_P7=0u; \
/*--- Notice: This function will configure both of P46/X0A and P47/X1A as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_GPIO->SPSR, 0u, 2u, 0x00u ); \
                                         bFM_GPIO_PFR4_P7=0u; }while(0u)

#define GPIO1PIN_P47_INITOUT(v)      do{ bFM_GPIO_PDOR4_P7=(v).bInitVal; \
                                         bFM_GPIO_DDR4_P7=1u; \
/*--- Notice: This function will configure both of P46/X0A and P47/X1A as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_GPIO->SPSR, 0u, 2u, 0x00u ); \
                                         bFM_GPIO_PFR4_P7=0u; }while(0u)


/*---- GPIO bit NP47 ----*/
#define GPIO1PIN_NP47_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR4_P7)) )

#define GPIO1PIN_NP47_PUT(v)         ( bFM_GPIO_PDOR4_P7=(uint32_t)(!(v)) )

#define GPIO1PIN_NP47_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP47_INITIN(v) \
                                                        : GPIO1PIN_NP47_INITOUT(v) )

#define GPIO1PIN_NP47_INITIN(v)      do{ bFM_GPIO_PCR4_P7=(v).bPullup; \
                                         bFM_GPIO_DDR4_P7=0u; \
/*--- Notice: This function will configure both of P46/X0A and P47/X1A as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_GPIO->SPSR, 0u, 2u, 0x00u ); \
                                         bFM_GPIO_PFR4_P7=0u; }while(0u)

#define GPIO1PIN_NP47_INITOUT(v)     do{ bFM_GPIO_PDOR4_P7=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR4_P7=1u; \
/*--- Notice: This function will configure both of P46/X0A and P47/X1A as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_GPIO->SPSR, 0u, 2u, 0x00u ); \
                                         bFM_GPIO_PFR4_P7=0u; }while(0u)


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
   Fast GPIO
*******************************************************************************/

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


/*---- Fast GPIO bit P04 ----*/

#define FGPIO1PIN_P04_GET             ( FM_FASTIO->FPDIR0_f.P4 )

#define FGPIO1PIN_P04_PUT(v)          ( FM_FASTIO->FPDOR0_f.P4=(v) )

#define FGPIO1PIN_P04_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P04_INITIN(v) \
                                                         : FGPIO1PIN_P04_INITOUT(v) )

#define FGPIO1PIN_P04_INITIN(v)       do{ bFM_GPIO_PCR0_P4=(v).bPullup; \
                                          bFM_GPIO_DDR0_P4=0u; \
                                          bFM_GPIO_PFR0_P4=0u; }while(0)

#define FGPIO1PIN_P04_INITOUT(v)      do{ FM_FASTIO->FPDOR0_f.P4=(v).bInitVal; \
                                          bFM_GPIO_DDR0_P4=1u; \
                                          bFM_GPIO_PFR0_P4=0u; }while(0)


/*---- Fast GPIO bit NP04 ----*/
#define FGPIO1PIN_NP04_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR0_f.P4)) )

#define FGPIO1PIN_NP04_PUT(v)         ( FM_FASTIO->FPDOR0_f.P4=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP04_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP04_INITIN(v) \
                                                         : FGPIO1PIN_NP04_INITOUT(v) )

#define FGPIO1PIN_NP04_INITIN(v)      do{ bFM_GPIO_PCR0_P4=(v).bPullup; \
                                          bFM_GPIO_DDR0_P4=0u; \
                                          bFM_GPIO_PFR0_P4=0u; }while(0)

#define FGPIO1PIN_NP04_INITOUT(v)     do{ bFM_GPIO_PDOR0_P4=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR0_P4=1u; \
                                          bFM_GPIO_PFR0_P4=0u; }while(0)


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


/*---- Fast GPIO bit P46 ----*/

#define FGPIO1PIN_P46_GET             ( FM_FASTIO->FPDIR4_f.P6 )

#define FGPIO1PIN_P46_PUT(v)          ( FM_FASTIO->FPDOR4_f.P6=(v) )

#define FGPIO1PIN_P46_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P46_INITIN(v) \
                                                         : FGPIO1PIN_P46_INITOUT(v) )

#define FGPIO1PIN_P46_INITIN(v)       do{ bFM_GPIO_PCR4_P6=(v).bPullup; \
                                          bFM_GPIO_DDR4_P6=0u; \
/*--- Notice: This function will configure both of P46/X0A and P47/X1A as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_GPIO->SPSR, 0u, 2u, 0x00u ); \
                                          bFM_GPIO_PFR4_P6=0u; }while(0)

#define FGPIO1PIN_P46_INITOUT(v)      do{ FM_FASTIO->FPDOR4_f.P6=(v).bInitVal; \
                                          bFM_GPIO_DDR4_P6=1u; \
/*--- Notice: This function will configure both of P46/X0A and P47/X1A as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_GPIO->SPSR, 0u, 2u, 0x00u ); \
                                          bFM_GPIO_PFR4_P6=0u; }while(0)


/*---- Fast GPIO bit NP46 ----*/
#define FGPIO1PIN_NP46_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR4_f.P6)) )

#define FGPIO1PIN_NP46_PUT(v)         ( FM_FASTIO->FPDOR4_f.P6=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP46_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP46_INITIN(v) \
                                                         : FGPIO1PIN_NP46_INITOUT(v) )

#define FGPIO1PIN_NP46_INITIN(v)      do{ bFM_GPIO_PCR4_P6=(v).bPullup; \
                                          bFM_GPIO_DDR4_P6=0u; \
/*--- Notice: This function will configure both of P46/X0A and P47/X1A as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_GPIO->SPSR, 0u, 2u, 0x00u ); \
                                          bFM_GPIO_PFR4_P6=0u; }while(0)

#define FGPIO1PIN_NP46_INITOUT(v)     do{ bFM_GPIO_PDOR4_P6=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR4_P6=1u; \
/*--- Notice: This function will configure both of P46/X0A and P47/X1A as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_GPIO->SPSR, 0u, 2u, 0x00u ); \
                                          bFM_GPIO_PFR4_P6=0u; }while(0)


/*---- Fast GPIO bit P47 ----*/

#define FGPIO1PIN_P47_GET             ( FM_FASTIO->FPDIR4_f.P7 )

#define FGPIO1PIN_P47_PUT(v)          ( FM_FASTIO->FPDOR4_f.P7=(v) )

#define FGPIO1PIN_P47_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P47_INITIN(v) \
                                                         : FGPIO1PIN_P47_INITOUT(v) )

#define FGPIO1PIN_P47_INITIN(v)       do{ bFM_GPIO_PCR4_P7=(v).bPullup; \
                                          bFM_GPIO_DDR4_P7=0u; \
/*--- Notice: This function will configure both of P46/X0A and P47/X1A as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_GPIO->SPSR, 0u, 2u, 0x00u ); \
                                          bFM_GPIO_PFR4_P7=0u; }while(0)

#define FGPIO1PIN_P47_INITOUT(v)      do{ FM_FASTIO->FPDOR4_f.P7=(v).bInitVal; \
                                          bFM_GPIO_DDR4_P7=1u; \
/*--- Notice: This function will configure both of P46/X0A and P47/X1A as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_GPIO->SPSR, 0u, 2u, 0x00u ); \
                                          bFM_GPIO_PFR4_P7=0u; }while(0)


/*---- Fast GPIO bit NP47 ----*/
#define FGPIO1PIN_NP47_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR4_f.P7)) )

#define FGPIO1PIN_NP47_PUT(v)         ( FM_FASTIO->FPDOR4_f.P7=(uint8_t)(!(v)) )

#define FGPIO1PIN_NP47_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP47_INITIN(v) \
                                                         : FGPIO1PIN_NP47_INITOUT(v) )

#define FGPIO1PIN_NP47_INITIN(v)      do{ bFM_GPIO_PCR4_P7=(v).bPullup; \
                                          bFM_GPIO_DDR4_P7=0u; \
/*--- Notice: This function will configure both of P46/X0A and P47/X1A as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_GPIO->SPSR, 0u, 2u, 0x00u ); \
                                          bFM_GPIO_PFR4_P7=0u; }while(0)

#define FGPIO1PIN_NP47_INITOUT(v)     do{ bFM_GPIO_PDOR4_P7=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR4_P7=1u; \
/*--- Notice: This function will configure both of P46/X0A and P47/X1A as gpio pins. ---*/ \
                                         PINRELOC_SET_EPFR( FM_GPIO->SPSR, 0u, 2u, 0x00u ); \
                                          bFM_GPIO_PFR4_P7=0u; }while(0)


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


/*---- Fast GPIO bit PE0 ----*/

#define FGPIO1PIN_PE0_GET             ( FM_FASTIO->FPDIRE_f.P0 )

#define FGPIO1PIN_PE0_PUT(v)          ( FM_FASTIO->FPDORE_f.P0=(v) )

#define FGPIO1PIN_PE0_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_PE0_INITIN(v) \
                                                         : FGPIO1PIN_PE0_INITOUT(v) )

#define FGPIO1PIN_PE0_INITIN(v)       do{ bFM_GPIO_PCRE_P0=(v).bPullup; \
                                          bFM_GPIO_DDRE_P0=0u; \
                                          bFM_GPIO_PFRE_P0=0u; }while(0)

#define FGPIO1PIN_PE0_INITOUT(v)      do{ FM_FASTIO->FPDORE_f.P0=(v).bInitVal; \
                                          bFM_GPIO_DDRE_P0=1u; \
                                          bFM_GPIO_PFRE_P0=0u; }while(0)


/*---- Fast GPIO bit NPE0 ----*/
#define FGPIO1PIN_NPE0_GET            ( (uint8_t)(!(FM_FASTIO->FPDIRE_f.P0)) )

#define FGPIO1PIN_NPE0_PUT(v)         ( FM_FASTIO->FPDORE_f.P0=(uint8_t)(!(v)) )

#define FGPIO1PIN_NPE0_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NPE0_INITIN(v) \
                                                         : FGPIO1PIN_NPE0_INITOUT(v) )

#define FGPIO1PIN_NPE0_INITIN(v)      do{ bFM_GPIO_PCRE_P0=(v).bPullup; \
                                          bFM_GPIO_DDRE_P0=0u; \
                                          bFM_GPIO_PFRE_P0=0u; }while(0)

#define FGPIO1PIN_NPE0_INITOUT(v)     do{ bFM_GPIO_PDORE_P0=(uint8_t)(!((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDRE_P0=1u; \
                                          bFM_GPIO_PFRE_P0=0u; }while(0)


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

/*--- ADTG_1 for ADC#0 ---*/
#define SetPinFunc_ADTG_1_ADC0(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 12u, 4u, 0x02u ); \
                                                 bFM_GPIO_PFRE_P0 = 1u; }while(0)

/*--- ADTG_1 for ADC#1 ---*/
#define SetPinFunc_ADTG_1_ADC1(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 16u, 4u, 0x02u ); \
                                                 bFM_GPIO_PFRE_P0 = 1u; }while(0)

/*--- ADTG_1 for ADC#2 ---*/
#define SetPinFunc_ADTG_1_ADC2(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 20u, 4u, 0x02u ); \
                                                 bFM_GPIO_PFRE_P0 = 1u; }while(0)

/*--- AIN0_0 ---*/
#define SetPinFunc_AIN0_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 0u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_PD = 1u; }while(0)

/*--- AIN0_1 ---*/
#define SetPinFunc_AIN0_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 0u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P3 = 1u; \
                                                 bFM_GPIO_ADE_AN06 = 0u; }while(0)

/*--- AIN0_3 ---*/
#define SetPinFunc_AIN0_3(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 0u, 2u, 0x00u ); \
                                                 bFM_GPIO_EPFR21_QAIN0S = 1u; \
                                                 bFM_GPIO_PFR3_PA = 1u; }while(0)

/*--- BIN0_0 ---*/
#define SetPinFunc_BIN0_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 2u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_PE = 1u; }while(0)

/*--- BIN0_1 ---*/
#define SetPinFunc_BIN0_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 2u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P1 = 1u; }while(0)

/*--- BIN0_3 ---*/
#define SetPinFunc_BIN0_3(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 2u, 2u, 0x00u ); \
                                                 bFM_GPIO_EPFR21_QBIN0S = 1u; \
                                                 bFM_GPIO_PFR3_PB = 1u; }while(0)

/*--- CROUT_1 ---*/
#define SetPinFunc_CROUT_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR00, 1u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR0_PF = 1u; }while(0)

/*--- DTTI0X_1 ---*/
#define SetPinFunc_DTTI0X_1(dummy)           do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 16u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFRE_P0 = 1u; }while(0)

/*--- DTTI0X_2 ---*/
#define SetPinFunc_DTTI0X_2(dummy)           do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 16u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR6_P1 = 1u; }while(0)

/*--- FRCK0_0 ---*/
#define SetPinFunc_FRCK0_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 18u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR2_P1 = 1u; }while(0)

/*--- FRCK0_2 ---*/
#define SetPinFunc_FRCK0_2(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 18u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR1_P1 = 1u; \
                                                 bFM_GPIO_ADE_AN01 = 0u; }while(0)

/*--- IC00_0 ---*/
#define SetPinFunc_IC00_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 20u, 3u, 0x01u ); \
                                                 bFM_GPIO_PFR6_P0 = 1u; }while(0)

/*--- IC00_2 ---*/
#define SetPinFunc_IC00_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 20u, 3u, 0x03u ); \
                                                 bFM_GPIO_PFR1_P2 = 1u; \
                                                 bFM_GPIO_ADE_AN02 = 0u; }while(0)

/*--- IC00_LSYN0 ---*/
#define SetPinFunc_IC00_LSYN0(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 20u, 3u, 0x04u ); \
                                                 }while(0)

/*--- IC00_LSYN4 ---*/
#define SetPinFunc_IC00_LSYN4(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 20u, 3u, 0x05u ); \
                                                 }while(0)

/*--- IC01_1 ---*/
#define SetPinFunc_IC01_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 23u, 3u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P1 = 1u; }while(0)

/*--- IC01_2 ---*/
#define SetPinFunc_IC01_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 23u, 3u, 0x03u ); \
                                                 bFM_GPIO_PFR1_P3 = 1u; \
                                                 bFM_GPIO_ADE_AN03 = 0u; }while(0)

/*--- IC01_LSYN1 ---*/
#define SetPinFunc_IC01_LSYN1(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 23u, 3u, 0x04u ); \
                                                 }while(0)

/*--- IC01_LSYN5 ---*/
#define SetPinFunc_IC01_LSYN5(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 23u, 3u, 0x05u ); \
                                                 }while(0)

/*--- IC02_0 ---*/
#define SetPinFunc_IC02_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 26u, 3u, 0x01u ); \
                                                 bFM_GPIO_PFR1_P1 = 1u; \
                                                 bFM_GPIO_ADE_AN01 = 0u; }while(0)

/*--- IC02_1 ---*/
#define SetPinFunc_IC02_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 26u, 3u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P3 = 1u; \
                                                 bFM_GPIO_ADE_AN06 = 0u; }while(0)

/*--- IC02_LSYN2 ---*/
#define SetPinFunc_IC02_LSYN2(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 26u, 3u, 0x04u ); \
                                                 }while(0)

/*--- IC02_LSYN6 ---*/
#define SetPinFunc_IC02_LSYN6(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 26u, 3u, 0x05u ); \
                                                 }while(0)

/*--- IC03_1 ---*/
#define SetPinFunc_IC03_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 29u, 3u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P2 = 1u; \
                                                 bFM_GPIO_ADE_AN07 = 0u; }while(0)

/*--- IC03_LSYN3 ---*/
#define SetPinFunc_IC03_LSYN3(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 29u, 3u, 0x04u ); \
                                                 }while(0)

/*--- IC03_LSYN7 ---*/
#define SetPinFunc_IC03_LSYN7(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 29u, 3u, 0x05u ); \
                                                 }while(0)

/*--- IGTRG0_0 ---*/
#define SetPinFunc_IGTRG0_0(dummy)           do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 13u, 1u, 0x00u ); \
                                                 bFM_GPIO_PFR6_P0 = 1u; }while(0)

/*--- IGTRG0_1 ---*/
#define SetPinFunc_IGTRG0_1(dummy)           do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 13u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR0_P4 = 1u; }while(0)

/*--- INT00_1 ---*/
#define SetPinFunc_INT00_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 0u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P3 = 1u; \
                                                 bFM_GPIO_ADE_AN03 = 0u; }while(0)

/*--- INT01_1 ---*/
#define SetPinFunc_INT01_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 2u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P2 = 1u; \
                                                 bFM_GPIO_ADE_AN02 = 0u; }while(0)

/*--- INT02_1 ---*/
#define SetPinFunc_INT02_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 4u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P1 = 1u; \
                                                 bFM_GPIO_ADE_AN01 = 0u; }while(0)

/*--- INT02_2 ---*/
#define SetPinFunc_INT02_2(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 4u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFRE_P0 = 1u; }while(0)

/*--- INT03_0 ---*/
#define SetPinFunc_INT03_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 6u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_PA = 1u; }while(0)

/*--- INT03_2 ---*/
#define SetPinFunc_INT03_2(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 6u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR0_P4 = 1u; }while(0)

/*--- INT04_0 ---*/
#define SetPinFunc_INT04_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 8u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_PB = 1u; }while(0)

/*--- INT04_1 ---*/
#define SetPinFunc_INT04_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 8u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P3 = 1u; \
                                                 bFM_GPIO_ADE_AN06 = 0u; }while(0)

/*--- INT05_0 ---*/
#define SetPinFunc_INT05_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 10u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_PC = 1u; }while(0)

/*--- INT05_1 ---*/
#define SetPinFunc_INT05_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 10u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P2 = 1u; \
                                                 bFM_GPIO_ADE_AN07 = 0u; }while(0)

/*--- INT06_0 ---*/
#define SetPinFunc_INT06_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 12u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_PD = 1u; }while(0)

/*--- INT06_1 ---*/
#define SetPinFunc_INT06_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 12u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P1 = 1u; }while(0)

/*--- INT15_0 ---*/
#define SetPinFunc_INT15_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 30u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_PE = 1u; }while(0)

/*--- INT15_1 ---*/
#define SetPinFunc_INT15_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 30u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR6_P0 = 1u; }while(0)

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

/*--- RTO00_0 ---*/
#define SetPinFunc_RTO00_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 0u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_PA = 1u; }while(0)

/*--- RTO01_0 ---*/
#define SetPinFunc_RTO01_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 2u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_PB = 1u; }while(0)

/*--- RTO02_0 ---*/
#define SetPinFunc_RTO02_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 4u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_PC = 1u; }while(0)

/*--- RTO03_0 ---*/
#define SetPinFunc_RTO03_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 6u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_PD = 1u; }while(0)

/*--- RTO04_0 ---*/
#define SetPinFunc_RTO04_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 8u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_PE = 1u; }while(0)

/*--- RTO05_0 ---*/
#define SetPinFunc_RTO05_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 10u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_PF = 1u; }while(0)

/*--- SCK0_0 ---*/
#define SetPinFunc_SCK0_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 8u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR2_P3 = 1u; \
                                                 bFM_GPIO_ADE_AN06 = 0u; }while(0)

/*--- SCK0_2 ---*/
#define SetPinFunc_SCK0_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 8u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR3_PA = 1u; }while(0)

/*--- SCK1_1 ---*/
#define SetPinFunc_SCK1_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 14u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P3 = 1u; \
                                                 bFM_GPIO_ADE_AN03 = 0u; }while(0)

/*--- SCK3_0 ---*/
#define SetPinFunc_SCK3_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 26u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR0_P4 = 1u; }while(0)

/*--- SCK3_2 ---*/
#define SetPinFunc_SCK3_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 26u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR3_PD = 1u; }while(0)

/*--- SCS10_2 ---*/
#define SetPinFunc_SCS10_2(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR22, 4u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR6_P0 = 1u; }while(0)

/*--- SCS11_2 ---*/
#define SetPinFunc_SCS11_2(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR22, 6u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR6_P1 = 1u; }while(0)

/*--- SCS30_2 ---*/
#define SetPinFunc_SCS30_2(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR22, 12u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR3_PC = 1u; }while(0)

/*--- SCS31_2 ---*/
#define SetPinFunc_SCS31_2(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR22, 14u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR3_PB = 1u; }while(0)

/*--- SIN0_0 ---*/
#define SetPinFunc_SIN0_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 4u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR2_P1 = 1u; }while(0)

/*--- SIN0_2 ---*/
#define SetPinFunc_SIN0_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 4u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR3_PC = 1u; }while(0)

/*--- SIN1_1 ---*/
#define SetPinFunc_SIN1_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 10u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P1 = 1u; \
                                                 bFM_GPIO_ADE_AN01 = 0u; }while(0)

/*--- SIN3_0 ---*/
#define SetPinFunc_SIN3_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 22u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR6_P0 = 1u; }while(0)

/*--- SIN3_2 ---*/
#define SetPinFunc_SIN3_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 22u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR3_PF = 1u; }while(0)

/*--- SOT0_0 ---*/
#define SetPinFunc_SOT0_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 6u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR2_P2 = 1u; \
                                                 bFM_GPIO_ADE_AN07 = 0u; }while(0)

/*--- SOT0_2 ---*/
#define SetPinFunc_SOT0_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 6u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR3_PB = 1u; }while(0)

/*--- SOT1_1 ---*/
#define SetPinFunc_SOT1_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 12u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P2 = 1u; \
                                                 bFM_GPIO_ADE_AN02 = 0u; }while(0)

/*--- SOT3_0 ---*/
#define SetPinFunc_SOT3_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 24u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR6_P1 = 1u; }while(0)

/*--- SOT3_2 ---*/
#define SetPinFunc_SOT3_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 24u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR3_PE = 1u; }while(0)

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
                                                 bFM_GPIO_PFR3_PE = 1u; }while(0)

/*--- TIOA0_1_OUT ---*/
#define SetPinFunc_TIOA0_1_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 2u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_PA = 1u; }while(0)

/*--- TIOA1_0_IN ---*/
#define SetPinFunc_TIOA1_0_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 8u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_PF = 1u; }while(0)

/*--- TIOA1_0_OUT ---*/
#define SetPinFunc_TIOA1_0_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 10u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_PF = 1u; }while(0)

/*--- TIOA1_1_IN ---*/
#define SetPinFunc_TIOA1_1_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 8u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_PB = 1u; }while(0)

/*--- TIOA1_1_OUT ---*/
#define SetPinFunc_TIOA1_1_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 10u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_PB = 1u; }while(0)

/*--- TIOA2_0_OUT ---*/
#define SetPinFunc_TIOA2_0_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 18u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR2_P3 = 1u; \
                                                 bFM_GPIO_ADE_AN06 = 0u; }while(0)

/*--- TIOA2_1_OUT ---*/
#define SetPinFunc_TIOA2_1_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 18u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_PC = 1u; }while(0)

/*--- TIOA2_2_OUT ---*/
#define SetPinFunc_TIOA2_2_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 18u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR6_P0 = 1u; }while(0)

/*--- TIOA3_1_IN ---*/
#define SetPinFunc_TIOA3_1_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 24u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_PD = 1u; }while(0)

/*--- TIOA3_1_OUT ---*/
#define SetPinFunc_TIOA3_1_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 26u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_PD = 1u; }while(0)

/*--- TIOB0_1_IN ---*/
#define SetPinFunc_TIOB0_1_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 4u, 3u, 0x02u ); \
                                                 bFM_GPIO_PFR0_P4 = 1u; }while(0)

/*--- TIOB1_1_IN ---*/
#define SetPinFunc_TIOB1_1_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 12u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P1 = 1u; }while(0)

/*--- TIOB2_0_IN ---*/
#define SetPinFunc_TIOB2_0_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 20u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR2_P2 = 1u; \
                                                 bFM_GPIO_ADE_AN07 = 0u; }while(0)

/*--- TIOB2_2_IN ---*/
#define SetPinFunc_TIOB2_2_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 20u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR6_P1 = 1u; }while(0)

/*--- X0 ---*/
/*--- Notice: This function will configure X0 and X1 as clock pins. ---*/
#define SetPinFunc_X0(dummy)                 do{ PINRELOC_SET_EPFR( FM_GPIO->SPSR, 2u, 2u, 0x01u ); }while(0u)

/*--- X0A ---*/
/*--- Notice: This function will configure X0A and X1A as clock pins. ---*/
#define SetPinFunc_X0A(dummy)                do{ PINRELOC_SET_EPFR( FM_GPIO->SPSR, 0u, 2u, 0x01u ); }while(0u)

/*--- X1 ---*/
/*--- Notice: This function will configure X0 and X1 as clock pins. ---*/
#define SetPinFunc_X1(dummy)                 do{ PINRELOC_SET_EPFR( FM_GPIO->SPSR, 2u, 2u, 0x01u ); }while(0u)

/*--- X1A ---*/
/*--- Notice: This function will configure X0A and X1A as clock pins. ---*/
#define SetPinFunc_X1A(dummy)                do{ PINRELOC_SET_EPFR( FM_GPIO->SPSR, 0u, 2u, 0x01u ); }while(0u)

/*--- ZIN0_0 ---*/
#define SetPinFunc_ZIN0_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 4u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_PF = 1u; }while(0)

/*--- ZIN0_1 ---*/
#define SetPinFunc_ZIN0_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 4u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P2 = 1u; \
                                                 bFM_GPIO_ADE_AN07 = 0u; }while(0)

/*--- ZIN0_3 ---*/
#define SetPinFunc_ZIN0_3(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 4u, 2u, 0x00u ); \
                                                 bFM_GPIO_EPFR21_QZIN0S = 1u; \
                                                 bFM_GPIO_PFR3_PC = 1u; }while(0)


/******************************************************************************
   ANALOG PINS
*******************************************************************************/

/*--- AN01 ---*/
#define SetPinFunc_AN01(dummy)               ( bFM_GPIO_ADE_AN01 = 1 )

/*--- AN02 ---*/
#define SetPinFunc_AN02(dummy)               ( bFM_GPIO_ADE_AN02 = 1 )

/*--- AN03 ---*/
#define SetPinFunc_AN03(dummy)               ( bFM_GPIO_ADE_AN03 = 1 )

/*--- AN06 ---*/
#define SetPinFunc_AN06(dummy)               ( bFM_GPIO_ADE_AN06 = 1 )

/*--- AN07 ---*/
#define SetPinFunc_AN07(dummy)               ( bFM_GPIO_ADE_AN07 = 1 )


#endif // #ifndef __GPIO_S6E1A1XB_H__


/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/

