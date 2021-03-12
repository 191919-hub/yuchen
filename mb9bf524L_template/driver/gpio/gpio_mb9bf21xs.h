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
/** \file gpio_mb9bf21xs.h 
 **
 ** Headerfile for MB9BF21XS GPIO functions, included in gpio.h
 **
 ** History:
 **   - 2014-05-04  1.0  Edison Zhang    First version
 **
 ******************************************************************************/

#ifndef __GPIO_MB9BF21XS_H__
#define __GPIO_MB9BF21XS_H__

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
                                         bFM_GPIO_PFR0_P0=0u; }while(0)

#define GPIO1PIN_P00_INITOUT(v)      do{ bFM_GPIO_PDOR0_P0=(v).bInitVal; \
                                         bFM_GPIO_DDR0_P0=1u; \
                                         bFM_GPIO_PFR0_P0=0u; }while(0)


/*---- GPIO bit NP00 ----*/
#define GPIO1PIN_NP00_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR0_P0)) )

#define GPIO1PIN_NP00_PUT(v)         ( bFM_GPIO_PDOR0_P0=(uint32_t)(!(v)) )

#define GPIO1PIN_NP00_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP00_INITIN(v) \
                                                        : GPIO1PIN_NP00_INITOUT(v) )

#define GPIO1PIN_NP00_INITIN(v)      do{ bFM_GPIO_PCR0_P0=(v).bPullup; \
                                         bFM_GPIO_DDR0_P0=0u; \
                                         bFM_GPIO_PFR0_P0=0u; }while(0)

#define GPIO1PIN_NP00_INITOUT(v)     do{ bFM_GPIO_PDOR0_P0=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR0_P0=1u; \
                                         bFM_GPIO_PFR0_P0=0u; }while(0)


/*---- GPIO bit P01 ----*/

#define GPIO1PIN_P01_GET             ( bFM_GPIO_PDIR0_P1 )

#define GPIO1PIN_P01_PUT(v)          ( bFM_GPIO_PDOR0_P1=(v) )

#define GPIO1PIN_P01_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P01_INITIN(v) \
                                                        : GPIO1PIN_P01_INITOUT(v) )

#define GPIO1PIN_P01_INITIN(v)       do{ bFM_GPIO_PCR0_P1=(v).bPullup; \
                                         bFM_GPIO_DDR0_P1=0u; \
                                         bFM_GPIO_PFR0_P1=0u; }while(0)

#define GPIO1PIN_P01_INITOUT(v)      do{ bFM_GPIO_PDOR0_P1=(v).bInitVal; \
                                         bFM_GPIO_DDR0_P1=1u; \
                                         bFM_GPIO_PFR0_P1=0u; }while(0)


/*---- GPIO bit NP01 ----*/
#define GPIO1PIN_NP01_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR0_P1)) )

#define GPIO1PIN_NP01_PUT(v)         ( bFM_GPIO_PDOR0_P1=(uint32_t)(!(v)) )

#define GPIO1PIN_NP01_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP01_INITIN(v) \
                                                        : GPIO1PIN_NP01_INITOUT(v) )

#define GPIO1PIN_NP01_INITIN(v)      do{ bFM_GPIO_PCR0_P1=(v).bPullup; \
                                         bFM_GPIO_DDR0_P1=0u; \
                                         bFM_GPIO_PFR0_P1=0u; }while(0)

#define GPIO1PIN_NP01_INITOUT(v)     do{ bFM_GPIO_PDOR0_P1=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR0_P1=1u; \
                                         bFM_GPIO_PFR0_P1=0u; }while(0)


/*---- GPIO bit P02 ----*/

#define GPIO1PIN_P02_GET             ( bFM_GPIO_PDIR0_P2 )

#define GPIO1PIN_P02_PUT(v)          ( bFM_GPIO_PDOR0_P2=(v) )

#define GPIO1PIN_P02_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P02_INITIN(v) \
                                                        : GPIO1PIN_P02_INITOUT(v) )

#define GPIO1PIN_P02_INITIN(v)       do{ bFM_GPIO_PCR0_P2=(v).bPullup; \
                                         bFM_GPIO_DDR0_P2=0u; \
                                         bFM_GPIO_PFR0_P2=0u; }while(0)

#define GPIO1PIN_P02_INITOUT(v)      do{ bFM_GPIO_PDOR0_P2=(v).bInitVal; \
                                         bFM_GPIO_DDR0_P2=1u; \
                                         bFM_GPIO_PFR0_P2=0u; }while(0)


/*---- GPIO bit NP02 ----*/
#define GPIO1PIN_NP02_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR0_P2)) )

#define GPIO1PIN_NP02_PUT(v)         ( bFM_GPIO_PDOR0_P2=(uint32_t)(!(v)) )

#define GPIO1PIN_NP02_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP02_INITIN(v) \
                                                        : GPIO1PIN_NP02_INITOUT(v) )

#define GPIO1PIN_NP02_INITIN(v)      do{ bFM_GPIO_PCR0_P2=(v).bPullup; \
                                         bFM_GPIO_DDR0_P2=0u; \
                                         bFM_GPIO_PFR0_P2=0u; }while(0)

#define GPIO1PIN_NP02_INITOUT(v)     do{ bFM_GPIO_PDOR0_P2=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR0_P2=1u; \
                                         bFM_GPIO_PFR0_P2=0u; }while(0)


/*---- GPIO bit P03 ----*/

#define GPIO1PIN_P03_GET             ( bFM_GPIO_PDIR0_P3 )

#define GPIO1PIN_P03_PUT(v)          ( bFM_GPIO_PDOR0_P3=(v) )

#define GPIO1PIN_P03_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P03_INITIN(v) \
                                                        : GPIO1PIN_P03_INITOUT(v) )

#define GPIO1PIN_P03_INITIN(v)       do{ bFM_GPIO_PCR0_P3=(v).bPullup; \
                                         bFM_GPIO_DDR0_P3=0u; \
                                         bFM_GPIO_PFR0_P3=0u; }while(0)

#define GPIO1PIN_P03_INITOUT(v)      do{ bFM_GPIO_PDOR0_P3=(v).bInitVal; \
                                         bFM_GPIO_DDR0_P3=1u; \
                                         bFM_GPIO_PFR0_P3=0u; }while(0)


/*---- GPIO bit NP03 ----*/
#define GPIO1PIN_NP03_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR0_P3)) )

#define GPIO1PIN_NP03_PUT(v)         ( bFM_GPIO_PDOR0_P3=(uint32_t)(!(v)) )

#define GPIO1PIN_NP03_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP03_INITIN(v) \
                                                        : GPIO1PIN_NP03_INITOUT(v) )

#define GPIO1PIN_NP03_INITIN(v)      do{ bFM_GPIO_PCR0_P3=(v).bPullup; \
                                         bFM_GPIO_DDR0_P3=0u; \
                                         bFM_GPIO_PFR0_P3=0u; }while(0)

#define GPIO1PIN_NP03_INITOUT(v)     do{ bFM_GPIO_PDOR0_P3=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR0_P3=1u; \
                                         bFM_GPIO_PFR0_P3=0u; }while(0)


/*---- GPIO bit P04 ----*/

#define GPIO1PIN_P04_GET             ( bFM_GPIO_PDIR0_P4 )

#define GPIO1PIN_P04_PUT(v)          ( bFM_GPIO_PDOR0_P4=(v) )

#define GPIO1PIN_P04_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P04_INITIN(v) \
                                                        : GPIO1PIN_P04_INITOUT(v) )

#define GPIO1PIN_P04_INITIN(v)       do{ bFM_GPIO_PCR0_P4=(v).bPullup; \
                                         bFM_GPIO_DDR0_P4=0u; \
                                         bFM_GPIO_PFR0_P4=0u; }while(0)

#define GPIO1PIN_P04_INITOUT(v)      do{ bFM_GPIO_PDOR0_P4=(v).bInitVal; \
                                         bFM_GPIO_DDR0_P4=1u; \
                                         bFM_GPIO_PFR0_P4=0u; }while(0)


/*---- GPIO bit NP04 ----*/
#define GPIO1PIN_NP04_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR0_P4)) )

#define GPIO1PIN_NP04_PUT(v)         ( bFM_GPIO_PDOR0_P4=(uint32_t)(!(v)) )

#define GPIO1PIN_NP04_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP04_INITIN(v) \
                                                        : GPIO1PIN_NP04_INITOUT(v) )

#define GPIO1PIN_NP04_INITIN(v)      do{ bFM_GPIO_PCR0_P4=(v).bPullup; \
                                         bFM_GPIO_DDR0_P4=0u; \
                                         bFM_GPIO_PFR0_P4=0u; }while(0)

#define GPIO1PIN_NP04_INITOUT(v)     do{ bFM_GPIO_PDOR0_P4=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR0_P4=1u; \
                                         bFM_GPIO_PFR0_P4=0u; }while(0)


/*---- GPIO bit P05 ----*/

#define GPIO1PIN_P05_GET             ( bFM_GPIO_PDIR0_P5 )

#define GPIO1PIN_P05_PUT(v)          ( bFM_GPIO_PDOR0_P5=(v) )

#define GPIO1PIN_P05_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P05_INITIN(v) \
                                                        : GPIO1PIN_P05_INITOUT(v) )

#define GPIO1PIN_P05_INITIN(v)       do{ bFM_GPIO_PCR0_P5=(v).bPullup; \
                                         bFM_GPIO_DDR0_P5=0u; \
                                         bFM_GPIO_PFR0_P5=0u; }while(0)

#define GPIO1PIN_P05_INITOUT(v)      do{ bFM_GPIO_PDOR0_P5=(v).bInitVal; \
                                         bFM_GPIO_DDR0_P5=1u; \
                                         bFM_GPIO_PFR0_P5=0u; }while(0)


/*---- GPIO bit NP05 ----*/
#define GPIO1PIN_NP05_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR0_P5)) )

#define GPIO1PIN_NP05_PUT(v)         ( bFM_GPIO_PDOR0_P5=(uint32_t)(!(v)) )

#define GPIO1PIN_NP05_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP05_INITIN(v) \
                                                        : GPIO1PIN_NP05_INITOUT(v) )

#define GPIO1PIN_NP05_INITIN(v)      do{ bFM_GPIO_PCR0_P5=(v).bPullup; \
                                         bFM_GPIO_DDR0_P5=0u; \
                                         bFM_GPIO_PFR0_P5=0u; }while(0)

#define GPIO1PIN_NP05_INITOUT(v)     do{ bFM_GPIO_PDOR0_P5=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR0_P5=1u; \
                                         bFM_GPIO_PFR0_P5=0u; }while(0)


/*---- GPIO bit P06 ----*/

#define GPIO1PIN_P06_GET             ( bFM_GPIO_PDIR0_P6 )

#define GPIO1PIN_P06_PUT(v)          ( bFM_GPIO_PDOR0_P6=(v) )

#define GPIO1PIN_P06_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P06_INITIN(v) \
                                                        : GPIO1PIN_P06_INITOUT(v) )

#define GPIO1PIN_P06_INITIN(v)       do{ bFM_GPIO_PCR0_P6=(v).bPullup; \
                                         bFM_GPIO_DDR0_P6=0u; \
                                         bFM_GPIO_PFR0_P6=0u; }while(0)

#define GPIO1PIN_P06_INITOUT(v)      do{ bFM_GPIO_PDOR0_P6=(v).bInitVal; \
                                         bFM_GPIO_DDR0_P6=1u; \
                                         bFM_GPIO_PFR0_P6=0u; }while(0)


/*---- GPIO bit NP06 ----*/
#define GPIO1PIN_NP06_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR0_P6)) )

#define GPIO1PIN_NP06_PUT(v)         ( bFM_GPIO_PDOR0_P6=(uint32_t)(!(v)) )

#define GPIO1PIN_NP06_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP06_INITIN(v) \
                                                        : GPIO1PIN_NP06_INITOUT(v) )

#define GPIO1PIN_NP06_INITIN(v)      do{ bFM_GPIO_PCR0_P6=(v).bPullup; \
                                         bFM_GPIO_DDR0_P6=0u; \
                                         bFM_GPIO_PFR0_P6=0u; }while(0)

#define GPIO1PIN_NP06_INITOUT(v)     do{ bFM_GPIO_PDOR0_P6=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR0_P6=1u; \
                                         bFM_GPIO_PFR0_P6=0u; }while(0)


/*---- GPIO bit P07 ----*/

#define GPIO1PIN_P07_GET             ( bFM_GPIO_PDIR0_P7 )

#define GPIO1PIN_P07_PUT(v)          ( bFM_GPIO_PDOR0_P7=(v) )

#define GPIO1PIN_P07_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P07_INITIN(v) \
                                                        : GPIO1PIN_P07_INITOUT(v) )

#define GPIO1PIN_P07_INITIN(v)       do{ bFM_GPIO_PCR0_P7=(v).bPullup; \
                                         bFM_GPIO_DDR0_P7=0u; \
                                         bFM_GPIO_PFR0_P7=0u; }while(0)

#define GPIO1PIN_P07_INITOUT(v)      do{ bFM_GPIO_PDOR0_P7=(v).bInitVal; \
                                         bFM_GPIO_DDR0_P7=1u; \
                                         bFM_GPIO_PFR0_P7=0u; }while(0)


/*---- GPIO bit NP07 ----*/
#define GPIO1PIN_NP07_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR0_P7)) )

#define GPIO1PIN_NP07_PUT(v)         ( bFM_GPIO_PDOR0_P7=(uint32_t)(!(v)) )

#define GPIO1PIN_NP07_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP07_INITIN(v) \
                                                        : GPIO1PIN_NP07_INITOUT(v) )

#define GPIO1PIN_NP07_INITIN(v)      do{ bFM_GPIO_PCR0_P7=(v).bPullup; \
                                         bFM_GPIO_DDR0_P7=0u; \
                                         bFM_GPIO_PFR0_P7=0u; }while(0)

#define GPIO1PIN_NP07_INITOUT(v)     do{ bFM_GPIO_PDOR0_P7=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR0_P7=1u; \
                                         bFM_GPIO_PFR0_P7=0u; }while(0)


/*---- GPIO bit P08 ----*/

#define GPIO1PIN_P08_GET             ( bFM_GPIO_PDIR0_P8 )

#define GPIO1PIN_P08_PUT(v)          ( bFM_GPIO_PDOR0_P8=(v) )

#define GPIO1PIN_P08_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P08_INITIN(v) \
                                                        : GPIO1PIN_P08_INITOUT(v) )

#define GPIO1PIN_P08_INITIN(v)       do{ bFM_GPIO_PCR0_P8=(v).bPullup; \
                                         bFM_GPIO_DDR0_P8=0u; \
                                         bFM_GPIO_PFR0_P8=0u; }while(0)

#define GPIO1PIN_P08_INITOUT(v)      do{ bFM_GPIO_PDOR0_P8=(v).bInitVal; \
                                         bFM_GPIO_DDR0_P8=1u; \
                                         bFM_GPIO_PFR0_P8=0u; }while(0)


/*---- GPIO bit NP08 ----*/
#define GPIO1PIN_NP08_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR0_P8)) )

#define GPIO1PIN_NP08_PUT(v)         ( bFM_GPIO_PDOR0_P8=(uint32_t)(!(v)) )

#define GPIO1PIN_NP08_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP08_INITIN(v) \
                                                        : GPIO1PIN_NP08_INITOUT(v) )

#define GPIO1PIN_NP08_INITIN(v)      do{ bFM_GPIO_PCR0_P8=(v).bPullup; \
                                         bFM_GPIO_DDR0_P8=0u; \
                                         bFM_GPIO_PFR0_P8=0u; }while(0)

#define GPIO1PIN_NP08_INITOUT(v)     do{ bFM_GPIO_PDOR0_P8=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR0_P8=1u; \
                                         bFM_GPIO_PFR0_P8=0u; }while(0)


/*---- GPIO bit P09 ----*/

#define GPIO1PIN_P09_GET             ( bFM_GPIO_PDIR0_P9 )

#define GPIO1PIN_P09_PUT(v)          ( bFM_GPIO_PDOR0_P9=(v) )

#define GPIO1PIN_P09_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P09_INITIN(v) \
                                                        : GPIO1PIN_P09_INITOUT(v) )

#define GPIO1PIN_P09_INITIN(v)       do{ bFM_GPIO_PCR0_P9=(v).bPullup; \
                                         bFM_GPIO_DDR0_P9=0u; \
                                         bFM_GPIO_PFR0_P9=0u; }while(0)

#define GPIO1PIN_P09_INITOUT(v)      do{ bFM_GPIO_PDOR0_P9=(v).bInitVal; \
                                         bFM_GPIO_DDR0_P9=1u; \
                                         bFM_GPIO_PFR0_P9=0u; }while(0)


/*---- GPIO bit NP09 ----*/
#define GPIO1PIN_NP09_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR0_P9)) )

#define GPIO1PIN_NP09_PUT(v)         ( bFM_GPIO_PDOR0_P9=(uint32_t)(!(v)) )

#define GPIO1PIN_NP09_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP09_INITIN(v) \
                                                        : GPIO1PIN_NP09_INITOUT(v) )

#define GPIO1PIN_NP09_INITIN(v)      do{ bFM_GPIO_PCR0_P9=(v).bPullup; \
                                         bFM_GPIO_DDR0_P9=0u; \
                                         bFM_GPIO_PFR0_P9=0u; }while(0)

#define GPIO1PIN_NP09_INITOUT(v)     do{ bFM_GPIO_PDOR0_P9=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR0_P9=1u; \
                                         bFM_GPIO_PFR0_P9=0u; }while(0)


/*---- GPIO bit P10 ----*/

#define GPIO1PIN_P10_GET             ( bFM_GPIO_PDIR1_P0 )

#define GPIO1PIN_P10_PUT(v)          ( bFM_GPIO_PDOR1_P0=(v) )

#define GPIO1PIN_P10_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P10_INITIN(v) \
                                                        : GPIO1PIN_P10_INITOUT(v) )

#define GPIO1PIN_P10_INITIN(v)       do{ bFM_GPIO_PCR1_P0=(v).bPullup; \
                                         bFM_GPIO_DDR1_P0=0u; \
                                         bFM_GPIO_PFR1_P0=0u; \
                                         bFM_GPIO_ADE_AN00=0u; }while(0)


#define GPIO1PIN_P10_INITOUT(v)      do{ bFM_GPIO_PDOR1_P0=(v).bInitVal; \
                                         bFM_GPIO_DDR1_P0=1u; \
                                         bFM_GPIO_PFR1_P0=0u; \
                                         bFM_GPIO_ADE_AN00=0u; }while(0)


/*---- GPIO bit NP10 ----*/
#define GPIO1PIN_NP10_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR1_P0)) )

#define GPIO1PIN_NP10_PUT(v)         ( bFM_GPIO_PDOR1_P0=(uint32_t)(!(v)) )

#define GPIO1PIN_NP10_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP10_INITIN(v) \
                                                        : GPIO1PIN_NP10_INITOUT(v) )

#define GPIO1PIN_NP10_INITIN(v)      do{ bFM_GPIO_PCR1_P0=(v).bPullup; \
                                         bFM_GPIO_DDR1_P0=0u; \
                                         bFM_GPIO_PFR1_P0=0u; \
                                         bFM_GPIO_ADE_AN00=0u; }while(0)


#define GPIO1PIN_NP10_INITOUT(v)     do{ bFM_GPIO_PDOR1_P0=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR1_P0=1u; \
                                         bFM_GPIO_PFR1_P0=0u; \
                                         bFM_GPIO_ADE_AN00=0u; }while(0)


/*---- GPIO bit P11 ----*/

#define GPIO1PIN_P11_GET             ( bFM_GPIO_PDIR1_P1 )

#define GPIO1PIN_P11_PUT(v)          ( bFM_GPIO_PDOR1_P1=(v) )

#define GPIO1PIN_P11_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P11_INITIN(v) \
                                                        : GPIO1PIN_P11_INITOUT(v) )

#define GPIO1PIN_P11_INITIN(v)       do{ bFM_GPIO_PCR1_P1=(v).bPullup; \
                                         bFM_GPIO_DDR1_P1=0u; \
                                         bFM_GPIO_PFR1_P1=0u; \
                                         bFM_GPIO_ADE_AN01=0u; }while(0)


#define GPIO1PIN_P11_INITOUT(v)      do{ bFM_GPIO_PDOR1_P1=(v).bInitVal; \
                                         bFM_GPIO_DDR1_P1=1u; \
                                         bFM_GPIO_PFR1_P1=0u; \
                                         bFM_GPIO_ADE_AN01=0u; }while(0)


/*---- GPIO bit NP11 ----*/
#define GPIO1PIN_NP11_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR1_P1)) )

#define GPIO1PIN_NP11_PUT(v)         ( bFM_GPIO_PDOR1_P1=(uint32_t)(!(v)) )

#define GPIO1PIN_NP11_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP11_INITIN(v) \
                                                        : GPIO1PIN_NP11_INITOUT(v) )

#define GPIO1PIN_NP11_INITIN(v)      do{ bFM_GPIO_PCR1_P1=(v).bPullup; \
                                         bFM_GPIO_DDR1_P1=0u; \
                                         bFM_GPIO_PFR1_P1=0u; \
                                         bFM_GPIO_ADE_AN01=0u; }while(0)


#define GPIO1PIN_NP11_INITOUT(v)     do{ bFM_GPIO_PDOR1_P1=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR1_P1=1u; \
                                         bFM_GPIO_PFR1_P1=0u; \
                                         bFM_GPIO_ADE_AN01=0u; }while(0)


/*---- GPIO bit P12 ----*/

#define GPIO1PIN_P12_GET             ( bFM_GPIO_PDIR1_P2 )

#define GPIO1PIN_P12_PUT(v)          ( bFM_GPIO_PDOR1_P2=(v) )

#define GPIO1PIN_P12_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P12_INITIN(v) \
                                                        : GPIO1PIN_P12_INITOUT(v) )

#define GPIO1PIN_P12_INITIN(v)       do{ bFM_GPIO_PCR1_P2=(v).bPullup; \
                                         bFM_GPIO_DDR1_P2=0u; \
                                         bFM_GPIO_PFR1_P2=0u; \
                                         bFM_GPIO_ADE_AN02=0u; }while(0)


#define GPIO1PIN_P12_INITOUT(v)      do{ bFM_GPIO_PDOR1_P2=(v).bInitVal; \
                                         bFM_GPIO_DDR1_P2=1u; \
                                         bFM_GPIO_PFR1_P2=0u; \
                                         bFM_GPIO_ADE_AN02=0u; }while(0)


/*---- GPIO bit NP12 ----*/
#define GPIO1PIN_NP12_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR1_P2)) )

#define GPIO1PIN_NP12_PUT(v)         ( bFM_GPIO_PDOR1_P2=(uint32_t)(!(v)) )

#define GPIO1PIN_NP12_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP12_INITIN(v) \
                                                        : GPIO1PIN_NP12_INITOUT(v) )

#define GPIO1PIN_NP12_INITIN(v)      do{ bFM_GPIO_PCR1_P2=(v).bPullup; \
                                         bFM_GPIO_DDR1_P2=0u; \
                                         bFM_GPIO_PFR1_P2=0u; \
                                         bFM_GPIO_ADE_AN02=0u; }while(0)


#define GPIO1PIN_NP12_INITOUT(v)     do{ bFM_GPIO_PDOR1_P2=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR1_P2=1u; \
                                         bFM_GPIO_PFR1_P2=0u; \
                                         bFM_GPIO_ADE_AN02=0u; }while(0)


/*---- GPIO bit P13 ----*/

#define GPIO1PIN_P13_GET             ( bFM_GPIO_PDIR1_P3 )

#define GPIO1PIN_P13_PUT(v)          ( bFM_GPIO_PDOR1_P3=(v) )

#define GPIO1PIN_P13_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P13_INITIN(v) \
                                                        : GPIO1PIN_P13_INITOUT(v) )

#define GPIO1PIN_P13_INITIN(v)       do{ bFM_GPIO_PCR1_P3=(v).bPullup; \
                                         bFM_GPIO_DDR1_P3=0u; \
                                         bFM_GPIO_PFR1_P3=0u; \
                                         bFM_GPIO_ADE_AN03=0u; }while(0)


#define GPIO1PIN_P13_INITOUT(v)      do{ bFM_GPIO_PDOR1_P3=(v).bInitVal; \
                                         bFM_GPIO_DDR1_P3=1u; \
                                         bFM_GPIO_PFR1_P3=0u; \
                                         bFM_GPIO_ADE_AN03=0u; }while(0)


/*---- GPIO bit NP13 ----*/
#define GPIO1PIN_NP13_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR1_P3)) )

#define GPIO1PIN_NP13_PUT(v)         ( bFM_GPIO_PDOR1_P3=(uint32_t)(!(v)) )

#define GPIO1PIN_NP13_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP13_INITIN(v) \
                                                        : GPIO1PIN_NP13_INITOUT(v) )

#define GPIO1PIN_NP13_INITIN(v)      do{ bFM_GPIO_PCR1_P3=(v).bPullup; \
                                         bFM_GPIO_DDR1_P3=0u; \
                                         bFM_GPIO_PFR1_P3=0u; \
                                         bFM_GPIO_ADE_AN03=0u; }while(0)


#define GPIO1PIN_NP13_INITOUT(v)     do{ bFM_GPIO_PDOR1_P3=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR1_P3=1u; \
                                         bFM_GPIO_PFR1_P3=0u; \
                                         bFM_GPIO_ADE_AN03=0u; }while(0)


/*---- GPIO bit P14 ----*/

#define GPIO1PIN_P14_GET             ( bFM_GPIO_PDIR1_P4 )

#define GPIO1PIN_P14_PUT(v)          ( bFM_GPIO_PDOR1_P4=(v) )

#define GPIO1PIN_P14_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P14_INITIN(v) \
                                                        : GPIO1PIN_P14_INITOUT(v) )

#define GPIO1PIN_P14_INITIN(v)       do{ bFM_GPIO_PCR1_P4=(v).bPullup; \
                                         bFM_GPIO_DDR1_P4=0u; \
                                         bFM_GPIO_PFR1_P4=0u; \
                                         bFM_GPIO_ADE_AN04=0u; }while(0)


#define GPIO1PIN_P14_INITOUT(v)      do{ bFM_GPIO_PDOR1_P4=(v).bInitVal; \
                                         bFM_GPIO_DDR1_P4=1u; \
                                         bFM_GPIO_PFR1_P4=0u; \
                                         bFM_GPIO_ADE_AN04=0u; }while(0)


/*---- GPIO bit NP14 ----*/
#define GPIO1PIN_NP14_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR1_P4)) )

#define GPIO1PIN_NP14_PUT(v)         ( bFM_GPIO_PDOR1_P4=(uint32_t)(!(v)) )

#define GPIO1PIN_NP14_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP14_INITIN(v) \
                                                        : GPIO1PIN_NP14_INITOUT(v) )

#define GPIO1PIN_NP14_INITIN(v)      do{ bFM_GPIO_PCR1_P4=(v).bPullup; \
                                         bFM_GPIO_DDR1_P4=0u; \
                                         bFM_GPIO_PFR1_P4=0u; \
                                         bFM_GPIO_ADE_AN04=0u; }while(0)


#define GPIO1PIN_NP14_INITOUT(v)     do{ bFM_GPIO_PDOR1_P4=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR1_P4=1u; \
                                         bFM_GPIO_PFR1_P4=0u; \
                                         bFM_GPIO_ADE_AN04=0u; }while(0)


/*---- GPIO bit P15 ----*/

#define GPIO1PIN_P15_GET             ( bFM_GPIO_PDIR1_P5 )

#define GPIO1PIN_P15_PUT(v)          ( bFM_GPIO_PDOR1_P5=(v) )

#define GPIO1PIN_P15_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P15_INITIN(v) \
                                                        : GPIO1PIN_P15_INITOUT(v) )

#define GPIO1PIN_P15_INITIN(v)       do{ bFM_GPIO_PCR1_P5=(v).bPullup; \
                                         bFM_GPIO_DDR1_P5=0u; \
                                         bFM_GPIO_PFR1_P5=0u; \
                                         bFM_GPIO_ADE_AN05=0u; }while(0)


#define GPIO1PIN_P15_INITOUT(v)      do{ bFM_GPIO_PDOR1_P5=(v).bInitVal; \
                                         bFM_GPIO_DDR1_P5=1u; \
                                         bFM_GPIO_PFR1_P5=0u; \
                                         bFM_GPIO_ADE_AN05=0u; }while(0)


/*---- GPIO bit NP15 ----*/
#define GPIO1PIN_NP15_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR1_P5)) )

#define GPIO1PIN_NP15_PUT(v)         ( bFM_GPIO_PDOR1_P5=(uint32_t)(!(v)) )

#define GPIO1PIN_NP15_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP15_INITIN(v) \
                                                        : GPIO1PIN_NP15_INITOUT(v) )

#define GPIO1PIN_NP15_INITIN(v)      do{ bFM_GPIO_PCR1_P5=(v).bPullup; \
                                         bFM_GPIO_DDR1_P5=0u; \
                                         bFM_GPIO_PFR1_P5=0u; \
                                         bFM_GPIO_ADE_AN05=0u; }while(0)


#define GPIO1PIN_NP15_INITOUT(v)     do{ bFM_GPIO_PDOR1_P5=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR1_P5=1u; \
                                         bFM_GPIO_PFR1_P5=0u; \
                                         bFM_GPIO_ADE_AN05=0u; }while(0)


/*---- GPIO bit P16 ----*/

#define GPIO1PIN_P16_GET             ( bFM_GPIO_PDIR1_P6 )

#define GPIO1PIN_P16_PUT(v)          ( bFM_GPIO_PDOR1_P6=(v) )

#define GPIO1PIN_P16_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P16_INITIN(v) \
                                                        : GPIO1PIN_P16_INITOUT(v) )

#define GPIO1PIN_P16_INITIN(v)       do{ bFM_GPIO_PCR1_P6=(v).bPullup; \
                                         bFM_GPIO_DDR1_P6=0u; \
                                         bFM_GPIO_PFR1_P6=0u; \
                                         bFM_GPIO_ADE_AN06=0u; }while(0)


#define GPIO1PIN_P16_INITOUT(v)      do{ bFM_GPIO_PDOR1_P6=(v).bInitVal; \
                                         bFM_GPIO_DDR1_P6=1u; \
                                         bFM_GPIO_PFR1_P6=0u; \
                                         bFM_GPIO_ADE_AN06=0u; }while(0)


/*---- GPIO bit NP16 ----*/
#define GPIO1PIN_NP16_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR1_P6)) )

#define GPIO1PIN_NP16_PUT(v)         ( bFM_GPIO_PDOR1_P6=(uint32_t)(!(v)) )

#define GPIO1PIN_NP16_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP16_INITIN(v) \
                                                        : GPIO1PIN_NP16_INITOUT(v) )

#define GPIO1PIN_NP16_INITIN(v)      do{ bFM_GPIO_PCR1_P6=(v).bPullup; \
                                         bFM_GPIO_DDR1_P6=0u; \
                                         bFM_GPIO_PFR1_P6=0u; \
                                         bFM_GPIO_ADE_AN06=0u; }while(0)


#define GPIO1PIN_NP16_INITOUT(v)     do{ bFM_GPIO_PDOR1_P6=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR1_P6=1u; \
                                         bFM_GPIO_PFR1_P6=0u; \
                                         bFM_GPIO_ADE_AN06=0u; }while(0)


/*---- GPIO bit P17 ----*/

#define GPIO1PIN_P17_GET             ( bFM_GPIO_PDIR1_P7 )

#define GPIO1PIN_P17_PUT(v)          ( bFM_GPIO_PDOR1_P7=(v) )

#define GPIO1PIN_P17_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P17_INITIN(v) \
                                                        : GPIO1PIN_P17_INITOUT(v) )

#define GPIO1PIN_P17_INITIN(v)       do{ bFM_GPIO_PCR1_P7=(v).bPullup; \
                                         bFM_GPIO_DDR1_P7=0u; \
                                         bFM_GPIO_PFR1_P7=0u; \
                                         bFM_GPIO_ADE_AN07=0u; }while(0)


#define GPIO1PIN_P17_INITOUT(v)      do{ bFM_GPIO_PDOR1_P7=(v).bInitVal; \
                                         bFM_GPIO_DDR1_P7=1u; \
                                         bFM_GPIO_PFR1_P7=0u; \
                                         bFM_GPIO_ADE_AN07=0u; }while(0)


/*---- GPIO bit NP17 ----*/
#define GPIO1PIN_NP17_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR1_P7)) )

#define GPIO1PIN_NP17_PUT(v)         ( bFM_GPIO_PDOR1_P7=(uint32_t)(!(v)) )

#define GPIO1PIN_NP17_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP17_INITIN(v) \
                                                        : GPIO1PIN_NP17_INITOUT(v) )

#define GPIO1PIN_NP17_INITIN(v)      do{ bFM_GPIO_PCR1_P7=(v).bPullup; \
                                         bFM_GPIO_DDR1_P7=0u; \
                                         bFM_GPIO_PFR1_P7=0u; \
                                         bFM_GPIO_ADE_AN07=0u; }while(0)


#define GPIO1PIN_NP17_INITOUT(v)     do{ bFM_GPIO_PDOR1_P7=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR1_P7=1u; \
                                         bFM_GPIO_PFR1_P7=0u; \
                                         bFM_GPIO_ADE_AN07=0u; }while(0)


/*---- GPIO bit P18 ----*/

#define GPIO1PIN_P18_GET             ( bFM_GPIO_PDIR1_P8 )

#define GPIO1PIN_P18_PUT(v)          ( bFM_GPIO_PDOR1_P8=(v) )

#define GPIO1PIN_P18_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P18_INITIN(v) \
                                                        : GPIO1PIN_P18_INITOUT(v) )

#define GPIO1PIN_P18_INITIN(v)       do{ bFM_GPIO_PCR1_P8=(v).bPullup; \
                                         bFM_GPIO_DDR1_P8=0u; \
                                         bFM_GPIO_PFR1_P8=0u; \
                                         bFM_GPIO_ADE_AN08=0u; }while(0)


#define GPIO1PIN_P18_INITOUT(v)      do{ bFM_GPIO_PDOR1_P8=(v).bInitVal; \
                                         bFM_GPIO_DDR1_P8=1u; \
                                         bFM_GPIO_PFR1_P8=0u; \
                                         bFM_GPIO_ADE_AN08=0u; }while(0)


/*---- GPIO bit NP18 ----*/
#define GPIO1PIN_NP18_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR1_P8)) )

#define GPIO1PIN_NP18_PUT(v)         ( bFM_GPIO_PDOR1_P8=(uint32_t)(!(v)) )

#define GPIO1PIN_NP18_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP18_INITIN(v) \
                                                        : GPIO1PIN_NP18_INITOUT(v) )

#define GPIO1PIN_NP18_INITIN(v)      do{ bFM_GPIO_PCR1_P8=(v).bPullup; \
                                         bFM_GPIO_DDR1_P8=0u; \
                                         bFM_GPIO_PFR1_P8=0u; \
                                         bFM_GPIO_ADE_AN08=0u; }while(0)


#define GPIO1PIN_NP18_INITOUT(v)     do{ bFM_GPIO_PDOR1_P8=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR1_P8=1u; \
                                         bFM_GPIO_PFR1_P8=0u; \
                                         bFM_GPIO_ADE_AN08=0u; }while(0)


/*---- GPIO bit P19 ----*/

#define GPIO1PIN_P19_GET             ( bFM_GPIO_PDIR1_P9 )

#define GPIO1PIN_P19_PUT(v)          ( bFM_GPIO_PDOR1_P9=(v) )

#define GPIO1PIN_P19_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P19_INITIN(v) \
                                                        : GPIO1PIN_P19_INITOUT(v) )

#define GPIO1PIN_P19_INITIN(v)       do{ bFM_GPIO_PCR1_P9=(v).bPullup; \
                                         bFM_GPIO_DDR1_P9=0u; \
                                         bFM_GPIO_PFR1_P9=0u; \
                                         bFM_GPIO_ADE_AN09=0u; }while(0)


#define GPIO1PIN_P19_INITOUT(v)      do{ bFM_GPIO_PDOR1_P9=(v).bInitVal; \
                                         bFM_GPIO_DDR1_P9=1u; \
                                         bFM_GPIO_PFR1_P9=0u; \
                                         bFM_GPIO_ADE_AN09=0u; }while(0)


/*---- GPIO bit NP19 ----*/
#define GPIO1PIN_NP19_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR1_P9)) )

#define GPIO1PIN_NP19_PUT(v)         ( bFM_GPIO_PDOR1_P9=(uint32_t)(!(v)) )

#define GPIO1PIN_NP19_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP19_INITIN(v) \
                                                        : GPIO1PIN_NP19_INITOUT(v) )

#define GPIO1PIN_NP19_INITIN(v)      do{ bFM_GPIO_PCR1_P9=(v).bPullup; \
                                         bFM_GPIO_DDR1_P9=0u; \
                                         bFM_GPIO_PFR1_P9=0u; \
                                         bFM_GPIO_ADE_AN09=0u; }while(0)


#define GPIO1PIN_NP19_INITOUT(v)     do{ bFM_GPIO_PDOR1_P9=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR1_P9=1u; \
                                         bFM_GPIO_PFR1_P9=0u; \
                                         bFM_GPIO_ADE_AN09=0u; }while(0)


/*---- GPIO bit P1A ----*/

#define GPIO1PIN_P1A_GET             ( bFM_GPIO_PDIR1_PA )

#define GPIO1PIN_P1A_PUT(v)          ( bFM_GPIO_PDOR1_PA=(v) )

#define GPIO1PIN_P1A_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P1A_INITIN(v) \
                                                        : GPIO1PIN_P1A_INITOUT(v) )

#define GPIO1PIN_P1A_INITIN(v)       do{ bFM_GPIO_PCR1_PA=(v).bPullup; \
                                         bFM_GPIO_DDR1_PA=0u; \
                                         bFM_GPIO_PFR1_PA=0u; \
                                         bFM_GPIO_ADE_AN10=0u; }while(0)


#define GPIO1PIN_P1A_INITOUT(v)      do{ bFM_GPIO_PDOR1_PA=(v).bInitVal; \
                                         bFM_GPIO_DDR1_PA=1u; \
                                         bFM_GPIO_PFR1_PA=0u; \
                                         bFM_GPIO_ADE_AN10=0u; }while(0)


/*---- GPIO bit NP1A ----*/
#define GPIO1PIN_NP1A_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR1_PA)) )

#define GPIO1PIN_NP1A_PUT(v)         ( bFM_GPIO_PDOR1_PA=(uint32_t)(!(v)) )

#define GPIO1PIN_NP1A_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP1A_INITIN(v) \
                                                        : GPIO1PIN_NP1A_INITOUT(v) )

#define GPIO1PIN_NP1A_INITIN(v)      do{ bFM_GPIO_PCR1_PA=(v).bPullup; \
                                         bFM_GPIO_DDR1_PA=0u; \
                                         bFM_GPIO_PFR1_PA=0u; \
                                         bFM_GPIO_ADE_AN10=0u; }while(0)


#define GPIO1PIN_NP1A_INITOUT(v)     do{ bFM_GPIO_PDOR1_PA=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR1_PA=1u; \
                                         bFM_GPIO_PFR1_PA=0u; \
                                         bFM_GPIO_ADE_AN10=0u; }while(0)


/*---- GPIO bit P1B ----*/

#define GPIO1PIN_P1B_GET             ( bFM_GPIO_PDIR1_PB )

#define GPIO1PIN_P1B_PUT(v)          ( bFM_GPIO_PDOR1_PB=(v) )

#define GPIO1PIN_P1B_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P1B_INITIN(v) \
                                                        : GPIO1PIN_P1B_INITOUT(v) )

#define GPIO1PIN_P1B_INITIN(v)       do{ bFM_GPIO_PCR1_PB=(v).bPullup; \
                                         bFM_GPIO_DDR1_PB=0u; \
                                         bFM_GPIO_PFR1_PB=0u; \
                                         bFM_GPIO_ADE_AN11=0u; }while(0)


#define GPIO1PIN_P1B_INITOUT(v)      do{ bFM_GPIO_PDOR1_PB=(v).bInitVal; \
                                         bFM_GPIO_DDR1_PB=1u; \
                                         bFM_GPIO_PFR1_PB=0u; \
                                         bFM_GPIO_ADE_AN11=0u; }while(0)


/*---- GPIO bit NP1B ----*/
#define GPIO1PIN_NP1B_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR1_PB)) )

#define GPIO1PIN_NP1B_PUT(v)         ( bFM_GPIO_PDOR1_PB=(uint32_t)(!(v)) )

#define GPIO1PIN_NP1B_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP1B_INITIN(v) \
                                                        : GPIO1PIN_NP1B_INITOUT(v) )

#define GPIO1PIN_NP1B_INITIN(v)      do{ bFM_GPIO_PCR1_PB=(v).bPullup; \
                                         bFM_GPIO_DDR1_PB=0u; \
                                         bFM_GPIO_PFR1_PB=0u; \
                                         bFM_GPIO_ADE_AN11=0u; }while(0)


#define GPIO1PIN_NP1B_INITOUT(v)     do{ bFM_GPIO_PDOR1_PB=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR1_PB=1u; \
                                         bFM_GPIO_PFR1_PB=0u; \
                                         bFM_GPIO_ADE_AN11=0u; }while(0)


/*---- GPIO bit P1C ----*/

#define GPIO1PIN_P1C_GET             ( bFM_GPIO_PDIR1_PC )

#define GPIO1PIN_P1C_PUT(v)          ( bFM_GPIO_PDOR1_PC=(v) )

#define GPIO1PIN_P1C_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P1C_INITIN(v) \
                                                        : GPIO1PIN_P1C_INITOUT(v) )

#define GPIO1PIN_P1C_INITIN(v)       do{ bFM_GPIO_PCR1_PC=(v).bPullup; \
                                         bFM_GPIO_DDR1_PC=0u; \
                                         bFM_GPIO_PFR1_PC=0u; \
                                         bFM_GPIO_ADE_AN12=0u; }while(0)


#define GPIO1PIN_P1C_INITOUT(v)      do{ bFM_GPIO_PDOR1_PC=(v).bInitVal; \
                                         bFM_GPIO_DDR1_PC=1u; \
                                         bFM_GPIO_PFR1_PC=0u; \
                                         bFM_GPIO_ADE_AN12=0u; }while(0)


/*---- GPIO bit NP1C ----*/
#define GPIO1PIN_NP1C_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR1_PC)) )

#define GPIO1PIN_NP1C_PUT(v)         ( bFM_GPIO_PDOR1_PC=(uint32_t)(!(v)) )

#define GPIO1PIN_NP1C_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP1C_INITIN(v) \
                                                        : GPIO1PIN_NP1C_INITOUT(v) )

#define GPIO1PIN_NP1C_INITIN(v)      do{ bFM_GPIO_PCR1_PC=(v).bPullup; \
                                         bFM_GPIO_DDR1_PC=0u; \
                                         bFM_GPIO_PFR1_PC=0u; \
                                         bFM_GPIO_ADE_AN12=0u; }while(0)


#define GPIO1PIN_NP1C_INITOUT(v)     do{ bFM_GPIO_PDOR1_PC=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR1_PC=1u; \
                                         bFM_GPIO_PFR1_PC=0u; \
                                         bFM_GPIO_ADE_AN12=0u; }while(0)


/*---- GPIO bit P1D ----*/

#define GPIO1PIN_P1D_GET             ( bFM_GPIO_PDIR1_PD )

#define GPIO1PIN_P1D_PUT(v)          ( bFM_GPIO_PDOR1_PD=(v) )

#define GPIO1PIN_P1D_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P1D_INITIN(v) \
                                                        : GPIO1PIN_P1D_INITOUT(v) )

#define GPIO1PIN_P1D_INITIN(v)       do{ bFM_GPIO_PCR1_PD=(v).bPullup; \
                                         bFM_GPIO_DDR1_PD=0u; \
                                         bFM_GPIO_PFR1_PD=0u; \
                                         bFM_GPIO_ADE_AN13=0u; }while(0)


#define GPIO1PIN_P1D_INITOUT(v)      do{ bFM_GPIO_PDOR1_PD=(v).bInitVal; \
                                         bFM_GPIO_DDR1_PD=1u; \
                                         bFM_GPIO_PFR1_PD=0u; \
                                         bFM_GPIO_ADE_AN13=0u; }while(0)


/*---- GPIO bit NP1D ----*/
#define GPIO1PIN_NP1D_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR1_PD)) )

#define GPIO1PIN_NP1D_PUT(v)         ( bFM_GPIO_PDOR1_PD=(uint32_t)(!(v)) )

#define GPIO1PIN_NP1D_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP1D_INITIN(v) \
                                                        : GPIO1PIN_NP1D_INITOUT(v) )

#define GPIO1PIN_NP1D_INITIN(v)      do{ bFM_GPIO_PCR1_PD=(v).bPullup; \
                                         bFM_GPIO_DDR1_PD=0u; \
                                         bFM_GPIO_PFR1_PD=0u; \
                                         bFM_GPIO_ADE_AN13=0u; }while(0)


#define GPIO1PIN_NP1D_INITOUT(v)     do{ bFM_GPIO_PDOR1_PD=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR1_PD=1u; \
                                         bFM_GPIO_PFR1_PD=0u; \
                                         bFM_GPIO_ADE_AN13=0u; }while(0)


/*---- GPIO bit P1E ----*/

#define GPIO1PIN_P1E_GET             ( bFM_GPIO_PDIR1_PE )

#define GPIO1PIN_P1E_PUT(v)          ( bFM_GPIO_PDOR1_PE=(v) )

#define GPIO1PIN_P1E_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P1E_INITIN(v) \
                                                        : GPIO1PIN_P1E_INITOUT(v) )

#define GPIO1PIN_P1E_INITIN(v)       do{ bFM_GPIO_PCR1_PE=(v).bPullup; \
                                         bFM_GPIO_DDR1_PE=0u; \
                                         bFM_GPIO_PFR1_PE=0u; \
                                         bFM_GPIO_ADE_AN14=0u; }while(0)


#define GPIO1PIN_P1E_INITOUT(v)      do{ bFM_GPIO_PDOR1_PE=(v).bInitVal; \
                                         bFM_GPIO_DDR1_PE=1u; \
                                         bFM_GPIO_PFR1_PE=0u; \
                                         bFM_GPIO_ADE_AN14=0u; }while(0)


/*---- GPIO bit NP1E ----*/
#define GPIO1PIN_NP1E_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR1_PE)) )

#define GPIO1PIN_NP1E_PUT(v)         ( bFM_GPIO_PDOR1_PE=(uint32_t)(!(v)) )

#define GPIO1PIN_NP1E_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP1E_INITIN(v) \
                                                        : GPIO1PIN_NP1E_INITOUT(v) )

#define GPIO1PIN_NP1E_INITIN(v)      do{ bFM_GPIO_PCR1_PE=(v).bPullup; \
                                         bFM_GPIO_DDR1_PE=0u; \
                                         bFM_GPIO_PFR1_PE=0u; \
                                         bFM_GPIO_ADE_AN14=0u; }while(0)


#define GPIO1PIN_NP1E_INITOUT(v)     do{ bFM_GPIO_PDOR1_PE=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR1_PE=1u; \
                                         bFM_GPIO_PFR1_PE=0u; \
                                         bFM_GPIO_ADE_AN14=0u; }while(0)


/*---- GPIO bit P1F ----*/

#define GPIO1PIN_P1F_GET             ( bFM_GPIO_PDIR1_PF )

#define GPIO1PIN_P1F_PUT(v)          ( bFM_GPIO_PDOR1_PF=(v) )

#define GPIO1PIN_P1F_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P1F_INITIN(v) \
                                                        : GPIO1PIN_P1F_INITOUT(v) )

#define GPIO1PIN_P1F_INITIN(v)       do{ bFM_GPIO_PCR1_PF=(v).bPullup; \
                                         bFM_GPIO_DDR1_PF=0u; \
                                         bFM_GPIO_PFR1_PF=0u; \
                                         bFM_GPIO_ADE_AN15=0u; }while(0)


#define GPIO1PIN_P1F_INITOUT(v)      do{ bFM_GPIO_PDOR1_PF=(v).bInitVal; \
                                         bFM_GPIO_DDR1_PF=1u; \
                                         bFM_GPIO_PFR1_PF=0u; \
                                         bFM_GPIO_ADE_AN15=0u; }while(0)


/*---- GPIO bit NP1F ----*/
#define GPIO1PIN_NP1F_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR1_PF)) )

#define GPIO1PIN_NP1F_PUT(v)         ( bFM_GPIO_PDOR1_PF=(uint32_t)(!(v)) )

#define GPIO1PIN_NP1F_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP1F_INITIN(v) \
                                                        : GPIO1PIN_NP1F_INITOUT(v) )

#define GPIO1PIN_NP1F_INITIN(v)      do{ bFM_GPIO_PCR1_PF=(v).bPullup; \
                                         bFM_GPIO_DDR1_PF=0u; \
                                         bFM_GPIO_PFR1_PF=0u; \
                                         bFM_GPIO_ADE_AN15=0u; }while(0)


#define GPIO1PIN_NP1F_INITOUT(v)     do{ bFM_GPIO_PDOR1_PF=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR1_PF=1u; \
                                         bFM_GPIO_PFR1_PF=0u; \
                                         bFM_GPIO_ADE_AN15=0u; }while(0)


/*---- GPIO bit P20 ----*/

#define GPIO1PIN_P20_GET             ( bFM_GPIO_PDIR2_P0 )

#define GPIO1PIN_P20_PUT(v)          ( bFM_GPIO_PDOR2_P0=(v) )

#define GPIO1PIN_P20_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P20_INITIN(v) \
                                                        : GPIO1PIN_P20_INITOUT(v) )

#define GPIO1PIN_P20_INITIN(v)       do{ bFM_GPIO_PCR2_P0=(v).bPullup; \
                                         bFM_GPIO_DDR2_P0=0u; \
                                         bFM_GPIO_PFR2_P0=0u; }while(0)

#define GPIO1PIN_P20_INITOUT(v)      do{ bFM_GPIO_PDOR2_P0=(v).bInitVal; \
                                         bFM_GPIO_DDR2_P0=1u; \
                                         bFM_GPIO_PFR2_P0=0u; }while(0)


/*---- GPIO bit NP20 ----*/
#define GPIO1PIN_NP20_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR2_P0)) )

#define GPIO1PIN_NP20_PUT(v)         ( bFM_GPIO_PDOR2_P0=(uint32_t)(!(v)) )

#define GPIO1PIN_NP20_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP20_INITIN(v) \
                                                        : GPIO1PIN_NP20_INITOUT(v) )

#define GPIO1PIN_NP20_INITIN(v)      do{ bFM_GPIO_PCR2_P0=(v).bPullup; \
                                         bFM_GPIO_DDR2_P0=0u; \
                                         bFM_GPIO_PFR2_P0=0u; }while(0)

#define GPIO1PIN_NP20_INITOUT(v)     do{ bFM_GPIO_PDOR2_P0=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR2_P0=1u; \
                                         bFM_GPIO_PFR2_P0=0u; }while(0)


/*---- GPIO bit P21 ----*/

#define GPIO1PIN_P21_GET             ( bFM_GPIO_PDIR2_P1 )

#define GPIO1PIN_P21_PUT(v)          ( bFM_GPIO_PDOR2_P1=(v) )

#define GPIO1PIN_P21_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P21_INITIN(v) \
                                                        : GPIO1PIN_P21_INITOUT(v) )

#define GPIO1PIN_P21_INITIN(v)       do{ bFM_GPIO_PCR2_P1=(v).bPullup; \
                                         bFM_GPIO_DDR2_P1=0u; \
                                         bFM_GPIO_PFR2_P1=0u; }while(0)

#define GPIO1PIN_P21_INITOUT(v)      do{ bFM_GPIO_PDOR2_P1=(v).bInitVal; \
                                         bFM_GPIO_DDR2_P1=1u; \
                                         bFM_GPIO_PFR2_P1=0u; }while(0)


/*---- GPIO bit NP21 ----*/
#define GPIO1PIN_NP21_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR2_P1)) )

#define GPIO1PIN_NP21_PUT(v)         ( bFM_GPIO_PDOR2_P1=(uint32_t)(!(v)) )

#define GPIO1PIN_NP21_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP21_INITIN(v) \
                                                        : GPIO1PIN_NP21_INITOUT(v) )

#define GPIO1PIN_NP21_INITIN(v)      do{ bFM_GPIO_PCR2_P1=(v).bPullup; \
                                         bFM_GPIO_DDR2_P1=0u; \
                                         bFM_GPIO_PFR2_P1=0u; }while(0)

#define GPIO1PIN_NP21_INITOUT(v)     do{ bFM_GPIO_PDOR2_P1=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR2_P1=1u; \
                                         bFM_GPIO_PFR2_P1=0u; }while(0)


/*---- GPIO bit P22 ----*/

#define GPIO1PIN_P22_GET             ( bFM_GPIO_PDIR2_P2 )

#define GPIO1PIN_P22_PUT(v)          ( bFM_GPIO_PDOR2_P2=(v) )

#define GPIO1PIN_P22_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P22_INITIN(v) \
                                                        : GPIO1PIN_P22_INITOUT(v) )

#define GPIO1PIN_P22_INITIN(v)       do{ bFM_GPIO_PCR2_P2=(v).bPullup; \
                                         bFM_GPIO_DDR2_P2=0u; \
                                         bFM_GPIO_PFR2_P2=0u; \
                                         bFM_GPIO_ADE_AN31=0u; }while(0)


#define GPIO1PIN_P22_INITOUT(v)      do{ bFM_GPIO_PDOR2_P2=(v).bInitVal; \
                                         bFM_GPIO_DDR2_P2=1u; \
                                         bFM_GPIO_PFR2_P2=0u; \
                                         bFM_GPIO_ADE_AN31=0u; }while(0)


/*---- GPIO bit NP22 ----*/
#define GPIO1PIN_NP22_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR2_P2)) )

#define GPIO1PIN_NP22_PUT(v)         ( bFM_GPIO_PDOR2_P2=(uint32_t)(!(v)) )

#define GPIO1PIN_NP22_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP22_INITIN(v) \
                                                        : GPIO1PIN_NP22_INITOUT(v) )

#define GPIO1PIN_NP22_INITIN(v)      do{ bFM_GPIO_PCR2_P2=(v).bPullup; \
                                         bFM_GPIO_DDR2_P2=0u; \
                                         bFM_GPIO_PFR2_P2=0u; \
                                         bFM_GPIO_ADE_AN31=0u; }while(0)


#define GPIO1PIN_NP22_INITOUT(v)     do{ bFM_GPIO_PDOR2_P2=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR2_P2=1u; \
                                         bFM_GPIO_PFR2_P2=0u; \
                                         bFM_GPIO_ADE_AN31=0u; }while(0)


/*---- GPIO bit P23 ----*/

#define GPIO1PIN_P23_GET             ( bFM_GPIO_PDIR2_P3 )

#define GPIO1PIN_P23_PUT(v)          ( bFM_GPIO_PDOR2_P3=(v) )

#define GPIO1PIN_P23_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P23_INITIN(v) \
                                                        : GPIO1PIN_P23_INITOUT(v) )

#define GPIO1PIN_P23_INITIN(v)       do{ bFM_GPIO_PCR2_P3=(v).bPullup; \
                                         bFM_GPIO_DDR2_P3=0u; \
                                         bFM_GPIO_PFR2_P3=0u; \
                                         bFM_GPIO_ADE_AN30=0u; }while(0)


#define GPIO1PIN_P23_INITOUT(v)      do{ bFM_GPIO_PDOR2_P3=(v).bInitVal; \
                                         bFM_GPIO_DDR2_P3=1u; \
                                         bFM_GPIO_PFR2_P3=0u; \
                                         bFM_GPIO_ADE_AN30=0u; }while(0)


/*---- GPIO bit NP23 ----*/
#define GPIO1PIN_NP23_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR2_P3)) )

#define GPIO1PIN_NP23_PUT(v)         ( bFM_GPIO_PDOR2_P3=(uint32_t)(!(v)) )

#define GPIO1PIN_NP23_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP23_INITIN(v) \
                                                        : GPIO1PIN_NP23_INITOUT(v) )

#define GPIO1PIN_NP23_INITIN(v)      do{ bFM_GPIO_PCR2_P3=(v).bPullup; \
                                         bFM_GPIO_DDR2_P3=0u; \
                                         bFM_GPIO_PFR2_P3=0u; \
                                         bFM_GPIO_ADE_AN30=0u; }while(0)


#define GPIO1PIN_NP23_INITOUT(v)     do{ bFM_GPIO_PDOR2_P3=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR2_P3=1u; \
                                         bFM_GPIO_PFR2_P3=0u; \
                                         bFM_GPIO_ADE_AN30=0u; }while(0)


/*---- GPIO bit P24 ----*/

#define GPIO1PIN_P24_GET             ( bFM_GPIO_PDIR2_P4 )

#define GPIO1PIN_P24_PUT(v)          ( bFM_GPIO_PDOR2_P4=(v) )

#define GPIO1PIN_P24_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P24_INITIN(v) \
                                                        : GPIO1PIN_P24_INITOUT(v) )

#define GPIO1PIN_P24_INITIN(v)       do{ bFM_GPIO_PCR2_P4=(v).bPullup; \
                                         bFM_GPIO_DDR2_P4=0u; \
                                         bFM_GPIO_PFR2_P4=0u; \
                                         bFM_GPIO_ADE_AN29=0u; }while(0)


#define GPIO1PIN_P24_INITOUT(v)      do{ bFM_GPIO_PDOR2_P4=(v).bInitVal; \
                                         bFM_GPIO_DDR2_P4=1u; \
                                         bFM_GPIO_PFR2_P4=0u; \
                                         bFM_GPIO_ADE_AN29=0u; }while(0)


/*---- GPIO bit NP24 ----*/
#define GPIO1PIN_NP24_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR2_P4)) )

#define GPIO1PIN_NP24_PUT(v)         ( bFM_GPIO_PDOR2_P4=(uint32_t)(!(v)) )

#define GPIO1PIN_NP24_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP24_INITIN(v) \
                                                        : GPIO1PIN_NP24_INITOUT(v) )

#define GPIO1PIN_NP24_INITIN(v)      do{ bFM_GPIO_PCR2_P4=(v).bPullup; \
                                         bFM_GPIO_DDR2_P4=0u; \
                                         bFM_GPIO_PFR2_P4=0u; \
                                         bFM_GPIO_ADE_AN29=0u; }while(0)


#define GPIO1PIN_NP24_INITOUT(v)     do{ bFM_GPIO_PDOR2_P4=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR2_P4=1u; \
                                         bFM_GPIO_PFR2_P4=0u; \
                                         bFM_GPIO_ADE_AN29=0u; }while(0)


/*---- GPIO bit P25 ----*/

#define GPIO1PIN_P25_GET             ( bFM_GPIO_PDIR2_P5 )

#define GPIO1PIN_P25_PUT(v)          ( bFM_GPIO_PDOR2_P5=(v) )

#define GPIO1PIN_P25_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P25_INITIN(v) \
                                                        : GPIO1PIN_P25_INITOUT(v) )

#define GPIO1PIN_P25_INITIN(v)       do{ bFM_GPIO_PCR2_P5=(v).bPullup; \
                                         bFM_GPIO_DDR2_P5=0u; \
                                         bFM_GPIO_PFR2_P5=0u; \
                                         bFM_GPIO_ADE_AN28=0u; }while(0)


#define GPIO1PIN_P25_INITOUT(v)      do{ bFM_GPIO_PDOR2_P5=(v).bInitVal; \
                                         bFM_GPIO_DDR2_P5=1u; \
                                         bFM_GPIO_PFR2_P5=0u; \
                                         bFM_GPIO_ADE_AN28=0u; }while(0)


/*---- GPIO bit NP25 ----*/
#define GPIO1PIN_NP25_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR2_P5)) )

#define GPIO1PIN_NP25_PUT(v)         ( bFM_GPIO_PDOR2_P5=(uint32_t)(!(v)) )

#define GPIO1PIN_NP25_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP25_INITIN(v) \
                                                        : GPIO1PIN_NP25_INITOUT(v) )

#define GPIO1PIN_NP25_INITIN(v)      do{ bFM_GPIO_PCR2_P5=(v).bPullup; \
                                         bFM_GPIO_DDR2_P5=0u; \
                                         bFM_GPIO_PFR2_P5=0u; \
                                         bFM_GPIO_ADE_AN28=0u; }while(0)


#define GPIO1PIN_NP25_INITOUT(v)     do{ bFM_GPIO_PDOR2_P5=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR2_P5=1u; \
                                         bFM_GPIO_PFR2_P5=0u; \
                                         bFM_GPIO_ADE_AN28=0u; }while(0)


/*---- GPIO bit P26 ----*/

#define GPIO1PIN_P26_GET             ( bFM_GPIO_PDIR2_P6 )

#define GPIO1PIN_P26_PUT(v)          ( bFM_GPIO_PDOR2_P6=(v) )

#define GPIO1PIN_P26_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P26_INITIN(v) \
                                                        : GPIO1PIN_P26_INITOUT(v) )

#define GPIO1PIN_P26_INITIN(v)       do{ bFM_GPIO_PCR2_P6=(v).bPullup; \
                                         bFM_GPIO_DDR2_P6=0u; \
                                         bFM_GPIO_PFR2_P6=0u; \
                                         bFM_GPIO_ADE_AN27=0u; }while(0)


#define GPIO1PIN_P26_INITOUT(v)      do{ bFM_GPIO_PDOR2_P6=(v).bInitVal; \
                                         bFM_GPIO_DDR2_P6=1u; \
                                         bFM_GPIO_PFR2_P6=0u; \
                                         bFM_GPIO_ADE_AN27=0u; }while(0)


/*---- GPIO bit NP26 ----*/
#define GPIO1PIN_NP26_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR2_P6)) )

#define GPIO1PIN_NP26_PUT(v)         ( bFM_GPIO_PDOR2_P6=(uint32_t)(!(v)) )

#define GPIO1PIN_NP26_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP26_INITIN(v) \
                                                        : GPIO1PIN_NP26_INITOUT(v) )

#define GPIO1PIN_NP26_INITIN(v)      do{ bFM_GPIO_PCR2_P6=(v).bPullup; \
                                         bFM_GPIO_DDR2_P6=0u; \
                                         bFM_GPIO_PFR2_P6=0u; \
                                         bFM_GPIO_ADE_AN27=0u; }while(0)


#define GPIO1PIN_NP26_INITOUT(v)     do{ bFM_GPIO_PDOR2_P6=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR2_P6=1u; \
                                         bFM_GPIO_PFR2_P6=0u; \
                                         bFM_GPIO_ADE_AN27=0u; }while(0)


/*---- GPIO bit P27 ----*/

#define GPIO1PIN_P27_GET             ( bFM_GPIO_PDIR2_P7 )

#define GPIO1PIN_P27_PUT(v)          ( bFM_GPIO_PDOR2_P7=(v) )

#define GPIO1PIN_P27_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P27_INITIN(v) \
                                                        : GPIO1PIN_P27_INITOUT(v) )

#define GPIO1PIN_P27_INITIN(v)       do{ bFM_GPIO_PCR2_P7=(v).bPullup; \
                                         bFM_GPIO_DDR2_P7=0u; \
                                         bFM_GPIO_PFR2_P7=0u; \
                                         bFM_GPIO_ADE_AN26=0u; }while(0)


#define GPIO1PIN_P27_INITOUT(v)      do{ bFM_GPIO_PDOR2_P7=(v).bInitVal; \
                                         bFM_GPIO_DDR2_P7=1u; \
                                         bFM_GPIO_PFR2_P7=0u; \
                                         bFM_GPIO_ADE_AN26=0u; }while(0)


/*---- GPIO bit NP27 ----*/
#define GPIO1PIN_NP27_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR2_P7)) )

#define GPIO1PIN_NP27_PUT(v)         ( bFM_GPIO_PDOR2_P7=(uint32_t)(!(v)) )

#define GPIO1PIN_NP27_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP27_INITIN(v) \
                                                        : GPIO1PIN_NP27_INITOUT(v) )

#define GPIO1PIN_NP27_INITIN(v)      do{ bFM_GPIO_PCR2_P7=(v).bPullup; \
                                         bFM_GPIO_DDR2_P7=0u; \
                                         bFM_GPIO_PFR2_P7=0u; \
                                         bFM_GPIO_ADE_AN26=0u; }while(0)


#define GPIO1PIN_NP27_INITOUT(v)     do{ bFM_GPIO_PDOR2_P7=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR2_P7=1u; \
                                         bFM_GPIO_PFR2_P7=0u; \
                                         bFM_GPIO_ADE_AN26=0u; }while(0)


/*---- GPIO bit P28 ----*/

#define GPIO1PIN_P28_GET             ( bFM_GPIO_PDIR2_P8 )

#define GPIO1PIN_P28_PUT(v)          ( bFM_GPIO_PDOR2_P8=(v) )

#define GPIO1PIN_P28_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P28_INITIN(v) \
                                                        : GPIO1PIN_P28_INITOUT(v) )

#define GPIO1PIN_P28_INITIN(v)       do{ bFM_GPIO_PCR2_P8=(v).bPullup; \
                                         bFM_GPIO_DDR2_P8=0u; \
                                         bFM_GPIO_PFR2_P8=0u; \
                                         bFM_GPIO_ADE_AN25=0u; }while(0)


#define GPIO1PIN_P28_INITOUT(v)      do{ bFM_GPIO_PDOR2_P8=(v).bInitVal; \
                                         bFM_GPIO_DDR2_P8=1u; \
                                         bFM_GPIO_PFR2_P8=0u; \
                                         bFM_GPIO_ADE_AN25=0u; }while(0)


/*---- GPIO bit NP28 ----*/
#define GPIO1PIN_NP28_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR2_P8)) )

#define GPIO1PIN_NP28_PUT(v)         ( bFM_GPIO_PDOR2_P8=(uint32_t)(!(v)) )

#define GPIO1PIN_NP28_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP28_INITIN(v) \
                                                        : GPIO1PIN_NP28_INITOUT(v) )

#define GPIO1PIN_NP28_INITIN(v)      do{ bFM_GPIO_PCR2_P8=(v).bPullup; \
                                         bFM_GPIO_DDR2_P8=0u; \
                                         bFM_GPIO_PFR2_P8=0u; \
                                         bFM_GPIO_ADE_AN25=0u; }while(0)


#define GPIO1PIN_NP28_INITOUT(v)     do{ bFM_GPIO_PDOR2_P8=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR2_P8=1u; \
                                         bFM_GPIO_PFR2_P8=0u; \
                                         bFM_GPIO_ADE_AN25=0u; }while(0)


/*---- GPIO bit P29 ----*/

#define GPIO1PIN_P29_GET             ( bFM_GPIO_PDIR2_P9 )

#define GPIO1PIN_P29_PUT(v)          ( bFM_GPIO_PDOR2_P9=(v) )

#define GPIO1PIN_P29_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P29_INITIN(v) \
                                                        : GPIO1PIN_P29_INITOUT(v) )

#define GPIO1PIN_P29_INITIN(v)       do{ bFM_GPIO_PCR2_P9=(v).bPullup; \
                                         bFM_GPIO_DDR2_P9=0u; \
                                         bFM_GPIO_PFR2_P9=0u; \
                                         bFM_GPIO_ADE_AN24=0u; }while(0)


#define GPIO1PIN_P29_INITOUT(v)      do{ bFM_GPIO_PDOR2_P9=(v).bInitVal; \
                                         bFM_GPIO_DDR2_P9=1u; \
                                         bFM_GPIO_PFR2_P9=0u; \
                                         bFM_GPIO_ADE_AN24=0u; }while(0)


/*---- GPIO bit NP29 ----*/
#define GPIO1PIN_NP29_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR2_P9)) )

#define GPIO1PIN_NP29_PUT(v)         ( bFM_GPIO_PDOR2_P9=(uint32_t)(!(v)) )

#define GPIO1PIN_NP29_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP29_INITIN(v) \
                                                        : GPIO1PIN_NP29_INITOUT(v) )

#define GPIO1PIN_NP29_INITIN(v)      do{ bFM_GPIO_PCR2_P9=(v).bPullup; \
                                         bFM_GPIO_DDR2_P9=0u; \
                                         bFM_GPIO_PFR2_P9=0u; \
                                         bFM_GPIO_ADE_AN24=0u; }while(0)


#define GPIO1PIN_NP29_INITOUT(v)     do{ bFM_GPIO_PDOR2_P9=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR2_P9=1u; \
                                         bFM_GPIO_PFR2_P9=0u; \
                                         bFM_GPIO_ADE_AN24=0u; }while(0)


/*---- GPIO bit P36 ----*/

#define GPIO1PIN_P36_GET             ( bFM_GPIO_PDIR3_P6 )

#define GPIO1PIN_P36_PUT(v)          ( bFM_GPIO_PDOR3_P6=(v) )

#define GPIO1PIN_P36_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P36_INITIN(v) \
                                                        : GPIO1PIN_P36_INITOUT(v) )

#define GPIO1PIN_P36_INITIN(v)       do{ bFM_GPIO_PCR3_P6=(v).bPullup; \
                                         bFM_GPIO_DDR3_P6=0u; \
                                         bFM_GPIO_PFR3_P6=0u; }while(0)

#define GPIO1PIN_P36_INITOUT(v)      do{ bFM_GPIO_PDOR3_P6=(v).bInitVal; \
                                         bFM_GPIO_DDR3_P6=1u; \
                                         bFM_GPIO_PFR3_P6=0u; }while(0)


/*---- GPIO bit NP36 ----*/
#define GPIO1PIN_NP36_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR3_P6)) )

#define GPIO1PIN_NP36_PUT(v)         ( bFM_GPIO_PDOR3_P6=(uint32_t)(!(v)) )

#define GPIO1PIN_NP36_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP36_INITIN(v) \
                                                        : GPIO1PIN_NP36_INITOUT(v) )

#define GPIO1PIN_NP36_INITIN(v)      do{ bFM_GPIO_PCR3_P6=(v).bPullup; \
                                         bFM_GPIO_DDR3_P6=0u; \
                                         bFM_GPIO_PFR3_P6=0u; }while(0)

#define GPIO1PIN_NP36_INITOUT(v)     do{ bFM_GPIO_PDOR3_P6=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR3_P6=1u; \
                                         bFM_GPIO_PFR3_P6=0u; }while(0)


/*---- GPIO bit P37 ----*/

#define GPIO1PIN_P37_GET             ( bFM_GPIO_PDIR3_P7 )

#define GPIO1PIN_P37_PUT(v)          ( bFM_GPIO_PDOR3_P7=(v) )

#define GPIO1PIN_P37_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P37_INITIN(v) \
                                                        : GPIO1PIN_P37_INITOUT(v) )

#define GPIO1PIN_P37_INITIN(v)       do{ bFM_GPIO_PCR3_P7=(v).bPullup; \
                                         bFM_GPIO_DDR3_P7=0u; \
                                         bFM_GPIO_PFR3_P7=0u; }while(0)

#define GPIO1PIN_P37_INITOUT(v)      do{ bFM_GPIO_PDOR3_P7=(v).bInitVal; \
                                         bFM_GPIO_DDR3_P7=1u; \
                                         bFM_GPIO_PFR3_P7=0u; }while(0)


/*---- GPIO bit NP37 ----*/
#define GPIO1PIN_NP37_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR3_P7)) )

#define GPIO1PIN_NP37_PUT(v)         ( bFM_GPIO_PDOR3_P7=(uint32_t)(!(v)) )

#define GPIO1PIN_NP37_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP37_INITIN(v) \
                                                        : GPIO1PIN_NP37_INITOUT(v) )

#define GPIO1PIN_NP37_INITIN(v)      do{ bFM_GPIO_PCR3_P7=(v).bPullup; \
                                         bFM_GPIO_DDR3_P7=0u; \
                                         bFM_GPIO_PFR3_P7=0u; }while(0)

#define GPIO1PIN_NP37_INITOUT(v)     do{ bFM_GPIO_PDOR3_P7=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR3_P7=1u; \
                                         bFM_GPIO_PFR3_P7=0u; }while(0)


/*---- GPIO bit P38 ----*/

#define GPIO1PIN_P38_GET             ( bFM_GPIO_PDIR3_P8 )

#define GPIO1PIN_P38_PUT(v)          ( bFM_GPIO_PDOR3_P8=(v) )

#define GPIO1PIN_P38_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P38_INITIN(v) \
                                                        : GPIO1PIN_P38_INITOUT(v) )

#define GPIO1PIN_P38_INITIN(v)       do{ bFM_GPIO_PCR3_P8=(v).bPullup; \
                                         bFM_GPIO_DDR3_P8=0u; \
                                         bFM_GPIO_PFR3_P8=0u; }while(0)

#define GPIO1PIN_P38_INITOUT(v)      do{ bFM_GPIO_PDOR3_P8=(v).bInitVal; \
                                         bFM_GPIO_DDR3_P8=1u; \
                                         bFM_GPIO_PFR3_P8=0u; }while(0)


/*---- GPIO bit NP38 ----*/
#define GPIO1PIN_NP38_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR3_P8)) )

#define GPIO1PIN_NP38_PUT(v)         ( bFM_GPIO_PDOR3_P8=(uint32_t)(!(v)) )

#define GPIO1PIN_NP38_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP38_INITIN(v) \
                                                        : GPIO1PIN_NP38_INITOUT(v) )

#define GPIO1PIN_NP38_INITIN(v)      do{ bFM_GPIO_PCR3_P8=(v).bPullup; \
                                         bFM_GPIO_DDR3_P8=0u; \
                                         bFM_GPIO_PFR3_P8=0u; }while(0)

#define GPIO1PIN_NP38_INITOUT(v)     do{ bFM_GPIO_PDOR3_P8=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR3_P8=1u; \
                                         bFM_GPIO_PFR3_P8=0u; }while(0)


/*---- GPIO bit P39 ----*/

#define GPIO1PIN_P39_GET             ( bFM_GPIO_PDIR3_P9 )

#define GPIO1PIN_P39_PUT(v)          ( bFM_GPIO_PDOR3_P9=(v) )

#define GPIO1PIN_P39_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P39_INITIN(v) \
                                                        : GPIO1PIN_P39_INITOUT(v) )

#define GPIO1PIN_P39_INITIN(v)       do{ bFM_GPIO_PCR3_P9=(v).bPullup; \
                                         bFM_GPIO_DDR3_P9=0u; \
                                         bFM_GPIO_PFR3_P9=0u; }while(0)

#define GPIO1PIN_P39_INITOUT(v)      do{ bFM_GPIO_PDOR3_P9=(v).bInitVal; \
                                         bFM_GPIO_DDR3_P9=1u; \
                                         bFM_GPIO_PFR3_P9=0u; }while(0)


/*---- GPIO bit NP39 ----*/
#define GPIO1PIN_NP39_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR3_P9)) )

#define GPIO1PIN_NP39_PUT(v)         ( bFM_GPIO_PDOR3_P9=(uint32_t)(!(v)) )

#define GPIO1PIN_NP39_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP39_INITIN(v) \
                                                        : GPIO1PIN_NP39_INITOUT(v) )

#define GPIO1PIN_NP39_INITIN(v)      do{ bFM_GPIO_PCR3_P9=(v).bPullup; \
                                         bFM_GPIO_DDR3_P9=0u; \
                                         bFM_GPIO_PFR3_P9=0u; }while(0)

#define GPIO1PIN_NP39_INITOUT(v)     do{ bFM_GPIO_PDOR3_P9=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR3_P9=1u; \
                                         bFM_GPIO_PFR3_P9=0u; }while(0)


/*---- GPIO bit P3A ----*/

#define GPIO1PIN_P3A_GET             ( bFM_GPIO_PDIR3_PA )

#define GPIO1PIN_P3A_PUT(v)          ( bFM_GPIO_PDOR3_PA=(v) )

#define GPIO1PIN_P3A_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P3A_INITIN(v) \
                                                        : GPIO1PIN_P3A_INITOUT(v) )

#define GPIO1PIN_P3A_INITIN(v)       do{ bFM_GPIO_PCR3_PA=(v).bPullup; \
                                         bFM_GPIO_DDR3_PA=0u; \
                                         bFM_GPIO_PFR3_PA=0u; }while(0)

#define GPIO1PIN_P3A_INITOUT(v)      do{ bFM_GPIO_PDOR3_PA=(v).bInitVal; \
                                         bFM_GPIO_DDR3_PA=1u; \
                                         bFM_GPIO_PFR3_PA=0u; }while(0)


/*---- GPIO bit NP3A ----*/
#define GPIO1PIN_NP3A_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR3_PA)) )

#define GPIO1PIN_NP3A_PUT(v)         ( bFM_GPIO_PDOR3_PA=(uint32_t)(!(v)) )

#define GPIO1PIN_NP3A_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP3A_INITIN(v) \
                                                        : GPIO1PIN_NP3A_INITOUT(v) )

#define GPIO1PIN_NP3A_INITIN(v)      do{ bFM_GPIO_PCR3_PA=(v).bPullup; \
                                         bFM_GPIO_DDR3_PA=0u; \
                                         bFM_GPIO_PFR3_PA=0u; }while(0)

#define GPIO1PIN_NP3A_INITOUT(v)     do{ bFM_GPIO_PDOR3_PA=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR3_PA=1u; \
                                         bFM_GPIO_PFR3_PA=0u; }while(0)


/*---- GPIO bit P3B ----*/

#define GPIO1PIN_P3B_GET             ( bFM_GPIO_PDIR3_PB )

#define GPIO1PIN_P3B_PUT(v)          ( bFM_GPIO_PDOR3_PB=(v) )

#define GPIO1PIN_P3B_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P3B_INITIN(v) \
                                                        : GPIO1PIN_P3B_INITOUT(v) )

#define GPIO1PIN_P3B_INITIN(v)       do{ bFM_GPIO_PCR3_PB=(v).bPullup; \
                                         bFM_GPIO_DDR3_PB=0u; \
                                         bFM_GPIO_PFR3_PB=0u; }while(0)

#define GPIO1PIN_P3B_INITOUT(v)      do{ bFM_GPIO_PDOR3_PB=(v).bInitVal; \
                                         bFM_GPIO_DDR3_PB=1u; \
                                         bFM_GPIO_PFR3_PB=0u; }while(0)


/*---- GPIO bit NP3B ----*/
#define GPIO1PIN_NP3B_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR3_PB)) )

#define GPIO1PIN_NP3B_PUT(v)         ( bFM_GPIO_PDOR3_PB=(uint32_t)(!(v)) )

#define GPIO1PIN_NP3B_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP3B_INITIN(v) \
                                                        : GPIO1PIN_NP3B_INITOUT(v) )

#define GPIO1PIN_NP3B_INITIN(v)      do{ bFM_GPIO_PCR3_PB=(v).bPullup; \
                                         bFM_GPIO_DDR3_PB=0u; \
                                         bFM_GPIO_PFR3_PB=0u; }while(0)

#define GPIO1PIN_NP3B_INITOUT(v)     do{ bFM_GPIO_PDOR3_PB=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR3_PB=1u; \
                                         bFM_GPIO_PFR3_PB=0u; }while(0)


/*---- GPIO bit P3C ----*/

#define GPIO1PIN_P3C_GET             ( bFM_GPIO_PDIR3_PC )

#define GPIO1PIN_P3C_PUT(v)          ( bFM_GPIO_PDOR3_PC=(v) )

#define GPIO1PIN_P3C_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P3C_INITIN(v) \
                                                        : GPIO1PIN_P3C_INITOUT(v) )

#define GPIO1PIN_P3C_INITIN(v)       do{ bFM_GPIO_PCR3_PC=(v).bPullup; \
                                         bFM_GPIO_DDR3_PC=0u; \
                                         bFM_GPIO_PFR3_PC=0u; }while(0)

#define GPIO1PIN_P3C_INITOUT(v)      do{ bFM_GPIO_PDOR3_PC=(v).bInitVal; \
                                         bFM_GPIO_DDR3_PC=1u; \
                                         bFM_GPIO_PFR3_PC=0u; }while(0)


/*---- GPIO bit NP3C ----*/
#define GPIO1PIN_NP3C_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR3_PC)) )

#define GPIO1PIN_NP3C_PUT(v)         ( bFM_GPIO_PDOR3_PC=(uint32_t)(!(v)) )

#define GPIO1PIN_NP3C_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP3C_INITIN(v) \
                                                        : GPIO1PIN_NP3C_INITOUT(v) )

#define GPIO1PIN_NP3C_INITIN(v)      do{ bFM_GPIO_PCR3_PC=(v).bPullup; \
                                         bFM_GPIO_DDR3_PC=0u; \
                                         bFM_GPIO_PFR3_PC=0u; }while(0)

#define GPIO1PIN_NP3C_INITOUT(v)     do{ bFM_GPIO_PDOR3_PC=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR3_PC=1u; \
                                         bFM_GPIO_PFR3_PC=0u; }while(0)


/*---- GPIO bit P3D ----*/

#define GPIO1PIN_P3D_GET             ( bFM_GPIO_PDIR3_PD )

#define GPIO1PIN_P3D_PUT(v)          ( bFM_GPIO_PDOR3_PD=(v) )

#define GPIO1PIN_P3D_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P3D_INITIN(v) \
                                                        : GPIO1PIN_P3D_INITOUT(v) )

#define GPIO1PIN_P3D_INITIN(v)       do{ bFM_GPIO_PCR3_PD=(v).bPullup; \
                                         bFM_GPIO_DDR3_PD=0u; \
                                         bFM_GPIO_PFR3_PD=0u; }while(0)

#define GPIO1PIN_P3D_INITOUT(v)      do{ bFM_GPIO_PDOR3_PD=(v).bInitVal; \
                                         bFM_GPIO_DDR3_PD=1u; \
                                         bFM_GPIO_PFR3_PD=0u; }while(0)


/*---- GPIO bit NP3D ----*/
#define GPIO1PIN_NP3D_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR3_PD)) )

#define GPIO1PIN_NP3D_PUT(v)         ( bFM_GPIO_PDOR3_PD=(uint32_t)(!(v)) )

#define GPIO1PIN_NP3D_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP3D_INITIN(v) \
                                                        : GPIO1PIN_NP3D_INITOUT(v) )

#define GPIO1PIN_NP3D_INITIN(v)      do{ bFM_GPIO_PCR3_PD=(v).bPullup; \
                                         bFM_GPIO_DDR3_PD=0u; \
                                         bFM_GPIO_PFR3_PD=0u; }while(0)

#define GPIO1PIN_NP3D_INITOUT(v)     do{ bFM_GPIO_PDOR3_PD=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR3_PD=1u; \
                                         bFM_GPIO_PFR3_PD=0u; }while(0)


/*---- GPIO bit P3E ----*/

#define GPIO1PIN_P3E_GET             ( bFM_GPIO_PDIR3_PE )

#define GPIO1PIN_P3E_PUT(v)          ( bFM_GPIO_PDOR3_PE=(v) )

#define GPIO1PIN_P3E_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P3E_INITIN(v) \
                                                        : GPIO1PIN_P3E_INITOUT(v) )

#define GPIO1PIN_P3E_INITIN(v)       do{ bFM_GPIO_PCR3_PE=(v).bPullup; \
                                         bFM_GPIO_DDR3_PE=0u; \
                                         bFM_GPIO_PFR3_PE=0u; }while(0)

#define GPIO1PIN_P3E_INITOUT(v)      do{ bFM_GPIO_PDOR3_PE=(v).bInitVal; \
                                         bFM_GPIO_DDR3_PE=1u; \
                                         bFM_GPIO_PFR3_PE=0u; }while(0)


/*---- GPIO bit NP3E ----*/
#define GPIO1PIN_NP3E_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR3_PE)) )

#define GPIO1PIN_NP3E_PUT(v)         ( bFM_GPIO_PDOR3_PE=(uint32_t)(!(v)) )

#define GPIO1PIN_NP3E_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP3E_INITIN(v) \
                                                        : GPIO1PIN_NP3E_INITOUT(v) )

#define GPIO1PIN_NP3E_INITIN(v)      do{ bFM_GPIO_PCR3_PE=(v).bPullup; \
                                         bFM_GPIO_DDR3_PE=0u; \
                                         bFM_GPIO_PFR3_PE=0u; }while(0)

#define GPIO1PIN_NP3E_INITOUT(v)     do{ bFM_GPIO_PDOR3_PE=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR3_PE=1u; \
                                         bFM_GPIO_PFR3_PE=0u; }while(0)


/*---- GPIO bit P3F ----*/

#define GPIO1PIN_P3F_GET             ( bFM_GPIO_PDIR3_PF )

#define GPIO1PIN_P3F_PUT(v)          ( bFM_GPIO_PDOR3_PF=(v) )

#define GPIO1PIN_P3F_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P3F_INITIN(v) \
                                                        : GPIO1PIN_P3F_INITOUT(v) )

#define GPIO1PIN_P3F_INITIN(v)       do{ bFM_GPIO_PCR3_PF=(v).bPullup; \
                                         bFM_GPIO_DDR3_PF=0u; \
                                         bFM_GPIO_PFR3_PF=0u; }while(0)

#define GPIO1PIN_P3F_INITOUT(v)      do{ bFM_GPIO_PDOR3_PF=(v).bInitVal; \
                                         bFM_GPIO_DDR3_PF=1u; \
                                         bFM_GPIO_PFR3_PF=0u; }while(0)


/*---- GPIO bit NP3F ----*/
#define GPIO1PIN_NP3F_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR3_PF)) )

#define GPIO1PIN_NP3F_PUT(v)         ( bFM_GPIO_PDOR3_PF=(uint32_t)(!(v)) )

#define GPIO1PIN_NP3F_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP3F_INITIN(v) \
                                                        : GPIO1PIN_NP3F_INITOUT(v) )

#define GPIO1PIN_NP3F_INITIN(v)      do{ bFM_GPIO_PCR3_PF=(v).bPullup; \
                                         bFM_GPIO_DDR3_PF=0u; \
                                         bFM_GPIO_PFR3_PF=0u; }while(0)

#define GPIO1PIN_NP3F_INITOUT(v)     do{ bFM_GPIO_PDOR3_PF=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR3_PF=1u; \
                                         bFM_GPIO_PFR3_PF=0u; }while(0)


/*---- GPIO bit P40 ----*/

#define GPIO1PIN_P40_GET             ( bFM_GPIO_PDIR4_P0 )

#define GPIO1PIN_P40_PUT(v)          ( bFM_GPIO_PDOR4_P0=(v) )

#define GPIO1PIN_P40_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P40_INITIN(v) \
                                                        : GPIO1PIN_P40_INITOUT(v) )

#define GPIO1PIN_P40_INITIN(v)       do{ bFM_GPIO_PCR4_P0=(v).bPullup; \
                                         bFM_GPIO_DDR4_P0=0u; \
                                         bFM_GPIO_PFR4_P0=0u; }while(0)

#define GPIO1PIN_P40_INITOUT(v)      do{ bFM_GPIO_PDOR4_P0=(v).bInitVal; \
                                         bFM_GPIO_DDR4_P0=1u; \
                                         bFM_GPIO_PFR4_P0=0u; }while(0)


/*---- GPIO bit NP40 ----*/
#define GPIO1PIN_NP40_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR4_P0)) )

#define GPIO1PIN_NP40_PUT(v)         ( bFM_GPIO_PDOR4_P0=(uint32_t)(!(v)) )

#define GPIO1PIN_NP40_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP40_INITIN(v) \
                                                        : GPIO1PIN_NP40_INITOUT(v) )

#define GPIO1PIN_NP40_INITIN(v)      do{ bFM_GPIO_PCR4_P0=(v).bPullup; \
                                         bFM_GPIO_DDR4_P0=0u; \
                                         bFM_GPIO_PFR4_P0=0u; }while(0)

#define GPIO1PIN_NP40_INITOUT(v)     do{ bFM_GPIO_PDOR4_P0=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR4_P0=1u; \
                                         bFM_GPIO_PFR4_P0=0u; }while(0)


/*---- GPIO bit P41 ----*/

#define GPIO1PIN_P41_GET             ( bFM_GPIO_PDIR4_P1 )

#define GPIO1PIN_P41_PUT(v)          ( bFM_GPIO_PDOR4_P1=(v) )

#define GPIO1PIN_P41_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P41_INITIN(v) \
                                                        : GPIO1PIN_P41_INITOUT(v) )

#define GPIO1PIN_P41_INITIN(v)       do{ bFM_GPIO_PCR4_P1=(v).bPullup; \
                                         bFM_GPIO_DDR4_P1=0u; \
                                         bFM_GPIO_PFR4_P1=0u; }while(0)

#define GPIO1PIN_P41_INITOUT(v)      do{ bFM_GPIO_PDOR4_P1=(v).bInitVal; \
                                         bFM_GPIO_DDR4_P1=1u; \
                                         bFM_GPIO_PFR4_P1=0u; }while(0)


/*---- GPIO bit NP41 ----*/
#define GPIO1PIN_NP41_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR4_P1)) )

#define GPIO1PIN_NP41_PUT(v)         ( bFM_GPIO_PDOR4_P1=(uint32_t)(!(v)) )

#define GPIO1PIN_NP41_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP41_INITIN(v) \
                                                        : GPIO1PIN_NP41_INITOUT(v) )

#define GPIO1PIN_NP41_INITIN(v)      do{ bFM_GPIO_PCR4_P1=(v).bPullup; \
                                         bFM_GPIO_DDR4_P1=0u; \
                                         bFM_GPIO_PFR4_P1=0u; }while(0)

#define GPIO1PIN_NP41_INITOUT(v)     do{ bFM_GPIO_PDOR4_P1=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR4_P1=1u; \
                                         bFM_GPIO_PFR4_P1=0u; }while(0)


/*---- GPIO bit P42 ----*/

#define GPIO1PIN_P42_GET             ( bFM_GPIO_PDIR4_P2 )

#define GPIO1PIN_P42_PUT(v)          ( bFM_GPIO_PDOR4_P2=(v) )

#define GPIO1PIN_P42_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P42_INITIN(v) \
                                                        : GPIO1PIN_P42_INITOUT(v) )

#define GPIO1PIN_P42_INITIN(v)       do{ bFM_GPIO_PCR4_P2=(v).bPullup; \
                                         bFM_GPIO_DDR4_P2=0u; \
                                         bFM_GPIO_PFR4_P2=0u; }while(0)

#define GPIO1PIN_P42_INITOUT(v)      do{ bFM_GPIO_PDOR4_P2=(v).bInitVal; \
                                         bFM_GPIO_DDR4_P2=1u; \
                                         bFM_GPIO_PFR4_P2=0u; }while(0)


/*---- GPIO bit NP42 ----*/
#define GPIO1PIN_NP42_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR4_P2)) )

#define GPIO1PIN_NP42_PUT(v)         ( bFM_GPIO_PDOR4_P2=(uint32_t)(!(v)) )

#define GPIO1PIN_NP42_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP42_INITIN(v) \
                                                        : GPIO1PIN_NP42_INITOUT(v) )

#define GPIO1PIN_NP42_INITIN(v)      do{ bFM_GPIO_PCR4_P2=(v).bPullup; \
                                         bFM_GPIO_DDR4_P2=0u; \
                                         bFM_GPIO_PFR4_P2=0u; }while(0)

#define GPIO1PIN_NP42_INITOUT(v)     do{ bFM_GPIO_PDOR4_P2=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR4_P2=1u; \
                                         bFM_GPIO_PFR4_P2=0u; }while(0)


/*---- GPIO bit P43 ----*/

#define GPIO1PIN_P43_GET             ( bFM_GPIO_PDIR4_P3 )

#define GPIO1PIN_P43_PUT(v)          ( bFM_GPIO_PDOR4_P3=(v) )

#define GPIO1PIN_P43_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P43_INITIN(v) \
                                                        : GPIO1PIN_P43_INITOUT(v) )

#define GPIO1PIN_P43_INITIN(v)       do{ bFM_GPIO_PCR4_P3=(v).bPullup; \
                                         bFM_GPIO_DDR4_P3=0u; \
                                         bFM_GPIO_PFR4_P3=0u; }while(0)

#define GPIO1PIN_P43_INITOUT(v)      do{ bFM_GPIO_PDOR4_P3=(v).bInitVal; \
                                         bFM_GPIO_DDR4_P3=1u; \
                                         bFM_GPIO_PFR4_P3=0u; }while(0)


/*---- GPIO bit NP43 ----*/
#define GPIO1PIN_NP43_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR4_P3)) )

#define GPIO1PIN_NP43_PUT(v)         ( bFM_GPIO_PDOR4_P3=(uint32_t)(!(v)) )

#define GPIO1PIN_NP43_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP43_INITIN(v) \
                                                        : GPIO1PIN_NP43_INITOUT(v) )

#define GPIO1PIN_NP43_INITIN(v)      do{ bFM_GPIO_PCR4_P3=(v).bPullup; \
                                         bFM_GPIO_DDR4_P3=0u; \
                                         bFM_GPIO_PFR4_P3=0u; }while(0)

#define GPIO1PIN_NP43_INITOUT(v)     do{ bFM_GPIO_PDOR4_P3=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR4_P3=1u; \
                                         bFM_GPIO_PFR4_P3=0u; }while(0)


/*---- GPIO bit P44 ----*/

#define GPIO1PIN_P44_GET             ( bFM_GPIO_PDIR4_P4 )

#define GPIO1PIN_P44_PUT(v)          ( bFM_GPIO_PDOR4_P4=(v) )

#define GPIO1PIN_P44_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P44_INITIN(v) \
                                                        : GPIO1PIN_P44_INITOUT(v) )

#define GPIO1PIN_P44_INITIN(v)       do{ bFM_GPIO_PCR4_P4=(v).bPullup; \
                                         bFM_GPIO_DDR4_P4=0u; \
                                         bFM_GPIO_PFR4_P4=0u; }while(0)

#define GPIO1PIN_P44_INITOUT(v)      do{ bFM_GPIO_PDOR4_P4=(v).bInitVal; \
                                         bFM_GPIO_DDR4_P4=1u; \
                                         bFM_GPIO_PFR4_P4=0u; }while(0)


/*---- GPIO bit NP44 ----*/
#define GPIO1PIN_NP44_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR4_P4)) )

#define GPIO1PIN_NP44_PUT(v)         ( bFM_GPIO_PDOR4_P4=(uint32_t)(!(v)) )

#define GPIO1PIN_NP44_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP44_INITIN(v) \
                                                        : GPIO1PIN_NP44_INITOUT(v) )

#define GPIO1PIN_NP44_INITIN(v)      do{ bFM_GPIO_PCR4_P4=(v).bPullup; \
                                         bFM_GPIO_DDR4_P4=0u; \
                                         bFM_GPIO_PFR4_P4=0u; }while(0)

#define GPIO1PIN_NP44_INITOUT(v)     do{ bFM_GPIO_PDOR4_P4=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR4_P4=1u; \
                                         bFM_GPIO_PFR4_P4=0u; }while(0)


/*---- GPIO bit P45 ----*/

#define GPIO1PIN_P45_GET             ( bFM_GPIO_PDIR4_P5 )

#define GPIO1PIN_P45_PUT(v)          ( bFM_GPIO_PDOR4_P5=(v) )

#define GPIO1PIN_P45_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P45_INITIN(v) \
                                                        : GPIO1PIN_P45_INITOUT(v) )

#define GPIO1PIN_P45_INITIN(v)       do{ bFM_GPIO_PCR4_P5=(v).bPullup; \
                                         bFM_GPIO_DDR4_P5=0u; \
                                         bFM_GPIO_PFR4_P5=0u; }while(0)

#define GPIO1PIN_P45_INITOUT(v)      do{ bFM_GPIO_PDOR4_P5=(v).bInitVal; \
                                         bFM_GPIO_DDR4_P5=1u; \
                                         bFM_GPIO_PFR4_P5=0u; }while(0)


/*---- GPIO bit NP45 ----*/
#define GPIO1PIN_NP45_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR4_P5)) )

#define GPIO1PIN_NP45_PUT(v)         ( bFM_GPIO_PDOR4_P5=(uint32_t)(!(v)) )

#define GPIO1PIN_NP45_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP45_INITIN(v) \
                                                        : GPIO1PIN_NP45_INITOUT(v) )

#define GPIO1PIN_NP45_INITIN(v)      do{ bFM_GPIO_PCR4_P5=(v).bPullup; \
                                         bFM_GPIO_DDR4_P5=0u; \
                                         bFM_GPIO_PFR4_P5=0u; }while(0)

#define GPIO1PIN_NP45_INITOUT(v)     do{ bFM_GPIO_PDOR4_P5=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR4_P5=1u; \
                                         bFM_GPIO_PFR4_P5=0u; }while(0)


/*---- GPIO bit P46 ----*/

#define GPIO1PIN_P46_GET             ( bFM_GPIO_PDIR4_P6 )

#define GPIO1PIN_P46_PUT(v)          ( bFM_GPIO_PDOR4_P6=(v) )

#define GPIO1PIN_P46_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P46_INITIN(v) \
                                                        : GPIO1PIN_P46_INITOUT(v) )

#define GPIO1PIN_P46_INITIN(v)       do{ bFM_GPIO_PCR4_P6=(v).bPullup; \
                                         bFM_GPIO_DDR4_P6=0u; \
                                         bFM_GPIO_SPSR_SUBXC0=0u; \
                                         bFM_GPIO_SPSR_SUBXC1=0u; \
                                         bFM_GPIO_PFR4_P6=0u; }while(0)

#define GPIO1PIN_P46_INITOUT(v)      do{ bFM_GPIO_PDOR4_P6=(v).bInitVal; \
                                         bFM_GPIO_DDR4_P6=1u; \
                                         bFM_GPIO_SPSR_SUBXC0=0u; \
                                         bFM_GPIO_SPSR_SUBXC1=0u; \
                                         bFM_GPIO_PFR4_P6=0u; }while(0)


/*---- GPIO bit NP46 ----*/
#define GPIO1PIN_NP46_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR4_P6)) )

#define GPIO1PIN_NP46_PUT(v)         ( bFM_GPIO_PDOR4_P6=(uint32_t)(!(v)) )

#define GPIO1PIN_NP46_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP46_INITIN(v) \
                                                        : GPIO1PIN_NP46_INITOUT(v) )

#define GPIO1PIN_NP46_INITIN(v)      do{ bFM_GPIO_PCR4_P6=(v).bPullup; \
                                         bFM_GPIO_DDR4_P6=0u; \
                                         bFM_GPIO_SPSR_SUBXC0=0u; \
                                         bFM_GPIO_SPSR_SUBXC1=0u; \
                                         bFM_GPIO_PFR4_P6=0u; }while(0)

#define GPIO1PIN_NP46_INITOUT(v)     do{ bFM_GPIO_PDOR4_P6=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR4_P6=1u; \
                                         bFM_GPIO_SPSR_SUBXC0=0u; \
                                         bFM_GPIO_SPSR_SUBXC1=0u; \
                                         bFM_GPIO_PFR4_P6=0u; }while(0)


/*---- GPIO bit P47 ----*/

#define GPIO1PIN_P47_GET             ( bFM_GPIO_PDIR4_P7 )

#define GPIO1PIN_P47_PUT(v)          ( bFM_GPIO_PDOR4_P7=(v) )

#define GPIO1PIN_P47_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P47_INITIN(v) \
                                                        : GPIO1PIN_P47_INITOUT(v) )

#define GPIO1PIN_P47_INITIN(v)       do{ bFM_GPIO_PCR4_P7=(v).bPullup; \
                                         bFM_GPIO_DDR4_P7=0u; \
                                         bFM_GPIO_SPSR_SUBXC0=0u; \
                                         bFM_GPIO_SPSR_SUBXC1=0u; \
                                         bFM_GPIO_PFR4_P7=0u; }while(0)

#define GPIO1PIN_P47_INITOUT(v)      do{ bFM_GPIO_PDOR4_P7=(v).bInitVal; \
                                         bFM_GPIO_DDR4_P7=1u; \
                                         bFM_GPIO_SPSR_SUBXC0=0u; \
                                         bFM_GPIO_SPSR_SUBXC1=0u; \
                                         bFM_GPIO_PFR4_P7=0u; }while(0)


/*---- GPIO bit NP47 ----*/
#define GPIO1PIN_NP47_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR4_P7)) )

#define GPIO1PIN_NP47_PUT(v)         ( bFM_GPIO_PDOR4_P7=(uint32_t)(!(v)) )

#define GPIO1PIN_NP47_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP47_INITIN(v) \
                                                        : GPIO1PIN_NP47_INITOUT(v) )

#define GPIO1PIN_NP47_INITIN(v)      do{ bFM_GPIO_PCR4_P7=(v).bPullup; \
                                         bFM_GPIO_DDR4_P7=0u; \
                                         bFM_GPIO_SPSR_SUBXC0=0u; \
                                         bFM_GPIO_SPSR_SUBXC1=0u; \
                                         bFM_GPIO_PFR4_P7=0u; }while(0)

#define GPIO1PIN_NP47_INITOUT(v)     do{ bFM_GPIO_PDOR4_P7=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR4_P7=1u; \
                                         bFM_GPIO_SPSR_SUBXC0=0u; \
                                         bFM_GPIO_SPSR_SUBXC1=0u; \
                                         bFM_GPIO_PFR4_P7=0u; }while(0)


/*---- GPIO bit P48 ----*/

#define GPIO1PIN_P48_GET             ( bFM_GPIO_PDIR4_P8 )

#define GPIO1PIN_P48_PUT(v)          ( bFM_GPIO_PDOR4_P8=(v) )

#define GPIO1PIN_P48_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P48_INITIN(v) \
                                                        : GPIO1PIN_P48_INITOUT(v) )

#define GPIO1PIN_P48_INITIN(v)       do{ bFM_GPIO_PCR4_P8=(v).bPullup; \
                                         bFM_GPIO_DDR4_P8=0u; \
                                         bFM_GPIO_PFR4_P8=0u; }while(0)

#define GPIO1PIN_P48_INITOUT(v)      do{ bFM_GPIO_PDOR4_P8=(v).bInitVal; \
                                         bFM_GPIO_DDR4_P8=1u; \
                                         bFM_GPIO_PFR4_P8=0u; }while(0)


/*---- GPIO bit NP48 ----*/
#define GPIO1PIN_NP48_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR4_P8)) )

#define GPIO1PIN_NP48_PUT(v)         ( bFM_GPIO_PDOR4_P8=(uint32_t)(!(v)) )

#define GPIO1PIN_NP48_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP48_INITIN(v) \
                                                        : GPIO1PIN_NP48_INITOUT(v) )

#define GPIO1PIN_NP48_INITIN(v)      do{ bFM_GPIO_PCR4_P8=(v).bPullup; \
                                         bFM_GPIO_DDR4_P8=0u; \
                                         bFM_GPIO_PFR4_P8=0u; }while(0)

#define GPIO1PIN_NP48_INITOUT(v)     do{ bFM_GPIO_PDOR4_P8=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR4_P8=1u; \
                                         bFM_GPIO_PFR4_P8=0u; }while(0)


/*---- GPIO bit P49 ----*/

#define GPIO1PIN_P49_GET             ( bFM_GPIO_PDIR4_P9 )

#define GPIO1PIN_P49_PUT(v)          ( bFM_GPIO_PDOR4_P9=(v) )

#define GPIO1PIN_P49_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P49_INITIN(v) \
                                                        : GPIO1PIN_P49_INITOUT(v) )

#define GPIO1PIN_P49_INITIN(v)       do{ bFM_GPIO_PCR4_P9=(v).bPullup; \
                                         bFM_GPIO_DDR4_P9=0u; \
                                         bFM_GPIO_PFR4_P9=0u; }while(0)

#define GPIO1PIN_P49_INITOUT(v)      do{ bFM_GPIO_PDOR4_P9=(v).bInitVal; \
                                         bFM_GPIO_DDR4_P9=1u; \
                                         bFM_GPIO_PFR4_P9=0u; }while(0)


/*---- GPIO bit NP49 ----*/
#define GPIO1PIN_NP49_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR4_P9)) )

#define GPIO1PIN_NP49_PUT(v)         ( bFM_GPIO_PDOR4_P9=(uint32_t)(!(v)) )

#define GPIO1PIN_NP49_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP49_INITIN(v) \
                                                        : GPIO1PIN_NP49_INITOUT(v) )

#define GPIO1PIN_NP49_INITIN(v)      do{ bFM_GPIO_PCR4_P9=(v).bPullup; \
                                         bFM_GPIO_DDR4_P9=0u; \
                                         bFM_GPIO_PFR4_P9=0u; }while(0)

#define GPIO1PIN_NP49_INITOUT(v)     do{ bFM_GPIO_PDOR4_P9=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR4_P9=1u; \
                                         bFM_GPIO_PFR4_P9=0u; }while(0)


/*---- GPIO bit P4A ----*/

#define GPIO1PIN_P4A_GET             ( bFM_GPIO_PDIR4_PA )

#define GPIO1PIN_P4A_PUT(v)          ( bFM_GPIO_PDOR4_PA=(v) )

#define GPIO1PIN_P4A_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P4A_INITIN(v) \
                                                        : GPIO1PIN_P4A_INITOUT(v) )

#define GPIO1PIN_P4A_INITIN(v)       do{ bFM_GPIO_PCR4_PA=(v).bPullup; \
                                         bFM_GPIO_DDR4_PA=0u; \
                                         bFM_GPIO_PFR4_PA=0u; }while(0)

#define GPIO1PIN_P4A_INITOUT(v)      do{ bFM_GPIO_PDOR4_PA=(v).bInitVal; \
                                         bFM_GPIO_DDR4_PA=1u; \
                                         bFM_GPIO_PFR4_PA=0u; }while(0)


/*---- GPIO bit NP4A ----*/
#define GPIO1PIN_NP4A_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR4_PA)) )

#define GPIO1PIN_NP4A_PUT(v)         ( bFM_GPIO_PDOR4_PA=(uint32_t)(!(v)) )

#define GPIO1PIN_NP4A_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP4A_INITIN(v) \
                                                        : GPIO1PIN_NP4A_INITOUT(v) )

#define GPIO1PIN_NP4A_INITIN(v)      do{ bFM_GPIO_PCR4_PA=(v).bPullup; \
                                         bFM_GPIO_DDR4_PA=0u; \
                                         bFM_GPIO_PFR4_PA=0u; }while(0)

#define GPIO1PIN_NP4A_INITOUT(v)     do{ bFM_GPIO_PDOR4_PA=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR4_PA=1u; \
                                         bFM_GPIO_PFR4_PA=0u; }while(0)


/*---- GPIO bit P4B ----*/

#define GPIO1PIN_P4B_GET             ( bFM_GPIO_PDIR4_PB )

#define GPIO1PIN_P4B_PUT(v)          ( bFM_GPIO_PDOR4_PB=(v) )

#define GPIO1PIN_P4B_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P4B_INITIN(v) \
                                                        : GPIO1PIN_P4B_INITOUT(v) )

#define GPIO1PIN_P4B_INITIN(v)       do{ bFM_GPIO_PCR4_PB=(v).bPullup; \
                                         bFM_GPIO_DDR4_PB=0u; \
                                         bFM_GPIO_PFR4_PB=0u; }while(0)

#define GPIO1PIN_P4B_INITOUT(v)      do{ bFM_GPIO_PDOR4_PB=(v).bInitVal; \
                                         bFM_GPIO_DDR4_PB=1u; \
                                         bFM_GPIO_PFR4_PB=0u; }while(0)


/*---- GPIO bit NP4B ----*/
#define GPIO1PIN_NP4B_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR4_PB)) )

#define GPIO1PIN_NP4B_PUT(v)         ( bFM_GPIO_PDOR4_PB=(uint32_t)(!(v)) )

#define GPIO1PIN_NP4B_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP4B_INITIN(v) \
                                                        : GPIO1PIN_NP4B_INITOUT(v) )

#define GPIO1PIN_NP4B_INITIN(v)      do{ bFM_GPIO_PCR4_PB=(v).bPullup; \
                                         bFM_GPIO_DDR4_PB=0u; \
                                         bFM_GPIO_PFR4_PB=0u; }while(0)

#define GPIO1PIN_NP4B_INITOUT(v)     do{ bFM_GPIO_PDOR4_PB=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR4_PB=1u; \
                                         bFM_GPIO_PFR4_PB=0u; }while(0)


/*---- GPIO bit P4C ----*/

#define GPIO1PIN_P4C_GET             ( bFM_GPIO_PDIR4_PC )

#define GPIO1PIN_P4C_PUT(v)          ( bFM_GPIO_PDOR4_PC=(v) )

#define GPIO1PIN_P4C_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P4C_INITIN(v) \
                                                        : GPIO1PIN_P4C_INITOUT(v) )

#define GPIO1PIN_P4C_INITIN(v)       do{ bFM_GPIO_PCR4_PC=(v).bPullup; \
                                         bFM_GPIO_DDR4_PC=0u; \
                                         bFM_GPIO_PFR4_PC=0u; }while(0)

#define GPIO1PIN_P4C_INITOUT(v)      do{ bFM_GPIO_PDOR4_PC=(v).bInitVal; \
                                         bFM_GPIO_DDR4_PC=1u; \
                                         bFM_GPIO_PFR4_PC=0u; }while(0)


/*---- GPIO bit NP4C ----*/
#define GPIO1PIN_NP4C_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR4_PC)) )

#define GPIO1PIN_NP4C_PUT(v)         ( bFM_GPIO_PDOR4_PC=(uint32_t)(!(v)) )

#define GPIO1PIN_NP4C_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP4C_INITIN(v) \
                                                        : GPIO1PIN_NP4C_INITOUT(v) )

#define GPIO1PIN_NP4C_INITIN(v)      do{ bFM_GPIO_PCR4_PC=(v).bPullup; \
                                         bFM_GPIO_DDR4_PC=0u; \
                                         bFM_GPIO_PFR4_PC=0u; }while(0)

#define GPIO1PIN_NP4C_INITOUT(v)     do{ bFM_GPIO_PDOR4_PC=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR4_PC=1u; \
                                         bFM_GPIO_PFR4_PC=0u; }while(0)


/*---- GPIO bit P4D ----*/

#define GPIO1PIN_P4D_GET             ( bFM_GPIO_PDIR4_PD )

#define GPIO1PIN_P4D_PUT(v)          ( bFM_GPIO_PDOR4_PD=(v) )

#define GPIO1PIN_P4D_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P4D_INITIN(v) \
                                                        : GPIO1PIN_P4D_INITOUT(v) )

#define GPIO1PIN_P4D_INITIN(v)       do{ bFM_GPIO_PCR4_PD=(v).bPullup; \
                                         bFM_GPIO_DDR4_PD=0u; \
                                         bFM_GPIO_PFR4_PD=0u; }while(0)

#define GPIO1PIN_P4D_INITOUT(v)      do{ bFM_GPIO_PDOR4_PD=(v).bInitVal; \
                                         bFM_GPIO_DDR4_PD=1u; \
                                         bFM_GPIO_PFR4_PD=0u; }while(0)


/*---- GPIO bit NP4D ----*/
#define GPIO1PIN_NP4D_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR4_PD)) )

#define GPIO1PIN_NP4D_PUT(v)         ( bFM_GPIO_PDOR4_PD=(uint32_t)(!(v)) )

#define GPIO1PIN_NP4D_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP4D_INITIN(v) \
                                                        : GPIO1PIN_NP4D_INITOUT(v) )

#define GPIO1PIN_NP4D_INITIN(v)      do{ bFM_GPIO_PCR4_PD=(v).bPullup; \
                                         bFM_GPIO_DDR4_PD=0u; \
                                         bFM_GPIO_PFR4_PD=0u; }while(0)

#define GPIO1PIN_NP4D_INITOUT(v)     do{ bFM_GPIO_PDOR4_PD=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR4_PD=1u; \
                                         bFM_GPIO_PFR4_PD=0u; }while(0)


/*---- GPIO bit P4E ----*/

#define GPIO1PIN_P4E_GET             ( bFM_GPIO_PDIR4_PE )

#define GPIO1PIN_P4E_PUT(v)          ( bFM_GPIO_PDOR4_PE=(v) )

#define GPIO1PIN_P4E_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P4E_INITIN(v) \
                                                        : GPIO1PIN_P4E_INITOUT(v) )

#define GPIO1PIN_P4E_INITIN(v)       do{ bFM_GPIO_PCR4_PE=(v).bPullup; \
                                         bFM_GPIO_DDR4_PE=0u; \
                                         bFM_GPIO_PFR4_PE=0u; }while(0)

#define GPIO1PIN_P4E_INITOUT(v)      do{ bFM_GPIO_PDOR4_PE=(v).bInitVal; \
                                         bFM_GPIO_DDR4_PE=1u; \
                                         bFM_GPIO_PFR4_PE=0u; }while(0)


/*---- GPIO bit NP4E ----*/
#define GPIO1PIN_NP4E_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR4_PE)) )

#define GPIO1PIN_NP4E_PUT(v)         ( bFM_GPIO_PDOR4_PE=(uint32_t)(!(v)) )

#define GPIO1PIN_NP4E_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP4E_INITIN(v) \
                                                        : GPIO1PIN_NP4E_INITOUT(v) )

#define GPIO1PIN_NP4E_INITIN(v)      do{ bFM_GPIO_PCR4_PE=(v).bPullup; \
                                         bFM_GPIO_DDR4_PE=0u; \
                                         bFM_GPIO_PFR4_PE=0u; }while(0)

#define GPIO1PIN_NP4E_INITOUT(v)     do{ bFM_GPIO_PDOR4_PE=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR4_PE=1u; \
                                         bFM_GPIO_PFR4_PE=0u; }while(0)


/*---- GPIO bit P50 ----*/

#define GPIO1PIN_P50_GET             ( bFM_GPIO_PDIR5_P0 )

#define GPIO1PIN_P50_PUT(v)          ( bFM_GPIO_PDOR5_P0=(v) )

#define GPIO1PIN_P50_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P50_INITIN(v) \
                                                        : GPIO1PIN_P50_INITOUT(v) )

#define GPIO1PIN_P50_INITIN(v)       do{ bFM_GPIO_PCR5_P0=(v).bPullup; \
                                         bFM_GPIO_DDR5_P0=0u; \
                                         bFM_GPIO_PFR5_P0=0u; }while(0)

#define GPIO1PIN_P50_INITOUT(v)      do{ bFM_GPIO_PDOR5_P0=(v).bInitVal; \
                                         bFM_GPIO_DDR5_P0=1u; \
                                         bFM_GPIO_PFR5_P0=0u; }while(0)


/*---- GPIO bit NP50 ----*/
#define GPIO1PIN_NP50_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR5_P0)) )

#define GPIO1PIN_NP50_PUT(v)         ( bFM_GPIO_PDOR5_P0=(uint32_t)(!(v)) )

#define GPIO1PIN_NP50_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP50_INITIN(v) \
                                                        : GPIO1PIN_NP50_INITOUT(v) )

#define GPIO1PIN_NP50_INITIN(v)      do{ bFM_GPIO_PCR5_P0=(v).bPullup; \
                                         bFM_GPIO_DDR5_P0=0u; \
                                         bFM_GPIO_PFR5_P0=0u; }while(0)

#define GPIO1PIN_NP50_INITOUT(v)     do{ bFM_GPIO_PDOR5_P0=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR5_P0=1u; \
                                         bFM_GPIO_PFR5_P0=0u; }while(0)


/*---- GPIO bit P51 ----*/

#define GPIO1PIN_P51_GET             ( bFM_GPIO_PDIR5_P1 )

#define GPIO1PIN_P51_PUT(v)          ( bFM_GPIO_PDOR5_P1=(v) )

#define GPIO1PIN_P51_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P51_INITIN(v) \
                                                        : GPIO1PIN_P51_INITOUT(v) )

#define GPIO1PIN_P51_INITIN(v)       do{ bFM_GPIO_PCR5_P1=(v).bPullup; \
                                         bFM_GPIO_DDR5_P1=0u; \
                                         bFM_GPIO_PFR5_P1=0u; }while(0)

#define GPIO1PIN_P51_INITOUT(v)      do{ bFM_GPIO_PDOR5_P1=(v).bInitVal; \
                                         bFM_GPIO_DDR5_P1=1u; \
                                         bFM_GPIO_PFR5_P1=0u; }while(0)


/*---- GPIO bit NP51 ----*/
#define GPIO1PIN_NP51_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR5_P1)) )

#define GPIO1PIN_NP51_PUT(v)         ( bFM_GPIO_PDOR5_P1=(uint32_t)(!(v)) )

#define GPIO1PIN_NP51_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP51_INITIN(v) \
                                                        : GPIO1PIN_NP51_INITOUT(v) )

#define GPIO1PIN_NP51_INITIN(v)      do{ bFM_GPIO_PCR5_P1=(v).bPullup; \
                                         bFM_GPIO_DDR5_P1=0u; \
                                         bFM_GPIO_PFR5_P1=0u; }while(0)

#define GPIO1PIN_NP51_INITOUT(v)     do{ bFM_GPIO_PDOR5_P1=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR5_P1=1u; \
                                         bFM_GPIO_PFR5_P1=0u; }while(0)


/*---- GPIO bit P52 ----*/

#define GPIO1PIN_P52_GET             ( bFM_GPIO_PDIR5_P2 )

#define GPIO1PIN_P52_PUT(v)          ( bFM_GPIO_PDOR5_P2=(v) )

#define GPIO1PIN_P52_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P52_INITIN(v) \
                                                        : GPIO1PIN_P52_INITOUT(v) )

#define GPIO1PIN_P52_INITIN(v)       do{ bFM_GPIO_PCR5_P2=(v).bPullup; \
                                         bFM_GPIO_DDR5_P2=0u; \
                                         bFM_GPIO_PFR5_P2=0u; }while(0)

#define GPIO1PIN_P52_INITOUT(v)      do{ bFM_GPIO_PDOR5_P2=(v).bInitVal; \
                                         bFM_GPIO_DDR5_P2=1u; \
                                         bFM_GPIO_PFR5_P2=0u; }while(0)


/*---- GPIO bit NP52 ----*/
#define GPIO1PIN_NP52_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR5_P2)) )

#define GPIO1PIN_NP52_PUT(v)         ( bFM_GPIO_PDOR5_P2=(uint32_t)(!(v)) )

#define GPIO1PIN_NP52_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP52_INITIN(v) \
                                                        : GPIO1PIN_NP52_INITOUT(v) )

#define GPIO1PIN_NP52_INITIN(v)      do{ bFM_GPIO_PCR5_P2=(v).bPullup; \
                                         bFM_GPIO_DDR5_P2=0u; \
                                         bFM_GPIO_PFR5_P2=0u; }while(0)

#define GPIO1PIN_NP52_INITOUT(v)     do{ bFM_GPIO_PDOR5_P2=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR5_P2=1u; \
                                         bFM_GPIO_PFR5_P2=0u; }while(0)


/*---- GPIO bit P53 ----*/

#define GPIO1PIN_P53_GET             ( bFM_GPIO_PDIR5_P3 )

#define GPIO1PIN_P53_PUT(v)          ( bFM_GPIO_PDOR5_P3=(v) )

#define GPIO1PIN_P53_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P53_INITIN(v) \
                                                        : GPIO1PIN_P53_INITOUT(v) )

#define GPIO1PIN_P53_INITIN(v)       do{ bFM_GPIO_PCR5_P3=(v).bPullup; \
                                         bFM_GPIO_DDR5_P3=0u; \
                                         bFM_GPIO_PFR5_P3=0u; }while(0)

#define GPIO1PIN_P53_INITOUT(v)      do{ bFM_GPIO_PDOR5_P3=(v).bInitVal; \
                                         bFM_GPIO_DDR5_P3=1u; \
                                         bFM_GPIO_PFR5_P3=0u; }while(0)


/*---- GPIO bit NP53 ----*/
#define GPIO1PIN_NP53_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR5_P3)) )

#define GPIO1PIN_NP53_PUT(v)         ( bFM_GPIO_PDOR5_P3=(uint32_t)(!(v)) )

#define GPIO1PIN_NP53_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP53_INITIN(v) \
                                                        : GPIO1PIN_NP53_INITOUT(v) )

#define GPIO1PIN_NP53_INITIN(v)      do{ bFM_GPIO_PCR5_P3=(v).bPullup; \
                                         bFM_GPIO_DDR5_P3=0u; \
                                         bFM_GPIO_PFR5_P3=0u; }while(0)

#define GPIO1PIN_NP53_INITOUT(v)     do{ bFM_GPIO_PDOR5_P3=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR5_P3=1u; \
                                         bFM_GPIO_PFR5_P3=0u; }while(0)


/*---- GPIO bit P54 ----*/

#define GPIO1PIN_P54_GET             ( bFM_GPIO_PDIR5_P4 )

#define GPIO1PIN_P54_PUT(v)          ( bFM_GPIO_PDOR5_P4=(v) )

#define GPIO1PIN_P54_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P54_INITIN(v) \
                                                        : GPIO1PIN_P54_INITOUT(v) )

#define GPIO1PIN_P54_INITIN(v)       do{ bFM_GPIO_PCR5_P4=(v).bPullup; \
                                         bFM_GPIO_DDR5_P4=0u; \
                                         bFM_GPIO_PFR5_P4=0u; }while(0)

#define GPIO1PIN_P54_INITOUT(v)      do{ bFM_GPIO_PDOR5_P4=(v).bInitVal; \
                                         bFM_GPIO_DDR5_P4=1u; \
                                         bFM_GPIO_PFR5_P4=0u; }while(0)


/*---- GPIO bit NP54 ----*/
#define GPIO1PIN_NP54_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR5_P4)) )

#define GPIO1PIN_NP54_PUT(v)         ( bFM_GPIO_PDOR5_P4=(uint32_t)(!(v)) )

#define GPIO1PIN_NP54_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP54_INITIN(v) \
                                                        : GPIO1PIN_NP54_INITOUT(v) )

#define GPIO1PIN_NP54_INITIN(v)      do{ bFM_GPIO_PCR5_P4=(v).bPullup; \
                                         bFM_GPIO_DDR5_P4=0u; \
                                         bFM_GPIO_PFR5_P4=0u; }while(0)

#define GPIO1PIN_NP54_INITOUT(v)     do{ bFM_GPIO_PDOR5_P4=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR5_P4=1u; \
                                         bFM_GPIO_PFR5_P4=0u; }while(0)


/*---- GPIO bit P55 ----*/

#define GPIO1PIN_P55_GET             ( bFM_GPIO_PDIR5_P5 )

#define GPIO1PIN_P55_PUT(v)          ( bFM_GPIO_PDOR5_P5=(v) )

#define GPIO1PIN_P55_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P55_INITIN(v) \
                                                        : GPIO1PIN_P55_INITOUT(v) )

#define GPIO1PIN_P55_INITIN(v)       do{ bFM_GPIO_PCR5_P5=(v).bPullup; \
                                         bFM_GPIO_DDR5_P5=0u; \
                                         bFM_GPIO_PFR5_P5=0u; }while(0)

#define GPIO1PIN_P55_INITOUT(v)      do{ bFM_GPIO_PDOR5_P5=(v).bInitVal; \
                                         bFM_GPIO_DDR5_P5=1u; \
                                         bFM_GPIO_PFR5_P5=0u; }while(0)


/*---- GPIO bit NP55 ----*/
#define GPIO1PIN_NP55_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR5_P5)) )

#define GPIO1PIN_NP55_PUT(v)         ( bFM_GPIO_PDOR5_P5=(uint32_t)(!(v)) )

#define GPIO1PIN_NP55_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP55_INITIN(v) \
                                                        : GPIO1PIN_NP55_INITOUT(v) )

#define GPIO1PIN_NP55_INITIN(v)      do{ bFM_GPIO_PCR5_P5=(v).bPullup; \
                                         bFM_GPIO_DDR5_P5=0u; \
                                         bFM_GPIO_PFR5_P5=0u; }while(0)

#define GPIO1PIN_NP55_INITOUT(v)     do{ bFM_GPIO_PDOR5_P5=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR5_P5=1u; \
                                         bFM_GPIO_PFR5_P5=0u; }while(0)


/*---- GPIO bit P56 ----*/

#define GPIO1PIN_P56_GET             ( bFM_GPIO_PDIR5_P6 )

#define GPIO1PIN_P56_PUT(v)          ( bFM_GPIO_PDOR5_P6=(v) )

#define GPIO1PIN_P56_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P56_INITIN(v) \
                                                        : GPIO1PIN_P56_INITOUT(v) )

#define GPIO1PIN_P56_INITIN(v)       do{ bFM_GPIO_PCR5_P6=(v).bPullup; \
                                         bFM_GPIO_DDR5_P6=0u; \
                                         bFM_GPIO_PFR5_P6=0u; }while(0)

#define GPIO1PIN_P56_INITOUT(v)      do{ bFM_GPIO_PDOR5_P6=(v).bInitVal; \
                                         bFM_GPIO_DDR5_P6=1u; \
                                         bFM_GPIO_PFR5_P6=0u; }while(0)


/*---- GPIO bit NP56 ----*/
#define GPIO1PIN_NP56_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR5_P6)) )

#define GPIO1PIN_NP56_PUT(v)         ( bFM_GPIO_PDOR5_P6=(uint32_t)(!(v)) )

#define GPIO1PIN_NP56_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP56_INITIN(v) \
                                                        : GPIO1PIN_NP56_INITOUT(v) )

#define GPIO1PIN_NP56_INITIN(v)      do{ bFM_GPIO_PCR5_P6=(v).bPullup; \
                                         bFM_GPIO_DDR5_P6=0u; \
                                         bFM_GPIO_PFR5_P6=0u; }while(0)

#define GPIO1PIN_NP56_INITOUT(v)     do{ bFM_GPIO_PDOR5_P6=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR5_P6=1u; \
                                         bFM_GPIO_PFR5_P6=0u; }while(0)


/*---- GPIO bit P57 ----*/

#define GPIO1PIN_P57_GET             ( bFM_GPIO_PDIR5_P7 )

#define GPIO1PIN_P57_PUT(v)          ( bFM_GPIO_PDOR5_P7=(v) )

#define GPIO1PIN_P57_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P57_INITIN(v) \
                                                        : GPIO1PIN_P57_INITOUT(v) )

#define GPIO1PIN_P57_INITIN(v)       do{ bFM_GPIO_PCR5_P7=(v).bPullup; \
                                         bFM_GPIO_DDR5_P7=0u; \
                                         bFM_GPIO_PFR5_P7=0u; }while(0)

#define GPIO1PIN_P57_INITOUT(v)      do{ bFM_GPIO_PDOR5_P7=(v).bInitVal; \
                                         bFM_GPIO_DDR5_P7=1u; \
                                         bFM_GPIO_PFR5_P7=0u; }while(0)


/*---- GPIO bit NP57 ----*/
#define GPIO1PIN_NP57_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR5_P7)) )

#define GPIO1PIN_NP57_PUT(v)         ( bFM_GPIO_PDOR5_P7=(uint32_t)(!(v)) )

#define GPIO1PIN_NP57_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP57_INITIN(v) \
                                                        : GPIO1PIN_NP57_INITOUT(v) )

#define GPIO1PIN_NP57_INITIN(v)      do{ bFM_GPIO_PCR5_P7=(v).bPullup; \
                                         bFM_GPIO_DDR5_P7=0u; \
                                         bFM_GPIO_PFR5_P7=0u; }while(0)

#define GPIO1PIN_NP57_INITOUT(v)     do{ bFM_GPIO_PDOR5_P7=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR5_P7=1u; \
                                         bFM_GPIO_PFR5_P7=0u; }while(0)


/*---- GPIO bit P58 ----*/

#define GPIO1PIN_P58_GET             ( bFM_GPIO_PDIR5_P8 )

#define GPIO1PIN_P58_PUT(v)          ( bFM_GPIO_PDOR5_P8=(v) )

#define GPIO1PIN_P58_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P58_INITIN(v) \
                                                        : GPIO1PIN_P58_INITOUT(v) )

#define GPIO1PIN_P58_INITIN(v)       do{ bFM_GPIO_PCR5_P8=(v).bPullup; \
                                         bFM_GPIO_DDR5_P8=0u; \
                                         bFM_GPIO_PFR5_P8=0u; }while(0)

#define GPIO1PIN_P58_INITOUT(v)      do{ bFM_GPIO_PDOR5_P8=(v).bInitVal; \
                                         bFM_GPIO_DDR5_P8=1u; \
                                         bFM_GPIO_PFR5_P8=0u; }while(0)


/*---- GPIO bit NP58 ----*/
#define GPIO1PIN_NP58_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR5_P8)) )

#define GPIO1PIN_NP58_PUT(v)         ( bFM_GPIO_PDOR5_P8=(uint32_t)(!(v)) )

#define GPIO1PIN_NP58_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP58_INITIN(v) \
                                                        : GPIO1PIN_NP58_INITOUT(v) )

#define GPIO1PIN_NP58_INITIN(v)      do{ bFM_GPIO_PCR5_P8=(v).bPullup; \
                                         bFM_GPIO_DDR5_P8=0u; \
                                         bFM_GPIO_PFR5_P8=0u; }while(0)

#define GPIO1PIN_NP58_INITOUT(v)     do{ bFM_GPIO_PDOR5_P8=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR5_P8=1u; \
                                         bFM_GPIO_PFR5_P8=0u; }while(0)


/*---- GPIO bit P59 ----*/

#define GPIO1PIN_P59_GET             ( bFM_GPIO_PDIR5_P9 )

#define GPIO1PIN_P59_PUT(v)          ( bFM_GPIO_PDOR5_P9=(v) )

#define GPIO1PIN_P59_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P59_INITIN(v) \
                                                        : GPIO1PIN_P59_INITOUT(v) )

#define GPIO1PIN_P59_INITIN(v)       do{ bFM_GPIO_PCR5_P9=(v).bPullup; \
                                         bFM_GPIO_DDR5_P9=0u; \
                                         bFM_GPIO_PFR5_P9=0u; }while(0)

#define GPIO1PIN_P59_INITOUT(v)      do{ bFM_GPIO_PDOR5_P9=(v).bInitVal; \
                                         bFM_GPIO_DDR5_P9=1u; \
                                         bFM_GPIO_PFR5_P9=0u; }while(0)


/*---- GPIO bit NP59 ----*/
#define GPIO1PIN_NP59_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR5_P9)) )

#define GPIO1PIN_NP59_PUT(v)         ( bFM_GPIO_PDOR5_P9=(uint32_t)(!(v)) )

#define GPIO1PIN_NP59_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP59_INITIN(v) \
                                                        : GPIO1PIN_NP59_INITOUT(v) )

#define GPIO1PIN_NP59_INITIN(v)      do{ bFM_GPIO_PCR5_P9=(v).bPullup; \
                                         bFM_GPIO_DDR5_P9=0u; \
                                         bFM_GPIO_PFR5_P9=0u; }while(0)

#define GPIO1PIN_NP59_INITOUT(v)     do{ bFM_GPIO_PDOR5_P9=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR5_P9=1u; \
                                         bFM_GPIO_PFR5_P9=0u; }while(0)


/*---- GPIO bit P5A ----*/

#define GPIO1PIN_P5A_GET             ( bFM_GPIO_PDIR5_PA )

#define GPIO1PIN_P5A_PUT(v)          ( bFM_GPIO_PDOR5_PA=(v) )

#define GPIO1PIN_P5A_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P5A_INITIN(v) \
                                                        : GPIO1PIN_P5A_INITOUT(v) )

#define GPIO1PIN_P5A_INITIN(v)       do{ bFM_GPIO_PCR5_PA=(v).bPullup; \
                                         bFM_GPIO_DDR5_PA=0u; \
                                         bFM_GPIO_PFR5_PA=0u; }while(0)

#define GPIO1PIN_P5A_INITOUT(v)      do{ bFM_GPIO_PDOR5_PA=(v).bInitVal; \
                                         bFM_GPIO_DDR5_PA=1u; \
                                         bFM_GPIO_PFR5_PA=0u; }while(0)


/*---- GPIO bit NP5A ----*/
#define GPIO1PIN_NP5A_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR5_PA)) )

#define GPIO1PIN_NP5A_PUT(v)         ( bFM_GPIO_PDOR5_PA=(uint32_t)(!(v)) )

#define GPIO1PIN_NP5A_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP5A_INITIN(v) \
                                                        : GPIO1PIN_NP5A_INITOUT(v) )

#define GPIO1PIN_NP5A_INITIN(v)      do{ bFM_GPIO_PCR5_PA=(v).bPullup; \
                                         bFM_GPIO_DDR5_PA=0u; \
                                         bFM_GPIO_PFR5_PA=0u; }while(0)

#define GPIO1PIN_NP5A_INITOUT(v)     do{ bFM_GPIO_PDOR5_PA=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR5_PA=1u; \
                                         bFM_GPIO_PFR5_PA=0u; }while(0)


/*---- GPIO bit P5B ----*/

#define GPIO1PIN_P5B_GET             ( bFM_GPIO_PDIR5_PB )

#define GPIO1PIN_P5B_PUT(v)          ( bFM_GPIO_PDOR5_PB=(v) )

#define GPIO1PIN_P5B_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P5B_INITIN(v) \
                                                        : GPIO1PIN_P5B_INITOUT(v) )

#define GPIO1PIN_P5B_INITIN(v)       do{ bFM_GPIO_PCR5_PB=(v).bPullup; \
                                         bFM_GPIO_DDR5_PB=0u; \
                                         bFM_GPIO_PFR5_PB=0u; }while(0)

#define GPIO1PIN_P5B_INITOUT(v)      do{ bFM_GPIO_PDOR5_PB=(v).bInitVal; \
                                         bFM_GPIO_DDR5_PB=1u; \
                                         bFM_GPIO_PFR5_PB=0u; }while(0)


/*---- GPIO bit NP5B ----*/
#define GPIO1PIN_NP5B_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR5_PB)) )

#define GPIO1PIN_NP5B_PUT(v)         ( bFM_GPIO_PDOR5_PB=(uint32_t)(!(v)) )

#define GPIO1PIN_NP5B_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP5B_INITIN(v) \
                                                        : GPIO1PIN_NP5B_INITOUT(v) )

#define GPIO1PIN_NP5B_INITIN(v)      do{ bFM_GPIO_PCR5_PB=(v).bPullup; \
                                         bFM_GPIO_DDR5_PB=0u; \
                                         bFM_GPIO_PFR5_PB=0u; }while(0)

#define GPIO1PIN_NP5B_INITOUT(v)     do{ bFM_GPIO_PDOR5_PB=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR5_PB=1u; \
                                         bFM_GPIO_PFR5_PB=0u; }while(0)


/*---- GPIO bit P60 ----*/

#define GPIO1PIN_P60_GET             ( bFM_GPIO_PDIR6_P0 )

#define GPIO1PIN_P60_PUT(v)          ( bFM_GPIO_PDOR6_P0=(v) )

#define GPIO1PIN_P60_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P60_INITIN(v) \
                                                        : GPIO1PIN_P60_INITOUT(v) )

#define GPIO1PIN_P60_INITIN(v)       do{ bFM_GPIO_PCR6_P0=(v).bPullup; \
                                         bFM_GPIO_DDR6_P0=0u; \
                                         bFM_GPIO_PFR6_P0=0u; }while(0)

#define GPIO1PIN_P60_INITOUT(v)      do{ bFM_GPIO_PDOR6_P0=(v).bInitVal; \
                                         bFM_GPIO_DDR6_P0=1u; \
                                         bFM_GPIO_PFR6_P0=0u; }while(0)


/*---- GPIO bit NP60 ----*/
#define GPIO1PIN_NP60_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR6_P0)) )

#define GPIO1PIN_NP60_PUT(v)         ( bFM_GPIO_PDOR6_P0=(uint32_t)(!(v)) )

#define GPIO1PIN_NP60_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP60_INITIN(v) \
                                                        : GPIO1PIN_NP60_INITOUT(v) )

#define GPIO1PIN_NP60_INITIN(v)      do{ bFM_GPIO_PCR6_P0=(v).bPullup; \
                                         bFM_GPIO_DDR6_P0=0u; \
                                         bFM_GPIO_PFR6_P0=0u; }while(0)

#define GPIO1PIN_NP60_INITOUT(v)     do{ bFM_GPIO_PDOR6_P0=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR6_P0=1u; \
                                         bFM_GPIO_PFR6_P0=0u; }while(0)


/*---- GPIO bit P61 ----*/

#define GPIO1PIN_P61_GET             ( bFM_GPIO_PDIR6_P1 )

#define GPIO1PIN_P61_PUT(v)          ( bFM_GPIO_PDOR6_P1=(v) )

#define GPIO1PIN_P61_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P61_INITIN(v) \
                                                        : GPIO1PIN_P61_INITOUT(v) )

#define GPIO1PIN_P61_INITIN(v)       do{ bFM_GPIO_PCR6_P1=(v).bPullup; \
                                         bFM_GPIO_DDR6_P1=0u; \
                                         bFM_GPIO_PFR6_P1=0u; }while(0)

#define GPIO1PIN_P61_INITOUT(v)      do{ bFM_GPIO_PDOR6_P1=(v).bInitVal; \
                                         bFM_GPIO_DDR6_P1=1u; \
                                         bFM_GPIO_PFR6_P1=0u; }while(0)


/*---- GPIO bit NP61 ----*/
#define GPIO1PIN_NP61_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR6_P1)) )

#define GPIO1PIN_NP61_PUT(v)         ( bFM_GPIO_PDOR6_P1=(uint32_t)(!(v)) )

#define GPIO1PIN_NP61_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP61_INITIN(v) \
                                                        : GPIO1PIN_NP61_INITOUT(v) )

#define GPIO1PIN_NP61_INITIN(v)      do{ bFM_GPIO_PCR6_P1=(v).bPullup; \
                                         bFM_GPIO_DDR6_P1=0u; \
                                         bFM_GPIO_PFR6_P1=0u; }while(0)

#define GPIO1PIN_NP61_INITOUT(v)     do{ bFM_GPIO_PDOR6_P1=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR6_P1=1u; \
                                         bFM_GPIO_PFR6_P1=0u; }while(0)


/*---- GPIO bit P62 ----*/

#define GPIO1PIN_P62_GET             ( bFM_GPIO_PDIR6_P2 )

#define GPIO1PIN_P62_PUT(v)          ( bFM_GPIO_PDOR6_P2=(v) )

#define GPIO1PIN_P62_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P62_INITIN(v) \
                                                        : GPIO1PIN_P62_INITOUT(v) )

#define GPIO1PIN_P62_INITIN(v)       do{ bFM_GPIO_PCR6_P2=(v).bPullup; \
                                         bFM_GPIO_DDR6_P2=0u; \
                                         bFM_GPIO_PFR6_P2=0u; }while(0)

#define GPIO1PIN_P62_INITOUT(v)      do{ bFM_GPIO_PDOR6_P2=(v).bInitVal; \
                                         bFM_GPIO_DDR6_P2=1u; \
                                         bFM_GPIO_PFR6_P2=0u; }while(0)


/*---- GPIO bit NP62 ----*/
#define GPIO1PIN_NP62_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR6_P2)) )

#define GPIO1PIN_NP62_PUT(v)         ( bFM_GPIO_PDOR6_P2=(uint32_t)(!(v)) )

#define GPIO1PIN_NP62_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP62_INITIN(v) \
                                                        : GPIO1PIN_NP62_INITOUT(v) )

#define GPIO1PIN_NP62_INITIN(v)      do{ bFM_GPIO_PCR6_P2=(v).bPullup; \
                                         bFM_GPIO_DDR6_P2=0u; \
                                         bFM_GPIO_PFR6_P2=0u; }while(0)

#define GPIO1PIN_NP62_INITOUT(v)     do{ bFM_GPIO_PDOR6_P2=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR6_P2=1u; \
                                         bFM_GPIO_PFR6_P2=0u; }while(0)


/*---- GPIO bit P70 ----*/

#define GPIO1PIN_P70_GET             ( bFM_GPIO_PDIR7_P0 )

#define GPIO1PIN_P70_PUT(v)          ( bFM_GPIO_PDOR7_P0=(v) )

#define GPIO1PIN_P70_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P70_INITIN(v) \
                                                        : GPIO1PIN_P70_INITOUT(v) )

#define GPIO1PIN_P70_INITIN(v)       do{ bFM_GPIO_PCR7_P0=(v).bPullup; \
                                         bFM_GPIO_DDR7_P0=0u; \
                                         bFM_GPIO_PFR7_P0=0u; }while(0)

#define GPIO1PIN_P70_INITOUT(v)      do{ bFM_GPIO_PDOR7_P0=(v).bInitVal; \
                                         bFM_GPIO_DDR7_P0=1u; \
                                         bFM_GPIO_PFR7_P0=0u; }while(0)


/*---- GPIO bit NP70 ----*/
#define GPIO1PIN_NP70_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR7_P0)) )

#define GPIO1PIN_NP70_PUT(v)         ( bFM_GPIO_PDOR7_P0=(uint32_t)(!(v)) )

#define GPIO1PIN_NP70_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP70_INITIN(v) \
                                                        : GPIO1PIN_NP70_INITOUT(v) )

#define GPIO1PIN_NP70_INITIN(v)      do{ bFM_GPIO_PCR7_P0=(v).bPullup; \
                                         bFM_GPIO_DDR7_P0=0u; \
                                         bFM_GPIO_PFR7_P0=0u; }while(0)

#define GPIO1PIN_NP70_INITOUT(v)     do{ bFM_GPIO_PDOR7_P0=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR7_P0=1u; \
                                         bFM_GPIO_PFR7_P0=0u; }while(0)


/*---- GPIO bit P71 ----*/

#define GPIO1PIN_P71_GET             ( bFM_GPIO_PDIR7_P1 )

#define GPIO1PIN_P71_PUT(v)          ( bFM_GPIO_PDOR7_P1=(v) )

#define GPIO1PIN_P71_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P71_INITIN(v) \
                                                        : GPIO1PIN_P71_INITOUT(v) )

#define GPIO1PIN_P71_INITIN(v)       do{ bFM_GPIO_PCR7_P1=(v).bPullup; \
                                         bFM_GPIO_DDR7_P1=0u; \
                                         bFM_GPIO_PFR7_P1=0u; }while(0)

#define GPIO1PIN_P71_INITOUT(v)      do{ bFM_GPIO_PDOR7_P1=(v).bInitVal; \
                                         bFM_GPIO_DDR7_P1=1u; \
                                         bFM_GPIO_PFR7_P1=0u; }while(0)


/*---- GPIO bit NP71 ----*/
#define GPIO1PIN_NP71_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR7_P1)) )

#define GPIO1PIN_NP71_PUT(v)         ( bFM_GPIO_PDOR7_P1=(uint32_t)(!(v)) )

#define GPIO1PIN_NP71_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP71_INITIN(v) \
                                                        : GPIO1PIN_NP71_INITOUT(v) )

#define GPIO1PIN_NP71_INITIN(v)      do{ bFM_GPIO_PCR7_P1=(v).bPullup; \
                                         bFM_GPIO_DDR7_P1=0u; \
                                         bFM_GPIO_PFR7_P1=0u; }while(0)

#define GPIO1PIN_NP71_INITOUT(v)     do{ bFM_GPIO_PDOR7_P1=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR7_P1=1u; \
                                         bFM_GPIO_PFR7_P1=0u; }while(0)


/*---- GPIO bit P72 ----*/

#define GPIO1PIN_P72_GET             ( bFM_GPIO_PDIR7_P2 )

#define GPIO1PIN_P72_PUT(v)          ( bFM_GPIO_PDOR7_P2=(v) )

#define GPIO1PIN_P72_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P72_INITIN(v) \
                                                        : GPIO1PIN_P72_INITOUT(v) )

#define GPIO1PIN_P72_INITIN(v)       do{ bFM_GPIO_PCR7_P2=(v).bPullup; \
                                         bFM_GPIO_DDR7_P2=0u; \
                                         bFM_GPIO_PFR7_P2=0u; }while(0)

#define GPIO1PIN_P72_INITOUT(v)      do{ bFM_GPIO_PDOR7_P2=(v).bInitVal; \
                                         bFM_GPIO_DDR7_P2=1u; \
                                         bFM_GPIO_PFR7_P2=0u; }while(0)


/*---- GPIO bit NP72 ----*/
#define GPIO1PIN_NP72_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR7_P2)) )

#define GPIO1PIN_NP72_PUT(v)         ( bFM_GPIO_PDOR7_P2=(uint32_t)(!(v)) )

#define GPIO1PIN_NP72_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP72_INITIN(v) \
                                                        : GPIO1PIN_NP72_INITOUT(v) )

#define GPIO1PIN_NP72_INITIN(v)      do{ bFM_GPIO_PCR7_P2=(v).bPullup; \
                                         bFM_GPIO_DDR7_P2=0u; \
                                         bFM_GPIO_PFR7_P2=0u; }while(0)

#define GPIO1PIN_NP72_INITOUT(v)     do{ bFM_GPIO_PDOR7_P2=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR7_P2=1u; \
                                         bFM_GPIO_PFR7_P2=0u; }while(0)


/*---- GPIO bit P73 ----*/

#define GPIO1PIN_P73_GET             ( bFM_GPIO_PDIR7_P3 )

#define GPIO1PIN_P73_PUT(v)          ( bFM_GPIO_PDOR7_P3=(v) )

#define GPIO1PIN_P73_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P73_INITIN(v) \
                                                        : GPIO1PIN_P73_INITOUT(v) )

#define GPIO1PIN_P73_INITIN(v)       do{ bFM_GPIO_PCR7_P3=(v).bPullup; \
                                         bFM_GPIO_DDR7_P3=0u; \
                                         bFM_GPIO_PFR7_P3=0u; }while(0)

#define GPIO1PIN_P73_INITOUT(v)      do{ bFM_GPIO_PDOR7_P3=(v).bInitVal; \
                                         bFM_GPIO_DDR7_P3=1u; \
                                         bFM_GPIO_PFR7_P3=0u; }while(0)


/*---- GPIO bit NP73 ----*/
#define GPIO1PIN_NP73_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR7_P3)) )

#define GPIO1PIN_NP73_PUT(v)         ( bFM_GPIO_PDOR7_P3=(uint32_t)(!(v)) )

#define GPIO1PIN_NP73_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP73_INITIN(v) \
                                                        : GPIO1PIN_NP73_INITOUT(v) )

#define GPIO1PIN_NP73_INITIN(v)      do{ bFM_GPIO_PCR7_P3=(v).bPullup; \
                                         bFM_GPIO_DDR7_P3=0u; \
                                         bFM_GPIO_PFR7_P3=0u; }while(0)

#define GPIO1PIN_NP73_INITOUT(v)     do{ bFM_GPIO_PDOR7_P3=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR7_P3=1u; \
                                         bFM_GPIO_PFR7_P3=0u; }while(0)


/*---- GPIO bit P74 ----*/

#define GPIO1PIN_P74_GET             ( bFM_GPIO_PDIR7_P4 )

#define GPIO1PIN_P74_PUT(v)          ( bFM_GPIO_PDOR7_P4=(v) )

#define GPIO1PIN_P74_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P74_INITIN(v) \
                                                        : GPIO1PIN_P74_INITOUT(v) )

#define GPIO1PIN_P74_INITIN(v)       do{ bFM_GPIO_PCR7_P4=(v).bPullup; \
                                         bFM_GPIO_DDR7_P4=0u; \
                                         bFM_GPIO_PFR7_P4=0u; }while(0)

#define GPIO1PIN_P74_INITOUT(v)      do{ bFM_GPIO_PDOR7_P4=(v).bInitVal; \
                                         bFM_GPIO_DDR7_P4=1u; \
                                         bFM_GPIO_PFR7_P4=0u; }while(0)


/*---- GPIO bit NP74 ----*/
#define GPIO1PIN_NP74_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR7_P4)) )

#define GPIO1PIN_NP74_PUT(v)         ( bFM_GPIO_PDOR7_P4=(uint32_t)(!(v)) )

#define GPIO1PIN_NP74_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP74_INITIN(v) \
                                                        : GPIO1PIN_NP74_INITOUT(v) )

#define GPIO1PIN_NP74_INITIN(v)      do{ bFM_GPIO_PCR7_P4=(v).bPullup; \
                                         bFM_GPIO_DDR7_P4=0u; \
                                         bFM_GPIO_PFR7_P4=0u; }while(0)

#define GPIO1PIN_NP74_INITOUT(v)     do{ bFM_GPIO_PDOR7_P4=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR7_P4=1u; \
                                         bFM_GPIO_PFR7_P4=0u; }while(0)


/*---- GPIO bit P75 ----*/

#define GPIO1PIN_P75_GET             ( bFM_GPIO_PDIR7_P5 )

#define GPIO1PIN_P75_PUT(v)          ( bFM_GPIO_PDOR7_P5=(v) )

#define GPIO1PIN_P75_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P75_INITIN(v) \
                                                        : GPIO1PIN_P75_INITOUT(v) )

#define GPIO1PIN_P75_INITIN(v)       do{ bFM_GPIO_PCR7_P5=(v).bPullup; \
                                         bFM_GPIO_DDR7_P5=0u; \
                                         bFM_GPIO_PFR7_P5=0u; }while(0)

#define GPIO1PIN_P75_INITOUT(v)      do{ bFM_GPIO_PDOR7_P5=(v).bInitVal; \
                                         bFM_GPIO_DDR7_P5=1u; \
                                         bFM_GPIO_PFR7_P5=0u; }while(0)


/*---- GPIO bit NP75 ----*/
#define GPIO1PIN_NP75_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR7_P5)) )

#define GPIO1PIN_NP75_PUT(v)         ( bFM_GPIO_PDOR7_P5=(uint32_t)(!(v)) )

#define GPIO1PIN_NP75_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP75_INITIN(v) \
                                                        : GPIO1PIN_NP75_INITOUT(v) )

#define GPIO1PIN_NP75_INITIN(v)      do{ bFM_GPIO_PCR7_P5=(v).bPullup; \
                                         bFM_GPIO_DDR7_P5=0u; \
                                         bFM_GPIO_PFR7_P5=0u; }while(0)

#define GPIO1PIN_NP75_INITOUT(v)     do{ bFM_GPIO_PDOR7_P5=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR7_P5=1u; \
                                         bFM_GPIO_PFR7_P5=0u; }while(0)


/*---- GPIO bit P76 ----*/

#define GPIO1PIN_P76_GET             ( bFM_GPIO_PDIR7_P6 )

#define GPIO1PIN_P76_PUT(v)          ( bFM_GPIO_PDOR7_P6=(v) )

#define GPIO1PIN_P76_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P76_INITIN(v) \
                                                        : GPIO1PIN_P76_INITOUT(v) )

#define GPIO1PIN_P76_INITIN(v)       do{ bFM_GPIO_PCR7_P6=(v).bPullup; \
                                         bFM_GPIO_DDR7_P6=0u; \
                                         bFM_GPIO_PFR7_P6=0u; }while(0)

#define GPIO1PIN_P76_INITOUT(v)      do{ bFM_GPIO_PDOR7_P6=(v).bInitVal; \
                                         bFM_GPIO_DDR7_P6=1u; \
                                         bFM_GPIO_PFR7_P6=0u; }while(0)


/*---- GPIO bit NP76 ----*/
#define GPIO1PIN_NP76_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR7_P6)) )

#define GPIO1PIN_NP76_PUT(v)         ( bFM_GPIO_PDOR7_P6=(uint32_t)(!(v)) )

#define GPIO1PIN_NP76_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP76_INITIN(v) \
                                                        : GPIO1PIN_NP76_INITOUT(v) )

#define GPIO1PIN_NP76_INITIN(v)      do{ bFM_GPIO_PCR7_P6=(v).bPullup; \
                                         bFM_GPIO_DDR7_P6=0u; \
                                         bFM_GPIO_PFR7_P6=0u; }while(0)

#define GPIO1PIN_NP76_INITOUT(v)     do{ bFM_GPIO_PDOR7_P6=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR7_P6=1u; \
                                         bFM_GPIO_PFR7_P6=0u; }while(0)


/*---- GPIO bit P77 ----*/

#define GPIO1PIN_P77_GET             ( bFM_GPIO_PDIR7_P7 )

#define GPIO1PIN_P77_PUT(v)          ( bFM_GPIO_PDOR7_P7=(v) )

#define GPIO1PIN_P77_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P77_INITIN(v) \
                                                        : GPIO1PIN_P77_INITOUT(v) )

#define GPIO1PIN_P77_INITIN(v)       do{ bFM_GPIO_PCR7_P7=(v).bPullup; \
                                         bFM_GPIO_DDR7_P7=0u; \
                                         bFM_GPIO_PFR7_P7=0u; }while(0)

#define GPIO1PIN_P77_INITOUT(v)      do{ bFM_GPIO_PDOR7_P7=(v).bInitVal; \
                                         bFM_GPIO_DDR7_P7=1u; \
                                         bFM_GPIO_PFR7_P7=0u; }while(0)


/*---- GPIO bit NP77 ----*/
#define GPIO1PIN_NP77_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR7_P7)) )

#define GPIO1PIN_NP77_PUT(v)         ( bFM_GPIO_PDOR7_P7=(uint32_t)(!(v)) )

#define GPIO1PIN_NP77_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP77_INITIN(v) \
                                                        : GPIO1PIN_NP77_INITOUT(v) )

#define GPIO1PIN_NP77_INITIN(v)      do{ bFM_GPIO_PCR7_P7=(v).bPullup; \
                                         bFM_GPIO_DDR7_P7=0u; \
                                         bFM_GPIO_PFR7_P7=0u; }while(0)

#define GPIO1PIN_NP77_INITOUT(v)     do{ bFM_GPIO_PDOR7_P7=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR7_P7=1u; \
                                         bFM_GPIO_PFR7_P7=0u; }while(0)


/*---- GPIO bit P78 ----*/

#define GPIO1PIN_P78_GET             ( bFM_GPIO_PDIR7_P8 )

#define GPIO1PIN_P78_PUT(v)          ( bFM_GPIO_PDOR7_P8=(v) )

#define GPIO1PIN_P78_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P78_INITIN(v) \
                                                        : GPIO1PIN_P78_INITOUT(v) )

#define GPIO1PIN_P78_INITIN(v)       do{ bFM_GPIO_PCR7_P8=(v).bPullup; \
                                         bFM_GPIO_DDR7_P8=0u; \
                                         bFM_GPIO_PFR7_P8=0u; }while(0)

#define GPIO1PIN_P78_INITOUT(v)      do{ bFM_GPIO_PDOR7_P8=(v).bInitVal; \
                                         bFM_GPIO_DDR7_P8=1u; \
                                         bFM_GPIO_PFR7_P8=0u; }while(0)


/*---- GPIO bit NP78 ----*/
#define GPIO1PIN_NP78_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR7_P8)) )

#define GPIO1PIN_NP78_PUT(v)         ( bFM_GPIO_PDOR7_P8=(uint32_t)(!(v)) )

#define GPIO1PIN_NP78_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP78_INITIN(v) \
                                                        : GPIO1PIN_NP78_INITOUT(v) )

#define GPIO1PIN_NP78_INITIN(v)      do{ bFM_GPIO_PCR7_P8=(v).bPullup; \
                                         bFM_GPIO_DDR7_P8=0u; \
                                         bFM_GPIO_PFR7_P8=0u; }while(0)

#define GPIO1PIN_NP78_INITOUT(v)     do{ bFM_GPIO_PDOR7_P8=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR7_P8=1u; \
                                         bFM_GPIO_PFR7_P8=0u; }while(0)


/*---- GPIO bit P79 ----*/

#define GPIO1PIN_P79_GET             ( bFM_GPIO_PDIR7_P9 )

#define GPIO1PIN_P79_PUT(v)          ( bFM_GPIO_PDOR7_P9=(v) )

#define GPIO1PIN_P79_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P79_INITIN(v) \
                                                        : GPIO1PIN_P79_INITOUT(v) )

#define GPIO1PIN_P79_INITIN(v)       do{ bFM_GPIO_PCR7_P9=(v).bPullup; \
                                         bFM_GPIO_DDR7_P9=0u; \
                                         bFM_GPIO_PFR7_P9=0u; }while(0)

#define GPIO1PIN_P79_INITOUT(v)      do{ bFM_GPIO_PDOR7_P9=(v).bInitVal; \
                                         bFM_GPIO_DDR7_P9=1u; \
                                         bFM_GPIO_PFR7_P9=0u; }while(0)


/*---- GPIO bit NP79 ----*/
#define GPIO1PIN_NP79_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR7_P9)) )

#define GPIO1PIN_NP79_PUT(v)         ( bFM_GPIO_PDOR7_P9=(uint32_t)(!(v)) )

#define GPIO1PIN_NP79_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP79_INITIN(v) \
                                                        : GPIO1PIN_NP79_INITOUT(v) )

#define GPIO1PIN_NP79_INITIN(v)      do{ bFM_GPIO_PCR7_P9=(v).bPullup; \
                                         bFM_GPIO_DDR7_P9=0u; \
                                         bFM_GPIO_PFR7_P9=0u; }while(0)

#define GPIO1PIN_NP79_INITOUT(v)     do{ bFM_GPIO_PDOR7_P9=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR7_P9=1u; \
                                         bFM_GPIO_PFR7_P9=0u; }while(0)


/*---- GPIO bit P7A ----*/

#define GPIO1PIN_P7A_GET             ( bFM_GPIO_PDIR7_PA )

#define GPIO1PIN_P7A_PUT(v)          ( bFM_GPIO_PDOR7_PA=(v) )

#define GPIO1PIN_P7A_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P7A_INITIN(v) \
                                                        : GPIO1PIN_P7A_INITOUT(v) )

#define GPIO1PIN_P7A_INITIN(v)       do{ bFM_GPIO_PCR7_PA=(v).bPullup; \
                                         bFM_GPIO_DDR7_PA=0u; \
                                         bFM_GPIO_PFR7_PA=0u; }while(0)

#define GPIO1PIN_P7A_INITOUT(v)      do{ bFM_GPIO_PDOR7_PA=(v).bInitVal; \
                                         bFM_GPIO_DDR7_PA=1u; \
                                         bFM_GPIO_PFR7_PA=0u; }while(0)


/*---- GPIO bit NP7A ----*/
#define GPIO1PIN_NP7A_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR7_PA)) )

#define GPIO1PIN_NP7A_PUT(v)         ( bFM_GPIO_PDOR7_PA=(uint32_t)(!(v)) )

#define GPIO1PIN_NP7A_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP7A_INITIN(v) \
                                                        : GPIO1PIN_NP7A_INITOUT(v) )

#define GPIO1PIN_NP7A_INITIN(v)      do{ bFM_GPIO_PCR7_PA=(v).bPullup; \
                                         bFM_GPIO_DDR7_PA=0u; \
                                         bFM_GPIO_PFR7_PA=0u; }while(0)

#define GPIO1PIN_NP7A_INITOUT(v)     do{ bFM_GPIO_PDOR7_PA=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR7_PA=1u; \
                                         bFM_GPIO_PFR7_PA=0u; }while(0)


/*---- GPIO bit P80 ----*/

#define GPIO1PIN_P80_GET             ( bFM_GPIO_PDIR8_P0 )

#define GPIO1PIN_P80_PUT(v)          ( bFM_GPIO_PDOR8_P0=(v) )

#define GPIO1PIN_P80_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P80_INITIN(v) \
                                                        : GPIO1PIN_P80_INITOUT(v) )

#define GPIO1PIN_P80_INITIN(v)       do{ bFM_GPIO_PCR8_P0=(v).bPullup; \
                                         bFM_GPIO_DDR8_P0=0u; \
                                         bFM_GPIO_PFR8_P0=0u; }while(0)

#define GPIO1PIN_P80_INITOUT(v)      do{ bFM_GPIO_PDOR8_P0=(v).bInitVal; \
                                         bFM_GPIO_DDR8_P0=1u; \
                                         bFM_GPIO_PFR8_P0=0u; }while(0)


/*---- GPIO bit NP80 ----*/
#define GPIO1PIN_NP80_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR8_P0)) )

#define GPIO1PIN_NP80_PUT(v)         ( bFM_GPIO_PDOR8_P0=(uint32_t)(!(v)) )

#define GPIO1PIN_NP80_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP80_INITIN(v) \
                                                        : GPIO1PIN_NP80_INITOUT(v) )

#define GPIO1PIN_NP80_INITIN(v)      do{ bFM_GPIO_PCR8_P0=(v).bPullup; \
                                         bFM_GPIO_DDR8_P0=0u; \
                                         bFM_GPIO_PFR8_P0=0u; }while(0)

#define GPIO1PIN_NP80_INITOUT(v)     do{ bFM_GPIO_PDOR8_P0=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR8_P0=1u; \
                                         bFM_GPIO_PFR8_P0=0u; }while(0)


/*---- GPIO bit P81 ----*/

#define GPIO1PIN_P81_GET             ( bFM_GPIO_PDIR8_P1 )

#define GPIO1PIN_P81_PUT(v)          ( bFM_GPIO_PDOR8_P1=(v) )

#define GPIO1PIN_P81_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P81_INITIN(v) \
                                                        : GPIO1PIN_P81_INITOUT(v) )

#define GPIO1PIN_P81_INITIN(v)       do{ bFM_GPIO_PCR8_P1=(v).bPullup; \
                                         bFM_GPIO_DDR8_P1=0u; \
                                         bFM_GPIO_PFR8_P1=0u; }while(0)

#define GPIO1PIN_P81_INITOUT(v)      do{ bFM_GPIO_PDOR8_P1=(v).bInitVal; \
                                         bFM_GPIO_DDR8_P1=1u; \
                                         bFM_GPIO_PFR8_P1=0u; }while(0)


/*---- GPIO bit NP81 ----*/
#define GPIO1PIN_NP81_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR8_P1)) )

#define GPIO1PIN_NP81_PUT(v)         ( bFM_GPIO_PDOR8_P1=(uint32_t)(!(v)) )

#define GPIO1PIN_NP81_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP81_INITIN(v) \
                                                        : GPIO1PIN_NP81_INITOUT(v) )

#define GPIO1PIN_NP81_INITIN(v)      do{ bFM_GPIO_PCR8_P1=(v).bPullup; \
                                         bFM_GPIO_DDR8_P1=0u; \
                                         bFM_GPIO_PFR8_P1=0u; }while(0)

#define GPIO1PIN_NP81_INITOUT(v)     do{ bFM_GPIO_PDOR8_P1=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR8_P1=1u; \
                                         bFM_GPIO_PFR8_P1=0u; }while(0)


/*---- GPIO bit P82 ----*/

#define GPIO1PIN_P82_GET             ( bFM_GPIO_PDIR8_P2 )

#define GPIO1PIN_P82_PUT(v)          ( bFM_GPIO_PDOR8_P2=(v) )

#define GPIO1PIN_P82_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P82_INITIN(v) \
                                                        : GPIO1PIN_P82_INITOUT(v) )

#define GPIO1PIN_P82_INITIN(v)       do{ bFM_GPIO_PCR8_P2=(v).bPullup; \
                                         bFM_GPIO_DDR8_P2=0u; \
                                         bFM_GPIO_PFR8_P2=0u; }while(0)

#define GPIO1PIN_P82_INITOUT(v)      do{ bFM_GPIO_PDOR8_P2=(v).bInitVal; \
                                         bFM_GPIO_DDR8_P2=1u; \
                                         bFM_GPIO_PFR8_P2=0u; }while(0)


/*---- GPIO bit NP82 ----*/
#define GPIO1PIN_NP82_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR8_P2)) )

#define GPIO1PIN_NP82_PUT(v)         ( bFM_GPIO_PDOR8_P2=(uint32_t)(!(v)) )

#define GPIO1PIN_NP82_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP82_INITIN(v) \
                                                        : GPIO1PIN_NP82_INITOUT(v) )

#define GPIO1PIN_NP82_INITIN(v)      do{ bFM_GPIO_PCR8_P2=(v).bPullup; \
                                         bFM_GPIO_DDR8_P2=0u; \
                                         bFM_GPIO_PFR8_P2=0u; }while(0)

#define GPIO1PIN_NP82_INITOUT(v)     do{ bFM_GPIO_PDOR8_P2=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR8_P2=1u; \
                                         bFM_GPIO_PFR8_P2=0u; }while(0)


/*---- GPIO bit P83 ----*/

#define GPIO1PIN_P83_GET             ( bFM_GPIO_PDIR8_P3 )

#define GPIO1PIN_P83_PUT(v)          ( bFM_GPIO_PDOR8_P3=(v) )

#define GPIO1PIN_P83_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P83_INITIN(v) \
                                                        : GPIO1PIN_P83_INITOUT(v) )

#define GPIO1PIN_P83_INITIN(v)       do{ bFM_GPIO_PCR8_P3=(v).bPullup; \
                                         bFM_GPIO_DDR8_P3=0u; \
                                         bFM_GPIO_PFR8_P3=0u; }while(0)

#define GPIO1PIN_P83_INITOUT(v)      do{ bFM_GPIO_PDOR8_P3=(v).bInitVal; \
                                         bFM_GPIO_DDR8_P3=1u; \
                                         bFM_GPIO_PFR8_P3=0u; }while(0)


/*---- GPIO bit NP83 ----*/
#define GPIO1PIN_NP83_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR8_P3)) )

#define GPIO1PIN_NP83_PUT(v)         ( bFM_GPIO_PDOR8_P3=(uint32_t)(!(v)) )

#define GPIO1PIN_NP83_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP83_INITIN(v) \
                                                        : GPIO1PIN_NP83_INITOUT(v) )

#define GPIO1PIN_NP83_INITIN(v)      do{ bFM_GPIO_PCR8_P3=(v).bPullup; \
                                         bFM_GPIO_DDR8_P3=0u; \
                                         bFM_GPIO_PFR8_P3=0u; }while(0)

#define GPIO1PIN_NP83_INITOUT(v)     do{ bFM_GPIO_PDOR8_P3=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR8_P3=1u; \
                                         bFM_GPIO_PFR8_P3=0u; }while(0)


/*---- GPIO bit PA0 ----*/

#define GPIO1PIN_PA0_GET             ( bFM_GPIO_PDIRA_P0 )

#define GPIO1PIN_PA0_PUT(v)          ( bFM_GPIO_PDORA_P0=(v) )

#define GPIO1PIN_PA0_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_PA0_INITIN(v) \
                                                        : GPIO1PIN_PA0_INITOUT(v) )

#define GPIO1PIN_PA0_INITIN(v)       do{ bFM_GPIO_PCRA_P0=(v).bPullup; \
                                         bFM_GPIO_DDRA_P0=0u; \
                                         bFM_GPIO_PFRA_P0=0u; }while(0)

#define GPIO1PIN_PA0_INITOUT(v)      do{ bFM_GPIO_PDORA_P0=(v).bInitVal; \
                                         bFM_GPIO_DDRA_P0=1u; \
                                         bFM_GPIO_PFRA_P0=0u; }while(0)


/*---- GPIO bit NPA0 ----*/
#define GPIO1PIN_NPA0_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIRA_P0)) )

#define GPIO1PIN_NPA0_PUT(v)         ( bFM_GPIO_PDORA_P0=(uint32_t)(!(v)) )

#define GPIO1PIN_NPA0_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NPA0_INITIN(v) \
                                                        : GPIO1PIN_NPA0_INITOUT(v) )

#define GPIO1PIN_NPA0_INITIN(v)      do{ bFM_GPIO_PCRA_P0=(v).bPullup; \
                                         bFM_GPIO_DDRA_P0=0u; \
                                         bFM_GPIO_PFRA_P0=0u; }while(0)

#define GPIO1PIN_NPA0_INITOUT(v)     do{ bFM_GPIO_PDORA_P0=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDRA_P0=1u; \
                                         bFM_GPIO_PFRA_P0=0u; }while(0)


/*---- GPIO bit PA1 ----*/

#define GPIO1PIN_PA1_GET             ( bFM_GPIO_PDIRA_P1 )

#define GPIO1PIN_PA1_PUT(v)          ( bFM_GPIO_PDORA_P1=(v) )

#define GPIO1PIN_PA1_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_PA1_INITIN(v) \
                                                        : GPIO1PIN_PA1_INITOUT(v) )

#define GPIO1PIN_PA1_INITIN(v)       do{ bFM_GPIO_PCRA_P1=(v).bPullup; \
                                         bFM_GPIO_DDRA_P1=0u; \
                                         bFM_GPIO_PFRA_P1=0u; }while(0)

#define GPIO1PIN_PA1_INITOUT(v)      do{ bFM_GPIO_PDORA_P1=(v).bInitVal; \
                                         bFM_GPIO_DDRA_P1=1u; \
                                         bFM_GPIO_PFRA_P1=0u; }while(0)


/*---- GPIO bit NPA1 ----*/
#define GPIO1PIN_NPA1_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIRA_P1)) )

#define GPIO1PIN_NPA1_PUT(v)         ( bFM_GPIO_PDORA_P1=(uint32_t)(!(v)) )

#define GPIO1PIN_NPA1_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NPA1_INITIN(v) \
                                                        : GPIO1PIN_NPA1_INITOUT(v) )

#define GPIO1PIN_NPA1_INITIN(v)      do{ bFM_GPIO_PCRA_P1=(v).bPullup; \
                                         bFM_GPIO_DDRA_P1=0u; \
                                         bFM_GPIO_PFRA_P1=0u; }while(0)

#define GPIO1PIN_NPA1_INITOUT(v)     do{ bFM_GPIO_PDORA_P1=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDRA_P1=1u; \
                                         bFM_GPIO_PFRA_P1=0u; }while(0)


/*---- GPIO bit PA2 ----*/

#define GPIO1PIN_PA2_GET             ( bFM_GPIO_PDIRA_P2 )

#define GPIO1PIN_PA2_PUT(v)          ( bFM_GPIO_PDORA_P2=(v) )

#define GPIO1PIN_PA2_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_PA2_INITIN(v) \
                                                        : GPIO1PIN_PA2_INITOUT(v) )

#define GPIO1PIN_PA2_INITIN(v)       do{ bFM_GPIO_PCRA_P2=(v).bPullup; \
                                         bFM_GPIO_DDRA_P2=0u; \
                                         bFM_GPIO_PFRA_P2=0u; }while(0)

#define GPIO1PIN_PA2_INITOUT(v)      do{ bFM_GPIO_PDORA_P2=(v).bInitVal; \
                                         bFM_GPIO_DDRA_P2=1u; \
                                         bFM_GPIO_PFRA_P2=0u; }while(0)


/*---- GPIO bit NPA2 ----*/
#define GPIO1PIN_NPA2_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIRA_P2)) )

#define GPIO1PIN_NPA2_PUT(v)         ( bFM_GPIO_PDORA_P2=(uint32_t)(!(v)) )

#define GPIO1PIN_NPA2_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NPA2_INITIN(v) \
                                                        : GPIO1PIN_NPA2_INITOUT(v) )

#define GPIO1PIN_NPA2_INITIN(v)      do{ bFM_GPIO_PCRA_P2=(v).bPullup; \
                                         bFM_GPIO_DDRA_P2=0u; \
                                         bFM_GPIO_PFRA_P2=0u; }while(0)

#define GPIO1PIN_NPA2_INITOUT(v)     do{ bFM_GPIO_PDORA_P2=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDRA_P2=1u; \
                                         bFM_GPIO_PFRA_P2=0u; }while(0)


/*---- GPIO bit PA3 ----*/

#define GPIO1PIN_PA3_GET             ( bFM_GPIO_PDIRA_P3 )

#define GPIO1PIN_PA3_PUT(v)          ( bFM_GPIO_PDORA_P3=(v) )

#define GPIO1PIN_PA3_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_PA3_INITIN(v) \
                                                        : GPIO1PIN_PA3_INITOUT(v) )

#define GPIO1PIN_PA3_INITIN(v)       do{ bFM_GPIO_PCRA_P3=(v).bPullup; \
                                         bFM_GPIO_DDRA_P3=0u; \
                                         bFM_GPIO_PFRA_P3=0u; }while(0)

#define GPIO1PIN_PA3_INITOUT(v)      do{ bFM_GPIO_PDORA_P3=(v).bInitVal; \
                                         bFM_GPIO_DDRA_P3=1u; \
                                         bFM_GPIO_PFRA_P3=0u; }while(0)


/*---- GPIO bit NPA3 ----*/
#define GPIO1PIN_NPA3_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIRA_P3)) )

#define GPIO1PIN_NPA3_PUT(v)         ( bFM_GPIO_PDORA_P3=(uint32_t)(!(v)) )

#define GPIO1PIN_NPA3_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NPA3_INITIN(v) \
                                                        : GPIO1PIN_NPA3_INITOUT(v) )

#define GPIO1PIN_NPA3_INITIN(v)      do{ bFM_GPIO_PCRA_P3=(v).bPullup; \
                                         bFM_GPIO_DDRA_P3=0u; \
                                         bFM_GPIO_PFRA_P3=0u; }while(0)

#define GPIO1PIN_NPA3_INITOUT(v)     do{ bFM_GPIO_PDORA_P3=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDRA_P3=1u; \
                                         bFM_GPIO_PFRA_P3=0u; }while(0)


/*---- GPIO bit PA4 ----*/

#define GPIO1PIN_PA4_GET             ( bFM_GPIO_PDIRA_P4 )

#define GPIO1PIN_PA4_PUT(v)          ( bFM_GPIO_PDORA_P4=(v) )

#define GPIO1PIN_PA4_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_PA4_INITIN(v) \
                                                        : GPIO1PIN_PA4_INITOUT(v) )

#define GPIO1PIN_PA4_INITIN(v)       do{ bFM_GPIO_PCRA_P4=(v).bPullup; \
                                         bFM_GPIO_DDRA_P4=0u; \
                                         bFM_GPIO_PFRA_P4=0u; }while(0)

#define GPIO1PIN_PA4_INITOUT(v)      do{ bFM_GPIO_PDORA_P4=(v).bInitVal; \
                                         bFM_GPIO_DDRA_P4=1u; \
                                         bFM_GPIO_PFRA_P4=0u; }while(0)


/*---- GPIO bit NPA4 ----*/
#define GPIO1PIN_NPA4_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIRA_P4)) )

#define GPIO1PIN_NPA4_PUT(v)         ( bFM_GPIO_PDORA_P4=(uint32_t)(!(v)) )

#define GPIO1PIN_NPA4_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NPA4_INITIN(v) \
                                                        : GPIO1PIN_NPA4_INITOUT(v) )

#define GPIO1PIN_NPA4_INITIN(v)      do{ bFM_GPIO_PCRA_P4=(v).bPullup; \
                                         bFM_GPIO_DDRA_P4=0u; \
                                         bFM_GPIO_PFRA_P4=0u; }while(0)

#define GPIO1PIN_NPA4_INITOUT(v)     do{ bFM_GPIO_PDORA_P4=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDRA_P4=1u; \
                                         bFM_GPIO_PFRA_P4=0u; }while(0)


/*---- GPIO bit PA5 ----*/

#define GPIO1PIN_PA5_GET             ( bFM_GPIO_PDIRA_P5 )

#define GPIO1PIN_PA5_PUT(v)          ( bFM_GPIO_PDORA_P5=(v) )

#define GPIO1PIN_PA5_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_PA5_INITIN(v) \
                                                        : GPIO1PIN_PA5_INITOUT(v) )

#define GPIO1PIN_PA5_INITIN(v)       do{ bFM_GPIO_PCRA_P5=(v).bPullup; \
                                         bFM_GPIO_DDRA_P5=0u; \
                                         bFM_GPIO_PFRA_P5=0u; }while(0)

#define GPIO1PIN_PA5_INITOUT(v)      do{ bFM_GPIO_PDORA_P5=(v).bInitVal; \
                                         bFM_GPIO_DDRA_P5=1u; \
                                         bFM_GPIO_PFRA_P5=0u; }while(0)


/*---- GPIO bit NPA5 ----*/
#define GPIO1PIN_NPA5_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIRA_P5)) )

#define GPIO1PIN_NPA5_PUT(v)         ( bFM_GPIO_PDORA_P5=(uint32_t)(!(v)) )

#define GPIO1PIN_NPA5_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NPA5_INITIN(v) \
                                                        : GPIO1PIN_NPA5_INITOUT(v) )

#define GPIO1PIN_NPA5_INITIN(v)      do{ bFM_GPIO_PCRA_P5=(v).bPullup; \
                                         bFM_GPIO_DDRA_P5=0u; \
                                         bFM_GPIO_PFRA_P5=0u; }while(0)

#define GPIO1PIN_NPA5_INITOUT(v)     do{ bFM_GPIO_PDORA_P5=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDRA_P5=1u; \
                                         bFM_GPIO_PFRA_P5=0u; }while(0)


/*---- GPIO bit PC0 ----*/

#define GPIO1PIN_PC0_GET             ( bFM_GPIO_PDIRC_P0 )

#define GPIO1PIN_PC0_PUT(v)          ( bFM_GPIO_PDORC_P0=(v) )

#define GPIO1PIN_PC0_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_PC0_INITIN(v) \
                                                        : GPIO1PIN_PC0_INITOUT(v) )

#define GPIO1PIN_PC0_INITIN(v)       do{ bFM_GPIO_PCRC_P0=(v).bPullup; \
                                         bFM_GPIO_DDRC_P0=0u; \
                                         bFM_GPIO_PFRC_P0=0u; }while(0)

#define GPIO1PIN_PC0_INITOUT(v)      do{ bFM_GPIO_PDORC_P0=(v).bInitVal; \
                                         bFM_GPIO_DDRC_P0=1u; \
                                         bFM_GPIO_PFRC_P0=0u; }while(0)


/*---- GPIO bit NPC0 ----*/
#define GPIO1PIN_NPC0_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIRC_P0)) )

#define GPIO1PIN_NPC0_PUT(v)         ( bFM_GPIO_PDORC_P0=(uint32_t)(!(v)) )

#define GPIO1PIN_NPC0_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NPC0_INITIN(v) \
                                                        : GPIO1PIN_NPC0_INITOUT(v) )

#define GPIO1PIN_NPC0_INITIN(v)      do{ bFM_GPIO_PCRC_P0=(v).bPullup; \
                                         bFM_GPIO_DDRC_P0=0u; \
                                         bFM_GPIO_PFRC_P0=0u; }while(0)

#define GPIO1PIN_NPC0_INITOUT(v)     do{ bFM_GPIO_PDORC_P0=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDRC_P0=1u; \
                                         bFM_GPIO_PFRC_P0=0u; }while(0)


/*---- GPIO bit PC1 ----*/

#define GPIO1PIN_PC1_GET             ( bFM_GPIO_PDIRC_P1 )

#define GPIO1PIN_PC1_PUT(v)          ( bFM_GPIO_PDORC_P1=(v) )

#define GPIO1PIN_PC1_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_PC1_INITIN(v) \
                                                        : GPIO1PIN_PC1_INITOUT(v) )

#define GPIO1PIN_PC1_INITIN(v)       do{ bFM_GPIO_PCRC_P1=(v).bPullup; \
                                         bFM_GPIO_DDRC_P1=0u; \
                                         bFM_GPIO_PFRC_P1=0u; }while(0)

#define GPIO1PIN_PC1_INITOUT(v)      do{ bFM_GPIO_PDORC_P1=(v).bInitVal; \
                                         bFM_GPIO_DDRC_P1=1u; \
                                         bFM_GPIO_PFRC_P1=0u; }while(0)


/*---- GPIO bit NPC1 ----*/
#define GPIO1PIN_NPC1_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIRC_P1)) )

#define GPIO1PIN_NPC1_PUT(v)         ( bFM_GPIO_PDORC_P1=(uint32_t)(!(v)) )

#define GPIO1PIN_NPC1_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NPC1_INITIN(v) \
                                                        : GPIO1PIN_NPC1_INITOUT(v) )

#define GPIO1PIN_NPC1_INITIN(v)      do{ bFM_GPIO_PCRC_P1=(v).bPullup; \
                                         bFM_GPIO_DDRC_P1=0u; \
                                         bFM_GPIO_PFRC_P1=0u; }while(0)

#define GPIO1PIN_NPC1_INITOUT(v)     do{ bFM_GPIO_PDORC_P1=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDRC_P1=1u; \
                                         bFM_GPIO_PFRC_P1=0u; }while(0)


/*---- GPIO bit PC2 ----*/

#define GPIO1PIN_PC2_GET             ( bFM_GPIO_PDIRC_P2 )

#define GPIO1PIN_PC2_PUT(v)          ( bFM_GPIO_PDORC_P2=(v) )

#define GPIO1PIN_PC2_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_PC2_INITIN(v) \
                                                        : GPIO1PIN_PC2_INITOUT(v) )

#define GPIO1PIN_PC2_INITIN(v)       do{ bFM_GPIO_PCRC_P2=(v).bPullup; \
                                         bFM_GPIO_DDRC_P2=0u; \
                                         bFM_GPIO_PFRC_P2=0u; }while(0)

#define GPIO1PIN_PC2_INITOUT(v)      do{ bFM_GPIO_PDORC_P2=(v).bInitVal; \
                                         bFM_GPIO_DDRC_P2=1u; \
                                         bFM_GPIO_PFRC_P2=0u; }while(0)


/*---- GPIO bit NPC2 ----*/
#define GPIO1PIN_NPC2_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIRC_P2)) )

#define GPIO1PIN_NPC2_PUT(v)         ( bFM_GPIO_PDORC_P2=(uint32_t)(!(v)) )

#define GPIO1PIN_NPC2_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NPC2_INITIN(v) \
                                                        : GPIO1PIN_NPC2_INITOUT(v) )

#define GPIO1PIN_NPC2_INITIN(v)      do{ bFM_GPIO_PCRC_P2=(v).bPullup; \
                                         bFM_GPIO_DDRC_P2=0u; \
                                         bFM_GPIO_PFRC_P2=0u; }while(0)

#define GPIO1PIN_NPC2_INITOUT(v)     do{ bFM_GPIO_PDORC_P2=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDRC_P2=1u; \
                                         bFM_GPIO_PFRC_P2=0u; }while(0)


/*---- GPIO bit PC3 ----*/

#define GPIO1PIN_PC3_GET             ( bFM_GPIO_PDIRC_P3 )

#define GPIO1PIN_PC3_PUT(v)          ( bFM_GPIO_PDORC_P3=(v) )

#define GPIO1PIN_PC3_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_PC3_INITIN(v) \
                                                        : GPIO1PIN_PC3_INITOUT(v) )

#define GPIO1PIN_PC3_INITIN(v)       do{ bFM_GPIO_PCRC_P3=(v).bPullup; \
                                         bFM_GPIO_DDRC_P3=0u; \
                                         bFM_GPIO_PFRC_P3=0u; }while(0)

#define GPIO1PIN_PC3_INITOUT(v)      do{ bFM_GPIO_PDORC_P3=(v).bInitVal; \
                                         bFM_GPIO_DDRC_P3=1u; \
                                         bFM_GPIO_PFRC_P3=0u; }while(0)


/*---- GPIO bit NPC3 ----*/
#define GPIO1PIN_NPC3_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIRC_P3)) )

#define GPIO1PIN_NPC3_PUT(v)         ( bFM_GPIO_PDORC_P3=(uint32_t)(!(v)) )

#define GPIO1PIN_NPC3_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NPC3_INITIN(v) \
                                                        : GPIO1PIN_NPC3_INITOUT(v) )

#define GPIO1PIN_NPC3_INITIN(v)      do{ bFM_GPIO_PCRC_P3=(v).bPullup; \
                                         bFM_GPIO_DDRC_P3=0u; \
                                         bFM_GPIO_PFRC_P3=0u; }while(0)

#define GPIO1PIN_NPC3_INITOUT(v)     do{ bFM_GPIO_PDORC_P3=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDRC_P3=1u; \
                                         bFM_GPIO_PFRC_P3=0u; }while(0)


/*---- GPIO bit PC4 ----*/

#define GPIO1PIN_PC4_GET             ( bFM_GPIO_PDIRC_P4 )

#define GPIO1PIN_PC4_PUT(v)          ( bFM_GPIO_PDORC_P4=(v) )

#define GPIO1PIN_PC4_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_PC4_INITIN(v) \
                                                        : GPIO1PIN_PC4_INITOUT(v) )

#define GPIO1PIN_PC4_INITIN(v)       do{ bFM_GPIO_PCRC_P4=(v).bPullup; \
                                         bFM_GPIO_DDRC_P4=0u; \
                                         bFM_GPIO_PFRC_P4=0u; }while(0)

#define GPIO1PIN_PC4_INITOUT(v)      do{ bFM_GPIO_PDORC_P4=(v).bInitVal; \
                                         bFM_GPIO_DDRC_P4=1u; \
                                         bFM_GPIO_PFRC_P4=0u; }while(0)


/*---- GPIO bit NPC4 ----*/
#define GPIO1PIN_NPC4_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIRC_P4)) )

#define GPIO1PIN_NPC4_PUT(v)         ( bFM_GPIO_PDORC_P4=(uint32_t)(!(v)) )

#define GPIO1PIN_NPC4_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NPC4_INITIN(v) \
                                                        : GPIO1PIN_NPC4_INITOUT(v) )

#define GPIO1PIN_NPC4_INITIN(v)      do{ bFM_GPIO_PCRC_P4=(v).bPullup; \
                                         bFM_GPIO_DDRC_P4=0u; \
                                         bFM_GPIO_PFRC_P4=0u; }while(0)

#define GPIO1PIN_NPC4_INITOUT(v)     do{ bFM_GPIO_PDORC_P4=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDRC_P4=1u; \
                                         bFM_GPIO_PFRC_P4=0u; }while(0)


/*---- GPIO bit PC5 ----*/

#define GPIO1PIN_PC5_GET             ( bFM_GPIO_PDIRC_P5 )

#define GPIO1PIN_PC5_PUT(v)          ( bFM_GPIO_PDORC_P5=(v) )

#define GPIO1PIN_PC5_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_PC5_INITIN(v) \
                                                        : GPIO1PIN_PC5_INITOUT(v) )

#define GPIO1PIN_PC5_INITIN(v)       do{ bFM_GPIO_PCRC_P5=(v).bPullup; \
                                         bFM_GPIO_DDRC_P5=0u; \
                                         bFM_GPIO_PFRC_P5=0u; }while(0)

#define GPIO1PIN_PC5_INITOUT(v)      do{ bFM_GPIO_PDORC_P5=(v).bInitVal; \
                                         bFM_GPIO_DDRC_P5=1u; \
                                         bFM_GPIO_PFRC_P5=0u; }while(0)


/*---- GPIO bit NPC5 ----*/
#define GPIO1PIN_NPC5_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIRC_P5)) )

#define GPIO1PIN_NPC5_PUT(v)         ( bFM_GPIO_PDORC_P5=(uint32_t)(!(v)) )

#define GPIO1PIN_NPC5_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NPC5_INITIN(v) \
                                                        : GPIO1PIN_NPC5_INITOUT(v) )

#define GPIO1PIN_NPC5_INITIN(v)      do{ bFM_GPIO_PCRC_P5=(v).bPullup; \
                                         bFM_GPIO_DDRC_P5=0u; \
                                         bFM_GPIO_PFRC_P5=0u; }while(0)

#define GPIO1PIN_NPC5_INITOUT(v)     do{ bFM_GPIO_PDORC_P5=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDRC_P5=1u; \
                                         bFM_GPIO_PFRC_P5=0u; }while(0)


/*---- GPIO bit PC6 ----*/

#define GPIO1PIN_PC6_GET             ( bFM_GPIO_PDIRC_P6 )

#define GPIO1PIN_PC6_PUT(v)          ( bFM_GPIO_PDORC_P6=(v) )

#define GPIO1PIN_PC6_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_PC6_INITIN(v) \
                                                        : GPIO1PIN_PC6_INITOUT(v) )

#define GPIO1PIN_PC6_INITIN(v)       do{ bFM_GPIO_PCRC_P6=(v).bPullup; \
                                         bFM_GPIO_DDRC_P6=0u; \
                                         bFM_GPIO_PFRC_P6=0u; }while(0)

#define GPIO1PIN_PC6_INITOUT(v)      do{ bFM_GPIO_PDORC_P6=(v).bInitVal; \
                                         bFM_GPIO_DDRC_P6=1u; \
                                         bFM_GPIO_PFRC_P6=0u; }while(0)


/*---- GPIO bit NPC6 ----*/
#define GPIO1PIN_NPC6_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIRC_P6)) )

#define GPIO1PIN_NPC6_PUT(v)         ( bFM_GPIO_PDORC_P6=(uint32_t)(!(v)) )

#define GPIO1PIN_NPC6_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NPC6_INITIN(v) \
                                                        : GPIO1PIN_NPC6_INITOUT(v) )

#define GPIO1PIN_NPC6_INITIN(v)      do{ bFM_GPIO_PCRC_P6=(v).bPullup; \
                                         bFM_GPIO_DDRC_P6=0u; \
                                         bFM_GPIO_PFRC_P6=0u; }while(0)

#define GPIO1PIN_NPC6_INITOUT(v)     do{ bFM_GPIO_PDORC_P6=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDRC_P6=1u; \
                                         bFM_GPIO_PFRC_P6=0u; }while(0)


/*---- GPIO bit PC7 ----*/

#define GPIO1PIN_PC7_GET             ( bFM_GPIO_PDIRC_P7 )

#define GPIO1PIN_PC7_PUT(v)          ( bFM_GPIO_PDORC_P7=(v) )

#define GPIO1PIN_PC7_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_PC7_INITIN(v) \
                                                        : GPIO1PIN_PC7_INITOUT(v) )

#define GPIO1PIN_PC7_INITIN(v)       do{ bFM_GPIO_PCRC_P7=(v).bPullup; \
                                         bFM_GPIO_DDRC_P7=0u; \
                                         bFM_GPIO_PFRC_P7=0u; }while(0)

#define GPIO1PIN_PC7_INITOUT(v)      do{ bFM_GPIO_PDORC_P7=(v).bInitVal; \
                                         bFM_GPIO_DDRC_P7=1u; \
                                         bFM_GPIO_PFRC_P7=0u; }while(0)


/*---- GPIO bit NPC7 ----*/
#define GPIO1PIN_NPC7_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIRC_P7)) )

#define GPIO1PIN_NPC7_PUT(v)         ( bFM_GPIO_PDORC_P7=(uint32_t)(!(v)) )

#define GPIO1PIN_NPC7_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NPC7_INITIN(v) \
                                                        : GPIO1PIN_NPC7_INITOUT(v) )

#define GPIO1PIN_NPC7_INITIN(v)      do{ bFM_GPIO_PCRC_P7=(v).bPullup; \
                                         bFM_GPIO_DDRC_P7=0u; \
                                         bFM_GPIO_PFRC_P7=0u; }while(0)

#define GPIO1PIN_NPC7_INITOUT(v)     do{ bFM_GPIO_PDORC_P7=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDRC_P7=1u; \
                                         bFM_GPIO_PFRC_P7=0u; }while(0)


/*---- GPIO bit PC8 ----*/

#define GPIO1PIN_PC8_GET             ( bFM_GPIO_PDIRC_P8 )

#define GPIO1PIN_PC8_PUT(v)          ( bFM_GPIO_PDORC_P8=(v) )

#define GPIO1PIN_PC8_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_PC8_INITIN(v) \
                                                        : GPIO1PIN_PC8_INITOUT(v) )

#define GPIO1PIN_PC8_INITIN(v)       do{ bFM_GPIO_PCRC_P8=(v).bPullup; \
                                         bFM_GPIO_DDRC_P8=0u; \
                                         bFM_GPIO_PFRC_P8=0u; }while(0)

#define GPIO1PIN_PC8_INITOUT(v)      do{ bFM_GPIO_PDORC_P8=(v).bInitVal; \
                                         bFM_GPIO_DDRC_P8=1u; \
                                         bFM_GPIO_PFRC_P8=0u; }while(0)


/*---- GPIO bit NPC8 ----*/
#define GPIO1PIN_NPC8_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIRC_P8)) )

#define GPIO1PIN_NPC8_PUT(v)         ( bFM_GPIO_PDORC_P8=(uint32_t)(!(v)) )

#define GPIO1PIN_NPC8_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NPC8_INITIN(v) \
                                                        : GPIO1PIN_NPC8_INITOUT(v) )

#define GPIO1PIN_NPC8_INITIN(v)      do{ bFM_GPIO_PCRC_P8=(v).bPullup; \
                                         bFM_GPIO_DDRC_P8=0u; \
                                         bFM_GPIO_PFRC_P8=0u; }while(0)

#define GPIO1PIN_NPC8_INITOUT(v)     do{ bFM_GPIO_PDORC_P8=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDRC_P8=1u; \
                                         bFM_GPIO_PFRC_P8=0u; }while(0)


/*---- GPIO bit PC9 ----*/

#define GPIO1PIN_PC9_GET             ( bFM_GPIO_PDIRC_P9 )

#define GPIO1PIN_PC9_PUT(v)          ( bFM_GPIO_PDORC_P9=(v) )

#define GPIO1PIN_PC9_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_PC9_INITIN(v) \
                                                        : GPIO1PIN_PC9_INITOUT(v) )

#define GPIO1PIN_PC9_INITIN(v)       do{ bFM_GPIO_PCRC_P9=(v).bPullup; \
                                         bFM_GPIO_DDRC_P9=0u; \
                                         bFM_GPIO_PFRC_P9=0u; }while(0)

#define GPIO1PIN_PC9_INITOUT(v)      do{ bFM_GPIO_PDORC_P9=(v).bInitVal; \
                                         bFM_GPIO_DDRC_P9=1u; \
                                         bFM_GPIO_PFRC_P9=0u; }while(0)


/*---- GPIO bit NPC9 ----*/
#define GPIO1PIN_NPC9_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIRC_P9)) )

#define GPIO1PIN_NPC9_PUT(v)         ( bFM_GPIO_PDORC_P9=(uint32_t)(!(v)) )

#define GPIO1PIN_NPC9_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NPC9_INITIN(v) \
                                                        : GPIO1PIN_NPC9_INITOUT(v) )

#define GPIO1PIN_NPC9_INITIN(v)      do{ bFM_GPIO_PCRC_P9=(v).bPullup; \
                                         bFM_GPIO_DDRC_P9=0u; \
                                         bFM_GPIO_PFRC_P9=0u; }while(0)

#define GPIO1PIN_NPC9_INITOUT(v)     do{ bFM_GPIO_PDORC_P9=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDRC_P9=1u; \
                                         bFM_GPIO_PFRC_P9=0u; }while(0)


/*---- GPIO bit PCA ----*/

#define GPIO1PIN_PCA_GET             ( bFM_GPIO_PDIRC_PA )

#define GPIO1PIN_PCA_PUT(v)          ( bFM_GPIO_PDORC_PA=(v) )

#define GPIO1PIN_PCA_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_PCA_INITIN(v) \
                                                        : GPIO1PIN_PCA_INITOUT(v) )

#define GPIO1PIN_PCA_INITIN(v)       do{ bFM_GPIO_PCRC_PA=(v).bPullup; \
                                         bFM_GPIO_DDRC_PA=0u; \
                                         bFM_GPIO_PFRC_PA=0u; }while(0)

#define GPIO1PIN_PCA_INITOUT(v)      do{ bFM_GPIO_PDORC_PA=(v).bInitVal; \
                                         bFM_GPIO_DDRC_PA=1u; \
                                         bFM_GPIO_PFRC_PA=0u; }while(0)


/*---- GPIO bit NPCA ----*/
#define GPIO1PIN_NPCA_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIRC_PA)) )

#define GPIO1PIN_NPCA_PUT(v)         ( bFM_GPIO_PDORC_PA=(uint32_t)(!(v)) )

#define GPIO1PIN_NPCA_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NPCA_INITIN(v) \
                                                        : GPIO1PIN_NPCA_INITOUT(v) )

#define GPIO1PIN_NPCA_INITIN(v)      do{ bFM_GPIO_PCRC_PA=(v).bPullup; \
                                         bFM_GPIO_DDRC_PA=0u; \
                                         bFM_GPIO_PFRC_PA=0u; }while(0)

#define GPIO1PIN_NPCA_INITOUT(v)     do{ bFM_GPIO_PDORC_PA=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDRC_PA=1u; \
                                         bFM_GPIO_PFRC_PA=0u; }while(0)


/*---- GPIO bit PCB ----*/

#define GPIO1PIN_PCB_GET             ( bFM_GPIO_PDIRC_PB )

#define GPIO1PIN_PCB_PUT(v)          ( bFM_GPIO_PDORC_PB=(v) )

#define GPIO1PIN_PCB_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_PCB_INITIN(v) \
                                                        : GPIO1PIN_PCB_INITOUT(v) )

#define GPIO1PIN_PCB_INITIN(v)       do{ bFM_GPIO_PCRC_PB=(v).bPullup; \
                                         bFM_GPIO_DDRC_PB=0u; \
                                         bFM_GPIO_PFRC_PB=0u; }while(0)

#define GPIO1PIN_PCB_INITOUT(v)      do{ bFM_GPIO_PDORC_PB=(v).bInitVal; \
                                         bFM_GPIO_DDRC_PB=1u; \
                                         bFM_GPIO_PFRC_PB=0u; }while(0)


/*---- GPIO bit NPCB ----*/
#define GPIO1PIN_NPCB_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIRC_PB)) )

#define GPIO1PIN_NPCB_PUT(v)         ( bFM_GPIO_PDORC_PB=(uint32_t)(!(v)) )

#define GPIO1PIN_NPCB_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NPCB_INITIN(v) \
                                                        : GPIO1PIN_NPCB_INITOUT(v) )

#define GPIO1PIN_NPCB_INITIN(v)      do{ bFM_GPIO_PCRC_PB=(v).bPullup; \
                                         bFM_GPIO_DDRC_PB=0u; \
                                         bFM_GPIO_PFRC_PB=0u; }while(0)

#define GPIO1PIN_NPCB_INITOUT(v)     do{ bFM_GPIO_PDORC_PB=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDRC_PB=1u; \
                                         bFM_GPIO_PFRC_PB=0u; }while(0)


/*---- GPIO bit PCC ----*/

#define GPIO1PIN_PCC_GET             ( bFM_GPIO_PDIRC_PC )

#define GPIO1PIN_PCC_PUT(v)          ( bFM_GPIO_PDORC_PC=(v) )

#define GPIO1PIN_PCC_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_PCC_INITIN(v) \
                                                        : GPIO1PIN_PCC_INITOUT(v) )

#define GPIO1PIN_PCC_INITIN(v)       do{ bFM_GPIO_PCRC_PC=(v).bPullup; \
                                         bFM_GPIO_DDRC_PC=0u; \
                                         bFM_GPIO_PFRC_PC=0u; }while(0)

#define GPIO1PIN_PCC_INITOUT(v)      do{ bFM_GPIO_PDORC_PC=(v).bInitVal; \
                                         bFM_GPIO_DDRC_PC=1u; \
                                         bFM_GPIO_PFRC_PC=0u; }while(0)


/*---- GPIO bit NPCC ----*/
#define GPIO1PIN_NPCC_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIRC_PC)) )

#define GPIO1PIN_NPCC_PUT(v)         ( bFM_GPIO_PDORC_PC=(uint32_t)(!(v)) )

#define GPIO1PIN_NPCC_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NPCC_INITIN(v) \
                                                        : GPIO1PIN_NPCC_INITOUT(v) )

#define GPIO1PIN_NPCC_INITIN(v)      do{ bFM_GPIO_PCRC_PC=(v).bPullup; \
                                         bFM_GPIO_DDRC_PC=0u; \
                                         bFM_GPIO_PFRC_PC=0u; }while(0)

#define GPIO1PIN_NPCC_INITOUT(v)     do{ bFM_GPIO_PDORC_PC=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDRC_PC=1u; \
                                         bFM_GPIO_PFRC_PC=0u; }while(0)


/*---- GPIO bit PCD ----*/

#define GPIO1PIN_PCD_GET             ( bFM_GPIO_PDIRC_PD )

#define GPIO1PIN_PCD_PUT(v)          ( bFM_GPIO_PDORC_PD=(v) )

#define GPIO1PIN_PCD_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_PCD_INITIN(v) \
                                                        : GPIO1PIN_PCD_INITOUT(v) )

#define GPIO1PIN_PCD_INITIN(v)       do{ bFM_GPIO_PCRC_PD=(v).bPullup; \
                                         bFM_GPIO_DDRC_PD=0u; \
                                         bFM_GPIO_PFRC_PD=0u; }while(0)

#define GPIO1PIN_PCD_INITOUT(v)      do{ bFM_GPIO_PDORC_PD=(v).bInitVal; \
                                         bFM_GPIO_DDRC_PD=1u; \
                                         bFM_GPIO_PFRC_PD=0u; }while(0)


/*---- GPIO bit NPCD ----*/
#define GPIO1PIN_NPCD_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIRC_PD)) )

#define GPIO1PIN_NPCD_PUT(v)         ( bFM_GPIO_PDORC_PD=(uint32_t)(!(v)) )

#define GPIO1PIN_NPCD_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NPCD_INITIN(v) \
                                                        : GPIO1PIN_NPCD_INITOUT(v) )

#define GPIO1PIN_NPCD_INITIN(v)      do{ bFM_GPIO_PCRC_PD=(v).bPullup; \
                                         bFM_GPIO_DDRC_PD=0u; \
                                         bFM_GPIO_PFRC_PD=0u; }while(0)

#define GPIO1PIN_NPCD_INITOUT(v)     do{ bFM_GPIO_PDORC_PD=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDRC_PD=1u; \
                                         bFM_GPIO_PFRC_PD=0u; }while(0)


/*---- GPIO bit PCE ----*/

#define GPIO1PIN_PCE_GET             ( bFM_GPIO_PDIRC_PE )

#define GPIO1PIN_PCE_PUT(v)          ( bFM_GPIO_PDORC_PE=(v) )

#define GPIO1PIN_PCE_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_PCE_INITIN(v) \
                                                        : GPIO1PIN_PCE_INITOUT(v) )

#define GPIO1PIN_PCE_INITIN(v)       do{ bFM_GPIO_PCRC_PE=(v).bPullup; \
                                         bFM_GPIO_DDRC_PE=0u; \
                                         bFM_GPIO_PFRC_PE=0u; }while(0)

#define GPIO1PIN_PCE_INITOUT(v)      do{ bFM_GPIO_PDORC_PE=(v).bInitVal; \
                                         bFM_GPIO_DDRC_PE=1u; \
                                         bFM_GPIO_PFRC_PE=0u; }while(0)


/*---- GPIO bit NPCE ----*/
#define GPIO1PIN_NPCE_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIRC_PE)) )

#define GPIO1PIN_NPCE_PUT(v)         ( bFM_GPIO_PDORC_PE=(uint32_t)(!(v)) )

#define GPIO1PIN_NPCE_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NPCE_INITIN(v) \
                                                        : GPIO1PIN_NPCE_INITOUT(v) )

#define GPIO1PIN_NPCE_INITIN(v)      do{ bFM_GPIO_PCRC_PE=(v).bPullup; \
                                         bFM_GPIO_DDRC_PE=0u; \
                                         bFM_GPIO_PFRC_PE=0u; }while(0)

#define GPIO1PIN_NPCE_INITOUT(v)     do{ bFM_GPIO_PDORC_PE=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDRC_PE=1u; \
                                         bFM_GPIO_PFRC_PE=0u; }while(0)


/*---- GPIO bit PCF ----*/

#define GPIO1PIN_PCF_GET             ( bFM_GPIO_PDIRC_PF )

#define GPIO1PIN_PCF_PUT(v)          ( bFM_GPIO_PDORC_PF=(v) )

#define GPIO1PIN_PCF_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_PCF_INITIN(v) \
                                                        : GPIO1PIN_PCF_INITOUT(v) )

#define GPIO1PIN_PCF_INITIN(v)       do{ bFM_GPIO_PCRC_PF=(v).bPullup; \
                                         bFM_GPIO_DDRC_PF=0u; \
                                         bFM_GPIO_PFRC_PF=0u; }while(0)

#define GPIO1PIN_PCF_INITOUT(v)      do{ bFM_GPIO_PDORC_PF=(v).bInitVal; \
                                         bFM_GPIO_DDRC_PF=1u; \
                                         bFM_GPIO_PFRC_PF=0u; }while(0)


/*---- GPIO bit NPCF ----*/
#define GPIO1PIN_NPCF_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIRC_PF)) )

#define GPIO1PIN_NPCF_PUT(v)         ( bFM_GPIO_PDORC_PF=(uint32_t)(!(v)) )

#define GPIO1PIN_NPCF_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NPCF_INITIN(v) \
                                                        : GPIO1PIN_NPCF_INITOUT(v) )

#define GPIO1PIN_NPCF_INITIN(v)      do{ bFM_GPIO_PCRC_PF=(v).bPullup; \
                                         bFM_GPIO_DDRC_PF=0u; \
                                         bFM_GPIO_PFRC_PF=0u; }while(0)

#define GPIO1PIN_NPCF_INITOUT(v)     do{ bFM_GPIO_PDORC_PF=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDRC_PF=1u; \
                                         bFM_GPIO_PFRC_PF=0u; }while(0)


/*---- GPIO bit PD0 ----*/

#define GPIO1PIN_PD0_GET             ( bFM_GPIO_PDIRD_P0 )

#define GPIO1PIN_PD0_PUT(v)          ( bFM_GPIO_PDORD_P0=(v) )

#define GPIO1PIN_PD0_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_PD0_INITIN(v) \
                                                        : GPIO1PIN_PD0_INITOUT(v) )

#define GPIO1PIN_PD0_INITIN(v)       do{ bFM_GPIO_PCRD_P0=(v).bPullup; \
                                         bFM_GPIO_DDRD_P0=0u; \
                                         bFM_GPIO_PFRD_P0=0u; }while(0)

#define GPIO1PIN_PD0_INITOUT(v)      do{ bFM_GPIO_PDORD_P0=(v).bInitVal; \
                                         bFM_GPIO_DDRD_P0=1u; \
                                         bFM_GPIO_PFRD_P0=0u; }while(0)


/*---- GPIO bit NPD0 ----*/
#define GPIO1PIN_NPD0_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIRD_P0)) )

#define GPIO1PIN_NPD0_PUT(v)         ( bFM_GPIO_PDORD_P0=(uint32_t)(!(v)) )

#define GPIO1PIN_NPD0_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NPD0_INITIN(v) \
                                                        : GPIO1PIN_NPD0_INITOUT(v) )

#define GPIO1PIN_NPD0_INITIN(v)      do{ bFM_GPIO_PCRD_P0=(v).bPullup; \
                                         bFM_GPIO_DDRD_P0=0u; \
                                         bFM_GPIO_PFRD_P0=0u; }while(0)

#define GPIO1PIN_NPD0_INITOUT(v)     do{ bFM_GPIO_PDORD_P0=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDRD_P0=1u; \
                                         bFM_GPIO_PFRD_P0=0u; }while(0)


/*---- GPIO bit PD1 ----*/

#define GPIO1PIN_PD1_GET             ( bFM_GPIO_PDIRD_P1 )

#define GPIO1PIN_PD1_PUT(v)          ( bFM_GPIO_PDORD_P1=(v) )

#define GPIO1PIN_PD1_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_PD1_INITIN(v) \
                                                        : GPIO1PIN_PD1_INITOUT(v) )

#define GPIO1PIN_PD1_INITIN(v)       do{ bFM_GPIO_PCRD_P1=(v).bPullup; \
                                         bFM_GPIO_DDRD_P1=0u; \
                                         bFM_GPIO_PFRD_P1=0u; }while(0)

#define GPIO1PIN_PD1_INITOUT(v)      do{ bFM_GPIO_PDORD_P1=(v).bInitVal; \
                                         bFM_GPIO_DDRD_P1=1u; \
                                         bFM_GPIO_PFRD_P1=0u; }while(0)


/*---- GPIO bit NPD1 ----*/
#define GPIO1PIN_NPD1_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIRD_P1)) )

#define GPIO1PIN_NPD1_PUT(v)         ( bFM_GPIO_PDORD_P1=(uint32_t)(!(v)) )

#define GPIO1PIN_NPD1_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NPD1_INITIN(v) \
                                                        : GPIO1PIN_NPD1_INITOUT(v) )

#define GPIO1PIN_NPD1_INITIN(v)      do{ bFM_GPIO_PCRD_P1=(v).bPullup; \
                                         bFM_GPIO_DDRD_P1=0u; \
                                         bFM_GPIO_PFRD_P1=0u; }while(0)

#define GPIO1PIN_NPD1_INITOUT(v)     do{ bFM_GPIO_PDORD_P1=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDRD_P1=1u; \
                                         bFM_GPIO_PFRD_P1=0u; }while(0)


/*---- GPIO bit PD2 ----*/

#define GPIO1PIN_PD2_GET             ( bFM_GPIO_PDIRD_P2 )

#define GPIO1PIN_PD2_PUT(v)          ( bFM_GPIO_PDORD_P2=(v) )

#define GPIO1PIN_PD2_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_PD2_INITIN(v) \
                                                        : GPIO1PIN_PD2_INITOUT(v) )

#define GPIO1PIN_PD2_INITIN(v)       do{ bFM_GPIO_PCRD_P2=(v).bPullup; \
                                         bFM_GPIO_DDRD_P2=0u; \
                                         bFM_GPIO_PFRD_P2=0u; }while(0)

#define GPIO1PIN_PD2_INITOUT(v)      do{ bFM_GPIO_PDORD_P2=(v).bInitVal; \
                                         bFM_GPIO_DDRD_P2=1u; \
                                         bFM_GPIO_PFRD_P2=0u; }while(0)


/*---- GPIO bit NPD2 ----*/
#define GPIO1PIN_NPD2_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIRD_P2)) )

#define GPIO1PIN_NPD2_PUT(v)         ( bFM_GPIO_PDORD_P2=(uint32_t)(!(v)) )

#define GPIO1PIN_NPD2_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NPD2_INITIN(v) \
                                                        : GPIO1PIN_NPD2_INITOUT(v) )

#define GPIO1PIN_NPD2_INITIN(v)      do{ bFM_GPIO_PCRD_P2=(v).bPullup; \
                                         bFM_GPIO_DDRD_P2=0u; \
                                         bFM_GPIO_PFRD_P2=0u; }while(0)

#define GPIO1PIN_NPD2_INITOUT(v)     do{ bFM_GPIO_PDORD_P2=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDRD_P2=1u; \
                                         bFM_GPIO_PFRD_P2=0u; }while(0)


/*---- GPIO bit PD3 ----*/

#define GPIO1PIN_PD3_GET             ( bFM_GPIO_PDIRD_P3 )

#define GPIO1PIN_PD3_PUT(v)          ( bFM_GPIO_PDORD_P3=(v) )

#define GPIO1PIN_PD3_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_PD3_INITIN(v) \
                                                        : GPIO1PIN_PD3_INITOUT(v) )

#define GPIO1PIN_PD3_INITIN(v)       do{ bFM_GPIO_PCRD_P3=(v).bPullup; \
                                         bFM_GPIO_DDRD_P3=0u; \
                                         bFM_GPIO_PFRD_P3=0u; }while(0)

#define GPIO1PIN_PD3_INITOUT(v)      do{ bFM_GPIO_PDORD_P3=(v).bInitVal; \
                                         bFM_GPIO_DDRD_P3=1u; \
                                         bFM_GPIO_PFRD_P3=0u; }while(0)


/*---- GPIO bit NPD3 ----*/
#define GPIO1PIN_NPD3_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIRD_P3)) )

#define GPIO1PIN_NPD3_PUT(v)         ( bFM_GPIO_PDORD_P3=(uint32_t)(!(v)) )

#define GPIO1PIN_NPD3_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NPD3_INITIN(v) \
                                                        : GPIO1PIN_NPD3_INITOUT(v) )

#define GPIO1PIN_NPD3_INITIN(v)      do{ bFM_GPIO_PCRD_P3=(v).bPullup; \
                                         bFM_GPIO_DDRD_P3=0u; \
                                         bFM_GPIO_PFRD_P3=0u; }while(0)

#define GPIO1PIN_NPD3_INITOUT(v)     do{ bFM_GPIO_PDORD_P3=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDRD_P3=1u; \
                                         bFM_GPIO_PFRD_P3=0u; }while(0)


/*---- GPIO bit PE0 ----*/

#define GPIO1PIN_PE0_GET             ( bFM_GPIO_PDIRE_P0 )

#define GPIO1PIN_PE0_PUT(v)          ( bFM_GPIO_PDORE_P0=(v) )

#define GPIO1PIN_PE0_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_PE0_INITIN(v) \
                                                        : GPIO1PIN_PE0_INITOUT(v) )

#define GPIO1PIN_PE0_INITIN(v)       do{ bFM_GPIO_PCRE_P0=(v).bPullup; \
                                         bFM_GPIO_DDRE_P0=0u; \
                                         bFM_GPIO_PFRE_P0=0u; }while(0)

#define GPIO1PIN_PE0_INITOUT(v)      do{ bFM_GPIO_PDORE_P0=(v).bInitVal; \
                                         bFM_GPIO_DDRE_P0=1u; \
                                         bFM_GPIO_PFRE_P0=0u; }while(0)


/*---- GPIO bit NPE0 ----*/
#define GPIO1PIN_NPE0_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIRE_P0)) )

#define GPIO1PIN_NPE0_PUT(v)         ( bFM_GPIO_PDORE_P0=(uint32_t)(!(v)) )

#define GPIO1PIN_NPE0_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NPE0_INITIN(v) \
                                                        : GPIO1PIN_NPE0_INITOUT(v) )

#define GPIO1PIN_NPE0_INITIN(v)      do{ bFM_GPIO_PCRE_P0=(v).bPullup; \
                                         bFM_GPIO_DDRE_P0=0u; \
                                         bFM_GPIO_PFRE_P0=0u; }while(0)

#define GPIO1PIN_NPE0_INITOUT(v)     do{ bFM_GPIO_PDORE_P0=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDRE_P0=1u; \
                                         bFM_GPIO_PFRE_P0=0u; }while(0)


/*---- GPIO bit PE2 ----*/

#define GPIO1PIN_PE2_GET             ( bFM_GPIO_PDIRE_P2 )

#define GPIO1PIN_PE2_PUT(v)          ( bFM_GPIO_PDORE_P2=(v) )

#define GPIO1PIN_PE2_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_PE2_INITIN(v) \
                                                        : GPIO1PIN_PE2_INITOUT(v) )

#define GPIO1PIN_PE2_INITIN(v)       do{ bFM_GPIO_PCRE_P2=(v).bPullup; \
                                         bFM_GPIO_DDRE_P2=0u; \
                                         bFM_GPIO_SPSR_MAINXC0=0u; \
                                         bFM_GPIO_SPSR_MAINXC1=0u; \
                                         bFM_GPIO_PFRE_P2=0u; }while(0)

#define GPIO1PIN_PE2_INITOUT(v)      do{ bFM_GPIO_PDORE_P2=(v).bInitVal; \
                                         bFM_GPIO_DDRE_P2=1u; \
                                         bFM_GPIO_SPSR_MAINXC0=0u; \
                                         bFM_GPIO_SPSR_MAINXC1=0u; \
                                         bFM_GPIO_PFRE_P2=0u; }while(0)


/*---- GPIO bit NPE2 ----*/
#define GPIO1PIN_NPE2_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIRE_P2)) )

#define GPIO1PIN_NPE2_PUT(v)         ( bFM_GPIO_PDORE_P2=(uint32_t)(!(v)) )

#define GPIO1PIN_NPE2_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NPE2_INITIN(v) \
                                                        : GPIO1PIN_NPE2_INITOUT(v) )

#define GPIO1PIN_NPE2_INITIN(v)      do{ bFM_GPIO_PCRE_P2=(v).bPullup; \
                                         bFM_GPIO_DDRE_P2=0u; \
                                         bFM_GPIO_SPSR_MAINXC0=0u; \
                                         bFM_GPIO_SPSR_MAINXC1=0u; \
                                         bFM_GPIO_PFRE_P2=0u; }while(0)

#define GPIO1PIN_NPE2_INITOUT(v)     do{ bFM_GPIO_PDORE_P2=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDRE_P2=1u; \
                                         bFM_GPIO_SPSR_MAINXC0=0u; \
                                         bFM_GPIO_SPSR_MAINXC1=0u; \
                                         bFM_GPIO_PFRE_P2=0u; }while(0)


/*---- GPIO bit PE3 ----*/

#define GPIO1PIN_PE3_GET             ( bFM_GPIO_PDIRE_P3 )

#define GPIO1PIN_PE3_PUT(v)          ( bFM_GPIO_PDORE_P3=(v) )

#define GPIO1PIN_PE3_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_PE3_INITIN(v) \
                                                        : GPIO1PIN_PE3_INITOUT(v) )

#define GPIO1PIN_PE3_INITIN(v)       do{ bFM_GPIO_PCRE_P3=(v).bPullup; \
                                         bFM_GPIO_DDRE_P3=0u; \
                                         bFM_GPIO_SPSR_MAINXC0=0u; \
                                         bFM_GPIO_SPSR_MAINXC1=0u; \
                                         bFM_GPIO_PFRE_P3=0u; }while(0)

#define GPIO1PIN_PE3_INITOUT(v)      do{ bFM_GPIO_PDORE_P3=(v).bInitVal; \
                                         bFM_GPIO_DDRE_P3=1u; \
                                         bFM_GPIO_SPSR_MAINXC0=0u; \
                                         bFM_GPIO_SPSR_MAINXC1=0u; \
                                         bFM_GPIO_PFRE_P3=0u; }while(0)


/*---- GPIO bit NPE3 ----*/
#define GPIO1PIN_NPE3_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIRE_P3)) )

#define GPIO1PIN_NPE3_PUT(v)         ( bFM_GPIO_PDORE_P3=(uint32_t)(!(v)) )

#define GPIO1PIN_NPE3_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NPE3_INITIN(v) \
                                                        : GPIO1PIN_NPE3_INITOUT(v) )

#define GPIO1PIN_NPE3_INITIN(v)      do{ bFM_GPIO_PCRE_P3=(v).bPullup; \
                                         bFM_GPIO_DDRE_P3=0u; \
                                         bFM_GPIO_SPSR_MAINXC0=0u; \
                                         bFM_GPIO_SPSR_MAINXC1=0u; \
                                         bFM_GPIO_PFRE_P3=0u; }while(0)

#define GPIO1PIN_NPE3_INITOUT(v)     do{ bFM_GPIO_PDORE_P3=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDRE_P3=1u; \
                                         bFM_GPIO_SPSR_MAINXC0=0u; \
                                         bFM_GPIO_SPSR_MAINXC1=0u; \
                                         bFM_GPIO_PFRE_P3=0u; }while(0)


/*---- GPIO bit PF5 ----*/

#define GPIO1PIN_PF5_GET             ( bFM_GPIO_PDIRF_P5 )

#define GPIO1PIN_PF5_PUT(v)          ( bFM_GPIO_PDORF_P5=(v) )

#define GPIO1PIN_PF5_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_PF5_INITIN(v) \
                                                        : GPIO1PIN_PF5_INITOUT(v) )

#define GPIO1PIN_PF5_INITIN(v)       do{ bFM_GPIO_PCRF_P5=(v).bPullup; \
                                         bFM_GPIO_DDRF_P5=0u; \
                                         bFM_GPIO_PFRF_P5=0u; }while(0)

#define GPIO1PIN_PF5_INITOUT(v)      do{ bFM_GPIO_PDORF_P5=(v).bInitVal; \
                                         bFM_GPIO_DDRF_P5=1u; \
                                         bFM_GPIO_PFRF_P5=0u; }while(0)


/*---- GPIO bit NPF5 ----*/
#define GPIO1PIN_NPF5_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIRF_P5)) )

#define GPIO1PIN_NPF5_PUT(v)         ( bFM_GPIO_PDORF_P5=(uint32_t)(!(v)) )

#define GPIO1PIN_NPF5_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NPF5_INITIN(v) \
                                                        : GPIO1PIN_NPF5_INITOUT(v) )

#define GPIO1PIN_NPF5_INITIN(v)      do{ bFM_GPIO_PCRF_P5=(v).bPullup; \
                                         bFM_GPIO_DDRF_P5=0u; \
                                         bFM_GPIO_PFRF_P5=0u; }while(0)

#define GPIO1PIN_NPF5_INITOUT(v)     do{ bFM_GPIO_PDORF_P5=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDRF_P5=1u; \
                                         bFM_GPIO_PFRF_P5=0u; }while(0)


/*---- GPIO bit PF6 ----*/

#define GPIO1PIN_PF6_GET             ( bFM_GPIO_PDIRF_P6 )

#define GPIO1PIN_PF6_PUT(v)          ( bFM_GPIO_PDORF_P6=(v) )

#define GPIO1PIN_PF6_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_PF6_INITIN(v) \
                                                        : GPIO1PIN_PF6_INITOUT(v) )

#define GPIO1PIN_PF6_INITIN(v)       do{ bFM_GPIO_PCRF_P6=(v).bPullup; \
                                         bFM_GPIO_DDRF_P6=0u; \
                                         bFM_GPIO_PFRF_P6=0u; }while(0)

#define GPIO1PIN_PF6_INITOUT(v)      do{ bFM_GPIO_PDORF_P6=(v).bInitVal; \
                                         bFM_GPIO_DDRF_P6=1u; \
                                         bFM_GPIO_PFRF_P6=0u; }while(0)


/*---- GPIO bit NPF6 ----*/
#define GPIO1PIN_NPF6_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIRF_P6)) )

#define GPIO1PIN_NPF6_PUT(v)         ( bFM_GPIO_PDORF_P6=(uint32_t)(!(v)) )

#define GPIO1PIN_NPF6_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NPF6_INITIN(v) \
                                                        : GPIO1PIN_NPF6_INITOUT(v) )

#define GPIO1PIN_NPF6_INITIN(v)      do{ bFM_GPIO_PCRF_P6=(v).bPullup; \
                                         bFM_GPIO_DDRF_P6=0u; \
                                         bFM_GPIO_PFRF_P6=0u; }while(0)

#define GPIO1PIN_NPF6_INITOUT(v)     do{ bFM_GPIO_PDORF_P6=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDRF_P6=1u; \
                                         bFM_GPIO_PFRF_P6=0u; }while(0)



/******************************************************************************
   PIN RELOCATION
*******************************************************************************/

/*--- ADTG_0 for ADC#0 ---*/
#define SetPinFunc_ADTG_0_ADC0(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 12u, 4u, 0x01u ); \
                                                 bFM_GPIO_PFR0_P7 = 1u; }while(0)

/*--- ADTG_0 for ADC#1 ---*/
#define SetPinFunc_ADTG_0_ADC1(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 16u, 4u, 0x01u ); \
                                                 bFM_GPIO_PFR0_P7 = 1u; }while(0)

/*--- ADTG_0 for ADC#2 ---*/
#define SetPinFunc_ADTG_0_ADC2(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 20u, 4u, 0x01u ); \
                                                 bFM_GPIO_PFR0_P7 = 1u; }while(0)

/*--- ADTG_1 for ADC#0 ---*/
#define SetPinFunc_ADTG_1_ADC0(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 12u, 4u, 0x02u ); \
                                                 bFM_GPIO_PFR5_P5 = 1u; }while(0)

/*--- ADTG_1 for ADC#1 ---*/
#define SetPinFunc_ADTG_1_ADC1(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 16u, 4u, 0x02u ); \
                                                 bFM_GPIO_PFR5_P5 = 1u; }while(0)

/*--- ADTG_1 for ADC#2 ---*/
#define SetPinFunc_ADTG_1_ADC2(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 20u, 4u, 0x02u ); \
                                                 bFM_GPIO_PFR5_P5 = 1u; }while(0)

/*--- ADTG_2 for ADC#0 ---*/
#define SetPinFunc_ADTG_2_ADC0(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 12u, 4u, 0x03u ); \
                                                 bFM_GPIO_PFR3_P9 = 1u; }while(0)

/*--- ADTG_2 for ADC#1 ---*/
#define SetPinFunc_ADTG_2_ADC1(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 16u, 4u, 0x03u ); \
                                                 bFM_GPIO_PFR3_P9 = 1u; }while(0)

/*--- ADTG_2 for ADC#2 ---*/
#define SetPinFunc_ADTG_2_ADC2(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 20u, 4u, 0x03u ); \
                                                 bFM_GPIO_PFR3_P9 = 1u; }while(0)

/*--- ADTG_3 for ADC#0 ---*/
#define SetPinFunc_ADTG_3_ADC0(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 12u, 4u, 0x04u ); \
                                                 bFM_GPIO_PFR6_P2 = 1u; }while(0)

/*--- ADTG_3 for ADC#1 ---*/
#define SetPinFunc_ADTG_3_ADC1(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 16u, 4u, 0x04u ); \
                                                 bFM_GPIO_PFR6_P2 = 1u; }while(0)

/*--- ADTG_3 for ADC#2 ---*/
#define SetPinFunc_ADTG_3_ADC2(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 20u, 4u, 0x04u ); \
                                                 bFM_GPIO_PFR6_P2 = 1u; }while(0)

/*--- ADTG_4 for ADC#0 ---*/
#define SetPinFunc_ADTG_4_ADC0(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 12u, 4u, 0x05u ); \
                                                 bFM_GPIO_PFR2_P8 = 1u; \
                                                 bFM_GPIO_ADE_AN25 = 0u; }while(0)

/*--- ADTG_4 for ADC#1 ---*/
#define SetPinFunc_ADTG_4_ADC1(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 16u, 4u, 0x05u ); \
                                                 bFM_GPIO_PFR2_P8 = 1u; \
                                                 bFM_GPIO_ADE_AN25 = 0u; }while(0)

/*--- ADTG_4 for ADC#2 ---*/
#define SetPinFunc_ADTG_4_ADC2(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 20u, 4u, 0x05u ); \
                                                 bFM_GPIO_PFR2_P8 = 1u; \
                                                 bFM_GPIO_ADE_AN25 = 0u; }while(0)

/*--- ADTG_5 for ADC#0 ---*/
#define SetPinFunc_ADTG_5_ADC0(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 12u, 4u, 0x06u ); \
                                                 bFM_GPIO_PFR1_PF = 1u; \
                                                 bFM_GPIO_ADE_AN15 = 0u; }while(0)

/*--- ADTG_5 for ADC#1 ---*/
#define SetPinFunc_ADTG_5_ADC1(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 16u, 4u, 0x06u ); \
                                                 bFM_GPIO_PFR1_PF = 1u; \
                                                 bFM_GPIO_ADE_AN15 = 0u; }while(0)

/*--- ADTG_5 for ADC#2 ---*/
#define SetPinFunc_ADTG_5_ADC2(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 20u, 4u, 0x06u ); \
                                                 bFM_GPIO_PFR1_PF = 1u; \
                                                 bFM_GPIO_ADE_AN15 = 0u; }while(0)

/*--- ADTG_7 for ADC#0 ---*/
#define SetPinFunc_ADTG_7_ADC0(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 12u, 4u, 0x08u ); \
                                                 bFM_GPIO_PFR4_P3 = 1u; }while(0)

/*--- ADTG_7 for ADC#1 ---*/
#define SetPinFunc_ADTG_7_ADC1(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 16u, 4u, 0x08u ); \
                                                 bFM_GPIO_PFR4_P3 = 1u; }while(0)

/*--- ADTG_7 for ADC#2 ---*/
#define SetPinFunc_ADTG_7_ADC2(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 20u, 4u, 0x08u ); \
                                                 bFM_GPIO_PFR4_P3 = 1u; }while(0)

/*--- ADTG_8 for ADC#0 ---*/
#define SetPinFunc_ADTG_8_ADC0(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 12u, 4u, 0x09u ); \
                                                 bFM_GPIO_PFR7_P5 = 1u; }while(0)

/*--- ADTG_8 for ADC#1 ---*/
#define SetPinFunc_ADTG_8_ADC1(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 16u, 4u, 0x09u ); \
                                                 bFM_GPIO_PFR7_P5 = 1u; }while(0)

/*--- ADTG_8 for ADC#2 ---*/
#define SetPinFunc_ADTG_8_ADC2(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 20u, 4u, 0x09u ); \
                                                 bFM_GPIO_PFR7_P5 = 1u; }while(0)

/*--- AIN0_1 ---*/
#define SetPinFunc_AIN0_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 0u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR4_P9 = 1u; }while(0)

/*--- AIN0_2 ---*/
#define SetPinFunc_AIN0_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 0u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR5_P0 = 1u; }while(0)

/*--- AIN1_0 ---*/
#define SetPinFunc_AIN1_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 6u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR7_P8 = 1u; }while(0)

/*--- AIN1_1 ---*/
#define SetPinFunc_AIN1_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 6u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P0 = 1u; }while(0)

/*--- AIN1_2 ---*/
#define SetPinFunc_AIN1_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 6u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR4_PC = 1u; }while(0)

/*--- AIN2_0 ---*/
#define SetPinFunc_AIN2_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR14, 0u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR7_P2 = 1u; }while(0)

/*--- BIN0_1 ---*/
#define SetPinFunc_BIN0_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 2u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR4_PA = 1u; }while(0)

/*--- BIN0_2 ---*/
#define SetPinFunc_BIN0_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 2u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR5_P1 = 1u; }while(0)

/*--- BIN1_0 ---*/
#define SetPinFunc_BIN1_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 8u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR7_P9 = 1u; }while(0)

/*--- BIN1_1 ---*/
#define SetPinFunc_BIN1_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 8u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P1 = 1u; }while(0)

/*--- BIN1_2 ---*/
#define SetPinFunc_BIN1_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 8u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR4_PD = 1u; }while(0)

/*--- BIN2_0 ---*/
#define SetPinFunc_BIN2_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR14, 2u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR7_P3 = 1u; }while(0)

/*--- CROUT_0 ---*/
#define SetPinFunc_CROUT_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR00, 1u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR2_P0 = 1u; }while(0)

/*--- CROUT_1 ---*/
#define SetPinFunc_CROUT_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR00, 1u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFRC_P7 = 1u; }while(0)

/*--- CTS4_0 ---*/
#define SetPinFunc_CTS4_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 2u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFRC_PF = 1u; }while(0)

/*--- CTS4_1 ---*/
#define SetPinFunc_CTS4_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 2u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_PD = 1u; \
                                                 bFM_GPIO_ADE_AN13 = 0u; }while(0)

/*--- CTS4_2 ---*/
#define SetPinFunc_CTS4_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 2u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR0_P8 = 1u; }while(0)

/*--- DTTI0X_0 ---*/
#define SetPinFunc_DTTI0X_0(dummy)           do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 16u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_P9 = 1u; }while(0)

/*--- DTTI0X_1 ---*/
#define SetPinFunc_DTTI0X_1(dummy)           do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 16u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_PE = 1u; \
                                                 bFM_GPIO_ADE_AN14 = 0u; }while(0)

/*--- DTTI1X_0 ---*/
#define SetPinFunc_DTTI1X_0(dummy)           do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR02, 16u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P6 = 1u; }while(0)

/*--- DTTI1X_1 ---*/
#define SetPinFunc_DTTI1X_1(dummy)           do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR02, 16u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR4_P8 = 1u; }while(0)

/*--- DTTI2X_0 ---*/
#define SetPinFunc_DTTI2X_0(dummy)           do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR03, 16u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR0_P9 = 1u; }while(0)

/*--- E_COUT ---*/
#define SetPinFunc_E_COUT(dummy)             do{ bFM_GPIO_EPFR14_E_CKE = 1; \
                                                 bFM_GPIO_PFRC_PB = 1u; }while(0)

/*--- E_MDC ---*/
#define SetPinFunc_E_MDC(dummy)              do{ bFM_GPIO_EPFR22_E_MC0E = 1; \
                                                 bFM_GPIO_PFRC_P7 = 1u; }while(0)

/*--- E_MDIO ---*/
#define SetPinFunc_E_MDIO(dummy)             do{ bFM_GPIO_EPFR14_E_MD0B = 1; \
                                                 bFM_GPIO_PFRC_P6 = 1u; }while(0)

/*--- E_PPS ---*/
#define SetPinFunc_E_PPS(dummy)              do{ bFM_GPIO_EPFR14_E_PSE = 1; \
                                                 bFM_GPIO_PFR6_P2 = 1u; }while(0)

/*--- E_TX00 ---*/
#define SetPinFunc_E_TX00(dummy)             do{ bFM_GPIO_EPFR14_E_TD0E = 1; \
                                                 bFM_GPIO_PFRD_P2 = 1u; }while(0)

/*--- E_TX01 ---*/
#define SetPinFunc_E_TX01(dummy)             do{ bFM_GPIO_EPFR14_E_TD0E = 1; \
                                                 bFM_GPIO_PFRD_P1 = 1u; }while(0)

/*--- E_TX02 ---*/
#define SetPinFunc_E_TX02(dummy)             do{ bFM_GPIO_EPFR14_E_TD1E = 1; \
                                                 bFM_GPIO_PFRD_P0 = 1u; }while(0)

/*--- E_TX03 ---*/
#define SetPinFunc_E_TX03(dummy)             do{ bFM_GPIO_EPFR14_E_TD1E = 1; \
                                                 bFM_GPIO_PFRC_PF = 1u; }while(0)

/*--- E_TXEN ---*/
#define SetPinFunc_E_TXEN(dummy)             do{ bFM_GPIO_EPFR14_E_TE0E = 1; \
                                                 bFM_GPIO_PFRD_P3 = 1u; }while(0)

/*--- E_TXER ---*/
#define SetPinFunc_E_TXER(dummy)             do{ bFM_GPIO_EPFR14_E_TE1E = 1; \
                                                 bFM_GPIO_PFRC_PE = 1u; }while(0)

/*--- FRCK0_1 ---*/
#define SetPinFunc_FRCK0_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 18u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_PF = 1u; \
                                                 bFM_GPIO_ADE_AN15 = 0u; }while(0)

/*--- FRCK0_2 ---*/
#define SetPinFunc_FRCK0_2(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 18u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR1_P1 = 1u; \
                                                 bFM_GPIO_ADE_AN01 = 0u; }while(0)

/*--- FRCK1_0 ---*/
#define SetPinFunc_FRCK1_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR02, 18u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFRA_P0 = 1u; }while(0)

/*--- FRCK1_1 ---*/
#define SetPinFunc_FRCK1_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR02, 18u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR4_PD = 1u; }while(0)

/*--- FRCK2_0 ---*/
#define SetPinFunc_FRCK2_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR03, 18u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFRF_P6 = 1u; }while(0)

/*--- IC00_0 ---*/
#define SetPinFunc_IC00_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 20u, 3u, 0x01u ); \
                                                 bFM_GPIO_PFR3_P8 = 1u; }while(0)

/*--- IC00_1 ---*/
#define SetPinFunc_IC00_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 20u, 3u, 0x02u ); \
                                                 bFM_GPIO_PFR1_PA = 1u; \
                                                 bFM_GPIO_ADE_AN10 = 0u; }while(0)

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

/*--- IC01_0 ---*/
#define SetPinFunc_IC01_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 23u, 3u, 0x01u ); \
                                                 bFM_GPIO_PFR3_P7 = 1u; }while(0)

/*--- IC01_1 ---*/
#define SetPinFunc_IC01_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 23u, 3u, 0x02u ); \
                                                 bFM_GPIO_PFR1_PB = 1u; \
                                                 bFM_GPIO_ADE_AN11 = 0u; }while(0)

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
                                                 bFM_GPIO_PFR3_P6 = 1u; }while(0)

/*--- IC02_1 ---*/
#define SetPinFunc_IC02_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 26u, 3u, 0x02u ); \
                                                 bFM_GPIO_PFR1_PC = 1u; \
                                                 bFM_GPIO_ADE_AN12 = 0u; }while(0)

/*--- IC02_2 ---*/
#define SetPinFunc_IC02_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 26u, 3u, 0x03u ); \
                                                 bFM_GPIO_PFR1_P4 = 1u; \
                                                 bFM_GPIO_ADE_AN04 = 0u; }while(0)

/*--- IC02_LSYN2 ---*/
#define SetPinFunc_IC02_LSYN2(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 26u, 3u, 0x04u ); \
                                                 }while(0)

/*--- IC02_LSYN6 ---*/
#define SetPinFunc_IC02_LSYN6(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 26u, 3u, 0x05u ); \
                                                 }while(0)

/*--- IC03_1 ---*/
#define SetPinFunc_IC03_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 29u, 3u, 0x02u ); \
                                                 bFM_GPIO_PFR1_PD = 1u; \
                                                 bFM_GPIO_ADE_AN13 = 0u; }while(0)

/*--- IC03_2 ---*/
#define SetPinFunc_IC03_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 29u, 3u, 0x03u ); \
                                                 bFM_GPIO_PFR1_P5 = 1u; \
                                                 bFM_GPIO_ADE_AN05 = 0u; }while(0)

/*--- IC03_LSYN3 ---*/
#define SetPinFunc_IC03_LSYN3(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 29u, 3u, 0x04u ); \
                                                 }while(0)

/*--- IC03_LSYN7 ---*/
#define SetPinFunc_IC03_LSYN7(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 29u, 3u, 0x05u ); \
                                                 }while(0)

/*--- IC10_0 ---*/
#define SetPinFunc_IC10_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR02, 20u, 3u, 0x01u ); \
                                                 bFM_GPIO_PFRA_P1 = 1u; }while(0)

/*--- IC10_1 ---*/
#define SetPinFunc_IC10_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR02, 20u, 3u, 0x02u ); \
                                                 bFM_GPIO_PFR4_P9 = 1u; }while(0)

/*--- IC10_LSYN0 ---*/
#define SetPinFunc_IC10_LSYN0(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR02, 20u, 3u, 0x04u ); \
                                                 }while(0)

/*--- IC10_LSYN4 ---*/
#define SetPinFunc_IC10_LSYN4(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR02, 20u, 3u, 0x05u ); \
                                                 }while(0)

/*--- IC11_0 ---*/
#define SetPinFunc_IC11_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR02, 23u, 3u, 0x01u ); \
                                                 bFM_GPIO_PFRA_P2 = 1u; }while(0)

/*--- IC11_1 ---*/
#define SetPinFunc_IC11_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR02, 23u, 3u, 0x02u ); \
                                                 bFM_GPIO_PFR4_PA = 1u; }while(0)

/*--- IC11_LSYN1 ---*/
#define SetPinFunc_IC11_LSYN1(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR02, 23u, 3u, 0x04u ); \
                                                 }while(0)

/*--- IC11_LSYN5 ---*/
#define SetPinFunc_IC11_LSYN5(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR02, 23u, 3u, 0x05u ); \
                                                 }while(0)

/*--- IC12_0 ---*/
#define SetPinFunc_IC12_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR02, 26u, 3u, 0x01u ); \
                                                 bFM_GPIO_PFRA_P3 = 1u; }while(0)

/*--- IC12_1 ---*/
#define SetPinFunc_IC12_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR02, 26u, 3u, 0x02u ); \
                                                 bFM_GPIO_PFR4_PB = 1u; }while(0)

/*--- IC12_LSYN2 ---*/
#define SetPinFunc_IC12_LSYN2(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR02, 26u, 3u, 0x04u ); \
                                                 }while(0)

/*--- IC12_LSYN6 ---*/
#define SetPinFunc_IC12_LSYN6(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR02, 26u, 3u, 0x05u ); \
                                                 }while(0)

/*--- IC13_0 ---*/
#define SetPinFunc_IC13_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR02, 29u, 3u, 0x01u ); \
                                                 bFM_GPIO_PFRA_P4 = 1u; }while(0)

/*--- IC13_1 ---*/
#define SetPinFunc_IC13_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR02, 29u, 3u, 0x02u ); \
                                                 bFM_GPIO_PFR4_PC = 1u; }while(0)

/*--- IC13_LSYN3 ---*/
#define SetPinFunc_IC13_LSYN3(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR02, 29u, 3u, 0x04u ); \
                                                 }while(0)

/*--- IC13_LSYN7 ---*/
#define SetPinFunc_IC13_LSYN7(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR02, 29u, 3u, 0x05u ); \
                                                 }while(0)

/*--- IC20_0 ---*/
#define SetPinFunc_IC20_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR03, 20u, 3u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P0 = 1u; }while(0)

/*--- IC20_LSYN0 ---*/
#define SetPinFunc_IC20_LSYN0(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR03, 20u, 3u, 0x04u ); \
                                                 }while(0)

/*--- IC20_LSYN4 ---*/
#define SetPinFunc_IC20_LSYN4(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR03, 20u, 3u, 0x05u ); \
                                                 }while(0)

/*--- IC21_0 ---*/
#define SetPinFunc_IC21_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR03, 23u, 3u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P1 = 1u; }while(0)

/*--- IC21_LSYN1 ---*/
#define SetPinFunc_IC21_LSYN1(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR03, 23u, 3u, 0x04u ); \
                                                 }while(0)

/*--- IC21_LSYN5 ---*/
#define SetPinFunc_IC21_LSYN5(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR03, 23u, 3u, 0x05u ); \
                                                 }while(0)

/*--- IC22_0 ---*/
#define SetPinFunc_IC22_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR03, 26u, 3u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P2 = 1u; }while(0)

/*--- IC22_LSYN2 ---*/
#define SetPinFunc_IC22_LSYN2(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR03, 26u, 3u, 0x04u ); \
                                                 }while(0)

/*--- IC22_LSYN6 ---*/
#define SetPinFunc_IC22_LSYN6(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR03, 26u, 3u, 0x05u ); \
                                                 }while(0)

/*--- IC23_0 ---*/
#define SetPinFunc_IC23_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR03, 29u, 3u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P3 = 1u; }while(0)

/*--- IC23_LSYN3 ---*/
#define SetPinFunc_IC23_LSYN3(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR03, 29u, 3u, 0x04u ); \
                                                 }while(0)

/*--- IC23_LSYN7 ---*/
#define SetPinFunc_IC23_LSYN7(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR03, 29u, 3u, 0x05u ); \
                                                 }while(0)

/*--- INT00_0 ---*/
#define SetPinFunc_INT00_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 0u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P0 = 1u; }while(0)

/*--- INT00_1 ---*/
#define SetPinFunc_INT00_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 0u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR0_P5 = 1u; }while(0)

/*--- INT00_2 ---*/
#define SetPinFunc_INT00_2(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 0u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFRD_P2 = 1u; }while(0)

/*--- INT01_0 ---*/
#define SetPinFunc_INT01_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 2u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P1 = 1u; }while(0)

/*--- INT01_1 ---*/
#define SetPinFunc_INT01_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 2u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR0_P6 = 1u; }while(0)

/*--- INT01_2 ---*/
#define SetPinFunc_INT01_2(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 2u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR2_P4 = 1u; \
                                                 bFM_GPIO_ADE_AN29 = 0u; }while(0)

/*--- INT02_0 ---*/
#define SetPinFunc_INT02_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 4u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P2 = 1u; }while(0)

/*--- INT02_1 ---*/
#define SetPinFunc_INT02_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 4u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P1 = 1u; \
                                                 bFM_GPIO_ADE_AN01 = 0u; }while(0)

/*--- INT02_2 ---*/
#define SetPinFunc_INT02_2(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 4u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR2_P7 = 1u; \
                                                 bFM_GPIO_ADE_AN26 = 0u; }while(0)

/*--- INT03_0 ---*/
#define SetPinFunc_INT03_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 6u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFRA_P4 = 1u; }while(0)

/*--- INT03_1 ---*/
#define SetPinFunc_INT03_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 6u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P4 = 1u; \
                                                 bFM_GPIO_ADE_AN04 = 0u; }while(0)

/*--- INT04_1 ---*/
#define SetPinFunc_INT04_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 8u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P7 = 1u; \
                                                 bFM_GPIO_ADE_AN07 = 0u; }while(0)

/*--- INT05_0 ---*/
#define SetPinFunc_INT05_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 10u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR2_P0 = 1u; }while(0)

/*--- INT05_1 ---*/
#define SetPinFunc_INT05_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 10u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_PA = 1u; \
                                                 bFM_GPIO_ADE_AN10 = 0u; }while(0)

/*--- INT06_1 ---*/
#define SetPinFunc_INT06_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 12u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P1 = 1u; }while(0)

/*--- INT06_2 ---*/
#define SetPinFunc_INT06_2(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 12u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR4_PE = 1u; }while(0)

/*--- INT07_1 ---*/
#define SetPinFunc_INT07_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 14u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR7_P5 = 1u; }while(0)

/*--- INT07_2 ---*/
#define SetPinFunc_INT07_2(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 14u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR5_P3 = 1u; }while(0)

/*--- INT08_0 ---*/
#define SetPinFunc_INT08_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 16u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFRF_P5 = 1u; }while(0)

/*--- INT08_2 ---*/
#define SetPinFunc_INT08_2(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 16u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR5_P6 = 1u; }while(0)

/*--- INT09_0 ---*/
#define SetPinFunc_INT09_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 18u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR2_P8 = 1u; \
                                                 bFM_GPIO_ADE_AN25 = 0u; }while(0)

/*--- INT09_1 ---*/
#define SetPinFunc_INT09_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 18u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P6 = 1u; }while(0)

/*--- INT09_2 ---*/
#define SetPinFunc_INT09_2(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 18u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR5_P9 = 1u; }while(0)

/*--- INT10_1 ---*/
#define SetPinFunc_INT10_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 20u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P7 = 1u; }while(0)

/*--- INT10_2 ---*/
#define SetPinFunc_INT10_2(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 20u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFRA_P5 = 1u; }while(0)

/*--- INT11_1 ---*/
#define SetPinFunc_INT11_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 22u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P8 = 1u; }while(0)

/*--- INT11_2 ---*/
#define SetPinFunc_INT11_2(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 22u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR7_P6 = 1u; }while(0)

/*--- INT12_1 ---*/
#define SetPinFunc_INT12_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 24u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR4_P0 = 1u; }while(0)

/*--- INT12_2 ---*/
#define SetPinFunc_INT12_2(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 24u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR7_P7 = 1u; }while(0)

/*--- INT13_1 ---*/
#define SetPinFunc_INT13_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 26u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR4_P1 = 1u; }while(0)

/*--- INT13_2 ---*/
#define SetPinFunc_INT13_2(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 26u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR7_P1 = 1u; }while(0)

/*--- INT14_1 ---*/
#define SetPinFunc_INT14_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 28u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR4_P8 = 1u; }while(0)

/*--- INT14_2 ---*/
#define SetPinFunc_INT14_2(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 28u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR7_P2 = 1u; }while(0)

/*--- INT15_1 ---*/
#define SetPinFunc_INT15_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 30u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR6_P0 = 1u; }while(0)

/*--- INT15_2 ---*/
#define SetPinFunc_INT15_2(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 30u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR7_P3 = 1u; }while(0)

/*--- INT16_1 ---*/
#define SetPinFunc_INT16_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR15, 0u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR5_P7 = 1u; }while(0)

/*--- INT17_1 ---*/
#define SetPinFunc_INT17_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR15, 2u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR5_P8 = 1u; }while(0)

/*--- INT18_1 ---*/
#define SetPinFunc_INT18_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR15, 4u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR5_PA = 1u; }while(0)

/*--- INT19_1 ---*/
#define SetPinFunc_INT19_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR15, 6u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR5_PB = 1u; }while(0)

/*--- INT20_1 ---*/
#define SetPinFunc_INT20_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR15, 8u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P6 = 1u; \
                                                 bFM_GPIO_ADE_AN06 = 0u; }while(0)

/*--- INT21_1 ---*/
#define SetPinFunc_INT21_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR15, 10u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P8 = 1u; \
                                                 bFM_GPIO_ADE_AN08 = 0u; }while(0)

/*--- INT22_1 ---*/
#define SetPinFunc_INT22_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR15, 12u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P9 = 1u; \
                                                 bFM_GPIO_ADE_AN09 = 0u; }while(0)

/*--- INT23_1 ---*/
#define SetPinFunc_INT23_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR15, 14u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR7_P9 = 1u; }while(0)

/*--- INT24_1 ---*/
#define SetPinFunc_INT24_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR15, 16u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR7_PA = 1u; }while(0)

/*--- INT25_1 ---*/
#define SetPinFunc_INT25_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR15, 18u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_PB = 1u; \
                                                 bFM_GPIO_ADE_AN11 = 0u; }while(0)

/*--- INT26_1 ---*/
#define SetPinFunc_INT26_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR15, 20u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_PC = 1u; \
                                                 bFM_GPIO_ADE_AN12 = 0u; }while(0)

/*--- INT27_1 ---*/
#define SetPinFunc_INT27_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR15, 22u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_PD = 1u; \
                                                 bFM_GPIO_ADE_AN13 = 0u; }while(0)

/*--- INT28_1 ---*/
#define SetPinFunc_INT28_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR15, 24u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_PE = 1u; \
                                                 bFM_GPIO_ADE_AN14 = 0u; }while(0)

/*--- INT29_1 ---*/
#define SetPinFunc_INT29_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR15, 26u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_PF = 1u; \
                                                 bFM_GPIO_ADE_AN15 = 0u; }while(0)

/*--- INT30_1 ---*/
#define SetPinFunc_INT30_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR15, 28u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFRD_P0 = 1u; }while(0)

/*--- INT31_1 ---*/
#define SetPinFunc_INT31_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR15, 30u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFRD_P1 = 1u; }while(0)

/*--- MAD00_0 ---*/
#define SetPinFunc_MAD00_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR10, 14u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR1_P4 = 1u; \
                                                 bFM_GPIO_ADE_AN04 = 0u; }while(0)

/*--- MAD01_0 ---*/
#define SetPinFunc_MAD01_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR11, 2u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR1_P5 = 1u; \
                                                 bFM_GPIO_ADE_AN05 = 0u; }while(0)

/*--- MAD02_0 ---*/
#define SetPinFunc_MAD02_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR11, 3u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR1_P6 = 1u; \
                                                 bFM_GPIO_ADE_AN06 = 0u; }while(0)

/*--- MAD03_0 ---*/
#define SetPinFunc_MAD03_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR11, 2u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR1_P7 = 1u; \
                                                 bFM_GPIO_ADE_AN07 = 0u; }while(0)

/*--- MAD04_0 ---*/
#define SetPinFunc_MAD04_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR11, 5u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR1_P8 = 1u; \
                                                 bFM_GPIO_ADE_AN08 = 0u; }while(0)

/*--- MAD05_0 ---*/
#define SetPinFunc_MAD05_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR11, 6u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR1_P9 = 1u; \
                                                 bFM_GPIO_ADE_AN09 = 0u; }while(0)

/*--- MAD06_0 ---*/
#define SetPinFunc_MAD06_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR11, 7u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR1_PA = 1u; \
                                                 bFM_GPIO_ADE_AN10 = 0u; }while(0)

/*--- MAD07_0 ---*/
#define SetPinFunc_MAD07_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR11, 8u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR1_PB = 1u; \
                                                 bFM_GPIO_ADE_AN11 = 0u; }while(0)

/*--- MAD08_0 ---*/
#define SetPinFunc_MAD08_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR10, 15u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR1_PC = 1u; \
                                                 bFM_GPIO_ADE_AN12 = 0u; }while(0)

/*--- MAD09_0 ---*/
#define SetPinFunc_MAD09_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR10, 16u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR1_PD = 1u; \
                                                 bFM_GPIO_ADE_AN13 = 0u; }while(0)

/*--- MAD10_0 ---*/
#define SetPinFunc_MAD10_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR10, 17u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR1_PE = 1u; \
                                                 bFM_GPIO_ADE_AN14 = 0u; }while(0)

/*--- MAD11_0 ---*/
#define SetPinFunc_MAD11_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR10, 18u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR1_PF = 1u; \
                                                 bFM_GPIO_ADE_AN15 = 0u; }while(0)

/*--- MAD12_0 ---*/
#define SetPinFunc_MAD12_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR10, 19u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR2_P9 = 1u; \
                                                 bFM_GPIO_ADE_AN24 = 0u; }while(0)

/*--- MAD13_0 ---*/
#define SetPinFunc_MAD13_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR10, 20u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR2_P8 = 1u; \
                                                 bFM_GPIO_ADE_AN25 = 0u; }while(0)

/*--- MAD14_0 ---*/
#define SetPinFunc_MAD14_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR10, 21u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR2_P7 = 1u; \
                                                 bFM_GPIO_ADE_AN26 = 0u; }while(0)

/*--- MAD15_0 ---*/
#define SetPinFunc_MAD15_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR10, 22u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR2_P6 = 1u; \
                                                 bFM_GPIO_ADE_AN27 = 0u; }while(0)

/*--- MAD16_0 ---*/
#define SetPinFunc_MAD16_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR10, 23u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR2_P5 = 1u; \
                                                 bFM_GPIO_ADE_AN28 = 0u; }while(0)

/*--- MAD17_0 ---*/
#define SetPinFunc_MAD17_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR10, 24u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR2_P4 = 1u; \
                                                 bFM_GPIO_ADE_AN29 = 0u; }while(0)

/*--- MAD18_0 ---*/
#define SetPinFunc_MAD18_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR10, 25u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR2_P0 = 1u; }while(0)

/*--- MADATA00_0 ---*/
#define SetPinFunc_MADATA00_0(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR11, 9u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR4_PA = 1u; }while(0)

/*--- MADATA01_0 ---*/
#define SetPinFunc_MADATA01_0(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR11, 10u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR4_PB = 1u; }while(0)

/*--- MADATA02_0 ---*/
#define SetPinFunc_MADATA02_0(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR11, 11u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR4_PC = 1u; }while(0)

/*--- MADATA03_0 ---*/
#define SetPinFunc_MADATA03_0(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR11, 12u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR4_PD = 1u; }while(0)

/*--- MADATA04_0 ---*/
#define SetPinFunc_MADATA04_0(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR11, 13u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR4_PE = 1u; }while(0)

/*--- MADATA05_0 ---*/
#define SetPinFunc_MADATA05_0(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR11, 14u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR7_P0 = 1u; }while(0)

/*--- MADATA06_0 ---*/
#define SetPinFunc_MADATA06_0(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR11, 15u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR7_P1 = 1u; }while(0)

/*--- MADATA07_0 ---*/
#define SetPinFunc_MADATA07_0(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR11, 16u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR7_P2 = 1u; }while(0)

/*--- MADATA08_0 ---*/
#define SetPinFunc_MADATA08_0(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR11, 17u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR7_P3 = 1u; }while(0)

/*--- MADATA09_0 ---*/
#define SetPinFunc_MADATA09_0(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR11, 18u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR7_P4 = 1u; }while(0)

/*--- MADATA10_0 ---*/
#define SetPinFunc_MADATA10_0(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR11, 19u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR7_P5 = 1u; }while(0)

/*--- MADATA11_0 ---*/
#define SetPinFunc_MADATA11_0(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR11, 20u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR7_P6 = 1u; }while(0)

/*--- MADATA12_0 ---*/
#define SetPinFunc_MADATA12_0(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR11, 21u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR7_P7 = 1u; }while(0)

/*--- MADATA13_0 ---*/
#define SetPinFunc_MADATA13_0(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR11, 22u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR7_P8 = 1u; }while(0)

/*--- MADATA14_0 ---*/
#define SetPinFunc_MADATA14_0(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR11, 23u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR7_P9 = 1u; }while(0)

/*--- MADATA15_0 ---*/
#define SetPinFunc_MADATA15_0(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR11, 24u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR7_PA = 1u; }while(0)

/*--- MALE_0 ---*/
#define SetPinFunc_MALE_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR11, 0u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P4 = 1u; }while(0)

/*--- MCLKOUT_0 ---*/
#define SetPinFunc_MCLKOUT_0(dummy)          do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR10, 2u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR3_P8 = 1u; }while(0)

/*--- MCSX0_0 ---*/
#define SetPinFunc_MCSX0_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR11, 1u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR5_PA = 1u; }while(0)

/*--- MCSX1_0 ---*/
#define SetPinFunc_MCSX1_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR10, 7u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR5_PB = 1u; }while(0)

/*--- MCSX2_0 ---*/
#define SetPinFunc_MCSX2_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR10, 8u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR3_P6 = 1u; }while(0)

/*--- MCSX3_0 ---*/
#define SetPinFunc_MCSX3_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR10, 9u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR3_P7 = 1u; }while(0)

/*--- MCSX4_0 ---*/
#define SetPinFunc_MCSX4_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR10, 10u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR1_P3 = 1u; \
                                                 bFM_GPIO_ADE_AN03 = 0u; }while(0)

/*--- MCSX5_0 ---*/
#define SetPinFunc_MCSX5_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR10, 11u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR1_P2 = 1u; \
                                                 bFM_GPIO_ADE_AN02 = 0u; }while(0)

/*--- MCSX6_0 ---*/
#define SetPinFunc_MCSX6_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR10, 12u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR1_P1 = 1u; \
                                                 bFM_GPIO_ADE_AN01 = 0u; }while(0)

/*--- MCSX7_0 ---*/
#define SetPinFunc_MCSX7_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR10, 13u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR1_P0 = 1u; \
                                                 bFM_GPIO_ADE_AN00 = 0u; }while(0)

/*--- MDQM0_0 ---*/
#define SetPinFunc_MDQM0_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR10, 4u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P2 = 1u; }while(0)

/*--- MDQM1_0 ---*/
#define SetPinFunc_MDQM1_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR10, 4u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P3 = 1u; }while(0)

/*--- MNALE_0 ---*/
#define SetPinFunc_MNALE_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR10, 6u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P6 = 1u; }while(0)

/*--- MNCLE_0 ---*/
#define SetPinFunc_MNCLE_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR10, 6u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P7 = 1u; }while(0)

/*--- MNREX_0 ---*/
#define SetPinFunc_MNREX_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR10, 6u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P9 = 1u; }while(0)

/*--- MNWEX_0 ---*/
#define SetPinFunc_MNWEX_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR10, 6u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P8 = 1u; }while(0)

/*--- MOEX_0 ---*/
#define SetPinFunc_MOEX_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR10, 5u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P0 = 1u; }while(0)

/*--- MWEX_0 ---*/
#define SetPinFunc_MWEX_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR10, 3u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P1 = 1u; }while(0)

/*--- NMIX ---*/
#define SetPinFunc_NMIX(dummy)               do{ bFM_GPIO_EPFR00_NMIS = 1; \
                                                 bFM_GPIO_PFRF_P6 = 1u; }while(0)

/*--- RTO00_0 ---*/
#define SetPinFunc_RTO00_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 0u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_PA = 1u; }while(0)

/*--- RTO00_1 ---*/
#define SetPinFunc_RTO00_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 0u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P3 = 1u; \
                                                 bFM_GPIO_ADE_AN30 = 0u; }while(0)

/*--- RTO01_0 ---*/
#define SetPinFunc_RTO01_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 2u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_PB = 1u; }while(0)

/*--- RTO01_1 ---*/
#define SetPinFunc_RTO01_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 2u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P4 = 1u; \
                                                 bFM_GPIO_ADE_AN29 = 0u; }while(0)

/*--- RTO02_0 ---*/
#define SetPinFunc_RTO02_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 4u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_PC = 1u; }while(0)

/*--- RTO02_1 ---*/
#define SetPinFunc_RTO02_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 4u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P5 = 1u; \
                                                 bFM_GPIO_ADE_AN28 = 0u; }while(0)

/*--- RTO03_0 ---*/
#define SetPinFunc_RTO03_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 6u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_PD = 1u; }while(0)

/*--- RTO03_1 ---*/
#define SetPinFunc_RTO03_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 6u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P6 = 1u; \
                                                 bFM_GPIO_ADE_AN27 = 0u; }while(0)

/*--- RTO04_0 ---*/
#define SetPinFunc_RTO04_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 8u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_PE = 1u; }while(0)

/*--- RTO04_1 ---*/
#define SetPinFunc_RTO04_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 8u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P7 = 1u; \
                                                 bFM_GPIO_ADE_AN26 = 0u; }while(0)

/*--- RTO05_0 ---*/
#define SetPinFunc_RTO05_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 10u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_PF = 1u; }while(0)

/*--- RTO05_1 ---*/
#define SetPinFunc_RTO05_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 10u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P8 = 1u; \
                                                 bFM_GPIO_ADE_AN25 = 0u; }while(0)

/*--- RTO10_0 ---*/
#define SetPinFunc_RTO10_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR02, 0u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P0 = 1u; }while(0)

/*--- RTO10_1 ---*/
#define SetPinFunc_RTO10_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR02, 0u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR4_P0 = 1u; }while(0)

/*--- RTO11_0 ---*/
#define SetPinFunc_RTO11_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR02, 2u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P1 = 1u; }while(0)

/*--- RTO11_1 ---*/
#define SetPinFunc_RTO11_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR02, 2u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR4_P1 = 1u; }while(0)

/*--- RTO12_0 ---*/
#define SetPinFunc_RTO12_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR02, 4u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P2 = 1u; }while(0)

/*--- RTO12_1 ---*/
#define SetPinFunc_RTO12_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR02, 4u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR4_P2 = 1u; }while(0)

/*--- RTO13_0 ---*/
#define SetPinFunc_RTO13_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR02, 6u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P3 = 1u; }while(0)

/*--- RTO13_1 ---*/
#define SetPinFunc_RTO13_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR02, 6u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR4_P3 = 1u; }while(0)

/*--- RTO14_0 ---*/
#define SetPinFunc_RTO14_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR02, 8u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P4 = 1u; }while(0)

/*--- RTO14_1 ---*/
#define SetPinFunc_RTO14_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR02, 8u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR4_P4 = 1u; }while(0)

/*--- RTO15_0 ---*/
#define SetPinFunc_RTO15_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR02, 10u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P5 = 1u; }while(0)

/*--- RTO15_1 ---*/
#define SetPinFunc_RTO15_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR02, 10u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR4_P5 = 1u; }while(0)

/*--- RTO20_0 ---*/
#define SetPinFunc_RTO20_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR03, 0u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFRA_P0 = 1u; }while(0)

/*--- RTO21_0 ---*/
#define SetPinFunc_RTO21_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR03, 2u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFRA_P1 = 1u; }while(0)

/*--- RTO22_0 ---*/
#define SetPinFunc_RTO22_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR03, 4u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFRA_P2 = 1u; }while(0)

/*--- RTO23_0 ---*/
#define SetPinFunc_RTO23_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR03, 6u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFRA_P3 = 1u; }while(0)

/*--- RTO24_0 ---*/
#define SetPinFunc_RTO24_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR03, 8u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFRA_P4 = 1u; }while(0)

/*--- RTO25_0 ---*/
#define SetPinFunc_RTO25_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR03, 10u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFRA_P5 = 1u; }while(0)

/*--- RTS4_0 ---*/
#define SetPinFunc_RTS4_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 0u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFRC_PE = 1u; }while(0)

/*--- RTS4_1 ---*/
#define SetPinFunc_RTS4_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 0u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_PE = 1u; \
                                                 bFM_GPIO_ADE_AN14 = 0u; }while(0)

/*--- RTS4_2 ---*/
#define SetPinFunc_RTS4_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 0u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR0_P9 = 1u; }while(0)

/*--- SCK0_0 ---*/
#define SetPinFunc_SCK0_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 8u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR2_P3 = 1u; \
                                                 bFM_GPIO_ADE_AN30 = 0u; }while(0)

/*--- SCK0_1 ---*/
#define SetPinFunc_SCK0_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 8u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P6 = 1u; \
                                                 bFM_GPIO_ADE_AN06 = 0u; }while(0)

/*--- SCK1_0 ---*/
#define SetPinFunc_SCK1_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 14u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P8 = 1u; }while(0)

/*--- SCK1_1 ---*/
#define SetPinFunc_SCK1_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 14u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P3 = 1u; \
                                                 bFM_GPIO_ADE_AN03 = 0u; }while(0)

/*--- SCK2_0 ---*/
#define SetPinFunc_SCK2_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 20u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR7_P4 = 1u; }while(0)

/*--- SCK2_1 ---*/
#define SetPinFunc_SCK2_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 20u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P6 = 1u; \
                                                 bFM_GPIO_ADE_AN27 = 0u; }while(0)

/*--- SCK2_2 ---*/
#define SetPinFunc_SCK2_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 20u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR1_P9 = 1u; \
                                                 bFM_GPIO_ADE_AN09 = 0u; }while(0)

/*--- SCK3_0 ---*/
#define SetPinFunc_SCK3_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 26u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR7_P7 = 1u; }while(0)

/*--- SCK3_1 ---*/
#define SetPinFunc_SCK3_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 26u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR5_P2 = 1u; }while(0)

/*--- SCK3_2 ---*/
#define SetPinFunc_SCK3_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 26u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR4_PA = 1u; }while(0)

/*--- SCK4_0 ---*/
#define SetPinFunc_SCK4_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 8u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFRD_P0 = 1u; }while(0)

/*--- SCK4_1 ---*/
#define SetPinFunc_SCK4_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 8u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_PC = 1u; \
                                                 bFM_GPIO_ADE_AN12 = 0u; }while(0)

/*--- SCK4_2 ---*/
#define SetPinFunc_SCK4_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 8u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR0_P7 = 1u; }while(0)

/*--- SCK5_0 ---*/
#define SetPinFunc_SCK5_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 14u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR6_P2 = 1u; }while(0)

/*--- SCK5_2 ---*/
#define SetPinFunc_SCK5_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 14u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR3_P8 = 1u; }while(0)

/*--- SCK6_0 ---*/
#define SetPinFunc_SCK6_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 20u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P5 = 1u; }while(0)

/*--- SCK6_2 ---*/
#define SetPinFunc_SCK6_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 20u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFRF_P5 = 1u; }while(0)

/*--- SCK7_0 ---*/
#define SetPinFunc_SCK7_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 26u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_PB = 1u; }while(0)

/*--- SCK7_1 ---*/
#define SetPinFunc_SCK7_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 26u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR4_PC = 1u; }while(0)

/*--- SIN0_0 ---*/
#define SetPinFunc_SIN0_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 4u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR2_P1 = 1u; }while(0)

/*--- SIN0_1 ---*/
#define SetPinFunc_SIN0_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 4u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P4 = 1u; \
                                                 bFM_GPIO_ADE_AN04 = 0u; }while(0)

/*--- SIN1_0 ---*/
#define SetPinFunc_SIN1_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 10u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P6 = 1u; }while(0)

/*--- SIN1_1 ---*/
#define SetPinFunc_SIN1_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 10u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P1 = 1u; \
                                                 bFM_GPIO_ADE_AN01 = 0u; }while(0)

/*--- SIN2_0 ---*/
#define SetPinFunc_SIN2_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 16u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR7_P2 = 1u; }while(0)

/*--- SIN2_1 ---*/
#define SetPinFunc_SIN2_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 16u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P4 = 1u; \
                                                 bFM_GPIO_ADE_AN29 = 0u; }while(0)

/*--- SIN2_2 ---*/
#define SetPinFunc_SIN2_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 16u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR1_P7 = 1u; \
                                                 bFM_GPIO_ADE_AN07 = 0u; }while(0)

/*--- SIN3_0 ---*/
#define SetPinFunc_SIN3_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 22u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR7_P5 = 1u; }while(0)

/*--- SIN3_1 ---*/
#define SetPinFunc_SIN3_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 22u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR5_P0 = 1u; }while(0)

/*--- SIN3_2 ---*/
#define SetPinFunc_SIN3_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 22u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR4_P8 = 1u; }while(0)

/*--- SIN4_0 ---*/
#define SetPinFunc_SIN4_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 4u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFRD_P2 = 1u; }while(0)

/*--- SIN4_1 ---*/
#define SetPinFunc_SIN4_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 4u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_PA = 1u; \
                                                 bFM_GPIO_ADE_AN10 = 0u; }while(0)

/*--- SIN4_2 ---*/
#define SetPinFunc_SIN4_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 4u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR0_P5 = 1u; }while(0)

/*--- SIN5_0 ---*/
#define SetPinFunc_SIN5_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 10u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR6_P0 = 1u; }while(0)

/*--- SIN5_2 ---*/
#define SetPinFunc_SIN5_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 10u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR3_P6 = 1u; }while(0)

/*--- SIN6_0 ---*/
#define SetPinFunc_SIN6_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 16u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P3 = 1u; }while(0)

/*--- SIN7_0 ---*/
#define SetPinFunc_SIN7_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 22u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P9 = 1u; }while(0)

/*--- SIN7_1 ---*/
#define SetPinFunc_SIN7_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 22u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR4_PE = 1u; }while(0)

/*--- SOT0_0 ---*/
#define SetPinFunc_SOT0_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 6u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR2_P2 = 1u; \
                                                 bFM_GPIO_ADE_AN31 = 0u; }while(0)

/*--- SOT0_1 ---*/
#define SetPinFunc_SOT0_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 6u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P5 = 1u; \
                                                 bFM_GPIO_ADE_AN05 = 0u; }while(0)

/*--- SOT1_0 ---*/
#define SetPinFunc_SOT1_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 12u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P7 = 1u; }while(0)

/*--- SOT1_1 ---*/
#define SetPinFunc_SOT1_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 12u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P2 = 1u; \
                                                 bFM_GPIO_ADE_AN02 = 0u; }while(0)

/*--- SOT2_0 ---*/
#define SetPinFunc_SOT2_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 18u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR7_P3 = 1u; }while(0)

/*--- SOT2_1 ---*/
#define SetPinFunc_SOT2_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 18u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P5 = 1u; \
                                                 bFM_GPIO_ADE_AN28 = 0u; }while(0)

/*--- SOT2_2 ---*/
#define SetPinFunc_SOT2_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 18u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR1_P8 = 1u; \
                                                 bFM_GPIO_ADE_AN08 = 0u; }while(0)

/*--- SOT3_0 ---*/
#define SetPinFunc_SOT3_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 24u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR7_P6 = 1u; }while(0)

/*--- SOT3_1 ---*/
#define SetPinFunc_SOT3_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 24u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR5_P1 = 1u; }while(0)

/*--- SOT3_2 ---*/
#define SetPinFunc_SOT3_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 24u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR4_P9 = 1u; }while(0)

/*--- SOT4_0 ---*/
#define SetPinFunc_SOT4_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 6u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFRD_P1 = 1u; }while(0)

/*--- SOT4_1 ---*/
#define SetPinFunc_SOT4_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 6u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_PB = 1u; \
                                                 bFM_GPIO_ADE_AN11 = 0u; }while(0)

/*--- SOT4_2 ---*/
#define SetPinFunc_SOT4_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 6u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR0_P6 = 1u; }while(0)

/*--- SOT5_0 ---*/
#define SetPinFunc_SOT5_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 12u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR6_P1 = 1u; }while(0)

/*--- SOT5_2 ---*/
#define SetPinFunc_SOT5_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 12u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR3_P7 = 1u; }while(0)

/*--- SOT6_0 ---*/
#define SetPinFunc_SOT6_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 18u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P4 = 1u; }while(0)

/*--- SOT7_0 ---*/
#define SetPinFunc_SOT7_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 24u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_PA = 1u; }while(0)

/*--- SOT7_1 ---*/
#define SetPinFunc_SOT7_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 24u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR4_PD = 1u; }while(0)

/*--- TIOA00_0_OUT ---*/
#define SetPinFunc_TIOA00_0_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 2u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR4_P0 = 1u; }while(0)

/*--- TIOA00_1_OUT ---*/
#define SetPinFunc_TIOA00_1_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 2u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_PA = 1u; }while(0)

/*--- TIOA00_2_OUT ---*/
#define SetPinFunc_TIOA00_2_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 2u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR0_P8 = 1u; }while(0)

/*--- TIOA01_0_IN ---*/
#define SetPinFunc_TIOA01_0_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 8u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR4_P1 = 1u; }while(0)

/*--- TIOA01_0_OUT ---*/
#define SetPinFunc_TIOA01_0_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 10u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR4_P1 = 1u; }while(0)

/*--- TIOA01_1_IN ---*/
#define SetPinFunc_TIOA01_1_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 8u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_PB = 1u; }while(0)

/*--- TIOA01_1_OUT ---*/
#define SetPinFunc_TIOA01_1_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 10u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_PB = 1u; }while(0)

/*--- TIOA01_2_IN ---*/
#define SetPinFunc_TIOA01_2_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 8u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR5_P3 = 1u; }while(0)

/*--- TIOA01_2_OUT ---*/
#define SetPinFunc_TIOA01_2_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 10u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR5_P3 = 1u; }while(0)

/*--- TIOA02_0_OUT ---*/
#define SetPinFunc_TIOA02_0_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 18u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR4_P2 = 1u; }while(0)

/*--- TIOA02_1_OUT ---*/
#define SetPinFunc_TIOA02_1_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 18u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_PC = 1u; }while(0)

/*--- TIOA02_2_OUT ---*/
#define SetPinFunc_TIOA02_2_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 18u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR6_P0 = 1u; }while(0)

/*--- TIOA03_0_IN ---*/
#define SetPinFunc_TIOA03_0_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 24u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR4_P3 = 1u; }while(0)

/*--- TIOA03_0_OUT ---*/
#define SetPinFunc_TIOA03_0_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 26u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR4_P3 = 1u; }while(0)

/*--- TIOA03_1_IN ---*/
#define SetPinFunc_TIOA03_1_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 24u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_PD = 1u; }while(0)

/*--- TIOA03_1_OUT ---*/
#define SetPinFunc_TIOA03_1_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 26u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_PD = 1u; }while(0)

/*--- TIOA03_2_IN ---*/
#define SetPinFunc_TIOA03_2_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 24u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFRD_P2 = 1u; }while(0)

/*--- TIOA03_2_OUT ---*/
#define SetPinFunc_TIOA03_2_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 26u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFRD_P2 = 1u; }while(0)

/*--- TIOA04_0_OUT ---*/
#define SetPinFunc_TIOA04_0_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 2u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR4_P4 = 1u; }while(0)

/*--- TIOA04_1_OUT ---*/
#define SetPinFunc_TIOA04_1_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 2u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_PE = 1u; }while(0)

/*--- TIOA04_2_OUT ---*/
#define SetPinFunc_TIOA04_2_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 2u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR7_P0 = 1u; }while(0)

/*--- TIOA05_0_IN ---*/
#define SetPinFunc_TIOA05_0_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 8u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR4_P5 = 1u; }while(0)

/*--- TIOA05_0_OUT ---*/
#define SetPinFunc_TIOA05_0_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 10u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR4_P5 = 1u; }while(0)

/*--- TIOA05_1_IN ---*/
#define SetPinFunc_TIOA05_1_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 8u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_PF = 1u; }while(0)

/*--- TIOA05_1_OUT ---*/
#define SetPinFunc_TIOA05_1_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 10u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_PF = 1u; }while(0)

/*--- TIOA05_2_IN ---*/
#define SetPinFunc_TIOA05_2_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 8u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR0_P5 = 1u; }while(0)

/*--- TIOA05_2_OUT ---*/
#define SetPinFunc_TIOA05_2_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 10u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR0_P5 = 1u; }while(0)

/*--- TIOA06_1_OUT ---*/
#define SetPinFunc_TIOA06_1_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 18u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFRC_P3 = 1u; }while(0)

/*--- TIOA07_1_IN ---*/
#define SetPinFunc_TIOA07_1_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 24u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P3 = 1u; \
                                                 bFM_GPIO_ADE_AN30 = 0u; }while(0)

/*--- TIOA07_1_OUT ---*/
#define SetPinFunc_TIOA07_1_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 26u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P3 = 1u; \
                                                 bFM_GPIO_ADE_AN30 = 0u; }while(0)

/*--- TIOA07_2_IN ---*/
#define SetPinFunc_TIOA07_2_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 24u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR7_P6 = 1u; }while(0)

/*--- TIOA07_2_OUT ---*/
#define SetPinFunc_TIOA07_2_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 26u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR7_P6 = 1u; }while(0)

/*--- TIOA08_0_OUT ---*/
#define SetPinFunc_TIOA08_0_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR12, 2u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFRA_P0 = 1u; }while(0)

/*--- TIOA08_2_OUT ---*/
#define SetPinFunc_TIOA08_2_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR12, 2u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFRC_P4 = 1u; }while(0)

/*--- TIOA09_0_IN ---*/
#define SetPinFunc_TIOA09_0_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR12, 8u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFRA_P1 = 1u; }while(0)

/*--- TIOA09_0_OUT ---*/
#define SetPinFunc_TIOA09_0_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR12, 10u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFRA_P1 = 1u; }while(0)

/*--- TIOA09_2_IN ---*/
#define SetPinFunc_TIOA09_2_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR12, 8u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR5_P6 = 1u; }while(0)

/*--- TIOA09_2_OUT ---*/
#define SetPinFunc_TIOA09_2_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR12, 10u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR5_P6 = 1u; }while(0)

/*--- TIOA10_0_OUT ---*/
#define SetPinFunc_TIOA10_0_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR12, 18u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFRA_P2 = 1u; }while(0)

/*--- TIOA10_2_OUT ---*/
#define SetPinFunc_TIOA10_2_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR12, 18u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFRC_P5 = 1u; }while(0)

/*--- TIOA11_0_IN ---*/
#define SetPinFunc_TIOA11_0_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR12, 24u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFRA_P3 = 1u; }while(0)

/*--- TIOA11_0_OUT ---*/
#define SetPinFunc_TIOA11_0_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR12, 26u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFRA_P3 = 1u; }while(0)

/*--- TIOA11_2_IN ---*/
#define SetPinFunc_TIOA11_2_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR12, 24u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR5_P8 = 1u; }while(0)

/*--- TIOA11_2_OUT ---*/
#define SetPinFunc_TIOA11_2_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR12, 26u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR5_P8 = 1u; }while(0)

/*--- TIOA12_0_OUT ---*/
#define SetPinFunc_TIOA12_0_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR13, 2u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFRA_P4 = 1u; }while(0)

/*--- TIOA12_2_OUT ---*/
#define SetPinFunc_TIOA12_2_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR13, 2u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR3_P6 = 1u; }while(0)

/*--- TIOA13_0_IN ---*/
#define SetPinFunc_TIOA13_0_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR13, 8u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFRA_P5 = 1u; }while(0)

/*--- TIOA13_0_OUT ---*/
#define SetPinFunc_TIOA13_0_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR13, 10u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFRA_P5 = 1u; }while(0)

/*--- TIOA13_1_IN ---*/
#define SetPinFunc_TIOA13_1_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR13, 8u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR5_PA = 1u; }while(0)

/*--- TIOA13_1_OUT ---*/
#define SetPinFunc_TIOA13_1_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR13, 10u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR5_PA = 1u; }while(0)

/*--- TIOA13_2_IN ---*/
#define SetPinFunc_TIOA13_2_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR13, 8u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR1_PA = 1u; \
                                                 bFM_GPIO_ADE_AN10 = 0u; }while(0)

/*--- TIOA13_2_OUT ---*/
#define SetPinFunc_TIOA13_2_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR13, 10u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR1_PA = 1u; \
                                                 bFM_GPIO_ADE_AN10 = 0u; }while(0)

/*--- TIOA14_0_OUT ---*/
#define SetPinFunc_TIOA14_0_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR13, 18u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFRC_P6 = 1u; }while(0)

/*--- TIOA14_2_OUT ---*/
#define SetPinFunc_TIOA14_2_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR13, 18u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR1_PC = 1u; \
                                                 bFM_GPIO_ADE_AN12 = 0u; }while(0)

/*--- TIOA15_0_IN ---*/
#define SetPinFunc_TIOA15_0_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR13, 24u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR7_P8 = 1u; }while(0)

/*--- TIOA15_0_OUT ---*/
#define SetPinFunc_TIOA15_0_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR13, 26u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR7_P8 = 1u; }while(0)

/*--- TIOA15_2_IN ---*/
#define SetPinFunc_TIOA15_2_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR13, 24u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR1_PE = 1u; \
                                                 bFM_GPIO_ADE_AN14 = 0u; }while(0)

/*--- TIOA15_2_OUT ---*/
#define SetPinFunc_TIOA15_2_OUT(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR13, 26u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR1_PE = 1u; \
                                                 bFM_GPIO_ADE_AN14 = 0u; }while(0)

/*--- TIOB00_0_IN ---*/
#define SetPinFunc_TIOB00_0_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 4u, 3u, 0x01u ); \
                                                 bFM_GPIO_PFR4_P9 = 1u; }while(0)

/*--- TIOB00_2_IN ---*/
#define SetPinFunc_TIOB00_2_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 4u, 3u, 0x03u ); \
                                                 bFM_GPIO_PFR0_P9 = 1u; }while(0)

/*--- TIOB01_0_IN ---*/
#define SetPinFunc_TIOB01_0_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 12u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR4_PA = 1u; }while(0)

/*--- TIOB01_2_IN ---*/
#define SetPinFunc_TIOB01_2_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 12u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR5_P4 = 1u; }while(0)

/*--- TIOB02_0_IN ---*/
#define SetPinFunc_TIOB02_0_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 20u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR4_PB = 1u; }while(0)

/*--- TIOB02_2_IN ---*/
#define SetPinFunc_TIOB02_2_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 20u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR6_P1 = 1u; }while(0)

/*--- TIOB03_0_IN ---*/
#define SetPinFunc_TIOB03_0_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 28u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR4_PC = 1u; }while(0)

/*--- TIOB03_2_IN ---*/
#define SetPinFunc_TIOB03_2_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 28u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFRD_P3 = 1u; }while(0)

/*--- TIOB04_0_IN ---*/
#define SetPinFunc_TIOB04_0_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 4u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR4_PD = 1u; }while(0)

/*--- TIOB04_2_IN ---*/
#define SetPinFunc_TIOB04_2_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 4u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR7_P1 = 1u; }while(0)

/*--- TIOB05_0_IN ---*/
#define SetPinFunc_TIOB05_0_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 12u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR4_PE = 1u; }while(0)

/*--- TIOB05_2_IN ---*/
#define SetPinFunc_TIOB05_2_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 12u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR0_P6 = 1u; }while(0)

/*--- TIOB06_1_IN ---*/
#define SetPinFunc_TIOB06_1_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 20u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFRC_PE = 1u; }while(0)

/*--- TIOB07_1_IN ---*/
#define SetPinFunc_TIOB07_1_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 28u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P2 = 1u; \
                                                 bFM_GPIO_ADE_AN31 = 0u; }while(0)

/*--- TIOB07_2_IN ---*/
#define SetPinFunc_TIOB07_2_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 28u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR7_P7 = 1u; }while(0)

/*--- TIOB08_2_IN ---*/
#define SetPinFunc_TIOB08_2_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR12, 4u, 3u, 0x03u ); \
                                                 bFM_GPIO_PFRC_PF = 1u; }while(0)

/*--- TIOB09_2_IN ---*/
#define SetPinFunc_TIOB09_2_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR12, 12u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR5_P7 = 1u; }while(0)

/*--- TIOB10_2_IN ---*/
#define SetPinFunc_TIOB10_2_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR12, 20u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFRD_P0 = 1u; }while(0)

/*--- TIOB11_2_IN ---*/
#define SetPinFunc_TIOB11_2_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR12, 28u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR5_P9 = 1u; }while(0)

/*--- TIOB12_2_IN ---*/
#define SetPinFunc_TIOB12_2_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR13, 4u, 3u, 0x03u ); \
                                                 bFM_GPIO_PFR3_P7 = 1u; }while(0)

/*--- TIOB13_1_IN ---*/
#define SetPinFunc_TIOB13_1_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR13, 12u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR5_PB = 1u; }while(0)

/*--- TIOB13_2_IN ---*/
#define SetPinFunc_TIOB13_2_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR13, 12u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR1_PB = 1u; \
                                                 bFM_GPIO_ADE_AN11 = 0u; }while(0)

/*--- TIOB14_0_IN ---*/
#define SetPinFunc_TIOB14_0_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR13, 20u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFRD_P1 = 1u; }while(0)

/*--- TIOB14_2_IN ---*/
#define SetPinFunc_TIOB14_2_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR13, 20u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR1_PD = 1u; \
                                                 bFM_GPIO_ADE_AN13 = 0u; }while(0)

/*--- TIOB15_0_IN ---*/
#define SetPinFunc_TIOB15_0_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR13, 28u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR7_P9 = 1u; }while(0)

/*--- TIOB15_2_IN ---*/
#define SetPinFunc_TIOB15_2_IN(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR13, 28u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR1_PF = 1u; \
                                                 bFM_GPIO_ADE_AN15 = 0u; }while(0)

/*--- TRACECLK ---*/
#define SetPinFunc_TRACECLK(dummy)           do{ bFM_GPIO_EPFR00_TRC0E = 1; \
                                                 bFM_GPIO_PFR0_P9 = 1u; }while(0)

/*--- TRACED0 ---*/
#define SetPinFunc_TRACED0(dummy)            do{ bFM_GPIO_EPFR00_TRC0E = 1; \
                                                 bFM_GPIO_PFR0_P5 = 1u; }while(0)

/*--- TRACED1 ---*/
#define SetPinFunc_TRACED1(dummy)            do{ bFM_GPIO_EPFR00_TRC0E = 1; \
                                                 bFM_GPIO_PFR0_P6 = 1u; }while(0)

/*--- TRACED2 ---*/
#define SetPinFunc_TRACED2(dummy)            do{ bFM_GPIO_EPFR00_TRC1E = 1; \
                                                 bFM_GPIO_PFR0_P7 = 1u; }while(0)

/*--- TRACED3 ---*/
#define SetPinFunc_TRACED3(dummy)            do{ bFM_GPIO_EPFR00_TRC1E = 1; \
                                                 bFM_GPIO_PFR0_P8 = 1u; }while(0)

/*--- ZIN0_1 ---*/
#define SetPinFunc_ZIN0_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 4u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR4_PB = 1u; }while(0)

/*--- ZIN0_2 ---*/
#define SetPinFunc_ZIN0_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 4u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR5_P2 = 1u; }while(0)

/*--- ZIN1_0 ---*/
#define SetPinFunc_ZIN1_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 10u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR7_PA = 1u; }while(0)

/*--- ZIN1_1 ---*/
#define SetPinFunc_ZIN1_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 10u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P2 = 1u; \
                                                 bFM_GPIO_ADE_AN31 = 0u; }while(0)

/*--- ZIN1_2 ---*/
#define SetPinFunc_ZIN1_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 10u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR4_PE = 1u; }while(0)

/*--- ZIN2_0 ---*/
#define SetPinFunc_ZIN2_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR14, 4u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR7_P4 = 1u; }while(0)

/*--- ZIN2_1 ---*/
#define SetPinFunc_ZIN2_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR14, 4u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFRF_P5 = 1u; }while(0)


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

/*--- AN08 ---*/
#define SetPinFunc_AN08(dummy)               ( bFM_GPIO_ADE_AN08 = 1 )

/*--- AN09 ---*/
#define SetPinFunc_AN09(dummy)               ( bFM_GPIO_ADE_AN09 = 1 )

/*--- AN10 ---*/
#define SetPinFunc_AN10(dummy)               ( bFM_GPIO_ADE_AN10 = 1 )

/*--- AN11 ---*/
#define SetPinFunc_AN11(dummy)               ( bFM_GPIO_ADE_AN11 = 1 )

/*--- AN12 ---*/
#define SetPinFunc_AN12(dummy)               ( bFM_GPIO_ADE_AN12 = 1 )

/*--- AN13 ---*/
#define SetPinFunc_AN13(dummy)               ( bFM_GPIO_ADE_AN13 = 1 )

/*--- AN14 ---*/
#define SetPinFunc_AN14(dummy)               ( bFM_GPIO_ADE_AN14 = 1 )

/*--- AN15 ---*/
#define SetPinFunc_AN15(dummy)               ( bFM_GPIO_ADE_AN15 = 1 )

/*--- AN24 ---*/
#define SetPinFunc_AN24(dummy)               ( bFM_GPIO_ADE_AN24 = 1 )

/*--- AN25 ---*/
#define SetPinFunc_AN25(dummy)               ( bFM_GPIO_ADE_AN25 = 1 )

/*--- AN26 ---*/
#define SetPinFunc_AN26(dummy)               ( bFM_GPIO_ADE_AN26 = 1 )

/*--- AN27 ---*/
#define SetPinFunc_AN27(dummy)               ( bFM_GPIO_ADE_AN27 = 1 )

/*--- AN28 ---*/
#define SetPinFunc_AN28(dummy)               ( bFM_GPIO_ADE_AN28 = 1 )

/*--- AN29 ---*/
#define SetPinFunc_AN29(dummy)               ( bFM_GPIO_ADE_AN29 = 1 )

/*--- AN30 ---*/
#define SetPinFunc_AN30(dummy)               ( bFM_GPIO_ADE_AN30 = 1 )

/*--- AN31 ---*/
#define SetPinFunc_AN31(dummy)               ( bFM_GPIO_ADE_AN31 = 1 )


#endif // #ifndef __GPIO_MB9BF21XS_H__


/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/

