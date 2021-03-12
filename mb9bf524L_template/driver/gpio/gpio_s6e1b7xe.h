/******************************************************************************
* Copyright (C) 2013 Spansion LLC. All Rights Reserved.                        
*                                                                              
* This software is owned and published by:                                     
* Spansion LLC, 915 DeGuigne Dr. Sunnyvale, CA  94088-3453 ("Spansion").       
*                                                                              
* BY DOWNLOADING, INSTALLING OR USING THIS SOFTWARE, YOU AGREE TO BE BOUND     
* BY ALL THE TERMS AND CONDITIONS OF THIS AGREEMENT.                           
*                                                                              
* This software contains source code for use with Spansion                     
* components. This software is licensed by Spansion to be adapted only         
* for use in systems utilizing Spansion components. Spansion shall not be      
* responsible for misuse or illegal use of this software for devices not       
* supported herein.  Spansion is providing this software "AS IS" and will      
* not be responsible for issues arising from incorrect user implementation     
* of the software.                                                             
*                                                                              
* SPANSION MAKES NO WARRANTY, EXPRESS OR IMPLIED, ARISING BY LAW OR OTHERWISE, 
* REGARDING THE SOFTWARE (INCLUDING ANY ACOOMPANYING WRITTEN MATERIALS),       
* ITS PERFORMANCE OR SUITABILITY FOR YOUR INTENDED USE, INCLUDING,             
* WITHOUT LIMITATION, THE IMPLIED WARRANTY OF MERCHANTABILITY, THE IMPLIED     
* WARRANTY OF FITNESS FOR A PARTICULAR PURPOSE OR USE, AND THE IMPLIED         
* WARRANTY OF NONINFRINGEMENT.                                                 
* SPANSION SHALL HAVE NO LIABILITY (WHETHER IN CONTRACT, WARRANTY, TORT,       
* NEGLIGENCE OR OTHERWISE) FOR ANY DAMAGES WHATSOEVER (INCLUDING, WITHOUT      
* LIMITATION, DAMAGES FOR LOSS OF BUSINESS PROFITS, BUSINESS INTERRUPTION,     
* LOSS OF BUSINESS INFORMATION, OR OTHER PECUNIARY LOSS) ARISING FROM USE OR   
* INABILITY TO USE THE SOFTWARE, INCLUDING, WITHOUT LIMITATION, ANY DIRECT,    
* INDIRECT, INCIDENTAL, SPECIAL OR CONSEQUENTIAL DAMAGES OR LOSS OF DATA,      
* SAVINGS OR PROFITS,                                                          
* EVEN IF SPANSION HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.        
* YOU ASSUME ALL RESPONSIBILITIES FOR SELECTION OF THE SOFTWARE TO ACHIEVE YOUR
* INTENDED RESULTS, AND FOR THE INSTALLATION OF, USE OF, AND RESULTS OBTAINED  
* FROM, THE SOFTWARE.                                                          
*                                                                              
* This software may be replicated in part or whole for the licensed use,       
* with the restriction that this Disclaimer and Copyright notice must be       
* included with each copy of this software, whether used in part or whole,     
* at all times.                                                                
*/
/******************************************************************************/
/** \file gpio_s6e1b7xe.h 
 **
 ** Headerfile for S6E1B7XE GPIO functions, included in gpio.h
 **
 ** History:
 **   - 2014-12-24  1.0  Ken Xiao    First version
 **   - 2015-07-23  1.1  Nobuo Suzuki  Corrected duplication of P81.
 **   - 2015-08-17  1.2  Nobuo Suzuki  Changed P46,P47,P48,P49 to -- for VBAT configuration support.
 **
 ******************************************************************************/

#ifndef __GPIO_S6E1B7XE_H__
#define __GPIO_S6E1B7XE_H__

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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR0_P4=0u; \
                                         bFM_GPIO_PFR0_P4=0u; }while(0)

#define GPIO1PIN_NP04_INITOUT(v)     do{ bFM_GPIO_PDOR0_P4=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR0_P4=1u; \
                                         bFM_GPIO_PFR0_P4=0u; }while(0)


/*---- GPIO bit P07 ----*/

#define GPIO1PIN_P07_GET             ( bFM_GPIO_PDIR0_P7 )

#define GPIO1PIN_P07_PUT(v)          ( bFM_GPIO_PDOR0_P7=(v) )

#define GPIO1PIN_P07_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P07_INITIN(v) \
                                                        : GPIO1PIN_P07_INITOUT(v) )

#define GPIO1PIN_P07_INITIN(v)       do{ bFM_GPIO_PCR0_P7=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR0_P7=0u; \
                                         bFM_GPIO_PFR0_P7=0u; \
                                         bFM_GPIO_ADE_AN22=0u; }while(0)


#define GPIO1PIN_P07_INITOUT(v)      do{ bFM_GPIO_PDOR0_P7=(v).bInitVal; \
                                         bFM_GPIO_DDR0_P7=1u; \
                                         bFM_GPIO_PFR0_P7=0u; \
                                         bFM_GPIO_ADE_AN22=0u; }while(0)


/*---- GPIO bit NP07 ----*/
#define GPIO1PIN_NP07_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR0_P7)) )

#define GPIO1PIN_NP07_PUT(v)         ( bFM_GPIO_PDOR0_P7=(uint32_t)(!(v)) )

#define GPIO1PIN_NP07_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP07_INITIN(v) \
                                                        : GPIO1PIN_NP07_INITOUT(v) )

#define GPIO1PIN_NP07_INITIN(v)      do{ bFM_GPIO_PCR0_P7=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR0_P7=0u; \
                                         bFM_GPIO_PFR0_P7=0u; \
                                         bFM_GPIO_ADE_AN22=0u; }while(0)


#define GPIO1PIN_NP07_INITOUT(v)     do{ bFM_GPIO_PDOR0_P7=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR0_P7=1u; \
                                         bFM_GPIO_PFR0_P7=0u; \
                                         bFM_GPIO_ADE_AN22=0u; }while(0)


/*---- GPIO bit P0A ----*/

#define GPIO1PIN_P0A_GET             ( bFM_GPIO_PDIR0_PA )

#define GPIO1PIN_P0A_PUT(v)          ( bFM_GPIO_PDOR0_PA=(v) )

#define GPIO1PIN_P0A_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P0A_INITIN(v) \
                                                        : GPIO1PIN_P0A_INITOUT(v) )

#define GPIO1PIN_P0A_INITIN(v)       do{ bFM_GPIO_PCR0_PA=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR0_PA=0u; \
                                         bFM_GPIO_PFR0_PA=0u; }while(0)

#define GPIO1PIN_P0A_INITOUT(v)      do{ bFM_GPIO_PDOR0_PA=(v).bInitVal; \
                                         bFM_GPIO_DDR0_PA=1u; \
                                         bFM_GPIO_PFR0_PA=0u; }while(0)


/*---- GPIO bit NP0A ----*/
#define GPIO1PIN_NP0A_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR0_PA)) )

#define GPIO1PIN_NP0A_PUT(v)         ( bFM_GPIO_PDOR0_PA=(uint32_t)(!(v)) )

#define GPIO1PIN_NP0A_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP0A_INITIN(v) \
                                                        : GPIO1PIN_NP0A_INITOUT(v) )

#define GPIO1PIN_NP0A_INITIN(v)      do{ bFM_GPIO_PCR0_PA=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR0_PA=0u; \
                                         bFM_GPIO_PFR0_PA=0u; }while(0)

#define GPIO1PIN_NP0A_INITOUT(v)     do{ bFM_GPIO_PDOR0_PA=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR0_PA=1u; \
                                         bFM_GPIO_PFR0_PA=0u; }while(0)


/*---- GPIO bit P0B ----*/

#define GPIO1PIN_P0B_GET             ( bFM_GPIO_PDIR0_PB )

#define GPIO1PIN_P0B_PUT(v)          ( bFM_GPIO_PDOR0_PB=(v) )

#define GPIO1PIN_P0B_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P0B_INITIN(v) \
                                                        : GPIO1PIN_P0B_INITOUT(v) )

#define GPIO1PIN_P0B_INITIN(v)       do{ bFM_GPIO_PCR0_PB=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR0_PB=0u; \
                                         bFM_GPIO_PFR0_PB=0u; }while(0)

#define GPIO1PIN_P0B_INITOUT(v)      do{ bFM_GPIO_PDOR0_PB=(v).bInitVal; \
                                         bFM_GPIO_DDR0_PB=1u; \
                                         bFM_GPIO_PFR0_PB=0u; }while(0)


/*---- GPIO bit NP0B ----*/
#define GPIO1PIN_NP0B_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR0_PB)) )

#define GPIO1PIN_NP0B_PUT(v)         ( bFM_GPIO_PDOR0_PB=(uint32_t)(!(v)) )

#define GPIO1PIN_NP0B_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP0B_INITIN(v) \
                                                        : GPIO1PIN_NP0B_INITOUT(v) )

#define GPIO1PIN_NP0B_INITIN(v)      do{ bFM_GPIO_PCR0_PB=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR0_PB=0u; \
                                         bFM_GPIO_PFR0_PB=0u; }while(0)

#define GPIO1PIN_NP0B_INITOUT(v)     do{ bFM_GPIO_PDOR0_PB=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR0_PB=1u; \
                                         bFM_GPIO_PFR0_PB=0u; }while(0)


/*---- GPIO bit P0C ----*/

#define GPIO1PIN_P0C_GET             ( bFM_GPIO_PDIR0_PC )

#define GPIO1PIN_P0C_PUT(v)          ( bFM_GPIO_PDOR0_PC=(v) )

#define GPIO1PIN_P0C_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P0C_INITIN(v) \
                                                        : GPIO1PIN_P0C_INITOUT(v) )

#define GPIO1PIN_P0C_INITIN(v)       do{ bFM_GPIO_PCR0_PC=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR0_PC=0u; \
                                         bFM_GPIO_PFR0_PC=0u; }while(0)

#define GPIO1PIN_P0C_INITOUT(v)      do{ bFM_GPIO_PDOR0_PC=(v).bInitVal; \
                                         bFM_GPIO_DDR0_PC=1u; \
                                         bFM_GPIO_PFR0_PC=0u; }while(0)


/*---- GPIO bit NP0C ----*/
#define GPIO1PIN_NP0C_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR0_PC)) )

#define GPIO1PIN_NP0C_PUT(v)         ( bFM_GPIO_PDOR0_PC=(uint32_t)(!(v)) )

#define GPIO1PIN_NP0C_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP0C_INITIN(v) \
                                                        : GPIO1PIN_NP0C_INITOUT(v) )

#define GPIO1PIN_NP0C_INITIN(v)      do{ bFM_GPIO_PCR0_PC=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR0_PC=0u; \
                                         bFM_GPIO_PFR0_PC=0u; }while(0)

#define GPIO1PIN_NP0C_INITOUT(v)     do{ bFM_GPIO_PDOR0_PC=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR0_PC=1u; \
                                         bFM_GPIO_PFR0_PC=0u; }while(0)


/*---- GPIO bit P0D ----*/

#define GPIO1PIN_P0D_GET             ( bFM_GPIO_PDIR0_PD )

#define GPIO1PIN_P0D_PUT(v)          ( bFM_GPIO_PDOR0_PD=(v) )

#define GPIO1PIN_P0D_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P0D_INITIN(v) \
                                                        : GPIO1PIN_P0D_INITOUT(v) )

#define GPIO1PIN_P0D_INITIN(v)       do{ bFM_GPIO_PCR0_PD=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR0_PD=0u; \
                                         bFM_GPIO_PFR0_PD=0u; }while(0)

#define GPIO1PIN_P0D_INITOUT(v)      do{ bFM_GPIO_PDOR0_PD=(v).bInitVal; \
                                         bFM_GPIO_DDR0_PD=1u; \
                                         bFM_GPIO_PFR0_PD=0u; }while(0)


/*---- GPIO bit NP0D ----*/
#define GPIO1PIN_NP0D_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR0_PD)) )

#define GPIO1PIN_NP0D_PUT(v)         ( bFM_GPIO_PDOR0_PD=(uint32_t)(!(v)) )

#define GPIO1PIN_NP0D_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP0D_INITIN(v) \
                                                        : GPIO1PIN_NP0D_INITOUT(v) )

#define GPIO1PIN_NP0D_INITIN(v)      do{ bFM_GPIO_PCR0_PD=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR0_PD=0u; \
                                         bFM_GPIO_PFR0_PD=0u; }while(0)

#define GPIO1PIN_NP0D_INITOUT(v)     do{ bFM_GPIO_PDOR0_PD=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR0_PD=1u; \
                                         bFM_GPIO_PFR0_PD=0u; }while(0)


/*---- GPIO bit P0E ----*/

#define GPIO1PIN_P0E_GET             ( bFM_GPIO_PDIR0_PE )

#define GPIO1PIN_P0E_PUT(v)          ( bFM_GPIO_PDOR0_PE=(v) )

#define GPIO1PIN_P0E_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P0E_INITIN(v) \
                                                        : GPIO1PIN_P0E_INITOUT(v) )

#define GPIO1PIN_P0E_INITIN(v)       do{ bFM_GPIO_PCR0_PE=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR0_PE=0u; \
                                         bFM_GPIO_PFR0_PE=0u; }while(0)

#define GPIO1PIN_P0E_INITOUT(v)      do{ bFM_GPIO_PDOR0_PE=(v).bInitVal; \
                                         bFM_GPIO_DDR0_PE=1u; \
                                         bFM_GPIO_PFR0_PE=0u; }while(0)


/*---- GPIO bit NP0E ----*/
#define GPIO1PIN_NP0E_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR0_PE)) )

#define GPIO1PIN_NP0E_PUT(v)         ( bFM_GPIO_PDOR0_PE=(uint32_t)(!(v)) )

#define GPIO1PIN_NP0E_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP0E_INITIN(v) \
                                                        : GPIO1PIN_NP0E_INITOUT(v) )

#define GPIO1PIN_NP0E_INITIN(v)      do{ bFM_GPIO_PCR0_PE=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR0_PE=0u; \
                                         bFM_GPIO_PFR0_PE=0u; }while(0)

#define GPIO1PIN_NP0E_INITOUT(v)     do{ bFM_GPIO_PDOR0_PE=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR0_PE=1u; \
                                         bFM_GPIO_PFR0_PE=0u; }while(0)


/*---- GPIO bit P0F ----*/

#define GPIO1PIN_P0F_GET             ( bFM_GPIO_PDIR0_PF )

#define GPIO1PIN_P0F_PUT(v)          ( bFM_GPIO_PDOR0_PF=(v) )

#define GPIO1PIN_P0F_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P0F_INITIN(v) \
                                                        : GPIO1PIN_P0F_INITOUT(v) )

#define GPIO1PIN_P0F_INITIN(v)       do{ bFM_GPIO_PCR0_PF=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR0_PF=0u; \
                                         bFM_GPIO_PFR0_PF=0u; }while(0)

#define GPIO1PIN_P0F_INITOUT(v)      do{ bFM_GPIO_PDOR0_PF=(v).bInitVal; \
                                         bFM_GPIO_DDR0_PF=1u; \
                                         bFM_GPIO_PFR0_PF=0u; }while(0)


/*---- GPIO bit NP0F ----*/
#define GPIO1PIN_NP0F_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR0_PF)) )

#define GPIO1PIN_NP0F_PUT(v)         ( bFM_GPIO_PDOR0_PF=(uint32_t)(!(v)) )

#define GPIO1PIN_NP0F_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP0F_INITIN(v) \
                                                        : GPIO1PIN_NP0F_INITOUT(v) )

#define GPIO1PIN_NP0F_INITIN(v)      do{ bFM_GPIO_PCR0_PF=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR0_PF=0u; \
                                         bFM_GPIO_PFR0_PF=0u; }while(0)

#define GPIO1PIN_NP0F_INITOUT(v)     do{ bFM_GPIO_PDOR0_PF=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR0_PF=1u; \
                                         bFM_GPIO_PFR0_PF=0u; }while(0)


/*---- GPIO bit P10 ----*/

#define GPIO1PIN_P10_GET             ( bFM_GPIO_PDIR1_P0 )

#define GPIO1PIN_P10_PUT(v)          ( bFM_GPIO_PDOR1_P0=(v) )

#define GPIO1PIN_P10_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P10_INITIN(v) \
                                                        : GPIO1PIN_P10_INITOUT(v) )

#define GPIO1PIN_P10_INITIN(v)       do{ bFM_GPIO_PCR1_P0=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR1_PB=0u; \
                                         bFM_GPIO_PFR1_PB=0u; \
                                         bFM_GPIO_ADE_AN11=0u; }while(0)


#define GPIO1PIN_NP1B_INITOUT(v)     do{ bFM_GPIO_PDOR1_PB=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR1_PB=1u; \
                                         bFM_GPIO_PFR1_PB=0u; \
                                         bFM_GPIO_ADE_AN11=0u; }while(0)


/*---- GPIO bit P21 ----*/

#define GPIO1PIN_P21_GET             ( bFM_GPIO_PDIR2_P1 )

#define GPIO1PIN_P21_PUT(v)          ( bFM_GPIO_PDOR2_P1=(v) )

#define GPIO1PIN_P21_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P21_INITIN(v) \
                                                        : GPIO1PIN_P21_INITOUT(v) )

#define GPIO1PIN_P21_INITIN(v)       do{ bFM_GPIO_PCR2_P1=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR2_P1=0u; \
                                         bFM_GPIO_PFR2_P1=0u; \
                                         bFM_GPIO_ADE_AN18=0u; }while(0)


#define GPIO1PIN_P21_INITOUT(v)      do{ bFM_GPIO_PDOR2_P1=(v).bInitVal; \
                                         bFM_GPIO_DDR2_P1=1u; \
                                         bFM_GPIO_PFR2_P1=0u; \
                                         bFM_GPIO_ADE_AN18=0u; }while(0)


/*---- GPIO bit NP21 ----*/
#define GPIO1PIN_NP21_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR2_P1)) )

#define GPIO1PIN_NP21_PUT(v)         ( bFM_GPIO_PDOR2_P1=(uint32_t)(!(v)) )

#define GPIO1PIN_NP21_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP21_INITIN(v) \
                                                        : GPIO1PIN_NP21_INITOUT(v) )

#define GPIO1PIN_NP21_INITIN(v)      do{ bFM_GPIO_PCR2_P1=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR2_P1=0u; \
                                         bFM_GPIO_PFR2_P1=0u; \
                                         bFM_GPIO_ADE_AN18=0u; }while(0)


#define GPIO1PIN_NP21_INITOUT(v)     do{ bFM_GPIO_PDOR2_P1=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR2_P1=1u; \
                                         bFM_GPIO_PFR2_P1=0u; \
                                         bFM_GPIO_ADE_AN18=0u; }while(0)


/*---- GPIO bit P22 ----*/

#define GPIO1PIN_P22_GET             ( bFM_GPIO_PDIR2_P2 )

#define GPIO1PIN_P22_PUT(v)          ( bFM_GPIO_PDOR2_P2=(v) )

#define GPIO1PIN_P22_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P22_INITIN(v) \
                                                        : GPIO1PIN_P22_INITOUT(v) )

#define GPIO1PIN_P22_INITIN(v)       do{ bFM_GPIO_PCR2_P2=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR2_P2=0u; \
                                         bFM_GPIO_PFR2_P2=0u; \
                                         bFM_GPIO_ADE_AN17=0u; }while(0)


#define GPIO1PIN_P22_INITOUT(v)      do{ bFM_GPIO_PDOR2_P2=(v).bInitVal; \
                                         bFM_GPIO_DDR2_P2=1u; \
                                         bFM_GPIO_PFR2_P2=0u; \
                                         bFM_GPIO_ADE_AN17=0u; }while(0)


/*---- GPIO bit NP22 ----*/
#define GPIO1PIN_NP22_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR2_P2)) )

#define GPIO1PIN_NP22_PUT(v)         ( bFM_GPIO_PDOR2_P2=(uint32_t)(!(v)) )

#define GPIO1PIN_NP22_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP22_INITIN(v) \
                                                        : GPIO1PIN_NP22_INITOUT(v) )

#define GPIO1PIN_NP22_INITIN(v)      do{ bFM_GPIO_PCR2_P2=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR2_P2=0u; \
                                         bFM_GPIO_PFR2_P2=0u; \
                                         bFM_GPIO_ADE_AN17=0u; }while(0)


#define GPIO1PIN_NP22_INITOUT(v)     do{ bFM_GPIO_PDOR2_P2=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR2_P2=1u; \
                                         bFM_GPIO_PFR2_P2=0u; \
                                         bFM_GPIO_ADE_AN17=0u; }while(0)


/*---- GPIO bit P23 ----*/

#define GPIO1PIN_P23_GET             ( bFM_GPIO_PDIR2_P3 )

#define GPIO1PIN_P23_PUT(v)          ( bFM_GPIO_PDOR2_P3=(v) )

#define GPIO1PIN_P23_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P23_INITIN(v) \
                                                        : GPIO1PIN_P23_INITOUT(v) )

#define GPIO1PIN_P23_INITIN(v)       do{ bFM_GPIO_PCR2_P3=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR2_P3=0u; \
                                         bFM_GPIO_PFR2_P3=0u; \
                                         bFM_GPIO_ADE_AN16=0u; }while(0)


#define GPIO1PIN_P23_INITOUT(v)      do{ bFM_GPIO_PDOR2_P3=(v).bInitVal; \
                                         bFM_GPIO_DDR2_P3=1u; \
                                         bFM_GPIO_PFR2_P3=0u; \
                                         bFM_GPIO_ADE_AN16=0u; }while(0)


/*---- GPIO bit NP23 ----*/
#define GPIO1PIN_NP23_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR2_P3)) )

#define GPIO1PIN_NP23_PUT(v)         ( bFM_GPIO_PDOR2_P3=(uint32_t)(!(v)) )

#define GPIO1PIN_NP23_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP23_INITIN(v) \
                                                        : GPIO1PIN_NP23_INITOUT(v) )

#define GPIO1PIN_NP23_INITIN(v)      do{ bFM_GPIO_PCR2_P3=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR2_P3=0u; \
                                         bFM_GPIO_PFR2_P3=0u; \
                                         bFM_GPIO_ADE_AN16=0u; }while(0)


#define GPIO1PIN_NP23_INITOUT(v)     do{ bFM_GPIO_PDOR2_P3=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR2_P3=1u; \
                                         bFM_GPIO_PFR2_P3=0u; \
                                         bFM_GPIO_ADE_AN16=0u; }while(0)


/*---- GPIO bit P30 ----*/

#define GPIO1PIN_P30_GET             ( bFM_GPIO_PDIR3_P0 )

#define GPIO1PIN_P30_PUT(v)          ( bFM_GPIO_PDOR3_P0=(v) )

#define GPIO1PIN_P30_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P30_INITIN(v) \
                                                        : GPIO1PIN_P30_INITOUT(v) )

#define GPIO1PIN_P30_INITIN(v)       do{ bFM_GPIO_PCR3_P0=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR3_P0=0u; \
                                         bFM_GPIO_PFR3_P0=0u; }while(0)

#define GPIO1PIN_P30_INITOUT(v)      do{ bFM_GPIO_PDOR3_P0=(v).bInitVal; \
                                         bFM_GPIO_DDR3_P0=1u; \
                                         bFM_GPIO_PFR3_P0=0u; }while(0)


/*---- GPIO bit NP30 ----*/
#define GPIO1PIN_NP30_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR3_P0)) )

#define GPIO1PIN_NP30_PUT(v)         ( bFM_GPIO_PDOR3_P0=(uint32_t)(!(v)) )

#define GPIO1PIN_NP30_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP30_INITIN(v) \
                                                        : GPIO1PIN_NP30_INITOUT(v) )

#define GPIO1PIN_NP30_INITIN(v)      do{ bFM_GPIO_PCR3_P0=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR3_P0=0u; \
                                         bFM_GPIO_PFR3_P0=0u; }while(0)

#define GPIO1PIN_NP30_INITOUT(v)     do{ bFM_GPIO_PDOR3_P0=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR3_P0=1u; \
                                         bFM_GPIO_PFR3_P0=0u; }while(0)


/*---- GPIO bit P31 ----*/

#define GPIO1PIN_P31_GET             ( bFM_GPIO_PDIR3_P1 )

#define GPIO1PIN_P31_PUT(v)          ( bFM_GPIO_PDOR3_P1=(v) )

#define GPIO1PIN_P31_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P31_INITIN(v) \
                                                        : GPIO1PIN_P31_INITOUT(v) )

#define GPIO1PIN_P31_INITIN(v)       do{ bFM_GPIO_PCR3_P1=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR3_P1=0u; \
                                         bFM_GPIO_PFR3_P1=0u; }while(0)

#define GPIO1PIN_P31_INITOUT(v)      do{ bFM_GPIO_PDOR3_P1=(v).bInitVal; \
                                         bFM_GPIO_DDR3_P1=1u; \
                                         bFM_GPIO_PFR3_P1=0u; }while(0)


/*---- GPIO bit NP31 ----*/
#define GPIO1PIN_NP31_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR3_P1)) )

#define GPIO1PIN_NP31_PUT(v)         ( bFM_GPIO_PDOR3_P1=(uint32_t)(!(v)) )

#define GPIO1PIN_NP31_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP31_INITIN(v) \
                                                        : GPIO1PIN_NP31_INITOUT(v) )

#define GPIO1PIN_NP31_INITIN(v)      do{ bFM_GPIO_PCR3_P1=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR3_P1=0u; \
                                         bFM_GPIO_PFR3_P1=0u; }while(0)

#define GPIO1PIN_NP31_INITOUT(v)     do{ bFM_GPIO_PDOR3_P1=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR3_P1=1u; \
                                         bFM_GPIO_PFR3_P1=0u; }while(0)


/*---- GPIO bit P32 ----*/

#define GPIO1PIN_P32_GET             ( bFM_GPIO_PDIR3_P2 )

#define GPIO1PIN_P32_PUT(v)          ( bFM_GPIO_PDOR3_P2=(v) )

#define GPIO1PIN_P32_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P32_INITIN(v) \
                                                        : GPIO1PIN_P32_INITOUT(v) )

#define GPIO1PIN_P32_INITIN(v)       do{ bFM_GPIO_PCR3_P2=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR3_P2=0u; \
                                         bFM_GPIO_PFR3_P2=0u; }while(0)

#define GPIO1PIN_P32_INITOUT(v)      do{ bFM_GPIO_PDOR3_P2=(v).bInitVal; \
                                         bFM_GPIO_DDR3_P2=1u; \
                                         bFM_GPIO_PFR3_P2=0u; }while(0)


/*---- GPIO bit NP32 ----*/
#define GPIO1PIN_NP32_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR3_P2)) )

#define GPIO1PIN_NP32_PUT(v)         ( bFM_GPIO_PDOR3_P2=(uint32_t)(!(v)) )

#define GPIO1PIN_NP32_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP32_INITIN(v) \
                                                        : GPIO1PIN_NP32_INITOUT(v) )

#define GPIO1PIN_NP32_INITIN(v)      do{ bFM_GPIO_PCR3_P2=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR3_P2=0u; \
                                         bFM_GPIO_PFR3_P2=0u; }while(0)

#define GPIO1PIN_NP32_INITOUT(v)     do{ bFM_GPIO_PDOR3_P2=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR3_P2=1u; \
                                         bFM_GPIO_PFR3_P2=0u; }while(0)


/*---- GPIO bit P33 ----*/

#define GPIO1PIN_P33_GET             ( bFM_GPIO_PDIR3_P3 )

#define GPIO1PIN_P33_PUT(v)          ( bFM_GPIO_PDOR3_P3=(v) )

#define GPIO1PIN_P33_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P33_INITIN(v) \
                                                        : GPIO1PIN_P33_INITOUT(v) )

#define GPIO1PIN_P33_INITIN(v)       do{ bFM_GPIO_PCR3_P3=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR3_P3=0u; \
                                         bFM_GPIO_PFR3_P3=0u; }while(0)

#define GPIO1PIN_P33_INITOUT(v)      do{ bFM_GPIO_PDOR3_P3=(v).bInitVal; \
                                         bFM_GPIO_DDR3_P3=1u; \
                                         bFM_GPIO_PFR3_P3=0u; }while(0)


/*---- GPIO bit NP33 ----*/
#define GPIO1PIN_NP33_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR3_P3)) )

#define GPIO1PIN_NP33_PUT(v)         ( bFM_GPIO_PDOR3_P3=(uint32_t)(!(v)) )

#define GPIO1PIN_NP33_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP33_INITIN(v) \
                                                        : GPIO1PIN_NP33_INITOUT(v) )

#define GPIO1PIN_NP33_INITIN(v)      do{ bFM_GPIO_PCR3_P3=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR3_P3=0u; \
                                         bFM_GPIO_PFR3_P3=0u; }while(0)

#define GPIO1PIN_NP33_INITOUT(v)     do{ bFM_GPIO_PDOR3_P3=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR3_P3=1u; \
                                         bFM_GPIO_PFR3_P3=0u; }while(0)


/*---- GPIO bit P39 ----*/

#define GPIO1PIN_P39_GET             ( bFM_GPIO_PDIR3_P9 )

#define GPIO1PIN_P39_PUT(v)          ( bFM_GPIO_PDOR3_P9=(v) )

#define GPIO1PIN_P39_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P39_INITIN(v) \
                                                        : GPIO1PIN_P39_INITOUT(v) )

#define GPIO1PIN_P39_INITIN(v)       do{ bFM_GPIO_PCR3_P9=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR3_PF=0u; \
                                         bFM_GPIO_PFR3_PF=0u; }while(0)

#define GPIO1PIN_NP3F_INITOUT(v)     do{ bFM_GPIO_PDOR3_PF=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR3_PF=1u; \
                                         bFM_GPIO_PFR3_PF=0u; }while(0)


/*---- GPIO bit P44 ----*/

#define GPIO1PIN_P44_GET             ( bFM_GPIO_PDIR4_P4 )

#define GPIO1PIN_P44_PUT(v)          ( bFM_GPIO_PDOR4_P4=(v) )

#define GPIO1PIN_P44_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P44_INITIN(v) \
                                                        : GPIO1PIN_P44_INITOUT(v) )

#define GPIO1PIN_P44_INITIN(v)       do{ bFM_GPIO_PCR4_P4=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR4_P5=0u; \
                                         bFM_GPIO_PFR4_P5=0u; }while(0)

#define GPIO1PIN_NP45_INITOUT(v)     do{ bFM_GPIO_PDOR4_P5=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR4_P5=1u; \
                                         bFM_GPIO_PFR4_P5=0u; }while(0)


/*---- GPIO bit P4A ----*/

#define GPIO1PIN_P4A_GET             ( bFM_GPIO_PDIR4_PA )

#define GPIO1PIN_P4A_PUT(v)          ( bFM_GPIO_PDOR4_PA=(v) )

#define GPIO1PIN_P4A_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P4A_INITIN(v) \
                                                        : GPIO1PIN_P4A_INITOUT(v) )

#define GPIO1PIN_P4A_INITIN(v)       do{ bFM_GPIO_PCR4_PA=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR4_PD=0u; \
                                         bFM_GPIO_PFR4_PD=0u; }while(0)

#define GPIO1PIN_NP4D_INITOUT(v)     do{ bFM_GPIO_PDOR4_PD=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR4_PD=1u; \
                                         bFM_GPIO_PFR4_PD=0u; }while(0)


/*---- GPIO bit P50 ----*/

#define GPIO1PIN_P50_GET             ( bFM_GPIO_PDIR5_P0 )

#define GPIO1PIN_P50_PUT(v)          ( bFM_GPIO_PDOR5_P0=(v) )

#define GPIO1PIN_P50_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P50_INITIN(v) \
                                                        : GPIO1PIN_P50_INITOUT(v) )

#define GPIO1PIN_P50_INITIN(v)       do{ bFM_GPIO_PCR5_P0=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR5_P0=0u; \
                                         bFM_GPIO_PFR5_P0=0u; \
                                         bFM_LCDC_LCDCC3_VE4=0u; }while(0)


#define GPIO1PIN_P50_INITOUT(v)      do{ bFM_GPIO_PDOR5_P0=(v).bInitVal; \
                                         bFM_GPIO_DDR5_P0=1u; \
                                         bFM_GPIO_PFR5_P0=0u; \
                                         bFM_LCDC_LCDCC3_VE4=0u; }while(0)


/*---- GPIO bit NP50 ----*/
#define GPIO1PIN_NP50_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR5_P0)) )

#define GPIO1PIN_NP50_PUT(v)         ( bFM_GPIO_PDOR5_P0=(uint32_t)(!(v)) )

#define GPIO1PIN_NP50_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP50_INITIN(v) \
                                                        : GPIO1PIN_NP50_INITOUT(v) )

#define GPIO1PIN_NP50_INITIN(v)      do{ bFM_GPIO_PCR5_P0=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR5_P0=0u; \
                                         bFM_GPIO_PFR5_P0=0u; \
                                         bFM_LCDC_LCDCC3_VE4=0u; }while(0)


#define GPIO1PIN_NP50_INITOUT(v)     do{ bFM_GPIO_PDOR5_P0=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR5_P0=1u; \
                                         bFM_GPIO_PFR5_P0=0u; \
                                         bFM_LCDC_LCDCC3_VE4=0u; }while(0)


/*---- GPIO bit P51 ----*/

#define GPIO1PIN_P51_GET             ( bFM_GPIO_PDIR5_P1 )

#define GPIO1PIN_P51_PUT(v)          ( bFM_GPIO_PDOR5_P1=(v) )

#define GPIO1PIN_P51_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P51_INITIN(v) \
                                                        : GPIO1PIN_P51_INITOUT(v) )

#define GPIO1PIN_P51_INITIN(v)       do{ bFM_GPIO_PCR5_P1=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR5_P1=0u; \
                                         bFM_GPIO_PFR5_P1=0u; \
                                         bFM_LCDC_LCDCC3_VE3=0u; }while(0)


#define GPIO1PIN_P51_INITOUT(v)      do{ bFM_GPIO_PDOR5_P1=(v).bInitVal; \
                                         bFM_GPIO_DDR5_P1=1u; \
                                         bFM_GPIO_PFR5_P1=0u; \
                                         bFM_LCDC_LCDCC3_VE3=0u; }while(0)


/*---- GPIO bit NP51 ----*/
#define GPIO1PIN_NP51_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR5_P1)) )

#define GPIO1PIN_NP51_PUT(v)         ( bFM_GPIO_PDOR5_P1=(uint32_t)(!(v)) )

#define GPIO1PIN_NP51_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP51_INITIN(v) \
                                                        : GPIO1PIN_NP51_INITOUT(v) )

#define GPIO1PIN_NP51_INITIN(v)      do{ bFM_GPIO_PCR5_P1=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR5_P1=0u; \
                                         bFM_GPIO_PFR5_P1=0u; \
                                         bFM_LCDC_LCDCC3_VE3=0u; }while(0)


#define GPIO1PIN_NP51_INITOUT(v)     do{ bFM_GPIO_PDOR5_P1=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR5_P1=1u; \
                                         bFM_GPIO_PFR5_P1=0u; \
                                         bFM_LCDC_LCDCC3_VE3=0u; }while(0)


/*---- GPIO bit P52 ----*/

#define GPIO1PIN_P52_GET             ( bFM_GPIO_PDIR5_P2 )

#define GPIO1PIN_P52_PUT(v)          ( bFM_GPIO_PDOR5_P2=(v) )

#define GPIO1PIN_P52_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P52_INITIN(v) \
                                                        : GPIO1PIN_P52_INITOUT(v) )

#define GPIO1PIN_P52_INITIN(v)       do{ bFM_GPIO_PCR5_P2=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR5_P2=0u; \
                                         bFM_GPIO_PFR5_P2=0u; \
                                         bFM_LCDC_LCDCC3_VE2=0u; }while(0)


#define GPIO1PIN_P52_INITOUT(v)      do{ bFM_GPIO_PDOR5_P2=(v).bInitVal; \
                                         bFM_GPIO_DDR5_P2=1u; \
                                         bFM_GPIO_PFR5_P2=0u; \
                                         bFM_LCDC_LCDCC3_VE2=0u; }while(0)


/*---- GPIO bit NP52 ----*/
#define GPIO1PIN_NP52_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR5_P2)) )

#define GPIO1PIN_NP52_PUT(v)         ( bFM_GPIO_PDOR5_P2=(uint32_t)(!(v)) )

#define GPIO1PIN_NP52_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP52_INITIN(v) \
                                                        : GPIO1PIN_NP52_INITOUT(v) )

#define GPIO1PIN_NP52_INITIN(v)      do{ bFM_GPIO_PCR5_P2=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR5_P2=0u; \
                                         bFM_GPIO_PFR5_P2=0u; \
                                         bFM_LCDC_LCDCC3_VE2=0u; }while(0)


#define GPIO1PIN_NP52_INITOUT(v)     do{ bFM_GPIO_PDOR5_P2=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR5_P2=1u; \
                                         bFM_GPIO_PFR5_P2=0u; \
                                         bFM_LCDC_LCDCC3_VE2=0u; }while(0)


/*---- GPIO bit P53 ----*/

#define GPIO1PIN_P53_GET             ( bFM_GPIO_PDIR5_P3 )

#define GPIO1PIN_P53_PUT(v)          ( bFM_GPIO_PDOR5_P3=(v) )

#define GPIO1PIN_P53_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P53_INITIN(v) \
                                                        : GPIO1PIN_P53_INITOUT(v) )

#define GPIO1PIN_P53_INITIN(v)       do{ bFM_GPIO_PCR5_P3=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR5_P3=0u; \
                                         bFM_GPIO_PFR5_P3=0u; \
                                         bFM_LCDC_LCDCC3_VE1=0u; }while(0)


#define GPIO1PIN_P53_INITOUT(v)      do{ bFM_GPIO_PDOR5_P3=(v).bInitVal; \
                                         bFM_GPIO_DDR5_P3=1u; \
                                         bFM_GPIO_PFR5_P3=0u; \
                                         bFM_LCDC_LCDCC3_VE1=0u; }while(0)


/*---- GPIO bit NP53 ----*/
#define GPIO1PIN_NP53_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR5_P3)) )

#define GPIO1PIN_NP53_PUT(v)         ( bFM_GPIO_PDOR5_P3=(uint32_t)(!(v)) )

#define GPIO1PIN_NP53_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP53_INITIN(v) \
                                                        : GPIO1PIN_NP53_INITOUT(v) )

#define GPIO1PIN_NP53_INITIN(v)      do{ bFM_GPIO_PCR5_P3=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR5_P3=0u; \
                                         bFM_GPIO_PFR5_P3=0u; \
                                         bFM_LCDC_LCDCC3_VE1=0u; }while(0)


#define GPIO1PIN_NP53_INITOUT(v)     do{ bFM_GPIO_PDOR5_P3=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR5_P3=1u; \
                                         bFM_GPIO_PFR5_P3=0u; \
                                         bFM_LCDC_LCDCC3_VE1=0u; }while(0)


/*---- GPIO bit P54 ----*/

#define GPIO1PIN_P54_GET             ( bFM_GPIO_PDIR5_P4 )

#define GPIO1PIN_P54_PUT(v)          ( bFM_GPIO_PDOR5_P4=(v) )

#define GPIO1PIN_P54_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P54_INITIN(v) \
                                                        : GPIO1PIN_P54_INITOUT(v) )

#define GPIO1PIN_P54_INITIN(v)       do{ bFM_GPIO_PCR5_P4=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR5_P4=0u; \
                                         bFM_GPIO_PFR5_P4=0u; \
                                         bFM_LCDC_LCDCC3_VE0=0u; }while(0)


#define GPIO1PIN_P54_INITOUT(v)      do{ bFM_GPIO_PDOR5_P4=(v).bInitVal; \
                                         bFM_GPIO_DDR5_P4=1u; \
                                         bFM_GPIO_PFR5_P4=0u; \
                                         bFM_LCDC_LCDCC3_VE0=0u; }while(0)


/*---- GPIO bit NP54 ----*/
#define GPIO1PIN_NP54_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR5_P4)) )

#define GPIO1PIN_NP54_PUT(v)         ( bFM_GPIO_PDOR5_P4=(uint32_t)(!(v)) )

#define GPIO1PIN_NP54_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP54_INITIN(v) \
                                                        : GPIO1PIN_NP54_INITOUT(v) )

#define GPIO1PIN_NP54_INITIN(v)      do{ bFM_GPIO_PCR5_P4=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR5_P4=0u; \
                                         bFM_GPIO_PFR5_P4=0u; \
                                         bFM_LCDC_LCDCC3_VE0=0u; }while(0)


#define GPIO1PIN_NP54_INITOUT(v)     do{ bFM_GPIO_PDOR5_P4=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR5_P4=1u; \
                                         bFM_GPIO_PFR5_P4=0u; \
                                         bFM_LCDC_LCDCC3_VE0=0u; }while(0)


/*---- GPIO bit P55 ----*/

#define GPIO1PIN_P55_GET             ( bFM_GPIO_PDIR5_P5 )

#define GPIO1PIN_P55_PUT(v)          ( bFM_GPIO_PDOR5_P5=(v) )

#define GPIO1PIN_P55_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P55_INITIN(v) \
                                                        : GPIO1PIN_P55_INITOUT(v) )

#define GPIO1PIN_P55_INITIN(v)       do{ bFM_GPIO_PCR5_P5=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR5_P6=0u; \
                                         bFM_GPIO_PFR5_P6=0u; }while(0)

#define GPIO1PIN_NP56_INITOUT(v)     do{ bFM_GPIO_PDOR5_P6=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR5_P6=1u; \
                                         bFM_GPIO_PFR5_P6=0u; }while(0)


/*---- GPIO bit P60 ----*/

#define GPIO1PIN_P60_GET             ( bFM_GPIO_PDIR6_P0 )

#define GPIO1PIN_P60_PUT(v)          ( bFM_GPIO_PDOR6_P0=(v) )

#define GPIO1PIN_P60_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P60_INITIN(v) \
                                                        : GPIO1PIN_P60_INITOUT(v) )

#define GPIO1PIN_P60_INITIN(v)       do{ bFM_GPIO_PCR6_P0=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR6_P2=0u; \
                                         bFM_GPIO_PFR6_P2=0u; }while(0)

#define GPIO1PIN_NP62_INITOUT(v)     do{ bFM_GPIO_PDOR6_P2=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR6_P2=1u; \
                                         bFM_GPIO_PFR6_P2=0u; }while(0)


/*---- GPIO bit P63 ----*/

#define GPIO1PIN_P63_GET             ( bFM_GPIO_PDIR6_P3 )

#define GPIO1PIN_P63_PUT(v)          ( bFM_GPIO_PDOR6_P3=(v) )

#define GPIO1PIN_P63_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P63_INITIN(v) \
                                                        : GPIO1PIN_P63_INITOUT(v) )

#define GPIO1PIN_P63_INITIN(v)       do{ bFM_GPIO_PCR6_P3=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR6_P3=0u; \
                                         bFM_GPIO_PFR6_P3=0u; }while(0)

#define GPIO1PIN_P63_INITOUT(v)      do{ bFM_GPIO_PDOR6_P3=(v).bInitVal; \
                                         bFM_GPIO_DDR6_P3=1u; \
                                         bFM_GPIO_PFR6_P3=0u; }while(0)


/*---- GPIO bit NP63 ----*/
#define GPIO1PIN_NP63_GET            ( (uint32_t)(!(uint32_t)(bFM_GPIO_PDIR6_P3)) )

#define GPIO1PIN_NP63_PUT(v)         ( bFM_GPIO_PDOR6_P3=(uint32_t)(!(v)) )

#define GPIO1PIN_NP63_INIT(v)        ( (0==(v).bOutput) ? GPIO1PIN_NP63_INITIN(v) \
                                                        : GPIO1PIN_NP63_INITOUT(v) )

#define GPIO1PIN_NP63_INITIN(v)      do{ bFM_GPIO_PCR6_P3=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR6_P3=0u; \
                                         bFM_GPIO_PFR6_P3=0u; }while(0)

#define GPIO1PIN_NP63_INITOUT(v)     do{ bFM_GPIO_PDOR6_P3=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR6_P3=1u; \
                                         bFM_GPIO_PFR6_P3=0u; }while(0)


/*---- GPIO bit P80 ----*/

#define GPIO1PIN_P80_GET             ( bFM_GPIO_PDIR8_P0 )

#define GPIO1PIN_P80_PUT(v)          ( bFM_GPIO_PDOR8_P0=(v) )

#define GPIO1PIN_P80_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_P80_INITIN(v) \
                                                        : GPIO1PIN_P80_INITOUT(v) )

#define GPIO1PIN_P80_INITIN(v)       do{ bFM_GPIO_PCR8_P0=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDR8_P2=0u; \
                                         bFM_GPIO_PFR8_P2=0u; }while(0)

#define GPIO1PIN_NP82_INITOUT(v)     do{ bFM_GPIO_PDOR8_P2=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDR8_P2=1u; \
                                         bFM_GPIO_PFR8_P2=0u; }while(0)


/*---- GPIO bit PE0 ----*/

#define GPIO1PIN_PE0_GET             ( bFM_GPIO_PDIRE_P0 )

#define GPIO1PIN_PE0_PUT(v)          ( bFM_GPIO_PDORE_P0=(v) )

#define GPIO1PIN_PE0_INIT(v)         ( (0==(v).bOutput) ? GPIO1PIN_PE0_INITIN(v) \
                                                        : GPIO1PIN_PE0_INITOUT(v) )

#define GPIO1PIN_PE0_INITIN(v)       do{ bFM_GPIO_PCRE_P0=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
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
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                         bFM_GPIO_DDRE_P3=0u; \
                                         bFM_GPIO_SPSR_MAINXC0=0u; \
                                         bFM_GPIO_SPSR_MAINXC1=0u; \
                                         bFM_GPIO_PFRE_P3=0u; }while(0)

#define GPIO1PIN_NPE3_INITOUT(v)     do{ bFM_GPIO_PDORE_P3=(uint32_t)(!((uint32_t)(v).bInitVal)); \
                                         bFM_GPIO_DDRE_P3=1u; \
                                         bFM_GPIO_SPSR_MAINXC0=0u; \
                                         bFM_GPIO_SPSR_MAINXC1=0u; \
                                         bFM_GPIO_PFRE_P3=0u; }while(0)


/******************************************************************************
   Fast GPIO
*******************************************************************************/

/*---- Fast GPIO bit P00 ----*/

#define FGPIO1PIN_P00_GET             ( FM_FASTIO->FPDIR0_f.P0 )

#define FGPIO1PIN_P00_PUT(v)          ( bFM_GPIO_FPDOR0_P0=(v) )

#define FGPIO1PIN_P00_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P00_INITIN(v) \
                                                         : FGPIO1PIN_P00_INITOUT(v) )

#define FGPIO1PIN_P00_INITIN(v)       do{ bFM_GPIO_PCR0_P0=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR0_P0=0u; \
                                          bFM_GPIO_PFR0_P0=0u; }while(0)

#define FGPIO1PIN_P00_INITOUT(v)      do{ bFM_GPIO_FPDOR0_P0=(v).bInitVal; \
                                          bFM_GPIO_DDR0_P0=1u; \
                                          bFM_GPIO_PFR0_P0=0u; }while(0)


/*---- Fast GPIO bit NP00 ----*/
#define FGPIO1PIN_NP00_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR0_f.P0)) )

#define FGPIO1PIN_NP00_PUT(v)         ( bFM_GPIO_FPDOR0_P0=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP00_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP00_INITIN(v) \
                                                         : FGPIO1PIN_NP00_INITOUT(v) )

#define FGPIO1PIN_NP00_INITIN(v)      do{ bFM_GPIO_PCR0_P0=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR0_P0=0u; \
                                          bFM_GPIO_PFR0_P0=0u; }while(0)

#define FGPIO1PIN_NP00_INITOUT(v)     do{ bFM_GPIO_PDOR0_P0=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR0_P0=1u; \
                                          bFM_GPIO_PFR0_P0=0u; }while(0)


/*---- Fast GPIO bit P01 ----*/

#define FGPIO1PIN_P01_GET             ( FM_FASTIO->FPDIR0_f.P1 )

#define FGPIO1PIN_P01_PUT(v)          ( bFM_GPIO_FPDOR0_P1=(v) )

#define FGPIO1PIN_P01_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P01_INITIN(v) \
                                                         : FGPIO1PIN_P01_INITOUT(v) )

#define FGPIO1PIN_P01_INITIN(v)       do{ bFM_GPIO_PCR0_P1=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR0_P1=0u; \
                                          bFM_GPIO_PFR0_P1=0u; }while(0)

#define FGPIO1PIN_P01_INITOUT(v)      do{ bFM_GPIO_FPDOR0_P1=(v).bInitVal; \
                                          bFM_GPIO_DDR0_P1=1u; \
                                          bFM_GPIO_PFR0_P1=0u; }while(0)


/*---- Fast GPIO bit NP01 ----*/
#define FGPIO1PIN_NP01_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR0_f.P1)) )

#define FGPIO1PIN_NP01_PUT(v)         ( bFM_GPIO_FPDOR0_P1=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP01_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP01_INITIN(v) \
                                                         : FGPIO1PIN_NP01_INITOUT(v) )

#define FGPIO1PIN_NP01_INITIN(v)      do{ bFM_GPIO_PCR0_P1=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR0_P1=0u; \
                                          bFM_GPIO_PFR0_P1=0u; }while(0)

#define FGPIO1PIN_NP01_INITOUT(v)     do{ bFM_GPIO_PDOR0_P1=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR0_P1=1u; \
                                          bFM_GPIO_PFR0_P1=0u; }while(0)


/*---- Fast GPIO bit P02 ----*/

#define FGPIO1PIN_P02_GET             ( FM_FASTIO->FPDIR0_f.P2 )

#define FGPIO1PIN_P02_PUT(v)          ( bFM_GPIO_FPDOR0_P2=(v) )

#define FGPIO1PIN_P02_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P02_INITIN(v) \
                                                         : FGPIO1PIN_P02_INITOUT(v) )

#define FGPIO1PIN_P02_INITIN(v)       do{ bFM_GPIO_PCR0_P2=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR0_P2=0u; \
                                          bFM_GPIO_PFR0_P2=0u; }while(0)

#define FGPIO1PIN_P02_INITOUT(v)      do{ bFM_GPIO_FPDOR0_P2=(v).bInitVal; \
                                          bFM_GPIO_DDR0_P2=1u; \
                                          bFM_GPIO_PFR0_P2=0u; }while(0)


/*---- Fast GPIO bit NP02 ----*/
#define FGPIO1PIN_NP02_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR0_f.P2)) )

#define FGPIO1PIN_NP02_PUT(v)         ( bFM_GPIO_FPDOR0_P2=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP02_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP02_INITIN(v) \
                                                         : FGPIO1PIN_NP02_INITOUT(v) )

#define FGPIO1PIN_NP02_INITIN(v)      do{ bFM_GPIO_PCR0_P2=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR0_P2=0u; \
                                          bFM_GPIO_PFR0_P2=0u; }while(0)

#define FGPIO1PIN_NP02_INITOUT(v)     do{ bFM_GPIO_PDOR0_P2=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR0_P2=1u; \
                                          bFM_GPIO_PFR0_P2=0u; }while(0)


/*---- Fast GPIO bit P03 ----*/

#define FGPIO1PIN_P03_GET             ( FM_FASTIO->FPDIR0_f.P3 )

#define FGPIO1PIN_P03_PUT(v)          ( bFM_GPIO_FPDOR0_P3=(v) )

#define FGPIO1PIN_P03_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P03_INITIN(v) \
                                                         : FGPIO1PIN_P03_INITOUT(v) )

#define FGPIO1PIN_P03_INITIN(v)       do{ bFM_GPIO_PCR0_P3=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR0_P3=0u; \
                                          bFM_GPIO_PFR0_P3=0u; }while(0)

#define FGPIO1PIN_P03_INITOUT(v)      do{ bFM_GPIO_FPDOR0_P3=(v).bInitVal; \
                                          bFM_GPIO_DDR0_P3=1u; \
                                          bFM_GPIO_PFR0_P3=0u; }while(0)


/*---- Fast GPIO bit NP03 ----*/
#define FGPIO1PIN_NP03_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR0_f.P3)) )

#define FGPIO1PIN_NP03_PUT(v)         ( bFM_GPIO_FPDOR0_P3=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP03_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP03_INITIN(v) \
                                                         : FGPIO1PIN_NP03_INITOUT(v) )

#define FGPIO1PIN_NP03_INITIN(v)      do{ bFM_GPIO_PCR0_P3=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR0_P3=0u; \
                                          bFM_GPIO_PFR0_P3=0u; }while(0)

#define FGPIO1PIN_NP03_INITOUT(v)     do{ bFM_GPIO_PDOR0_P3=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR0_P3=1u; \
                                          bFM_GPIO_PFR0_P3=0u; }while(0)


/*---- Fast GPIO bit P04 ----*/

#define FGPIO1PIN_P04_GET             ( FM_FASTIO->FPDIR0_f.P4 )

#define FGPIO1PIN_P04_PUT(v)          ( bFM_GPIO_FPDOR0_P4=(v) )

#define FGPIO1PIN_P04_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P04_INITIN(v) \
                                                         : FGPIO1PIN_P04_INITOUT(v) )

#define FGPIO1PIN_P04_INITIN(v)       do{ bFM_GPIO_PCR0_P4=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR0_P4=0u; \
                                          bFM_GPIO_PFR0_P4=0u; }while(0)

#define FGPIO1PIN_P04_INITOUT(v)      do{ bFM_GPIO_FPDOR0_P4=(v).bInitVal; \
                                          bFM_GPIO_DDR0_P4=1u; \
                                          bFM_GPIO_PFR0_P4=0u; }while(0)


/*---- Fast GPIO bit NP04 ----*/
#define FGPIO1PIN_NP04_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR0_f.P4)) )

#define FGPIO1PIN_NP04_PUT(v)         ( bFM_GPIO_FPDOR0_P4=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP04_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP04_INITIN(v) \
                                                         : FGPIO1PIN_NP04_INITOUT(v) )

#define FGPIO1PIN_NP04_INITIN(v)      do{ bFM_GPIO_PCR0_P4=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR0_P4=0u; \
                                          bFM_GPIO_PFR0_P4=0u; }while(0)

#define FGPIO1PIN_NP04_INITOUT(v)     do{ bFM_GPIO_PDOR0_P4=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR0_P4=1u; \
                                          bFM_GPIO_PFR0_P4=0u; }while(0)


/*---- Fast GPIO bit P07 ----*/

#define FGPIO1PIN_P07_GET             ( FM_FASTIO->FPDIR0_f.P7 )

#define FGPIO1PIN_P07_PUT(v)          ( bFM_GPIO_FPDOR0_P7=(v) )

#define FGPIO1PIN_P07_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P07_INITIN(v) \
                                                         : FGPIO1PIN_P07_INITOUT(v) )

#define FGPIO1PIN_P07_INITIN(v)       do{ bFM_GPIO_PCR0_P7=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR0_P7=0u; \
                                          bFM_GPIO_PFR0_P7=0u; \
                                          bFM_GPIO_ADE_AN22=0u; }while(0)


#define FGPIO1PIN_P07_INITOUT(v)      do{ bFM_GPIO_FPDOR0_P7=(v).bInitVal; \
                                          bFM_GPIO_DDR0_P7=1u; \
                                          bFM_GPIO_PFR0_P7=0u; \
                                          bFM_GPIO_ADE_AN22=0u; }while(0)


/*---- Fast GPIO bit NP07 ----*/
#define FGPIO1PIN_NP07_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR0_f.P7)) )

#define FGPIO1PIN_NP07_PUT(v)         ( bFM_GPIO_FPDOR0_P7=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP07_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP07_INITIN(v) \
                                                         : FGPIO1PIN_NP07_INITOUT(v) )

#define FGPIO1PIN_NP07_INITIN(v)      do{ bFM_GPIO_PCR0_P7=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR0_P7=0u; \
                                          bFM_GPIO_PFR0_P7=0u; \
                                          bFM_GPIO_ADE_AN22=0u; }while(0)


#define FGPIO1PIN_NP07_INITOUT(v)     do{ bFM_GPIO_PDOR0_P7=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR0_P7=1u; \
                                          bFM_GPIO_PFR0_P7=0u; \
                                          bFM_GPIO_ADE_AN22=0u; }while(0)


/*---- Fast GPIO bit P0A ----*/

#define FGPIO1PIN_P0A_GET             ( FM_FASTIO->FPDIR0_f.PA )

#define FGPIO1PIN_P0A_PUT(v)          ( bFM_GPIO_FPDOR0_PA=(v) )

#define FGPIO1PIN_P0A_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P0A_INITIN(v) \
                                                         : FGPIO1PIN_P0A_INITOUT(v) )

#define FGPIO1PIN_P0A_INITIN(v)       do{ bFM_GPIO_PCR0_PA=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR0_PA=0u; \
                                          bFM_GPIO_PFR0_PA=0u; }while(0)

#define FGPIO1PIN_P0A_INITOUT(v)      do{ bFM_GPIO_FPDOR0_PA=(v).bInitVal; \
                                          bFM_GPIO_DDR0_PA=1u; \
                                          bFM_GPIO_PFR0_PA=0u; }while(0)


/*---- Fast GPIO bit NP0A ----*/
#define FGPIO1PIN_NP0A_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR0_f.PA)) )

#define FGPIO1PIN_NP0A_PUT(v)         ( bFM_GPIO_FPDOR0_PA=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP0A_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP0A_INITIN(v) \
                                                         : FGPIO1PIN_NP0A_INITOUT(v) )

#define FGPIO1PIN_NP0A_INITIN(v)      do{ bFM_GPIO_PCR0_PA=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR0_PA=0u; \
                                          bFM_GPIO_PFR0_PA=0u; }while(0)

#define FGPIO1PIN_NP0A_INITOUT(v)     do{ bFM_GPIO_PDOR0_PA=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR0_PA=1u; \
                                          bFM_GPIO_PFR0_PA=0u; }while(0)


/*---- Fast GPIO bit P0B ----*/

#define FGPIO1PIN_P0B_GET             ( FM_FASTIO->FPDIR0_f.PB )

#define FGPIO1PIN_P0B_PUT(v)          ( bFM_GPIO_FPDOR0_PB=(v) )

#define FGPIO1PIN_P0B_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P0B_INITIN(v) \
                                                         : FGPIO1PIN_P0B_INITOUT(v) )

#define FGPIO1PIN_P0B_INITIN(v)       do{ bFM_GPIO_PCR0_PB=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR0_PB=0u; \
                                          bFM_GPIO_PFR0_PB=0u; }while(0)

#define FGPIO1PIN_P0B_INITOUT(v)      do{ bFM_GPIO_FPDOR0_PB=(v).bInitVal; \
                                          bFM_GPIO_DDR0_PB=1u; \
                                          bFM_GPIO_PFR0_PB=0u; }while(0)


/*---- Fast GPIO bit NP0B ----*/
#define FGPIO1PIN_NP0B_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR0_f.PB)) )

#define FGPIO1PIN_NP0B_PUT(v)         ( bFM_GPIO_FPDOR0_PB=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP0B_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP0B_INITIN(v) \
                                                         : FGPIO1PIN_NP0B_INITOUT(v) )

#define FGPIO1PIN_NP0B_INITIN(v)      do{ bFM_GPIO_PCR0_PB=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR0_PB=0u; \
                                          bFM_GPIO_PFR0_PB=0u; }while(0)

#define FGPIO1PIN_NP0B_INITOUT(v)     do{ bFM_GPIO_PDOR0_PB=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR0_PB=1u; \
                                          bFM_GPIO_PFR0_PB=0u; }while(0)


/*---- Fast GPIO bit P0C ----*/

#define FGPIO1PIN_P0C_GET             ( FM_FASTIO->FPDIR0_f.PC )

#define FGPIO1PIN_P0C_PUT(v)          ( bFM_GPIO_FPDOR0_PC=(v) )

#define FGPIO1PIN_P0C_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P0C_INITIN(v) \
                                                         : FGPIO1PIN_P0C_INITOUT(v) )

#define FGPIO1PIN_P0C_INITIN(v)       do{ bFM_GPIO_PCR0_PC=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR0_PC=0u; \
                                          bFM_GPIO_PFR0_PC=0u; }while(0)

#define FGPIO1PIN_P0C_INITOUT(v)      do{ bFM_GPIO_FPDOR0_PC=(v).bInitVal; \
                                          bFM_GPIO_DDR0_PC=1u; \
                                          bFM_GPIO_PFR0_PC=0u; }while(0)


/*---- Fast GPIO bit NP0C ----*/
#define FGPIO1PIN_NP0C_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR0_f.PC)) )

#define FGPIO1PIN_NP0C_PUT(v)         ( bFM_GPIO_FPDOR0_PC=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP0C_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP0C_INITIN(v) \
                                                         : FGPIO1PIN_NP0C_INITOUT(v) )

#define FGPIO1PIN_NP0C_INITIN(v)      do{ bFM_GPIO_PCR0_PC=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR0_PC=0u; \
                                          bFM_GPIO_PFR0_PC=0u; }while(0)

#define FGPIO1PIN_NP0C_INITOUT(v)     do{ bFM_GPIO_PDOR0_PC=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR0_PC=1u; \
                                          bFM_GPIO_PFR0_PC=0u; }while(0)


/*---- Fast GPIO bit P0D ----*/

#define FGPIO1PIN_P0D_GET             ( FM_FASTIO->FPDIR0_f.PD )

#define FGPIO1PIN_P0D_PUT(v)          ( bFM_GPIO_FPDOR0_PD=(v) )

#define FGPIO1PIN_P0D_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P0D_INITIN(v) \
                                                         : FGPIO1PIN_P0D_INITOUT(v) )

#define FGPIO1PIN_P0D_INITIN(v)       do{ bFM_GPIO_PCR0_PD=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR0_PD=0u; \
                                          bFM_GPIO_PFR0_PD=0u; }while(0)

#define FGPIO1PIN_P0D_INITOUT(v)      do{ bFM_GPIO_FPDOR0_PD=(v).bInitVal; \
                                          bFM_GPIO_DDR0_PD=1u; \
                                          bFM_GPIO_PFR0_PD=0u; }while(0)


/*---- Fast GPIO bit NP0D ----*/
#define FGPIO1PIN_NP0D_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR0_f.PD)) )

#define FGPIO1PIN_NP0D_PUT(v)         ( bFM_GPIO_FPDOR0_PD=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP0D_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP0D_INITIN(v) \
                                                         : FGPIO1PIN_NP0D_INITOUT(v) )

#define FGPIO1PIN_NP0D_INITIN(v)      do{ bFM_GPIO_PCR0_PD=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR0_PD=0u; \
                                          bFM_GPIO_PFR0_PD=0u; }while(0)

#define FGPIO1PIN_NP0D_INITOUT(v)     do{ bFM_GPIO_PDOR0_PD=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR0_PD=1u; \
                                          bFM_GPIO_PFR0_PD=0u; }while(0)


/*---- Fast GPIO bit P0E ----*/

#define FGPIO1PIN_P0E_GET             ( FM_FASTIO->FPDIR0_f.PE )

#define FGPIO1PIN_P0E_PUT(v)          ( bFM_GPIO_FPDOR0_PE=(v) )

#define FGPIO1PIN_P0E_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P0E_INITIN(v) \
                                                         : FGPIO1PIN_P0E_INITOUT(v) )

#define FGPIO1PIN_P0E_INITIN(v)       do{ bFM_GPIO_PCR0_PE=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR0_PE=0u; \
                                          bFM_GPIO_PFR0_PE=0u; }while(0)

#define FGPIO1PIN_P0E_INITOUT(v)      do{ bFM_GPIO_FPDOR0_PE=(v).bInitVal; \
                                          bFM_GPIO_DDR0_PE=1u; \
                                          bFM_GPIO_PFR0_PE=0u; }while(0)


/*---- Fast GPIO bit NP0E ----*/
#define FGPIO1PIN_NP0E_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR0_f.PE)) )

#define FGPIO1PIN_NP0E_PUT(v)         ( bFM_GPIO_FPDOR0_PE=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP0E_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP0E_INITIN(v) \
                                                         : FGPIO1PIN_NP0E_INITOUT(v) )

#define FGPIO1PIN_NP0E_INITIN(v)      do{ bFM_GPIO_PCR0_PE=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR0_PE=0u; \
                                          bFM_GPIO_PFR0_PE=0u; }while(0)

#define FGPIO1PIN_NP0E_INITOUT(v)     do{ bFM_GPIO_PDOR0_PE=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR0_PE=1u; \
                                          bFM_GPIO_PFR0_PE=0u; }while(0)


/*---- Fast GPIO bit P0F ----*/

#define FGPIO1PIN_P0F_GET             ( FM_FASTIO->FPDIR0_f.PF )

#define FGPIO1PIN_P0F_PUT(v)          ( bFM_GPIO_FPDOR0_PF=(v) )

#define FGPIO1PIN_P0F_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P0F_INITIN(v) \
                                                         : FGPIO1PIN_P0F_INITOUT(v) )

#define FGPIO1PIN_P0F_INITIN(v)       do{ bFM_GPIO_PCR0_PF=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR0_PF=0u; \
                                          bFM_GPIO_PFR0_PF=0u; }while(0)

#define FGPIO1PIN_P0F_INITOUT(v)      do{ bFM_GPIO_FPDOR0_PF=(v).bInitVal; \
                                          bFM_GPIO_DDR0_PF=1u; \
                                          bFM_GPIO_PFR0_PF=0u; }while(0)


/*---- Fast GPIO bit NP0F ----*/
#define FGPIO1PIN_NP0F_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR0_f.PF)) )

#define FGPIO1PIN_NP0F_PUT(v)         ( bFM_GPIO_FPDOR0_PF=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP0F_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP0F_INITIN(v) \
                                                         : FGPIO1PIN_NP0F_INITOUT(v) )

#define FGPIO1PIN_NP0F_INITIN(v)      do{ bFM_GPIO_PCR0_PF=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR0_PF=0u; \
                                          bFM_GPIO_PFR0_PF=0u; }while(0)

#define FGPIO1PIN_NP0F_INITOUT(v)     do{ bFM_GPIO_PDOR0_PF=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR0_PF=1u; \
                                          bFM_GPIO_PFR0_PF=0u; }while(0)


/*---- Fast GPIO bit P10 ----*/

#define FGPIO1PIN_P10_GET             ( FM_FASTIO->FPDIR1_f.P0 )

#define FGPIO1PIN_P10_PUT(v)          ( bFM_GPIO_FPDOR1_P0=(v) )

#define FGPIO1PIN_P10_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P10_INITIN(v) \
                                                         : FGPIO1PIN_P10_INITOUT(v) )

#define FGPIO1PIN_P10_INITIN(v)       do{ bFM_GPIO_PCR1_P0=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR1_P0=0u; \
                                          bFM_GPIO_PFR1_P0=0u; \
                                          bFM_GPIO_ADE_AN00=0u; }while(0)


#define FGPIO1PIN_P10_INITOUT(v)      do{ bFM_GPIO_FPDOR1_P0=(v).bInitVal; \
                                          bFM_GPIO_DDR1_P0=1u; \
                                          bFM_GPIO_PFR1_P0=0u; \
                                          bFM_GPIO_ADE_AN00=0u; }while(0)


/*---- Fast GPIO bit NP10 ----*/
#define FGPIO1PIN_NP10_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR1_f.P0)) )

#define FGPIO1PIN_NP10_PUT(v)         ( bFM_GPIO_FPDOR1_P0=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP10_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP10_INITIN(v) \
                                                         : FGPIO1PIN_NP10_INITOUT(v) )

#define FGPIO1PIN_NP10_INITIN(v)      do{ bFM_GPIO_PCR1_P0=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR1_P0=0u; \
                                          bFM_GPIO_PFR1_P0=0u; \
                                          bFM_GPIO_ADE_AN00=0u; }while(0)


#define FGPIO1PIN_NP10_INITOUT(v)     do{ bFM_GPIO_PDOR1_P0=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR1_P0=1u; \
                                          bFM_GPIO_PFR1_P0=0u; \
                                          bFM_GPIO_ADE_AN00=0u; }while(0)


/*---- Fast GPIO bit P11 ----*/

#define FGPIO1PIN_P11_GET             ( FM_FASTIO->FPDIR1_f.P1 )

#define FGPIO1PIN_P11_PUT(v)          ( bFM_GPIO_FPDOR1_P1=(v) )

#define FGPIO1PIN_P11_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P11_INITIN(v) \
                                                         : FGPIO1PIN_P11_INITOUT(v) )

#define FGPIO1PIN_P11_INITIN(v)       do{ bFM_GPIO_PCR1_P1=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR1_P1=0u; \
                                          bFM_GPIO_PFR1_P1=0u; \
                                          bFM_GPIO_ADE_AN01=0u; }while(0)


#define FGPIO1PIN_P11_INITOUT(v)      do{ bFM_GPIO_FPDOR1_P1=(v).bInitVal; \
                                          bFM_GPIO_DDR1_P1=1u; \
                                          bFM_GPIO_PFR1_P1=0u; \
                                          bFM_GPIO_ADE_AN01=0u; }while(0)


/*---- Fast GPIO bit NP11 ----*/
#define FGPIO1PIN_NP11_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR1_f.P1)) )

#define FGPIO1PIN_NP11_PUT(v)         ( bFM_GPIO_FPDOR1_P1=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP11_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP11_INITIN(v) \
                                                         : FGPIO1PIN_NP11_INITOUT(v) )

#define FGPIO1PIN_NP11_INITIN(v)      do{ bFM_GPIO_PCR1_P1=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR1_P1=0u; \
                                          bFM_GPIO_PFR1_P1=0u; \
                                          bFM_GPIO_ADE_AN01=0u; }while(0)


#define FGPIO1PIN_NP11_INITOUT(v)     do{ bFM_GPIO_PDOR1_P1=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR1_P1=1u; \
                                          bFM_GPIO_PFR1_P1=0u; \
                                          bFM_GPIO_ADE_AN01=0u; }while(0)


/*---- Fast GPIO bit P12 ----*/

#define FGPIO1PIN_P12_GET             ( FM_FASTIO->FPDIR1_f.P2 )

#define FGPIO1PIN_P12_PUT(v)          ( bFM_GPIO_FPDOR1_P2=(v) )

#define FGPIO1PIN_P12_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P12_INITIN(v) \
                                                         : FGPIO1PIN_P12_INITOUT(v) )

#define FGPIO1PIN_P12_INITIN(v)       do{ bFM_GPIO_PCR1_P2=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR1_P2=0u; \
                                          bFM_GPIO_PFR1_P2=0u; \
                                          bFM_GPIO_ADE_AN02=0u; }while(0)


#define FGPIO1PIN_P12_INITOUT(v)      do{ bFM_GPIO_FPDOR1_P2=(v).bInitVal; \
                                          bFM_GPIO_DDR1_P2=1u; \
                                          bFM_GPIO_PFR1_P2=0u; \
                                          bFM_GPIO_ADE_AN02=0u; }while(0)


/*---- Fast GPIO bit NP12 ----*/
#define FGPIO1PIN_NP12_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR1_f.P2)) )

#define FGPIO1PIN_NP12_PUT(v)         ( bFM_GPIO_FPDOR1_P2=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP12_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP12_INITIN(v) \
                                                         : FGPIO1PIN_NP12_INITOUT(v) )

#define FGPIO1PIN_NP12_INITIN(v)      do{ bFM_GPIO_PCR1_P2=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR1_P2=0u; \
                                          bFM_GPIO_PFR1_P2=0u; \
                                          bFM_GPIO_ADE_AN02=0u; }while(0)


#define FGPIO1PIN_NP12_INITOUT(v)     do{ bFM_GPIO_PDOR1_P2=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR1_P2=1u; \
                                          bFM_GPIO_PFR1_P2=0u; \
                                          bFM_GPIO_ADE_AN02=0u; }while(0)


/*---- Fast GPIO bit P13 ----*/

#define FGPIO1PIN_P13_GET             ( FM_FASTIO->FPDIR1_f.P3 )

#define FGPIO1PIN_P13_PUT(v)          ( bFM_GPIO_FPDOR1_P3=(v) )

#define FGPIO1PIN_P13_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P13_INITIN(v) \
                                                         : FGPIO1PIN_P13_INITOUT(v) )

#define FGPIO1PIN_P13_INITIN(v)       do{ bFM_GPIO_PCR1_P3=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR1_P3=0u; \
                                          bFM_GPIO_PFR1_P3=0u; \
                                          bFM_GPIO_ADE_AN03=0u; }while(0)


#define FGPIO1PIN_P13_INITOUT(v)      do{ bFM_GPIO_FPDOR1_P3=(v).bInitVal; \
                                          bFM_GPIO_DDR1_P3=1u; \
                                          bFM_GPIO_PFR1_P3=0u; \
                                          bFM_GPIO_ADE_AN03=0u; }while(0)


/*---- Fast GPIO bit NP13 ----*/
#define FGPIO1PIN_NP13_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR1_f.P3)) )

#define FGPIO1PIN_NP13_PUT(v)         ( bFM_GPIO_FPDOR1_P3=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP13_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP13_INITIN(v) \
                                                         : FGPIO1PIN_NP13_INITOUT(v) )

#define FGPIO1PIN_NP13_INITIN(v)      do{ bFM_GPIO_PCR1_P3=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR1_P3=0u; \
                                          bFM_GPIO_PFR1_P3=0u; \
                                          bFM_GPIO_ADE_AN03=0u; }while(0)


#define FGPIO1PIN_NP13_INITOUT(v)     do{ bFM_GPIO_PDOR1_P3=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR1_P3=1u; \
                                          bFM_GPIO_PFR1_P3=0u; \
                                          bFM_GPIO_ADE_AN03=0u; }while(0)


/*---- Fast GPIO bit P14 ----*/

#define FGPIO1PIN_P14_GET             ( FM_FASTIO->FPDIR1_f.P4 )

#define FGPIO1PIN_P14_PUT(v)          ( bFM_GPIO_FPDOR1_P4=(v) )

#define FGPIO1PIN_P14_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P14_INITIN(v) \
                                                         : FGPIO1PIN_P14_INITOUT(v) )

#define FGPIO1PIN_P14_INITIN(v)       do{ bFM_GPIO_PCR1_P4=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR1_P4=0u; \
                                          bFM_GPIO_PFR1_P4=0u; \
                                          bFM_GPIO_ADE_AN04=0u; }while(0)


#define FGPIO1PIN_P14_INITOUT(v)      do{ bFM_GPIO_FPDOR1_P4=(v).bInitVal; \
                                          bFM_GPIO_DDR1_P4=1u; \
                                          bFM_GPIO_PFR1_P4=0u; \
                                          bFM_GPIO_ADE_AN04=0u; }while(0)


/*---- Fast GPIO bit NP14 ----*/
#define FGPIO1PIN_NP14_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR1_f.P4)) )

#define FGPIO1PIN_NP14_PUT(v)         ( bFM_GPIO_FPDOR1_P4=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP14_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP14_INITIN(v) \
                                                         : FGPIO1PIN_NP14_INITOUT(v) )

#define FGPIO1PIN_NP14_INITIN(v)      do{ bFM_GPIO_PCR1_P4=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR1_P4=0u; \
                                          bFM_GPIO_PFR1_P4=0u; \
                                          bFM_GPIO_ADE_AN04=0u; }while(0)


#define FGPIO1PIN_NP14_INITOUT(v)     do{ bFM_GPIO_PDOR1_P4=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR1_P4=1u; \
                                          bFM_GPIO_PFR1_P4=0u; \
                                          bFM_GPIO_ADE_AN04=0u; }while(0)


/*---- Fast GPIO bit P15 ----*/

#define FGPIO1PIN_P15_GET             ( FM_FASTIO->FPDIR1_f.P5 )

#define FGPIO1PIN_P15_PUT(v)          ( bFM_GPIO_FPDOR1_P5=(v) )

#define FGPIO1PIN_P15_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P15_INITIN(v) \
                                                         : FGPIO1PIN_P15_INITOUT(v) )

#define FGPIO1PIN_P15_INITIN(v)       do{ bFM_GPIO_PCR1_P5=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR1_P5=0u; \
                                          bFM_GPIO_PFR1_P5=0u; \
                                          bFM_GPIO_ADE_AN05=0u; }while(0)


#define FGPIO1PIN_P15_INITOUT(v)      do{ bFM_GPIO_FPDOR1_P5=(v).bInitVal; \
                                          bFM_GPIO_DDR1_P5=1u; \
                                          bFM_GPIO_PFR1_P5=0u; \
                                          bFM_GPIO_ADE_AN05=0u; }while(0)


/*---- Fast GPIO bit NP15 ----*/
#define FGPIO1PIN_NP15_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR1_f.P5)) )

#define FGPIO1PIN_NP15_PUT(v)         ( bFM_GPIO_FPDOR1_P5=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP15_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP15_INITIN(v) \
                                                         : FGPIO1PIN_NP15_INITOUT(v) )

#define FGPIO1PIN_NP15_INITIN(v)      do{ bFM_GPIO_PCR1_P5=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR1_P5=0u; \
                                          bFM_GPIO_PFR1_P5=0u; \
                                          bFM_GPIO_ADE_AN05=0u; }while(0)


#define FGPIO1PIN_NP15_INITOUT(v)     do{ bFM_GPIO_PDOR1_P5=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR1_P5=1u; \
                                          bFM_GPIO_PFR1_P5=0u; \
                                          bFM_GPIO_ADE_AN05=0u; }while(0)


/*---- Fast GPIO bit P16 ----*/

#define FGPIO1PIN_P16_GET             ( FM_FASTIO->FPDIR1_f.P6 )

#define FGPIO1PIN_P16_PUT(v)          ( bFM_GPIO_FPDOR1_P6=(v) )

#define FGPIO1PIN_P16_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P16_INITIN(v) \
                                                         : FGPIO1PIN_P16_INITOUT(v) )

#define FGPIO1PIN_P16_INITIN(v)       do{ bFM_GPIO_PCR1_P6=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR1_P6=0u; \
                                          bFM_GPIO_PFR1_P6=0u; \
                                          bFM_GPIO_ADE_AN06=0u; }while(0)


#define FGPIO1PIN_P16_INITOUT(v)      do{ bFM_GPIO_FPDOR1_P6=(v).bInitVal; \
                                          bFM_GPIO_DDR1_P6=1u; \
                                          bFM_GPIO_PFR1_P6=0u; \
                                          bFM_GPIO_ADE_AN06=0u; }while(0)


/*---- Fast GPIO bit NP16 ----*/
#define FGPIO1PIN_NP16_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR1_f.P6)) )

#define FGPIO1PIN_NP16_PUT(v)         ( bFM_GPIO_FPDOR1_P6=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP16_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP16_INITIN(v) \
                                                         : FGPIO1PIN_NP16_INITOUT(v) )

#define FGPIO1PIN_NP16_INITIN(v)      do{ bFM_GPIO_PCR1_P6=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR1_P6=0u; \
                                          bFM_GPIO_PFR1_P6=0u; \
                                          bFM_GPIO_ADE_AN06=0u; }while(0)


#define FGPIO1PIN_NP16_INITOUT(v)     do{ bFM_GPIO_PDOR1_P6=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR1_P6=1u; \
                                          bFM_GPIO_PFR1_P6=0u; \
                                          bFM_GPIO_ADE_AN06=0u; }while(0)


/*---- Fast GPIO bit P17 ----*/

#define FGPIO1PIN_P17_GET             ( FM_FASTIO->FPDIR1_f.P7 )

#define FGPIO1PIN_P17_PUT(v)          ( bFM_GPIO_FPDOR1_P7=(v) )

#define FGPIO1PIN_P17_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P17_INITIN(v) \
                                                         : FGPIO1PIN_P17_INITOUT(v) )

#define FGPIO1PIN_P17_INITIN(v)       do{ bFM_GPIO_PCR1_P7=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR1_P7=0u; \
                                          bFM_GPIO_PFR1_P7=0u; \
                                          bFM_GPIO_ADE_AN07=0u; }while(0)


#define FGPIO1PIN_P17_INITOUT(v)      do{ bFM_GPIO_FPDOR1_P7=(v).bInitVal; \
                                          bFM_GPIO_DDR1_P7=1u; \
                                          bFM_GPIO_PFR1_P7=0u; \
                                          bFM_GPIO_ADE_AN07=0u; }while(0)


/*---- Fast GPIO bit NP17 ----*/
#define FGPIO1PIN_NP17_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR1_f.P7)) )

#define FGPIO1PIN_NP17_PUT(v)         ( bFM_GPIO_FPDOR1_P7=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP17_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP17_INITIN(v) \
                                                         : FGPIO1PIN_NP17_INITOUT(v) )

#define FGPIO1PIN_NP17_INITIN(v)      do{ bFM_GPIO_PCR1_P7=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR1_P7=0u; \
                                          bFM_GPIO_PFR1_P7=0u; \
                                          bFM_GPIO_ADE_AN07=0u; }while(0)


#define FGPIO1PIN_NP17_INITOUT(v)     do{ bFM_GPIO_PDOR1_P7=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR1_P7=1u; \
                                          bFM_GPIO_PFR1_P7=0u; \
                                          bFM_GPIO_ADE_AN07=0u; }while(0)


/*---- Fast GPIO bit P18 ----*/

#define FGPIO1PIN_P18_GET             ( FM_FASTIO->FPDIR1_f.P8 )

#define FGPIO1PIN_P18_PUT(v)          ( bFM_GPIO_FPDOR1_P8=(v) )

#define FGPIO1PIN_P18_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P18_INITIN(v) \
                                                         : FGPIO1PIN_P18_INITOUT(v) )

#define FGPIO1PIN_P18_INITIN(v)       do{ bFM_GPIO_PCR1_P8=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR1_P8=0u; \
                                          bFM_GPIO_PFR1_P8=0u; \
                                          bFM_GPIO_ADE_AN08=0u; }while(0)


#define FGPIO1PIN_P18_INITOUT(v)      do{ bFM_GPIO_FPDOR1_P8=(v).bInitVal; \
                                          bFM_GPIO_DDR1_P8=1u; \
                                          bFM_GPIO_PFR1_P8=0u; \
                                          bFM_GPIO_ADE_AN08=0u; }while(0)


/*---- Fast GPIO bit NP18 ----*/
#define FGPIO1PIN_NP18_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR1_f.P8)) )

#define FGPIO1PIN_NP18_PUT(v)         ( bFM_GPIO_FPDOR1_P8=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP18_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP18_INITIN(v) \
                                                         : FGPIO1PIN_NP18_INITOUT(v) )

#define FGPIO1PIN_NP18_INITIN(v)      do{ bFM_GPIO_PCR1_P8=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR1_P8=0u; \
                                          bFM_GPIO_PFR1_P8=0u; \
                                          bFM_GPIO_ADE_AN08=0u; }while(0)


#define FGPIO1PIN_NP18_INITOUT(v)     do{ bFM_GPIO_PDOR1_P8=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR1_P8=1u; \
                                          bFM_GPIO_PFR1_P8=0u; \
                                          bFM_GPIO_ADE_AN08=0u; }while(0)


/*---- Fast GPIO bit P19 ----*/

#define FGPIO1PIN_P19_GET             ( FM_FASTIO->FPDIR1_f.P9 )

#define FGPIO1PIN_P19_PUT(v)          ( bFM_GPIO_FPDOR1_P9=(v) )

#define FGPIO1PIN_P19_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P19_INITIN(v) \
                                                         : FGPIO1PIN_P19_INITOUT(v) )

#define FGPIO1PIN_P19_INITIN(v)       do{ bFM_GPIO_PCR1_P9=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR1_P9=0u; \
                                          bFM_GPIO_PFR1_P9=0u; \
                                          bFM_GPIO_ADE_AN09=0u; }while(0)


#define FGPIO1PIN_P19_INITOUT(v)      do{ bFM_GPIO_FPDOR1_P9=(v).bInitVal; \
                                          bFM_GPIO_DDR1_P9=1u; \
                                          bFM_GPIO_PFR1_P9=0u; \
                                          bFM_GPIO_ADE_AN09=0u; }while(0)


/*---- Fast GPIO bit NP19 ----*/
#define FGPIO1PIN_NP19_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR1_f.P9)) )

#define FGPIO1PIN_NP19_PUT(v)         ( bFM_GPIO_FPDOR1_P9=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP19_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP19_INITIN(v) \
                                                         : FGPIO1PIN_NP19_INITOUT(v) )

#define FGPIO1PIN_NP19_INITIN(v)      do{ bFM_GPIO_PCR1_P9=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR1_P9=0u; \
                                          bFM_GPIO_PFR1_P9=0u; \
                                          bFM_GPIO_ADE_AN09=0u; }while(0)


#define FGPIO1PIN_NP19_INITOUT(v)     do{ bFM_GPIO_PDOR1_P9=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR1_P9=1u; \
                                          bFM_GPIO_PFR1_P9=0u; \
                                          bFM_GPIO_ADE_AN09=0u; }while(0)


/*---- Fast GPIO bit P1A ----*/

#define FGPIO1PIN_P1A_GET             ( FM_FASTIO->FPDIR1_f.PA )

#define FGPIO1PIN_P1A_PUT(v)          ( bFM_GPIO_FPDOR1_PA=(v) )

#define FGPIO1PIN_P1A_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P1A_INITIN(v) \
                                                         : FGPIO1PIN_P1A_INITOUT(v) )

#define FGPIO1PIN_P1A_INITIN(v)       do{ bFM_GPIO_PCR1_PA=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR1_PA=0u; \
                                          bFM_GPIO_PFR1_PA=0u; \
                                          bFM_GPIO_ADE_AN10=0u; }while(0)


#define FGPIO1PIN_P1A_INITOUT(v)      do{ bFM_GPIO_FPDOR1_PA=(v).bInitVal; \
                                          bFM_GPIO_DDR1_PA=1u; \
                                          bFM_GPIO_PFR1_PA=0u; \
                                          bFM_GPIO_ADE_AN10=0u; }while(0)


/*---- Fast GPIO bit NP1A ----*/
#define FGPIO1PIN_NP1A_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR1_f.PA)) )

#define FGPIO1PIN_NP1A_PUT(v)         ( bFM_GPIO_FPDOR1_PA=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP1A_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP1A_INITIN(v) \
                                                         : FGPIO1PIN_NP1A_INITOUT(v) )

#define FGPIO1PIN_NP1A_INITIN(v)      do{ bFM_GPIO_PCR1_PA=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR1_PA=0u; \
                                          bFM_GPIO_PFR1_PA=0u; \
                                          bFM_GPIO_ADE_AN10=0u; }while(0)


#define FGPIO1PIN_NP1A_INITOUT(v)     do{ bFM_GPIO_PDOR1_PA=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR1_PA=1u; \
                                          bFM_GPIO_PFR1_PA=0u; \
                                          bFM_GPIO_ADE_AN10=0u; }while(0)


/*---- Fast GPIO bit P1B ----*/

#define FGPIO1PIN_P1B_GET             ( FM_FASTIO->FPDIR1_f.PB )

#define FGPIO1PIN_P1B_PUT(v)          ( bFM_GPIO_FPDOR1_PB=(v) )

#define FGPIO1PIN_P1B_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P1B_INITIN(v) \
                                                         : FGPIO1PIN_P1B_INITOUT(v) )

#define FGPIO1PIN_P1B_INITIN(v)       do{ bFM_GPIO_PCR1_PB=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR1_PB=0u; \
                                          bFM_GPIO_PFR1_PB=0u; \
                                          bFM_GPIO_ADE_AN11=0u; }while(0)


#define FGPIO1PIN_P1B_INITOUT(v)      do{ bFM_GPIO_FPDOR1_PB=(v).bInitVal; \
                                          bFM_GPIO_DDR1_PB=1u; \
                                          bFM_GPIO_PFR1_PB=0u; \
                                          bFM_GPIO_ADE_AN11=0u; }while(0)


/*---- Fast GPIO bit NP1B ----*/
#define FGPIO1PIN_NP1B_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR1_f.PB)) )

#define FGPIO1PIN_NP1B_PUT(v)         ( bFM_GPIO_FPDOR1_PB=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP1B_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP1B_INITIN(v) \
                                                         : FGPIO1PIN_NP1B_INITOUT(v) )

#define FGPIO1PIN_NP1B_INITIN(v)      do{ bFM_GPIO_PCR1_PB=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR1_PB=0u; \
                                          bFM_GPIO_PFR1_PB=0u; \
                                          bFM_GPIO_ADE_AN11=0u; }while(0)


#define FGPIO1PIN_NP1B_INITOUT(v)     do{ bFM_GPIO_PDOR1_PB=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR1_PB=1u; \
                                          bFM_GPIO_PFR1_PB=0u; \
                                          bFM_GPIO_ADE_AN11=0u; }while(0)


/*---- Fast GPIO bit P21 ----*/

#define FGPIO1PIN_P21_GET             ( FM_FASTIO->FPDIR2_f.P1 )

#define FGPIO1PIN_P21_PUT(v)          ( bFM_GPIO_FPDOR2_P1=(v) )

#define FGPIO1PIN_P21_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P21_INITIN(v) \
                                                         : FGPIO1PIN_P21_INITOUT(v) )

#define FGPIO1PIN_P21_INITIN(v)       do{ bFM_GPIO_PCR2_P1=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR2_P1=0u; \
                                          bFM_GPIO_PFR2_P1=0u; \
                                          bFM_GPIO_ADE_AN18=0u; }while(0)


#define FGPIO1PIN_P21_INITOUT(v)      do{ bFM_GPIO_FPDOR2_P1=(v).bInitVal; \
                                          bFM_GPIO_DDR2_P1=1u; \
                                          bFM_GPIO_PFR2_P1=0u; \
                                          bFM_GPIO_ADE_AN18=0u; }while(0)


/*---- Fast GPIO bit NP21 ----*/
#define FGPIO1PIN_NP21_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR2_f.P1)) )

#define FGPIO1PIN_NP21_PUT(v)         ( bFM_GPIO_FPDOR2_P1=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP21_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP21_INITIN(v) \
                                                         : FGPIO1PIN_NP21_INITOUT(v) )

#define FGPIO1PIN_NP21_INITIN(v)      do{ bFM_GPIO_PCR2_P1=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR2_P1=0u; \
                                          bFM_GPIO_PFR2_P1=0u; \
                                          bFM_GPIO_ADE_AN18=0u; }while(0)


#define FGPIO1PIN_NP21_INITOUT(v)     do{ bFM_GPIO_PDOR2_P1=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR2_P1=1u; \
                                          bFM_GPIO_PFR2_P1=0u; \
                                          bFM_GPIO_ADE_AN18=0u; }while(0)


/*---- Fast GPIO bit P22 ----*/

#define FGPIO1PIN_P22_GET             ( FM_FASTIO->FPDIR2_f.P2 )

#define FGPIO1PIN_P22_PUT(v)          ( bFM_GPIO_FPDOR2_P2=(v) )

#define FGPIO1PIN_P22_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P22_INITIN(v) \
                                                         : FGPIO1PIN_P22_INITOUT(v) )

#define FGPIO1PIN_P22_INITIN(v)       do{ bFM_GPIO_PCR2_P2=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR2_P2=0u; \
                                          bFM_GPIO_PFR2_P2=0u; \
                                          bFM_GPIO_ADE_AN17=0u; }while(0)


#define FGPIO1PIN_P22_INITOUT(v)      do{ bFM_GPIO_FPDOR2_P2=(v).bInitVal; \
                                          bFM_GPIO_DDR2_P2=1u; \
                                          bFM_GPIO_PFR2_P2=0u; \
                                          bFM_GPIO_ADE_AN17=0u; }while(0)


/*---- Fast GPIO bit NP22 ----*/
#define FGPIO1PIN_NP22_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR2_f.P2)) )

#define FGPIO1PIN_NP22_PUT(v)         ( bFM_GPIO_FPDOR2_P2=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP22_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP22_INITIN(v) \
                                                         : FGPIO1PIN_NP22_INITOUT(v) )

#define FGPIO1PIN_NP22_INITIN(v)      do{ bFM_GPIO_PCR2_P2=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR2_P2=0u; \
                                          bFM_GPIO_PFR2_P2=0u; \
                                          bFM_GPIO_ADE_AN17=0u; }while(0)


#define FGPIO1PIN_NP22_INITOUT(v)     do{ bFM_GPIO_PDOR2_P2=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR2_P2=1u; \
                                          bFM_GPIO_PFR2_P2=0u; \
                                          bFM_GPIO_ADE_AN17=0u; }while(0)


/*---- Fast GPIO bit P23 ----*/

#define FGPIO1PIN_P23_GET             ( FM_FASTIO->FPDIR2_f.P3 )

#define FGPIO1PIN_P23_PUT(v)          ( bFM_GPIO_FPDOR2_P3=(v) )

#define FGPIO1PIN_P23_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P23_INITIN(v) \
                                                         : FGPIO1PIN_P23_INITOUT(v) )

#define FGPIO1PIN_P23_INITIN(v)       do{ bFM_GPIO_PCR2_P3=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR2_P3=0u; \
                                          bFM_GPIO_PFR2_P3=0u; \
                                          bFM_GPIO_ADE_AN16=0u; }while(0)


#define FGPIO1PIN_P23_INITOUT(v)      do{ bFM_GPIO_FPDOR2_P3=(v).bInitVal; \
                                          bFM_GPIO_DDR2_P3=1u; \
                                          bFM_GPIO_PFR2_P3=0u; \
                                          bFM_GPIO_ADE_AN16=0u; }while(0)


/*---- Fast GPIO bit NP23 ----*/
#define FGPIO1PIN_NP23_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR2_f.P3)) )

#define FGPIO1PIN_NP23_PUT(v)         ( bFM_GPIO_FPDOR2_P3=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP23_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP23_INITIN(v) \
                                                         : FGPIO1PIN_NP23_INITOUT(v) )

#define FGPIO1PIN_NP23_INITIN(v)      do{ bFM_GPIO_PCR2_P3=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR2_P3=0u; \
                                          bFM_GPIO_PFR2_P3=0u; \
                                          bFM_GPIO_ADE_AN16=0u; }while(0)


#define FGPIO1PIN_NP23_INITOUT(v)     do{ bFM_GPIO_PDOR2_P3=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR2_P3=1u; \
                                          bFM_GPIO_PFR2_P3=0u; \
                                          bFM_GPIO_ADE_AN16=0u; }while(0)


/*---- Fast GPIO bit P30 ----*/

#define FGPIO1PIN_P30_GET             ( FM_FASTIO->FPDIR3_f.P0 )

#define FGPIO1PIN_P30_PUT(v)          ( bFM_GPIO_FPDOR3_P0=(v) )

#define FGPIO1PIN_P30_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P30_INITIN(v) \
                                                         : FGPIO1PIN_P30_INITOUT(v) )

#define FGPIO1PIN_P30_INITIN(v)       do{ bFM_GPIO_PCR3_P0=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR3_P0=0u; \
                                          bFM_GPIO_PFR3_P0=0u; }while(0)

#define FGPIO1PIN_P30_INITOUT(v)      do{ bFM_GPIO_FPDOR3_P0=(v).bInitVal; \
                                          bFM_GPIO_DDR3_P0=1u; \
                                          bFM_GPIO_PFR3_P0=0u; }while(0)


/*---- Fast GPIO bit NP30 ----*/
#define FGPIO1PIN_NP30_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR3_f.P0)) )

#define FGPIO1PIN_NP30_PUT(v)         ( bFM_GPIO_FPDOR3_P0=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP30_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP30_INITIN(v) \
                                                         : FGPIO1PIN_NP30_INITOUT(v) )

#define FGPIO1PIN_NP30_INITIN(v)      do{ bFM_GPIO_PCR3_P0=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR3_P0=0u; \
                                          bFM_GPIO_PFR3_P0=0u; }while(0)

#define FGPIO1PIN_NP30_INITOUT(v)     do{ bFM_GPIO_PDOR3_P0=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR3_P0=1u; \
                                          bFM_GPIO_PFR3_P0=0u; }while(0)


/*---- Fast GPIO bit P31 ----*/

#define FGPIO1PIN_P31_GET             ( FM_FASTIO->FPDIR3_f.P1 )

#define FGPIO1PIN_P31_PUT(v)          ( bFM_GPIO_FPDOR3_P1=(v) )

#define FGPIO1PIN_P31_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P31_INITIN(v) \
                                                         : FGPIO1PIN_P31_INITOUT(v) )

#define FGPIO1PIN_P31_INITIN(v)       do{ bFM_GPIO_PCR3_P1=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR3_P1=0u; \
                                          bFM_GPIO_PFR3_P1=0u; }while(0)

#define FGPIO1PIN_P31_INITOUT(v)      do{ bFM_GPIO_FPDOR3_P1=(v).bInitVal; \
                                          bFM_GPIO_DDR3_P1=1u; \
                                          bFM_GPIO_PFR3_P1=0u; }while(0)


/*---- Fast GPIO bit NP31 ----*/
#define FGPIO1PIN_NP31_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR3_f.P1)) )

#define FGPIO1PIN_NP31_PUT(v)         ( bFM_GPIO_FPDOR3_P1=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP31_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP31_INITIN(v) \
                                                         : FGPIO1PIN_NP31_INITOUT(v) )

#define FGPIO1PIN_NP31_INITIN(v)      do{ bFM_GPIO_PCR3_P1=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR3_P1=0u; \
                                          bFM_GPIO_PFR3_P1=0u; }while(0)

#define FGPIO1PIN_NP31_INITOUT(v)     do{ bFM_GPIO_PDOR3_P1=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR3_P1=1u; \
                                          bFM_GPIO_PFR3_P1=0u; }while(0)


/*---- Fast GPIO bit P32 ----*/

#define FGPIO1PIN_P32_GET             ( FM_FASTIO->FPDIR3_f.P2 )

#define FGPIO1PIN_P32_PUT(v)          ( bFM_GPIO_FPDOR3_P2=(v) )

#define FGPIO1PIN_P32_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P32_INITIN(v) \
                                                         : FGPIO1PIN_P32_INITOUT(v) )

#define FGPIO1PIN_P32_INITIN(v)       do{ bFM_GPIO_PCR3_P2=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR3_P2=0u; \
                                          bFM_GPIO_PFR3_P2=0u; }while(0)

#define FGPIO1PIN_P32_INITOUT(v)      do{ bFM_GPIO_FPDOR3_P2=(v).bInitVal; \
                                          bFM_GPIO_DDR3_P2=1u; \
                                          bFM_GPIO_PFR3_P2=0u; }while(0)


/*---- Fast GPIO bit NP32 ----*/
#define FGPIO1PIN_NP32_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR3_f.P2)) )

#define FGPIO1PIN_NP32_PUT(v)         ( bFM_GPIO_FPDOR3_P2=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP32_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP32_INITIN(v) \
                                                         : FGPIO1PIN_NP32_INITOUT(v) )

#define FGPIO1PIN_NP32_INITIN(v)      do{ bFM_GPIO_PCR3_P2=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR3_P2=0u; \
                                          bFM_GPIO_PFR3_P2=0u; }while(0)

#define FGPIO1PIN_NP32_INITOUT(v)     do{ bFM_GPIO_PDOR3_P2=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR3_P2=1u; \
                                          bFM_GPIO_PFR3_P2=0u; }while(0)


/*---- Fast GPIO bit P33 ----*/

#define FGPIO1PIN_P33_GET             ( FM_FASTIO->FPDIR3_f.P3 )

#define FGPIO1PIN_P33_PUT(v)          ( bFM_GPIO_FPDOR3_P3=(v) )

#define FGPIO1PIN_P33_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P33_INITIN(v) \
                                                         : FGPIO1PIN_P33_INITOUT(v) )

#define FGPIO1PIN_P33_INITIN(v)       do{ bFM_GPIO_PCR3_P3=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR3_P3=0u; \
                                          bFM_GPIO_PFR3_P3=0u; }while(0)

#define FGPIO1PIN_P33_INITOUT(v)      do{ bFM_GPIO_FPDOR3_P3=(v).bInitVal; \
                                          bFM_GPIO_DDR3_P3=1u; \
                                          bFM_GPIO_PFR3_P3=0u; }while(0)


/*---- Fast GPIO bit NP33 ----*/
#define FGPIO1PIN_NP33_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR3_f.P3)) )

#define FGPIO1PIN_NP33_PUT(v)         ( bFM_GPIO_FPDOR3_P3=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP33_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP33_INITIN(v) \
                                                         : FGPIO1PIN_NP33_INITOUT(v) )

#define FGPIO1PIN_NP33_INITIN(v)      do{ bFM_GPIO_PCR3_P3=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR3_P3=0u; \
                                          bFM_GPIO_PFR3_P3=0u; }while(0)

#define FGPIO1PIN_NP33_INITOUT(v)     do{ bFM_GPIO_PDOR3_P3=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR3_P3=1u; \
                                          bFM_GPIO_PFR3_P3=0u; }while(0)


/*---- Fast GPIO bit P39 ----*/

#define FGPIO1PIN_P39_GET             ( FM_FASTIO->FPDIR3_f.P9 )

#define FGPIO1PIN_P39_PUT(v)          ( bFM_GPIO_FPDOR3_P9=(v) )

#define FGPIO1PIN_P39_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P39_INITIN(v) \
                                                         : FGPIO1PIN_P39_INITOUT(v) )

#define FGPIO1PIN_P39_INITIN(v)       do{ bFM_GPIO_PCR3_P9=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR3_P9=0u; \
                                          bFM_GPIO_PFR3_P9=0u; }while(0)

#define FGPIO1PIN_P39_INITOUT(v)      do{ bFM_GPIO_FPDOR3_P9=(v).bInitVal; \
                                          bFM_GPIO_DDR3_P9=1u; \
                                          bFM_GPIO_PFR3_P9=0u; }while(0)


/*---- Fast GPIO bit NP39 ----*/
#define FGPIO1PIN_NP39_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR3_f.P9)) )

#define FGPIO1PIN_NP39_PUT(v)         ( bFM_GPIO_FPDOR3_P9=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP39_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP39_INITIN(v) \
                                                         : FGPIO1PIN_NP39_INITOUT(v) )

#define FGPIO1PIN_NP39_INITIN(v)      do{ bFM_GPIO_PCR3_P9=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR3_P9=0u; \
                                          bFM_GPIO_PFR3_P9=0u; }while(0)

#define FGPIO1PIN_NP39_INITOUT(v)     do{ bFM_GPIO_PDOR3_P9=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR3_P9=1u; \
                                          bFM_GPIO_PFR3_P9=0u; }while(0)


/*---- Fast GPIO bit P3A ----*/

#define FGPIO1PIN_P3A_GET             ( FM_FASTIO->FPDIR3_f.PA )

#define FGPIO1PIN_P3A_PUT(v)          ( bFM_GPIO_FPDOR3_PA=(v) )

#define FGPIO1PIN_P3A_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P3A_INITIN(v) \
                                                         : FGPIO1PIN_P3A_INITOUT(v) )

#define FGPIO1PIN_P3A_INITIN(v)       do{ bFM_GPIO_PCR3_PA=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR3_PA=0u; \
                                          bFM_GPIO_PFR3_PA=0u; }while(0)

#define FGPIO1PIN_P3A_INITOUT(v)      do{ bFM_GPIO_FPDOR3_PA=(v).bInitVal; \
                                          bFM_GPIO_DDR3_PA=1u; \
                                          bFM_GPIO_PFR3_PA=0u; }while(0)


/*---- Fast GPIO bit NP3A ----*/
#define FGPIO1PIN_NP3A_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR3_f.PA)) )

#define FGPIO1PIN_NP3A_PUT(v)         ( bFM_GPIO_FPDOR3_PA=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP3A_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP3A_INITIN(v) \
                                                         : FGPIO1PIN_NP3A_INITOUT(v) )

#define FGPIO1PIN_NP3A_INITIN(v)      do{ bFM_GPIO_PCR3_PA=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR3_PA=0u; \
                                          bFM_GPIO_PFR3_PA=0u; }while(0)

#define FGPIO1PIN_NP3A_INITOUT(v)     do{ bFM_GPIO_PDOR3_PA=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR3_PA=1u; \
                                          bFM_GPIO_PFR3_PA=0u; }while(0)


/*---- Fast GPIO bit P3B ----*/

#define FGPIO1PIN_P3B_GET             ( FM_FASTIO->FPDIR3_f.PB )

#define FGPIO1PIN_P3B_PUT(v)          ( bFM_GPIO_FPDOR3_PB=(v) )

#define FGPIO1PIN_P3B_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P3B_INITIN(v) \
                                                         : FGPIO1PIN_P3B_INITOUT(v) )

#define FGPIO1PIN_P3B_INITIN(v)       do{ bFM_GPIO_PCR3_PB=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR3_PB=0u; \
                                          bFM_GPIO_PFR3_PB=0u; }while(0)

#define FGPIO1PIN_P3B_INITOUT(v)      do{ bFM_GPIO_FPDOR3_PB=(v).bInitVal; \
                                          bFM_GPIO_DDR3_PB=1u; \
                                          bFM_GPIO_PFR3_PB=0u; }while(0)


/*---- Fast GPIO bit NP3B ----*/
#define FGPIO1PIN_NP3B_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR3_f.PB)) )

#define FGPIO1PIN_NP3B_PUT(v)         ( bFM_GPIO_FPDOR3_PB=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP3B_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP3B_INITIN(v) \
                                                         : FGPIO1PIN_NP3B_INITOUT(v) )

#define FGPIO1PIN_NP3B_INITIN(v)      do{ bFM_GPIO_PCR3_PB=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR3_PB=0u; \
                                          bFM_GPIO_PFR3_PB=0u; }while(0)

#define FGPIO1PIN_NP3B_INITOUT(v)     do{ bFM_GPIO_PDOR3_PB=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR3_PB=1u; \
                                          bFM_GPIO_PFR3_PB=0u; }while(0)


/*---- Fast GPIO bit P3C ----*/

#define FGPIO1PIN_P3C_GET             ( FM_FASTIO->FPDIR3_f.PC )

#define FGPIO1PIN_P3C_PUT(v)          ( bFM_GPIO_FPDOR3_PC=(v) )

#define FGPIO1PIN_P3C_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P3C_INITIN(v) \
                                                         : FGPIO1PIN_P3C_INITOUT(v) )

#define FGPIO1PIN_P3C_INITIN(v)       do{ bFM_GPIO_PCR3_PC=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR3_PC=0u; \
                                          bFM_GPIO_PFR3_PC=0u; }while(0)

#define FGPIO1PIN_P3C_INITOUT(v)      do{ bFM_GPIO_FPDOR3_PC=(v).bInitVal; \
                                          bFM_GPIO_DDR3_PC=1u; \
                                          bFM_GPIO_PFR3_PC=0u; }while(0)


/*---- Fast GPIO bit NP3C ----*/
#define FGPIO1PIN_NP3C_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR3_f.PC)) )

#define FGPIO1PIN_NP3C_PUT(v)         ( bFM_GPIO_FPDOR3_PC=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP3C_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP3C_INITIN(v) \
                                                         : FGPIO1PIN_NP3C_INITOUT(v) )

#define FGPIO1PIN_NP3C_INITIN(v)      do{ bFM_GPIO_PCR3_PC=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR3_PC=0u; \
                                          bFM_GPIO_PFR3_PC=0u; }while(0)

#define FGPIO1PIN_NP3C_INITOUT(v)     do{ bFM_GPIO_PDOR3_PC=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR3_PC=1u; \
                                          bFM_GPIO_PFR3_PC=0u; }while(0)


/*---- Fast GPIO bit P3D ----*/

#define FGPIO1PIN_P3D_GET             ( FM_FASTIO->FPDIR3_f.PD )

#define FGPIO1PIN_P3D_PUT(v)          ( bFM_GPIO_FPDOR3_PD=(v) )

#define FGPIO1PIN_P3D_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P3D_INITIN(v) \
                                                         : FGPIO1PIN_P3D_INITOUT(v) )

#define FGPIO1PIN_P3D_INITIN(v)       do{ bFM_GPIO_PCR3_PD=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR3_PD=0u; \
                                          bFM_GPIO_PFR3_PD=0u; }while(0)

#define FGPIO1PIN_P3D_INITOUT(v)      do{ bFM_GPIO_FPDOR3_PD=(v).bInitVal; \
                                          bFM_GPIO_DDR3_PD=1u; \
                                          bFM_GPIO_PFR3_PD=0u; }while(0)


/*---- Fast GPIO bit NP3D ----*/
#define FGPIO1PIN_NP3D_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR3_f.PD)) )

#define FGPIO1PIN_NP3D_PUT(v)         ( bFM_GPIO_FPDOR3_PD=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP3D_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP3D_INITIN(v) \
                                                         : FGPIO1PIN_NP3D_INITOUT(v) )

#define FGPIO1PIN_NP3D_INITIN(v)      do{ bFM_GPIO_PCR3_PD=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR3_PD=0u; \
                                          bFM_GPIO_PFR3_PD=0u; }while(0)

#define FGPIO1PIN_NP3D_INITOUT(v)     do{ bFM_GPIO_PDOR3_PD=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR3_PD=1u; \
                                          bFM_GPIO_PFR3_PD=0u; }while(0)


/*---- Fast GPIO bit P3E ----*/

#define FGPIO1PIN_P3E_GET             ( FM_FASTIO->FPDIR3_f.PE )

#define FGPIO1PIN_P3E_PUT(v)          ( bFM_GPIO_FPDOR3_PE=(v) )

#define FGPIO1PIN_P3E_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P3E_INITIN(v) \
                                                         : FGPIO1PIN_P3E_INITOUT(v) )

#define FGPIO1PIN_P3E_INITIN(v)       do{ bFM_GPIO_PCR3_PE=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR3_PE=0u; \
                                          bFM_GPIO_PFR3_PE=0u; }while(0)

#define FGPIO1PIN_P3E_INITOUT(v)      do{ bFM_GPIO_FPDOR3_PE=(v).bInitVal; \
                                          bFM_GPIO_DDR3_PE=1u; \
                                          bFM_GPIO_PFR3_PE=0u; }while(0)


/*---- Fast GPIO bit NP3E ----*/
#define FGPIO1PIN_NP3E_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR3_f.PE)) )

#define FGPIO1PIN_NP3E_PUT(v)         ( bFM_GPIO_FPDOR3_PE=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP3E_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP3E_INITIN(v) \
                                                         : FGPIO1PIN_NP3E_INITOUT(v) )

#define FGPIO1PIN_NP3E_INITIN(v)      do{ bFM_GPIO_PCR3_PE=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR3_PE=0u; \
                                          bFM_GPIO_PFR3_PE=0u; }while(0)

#define FGPIO1PIN_NP3E_INITOUT(v)     do{ bFM_GPIO_PDOR3_PE=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR3_PE=1u; \
                                          bFM_GPIO_PFR3_PE=0u; }while(0)


/*---- Fast GPIO bit P3F ----*/

#define FGPIO1PIN_P3F_GET             ( FM_FASTIO->FPDIR3_f.PF )

#define FGPIO1PIN_P3F_PUT(v)          ( bFM_GPIO_FPDOR3_PF=(v) )

#define FGPIO1PIN_P3F_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P3F_INITIN(v) \
                                                         : FGPIO1PIN_P3F_INITOUT(v) )

#define FGPIO1PIN_P3F_INITIN(v)       do{ bFM_GPIO_PCR3_PF=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR3_PF=0u; \
                                          bFM_GPIO_PFR3_PF=0u; }while(0)

#define FGPIO1PIN_P3F_INITOUT(v)      do{ bFM_GPIO_FPDOR3_PF=(v).bInitVal; \
                                          bFM_GPIO_DDR3_PF=1u; \
                                          bFM_GPIO_PFR3_PF=0u; }while(0)


/*---- Fast GPIO bit NP3F ----*/
#define FGPIO1PIN_NP3F_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR3_f.PF)) )

#define FGPIO1PIN_NP3F_PUT(v)         ( bFM_GPIO_FPDOR3_PF=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP3F_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP3F_INITIN(v) \
                                                         : FGPIO1PIN_NP3F_INITOUT(v) )

#define FGPIO1PIN_NP3F_INITIN(v)      do{ bFM_GPIO_PCR3_PF=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR3_PF=0u; \
                                          bFM_GPIO_PFR3_PF=0u; }while(0)

#define FGPIO1PIN_NP3F_INITOUT(v)     do{ bFM_GPIO_PDOR3_PF=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR3_PF=1u; \
                                          bFM_GPIO_PFR3_PF=0u; }while(0)


/*---- Fast GPIO bit P44 ----*/

#define FGPIO1PIN_P44_GET             ( FM_FASTIO->FPDIR4_f.P4 )

#define FGPIO1PIN_P44_PUT(v)          ( bFM_GPIO_FPDOR4_P4=(v) )

#define FGPIO1PIN_P44_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P44_INITIN(v) \
                                                         : FGPIO1PIN_P44_INITOUT(v) )

#define FGPIO1PIN_P44_INITIN(v)       do{ bFM_GPIO_PCR4_P4=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR4_P4=0u; \
                                          bFM_GPIO_PFR4_P4=0u; }while(0)

#define FGPIO1PIN_P44_INITOUT(v)      do{ bFM_GPIO_FPDOR4_P4=(v).bInitVal; \
                                          bFM_GPIO_DDR4_P4=1u; \
                                          bFM_GPIO_PFR4_P4=0u; }while(0)


/*---- Fast GPIO bit NP44 ----*/
#define FGPIO1PIN_NP44_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR4_f.P4)) )

#define FGPIO1PIN_NP44_PUT(v)         ( bFM_GPIO_FPDOR4_P4=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP44_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP44_INITIN(v) \
                                                         : FGPIO1PIN_NP44_INITOUT(v) )

#define FGPIO1PIN_NP44_INITIN(v)      do{ bFM_GPIO_PCR4_P4=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR4_P4=0u; \
                                          bFM_GPIO_PFR4_P4=0u; }while(0)

#define FGPIO1PIN_NP44_INITOUT(v)     do{ bFM_GPIO_PDOR4_P4=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR4_P4=1u; \
                                          bFM_GPIO_PFR4_P4=0u; }while(0)


/*---- Fast GPIO bit P45 ----*/

#define FGPIO1PIN_P45_GET             ( FM_FASTIO->FPDIR4_f.P5 )

#define FGPIO1PIN_P45_PUT(v)          ( bFM_GPIO_FPDOR4_P5=(v) )

#define FGPIO1PIN_P45_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P45_INITIN(v) \
                                                         : FGPIO1PIN_P45_INITOUT(v) )

#define FGPIO1PIN_P45_INITIN(v)       do{ bFM_GPIO_PCR4_P5=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR4_P5=0u; \
                                          bFM_GPIO_PFR4_P5=0u; }while(0)

#define FGPIO1PIN_P45_INITOUT(v)      do{ bFM_GPIO_FPDOR4_P5=(v).bInitVal; \
                                          bFM_GPIO_DDR4_P5=1u; \
                                          bFM_GPIO_PFR4_P5=0u; }while(0)


/*---- Fast GPIO bit NP45 ----*/
#define FGPIO1PIN_NP45_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR4_f.P5)) )

#define FGPIO1PIN_NP45_PUT(v)         ( bFM_GPIO_FPDOR4_P5=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP45_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP45_INITIN(v) \
                                                         : FGPIO1PIN_NP45_INITOUT(v) )

#define FGPIO1PIN_NP45_INITIN(v)      do{ bFM_GPIO_PCR4_P5=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR4_P5=0u; \
                                          bFM_GPIO_PFR4_P5=0u; }while(0)

#define FGPIO1PIN_NP45_INITOUT(v)     do{ bFM_GPIO_PDOR4_P5=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR4_P5=1u; \
                                          bFM_GPIO_PFR4_P5=0u; }while(0)


/*---- Fast GPIO bit P4A ----*/

#define FGPIO1PIN_P4A_GET             ( FM_FASTIO->FPDIR4_f.PA )

#define FGPIO1PIN_P4A_PUT(v)          ( bFM_GPIO_FPDOR4_PA=(v) )

#define FGPIO1PIN_P4A_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P4A_INITIN(v) \
                                                         : FGPIO1PIN_P4A_INITOUT(v) )

#define FGPIO1PIN_P4A_INITIN(v)       do{ bFM_GPIO_PCR4_PA=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR4_PA=0u; \
                                          bFM_GPIO_PFR4_PA=0u; }while(0)

#define FGPIO1PIN_P4A_INITOUT(v)      do{ bFM_GPIO_FPDOR4_PA=(v).bInitVal; \
                                          bFM_GPIO_DDR4_PA=1u; \
                                          bFM_GPIO_PFR4_PA=0u; }while(0)


/*---- Fast GPIO bit NP4A ----*/
#define FGPIO1PIN_NP4A_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR4_f.PA)) )

#define FGPIO1PIN_NP4A_PUT(v)         ( bFM_GPIO_FPDOR4_PA=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP4A_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP4A_INITIN(v) \
                                                         : FGPIO1PIN_NP4A_INITOUT(v) )

#define FGPIO1PIN_NP4A_INITIN(v)      do{ bFM_GPIO_PCR4_PA=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR4_PA=0u; \
                                          bFM_GPIO_PFR4_PA=0u; }while(0)

#define FGPIO1PIN_NP4A_INITOUT(v)     do{ bFM_GPIO_PDOR4_PA=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR4_PA=1u; \
                                          bFM_GPIO_PFR4_PA=0u; }while(0)


/*---- Fast GPIO bit P4B ----*/

#define FGPIO1PIN_P4B_GET             ( FM_FASTIO->FPDIR4_f.PB )

#define FGPIO1PIN_P4B_PUT(v)          ( bFM_GPIO_FPDOR4_PB=(v) )

#define FGPIO1PIN_P4B_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P4B_INITIN(v) \
                                                         : FGPIO1PIN_P4B_INITOUT(v) )

#define FGPIO1PIN_P4B_INITIN(v)       do{ bFM_GPIO_PCR4_PB=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR4_PB=0u; \
                                          bFM_GPIO_PFR4_PB=0u; }while(0)

#define FGPIO1PIN_P4B_INITOUT(v)      do{ bFM_GPIO_FPDOR4_PB=(v).bInitVal; \
                                          bFM_GPIO_DDR4_PB=1u; \
                                          bFM_GPIO_PFR4_PB=0u; }while(0)


/*---- Fast GPIO bit NP4B ----*/
#define FGPIO1PIN_NP4B_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR4_f.PB)) )

#define FGPIO1PIN_NP4B_PUT(v)         ( bFM_GPIO_FPDOR4_PB=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP4B_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP4B_INITIN(v) \
                                                         : FGPIO1PIN_NP4B_INITOUT(v) )

#define FGPIO1PIN_NP4B_INITIN(v)      do{ bFM_GPIO_PCR4_PB=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR4_PB=0u; \
                                          bFM_GPIO_PFR4_PB=0u; }while(0)

#define FGPIO1PIN_NP4B_INITOUT(v)     do{ bFM_GPIO_PDOR4_PB=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR4_PB=1u; \
                                          bFM_GPIO_PFR4_PB=0u; }while(0)


/*---- Fast GPIO bit P4C ----*/

#define FGPIO1PIN_P4C_GET             ( FM_FASTIO->FPDIR4_f.PC )

#define FGPIO1PIN_P4C_PUT(v)          ( bFM_GPIO_FPDOR4_PC=(v) )

#define FGPIO1PIN_P4C_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P4C_INITIN(v) \
                                                         : FGPIO1PIN_P4C_INITOUT(v) )

#define FGPIO1PIN_P4C_INITIN(v)       do{ bFM_GPIO_PCR4_PC=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR4_PC=0u; \
                                          bFM_GPIO_PFR4_PC=0u; }while(0)

#define FGPIO1PIN_P4C_INITOUT(v)      do{ bFM_GPIO_FPDOR4_PC=(v).bInitVal; \
                                          bFM_GPIO_DDR4_PC=1u; \
                                          bFM_GPIO_PFR4_PC=0u; }while(0)


/*---- Fast GPIO bit NP4C ----*/
#define FGPIO1PIN_NP4C_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR4_f.PC)) )

#define FGPIO1PIN_NP4C_PUT(v)         ( bFM_GPIO_FPDOR4_PC=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP4C_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP4C_INITIN(v) \
                                                         : FGPIO1PIN_NP4C_INITOUT(v) )

#define FGPIO1PIN_NP4C_INITIN(v)      do{ bFM_GPIO_PCR4_PC=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR4_PC=0u; \
                                          bFM_GPIO_PFR4_PC=0u; }while(0)

#define FGPIO1PIN_NP4C_INITOUT(v)     do{ bFM_GPIO_PDOR4_PC=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR4_PC=1u; \
                                          bFM_GPIO_PFR4_PC=0u; }while(0)


/*---- Fast GPIO bit P4D ----*/

#define FGPIO1PIN_P4D_GET             ( FM_FASTIO->FPDIR4_f.PD )

#define FGPIO1PIN_P4D_PUT(v)          ( bFM_GPIO_FPDOR4_PD=(v) )

#define FGPIO1PIN_P4D_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P4D_INITIN(v) \
                                                         : FGPIO1PIN_P4D_INITOUT(v) )

#define FGPIO1PIN_P4D_INITIN(v)       do{ bFM_GPIO_PCR4_PD=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR4_PD=0u; \
                                          bFM_GPIO_PFR4_PD=0u; }while(0)

#define FGPIO1PIN_P4D_INITOUT(v)      do{ bFM_GPIO_FPDOR4_PD=(v).bInitVal; \
                                          bFM_GPIO_DDR4_PD=1u; \
                                          bFM_GPIO_PFR4_PD=0u; }while(0)


/*---- Fast GPIO bit NP4D ----*/
#define FGPIO1PIN_NP4D_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR4_f.PD)) )

#define FGPIO1PIN_NP4D_PUT(v)         ( bFM_GPIO_FPDOR4_PD=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP4D_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP4D_INITIN(v) \
                                                         : FGPIO1PIN_NP4D_INITOUT(v) )

#define FGPIO1PIN_NP4D_INITIN(v)      do{ bFM_GPIO_PCR4_PD=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR4_PD=0u; \
                                          bFM_GPIO_PFR4_PD=0u; }while(0)

#define FGPIO1PIN_NP4D_INITOUT(v)     do{ bFM_GPIO_PDOR4_PD=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR4_PD=1u; \
                                          bFM_GPIO_PFR4_PD=0u; }while(0)


/*---- Fast GPIO bit P50 ----*/

#define FGPIO1PIN_P50_GET             ( FM_FASTIO->FPDIR5_f.P0 )

#define FGPIO1PIN_P50_PUT(v)          ( bFM_GPIO_FPDOR5_P0=(v) )

#define FGPIO1PIN_P50_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P50_INITIN(v) \
                                                         : FGPIO1PIN_P50_INITOUT(v) )

#define FGPIO1PIN_P50_INITIN(v)       do{ bFM_GPIO_PCR5_P0=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR5_P0=0u; \
                                         bFM_GPIO_PFR5_P0=0u; \
                                         bFM_LCDC_LCDCC3_VE4=0u; }while(0)


#define FGPIO1PIN_P50_INITOUT(v)      do{ bFM_GPIO_FPDOR5_P0=(v).bInitVal; \
                                          bFM_GPIO_DDR5_P0=1u; \
                                         bFM_GPIO_PFR5_P0=0u; \
                                         bFM_LCDC_LCDCC3_VE4=0u; }while(0)


/*---- Fast GPIO bit NP50 ----*/
#define FGPIO1PIN_NP50_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR5_f.P0)) )

#define FGPIO1PIN_NP50_PUT(v)         ( bFM_GPIO_FPDOR5_P0=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP50_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP50_INITIN(v) \
                                                         : FGPIO1PIN_NP50_INITOUT(v) )

#define FGPIO1PIN_NP50_INITIN(v)      do{ bFM_GPIO_PCR5_P0=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR5_P0=0u; \
                                         bFM_GPIO_PFR5_P0=0u; \
                                         bFM_LCDC_LCDCC3_VE4=0u; }while(0)


#define FGPIO1PIN_NP50_INITOUT(v)     do{ bFM_GPIO_PDOR5_P0=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR5_P0=1u; \
                                         bFM_GPIO_PFR5_P0=0u; \
                                         bFM_LCDC_LCDCC3_VE4=0u; }while(0)


/*---- Fast GPIO bit P51 ----*/

#define FGPIO1PIN_P51_GET             ( FM_FASTIO->FPDIR5_f.P1 )

#define FGPIO1PIN_P51_PUT(v)          ( bFM_GPIO_FPDOR5_P1=(v) )

#define FGPIO1PIN_P51_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P51_INITIN(v) \
                                                         : FGPIO1PIN_P51_INITOUT(v) )

#define FGPIO1PIN_P51_INITIN(v)       do{ bFM_GPIO_PCR5_P1=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR5_P1=0u; \
                                         bFM_GPIO_PFR5_P1=0u; \
                                         bFM_LCDC_LCDCC3_VE3=0u; }while(0)


#define FGPIO1PIN_P51_INITOUT(v)      do{ bFM_GPIO_FPDOR5_P1=(v).bInitVal; \
                                          bFM_GPIO_DDR5_P1=1u; \
                                         bFM_GPIO_PFR5_P1=0u; \
                                         bFM_LCDC_LCDCC3_VE3=0u; }while(0)


/*---- Fast GPIO bit NP51 ----*/
#define FGPIO1PIN_NP51_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR5_f.P1)) )

#define FGPIO1PIN_NP51_PUT(v)         ( bFM_GPIO_FPDOR5_P1=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP51_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP51_INITIN(v) \
                                                         : FGPIO1PIN_NP51_INITOUT(v) )

#define FGPIO1PIN_NP51_INITIN(v)      do{ bFM_GPIO_PCR5_P1=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR5_P1=0u; \
                                         bFM_GPIO_PFR5_P1=0u; \
                                         bFM_LCDC_LCDCC3_VE3=0u; }while(0)


#define FGPIO1PIN_NP51_INITOUT(v)     do{ bFM_GPIO_PDOR5_P1=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR5_P1=1u; \
                                         bFM_GPIO_PFR5_P1=0u; \
                                         bFM_LCDC_LCDCC3_VE3=0u; }while(0)


/*---- Fast GPIO bit P52 ----*/

#define FGPIO1PIN_P52_GET             ( FM_FASTIO->FPDIR5_f.P2 )

#define FGPIO1PIN_P52_PUT(v)          ( bFM_GPIO_FPDOR5_P2=(v) )

#define FGPIO1PIN_P52_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P52_INITIN(v) \
                                                         : FGPIO1PIN_P52_INITOUT(v) )

#define FGPIO1PIN_P52_INITIN(v)       do{ bFM_GPIO_PCR5_P2=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR5_P2=0u; \
                                         bFM_GPIO_PFR5_P2=0u; \
                                         bFM_LCDC_LCDCC3_VE2=0u; }while(0)


#define FGPIO1PIN_P52_INITOUT(v)      do{ bFM_GPIO_FPDOR5_P2=(v).bInitVal; \
                                          bFM_GPIO_DDR5_P2=1u; \
                                         bFM_GPIO_PFR5_P2=0u; \
                                         bFM_LCDC_LCDCC3_VE2=0u; }while(0)


/*---- Fast GPIO bit NP52 ----*/
#define FGPIO1PIN_NP52_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR5_f.P2)) )

#define FGPIO1PIN_NP52_PUT(v)         ( bFM_GPIO_FPDOR5_P2=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP52_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP52_INITIN(v) \
                                                         : FGPIO1PIN_NP52_INITOUT(v) )

#define FGPIO1PIN_NP52_INITIN(v)      do{ bFM_GPIO_PCR5_P2=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR5_P2=0u; \
                                         bFM_GPIO_PFR5_P2=0u; \
                                         bFM_LCDC_LCDCC3_VE2=0u; }while(0)


#define FGPIO1PIN_NP52_INITOUT(v)     do{ bFM_GPIO_PDOR5_P2=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR5_P2=1u; \
                                         bFM_GPIO_PFR5_P2=0u; \
                                         bFM_LCDC_LCDCC3_VE2=0u; }while(0)


/*---- Fast GPIO bit P53 ----*/

#define FGPIO1PIN_P53_GET             ( FM_FASTIO->FPDIR5_f.P3 )

#define FGPIO1PIN_P53_PUT(v)          ( bFM_GPIO_FPDOR5_P3=(v) )

#define FGPIO1PIN_P53_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P53_INITIN(v) \
                                                         : FGPIO1PIN_P53_INITOUT(v) )

#define FGPIO1PIN_P53_INITIN(v)       do{ bFM_GPIO_PCR5_P3=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR5_P3=0u; \
                                         bFM_GPIO_PFR5_P3=0u; \
                                         bFM_LCDC_LCDCC3_VE1=0u; }while(0)


#define FGPIO1PIN_P53_INITOUT(v)      do{ bFM_GPIO_FPDOR5_P3=(v).bInitVal; \
                                          bFM_GPIO_DDR5_P3=1u; \
                                         bFM_GPIO_PFR5_P3=0u; \
                                         bFM_LCDC_LCDCC3_VE1=0u; }while(0)


/*---- Fast GPIO bit NP53 ----*/
#define FGPIO1PIN_NP53_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR5_f.P3)) )

#define FGPIO1PIN_NP53_PUT(v)         ( bFM_GPIO_FPDOR5_P3=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP53_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP53_INITIN(v) \
                                                         : FGPIO1PIN_NP53_INITOUT(v) )

#define FGPIO1PIN_NP53_INITIN(v)      do{ bFM_GPIO_PCR5_P3=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR5_P3=0u; \
                                         bFM_GPIO_PFR5_P3=0u; \
                                         bFM_LCDC_LCDCC3_VE1=0u; }while(0)


#define FGPIO1PIN_NP53_INITOUT(v)     do{ bFM_GPIO_PDOR5_P3=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR5_P3=1u; \
                                         bFM_GPIO_PFR5_P3=0u; \
                                         bFM_LCDC_LCDCC3_VE1=0u; }while(0)


/*---- Fast GPIO bit P54 ----*/

#define FGPIO1PIN_P54_GET             ( FM_FASTIO->FPDIR5_f.P4 )

#define FGPIO1PIN_P54_PUT(v)          ( bFM_GPIO_FPDOR5_P4=(v) )

#define FGPIO1PIN_P54_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P54_INITIN(v) \
                                                         : FGPIO1PIN_P54_INITOUT(v) )

#define FGPIO1PIN_P54_INITIN(v)       do{ bFM_GPIO_PCR5_P4=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR5_P4=0u; \
                                         bFM_GPIO_PFR5_P4=0u; \
                                         bFM_LCDC_LCDCC3_VE0=0u; }while(0)


#define FGPIO1PIN_P54_INITOUT(v)      do{ bFM_GPIO_FPDOR5_P4=(v).bInitVal; \
                                          bFM_GPIO_DDR5_P4=1u; \
                                         bFM_GPIO_PFR5_P4=0u; \
                                         bFM_LCDC_LCDCC3_VE0=0u; }while(0)


/*---- Fast GPIO bit NP54 ----*/
#define FGPIO1PIN_NP54_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR5_f.P4)) )

#define FGPIO1PIN_NP54_PUT(v)         ( bFM_GPIO_FPDOR5_P4=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP54_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP54_INITIN(v) \
                                                         : FGPIO1PIN_NP54_INITOUT(v) )

#define FGPIO1PIN_NP54_INITIN(v)      do{ bFM_GPIO_PCR5_P4=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR5_P4=0u; \
                                         bFM_GPIO_PFR5_P4=0u; \
                                         bFM_LCDC_LCDCC3_VE0=0u; }while(0)


#define FGPIO1PIN_NP54_INITOUT(v)     do{ bFM_GPIO_PDOR5_P4=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR5_P4=1u; \
                                         bFM_GPIO_PFR5_P4=0u; \
                                         bFM_LCDC_LCDCC3_VE0=0u; }while(0)


/*---- Fast GPIO bit P55 ----*/

#define FGPIO1PIN_P55_GET             ( FM_FASTIO->FPDIR5_f.P5 )

#define FGPIO1PIN_P55_PUT(v)          ( bFM_GPIO_FPDOR5_P5=(v) )

#define FGPIO1PIN_P55_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P55_INITIN(v) \
                                                         : FGPIO1PIN_P55_INITOUT(v) )

#define FGPIO1PIN_P55_INITIN(v)       do{ bFM_GPIO_PCR5_P5=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR5_P5=0u; \
                                          bFM_GPIO_PFR5_P5=0u; }while(0)

#define FGPIO1PIN_P55_INITOUT(v)      do{ bFM_GPIO_FPDOR5_P5=(v).bInitVal; \
                                          bFM_GPIO_DDR5_P5=1u; \
                                          bFM_GPIO_PFR5_P5=0u; }while(0)


/*---- Fast GPIO bit NP55 ----*/
#define FGPIO1PIN_NP55_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR5_f.P5)) )

#define FGPIO1PIN_NP55_PUT(v)         ( bFM_GPIO_FPDOR5_P5=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP55_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP55_INITIN(v) \
                                                         : FGPIO1PIN_NP55_INITOUT(v) )

#define FGPIO1PIN_NP55_INITIN(v)      do{ bFM_GPIO_PCR5_P5=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR5_P5=0u; \
                                          bFM_GPIO_PFR5_P5=0u; }while(0)

#define FGPIO1PIN_NP55_INITOUT(v)     do{ bFM_GPIO_PDOR5_P5=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR5_P5=1u; \
                                          bFM_GPIO_PFR5_P5=0u; }while(0)


/*---- Fast GPIO bit P56 ----*/

#define FGPIO1PIN_P56_GET             ( FM_FASTIO->FPDIR5_f.P6 )

#define FGPIO1PIN_P56_PUT(v)          ( bFM_GPIO_FPDOR5_P6=(v) )

#define FGPIO1PIN_P56_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P56_INITIN(v) \
                                                         : FGPIO1PIN_P56_INITOUT(v) )

#define FGPIO1PIN_P56_INITIN(v)       do{ bFM_GPIO_PCR5_P6=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR5_P6=0u; \
                                          bFM_GPIO_PFR5_P6=0u; }while(0)

#define FGPIO1PIN_P56_INITOUT(v)      do{ bFM_GPIO_FPDOR5_P6=(v).bInitVal; \
                                          bFM_GPIO_DDR5_P6=1u; \
                                          bFM_GPIO_PFR5_P6=0u; }while(0)


/*---- Fast GPIO bit NP56 ----*/
#define FGPIO1PIN_NP56_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR5_f.P6)) )

#define FGPIO1PIN_NP56_PUT(v)         ( bFM_GPIO_FPDOR5_P6=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP56_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP56_INITIN(v) \
                                                         : FGPIO1PIN_NP56_INITOUT(v) )

#define FGPIO1PIN_NP56_INITIN(v)      do{ bFM_GPIO_PCR5_P6=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR5_P6=0u; \
                                          bFM_GPIO_PFR5_P6=0u; }while(0)

#define FGPIO1PIN_NP56_INITOUT(v)     do{ bFM_GPIO_PDOR5_P6=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR5_P6=1u; \
                                          bFM_GPIO_PFR5_P6=0u; }while(0)


/*---- Fast GPIO bit P60 ----*/

#define FGPIO1PIN_P60_GET             ( FM_FASTIO->FPDIR6_f.P0 )

#define FGPIO1PIN_P60_PUT(v)          ( bFM_GPIO_FPDOR6_P0=(v) )

#define FGPIO1PIN_P60_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P60_INITIN(v) \
                                                         : FGPIO1PIN_P60_INITOUT(v) )

#define FGPIO1PIN_P60_INITIN(v)       do{ bFM_GPIO_PCR6_P0=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR6_P0=0u; \
                                          bFM_GPIO_PFR6_P0=0u; }while(0)

#define FGPIO1PIN_P60_INITOUT(v)      do{ bFM_GPIO_FPDOR6_P0=(v).bInitVal; \
                                          bFM_GPIO_DDR6_P0=1u; \
                                          bFM_GPIO_PFR6_P0=0u; }while(0)


/*---- Fast GPIO bit NP60 ----*/
#define FGPIO1PIN_NP60_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR6_f.P0)) )

#define FGPIO1PIN_NP60_PUT(v)         ( bFM_GPIO_FPDOR6_P0=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP60_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP60_INITIN(v) \
                                                         : FGPIO1PIN_NP60_INITOUT(v) )

#define FGPIO1PIN_NP60_INITIN(v)      do{ bFM_GPIO_PCR6_P0=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR6_P0=0u; \
                                          bFM_GPIO_PFR6_P0=0u; }while(0)

#define FGPIO1PIN_NP60_INITOUT(v)     do{ bFM_GPIO_PDOR6_P0=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR6_P0=1u; \
                                          bFM_GPIO_PFR6_P0=0u; }while(0)


/*---- Fast GPIO bit P61 ----*/

#define FGPIO1PIN_P61_GET             ( FM_FASTIO->FPDIR6_f.P1 )

#define FGPIO1PIN_P61_PUT(v)          ( bFM_GPIO_FPDOR6_P1=(v) )

#define FGPIO1PIN_P61_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P61_INITIN(v) \
                                                         : FGPIO1PIN_P61_INITOUT(v) )

#define FGPIO1PIN_P61_INITIN(v)       do{ bFM_GPIO_PCR6_P1=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR6_P1=0u; \
                                          bFM_GPIO_PFR6_P1=0u; }while(0)

#define FGPIO1PIN_P61_INITOUT(v)      do{ bFM_GPIO_FPDOR6_P1=(v).bInitVal; \
                                          bFM_GPIO_DDR6_P1=1u; \
                                          bFM_GPIO_PFR6_P1=0u; }while(0)


/*---- Fast GPIO bit NP61 ----*/
#define FGPIO1PIN_NP61_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR6_f.P1)) )

#define FGPIO1PIN_NP61_PUT(v)         ( bFM_GPIO_FPDOR6_P1=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP61_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP61_INITIN(v) \
                                                         : FGPIO1PIN_NP61_INITOUT(v) )

#define FGPIO1PIN_NP61_INITIN(v)      do{ bFM_GPIO_PCR6_P1=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR6_P1=0u; \
                                          bFM_GPIO_PFR6_P1=0u; }while(0)

#define FGPIO1PIN_NP61_INITOUT(v)     do{ bFM_GPIO_PDOR6_P1=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR6_P1=1u; \
                                          bFM_GPIO_PFR6_P1=0u; }while(0)


/*---- Fast GPIO bit P62 ----*/

#define FGPIO1PIN_P62_GET             ( FM_FASTIO->FPDIR6_f.P2 )

#define FGPIO1PIN_P62_PUT(v)          ( bFM_GPIO_FPDOR6_P2=(v) )

#define FGPIO1PIN_P62_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P62_INITIN(v) \
                                                         : FGPIO1PIN_P62_INITOUT(v) )

#define FGPIO1PIN_P62_INITIN(v)       do{ bFM_GPIO_PCR6_P2=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR6_P2=0u; \
                                          bFM_GPIO_PFR6_P2=0u; }while(0)

#define FGPIO1PIN_P62_INITOUT(v)      do{ bFM_GPIO_FPDOR6_P2=(v).bInitVal; \
                                          bFM_GPIO_DDR6_P2=1u; \
                                          bFM_GPIO_PFR6_P2=0u; }while(0)


/*---- Fast GPIO bit NP62 ----*/
#define FGPIO1PIN_NP62_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR6_f.P2)) )

#define FGPIO1PIN_NP62_PUT(v)         ( bFM_GPIO_FPDOR6_P2=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP62_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP62_INITIN(v) \
                                                         : FGPIO1PIN_NP62_INITOUT(v) )

#define FGPIO1PIN_NP62_INITIN(v)      do{ bFM_GPIO_PCR6_P2=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR6_P2=0u; \
                                          bFM_GPIO_PFR6_P2=0u; }while(0)

#define FGPIO1PIN_NP62_INITOUT(v)     do{ bFM_GPIO_PDOR6_P2=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR6_P2=1u; \
                                          bFM_GPIO_PFR6_P2=0u; }while(0)


/*---- Fast GPIO bit P63 ----*/

#define FGPIO1PIN_P63_GET             ( FM_FASTIO->FPDIR6_f.P3 )

#define FGPIO1PIN_P63_PUT(v)          ( bFM_GPIO_FPDOR6_P3=(v) )

#define FGPIO1PIN_P63_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P63_INITIN(v) \
                                                         : FGPIO1PIN_P63_INITOUT(v) )

#define FGPIO1PIN_P63_INITIN(v)       do{ bFM_GPIO_PCR6_P3=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR6_P3=0u; \
                                          bFM_GPIO_PFR6_P3=0u; }while(0)

#define FGPIO1PIN_P63_INITOUT(v)      do{ bFM_GPIO_FPDOR6_P3=(v).bInitVal; \
                                          bFM_GPIO_DDR6_P3=1u; \
                                          bFM_GPIO_PFR6_P3=0u; }while(0)


/*---- Fast GPIO bit NP63 ----*/
#define FGPIO1PIN_NP63_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR6_f.P3)) )

#define FGPIO1PIN_NP63_PUT(v)         ( bFM_GPIO_FPDOR6_P3=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP63_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP63_INITIN(v) \
                                                         : FGPIO1PIN_NP63_INITOUT(v) )

#define FGPIO1PIN_NP63_INITIN(v)      do{ bFM_GPIO_PCR6_P3=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR6_P3=0u; \
                                          bFM_GPIO_PFR6_P3=0u; }while(0)

#define FGPIO1PIN_NP63_INITOUT(v)     do{ bFM_GPIO_PDOR6_P3=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR6_P3=1u; \
                                          bFM_GPIO_PFR6_P3=0u; }while(0)


/*---- Fast GPIO bit P80 ----*/

#define FGPIO1PIN_P80_GET             ( FM_FASTIO->FPDIR8_f.P0 )

#define FGPIO1PIN_P80_PUT(v)          ( bFM_GPIO_FPDOR8_P0=(v) )

#define FGPIO1PIN_P80_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P80_INITIN(v) \
                                                         : FGPIO1PIN_P80_INITOUT(v) )

#define FGPIO1PIN_P80_INITIN(v)       do{ bFM_GPIO_PCR8_P0=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR8_P0=0u; \
                                          bFM_GPIO_PFR8_P0=0u; }while(0)

#define FGPIO1PIN_P80_INITOUT(v)      do{ bFM_GPIO_FPDOR8_P0=(v).bInitVal; \
                                          bFM_GPIO_DDR8_P0=1u; \
                                          bFM_GPIO_PFR8_P0=0u; }while(0)


/*---- Fast GPIO bit NP80 ----*/
#define FGPIO1PIN_NP80_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR8_f.P0)) )

#define FGPIO1PIN_NP80_PUT(v)         ( bFM_GPIO_FPDOR8_P0=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP80_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP80_INITIN(v) \
                                                         : FGPIO1PIN_NP80_INITOUT(v) )

#define FGPIO1PIN_NP80_INITIN(v)      do{ bFM_GPIO_PCR8_P0=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR8_P0=0u; \
                                          bFM_GPIO_PFR8_P0=0u; }while(0)

#define FGPIO1PIN_NP80_INITOUT(v)     do{ bFM_GPIO_PDOR8_P0=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR8_P0=1u; \
                                          bFM_GPIO_PFR8_P0=0u; }while(0)


/*---- Fast GPIO bit P81 ----*/

#define FGPIO1PIN_P81_GET             ( FM_FASTIO->FPDIR8_f.P1 )

#define FGPIO1PIN_P81_PUT(v)          ( bFM_GPIO_FPDOR8_P1=(v) )

#define FGPIO1PIN_P81_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P81_INITIN(v) \
                                                         : FGPIO1PIN_P81_INITOUT(v) )

#define FGPIO1PIN_P81_INITIN(v)       do{ bFM_GPIO_PCR8_P1=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR8_P1=0u; \
                                          bFM_GPIO_PFR8_P1=0u; }while(0)

#define FGPIO1PIN_P81_INITOUT(v)      do{ bFM_GPIO_FPDOR8_P1=(v).bInitVal; \
                                          bFM_GPIO_DDR8_P1=1u; \
                                          bFM_GPIO_PFR8_P1=0u; }while(0)


/*---- Fast GPIO bit NP81 ----*/
#define FGPIO1PIN_NP81_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR8_f.P1)) )

#define FGPIO1PIN_NP81_PUT(v)         ( bFM_GPIO_FPDOR8_P1=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP81_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP81_INITIN(v) \
                                                         : FGPIO1PIN_NP81_INITOUT(v) )

#define FGPIO1PIN_NP81_INITIN(v)      do{ bFM_GPIO_PCR8_P1=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR8_P1=0u; \
                                          bFM_GPIO_PFR8_P1=0u; }while(0)

#define FGPIO1PIN_NP81_INITOUT(v)     do{ bFM_GPIO_PDOR8_P1=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR8_P1=1u; \
                                          bFM_GPIO_PFR8_P1=0u; }while(0)


/*---- Fast GPIO bit P82 ----*/

#define FGPIO1PIN_P82_GET             ( FM_FASTIO->FPDIR8_f.P2 )

#define FGPIO1PIN_P82_PUT(v)          ( bFM_GPIO_FPDOR8_P2=(v) )

#define FGPIO1PIN_P82_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_P82_INITIN(v) \
                                                         : FGPIO1PIN_P82_INITOUT(v) )

#define FGPIO1PIN_P82_INITIN(v)       do{ bFM_GPIO_PCR8_P2=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR8_P2=0u; \
                                          bFM_GPIO_PFR8_P2=0u; }while(0)

#define FGPIO1PIN_P82_INITOUT(v)      do{ bFM_GPIO_FPDOR8_P2=(v).bInitVal; \
                                          bFM_GPIO_DDR8_P2=1u; \
                                          bFM_GPIO_PFR8_P2=0u; }while(0)


/*---- Fast GPIO bit NP82 ----*/
#define FGPIO1PIN_NP82_GET            ( (uint8_t)(!(FM_FASTIO->FPDIR8_f.P2)) )

#define FGPIO1PIN_NP82_PUT(v)         ( bFM_GPIO_FPDOR8_P2=(uint8_t)(~(v)) )

#define FGPIO1PIN_NP82_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NP82_INITIN(v) \
                                                         : FGPIO1PIN_NP82_INITOUT(v) )

#define FGPIO1PIN_NP82_INITIN(v)      do{ bFM_GPIO_PCR8_P2=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDR8_P2=0u; \
                                          bFM_GPIO_PFR8_P2=0u; }while(0)

#define FGPIO1PIN_NP82_INITOUT(v)     do{ bFM_GPIO_PDOR8_P2=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDR8_P2=1u; \
                                          bFM_GPIO_PFR8_P2=0u; }while(0)


/*---- Fast GPIO bit PE0 ----*/

#define FGPIO1PIN_PE0_GET             ( FM_FASTIO->FPDIRE_f.P0 )

#define FGPIO1PIN_PE0_PUT(v)          ( bFM_GPIO_FPDORE_P0=(v) )

#define FGPIO1PIN_PE0_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_PE0_INITIN(v) \
                                                         : FGPIO1PIN_PE0_INITOUT(v) )

#define FGPIO1PIN_PE0_INITIN(v)       do{ bFM_GPIO_PCRE_P0=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDRE_P0=0u; \
                                          bFM_GPIO_PFRE_P0=0u; }while(0)

#define FGPIO1PIN_PE0_INITOUT(v)      do{ bFM_GPIO_FPDORE_P0=(v).bInitVal; \
                                          bFM_GPIO_DDRE_P0=1u; \
                                          bFM_GPIO_PFRE_P0=0u; }while(0)


/*---- Fast GPIO bit NPE0 ----*/
#define FGPIO1PIN_NPE0_GET            ( (uint8_t)(!(FM_FASTIO->FPDIRE_f.P0)) )

#define FGPIO1PIN_NPE0_PUT(v)         ( bFM_GPIO_FPDORE_P0=(uint8_t)(~(v)) )

#define FGPIO1PIN_NPE0_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NPE0_INITIN(v) \
                                                         : FGPIO1PIN_NPE0_INITOUT(v) )

#define FGPIO1PIN_NPE0_INITIN(v)      do{ bFM_GPIO_PCRE_P0=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDRE_P0=0u; \
                                          bFM_GPIO_PFRE_P0=0u; }while(0)

#define FGPIO1PIN_NPE0_INITOUT(v)     do{ bFM_GPIO_PDORE_P0=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDRE_P0=1u; \
                                          bFM_GPIO_PFRE_P0=0u; }while(0)


/*---- Fast GPIO bit PE2 ----*/

#define FGPIO1PIN_PE2_GET             ( FM_FASTIO->FPDIRE_f.P2 )

#define FGPIO1PIN_PE2_PUT(v)          ( bFM_GPIO_FPDORE_P2=(v) )

#define FGPIO1PIN_PE2_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_PE2_INITIN(v) \
                                                         : FGPIO1PIN_PE2_INITOUT(v) )

#define FGPIO1PIN_PE2_INITIN(v)       do{ bFM_GPIO_PCRE_P2=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDRE_P2=0u; \
                                          bFM_GPIO_SPSR_MAINXC0=0u; \
                                          bFM_GPIO_SPSR_MAINXC1=0u; \
                                          bFM_GPIO_PFRE_P2=0u; }while(0)

#define FGPIO1PIN_PE2_INITOUT(v)      do{ bFM_GPIO_FPDORE_P2=(v).bInitVal; \
                                          bFM_GPIO_DDRE_P2=1u; \
                                          bFM_GPIO_SPSR_MAINXC0=0u; \
                                          bFM_GPIO_SPSR_MAINXC1=0u; \
                                          bFM_GPIO_PFRE_P2=0u; }while(0)


/*---- Fast GPIO bit NPE2 ----*/
#define FGPIO1PIN_NPE2_GET            ( (uint8_t)(!(FM_FASTIO->FPDIRE_f.P2)) )

#define FGPIO1PIN_NPE2_PUT(v)         ( bFM_GPIO_FPDORE_P2=(uint8_t)(~(v)) )

#define FGPIO1PIN_NPE2_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NPE2_INITIN(v) \
                                                         : FGPIO1PIN_NPE2_INITOUT(v) )

#define FGPIO1PIN_NPE2_INITIN(v)      do{ bFM_GPIO_PCRE_P2=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDRE_P2=0u; \
                                          bFM_GPIO_SPSR_MAINXC0=0u; \
                                          bFM_GPIO_SPSR_MAINXC1=0u; \
                                          bFM_GPIO_PFRE_P2=0u; }while(0)

#define FGPIO1PIN_NPE2_INITOUT(v)     do{ bFM_GPIO_PDORE_P2=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDRE_P2=1u; \
                                          bFM_GPIO_SPSR_MAINXC0=0u; \
                                          bFM_GPIO_SPSR_MAINXC1=0u; \
                                          bFM_GPIO_PFRE_P2=0u; }while(0)


/*---- Fast GPIO bit PE3 ----*/

#define FGPIO1PIN_PE3_GET             ( FM_FASTIO->FPDIRE_f.P3 )

#define FGPIO1PIN_PE3_PUT(v)          ( bFM_GPIO_FPDORE_P3=(v) )

#define FGPIO1PIN_PE3_INIT(v)         ( (0==(v).bOutput) ? FGPIO1PIN_PE3_INITIN(v) \
                                                         : FGPIO1PIN_PE3_INITOUT(v) )

#define FGPIO1PIN_PE3_INITIN(v)       do{ bFM_GPIO_PCRE_P3=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDRE_P3=0u; \
                                          bFM_GPIO_SPSR_MAINXC0=0u; \
                                          bFM_GPIO_SPSR_MAINXC1=0u; \
                                          bFM_GPIO_PFRE_P3=0u; }while(0)

#define FGPIO1PIN_PE3_INITOUT(v)      do{ bFM_GPIO_FPDORE_P3=(v).bInitVal; \
                                          bFM_GPIO_DDRE_P3=1u; \
                                          bFM_GPIO_SPSR_MAINXC0=0u; \
                                          bFM_GPIO_SPSR_MAINXC1=0u; \
                                          bFM_GPIO_PFRE_P3=0u; }while(0)


/*---- Fast GPIO bit NPE3 ----*/
#define FGPIO1PIN_NPE3_GET            ( (uint8_t)(!(FM_FASTIO->FPDIRE_f.P3)) )

#define FGPIO1PIN_NPE3_PUT(v)         ( bFM_GPIO_FPDORE_P3=(uint8_t)(~(v)) )

#define FGPIO1PIN_NPE3_INIT(v)        ( (0==(v).bOutput) ? FGPIO1PIN_NPE3_INITIN(v) \
                                                         : FGPIO1PIN_NPE3_INITOUT(v) )

#define FGPIO1PIN_NPE3_INITIN(v)      do{ bFM_GPIO_PCRE_P3=(v).bPullup; \
                                         bFM_LCDC_LCDCC3_PICTL=1u; \
                                          bFM_GPIO_DDRE_P3=0u; \
                                          bFM_GPIO_SPSR_MAINXC0=0u; \
                                          bFM_GPIO_SPSR_MAINXC1=0u; \
                                          bFM_GPIO_PFRE_P3=0u; }while(0)

#define FGPIO1PIN_NPE3_INITOUT(v)     do{ bFM_GPIO_PDORE_P3=(uint8_t)(~((uint8_t)(v).bInitVal)); \
                                          bFM_GPIO_DDRE_P3=1u; \
                                          bFM_GPIO_SPSR_MAINXC0=0u; \
                                          bFM_GPIO_SPSR_MAINXC1=0u; \
                                          bFM_GPIO_PFRE_P3=0u; }while(0)



/******************************************************************************
   PIN RELOCATION
*******************************************************************************/

/*--- ADTG_0 for ADC#0 ---*/
#define SetPinFunc_ADTG_0_ADC0(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 12u, 4u, 0x01u ); \
                                                 bFM_GPIO_PFR0_P7 = 1u; \
                                                 bFM_GPIO_ADE_AN22 = 0u; }while(0)

/*--- ADTG_0 for ADC#1 ---*/
#define SetPinFunc_ADTG_0_ADC1(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 16u, 4u, 0x01u ); \
                                                 bFM_GPIO_PFR0_P7 = 1u; \
                                                 bFM_GPIO_ADE_AN22 = 0u; }while(0)

/*--- ADTG_0 for ADC#2 ---*/
#define SetPinFunc_ADTG_0_ADC2(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 20u, 4u, 0x01u ); \
                                                 bFM_GPIO_PFR0_P7 = 1u; \
                                                 bFM_GPIO_ADE_AN22 = 0u; }while(0)

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

/*--- ADTG_6 for ADC#0 ---*/
#define SetPinFunc_ADTG_6_ADC0(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 12u, 4u, 0x07u ); \
                                                 bFM_GPIO_PFR3_P3 = 1u; }while(0)

/*--- ADTG_6 for ADC#1 ---*/
#define SetPinFunc_ADTG_6_ADC1(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 16u, 4u, 0x07u ); \
                                                 bFM_GPIO_PFR3_P3 = 1u; }while(0)

/*--- ADTG_6 for ADC#2 ---*/
#define SetPinFunc_ADTG_6_ADC2(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR09, 20u, 4u, 0x07u ); \
                                                 bFM_GPIO_PFR3_P3 = 1u; }while(0)

/*--- CEC0_0 ---*/
#define SetPinFunc_CEC0_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR18, 0u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR4_PC = 1u; }while(0)

/*--- CEC0_1 ---*/
#define SetPinFunc_CEC0_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR18, 0u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR0_PA = 1u; }while(0)

/*--- CEC1_0 ---*/
#define SetPinFunc_CEC1_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR18, 2u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR6_P0 = 1u; }while(0)

/*--- CEC1_1 ---*/
#define SetPinFunc_CEC1_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR18, 2u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR5_P6 = 1u; }while(0)

/*--- CROUT_1 ---*/
#define SetPinFunc_CROUT_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR00, 1u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR0_PF = 1u; }while(0)

/*--- CTS1_1 ---*/
#define SetPinFunc_CTS1_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR34, 6u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P0 = 1u; \
                                                 bFM_GPIO_ADE_AN00 = 0u; }while(0)

/*--- CTS4_0 ---*/
#define SetPinFunc_CTS4_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 2u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR0_PE = 1u; }while(0)

/*--- DTTI0X_0 ---*/
#define SetPinFunc_DTTI0X_0(dummy)           do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 16u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_P9 = 1u; }while(0)

/*--- DTTI0X_2 ---*/
#define SetPinFunc_DTTI0X_2(dummy)           do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 16u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR6_P1 = 1u; }while(0)

/*--- FRCK0_2 ---*/
#define SetPinFunc_FRCK0_2(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 18u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR1_P1 = 1u; \
                                                 bFM_GPIO_ADE_AN01 = 0u; }while(0)

/*--- I2SCK5_0 ---*/
#define SetPinFunc_I2SCK5_0(dummy)           do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR37, 20u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR6_P2 = 1u; }while(0)

/*--- I2SCK6_1 ---*/
#define SetPinFunc_I2SCK6_1(dummy)           do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR38, 4u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P1 = 1u; }while(0)

/*--- I2SDI5_0 ---*/
#define SetPinFunc_I2SDI5_0(dummy)           do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR37, 24u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR6_P0 = 1u; }while(0)

/*--- I2SDI6_1 ---*/
#define SetPinFunc_I2SDI6_1(dummy)           do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR38, 8u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P3 = 1u; }while(0)

/*--- I2SDO5_0 ---*/
#define SetPinFunc_I2SDO5_0(dummy)           do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR37, 26u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR6_P1 = 1u; }while(0)

/*--- I2SDO6_1 ---*/
#define SetPinFunc_I2SDO6_1(dummy)           do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR38, 10u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P2 = 1u; }while(0)

/*--- I2SMCK5_0_IN ---*/
#define SetPinFunc_I2SMCK5_0_IN(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR37, 16u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR0_PF = 1u; }while(0)

/*--- I2SMCK5_0_OUT ---*/
#define SetPinFunc_I2SMCK5_0_OUT(dummy)      do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR37, 18u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR0_PF = 1u; }while(0)

/*--- I2SMCK6_1_IN ---*/
#define SetPinFunc_I2SMCK6_1_IN(dummy)       do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR38, 0u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR5_P6 = 1u; }while(0)

/*--- I2SMCK6_1_OUT ---*/
#define SetPinFunc_I2SMCK6_1_OUT(dummy)      do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR38, 2u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR5_P6 = 1u; }while(0)

/*--- I2SWS5_0 ---*/
#define SetPinFunc_I2SWS5_0(dummy)           do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR37, 22u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR6_P3 = 1u; }while(0)

/*--- I2SWS6_1 ---*/
#define SetPinFunc_I2SWS6_1(dummy)           do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR38, 6u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P0 = 1u; }while(0)

/*--- IC00_1 ---*/
#define SetPinFunc_IC00_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 20u, 3u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P9 = 1u; \
                                                 bFM_GPIO_ADE_AN09 = 0u; }while(0)

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
                                                 bFM_GPIO_PFR1_PA = 1u; \
                                                 bFM_GPIO_ADE_AN10 = 0u; }while(0)

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

/*--- IC02_1 ---*/
#define SetPinFunc_IC02_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 26u, 3u, 0x02u ); \
                                                 bFM_GPIO_PFR1_PB = 1u; \
                                                 bFM_GPIO_ADE_AN11 = 0u; }while(0)

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

/*--- IC0_CIN_0 ---*/
#define SetPinFunc_IC0_CIN_0(dummy)          do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR33, 0u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR0_PA = 1u; }while(0)

/*--- IC0_CIN_1 ---*/
#define SetPinFunc_IC0_CIN_1(dummy)          do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR33, 0u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR4_P5 = 1u; }while(0)

/*--- IC0_CLK_0 ---*/
#define SetPinFunc_IC0_CLK_0(dummy)          do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR33, 10u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR0_PF = 1u; }while(0)

/*--- IC0_DATA_0 ---*/
#define SetPinFunc_IC0_DATA_0(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR33, 2u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR6_P3 = 1u; }while(0)

/*--- IC0_DATA_1 ---*/
#define SetPinFunc_IC0_DATA_1(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR33, 2u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR4_P4 = 1u; }while(0)

/*--- IC0_RST_0 ---*/
#define SetPinFunc_IC0_RST_0(dummy)          do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR33, 4u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR6_P2 = 1u; }while(0)

/*--- IC1_CIN_0 ---*/
#define SetPinFunc_IC1_CIN_0(dummy)          do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR33, 16u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_PA = 1u; }while(0)

/*--- IC1_CIN_1 ---*/
#define SetPinFunc_IC1_CIN_1(dummy)          do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR33, 16u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P5 = 1u; \
                                                 bFM_GPIO_ADE_AN05 = 0u; }while(0)

/*--- IC1_CLK_0 ---*/
#define SetPinFunc_IC1_CLK_0(dummy)          do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR33, 26u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_PF = 1u; }while(0)

/*--- IC1_CLK_1 ---*/
#define SetPinFunc_IC1_CLK_1(dummy)          do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR33, 26u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P0 = 1u; \
                                                 bFM_GPIO_ADE_AN00 = 0u; }while(0)

/*--- IC1_DATA_0 ---*/
#define SetPinFunc_IC1_DATA_0(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR33, 18u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_PB = 1u; }while(0)

/*--- IC1_DATA_1 ---*/
#define SetPinFunc_IC1_DATA_1(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR33, 18u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P4 = 1u; \
                                                 bFM_GPIO_ADE_AN04 = 0u; }while(0)

/*--- IC1_RST_0 ---*/
#define SetPinFunc_IC1_RST_0(dummy)          do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR33, 20u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_PC = 1u; }while(0)

/*--- IC1_RST_1 ---*/
#define SetPinFunc_IC1_RST_1(dummy)          do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR33, 20u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P3 = 1u; \
                                                 bFM_GPIO_ADE_AN03 = 0u; }while(0)

/*--- IC1_VCC_0 ---*/
#define SetPinFunc_IC1_VCC_0(dummy)          do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR33, 24u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_PE = 1u; }while(0)

/*--- IC1_VCC_1 ---*/
#define SetPinFunc_IC1_VCC_1(dummy)          do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR33, 24u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P1 = 1u; \
                                                 bFM_GPIO_ADE_AN01 = 0u; }while(0)

/*--- IC1_VPEN_0 ---*/
#define SetPinFunc_IC1_VPEN_0(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR33, 22u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_PD = 1u; }while(0)

/*--- IC1_VPEN_1 ---*/
#define SetPinFunc_IC1_VPEN_1(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR33, 22u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P2 = 1u; \
                                                 bFM_GPIO_ADE_AN02 = 0u; }while(0)

/*--- IGTRG0_1 ---*/
#define SetPinFunc_IGTRG0_1(dummy)           do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 13u, 1u, 0x01u ); \
                                                 bFM_GPIO_PFR6_P0 = 1u; }while(0)

/*--- INT00_0 ---*/
#define SetPinFunc_INT00_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 0u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P0 = 1u; \
                                                 bFM_LCDC_LCDCC3_VE4 = 0u; }while(0)

/*--- INT00_2 ---*/
#define SetPinFunc_INT00_2(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 0u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR0_PA = 1u; }while(0)

/*--- INT01_0 ---*/
#define SetPinFunc_INT01_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 2u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P1 = 1u; \
                                                 bFM_LCDC_LCDCC3_VE3 = 0u; }while(0)

/*--- INT02_0 ---*/
#define SetPinFunc_INT02_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 4u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P2 = 1u; \
                                                 bFM_LCDC_LCDCC3_VE2 = 0u; }while(0)

/*--- INT02_1 ---*/
#define SetPinFunc_INT02_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 4u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P1 = 1u; \
                                                 bFM_GPIO_ADE_AN01 = 0u; }while(0)

/*--- INT03_0 ---*/
#define SetPinFunc_INT03_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 6u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR6_P3 = 1u; }while(0)

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

/*--- INT04_1 ---*/
#define SetPinFunc_INT04_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 8u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P7 = 1u; \
                                                 bFM_GPIO_ADE_AN07 = 0u; }while(0)

/*--- INT04_2 ---*/
#define SetPinFunc_INT04_2(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 8u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR3_P1 = 1u; }while(0)

/*--- INT05_1 ---*/
#define SetPinFunc_INT05_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 10u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_PA = 1u; \
                                                 bFM_GPIO_ADE_AN10 = 0u; }while(0)

/*--- INT05_2 ---*/
#define SetPinFunc_INT05_2(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 10u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR3_P2 = 1u; }while(0)

/*--- INT06_0 ---*/
#define SetPinFunc_INT06_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 12u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_P9 = 1u; }while(0)

/*--- INT06_1 ---*/
#define SetPinFunc_INT06_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 12u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P1 = 1u; \
                                                 bFM_GPIO_ADE_AN18 = 0u; }while(0)

/*--- INT06_2 ---*/
#define SetPinFunc_INT06_2(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 12u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR0_P4 = 1u; }while(0)

/*--- INT07_0 ---*/
#define SetPinFunc_INT07_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 14u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_PA = 1u; }while(0)

/*--- INT07_1 ---*/
#define SetPinFunc_INT07_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 14u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR6_P2 = 1u; }while(0)

/*--- INT07_2 ---*/
#define SetPinFunc_INT07_2(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 14u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR5_P3 = 1u; \
                                                 bFM_LCDC_LCDCC3_VE1 = 0u; }while(0)

/*--- INT08_2 ---*/
#define SetPinFunc_INT08_2(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 16u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR5_P6 = 1u; }while(0)

/*--- INT10_0 ---*/
#define SetPinFunc_INT10_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 20u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR4_P4 = 1u; }while(0)

/*--- INT11_0 ---*/
#define SetPinFunc_INT11_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 22u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR8_P1 = 1u; }while(0)

/*--- INT12_0 ---*/
#define SetPinFunc_INT12_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 24u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR4_PC = 1u; }while(0)

/*--- INT13_0 ---*/
#define SetPinFunc_INT13_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 26u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR4_PD = 1u; }while(0)

/*--- INT14_0 ---*/
#define SetPinFunc_INT14_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 28u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR1_P5 = 1u; \
                                                 bFM_GPIO_ADE_AN05 = 0u; }while(0)

/*--- INT14_1 ---*/
#define SetPinFunc_INT14_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 28u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR0_P0 = 1u; }while(0)

/*--- INT15_0 ---*/
#define SetPinFunc_INT15_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 30u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR1_P6 = 1u; \
                                                 bFM_GPIO_ADE_AN06 = 0u; }while(0)

/*--- INT15_1 ---*/
#define SetPinFunc_INT15_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR06, 30u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR6_P0 = 1u; }while(0)

/*--- INT18_0 ---*/
#define SetPinFunc_INT18_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR15, 4u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR0_PB = 1u; }while(0)

/*--- INT18_1 ---*/
#define SetPinFunc_INT18_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR15, 4u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR5_P4 = 1u; \
                                                 bFM_LCDC_LCDCC3_VE0 = 0u; }while(0)

/*--- INT18_2 ---*/
#define SetPinFunc_INT18_2(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR15, 4u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR3_PC = 1u; }while(0)

/*--- INT19_0 ---*/
#define SetPinFunc_INT19_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR15, 6u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR0_PC = 1u; }while(0)

/*--- INT19_1 ---*/
#define SetPinFunc_INT19_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR15, 6u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR5_P5 = 1u; }while(0)

/*--- INT19_2 ---*/
#define SetPinFunc_INT19_2(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR15, 6u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR3_PE = 1u; }while(0)

/*--- INT20_0 ---*/
#define SetPinFunc_INT20_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR15, 8u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR0_PD = 1u; }while(0)

/*--- INT20_1 ---*/
#define SetPinFunc_INT20_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR15, 8u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR8_P0 = 1u; }while(0)

/*--- INT20_2 ---*/
#define SetPinFunc_INT20_2(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR15, 8u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR1_PB = 1u; \
                                                 bFM_GPIO_ADE_AN11 = 0u; }while(0)

/*--- INT21_0 ---*/
#define SetPinFunc_INT21_0(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR15, 10u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR0_PE = 1u; }while(0)

/*--- INT21_1 ---*/
#define SetPinFunc_INT21_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR15, 10u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR4_PA = 1u; }while(0)

/*--- INT22_1 ---*/
#define SetPinFunc_INT22_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR15, 12u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR4_PB = 1u; }while(0)

/*--- INT23_1 ---*/
#define SetPinFunc_INT23_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR15, 14u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR0_P7 = 1u; \
                                                 bFM_GPIO_ADE_AN22 = 0u; }while(0)

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

/*--- RTO00_1 ---*/
#define SetPinFunc_RTO00_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR01, 0u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P3 = 1u; \
                                                 bFM_GPIO_ADE_AN16 = 0u; }while(0)

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

/*--- RTS1_1 ---*/
#define SetPinFunc_RTS1_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR34, 4u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P4 = 1u; \
                                                 bFM_GPIO_ADE_AN04 = 0u; }while(0)

/*--- RTS1_2 ---*/
#define SetPinFunc_RTS1_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR34, 4u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR4_P4 = 1u; }while(0)

/*--- RTS4_0 ---*/
#define SetPinFunc_RTS4_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 0u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR0_PD = 1u; }while(0)

/*--- SCK0_0 ---*/
#define SetPinFunc_SCK0_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 8u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR2_P3 = 1u; \
                                                 bFM_GPIO_ADE_AN16 = 0u; }while(0)

/*--- SCK0_1 ---*/
#define SetPinFunc_SCK0_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 8u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P6 = 1u; \
                                                 bFM_GPIO_ADE_AN06 = 0u; }while(0)

/*--- SCK1_1 ---*/
#define SetPinFunc_SCK1_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 14u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P3 = 1u; \
                                                 bFM_GPIO_ADE_AN03 = 0u; }while(0)

/*--- SCK2_2 ---*/
#define SetPinFunc_SCK2_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 20u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR1_P9 = 1u; \
                                                 bFM_GPIO_ADE_AN09 = 0u; }while(0)

/*--- SCK3_1 ---*/
#define SetPinFunc_SCK3_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 26u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR5_P2 = 1u; \
                                                 bFM_LCDC_LCDCC3_VE2 = 0u; }while(0)

/*--- SCK3_2 ---*/
#define SetPinFunc_SCK3_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 26u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR0_P4 = 1u; }while(0)

/*--- SCK4_2 ---*/
#define SetPinFunc_SCK4_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 8u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR0_P7 = 1u; \
                                                 bFM_GPIO_ADE_AN22 = 0u; }while(0)

/*--- SCK5_0 ---*/
#define SetPinFunc_SCK5_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 14u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR6_P2 = 1u; }while(0)

/*--- SCK6_0 ---*/
#define SetPinFunc_SCK6_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 20u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P5 = 1u; }while(0)

/*--- SCK6_1 ---*/
#define SetPinFunc_SCK6_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 20u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P1 = 1u; }while(0)

/*--- SCK7_1 ---*/
#define SetPinFunc_SCK7_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 26u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR4_PD = 1u; }while(0)

/*--- SCK7_2 ---*/
#define SetPinFunc_SCK7_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 26u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR8_P2 = 1u; }while(0)

/*--- SCS60_1 ---*/
#define SetPinFunc_SCS60_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR23, 0u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P0 = 1u; }while(0)

/*--- SCS70_1 ---*/
#define SetPinFunc_SCS70_1(dummy)            do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR23, 8u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR4_PA = 1u; }while(0)

/*--- SIN0_0 ---*/
#define SetPinFunc_SIN0_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 4u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR2_P1 = 1u; \
                                                 bFM_GPIO_ADE_AN18 = 0u; }while(0)

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

/*--- SIN2_2 ---*/
#define SetPinFunc_SIN2_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 16u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR1_P7 = 1u; \
                                                 bFM_GPIO_ADE_AN07 = 0u; }while(0)

/*--- SIN3_1 ---*/
#define SetPinFunc_SIN3_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 22u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR5_P0 = 1u; \
                                                 bFM_LCDC_LCDCC3_VE4 = 0u; }while(0)

/*--- SIN3_2 ---*/
#define SetPinFunc_SIN3_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 22u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR0_P2 = 1u; }while(0)

/*--- SIN4_0 ---*/
#define SetPinFunc_SIN4_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 4u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR0_PA = 1u; }while(0)

/*--- SIN4_1 ---*/
#define SetPinFunc_SIN4_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 4u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_PA = 1u; \
                                                 bFM_GPIO_ADE_AN10 = 0u; }while(0)

/*--- SIN5_0 ---*/
#define SetPinFunc_SIN5_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 10u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR6_P0 = 1u; }while(0)

/*--- SIN5_1 ---*/
#define SetPinFunc_SIN5_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 10u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR6_P3 = 1u; }while(0)

/*--- SIN6_0 ---*/
#define SetPinFunc_SIN6_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 16u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P3 = 1u; \
                                                 bFM_LCDC_LCDCC3_VE1 = 0u; }while(0)

/*--- SIN6_1 ---*/
#define SetPinFunc_SIN6_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 16u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P3 = 1u; }while(0)

/*--- SIN7_1 ---*/
#define SetPinFunc_SIN7_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 22u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR4_PB = 1u; }while(0)

/*--- SIN7_2 ---*/
#define SetPinFunc_SIN7_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 22u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR8_P0 = 1u; }while(0)

/*--- SOT0_0 ---*/
#define SetPinFunc_SOT0_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 6u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR2_P2 = 1u; \
                                                 bFM_GPIO_ADE_AN17 = 0u; }while(0)

/*--- SOT0_1 ---*/
#define SetPinFunc_SOT0_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 6u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P5 = 1u; \
                                                 bFM_GPIO_ADE_AN05 = 0u; }while(0)

/*--- SOT1_1 ---*/
#define SetPinFunc_SOT1_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 12u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_P2 = 1u; \
                                                 bFM_GPIO_ADE_AN02 = 0u; }while(0)

/*--- SOT2_2 ---*/
#define SetPinFunc_SOT2_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 18u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR1_P8 = 1u; \
                                                 bFM_GPIO_ADE_AN08 = 0u; }while(0)

/*--- SOT3_1 ---*/
#define SetPinFunc_SOT3_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 24u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR5_P1 = 1u; \
                                                 bFM_LCDC_LCDCC3_VE3 = 0u; }while(0)

/*--- SOT3_2 ---*/
#define SetPinFunc_SOT3_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR07, 24u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR0_P0 = 1u; }while(0)

/*--- SOT4_0 ---*/
#define SetPinFunc_SOT4_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 6u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR0_P7 = 1u; \
                                                 bFM_GPIO_ADE_AN22 = 0u; }while(0)

/*--- SOT4_1 ---*/
#define SetPinFunc_SOT4_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 6u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR1_PB = 1u; \
                                                 bFM_GPIO_ADE_AN11 = 0u; }while(0)

/*--- SOT5_0 ---*/
#define SetPinFunc_SOT5_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 12u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR6_P1 = 1u; }while(0)

/*--- SOT6_0 ---*/
#define SetPinFunc_SOT6_0(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 18u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR5_P4 = 1u; \
                                                 bFM_LCDC_LCDCC3_VE0 = 0u; }while(0)

/*--- SOT6_1 ---*/
#define SetPinFunc_SOT6_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 18u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P2 = 1u; }while(0)

/*--- SOT7_1 ---*/
#define SetPinFunc_SOT7_1(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 24u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR4_PC = 1u; }while(0)

/*--- SOT7_2 ---*/
#define SetPinFunc_SOT7_2(dummy)             do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR08, 24u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR8_P1 = 1u; }while(0)

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

/*--- TIOA0_1_OUT ---*/
#define SetPinFunc_TIOA0_1_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 2u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_PA = 1u; }while(0)

/*--- TIOA1_1_IN ---*/
#define SetPinFunc_TIOA1_1_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 8u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_PB = 1u; }while(0)

/*--- TIOA1_1_OUT ---*/
#define SetPinFunc_TIOA1_1_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 10u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_PB = 1u; }while(0)

/*--- TIOA1_2_IN ---*/
#define SetPinFunc_TIOA1_2_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 8u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR5_P3 = 1u; \
                                                 bFM_LCDC_LCDCC3_VE1 = 0u; }while(0)

/*--- TIOA1_2_OUT ---*/
#define SetPinFunc_TIOA1_2_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 10u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR5_P3 = 1u; \
                                                 bFM_LCDC_LCDCC3_VE1 = 0u; }while(0)

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

/*--- TIOA3_2_IN ---*/
#define SetPinFunc_TIOA3_2_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 24u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR0_PD = 1u; }while(0)

/*--- TIOA3_2_OUT ---*/
#define SetPinFunc_TIOA3_2_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 26u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR0_PD = 1u; }while(0)

/*--- TIOA4_0_OUT ---*/
#define SetPinFunc_TIOA4_0_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 2u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR4_P4 = 1u; }while(0)

/*--- TIOA4_1_OUT ---*/
#define SetPinFunc_TIOA4_1_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 2u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_PE = 1u; }while(0)

/*--- TIOA5_0_IN ---*/
#define SetPinFunc_TIOA5_0_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 8u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR4_P5 = 1u; }while(0)

/*--- TIOA5_0_OUT ---*/
#define SetPinFunc_TIOA5_0_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 10u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR4_P5 = 1u; }while(0)

/*--- TIOA5_1_IN ---*/
#define SetPinFunc_TIOA5_1_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 8u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_PF = 1u; }while(0)

/*--- TIOA5_1_OUT ---*/
#define SetPinFunc_TIOA5_1_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 10u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_PF = 1u; }while(0)

/*--- TIOA6_1_OUT ---*/
#define SetPinFunc_TIOA6_1_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 18u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR6_P2 = 1u; }while(0)

/*--- TIOA7_1_IN ---*/
#define SetPinFunc_TIOA7_1_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 24u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P3 = 1u; \
                                                 bFM_GPIO_ADE_AN16 = 0u; }while(0)

/*--- TIOA7_1_OUT ---*/
#define SetPinFunc_TIOA7_1_OUT(dummy)        do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 26u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P3 = 1u; \
                                                 bFM_GPIO_ADE_AN16 = 0u; }while(0)

/*--- TIOB0_0_IN ---*/
#define SetPinFunc_TIOB0_0_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 4u, 3u, 0x01u ); \
                                                 bFM_GPIO_PFR4_PA = 1u; }while(0)

/*--- TIOB0_1_IN ---*/
#define SetPinFunc_TIOB0_1_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 4u, 3u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P0 = 1u; }while(0)

/*--- TIOB1_0_IN ---*/
#define SetPinFunc_TIOB1_0_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 12u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR4_PB = 1u; }while(0)

/*--- TIOB1_1_IN ---*/
#define SetPinFunc_TIOB1_1_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 12u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P1 = 1u; }while(0)

/*--- TIOB1_2_IN ---*/
#define SetPinFunc_TIOB1_2_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 12u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR5_P4 = 1u; \
                                                 bFM_LCDC_LCDCC3_VE0 = 0u; }while(0)

/*--- TIOB2_0_IN ---*/
#define SetPinFunc_TIOB2_0_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 20u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR4_PC = 1u; }while(0)

/*--- TIOB2_1_IN ---*/
#define SetPinFunc_TIOB2_1_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 20u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P2 = 1u; }while(0)

/*--- TIOB2_2_IN ---*/
#define SetPinFunc_TIOB2_2_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 20u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR6_P1 = 1u; }while(0)

/*--- TIOB3_0_IN ---*/
#define SetPinFunc_TIOB3_0_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 28u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR4_PD = 1u; }while(0)

/*--- TIOB3_1_IN ---*/
#define SetPinFunc_TIOB3_1_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 28u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR3_P3 = 1u; }while(0)

/*--- TIOB3_2_IN ---*/
#define SetPinFunc_TIOB3_2_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR04, 28u, 2u, 0x03u ); \
                                                 bFM_GPIO_PFR0_PE = 1u; }while(0)

/*--- TIOB4_0_IN ---*/
#define SetPinFunc_TIOB4_0_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 4u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR3_P9 = 1u; }while(0)

/*--- TIOB5_0_IN ---*/
#define SetPinFunc_TIOB5_0_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 12u, 2u, 0x01u ); \
                                                 bFM_GPIO_PFR0_P2 = 1u; }while(0)

/*--- TIOB6_1_IN ---*/
#define SetPinFunc_TIOB6_1_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 20u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR6_P3 = 1u; }while(0)

/*--- TIOB7_1_IN ---*/
#define SetPinFunc_TIOB7_1_IN(dummy)         do{ PINRELOC_SET_EPFR( FM_GPIO->EPFR05, 28u, 2u, 0x02u ); \
                                                 bFM_GPIO_PFR2_P2 = 1u; \
                                                 bFM_GPIO_ADE_AN17 = 0u; }while(0)


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

/*--- AN16 ---*/
#define SetPinFunc_AN16(dummy)               ( bFM_GPIO_ADE_AN16 = 1 )

/*--- AN17 ---*/
#define SetPinFunc_AN17(dummy)               ( bFM_GPIO_ADE_AN17 = 1 )

/*--- AN18 ---*/
#define SetPinFunc_AN18(dummy)               ( bFM_GPIO_ADE_AN18 = 1 )

/*--- AN22 ---*/
#define SetPinFunc_AN22(dummy)               ( bFM_GPIO_ADE_AN22 = 1 )


#endif // #ifndef __GPIO_S6E1B7XE_H__


/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/

