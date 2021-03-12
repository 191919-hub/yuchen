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
/** \file gpio.h
 **
 ** Headerfile for GPIO functions
 **
 ** History:
 **   - 2014-11-10  1.0  EZh    First version.
 **
 ******************************************************************************/

#ifndef __GPIO_H__
#define __GPIO_H__

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "mcu.h"
#include "pdl_user.h"

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/**
 ******************************************************************************
 ** \defgroup GpioGroup GPIO Defintions (GPIO)
 **
 ** Definitions of GPIO and resource pin relocation
 **
 ** \attention
 **            - Carefully check in device documentation, whether SOUBOUT pin
 **              at SOUBOUT[_n] or TIOB0 pin should be output. TIOB0-SUBOUT
 **              is <b>not</b> provided by this driver!
 **            - Internal LSYN connection is not provided by this driver!
 ******************************************************************************/
//@{

/******************************************************************************/
/* Defines                                                                    */
/******************************************************************************/

#define Gpio1pin_InitIn(p,settings)    do{ stc_gpio1pin_init_t __v__;\
                                         __v__.bPullup=0u;__v__.bInitVal=0u;\
                                         (settings);\
                                         p##_INITIN(__v__); }while(0)

#define Gpio1pin_InitOut(p,settings)   do{ stc_gpio1pin_init_t __v__;\
                                         __v__.bPullup=0u;__v__.bInitVal=0u;\
                                         (settings);\
                                         p##_INITOUT(__v__); }while(0)

#define Gpio1pin_Init(p,settings)      do{ stc_gpio1pin_init_t __v__;__v__.bOutput=0u;\
                                         __v__.bPullup=0u;__v__.bInitVal=0u;\
                                         (settings);\
                                         p##_INIT( __v__ ); }while(0)

#define Gpio1pin_InitDirectionInput    (__v__.bOutput=0u)
#define Gpio1pin_InitDirectionOutput   (__v__.bOutput=1u)
#define Gpio1pin_InitPullup(v)         (__v__.bPullup=(v))
#define Gpio1pin_InitVal(v)            (__v__.bInitVal=(v))



#define Gpio1pin_Get(p)	         p##_GET
#define Gpio1pin_Put(p,v)        p##_PUT(v)

#define PINRELOC_SET_EPFR(epfr,pos,width,value)    \
          ((epfr) = ((epfr) & ~(((1u<<(width))-1u)<<(pos)) | \
          ((value) << (pos))))

/******************************************************************************/
/* Inclusion of GPIO defines of user defined device                           */
/******************************************************************************/
// FM3
#if (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AF10X)  // MB9AF10X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N)
    #include "gpio_mb9af10xn.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_R)
    #include "gpio_mb9af10xr.h"
  #else
    #error Package for MB9AF10X not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AF11X)  // MB9AF11X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N)
    #include "gpio_mb9af11xn.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_M)
    #include "gpio_mb9af11xm.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_L)
    #include "gpio_mb9af11xl.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_K)
    #include "gpio_mb9af11xk.h"
  #else
    #error Package for MB9AF11X not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AF12X)  // MB9AF12X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_L)
    #include "gpio_mb9af12xl.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_K)
    #include "gpio_mb9af12xk.h"
  #else
    #error Package for MB9AF12X not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AF13X)  // MB9AF13X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N)
    #include "gpio_mb9af13xn.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_M)
    #include "gpio_mb9af13xm.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_L)
    #include "gpio_mb9af13xl.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_K)
    #include "gpio_mb9af13xk.h"
  #else
    #error Package for MB9AF13X not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AF14X)  // MB9AF14X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N)
    #include "gpio_mb9af14xn.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_M)
    #include "gpio_mb9af14xm.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_L)
    #include "gpio_mb9af14xl.h"
  #else
    #error Package for MB9AF14X not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AF15X)  // MB9AF15X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_R)
    #include "gpio_mb9af15xr.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N)
    #include "gpio_mb9af15xn.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_M)
    #include "gpio_mb9af15xm.h"
  #else
    #error Package for MB9AF15X not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AF31X)  // MB9AF31X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N)
    #include "gpio_mb9af31xn.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_M)
    #include "gpio_mb9af31xm.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_L)
    #include "gpio_mb9af31xl.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_K)
    #include "gpio_mb9af31xk.h"
  #else
    #error Package for MB9AF31X not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AF34X)  // MB9AF34X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N)
    #include "gpio_mb9af34xn.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_M)
    #include "gpio_mb9af34xm.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_L)
    #include "gpio_mb9af34xl.h"
  #else
    #error Package for MB9AF34X not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AF42X)  // MB9AF42X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_L)
    #include "gpio_mb9af42xl.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_K)
    #include "gpio_mb9af42xk.h"
  #else
    #error Package for MB9AF42X not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AFA3X)  // MB9AFA3X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N)
    #include "gpio_mb9afa3xn.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_M)
    #include "gpio_mb9afa3xm.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_L)
    #include "gpio_mb9afa3xl.h"
  #else
    #error Package for MB9AFA3X not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AFA4X) // MB9AFA4X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N)
    #include "gpio_mb9afa4xn.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_M)
    #include "gpio_mb9afa4xm.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_L)
    #include "gpio_mb9afa4xl.h"
  #else
    #error Package for MB9AFA4X not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9AFB4X) // MB9AFB4X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N)
    #include "gpio_mb9afb4xn.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_M)
    #include "gpio_mb9afb4xm.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_L)
    #include "gpio_mb9afb4xl.h"
  #else
    #error Package for MB9AFB4X not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF10X) // MB9BF10X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_R)
    #include "gpio_mb9bf10xr.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N)
    #include "gpio_mb9bf10xn.h"
  #else
    #error Package for MB9BF10X not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF11X) // MB9BF11X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_R)
    #include "gpio_mb9bf11xr.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N)
    #include "gpio_mb9bf11xn.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_S)
    #include "gpio_mb9bf11xs.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_T)
    #include "gpio_mb9bf11xt.h"
  #else
    #error Package for MB9BF11X not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF12X) // MB9BF12X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_T)
    #include "gpio_mb9bf12xt.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_S)
    #include "gpio_mb9bf12xs.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_M)
    #include "gpio_mb9bf12xm.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_L)
    #include "gpio_mb9bf12xl.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_K)
    #include "gpio_mb9bf12xk.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_J)
    #include "gpio_mb9bf12xj.h"          
  #else
    #error Package for MB9BF12X not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF21X) // MB9BF21X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_T)
    #include "gpio_mb9bf21xt.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_S)
    #include "gpio_mb9bf21xs.h"
  #else
    #error Package for MB9BF21X not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF30X) // MB9BF30X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_R)
    #include "gpio_mb9bf30xr.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N)
    #include "gpio_mb9bf30xn.h"
  #else
    #error Package for MB9BF30X not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF31X) // MB9BF31X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_T)
    #include "gpio_mb9bf31xt.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_S)
    #include "gpio_mb9bf31xs.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_R)
    #include "gpio_mb9bf31xr.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N)
    #include "gpio_mb9bf31xn.h"
  #else
    #error Package for MB9BF31X not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF32X) // MB9BF32X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_T)
    #include "gpio_mb9bf32xt.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_S)
    #include "gpio_mb9bf32xs.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_M)
    #include "gpio_mb9bf32xm.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_L)
    #include "gpio_mb9bf32xl.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_K)
    #include "gpio_mb9bf32xk.h"
  #else
    #error Package for MB9BF32X not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF40X) // MB9BF40X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_R)
    #include "gpio_mb9bf40xr.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N)
    #include "gpio_mb9bf40xn.h"
  #else
    #error Package for MB9BF40X not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF41X) // MB9BF41X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_T)
    #include "gpio_mb9bf41xt.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_S)
    #include "gpio_mb9bf41xs.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_R)
    #include "gpio_mb9bf41xr.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N)
    #include "gpio_mb9bf41xn.h"
  #else
    #error Package for MB9BF41X not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF42X) // MB9BF42X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_T)
    #include "gpio_mb9bf42xt.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_S)
    #include "gpio_mb9bf42xs.h"
  #else
    #error Package for MB9BF41X not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF50X) // MB9BF50X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_R)
    #include "gpio_mb9bf50xr.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N)
    #include "gpio_mb9bf50xn.h"
  #else
    #error Package for MB9BF50X not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF51X) // MB9BF51X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_T)
    #include "gpio_mb9bf51xt.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_S)
    #include "gpio_mb9bf51xs.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_R)
    #include "gpio_mb9bf51xr.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N)
    #include "gpio_mb9bf51xn.h"
  #else
    #error Package for MB9BF50X not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF52X) // MB9BF52X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_T)
    #include "gpio_mb9bf52xt.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_S)
    #include "gpio_mb9bf52xs.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_M)
    #include "gpio_mb9bf52xm.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_L)
    #include "gpio_mb9bf52xl.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_K)
    #include "gpio_mb9bf52xk.h"
  #else
    #error Package for MB9BF52X not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF61X) // MB9BF61X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_T)
    #include "gpio_mb9bf61xt.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_S)
    #include "gpio_mb9bf61xs.h"
  #else
    #error Package for MB9BF61X not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF61X) // MB9BFD1X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_T)
    #include "gpio_mb9bfd1xt.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_S)
    #include "gpio_mb9bfd1xs.h"
  #else
    #error Package for MB9BF61X not found!
  #endif
// FM4
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF16X) // MB9BF16X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_M)
    #include "gpio_mb9bf16xm.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N)
    #include "gpio_mb9bf16xn.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_R)
    #include "gpio_mb9bf16xr.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_K)
    #include "gpio_mb9bf16xk.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_L)
    #include "gpio_mb9bf16xl.h"
  #else
    #error Package for MB9BF16X not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF36X) // MB9BF36X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_M)
    #include "gpio_mb9bf36xm.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N)
    #include "gpio_mb9bf36xn.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_R)
    #include "gpio_mb9bf36xr.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_K)
    #include "gpio_mb9bf36xk.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_L)
    #include "gpio_mb9bf36xl.h"
  #else
    #error Package for MB9BF36X not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF46X) // MB9BF46X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_M)
    #include "gpio_mb9bf46xm.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N)
    #include "gpio_mb9bf46xn.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_R)
    #include "gpio_mb9bf46xr.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_K)
    #include "gpio_mb9bf46xk.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_L)
    #include "gpio_mb9bf46xl.h"
  #else
    #error Package for MB9BF46X not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_MB9BF56X) // MB9BF56X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_M)
    #include "gpio_mb9bf56xm.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_N)
    #include "gpio_mb9bf56xn.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_R)
    #include "gpio_mb9bf56xr.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_K)
    #include "gpio_mb9bf56xk.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_MB_L)
    #include "gpio_mb9bf56xl.h"
  #else
    #error Package for MB9BF56X not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2C1X)  // S6E2C1X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_H)
    #include "gpio_s6e2c1xh.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_J)
    #include "gpio_s6e2c1xj.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_L)
    #include "gpio_s6e2c1xl.h"
  #else
    #error Package for S6E2C1X not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2C2X)  // S6E2C2X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_H)
    #include "gpio_s6e2c2xh.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_J)
    #include "gpio_s6e2c2xj.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_L)
    #include "gpio_s6e2c2xl.h"
  #else
    #error Package for S6E2C2X not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2C3X)  // S6E2C3X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_H)
    #include "gpio_s6e2c3xh.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_J)
    #include "gpio_s6e2c3xj.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_L)
    #include "gpio_s6e2c3xl.h"
  #else
    #error Package for S6E2C3X not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2C4X)  // S6E2C4X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_H)
    #include "gpio_s6e2c4xh.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_J)
    #include "gpio_s6e2c4xj.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_L)
    #include "gpio_s6e2c4xl.h"
  #else
    #error Package for S6E2C4X not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2C5X)  // S6E2C5X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_H)
    #include "gpio_s6e2c5xh.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_J)
    #include "gpio_s6e2c5xj.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_L)
    #include "gpio_s6e2c5xl.h"
  #else
    #error Package for S6E2C5X not found!
  #endif            
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2CCX)  // S6E2CCX
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_H)
    #include "gpio_s6e2ccxh.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_J)
    #include "gpio_s6e2ccxj.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_L)
    #include "gpio_s6e2ccxl.h"
  #else
    #error Package for S6E2CCX not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2D3X)  // S6E2D3
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_J)
    #include "gpio_s6e2d3xj.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_G)  
    #include "gpio_s6e2d3xg.h"            
  #else
    #error Package for S6E2D3X not found!        
  #endif            
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2D5X)  // S6E2D5
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_J)
    #include "gpio_s6e2d5xj.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_G)  
    #include "gpio_s6e2d5xg.h"            
  #else
    #error Package for S6E2D5X not found!        
  #endif            
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2DFX)  // S6E2DF
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_J)
    #include "gpio_s6e2dfxj.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_G)  
    #include "gpio_s6e2dfxg.h"            
  #else
    #error Package for S6E2DFX not found!        
  #endif             
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2DHX)  // S6E2DH
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_J)
    #include "gpio_s6e2dhxj.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_G)  
    #include "gpio_s6e2dhxg.h"            
  #else
    #error Package for S6E2DHX not found!        
  #endif          
// FM0+
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E1A1X) // S6E1A1X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_B)
    #include "gpio_s6e1a1xb.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_C)
    #include "gpio_s6e1a1xc.h"
  #else
    #error Package for S6E1A1 series not found!
  #endif     
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E100X) // S6E100X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_F)
    #include "gpio_s6e100xf.h" 
  #else
    #error Package for S6E100 series not found!          
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E1B3X) // S6E1B3X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_E)
    #include "gpio_s6e1b3xe.h" 
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_F)
    #include "gpio_s6e1b3xf.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_G)
    #include "gpio_s6e1b3xg.h"
  #else
    #error Package for S6E1B3 series not found!            
  #endif            
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E1B7X) // S6E1B7X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_E)
    #include "gpio_s6e1b7xe.h" 
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_F)
    #include "gpio_s6e1b7xf.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_G)
    #include "gpio_s6e1b7xg.h"
  #else
    #error Package for S6E1B7 series not found!            
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E1B8X) // S6E1B8X
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_E)
    #include "gpio_s6e1b8xe.h" 
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_F)
    #include "gpio_s6e1b8xf.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_G)
    #include "gpio_s6e1b8xg.h"
  #else
    #error Package for S6E1B8 series not found!            
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2G2X) // S6E2G2X 
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_H)
    #include "gpio_s6e2g2xh.h" 
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_J)
    #include "gpio_s6e2g2xj.h"
  #else
    #error Package for S6E2G2 series not found!            
  #endif             
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2G3X) // S6E2G3X 
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_H)
    #include "gpio_s6e2g3xh.h" 
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_J)
    #include "gpio_s6e2g3xj.h"
  #else
    #error Package for S6E2G3 series not found!            
  #endif            
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2GKX) // S6E2GKX 
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_H)
    #include "gpio_s6e2gkxh.h" 
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_J)
    #include "gpio_s6e2gkxj.h"
  #else
    #error Package for S6E2GK series not found!            
  #endif            
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2GHX) // S6E2GHX 
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_H)
    #include "gpio_s6e2ghxh.h" 
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_J)
    #include "gpio_s6e2ghxj.h"
  #else
    #error Package for S6E2GH series not found!            
  #endif            
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2GMX) // S6E2GMX 
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_H)
    #include "gpio_s6e2gmxh.h" 
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_J)
    #include "gpio_s6e2gmxj.h"
  #else
    #error Package for S6E2GM series not found!            
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2H1X) // S6E2H1X 
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_E)
    #include "gpio_s6e2h1xe.h" 
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_F)
    #include "gpio_s6e2h1xf.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_G)
    #include "gpio_s6e2h1xg.h"            
  #else
    #error Package for S6E2H1 series not found!            
  #endif             
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2H4X) // S6E2H4X 
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_E)
    #include "gpio_s6e2h4xe.h" 
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_F)
    #include "gpio_s6e2h4xf.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_G)
    #include "gpio_s6e2h4xg.h"            
  #else
    #error Package for S6E2H4 series not found!            
  #endif            
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2HEX) // S6E2HEX 
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_E)
    #include "gpio_s6e2hexe.h" 
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_F)
    #include "gpio_s6e2hexf.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_G)
    #include "gpio_s6e2hexg.h"            
  #else
    #error Package for S6E2HE series not found!            
  #endif            
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E2HGX) // S6E2HGX 
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_E)
    #include "gpio_s6e2hgxe.h" 
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_F)
    #include "gpio_s6e2hgxf.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_G)
    #include "gpio_s6e2hgxg.h"            
  #else
    #error Package for S6E2HG series not found!            
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E1C1X) // S6E1C1X 
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_B)
    #include "gpio_s6e1c1xb.h" 
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_C)
    #include "gpio_s6e1c1xc.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_D)
    #include "gpio_s6e1c1xd.h"            
  #else
    #error Package for S6E1C1 series not found!            
  #endif            
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E1C3X) // S6E1C3X 
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_B)
    #include "gpio_s6e1c3xb.h" 
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_C)
    #include "gpio_s6e1c3xc.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_D)
    #include "gpio_s6e1c3xd.h"            
  #else
    #error Package for S6E1C3 series not found!            
  #endif             
#else            
  #error Device not found!
#endif

/******************************************************************************/
/* Types                                                                      */
/******************************************************************************/

typedef struct stc_gpio1pin_init
{
    boolean_t bOutput;
    boolean_t bInitVal;
    boolean_t bPullup;
} stc_gpio1pin_init_t;


//@} // GpioGroup

#ifdef __cplusplus
}
#endif

#endif /* __GPIO_H__ */

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
