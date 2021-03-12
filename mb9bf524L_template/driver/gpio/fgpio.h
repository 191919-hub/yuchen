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
/** \file fgpio.h
 **
 ** Headerfile for Fast GPIO functions
 **
 ** History:
 **   - 2014-01-10  1.0  EZh    First version.
 **
 ******************************************************************************/

#ifndef __FGPIO_H__
#define __FGPIO_H__

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
 ** \defgroup FGpioGroup Fast GPIO Defintions (Fast GPIO)
 **
 ** Definitions of Fast GPIO and resource pin relocation
 **
 ** \attention
 **            - before using the Fast GPIO output, FGpio_EnableOutput() has
 **              to be called.
 ******************************************************************************/
//@{

/******************************************************************************/
/* \brief Fast GPIO intialization structure                                   */
/******************************************************************************/

typedef struct stc_fgpio1pin_init
{
    boolean_t bOutput;
    boolean_t bInitVal;
    boolean_t bPullup;
} stc_fgpio1pin_init_t;


/**
 ******************************************************************************
 ** \brief GPIO port list
 ******************************************************************************/
typedef enum en_fgpio_port
{
    FGpioPort0 = 0u,   ///< Fast GPIO port 0
    FGpioPort1 = 1u,   ///< Fast GPIO port 1
    FGpioPort2 = 2u,   ///< Fast GPIO port 2
    FGpioPort3 = 3u,   ///< Fast GPIO port 3
    FGpioPort4 = 4u,   ///< Fast GPIO port 4
    FGpioPort5 = 5u,   ///< Fast GPIO port 5
    FGpioPort6 = 6u,   ///< Fast GPIO port 6
    FGpioPort7 = 7u,   ///< Fast GPIO port 7
    FGpioPort8 = 8u,   ///< Fast GPIO port 8
    FGpioPort9 = 9u,   ///< Fast GPIO port 9
    FGpioPortA = 10u,  ///< Fast GPIO port 10
    FGpioPortB = 11u,  ///< Fast GPIO port 11
    FGpioPortC = 12u,  ///< Fast GPIO port 12
    FGpioPortD = 13u,  ///< Fast GPIO port 13
    FGpioPortE = 14u,  ///< Fast GPIO port 14
    FGpioPortF = 15u,  ///< Fast GPIO port 15

}en_fgpio_port_t;

/******************************************************************************/
/* Defines                                                                    */
/******************************************************************************/

#define FGpio_EnableOutput(port, pins)   do {uint32_t addr; \
                                            addr = (uint32_t)&FM_GPIO->FPOER0 + (uint32_t)port*4u; \
                                            *(uint16_t*)(addr) = pins;  \
                                            }while(0);
#define FGpio_DisableOutput(port)        do {uint32_t addr; \
                                            addr = (uint32_t)&FM_GPIO->FPOER0 + (uint32_t)port*4u; \
                                            *(uint16_t*)(addr) = 0x0000u;  \
                                            }while(0);

#define FGpio1pin_InitIn(p,settings)    do{ stc_fgpio1pin_init_t __v__;\
                                         __v__.bPullup=0u;__v__.bInitVal=0u;\
                                         (settings);\
                                         p##_INITIN(__v__); }while(0)

#define FGpio1pin_InitOut(p,settings)   do{ stc_fgpio1pin_init_t __v__;\
                                         __v__.bPullup=0u;__v__.bInitVal=0u;\
                                         (settings);\
                                         p##_INITOUT(__v__); }while(0)

#define FGpio1pin_Init(p,settings)      do{ stc_fgpio1pin_init_t __v__;__v__.bOutput=0u;\
                                         __v__.bPullup=0u;__v__.bInitVal=0u;\
                                         (settings);\
                                         p##_INIT( __v__ ); }while(0)

#define FGpio1pin_InitDirectionInput    (__v__.bOutput=0u)
#define FGpio1pin_InitDirectionOutput   (__v__.bOutput=1u)
#define FGpio1pin_InitPullup(v)         (__v__.bPullup=(v))
#define FGpio1pin_InitVal(v)            (__v__.bInitVal=(v))



#define FGpio1pin_Get(p)	         p##_GET
#define FGpio1pin_Put(p,v)           p##_PUT(v)


/******************************************************************************/
/* Inclusion of GPIO defines of user defined device                           */
/******************************************************************************/

#if (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E1A1X)
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_B)
    #include "gpio_s6e1a1xb.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_C)
    #include "gpio_s6e1a1xc.h"
  #else
    #error Package for S6E1A1 series not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E100X)
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_F)
    #include "gpio_s6e100xf.h"
  #else
    #error Package for S6E1A1 series not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E1B3X)
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_E)
    #include "gpio_s6e1b3xe.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_F)
    #include "gpio_s6e1b3xf.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_G)
    #include "gpio_s6e1b3xg.h"                                           
  #else
    #error Package for S6E1B3 series not found!
  #endif                                            
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E1B7X)
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_E)
    #include "gpio_s6e1b7xe.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_F)
    #include "gpio_s6e1b7xf.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_G)
    #include "gpio_s6e1b7xg.h"                                           
  #else
    #error Package for S6E1B7 series not found!
  #endif 
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E1B8X)
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_E)
    #include "gpio_s6e1b8xe.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_F)
    #include "gpio_s6e1b8xf.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_G)
    #include "gpio_s6e1b8xg.h"                                           
  #else
    #error Package for S6E1B8 series not found!
  #endif
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E1C1X)
  #if (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_B)
    #include "gpio_s6e1c1xb.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_C)
    #include "gpio_s6e1c1xc.h"
  #elif (PDL_MCU_PACKAGE == PDL_DEVICE_PACKAGE_S6_D)
    #include "gpio_s6e1c1xd.h"                                           
  #else
    #error Package for S6E1C1 series not found!
  #endif                                           
#elif (PDL_MCU_SERIES == PDL_DEVICE_SERIES_S6E1C3X)
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




//@} // FGpioGroup

#ifdef __cplusplus
}
#endif

#endif /* __GPIO_H__ */

/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
