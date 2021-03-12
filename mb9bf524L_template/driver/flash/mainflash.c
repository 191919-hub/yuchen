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
/** \file mainflash.c
 **
 ** A detailed description is available at 
 ** @link MainFlashGroup Main Flash Module description @endlink
 **
 ** History:
 **   - 2015-01-20 1.0  KXi First version for FM Universal PDL.
 **   - 2015-06-26 1.1  EZh Add the support to Dual Flash mode of FM4 TYPE3 
 **                         products.
 **
 ******************************************************************************/
/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "mainflash.h"

#if defined(PDL_PERIPHERAL_MAIN_FLASH_ACTIVE) 
 
#ifdef __ICCARM__
    #pragma section = ".flash_ram_code"
#endif
/**
 ******************************************************************************
 ** \addtogroup MainFlashGroup
 ******************************************************************************/
//@{   
   
/******************************************************************************/
/* Local pre-processor symbols/macros ('#define')                             */
/******************************************************************************/
/*!
 ******************************************************************************
 ** \brief Read a half word data from Flash
 ** \param addr Pointer to read data address
 ******************************************************************************
 */
#define Flash_Read(addr)        *(volatile uint16_t*)((uint32_t)(addr))

/*!
 ******************************************************************************
 ** \brief Wirte a half word data into Flash
 ** \param addr Pointer to read data address
 ** \param data Write data
 ******************************************************************************
 */
#define Flash_Write(addr, data) *(volatile uint16_t*)((uint32_t)(addr)) = (uint16_t)(data)
/******************************************************************************/
/* Global variable definitions (declared in header file with 'extern')        */
/******************************************************************************/


/******************************************************************************/
/* Local type definitions ('typedef')                                         */
/******************************************************************************/

/******************************************************************************/
/* Local function prototypes ('static')                                       */
/******************************************************************************/
/*
#if defined ( __ICCARM__ )
__ramfunc 
#endif
static en_mflash_toggle_t MFlash_CheckToggle( uint16_t* pAddr );

#if defined ( __ICCARM__ )
__ramfunc 
#endif */

#ifdef __ICCARM__
  __ramfunc
#elif  __CC_ARM
  __attribute__ ((section (".ramfunc")))
#else
  #error Please check compiler and linker settings for RAM code
#endif

static void MFlash_ReadResetCmd(uint16_t* pResetSecAddr);
/******************************************************************************/
/* Local variable definitions ('static')                                      */
/******************************************************************************/

/*!
 ******************************************************************************
 ** \brief Issue read/reset command
 **
 ** \param pu16ResetSecAddr address of sector  
 **
 ** \return None
 **
 ******************************************************************************
 */
/*
#if defined ( __ICCARM__ )
__ramfunc 
#endif */
#ifdef __ICCARM__
  __ramfunc
#elif  __CC_ARM
  __attribute__ ((section (".ramfunc")))
#else
  #error Please check compiler and linker settings for RAM code
#endif
static void MFlash_ReadResetCmd(uint16_t* pu16ResetSecAddr)
{
    uint8_t  u8Dummy;

    /*  issue read/reset command    */
    Flash_Write(0x0000u, 0xF0u) ;
    u8Dummy = Flash_Read(pu16ResetSecAddr) ;
    if(u8Dummy)  /* avoid warning */  
        ;
    return ;
}

/******************************************************************************/
/* Function implementation - global ('extern') and local ('static')           */
/******************************************************************************/


#ifdef __ICCARM__
  __ramfunc
#elif  __CC_ARM
  __attribute__ ((section (".ramfunc")))
#else
  #error Please check compiler and linker settings for RAM code
#endif
static en_mflash_toggle_t MFlash_CheckToggle( uint16_t* pu16Addr )
{
    uint16_t   u16SequenceFlag1, u16SequenceFlag2 ;  /*  hardware sequence flag */
    en_mflash_toggle_t   emRetValue  = MFLASH_CHK_TOGG_NORMAL ;

    /* set hardware sequence flag */
    u16SequenceFlag1 = Flash_Read(pu16Addr) ;
    u16SequenceFlag2 = Flash_Read(pu16Addr) ;
    /*  if automatic algorithm is executing */
    while(MFLASH_CHK_TOGG_MASK == (( u16SequenceFlag1 ^ u16SequenceFlag2) & MFLASH_CHK_TOGG_MASK))
    {
        /*  if exceeds the timing limit */
        if(( u16SequenceFlag1 & MFLASH_CHK_TLOV_MASK) == MFLASH_CHK_TLOV_MASK)
        {
            /* set hardware sequence flag */
            u16SequenceFlag1 = Flash_Read(pu16Addr) ;
            u16SequenceFlag2 = Flash_Read(pu16Addr) ;

            /*  if automatic algorithm is executing */
            if((( u16SequenceFlag1 ^ u16SequenceFlag2) & MFLASH_CHK_TOGG_MASK) == MFLASH_CHK_TOGG_MASK)
            {
                /*  abnormally complete */
                emRetValue  = MFLASH_CHK_TOGG_ABNORMAL ;

                break;
            }
        }

        /* set hardware sequence flag */
        u16SequenceFlag1 = Flash_Read(pu16Addr) ;
        u16SequenceFlag2 = Flash_Read(pu16Addr) ;
    }

    return emRetValue ;
}
/*!
 ******************************************************************************
 ** \brief Flash chip erase       
 **
 ** \param bCrRemain Protect CR data or not
 ** \arg TRUE  Remain CR data after chip erase
 ** \arg FALSE CR data will be lost after chip erase
 **
 ** \return Operation status
 ** \retval Ok                     Chip erase successfully done.
 ** \retval ErrorInvalidParameter  If one of following conditions are met:
 **                                - automatic algorithm of flash memory execution abnormally
 **
 ******************************************************************************
 */
/*
#if defined ( __ICCARM__ )
__ramfunc 
#endif*/
#ifdef __ICCARM__
  __ramfunc
#elif  __CC_ARM
  __attribute__ ((section (".ramfunc")))
#else
  #error Please check compiler and linker settings for RAM code
#endif
en_result_t MFlash_ChipErase(boolean_t bCrRemain)
{
    uint8_t  u8Cnt;
    uint32_t u32CrData, u32CrAddr;
    uint16_t u16WriteData;
    en_result_t emRetValue = Ok;
#if (PDL_MCU_CORE != PDL_FM0P_CORE)   
    uint8_t  u8Dummy;
#endif
    
    __disable_irq(); 
    
#if (PDL_MCU_CORE != PDL_FM0P_CORE)
    FM_FLASH_IF->FASZR = 0x01u;                                 ///< set 16 bit read/write (CPU programming mode)
    u8Dummy = FM_FLASH_IF->FASZR;     
    if(u8Dummy);  
#endif
    
    /* Save CR value */
    if (bCrRemain == TRUE)
    {
        u32CrData = *(uint32_t*)CR_DATA_ADDR;
    }

    Flash_Write(MFLASH_CODE1, 0x00AAu);
    Flash_Write(MFLASH_CODE2, 0x0055u);
    Flash_Write(MFLASH_CODE1, 0x0080u);
    Flash_Write(MFLASH_CODE1, 0x00AAu);
    Flash_Write(MFLASH_CODE2, 0x0055u);
    Flash_Write(MFLASH_CODE1, 0x0010u);

     /*  if execution result of the automatic algorithm of flash memory is abnormally completed  */
    if( MFLASH_CHK_TOGG_ABNORMAL == MFlash_CheckToggle((uint16_t*)0u))
    {
        /*  sending the read/reset command to the reset sector  */
        MFlash_ReadResetCmd((uint16_t*)0u) ;

        /*  return flash operation abnormally   */
        emRetValue  = ErrorInvalidParameter ;
    }
    
    /* restore CR data in Flash */
    if (bCrRemain == TRUE)
    {
        for(u8Cnt=2,u16WriteData=(uint16_t)u32CrData, u32CrAddr = CR_DATA_ADDR;u8Cnt;u8Cnt--)
        {
            /*  issue write command   */
            Flash_Write(MFLASH_CODE1, 0x00AAu) ;
            Flash_Write(MFLASH_CODE2, 0x0055u) ;
            Flash_Write(MFLASH_CODE1, 0x00A0u) ;
            Flash_Write((uint16_t*)u32CrAddr, u16WriteData);
            
            /*  execution result of the automatic algorithm of flash memory is abnormally complete or verify error  */
            if(( MFLASH_CHK_TOGG_ABNORMAL == MFlash_CheckToggle((uint16_t*)u32CrAddr)) ||
              ( Flash_Read((uint16_t*)u32CrAddr) != u16WriteData))
            {
                /*  issue read/reset command to reset sector    */
                MFlash_ReadResetCmd((uint16_t*)u32CrAddr) ;
    
                /*  return flash operation abnormally   */
                emRetValue  = ErrorInvalidParameter ;
            }      
            u16WriteData = (uint16_t)(u32CrData>>16);
            u32CrAddr += 2;
        }
    }  
    
#if (PDL_MCU_CORE != PDL_FM0P_CORE)    
    FM_FLASH_IF->FASZR = 0x02u;                                 ///< set 32 bit read(CPU ROM mode)        
    u8Dummy = FM_FLASH_IF->FASZR;     
    if(u8Dummy);
#endif    
    
    /* Recover IRQ  */
    __enable_irq();
    
    return emRetValue;
}

/*!
 ******************************************************************************
 ** \brief Flash sector erase       
 **
 ** \param pu16SecAddr address of flash sector  
 **
 ** \return Operation status
 ** \retval Ok                     sector erase successfully done.
 ** \retval ErrorInvalidParameter  If one of following conditions are met:
 **                                - automatic algorithm of flash memory execution abnormally
 **
 ******************************************************************************
 */
/*
#if defined ( __ICCARM__ )
__ramfunc 
#endif */
#ifdef __ICCARM__
  __ramfunc
#elif  __CC_ARM
  __attribute__ ((section (".ramfunc")))
#else
  #error Please check compiler and linker settings for RAM code
#endif
en_result_t MFlash_SectorErase(uint16_t* pu16SecAddr)
{
#if (PDL_MCU_CORE != PDL_FM0P_CORE)   
    uint8_t  u8Dummy;
#endif
    en_result_t emRetValue = Ok;
    
    __disable_irq(); 
#if (PDL_MCU_CORE != PDL_FM0P_CORE)
    FM_FLASH_IF->FASZR = 0x01u;                                 ///< set 16 bit read/write (CPU programming mode)
    u8Dummy = FM_FLASH_IF->FASZR;     
    if(u8Dummy);  
#endif
    /*
    Flash_Write(MFLASH_CODE1 | ((uint32_t)pu16SecAddr & 0xFFFFF000), 0x00AAu);
    Flash_Write(MFLASH_CODE2 | ((uint32_t)pu16SecAddr & 0xFFFFF000), 0x0055u);
    Flash_Write(MFLASH_CODE1 | ((uint32_t)pu16SecAddr & 0xFFFFF000), 0x0080u);
    Flash_Write(MFLASH_CODE1 | ((uint32_t)pu16SecAddr & 0xFFFFF000), 0x00AAu);
    Flash_Write(MFLASH_CODE2 | ((uint32_t)pu16SecAddr & 0xFFFFF000), 0x0055u);
    Flash_Write(pu16SecAddr, 0x0030u);*/
    
    Flash_Write(0xaa8 | ((uint32_t)pu16SecAddr & 0xFFFFF000), 0x00AAu);
    Flash_Write(0x554 | ((uint32_t)pu16SecAddr & 0xFFFFF000), 0x0055u);
    Flash_Write(0xaa8 | ((uint32_t)pu16SecAddr & 0xFFFFF000), 0x0080u);
    Flash_Write(0xaa8 | ((uint32_t)pu16SecAddr & 0xFFFFF000), 0x00AAu);
    Flash_Write(0x554 | ((uint32_t)pu16SecAddr & 0xFFFFF000), 0x0055u);
    Flash_Write(pu16SecAddr, 0x0030u);

     /*  if execution result of the automatic algorithm of flash memory is abnormally completed  */
    if( MFLASH_CHK_TOGG_ABNORMAL == MFlash_CheckToggle(pu16SecAddr))
    {
        /*  sending the read/reset command to the reset sector  */
        MFlash_ReadResetCmd(pu16SecAddr) ;

        /*  return flash operation abnormally   */
        emRetValue  = ErrorInvalidParameter ;
    }
#if (PDL_MCU_CORE != PDL_FM0P_CORE)    
    FM_FLASH_IF->FASZR = 0x02u;                                 ///< set 32 bit read(CPU ROM mode)        
    u8Dummy = FM_FLASH_IF->FASZR;     
    if(u8Dummy);
#endif
    /* Recover IRQ  */
    __enable_irq();
    return emRetValue;
}

/*!
 ******************************************************************************
 ** \brief Flash word write with ECC  
 **
 ** \param pu32WriteAddr address of flash data
 ** \param pu32WriteData pointer to write data
 ** \param u32Size data size, 1 indicates 1 32-bit data
 ** \param bVerifyAndEccCheck Whether to verify and check ECC
 **
 ** \return Operation status
 ** \retval Ok                     Flash written successfully done.
 ** \retval ErrorInvalidParameter  If one of following conditions are met:
 **                                - automatic algorithm of flash memory execution abnormally
 **                                - ECC abnormally
 **
 ******************************************************************************
 */
/*
#if defined ( __ICCARM__ )
__ramfunc 
#endif */
#ifdef __ICCARM__
  __ramfunc
#elif  __CC_ARM
  __attribute__ ((section (".ramfunc")))
#else
  #error Please check compiler and linker settings for RAM code
#endif
en_result_t MFlash_WriteData32Bit(uint32_t*  pu32WriteAddr,\
                                  uint32_t* pu32WriteData, \
                                  uint32_t u32Size, \
                                  boolean_t bVerifyAndEccCheck)
{
    uint16_t *pAddr, *pData;
#if (PDL_MCU_CORE != PDL_FM0P_CORE)   
    uint8_t  u8Dummy;
#endif
    en_result_t emRetValue  = Ok ;
    uint32_t   u32Cnt;
    pAddr = (uint16_t*)pu32WriteAddr;
    pData = (uint16_t*)pu32WriteData;
    __disable_irq(); 
#if (PDL_MCU_CORE != PDL_FM0P_CORE)
    FM_FLASH_IF->FASZR = 0x01u;                                 ///< set 16 bit read/write (CPU programming mode)
    u8Dummy = FM_FLASH_IF->FASZR;     
    if(u8Dummy);  
#endif
    for(u32Cnt=u32Size*sizeof(uint16_t);u32Cnt;u32Cnt--)
    {
        /*  issue write command   */
        Flash_Write(MFLASH_CODE1 | ((uint32_t)pAddr & 0xFFFFF000), 0x00AAu) ;
        Flash_Write(MFLASH_CODE2 | ((uint32_t)pAddr & 0xFFFFF000), 0x0055u) ;
        Flash_Write(MFLASH_CODE1 | ((uint32_t)pAddr & 0xFFFFF000), 0x00A0u) ;
        Flash_Write((uint32_t)pAddr, (uint16_t)*pData);
        /*  execution result of the automatic algorithm of flash memory is abnormally complete or verify error  */
        if(( MFLASH_CHK_TOGG_ABNORMAL == MFlash_CheckToggle(pAddr)) ||
          ( Flash_Read((uint32_t)pAddr) != (uint16_t)*pData))
        {
            /*  issue read/reset command to reset sector    */
            MFlash_ReadResetCmd(pAddr) ;

            /*  return flash operation abnormally   */
            emRetValue  = ErrorInvalidParameter ;
        }      
        /* Prepare next h-word write */
        pAddr++;
        pData++; 
    }
#if (PDL_MCU_CORE != PDL_FM0P_CORE)    
    FM_FLASH_IF->FASZR = 0x02u;                                 ///< set 32 bit read(CPU ROM mode)        
    u8Dummy = FM_FLASH_IF->FASZR;     
    if(u8Dummy);
#endif
    /* Recover IRQ  */
    __enable_irq();

#if (PDL_MCU_CORE != PDL_FM0P_CORE)    
    if(MFLASH_ECC_ABNORMAL ==  bVerifyAndEccCheck && 1 == FM_FLASH_IF->FSTR_f.ERR)
    {
      emRetValue = ErrorInvalidParameter;
      bVerifyAndEccCheck = MFLASH_ECC_NORMAL;                     ///< clear ECC error bit
    }
#endif
    return emRetValue ;
}
/*!
 ******************************************************************************
 ** \brief Flash half word write with ECC  
 **
 ** \param pu16WriteAddr address of flash data
 ** \param pu16WriteData pointer to write data
 ** \param u32Size data size, 1 indicates 1 16-bit data
 **
 ** \return Operation status
 ** \retval MFLASH_RET_OK
 ** \retval MFLASH_RET_ABNORMAL
 ** \retval MFLASH_RET_INVALID_PARA
 **
 ******************************************************************************
 */
/*
#if defined ( __ICCARM__ )
__ramfunc 
#endif */
#ifdef __ICCARM__
  __ramfunc
#elif  __CC_ARM
  __attribute__ ((section (".ramfunc")))
#else
  #error Please check compiler and linker settings for RAM code
#endif
en_result_t MFlash_WriteData16Bit(uint16_t*  pu16WriteAddr, \
                                  uint16_t* pu16WriteData, \
                                  uint32_t u32Size)
{
#if (PDL_MCU_CORE != PDL_FM0P_CORE)   
    uint8_t  u8Dummy;
#endif
    en_result_t emRetValue  = Ok ;
    uint32_t   u32Cnt;
    
    __disable_irq(); 
#if (PDL_MCU_CORE != PDL_FM0P_CORE)
    FM_FLASH_IF->FASZR = 0x01u;                                 ///< set 16 bit read/write (CPU programming mode)
    u8Dummy = FM_FLASH_IF->FASZR;     
    if(u8Dummy);  
#endif
    for(u32Cnt=u32Size;u32Cnt;u32Cnt--)
    {
        /*  issue write command   */
        Flash_Write(MFLASH_CODE1 | ((uint32_t)pu16WriteAddr & 0xFFFFF000), 0x00AAu) ;
        Flash_Write(MFLASH_CODE2 | ((uint32_t)pu16WriteAddr & 0xFFFFF000), 0x0055u) ;
        Flash_Write(MFLASH_CODE1 | ((uint32_t)pu16WriteAddr & 0xFFFFF000), 0x00A0u) ;
        Flash_Write(pu16WriteAddr, (uint16_t)*pu16WriteData);
        
        /*  execution result of the automatic algorithm of flash memory is abnormally complete or verify error  */
        if(( MFLASH_CHK_TOGG_ABNORMAL == MFlash_CheckToggle(pu16WriteAddr)) ||
          ( Flash_Read(pu16WriteAddr) != *pu16WriteData))
        {
            /*  issue read/reset command to reset sector    */
            MFlash_ReadResetCmd(pu16WriteAddr) ;

            /*  return flash operation abnormally   */
            emRetValue  = ErrorInvalidParameter ;
        }      
        /* Prepare next h-word write */
        pu16WriteAddr++;
        pu16WriteData++; 
    }
#if (PDL_MCU_CORE != PDL_FM0P_CORE)    
    FM_FLASH_IF->FASZR = 0x02u;                                 ///< set 32 bit read(CPU ROM mode)        
    u8Dummy = FM_FLASH_IF->FASZR;     
    if(u8Dummy);
#endif
    /* Recover IRQ  */
    __enable_irq();
    return emRetValue ;
}
/*
#if defined ( __ICCARM__ )
__ramfunc 
#endif */
#ifdef __ICCARM__
  __ramfunc
#elif  __CC_ARM
  __attribute__ ((section (".ramfunc")))
#else
  #error Please check compiler and linker settings for RAM code
#endif
/*!
 ******************************************************************************
 ** \brief Flash half word write with ECC  
 **
 ** \param pu16WriteAddr address of flash data
 ** \param pu16WriteData pointer to write data
 ** \param u32Size data size, 1 indicates 1 16-bit data
 **
 ** \return Operation status
 ** \retval MFLASH_RET_OK
 ** \retval MFLASH_RET_ABNORMAL
 ** \retval MFLASH_RET_INVALID_PARA
 **
 ******************************************************************************
 */
en_result_t MFlash_WriteData16Bit_Fm0Type3CrSecureArea(uint16_t*  pu16WriteAddr, \
                                  uint16_t* pu16WriteData, \
                                  uint32_t u32Size)
{
#if (PDL_MCU_CORE != PDL_FM0P_CORE)   
    uint8_t  u8Dummy;
#endif
    en_result_t emRetValue  = Ok ;
    uint32_t   u32Cnt;
    
    __disable_irq(); 
#if (PDL_MCU_CORE != PDL_FM0P_CORE)
    FM_FLASH_IF->FASZR = 0x01u;                                 ///< set 16 bit read/write (CPU programming mode)
    u8Dummy = FM_FLASH_IF->FASZR;     
    if(u8Dummy);  
#endif
    for(u32Cnt=u32Size;u32Cnt;u32Cnt--)
    {
        /*  issue write command   */
        Flash_Write(MFLASH_CODE1 , 0x00AAu) ;
        Flash_Write(MFLASH_CODE2 , 0x0055u) ;
        Flash_Write(MFLASH_CODE1 , 0x00A0u) ;
        Flash_Write(pu16WriteAddr, (uint16_t)*pu16WriteData);
        
        /*  execution result of the automatic algorithm of flash memory is abnormally complete or verify error  */
        if(( MFLASH_CHK_TOGG_ABNORMAL == MFlash_CheckToggle(pu16WriteAddr)) ||
          ( Flash_Read(pu16WriteAddr) != *pu16WriteData))
        {
            /*  issue read/reset command to reset sector    */
            MFlash_ReadResetCmd(pu16WriteAddr) ;

            /*  return flash operation abnormally   */
            emRetValue  = ErrorInvalidParameter ;
        }      
        /* Prepare next h-word write */
        pu16WriteAddr++;
        pu16WriteData++; 
    }
#if (PDL_MCU_CORE != PDL_FM0P_CORE)    
    FM_FLASH_IF->FASZR = 0x02u;                                 ///< set 32 bit read(CPU ROM mode)        
    u8Dummy = FM_FLASH_IF->FASZR;     
    if(u8Dummy);
#endif
    /* Recover IRQ  */
    __enable_irq();
    return emRetValue ;
}
/*!
 ******************************************************************************
 ** \brief automatic algorithm of flash memory execution 
 **
 ** \param pu16Addr address of flash data
 **
 ** \return Operation status
 ** \retval MFLASH_RET_OK
 ** \retval MFLASH_RET_ABNORMAL
 ** \retval MFLASH_RET_INVALID_PARA
 **
 ******************************************************************************
 */
/*
#if defined ( __ICCARM__ )
__ramfunc 
#endif */


#if (PDL_MCU_TYPE == PDL_FM4_TYPE3)
/*!
 ******************************************************************************
 ** \brief Issue read/reset command
 **
 ** \param pu16ResetSecAddr address of sector  
 **
 ** \return None
 **
 ******************************************************************************
 */
static void MFlash_DualReadResetCmd(uint16_t* pu16ResetSecAddr)
{
    uint8_t  u8Dummy;

    /*  issue read/reset command    */
    Flash_Write(0x0000u, 0xF0u) ;
    u8Dummy = Flash_Read(pu16ResetSecAddr) ;
    if(u8Dummy)  /* avoid warning */  
        ;
    return ;
}
#ifdef __ICCARM__
  __ramfunc
#elif  __CC_ARM
  __attribute__ ((section (".ramfunc")))
#else
  #error Please check compiler and linker settings for RAM code
#endif
/*!
 ******************************************************************************
 ** \brief automatic algorithm of flash memory execution 
 **
 ** \param pu16Addr address of flash data
 **
 ** \return Operation status
 ** \retval MFLASH_RET_OK
 ** \retval MFLASH_RET_ABNORMAL
 ** \retval MFLASH_RET_INVALID_PARA
 **
 ******************************************************************************
 */
static en_mflash_toggle_t MFlash_DualCheckToggle( uint16_t* pu16Addr )
{
    uint16_t   u16SequenceFlag1, u16SequenceFlag2 ;  /*  hardware sequence flag */
    en_mflash_toggle_t   emRetValue  = MFLASH_CHK_TOGG_NORMAL ;

    /* set hardware sequence flag */
    u16SequenceFlag1 = Flash_Read(pu16Addr) ;
    u16SequenceFlag2 = Flash_Read(pu16Addr) ;
    /*  if automatic algorithm is executing */
    while(MFLASH_CHK_TOGG_MASK == (( u16SequenceFlag1 ^ u16SequenceFlag2) & MFLASH_CHK_TOGG_MASK))
    {
        /*  if exceeds the timing limit */
        if(( u16SequenceFlag1 & MFLASH_CHK_TLOV_MASK) == MFLASH_CHK_TLOV_MASK)
        {
            /* set hardware sequence flag */
            u16SequenceFlag1 = Flash_Read(pu16Addr) ;
            u16SequenceFlag2 = Flash_Read(pu16Addr) ;

            /*  if automatic algorithm is executing */
            if((( u16SequenceFlag1 ^ u16SequenceFlag2) & MFLASH_CHK_TOGG_MASK) == MFLASH_CHK_TOGG_MASK)
            {
                /*  abnormally complete */
                emRetValue  = MFLASH_CHK_TOGG_ABNORMAL ;

                break;
            }
        }

        /* set hardware sequence flag */
        u16SequenceFlag1 = Flash_Read(pu16Addr) ;
        u16SequenceFlag2 = Flash_Read(pu16Addr) ;
    }

    return emRetValue ;
}
#ifdef __ICCARM__
  __ramfunc
#elif  __CC_ARM
  __attribute__ ((section (".ramfunc")))
#else
  #error Please check compiler and linker settings for RAM code
#endif
/*!
 ******************************************************************************
 ** \brief Dual Flash sector erase       
 **
 ** \param pu16SecAddr address of flash sector  
 **
 ** \return Operation status
 ** \retval Ok                     sector erase successfully done.
 ** \retval ErrorInvalidParameter  If one of following conditions are met:
 **                                - automatic algorithm of flash memory execution abnormally
 **
 ******************************************************************************
 */
en_result_t MFlash_DualSectorErase(uint16_t* pu16SecAddr)
{

    uint8_t  u8Dummy;
    en_result_t emRetValue = Ok;


    FM_DUALFLASH_IF->DFASZR = 0x01u;                                 ///< set 16 bit read/write (CPU programming mode)
    u8Dummy = FM_DUALFLASH_IF->DFASZR;     
    if(u8Dummy);  

    Flash_Write(MFLASH_CODE1 | ((uint32_t)pu16SecAddr & 0xFFFFF000), 0x00AAu);
    Flash_Write(MFLASH_CODE2 | ((uint32_t)pu16SecAddr & 0xFFFFF000), 0x0055u);
    Flash_Write(MFLASH_CODE1 | ((uint32_t)pu16SecAddr & 0xFFFFF000), 0x0080u);
    Flash_Write(MFLASH_CODE1 | ((uint32_t)pu16SecAddr & 0xFFFFF000), 0x00AAu);
    Flash_Write(MFLASH_CODE2 | ((uint32_t)pu16SecAddr & 0xFFFFF000), 0x0055u);
    Flash_Write(pu16SecAddr, 0x0030u);

     /*  if execution result of the automatic algorithm of flash memory is abnormally completed  */
    if( MFLASH_CHK_TOGG_ABNORMAL == MFlash_DualCheckToggle(pu16SecAddr))
    {
        /*  sending the read/reset command to the reset sector  */
        MFlash_DualReadResetCmd(pu16SecAddr) ;

        /*  return flash operation abnormally   */
        emRetValue  = ErrorInvalidParameter ;
    }

    FM_DUALFLASH_IF->DFASZR = 0x02u;                                 ///< set 32 bit read(CPU ROM mode)        
    u8Dummy = FM_DUALFLASH_IF->DFASZR;     
    if(u8Dummy);

    /* Recover IRQ  */
    return emRetValue;
}
#ifdef __ICCARM__
  __ramfunc
#elif  __CC_ARM
  __attribute__ ((section (".ramfunc")))
#else
  #error Please check compiler and linker settings for RAM code
#endif
/*!
 ******************************************************************************
 ** \brief Set Dual Flash Mode of FM4 TYPE3 product
 **
 ** \param bDualMode 
 ** \arg TRUE   Dual Flash mode
 ** \arg FALSE  Single mode or Main Flash mode
 **
 ** \return Ok Mode is set 
 **
 ******************************************************************************
 */
en_result_t MFlash_SetDualMode(boolean_t bDualMode)
{
    uint32_t u32Reg;
    
    u32Reg = FM_FLASH_IF->DFCTRLR;
    
    u32Reg &= ~1ul;             // Clear DFE bit
    u32Reg |= 0xEACC0000;       // Set WKEY
    
    if(TRUE == bDualMode)
    {
        u32Reg |= 0x0001;       // Set DFE (enable Dual Flash mode)
    }
    
    FM_FLASH_IF->DFCTRLR = u32Reg;
    
    return Ok;
}
#ifdef __ICCARM__
  __ramfunc
#elif  __CC_ARM
  __attribute__ ((section (".ramfunc")))
#else
  #error Please check compiler and linker settings for RAM code
#endif
/*!
 ******************************************************************************
 ** \brief Set remap function of FM4 TYPE3 product
 **
 ** \param bRemapMode 
 ** \arg TRUE   Remap mode
 ** \arg FALSE  Normal mode
 **
 ** \return Ok Mode is set 
 **
 ** \note This mode is only aviable for S6E2CCA (with 2048k+40k Flash)
 **
 ******************************************************************************
 */
en_result_t MFlash_SetRemapMode(boolean_t bRemapMode)
{
    uint32_t u32Reg;
    
    u32Reg = FM_FLASH_IF->DFCTRLR;
    
    u32Reg &= ~2ul;             // Clear RME bit
    u32Reg |= 0xEACC0000;       // Set WKEY
    
    if(TRUE == bRemapMode)
    {
        u32Reg |= 0x0002;       // Set RME (enable Dual Flash mode)
    }
    
    FM_FLASH_IF->DFCTRLR = u32Reg;
    
    return Ok;
}
#ifdef __ICCARM__
  __ramfunc
#elif  __CC_ARM
  __attribute__ ((section (".ramfunc")))
#else
  #error Please check compiler and linker settings for RAM code
#endif
/*!
 ******************************************************************************
 ** \brief Dual Flash word write with ECC  
 **
 ** \param pu32WriteAddr address of flash data
 ** \param pu32WriteData pointer to write data
 ** \param u32Size data size, 1 indicates 1 32-bit data
 ** \param bVerifyAndEccCheck Whether to verify and check ECC
 **
 ** \return Operation status
 ** \retval Ok                     Dual Flash written successfully done.
 ** \retval ErrorInvalidParameter  If one of following conditions are met:
 **                                - automatic algorithm of flash memory execution abnormally
 **                                - ECC abnormally
 **
 ******************************************************************************
 */
en_result_t MFlash_DualWriteData32Bit(uint32_t*  pu32WriteAddr,\
                                      uint32_t* pu32WriteData, \
                                      uint32_t u32Size, \
                                      boolean_t bVerifyAndEccCheck)
{
    uint16_t *pAddr, *pData;
    uint8_t  u8Dummy;

    en_result_t emRetValue  = Ok ;
    uint32_t   u32Cnt;
    pAddr = (uint16_t*)pu32WriteAddr;
    pData = (uint16_t*)pu32WriteData;

    FM_DUALFLASH_IF->DFASZR = 0x01u;                                 ///< set 16 bit read/write (CPU programming mode)
    u8Dummy = FM_DUALFLASH_IF->DFASZR;     
    if(u8Dummy);  

    for(u32Cnt=u32Size*sizeof(uint16_t);u32Cnt;u32Cnt--)
    {
        /*  issue write command   */
        Flash_Write(MFLASH_CODE1 | ((uint32_t)pu32WriteAddr & 0xFFFFF000), 0x00AAu) ;
        Flash_Write(MFLASH_CODE2 | ((uint32_t)pu32WriteAddr & 0xFFFFF000), 0x0055u) ;
        Flash_Write(MFLASH_CODE1 | ((uint32_t)pu32WriteAddr & 0xFFFFF000), 0x00A0u) ;
        Flash_Write((uint32_t)pAddr, (uint16_t)*pData);
        /*  execution result of the automatic algorithm of flash memory is abnormally complete or verify error  */
        if(( MFLASH_CHK_TOGG_ABNORMAL == MFlash_DualCheckToggle(pAddr)) ||
          ( Flash_Read((uint32_t)pAddr) != (uint16_t)*pData))
        {
            /*  issue read/reset command to reset sector    */
            MFlash_DualReadResetCmd(pAddr) ;

            /*  return flash operation abnormally   */
            emRetValue  = ErrorInvalidParameter ;
        }      
        /* Prepare next h-word write */
        pAddr++;
        pData++; 
    }

    FM_DUALFLASH_IF->DFASZR = 0x02u;                                 ///< set 32 bit read(CPU ROM mode)        
    u8Dummy = FM_DUALFLASH_IF->DFASZR;     
    if(u8Dummy);
  
    if(MFLASH_ECC_ABNORMAL ==  bVerifyAndEccCheck && 1 == FM_DUALFLASH_IF->DFSTR_f.DFERR)
    {
      emRetValue = ErrorInvalidParameter;
      bVerifyAndEccCheck = MFLASH_ECC_NORMAL;                     ///< clear ECC error bit
    }

    return emRetValue ;
}
#ifdef __ICCARM__
  __ramfunc
#elif  __CC_ARM
  __attribute__ ((section (".ramfunc")))
#else
  #error Please check compiler and linker settings for RAM code
#endif
/*!
 ******************************************************************************
 ** \brief Dual Flash half-word write
 **
 ** \param pu16WriteAddr address of flash data
 ** \param pu16WriteData pointer to write data
 ** \param u32Size data size, 1 indicates 1 16-bit data
 **
 ** \return Operation status
 ** \retval Ok                     Dual Flash written successfully done.
 ** \retval ErrorInvalidParameter  If one of following conditions are met:
 **                                - automatic algorithm of flash memory execution abnormally
 **                                - ECC abnormally
 **
 ******************************************************************************
 */
en_result_t MFlash_DualWriteData16Bit(uint16_t*  pu16WriteAddr, \
                                      uint16_t* pu16WriteData, \
                                      uint32_t u32Size)
{

    uint8_t  u8Dummy;
    en_result_t emRetValue  = Ok ;
    uint32_t   u32Cnt;

    FM_DUALFLASH_IF->DFASZR = 0x01u;                                 ///< set 16 bit read/write (CPU programming mode)
    u8Dummy = FM_DUALFLASH_IF->DFASZR;     
    if(u8Dummy);  

    for(u32Cnt=u32Size;u32Cnt;u32Cnt--)
    {
        /*  issue write command   */
        Flash_Write(MFLASH_CODE1 | ((uint32_t)pu16WriteAddr & 0xFFFFF000), 0x00AAu) ;
        Flash_Write(MFLASH_CODE2 | ((uint32_t)pu16WriteAddr & 0xFFFFF000), 0x0055u) ;
        Flash_Write(MFLASH_CODE1 | ((uint32_t)pu16WriteAddr & 0xFFFFF000), 0x00A0u) ;
        Flash_Write(pu16WriteAddr, (uint16_t)*pu16WriteData);
        
        /*  execution result of the automatic algorithm of flash memory is abnormally complete or verify error  */
        if(( MFLASH_CHK_TOGG_ABNORMAL == MFlash_DualCheckToggle(pu16WriteAddr)) ||
          ( Flash_Read(pu16WriteAddr) != *pu16WriteData))
        {
            /*  issue read/reset command to reset sector    */
            MFlash_DualReadResetCmd(pu16WriteAddr) ;

            /*  return flash operation abnormally   */
            emRetValue  = ErrorInvalidParameter ;
        }      
        /* Prepare next h-word write */
        pu16WriteAddr++;
        pu16WriteData++; 
    }

    FM_DUALFLASH_IF->DFASZR = 0x02u;                                 ///< set 32 bit read(CPU ROM mode)        
    u8Dummy = FM_DUALFLASH_IF->DFASZR;     

    return emRetValue ;
}

#endif

//@}

#endif
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
