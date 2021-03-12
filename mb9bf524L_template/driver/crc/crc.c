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
/** \file crc.c
 **
 ** A detailed description is available at 
 ** @link CrcGroup CRC Module description @endlink
 **
 ** History:
 **   - 2014-11-18  1.0  EZh   First version for FM universal PDL.
 **
 ******************************************************************************/

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "crc.h"

#if (defined(PDL_PERIPHERAL_CRC_ACTIVE))

/**
 ******************************************************************************
 ** \addtogroup CrcGroup
 ******************************************************************************/
//@{

/******************************************************************************/
/* Local pre-processor symbols/macros ('#define')                             */
/******************************************************************************/

/******************************************************************************/
/* Global variable definitions (declared in header file with 'extern')        */
/******************************************************************************/

/******************************************************************************/
/* Local type definitions ('typedef')                                         */
/******************************************************************************/

/******************************************************************************/
/* Local function prototypes ('static')                                       */
/******************************************************************************/

/******************************************************************************/
/* Local variable definitions ('static')                                      */
/******************************************************************************/

/******************************************************************************/
/* Function implementation - global ('extern') and local ('static')           */
/******************************************************************************/


/**
 ******************************************************************************
 ** \brief Initialisation of a CRC module.
 **
 ** \param [in]  pstcConfig    CRC module configuration 
 **
 ** \retval Ok                     Initializiation of CRC module successfully 
 **                                done.
 ** \retval ErrorInvalidParameter  pstcConfig == NULL,
 **                                parameter out of range
 ******************************************************************************/
en_result_t Crc_Init(stc_crc_config_t* pstcConfig)
{
    en_result_t enResult;
    stc_crc_crccr_field_t stcCRCCR; /* local preset of CRCCR */

    /* Check for valid pointers */
    if (NULL == pstcConfig)
    {
        enResult = ErrorInvalidParameter;
    }
    else
    {
        enResult = Ok;
        /* Check CRC mode select and setting */
        switch (pstcConfig->enMode)
        {
            case Crc16:
                /* Set CRC16 */
                stcCRCCR.CRC32 = FALSE;
                /* Mask the initial value for 16 bits */
                pstcConfig->u32CrcInitValue &= 0x0000FFFFu;
                break;
            case Crc32:
                /* Set CRC32 */
                stcCRCCR.CRC32 = TRUE;
                break;
            default:
                enResult = ErrorInvalidParameter;
                break;
        }

        if (Ok == enResult)
        {
            /* XOR is executed finaly or not */
            if (TRUE == pstcConfig->bFinalXor)
            {
                stcCRCCR.FXOR = TRUE;
            }
            else
            {
                stcCRCCR.FXOR = FALSE;
            }
            /* Bit order for CRC result */
            if (TRUE == pstcConfig->bResultLsbFirst)
            {
                stcCRCCR.CRCLSF = TRUE;
            }
            else
            {
                stcCRCCR.CRCLSF = FALSE;
            }
            /* Byte order for CRC result */
            if (TRUE == pstcConfig->bResultLittleEndian)
            {
                stcCRCCR.CRCLTE = TRUE;
            }
            else
            {
                stcCRCCR.CRCLTE = FALSE;
            }
            /* Bit order for DATA */
            if (TRUE == pstcConfig->bDataLsbFirst)
            {
                stcCRCCR.LSBFST = TRUE;
            }
            else
            {
                stcCRCCR.LSBFST = FALSE;
            }
            /* Byte order for DATA */
            if (TRUE == pstcConfig->bDataLittleEndian)
            {
                stcCRCCR.LTLEND = TRUE;
            }
            else
            {
                stcCRCCR.LTLEND = FALSE;
            }
            /* Initialize CRC configuration */
            stcCRCCR.INIT = TRUE;

            /* Set the initial value */
            FM_CRC->CRCINIT = pstcConfig->u32CrcInitValue;
            /* Write setting to CRC control register */
            FM_CRC->CRCCR_f = stcCRCCR;
        }
    }

    return (enResult);
} /* Crc_Init */


/**
 ******************************************************************************
 ** \brief De-Initialisation of a CRC module.
 **
 ******************************************************************************/
void Crc_DeInit(void)
{
    /* clear hardware */
    FM_CRC->CRCIN = 0u;
    FM_CRC->CRCCR = 0u;

} /* Crc_DeInit */

/**
 ******************************************************************************
 ** \brief Push 8-bit integer data to a CRC module with if no DMA is used.
 **
 ** \param [in]  u8DataToPush  8-Bit data to be pushed to CRC
 **
 ******************************************************************************/
void Crc_Push8(uint8_t u8DataToPush)
{
    /* Caluculate CRC (Push 8bit data) */
    FM_CRC->CRCINLL = u8DataToPush;

} /* Crc_Push8 */

/**
 ******************************************************************************
 ** \brief Push 16-bit integer data to a CRC module with if no DMA is used.
 **
 ** \note Be careful with the endianess. Byte swapping might have to be
 **       performed before pushing 16-bit data.
 **
 ** \param [in]  u16DataToPush  16-Bit data to be pushed to CRC
 **
 ******************************************************************************/
void Crc_Push16(uint16_t u16DataToPush)
{
    /* Caluculate CRC (Push 16bit data) */
    FM_CRC->CRCINL = u16DataToPush;

} /* Crc_Push16 */

/**
 ******************************************************************************
 ** \brief Push 32-bit integer data to a CRC module with if no DMA is used.
 **
 ** \param [in]  u32DataToPush  32-Bit data to be pushed to CRC
 **
 ******************************************************************************/
void Crc_Push32(uint32_t u32DataToPush)
{
    /* Caluculate CRC (Push 32bit data) */
    FM_CRC->CRCIN = u32DataToPush;

} /* Crc_Push32 */

/**
 ******************************************************************************
 ** \brief Read CRC result register
 **
 ** \note This function returns a 32-bit value regardless of a valid
 **       pointer to the CRC instance anyhow.
 **
 ** \retval value of CRC result
 **
 ******************************************************************************/
uint32_t Crc_ReadResult(void)
{
    /* Return value of CRC result register */
    return (FM_CRC->CRCR);
} /* Crc_ReadResult */


//@} // CrcGroup

#endif /* #if (defined(PDL_PERIPHERAL_CRC_ACTIVE)) */
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
