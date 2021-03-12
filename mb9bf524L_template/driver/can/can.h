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
/** \file can.h
 **
 ** Headerfile for CAN functions
 **  
 ** History:
 **   - 2013-08-21  1.0  MWi  First version.
 **   - 2015-03-06  1.1  EZh  Port to FM universal PDL structure.
 **
 ******************************************************************************/

#ifndef __CAN_H__
#define __CAN_H__

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_user.h"
#include "mcu.h"

#if (defined(PDL_PERIPHERAL_CAN_ACTIVE))

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif
  
/**
 ******************************************************************************
 ** \defgroup CanGroup Controller Area Network (CAN)
 **
 ** Provided functions of CAN module:
 ** - Can_Init() (Initialisation of CAN module)
 ** - Can_DeInit() (Reset of all CAN related register to default value)
 ** - Can_SetTransmitMsgBuffer() (Used to initiate and configure a message buffer for a CAN message trasnmission)
 ** - Can_UpdateAndTransmitMsgBuffer() (Used to update the data in a message buffer and to transmit a message)
 ** - Can_SetReceiveMsgBuffer() (Used to initiate and setup a message buffer for reveiving data)
 **
 ** <b>How to use the CAN module:</b>
 ** 
 ** First, to initialize a CAN module, Can_Init() must be called. 
 ** The callback functions are optional, but recommended, otherwise there is no 
 ** report to the API in case of any error. 
 ** Can_DeInit() has to be used if any of the settings from Can_Init() have
 ** to be changed (use Can_DeInit() and afterwards Can_Init()).
 ** Can_DeInit() is used to completely disable the CAN module. 
 ** With Can_DeInit() all CAN related register values are reset to their default
 ** values. Also any pending or ongoing transmission or reception will be 
 ** aborted. 
 ** Each CAN module has #CAN_MESSAGE_BUFFER_COUNT number of message buffers 
 ** which can be used either for reception or transmission of CAN messages.
 ** Each message buffer for transmission has to be set up first by
 ** calling Can_SetTransmitMsgBuffer().
 ** For receiving CAN messages the function Can_SetReceiveMsgBuffer() has to be
 ** used. 
 ** It is possible to set a callback function which will be notified whenever a
 ** message has been received. 
 **
 ** \note The numbers of the message buffers used in this driver are
 ** indexed from 0 to 31 although the 'physical addresses' of these buffers are
 ** indexed from 1 to 32!
 ******************************************************************************/
//@{
  
/******************************************************************************
 * Global definitions
 ******************************************************************************/
#define stc_cann_t   FM_CAN_TypeDef  
#define CAN0       (*((volatile stc_cann_t *) FM_CAN0_BASE))
#define CAN1       (*((volatile stc_cann_t *) FM_CAN1_BASE))

/// Size of data bytes in a receive or transmit operation (usually 8)
#define CAN_MESSAGE_DATA_BUFFER_SIZE     8u

/// Number of possible message buffer for receive and transmit messages (usually 32)
#define CAN_MESSAGE_BUFFER_COUNT        32u

/// Lower limit for TSEG1 in structure stc_can_bitrate_t
#define CAN_BITRATE_TSEG1_MIN            2u
/// Upper limit for TSEG1 in structure stc_can_bitrate_t
#define CAN_BITRATE_TSEG1_MAX           16u

/// Lower limit for TSEG2 in structure stc_can_bitrate_t
#define CAN_BITRATE_TSEG2_MIN            1u
/// Upper limit for TSEG2 in structure stc_can_bitrate_t
#define CAN_BITRATE_TSEG2_MAX            8u


/// Lower limit for Sync Jump Width in structure stc_can_bitrate_t
#define CAN_BITRATE_SYNC_JUMP_WIDTH_MIN  1u
/// Upper limit for Sync Jump Width in structure stc_can_bitrate_t
#define CAN_BITRATE_SYNC_JUMP_WIDTH_MAX  4u

/// Lower limit for Prescaler in structure stc_can_bitrate_t
#define CAN_BITRATE_PRESCALER_MIN        1u
/// Upper limit for Prescaler in structure stc_can_bitrate_t
#define CAN_BITRATE_PRESCALER_MAX       64u
 
/// Maximum CAN clock frequency (after prescaler)
#if (PDL_MCU_TYPE == PDL_FM4_TYPE3) || (PDL_MCU_TYPE == PDL_FM4_TYPE4) || (PDL_MCU_TYPE == PDL_FM4_TYPE5)
#define CAN_MAX_CLK                     40000000u  
#else  
#define CAN_MAX_CLK                     16000000u
#endif  
  
/******************************************************************************
 * Global type definitions
 ******************************************************************************/
/**
 ******************************************************************************
 ** \brief CAN Unit configuration structures
 ******************************************************************************/

/**
 ******************************************************************************
 ** \brief CAN Prescaler
 ******************************************************************************/
typedef enum en_can_prescaler
{
  CanPreDiv11  = 0u,   ///< CAN prescaler clock: no division
  CanPreDiv12  = 1u,   ///< CAN prescaler clock: 1/2
  CanPreDiv14  = 2u,   ///< CAN prescaler clock: 1/4
  CanPreDiv18  = 3u,   ///< CAN prescaler clock: 1/8
  CanPreDiv23  = 4u,   ///< CAN prescaler clock: 2/3
  CanPreDiv13  = 5u,   ///< CAN prescaler clock: 1/3
  CanPreDiv16  = 6u,   ///< CAN prescaler clock: 1/6
  CanPreDiv112 = 7u,   ///< CAN prescaler clock: 1/12
  CanPreDiv15  = 8u,   ///< CAN prescaler clock: 1/5
  CanPreDiv110 = 9u    ///< CAN prescaler clock: 1/10
} en_can_prescaler_t;

/**
 ******************************************************************************
 ** \brief CAN bitrate.
 ** 
 ** This structure is used to set the bitrate register. All values are 
 ** decremented by 1 before writing to the CANn_BTR register 
 ** (see HW manual for more details).
 ******************************************************************************/
typedef struct stc_can_bitrate
{
  uint8_t             u8TimeSegment1;  ///< Range CAN_BITRATE_TSEG1_MIN to CAN_BITRATE_TSEG1_MAX (see define section)
  uint8_t             u8TimeSegment2;  ///< Range CAN_BITRATE_TSEG2_MIN to CAN_BITRATE_TSEG2_MAX (see define section)
  uint8_t             u8SyncJumpWidth; ///< Range CAN_BITRATE_SYNC_JUMP_WIDTH_MIN to CAN_BITRATE_SYNC_JUMP_WIDTH_MAX (see define section)
  uint16_t            u16Prescaler;    ///< Range PRESCALER_MIN to CAN_BITRATE_PRESCALER_MAX (see define section, divider for the peripheral clock CLKP2)
  en_can_prescaler_t  enCanPrescaler;  ///< CAN PRESCALER setting. See #en_can_prescaler_t for details
} stc_can_bitrate_t;


/**
 ******************************************************************************
 ** \brief CAN message identifier
 ** 
 ** This structure is used to set the CAN message identifier for transmit and 
 ** receive operations. The identifiction is needed when setting the CAN 
 ** arbitration register. On the one side this structure is used in the 
 ** Can_SetTransmitMsgBuffer() function, on the other side same structure is 
 ** used in the structure stc_can_msg_t which is used for receive data in 
 ** Can_SetReceiveMsgBuffer(...) function.
 ******************************************************************************/
typedef struct stc_can_msg_id
{
  uint32_t  u32Identifier;    ///< 11- or 29-bit identifier. The valid bit length depends on bExtended.
  boolean_t bExtended;        ///< TRUE: 29-bit identifier,  FALSE: 11-bit identifier.
} stc_can_msg_id_t;


/**
 ******************************************************************************
 ** \brief CAN message data.
 ** 
 ** Data structure for transmit and receive operations, maximum value for 
 ** parameter u8DataLengthCode is CAN_MESSAGE_DATA_BUFFER_SIZE.
 ******************************************************************************/
typedef struct stc_can_msg_data
{
  uint8_t au8Data[CAN_MESSAGE_DATA_BUFFER_SIZE];  ///< Data of CAN message.
  uint8_t u8DataLengthCode;                       ///< Number of valid bytes in au8Data and DLC of CAN message.
} stc_can_msg_data_t;


/**
 ******************************************************************************
 ** \brief CAN transmit mode.
 ** 
 ** To select how transmit data should be processed.
 ******************************************************************************/
typedef enum en_can_tx_mode
{
  CanImmediateTransmit   = 0u,    ///< Transmit message immediately
  CanRemoteTransmit      = 1u     ///< Transmit message on reception of remote frame
} en_can_tx_mode_t;


/**
 ******************************************************************************
 ** \brief CAN status.
 ** 
 ** These state values represent the current CAN state, and are used in the 
 ** CAN status changed callback function (can_status_chg_func_ptr_t).
 ******************************************************************************/
typedef enum en_can_status
{
  CanNoError    = 0u,   ///< No error pending.
  CanStuffError = 1u,   ///< More than 5 equal bits in a sequence have occurred in a part of a received message.
  CanFormError  = 2u,   ///< A fixed format part of a received frame has the wrong format.
  CanAckError   = 3u,   ///< The message this CAN Core transmitted was not acknowledged by another node.
  CanBit1Error  = 4u,   ///< While trying to send a recessive bit (1) a dominant level (0) was sampled.
  CanBit0Error  = 5u,   ///< While trying to send a dominant bit (0) a recessive level (1) was sampled.
  CanCRCError   = 6u    ///< The CRC checksum was incorrect.
} en_can_status_t;


/**
 ******************************************************************************
 ** \brief CAN error.
 ** 
 ** These error values are used to report any CAN related error to the API via
 ** the CAN error callback function (can_error_func_ptr_t).
 ******************************************************************************/
typedef enum en_can_error
{
  CanBusOff        = 0u,    ///< The CAN module is in busoff state.
  CanWarning       = 1u     ///< At least one error counter has reached error warning limit of 96.
} en_can_error_t;

/**
 ******************************************************************************
 ** \brief CAN message.
 ** 
 ** This structure stores a CAN message, including the identifier, data 
 ** and data length code (DLC). It also contains an overflow and new flag 
 ** which indicates the message state for received messages.
 ******************************************************************************/
typedef struct stc_can_msg
{
  stc_can_msg_id_t    stcIdentifier;  ///< 11- or 29-bit identifier (ID).
  stc_can_msg_data_t  stcData;        ///< Data and DLC.
  volatile boolean_t  bOverflow;      ///< TRUE if new message was received while bNew still TRUE 
  volatile boolean_t  bNew;           ///< TRUE if new message received (must be reset to FALSE by application to release message object).
} stc_can_msg_t;

/**
 *****************************************************************************
 ** \brief Message transmission complete callback function (can_tx_msg_func_ptr_t).
 **
 ** Signals a successful completed transmission.
 *****************************************************************************/
typedef void (*can_tx_msg_func_ptr_t)(uint8_t u8MsgBuf);


/**
 *****************************************************************************
 ** \brief Message reception callback function (can_rx_msg_func_ptr_t).
 **
 ** Signals that CAN has received a new message.
 *****************************************************************************/
typedef void (*can_rx_msg_func_ptr_t)(uint8_t u8MsgBuf, stc_can_msg_t* pstcRxMsg);


/**
 *****************************************************************************
 ** \brief Status changed callback function (can_status_chg_func_ptr_t).
 **
 ** Any status change will be reported to the API (see #en_can_status_t for 
 ** possible status change codes).
 *****************************************************************************/
typedef void (*can_status_chg_func_ptr_t)(en_can_status_t enCanStatus);


/**
 *****************************************************************************
 ** \brief Error callback function (can_error_func_ptr_t).
 **
 ** Any error will be reported to the API (see #en_can_error_t for 
 ** possible error codes).
 *****************************************************************************/
typedef void (*can_error_func_ptr_t)(en_can_error_t enCanError);

/**
 *****************************************************************************
 ** \brief CAN interrupt pointer structure
 **
 ** Holds some pointers to callback functions and buffer
 *****************************************************************************/
typedef struct stc_can_interrupt_handling
{
  stc_can_msg_t*        pstcMsgBuffer; ///< pointer to message buffer
  can_tx_msg_func_ptr_t pfnCanTxInterruptFunction; ///< pointer to transmit interrupt callback
  can_rx_msg_func_ptr_t pfnCanRxInterruptFunction; ///< pointer to receive interrupt callback
} stc_can_interrupt_handling_t;

/**
 *****************************************************************************
 ** \brief CAN notification pointer structure
 **
 ** Holds some pointers to callback functions for status and error notification
 *****************************************************************************/
typedef struct stc_can_notification
{
  can_status_chg_func_ptr_t pfnCanStatusInterruptFunction ;
  can_error_func_ptr_t      pfnCanErrorInterruptFunction ;
} stc_can_notification_t;

/**
 ******************************************************************************
 ** \brief CAN configuration.
 ** 
 ** Is used to set the CAN bitrate and allows deactivating of automatic
 ** retransmission in case of a transmission error. Optionally error and
 ** status changed notification can be set.  
 ******************************************************************************/
typedef struct stc_can_config
{
  can_status_chg_func_ptr_t   pfnStatusCallback;                ///< Callback function for CAN status changes, can be NULL.
  can_error_func_ptr_t        pfnErrorCallback;                 ///< Callback function for CAN related errors, can be NULL.
  boolean_t                   bDisableAutomaticRetransmission;  ///< TRUE: Automatic retransmission is disabled, FALSE: Automatic retransmission is enabled.
  stc_can_bitrate_t           stcBitrate;                       ///< CAN bitrate setting.
  boolean_t                   bTouchPrescaler;                  ///< TRUE: Can_Init() initializes CANPRES, FALSE: CANPRES not touched
} stc_can_config_t;

/// Enumeration to define an index for each enabled CAN instance
typedef enum en_can_instance_index
{
  #if (PDL_PERIPHERAL_ENABLE_CAN0 == PDL_ON)
    CanInstanceIndexCan0,
  #endif
  #if (PDL_PERIPHERAL_ENABLE_CAN1 == PDL_ON)
    CanInstanceIndexCan1,
  #endif
    CanInstanceIndexMax
} en_can_instance_index_t;

/// CAN instance internal data, storing internal information for each enabled CAN instance.
typedef struct stc_can_intern_data
{
  stc_can_interrupt_handling_t stcCanInterruptHandling[CAN_MESSAGE_BUFFER_COUNT];
  stc_can_notification_t       stcCanNotificationCb;
  boolean_t                    bEobZero;
} stc_can_intern_data_t;

/// CAN instance data type
typedef struct stc_can_instance_data
{
  volatile stc_cann_t*  pstcInstance;  ///< pointer to registers of an instance
  stc_can_intern_data_t stcInternData; ///< module internal data of instance
} stc_can_instance_data_t;

/******************************************************************************/
/* Global variable declarations ('extern', definition in C source)            */
/******************************************************************************/

/// Look-up table for all enabled CAN instances and their internal data
extern stc_can_instance_data_t m_astcCanInstanceDataLut[];

/******************************************************************************/
/* Global function prototypes ('extern', definition in C source)              */
/******************************************************************************/
extern void CanIrqHandler( volatile stc_cann_t* pstcCan,
                           stc_can_intern_data_t* pstcCanInternData
                         ) ;

extern en_result_t Can_Init( volatile stc_cann_t* pstcCan, 
                             const stc_can_config_t* pstcConfig
                           ) ;

extern en_result_t Can_DeInit( volatile stc_cann_t* pstcCan ) ;

extern en_result_t Can_SetTransmitMsgBuffer( volatile stc_cann_t* pstcCan, 
                                             uint8_t u8MsgBuf, 
                                             const stc_can_msg_id_t* pstcMsgId, 
                                             can_tx_msg_func_ptr_t pfnCallback
                                           ) ;

extern en_result_t Can_UpdateAndTransmitMsgBuffer( 
                                          volatile stc_cann_t* pstcCan, 
                                          uint8_t u8MsgBuf, 
                                          const stc_can_msg_data_t* pstcMsgData, 
                                          en_can_tx_mode_t enTxMode 
                                        ) ;

extern en_result_t Can_UpdateAndTransmitFifoMsgBuffer( volatile stc_cann_t* pstcCan,
                                                uint8_t u8MsgBuf,
                                                const stc_can_msg_data_t* pstcMsgData,
                                                en_can_tx_mode_t enTxMode
                                              ) ;

extern en_result_t Can_SetReceiveMsgBuffer( volatile stc_cann_t* pstcCan, 
                                            uint8_t u8MsgBuf, 
                                            stc_can_msg_t* pstcMsgBuffer, 
                                            uint32_t u32MsgIdMask, 
                                            can_rx_msg_func_ptr_t pfnCallback
                                          ) ;

extern en_result_t Can_SetReceiveFifoMsgBuffer( volatile stc_cann_t* pstcCan,
                                                uint8_t u8MsgBuf,
                                                stc_can_msg_t* pstcMsgBuffer,
                                                uint32_t u32MsgIdMask,
                                                can_rx_msg_func_ptr_t pfnCallback
                                              ) ;

extern en_result_t Can_ResetMsgBuffer( volatile stc_cann_t* pstcCan, uint8_t u8MsgBuf ) ;

//@}
#ifdef __cplusplus
}
#endif

#endif // #if (defined(PDL_PERIPHERAL_CAN_ACTIVE))

#endif /* __CAN_H__ */
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
