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
/** \file canfd.h
 **
 ** Headerfile for CAN FD functions
 **
 ** History:
 **   - 2014-08-28  1.0  AI   First version.
 **
 ******************************************************************************/

#ifndef __CANFD_H__
#define __CANFD_H__

/******************************************************************************/
/* Include files                                                              */
/******************************************************************************/
#include "pdl_user.h"
#include "mcu.h"

#if (defined(PDL_PERIPHERAL_CANFD_ACTIVE))

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/**
 ******************************************************************************
 ** \defgroup CanfdGroup Controller Area Network with Flexible Data-Rate (CANFD)
 **
 ** Provided functions of CANFD module:
 ** - Canfd_Init() (Initialisation of CAN FD module)
 ** - Canfd_DeInit() (Reset of all CAN FD related register to default value)
 ** - Canfd_Start() (Start communication)
 ** - Canfd_Stop() (Stop communication)
 ** - Canfd_Restart() (Restart communication)
 ** - Canfd_TransmitMsg() (Request to send message)
 ** - Canfd_ReceiveMsg() (Receive message by polling)
 ** - Canfd_GetBusStatus() (Detect BusOff)
 **
 ** <b>How to use the CAN FD module:</b>
 **  
 ** First, to initialize a CAN module, Canfd_Init() must be called.
 ** The callback functions are optional, but recommended, otherwise there is no
 ** report to the API in case of any reception, transmission, bus state and error
 ** event.
 ** Canfd_DeInit() has to be used if any of the settings from Canfd_Init() have
 ** to be changed (use Canfd_DeInit() and afterwards Canfd_Init()).
 ** Canfd_DeInit() is used to completely disable the CAN FD module.
 ** With Canfd_DeInit() all CAN FD related register values are reset to their default
 ** values. Also any pending or ongoing transmission or reception will be
 ** aborted.
 ** Each CAN FD module has dedicated message buffers for receiving and transmiting.
 ** The receiving buffer consists of dedicated Rx byffer, Rx FIFO0 and Rx FIFO1.
 ** In the same way, the transmission buffer consists of dedicated Tx buffer and 
 ** Tx FIFO/QUEUE.
 ** Canfd_Start() is used to start communication via CAN FD, and Canfd_Stop() is used
 ** to stop communication.
 ** Canfd_Restart() is used to restart communication. Usually, it will be called
 ** to recovery from CAN FD error state.
 ** Canfd_ReceiveMsg() is used to receive CAN messages.
 ** Canfd_TransmitMsg() is used to transmit CAN messages.
 ** Canfd_GetBusStatus() is used to get the bus status.
 ** Received message, completion of message transmission, changing of bus status
 ** and occurring of errors are always notified by calling to corresponding
 ** callback functions. The application has to specify the callback function that
 ** needs to know, or the corresponding event is never notified.
 **
 ** \note The numbers of the message buffers used in this driver are shown as
 ** follows;
 **   dedicated Rx buffer : 1 element, indexed 0.
 **   Rx FIFO0            : 2 elements.
 **   Rx FIFO1            : not used.
 **   Tx buffer           : 1 element, indexed 0.
 **   Tx FIFO/QUEUE       : not used.
 ******************************************************************************/
//@{

/******************************************************************************
 * Global definitions
 ******************************************************************************/

// CANFD controller
#define stc_canfdn_t  FM_CANFD_TypeDef
#define CANFD0      (*((volatile stc_canfdn_t *)FM_CANFD0_BASE))

/// Size of data bytes in a receive or transmit operation
#define CANFD_MESSAGE_DATA_BUFFER_SIZE			64
/// Size of data word in a receive or transmit operation
#define CANFD_MESSAGE_DATA_BUFFER_SIZEW			(CANFD_MESSAGE_DATA_BUFFER_SIZE / 4)

/// Number of possible dedicated Rx buffer for receive
#define CANFD_MESSAGE_RXBUFFER_COUNT    		64

/// Number of possible Rx FIFO for receive messages
#define CANFD_MESSAGE_RXFIFO_COUNT      		32

/// Number of possible dedicated Tx buffer, Tx Queue and Tx FIFO for transmit messages
#define CANFD_MESSAGE_TXBUFFER_COUNT    		32

/******************************************************************************
 * Global type definitions
 ******************************************************************************/

/**
 ******************************************************************************
 ** \brief The CAN FD driver uses definitions of the structure of registers on
 ** the CAN FD controller that were defined in the "REGISTER DEFINITION HEADER"
 ** for the target MCU.
 ******************************************************************************/

/**
 ******************************************************************************
 ** \brief CAN FD Unit configuration structures
 ******************************************************************************/

/**
 ******************************************************************************
 ** \brief CAN FD received message index.
 **
 ** These are used to specify CAN operation mode.
 ******************************************************************************/
#define	CANFD_MSGIDX_MIN		(0U)		// minimum index of dedicated Rx buffer.
#define	CANFD_MSGIDX_MAX		(63U)		// maximum index of dedicated Rx buffer.
#define	CANFD_MSGIDX_FIFO0		(254U)		// received message stored into Rx FIFO0.
#define	CANFD_MSGIDX_FIFO1		(255U)		// received message stored into Rx FIFO1.

#define	CANFD_MAX_STDDLC		(8)			// maximum data length in standard CAN operation.

/**
 ******************************************************************************
 ** \brief CAN operation mode.
 **
 ** These are used to specify CAN operation mode.
 ******************************************************************************/
typedef enum en_canfd_mode
{
	CanfdModeClassic = 0,	// classic CAN (CAN2.0)
	CanfdModeFDFixed = 1,	// CAN-FD with fixed data rate
	CanfdModeFDFlex  = 2	// CAN-FD with flexible data rate
} en_canfd_mode_t;

/**
 ******************************************************************************
 ** \brief CAN FD operation clock frequency.
 **
 ** These are used to determin clock frequency of CAN FD controller.
 ******************************************************************************/
typedef enum en_canfd_clock
{
	CanfdClock32MHz = 0,	// 32MHz
	CanfdClock40MHz = 1		// 40MHz
} en_canfd_clock_t;


/**
 ******************************************************************************
 ** \brief CAN message identifier
 **
 ** This structure is used to set the CAN message identifier for transmit and
 ** receive operations.
 ******************************************************************************/
typedef struct stc_canfd_msg_id
{
	uint32_t	u32Identifier;    	// 11- or 29-bit identifier. The valid bit length depends on bExtended.
	boolean_t	bExtended;			// TRUE: 29-bit identifier,  FALSE: 11-bit identifier.
} stc_canfd_msg_id_t;


/**
 ******************************************************************************
 ** \brief CAN message data.
 **
 ** Data structure for transmit and receive operations, maximum value for
 ** parameter u8DataLengthCode is CANFD_MESSAGE_DATA_BUFFER_SIZE.
 ******************************************************************************/
typedef struct stc_canfd_msg_data
{
	union
	{
		uint32_t	au32Data[CANFD_MESSAGE_DATA_BUFFER_SIZEW];	// Data of CAN message.
		uint8_t		au8Data[CANFD_MESSAGE_DATA_BUFFER_SIZE];	// Data of CAN message.
	};
	uint8_t	u8DataLengthCode;                       			// Number of valid bytes in au8Data and DLC of CAN message.
} stc_canfd_msg_data_t;


/**
 ******************************************************************************
 ** \brief CAN FD module status.
 **
 ** These state values represent the current CAN FD module state, and are used 
 ** in the CAN FD status changed callback function (canfd_status_chg_func_ptr_t).
 ******************************************************************************/
typedef enum en_canfd_status
{
	CanfdBusOff  = 0,   // The CAN FD module is in busoff state.
	CanfdWarning = 1    // At least one error counter has reached error warning limit of 96.
} en_canfd_status_t;


/**
 ******************************************************************************
 ** \brief CAN FD error.
 **
 ** These error values are used to report any CAN FD related error to the API via
 ** the CAN FD error callback function.
 ** The parameter of this callback function is uint32_t type, and it includes 
 ** all of detected error flags as shown below;
 ******************************************************************************/
#define	CANFD_ERROR_STE		(1UL << 31)		// [bit31] STE : Stuff Error / More than 5 equal bits in a sequence occurred.
#define	CANFD_ERROR_FOE		(1UL << 30)		// [bit30] FOE : Format Error / A fixed format part of a received frame has the wrong format.
#define	CANFD_ERROR_ACKE	(1UL << 29)		// [bit29] ACKE: Acknowledge Error / A transmitted message was not acknowledged by another node.
#define	CANFD_ERROR_BE		(1UL << 28)		// [bit28] BE  : Bit Error / CAN FD Controller wanted to send a recessive/dominant level, but monitored bus level was dominant/recessive.
#define	CANFD_ERROR_CRCE	(1UL << 27)		// [bit27] CRCE: CRC Error / Received CRC did not match the calculated CRC.
#define	CANFD_ERROR_WDI		(1UL << 26)		// [bit26] WDI : Watchdog Interrupt / Message RAM Watchdog event due to missing READY from Message RAM.
#define	CANFD_ERROR_ELO		(1UL << 22)		// [bit22] ELO : Error Logging Overflow / Overflow of CAN Error Logging Counter (ECR.CEL[7:0]) occurred.
#define	CANFD_ERROR_BEU		(1UL << 21)		// [bit21] BEU : Bit Error Uncorrected / Bit error detected, but could not be corrected.
#define	CANFD_ERROR_BEC		(1UL << 20)		// [bit20] BEC : Bit Error Corrected / Bit error detected and corrected by ECC logic.
#define	CANFD_ERROR_TOO		(1UL << 18)		// [bit18] TOO : Timeout Occurred / Timeout reached.
#define	CANFD_ERROR_MRAF	(1UL << 17)		// [bit17] MRAF: Message RAM Access Failure / Message RAM access failure occurred.
#define	CANFD_ERROR_TEFL	(1UL << 15)		// [bit15] TEFL: Tx Event FIFO Element Lost / Tx Event FIFO element lost. Also set after write attempt to Tx Event FIFO of size zero.
#define	CANFD_ERROR_RF1L	(1UL <<  7)		// [bit7]  RF1L: Rx FIFO 1 Message Lost / Rx FIFO 1 message lost. Also set after write attempt to Rx FIFO 1 of size zero.
#define	CANFD_ERROR_RF0L	(1UL <<  3)		// [bit3]  RF0L: Rx FIFO 0 Message Lost / Rx FIFO 0 message lost. Also set after write attempt to Rx FIFO 0 of size zero.


/**
 ******************************************************************************
 ** \brief CAN message.
 **
 ** This structure stores a CAN or CAN FD message, including the identifier, 
 ** data and data length code (DLC). It also contains an overflow and new flag
 ** which indicates the message state for received messages.
 ******************************************************************************/
typedef struct stc_canfd_msg
{
	stc_canfd_msg_id_t		stcIdentifier;  // 11- or 29-bit identifier (ID).
	stc_canfd_msg_data_t	stcData;		// Data and DLC.
	boolean_t				bCanfd;			// TRUE : DLC is extended, FALSE : Normal DLC
											// (valid received message only.)
} stc_canfd_msg_t;


/**
 *****************************************************************************
 ** \brief Message transmission complete callback function (canfd_tx_msg_func_ptr_t).
 **
 ** Signals a successful completed transmission.
 *****************************************************************************/
typedef void (*canfd_tx_msg_func_ptr_t)( uint8_t u8MsgBuf );


/**
 *****************************************************************************
 ** \brief Message reception callback function (canfd_rx_msg_func_ptr_t).
 **
 ** Signals that CAN has received a new message.
 *****************************************************************************/
typedef void (*canfd_rx_msg_func_ptr_t)( uint8_t u8MsgBuf, stc_canfd_msg_t* pstcRxMsg );


/**
 *****************************************************************************
 ** \brief Status changed callback function (canfd_status_chg_func_ptr_t).
 **
 ** Any status change will be reported to the API (see #en_canfd_status_t for
 ** possible status change codes).
 *****************************************************************************/
typedef void (*canfd_status_chg_func_ptr_t)( en_canfd_status_t enCanfdStatus );


/**
 *****************************************************************************
 ** \brief Error callback function (canfd_error_func_ptr_t).
 **
 ** Any error will be reported to the API 
 *****************************************************************************/
typedef void (*canfd_error_func_ptr_t)( uint32_t u32CanfdError );


/**
 *****************************************************************************
 ** \brief CAN FD interrupt pointer structure
 **
 ** Holds some pointers that point to callback functions for message reception
 ** and transmission.
 *****************************************************************************/
typedef struct stc_canfd_interrupt_handling
{
	canfd_tx_msg_func_ptr_t	pfnCanfdTxInterruptFunction; 	// pointer to transmit interrupt callback
	canfd_rx_msg_func_ptr_t	pfnCanfdRxInterruptFunction; 	// pointer to receive interrupt callback
} stc_canfd_interrupt_handling_t;


/**
 *****************************************************************************
 ** \brief CAN FD notification pointer structure
 **
 ** Holds some pointers to callback functions for status and error notification
 *****************************************************************************/
typedef struct stc_canfd_notification
{
	canfd_status_chg_func_ptr_t	pfnCanfdStatusInterruptFunction ;
	canfd_error_func_ptr_t		pfnCanfdErrorInterruptFunction ;
} stc_canfd_notification_t;


/**
 ******************************************************************************
 ** \brief CAN FD configuration.
 **
 ** Is used to set the CAN operation mode and CAN FD clock frequency.
 ** Optionally reception, transmission, error and status notification callback
 ** functions can be set.
 ******************************************************************************/
typedef struct stc_canfd_config
{
	en_canfd_mode_t				enCanfdMode;			// CAN operation mode.
	en_canfd_clock_t			enCanfdClock;			// CAN FD operation clock (prescaler output).
	canfd_rx_msg_func_ptr_t		pfnReceiveMsgCallback; 	// Callback function for receive message, can be NULL.
	canfd_tx_msg_func_ptr_t		pfnTransmitMsgCallback; // Callback function for transmit message, can be NULL.
	canfd_status_chg_func_ptr_t	pfnStatusCallback;      // Callback function for CAN FD status changes, can be NULL.
	canfd_error_func_ptr_t		pfnErrorCallback;       // Callback function for CAN FD related errors, can be NULL.
} stc_canfd_config_t;


/******************************************************************************/
/* Global variable declarations ('extern', definition in C source)            */
/******************************************************************************/


/******************************************************************************/
/* Global function prototypes ('extern', definition in C source)              */
/******************************************************************************/

extern void CanfdIrqHandler( volatile stc_canfdn_t* pstcCanfd );

extern en_result_t Canfd_Init( volatile stc_canfdn_t* pstcCanfd,
							   const stc_canfd_config_t* pstcConfig
							 );

extern en_result_t Canfd_DeInit( volatile stc_canfdn_t* pstcCanfd );

extern en_result_t Canfd_Start( volatile stc_canfdn_t* pstcCanfd );

extern en_result_t Canfd_Stop( volatile stc_canfdn_t* pstcCanfd );

extern en_result_t Canfd_Restart( volatile stc_canfdn_t* pstcCanfd );

extern en_result_t Canfd_TransmitMsg( volatile stc_canfdn_t* pstcCanfd,
									  uint8_t u8MsgBuf,
									  stc_canfd_msg_t* pstcMsg
									);

extern en_result_t Canfd_ReceiveMsg( volatile stc_canfdn_t* pstcCanfd );

extern en_result_t Canfd_GetBusStatus( volatile stc_canfdn_t* pstcCanfd );

#ifdef __cplusplus
}
#endif

//@}

#endif // #if (defined(PDL_PERIPHERAL_CANFD_ACTIVE))

#endif /* __CANFD_H__ */
/******************************************************************************/
/* EOF (not truncated)                                                        */
/******************************************************************************/
