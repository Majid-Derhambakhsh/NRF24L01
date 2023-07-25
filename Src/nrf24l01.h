/*
------------------------------------------------------------------------------
~ File   : nrf24l01.h
~ Author : Majid Derhambakhsh
~ Version: V1.0.0
~ Created: 01/27/2023 10:00:00 PM
~ Brief  :
~ Support:
           E-Mail : Majid.Derhambakhsh@gmail.com (subject : Embedded Library Support)
           
           Github : https://github.com/Majid-Derhambakhsh
------------------------------------------------------------------------------
~ Description:    

~ Attention  :    This file is for AVR/ARM MCU

~ Changes    :
------------------------------------------------------------------------------
*/

#ifndef __NRF24L01_H_
#define __NRF24L01_H_

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Include ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include <stdint.h>

#include "nrf24l01_conf.h"

/* ------------------------------------------------------------------ */
#ifdef _CodeVISIONAVR__  /* Check compiler */

#pragma warn_unref_func- /* Disable 'unused function' warning */

#ifndef __SPI_UNIT_H_
	#include "SPI_UNIT/spi_unit.h"
#endif /* __SPI_UNIT_H_ */

#ifndef __GPIO_UNIT_H_
	#include "GPIO_UNIT/gpio_unit.h"
#endif /* __GPIO_UNIT_H_ */

/* ------------------------------------------------------------------ */
#elif defined(__GNUC__) && !defined(USE_HAL_DRIVER)  /* Check compiler */

#pragma GCC diagnostic ignored "-Wunused-function" /* Disable 'unused function' warning */

#ifndef __SPI_UNIT_H_
	#include "SPI_UNIT/spi_unit.h"
#endif /* __SPI_UNIT_H_ */

#ifndef __GPIO_UNIT_H_
	#include "GPIO_UNIT/gpio_unit.h"
#endif /* __GPIO_UNIT_H_ */

/* ------------------------------------------------------------------ */
#elif defined(USE_HAL_DRIVER)  /* Check driver */

	#if LCD_USE_FREE_RTOS == 1
		#include "cmsis_os.h"
	#endif /* _LCD_USE_FREE_RTOS */
	
	/* --------------- Check Mainstream series --------------- */
	#ifdef STM32F0
		#include "stm32f0xx_hal.h"       /* Import HAL library */
	#elif defined(STM32F1)
		#include "stm32f1xx_hal.h"       /* Import HAL library */
	#elif defined(STM32F2)
		#include "stm32f2xx_hal.h"       /* Import HAL library */
	#elif defined(STM32F3)
		#include "stm32f3xx_hal.h"       /* Import HAL library */
	#elif defined(STM32F4)
		#include "stm32f4xx_hal.h"       /* Import HAL library */
	#elif defined(STM32F7)
		#include "stm32f7xx_hal.h"       /* Import HAL library */
	#elif defined(STM32G0)
		#include "stm32g0xx_hal.h"       /* Import HAL library */
	#elif defined(STM32G4)
		#include "stm32g4xx_hal.h"       /* Import HAL library */
	
	/* ------------ Check High Performance series ------------ */
	#elif defined(STM32H7)
		#include "stm32h7xx_hal.h"       /* Import HAL library */
	
	/* ------------ Check Ultra low power series ------------- */
	#elif defined(STM32L0)
		#include "stm32l0xx_hal.h"       /* Import HAL library */
	#elif defined(STM32L1)
		#include "stm32l1xx_hal.h"       /* Import HAL library */
	#elif defined(STM32L5)
		#include "stm32l5xx_hal.h"       /* Import HAL library */
	#elif defined(STM32L4)
		#include "stm32l4xx_hal.h"       /* Import HAL library */
	#elif defined(STM32H7)
		#include "stm32h7xx_hal.h"       /* Import HAL library */
	#else
	#endif /* STM32F1 */
	
	/* ------------------------------------------------------- */
	#if defined(__ICCARM__) /* ICCARM Compiler */
	
		#pragma diag_suppress=Pe177   /* Disable 'unused function' warning */
		
	#elif defined(__GNUC__) /* GNU Compiler */
	
		#pragma diag_suppress 177     /* Disable 'unused function' warning */
		
	#endif /* __ICCARM__ */

/* ------------------------------------------------------------------ */
#else                     /* Compiler not found */

#error Chip or Library not supported  /* Send error */

#endif /* _CodeVISIONAVR__ */

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ------------------------------ NRF ------------------------------ */
/* .................... Radio ..................... */
#define NRF24L01_BASE_CHANNEL_MHZ 2400

/* Addresses */
#define NRF24L01_ADDRESS_WIDTH    5

#define NRF24L01_REG_RX_ADDR_P0_P5_DEF 0xE7

/* .................... Common .................... */
/* ................... Macro's .................... */
#define __NRF24L01_SET_BIT(REG, BIT)     ((REG) |= (1 << (BIT)))
#define __NRF24L01_RESET_BIT(REG, BIT)   ((REG) &= ~(1 << (BIT)))
#define __NRF24L01_CHECK_BIT(REG, BIT)   (((REG) >> (BIT)) & 1)
#define __NRF24L01_MASK_BIT(REG, MASK)   ((REG) &= (MASK))

/* ----------------------- Define by compiler ---------------------- */
#if (defined(_CodeVISIONAVR__) || defined(__GNUC__)) && !defined(USE_HAL_DRIVER)
	
	#ifndef NRF24L01_GPIO_WritePin
		#define NRF24L01_GPIO_WritePin(gpiox , gpio_pin , pin_state)           GPIO_WritePin(&(gpiox) , (uint8_t)(gpio_pin) , pin_state)
	#endif
	
	#ifndef NRF24L01_GPIO_PIN_SET
		#define NRF24L01_GPIO_PIN_SET                                          _GPIO_PIN_SET
	#endif
	
	#ifndef NRF24L01_GPIO_PIN_RESET
		#define NRF24L01_GPIO_PIN_RESET                                        _GPIO_PIN_RESET
	#endif
	
	#ifndef NRF24L01_SPI_TransmitReceive
		#define NRF24L01_SPI_TransmitReceive(pTxData, pRxData, Size, Timeout)  SPI_TransmitReceive((pTxData), (pRxData), (Size), (Timeout))
		#define NRF24L01_SPI_Transmit(pTxData, Size, Timeout)                  SPI_Transmit((pTxData), (Size), (Timeout))
		#define NRF24L01_SPI_Receive(pRxData, Size, Timeout)                   SPI_Receive((pRxData), (Size), (Timeout))
	#endif
	
/*----------------------------------------------------------*/
#elif defined(USE_HAL_DRIVER)
	
	#ifndef NRF24L01_GPIO_WritePin
		#define NRF24L01_GPIO_WritePin(gpiox , gpio_pin , pin_state)           HAL_GPIO_WritePin((gpiox) , (uint32_t)(gpio_pin) , (pin_state))
	#endif
	
	#ifndef NRF24L01_GPIO_PIN_SET
		#define NRF24L01_GPIO_PIN_SET                                          GPIO_PIN_SET
	#endif
	
	#ifndef NRF24L01_GPIO_PIN_RESET
		#define NRF24L01_GPIO_PIN_RESET                                        GPIO_PIN_RESET
	#endif
		
	#ifndef NRF24L01_SPI_TransmitReceive
		#define NRF24L01_SPI_TransmitReceive(pTxData, pRxData, Size, Timeout)  HAL_SPI_TransmitReceive(&NRF24L01_SPI, (pTxData), (pRxData), (Size), (Timeout))
		#define NRF24L01_SPI_Transmit(pTxData, Size, Timeout)                  HAL_SPI_Transmit(&NRF24L01_SPI, (pTxData), (Size), (Timeout))
		#define NRF24L01_SPI_Receive(pRxData, Size, Timeout)                   HAL_SPI_Receive(&NRF24L01_SPI, (pRxData), (Size), (Timeout))
	#endif

#endif /* __GNUC__ */

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ----------------------- NRF24L01+ Commands ---------------------- */
typedef enum /* NRF24L01+ Command Type */
{
	
	NRF24L01_CMD_R_REGISTER         = 0b00000000,
	NRF24L01_CMD_W_REGISTER         = 0b00100000,
	NRF24L01_CMD_R_RX_PAYLOAD       = 0b01100001,
	NRF24L01_CMD_W_TX_PAYLOAD       = 0b10100000,
	NRF24L01_CMD_FLUSH_TX           = 0b11100001,
	NRF24L01_CMD_FLUSH_RX           = 0b11100010,
	NRF24L01_CMD_REUSE_TX_PL        = 0b11100011,
	NRF24L01_CMD_R_RX_PL_WID        = 0b01100000,
	NRF24L01_CMD_W_ACK_PAYLOAD      = 0b10101000,
	NRF24L01_CMD_W_TX_PAYLOAD_NOACK = 0b10110000,
	NRF24L01_CMD_NOP                = 0b11111111
	
}NRF24L01_CMDTypeDef;

/* ---------------------- NRF24L01 Register's ---------------------- */
typedef enum /* NRF24L01+ Registers */
{
	
	NRF24L01_REG_CONFIG      = 0x00,
	NRF24L01_REG_EN_AA       = 0x01,
	NRF24L01_REG_EN_RXADDR   = 0x02,
	NRF24L01_REG_SETUP_AW    = 0x03,
	NRF24L01_REG_SETUP_RETR  = 0x04,
	NRF24L01_REG_RF_CH       = 0x05,
	NRF24L01_REG_RF_SETUP    = 0x06,
	NRF24L01_REG_STATUS      = 0x07,
	NRF24L01_REG_OBSERVE_TX  = 0x08, // Read-Only
	NRF24L01_REG_RPD         = 0x09, // Read-Only
	NRF24L01_REG_RX_ADDR_P0  = 0x0A,
	NRF24L01_REG_RX_ADDR_P1  = 0x0B,
	NRF24L01_REG_RX_ADDR_P2  = 0x0C,
	NRF24L01_REG_RX_ADDR_P3  = 0x0D,
	NRF24L01_REG_RX_ADDR_P4  = 0x0E,
	NRF24L01_REG_RX_ADDR_P5  = 0x0F,
	NRF24L01_REG_TX_ADDR     = 0x10,
	NRF24L01_REG_RX_PW_P0    = 0x11,
	NRF24L01_REG_RX_PW_P1    = 0x12,
	NRF24L01_REG_RX_PW_P2    = 0x13,
	NRF24L01_REG_RX_PW_P3    = 0x14,
	NRF24L01_REG_RX_PW_P4    = 0x15,
	NRF24L01_REG_RX_PW_P5    = 0x16,
	NRF24L01_REG_FIFO_STATUS = 0x17,
	NRF24L01_REG_DYNPD       = 0x1C,
	NRF24L01_REG_FEATURE     = 0x1D
	
}NRF24L01_RegTypeDef;

typedef enum /* NRF24L01+ Default Registers Value */
{
	
	NRF24L01_REG_DEF_CONFIG      = 0x08,
	NRF24L01_REG_DEF_EN_AA       = 0x3F,
	NRF24L01_REG_DEF_EN_RXADDR   = 0x03,
	NRF24L01_REG_DEF_SETUP_AW    = 0x03,
	NRF24L01_REG_DEF_SETUP_RETR  = 0x03,
	NRF24L01_REG_DEF_RF_CH       = 0x02,
	NRF24L01_REG_DEF_RF_SETUP    = 0x07,
	NRF24L01_REG_DEF_STATUS      = 0x7E,
	NRF24L01_REG_DEF_RX_ADDR_P0  = 0xE7,
	NRF24L01_REG_DEF_RX_ADDR_P1  = 0xC2,
	NRF24L01_REG_DEF_RX_ADDR_P2  = 0xC3,
	NRF24L01_REG_DEF_RX_ADDR_P3  = 0xC4,
	NRF24L01_REG_DEF_RX_ADDR_P4  = 0xC5,
	NRF24L01_REG_DEF_RX_ADDR_P5  = 0xC6,
	NRF24L01_REG_DEF_RX_PW_P0_P5 = 0x00,
	NRF24L01_REG_DEF_FIFO_STATUS = 0x11,
	NRF24L01_REG_DEF_DYNPD       = 0x00,
	NRF24L01_REG_DEF_FEATURE     = 0x00
	
}NRF24L01_RegRSTValTypeDef;

typedef enum /* NRF24L01+ Register bit's */
{
	
	NRF24L01_RX_DR               = 6,
	NRF24L01_TX_DS               = 5,
	NRF24L01_TX_DS_MASK          = 0x20,
	NRF24L01_MAX_RT              = 4,

	/* Config Register */
	NRF24L01_PRIM_RX             = 0,
	NRF24L01_PWR_UP              = 1,
	NRF24L01_CRCO                = 2,
	NRF24L01_CRCO_MASK_RST       = 0xFB,
	NRF24L01_EN_CRC              = 3,

	NRF24L01_ARC                 = 0,
	NRF24L01_ARC_MASK_RST        = 0xF0,
	NRF24L01_ARD                 = 4,
	NRF24L01_ARD_BIT_MASK_RST    = 0x0F,
	NRF24L01_RF_PWR              = 1,
	NRF24L01_RF_PWR_BIT_MASK_RST = 0xF9,
	NRF24L01_RF_DR_BIT_MASK_RST  = 0xD7,


	/* Enable Auto Acknowledgment */
	NRF24L01_ENAA_P5 = 5,
	NRF24L01_ENAA_P4 = 4,
	NRF24L01_ENAA_P3 = 3,
	NRF24L01_ENAA_P2 = 2,
	NRF24L01_ENAA_P1 = 1,
	NRF24L01_ENAA_P0 = 0,

	/* Enable Rx Addresses */
	NRF24L01_ERX_P5 = 5,
	NRF24L01_ERX_P4 = 4,
	NRF24L01_ERX_P3 = 3,
	NRF24L01_ERX_P2 = 2,
	NRF24L01_ERX_P1 = 1,
	NRF24L01_ERX_P0 = 0,
	
}NRF24L01_RegBitTypeDef;

/* ---------------------- NRF24L01 Parameter's --------------------- */
typedef enum /* NRF24L01+ Data Rate */
{
	
	NRF24L01_DATA_RATE_1MBPS   = 0,
	NRF24L01_DATA_RATE_2MBPS   = 1,
	NRF24L01_DATA_RATE_250KBPS = 2,
	
}NRF24L01_DataRateTypeDef;

typedef enum /* NRF24L01+ Output Power */
{
	
	NRF24L01_PWR_0DBM  = 3,
	NRF24L01_PWR_6DBM  = 2,
	NRF24L01_PWR_12DBM = 1,
	NRF24L01_PWR_18DBM = 0

}NRF24L01_PWRTypeDef;

typedef enum /* NRF24L01+ Enable Auto ACK */
{

	NRF24L01_AUTO_ACK_P0  = 0x01,
	NRF24L01_AUTO_ACK_P1  = 0x02,
	NRF24L01_AUTO_ACK_P2  = 0x04,
	NRF24L01_AUTO_ACK_P3  = 0x08,
	NRF24L01_AUTO_ACK_P4  = 0x10,
	NRF24L01_AUTO_ACK_P5  = 0x20,
	NRF24L01_AUTO_ACK_ALL = 0x3F,

}NRF24L01_AutoACKTypeDef;

typedef enum /* NRF24L01+ Enable Rx Addresses */
{

	NRF24L01_RX_ADDRESS_P0  = 0x01,
	NRF24L01_RX_ADDRESS_P1  = 0x02,
	NRF24L01_RX_ADDRESS_P2  = 0x04,
	NRF24L01_RX_ADDRESS_P3  = 0x08,
	NRF24L01_RX_ADDRESS_P4  = 0x10,
	NRF24L01_RX_ADDRESS_P5  = 0x20,
	NRF24L01_RX_ADDRESS_ALL = 0x3F,

}NRF24L01_RxAddTypeDef;

typedef enum /* NRF24L01+ States */
{

	NRF24L01_DISABLE = 0,
	NRF24L01_ENABLE  = 1
	
}NRF24L01_StateTypeDef;

/* ---------------------------- Common ----------------------------- */

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Enum ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Struct ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ......................... Initialize ........................ */
/*
 * Function: NRF24L01_RxInit
 * -------------------------
 * Initialize NRF24L01 as receiver
 *
 * Param  : 
 *         Channel  : RF Channel Frequency (1 to 125)
 *         DataRate : RF Air Data Rate (1 or 2Mbps)
 *         Timeout  : Timeout duration
 *         
 * Returns: 
 *         -
 *         
 * Example: 
 *         NRF24L01_RxInit(100, NRF24L01_DATA_RATE_1MBPS, 1000);
 *         ...
 *         
 */
void NRF24L01_RxInit(uint8_t Channel, NRF24L01_DataRateTypeDef DataRate, uint16_t Timeout);

/*
 * Function: NRF24L01_TxInit
 * -------------------------
 * Initialize NRF24L01 as transmitter
 *
 * Param  : 
 *         Channel  : RF Channel Frequency (1 to 125)
 *         DataRate : RF Air Data Rate (1 or 2Mbps)
 *         Timeout  : Timeout duration
 *         
 * Returns: 
 *         -
 *         
 * Example: 
 *         NRF24L01_TxInit(100, NRF24L01_DATA_RATE_1MBPS, 1000);
 *         ...
 *         
 */
void NRF24L01_TxInit(uint8_t Channel, NRF24L01_DataRateTypeDef DataRate, uint16_t Timeout);

/* .......................... Control .......................... */
/*
 * Function: NRF24L01_Reset
 * ------------------------
 * Reset NRF24L01
 *
 * Param  : 
 *         Timeout : Timeout duration
 *         
 * Returns: 
 *         -
 *         
 * Example: 
 *         NRF24L01_Reset(1000);
 *         ...
 *         
 */
void NRF24L01_Reset(uint16_t Timeout);

/* ........................ Primary Mode ....................... */
/*
 * Function: NRF24L01_PRxMode
 * --------------------------
 * Set NRF24L01 Primary Mode as Rx
 *
 * Param  : 
 *         Timeout : Timeout duration
 *         
 * Returns: 
 *         -
 *         
 * Example: 
 *         NRF24L01_PRxMode(1000);
 *         ...
 *         
 */
void NRF24L01_PRxMode(uint16_t Timeout);

/*
 * Function: NRF24L01_PTxMode
 * --------------------------
 * Set NRF24L01 Primary Mode as Tx
 *
 * Param  : 
 *         Timeout : Timeout duration
 *         
 * Returns: 
 *         -
 *         
 * Example: 
 *         NRF24L01_PTxMode(1000);
 *         ...
 *         
 */
void NRF24L01_PTxMode(uint16_t Timeout);

/* ...................... Transmit/Receive ..................... */
/*
 * Function: NRF24L01_RxReceive
 * ----------------------------
 * Read receive data from NRF24L01
 *
 * Param  : 
 *         pRxPayload      : Pointer to reception data buffer
 *         RxPayloadNumber : Pointer to receiver number (Data Pipe x)
 *         Timeout         : Timeout duration
 *         
 * Returns: 
 *         -
 *         
 * Example: 
 *         uint8_t RxNumber;
 *         uint8_t DataBuffer[NRF24L01_PAYLOAD_LENGTH];
 *         
 *         NRF24L01_RxReceive(DataBuffer, &RxNumber, 1000);
 *         ...
 *         
 */
void NRF24L01_RxReceive(uint8_t *pRxPayload, uint8_t *RxPayloadNumber, uint16_t Timeout);

/*
 * Function: NRF24L01_TxTransmit
 * -----------------------------
 * Send data with NRF24L01
 *
 * Param  : 
 *         pTxPayload : Pointer to transmission data buffer
 *         Timeout    : Timeout duration
 *         
 * Returns: 
 *         -
 *         
 * Example: 
 *         uint8_t DataBuffer[NRF24L01_PAYLOAD_LENGTH] = "HelloNRF";
 *         
 *         NRF24L01_TxTransmit(DataBuffer, 1000);
 *         ...
 *         
 */
void NRF24L01_TxTransmit(uint8_t* pTxPayload, uint16_t Timeout);

/* Rx/Tx Status Control */
/*
 * Function: NRF24L01_ClearRxDR
 * ----------------------------
 * Clear NRF24L01 Rx Data Transmit flag
 *
 * Param  : 
 *         Timeout : Timeout duration
 *         
 * Returns: 
 *         -
 *         
 * Example: 
 *         NRF24L01_ClearRxDR(1000);
 *         ...
 *         
 */
void NRF24L01_ClearRxDR(uint16_t Timeout);

/*
 * Function: NRF24L01_ClearTxDS
 * ----------------------------
 * Clear NRF24L01 Tx Data Sent flag
 *
 * Param  : 
 *         Timeout : Timeout duration
 *         
 * Returns: 
 *         -
 *         
 * Example: 
 *         NRF24L01_ClearTxDS(1000);
 *         ...
 *         
 */
void NRF24L01_ClearTxDS(uint16_t Timeout);

/*
 * Function: NRF24L01_ClearMaxRT
 * -----------------------------
 * Clear NRF24L01 MaxRT flag
 *
 * Param  : 
 *         Timeout : Timeout duration
 *         
 * Returns: 
 *         -
 *         
 * Example: 
 *         NRF24L01_ClearMaxRT(1000);
 *         ...
 *         
 */
void NRF24L01_ClearMaxRT(uint16_t Timeout);

/* ........................ FIFO Control ....................... */
/* Read/Write */
/*
 * Function: NRF24L01_ReadRxFIFO
 * -----------------------------
 * Read receive data from NRF24L01 FIFO
 *
 * Param  : 
 *         pRxPayload : Pointer to reception data buffer
 *         Timeout    : Timeout duration
 *         
 * Returns: 
 *         -
 *         
 * Example: 
 *         uint8_t DataBuffer[NRF24L01_PAYLOAD_LENGTH];
 *         
 *         NRF24L01_ReadRxFIFO(DataBuffer, 1000);
 *         ...
 *         
 */
uint8_t NRF24L01_ReadRxFIFO(uint8_t *pRxPayload, uint16_t Timeout);

/*
 * Function: NRF24L01_WriteTxFIFO
 * ------------------------------
 * Write data to NRF24L01 FIFO to transmit
 *
 * Param  : 
 *         pTxPayload : Pointer to transmission data buffer
 *         Timeout    : Timeout duration
 *         
 * Returns: 
 *         -
 *         
 * Example: 
 *         uint8_t DataBuffer[NRF24L01_PAYLOAD_LENGTH] = "HelloNRF";
 *         
 *         NRF24L01_WriteTxFIFO(DataBuffer, 1000);
 *         ...
 *         
 */
uint8_t NRF24L01_WriteTxFIFO(uint8_t *pTxPayload, uint16_t Timeout);

/* Flush */
/*
 * Function: NRF24L01_FlushRxFIFO
 * ------------------------------
 * Flush the NRF24L01 Rx FIFO
 *
 * Param  : 
 *         Timeout : Timeout duration
 *         
 * Returns: 
 *         -
 *         
 * Example: 
 *         NRF24L01_FlushRxFIFO(1000);
 *         ...
 *         
 */
void NRF24L01_FlushRxFIFO(uint16_t Timeout);

/*
 * Function: NRF24L01_FlushTxFIFO
 * ------------------------------
 * Flush the NRF24L01 Tx FIFO
 *
 * Param  : 
 *         Timeout : Timeout duration
 *         
 * Returns: 
 *         -
 *         
 * Example: 
 *         NRF24L01_FlushTxFIFO(1000);
 *         ...
 *         
 */
void NRF24L01_FlushTxFIFO(uint16_t Timeout);

/* Status */
/*
 * Function: NRF24L01_GetFIFOStatus
 * --------------------------------
 * Get the NRF24L01 FIFO status
 *
 * Param  : 
 *         Timeout : Timeout duration
 *         
 * Returns: 
 *         FIFO Status : FIFO Status Register value
 *         
 * Example: 
 *         uint8_t Status;
 *
 *         Status = NRF24L01_GetFIFOStatus(1000);
 *         ...
 *         
 */
uint8_t NRF24L01_GetFIFOStatus(uint16_t Timeout);

/* ........................... Power ........................... */
/*
 * Function: NRF24L01_PowerUp
 * --------------------------
 * Power Up the NRF24L01
 *
 * Param  : 
 *         Timeout : Timeout duration
 *         
 * Returns: 
 *         -
 *         
 * Example: 
 *         NRF24L01_PowerUp(1000);
 *         ...
 *         
 */
void NRF24L01_PowerUp(uint16_t Timeout);

/*
 * Function: NRF24L01_PowerDown
 * ----------------------------
 * Power Down the NRF24L01
 *
 * Param  : 
 *         Timeout : Timeout duration
 *         
 * Returns: 
 *         -
 *         
 * Example: 
 *         NRF24L01_PowerDown(1000);
 *         ...
 *         
 */
void NRF24L01_PowerDown(uint16_t Timeout);

/* ......................... RF Address ........................ */
/*
 * Function: NRF24L01_EnableRxAddress
 * ----------------------------------
 * Enable the NRF24L01 Rx Addresses
 *
 * Param  : 
 *         RxAddress : Rx Data Pipe 0 to 5 (NRF24L01_RX_ADDRESS_x)
 *         Timeout   : Timeout duration
 *         
 * Returns: 
 *         -
 *         
 * Example: 
 *         NRF24L01_EnableRxAddress(NRF24L01_RX_ADDRESS_P0, 1000);
 *         NRF24L01_EnableRxAddress(NRF24L01_RX_ADDRESS_ALL, 1000);
 *         ...
 *         
 */
void NRF24L01_EnableRxAddress(NRF24L01_RxAddTypeDef RxAddress, uint16_t Timeout);

/*
 * Function: NRF24L01_DisableRxAddress
 * -----------------------------------
 * Disable the NRF24L01 Rx Addresses
 *
 * Param  : 
 *         RxAddress : Rx Data Pipe 0 to 5 (NRF24L01_RX_ADDRESS_x)
 *         Timeout   : Timeout duration
 *         
 * Returns: 
 *         -
 *         
 * Example: 
 *         NRF24L01_DisableRxAddress(NRF24L01_RX_ADDRESS_P0, 1000);
 *         NRF24L01_DisableRxAddress(NRF24L01_RX_ADDRESS_ALL, 1000);
 *         ...
 *         
 */
void NRF24L01_DisableRxAddress(NRF24L01_RxAddTypeDef RxAddress, uint16_t Timeout);

/*
 * Function: NRF24L01_SetAddressWidths
 * -----------------------------------
 * Set the NRF24L01 Rx/Tx Address Width
 *
 * Param  : 
 *         Width   : Width of Rx/Tx address (2 to 5 Byte)
 *         Timeout : Timeout duration
 *         
 * Returns: 
 *         -
 *         
 * Example: 
 *         NRF24L01_SetAddressWidths(5, 1000);
 *         ...
 *         
 */
void NRF24L01_SetAddressWidths(uint8_t Width, uint16_t Timeout);

/*
 * Function: NRF24L01_SetRxAddress
 * -------------------------------
 * Set the NRF24L01 Rx Addresses
 *
 * Param  : 
 *         RxAddress : Rx Data Pipe 0 to 5 (NRF24L01_RX_ADDRESS_x)
 *         Address   : Pointer to address, address 0~1 width is set by NRF24L01_SetAddressWidths, addresses 2 to 5 have only 1B
 *         Timeout   : Timeout duration
 *         
 * Returns: 
 *         -
 *         
 * Example: 
 *         uint8_t rxAdd0[5] = {0xB3, 0xB4, 0xB5, 0xB6, 0xCD};
 *         uint8_t rxAdd1[5] = {0x78, 0x78, 0x78, 0x78, 0x78};
 *         
 *         // Receive address data pipe 2 to 5. Only LSB. MSBytes will be equal to RX_ADDR_P1
 *         uint8_t rxAdd2 = 0xF3; // Address 2 is equal to {rxAdd1[0], rxAdd1[1], rxAdd1[2], rxAdd1[3], 0xF3}
 *         uint8_t rxAdd5 = 0xAB; // Address 5 is equal to {rxAdd1[0], rxAdd1[1], rxAdd1[2], rxAdd1[3], 0xAB}
 *         
 *         NRF24L01_SetRxAddress(NRF24L01_RX_ADDRESS_P0, rxAdd0, 1000);  // 0xB3, 0xB4, 0xB5, 0xB6, 0xCD
 *         NRF24L01_SetRxAddress(NRF24L01_RX_ADDRESS_P1, rxAdd1, 1000);  // 0x78, 0x78, 0x78, 0x78, 0x78
 *         NRF24L01_SetRxAddress(NRF24L01_RX_ADDRESS_P2, &rxAdd2, 1000); // 0x78, 0x78, 0x78, 0x78, 0xF3
 *         NRF24L01_SetRxAddress(NRF24L01_RX_ADDRESS_P5, &rxAdd5, 1000); // 0x78, 0x78, 0x78, 0x78, 0xAB
 *         ...
 *         
 */
void NRF24L01_SetRxAddress(NRF24L01_RxAddTypeDef RxAddress, uint8_t *Address, uint16_t Timeout);

/*
 * Function: NRF24L01_SetTxAddress
 * -------------------------------
 * Set the NRF24L01 Tx Address
 *
 * Param  : 
 *         Address : Pointer to address, address width is set by NRF24L01_SetAddressWidths
 *         Timeout : Timeout duration
 *         
 * Returns: 
 *         -
 *         
 * Example: 
 *         uint8_t txAdd[5] = {0xB3, 0xB4, 0xB5, 0xB6, 0xCD};
 *         
 *         NRF24L01_SetTxAddress(txAdd, 1000);  // 0xB3, 0xB4, 0xB5, 0xB6, 0xCD
 *         ...
 *         
 */
void NRF24L01_SetTxAddress(uint8_t *Address, uint16_t Timeout);

/* ......................... RF Control ........................ */
/*
 * Function: NRF24L01_SetRxPayloadWidths
 * -------------------------------------
 * Set the NRF24L01 Rx Payload Width (Data Packet Size)
 *
 * Param  : 
 *         Width   : Payload Width (1 to 32 Byte)
 *         Timeout : Timeout duration
 *         
 * Returns: 
 *         -
 *         
 * Example: 
 *         NRF24L01_SetRxPayloadWidths(8, 1000);
 *         ...
 *         
 */
void NRF24L01_SetRxPayloadWidths(uint8_t Width, uint16_t Timeout);

/*
 * Function: NRF24L01_SetCRCLength
 * -------------------------------
 * Set the NRF24L01 CRC Length
 *
 * Param  : 
 *         Length  : CRC Length (1 or 2 Byte)
 *         Timeout : Timeout duration
 *         
 * Returns: 
 *         -
 *         
 * Example: 
 *         NRF24L01_SetCRCLength(1, 1000);
 *         ...
 *         
 */
void NRF24L01_SetCRCLength(uint8_t Length, uint16_t Timeout);

/*
 * Function: NRF24L01_SetAutoACK
 * -----------------------------
 * Enable/Disable the NRF24L01 Auto Acknowledgment
 *
 * Param  : 
 *         AACK    : Auto Acknowledgment Data Pipe 0 to 5 (NRF24L01_AUTO_ACK_x)
 *         State   : State of Auto Acknowledgment (NRF24L01_DISABLE / NRF24L01_ENABLE)
 *         Timeout : Timeout duration
 *         
 * Returns: 
 *         -
 *         
 * Example: 
 *         NRF24L01_SetAutoACK(NRF24L01_AUTO_ACK_ALL, NRF24L01_DISABLE, 1000);
 *         NRF24L01_SetAutoACK(NRF24L01_AUTO_ACK_P0, NRF24L01_ENABLE, 1000);
 *         ...
 *         
 */
void NRF24L01_SetAutoACK(NRF24L01_AutoACKTypeDef AACK, NRF24L01_StateTypeDef State, uint16_t Timeout);

/*
 * Function: NRF24L01_AutoRetransmitCount
 * --------------------------------------
 * Set the NRF24L01 Auto Retransmit Count
 *
 * Param  : 
 *         Count   : Number of Retransmit (0 to 15 Re-Transmit on fail of AA)
 *         Timeout : Timeout duration
 *         
 * Returns: 
 *         -
 *         
 * Example: 
 *         NRF24L01_AutoRetransmitCount(10, 1000);
 *         ...
 *         
 */
void NRF24L01_AutoRetransmitCount(uint8_t Count, uint16_t Timeout);

/*
 * Function: NRF24L01_AutoRetransmitDelay
 * --------------------------------------
 * Set the NRF24L01 Auto Retransmit Delay Time (uS)
 *
 * Param  : 
 *         DelayTimeUS : Delay Time of Retransmit (2500 to 4000uS)
 *         Timeout     : Timeout duration
 *         
 * Returns: 
 *         -
 *         
 * Example: 
 *         NRF24L01_AutoRetransmitDelay(2000, 1000);
 *         ...
 *         
 */
void NRF24L01_AutoRetransmitDelay(uint16_t DelayTimeUS, uint16_t Timeout);

/*
 * Function: NRF24L01_SetRFChannel
 * -------------------------------
 * Set the NRF24L01 RF Channel
 *
 * Param  : 
 *         Channel : Channel of RF Signal (0 to 125)
 *         Timeout : Timeout duration
 *         
 * Returns: 
 *         -
 *         
 * Example: 
 *         NRF24L01_SetRFChannel(100, 1000);
 *         ...
 *         
 */
void NRF24L01_SetRFChannel(uint8_t Channel, uint16_t Timeout);

/*
 * Function: NRF24L01_SetRFTxOutputPower
 * -------------------------------------
 * Set the NRF24L01 RF output power in Tx mode
 *
 * Param  : 
 *         dBm     : RF output power in Tx mode
 *         Timeout : Timeout duration
 *         
 * Returns: 
 *         -
 *         
 * Example: 
 *         NRF24L01_SetRFTxOutputPower(NRF24L01_PWR_0DBM, 1000);
 *         ...
 *         
 */
void NRF24L01_SetRFTxOutputPower(NRF24L01_PWRTypeDef dBm, uint16_t Timeout);

/*
 * Function: NRF24L01_SetRFAirDataRate
 * -----------------------------------
 * Set the NRF24L01 RF Air Data Rate (Mbps)
 *
 * Param  : 
 *         bps     : Air Data Rate (Mbps)
 *         Timeout : Timeout duration
 *         
 * Returns: 
 *         -
 *         
 * Example: 
 *         NRF24L01_SetRFAirDataRate(NRF24L01_DATA_RATE_1MBPS, 1000);
 *         ...
 *         
 */
void NRF24L01_SetRFAirDataRate(NRF24L01_DataRateTypeDef bps, uint16_t Timeout);

/* ......................... IRQ Handle ........................ */
/*
 * Function: NRF24L01_TxIRQHandle
 * ------------------------------
 * NRF24L01 Tx IRQ handling
 *
 * Param  : 
 *         Timeout : Timeout duration
 *         
 * Returns: 
 *         -
 *         
 * Example: 
 *         ISR(EXTI0) // External Interrupt 0 is connected to NRF24L01 IRQ pin
 *         {
 *             NRF24L01_TxIRQHandle(100);
 *         }
 *         ...
 *         
 */
void NRF24L01_TxIRQHandle(uint16_t Timeout);

/* ............................................................. */
/*
 * Function: NRF24L01_GetStatus
 * ----------------------------
 * Get the NRF24L01 Status Register Value
 *
 * Param  : 
 *         Timeout : Timeout duration
 *         
 * Returns: 
 *         Status : Status Register value
 *         
 * Example: 
 *         uint8_t Status;
 *
 *         Status = NRF24L01_GetStatus(1000);
 *         ...
 *         
 */
uint8_t NRF24L01_GetStatus(uint16_t Timeout);

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#endif /* __NRF24L01_H_ */
