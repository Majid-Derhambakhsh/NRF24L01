/*
------------------------------------------------------------------------------
~ File   : nrf24l01.c
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

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Include ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include "nrf24l01.h"

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ G Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ G Enum ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ G Struct ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ G Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
uint8_t NRF24L01_ReadRegister(uint8_t Reg, uint16_t Timeout)
{
	
	uint8_t cmd = NRF24L01_CMD_R_REGISTER | Reg;
	
	uint8_t status;
	uint8_t regVal;
	
	/* --------------- Set Chip Select -------------- */
	NRF24L01_GPIO_WritePin(NRF24L01_SPI_CS_GPIO_PORT, NRF24L01_SPI_CS_GPIO_PIN, NRF24L01_GPIO_PIN_RESET);
	
	/* ---------------- Receive Data ---------------- */
	NRF24L01_SPI_TransmitReceive(&cmd, &status, 1, Timeout);
	
	NRF24L01_SPI_Receive(&regVal, 1, Timeout);
	
	/* --------------- Set Chip Select -------------- */
	NRF24L01_GPIO_WritePin(NRF24L01_SPI_CS_GPIO_PORT, NRF24L01_SPI_CS_GPIO_PIN, NRF24L01_GPIO_PIN_SET);

	return regVal;
	
}

void NRF24L01_WriteRegister(uint8_t Reg, uint8_t Data, uint16_t Timeout)
{
	
	uint8_t cmd = NRF24L01_CMD_W_REGISTER | Reg;
	
	uint8_t status;
	
	/* --------------- Set Chip Select -------------- */
	NRF24L01_GPIO_WritePin(NRF24L01_SPI_CS_GPIO_PORT, NRF24L01_SPI_CS_GPIO_PIN, NRF24L01_GPIO_PIN_RESET);
	
	/* ---------------- Transmit Data --------------- */
	NRF24L01_SPI_TransmitReceive(&cmd, &status, 1, Timeout);

	NRF24L01_SPI_Transmit(&Data, 1, Timeout);

	/* --------------- Set Chip Select -------------- */
	NRF24L01_GPIO_WritePin(NRF24L01_SPI_CS_GPIO_PORT, NRF24L01_SPI_CS_GPIO_PIN, NRF24L01_GPIO_PIN_SET);

}

void NRF24L01_WriteRegisterMulti(uint8_t Reg, uint8_t *Data, uint16_t Size, uint16_t Timeout)
{
	
	uint8_t cmd = NRF24L01_CMD_W_REGISTER | Reg;
	
	uint8_t status;
	
	/* --------------- Set Chip Select -------------- */
	NRF24L01_GPIO_WritePin(NRF24L01_SPI_CS_GPIO_PORT, NRF24L01_SPI_CS_GPIO_PIN, NRF24L01_GPIO_PIN_RESET);
	
	/* ---------------- Transmit Data --------------- */
	NRF24L01_SPI_TransmitReceive(&cmd, &status, 1, Timeout);
	
	NRF24L01_SPI_Transmit(Data, Size, Timeout);
	
	/* --------------- Set Chip Select -------------- */
	NRF24L01_GPIO_WritePin(NRF24L01_SPI_CS_GPIO_PORT, NRF24L01_SPI_CS_GPIO_PIN, NRF24L01_GPIO_PIN_SET);
	
}

/* ......................... Initialize ........................ */
void NRF24L01_RxInit(uint8_t Channel, NRF24L01_DataRateTypeDef DataRate, uint16_t Timeout)
{
	
	/* ------------------ Reset NRF ----------------- */
	NRF24L01_Reset(Timeout);
	
	/* ----------- Set NRF Payload Length ----------- */
	NRF24L01_SetRxPayloadWidths(NRF24L01_PAYLOAD_LENGTH, Timeout);
	
	/* ------------- Set NRF Radio Param ------------ */
	NRF24L01_SetRFChannel(Channel, Timeout);
	NRF24L01_SetRFAirDataRate(DataRate, Timeout);
	NRF24L01_SetRFTxOutputPower(NRF24L01_PWR_0DBM, Timeout);
	
	/* ------------- Set NRF CRC Length ------------- */
	NRF24L01_SetCRCLength(1, Timeout);

	/* ---------------- Set Auto ACK ---------------- */
	NRF24L01_SetAutoACK(NRF24L01_AUTO_ACK_ALL, NRF24L01_ENABLE, Timeout);

	/* ------------- Enable Rx Addresses ------------ */
	NRF24L01_EnableRxAddress(NRF24L01_RX_ADDRESS_ALL, Timeout);
	
	/* ----------- Set NRF Address Width ------------ */
	NRF24L01_SetAddressWidths(NRF24L01_ADDRESS_WIDTH, Timeout);
	
	/* ----------- Set NRF Auto Retransmit ---------- */
	NRF24L01_AutoRetransmitCount(3, Timeout);
	NRF24L01_AutoRetransmitDelay(250, Timeout);

	/* ---------------- Set NRF Power --------------- */
	NRF24L01_PRxMode(Timeout);
	NRF24L01_PowerUp(Timeout);
	
	/* --------------- Set Chip Select -------------- */
	NRF24L01_GPIO_WritePin(NRF24L01_CE_GPIO_PORT, NRF24L01_CE_GPIO_PIN, NRF24L01_GPIO_PIN_SET);
	
}

void NRF24L01_TxInit(uint8_t Channel, NRF24L01_DataRateTypeDef DataRate, uint16_t Timeout)
{
	
	/* ------------------ Reset NRF ----------------- */
	NRF24L01_Reset(Timeout);
	
	/* ------------- Set NRF Radio Param ------------ */
	NRF24L01_SetRFChannel(Channel, Timeout);
	NRF24L01_SetRFAirDataRate(DataRate, Timeout);
	NRF24L01_SetRFTxOutputPower(NRF24L01_PWR_0DBM, Timeout);
	
	/* ------------- Set NRF CRC Length ------------- */
	NRF24L01_SetCRCLength(1, Timeout);
	
	/* ---------------- Set Auto ACK ---------------- */
	NRF24L01_SetAutoACK(NRF24L01_AUTO_ACK_ALL, NRF24L01_ENABLE, Timeout);

	/* ------------- Enable Rx Addresses ------------ */
	NRF24L01_EnableRxAddress(NRF24L01_RX_ADDRESS_ALL, Timeout);

	/* ----------- Set NRF Address Width ------------ */
	NRF24L01_SetAddressWidths(NRF24L01_ADDRESS_WIDTH, Timeout);
	
	/* ----------- Set NRF Auto Retransmit ---------- */
	NRF24L01_AutoRetransmitCount(3, Timeout);
	NRF24L01_AutoRetransmitDelay(250, Timeout);

	/* ---------------- Set NRF Power --------------- */
	NRF24L01_PTxMode(Timeout);
	NRF24L01_PowerUp(Timeout);
	
	/* --------------- Set Chip Select -------------- */
	NRF24L01_GPIO_WritePin(NRF24L01_CE_GPIO_PORT, NRF24L01_CE_GPIO_PIN, NRF24L01_GPIO_PIN_SET);
	
}

/* .......................... Control .......................... */
void NRF24L01_Reset(uint16_t Timeout)
{
	
	uint8_t data[NRF24L01_ADDRESS_WIDTH];
	
	/* ---------- Set Pin to default Select --------- */
	NRF24L01_GPIO_WritePin(NRF24L01_SPI_CS_GPIO_PORT, NRF24L01_SPI_CS_GPIO_PIN, NRF24L01_GPIO_PIN_SET);
	NRF24L01_GPIO_WritePin(NRF24L01_CE_GPIO_PORT, NRF24L01_CE_GPIO_PIN, NRF24L01_GPIO_PIN_RESET);
	
	/* ---------- Set Registers to default ---------- */
	NRF24L01_WriteRegister(NRF24L01_REG_CONFIG, NRF24L01_REG_DEF_CONFIG, Timeout);
	NRF24L01_WriteRegister(NRF24L01_REG_EN_AA, NRF24L01_REG_DEF_EN_AA, Timeout);
	NRF24L01_WriteRegister(NRF24L01_REG_EN_RXADDR, NRF24L01_REG_DEF_EN_RXADDR, Timeout);
	NRF24L01_WriteRegister(NRF24L01_REG_SETUP_AW, NRF24L01_REG_DEF_SETUP_AW, Timeout);
	NRF24L01_WriteRegister(NRF24L01_REG_SETUP_RETR, NRF24L01_REG_DEF_SETUP_RETR, Timeout);
	NRF24L01_WriteRegister(NRF24L01_REG_RF_CH, NRF24L01_REG_DEF_RF_CH, Timeout);
	NRF24L01_WriteRegister(NRF24L01_REG_RF_SETUP, NRF24L01_REG_DEF_RF_SETUP, Timeout);
	NRF24L01_WriteRegister(NRF24L01_REG_STATUS, NRF24L01_REG_DEF_STATUS, Timeout);
	
	/* Data Pipe 0 and Tx Address */
	data[0] = NRF24L01_REG_DEF_RX_ADDR_P0;
	data[1] = NRF24L01_REG_DEF_RX_ADDR_P0;
	data[2] = NRF24L01_REG_DEF_RX_ADDR_P0;
	data[3] = NRF24L01_REG_DEF_RX_ADDR_P0;
	data[4] = NRF24L01_REG_DEF_RX_ADDR_P0;
	
	NRF24L01_WriteRegisterMulti(NRF24L01_REG_RX_ADDR_P0, data, NRF24L01_ADDRESS_WIDTH, Timeout);
	NRF24L01_WriteRegisterMulti(NRF24L01_REG_TX_ADDR, data, NRF24L01_ADDRESS_WIDTH, Timeout);
	
	/* Data Pipe 1 */
	data[0] = NRF24L01_REG_DEF_RX_ADDR_P1;
	data[1] = NRF24L01_REG_DEF_RX_ADDR_P1;
	data[2] = NRF24L01_REG_DEF_RX_ADDR_P1;
	data[3] = NRF24L01_REG_DEF_RX_ADDR_P1;
	data[4] = NRF24L01_REG_DEF_RX_ADDR_P1;
	
	NRF24L01_WriteRegisterMulti(NRF24L01_REG_RX_ADDR_P1, data, NRF24L01_ADDRESS_WIDTH, Timeout);
	
	/* Data Pipe 2 to 5 */
	NRF24L01_WriteRegister(NRF24L01_REG_RX_ADDR_P2, NRF24L01_REG_DEF_RX_ADDR_P2, Timeout);
	NRF24L01_WriteRegister(NRF24L01_REG_RX_ADDR_P3, NRF24L01_REG_DEF_RX_ADDR_P3, Timeout);
	NRF24L01_WriteRegister(NRF24L01_REG_RX_ADDR_P4, NRF24L01_REG_DEF_RX_ADDR_P4, Timeout);
	NRF24L01_WriteRegister(NRF24L01_REG_RX_ADDR_P5, NRF24L01_REG_DEF_RX_ADDR_P5, Timeout);
	
	NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P0, NRF24L01_REG_DEF_RX_PW_P0_P5, Timeout);
	NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P1, NRF24L01_REG_DEF_RX_PW_P0_P5, Timeout);
	NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P2, NRF24L01_REG_DEF_RX_PW_P0_P5, Timeout);
	NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P3, NRF24L01_REG_DEF_RX_PW_P0_P5, Timeout);
	NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P4, NRF24L01_REG_DEF_RX_PW_P0_P5, Timeout);
	NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P5, NRF24L01_REG_DEF_RX_PW_P0_P5, Timeout);
	NRF24L01_WriteRegister(NRF24L01_REG_FIFO_STATUS, NRF24L01_REG_DEF_FIFO_STATUS, Timeout);
	NRF24L01_WriteRegister(NRF24L01_REG_DYNPD, NRF24L01_REG_DEF_DYNPD, Timeout);
	NRF24L01_WriteRegister(NRF24L01_REG_FEATURE, NRF24L01_REG_DEF_FEATURE, Timeout);
	
	/* ----------------- Reset FIFO ----------------- */
	NRF24L01_FlushRxFIFO(Timeout);
	NRF24L01_FlushTxFIFO(Timeout);
	
}

/* ........................ Primary Mode ....................... */
void NRF24L01_PRxMode(uint16_t Timeout)
{
	
	/* ------------ Read Config Register ------------ */
	uint8_t regVal = NRF24L01_ReadRegister(NRF24L01_REG_CONFIG, Timeout);
	
	/* ----------- Set Mode to Primary Rx ----------- */
	__NRF24L01_SET_BIT(regVal, NRF24L01_PRIM_RX);
	
	NRF24L01_WriteRegister(NRF24L01_REG_CONFIG, regVal, Timeout);
	
}

void NRF24L01_PTxMode(uint16_t Timeout)
{
	
	/* ------------ Read Config Register ------------ */
	uint8_t regVal = NRF24L01_ReadRegister(NRF24L01_REG_CONFIG, Timeout);
	
	/* ----------- Set Mode to Primary Tx ----------- */
	__NRF24L01_RESET_BIT(regVal, NRF24L01_PRIM_RX);
	
	NRF24L01_WriteRegister(NRF24L01_REG_CONFIG, regVal, Timeout);
	
}

/* ...................... Transmit/Receive ..................... */
void NRF24L01_RxReceive(uint8_t *pRxPayload, uint8_t *RxPayloadNumber, uint16_t Timeout)
{
	
	/* ------------- Read Received Data ------------- */
	*RxPayloadNumber = (NRF24L01_ReadRegister(NRF24L01_REG_STATUS, Timeout) >> 1) & 0x07;
	NRF24L01_ReadRxFIFO(pRxPayload, Timeout);
	
	/* ----------------- Clear bit ------------------ */
	NRF24L01_ClearRxDR(Timeout);
	
}

void NRF24L01_TxTransmit(uint8_t *pTxPayload, uint16_t Timeout)
{
	NRF24L01_WriteTxFIFO(pTxPayload, Timeout);
}

/* Rx/Tx Status Control */
void NRF24L01_ClearRxDR(uint16_t Timeout)
{
	
	/* ------------ Read Status Register ------------ */
	uint8_t status = NRF24L01_GetStatus(Timeout);
	
	/* ----------- Clear Rx Data Ready bit ---------- */
	__NRF24L01_SET_BIT(status, NRF24L01_RX_DR);
	
	NRF24L01_WriteRegister(NRF24L01_REG_STATUS, status, Timeout);
	
}

void NRF24L01_ClearTxDS(uint16_t Timeout)
{
	
	/* ------------ Read Status Register ------------ */
	uint8_t status = NRF24L01_GetStatus(Timeout);
	
	/* ----------- Clear Tx Data Sent bit ----------- */
	__NRF24L01_SET_BIT(status, NRF24L01_TX_DS);
	
	NRF24L01_WriteRegister(NRF24L01_REG_STATUS, status, Timeout);
	
}

void NRF24L01_ClearMaxRT(uint16_t Timeout)
{
	
	/* ------------ Read Status Register ------------ */
	uint8_t status = NRF24L01_GetStatus(Timeout);
	
	/* ----------- Clear Tx Data Sent bit ----------- */
	__NRF24L01_SET_BIT(status, NRF24L01_MAX_RT);
	
	NRF24L01_WriteRegister(NRF24L01_REG_STATUS, status, Timeout);
	
}

/* ........................ FIFO Control ....................... */
/* Read/Write */
uint8_t NRF24L01_ReadRxFIFO(uint8_t *pRxPayload, uint16_t Timeout)
{
	
	uint8_t cmd = NRF24L01_CMD_R_RX_PAYLOAD;
	
	uint8_t status;
	
	/* --------------- Set Chip Select -------------- */
	NRF24L01_GPIO_WritePin(NRF24L01_SPI_CS_GPIO_PORT, NRF24L01_SPI_CS_GPIO_PIN, NRF24L01_GPIO_PIN_RESET);
	
	/* ---------------- Receive Data ---------------- */
	NRF24L01_SPI_TransmitReceive(&cmd, &status, 1, Timeout);
	
	NRF24L01_SPI_Receive(pRxPayload, NRF24L01_PAYLOAD_LENGTH, Timeout);
	
	/* --------------- Set Chip Select -------------- */
	NRF24L01_GPIO_WritePin(NRF24L01_SPI_CS_GPIO_PORT, NRF24L01_SPI_CS_GPIO_PIN, NRF24L01_GPIO_PIN_SET);
	
	return status;
	
}

uint8_t NRF24L01_WriteTxFIFO(uint8_t *pTxPayload, uint16_t Timeout)
{
	
	uint8_t cmd = NRF24L01_CMD_W_TX_PAYLOAD;
	
	uint8_t status;
	
	/* --------------- Set Chip Select -------------- */
	NRF24L01_GPIO_WritePin(NRF24L01_SPI_CS_GPIO_PORT, NRF24L01_SPI_CS_GPIO_PIN, NRF24L01_GPIO_PIN_RESET);
	
	/* ---------------- Transmit Data --------------- */
	NRF24L01_SPI_TransmitReceive(&cmd, &status, 1, Timeout);
	
	NRF24L01_SPI_Transmit(pTxPayload, NRF24L01_PAYLOAD_LENGTH, Timeout);
	
	/* --------------- Set Chip Select -------------- */
	NRF24L01_GPIO_WritePin(NRF24L01_SPI_CS_GPIO_PORT, NRF24L01_SPI_CS_GPIO_PIN, NRF24L01_GPIO_PIN_SET);

	return status;
	
}

/* Flush */
void NRF24L01_FlushRxFIFO(uint16_t Timeout)
{
	
	uint8_t cmd = NRF24L01_CMD_FLUSH_RX;
	
	uint8_t status;
	
	/* --------------- Set Chip Select -------------- */
	NRF24L01_GPIO_WritePin(NRF24L01_SPI_CS_GPIO_PORT, NRF24L01_SPI_CS_GPIO_PIN, NRF24L01_GPIO_PIN_RESET);
	
	/* ---------------- Transmit Data --------------- */
	NRF24L01_SPI_TransmitReceive(&cmd, &status, 1, Timeout);
	
	/* --------------- Set Chip Select -------------- */
	NRF24L01_GPIO_WritePin(NRF24L01_SPI_CS_GPIO_PORT, NRF24L01_SPI_CS_GPIO_PIN, NRF24L01_GPIO_PIN_SET);
	
}

void NRF24L01_FlushTxFIFO(uint16_t Timeout)
{
	
	uint8_t cmd = NRF24L01_CMD_FLUSH_TX;
	
	uint8_t status;

	/* --------------- Set Chip Select -------------- */
	NRF24L01_GPIO_WritePin(NRF24L01_SPI_CS_GPIO_PORT, NRF24L01_SPI_CS_GPIO_PIN, NRF24L01_GPIO_PIN_RESET);
	
	/* ---------------- Transmit Data --------------- */
	NRF24L01_SPI_TransmitReceive(&cmd, &status, 1, Timeout);
	
	/* --------------- Set Chip Select -------------- */
	NRF24L01_GPIO_WritePin(NRF24L01_SPI_CS_GPIO_PORT, NRF24L01_SPI_CS_GPIO_PIN, NRF24L01_GPIO_PIN_SET);
	
}

/* Status */
uint8_t NRF24L01_GetFIFOStatus(uint16_t Timeout)
{
	return NRF24L01_ReadRegister(NRF24L01_REG_FIFO_STATUS, Timeout);
}

/* ........................... Power ........................... */
void NRF24L01_PowerUp(uint16_t Timeout)
{
	
	/* ------------ Read Config Register ------------ */
	uint8_t regVal = NRF24L01_ReadRegister(NRF24L01_REG_CONFIG, Timeout);
	
	/* ------------ Set Chip to Power Up ------------ */
	__NRF24L01_SET_BIT(regVal, NRF24L01_PWR_UP);
	
	NRF24L01_WriteRegister(NRF24L01_REG_CONFIG, regVal, Timeout);
	
}

void NRF24L01_PowerDown(uint16_t Timeout)
{
	
	/* ------------ Read Config Register ------------ */
	uint8_t regVal = NRF24L01_ReadRegister(NRF24L01_REG_CONFIG, Timeout);
	
	/* ------------ Set Chip to Power Up ------------ */
	__NRF24L01_RESET_BIT(regVal, NRF24L01_PWR_UP);

	NRF24L01_WriteRegister(NRF24L01_REG_CONFIG, regVal, Timeout);

}

/* ......................... RF Address ........................ */
void NRF24L01_EnableRxAddress(NRF24L01_RxAddTypeDef RxAddress, uint16_t Timeout)
{

	/* ------------ Read Config Register ------------ */
	uint8_t regVal = NRF24L01_ReadRegister(NRF24L01_REG_EN_RXADDR, Timeout) | RxAddress;

	/* -------------- Update Auto ACK --------------- */
	NRF24L01_WriteRegister(NRF24L01_REG_EN_RXADDR, regVal, Timeout);

}

void NRF24L01_DisableRxAddress(NRF24L01_RxAddTypeDef RxAddress, uint16_t Timeout)
{

	/* ------------ Read Config Register ------------ */
	uint8_t regVal = NRF24L01_ReadRegister(NRF24L01_REG_EN_RXADDR, Timeout) & (~RxAddress);

	/* -------------- Update Auto ACK --------------- */
	NRF24L01_WriteRegister(NRF24L01_REG_EN_RXADDR, regVal, Timeout);

}

void NRF24L01_SetAddressWidths(uint8_t Width, uint16_t Timeout)
{
	NRF24L01_WriteRegister(NRF24L01_REG_SETUP_AW, (Width - 2), Timeout);
}

void NRF24L01_SetRxAddress(NRF24L01_RxAddTypeDef RxAddress, uint8_t *Address, uint16_t Timeout) // My Address
{

	uint8_t _addressIndx = 0;

	uint8_t _address[NRF24L01_ADDRESS_WIDTH];

	/* --------------- Set Chip Enable -------------- */
	NRF24L01_GPIO_WritePin(NRF24L01_CE_GPIO_PORT, NRF24L01_CE_GPIO_PIN, NRF24L01_GPIO_PIN_RESET);

	/* --------------- Update Address --------------- */
	switch (RxAddress)
	{
		case NRF24L01_RX_ADDRESS_P0:
		{

			for (; _addressIndx < NRF24L01_ADDRESS_WIDTH; _addressIndx++)
			{
				_address[_addressIndx] = Address[NRF24L01_ADDRESS_WIDTH - _addressIndx - 1];
			}

			NRF24L01_WriteRegisterMulti(NRF24L01_REG_RX_ADDR_P0, _address, NRF24L01_ADDRESS_WIDTH, Timeout);

		}
		break;
		case NRF24L01_RX_ADDRESS_P1:
		{

			for (; _addressIndx < NRF24L01_ADDRESS_WIDTH; _addressIndx++)
			{
				_address[_addressIndx] = Address[NRF24L01_ADDRESS_WIDTH - _addressIndx - 1];
			}

			NRF24L01_WriteRegisterMulti(NRF24L01_REG_RX_ADDR_P1, _address, NRF24L01_ADDRESS_WIDTH, Timeout);

		}
		break;
		case NRF24L01_RX_ADDRESS_P2:
		{
			NRF24L01_WriteRegister(NRF24L01_REG_RX_ADDR_P2, Address[0], Timeout);
		}
		break;
		case NRF24L01_RX_ADDRESS_P3:
		{
			NRF24L01_WriteRegister(NRF24L01_REG_RX_ADDR_P3, Address[0], Timeout);
		}
		break;
		case NRF24L01_RX_ADDRESS_P4:
		{
			NRF24L01_WriteRegister(NRF24L01_REG_RX_ADDR_P4, Address[0], Timeout);
		}
		break;
		case NRF24L01_RX_ADDRESS_P5:
		{
			NRF24L01_WriteRegister(NRF24L01_REG_RX_ADDR_P5, Address[0], Timeout);
		}
		break;
		default:
		break;
	}

	/* --------------- Set Chip Enable -------------- */
	NRF24L01_GPIO_WritePin(NRF24L01_CE_GPIO_PORT, NRF24L01_CE_GPIO_PIN, NRF24L01_GPIO_PIN_SET);

}

void NRF24L01_SetTxAddress(uint8_t *Address, uint16_t Timeout)
{
	
	uint8_t _addressIndx = 0;

	uint8_t _address[NRF24L01_ADDRESS_WIDTH];

	/* --------------- Update Address --------------- */
	for (; _addressIndx < NRF24L01_ADDRESS_WIDTH; _addressIndx++)
	{
		_address[_addressIndx] = Address[NRF24L01_ADDRESS_WIDTH - _addressIndx - 1];
	}

	NRF24L01_WriteRegisterMulti(NRF24L01_REG_RX_ADDR_P0, _address, NRF24L01_ADDRESS_WIDTH, Timeout);
	NRF24L01_WriteRegisterMulti(NRF24L01_REG_TX_ADDR, _address, NRF24L01_ADDRESS_WIDTH, Timeout);
	
}

/* ......................... RF Control ........................ */
void NRF24L01_SetRxPayloadWidths(uint8_t Width, uint16_t Timeout)
{
	NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P0, Width, Timeout);
	NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P1, Width, Timeout);
	NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P2, Width, Timeout);
	NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P3, Width, Timeout);
	NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P4, Width, Timeout);
	NRF24L01_WriteRegister(NRF24L01_REG_RX_PW_P5, Width, Timeout);
}

void NRF24L01_SetCRCLength(uint8_t Length, uint16_t Timeout)
{
	
	/* ------------ Read Config Register ------------ */
	uint8_t regVal = NRF24L01_ReadRegister(NRF24L01_REG_CONFIG, Timeout) & NRF24L01_CRCO_MASK_RST;

	__NRF24L01_SET_BIT(regVal, NRF24L01_EN_CRC);
	
	/* ------------- Update CRC Length -------------- */
	switch (Length)
	{
		case 1:
		{
			__NRF24L01_RESET_BIT(regVal, NRF24L01_CRCO); // 1Byte
		}
		break;
		case 2:
		{
			__NRF24L01_SET_BIT(regVal, NRF24L01_CRCO); // 2Byte
		}
		break;
		default:
		break;
	}
	
	NRF24L01_WriteRegister(NRF24L01_REG_CONFIG, regVal, Timeout);
	
}

void NRF24L01_SetAutoACK(NRF24L01_AutoACKTypeDef AACK, NRF24L01_StateTypeDef State, uint16_t Timeout)
{

	/* ------------ Read Config Register ------------ */
	uint8_t regVal = NRF24L01_ReadRegister(NRF24L01_REG_EN_AA, Timeout);

	/* -------------- Update Auto ACK --------------- */
	if (State == NRF24L01_ENABLE)
	{
		regVal |= AACK;
	}
	else
	{
		regVal &= ~AACK;
	}

	NRF24L01_WriteRegister(NRF24L01_REG_EN_AA, regVal, Timeout);

}

void NRF24L01_AutoRetransmitCount(uint8_t Count, uint16_t Timeout)
{
	
	/* ------- Read Setup Retransmit Register ------- */
	uint8_t regVal = NRF24L01_ReadRegister(NRF24L01_REG_SETUP_RETR, Timeout) & NRF24L01_ARC_MASK_RST;
	
	/* ------- Reset and Update ARC Register -------- */
	regVal |= Count;
	
	NRF24L01_WriteRegister(NRF24L01_REG_SETUP_RETR, regVal, Timeout);
	
}

void NRF24L01_AutoRetransmitDelay(uint16_t DelayTimeUS, uint16_t Timeout)
{
	
	/* ------- Read Setup Retransmit Register ------- */
	uint8_t regVal = NRF24L01_ReadRegister(NRF24L01_REG_SETUP_RETR, Timeout) & NRF24L01_ARD_BIT_MASK_RST;
	
	/* ------- Reset and Update ARD Register -------- */
	
	regVal |= (((DelayTimeUS / 250) - 1) << NRF24L01_ARD);
	
	NRF24L01_WriteRegister(NRF24L01_REG_SETUP_RETR, regVal, Timeout);
	
}

void NRF24L01_SetRFChannel(uint8_t Channel, uint16_t Timeout)
{
	
	/* ------------- Update NRF Channel ------------- */
	NRF24L01_WriteRegister(NRF24L01_REG_RF_CH, Channel, Timeout);
	
}

void NRF24L01_SetRFTxOutputPower(NRF24L01_PWRTypeDef dBm, uint16_t Timeout)
{
	
	/* ----------- Read RF Setup Register ----------- */
	uint8_t regVal = NRF24L01_ReadRegister(NRF24L01_REG_RF_SETUP, Timeout) & NRF24L01_RF_PWR_BIT_MASK_RST;
	
	/* ---------- Reset and Update RF Power --------- */
	regVal |= (dBm << NRF24L01_RF_PWR);
	
	NRF24L01_WriteRegister(NRF24L01_REG_RF_SETUP, regVal, Timeout);
	
}

void NRF24L01_SetRFAirDataRate(NRF24L01_DataRateTypeDef bps, uint16_t Timeout)
{
	
	/* ----------- Read RF Setup Register ----------- */
	uint8_t regVal = NRF24L01_ReadRegister(NRF24L01_REG_RF_SETUP, Timeout) & NRF24L01_RF_DR_BIT_MASK_RST;
	
	/* ------- Reset and Update RF Data Rate -------- */
	switch (bps)
	{
		case NRF24L01_DATA_RATE_1MBPS:
		break;
		case NRF24L01_DATA_RATE_2MBPS:
		{
			regVal |= (1 << 3);
		}
		break;
		case NRF24L01_DATA_RATE_250KBPS:
		{
			regVal |= (1 << 5);
		}
		break;
		default:
		break;
	}
	
	NRF24L01_WriteRegister(NRF24L01_REG_RF_SETUP, regVal, Timeout);
	
}

/* ......................... IRQ Handle ........................ */
void NRF24L01_TxIRQHandle(uint16_t Timeout)
{
	
	/* ------------ Read Status Register ------------ */
	uint8_t txDataSent = NRF24L01_GetStatus(Timeout) & NRF24L01_TX_DS_MASK;
	
	/* ------------- Check Data Sent bit ------------ */
	// __NRF24L01_MASK_BIT(txDataSent, NRF24L01_TX_DS_MASK); // __CHECK_BIT(txDataSent, NRF24L01_TX_DS_BIT)
	
	if(txDataSent)
	{
		// TX_DS
		NRF24L01_ClearTxDS(Timeout);
	}
	else
	{
		// MAX_RT
		NRF24L01_ClearMaxRT(Timeout);
	}
	
}

/* ............................................................. */
uint8_t NRF24L01_GetStatus(uint16_t Timeout)
{
	
	uint8_t cmd = NRF24L01_CMD_NOP;
	
	uint8_t status;
	
	/* --------------- Set Chip Select -------------- */
	NRF24L01_GPIO_WritePin(NRF24L01_SPI_CS_GPIO_PORT, NRF24L01_SPI_CS_GPIO_PIN, NRF24L01_GPIO_PIN_RESET);
	
	/* ---------------- Receive Data ---------------- */
	NRF24L01_SPI_TransmitReceive(&cmd, &status, 1, 2000);
	
	/* --------------- Set Chip Select -------------- */
	NRF24L01_GPIO_WritePin(NRF24L01_SPI_CS_GPIO_PORT, NRF24L01_SPI_CS_GPIO_PIN, NRF24L01_GPIO_PIN_SET);
	
	return status;
	
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
