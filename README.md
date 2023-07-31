![Banner](NRF24L01.png)
# NRF24L01 Library
Multi-Channel nRF24L01+ Library for ARM Cortex M (STM32) and AVR microcontrollers

## Release
### Version: 1.0.0

- #### Type: Embedded Software.

- #### Support :  
               - AVR series  
               - ARM STM32 series  

- #### Program Language: C

- #### Properties:
               - Support fully nrf24 configuration  
               - Support Interrupt
               - Support for 6 receiving addresses at the same time (6 Pipeline)

                 and more features...

## Overview 
### Initialization and de-initialization functions:
```c++
void NRF24L01_RxInit(uint8_t Channel, NRF24L01_DataRateTypeDef DataRate, uint16_t Timeout);
void NRF24L01_TxInit(uint8_t Channel, NRF24L01_DataRateTypeDef DataRate, uint16_t Timeout);
```  

### Operation functions:
```c++  
/* .......................... Control .......................... */
void NRF24L01_Reset(uint16_t Timeout);

/* ........................ Primary Mode ....................... */
void NRF24L01_PRxMode(uint16_t Timeout);
void NRF24L01_PTxMode(uint16_t Timeout);

/* ...................... Transmit/Receive ..................... */
void NRF24L01_RxReceive(uint8_t *pRxPayload, uint8_t *RxPayloadNumber, uint16_t Timeout);
void NRF24L01_TxTransmit(uint8_t* pTxPayload, uint16_t Timeout);

void NRF24L01_ClearRxDR(uint16_t Timeout);
void NRF24L01_ClearTxDS(uint16_t Timeout);
void NRF24L01_ClearMaxRT(uint16_t Timeout);

/* ........................ FIFO Control ....................... */
uint8_t NRF24L01_ReadRxFIFO(uint8_t *pRxPayload, uint16_t Timeout);
uint8_t NRF24L01_WriteTxFIFO(uint8_t *pTxPayload, uint16_t Timeout);
void NRF24L01_FlushRxFIFO(uint16_t Timeout);
void NRF24L01_FlushTxFIFO(uint16_t Timeout);
uint8_t NRF24L01_GetFIFOStatus(uint16_t Timeout);

/* ........................... Power ........................... */
void NRF24L01_PowerUp(uint16_t Timeout);
void NRF24L01_PowerDown(uint16_t Timeout);

/* ......................... RF Address ........................ */
void NRF24L01_EnableRxAddress(NRF24L01_RxAddTypeDef RxAddress, uint16_t Timeout);
void NRF24L01_DisableRxAddress(NRF24L01_RxAddTypeDef RxAddress, uint16_t Timeout);
void NRF24L01_SetAddressWidths(uint8_t Width, uint16_t Timeout);
void NRF24L01_SetRxAddress(NRF24L01_RxAddTypeDef RxAddress, uint8_t *Address, uint16_t Timeout);
void NRF24L01_SetTxAddress(uint8_t *Address, uint16_t Timeout);

/* ......................... RF Control ........................ */
void NRF24L01_SetRxPayloadWidths(uint8_t Width, uint16_t Timeout);
void NRF24L01_SetCRCLength(uint8_t Length, uint16_t Timeout);
void NRF24L01_SetAutoACK(NRF24L01_AutoACKTypeDef AACK, NRF24L01_StateTypeDef State, uint16_t Timeout);
void NRF24L01_AutoRetransmitCount(uint8_t Count, uint16_t Timeout);
void NRF24L01_AutoRetransmitDelay(uint16_t DelayTimeUS, uint16_t Timeout);
void NRF24L01_SetRFChannel(uint8_t Channel, uint16_t Timeout);
void NRF24L01_SetRFTxOutputPower(NRF24L01_PWRTypeDef dBm, uint16_t Timeout);
void NRF24L01_SetRFAirDataRate(NRF24L01_DataRateTypeDef bps, uint16_t Timeout);

/* ......................... IRQ Handle ........................ */
void NRF24L01_TxIRQHandle(uint16_t Timeout);
uint8_t NRF24L01_GetStatus(uint16_t Timeout);

``` 
### Macros:
```c++  
- None 
``` 

## Guide

#### This library can be used as follows:
#### 1.  Add nrf24l01.h header
#### 2.  Add required headers and set up SPI and GPIO in nrf24l01_conf.h header:
  ```c++
#include "spi.h"

#define NRF24L01_SPI                 hspi1

#define NRF24L01_SPI_CS_GPIO_PORT    GPIOA
#define NRF24L01_SPI_CS_GPIO_PIN     GPIO_PIN_4

#define NRF24L01_CE_GPIO_PORT        GPIOA
#define NRF24L01_CE_GPIO_PIN         GPIO_PIN_3

#define NRF24L01_IRQ_GPIO_PORT       GPIOA
#define NRF24L01_IRQ_GPIO_PIN        GPIO_PIN_2
  ``` 
#### 3.  Set payload length (1B ~ 32B) in nrf24l01_conf.h header, for example:
  ```c++
#define NRF24L01_PAYLOAD_LENGTH   24
  ``` 
#### 4.  Create a global buffer array with the payload length size, for example:      
  ```c++
  uint8_t Buffer[NRF24L01_PAYLOAD_LENGTH] = {0};
  ``` 
#### 5.  Initialize NRF24L01 and set Tx/Rx address, for example:      
* Functions:
  ```c++
  void NRF24L01_RxInit(uint8_t Channel, NRF24L01_DataRateTypeDef DataRate, uint16_t Timeout);
  void NRF24L01_TxInit(uint8_t Channel, NRF24L01_DataRateTypeDef DataRate, uint16_t Timeout);
  ```
* Parameters:  
     * Channel : RF Channel Frequency (1 to 125)
     * DataRate : RF Air Data Rate (1 or 2Mbps)
     * Timeout  : Timeout duration

* Example of Transmitter Mode:
  ```c++
  uint8_t TxAddress1[5] = {0xB3, 0xB4, 0xB5, 0xB6, 0xCD};

	NRF24L01_TxInit(100, NRF24L01_DATA_RATE_1MBPS, 2000);

	NRF24L01_SetTxAddress(TxAddress1, 2000);
  ```
  
* Example of Receiver Mode (Single Address):
  ```c++
  uint8_t RxAddress1[5] = {0xB3, 0xB4, 0xB5, 0xB6, 0xCD};

	NRF24L01_RxInit(100, NRF24L01_DATA_RATE_1MBPS, 2000);

  NRF24L01_SetRxAddress(NRF24L01_RX_ADDRESS_P0, RxAddress0, 2000);
  ```

* Example of Receiver Mode (Multiple Address):
  ```c++
  uint8_t RxAddress0[5] = {0x78, 0x78, 0x78, 0x78, 0x78};
  uint8_t RxAddress1[5] = {0xB3, 0xB4, 0xB5, 0xB6, 0xCD};
  uint8_t RxAddress2 = 0xF3; // Last byte of RxAddress1, the RxAddress2 is set to {0xB3, 0xB4, 0xB5, 0xB6, 0xF3}
  uint8_t RxAddress3 = 0xF2; // Last byte of RxAddress1, the RxAddress3 is set to {0xB3, 0xB4, 0xB5, 0xB6, 0xF2}
  uint8_t RxAddress4 = 0xF4; // Last byte of RxAddress1, the RxAddress4 is set to {0xB3, 0xB4, 0xB5, 0xB6, 0xF4}
  uint8_t RxAddress5 = 0xF1; // Last byte of RxAddress1, the RxAddress5 is set to {0xB3, 0xB4, 0xB5, 0xB6, 0xF1}

  NRF24L01_RxInit(100, NRF24L01_DATA_RATE_1MBPS, 2000);

  NRF24L01_SetRxAddress(NRF24L01_RX_ADDRESS_P0, RxAddress0, 2000);
  NRF24L01_SetRxAddress(NRF24L01_RX_ADDRESS_P1, RxAddress1, 2000);
  NRF24L01_SetRxAddress(NRF24L01_RX_ADDRESS_P2, &RxAddress2, 2000);
  NRF24L01_SetRxAddress(NRF24L01_RX_ADDRESS_P3, &RxAddress3, 2000);
  NRF24L01_SetRxAddress(NRF24L01_RX_ADDRESS_P4, &RxAddress4, 2000);
  NRF24L01_SetRxAddress(NRF24L01_RX_ADDRESS_P5, &RxAddress5, 2000);
  ``` 
#### 6.  Add Transmit/Receive function in External Interrupt Handler, for example:  
* Example of Transmitter Mode:
  ```c++
  void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
  {
	  if(GPIO_Pin == NRF24L01_IRQ_GPIO_PIN)
	  {
		  NRF24L01_TxIRQHandle(2000);
	  }
  }
  ```
  
* Example of Receiver Mode:
  ```c++
  void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
  {
    uint8_t RxNumber;
  
	  if(GPIO_Pin == NRF24L01_IRQ_GPIO_PIN)
	  {
		  NRF24L01_RxReceive(Buffer, &RxNumber, 2000);
	    sprintf(msg, "%s  From Pipe %d\n", Buffer, RxNumber);
      HAL_UART_Transmit(&hlpuart1, (uint8_t *)msg, strlen(msg), 100);
	  }
  }
  ```
  
#### 7.  Use the Transmit function in Tx Mode, for example:  
        
```c++
sprintf((char *)Buffer, "Transmitter Time:%07ld", (HAL_GetTick() / 1000));
NRF24L01_TxTransmit(Buffer, 2000);
```  
      
## Examples  

#### Example 1: Transmit MCU uptime
```c++  
#include "main.h"
#include <stdio.h>
#include "nrf24l01.h"

uint8_t TxBuffer[NRF24L01_PAYLOAD_LENGTH] = {0};

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == NRF24L01_IRQ_GPIO_PIN)
	{
		NRF24L01_TxIRQHandle(2000);
	}
}

int main(void)
{
	HW_Init();

	uint8_t TxAddress1[5] = {0xB3, 0xB4, 0xB5, 0xB6, 0xCD};

	NRF24L01_TxInit(100, NRF24L01_DATA_RATE_1MBPS, 2000);

	NRF24L01_SetTxAddress(TxAddress1, 2000);

	while (1)
	{
		sprintf((char *)TxBuffer, "Transmitter Time:%07ld", (HAL_GetTick() / 1000));
		NRF24L01_TxTransmit(TxBuffer, 2000);
		HAL_Delay(100);
	}
}
``` 
#### Example 2: Receive transmitted MCU uptime
```c++  
#include "main.h"
#include <stdio.h>
#include <string.h>
#include "nrf24l01.h"

char msg[50] = {0};

uint8_t RxNumber = 10;

uint8_t RxBuffer[NRF24L01_PAYLOAD_LENGTH] = { 0, };

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == NRF24L01_IRQ_GPIO_PIN)
	{
		NRF24L01_RxReceive(RxBuffer, &RxNumber, 2000);
		sprintf(msg, "%s  From Pipe %d\n", RxBuffer, RxNumber);
		HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 100);
	}
}

int main(void)
{
	HW_Init();

	uint8_t RxAddress0[5] = {0x78, 0x78, 0x78, 0x78, 0x78};
	uint8_t RxAddress1[5] = {0xB3, 0xB4, 0xB5, 0xB6, 0xCD};
	uint8_t RxAddress2 = 0xF3;
	uint8_t RxAddress3 = 0xF2;
	uint8_t RxAddress4 = 0xF4;
	uint8_t RxAddress5 = 0xF1;

	NRF24L01_RxInit(100, NRF24L01_DATA_RATE_1MBPS, 2000);

	NRF24L01_SetRxAddress(NRF24L01_RX_ADDRESS_P0, RxAddress0, 2000);
	NRF24L01_SetRxAddress(NRF24L01_RX_ADDRESS_P1, RxAddress1, 2000);
	NRF24L01_SetRxAddress(NRF24L01_RX_ADDRESS_P2, &RxAddress2, 2000);
	NRF24L01_SetRxAddress(NRF24L01_RX_ADDRESS_P3, &RxAddress3, 2000);
	NRF24L01_SetRxAddress(NRF24L01_RX_ADDRESS_P4, &RxAddress4, 2000);
	NRF24L01_SetRxAddress(NRF24L01_RX_ADDRESS_P5, &RxAddress5, 2000);

	while (1)
	{
		HAL_Delay(100);
	}
}
``` 

## Test Performed:
- [x] STM32F1  
- [x] STM32F4  
- [x] ESP32-WROOM  
- [ ] AVR  

### Developer: Majid Derhambakhsh
