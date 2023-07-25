# NRF24L01 Library
Multi-Channel nRF24L01+ Library for ARM Cortex M (STM32) and AVR microcontrollers

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

## How to use this library

## Test Performed:
- [x] STM32F1  
- [x] STM32F4  
- [x] ESP32-WROOM  
- [x] AVR  

#### Developer: Majid Derhambakhsh
