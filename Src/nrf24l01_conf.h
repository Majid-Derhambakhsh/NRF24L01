/*
------------------------------------------------------------------------------
~ File   : nrf24l01_conf.h
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

#ifndef __NRF24L01_CFG_H_
#define __NRF24L01_CFG_H_

/* ~~~~~~~~~~~~~~ Required Headers ~~~~~~~~~~~~~ */
/* Driver-library for AVR */
//#include "gpio_unit.h"
//#include "spi_unit.h"

/* Driver-library for STM32 */
//#include "main.h"
//#include "spi.h"

/* ~~~~~~~~~~~~~~~ Configurations ~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~ SPI ~~~~~~~~~~~ */
#define NRF24L01_SPI                 hspi1

#define NRF24L01_SPI_CS_GPIO_PORT    GPIOB
#define NRF24L01_SPI_CS_GPIO_PIN     GPIO_PIN_12

#define NRF24L01_CE_GPIO_PORT        GPIOC
#define NRF24L01_CE_GPIO_PIN         GPIO_PIN_1

#define NRF24L01_IRQ_GPIO_PORT       GPIOC
#define NRF24L01_IRQ_GPIO_PIN        GPIO_PIN_13

/*
	Guide :
			NRF24L01_SPI              : Specifies the SPI peripheral
			
			NRF24L01_SPI_CS_GPIO_PORT : Specifies the CS (CSN) port
			NRF24L01_SPI_CS_GPIO_PIN  : Specifies the CS (CSN) pin
			
			NRF24L01_CE_GPIO_PORT     : Specifies the CE port
			NRF24L01_CE_GPIO_PIN      : Specifies the CE pin
			
			NRF24L01_IRQ_GPIO_PORT    : Specifies the IRQ port
			NRF24L01_IRQ_GPIO_PIN     : Specifies the IRQ pin
	
	Example:
			ARM:
				#define NRF24L01_SPI               hspi1

				#define NRF24L01_SPI_CS_GPIO_PORT  GPIOA
				#define NRF24L01_SPI_CS_GPIO_PIN   0
				
				#define NRF24L01_CE_GPIO_PORT      GPIOA
				#define NRF24L01_CE_GPIO_PIN       1
				
				#define NRF24L01_IRQ_GPIO_PORT     GPIOA
				#define NRF24L01_IRQ_GPIO_PIN      2
*/

/* ~~~~~~~~~~~~~~~~~~ Options ~~~~~~~~~~~~~~~~~~ */
#define NRF24L01_PAYLOAD_LENGTH   24 // 1B ~ 32B

/*
	Guide :
			NRF24L01_PAYLOAD_LENGTH : Length of NRF Payload (1B ~ 32B)
	
	Example:
			#define NRF24L01_PAYLOAD_LENGTH   8
*/

/* ~~~~~~~~ STM32 MCU ~~~~~~~ */
#define STM32F4

/*
	Guide   :
				#define STM32Xx : STM32 Family.
	Example :
				#define STM32F1
				#define STM32H7
				#define STM32L0
*/

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#endif /* __NRF24L01_CFG_H_ */
