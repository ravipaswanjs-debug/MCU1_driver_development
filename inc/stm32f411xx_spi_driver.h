/*
 * stm32f411x_spi_driver.h
 *
 * Created on: Jan 20, 2026
 * Author: ravi
 */
#include <stdint.h>
#ifndef INC_STM32F411X_SPI_DRIVER_H_
#define INC_STM32F411X_SPI_DRIVER_H_


typedef struct{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

typedef struct{
	SPI_RegDef_t *pSPIx;  /* This hold the base address of SPIx(x=0,1,2) Peripheral*/
	SPI_Config_t SPIConfig;
}SPI_Handle_t;

/*****************************************************
*          APIs supported by this driver headder file
******************************************************/

// Peripheral Clock Enable APP
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi);



//SPI Init and Deinit API definitions
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_Deinit(SPI_RegDef_t *pSPIx);

// SPI Data send and recieve APIs
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t pTxBuffer, uint32_t len);
void SPI_RecieveData(SPI_RegDef_t *pSPIx,uint8_t pRxBuffer, uint32_len);

//IRQ Coniguration and ISR handling
void SPI_IRQInterruptConfig(uint8_t IRQ_Number,uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQ_Number, uint32 IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/*
* SPI Device Modes Macros 
*/
#define SPI_DEVICE_MODE_MASTER			1
#define SPI_DEVICE_MODE_SLAVE 			0

/*
* SPI Possible Bus configs Macros
*/

#define SPI_BUS_CONFIG_FD			1
#define SPI_BUS_CONFIG_HD			2
#define SPI_BUS_CONFIG_STXRX		3

/*
* SPI Possible Clock Speed Macros
*/
#define SPI_SCLCK_SPEED_DIV2		0
#define SPI_SCLCK_SPEED_DIV4		1
#define SPI_SCLCK_SPEED_DIV8		2
#define SPI_SCLCK_SPEED_DIV16		3
#define SPI_SCLCK_SPEED_DIV32		4
#define SPI_SCLCK_SPEED_DIV64		5
#define SPI_SCLCK_SPEED_DIV128		6
#define SPI_SCLCK_SPEED_DIV256		7

/*
* SPI Possible data frame formats Macros
*/
#define SPI_DFF_8BITS				0
#define SPI_DFF_16BITS				1

/*
* @CPOL
*/
#define SPI_CPOL_HIGH				0
#define SPI_CPOL_LOW			 	1

/*
* @CPHA
*/
#define SPI_CPHA_HIGH				0
#define SPI_CPLA_LOW				1

/* SPI Software Slave Managment */

#define SPI_SSM_ENABLED				1
#define SPI_SSM_DISABLED			0













#endif /* INC_STM32F411X_SPI_DRIVER_H_ */
