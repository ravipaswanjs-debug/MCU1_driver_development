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


#endif /* INC_STM32F411X_SPI_DRIVER_H_ */
