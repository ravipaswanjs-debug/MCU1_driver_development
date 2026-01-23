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
void SPI_Init(SPI_Handle_t *pSPIHandle)
void SPI_Deinit(SPI_RegDef_t*pSPIx


/*
* SPI Device Modes Macros 
*/
#define SPI_MODE_MASTER			1
#define SPI_MODE_SLAVE 			0

/*
* SPI Possible Bus configs Macros
*/

#define SPI_FULL_DUPLEX			0
#define SPI_HALF_DUPLEX			1
#define SPI_SIMPLEX_RXTX		2

/*
* SPI Possible Clock Speed Macros
*/
#define SPI_FCLCK_2				0
#define SPI_FCLCK_4				0
#define SPI_FCLCK_8				0
#define SPI_FCLCK_16			0
#define SPI_FCLCK_32			0
#define SPI_FCLCK_64			0
#define SPI_FCLCK_128			0
#define SPI_FCLCK_256			0

/*
* SPI Possible data frame formats Macros
*/
#define SPI_DFF_8				0
#define SPI_DFF_16				1



















#endif /* INC_STM32F411X_SPI_DRIVER_H_ */
