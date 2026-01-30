/*
 * stm32f411xx.h
 *
 * Created on: Jan 17, 2026
 * Author: ravi
 */
#include <stdint.h>
#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#define __vo volatile

/*
 * Memory Base Addresses for STM32F411
 */

#define FLASH_BASEADDR          0x08000000U       /*!< Start address of Flash memory */
#define SRAM1_BASEADDR          0x20000000U       /*!< Start address of SRAM (128KB on F411) */
#define ROM_BASEADDR            0x1FFF0000U       /*!< Start address of System Memory (Bootloader) */
#define SRAM                    SRAM1_BASEADDR    /*!< Main SRAM alias */

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR         0x40000000U
#define APB1PERIPH_BASEADDR     PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR     0x40010000U
#define AHB1PERIPH_BASEADDR     0x40020000U
#define AHB2PERIPH_BASEADDR     0x50000000U       /*!< USB OTG FS is on AHB2 */

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 * Note: STM32F411 only supports GPIO Ports A, B, C, D, E, and H.
 */

#define GPIOA_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR          (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR          (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOH_BASEADDR          (AHB1PERIPH_BASEADDR + 0x1C00)

#define CRC_BASEADDR            (AHB1PERIPH_BASEADDR + 0x3000)
#define RCC_BASEADDR            (AHB1PERIPH_BASEADDR + 0x3800)
#define DMA1_BASEADDR           (AHB1PERIPH_BASEADDR + 0x6000)
#define DMA2_BASEADDR           (AHB1PERIPH_BASEADDR + 0x6400)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */

#define TIM2_BASEADDR           (APB1PERIPH_BASEADDR + 0x0000)
#define TIM3_BASEADDR           (APB1PERIPH_BASEADDR + 0x0400)
#define TIM4_BASEADDR           (APB1PERIPH_BASEADDR + 0x0800)
#define TIM5_BASEADDR           (APB1PERIPH_BASEADDR + 0x0C00)

#define WWDG_BASEADDR           (APB1PERIPH_BASEADDR + 0x2C00)
#define SPI2_BASEADDR           (APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR           (APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR         (APB1PERIPH_BASEADDR + 0x4400) // F411 only has USART1, 2, 6

#define I2C1_BASEADDR           (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR           (APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR           (APB1PERIPH_BASEADDR + 0x5C00)

#define PWR_BASEADDR            (APB1PERIPH_BASEADDR + 0x7000)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */

#define TIM1_BASEADDR           (APB2PERIPH_BASEADDR + 0x0000)
#define USART1_BASEADDR         (APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR         (APB2PERIPH_BASEADDR + 0x1400)
#define ADC1_BASEADDR           (APB2PERIPH_BASEADDR + 0x2000) // F411 only has ADC1
#define SDIO_BASEADDR           (APB2PERIPH_BASEADDR + 0x2C00)

#define SPI1_BASEADDR           (APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR           (APB2PERIPH_BASEADDR + 0x3400) // F411 Specific
#define SYSCFG_BASEADDR         (APB2PERIPH_BASEADDR + 0x3800)
#define EXTI_BASEADDR           (APB2PERIPH_BASEADDR + 0x3C00)

#define TIM9_BASEADDR           (APB2PERIPH_BASEADDR + 0x4000)
#define TIM10_BASEADDR          (APB2PERIPH_BASEADDR + 0x4400)
#define TIM11_BASEADDR          (APB2PERIPH_BASEADDR + 0x4800)
#define SPI5_BASEADDR           (APB2PERIPH_BASEADDR + 0x5000) // F411 Specific



/*** =**************** Peripheral Register definition Structures *************************/
typedef struct
{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];

}GPIO_RegDef_t;



typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	uint32_t 	  Reserved1[2];
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t 	  Reserve2[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	uint32_t 	  Reserve3[2];
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t 	  Reserve4[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	uint32_t 	  Reserve5[2];
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t 	  Reserve6[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t 	  Reserve7[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	uint32_t 	  Reserve8;
	__vo uint32_t DCKCFGR;

}RCC_RegDef_t;

typedef struct{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
}SPI_RegDef_t;


// I2C reg def struct
typedef struct{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo unit32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo unit32_t FLTR;
}I2C_RegDef_t;
/*
 * Peripheral Definitions (Peripheral base addresses typecasted to xx_RegDef_t
 */
#define GPIOA		 ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB		 ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC		 ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD		 ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE		 ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH		 ((GPIO_RegDef_t*)GPIOH_BASEADDR)



#define RCC 		((RCC_RegDef_t*)RCC_BASEADDR)

#define SPI1		((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2		((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3		((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4		((SPI_RegDef_t*)SPI4_BASEADDR)

#define I2C1		((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2		((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3		((I2C_RegDef_t*)I2C3_BASEADDR)

/*
 * Clock Enable Macros for GPIOx Peripherals
 */

#define GPIOA_PCLCK_EN()		(RCC->AHB1ENR |= (1<<0));
#define GPIOB_PCLCK_EN()		(RCC->AHB1ENR |= (1<<1));
#define GPIOC_PCLCK_EN()		(RCC->AHB1ENR |= (1<<2));
#define GPIOD_PCLCK_EN()		(RCC->AHB1ENR |= (1<<3));
#define GPIOE_PCLCK_EN()		(RCC->AHB1ENR |= (1<<4));
#define GPIOH_PCLCK_EN()		(RCC->AHB1ENR |= (1<<7));


/*
 * Clock Disable Macros for GPIOx Peripherals
 */

#define GPIOA_PCLCK_DI()		(RCC->AHB1ENR &= ~(1<<0));
#define GPIOB_PCLCK_DI()		(RCC->AHB1ENR &= ~(1<<1));
#define GPIOC_PCLCK_DI()		(RCC->AHB1ENR &= ~(1<<2));
#define GPIOD_PCLCK_DI()		(RCC->AHB1ENR &= ~(1<<3));
#define GPIOE_PCLCK_DI()		(RCC->AHB1ENR &= ~(1<<4));
#define GPIOH_PCLCK_DI()		(RCC->AHB1ENR &= ~(1<<7));


/*
 * CLock Enable Macro for I2Cx Peripherals
 */

#define I2C1_PCLCK_EN()			(RCC->APB1ENR |= (1<<21));
#define I2C2_PCLCK_EN()			(RCC->APB1ENR |= (1<<22));
#define I2C3_PCLCK_EN()			(RCC->APB1ENR |= (1<<23));


/*
 * CLock Disable Macro for I2Cx Peripherals
 */

#define I2C1_PCLCK_DI()			(RCC->APB1ENR &= ~(1<<21));
#define I2C2_PCLCK_DI()			(RCC->APB1ENR &= ~(1<<22));
#define I2C3_PCLCK_DI()			(RCC->APB1ENR &= ~(1<<23));


/*
 * Clock Enable Macros for SPIx Peripherals
 */
#define SPI2_PCLCK_EN()			(RCC->APB1ENR |= (1<<14));
#define SPI3_PCLCK_EN()			(RCC->APB1ENR |= (1<<15));
#define SPI1_PCLCK_EN()			(RCC->APB2ENR |= (1<<12));
#define SPI4_PCLCK_EN()			(RCC->APB2ENR |= (1<<13));



/*
 * Clock Disable Macros for SPIx Peripherals
 */
#define SPI2_PCLCK_DI()			(RCC->APB1ENR &= ~(1<<14));
#define SPI3_PCLCK_DI()			(RCC->APB1ENR &= ~(1<<15));
#define SPI1_PCLCK_DI()			(RCC->APB2ENR &= ~(1<<12));
#define SPI4_PCLCK_DI()			(RCC->APB2ENR &= ~(1<<13));

/*
 * Clock Enable Macros for USRATx Peripherals
 */

#define USART1_PCLCK_EN()			(RCC->APB2ENR |= (1<<4));
#define USART2_PCLCK_EN()			(RCC->APB1ENR |= (1<<17));
#define USART6_PCLCK_EN()			(RCC->APB2ENR |= (1<<5));

/*
 * Clock Enable Macros for USRATx Peripherals
 */

#define USART1_PCLCK_DI()			(RCC->APB2ENR &= ~(1<<4));
#define USART2_PCLCK_DI()			(RCC->APB1ENR &= ~(1<<17));
#define USART6_PCLCK_DI()			(RCC->APB2ENR &= ~(1<<5));


/*
 * Clock Enable Macros for SYSCFG Peripherals
 */

#define SYSCFG_PCLCK_EN()			(RCC->APB2ENR |= (1<<14));


/*
 * Clock Enable Macros for SYSCFG Peripherals
 */

#define SYSCFG_PCLCK_DI()			(RCC->APB2ENR &= ~(1<<14));

/*
 * GPIO Register Reset Macros
 */
#define GPIOA_REG_RESET()				do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RESET()				do{(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET()				do{(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET()				do{(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET()				do{(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOH_REG_RESET()				do{(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));}while(0)

/*
* SPI Register reset macros
*/
#define SPI1_REG_RESET()				do{(RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12));}while(0)
#define SPI2_REG_RESET()				do{(RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14));}while(0)
#define SPI3_REG_RESET()				do{(RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15));}while(0)
#define SPI4_REG_RESET()				do{(RCC->APB2RSTR |= (1 << 13)); (RCC->APB2RSTR &= ~(1 << 13));}while(0)

/*
* I2C Register Reset macros
*/

#define I2C1_REG_RESET()				do{(RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21));}while(0)
#define I2C2_REG_RESET()				do{(RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22));}while(0)
#define I2C1_REG_RESET()				do{(RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR &= ~(1 << 23));}while(0)

// Some Generic Macros
#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET 		SET
#define GPIO_PIN_RESET		RESET

/***********************************************************************
*					Bit Position Defintions of I2C Registers
**********************************************************************/

/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH  				7
#define I2C_CR1_START 					8
#define I2C_CR1_STOP  				 	9
#define I2C_CR1_ACK 				 	10
#define I2C_CR1_SWRST  				 	15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 	15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15

#include "stm32f411xx_gpio_driver.h"
#include "stm32f411xx_spi_driver.h"
#include "stm32f411xx_i2c_driver.h"


#endif /* INC_STM32F411XX_H_ */
