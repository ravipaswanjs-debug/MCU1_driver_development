/*
 * stm32f411xx_gpio_driver.h
 *
 * Created on: Jan 17, 2026
 * Author: ravi
 */

#ifndef INC_STM32F411XX_GPIO_DRIVER_H_
#define INC_STM32F411XX_GPIO_DRIVER_H_

#include "stm32f411xx.h"

typedef struct {
    uint8_t GPIO_PinNumber;
    uint8_t GPIO_PinMode;
    uint8_t GPIO_PinSpeed;
    uint8_t GPIO_PinPuPdControl;
    uint8_t GPIO_PinOPType;
    uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

/*
 * Handle Structure for GPIO Pin
 */
typedef struct {
    GPIO_RegDef_t *pGPIOx;
    GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle_t;

/*
 * GPIO Pin Number Macros
 */
#define GPIO_PIN_NO_0       0
#define GPIO_PIN_NO_1       1
#define GPIO_PIN_NO_2       2
#define GPIO_PIN_NO_3       3
#define GPIO_PIN_NO_4       4
#define GPIO_PIN_NO_5       5
#define GPIO_PIN_NO_6       6
#define GPIO_PIN_NO_7       7
#define GPIO_PIN_NO_8       8
#define GPIO_PIN_NO_9       9
#define GPIO_PIN_NO_10      10
#define GPIO_PIN_NO_11      11
#define GPIO_PIN_NO_12      12
#define GPIO_PIN_NO_13      13
#define GPIO_PIN_NO_14      14
#define GPIO_PIN_NO_15      15

/*
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN        0
#define GPIO_MODE_OUT       1
#define GPIO_MODE_ALTFN     2
#define GPIO_MODE_ANALOG    3
#define GPIO_MODE_IT_FT     4
#define GPIO_MODE_IT_RT     5
#define GPIO_MODE_IT_RFT    6

/*
 * GPIO pin possible Output Types
 */
#define GPIO_OP_TYPE_PP     0
#define GPIO_OP_TYPE_OD     1

/*
 * GPIO pin Possible Speed
 */
#define GPIO_SPEED_LOW      0
#define GPIO_SPEED_MEDIUM   1
#define GPIO_SPEED_HIGH     2
#define GPIO_SPEED_FAST     3

/*
 * GPIO PIN pull up pull down Macros
 */
#define GPIO_NO_PUPD        0
#define GPIO_PIN_PU         1
#define GPIO__PIN_PD        2

/******************************************************************************
 *
 * APIs Supported by this driver
 * for more information about the APIs check the function definitions
 ********************************************************************************/

/*
 * Peripheral Clock Setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi) {
    if (EnorDi == ENABLE) {
        if (pGPIOx == GPIOA) {
            GPIOA_PCLCK_EN();
        } else if (pGPIOx == GPIOB) {
            GPIOB_PCLCK_EN();
        } else if (pGPIOx == GPIOC) {
            GPIOC_PCLCK_EN();
        } else if (pGPIOx == GPIOD) {
            GPIOD_PCLCK_EN();
        } else if (pGPIOx == GPIOE) {
            GPIOE_PCLCK_EN();
        } else if (pGPIOx == GPIOH) {
            GPIOH_PCLCK_EN();
        }
    } else {
        if (pGPIOx == GPIOA) {
            GPIOA_PCLCK_DI();
        } else if (pGPIOx == GPIOB) {
            GPIOB_PCLCK_DI();
        } else if (pGPIOx == GPIOC) {
            GPIOC_PCLCK_DI();
        } else if (pGPIOx == GPIOD) {
            GPIOD_PCLCK_DI();
        } else if (pGPIOx == GPIOE) {
            GPIOE_PCLCK_DI();
        } else if (pGPIOx == GPIOH) {
            GPIOH_PCLCK_DI();
        }
    }
}

/*
 * GPIO Enabling and disabling
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
    uint32_t temp;
    // 1. Configure the mode of the Pin
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
        // non interrupt mode
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->MODER &= ~(0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
        pGPIOHandle->pGPIOx->MODER |= temp; // setting
        temp = 0;
    } else {
        // Interrupt Mode
    }

    // 2. Configure the Speed
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
    pGPIOHandle->pGPIOx->OSPEEDR |= temp;
    temp = 0;

    // 3. configure the PuPd Setting
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
    pGPIOHandle->pGPIOx->PUPDR |= temp;
    temp = 0;

    // 4. Configure the optype
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x01 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
    pGPIOHandle->pGPIOx->OTYPER |= temp;
    temp = 0;

    // 5. Configure the alt functionality
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
        uint8_t temp1, temp2;
        temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8);
        temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8);
        pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << 4 * temp2); // Clearing
        pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << 4 * temp2);
    }
}

void GPIO_deinit(GPIO_RegDef_t *pGPIOx) {
    if (pGPIOx == GPIOA) {
        GPIOA_REG_RESET();
    } else if (pGPIOx == GPIOB) {
        GPIOB_REG_RESET();
    } else if (pGPIOx == GPIOC) {
        GPIOC_REG_RESET();
    } else if (pGPIOx == GPIOD) {
        GPIOD_REG_RESET();
    } else if (pGPIOx == GPIOE) {
        GPIOE_REG_RESET();
    } else if (pGPIOx == GPIOH) {
        GPIOH_REG_RESET();
    }
}

/*
 * GPIO Read and Write Functionalities
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
    uint8_t value;
    value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
    return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
    uint16_t value;
    value = (uint16_t)pGPIOx->IDR;
    return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value) {
    if (Value == GPIO_PIN_SET) {
        // Write 1 to the Bit position
        pGPIOx->ODR |= (1 << PinNumber);
    } else {
        // Write 0 to the Bit Position
        pGPIOx->ODR &= ~(1 << PinNumber);
    }
}

void GPIO_WritetoOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value) {
    pGPIOx->ODR = Value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
    pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * GPIO Interrupt Handling and configuration
 */
void GPIO_IRQHanding(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQConfig(uint8_t PinNumber);

#endif /* INC_STM32F411XX_GPIO_DRIVER_H_ */
