#include "stm32f429.h"
#include <stdint.h>

/* GPIO Driver APIs Implementation*/

void GPIOPeriClockControl(GPIO_RegDef_t * pGPIOx, uint8_t EnorDi){
    if (EnorDi == ENABLE){
            if (pGPIOx == GPIOA){
              GPIOA_PERI_CLCK_EN();
            }
            else if (pGPIOx = GPIOB){
              GPIOB_PCLCK_ENABLE();
            }
            else if (pGPIOx = GPIOC){
              GPIOB_PCLCK_ENABLE();
            }
            else if (pGPIOx = GPIOD){
              GPIOB_PCLCK_ENABLE();
            }
            else if (pGPIOx = GPIOE){
              GPIOB_PCLCK_ENABLE();
            }
            else if (pGPIOx = GPIOF){
              GPIOB_PCLCK_ENABLE();
            }
            else if (pGPIOx = GPIOG){
              GPIOB_PCLCK_ENABLE();
            }
            else if (pGPIOx = GPIOH){
              GPIOB_PCLCK_ENABLE();
            }
            else if (pGPIOx = GPIOI){
              GPIOB_PCLCK_ENABLE();
            }
            else if (pGPIOx = GPIOJ){
              GPIOB_PCLCK_ENABLE();
            }
            else if (pGPIOx = GPIOK){
              GPIOB_PCLCK_ENABLE();
            }
      
      
    }
}

// Fucntion to inititizalize the GPIO peripheral in the driver code 

void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

    unint32_t temp;
    //1 Configure the mode of the GPIO Pin 
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
      // non Interrupt mode 
        temp = (pGPIOHandle->GPIO_PinConfig.PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        // CLear the required the bit field
        pGPIOHandle->GPIOx->MODER &= ~(0x03 << pGPIOHandle->pGPIOHandle->GPIO_PinConfig.PinNumber);
        // Now store the required bit in the appropiate register
        pGPIOHandle->pGPIOx->MODER = temp;
    }
    else {
        // Interrupt Code todo
    }
    //2 Now set the SPEED of the GPIO Pin to be configured
    temp = 0 
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PIN_SPEED << (2 * pGPIOHandle.GPIO_PinConfig.GPIO_PinNumber));
    //Clearing the existing bit field position
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x03 << GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OSPEEDR = temp;
    temp = 0;
    
}|

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
    if(pGPIOx == GPIOA){
        GPIOA_REG_RESET();
    }
    else if (pGPIOx == GPIOB){
        GPIOB_REG_RESET();
    }
    else if (pGPIOx == GPIOC){
        GPIOC_REG_RESET();
    }
    else if (pGPIOx == GPIOD){
        GPIOD_REG_RESET();
    }
    else if (pGPIOx == GPIOE){
        GPIOE_REG_RESET();
    }
    else if (pGPIOx == GPIOF){
        GPIOF_REG_RESET();
    }
    else if (pGPIOx == GPIOH){
        GPIOH_REG_RESET();
    }
    else if (pGPIOx == GPIOI){
        GPIOI_REG_RESET();
    }
    else if (pGPIOx == GPIOJ){
        GPIOJ_REG_RESET();
    }
    else if (pGPIOx == GPIOK){
        GPIOK_REG_RESET();
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
*/
