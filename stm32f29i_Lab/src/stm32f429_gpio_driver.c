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
