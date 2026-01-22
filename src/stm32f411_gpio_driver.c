#include "stm32f411xx_gpio_driver.h"



/*
* API Implementation of the GPIO functions
*/

void SPIPeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
  if (EnorDi == ENABLE){
        if (SPIx == SPI1){
          SPI1_PCLCK_ENABLE();
        }
        else if (SPIx == SPI2){
          SPI2_PCLCK_ENABLE();
        }
        else if (SPIx == SPI3){
          SPI3_PCLCK_ENABLE();
        }
    
  }else {
    
  }

}
