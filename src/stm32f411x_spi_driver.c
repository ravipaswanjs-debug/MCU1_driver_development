#include "stm32f411x_spi_driver.h"

/*
 * Peripheral Clock Setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
    if (EnorDi == ENABLE) {
        if (pSPIx == SPI1) {
           SPI1_PCLCK_EN();
        } else if (pSPIx == SPI2) {
            SPI2_PCLCK_EN();
        } else if (pSPIx == SPI3) {
            SPI3_PCLCK_EN();
        } else if (pSPIx == SPI4) {
            SPI4_PCLCK_EN();
        }
    } else {
        if (pSPIx == SPI1) {
            SPI1_PCLCK_DI();
        } else if (pSPIx == SP12) {
            SPI2_PCLCK_DI();
        } else if (pSPIx == SPI3) {
            SPI3_PCLCK_DI();
        } else if (pSPIx == SPI4) {
            SPI4_PCLCK_DI();
        } 
    }
}

void SPI_Init(SPI_Handle_t *pSPIHandle){
  
}

