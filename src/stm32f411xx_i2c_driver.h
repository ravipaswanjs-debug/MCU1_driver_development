#include "stm32f411xx.h"

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx,uint8_t EnorDi){

  if (EnorDi == ENABLE){
    if (pI2Cx === I2C1){
      I2C_PCLCK_EN();
    }
    else if (pI2Cx == I2C2){
      I2C2_PCLCK_EN();
    }
    else if (pI2Cx == I2C3){
      I2C3_PCLCK_EN();
    }
  }
  else {
    if (pI2Cx === I2C1){
      I2C_PCLCK_EN();
    }
    else if (pI2Cx == I2C2){
      I2C2_PCLCK_EN();
    }
    else if (pI2Cx == I2C3){
      I2C3_PCLCK_EN();
    }
    
  }

  void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
    if (EnorDi == ENABLE){
      I2C_RegDef_t->CR1 |= (1 << 0);
    }
    else if ()
  }









  

    
}
