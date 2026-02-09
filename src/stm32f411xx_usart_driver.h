#include "stm32f411xx_usart_driver.h"


void USART_PeriClockControl(USART_RegDef_t  *pUSARTx,uint8 EnorDi){

  if (EnorDi == ENABLE){
    if (*pUSART == USART1){
      USART1_PCLK_ENABLE();
    }
    else if (*pUSARTx == USART2){
        USART_PCLK_ENABLE();
    }
    else if (*pUSATYx == USART6){
        USART_PCLK_ENABLE();
    }
    }
  else {
    
  }
  }
