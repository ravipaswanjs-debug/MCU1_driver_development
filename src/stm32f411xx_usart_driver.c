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
  if (pUSARTx == USART1){
    USART1_PCLCK_DI();
  }
  else if (pUSARTx == USART2){
    USART2_PCLCK_DI();
  }
  else if (pUSARTx == USART6){
    USART6_PCLCK_DI();
  }
}

void USART_Init(USART_Handle_t *pUSARTHandle)
{

	//Temporary variable
  uint32_t tempreg;
/******************************** Configuration of CR1******************************************/

	//Implement the code to enable the Clock for given USART peripheral
  USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);
	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
    if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_RX_ONLY){
            tempreg |= (1 << USART_CR1_RXE);
    }
	else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TX_ONLY){
		tempreg |= (1 << USART_CR_TXE);
	}
		else if (pUSARTHandle->USARTConfig.USART_Mode == USART_MODETXRX_ONLY){
			tempreg |= (1 << USART_CR1_TXE) | (1 << USART_CR1_RXE)
		}
    else if (pUSARTHandle->USARTC) 
	  
		//Implement the code to enable the Receiver bit field
	
		//Implement the code to enable the Transmitter bit field

		//Implement the code to enable the both Transmitter and Receiver bit fields

    //Implement the code to configure the Word length configuration item

    //Configuration of parity control bit fields
	
		//Implement the code to enable the parity control

		//Implement the code to enable EVEN parity
		//Not required because by default EVEN parity will be selected once you enable the parity control

	
		//Implement the code to enable the parity control

	    //Implement the code to enable ODD parity

   //Program the CR1 register

/******************************** Configuration of CR2******************************************/

	//Implement the code to configure the number of stop bits inserted during USART frame transmission

	//Program the CR2 register

/******************************** Configuration of CR3******************************************/

	//Configuration of USART hardware flow control
	
		//Implement the code to enable CTS flow control


		//Implement the code to enable RTS flow control

	
		//Implement the code to enable both CTS and RTS Flow control

/******************************** Configuration of BRR(Baudrate register)******************************************/

	//Implement the code to configure the baud rate
	//We will cover this in the lecture. No action required here

}

