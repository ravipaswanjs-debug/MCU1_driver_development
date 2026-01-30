// THis is th device specific for the I2C specific header files 
/*
 *
 *  Created on: Feb 20, 2019
 *      Author: Ravi
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"

/*
 * Configuration structure for I2Cx peripheral
 */
typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	uint8_t  I2C_AckControl;
	uint8_t  I2C_FMDutyCycle;

}I2C_Config_t;

/*
 *Handle structure for I2Cx peripheral
 */
typedef struct
{
	I2C_RegDef_t 	*pI2Cx;
	I2C_Config_t 	I2C_Config;
	uint8_t 		*pTxBuffer; /* !< To store the app. Tx buffer address > */
	uint8_t 		*pRxBuffer;	/* !< To store the app. Rx buffer address > */
	uint32_t 		TxLen;		/* !< To store Tx len > */
	uint32_t 		RxLen;		/* !< To store Tx len > */
	uint8_t 		TxRxState;	/* !< To store Communication state > */
	uint8_t 		DevAddr;	/* !< To store slave/device address > */
    uint32_t        RxSize;		/* !< To store Rx size  > */
    uint8_t         Sr;			/* !< To store repeated start value  > */
}I2C_Handle_t;


void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){

      if (EnorDi == ENABLE){

        if (pI2Cx == I2C1){
          I2C1_PCLCK_EN();
        }
        else if (pSPIx == I2C2){
          I2C2_PCLCK_EN();
        }
        else if (pI2Cx == I2C3){
          I2C3_PCLCK_EN();
        }
      }
    else {
        if (pI2Cx == I2C1){
          I2C1_PCLCK_EN();
        }
        else if (pSPIx == I2C2){
          I2C2_PCLCK_EN();
        }
        else if (pI2Cx == I2C3){
          I2C3_PCLCK_EN();
        }
    }
}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8 EnorDi){

  if (EnorDi == ENABLE){

    pI2Cx->I2C_CR1 |= (1 << 0);
  }
  else {
    pI2Cx->I2C_CR1 &= ~(1 << 0);
  }
}



