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
      pI2C->CR1 |= (1 << 0);
    }
    else if (EnorDi == DISABLE){
      pI2C->CR1 &= ~(1 << 0);
    }
  }

  
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	uint32_t tempreg = 0 ;

	//enable the clock for the i2cx peripheral
	I2C_PeriClockControl(pI2CHandle->pI2Cx,ENABLE);

	//ack control bit
	tempreg |= pI2CHandle->I2C_Config.I2C_AckControl << 10;
	pI2CHandle->pI2Cx->CR1 = tempreg;

	//configure the FREQ field of CR2
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() /1000000U ;
	pI2CHandle->pI2Cx->CR2 =  (tempreg & 0x3F);

   //program the device own address
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	tempreg |= ( 1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	//CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode
		ccr_value = (RCC_GetPCLK1Value() / ( 2 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		tempreg |= (ccr_value & 0xFFF);
	}else
	{
		//mode is fast mode
		tempreg |= ( 1 << 15);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value() / ( 3 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		}else
		{
			ccr_value = (RCC_GetPCLK1Value() / ( 25 * pI2CHandle->I2C_Config.I2C_SCLSpeed ) );
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	//TRISE Configuration
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//mode is standard mode

		tempreg = (RCC_GetPCLK1Value() /1000000U) + 1 ;

	}else
	{
		//mode is fast mode
		tempreg = ( (RCC_GetPCLK1Value() * 300) / 1000000000U ) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}
     
}

// Start Pasting from here 3 Feb 2026

static void void I2C_GenerateStartConditon(I2C_RegDef_t *pI2Cx);

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len,uint8_t SlaveAddr){

 // Generate The Start condition 
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

// confirm that the start generation is completed by cheching the SB
// Flag in the SR1 register 
// Note : Until SB is cleared SCL will be streched (Pulledd to low)
while(! (pI2CHandle->I2Cx->SB1 ))
	
}

void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_START)
}











