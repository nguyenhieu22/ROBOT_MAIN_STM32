/*
 * CCS811.c
 *
 *  Created on: Apr 8, 2024
 *      Author: User
 */
#include "stm32f1xx_hal.h"
#include "CCS811.h"

extern I2C_HandleTypeDef hi2c1;

#define CCS_I2C &hi2c1

void writeReg(uint8_t reg, uint8_t *pBuf, size_t size){

	HAL_I2C_Mem_Write(CCS_I2C, (CCS811_I2C_ADDRESS1<<1),reg, 1, pBuf, size, 800); //Ghi lien tiep size bytes.

}



void CCS811_softReset(void){

	uint8_t value[4] = {0x11, 0xE5, 0x72, 0x8A}; // gia tri trang 15 datasheet.
	writeReg(CCS811_REG_SW_RESET, value, 4);
}

uint8_t readReg(uint8_t reg, uint8_t *pBuf, size_t size){

	HAL_I2C_Mem_Read(CCS_I2C,(CCS811_I2C_ADDRESS1<<1), reg, 1, pBuf, size, 800);
	return size;
}

void setMeasurementMode(eCycle_t mode, uint8_t thresh, uint8_t interrupt){

	uint8_t measurement[1] = {0};

	measurement[0] = (thresh << 2) | (interrupt << 3) | (mode << 4);

	writeReg(CCS811_REG_MEAS_MODE, measurement, 1);

}

void setInTempHum(float temperature, float humidity){
	int _temp, _rh;

	if(temperature>0)
		_temp = (int)temperature + 0.5;  // this will round off the floating point to the nearest integer value
	else if(temperature<0) // account for negative temperatures
		_temp = (int)temperature - 0.5;
	_temp = _temp + 25;  // temperature high byte is stored as T+25°C in the sensor's memory so the value of byte is positive
	_rh = (int)humidity + 0.5;  // this will round off the floating point to the nearest integer value

	uint8_t envData[4];

	envData[0] = _rh << 1;  // shift the binary number to left by 1. This is stored as a 7-bit value
	envData[1] = 0;  // most significant fractional bit. Using 0 here - gives us accuracy of +/-1%. Current firmware (2016) only supports fractional increments of 0.5
	envData[2] = _temp << 1;
	envData[3] = 0;

	writeReg(CCS811_REG_ENV_DATA, envData, 4);

}
uint8_t CCS811_Init(void){
	uint8_t id[1];
	CCS811_softReset();
	 HAL_Delay(100);
	if(readReg(CCS811_REG_HW_ID, id,1)==0){
		return 0;
	}
	if(id[0] ==0x81){
		HAL_Delay(20);
		//writeReg(CCS811_BOOTLOADER_APP_START, NULL, 0);
		//uint8_t buf[1]={0};
		//writeReg(CCS811_BOOTLOADER_APP_START,buf, 1);
		uint8_t    lodata[1];
			       lodata[0]=CCS811_BOOTLOADER_APP_START;
		HAL_I2C_Master_Transmit(CCS_I2C, (CCS811_I2C_ADDRESS1<<1), lodata, 1, 100);
		HAL_Delay(20);
		setMeasurementMode(eCycle_250ms,0,0);
		HAL_Delay(10);
		setInTempHum(25, 50);
		return 1;
	}
	else return 0;
}

uint8_t CCS811_checkDataReady()
{
    uint8_t status[1] = {0};

    readReg(CCS811_REG_STATUS, status, 1);

    //DBG(status[0],HEX);
    if(!((status[0] >> 3) & 0x01))
        return 0;
    else
        return 1;
}

uint16_t CCS811_getCO2PPM(){
    uint8_t buffer[8];
    uint16_t eCO2;
    readReg(CCS811_REG_ALG_RESULT_DATA, buffer, 8);

    eCO2 = (((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1]);

    return eCO2;
}

uint16_t CCS811_getTVOCPPB(){
    uint8_t buffer[8];
    uint16_t eTVOC;
    readReg(CCS811_REG_ALG_RESULT_DATA, buffer, 8);
    eTVOC = (((uint16_t)buffer[2] << 8) | (uint16_t)buffer[3]);
    return eTVOC;
}

void CCS811_writeBaseLine(uint16_t baseLine){
    uint8_t buffer[2];

    buffer[0] = baseLine>>8;
    buffer[1] = baseLine;
    writeReg(CCS811_REG_BASELINE, buffer, 2);
}



