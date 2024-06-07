/*
 * NRF24L01.c
 *
 *  Created on: Mar 26, 2024
 *      Author: User
 */
#include "stm32f1xx_hal.h"
#include "NRF24L01.h"

extern SPI_HandleTypeDef hspi1;
#define NRF24_SPI &hspi1
// Dinh nghia chan
#define NRF24_CE_PORT GPIOA
#define NRF24_CE_PIN  GPIO_PIN_2

#define NRF24_CSN_PORT GPIOA
#define NRF24_CSN_PIN  GPIO_PIN_3


//Cac ham trong thu vien

void CS_Select(void){
	HAL_GPIO_WritePin(NRF24_CSN_PORT,NRF24_CSN_PIN,GPIO_PIN_RESET);
}
void CS_UnSelect(void){
	HAL_GPIO_WritePin(NRF24_CSN_PORT,NRF24_CSN_PIN,GPIO_PIN_SET);
}

void CE_Enable(void){
	HAL_GPIO_WritePin(NRF24_CE_PORT,NRF24_CE_PIN,GPIO_PIN_SET);
}
void CE_Disable(void){
	HAL_GPIO_WritePin(NRF24_CE_PORT,NRF24_CE_PIN,GPIO_PIN_RESET);
}



void nrf24_WriteReg(uint8_t Reg, uint8_t Data){
	uint8_t buf[2];
	buf[0] = Reg|1<<5;
	buf[1]= Data;

	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, buf,2,1000);

	CS_UnSelect();
}

void nrf24_WriteRegMulti(uint8_t Reg, uint8_t *data, int size){
	uint8_t buf[2];
	buf[0] = Reg|1<<5;
	//buf[1]= Data;

	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, buf,1,100);
	HAL_SPI_Transmit(NRF24_SPI, data,size,1000);

	CS_UnSelect();
}

uint8_t nrf24_ReadReg(uint8_t Reg){
	uint8_t data = 0;

	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, &Reg, 1, 100);
	HAL_SPI_Receive(NRF24_SPI, &data, 1, 100);

	CS_UnSelect();
	return data;
}

void nrf24_ReadReg_Multi(uint8_t Reg, uint8_t *data, int size){
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, &Reg, 1, 100);
	HAL_SPI_Receive(NRF24_SPI, data, size, 1000);

	CS_UnSelect();
}

// Gui ma lenh den nRF24
void nrfsendCmd(uint8_t cmd){
	CS_Select();

	HAL_SPI_Transmit(NRF24_SPI, &cmd, 1, 100);

	CS_UnSelect();
}

void NRF24_Init(void){
	CE_Disable();


	nrf24_WriteReg(CONFIG, 0);
	nrf24_WriteReg(EN_AA, 0);   // No Auto ACK
	nrf24_WriteReg(EN_RXADDR, 0);
	nrf24_WriteReg(SETUP_AW, 0X03);  // 5 bytes for the TX/RX
	nrf24_WriteReg(SETUP_RETR, 0);
	nrf24_WriteReg(SETUP_RETR, 0);
	nrf24_WriteReg(RF_CH, 0);
	nrf24_WriteReg(RF_SETUP, 0X0E);// POWER = 0dB, data rate = 20Mbps

	CE_Enable();
}

void NRF24_TxMode(uint8_t *Address, uint8_t channel){
	CE_Disable();


	nrf24_WriteReg(RF_CH, channel);

	nrf24_WriteRegMulti(TX_ADDR, Address, 5);

	// power up the device
	uint8_t config = nrf24_ReadReg(CONFIG);
	config = config|(1<<1);
	nrf24_WriteReg(CONFIG, config);

	CE_Enable();
}

// Transmit the data
uint8_t NRF24_Transmit(uint8_t *data){
	uint8_t cmdtosend = 0;

	CS_Select();

	cmdtosend = W_TX_PAYLOAD;
	HAL_SPI_Transmit(NRF24_SPI, &cmdtosend, 1, 100);

	HAL_SPI_Transmit(NRF24_SPI, data, 32, 1000);

	CS_UnSelect();
	HAL_Delay(1);
	uint8_t fifostatus = nrf24_ReadReg(FIFO_STATUS);
	if((fifostatus&(1<<4))&&(!(fifostatus&(1<<3)))){
		cmdtosend = FLUSH_TX;
		nrfsendCmd(cmdtosend);
		return 1;
	}
	return 0;
}

void NRF24_RxMode(uint8_t *Address, uint8_t channel){
	CE_Disable();

	nrf24_WriteReg(RF_CH, channel);

	// Chon ong du lieu 1
	uint8_t en_rxaddr = nrf24_ReadReg(EN_RXADDR);
	en_rxaddr = en_rxaddr | (1<<1);
	nrf24_WriteReg(EN_RXADDR, en_rxaddr);

	nrf24_WriteRegMulti(RX_ADDR_P1, Address, 5);
	nrf24_WriteReg(RX_PW_P1, 32);  // 32 BYTE.

	// power up the device
	uint8_t config = nrf24_ReadReg(CONFIG);
	config = config|(1<<1)|(1<<0);
	nrf24_WriteReg(CONFIG, config);

	CE_Enable();
}

uint8_t isDataAvailable(int pipenum){
	uint8_t status = nrf24_ReadReg(STATUS);
	if((status&(1<<6))&&(status&(pipenum<<1))){
		nrf24_WriteReg(STATUS, (1<<6));
		return 1;
	}
	return 0;
}

void NRF_Receive(uint8_t *data){
	uint8_t cmdtosend = 0;
    CS_Select();

	cmdtosend = R_RX_PAYLOAD;
	HAL_SPI_Transmit(NRF24_SPI, &cmdtosend, 1, 100);

	HAL_SPI_Receive(NRF24_SPI, data, 32, 1000);

	CS_UnSelect();

	HAL_Delay(1);

	cmdtosend = FLUSH_RX;
	nrfsendCmd(cmdtosend);

}



