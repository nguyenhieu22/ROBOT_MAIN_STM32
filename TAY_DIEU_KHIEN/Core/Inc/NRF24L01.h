/*
 * NRF24L01.h
 *
 *  Created on: May 22, 2024
 *      Author: User
 */

#ifndef INC_NRF24L01_H_
#define INC_NRF24L01_H_


#define CONFIG      0X00
#define EN_AA       0X01
#define EN_RXADDR   0X02
#define SETUP_AW    0X03
#define SETUP_RETR  0X04
#define RF_CH       0X05
#define RF_SETUP    0X06
#define STATUS      0X07
#define OBSERVE_TX  0X08
#define CD          0X09
#define RX_ADDR_P0  0X0A
#define RX_ADDR_P1  0X0B
#define RX_ADDR_P2  0X0C
#define RX_ADDR_P3  0X0D
#define RX_ADDR_P4  0X0E
#define RX_ADDR_P5  0X0F
#define TX_ADDR     0X10
#define RX_PW_P0    0X11
#define RX_PW_P1    0X12
#define RX_PW_P2    0X13
#define RX_PW_P3    0X14
#define RX_PW_P4    0X15
#define RX_PW_P5    0X16
#define FIFO_STATUS 0X17
#define DYNPD       0X1C
#define FEATURE     0X1D


/*----------*/
#define R_REGISTER     0X00
#define W_REGISTER     0X20
#define REGISTER_MASK  0X1F
#define ACTIVATE       0X50
#define R_RX_PL_WID    0X60
#define R_RX_PAYLOAD   0X61
#define W_TX_PAYLOAD   0XA0
#define W_ACK_PAYLOAD  0XA8
#define FLUSH_TX       0XE1
#define FLUSH_RX       0XE2
#define REUSE_TX__PL   0XE3
#define NOP            0XFF

// mot so ham

void CS_Select(void);
void CS_UnSelect(void);

void CE_Enable(void);
void CE_Disable(void);

void nrf24_WriteReg(uint8_t Reg, uint8_t Data);
void nrf24_WriteRegMulti(uint8_t Reg, uint8_t *data, int size);

uint8_t nrf24_ReadReg(uint8_t Reg);
void nrf24_ReadReg_Multi(uint8_t Reg, uint8_t *data, int size);

void nrfsendCmd(uint8_t cmd);

void NRF24_Init(void);

void NRF24_TxMode(uint8_t *Address, uint8_t channel);

uint8_t NRF24_Transmit(uint8_t *data);


void NRF24_RxMode(uint8_t *Address, uint8_t channel);
uint8_t isDataAvailable(int pipenum);
void NRF_Receive(uint8_t *data);

#endif /* INC_NRF24L01_H_ */
