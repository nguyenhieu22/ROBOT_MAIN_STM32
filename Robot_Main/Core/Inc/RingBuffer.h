/*
 * RingBuffer.h
 *
 *  Created on: May 7, 2024
 *      Author: User
 */

#ifndef INC_RINGBUFFER_H_
#define INC_RINGBUFFER_H_

#include "stm32f1xx_hal.h"
#include <string.h>
#include "stdint.h"



/* change the size of the buffer */
#define UART_BUFFER_SIZE 512

typedef struct
{
  unsigned char buffer[UART_BUFFER_SIZE];
  volatile unsigned int head;
  volatile unsigned int tail;
} ring_buffer;


/*---------------------------------FUNCTION------------------------------------------*/

/* Initialize the ring buffer */
void Ringbuf_init(void);

// reads the data in the rx_buffer and increment the tail count in rx_buffer
int Uart_read(void);

// Writes the data to the tx_buffer and increment the head count in tx_buffer
void Uart_write(int c);

// function to send the string to the uart
void Uart_sendstring(const char *s);

// Print a number with any base
// base can be 10, 8 etc
void Uart_printbase (long n, uint8_t base);

// Checks if the data is available to read in the rx_buffer
int IsDataAvailable(void);

int Look_for (char *str, char *buffertolookinto);

void GetDataFromBuffer (char *startString, char *endString, char *buffertocopyfrom, char *buffertocopyinto);

// Resets the entire ring buffer, the new data will start from position 0
void Uart_flush (void);

int Uart_peek();

int Copy_upto (char *string, char *buffertocopyinto);

int Get_after (char *string, uint8_t numberofchars, char *buffertosave);

int Wait_for (char *string);

// The ISR for the uart. put it in the IRQ handler
void Uart_isr (UART_HandleTypeDef *huart);








#endif /* INC_RINGBUFFER_H_ */
