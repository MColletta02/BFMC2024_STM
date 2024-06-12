/*
 * Raspberry_UART.h
 *
 *  Created on: Oct 27, 2023
 *      Author: M.Cristina Giannini
 */

#ifndef INC_RASPBERRY_UART_H_
#define INC_RASPBERRY_UART_H_

#include "main.h"

//uint8_t msg[45] = { "\0" };
extern UART_HandleTypeDef huart6;

void parseCSV(const char *, float *);
int mainSerialRead(uint8_t *, uint32_t);


#endif /* INC_RASPBERRY_UART_H_ */
