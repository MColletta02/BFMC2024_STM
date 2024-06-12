/*
 * Raspberry_UART.h
 *
 *  Created on: Oct 27, 2023
 *      Author: M.Cristina Giannini
 */

#ifndef INC_RX_UART_H_
#define INC_RX_UART_H_

#include "main.h"

//uint8_t msg[45] = { "\0" };
extern UART_HandleTypeDef huart6;

void csvInterpreter(float *, char *);


#endif /* INC_RX_UART_H_ */
