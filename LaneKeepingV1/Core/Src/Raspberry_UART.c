/*
 * Raspberry_UART.c
 *
 *  Created on: Oct 27, 2023
 *      Author: M.Cristina Giannini
 */


#include "Raspberry_UART.h"

#include "main.h"
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>


#define MAX_VALUES 5


void parseCSV(const char *csvString, float *values) {
    char *token;
    char *copy = strdup(csvString); // Make a copy of the string to avoid modifying the original
    int index = 0;

    token = strtok(copy, ",");
    while (token != NULL && index < MAX_VALUES) {
        values[index++] = strtof(token, NULL); // Convert token to float and store in the array
        token = strtok(NULL, ",");
    }

    free(copy); // Free the dynamically allocated memory for the copied string
}

//USART6 per la ricezione dei dati del Raspberry
void mainSerialRead(uint8_t *msg, uint32_t message_size) {
	uint16_t timeout = 0xFFFF;
		uint8_t app = '\0';
		int msg_len = 0;

		int i = 0;

		while(app!='\n' && i <message_size-1){
			HAL_UART_Receive(&huart6, &app, 1, timeout);

			if(app != '\n'){
				msg[i] = app;
				msg_len = i;
			}
			i++;
		}

		printf("%s\r\n",msg);
}
