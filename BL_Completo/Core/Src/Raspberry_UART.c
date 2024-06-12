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


#define MAX_VALUES 6

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
/*
// USART6 per la ricezione dei dati del Raspberry
int mainSerialRead(uint8_t *msg, uint32_t message_size) {

    uint16_t timeout = 0xFFFF;
    uint8_t app = '\0';


    // Check if data is available to read
    int ret = 0;
    HAL_UART_Receive(&huart6, &app, 1, timeout);


    printf("%c", app);
    if(app == 's'){
    	stato_rx = 1;
    	msg_len = 0;
    	i = 0;

    	for(int j = 0; j < sizeof(msg); j++){
    		msg[j] = '\0';
    	}
    } else if (app == '\n'){
    	stato_rx = 0;
    	ret = 1;
    } else {
    	msg[i] = app;
    	msg_len = i;
    	i++;
    }

    /*
    switch(stato_rx){
    default:
    	if(app == 's'){
    		stato_rx = 1;
    		msg_len = 0;
    		i = 0;

    		for(int j = 0; j < sizeof(msg); j++){
    			msg[j] = '\0';
    		}

    	}
    	break;
    case 1:
    	if(app != '\n'){
    		msg[i] = app;
    		msg_len = i;
    		i++;
    	}else{
    		stato_rx = 0;
    		ret = 1;
    	}
    	break;
    }

    return ret;

    //printf("Stringa: %s\r\n", msg);
}
*/
