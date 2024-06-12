/*
 * Raspberry_UART.c
 *
 *  Created on: Oct 27, 2023
 *      Author: M.Cristina Giannini
 */


#include "RX_UART.h"

#include "main.h"
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>


#define MAX_VALUES 3


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
