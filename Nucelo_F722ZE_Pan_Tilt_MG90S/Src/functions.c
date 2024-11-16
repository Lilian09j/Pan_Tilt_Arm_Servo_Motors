/*
 * functions.c
 *
 *  Created on: Nov 6, 2024
 *      Author: lilian.jaouanne
 */


/* Includes ------------------------------------------------------------------*/
#include "functions.h"

uint32_t log_uart_nbr_bytes = 0;
char log_uart5[LOG_SIZE] = "\0";


void add_log(char* message, uint32_t size){
	log_uart_nbr_bytes += size;

	uint32_t index_log = 0;
	while(log_uart5[index_log] != '\0')
		index_log++;

	memcpy(&log_uart5[index_log], message, size);
}
