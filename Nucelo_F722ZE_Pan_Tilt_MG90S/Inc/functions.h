/*
 * functions.h
 *
 *  Created on: Nov 6, 2024
 *      Author: lilian.jaouanne
 */

#ifndef INC_FUNCTIONS_H_
#define INC_FUNCTIONS_H_

#include "main.h"


#define LOG_SIZE	2000//Bytes (char)

extern uint32_t log_uart_nbr_bytes;
extern char log_uart5[LOG_SIZE];

void add_log(char* message, uint32_t size);

#endif /* INC_FUNCTIONS_H_ */
