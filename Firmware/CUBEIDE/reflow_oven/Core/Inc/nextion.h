/*
 * nextion.h
 *
 *  Created on: Apr 20, 2022
 *      Author: lukas
 */

#ifndef INC_NEXTION_H_
#define INC_NEXTION_H_

#define NX_OK 0
#define NX_ERROR -1


typedef int (*FPTR_STR) (void*, char*, char*);

typedef int (*FPTR_VAL) (void*, char*, int);

typedef int (*FPTR_FLOAT) (void*, char*, float);

typedef int (*FPTR_WRITE) (void*, char*);


typedef struct {

	unsigned char page;
	unsigned char id;

} nextion_response_t;

typedef struct {

	nextion_response_t response;

	uint8_t trmRdy;
	UART_HandleTypeDef *uart;
	uint8_t rcv[50];
	uint8_t trm[50];
	uint32_t baud;
	FPTR_STR write_id_str;
	FPTR_VAL write_id_val;
	FPTR_WRITE write;
	FPTR_FLOAT write_id_float;

} nextion_t;


int nextion_init(nextion_t* dp, UART_HandleTypeDef *uart);
void nextion_decode_response(nextion_t* dp);
void nextion_print_page(nextion_t* dp);

extern const uint8_t nxEnd[3];


#endif /* INC_NEXTION_H_ */
