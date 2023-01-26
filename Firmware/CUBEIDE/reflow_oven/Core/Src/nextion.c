/*
 * nextion.c
 *
 *  Created on: Apr 20, 2022
 *      Author: lukas
 */

#include "main.h"
#include "nextion.h"

const uint8_t nxEnd[3] = {0xFF,0xFF,0xFF};

int nextion_send_str_to_id(nextion_t* dp, char* id, char* msg) {

	int len = sprintf(dp->trm, "%s.txt=\"%s\"",id,msg);

	if(HAL_UART_Transmit(dp->uart, (uint8_t*)dp->trm, len, 1000) != HAL_OK) {
		return NX_ERROR;
	}
	if(HAL_UART_Transmit(dp->uart, nxEnd, 3, 1000) != HAL_OK) {
		return NX_ERROR;
	}
	return NX_OK;
}

int nextion_send_val_to_id(nextion_t* dp, char* id, int val) {

	int len = sprintf(dp->trm, "%s.val=%d",id,val);

	if(HAL_UART_Transmit(dp->uart, (uint8_t*)dp->trm, len, 1000) != HAL_OK) {
		return NX_ERROR;
	}
	if(HAL_UART_Transmit(dp->uart, nxEnd, 3, 1000) != HAL_OK) {
		return NX_ERROR;
	}
	return NX_OK;
}

int nextion_send_float_to_id(nextion_t* dp, char* id, float val) {

	int len = sprintf(dp->trm, "%s.txt=\"%.2f\"",id,val);

	if(HAL_UART_Transmit(dp->uart, (uint8_t*)dp->trm, len,1000) != HAL_OK) {
		return NX_ERROR;
	}

	if(HAL_UART_Transmit(dp->uart, (uint8_t*)nxEnd, 3,1000) != HAL_OK) {
		return NX_ERROR;
	}
	return NX_OK;
}

int write(nextion_t* dp, char* msg) {

	int len = sprintf(dp->trm, "%s", msg);

	if(HAL_UART_Transmit(dp->uart, (uint8_t*)dp->trm, len, 1000) != HAL_OK) {
		return NX_ERROR;
	}
	if(HAL_UART_Transmit(dp->uart, nxEnd, 3, 1000) != HAL_OK) {
		return NX_ERROR;
	}
	return NX_OK;
}

void nextion_decode_response(nextion_t* dp) {

	if(dp->rcv[0] == '$' && dp->rcv[3] == '&') {

		dp->response.page = dp->rcv[1];
		dp->response.id = dp->rcv[2];
	}

}

void nextion_print_page(nextion_t* dp) {
	dp->write_id_str(dp, "varProfile", "TS391LT50");
	dp->write_id_str(dp, "varState", "IDLE");
	dp->write_id_str(dp, "prepStart", "0");
	dp->write_id_str(dp, "prepEnd", "-");
	dp->write_id_str(dp, "prepTemp", "30");
	dp->write_id_str(dp, "heatStart", "-");
	dp->write_id_str(dp, "heatEnd", "90");
	dp->write_id_str(dp, "heatTemp", "90");
	dp->write_id_str(dp, "soakStart", "90");
	dp->write_id_str(dp, "soakEnd", "180");
	dp->write_id_str(dp, "soakTemp", "130");
	dp->write_id_str(dp, "rampStart", "180");
	dp->write_id_str(dp, "rampEnd", "210");
	dp->write_id_str(dp, "rampTemp", "138");
	dp->write_id_str(dp, "reflowStart", "210");
	dp->write_id_str(dp, "reflowEnd", "240");
	dp->write_id_str(dp, "reflowTemp", "165");
	dp->write_id_str(dp, "coolStart", "240");
	dp->write_id_str(dp, "coolEnd", "270");
	dp->write_id_str(dp, "coolTemp", "138");
}


int nextion_init(nextion_t* dp, UART_HandleTypeDef *uart) {
	dp->uart = uart;
	dp->write_id_str = &nextion_send_str_to_id;
	dp->write_id_val = &nextion_send_val_to_id;
	dp->write_id_float = &nextion_send_float_to_id;
	dp->write = &write;
	return NX_OK;
}


