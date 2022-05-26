/*
 * MAX31855.c
 *
 *  Created on: Apr 8, 2022
 *      Author: lukas
 */

#include "main.h"
#include "MAX31855.h"

void MAX31855_init(max31855_t* dev, GPIO_TypeDef* gpio, uint16_t pin, SPI_HandleTypeDef* spi) {
	dev->gpio = T1_CS_GPIO_Port;
	dev->pin = T1_CS_Pin;
	dev->spi = spi;

	HAL_GPIO_WritePin(dev->gpio, dev->pin, GPIO_PIN_SET);

}


float MAX31855_read_celsius(max31855_t* dev) {

	HAL_GPIO_WritePin(dev->gpio, dev->pin, GPIO_PIN_RESET);
	HAL_SPI_Receive(dev->spi, dev->buf, 4, 1000);
	HAL_GPIO_WritePin(dev->gpio, dev->pin, GPIO_PIN_SET);

	dev->rawData = dev->buf[3] | ( dev->buf[2] << 8 ) | ( dev->buf[1] << 16 ) | ( dev->buf[0] << 24 );

	if (dev->rawData & 0x80000000) {
		// Negative value, drop the lower 18 bits and explicitly extend sign bits.
		dev->rawData = 0xFFFFC000 | ((dev->rawData >> 18) & 0x00003FFF);
	} else {
		dev->rawData >>= 18;
	}

	dev->temp = dev->rawData;
	// LSB = 0.25 degrees C
	dev->temp *= 0.25;
	return dev->temp;
}


