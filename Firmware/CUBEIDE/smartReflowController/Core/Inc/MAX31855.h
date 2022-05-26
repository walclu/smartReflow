/*
 * MAX31855.h
 *
 *  Created on: Apr 8, 2022
 *      Author: lukas
 */

#ifndef INC_MAX31855_H_
#define INC_MAX31855_H_

struct _max31855_t {

	SPI_HandleTypeDef* spi;
	uint8_t buf[5];
	GPIO_TypeDef* gpio;
	uint16_t pin;

	int rawData;
	float temp;

};
typedef struct _max31855_t max31855_t;

void MAX31855_init(max31855_t* dev, GPIO_TypeDef* gpio, uint16_t pin, SPI_HandleTypeDef* spi);
float MAX31855_read_celsius(max31855_t* dev);


#endif /* INC_MAX31855_H_ */
