/*
 * DHT22.h
 *
 *  Created on: Aug 2, 2022
 *      Author: Thang
 */

#ifndef INC_DHT22_H_
#define INC_DHT22_H_
#include "stm32f1xx_hal.h"
#include "main.h"
typedef struct
{
	float Temperature;
	float Humidity;
}DHT_DataTypedef;

void DHT_Start (void);
void DHT_GetData (DHT_DataTypedef *DHT_Data);

#endif /* INC_DHT22_H_ */
