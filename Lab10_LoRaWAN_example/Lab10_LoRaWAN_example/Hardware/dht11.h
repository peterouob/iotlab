#ifndef __DHT11_H
#define __DHT11_H

#include "stm32f4xx_hal.h"

uint8_t DHT11GetData(float *Humi, float *Temp);

#endif