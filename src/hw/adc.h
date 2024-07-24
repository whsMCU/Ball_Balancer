/*
 * adc.h
 *
 *  Created on: Jul 20, 2024
 *      Author: WANG
 */

#ifndef SRC_HW_ADC_H_
#define SRC_HW_ADC_H_

#include "hw.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef _USE_HW_SPI

#define ADC_MAX_CH          HW_ADC_MAX_CH

extern ADC_HandleTypeDef hadc1;

bool     adcInit(void);
bool     adcOpen(uint8_t ch);
bool     adcClose(uint8_t ch);
uint16_t adcRead(uint8_t ch);

#endif

#ifdef __cplusplus
}
#endif

#endif /* SRC_HW_ADC_H_ */
