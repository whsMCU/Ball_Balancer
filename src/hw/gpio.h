/*
 * gpio.h
 *
 *  Created on: 2021. 8. 14.
 *      Author: WANG
 */

#ifndef SRC_COMMON_HW_INCLUDE_GPIO_H_
#define SRC_COMMON_HW_INCLUDE_GPIO_H_

#include "hw.h"

#ifdef __cplusplus
extern "C" {
#endif


#ifdef _USE_HW_GPIO


#define GPIO_MAX_CH     HW_GPIO_MAX_CH

enum
{
	StepA_EN,
	StepA_STEP,
	StepA_DIR,
	StepB_EN,
	StepB_STEP,
	StepB_DIR,
	StepC_EN,
	StepC_STEP,
	StepC_DIR,
	StepE0_EN,
	StepE0_STEP,
	StepE0_DIR,
	ADC_Touch_xp,
  ADC_Touch_ym,
	TFT_CS,
	TFT_DC,
	TFT_RST,
	TFT_BKL,
	Toutch_CS,
	BEEPER,
	FLASH_CS,
	Touch_xm,
  Touch_yp,
};


bool gpioInit(void);
bool gpioPinMode(uint8_t ch, uint8_t mode);
void gpioPinWrite(uint8_t ch, bool value);
bool gpioPinRead(uint8_t ch);
void gpioPinToggle(uint8_t ch);


#endif

#ifdef __cplusplus
}
#endif

#endif /* SRC_COMMON_HW_INCLUDE_GPIO_H_ */
