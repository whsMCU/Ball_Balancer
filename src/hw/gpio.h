/*
 * gpio.h
 *
 *  Created on: 2021. 8. 14.
 *      Author: WANG
 */

#ifndef SRC_COMMON_HW_INCLUDE_GPIO_H_
#define SRC_COMMON_HW_INCLUDE_GPIO_H_

#include "hw.h"


#ifdef _USE_HW_GPIO


#define GPIO_MAX_CH     HW_GPIO_MAX_CH

enum
{
	StepX_EN,
	StepX_STEP,
	StepX_DIR,
	StepY_EN,
	StepY_STEP,
	StepY_DIR,
	StepZ_EN,
	StepZ_STEP,
	StepZ_DIR,
	StepE0_EN,
	StepE0_STEP,
	StepE0_DIR,
	StepE1_EN,
	StepE1_STEP,
	StepE1_DIR,
	TFT_CS,
	TFT_DC,
	TFT_RST,
	TFT_BKL,
	Toutch_CS,
	BEEPER,
	FLASH_CS,
};


bool gpioInit(void);
bool gpioPinMode(uint8_t ch, uint8_t mode);
void gpioPinWrite(uint8_t ch, bool value);
bool gpioPinRead(uint8_t ch);
void gpioPinToggle(uint8_t ch);


#endif

#endif /* SRC_COMMON_HW_INCLUDE_GPIO_H_ */
