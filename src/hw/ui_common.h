/*
 * ui_common.h
 *
 *  Created on: 2021. 9. 17.
 *      Author: WANG
 */

#ifndef SRC_COMMON_HW_INCLUDE_UI_COMMON_H_
#define SRC_COMMON_HW_INCLUDE_UI_COMMON_H_

#include "hw_def.h"
#include "hw.h"

void draw_fan_status(uint16_t x, uint16_t y, const bool blink);
bool get_blink();

#endif /* SRC_COMMON_HW_INCLUDE_UI_COMMON_H_ */
