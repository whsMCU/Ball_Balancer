/*
 * st7796.h
 *
 *  Created on: Jan 29, 2024
 *      Author: 왕학승
 */

#ifndef SRC_DRIVERS_LCD_ST7796_H_
#define SRC_DRIVERS_LCD_ST7796_H_

#include "hw.h"
#include "ST7796_Defines.h"


bool st7796Init(void);
bool st7796InitDriver(lcd_driver_t *p_driver);
bool st7796DrawAvailable(void);
bool st7796RequestDraw(void);
void st7796SetWindow(int32_t x, int32_t y, int32_t w, int32_t h);

uint32_t st7796GetFps(void);
uint32_t st7796GetFpsTime(void);

uint16_t st7796GetWidth(void);
uint16_t st7796GetHeight(void);

void writecommand(uint8_t c);

void writedata(uint8_t d);

bool st7796SetCallBack(void (*p_func)(void));
bool st7796SendBuffer(uint8_t *p_data, uint32_t length, uint32_t timeout_ms);
bool st7796DrawBuffer(int16_t x, int16_t y, uint16_t *buffer, uint16_t w, uint16_t h);
bool st7796DrawBufferedLine(int16_t x, int16_t y, uint16_t *buffer, uint16_t w);



#endif /* SRC_DRIVERS_LCD_ST7796_H_ */
