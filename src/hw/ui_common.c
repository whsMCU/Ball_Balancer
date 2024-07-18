/*
 * ui_common.c
 *
 *  Created on: 2021. 9. 17.
 *      Author: WANG
 */
#include "ui_common.h"
#include "image.h"


void draw_fan_status(uint16_t x, uint16_t y, const bool blink) {
  uint8_t fanSpeed = 50;
  UI_Image image;

  if (fanSpeed >= 127)
  {
    image = blink ? Fan1_32x32x4 : Fan0_32x32x4;
  }
  else if (fanSpeed > 0)
  {
    image = blink ? Fan1_32x32x4 : Fan0_32x32x4;
  }
  else
  {
    image = Fan0_32x32x4;
  }
  //lcdDrawBufferImage(x, y, image.width, image.height, image.data);
}

bool get_blink() {
  static uint8_t blink = 0;
  static millis_t next_blink_ms = 0;
  millis_t ms = millis();
  if (ELAPSED(ms, next_blink_ms)) {
    blink ^= 0xFF;
    next_blink_ms = ms + 1000 - (100) / 2;
  }
  return blink != 0;
}
