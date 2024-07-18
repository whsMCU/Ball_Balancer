/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

#include "hw.h"

#ifndef TOUCH_SCREEN_CALIBRATION_PRECISION
  #define TOUCH_SCREEN_CALIBRATION_PRECISION  80
#endif

#ifndef TOUCH_SCREEN_HOLD_TO_CALIBRATE_MS
  #define TOUCH_SCREEN_HOLD_TO_CALIBRATE_MS   2500
#endif

typedef struct __attribute__((__packed__)) {
  int32_t x, y;
  int16_t offset_x, offset_y;
  uint8_t orientation;
} touch_calibration_t;

typedef struct __attribute__((__packed__)) {
  uint16_t x, y;
  int16_t raw_x, raw_y;
} touch_calibration_point_t;

typedef enum
{
  CALIBRATION_TOP_LEFT = 0x00,
  CALIBRATION_BOTTOM_LEFT,
  CALIBRATION_TOP_RIGHT,
  CALIBRATION_BOTTOM_RIGHT,
  CALIBRATION_SUCCESS,
  CALIBRATION_FAIL,
  CALIBRATION_NONE
}calibrationState;

extern touch_calibration_point_t calibration_points[4];

calibrationState calibration_start(void);
void calibration_end(void);
calibrationState get_calibration_state(void);

bool handleTouch(uint16_t x, uint16_t y);
