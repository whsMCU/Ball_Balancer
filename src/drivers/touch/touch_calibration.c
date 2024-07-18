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

#include "touch_calibration.h"

#define TOUCH_CALIBRATION_MAX_RETRIES 5


//TouchCalibration touch_calibration;

touch_calibration_t calibration;
static uint8_t      failed_count;
static calibrationState calibration_state = CALIBRATION_NONE;
touch_calibration_point_t calibration_points[4];

static bool validate_precision(int32_t a, int32_t b) { return (a > b ? (100 * b) / a :  (100 * a) / b) > TOUCH_SCREEN_CALIBRATION_PRECISION; }
static bool validate_precision_x(uint8_t a, uint8_t b) { return validate_precision(calibration_points[a].raw_x, calibration_points[b].raw_x); }
static bool validate_precision_y(uint8_t a, uint8_t b) { return validate_precision(calibration_points[a].raw_y, calibration_points[b].raw_y); }

calibrationState calibration_start(void) {
	calibration.x = 0;
	calibration.y = 0;
	calibration.offset_x = 0;
	calibration.offset_y = 0;
	calibration.orientation = TOUCH_ORIENTATION_NONE;
	calibration_state = CALIBRATION_TOP_LEFT;
	calibration_points[CALIBRATION_TOP_LEFT].x = 30;
	calibration_points[CALIBRATION_TOP_LEFT].y = 30;
	calibration_points[CALIBRATION_BOTTOM_LEFT].x = 30;
	calibration_points[CALIBRATION_BOTTOM_LEFT].y = HW_LCD_HEIGHT - 31;
	calibration_points[CALIBRATION_TOP_RIGHT].x = HW_LCD_WIDTH - 31;
	calibration_points[CALIBRATION_TOP_RIGHT].y = 30;
	calibration_points[CALIBRATION_BOTTOM_RIGHT].x = HW_LCD_WIDTH - 31;
	calibration_points[CALIBRATION_BOTTOM_RIGHT].y = HW_LCD_HEIGHT - 31;
	failed_count = 0;
	return calibration_state;
}

static void calibration_reset(void)
{
	calibration.x = TOUCH_CALIBRATION_X;
	calibration.y = TOUCH_CALIBRATION_Y;
	calibration.offset_x = TOUCH_OFFSET_X;
	calibration.offset_y = TOUCH_OFFSET_Y;
	calibration.orientation = TOUCH_ORIENTATION;
}
static bool need_calibration(void)
{
	return !calibration.offset_x && !calibration.offset_y && !calibration.x && !calibration.y;
}

void calibration_end(void) {
	calibration_state = CALIBRATION_NONE;
}
calibrationState get_calibration_state(void)
{
	return calibration_state;
}
bool calibration_loaded(void) {
	if (need_calibration()) calibration_reset();
	return !need_calibration();
}

static void validate_calibration() {
  #define VALIDATE_PRECISION(XY, A, B) validate_precision_##XY(CALIBRATION_##A, CALIBRATION_##B)
  const bool landscape = VALIDATE_PRECISION(x, TOP_LEFT, BOTTOM_LEFT)
                      && VALIDATE_PRECISION(x, TOP_RIGHT, BOTTOM_RIGHT)
                      && VALIDATE_PRECISION(y, TOP_LEFT, TOP_RIGHT)
                      && VALIDATE_PRECISION(y, BOTTOM_LEFT, BOTTOM_RIGHT);
  const bool portrait = VALIDATE_PRECISION(y, TOP_LEFT, BOTTOM_LEFT)
                     && VALIDATE_PRECISION(y, TOP_RIGHT, BOTTOM_RIGHT)
                     && VALIDATE_PRECISION(x, TOP_LEFT, TOP_RIGHT)
                     && VALIDATE_PRECISION(x, BOTTOM_LEFT, BOTTOM_RIGHT);
  #undef VALIDATE_PRECISION

  #define CAL_PTS(N) calibration_points[CALIBRATION_##N]
  if (landscape) {
    calibration_state = CALIBRATION_SUCCESS;
    calibration.x = ((CAL_PTS(TOP_RIGHT).x - CAL_PTS(TOP_LEFT).x) << 17) / (CAL_PTS(BOTTOM_RIGHT).raw_x + CAL_PTS(TOP_RIGHT).raw_x - CAL_PTS(BOTTOM_LEFT).raw_x - CAL_PTS(TOP_LEFT).raw_x);
    calibration.y = ((CAL_PTS(BOTTOM_LEFT).y - CAL_PTS(TOP_LEFT).y) << 17) / (CAL_PTS(BOTTOM_RIGHT).raw_y - CAL_PTS(TOP_RIGHT).raw_y + CAL_PTS(BOTTOM_LEFT).raw_y - CAL_PTS(TOP_LEFT).raw_y);
    calibration.offset_x = CAL_PTS(TOP_LEFT).x - (int16_t)(((CAL_PTS(TOP_LEFT).raw_x + CAL_PTS(BOTTOM_LEFT).raw_x) * calibration.x) >> 17);
    calibration.offset_y = CAL_PTS(TOP_LEFT).y - (int16_t)(((CAL_PTS(TOP_LEFT).raw_y + CAL_PTS(TOP_RIGHT).raw_y) * calibration.y) >> 17);
    calibration.orientation = TOUCH_LANDSCAPE;
  }
  else if (portrait) {
    calibration_state = CALIBRATION_SUCCESS;
    calibration.x = ((CAL_PTS(TOP_RIGHT).x - CAL_PTS(TOP_LEFT).x) << 17) / (CAL_PTS(BOTTOM_RIGHT).raw_y + CAL_PTS(TOP_RIGHT).raw_y - CAL_PTS(BOTTOM_LEFT).raw_y - CAL_PTS(TOP_LEFT).raw_y);
    calibration.y = ((CAL_PTS(BOTTOM_LEFT).y - CAL_PTS(TOP_LEFT).y) << 17) / (CAL_PTS(BOTTOM_RIGHT).raw_x - CAL_PTS(TOP_RIGHT).raw_x + CAL_PTS(BOTTOM_LEFT).raw_x - CAL_PTS(TOP_LEFT).raw_x);
    calibration.offset_x = CAL_PTS(TOP_LEFT).x - (int16_t)(((CAL_PTS(TOP_LEFT).raw_y + CAL_PTS(BOTTOM_LEFT).raw_y) * calibration.x) >> 17);
    calibration.offset_y = CAL_PTS(TOP_LEFT).y - (int16_t)(((CAL_PTS(TOP_LEFT).raw_x + CAL_PTS(TOP_RIGHT).raw_x) * calibration.y) >> 17);
    calibration.orientation = TOUCH_PORTRAIT;
  }
  else {
    calibration_state = CALIBRATION_FAIL;
    calibration_reset();
    if (need_calibration() && failed_count++ < TOUCH_CALIBRATION_MAX_RETRIES) calibration_state = CALIBRATION_TOP_LEFT;
  }
  #undef CAL_PTS

  if (calibration_state == CALIBRATION_SUCCESS) {
  	cliPrintf("Touch screen calibration completed\r\n");
  	cliPrintf("TOUCH_CALIBRATION_X : %d\r\n", calibration.x);
  	cliPrintf("TOUCH_CALIBRATION_Y : %d\r\n", calibration.y);
  	cliPrintf("TOUCH_OFFSET_X : %d\r\n", calibration.offset_x);
  	cliPrintf("TOUCH_OFFSET_Y : %d\r\n", calibration.offset_y);
    //SERIAL_ECHO_TERNARY(calibration.orientation == TOUCH_LANDSCAPE, "TOUCH_ORIENTATION ", "TOUCH_LANDSCAPE", "TOUCH_PORTRAIT", "\n");
    //TERN_(TOUCH_CALIBRATION_AUTO_SAVE, settings.save());
  }
}

bool handleTouch(uint16_t x, uint16_t y) {
  static millis_t next_button_update_ms = 0;
  const millis_t now = millis();
  if (PENDING(now, next_button_update_ms)) return false;
  next_button_update_ms = now + BUTTON_DELAY_MENU;

  if (calibration_state < CALIBRATION_SUCCESS) {
    calibration_points[calibration_state].raw_x = x;
    calibration_points[calibration_state].raw_y = y;
    cliPrintf("TouchCalibration - State: %6d, x: %6d, raw_x : %6d, y: %6d, raw_y: %6d\r\n", calibration_state, calibration_points[calibration_state].x, x,calibration_points[calibration_state].y, y);
  }

  switch (calibration_state) {
    case CALIBRATION_TOP_LEFT: calibration_state = CALIBRATION_BOTTOM_LEFT; break;
    case CALIBRATION_BOTTOM_LEFT: calibration_state = CALIBRATION_TOP_RIGHT; break;
    case CALIBRATION_TOP_RIGHT: calibration_state = CALIBRATION_BOTTOM_RIGHT; break;
    case CALIBRATION_BOTTOM_RIGHT: validate_calibration(); break;
    default: break;
  }

  return true;
}
