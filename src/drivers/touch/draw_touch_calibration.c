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

#include "hw.h"


#include "draw_touch_calibration.h"
#include "touch_calibration.h"
#include "lcd.h"


enum {
  ID_TC_RETURN = 1
};

static void drawCross(uint16_t x, uint16_t y, uint16_t color) {
  lcd.setWindow(x - 15, y, x + 15, y);
  WriteSequence(&color, 31);
  lcd.setWindow(x, y - 15, x, y + 15);
  WriteSequence(&color, 31);
}

void update_touch_calibration_screen() {
  uint16_t x, y;

  calibrationState calibration_stage = get_calibration_state();
  if (calibration_stage == CALIBRATION_NONE) {
    // start and clear screen
    calibration_stage = calibration_start();
  }
  else {
    // clear last cross
    x = calibration_points[_MIN(calibration_stage - 1, CALIBRATION_BOTTOM_RIGHT)].x;
    y = calibration_points[_MIN(calibration_stage - 1, CALIBRATION_BOTTOM_RIGHT)].y;
    drawCross(x, y, TFT_BLACK);
  }

  const char *str = NULL;
  if (calibration_stage < CALIBRATION_SUCCESS) {
    // handle current state
    switch (calibration_stage) {
      case CALIBRATION_TOP_LEFT:     str = "TOP_LEFT\r\n"; break;
      case CALIBRATION_BOTTOM_LEFT:  str = "BOTTOM_LEFT\r\n"; break;
      case CALIBRATION_TOP_RIGHT:    str = "TOP_RIGHT\r\n"; break;
      case CALIBRATION_BOTTOM_RIGHT: str = "BOTTOM_RIGHT\r\n"; break;
      default: break;
    }

    x = calibration_points[calibration_stage].x;
    y = calibration_points[calibration_stage].y;
    drawCross(x, y, TFT_WHITE);
  }
  else {
    // end calibration
    str = calibration_stage == CALIBRATION_SUCCESS ? "CALIBRATION_COMPLETED\r\n" : "CALIBRATION_FAILED\r\n";
    calibration_end();
    //lv_big_button_create(scr, "F:/bmp_return.bin", common_menu.text_back, BTN_X_PIXEL * 3 + INTERVAL_V * 4, BTN_Y_PIXEL + INTERVAL_H + titleHeight, event_handler, ID_TC_RETURN);
  }

  // draw current message
  cliPrintf(str);
  //lv_obj_align(status_label, nullptr, LV_ALIGN_CENTER, 0, 0);
}

void draw_touch_calibration_screen() {
  update_touch_calibration_screen();
}

void clear_touch_calibration_screen() {
  //lv_obj_del(scr);
}
