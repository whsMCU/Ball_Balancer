/*
 * touch.h
 *
 *  Created on: Jan 26, 2024
 *      Author: 차용선
 */

#ifndef SRC_DRIVERS_TOUCH_TOUCH_H_
#define SRC_DRIVERS_TOUCH_TOUCH_H_

#include "hw.h"

#define Toutch_CS_L 		gpioPinWrite(Toutch_CS,  _DEF_LOW);
#define Toutch_CS_H 		gpioPinWrite(Toutch_CS,  _DEF_HIGH);

#define TOUCH_ORIENTATION_NONE  0
#define TOUCH_LANDSCAPE         1
#define TOUCH_PORTRAIT          2

#ifndef TOUCH_CALIBRATION_X
	#define TOUCH_CALIBRATION_X           -17341
#endif
#ifndef TOUCH_CALIBRATION_Y
	#define TOUCH_CALIBRATION_Y            11598
#endif
#ifndef TOUCH_OFFSET_X
	#define TOUCH_OFFSET_X                   507
#endif
#ifndef TOUCH_OFFSET_Y
	#define TOUCH_OFFSET_Y                   -25
#endif
#ifndef TOUCH_ORIENTATION
	#define TOUCH_ORIENTATION    TOUCH_LANDSCAPE
#endif

#define BUTTON_DELAY_EDIT  50 // (ms) Button repeat delay for edit screens
#define BUTTON_DELAY_MENU 250 // (ms) Button repeat delay for menus


#define XPT2046_DFR_MODE        0x00
#define XPT2046_SER_MODE        0x04
#define XPT2046_CONTROL         0x80


#define XPT2046_X  (0x10 | XPT2046_CONTROL | XPT2046_DFR_MODE)
#define XPT2046_Y  (0x50 | XPT2046_CONTROL | XPT2046_DFR_MODE)
#define XPT2046_Z1 (0x30 | XPT2046_CONTROL | XPT2046_DFR_MODE)
#define XPT2046_Z2 (0x40 | XPT2046_CONTROL | XPT2046_DFR_MODE)

#define XPT2046_Z1_THRESHOLD 10

typedef struct
{
	// Initialise with example calibration values so processor does not crash if setTouch() not called in setup()
	uint16_t touchCalibration_x0, touchCalibration_x1, touchCalibration_y0, touchCalibration_y1;
	uint8_t  touchCalibration_rotate, touchCalibration_invert_x, touchCalibration_invert_y;

	uint32_t _pressTime;        // Press and hold time-out
	uint16_t _pressX, _pressY;  // For future use (last sampled calibrated coordinates)
}tft_touch_t; // End of class TFT_eSPI

void 	 touch_init(void);
void 	 Init_touch_param(tft_touch_t *param);
	   // Get raw x,y ADC values from touch controller
uint16_t  getTouchRaw(uint8_t coordinate);
bool isTouched(void);
bool getRawPoint(int16_t *x, int16_t *y);
bool get_point(int16_t *x, int16_t *y);
	   // Convert raw x,y values to calibrated and correctly rotated screen coordinates
void     convertRawXY(uint16_t *x, uint16_t *y);
	   // Get the screen touch coordinates, returns true if screen has been touched
	   // if the touch coordinates are off screen then x and y are not updated
	   // The returned value can be treated as a bool type, false or 0 means touch not detected
	   // In future the function may return an 8-bit "quality" (jitter) value.
	   // The threshold value is optional, this must be higher than the bias level for z (pressure)
	   // reported by Test_Touch_Controller when the screen is NOT touched. When touched the z value
	   // must be higher than the threshold for a touch to be detected.
uint8_t  getTouch(uint16_t *x, uint16_t *y, uint16_t threshold); //600

	   // Set the screen calibration values
void     setTouch(uint16_t *data);

//private:

	   // Handlers for the touch controller bus settings
//inline void begin_touch_read_write() __attribute__((always_inline));
//inline void end_touch_read_write()   __attribute__((always_inline));

	   // Private function to validate a touch, allow settle time and reduce spurious coordinates
uint8_t  validTouch(uint16_t *x, uint16_t *y, uint16_t threshold); //600

#endif /* SRC_DRIVERS_TOUCH_TOUCH_H_ */
