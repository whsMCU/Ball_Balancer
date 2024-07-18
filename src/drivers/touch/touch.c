/*
 * touch.c
 *
 *  Created on: Jan 26, 2024
 *      Author: 차용선
 */

#include "touch.h"
#include "draw_touch_calibration.h"
#include "touch_calibration.h"

tft_touch_t touch;

/***************************************************************************************
** Function name:           begin_touch_read_write - was spi_begin_touch
** Description:             Start transaction and select touch controller
***************************************************************************************/
// The touch controller has a low SPI clock rate
static inline void begin_touch_read_write(void){
	TFT_CS_H; // Just in case it has been left low
	spiSetBitWidth(_DEF_SPI1, 8);
	SPI_Set_Speed_hz(_DEF_SPI1, SPI_TOUCH_FREQUENCY);
  Toutch_CS_L;
}

/***************************************************************************************
** Function name:           end_touch_read_write - was spi_end_touch
** Description:             End transaction and deselect touch controller
***************************************************************************************/
static inline void end_touch_read_write(void){
	Toutch_CS_H;
  SPI_Set_Speed_hz(_DEF_SPI1, SPI_FREQUENCY);
}

#ifdef _USE_HW_CLI
static void cliTouch(cli_args_t *args);
#endif

void touch_init(void)
{
	Init_touch_param(&touch);

#ifdef _USE_HW_CLI
  cliAdd("touch", cliTouch);
#endif

}

void Init_touch_param(tft_touch_t *param)
{
	param->touchCalibration_x0 = 300;
	param->touchCalibration_x1 = 3600;
	param->touchCalibration_y0 = 300;
	param->touchCalibration_y1 = 3600;
	param->touchCalibration_rotate = 1;
	param->touchCalibration_invert_x = 2;
	param->touchCalibration_invert_y = 0;
}
uint16_t delta(uint16_t a, uint16_t b) { return a > b ? a - b : b - a; }
/***************************************************************************************
** Function name:           getTouchRaw
** Description:             read raw touch position.  Always returns true.
***************************************************************************************/
uint16_t getTouchRaw(uint8_t coordinate){
  uint8_t buf[2] = {0, 0};
  uint16_t data[3] = {0, 0, 0};
  uint16_t tmp;

  begin_touch_read_write();

  for(int i = 0; i < 3; i++)
  {
  	spiReadRegBuf(_DEF_SPI1, coordinate, buf, 2);
  	tmp = buf[0] << 4;
  	data[i] = (tmp | (buf[1] >> 4));
  }

  end_touch_read_write();

	uint16_t delta01 = delta(data[0], data[1]);
	uint16_t delta02 = delta(data[0], data[2]);
	uint16_t delta12 = delta(data[1], data[2]);

	if (delta01 > delta02 || delta01 > delta12) {
		if (delta02 > delta12)
		{
			data[0] = data[2];
		}
		else
		{
			data[1] = data[2];
		}
	}
  return (data[0] + data[1]) >> 1;
}

bool isTouched() {
  return getTouchRaw(XPT2046_Z1) >= XPT2046_Z1_THRESHOLD;
}

bool getRawPoint(int16_t *x, int16_t *y) {
  if (!isTouched()) return false;
  *x = getTouchRaw(XPT2046_X);
  *y = getTouchRaw(XPT2046_Y);
  return isTouched();
}

bool get_point(int16_t *x, int16_t *y) {
  if (!getRawPoint(x, y)) { return false; }

  #if ENABLED(TOUCH_SCREEN_CALIBRATION)
    const calibrationState state = touch_calibration.get_calibration_state();
    if (state >= CALIBRATION_TOP_LEFT && state <= CALIBRATION_BOTTOM_RIGHT) {
      if (touch_calibration.handleTouch(*x, *y)) lv_update_touch_calibration_screen();
      return false;
    }
    *x = (int16_t)(((int32_t)(*x) * touch_calibration.calibration.x) >> 16) + touch_calibration.calibration.offset_x;
    *y = (int16_t)(((int32_t)(*y) * touch_calibration.calibration.y) >> 16) + touch_calibration.calibration.offset_y;
  #else
    *x = (int16_t)(((int32_t)(*x) * TOUCH_CALIBRATION_X) >> 16) + TOUCH_OFFSET_X;
    *y = (int16_t)(((int32_t)(*y) * TOUCH_CALIBRATION_Y) >> 16) + TOUCH_OFFSET_Y;
  #endif

  return true;
}


/***************************************************************************************
** Function name:           getTouch
** Description:             read callibrated position. Return false if not pressed.
***************************************************************************************/
uint8_t getTouch(uint16_t *x, uint16_t *y, uint16_t threshold){
  uint16_t x_tmp, y_tmp;

  if (threshold<20) threshold = 20;
  if (touch._pressTime > millis()) threshold=20;

  uint8_t n = 5;
  uint8_t valid = 0;
  while (n--)
  {
    if (validTouch(&x_tmp, &y_tmp, threshold)) valid++;;
  }

  if (valid<1) { touch._pressTime = 0; return false; }

  touch._pressTime = millis() + 50;

  convertRawXY(&x_tmp, &y_tmp);

  if (x_tmp >= HW_LCD_WIDTH || y_tmp >= HW_LCD_HEIGHT) return false;

  touch._pressX = x_tmp;
  touch._pressY = y_tmp;
  *x = touch._pressX;
  *y = touch._pressY;
  return valid;
}

/***************************************************************************************
** Function name:           convertRawXY
** Description:             convert raw touch x,y values to screen coordinates
***************************************************************************************/
void convertRawXY(uint16_t *x, uint16_t *y)
{
  uint16_t x_tmp = *x, y_tmp = *y, xx, yy;

  if(!touch.touchCalibration_rotate){
    xx=(x_tmp-touch.touchCalibration_x0)*HW_LCD_WIDTH/touch.touchCalibration_x1;
    yy=(y_tmp-touch.touchCalibration_y0)*HW_LCD_HEIGHT/touch.touchCalibration_y1;
    if(touch.touchCalibration_invert_x)
      xx = HW_LCD_WIDTH - xx;
    if(touch.touchCalibration_invert_y)
      yy = HW_LCD_HEIGHT - yy;
  } else {
    xx=(y_tmp-touch.touchCalibration_x0)*HW_LCD_WIDTH/touch.touchCalibration_x1;
    yy=(x_tmp-touch.touchCalibration_y0)*HW_LCD_HEIGHT/touch.touchCalibration_y1;
    if(touch.touchCalibration_invert_x)
      xx =HW_LCD_WIDTH - xx;
    if(touch.touchCalibration_invert_y)
      yy = HW_LCD_HEIGHT - yy;
  }
  *x = xx;
  *y = yy;
}

/***************************************************************************************
** Function name:           setTouch
** Description:             imports calibration parameters for touchscreen.
***************************************************************************************/
void setTouch(uint16_t *parameters){
	touch.touchCalibration_x0 = parameters[0];
	touch.touchCalibration_x1 = parameters[1];
	touch.touchCalibration_y0 = parameters[2];
	touch.touchCalibration_y1 = parameters[3];

  if(touch.touchCalibration_x0 == 0) touch.touchCalibration_x0 = 1;
  if(touch.touchCalibration_x1 == 0) touch.touchCalibration_x1 = 1;
  if(touch.touchCalibration_y0 == 0) touch.touchCalibration_y0 = 1;
  if(touch.touchCalibration_y1 == 0) touch.touchCalibration_y1 = 1;

  touch.touchCalibration_rotate = parameters[4] & 0x01;
  touch.touchCalibration_invert_x = parameters[4] & 0x02;
  touch.touchCalibration_invert_y = parameters[4] & 0x04;
}

#ifdef _USE_HW_CLI
void cliTouch(cli_args_t *args)
{
  bool ret = false;

  if (args->argc == 1 && args->isStr(0, "show") == true)
  {
    while(cliKeepLoop())
    {
    	cliPrintf("x: %6d     ", getTouchRaw(XPT2046_X));

    	cliPrintf("y: %6d     ", getTouchRaw(XPT2046_Y));

    	cliPrintf("z: %6d \r\n     ", getTouchRaw(XPT2046_Z1));

    	delay(100);
    }
    ret = true;
  }

  if (args->argc == 1 && args->isStr(0, "calibration") == true)
  {
  	int16_t x, y;
  	lcd.fillRect(0, 0, lcd._width, lcd._height, TFT_BLACK);
		draw_touch_calibration_screen();
    while(cliKeepLoop())
    {
    	if(!getRawPoint(&x, &y))
    	{
    		continue;
    	}
    	const calibrationState state = get_calibration_state();
      if (state >= CALIBRATION_TOP_LEFT && state <= CALIBRATION_BOTTOM_RIGHT)
      {
				if(handleTouch(x, y))draw_touch_calibration_screen();
      }
    }
    ret = true;
  }

  if (ret != true)
  {
    cliPrintf("touch show\r\n");
    cliPrintf("touch calibration\r\n");
  }
}
#endif



