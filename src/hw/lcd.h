/*
 * lcd.h
 *
 *  Created on: Jan 25, 2024
 *      Author: 차용선
 */

#ifndef SRC_HW_LCD_H_
#define SRC_HW_LCD_H_

#include "hw_def.h"
#include "hw.h"
#include "ST7796_Defines.h"

#ifdef _USE_HW_LCD

#define TFT_CS_L 		gpioPinWrite(TFT_CS,  _DEF_LOW);
#define TFT_CS_H 		gpioPinWrite(TFT_CS,  _DEF_HIGH);
#define TFT_DC_C 		gpioPinWrite(TFT_DC,  _DEF_LOW);
#define TFT_DC_D 		gpioPinWrite(TFT_DC,  _DEF_HIGH);
#define TFT_RST_L 		gpioPinWrite(TFT_RST, _DEF_LOW);
#define TFT_RST_H 		gpioPinWrite(TFT_RST, _DEF_HIGH);
#define TFT_BKL_L 		gpioPinWrite(TFT_BKL, _DEF_LOW);
#define TFT_BKL_H 		gpioPinWrite(TFT_BKL, _DEF_HIGH);


#define GETR(c) (((uint16_t)(c)) >> 11)
#define GETG(c) (((c) & 0x07E0)>>5)
#define GETB(c) ((c) & 0x1F)
#define RGB2COLOR(r, g, b) ((((r>>3)<<11) | ((g>>2)<<5) | (b>>3)))

// If the SPI frequency is not defined, set a default
#ifndef SPI_FREQUENCY
  #define SPI_FREQUENCY  40000000
#endif

// If the XPT2046 SPI frequency is not defined, set a default
#ifndef SPI_TOUCH_FREQUENCY
  #define SPI_TOUCH_FREQUENCY  2500000
#endif

#define TFT_BUFFER_SIZE 14400 //480*30

/***************************************************************************************
**                         Section 6: Colour enumeration
***************************************************************************************/
// Default color definitions
#define TFT_BLACK       0x0000      /*   0,   0,   0 */
#define TFT_NAVY        0x000F      /*   0,   0, 128 */
#define TFT_DARKGREEN   0x03E0      /*   0, 128,   0 */
#define TFT_DARKCYAN    0x03EF      /*   0, 128, 128 */
#define TFT_MAROON      0x7800      /* 128,   0,   0 */
#define TFT_PURPLE      0x780F      /* 128,   0, 128 */
#define TFT_OLIVE       0x7BE0      /* 128, 128,   0 */
#define TFT_LIGHTGREY   0xD69A      /* 211, 211, 211 */
#define TFT_DARKGREY    0x7BEF      /* 128, 128, 128 */
#define TFT_BLUE        0x001F      /*   0,   0, 255 */
#define TFT_GREEN       0x07E0      /*   0, 255,   0 */
#define TFT_CYAN        0x07FF      /*   0, 255, 255 */
#define TFT_RED         0xF800      /* 255,   0,   0 */
#define TFT_MAGENTA     0xF81F      /* 255,   0, 255 */
#define TFT_YELLOW      0xFFE0      /* 255, 255,   0 */
#define TFT_WHITE       0xFFFF      /* 255, 255, 255 */
#define TFT_ORANGE      0xFDA0      /* 255, 180,   0 */
#define TFT_GREENYELLOW 0xB7E0      /* 180, 255,   0 */
#define TFT_PINK        0xFE19      /* 255, 192, 203 */ //Lighter pink, was 0xFC9F
#define TFT_BROWN       0x9A60      /* 150,  75,   0 */
#define TFT_GOLD        0xFEA0      /* 255, 215,   0 */
#define TFT_SILVER      0xC618      /* 192, 192, 192 */
#define TFT_SKYBLUE     0x867D      /* 135, 206, 235 */
#define TFT_VIOLET      0x915C      /* 180,  46, 226 */

typedef struct lcd_driver_t_ lcd_driver_t;

typedef struct lcd_driver_t_
{
  bool     (*init)(void);
  bool     (*reset)(void);
  void     (*setWindow)(int32_t x, int32_t y, int32_t w, int32_t h);
  uint16_t (*getWidth)(void);
  uint16_t (*getHeight)(void);
  bool     (*setCallBack)(void (*p_func)(void));
  bool     (*sendBuffer)(uint8_t *p_data, uint32_t length, uint32_t timeout_ms);
  void     (*fillRect)(int32_t x, int32_t y, int32_t w, int32_t h, uint32_t color);

  uint16_t _width;
  uint16_t _height;

} lcd_driver_t;

extern lcd_driver_t lcd;

bool lcdInit(void);
bool lcdIsInit(void);
void lcdReset(void);

uint8_t lcdGetBackLight(void);
void    lcdSetBackLight(uint8_t value);

uint32_t lcdReadPixel(uint16_t x_pos, uint16_t y_pos);
void lcdClear(uint32_t rgb_code);
void lcdClearBuffer(uint32_t rgb_code);

bool lcdDrawAvailable(void);
bool lcdRequestDraw(void);
void lcdUpdateDraw(void);
void lcdSetWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
void lcdSendBuffer(uint8_t *p_data, uint32_t length, uint32_t timeout_ms);
void lcdDisplayOff(void);
void lcdDisplayOn(void);

uint32_t lcdGetFps(void);
uint32_t lcdGetFpsTime(void);
uint32_t lcdGetDrawTime(void);

int32_t lcdGetWidth(void);
int32_t lcdGetHeight(void);

uint16_t *lcdGetFrameBuffer(void);
uint16_t *lcdGetCurrentFrameBuffer(void);
void lcdSetDoubleBuffer(bool enable);

void lcdDrawPixel(uint16_t x_pos, uint16_t y_pos, uint32_t rgb_code);
void lcdDrawPixelMix(uint16_t x_pos, uint16_t y_pos, uint32_t rgb_code, uint8_t mix);
void lcdDrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,uint16_t color);
void lcdDrawVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
void lcdDrawHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
void lcdDrawFillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void lcdDrawFillScreen(uint16_t color);
void lcdDrawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void lcdDrawFillCircle(int32_t x0, int32_t y0, int32_t r, uint16_t color);
void lcdDrawRoundRect(int32_t x, int32_t y, int32_t w, int32_t h, int32_t r, uint32_t color);
void lcdDrawFillRoundRect(int32_t x, int32_t y, int32_t w, int32_t h, int32_t r, uint32_t color);
void lcdDrawTriangle(int32_t x1, int32_t y1, int32_t x2, int32_t y2, int32_t x3, int32_t y3, uint32_t color);
void lcdDrawFillTriangle(int32_t x1, int32_t y1, int32_t x2, int32_t y2, int32_t x3, int32_t y3, uint32_t color);
void lcdDrawString(int x, int y, uint16_t color, const char *str);


void LCD_Draw_Logo(void);

void WriteSequence(uint16_t *Data, uint16_t Count);
void WriteMultiple(uint16_t Color, uint32_t Count);

void lcdDrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data);
void lcdDrawBufferImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data);
void drawBitmap(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color);
void drawPixel(int32_t x, int32_t y, uint32_t color);
void DrawBitmap(uint16_t w, uint16_t h, uint8_t *s);
void DrawBitmapDMA(uint16_t w, uint16_t h, uint8_t *s);
void pushPixelsDMA(uint16_t* image, uint32_t len);


#endif /* _USE_HW_LCD */

#endif /* SRC_HW_LCD_H_ */
