/*
 * lcd.c
 *
 *  Created on: Jan 25, 2024
 *      Author: 차용선
 */


#include "lcd.h"
#include "cli.h"

#include "lcd/st7796.h"

#ifdef _USE_HW_LCD

#ifndef _swap_int16_t
#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }
#endif

#define MAKECOL(r, g, b) ( ((r)<<11) | ((g)<<5) | (b))

#define LCD_OPT_DEF   __attribute__((optimize("O2")))

typedef struct
{
  int16_t x;
  int16_t y;
} lcd_pixel_t;


lcd_driver_t lcd;

static bool is_init = false;
static volatile bool is_tx_done = true;
static uint8_t backlight_value = 100;

static bool lcd_request_draw = false;

static volatile uint32_t fps_pre_time;
static volatile uint32_t fps_time;
static volatile uint32_t fps_count = 0;

//static volatile uint32_t draw_fps = 30;
static volatile uint32_t draw_frame_time = 0;


static uint16_t *p_draw_frame_buf = NULL;
//static uint16_t __attribute__((aligned(64))) frame_buffer[2][14400];

#define LOGO_MAX_SIZE_TFT35             (300 * 1024)
uint8_t bmp_public_buf[14 * 1024];

//static volatile bool requested_from_thread = false;

static void lcdDrawLineBuffer(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color, lcd_pixel_t *line);

#ifdef _USE_HW_CLI
static void cliLcd(cli_args_t *args);
#endif

void TransferDoneISR(void)
{
  fps_time = millis() - fps_pre_time;
  fps_pre_time = millis();

  if (fps_time > 0)
  {
    fps_count = 1000 / fps_time;
  }

  lcd_request_draw = false;
}

bool lcdInit(void)
{
	backlight_value = 100;

	TFT_RST_H;
	delay(5);
	TFT_RST_L;
	delay(20);
	TFT_RST_H;

	st7796InitDriver(&lcd);
	is_init = st7796Init();

	lcd.setCallBack(TransferDoneISR);

	lcd.fillRect(0, 0, lcd._width, lcd._height, TFT_BLACK);
//	drawBitmap(0, 0, (const uint8_t *)background, 320, 240, TFT_RED);

//	lcdDrawFillRect(0, 0, lcd._width, lcd._height, TFT_RED);
//	lcdUpdateDraw();

	lcdSetBackLight(100);

	if (is_init != true)
	{
		return false;
	}

	#ifdef _USE_HW_CLI
		cliAdd("lcd", cliLcd);
	#endif

	return true;
}

// Robin_pro pic addr
#define PIC_NAME_ADDR                 0x003000      // Pic information addr
#define PIC_SIZE_ADDR                 0x007000      // Pic size information addr
#define PIC_COUNTER_ADDR              0x008000      // Pic total number
#define PIC_LOGO_ADDR                 0x009000      // Logo addr

uint32_t logo_addroffset = 0;
void Pic_Logo_Read(uint8_t *LogoName, uint8_t *Logo_Rbuff, uint32_t LogoReadsize) {
  SPI_FLASH_BufferRead(Logo_Rbuff, PIC_LOGO_ADDR + logo_addroffset, LogoReadsize);
  //memcpy(Logo_Rbuff, marlin_logo_480x320x16 + logo_addroffset, LogoReadsize);
  logo_addroffset += LogoReadsize;
  if (logo_addroffset >= LOGO_MAX_SIZE_TFT35)
    logo_addroffset = 0;
}

void LCD_Draw_Logo(void)
{
    lcd.setWindow(0, 0, lcd._width-1, lcd._height-1);
    for (uint16_t i = 0; i < (lcd._height); i++) {
      Pic_Logo_Read((uint8_t *)"", (uint8_t *)bmp_public_buf, (lcd._width)*2);
      WriteSequence((uint16_t *)bmp_public_buf, lcd._width);
    }
}
uint32_t lcdGetDrawTime(void)
{
  return draw_frame_time;
}

bool lcdIsInit(void)
{
  return is_init;
}

void lcdReset(void)
{
  lcd.reset();
}

uint8_t lcdGetBackLight(void)
{
  return backlight_value;
}

void lcdSetBackLight(uint8_t value)
{
  value = constrain(value, 0, 100);

  if (value != backlight_value)
  {
    backlight_value = value;
  }

  if (backlight_value > 0)
  {
	  TFT_BKL_H;
  }
  else
  {
	  TFT_BKL_L;
  }
}

LCD_OPT_DEF uint32_t lcdReadPixel(uint16_t x_pos, uint16_t y_pos)
{
  return p_draw_frame_buf[y_pos * lcd._width + x_pos];
}

LCD_OPT_DEF void lcdDrawPixel(uint16_t x_pos, uint16_t y_pos, uint32_t rgb_code)
{
  p_draw_frame_buf[y_pos * lcd._width + x_pos] = rgb_code;
}

void WriteSequence(uint16_t *Data, uint16_t Count)
{
  spiSetBitWidth(_DEF_SPI1, 16);
  TFT_DC_D;
  TFT_CS_L;
  spiDmaTxTransfer(_DEF_SPI1, (void*)Data, Count, 100);

  TFT_CS_H;
}

void WriteMultiple(uint16_t Color, uint32_t Count)
{
  static uint16_t Data;

  Data = Color;
  spiSetBitWidth(_DEF_SPI1, 16);
  TFT_DC_D;
  TFT_CS_L;
  while (Count > 0) {
  	if(spiDmaTxTransfer(_DEF_SPI1, (void*)&Data, Count > 0xFFFF ? 0xFFFF : Count, 100) != true)
    Count = Count > 0xFFFF ? Count - 0xFFFF : 0;
  }
  TFT_CS_H;
}

LCD_OPT_DEF void lcdClear(uint32_t rgb_code)
{
//  lcdClearBuffer(rgb_code);
//
//  lcdUpdateDraw();
  lcd.setWindow(0, 0, lcd._width-1, lcd._height-1);
  WriteMultiple(rgb_code, (uint32_t)lcd._width * (uint32_t)lcd._height);

}

LCD_OPT_DEF void lcdClearBuffer(uint32_t rgb_code)
{
  uint16_t *p_buf = lcdGetFrameBuffer();

  for (int i=0; i<lcd._width * lcd._height; i++)
  {
    p_buf[i] = rgb_code;
  }
}

LCD_OPT_DEF void lcdDrawFillCircle(int32_t x0, int32_t y0, int32_t r, uint16_t color)
{
  int32_t  x  = 0;
  int32_t  dx = 1;
  int32_t  dy = r+r;
  int32_t  p  = -(r>>1);


  lcdDrawHLine(x0 - r, y0, dy+1, color);

  while(x<r)
  {

    if(p>=0) {
      dy-=2;
      p-=dy;
      r--;
    }

    dx+=2;
    p+=dx;

    x++;

    lcdDrawHLine(x0 - r, y0 + x, 2 * r+1, color);
    lcdDrawHLine(x0 - r, y0 - x, 2 * r+1, color);
    lcdDrawHLine(x0 - x, y0 + r, 2 * x+1, color);
    lcdDrawHLine(x0 - x, y0 - r, 2 * x+1, color);
  }
}

LCD_OPT_DEF void lcdDrawCircleHelper( int32_t x0, int32_t y0, int32_t r, uint8_t cornername, uint32_t color)
{
  int32_t f     = 1 - r;
  int32_t ddF_x = 1;
  int32_t ddF_y = -2 * r;
  int32_t x     = 0;

  while (x < r)
  {
    if (f >= 0)
    {
      r--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;
    if (cornername & 0x4)
    {
      lcdDrawPixel(x0 + x, y0 + r, color);
      lcdDrawPixel(x0 + r, y0 + x, color);
    }
    if (cornername & 0x2)
    {
      lcdDrawPixel(x0 + x, y0 - r, color);
      lcdDrawPixel(x0 + r, y0 - x, color);
    }
    if (cornername & 0x8)
    {
      lcdDrawPixel(x0 - r, y0 + x, color);
      lcdDrawPixel(x0 - x, y0 + r, color);
    }
    if (cornername & 0x1)
    {
      lcdDrawPixel(x0 - r, y0 - x, color);
      lcdDrawPixel(x0 - x, y0 - r, color);
    }
  }
}

LCD_OPT_DEF void lcdDrawRoundRect(int32_t x, int32_t y, int32_t w, int32_t h, int32_t r, uint32_t color)
{
  // smarter version
  lcdDrawHLine(x + r    , y        , w - r - r, color); // Top
  lcdDrawHLine(x + r    , y + h - 1, w - r - r, color); // Bottom
  lcdDrawVLine(x        , y + r    , h - r - r, color); // Left
  lcdDrawVLine(x + w - 1, y + r    , h - r - r, color); // Right

  // draw four corners
  lcdDrawCircleHelper(x + r        , y + r        , r, 1, color);
  lcdDrawCircleHelper(x + w - r - 1, y + r        , r, 2, color);
  lcdDrawCircleHelper(x + w - r - 1, y + h - r - 1, r, 4, color);
  lcdDrawCircleHelper(x + r        , y + h - r - 1, r, 8, color);
}

LCD_OPT_DEF void lcdDrawFillCircleHelper(int32_t x0, int32_t y0, int32_t r, uint8_t cornername, int32_t delta, uint32_t color)
{
  int32_t f     = 1 - r;
  int32_t ddF_x = 1;
  int32_t ddF_y = -r - r;
  int32_t y     = 0;

  delta++;

  while (y < r)
  {
    if (f >= 0)
    {
      r--;
      ddF_y += 2;
      f     += ddF_y;
    }

    y++;
    ddF_x += 2;
    f     += ddF_x;

    if (cornername & 0x1)
    {
      lcdDrawHLine(x0 - r, y0 + y, r + r + delta, color);
      lcdDrawHLine(x0 - y, y0 + r, y + y + delta, color);
    }
    if (cornername & 0x2)
    {
      lcdDrawHLine(x0 - r, y0 - y, r + r + delta, color); // 11995, 1090
      lcdDrawHLine(x0 - y, y0 - r, y + y + delta, color);
    }
  }
}

LCD_OPT_DEF void lcdDrawFillRoundRect(int32_t x, int32_t y, int32_t w, int32_t h, int32_t r, uint32_t color)
{
  // smarter version
  lcdDrawFillRect(x, y + r, w, h - r - r, color);

  // draw four corners
  lcdDrawFillCircleHelper(x + r, y + h - r - 1, r, 1, w - r - r - 1, color);
  lcdDrawFillCircleHelper(x + r, y + r        , r, 2, w - r - r - 1, color);
}

LCD_OPT_DEF void lcdDrawTriangle(int32_t x1, int32_t y1, int32_t x2, int32_t y2, int32_t x3, int32_t y3, uint32_t color)
{
  lcdDrawLine(x1, y1, x2, y2, color);
  lcdDrawLine(x1, y1, x3, y3, color);
  lcdDrawLine(x2, y2, x3, y3, color);
}

LCD_OPT_DEF void lcdDrawFillTriangle(int32_t x1, int32_t y1, int32_t x2, int32_t y2, int32_t x3, int32_t y3, uint32_t color)
{
  uint16_t max_line_size_12 = max(abs(x1-x2), abs(y1-y2));
  uint16_t max_line_size_13 = max(abs(x1-x3), abs(y1-y3));
  uint16_t max_line_size_23 = max(abs(x2-x3), abs(y2-y3));
  uint16_t max_line_size = max_line_size_12;
  uint16_t i = 0;

  if (max_line_size_13 > max_line_size)
  {
    max_line_size = max_line_size_13;
  }
  if (max_line_size_23 > max_line_size)
  {
    max_line_size = max_line_size_23;
  }

  lcd_pixel_t line[max_line_size];

  lcdDrawLineBuffer(x1, y1, x2, y2, color, line);
  for (i = 0; i < max_line_size_12; i++)
  {
    lcdDrawLine(x3, y3, line[i].x, line[i].y, color);
  }
  lcdDrawLineBuffer(x1, y1, x3, y3, color, line);
  for (i = 0; i < max_line_size_13; i++)
  {
    lcdDrawLine(x2, y2, line[i].x, line[i].y, color);
  }
  lcdDrawLineBuffer(x2, y2, x3, y3, color, line);
  for (i = 0; i < max_line_size_23; i++)
  {
    lcdDrawLine(x1, y1, line[i].x, line[i].y, color);
  }
}

uint32_t lcdGetFps(void)
{
  return fps_count;
}

uint32_t lcdGetFpsTime(void)
{
  return fps_time;
}

bool lcdDrawAvailable(void)
{
  return !lcd_request_draw;
}

void lcdSetWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  if (is_init != true)
  {
    return;
  }

  lcd.setWindow(x, y, w, h);
}

uint16_t *lcdGetFrameBuffer(void)
{
  return (uint16_t *)p_draw_frame_buf;
}

void lcdDisplayOff(void)
{
}

void lcdDisplayOn(void)
{
  lcdSetBackLight(lcdGetBackLight());
}

int32_t lcdGetWidth(void)
{
  return lcd._width;
}

int32_t lcdGetHeight(void)
{
  return lcd._height;
}


void lcdDrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
  int16_t steep = abs(y1 - y0) > abs(x1 - x0);

  if (x0 < 0) x0 = 0;
  if (y0 < 0) y0 = 0;
  if (x1 < 0) x1 = 0;
  if (y1 < 0) y1 = 0;


  if (steep)
  {
    _swap_int16_t(x0, y0);
    _swap_int16_t(x1, y1);
  }

  if (x0 > x1)
  {
    _swap_int16_t(x0, x1);
    _swap_int16_t(y0, y1);
  }

  int16_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int16_t err = dx / 2;
  int16_t ystep;

  if (y0 < y1)
  {
    ystep = 1;
  } else {
    ystep = -1;
  }

  for (; x0<=x1; x0++)
  {
    if (steep)
    {
      //lcdDrawPixel(y0, x0, color);
      drawPixel(y0, x0, color);
    } else
    {
      //lcdDrawPixel(x0, y0, color);
      drawPixel(y0, x0, color);
    }
    err -= dy;
    if (err < 0)
    {
      y0 += ystep;
      err += dx;
    }
  }
}

void lcdDrawLineBuffer(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color, lcd_pixel_t *line)
{
  int16_t steep = abs(y1 - y0) > abs(x1 - x0);

  if (x0 < 0) x0 = 0;
  if (y0 < 0) y0 = 0;
  if (x1 < 0) x1 = 0;
  if (y1 < 0) y1 = 0;


  if (steep)
  {
    _swap_int16_t(x0, y0);
    _swap_int16_t(x1, y1);
  }

  if (x0 > x1)
  {
    _swap_int16_t(x0, x1);
    _swap_int16_t(y0, y1);
  }

  int16_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int16_t err = dx / 2;
  int16_t ystep;

  if (y0 < y1)
  {
    ystep = 1;
  } else {
    ystep = -1;
  }

  for (; x0<=x1; x0++)
  {
    if (steep)
    {
      if (line != NULL)
      {
        line->x = y0;
        line->y = x0;
      }
      lcdDrawPixel(y0, x0, color);
    } else
    {
      if (line != NULL)
      {
        line->x = x0;
        line->y = y0;
      }
      lcdDrawPixel(x0, y0, color);
    }
    if (line != NULL)
    {
      line++;
    }
    err -= dy;
    if (err < 0)
    {
      y0 += ystep;
      err += dx;
    }
  }
}

void lcdDrawVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{
  lcdDrawLine(x, y, x, y+h-1, color);
}

void lcdDrawHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
{
  lcdDrawLine(x, y, x+w-1, y, color);
}

void lcdDrawFillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
  for (int16_t i=x; i<x+w; i++)
  {
    lcdDrawVLine(i, y, h, color);
  }
}

void lcdDrawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
  lcdDrawHLine(x, y, w, color);
  lcdDrawHLine(x, y+h-1, w, color);
  lcdDrawVLine(x, y, h, color);
  lcdDrawVLine(x+w-1, y, h, color);
}

void lcdDrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data) {
    if((x >= lcd._width) || (y >= lcd._height)) return;
    if((x + w - 1) >= lcd._width) return;
    if((y + h - 1) >= lcd._height) return;

    lcd.setWindow(x, y, x+w-1, y+h-1);
    lcd.sendBuffer((uint8_t *)data, sizeof(uint16_t)*w*h, 10);
}
void lcdDrawBufferImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data) {
    if((x >= lcd._width) || (y >= lcd._height)) return;
    if((x + w - 1) >= lcd._width) return;
    if((y + h - 1) >= lcd._height) return;

    uint16_t pixel_cnt = 0;

    for (int16_t i=y; i<y+h; i++)
    {
		for (int16_t j=x; j<x+w; j++)
		{
			lcdDrawPixel(j, i, data[pixel_cnt++]);
		}
    }
}

#define pgm_read_byte(addr)   (*(const unsigned char *)(addr))

void drawBitmap(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color)
{
  int32_t i, j, byteWidth = (w + 7) / 8;

  for (j = 0; j < h; j++) {
    for (i = 0; i < w; i++ ) {
      if (pgm_read_byte(bitmap + j * byteWidth + i / 8) & (128 >> (i & 7))) {
        drawPixel(x + i, y + j, color);
      }
    }
  }
  TFT_CS_H;
}

void drawPixel(int32_t x, int32_t y, uint32_t color)
{
  TFT_CS_L;

  writecommand(TFT_CASET);
  writedata((x >> 8) & 0xFF);
  writedata(x & 0xFF);

  writecommand(TFT_PASET);
  writedata((y >> 8) & 0xFF);
  writedata(y & 0xFF);

  writecommand(TFT_RAMWR); // write to RAM

  spiSetBitWidth(_DEF_SPI1, 16);

	TFT_DC_D;        // Play safe, but should already be in data mode
	TFT_CS_L;
	spiTransfer16(_DEF_SPI1, color);
  TFT_CS_H;
	spiSetBitWidth(_DEF_SPI1, 8);

}

static void ConvHL(uint8_t *s, int32_t l)
{
	uint8_t v;
	while (l > 0) {
		v = *(s+1);
		*(s+1) = *s;
		*s = v;
		s += 2;
		l -= 2;
	}
}

void DrawBitmap(uint16_t w, uint16_t h, uint8_t *s)
{
	TFT_DC_D;        // Play safe, but should already be in data mode
	TFT_CS_L;

  spiSetBitWidth(_DEF_SPI1, 8);
  ConvHL(s, (int32_t)w*h*2);
  HAL_SPI_Transmit(&hspi1, (uint8_t*)s, w * h *2, HAL_MAX_DELAY);
  TFT_CS_H;
	//spiSetBitWidth(_DEF_SPI1, 8);
}

void DrawBitmapDMA(uint16_t w, uint16_t h, uint8_t *s)
{
	TFT_DC_D;        // Play safe, but should already be in data mode
	TFT_CS_L;

  spiSetBitWidth(_DEF_SPI1, 16);
  //ConvHL(s, (int32_t)w*h*2);
  spiDmaTxStart(_DEF_SPI1, s, w * h);
  //HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*)s, w * h);
	//spiSetBitWidth(_DEF_SPI1, 8);
}

void pushPixelsDMA(uint16_t* image, uint32_t len)
{
  if (len == 0) return;

  // Wait for any current DMA transaction to end
  while (hspi1.State == HAL_SPI_STATE_BUSY_TX); // Check if SPI Tx is busy

//  if(_swapBytes) {
//    for (uint32_t i = 0; i < len; i++) (image[i] = image[i] << 8 | image[i] >> 8);
//  }

  spiDmaTxStart(_DEF_SPI1, (uint8_t*)image, len << 1);
}

void lcdDrawFillScreen(uint16_t color)
{
  lcdDrawFillRect(0, 0, lcd._width, lcd._height, color);
}

#ifdef _USE_HW_CLI
void cliLcd(cli_args_t *args)
{
  bool ret = false;


  if (args->argc == 1 && args->isStr(0, "test") == true)
  {
    //lcdSetFont(LCD_FONT_HAN);

    while(cliKeepLoop())
    {
      if (lcdDrawAvailable() == true)
      {
        lcdClearBuffer(TFT_BLACK);

//        lcdPrintf(25,16*0, TFT_GREEN, "[LCD 테스트]");
//
//        lcdPrintf(0,16*1, TFT_WHITE, "%d fps", lcdGetFps());
//        lcdPrintf(0,16*2, TFT_WHITE, "%d ms" , lcdGetFpsTime());
//        lcdPrintf(0,16*3, TFT_WHITE, "%d ms" , millis());

        lcdDrawFillRect( 0, 70, 10, 10, TFT_RED);
        lcdDrawFillRect(10, 70, 10, 10, TFT_GREEN);
        lcdDrawFillRect(20, 70, 10, 10, TFT_BLUE);

//        lcdRequestDraw();
      }
    }

    lcdClearBuffer(TFT_BLACK);
    //lcdUpdateDraw();

    ret = true;
  }

  if (args->argc == 1 && args->isStr(0, "image") == true)
  {
	  lcdClearBuffer(TFT_BLACK);
	  //lcdUpdateDraw();
	  while(cliKeepLoop())
	  {
		  static bool blink = 0;
		  blink = get_blink();
		  draw_fan_status(0, 0, blink);
	  }
	  lcdClearBuffer(TFT_BLACK);
	  //lcdUpdateDraw();

	  ret = true;
  }

  if (ret != true)
  {
    cliPrintf("lcd test\n");
    cliPrintf("lcd image\n");
  }
}
#endif

#endif


