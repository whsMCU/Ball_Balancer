/*
 * st7796.c
 *
 *  Created on: Jan 29, 2024
 *      Author: 왕학승
 */


#include "st7796.h"

static uint8_t   spi_ch = _DEF_SPI1;

lcd_driver_t *p_lcd_driver;
static void (*frameCallBack)(void) = NULL;
volatile static bool  is_write_frame = false;

const uint32_t colstart = 0;
const uint32_t rowstart = 0;

static void st7796InitRegs(void);
static void st7796FillRect(int32_t x, int32_t y, int32_t w, int32_t h, uint32_t color);
static void st7796SetRotation(uint8_t m);
static bool st7796Reset(void);


static void TransferDoneISR(void)
{
//  if (is_write_frame == true)
//  {
    is_write_frame = false;
    TFT_CS_H;

    if (frameCallBack != NULL)
    {
      frameCallBack();
    }
//  }
}


bool st7796Init(void)
{
  bool ret;

  ret = st7796Reset();

  return ret;
}

bool st7796InitDriver(lcd_driver_t *p_driver)
{
  p_lcd_driver = p_driver;
  p_driver->init = st7796Init;
  p_driver->reset = st7796Reset;
  p_driver->setWindow = st7796SetWindow;
  p_driver->getWidth = st7796GetWidth;
  p_driver->getHeight = st7796GetHeight;
  p_driver->setCallBack = st7796SetCallBack;
  p_driver->sendBuffer = st7796SendBuffer;
  p_driver->fillRect = st7796FillRect;

  p_driver->_width = HW_LCD_WIDTH;
  p_driver->_height = HW_LCD_HEIGHT;
  return true;
}

bool st7796Reset(void)
{
  spiBegin(spi_ch);
  spiSetDataMode(spi_ch, SPI_MODE0);

  spiAttachTxInterrupt(spi_ch, TransferDoneISR);

  TFT_BKL_L;
  TFT_DC_D;
  TFT_CS_H;
  delay(10);

  st7796InitRegs();

  st7796SetRotation(1);
  st7796FillRect(0, 0, HW_LCD_WIDTH, HW_LCD_HEIGHT, TFT_BLACK);

  return true;
}

uint16_t st7796GetWidth(void)
{
  return HW_LCD_WIDTH;
}

uint16_t st7796GetHeight(void)
{
  return HW_LCD_HEIGHT;
}

void writecommand(uint8_t c)
{
	TFT_DC_C;
	TFT_CS_L;

	spiTransfer8(_DEF_SPI1, c);

	TFT_CS_H;
}

void writedata(uint8_t d)
{
	TFT_DC_D;        // Play safe, but should already be in data mode
	TFT_CS_L;

	spiTransfer8(_DEF_SPI1, d);

	TFT_CS_H;
}

void st7796InitRegs(void)
{
	delay(120);
	writecommand(ST7796_SWRESET); //Software reset
	delay(120);
	writecommand(ST7796_SLPOUT); // Sleep out

	delay(20);

	writecommand(ST7796_CSCON); //Command Set Control
	writedata(0xC3);            //enable command 2 part I

	writecommand(ST7796_CSCON);	//Command Set Control
	writedata(0x96);           	//enable command 2 part II

	writecommand(ST7796_MADCTL);//Memory Data Access Control
	writedata(TFT_MAD_MX | TFT_MAD_COLOR_ORDER);

	writecommand(ST7796_PIXFMT);//Interface Pixel Format
	writedata(0x55);			//16bit/pixel

	writecommand(ST7796_INVCTR);//Display Inversion Cotnrol
	writedata(0x01);			//1-dot inversion

	writecommand(ST7796_DFUNCTR); //Display Function Control
	writedata(0x80);    //Bypass
	writedata(0x02);    //Source Output Scan from S1 to S960, Gate Output scan from G1 to G480, scan cycle=2
	writedata(0x3B);    //LCD Drive Line=8*(59+1)

	writecommand(ST7796_EM);	//Entry Mode Set
	writedata(0xC6);

	writecommand(ST7796_PWCTR2);
	writedata(0x15);

	writecommand(ST7796_PWCTR3);
	writedata(0xAF);

	writecommand(ST7796_VMCTR1);
	writedata(0x22);

	writecommand(ST7796_VMCOFF);
	writedata(0x00);

	writecommand(ST7796_DOCA);  //Display Output Ctrl Adjust
	writedata(0x40);
	writedata(0x8A);
	writedata(0x00);
	writedata(0x00);
	writedata(0x29);
	writedata(0x19);
	writedata(0xA5);
	writedata(0x33);

	writecommand(ST7796_GMCTRP1); //Positiove Gamma Cotnrol
	writedata(0xF0);
	writedata(0x04);
	writedata(0x08);
	writedata(0x09);
	writedata(0x08);
	writedata(0x15);
	writedata(0x2F);
	writedata(0x42);
	writedata(0x46);
	writedata(0x28);
	writedata(0x15);
	writedata(0x16);
	writedata(0x29);
	writedata(0x2D);

	writecommand(ST7796_GMCTRN1); //Negative Gamma Cotnrol
	writedata(0xF0);
	writedata(0x04);
	writedata(0x09);
	writedata(0x09);
	writedata(0x08);
	writedata(0x15);
	writedata(0x2E);
	writedata(0x46);
	writedata(0x46);
	writedata(0x28);
	writedata(0x15);
	writedata(0x16);
	writedata(0x29);
	writedata(0x2D);

	writecommand(ST7796_NORON);

	writecommand(ST7796_WRCTRLD);
	writedata(0x24);

	writecommand(ST7796_CSCON);
	writedata(0xC3);

	writecommand(ST7796_CSCON);
	writedata(0x69);

	writecommand(ST7796_DISPON);
}

void st7796SetRotation(uint8_t mode)
{

	writecommand(TFT_MADCTL);
	switch (mode) {
	  case 0:
		writedata(TFT_MAD_MX | TFT_MAD_COLOR_ORDER);
			p_lcd_driver->_width  = HW_LCD_HEIGHT;
			p_lcd_driver->_height = HW_LCD_WIDTH;
	#ifdef CGRAM_OFFSET
		if (lcd._init_width == 222)
		{
			lcd.colstart = 49;
			lcd.rowstart = 0;
		}
	#endif
		break;
	  case 1:
		writedata(TFT_MAD_MV | TFT_MAD_COLOR_ORDER);
		p_lcd_driver->_width  = HW_LCD_WIDTH;
		p_lcd_driver->_height = HW_LCD_HEIGHT;
	#ifdef CGRAM_OFFSET
		if (lcd._init_width == 222)
		{
			lcd.colstart = 0;
			lcd.rowstart = 49;
		}
	#endif
		break;
	  case 2:
		writedata(TFT_MAD_MY | TFT_MAD_COLOR_ORDER);
		p_lcd_driver->_width  = HW_LCD_HEIGHT;
		p_lcd_driver->_height = HW_LCD_WIDTH;
	#ifdef CGRAM_OFFSET
		if (lcd._init_width == 222)
		{
			lcd.colstart = 49;
			lcd.rowstart = 0;
		}
	#endif
		break;
	  case 3:
		writedata(TFT_MAD_MX | TFT_MAD_MY | TFT_MAD_MV | TFT_MAD_COLOR_ORDER);
		p_lcd_driver->_width  = HW_LCD_WIDTH;
		p_lcd_driver->_height = HW_LCD_HEIGHT;
	#ifdef CGRAM_OFFSET
		if (lcd._init_width == 222)
		{
			lcd.colstart = 0;
			lcd.rowstart = 49;
		}
	#endif
		break;
	// These next rotations are for bottom up BMP drawing
	  case 4:
		writedata(TFT_MAD_MX | TFT_MAD_MY | TFT_MAD_COLOR_ORDER);
		p_lcd_driver->_width  = HW_LCD_HEIGHT;
		p_lcd_driver->_height = HW_LCD_WIDTH;
		break;
	  case 5:
		writedata(TFT_MAD_MV | TFT_MAD_MX | TFT_MAD_COLOR_ORDER);
		p_lcd_driver->_width  = HW_LCD_WIDTH;
		p_lcd_driver->_height = HW_LCD_HEIGHT;
		break;
	  case 6:
		writedata(TFT_MAD_COLOR_ORDER);
		p_lcd_driver->_width  = HW_LCD_HEIGHT;
		p_lcd_driver->_height = HW_LCD_WIDTH;
		break;
	  case 7:
		writedata(TFT_MAD_MY | TFT_MAD_MV | TFT_MAD_COLOR_ORDER);
		p_lcd_driver->_width  = HW_LCD_WIDTH;
		p_lcd_driver->_height = HW_LCD_HEIGHT;
		break;

	}

	delayMicroseconds(10);
}

void st7796SetWindow(int32_t x0, int32_t y0, int32_t x1, int32_t y1)
{
	spiSetBitWidth(spi_ch, 8);

	#ifdef CGRAM_OFFSET
	x0+=colstart;
	x1+=colstart;
	y0+=rowstart;
	y1+=rowstart;
	#endif

	writecommand(TFT_CASET); // Column addr set
	writedata((x0 >> 8) & 0xFF);
	writedata(x0 & 0xFF);     // XSTART
	writedata((x1 >> 8) & 0xFF);
	writedata(x1 & 0xFF);     // XEND

	writecommand(TFT_PASET); // Row addr set
	writedata((y0 >> 8) & 0xFF);
	writedata(y0 & 0xFF);     // YSTART
	writedata((y1 >>8) & 0xFF);
	writedata(y1 & 0xFF);     // YEND

	writecommand(TFT_RAMWR); // write to RAM
}

void st7796FillRect(int32_t x, int32_t y, int32_t w, int32_t h, uint32_t color)
{
  uint16_t line_buf[w];

  // Clipping
  if ((x >= p_lcd_driver->_width) || (y >= p_lcd_driver->_height)) return;

  if (x < 0) { w += x; x = 0; }
  if (y < 0) { h += y; y = 0; }

  if ((x + w) > p_lcd_driver->_width)  w = p_lcd_driver->_width  - x;
  if ((y + h) > p_lcd_driver->_height) h = p_lcd_driver->_height - y;

  if ((w < 1) || (h < 1)) return;


  st7796SetWindow(x, y, x + w - 1, y + h - 1);
  spiSetBitWidth(spi_ch, 16);

  TFT_DC_D;
  TFT_CS_L;

  for (int i=0; i<w; i++)
  {
    line_buf[i] = color;
  }
  for (int i=0; i<h; i++)
  {
    if (spiDmaTxTransfer(_DEF_SPI1, (void *)line_buf, w, 10) != true)
    {
      break;
    }
  }
  spiSetBitWidth(spi_ch, 8);
  TFT_CS_H;
}

bool st7796SendBuffer(uint8_t *p_data, uint32_t length, uint32_t timeout_ms)
{
  is_write_frame = true;

  spiSetBitWidth(spi_ch, 16);

  TFT_DC_D;
  TFT_CS_L;

  spiDmaTxTransfer(_DEF_SPI1, (void *)p_data, length, 0);
  spiSetBitWidth(spi_ch, 8);
  TFT_CS_H;
  return true;
}

bool st7796SetCallBack(void (*p_func)(void))
{
  frameCallBack = p_func;

  return true;
}

