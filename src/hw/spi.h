/*
 * i2c.h
 *
 *  Created on: 2020. 12. 20.
 *      Author: WANG
 */

#ifndef SRC_COMMON_HW_INCLUDE_SPI_H_
#define SRC_COMMON_HW_INCLUDE_SPI_H_

#include "hw.h"


#ifdef _USE_HW_SPI

#define SPI_MAX_CH          HW_SPI_MAX_CH


#define SPI_MODE0           0
#define SPI_MODE1           1
#define SPI_MODE2           2
#define SPI_MODE3           3

#define SPI_SPEED_CLOCK_DIV2_MHZ    ((uint32_t)2)
#define SPI_SPEED_CLOCK_DIV4_MHZ    ((uint32_t)4)
#define SPI_SPEED_CLOCK_DIV8_MHZ    ((uint32_t)8)
#define SPI_SPEED_CLOCK_DIV16_MHZ   ((uint32_t)16)
#define SPI_SPEED_CLOCK_DIV32_MHZ   ((uint32_t)32)
#define SPI_SPEED_CLOCK_DIV64_MHZ   ((uint32_t)64)
#define SPI_SPEED_CLOCK_DIV128_MHZ  ((uint32_t)128)
#define SPI_SPEED_CLOCK_DIV256_MHZ  ((uint32_t)256)

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern DMA_HandleTypeDef hdma_spi2_rx;

typedef enum {
    BUS_READY,
    BUS_BUSY,
    BUS_ABORT
} busStatus_e;


bool spiInit(void);
bool spiBegin(uint8_t dev);
bool spiIsBegin(uint8_t dev);
void spiSetDataMode(uint8_t dev, uint8_t dataMode);
void spiSetBitWidth(uint8_t dev, uint8_t bit_width);
uint32_t SPI_Get_Speed(uint8_t dev);
bool SPI_Set_Speed_scal(uint8_t dev, uint32_t prescaler);
bool SPI_Set_Speed_hz(uint8_t dev, uint32_t speed);
void spiSetClkDivisor(uint8_t dev, uint32_t prescaler);

bool     spiTransfer(uint8_t dev, uint8_t *tx_buf, uint8_t *rx_buf, uint32_t length, uint32_t timeout);
uint8_t  spiTransfer8(uint8_t ch, uint8_t data);
uint16_t spiTransfer16(uint8_t ch, uint16_t data);

void spiDmaTxStart(uint8_t ch, uint8_t *p_buf, uint32_t length);
bool spiDmaTxTransfer(uint8_t ch, void *buf, uint32_t length, uint32_t timeout);

bool spiDmaTxIsDone(uint8_t ch);
void spiAttachTxInterrupt(uint8_t ch, void (*func)());

void spiReadRegBuf(uint8_t ch, uint8_t reg, uint8_t *data, uint8_t length);
void spiDmaTxRxStart(uint8_t ch, uint8_t *p_txbuf, uint8_t *p_rxbuf, uint16_t length);

#endif

#endif /* SRC_COMMON_HW_INCLUDE_I2C_H_ */
