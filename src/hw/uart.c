/*
 * uart.c
 *
 *  Created on: 2020. 12. 8.
 *      Author: baram
 */


#include "uart.h"
#include "ring_buffer.h"
#include "usbd_cdc_if.h"

static bool is_open[UART_MAX_CH];
#define MAX_SIZE 128

static qbuffer_t ring_buffer[UART_MAX_CH];
static volatile uint8_t rx_buf[UART_MAX_CH-1][MAX_SIZE];
static volatile uint8_t rx_buf2[MAX_SIZE];
static volatile uint8_t rx_buf3[MAX_SIZE];

UART_HandleTypeDef huart3;

DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart3_rx;

const uint32_t baudRates[] = {0, 9600, 19200, 38400, 57600, 115200, 230400, 250000,
        400000, 460800, 500000, 921600, 1000000, 1500000, 2000000, 2470000}; // see baudRate_e

#define BAUD_RATE_COUNT (sizeof(baudRates) / sizeof(baudRates[0]))

bool uartInit(void)
{
  for (int i=0; i<UART_MAX_CH; i++)
  {
    is_open[i] = false;
  }

  return true;
}

bool uartOpen(uint8_t ch, uint32_t baud)
{
  bool ret = false;

  switch(ch)
  {
    case _DEF_USB:
      is_open[ch] = true;
      ret = true;
      break;

    case _DEF_UART3:
    	huart3.Instance = USART3;
    	huart3.Init.BaudRate = baud;
    	huart3.Init.WordLength = UART_WORDLENGTH_8B;
      huart3.Init.StopBits = UART_STOPBITS_1;
    	huart3.Init.Parity = UART_PARITY_NONE;
    	huart3.Init.Mode = UART_MODE_TX_RX;
    	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    	huart3.Init.OverSampling = UART_OVERSAMPLING_16;

    	qbufferCreate(&ring_buffer[ch], (uint8_t *)&rx_buf3, MAX_SIZE);

      if (HAL_UART_Init(&huart3) != HAL_OK)
      {
        Error_Handler();
      }
      else
      {
        ret = true;
        is_open[ch] = true;
        if(HAL_UART_Receive_IT(&huart3, (uint8_t *)&rx_buf3, 1) != HAL_OK)
        {
          ret = false;
        }
      }
      break;
  }

  return ret;
}

uint32_t uartAvailable(uint8_t ch)
{
  uint32_t ret = 0;

  switch(ch)
  {

    case _DEF_USB:
      ret = cdcAvailable();
      break;

    case _DEF_UART3:
    	ring_buffer[ch].in = (ring_buffer[ch].len - hdma_usart3_rx.Instance->NDTR);
      ret = qbufferAvailable(&ring_buffer[ch]);
      break;
  }
  return ret;
}

uint8_t uartRead(uint8_t ch)
{
  uint8_t ret = 0;

  switch(ch)
  {
    case _DEF_USB:
      ret = cdcRead();
      break;

    case _DEF_UART3:
    	qbufferRead(&ring_buffer[ch], &ret, 1);
      break;
  }

  return ret;
}

uint32_t uartWrite(uint8_t ch, uint8_t *p_data, uint32_t length)
{
  uint32_t ret = 0;
  HAL_StatusTypeDef status = HAL_ERROR;

  switch(ch)
  {

    case _DEF_USB:
      ret = cdcWrite(p_data, length);
      break; 

    case _DEF_UART3:
      status = HAL_UART_Transmit(&huart3, p_data, length, 100);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;
  }

  return ret;
}

uint32_t uartWriteIT(uint8_t ch, uint8_t *p_data, uint32_t length)
{
  uint32_t ret = 0;
  HAL_StatusTypeDef status = HAL_ERROR;

  switch(ch)
  {
    case _DEF_USB:
      //status = HAL_UART_Transmit_IT(&huart2, p_data, length);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;

    case _DEF_UART3:
      status = HAL_UART_Transmit_IT(&huart3, p_data, length);
      if (status == HAL_OK)
      {
        ret = length;
      }
      break;
  }

  return ret;
}

void serialPrint(uint8_t channel, const char *str)
{
    uint8_t ch;
    while ((ch = *(str++)) != 0) {
      uartWrite(channel, &ch, 1);
    }
}

uint32_t uartVPrintf(uint8_t ch, const char *fmt, va_list arg)
{
  uint32_t ret = 0;
  char print_buf[256];


  int len;
  len = vsnprintf(print_buf, 256, fmt, arg);

  if (len > 0)
  {
    ret = uartWrite(ch, (uint8_t *)print_buf, len);
  }

  return ret;
}

uint32_t uartPrintf(uint8_t ch, char *fmt, ...)
{
  char buf[MAX_SIZE];
  va_list args;
  int len;
  uint32_t ret;

  va_start(args, fmt);
  len = vsnprintf(buf, MAX_SIZE, fmt, args);

  ret = uartWrite(ch, (uint8_t *)buf, len);

  va_end(args);


  return ret;
}

uint32_t uartPrintf_IT(uint8_t ch, char *fmt, ...)
{
  char buf[MAX_SIZE];
  va_list args;
  int len;
  uint32_t ret;

  va_start(args, fmt);
  len = vsnprintf(buf, MAX_SIZE, fmt, args);

  ret = uartWriteIT(ch, (uint8_t *)buf, len);

  va_end(args);


  return ret;
}

uint32_t uartGetBaud(uint8_t ch)
{
  uint32_t ret = 0;


  switch(ch)
  {
    case _DEF_USB:
      ret = cdcGetBaud();
      break;

    case _DEF_UART3:
      ret = huart3.Init.BaudRate;
      break;
  }

  return ret;
}

bool uartSetBaud(uint8_t ch, uint32_t baud)
{
	bool ret = false;

	switch(ch)
	{
    case _DEF_USB:
			//huart2.Init.BaudRate = baud;
    	//if (HAL_UART_Init(&huart2) != HAL_OK)
//    	{
//    	  Error_Handler();
//    	}else
//    	{
//    		ret = true;
//    	}
//			break;

    case _DEF_UART3:
			huart3.Init.BaudRate = baud;
    	if (HAL_UART_Init(&huart3) != HAL_OK)
    	{
    	  Error_Handler();
    	}else
    	{
    		ret = true;
    	}
			break;
	}

	return ret;
}

baudRate_e lookupBaudRateIndex(uint32_t baudRate)
{
    uint8_t index;

    for (index = 0; index < BAUD_RATE_COUNT; index++) {
        if (baudRates[index] == baudRate) {
            return index;
        }
    }
    return BAUD_AUTO;
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
		//HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)&rx_buf2[0], MAX_SIZE);
		//__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
  }
}

uint8_t uart3_rx_data = 0;
uint8_t telemetry_rx_buf[20];
uint8_t telemetry_rx_cplt_flag;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  static unsigned char cnt = 0;

  if(huart->Instance == USART3)
  {
    HAL_UART_Receive_IT(&huart3, (uint8_t *)&rx_buf3, 1);
    qbufferWrite(&ring_buffer[_DEF_UART3], (uint8_t *)&rx_buf3, 1);
    qbufferRead(&ring_buffer[_DEF_UART3], (uint8_t *)&uart3_rx_data, 1);

    switch(cnt)
    {
    case 0:
      if(uart3_rx_data == 0x47)
      {
        telemetry_rx_buf[cnt] = uart3_rx_data;
        cnt++;
      }
      break;
    case 1:
      if(uart3_rx_data == 0x53)
      {
        telemetry_rx_buf[cnt] = uart3_rx_data;
        cnt++;
      }
      else
        cnt = 0;
      break;
    case 19:
      telemetry_rx_buf[cnt] = uart3_rx_data;
      cnt = 0;
      telemetry_rx_cplt_flag = 1;
      break;
    default:
      telemetry_rx_buf[cnt] = uart3_rx_data;
      cnt++;
      break;
    }

  }
}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
	/* USART3 clock enable */
	__HAL_RCC_USART3_CLK_ENABLE();

	__HAL_RCC_GPIOB_CLK_ENABLE();
	/**USART3 GPIO Configuration
	PB10     ------> USART3_TX
	PB11     ------> USART3_RX
	*/
	GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USART3 DMA Init */
	/* USART3_TX Init */
	hdma_usart3_tx.Instance = DMA1_Stream3;
	hdma_usart3_tx.Init.Channel = DMA_CHANNEL_4;
	hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart3_tx.Init.Mode = DMA_NORMAL;
	hdma_usart3_tx.Init.Priority = DMA_PRIORITY_LOW;
	hdma_usart3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK)
	{
	  Error_Handler();
	}

	__HAL_LINKDMA(uartHandle,hdmatx,hdma_usart3_tx);

	/* USART3_RX Init */
	hdma_usart3_rx.Instance = DMA1_Stream1;
	hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;
	hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;
	hdma_usart3_rx.Init.Priority = DMA_PRIORITY_LOW;
	hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
	{
	  Error_Handler();
	}

	__HAL_LINKDMA(uartHandle,hdmarx,hdma_usart3_rx);

	/* USART3 interrupt Init */
	HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
	/* Peripheral clock disable */
	__HAL_RCC_USART3_CLK_DISABLE();

	/**USART3 GPIO Configuration
	PB10     ------> USART3_TX
	PB11     ------> USART3_RX
	*/
	HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

	/* USART3 DMA DeInit */
	HAL_DMA_DeInit(uartHandle->hdmatx);
	HAL_DMA_DeInit(uartHandle->hdmarx);

	/* USART3 interrupt Deinit */
	HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}
