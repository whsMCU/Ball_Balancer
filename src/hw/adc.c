/*
 * adc.c
 *
 *  Created on: Jul 20, 2024
 *      Author: WANG
 */

#include "adc.h"

static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to)
{
  if (from != to) {
    if (from > to) {
      value = (value < (uint32_t)(1 << (from - to))) ? 0 : ((value + 1) >> (from - to)) - 1;
    } else {
      if (value != 0) {
        value = ((value + 1) << (to - from)) - 1;
      }
    }
  }
  return value;
}

typedef struct
{
  bool is_open;
  bool is_adc_done;

  ADC_HandleTypeDef *h_adc;
  ADC_ChannelConfTypeDef *h_sConfig;
} adc_t;

adc_t adc_tbl[ADC_MAX_CH];

ADC_HandleTypeDef hadc1;
ADC_ChannelConfTypeDef sConfig = {0};

static void cliADC(cli_args_t *args);

bool adcInit(void)
{
  bool ret = true;

  for (int i=0; i<ADC_MAX_CH; i++)
  {
    adc_tbl[i].is_open = false;
    adc_tbl[i].is_adc_done = false;
    adc_tbl[i].h_adc = NULL;
    adc_tbl[i].h_sConfig = NULL;
  }

  cliAdd("adc", cliADC);
  logPrintf("[%s] adc_Init()\r\n", ret ? "OK":"NG");
  return ret;

}

bool adcOpen(uint8_t ch)
{
  bool ret = false;
  adc_t *p_adc = &adc_tbl[ch];

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  switch(ch)
  {
    case _DEF_ADC1:
      p_adc->h_adc = &hadc1;
      p_adc->h_sConfig = &sConfig;
      p_adc->h_adc->Instance = ADC1;
      p_adc->h_adc->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
      p_adc->h_adc->Init.Resolution = ADC_RESOLUTION_10B;
      p_adc->h_adc->Init.ScanConvMode = DISABLE;
      p_adc->h_adc->Init.ContinuousConvMode = DISABLE;
      p_adc->h_adc->Init.DiscontinuousConvMode = DISABLE;
      p_adc->h_adc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
      p_adc->h_adc->Init.ExternalTrigConv = ADC_SOFTWARE_START;
      p_adc->h_adc->Init.DataAlign = ADC_DATAALIGN_RIGHT;
      p_adc->h_adc->Init.NbrOfConversion = 1;
      p_adc->h_adc->Init.DMAContinuousRequests = DISABLE;
      p_adc->h_adc->Init.EOCSelection = ADC_EOC_SINGLE_CONV;

      /* ADC1 clock enable */
      __HAL_RCC_ADC1_CLK_ENABLE();
      __HAL_RCC_GPIOA_CLK_ENABLE();

      /**ADC1 GPIO Configuration
      PA5     ------> ADC1_IN5
      */
      GPIO_InitStruct.Pin = GPIO_PIN_5;
      GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

      /* ADC1 interrupt Init */
      HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
      HAL_NVIC_EnableIRQ(ADC_IRQn);

      if (HAL_ADC_Init(p_adc->h_adc) == HAL_OK)
      {
        p_adc->is_open = true;
        ret = true;
      }
      p_adc->h_sConfig->Channel = ADC_CHANNEL_5;
      if(!IS_ADC_CHANNEL(p_adc->h_sConfig->Channel))
      {
        return 0;
      }
      p_adc->h_sConfig->Rank = 1;
      p_adc->h_sConfig->SamplingTime = ADC_SAMPLETIME_15CYCLES;
      if (HAL_ADC_ConfigChannel(p_adc->h_adc, p_adc->h_sConfig) == HAL_OK)
      {
        p_adc->is_open = true;
        ret = true;
      }
      //logPrintf("[%s] adc1_Init()\r\n", ret ? "OK":"NG");
      break;

    case _DEF_ADC2:
      p_adc->h_adc = &hadc1;
      p_adc->h_sConfig = &sConfig;
      p_adc->h_adc->Instance = ADC1;
      p_adc->h_adc->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
      p_adc->h_adc->Init.Resolution = ADC_RESOLUTION_10B;
      p_adc->h_adc->Init.ScanConvMode = DISABLE;
      p_adc->h_adc->Init.ContinuousConvMode = DISABLE;
      p_adc->h_adc->Init.DiscontinuousConvMode = DISABLE;
      p_adc->h_adc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
      p_adc->h_adc->Init.ExternalTrigConv = ADC_SOFTWARE_START;
      p_adc->h_adc->Init.DataAlign = ADC_DATAALIGN_RIGHT;
      p_adc->h_adc->Init.NbrOfConversion = 1;
      p_adc->h_adc->Init.DMAContinuousRequests = DISABLE;
      p_adc->h_adc->Init.EOCSelection = ADC_EOC_SINGLE_CONV;

      /* ADC1 clock enable */
      __HAL_RCC_ADC1_CLK_ENABLE();
      __HAL_RCC_GPIOA_CLK_ENABLE();

      /**ADC1 GPIO Configuration
      PA6     ------> ADC1_IN6
      */
      GPIO_InitStruct.Pin = GPIO_PIN_6;
      GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

      /* ADC1 interrupt Init */
      HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
      HAL_NVIC_EnableIRQ(ADC_IRQn);

      if (HAL_ADC_Init(p_adc->h_adc) == HAL_OK)
      {
        p_adc->is_open = true;
        ret = true;
      }
      p_adc->h_sConfig->Channel = ADC_CHANNEL_6;
      p_adc->h_sConfig->Rank = 1;
      p_adc->h_sConfig->SamplingTime = ADC_SAMPLETIME_15CYCLES;
      if (HAL_ADC_ConfigChannel(p_adc->h_adc, p_adc->h_sConfig) == HAL_OK)
      {
        p_adc->is_open = true;
        ret = true;
      }
      //logPrintf("[%s] adc2_Init()\r\n", ret ? "OK":"NG");
      break;
  }
  return ret;
}

bool adcClose(uint8_t ch)
{
  bool ret = false;

  switch(ch)
  {
    case _DEF_ADC1:
      /* Peripheral clock disable */
      __HAL_RCC_ADC1_CLK_DISABLE();

      /**ADC1 GPIO Configuration
      PA1     ------> ADC1_IN1
      */
      HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1);

      /* ADC1 interrupt Deinit */
      HAL_NVIC_DisableIRQ(ADC_IRQn);
      ret = true;
      break;

    case _DEF_ADC2:
      /* Peripheral clock disable */
      __HAL_RCC_ADC1_CLK_DISABLE();

      /**ADC1 GPIO Configuration
      PA3     ------> ADC1_IN3
      */
      HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3);

      /* ADC1 interrupt Deinit */
      HAL_NVIC_DisableIRQ(ADC_IRQn);
      ret = true;
      break;
  }

  return ret;
}

uint32_t analogRead(uint8_t ch)
{
  uint16_t uhADCxConvertedValue = 0;
  uint8_t adc_ch = 0;

  switch(ch)
  {
    case ADC_Touch_ym:
      adc_ch = 0;
      break;

    case ADC_Touch_xp:
      adc_ch = 1;
      break;
  }

  adcOpen(adc_ch);

  adc_t *p_adc = &adc_tbl[adc_ch];

  switch(adc_ch)
  {
    case _DEF_ADC1:
      /*##-3- Start the conversion process ####################*/
      if (HAL_ADC_Start(p_adc->h_adc) != HAL_OK) {
        /* Start Conversion Error */
        return 0;
      }

      /*##-4- Wait for the end of conversion #####################################*/
      /*  For simplicity reasons, this example is just waiting till the end of the
          conversion, but application may perform other tasks while conversion
          operation is ongoing. */
      if (HAL_ADC_PollForConversion(p_adc->h_adc, 10) != HAL_OK) {
        /* End Of Conversion flag not set on time */
        return 0;
      }

      /* Check if the continuous conversion of regular channel is finished */
      if ((HAL_ADC_GetState(p_adc->h_adc) & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC) {
        /*##-5- Get the converted value of regular channel  ########################*/
        uhADCxConvertedValue = HAL_ADC_GetValue(p_adc->h_adc);
      }

      if (HAL_ADC_Stop(p_adc->h_adc) != HAL_OK) {
        /* Stop Conversation Error */
        return 0;
      }
      break;

    case _DEF_ADC2:
      /*##-3- Start the conversion process ####################*/
      if (HAL_ADC_Start(p_adc->h_adc) != HAL_OK) {
        /* Start Conversion Error */
        return 0;
      }

      /*##-4- Wait for the end of conversion #####################################*/
      /*  For simplicity reasons, this example is just waiting till the end of the
          conversion, but application may perform other tasks while conversion
          operation is ongoing. */
      if (HAL_ADC_PollForConversion(p_adc->h_adc, 10) != HAL_OK) {
        /* End Of Conversion flag not set on time */
        return 0;
      }

      /* Check if the continuous conversion of regular channel is finished */
      if ((HAL_ADC_GetState(p_adc->h_adc) & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC) {
        /*##-5- Get the converted value of regular channel  ########################*/
        uhADCxConvertedValue = HAL_ADC_GetValue(p_adc->h_adc);
      }

      if (HAL_ADC_Stop(p_adc->h_adc) != HAL_OK) {
        /* Stop Conversation Error */
        return 0;
      }
      break;
  }
  adcClose(adc_ch);
  return uhADCxConvertedValue;
}

void cliADC(cli_args_t *args)
{
  bool ret = true;

  if (ret == false)
  {
    cliPrintf( "adc scan\r\n");
    cliPrintf( "adc read dev_addr reg_addr length\r\n");
    cliPrintf( "adc write dev_addr reg_addr data\r\n");
  }
}
