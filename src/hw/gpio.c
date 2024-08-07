/*
 * gpio.c
 *
 *  Created on: 2021. 8. 14.
 *      Author: WANG
 */


#include "gpio.h"
#include "cli.h"


typedef struct
{
  GPIO_TypeDef *port;
  uint32_t      pin;
  uint8_t       mode;
  GPIO_PinState on_state;
  GPIO_PinState off_state;
  bool          init_value;
} gpio_tbl_t;


const gpio_tbl_t gpio_tbl[GPIO_MAX_CH] =
{
  {GPIOE, GPIO_PIN_4,  _DEF_OUTPUT,   GPIO_PIN_SET,   GPIO_PIN_RESET, _DEF_HIGH},    //  0. StepA_EN
	{GPIOE, GPIO_PIN_3,  _DEF_OUTPUT,   GPIO_PIN_SET,   GPIO_PIN_RESET, _DEF_LOW },    //  1. StepA_STEP
	{GPIOE, GPIO_PIN_2,  _DEF_OUTPUT,   GPIO_PIN_SET,   GPIO_PIN_RESET, _DEF_LOW },    //  2. StepA_DIR
  {GPIOE, GPIO_PIN_1,  _DEF_OUTPUT, 	GPIO_PIN_SET,   GPIO_PIN_RESET, _DEF_HIGH},    //  3. StepB_EN
	{GPIOE, GPIO_PIN_0,  _DEF_OUTPUT,   GPIO_PIN_SET,   GPIO_PIN_RESET, _DEF_LOW },    //  4. StepB_STEP
	{GPIOB, GPIO_PIN_9,  _DEF_OUTPUT,   GPIO_PIN_SET,   GPIO_PIN_RESET, _DEF_LOW },    //  5. StepB_DIR
  {GPIOB, GPIO_PIN_8,  _DEF_OUTPUT,   GPIO_PIN_SET,   GPIO_PIN_RESET, _DEF_HIGH},    //  6. StepC_EN
	{GPIOB, GPIO_PIN_5,  _DEF_OUTPUT,   GPIO_PIN_SET,   GPIO_PIN_RESET, _DEF_LOW },    //  7. StepC_STEP
	{GPIOB, GPIO_PIN_4,  _DEF_OUTPUT,   GPIO_PIN_SET,   GPIO_PIN_RESET, _DEF_LOW },    //  8. StepC_DIR
  {GPIOB, GPIO_PIN_3,  _DEF_OUTPUT,   GPIO_PIN_SET,   GPIO_PIN_RESET, _DEF_HIGH},    //  9. StepE0_EN
	{GPIOD, GPIO_PIN_6,  _DEF_OUTPUT,   GPIO_PIN_SET,   GPIO_PIN_RESET, _DEF_LOW },    // 10. StepE0_STEP
	{GPIOD, GPIO_PIN_3,  _DEF_OUTPUT,   GPIO_PIN_SET,   GPIO_PIN_RESET, _DEF_LOW },    // 11. StepE0_DIR
	{GPIOA, GPIO_PIN_3,  _DEF_OUTPUT,   GPIO_PIN_SET,   GPIO_PIN_RESET, _DEF_LOW},     // 12. ADC_Touch_xm
	{GPIOA, GPIO_PIN_1,  _DEF_OUTPUT,   GPIO_PIN_SET,   GPIO_PIN_RESET, _DEF_LOW },    // 13. ADC_Touch_yp
	{GPIOD, GPIO_PIN_11, _DEF_OUTPUT,   GPIO_PIN_SET,   GPIO_PIN_RESET, _DEF_HIGH},    // 14. TFT_CS
	{GPIOD, GPIO_PIN_10, _DEF_OUTPUT,   GPIO_PIN_SET,   GPIO_PIN_RESET, _DEF_HIGH},    // 15. TFT_DC
	{GPIOC, GPIO_PIN_6,  _DEF_OUTPUT,   GPIO_PIN_SET,   GPIO_PIN_RESET, _DEF_HIGH},    // 16. TFT_RST
	{GPIOD, GPIO_PIN_13, _DEF_OUTPUT,   GPIO_PIN_SET,   GPIO_PIN_RESET, _DEF_HIGH},    // 17. TFT_BKL
	{GPIOE, GPIO_PIN_14, _DEF_OUTPUT,   GPIO_PIN_SET,   GPIO_PIN_RESET, _DEF_HIGH},    // 18. Toutch_CS
	{GPIOC, GPIO_PIN_5,  _DEF_OUTPUT,   GPIO_PIN_SET,   GPIO_PIN_RESET, _DEF_LOW },    // 19. BEEPER
	{GPIOB, GPIO_PIN_12, _DEF_OUTPUT,  GPIO_PIN_SET,   GPIO_PIN_RESET, _DEF_LOW },     // 20. FLASH_CS
  {GPIOC, GPIO_PIN_0,  _DEF_OUTPUT,   GPIO_PIN_SET,   GPIO_PIN_RESET, _DEF_LOW},     // 21. Touch_xp
  {GPIOC, GPIO_PIN_1,  _DEF_OUTPUT,   GPIO_PIN_SET,   GPIO_PIN_RESET, _DEF_LOW },    // 22. Touch_ym
};


#ifdef _USE_HW_CLI
static void cliGpio(cli_args_t *args);
#endif

bool gpioInit(void)
{
  bool ret = true;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);


  for (int i=0; i<GPIO_MAX_CH; i++)
  {
    gpioPinMode(i, gpio_tbl[i].mode);
    gpioPinWrite(i, gpio_tbl[i].init_value);
  }

  /* EXTI interrupt init*/
  //HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  //HAL_NVIC_EnableIRQ(EXTI4_IRQn);

#ifdef _USE_HW_CLI
  cliAdd("gpio", cliGpio);
#endif
  logPrintf("[%s] gpio_Init()\r\n", ret ? "OK":"NG");

  return ret;
}

bool gpioPinMode(uint8_t ch, uint8_t mode)
{
  bool ret = true;
  GPIO_InitTypeDef GPIO_InitStruct = {0};


  if (ch >= GPIO_MAX_CH)
  {
    return false;
  }

  switch(mode)
  {
    case _DEF_INPUT:
      GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      break;

    case _DEF_INPUT_PULLUP:
      GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
      GPIO_InitStruct.Pull = GPIO_PULLUP;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      break;

    case _DEF_INPUT_PULLDOWN:
      GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
      GPIO_InitStruct.Pull = GPIO_PULLDOWN;
      break;

    case _DEF_INPUT_IT_RISING:
      GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      break;

    case _DEF_OUTPUT:
      GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      break;

    case _DEF_OUTPUT_PULLUP:
      GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Pull = GPIO_PULLUP;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      break;

    case _DEF_OUTPUT_PULLDOWN:
      GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Pull = GPIO_PULLDOWN;
      break;

    case _DEF_INPUT_AF_PP:
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
      break;

    case _DEF_INPUT_ANALOG:
      GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      break;
  }

  GPIO_InitStruct.Pin = gpio_tbl[ch].pin;
  HAL_GPIO_Init(gpio_tbl[ch].port, &GPIO_InitStruct);

  return ret;
}

void gpioPinWrite(uint8_t ch, bool value)
{
  if (ch >= GPIO_MAX_CH)
  {
    return;
  }

  if (value)
  {
    HAL_GPIO_WritePin(gpio_tbl[ch].port, gpio_tbl[ch].pin, gpio_tbl[ch].on_state);
  }
  else
  {
    HAL_GPIO_WritePin(gpio_tbl[ch].port, gpio_tbl[ch].pin, gpio_tbl[ch].off_state);
  }
}

bool gpioPinRead(uint8_t ch)
{
  bool ret = false;

  if (ch >= GPIO_MAX_CH)
  {
    return false;
  }

  if (HAL_GPIO_ReadPin(gpio_tbl[ch].port, gpio_tbl[ch].pin) == gpio_tbl[ch].on_state)
  {
    ret = true;
  }

  return ret;
}

void gpioPinToggle(uint8_t ch)
{
  if (ch >= GPIO_MAX_CH)
  {
    return;
  }

  HAL_GPIO_TogglePin(gpio_tbl[ch].port, gpio_tbl[ch].pin);
}





#ifdef _USE_HW_CLI
void cliGpio(cli_args_t *args)
{
  bool ret = false;


  if (args->argc == 1 && args->isStr(0, "show") == true)
  {
    while(cliKeepLoop())
    {
      for (int i=0; i<GPIO_MAX_CH; i++)
      {
        cliPrintf("%d", gpioPinRead(i));
      }
      cliPrintf("\r\n");
      delay(100);
    }
    ret = true;
  }

  if (args->argc == 2 && args->isStr(0, "read") == true)
  {
    uint8_t ch;

    ch = (uint8_t)args->getData(1);

    while(cliKeepLoop())
    {
      cliPrintf("gpio read %d : %d\r\n", ch, gpioPinRead(ch));
      delay(100);
    }

    ret = true;
  }

  if (args->argc == 3 && args->isStr(0, "write") == true)
  {
    uint8_t ch;
    uint8_t data;

    ch   = (uint8_t)args->getData(1);
    data = (uint8_t)args->getData(2);

    gpioPinWrite(ch, data);

    cliPrintf("gpio write %d : %d\r\n", ch, data);
    ret = true;
  }

  if (ret != true)
  {
    cliPrintf("gpio show\r\n");
    cliPrintf("gpio read ch[0~%d]\r\n", GPIO_MAX_CH-1);
    cliPrintf("gpio write ch[0~%d] 0:1\r\n", GPIO_MAX_CH-1);
  }
}
#endif
