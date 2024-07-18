/*
 * button.c
 *
 *  Created on: 2021. 8. 14.
 *      Author: WANG
 */


#include "button.h"
#include "cli.h"


enum
{
	BUTTON_IDLE,
	BUTTON_Pressed,
	BUTTON_LONG_Pressed,
};

typedef struct
{
  GPIO_TypeDef *port;
  uint32_t      pin;
  uint32_t		  pull;
  GPIO_PinState on_state;
} button_tbl_t;


button_tbl_t button_tbl[BUTTON_MAX_CH] =
{
  {GPIOE, GPIO_PIN_8,  GPIO_PULLUP, GPIO_PIN_RESET}, // BTN_EN1
  {GPIOE, GPIO_PIN_11, GPIO_PULLUP, GPIO_PIN_RESET}, // BTN_EN2
  {GPIOE, GPIO_PIN_13, GPIO_PULLUP, GPIO_PIN_RESET}, // BTN_ENC
};


#ifdef _USE_HW_CLI
static void cliButton(cli_args_t *args);
#endif


bool buttonInit(void)
{
  bool ret = true;
  GPIO_InitTypeDef GPIO_InitStruct = {0};


  __HAL_RCC_GPIOE_CLK_ENABLE();


  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;

  for (int i=0; i<BUTTON_MAX_CH; i++)
  {
    GPIO_InitStruct.Pin = button_tbl[i].pin;
    GPIO_InitStruct.Pull = button_tbl[i].pull;
    HAL_GPIO_Init(button_tbl[i].port, &GPIO_InitStruct);
  }

#ifdef _USE_HW_CLI
  cliAdd("button", cliButton);
#endif
  logPrintf("[%s] button_Init()\r\n", ret ? "OK":"NG");

  return ret;
}

bool buttonGetPressed(uint8_t ch)
{
  bool ret = false;


  if (ch >= BUTTON_MAX_CH)
  {
    return false;
  }

  if (HAL_GPIO_ReadPin(button_tbl[ch].port, button_tbl[ch].pin) == button_tbl[ch].on_state)
  {
    ret = true;
  }

  return ret;
}

#if HW_BUTTON_OBJ_USE == 1
enum ButtonObjState
{
  BUTTON_OBJ_WAIT_FOR_RELEASED,
  BUTTON_OBJ_WAIT_FOR_PRESSED,
  BUTTON_OBJ_PRESSED,
  BUTTON_OBJ_REPEATED_START,
  BUTTON_OBJ_REPEATED,
};

void buttonObjCreate(button_obj_t *p_obj, uint8_t ch, uint32_t pressed_time, uint32_t repeat_start_time, uint32_t repeat_pressed_time)
{
  p_obj->ch = ch;
  p_obj->state = 0;
  p_obj->pre_time = millis();
  p_obj->pressed_time = pressed_time;
  p_obj->repeat_start_time = repeat_start_time;
  p_obj->repeat_pressed_time = repeat_pressed_time;
  p_obj->event_flag = 0;
  p_obj->state_flag = 0;
  p_obj->click_count = 0;
}

bool buttonObjUpdateEx(button_obj_t *p_obj, bool clear_event)
{
  bool ret = false;


  if (clear_event == true)
  {
    buttonObjClearEventAll(p_obj);
  }

  switch(p_obj->state)
  {
    case BUTTON_OBJ_WAIT_FOR_RELEASED:
      if (buttonGetPressed(p_obj->ch) == false)
      {
        p_obj->state = BUTTON_OBJ_WAIT_FOR_PRESSED;
      }
      break;

    case BUTTON_OBJ_WAIT_FOR_PRESSED:
      if (buttonGetPressed(p_obj->ch) == true)
      {
        p_obj->state = BUTTON_OBJ_PRESSED;
        p_obj->pre_time = millis();
        p_obj->click_count = 0;
      }
      break;

    case BUTTON_OBJ_PRESSED:
      if (buttonGetPressed(p_obj->ch) == true)
      {
        if (millis()-p_obj->pre_time >= p_obj->pressed_time)
        {
          ret = true;
          p_obj->state = BUTTON_OBJ_REPEATED_START;
          p_obj->pre_time = millis();
          p_obj->event_flag |= BUTTON_EVT_CLICKED;

          p_obj->state_flag |= BUTTON_STATE_PRESSED;
          p_obj->click_count++;
        }
      }
      else
      {
        p_obj->state = BUTTON_OBJ_WAIT_FOR_PRESSED;

        if (p_obj->state_flag & BUTTON_STATE_PRESSED)
        {
          p_obj->event_flag |= BUTTON_EVT_RELEASED;

          p_obj->state_flag |= BUTTON_STATE_RELEASED;
          p_obj->state_flag &= ~BUTTON_STATE_PRESSED;
          p_obj->state_flag &= ~BUTTON_STATE_REPEATED;
        }
      }
      break;

    case BUTTON_OBJ_REPEATED_START:
      if (buttonGetPressed(p_obj->ch) == true)
      {
        if (millis()-p_obj->pre_time >= p_obj->repeat_start_time)
        {
          ret = true;
          p_obj->pre_time = millis();

          ret = true;
          p_obj->state = BUTTON_OBJ_REPEATED;

          p_obj->event_flag |= BUTTON_EVT_CLICKED;
          p_obj->event_flag |= BUTTON_EVT_REPEATED;

          p_obj->state_flag |= BUTTON_STATE_REPEATED;
          p_obj->click_count++;
        }
      }
      else
      {
        p_obj->state = BUTTON_OBJ_PRESSED;
        p_obj->pre_time = millis();
      }
      break;

    case BUTTON_OBJ_REPEATED:
      if (buttonGetPressed(p_obj->ch) == true)
      {
        if (millis()-p_obj->pre_time >= p_obj->repeat_pressed_time)
        {
          ret = true;
          p_obj->pre_time = millis();

          p_obj->event_flag |= BUTTON_EVT_CLICKED;
          p_obj->event_flag |= BUTTON_EVT_REPEATED;

          p_obj->state_flag |= BUTTON_STATE_REPEATED;
          p_obj->click_count++;
        }
      }
      else
      {
        p_obj->state = BUTTON_OBJ_PRESSED;
        p_obj->pre_time = millis();

      }
      break;
  }

  return ret;
}

bool buttonObjInit(button_obj_t *p_obj)
{
  p_obj->state = 0;
  p_obj->pre_time = millis();
  p_obj->event_flag = 0;
  p_obj->state_flag = 0;
  p_obj->click_count = 0;
  return true;
}

bool buttonObjUpdate(button_obj_t *p_obj)
{
  return buttonObjUpdateEx(p_obj, false);
}

bool buttonObjClearAndUpdate(button_obj_t *p_obj)
{
  return buttonObjUpdateEx(p_obj, true);
}

uint8_t buttonObjGetEvent(button_obj_t *p_obj)
{
  return p_obj->event_flag;
}

void buttonObjClearEventAll(button_obj_t *p_obj)
{
  p_obj->event_flag = 0;
}

void buttonObjClearEvent(button_obj_t *p_obj, uint8_t event_bit)
{
  p_obj->event_flag &= ~event_bit;
}

uint8_t buttonObjGetState(button_obj_t *p_obj)
{
  return p_obj->state_flag;
}
#endif



#ifdef _USE_HW_CLI

void cliButton(cli_args_t *args)
{
  bool ret = false;
  EncoderState encoder;
  uint32_t temp;

  if (args->argc == 1 && args->isStr(0, "show"))
  {
  	temp = micros();
    while(cliKeepLoop())
    {
    	encoder = Encoder_ReceiveAnalyze();
    	if(micros()-temp >= 100000)
    	{
    		temp = micros();
				for (int i=0; i<BUTTON_MAX_CH; i++)
				{
					cliPrintf("%d", buttonGetPressed(i));
				}
				cliPrintf(" Encoder : %d", encoder);
				cliPrintf("\r\n");
    	}
      //delay(100);
    }

    ret = true;
  }

#if HW_BUTTON_OBJ_USE == 1
  if (args->argc == 1 && args->isStr(0, "event"))
  {
    button_obj_t button[BUTTON_MAX_CH];
    uint8_t button_event;

    for (int i=0; i<BUTTON_MAX_CH; i++)
    {
      buttonObjCreate(&button[i], i, 50, 1000, 100);
    }

    while(cliKeepLoop())
    {
      for (int i=0; i<BUTTON_MAX_CH; i++)
      {
        buttonObjUpdate(&button[i]);

        button_event = buttonObjGetEvent(&button[i]);

        if (button_event > 0)
        {
          if (button_event & BUTTON_EVT_PRESSED)
            cliPrintf("button %d pressed\r\n", i);
          if (button_event & BUTTON_EVT_CLICKED)
            cliPrintf("button %d clicked cnt : %d\r\n", i, button[i].click_count);
          if (button_event & BUTTON_EVT_RELEASED)
            cliPrintf("button %d released\r\n", i);

          buttonObjClearEventAll(&button[i]);
        }
      }

      delay(5);
    }

    ret = true;
  }
  #endif


  if (ret != true)
  {
    cliPrintf("button show\r\n");
	#if HW_BUTTON_OBJ_USE == 1
	cliPrintf("button event\r\n");
	#endif
  }
}
#endif
