/*
 * button.h
 *
 *  Created on: 2021. 8. 14.
 *      Author: WANG
 */

#ifndef SRC_COMMON_HW_INCLUDE_BUTTON_H_
#define SRC_COMMON_HW_INCLUDE_BUTTON_H_

#include "hw_def.h"
#include "hw.h"


#ifdef _USE_HW_BUTTON

#define BUTTON_MAX_CH         HW_BUTTON_MAX_CH

enum
{
	BTN_EN1,
	BTN_EN2,
	BTN_ENC,
};


typedef struct
{
  uint8_t  ch;
  uint8_t  state;
  uint8_t  state_pre;
  uint8_t  state_next;
  uint32_t pressed_time;
  uint32_t repeat_start_time;
  uint32_t repeat_pressed_time;
  uint32_t pre_time;

  uint8_t event_flag;
  uint8_t state_flag;
  uint8_t click_count;
} button_obj_t;

enum ButtonEvt
{
  BUTTON_EVT_PRESSED  = (1<<0),
  BUTTON_EVT_RELEASED = (1<<1),
  BUTTON_EVT_CLICKED  = (1<<2),
  BUTTON_EVT_REPEATED = (1<<3),
};

enum ButtonState
{
  BUTTON_STATE_PRESSED  = (1<<0),
  BUTTON_STATE_RELEASED = (1<<1),
  BUTTON_STATE_REPEATED = (1<<2),
};

// Wheel spin pins where BA is 00, 10, 11, 01 (1 bit always changes)
#define BLEN_A 0
#define BLEN_B 1

#define EN_A _BV(BLEN_A)
#define EN_B _BV(BLEN_B)


bool buttonInit(void);
bool buttonGetPressed(uint8_t ch);

void buttonObjCreate(button_obj_t *p_obj, uint8_t ch, uint32_t pressed_time, uint32_t repeat_start_time, uint32_t repeat_pressed_time);
bool buttonObjInit(button_obj_t *p_obj);
bool buttonObjUpdate(button_obj_t *p_obj);
bool buttonObjClearAndUpdate(button_obj_t *p_obj);
uint8_t buttonObjGetEvent(button_obj_t *p_obj);
void buttonObjClearEventAll(button_obj_t *p_obj);
void buttonObjClearEvent(button_obj_t *p_obj, uint8_t event_bit);
uint8_t buttonObjGetState(button_obj_t *p_obj);


#endif

#endif /* SRC_COMMON_HW_INCLUDE_BUTTON_H_ */
