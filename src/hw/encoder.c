/*
 * encoder.c
 *
 *  Created on: Jan 22, 2024
 *      Author: WANG
 */

#include "encoder.h"


#define ENCODER_PULSES_PER_STEP 4
#define ENCODER_WAIT_MS                  20

ENCODER_Rate EncoderRate;

EncoderState get_encoder_state() {
  static millis_t Encoder_ms = 0;
  const millis_t ms = millis();
  if (PENDING(ms, Encoder_ms)) return ENCODER_DIFF_NO;
  const EncoderState state = Encoder_ReceiveAnalyze();
  if (state != ENCODER_DIFF_NO) Encoder_ms = ms + ENCODER_WAIT_MS;
  return state;
}


// Analyze encoder value and return state
EncoderState Encoder_ReceiveAnalyze() {
  const millis_t now = millis();
  static uint8_t lastEncoderBits;
  uint8_t newbutton = 0;
  static signed char temp_diff = 0;

  EncoderState temp_diffState = ENCODER_DIFF_NO;
  if (buttonGetPressed(BTN_EN1)) newbutton |= EN_A;
  if (buttonGetPressed(BTN_EN2)) newbutton |= EN_B;
  if (buttonGetPressed(BTN_ENC)) {
    static millis_t next_click_update_ms;
    if (ELAPSED(now, next_click_update_ms)) {
      next_click_update_ms = millis() + 300;
      //Encoder_tick();

//      if (!ui.backlight) ui.refresh_brightness();
//      const bool was_waiting = wait_for_user;
//      wait_for_user = false;
//      return was_waiting ? ENCODER_DIFF_NO : ENCODER_DIFF_ENTER;
    }
    else return ENCODER_DIFF_NO;
  }
  if (newbutton != lastEncoderBits) {
    switch (newbutton) {
      case ENCODER_PHASE_0:
             if (lastEncoderBits == ENCODER_PHASE_3) temp_diff++;
        else if (lastEncoderBits == ENCODER_PHASE_1) temp_diff--;
        break;
      case ENCODER_PHASE_1:
             if (lastEncoderBits == ENCODER_PHASE_0) temp_diff++;
        else if (lastEncoderBits == ENCODER_PHASE_2) temp_diff--;
        break;
      case ENCODER_PHASE_2:
             if (lastEncoderBits == ENCODER_PHASE_1) temp_diff++;
        else if (lastEncoderBits == ENCODER_PHASE_3) temp_diff--;
        break;
      case ENCODER_PHASE_3:
             if (lastEncoderBits == ENCODER_PHASE_2) temp_diff++;
        else if (lastEncoderBits == ENCODER_PHASE_0) temp_diff--;
        break;
    }
    lastEncoderBits = newbutton;
  }

  if (ABS(temp_diff) >= ENCODER_PULSES_PER_STEP) {
    if (temp_diff > 0) temp_diffState = TERN(REVERSE_ENCODER_DIRECTION, ENCODER_DIFF_CCW, ENCODER_DIFF_CW);
    else temp_diffState = TERN(REVERSE_ENCODER_DIRECTION, ENCODER_DIFF_CW, ENCODER_DIFF_CCW);

      millis_t ms = millis();
      int32_t encoderMultiplier = 1;

      // if must encoder rati multiplier
      if (EncoderRate.enabled) {
        const float abs_diff = ABS(temp_diff),
                    encoderMovementSteps = abs_diff / (ENCODER_PULSES_PER_STEP);
        if (EncoderRate.lastEncoderTime) {
          // Note that the rate is always calculated between two passes through the
          // loop and that the abs of the temp_diff value is tracked.
          const float encoderStepRate = encoderMovementSteps / (float)(ms - EncoderRate.lastEncoderTime) * 1000;
               if (encoderStepRate >= ENCODER_100X_STEPS_PER_SEC) encoderMultiplier = 100;
          else if (encoderStepRate >= ENCODER_10X_STEPS_PER_SEC)  encoderMultiplier = 10;
          #if ENCODER_5X_STEPS_PER_SEC
            else if (encoderStepRate >= ENCODER_5X_STEPS_PER_SEC) encoderMultiplier = 5;
          #endif
        }
        EncoderRate.lastEncoderTime = ms;
      }

    // EncoderRate.encoderMoveValue += (temp_diff * encoderMultiplier) / (ENCODER_PULSES_PER_STEP);
    EncoderRate.encoderMoveValue = (temp_diff * encoderMultiplier) / (ENCODER_PULSES_PER_STEP);
    if (EncoderRate.encoderMoveValue < 0) EncoderRate.encoderMoveValue = -EncoderRate.encoderMoveValue;

    temp_diff = 0;
  }
  return temp_diffState;
}
