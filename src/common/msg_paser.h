/*
 * msg_paser.h
 *
 *  Created on: Aug 26, 2024
 *      Author: WANG
 */

#ifndef SRC_COMMON_MSG_PASER_H_
#define SRC_COMMON_MSG_PASER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "hw.h"

extern double kp, ki, kd;
extern uint8_t telemetry_tx_buf[40];

void msg_paser(void);

#ifdef __cplusplus
}
#endif
#endif /* SRC_COMMON_MSG_PASER_H_ */
