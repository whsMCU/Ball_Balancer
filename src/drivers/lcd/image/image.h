/*
 * image.h
 *
 *  Created on: 2021. 9. 12.
 *      Author: WANG
 */

#ifndef SRC_HW_DRIVER_IMAGE_IMAGE_H_
#define SRC_HW_DRIVER_IMAGE_IMAGE_H_

#include "hw.h"

extern const uint16_t fan0_32x32x4[], fan1_32x32x4[];
extern const uint8_t fan_slow0_64x64x4[], fan_slow1_64x64x4[];
extern const uint8_t fan_fast0_64x64x4[], fan_fast1_64x64x4[];

enum Image {
  imgFanIdle,
  imgFanSlow0,
  imgFanSlow1,
  imgFanFast0,
  imgFanFast1,
  imgCount,
};

typedef struct __attribute__((__packed__)) {
  void *data;
  uint16_t width;
  uint16_t height;
} UI_Image;

extern const UI_Image Fan0_32x32x4;
extern const UI_Image Fan1_32x32x4;
extern const UI_Image Fan_Slow0_64x64x4;
extern const UI_Image Fan_Slow1_64x64x4;
extern const UI_Image Fan_Fast0_64x64x4;
extern const UI_Image Fan_Fast1_64x64x4;


#endif /* SRC_HW_DRIVER_IMAGE_IMAGE_H_ */
