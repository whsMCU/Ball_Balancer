#include "usb.h"

#ifdef _USE_HW_USB
#include "usb_device.h"


bool usbInit(void)
{
  bool ret = true;

  MX_USB_DEVICE_Init();
  HAL_Delay(100);

  logPrintf("[%s] usb_Init()\n", ret ? "OK":"NG");

  return ret;
}


#endif
