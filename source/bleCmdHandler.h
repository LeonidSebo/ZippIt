#ifndef  BLECMDHEADER_H
#define  BLECMDHEADER_H
#include <stdint.h>
#include "types.h"
#include "ble_def.h"
#include "nrf_drv_rtc.h"
//#include "int_flash.h"
#include "periferal.h"


RESULT bleGetDeviceInfo(DEVICE_INFO* pDeviceInfo);
RESULT bleSetCaseState(CASE_STATE CaseState);
RESULT bleSetLedState(LED_CONTROL LedControl);
RESULT bleSetMotorTimes(MOTOR_ACTIVE_TIME MotorTimes);
RESULT bleSetRtcTime(uint32_t DateTime);
RESULT bleSetBatteryAlarmLevel(uint32_t BatLevel);

#define bleFlashErase     int_flash_erase   //    (uint32_t addr, size_t pages_cnt)
#define bleFlashWrite     int_flash_write   //    (uint32_t addr, uint32_t* pdata, size_t size)
#define bleFlashRead      int_flash_read    //    (uint32_t addr, uint32_t* pdata, size_t size)

#endif  //BLECMDHEADER_H