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
RESULT bleFlashLogErase(void);
RESULT bleGetBatteryVoltage(uint16_t* BatteryVoltage);
RESULT bleSetLightAlarmLevel(uint16_t LightAlarm);
RESULT bleSetHardwareVersion(HARDWARE_VERSION HardwareVersion);
RESULT bleGetDeviceStatus(DEVICE_STATUS* pDeviceStatus);
RESULT bleSetNumberRetries(uint8_t NumberRetries);
RESULT bleFlashLogRead(uint32_t Offset, uint8_t DataLength, uint32_t* Data, uint8_t* DataLengthRet);

uint8_t bleGetNumberRetries();

void bleShowParamTab(void);


#define bleFlashErase     int_flash_erase   //    (uint32_t addr, size_t pages_cnt)
#define bleFlashWrite     int_flash_write   //    (uint32_t addr, uint32_t* pdata, size_t size)
#define bleFlashRead      int_flash_read    //    (uint32_t addr, uint32_t* pdata, size_t size)

#endif  //BLECMDHEADER_H