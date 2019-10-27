#ifndef TYPES_H
#define TYPES_H

#include "lsensor.h"
//#include "rtc.h"
#include "BLE_Def.h"

/******* RTC *****************/

#define TICK_PER_SEC  8

typedef struct _DateTime_t
{
  unsigned tm_sec   : 6;
  unsigned tm_min   : 6;
  unsigned tm_hour  : 5;
  unsigned tm_mday  : 5;
  unsigned tm_mon   : 5;
  unsigned tm_year  : 5;
} DateTime_t;

typedef union  _DateTime_int_t
{
  DateTime_t  AsStruct;
  uint32_t     AsInt;
}DateTime_int_t;

typedef  enum _Month_t
{
  JANUARY = 1,
  FEBRUARY,
  MARCH,
  APRIL,
  MAY,
  JUNE,
  JULY,
  AUGUST,
  SEPTEMBER,
  OCTOBER,
  NOVEMBER,
  DECEMBER
}Month_t;

#define SECOND_PER_MINUTE             60
#define MINUTES_PER_HOUR              60
#define HOURS_PER_DAY                 24

#define FEBRUARY_NO_LEAP_YEAR_LEN     28
#define FEBRUARY_LEAP_YEAR_LEN        29
#define LONG_MONTH_LEN                31
#define SHORT_MONTH_LEN               30
/*************************************/

#define motor_active_time_t   MOTOR_ACTIVE_TIME

//The type is for table of parameters. This table is stored in the flash
// and can be changed during operations.
typedef struct _ParamTable_t
{
//  uint32_t    id;
  lsensor_t   lsensor;
  motor_active_time_t    MotorActiveTime;
  uint32_t    HW_revision;
  uint32_t    BatteryAlarmLevel;
}ParamTable_t;

#define device_status_t     DEVICE_STATUS
//typedef struct _device_status_t
//{
//  uint32_t DEVSTAT_SW1          : 1;
//  uint32_t DEVSTAT_SW2          : 1;
//  uint32_t DEVSTAT_SW3          : 1;
//  uint32_t DEVSTAT_WIRE_PIN     : 1;
//  uint32_t DEVSTAT_POWER        : 1;
//  uint32_t LIGHT_SENSOR         : 1;
//  uint32_t unused0              : 2;
//  uint32_t ALARM_POWER          : 2;
//  uint32_t ALARM_LIGHT          : 2;
//
//  uint32_t DEVSTAT_Reserved_0   : 23;  
//}device_status_t;

//typedef union _current_state_int_t 
//{
//  device_status_t  AsStruct;
//  uint32_t        AsInt;
//}device_status_int_t;

//The type is for reports table. This table is stored in the flash
// and can be changed during operations.
typedef struct _report_t
{
  DateTime_int_t        DateTime;
  device_status_t   current_status;
}report_t;


typedef struct _main_status_t
{
  uint32_t change_case_state_req   :  2;     //0 - idle, 1 - look request, 2 - unlook request;3 - manual;
  uint32_t change_case_state_buzy  :  1;
  uint32_t DateTime_change_req     :  1;
  uint32_t ParamTab_change_req     :  2;
  uint32_t LockSwitchEventTime     :  3;
  uint32_t StoreDevStatusReq       :  1;
  uint32_t NotifyReq               :  1;
  uint32_t WireEventTime           :  3;
  uint32_t MotorPowerOffReq        :  2;     //0 - not request , 1 - Request motor power off, 
                                            //2 -  Request motor power off after one second
  uint32_t MotorAttemptCntr        :  2;
  uint32_t FlashEraseBuzy         :  1;
  uint32_t FlashWriteBuzy         :  1;
  uint32_t spare                   : 11;
}main_status_t;

typedef struct _rtc_tick_enable_t{
  uint32_t led_bilnk               :  1;
  uint32_t motor_buzy                       :  1;
  uint32_t spare                   : 20;
}rtc_tick_enable_t;

typedef enum _case_state_req_t
{
  CASE_STATE_REQ_IDLE,
  CASE_STATE_REQ_LOOK,
  CASE_STATE_REQ_UNLOOK,
  CASE_STATE_REQ_MANUAL,
}case_state_req_t;

typedef struct _ble_status_t
{
  int connected   : 1;
  int spare       : 31;
}ble_status_t;

typedef enum _req_t
{
  REQ_NONE,
  REQ,
  REQ_ERASE_IN_PROGRESS,
  REQ_WRITE_IN_PROGRESS
}req_t;

typedef enum _pow_st_t
{
  LOW,
  HIGHT
}pow_st_t;

#define YES   1
#define NO    0


typedef enum _alarm_t
{
  ALARM_NONE,
  ALARM,
  ALARM_SENT,
  ALARM_UNUSED
}alarm_t;
#define led_control_t  LED_CONTROL
#define led_state_t    LED_STATE


#endif  //TYPES_H