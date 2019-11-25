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
  uint32_t    NumberRetries;
}ParamTable_t;

#define PAR_TAB_MOTOR_ACTIVE_TIME_OFFSET    sizeof(lsensor_t)
#define HW_REVISEON_OFFSET          sizeof(lsensor_t)+sizeof(motor_active_time_t)
#define PAR_TAB_BAT_ALARM_LEVEL_OFFSET         HW_REVISEON_OFFSET+sizeof(uint32_t)
#define LSENS_LOWER_THRESH_OFFSET      2
#define PAR_TAB_NUM_RETR_OFFSET                PAR_TAB_BAT_ALARM_LEVEL_OFFSET+sizeof(uint32_t) 

#define device_status_t     DEVICE_STATUS_EVENT

//The type is for reports table. This table is stored in the flash
// and can be changed during operations.
typedef struct _report_t
{
  DateTime_int_t        DateTime;
  device_status_t   current_status;
}report_t;


typedef struct _main_status_t
{
//  uint32_t change_case_state_req   :  2;     //0 - idle, 1 - look request, 2 - unlook request;3 - manual;
//  uint32_t change_case_state_buzy  :  1;
  uint32_t DateTime_change_req     :  1;
  uint32_t ParamTab_change_req     :  2;
  uint32_t LightSensorWeakupTime   :  3;
  uint32_t LightSensorProblem      :  1;
  uint32_t NotifyReq               :  1;
  uint32_t MotorPowerOffReq        :  2;     //0 - not request , 1 - Request motor power off, 
                                            //2 -  Request motor power off after one second
//  uint32_t MotorAttemptCntr        :  2;
  uint32_t FlashErase_req          :  1;
  uint32_t FlashBuzy               :  1;
//  uint32_t CaseState               :  2;
//  uint32_t CaseStateError          :  1;
  uint32_t CaseStateNextEnabled    :  2;  //when CASE_STATE_REQ_IDLE: all states enabled, when others: requered state 
  uint32_t spare                   : 16;
}main_status_t;

typedef struct _rtc_tick_enable_t{
  uint32_t led_bilnk               :  1;
  uint32_t motor_buzy              :  1;
  uint32_t spare                   :  6;
}rtc_tick_enable_t;

typedef enum _case_state_req_t
{
  CASE_STATE_REQ_IDLE,
  CASE_STATE_REQ_LOOK,
  CASE_STATE_REQ_UNLOOK,
  CASE_STATE_REQ_HANDEL_OPEN,
}case_state_req_t;

typedef struct _ble_status_t
{
  int connected   : 1;
  int spare       : 31;
}ble_status_t;

typedef enum _req_t
{
  REQ_NONE,
  REQ_CHNGE,
  REQ_WRITE,
  REQ_BUZY
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

typedef enum _log_store_t
{
  LOG_STORE_NO_REQ,
  LOG_STORE_REQ,
  LOG_STORE_BUZY
}_log_store_t;

typedef enum _log_event_id_t
{
  LOG_EVENT_DEVICE_CONNECTED,
  LOG_EVENT_DEVICE_DISCONNECTED,
  LOG_EVENT_CASE_OPEN,
  LOG_EVENT_CASE_CLOSED,
  LOG_EVENT_CASE_HANDLE_OPEN,
  LOG_EVENT_CASE_STATE_TIMEOUT,
  LOG_EVENT_WIRE_CHANGED,
  LOG_EVENT_LIGHT_CHANGED,
  LOG_EVENT_BATTERY_LOW,
  LOG_EVENT_UNAUTHORIZED_CMD,
  LOG_EVENT_ERROR,                /*8*/
  LOG_EVENT_LOW_FLASH_MEMORY,
  LOG_EVENT_GPS_LOCATION
}log_event_id_t;

typedef  struct _log_event_t 
{
  DateTime_t DateTime;
  uint8_t log_event;
  uint8_t param0;
  uint8_t param1;
  uint8_t param2;
  uint32_t  store_flag;
}log_event_t;

#define EVENT_LOG_SIZE    (sizeof(log_event_t)-sizeof(uint32_t))
#define LOG_EVENT_MAX_CNT   4
typedef struct _log_event_store_t
{
  log_event_t log_event[LOG_EVENT_MAX_CNT];
  uint8_t log_event_wr_idx;
  uint8_t log_event_rd_idx;
}log_event_store_t;

typedef struct _case_state_t
{
  CASE_STATE  LastTrueCaseState;      // Last true case state
  CASE_STATE  CurrentCaseState;       // Current case state
  uint32_t change_case_state_req  :2; // if it is not CASE_STATE_REQ_IDLE - Reques change state to this state
  uint32_t UnloockStateErrCntr       :3; // Errors counter of change_case_state_req == CASE_STATE_REQ_UNLOOK
  uint32_t LookStateErrCntr          :3; // Errors counter of change_case_state_req == CASE_STATE_REQ_LOOK
  uint32_t MidleStateErrCntr         :3; // Errors counter of change_case_state_req == CASE_STATE_REQ_HANDEL_OPEN
  uint32_t UnloockStateDes        :1; // Desable transition to UNLOOK state
  uint32_t LookStateDes           :1; // Desable transition to LOOK state
  uint32_t MidleStateDes          :1; // Desable transition to HANDEL_OPEN state
}case_state_t;

#endif  //TYPES_H