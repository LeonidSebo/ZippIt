#ifndef ZIPPLT_DEF_H__
#define ZIPPLT_DEF_H__

#include "app_button.h"
#include "app_timer.h"
#include "ble.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "boards.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_drv_rtc.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
/*----------------------------------------*/
//#include "Debug.h"

#define DEVICE_NAME "ZippIT" /**< Name of device. Will be included in the advertising data. */


typedef enum _RESULT {
  ERR_NO,
  ERR_BLE_DEVICE_NOT_CONNECTED,
  ERR_BLE_CHARACTERISTIC_ID,
  //
  ERR_BLE_CMD_ID,
  ERR_CMD_GET_RANDOM_NUMBERS_NOT_FIRST,
  //
  ERR_BLE_CMD_LEN,
  ERR_BLE_CMD_DATA,

  ERR_BLE_MESSAGE_SEND,
  ERR_BLE_MODULE_BUZY,
  ERR_BLE_PARAM,

  ERR_AES_ECB_BLOCK_ENCRYPT,
  ERR_GET_RAND_BYTES,

  ERR_LOW_BATTERY,      /*12*/
  ERR_LIGHT_SENSOR_PROBLEM

} RESULT;

typedef RESULT OPERATION_STATUS;

typedef enum _BLE_COMMANDS_ID {
  CMD_ID_GET_DEVICE_INFO = 0x00,
  CMD_ID_SET_CASE_STATE = 0x01,
  CMD_ID_GET_DEVICE_STATUS = 0x02,
  CMD_ID_SET_LED_STATE = 0x03,
  CMD_ID_SET_MOTOR_TIMES = 0x04,
  CMD_ID_SET_RTC_TIME = 0x05,
  CMD_ID_SET_BATTERY_ALARM_LEVEL = 0x06,
  CMD_ID_SET_LIGHT_ALARM_LEVEL = 0x07,
  CMD_ID_SET_NUMBER_RETRIES = 0x08,
  CMD_ID_GET_RANDOM_NUMBERS = 0x09,
  CMD_ID_GET_BATTERY_CHARGING_LEVEL = 0x0A,
  CMD_ID_SET_HARDWARE_VERSION = 0x0B,

  //----------------------------
  CMD_ID_FLASH_LOG_ERRASE = 0x0C,
  CMD_ID_FLASH_LOG_READ = 0x0D,
//  CMD_ID_FLASH_SETUP_WRITE = 0x0C,
//  CMD_ID_FLASH_SETUP_READ = 0x0D,
  //--------------------------------------
  CMD_NO,
  //--------------------------------------
  CMD_ID_DEBUG = 0x50
  //CMD_ID_DEBUG = 0x50

} BLE_COMMANDS_ID;

#define ANSWER_ID_FLAG 0x80

typedef enum _BLE_MESSAGE_ID {
  MSG_NEW_RANDOM_NUMBERS = 0x00,/* not used */
  MSG_DEVICE_STATUS_CHANGED = 0x01,
  MSG_DEVICE_ERROR = 0x02, /* TBD */
  MSG_ATTENTION = 0x03,
  MSG_BLE_CMD_ID_UNKNOWN = 0x04,
  MSG_LOG_FILE_IS_FULL = 0x05,
  //MSG_RANDOM_NUMBERS              = 0x05,
} BLE_MESSAGE_ID;

typedef enum _LOG_RECORD_ID {
  LOG_ERROR = 0x00,
  LOG_ATTENTION = 0x01,
} LOG_RECORD_ID;

#define LOG_ERROR(PARAM) /* LOG_ERROR */

#define RESULT_CHECK(result) \
  if (result != ERR_NO) {    \
    return result;           \
  }

#define RESULT_CHECK_WITH_LOG(result) \
  if (result != ERR_NO) {             \
    LOG_ERROR(result);                \
    return result;                    \
  }
//===============================================
/* COMMAND Characteristic */
#define IDX_CMD_COMMAND 0
#define IDX_CMD_LENGTH 1
#define IDX_CMD_DATA 2

#define BLE_BLOCK_SIZE_BYTE 16
#define BLE_BLOCK_COMMAND_HEADER_SIZE_BYTE 2
#define BLE_BLOCK_ANSWER_HEADER_SIZE_BYTE 3
#define BLE_BLOCK_MESSAGE_HEADER_SIZE_BYTE 2
#define BLE_BLOCK_ID_SIZE_BYTE 1
#define BLE_BLOCK_DATALEN_SIZE_BYTE 1
#define BLE_BLOCK_OPERATION_STATUS_SIZE_BYTE 1
#define BLE_BLOCK_DATA_SIZE_BYTE \
  (BLE_BLOCK_SIZE_BYTE - BLE_BLOCK_ID_SIZE_BYTE - BLE_BLOCK_DATALEN_SIZE_BYTE)

typedef struct _BLE_BLOCK {
  uint8_t ID;
  uint8_t DataLength;
  uint8_t Data[BLE_BLOCK_DATA_SIZE_BYTE];
} BLE_BLOCK;

typedef struct _BLE_COMMAND {
  uint8_t CommandID;
  uint8_t DataLength;
  uint8_t Data[BLE_BLOCK_DATA_SIZE_BYTE];
} BLE_COMMAND;

typedef struct _BLE_ANSWER {
  uint8_t AnswerID;
  uint8_t AnswerLength;
  uint8_t OperationStatus;
  uint8_t Data[BLE_BLOCK_DATA_SIZE_BYTE - BLE_BLOCK_OPERATION_STATUS_SIZE_BYTE];
} BLE_ANSWER;

typedef struct _BLE_MESSAGE {
  uint8_t MessageID;
  uint8_t DataLength;
  uint8_t Data[BLE_BLOCK_DATA_SIZE_BYTE];
} BLE_MESSAGE;

//===============================================

typedef struct _HARDWARE_VERSION {
  uint8_t HV_VERSION_MINOR;
  uint8_t HV_VERSION_MAJOR;
  uint16_t HV_RESERVED;
} HARDWARE_VERSION;

typedef struct _LED_CONTROL {
  uint8_t LED_RED : 2;
  uint8_t LED_GREEN : 2;
  uint8_t LED_BLUE : 2;
  uint8_t Reserved_0 : 2;
} LED_CONTROL;

typedef enum _LED_STATE {
  LED_OFF,
  LED_ON,
  LED_BLINK
} LED_STATE;

typedef enum _MOTOR_STATE {
  MOTOR_RIGTH,
  MOTOR_LEFT,
  MOTOR_COAST,
  MOTOR_BRAKE,
} MOTOR_STATE;

typedef enum _CASE_STATE {
  CASE_UNLOCK,
  CASE_HANDEL_OPEN,
  CASE_LOCK,
  CASE_UNRESOLVED,
} CASE_STATE;

typedef uint16_t BATTERY_ALARM_LEVEL;
typedef uint16_t LIGHT_ALARM_LEVEL;
typedef uint8_t NUMBER_RETRIES;

typedef struct _RTC_VALUE {
  uint32_t SECOND : 6;
  uint32_t MINUTE : 6;
  uint32_t HOUR : 5;
  uint32_t DAY : 5;
  uint32_t MONTH : 5;
  uint32_t YEAR : 5;
} RTC_VALUE;

typedef struct _DEVICE_STATUS {
  uint32_t STATE_OF_SW_1 : 1;
  uint32_t STATE_OF_SW_2 : 1;
  uint32_t STATE_OF_SW_3 : 1;
  uint32_t WIRE_PIN : 1;
  uint32_t LIGHT_PENETRATION : 1;
  uint32_t POWER_LOW : 1;
  uint32_t Reserved_0 : 10;
  /*-------------------------------*/
  uint32_t LIGHT_SENSOR_NOT_PRESENT : 1; /* Not implemented */
  uint32_t FLASH_LOG_FULL : 1;
  uint32_t Reserved_1 : 14;
} DEVICE_STATUS;

typedef struct _DEVICE_STATUS_EVENT {
  uint32_t DEVSTAT_STATE_OF_SW_1 : 1;
  uint32_t DEVSTAT_STATE_OF_SW_1_CHANGED : 1;
  uint32_t DEVSTAT_STATE_OF_SW_2 : 1;
  uint32_t DEVSTAT_STATE_OF_SW_2_CHANGED : 1;
  uint32_t DEVSTAT_STATE_OF_SW_3 : 1;
  uint32_t DEVSTAT_STATE_OF_SW_3_CHANGED : 1;
  uint32_t DEVSTAT_WIRE_PIN : 1;
  uint32_t DEVSTAT_WIRE_PIN_CHANGED : 1;
  uint32_t DEVSTAT_LIGHT_PENETRATION : 1;
  uint32_t DEVSTAT_LIGHT_PENETRATION_CHANGED : 1;
  uint32_t DEVSTAT_POWER_LOW : 1;
  uint32_t DEVSTAT_POWER_LOW_CHANGED : 1;
  uint32_t DEVSTAT_FLASH_LOG_FULL : 1;
  uint32_t DEVSTAT_FLASH_LOG_FULL_CHANGED : 1;
  uint32_t DEVSTAT_Reserved_0 : 18;
} DEVICE_STATUS_EVENT;

typedef struct _MOTOR_ACTIVE_TIME {
  uint16_t MOTOR_CW_FULL_TIME_MS;
  uint16_t MOTOR_CW_HALF_TIME_MS;
  uint16_t MOTOR_CCW_FULL_TIME_MS;
  uint16_t MOTOR_CCW_HALF_TIME_MS;
} MOTOR_ACTIVE_TIME;

typedef struct _DEVICE_INFO {
  uint8_t SW_VERSION_MINOR;
  uint8_t SW_VERSION_MAJOR;
  uint8_t HW_VERSION_MINOR;
  uint8_t HW_VERSION_MAJOR;
} DEVICE_INFO;

#define CHARACTERISTICS_NO 3
typedef enum _CHARACTERISTIC_ID {
  CHAR_COMMAND,
  CHAR_ANSWER,
  CHAR_MESSAGE,
  CHAR_FLASH_DATA
} CHARACTERISTIC_ID;


#define UUID_BASE                                             \
  { 0x66, 0x9A, 0x0C, 0x20, 0x00, 0x08, /**/ 0x23, 0x15, /**/ \
    0xE9, 0x11, 0x35, 0xD5, 0x50, 0xB4, 0x2C, 0xCE }

/*=============================================================*/

#define UUID_SERVISE 0x1523

#define UUID_CHAR_COMMAND 0x1524
#define UUID_CHAR_ANSWER 0x1525
#define UUID_CHAR_MESSAGE 0x1526
#define UUID_CHAR_FLASH_DATA 0x1527

/*=============================================================*/
/* Debuging */
#define ADVERTISING_LED BSP_BOARD_LED_0 /**< Is on when device is advertising. */
#define CONNECTED_LED BSP_BOARD_LED_1   /**< Is on when device has connected. */
#define LEDBUTTON_LED BSP_BOARD_LED_2   /**< LED to be toggled with the help of the LED Button Service. */
#define LEDBUTTON_BUTTON BSP_BUTTON_0   /**< Button that will trigger the notification event with the LED Button Service */
/*=============================================================*/

#define APP_BLE_OBSERVER_PRIO 3 /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG 1  /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL 64                                    /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_DURATION BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */

#define MIN_CONN_INTERVAL MSEC_TO_UNITS(100, UNIT_1_25_MS) /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(200, UNIT_1_25_MS) /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY 0                                    /**< Slave latency. */
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS)   /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(20000) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000)   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT 3                        /**< Number of attempts before giving up the connection parameter negotiation. */

#endif //ZIPPLT_DEF_H__