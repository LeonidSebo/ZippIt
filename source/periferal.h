#ifndef PERIFERAL_H
#define PERIFERAL_H


#include "sdk_config.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "app_error.h"
#include "boards.h"
#include "types.h"
#include "CmdHandler.h"
#include "nrf_drv_saadc.h"
#include "nrf_nvmc.h"
#include "app_error.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"
#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"
#include "app_button.h"
#include "crc32.h"
#include "nrfx_saadc.h"


#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                     /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */


#ifdef BSP_LED_0
    #define GREEN_LED     BSP_LED_1                            /**< Pin number for indicating tick event. */
#endif
#ifdef BSP_LED_1
    #define RED_LED   BSP_LED_2                                /**< Pin number for indicating compare event. */
#endif

#define  LED_BLINK_ON_TIME_TICK     2

#define  SW_EVENT_TIMEOUT           2

//#define MOTOR_STOP()              nrf_gpio_pin_clear(MOTOR_IN1_PIN);	\
//                                  nrf_gpio_pin_clear(MOTOR_IN2_PIN);	\
//                                  nrf_gpio_pin_clear(MOTOR_EN_PIN);	\
//                                  nrf_gpio_pin_clear(EN_6V_PIN);

#define MOTOR_STOP()              nrf_gpio_pin_set(MOTOR_IN1_PIN);	\
                                  nrf_gpio_pin_set(MOTOR_IN2_PIN);	\
                                  nrf_gpio_pin_set(MOTOR_EN_PIN);	\
                                  nrf_gpio_pin_set(EN_6V_PIN);          \
                                  main_status.MotorPowerOffReq = 2;

#define MOTOR_CLOSE_CASE()        nrf_gpio_pin_set(EN_6V_PIN);          \
                                  nrf_gpio_pin_set(MOTOR_IN1_PIN);	\
                                  nrf_gpio_pin_clear(MOTOR_IN2_PIN);	\
                                  nrf_gpio_pin_set(MOTOR_EN_PIN);	\
                                  main_status.MotorPowerOffReq = 0;     \
                                  NRF_LOG_INFO("CLOSE CASE");

#define MOTOR_OPEN_CASE()         nrf_gpio_pin_set(EN_6V_PIN);          \
                                  nrf_gpio_pin_clear(MOTOR_IN1_PIN);	\
                                  nrf_gpio_pin_set(MOTOR_IN2_PIN);	\
                                  nrf_gpio_pin_set(MOTOR_EN_PIN);	\
                                  main_status.MotorPowerOffReq = 0;     \
                                  NRF_LOG_INFO("OPEN CASE");

#define MOTOR_POWER_OFF()         nrf_gpio_pin_clear(EN_6V_PIN);        \
                                  nrf_gpio_pin_clear(MOTOR_IN1_PIN);	\
                                  nrf_gpio_pin_clear(MOTOR_IN2_PIN);	\
                                  nrf_gpio_pin_clear(MOTOR_EN_PIN);	\
                                  main_status.MotorPowerOffReq = 0;     


#define DEF_MOTOR_ACTIVE_TIME     0xb8,0x0b,0xdc,0x05,0xb8,0x0b,0xdc,0x05
#define DEF_HW_REVISION           0x00,0x01,0x00,0x00
#define DEF_BAT_ALARM_LEVEL       0xBC,0x02,0x00,0x00
#define DEF_CRC                   0x00,0x00,0x00,0x00
#define DEF_NUM_RETR              0x01,0x00,0x00,0x00
#define DEF_PARAM_TAB             LSENSOR_DEF,\
                                  DEF_MOTOR_ACTIVE_TIME,\
                                  DEF_HW_REVISION,\
                                  DEF_BAT_ALARM_LEVEL,\
                                  DEF_NUM_RETR,\
                                  DEF_CRC

#define     LIGHT_SENSOR_WEAKUP_ENABLE()              main_status.LightSensorWeakupTime = SW_EVENT_TIMEOUT

#define LOG_EVENT_TAB_START_ADDR              (uint32_t)pParamTable + 0x1000 

#define RTC_TICK_TIME       125

#define INT_FLESH_ALMOST_FULL_REST    0x100

//#define PARAM_TAB_ADDR      0x10000
#define PAGE_SIZE           0x01000
//#define PAGE_SIZE_WORDS     0x00400

#define  CASE_STATE_LED_ON_TIME               6
#define  CASE_STATE_ERROR_LED_BLINK_TIME      31
#define  ALARM_BLINK_TIME                     31
#define  MAX_CASE_STATE_ERR_CNT               5


uint32_t find_free_addr(uint32_t start_addr);
void int_flash_read(uint32_t addr, uint32_t* pdata, size_t size);
void int_flash_erase(uint32_t addr, size_t pages_cnt);
void int_flash_write(uint32_t addr, uint32_t* pdata, size_t size);
void get_current_status(void);
void init_periferal(void);
void logEventStorageReq(log_event_id_t event,uint8_t param0,uint8_t param1,uint8_t param2);
void TickRetries_16s();


#endif