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

#define  SW_EVENT_TIMEOUT           4

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
                                  main_status.MotorPowerOffReq = 0;

#define MOTOR_OPEN_CASE()         nrf_gpio_pin_set(EN_6V_PIN);          \
                                  nrf_gpio_pin_clear(MOTOR_IN1_PIN);	\
                                  nrf_gpio_pin_set(MOTOR_IN2_PIN);	\
                                  nrf_gpio_pin_set(MOTOR_EN_PIN);	\
                                  main_status.MotorPowerOffReq = 0;     

#define MOTOR_POWER_OFF()         nrf_gpio_pin_clear(EN_6V_PIN);        \
                                  nrf_gpio_pin_clear(MOTOR_IN1_PIN);	\
                                  nrf_gpio_pin_clear(MOTOR_IN2_PIN);	\
                                  nrf_gpio_pin_clear(MOTOR_EN_PIN);	\
                                  main_status.MotorPowerOffReq = 0;     

#define PARAM_TAB_ID            0x4C656F6E
#define MOTOR_TIMEOUT_DEF {3000,1500,3000,1500}
#define BATTERY_ALARM_LEVEL_DEF     0x0100

#define RTC_TICK_TIME       125

#define INT_FLESH_START_ADDR    0x10000
#define INT_FLESH_LAST_ADDR     0x7FFFF
#define REPORTS_START_ADDR      0x3A000

#define PARAM_TAB_ADDR      0x10000
#define PAGE_SIZE           0x01000
#define PAGE_SIZE_WORDS     0x00400

#define  CASE_STATE_LED_ON_TIME               6
#define  CASE_STATE_ERROR_LED_BLINK_TIME      31
#define  ALARM_BLINK_TIME                     301

uint32_t find_free_addr(uint32_t start_addr);
//void fstorage_init(void);
void int_flash_read(uint32_t addr, uint32_t* pdata, size_t size);
void int_flash_erase(uint32_t addr, size_t pages_cnt);
void int_flash_write(uint32_t addr, uint32_t* pdata, size_t size);
void WriteParamTab(void);


void gpio_init(void);
//void motor_ctrl(void);
void saadc_init(void);
void incr_date_time(DateTime_t* pDateTime);
void set_date_time(DateTime_int_t* CurrentDateTime, uint32_t NewDateTime);
uint32_t get_current_date_time(void);
void LockSwitchEvent_handler(void);
void StoreDevStatus(void);
void get_current_status(void);
void WireEvent_handler(void);
void rtc_config(void);
void init_periferal(void);

//void buttons_init(void);

#endif