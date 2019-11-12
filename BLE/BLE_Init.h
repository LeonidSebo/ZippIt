#ifndef BLE_INIT_H__
#define BLE_INIT_H__

#include "BLE_Def.h"
#include "BLE_Services.h"
//#include "Debug.h"

RESULT BLE_Init(void);
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name);
static void ble_stack_init(void);
static void services_init(void);
static void advertising_init(void);
static RESULT advertising_start(void);
static void gap_params_init(void);
static void gatt_init(void);
void services_init(void);
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt);
static void nrf_qwr_error_handler(uint32_t nrf_error);
static void conn_params_init(void);
static void conn_params_error_handler(uint32_t nrf_error);
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context);

static RESULT advertising_stop(void);
RESULT sleep(void);
RESULT wake(void);

#endif /* BLE_INIT_H__ */