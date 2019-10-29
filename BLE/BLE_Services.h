/*
 * @defgroup ble_lbs LED Button Service Server
 * @{
 * @ingroup ble_sdk_srv
 *
 * @brief LED Button Service Server module.
 *
 * @details This module implements a custom LED Button Service with an LED and Button Characteristics.
 *          During initialization, the module adds the LED Button Service and Characteristics
 *          to the BLE stack database.
 *
 *          The application must supply an event handler for receiving LED Button Service
 *          events. Using this handler, the service notifies the application when the
 *          LED value changes.
 *
 *          The service also provides a function for letting the application notify
 *          the state of the Button Characteristic to connected peers.
 *
 * @note    The application must register this module as BLE event observer using the
 *          NRF_SDH_BLE_OBSERVER macro. Example:
 *          @code
 *              ble_hids_t instance;
 *              NRF_SDH_BLE_OBSERVER(anything, BLE_HIDS_BLE_OBSERVER_PRIO,
 *                                   ble_hids_on_ble_evt, &instance);
 *          @endcode
 */

#ifndef ZIPPLT_BLE_H__
#define ZIPPLT_BLE_H__

#include "BLE_Def.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**@brief   Macro for defining a ble_lbs instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_MAIN_SERVICE_DEF(_name)    \
  /*static*/ ble_main_service_t _name; \
  NRF_SDH_BLE_OBSERVER(_name##_obs,    \
      BLE_LBS_BLE_OBSERVER_PRIO,       \
      ble_main_service_on_ble_evt, &_name)

// Forward declaration of the ble_main_service_t type.
typedef struct BLE_MAIN_SERVICE ble_main_service_t;

typedef void (*evhCommandWrite_t)(uint16_t conn_handle, ble_main_service_t *p_lbs, uint16_t CommandLen, uint8_t *pCommand);

//typedef void (*evhCommandWrite_t) (uint16_t conn_handle, ble_main_service_t * p_lbs, uint16_t CommandLen, uint8_t *pCommand);
//typedef void (*hRtsWrite_t) (uint16_t conn_handle, ble_main_service_t * p_lbs, uint32_t new_state);
//typedef void (*hMotorActiveTimeWrite_t) (uint16_t conn_handle, ble_main_service_t * p_lbs, uint32_t new_state);
//typedef void (*hNumberRetriesWrite_t) (uint16_t conn_handle, ble_main_service_t * p_lbs, uint8_t new_state);

/** @brief LED Button Service init structure. This structure contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
  evhCommandWrite_t evhCommandWrite; /**< Event handler to be called when the LED Characteristic is written. */
} BLE_MAIN_SERVICE_INIT;

/**@brief ZippIT Service structure. This structure contains various status information for the service. */
struct BLE_MAIN_SERVICE {
  uint16_t service_handle;                /**< Handle of ZippIT Service. */
  ble_gatts_char_handles_t hCommandChar;  /**< Handle related to the COMMAND Characteristic. */
  ble_gatts_char_handles_t hAnswerChar;   /**< Handle related to the Answer Characteristic. */
  ble_gatts_char_handles_t hMessageChar;  /**< Handle related to the DEVICE_STSTUS Characteristic. */
  ble_gatts_char_handles_t hFlashDatChar; /**< Handle related to the DEVICE_STSTUS Characteristic. */
  uint8_t uuid_type;                      /**< UUID type for the ZippIT Button Service. */

  evhCommandWrite_t evhCommandWrite; /**< Event handler to be called when the COMMAND is written. */
};

/**@brief Function for initializing the LED Button Service.
 *
 * @param[out] p_lbs      LED Button Service structure. This structure must be supplied by
 *                        the application. It is initialized by this function and will later
 *                        be used to identify this particular service instance.
 * @param[in] p_main_service_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was initialized successfully. Otherwise, an error code is returned.
 */
uint32_t ble_main_service_init(ble_main_service_t *p_lbs, const BLE_MAIN_SERVICE_INIT *p_main_service_init);

/**@brief Function for handling the application's BLE stack events.
 *
 * @details This function handles all events from the BLE stack that are of interest to the LED Button Service.
 *
 * @param[in] p_ble_evt  Event received from the BLE stack.
 * @param[in] p_context  LED Button Service structure.
 */
void ble_main_service_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context);

/**@brief Function for sending a button state notification.
 *
 ' @param[in] conn_handle   Handle of the peripheral connection to which the button state notification will be sent.
 * @param[in] p_lbs         LED Button Service structure.
 * @param[in] button_state  New button state.
 *
 * @retval NRF_SUCCESS If the notification was sent successfully. Otherwise, an error code is returned.
 */
//uint32_t DeviceStatusNotification(uint16_t conn_handle, ble_main_service_t * p_lbs, uint16_t DeviceStatus);
//uint32_t AnswerNotification(uint16_t conn_handle, ble_main_service_t * p_lbs, uint16_t AnswerLen, uint8_t *pAnswer);

RESULT Serv_SendToHost(CHARACTERISTIC_ID CharId, uint8_t *pData, uint16_t DataLen);
RESULT Serv_IsDeviceConnected();

#ifdef __cplusplus
}
#endif

#endif // ZIPPLT_BLE_H__

/** @} */