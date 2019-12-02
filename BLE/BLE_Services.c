#include "sdk_common.h"

#if NRF_MODULE_ENABLED(BLE_LBS)

#include "BLE_Services.h"
#include "ble_srv_common.h"

extern uint16_t m_conn_handle;
extern ble_main_service_t m_lbs;

/* BLE Characteristics */
///* ==== Characteristics variable ==== */
//#define CHAR_COMMAND_SIZE 16
//#define CHAR_ANSWER_SIZE 16
//#define CHAR_MESSAGE_SIZE 16
//#define CHAR_FLASH_DATA_SIZE 256

uint8_t gCommand[CHAR_COMMAND_SIZE];        /* Write */
uint8_t gAnswer[CHAR_ANSWER_SIZE];          /* Indication */
uint8_t gMessage[CHAR_MESSAGE_SIZE];        /* Indication */
uint8_t gFlashData[CHAR_FLASH_DATA_SIZE];   /* Notification */

/* ================================== */

/**@brief Function for handling the Write event.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
/*static*/ void on_write(ble_main_service_t *p_lbs, ble_evt_t const *p_ble_evt) {
  ble_gatts_evt_write_t const *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

  if ((p_evt_write->handle == p_lbs->hCommandChar.value_handle)
      //&& (p_evt_write->len == 1)
      && (p_lbs->evhCommandWrite != NULL)) {
    p_lbs->evhCommandWrite(p_ble_evt->evt.gap_evt.conn_handle, p_lbs, p_evt_write->len, (uint8_t *)&p_evt_write->data);
  }
}

void ble_main_service_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context) {
  ble_main_service_t *p_lbs = (ble_main_service_t *)p_context;

  switch (p_ble_evt->header.evt_id) {
  case BLE_GATTS_EVT_WRITE:
    on_write(p_lbs, p_ble_evt);
    break;

  default:
    // No implementation needed.
    p_lbs = (ble_main_service_t *)p_context;
    break;
  }
}

uint32_t ble_main_service_init(ble_main_service_t *p_lbs, const BLE_MAIN_SERVICE_INIT *p_main_service_init) {
  uint32_t err_code;
  ble_uuid_t ble_uuid;
  ble_add_char_params_t add_char_params;

  // Initialize service structure.
  p_lbs->evhCommandWrite = p_main_service_init->evhCommandWrite;

  // Add service.
  ble_uuid128_t base_uuid = {UUID_BASE};
  err_code = sd_ble_uuid_vs_add(&base_uuid, &p_lbs->uuid_type);
  VERIFY_SUCCESS(err_code);

  ble_uuid.type = p_lbs->uuid_type;
  ble_uuid.uuid = UUID_SERVISE;

  err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_lbs->service_handle);
  VERIFY_SUCCESS(err_code);

  /* ===================================*/
  /* === Add COMMAND characteristic. ===*/
  /* ===================++++============*/
  memset(&add_char_params, 0, sizeof(add_char_params));
  add_char_params.uuid = UUID_CHAR_COMMAND;
  add_char_params.uuid_type = p_lbs->uuid_type;
  add_char_params.init_len = CHAR_COMMAND_SIZE; //sizeof(uint8_t) * 16;
  add_char_params.max_len = CHAR_COMMAND_SIZE;  //sizeof(uint8_t) * 16;
  //add_char_params.char_props.read     = 1;
  add_char_params.char_props.write = 1;

  add_char_params.is_value_user = true;
  //gCommand[0]            = 0x00;
  add_char_params.p_init_value = gCommand;

  //add_char_params.read_access         = SEC_OPEN;
  add_char_params.write_access = SEC_OPEN;

  err_code = characteristic_add(p_lbs->service_handle,
      &add_char_params,
      &p_lbs->hCommandChar);
  if (err_code != NRF_SUCCESS) {
    return err_code;
  }

  /* ==================================*/
  /* === Add ANSWER characteristic. ===*/
  /* ==================================*/
  memset(&add_char_params, 0, sizeof(add_char_params));
  add_char_params.uuid = UUID_CHAR_ANSWER;
  add_char_params.uuid_type = p_lbs->uuid_type;
  add_char_params.init_len = CHAR_ANSWER_SIZE; //sizeof(uint8_t);
  add_char_params.max_len = CHAR_ANSWER_SIZE;  //sizeof(uint8_t);
  //add_char_params.char_props.read     = 1;
  //add_char_params.char_props.notify   = 1;
  add_char_params.char_props.indicate = 1;

  add_char_params.is_value_user = true;
  //gDeviceStatus                       = 0x0000;
  add_char_params.p_init_value = (uint8_t *)&gAnswer;

  //add_char_params.read_access         = SEC_OPEN;
  add_char_params.cccd_write_access = SEC_OPEN;

  err_code = characteristic_add(p_lbs->service_handle,
      &add_char_params,
      &p_lbs->hAnswerChar);
  if (err_code != NRF_SUCCESS) {
    return err_code;
  }

  /* =========================================*/
  /* === Add MESSAGE characteristic. ===*/
  /* =========================================*/
  memset(&add_char_params, 0, sizeof(add_char_params));
  add_char_params.uuid = UUID_CHAR_MESSAGE;
  add_char_params.uuid_type = p_lbs->uuid_type;
  add_char_params.init_len = CHAR_MESSAGE_SIZE; //sizeof(uint16_t);
  add_char_params.max_len = CHAR_MESSAGE_SIZE;  //sizeof(uint16_t);
  //add_char_params.char_props.read     = 1;
  //add_char_params.char_props.notify   = 1;
  add_char_params.char_props.indicate = 1;

  add_char_params.is_value_user = true;
  //gDeviceStatus                       = 0x0000;
  add_char_params.p_init_value = (uint8_t *)&gMessage;

  //add_char_params.read_access         = SEC_OPEN;
  add_char_params.cccd_write_access = SEC_OPEN;

  err_code = characteristic_add(p_lbs->service_handle,
      &add_char_params,
      &p_lbs->hMessageChar);
  if (err_code != NRF_SUCCESS) {
    return err_code;
  }

  /* ========================================*/
  /* === Add FLASH_DATA characteristic.   ===*/
  /* ========================================*/
  memset(&add_char_params, 0, sizeof(add_char_params));
  add_char_params.uuid = UUID_CHAR_FLASH_DATA;
  add_char_params.uuid_type = p_lbs->uuid_type;
  add_char_params.init_len = CHAR_FLASH_DATA_SIZE;
  add_char_params.max_len = CHAR_FLASH_DATA_SIZE;
  //add_char_params.char_props.read     = 1;
  add_char_params.char_props.notify   = 1;
  //add_char_params.char_props.indicate = 1;

  add_char_params.is_value_user = false;
  //gDeviceStatus                       = 0x0000;
  //add_char_params.p_init_value = (uint8_t *)&gFlashData;

  //add_char_params.read_access         = SEC_OPEN;
  add_char_params.cccd_write_access = SEC_OPEN;

  err_code = characteristic_add(p_lbs->service_handle,
      &add_char_params,
      &p_lbs->hFlashDatChar);
  if (err_code != NRF_SUCCESS) {
    return err_code;
  }
  return err_code;
}

//uint32_t AnswerNotification(uint16_t conn_handle, ble_main_service_t * p_lbs, uint16_t AnswerLen, uint8_t *pAnswer)
//{
//    ble_gatts_hvx_params_t params;
//
//    memset(&params, 0, sizeof(params));
//    params.type   = BLE_GATT_HVX_NOTIFICATION;
//    params.handle = p_lbs->hAnswerChar.value_handle;
//    params.p_data = pAnswer;
//    params.p_len  = &AnswerLen;
//
//    return sd_ble_gatts_hvx(conn_handle, &params);
//}
//
//uint32_t DeviceStatusNotification(uint16_t conn_handle, ble_main_service_t * p_lbs, uint16_t DeviceStatus)
//{
//    ble_gatts_hvx_params_t params;
//    uint16_t len = sizeof(DeviceStatus);
//
//    memset(&params, 0, sizeof(params));
//    params.type   = BLE_GATT_HVX_NOTIFICATION;
//    params.handle = p_lbs->hDeviceStatusChar.value_handle;
//    params.p_data = (uint8_t*)&DeviceStatus;
//    params.p_len  = &len;
//
//    return sd_ble_gatts_hvx(conn_handle, &params);
//}

RESULT Serv_IsDeviceConnected() {
  return (m_conn_handle == BLE_CONN_HANDLE_INVALID) ? ERR_BLE_DEVICE_NOT_CONNECTED : ERR_NO;
}

RESULT Serv_SendToHost(CHARACTERISTIC_ID CharId, uint8_t *pData, uint16_t DataLen) {
  uint32_t res32;
  RESULT res = ERR_NO;
  ble_gatts_hvx_params_t params;

  res = Serv_IsDeviceConnected();
  RESULT_CHECK(res);

  memset(&params, 0, sizeof(params));
  //params.type   = BLE_GATT_HVX_NOTIFICATION;
  switch (CharId) {
  case CHAR_ANSWER:
    params.type = BLE_GATT_HVX_INDICATION;
    params.handle = m_lbs.hAnswerChar.value_handle;
    break;

  case CHAR_MESSAGE:
    params.type = BLE_GATT_HVX_INDICATION;
    params.handle = m_lbs.hMessageChar.value_handle;
    break;
  
  case CHAR_FLASH_DATA:
    params.type = BLE_GATT_HVX_NOTIFICATION;
    params.handle = m_lbs.hFlashDatChar.value_handle;
    break;

  default:
    return ERR_BLE_CHARACTERISTIC_ID;
    break;
  
  }

  

  params.p_data = pData;
  params.p_len = &DataLen;

  res32 = sd_ble_gatts_hvx(m_conn_handle, &params);
  if (res32) {

    res = ERR_BLE_MESSAGE_SEND;
  }
  NRF_LOG_INFO("Serv_SendToHost: Characteristic %d, res = %d, res32 = %d", CharId,res, res32);
  return res;
}

#endif // NRF_MODULE_ENABLED(BLE_LBS)