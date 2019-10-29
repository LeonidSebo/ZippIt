#include "CmdHandler.h"
#include "AES_Security.h"
#include "Debug.h"
#include "sdk_common.h"

//#if NRF_MODULE_ENABLED(BLE_LBS)
//#include "ble_lbs.h"
#include "BLE_Services.h"
#include "bleCmdHandler.h"
//#include "AES_Security.h"
extern ble_main_service_t m_lbs;
//extern uint16_t m_conn_handle; /*BLE_CONN_HANDLE_INVALID*/

//extern uint16_t m_conn_handle;
//extern ble_main_service_t m_lbs;

#define ENCRIPTION_DISABLE 1

void CmdH_Command_Write(uint16_t conn_handle, ble_main_service_t *p_lbs,
    uint16_t CommandLen, uint8_t *pCommandEncr) {
  BLE_COMMAND Command;

#if ENCRIPTION_DISABLE == 0
  CmdH_Command_Handler((BLE_COMMAND *)pCommandEncr);
  return;
#endif

  RESULT res = ERR_NO;
  if (CommandLen != AES_BLOCK_SIZE_BYTE) {
    return;
  }

  res = AES_BlockDecript(CHAR_COMMAND, pCommandEncr, (uint8_t *)&Command);
  if (res != ERR_NO) {
    return;
  }
  // DEBUG
  //CmdH_Message_SendToHost(Command.CommandID, Command.Data, Command.DataLength - 1);
  // DEBUG
  CmdH_Command_Handler(&Command);
}

void CmdH_Command_Handler(BLE_COMMAND *pCommand) {
  // Debug_PrintHexArray("Get Command: ", pCommand, CommandLen);
  uint8_t answer[2];
  NRF_LOG_INFO("CmdH_Command_Handler. Command: %d. Length %d", pCommand->CommandID, pCommand->DataLength);

  switch (pCommand->CommandID) {
  case CMD_ID_GET_DEVICE_INFO:
    Cmd_GetDeviceInfo(pCommand);
    break;
  case CMD_ID_SET_CASE_STATE:
    Cmd_SetCaseState(pCommand);
    break;
  case CMD_ID_SET_LED_STATE:
    Cmd_SetLedState(pCommand);
    break;
  case CMD_ID_SET_MOTOR_TIMES:
    Cmd_SetMotorTimes(pCommand);
    break;
  case CMD_ID_SET_RTC_TIME:
    Cmd_SetRtcTime(pCommand);
    break;
  case CMD_ID_SET_BATTERY_ALARM_LEVEL:
    Cmd_SetBatteryAlarmLevel(pCommand);
    break;
  case CMD_ID_SET_LIGHT_ALARM_LEVEL:
    Cmd_SetLightAlarmLevel(pCommand);
    break;
  case CMD_ID_SET_NUMBER_RETRIES:
    Cmd_SetNumberRetries(pCommand);
    break;
  case CMD_ID_DEBUG:
    Debug_Func(pCommand);
    break;
  case CMD_ID_GET_RANDOM_NUMBERS:
    SetNewRandomNubers(true);
    break;
    //    case CMD_ID_FLASH_ERRASE:
    //      FlashErase(pCommand);
    //      break;
    //    case CMD_ID_FLASH_WRITE:
    //      FlashWrite(pCommand);
    //      break;
    //    case CMD_ID_FLASH_READ:
    //      FlashRead(pCommand);
    //      break;
  default:

    Message_Attention(ERR_BLE_CMD_ID);

    //      answer[0] = pCommand->ID;
    //      answer[1] = ERR_BLE_CMD_ID;
    //      Answer_Send(2, answer);

    break;
  }
  //SetCommand((DEVICE_COMMANDS)pCommand);
}

/* ================ COMMANDS ========================== */
RESULT Cmd_GetDeviceInfo(BLE_COMMAND *pCommand) {
  RESULT res = ERR_NO;
  DEVICE_INFO DeviceInfo = *(DEVICE_INFO *)pCommand->Data;
  if (pCommand->DataLength != sizeof(DEVICE_INFO)) {
    return Answer_OperationStatus(pCommand->CommandID, ERR_BLE_CMD_LEN);
  }

  res = bleGetDeviceInfo(&DeviceInfo);
  res = Answer_SendToHost(pCommand->CommandID, res, (uint8_t *)&DeviceInfo, sizeof(DEVICE_INFO));
  return res;
}
RESULT Cmd_SetCaseState(BLE_COMMAND *pCommand) {
  RESULT res = ERR_NO;
  CASE_STATE CaseState = *(CASE_STATE *)pCommand->Data;
  NRF_LOG_INFO("SetCaseState: State = %d", *(uint8_t *)&CaseState);
  if (pCommand->DataLength != sizeof(CASE_STATE)) {
    return Answer_OperationStatus(pCommand->CommandID, ERR_BLE_CMD_LEN);
  }

  if (CaseState > CASE_LOCK) {
    return Answer_OperationStatus(pCommand->CommandID, ERR_BLE_CMD_DATA);
  }
  res = bleSetCaseState(CaseState);
  res = Answer_OperationStatus(pCommand->CommandID, res);
  if (res != ERR_NO) {
    return res + 0;
  }
  return res;
}
RESULT Cmd_SetLedState(BLE_COMMAND *pCommand) {
  RESULT res = ERR_NO;
  LED_CONTROL LedControl = *(LED_CONTROL *)pCommand->Data;
  NRF_LOG_INFO("SetLedState: State = %d", *(uint8_t *)&LedControl);
  if (pCommand->DataLength != sizeof(LED_CONTROL)) {
    return Answer_OperationStatus(pCommand->CommandID, ERR_BLE_CMD_LEN);
  }
  res = bleSetLedState(LedControl); // Memory owerload;
  res = Answer_OperationStatus(pCommand->CommandID, res);
  return res;
}

RESULT Cmd_SetMotorTimes(BLE_COMMAND *pCommand) {
  RESULT res = ERR_NO;
  MOTOR_ACTIVE_TIME MotorTimes = *(MOTOR_ACTIVE_TIME *)pCommand->Data;
  NRF_LOG_INFO("SetMotorTimes");
  if (pCommand->DataLength != sizeof(MOTOR_ACTIVE_TIME)) {
    return Answer_OperationStatus(pCommand->CommandID, ERR_BLE_CMD_LEN);
  }
  //NRF_LOG_INFO("SetMotorTimes: CW = %d, CCW = %d", MotorTimes.MOTOR_CW_TIME_MS, MotorTimes.MOTOR_CCW_TIME_MS);
  res = bleSetMotorTimes(MotorTimes); 
  res = Answer_OperationStatus(pCommand->CommandID, res);
  return res;
}

RESULT Cmd_SetRtcTime(BLE_COMMAND *pCommand) {
  RESULT res = ERR_NO;
  uint32_t DateTime = *(uint32_t *)pCommand->Data;
  NRF_LOG_INFO("SetRtcTime.");
  if (pCommand->DataLength != sizeof(MOTOR_ACTIVE_TIME)) {
    return Answer_OperationStatus(pCommand->CommandID, ERR_BLE_CMD_LEN);
  }
  //res = bleSetRtcTime(DateTime); Memory owerflow
  res = Answer_OperationStatus(pCommand->CommandID, res);
  return res;
}

RESULT Cmd_SetBatteryAlarmLevel(BLE_COMMAND *pCommand) {
  RESULT res = ERR_NO;
  BATTERY_ALARM_LEVEL BatteryAlarmLevel = pCommand->Data[0];
  NRF_LOG_INFO("SetBatteryAlarmLevel: Level = %d", BatteryAlarmLevel);
  if (pCommand->DataLength != sizeof(BATTERY_ALARM_LEVEL)) {
    return Answer_OperationStatus(pCommand->CommandID, ERR_BLE_CMD_LEN);
  }
  //res = bleSetBatteryAlarmLevel(BatteryAlarmLevel);
  res = Answer_OperationStatus(pCommand->CommandID, res);
  return res;
}
RESULT Cmd_SetLightAlarmLevel(BLE_COMMAND *pCommand) {
  RESULT res = ERR_NO;
  LIGHT_ALARM_LEVEL LightAlarmLevel = pCommand->Data[0];
  NRF_LOG_INFO("SetLightAlarmLevel: Level = %d", LightAlarmLevel);
  if (pCommand->DataLength != sizeof(LIGHT_ALARM_LEVEL)) {
    return Answer_OperationStatus(pCommand->CommandID, ERR_BLE_CMD_LEN);
  }
  //res = bleSetLightAlarmLevel(LightAlarmLevel);
  res = Answer_OperationStatus(pCommand->CommandID, res);
  return res;
}
RESULT Cmd_SetNumberRetries(BLE_COMMAND *pCommand) {
  RESULT res = ERR_NO;
  NUMBER_RETRIES NumberRetries = pCommand->Data[0];
  NRF_LOG_INFO("SetNumberRetries: NumberRetries = %d", NumberRetries);
  if (pCommand->DataLength != sizeof(NUMBER_RETRIES)) {
    return Answer_OperationStatus(pCommand->CommandID, ERR_BLE_CMD_LEN);
  }
  //res = bleSetNumberRetriel(LightAlarmLevel);
  res = Answer_OperationStatus(pCommand->CommandID, res);
  return res;
}
/* ================ COMMANDS ========================== */

/* ================ ANSWERS ========================= */

/*
RESULT FlashErase(BLE_COMMAND *pCommand)      
{

    RESULT res = ERR_NO;
    uint32_t FlashAddress = pData[0] + (pData[1] << 8) + + (pData[2] << 16);
    uint8_t BlocksNo = pData[3];
    NRF_LOG_INFO("FlashErase: Address = 0x%x, BlocksNo = %d", FlashAddress, BlocksNo);
    
    return res;
}
RESULT FlashWrite(BLE_COMMAND *pCommand)      
{
    RESULT res = ERR_NO;
    uint32_t FlashAddress = pData[0] + (pData[1] << 8) + + (pData[2] << 16);
    uint8_t BlocksNo = pData[3];
    NRF_LOG_INFO("FlashWrite: Address = 0x%x, DataLen = %d", FlashAddress, BlocksNo);
    return res;
}
RESULT FlashRead(BLE_COMMAND *pCommand)      
{
    RESULT res = ERR_NO;
    uint32_t FlashAddress = pData[0] + (pData[1] << 8) + + (pData[2] << 16);
    uint8_t DataLen = pData[3];
    NRF_LOG_INFO("FlashRead: Address = 0x%x, DataLen = %d", FlashAddress, DataLen);
    return res;
}
*/

RESULT Answer_OperationStatus(BLE_COMMANDS_ID CommandID, RESULT OperationStatus) {
  return Answer_SendToHost(CommandID, OperationStatus, NULL, 0);
}

RESULT Answer_SendToHost(BLE_COMMANDS_ID CommandID, RESULT OperationStatus, uint8_t *pData, uint8_t DataLength) {
  RESULT res;

  BLE_ANSWER Answer;
  uint8_t CipherBlock16[AES_BLOCK_SIZE_BYTE];

  Answer.AnswerID = CommandID | ANSWER_ID_FLAG;
  Answer.AnswerLength = DataLength + 1;
  Answer.OperationStatus = OperationStatus;
  if (DataLength) {
    memcpy(Answer.Data, pData, DataLength);
  }
  res = AES_BlockEncript(CHAR_ANSWER, (uint8_t *)&Answer, DataLength + BLE_BLOCK_HEADER_SIZE_BYTE, CipherBlock16);
  RESULT_CHECK_WITH_LOG(res);
  res = Serv_SendToHost(CHAR_ANSWER, (uint8_t *)&CipherBlock16, AES_BLOCK_SIZE_BYTE);
  RESULT_CHECK_WITH_LOG(res);
  AES_SetNewCharRandomVal(CHAR_COMMAND);
  AES_SetNewCharRandomVal(CHAR_ANSWER);
  return res;
}
/* ================ ANSWERS ========================== */
/* ================ MESSAGES ========================= */
RESULT Message_DeviceStatus(DEVICE_STATUS DeviceStatus) {
  NRF_LOG_INFO("Message_DeviceStatus = %08x", *(uint32_t *)&DeviceStatus);
  return CmdH_Message_SendToHost(MSG_DEVICE_STATUS_CHANGED, (uint8_t *)&DeviceStatus, sizeof(DEVICE_STATUS));
}

RESULT Message_DeviceError(RESULT Result) {
  return CmdH_Message_SendToHost(MSG_DEVICE_ERROR, (uint8_t *)&Result, 1);
}

RESULT Message_Attention(RESULT Result) {
  return CmdH_Message_SendToHost(MSG_ATTENTION, (uint8_t *)&Result, 1);
}

RESULT Message_Byte_1(BLE_MESSAGE_ID MessageTD, uint8_t Message) {

  return CmdH_Message_SendToHost(MessageTD, (uint8_t *)&Message, 1);
}

//RESULT Message_DeviceStatus(DEVICE_STATUS DeviceStatus)
//{
//  return CmdH_Message_SendToHost(MSG_BLE_CMD_ID_UNKNOWN, (uint8_t*)&Message, sizeof(DEVICE_STATUS));
//}

RESULT CmdH_Message_SendToHost(BLE_MESSAGE_ID MessageID, uint8_t *pData, uint8_t DataLength) {
  RESULT res;

  BLE_MESSAGE Message;
  uint8_t CipherBlock16[AES_BLOCK_SIZE_BYTE];

  Message.MessageID = MessageID;
  Message.DataLength = DataLength;
  memcpy(Message.Data, pData, DataLength);
  res = AES_BlockEncript(CHAR_MESSAGE, (uint8_t *)&Message, DataLength + BLE_BLOCK_HEADER_SIZE_BYTE, CipherBlock16);
  RESULT_CHECK_WITH_LOG(res);
  res = Serv_SendToHost(CHAR_MESSAGE, (uint8_t *)&CipherBlock16, AES_BLOCK_SIZE_BYTE);
  RESULT_CHECK_WITH_LOG(res);
  AES_SetNewCharRandomVal(CHAR_MESSAGE);
  //Debug_PrintHexArray("NewRandNumber = ", pData, DataLength);
  return ERR_NO;
}

//RESULT Message_AES_CountersChanged(uint8_t *pCounters)
//{
//  return ERR_NO;
//}

/* ================ MESSAGES ========================= */

RESULT CmdH_DeviceConnected() {
  //return SetNewRandomNubers(false);
  AES_SetRandomNumberDefault();
  return ERR_NO;
}

RESULT CmdH_DeviceDisconnected() {
  return ERR_NO;
}

RESULT SetNewRandomNubers(bool AnswerChar) {
  RESULT res;

  uint8_t pNewRandomNumbers[AES_BLOCK_RANDOM_NO_SIZE_BYTE * CHARACTERISTICS_NO];

  res = AES_RandFillArray(pNewRandomNumbers, AES_BLOCK_RANDOM_NO_SIZE_BYTE * CHARACTERISTICS_NO);
  RESULT_CHECK(res);

  //Debug_PrintHexArray("pNewRandNumber = ", pNewRandNumber, 9);

  AES_SetRandomNumberDefault();
  if (AnswerChar) {
    res = Answer_SendToHost(CMD_ID_GET_RANDOM_NUMBERS, ERR_NO, pNewRandomNumbers, AES_BLOCK_RANDOM_NO_SIZE_BYTE * CHARACTERISTICS_NO);
  } else {
    //NRF_LOG_INFO("CmdH_Message_SendToHost.");
    res = CmdH_Message_SendToHost(MSG_NEW_RANDOM_NUMBERS, pNewRandomNumbers, AES_BLOCK_RANDOM_NO_SIZE_BYTE * CHARACTERISTICS_NO);
  }
  if (res == ERR_NO) {
    AES_SetNewRandomNumbers(pNewRandomNumbers);
  }
  return res;
}
//bool IsDeviceConnected()
//{
//    return m_conn_handle == BLE_CONN_HANDLE_INVALID;
//}
//RESULT Message_DeviceStatus(DEVICE_STATUS DeviceStatus)
//{
//  RESULT res;
//
//  AES_BLOCK_DATA ClearBlock;
//  uint8_t CipherBlock16[AES_BLOCK_SIZE_BYTE];
//
//  ClearBlock.ID = MSG_DEVICE_STATUS_CHANGED;
//  ClearBlock.DataLength = sizeof(DEVICE_STATUS);
//  memcpy(ClearBlock.Data, &DeviceStatus, sizeof(DEVICE_STATUS));
//
//  res =  AES_BlockEncript(CHAR_MESSAGE, &ClearBlock, CipherBlock16);
//  RESULT_CHECK_WITH_LOG(res);
//  res = Message_Send((uint8_t*)&CipherBlock16, AES_BLOCK_SIZE_BYTE);
//  RESULT_CHECK_WITH_LOG(res);
//  AES_ChangeCounter(CHAR_MESSAGE);
//  return ERR_NO;
//}

//RESULT Answer_Send(uint16_t AnswerLen, uint8_t* pAnswer)
//{
//    uint32_t res32;
//    RESULT res;
//    ble_gatts_hvx_params_t params;
//    if(!IsDeviceConnected())
//    {
//      return ERR_BLE_DEVICE_NOT_CONNECTED;
//    }
//    memset(&params, 0, sizeof(params));
//    params.type   = BLE_GATT_HVX_NOTIFICATION;
//    params.handle = m_lbs.hAnswerChar.value_handle;;
//    params.p_data = pAnswer;
//    params.p_len  = &AnswerLen;
//
//    res32 = sd_ble_gatts_hvx(m_conn_handle, &params);
//    if(res32)
//    {
//
//        res = ERR_BLE_MESSAGE_SEND;
//    }
//    NRF_LOG_INFO("Answer_Send: res = %d, res32 = %d", res, res32);
//    return ERR_NO;
//}

//RESULT Message_Send(uint8_t *pData, uint8_t Length)
//{
//  uint32_t res32;
//  if(!IsDeviceConnected())
//  {
//    return ERR_BLE_DEVICE_NOT_CONNECTED;
//  }
//  ble_gatts_hvx_params_t params;
//  memset(&params, 0, sizeof(params));
//  params.type   = BLE_GATT_HVX_NOTIFICATION;
//  params.handle = m_lbs.hDeviceStatusChar.value_handle;
//  params.p_data = pData;
//  uint16_t Len = Length;
//  params.p_len  = &Len;
//
//  res32 = sd_ble_gatts_hvx(m_conn_handle, &params);
//  if(res32)
//  {
//    return ERR_BLE_MESSAGE_SEND;
//  }
//  return ERR_NO;
//}