#include "CmdHandler.h"
#include "AES_Security.h"
#include "Debug.h"
#include "sdk_common.h"

#include "BLE_Services.h"
#include "bleCmdHandler.h"

extern ble_main_service_t m_lbs;

extern uint8_t gCommand[CHAR_COMMAND_SIZE];      /* Write */
extern uint8_t gAnswer[CHAR_ANSWER_SIZE];        /* Indication */
extern uint8_t gMessage[CHAR_MESSAGE_SIZE];      /* Indication */
extern uint8_t gFlashData[CHAR_FLASH_DATA_SIZE]; /* Notification */

//uint8_t gRetries; /* value from FLASh */
uint16_t gRetriesCmdCounter;
int16_t gRetriesAlertTimer;
bool gRetriesStopDevice;
NUMBER_RETRIES gNumberRetries;//; = COUNT_ATTENTION_EVENT_MAX_VALUE;

#define TIMER_TICK_sec 16                     /*sec*/
#define RETRIES_ALERT_DEVICE_STOP_TIME_MIN 30 /*min*/
#define RETRIES_ALERT_DEVICE_STOP_TIME_16secTick (RETRIES_ALERT_DEVICE_STOP_TIME_MIN * 60 / TIMER_TICK_sec)

#define CHAR_COMMAND_ENCRIPTION_DISABLE 0
//-------------------------------------------------------//
#define COUNT_ATTENTION_EVENT_MAX_VALUE 10

//-------------------------------------------------------//
bool gCmdGetRandomNumberWait;
#define COUNT_CMD_GET_RANDOM_NUMBER_NOT_FIRST_MAX_VALUE COUNT_ATTENTION_EVENT_MAX_VALUE
bool gCmdGetRandomNumberNotFirstCount;
//-------------------------------------------------------//
#define COUNT_CMD_ID_ERROR_MAX_VALUE COUNT_ATTENTION_EVENT_MAX_VALUE
bool gCmd_ID_ErrorCount;
//-------------------------------------------------------//

void CmdH_Init() 
{
  gNumberRetries = bleGetNumberRetries();
}

void CmdH_Command_Write(uint16_t conn_handle, ble_main_service_t *p_lbs, uint16_t CommandLen, uint8_t *pCommandEncr) {
  BLE_COMMAND Command;
  RESULT res = ERR_NO;

#if CHAR_COMMAND_ENCRIPTION_DISABLE
  CmdH_Command_Handler((BLE_COMMAND *)pCommandEncr);
  return;
#endif
  gRetriesCmdCounter++;
  if (CommandLen != AES_BLOCK_SIZE_BYTE) {
    return;
  }

  res = AES_BlockDecript(CHAR_COMMAND, pCommandEncr, (uint8_t *)&Command);
  if (res != ERR_NO) {
    return;
  }
  // DEBUG
  // CmdH_Message_SendToHost(Command.CommandID, Command.Data, Command.DataLength - 1);
  // DEBUG
  CmdH_Command_Handler(&Command);
}

void CmdH_Command_Handler(BLE_COMMAND *pCommand) {
  RESULT res = ERR_NO;
  NRF_LOG_INFO("CmdH_Command_Handler. Command: %d. Length %d", pCommand->CommandID, pCommand->DataLength);

#if !CHAR_COMMAND_ENCRIPTION_DISABLE
//  if (gCmdGetRandomNumberWait) {
//    if (pCommand->CommandID != CMD_ID_GET_RANDOM_NUMBERS) {
//      /* Command Get Random Number Not First */
//      gCmdGetRandomNumberNotFirstCount++;
//      if (gCmdGetRandomNumberNotFirstCount <= COUNT_CMD_GET_RANDOM_NUMBER_NOT_FIRST_MAX_VALUE) {
//        return;
//      }
//      gCmdGetRandomNumberNotFirstCount = 0;
//      res = ERR_CMD_GET_RANDOM_NUMBERS_NOT_FIRST;
//      goto ExitFunc;
//    }
//  }
#endif

  Debug_PrintHexArray("Command: ", (uint8_t *)pCommand, pCommand->DataLength + 2);
  switch (pCommand->CommandID) {
  case CMD_ID_GET_DEVICE_INFO:
    res = Cmd_GetDeviceInfo(pCommand);
    return;
  case CMD_ID_SET_CASE_STATE:
    res = Cmd_SetCaseState(pCommand);
    return;
  case CMD_ID_GET_DEVICE_STATUS:
    res = Cmd_GetDeviceStatus(pCommand);
    return;
  case CMD_ID_SET_LED_STATE:
    res = Cmd_SetLedState(pCommand);
    return;
  case CMD_ID_SET_MOTOR_TIMES:
    res = Cmd_SetMotorTimes(pCommand);
    return;
  case CMD_ID_SET_RTC_TIME:
    res = Cmd_SetRtcTime(pCommand);
    return;
  case CMD_ID_SET_BATTERY_ALARM_LEVEL:
    res = Cmd_SetBatteryAlarmLevel(pCommand);
    return;
  case CMD_ID_SET_LIGHT_ALARM_LEVEL:
    res = Cmd_SetLightAlarmLevel(pCommand);
    return;
  case CMD_ID_SET_NUMBER_RETRIES:
    res = Cmd_SetNumberRetries(pCommand);
    return;
  case CMD_ID_GET_BATTERY_CHARGING_LEVEL:
    res = Cmd_GetBatteryChargingLevel(pCommand);
    return;
  case CMD_ID_FLASH_LOG_ERRASE:
    res = Cmd_FlashLogErase(pCommand);
    return;
  case CMD_ID_FLASH_LOG_READ:
    res = Cmd_GetFlashLog(pCommand);
    return;
  case CMD_ID_GET_RANDOM_NUMBERS:
    res = Cmd_SetNewRandomNubers(true);
    if (res == ERR_NO) {
      gCmdGetRandomNumberWait = false;
    }
    return;

  case CMD_ID_SET_HARDWARE_VERSION:
    res = Cmd_SetHardwareVersion(pCommand);
    return;

  //------------------------------//
  case CMD_ID_DEBUG:
    Debug_Func(pCommand);
    return;

  default: /* Command ID Error*/

    gCmd_ID_ErrorCount++;
    if (gCmd_ID_ErrorCount <= COUNT_CMD_ID_ERROR_MAX_VALUE) {
      return;
    }
    gCmd_ID_ErrorCount = 0;
    res = ERR_BLE_CMD_ID;
    goto ExitFunc;
  }
  gCmd_ID_ErrorCount = 0;

ExitFunc:
  Message_Attention(res);
}

/* ================ COMMANDS ========================== */
RESULT Cmd_GetDeviceInfo(BLE_COMMAND *pCommand) {
  RESULT res = ERR_NO;
  DEVICE_INFO DeviceInfo; // = *(DEVICE_INFO *)pCommand->Data;
  if (pCommand->DataLength != 0) {
    return Answer_OperationStatus(pCommand->CommandID, ERR_BLE_CMD_LEN);
  }

  res = bleGetDeviceInfo(&DeviceInfo);
  NRF_LOG_INFO("Cmd_GetDeviceInfo: HW VER %d.%d, SW VER %d.%d",
      DeviceInfo.HW_VERSION_MAJOR,
      DeviceInfo.HW_VERSION_MINOR,
      DeviceInfo.SW_VERSION_MAJOR,
      DeviceInfo.SW_VERSION_MINOR);

  res = Answer_SendToHost(pCommand->CommandID, res, (uint8_t *)&DeviceInfo, sizeof(DEVICE_INFO));
  return res;
}

RESULT Cmd_GetDeviceStatus(BLE_COMMAND *pCommand) {
  RESULT res = ERR_NO;
  DEVICE_STATUS DeviceStatus;
  if (pCommand->DataLength != 0) {
    return Answer_OperationStatus(pCommand->CommandID, ERR_BLE_CMD_LEN);
  }

  res = bleGetDeviceStatus(&DeviceStatus);
  NRF_LOG_INFO("Cmd_GetDeviceStatus:  0x%08x", *(uint32_t *)&DeviceStatus);

  res = Answer_SendToHost(pCommand->CommandID, res, (uint8_t *)&DeviceStatus, sizeof(DEVICE_STATUS));
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
  // NRF_LOG_INFO("SetMotorTimes: CW = %d, CCW = %d", MotorTimes.MOTOR_CW_TIME_MS, MotorTimes.MOTOR_CCW_TIME_MS);
  res = bleSetMotorTimes(MotorTimes);
  res = Answer_OperationStatus(pCommand->CommandID, res);
  return res;
}

RESULT Cmd_SetRtcTime(BLE_COMMAND *pCommand) {
  RESULT res = ERR_NO;
  uint32_t DateTime = *(uint32_t *)pCommand->Data;
  NRF_LOG_INFO("SetRtcTime.");
  if (pCommand->DataLength != sizeof(RTC_VALUE)) {
    return Answer_OperationStatus(pCommand->CommandID, ERR_BLE_CMD_LEN);
  }
  res = bleSetRtcTime(DateTime);
  res = Answer_OperationStatus(pCommand->CommandID, res);
  return res;
}

RESULT Cmd_SetBatteryAlarmLevel(BLE_COMMAND *pCommand) {
  RESULT res = ERR_NO;
  BATTERY_ALARM_LEVEL BatteryAlarmLevel;
  memcpy(&BatteryAlarmLevel, pCommand->Data, sizeof(BATTERY_ALARM_LEVEL));
  NRF_LOG_INFO("SetBatteryAlarmLevel: Level = %d", BatteryAlarmLevel);
  if (pCommand->DataLength != sizeof(BATTERY_ALARM_LEVEL)) {
    return Answer_OperationStatus(pCommand->CommandID, ERR_BLE_CMD_LEN);
  }
  res = bleSetBatteryAlarmLevel(BatteryAlarmLevel);
  res = Answer_OperationStatus(pCommand->CommandID, res);
  return res;
}

RESULT Cmd_SetLightAlarmLevel(BLE_COMMAND *pCommand) {
  RESULT res = ERR_NO;
  LIGHT_ALARM_LEVEL LightAlarmLevel;
  memcpy(&LightAlarmLevel, pCommand->Data, sizeof(LIGHT_ALARM_LEVEL));
  NRF_LOG_INFO("SetLightAlarmLevel: Level = %d", LightAlarmLevel);
  if (pCommand->DataLength != sizeof(LIGHT_ALARM_LEVEL)) {
    return Answer_OperationStatus(pCommand->CommandID, ERR_BLE_CMD_LEN);
  }
  res = bleSetLightAlarmLevel(LightAlarmLevel);
  res = Answer_OperationStatus(pCommand->CommandID, res);
  return res;
}

RESULT Cmd_SetNumberRetries(BLE_COMMAND *pCommand) {
  RESULT res = ERR_NO;

  if (pCommand->DataLength != sizeof(NUMBER_RETRIES)) {
    return Answer_OperationStatus(pCommand->CommandID, ERR_BLE_CMD_LEN);
  }

  gNumberRetries = pCommand->Data[0];
  NRF_LOG_INFO("SetNumberRetries: NumberRetries = %d", gNumberRetries);
  res = bleSetNumberRetries(gNumberRetries);
  res = Answer_OperationStatus(pCommand->CommandID, res);
  return res;
}

RESULT Cmd_GetBatteryChargingLevel(BLE_COMMAND *pCommand) {
  uint16_t BatteryChargingLevel;
  RESULT res;
  res = bleGetBatteryVoltage(&BatteryChargingLevel);
  NRF_LOG_INFO("Cmd_GetBatteryChargingLevel: BatteryChargingLevel = %d", BatteryChargingLevel);
  res = Answer_BatteryChargingLevel(pCommand->CommandID, res, BatteryChargingLevel);
  return res;
}

RESULT Cmd_SetHardwareVersion(BLE_COMMAND *pCommand) {
  HARDWARE_VERSION HardwareVersion;
  RESULT res;
  // memcpy(&HardwareVersion)
  res = bleSetHardwareVersion(*(HARDWARE_VERSION *)pCommand->Data);
  res = Answer_SetHardwareVersion(pCommand->CommandID, res, HardwareVersion);
  return res;
}

RESULT Cmd_FlashLogErase(BLE_COMMAND *pCommand) {
  RESULT res;
  res = bleFlashLogErase();
  res = Answer_OperationStatus(pCommand->CommandID, res);
  return res;
}

RESULT Cmd_GetFlashLog(BLE_COMMAND *pCommand) {
  RESULT res;
  uint32_t Offset;
  uint32_t DataLength;

  Offset = pCommand->Data[0] + (pCommand->Data[1] << 8) + (pCommand->Data[2] << 16);
  DataLength = pCommand->Data[3] + (pCommand->Data[4] << 8);

  res = Answer_OperationStatus(pCommand->CommandID, res);
  res = Flash_LogRead(Offset, DataLength);
  return res;
}

/* ================ ANSWERS ========================= */

RESULT Answer_OperationStatus(BLE_COMMANDS_ID CommandID, RESULT OperationStatus) {
  return Answer_SendToHost(CommandID, OperationStatus, NULL, 0);
}

RESULT Answer_BatteryChargingLevel(BLE_COMMANDS_ID CommandID, RESULT OperationStatus, uint16_t BatteryChargingLevel) {
  if (OperationStatus == ERR_NO) {
    return Answer_SendToHost(
        CommandID, OperationStatus, (uint8_t *)&BatteryChargingLevel, sizeof(BatteryChargingLevel));
  } else {
    return Answer_OperationStatus(CommandID, OperationStatus);
  }
}

RESULT Answer_SetHardwareVersion(BLE_COMMANDS_ID CommandID, RESULT OperationStatus, HARDWARE_VERSION HardwareVersion) {
  if (OperationStatus == ERR_NO) {
    return Answer_SendToHost(CommandID, OperationStatus, (uint8_t *)&HardwareVersion, sizeof(HARDWARE_VERSION));
  } else {
    return Answer_SendToHost(CommandID, OperationStatus, NULL, 0);
  }
}

RESULT Answer_SendToHost(BLE_COMMANDS_ID CommandID, RESULT OperationStatus, uint8_t *pData, uint8_t DataLength) {
  RESULT res;

  BLE_ANSWER Answer;
  uint8_t CipherBlock16[AES_BLOCK_SIZE_BYTE];

  Answer.AnswerID = CommandID | ANSWER_ID_FLAG;
  Answer.AnswerLength = DataLength + BLE_BLOCK_OPERATION_STATUS_SIZE_BYTE;
  Answer.OperationStatus = OperationStatus;
  if (DataLength) {
    memcpy(Answer.Data, pData, DataLength);
  }

  Debug_PrintHexArray("Answer: ", (uint8_t *)&Answer, DataLength + BLE_BLOCK_ANSWER_HEADER_SIZE_BYTE);
  res = AES_BlockEncript(CHAR_ANSWER, (uint8_t *)&Answer, DataLength + BLE_BLOCK_ANSWER_HEADER_SIZE_BYTE, CipherBlock16);
  RESULT_CHECK_WITH_LOG(res);
  res = Serv_SendToHost(CHAR_ANSWER, (uint8_t *)&CipherBlock16, AES_BLOCK_SIZE_BYTE);
  RESULT_CHECK_WITH_LOG(res);
  AES_SetNewCharRandomVal(CHAR_COMMAND);
  AES_SetNewCharRandomVal(CHAR_ANSWER);
  return res;
}

/* ================ MESSAGES ========================= */
RESULT Message_DeviceStatus(DEVICE_STATUS_EVENT DeviceStatus) {
  NRF_LOG_INFO("Message_DeviceStatus = %08x", *(uint32_t *)&DeviceStatus);
  return Message_SendToHost(MSG_DEVICE_STATUS_CHANGED, (uint8_t *)&DeviceStatus, sizeof(DEVICE_STATUS_EVENT));
}

RESULT Message_DeviceError(RESULT Result) {
  return Message_SendToHost(MSG_DEVICE_ERROR, (uint8_t *)&Result, 1);
}

RESULT Message_Attention(RESULT Result) {
  return Message_SendToHost(MSG_ATTENTION, (uint8_t *)&Result, 1);
}

RESULT Message_Byte_1(BLE_MESSAGE_ID MessageTD, uint8_t Message) {
  return Message_SendToHost(MessageTD, (uint8_t *)&Message, 1);
}

RESULT Message_SendToHost(BLE_MESSAGE_ID MessageID, uint8_t *pData, uint8_t DataLength) {
  RESULT res;

  BLE_MESSAGE Message;
  uint8_t CipherBlock16[AES_BLOCK_SIZE_BYTE];

  Message.MessageID = MessageID;
  Message.DataLength = DataLength;
  memcpy(Message.Data, pData, DataLength);
  res = AES_BlockEncript(CHAR_MESSAGE, (uint8_t *)&Message, DataLength + BLE_BLOCK_MESSAGE_HEADER_SIZE_BYTE, CipherBlock16);
  RESULT_CHECK_WITH_LOG(res);
  res = Serv_SendToHost(CHAR_MESSAGE, (uint8_t *)&CipherBlock16, AES_BLOCK_SIZE_BYTE);
  RESULT_CHECK_WITH_LOG(res);
  AES_SetNewCharRandomVal(CHAR_MESSAGE);
  return ERR_NO;
}


/* ================ MESSAGES ========================= */

RESULT CmdH_DeviceConnected() {
  AES_SetRandomNumberDefault();
  gCmdGetRandomNumberWait = true;
  //gCmdGetRandomNumberNotFirstCount = 0;
  gCmd_ID_ErrorCount = 0;
  gRetriesCmdCounter = 0;
  gRetriesAlertTimer = 0;
  gRetriesStopDevice = false;
  logEventStorageReq(LOG_EVENT_DEVICE_CONNECTED, 0, 0, 0);
  return ERR_NO;
}

RESULT CmdH_DeviceDisconnected() {
  logEventStorageReq(LOG_EVENT_DEVICE_DISCONNECTED, 0, 0, 0);
  return ERR_NO;
}

RESULT Cmd_SetNewRandomNubers(bool AnswerChar) {
  RESULT res;

  uint8_t pNewRandomNumbers[AES_BLOCK_RANDOM_NO_SIZE_BYTE * CHARACTERISTICS_NO];

  res = AES_RandFillArray(pNewRandomNumbers, AES_BLOCK_RANDOM_NO_SIZE_BYTE * CHARACTERISTICS_NO);
  RESULT_CHECK(res);

  // Debug_PrintHexArray("pNewRandNumber = ", pNewRandNumber, 9);

  AES_SetRandomNumberDefault();
  if (AnswerChar) {
    res = Answer_SendToHost(
        CMD_ID_GET_RANDOM_NUMBERS, ERR_NO, pNewRandomNumbers, AES_BLOCK_RANDOM_NO_SIZE_BYTE * CHARACTERISTICS_NO);
  } else {
    res = Message_SendToHost(
        MSG_NEW_RANDOM_NUMBERS, pNewRandomNumbers, AES_BLOCK_RANDOM_NO_SIZE_BYTE * CHARACTERISTICS_NO);
  }
  if (res == ERR_NO) {
    AES_SetNewRandomNumbers(pNewRandomNumbers);
  }
  return res;
}

/* ================ FLASH_DATA ========================= */
RESULT Flash_LogRead(uint32_t Offset, uint32_t DataLength) {
  RESULT res;
  uint16_t DataLengthRet;
  uint8_t Data[CHAR_FLASH_DATA_SIZE];
  //res = bleFlashLogRead(Offset, DataLength, Data + BLE_FLASH_DATA_HEADER_LEN, &DataLengthRet);
  res = Serv_SendToHost(CHAR_FLASH_DATA, Data, DataLength + BLE_FLASH_DATA_HEADER_LEN);
  return res;
}

RESULT FlashData_SendToHost(BLE_FLASH_DATA_ID DataID, RESULT OperationStatus, uint8_t *pData, uint8_t DataLength) {
  RESULT res;
  uint32_t i = 0;
  uint32_t DataCount = DataLength;
  uint32_t CurrentBlockLength;
  uint32_t Offset = 0;

  BLE_FLASH_DATA *pFlashData = (BLE_FLASH_DATA *)gFlashData;
  pFlashData->DataID = DataID;
  pFlashData->DataLength = DataLength + BLE_FLASH_DATA_OPERATION_STATUS_LEN;
  pFlashData->OperationStatus = OperationStatus;

  uint8_t CipherBlock16[AES_BLOCK_SIZE_BYTE];

  while (true) {
    CurrentBlockLength = (DataCount < AES_BLOCK_SIZE_BYTE) ? DataCount : AES_BLOCK_SIZE_BYTE;
    Offset = AES_BLOCK_SIZE_BYTE * i++;
    res = AES_BlockEncript(CHAR_FLASH_DATA, (uint8_t *)pData + Offset, CurrentBlockLength, gFlashData + Offset);
    RESULT_CHECK_WITH_LOG(res);
    /* AES_SetNewCharRandomVal(CHAR_FLASH_DATA); */
  }

  res = Serv_SendToHost(CHAR_FLASH_DATA, (uint8_t *)gFlashData, DataLength);
  RESULT_CHECK_WITH_LOG(res);
  AES_SetNewCharRandomVal(CHAR_FLASH_DATA);
  return ERR_NO;
}
/*============== RETRIES ==============*/
void TickRetries_16s() {
  if (gRetriesStopDevice) {
    if (gRetriesAlertTimer > 0) {
      gRetriesAlertTimer--;
    }
    else
    {
      gRetriesAlertTimer = 0;
      gRetriesStopDevice = false;
      wake();
    }
    return;
  }

  if (gRetriesCmdCounter > gNumberRetries) {
    gRetriesStopDevice = true;
    gRetriesAlertTimer = RETRIES_ALERT_DEVICE_STOP_TIME_16secTick;
    Message_Byte_1(MSG_DISCOVERED_RETRIES_NO, gRetriesCmdCounter);
    sleep();
  }
  gRetriesCmdCounter = 0;
}