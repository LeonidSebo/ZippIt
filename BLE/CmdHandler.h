
#ifndef CMD_HANDLER_H__
#define CMD_HANDLER_H__

#include "BLE_Def.h"
#include "BLE_Services.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
RESULT CmdH_DeviceConnected();
RESULT CmdH_DeviceDisconnected();
void CmdH_Command_Write(uint16_t conn_handle, ble_main_service_t *p_lbs, uint16_t CommandLen, uint8_t *pCommand);
void CmdH_Command_Handler(BLE_COMMAND *pCommand);
//bool IsDeviceConnected();
RESULT SetNewRandomNubers(bool AnswerChar);

/* COMMAND Characteristic */
RESULT Cmd_SetCaseState(BLE_COMMAND *pCommand);
RESULT Cmd_GetDeviceInfo(BLE_COMMAND *pCommand);
RESULT Cmd_SetLedState(BLE_COMMAND *pCommand);
RESULT Cmd_SetMotorTimes(BLE_COMMAND *pCommand);
RESULT Cmd_SetRtcTime(BLE_COMMAND *pCommand);
RESULT Cmd_SetBatteryAlarmLevel(BLE_COMMAND *pCommand);
;
RESULT Cmd_SetLightAlarmLevel(BLE_COMMAND *pCommand);
RESULT Cmd_SetNumberRetries(BLE_COMMAND *pCommand);
RESULT Cmd_FlashErase(BLE_COMMAND *pCommand);
RESULT Cmd_FlashWrite(BLE_COMMAND *pCommand);
RESULT Cmd_FlashRead(BLE_COMMAND *pCommand);
//RESULT Cmd_DeviceConnected();

/* ANSWER Characteristic */
RESULT Answer_SendToHost(BLE_COMMANDS_ID CommandID, RESULT OperationStatus, uint8_t *pData, uint8_t DataLength);
RESULT Answer_OperationStatus(BLE_COMMANDS_ID CommandID, RESULT OperationStatus);

/* MESSAGE Characteristic */
RESULT Message_DeviceStatus(DEVICE_STATUS DeviceStatus);
//RESULT Message_AES_CountersChanged(uint8_t *pCounters);
RESULT Message_Attention(RESULT Result);
RESULT Message_Byte_1(BLE_MESSAGE_ID MessageTD, uint8_t Message);
RESULT CmdH_Message_SendToHost(BLE_MESSAGE_ID MessageID, uint8_t *pData, uint8_t DataLength);

#endif // CMD_HANDLER_H__