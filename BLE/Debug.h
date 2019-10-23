
#ifndef APP_DEBUG_H__
#define APP_DEBUG_H__

#include "BLE_Init.h"
#include <stdbool.h>
#include <stdint.h>

void Debug_PrintHexArray(char const *p_label, char const *p_text, size_t len);
RESULT Debug_EncriptDecript();
void Debug_CheckEncDec();
void Debug_Func(BLE_COMMAND *pCommand);
void Debug_GetEncrCommand();
void Debug_MessageSend();
RESULT Message_DeviceError(RESULT Result);
#endif // APP_DEBUG_H__