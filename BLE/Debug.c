#include "Debug.h"
#include "AES_Security.h"
#include "CmdHandler.h"
#include "bleCmdHandler.h"

void hex_text_print(char const *p_label, char const *p_text, size_t len) {
  NRF_LOG_RAW_INFO("---- %s (len: %u) ----\r\n", p_label, len);
  NRF_LOG_FLUSH();

  // Handle partial line (left)
  for (size_t i = 0; i < len; i++) {
    if (((i & 0xF) == 0) && (i > 0)) {
      NRF_LOG_RAW_INFO("\r\n");
      NRF_LOG_FLUSH();
    }

    NRF_LOG_RAW_INFO("%02x ", p_text[i]);
    NRF_LOG_FLUSH();
  }
  NRF_LOG_RAW_INFO("\r\n");
  NRF_LOG_RAW_INFO("---- %s end ----\r\n\r\n", p_label);
  NRF_LOG_FLUSH();
}

void Debug_PrintHexArray(char const *p_label, char const *p_text, size_t len) {
  //NRF_LOG_INFO

  NRF_LOG_RAW_INFO("%s (len %u): ", p_label, len);
  NRF_LOG_FLUSH();

  // Handle partial line (left)
  for (size_t i = 0; i < len; i++) {
    if (((i & 0xF) == 0) && (i > 0)) {
      NRF_LOG_RAW_INFO("\r\n");
      NRF_LOG_FLUSH();
    }

    NRF_LOG_RAW_INFO("%02x ", p_text[i]);
    NRF_LOG_FLUSH();
  }
  NRF_LOG_RAW_INFO("\r\n");
}

RESULT Debug_EncriptDecript() {
  RESULT res = ERR_NO;
  //  return ERR_NO;
  nrf_ecb_hal_data_t EsbParam;
  ret_code_t err_code;

  uint8_t Key[] = {
      0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
      0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};

  uint8_t Data_0[] = {
      0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
      0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};

  uint8_t Data_0__[] = {
      0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
      0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};

  memset(&EsbParam, 0, sizeof(nrf_ecb_hal_data_t));
  memcpy(EsbParam.key, Key, SOC_ECB_KEY_LENGTH);
  memcpy(EsbParam.cleartext, Data_0, SOC_ECB_CLEARTEXT_LENGTH);
  //memcpy(EsbParam.ciphertext, Data_0, SOC_ECB_CIPHERTEXT_LENGTH);

  NRF_LOG_RAW_INFO("===============================================\r\n");
  Debug_PrintHexArray(".key", EsbParam.key, sizeof(EsbParam.key));
  Debug_PrintHexArray(".cleartext", EsbParam.cleartext, sizeof(EsbParam.cleartext));
  Debug_PrintHexArray(".ciphertext", EsbParam.ciphertext, sizeof(EsbParam.ciphertext));

  err_code = sd_ecb_block_encrypt(&EsbParam);
  NRF_LOG_RAW_INFO("err_code %d\r\n", err_code);

  NRF_LOG_RAW_INFO("-----------------------------------------------\r\n");
  Debug_PrintHexArray(".key", EsbParam.key, sizeof(EsbParam.key));
  Debug_PrintHexArray(".cleartext", EsbParam.cleartext, sizeof(EsbParam.cleartext));
  Debug_PrintHexArray(".ciphertext", EsbParam.ciphertext, sizeof(EsbParam.ciphertext));

  //  memset(&EsbParam, 0, sizeof(nrf_ecb_hal_data_t));
  memcpy(EsbParam.key, Key, SOC_ECB_KEY_LENGTH);
  memcpy(EsbParam.cleartext, EsbParam.ciphertext, SOC_ECB_CLEARTEXT_LENGTH);
  memset(EsbParam.ciphertext, 0, SOC_ECB_CIPHERTEXT_LENGTH);
  //  memcpy(EsbParam.ciphertext, Data_0, SOC_ECB_CIPHERTEXT_LENGTH);
  NRF_LOG_RAW_INFO("-----------------------------------------------\r\n");
  Debug_PrintHexArray(".key", EsbParam.key, sizeof(EsbParam.key));
  Debug_PrintHexArray(".cleartext", EsbParam.cleartext, sizeof(EsbParam.cleartext));
  Debug_PrintHexArray(".ciphertext", EsbParam.ciphertext, sizeof(EsbParam.ciphertext));

  err_code = sd_ecb_block_encrypt(&EsbParam);
  NRF_LOG_RAW_INFO("err_code %d\r\n", err_code);
  NRF_LOG_RAW_INFO("-----------------------------------------------\r\n");
  Debug_PrintHexArray(".key", EsbParam.key, sizeof(EsbParam.key));
  Debug_PrintHexArray(".cleartext", EsbParam.cleartext, sizeof(EsbParam.cleartext));
  Debug_PrintHexArray(".ciphertext", EsbParam.ciphertext, sizeof(EsbParam.ciphertext));
  NRF_LOG_RAW_INFO("-----------------------------------------------\r\n");

  return res;
}
 
 uint8_t data1[128+ 10];
void Debug_Func(BLE_COMMAND *pCommand) {
 RESULT res = ERR_NO;

  int8_t i = 0;
//  FlashData_SendToHost(FD_DATA_LOG_FILE, res, data1, 40);
  /* Debug_CheckEncDec(); */
  /* Debug_GetEncrCommand(); */
  /* Debug_MessageSend(); */
  /* SetNewRandomNubers(false); */
  //Flash_LogRead(0, 256);
  //Cmd_SetNewRandomNubers(true);
  /* Leonid */
  bleShowParamTab();
  //  bleFlashErase(*(uint32_t*)pCommand->Data, *(uint32_t*)(pCommand->Data + 4));
  //  bleFlashWrite(*(uint32_t*)pCommand->Data, (uint32_t*)(pCommand->Data + 4),1);
//  for(i = 0; i < 128; i++)
//  {
//    data1[i] = i;
//  }

 // i= 0;
//  while(i < 128)
//  {
//    data1[i] = i++;
//  }
//  data1[127] = 33; 

}

void Debug_CheckFrameLength() {

}

void Debug_MessageSend() {
  Message_DeviceError(1);
}

void Debug_CheckEncDec() {
  RESULT res;
  uint8_t CommandCaseFullClose[16] = {0x01, 0x01, 0x02};
  uint8_t CipherBlock16[16];
  uint8_t ClearBlock[16];

  AES_SetRandomNumberDefault();

  res = AES_BlockEncript(CHAR_COMMAND, CommandCaseFullClose, 3, CipherBlock16);
  res = AES_BlockDecript(CHAR_COMMAND, CipherBlock16, ClearBlock);
}

void Debug_GetEncrCommand() {

  uint8_t CmdFullOpen[] = {
      //0xbf, 0xa1, 0xf8, 0xb4, 0xda, 0xb2, 0xb3, 0x47,
      //0x38, 0x07, 0x92, 0x45, 0x16, 0x1d, 0xbe, 0xe2};
      0xa9, 0x4f, 0x41, 0xbf, 0xd9, 0x2c, 0xc8, 0xbb,
      0x1a, 0xea, 0x05, 0x9b, 0x15, 0xdd, 0xb8, 0x18};

  uint8_t CmdFullLock[] = {
      // 0x01, 0x01, 0x02, 0x91, 0xe8, 0x78, 0xfd, 0x30,
      // 0x32, 0x07, 0x4e, 0xdd, 0x24, 0x83, 0xdd, 0xab};
      0x29, 0x7c, 0xf4, 0x31, 0x23, 0x16, 0xbf, 0xfa,
      0x86, 0x2e, 0xdf, 0xac, 0xa3, 0xf4, 0x03, 0xd2};

  uint8_t i;

  AES_SetNewCharRandomVal(CHAR_COMMAND);

  CmdH_Command_Write(NULL, NULL,
      16, CmdFullLock);
  AES_SetNewCharRandomVal(CHAR_COMMAND);

  CmdH_Command_Write(NULL, NULL,
      16, CmdFullLock);
  //  memcpy(pCommand, CmdFullOpen, 16);
  //  memcpy(pCommand, CmdFullLock, 16);
}

//typedef struct _RTC_VALUE {
//  uint32_t SECOND : 6;
//  uint32_t MINUTE : 6;
//  uint32_t HOUR : 5;
//  uint32_t DAY : 5;
//  uint32_t MONTH : 5;
//  uint32_t YEAR : 5;
//} RTC_VALUE;

uint32_t Time_To_uint32_t(RTC_VALUE RTC_time)
{
  uint32_t value = 0;
  value = RTC_time.SECOND +
  (RTC_time.MINUTE << 6) +
  (RTC_time.HOUR << 12) +
  (RTC_time.DAY << 17) +
  (RTC_time.MONTH << 22) +
  (RTC_time.YEAR << 27);
}

RTC_VALUE uint32_t_To_Time(uint32_t value)
{
  RTC_VALUE RTC_time;
  
  RTC_time.SECOND = value & 0b111111;
  RTC_time.MINUTE = (value >> 6) & 0b111111;
  RTC_time.HOUR = (value >> 12) & 0b11111;
  RTC_time.DAY = (value >> 17) & 0b11111;
  RTC_time.MONTH = (value >> 22) & 0b11111;
  RTC_time.YEAR = (value >> 27) & 0b11111;
}
