#include "AES_Security.h"
//#include "nrf_soc.h"
#include "math.h"
#include "nrf_crypto_error.h"
#include "nrf_delay.h"
#include "Debug.h"

#define DEBUG_PRINT_RANDOM_AND_KEY_EN     0

#define NRF_CRYPTO_EXAMPLE_AES_MAX_TEXT_SIZE 120
#define RNG_BYTE_WAIT_US (124UL)
#define AES_DATA_OFFSET 0


AES_CHARACTERISTIC_INFO gCharInfo[CHARACTERISTICS_NO];

static uint8_t gKey[16] = {
    0x79, 0x2F, 0x42, 0x3F, 0x45, 0x28, 0x48, 0x2B,
    0x4D, 0x62, 0x51, 0x65, 0x54, 0x68, 0x57, 0x6D};

static uint8_t gIV[16] = {
    0x46, 0x29, 0x4A, 0x40, 0x4E, 0x63, 0x52, 0x66, 
    0x55, 0x6A, 0x57, 0x6E, 0x5A, 0x72, 0x34, 0x75};

RESULT AES_Init() {
  AES_SetRandomNumberDefault();
}

void AES_SetRandomNumberDefault() 
{
  uint32_t RandomDefault;
  
  RandomDefault = gKey[0] + (gKey[1] << 8) + (gKey[2] << 16);
  memset(gCharInfo, 0, sizeof(AES_CHARACTERISTIC_INFO) * CHARACTERISTICS_NO);
  
  gCharInfo[CHAR_COMMAND].PRandomNo = RandomDefault;
  gCharInfo[CHAR_ANSWER].PRandomNo = RandomDefault;
  gCharInfo[CHAR_MESSAGE].PRandomNo = RandomDefault;
  gCharInfo[CHAR_FLASH_DATA].PRandomNo = RandomDefault;

#if DEBUG_PRINT_RANDOM_AND_KEY_EN
   NRF_LOG_INFO("AES_SetRandomNumberDefault: 0x%06x, 0x%06x, 0x%06x", gCharInfo[CHAR_COMMAND].PRandomNo, 
   gCharInfo[CHAR_ANSWER].PRandomNo, gCharInfo[CHAR_MESSAGE].PRandomNo);
#endif
}

void AES_SetNewCharRandomVal(CHARACTERISTIC_ID CharacteristicID) {
  int32_t m;
  uint32_t Counter;
  uint32_t Temp;
  m = (int32_t)pow(2, (double)PRAND_FACTOR_k);
  Counter = gCharInfo[CharacteristicID].PRandomNo;
  Temp = (Counter * PRAND_FACTOR_a) + PRAND_FACTOR_b;
  gCharInfo[CharacteristicID].PRandomNo = Temp % m;

#if DEBUG_PRINT_RANDOM_AND_KEY_EN
   //NRF_LOG_INFO("AES_SetNewCharRandomVal: Characteristic %d - 0x%06x", CharacteristicID, gCharInfo[CharacteristicID].PRandomNo); 
   //gCharInfo[CHAR_ANSWER].PRandomNo, gCharInfo[CHAR_MESSAGE].PRandomNo);
#endif
}

RESULT AES_GetNewRandomNumbers(uint8_t *pNewRandomNumbers) {
  RESULT res;
  int32_t i;

  /* Generate new Counters values*/
  res = AES_RandFillArray(pNewRandomNumbers, AES_BLOCK_RANDOM_NO_SIZE_BYTE * CHARACTERISTICS_NO);
  RESULT_CHECK(res);
  
  Debug_PrintHexArray("AES_GetNewRandomNumbers: ", pNewRandomNumbers, 9);

//  uint32_t val;
//  val = 0x111111;
//  memset(pNewRandomNumbers, val, 3);
//  val = 0x222222;
//  memset(pNewRandomNumbers + 3, val, 3);
//  val = 0x333333;
//  memset(pNewRandomNumbers + 6, val, 3);
  
  
  //  for(i = 0; i < CHARACTERISTICS_NO; i++)
  //  {
  //    gCharInfo[i].PRandomNo = 0;
  //    memcpy(&(gCharInfo[i].PRandomNo),
  //          pNewCounters + AES_BLOCK_COUNTER_SIZE_BYTE * i, AES_BLOCK_COUNTER_SIZE_BYTE);
  //  }
}

void AES_SetNewRandomNumbers(uint8_t *pNewRandomNumbers) {
  int32_t i;
  NRF_LOG_FLUSH();
  for (i = 0; i < CHARACTERISTICS_NO; i++) {
    gCharInfo[i].PRandomNo = 0;
    memcpy(&(gCharInfo[i].PRandomNo),
        pNewRandomNumbers + AES_BLOCK_RANDOM_NO_SIZE_BYTE * i, AES_BLOCK_RANDOM_NO_SIZE_BYTE);
    NRF_LOG_INFO("Old Random %d %02x", i, gCharInfo[i].PRandomNo);
  }
  AES_SetNewCharRandomVal(CHAR_COMMAND);
  AES_SetNewCharRandomVal(CHAR_ANSWER);
  AES_SetNewCharRandomVal(CHAR_MESSAGE);
  AES_SetNewCharRandomVal(CHAR_FLASH_DATA);
  for (i = 0; i < CHARACTERISTICS_NO; i++) {
    NRF_LOG_INFO("New Random %d %06x", i, gCharInfo[i].PRandomNo);
  }
}

RESULT AES_BlockEncript(CHARACTERISTIC_ID CharacteristicID, uint8_t *pClearBlock, uint8_t ClearBlockLen,
  uint8_t *pCipherBlock16) {
  
  uint8_t EncrBuffer[AES_BLOCK_SIZE_BYTE];
  uint8_t NewKey[AES_KEY_SIZE_BYTE];
  RESULT res;

  uint32_t DataLen = ClearBlockLen;

  /* Copy Security Key */
  memcpy(NewKey, gKey, AES_KEY_SIZE_BYTE);
  /* Copy Counter */
  memcpy(NewKey, &(gCharInfo[CharacteristicID].PRandomNo), AES_BLOCK_RANDOM_NO_SIZE_BYTE);

  /* Fill the buffer with random numbers  */
  res = AES_RandFillArray(pCipherBlock16, AES_BLOCK_SIZE_BYTE);
  RESULT_CHECK(res);
  //memset(pCipherBlock16, 0, AES_BLOCK_SIZE_BYTE);

  /* Fill Data */
  memcpy(pCipherBlock16 + AES_DATA_OFFSET, pClearBlock, DataLen);
  /* Encription IV */
  #if(!CHAR_ANSWER_ENCRIPTION_DISABLE)
  {
    res = AES_EncodeBlock(NewKey, gIV, EncrBuffer);
    RESULT_CHECK(res);
    /* Xor ClearData with Encription IV*/
    AES_XorArray2(EncrBuffer, pCipherBlock16, AES_BLOCK_SIZE_BYTE);
      res = AES_EncodeBlock(NewKey, gIV, EncrBuffer);
      RESULT_CHECK(res);
  }
  #endif

#if DEBUG_PRINT_RANDOM_AND_KEY_EN 
  NRF_LOG_INFO("Characteritic %d:", CharacteristicID);
  Debug_PrintHexArray("In  AES_BlockEncript", pClearBlock, 16);
  Debug_PrintHexArray("Out AES_BlockEncript", pCipherBlock16, 16);
#endif
  return ERR_NO;
}

RESULT AES_BlockDecript(CHARACTERISTIC_ID CharacteristicID,
    uint8_t *pCipherBlock16, uint8_t *pClearBlock) {
  uint8_t DecrBuffer[AES_BLOCK_SIZE_BYTE];
  uint8_t NewKey[AES_KEY_SIZE_BYTE];
  RESULT res;

  /* Copy Security Key */
  memcpy(NewKey, gKey, AES_KEY_SIZE_BYTE);
  /* Copy PRandom number */
  memcpy(NewKey, &(gCharInfo[CharacteristicID].PRandomNo), AES_BLOCK_RANDOM_NO_SIZE_BYTE);

  /* Encription IV */
  res = AES_EncodeBlock(NewKey, gIV, DecrBuffer);
  RESULT_CHECK(res);
  /* Xor Chiper Data with Encription IV*/
  AES_XorArray3(DecrBuffer, pCipherBlock16, pClearBlock, AES_BLOCK_SIZE_BYTE);

#if DEBUG_PRINT_RANDOM_AND_KEY_EN
  NRF_LOG_INFO("Characteritic %d:", CharacteristicID);
  Debug_PrintHexArray("In  AES_BlockDecript", pCipherBlock16, 16);
  Debug_PrintHexArray("Out AES_BlockDecript", pClearBlock, 16);
#endif
  return ERR_NO;
}

RESULT AES_BlockEncript1(CHARACTERISTIC_ID CharacteristicID, uint8_t ID,
  uint8_t *pClearData, uint8_t ClearDataLength, uint8_t *pCipherBlock16) {
  uint8_t EncrBuffer[AES_BLOCK_SIZE_BYTE];
  uint8_t NewKey[AES_KEY_SIZE_BYTE];
  RESULT res;

  /* Copy Security Key */
  memcpy(NewKey, gKey, AES_KEY_SIZE_BYTE);
  /* Copy PRandom number */
  memcpy(NewKey, &(gCharInfo[CharacteristicID].PRandomNo), AES_BLOCK_RANDOM_NO_SIZE_BYTE);

  res = AES_RandFillArray(pCipherBlock16, AES_BLOCK_SIZE_BYTE);
  RESULT_CHECK(res);

  /* Fill Data */
  pCipherBlock16[0] = ID;
  pCipherBlock16[1] = ClearDataLength;
  memcpy(pCipherBlock16 + 2, pClearData, ClearDataLength);
  /* Encription IV */
  AES_EncodeBlock(NewKey, gIV, EncrBuffer);
  /* Xor ClearData with Encription IV*/
  AES_XorArray2(EncrBuffer, pCipherBlock16, AES_BLOCK_SIZE_BYTE);
  return ERR_NO;
}

RESULT AES_BlockDecript1(CHARACTERISTIC_ID CharacteristicID, uint8_t *pBlock16,
    uint8_t *pData, uint8_t *pDataLength) {
  uint8_t NewKey[AES_BLOCK_SIZE_BYTE];
  uint8_t DecrBuffer[AES_KEY_SIZE_BYTE];
  RESULT res;

  memcpy(NewKey, gKey, AES_KEY_SIZE_BYTE);
  memcpy(NewKey, &(gCharInfo[CharacteristicID].PRandomNo), AES_BLOCK_RANDOM_NO_SIZE_BYTE);

  AES_EncodeBlock(NewKey, gIV, DecrBuffer);
  AES_XorArray2(DecrBuffer, pBlock16, AES_BLOCK_SIZE_BYTE);

  *pDataLength = pBlock16[0];

  memcpy(pData, pBlock16 + 1, *pDataLength);

  return ERR_NO;
}

/*
uint32_t AES_GetDefaultKey(CHARACTERISTIC_ID CharacteristicID) {
  switch (CharacteristicID) {
  case CHAR_COMMAND:
    return DEFAULT_VAL_PRANDOM_CHAR_COMMAND;
  case CHAR_ANSWER:
    return DEFAULT_VAL_PRANDOM_CHAR_ANSWER;
  case CHAR_MESSAGE:
    return DEFAULT_VAL_PRANDOM_CHAR_MESSAGE;
  }
  return 0;
}
*/

RESULT AES_EncodeBlock(uint8_t *pKey, uint8_t *pCleartext, uint8_t *pCiphertext) {
  nrf_ecb_hal_data_t aes;
  ret_code_t err_code;

  memset(&aes, 0, sizeof(nrf_ecb_hal_data_t));
  memcpy(aes.key, pKey, SOC_ECB_KEY_LENGTH);
  memcpy(aes.cleartext, gIV, SOC_ECB_CLEARTEXT_LENGTH);

  err_code = sd_ecb_block_encrypt(&aes);
  if (err_code) {
    return ERR_AES_ECB_BLOCK_ENCRYPT;
  }

  memcpy(pCiphertext, aes.ciphertext, SOC_ECB_CLEARTEXT_LENGTH);
  return ERR_NO;
}

RESULT AES_RandFillArray(uint8_t *pArray, uint32_t Len) {
  uint32_t err_code;
  uint8_t available = 0;

  while (Len > available) {
    err_code = sd_rand_application_bytes_available_get(&available);
    if (err_code) {
      return ERR_GET_RAND_BYTES;
    }
    if (Len > available) {
      nrf_delay_us(RNG_BYTE_WAIT_US * (Len - available));
    }
  }
  err_code = sd_rand_application_vector_get(pArray, Len);
  if (err_code) {
    return ERR_GET_RAND_BYTES;
  }
  return ERR_NO;
}

void AES_XorArray3(uint8_t *pIn0, uint8_t *pIn1, uint8_t *pOut, uint32_t Len) {
  uint8_t i;
  for (i = 0; i < Len; i++) {
    pOut[i] = pIn0[i] ^ pIn1[i];
  }
}

void AES_XorArray2(uint8_t *pIn, uint8_t *pInOut, uint32_t Len) {
  uint8_t i;
  for (i = 0; i < Len; i++) {
    pInOut[i] ^= pIn[i];
  }
}

void Debug_OFB_Encript() {
  //RESULT res = ERR_NO;
  //  return ERR_NO;
  ret_code_t err_code;

  uint8_t Key[] = {
      0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
      0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};

  uint8_t IV[] = {
      0x00, 010, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70,
      0x80, 0x90, 0xA0, 0xB0, 0xC0, 0xD0, 0xE0, 0xF0};

  uint8_t Data_0[] = {
      0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
      0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};

  uint8_t EncdBuffer[SOC_ECB_CIPHERTEXT_LENGTH];
  uint8_t DecBuffer[SOC_ECB_CIPHERTEXT_LENGTH];

  AES_EncodeBlock(Key, IV, EncdBuffer);
  AES_XorArray2(Data_0, EncdBuffer, SOC_ECB_CIPHERTEXT_LENGTH);

  AES_EncodeBlock(Key, IV, DecBuffer);
  AES_XorArray2(EncdBuffer, DecBuffer, SOC_ECB_CIPHERTEXT_LENGTH);
}
