#include "AES_Security.h"
//#include "nrf_soc.h"
#include "math.h"
#include "nrf_crypto_error.h"
#include "nrf_delay.h"

#define NRF_CRYPTO_EXAMPLE_AES_MAX_TEXT_SIZE 120
#define RNG_BYTE_WAIT_US (124UL)
#define AES_DATA_OFFSET 0

//#define AES_ERROR_CHECK(error)  \
//    do {            \
//        if (error)  \
//        {           \
//            NRF_LOG_RAW_INFO("\r\nError = 0x%x\r\n%s\r\n",           \
//                             (error),                                \
//                             nrf_crypto_error_string_get(error));    \
//            return ERR_NRF_CRYPTO; \
//        }           \
//    } while (0);
//

AES_CHARACTERISTIC_INFO gCharInfo[CHARACTERISTICS_NO];

//static uint8_t gKey[16] = {
//    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
//    0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
//
//static uint8_t gIV[16] = {
//    0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77,
//    0x78, 0x79, 0x7A, 0x7B, 0x7C, 0x7D, 0x7E, 0x7F};

static uint8_t gKey[16] = {
    0x00, 0x8e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
    0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c};

static uint8_t gIV[16] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};

#define DEFAULT_VAL_PRANDOM_CHAR_COMMAND 0x0306
#define DEFAULT_VAL_PRANDOM_CHAR_ANSWER 0x0306
#define DEFAULT_VAL_PRANDOM_CHAR_MESSAGE 0x0306

RESULT AES_Init() {
  AES_SetCountersDefault();
}

void AES_SetCountersDefault() 
{
  uint32_t RandomDefault;
  
  RandomDefault = gKey[0] + (gKey[1] << 8) + (gKey[2] << 16);
  memset(gCharInfo, 0, sizeof(AES_CHARACTERISTIC_INFO) * CHARACTERISTICS_NO);
  
  gCharInfo[CHAR_COMMAND].PRandomNo = RandomDefault;
  gCharInfo[CHAR_ANSWER].PRandomNo = RandomDefault;
  gCharInfo[CHAR_MESSAGE].PRandomNo = RandomDefault;
  /*
  gCharInfo[CHAR_COMMAND].PRandomNo = DEFAULT_VAL_PRANDOM_CHAR_COMMAND;
  gCharInfo[CHAR_ANSWER].PRandomNo = DEFAULT_VAL_PRANDOM_CHAR_ANSWER;
  gCharInfo[CHAR_MESSAGE].PRandomNo = DEFAULT_VAL_PRANDOM_CHAR_MESSAGE;
  */
}

void AES_SetNewCharCounter(CHARACTERISTIC_ID CharacteristicID) {
  int32_t m;
  uint32_t Counter;
  uint32_t Temp;
  m = (int32_t)pow(2, (double)PRAND_FACTOR_k);
  Counter = gCharInfo[CharacteristicID].PRandomNo;
  Temp = (Counter * PRAND_FACTOR_a) + PRAND_FACTOR_b;
  gCharInfo[CharacteristicID].PRandomNo = Temp % m;
}

RESULT AES_GetNewCounters(uint8_t *pNewCounters) {
  RESULT res;
  int32_t i;

  /* Generate new Counters values*/
  res = AES_RandFillArray(pNewCounters, AES_BLOCK_RANDOM_NO_SIZE_BYTE * CHARACTERISTICS_NO);
  RESULT_CHECK(res);
  //  for(i = 0; i < CHARACTERISTICS_NO; i++)
  //  {
  //    gCharInfo[i].PRandomNo = 0;
  //    memcpy(&(gCharInfo[i].PRandomNo),
  //          pNewCounters + AES_BLOCK_COUNTER_SIZE_BYTE * i, AES_BLOCK_COUNTER_SIZE_BYTE);
  //  }
}

void AES_SetNewRandomNumbers(uint8_t *pNewRandNumber) {
  int32_t i;
  for (i = 0; i < CHARACTERISTICS_NO; i++) {
    gCharInfo[i].PRandomNo = 0;
    memcpy(&(gCharInfo[i].PRandomNo),
        pNewRandNumber + AES_BLOCK_RANDOM_NO_SIZE_BYTE * i, AES_BLOCK_RANDOM_NO_SIZE_BYTE);
    NRF_LOG_INFO("Key %d %02x", i, gCharInfo[i].PRandomNo);
  }
  AES_SetNewCharCounter(CHAR_COMMAND);
  AES_SetNewCharCounter(CHAR_ANSWER);
  AES_SetNewCharCounter(CHAR_MESSAGE);
  for (i = 0; i < CHARACTERISTICS_NO; i++) {
    NRF_LOG_INFO("New Key %d %06x", i, gCharInfo[i].PRandomNo);
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
  //res = AES_RandFillArray(pCipherBlock16, AES_BLOCK_SIZE_BYTE);
  //RESULT_CHECK(res);
  memset(pCipherBlock16, 0, AES_BLOCK_SIZE_BYTE);

  /* Fill Data */
  memcpy(pCipherBlock16 + AES_DATA_OFFSET, pClearBlock, DataLen);
  /* Encription IV */
  res = AES_EncodeBlock(NewKey, gIV, EncrBuffer);
  RESULT_CHECK(res);
  /* Xor ClearData with Encription IV*/
  AES_XorArray2(EncrBuffer, pCipherBlock16, AES_BLOCK_SIZE_BYTE);
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
//RESULT AES_Encrypt(uint8_t Len_In, uint8_t *pData_In, uint8_t *pLen_Out, uint8_t *pData_Out)
//{
////    /* Encryption phase */
//   ret_code_t ret_val;
//    static nrf_crypto_aes_context_t cbc_encr_128_ctx; // AES CBC encryption context
//
//    /* Init encryption context for 128 bit key and PKCS7 padding mode */
//    ret_val = nrf_crypto_aes_init(&cbc_encr_128_ctx,
//                                  &g_nrf_crypto_aes_cbc_128_info,
//                                  //&g_nrf_crypto_aes_cbc_128_pad_pkcs7_info,
//                                  NRF_CRYPTO_ENCRYPT);
//
//    AES_ERROR_CHECK(ret_val);
//
//    /* Set key for encryption context - only first 128 key bits will be used */
//    ret_val = nrf_crypto_aes_key_set(&cbc_encr_128_ctx, gKey);
//    AES_ERROR_CHECK(ret_val);
//
//    //    memset(gIV, 0, sizeof(gIV));
//    /* Set IV for encryption context */
//
//    ret_val = nrf_crypto_aes_iv_set(&cbc_encr_128_ctx, gIV);
//    AES_ERROR_CHECK(ret_val);
//
//    //Len_In = strlen(m_plain_text);
//    //len_out = sizeof(encrypted_text);
//
//    /* Encrypt text
//       When padding is selected m_encrypted_text buffer shall be at least 16 bytes larger
//       than text_len. */
//    ret_val = nrf_crypto_aes_finalize(&cbc_encr_128_ctx,
//                                      pData_In,
//                                      Len_In,
//                                      pData_Out,
//                                      (size_t*)pLen_Out);
//    AES_ERROR_CHECK(ret_val);
//
//    // print the encrypted text
//    //encrypted_text_print(encrypted_text, len_out);
//
//    return ERR_NO;
//}
//
//RESULT AES_Decrypt(uint8_t Len_In, uint8_t *pData_In, uint8_t *pLen_Out, uint8_t *pData_Out)
//{
//    /* Decryption phase */
//    ret_code_t ret_val;
//    static nrf_crypto_aes_context_t cbc_decr_128_ctx; // AES CBC decryption context
//
//    /* Init decryption context for 128 bit key and PKCS7 padding mode */
//    ret_val = nrf_crypto_aes_init(&cbc_decr_128_ctx,
//                                  &g_nrf_crypto_aes_cbc_128_info,
//                                  //&g_nrf_crypto_aes_cbc_128_pad_pkcs7_info,
//                                  NRF_CRYPTO_DECRYPT);
//    AES_ERROR_CHECK(ret_val);
//
//
//    /* Set key for decryption context - only first 128 key bits will be used */
//    ret_val = nrf_crypto_aes_key_set(&cbc_decr_128_ctx, gKey);
//    //AES_ERROR_CHECK(ret_val);
//
//    memset(gIV, 0, sizeof(gIV));
//    /* Set IV for decryption context */
//
//    ret_val = nrf_crypto_aes_iv_set(&cbc_decr_128_ctx, gIV);
//    AES_ERROR_CHECK(ret_val);
//
//    /* Decrypt text */
//    ret_val = nrf_crypto_aes_finalize(&cbc_decr_128_ctx,
//                                      pData_In,
//                                      Len_In,
//                                      pData_Out,
//                                      (size_t*)pLen_Out);
//    //AES_ERROR_CHECK(ret_val);
//
//    /* trim padding */
//    //pData_Out[len_out] = '\0';
//
//    //decrypted_text_print(decrypted_text, len_out);
//
////    NRF_LOG_FLUSH();
////    if (memcmp(m_plain_text, decrypted_text, strlen(m_plain_text)) == 0)
////    {
////        NRF_LOG_RAW_INFO("AES CBC example with padding executed successfully.\r\n");
////    }
////    else
////    {
////        NRF_LOG_RAW_INFO("AES CBC example with padding failed!!!\r\n");
////    }
////
//
////    return (RESULT)ret_val;//ERR_NO;
//    return ERR_NO;
//}