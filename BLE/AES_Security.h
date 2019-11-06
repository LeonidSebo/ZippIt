
#ifndef AES_SECURITY_H__
#define AES_SECURITY_H__

#include "BLE_Def.h"
#include <stdbool.h>
#include <stdint.h>

//#define AES_BLOCK_DATA_SIZE_BYTE                    (AES_BLOCK_SIZE_BYTE - 2)

typedef BLE_BLOCK AES_BLOCK_DATA;

#define AES_BLOCK_SIZE_BYTE 16
#define AES_KEY_SIZE_BYTE AES_BLOCK_SIZE_BYTE
#define AES_IV_SIZE_BYTE AES_BLOCK_SIZE_BYTE
#define AES_BLOCK_RANDOM_NO_SIZE_BYTE 3

//#define AES_BLOCK_ID_SIZE_BYTE                      1
//#define AES_BLOCK_DATALEN_SIZE_BYTE                 1
//#define AES_BLOCK_DATA_SIZE_BYTE \
//  (AES_BLOCK_SIZE_BYTE  - AES_BLOCK_ID_SIZE_BYTE - AES_BLOCK_DATALEN_SIZE_BYTE)

//typedef struct _AES_HEADER_BLOCK
//{
//  uint8_t ID;
//  uint8_t Length;
//  uint8_t Data[AES_BLOCK_DATA_SIZE_BYTE];
//}AES_HEADER_BLOCK;

//#define AES_IDX_BLOCK_DATALENGTH                    3
//#define AES_IDX_BLOCK_DATA

#define PRAND_FACTOR_a 13
#define PRAND_FACTOR_b 17
#define PRAND_FACTOR_c 3
#define PRAND_FACTOR_k 7.0

typedef struct _AES_CHARACTERISTIC_INFO {
  uint32_t PRandomNo;
} AES_CHARACTERISTIC_INFO;

//typedef struct _AES_BLOCK_DATA
//{
//  uint8_t ID;
//  uint8_t DataLength;
//  uint8_t Data[AES_BLOCK_DATA_SIZE_BYTE] ;
//}AES_BLOCK_DATA;

RESULT AES_Init();

RESULT AES_EncodeBlock(uint8_t *pKey, uint8_t *pCleartext, uint8_t *pCiphertext);
//RESULT AES_BlockEncript(CHARACTERISTIC_ID CharacteristicID, uint8_t *pBlock16,
//                          uint8_t *pData, uint8_t DataLength);
RESULT AES_BlockEncript(CHARACTERISTIC_ID CharacteristicID, uint8_t *pClearBlock, uint8_t ClearBlockLen,
    uint8_t *pCipherBlock16);

RESULT AES_BlockEncript1(CHARACTERISTIC_ID CharacteristicID, uint8_t ID,
    uint8_t *pClearData16, uint8_t ClearDataLength, uint8_t *pCipherBlock16);
RESULT AES_BlockDecript(CHARACTERISTIC_ID CharacteristicID,
    uint8_t *pCipherBlock16, uint8_t *pClearBlock);
RESULT AES_BlockDecript1(CHARACTERISTIC_ID CharacteristicID, uint8_t *pBlock16,
    uint8_t *pData, uint8_t *pDataLength);

void AES_XorArray2(uint8_t *pIn, uint8_t *pInOut, uint32_t Len);
void AES_XorArray3(uint8_t *pIn0, uint8_t *pIn1, uint8_t *pOut, uint32_t Len);
RESULT AES_RandFillArray(uint8_t *pArray, uint32_t Len);
void AES_SetNewCharRandomVal(CHARACTERISTIC_ID CharacteristicID);
void AES_Connect(uint8_t *pNewCounters);
/* uint32_t AES_GetDefaultKey(CHARACTERISTIC_ID CharacteristicID); */
RESULT AES_GetNewRandomNumbers(uint8_t *pNewRandomNumbers);
void AES_SetNewRandomNumbers(uint8_t *pNewRandomNumbers);
void AES_SetRandomNumberDefault();

void Debug_OFB_Encript();

#endif // AES_SECURITY_H__