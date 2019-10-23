#ifndef INT_FLASH_H
#define INT_FLASH_H
#include <stdint.h>
#include <stddef.h>


#define INT_FLESH_START_ADDR    0x10000
#define INT_FLESH_LAST_ADDR     0x7FFFF

#define PARAM_TAB_ADDR      0x10000
#define PAGE_SIZE           0x01000
#define PAGE_SIZE_WORDS     0x00400

uint32_t find_free_addr(uint32_t* start_addr);
//void fstorage_init(void);
void int_flash_read(uint32_t addr, uint32_t* pdata, size_t size);
void int_flash_erase(uint32_t addr, size_t pages_cnt);
void int_flash_write(uint32_t addr, uint32_t* pdata, size_t size);
void WriteParamTab(void);

#endif  //INT_FLASH_H