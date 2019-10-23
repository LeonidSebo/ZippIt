
#include "int_flash.h"
#include "nrf_nvmc.h"
#include "types.h"
#include "app_error.h"
//
///******* flash control ************/

extern const ParamTable_t  ParamTable; 
extern const ParamTable_t  NewParamTable; 
extern main_status_t main_status;
extern uint32_t NewDateTime;


// read data from internal flash
// addr - start write address
// pdata - pointer to data buffer
// size - number of words (4 bytes) to read
void int_flash_read(uint32_t addr, uint32_t* pdata, size_t size)
{
    uint32_t i;
    uint32_t* paddr = (uint32_t*)addr;
    size &= 0xfffffffc;
    for(i= 0;i<size;i++){
      pdata[i] = (uint32_t)paddr[i];
    }
}

void int_flash_erase(uint32_t addr, size_t pages_cnt)
{
    int32_t i;
    uint32_t PageAddr = addr&0xFFFFF000;
    for(i=0;i<pages_cnt;i++){
      nrf_nvmc_page_erase(PageAddr);
      PageAddr += PAGE_SIZE;
    }
}

// write data to internal flash
// addr - start write address
// pdata - pointer to data
// size - number of words (4 bytes) to write
//  page must be erased.
void int_flash_write(uint32_t addr, uint32_t* pdata, size_t size)
{
//    ret_code_t rc;
    uint32_t i;
    uint32_t PageAddr = addr&0xFFFFF000;
    uint32_t WrOffset = addr&0x0000FFF;
    uint32_t WrSize = ((WrOffset + size)>PAGE_SIZE_WORDS)? (PAGE_SIZE_WORDS - WrOffset):size;  
    

    if((addr + size) > INT_FLESH_LAST_ADDR){
      APP_ERROR_CHECK(NRF_ERROR_INVALID_PARAM);
    } 
    do{
      nrf_nvmc_write_words(addr, pdata, WrSize);
      size -= WrSize;
      pdata += WrSize;
      WrSize = (size > PAGE_SIZE_WORDS)? PAGE_SIZE_WORDS:size;   
    }while(size);  
}

uint32_t find_free_addr(uint32_t* addr)
{
  uint32_t data = 0;
  do{
    if (addr[0] == 0xFFFFFFFF) 
      return (uint32_t)addr;
    addr += (sizeof(report_t)>>2);
  } while ((uint32_t)addr < (INT_FLESH_LAST_ADDR - 1)); 
  return 0xFFFFFFFF;
}

void WriteParamTab(void)
{
    uint32_t addr;
    if(main_status.ParamTab_change_req == REQ){
      main_status.ParamTab_change_req = REQ_IN_PROGRESS;
      addr = (uint32_t)&ParamTable;
      int_flash_erase(addr, 1);
      int_flash_write(addr,(uint32_t*)&NewParamTable, sizeof(ParamTable_t)>>2);
      main_status.ParamTab_change_req = REQ_NONE;
    }
}