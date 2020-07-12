/* file aux_i2c.c  
// auxiliary i2c interface 
// Created by Leonid Sorkin */

#include <stdbool.h>
#include  <string.h>
#include "nrf_drv_twi.h"
#include "sdk_config.h"
#include "boards.h"
#include "types.h"
#include "aux_i2c.h"



/* TWI instance ID. */
#define TWI_INSTANCE_ID_AUX     1



/* Indicates if operation on TWI has ended. */
//static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi_aux = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID_AUX);

void i2c_aux_init(void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_aux_config = {
       .scl                = AUX_SCL,
       .sda                = AUX_SDA,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi_aux, &twi_aux_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi_aux);
}

void aux_i2c_tx(uint8_t const* pdata)
{
  ret_code_t ret;
  uint8_t buffer[3]; /* Addr + data */
  buffer[0] = ALARM_REG_ADDR;          
  buffer[1] = pdata[0];   // alarm
  buffer[2] = 0;
  ret = nrf_drv_twi_tx(&m_twi_aux, AUX_I2C_ADDR, buffer,3, false);
  if(ret == NRF_SUCCESS){
    if(pdata[0]&ALARM_BAT){
      buffer[0] = BAT_LEVEL_REG_ADDR;          
      buffer[1] = pdata[2];   
      buffer[2] = pdata[3];
      ret = nrf_drv_twi_tx(&m_twi_aux, AUX_I2C_ADDR, buffer,3, false);
    }
  }
}
