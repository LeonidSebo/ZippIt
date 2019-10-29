/* file lsensor.c  
// Light sensor control 
// I2C using */

#include "lsensor.h"
#include <stdbool.h>
#include  <string.h>
#include "nrf_drv_twi.h"
#include "sdk_config.h"
#include "boards.h"
#include "types.h"



/* TWI instance ID. */
#define TWI_INSTANCE_ID     0
extern const ParamTable_t*  pParamTable;


/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);


void twi_init(void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_lsnsor_config = {
       .scl                = SENSOR_SCL_PIN,
       .sda                = SENSOR_SDA_PIN,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_lsnsor_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

void lsensor_tx(uint8_t reg_addr, uint8_t const* pdata, size_t size)
{
    ret_code_t ret;
    uint32_t i;
    uint32_t repetition_cntr = I2C_REPETITION_MAX;
    uint8_t buffer[LSENSOR_MAX_TRANSACTION_SIZE + 1]; /* Addr + data */
    for(i = 0;i < size; i++){
      buffer[0] = reg_addr + i;
      buffer[1] = pdata[i];
      while(repetition_cntr--){
        ret = nrf_drv_twi_tx(&m_twi, LSENSOR_I2C_ADDR, buffer,I2C_ADDR_LEN + 1, false);
        if(ret == NRF_SUCCESS){
          break;
        }
        if(ret == NRF_ERROR_DRV_TWI_ERR_ANACK){
          continue;
        }
        APP_ERROR_CHECK(ret);
      }
    }
}

void lsensor_rx(uint8_t reg_addr, uint8_t* pdata, size_t size)
{
    ret_code_t ret;
    uint32_t i;
    uint32_t repetition_cntr = I2C_REPETITION_MAX;
    uint8_t buffer; 
    for(i = 0;i<size;i++){
      buffer = reg_addr + i;
      while(repetition_cntr--){
        ret = nrf_drv_twi_tx(&m_twi, LSENSOR_I2C_ADDR, &buffer, I2C_ADDR_LEN, false);
        if(ret == NRF_SUCCESS){
          break;
        }
        if(ret == NRF_ERROR_DRV_TWI_ERR_ANACK){
          continue;
        }
        APP_ERROR_CHECK(ret);
      }
      ret = nrf_drv_twi_rx(&m_twi, LSENSOR_I2C_ADDR, pdata+i, 1);
      APP_ERROR_CHECK(ret);
    }
}

void lsensor_init(void)
{
  uint8_t contr_reg = pParamTable->lsensor.contr_reg & 0xFE; // Stand-by mode

  lsensor_tx(LSEN_ALS_CONTR_REG,&contr_reg,1);
  lsensor_tx(LSEN_ALS_THRES_UP_0_REG,&(pParamTable->lsensor.upper_thresh_low),4);
  lsensor_tx(LSEN_INTERRUPT_PERSIST_REG,&(pParamTable->lsensor.int_persist_reg),1);
  lsensor_tx(LSEN_ALS_MEAS_RATE_REG,&(pParamTable->lsensor.meas_rate),1);
  lsensor_tx(LSEN_INTERRUPT_REG,&(pParamTable->lsensor.interrupt),1);

}

void lsensor_sleep(void)
{
  uint8_t contr_reg = pParamTable->lsensor.contr_reg & 0xFE; // Stand-by mode
  nrfx_gpiote_in_event_disable(SENSOR_INT_PIN);            // desable interrupt
  lsensor_tx(LSEN_ALS_CONTR_REG,&contr_reg,1);             // send reg value
}

void lsensor_weak_up(void)
{
  uint8_t contr_reg = pParamTable->lsensor.contr_reg | 0x01; // Active mode
  nrfx_gpiote_in_event_disable(SENSOR_INT_PIN);            // desable interrupt
  lsensor_tx(LSEN_ALS_CONTR_REG,&contr_reg,1);             // send reg value
  nrfx_gpiote_in_event_enable(SENSOR_INT_PIN, true);

}
