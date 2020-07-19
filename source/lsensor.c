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

void logEventStorageReq(log_event_id_t event,uint8_t param0,uint8_t param1,uint8_t param2);

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0
extern const ParamTable_t*  pParamTable;
extern main_status_t main_status;

int8_t lsensor_DeadTime;
/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);


void twi_init(void)
{
    ret_code_t err_code;
    main_status.LightSensorWeakupTime = 0;
    main_status.LightSensorState = LS_SLEEP;
    main_status.LightSensorWeakup_req = 0;
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


#if LSENSOR == LSEN_LTR_303
void lsensor_tx(uint8_t reg_addr, uint8_t const* pdata, size_t size)
{
  ret_code_t ret;
  uint32_t i;
  uint8_t buffer[LSENSOR_MAX_TRANSACTION_SIZE + 1]; /* Addr + data */
  for(i = 0;i<size;i++){
    buffer[0] = reg_addr + i;
    buffer[1] = pdata[i];
    ret = nrf_drv_twi_tx(&m_twi, LSENSOR_I2C_ADDR, buffer,I2C_ADDR_LEN + 1, false);
    if(ret != NRF_SUCCESS){
      if(!main_status.LightSensorProblem){
        logEventStorageReq(LOG_EVENT_ERROR,ERR_LIGHT_SENSOR_PROBLEM,0,0);
        main_status.LightSensorProblem = 1;
      }else{
        main_status.LightSensorProblem = 0;
      }
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
      ret = nrf_drv_twi_tx(&m_twi, LSENSOR_I2C_ADDR, &buffer, I2C_ADDR_LEN, false);
      if(ret != NRF_SUCCESS){
        if(!main_status.LightSensorProblem){
          logEventStorageReq(LOG_EVENT_ERROR,ERR_LIGHT_SENSOR_PROBLEM,0,0);
          main_status.LightSensorProblem = 1;
        }else{
          main_status.LightSensorProblem = 0;
        }
      }
      ret = nrf_drv_twi_rx(&m_twi, LSENSOR_I2C_ADDR, pdata+i, 1);
      if(ret != NRF_SUCCESS){
        if(!main_status.LightSensorProblem){
          logEventStorageReq(LOG_EVENT_ERROR,ERR_LIGHT_SENSOR_PROBLEM,0,0);
          main_status.LightSensorProblem = 1;
        }else{
          main_status.LightSensorProblem = 0;
        }
      }
   }
}

void lsensor_init(void)
{
  uint8_t contr_reg = 2; // SW Reset
  main_status.LightSensorState = LS_SLEEP;
  lsensor_tx(LSEN_ALS_CONTR_REG,&contr_reg,1);
  contr_reg = pParamTable->lsensor.contr_reg & 0xFE; // Stand-by mode
  lsensor_tx(LSEN_ALS_CONTR_REG,&contr_reg,1);

  lsensor_tx(LSEN_ALS_THRES_UP_0_REG,&(pParamTable->lsensor.upper_thresh_low),4);
  lsensor_tx(LSEN_INTERRUPT_PERSIST_REG,&(pParamTable->lsensor.int_persist_reg),1);
  lsensor_tx(LSEN_ALS_MEAS_RATE_REG,&(pParamTable->lsensor.meas_rate),1);
  contr_reg = LSEN_INTERRUPT_EN;
  lsensor_tx(LSEN_INTERRUPT_REG,&contr_reg,1);
  lsensor_rx(LSEN_ALS_STATUS_REG,&contr_reg,1); //dumy read
}

void lsensor_weak_up(void)
{
//    NRF_LOG_INFO("lsensor_weak_up.  LightSensorState = %d", main_status.LightSensorState);
  uint8_t DeadTimeId;

  if(main_status.LightSensorState != LS_WORK){
    uint8_t i;
    int8_t contr_reg = 2;  

    lsensor_tx(LSEN_ALS_CONTR_REG,&contr_reg,1);
    contr_reg = pParamTable->lsensor.contr_reg & 0xFE; // Stand-by mode
    lsensor_tx(LSEN_ALS_CONTR_REG,&contr_reg,1);
    lsensor_tx(LSEN_ALS_THRES_UP_0_REG,&(pParamTable->lsensor.upper_thresh_low),4);
    lsensor_tx(LSEN_INTERRUPT_PERSIST_REG,&(pParamTable->lsensor.int_persist_reg),1);
    lsensor_tx(LSEN_ALS_MEAS_RATE_REG,&(pParamTable->lsensor.meas_rate),1);
    contr_reg = LSEN_INTERRUPT_EN;
    lsensor_tx(LSEN_INTERRUPT_REG,&contr_reg,1);
    contr_reg = pParamTable->lsensor.contr_reg | 0x01; // Active mode
    lsensor_tx(LSEN_ALS_CONTR_REG,&contr_reg,1);             // send reg value
    main_status.LightSensorState = LS_WORK;
    NRF_LOG_INFO("LightSensorState = %01x",main_status.LightSensorState);
    lsensor_DeadTime = LS_DEAD_TIME;
    main_status.LightSensor_IntEn = 1;

  }
}

void lsensor_rx_all( void ){
  uint8_t buff[2];

  lsensor_rx(LSEN_ALS_CONTR_REG,buff,1); 
  NRF_LOG_INFO("ALS_CONTR: %01x", buff[0]);
  lsensor_rx(LSEN_ALS_STATUS_REG,buff,1); 
  NRF_LOG_INFO("ALS_STATUS: %01x", buff[0]);
  lsensor_rx(LSEN_INTERRUPT_REG,buff,1); 
  NRF_LOG_INFO("INTERRUPT: %01x", buff[0]);
  lsensor_rx(LSEN_INTERRUPT_PERSIST_REG,buff,1); 
  NRF_LOG_INFO("INTERRUPT_PERSIST: %01x", buff[0]);
}

//#else
//#ifdef  LSEN_OPT3002
#elif LSENSOR == LSEN_OPT3002


void lsensor_tx(uint8_t reg_addr, uint8_t const* pdata, size_t size)
{
  ret_code_t ret;
  uint32_t i;
  uint8_t buffer[LSENSOR_MAX_TRANSACTION_SIZE + 1]; /* Addr + data */
  for(i = 0;i<size;i+=2){
    buffer[0] = reg_addr + i;
    buffer[2] = pdata[i];
    buffer[1] = pdata[i+1];
    ret = nrf_drv_twi_tx(&m_twi, LSENSOR_I2C_ADDR, buffer,I2C_ADDR_LEN + 2, false);
    if(ret != NRF_SUCCESS){
      if(!main_status.LightSensorProblem){
        logEventStorageReq(LOG_EVENT_ERROR,ERR_LIGHT_SENSOR_PROBLEM,0,0);
        main_status.LightSensorProblem = 1;
      }else{
        main_status.LightSensorProblem = 0;
      }
    }
  }
}

void lsensor_rx(uint8_t reg_addr, uint8_t* pdata, size_t size)
{
    ret_code_t ret;
    uint32_t i;
    uint32_t repetition_cntr = I2C_REPETITION_MAX;
    uint8_t RxData[2];
    uint8_t buffer; 
    for(i = 0;i<size;i+=2){
      buffer = reg_addr + i;
      ret = nrf_drv_twi_tx(&m_twi, LSENSOR_I2C_ADDR, &buffer, I2C_ADDR_LEN, false);
      if(ret != NRF_SUCCESS){
        if(!main_status.LightSensorProblem){
          logEventStorageReq(LOG_EVENT_ERROR,ERR_LIGHT_SENSOR_PROBLEM,0,0);
          main_status.LightSensorProblem = 1;
        }else{
          main_status.LightSensorProblem = 0;
        }
      }
      ret = nrf_drv_twi_rx(&m_twi, LSENSOR_I2C_ADDR, RxData, 2);
      if(ret != NRF_SUCCESS){
        if(!main_status.LightSensorProblem){
          logEventStorageReq(LOG_EVENT_ERROR,ERR_LIGHT_SENSOR_PROBLEM,0,0);
          main_status.LightSensorProblem = 1;
        }else{
          main_status.LightSensorProblem = 0;
        }
      }
      pdata[i]=RxData[0];
      pdata[i+1]=RxData[1];
   }
}

void lsensor_init(void)
{
  uint16_t reg_val = 0; // Sleep mode
  main_status.LightSensorState = LS_SLEEP;
  reg_val = pParamTable->lsensor.config_reg & 0x00FF;
  lsensor_tx(LSEN_CONFIG_REG,(uint8_t*)&reg_val,2);
  lsensor_tx(LSEN_LOW_LIMIT_REG,(uint8_t*)&(pParamTable->lsensor.lower_threshold),2); 
  lsensor_rx(LSEN_CONFIG_REG,(uint8_t*)&reg_val,2); // read configuration register to reset interrupt
}

void lsensor_weak_up(void)
{
//    NRF_LOG_INFO("lsensor_weak_up.  LightSensorState = %d", main_status.LightSensorState);
  int8_t buff[2];  

  if(main_status.LightSensorState != LS_WORK){
    lsensor_tx(LSEN_CONFIG_REG,(uint8_t*)&(pParamTable->lsensor.config_reg),4);
    main_status.LightSensorState = LS_WORK;
    NRF_LOG_INFO("LightSensorState = %01x",main_status.LightSensorState);
    lsensor_rx(LSEN_CONFIG_REG,buff,2);   // read configuration register to reset interrupt
    lsensor_DeadTime = LS_DEAD_TIME;
    main_status.LightSensor_IntEn = 1;
  }
}

void lsensor_rx_all( void ){
  uint8_t buff[2] = {0x00,0x05};
  
  lsensor_rx(LSEN_RESULT_REG,buff,2); 
  NRF_LOG_INFO("LSEN_RESULT_REG: %02x%02x", buff[0],buff[1]);
  lsensor_rx(LSEN_CONFIG_REG,buff,2); 
  NRF_LOG_INFO("LSEN_CONFIG_REG: %02x%02x", buff[0],buff[1]);
  lsensor_rx(LSEN_LOW_LIMIT_REG,buff,2); 
  NRF_LOG_INFO("LSEN_LOW_LIMIT_REG: %02x%02x", buff[0],buff[1]);
  lsensor_rx(LSEN_HIGHT_LIMIT_REG,buff,2); 
  NRF_LOG_INFO("LSEN_HIGHT_LIMIT_REG: %02x%02x", buff[0],buff[1]);
  lsensor_rx(LSEN_MANUFACT_ID_REG,buff,2); 
  NRF_LOG_INFO("LSEN_MANUFACT_ID_REG: %02x%02x", buff[0],buff[1]);

}

void seek_addr(void)
{
  ret_code_t ret;
  uint32_t i;
  uint8_t i2c_addr = 1;
  uint8_t buffer[4]; /* Addr + data */
  for(i2c_addr = 1; i2c_addr<0x80;i2c_addr++){

      buffer[0] = LSEN_CONFIG_REG;
      buffer[2] = 0;
      buffer[1] = 0;
      ret = nrf_drv_twi_tx(&m_twi, i2c_addr, buffer,I2C_ADDR_LEN + 2, false);
      if(ret == NRF_SUCCESS){
        NRF_LOG_INFO("ADDR: %01",i2c_addr);
        break;
      
      }
  }
}

#endif
//#endif
