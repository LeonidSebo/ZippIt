#ifndef LSENSOR_H
#define LSENSOR_H

#include <stdint.h>
#include "nrf_drv_gpiote.h"

#define LSEN_LTR_303              0x0303
#define LSEN_OPT3002              0x3002
#define LSENSOR                   LSEN_OPT3002


//#ifdef LSEN_LTR_303
#if LSENSOR == LSEN_LTR_303
#define  LSENSOR_I2C_ADDR 0x29

// LTR-303ALS-01 registers
#define LSEN_ALS_CONTR_REG         0x80
#define LSEN_ALS_MEAS_RATE_REG     0x85
#define LSEN_PART_ID_REG           0x86
#define LSEN_MANUFAC_ID_REG        0x87
#define LSEN_ALS_DATA_CH1_0_REG    0x88
#define LSEN_ALS_DATA_CH1_1_REG    0x89
#define LSEN_ALS_DATA_CH0_0_REG    0x8A
#define LSEN_ALS_DATA_CH0_1_REG    0x8B
#define LSEN_ALS_STATUS_REG        0x8C
#define LSEN_INTERRUPT_REG         0x8F
#define LSEN_ALS_THRES_UP_0_REG    0x97
#define LSEN_ALS_THRES_UP_1_REG    0x98
#define LSEN_ALS_THRES_UP_LOW_0_REG    0x99
#define LSEN_ALS_THRES_UP_LOW_1_REG    0x9A
#define LSEN_INTERRUPT_PERSIST_REG     0x9E

#define LSEN_PART_NUMBER_ID        0x0A
#define LSEN_MANUFACTURER_ID       0x05

#define LSENSOR_MAX_TRANSACTION_SIZE           0x01
#define I2C_ADDR_LEN                              1

#define LSEN_UPPER_THRESH_LOW_DEF   0xFF
#define LSEN_UPPER_THRESH_HIGHT_DEF 0xFF
#define LSEN_LOWER_THRESH_LOW_DEF   0x20
#define LSEN_LOWER_THRESH_HIGHT_DEF 0x00
#define LSEN_ALS_MEAS_RATE_DEF      0x12   // Int time = 200ms, Meas rate = 200ms
#define LSEN_INTERRUPT_DEF          0x00   // Int. enable, active 0
#define LSEN_CONTR_REG_DEF          0x01    // Gain = 1, mode = Ative
#define LSEN_INT_PERSIST_DEF        0x02    // 3 measurment before asserting intr=errupt
#define LSEN_INTERRUPT_EN           0x06
#define LSEN_SLEEP     LSEN_CONTR_REG_DEF & 0xFE    // Gain = 1, mode = Stand-by
#define LSEN_WEAKUP    LSEN_CONTR_REG_DEF          // Gain = 1, mode = Active



#define LSENSOR_DEF       LSEN_UPPER_THRESH_LOW_DEF, \
                          LSEN_UPPER_THRESH_HIGHT_DEF,\
                          LSEN_LOWER_THRESH_LOW_DEF,\
                          LSEN_LOWER_THRESH_HIGHT_DEF,\
                          LSEN_ALS_MEAS_RATE_DEF,\
                          LSEN_INTERRUPT_DEF,\
                          LSEN_CONTR_REG_DEF,\
                          LSEN_INT_PERSIST_DEF 

typedef struct _lsensor_t
{
  uint8_t   upper_thresh_low;
  uint8_t   upper_thresh_hight;
  uint8_t   lower_thresh_low;
  uint8_t   lower_thresh_hight;
  uint8_t   meas_rate;
  uint8_t   interrupt;
  uint8_t   contr_reg;
  uint8_t   int_persist_reg;
}lsensor_t;

#elif LSENSOR == LSEN_OPT3002

#define  LSENSOR_I2C_ADDR                       0x44
//#define  LSENSOR_I2C_ADDR                       0x45
//#define  LSENSOR_I2C_ADDR                       0x46
//#define  LSENSOR_I2C_ADDR                       0x47
#define I2C_ADDR_LEN                              1

#define LSENSOR_MAX_TRANSACTION_SIZE            0x02

#define LSEN_RESULT_REG             0
#define LSEN_CONFIG_REG             1
#define LSEN_LOW_LIMIT_REG          2
#define LSEN_HIGHT_LIMIT_REG        3
#define LSEN_MANUFACT_ID_REG        0x7E


#define LSEN_UPPER_THRESH_LSB_DEF   0x00
#define LSEN_UPPER_THRESH_MSB_DEF   0x06
#define LSEN_LOWER_THRESH_LSB_DEF   0x20
#define LSEN_LOWER_THRESH_MSB_DEF   0x00
#define LSEN_SPARE_LSB_DEF          0x00   // Int time = 200ms, Meas rate = 200ms
#define LSEN_SPARE_MSB_DEF          0x00   // Int. enable, active 0
#define LSEN_CONFIG_REG_LSB_DEF     0x14   // 
#define LSEN_CONFIG_REG_MSB_DEF     0x06   // 


#define LSENSOR_DEF       LSEN_UPPER_THRESH_LSB_DEF,\
                          LSEN_UPPER_THRESH_MSB_DEF,\
                          LSEN_LOWER_THRESH_LSB_DEF,\
                          LSEN_LOWER_THRESH_MSB_DEF,\
                          LSEN_SPARE_LSB_DEF,\
                          LSEN_SPARE_MSB_DEF,\
                          LSEN_CONFIG_REG_LSB_DEF,\ 
                          LSEN_CONFIG_REG_MSB_DEF

typedef struct _lsensor_t
{
  uint16_t  upper_threshold;
  uint16_t  lower_threshold;
  uint16_t  spare;
  uint16_t  config_reg;
}lsensor_t;

#endif
//#endif

#define LS_SLEEP        1
#define LS_WORK         2

#define  LS_DEAD_TIME    3



#define I2C_REPETITION_MAX          5




void twi_init(void);
void lsensor_init(void);
void lsensor_weak_up(void);
void lsensor_sleep(void);
void lsensor_rx(uint8_t reg_addr, uint8_t* pdata, size_t size);
void lsensor_rx_all( void );
void seek_addr(void);

#define lsensor_sleep()     lsensor_init()

#endif  //LSENSOR_H