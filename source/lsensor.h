#ifndef LSENSOR_H
#define LSENSOR_H

#include <stdint.h>
#include "nrf_drv_gpiote.h"

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
#define LSEN_AALS_STATUS_REG       0x8C
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
#define LSEN_INTERRUPT_DEF          0x02    // Int. enable, active 0
#define LSEN_CONTR_REG_DEF          0x01    // Gain = 1, mode = Ative
#define LSEN_INT_PERSIST_DEF        0x02    // 3 measurment before asserting intr=errupt

#define LSEN_SLEEP     LSEN_CONTR_REG_DEF & 0xFE    // Gain = 1, mode = Stand-by
#define LSEN_WEAKUP    LSEN_CONTR_REG_DEF          // Gain = 1, mode = Active

#define LS_SLEEP  1
#define LS_WORK   2

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

#define LSENSOR_DEF      LSEN_UPPER_THRESH_LOW_DEF, \
                          LSEN_UPPER_THRESH_HIGHT_DEF,\
                          LSEN_LOWER_THRESH_LOW_DEF,\
                          LSEN_LOWER_THRESH_HIGHT_DEF,\
                          LSEN_ALS_MEAS_RATE_DEF,\
                          LSEN_INTERRUPT_DEF,\
                          LSEN_CONTR_REG_DEF,\
                          LSEN_INT_PERSIST_DEF 

#define I2C_REPETITION_MAX          5




void twi_init(void);
void lsensor_init(void);
void lsensor_weak_up(void);
void lsensor_sleep(void);
void lsensor_rx(uint8_t reg_addr, uint8_t* pdata, size_t size);


#endif  //LSENSOR_H