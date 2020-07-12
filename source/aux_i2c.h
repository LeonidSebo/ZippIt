/* Created by Leonid Sorkin */

#ifndef AUX_I2C_H
#define AUX_I2C_H

#define AUX_I2C_ADDR  0x54

#define ALARM_REG_ADDR      0
#define BAT_LEVEL_REG_ADDR  1

#define ALARM_CABLE         (1<<0)
#define ALARM_LIGHT         (1<<1)
#define ALARM_BAT           (1<<2)

void i2c_aux_init(void);
void aux_i2c_tx(uint8_t const* pdata);

#endif //AUX_I2C_H