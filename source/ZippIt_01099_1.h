/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef PCA10040_H
#define PCA10040_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"

#define SENSOR_INT_PIN		2
#define SENSOR_SDA_PIN		3
#define SENSOR_SCL_PIN		4
#define SENSOR_SPARE_PIN	5

#define SPI_MISO_PIN		6
#define SPI_MOSI_PIN		7
#define SPI_CLK_PIN			8
#define SPI_nCS_PIN			12

#define NFC1_PIN_PIN		9
#define NFC2_PIN_PIN		10

#define MOTOR_IN1_PIN		11
#define MOTOR_IN2_PIN		17
#define MOTOR_EN_PIN		31

#define nSW3_PIN			13
#define nSW2_PIN			14
#define nSW1_PIN			15

#define EN_6V_PIN			16

#define LED_R_PIN			18
#define LED_G_PIN			19
#define LED_B_PIN			20

#define MODE_nSW0_PIN		22
#define MODE_nSW1_PIN		26
#define MODE_nSW2_PIN		27
#define MODE_nSW3_PIN		28

#define nWIRE_PIN			29

#define BAT_IN_PIN			30
#define BAT_IN_ANALOG_CH4	4

// LEDs definitions 
#define LEDS_NUMBER    3

#define LED_START      LED_R_PIN
#define LED_STOP       LED_B_PIN

#define LEDS_ACTIVE_STATE 1

#define LEDS_INV_MASK  LEDS_MASK

#define LEDS_LIST { LED_R_PIN, LED_G_PIN, LED_B_PIN}

#define BSP_LED_0      LED_R_PIN
#define BSP_LED_1      LED_G_PIN
#define BSP_LED_2      LED_B_PIN

#define TP2_PIN			25

/**************************************/
// One of the virtual pages is reserved by the system for garbage collection.
// Therefore, the minimum is two virtual pages: one page to store data and one page to be used by the system for garbage collection.
// The total amount of flash memory that is used by FDS amounts to @ref FDS_VIRTUAL_PAGES * @ref FDS_VIRTUAL_PAGE_SIZE * 4 bytes.
// FDS_VIRTUAL_PAGE_SIZE = 1024
#define FLASH_VIRTUAL_PAGES			10+1 



#ifdef __cplusplus
}
#endif

#endif // PCA10040_H
