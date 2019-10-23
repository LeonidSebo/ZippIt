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
#ifndef RAYTEC_H
#define RAYTEC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"

// LEDs definitions for PCA10040
#define LEDS_NUMBER    3

#define LED_START      6
#define LED_1          6
#define LED_2          7
#define LED_3          8
//#define LED_4          20
#define LED_STOP       8

#define LEDS_ACTIVE_STATE 1

#define LEDS_INV_MASK  LEDS_MASK

#define LEDS_LIST { LED_1, LED_2, LED_3} //, LED_4 }

#define BSP_LED_0      LED_1
#define BSP_LED_1      LED_2
#define BSP_LED_2      LED_3
//#define BSP_LED_3      LED_4

#define BUTTONS_NUMBER 4

#define BUTTON_START   25
#define BUTTON_1       25
#define BUTTON_2       26
#define BUTTON_3       27
#define BUTTON_4       28
#define BUTTON_STOP    28
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_ACTIVE_STATE 0

#define BUTTONS_LIST { BUTTON_1, BUTTON_2, BUTTON_3} //, BUTTON_4 }

#define BSP_BUTTON_0   BUTTON_1
#define BSP_BUTTON_1   BUTTON_2
#define BSP_BUTTON_2   BUTTON_3
//#define BSP_BUTTON_3   BUTTON_4

#define SENSOR_INT_PIN BUTTON_1

#define SENSOR_SDA_PIN	3
#define SENSOR_SCL_PIN	4


#define RX_PIN_NUMBER     17
#define TX_PIN_NUMBER     21
#define CTS_PIN_NUMBER    18
#define RTS_PIN_NUMBER    19

/******************************/
#define MOTOR_IN1_PIN		11
#define MOTOR_IN2_PIN		12
#define MOTOR_EN_PIN		13

#define nSW3_PIN			16
#define nSW2_PIN			14
#define nSW1_PIN			15
#define nWIRE_PIN			20

#define LED_R_PIN			6
#define LED_G_PIN			7
#define LED_B_PIN			8

#define BAT_IN_PIN			30
#define BAT_IN_ANALOG_CH4	4

#define EN_6V_PIN			21

#ifdef __cplusplus
}
#endif

#endif // RAYTEC_H
