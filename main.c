

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
//=======================================
#include "Debug.h"
#include "BLE_Init.h"
#include "periferal.h"


#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                     /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */
#define NRF_CRYPTO_AES      1

uint32_t gEventsFlag;

/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    bsp_board_init(BSP_INIT_LEDS);
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}



static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

/**@brief Function for application main entry.
 */
int main(void)
{
    gEventsFlag = 0;
    log_init();
    timers_init();
    init_periferal();
    power_management_init();
    BLE_Init();
    NRF_LOG_INFO("ZippIT started.");
    for (;;)
    {
      if(gEventsFlag & FLASH_READ_LOG_REQUEST)
      {
        gEventsFlag &= ~FLASH_READ_LOG_REQUEST;
        Req_Flash_LogRead();
      }
      idle_state_handle();
    }
}

