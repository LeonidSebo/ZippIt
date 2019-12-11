#include "periferal.h"
#include "bleCmdHandler.h"


extern uint8_t  NewParamTable[sizeof(ParamTable_t) + sizeof(uint32_t)];
extern main_status_t main_status;

__attribute__ ((section(".ParamTab")))
const uint32_t endOfPrg = 0xFFFFFFFF;

const ParamTable_t* pParamTable = (ParamTable_t*) ((uint8_t*)&endOfPrg + PAGE_SIZE);
const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(2); /**< Declaring an instance of nrf_drv_rtc for RTC0. */

DateTime_int_t CurrentDateTime = {0};
uint32_t ReportAddr;
uint32_t CaseStateLedOnTime = 0;
uint32_t NewDateTime;
uint32_t BatVoltage;
main_status_t main_status = {0};
device_status_t device_status;
led_control_t led_control = {0,0,0};
uint32_t motorTimeout = 0;
rtc_tick_enable_t rtcTickRequest = {0,0,0};
ParamTable_t  ParamTab;
case_state_t  CaseState = {0,0,0,0,0};

log_event_store_t log_event;

//uint8_t debug_buffer[128];


#define COMPARE_COUNTERTIME  (3UL)                                        /**< Get Compare event COMPARE_TIME seconds after the counter starts from 0. */

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);

NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
{
    /* Set a handler for fstorage events. */
    .evt_handler = fstorage_evt_handler,

    /* These below are the boundaries of the flash space assigned to this instance of fstorage.
     * You must set these manually, even at runtime, before nrf_fstorage_init() is called.
     * The function nrf5_flash_end_addr_get() can be used to retrieve the last address on the
     * last page of flash available to write data. */
    .start_addr = (uint32_t)&endOfPrg, 
    .end_addr   = 0x7ffff,
};

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
    if (p_evt->result != NRF_SUCCESS)
    {
        NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.");
        return;
    }

    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x.",
                         p_evt->len, p_evt->addr);
            NRF_LOG_INFO("--> wrote %02x %02x %02x %02x %02x %02x ",((uint8_t*)&NewParamTable)[0],
                        ((uint8_t*)&NewParamTable)[1],((uint8_t*)&NewParamTable)[2],((uint8_t*)&NewParamTable)[3],
                        ((uint8_t*)&NewParamTable)[4],((uint8_t*)&NewParamTable)[5]);
//            int_flash_read((uint32_t)pParamTable, (uint32_t*) debug_buffer, sizeof(debug_buffer));
//            NRF_LOG_INFO("--> readed %02x %02x %02x %02x %02x %02x ",((uint8_t*)&debug_buffer)[0],
//                        ((uint8_t*)&debug_buffer)[1],((uint8_t*)&debug_buffer)[2],((uint8_t*)&debug_buffer)[3],
//                        ((uint8_t*)&debug_buffer)[4],((uint8_t*)&debug_buffer)[5]);
            if(log_event.log_event[log_event.log_event_rd_idx].store_flag == LOG_STORE_REQ){
              uint32_t lReportAddr = ReportAddr;
              ReportAddr += p_evt->len;
              log_event.log_event[log_event.log_event_rd_idx].store_flag = LOG_STORE_NO_REQ;
              if(log_event.log_event_rd_idx == (LOG_EVENT_MAX_CNT - 1)){
                log_event.log_event_rd_idx = 0;
              }else{
                log_event.log_event_rd_idx++;
              }
              if((lReportAddr<fstorage.end_addr - INT_FLESH_ALMOST_FULL_REST)&&(ReportAddr>=fstorage.end_addr - INT_FLESH_ALMOST_FULL_REST)){
                device_status.DEVSTAT_FLASH_LOG_FULL = 1;
                device_status.DEVSTAT_FLASH_LOG_FULL_CHANGED = YES;
                Message_DeviceStatus(device_status);           // Flash almost full event
                device_status.DEVSTAT_FLASH_LOG_FULL_CHANGED = NO;
                logEventStorageReq(LOG_EVENT_LOW_FLASH_MEMORY,0,0,0);
              }
            }
            if(main_status.ParamTab_change_req == REQ_WRITE){
              main_status.ParamTab_change_req = REQ_NONE;
            }
        } break;

        case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.",
                         p_evt->len, p_evt->addr);
            if(main_status.FlashErase_req == 1){
              main_status.FlashErase_req = 0;
              device_status.DEVSTAT_FLASH_LOG_FULL = 0;
              ReportAddr = LOG_EVENT_TAB_START_ADDR;
            }
            if(main_status.ParamTab_change_req == REQ_CHNGE){
              main_status.ParamTab_change_req = REQ_WRITE;
            }

        } break;

        default:
            break;
    }
}




void get_current_status(void)
{
  device_status.DEVSTAT_STATE_OF_SW_1 = nrf_gpio_pin_read(nSW1_PIN);
  device_status.DEVSTAT_STATE_OF_SW_2 = nrf_gpio_pin_read(nSW2_PIN);
  device_status.DEVSTAT_STATE_OF_SW_3 = nrf_gpio_pin_read(nSW3_PIN);
  device_status.DEVSTAT_WIRE_PIN = nrf_gpio_pin_read(nWIRE_PIN);
  device_status.DEVSTAT_LIGHT_PENETRATION = nrf_gpio_pin_read(SENSOR_INT_PIN); // debug
}

static void stop_motor(void)
{
  MOTOR_STOP();     
  get_current_status();
// put off motor
  motorTimeout = 0;
  rtcTickRequest.motor_buzy = 0;
  if(*(uint32_t*)&rtcTickRequest == 0)
     nrf_drv_rtc_tick_disable(&rtc);
// control state
  if((CaseState.change_case_state_req == CASE_STATE_REQ_LOOK)&&(device_status.DEVSTAT_STATE_OF_SW_3 == 0)||
     (CaseState.change_case_state_req == CASE_STATE_REQ_UNLOOK)&&(device_status.DEVSTAT_STATE_OF_SW_1 == 0)||
     (CaseState.change_case_state_req == CASE_STATE_REQ_HANDEL_OPEN)&&(device_status.DEVSTAT_STATE_OF_SW_2 == 0)){
        // Case is in true state
        NRF_LOG_INFO("Motor stop. True case state");
    switch(CaseState.change_case_state_req){
      case CASE_STATE_REQ_LOOK:
        CaseState.LookStateErrCntr = 0;
//        CaseState.LookStateDes = 0;
        CaseState.LastTrueCaseState = CASE_LOCK;
        CaseState.CurrentCaseState = CASE_LOCK;
        break;
      case CASE_STATE_REQ_HANDEL_OPEN:
        CaseState.MidleStateErrCntr = 0;
        if(CaseState.LookStateErrCntr < MAX_CASE_STATE_ERR_CNT){
          CaseState.LookStateDes = 0;
        }
        CaseState.LastTrueCaseState = CASE_HANDEL_OPEN;
        CaseState.CurrentCaseState = CASE_HANDEL_OPEN;
        break;
      case CASE_STATE_REQ_UNLOOK:
        CaseState.UnloockStateErrCntr = 0;
        CaseState.UnloockStateDes = 0;
        if(CaseState.MidleStateErrCntr < MAX_CASE_STATE_ERR_CNT){
          CaseState.MidleStateDes = 0;
          if(CaseState.LookStateErrCntr < MAX_CASE_STATE_ERR_CNT){
            CaseState.LookStateDes = 0;
          }
        }
        CaseState.LastTrueCaseState = CASE_UNLOCK;
        CaseState.CurrentCaseState = CASE_UNLOCK;
        break;
    }
  }
  else{
    CaseState.CurrentCaseState = CASE_UNRESOLVED;
    NRF_LOG_INFO("Motor stop. Error case state.");
    switch(CaseState.change_case_state_req){
      case CASE_STATE_REQ_LOOK:
      if(++CaseState.LookStateErrCntr >= MAX_CASE_STATE_ERR_CNT){
        CaseState.LookStateDes = 1;
      }
      NRF_LOG_INFO("LookStateErrCntr = %d",CaseState.LookStateErrCntr);
      break;
      case CASE_STATE_REQ_HANDEL_OPEN:
      if(++CaseState.MidleStateErrCntr >= MAX_CASE_STATE_ERR_CNT){
        CaseState.MidleStateDes = 1;
      }
      NRF_LOG_INFO("MidleStateErrCntr = %d",CaseState.MidleStateErrCntr);
      break;
      case CASE_STATE_REQ_UNLOOK:
      if(++CaseState.UnloockStateErrCntr >= MAX_CASE_STATE_ERR_CNT){
        CaseState.UnloockStateDes = 1;
      }
      NRF_LOG_INFO("UnloockStateErrCntr = %d",CaseState.UnloockStateErrCntr);
      break;
    }
  }
  CaseState.change_case_state_req = CASE_STATE_REQ_IDLE;  
//  else if(main_status.MotorAttemptCntr > 0){
//        // Case is in FALSE state after second attempt 
//        // send warning ERROR_CASE_STATE
//        NRF_LOG_INFO("Motor stop. Error case state after second attempt");
//        CaseState.change_case_state_req = CASE_STATE_REQ_IDLE;
//        main_status.MotorAttemptCntr = 0;
//        motorTimeout = 0;
//        rtcTickRequest.motor_buzy = 0;
//        if(*(uint32_t*)&rtcTickRequest == 0)
//           nrf_drv_rtc_tick_disable(&rtc);	
//  }else{
//      NRF_LOG_INFO("Motor stop. Error case state after first attempt");
//      CaseState.change_case_state_req = CASE_STATE_REQ_IDLE;
//      main_status.MotorAttemptCntr = 1;
//      motorTimeout = 0;
//      rtcTickRequest.motor_buzy = 0;
//      if(*(uint32_t*)&rtcTickRequest == 0){
//         nrf_drv_rtc_tick_disable(&rtc);	
//      }
//  }
}

void logEventStorageReq(log_event_id_t event,uint8_t param0,uint8_t param1,uint8_t param2)
{
//  NRF_LOG_INFO("logEventStorageReq: wr_idx = %d,  rd_idx = %d, store_flag = %d", 
//                log_event.log_event_wr_idx,log_event.log_event_rd_idx,
//                log_event.log_event[log_event.log_event_wr_idx].store_flag );
  if(log_event.log_event[log_event.log_event_wr_idx].store_flag == LOG_STORE_NO_REQ){
    log_event.log_event[log_event.log_event_wr_idx].log_event = event;
    log_event.log_event[log_event.log_event_wr_idx].param0 = param0;
    log_event.log_event[log_event.log_event_wr_idx].param1 = param1;
    log_event.log_event[log_event.log_event_wr_idx].param2 = param2;
    log_event.log_event[log_event.log_event_wr_idx].store_flag = LOG_STORE_REQ;
    memcpy( &log_event.log_event[log_event.log_event_wr_idx].DateTime,&CurrentDateTime,sizeof(DateTime_t));
    if(log_event.log_event_wr_idx == (LOG_EVENT_MAX_CNT-1))
      log_event.log_event_wr_idx = 0;
    else
      log_event.log_event_wr_idx++;
  }
} 

static void SetLedControl(LED_STATE R,LED_STATE G,LED_STATE B)
{
  uint32_t state;
  led_control.LED_RED = R;
  led_control.LED_GREEN = G;
  led_control.LED_BLUE = B;

//   NRF_LOG_INFO("Led State - 0x%02x", *(uint8_t*)&led_control);

  if((R == LED_BLINK)||(G == LED_BLINK)||(B == LED_BLINK)){
//    NRF_LOG_INFO("RGB - 0x%02x  0x%02x  0x%02x", *(uint8_t*)&R, *(uint8_t*)&G, *(uint8_t*)&B);
     rtcTickRequest.led_bilnk = 1;
     nrf_drv_rtc_tick_enable(&rtc,true);
  }
  if(R == LED_ON)
    nrf_gpio_pin_set(LED_R_PIN);
  else 
    nrf_gpio_pin_clear(LED_R_PIN);
  if(G == LED_ON)
    nrf_gpio_pin_set(LED_G_PIN);
  else 
    nrf_gpio_pin_clear(LED_G_PIN);
  if(B == LED_ON)
    nrf_gpio_pin_set(LED_B_PIN);
  else 
    nrf_gpio_pin_clear(LED_B_PIN);

}

static void switch_int_handler(uint8_t pin, uint8_t action)
{
  get_current_status();
  switch(pin){
    case nSW1_PIN:  // case is open
      if(CaseState.change_case_state_req == CASE_STATE_REQ_UNLOOK){
        logEventStorageReq(LOG_EVENT_CASE_OPEN, 0,0,0);
        CaseState.CurrentCaseState = CASE_UNLOCK;
        lsensor_sleep();
        main_status.LightSensorWeakupTime = 0;
        device_status.DEVSTAT_STATE_OF_SW_1_CHANGED = YES; 
        SetLedControl(LED_OFF,LED_ON,LED_OFF);
        CaseStateLedOnTime = CASE_STATE_LED_ON_TIME;
        stop_motor();
      }
      break;
    case nSW2_PIN: // case is in handel open mode
      if(CaseState.change_case_state_req == CASE_STATE_REQ_HANDEL_OPEN){
        logEventStorageReq(LOG_EVENT_CASE_HANDLE_OPEN, 0,0,0);
        CaseState.CurrentCaseState = CASE_HANDEL_OPEN;
        LIGHT_SENSOR_WEAKUP_ENABLE();
        device_status.DEVSTAT_STATE_OF_SW_2_CHANGED = YES; 
        SetLedControl(LED_OFF,LED_OFF,LED_ON);
        CaseStateLedOnTime = CASE_STATE_LED_ON_TIME;
        stop_motor();
      }
      break;
    case nSW3_PIN: // case is closed
      if(CaseState.change_case_state_req == CASE_STATE_REQ_LOOK){
        logEventStorageReq(LOG_EVENT_CASE_CLOSED, 0,0,0);
        CaseState.CurrentCaseState = CASE_LOCK;
        LIGHT_SENSOR_WEAKUP_ENABLE();
        device_status.DEVSTAT_STATE_OF_SW_3_CHANGED = YES; 
        SetLedControl(LED_ON,LED_OFF,LED_OFF);
        CaseStateLedOnTime = CASE_STATE_LED_ON_TIME;
        stop_motor();
      }
      break;
    case nWIRE_PIN:
      if(CaseState.CurrentCaseState == CASE_LOCK){
        logEventStorageReq(LOG_EVENT_WIRE_CHANGED, 0,0,0);
        device_status.DEVSTAT_WIRE_PIN_CHANGED = YES; 
      }
      break;

    case SENSOR_INT_PIN:
      device_status.DEVSTAT_LIGHT_PENETRATION = nrf_gpio_pin_read(SENSOR_INT_PIN);
      if(device_status.DEVSTAT_LIGHT_PENETRATION){
        if((CaseState.CurrentCaseState == CASE_LOCK)||(CaseState.CurrentCaseState == CASE_HANDEL_OPEN)){
          logEventStorageReq(LOG_EVENT_LIGHT_CHANGED, 0,0,0);
          device_status.DEVSTAT_LIGHT_PENETRATION_CHANGED = YES;
          NRF_LOG_INFO("Light sensor event");
          CaseStateLedOnTime = ALARM_BLINK_TIME;
          SetLedControl(LED_BLINK,LED_OFF,LED_OFF);
        }
      }
      break;
  }
  Message_DeviceStatus( device_status);             // switches state changed event
  device_status.DEVSTAT_STATE_OF_SW_1_CHANGED = NO; 
  device_status.DEVSTAT_STATE_OF_SW_2_CHANGED = NO; 
  device_status.DEVSTAT_STATE_OF_SW_3_CHANGED = NO; 
  device_status.DEVSTAT_WIRE_PIN_CHANGED = NO; 
  device_status.DEVSTAT_LIGHT_PENETRATION_CHANGED = NO;
}

static void switch_init(void)
{
    ret_code_t err_code;
    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {nSW3_PIN, APP_BUTTON_ACTIVE_LOW, NRF_GPIO_PIN_PULLUP, switch_int_handler},
        {nSW2_PIN, APP_BUTTON_ACTIVE_LOW, NRF_GPIO_PIN_PULLUP, switch_int_handler},
        {nSW1_PIN, APP_BUTTON_ACTIVE_LOW, NRF_GPIO_PIN_PULLUP, switch_int_handler},
        {nWIRE_PIN, APP_BUTTON_ACTIVE_LOW, NRF_GPIO_PIN_PULLUP, switch_int_handler},
        {SENSOR_INT_PIN, APP_BUTTON_ACTIVE_LOW, NRF_GPIO_PIN_PULLUP, switch_int_handler}
    };

    err_code = app_button_init(buttons, ARRAY_SIZE(buttons), BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);

    err_code = app_button_enable();
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for configuring: SENSOR_INT_PIN pin for input, RED_LED pin for output,
 * and configures GPIOTE to give an interrupt on pin change.
 */
static void gpio_init(void)
{
    ret_code_t err_code;
    switch_init();
    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }

/***********   initialization motor pins ***************************/
    err_code = nrf_drv_gpiote_out_init(EN_6V_PIN, &out_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_gpiote_out_init(MOTOR_EN_PIN, &out_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_gpiote_out_init(MOTOR_IN1_PIN, &out_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_gpiote_out_init(MOTOR_IN2_PIN, &out_config);
    APP_ERROR_CHECK(err_code);


/***********   initialization leds ***************************/
    bsp_board_init(BSP_INIT_LEDS);   

/********** init SENSOR_INT_PIN *******************************/
    nrf_drv_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);

    in_config.pull = NRF_GPIO_PIN_PULLUP;
}

static void StoreDevLog(void)
{
  if(nrf_fstorage_is_busy(&fstorage))
    return;
  if(log_event.log_event[log_event.log_event_rd_idx].store_flag == LOG_STORE_REQ){
    uint32_t DtateTime = *(uint32_t*)&log_event.log_event[log_event.log_event_rd_idx].DateTime;
    uint8_t event = *(uint8_t*)&log_event.log_event[log_event.log_event_rd_idx].log_event;
    NRF_LOG_INFO("StoreDevLog at addr = 0x%08x   date = 0x%08x  event = %d",ReportAddr,DtateTime,event);
    int_flash_write(ReportAddr,(uint32_t*)&log_event.log_event[log_event.log_event_rd_idx], EVENT_LOG_SIZE);
  }
}

static void LockSwitchEvent_handler(void)
{
  switch(main_status.LightSensorWeakupTime)
  {
    case 0:   // do nothing
      break;
    case 1:   // stable state
      if((CaseState.CurrentCaseState == CASE_LOCK)||(CaseState.CurrentCaseState == CASE_HANDEL_OPEN)){
        lsensor_weak_up();
        main_status.LightSensorWeakupTime = 0;
      }
    default:   // debounce time
      main_status.LightSensorWeakupTime--;
  }
}


/************* SAADC ***************************/

#define SAMPLES_IN_BUFFER 1
volatile uint8_t state = 1;

static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_IN_BUFFER];
static uint32_t              m_adc_evt_counter;

#define LOW_BATTERY_TRACE_ENABLE  0
void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
  static uint32_t minVoltage = 0xffffffff;
  if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
  {
    ret_code_t err_code;
                      
    err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
    BatVoltage = p_event->data.done.p_buffer[0];
    minVoltage = (minVoltage > BatVoltage)? BatVoltage:minVoltage;
    if(motorTimeout < RTC_TICK_TIME){
//      NRF_LOG_INFO("BatVoltage = %d   minVoltage = %d",BatVoltage,minVoltage);
    }
    if(BatVoltage < pParamTable->BatteryAlarmLevel){
      #if LOW_BATTERY_TRACE_ENABLE
        NRF_LOG_INFO("Low Battery");
      #endif
      if(device_status.DEVSTAT_POWER_LOW == NO){
        device_status.DEVSTAT_POWER_LOW = YES;
        logEventStorageReq(LOG_EVENT_ERROR,0,0,0);
        device_status.DEVSTAT_POWER_LOW_CHANGED = YES;
        get_current_status();
        Message_DeviceStatus( device_status);       // low battery event
       device_status.DEVSTAT_POWER_LOW_CHANGED = NO;
      }
    }
// cancel the measuring  
    nrf_drv_saadc_uninit();
    NRF_SAADC->INTENCLR = (SAADC_INTENCLR_END_Clear << SAADC_INTENCLR_END_Pos);
    NVIC_ClearPendingIRQ(SAADC_IRQn);
    main_status.FlashBuzy = NO;
  }
}

static void saadc_init(void)
{
    // reference voltage is 0.6 V.
    // resolution is 10 bit
    // input voltage devider is 6;
    // U (V) = 6 * 0.6 * mea_val/1024 
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}

/************** imbedded flash *********************/

/**@brief   Sleep until an event is received. */
static void power_manage(void)
{
#ifdef SOFTDEVICE_PRESENT
    (void) sd_app_evt_wait();
#else
    __WFE();
#endif
}
void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage)
{
    /* While fstorage is busy, sleep and wait for an event. */
    while (nrf_fstorage_is_busy(p_fstorage))
    {
      if (NRF_LOG_PROCESS() == false)
      {
          nrf_pwr_mgmt_run();
      }
    }  
}


// read data from internal flash
// addr - start write address
// pdata - pointer to data buffer
// size - number of bytes to read
void int_flash_read(uint32_t addr, uint32_t* pdata, size_t size)
{
    uint32_t i;
    uint32_t* paddr = (uint32_t*)(addr & 0xFFFFFFFC);
    size /= sizeof(uint32_t);
    for(i= 0;i<size;i++){
      pdata[i] = (uint32_t)paddr[i];
    }
}

void int_flash_erase(uint32_t addr, size_t pages_cnt)
{
  ret_code_t rc;

  rc = nrf_fstorage_erase(&fstorage, addr, pages_cnt, NULL);
  APP_ERROR_CHECK(rc);
  return ;
}

// write data to internal flash
// addr - start write address
// pdata - pointer to data
// size - number of words (4 bytes) to write
//  page must be erased.
void int_flash_write(uint32_t addr, uint32_t* pdata, size_t size)
{
    ret_code_t rc;
    rc =  nrf_fstorage_write(&fstorage,addr,pdata,size,NULL);
    APP_ERROR_CHECK(rc);
//    uint32_t i;
//    uint32_t PageAddr = addr&0xFFFFF000;
//    uint32_t WrPageOffset = addr&0x0000FFF;
//    uint32_t WrSize = ((WrPageOffset + size)>PAGE_SIZE)? (PAGE_SIZE - WrPageOffset):size;  
//    
//    if((addr + size) > fstorage.end_addr /*INT_FLESH_LAST_ADDR*/){
//      APP_ERROR_CHECK(NRF_ERROR_INVALID_PARAM);
//    } 
//    do{
//    rc =  nrf_fstorage_write(&fstorage,addr,pdata,WrSize,NULL);
//    APP_ERROR_CHECK(rc);
//      size -= WrSize;
//      pdata += WrSize;
//      WrSize = (size > PAGE_SIZE)? PAGE_SIZE:size;   
//    }while(size);  
}

static void print_flash_info(nrf_fstorage_t * p_fstorage)
{
    NRF_LOG_INFO("========| flash info |========");
    NRF_LOG_INFO("Start address: \t%08x ",   p_fstorage->start_addr);
    NRF_LOG_INFO("End address: \t%08x",     p_fstorage->end_addr);
    NRF_LOG_INFO("event handler: \t%08x",      p_fstorage->evt_handler);
    NRF_LOG_INFO("erase unit: \t%d bytes",      p_fstorage->p_flash_info->erase_unit);
    NRF_LOG_INFO("program unit: \t%d bytes",    p_fstorage->p_flash_info->program_unit);
    NRF_LOG_INFO("==============================");
}

uint32_t find_free_addr(uint32_t startAddr)
{
  uint32_t data;
  uint32_t addr = startAddr;
  do{
    int_flash_read(addr, &data, 4);
    
    if (data == 0xFFFFFFFF) 
      return (uint32_t)addr;
    addr += (sizeof(report_t));
  } while ((uint32_t)addr < (fstorage.end_addr /*INT_FLESH_LAST_ADDR*/ - 1)); 
  return 0xFFFFFFFF;
}

static void WriteParamTab(void)
{
    uint32_t addr;
    uint32_t ParamTabCrc;
    uint8_t currentParamTable[sizeof(ParamTable_t)+sizeof(uint32_t)];
#if 0
   uint8_t deb_buf[128];
   uint8_t i;
   for(i = 0;i<128;i++){
      debug_buffer[i] = 2;
   }
#endif
#if 0     
//debug
    int_flash_read((uint32_t)pParamTable, (uint32_t*)&currentParamTable, 8);
    NRF_LOG_INFO("currentParamTable[0] = 0x%02x   Address = 0x%08x",currentParamTable[4],(uint32_t)pParamTable+4);

#endif
    if(nrf_fstorage_is_busy(&fstorage))
      return;
#if 0
    if(main_status.ParamTab_change_req == REQ_CHNGE){
      main_status.ParamTab_change_req = REQ_WRITE;
      int_flash_erase((uint32_t)pParamTable, 1);
//      wait_for_flash_ready(&fstorage);
//      ParamTabCrc = crc32_compute((uint8_t*)&NewParamTable,sizeof(ParamTable_t),NULL);
//      memcpy(currentParamTable,&NewParamTable,sizeof(ParamTable_t));
//      memcpy(currentParamTable+sizeof(ParamTable_t),&ParamTabCrc,sizeof(uint32_t));
//      int_flash_write((uint32_t)pParamTable,(uint32_t*)currentParamTable,sizeof(currentParamTable));
//      wait_for_flash_ready(&fstorage);
    }
#else
    switch(main_status.ParamTab_change_req){
      case REQ_CHNGE:
        NRF_LOG_INFO("Erase ParamTable");
        int_flash_erase((uint32_t)pParamTable, 1);
        break;
      case REQ_WRITE:
#if 1
        int_flash_read((uint32_t)pParamTable, (uint32_t*) currentParamTable, 8);
        NRF_LOG_INFO("ParamTable  %01x %01x %01x %01x %01x %01x ...",currentParamTable[0],
           currentParamTable[1],currentParamTable[2],currentParamTable[3],currentParamTable[4],currentParamTable[5]);
#endif

        ParamTabCrc = crc32_compute(NewParamTable,sizeof(ParamTable_t),NULL);
        memcpy(NewParamTable+sizeof(ParamTable_t),&ParamTabCrc,sizeof(ParamTabCrc));
        int_flash_write((uint32_t)pParamTable,(uint32_t*)NewParamTable,sizeof(NewParamTable));
        break;
    }
#endif
}

static void initParamTab(void){
    uint8_t currentParamTable[sizeof(ParamTable_t)+sizeof(uint32_t)];
    uint32_t  ParamTabCrc;
    uint32_t  currCrc;
    NRF_LOG_INFO("initParamTab() begin");
#define FIRST_TIME 0
#if FIRST_TIME
//        NRF_LOG_INFO("update ParamTable");
//        ParamTable_t newTab = {LSENSOR_DEF, MOTOR_TIMEOUT_DEF,BATTERY_ALARM_LEVEL_DEF,1};
//        memcpy(&NewParamTable,&newTab,sizeof(ParamTable_t));
        main_status.ParamTab_change_req = REQ_CHNGE;
        int_flash_erase((uint32_t)pParamTable, 1);
//        WriteParamTab();
#else

    int_flash_read((uint32_t)pParamTable, (uint32_t*)&currentParamTable, sizeof(currentParamTable));
    memcpy(&NewParamTable,currentParamTable,sizeof(ParamTable_t));
    ParamTabCrc = crc32_compute(currentParamTable,sizeof(ParamTable_t),NULL);
    int_flash_read((uint32_t)pParamTable + sizeof(ParamTable_t), (uint32_t*)&currCrc, sizeof(uint32_t));
    NRF_LOG_INFO("ParamTabCrc = 0x%08x     currCrc = 0x%08x",ParamTabCrc,currCrc);
    if(ParamTabCrc != currCrc){
        NRF_LOG_INFO("update ParamTable");
        main_status.ParamTab_change_req = REQ_CHNGE;
        WriteParamTab();
    }else{
      NRF_LOG_INFO("initParamTab - ParamTable is good");
    }
    NRF_LOG_INFO("initParamTab() end");
#endif
}
/************************ RTC ****************/

static void incr_date_time(DateTime_t* pDateTime)
{
  uint32_t february_len;
  
  if(main_status.DateTime_change_req){
    ((uint32_t*)pDateTime)[0] = NewDateTime;
    main_status.DateTime_change_req = 0;
  }

  if(pDateTime->tm_sec >= SECOND_PER_MINUTE-1){
    pDateTime->tm_sec = 0;
    pDateTime->tm_min++;
  }
  else{
    pDateTime->tm_sec++;
    return;
  }
  if(pDateTime->tm_min >= MINUTES_PER_HOUR){
      pDateTime->tm_min = 0;
    pDateTime->tm_hour++;
  }
  else{
    return;
  }
  if(pDateTime->tm_hour >= HOURS_PER_DAY){
    pDateTime->tm_hour = 0;
    pDateTime->tm_mday++;
  }
  else{
    return;
  }
  if(pDateTime->tm_mon == FEBRUARY){
    february_len = (pDateTime->tm_year & 0x03)? FEBRUARY_NO_LEAP_YEAR_LEN : FEBRUARY_LEAP_YEAR_LEN;     
    if(pDateTime->tm_mday >= february_len){
      pDateTime->tm_mday = 0;
      pDateTime->tm_mon++;
    }
    else{
      return;
    }
  }
  if((pDateTime->tm_mon == JANUARY) ||
     (pDateTime->tm_mon == MARCH) ||
     (pDateTime->tm_mon == MAY) ||
     (pDateTime->tm_mon == JULY) ||
     (pDateTime->tm_mon == AUGUST) ||
     (pDateTime->tm_mon == OCTOBER) ||
     (pDateTime->tm_mon == DECEMBER)){

    if(pDateTime->tm_mday >= LONG_MONTH_LEN){
      pDateTime->tm_mday = 0;
      if(pDateTime->tm_mon == DECEMBER){
        pDateTime->tm_mon == JANUARY;
        pDateTime->tm_year++;
        return;
      }
      else{
        pDateTime->tm_mon++;
      }
    }
  }
}

static void GetCaseState(void)
{
  get_current_status();
  if(device_status.DEVSTAT_STATE_OF_SW_1 == 0){
    CaseState.CurrentCaseState = CASE_UNLOCK;
  }else if(device_status.DEVSTAT_STATE_OF_SW_2 == 0){
    CaseState.CurrentCaseState = CASE_HANDEL_OPEN;
  }else if(device_status.DEVSTAT_STATE_OF_SW_3 == 0){
    CaseState.CurrentCaseState = CASE_LOCK;
  }else{
    CaseState.CurrentCaseState = CASE_UNRESOLVED;
  }
}

/** @brief: Function for handling the RTC0 interrupts.
 * Triggered on TICK and COMPARE0 match.
 */
static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    static uint32_t RTC_cntr_sec = 0;
    static uint32_t RTC_cntr_tick = 0;
    uint32_t err_code;
    if (int_type == NRF_DRV_RTC_INT_COMPARE0){ // one second event
//        if(motorTimeout)
//          NRF_LOG_INFO("MotorTimeout: %d",motorTimeout); 
        RTC_cntr_sec++;
        RTC_cntr_tick = 0;
        err_code = nrf_drv_rtc_cc_set(&rtc,0,nrfx_rtc_counter_get(&rtc) + TICK_PER_SEC,true);
        APP_ERROR_CHECK(err_code);
        incr_date_time(&CurrentDateTime.AsStruct);
        WriteParamTab();
        LockSwitchEvent_handler();
        StoreDevLog();
                   
        if(((RTC_cntr_sec & 0x0F)==0)&&(main_status.FlashBuzy==NO)){ // measure Battery Voltage every 16 second
          main_status.FlashBuzy = YES;
//          NRF_LOG_INFO("Current Time = 0x%08x",CurrentDateTime.AsInt);
          saadc_init();
          err_code = nrfx_saadc_sample();
          APP_ERROR_CHECK(err_code);

          TickRetries_16s();    
        }
        if(CaseStateLedOnTime > 1){
          CaseStateLedOnTime--;
        }else if(CaseStateLedOnTime == 1){
          CaseStateLedOnTime = 0;
          *(uint32_t*)&led_control = 0;   
          nrf_gpio_pin_clear(LED_R_PIN);
          nrf_gpio_pin_clear(LED_G_PIN);
          nrf_gpio_pin_clear(LED_B_PIN);
          rtcTickRequest.led_bilnk = 0;
        }

        if(led_control.LED_BLUE == LED_BLINK){
          nrf_gpio_pin_set(LED_B_PIN);
        }
        if(led_control.LED_GREEN == LED_BLINK){
          nrf_gpio_pin_set(LED_G_PIN);
        }
        if(led_control.LED_RED == LED_BLINK){
          nrf_gpio_pin_set(LED_R_PIN);
        }
        if(main_status.MotorPowerOffReq == 2){
          main_status.MotorPowerOffReq--;
        }else if(main_status.MotorPowerOffReq == 1){
          MOTOR_POWER_OFF();
        }
        if((main_status.FlashErase_req)&&(!nrf_fstorage_is_busy(&fstorage)))
        {
          uint32_t BlocksNo = (fstorage.end_addr - fstorage.start_addr)/PAGE_SIZE - 1;
          memset(&log_event,0,sizeof(log_event_store_t));
          int_flash_erase((uint32_t)pParamTable + PAGE_SIZE,BlocksNo);
        }
    }
    else if (int_type == NRF_DRV_RTC_INT_TICK)
    {
        RTC_cntr_tick++;
        if(RTC_cntr_tick == LED_BLINK_ON_TIME_TICK){
          if(led_control.LED_BLUE == LED_BLINK){
            nrf_gpio_pin_clear(LED_B_PIN);
          }
          if(led_control.LED_GREEN == LED_BLINK){
            nrf_gpio_pin_clear(LED_G_PIN);
          }
          if(led_control.LED_RED == LED_BLINK){
            nrf_gpio_pin_clear(LED_R_PIN);
          }
        }
        if(CaseState.change_case_state_req != CASE_STATE_REQ_IDLE){
          if(motorTimeout > RTC_TICK_TIME){
            motorTimeout -= RTC_TICK_TIME;
            if((RTC_cntr_tick&0x01)&&(main_status.FlashBuzy==0)){ // SAADC
              main_status.FlashBuzy = YES;
              saadc_init();
              err_code = nrfx_saadc_sample();
              APP_ERROR_CHECK(err_code);
            }
          }else{
            uint8_t param0;   // requered case state
            uint8_t param1;   // current case state
            GetCaseState();
            switch(CaseState.change_case_state_req){
              case CASE_STATE_REQ_LOOK:
                param0 = CASE_LOCK;
                break;
              case CASE_STATE_REQ_UNLOOK:
                param0 = CASE_UNLOCK;
                break;
              case CASE_STATE_REQ_HANDEL_OPEN:
                param0 = CASE_HANDEL_OPEN;
                break;
            }
            param1 = CaseState.CurrentCaseState;
            CaseStateLedOnTime = CASE_STATE_ERROR_LED_BLINK_TIME;
            logEventStorageReq(LOG_EVENT_CASE_STATE_TIMEOUT,param0,param1,0);
            SetLedControl(LED_BLINK,LED_BLINK,LED_OFF);
            stop_motor();
          }
        }
    }
}

/** @brief Function initialization and configuration of RTC driver instance.
 */
static void rtc_config(void)
{
    uint32_t err_code;

    //Initialize RTC instance
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    config.prescaler = 4095;    // 32768/4096 = 8 tiks/sec
    err_code = nrf_drv_rtc_init(&rtc, &config, rtc_handler);
    APP_ERROR_CHECK(err_code);

    //Enable tick event & interrupt
    nrf_drv_rtc_tick_enable(&rtc,true);

    //Set compare channel to trigger interrupt after 1 seconds
    err_code = nrf_drv_rtc_cc_set(&rtc,0,TICK_PER_SEC,true);
    APP_ERROR_CHECK(err_code);

    //Power on RTC instance
    nrf_drv_rtc_enable(&rtc);
}

/** @brief Function starting the internal LFCLK XTAL oscillator.
 */
static void lfclk_config(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);

    // Wait for the clock to be ready.
    while (!nrf_clock_lf_is_running()) {;}
}

void init_periferal(void)
{
  ret_code_t rc;
  nrf_fstorage_api_t * p_fs_api;
  NRF_POWER->DCDCEN = 1;          //Enabling the DCDC converter
  lfclk_config();                 //initialization clock 32768 
  rtc_config();
  gpio_init();
  twi_init();                     //initialization i2c
  p_fs_api = &nrf_fstorage_sd;
  rc = nrf_fstorage_init(&fstorage, p_fs_api, NULL);
  APP_ERROR_CHECK(rc);
  int_flash_read((uint32_t)pParamTable, (uint32_t*)&ParamTab, sizeof(ParamTable_t));
  print_flash_info(&fstorage);
  initParamTab();
  ReportAddr = find_free_addr((uint32_t)pParamTable + PAGE_SIZE);
  lsensor_init();
  memset(&log_event,0,sizeof(log_event_store_t));
  GetCaseState();
  CaseState.LastTrueCaseState = CaseState.CurrentCaseState;
//  get_current_status();
//  if(device_status.DEVSTAT_STATE_OF_SW_1 == 0){
//    main_status.CaseState = CASE_UNLOCK;
//  }else if(device_status.DEVSTAT_STATE_OF_SW_2 == 0){
//    main_status.CaseState = CASE_HANDEL_OPEN;
//  }else if(device_status.DEVSTAT_STATE_OF_SW_3 == 0){
//    main_status.CaseState = CASE_LOCK;
//  }else{
//    main_status.CaseState = CASE_UNRESOLVED;
//  }
}