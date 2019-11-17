#include "bleCmdHandler.h"

extern const ParamTable_t*  pParamTable; 
extern ParamTable_t  ParamTab;

extern main_status_t main_status;
extern uint32_t NewDateTime;
extern led_control_t led_control;
extern const nrf_drv_rtc_t rtc;
extern device_status_t device_status;
extern uint32_t motorTimeout;
extern rtc_tick_enable_t rtcTickRequest;
extern uint32_t BatVoltage;

uint16_t sw_rewision  = 0x0104;
uint8_t  NewParamTable[sizeof(ParamTable_t) + sizeof(uint32_t)] = {DEF_PARAM_TAB};

RESULT bleGetDeviceInfo(DEVICE_INFO* pDeviceInfo)
{
  pDeviceInfo->SW_VERSION_MINOR = sw_rewision&0xFF;
  pDeviceInfo->SW_VERSION_MAJOR = sw_rewision>>8;
  pDeviceInfo->HW_VERSION_MINOR = pParamTable->HW_revision&0xFF;
  pDeviceInfo->HW_VERSION_MAJOR = pParamTable->HW_revision>>8;
  return ERR_NO;
}

RESULT bleSetCaseState(CASE_STATE CaseState)
{
  NRF_LOG_INFO("bleSetCaseState   0x%02x ",(uint8_t)CaseState);
  if(main_status.change_case_state_req != CASE_STATE_REQ_IDLE){
    NRF_LOG_INFO("ERR_MODULE_BUZY");
    return ERR_MODULE_BUZY;
  }
  get_current_status();
  switch(CaseState){
    case CASE_LOCK:
      if(device_status.DEVSTAT_POWER_LOW == YES){
        NRF_LOG_INFO("LOW BATTERY");
        return ERR_LOW_BATTERY;
      }
      if(device_status.DEVSTAT_STATE_OF_SW_3){ // case isn't looked
        MOTOR_CLOSE_CASE();
        motorTimeout = (device_status.DEVSTAT_STATE_OF_SW_2 == 0)? ParamTab.MotorActiveTime.MOTOR_CCW_HALF_TIME_MS:
                                                                  ParamTab.MotorActiveTime.MOTOR_CCW_FULL_TIME_MS;
        main_status.change_case_state_req = CASE_STATE_REQ_LOOK;
        rtcTickRequest.motor_buzy = 1;
        nrf_drv_rtc_tick_enable(&rtc,true);
      }
      break;
    case CASE_UNLOCK:
      if(device_status.DEVSTAT_STATE_OF_SW_1){ // case isn't unlooked
        MOTOR_OPEN_CASE();
        motorTimeout = (device_status.DEVSTAT_STATE_OF_SW_2)? ParamTab.MotorActiveTime.MOTOR_CW_HALF_TIME_MS:
                                                                  ParamTab.MotorActiveTime.MOTOR_CW_FULL_TIME_MS;
        main_status.change_case_state_req = CASE_STATE_REQ_UNLOOK;
        rtcTickRequest.motor_buzy = 1;
        nrf_drv_rtc_tick_enable(&rtc,true);
      }
      break;
    case CASE_HANDEL_OPEN:
      if(device_status.DEVSTAT_STATE_OF_SW_2){ // case isn't manual
        if(device_status.DEVSTAT_STATE_OF_SW_3){  // case is looked
          if(device_status.DEVSTAT_POWER_LOW == YES){
            NRF_LOG_INFO("LOW BATTERY");
            return ERR_LOW_BATTERY;
          }
          MOTOR_CLOSE_CASE();
          motorTimeout = ParamTab.MotorActiveTime.MOTOR_CW_HALF_TIME_MS;
          main_status.change_case_state_req = CASE_STATE_REQ_HANDEL_OPEN;
          rtcTickRequest.motor_buzy = 1;
          nrf_drv_rtc_tick_enable(&rtc,true);
        }else{
          MOTOR_OPEN_CASE();
          motorTimeout = ParamTab.MotorActiveTime.MOTOR_CCW_HALF_TIME_MS;
          main_status.change_case_state_req = CASE_STATE_REQ_HANDEL_OPEN;
          rtcTickRequest.motor_buzy = 1;
          nrf_drv_rtc_tick_enable(&rtc,true);
        }
      }
      break;
    default:
      return ERR_BLE_PARAM;
  }
  return ERR_NO;
}

RESULT bleSetLedState(LED_CONTROL LedControl)
{
  led_control = LedControl;
  uint32_t pin_state = (LED_R_PIN,LedControl.LED_RED == LED_ON)? 1:0;
  nrf_gpio_pin_write(LED_R_PIN,pin_state);
  pin_state = (LED_R_PIN,LedControl.LED_GREEN == LED_ON)? 1:0;
  nrf_gpio_pin_write(LED_G_PIN,pin_state);
  pin_state = (LED_R_PIN,LedControl.LED_BLUE == LED_ON)? 1:0;
  nrf_gpio_pin_write(LED_B_PIN,pin_state);
  if((led_control.LED_BLUE == LED_BLINK)||
     (led_control.LED_GREEN == LED_BLINK)||
     (led_control.LED_RED == LED_BLINK)){
     //Enable tick event & interrupt
    rtcTickRequest.led_bilnk = 1;
    nrf_drv_rtc_tick_enable(&rtc,true);
  } 
  else{
    rtcTickRequest.led_bilnk = 0;
    if(*(uint32_t*)&rtcTickRequest == 0)
      nrf_drv_rtc_tick_disable(&rtc);	
  }
   
  return ERR_NO;
}

RESULT  bleSetMotorTimes(MOTOR_ACTIVE_TIME MotorTimes)
{
  uint32_t addr;
  motor_active_time_t    MotorActiveTime;

  NRF_LOG_INFO("sent MotorTimes = %d  %d   %d  %d",MotorTimes.MOTOR_CCW_FULL_TIME_MS,
                                              MotorTimes.MOTOR_CCW_HALF_TIME_MS,
                                              MotorTimes.MOTOR_CW_FULL_TIME_MS,
                                              MotorTimes.MOTOR_CW_HALF_TIME_MS);

  if((pParamTable->MotorActiveTime.MOTOR_CCW_FULL_TIME_MS != MotorTimes.MOTOR_CCW_FULL_TIME_MS)||
     (pParamTable->MotorActiveTime.MOTOR_CCW_HALF_TIME_MS  != MotorTimes.MOTOR_CCW_HALF_TIME_MS)||
     (pParamTable->MotorActiveTime.MOTOR_CW_FULL_TIME_MS  != MotorTimes.MOTOR_CW_FULL_TIME_MS)||
     (pParamTable->MotorActiveTime.MOTOR_CW_HALF_TIME_MS  != MotorTimes.MOTOR_CW_HALF_TIME_MS))
  {

    if(main_status.ParamTab_change_req != REQ_NONE){
      return ERR_MODULE_BUZY;
    }
    int_flash_read((uint32_t)pParamTable, (uint32_t*)&NewParamTable, sizeof(ParamTable_t));
    MotorActiveTime.MOTOR_CCW_FULL_TIME_MS = MotorTimes.MOTOR_CCW_FULL_TIME_MS;
    MotorActiveTime.MOTOR_CCW_HALF_TIME_MS  = MotorTimes.MOTOR_CCW_HALF_TIME_MS;
    MotorActiveTime.MOTOR_CW_FULL_TIME_MS = MotorTimes.MOTOR_CW_FULL_TIME_MS;
    MotorActiveTime.MOTOR_CW_HALF_TIME_MS  = MotorTimes.MOTOR_CW_HALF_TIME_MS;
    memcpy(NewParamTable + MOTOR_ACTIVE_TIME_OFFSET,&MotorActiveTime,sizeof(motor_active_time_t));
    main_status.ParamTab_change_req = REQ_CHNGE;
  }
  
  NRF_LOG_INFO("MotorActiveTime = %d  %d   %d  %d",MotorActiveTime.MOTOR_CCW_FULL_TIME_MS,
                                                    MotorActiveTime.MOTOR_CCW_HALF_TIME_MS,
                                                    MotorActiveTime.MOTOR_CW_FULL_TIME_MS,
                                                    MotorActiveTime.MOTOR_CW_HALF_TIME_MS);
  return ERR_NO;
}

RESULT bleSetRtcTime(uint32_t DateTime)
{
  main_status.DateTime_change_req = 1;
  NewDateTime = DateTime;
  return ERR_NO;
}

RESULT bleSetBatteryAlarmLevel(uint32_t BatLevel)
{
  uint32_t addr;
  if(pParamTable->BatteryAlarmLevel != BatLevel){
    if(main_status.ParamTab_change_req != REQ_NONE){
      return ERR_MODULE_BUZY;
    }
    int_flash_read((uint32_t)pParamTable, (uint32_t*)&NewParamTable, sizeof(ParamTable_t));
    memcpy(NewParamTable+BAT_ALARM_LEVEL_OFFSET,&BatLevel,sizeof(BatLevel));
    main_status.ParamTab_change_req = REQ_CHNGE;
  }
  return ERR_NO;
}

RESULT bleSetLightAlarmLevel(uint16_t LightAlarm)
{
  uint32_t addr;
  if((pParamTable->lsensor.upper_thresh_low != LightAlarm & 0xFF)||
     (pParamTable->lsensor.upper_thresh_hight != ((LightAlarm >> 8)&0xFF))){
    if(main_status.ParamTab_change_req != REQ_NONE){
      return ERR_MODULE_BUZY;
    }
    int_flash_read((uint32_t)pParamTable, (uint32_t*)&NewParamTable, sizeof(ParamTable_t));
    memcpy(NewParamTable+LSENS_LOWER_THRESH_OFFSET,&LightAlarm,sizeof(LightAlarm));
    main_status.ParamTab_change_req = REQ_CHNGE;
  }
  return ERR_NO;
}

RESULT bleFlashLogErase(void)
{
  main_status.FlashErase_req = 1;
  return ERR_NO;
}

RESULT bleGetBatteryVoltage(uint16_t* BatteryVoltage)
{
  *BatteryVoltage = (uint16_t)BatVoltage;
}

RESULT bleSetHardwareVersion(HARDWARE_VERSION HardwareVersion) 
{
    int_flash_read((uint32_t)pParamTable, (uint32_t*)&NewParamTable, sizeof(ParamTable_t));
    memcpy(NewParamTable+HW_REVISEON_OFFSET,&HardwareVersion,sizeof(HardwareVersion));
    main_status.ParamTab_change_req = REQ_CHNGE;
}

RESULT bleGetDeviceStatus(DEVICE_STATUS* tDeviceStatus)
{
  get_current_status();
  tDeviceStatus->STATE_OF_SW_1            = device_status.DEVSTAT_STATE_OF_SW_1;
  tDeviceStatus->STATE_OF_SW_2            = device_status.DEVSTAT_STATE_OF_SW_2;
  tDeviceStatus->STATE_OF_SW_3            = device_status.DEVSTAT_STATE_OF_SW_3;
  tDeviceStatus->WIRE_PIN                 = device_status.DEVSTAT_WIRE_PIN;
  tDeviceStatus->LIGHT_PENETRATION        = device_status.DEVSTAT_LIGHT_PENETRATION;
  tDeviceStatus->POWER_LOW                = device_status.DEVSTAT_POWER_LOW;
  tDeviceStatus->FLASH_LOG_FULL           = device_status.DEVSTAT_FLASH_LOG_FULL;
  return ERR_NO;
}

void bleShowParamTab(void)
{
  uint8_t  ParamTable[sizeof(ParamTable_t) + sizeof(uint32_t)];
  int_flash_read((uint32_t) pParamTable, (uint32_t*)ParamTable, sizeof(ParamTable));
  NRF_LOG_INFO("MOTOR_ACTIVE_TIME CW_FULL    0x%04x ",*(uint16_t*)(ParamTable + MOTOR_ACTIVE_TIME_OFFSET));
  NRF_LOG_INFO("MOTOR_ACTIVE_TIME CW_HALF    0x%04x ",*(uint16_t*)(ParamTable + MOTOR_ACTIVE_TIME_OFFSET+2));
  NRF_LOG_INFO("MOTOR_ACTIVE_TIME CCW_FULL   0x%04x ",*(uint16_t*)(ParamTable+MOTOR_ACTIVE_TIME_OFFSET+4));
  NRF_LOG_INFO("MOTOR_ACTIVE_TIME CCW_HALF   0x%04x ",*(uint16_t*)(ParamTable+MOTOR_ACTIVE_TIME_OFFSET+6));
  NRF_LOG_INFO("BAT_ALARM_LEVEL              0x%04x ",*(uint16_t*)(ParamTable+BAT_ALARM_LEVEL_OFFSET));
  NRF_LOG_INFO("HW_REVISEON                  0x%08x ",*(uint32_t*)(ParamTable+HW_REVISEON_OFFSET));
  NRF_LOG_INFO("LSENS_LOWER_THRESH           0x%04x ",*(uint16_t*)(ParamTable+LSENS_LOWER_THRESH_OFFSET));
}