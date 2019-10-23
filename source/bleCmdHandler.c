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

uint16_t sw_rewision  = 0x0100;
ParamTable_t  NewParamTable;



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
  if(main_status.change_case_state_req != CASE_STATE_REQ_IDLE){
    return ERR_BLE_MODULE_BUZY;
  }
  get_current_status();
//  uint32_t sw_state = device_status.AsInt & 0x07;
  switch(CaseState){
    case CASE_LOCK:
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
    case CASE_MANUAL:
      if(device_status.DEVSTAT_STATE_OF_SW_2){ // case isn't manual
        if(device_status.DEVSTAT_STATE_OF_SW_3){  // case is looked
          MOTOR_OPEN_CASE();
          motorTimeout = ParamTab.MotorActiveTime.MOTOR_CW_HALF_TIME_MS;
          main_status.change_case_state_req = CASE_STATE_REQ_MANUAL;
          rtcTickRequest.motor_buzy = 1;
          nrf_drv_rtc_tick_enable(&rtc,true);
        }else{
          MOTOR_CLOSE_CASE();
          motorTimeout = ParamTab.MotorActiveTime.MOTOR_CCW_HALF_TIME_MS;
          main_status.change_case_state_req = CASE_STATE_REQ_MANUAL;
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

RESULT  bleSetMotorTimes(motor_active_time_t MotorTimes)
{
  uint32_t addr;
  if((pParamTable->MotorActiveTime.MOTOR_CCW_FULL_TIME_MS != MotorTimes.MOTOR_CCW_FULL_TIME_MS)||
     (pParamTable->MotorActiveTime.MOTOR_CCW_HALF_TIME_MS  != MotorTimes.MOTOR_CCW_HALF_TIME_MS)||
     (pParamTable->MotorActiveTime.MOTOR_CW_FULL_TIME_MS  != MotorTimes.MOTOR_CW_FULL_TIME_MS)||
     (pParamTable->MotorActiveTime.MOTOR_CW_HALF_TIME_MS  != MotorTimes.MOTOR_CW_HALF_TIME_MS))
  {
    if(main_status.ParamTab_change_req != REQ_NONE){
      return ERR_BLE_MODULE_BUZY;
    }
    memcpy(&NewParamTable,pParamTable,sizeof(ParamTable_t));
    NewParamTable.MotorActiveTime.MOTOR_CCW_FULL_TIME_MS = MotorTimes.MOTOR_CCW_FULL_TIME_MS;
    NewParamTable.MotorActiveTime.MOTOR_CCW_HALF_TIME_MS  = MotorTimes.MOTOR_CCW_HALF_TIME_MS;
    NewParamTable.MotorActiveTime.MOTOR_CW_FULL_TIME_MS = MotorTimes.MOTOR_CW_FULL_TIME_MS;
    NewParamTable.MotorActiveTime.MOTOR_CW_HALF_TIME_MS  = MotorTimes.MOTOR_CW_HALF_TIME_MS;
    main_status.ParamTab_change_req = REQ;
  }
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
      return ERR_BLE_MODULE_BUZY;
    }
    memcpy(&NewParamTable,pParamTable,sizeof(ParamTable_t));
    NewParamTable.BatteryAlarmLevel = BatLevel;
    main_status.ParamTab_change_req = REQ;
  }
  return ERR_NO;
}

RESULT bleSetLIghtAlarmLevel(uint32_t LightAlarm)
{
  uint32_t addr;
  if((pParamTable->lsensor.upper_thresh_low != LightAlarm & 0xFF)||
     (pParamTable->lsensor.upper_thresh_hight != ((LightAlarm >> 8)&0xFF))){
    if(main_status.ParamTab_change_req != REQ_NONE){
      return ERR_BLE_MODULE_BUZY;
    }
    memcpy(&NewParamTable,pParamTable,sizeof(ParamTable_t));
    NewParamTable.lsensor.upper_thresh_hight = (LightAlarm >> 8)&0xFF;
    NewParamTable.lsensor.upper_thresh_low = LightAlarm & 0xFF;
    main_status.ParamTab_change_req = REQ;
  }
  return ERR_NO;
}

//RESULT bleFlashErase(uint32_t FlashAddress, size_t BlocksNo)
//{
//  int_flash_erase(FlashAddress, BlocksNo);
//  return ERR_NO;
//}