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
extern case_state_t  CaseState;

uint16_t sw_rewision  = 0x0106;
uint8_t  NewParamTable[sizeof(ParamTable_t) + sizeof(uint32_t)] = {DEF_PARAM_TAB};

RESULT bleGetDeviceInfo(DEVICE_INFO* pDeviceInfo)
{
  pDeviceInfo->SW_VERSION_MINOR = sw_rewision&0xFF;
  pDeviceInfo->SW_VERSION_MAJOR = sw_rewision>>8;
  pDeviceInfo->HW_VERSION_MINOR = pParamTable->HW_revision&0xFF;
  pDeviceInfo->HW_VERSION_MAJOR = pParamTable->HW_revision>>8;
  return ERR_NO;
}

RESULT bleSetCaseState(CASE_STATE ReqCaseState)
{
  NRF_LOG_INFO("bleSetCaseState   0x%02x ",(uint8_t)ReqCaseState);
  
  if(CaseState.change_case_state_req != CASE_STATE_REQ_IDLE){
    NRF_LOG_INFO("ERR_MODULE_BUZY");
    return ERR_MODULE_BUZY;
  }
  switch(ReqCaseState){
    case CASE_LOCK:
      if(CaseState.LookStateDes){
        NRF_LOG_INFO("SetCaseState Error. LOCK state desabled");
        return ERR_CASE_STATE;
      }
      if(device_status.DEVSTAT_POWER_LOW == YES){
        NRF_LOG_INFO("LOW BATTERY");
        return ERR_LOW_BATTERY;
      }
      if(CaseState.CurrentCaseState != CASE_LOCK){ //case isn't looked
        MOTOR_CLOSE_CASE();
        motorTimeout = (device_status.DEVSTAT_STATE_OF_SW_2 == 0)? pParamTable->MotorActiveTime.MOTOR_CCW_HALF_TIME_MS:
                                                                  pParamTable->MotorActiveTime.MOTOR_CCW_FULL_TIME_MS;
        CaseState.change_case_state_req = CASE_STATE_REQ_LOOK;
        rtcTickRequest.motor_buzy = 1;
        nrf_drv_rtc_tick_enable(&rtc,true);
      }
      break;
    case CASE_UNLOCK:
      if(CaseState.UnloockStateDes){
        NRF_LOG_INFO("SetCaseState Error. UNLOCK state desabled");
        return ERR_CASE_STATE;
      }
      if(CaseState.CurrentCaseState != CASE_UNLOCK){ //case isn't unlooked
        MOTOR_OPEN_CASE();
        motorTimeout = (device_status.DEVSTAT_STATE_OF_SW_2)? pParamTable->MotorActiveTime.MOTOR_CW_HALF_TIME_MS:
                                                                  pParamTable->MotorActiveTime.MOTOR_CW_FULL_TIME_MS;
        CaseState.change_case_state_req = CASE_STATE_REQ_UNLOOK;
        rtcTickRequest.motor_buzy = 1;
        nrf_drv_rtc_tick_enable(&rtc,true);
      }
      break;
    case CASE_HANDEL_OPEN:
      if(CaseState.MidleStateDes){
        NRF_LOG_INFO("SetCaseState Error. HANDEL_OPEN state desabled");
        return ERR_CASE_STATE;
      }
      if(CaseState.CurrentCaseState != CASE_HANDEL_OPEN){ //case was UNLOCK
 
        if(CaseState.LastTrueCaseState == CASE_UNLOCK){ //case was UNLOCK
          if(device_status.DEVSTAT_POWER_LOW == YES){
            NRF_LOG_INFO("LOW BATTERY");
            return ERR_LOW_BATTERY;
          }
          MOTOR_CLOSE_CASE();
          motorTimeout = pParamTable->MotorActiveTime.MOTOR_CW_HALF_TIME_MS;
          CaseState.change_case_state_req = CASE_STATE_REQ_HANDEL_OPEN;
          rtcTickRequest.motor_buzy = 1;
          nrf_drv_rtc_tick_enable(&rtc,true);
        }
        else {
          MOTOR_OPEN_CASE();
          motorTimeout = pParamTable->MotorActiveTime.MOTOR_CCW_HALF_TIME_MS;
          CaseState.change_case_state_req = CASE_STATE_REQ_HANDEL_OPEN;
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
  motor_active_time_t    MotorActiveTime;

  NRF_LOG_INFO("sent MotorTimes = 0x%02x  0x%02x   0x%02x  0x%02x",
                                              MotorTimes.MOTOR_CCW_FULL_TIME_MS,
                                              MotorTimes.MOTOR_CCW_HALF_TIME_MS,
                                              MotorTimes.MOTOR_CW_FULL_TIME_MS,
                                              MotorTimes.MOTOR_CW_HALF_TIME_MS);

  if((pParamTable->MotorActiveTime.MOTOR_CCW_FULL_TIME_MS  != MotorTimes.MOTOR_CCW_FULL_TIME_MS)||
     (pParamTable->MotorActiveTime.MOTOR_CCW_HALF_TIME_MS  != MotorTimes.MOTOR_CCW_HALF_TIME_MS)||
     (pParamTable->MotorActiveTime.MOTOR_CW_FULL_TIME_MS   != MotorTimes.MOTOR_CW_FULL_TIME_MS)||
     (pParamTable->MotorActiveTime.MOTOR_CW_HALF_TIME_MS   != MotorTimes.MOTOR_CW_HALF_TIME_MS))
  {

    if(main_status.ParamTab_change_req != REQ_NONE){
      return ERR_MODULE_BUZY;
    }
    int_flash_read((uint32_t)pParamTable, (uint32_t*)&NewParamTable, sizeof(ParamTable_t));
    MotorActiveTime.MOTOR_CCW_FULL_TIME_MS  = MotorTimes.MOTOR_CCW_FULL_TIME_MS;
    MotorActiveTime.MOTOR_CCW_HALF_TIME_MS  = MotorTimes.MOTOR_CCW_HALF_TIME_MS;
    MotorActiveTime.MOTOR_CW_FULL_TIME_MS   = MotorTimes.MOTOR_CW_FULL_TIME_MS;
    MotorActiveTime.MOTOR_CW_HALF_TIME_MS   = MotorTimes.MOTOR_CW_HALF_TIME_MS;
    memcpy(NewParamTable + PAR_TAB_MOTOR_ACTIVE_TIME_OFFSET,&MotorActiveTime,sizeof(motor_active_time_t));
    main_status.ParamTab_change_req = REQ_CHNGE;
  }
  NRF_LOG_INFO("MotorActiveTime = 0x%02x  0x%02x   0x%02x  0x%02x", 
                                                    MotorActiveTime.MOTOR_CCW_FULL_TIME_MS,
                                                    MotorActiveTime.MOTOR_CCW_HALF_TIME_MS,
                                                    MotorActiveTime.MOTOR_CW_FULL_TIME_MS,
                                                    MotorActiveTime.MOTOR_CW_HALF_TIME_MS);
  return ERR_NO;
}

RESULT bleSetRtcTime(uint32_t DateTime)
{
  main_status.DateTime_change_req = 1;
  NewDateTime = DateTime;
  NRF_LOG_INFO("NewDateTime: 0x%08x",NewDateTime);
  return ERR_NO;
}

RESULT bleSetBatteryAlarmLevel(uint32_t BatLevel)
{
  uint32_t addr;
  NRF_LOG_INFO("Curr BatteryAlarmLevel = 0x%04x",pParamTable->BatteryAlarmLevel);
  if(pParamTable->BatteryAlarmLevel != BatLevel){
    if(main_status.ParamTab_change_req != REQ_NONE){
      return ERR_MODULE_BUZY;
    }
    int_flash_read((uint32_t)pParamTable, (uint32_t*)&NewParamTable, sizeof(ParamTable_t));
    memcpy(NewParamTable+PAR_TAB_BAT_ALARM_LEVEL_OFFSET,&BatLevel,sizeof(BatLevel));
    main_status.ParamTab_change_req = REQ_CHNGE;
  }
  return ERR_NO;
}

RESULT bleSetLightAlarmLevel(uint16_t LightAlarm)
{
  uint32_t addr;
  if((pParamTable->lsensor.lower_thresh_low != LightAlarm & 0xFF)||
     (pParamTable->lsensor.lower_thresh_hight != ((LightAlarm >> 8)&0xFF))){
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
  NRF_LOG_INFO("bleFlashLogErase");
  NRF_LOG_FLUSH();
  main_status.FlashErase_req = 1;
  return ERR_NO;
}

RESULT bleGetBatteryVoltage(uint16_t* BatteryVoltage)
{
  *BatteryVoltage = (uint16_t)BatVoltage;
  return ERR_NO;
}

RESULT bleSetHardwareVersion(HARDWARE_VERSION HardwareVersion) 
{
  int_flash_read((uint32_t)pParamTable, (uint32_t*)&NewParamTable, sizeof(ParamTable_t));
  memcpy(NewParamTable+HW_REVISEON_OFFSET,&HardwareVersion,sizeof(HardwareVersion));
  main_status.ParamTab_change_req = REQ_CHNGE;
  return ERR_NO;
}
RESULT bleSetNumberRetries(uint8_t NumberRetries) 
{
  uint32_t NumRetries = NumberRetries;
  int_flash_read((uint32_t)pParamTable, (uint32_t*)&NewParamTable, sizeof(ParamTable_t));
  memcpy(NewParamTable+PAR_TAB_NUM_RETR_OFFSET,&NumRetries,sizeof(NumRetries));
  main_status.ParamTab_change_req = REQ_CHNGE;
  return ERR_NO;
}

uint8_t bleGetNumberRetries(void)
{
  return (uint8_t)pParamTable->NumberRetries;
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
/*This function created for debugging. */
  uint8_t  ParamTable[sizeof(ParamTable_t) + sizeof(uint32_t)];
  NRF_LOG_INFO("bleShowParamTab:  BAT_ALARM_LEVEL              0x%08x ",pParamTable->BatteryAlarmLevel);

  int_flash_read((uint32_t) pParamTable, (uint32_t*)ParamTable, sizeof(ParamTable));
  NRF_LOG_INFO("LIGHT_AL_LEV            0x%04x ",*(uint16_t*)(ParamTable + LSENS_LOWER_THRESH_OFFSET));
  NRF_LOG_INFO("MOTOR_ACT_TIME CW_F    0x%04x ",*(uint16_t*)(ParamTable + PAR_TAB_MOTOR_ACTIVE_TIME_OFFSET));
  NRF_LOG_INFO("MOTOR_ACT_TIME CW_H    0x%04x ",*(uint16_t*)(ParamTable + PAR_TAB_MOTOR_ACTIVE_TIME_OFFSET+2));
  NRF_LOG_INFO("MOTOR_ACT_TIME CCW_F   0x%04x ",*(uint16_t*)(ParamTable+PAR_TAB_MOTOR_ACTIVE_TIME_OFFSET+4));
  NRF_LOG_INFO("MOTOR_ACT_TIME CCW_H   0x%04x ",*(uint16_t*)(ParamTable+PAR_TAB_MOTOR_ACTIVE_TIME_OFFSET+6));
  NRF_LOG_INFO("BAT_AL_LEVEL              0x%04x ",*(uint16_t*)(ParamTable+PAR_TAB_BAT_ALARM_LEVEL_OFFSET));
  NRF_LOG_FLUSH();
  NRF_LOG_INFO("HW_REV                  0x%04x ",*(uint32_t*)(ParamTable+HW_REVISEON_OFFSET));
  NRF_LOG_INFO("NUMBER_RETR               0x%02x ",*(uint16_t*)(ParamTable+PAR_TAB_NUM_RETR_OFFSET));
}

RESULT bleFlashLogRead(uint32_t Offset, uint16_t DataLength, uint32_t* Data, uint16_t* DataLengthRet)
{
  uint8_t i;
  DataLengthRet[0] = DataLength;
  int_flash_read(LOG_EVENT_TAB_START_ADDR+Offset,Data,DataLength);
  for(i = 0;i<DataLength/4;i++){
    if(Data[i] == 0xFFFFFFFF){
      DataLengthRet[0] = i*4;   
      break;
    }
  }
  NRF_LOG_INFO("FlashLogRead:");
  NRF_LOG_INFO("Offset = 0x%06x  Req DataLength = 0x%04x   DataLengthRet = 0x%04x ",Offset,DataLength,DataLengthRet[0]);
  NRF_LOG_INFO("Data[0] = 0x%08x  Data[1] = 0x%08x  Data[2] = 0x%08x  Data[3] = 0x%08x",Data[0],Data[1],Data[2],Data[3]);

  return ERR_NO;
}