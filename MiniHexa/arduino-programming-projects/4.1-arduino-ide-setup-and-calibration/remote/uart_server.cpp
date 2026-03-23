#include "uart_server.h"

String rec_data[62];
static RecData_t read_data(String data)
{
  String data_update;
  RecData_t rec;
  uint8_t index = 0;

  data_update = data;

  while (data_update.indexOf('|') != -1)
  {
    rec_data[index] = data_update.substring(0, data_update.indexOf('|'));  /* Extract substring */
    data_update = data_update.substring(data_update.indexOf('|') + 1);       /* Update string, remove extracted substring and delimiter */
    index++;      /* Update index */
  }

  rec_data[index] = data_update;
  
  if(rec_data[0] == "B") {
    rec.mode = MINIHEXA_CRAWL_STATE;
  }
  else if(rec_data[0] == "C") {
    rec.mode = MINIHEXA_MOVING_CONTROL;
    for(uint8_t i = 0; i < 3; i++) {
      rec.data[i] = (uint8_t)atoi(rec_data[i + 1].c_str());
    }
  }
  else if(rec_data[0] == "F") {
    rec.mode = MINIHEXA_POSE_CONTROL;
    for(uint8_t i = 0; i < 6; i++) {
      rec.data[i] = (uint8_t)atoi(rec_data[1 + i].c_str());
    }    
  }
  else if(rec_data[0] == "G") {
    switch(atoi(rec_data[1].c_str()))
    {
    case 1:
      rec.mode = MINIHEXA_LEG_OFFSET_SET;
      for(uint8_t i = 0; i < 4; i++) {
        rec.data[i] = (uint8_t)atoi(rec_data[2 + i].c_str());
        
      }
      break;

    case 2:
      rec.mode = MINIHEXA_OFFSET_SAVE;
      rec.data[0] = (uint8_t)atoi(rec_data[2].c_str());
      break;

    case 3:
      rec.mode = MINIHEXA_OFFSET_READ;
      rec.data[0] = (uint8_t)atoi(rec_data[2].c_str());
      break;

    }
  }  
  else if(rec_data[0] == "K") {
    switch(atoi(rec_data[1].c_str()))
    {
      case 1:
        rec.mode = MINIHEXA_ACTION_GROUP_RUN;
        rec.data[0] = (uint8_t)atoi(rec_data[2].c_str());
        break;

      case 2:
        rec.mode = MINIHEXA_ACTION_GROUP_STOP;
        break;

      case 3:
        rec.mode = MINIHEXA_ACTION_GROUP_DOWNLOAD;
        for(uint8_t i = 0; i < 60; i++) {
          rec.data[i] = (uint8_t)atoi(rec_data[2 + i].c_str());
        }
        break;

      case 4:
        rec.mode = MINIHEXA_ACTION_GROUP_ERASE;
        rec.data[0] = (uint8_t)atoi(rec_data[2].c_str());
        break;

      case 5:
        rec.mode = MINIHEXA_ACTION_GROUP_ALL_ERASE;

        break;
    }
  }
  else if(rec_data[0] == "L") {
    rec.mode = MINIHEXA_SERVO_CONTROL;
    for(uint8_t i = 0; i < 57; i++) {
      rec.data[i] = (uint8_t)atoi(rec_data[1 + i].c_str());
    }
  }
  else if(rec_data[0] == "M") {
    rec.mode = MINIHEXA_SERVO_DUTY_READ;
  }
  else {
    rec.mode = MINIHEXA_NULL;
    memset(rec.data, 0, sizeof(rec.data));
  }
  return rec;
}

void UartServerManager::begin()
{
  Serial.setRxBufferSize(2048);  // Increase TX buffer to 2KB
  Serial.begin(115200);
}

void UartServerManager::on_receive(void (*callback)(void)) {
  Serial.onReceive(callback, true); 
}

void UartServerManager::receive_message()
{
  while(Serial.available())
  {
    String fullData = Serial.readStringUntil('&');
    rec = read_data(fullData);
  }
}