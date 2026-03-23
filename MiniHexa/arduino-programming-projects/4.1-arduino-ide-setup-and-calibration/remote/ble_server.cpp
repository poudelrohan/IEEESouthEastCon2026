#include "ble_server.h"

String BLEServerManager::last_rec_data[7] = {}; 
RecData_t BLEServerManager::rec = {}; 
RECState_t BLEServerManager::rec_state = RECEIVE_DATA_FAILD;

RecData_t BLEServerManager::read_data(String data)
{
  String rec_data[7];
  String data_update;
  RecData_t rec;
  uint8_t index = 0;
  uint8_t count = 0;
  data_update = data;

  while (data_update.indexOf('|') != -1) {
    rec_data[index] = data_update.substring(0, data_update.indexOf('|'));  /* Extract substring */
    data_update = data_update.substring(data_update.indexOf('|') + 1);       /* Update string, remove extracted substring and delimiter */
    index++;      /* Update index */
  }
  rec_data[index] = data_update;
  // for(uint8_t i  =0; i < 7; i++) {
  //   if(rec_data[i] == last_rec_data[i]) {
  //     count++;
  //   }
  // }
  // if(count == 7) {
  //   for(uint8_t i = 0; i < 6; i++) {
  //     rec.data[i] = 0;
  //   } 
  //   return rec;
  // }
  // else {
    memcpy(last_rec_data,rec_data,sizeof(rec_data));
    if(rec_data[0] == "C") {
      rec.mode = MINIHEXA_MOVING_CONTROL;
      for(uint8_t i = 0; i < 3; i++) {
        rec.data[i] = (uint8_t)atoi(rec_data[1 + i].c_str());
      } 
    }
    else if(rec_data[0] == "D") {
      rec.mode = MINIHEXA_ARM_CONTROL;
    }
    else if(rec_data[0] == "F") {
      rec.mode = MINIHEXA_POSE_CONTROL;
      for(uint8_t i = 0; i < 6; i++) {
        rec.data[i] = (uint8_t)atoi(rec_data[1 + i].c_str());
      }  
    }
    else if(rec_data[0] == "H") {
      rec.mode = MINIHEXA_RGB_ADJUST;
      for(uint8_t i = 0; i < 3; i++) {
        rec.data[i] = (uint8_t)atoi(rec_data[1 + i].c_str());
      }  
    }
    else if(rec_data[0] == "I") {
      rec.mode = MINIHEXA_AVOID;
      rec.data[0] = (uint8_t)atoi(rec_data[1].c_str());
    }  
    else if(rec_data[0] == "J") {
      rec.mode = MINIHEXA_BALANCE;
      rec.data[0] = (uint8_t)atoi(rec_data[1].c_str());
    }    
    else if(rec_data[0] == "K") {
      if(atoi(rec_data[1].c_str()) == 1) {
        rec.mode = MINIHEXA_ACTION_GROUP_RUN;
        rec.data[0] = (uint8_t)atoi(rec_data[2].c_str());
      }
    }  
    else if(rec_data[0] == "O") {
      rec.mode = MINIHEXA_RESET;
    }  
    else {
      rec.mode = MINIHEXA_NULL;
    } 
    return rec;
  // }
}

void BLEServerManager::MyCallbacks::onWrite(BLECharacteristic *pCharacteristic)
{
  String value = pCharacteristic->getValue();
  if (value.length() > 0) {
    ble_buf += value;  // Append received data to buffer
    int endPos = ble_buf.indexOf('&');
    if(endPos != -1) {
      rec_state = RECEIVE_DATA_SUCCESS;
      String completeData = ble_buf.substring(0, endPos);  // Extract complete data
      ble_buf = ble_buf.substring(endPos + 1);  // Remove processed data
      rec = BLEServerManager::read_data(completeData);
      ESP_LOGI("BLE", "receive %s\n", completeData.c_str());
    }
  }
}

void BLEServerManager::MyServerCallbacks::onConnect(BLEServer* pServer)
{
  manager->state = SWITCH_BLE;
  ESP_LOGI("BLE", "BLE client connected.\n");
  BLEDevice::getAdvertising()->stop();
}

void BLEServerManager::MyServerCallbacks::onDisconnect(BLEServer* pServer)
{
  manager->state = SWITCH_UART;
  BLEDevice::getAdvertising()->start();
  ESP_LOGI("BLE", "Device disconnected, restarting advertising...\n");
}

void BLEServerManager::begin()
{
  pTxCharacteristic = NULL;
  pServer = NULL;
  state = SWITCH_UART;
  memset(&rec, 0, sizeof(RecData_t));

  BLEDevice::init(BLUETOOTH_NAME);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks(this));

  BLEService *pService = pServer->createService(SERVICE_UUID);

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
                                          UART_RX_CHARACTERISTIC_UUID,
                                          BLECharacteristic::PROPERTY_WRITE);

  pTxCharacteristic = pService->createCharacteristic(
                                          UART_TX_CHARACTERISTIC_UUID,
                                          BLECharacteristic::PROPERTY_NOTIFY);

  pTxCharacteristic->addDescriptor(new BLE2902());

  pRxCharacteristic->setCallbacks(new MyCallbacks(this));
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();  
  ESP_LOGI("BLE", "The BLE server has started and is waiting for client connections.\n");
}

void BLEServerManager::send_message(uint16_t value_1, uint16_t value_2)
{
  String message = "$" + String(value_1) + "$" + String(value_2) + "$";
  if (message.length() > 0)
  {
    pTxCharacteristic->setValue(message);
    pTxCharacteristic->notify();
  }
}