#ifndef BLE_SERVER_H_
#define BLE_SERVER_H_

#include "BLEDevice.h"
#include "BLEUtils.h"
#include "BLEServer.h"
#include "BLE2902.h"
#include "global.h"
#include <Arduino.h>

#define BLUETOOTH_NAME              "miniHexa"
#define SERVICE_UUID                "0000ffe0-0000-1000-8000-00805f9b34fb"
#define UART_TX_CHARACTERISTIC_UUID "0000ffe2-0000-1000-8000-00805f9b34fb"
#define UART_RX_CHARACTERISTIC_UUID "0000ffe1-0000-1000-8000-00805f9b34fb"

#ifdef __cplusplus
extern "C" 
{
#endif

typedef enum
{
  SWITCH_UART = 0,
  SWITCH_BLE,
  SWITCH_WIFI
}BLEState_t;

typedef enum
{
  RECEIVE_DATA_FAILD = 0,
  RECEIVE_DATA_SUCCESS
}RECState_t;

#ifdef __cplusplus
} // extern "C"
#endif

class BLEServerManager
{
public:

  BLEState_t state;
  static RECState_t rec_state;
  static RecData_t rec;
  void begin();
  void send_message(uint16_t value_1, uint16_t value_2);

private: 

  BLECharacteristic *pTxCharacteristic;
  BLEServer *pServer;
  
  // RecData_t readVariable;
  class MyCallbacks : public BLECharacteristicCallbacks {
  private:
    BLEServerManager* manager;  // Reference to BLEServerManager
    String ble_buf;
  public:
    MyCallbacks(BLEServerManager* mgr) : manager(mgr) {}
    void onWrite(BLECharacteristic *pCharacteristic) override;
  };        

  class MyServerCallbacks : public BLEServerCallbacks {
  private:
    BLEServerManager* manager;  // Reference to BLEServerManager
  public:
    MyServerCallbacks(BLEServerManager* mgr) : manager(mgr) {}
    void onConnect(BLEServer* pServer) override;
    void onDisconnect(BLEServer* pServer) override;
  };

  static String last_rec_data[7];
  static void data_parsing(RecData_t *self);
  static RecData_t read_data(String data);
};
#endif
