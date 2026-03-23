#ifndef WIFI_SERVER_H_
#define WIFI_SERVER_H_

#include <esp_wifi.h>
#include "WiFi.h"
#include "global.h"

#define WIFI_SSID             "hiwonder"
#define WIFI_PASSWORD         "hiwonder"

#define UDP_LISTENER_PORT     9027
#define UDP_SEND_PORT         9025
#define TCP_LISTENER_PORT     9023

#ifdef __cplusplus
extern "C" 
{
#endif

#ifdef __cplusplus
} // extern "C"
#endif

class WiFiServerManager
{
public:
  enum TCPState {DISCONNECTED, CONNECTED, RECEIVEDATA};

  RecData_t rec;
  TCPState tcp_state;
  
  WiFiClient tcpClient;     // TCP 配置

  WiFiServerManager();
  bool begin();
  bool tcp_server();
  bool udp_server();
  void parameters_reset();

private:
  IPAddress clientIP;       // 用于存储从广播中提取的TCP 客户端IP 地址
  IPAddress selfIP;      // 用于存储自身IP 地址
  String clientAddress;
  const char* cClientAddress;  

  WiFiUDP udpClient;
  uint32_t tick_start = 0;
  char incomingPacket[255];  // 缓存接收的广播数据   
  bool udp_pairing_state; 

  
  WiFiServer tcpServer;  // 创建TCP服务器，监听9023端口
  bool tcp_connection_state;     // 连接标志状态

  bool wifi_connection_state;

  char rec_buffer[256]; // 假设每次读取的最大数据长度为 1024 字节

  static RecData_t read_data(String data);
};

#endif