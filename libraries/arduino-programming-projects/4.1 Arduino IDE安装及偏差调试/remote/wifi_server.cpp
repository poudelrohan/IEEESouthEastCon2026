#include "wifi_server.h"
#include "math.h"

WiFiServerManager::WiFiServerManager()
{
  wifi_connection_state = false;
  tcp_connection_state = false;
  tcp_state = DISCONNECTED;
  udp_pairing_state = false;

  memset(incomingPacket, 0, sizeof(incomingPacket));    
}

RecData_t WiFiServerManager::read_data(String data)
{
  String rec_data[7];
  String data_update;
  RecData_t rec;
  uint8_t index = 0;
  // const char charArray;
  data_update = data;

  while (data_update.indexOf('|') != -1)
  {
    rec_data[index] = data_update.substring(0, data_update.indexOf('|'));  /* 提取字符串 */
    data_update = data_update.substring(data_update.indexOf('|') + 1);       /* 更新字符串，去掉已提取的子字符串和分隔符 */
    index++;      /* 更新索引 */
  }
  rec_data[index] = data_update;
  
  // char charArray = rec_data[0].c_str();      /* 转成C字符串形式 */

  if(rec_data[0] == "C") {
    rec.mode = MINIHEXA_MOVING_CONTROL;
    for(uint8_t i = 0; i < 3; i++) {
      rec.data[i] = static_cast<uint8_t>(atoi(rec_data[i + 1].c_str()));
    }
  }
  else if(rec_data[0] == "F") {
    rec.mode = MINIHEXA_POSE_CONTROL;
    for(uint8_t i = 0; i < 6; i++) {
      rec.data[i] = static_cast<uint8_t>(atoi(rec_data[1 + i].c_str()));
    }    
  }
  else {
    rec.mode = MINIHEXA_NULL;
  }

  return rec;
}

/* 若无连接到对应wifi则返回false,若连接到wifi则返回true并进入WIFI数据接收模式 */
bool WiFiServerManager::begin()
{
  if((WiFi.status() == WL_CONNECTED) && (wifi_connection_state == false))
  {
    ESP_LOGI("WIFI", "Connected to WIFI!\n");
    String macAddress = WiFi.macAddress();
    macAddress.replace(":", "");           // 去掉 MAC 地址中的冒号，得到 240AC4123456
    clientAddress = "MINIHEXA:" + macAddress;
    cClientAddress = clientAddress.c_str();
    udpClient.begin(UDP_LISTENER_PORT);
    tcpServer.begin(TCP_LISTENER_PORT);
    tcpClient.setTimeout(0);
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    selfIP = WiFi.localIP();
    wifi_connection_state = true;
    ESP_LOGI("WIFI", "Local IPAddress: %d.%d.%d.%d\n", selfIP[0], selfIP[1], selfIP[2], selfIP[3]);
  }
  return wifi_connection_state;
}

/* 进入到WIFI模式后如果UDP数据配对成功，则进入到数据传输模式，进入到数据传输模式后，需要把udp_pairing_state 和 wifi_connection_state重新设置为false*/
bool WiFiServerManager::udp_server()
{
  int packetSize = 0;
  static uint32_t tick_start = 0;
  
  if(millis() - tick_start > 1000) {
    // 检查是否有新的 UDP 广播数据
    packetSize = udpClient.parsePacket();
    tick_start = millis();
  }

  if (packetSize)
  {
    int len = udpClient.read(incomingPacket, 255);    
    if (len > 0)
    {
        incomingPacket[len] = 0; // 确保字符串结束
    }
    if (strcmp(incomingPacket, "LOBOT_NET_DISCOVER") == 0)
    {
      // 获取广播发送方的 IP 地址
      clientIP = udpClient.remoteIP();
      ESP_LOGI("WIFI", "Client IPAddress: %d.%d.%d.%d\n", clientIP[0], clientIP[1], clientIP[2], clientIP[3]);
      udpClient.beginPacket(clientIP, UDP_SEND_PORT);
      udpClient.print(clientAddress);  // 发送 MAC 地址
      udpClient.endPacket();
      // memset(incomingPacket, 0, sizeof(incomingPacket));   
      udp_pairing_state = true;
    }
  }
  return udp_pairing_state;
}

bool WiFiServerManager::tcp_server()
{
  // 检查是否有新的客户端连接
  if (!tcpClient && (tcp_connection_state == false))
  {
    tcpClient = tcpServer.available();  // 检查是否有客户端连接
    
    if (tcpClient)
    {
      parameters_reset();
      ESP_LOGI("WIFI", "TCP Client connected!\n");
      tcp_connection_state = true;
    }
    else
    {
      tcp_state = DISCONNECTED;
      return false;
    }
  }  
  // 如果客户端已经连接，检查是否有数据可读
  if(tcp_connection_state == true)
  {
    if(!tcpClient.connected())
    {
      ESP_LOGI("WIFI", "Client disconnected!\n");
      tcp_connection_state = false;
      udpClient.stop();  // 关闭客户端连接
      tcpClient.stop();  // 关闭客户端连接
      tcp_state = DISCONNECTED;
      return false;
    }
    if (tcpClient.available())
    {
      // tcp_state = 2;
      size_t bytesRead = tcpClient.readBytesUntil('&', rec_buffer, sizeof(rec_buffer) - 1);
      rec_buffer[bytesRead] = '\0';  // 添加字符串结束符
      String fullData = String(rec_buffer);  // 转换 rec_buffer 为 String
      rec = WiFiServerManager::read_data(fullData);
      // printf("fullData %s\n", fullData.c_str());
      ESP_LOGI("Received from client", "%s", fullData.c_str());
      // 向客户端发送响应数据
      // String response = "ESP32 received: " + clientData;
      // tcpClient.println(response);
      tcp_state = RECEIVEDATA;
      return true;
    } 
    else
    {
      tcp_state = CONNECTED;
      return true;
    }
  }
}

void WiFiServerManager::parameters_reset()
{
  wifi_connection_state = false;
  tcp_connection_state = false;
  udp_pairing_state = false;

  memset(incomingPacket, 0, sizeof(incomingPacket));    
}

