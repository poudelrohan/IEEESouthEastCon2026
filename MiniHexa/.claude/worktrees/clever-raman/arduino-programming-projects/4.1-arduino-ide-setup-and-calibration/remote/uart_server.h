#ifndef UART_SERVER_H_
#define UART_SERVER_H_

#include "Arduino.h"
#include "global.h"

class UartServerManager
{
public:
  RecData_t rec;
  void begin();
  void receive_message();
  void on_receive(void (*callback)(void));
};
#endif
