#ifndef __HIWONDER_SERVO_H__
#define __HIWONDER_SERVO_H__

#include "global.h"
#include <vector>

#define PACKET_HEADER1                      0x55
#define PACKET_HEADER2                      0x55

#define PACKET_FUNC_PWM_SERVO_CONTROL       0x01

#define PACKET_CMD_SET_SERVO                0x01

#define TIMEOUT                               5

typedef struct {
  uint8_t id;
  uint16_t duty;
}ServoArg_t;

class HW_Servo {
  public:
    void begin();
    uint8_t set(uint8_t id, uint16_t duty, uint16_t time);
    uint8_t multi_set(ServoArg_t* arg, uint16_t servo_num, uint16_t time);
    
  private:
    uint8_t checksum_sum(const uint8_t *buf, uint16_t len);
    void transmit_packet(const uint8_t *data, uint8_t data_len);
    void transmit_servo_packet(ServoArg_t* servo, uint8_t num, uint16_t time);
    uint8_t id;
    uint16_t duty;
    const uint16_t min_duty = 500;
    const uint16_t max_duty = 2500;
    uint16_t time;
};

#endif
