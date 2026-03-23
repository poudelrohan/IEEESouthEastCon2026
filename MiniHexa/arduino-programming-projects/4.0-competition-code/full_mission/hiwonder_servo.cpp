#include "hiwonder_servo.h"

uint8_t HW_Servo::checksum_sum(const uint8_t *buf, uint16_t len) {
    uint16_t check = 0;
    while (len--) {
        check += *buf++;
    }
    return (uint8_t)(check & 0x00FF);
}

void HW_Servo::transmit_packet(const uint8_t *data, uint8_t data_len) {
  /* Calculate total frame length (header 2 + type 1 + data length 1 + data data_len + checksum 1) */
  uint8_t frame_len = 2 + 1 + 1 + data_len + 1;
  uint8_t frame[frame_len];

    /* Fill frame header */
    frame[0] = PACKET_HEADER1;
    frame[1] = PACKET_HEADER2;

    /* Type */
    frame[2] = PACKET_FUNC_PWM_SERVO_CONTROL;

    /* Data length (data_len) */
    frame[3] = data_len + 2 + 1;

    for (uint8_t i = 0; i < data_len; i++) {
      frame[4 + i] = data[i];
      // Serial.print(frame[4 + i]);
      // Serial.print(" ,");
    }

    /* Calculate checksum (header, function ID, data length, data) */
    uint8_t checksum = checksum_sum(frame, frame_len - 1);
    // /* Fill checksum byte (last byte) */
    frame[frame_len - 1] = checksum;
    // Serial.print(checksum);
    // Serial.println();
    /* Send frame via serial */
    Serial1.write(frame, frame_len);
    Serial1.flush();
}

void HW_Servo::transmit_servo_packet(ServoArg_t* servo, uint8_t num, uint16_t time) {

  /* cmd|num|time_l|time_h|id|duty_l|duty_h|...*/
  const uint8_t data_len = 4 + num * 3;
  uint8_t data_raw[data_len];

  data_raw[0] = PACKET_CMD_SET_SERVO;
  data_raw[1] = num;
  data_raw[2] = GET_LOW_BYTE(time);
  data_raw[3] = GET_HIGH_BYTE(time);

  for(uint8_t i = 0; i < num; i++) {
    data_raw[4 + (i * 3)] = servo[i].id;
    data_raw[4 + (i * 3) + 1] = GET_LOW_BYTE(servo[i].duty);
    data_raw[4 + (i * 3) + 2] = GET_HIGH_BYTE(servo[i].duty);
  }
  transmit_packet(data_raw, data_len);
}

void HW_Servo::begin() {
  /* Servo driver */
  Serial1.setTxBufferSize(2048);  // Increase TX buffer to 2KB
  Serial1.begin(230400, SERIAL_8N1, RXD2,TXD2);
}

uint8_t HW_Servo::set(uint8_t id, uint16_t duty, uint16_t time) {
  if(id > 21) return 0;

  this->duty = duty > this->max_duty ? 2500 : duty;
  this->duty = duty < this->min_duty ? 500 : duty;

  ServoArg_t servo = {.id = id, .duty = duty};
  transmit_servo_packet(&servo, 1, time);
  return 1;
}

uint8_t HW_Servo::multi_set(ServoArg_t* arg, uint16_t servo_num, uint16_t time) {
  if (arg == nullptr) return 0;
  if(servo_num == 0 || servo_num > 21) return 0;

  for(uint8_t i = 0; i < servo_num; i++) {
    if (arg[i].duty < this->min_duty) {
      arg[i].duty = this->min_duty;
    } else if (arg[i].duty > this->max_duty) {
      arg[i].duty = this->max_duty;
    }
  }
  transmit_servo_packet(arg, servo_num, time);
  return 1;
}
