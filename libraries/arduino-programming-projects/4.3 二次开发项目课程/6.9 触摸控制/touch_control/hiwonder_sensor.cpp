#include "hiwonder_sensor.h"
#include "hiwonder_i2c.h"

uint8_t alarm_state;

uint8_t Wonder_Echo::rec_recognition(void) {
  uint8_t result = 0;

  wire_read_array(WONDER_ECHO_ADDR, ASR_RESULT_REG, &result, 1);
  return result;
}

void Wonder_Echo::speak(uint8_t cmd,uint8_t id) {
  uint8_t send[2];

  if(cmd == ASR_COMMAND || cmd == ASR_ANNOUNCER) {
    send[0] = cmd;
    send[1] = id;
    wire_write_array(WONDER_ECHO_ADDR, ASR_SPEAK_REG, send, 2);
  }
}

uint16_t ESP32S3_Cam::red_block_detection(uint8_t *buf, uint8_t buf_len) {
  return (uint16_t)wire_read_array(ESP32S3_CAM_ADDR, COLOR_DETECTION_FIRST_COLOR_REG, buf, buf_len);
}

uint16_t ESP32S3_Cam::green_block_detection(uint8_t *buf, uint8_t buf_len) {
  return (uint16_t)wire_read_array(ESP32S3_CAM_ADDR, COLOR_DETECTION_SECOND_COLOR_REG, buf, buf_len);
}

uint16_t ESP32S3_Cam::blue_block_detection(uint8_t *buf, uint8_t buf_len) {
  return (uint16_t)wire_read_array(ESP32S3_CAM_ADDR, COLOR_DETECTION_THIRD_COLOR_REG, buf, buf_len);
}

uint16_t ESP32S3_Cam::purple_block_detection(uint8_t *buf, uint8_t buf_len) {
  return (uint16_t)wire_read_array(ESP32S3_CAM_ADDR, COLOR_DETECTION_FOURTH_COLOR_REG, buf, buf_len);
}

uint16_t ESP32S3_Cam::color_id_detection(uint8_t *buf, uint8_t buf_len) {
  return (uint16_t)wire_read_array(ESP32S3_CAM_ADDR, COLOR_DETECTION_ID_REG, buf, buf_len);
}

uint16_t ESP32S3_Cam::face_data_receive(uint8_t *buf, uint8_t buf_len) {
  return (uint16_t)wire_read_array(ESP32S3_CAM_ADDR, FACE_DETECTION_REG, buf, buf_len);
}

uint16_t ESP32S3_Cam::region1_red_block_detection(uint8_t *buf, uint8_t buf_len) {
  return (uint16_t)wire_read_array(ESP32S3_CAM_ADDR, LINE_FOLLOWING_FIRST_COLOR_REG1, buf, buf_len);
}

uint16_t ESP32S3_Cam::region2_red_block_detection(uint8_t *buf, uint8_t buf_len) {
  return (uint16_t)wire_read_array(ESP32S3_CAM_ADDR, LINE_FOLLOWING_FIRST_COLOR_REG2, buf, buf_len);
}

uint16_t ESP32S3_Cam::region1_green_block_detection(uint8_t *buf, uint8_t buf_len) {
  return (uint16_t)wire_read_array(ESP32S3_CAM_ADDR, LINE_FOLLOWING_SECOND_COLOR_REG1, buf, buf_len);
}

uint16_t ESP32S3_Cam::region2_green_block_detection(uint8_t *buf, uint8_t buf_len) {
  return (uint16_t)wire_read_array(ESP32S3_CAM_ADDR, LINE_FOLLOWING_SECOND_COLOR_REG2, buf, buf_len);
}

uint16_t ESP32S3_Cam::region1_blue_block_detection(uint8_t *buf, uint8_t buf_len) {
  return (uint16_t)wire_read_array(ESP32S3_CAM_ADDR, LINE_FOLLOWING_THIRD_COLOR_REG1, buf, buf_len);
}

uint16_t ESP32S3_Cam::region2_blue_block_detection(uint8_t *buf, uint8_t buf_len) {
  return (uint16_t)wire_read_array(ESP32S3_CAM_ADDR, LINE_FOLLOWING_THIRD_COLOR_REG2, buf, buf_len);
}

uint16_t ESP32S3_Cam::region1_purple_block_detection(uint8_t *buf, uint8_t buf_len) {
  return (uint16_t)wire_read_array(ESP32S3_CAM_ADDR, LINE_FOLLOWING_FOURTH_COLOR_REG1, buf, buf_len);
}

uint16_t ESP32S3_Cam::region2_purple_block_detection(uint8_t *buf, uint8_t buf_len) {
  return (uint16_t)wire_read_array(ESP32S3_CAM_ADDR, LINE_FOLLOWING_FOURTH_COLOR_REG2, buf, buf_len);
} 

void HW_Sensor::begin() {
  Wire.setPins(SDA, SCL);
  Wire.begin();
  pinMode(io1_pin, INPUT); 
  pinMode(io3_pin, INPUT); 
}

void HW_Sensor::set_ultrasound_rgb(uint8_t mode, uint8_t *rgb1, uint8_t *rgb2) {
  uint8_t value;
  uint8_t rgb[6];

  switch(mode) {
    /* simple mdoe */
    case 0:
      value = RGB_WORK_SOLID_MODE;
      wire_write_array(ULTRASOUND_ADDR, RGB_WORK_MODE_REG, &value, 1);
      for(uint8_t i = 0; i < 6; i++) {
        rgb[i] = i < 3 ? rgb1[i] : rgb2[i - 3];
      }

      wire_write_array(ULTRASOUND_ADDR, SOLID_RGB_SET_REG, rgb, 6);
      break;

    /* breathing mdoe */
    case 1:
      value = RGB_WORK_BREATHING_MODE;
      wire_write_array(ULTRASOUND_ADDR, RGB_WORK_MODE_REG, &value, 1);
      for(uint8_t i = 0; i < 6; i++) {
        rgb[i] = i < 3 ? rgb1[i] : rgb2[i - 3];
      }
      wire_write_array(ULTRASOUND_ADDR, BREATHING_RGB_SET_REG, rgb, 6);
      break;

    default:
      break;
  }
}

uint16_t HW_Sensor::_get_distance() {
  uint16_t distance;

  wire_read_array(ULTRASOUND_ADDR, DISTANCE_REG, (uint8_t *)&distance, 2);
  return distance;
}

uint16_t HW_Sensor::get_distance() {
  uint32_t filter_sum = 0;

  filter[FILTER_NUM] = _get_distance();
  for(uint8_t i = 0; i < FILTER_NUM; i++) {
    filter[i] = filter[i + 1];
    filter_sum += filter[i];
  }
  return (uint16_t)(filter_sum / FILTER_NUM);
}



uint8_t HW_Sensor::get_touch_state() {
  return digitalRead(io3_pin);
}

uint8_t HW_Sensor::get_ir1_state() {
  return digitalRead(io1_pin);
}

uint8_t HW_Sensor::get_ir2_state() {
  return digitalRead(io3_pin);
}