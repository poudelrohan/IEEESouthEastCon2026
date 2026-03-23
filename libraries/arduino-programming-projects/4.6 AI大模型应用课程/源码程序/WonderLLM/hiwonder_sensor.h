#ifndef __HIWONDER_SENSOR_H__
#define __HIWONDER_SENSOR_H__

#include "global.h"

#define ULTRASOUND_ADDR                     0x77  /* 超声波模块I2C从机地址 */
#define DISTANCE_REG                        0     /* 距离低8位，单位mm */
#define RGB_WORK_MODE_REG                   2     /*  RGB灯模式设置寄存器 */
#define RGB_WORK_SOLID_MODE                 0
#define RGB_WORK_BREATHING_MODE             1
#define SOLID_RGB_SET_REG                   3
#define BREATHING_RGB_SET_REG               9
#define FILTER_NUM                          3


#define WONDER_ECHO_ADDR                    0x34
#define ASR_RESULT_REG                      100
#define ASR_SPEAK_REG                       110
#define ASR_COMMAND                         0x00
#define ASR_ANNOUNCER                       0xFF

#define ESP32S3_CAM_ADDR                    0x52
#define COLOR_DETECTION_FIRST_COLOR_REG     0x00
#define COLOR_DETECTION_SECOND_COLOR_REG    0x01
#define COLOR_DETECTION_THIRD_COLOR_REG     0x02
#define COLOR_DETECTION_FOURTH_COLOR_REG    0x03
#define COLOR_DETECTION_ID_REG              0x04
#define FACE_DETECTION_REG                  0x01
#define LINE_FOLLOWING_FIRST_COLOR_REG1     0xA0
#define LINE_FOLLOWING_FIRST_COLOR_REG2     0xA1
#define LINE_FOLLOWING_SECOND_COLOR_REG1    0xA2
#define LINE_FOLLOWING_SECOND_COLOR_REG2    0xA3
#define LINE_FOLLOWING_THIRD_COLOR_REG1     0xA4
#define LINE_FOLLOWING_THIRD_COLOR_REG2     0xA5
#define LINE_FOLLOWING_FOURTH_COLOR_REG1    0xA6
#define LINE_FOLLOWING_FOURTH_COLOR_REG2    0xA7

class Wonder_Echo {
  public:
    uint8_t rec_recognition(void);
    void speak(uint8_t cmd, uint8_t id);
};

class ESP32S3_Cam {
  public:
    uint16_t red_block_detection(uint8_t *buf, uint8_t buf_len);
    uint16_t green_block_detection(uint8_t *buf, uint8_t buf_len);
    uint16_t blue_block_detection(uint8_t *buf, uint8_t buf_len);
    uint16_t purple_block_detection(uint8_t *buf, uint8_t buf_len);
    uint16_t color_id_detection(uint8_t *buf, uint8_t buf_len); 
    uint16_t face_data_receive(uint8_t *buf, uint8_t buf_len);  
    uint16_t region1_red_block_detection(uint8_t *buf, uint8_t buf_len);
    uint16_t region1_green_block_detection(uint8_t *buf, uint8_t buf_len);
    uint16_t region1_blue_block_detection(uint8_t *buf, uint8_t buf_len);
    uint16_t region1_purple_block_detection(uint8_t *buf, uint8_t buf_len);

    uint16_t region2_red_block_detection(uint8_t *buf, uint8_t buf_len);
    uint16_t region2_green_block_detection(uint8_t *buf, uint8_t buf_len);
    uint16_t region2_blue_block_detection(uint8_t *buf, uint8_t buf_len);
    uint16_t region2_purple_block_detection(uint8_t *buf, uint8_t buf_len);     
};

class HW_Sensor{
  public:
    void begin();
    /*  */
    void set_ultrasound_rgb(uint8_t mode, uint8_t *rgb1, uint8_t *rgb2); //r1，g1，b1表示右边rgb灯的呼吸周期，20表示2s一个周期
    uint16_t _get_distance(void);
    uint16_t get_distance(void);
    uint16_t get_bat_voltage(void);
    uint8_t get_touch_state(void);  /* 0-按下   1-松开 */
    uint8_t get_ir1_state(void);    /* 0-有障碍 1-无 */
    uint8_t get_ir2_state(void);

    Wonder_Echo asr;
    ESP32S3_Cam camera;

  private:
    uint16_t filter[FILTER_NUM + 1];

    const int io1_pin = 18;
    const int io2_pin = 19;
    const int io3_pin = 32;
    const int v1_0_io4_pin = 33;
    const int v1_1_io4_pin = 14;

};

#endif