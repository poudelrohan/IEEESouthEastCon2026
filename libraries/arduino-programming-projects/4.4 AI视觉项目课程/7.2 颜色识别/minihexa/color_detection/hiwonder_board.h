#ifndef __HIWONDER_BOARD_H__
#define __HIWONDER_BOARD_H__

#include "global.h"
#include "SensorQMI8658.hpp"
#include "MadgwickAHRS.h"
#include "esp_adc_cal.h"
#include "driver/ledc.h"
#include "driver/adc.h"

#define MINIHEXA_V1_1

#define BUZZER_PIN      21
#define LEDC_CHANNEL_0  0
#define LEDC_TIMER_BIT  13                // 设置 13 位分辨率
#define LEDC_BASE_FREQ  3000              // 设置 5kHz 的 PWM 基础频率

#define BAT_DEC_ADDR    0x46
#define BAT_READ_REG    0
#define DEFAULT_VREF    1100              // 默认1.1V的参考电压
#define NO_OF_SAMPLES   64                // ADC采样次数
#define ADC_WIDTH       ADC_WIDTH_12Bit   // ADC 12位宽度
#define ADC_ATTEN       ADC_ATTEN_DB_11   // 6dB衰减器
#define ADC_PIN         ADC1_CHANNEL_5    // ADC引脚
#define WINDOW_SIZE     8  // 滤波窗口大小
#define R21             100000
#define R22             10000


class HW_Board {
  public:
    uint16_t bat_voltage;
    
    void begin(void);
    /**
     * @brief 获取按键状态
     * 
     * @return true  -没按下
     * @return false -按下
     */
    bool get_button_state(void);

    /**
     * @brief 获取声音传感器数值
     * 
     * @return int 
     */
    int get_sound_val(void);

    /**
     * @brief 更新陀螺仪数据
     * 
     */
    void _imu_update(void);

    /**
     * @brief 更新陀螺仪数据
     * @param  state    -true  打开
     *                  -false 关闭
     */
    void imu_update(bool state);

    /**
     * @brief 获取陀螺仪加速度数据
     * 
     * @param  val
     */
    void get_imu_acc(float *val);

    /**
     * @brief 获取陀螺仪角速度数据
     * 
     * @param  val
     */
    void get_imu_gyro(float *val);

    /**
     * @brief 获取陀螺仪欧拉角数据
     * 
     * @param  val
     */
    void get_imu_euler(float *val);  
    /**
     * @brief 更新电池电压数据
     * 
     * @return  
     */
    void bat_voltage_update(void);

  private:

    IMUdata acc;
    IMUdata acc_offset;
    IMUdata acc_cali;
    IMUdata gyro;
    IMUdata gyro_offset;
    IMUdata gyro_cali;
    IMUdata euler;  /* x-roll y-pitch z-yaw */

    SensorQMI8658 qmi;
    Madgwick filter;

    TimerHandle_t imu_timer;
    TimerHandle_t buzzer_timer;
    TimerHandle_t bat_monitor_timer;
    esp_adc_cal_characteristics_t *adc_chars = NULL;
    
    bool buzzer_on_state = false;
    bool imu_on_state = false;
    uint8_t voltage_index = 0;  
    int last_sample_voltage = 0;
    uint32_t voltage_buffer[WINDOW_SIZE] = {0};

    const int led_pin = 2;
    const int button_pin = 0;
    const int sda_pin = 22;
    const int scl_pin = 23;
    const int sound_pin = 34;
    const uint16_t imu_update_interval = 10;
    const uint16_t bat_monitor_update_interval = 1000;

    /* 静态校准 */
    void imu_calibarate(void);

};

#endif
