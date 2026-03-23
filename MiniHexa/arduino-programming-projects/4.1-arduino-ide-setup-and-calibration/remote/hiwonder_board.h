#ifndef __HIWONDER_BOARD_H__
#define __HIWONDER_BOARD_H__

#include "global.h"
#include "SensorQMI8658.hpp"
#include "MadgwickAHRS.h"
#include "driver/ledc.h"

#define MINIHEXA_V1_1

#define BUZZER_PIN      21
#define LEDC_CHANNEL_0  0
#define LEDC_TIMER_BIT  13                // Set 13-bit resolution
#define LEDC_BASE_FREQ  3000              // Set 5kHz PWM base frequency

#define BAT_DEC_ADDR    0x46
#define BAT_READ_REG    0
#define BAT_ADC_PIN     33                // Battery ADC pin (GPIO33 = ADC1_CH5)
#define WINDOW_SIZE     8  // Filter window size
#define R21             100000
#define R22             10000


class HW_Board {
  public:
    uint16_t bat_voltage;
    
    void begin(void);
    /**
     * @brief Get button state
     * 
     * @return true  - not pressed
     * @return false - pressed
     */
    bool get_button_state(void);

    /**
     * @brief Get sound sensor value
     * 
     * @return int 
     */
    int get_sound_val(void);

    /**
     * @brief Update IMU data
     * 
     */
    void _imu_update(void);

    /**
     * @brief Update IMU data
     * @param  state    - true  enable
     *                  - false disable
     */
    void imu_update(bool state);

    /**
     * @brief Get IMU accelerometer data
     * 
     * @param  val
     */
    void get_imu_acc(float *val);

    /**
     * @brief Get IMU gyroscope data
     * 
     * @param  val
     */
    void get_imu_gyro(float *val);

    /**
     * @brief Get IMU Euler angle data
     * 
     * @param  val
     */
    void get_imu_euler(float *val);  
    /**
     * @brief Update battery voltage data
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

    /* Static calibration */
    void imu_calibarate(void);

};

#endif
