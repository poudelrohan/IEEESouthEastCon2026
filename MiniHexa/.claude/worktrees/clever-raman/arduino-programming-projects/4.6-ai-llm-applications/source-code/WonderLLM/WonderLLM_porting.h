#ifndef _WONDERLLM_PORTING_H
#define _WONDERLLM_PORTING_H

#include <Arduino.h>
#include <stdbool.h>
#include "stdint.h"
#include <Wire.h>
// --- I2C & Protocol Definitions ---
#define WONDERLLM_SLAVE_ADDRESS 0x55//0x55
#define I2C_TIMEOUT              10 // ms

extern const char tool_finish[];
extern const char tool_buzzer[];
extern const char tool_led[];
extern const char tool_stop[];
extern const char tool_SpeedMode[];
extern const char tool_RunningMode[];
extern const char tool_status[];
extern const char tool_move[];
extern const char tool_BaryCenterMove[];
extern const char tool_InclinationAngleMove[];
extern const char tool_ActionGroup[];

extern const char vision_prompt[];

/**
 * @brief System delay interface
 */
void delay_ms(int ms_num);

/**
 * @brief System real-time clock interface
 */
uint32_t Get_time_now(void);

/**
 * @brief I2C scan interface
 * @note  Optional, only for program robustness
 */
bool Detect_WonderLLM();

/**
 * @brief I2C rate configuration interface
 * @note For long string transmission, I2C rate must be set to 400kHz
 *       using this function, otherwise WonderLLM cannot receive complete data
 */
void IIC_Config_MCP_Transmit(void);

/**
 * @brief I2C rate configuration interface
 * @note Optional: if all I2C devices support 400kHz, no need to switch back
 *       to 100kHz. This is for compatibility with slower I2C devices
 */
void IIC_Config_normal_Transmit(void);

/**
 * @brief I2C low-level data receive interface
 */
// int WonderLLM_Receive_Data(uint8_t* buffer, uint16_t size);
int WonderLLM_Receive_Data(uint8_t* buffer, uint16_t size,bool stop_flag);

/**
 * @brief I2C low-level data send interface
 */
int WonderLLM_Send_Data(uint8_t* buffer, uint16_t len);


#endif