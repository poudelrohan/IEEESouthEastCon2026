#ifndef _WONDERLLM_PORTING_H
#define _WONDERLLM_PORTING_H

#include <Arduino.h>
#include <stdbool.h>
#include "stdint.h"
#include <Wire.h>
// --- I2C & 协议定义 ---
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
 * @brief 系统延时功能接口
 */
void delay_ms(int ms_num);

/**
 * @brief 系统实时时间获取接口
 */
uint32_t Get_time_now(void);

/**
 * @brief IIC扫描接口
 * @note  1.执行非必要，仅是出于程序健壮性的考虑
 */
bool Detect_WonderLLM();

/**
 * @brief IIC速率配置接口
 * @note 涉及较长字符串（尤其是中文）的传输，必须使用调用本函数将IIC速率
 *       提升至400,000，否则WonderLLM将无法完整接收数据 
 */
void IIC_Config_MCP_Transmit(void);

/**
 * @brief IIC速率配置接口
 * @note 1.执行非必要，如果其他IIC设备均支持400W速率通信，则可不必切换回
 *         较低的100W，执行该函数是出于兼容其他低速IIC设备的考虑
 */
void IIC_Config_normal_Transmit(void);

/**
 * @brief I2C底层数据接收接口
 */
// int WonderLLM_Receive_Data(uint8_t* buffer, uint16_t size);
int WonderLLM_Receive_Data(uint8_t* buffer, uint16_t size,bool stop_flag);

/**
 * @brief I2C底层数据发送接口
 */
int WonderLLM_Send_Data(uint8_t* buffer, uint16_t len);


#endif