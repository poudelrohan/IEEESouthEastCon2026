#ifndef __WONDERLLM_H
#define __WONDERLLM_H

#include "stdint.h"
#include <stdbool.h>
#include "WonderLLM_porting.h"

#define WONDERLLM_MAX_FRAME_SIZE 256

/*定义wonderllm回传消息类型*/
typedef enum{
	Frame_NULL = 0,
	Frame_move = 1,
	Frame_get_status_battery = 2,
	Frame_get_status_distance,
	Frame_get_status_running_mode,
	Frame_get_status_Speed_mode,
	Frame_get_status_Bodystate,
	Frame_get_status_poseture,
	Frame_set_running_mode,
	Frame_set_speed_mode,
	Frame_set_led_color,
	Frame_set_buzzer,
	Frame_vision_analysis,
	// Frame_stop,
	Frame_BaryCenterMove,
	Frame_InclinationAngleMove,
	Frame_ActionGroup,
	
}FrameMode;

/*定义wonderllm接收消息结构体*/
typedef struct{
		int16_t motion_target_step;         /*常规运动下的运动步数*/
		int16_t motion_target_RunningTime;  /*常规运动下的运动持续时间*/
		int16_t motion_target_RotationAngle;/*常规运动下的转动角度(仅用于原地左右转)*/
		int16_t motion_target_distance;     /*常规运动下的运动距离(用于八个方向的平移)*/
		char running_mode;               		/*1-normal(常规), 2-intelligent_avoidance(智能避障), 3-intelligent_track(智能跟随)*/
		uint16_t avoid_distance;         		/*机器人与目标保持的最小距离间隔，用于智能避障、智能跟随模式*/
		uint8_t rgb_left[3];             		/*发光超声波模块左侧RGB灯光强比*/
		uint8_t rgb_right[3];            		/*发光超声波模块右侧RGB灯光强比*/
		char buzzer_count;               		/*蜂鸣器鸣响次数*/
		char Speed_Mode;                 		/*1-Low_speed(低速), 2-High_speed(高速)*/
		char Detection_WonderLLM;      		/*0-总线未检测到模块  1-总线检测到模块*/
		char Vision_Result;             		/*视觉识别结果*/
		char incline_direction;         		/*常规运动下的机器人绕欧拉角转动倾斜方向，1-前倾、2-后倾、3-左倾、4-右倾、5-左扭、6-右扭*/
		char BaryCenter_direction;       		/*常规运动下的机器人重心移动方向，1-正前、2-右前、3-正右、4-右后、5-正后、6-左后、7-正左、8-左前、9-正上、10-正下*/
		char ActionNum;                     /*执行动作组序号*/
		char ExecuteActionNum;              /*执行动作组次数*/
		char movement_direction;            /*常规运动下的机器人平移移动方向，1-正前、2-右前、3-正右、4-右后、5-正后、6-左后、7-正左、8-左前 9-原地左转 10-原地右转*/

		FrameMode Frame_mode;               /*当前MCP指令类型*/
		char json_data_raw[256];            /*存放MCP原始指令的缓冲区*/
}WonderLLM_Info;

/*暴露一个wonderllm接收消息对象接口*/
extern WonderLLM_Info WonderLLM_hiwonder;


/**
 * @brief 初始化WonderLLM通信模块。
 *        该函数会扫描并等待从机就绪，然后向从机发送工具注册指令。
 * @return bool true表示初始化成功, false表示失败.
 */
bool WonderLLM_Init(void);

/**
 * @brief 主循环中调用的轮询和处理函数。
 *        该函数会定时向WonderLLM请求指令，并处理收到的指令。
 */
void WonderLLM_Info_Get(WonderLLM_Info *obj);

/**
 * @brief 当动作完成时，调用此函数通知WonderLLM。
 */
void WonderLLM_Send_Action_Finish(void);

/**
 * @brief 当需要返回状态数据时，调用此函数。
 * @param params_str 格式为 "[[\"key\",\"value\"],...]" 的字符串.
 */
void WonderLLM_Send_Status(const char* params_str);

/**
 * @brief 让WonderLLM调用摄像头。
 */
void WonderLLM_Request_Vision(const char* params_str);


#endif /* __WONDERLLM_H */