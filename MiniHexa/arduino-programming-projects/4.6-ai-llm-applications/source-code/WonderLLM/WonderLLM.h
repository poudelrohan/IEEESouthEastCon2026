#ifndef __WONDERLLM_H
#define __WONDERLLM_H

#include "stdint.h"
#include <stdbool.h>
#include "WonderLLM_porting.h"

#define WONDERLLM_MAX_FRAME_SIZE 256

/* Define WonderLLM response message types */
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

/* Define WonderLLM received message struct */
typedef struct{
		int16_t motion_target_step;         /* Motion steps for normal movement */
		int16_t motion_target_RunningTime;  /* Motion duration for normal movement */
		int16_t motion_target_RotationAngle;/* Rotation angle for normal movement (only for in-place turns) */
		int16_t motion_target_distance;     /* Movement distance for normal movement (for 8-direction translation) */
		char running_mode;               		/* 1-normal, 2-intelligent_avoidance, 3-intelligent_track */
		uint16_t avoid_distance;         		/* Minimum distance to target for avoidance/tracking modes */
		uint8_t rgb_left[3];             		/* Ultrasonic module left RGB LED intensity ratio */
		uint8_t rgb_right[3];            		/* Ultrasonic module right RGB LED intensity ratio */
		char buzzer_count;               		/* Buzzer beep count */
		char Speed_Mode;                 		/* 1-Low_speed, 2-High_speed */
		char Detection_WonderLLM;      		/* 0-module not detected on bus, 1-module detected */
		char Vision_Result;             		/* Vision recognition result */
		char incline_direction;         		/* Euler angle tilt direction: 1-forward, 2-backward, 3-left, 4-right, 5-twist left, 6-twist right */
		char BaryCenter_direction;       		/* Body center shift direction: 1-F, 2-FR, 3-R, 4-BR, 5-B, 6-BL, 7-L, 8-FL, 9-up, 10-down */
		char ActionNum;                     /* Action group number to execute */
		char ExecuteActionNum;              /* Action group execution count */
		char movement_direction;            /* Movement direction: 1-F, 2-FR, 3-R, 4-BR, 5-B, 6-BL, 7-L, 8-FL, 9-turn left, 10-turn right */

		FrameMode Frame_mode;               /* Current MCP command type */
		char json_data_raw[256];            /* Buffer for raw MCP command data */
}WonderLLM_Info;

/* Expose WonderLLM received message object interface */
extern WonderLLM_Info WonderLLM_hiwonder;


/**
 * @brief Initialize WonderLLM communication module.
 *        Scans and waits for slave ready, then sends tool registration commands.
 * @return bool true = initialization success, false = failure.
 */
bool WonderLLM_Init(void);

/**
 * @brief Polling and processing function called in main loop.
 *        Periodically requests commands from WonderLLM and processes received commands.
 */
void WonderLLM_Info_Get(WonderLLM_Info *obj);

/**
 * @brief Call this function to notify WonderLLM when action is complete.
 */
void WonderLLM_Send_Action_Finish(void);

/**
 * @brief Call this function when status data needs to be returned.
 * @param params_str Format: "[[\"key\",\"value\"],...]".
 */
void WonderLLM_Send_Status(const char* params_str);

/**
 * @brief Let WonderLLM invoke the camera.
 */
void WonderLLM_Request_Vision(const char* params_str);


#endif /* __WONDERLLM_H */