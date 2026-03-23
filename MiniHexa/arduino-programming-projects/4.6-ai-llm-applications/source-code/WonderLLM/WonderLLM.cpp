/**
 * @file WonderLLM.cpp
 * @brief WonderLLM module driver (abstraction layer)
 * @note  This file handles module logic only. See WonderLLM_porting for hardware ops. No changes needed for porting.
 * @author ZhiYuan (Gilbert@hiwonder.com)
 */

#include "WonderLLM.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <Wire.h>

#define debug_mode 0

// --- Internal static variables ---
WonderLLM_Info WonderLLM_hiwonder;

// --- Static function prototypes ---
static int parse_command(WonderLLM_Info *obj,const char* json_str);
static bool send_frame(const uint8_t* data, uint16_t len);
static bool receive_frame(uint8_t* buffer, uint16_t* len);
static uint8_t calculate_checksum(const uint8_t* data, uint16_t len);
static bool register_tools(void);

/**
 * @brief Check if module exists, complete custom MCP tool registration
 */
bool WonderLLM_Init(void) {
    uint32_t start_tick =  Get_time_now(); // Record start time

    while (1) {
        // 1. Check if device is ready
        if (Detect_WonderLLM() == true) {
            // Device found, execute initialization sequence
						IIC_Config_MCP_Transmit();	
            delay_ms(5);
            register_tools(); // Call MCP tool registration	
						IIC_Config_normal_Transmit();

						return true;

         }else{		// 2. Device not found, check timeout

						if(Get_time_now() - start_tick > 3000){
								// Device not found after 5s, initialization failed
								return false;						
						}else{
								// 3. Brief delay to avoid excessive I2C bus polling
				        delay_ms(100); 						
						}
				 
				 }  
		}				        
}


/**
 * @brief Poll and receive commands from WonderLLM in main loop
 */

void WonderLLM_Info_Get(WonderLLM_Info *obj) {

		obj->Frame_mode = Frame_NULL;
		if (Detect_WonderLLM() == true) {
				obj->Detection_WonderLLM = 1;

				uint16_t received_len = sizeof(obj->json_data_raw);
				// Try to receive data from I2C bus
				// Copy raw JSON string from WonderLLM to json_data_raw backup
				if (receive_frame((uint8_t*)obj->json_data_raw, &received_len)) {
						if (received_len > 0) {
								// Append '\0'to build a string for strstr, sscanf etc.
								obj->json_data_raw[received_len] = '\0';
								// Parse message frame
								obj->Frame_mode = (FrameMode)parse_command(obj,obj->json_data_raw);
							
						}
				}
		}else{
			obj->Detection_WonderLLM = 0;
	}
}

/**
 * @brief Send action execution success response
 * @note System command, see communication protocol
 */

void WonderLLM_Send_Action_Finish(void) {
    char json_str[72];
    snprintf(json_str, sizeof(json_str), "{\"command\":\"action_finish\",\"params\":\"true\"}");
    send_frame((uint8_t*)json_str, strlen(json_str));
}

/**
 * @brief Send status information
 * @note System command, see communication protocol
 */
void WonderLLM_Send_Status(const char* params_str) {
    char json_str[128];
    snprintf(json_str, sizeof(json_str), "{\"command\":\"status\",\"params\":%s}", params_str);
    send_frame((uint8_t*)json_str, strlen(json_str));
}


/**
 * @brief Request WonderLLM module to perform visual recognition
 * @param prompt String describing what you want the LLM to do
 * @note 1. System command, see communication protocol
 *       2. Prompt string must not contain double quotes or WonderLLM parsing will fail
 *       3. For long string transmission, I2C rate must be set to 400kHz using IIC_Config_MCP_Transmit,
 *         otherwise WonderLLM cannot receive complete data
 *       4. This function is invalid before module network setup is complete (emoji face appears)     
 */
void WonderLLM_Request_Vision(const char* prompt) {
    char json_str[256];
    
    // Safely build JSON string using snprintf
    snprintf(json_str, sizeof(json_str), 
             "{\"tool_name\":\"mcu.request\",\"command\":\"vision\",\"params\":\"%s\"}", 
             prompt);
		
		IIC_Config_MCP_Transmit();				
						
    delay_ms(5);
		
    // Call existing send_frame function to send this request
    send_frame((uint8_t*)json_str, strlen(json_str));			

}

/**
 * @brief Parse received JSON command string
 */

static int parse_command(WonderLLM_Info *obj,const char* json_str) {
	
		 /* Check if frame contains "move" for self.robot.move type */
		 if (strstr(json_str, "move") != NULL) {
				
        char* dist_ptr;
				// Scan for "move" field
        dist_ptr = strstr(json_str, "move");
				// Extract data from "move" field
				if (dist_ptr) sscanf(dist_ptr, "%*[^:]:%hhd", &(obj->movement_direction));

				// Scan for "step_num" field
        dist_ptr = strstr(json_str, "step_num");
				// Extract data from "step_num" field
				if (dist_ptr) sscanf(dist_ptr, "%*[^:]:%hd", &(obj->motion_target_step));		

        dist_ptr = strstr(json_str, "duration");
				// Extract data from "duration" field
				if (dist_ptr) sscanf(dist_ptr, "%*[^:]:%hd", &(obj->motion_target_RunningTime));

        dist_ptr = strstr(json_str, "distance");
				// Extract data from "duration" field
				if (dist_ptr) sscanf(dist_ptr, "%*[^:]:%hd", &(obj->motion_target_distance));

        dist_ptr = strstr(json_str, "angle");
				// Extract data from "duration" field
				if (dist_ptr) sscanf(dist_ptr, "%*[^:]:%hd", &(obj->motion_target_RotationAngle));

				return Frame_move;
     }

		 /* Check if frame contains "status_name" for get_status type */
		 /* Note 1: get_status detection must precede set_mode detection,
		            otherwise get_status frames containing running_mode field
		            would be misidentified as set_mode type */
		 /* Note 2: get_status detection must precede BaryCenterMove detection, same reason */
     else if (strstr(json_str, "status_name") != NULL) {
				 FrameMode mode = Frame_NULL; 
			 
         if (strstr(json_str, "battery")) {

						 mode = Frame_get_status_battery;
         }
				 if (strstr(json_str, "Speed_Mode")) {

						 mode = Frame_get_status_Speed_mode;
         }
				 if (strstr(json_str, "distance")) {

						 mode = Frame_get_status_distance;
         }
				 if (strstr(json_str, "running_mode")) {

						 mode = Frame_get_status_running_mode;
         }
				 if (strstr(json_str, "Bodystate")) {

						 mode = Frame_get_status_Bodystate;
         }
				 if (strstr(json_str, "poseture")) {
						 mode = Frame_get_status_poseture;
         }

				 return mode;
     }

		 /* Check if frame contains "pose" for BaryCenterMove type */
		 else if (strstr(json_str, "pose") != NULL) {
				
        char* dist_ptr;
				// Scan for "pose" field
        dist_ptr = strstr(json_str, "pose");
				// Extract data from "pose" field
				if (dist_ptr) {
					sscanf(dist_ptr, "%*[^:]:%hhd", &(obj->BaryCenter_direction));
				}
        
				return Frame_BaryCenterMove;
     }

		 /* Check if frame contains "Inclination" for InclinationAngleMove type */
		 else if (strstr(json_str, "Inclination") != NULL) {
				
        char* dist_ptr;
				// Scan for "Inclination" field
        dist_ptr = strstr(json_str, "Inclination");
				// Extract data from "Inclination" field
				if (dist_ptr) {
					sscanf(dist_ptr, "%*[^:]:%hhd", &(obj->incline_direction));
				}
        
				return Frame_InclinationAngleMove;
     }

		//  /* Check if frame contains "stop" for self.robot.stop type */
		//  if (strstr(json_str, "stop") != NULL) {
		// 		return Frame_stop;
		//  }

		 /* Check if frame contains "running_mode" for set_SpeedMode/set_RunningMode type */
		 else if (strstr(json_str, "running_mode") != NULL) {
			 if (strstr(json_str, "normal")) {
					obj->running_mode = 1; 
					obj->avoid_distance = 0;
					
			 }else{
					if (strstr(json_str, "intelligent_avoidance")) {
							obj->running_mode = 2;
					} 
					
					if (strstr(json_str, "intelligent_track")) {
							obj->running_mode = 3;
					}

					char* dist_ptr;
					// Scan for "distance" field
					dist_ptr = strstr(json_str, "distance");
					// Extract data from "distance" field
					if (dist_ptr) {
						sscanf(dist_ptr, "%*[^:]:%hu", &(obj->avoid_distance));
					}				 				
			 }
 

				 
			 return Frame_set_running_mode;
		   
     }

		 /* Check if frame contains "running_mode" for set_SpeedMode/set_RunningMode type */
		 else if (strstr(json_str, "speed_mode") != NULL) {
			 if (strstr(json_str, "Low_speed")) {
					obj->Speed_Mode = 1; 
			 }  
			 
			 if (strstr(json_str, "High_speed")) {
					 obj->Speed_Mode = 2;
			 } 
				 
			 return Frame_set_speed_mode;
		   
     }
		
		 /* Check if frame contains "lr" for set_led_color type */
		 else if (strstr(json_str, "\"lr\"") != NULL) {

  			char* ptr;
        
        ptr = strstr(json_str, "\"lr\"");
				if (ptr) sscanf(ptr, "%*[^:]:%hhu", &(obj->rgb_left[0]));

				ptr = strstr(json_str, "\"lg\"");
				if (ptr) sscanf(ptr, "%*[^:]:%hhu", &(obj->rgb_left[1]));

				ptr = strstr(json_str, "\"lb\"");
				if (ptr) sscanf(ptr, "%*[^:]:%hhu", &(obj->rgb_left[2]));

				ptr = strstr(json_str, "\"rr\"");
				if (ptr) sscanf(ptr, "%*[^:]:%hhu", &(obj->rgb_right[0]));

				ptr = strstr(json_str, "\"rg\"");
				if (ptr) sscanf(ptr, "%*[^:]:%hhu", &(obj->rgb_right[1]));

				ptr = strstr(json_str, "\"rb\"");
				if (ptr) sscanf(ptr, "%*[^:]:%hhu", &(obj->rgb_right[2]));

        // Assign parsed values to global RGB arrays
        // Perform range check and type conversion
        obj->rgb_left[0] = (obj->rgb_left[0] > 255) ? 255 : ((obj->rgb_left[0] < 0) ? 0 : obj->rgb_left[0]);
        obj->rgb_left[1] = (obj->rgb_left[1] > 255) ? 255 : ((obj->rgb_left[1] < 0) ? 0 : obj->rgb_left[1]);
        obj->rgb_left[2] = (obj->rgb_left[2] > 255) ? 255 : ((obj->rgb_left[2] < 0) ? 0 : obj->rgb_left[2]);

        obj->rgb_right[0] = (obj->rgb_right[0] > 255) ? 255 : ((obj->rgb_right[0] < 0) ? 0 : obj->rgb_right[0]);
        obj->rgb_right[1] = (obj->rgb_right[1] > 255) ? 255 : ((obj->rgb_right[1] < 0) ? 0 : obj->rgb_right[1]);
        obj->rgb_right[2] = (obj->rgb_right[2] > 255) ? 255 : ((obj->rgb_right[2] < 0) ? 0 : obj->rgb_right[2]);

				return Frame_set_led_color;
    }
		
		 /* Check if frame contains "count" for set_buzzer type */
		 else if (strstr(json_str, "count") != NULL) {				
		 	 char* ptr;			 
			 ptr = strstr(json_str, "\"count\"");		 
			 if (ptr){
			 	sscanf(ptr, "%*[^:]:%hhd", &(obj->buzzer_count));
			 } 
			 return Frame_set_buzzer;
     }

		 /* Check if frame contains "actionNum" for ActionGroup type */
		 else if (strstr(json_str, "actionNum") != NULL) {
		 	 char* ptr;
			 ptr = strstr(json_str, "\"actionNum\"");
			 if (ptr) sscanf(ptr, "%*[^:]:%hhd", &(obj->ActionNum));

			 ptr = strstr(json_str, "\"executeNum\"");
			 if (ptr) sscanf(ptr, "%*[^:]:%hhd", &(obj->ExecuteActionNum));

			 return Frame_ActionGroup;
     }

		 /* Check if frame contains "vision" for system type */
		 else if (strstr(json_str, "vision") != NULL) {
				// if (strstr(json_str, "true") != NULL) {
				// 	obj->Vision_Result = 1;
				// }else{
				// 	obj->Vision_Result = 0;
				// }

			  return Frame_vision_analysis;
     }

		//  /* Check if frame contains "reply" for system type */
		//  else if (strstr(json_str, "reply") != NULL) {

		// 	 return Frame_reply;
    //  }		
		 return Frame_NULL;		 
}

/**
 * @brief Register all available tools (functions) with WonderLLM module
 * @note  For long string transmission, I2C rate must be set to 400kHz using
 *        IIC_Config_MCP_Transmit, otherwise WonderLLM cannot receive complete data
 */
static bool register_tools(void) {

/*
    Register custom MCP tools:
    tool_name: Registered MCP tool name (must start with `self`, use clear naming so the LLM understands its purpose)
    command: Guide the LLM on when to use this tool, describe parameters to return to host
    params: Parameters in the message frame from WonderLLM to host.
            Format: [[param_name, param_type, min(optional), max(optional)], [param2...]]
            (string type = content determined by LLM based on conversation context)
            (Supported param types: string, int, bool)
    return: Whether host needs to return data to WonderLLM
            true - required, when true block setting is ignored (return is inherently blocking)
                   (use system command status, see WonderLLM_Send_Status)
            false - not required
    block: Whether execution is blocking. If yes, WonderLLM waits for host confirmation before continuing
           true - host must reply command completion after executing
                  (use system command action_finish, see WonderLLM_Send_Action_Finish)
           false - WonderLLM sends command and continues without waiting for result
*/


		if (!send_frame((uint8_t*)tool_buzzer, strlen(tool_buzzer))){
			return false;
		} 

		if (!send_frame((uint8_t*)tool_led, strlen(tool_led))){
			return false;
		}
		
		// if (!send_frame((uint8_t*)tool_stop, strlen(tool_stop))){
		// 	return false;
		// }

		if (!send_frame((uint8_t*)tool_SpeedMode, strlen(tool_SpeedMode))){
			return false;
		}
	
		if (!send_frame((uint8_t*)tool_RunningMode, strlen(tool_RunningMode))){
			return false;
		}
	
		if (!send_frame((uint8_t*)tool_status, strlen(tool_status))){
			return false;
		}

		if (!send_frame((uint8_t*)tool_move, strlen(tool_move))){
			return false;
		}

		if (!send_frame((uint8_t*)tool_BaryCenterMove, strlen(tool_BaryCenterMove))){
			return false;
		}

		if (!send_frame((uint8_t*)tool_InclinationAngleMove, strlen(tool_InclinationAngleMove))){
			return false;
		}

		if (!send_frame((uint8_t*)tool_ActionGroup, strlen(tool_ActionGroup))){
			return false;
		}

		/* System command - MCP registration complete */		
    // static const char* tool_finish = "{\"command\":\"mcp_setting\",\"params\":\"true\"}";	
		if (!send_frame((uint8_t*)tool_finish, strlen(tool_finish))){
			return false;
		}
    
    return true;
}

/**
 * @brief Send data frame via I2C
 */
static bool send_frame(const uint8_t* data, uint16_t len) {

	   // Check if data and length are valid
    if (data == NULL || len == 0) {
        return false;
    }
		
		// Send data pointed to by data pointer, length = len
    if(WonderLLM_Send_Data((uint8_t *)data,len) !=0){
			return false;
		} 

		return true;
		
}

/**
 * @brief Receive 8-byte frame header
 */
static bool receive_frame_head(uint16_t* part_ID, uint16_t* part_num, uint16_t* data_len) {
		uint8_t header[8];

		/* step1: Receive header+length and verify */
    if(WonderLLM_Receive_Data(header, 8, true) != 0){
			return false;
		}

		/* step1.1: Verify frame header */
    if (header[0] != 0xAA || header[1] != 0x55) return false;
		delay_ms(1);
    
		/* step1.2: Parse frame length and verify validity */
    *data_len = ((uint16_t)header[2] << 8) | header[3];

    if (*data_len == 0|| *data_len > 31){
			Serial.println(F("data abnormal"));
			delay_ms(100);
			return false;
		} 

		#if (debug_mode ==1)
			if(*data_len != 1){
				// Toggle pin for logic analyzer debugging of JSON data reception timing
				digitalWrite(6, HIGH);
				Serial.print("rec_dataLen:");
				Serial.println(*data_len);
				digitalWrite(6, LOW);
			}
		#endif

		/* step1.3: Parse total fragments and current fragment ID */
		*part_ID = ((uint16_t)header[5] << 8) | header[4]; 
		*part_num = ((uint16_t)header[7] << 8) | header[6];

		return true;
}

/**
 * @brief Receive a complete data frame from I2C
 */
static bool receive_frame(uint8_t* buffer, uint16_t* len) {
		uint16_t data_len = 0;
		uint16_t data_len_max = 0;
		uint16_t part_ID = 0;
		uint16_t part_num = 0;
		uint16_t buffer_index = 0;
		uint16_t part_ID_temp = 0;
		uint16_t part_num_temp = 0;

		data_len_max = *len;
		*len =0;

		/* step1: Receive frame header and verify */
		if(receive_frame_head(&part_ID, &part_num, &data_len)){
			*len += data_len;
		}else{
			Serial.println(F("receive_frame_fail"));
			return false;
		}

		/* step2: Receive frame data + checksum */
		// If current packet is not the first, packets were lost - exit
		if(part_ID != 1){
			Serial.println(F("receive_frame_not_first"));
			return false;

		}else{  // Start receiving all data from first packet

			uint8_t result =0;

				// data_len (data length) + 1 (checksum)
				// Each packet's checksum is overwritten when receiving the next packet
				// Last packet's checksum is overwritten by null terminator
			for(int i=1; i<=part_num; i++){

				delay_ms(10);

				result = WonderLLM_Receive_Data((buffer + buffer_index), data_len + 1, true);			

				// Error handling - check if read was successful
				if(result != 0){
					Serial.print("rec_result:");
					Serial.println(result);
					Serial.println(F("receive error"));
					return false;
				}

				// Verify checksum validity
				if (buffer[buffer_index + data_len] == calculate_checksum((buffer + buffer_index), data_len)){

				#if(debug_mode ==1)					
						Serial.print(F("data:"));
						for(uint8_t t=0;t < data_len;t++){
							Serial.print((int)buffer[buffer_index + t]);
							Serial.print(' ');
						}
						Serial.println();
				#endif

					buffer_index += data_len;

				}else{ // Checksum invalid, clear stored data and exit

					Serial.println(F("calculate error"));

					#if(debuf_mode ==1)
						/* Debug print */
						for(int i=0;i<=(*len);i++){
							Serial.print(buffer[i]);
							Serial.print(' ');
						}
						Serial.println();
					#endif

					memset(buffer,0,sizeof(buffer));
					return false;
				}

				// Receive next packet header for length/slice info
				if(i < part_num){

					delay_ms(100);

					if(receive_frame_head(&part_ID_temp, &part_num_temp, &data_len)){
						*len += data_len;
					}else{
						return false;
					}

					/* Verify next packet fragment ID and total fragments */
					// Fragment ID not consecutive, packet lost - exit
					if(( (part_ID + 1) != part_ID_temp ) || (part_num_temp != part_num) ){
						Serial.println(part_ID);
						Serial.println(part_ID_temp);
						Serial.println(F("data packet dropout"));
						return false;
					}else{
						part_ID += 1;
					}

					/* Overflow protection to prevent out-of-bounds write */
					if((buffer_index + data_len + 1) > data_len_max ){
						Serial.println(F("Insufficient buffer space"));
						return false;
					}
				
				}

			}

			return true;			

		}

    return false;
}

/**
 * @brief Calculate XOR checksum of data
 */
static uint8_t calculate_checksum(const uint8_t* data, uint16_t len) {
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}