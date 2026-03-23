#include "hiwonder_robot.h"
#include <string.h>
#include <Wire.h>
#include "WonderLLM.h"

#define Move_stride_LowSpeed_1 70       // Low speed mode forward/backward single-step stride (unit: mm)
#define Move_stride_HighSpeed_1 85      // High speed mode forward/backward single-step stride (unit: mm)
#define Move_stride_LowSpeed_2 65       // Low speed mode left/right single-step stride (unit: mm)
#define Move_stride_HighSpeed_2 75      // High speed mode left/right single-step stride (unit: mm)
#define Move_stride_LowSpeed_3 90       // Low speed mode diagonal single-step stride (unit: mm)
#define Move_stride_HighSpeed_3 90      // High speed mode diagonal single-step stride (unit: mm)
#define Rotation_angle_LowSpeed 24      // Low speed mode single-step rotation angle (unit: degrees)
#define Rotation_angle_HighSpeed 45     // High speed mode single-step rotation angle (unit: degrees)

extern WonderLLM_Info WonderLLM_hiwonder;
static char info[128];

Robot minihexa;

uint8_t val[4];
uint8_t rgb1[3] = {0,0,255};
uint8_t rgb2[3] = {0,0,255};

Velocity_t vel = {0.0f,0.0f,0.0f};
Vector_t pos = {0.0f,0.0f,0.0f};
Euler_t att = {0.0f,0.0f,0.0f};

int timer = 0;
char sustain_move_flag = 0;

void setup() {
  delay(1000);
  Serial.begin(115200);
  minihexa.begin();
  minihexa.sensor.set_ultrasound_rgb(RGB_WORK_SOLID_MODE, rgb1, rgb2);
  WonderLLM_Init(); // Initialize WonderLLM module	
  WonderLLM_hiwonder.Speed_Mode = 1; // Initialize to low speed motion mode

  // Delay to ensure module is powered on and network configured (emoji face appears)
  delay(18000);

}
void loop() {
  // Get WonderLLM data
  WonderLLM_Info_Get(&WonderLLM_hiwonder); 
  //Serial.printf("%d\r\n",WonderLLM_hiwonder.Speed_Mode);
  if(WonderLLM_hiwonder.Frame_mode != Frame_NULL){
      sprintf(info,"raw str:%s\r\n",WonderLLM_hiwonder.json_data_raw);

      memset(WonderLLM_hiwonder.json_data_raw,0,sizeof(WonderLLM_hiwonder.json_data_raw));
      // Serial.print(info); 
    
      switch(WonderLLM_hiwonder.Frame_mode){
        case Frame_move:{
          sprintf(info,"Frame_move:direction:%d,step:%d,time:%d,distance:%d,angle:%d\r\n",WonderLLM_hiwonder.movement_direction,\
                                                                                          WonderLLM_hiwonder.motion_target_step,\
                                                                                          WonderLLM_hiwonder.motion_target_RunningTime,\
                                                                                          WonderLLM_hiwonder.motion_target_distance,\
                                                                                          WonderLLM_hiwonder.motion_target_RotationAngle);
          Serial.print(info);
          
          /* Execute user-defined motion control API */
          if(WonderLLM_hiwonder.motion_target_RotationAngle == -1 && WonderLLM_hiwonder.motion_target_distance == -1){ // No specified distance or rotation angle

              if(WonderLLM_hiwonder.motion_target_step == -1 && WonderLLM_hiwonder.motion_target_RunningTime == -1){ // No specified steps or duration, keep walking indefinitely
                sustain_move_flag = 1;

              }else if(WonderLLM_hiwonder.motion_target_step != -1 && WonderLLM_hiwonder.motion_target_RunningTime == -1){ // Specified steps but no duration
                timer = 600 * WonderLLM_hiwonder.motion_target_step; // Default 600ms per step

              }else if(WonderLLM_hiwonder.motion_target_step == -1 && WonderLLM_hiwonder.motion_target_RunningTime != -1){ // Specified steps but no duration
                timer = WonderLLM_hiwonder.motion_target_RunningTime;

              }else if(WonderLLM_hiwonder.motion_target_step != -1 && WonderLLM_hiwonder.motion_target_RunningTime != -1){ // Both steps and duration specified, conflicting params, duration takes priority
                timer = WonderLLM_hiwonder.motion_target_RunningTime;
              }

          }else if(WonderLLM_hiwonder.motion_target_RotationAngle != -1){ // Specified rotation angle

            // Convert rotation angle to steps, processed as "no distance/angle" -- "steps without duration"
            if(WonderLLM_hiwonder.Speed_Mode == 1){  // Low speed mode
                WonderLLM_hiwonder.motion_target_step = WonderLLM_hiwonder.motion_target_RotationAngle / Rotation_angle_LowSpeed;

            }else if(WonderLLM_hiwonder.Speed_Mode == 2){ // High speed mode
                WonderLLM_hiwonder.motion_target_step = WonderLLM_hiwonder.motion_target_RotationAngle / Rotation_angle_HighSpeed;
                
            }
            Serial.println(WonderLLM_hiwonder.motion_target_step);

            timer = 600 * WonderLLM_hiwonder.motion_target_step; // Default 600ms per step

          }else if(WonderLLM_hiwonder.motion_target_distance != -1){ // Specified movement distance

            // Convert distance to steps, processed as "no distance/angle" -- "steps without duration"
            if(WonderLLM_hiwonder.Speed_Mode == 1){  // Low speed mode
                switch(WonderLLM_hiwonder.movement_direction){
                  case 1: // Forward
                  case 5:{// Backward
                    WonderLLM_hiwonder.motion_target_step = WonderLLM_hiwonder.motion_target_distance / Move_stride_LowSpeed_1;
                    Serial.println(WonderLLM_hiwonder.motion_target_distance);
                    break;
                  }
                  case 3: // Right strafe
                  case 7:{// Left strafe
                    WonderLLM_hiwonder.motion_target_step = WonderLLM_hiwonder.motion_target_distance / Move_stride_LowSpeed_2;
                    break;
                  } 
                  case 2:// Front-right
                  case 4:// Rear-right
                  case 6:// Rear-left
                  case 8:{// Front-left
                    WonderLLM_hiwonder.motion_target_step = WonderLLM_hiwonder.motion_target_distance / Move_stride_LowSpeed_3;
                    break;
                  }
                }
                

            }else if(WonderLLM_hiwonder.Speed_Mode == 2){ // High speed mode
                switch(WonderLLM_hiwonder.movement_direction){
                  case 1: // Forward
                  case 5:{// Backward
                    WonderLLM_hiwonder.motion_target_step = WonderLLM_hiwonder.motion_target_distance / Move_stride_HighSpeed_1;
                    break;
                  }
                  case 3: // Right strafe
                  case 7:{// Left strafe
                    WonderLLM_hiwonder.motion_target_step = WonderLLM_hiwonder.motion_target_distance / Move_stride_HighSpeed_2;
                    break;
                  } 
                  case 2:// Front-right
                  case 4:// Rear-right
                  case 6:// Rear-left
                  case 8:{// Front-left
                    WonderLLM_hiwonder.motion_target_step = WonderLLM_hiwonder.motion_target_distance / Move_stride_HighSpeed_3;
                    break;
                  }
                }

            }
            timer = 600 * WonderLLM_hiwonder.motion_target_step; // Default 600ms per step

          }else{ // Both distance and angle specified, conflicting params, robot skips this command
              WonderLLM_hiwonder.motion_target_step = 0;
              WonderLLM_hiwonder.motion_target_RunningTime = 0;
          }



          switch(WonderLLM_hiwonder.movement_direction){
            case 1:{ // Forward
              Serial.println(36);
              if(WonderLLM_hiwonder.motion_target_step == 0 && WonderLLM_hiwonder.motion_target_RunningTime == 0){ // Current command: robot stop
                sustain_move_flag = 0;
              }
              vel = {0.0f,2.0f*WonderLLM_hiwonder.Speed_Mode,0.0f};              
              break;
            }
            case 2:{ // Front-right
              vel = {2.0f*WonderLLM_hiwonder.Speed_Mode,2.0f*WonderLLM_hiwonder.Speed_Mode,0.0f};
              break;
            }
            case 3:{ // Right
              vel = {2.0f*WonderLLM_hiwonder.Speed_Mode,0.0f,0.0f};
              break;
            }
            case 4:{ // Rear-right
              vel = {2.0f*WonderLLM_hiwonder.Speed_Mode,-2.0f*WonderLLM_hiwonder.Speed_Mode,0.0f};
              break;
            }
            case 5:{ // Backward
              vel = {0.0f,-2.0f*WonderLLM_hiwonder.Speed_Mode,0.0f};
              break;
            }
            case 6:{ // Rear-left
              vel = {-2.0f*WonderLLM_hiwonder.Speed_Mode,-2.0f*WonderLLM_hiwonder.Speed_Mode,0.0f};
              break;
            }
            case 7:{ // Left
              vel = {-2.0f*WonderLLM_hiwonder.Speed_Mode,0.0f,0.0f};
              break;
            }
            case 8:{ // Front-left
              vel = {-2.0f*WonderLLM_hiwonder.Speed_Mode,2.0f*WonderLLM_hiwonder.Speed_Mode,0.0f};
              break;
            }
            case 9:{ // Turn left in place
              vel = {0.0f,0.0f,2.0f*WonderLLM_hiwonder.Speed_Mode};
              break;
            }
            case 10:{ // Turn right in place
              vel = {0.0f,0.0f,-2.0f*WonderLLM_hiwonder.Speed_Mode};
              break;
            }

          }
          pos = {0.0f,0.0f,0.0f};
          att = {0.0f,0.0f,0.0f};
          
          //Serial.printf("%f %f %f\r\n",vel.vx,vel.vy,vel.omega);
          minihexa.move(&vel, &pos, &att, 600, WonderLLM_hiwonder.motion_target_step);

          if(sustain_move_flag != 1){
            delay(timer);
            vel = {0.0f,0.0f,0.0f};
            minihexa.move(&vel, &pos, &att);
          } 
          
          // This MCP tool executes in blocking mode (block=true), call action_finish system command when done
          WonderLLM_Send_Action_Finish();							
          break;
        }

        case Frame_BaryCenterMove:{
          sprintf(info,"Frame_BaryCenterMove:direction:%d\r\n",WonderLLM_hiwonder.BaryCenter_direction);
          Serial.print(info);
          
          /* Execute user-defined motion control API */
          switch(WonderLLM_hiwonder.BaryCenter_direction){
            case 1:{ // Forward 
              pos = {0.0f,3.0f,0.0f};
              break;
            }
            case 2:{ // Front-right
              pos = {3.0f,3.0f,0.0f};
              break;
            }
            case 3:{ // Right
              pos = {3.0f,0.0f,0.0f};
              break;
            }
            case 4:{ // Rear-right
              pos = {3.0f,-3.0f,0.0f};
              break;
            }
            case 5:{ // Backward
              pos = {0.0f,-3.0f,0.0f};
              break;
            }
            case 6:{ // Rear-left
              pos = {-3.0f,-3.0f,0.0f};
              break;
            }
            case 7:{ // Left
              pos = {-3.0f,0.0f,0.0f};
              break;
            }
            case 8:{ // Front-left
              pos = {-3.0f,3.0f,0.0f};
              break;
            }
            case 9:{ // Up
              pos = {0.0f,0.0f,3.0f};
              break;
            }
            case 10:{ // Down
              pos = {0.0f,0.0f,-1.5f};
              break;
            }
            case 11:{ // Restore initial posture
              pos = {0.0f,0.0f,0.0f};
              break;
            }
          }
          vel = {0.0f,0.0f,0.0f};
          att = {0.0f,0.0f,0.0f};
          minihexa.move(&vel, &pos, &att);

          // This MCP tool executes in blocking mode (block=true), call action_finish system command when done
          WonderLLM_Send_Action_Finish();							
          break;
        }

        case Frame_InclinationAngleMove:{
          sprintf(info,"Frame_InclinationAngleMove:direction:%d\r\n",WonderLLM_hiwonder.incline_direction);
          Serial.print(info);
          
          /* Execute user-defined motion control API */
          switch(WonderLLM_hiwonder.incline_direction){
            case 1:{ // Tilt forward
              att = {12.0f,0.0f,0.0f};
              break;
            }
            case 2:{ // Tilt backward
              att = {-12.0f,0.0f,0.0f};
              break;
            }
            case 3:{ // Tilt left
              att = {0.0f,15.0f,0.0f};
              break;
            }
            case 4:{ // Tilt right
              att = {0.0f,-15.0f,0.0f};
              break;
            }
            case 5:{ // Twist left
              att = {0.0f,0.0f,-15.0f};
              break;
            }
            case 6:{ // Twist right
              att = {0.0f,-0.0f,15.0f};
              break;
            }
          }
          vel = {0.0f,0.0f,0.0f};
          pos = {0.0f,0.0f,0.0f};
          minihexa.move(&vel, &pos, &att);

          // This MCP tool executes in blocking mode (block=true), call action_finish system command when done
          WonderLLM_Send_Action_Finish();							
          break;
        }

        // case Frame_stop:{
        //   sprintf(info,"Frame_stop:robot reset\r\n");
        //   Serial.print(info);

        //   vel = {0.0f,0.0f,0.0f};
        //   pos = {0.0f,0.0f,0.0f};
        //   att = {0.0f,0.0f,0.0f};
        //   minihexa.move(&vel, &pos, &att);
        //   minihexa.reset();
        //   WonderLLM_hiwonder.running_mode = 1;

        //   // This MCP tool executes in blocking mode (block=true), call action_finish system command when done
        //   WonderLLM_Send_Action_Finish();							
        //   break;          
        // }
        
        case Frame_get_status_battery:{
          sprintf(info, "[\"battery\",\"%d\",\"mV\"]", minihexa.board.bat_voltage);
          
          // // This MCP tool returns params to WonderLLM (return=true), call status command with formatted params when done
          WonderLLM_Send_Status(info);
          break;
        }

        case Frame_get_status_Bodystate:{
          float BodyPosition[3];
          float euler_angle[3];
          minihexa.board.get_imu_euler(euler_angle);
          minihexa.read_position(BodyPosition);
          if(fabs(euler_angle[0]) > 150.0f){  // Robot fell over, roll angle absolute value > 150 degrees
            sprintf(info, "[\"Bodystate\",\"2\"]");
          }else{
            sprintf(info, "[\"Bodystate\",\"%d\"]", (BodyPosition[2] < 0) ? 1 : 0);
          }
          
          
          // // This MCP tool returns params to WonderLLM (return=true), call status command with formatted params when done
          WonderLLM_Send_Status(info);
          break;
        }

        case Frame_get_status_poseture:{
          float euler_angle[3];
          minihexa.board.get_imu_euler(euler_angle);
          sprintf(info, "[[\"roll\",\"%.1f\"],[\"pitch\",\"%.1f\"]]",euler_angle[0],euler_angle[1]);
          
          // // This MCP tool returns params to WonderLLM (return=true), call status command with formatted params when done
          WonderLLM_Send_Status(info);
          break;
        }

        case Frame_get_status_Speed_mode:{
          sprintf(info, "[\"Speed_mode\",\"%d\"]", WonderLLM_hiwonder.Speed_Mode);
          
          // This MCP tool returns params to WonderLLM (return=true), call status command with formatted params when done
          WonderLLM_Send_Status(info);						
          break;
        }
        
        case Frame_get_status_distance:{       
          sprintf(info, "[\"distance\",\"%d\",\"mm\"]", minihexa.sensor._get_distance());
      
          // This MCP tool returns params to WonderLLM (return=true), call status command with formatted params when done
          WonderLLM_Send_Status(info);
          break;
        }
        
        case Frame_get_status_running_mode:{
          sprintf(info, "[\"running_mode\",\"%d\"]", WonderLLM_hiwonder.running_mode);
          
          // This MCP tool returns params to WonderLLM (return=true), call status command with formatted params when done
          WonderLLM_Send_Status(info);						
          break;
        }

        case Frame_set_running_mode:{
          sprintf(info,"Frame_set_running_mode:%d,distance:%d\r\n",WonderLLM_hiwonder.running_mode,WonderLLM_hiwonder.avoid_distance);
          Serial.print(info);

          vel = {0.0f,0.0f,0.0f};
          pos = {0.0f,0.0f,0.0f};
          att = {0.0f,0.0f,0.0f};
          minihexa.move(&vel, &pos, &att); 

          // This MCP tool executes in blocking mode (block=true), call action_finish system command when done
          WonderLLM_Send_Action_Finish();					
          break;
        }
        
        case Frame_set_led_color:{
          sprintf(info,"Frame_set_RGB: Left:%d,%d,%d,Right:%d,%d,%d\r\n",\
                WonderLLM_hiwonder.rgb_left[0],WonderLLM_hiwonder.rgb_left[1],WonderLLM_hiwonder.rgb_left[2],\
                WonderLLM_hiwonder.rgb_right[0],WonderLLM_hiwonder.rgb_right[1],WonderLLM_hiwonder.rgb_right[2]);
          Serial.print(info);
          
          /* Execute user-defined RGB LED control API */
          minihexa.sensor.set_ultrasound_rgb(RGB_WORK_SOLID_MODE, WonderLLM_hiwonder.rgb_left, WonderLLM_hiwonder.rgb_right);
          
          // This MCP tool executes in blocking mode (block=true), call action_finish system command when done
          WonderLLM_Send_Action_Finish();
          break;
        }
        
        case Frame_set_buzzer:{
          sprintf(info,"Frame_set_buzzer:%d\r\n",WonderLLM_hiwonder.buzzer_count);
          Serial.print(info);
          
          /* Execute user-defined buzzer control API */
          for(int i = 0; i < WonderLLM_hiwonder.buzzer_count; i++){
            ledcWriteTone(LEDC_CHANNEL_0, 3000);
            delay(200);
            ledcWriteTone(LEDC_CHANNEL_0, 0);
            delay(500);
          }
          
          // This MCP tool executes in blocking mode (block=true), call action_finish system command when done
          WonderLLM_Send_Action_Finish();
          break;
        }

        case Frame_set_speed_mode:{ 
          sprintf(info,"Frame_set_speed_mode:%d\r\n",WonderLLM_hiwonder.Speed_Mode);
          Serial.print(info);

          // This MCP tool executes in blocking mode (block=true), call action_finish system command when done
          WonderLLM_Send_Action_Finish();
          break;						
        }

        case Frame_ActionGroup:{ 
          sprintf(info,"Frame_ActionGroup:index%d time:%d\r\n",WonderLLM_hiwonder.ActionNum,WonderLLM_hiwonder.ExecuteActionNum);
          Serial.print(info);

          // Send reply before executing action group to prevent WonderLLM timeout
          // This MCP tool executes in blocking mode (block=true), call action_finish system command when done
          WonderLLM_Send_Action_Finish();

          for(int i = 0; i < WonderLLM_hiwonder.ExecuteActionNum; i++){
            delay(1000);
            switch(WonderLLM_hiwonder.ActionNum){
              case 3:{ // Wake up action
                minihexa.wake_up();
                break;
              }
              case 4:{ // Wake up running action
                minihexa._wake_up();
                break;
              }
              case 5:{ // Cute/playful action
                minihexa.acting_cute();
                break;
              }
              case 6:   // Obstacle crossing action
              case 7:   // Left leg fighting action
              case 8:   // Right leg fighting action
              case 9:   // Left foot forward kick action
              case 10:  // Left foot right kick action
              case 11:  // Right foot forward kick action
              case 12:  // Right foot left kick action 
              case 13:  // Push door action
              case 14:{ // Wave hand action
                minihexa.action_group_run(WonderLLM_hiwonder.ActionNum);
                break;
              }                                                                                                                          
            }

          }

          break;						
        }

        case Frame_vision_analysis:{
          // sprintf(info,"Frame_vision_analysis:result:%d\r\n",WonderLLM_hiwonder.Vision_Result);
          // Serial.print(info);

          // if(WonderLLM_hiwonder.Vision_Result == 1){
          //   minihexa.action_group_run(14);
          //   WonderLLM_hiwonder.Vision_Result = 0;
          // }

          // Lower I2C rate to 100kHz
          /* Optional: if all I2C devices support 400kHz, no need to switch back
             to 100kHz. This is for compatibility with slower I2C devices */
          IIC_Config_normal_Transmit();	
          break;
        }

        default:{ 
          //do nothing...
          break;						
        }
      }
  }
  
  if(WonderLLM_hiwonder.running_mode == 1){
    // loop_timer++;
    // if(loop_timer%50 == 0){
    //   WonderLLM_Request_Vision(vision_prompt);
    //   loop_timer = 0;
    // }

  }else if(WonderLLM_hiwonder.running_mode == 2){
    WonderLLM_hiwonder.avoid_distance = (WonderLLM_hiwonder.avoid_distance < 200) ?200 : WonderLLM_hiwonder.avoid_distance; 
    minihexa._avoid(minihexa.sensor._get_distance(), WonderLLM_hiwonder.avoid_distance);

  }else if(WonderLLM_hiwonder.running_mode == 3){
    WonderLLM_hiwonder.avoid_distance = (WonderLLM_hiwonder.avoid_distance < 200) ?200 : WonderLLM_hiwonder.avoid_distance;

    int dis = minihexa.sensor._get_distance();
    Serial.println(dis);
    if(dis > (WonderLLM_hiwonder.avoid_distance + 50)){
      vel = {0.0f,2.0f,0.0f};
    }else if(dis < (WonderLLM_hiwonder.avoid_distance - 50)){
      vel = {0.0f,-2.0f,0.0f};
    }else{
      vel = {0.0f,0.0f,0.0f};
    }
    pos = {0.0f,0.0f,0.0f};
    att = {0.0f,0.0f,0.0f};
    minihexa.move(&vel, &pos, &att,500,2);    

  }

  delay_ms(100);
  // Serial.println(loop_timer);
}

