#include "hiwonder_robot.h"
#include <string.h>
#include <Wire.h>
#include "WonderLLM.h"

#define Move_stride_LowSpeed_1 70       //低速模式下前后运动单步平移步幅（单位：毫米）
#define Move_stride_HighSpeed_1 85      //高速模式下前后运动单步平移步幅（单位：毫米）
#define Move_stride_LowSpeed_2 65       //低速模式下左右运动单步平移步幅（单位：毫米）
#define Move_stride_HighSpeed_2 75      //高速模式下左右运动单步平移步幅（单位：毫米）
#define Move_stride_LowSpeed_3 90       //低速模式下斜向运动单步平移步幅（单位：毫米）
#define Move_stride_HighSpeed_3 90      //高速模式下斜向运动单步平移步幅（单位：毫米）
#define Rotation_angle_LowSpeed 24      //低速模式下单步旋转角度（单位：度）
#define Rotation_angle_HighSpeed 45     //高速模式下单步旋转角度（单位：度）

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
  WonderLLM_Init(); //初始化WonderLLM模块	
  WonderLLM_hiwonder.Speed_Mode = 1; //初始化为低速运动模式

  //延时一段时间，确保模块已经上电，并完成配网（出现表情界面）
  delay(18000);

}
void loop() {
  //获取WonderLLM数据
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
          
          /* 执行用户自定义运动控制API */
          if(WonderLLM_hiwonder.motion_target_RotationAngle == -1 && WonderLLM_hiwonder.motion_target_distance == -1){ //不指定运动距离和运动角度

              if(WonderLLM_hiwonder.motion_target_step == -1 && WonderLLM_hiwonder.motion_target_RunningTime == -1){ //不指定步数和运动时长，默认一直走不停下
                sustain_move_flag = 1;

              }else if(WonderLLM_hiwonder.motion_target_step != -1 && WonderLLM_hiwonder.motion_target_RunningTime == -1){ //指定运动步数但不指定运动时长
                timer = 600 * WonderLLM_hiwonder.motion_target_step; //迈一步默认耗时600ms

              }else if(WonderLLM_hiwonder.motion_target_step == -1 && WonderLLM_hiwonder.motion_target_RunningTime != -1){ //指定运动步数但不指定运动时长
                timer = WonderLLM_hiwonder.motion_target_RunningTime;

              }else if(WonderLLM_hiwonder.motion_target_step != -1 && WonderLLM_hiwonder.motion_target_RunningTime != -1){ //同时指定运动步数和运动时长，两个参数相冲突，以运动时长为准
                timer = WonderLLM_hiwonder.motion_target_RunningTime;
              }

          }else if(WonderLLM_hiwonder.motion_target_RotationAngle != -1){ //指定运动角度

            //将运动角度换算为运动步数，按照 “不指定运动距离和运动角度” -- “指定运动步数但不指定运动时长” 处理
            if(WonderLLM_hiwonder.Speed_Mode == 1){  //低速模式
                WonderLLM_hiwonder.motion_target_step = WonderLLM_hiwonder.motion_target_RotationAngle / Rotation_angle_LowSpeed;

            }else if(WonderLLM_hiwonder.Speed_Mode == 2){ //高速模式
                WonderLLM_hiwonder.motion_target_step = WonderLLM_hiwonder.motion_target_RotationAngle / Rotation_angle_HighSpeed;
                
            }
            Serial.println(WonderLLM_hiwonder.motion_target_step);

            timer = 600 * WonderLLM_hiwonder.motion_target_step; //迈一步默认耗时600ms

          }else if(WonderLLM_hiwonder.motion_target_distance != -1){ //指定运动距离

            //将运动距离换算为运动步数，按照 “不指定运动距离和运动角度” -- “指定运动步数但不指定运动时长” 处理
            if(WonderLLM_hiwonder.Speed_Mode == 1){  //低速模式
                switch(WonderLLM_hiwonder.movement_direction){
                  case 1: //前进
                  case 5:{//后退
                    WonderLLM_hiwonder.motion_target_step = WonderLLM_hiwonder.motion_target_distance / Move_stride_LowSpeed_1;
                    Serial.println(WonderLLM_hiwonder.motion_target_distance);
                    break;
                  }
                  case 3: //右平移
                  case 7:{//左平移
                    WonderLLM_hiwonder.motion_target_step = WonderLLM_hiwonder.motion_target_distance / Move_stride_LowSpeed_2;
                    break;
                  } 
                  case 2://右前
                  case 4://右后
                  case 6://左后
                  case 8:{//左前
                    WonderLLM_hiwonder.motion_target_step = WonderLLM_hiwonder.motion_target_distance / Move_stride_LowSpeed_3;
                    break;
                  }
                }
                

            }else if(WonderLLM_hiwonder.Speed_Mode == 2){ //高速模式
                switch(WonderLLM_hiwonder.movement_direction){
                  case 1: //前进
                  case 5:{//后退
                    WonderLLM_hiwonder.motion_target_step = WonderLLM_hiwonder.motion_target_distance / Move_stride_HighSpeed_1;
                    break;
                  }
                  case 3: //右平移
                  case 7:{//左平移
                    WonderLLM_hiwonder.motion_target_step = WonderLLM_hiwonder.motion_target_distance / Move_stride_HighSpeed_2;
                    break;
                  } 
                  case 2://右前
                  case 4://右后
                  case 6://左后
                  case 8:{//左前
                    WonderLLM_hiwonder.motion_target_step = WonderLLM_hiwonder.motion_target_distance / Move_stride_HighSpeed_3;
                    break;
                  }
                }

            }
            timer = 600 * WonderLLM_hiwonder.motion_target_step; //迈一步默认耗时600ms

          }else{ //同时指定运动距离和运动角度,两个参数冲突，机器人不执行本轮指令
              WonderLLM_hiwonder.motion_target_step = 0;
              WonderLLM_hiwonder.motion_target_RunningTime = 0;
          }



          switch(WonderLLM_hiwonder.movement_direction){
            case 1:{ //正前
              Serial.println(36);
              if(WonderLLM_hiwonder.motion_target_step == 0 && WonderLLM_hiwonder.motion_target_RunningTime == 0){ //当前指令为“机器人停止运动”
                sustain_move_flag = 0;
              }
              vel = {0.0f,2.0f*WonderLLM_hiwonder.Speed_Mode,0.0f};              
              break;
            }
            case 2:{ //右前
              vel = {2.0f*WonderLLM_hiwonder.Speed_Mode,2.0f*WonderLLM_hiwonder.Speed_Mode,0.0f};
              break;
            }
            case 3:{ //正右
              vel = {2.0f*WonderLLM_hiwonder.Speed_Mode,0.0f,0.0f};
              break;
            }
            case 4:{ //右后
              vel = {2.0f*WonderLLM_hiwonder.Speed_Mode,-2.0f*WonderLLM_hiwonder.Speed_Mode,0.0f};
              break;
            }
            case 5:{ //正后
              vel = {0.0f,-2.0f*WonderLLM_hiwonder.Speed_Mode,0.0f};
              break;
            }
            case 6:{ //左后
              vel = {-2.0f*WonderLLM_hiwonder.Speed_Mode,-2.0f*WonderLLM_hiwonder.Speed_Mode,0.0f};
              break;
            }
            case 7:{ //正左
              vel = {-2.0f*WonderLLM_hiwonder.Speed_Mode,0.0f,0.0f};
              break;
            }
            case 8:{ //左前
              vel = {-2.0f*WonderLLM_hiwonder.Speed_Mode,2.0f*WonderLLM_hiwonder.Speed_Mode,0.0f};
              break;
            }
            case 9:{ //原地左转
              vel = {0.0f,0.0f,2.0f*WonderLLM_hiwonder.Speed_Mode};
              break;
            }
            case 10:{ //原地右转
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
          
          //该类MCP工具阻塞式执行（block=true），执行完毕需调用基础系统指令action_finish回复WinderMind
          WonderLLM_Send_Action_Finish();							
          break;
        }

        case Frame_BaryCenterMove:{
          sprintf(info,"Frame_BaryCenterMove:direction:%d\r\n",WonderLLM_hiwonder.BaryCenter_direction);
          Serial.print(info);
          
          /* 执行用户自定义运动控制API */
          switch(WonderLLM_hiwonder.BaryCenter_direction){
            case 1:{ //正前 
              pos = {0.0f,3.0f,0.0f};
              break;
            }
            case 2:{ //右前
              pos = {3.0f,3.0f,0.0f};
              break;
            }
            case 3:{ //正右
              pos = {3.0f,0.0f,0.0f};
              break;
            }
            case 4:{ //右后
              pos = {3.0f,-3.0f,0.0f};
              break;
            }
            case 5:{ //正后
              pos = {0.0f,-3.0f,0.0f};
              break;
            }
            case 6:{ //左后
              pos = {-3.0f,-3.0f,0.0f};
              break;
            }
            case 7:{ //正左
              pos = {-3.0f,0.0f,0.0f};
              break;
            }
            case 8:{ //左前
              pos = {-3.0f,3.0f,0.0f};
              break;
            }
            case 9:{ //正上
              pos = {0.0f,0.0f,3.0f};
              break;
            }
            case 10:{ //正下
              pos = {0.0f,0.0f,-1.5f};
              break;
            }
            case 11:{ //恢复初始姿态
              pos = {0.0f,0.0f,0.0f};
              break;
            }
          }
          vel = {0.0f,0.0f,0.0f};
          att = {0.0f,0.0f,0.0f};
          minihexa.move(&vel, &pos, &att);

          //该类MCP工具阻塞式执行（block=true），执行完毕需调用基础系统指令action_finish回复WinderMind
          WonderLLM_Send_Action_Finish();							
          break;
        }

        case Frame_InclinationAngleMove:{
          sprintf(info,"Frame_InclinationAngleMove:direction:%d\r\n",WonderLLM_hiwonder.incline_direction);
          Serial.print(info);
          
          /* 执行用户自定义运动控制API */
          switch(WonderLLM_hiwonder.incline_direction){
            case 1:{ //前倾
              att = {12.0f,0.0f,0.0f};
              break;
            }
            case 2:{ //后倾
              att = {-12.0f,0.0f,0.0f};
              break;
            }
            case 3:{ //左倾
              att = {0.0f,15.0f,0.0f};
              break;
            }
            case 4:{ //右倾
              att = {0.0f,-15.0f,0.0f};
              break;
            }
            case 5:{ //左扭
              att = {0.0f,0.0f,-15.0f};
              break;
            }
            case 6:{ //右扭
              att = {0.0f,-0.0f,15.0f};
              break;
            }
          }
          vel = {0.0f,0.0f,0.0f};
          pos = {0.0f,0.0f,0.0f};
          minihexa.move(&vel, &pos, &att);

          //该类MCP工具阻塞式执行（block=true），执行完毕需调用基础系统指令action_finish回复WonderLLM
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

        //   //该类MCP工具阻塞式执行（block=true），执行完毕需调用基础系统指令action_finish回复WonderLLM
        //   WonderLLM_Send_Action_Finish();							
        //   break;          
        // }
        
        case Frame_get_status_battery:{
          sprintf(info, "[\"battery\",\"%d\",\"mV\"]", minihexa.board.bat_voltage);
          
          // //该类MCP工具需返回参数给WonderLLM（return=true），执行完毕需调用基础系统指令status按指定格式回传参数
          WonderLLM_Send_Status(info);
          break;
        }

        case Frame_get_status_Bodystate:{
          float BodyPosition[3];
          float euler_angle[3];
          minihexa.board.get_imu_euler(euler_angle);
          minihexa.read_position(BodyPosition);
          if(fabs(euler_angle[0]) > 150.0f){  //机器人翻倒，横滚角绝对值大于150°
            sprintf(info, "[\"Bodystate\",\"2\"]");
          }else{
            sprintf(info, "[\"Bodystate\",\"%d\"]", (BodyPosition[2] < 0) ? 1 : 0);
          }
          
          
          // //该类MCP工具需返回参数给WonderLLM（return=true），执行完毕需调用基础系统指令status按指定格式回传参数
          WonderLLM_Send_Status(info);
          break;
        }

        case Frame_get_status_poseture:{
          float euler_angle[3];
          minihexa.board.get_imu_euler(euler_angle);
          sprintf(info, "[[\"roll\",\"%.1f\"],[\"pitch\",\"%.1f\"]]",euler_angle[0],euler_angle[1]);
          
          // //该类MCP工具需返回参数给WonderLLM（return=true），执行完毕需调用基础系统指令status按指定格式回传参数
          WonderLLM_Send_Status(info);
          break;
        }

        case Frame_get_status_Speed_mode:{
          sprintf(info, "[\"Speed_mode\",\"%d\"]", WonderLLM_hiwonder.Speed_Mode);
          
          //该类MCP工具需返回参数给WonderLLM（return=true），执行完毕需调用基础系统指令status按指定格式回传参数
          WonderLLM_Send_Status(info);						
          break;
        }
        
        case Frame_get_status_distance:{       
          sprintf(info, "[\"distance\",\"%d\",\"mm\"]", minihexa.sensor._get_distance());
      
          //该类MCP工具需返回参数给WonderLLM（return=true），执行完毕需调用基础系统指令status按指定格式回传参数
          WonderLLM_Send_Status(info);
          break;
        }
        
        case Frame_get_status_running_mode:{
          sprintf(info, "[\"running_mode\",\"%d\"]", WonderLLM_hiwonder.running_mode);
          
          //该类MCP工具需返回参数给WonderLLM（return=true），执行完毕需调用基础系统指令status按指定格式回传参数
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

          //该类MCP工具阻塞式执行（block=true），执行完毕需调用基础系统指令action_finish回复WonderLLM
          WonderLLM_Send_Action_Finish();					
          break;
        }
        
        case Frame_set_led_color:{
          sprintf(info,"Frame_set_RGB: Left:%d,%d,%d,Right:%d,%d,%d\r\n",\
                WonderLLM_hiwonder.rgb_left[0],WonderLLM_hiwonder.rgb_left[1],WonderLLM_hiwonder.rgb_left[2],\
                WonderLLM_hiwonder.rgb_right[0],WonderLLM_hiwonder.rgb_right[1],WonderLLM_hiwonder.rgb_right[2]);
          Serial.print(info);
          
          /* 执行用户自定义RGB灯控制API */
          minihexa.sensor.set_ultrasound_rgb(RGB_WORK_SOLID_MODE, WonderLLM_hiwonder.rgb_left, WonderLLM_hiwonder.rgb_right);
          
          //该类MCP工具阻塞式执行（block=true），执行完毕需调用基础系统指令action_finish回复WonderLLM
          WonderLLM_Send_Action_Finish();
          break;
        }
        
        case Frame_set_buzzer:{
          sprintf(info,"Frame_set_buzzer:%d\r\n",WonderLLM_hiwonder.buzzer_count);
          Serial.print(info);
          
          /* 执行用户自定义蜂鸣器控制API */
          for(int i = 0; i < WonderLLM_hiwonder.buzzer_count; i++){
            ledcWriteTone(LEDC_CHANNEL_0, 3000);
            delay(200);
            ledcWriteTone(LEDC_CHANNEL_0, 0);
            delay(500);
          }
          
          //该类MCP工具阻塞式执行（block=true），执行完毕需调用基础系统指令action_finish回复WonderLLM
          WonderLLM_Send_Action_Finish();
          break;
        }

        case Frame_set_speed_mode:{ 
          sprintf(info,"Frame_set_speed_mode:%d\r\n",WonderLLM_hiwonder.Speed_Mode);
          Serial.print(info);

          //该类MCP工具阻塞式执行（block=true），执行完毕需调用基础系统指令action_finish回复WonderLLM
          WonderLLM_Send_Action_Finish();
          break;						
        }

        case Frame_ActionGroup:{ 
          sprintf(info,"Frame_ActionGroup:index%d time:%d\r\n",WonderLLM_hiwonder.ActionNum,WonderLLM_hiwonder.ExecuteActionNum);
          Serial.print(info);

          //为防止动作组执行时间过长，WonderLLM长时间收不到回复误判为指令执行超时，因此先发送回复信息再执行动作组
          //该类MCP工具阻塞式执行（block=true），执行完毕需调用基础系统指令action_finish回复WonderLLM
          WonderLLM_Send_Action_Finish();

          for(int i = 0; i < WonderLLM_hiwonder.ExecuteActionNum; i++){
            delay(1000);
            switch(WonderLLM_hiwonder.ActionNum){
              case 3:{ //唤醒动作
                minihexa.wake_up();
                break;
              }
              case 4:{ //唤醒奔跑动作
                minihexa._wake_up();
                break;
              }
              case 5:{ //撒娇动作
                minihexa.acting_cute();
                break;
              }
              case 6:   //越障动作
              case 7:   //左腿战斗动作
              case 8:   //右腿战斗动作
              case 9:   //左脚向前踢球动作
              case 10:  //左脚向右踢球动作
              case 11:  //右脚向前踢球动作
              case 12:  //右脚向左踢球动作 
              case 13:  //推门动作
              case 14:{ //挥手动作
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

          //将IIC速率降至100W
          /*执行非必要，如果其他IIC设备均支持400W速率通信，则可不必切换回
            较低的100W，执行该函数是出于兼容其他低速IIC设备的考虑*/
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

