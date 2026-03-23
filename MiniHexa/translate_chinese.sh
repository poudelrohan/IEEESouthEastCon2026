#!/bin/bash
# Script to translate all Chinese comments to English in .cpp and .h files

cd /Users/rohanpoudel/Documents/Arduino/MiniHexa/arduino-programming-projects

# global.h comments (already done: 通信引脚, 3 macro functions)
find . \( -name "*.cpp" -o -name "*.h" \) -exec sed -i '' \
  -e 's|// 设置 13 位分辨率|// Set 13-bit resolution|g' \
  -e 's|// 设置 5kHz 的 PWM 基础频率|// Set 5kHz PWM base frequency|g' \
  -e 's|// 默认1.1V的参考电压|// Default 1.1V reference voltage|g' \
  -e 's|// ADC采样次数|// ADC sampling count|g' \
  -e 's|// ADC 12位宽度|// ADC 12-bit width|g' \
  -e 's|// 6dB衰减器|// 6dB attenuator|g' \
  -e 's|// ADC引脚|// ADC pin|g' \
  -e 's|// 滤波窗口大小|// Filter window size|g' \
  {} +

# hiwonder_board.h doxygen comments
find . \( -name "*.cpp" -o -name "*.h" \) -exec sed -i '' \
  -e 's|@brief 获取按键状态|@brief Get button state|g' \
  -e 's|@return true  -没按下|@return true  - not pressed|g' \
  -e 's|@return false -按下|@return false - pressed|g' \
  -e 's|@brief 获取声音传感器数值|@brief Get sound sensor value|g' \
  -e 's|@brief 更新陀螺仪数据|@brief Update IMU data|g' \
  -e 's|@param  state    -true  打开|@param  state    - true  enable|g' \
  -e 's|                  -false 关闭|                  - false disable|g' \
  -e 's|@brief 获取陀螺仪加速度数据|@brief Get IMU accelerometer data|g' \
  -e 's|@brief 获取陀螺仪角速度数据|@brief Get IMU gyroscope data|g' \
  -e 's|@brief 获取陀螺仪欧拉角数据|@brief Get IMU Euler angle data|g' \
  -e 's|@brief 更新电池电压数据|@brief Update battery voltage data|g' \
  {} +

find . \( -name "*.cpp" -o -name "*.h" \) -exec sed -i '' \
  -e 's|/\* 静态校准 \*/|/* Static calibration */|g' \
  {} +

# hiwonder_board.cpp comments
find . \( -name "*.cpp" -o -name "*.h" \) -exec sed -i '' \
  -e 's|// 插入到滑动窗口|// Insert into sliding window|g' \
  -e 's|// 计算滑动平均|// Calculate sliding average|g' \
  -e 's|// 平均电压|// Average voltage|g' \
  {} +

# hiwonder_robot.cpp and hiwonder_robot.h comments
find . \( -name "*.cpp" -o -name "*.h" \) -exec sed -i '' \
  -e 's|/\* NVS非易失储存 \*/|/* NVS non-volatile storage */|g' \
  -e 's|// 以写模式打开|// Open in write mode|g' \
  -e 's|/\* 运动过程中强行写入目标值 \*/|/* Force write target value during motion */|g' \
  {} +

# hiwonder_robot.h CircularPath struct comments
find . \( -name "*.cpp" -o -name "*.h" \) -exec sed -i '' \
  -e 's|/\* 圆半径 \*/|/* Circle radius */|g' \
  -e 's|/\* 每圈步数 \*/|/* Steps per circle */|g' \
  -e 's|/\* 总圈数 \*/|/* Total circles */|g' \
  -e 's|/\* 当前步数（从0开始）\*/|/* Current step count (starting from 0) */|g' \
  -e 's|/\* 是否已完成全部路径 \*/|/* Whether the full path is completed */|g' \
  -e 's|/\* 旋转方向 \*/|/* Rotation direction */|g' \
  {} +

# RobotJoint class comments
find . \( -name "*.cpp" -o -name "*.h" \) -exec sed -i '' \
  -e 's|/\* dir = true 正转 dir = false 反转 \*/|/* dir = true forward, dir = false reverse */|g' \
  -e 's|@brief 绑定舵机ID号|@brief Bind servo ID|g' \
  -e 's|@param  id -分配的舵机序号|@param  id - assigned servo index|g' \
  -e 's|@brief 通过设置角度实现舵机控制|@brief Control servo by setting angle|g' \
  -e 's|@param  dir    -true 正转|@param  dir    - true forward|g' \
  -e 's|                -false 反转|                - false reverse|g' \
  -e 's|@param  angle  -角度值|@param  angle  - angle value|g' \
  -e 's|@param  time   -运行时间|@param  time   - execution time|g' \
  {} +

find . \( -name "*.cpp" -o -name "*.h" \) -exec sed -i '' \
  -e 's|@param  is_ops -是否直接控制舵机响应|@param  is_ops - whether to directly control servo response|g' \
  -e 's|                -true 舵机响应|                - true servo responds|g' \
  -e 's|                -false 舵机不响应|                - false servo does not respond|g' \
  -e 's|@attention  默认绑定的19、20、21号舵机为180度舵机|@attention  Servos 19, 20, 21 are bound as 180-degree servos by default|g' \
  -e 's|@brief 通过设置PWM脉宽实现舵机控制|@brief Control servo by setting PWM pulse width|g' \
  -e 's|@param  duty   -PWM脉宽|@param  duty   - PWM pulse width|g' \
  {} +

# Deviation comments
find . \( -name "*.cpp" -o -name "*.h" \) -exec sed -i '' \
  -e 's|@brief 偏差设置|@brief Set deviation|g' \
  -e 's|@param val  -偏差值|@param val  - deviation value|g' \
  -e 's|@brief 偏差读取|@brief Read deviation|g' \
  -e 's|@return int8_t -偏差|@return int8_t - deviation|g' \
  -e 's|@brief 偏差下载|@brief Save deviation to flash|g' \
  -e 's|@return true   -下载成功|@return true   - save successful|g' \
  -e 's|@return false  -下载失败|@return false  - save failed|g' \
  {} +

# RobotLeg comments
find . \( -name "*.cpp" -o -name "*.h" \) -exec sed -i '' \
  -e 's|/\* 读取到的舵机偏差值 \*/|/* Read servo deviation value */|g' \
  -e 's|/\* 腿部起始端在机体坐标系下的坐标 \*/|/* Leg start position in body coordinate system */|g' \
  -e 's|/\* 经过旋转变换的腿部末端坐标 \*/|/* Leg end position after rotation transformation */|g' \
  -e 's|/\* 腿部末端在机体坐标系下的坐标 \*/|/* Leg end position in body coordinate system */|g' \
  -e 's|/\* 全向运动下腿部末端目标落点位置 \*/|/* Target foot placement for omnidirectional movement */|g' \
  -e 's|/\* 轨迹规划目标位置 \*/|/* Trajectory planning target position */|g' \
  -e 's|/\* 姿态解算的坐标 \*/|/* Pose calculation coordinates */|g' \
  -e 's|/\* 初始化姿态解算的坐标 \*/|/* Initial pose calculation coordinates */|g' \
  {} +

find . \( -name "*.cpp" -o -name "*.h" \) -exec sed -i '' \
  -e 's|/\* 腿部运动幅度 \*/|/* Leg movement amplitude */|g' \
  -e 's|/\* 腿部末端上一次坐标 \*/|/* Previous leg end position */|g' \
  -e 's|/\* 腿部末端坐标 \*/|/* Current leg end position */|g' \
  {} +

# RobotLeg move/get_now_point doxygen
find . \( -name "*.cpp" -o -name "*.h" \) -exec sed -i '' \
  -e 's|@brief 运动腿部到指定坐标点|@brief Move leg to specified coordinate point|g' \
  -e 's|@param  point  -目标坐标|@param  point  - target coordinate|g' \
  -e 's|@return true   -正在运行|@return true   - running|g' \
  -e 's|@return false  -未在运行|@return false  - not running|g' \
  -e 's|@brief 获取当前坐标点|@brief Get current coordinate point|g' \
  -e 's|@return Vector_t -当前坐标点|@return Vector_t - current coordinate point|g' \
  {} +

# Robot class doxygen comments
find . \( -name "*.cpp" -o -name "*.h" \) -exec sed -i '' \
  -e 's|@brief 机体初始化|@brief Robot body initialization|g' \
  -e 's|@brief 位置更新  默认每10ms更新一次|@brief Position update, defaults to every 10ms|g' \
  -e 's|@brief 运动姿态|@brief Locomotion posture|g' \
  -e 's|@brief 机体移动|@brief Robot body movement|g' \
  -e 's|@param  _velocity    -机体x、y和绕z轴旋转速度|@param  _velocity    - body x, y and z-axis rotation speed|g' \
  -e 's|@param  _position    -相对于机体中心的x、y和z轴位置|@param  _position    - x, y and z position relative to body center|g' \
  -e 's|@param  _euler       -相对于机体中心的欧拉角|@param  _euler       - Euler angles relative to body center|g' \
  -e 's|@param  time         -运行时间|@param  time         - execution time|g' \
  -e 's|@param  step_num     -步数|@param  step_num     - number of steps|g' \
  {} +

find . \( -name "*.cpp" -o -name "*.h" \) -exec sed -i '' \
  -e 's|@brief 机体复位|@brief Reset robot body|g' \
  -e 's|@brief 避障|@brief Obstacle avoidance|g' \
  -e 's|@param  dis  -测得的当前距离|@param  dis  - measured current distance|g' \
  -e 's|@brief 自平衡|@brief Self-balancing|g' \
  -e 's|@brief 扭动动作|@brief Twist action|g' \
  -e 's|@param  radius            -半径|@param  radius            - radius|g' \
  -e 's|@param  circles           -圈数|@param  circles           - number of circles|g' \
  -e 's|@param  steps_per_circle  -每圈采样点数|@param  steps_per_circle  - sample points per circle|g' \
  -e 's|@param  dir               -方向|@param  dir               - direction|g' \
  {} +

find . \( -name "*.cpp" -o -name "*.h" \) -exec sed -i '' \
  -e 's|@brief 撒娇动作|@brief Acting cute action|g' \
  -e 's|@brief 唤醒动作|@brief Wake up action|g' \
  -e 's|@brief 唤醒奔跑动作|@brief Wake up and run action|g' \
  -e 's|@brief 列出当前存在的动作组文件名称及文件大小(需要打开LOG)|@brief List existing action group file names and sizes (requires LOG enabled)|g' \
  -e 's|@brief 动作组复位|@brief Action group stop/reset|g' \
  -e 's|@brief 动作组运行|@brief Run action group|g' \
  -e 's|@param  id -运行的动作组id号|@param  id - action group ID to run|g' \
  {} +

find . \( -name "*.cpp" -o -name "*.h" \) -exec sed -i '' \
  -e 's|@brief 动作组下载|@brief Download action group|g' \
  -e 's|@param  id       -动作组id号|@param  id       - action group ID|g' \
  -e 's|@param  buf      -下载的动作组数据|@param  buf      - action group data to download|g' \
  -e 's|@param  length   -下载的动作组数据长度|@param  length   - action group data length|g' \
  -e 's|@return true     -下载成功|@return true     - download successful|g' \
  -e 's|@return false    -下载失败|@return false    - download failed|g' \
  {} +

find . \( -name "*.cpp" -o -name "*.h" \) -exec sed -i '' \
  -e 's|@brief 动作组擦除|@brief Erase action group|g' \
  -e 's|@return true     -擦除成功|@return true     - erase successful|g' \
  -e 's|@return false     -擦除失败|@return false     - erase failed|g' \
  {} +

find . \( -name "*.cpp" -o -name "*.h" \) -exec sed -i '' \
  -e 's|@brief 多舵机控制|@brief Multi-servo control|g' \
  -e 's|@param  arg       -舵机参数结构体指针|@param  arg       - servo parameter struct pointer|g' \
  -e 's|@param  servo_num -舵机数量|@param  servo_num - number of servos|g' \
  -e 's|@param  time      -运行时间|@param  time      - execution time|g' \
  {} +

find . \( -name "*.cpp" -o -name "*.h" \) -exec sed -i '' \
  -e 's|@param id  -关节id|@param id  - joint ID|g' \
  -e 's|@param val  -偏差值|@param val  - deviation value|g' \
  -e 's|@param \*val -读取到的偏差值指针|@param *val - pointer to read deviation values|g' \
  {} +

# Robot private method doxygen
find . \( -name "*.cpp" -o -name "*.h" \) -exec sed -i '' \
  -e 's|@brief 移动落点坐标计算|@brief Calculate movement foothold coordinates|g' \
  -e 's|@brief  圆圈轨迹点计算|@brief  Circular trajectory point calculation|g' \
  -e 's|@param  path   -轨迹点结构体指针|@param  path   - trajectory point struct pointer|g' \
  -e 's|@param  x      -圆心x坐标|@param  x      - circle center x coordinate|g' \
  -e 's|@param  y      -圆心y坐标|@param  y      - circle center y coordinate|g' \
  -e 's|@return true   -运动完成|@return true   - motion completed|g' \
  -e 's|@return false  -运动未完成|@return false  - motion not completed|g' \
  {} +

# hiwonder_robot.cpp circular path comments
find . \( -name "*.cpp" -o -name "*.h" \) -exec sed -i '' \
  -e 's|/\* 已完成所有步骤（包括回到原点）\*/|/* All steps completed (including return to origin) */|g' \
  -e 's|/\* 已完成圆周运动，但还未返回原点 \*/|/* Circular motion completed, but not yet returned to origin */|g' \
  -e 's|/\* 正常圆周运动 \*/|/* Normal circular motion */|g' \
  -e 's|/\* 圆心相对于机体坐标系的方向向量 \*/|/* Direction vector of circle center relative to body coordinate system */|g' \
  {} +

# hiwonder_robot.cpp action group comment
find . \( -name "*.cpp" -o -name "*.h" \) -exec sed -i '' \
  -e 's|case READ_FRAME_NUM: /\*读取帧头\*/|case READ_FRAME_NUM: /* Read frame header */|g' \
  {} +

# hiwonder_sensor.h comments
find . \( -name "*.cpp" -o -name "*.h" \) -exec sed -i '' \
  -e 's|/\* 超声波模块I2C从机地址 \*/|/* Ultrasonic module I2C slave address */|g' \
  -e 's|/\* 距离低8位，单位mm \*/|/* Distance low 8 bits, unit: mm */|g' \
  -e 's|/\*  RGB灯模式设置寄存器 \*/|/* RGB LED mode setting register */|g' \
  -e 's|/\* 0-按下   1-松开 \*/|/* 0 - pressed, 1 - released */|g' \
  -e 's|/\* 0-有障碍 1-无 \*/|/* 0 - obstacle detected, 1 - clear */|g' \
  {} +

# hiwonder_sensor.h - rgb comment
find . \( -name "*.cpp" -o -name "*.h" \) -exec sed -i '' \
  -e 's|//r1，g1，b1表示右边rgb灯的呼吸周期，20表示2s一个周期|// r1, g1, b1 represent right RGB LED breathing period, 20 means 2s per cycle|g' \
  {} +

# hiwonder_servo.cpp comments
find . \( -name "*.cpp" -o -name "*.h" \) -exec sed -i '' \
  -e 's|/\* 计算总帧长度（帧头2 + 类型号1 + 数据长度1 + 数据data_len + 校验1） \*/|/* Calculate total frame length (header 2 + type 1 + data length 1 + data data_len + checksum 1) */|g' \
  -e 's|/\* 填充帧头\*/|/* Fill frame header */|g' \
  -e 's|/\* 类型 \*/|/* Type */|g' \
  -e 's|/\* 数据长度（data_len）\*/|/* Data length (data_len) */|g' \
  -e 's|/\* 计算校验和（校验帧头、功能号、数据长度、数据）\*/|/* Calculate checksum (header, function code, data length, data) */|g' \
  {} +

find . \( -name "*.cpp" -o -name "*.h" \) -exec sed -i '' \
  -e 's|// /\* 填充校验位（最后一位）\*/|// /* Fill checksum (last byte) */|g' \
  -e 's|/\* 通过串口发送帧 \*/|/* Send frame via serial port */|g' \
  -e 's|/\* 舵机驱动 \*/|/* Servo driver */|g' \
  -e 's|// 增大接收缓冲区到2KB|// Increase receive buffer to 2KB|g' \
  {} +

echo "Phase 1 complete"
