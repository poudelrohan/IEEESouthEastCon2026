#ifndef __HIWONDER_ROBOT_H__
#define __HIWONDER_ROBOT_H__

#include "hiwonder_servo.h"
#include "hiwonder_board.h"
#include "hiwonder_sensor.h"
#include "kinematics.h"
#include "SPIFFS.h"



#ifdef __cplusplus
extern "C" 
{
#endif
#define SAMPLE_INTERVAL 10

#define KNOT_SERVO_MIDPOINT   140.0f

#define SERVO_ANGLE_180_FACTOR    11.11111111111111f
#define SERVO_ANGLE_280_FACTOR    7.142857142857143f

#define LEG1_START_X    3.15f
#define LEG1_START_Y   -5.65f

#define LEG2_START_X    4.34f
#define LEG2_START_Y    0.0f

#define LEG3_START_X    3.15f
#define LEG3_START_Y    5.65f

#define LEG4_START_X   -3.15f
#define LEG4_START_Y    5.65f

#define LEG5_START_X   -4.34f 
#define LEG5_START_Y    0.0f 

#define LEG6_START_X   -3.15f 
#define LEG6_START_Y   -5.65f 

typedef enum {
  CLOCKWISE = 1, 
  COUNTER_CLOCKWISE = -1
}RotationDirection;

typedef struct {
  float r;                      /* 圆半径 */
  uint16_t steps_per_circle;    /* 每圈步数 */
  uint16_t total_circles;       /* 总圈数 */
  uint16_t current_step;        /* 当前步数（从0开始）*/
  bool is_completed;            /* 是否已完成全部路径 */
  RotationDirection direction;  /* 旋转方向 */
}CircularPath;

#ifdef __cplusplus
} // extern "C"
#endif



class RobotJoint {
  public: 
    RobotJoint();

    bool dir;                 /* dir = true 正转 dir = false 反转 */
    int8_t deviation;
    uint8_t id;
    uint16_t duty;
    uint16_t time;
    float angle;

    /**
     * @brief 绑定舵机ID号
     * 
     * @param  id -分配的舵机序号
     */
    void attach_servo(uint8_t id);
    
    /**
     * @brief 通过设置角度实现舵机控制
     * 
     * @param  dir    -true 正转 
     *                -false 反转
     * @param  angle  -角度值
     * @param  time   -运行时间
     * @param  is_ops -是否直接控制舵机响应 
     *                -true 舵机响应 
     *                -false 舵机不响应
     * @attention  默认绑定的19、20、21号舵机为180度舵机
     */
    void set_angle(bool dir, float angle, uint16_t time, bool is_ops);

    /**
     * @brief 通过设置PWM脉宽实现舵机控制
     * 
     * @param  duty   -PWM脉宽
     * @param  time   -运行时间
     * @param  is_ops -是否直接控制舵机响应 
     *                -true 舵机响应 
     *                -false 舵机不响应
     */
    void set_duty(uint16_t duty, uint16_t time, bool is_ops);

    /**
     * @brief 偏差设置
     * 
     * @param val  -偏差值
     */
    void set_deviation(int8_t val);

    /**
     * @brief 偏差读取
     * 
     * @return int8_t -偏差
     */
    int8_t read_deviation(void);

    /**
     * @brief 偏差下载
     * 
     * @return true   -下载成功
     * @return false  -下载失败
     */
    bool download_deviation(void);
};

class RobotLeg {
  public:
    RobotLeg();

    uint8_t id;
    uint32_t set_time = 0;
    volatile bool is_busy = false;

    RobotJoint joint_a, joint_b, joint_c;
    
    Theta_t _result;
    
    Vector_t offset;                                /* 读取到的舵机偏差值 */

    Vector_t b_leg_start = {0.0f, 0.0f, 0.0f};                           /* 腿部起始端在机体坐标系下的坐标 */
    Vector_t r_leg_start = {0.0f, 0.0f, 0.0f};
    Vector_t r_leg_end = {0.0f, 0.0f, 0.0f};                             /* 经过旋转变换的腿部末端坐标 */
    Vector_t b_leg_end = {0.0f, 0.0f, 0.0f};                             /* 腿部末端在机体坐标系下的坐标 */
    Vector_t omni_move_end_point = {0.0f, 0.0f, 0.0f};                   /* 全向运动下腿部末端目标落点位置 */  
    Vector_t trajectory_point = {0.0f, 0.0f, 0.0f};                      /* 轨迹规划目标位置 */ 
    Vector_t result = {0.0f, 0.0f, 0.0f};                                /* 姿态解算的坐标 */
    Vector_t start_result = {0.0f, 0.0f, 0.0f};                          /* 初始化姿态解算的坐标 */
    Vector_t amplitude = {0.0f, 0.0f, 0.0f};                             /* 腿部运动幅度 */
    Vector_t last_point = {0.0f, 0.0f, 0.0f};                            /* 腿部末端上一次坐标 */    
    Vector_t now_point = {0.0f, 0.0f, 0.0f};                             /* 腿部末端坐标 */
    Vector_t trans_point = {0.0f, 0.0f, 0.0f};
    Vector_t begin_point = {6.5f, 0.0f, -4.0f};
    Vector_t start_point = {6.5f, 0.0f, -4.0f};
    Vector_t goal_point = {6.5f, 0.0f, -4.0f};
    /**
     * @brief 运动腿部到指定坐标点
     * 
     * @param  point  -目标坐标
     * @param  time   -运行时间
     * @return true   -正在运行
     * @return false  -未在运行
     */
    bool move(Vector_t point, uint32_t time);

    /**
     * @brief 获取当前坐标点
     * 
     * @return Vector_t -当前坐标点
     */
    Vector_t get_now_point(void);
  private:
    uint32_t current_time = 0;
    uint32_t goal_time = 20;

    
};

class RobotArm {
  public:
    RobotArm();
    RobotJoint joint_a, joint_b, joint_c;
};

class Robot {
  public:
    Robot();

    HW_Board board;
    HW_Sensor sensor;
    RobotArm arm;
    RobotLeg leg1, leg2, leg3, leg4, leg5, leg6;

    enum Func_State {CALIBRATE, CRAWL, ACTION_GROUP};
    enum Move_State {MOVING, STOP, REST};
    enum Avoid_State {FORWARD, BACK, TURN, WAIT};
    enum Act_State {READ_FRAME_NUM, READ_FRAME_DATA, ACT_STOP};
    Func_State func_state = CRAWL;
    Move_State move_state = REST;
    Avoid_State avoid_state = FORWARD;
    Act_State act_state = READ_FRAME_NUM;

    uint32_t tick_count = 0;
    /**
     * @brief 机体初始化
     * 
     */
    void begin(void);
    
    /**
     * @brief 位置更新  默认每10ms更新一次
     * 
     */
    void update(void);
    
    /**
     * @brief 运动姿态
     * 
     */
    void crawl_state(void);

    /**
     * @brief 机体移动
     * 
     * @param  _velocity    -机体x、y和绕z轴旋转速度 
     * @param  _position    -相对于机体中心的x、y和z轴位置
     * @param  _euler       -相对于机体中心的欧拉角
     * @param  time         -运行时间
     * @param  step_num     -步数
     */
    void move(Velocity_t *_velocity, Vector_t *_position, Euler_t *_euler, uint32_t time = 600, int step_num = -1);

    /**
     * @brief 机体复位
     * 
     */
    void reset(void);

    /**
     * @brief 避障
     * 
     * @param  dis  -测得的当前距离
     */
    void avoid(uint16_t dis);

    /**
     * @brief 避障(可指定距离)
     * 
     * @param  dis  -测得的当前距离
     * @param  dis_avoid  -设置的待避距离
     */
    void _avoid(uint16_t dis_now,uint16_t dis_avoid);

    /**
     * @brief 自平衡
     * 
     */
    void balance(bool state);

    /**
     * @brief 扭动动作
     * 
     * @param  radius            -半径
     * @param  circles           -圈数
     * @param  steps_per_circle  -每圈采样点数
     * @param  dir               -方向
     */
    void twist(float radius, uint16_t circles, uint16_t steps_per_circle, RotationDirection dir);

    /**
     * @brief 撒娇动作
     * 
     */
    void acting_cute(void);

    /**
     * @brief 唤醒动作
     * 
     */
    void wake_up();

    /**
     * @brief 唤醒奔跑动作
     * 
     */
    void _wake_up();

    // void coor_action1();
    // void coor_action2();

    /**
     * @brief 列出当前存在的动作组文件名称及文件大小(需要打开LOG)
     * 
     */
    void list_action_group_dir(void);  

    /**
     * @brief 动作组复位
     * 
     */
    void action_group_stop(void);    

    /**
     * @brief 动作组运行
     * 
     * @param  id -运行的动作组id号
     */
    void action_group_run(uint8_t id);

    /**
     * @brief 动作组下载
     * 
     * @param  id       -动作组id号
     * @param  buf      -下载的动作组数据
     * @param  length   -下载的动作组数据长度
     * @return true     -下载成功
     * @return false    -下载失败
     */
    bool action_group_download(uint8_t id, uint8_t *buf, size_t length);

    /**
     * @brief 动作组擦除
     * 
     * @param  id       -动作组id号
     * @return true     -擦除成功
     * @return false     -擦除失败
     */
    bool action_group_erase(uint8_t id);

    /**
     * @brief 多舵机控制
     * 
     * @param  arg       -舵机参数结构体指针
     * @param  servo_num -舵机数量
     * @param  time      -运行时间
     */
    void multi_servo_control(ServoArg_t* arg, uint16_t servo_num, uint16_t time);

    /**
     * @brief 偏差设置
     * 
     * @param id  -关节id
     * @param val  -偏差值
     */
    void set_deviation(uint8_t id, int8_t val);

    /**
     * @brief 偏差读取
     * 
     * @param *val -读取到的偏差值指针
     */
    void read_deviation(int8_t *val);

    /**
     * @brief 机器人重心坐标值读取
     * 
     * @param *val -读取到的坐标值指针
     */
    void read_position(float *val);

    /**
     * @brief 偏差下载
     * 
     * @return true   -下载成功
     * @return false  -下载失败
     */
    bool download_deviation(void);        

  private:
    bool swing_state = false;
    uint32_t balance_tick_start = 0;
    uint32_t move_time = 0;
    float step_length;
    int _step_num;
    int last_step_num;
    float _leg_lift = 3;
    uint8_t act_read_frame_num = 0;
    const float max_half_step_length = 3.0f;
    const float body_width      = 7.6f;
    const float body_length     = 14.0f;

    Velocity_t velocity      = {0.0f, 0.0f, 0.0f};
    Velocity_t last_velocity = {0.0f, 0.0f, 0.0f};
    Vector_t   position      = {0.0f, 0.0f, 0.0f};
    Euler_t    euler         = {0.0f, 0.0f, 0.0f};
    Vector_t   circle_center;

    File file;
    TimerHandle_t timer;
    TimerHandle_t event;

    /**
     * @brief 移动落点坐标计算
     * 
     */
    void cal_omni_move_end_point(void);

    /**
     * @brief  圆圈轨迹点计算
     * 
     * @param  path   -轨迹点结构体指针
     * @param  x      -圆心x坐标
     * @param  y      -圆心y坐标
     * @return true   -运动完成
     * @return false  -运动未完成
     */
    bool next_circular_point(CircularPath* path, float* x, float* y);
};

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif