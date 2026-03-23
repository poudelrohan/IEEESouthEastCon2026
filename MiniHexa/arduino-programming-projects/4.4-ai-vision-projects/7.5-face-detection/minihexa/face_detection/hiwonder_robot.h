#ifndef __HIWONDER_ROBOT_H__
#define __HIWONDER_ROBOT_H__

#include "hiwonder_servo.h"
#include "hiwonder_board.h"
#include "hiwonder_sensor.h"
#include "kinematics.h"
#include "SPIFFS.h"

#define SAMPLE_INTERVAL 10

#ifdef __cplusplus
extern "C" 
{
#endif

typedef enum {
  CLOCKWISE = 1, 
  COUNTER_CLOCKWISE = -1
}RotationDirection;

typedef struct {
  float r;                      /* Circle radius */
  uint16_t steps_per_circle;    /* Steps per circle */
  uint16_t total_circles;       /* Total circles */
  uint16_t current_step;        /* Current step (from 0) */
  bool is_completed;            /* Whether full path is completed */
  RotationDirection direction;  /* Rotation direction */
}CircularPath;

#ifdef __cplusplus
} // extern "C"
#endif



class RobotJoint {
  public: 
    RobotJoint();

    bool dir;                 /* dir = true forward, dir = false reverse */
    int8_t offset_value;
    uint8_t id;
    uint16_t duty;
    uint16_t time;
    float angle;

    /**
     * @brief Bind servo ID
     * 
     * @param  id - assigned servo number
     */
    void attach_servo(uint8_t id);
    
    /**
     * @brief Control servo by setting angle
     * 
     * @param  dir    - true forward 
     *                - false reverse
     * @param  angle  - angle value
     * @param  time   - execution time
     * @param  is_ops - whether to directly control servo response 
     *                - true servo responds 
     *                - false servo does not respond
     * @attention  Default bound servos 19, 20, 21 are 180-degree servos
     */
    void set_angle(bool dir, float angle, uint16_t time, bool is_ops);

    /**
     * @brief Control servo by setting PWM pulse width
     * 
     * @param  duty   - PWM pulse width
     * @param  time   - execution time
     * @param  is_ops - whether to directly control servo response 
     *                - true servo responds 
     *                - false servo does not respond
     */
    void set_duty(uint16_t duty, uint16_t time, bool is_ops);

  private:
    const float min_duty = 400;
    const float max_duty = 2600;
    const float angle_180_factor = 11.11111111111111f;
    const float angle_280_factor = 7.142857142857143f;
};

class RobotLeg {
  public:
    RobotLeg();

    uint8_t id;
    uint32_t set_time = 0;
    volatile bool is_busy = false;

    RobotJoint joint_a, joint_b, joint_c;
    
    Theta_t _result;
    
    Vector_t offset;                                /* Read servo offset values */
    Vector_t b_leg_start;                           /* Leg start position in body coordinate system */
    Vector_t r_leg_end;                             /* Rotated leg end-point coordinates */
    Vector_t b_leg_end;                             /* Leg end-point in body coordinate system */
    Vector_t omni_move_end_point;                   /* Omnidirectional movement leg end target position */  
    Vector_t trajectory_point;                      /* Trajectory planning target position */ 
    Vector_t result;                                /* Pose calculation coordinates */
    Vector_t start_result;                          /* Initial pose calculation coordinates */
    Vector_t amplitude;                             /* Leg movement amplitude */
    Vector_t last_point;                            /* Leg end-point previous coordinates */    
    Vector_t now_point;                             /* Leg end-point coordinates */
    Vector_t trans_point;
    Vector_t begin_point = {7.0f, 0.0f, -4.0f};
    Vector_t start_point = {7.0f, 0.0f, -4.0f};
    Vector_t goal_point = {7.0f, 0.0f, -4.0f};
    Vector_t verified_point = {9.0f, 0.0f, -2.0f};  /* Leg offset verification initial coordinates */

    /**
     * @brief Move leg to specified coordinates
     * 
     * @param  point  - target coordinates
     * @param  time   - execution time
     * @return true   - running
     * @return false  - not running
     */
    bool move(Vector_t point, uint32_t time);

    /**
     * @brief Get current coordinates
     * 
     * @return Vector_t - current coordinates
     */
    Vector_t get_now_point(void);

    /**
     * @brief Set offset
     * 
     * @param  x  - x-axis offset
     * @param  y  - y-axis offset
     * @param  z  - z-axis offset
     */
    void offset_set(float x, float y, float z);

    /**
     * @brief Read offset
     * 
     * @return Vector_t - offset
     */
    Vector_t offset_read(void);

    /**
     * @brief Download offset
     * 
     * @return true   - download success
     * @return false  - download failed
     */
    bool offset_download(void);

    /**
     * @brief  Erase offset
     * 
     * @return true   - erase success
     * @return false  - erase failed
     */
    bool offset_erase(void);

    /**
     * @brief  Offset verification
     * 
     */
    void offset_verify(void);
  

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

    /**
     * @brief Body initialization
     * 
     */
    void begin(void);
    
    /**
     * @brief Position update, default every 10ms
     * 
     */
    void update(void);

    /**
     * @brief Offset calibration posture
     * 
     */
    void calibrate_state(void);

    /**
     * @brief Motion posture
     * 
     */
    void crawl_state(void);

    /**
     * @brief Body movement
     * 
     * @param  _velocity    - body x, y and z-axis rotation speed 
     * @param  _position    - x, y, z position relative to body center
     * @param  _euler       - Euler angles relative to body center
     * @param  time         - execution time
     * @param  step_num     - number of steps
     */
    void move(Velocity_t *_velocity, Vector_t *_position, Euler_t *_euler, uint32_t time = 1800, int step_num = -1);

    /**
     * @brief Body reset
     * 
     */
    void reset(void);

    /**
     * @brief Obstacle avoidance
     * 
     * @param  dis  - measured current distance
     */
    void avoid(uint16_t dis);

    /**
     * @brief Self-balancing
     * 
     */
    void balance(void);

    /**
     * @brief Twisting action
     * 
     * @param  radius            - radius
     * @param  circles           - number of circles
     * @param  steps_per_circle  - sample points per circle
     * @param  dir               - direction
     */
    void twist(float radius, uint16_t circles, uint16_t steps_per_circle, RotationDirection dir);

    /**
     * @brief Cute/playful action
     * 
     */
    void acting_cute(void);

    /**
     * @brief Wake up action
     * 
     */
    void wake_up();

    /**
     * @brief Wake up running action
     * 
     */
    void _wake_up();

    // void coor_action1();
    // void coor_action2();

    /**
     * @brief List existing action group file names and sizes (requires LOG enabled)
     * 
     */
    void list_action_group_dir(void);  

    /**
     * @brief Action group reset
     * 
     */
    void action_group_stop(void);    

    /**
     * @brief Action group run
     * 
     * @param  id - action group ID to run
     */
    void action_group_run(uint8_t id);

    /**
     * @brief Download action group
     * 
     * @param  id       - action group ID
     * @param  buf      - action group data buffer
     * @param  length   - action group data length
     * @return true     - download success
     * @return false    - download failed
     */
    bool action_group_download(uint8_t id, uint8_t *buf, size_t length);

    /**
     * @brief Erase action group
     * 
     * @param  id       - action group ID
     * @return true     - erase success
     * @return false     - erase failed
     */
    bool action_group_erase(uint8_t id);

    /**
     * @brief Multi-servo control
     * 
     * @param  arg       - servo parameter struct pointer
     * @param  servo_num - number of servos
     * @param  time      - execution time
     */
    void multi_servo_control(ServoArg_t* arg, uint16_t servo_num, uint16_t time);
    
  private:
    bool swing_state = false;
   
    uint32_t tick_count = 0;
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
     * @brief Calculate movement landing point coordinates
     * 
     */
    void cal_omni_move_end_point(void);

    /**
     * @brief Calculate circle trajectory point
     * 
     * @param  path   - trajectory point struct pointer
     * @param  x      - circle center x coordinate
     * @param  y      - circle center y coordinate
     * @return true   - motion complete
     * @return false  - motion not complete
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