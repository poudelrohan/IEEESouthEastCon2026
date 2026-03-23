#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include "arduino.h"

#ifdef __cplusplus
extern "C" 
{
#endif

/* 舵机中位点 */
#define SERVO_MIDPOINT       140.0f

/* unit:cm */
#define BODY_TOP_LEN          2.85f
/* 3.7 + 1.5 */
#define TOP_MIDDLE_LEN        3.70f
#define MIDDLE_BASE_LEN        7.2f

#define JOINT_POS_OFFSET       1.5f
// #define THETA_CALI           -21.9f

#define RAD_TO_THETA(rad) ((rad) * 180.0f / PI)
#define THETA_TO_RAD(theta) ((theta) * PI / 180.0f)

typedef struct 
{
    float vx;
    float vy;
    float omega;
}Velocity_t;

typedef struct
{
  float pitch;
  float roll;
  float yaw;
}Euler_t;

typedef struct 
{
  float start;
  float end;
}Bezier_arg_t;

typedef struct
{
  Bezier_arg_t x;
  Bezier_arg_t y;
  Bezier_arg_t z;
}Trajectory_t;

typedef struct 
{
  float top_servo_val;
  float middle_servo_val;
  float base_servo_val;
}Servo_val_t;

typedef enum {
  ADD,
  SUB
}Ops_state;

typedef enum {
  AXIS_X,
  AXIS_Y,
  AXIS_Z
}Axis_state;

typedef struct {
  volatile float x;
  volatile float y;
  volatile float z;
}Vector_t;

typedef struct {
  float a;
  float b;
  float c;
}Theta_t;

float invsqrt(float number);
float fmap(float x, float in_min, float in_max, float out_min, float out_max);

Vector_t vector_arg_ops(Vector_t* vector_1, Vector_t* vector_2, Ops_state ops);
Vector_t rotation_trans(Vector_t* vector, float theta, Axis_state axis);
Theta_t theta_arg_ops(Theta_t* theta_1, Theta_t* theta_2, Ops_state ops);

/**
* @brief 运动学正解
* 
* @param  *theta 
*  @arg  Theta_t类型的结构体指针
*
* @return Vector_t
*  @arg  返回正解后的坐标
*/
Vector_t fkine(Theta_t *theta);

/**
* @brief 运动学逆解
* 
* @param  *vector 
*  @arg  Vector_t类型的结构体指针
*
* @return Theta_t
*  @arg  返回逆解后的关节角度值
*/
Theta_t ikine(Vector_t *vector);

Vector_t pose_control(uint8_t index, 
                      Vector_t *position, 
                      Euler_t *euler, 
                      Vector_t *r_leg_end, 
                      Vector_t *b_leg_start);
#ifdef __cplusplus
} // extern "C"
#endif

#endif
