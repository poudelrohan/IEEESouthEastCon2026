#include <kinematics.h>

#ifndef __MINIHEXA_TYPES_H__
#define __MINIHEXA_TYPES_H__

#include "Arduino.h"

/* Communication pins */
#define RXD2                                  16
#define TXD2                                  17 
#define SDA                                   22
#define SCL                                   23

/* Macro function: get the low byte of A */
#define GET_LOW_BYTE(A) ((uint8_t)(A))
/* Macro function: get the high byte of A */
#define GET_HIGH_BYTE(A) ((uint8_t)((A) >> 8))
/* Macro function: combine high and low bytes into 16-bit */
#define BYTE_TO_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))

#define LIMIT(x, min, max) (((x) >= (max)) ? (max) : (((x) <= (min)) ? (min) : (x)))

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  V1_0 = 0,
  V1_1
}VersionState;

typedef enum
{
  MINIHEXA_NULL = 0,
  MINIHEXA_RESET,
  MINIHEXA_VOLTAGE_GET,
  MINIHEXA_CRAWL_STATE,
  MINIHEXA_MOVING_CONTROL,
  MINIHEXA_POSE_CONTROL,
  MINIHEXA_AVOID,
  MINIHEXA_BALANCE,
  MINIHEXA_RGB_ADJUST,
  MINIHEXA_OFFSET_STATE,
  MINIHEXA_OFFSET_READ,
  MINIHEXA_OFFSET_SAVE,
  MINIHEXA_OFFSET_VERIFY,
  MINIHEXA_LEG_OFFSET_SET,
  MINIHEXA_ACTION_GROUP_RUN,
  MINIHEXA_ACTION_GROUP_STOP,
  MINIHEXA_ACTION_GROUP_DOWNLOAD,
  MINIHEXA_ACTION_GROUP_ERASE,
  MINIHEXA_ACTION_GROUP_ALL_ERASE,
  MINIHEXA_SERVO_CONTROL,
  MINIHEXA_SERVO_DUTY_READ,
  MINIHEXA_ARM_CONTROL
}ModeState;


typedef struct
{
  uint8_t mode;
  uint8_t data[60];
}RecData_t;

extern uint16_t bat_voltage;
extern VersionState version;

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif