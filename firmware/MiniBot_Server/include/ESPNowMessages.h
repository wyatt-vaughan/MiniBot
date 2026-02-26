#ifndef __MESSAGES_ESPNOW_H__
#define __MESSAGES_ESPNOW_H__

#include <stdint.h>

typedef enum {
  MSG_TYPE_MOT_TEST_COMMAND = 0,
  MSG_TYPE_POSITION_COMMAND = 1,
  MSG_TYPE_POSITION_REQUEST = 2,
  MSG_TYPE_ACK_MESSAGE = 3,
  MSG_TYPE_NACK_MESSAGE = 4,
  MSG_TYPE_MAG_REQUEST = 5
} EspNowMessageType;

typedef enum {
  ERR_INVALID_MSG_SIZE = 0,
  ERR_QUEUE_FULL = 1,
  ERR_ROBOT_UNAVAILABLE = 2,
  ERR_NOT_IMPLEMENTED = 3,
  ERR_UNKNOWN_MSG = 4,
  ERR_LOW_BATTERY = 5
} EspNowErrorType;

typedef struct {
  uint8_t targetID;
  uint8_t msg_type;
  bool enabled;
  int8_t m0_vel;
  int8_t m1_vel;
} MotTestCommand;

typedef struct {
  uint8_t targetID;
  uint8_t msg_type;
  uint32_t timestamp;
  float target_x_mm;
  float target_y_mm;
  float target_a_rad;
  float move_duration_ms;
} PositionCommand;

typedef struct {
  uint8_t targetID;
  uint8_t msg_type;
  uint32_t timestamp;
} PositionRequest;

typedef struct {
  uint8_t responderID;
  uint8_t msg_type;
  uint32_t timestamp;
  float x;
  float y;
  float orientation_rad;
  float battery_voltage;
} AckMessage;

typedef struct {
  uint8_t responderID;
  uint8_t msg_type;
  uint32_t timestamp;
  uint8_t err_type;
} NackMessage;

typedef struct {
  uint8_t targetID;
  uint8_t msg_type;
  uint32_t timestamp;
} MagneticFieldRequest;

typedef struct {
  uint8_t responderID;
  uint8_t msg_type;
  uint32_t timestamp;
  float field_x_gauss;
  float field_y_gauss;
  float field_z_gauss;
} MagneticFieldResponse;

#endif // __MESSAGES_ESPNOW_H__