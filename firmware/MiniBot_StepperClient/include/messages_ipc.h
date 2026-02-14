#ifndef __MESSAGES_IPC_H__
#define __MESSAGES_IPC_H__

/**
 * Motion command structure passed from communicator to kinematics controller
 */
typedef struct {
    float target_position_x_mm;
    float target_position_y_mm;
    float target_orientation_rad;
    float move_duration_ms;
} MotionCommand;

#endif // __MESSAGES_IPC_H__