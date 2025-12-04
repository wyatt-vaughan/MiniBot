#ifndef __ROBOT_H__
#define __ROBOT_H__

#include <stdint.h>

typedef class StepperDriver {
public:
    StepperDriver();
    StepperDriver(uint8_t step_pin, uint8_t dir_pin, uint8_t enable_pin, uint8_t reset_pin) : 
        step_pin(step_pin), dir_pin(dir_pin), enable_pin(enable_pin), reset_pin(reset_pin) {};
    bool initialize();
    bool enable();
    bool disable();
    bool setDirection();
    bool step();
private:
    // Robot state variables
    uint8_t step_pin;
    uint8_t dir_pin;
    uint8_t enable_pin;
    uint8_t reset_pin;
} StepperDriver;

typedef struct MotionCommand {
    float target_position_x_mm;
    float target_position_y_mm;
    float move_duration_ms;
    MotionCommand* next;
} MotionCommand;

typedef class MotionQueue {
public:
    MotionQueue(uint16_t max_queue_size = 50) : max_size(max_queue_size) {};
    bool isEmpty(bool &empty);
    bool isFull(bool &full);
    bool getSize(uint16_t &size);
    bool enqueue(MotionCommand* command);
    bool dequeue(MotionCommand &command);

private:
    MotionCommand* head;
    MotionCommand* tail;
    uint16_t max_size;
    uint16_t current_size;
} MotionQueue;

typedef class Robot {
public:
    Robot();
    void initialize();
private:
    // Robot state variables
    StepperDriver left_wheel;
    StepperDriver right_wheel;
    float positionX;
    float positionY;
    float orientation;
} Robot;

#endif // __ROBOT_H__