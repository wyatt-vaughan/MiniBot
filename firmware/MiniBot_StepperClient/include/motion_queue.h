#ifndef __MOTION_QUEUE_H__
#define __MOTION_QUEUE_H__

#include "messages_ipc.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * Queue for passing commands from cummunicator to kinematics controller
 */
typedef struct {
    MotionCommand* head;
    MotionCommand* tail;
    uint16_t max_size;
    uint16_t current_size;
} MotionQueue;

/**
 * Initialize a new motion queue
 * @param max_queue_size Maximum number of commands in the queue
 * @return Pointer to initialized MotionQueue
 */
MotionQueue* MotionQueue_Create(uint16_t max_queue_size);

/**
 * Check if queue is empty
 * @param queue Pointer to MotionQueue
 * @return true if empty, false otherwise
 */
bool MotionQueue_IsEmpty(MotionQueue* queue);

/**
 * Check if queue is full
 * @param queue Pointer to MotionQueue
 * @return true if full, false otherwise
 */
bool MotionQueue_IsFull(MotionQueue* queue);

/**
 * Get current size of queue
 * @param queue Pointer to MotionQueue
 * @return Current number of commands in queue
 */
uint16_t MotionQueue_GetSize(MotionQueue* queue);

/**
 * Add a command to the queue
 * @param queue Pointer to MotionQueue
 * @param command Pointer to MotionCommand to enqueue
 * @return true on success, false if queue is full
 */
bool MotionQueue_Enqueue(MotionQueue* queue, MotionCommand* command);

/**
 * Remove a command from the queue
 * @param queue Pointer to MotionQueue
 * @param command Pointer to MotionCommand to populate with dequeued data
 * @return true on success, false if queue is empty
 */
bool MotionQueue_Dequeue(MotionQueue* queue, MotionCommand* command);

/**
 * Destroy queue and free all resources
 * @param queue Pointer to MotionQueue
 */
void MotionQueue_Destroy(MotionQueue* queue);

#endif // __MOTION_QUEUE_H__
