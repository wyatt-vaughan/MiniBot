#include "motion_queue.h"
#include <stdlib.h>

// ============================================================================
// Motion Queue Implementation
// ============================================================================

MotionQueue* MotionQueue_Create(uint16_t max_queue_size) {
    MotionQueue* queue = (MotionQueue*)malloc(sizeof(MotionQueue));
    if (queue == NULL) {
        return NULL;
    }
    
    queue->head = NULL;
    queue->tail = NULL;
    queue->max_size = max_queue_size;
    queue->current_size = 0;
    
    return queue;
}

bool MotionQueue_IsEmpty(MotionQueue* queue) {
    if (queue == NULL) {
        return true;
    }
    
    return queue->current_size == 0;
}

bool MotionQueue_IsFull(MotionQueue* queue) {
    if (queue == NULL) {
        return true;
    }
    
    return queue->current_size >= queue->max_size;
}

uint16_t MotionQueue_GetSize(MotionQueue* queue) {
    if (queue == NULL) {
        return 0;
    }
    
    return queue->current_size;
}

bool MotionQueue_Enqueue(MotionQueue* queue, MotionCommand* command) {
    if (queue == NULL || command == NULL) {
        return false;
    }
    
    if (MotionQueue_IsFull(queue)) {
        return false;
    }
    
    command->next = NULL;
    
    if (queue->head == NULL) {
        // Queue is empty
        queue->head = command;
        queue->tail = command;
    } else {
        // Add to tail
        queue->tail->next = command;
        queue->tail = command;
    }
    
    queue->current_size++;
    return true;
}

bool MotionQueue_Dequeue(MotionQueue* queue, MotionCommand* command) {
    if (queue == NULL || command == NULL) {
        return false;
    }
    
    if (MotionQueue_IsEmpty(queue)) {
        return false;
    }
    
    // Copy data from head to output structure
    *command = *queue->head;
    command->next = NULL;
    
    // Remove from queue
    MotionCommand* old_head = queue->head;
    queue->head = queue->head->next;
    
    if (queue->head == NULL) {
        queue->tail = NULL;
    }
    
    queue->current_size--;
    
    // Note: We don't free old_head here as the caller may need to manage memory
    
    return true;
}

void MotionQueue_Destroy(MotionQueue* queue) {
    if (queue == NULL) {
        return;
    }
    
    // Free all commands in queue
    MotionCommand* current = queue->head;
    while (current != NULL) {
        MotionCommand* next = current->next;
        free(current);
        current = next;
    }
    
    free(queue);
}
