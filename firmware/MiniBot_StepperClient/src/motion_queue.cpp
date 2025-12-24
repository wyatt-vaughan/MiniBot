#include "motion_queue.h"
#include <stdlib.h>

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
    
    MotionCommand* heap_command = (MotionCommand*)malloc(sizeof(MotionCommand));
    if (heap_command == NULL) {
        return false;
    }
    
    *heap_command = *command;
    heap_command->next = NULL;
    
    if (queue->head == NULL) {
        queue->head = heap_command;
        queue->tail = heap_command;
    } else {
        queue->tail->next = heap_command;
        queue->tail = heap_command;
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
    
    *command = *queue->head;
    command->next = NULL;
    
    MotionCommand* old_head = queue->head;
    queue->head = queue->head->next;
    
    if (queue->head == NULL) {
        queue->tail = NULL;
    }
    
    queue->current_size--;
    free(old_head);
    
    return true;
}

void MotionQueue_Destroy(MotionQueue* queue) {
    if (queue == NULL) {
        return;
    }
    
    MotionCommand* current = queue->head;
    while (current != NULL) {
        MotionCommand* next = current->next;
        free(current);
        current = next;
    }
    
    free(queue);
}
