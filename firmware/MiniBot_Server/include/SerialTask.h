#ifndef SERIAL_TASK_H
#define SERIAL_TASK_H

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Task handle
extern TaskHandle_t serialTaskHandle;

// Initialize serial task resources (Serial itself is started in main.cpp)
void initSerial();

// FreeRTOS serial task
void serialTask(void *parameter);

#endif // SERIAL_TASK_H
