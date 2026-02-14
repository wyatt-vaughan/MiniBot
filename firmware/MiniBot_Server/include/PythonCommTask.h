#ifndef PYTHON_COMM_TASK_H
#define PYTHON_COMM_TASK_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Task handle
extern TaskHandle_t pythonCommTaskHandle;

// Initialize Python communication (Serial2)
void initPythonComm();

// FreeRTOS task function
void pythonCommTask(void *parameter);

#endif // PYTHON_COMM_TASK_H
