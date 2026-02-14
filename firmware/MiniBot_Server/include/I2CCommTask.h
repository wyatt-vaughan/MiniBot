#ifndef I2C_COMM_TASK_H
#define I2C_COMM_TASK_H

#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Task handle
extern TaskHandle_t i2cCommTaskHandle;

// I2C Configuration
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_CLOCK_SPEED 400000  // 400kHz

// Initialize I2C communication
void initI2CComm();

// FreeRTOS task function
void i2cCommTask(void *parameter);

#endif // I2C_COMM_TASK_H
