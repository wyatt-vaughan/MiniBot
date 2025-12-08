#include "esp_now_communicator.h"

// Static variables for task state
static MotionQueue* comm_queue = NULL;

bool EspNowCommunicator_Init(MotionQueue* motion_queue) {
    if (motion_queue == NULL) {
        return false;
    }
    
    comm_queue = motion_queue;
    
    // TODO: Initialize ESP-NOW protocol
    // Set up WiFi radio, register callbacks for received messages
    
    return true;
}

void EspNowCommunicator_Task(void* pvParameters) {
    // Extract robot pointer from task parameters
    Robot* robot = (Robot*)pvParameters;
    
    // Task initialization
    if (comm_queue == NULL || robot == NULL) {
        vTaskDelete(NULL);
        return;
    }
    MotionCommand testmc0 = MotionCommand{20.0f, 0.0f, 0.0f, 2000, NULL};
    MotionCommand testmc1 = MotionCommand{20.0f, 0.0f, 2.0f, 2000, NULL};
    MotionCommand testmc2 = MotionCommand{30.0f, 30.0f, 0.0f, 2000, NULL};
    MotionCommand testmc3 = MotionCommand{0.0f, 0.0f, 0.0f, 2000, NULL};

    randomSeed(esp_random());

    vTaskDelay(pdMS_TO_TICKS(2000));
    
    while (1) {
        Serial.println("ESP-NOW Communicator Task loop started");
        
        // FOR TEST ONLY, SIMULATE RECEIVED COMMANDS
        if (MotionQueue_Enqueue(comm_queue, &testmc0))
            Serial.println("Enqueued testmc0");
        else
            Serial.println("Failed to enqueue testmc0");
        vTaskDelay(pdMS_TO_TICKS(5000));
        
        if (MotionQueue_Enqueue(comm_queue, &testmc1))
            Serial.println("Enqueued testmc1");
        else
            Serial.println("Failed to enqueue testmc1");
        vTaskDelay(pdMS_TO_TICKS(10));

        if (MotionQueue_Enqueue(comm_queue, &testmc2))
            Serial.println("Enqueued testmc2");
        else
            Serial.println("Failed to enqueue testmc2");
        vTaskDelay(pdMS_TO_TICKS(10));

        MotionCommand testmcRAND = MotionCommand{(float)random(0, 101), (float)random(0, 101), (float)random(0, 100) / 20, 5000, NULL};
        if (MotionQueue_Enqueue(comm_queue, &testmcRAND))
            Serial.println("Enqueued testmcRAND");
        else
            Serial.println("Failed to enqueue testmcRAND");
        vTaskDelay(pdMS_TO_TICKS(10000));

        if (MotionQueue_Enqueue(comm_queue, &testmc3))
            Serial.println("Enqueued testmc3");
        else
            Serial.println("Failed to enqueue testmc3");
        vTaskDelay(pdMS_TO_TICKS(5000));
        
    }
}
