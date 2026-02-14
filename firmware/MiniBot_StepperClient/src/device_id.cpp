#include "device_id.h"
#include <Arduino.h>
#include <nvs_flash.h>

static uint8_t cached_device_id = 0xFF;
static bool nvs_initialized = false;

static void ensureNVSInitialized() {
    if (!nvs_initialized) {
        esp_err_t err = nvs_flash_init();
        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            // NVS partition was truncated, erase and retry
            nvs_flash_erase();
            err = nvs_flash_init();
        }
        nvs_initialized = true;
    }
}

static void openNVShandle(nvs_handle_t* my_handle)
{
    Serial.println("\nOpening Non-Volatile Storage (NVS) handle...");
    esp_err_t err = nvs_open("storage", NVS_READWRITE, my_handle);
    if (err != ESP_OK) {
        Serial.printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
        return;
    }
}

uint8_t getDeviceID() {
    if (cached_device_id != 0xFF) {
        return cached_device_id;
    }
    ensureNVSInitialized();
    nvs_handle_t nvshandle;
    openNVShandle(&nvshandle);
    
    int16_t dev_id = 0xFF;
    esp_err_t err = nvs_get_i16(nvshandle, "device_id", &dev_id);
    switch (err) {
        case ESP_OK:
            Serial.printf("Device ID Read successful - %d\n", dev_id);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            Serial.printf("The device ID is not initialized yet!\n");
            break;
        default:
            Serial.printf("Error (%s) reading!\n", esp_err_to_name(err));
    }
    nvs_close(nvshandle);
    cached_device_id = (uint8_t)dev_id;
    return (uint8_t)dev_id;
}

void setDeviceID(uint8_t id) {
    ensureNVSInitialized();
    nvs_handle_t nvshandle;
    openNVShandle(&nvshandle);

    int16_t dev_id = (int16_t)id;
    esp_err_t err = nvs_set_i16(nvshandle, "device_id", dev_id);
    if (err != ESP_OK) {
        Serial.printf("Failed to write device ID!\n");
    }
    else {
        Serial.printf("Device ID set to: 0x%02X\n", id);
    }
    nvs_close(nvshandle);
    return;
}
