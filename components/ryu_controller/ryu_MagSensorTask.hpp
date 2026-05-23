#pragma once

#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace Controller {

class MagSensorTask {
private:
    static constexpr const char* TAG = "MagSensorTask";
    MagSensorTask() = default;
    ~MagSensorTask() = default;
    
    TaskHandle_t _taskHandle = nullptr;

    // 백그라운드 스레드 함수
    static void ReadMagSensorTask(void* pvParameters);

public:
    static MagSensorTask& getInstance() {
        static MagSensorTask instance;
        return instance;
    }
    MagSensorTask(const MagSensorTask&) = delete;
    MagSensorTask& operator=(const MagSensorTask&) = delete;

    esp_err_t initialize();
    bool is_initialized(){return _initialized;};
    void StartTask();
    
    TaskHandle_t getTaskHandle() const { return _taskHandle; }
    bool _initialized = false;
};

} // namespace Controller
