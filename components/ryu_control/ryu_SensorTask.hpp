#pragma once

// 전방 선언 (Forward Declaration)을 통해 컴파일 속도를 최적화합니다.
namespace Sensor {
    class ICM20948;
}
namespace Utils {
    class SharedDataManager;
}

namespace Service { 

class SensorTask {
private:        
    // [방어 설계] 모든 센서 인스턴스와 매니저 포인터는 private 영역에 격리합니다.
    Sensor::ICM20948            *_icm20948;
    Utils::SharedDataManager    *_data_manager;

public:
    inline static constexpr int SPI_CS_PIN = 9;
    // [방어 설계] 생성자에서 모든 포인터를 nullptr로 확실하게 초기화하여 쓰레기 값을 방지합니다.
    SensorTask() 
        : _icm20948(nullptr), _data_manager(nullptr) {}

    // [방어 설계] 동적 할당된 센서 객체를 안전하게 해제하도록 소멸자를 명시합니다.
    ~SensorTask();

    // FreeRTOS 태스크 진입용 static 함수
    static void ReadSensorTask(void* pvParameters);
    
    // 태스크 실행 생성 헬퍼
    void StartTask();
};

} // namespace Service
