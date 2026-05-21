#pragma once

namespace Sensor{
    class ICM20948;
}
namespace Utils{
    class ImuCalibrator;
}


namespace Controller{

class BaroTask{
    private:
        static constexpr const char* TAG = "BaroTask";



    public:
        static void ReadBaroTask(void *pvParameters);
        void StartTask();
};
}



// 고도 계산시 GPS의 정보를 상보필터로 교정한다.


// 필터 파라미터 설정
// const float alpha = 0.98f; // 기압계 변화량에 98%, GPS 절대값에 2% 가중치

// // 전역 변수 또는 정적 변수 초기화
// float final_altitude = 0.0f; 
// float prev_baro_alt = 0.0f;
// bool is_initialized = false;

// // 센서 데이터 융합 함수 (기압계 루프 주기마다 실행, 예: 50Hz)
// float filter_altitude(float current_baro_alt, float gps_hmsl, bool is_gps_updated) {
    
//     // 1. 최초 실행 시 현재 기압계 고도로 초기화
//     if (!is_initialized) {
//         final_altitude = current_baro_alt;
//         prev_baro_alt = current_baro_alt;
//         is_initialized = true;
//         return final_altitude;
//     }

//     // 2. 기압계의 고도 변화량(Delta) 계산
//     float delta_baro = current_baro_alt - prev_baro_alt;
//     prev_baro_alt = current_baro_alt; // 다음 루프를 위해 저장

//     // 3. 상보필터 적용
//     if (is_gps_updated) {
//         // GPS 데이터가 새로 수신되었을 때 (예: 1Hz~5Hz 주기)
//         final_altitude = alpha * (final_altitude + delta_baro) + (1.0f - alpha) * gps_hmsl;
//     } else {
//         // GPS 업데이트가 없는 타이밍에는 기압계 변화량만 누적
//         final_altitude = final_altitude + delta_baro;
//     }

//     return final_altitude;
// }
