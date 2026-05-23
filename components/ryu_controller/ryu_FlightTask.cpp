#include "ryu_FlightTask.hpp"

#include <esp_timer.h>
#include <esp_log.h>
#include <esp_task_wdt.h>


#include "ryu_Config.hpp"
#include "ryu_SharedDataManager.hpp"
#include "ryu_KalmanFilter.hpp"
#include "ryu_ImuSensorTask.hpp"
#include "ryu_MagSensorTask.hpp"
#include "ryu_BaroSensorTask.hpp"
#include "ryu_espnow.hpp"
#include "ryu_mavlink.hpp"
#include "ryu_timer.hpp"
#include "ryu_battery.hpp"
#include "ryu_gps.hpp"

namespace Controller {

esp_err_t Flight::initialize(){
    esp_err_t err = ESP_OK;
    if (!Driver::Battery::get_instance().is_initialized()){
        err = Driver::Battery::get_instance().initialize();
    }
    
    if(!SharedDataManager::getInstance().is_initialized()){
        err = SharedDataManager::getInstance().initialize();
    }

    if(!Sensor::Gps::getInstance().is_initialized()){
        Sensor::Gps::getInstance().initialize();
        Sensor::Gps::getInstance().StartTask();
    }

    if(!ImuSensorTask::getInstance().is_initialized()){
        err = ImuSensorTask::getInstance().initialize();
        ImuSensorTask::getInstance().StartTask();
    }

    if(!MagSensorTask::getInstance().is_initialized()){
        MagSensorTask::getInstance().initialize();
        MagSensorTask::getInstance().StartTask();
    }



    if(!BaroSensorTask::getInstance().is_initialized()){
        err = BaroSensorTask::getInstance().initialize();
        BaroSensorTask::getInstance().StartTask();
    }

    if(!Service::EspNow::get_instance().is_initialized()){
        Service::EspNow::get_instance().initialize();
        Service::EspNow::get_instance().StartTask();
        Service::EspNow::get_instance().connect_callback();
    }

    if(!Service::Timer::get_instance().is_initialized()){
        Service::Timer::get_instance().intiallize();
        Service::Timer::get_instance().Start();
    }

    if(!Service::Mavlink::get_instance().is_initialized()){
        Service::Mavlink::get_instance().initialize();
        Service::Mavlink::get_instance().StartTask();
    }

    return err;
}

esp_err_t Flight::deinitialize(){
    return ESP_OK;
}

void Flight::flight_task(void *pvParameters)
{
    esp_task_wdt_add(nullptr);
    //Flight* flight = static_cast<Flight*>(pvParameters);

    // 1. 중계자 데이터 매니저 가져오기
    auto& sharedData = SharedDataManager::getInstance();

    // 2. 칼만 필터 코어 초기화 (NED 기준)
    auto& kalman = Filter::KalmanFilter::getInstance();
    kalman.init(0.0f, 0.0f, 0.0f);


    uint32_t loop_cnt = 0;        
    SensorData cur_imu_data {};
    Vector3f   cur_mag_data {};
    ESP_LOGI(TAG, "Flight 제어 태스크가 Core 1에서 완벽한 데이터 동기화 모드로 가동되었습니다.");
    //SensorTask의 준비되어질 시간을 기다려줌. 300이면 1~2ms가 부족하다
    vTaskDelay(pdMS_TO_TICKS(320));

    while (true) {
        // [초고속 저지연 파이프라인] Core 0의 센서 태스크가 매니저에 데이터를 쓰고 신호를 줄 때까지 대기
        // 1ms 주기로 신호가 인입되므로, 센서 차단 등 비상시 탈출을 위해 타임아웃 마진을 5ms로 설정
        uint32_t notification_value = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));
        
        if (notification_value > 0){
            esp_task_wdt_reset(); 

            // 센서 캘리브레이션(0점 조절)이 완료될 때까지는 필터 연산을 유보하고 대기
            if (!sharedData.is_imu_calibrated()) {
                vTaskDelay(pdMS_TO_TICKS(1)); 
                continue; 
            }


            cur_imu_data = sharedData.get_shared_data< Data_type::DT_IMU_DATA>();
            static Vector3f previousMag{};
            //ist8310작동시 ist8310의 데이터로 대치
            if(sharedData.is_mag_updated()){
                cur_mag_data = sharedData.get_shared_data< Data_type::DT_MAG_DATA>();                
                //cur_imu_data.mag = cur_mag_data;   // 지자계를 대체한다.
                //previousMag = cur_mag_data;
            }else{ // 데이터를 일지 않을때에는 이전값을 보낸다.
                //cur_imu_data.mag = previousMag;
            }
            
            // 입력 데이터 가공 (입력이 도/초 단위일 경우 예측부 라디안 스케일링 일치 처리)
            Vector3f gyro_rad = cur_imu_data.gyro * DEG_TO_RAD;

            // [EKF 핵심 엔진 가동] 자이로 예측 후 가속도/지자계 순차 보정 처리
            kalman.update(cur_imu_data.acc,gyro_rad, cur_imu_data.mag,dt);
                        
            // 진북 기준 최종 오일러 각 추출 (라디안 단위)
            Attitude_t attitude = kalman.getEuler();

            // 제어 및 외부 송신을 위해 도(Degree) 단위로 변환
            attitude = attitude * RAD_TO_DEG;
            
            // [필수] TARGET_TRUE_NORTH는 반드시 태스크 내부 static 또는 클래스 멤버 변수여야 합니다.
            // 초기값은 우리나라 평균 편각인 -7.7f (서편각 7.7도)로 시작합니다.
            const static float target_true_north = TARGET_TRUE_NORTH; 

            // 1. GPS가 업데이트 되었을 때만 지자기 편각 필터링 수행 (Low-Pass Filter)
            if (SharedDataManager::getInstance().is_gps_updated()) {
                gps_data_t mgps = SharedDataManager::getInstance().get_shared_data<Data_type::DT_GPS_DATA>();
                
                // GPS 가 정상 Fix 상태여야 magDec 신뢰도가 높습니다.
                if (mgps.fixType >= 3 && mgps.horAcc < 3000) { 
        
                    // mgps.magDec는 이미 '도(Degree)' 단위이므로 스케일링 없이 그대로 필터 적용
                    //target_true_north = (target_true_north * 0.999f) + (mgps.magDec * 0.001f);
                }
            }

            // 3. 자북 방위각에 '단 한 번만' 편각을 더하여 진북 방위각 생성 (누적 방지)
            float corrected_yaw = attitude.yaw + target_true_north;

            // Yaw 각도 범위를 0~360도로 정규화 바인딩
            if (attitude.yaw < 0.0f)           attitude.yaw += 360.0f;
            else if (attitude.yaw >= 360.0f)   attitude.yaw -= 360.0f;

            // 5. 최종 보정된 진북 기준의 yaw를 자세 제어(PID) 알고리즘에 투입
            attitude.yaw = corrected_yaw; 

            // QGC 모니터링 전용 Mavlink 버퍼 구조체 데이터 밀어넣기
            attitude.data[3] = cur_imu_data.gyro.x;
            attitude.data[4] = cur_imu_data.gyro.y;
            attitude.data[5] = cur_imu_data.gyro.z;

            // [중계자 복귀] 최종 수렴된 현재 수평 자세를 데이터 매니저에 즉시 업데이트
            sharedData.publish_data<Data_type::DT_CURRENT_ATTITUDE>(attitude);
            
            BaroData baroData{};
            if(sharedData.is_baro_updated()){ //40ms단위로 데이터가 들어온다.
                baroData =  sharedData.get_shared_data<Data_type::DT_BARO_DATA>();
                // ESP_LOGI(TAG, "|R: %8.5f |P: %8.5f |Y: %8.5f| gnd_pressure : %8.5f | pressure: %8.5f | alt:%8.5f | climb_rate: %8.5F", 
                //         attitude.roll,        attitude.pitch,       attitude.yaw,
                //         baroData.gnd_pressure ,baroData.pressure, baroData.altitude,baroData.climb_rate
                //         );
            }
            // 여기에 추후 PID 제어 루프를 탑재하시면 됩니다.
            // run_pid_control(attitude, cur_imu_data.gyro);

            // [출력 가독성 최적화] UART 병목 및 로깅 오버헤드를 막기 위한 50Hz(20ms) 주기 필터링 로그
            // if (++loop_cnt >= 20) { 
            //     loop_cnt = 0;
            //     ESP_LOGI(TAG, "|AX: %8.5f |AY: %8.5f |AZ: %8.5f | GX: %8.5f |GY: %8.5f |GZ: %8.5f | MX: %8.5f |MY: %8.5f |MZ: %8.5f |R: %8.5f |P: %8.5f |Y: %8.5f|", 
            //             cur_imu_data.acc.x,   cur_imu_data.acc.y,   cur_imu_data.acc.z,
            //             gyro_rad.x,           gyro_rad.y,           gyro_rad.z,
            //             cur_imu_data.mag.x,   cur_imu_data.mag.y,   cur_imu_data.mag.z,
            //             attitude.roll,        attitude.pitch,       attitude.yaw
             
            //         );
            // }
        } else {
            // Failsafe 트리거: 5ms 동안 Core 0로부터 동기화 신호(Notification)가 누락된 상황 예외 처리
            // SensorTask에서 신호가 안오면 작동이 불능이 되므로 이곳이 실행되어진다.
            ESP_LOGW(TAG, "비상: 센서 데이터 동기화 신호 지연 감지!");
        }
    }
}

esp_err_t  Flight::StartTask()
{
    // 최상위 우선순위(configMAX_PRIORITIES - 1)로 가용한 최고 권력을 할당하여 Core 1에 전적 격리 구동
    auto res = xTaskCreatePinnedToCore(
        flight_task,                
        "flight_task",              
        8192, 
        this,                       
        configMAX_PRIORITIES - 1,   
        &_taskHandle,                    
        1 
    );
    if (res != pdTRUE){
        return ESP_FAIL;
    }
    return ESP_OK;
}

} // namespace Controller
