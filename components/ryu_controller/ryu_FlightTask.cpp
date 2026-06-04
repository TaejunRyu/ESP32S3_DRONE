#include "ryu_FlightTask.hpp"


#include <esp_timer.h>
#include <esp_log.h>
#include <esp_task_wdt.h>


#include "ryu_Config.hpp"
#include "ryu_SharedDataManager.hpp"
#include "ryu_KalmanFilter.hpp"
#include "ryu_VerticalFilter.hpp"
#include "ryu_ImuSensorTask.hpp"
#include "ryu_MagSensorTask.hpp"
#include "ryu_BaroSensorTask.hpp"
#include "ryu_espnow.hpp"
#include "ryu_mavlink.hpp"
#include "ryu_timer.hpp"
#include "ryu_battery.hpp"
#include "ryu_gps.hpp"
#include "ryu_PidController.hpp"
#include "ryu_motor.hpp"
#include "ryu_StateManager.hpp"

namespace Controller {

esp_err_t Flight::initialize(){
    esp_err_t err = ESP_OK;
    
    if(!Service::EspNow::get_instance().is_initialized()){
        Service::EspNow::get_instance().initialize();
        Service::EspNow::get_instance().StartTask();
        Service::EspNow::get_instance().connect_callback();
    }

    if (!Driver::Battery::get_instance().is_initialized()){
        err = Driver::Battery::get_instance().initialize();
    }
    
    if(!SharedDataManager::getInstance().is_initialized()){
        err = SharedDataManager::getInstance().initialize();
    }

    PidControl::getInstance().reset();

    if(!Sensor::Gps::getInstance().is_initialized()){
        Sensor::Gps::getInstance().initialize();
        Sensor::Gps::getInstance().StartTask();
    }

   
    if(!MagSensorTask::getInstance().is_initialized()){
        MagSensorTask::getInstance().initialize();
        MagSensorTask::getInstance().StartTask();
    }


    if(!BaroSensorTask::getInstance().is_initialized()){
        err = BaroSensorTask::getInstance().initialize();
        BaroSensorTask::getInstance().StartTask();
    }


    if(!ImuSensorTask::getInstance().is_initialized()){
        err = ImuSensorTask::getInstance().initialize();
        ImuSensorTask::getInstance().StartTask();
    }

    if(!Service::Timer::get_instance().is_initialized()){
        Service::Timer::get_instance().intiallize();
        Service::Timer::get_instance().Start();
    }

    if(!Service::Mavlink::get_instance().is_initialized()){
        Service::Mavlink::get_instance().initialize();
        Service::Mavlink::get_instance().StartTask();
    }

    if(!Driver::Motor::get_instance().is_initialized()){
        err = Driver::Motor::get_instance().initialize();
    }

    return err;
}

esp_err_t Flight::deinitialize(){
    return ESP_OK;
}

void Flight::flight_task(void *pvParameters)
{

    // esp_task_wdt_config_t wdt_config = {
    //     .timeout_ms = 10000,                            // 💡 원하는 시간 입력 (예: 10000ms = 10초)
    //     .idle_core_mask = (1 << CONFIG_FREERTOS_NUMBER_OF_CORES) - 1, // 모든 CPU 코어 감시
    //     .trigger_panic = true                           // 타임아웃 시 핵심 덤프 후 리셋
    // };

    // // 현재 구동 중인 워치독 타이머의 설정을 10초로 재구성합니다.
    // esp_err_t err = esp_task_wdt_reconfigure(&wdt_config);
    // if (err == ESP_OK) {
    //     ESP_LOGI("WDT", "태스크 워치독 시간이 성공적으로 늘어났습니다.");
    // }


    // esp_task_wdt_add(nullptr);
    Flight* flight = static_cast<Flight*>(pvParameters);

    // 1. 중계자 데이터 매니저 가져오기
    auto& sharedData = SharedDataManager::getInstance();

    // 2. 칼만 필터 코어 초기화 (NED 기준)
    auto& kalman = Filter::KalmanFilter::getInstance();
    kalman.init(0.0f, 0.0f, 0.0f);
    

    auto& v_kalman = Filter::VerticalFilter::getInstance();

    auto& pid = PidControl::getInstance();

    


    // 2. [고도 게인 튜닝 파라미터 독립 주입]
    Controller::AltitudeParams alt_config;
    alt_config.kp_alt         = 1.2f;   // 고도 -> 속도 변환율
    alt_config.kp_vel         = 1.8f;   // 속도 P
    alt_config.ki_vel         = 0.4f;   // 속도 I
    alt_config.kd_vel         = 0.01f;  // 속도 D
    alt_config.vel_limit      = 2.0f;   // 최대 2m/s
    alt_config.hover_throttle = 43.5f;  // 이 기체의 공중 유지 호버링 스로틀 추정치 43.5%
    alt_config.out_limit      = 85.0f;  // 최대 출력 한계 제한
    pid.setAltitudeParams(alt_config);

  
    // 2. PID 파라미터 초기화 (45cm급 소형 쿼드 기준 초기값)
    // 값 형식: {Kp, Ki, Kd, I_Limit, Output_Limit}
    Controller::PidParams angle_gain = {2.5f, 0.05f, 0.02f, 0.5f, 10.0f}; // 바깥 루프: 각도 오차 → 목표 각속도
    Controller::PidParams rate_gain  = {0.12f, 0.02f, 0.001f, 1.0f, 50.0f}; // 안쪽 루프: 목표 각속도 → 모터 출력

    pid.setAngleParams(angle_gain, angle_gain, angle_gain); // Roll, Pitch, Yaw 동일 적용 예시
    pid.setRateParams(rate_gain, rate_gain, rate_gain);
    pid.reset();


    // 가상 목표 및 센서 데이터 선언
    Attitude_t target_pose = {0.0f, 0.0f, 0.0f}; // 정밀 호버링 (평평한 상태) 목표

    uint32_t    loop_cnt = 0;        
    SensorData  cur_imu_data {};
    Vector3f    cur_mag_data {};
    
    //SensorTask의 준비되어질 시간을 기다려줌. 300이면 1~2ms가 부족하다
    vTaskDelay(pdMS_TO_TICKS(320));
    
    // 디버그용 GPIO 핀 설정 (예: LED 토글로 루프 주기 측정)
    #define DEBUG_GPIO_PIN  GPIO_NUM_17 // 남는 GPIO 핀 지정

    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << DEBUG_GPIO_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE; // ◀ 명시적 인터럽트 해제 추가
    gpio_config(&io_conf);

    while (true) {

        uint32_t notification_value = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));

        if (notification_value <= 0)[[unlikely]]{  
            ESP_LOGW(TAG, "비상: 센서 데이터 동기화 신호 지연 감지! (Notification Value: %u)", notification_value);                       
            continue; // 다음 루프로 넘어가서 다시 대기 (Failsafe: 데이터 지연 시 제어 연산 건너뛰고 대기 상태 유지)
        }   

        // 센서 캘리브레이션(0점 조절)이 완료될 때까지는 필터 연산을 유보하고 대기
        if (!sharedData.is_imu_calibrated())[[unlikely]] {
            vTaskDelay(pdMS_TO_TICKS(1)); 
            continue; 
        }
        uint64_t start_time = esp_timer_get_time();


        gpio_set_level(DEBUG_GPIO_PIN, 1); // 루프 시작: 핀을 High로!

        cur_imu_data = sharedData.get_shared_data< Data_type::DT_IMU_DATA>();
        // if(cur_imu_data.is_mag_updated){
        //     cur_imu_data.mag.normalize();
        // }

        //  IST8310 MagSensorTask에서 보내온 데이터를 받는다.
        { // 이블럭을 제거하면 ak09916으로 mag가 대체되어진다.
            cur_imu_data.mag = 0.0f;
            if(sharedData.is_mag_updated()){  // 업데이트 될때만 받아와서 적용한다.
                cur_mag_data = sharedData.get_shared_data< Data_type::DT_MAG_DATA>();
                cur_imu_data.mag = cur_mag_data;   // 지자계를 대체한다.
                //cur_imu_data.mag.normalize();        
            }
        }

        kalman.update(  cur_imu_data.acc,
                        cur_imu_data.gyro * DEG_TO_RAD, //// 입력 데이터 가공 (입력이 도/초 단위일 경우 예측부 라디안 스케일링 일치 처리)
                        cur_imu_data.mag,
                        dt);

        // 진북 기준 최종 오일러 각 추출 (라디안 단위)
        Attitude_t curAttitude = kalman.getEuler();


        // if (++loop_cnt >= 20) { 
        //     loop_cnt = 0;
        //     ESP_LOGW(TAG, "acc.x: %8.4f, acc.y: %8.4f, acc.z: %8.4f, gyro.x: %8.4f, gyro.y: %8.4f, gyro.z: %8.4f, Roll: %8.4f, Pitch: %8.4f, Yaw: %8.4f", 
        //        cur_imu_data.acc.x, cur_imu_data.acc.y, cur_imu_data.acc.z, cur_imu_data.gyro.x, cur_imu_data.gyro.y, cur_imu_data.gyro.z, curAttitude.roll, curAttitude.pitch, curAttitude.yaw); 
        // }


        // 초기값은 우리나라 평균 편각인 -7.7f (서편각 7.7도)로 시작합니다.
        static float target_true_north = TARGET_TRUE_NORTH * (M_PI / 180.0f); 

        // 1. GPS가 업데이트 되었을 때만 지자기 편각 필터링 수행 (Low-Pass Filter)
        if (SharedDataManager::getInstance().is_gps_updated()) {
            gps_data_t mgps = SharedDataManager::getInstance().get_shared_data<Data_type::DT_GPS_DATA>();
            // GPS 가 정상 Fix 상태여야 magDec 신뢰도가 높습니다.
            if (mgps.fixType >= 3 && mgps.horAcc < 3000) { 
                // mgps.magDec는 이미 '도(Degree)' 단위이므로 스케일링 없이 그대로 필터 적용
                target_true_north = (target_true_north * 0.999f) + ((mgps.magDec *DEG_TO_RAD * 0.001f));
            }
        }
                    
        // 3. 자북 방위각에 '단 한 번만' 편각을 더하여 진북 방위각 생성 (누적 방지)
        curAttitude.yaw = curAttitude.yaw + target_true_north;

        // 4. NED 좌표계 표준 경계선 처리 (-PI ~ +PI) 필수 수행
        while (curAttitude.yaw > M_PI)  curAttitude.yaw -= 2.0f * M_PI;
        while (curAttitude.yaw < -M_PI) curAttitude.yaw += 2.0f * M_PI;
        
        // 4. 자북 기준의 최신 자세에서 수직 가속도 성분만을 추출하여 칼만 필터 예측 단계에 투입할 '순수 수직 가속도'로 가공
        float pure_vertical_accel = kalman.get_pure_vertical_accel(cur_imu_data.acc);

        // 5. 1ms 주기로 수직 칼만필터 시간 예측 단계 실행
        v_kalman.predict(pure_vertical_accel, dt);

        //[목표 고도] ➔ Outer Loop (고도 P 제어) ➔ [목표 상승/하강 속도] ➔ Inner Loop (속도 PID) ➔ [최종 Throttle]
        BaroData baroData{};
        if(sharedData.is_baro_updated()){ //40ms단위로 데이터가 들어온다.
            baroData =  sharedData.get_shared_data<Data_type::DT_BARO_DATA>();
            v_kalman.update(baroData.altitude); // 40ms 주기 보정
            // ESP_LOGI(TAG, "Baro -> gnd_pressure: %5.2f, pressure: %5.2f, altitude: %5.2f", 
            //             baroData.gnd_pressure ,baroData.pressure,baroData.altitude);
        }

        // gps의 고도와 융합.
        if(sharedData.is_gps_updated()){ 
            gps_data_t gpsData = sharedData.get_shared_data<Data_type::DT_GPS_DATA>();                
            // 위성이 최소 3D Fix(3 이상) 이상 잡히고 정밀도가 신뢰할 만할 때만 필터 보정에 주입
            if (gpsData.fixType >= 3 && gpsData.verAcc < 4000) {
                // gpsData.horMSL(해수면 고도) 또는 지면 기준 고도 변수 매핑
                v_kalman.updateGPS(gpsData.horMSL); 
            }
        }

        // 7. 정제된 1000Hz 고도/속도로 고도 PID 연산 제어 처리...
        float current_alt = v_kalman.getAltitude();
        float current_vel = v_kalman.getVelocity();

        // if (++loop_cnt >= 20) { 
        //     loop_cnt = 0;
        //     ESP_LOGI(TAG, "Altitude -> est_alt: %5.2f, est_vel: %5.2f", current_alt,current_vel);
        // }

        rc_data_t rc_data{};
        static float target_rc_throttle = 0.0f;        
        //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
        //  flysky에서 데이터가 들어오면  qgc에서 오는 rc데이터는 무시하도록 한다.  flysky가 우선순위가 높다.  
        // flysky에서 싱호가 들어오면 qgc에서 오는 신호를 어떻게 무시하게 할까?
        //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
        rc_data = sharedData.get_shared_data<Data_type::DT_RC_DATA>();
        
        if(sharedData.is_rc_updated()){
            // flysky 모드에서는 RC 입력을 직접적으로 목표 자세에 반영 (예: 스틱 조작이 즉각적으로 자세 목표에 영향)
            if(rc_data.type == RC_FLYSKY){    
                // 입력되어진 rc에 대해서 30도만 목표 자세에 반영하도록 범위를 제한하여 극단적 입력에 대한 안전장치 역할 수행
                // 데이터를 30도로 스케일링하여 목표 자세에 적용 (예: 최대 ±30도 범위로 제한)
                target_pose.roll    = rc_data.roll * 0.5f * DEG_TO_RAD;   // 최대 ±30도 (0.5배 스케일링)
                target_pose.pitch   = rc_data.pitch * 0.5f * DEG_TO_RAD;
                target_pose.yaw     = rc_data.yaw * 0.5f * DEG_TO_RAD;
                target_rc_throttle  = rc_data.throttle; // 스로틀은 그대로 반영하여 고도 제어와 병행 가능하도록 합니다.

            } else if (rc_data.type == RC_QGC) {  
                // 다른 RC 타입이 활성화된 경우, 안전을 위해 목표 자세를 초기화하거나 유지
                target_pose.roll    = rc_data.roll * 0.5f * DEG_TO_RAD;   // 최대 ±30도 (0.5배 스케일링)
                target_pose.pitch   = rc_data.pitch * 0.5f * DEG_TO_RAD;
                target_pose.yaw     = rc_data.yaw * 0.5f * DEG_TO_RAD;
                target_rc_throttle  = rc_data.throttle; // 스로틀은 그대로 반영하여 고도 제어와 병행 가능하도록 합니다.
            } else if (rc_data.type == RC_NONE) {
                // RC 입력이 없는 경우, 안전한 기본 자세 유지 또는 고도 홀드 모드로 전환
                // is_user_hold_mode = true; // RC 입력이 없을 때 고도 홀드 모드 활성화 (선택 사항)                
                target_pose = {0.0f, 0.0f, 0.0f};
                target_rc_throttle = 0.0f; // 스로틀도 안전하게 초기화
            }
            // ESP_LOGI(TAG, "RC Input -> Throttle: %5.2f, Roll: %5.2f, Pitch: %5.2f, Yaw: %5.2f, HoldMode: %d", 
            //             rc_data.throttle, rc_data.roll, rc_data.pitch, rc_data.yaw, is_user_hold_mode);
        }

        // 8. 자이로 데이터에 간단한 저역 통과 필터 적용 (노이즈 완화)
        static Vector3f filtered_rate = {0.0f, 0.0f, 0.0f};
        const float alpha = 0.3f; 
        filtered_rate.x = alpha * cur_imu_data.gyro.x * DEG_TO_RAD + (1.0f - alpha) * filtered_rate.x;
        filtered_rate.y = alpha * cur_imu_data.gyro.y * DEG_TO_RAD + (1.0f - alpha) * filtered_rate.y;
        filtered_rate.z = alpha * cur_imu_data.gyro.z * DEG_TO_RAD + (1.0f - alpha) * filtered_rate.z;

        //-----------------------------------------------------------------------------------------------------
        // 여기까지 모든 데이터는 준비되었음 (센서,RC,GPS,필터링......)
        // 이제부터는 제어 연산과 모터 믹싱에만 집중하여 최적화된 연산 파이프라인으로 처리합니다.
        //-----------------------------------------------------------------------------------------------------
        
        // [고도 홀드 모드] RC 스로틀을 고도 목표로 활용하는 사용자 홀드 모드 처리
        static float hold_target_altitude = 1.5f;
        static bool is_user_hold_mode = false;
        flyingMode_e current_mode{};
        
        Controller::DroneStatusManager::getInstance().checkAndGetFlyingMode(current_mode);
        if ((current_mode == flyingMode_e::MODE_STANBY) || (current_mode == flyingMode_e::MODE_ALTCTL)) {
            // 홀드 모드 진입 시 현재 고도를 목표 고도로 설정하여 부드러운 전환 유도
            hold_target_altitude = current_alt;
            float throttle_offset = (rc_data.throttle - 50.0f) * 0.01f;
            hold_target_altitude += throttle_offset * dt;
            hold_target_altitude = std::clamp(hold_target_altitude, 0.1f, 10.0f);
            is_user_hold_mode = true;
        }else{
            is_user_hold_mode = false;
        }

        // -------------------------------------------------------------
        // 핵심 연산: 고도 제어 명령을 홀드 모드에서만 활성화하고,
        // 일반 모드에서는 RC 스로틀을 그대로 베이스 출력으로 사용합니다.
        // -------------------------------------------------------------
        float altitude_throttle{};
        if (is_user_hold_mode) {
            altitude_throttle = pid.updateAltitudeCascade(hold_target_altitude, current_alt, current_vel, dt);
            altitude_throttle = std::clamp(altitude_throttle, 10.0f, 85.0f);
            // if (++loop_cnt >= 20) { 
            //     loop_cnt = 0;
            //     ESP_LOGI(TAG, "Hold Mode -> Target Alt: %5.2f, Hold Throttle: %5.2f", hold_target_altitude, altitude_throttle);
            // }
        }

        // -------------------------------------------------------------
        // 핵심 연산: 자세 제어 명령을 병렬 독립 연산 처리
        // -------------------------------------------------------------
        Vector3f att_outputs = pid.updateCascade(target_pose, curAttitude, filtered_rate, dt);
        float base_throttle{};
        if(is_user_hold_mode)   base_throttle = altitude_throttle;
        else                    base_throttle = std::clamp(target_rc_throttle, 10.0f, 85.0f);
        

        static bool is_armed{};
        DroneStatusManager::getInstance().checkAndGetArmed(is_armed);
        if (is_armed) {
            // -------------------------------------------------------------
            // 최종 믹싱 단계: 베이스 스로틀에 자세 복원력을 축별 가감산
            // -------------------------------------------------------------
            float m1_fr = std::clamp(base_throttle - att_outputs.x - att_outputs.y - att_outputs.z, 0.0f, 100.0f);
            float m2_bl = std::clamp(base_throttle + att_outputs.x + att_outputs.y - att_outputs.z, 0.0f, 100.0f);
            float m3_fl = std::clamp(base_throttle + att_outputs.x - att_outputs.y + att_outputs.z, 0.0f, 100.0f);
            float m4_br = std::clamp(base_throttle - att_outputs.x + att_outputs.y + att_outputs.z, 0.0f, 100.0f);

            uint32_t pwm_m1 = flight->throttle_to_pwm(m1_fr);
            uint32_t pwm_m2 = flight->throttle_to_pwm(m2_bl);
            uint32_t pwm_m3 = flight->throttle_to_pwm(m3_fl);
            uint32_t pwm_m4 = flight->throttle_to_pwm(m4_br);

            Driver::Motor::get_instance().update_compare_value(pwm_m1, pwm_m2, pwm_m3, pwm_m4);
        }else{
            Driver::Motor::get_instance().update_compare_value(1000, 1000, 1000, 1000); // 비활성화 시 최소값으로 안전하게 유지
            pid.reset(); // 모터가 꺼질 때 PID 적분 항 초기화로 급격한 재가동 방지
        }
        
        
        
        // uint64_t loop_duration = esp_timer_get_time() - start_time;
        // if(++loop_cnt >= 20) { // 20ms마다 로그 출력
        //     loop_cnt = 0;
        //     ESP_LOGI(TAG, "Loop Duration: %10llu us, Motor PWM -> M1: %4d, M2: %4d, M3: %4d, M4: %4d",loop_duration, pwm_m1, pwm_m2, pwm_m3, pwm_m4);
        // }
        
        
        // -------------------------------------------------------------
        // [확장 마감] 자세 및 수직 상태 데이터 QGC 게시 연동 (10Hz)
        // -------------------------------------------------------------
        static uint16_t qgc_publish_count = 0;
        if (++qgc_publish_count >= 100) { 
            qgc_publish_count = 0;                
            QgcAttitude_t qgcAtt;
            
            qgcAtt.att              = curAttitude;                      // 진북 보정 완료된 오일러각
            qgcAtt.speed            = cur_imu_data.gyro * DEG_TO_RAD;   // 각속도 라디안
            qgcAtt.base_throttle    = base_throttle;

            qgcAtt.alt              = current_alt; 
            qgcAtt.v_speed          = current_vel; 
            sharedData.publish_data<Data_type::DT_QGC_ATTITUDE>(qgcAtt);
        }

        // [출력 가독성 최적화] UART 병목 및 로깅 오버헤드를 막기 위한 50Hz(20ms) 주기 필터링 로그
        // if (++loop_cnt >= 20) { 
        //     loop_cnt = 0;
        //     ESP_LOGI(TAG, "|AX: %8.5f |AY: %8.5f |AZ: %8.5f | GX: %8.5f |GY: %8.5f |GZ: %8.5f | MX: %8.5f |MY: %8.5f |MZ: %8.5f |R: %8.5f |P: %8.5f |Y: %8.5f|", 
        //             cur_imu_data.acc.x,   cur_imu_data.acc.y,   cur_imu_data.acc.z,
        //             cur_imu_data.gyro.x *DEG_TO_RAD,  cur_imu_data.gyro.y *DEG_TO_RAD,  cur_imu_data.gyro.z * DEG_TO_RAD,
        //             cur_imu_data.mag.x,   cur_imu_data.mag.y,   cur_imu_data.mag.z,
        //             curAttitude.roll,        curAttitude.pitch,       curAttitude.yaw 
        //         );
        // }

        gpio_set_level(DEBUG_GPIO_PIN, 0); // 루프 끝: 핀을 Low로!
        taskYIELD(); 
        //vTaskDelay(0); 
    }
}

/**
 * @brief 
 *      1. FlightTask을 생성하여 Core 1에 전적 격리 구동
 *     2. 최상위 우선순위(configMAX_PRIORITIES - 1)로 가용한 최고 권력을 할당하여 안정적인 실시간 제어 보장
 *     3. TaskFunction_t 형식의 함수 포인터로 flight_task를 지정하여 태스크의 실행 루틴으로 설정
 *     4. 태스크 이름은 "flight_task"로 지정하여 디버깅 및 모니터링 시 식별 용이
 *     5. 스택 크기는 8192 바이트로 충분히 할당
 * @return esp_err_t 
 */
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

/**
 * @brief 
 *      주어진 자기장 벡터를 지정된 각도만큼 회전시킵니다.
 * @param raw_mag 
 * @param rot_rad 
 * @return Vector3f 
 */
Vector3f Flight::rotateMagVector(const Vector3f& raw_mag, float rot_rad) {
    Vector3f corrected_mag{0.0f, 0.0f, 0.0f};

    float cos_ang = std::cos(rot_rad);
    float sin_ang = std::sin(rot_rad);

    corrected_mag.x = raw_mag.x * cos_ang - raw_mag.y * sin_ang;
    corrected_mag.y = raw_mag.x * sin_ang + raw_mag.y * cos_ang;
    corrected_mag.z = raw_mag.z; // Z축 성분(수직 자기장 분력)은 회전에 무관하므로 그대로 보존

    return corrected_mag;
}




} // namespace Controller
