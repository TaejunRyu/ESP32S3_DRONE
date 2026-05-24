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
    
    auto& v_kalman = Filter::VerticalFilter::getInstance();

    auto& pid = PidControl::getInstance();


    // 2. [자세 게인 튜닝 파라미터 주입] 구조: {Kp, Ki, Kd, I_Limit, Out_Limit}
    Controller::PidParams att_angle = {2.5f, 0.0f, 0.0f, 0.0f, 10.0f};  // 바깥 각도 루프
    Controller::PidParams att_rate  = {0.08f, 0.02f, 0.001f, 0.5f, 40.0f}; // 안쪽 각속도 루프
    pid.setAngleParams(att_angle, att_angle, att_angle);
    pid.setRateParams(att_rate, att_rate, att_rate);


    // 3. [고도 게인 튜닝 파라미터 독립 주입]
    Controller::AltitudeParams alt_config;
    alt_config.kp_alt         = 1.2f;   // 고도 -> 속도 변환율
    alt_config.kp_vel         = 1.8f;   // 속도 P
    alt_config.ki_vel         = 0.4f;   // 속도 I
    alt_config.kd_vel         = 0.01f;  // 속도 D
    alt_config.vel_limit      = 2.0f;   // 최대 2m/s
    alt_config.hover_throttle = 43.5f;  // 이 기체의 공중 유지 호버링 스로틀 추정치 43.5%
    alt_config.out_limit      = 85.0f;  // 최대 출력 한계 제한
    pid.setAltitudeParams(alt_config);

  
    // 2. PID 파라미터 초기화 (구동 대상 하드웨어에 맞는 튜닝 파라미터 입력 필요)
    // 값 형식: {Kp, Ki, Kd, I_Limit, Output_Limit}
    Controller::PidParams angle_gain = {2.0f, 0.1f, 0.05f, 0.5f, 10.0f}; // 바깥 루프 (목표 각속도 rad/s 한계)
    Controller::PidParams rate_gain  = {1.0f, 0.05f, 0.001f, 1.0f, 50.0f}; // 안쪽 루프 (최종 모터 출력 한계)

    pid.setAngleParams(angle_gain, angle_gain, angle_gain); // Roll, Pitch, Yaw 동일 적용 예시
    pid.setRateParams(rate_gain, rate_gain, rate_gain);
    pid.reset();


    // 가상 목표 및 센서 데이터 선언
    Attitude_t target_pose = {0.0f, 0.0f, 0.0f}; // 정밀 호버링 (평평한 상태) 목표

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
            if(cur_imu_data.is_mag_updated){
                cur_imu_data.mag.normalize();
            }

            //IST8310 MagSensorTask에서 보내온 데이터를 받는다.
            { // 이블럭을 제거하면 ak09916으로 mag가 대체되어진다.
                cur_imu_data.mag = 0.0f;
                if(sharedData.is_mag_updated()){  // 업데이트 될때만 받아와서 적용한다.
                    cur_mag_data = sharedData.get_shared_data< Data_type::DT_MAG_DATA>();
                    cur_imu_data.mag = cur_mag_data;   // 지자계를 대체한다.
                    cur_imu_data.mag.normalize();        
                }
            }
            // [EKF 핵심 엔진 가동] 자이로 예측 후 가속도/지자계 순차 보정 처리
            kalman.update(  cur_imu_data.acc,
                            cur_imu_data.gyro * DEG_TO_RAD, //// 입력 데이터 가공 (입력이 도/초 단위일 경우 예측부 라디안 스케일링 일치 처리)
                            cur_imu_data.mag,
                            dt);
            
            // 진북 기준 최종 오일러 각 추출 (라디안 단위)
            Attitude_t curAttitude = kalman.getEuler();

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
            curAttitude.yaw = curAttitude.yaw - target_true_north;

            // 4. NED 좌표계 표준 경계선 처리 (-PI ~ +PI) 필수 수행
            while (curAttitude.yaw > M_PI)  curAttitude.yaw -= 2.0f * M_PI;
            while (curAttitude.yaw < -M_PI) curAttitude.yaw += 2.0f * M_PI;


            float q_buffer[4] = {1.0f, 0.0f, 0.0f, 0.0f};
            
            // 1. [핵심] 기존 자세 EKF로부터 실시간 최신 쿼터니언 상태 변수 취득
            // (이 값이 실시간 기체의 롤, 피치 기울임 정보를 온전히 담고 있습니다.)
            kalman.getQuaternion(q_buffer);
            float q0 = q_buffer[0]; 
            float q1 = q_buffer[1]; 
            float q2 = q_buffer[2]; 
            float q3 = q_buffer[3];


            Vector3f acc;
            // 💡 센서 원시 데이터(m/s^2)를 9.81로 나누어 단위를 G 규격(정지 시 1.0)으로 가공
            acc.x = cur_imu_data.acc.x / 9.80665f;
            acc.y = cur_imu_data.acc.y / 9.80665f;
            acc.z = cur_imu_data.acc.z / 9.80665f;

            // 3. 취득한 기존 EKF 쿼터니언을 이용하여 체프 가속도를 지구 수직 방향(Z축 Down)으로 회전 투영
            float acc_z_earth = 2.0f * (q1*q3 - q0*q2) * acc.x + 
                                2.0f * (q0*q1 + q2*q3) * acc.y + 
                                (q0*q0 - q1*q1 - q2*q2 + q3*q3) * acc.z;

            // 4. 중력 성분(1.0G 또는 9.81m/s^2)을 차감하여 순수 운동 상승/하강 가속도만 분리
            // (가만히 수평 호버링 중일 때는 이 값이 정확히 0.0f 에 가깝게 홀딩되어야 성공입니다.)
            float pure_vertical_accel = acc_z_earth - 1.0f; ; // 가속도 단위 규격이 G(Gravity)인 경우

            // 5. 1ms 주기로 수직 칼만필터 시간 예측 단계 실행
            v_kalman.predict(pure_vertical_accel, dt);            

            //[목표 고도] ➔ Outer Loop (고도 P 제어) ➔ [목표 상승/하강 속도] ➔ Inner Loop (속도 PID) ➔ [최종 Throttle]
            BaroData baroData{};
            if(sharedData.is_baro_updated()){ //40ms단위로 데이터가 들어온다.
                baroData =  sharedData.get_shared_data<Data_type::DT_BARO_DATA>();
                v_kalman.update(baroData.altitude); // 40ms 주기 보정
                ESP_LOGI(TAG, "Baro -> gnd_pressure: %5.2f, pressure: %5.2f, altitude: %5.2f", 
                            baroData.gnd_pressure ,baroData.pressure,baroData.altitude);
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

            // 자이로 데이터는 진동이 발생할경우( 필터 제공 )
            static Vector3f filtered_rate = {0.0f, 0.0f, 0.0f};
            const float alpha = 0.3f; 
            filtered_rate.x = alpha * cur_imu_data.gyro.x * DEG_TO_RAD + (1.0f - alpha) * filtered_rate.x;
            filtered_rate.y = alpha * cur_imu_data.gyro.y * DEG_TO_RAD + (1.0f - alpha) * filtered_rate.y;
            filtered_rate.z = alpha * cur_imu_data.gyro.z * DEG_TO_RAD + (1.0f - alpha) * filtered_rate.z;

            // -------------------------------------------------------------
            // 핵심 연산: 자세와 고도 제어 명령을 병렬 독립 연산 처리
            // -------------------------------------------------------------
            float target_altitude  = 1.5f;               // 1.5m 고도 홀딩 명령
            Vector3f att_outputs = pid.updateCascade(target_pose, curAttitude, filtered_rate, dt);
            float base_throttle  = pid.updateAltitudeCascade(target_altitude, current_alt, current_vel, dt);
            
            // -------------------------------------------------------------
            // 최종 믹싱 단계: 고도 스로틀(Base)에 자세 복원력을 축별 가감산 (Quadcopter X-Type / NED 기준)
            // -------------------------------------------------------------
            float m1_fr = base_throttle - att_outputs.x - att_outputs.y - att_outputs.z; // 전방 우측
            float m2_bl = base_throttle + att_outputs.x + att_outputs.y - att_outputs.z; // 후방 좌측
            float m3_fl = base_throttle + att_outputs.x - att_outputs.y + att_outputs.z; // 전방 좌측
            float m4_br = base_throttle - att_outputs.x + att_outputs.y + att_outputs.z; // 후방 우측

            
            // [선택지 2] 만약 일반 표준 PWM 변속기(ESC)를 사용하시는 경우 (출력 범위: 1000us ~ 2000us)
            uint32_t pwm_m1 = (uint32_t)(1000.0f + (m1_fr / 100.0f) * 1000.0f);
            uint32_t pwm_m2 = (uint32_t)(1000.0f + (m2_bl / 100.0f) * 1000.0f);
            uint32_t pwm_m3 = (uint32_t)(1000.0f + (m3_fl / 100.0f) * 1000.0f);
            uint32_t pwm_m4 = (uint32_t)(1000.0f + (m4_br / 100.0f) * 1000.0f);


            // 안전 가이드 한계값 구속 (1000us 미만이나 2000us 초과 방어)
            if (pwm_m1 > 2000) pwm_m1 = 2000; 
            if (pwm_m1 < 1000) pwm_m1 = 1000;
            if (pwm_m2 > 2000) pwm_m2 = 2000; 
            if (pwm_m2 < 1000) pwm_m2 = 1000;
            if (pwm_m3 > 2000) pwm_m3 = 2000; 
            if (pwm_m3 < 1000) pwm_m3 = 1000;
            if (pwm_m4 > 2000) pwm_m4 = 2000; 
            if (pwm_m4 < 1000) pwm_m4 = 1000;


            // 실제 ESP32-S3 MCPWM 이나 LEDC 드라이버 채널에 고속 펄스 폭 업데이트
            // mcpwm_set_duty_in_us(..., pwm_m1);



            // -------------------------------------------------------------
            // [확장 마감] 자세 및 수직 상태 데이터 QGC 게시 연동 (10Hz)
            // -------------------------------------------------------------
            static uint16_t qgc_publish_count = 0;
            if (++qgc_publish_count >= 100) { 
                qgc_publish_count = 0;
                
                QgcAttitude_t qgcAtt;
                qgcAtt.att      = curAttitude;                  // 진북 보정 완료된 오일러각
                qgcAtt.speed    = cur_imu_data.gyro * DEG_TO_RAD; // 각속도 라디안
                
                // 💡 수직 칼만 필터가 계산한 정밀 고도와 속도를 10Hz 시퀀스에 결합합니다.
                qgcAtt.base_throttle = base_throttle;
                qgcAtt.alt      = current_alt; 
                qgcAtt.v_speed  = current_vel; 

                // 공유 메모리 매니저에 안전하게 배포 (락프리 더블 버퍼링 작동)
                sharedData.publish_data<Data_type::DT_QGC_ATTITUDE>(qgcAtt);
            }




            
            // NED 좌표 출력축 계산 결과 로그 출력
            // if (++loop_cnt >= 20) { 
            //     loop_cnt = 0;
            //     ESP_LOGI(TAG, "PID Output -> Roll Out: %5.2f, Pitch Out: %5.2f, Yaw Out: %5.2f", 
            //             motor_outputs.x, motor_outputs.y, motor_outputs.z);
            // }

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
