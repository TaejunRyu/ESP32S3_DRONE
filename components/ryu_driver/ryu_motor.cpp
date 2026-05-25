#include "ryu_motor.hpp"

#include <cmath>
#include <esp_log.h>
//#include "ryu_config.h"

namespace Driver
{

esp_err_t Motor::initialize()
{
    if (_initialized) return ESP_OK;

    // 1. MCPWM 타이머 설정 (예: 50Hz - 서보모터용)
    // 해상도: 1MHz로 설정하여 1틱(tick)당 1마이크로초(1us)가 걸리게 만듭니다.
    // 주기: 20,000틱으로 설정했으므로, 전체 주기는 20ms(50Hz)가 됩니다. 이는 일반적인 서보모터 제어 표준 규격입니다.
    mcpwm_timer_config_t t_cfg = {};
    t_cfg.group_id = 0;                             // 1번째
    t_cfg.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;    // 2번째
    t_cfg.resolution_hz = 1000000;                  // 3번째
    t_cfg.count_mode = MCPWM_TIMER_COUNT_MODE_UP;   // 4번째 (기존 코드에서 순서 바뀜)
    t_cfg.period_ticks = 20000;                     // 5번째
    t_cfg.intr_priority = 0;                        // (생략 가능하나 순서상 여기)

    esp_err_t err = mcpwm_new_timer(&t_cfg, &_timer);
    if(err != ESP_OK){
        return err;
    }

    // 2. 오퍼레이터 생성
    // 오퍼레이터는 타이머를 받아 실제 PWM 파형을 어떻게 요리할지 결정하는 단위입니다. 
    // 여기서는 2개를 만들어 각각의 오퍼레이터가 2개씩, 총 4개의 출력(제너레이터)을 관리하게 됩니다.
    mcpwm_oper_handle_t opers[3];
    mcpwm_operator_config_t o_cfg = {};
    o_cfg.group_id = 0;

    for (int i = 0; i < 3; i++) {
        err = mcpwm_new_operator(&o_cfg, &opers[i]);
        if(err != ESP_OK){
            return err;
        }

        err = mcpwm_operator_connect_timer(opers[i], _timer);
        if(err != ESP_OK){
            return err;
        }
    }

    // 제너레이터: MOTOR_PINS[i]에 설정된 실제 하드웨어 핀으로 신호를 내보내는 역할을 합니다.
    // 컴퍼레이터: "언제 신호를 끄고 켤지" 결정하는 기준값을 가집니다. comparators[i] 배열에 저장하여 나중에 속도나 각도를 조절할 때 이 값을 변경합니다.
    for (int i = 0; i < 4; i++) {
        // 3. 제너레이터 생성 (실제 출력 핀 지정)
        mcpwm_gen_handle_t gen = NULL; 
        mcpwm_generator_config_t g_cfg ={};
        g_cfg.gen_gpio_num = MOTOR_PINS[i];

        err = mcpwm_new_generator(opers[i/2], &g_cfg, &gen);
        if(err != ESP_OK){
            return err;
        }

        // 4    . 컴퍼레이터 생성 (듀티 조절용)
        mcpwm_comparator_config_t cmpr_config = {};
        cmpr_config.flags.update_cmp_on_tez = true;

        err = mcpwm_new_comparator(opers[i/2], &cmpr_config, &_comparators[i]);
        if(err != ESP_OK){
            return err;
        }

        // 5. 신호 발생 규칙 설정 (비교값에 따라 High/Low 전환) 
        //    이 부분이 PWM 파형의 모양을 결정합니다.
        err = mcpwm_generator_set_action_on_timer_event(gen, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
        if(err != ESP_OK){
            return err;
        }

        err = mcpwm_generator_set_action_on_compare_event(gen, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, _comparators[i], MCPWM_GEN_ACTION_LOW));
        if(err != ESP_OK){
            return err;
        }

        // 6. 듀티 사이클 설정 및 시작
        // 초기 비교값을 1000으로 잡았습니다. 전체 20,000 중 1,000이므로 5% 듀티 사이클(1ms) 신호가 나갑니다. 
        // 보통 서보모터에서 1ms~1.5ms는 정지 또는 특정 각도를 의미합니다.
        err = mcpwm_comparator_set_compare_value(_comparators[i], 1000); // Stop
        if(err != ESP_OK){
            return err;
        }

    }

    // 3-서보. 제너레이터 생성 (25번 핀)

    mcpwm_generator_config_t s_g_cfg = {};
    s_g_cfg.gen_gpio_num = SERVO_MOTOR_PIN; // 낚시 투하용 핀
    
    // 기존 opers[1] (2번 오퍼레이터)의 남는 자리에 연결하거나 새 오퍼레이터 생성 가능
    err = mcpwm_new_generator(opers[2], &s_g_cfg, &_servo_gen); 
    if(err != ESP_OK){
        return err;
    }


    // 4-서보. 컴퍼레이터 생성
    mcpwm_comparator_config_t s_cmpr_cfg ={};
    s_cmpr_cfg.flags.update_cmp_on_tez = true;

    err = mcpwm_new_comparator(opers[2], &s_cmpr_cfg, &_servo_comparator);
    if(err != ESP_OK){
        return err;
    }


    // 5-서보. 신호 규칙 설정
    err = mcpwm_generator_set_action_on_timer_event(_servo_gen, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    if(err != ESP_OK){
        return err;
    }

    err = mcpwm_generator_set_action_on_compare_event(_servo_gen, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, _servo_comparator, MCPWM_GEN_ACTION_LOW));
    if(err != ESP_OK){
        return err;
    }


    // 6-서보. 초기 각도 설정 (1ms = 잠금 상태)
    err = mcpwm_comparator_set_compare_value(_servo_comparator, 1000); 
    if(err != ESP_OK){
        return err;
    }


    err = mcpwm_timer_enable(_timer);
    if(err != ESP_OK){
        return err;
    }

    err = mcpwm_timer_start_stop(_timer, MCPWM_TIMER_START_NO_STOP);
    if(err != ESP_OK){
        return err;
    }


    _initialized = true;
    ESP_LOGI(TAG,"Initialized successfully.");
    return err;
}

esp_err_t Motor::stop_all_motors()
{
    esp_err_t ret = ESP_OK;
    for(auto& comp : _comparators){ 
        ret = mcpwm_comparator_set_compare_value(comp, 1000);
        if (ret != ESP_OK){
            return ret;
        }
    }
    return ESP_OK;
}

void Motor::set_drop_angle(int angle)
{
        if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    
    // 500us(0도) ~ 2500us(180도) 매핑
    uint32_t duty_ticks = 500 + (angle * (2500 - 500) / 180);
    
    // 신형 API를 사용하여 듀티값 변경
    mcpwm_comparator_set_compare_value(_servo_comparator, duty_ticks);

}

void Motor::update_compare_value(const uint32_t m1, const uint32_t m2, const uint32_t m3, const uint32_t m4)
{
    mcpwm_comparator_set_compare_value(_comparators[0], static_cast<uint32_t>(std::round(m1)));
    mcpwm_comparator_set_compare_value(_comparators[1], static_cast<uint32_t>(std::round(m2)));
    mcpwm_comparator_set_compare_value(_comparators[2], static_cast<uint32_t>(std::round(m3)));
    mcpwm_comparator_set_compare_value(_comparators[3], static_cast<uint32_t>(std::round(m4)));
}

}// namespace Motor



// // void task_drop_mechanism() {
// //     // 1. 목표지점 도달 사운드 재생
// //     sound_mission_complete();

// //     // 2. 낚시 도구 투하 (각도는 기구 설계에 맞게 조정)
// //     set_servo_angle(150); 
    
// //     // 3. 완전히 빠질 때까지 대기
// //     vTaskDelay(pdMS_TO_TICKS(1500)); 

// //     // 4. 서보 암 원위치 (선택 사항)
// //     set_servo_angle(10);
// // }


// /*
// 1. 거리 계산 함수 (Haversine Formula)
// 지구의 곡률을 고려하여 두 좌표 사이의 직선 거리(미터)를 구합니다.
// */
// #define EARTH_RADIUS 6371000.0 // 지구 반지름 (미터)
// #define DEG_TO_RAD(deg) ((deg) * M_PI / 180.0)

// // 두 좌표 사이의 거리 계산 (단위: 미터)
// double calculate_distance(double lat1, double lon1, double lat2, double lon2) {
//     double dLat = DEG_TO_RAD(lat2 - lat1);
//     double dLon = DEG_TO_RAD(lon2 - lon1);

//     double a = sin(dLat / 2) * sin(dLat / 2) +
//                cos(DEG_TO_RAD(lat1)) * cos(DEG_TO_RAD(lat2)) *
//                sin(dLon / 2) * sin(dLon / 2);
//     double c = 2 * atan2(sqrt(a), sqrt(1 - a));

//     return EARTH_RADIUS * c;
// }


// /*
// 2. 도착 판단 및 투하 로직
// GPS 오차를 고려하여 목표 지점의 일정 반경(예: 2~3m) 안에 들어오면 도착으로 간주합니다.
// */

// // 목표 좌표 (예시: 서울 어딘가)
// const double TARGET_LAT = 37.123456;
// const double TARGET_LON = 127.123456;
// const double ARRIVAL_THRESHOLD = 3.0; // 도착 인정 범위 (3미터)
// bool has_dropped = false; // 중복 투하 방지 플래그
// // 설정값
// const float  MIN_DROP_ALT = 2.0;    // 투하 가능 최소 고도 (2m)
// const float  MAX_DROP_ALT = 5.0;    // 투하 가능 최대 고도 (5m)

// // GPS는 정지해 있어도 신호 잡음 때문에 위치가 조금씩 변합니다(Drift).
// // 너무 작으면 (예: 0.5m): 드론이 목표 지점 바로 위를 지나가더라도 GPS 오차 때문에 "도착하지 않았다"고 판단하여 투하를 안 할 수 있습니다.
// // 너무 크면 (예: 10.0m): 목표 지점에서 너무 멀리 떨어진 곳에서 미끼를 투하하게 됩니다.
// const double DIST_THRESHOLD = 3.0; // 목표 지점 반경 3미터 이내면 도착으로 간주



// void check_arrival_and_drop(double current_lat, double current_lon) {
//     if (has_dropped) return; // 이미 투하했다면 종료

//     double distance = calculate_distance(current_lat, current_lon, TARGET_LAT, TARGET_LON);

//     printf("목표까지 거리: %.2f m\n", distance);

//     if (distance <= ARRIVAL_THRESHOLD) {
//         printf("목표 도달! 투하를 시작합니다.\n");
        
//         Driver::Buzzer::get_instance().sound_mission_complete(); // 아까 만든 도착 사운드 재생
//         set_drop_angle(120);      // 낚시 도구 투하 (서보 회전)
        
//         has_dropped = true;       // 투하 완료 표시
//     }
// }





// /**
//  * @param cur_lat 현재 위도
//  * @param cur_lon 현재 경도
//  * @param cur_alt 현재 고도 (m) - 바로미터 또는 GPS 고도
//  */
// void check_safe_drop(double cur_lat, double cur_lon, float cur_alt) {
//     if (has_dropped) return;

//     // 1. 수평 거리 계산
//     double distance = calculate_distance(cur_lat, cur_lon, TARGET_LAT, TARGET_LON);

//     // 2. 투하 조건 검사 (거리 && 고도 범위)
//     if (distance <= DIST_THRESHOLD) {
//         if (cur_alt >= MIN_DROP_ALT && cur_alt <= MAX_DROP_ALT) {
//             printf("조건 충족: 거리 %.1fm, 고도 %.1fm. 투하!\n", distance, cur_alt);
            
//             set_drop_angle(120); // 서보 구동 및 사운드 함수
//             has_dropped = true;
//         } else {
//             // 거리는 맞지만 고도가 맞지 않을 때 알림 (디버깅용)
//             printf("위치는 일치하나 고도가 부적절함: %.1fm\n", cur_alt);
//         }
//     }
// }
