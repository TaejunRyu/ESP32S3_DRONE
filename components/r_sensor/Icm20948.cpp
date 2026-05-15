#include "Icm20948.hpp"

#include <esp_log.h>
#include <rom/ets_sys.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include "BusInterface.hpp"

namespace Sensor{

ICM20948::~ICM20948()
{
    if (_ibus) {
        delete _ibus; // 이 순간 자식(SPIBus) 소멸자가 호출되며 하드웨어 자원(spi_bus_remove_device)까지 완전히 해제됩니다.
        _ibus = nullptr;
    }
}


esp_err_t ICM20948::init_bus(Interface::IBus *bus)
{
    if (!bus) return ESP_FAIL;

    if (_ibus) delete _ibus; // 기존에 할당된 버스가 있다면 해제
    _ibus = bus;            // 새로 생성된 I2CBus 또는 SPIBus 주입
    
    return ESP_OK;
}

void ICM20948::set_bus(Interface::IBus *bus)
{
    if (_ibus != nullptr) {
        delete _ibus; // 기존에 할당되어 있던 SPIBus 또는 I2CBus 객체 파괴
    }
    _ibus = bus; 
}

esp_err_t ICM20948::select_bank(uint8_t bank){
    esp_err_t err = ESP_FAIL;
    if (_ibus == nullptr) return err;
    err = _ibus->Write(REG_BANK_SEL,(uint8_t)(bank << 4));
    return err;
}


esp_err_t ICM20948::read_data(ImuData &raw){
    if (_ibus == nullptr) return ESP_ERR_INVALID_STATE;        
    
    esp_err_t err;
    select_bank(0);

    if (_ibus->get_type() == Interface::BusType::I2C) {
        // I2C 모드: 기본 12바이트 리딩 및 플래그 비활성화
        uint8_t d[12];
        err = _ibus->Read(B0_ACCEL_XOUT_H, d, 12);
        if (err != ESP_OK) return err;

        raw.acc.x = (int16_t)((d[0] << 8) | d[1]) / ACCEL_SENSITIVITY;
        raw.acc.y = (int16_t)((d[2] << 8) | d[3]) / ACCEL_SENSITIVITY;
        raw.acc.z = (int16_t)((d[4] << 8) | d[5]) / ACCEL_SENSITIVITY;

        raw.gyro.x = (int16_t)((d[6] << 8) | d[7])  / GYRO_SENSITIVITY;
        raw.gyro.y = (int16_t)((d[8] << 8) | d[9])  / GYRO_SENSITIVITY;
        raw.gyro.z = (int16_t)((d[10] << 8) | d[11]) / GYRO_SENSITIVITY;
        
        raw.timestamp = esp_timer_get_time();
        raw.is_mag_updated = false; // I2C 바이패스/마스터 설정을 별도로 부팅 시 완료하지 않았다면 무시
    } 
    else {
        // SPI 모드: 가속도6 + 자이로6 + 온도2 + 지자계데이터 패킷 연속 리딩 영역
        // AK09916의 ST1(Status1) 레지스터가 매핑된 버퍼 위치를 d[14]라고 가정 (하드웨어 매핑 스펙 기준)
        uint8_t d[22]; 
        err = _ibus->Read(B0_ACCEL_XOUT_H, d, 22);
        if (err != ESP_OK) return err;

        // 가속도 및 자이로 물리 변환
        raw.acc.x = (int16_t)((d[0] << 8) | d[1]) / ACCEL_SENSITIVITY;
        raw.acc.y = (int16_t)((d[2] << 8) | d[3]) / ACCEL_SENSITIVITY;
        raw.acc.z = (int16_t)((d[4] << 8) | d[5]) / ACCEL_SENSITIVITY;
        raw.gyro.x = (int16_t)((d[6] << 8) | d[7]) / GYRO_SENSITIVITY;
        raw.gyro.y = (int16_t)((d[8] << 8) | d[9]) / GYRO_SENSITIVITY;
        raw.gyro.z = (int16_t)((d[10] << 8) | d[11]) / GYRO_SENSITIVITY;
        raw.timestamp = esp_timer_get_time();

        // ★ [핵심 방어] 지자계 Data Ready(DRDY) 비트 검사
        uint8_t st1 = d[14]; // AK09916 ST1 레지스터 위치
        if (st1 & 0x01) {    // DRDY 비트가 1인 경우 = '하드웨어적으로 진짜 새 지자계가 확보됨'
            raw.is_mag_updated = true;
            raw.mag_timestamp = raw.timestamp;

            // 지자계 Little-Endian 패킷 파싱 (d[15]부터 마그네토미터 데이터 시작 가정)
            raw.mag.x = (int16_t)((d[16] << 8) | d[15]) * 0.15f; 
            raw.mag.y = (int16_t)((d[18] << 8) | d[17]) * 0.15f;
            raw.mag.z = (int16_t)((d[20] << 8) | d[19]) * 0.15f;
        } else {
            // 이번 고속 루프 주기에 새 지자계가 없으면 갱신 패스 알림
            raw.is_mag_updated = false;
        }
    }
    return err;
}

/**
 * @brief 
 * @param sample 
 * @return true 
 * @return false 
 */
esp_err_t ICM20948::updateSample(ImuData &sample)
{    
      // 1. 하드웨어 버스 연결 상태 방어적 체크
    if (_ibus == nullptr) return ESP_ERR_INVALID_STATE;        
    
    ImuData data = {}; // 임시 버퍼 초기화
    esp_err_t err = read_data(data); // 칩 레지스터 일괄 리딩 (내부에서 mag 데이터 및 플래그 갱신됨)
    
    // 2. 통신이 완벽하게 성공한 경우에만 상위 객체로 데이터 복사
    if (err == ESP_OK){
        // [기본 IMU 데이터 전달]
        sample.acc         = data.acc;
        sample.gyro        = data.gyro;
        sample.temperature = data.temperature;
        sample.timestamp   = data.timestamp;

        // ★ [핵심 추가] 새로 보완된 지자계(Mag) 관련 데이터 체인 연결
        sample.mag            = data.mag;
        sample.mag_timestamp  = data.mag_timestamp;
        sample.is_mag_updated = data.is_mag_updated; // 느린 지자계 지터 방어 플래그 토스
    }
    
    return err;
}

// // 메인 제어 루프 예시
// ImuData current_drone_data;
// if (icm20948.updateSample(current_drone_data) == ESP_OK) {
//     // 1. 필터로 노이즈와 오차 제거
//     calibrator.apply_filter(current_drone_data); 

//     // 2. 상위 연산이 원하는 좌표계(예: ENU)로 통일 변환
//     Utils::FrameTransformer::convert_to(current_drone_data, Utils::TargetCoord::ENU);

//     // 3. 지자계 갱신 플래그(is_mag_updated)를 보고 칼만 필터의 보정 연산 여부 안전하게 분기 제어!
// }



esp_err_t ICM20948::initialize()
{
    if (_initialized) {
        ESP_LOGI(TAG, "Already initialized.");
        return ESP_OK;
    }

    // [중요] _bus가 외부에서 주입(set_bus)되었는지 먼저 확인합니다.
    if (_ibus == nullptr) {
        ESP_LOGE(TAG, "Bus interface not set!");
        return ESP_FAIL;
    }

    esp_err_t err;

    // 0. 초기 뱅크 설정
    select_bank(0);

    // 1. Soft Reset
    // 기존: i2c_master_transmit(_dev_handle, reset_cmd, 2, ...)
    // 변경: 인터페이스의 write(레지스터, 데이터) 사용
    err = _ibus->Write(B0_PWR_MGMT_1, 0x80); 
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Soft Reset setting failed");
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    // 2. Sleep 해제 및 Auto Clock 선택
    err = _ibus->Write(B0_PWR_MGMT_1, 0x01);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Sleep & Auto Clock setting failed");
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    // --- [Bank 2로 이동] ---
    select_bank(2);
    vTaskDelay(pdMS_TO_TICKS(10));

    // --- [자이로스코프 설정] ---
    err = _ibus->Write(B2_GYRO_CONFIG_1, 0x1D);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Gyro DLPF setting failed");
        return err;
    }

    // --- [가속도계 설정] ---
    err = _ibus->Write(B2_ACCEL_CONFIG, 0x1D);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Accel DLPF setting failed");
        return err;
    }

    // --- [Bank 0로 복귀] ---
    select_bank(0);
    vTaskDelay(pdMS_TO_TICKS(10));

    _initialized = true;
    ESP_LOGI(TAG, "Initialized successfully via Interface.");
    return ESP_OK;
}

esp_err_t ICM20948::deinitialize(){
    if (!_initialized) {
        return ESP_OK;
    }
    _ibus = nullptr; 
    _initialized = false;
    ESP_LOGI(TAG, "Deinitialized successfully (Interface detached).");
    return ESP_OK;
}

} //namespace Sensor