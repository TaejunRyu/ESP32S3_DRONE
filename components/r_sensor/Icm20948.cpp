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
    Vector3f acc,gyro;
    select_bank(0);

    // [구분 처리] I2C vs SPI
    if (_ibus->get_type() == Interface::BusType::I2C) {
        // I2C일 때는 기존처럼 Accel + Gyro (12바이트)만 읽음
        uint8_t d[12];
        err = _ibus->Read(B0_ACCEL_XOUT_H, d, 12);
        if (err != ESP_OK) return err;

        acc.x = (int16_t)((d[0] << 8) | d[1]) / ACCEL_SENSITIVITY;
        acc.y = (int16_t)((d[2] << 8) | d[3]) / ACCEL_SENSITIVITY;
        acc.z = (int16_t)((d[4] << 8) | d[5]) / ACCEL_SENSITIVITY;

        gyro.x = (int16_t)((d[6] << 8) | d[7])  / GYRO_SENSITIVITY;
        gyro.y = (int16_t)((d[8] << 8) | d[9])  / GYRO_SENSITIVITY;
        gyro.z = (int16_t)((d[10] << 8) | d[11])/ GYRO_SENSITIVITY;
        raw.acc  = acc;
        raw.gyro = gyro;
        raw.timestamp = esp_timer_get_time();
    } 
    else {
        // SPI일 때는 Mag 포함 (21바이트) 읽음 (Accel 6 + Gyro 6 + Temp 2 + Mag 7)
        // 0x2D(Accel_X) ~ 0x41(Mag_Status2)까지 연속 읽기
        uint8_t d[21];
        err = _ibus->Read(B0_ACCEL_XOUT_H, d, 21);
        if (err != ESP_OK) return err;

        // Accel & Gyro 변환 (기존과 동일)
        acc.x = (int16_t)((d[0] << 8) | d[1]) / 4096.0f;
        acc.y = (int16_t)((d[2] << 8) | d[3]) / 4096.0f;
        acc.z = (int16_t)((d[4] << 8) | d[5]) / 4096.0f;

        gyro.x = (int16_t)((d[6] << 8) | d[7]) / 32.8f;
        gyro.y = (int16_t)((d[8] << 8) | d[9]) / 32.8f;
        gyro.z = (int16_t)((d[10] << 8) | d[11]) / 32.8f;

        raw.acc  = acc;
        raw.gyro = gyro;
        raw.timestamp = esp_timer_get_time();

        // 지자계 데이터 변환 (d[14]부터 Mag 데이터 시작)
        // AK09916 데이터 포맷: Little-Endian 방식 주의
        // d[14]: ST1, d[15]: HXL, d[16]: HXH, ... d[20]: ST2
        _mag.x = (int16_t)((d[16] << 8) | d[15]) * 0.15f; // uT 단위 변환
        _mag.y = (int16_t)((d[18] << 8) | d[17]) * 0.15f;
        _mag.z = (int16_t)((d[20] << 8) | d[19]) * 0.15f;
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
    if (_ibus == nullptr) return ESP_ERR_INVALID_STATE;        
    ImuData data;
    esp_err_t err = read_data(data);
    if (err == ESP_OK){
        // 주가되어질 항목 
        // 1. ENU방식으로 처리할것인가 NED방식으로 처리 할것인가에 따라 부호결정
        // 2. ENU로 갈 경우 KALMAN/ PID / QGC에서 처리방향 정리 할것
        // 3. NED로 갈 경우 KALMAN/ PID / QGC에서 처리방향 정리 할것
        // 4. 방향 설정되면 부호를 여기서 처리 할거라면 ENU 방식과 NED방식의 변경 함수를 만들어서 처리 할것.
        sample.acc = data.acc;
        sample.gyro = data.gyro;
        sample.temperature = data.temperature;
        sample.timestamp = data.timestamp;
    }
    return err;
}

void ICM20948::convertToCoordinate(ImuData &src, CoordSystem target)
{
    if (target == CoordSystem::NED) {
        // 이미 칩 자체가 북-동-아래(NED) 기준으로 안착되어 있다면 유지
        // 칩 장착 방향에 따라 축 반전이 필요하면 여기서 일괄 처리
        // 예시: 보드가 뒤집혀 장착된 경우 src.acc.z = -src.acc.z;
    } else if (target == CoordSystem::ENU) {
        // ENU 변환 공식 적용 (X -> Y, Y -> X, Z -> -Z 변환 등 적용)
        float temp_x = src.acc.x;
        src.acc.x = src.acc.y;     // East
        src.acc.y = temp_x;        // North
        src.acc.z = -src.acc.z;    // Up
        
        // 자이로도 동일하게 회전 방향 축 매핑 변경
        float temp_gx = src.gyro.x;
        src.gyro.x = src.gyro.y;
        src.gyro.y = temp_gx;
        src.gyro.z = -src.gyro.z;
    }
}

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