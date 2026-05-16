#include "Icm20948.hpp"

#include <esp_log.h>
#include <rom/ets_sys.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include "BusInterface.hpp"
#include "DroneConfig.hpp"


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

/**
 * @brief 
 *      1. SPI 드라이버를 사용할때에도 내부적으로 I2C로 통신을 한다.
 *      2. ICM20948이 I2C MASTER가 되어서 AK09916의 데이터를 자동으로 ICM20948의 특정 주소값을 읽는다.
 * @return esp_err_t 
 */
esp_err_t ICM20948::enable_mag_bypass()
{
    esp_err_t err = ESP_FAIL;
    if (_ibus == nullptr){ 
        return err;
    }
    if (_ibus->get_type() == Interface::BusType::I2C) {
        // --- [I2C 모드: 기존 Bypass 로직] ---
        select_bank(0);
        _ibus->Write(B0_USER_CTRL, 0x00);    // I2C Master Off
        _ibus->Write(B0_INT_PIN_CFG, 0x02); // Bypass On
        ESP_LOGI(TAG, "Bypass Mode Enabled");
    } 
    else if (_ibus->get_type() == Interface::BusType::SPI) {
        select_bank(0);
        _ibus->Write(B0_USER_CTRL, 0x20);    // I2C Master On        
        select_bank(3);
        _ibus->Write(B3_I2C_MST_CTRL, 0x07); // 400kHz
        _ibus->Write(B3_I2C_SLV0_ADDR, 0x0C | 0x80); // Mag Read
        _ibus->Write(B3_I2C_SLV0_REG, 0x11);
        _ibus->Write(B3_I2C_SLV0_CTRL, 0x89);
        ESP_LOGI(TAG, "SPI Internal I2C Master Enabled for Mag");
    }
    return ESP_OK;
}

esp_err_t ICM20948::select_bank(uint8_t bank)
{
    esp_err_t err = ESP_FAIL;
    if (_ibus == nullptr) return err;
    err = _ibus->Write(REG_BANK_SEL,(uint8_t)(bank << 4));
    return err;
}

esp_err_t ICM20948::read_data(ImuData &raw){
    if (_ibus == nullptr) return ESP_ERR_INVALID_STATE;        
    
    esp_err_t err;
    select_bank(0);
    uint8_t d[22]; 

    if (_ibus->get_type() == Interface::BusType::I2C) {
        // I2C 모드: 기본 12바이트 리딩 및 플래그 비활성화
        err = _ibus->Read(B0_ACCEL_XOUT_H, d, 12);
        if (err != ESP_OK) return err;
        raw.timestamp = esp_timer_get_time();
        raw.is_mag_updated = false; // I2C 바이패스/마스터 설정을 별도로 부팅 시 완료하지 않았다면 무시

    } else {
        // SPI 모드: 가속도6 + 자이로6 + 온도2 + 지자계데이터 패킷 연속 리딩 영역
        // AK09916의 ST1(Status1) 레지스터가 매핑된 버퍼 위치를 d[14]라고 가정 (하드웨어 매핑 스펙 기준)
        err = _ibus->Read(B0_ACCEL_XOUT_H, d, 22);
        if (err != ESP_OK) return err;
        // ★ [핵심 방어] 지자계 Data Ready(DRDY) 비트 검사
        uint8_t st1 = d[14]; // AK09916 ST1 레지스터 위치
        if (st1 & 0x01) {    // DRDY 비트가 1인 경우 = '하드웨어적으로 진짜 새 지자계가 확보됨'
            raw.is_mag_updated = true;
            raw.mag_timestamp = raw.timestamp;

            // 지자계 Little-Endian 패킷 파싱 (d[15]부터 마그네토미터 데이터 시작 가정)
            raw.mag.x = (int16_t)((d[16] << 8) | d[15]) * MAG_SENSITIVITY; 
            raw.mag.y = (int16_t)((d[18] << 8) | d[17]) * MAG_SENSITIVITY;
            raw.mag.z = (int16_t)((d[20] << 8) | d[19]) * MAG_SENSITIVITY;
        } else {
            // 이번 고속 루프 주기에 새 지자계가 없으면 갱신 패스 알림
            raw.is_mag_updated = false;
        }
    }

    raw.acc.x = (int16_t)((d[0] << 8) | d[1]) / ACCEL_SENSITIVITY;
    raw.acc.y = (int16_t)((d[2] << 8) | d[3]) / ACCEL_SENSITIVITY;
    raw.acc.z = (int16_t)((d[4] << 8) | d[5]) / ACCEL_SENSITIVITY;

    raw.gyro.x = (int16_t)((d[6] << 8) | d[7])  / GYRO_SENSITIVITY;
    raw.gyro.y = (int16_t)((d[8] << 8) | d[9])  / GYRO_SENSITIVITY;
    raw.gyro.z = (int16_t)((d[10] << 8) | d[11]) / GYRO_SENSITIVITY;
    
    // if (_ibus->get_type() == Interface::BusType::I2C) {
    //     // I2C 모드: 기본 12바이트 리딩 및 플래그 비활성화
    //     uint8_t d[12];
    //     err = _ibus->Read(B0_ACCEL_XOUT_H, d, 12);
    //     if (err != ESP_OK) return err;

    //     raw.acc.x = (int16_t)((d[0] << 8) | d[1]) / ACCEL_SENSITIVITY;
    //     raw.acc.y = (int16_t)((d[2] << 8) | d[3]) / ACCEL_SENSITIVITY;
    //     raw.acc.z = (int16_t)((d[4] << 8) | d[5]) / ACCEL_SENSITIVITY;

    //     raw.gyro.x = (int16_t)((d[6] << 8) | d[7])  / GYRO_SENSITIVITY;
    //     raw.gyro.y = (int16_t)((d[8] << 8) | d[9])  / GYRO_SENSITIVITY;
    //     raw.gyro.z = (int16_t)((d[10] << 8) | d[11]) / GYRO_SENSITIVITY;
        
    //     raw.timestamp = esp_timer_get_time();
    //     raw.is_mag_updated = false; // I2C 바이패스/마스터 설정을 별도로 부팅 시 완료하지 않았다면 무시
    // } 
    // else {
    //     // SPI 모드: 가속도6 + 자이로6 + 온도2 + 지자계데이터 패킷 연속 리딩 영역
    //     // AK09916의 ST1(Status1) 레지스터가 매핑된 버퍼 위치를 d[14]라고 가정 (하드웨어 매핑 스펙 기준)
    //     uint8_t d[22]; 
    //     err = _ibus->Read(B0_ACCEL_XOUT_H, d, 22);
    //     if (err != ESP_OK) return err;

    //     // 가속도 및 자이로 물리 변환
    //     raw.acc.x = (int16_t)((d[0] << 8) | d[1]) / ACCEL_SENSITIVITY;
    //     raw.acc.y = (int16_t)((d[2] << 8) | d[3]) / ACCEL_SENSITIVITY;
    //     raw.acc.z = (int16_t)((d[4] << 8) | d[5]) / ACCEL_SENSITIVITY;
    //     raw.gyro.x = (int16_t)((d[6] << 8) | d[7]) / GYRO_SENSITIVITY;
    //     raw.gyro.y = (int16_t)((d[8] << 8) | d[9]) / GYRO_SENSITIVITY;
    //     raw.gyro.z = (int16_t)((d[10] << 8) | d[11]) / GYRO_SENSITIVITY;
    //     raw.timestamp = esp_timer_get_time();

    //     // ★ [핵심 방어] 지자계 Data Ready(DRDY) 비트 검사
    //     uint8_t st1 = d[14]; // AK09916 ST1 레지스터 위치
    //     if (st1 & 0x01) {    // DRDY 비트가 1인 경우 = '하드웨어적으로 진짜 새 지자계가 확보됨'
    //         raw.is_mag_updated = true;
    //         raw.mag_timestamp = raw.timestamp;

    //         // 지자계 Little-Endian 패킷 파싱 (d[15]부터 마그네토미터 데이터 시작 가정)
    //         raw.mag.x = (int16_t)((d[16] << 8) | d[15]) * 0.15f; 
    //         raw.mag.y = (int16_t)((d[18] << 8) | d[17]) * 0.15f;
    //         raw.mag.z = (int16_t)((d[20] << 8) | d[19]) * 0.15f;
    //     } else {
    //         // 이번 고속 루프 주기에 새 지자계가 없으면 갱신 패스 알림
    //         raw.is_mag_updated = false;
    //     }
    // }
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
        sample.acc         = data.acc   - _acc_offset;
        sample.gyro        = data.gyro  - _gyro_offset;
        sample.temperature = data.temperature;
        sample.timestamp   = data.timestamp;

        // ★ [핵심 추가] 새로 보완된 지자계(Mag) 관련 데이터 체인 연결
        sample.mag    = (data.mag -_mag_offset) * _mag_scale;
        sample.mag_timestamp  = data.mag_timestamp;
        sample.is_mag_updated = data.is_mag_updated; // 느린 지자계 지터 방어 플래그 토스

        //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
        // 이건 다시 ENU로 할것인가 아니면 NED로 할것인가 선택문제.
        sample.acc.y *= -1.0f;  
        sample.gyro.x *= -1.0f;
        sample.gyro.z *= -1.0f; // gz: 오른쪽회전시 (+)로 증가

        // 이건 다시 ENU로 할것인가 아니면 NED로 할것인가 선택문제.
        sample.mag.x *=  -1.0f;
        sample.mag.y *=  -1.0f;
        //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    }
    return err;
}

esp_err_t ICM20948::do_calibration()
{
    esp_err_t err = ESP_OK;
    if (_calibration){ 
        ESP_LOGW(TAG, "Already Calibration was performed.");
        return err;
    }
    // 인터페이스 연결 확인
    if (_ibus == nullptr) {
        ESP_LOGE(TAG, "Bus interface not set!");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Zeroing... Keep the aircraft level.");

    Vector3f sum_acc{},sum_gyro{};
    int valid_samples = 0;
    const int samples = 500;
    uint8_t buf[12];
    
    select_bank(0);

    for (int i = 0; i < samples; i++) {
        uint64_t start_time = esp_timer_get_time();
        // [변경] i2c_master_transmit_receive 대신 인터페이스의 read 사용
        if (_ibus->Read(B0_ACCEL_XOUT_H, buf, 12) == ESP_OK) [[likely]] {
            sum_acc.x  += (int16_t)((buf[0] << 8)  | buf[1])  / ACCEL_SENSITIVITY;
            sum_acc.y  += (int16_t)((buf[2] << 8)  | buf[3])  / ACCEL_SENSITIVITY;
            sum_acc.z  += (int16_t)((buf[4] << 8)  | buf[5])  / ACCEL_SENSITIVITY;
            sum_gyro.x += (int16_t)((buf[6] << 8)  | buf[7])  / GYRO_SENSITIVITY;
            sum_gyro.y += (int16_t)((buf[8] << 8)  | buf[9])  / GYRO_SENSITIVITY;
            sum_gyro.z += (int16_t)((buf[10] << 8) | buf[11]) / GYRO_SENSITIVITY;
            valid_samples++;
        }
        // 주기 제어 (LOOP_TIME에 의해서 결정되어짐)
        uint64_t end_time = esp_timer_get_time();
        uint32_t elapsed = (uint32_t)(end_time - start_time);
        if (elapsed < LOOP_TIME) {
            esp_rom_delay_us(LOOP_TIME - elapsed);
        }
    }

    // (이하 평균값 계산 및 오프셋 저장 로직은 기존과 동일) ...
    if (valid_samples > 0) [[likely]] {
        _acc_offset     = sum_acc / valid_samples;
        _acc_offset.z  -= 1.0f;                     //중력을 뺀다. offetset이기 때문에 -1을 해준다.
        _gyro_offset    = sum_gyro / valid_samples;

        _calibration = true;
        ESP_LOGI(TAG, "Zeroing Completed.");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Calibration Failed (No valid samples).");
        return ESP_FAIL;
    }
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