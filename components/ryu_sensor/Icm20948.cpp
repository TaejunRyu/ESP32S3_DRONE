#include "Icm20948.hpp"

#include <esp_log.h>
#include <rom/ets_sys.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include "ryu_BusInterface.hpp"
#include "ryu_DroneConfig.hpp"


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
    if (bus == nullptr) return ESP_FAIL;
    if (_ibus) delete _ibus; // 기존에 할당된 버스가 있다면 해제
    _ibus = bus;            // 새로 생성된 I2CBus 또는 SPIBus 주입
    return ESP_OK;
}

void ICM20948::set_bus(Interface::IBus *bus){
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
    if (_ibus == nullptr) return ESP_FAIL;
    esp_err_t err;

    if (_ibus->get_type() == Interface::BusType::I2C) {
        // --- [I2C 모드: 기존 Bypass 로직 유지] ---
        select_bank(0);
        _ibus->Write(B0_USER_CTRL, 0x00);    // I2C Master Off
        _ibus->Write(B0_INT_PIN_CFG, 0x02); // Bypass On
        ESP_LOGI(TAG, "I2C Bypass Mode Enabled");
    } 
    else if (_ibus->get_type() == Interface::BusType::SPI) {
              ESP_LOGI(TAG, "Starting SPI Sensor Hub Initializer...");

        // 1. Bank 0: 기존 마스터 기능 끄고 리셋 전처리
        select_bank(0); esp_rom_delay_us(50);
        _ibus->Write(B0_USER_CTRL, 0x00);    
        vTaskDelay(pdMS_TO_TICKS(10));

        // 2. Bank 3: 보조 I2C 버스를 깨우기 위한 필수 클럭 가동 (가장 중요)
        select_bank(3); esp_rom_delay_us(50);
        _ibus->Write(B3_I2C_MST_CTRL, 0x1D);       // I2C Master 클럭 ~400kHz 세팅
        _ibus->Write(0x01, 0x0D);                  // I2C_MST_CTRL (MULT_MST_EN 등 칩 특성 반영)
        _ibus->Write(0x02, 0x80);                  // I2C_MST_DELAY_CTRL: 지연 레지스터 활성화
        vTaskDelay(pdMS_TO_TICKS(10));

        // 3. Bank 0: 내부 I2C Master 엔진 활성화 (USER_CTRL의 I2C_MST_EN 비트 켜기)
        select_bank(0); esp_rom_delay_us(50);
        _ibus->Write(B0_USER_CTRL, 0x20);          // 0x20 = I2C_MST_EN
        vTaskDelay(pdMS_TO_TICKS(20));             // 마스터 엔진 안정화 대기

        // 4. Bank 3 (Slave 4 단발성 통신): AK09916 지자계 센서 소프트 리셋
        select_bank(3); esp_rom_delay_us(50);
        _ibus->Write(B3_I2C_SLV4_ADDR, 0x0C);      // AK09916 Write 주소 (0x0C)
        _ibus->Write(B3_I2C_SLV4_REG, 0x30);       // CNTL1 (Reset)
        _ibus->Write(B3_I2C_SLV4_DO, 0x01);        // Soft Reset 명령값
        _ibus->Write(B3_I2C_SLV4_CTRL, 0x80);      // SLV4_EN (1회 전송 촉발)
        vTaskDelay(pdMS_TO_TICKS(50));             // 리셋 완료 대기

        // 5. Bank 3 (Slave 4 단발성 통신): 지자계 100Hz 연속 측정 모드 2 설정
        _ibus->Write(B3_I2C_SLV4_ADDR, 0x0C); 
        _ibus->Write(B3_I2C_SLV4_REG, 0x31);       // CNTL2 (Operation Mode)
        _ibus->Write(B3_I2C_SLV4_DO, 0x08);        // 0x08 = Continuous Measurement 2 (100Hz)
        _ibus->Write(B3_I2C_SLV4_CTRL, 0x80);      // 1회 전송 촉발
        vTaskDelay(pdMS_TO_TICKS(20));

        // 6. Bank 3 (Slave 0 백그라운드 스케줄러): 오토 리딩 영구 등록
        // ★ [핵심 주소 처리] SPI 환경에 따라 최상위 비트(0x80)가 유실될 수 있으므로 주소 확인 필수
        _ibus->Write(B3_I2C_SLV0_ADDR, 0x0C | 0x80); // 0x8C = 지자계 Read 주소
        _ibus->Write(B3_I2C_SLV0_REG, 0x10);         // ST1 레지스터(0x10)부터 읽기 시작
        _ibus->Write(B3_I2C_SLV0_CTRL, 0x89);        // 0x89 = Slave 활성화(0x80) + 9바이트 읽기(0x09)
        
        // Slave 0이 스케줄러에 동기화되어 밀리지 않도록 지연 섀도우 처리
        _ibus->Write(0x09, 0x01);                  // I2C_SLV0_DELAY_SHDW = 1
        
        select_bank(0); esp_rom_delay_us(50);
        ESP_LOGI(TAG, "SPI Sensor Hub Successfully Configured.");
    }
    return ESP_OK;
}

/**
 * @brief 
 *      1. icm20948은 기능이 많아서 bank를 활용하여 처리한다.
 * @param bank 
 * @return esp_err_t 
 */
esp_err_t ICM20948::select_bank(uint8_t bank)
{
    esp_err_t err = ESP_FAIL;
    if (_ibus == nullptr) return err;
    err = _ibus->Write(REG_BANK_SEL,(uint8_t)(bank << 4));
    return err;
}

/**
 * @brief 
 *      1. I2C && SPI IBus에 따라서 자동으로 읽는다.
 * @param raw 
 * @return esp_err_t 
 */
esp_err_t ICM20948::read_data(ImuData &raw) {
    if (_ibus == nullptr) return ESP_ERR_INVALID_STATE;        
    
    esp_err_t err = ESP_OK;
    select_bank(0);
    
    if (_ibus->get_type() == Interface::BusType::I2C) {
        // --- [I2C 모드: 기존 코드 유지] ---
        uint8_t d[12]{0,};
        err = _ibus->Read(B0_ACCEL_XOUT_H, d, 12);
        if (err != ESP_OK) return err;

        raw.acc.x = (int16_t)((d[0] << 8) | d[1]) * ACCEL_SCALE;
        raw.acc.y = (int16_t)((d[2] << 8) | d[3]) * ACCEL_SCALE;
        raw.acc.z = (int16_t)((d[4] << 8) | d[5]) * ACCEL_SCALE;
        raw.gyro.x = (int16_t)((d[6] << 8) | d[7])  * GYRO_SCALE;
        raw.gyro.y = (int16_t)((d[8] << 8) | d[9])  * GYRO_SCALE;
        raw.gyro.z = (int16_t)((d[10] << 8) | d[11])* GYRO_SCALE;
        raw.timestamp = esp_timer_get_time();
        raw.is_mag_updated = false;
    } 
    else {
          // 22바이트 연속 리딩 (Accel 6 + Gyro 6 + Temp 2 + Mag 8)
        uint8_t d[22];
        err = _ibus->Read(B0_ACCEL_XOUT_H, d, 22);
        if (err != ESP_OK) return err;

        // 1. 가속도 및 자이로 변환 (기존 코드 유지)
        raw.acc.x = (int16_t)((d[0] << 8) | d[1]) * ACCEL_SCALE;
        raw.acc.y = (int16_t)((d[2] << 8) | d[3]) * ACCEL_SCALE;
        raw.acc.z = (int16_t)((d[4] << 8) | d[5]) * ACCEL_SCALE;

        raw.gyro.x = (int16_t)((d[6] << 8) | d[7]) * GYRO_SCALE;
        raw.gyro.y = (int16_t)((d[8] << 8) | d[9]) * GYRO_SCALE;
        raw.gyro.z = (int16_t)((d[10] << 8) | d[11]) * GYRO_SCALE;
        raw.timestamp = esp_timer_get_time();

        // 2. [핵심] 지자계 상태 레지스터 추출
        uint8_t st1 = d[14]; // d[14]는 AK09916의 ST1 레지스터
        uint8_t st2 = d[21]; // 22바이트의 맨 마지막 데이터(d[21])는 ST2 레지스터

        // 3. 하드웨어 상태 검증 (새로운 데이터 체크 & 오버플로우 체크)
        // - st1 & 0x01 : DRDY 비트가 1인가? (지자계가 진짜 새로 갱신되었는가)
        // - !(st2 & 0x08) : HOFL 비트가 0인가? (자성 물질 근접으로 인한 센서 포화 오버플로우가 없는가)
        if ((st1 & 0x01) && !(st2 & 0x08)) {
            raw.is_mag_updated = true;
            raw.mag_timestamp = raw.timestamp;

            // 새로운 데이터가 정상 확인되었을 때만 물리 변환 수행
            raw.mag.x = (int16_t)((d[16] << 8) | d[15]) * MAG_SCALE; 
            raw.mag.y = (int16_t)((d[18] << 8) | d[17]) * MAG_SCALE;
            raw.mag.z = (int16_t)((d[20] << 8) | d[19]) * MAG_SCALE;
            _mag_previous = raw.mag;
            _mag_previous_time = raw.timestamp;
        } else {
            // 1. 새로운 데이터가 없는 고속 루프 구간에서는 플래그를 꺼서 외부 각도 계산 알고리즘이 누적 연산을 수행하지 않도록 방어합니다.
            // 2. 일단 데이터가 없는경우에는 이전 값을 반환한다.
            raw.is_mag_updated = false;
            raw.mag = _mag_previous;
            raw.mag_timestamp = _mag_previous_time;
        }
    }
    return err;
}

/**
 * @brief 
 *      1. 튀는 값을 LPF 필터링으로 안정화 시킴.
 * @param io_data 
 */
void ICM20948::apply_filter(ImuData &io_data){
    if (!_calibration) return;

    // 1차 LPF 필터링 처리
    _last_filtered_accel = (io_data.acc * _lpf_alpha)  + (_last_filtered_accel * (1.0f - _lpf_alpha));
    _last_filtered_gyro  = (io_data.gyro * _lpf_alpha) + (_last_filtered_gyro  * (1.0f - _lpf_alpha));

    // 최종 정제 데이터 반영
    io_data.acc  = _last_filtered_accel;
    io_data.gyro = _last_filtered_gyro;
}

/**
 * @brief 
 * @param sample 
 * @return true 
 * @return false 
 */
esp_err_t ICM20948::updateSample(ImuData &sample){    
    // 1. 하드웨어 버스 연결 상태 방어적 체크
    if (_ibus == nullptr) return ESP_ERR_INVALID_STATE;        
    
    ImuData data {}; // 임시 버퍼 초기화
    data.acc = 0.0f;
    data.gyro = 0.0f;
    data.mag = 0.0f;

    esp_err_t err = read_data(data); // 칩 레지스터 일괄 리딩 (내부에서 mag 데이터 및 플래그 갱신됨)
    
    // 2. 통신이 완벽하게 성공한 경우에만 상위 객체로 데이터 복사
    if (err == ESP_OK){
        // [기본 IMU 데이터 전달]
        sample.acc         = data.acc   - _acc_bias;
        sample.gyro        = data.gyro  - _gyro_bias;
        sample.temperature = data.temperature;
        sample.timestamp   = data.timestamp;

        // [mag 데이터 업데이발생이 안될 경우 이전값을 받아온다.]
        sample.mag    = (data.mag -_mag_offset) * _mag_scale;
        sample.mag_timestamp  = data.mag_timestamp;
        sample.is_mag_updated = data.is_mag_updated;            
    }
    return err;
}

/**
 * @brief 
 *      1. loop 안에서 돌리면 500번 처리하여 자동으로 calibration 처리됨.
 * @param data 
 * @param sample_count 
 * @return esp_err_t 
 */
esp_err_t  ICM20948::calibration_loop(const ImuData& data, int sample_count){
    if( _calibration){
        return ESP_OK;
    }
    _gyro_bias = _gyro_bias + data.gyro;
    _acc_bias = _acc_bias + Vector3f(data.acc.x, data.acc.y, data.acc.z - 1.0f); 
    if (sample_count >= 2000) {
        _gyro_bias  = _gyro_bias * (1.0f / 2000.0f);
        _acc_bias   = _acc_bias  * (1.0f / 2000.0f);
        _calibration = true;
        //ESP_LOGW(TAG, "교정 완료 -> Gyro Bias X: %f, Y: %f, Z: %f", _gyro_bias.x, _gyro_bias.y, _gyro_bias.z);
    }
    return ESP_OK;
}


// Utils::ImuCalibrator calibrator;

// void flight_control_loop() {
//     ImuData imu_data;
//     icm20948.updateSample(imu_data); // 1단계: 순수 데이터 취득

//     if (시스템_초기화_중) {
//         calibrator.accumulate_bias(imu_data, current_sample_count++);
//         return;
//     }

//     // [방어 레이어 가동]
//     calibrator.apply_filter(imu_data); // 0점 오차 제거 및 고주파 진동 부드럽게 커팅 완료!

//     // 2단계: 노이즈가 없는 깨끗한 데이터를 칼만 필터가 원하는 ENU 축으로 전환
//     Utils::FrameTransformer::convert_to(imu_data, Utils::TargetCoord::ENU);

//     // 3단계: 칼만 필터 연산 수행 (노이즈가 없으므로 정밀한 각도 추정 가능)
//     // kalman.update(imu_data);
// }



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
            sum_acc.x  += (int16_t)((buf[0] << 8)  | buf[1])  * ACCEL_SCALE;
            sum_acc.y  += (int16_t)((buf[2] << 8)  | buf[3])  * ACCEL_SCALE;
            sum_acc.z  += (int16_t)((buf[4] << 8)  | buf[5])  * ACCEL_SCALE;
            sum_gyro.x += (int16_t)((buf[6] << 8)  | buf[7])  * GYRO_SCALE;
            sum_gyro.y += (int16_t)((buf[8] << 8)  | buf[9])  * GYRO_SCALE;
            sum_gyro.z += (int16_t)((buf[10] << 8) | buf[11]) * GYRO_SCALE;
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
        _acc_bias     = sum_acc / valid_samples;
        _acc_bias.z  -= 1.0f;                     //중력을 뺀다. offetset이기 때문에 -1을 해준다.
        _gyro_bias    = sum_gyro / valid_samples;

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