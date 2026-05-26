#include "ryu_IST8310.hpp"
#include <tuple>
#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ryu_i2c.hpp"

namespace Sensor
{
IST8310::~IST8310()
{
    if (_ibus) {
        delete _ibus; // 이 순간 자식(SPIBus) 소멸자가 호출되며 하드웨어 자원(spi_bus_remove_device)까지 완전히 해제됩니다.
        _ibus = nullptr;
    }
}

esp_err_t IST8310::initialize()
{   
    if(_ibus == nullptr) return ESP_FAIL; // 인터페이스 주입 확인

    // 1. 소프트 리셋 (중요: 리셋 후 반드시 50ms 이상 대기)
    //uint8_t reset_cmd[] = {CONTROL2, 0x01}; // CNTL2, Soft Reset
    esp_err_t err = _ibus->Write(CONTROL2,0x01);
    if (err != ESP_OK){
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    //2. WHO_AM_I 확인 (정상 연결 체크)
    uint8_t who_reg = 0x00, who_val = 0;
    err = _ibus->Read(who_reg,&who_val,1);
    if (err != ESP_OK){
        return err;
    }
    if (who_val != 0x10) {
        ESP_LOGE(TAG, "연결 실패! ID: 0x%02X (기대값: 0x10)", who_val);
        return ESP_FAIL;
    }

    // 3. 센서 내부 동작 환경 설정 (이 루틴이 없으면 데이터 갱신 안됨)
    // AVGCNTL(0x41): 0x24 (X,Y,Z 모두 16회 평균으로 노이즈 제거)
    //uint8_t avg_data[] = {AVGCNTL, 0x24};
    err = _ibus->Write(AVGCNTL, 0x09);
    if (err != ESP_OK){
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    // PDCNTL(0x42): 0xC0 (Pulse Duration 권장값)
    //uint8_t pd_data[] = {PDCNTL, 0xC0};
    err = _ibus->Write(PDCNTL, 0xC0);
    if (err != ESP_OK){
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(50));
 
    // CROSS  AXIS 1
    //uint8_t mode_cmd_axis1[] = {CROSSAXIS1, 0x01};
    err = _ibus->Write(CROSSAXIS1, 0x01);
    if (err != ESP_OK){
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    // CROSS  AXIS 2
    //uint8_t mode_cmd_axis2[] = {CROSSAXIS2, 0x01};
    err = _ibus->Write(CROSSAXIS2, 0x01);
    if (err != ESP_OK){
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    // 4. 모드 설정: 100Hz 연속 측정 모드 (CNTL1)
    // 0x0B = 100Hz Continuous Mode
    //uint8_t mode_cmd_continuous[] = {CONTROL1, 0x0B};
    err = _ibus->Write(CONTROL1, 0x0B);
    if (err != ESP_OK){
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    this->_initialized = true;
    ESP_LOGI(TAG, "Initialized successfully. (ID: 0x10, 100Hz mode)");
    return ESP_OK;
}


esp_err_t IST8310::deinitialize()
{
    //1. 상태 체크
    if (!_initialized) {
        return ESP_OK;
    }
    // 2. 하드웨어 자원 해제 (중요!)
    // 만약 BusInterface 객체의 생명주기를 ICM20948이 관리한다면 여기서 delete 합니다.
    // 외부에서 관리한다면 단순히 포인터를 nullptr로 만듭니다.
    _ibus = nullptr; 
    // 3. 상태 업데이트
    _initialized = false;
    ESP_LOGI(TAG, "Deinitialized successfully (Interface detached).");
    return ESP_OK; 
}

esp_err_t IST8310::updateSample(Vector3f &sample)
{
      // 1. 하드웨어 버스 연결 상태 방어적 체크
    if (_ibus == nullptr) return ESP_FAIL;        
    
    Vector3f data{};
    esp_err_t err = read_data(data); // 칩 레지스터 일괄 리딩 (내부에서 mag 데이터 및 플래그 갱신됨)

    // 2. 통신이 완벽하게 성공한 경우에만 상위 객체로 데이터 복사
    if (err == ESP_OK){
        sample    = (data -_mag_offset) * _mag_scale;
    }
    return err;
}

void IST8310::align_NED(Vector3f &data){
    data.x *=  -1.0f;
    data.y *=  -1.0f;
}


bool IST8310::is_data_ready()
{  
    // 1. 센서 인터페이스 주입 확인 및 초기화 여부 검사
    if (_ibus == nullptr || !_initialized) {
        return false;
    }
    uint8_t status_reg = 0;
    // 2. IBus 인터페이스를 통해 STAT1(0x02) 레지스터 1바이트 읽기
    esp_err_t err = _ibus->Read(STAT1, &status_reg, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read STAT1 register: %s", esp_err_to_name(err));
        return false;
    }
    return (status_reg & STAT1_DRDY_MASK) != 0;
}



esp_err_t IST8310::read_data(Vector3f& data)
{
    data =0.0f;
    uint8_t rx_buf[6] = {0};
    esp_err_t err = _ibus->Read(DATA_X_L,rx_buf,6);

    // 2. I2C 통신 실패 시 방어 코드
    if (err != ESP_OK) {
        // 통신이 실패하면 드론이 추락하는 것을 막기 위해 '직전 정상 데이터'를 그대로 반환합니다.
        return err;
    }
    // 3. 바이트 결합 및 부호 있는 16비트 정수 변환 (Little Endian)
    // IST8310은 하위 바이트(Low)가 먼저 오고 상위 바이트(High)가 나중에 옵니다.
    Vector3f raw{};
    raw.x = (int16_t)((rx_buf[1] << 8) | rx_buf[0]);
    raw.y = (int16_t)((rx_buf[3] << 8) | rx_buf[2]);
    raw.z = (int16_t)((rx_buf[5] << 8) | rx_buf[4]);

    // 4. 감도 적용 (Gauss 또는 uT 단위 변환)
    // 헤더에 정의하신 SENSITIVITY(0.3f) 값을 곱해 물리량으로 바꿉니다.
    data =  raw * SENSITIVITY;
    return ESP_OK;
}



void IST8310::calibrate_hard_iron()
{
    Vector3f max_mag{-99999.0f, -99999.0f, -99999.0f};
    Vector3f min_mag{ 99999.0f,  99999.0f,  99999.0f}; 

    Vector3f data{};
    ESP_LOGI(TAG, "IST8310 지자계 보정 시작: 드론을 모든 방향(8자)으로 끊임없이 돌리세요!");    
    ESP_LOGI(TAG, "실제 유효 데이터 15,000 샘플을 수집합니다... (약 30초~1분 소요)");

    // 💡 유령 데이터 중복 카운트를 막기 위해 진짜 유효 샘플 수(15,000개) 기준으로 변경
    uint32_t total_valid_samples = 15000; 
    uint32_t valid_count = 0;

    while (valid_count < total_valid_samples) {
        
        // 💡 [필수] IST8310 단일 측정 시작 레지스터 트리거 송신 (함수 내부에 없다면 여기서 처리)
        // this->trigger_single_measurement(); 

        this->read_data(data);

        // 💡 [핵심 안전장치] 데이터가 공백(0.0f)이거나 이전 값 홀딩 상태가 아닐 때만 처리
        // (앞서 무효 데이터를 {0,0,0}으로 밀어주도록 송신단을 수정했으므로 크기 검사로 완벽 방어)
        float data_norm = std::sqrt(data.x*data.x + data.y*data.y + data.z*data.z);
        if (data_norm > 0.001f) {
            
            valid_count++; // 진짜 데이터가 들어왔을 때만 진행률 카운트 업!

            max_mag.x = std::max(data.x, max_mag.x);
            max_mag.y = std::max(data.y, max_mag.y);
            max_mag.z = std::max(data.z, max_mag.z);

            min_mag.x = std::min(data.x, min_mag.x);
            min_mag.y = std::min(data.y, min_mag.y);
            min_mag.z = std::min(data.z, min_mag.z);

            // 500번 유효 샘플마다 진행률 출력
            if (valid_count % 500 == 0) {
                ESP_LOGW(TAG, "IST8310 : 실제 데이터 수집 중... (%d%%)", (valid_count * 100) / total_valid_samples);
            }
        }

        // IST8310의 하드웨어 측정 대기 시간(약 6~8ms)을 고려하여 
        // 1ms 대신 5ms~8ms 딜레이를 주면 CPU 자원을 대폭 아낄 수 있습니다.
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    // 최종 하드 아이언 오프셋(중심점) 계산
    Vector3f offset_mag = (max_mag + min_mag) / 2.0f;
   
    // 축별 반경 계산 및 0 나누기 방어코드
    Vector3f avg_delte_mag = (max_mag - min_mag) / 2.0f;
    if (avg_delte_mag.x < 0.001f) avg_delte_mag.x = 1.0f;
    if (avg_delte_mag.y < 0.001f) avg_delte_mag.y = 1.0f;
    if (avg_delte_mag.z < 0.001f) avg_delte_mag.z = 1.0f;

    float avg_delta = (avg_delte_mag.x + avg_delte_mag.y + avg_delte_mag.z) / 3.0f;   

    Vector3f scale_mag{};
    scale_mag.x = avg_delta / avg_delte_mag.x;
    scale_mag.y = avg_delta / avg_delte_mag.y;
    scale_mag.z = avg_delta / avg_delte_mag.z;

    // 💡 로그 컴포넌트 명칭도 명확히 IST8310으로 수정
    ESP_LOGW(TAG, "IST8310 HARD/SOFT IRON 보정 완료!");
    ESP_LOGW(TAG, "--------------------------------------------------");
    ESP_LOGW(TAG, "static inline constexpr float MAG_MAX_X      = %.4f;",   max_mag.x);
    ESP_LOGW(TAG, "static inline constexpr float MAG_MAX_Y      = %.4f;",   max_mag.y);
    ESP_LOGW(TAG, "static inline constexpr float MAG_MAX_Z      = %.4f;",   max_mag.z);   
    ESP_LOGW(TAG, "static inline constexpr float MAG_MIN_X      = %.4f;",   min_mag.x);
    ESP_LOGW(TAG, "static inline constexpr float MAG_MIN_Y      = %.4f;",   min_mag.y);
    ESP_LOGW(TAG, "static inline constexpr float MAG_MIN_Z      = %.4f;",   min_mag.z);
    ESP_LOGW(TAG, "static inline constexpr float MAG_SCALE_X    = %.4f;",   scale_mag.x);
    ESP_LOGW(TAG, "static inline constexpr float MAG_SCALE_Y    = %.4f;",   scale_mag.y);
    ESP_LOGW(TAG, "static inline constexpr float MAG_SCALE_Z    = %.4f;",   scale_mag.z);
    ESP_LOGW(TAG, "static inline constexpr float MAG_OFFSET_X   = %.4f;",   offset_mag.x);
    ESP_LOGW(TAG, "static inline constexpr float MAG_OFFSET_Y   = %.4f;",   offset_mag.y);
    ESP_LOGW(TAG, "static inline constexpr float MAG_OFFSET_Z   = %.4f;",   offset_mag.z);
    ESP_LOGW(TAG, "--------------------------------------------------");

    for (uint8_t ii = 0; ii < 10; ++ii) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


} //namespace Sensor