#include "ryu_BMP388.hpp"

#include <tuple>
#include "ryu_BusInterface.hpp"

namespace Sensor{
BMP388::~BMP388(){
}

void BMP388::set_bus(Interface::IBus *bus)
{
    if (_ibus != nullptr) {
        delete _ibus; // 기존에 할당되어 있던 SPIBus 또는 I2CBus 객체 파괴
    }
    _ibus = bus; 
}


esp_err_t BMP388::initialize()
{    
    esp_err_t err = ESP_FAIL;

    if(_initialized){
        ESP_LOGI(TAG,"Already initialized.");
        return ESP_FAIL;
    }

    if(_ibus == nullptr) return ESP_FAIL; // 인터페이스 주입 확인

// [질문자님 가이드 반영] 1바이트를 안전하게 읽기 위해 '더미 안착용' 2바이트 배열 선언
    uint8_t id_check_buf[2] = {0}; 
    
    // 1. 초기 래치 해제용 CHIP_ID 공회전 (2바이트 요청)
    _ibus->Read(0x00, id_check_buf, 2);
    vTaskDelay(pdMS_TO_TICKS(20)); 

    // 2. Soft Reset 수행
    err = _ibus->Write(0x7E, 0xB6);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(50)); // 리셋 후 부팅 대기 (데이터시트 필수 스펙)

    // 3. 진짜 CHIP_ID 검증 (2바이트 요청)
    // 📢 통찰 반영: 버퍼 크기를 2로 주어 더미가 [0]번에 안전하게 박히도록 유도합니다.
    uint8_t chip_id_buf[2] = {0};
    err = _ibus->Read(0x00, chip_id_buf, 2);
    
    // [인덱스 1번 조립] 0번은 더미가 먹고, 1번에 안착한 진짜 알맹이(0x50)를 추출합니다!
    uint8_t actual_chip_id = chip_id_buf[1]; 
    
    if (err != ESP_OK || actual_chip_id != 0x50) {
        ESP_LOGE(TAG, "CHIP_ID Verification Failed! Read Dummy[0]: 0x%02X, Actual_ID[1]: 0x%02X (Expected: 0x50)", 
                 chip_id_buf[0], actual_chip_id);
        return ESP_FAIL; // 이 검증선이 뚫려야 센서가 정상 동작합니다.
    }
    ESP_LOGI(TAG, "BMP388 Hardware Connection Perfectly Verified! Chip ID: 0x%02X", actual_chip_id);

    // 4. Sleep Mode 진입 후 정석 설정값들 순서대로 주입
    _ibus->Write(0x1B, 0x00);
    vTaskDelay(pdMS_TO_TICKS(5));

    // 보정 계수 수집 (이전 대화에서 정립된 22바이트 버퍼 가이드 유지)
    err = read_calib();   
    if (err != ESP_OK) return err;
    init_coefficients();

    // 데이터시트 공식 오피셜 하드웨어 설정 매크로 작성
    _ibus->Write(0x1C, 0x0B);     // OSR 설정 (압력 x8, 온도 x2)
    vTaskDelay(pdMS_TO_TICKS(5));
    _ibus->Write(0x1F, 0x04);     // IIR 필터 계수 3
    vTaskDelay(pdMS_TO_TICKS(5));
    _ibus->Write(0x1D, 0x03);     // ODR 100Hz 주기 보정 (0x02에서 0x03으로 정정 완료)
    vTaskDelay(pdMS_TO_TICKS(15)); // 타이머 안정화 대기

    // 5. 최종 Normal Mode 가동 (0x33)
    err = _ibus->Write(0x1B, 0x33);
    if (err != ESP_OK) return err;
    
    vTaskDelay(pdMS_TO_TICKS(50)); // 첫 데이터 측정 필터링 적재 시간 휴식

    // 6. 교착 상태 해제를 위한 초기 지상 기압 획득
    float temp_press = 0.0f;
    for (int i = 0; i < 5; ++i) {
        get_pressure(&temp_press);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    _ground_pressure = (temp_press > 600.0f && temp_press < 1150.0f) ? temp_press : 1013.25f;

    _initialized = true;
    return ESP_OK;
}

esp_err_t BMP388::deinitialize()
{
    // 1. 상태 체크
    if (!_initialized) {
        return ESP_OK;
    }
    if (_ibus != nullptr) {
        delete _ibus; // 기존에 할당되어 있던 SPIBus 또는 I2CBus 객체 파괴
    }

    _ibus = nullptr; 
    // 3. 상태 업데이트
    _initialized = false;
    ESP_LOGI(TAG, "Deinitialized successfully (Interface detached).");
    return ESP_OK; 
}

/**
 * @brief 
 *      보정 계수 읽기.
 * @return esp_err_t 
 */
esp_err_t BMP388::read_calib()
{
   // 📢 통찰 반영: 원하는 길이(21)보다 1바이트 더 길게 잡음 (더미 안착용)
    uint8_t d[22] = {0}; 
    
    // SPI 인터페이스를 통해 총 22바이트 연속 로드
    esp_err_t ret = _ibus->Read(0x13, d, 22); // REG_CALIB = 0x13
    if (ret != ESP_OK) return ret;
    
    // [인덱스 1부터 조립 완성] 0번은 더미가 먹고, 1번부터 진짜 데이터 시작!
    _coef.t1 = (uint16_t)(((uint16_t)d[2] << 8) | d[1]);
    _coef.t2 = (uint16_t)(((uint16_t)d[4] << 8) | d[3]);
    _coef.t3 = (int8_t)d[5];
    
    _coef.p1 = (int16_t)(((uint16_t)d[7] << 8) | d[6]);
    _coef.p2 = (int16_t)(((uint16_t)d[9] << 8) | d[8]);
    _coef.p3 = (int8_t)d[10];
    _coef.p4 = (int8_t)d[11];
    
    _coef.p5 = (uint16_t)(((uint16_t)d[13] << 8) | d[12]);
    _coef.p6 = (uint16_t)(((uint16_t)d[15] << 8) | d[14]);
    _coef.p7 = (int8_t)d[16];
    _coef.p8 = (int8_t)d[17];
    
    _coef.p9 = (int16_t)(((uint16_t)d[19] << 8) | d[18]);
    _coef.p10 = (int8_t)d[20];
    _coef.p11 = (int8_t)d[21];
    
    return ESP_OK;
}


void BMP388::update_climb_rate(){
    // 현재 진행되어지는 고도는 fitered_alt가지고 작업 진행중...
    float raw_rate = (_filtered_alt - _last_altitude) / 0.020f; // 50Hz = 0.020s  , 40hz = 0.025s
    
    _last_altitude = _filtered_alt;
    // 속도 필터 (기압계 노이즈 제거용)
    _climb_rate = (_climb_rate * 0.8f) + (raw_rate * 0.2f);
    
}



esp_err_t BMP388::calibrate_ground_pressure(float* ground_pressure)
{
    float sum = 0;
    int count = 0;
    int attempts = 0; // 무한 루프 방지용
    int error_count = 0;

    ESP_LOGI(TAG, "✓ Start ground pressure correction (100 samplings)...");

    // 1. 센서 안정화를 위해 첫 데이터는 읽고 버림
    float pressure{};
    auto ret_code = get_pressure(&pressure);
    vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz 샘플링

    while(count < 100 && attempts < 200) { // 최대 200번 시도
        ret_code= get_pressure(&pressure);
        
        if (pressure > 800.0f && pressure < 1200.0f) { // 좀 더 타이트한 유효 범위 (지상 기준)
            sum += pressure;
            count++;
        } else {
            error_count ++;
            ESP_LOGW(TAG, "Detecting incorrect pressure values: %.2f hPa", pressure);
        }
        attempts++;
        vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz 샘플링

        if (count % 25 == 0 && count > 0) {
            ESP_LOGD(TAG, "보정 진행률: %d%%", count);
        }
        if(error_count > 10) {
            ESP_LOGE(TAG, "❌ Ground pressure correction failed (sensor check needed)");
            *ground_pressure = 0.0f;
            return ret_code;
        }
    }

    if (count >= 50) { // 최소 50개 이상의 유효 샘플 확보 시
        _ground_pressure = sum / (float)count;
        ESP_LOGI(TAG, "✓ Ground pressure setting complete: %.2f hPa (Samples: %d)",_ground_pressure, count);
        *ground_pressure = _ground_pressure;
        return ret_code;
    }
    ESP_LOGE(TAG, "❌ Ground pressure correction failed (sensor check needed)");
    *ground_pressure = 0.0f;
    return ret_code;
}

/**
 * @brief 
 *      어쩌면 이함수 사용 안하는게 나을지 모른다 I2C의 통신을 번잡하게 만들다.
 * @return true 
 * @return false 
 */
bool BMP388::is_data_ready()
{    
    // 4번째 bit : press  ready
    // 5번째 bit : temperature ready
    // 110000(2진수)   ==>  0x30
    uint8_t status = 0;
    uint8_t temp_bit  = 1<<5;
    uint8_t press_bit = 1<<4;
    uint8_t sum_mask = temp_bit | press_bit;

    // 타임아웃은 아주 짧게(1~2ms)
    esp_err_t err = _ibus->Read(this->STATUS,&status,1);
    if ( err == ESP_OK) {
        return ((status & sum_mask) == sum_mask); // 압력(0x10)과 온도(0x20) 모두 준비됨 확인
    }
    return false;
}

 
/**
 * @brief 
 *      1. 센서 초기화 함수(begin 등)에서 '딱 한 번' 계산
 */
void BMP388::init_coefficients() {
    // 여기서는 성능 걱정 없이 정확하게 계산합니다.
    _p1 = ((float)_coef.p1 - 16384.0f) / 1048576.0f;
    _p2 = ((float)_coef.p2 - 16384.0f) / 536870912.0f;
    _p3 = (float)_coef.p3 / 4294967296.0f;
    _p4 = (float)_coef.p4 / 137438953472.0f;
    _p5 = (float)_coef.p5 * 8.0f; 
    _p6 = (float)_coef.p6 / 64.0f;
    _p7 = (float)_coef.p7 / 256.0f;
    _p8 = (float)_coef.p8 / 32768.0f;
    _p9 = (float)_coef.p9 / 281474976710656.0f;
    _p10 = (float)_coef.p10 / 281474976710656.0f;
    _p11 = (float)_coef.p11 / 36893488147419103232.0f;
}


esp_err_t BMP388::get_pressure(float * pressure)
{
    uint32_t adc_p{},adc_t{}; 
    auto  ret_code = read_bmp388(&adc_p ,&adc_t);        

    if (ret_code == ESP_OK){
        float uncomp_p = static_cast<float>(adc_p);
        float uncomp_t = static_cast<float>(adc_t);

        // 2. 온도 보정 (정확한 지수값 사용)
        float partial_t1 = uncomp_t - static_cast<float>(_coef.t1) * 256.0f;
        float partial_t2 = partial_t1 * static_cast<float>(_coef.t2);
        // T-Lin 값 (압력 계산의 핵심 베이스)
        float t_lin = (partial_t2 / 1073741824.0f) + 
                    ((partial_t1 * partial_t1) * static_cast<float>(_coef.t3) / 281474976710656.0f);

        // 4. 최종 압력 계산
        float s1 = _p6 * t_lin;
        float s2 = _p7 * (t_lin * t_lin);
        float s3 = _p8 * (t_lin * t_lin * t_lin);
        float partial_out1 = _p5 + s1 + s2 + s3;

        s1 = _p2 * t_lin;
        s2 = _p3 * (t_lin * t_lin);
        s3 = _p4 * (t_lin * t_lin * t_lin);
        float partial_out2 = uncomp_p * (_p1 + s1 + s2 + s3);

        float d1 = uncomp_p * uncomp_p;
        float d2 = _p9 + _p10 * t_lin;
        float d3 = d1 * d2;
        float d4 = d3 + (uncomp_p * uncomp_p * uncomp_p) * _p11;

        float comp_press = partial_out1 + partial_out2 + d4;

        *pressure = static_cast<float>(comp_press * 0.01f);
        return ret_code; // Pa -> hPa
    } else {
        *pressure = 0.0f;
        return ret_code;
    }
}

esp_err_t BMP388::get_relative_altitude(float * filtered_alt)
{
    if (this->_ground_pressure <= 500.0f) {
        ESP_LOGE(TAG, "Error => Verify Ground Pressure..."); // 에러 종류 확인
        *filtered_alt = 0.0f;
        return ESP_FAIL; // 비정상적인 지면 기압 차단
    }
    float pressure{};
    auto ret_code = get_pressure(&pressure);
    
    if (ret_code != ESP_OK || pressure <= 500.0f ){
        *filtered_alt =_last_altitude;
        return ret_code; // 일시적 오류 시 이전 값 유지
    }

    // 고도 계산 공식 (ISA 모델)
    _current_alt = 44'330.0f * (1.0f - powf(pressure / _ground_pressure, 0.190295f));

    // 간단한 1차 저주파 필터 (Alpha: 0.1 ~ 0.3 권장)
    // 노이즈를 줄이고 부드러운 고도 변화를 만듭니다.
    const float alpha = 0.2f; 
    _filtered_alt = (_current_alt * alpha) + (_last_altitude * (1.0f - alpha));
    
    _last_altitude = _filtered_alt;

    update_climb_rate();
    *filtered_alt = _filtered_alt;
    return ret_code;
}



esp_err_t BMP388::read_bmp388(uint32_t* adcp,uint32_t* adct){
      // 📢 통찰 반영: 원하는 길이(6)보다 1바이트 더 길게 잡음 (더미 안착용)
    uint8_t d[7] = {0}; 
    
    // SPI 인터페이스를 통해 총 7바이트 연속 로드
    esp_err_t ret = _ibus->Read(0x04, d, 7); // REG_DATA = 0x04
    if (ret != ESP_OK) return ESP_FAIL;

    // [인덱스 1부터 조립 완성] 0번은 더미가 먹고, 1번부터 진짜 데이터 시작!
    // XLSB -> LSB -> MSB 정석 구조 매핑
    uint32_t adc_p = ((uint32_t)d[3] << 16) | ((uint32_t)d[2] << 8) | (uint32_t)d[1];
    uint32_t adc_t = ((uint32_t)d[6] << 16) | ((uint32_t)d[5] << 8) | (uint32_t)d[4];
    
    this->adc_p_last = adc_p;
    this->adc_t_last = adc_t;
    *adcp = adc_p;
    *adct = adc_t;
    
 ESP_LOGI("BMP_DIAG", "RAW ADC -> P: %u (0x%06X) | T: %u (0x%06X)", 
             (unsigned int)adc_p, (unsigned int)adc_p, 
             (unsigned int)adc_t, (unsigned int)adc_t);

    return ret;
}


}// namespace Sensor