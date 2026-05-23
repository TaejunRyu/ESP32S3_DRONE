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
                 chip_id_buf[1], actual_chip_id);
        return ESP_FAIL; // 이 검증선이 뚫려야 센서가 정상 동작합니다.
    }
    // ESP_LOGW(TAG,"chip_id_buf[0] : 0x%02X",chip_id_buf[0]);
    // ESP_LOGW(TAG,"chip_id_buf[1] : 0x%02X",chip_id_buf[1]);

    // ESP_LOGI(TAG, "BMP388 Hardware Connection Perfectly Verified! Chip ID: 0x%02X", actual_chip_id);


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
   // 질문자님의 통찰 반영: 21바이트 데이터를 위해 22바이트 배열 선언
    uint8_t d[22] = {0}; 
    
    // 0x13번지부터 더미 포함 22바이트 연속 로드
    esp_err_t ret = _ibus->Read(0x31, d, 22); 
    if (ret != ESP_OK) return ret;
    
    // 0번 인덱스(d[0])는 보쉬 SPI 필수 더미 바이트이므로 패스, 1번 인덱스부터 사용
    size_t o = 1; 

    // [부호 확장 차단 정밀 캐스팅 매핑]
    // 하위 바이트 d[o]와 상위 바이트 d[o+1]을 완벽하게 uint16_t 공간 안에서 OR 연산한 뒤,
    // 부호가 필요한 변수들만 최종적으로 (int16_t) 처리를 해줍니다.
    _coef.t1 = (uint16_t)(((uint16_t)d[o + 1] << 8) | (uint16_t)d[o + 0]);
    _coef.t2 = (uint16_t)(((uint16_t)d[o + 3] << 8) | (uint16_t)d[o + 2]);
    _coef.t3 = (int8_t)d[o + 4];
    
    _coef.p1 = (int16_t)(((uint16_t)d[o + 6] << 8) | (uint16_t)d[o + 5]);
    _coef.p2 = (int16_t)(((uint16_t)d[o + 8] << 8) | (uint16_t)d[o + 7]);
    _coef.p3 = (int8_t)d[o + 9];
    _coef.p4 = (int8_t)d[o + 10];
    
    _coef.p5 = (uint16_t)(((uint16_t)d[o + 12] << 8) | (uint16_t)d[o + 11]);
    _coef.p6 = (uint16_t)(((uint16_t)d[o + 14] << 8) | (uint16_t)d[o + 13]);
    _coef.p7 = (int8_t)d[o + 15];
    _coef.p8 = (int8_t)d[o + 16];
    
    _coef.p9 = (int16_t)(((uint16_t)d[o + 18] << 8) | (uint16_t)d[o + 17]);
    _coef.p10 = (int8_t)d[o + 19];
    _coef.p11 = (int8_t)d[o + 20];
    // read_calib() 내부 맨 하단에 추가
    // ESP_LOGW("CALIB_RAW", "d[1]:0x%02X, d[2]:0x%02X | d[6]:0x%02X, d[7]:0x%02X", d[1], d[2], d[6], d[7]);
    // ESP_LOGW("CALIB_COEF", "t1:%u, t2:%u, p1:%d, p2:%d", _coef.t1, _coef.t2, _coef.p1, _coef.p2);

    return ESP_OK;
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
    uint8_t status[2]{};
    uint8_t temp_bit  = 1<<5;
    uint8_t press_bit = 1<<4;
    uint8_t sum_mask = temp_bit | press_bit;

    // 타임아웃은 아주 짧게(1~2ms)
    esp_err_t err = _ibus->Read(this->STATUS,status,2);
    if ( err == ESP_OK) {
        // ESP_LOGW(TAG,"status[0]:0x%02X",status[0]);
        // ESP_LOGW(TAG,"status[1]:0x%02X",status[1]);
        return ((status[1] & sum_mask) == sum_mask); // 압력(0x10)과 온도(0x20) 모두 준비됨 확인
    }
    return false;
}

 
/**
 * @brief 
 *      1. 센서 초기화 함수(begin 등)에서 '딱 한 번' 계산
 */
void BMP388::init_coefficients() {
     // 거대한 정수 나눗셈 시 float 정밀도 붕괴(언더플로우)를 막기 위해,
    // 보쉬 공식 드라이버 사양 규격인 정확한 2의 거듭제곱 가중치 연산으로 전면 교정합니다.
    _p1 = ((float)_coef.p1 - 16384.0f) / 1048576.0f;               // (p1 - 2^14) / 2^20
    _p2 = ((float)_coef.p2 - 16384.0f) / 536870912.0f;              // (p2 - 2^14) / 2^29
    _p3 = (float)_coef.p3 / 4294967296.0f;                          // p3 / 2^32
    _p4 = (float)_coef.p4 / 137438953472.0f;                        // p4 / 2^37
    _p5 = (float)_coef.p5 * 8.0f;                                   // p5 * 2^3
    _p6 = (float)_coef.p6 / 64.0f;                                  // p6 / 2^6
    _p7 = (float)_coef.p7 / 256.0f;                                 // p7 / 2^8
    _p8 = (float)_coef.p8 / 32768.0f;                               // p8 / 2^15
    
    // [치명적 구간 수정] 거대한 상수는 double(64비트 정밀도) 캐스팅 후 연산하여 
    // 컴파일 시점의 정밀도 누수를 원천 차단한 뒤 최종 float에 대입합니다.
    _p9 = (float)((double)_coef.p9 / 281474976710656.0);            // p9 / 2^48
    _p10 = (float)((double)_coef.p10 / 281474976710656.0);          // p10 / 2^48
    _p11 = (float)((double)_coef.p11 / 36893488147419103232.0);      // p11 / 2^65
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
   
        //ESP_LOGI("BMP_DIAG", "pressure:%8.3f", *pressure);

        return ret_code; // Pa -> hPa
    } else {
        *pressure = 0.0f;
        return ret_code;
    }
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
    
    //  ESP_LOGI("BMP_DIAG", "RAW ADC -> P: %u (0x%06X) | T: %u (0x%06X)", 
    //              (unsigned int)adc_p, (unsigned int)adc_p, 
    //              (unsigned int)adc_t, (unsigned int)adc_t);

    return ret;
}


}// namespace Sensor