#include "ryu_BMP388.hpp"
#include <tuple>
#include <freertos/FreeRTOS.h>
#include "ryu_i2c.hpp" 
#include "ryu_BusInterface.hpp"

namespace Sensor{
BMP388::~BMP388(){
    //
}

esp_err_t BMP388::updateSample(SensorData &sample)
{
    return esp_err_t();
}

esp_err_t BMP388::initialize()
{    
    esp_err_t err = ESP_FAIL;

    if(_initialized){
        ESP_LOGI(TAG,"%s Already initialized.",_name.c_str());
        return ESP_FAIL;
    }

    if(_ibus == nullptr) return ESP_FAIL; // 인터페이스 주입 확인

    // 1. Soft Reset
    err = _ibus->Write(0x7E, 0xB6);
    if(err !=ESP_OK){ 
        ESP_LOGI(TAG,"%s Soft Reset Failure.",_name.c_str());
        return err;
    }    
    vTaskDelay(pdMS_TO_TICKS(100)); // 시간을 넉넉히 줍니다.

    // 2. 중요: 일단 Sleep Mode로 전환하여 설정을 초기화 (0x1B에 0x00)
    err = _ibus->Write(0x1B, 0x00);

    if(err !=ESP_OK) {
        ESP_LOGI(TAG,"%s Sleep Mode Failure.",_name.c_str());
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // 1. OSR 설정 (압력 x8, 온도 x2 권장: 0x11)
    err = _ibus->Write(0x1C, (0x03 << 0)  | (0x01 << 3));
    if(err !=ESP_OK) {
        ESP_LOGI(TAG,"%s OSR Failure.",_name.c_str());
        return err;
    }
    //IIR 필터 계수 (0x1F): 현재 0x02 << 1 (계수 3) 정도로 설정되어 있습니다. 
    //비행 중 진동이 심하다면 이 값을 조금 더 높여(예: 계수 7) 노이즈를 억제할 수 있습니다.
    // 2. IIR 필터 및 ODR(100Hz) 설정
    err = _ibus->Write(0x1F, 0x02<<1);
    if(err !=ESP_OK) {
        ESP_LOGI(TAG,"%s IIR Failure.",_name.c_str());
        return err;
    }
    // 2. ODR 설정 (100Hz로 설정하여 50Hz 읽기 루프 지원)
    err = _ibus->Write(0x1D, 0x02);
    if(err !=ESP_OK) {
        ESP_LOGI(TAG,"%s ODR Failure.",_name.c_str());
        return err;
    }
    // 0x13: Forced Mode, Temp EN, Press EN
    err = _ibus->Write(0x1B, 0x13);

    if(err !=ESP_OK) {
        ESP_LOGI(TAG,"%s Forced Mode Failure.",_name.c_str());
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(50)); // 측정 완료 대기

    // 4. 드디어 Normal Mode 작동 (0x33)
    err = _ibus->Write(0x1B, 0x33);
    if(err !=ESP_OK) {
        ESP_LOGI(TAG," %s Normal Mode Failure.",_name.c_str());
        return err;
    }
    // 5. 첫 측정 대기: 중요!
    vTaskDelay(pdMS_TO_TICKS(50));
    // 보정계수 읽어오기
    err = read_calib();   
    if(err !=ESP_OK){ 
        return err;    
    }
    else{ // 복잡한 수식 계산 미리 처리
        init_coefficients();
    }
    _initialized = true;

    ESP_LOGI(TAG,"%s Initialized sucessfully.",this->_name.c_str());    
    return err;
}

esp_err_t BMP388::deinitialize()
{
    // 1. 상태 체크
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

/**
 * @brief 
 *      보정 계수 읽기.
 * @return esp_err_t 
 */
esp_err_t BMP388::read_calib()
{
    esp_err_t ret_code = ESP_FAIL;
    uint8_t d[21];
    //ret_code = i2c_master_transmit_receive(_dev_handle, &REG_CALIB, 1, d, 21, pdMS_TO_TICKS(10));
    ret_code = _ibus->Read(REG_CALIB,d,21);
    if(ret_code != ESP_OK) return ret_code;
    // 데이터시트 Table 29: Compensation parameter storage 정밀 매핑
    _coef.t1 = (uint16_t)((uint16_t)d[1] << 8 | d[0]);
    _coef.t2 = (uint16_t)((uint16_t)d[3] << 8 | d[2]);
    _coef.t3 = (int8_t)d[4];
    _coef.p1 = (int16_t)((int16_t)d[6] << 8 | d[5]);
    _coef.p2 = (int16_t)((int16_t)d[8] << 8 | d[7]);
    _coef.p3 = (int8_t)d[9];
    _coef.p4 = (int8_t)d[10];
    _coef.p5 = (uint16_t)((uint16_t)d[12] << 8 | d[11]);
    _coef.p6 = (uint16_t)((uint16_t)d[14] << 8 | d[13]);
    _coef.p7 = (int8_t)d[15];
    _coef.p8 = (int8_t)d[16];
    _coef.p9 = (int16_t)((int16_t)d[18] << 8 | d[17]);
    _coef.p10 = (int8_t)d[19];
    _coef.p11 = (int8_t)d[20];
    return  ret_code;
}


float BMP388::update_climb_rate()
{
    // 현재 진행되어지는 고도는 fitered_alt가지고 작업 진행중...
    float raw_rate = (this->_filtered_alt - this->_last_altitude) / 0.020f; // 50Hz = 0.020s  , 40hz = 0.025s
    this->_last_altitude = this->_filtered_alt;
    // 속도 필터 (기압계 노이즈 제거용)
    this->_climb_rate = (this->_climb_rate * 0.8f) + (raw_rate * 0.2f);
    return this->_climb_rate;
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
        this->_ground_pressure = sum / (float)count;
        ESP_LOGI(TAG, "✓ Ground pressure setting complete: %.2f hPa (Samples: %d)", this->_ground_pressure, count);
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
    // (this->p1 등은 센서 고유의 보정 데이터)
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
        float uncomp_p = (float)adc_p;
        float uncomp_t = (float)adc_t;

        // 2. 온도 보정 (정확한 지수값 사용)
        float partial_t1 = uncomp_t - (float)_coef.t1 * 256.0f;
        float partial_t2 = partial_t1 * (float)_coef.t2;
        // T-Lin 값 (압력 계산의 핵심 베이스)
        float t_lin = (partial_t2 / 1073741824.0f) + 
                    ((partial_t1 * partial_t1) * (float)_coef.t3 / 281474976710656.0f);

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

        *pressure = (float)(comp_press * 0.01f);
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
    this->_current_alt = 44330.0f * (1.0f - powf(pressure / this->_ground_pressure, 0.190295f));

    // 간단한 1차 저주파 필터 (Alpha: 0.1 ~ 0.3 권장)
    // 노이즈를 줄이고 부드러운 고도 변화를 만듭니다.
    const float alpha = 0.2f; 
    this->_filtered_alt = (this->_current_alt * alpha) + (this->_last_altitude * (1.0f - alpha));
    
    this->_last_altitude = this->_filtered_alt;

    this->update_climb_rate();
    *filtered_alt = _filtered_alt;
    return ret_code;
}


inline esp_err_t BMP388::read_bmp388(uint32_t* adcp,uint32_t* adct){
    uint8_t d[6] = {0};
    // 데이터 읽기 실패 시 0 반환
    esp_err_t ret_code  = _ibus->Read(REG_DATA,d,6);
    if (ret_code != ESP_OK) {
        ESP_LOGE(TAG, "Read error: %s", esp_err_to_name(ret_code)); // 에러 종류 확인
        *adcp = 0;
        *adct = 0;
        return ESP_FAIL;
    }
    // 1. Raw ADC (24-bit) 조합: 데이터시트상 [0]=LSB, [1]=MSB, [2]=XLSB 순서임
    // 반드시 uint32_t로 먼저 합친 후 double로 변환해야 데이터가 안 깨짐
    uint32_t adc_p = (uint32_t)d[2] << 16 | (uint32_t)d[1] << 8 | (uint32_t)d[0];
    uint32_t adc_t = (uint32_t)d[5] << 16 | (uint32_t)d[4] << 8 | (uint32_t)d[3];
    
    this->adc_p_last = adc_p;
    this->adc_t_last = adc_t;
    *adcp = adc_p;
    *adct = adc_t;
    
    return ret_code;
}


}// namespace Sensor