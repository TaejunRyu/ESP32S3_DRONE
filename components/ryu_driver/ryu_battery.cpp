#include "ryu_battery.hpp"



/**
 * @brief 팁: 멀티미터로 실제 배터리 전압을 재보고, 
 *            get_battery_voltage() 출력값과 차이가 있다면 이 값을 소수점 단위로 미세 조정(예: 11.12f)하여 캘리브레이션하세요.
 * 
 */
namespace Driver
{

esp_err_t Battery::initialize()
{
    if(_initialized) return ESP_OK;
    // 1. ADC 유닛 초기화
    adc_oneshot_unit_init_cfg_t init_config = {};
    init_config.unit_id = ADC_UNIT_1;
    
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_unit_handle));

    // 2. ADC 채널 설정 (GPIO 34 = ADC_CHANNEL_6)
    adc_oneshot_chan_cfg_t config = {
        .atten =  adc_atten_t::ADC_ATTEN_DB_12, // 12dB 감쇄 (v5.0 명칭)
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_unit_handle, BATTERY_ADC_CH, &config));

    // 3. ADC 보정 (에러 메시지 제안: line_fitting 사용)
    adc_cali_curve_fitting_config_t cali_config = {};
    cali_config.unit_id = ADC_UNIT_1;
    cali_config.atten = ADC_ATTEN_DB_12;
    cali_config.bitwidth = ADC_BITWIDTH_DEFAULT;
    
    // 함수명 정정: adc_cali_create_scheme_line_fitting
    esp_err_t ret =  adc_cali_create_scheme_curve_fitting(&cali_config, &adc_cali_handle);
    if (ret == ESP_OK) {
        do_calibration = true;
    }
    ESP_LOGI(TAG,"Initialized successfully.");
    _initialized = true;
    return ESP_OK;
}

float Battery::get_battery_voltage()
{
    int adc_raw = 0;
    int voltage_mv = 0;
    int adc_raw_sum = 0;
    const int samples = 10;
    
    for (int i = 0; i < samples; i++) {
        int temp_raw = 0;
        adc_oneshot_read(adc_unit_handle, BATTERY_ADC_CH, &temp_raw);
        adc_raw_sum += temp_raw;
    }
    adc_raw = adc_raw_sum / samples;
    
    if (adc_raw == 0) return 0.0f;
      // 2. 보정 적용 (Raw가 0이 아닐 때만 실행)
    if (do_calibration && adc_cali_handle != NULL) {
        adc_cali_raw_to_voltage(adc_cali_handle, adc_raw, &voltage_mv);
    } else {
        voltage_mv = (adc_raw * 3113) / 4095; // 3.3V 연결 시 확인한 3113mV 기준
    }
    //printf("DEBUG: Raw=%d, mV=%d\n", adc_raw, voltage_mv);
    // 3. 실제 전압 복원 (11.0배)
    return (voltage_mv / 1000.0f) * VOLTAGE_DIVIDER_RATIO;
}

void Battery::battery_check_task(void *pvParameters)
{
    Battery* bat = static_cast<Battery*>(pvParameters);

    while (true) {
        float voltage = bat->get_battery_voltage();
        // 예: 1셀당 3.5V 미만일 때 (3S 배터리 기준 10.5V)
        if (voltage > 0.5f && voltage < 10.5f) { 
            // 에러 핸들러 태스크에 '저전압 비트' 세팅
            //auto& failsafe = Service::FailSafe::get_instance();
            //xTaskNotify(failsafe._task_handle, Service::FailSafe::ERR_BATTERY_LOW, eSetBits);
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1초마다 확인
    }
}
BaseType_t Battery::start_task()
{
    auto  res= xTaskCreatePinnedToCore(battery_check_task, "Battery", 4096, this, 10, NULL, 0);
    if (res != pdPASS) ESP_LOGE(TAG, "❌ 4.Battery Check Task is failed! code: %d", res);
    else ESP_LOGI(TAG, "✓ 4.Battery Task is passed...");
    return res;
   
}

}