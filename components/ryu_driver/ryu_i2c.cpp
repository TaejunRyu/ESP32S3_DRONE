#include "ryu_i2c.hpp"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>


namespace Driver{

esp_err_t I2C::initialize()
{        
    if(_initialized){
        return ESP_OK;
    }

    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port = _port;
    bus_cfg.sda_io_num = _port_sda;
    bus_cfg.scl_io_num = _port_scl;
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.intr_priority = 1;
    bus_cfg.flags.enable_internal_pullup = true;;
    bus_cfg.flags.allow_pd = false;

    // 내부에서 생성한 bus_handle의 주소를 전달
    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &_bus_handle);

    if (ret != ESP_OK)
    {
        ESP_LOGI(TAG, "Initialize Failed.");
        return ret; // 실패 시 NULL 반환
    }

    _initialized = true;
    ESP_LOGI(TAG, "Initialized successfully.");
    return ret; // 생성된 핸들 반환
    
}

esp_err_t I2C::deinitialize()
{
    esp_err_t err = ESP_OK;
    if (_bus_handle) {        
        err = i2c_del_master_bus(_bus_handle); 
        if (err !=ESP_OK)
            return err;
        _bus_handle = nullptr; 
    }
    _initialized = false;
    return err;
    ESP_LOGI(TAG, "I2C Bus & GPIO recovered and deinitialized.");
}


void I2C::scan_bus()
{
    printf("\n     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");    
    for (int i = 0; i < 128; i += 16) {
        printf("%02x: ", i);
        for (int j = 0; j < 16; j++) {
            uint8_t addr = i + j;
            
            // 주소 범위 체크 (I2C 표준: 0x03 ~ 0x77 사이만 유효)
            if (addr < 0x03 || addr > 0x77) {
                printf("   ");
                continue;
            }

            // i2c_master_probe: 장치가 응답(ACK)하는지 확인하는 전용 함수
            // 타임아웃은 10ms 정도면 충분합니다.
            esp_err_t ret = i2c_master_probe(_bus_handle, addr, pdMS_TO_TICKS(10));
            if (ret == ESP_OK) {
                printf("%02x ", addr);
            } else {
                printf("-- ");
            }
        }
        printf("\n");
    }
    printf("\n");
    vTaskDelay(pdMS_TO_TICKS(5000)); // 스캔 후 잠시 대기
}
}