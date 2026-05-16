#include "ryu_spi.hpp"
#include "esp_log.h"

namespace Driver {

esp_err_t SPI::initialize() {
    if (_initialized) {
        ESP_LOGW(TAG, "SPI bus already initialized.");
        return ESP_OK;
    }

    // SPI 버스 설정
    spi_bus_config_t buscfg = {};
    buscfg.miso_io_num = _port_miso;
    buscfg.mosi_io_num = _port_mosi;
    buscfg.sclk_io_num = _port_sclk;
    buscfg.quadwp_io_num = -1;    // Write Protect 핀 사용 안 함
    buscfg.quadhd_io_num = -1;    // Hold 핀 사용 안 함
    buscfg.max_transfer_sz = 4092; // DMA 전송 최대 크기

    // SPI 버스 초기화 (DMA 채널 자동 할당)
    esp_err_t ret = spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    _initialized = true;
    ESP_LOGI(TAG, "SPI bus initialized successfully (SCLK:%d, MOSI:%d, MISO:%d)", 
             _port_sclk, _port_mosi, _port_miso);
             
    return ESP_OK;
}

esp_err_t SPI::deinitialize() {
    if (!_initialized) {
        ESP_LOGW(TAG, "SPI bus not initialized.");
        return ESP_OK;
    }

    esp_err_t ret = spi_bus_free(SPI_HOST);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to free SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    _initialized = false;
    ESP_LOGI(TAG, "SPI bus deinitialized.");
    
    return ESP_OK;
}

} // namespace Driver
