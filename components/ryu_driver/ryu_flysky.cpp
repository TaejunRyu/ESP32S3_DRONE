#include "ryu_flysky.hpp"

#include <cstring>
#include <esp_log.h>
#include <driver/gpio.h>
#include "ryu_SharedDataManager.hpp"


namespace Driver {


Flysky::Flysky(uart_port_t port)
    : port_(port), uart_baud_(BAUD_RATE), tx_pin_(GPIO_NUM_17), rx_pin_(GPIO_NUM_18),
      initialized_(false), is_connected_(false), rx_task_handle_(nullptr), data_queue_(nullptr) {
    memset(rc_values_, 0, sizeof(rc_values_));
    ESP_LOGI(TAG, "Flysky created for UART%d", port_);
}

Flysky::~Flysky() {
    deinitialize();
}

void Flysky::set_uart_pins(gpio_num_t tx, gpio_num_t rx) {
    tx_pin_ = tx;
    rx_pin_ = rx;
}

esp_err_t Flysky::initialize() {
    if (initialized_) return ESP_OK;

    ESP_LOGI(TAG, "Initializing Flysky on UART%d...", port_);

    // UART 드라이버 설정
    uart_config_t uart_config = {};
    uart_config.baud_rate = uart_baud_;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    // 데이터 큐 생성
    data_queue_ = xQueueCreate(10, sizeof(uint8_t) * IBUS_PACKET_SIZE);
    if (!data_queue_) {
        ESP_LOGE(TAG, "Failed to create data queue");
        return ESP_ERR_NO_MEM;
    }

    esp_err_t ret;

    // UART 드라이버 설치
    ret = uart_driver_install(port_, UART_BUF_SIZE * 2, 0, 30, &data_queue_, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART driver installation failed: %s", esp_err_to_name(ret));
        vQueueDelete(data_queue_);
        data_queue_ = nullptr;
        return ret;
    }

    // UART 설정 적용
    ret = uart_param_config(port_, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART configuration failed: %s", esp_err_to_name(ret));
        uart_driver_delete(port_);
        return ret;
    }

    // 핀 설정
    ret = uart_set_pin(port_, tx_pin_, rx_pin_, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART pin configuration failed: %s", esp_err_to_name(ret));
        uart_driver_delete(port_);
        return ret;
    }

    // 수신 태스크 생성
    ret = xTaskCreate(flysky_rx_task, "flysky_rx", 2048, this, 5, &rx_task_handle_);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create RX task");
        uart_driver_delete(port_);
        return ESP_ERR_NO_MEM;
    }

    initialized_ = true;
    is_connected_ = true;
    ESP_LOGI(TAG, "Flysky initialized successfully");
    return ESP_OK;
}

void Flysky::deinitialize() {
    if (!initialized_) return;

    if (rx_task_handle_ != nullptr) {
        vTaskDelete(rx_task_handle_);
        rx_task_handle_ = nullptr;
    }

    uart_driver_delete(port_);

    if (data_queue_ != nullptr) {
        vQueueDelete(data_queue_);
        data_queue_ = nullptr;
    }

    initialized_ = false;
    is_connected_ = false;
    ESP_LOGI(TAG, "Flysky deinitialized");
}

uint16_t Flysky::get_channel(uint8_t channel) const {
    if (channel >= IBUS_CHANNELS) return 0;
    return rc_values_[channel];
}

const uint16_t* Flysky::get_all_channels() const {
    return rc_values_;
}

bool Flysky::is_connected() const {
    return initialized_ && is_connected_;
}

esp_err_t Flysky::parse_ibus_packet(const uint8_t* packet, size_t len) {
    if (!packet || len != IBUS_PACKET_SIZE) {
        return ESP_ERR_INVALID_ARG;
    }

    // iBUS 패킷 헤더 확인
    if (packet[0] != 0x20 || packet[1] != 0x40) {
        ESP_LOGD(TAG, "Invalid iBUS packet header");
        return ESP_ERR_INVALID_STATE;
    }

    // 체크섬 검증
    uint16_t calc_cksum = 0xFFFF;
    for (int j = 0; j < 30; j++) {
        calc_cksum -= packet[j];
    }
    uint16_t rx_cksum = packet[30] | (packet[31] << 8);

    if (calc_cksum != rx_cksum) {
        ESP_LOGD(TAG, "iBUS checksum mismatch");
        return ESP_ERR_INVALID_CRC;
    }

    // 채널 데이터 파싱 (14 채널)
    for (int ch = 0; ch < IBUS_CHANNELS; ch++) {
        rc_values_[ch] = packet[2 + ch * 2] | (packet[3 + ch * 2] << 8);
    }

    ESP_LOGD(TAG, "iBUS packet parsed successfully");
    return ESP_OK;
}

void Flysky::flysky_rx_task(void* arg) {
    Flysky* sensor = static_cast<Flysky*>(arg);

    uint8_t temp_buf[128];
    uint8_t packet[IBUS_PACKET_SIZE];
    int p_idx = 0;
    uart_event_t event;

    while (true) {
        if (xQueueReceive(sensor->data_queue_, (void*)&event, portMAX_DELAY) == pdPASS) {
            if (event.type == UART_DATA && event.size > 0) {
                int len = uart_read_bytes(sensor->port_, temp_buf, sizeof(temp_buf), 0);

                for (int i = 0; i < len; i++) {
                    uint8_t b = temp_buf[i];

                    // 상태 머신 기반 동기화
                    if (p_idx == 0) {
                        if (b == 0x20) {
                            packet[p_idx++] = b;
                        }
                    } else if (p_idx == 1) {
                        if (b == 0x40) {
                            packet[p_idx++] = b;
                        } else {
                            p_idx = 0;  // 동기화 실패 시 리셋
                        }
                    } else {
                        packet[p_idx++] = b;
                        if (p_idx == IBUS_PACKET_SIZE) {
                            // 패킷 파싱 및 처리
                            if (sensor->parse_ibus_packet(packet, IBUS_PACKET_SIZE) == ESP_OK) {
                                rc_data_t rc_data{};
                                const uint16_t *channel = sensor->get_all_channels();
                                rc_data.roll    = (channel[0] - 1500) * 0.2f;   // 스틱 입력을 -100 ~ 100 범위로 매핑
                                rc_data.pitch   = (channel[1] - 1500) * 0.2f;
                                rc_data.yaw     = (channel[3] - 1500) * 0.2f;

                                rc_data.throttle= (channel[2] - 1000) * 0.1f;   // 스로틀은 0 ~ 100 범위로 매핑
                                
                                rc_data.aux1    = (channel[4] - 1000) * 0.1f;   // 보조 채널은 0 ~ 100 범위로 매핑 (예: 스위치 조작 등)
                                rc_data.aux2    = (channel[5] - 1000) * 0.1f;
                                rc_data.aux3    = (channel[6] - 1000) * 0.1f;
                                rc_data.aux4    = (channel[7] - 1000) * 0.1f;
                                rc_data.type     = RC_FLYSKY;
                                Controller::SharedDataManager::getInstance().publish_data<Controller::Data_type::DT_RC_DATA>(rc_data);
                                Controller::SharedDataManager::getInstance().set_rc_updated(true); // RC 데이터 업데이트 플래그 세트

                                // ctrl.roll     = (ibus.channels[0] - 1500) * 0.06f;  // +/- 30도 범위
                                // ctrl.pitch    = (ibus.channels[1] - 1500) * 0.06f;
                                // ctrl.throttle = (ibus.channels[2] - 1000) / 1000.0f; // 0.0 ~ 1.0
                                // ctrl.yaw      = (ibus.channels[3] - 1500) * 0.1f;
                                // ctrl.arming   = (ibus.channels[4] > 1500);           // 스위치 기준

                                // ESP_LOGD(TAG, "iBUS channels: %u %u %u %u",
                                //         rc_data.roll, rc_data.pitch,
                                //         rc_data.throttle, rc_data.yaw);
                            } else {
                                ESP_LOGD(TAG, "Failed to parse iBUS packet");
                            }
                            p_idx = 0;
                        }
                    }
                }
            } else if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL) {
                ESP_LOGW(TAG, "UART buffer overflow");
                uart_flush_input(sensor->port_);
                xQueueReset(sensor->data_queue_);
                p_idx = 0;
            }
        }
    }
}

} // namespace Drivers
