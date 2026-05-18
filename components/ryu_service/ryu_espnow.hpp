
#pragma once

#include <array>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h> 
#include <esp_now.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <nvs_flash.h>
#include <lwip/sockets.h>
#include <c_library_v2/common/mavlink.h>


namespace Service
{

class  EspNow{
    private:
        EspNow() = default; 
        ~EspNow() = default;
        static constexpr const char* TAG = "EspNow";
    public:
        static EspNow& get_instance() {
            static EspNow instance; 
            return instance;
        }
        EspNow(const EspNow&) = delete;
        EspNow& operator=(const EspNow&) = delete;
        EspNow(EspNow&&) = delete;
        EspNow& operator=(EspNow&&) = delete;
       
        // 통신은 channel 6번 양쪽을 바추어야한다.
        static inline constexpr uint8_t   ESPNOW_CHANNEL          = 6;
        static inline constexpr uint16_t  ESP_NOW_MAX_LEN         = 290;     // MAVLink v2 최대 페이로드 크기 + 헤더
        static inline constexpr uint8_t   MAVLINK_TX_QUEUE_SIZE   = 60;
        static inline constexpr uint8_t   MAVLINK_RX_QUEUE_SIZE   = 60;

        typedef struct {
            uint8_t buffer[ESP_NOW_MAX_LEN]; // MAVLink 2.0 최대 크기 대응
            uint16_t len;
        } esp_now_data_t; 
        

        int8_t current_rssi=0;
        int8_t noise_floor =0;
        
        QueueHandle_t mavlink_tx_queue = nullptr;
        QueueHandle_t mavlink_rx_queue = nullptr; 

        esp_err_t initialize();
        bool is_initialized(){return _initialized;};
        static void on_esp_now_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);
        static void on_esp_now_send(const wifi_tx_info_t *send_info, esp_now_send_status_t status);
        esp_err_t connect_callback();
        esp_err_t disconnect_callback();
        esp_err_t send_esp_now(const uint8_t *data, size_t len);
        void dispatch_mavlink_msg(mavlink_message_t *msg);
        std::array<uint8_t, 6> get_my_mac_address(void);
        static void mavlink_tx_task(void *pvParameters);
        BaseType_t start_task();
    private:

        TaskHandle_t _task_handle = nullptr;
        // 브리지의 맥 ADDRESS가 바뀌면 이곳을 반드시 수정할것.
        uint8_t bridge_mac[6] = {0x1C,0xDB,0xD4,0xAE,0x82,0x04};
        esp_now_peer_info_t peer_info = {}; 
        bool _initialized = false;
};


} //namespace Service
