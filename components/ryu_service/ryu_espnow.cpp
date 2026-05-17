#include "ryu_espnow.hpp"

#include <array>
#include <esp_mac.h>


namespace Service
{   

esp_err_t EspNow::initialize()
{
    if(_initialized) return ESP_OK;
    mavlink_tx_queue   = xQueueCreate(MAVLINK_TX_QUEUE_SIZE,       sizeof(esp_now_data_t));
    mavlink_rx_queue   = xQueueCreate(MAVLINK_RX_QUEUE_SIZE,       sizeof(esp_now_data_t));

    //-----------WIFI 초기화----------
    // NVS 초기화
    //esp_err_t ret = nvs_flash_init();
    // if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    //     nvs_flash_erase();
    //     nvs_flash_init();
    // }

    // TCP/IP 스택 초기화
    ESP_ERROR_CHECK(esp_netif_init());
    //ESP_ERROR_CHECK(esp_event_loop_create_default());

    // WiFi STA 모드 초기화
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    
    // 채널 설정 (브릿지과 동일한 채널)
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    ESP_LOGI(TAG, "WiFi STA mode initialization complete, Channel: %d", ESPNOW_CHANNEL);


    // -----------ESP-NOW 초기화-----------
    ESP_ERROR_CHECK(esp_now_init());

    // Bridge peer 등록
    memcpy(peer_info.peer_addr, bridge_mac, ESP_NOW_ETH_ALEN);
    peer_info.channel = ESPNOW_CHANNEL;
    peer_info.encrypt = false;
    peer_info.ifidx = WIFI_IF_STA;

    esp_err_t add_peer_err = esp_now_add_peer(&peer_info);
    if (add_peer_err != ESP_OK) {
        ESP_LOGE(TAG, "Bridge peer 등록 실패: %s", esp_err_to_name(add_peer_err));
        return add_peer_err;
    }         
    
    // STA(드론 방향)는 드론의 언어인 LR 모드로! (_LR은 독불장군이다. 다른 프로토콜과 같이 사용 못함.)==> (125Kbps ~ 500kbps)
    //ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR));

    {  // _LR 모드 잠시 보류하고 WIFI_PHY_RATE_2M_S 로지정함.
        ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N));
        // 1. 설정 구조체 준비
        esp_now_rate_config_t rate_config = {};
        rate_config.phymode = WIFI_PHY_MODE_11B;      // 802.11b 모드 (거리 확보)
        //rate_config.rate = WIFI_PHY_RATE_2M_S;         // 2Mbps Short Preamble
        rate_config.rate = WIFI_PHY_RATE_6M;            // 2Mbps Short Preamble

        // 해당 설정시 _LR모드와 공존할수 없다.
        esp_now_set_peer_rate_config(bridge_mac, &rate_config); 
    }
    ESP_LOGI(TAG, "Bridge peer registration successful: %02x:%02x:%02x:%02x:%02x:%02x",
                 bridge_mac[0], bridge_mac[1], bridge_mac[2],
                 bridge_mac[3], bridge_mac[4], bridge_mac[5]);    
    ESP_LOGI(TAG,"Initialized successfully.");
    _initialized = true;
    return ESP_OK;
}


void EspNow::on_esp_now_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
if (len <= 0 || data == nullptr || recv_info == nullptr) return;

    auto& espnow = Service::EspNow::get_instance();
    // 실시간 정보를 구해서 bridge로 보낸다.
    espnow.current_rssi = recv_info->rx_ctrl->rssi;
    espnow.noise_floor  = recv_info->rx_ctrl->noise_floor;   

    if (espnow.mavlink_rx_queue != NULL) {
        Service::EspNow::esp_now_data_t pkt;
        pkt.len = len;
        // 실제 수신된 'data'를 pkt.data로 복사해야 함!
        memcpy(pkt.buffer, data, len); 

        // 큐에 넣기 (ISR이 아니므로 xQueueSend 사용 가능, 대기시간 0)
        if (xQueueSend(espnow.mavlink_rx_queue, &pkt, 0) != pdTRUE) {
            // 큐가 꽉 차서 버려지는 경우만 로그 출력
            // ESP_LOGW("ESP_NOW", "RX Queue Full"); 
        }
    }
}

void EspNow::on_esp_now_send(const wifi_tx_info_t *send_info, esp_now_send_status_t status)
{
    if(status == ESP_NOW_SEND_SUCCESS){
        // 성공시 
    }else{
        // 실패시
    } 
    //ESP_LOGI(TAG, "Send status=%s", status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

esp_err_t EspNow::connect_callback()
{
    //callback 함수가 데이터를 받아서 mavlink의 데이터 처리를 하기 때문에 콜백함수를 반드시 등록해야함.
    esp_err_t err = esp_now_register_recv_cb(on_esp_now_recv);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "recv callback register failed: %s", esp_err_to_name(err));
        return err;
    }
    err = esp_now_register_send_cb(on_esp_now_send);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "send callback register failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "EspNow callback registered.");    
    return err;
}

esp_err_t EspNow::disconnect_callback(){
    esp_err_t err = esp_now_unregister_recv_cb();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "recv callback unregister failed: %s", esp_err_to_name(err));
        return err;
    }    
    err = esp_now_unregister_send_cb();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "send callback unregister failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "EspNow callback unregistered.");    
    return err;
}

esp_err_t EspNow::send_esp_now(const uint8_t *data, size_t len)
{
    if (data == nullptr || len == 0) 
        return ESP_ERR_INVALID_ARG;
    
    // 1. ESP-NOW 최대 크기 체크 (안전 장치)
    if (len > 250) {
        ESP_LOGE(TAG, "Packet too large: %d", len);
        return ESP_ERR_INVALID_SIZE;
    }

    // 2. 즉시 전송 시도
    esp_err_t err = esp_now_send(peer_info.peer_addr, data, len);

    // 3. 에러 처리 최적화
    if (err != ESP_OK) {
        if (err == ESP_ERR_ESPNOW_NO_MEM) {
            // 큐가 가득 찼을 때: 아주 잠깐 쉬고 재시도하거나 호출측에 알림
            // 파라미터 전송 루프라면 여기서 vTaskDelay(1)을 주는 것이 도움됨
            return err; 
        }
        if (err == ESP_ERR_ESPNOW_NOT_FOUND) {
            // 상대방 주소가 등록 안 됨
            return err;
        }
    }
    return err;
}

void EspNow::dispatch_mavlink_msg(mavlink_message_t *msg)
{
    if ( mavlink_tx_queue == nullptr){
        return;
    }
    esp_now_data_t pkt;
    pkt.len = mavlink_msg_to_send_buffer(pkt.buffer, msg);

    // ISR(콜백) 여부에 따른 안전한 큐 삽입
    if (xPortInIsrContext()) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(mavlink_tx_queue, &pkt, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    } else {
        xQueueSend(mavlink_tx_queue, &pkt, 0/*portMAX_DELAY */);
    }
}

std::array<uint8_t, 6> EspNow::get_my_mac_address(void)
{
    std::array<uint8_t, 6> mac;
    // std::array의 내부 배열 주소를 직접 넘겨줌 (.data() 사용)
    esp_read_mac(mac.data(), ESP_MAC_WIFI_STA);
    return mac;
}

void EspNow::mavlink_tx_task(void *pvParameters)
{
    auto espnow = static_cast<Service::EspNow*>(pvParameters);

    esp_now_data_t tx_pkt;
    if(espnow->mavlink_tx_queue ==NULL) return;

    while (true) {
        // 큐에서 데이터 대기 (데이터가 올 때까지 Blocked 상태로 CPU 점유 0)
        if (xQueueReceive(espnow->mavlink_tx_queue, &tx_pkt, portMAX_DELAY) == pdPASS) {
            // 2. 정상 전송
            // ESP_OK : succeed - 
            // ESP_ERR_ESPNOW_NOT_INIT : ESPNOW is not initialized - 
            // ESP_ERR_ESPNOW_ARG : invalid argument - 
            // ESP_ERR_ESPNOW_INTERNAL : internal error - 
            // ESP_ERR_ESPNOW_NO_MEM : out of memory, when this happens, you can delay a while before sending the next data - 
            // ESP_ERR_ESPNOW_NOT_FOUND : peer is not found - 
            // ESP_ERR_ESPNOW_IF : current Wi-Fi interface doesn't match that of peer - 
            // ESP_ERR_ESPNOW_CHAN: current Wi-Fi channel doesn't match that of peer
            esp_err_t result = esp_now_send(espnow->bridge_mac, tx_pkt.buffer, tx_pkt.len);
            if(result == ESP_ERR_ESPNOW_NO_MEM){
                // 버퍼가 찰 때까지 너무 빨리 보낸 것이므로 잠시 쉬어줍니다.
                vTaskDelay(pdMS_TO_TICKS(1)); 
                // 재시도 로직 실행
                esp_now_send(espnow->bridge_mac, tx_pkt.buffer, tx_pkt.len);
            } else if (result != ESP_OK) {
                // 그 외 에러는 기록 후 패킷 폐기 (무한 루프 방지)
                ESP_LOGD(TAG, "Send failed: %s", esp_err_to_name(result));
            }
        }
    }    
}

BaseType_t EspNow::start_task()
{
    auto res = xTaskCreatePinnedToCore(mavlink_tx_task, "mavlink_tx_task", 4096, this, 15,&_task_handle, 0);
    if (res != pdPASS){ 
        ESP_LOGE(TAG, "❌ 0.EspNow TX Task is Failed! code: %d", res);
    }
    else{ 
        ESP_LOGI(TAG, "✓ 0.EspNow TX Task is passed... ");
    }
    return res;
}   

} // namespace WIFI
