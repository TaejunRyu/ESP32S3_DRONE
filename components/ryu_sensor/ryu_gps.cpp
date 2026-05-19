#include "ryu_gps.hpp"

#include <driver/uart.h>
#include <esp_log.h>
//#include "ryu_failsafe.h"


namespace  Sensor
{

esp_err_t  Gps::initialize()
{
    if(_initialized) return ESP_OK;;
    // 1단계: 기본 속도(115200)로 UART 설정
     //ESP_LOGI("GPS", "GPS 초기화 시작..."); // 시작 로그 추가
    uart_config_t gps_cfg = {};
    gps_cfg.baud_rate = GPS_UART_BAUD_RATE;
    gps_cfg.data_bits = UART_DATA_8_BITS;
    gps_cfg.parity = UART_PARITY_DISABLE;
    gps_cfg.stop_bits = UART_STOP_BITS_1;
    gps_cfg.source_clk = UART_SCLK_DEFAULT;

    // 기존에 드라이버가 설치되어 있다면 삭제 후 재설치 (안전용)
    if (uart_is_driver_installed(UART_NUM_1)) {
        uart_driver_delete(UART_NUM_1);
    }
    esp_err_t err = uart_driver_install(UART_NUM_1, 2048, 0, 0, NULL, 0);
    if (err != ESP_OK){
        return err;
    }
    err = uart_param_config(UART_NUM_1, &gps_cfg);
    if (err != ESP_OK){
        return err;
    }

    uart_set_pin(UART_NUM_1, GPS_TX, GPS_RX, -1, -1);
    ESP_LOGI("GPS", "Initialized sucessfully.");

    if (xGpsMutex == nullptr) {
        xGpsMutex = xSemaphoreCreateMutex();
    }
    _initialized = true;
    return ESP_OK;
}

Health Gps::check_gps_health(const gps_data_t &m_gps)
{
    uint32_t now = xTaskGetTickCount();
    
    // 1. 타임아웃 체크
    if (now - m_gps.last_update_tick > pdMS_TO_TICKS(2000)) return Health::LOST;
    if (now - m_gps.last_update_tick > pdMS_TO_TICKS(300))  return Health::STALE;

    // 2. 신호 품질 체크 (u-blox 기준 예시)
    if (m_gps.fixType < 3)  return Health::BAD_QUALITY; // 3D Fix 미만
    if (m_gps.pDOP > 200)   return Health::BAD_QUALITY; // HDOP > 2.0 (단위 0.01)
    if (m_gps.sats < 8)     return Health::BAD_QUALITY; // 위성 수 부족

    return Health::HEALTHY;
}

void Gps::calculate_ubx_checksum(uint8_t *data, int len, uint8_t *ck_a, uint8_t *ck_b)
{
    *ck_a = 0;
    *ck_b = 0;
    for (int i = 0; i < len; i++) {
        *ck_a = *ck_a + data[i];
        *ck_b = *ck_b + *ck_a;
    }
}

/**
 * @brief 현재 GPS 데이터가 신뢰할 수 있는지 통합 판단
 * @return 0: 사용불가, 1: 시간만 유효, 2: 위치/시간 모두 신뢰(추천)
        // 비트 (Bit)	이름	        의미 (1일 때 유효)
        // bit 0	    validDate	    날짜가 유효함 (UTC Date)
        // bit 1	    validTime	    시간가 유효함 (UTC Time)
        // bit 2	    fullyResolved	UTC 시간 초과(leap seconds)까지 완벽히 계산됨
        // bit 3	    validMag	    자기 편각(Magnetic declination) 정보가 유효함
 */
uint8_t Gps::checkDataReliability(ubx_nav_pvt_t *pvt)
{
    // 1. GNSS Fix 상태 확인 (bit 0: gnssFixOK)
    bool fixOk = (pvt->flags & 0x01);
    
    // 2. 시간/날짜 유효성 (bit 0: validDate, bit 1: validTime)
    bool timeOk = (pvt->valid & 0x03) == 0x03;
    
    // 3. 데이터 안정성 (flags2의 confirmedAvai 확인)
    uint8_t confirmed = (pvt->flags2 & 0x03);

    // 4. 수평 정확도 필터 (예: 5m 이내일 때만 '최상'으로 간주)
    bool accurate = (pvt->hAcc < 5000);

    if (fixOk && timeOk && (confirmed >= 1) && accurate) {
        return 2; // 매우 신뢰할 수 있음
    } else if (fixOk && timeOk) {
        return 1; // 사용은 가능하나 오차 가능성 있음
    }
    return 0; // 신뢰 불가
}

void Gps::gps_ubx_mode_task(void *pvParameters)
{
    uint8_t data;
    int state = 0;
    uint8_t payload[100];
    uint8_t payload_idx = 0;
    uint8_t ck_a = 0, ck_b = 0;
    uint16_t msg_len = 0;
 
    auto xLastWakeTime = xTaskGetTickCount();
    const auto xFrequency = pdMS_TO_TICKS(50); // 1 loop에 50ms  x 20번 = 1000ms = 1 second

    auto gps = static_cast<Gps*>(pvParameters);

    gps->share_gps.home_alt = -9999.0f;
    gps->share_gps.last_update_tick = xTaskGetTickCount();
 
    while (true) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        // 버퍼에 쌓인 모든 바이트를 소진할 때까지 반복
        size_t buffered_len;
        uart_get_buffered_data_len(UART_NUM_1, &buffered_len);

        while (buffered_len-- > 0) {
            if (uart_read_bytes(UART_NUM_1, &data, 1, pdMS_TO_TICKS(1)) > 0) {
                //printf("%01x ",data);
                switch (state) {
                    case 0: if (data == 0xB5) state++; break; // Sync Char 1
                    case 1: if (data == 0x62) state++; else state = 0; break; // Sync Char 2
                    case 2: if (data == 0x01) { state++; ck_a = data; ck_b = data; } else state = 0; break; // Class (NAV)
                    case 3: if (data == 0x07) { state++; ck_a += data; ck_b += ck_a; } else state = 0; break; // ID (PVT)                    
                    case 4: // Length L
                        msg_len = data;
                        ck_a += data; ck_b += ck_a; // 추가
                        state++; 
                        break;
                    case 5: // Length H
                        msg_len |= (data << 8);
                        ck_a += data; ck_b += ck_a; // 추가
                        state++; 
                        payload_idx = 0; 
                        break;

                    
                    case 6: // Payload 읽기
                        payload[payload_idx++] = data;
                        ck_a += data; ck_b += ck_a;
                        if (payload_idx >= msg_len) state++;
                        break;
                    case 7: // Checksum A
                        if (data == ck_a) state++; else state = 0;
                        break;
                    case 8: // Checksum B
                        if (data == ck_b) {
                            // 성공! 데이터를 구조체로 복사
                            ubx_nav_pvt_t *pvt = (ubx_nav_pvt_t *)payload;
                            
                            // 만약 같은 시간의 데이터이면 무시....
                            if (gps->share_gps.iTOW == pvt->iTOW){
                                state = 0;
                                break;
                            }
                            // 만약 마지막 복사 시점으로부터 1초 이상 지났다면 'GPS 연결 끊김' 상태로 전송
                            // if (xTaskGetTickCount() - share_gps.last_update_tick > pdMS_TO_TICKS(2000)) {
                            //     xTaskNotify(ERR::xErrorHandle, ERR::ERR_GPS_TIMEOUT, eSetBits);
                            //     share_gps.last_update_tick = xTaskGetTickCount();
                            //     state =0;
                            //     break;
                            // }

                            if (xSemaphoreTake(gps->xGpsMutex, pdMS_TO_TICKS(10)) == pdTRUE) {                                
                                // 위성 및 상태 정보
                                gps->share_gps.sats  = pvt->numSV;
                                gps->share_gps.pDOP  = static_cast<float>(pvt->pDOP) / 100.0f; // UBX는 pDOP을 제공 (Scaling 0.01)
                                gps->share_gps.fixType = pvt->fixType; // 3D Fix 이상일 때 true
                            
                                gps->Gps_status = gps->check_gps_health(gps->share_gps);


                                gps->share_gps.iTOW = pvt->iTOW;
                                // 신뢰도 통합 체크
                                uint8_t status = gps->checkDataReliability(pvt);

                                if (status >= 1) { // 최소 시간/날짜는 유효함
                                    gps->share_gps.date = (pvt->year * 10000) + (pvt->month * 100) + pvt->day;
                                    // ... 시간 저장 ...
                                }

                                if (status == 2) { // 위치 고정 및 안정화 완료
                                    gps->share_gps.lon = (double)pvt->lon / 1e7;
                                    gps->share_gps.lat = (double)pvt->lat / 1e7;
                                    // ... 고도 및 속도 저장 ...
                                }

                                // 1. 날짜 및 시간 (YYYYMMDD 형식 예시)
                                gps->share_gps.date = (pvt->year * 10000) + (pvt->month * 100) + pvt->day;
                                // UTC 시간 (HHMMSS.ss 형식으로 변환)
                                gps->share_gps.utc_time = (float)pvt->hour * 10000.0f + (float)pvt->min * 100.0f + (float)pvt->sec + (pvt->nano / 1000000000.0f);
                                
                                float current_msl_alt = (float)pvt->hMSL ;
                                
                                // 홈 고도가 설정되지 않았고(-9000 미만), 수평 오차가 2m 이내일 때만 평균 계산 시작
                                if (gps->share_gps.home_alt < -9000.0f && pvt->hAcc < 2000) {
                                    gps->share_gps.home_alt = current_msl_alt;
                                    //ESP_LOGI("GPS", "Home Alt Set: %.2f m (hAcc: %lu mm)", share_gps.home_alt, pvt->hAcc);
                                }

                                // 5. 속도 정보 (mm/s -> cm/s 변환)
                                // PVT 메시지는 velN, velE, velD를 mm/s로 제공하므로 10으로 나눕니다.
                                gps->share_gps.velN = static_cast<int16_t>(pvt->velN / 10); // cm/s
                                gps->share_gps.velE = static_cast<int16_t>(pvt->velE / 10); // cm/s
                                gps->share_gps.velD = static_cast<int16_t>(pvt->velD / 10); // cm/s
                                // 지면 속도 (mm/s -> cm/s)
                                gps->share_gps.gSpeed = static_cast<uint16_t>(pvt->gSpeed / 10); 
                                // 이동 방향 (Degree * 10^-5 -> Centi-Degree)
                                // 예: 180.50도 -> 18050
                                gps->share_gps.headMot = static_cast<uint16_t>(pvt->headMot / 1000); 
                                // 6. 자기 편차 (Magnetic Variation)
                                // UBX-NAV-PVT의 magDec 값을 사용 (Scaling 0.01)
                                gps->share_gps.magDec = static_cast<float>(pvt->magDec) * 0.01f;
                                gps->share_gps.vAcc  = pvt->vAcc;
                                gps->share_gps.hAcc  = pvt->hAcc;
                                gps->share_gps.sAcc  = pvt->sAcc;
                                gps->share_gps.height = pvt->height;
                                gps->share_gps.hMSL  = pvt->hMSL;

                                //check_gps_health(share_gps);
                                gps->share_gps.last_update_tick = xTaskGetTickCount();
                                xSemaphoreGive(gps->xGpsMutex);
                            }
                                //printf("Lat: %ld, Lon: %ld, Sats: %d\n", pvt->lat, pvt->lon, pvt->numSV);
                        }
                        state = 0;
                        break;
                }
            }
        }
    }
}

BaseType_t Gps::start_task()
{
    auto res = xTaskCreatePinnedToCore(gps_ubx_mode_task, "gps", 4096, this, 5,&_task_handle, 0);
    if (res != pdPASS) ESP_LOGE(TAG, "❌ 2.Gps Task is Failed! code: %d", res);
    else ESP_LOGI(TAG, "✓ 2.Gps Task is passed... ");
    return res;
}


}

/*
NMEA    의미
----------------------------------------------------------------------------
GPGGA	글로벌 포지셔닝 시스템 고정 데이터(시간, 위치, 고정 유형 데이터)
GPRMC	시간, 날짜, 위치, 코스 및 속도 데이터
GPGLL	지리적 위치, 위도, 경도
GPVTG	지면에 대한 코스 및 속도 정보
GPGSA	GPS 수신기 작동 모드, 위치 솔루션에 사용된 위성 및 DOP 값.
GPGSV	위성 ID 번호, 고도, 방위각 및 SNR 값을 볼 수 있는 GPS 위성의 수입니다.
GPMSS	무선 신호 수신기의 신호 대 잡음비, 신호 강도, 주파수 및 비트 전송률.
GPTRF	대중교통 수정 데이터
GPSTN	다중 데이터 ID
GPXTE	크로스 트랙 오류, 측정됨
GPZDA	날짜 및 시간(PPS 타이밍 메시지, PPS에 동기화됨).
----------------------------------------------------------------------------\
*/
// 아래 내용은 언제 사용될지 모르니 보관중...

// // 기동시 현재의 고도를 알아야지 상대적 고도를 알기 때문에     
// // 현재의 고도를 저장... 나중에 나오는 고도에서 이값을 빼준다.
// // 초기값 (설정 전임을 표시)

// // 전역 또는 정적 변수로 선언
// static double filtered_lat = 0;
// static double filtered_lon = 0;

// static void apply_filter(double new_lat, double new_lon) {
//     if (filtered_lat == 0) { // 초기값 설정
//         filtered_lat = new_lat;
//         filtered_lon = new_lon;
//     } else {
//         // 0.9와 0.1 비율은 테스트하며 조정 (숫자가 작을수록 새 값에 민감)
//         filtered_lat = (filtered_lat * 0.8) + (new_lat * 0.2);
//         filtered_lon = (filtered_lon * 0.8) + (new_lon * 0.2);
//     }
// } 

// void gps_nmea_mode_task(void *pv) {

//     uint8_t data[256];
//     char line[128];
//     int line_pos = 0;
   
//     auto xLastWakeTime = xTaskGetTickCount();
//     const auto xFrequency = pdMS_TO_TICKS(50); // 1 loop에 50ms  x 20번 = 1000ms = 1 second
//     // 초기 home의 고도를 설정하기위함.....
//     share_gps.home_alt = -9999.0f;
    
//     while (true) [[likely]]{
//         vTaskDelayUntil(&xLastWakeTime, xFrequency);
//         // 1. 버퍼에 쌓인 데이터 길이를 확인 후 한꺼번에 읽기
//         size_t length = 0;

//         uart_get_buffered_data_len(UART_NUM_1, &length);
//         if (length > 0)[[likely]] {

//             const int to_read = std::min<size_t>(length, sizeof(data));
//             int read_len = uart_read_bytes(UART_NUM_1, data, to_read, pdMS_TO_TICKS(10));
            
//             for (int i = 0; i < read_len; i++) {
//                 const char c = static_cast<char>(data[i]);
//                 //printf("%c", c);
//                 // 문장 시작 
//                 if (c == '$') { line_pos = 0; }                
//                 std::string_view sv(line);
                
//                 // 문장 끝 , line_pos는 충분한 길이를 보장하면 실행하라는 것...
//                 if ((c == '\n' || c == '\r') && (line_pos > 10 )) [[unlikely]] { 
//                     line[line_pos] = '\0';       
                    
//                     //ESP_LOGI("GPS", "line_pos : %d  , 데이터 : %s",line_pos,line);
//                        // 1. VTG 문장 처리 (속도 정보)
//                     if (sv.contains("VTG")) {
//                         float course_true = 0.0f, speed_knots = 0.0f, speed_kmh = 0.0f;
//                         char mode = 'N';
//                         // $GPVTG,코스,T,,M,속도(knots),N,속도(km/h),K,모드*체크섬
//                         int res = sscanf(line, "$%*[^,],%f,T,,M,%f,N,%f,K,%c", 
//                                         &course_true, &speed_knots, &speed_kmh, &mode);
//                         if (res >= 3) {
//                             // share_gps.speed_kmh = speed_kmh;
//                             // share_gps.course = course_true;
//                             // ESP_LOGI("GPS", "VTG -> Speed: %.2f km/h, Course: %.2f", speed_kmh, course_true);
//                         }
//                     }

//                     // 2. RMC 문장 처리 (시간, 날짜, 위치 통합)
//                     if (sv.contains("RMC")) {
//                         float utc_time = 0.0f;
//                         char status = 'V'; // A=유효, V=경고
//                         double raw_lat = 0.0, raw_lon = 0.0;
//                         char ns = 'N', ew = 'E';
//                         float speed_knots = 0.0f;
//                         float cog_f = 0.0f; // Course Over Ground
//                         int date = 0;
//                         float variation = 0.0f; // 자기 편차 (Magnetic Variation)
//                         char var_dir = 'E';

//                         // $GPRMC,시간,상태,위도,N,경도,E,속도,방향,날짜,자기편차,편차방향*체크섬
//                         // %f, %c 등을 이용해 순서대로 파싱합니다.
//                         int res = sscanf(line, "$%*[^,],%f,%c,%lf,%c,%lf,%c,%f,%f,%d,%f,%c",
//                                         &utc_time, &status, &raw_lat, &ns, &raw_lon, &ew, 
//                                         &speed_knots, &cog_f, &date, &variation, &var_dir);

//                         if (res >= 2 && status == 'A') {
//                             // 1. 기본 시간 및 날짜
//                             share_gps.utc_time = utc_time;
//                             share_gps.date = date;

//                             // 2. 위치 데이터 (DD 변환)
//                             auto nmea_to_dd = [](double val, char dir) -> double {
//                                 double deg = std::floor(val / 100.0);
//                                 double min = std::fmod(val, 100.0);
//                                 double dd = deg + (min / 60.0);
//                                 return (dir == 'S' || dir == 'W') ? -dd : dd;
//                             };
//                             share_gps.lat = nmea_to_dd(raw_lat, ns);
//                             share_gps.lon = nmea_to_dd(raw_lon, ew);

//                             // 3. 속도 및 방향 처리 (정수형 변환)
//                             // 1 knot = 51.4444 cm/s
//                             float speed_cms_f = speed_knots * 51.4444f;
//                             share_gps.gSpeed = (uint16_t)speed_cms_f;
//                             share_gps.headMot  = (uint16_t)(cog_f * 100.0f);

//                             // 4. NED 속도 성분 계산 (VelN, VelE)
//                             float rad = cog_f * (M_PI / 180.0f);
//                             share_gps.velN = (int16_t)(speed_cms_f * cosf(rad));
//                             share_gps.velE = (int16_t)(speed_cms_f * sinf(rad));

//                             // 5. 자기 편차 적용 (필요 시)
//                             float mag_var = (var_dir == 'W') ? -variation : variation;
//                             share_gps.magDec  = mag_var;

//                             share_gps.fixType = 3;
                            
//                             ESP_LOGI("GPS", "RMC Parsed: Time: %.0f, Spd: %u cm/s, Cog: %u cd", 
//                                       utc_time, share_gps.gSpeed, share_gps.headMot);
//                         }
//                     }


//                     // 전역 또는 정적 변수로 이전 값 저장
//                     static float last_alt = 0.0f;
//                     static uint32_t last_alt_time = 0;
//                     static float filtered_velD = 0.0f;
//                     // GGA 문장 필터링 ($GPGGA, $GNGGA 등 대응)
//                     if (sv.contains("GGA")) {
//                         double raw_lat = 0.0, raw_lon=0.0;
//                         char ns ='N', ew='E';
//                         int fix_q=0, sats=0;
//                         float alt =0.0f;
//                         float hdop =0.0f;
//                         char header[12]; // $GPGGA 저장용

//                         // 포맷 설명: $GPGGA,시간(skip),위도,N/S,경도,E/W,Fix,위성수,HDOP,고도
//                         int res = sscanf(line, "$%[^,],%*f,%lf,%c,%lf,%c,%d,%d,%f,%f", 
//                                         header, &raw_lat, &ns, &raw_lon, &ew, &fix_q, &sats, &hdop, &alt);

//                         // 조건문: 450급 기체라면 HDOP 1.5 이하는 아주 안전한 수치입니다.
//                         if (res < 9 || fix_q < 1 || hdop > 1.5f || sats < 7) [[unlikely]] {
//                             line_pos = 0;
//                             continue;
//                         }

//                         // --- VelD (수직 속도) 계산 시작 ---
//                         uint32_t now = xTaskGetTickCount();
//                         if (last_alt_time != 0) {
//                             float dt = (now - last_alt_time) * portTICK_PERIOD_MS / 1000.0f; // 초 단위 변환
//                             if (dt > 0.0f) {
//                                 // 현재 수직 속도 (m/s) = (현재고도 - 이전고도) / 시간차
//                                 // Down Velocity이므로 고도가 높아지면 (-)값이 되어야 함 (NED 좌표계)
//                                 float instant_velD_ms = -(alt - last_alt); 
                                
//                                 // 저주파 필터 적용 (0.7:0.3 비율로 부드럽게)
//                                 filtered_velD = (filtered_velD * 0.7f) + (instant_velD_ms * 0.3f);
                                
//                                 // cm/s 단위로 변환하여 저장
//                                 share_gps.velD = (int16_t)(filtered_velD * 100.0f);
//                             }
//                         }
//                         last_alt = alt;
//                         last_alt_time = now;
//                         // --- VelD 계산 끝 ---


//                         // 2. 홈 고도(0점) 설정 (처음 한 번만 실행)
//                         // 처음의 고도값이 -9000보다 작으면 아직 설정되지 않은 것으로 간주하고 평균을 내서 설정 (잡음 제거용)
//                         if (share_gps.home_alt < -9000.0f) {
//                             static float alt_sum = 0.0f;
//                             static int alt_count = 0;
//                             alt_sum += alt;
//                             alt_count++;
//                             if (alt_count >= 10) {
//                                 share_gps.home_alt = alt_sum / alt_count;
//                                 ESP_LOGI("GPS", "홈 고도 설정 완료: %.2f m", share_gps.home_alt);
//                             }
//                         }
//                         apply_filter(raw_lat,raw_lon);
//                         // 람다 함수를 이용한 도(DD) 변환 캡슐화
//                         auto nmea_to_dd = [](double val, char dir) -> double {
//                             double deg = std::floor(val / 100.0);
//                             double min = std::fmod(val, 100.0);
//                             double dd = deg + (min / 60.0);
//                             return (dir == 'S' || dir == 'W') ? -dd : dd;
//                         };

//                         share_gps.lat = nmea_to_dd(filtered_lat, ns);
//                         share_gps.lon = nmea_to_dd(filtered_lon, ew);
                        
//                         // 3. 데이터 업데이트
//                         // 아직 평균치를 확정하지 않았으면 고도는 그냥 원래값, 평균이 확정되면 상대고도로 변환
//                         if (share_gps.home_alt > -9000.0f)
//                             share_gps.alt   = alt - share_gps.home_alt; 
//                         share_gps.sats  = sats;
//                         share_gps.pDOP  = hdop;
//                         share_gps.fixType = 3;
//                         ESP_LOGI("GPS", "성공! Lat: %.6f, Lon: %.6f, Sats: %d", share_gps.lat, share_gps.lon, sats);                                                                   
//                     }
//                     line_pos = 0;                    
//                 }else if (line_pos < sizeof(line) - 1) 
//                 {
//                     line[line_pos++] = c;
//                 }
//             }
//         }
        
//     }
// }
