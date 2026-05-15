#include "Icm20948.hpp"

#include <esp_log.h>
#include <rom/ets_sys.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>

namespace Sensor{

static const char* TAG = "ICM20948";

ICM20948::ICM20948(spi_host_device_t spiHost, gpio_num_t csPin, int spiClockHz) 
    : _csPin(csPin), _currentBank(0xFF) {
    
    // SPI 장치 설정 및 등록
    spi_device_interface_config_t devCfg = {};
    devCfg.clock_speed_hz = spiClockHz;
    devCfg.mode = 3;  // SPI Mode 3 (ICM20948 규격)
    devCfg.spics_io_num = _csPin;
    devCfg.queue_size = 7;
    
    spi_bus_add_device(spiHost, &devCfg, &_spiHandle);
}

ICM20948::~ICM20948() {
    spi_bus_remove_device(_spiHandle);
}

void ICM20948::selectBank(uint8_t bank) {
    if (_currentBank == bank) return;
    // 모든 뱅크의 0x7F 레지스터는 REG_BANK_SEL 임
    uint8_t regAddr = 0x7F;
    uint8_t data = (bank << 4);
    
    spi_transaction_t t = {};
    t.length = 8;
    t.tx_buffer = &data;
    // 저수준 SPI 전송 코드 수행...
    _currentBank = bank;
}

bool ICM20948::initialize() {
    // 1. 디바이스 소프트 리셋
    selectBank(0);
    writeRegister(0x06, 0x80); // PWR_MGMT_1 리셋
    vTaskDelay(pdMS_TO_TICKS(100));
    writeRegister(0x06, 0x01); // 자동 시계원 선택
    
    // 2. WHO_AM_I 검증 (ICM20948의 기본 ID는 0xEA)
    uint8_t whoAmI = 0;
    readRegisters(0x00, &whoAmI, 1);
    if (whoAmI != 0xEA) {
        ESP_LOGE(TAG, "Failed to connect to ICM20948! ID: 0x%02X", whoAmI);
        return false;
    }

    // 3. 지자계 센서(AK09916) 연동을 위한 내장 I2C Master 활성화
    initAuxI2CMaster();

    return true;
}

void ICM20948::initAuxI2CMaster() {
    selectBank(0);
    writeRegister(0x03, 0x20); // USER_CTRL: I2C_MST_EN 활성화
    
    selectBank(3);
    writeRegister(0x01, 0x0D); // I2C_MST_CTRL: 400kHz 세팅
    
    // AK09916 리셋 및 연속 측정 모드 진입 명령 주입
    writeAK09916(0x31, 0x01); // CNTL2: 100Hz 연속 측정 모드
    
    // ICM20948이 주기적으로 AK09916의 데이터를 자동으로 읽어 가도록 Slave0 자동 갱신 지정
    writeRegister(0x03, 0x8C); // I2C_SLV0_ADDR: Read 모드 + AK09916 주소(0x0C)
    writeRegister(0x04, 0x11); // I2C_SLV0_REG: AK09916 데이터 시작 레지스터 (HXL)
    writeRegister(0x05, 0x89); // I2C_SLV0_CTRL: Enable + 9바이트 읽기 (ST1부터 ST2까지 한 번에 읽어야 락 해제됨)
}

bool ICM20948::updateSample(ImuData& sample) {
    uint8_t rawBuf[14 + 9]; // Accel(6) + Gyro(6) + Temp(2) + Mag(9)
    
    // 1. 가속도/자이로 읽기 (Bank 0)
    selectBank(0);
    readRegisters(0x2D, rawBuf, 14); // ACCEL_XOUT_H 부터 읽기 시작
    
    // 2. 자동 파싱 및 환산 (Vector3D 연산자 오버로딩 구조 활용)
    int16_t ax = (rawBuf[0] << 8) | rawBuf[1];
    int16_t ay = (rawBuf[2] << 8) | rawBuf[3];
    int16_t az = (rawBuf[4] << 8) | rawBuf[5];
    
    // 민감도 상수를 곱해 g단위 또는 m/s^2 단위의 float 벡터로 변환
    sample.acc = Vector3f(ax / 16384.0f, ay / 16384.0f, az / 16384.0f); // ±2g 기준
    
    // 3. I2C Slave0에 매핑되어 상시 자동 갱신 중인 Mag 데이터 읽기
    readRegisters(0x3B, &rawBuf[14], 9); // EXT_SLV_SENS_DATA_00 레지스터 영역
    
    // 시간 정보 기입
    sample.timestamp = esp_timer_get_time();
    return true;
}

} //namespace Sensor