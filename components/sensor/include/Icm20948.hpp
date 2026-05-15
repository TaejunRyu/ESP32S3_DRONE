// components/hardware/include/ICM20948.hpp
#pragma once
#include "ImuInterface.hpp"
#include "driver/spi_master.h"
#include "driver/gpio.h"

class ICM20948 : public ImuInterface{
private:
    spi_device_handle_t _spiHandle;    
    gpio_num_t _csPin;
    
    // ICM20948의 레지스터 뱅크 관리를 위한 현재 상태 변수
    uint8_t _currentBank;

    // 내부 SPI 통신 저수준 함수
    void selectBank(uint8_t bank);
    void writeRegister(uint8_t reg, uint8_t data);
    void readRegisters(uint8_t reg, uint8_t* buffer, size_t len);
    
    // 내부 AK09916 지자계 센서 설정용 함수 (ICM20948의 Aux I2C Master 사용)
    void initAuxI2CMaster();
    void writeAK09916(uint8_t reg, uint8_t data);

public:
    /**
     * @brief ICM20948 생성자
     * @param spiHost ESP32-S3 SPI 호스트 (예: SPI2_HOST)
     * @param csPin Chip Select GPIO 번호
     */
    ICM20948(spi_host_device_t spiHost, gpio_num_t csPin, int spiClockHz = 7000000); // Max 7MHz
    ~ICM20948() override;

    bool initialize() override;
    bool updateSample(ImuData& sample) override;
};
