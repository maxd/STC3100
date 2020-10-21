//
// Created by Maxim Dobryakov on 19/10/2020.
//

#include "STC3100.h"
#include "STC3100_Utils.h"


#include <esp_log.h>

#define STC3100_LOG_TAG "STC3100"

#define STC3100_16_BIT_MASK 0xFFFF
#define STC3100_14_BIT_MASK 0x3FFF
#define STC3100_12_BIT_MASK 0x0FFF

#define CHECK_READ_RESULT(result) \
    if (result != ESP_OK) { \
        ESP_LOGE(STC3100_LOG_TAG, "STC3100_I2CRead return: %d", result); \
        return false; \
    }

#define CHECK_WRITE_RESULT(result) \
    if (result != ESP_OK) { \
        ESP_LOGE(STC3100_LOG_TAG, "STC3100_I2CWrite return: %d", result); \
        return false; \
    }

void STC3100_I2CInit(int sclGpio, int sdaGpio, int i2cFrequency, i2c_port_t i2cPort) {
    i2c_config_t config;
    config.mode = I2C_MODE_MASTER;
    config.scl_io_num = sclGpio;
    config.scl_pullup_en = GPIO_PULLUP_ENABLE; //TODO: external pull-ups resistors (see off. doc)?
    config.sda_io_num = sdaGpio;
    config.sda_pullup_en = GPIO_PULLUP_ENABLE; //TODO: external pull-ups resistors (see off. doc)?
    config.master.clk_speed = i2cFrequency;
    ESP_ERROR_CHECK(i2c_param_config(i2cPort, &config));

    ESP_ERROR_CHECK(i2c_driver_install(i2cPort, config.mode, 0, 0, 0));
}

void STC3100_I2CDeInit(i2c_port_t i2cPort) {
    ESP_ERROR_CHECK(i2c_driver_delete(i2cPort));
}

esp_err_t STC3100_I2CRead(i2c_port_t i2cPort, uint8_t *buffer, size_t bufferLength, TickType_t ticksToWait) {
    i2c_cmd_handle_t command = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(command));
    ESP_ERROR_CHECK(i2c_master_write_byte(command, (STC3100_ADDRESS << 1) | I2C_MASTER_READ, true));
    if (bufferLength > 1) {
        ESP_ERROR_CHECK(i2c_master_read(command, buffer, bufferLength - 1, I2C_MASTER_ACK));
    }
    ESP_ERROR_CHECK(i2c_master_read_byte(command, buffer + bufferLength - 1, I2C_MASTER_NACK));
    ESP_ERROR_CHECK(i2c_master_stop(command));

    esp_err_t result = i2c_master_cmd_begin(i2cPort, command, ticksToWait);
    i2c_cmd_link_delete(command);
    return result;
}

esp_err_t STC3100_I2CWrite(i2c_port_t i2cPort, uint8_t *buffer, size_t bufferLength, TickType_t ticksToWait) {
    i2c_cmd_handle_t command = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(command));
    ESP_ERROR_CHECK(i2c_master_write_byte(command, (STC3100_ADDRESS << 1) | I2C_MASTER_WRITE, true));
    ESP_ERROR_CHECK(i2c_master_write(command, buffer, bufferLength, true));
    ESP_ERROR_CHECK(i2c_master_stop(command));

    esp_err_t result = i2c_master_cmd_begin(i2cPort, command, ticksToWait);
    i2c_cmd_link_delete(command);
    return result;
}

void STC3100_InitModeRegister(STC3100_ModeRegister *modeRegister) {
    modeRegister->selectExternalClock = false;
    modeRegister->gasGaugeResolution = STC3100_GasGaugeResolution14bits;
    modeRegister->gasGaugeCalibration = false;
    modeRegister->gasGaugeEnabled = false;
    modeRegister->unused = 0;
}

bool STC3100_ReadModeRegister(i2c_port_t i2cPort, STC3100_ModeRegister *modeRegister, TickType_t ticksToWait) {
    uint8_t registerAddress = STC3100_REG_MODE;
    CHECK_WRITE_RESULT(STC3100_I2CWrite(i2cPort, &registerAddress, sizeof(registerAddress), ticksToWait))
    CHECK_READ_RESULT(STC3100_I2CRead(i2cPort, (uint8_t *) modeRegister, sizeof(STC3100_ModeRegister), ticksToWait))

    return true;
}

bool STC3100_WriteModeRegister(i2c_port_t i2cPort, STC3100_ModeRegister *modeRegister, TickType_t ticksToWait) {
    uint8_t buffer[] = { STC3100_REG_MODE, ((uint8_t *)modeRegister)[0] };
    CHECK_WRITE_RESULT(STC3100_I2CWrite(i2cPort, buffer, sizeof(buffer), ticksToWait))

    return true;
}

void STC3100_InitControlRegister(STC3100_ControlRegister *controlRegister) {
    controlRegister->gpio0 = true;
    controlRegister->gasGaugeReset = false;
    controlRegister->gasGaugeEOC = true;
    controlRegister->voltageOrTemperatureEOC = true;
    controlRegister->powerOnReset = false;
    controlRegister->unused = 0;
}

bool STC3100_ReadControlRegister(i2c_port_t i2cPort, STC3100_ControlRegister *controlRegister, TickType_t ticksToWait) {
    uint8_t registerAddress = STC3100_REG_CTRL;
    CHECK_WRITE_RESULT(STC3100_I2CWrite(i2cPort, &registerAddress, sizeof(registerAddress), ticksToWait))
    CHECK_READ_RESULT(STC3100_I2CRead(i2cPort, (uint8_t *) controlRegister, sizeof(STC3100_ControlRegister), ticksToWait))

    return true;
}

bool STC3100_WriteControlRegister(i2c_port_t i2cPort, STC3100_ControlRegister *controlRegister, TickType_t ticksToWait) {
    uint8_t buffer[] = { STC3100_REG_CTRL, ((uint8_t *)controlRegister)[0] };
    CHECK_WRITE_RESULT(STC3100_I2CWrite(i2cPort, buffer, sizeof(buffer), ticksToWait))

    return true;
}

bool STC3100_ReadDeviceInfo(i2c_port_t i2cPort, uint8_t *partType, uint64_t *partId, TickType_t ticksToWait) {
    uint8_t registerAddress = STC3100_REG_ID0;

    struct {
        uint8_t partType;
        uint8_t deviceId[6];
        uint8_t crc;
    } __attribute__((packed)) deviceInfo;

    CHECK_WRITE_RESULT(STC3100_I2CWrite(i2cPort, &registerAddress, sizeof(registerAddress), ticksToWait))
    CHECK_READ_RESULT(STC3100_I2CRead(i2cPort, (uint8_t *)&deviceInfo, sizeof(deviceInfo), ticksToWait))

    uint8_t calculatedCrc = crc8((uint8_t *)&deviceInfo, sizeof(deviceInfo.partType) + sizeof(deviceInfo.deviceId));
    uint8_t receivedCrc = deviceInfo.crc;
    if (calculatedCrc != receivedCrc) {
        ESP_LOGE(STC3100_LOG_TAG, "Invalid CRC8: %02X (calculated) != %02X (received)", calculatedCrc, receivedCrc);
        return false;
    }

    *partType = deviceInfo.partType;
    *partId = ((uint64_t)deviceInfo.deviceId[0] << 0)
              | ((uint64_t)deviceInfo.deviceId[1] << 8)
              | ((uint64_t)deviceInfo.deviceId[2] << 16)
              | ((uint64_t)deviceInfo.deviceId[3] << 24)
              | ((uint64_t)deviceInfo.deviceId[4] << 32)
              | ((uint64_t)deviceInfo.deviceId[5] << 40);

    return true;
}

bool STC3100_ReadBatteryStatus(i2c_port_t i2cPort, STC3100_BatteryStatus *batteryStatus, TickType_t ticksToWait) {
    uint8_t registerAddress = STC3100_REG_CHARGE_LOW;

    struct {
      uint8_t chargeLow;
      uint8_t chargeHigh;
      uint8_t counterLow;
      uint8_t counterHigh;
      uint8_t currentLow;
      uint8_t currentHigh;
      uint8_t voltageLow;
      uint8_t voltageHigh;
      uint8_t temperatureLow;
      uint8_t temperatureHigh;
    } __attribute__((packed)) data;

    CHECK_WRITE_RESULT(STC3100_I2CWrite(i2cPort, &registerAddress, sizeof(registerAddress), ticksToWait))
    CHECK_READ_RESULT(STC3100_I2CRead(i2cPort, (uint8_t *) &data, sizeof(data), ticksToWait))

    batteryStatus->gasGauge = convertToInt16(data.chargeHigh, data.chargeLow, STC3100_16_BIT_MASK) / STC3100_R_SENSE_RESISTOR;
    batteryStatus->conversions = convertToUInt16(data.counterHigh, data.counterLow, STC3100_16_BIT_MASK);
    batteryStatus->current = convertToInt16(data.currentHigh, data.currentLow, STC3100_14_BIT_MASK) / STC3100_R_SENSE_RESISTOR;
    batteryStatus->voltage = convertToInt16(data.voltageHigh, data.voltageLow, STC3100_12_BIT_MASK);
    batteryStatus->temperature = convertToInt16(data.temperatureHigh, data.temperatureLow, STC3100_12_BIT_MASK);

    return true;
}

float STC3100_GetCapacity(STC3100_BatteryStatus *batteryStatus) {
    return STC3100_GAS_GAUGE_FACTOR * batteryStatus->gasGauge;
}

uint16_t STC3100_GetConversions(STC3100_BatteryStatus *batteryStatus) {
    return batteryStatus->conversions;
}

float STC3100_GetCurrent(STC3100_BatteryStatus *batteryStatus) {
    return STC3100_CURRENT_FACTOR * batteryStatus->current / 1000.0;
}

float STC3100_GetVoltage(STC3100_BatteryStatus *batteryStatus) {
    return STC3100_VOLTAGE_FACTOR * batteryStatus->voltage / 1000.0;
}

float STC3100_GetTemperature(STC3100_BatteryStatus *batteryStatus) {
    return STC3100_TEMPERATURE_FACTOR * batteryStatus->temperature;
}
