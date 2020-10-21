//
// Created by Maxim Dobryakov on 19/10/2020.
//

#include <STC3100.h>

#include "Settings.h"

#include <esp_log.h>
#include <unity.h>

TEST_CASE("init I2C test", "[utils]") {
    STC3100_I2CInit(SCL_GPIO, SDA_GPIO, STC3100_DEFAULT_I2C_FREQUENCY, I2C_NUM_0);
    STC3100_I2CDeInit(I2C_NUM_0);
}

TEST_CASE("read/write I2C test", "[utils]") {
    STC3100_I2CInit(SCL_GPIO, SDA_GPIO, STC3100_DEFAULT_I2C_FREQUENCY, I2C_NUM_0);
    if (TEST_PROTECT()) {
        uint8_t registerAddress = STC3100_REG_ID0;
        esp_err_t result = STC3100_I2CWrite(I2C_NUM_0, &registerAddress, sizeof(registerAddress), STC3100_DEFAULT_TICKS_TO_WAIT);
        TEST_ESP_OK(result);

        uint8_t partId;
        result = STC3100_I2CRead(I2C_NUM_0, &partId, sizeof(partId), STC3100_DEFAULT_TICKS_TO_WAIT);
        TEST_ESP_OK(result);
        TEST_ASSERT_EQUAL_INT8(STC3100_PART_TYPE, partId);
    }
    STC3100_I2CDeInit(I2C_NUM_0);
}

TEST_CASE("read device info", "[STC3100]") {
    STC3100_I2CInit(SCL_GPIO, SDA_GPIO, STC3100_DEFAULT_I2C_FREQUENCY, I2C_NUM_0);
    if (TEST_PROTECT()) {
        uint8_t partType;
        uint64_t partId;
        bool result = STC3100_ReadDeviceInfo(I2C_NUM_0, &partType, &partId, STC3100_DEFAULT_TICKS_TO_WAIT);
        TEST_ASSERT_EQUAL(true, result);

        ESP_LOGI("TEST", "Part Type: %02X; Part ID: %llu", partType, partId);

        TEST_ASSERT_EQUAL(STC3100_PART_TYPE, partType);
        TEST_ASSERT_NOT_EQUAL(0x0, partId)
    }
    STC3100_I2CDeInit(I2C_NUM_0);
}

TEST_CASE("read default mode register", "[STC3100]") {
    STC3100_I2CInit(SCL_GPIO, SDA_GPIO, STC3100_DEFAULT_I2C_FREQUENCY, I2C_NUM_0);
    if (TEST_PROTECT()) {
        STC3100_ModeRegister mode;
        STC3100_InitModeRegister(&mode);

        bool result = STC3100_ReadModeRegister(I2C_NUM_0, &mode, STC3100_DEFAULT_TICKS_TO_WAIT);
        TEST_ASSERT_EQUAL(true, result);

        ESP_LOG_BUFFER_HEX("TEST", &mode, sizeof(mode));

        TEST_ASSERT_EQUAL(false, mode.selectExternalClock);
        TEST_ASSERT_EQUAL(STC3100_GasGaugeResolution14bits, mode.gasGaugeResolution);
        TEST_ASSERT_EQUAL(false, mode.gasGaugeCalibration);
        TEST_ASSERT_EQUAL(false, mode.gasGaugeEnabled);
    }
    STC3100_I2CDeInit(I2C_NUM_0);
}

TEST_CASE("read/write mode register", "[STC3100]") {
    STC3100_I2CInit(SCL_GPIO, SDA_GPIO, STC3100_DEFAULT_I2C_FREQUENCY, I2C_NUM_0);
    if (TEST_PROTECT()) {
        bool result;

        STC3100_ModeRegister mode;
        STC3100_InitModeRegister(&mode);

        mode.gasGaugeResolution = STC3100_GasGaugeResolution12bits;
        mode.gasGaugeEnabled = true;

        result = STC3100_WriteModeRegister(I2C_NUM_0, &mode, STC3100_DEFAULT_TICKS_TO_WAIT);
        TEST_ASSERT_EQUAL(true, result);

        result = STC3100_ReadModeRegister(I2C_NUM_0, &mode, STC3100_DEFAULT_TICKS_TO_WAIT);
        TEST_ASSERT_EQUAL(true, result);

        TEST_ASSERT_EQUAL(false, mode.selectExternalClock);
        TEST_ASSERT_EQUAL(STC3100_GasGaugeResolution12bits, mode.gasGaugeResolution);
        TEST_ASSERT_EQUAL(false, mode.gasGaugeCalibration);
        TEST_ASSERT_EQUAL(true, mode.gasGaugeEnabled);
    }
    STC3100_I2CDeInit(I2C_NUM_0);
}

TEST_CASE("read default control register", "[STC3100]") {
    STC3100_I2CInit(SCL_GPIO, SDA_GPIO, STC3100_DEFAULT_I2C_FREQUENCY, I2C_NUM_0);
    if (TEST_PROTECT()) {
        STC3100_ControlRegister control;
        STC3100_InitControlRegister(&control);

        bool result = STC3100_ReadControlRegister(I2C_NUM_0, &control, STC3100_DEFAULT_TICKS_TO_WAIT);
        TEST_ASSERT_EQUAL(true, result);

        ESP_LOG_BUFFER_HEX("TEST", &control, sizeof(control));

        TEST_ASSERT_EQUAL(false, control.gasGaugeReset);
        TEST_ASSERT_EQUAL(true, control.gasGaugeEOC);
        TEST_ASSERT_EQUAL(true, control.voltageOrTemperatureEOC);
        TEST_ASSERT_EQUAL(true, control.powerOnReset);
    }
    STC3100_I2CDeInit(I2C_NUM_0);
}

TEST_CASE("read/write control register", "[STC3100]") {
    STC3100_I2CInit(SCL_GPIO, SDA_GPIO, STC3100_DEFAULT_I2C_FREQUENCY, I2C_NUM_0);
    if (TEST_PROTECT()) {
        bool result;

        STC3100_ControlRegister control;
        STC3100_InitControlRegister(&control);

        control.gpio0 = true;
        control.gasGaugeReset = true;

        result = STC3100_WriteControlRegister(I2C_NUM_0, &control, STC3100_DEFAULT_TICKS_TO_WAIT);
        TEST_ASSERT_EQUAL(true, result);

        result = STC3100_ReadControlRegister(I2C_NUM_0, &control, STC3100_DEFAULT_TICKS_TO_WAIT);
        TEST_ASSERT_EQUAL(true, result);

        TEST_ASSERT_EQUAL(true, control.gpio0);
        TEST_ASSERT_EQUAL(false, control.gasGaugeReset);
    }
    STC3100_I2CDeInit(I2C_NUM_0);
}

TEST_CASE("read battery status", "[STC3100]") {
    STC3100_I2CInit(SCL_GPIO, SDA_GPIO, STC3100_DEFAULT_I2C_FREQUENCY, I2C_NUM_0);
    if (TEST_PROTECT()) {
        STC3100_BatteryStatus batteryStatus;

        bool result = STC3100_ReadBatteryStatus(I2C_NUM_0, &batteryStatus, STC3100_DEFAULT_TICKS_TO_WAIT);
        TEST_ASSERT_EQUAL(true, result);

        ESP_LOGI("TEST", "Capacity: %f; Conversions: %d; Current: %f; Voltage: %f; Temperature: %f",
                 STC3100_GetCapacity(&batteryStatus),
                 STC3100_GetConversions(&batteryStatus),
                 STC3100_GetCurrent(&batteryStatus),
                 STC3100_GetVoltage(&batteryStatus),
                 STC3100_GetTemperature(&batteryStatus));

        TEST_ASSERT_GREATER_OR_EQUAL(0, batteryStatus.gasGauge);
        TEST_ASSERT_GREATER_OR_EQUAL(0, batteryStatus.conversions);
        TEST_ASSERT_GREATER_OR_EQUAL(0, batteryStatus.current);
        TEST_ASSERT_GREATER_OR_EQUAL(0, batteryStatus.voltage);
        TEST_ASSERT_GREATER_OR_EQUAL(0, batteryStatus.temperature);

        TEST_ASSERT_GREATER_OR_EQUAL(0, STC3100_GetCapacity(&batteryStatus));
        TEST_ASSERT_GREATER_OR_EQUAL(0, STC3100_GetConversions(&batteryStatus));
        TEST_ASSERT_GREATER_OR_EQUAL(0, STC3100_GetCurrent(&batteryStatus));
        TEST_ASSERT_GREATER_OR_EQUAL(0, STC3100_GetVoltage(&batteryStatus));
        TEST_ASSERT_GREATER_OR_EQUAL(0, STC3100_GetTemperature(&batteryStatus));
    }
    STC3100_I2CDeInit(I2C_NUM_0);
}
