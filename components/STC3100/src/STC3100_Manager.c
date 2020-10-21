//
// Created by Maxim Dobryakov on 22/10/2020.
//

#include "STC3100_Manager.h"
#include "STC3100.h"
#include "STC3100_Utils.h"

#include <math.h>
#include <esp_log.h>
#include <string.h>

#define STC3100_MANAGER_LOG_TAG "STC3100_Manager"

#define STC3100_MANAGER_PERSISTENT_DATA_HEADER 0x4242

typedef struct {
    uint16_t header;
    int16_t referenceCapacity;
    int16_t realCapacity;
    uint8_t crc8;
} STC3100_Manager_PersistentData;

void STC3100_Manager_InitPersistentData(STC3100_Manager_PersistentData *persistentData);
bool STC3100_Manager_ReadPersistentData(STC3100_Manager_PersistentData *persistentData, i2c_port_t i2cPort);
bool STC3100_Manager_WritePersistentData(STC3100_Manager_PersistentData *persistentData, i2c_port_t i2cPort);

bool STC3100_Manager_LoadState(STC3100_Manager_State *state, i2c_port_t i2cPort);
bool STC3100_Manager_SaveState(STC3100_Manager_State *state, i2c_port_t i2cPort);

void STC3100_Manager_InitState(STC3100_Manager_State *state, int16_t nominalCapacity, float minVoltage, float maxVoltage) {
    state->isCharging = false;

    state->current = 0;
    state->voltage = 0;
    state->temperature = 0;

    state->referenceCapacity = -1;
    state->measuredCapacity = -1;

    state->nominalCapacity = nominalCapacity;
    state->realCapacity = -1;

    state->minVoltage = minVoltage;
    state->maxVoltage = maxVoltage;

    state->measuredPercentOfChange = -1;
    state->approximatePercentOfChange = -1;
}

STC3100_Manager_Error STC3100_Manager_Start(STC3100_Manager_State *state, int sclGpio, int sdaGpio, int i2cFrequency, i2c_port_t i2cPort) {
    STC3100_I2CInit(sclGpio, sdaGpio, i2cFrequency, i2cPort);

    // Reset STC3100 counters and accumulators registers
    STC3100_ControlRegister controlRegister;
    STC3100_InitControlRegister(&controlRegister);
    controlRegister.gasGaugeReset = true;
    if (!STC3100_WriteControlRegister(i2cPort, &controlRegister, STC3100_DEFAULT_TICKS_TO_WAIT))
        return STC3100_Manager_Fail;

    // Enable STC3100 and set default resolution to 14 bit
    STC3100_ModeRegister modeRegister;
    STC3100_InitModeRegister(&modeRegister);
    modeRegister.gasGaugeResolution = STC3100_GasGaugeResolution14bits;
    modeRegister.gasGaugeEnabled = true;
    if (!STC3100_WriteModeRegister(i2cPort, &modeRegister, STC3100_DEFAULT_TICKS_TO_WAIT)) {
        ESP_LOGE(STC3100_MANAGER_LOG_TAG, "Can't start gas gauge.");
        return STC3100_Manager_Fail;
    }

    if(!STC3100_Manager_LoadState(state, i2cPort))
        return STC3100_Manager_Fail;

    return STC3100_Manager_OK;
}

STC3100_Manager_Error STC3100_Manager_UpdateState(STC3100_Manager_State *state, i2c_port_t i2cPort) {
    STC3100_BatteryStatus batteryStatus;
    if(!STC3100_ReadBatteryStatus(i2cPort, &batteryStatus, STC3100_DEFAULT_TICKS_TO_WAIT)) {
        ESP_LOGE(STC3100_MANAGER_LOG_TAG, "Can't read battery status.");
        return STC3100_Manager_Fail;
    }

    if (batteryStatus.voltage == 0) {
        ESP_LOGW(STC3100_MANAGER_LOG_TAG, "Battery not found.");
        return STC3100_Manager_BatteryNotConnected;
    }

    int16_t capacity = STC3100_GetCapacity(&batteryStatus);
    float current = STC3100_GetCurrent(&batteryStatus);
    float voltage = STC3100_GetVoltage(&batteryStatus);
    float temperature = STC3100_GetTemperature(&batteryStatus);

    state->isCharging = current > 0;

    state->current = current;
    state->voltage = voltage;
    state->temperature = temperature;

    bool saveState = false;
    if (voltage <= state->minVoltage) {
        saveState = state->referenceCapacity != 0;

        state->referenceCapacity = 0;
    } else if (voltage >= state->maxVoltage) {
        if (state->referenceCapacity >= 0) {
            saveState = state->realCapacity != (state->referenceCapacity + capacity);

            state->realCapacity = state->referenceCapacity + capacity;
        }
    }

    if (state->referenceCapacity >= 0 && state->realCapacity > 0) {
        saveState = state->measuredCapacity != (state->referenceCapacity + capacity);

        state->measuredCapacity = state->referenceCapacity + capacity;
        state->measuredPercentOfChange = fmaxf(fminf((float)state->measuredCapacity / (float)state->realCapacity, 1.0f), 0.0f);
    }

    state->approximatePercentOfChange = fmaxf(fminf((voltage - state->minVoltage) / (state->maxVoltage - state->minVoltage), 1.0f), 0.0f);

    if (saveState)
        STC3100_Manager_SaveState(state, i2cPort);

    return STC3100_Manager_OK;
}

STC3100_Manager_Error STC3100_Manager_Stop(STC3100_Manager_State *state, i2c_port_t i2cPort) {
    STC3100_Manager_SaveState(state, i2cPort);

    // Disable STC3100
    STC3100_ModeRegister modeRegister;
    STC3100_InitModeRegister(&modeRegister);
    modeRegister.gasGaugeEnabled = false;
    if (!STC3100_WriteModeRegister(i2cPort, &modeRegister, STC3100_DEFAULT_TICKS_TO_WAIT)) {
        ESP_LOGE(STC3100_MANAGER_LOG_TAG, "Can't stop gas gauge.");
        return STC3100_Manager_Fail;
    }

    STC3100_I2CDeInit(i2cPort);

    return STC3100_Manager_OK;
}

void STC3100_Manager_InitPersistentData(STC3100_Manager_PersistentData *persistentData) {
    persistentData->header = STC3100_MANAGER_PERSISTENT_DATA_HEADER;
    persistentData->referenceCapacity = -1;
    persistentData->realCapacity = -1;
    persistentData->crc8 = crc8((uint8_t *)persistentData, sizeof(*persistentData) - sizeof(persistentData->crc8));
}

bool STC3100_Manager_ReadPersistentData(STC3100_Manager_PersistentData *persistentData, i2c_port_t i2cPort) {
    esp_err_t result = STC3100_I2CRead(i2cPort, (uint8_t *)persistentData, sizeof(STC3100_Manager_PersistentData), STC3100_DEFAULT_TICKS_TO_WAIT);
    if (result != ESP_OK) {
        ESP_LOGE(STC3100_MANAGER_LOG_TAG, "STC3100_I2CRead return: %d", result);
        return false;
    }

    if (persistentData->header != STC3100_MANAGER_PERSISTENT_DATA_HEADER) {
        ESP_LOGW(STC3100_MANAGER_LOG_TAG, "Persistent data is not saved to RAM yet.");
        return false;
    }

    uint8_t calculatedCrc = crc8((uint8_t *)persistentData, sizeof(*persistentData) - sizeof(persistentData->crc8));
    uint8_t receivedCrc = persistentData->crc8;
    if (calculatedCrc != receivedCrc) {
        ESP_LOGW(STC3100_MANAGER_LOG_TAG, "Invalid CRC8 of persistent data: %02X (calculated) != %02X (received)", calculatedCrc, receivedCrc);
        return false;
    }

    return true;
}

bool STC3100_Manager_WritePersistentData(STC3100_Manager_PersistentData *persistentData, i2c_port_t i2cPort) {
    persistentData->header = STC3100_MANAGER_PERSISTENT_DATA_HEADER;
    persistentData->crc8 = crc8((uint8_t *)persistentData, sizeof(*persistentData) - sizeof(persistentData->crc8));

    uint8_t buffer[sizeof(*persistentData) + 1];
    buffer[0] = STC3100_REG_RAM0;
    memcpy(buffer + 1, persistentData, sizeof(*persistentData));

    esp_err_t result = STC3100_I2CWrite(i2cPort, buffer, sizeof(buffer), STC3100_DEFAULT_TICKS_TO_WAIT);
    if (result != ESP_OK) {
        ESP_LOGE(STC3100_MANAGER_LOG_TAG, "STC3100_I2CWrite return: %d", result);
        return false;
    }

    return true;
}

bool STC3100_Manager_LoadState(STC3100_Manager_State *state, i2c_port_t i2cPort) {
    STC3100_Manager_PersistentData persistentData;
    if (!STC3100_Manager_ReadPersistentData(&persistentData, i2cPort)) {
        ESP_LOGW(STC3100_MANAGER_LOG_TAG, "Persistent data will be initialized and saved to RAM later.");
        STC3100_Manager_InitPersistentData(&persistentData);
    }

    state->referenceCapacity = persistentData.referenceCapacity;
    state->measuredCapacity = persistentData.referenceCapacity;
    state->realCapacity = persistentData.realCapacity;

    ESP_LOGW(STC3100_MANAGER_LOG_TAG, "State loaded.");

    return true;
}

bool STC3100_Manager_SaveState(STC3100_Manager_State *state, i2c_port_t i2cPort) {
    STC3100_Manager_PersistentData persistentData;
    persistentData.referenceCapacity = state->measuredCapacity;
    persistentData.realCapacity = state->realCapacity;
    if (!STC3100_Manager_WritePersistentData(&persistentData, i2cPort)) {
        ESP_LOGE(STC3100_MANAGER_LOG_TAG, "Persistent data wasn't saved to RAM.");
        return false;
    }

    ESP_LOGW(STC3100_MANAGER_LOG_TAG, "State saved.");

    return true;
}
