//
// Created by Maxim Dobryakov on 22/10/2020.
//

#ifndef STC3100_STC3100_MANAGER_H
#define STC3100_STC3100_MANAGER_H

#include <stdint.h>
#include <driver/i2c.h>

typedef struct {
    bool isCharging;

    float current;
    float voltage;
    float temperature;

    int16_t referenceCapacity;
    int16_t measuredCapacity;

    int16_t nominalCapacity;
    int16_t realCapacity;

    float minVoltage;
    float maxVoltage;

    float measuredPercentOfChange;
    float approximatePercentOfChange;
} STC3100_Manager_State;

typedef enum {
    STC3100_Manager_OK,
    STC3100_Manager_Fail,
    STC3100_Manager_BatteryNotConnected,
} STC3100_Manager_Error;

void STC3100_Manager_InitState(STC3100_Manager_State *state, int16_t nominalCapacity, float minVoltage, float maxVoltage);

STC3100_Manager_Error STC3100_Manager_Start(STC3100_Manager_State *state, int sclGpio, int sdaGpio, int i2cFrequency, i2c_port_t i2cPort);
STC3100_Manager_Error STC3100_Manager_UpdateState(STC3100_Manager_State *state, i2c_port_t i2cPort);
STC3100_Manager_Error STC3100_Manager_Stop(STC3100_Manager_State *state, i2c_port_t i2cPort);

#endif //STC3100_STC3100_MANAGER_H
