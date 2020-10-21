//
// Created by Maxim Dobryakov on 19/10/2020.
//

#ifndef STC3100_STC3100_H
#define STC3100_STC3100_H

#include <driver/i2c.h>

#define STC3100_PART_TYPE 0x10 // Part Type = 10h for the STC3100

#ifndef STC3100_ADDRESS
#define STC3100_ADDRESS 0x70
#endif

#ifndef STC3100_DEFAULT_I2C_FREQUENCY
#define STC3100_DEFAULT_I2C_FREQUENCY 100000 // Hz
#endif

#ifndef STC3100_DEFAULT_TICKS_TO_WAIT
#define STC3100_DEFAULT_TICKS_TO_WAIT 10 // ticks
#endif

#ifndef STC3100_R_SENSE_RESISTOR
#define STC3100_R_SENSE_RESISTOR 20 // mOhm
#endif

#ifndef STC3100_GAS_GAUGE_FACTOR
#define STC3100_GAS_GAUGE_FACTOR 6.70 // uV.h
#endif

#ifndef STC3100_CURRENT_FACTOR
#define STC3100_CURRENT_FACTOR 11.77 // uV.h
#endif

#ifndef STC3100_VOLTAGE_FACTOR
#define STC3100_VOLTAGE_FACTOR 2.44 // mV
#endif

#ifndef STC3100_TEMPERATURE_FACTOR
#define STC3100_TEMPERATURE_FACTOR 0.125 // C
#endif

typedef enum {
    /* Control registers */

    /* Mode register */
    STC3100_REG_MODE = 0,
    /* Control and status register */
    STC3100_REG_CTRL = 1,
    /* Gas gauge charge data, bits 0-7 */
    STC3100_REG_CHARGE_LOW = 2,
    /* Gas gauge charge data, bits 8-15 */
    STC3100_REG_CHARGE_HIGH = 3,
    /* Number of conversions, bits 0-7 */
    STC3100_REG_COUNTER_LOW = 4,
    /* Number of conversions, bits 8-15 */
    STC3100_REG_COUNTER_HIGH = 5,
    /* Battery current value, bits 0-7 */
    STC3100_REG_CURRENT_LOW = 6,
    /* Battery current value, bits 8-15 */
    STC3100_REG_CURRENT_HIGH = 7,
    /* Battery voltage value, bits 0-7 */
    STC3100_REG_VOLTAGE_LOW = 8,
    /* Battery voltage value, bits 8-15 */
    STC3100_REG_VOLTAGE_HIGH = 9,
    /* Temperature value, bits 0-7 */
    STC3100_REG_TEMPERATURE_LOW = 10,
    /* Temperature value, bits 8-15 */
    STC3100_REG_TEMPERATURE_HIGH = 11,

    /* Device ID registers */

    /* Part type ID = 10h */
    STC3100_REG_ID0 = 24,
    /* Unique part ID, bits 0-7 */
    STC3100_REG_ID1 = 25,
    /* Unique part ID, bits 8-15 */
    STC3100_REG_ID2 = 26,
    /* Unique part ID, bits 16-23 */
    STC3100_REG_ID3 = 27,
    /* Unique part ID, bits 24-31 */
    STC3100_REG_ID4 = 28,
    /* Unique part ID, bits 32-39 */
    STC3100_REG_ID5 = 29,
    /* Unique part ID, bits 40-47 */
    STC3100_REG_ID6 = 30,
    /* Device ID CRC */
    STC3100_REG_ID7 = 31,

    /* RAM registers */

    /* General-purpose RAM register */
    STC3100_REG_RAM0 = 32,
    /* General-purpose RAM register */
    STC3100_REG_RAM1 = 33,
    /* General-purpose RAM register */
    STC3100_REG_RAM2 = 34,
    /* General-purpose RAM register */
    STC3100_REG_RAM3 = 35,
    /* General-purpose RAM register */
    STC3100_REG_RAM4 = 36,
    /* General-purpose RAM register */
    STC3100_REG_RAM5 = 37,
    /* General-purpose RAM register */
    STC3100_REG_RAM6 = 38,
    /* General-purpose RAM register */
    STC3100_REG_RAM7 = 39,
    /* General-purpose RAM register */
    STC3100_REG_RAM8 = 40,
    /* General-purpose RAM register */
    STC3100_REG_RAM9 = 41,
    /* General-purpose RAM register */
    STC3100_REG_RAM10 = 42,
    /* General-purpose RAM register */
    STC3100_REG_RAM11 = 43,
    /* General-purpose RAM register */
    STC3100_REG_RAM12 = 44,
    /* General-purpose RAM register */
    STC3100_REG_RAM13 = 45,
    /* General-purpose RAM register */
    STC3100_REG_RAM14 = 46,
    /* General-purpose RAM register */
    STC3100_REG_RAM15 = 47,
    /* General-purpose RAM register */
    STC3100_REG_RAM16 = 48,
    /* General-purpose RAM register */
    STC3100_REG_RAM17 = 49,
    /* General-purpose RAM register */
    STC3100_REG_RAM18 = 50,
    /* General-purpose RAM register */
    STC3100_REG_RAM19 = 51,
    /* General-purpose RAM register */
    STC3100_REG_RAM20 = 52,
    /* General-purpose RAM register */
    STC3100_REG_RAM21 = 53,
    /* General-purpose RAM register */
    STC3100_REG_RAM22 = 54,
    /* General-purpose RAM register */
    STC3100_REG_RAM23 = 55,
    /* General-purpose RAM register */
    STC3100_REG_RAM24 = 56,
    /* General-purpose RAM register */
    STC3100_REG_RAM25 = 57,
    /* General-purpose RAM register */
    STC3100_REG_RAM26 = 58,
    /* General-purpose RAM register */
    STC3100_REG_RAM27 = 59,
    /* General-purpose RAM register */
    STC3100_REG_RAM28 = 60,
    /* General-purpose RAM register */
    STC3100_REG_RAM29 = 61,
    /* General-purpose RAM register */
    STC3100_REG_RAM30 = 62,
    /* General-purpose RAM register */
    STC3100_REG_RAM31 = 63,
} STC3100_Registers;

typedef enum {
    STC3100_GasGaugeResolution14bits = 0b00,
    STC3100_GasGaugeResolution13bits = 0b01,
    STC3100_GasGaugeResolution12bits = 0b10,
} STC3100_GasGaugeResolution;

typedef struct {
    bool selectExternalClock: 1;
    STC3100_GasGaugeResolution gasGaugeResolution: 2;
    bool gasGaugeCalibration: 1;
    bool gasGaugeEnabled: 1;
    uint8_t unused:3;
} __attribute__((packed)) STC3100_ModeRegister;

typedef struct {
    bool gpio0: 1;
    bool gasGaugeReset: 1;
    bool gasGaugeEOC: 1;
    bool voltageOrTemperatureEOC: 1;
    bool powerOnReset: 1;
    uint8_t unused:3;
} __attribute__((packed)) STC3100_ControlRegister;

typedef struct {
    int16_t gasGauge;
    uint16_t conversions;
    int16_t current;
    int16_t voltage;
    int16_t temperature;
} STC3100_BatteryStatus;

void STC3100_I2CInit(int sclGpio, int sdaGpio, int i2cFrequency, i2c_port_t i2cPort);
void STC3100_I2CDeInit(i2c_port_t i2cPort);

esp_err_t STC3100_I2CRead(i2c_port_t i2cPort, uint8_t *buffer, size_t bufferLength, TickType_t ticksToWait);
esp_err_t STC3100_I2CWrite(i2c_port_t i2cPort, uint8_t *buffer, size_t bufferLength, TickType_t ticksToWait);

void STC3100_InitModeRegister(STC3100_ModeRegister *modeRegister);
bool STC3100_ReadModeRegister(i2c_port_t i2cPort, STC3100_ModeRegister *modeRegister, TickType_t ticksToWait);
bool STC3100_WriteModeRegister(i2c_port_t i2cPort, STC3100_ModeRegister *modeRegister, TickType_t ticksToWait);

void STC3100_InitControlRegister(STC3100_ControlRegister *controlRegister);
bool STC3100_ReadControlRegister(i2c_port_t i2cPort, STC3100_ControlRegister *controlRegister, TickType_t ticksToWait);
bool STC3100_WriteControlRegister(i2c_port_t i2cPort, STC3100_ControlRegister *controlRegister, TickType_t ticksToWait);

bool STC3100_ReadDeviceInfo(i2c_port_t i2cPort, uint8_t *partType, uint64_t *partId, TickType_t ticksToWait);

bool STC3100_ReadBatteryStatus(i2c_port_t i2cPort, STC3100_BatteryStatus *batteryStatus, TickType_t ticksToWait);

float STC3100_GetCapacity(STC3100_BatteryStatus *batteryStatus);
uint16_t STC3100_GetConversions(STC3100_BatteryStatus *batteryStatus);
float STC3100_GetCurrent(STC3100_BatteryStatus *batteryStatus);
float STC3100_GetVoltage(STC3100_BatteryStatus *batteryStatus);
float STC3100_GetTemperature(STC3100_BatteryStatus *batteryStatus);

#endif //STC3100_STC3100_H
