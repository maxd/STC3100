# STC3100 library for ESP32 MCU

FYI: This library is under construction.

## Info

`components/STC3100` directory contains sources of library.

* `STC3100.h` contains methods for interact with STC3100 IC by I2C bus.
* `STC3100_Manager.h` contains simple battery manager to calculate state of charge (SOC) (aka percent of charge).

## Tests

`test` directory contains test application. It run tests (manual, not unit-tests) from `components/STC3100/test` directory.