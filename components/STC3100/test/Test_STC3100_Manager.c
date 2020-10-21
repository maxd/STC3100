//
// Created by Maxim Dobryakov on 22/10/2020.
//

#include <STC3100.h>
#include <STC3100_Manager.h>

#include "Settings.h"

#include <esp_log.h>
#include <unity.h>

TEST_CASE("measure capacity", "[STC3100_Manager]") {
    STC3100_Manager_State state;
    STC3100_Manager_InitState(&state, 800.0f, 3.0f, 4.1f);

    TEST_ASSERT_EQUAL(STC3100_Manager_OK, STC3100_Manager_Start(&state, SCL_GPIO, SDA_GPIO, STC3100_DEFAULT_I2C_FREQUENCY, I2C_NUM_0));

    for(;;) {
        STC3100_Manager_Error result = STC3100_Manager_UpdateState(&state, I2C_NUM_0);
        if(result == STC3100_Manager_Fail)
            break;

        ESP_LOGI("", "isCh: %d; C: %6.2f; V: %6.2f; T: %6.2f; refCap: %6d; mCap: %6d; rCap: %6d; mPOC%%: %6.2f, aPOC%%: %6.2f",
                 state.isCharging,
                 state.current,
                 state.voltage,
                 state.temperature,
                 state.referenceCapacity,
                 state.measuredCapacity,
                 state.realCapacity,
                 state.measuredPercentOfChange,
                 state.approximatePercentOfChange);

        vTaskDelay(pdMS_TO_TICKS(3000));
    }

    TEST_ASSERT_EQUAL(STC3100_Manager_OK, STC3100_Manager_Stop(&state, I2C_NUM_0));
}
