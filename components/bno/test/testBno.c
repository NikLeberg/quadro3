/**
 * @file testBno.c
 * @author NikLeberg (niklaus.leuenb@gmail.com)
 * @brief Unittest für bno-component
 * @version 0.1
 * @date 2020-12-10
 * 
 * @copyright Copyright (c) 2020 Niklaus Leuenberger
 * 
 */

/**
 * @brief Includes
 * 
 */

#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "unity.h"
#include "i2c.h"
#include "sensors.h"
#include "bno.h"


/**
 * @brief Typdeklarationen
 * 
 */

/* ... */


/**
 * @brief Variablendeklarationen
 * 
 */

/* ... */


/**
 * @brief Tests
 * 
 */

/**
 * @brief Sensor kann gestartet werden und leitet die Daten zum sensors-component weiter.
 * 
 */
const TickType_t bnoStartupDelay = 1000 / portTICK_PERIOD_MS;
TEST_CASE("startsAndForwardsDataToSensorsComponent", "[bno]") {
    for (uint32_t i = 0; i < 2; ++i) { // Teste zweimal -> restart / reinitialisierung aller Komponenten funktioniert
        TEST_ASSERT_FALSE(i2cStart());
        TEST_ASSERT_FALSE(sensorsStart());
        sensorsData_t data;
        TEST_ASSERT_TRUE(sensorsGetState(SENSORS_STATE_ORIENTATION, SENSORS_ENU_LOCAL, &data)); // noch keine Daten
        TEST_ASSERT_TRUE(sensorsGetState(SENSORS_STATE_EULER, SENSORS_ENU_LOCAL, &data));
        TEST_ASSERT_TRUE(sensorsGetState(SENSORS_STATE_ROTATION, SENSORS_ENU_LOCAL, &data));
        TEST_ASSERT_TRUE(sensorsGetState(SENSORS_STATE_ACCELERATION, SENSORS_ENU_LOCAL, &data));
        TEST_ASSERT_FALSE(bnoStart());
        vTaskDelay(bnoStartupDelay);
        TEST_ASSERT_FALSE(sensorsGetState(SENSORS_STATE_ORIENTATION, SENSORS_ENU_LOCAL, &data)); // Daten verfügbar
        TEST_ASSERT_FALSE(sensorsGetState(SENSORS_STATE_EULER, SENSORS_ENU_LOCAL, &data));
        TEST_ASSERT_FALSE(sensorsGetState(SENSORS_STATE_ROTATION, SENSORS_ENU_LOCAL, &data));
        TEST_ASSERT_FALSE(sensorsGetState(SENSORS_STATE_ACCELERATION, SENSORS_ENU_LOCAL, &data));
        TEST_ASSERT_FALSE(bnoStop());
        vTaskDelay(bnoStartupDelay / 2);
        sensorsGetState(SENSORS_STATE_ORIENTATION, SENSORS_ENU_LOCAL, &data);
        int64_t timestamp = data.timestamp;
        vTaskDelay(10);
        sensorsGetState(SENSORS_STATE_ORIENTATION, SENSORS_ENU_LOCAL, &data);
        TEST_ASSERT_EQUAL_UINT32((uint32_t)timestamp, (uint32_t)data.timestamp); // keine weiteren Daten erhalten
        sensorsStop();
        i2cStop();
        vTaskDelay(10);
    }
}

/**
 * @brief Stelle sicher dass System immer korrekt gestoppt wird, egal wie die oberen Tests ausfallen.
 * 
 */
TEST_CASE("ensureCleanStopForFollowingTests", "[bno]") {
    bnoStop();
    vTaskDelay(bnoStartupDelay / 2);
    sensorsStop();
    i2cStop();
}
