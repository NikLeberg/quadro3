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
TEST_CASE("bnoFullUnitTest", "[bno]") {
    for (uint32_t i = 0; i < 2; ++i) { // Teste zweimal -> restart / reinitialisierung aller Komponenten funktioniert
        TEST_ASSERT_FALSE(i2cStart());
        TEST_ASSERT_FALSE(sensorsStart());
        sensorsData_t data;
        TEST_ASSERT_TRUE(sensorsGetRaw(SENSORS_ORIENTATION, &data)); // noch keine Daten
        TEST_ASSERT_TRUE(sensorsGetRaw(SENSORS_ROTATION, &data));
        TEST_ASSERT_TRUE(sensorsGetRaw(SENSORS_ACCELERATION, &data));
        TEST_ASSERT_TRUE(sensorsGetRaw(SENSORS_HEIGHT_ABOVE_SEA, &data));
        TEST_ASSERT_FALSE(bnoStart());
        vTaskDelay(bnoStartupDelay);
        TEST_ASSERT_FALSE(sensorsGetRaw(SENSORS_ORIENTATION, &data)); // Daten verfügbar
        TEST_ASSERT_NOT_EQUAL(0.0f, data.quaternion.i); // wird nie perfekt ausgerichtet sein
        TEST_ASSERT_NOT_EQUAL(0.0f, data.quaternion.j);
        TEST_ASSERT_NOT_EQUAL(0.0f, data.quaternion.k);
        TEST_ASSERT_NOT_EQUAL(0.0f, data.quaternion.real);
        TEST_ASSERT_FALSE(sensorsGetRaw(SENSORS_ROTATION, &data));
        //TEST_ASSERT_NOT_EQUAL(0.0f, data.vector.x);
        //TEST_ASSERT_NOT_EQUAL(0.0f, data.vector.y);
        //TEST_ASSERT_NOT_EQUAL(0.0f, data.vector.z);
        TEST_ASSERT_FALSE(sensorsGetRaw(SENSORS_ACCELERATION, &data));
        //TEST_ASSERT_NOT_EQUAL(0.0f, data.vector.x);
        //TEST_ASSERT_NOT_EQUAL(0.0f, data.vector.y);
        //TEST_ASSERT_NOT_EQUAL(0.0f, data.vector.z);
        TEST_ASSERT_FALSE(sensorsGetRaw(SENSORS_HEIGHT_ABOVE_SEA, &data));
        TEST_ASSERT_EQUAL(0.0f, data.vector.x);
        TEST_ASSERT_EQUAL(0.0f, data.vector.y);
        TEST_ASSERT_NOT_EQUAL(0.0f, data.vector.z);
        TEST_ASSERT_FALSE(bnoStop());
        vTaskDelay(bnoStartupDelay / 2);
        sensorsGetRaw(SENSORS_ORIENTATION, &data);
        int64_t lastOrientation = data.timestamp;
        sensorsGetRaw(SENSORS_ROTATION, &data);
        int64_t lastRotation = data.timestamp;
        sensorsGetRaw(SENSORS_ACCELERATION, &data);
        int64_t lastAcceleration = data.timestamp;
        sensorsGetRaw(SENSORS_HEIGHT_ABOVE_SEA, &data);
        int64_t lastHeight = data.timestamp;
        vTaskDelay(bnoStartupDelay / 2);
        sensorsGetRaw(SENSORS_ORIENTATION, &data);
        TEST_ASSERT_EQUAL_UINT32((uint32_t)lastOrientation, (uint32_t)data.timestamp); // keine weiteren Daten erhalten
        sensorsGetRaw(SENSORS_ROTATION, &data);
        TEST_ASSERT_EQUAL_UINT32((uint32_t)lastRotation, (uint32_t)data.timestamp);
        sensorsGetRaw(SENSORS_ACCELERATION, &data);
        TEST_ASSERT_EQUAL_UINT32((uint32_t)lastAcceleration, (uint32_t)data.timestamp);
        sensorsGetRaw(SENSORS_HEIGHT_ABOVE_SEA, &data);
        TEST_ASSERT_EQUAL_UINT32((uint32_t)lastHeight, (uint32_t)data.timestamp);
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
