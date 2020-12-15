/**
 * @file testSensors.c
 * @author NikLeberg (niklaus.leuenb@gmail.com)
 * @brief Unittest für sensors-component
 * @version 0.1
 * @date 2020-12-07
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
#include "sensors.h"


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
 * @brief Annahmen im Code werden eingehalten.
 * 
 */
TEST_CASE("assumptionsAreValid", "[sensors][ci]") {
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(sizeof(uint32_t), sizeof(0x1 << SENSORS_MAX));
}

/**
 * @brief Start erstellt einen Task.
 * 
 */
TEST_CASE("taskStartedAfterStart", "[sensors][ci]") {
    uint32_t numOfTasks = uxTaskGetNumberOfTasks();
    TEST_ASSERT_FALSE(sensorsStart());
    TEST_ASSERT_GREATER_THAN(numOfTasks, uxTaskGetNumberOfTasks());
}

/**
 * @brief Erneuter Start erstellt keinen weiteren Task.
 * 
 */
TEST_CASE("restartDoesNotCreateAdditionalTask", "[sensors][ci]") {
    uint32_t numOfTasks = uxTaskGetNumberOfTasks();
    TEST_ASSERT_FALSE(sensorsStart());
    TEST_ASSERT_EQUAL(numOfTasks, uxTaskGetNumberOfTasks());
}

/**
 * @brief Stop beendet den Task.
 * 
 */
TEST_CASE("taskStoppedAfterStop", "[sensors][ci]") {
    uint32_t numOfTasks = uxTaskGetNumberOfTasks();
    sensorsStop();
    TEST_ASSERT_LESS_THAN(numOfTasks, uxTaskGetNumberOfTasks());
}

/**
 * @brief Registrierung von intervallbasierten Sensoren funktioniert.
 * - Cookie wird weitergegeben
 * - Callback wird aufgerufen
 * - Intervall wird eingehalten
 * 
 */
uint32_t dummyIntervalSensorCallbackCallNum = 0;
TickType_t dummyIntervalSensorCallbackCalledAfter = 0;
const TickType_t interval = 10;
const TickType_t intervalJitter = 2;
void dummyIntervalSensorCallback(void *cookie) {
    TEST_ASSERT_EQUAL_HEX32(0xDEADBEEF, (uint32_t) cookie);
    ++dummyIntervalSensorCallbackCallNum;
    dummyIntervalSensorCallbackCalledAfter = xTaskGetTickCount();
    return;
}
TEST_CASE("registrationForIntervalBasedSensorWorks", "[sensors][ci]") {
    TEST_ASSERT_FALSE(sensorsRegister(SENSORS_ORIENTATION, dummyIntervalSensorCallback, (void*) 0xDEADBEEF, interval));
    TickType_t startTicks = xTaskGetTickCount();
    sensorsStart();
    vTaskDelay(interval + intervalJitter);
    TEST_ASSERT_EQUAL(1, dummyIntervalSensorCallbackCallNum);
    TEST_ASSERT_LESS_THAN(interval + intervalJitter, dummyIntervalSensorCallbackCalledAfter - startTicks);
    TEST_ASSERT_GREATER_THAN(interval - intervalJitter, dummyIntervalSensorCallbackCalledAfter - startTicks);
    sensorsRegister(SENSORS_ORIENTATION, NULL, NULL, 0);
    vTaskDelay(2 * interval);
    TEST_ASSERT_EQUAL(1, dummyIntervalSensorCallbackCallNum);
    sensorsStop();
}

/**
 * @brief Registrierung von eventbasierten Sensoren funktioniert.
 * - Cookie wird weitergegeben
 * - Callback wird aufgerufen
 * - Callback wird ohne grosse Verzögerung ausgeführt
 * 
 */
uint32_t dummyEventSensorCallbackCallNum = 0;
TickType_t dummyEventSensorCallbackCalledAfter = 0;
const TickType_t eventJitter = 10;
void dummyEventSensorCallback(void *cookie) {
    TEST_ASSERT_EQUAL_HEX32(0xBEEFDEAD, (uint32_t) cookie);
    ++dummyEventSensorCallbackCallNum;
    dummyEventSensorCallbackCalledAfter = xTaskGetTickCount();
    return;
}
TEST_CASE("registrationForEventBasedSensorWorks", "[sensors][ci]") {
    TEST_ASSERT_FALSE(sensorsRegister(SENSORS_ORIENTATION, dummyEventSensorCallback, (void*) 0xBEEFDEAD, 0));
    sensorsStart();
    vTaskDelay(eventJitter);
    TEST_ASSERT_EQUAL(0, dummyEventSensorCallbackCallNum);
    TickType_t startTicks = xTaskGetTickCount();
    sensorsNotify(SENSORS_ORIENTATION);
    vTaskDelay(2 * eventJitter);
    TEST_ASSERT_EQUAL(1, dummyEventSensorCallbackCallNum);
    TEST_ASSERT_LESS_OR_EQUAL(eventJitter, dummyEventSensorCallbackCalledAfter - startTicks);
    sensorsRegister(SENSORS_ORIENTATION, NULL, NULL, 0);
    vTaskDelay(eventJitter);
    TEST_ASSERT_EQUAL(1, dummyEventSensorCallbackCallNum);
    sensorsStop();
}

/**
 * @brief Setzen von Sensordaten nur innerhalb von Callback möglich
 * 
 */
bool dummySensorContextCalled = false;
void dummySensorContext(void *cookie) {
    (void) cookie;
    dummySensorContextCalled = true;
    sensorsData_t data;
    TEST_ASSERT_FALSE(sensorsSet(SENSORS_ORIENTATION, &data));
    return;
}
TEST_CASE("setOfRawDataOnlyInSensorContext", "[sensors][ci]") {
    sensorsRegister(SENSORS_ORIENTATION, dummySensorContext, NULL, 0);
    sensorsStart();
    sensorsData_t data;
    TEST_ASSERT_TRUE(sensorsSet(SENSORS_ORIENTATION, &data));
    sensorsNotify(SENSORS_ORIENTATION);
    vTaskDelay(eventJitter);
    TEST_ASSERT_TRUE(dummySensorContextCalled);
    sensorsStop();
    sensorsRegister(SENSORS_ORIENTATION, NULL, NULL, 0);
}

/**
 * @brief Erhalte richtig verarbeitete Orientierung
 * https://quaternions.online/
 */
uint32_t dummyOrientationCallNum = 0;
#define dummyOrientationTestPoints 5
const struct {
    sensorsQuaternion_t testInput;
    sensorsENU_t reference;
    sensorsQuaternion_t expectedLocal;
    sensorsVector_t expectedEulerLocal;
} dummyOrientationData[dummyOrientationTestPoints] = { // w = real; x = i; y = j; z = k
    {
        .testInput = {.i = 0.0, .j = 0.0, .k = 0.0, .real = 1.0},
        .reference = SENSORS_ENU_LOCAL,
        .expectedLocal = {.i = 0.0, .j = 0.0, .k = 0.0, .real = 1.0},
        .expectedEulerLocal = {.x = 0.0, .y = 0.0, .z = 0.0}
    },
    {
        .testInput = {.i = 0.707, .j = 0.0, .k = 0.0, .real = 0.707},
        .reference = SENSORS_ENU_LOCAL,
        .expectedLocal = {.i = 0.707, .j = 0.0, .k = 0.0, .real = 0.707},
        .expectedEulerLocal = {.x = 0.0, .y = 0.0, .z = 1.571}
    },
    {
        .testInput = {.i = 0.0, .j = 0.383, .k = 0.0, .real = 0.924},
        .reference = SENSORS_ENU_LOCAL,
        .expectedLocal = {.i = 0.0, .j = 0.383, .k = 0.0, .real = 0.924},
        .expectedEulerLocal = {.x = 0.0, .y = 0.786, .z = 0.0}
    },
    {
        .testInput = {.i = 0.5, .j = 0.5, .k = 0.5, .real = 0.5},
        .reference = SENSORS_ENU_LOCAL,
        .expectedLocal = {.i = 0.5, .j = 0.5, .k = 0.5, .real = 0.5},
        .expectedEulerLocal = {.x = 1.571, .y = 0.0, .z = 1.571}
    },
    {
        .testInput = {.i = 0.5, .j = 0.5, .k = 0.5, .real = 0.5},
        .reference = SENSORS_ENU_WORLD,
        .expectedLocal = {.i = -0.5, .j = -0.5, .k = -0.5, .real = 0.5},
        .expectedEulerLocal = {.x = 0.0, .y = -1.571, .z = 0.0}
    }
};
void dummyOrientation(void *cookie) {
    (void) cookie;
    sensorsData_t data;
    data.timestamp = 1;
    data.reference = dummyOrientationData[dummyOrientationCallNum].reference;
    data.quaternion = dummyOrientationData[dummyOrientationCallNum].testInput;
    sensorsSet(SENSORS_ORIENTATION, &data);
    dummyOrientationCallNum++;
    return;
}
TEST_CASE("orientationCorrect", "[sensors][ci]") {
    sensorsRegister(SENSORS_ORIENTATION, dummyOrientation, NULL, 0);
    sensorsStart();
    for (uint32_t i = 0; i < dummyOrientationTestPoints; ++i) {
        sensorsNotify(SENSORS_ORIENTATION);
        vTaskDelay(eventJitter);
        sensorsData_t data;
        TEST_ASSERT_FALSE(sensorsGet(SENSORS_STATE_ORIENTATION, SENSORS_ENU_LOCAL, &data));
        TEST_ASSERT_EQUAL_FLOAT(dummyOrientationData[i].expectedLocal.i, data.quaternion.i);
        TEST_ASSERT_EQUAL_FLOAT(dummyOrientationData[i].expectedLocal.j, data.quaternion.j);
        TEST_ASSERT_EQUAL_FLOAT(dummyOrientationData[i].expectedLocal.k, data.quaternion.k);
        TEST_ASSERT_EQUAL_FLOAT(dummyOrientationData[i].expectedLocal.real, data.quaternion.real);
        TEST_ASSERT_EQUAL(SENSORS_ENU_LOCAL, data.reference);
        TEST_ASSERT_FALSE(sensorsGet(SENSORS_STATE_ORIENTATION, SENSORS_ENU_WORLD, &data));
        TEST_ASSERT_EQUAL_FLOAT(-dummyOrientationData[i].expectedLocal.i, data.quaternion.i);
        TEST_ASSERT_EQUAL_FLOAT(-dummyOrientationData[i].expectedLocal.j, data.quaternion.j);
        TEST_ASSERT_EQUAL_FLOAT(-dummyOrientationData[i].expectedLocal.k, data.quaternion.k);
        TEST_ASSERT_EQUAL_FLOAT(dummyOrientationData[i].expectedLocal.real, data.quaternion.real);
        TEST_ASSERT_EQUAL(SENSORS_ENU_WORLD, data.reference);
        TEST_ASSERT_FALSE(sensorsGet(SENSORS_STATE_EULER, SENSORS_ENU_LOCAL, &data));
        TEST_ASSERT_FLOAT_WITHIN(0.002, dummyOrientationData[i].expectedEulerLocal.x, data.vector.x); // math Winkelfunktionen sind nicht super genau
        TEST_ASSERT_FLOAT_WITHIN(0.002, dummyOrientationData[i].expectedEulerLocal.y, data.vector.y);
        TEST_ASSERT_FLOAT_WITHIN(0.002, dummyOrientationData[i].expectedEulerLocal.z, data.vector.z);
        TEST_ASSERT_EQUAL(SENSORS_ENU_LOCAL, data.reference);
        TEST_ASSERT_FALSE(sensorsGet(SENSORS_STATE_EULER, SENSORS_ENU_WORLD, &data));
        TEST_ASSERT_FLOAT_WITHIN(0.002, -dummyOrientationData[i].expectedEulerLocal.x, data.vector.x);
        TEST_ASSERT_FLOAT_WITHIN(0.002, -dummyOrientationData[i].expectedEulerLocal.y, data.vector.y);
        TEST_ASSERT_FLOAT_WITHIN(0.002, -dummyOrientationData[i].expectedEulerLocal.z, data.vector.z);
        TEST_ASSERT_EQUAL(SENSORS_ENU_WORLD, data.reference);
    }
    sensorsStop();
}

/**
 * @brief Erhalte richtig verarbeitete Rotationen
 * http://danceswithcode.net/engineeringnotes/quaternions/conversion_tool.html
 */
uint32_t dummyVectorRotatesCallNum = 0;
#define dummyVectorRotatesTestPoints 5
sensorsType_t dummyVectorSensorUnderTest = SENSORS_ROTATION;
const struct {
    sensorsVector_t testInput;
    sensorsENU_t reference;
    sensorsQuaternion_t testOrientationLocal;
    sensorsVector_t expectedLocal;
    sensorsVector_t expectedWorld;
} dummyVectorRotatesData[dummyVectorRotatesTestPoints] = { // erstellt mit matlab quadrotate, quat2eul
    {
        .testInput = {.x = 0.0, .y = 0.0, .z = 0.0},
        .reference = SENSORS_ENU_LOCAL,
        .testOrientationLocal = {.i = 0.0, .j = 0.0, .k = 0.0, .real = 1.0},
        .expectedLocal = {.x = 0.0, .y = 0.0, .z = 0.0},
        .expectedWorld = {.x = 0.0, .y = 0.0, .z = 0.0}
    },
    {
        .testInput = {.x = 1.0, .y = 2.0, .z = 3.0},
        .reference = SENSORS_ENU_LOCAL,
        .testOrientationLocal = {.i = 0.707, .j = 0.0, .k = 0.0, .real = 0.707},
        .expectedLocal = {.x = 1.0, .y = 2.0, .z = 3.0},
        .expectedWorld = {.x = 1.0, .y = 3.0, .z = -2.0}
    },
    {
        .testInput = {.x = 2.3, .y = 3.4, .z = 4.5},
        .reference = SENSORS_ENU_LOCAL,
        .testOrientationLocal = {.i = -0.5, .j = -0.5, .k = -0.5, .real = 0.5},
        .expectedLocal = {.x = 2.3, .y = 3.4, .z = 4.5},
        .expectedWorld = {.x = 4.5, .y = 2.3, .z = 3.4}
    },
    {
        .testInput = {.x = 2.3, .y = 3.4, .z = 4.5},
        .reference = SENSORS_ENU_WORLD,
        .testOrientationLocal = {.i = 0.238, .j = 0.48, .k = -0.586, .real = -0.608},
        .expectedLocal = {.x = -5.866, .y = 1.617, .z = -0.273},
        .expectedWorld = {.x = 2.3, .y = 3.4, .z = 4.5}
    },
    {
        .testInput = {.x = 2.45, .y = 20.03, .z = -13.7},
        .reference = SENSORS_ENU_LOCAL,
        .testOrientationLocal = {.i = 0.5, .j = 0.5, .k = -0.183, .real = 0.683},
        .expectedLocal = {.x = 2.45, .y = 20.03, .z = -13.7},
        .expectedWorld = {.x = 17.933, .y = 3.661, .z = 16.121}
    }
};
void dummyVectorRotates(void *cookie) {
    (void) cookie;
    sensorsData_t data;
    data.timestamp = 1;
    data.reference = SENSORS_ENU_LOCAL;
    data.quaternion = dummyVectorRotatesData[dummyVectorRotatesCallNum].testOrientationLocal;
    sensorsSet(SENSORS_ORIENTATION, &data);
    data.reference = dummyVectorRotatesData[dummyVectorRotatesCallNum].reference;
    data.vector = dummyVectorRotatesData[dummyVectorRotatesCallNum].testInput;
    sensorsSet(dummyVectorSensorUnderTest, &data);
    dummyVectorRotatesCallNum++;
    if (dummyVectorRotatesCallNum == dummyVectorRotatesTestPoints) {
        dummyVectorRotatesCallNum = 0;
        dummyVectorSensorUnderTest = SENSORS_ACCELERATION;
    }
    return;
}
TEST_CASE("vectorRotatesCorrect", "[sensors]") {
    sensorsRegister(SENSORS_ROTATION, dummyVectorRotates, NULL, 0);
    sensorsStart();
    for (sensorsState_t s = SENSORS_STATE_ROTATION; s <= SENSORS_STATE_ACCELERATION; ++s) {
        for (uint32_t i = 0; i < dummyVectorRotatesTestPoints; ++i) {
            sensorsNotify(SENSORS_ROTATION);
            vTaskDelay(eventJitter);
            sensorsData_t data;
            TEST_ASSERT_FALSE(sensorsGet(s, SENSORS_ENU_LOCAL, &data));
            TEST_ASSERT_FLOAT_WITHIN(0.1, dummyVectorRotatesData[i].expectedLocal.x, data.vector.x);
            TEST_ASSERT_FLOAT_WITHIN(0.1, dummyVectorRotatesData[i].expectedLocal.y, data.vector.y);
            TEST_ASSERT_FLOAT_WITHIN(0.1, dummyVectorRotatesData[i].expectedLocal.z, data.vector.z);
            TEST_ASSERT_EQUAL(SENSORS_ENU_LOCAL, data.reference);
            TEST_ASSERT_FALSE(sensorsGet(s, SENSORS_ENU_WORLD, &data));
            TEST_ASSERT_FLOAT_WITHIN(0.1, dummyVectorRotatesData[i].expectedWorld.x, data.vector.x);
            TEST_ASSERT_FLOAT_WITHIN(0.1, dummyVectorRotatesData[i].expectedWorld.y, data.vector.y);
            TEST_ASSERT_FLOAT_WITHIN(0.1, dummyVectorRotatesData[i].expectedWorld.z, data.vector.z);
            TEST_ASSERT_EQUAL(SENSORS_ENU_WORLD, data.reference);
        }
    }
    sensorsStop();
}

/**
 * @brief Stelle sicher dass System immer korrekt gestoppt wird, egal wie die oberen Tests ausfallen.
 * 
 */
TEST_CASE("ensureCleanStopForFollowingTests", "[sensors][ci]") {
    sensorsStop();
}
