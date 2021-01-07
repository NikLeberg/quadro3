/**
 * @file testConfig.c
 * @author NikLeberg (niklaus.leuenb@gmail.com)
 * @brief Unittest für config-component
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
#include "unity.h"
#include "config.h"
#include "esp_heap_caps.h"


/**
 * @brief Typdeklarationen
 * 
 */

/* ... */


/**
 * @brief Variablendeklarationen
 * 
 */

size_t freeHeap;
float c1group1variable1;
float c1group1variable2;
float c1group2variable1;
float c2group1variable1;


/**
 * @brief Tests
 * 
 */

/**
 * @brief Annahmen im Code werden eingehalten.
 * 
 */
TEST_CASE("assumptionsAreValid", "[config][ci]") {
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(sizeof(float), sizeof(uint32_t));
}

/**
 * @brief Variablen einrichten, Defaultwerte werden gesetzt,
 * 
 */
TEST_CASE("canSetupConfigvariables", "[config][ci]") {
    // freier Speicher vor der Initialisierung
    freeHeap = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    // Variablen eines früheren ev. fehlgeschlagenen Tests löschen
    TEST_ASSERT_FALSE(configUnregister("c1", true));
    TEST_ASSERT_FALSE(configUnregister("c2", true));
    // Neu aufsetzen
    TEST_ASSERT_FALSE(CONFIG_REGISTER("c1", "group1", "variable1", c1group1variable1, 1.11));
    TEST_ASSERT_EQUAL_FLOAT(1.11, c1group1variable1);
    TEST_ASSERT_FALSE(CONFIG_REGISTER("c1", "group1", "variable2", c1group1variable2, 1.12));
    TEST_ASSERT_EQUAL_FLOAT(1.12, c1group1variable2);
    TEST_ASSERT_FALSE(CONFIG_REGISTER("c1", "group2", "variable1", c1group2variable1, 1.21));
    TEST_ASSERT_EQUAL_FLOAT(1.21, c1group2variable1);
    TEST_ASSERT_FALSE(CONFIG_REGISTER("c2", "group1", "variable1", c2group1variable1, 2.11));
    TEST_ASSERT_EQUAL_FLOAT(2.11, c2group1variable1);
}

/**
 * @brief Get/Set liefert korrekte Werte
 * 
 */
TEST_CASE("canGetSetConfigvariables", "[config][ci]") {
    float setValue = 100.0;
    TEST_ASSERT_TRUE(configSet("c1", "group2", "notRegistered", &setValue));
    TEST_ASSERT_FALSE(configSet("c1", "group1", "variable1", &setValue));
    TEST_ASSERT_EQUAL_FLOAT(100.0, c1group1variable1);
    float getValue;
    configType_t getType;
    TEST_ASSERT_FALSE(configGet("c1", "group1", "variable1", &getType, &getValue));
    TEST_ASSERT_EQUAL_FLOAT(100.0, getValue);
    TEST_ASSERT_EQUAL(CONFIG_TYPE_FLOAT, getType);
    c1group1variable1 = 1.11;
}

/**
 * @brief Löschen eines Components löscht nur das angegebene
 * 
 */
TEST_CASE("unregisterOnlyUnregistersTheRightOne", "[config][ci]") {
    TEST_ASSERT_FALSE(configUnregister("c1", true));
    float getValue;
    configType_t getType;
    TEST_ASSERT_TRUE(configGet("c1", "group1", "variable1", &getType, &getValue));
    TEST_ASSERT_TRUE(configGet("c1", "group1", "variable2", &getType, &getValue));
    TEST_ASSERT_TRUE(configGet("c1", "group2", "variable1", &getType, &getValue));
    TEST_ASSERT_FALSE(configGet("c2", "group1", "variable1", &getType, &getValue));
}

/**
 * @brief Variablen löschen, aktuelle Werte werden nicht überschrieben.
 * 
 */
TEST_CASE("canTeardownConfigvariables", "[config][ci]") {
    TEST_ASSERT_FALSE(configUnregister("c1", true));
    TEST_ASSERT_EQUAL_FLOAT(1.11, c1group1variable1);
    TEST_ASSERT_EQUAL_FLOAT(1.12, c1group1variable2);
    TEST_ASSERT_EQUAL_FLOAT(1.21, c1group2variable1);
    TEST_ASSERT_FALSE(configUnregister("c2", true));
    TEST_ASSERT_EQUAL_FLOAT(2.11, c2group1variable1);
    // kein Memoryleak
    TEST_ASSERT_LESS_OR_EQUAL(freeHeap, heap_caps_get_free_size(MALLOC_CAP_8BIT));
}
