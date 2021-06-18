/**
 * @file test_adder.h
 * @author NikLeberg (niklaus.leuenb@gmail.com)
 * @brief Tests für adder-Modul
 * @version 0.3
 * @date 2021-05-26
 * 
 * @copyright Copyright (c) 2021 Leuenberger Niklaus
 * 
 * Dieses Modul dient als Vorlage für andere Module.
 * 
 */


/*
 * Includes
 * 
 */

#include "esp_system.h"
#include "freertos/FreeRTOS.h"

#include "unity.h"

#include "adder.h"


/*
 * Tests
 * 
 */

/**
 * @brief 1 + 1 muss 2 ergeben
 * 
 */
TEST_CASE("one added to one equals two", "[adder][ci]") {
    TEST_ASSERT_EQUAL(add1(1, 1), 2);
}

/**
 * @brief 5 + (-5) muss 0 ergeben
 * 
 */
TEST_CASE("five added to negative five equals zero", "[adder][ci]") {
    TEST_ASSERT_EQUAL(add1(5, -5), 0);
}

/**
 * @brief Strukturvariante, 1 + 1 muss 2 ergeben
 * 
 */
TEST_CASE("struct one added to one equals two", "[adder][ci]") {
    TEST_ASSERT_EQUAL(add1(5, -5), 0);
    addition_t addition = {1, 1, 0};
    TEST_ASSERT_EQUAL(add2(&addition), 2);
    TEST_ASSERT_EQUAL(addition.result, 2);
}
