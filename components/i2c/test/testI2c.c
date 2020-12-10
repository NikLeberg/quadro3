/**
 * @file testI2c.c
 * @author NikLeberg (niklaus.leuenb@gmail.com)
 * @brief Unittest für i2c-component
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
#include "driver/i2c.h"
#include "i2c.h"


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
 * @brief Mockups
 * 
 */

void startSlave() {
    i2c_config_t i2cConfig;
    i2cConfig.mode           = I2C_MODE_SLAVE;
    i2cConfig.sda_io_num     = GPIO_NUM_2;
    i2cConfig.sda_pullup_en  = GPIO_PULLUP_DISABLE;
    i2cConfig.scl_io_num     = GPIO_NUM_4;
    i2cConfig.scl_pullup_en  = GPIO_PULLUP_DISABLE;
    i2cConfig.slave.slave_addr = 0x12;
    i2cConfig.slave.addr_10bit_en = 0;
    TEST_ASSERT_FALSE(i2c_param_config(I2C_NUM_1, &i2cConfig));
    TEST_ASSERT_FALSE(i2c_driver_install(I2C_NUM_1, I2C_MODE_SLAVE, 128, 128, 0));
}

void stopSlave() {
    i2c_driver_delete(I2C_NUM_1);
}


/**
 * @brief Tests
 * 
 */

/**
 * @brief Kann vom eigenen Slave lesen.
 * 
 */
TEST_CASE("canReadFromSlave", "[i2c]") {
    // Master und Slave Bus starten
    TEST_ASSERT_FALSE(i2cStart());
    startSlave();
    // Testbuffer
    uint8_t toRead[128];
    for (uint8_t i = 0; i < 128; ++i) {
        toRead[i] = 0x55;
    }
    i2c_slave_write_buffer(I2C_NUM_1, toRead, 128, portMAX_DELAY);
    // Einlesen
    uint8_t readBytes[128];
    TEST_ASSERT_FALSE(i2cRead(0x12, readBytes, 128, portMAX_DELAY));
    TEST_ASSERT_EQUAL_MEMORY_ARRAY(toRead, readBytes, 1, 128);
    // Master und Slave beenden
    stopSlave();
    i2cStop();
}

/**
 * @brief Kann zum eigenen Slave schreiben.
 * 
 */
TEST_CASE("canWriteToSlave", "[i2c]") {
    // Master und Slave Bus starten
    TEST_ASSERT_FALSE(i2cStart());
    startSlave();
    // Testbuffer
    uint8_t toWrite[16];
    for (uint8_t i = 0; i < 16; ++i) {
        toWrite[i] = 0x34;
    }
    // Schreiben
    TEST_ASSERT_EQUAL(0, i2cWrite(0x12, toWrite, 16, portMAX_DELAY));
    uint8_t writtenBytes[16];
    TEST_ASSERT_EQUAL(16, i2c_slave_read_buffer(I2C_NUM_1, writtenBytes, 16, portMAX_DELAY));
    // Prüfen
    TEST_ASSERT_EQUAL_MEMORY_ARRAY(toWrite, writtenBytes, 1, 16);
    // Master und Slave beenden
    stopSlave();
    i2cStop();
}
