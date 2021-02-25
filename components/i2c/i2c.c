/**
 * @file i2c.c
 * @author NikLeberg (niklaus.leuenb@gmail.com)
 * @brief I2C-Busfunktionen als Wrapper für ESP-IDF i2c-Treiber.
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

#include "i2c.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"


/**
 * @brief Typdeklarationen
 * 
 */

/* ... */


/**
 * @brief Variablendeklarationen
 * 
 */

#define I2C_NUM             I2C_NUM_0
#define I2C_LOCK_TIMEOUT_MS 100
#define I2C_BUS_TIMEOUT_MS  10

static SemaphoreHandle_t i2cLock;


/**
 * @brief Private Funktionsprototypen
 * 
 */

/* ... */

/**
 * @brief Implementation Öffentlicher Funktionen
 * 
 */

bool i2cStart() {
    // I2C Installieren
    i2c_config_t i2cConfig = {
        .mode           = I2C_MODE_MASTER,
        .sda_io_num     = I2C_SDA,
        .sda_pullup_en  = GPIO_PULLUP_ENABLE,
        .scl_io_num     = I2C_SCL,
        .scl_pullup_en  = GPIO_PULLUP_ENABLE,
        .master = {.clk_speed = I2C_CLOCK}
    };
    if (i2c_param_config(I2C_NUM, &i2cConfig)) return true;
    if (i2c_driver_install(I2C_NUM, I2C_MODE_MASTER, 0, 0, 0)) return true;
    if (i2c_set_timeout(I2C_NUM, (1ULL << 20) - 1)) return true; // ca 13 ms Timeout
    // SCL Dutycycle des Drivers korrigieren (idf setzt 50 %, gemäss Spezifikation aber maximal 33 % zulässig)
    uint32_t cycleThird = APB_CLK_FREQ / I2C_CLOCK / 3;
    if (i2c_set_period(I2C_NUM, cycleThird, cycleThird * 2)
    || i2c_set_start_timing(I2C_NUM, cycleThird, cycleThird)
    || i2c_set_stop_timing(I2C_NUM, cycleThird, cycleThird)
    || i2c_set_data_timing(I2C_NUM, cycleThird, cycleThird)) return true;
    // Sempahor einrichten
    i2cLock = xSemaphoreCreateMutex();
    return !i2cLock;
}

void i2cStop() {
    if (i2cLock) {
        xSemaphoreTake(i2cLock, portMAX_DELAY);
        vSemaphoreDelete(i2cLock);
        i2cLock = NULL;
    }
    i2c_driver_delete(I2C_NUM);
    return;
}

bool i2cWrite(uint8_t deviceAddr, uint8_t *pData, size_t dataLength, TickType_t maxDelay) {
    i2c_cmd_handle_t cmd;
    esp_err_t err;
    TickType_t startTick = xTaskGetTickCount();
    if (!i2cLock) return true;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, deviceAddr << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, pData, dataLength, true);
    i2c_master_stop(cmd);
    if (xSemaphoreTake(i2cLock, maxDelay) == pdFALSE) return true;
    if (maxDelay != portMAX_DELAY) {
        TickType_t dTick = xTaskGetTickCount() - startTick;
        if (dTick >= maxDelay) maxDelay = 0;
        else maxDelay -= dTick;
    }
    err = i2c_master_cmd_begin(I2C_NUM, cmd, maxDelay);
    xSemaphoreGive(i2cLock);
    i2c_cmd_link_delete(cmd);
    return err;
}

bool i2cRead(uint8_t deviceAddr, uint8_t *pData, size_t dataLength, TickType_t maxDelay) {
    i2c_cmd_handle_t cmd;
    esp_err_t err;
    TickType_t startTick = xTaskGetTickCount();
    if (!i2cLock) return true;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, deviceAddr << 1 | I2C_MASTER_READ, true);
    i2c_master_read(cmd, pData, dataLength, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    if (xSemaphoreTake(i2cLock, maxDelay) == pdFALSE) return true;
    if (maxDelay != portMAX_DELAY) {
        TickType_t dTick = xTaskGetTickCount() - startTick;
        if (dTick >= maxDelay) maxDelay = 0;
        else maxDelay -= dTick;
    }
    err = i2c_master_cmd_begin(I2C_NUM, cmd, maxDelay);
    xSemaphoreGive(i2cLock);
    i2c_cmd_link_delete(cmd);
    return err;
}


/**
 * @brief Implementation Privater Funktionen
 * 
 */

/* ... */
