/**
 * @file i2c.h
 * @author NikLeberg (niklaus.leuenb@gmail.com)
 * @brief I2C-Busfunktionen als Wrapper für ESP-IDF i2c-Treiber.
 * @version 0.1
 * @date 2020-12-07
 * 
 * @copyright Copyright (c) 2020 Niklaus Leuenberger
 * 
 */

#pragma once


/**
 * @brief Includes
 * 
 */

#include "esp_system.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"


/**
 * @brief Typdeklarationen
 * 
 */

/* ... */


/**
 * @brief Variablendeklarationen
 * 
 */

#define I2C_SCL             GPIO_NUM_5
#define I2C_SDA             GPIO_NUM_17
#define I2C_CLOCK           400000


/**
 * @brief Öffentliche Funktionen
 * 
 */

/**
 * @brief Startet I2C-Master.
 * 
 * @return true - Start fehlgeschlagen, false - Start erfolgreich
 * @return false 
 */
bool i2cStart();

/**
 * @brief Stoppe den I2C-Master.
 * 
 */
void i2cStop();

/**
 * @brief Sende Bytes an Slave.
 * 
 * @param deviceAddr Slaveadresse
 * @param pData Pointer zu den Daten
 * @param dataLength Länge der Daten in Bytes
 * @param maxDelay maximale Blockzeit
 * @return true - Sendefehler, false - Erfolgreich gesendet
 */
bool i2cWrite(uint8_t deviceAddr, uint8_t *pData, size_t dataLength, TickType_t maxDelay);

/**
 * @brief Lese Bytes aus Slave.
 * 
 * @param deviceAddr Slaveadresse
 * @param pData[out] Pointer zu den Daten
 * @param dataLength Länge der zu empfangenden Daten in Bytes
 * @param maxDelay maximale Blockzeit
 * @return true - Lesefehler, false - Erfolgreich gelesen
 */
bool i2cRead(uint8_t deviceAddr, uint8_t *pData, size_t dataLength, TickType_t maxDelay);
