/**
 * @file bno.h
 * @author NikLeberg (niklaus.leuenb@gmail.com)
 * @brief HAL-Implementation für sh2 Bibliothek von Hillcrest.
 * @version 0.1
 * @date 2020-12-09
 * 
 * @copyright Copyright (c) 2020 Niklaus Leuenberger
 * 
 * @note "Glue-Code" zwischen sh2, i2c und sensors-component
 * @note BNO050 Sensor liefert Orientierung, Rotierung, Beschleunigung und Luftdruck. Die Rohdaten werden grob vorverarbeitet
 * und dann an sensor-component weitergeleitet zur Verarbeitung.
 * @note Component läuft in keinem eigenen Task sondern nutzt das Callback-System des sensor-components.
 * 
 */

#pragma once


/**
 * @brief Includes
 * 
 */

#include "esp_system.h"
#include "driver/gpio.h"


/**
 * @brief Typdeklarationen
 * 
 */

/* ... */


/**
 * @brief Variablendeklarationen
 * 
 */

#define BNO_I2C_ADDRESS     0x4B
#define BNO_RESET_PIN       GPIO_NUM_22
#define BNO_INTERRUPT_PIN   GPIO_NUM_23


/**
 * @brief Öffentliche Funktionen
 * 
 */

/**
 * @brief Starte BNO Sensorhub.
 * Der tatsächliche Start aller Aktivitäten wird erst über Callbacks vom sensors-component durchgeführt.
 * 
 * @return true - Fehlgeschlagen, false - Erfolgreich Registriert
 */
bool bnoStart();

/**
 * @brief Beende BNO Sensorhub.
 * 
 * @return true - kann momentan nicht gestoppt werden, false - erfolgreich
 */
bool bnoStop();
