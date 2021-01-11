/**
 * @file flow.h
 * @author NikLeberg (niklaus.leuenb@gmail.com)
 * @brief Mateksys Optical Flow Sensor basierend auf PMW3901.
 * @version 0.1
 * @date 2020-12-16
 * 
 * @copyright Copyright (c) 2020 Niklaus Leuenberger
 * 
 * Kommuniziert über Multiwii Serial Protokoll Version 2 (MSPv2).
 * - Sendet mit ~10Hz Flow Rate in Pixel
 * - ebenfalls sendet es Lidar Entfernung in mm (max. 2 m)
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

#define FLOW_RX_PIN         GPIO_NUM_16
#define FLOW_UART           UART_NUM_1


/**
 * @brief Öffentliche Funktionen
 * 
 */

/**
 * @brief Starte den Flowsensor.
 * 
 * @return true - Fehlgeschlagen, false - Erfolgreich gestartet
 */
bool flowStart();

/**
 * @brief Beende den Flowsensor.
 * 
 * @return true - kann momentan nicht gestoppt werden, false - erfolgreich
 */
bool flowStop();

/**
 * @brief Setze aktuelle Lidarhöhe als Nullpunkt
 * 
 * @param average true - bilde Mittelwert aus vorherigem und jetztigem Offset, false - überschreibe Offset
 * @return true - ungültige Daten, false - erfolgreich
 */
bool flowLidarHome(bool average);

/**
 * @brief Kalibriere die Pixel/s vom Sensor in rad/s durch vergleich mit Gyrsoskop.
 * 
 * @param average true - bilde Mittelwert aus vorheriger und jetztiger Skalierung, false - überschreibe Skalierung
 * @return true - ungültige Daten, false - erfolgreich
 */
bool flowCalibrate(bool average);
