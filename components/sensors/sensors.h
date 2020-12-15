/**
 * @file sensors.h
 * @author NikLeberg (niklaus.leuenb@gmail.com)
 * @brief Fusioniert alle Sensordaten und publiziert den Zustand des Quadros für andere Module.
 * @version 0.1
 * @date 2020-12-05
 * 
 * @copyright Copyright (c) 2020 Niklaus Leuenberger
 * 
 * - Daten werden im ENU Koordinatensystem erwartet und publiziert.
 * - Jeder Sensor des Quadro registriert ein ProcessCallback. Dieser Callback wird entweder zyklisch oder
 * eventbasiert aufgerufen. Events lassen sich mit sensorsNotify() melden.
 * - Hat ein Sensor neue Messwerte übergibt er diese per sensorsSet().
 * - Messwerte werden verarbeitet, ggf. von ENU-lokal in world oder umgekehrt umgerechnet, ggf. fusioniert und dann als Stats abgespeichert.
 * - Components welche Daten benötigen können diese per sensorsGet() erhalten.
 * 
 */

#pragma once


/**
 * @brief Includes
 * 
 */

#include "esp_system.h"
#include "freertos/FreeRTOS.h"


/**
 * @brief Typdeklarationen
 * 
 */

/**
 * @brief Floating Point.
 * Hardwarebeschleunigung von floats mit der FPU in qemu sind nicht implementiert.
 * Nutze daher double welche immer in Software berechnet werden.
 * 
 */
#ifdef CI_TEST_IN_QEMU
    typedef double sensorsReal_t;
#else
    typedef float sensorsReal_t;
#endif

/**
 * @brief Unterstützte Sensortypen
 * 
 */
typedef enum {                      // Einheit      Feld
    SENSORS_ORIENTATION,            // quaternion   quaternion -> enu-lokal wandelt Vektoren von lokal zu world, enu-world wandelt Vektoren von world zu lokal
    SENSORS_ROTATION,               // rad/s        vector
    SENSORS_ACCELERATION,           // m/s^2        vector
    SENSORS_HEIGHT_ABOVE_SEA,       // m            -vector->z
    SENSORS_POSITION,               // m            vector
    SENSORS_GROUNDSPEED,            // m/s          vector
    SENSORS_OPTICAL_FLOW,           // rad/s        vector
    SENSORS_HEIGHT_ABOVE_GROUND,    // m            -vector->z
    SENSORS_VOLTAGE,                // V            value
    SENSORS_MAX
} sensorsType_t;

/**
 * @brief Generischer Vektor
 * 
 */
typedef struct {
    union {
        struct {
            sensorsReal_t x, y, z;
        };
        sensorsReal_t v[3];
    };
} sensorsVector_t;

/**
 * @brief Quaternion
 * 
 */
typedef struct {
    sensorsReal_t i, j, k;
    sensorsReal_t real;
} sensorsQuaternion_t;

/**
 * @brief Referenzpunkt
 * 
 */
typedef enum {
    SENSORS_ENU_LOCAL, // Datenpunkt ist relativ zum quadro gesehen
    SENSORS_ENU_WORLD, // Datenpunkt ist abolut in der Welt
    SENSORS_ENU_MAX
} sensorsENU_t;

/**
 * @brief Sensordatenpunkt
 * 
 */
typedef struct {
    int64_t timestamp;
    sensorsENU_t reference;
    union {
        sensorsReal_t value;
        sensorsVector_t vector;
        sensorsQuaternion_t quaternion;
    };
} sensorsData_t;

/**
 * @brief Pointer zu einem Verarbeitungscallback
 * Erwartet der Sensor eine Verarbeitung da er ein Event per sensorsNotify() gemeldet hat, oder sein Intervall fällig war,
 * wird sein registrierter Callback ausgeführt.
 * 
 * @param cookie Pointer zu einem optionalen Cookie der bei sensorsRegister() übergeben wurde.
 */
typedef void (*sensorsProcessCallback_t)(void*);

/**
 * @brief Verfügbare Stati
 * 
 */
typedef enum {                  // Einheit      Feld            Hinweis
    SENSORS_STATE_ORIENTATION,  // quaternion   quaternion
    SENSORS_STATE_EULER,        // rad          vector          x - roll, y - pitch, z - yaw, nur in lokaler Referenz
    SENSORS_STATE_ROTATION,     // rad/s        vector
    SENSORS_STATE_ACCELERATION, // m/s^2        vector
    SENSORS_STATE_VELOCITY,     // m/s          vector
    SENSORS_STATE_POSITION,     // m            vector          nur in world Referenz
    SENSORS_STATE_VOLTAGE,      // V            value           ohne Verwendung des Referenzpunkts
    SENSORS_STATE_MAX
} sensorsState_t;


/**
 * @brief Variablendeklarationen
 * 
 */

/* ... */


/**
 * @brief Öffentliche Funktionen
 * 
 */

/**
 * @brief Starte Sensors-Task und Fusionsalgorithmus.
 * 
 * @return true - Fehlgeschlagen, false - Erfolgreich
 */
bool sensorsStart();

/**
 * @brief Stoppe Sensors-Task.
 * 
 */
void sensorsStop();

/**
 * @brief (De-)Registriere ein Sensor.
 * 
 * @param type Sensortyp. Jeder Typ kann nur einmal gleichzeitig existieren.
 * @param callback Callback welcher ausgeführt wird wenn ein Notify erhalten wird, oder die Intervallzeit abgelaufen ist. Auf NULL
 * setzen um den Sensor zu deregistrieren.
 * @param cookie Pointer zu einem optionalen Cookie. Wird dem Callback übergeben.
 * @param interval Intervall resp. Frequenz mit der der Callback aufgerufen werden soll. 0 wenn der Sensor eventbasiert ist.
 * @return true - Registrierung nicht erfolgreich, false - Registrierung erfolgreich
 */
bool sensorsRegister(sensorsType_t type, sensorsProcessCallback_t callback, void *cookie, TickType_t interval);

/**
 * @brief Sende ein Sensorevent.
 * 
 * @param type Sensortyp der die Benachrichtigung auslöst.
 * @return true - Parameterfehler oder System nicht gestartet, false - erfolgreich
 */
bool sensorsNotify(sensorsType_t type);
bool sensorsNotifyFromISR(sensorsType_t type);

/**
 * @brief Setze Rohwerte / Sensordaten. Wird intern verarbeitet und ggf. fusioniert.
 * 
 * @note darf nur innerhalb des sensorsProcessCallback verwendet werden
 * @param type Sensortyp
 * @param data[in] neue Sensordaten
 * @return true - Daten nicht akzeptiert, false - Daten akzeptiert
 */
bool sensorsSet(sensorsType_t type, sensorsData_t *data);

/**
 * @brief Erhalte ein Status-Datenpunkt
 * 
 * @param state abgefragter Status
 * @param reference ENU lokal oder world, Bezugspunkt der Rotationen und Vektoren
 * @param data[out] wohin der Datenpunkt kopiert werden soll
 * @return true - Abfrage Fehlerhaft / noch keine Daten, false - Abfrage erfolgreich / Daten gültig
 */
bool sensorsGet(sensorsState_t state, sensorsENU_t reference, sensorsData_t *data);
