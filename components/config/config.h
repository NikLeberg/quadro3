/**
 * @file config.h
 * @author NikLeberg (niklaus.leuenb@gmail.com)
 * @brief Wrapper für nvs-component von esp-idf mit zusätzlichem Remotezugriff.
 * @version 0.1
 * @date 2020-12-20
 * 
 * @copyright Copyright (c) 2020 Niklaus Leuenberger
 * 
 * - Components können ihre Werte nichtflüchtig abspeichern
 * - andere Components können, sofern sie den Namen wissen, die Kofniguration verändern
 * - Gedacht ist, dass ein component welches die Fernsteuerung implementiert alle Werte einlesen kann.
 * 
 */

#pragma once


/**
 * @brief Includes
 * 
 */

#include "esp_system.h"


/**
 * @brief Typdeklarationen
 * 
 */

/**
 * @brief Typ der gespeicherten Variabel
 * 
 */
typedef enum {
    CONFIG_TYPE_INT,
    CONFIG_TYPE_FLOAT,
    CONFIG_TYPE_MAX
} configType_t;


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
 * @brief Registriere eine Konfigurationsvariable.
 * 
 * @param component Name des components
 * @param group optionaler Gruppenname, darf zusammen mit component nur maximal 15 Zeichen lang sein
 * @param name Name der Variable
 * @param type Typ der Variable
 * @param pValue[in] Pointer zum Speicherort
 * @return true - Registrierungsfehler, false - erfolgreich
 */
bool configRegister(const char *component, const char *group, const char *name, configType_t type, void *pValue);

/**
 * @brief Ease of Use Makro zur Registrierung von Konfigurationsvariablen.
 * 
 * @param component Name des components
 * @param group optionaler Gruppenname, darf zusammen mit component nur maximal 15 Zeichen lang sein
 * @param name Name der Variable
 * @param value Speicher der Variable
 * @param defaultValue Standartwert für die Variable wenn keine Konfiguration aus dem NVS geladen werden konnte
 * @return bool gemäss configRegister()
 */
#define CONFIG_REGISTER(component, group, name, variable, defaultValue) ({ \
    bool failed; \
    variable = defaultValue; \
    configType_t type; \
    _Generic((variable), \
        int32_t: type = CONFIG_TYPE_INT, \
        float: type = CONFIG_TYPE_FLOAT); \
    failed = configRegister(component, group, name, type, &(variable)); \
    failed; \
})

/**
 * @brief Deregistriere alle Konfigurationsvariablen eines components.
 * 
 * @param component Name des components
 * @param remove Lösche Variable auch aus dem NVS
 * @return true - Registrierungsfehler, false - erfolgreich deregistriert
 */
bool configUnregister(const char *component, bool remove);

/**
 * @brief Erhalte den Wert einer Konfigurationsvariable.
 * Falls der Name unbekannt ist, kann mittels "*" jede Konfiguration abgefragt werden.
 * 
 * @param component Name des components
 * @param group Gruppenname
 * @param name Name der Variable
 * @param type[out] Typ der Variable
 * @param pValue[out] Wird
 * @return true - Fehler oder bei "*" Ende erreicht, false ohne Fehler oder bei "*" noch weitere Elemente vorhanden
 */
bool configGet(const char *component, const char *group, const char *name, configType_t *type, void *pValue);

/**
 * @brief Setze den Wert einer Konfigurationsvariable.
 * 
 * @param component Name des components
 * @param group Gruppenname
 * @param name Name der Variable
 * @param pValue[in] neuer Wert
 * @return true - fehlerhaft, false - erfolgreich
 */
bool configSet(const char *component, const char *group, const char *name, void *pValue);
