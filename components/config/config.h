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
 * - andere Components können, sofern sie den Namen wissen, die Konfiguration verändern
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
    variable = defaultValue; \
    configType_t type; \
    _Generic((variable), \
        int32_t: type = CONFIG_TYPE_INT, \
        float: type = CONFIG_TYPE_FLOAT); \
    configRegister(component, group, name, type, &(variable)); \
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

/**
 * @brief Serialisiere alle registrierten Konfigurationsvariablen in Messagepack Paket:
 * [
 *  {"component": "sensors", "groups": [
 *      {"group": "fusion", "values": {
 *          "doReset": 0,
 *          "doTest": 1
 *      }}
 *  ]},
 *  {"component":"flow","groups":[]}
 * ]
 * 
 * @param buffer[out] Messagepack-Paket (muss free'd werden)
 * @param size[out] Grösse des Puffers
 * @return true - Fehler, false - ok
 */
bool actionSerialize(char **buffer, size_t *size);

/**
 * @brief Aktualisiere Variable per Messagepack
 * Empfange ein Messagepack "cstr:component" + "cstr:group" + "cstr:name" + "float|int:value".
 * Aktualisiere Variable.
 * Erstelle ein Messagepack "cstr:component" + "cstr:group" + "cstr:name" + "float|int:value".
 * 
 * @param buffer[in,out] in - empfangenes Paket, out - zu sendendes Paket (muss free'd werden)
 * @param size[in,out] in - Grösse des empfangenen Pakets, out - Grösse des zu sendenen Pakets
 * @return true - Fehler, false - ok
 */
bool configUpdate(char **buffer, size_t *size);
