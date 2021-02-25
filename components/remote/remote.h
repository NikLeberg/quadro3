/**
 * @file remote.h
 * @author NikLeberg (niklaus.leuenb@gmail.com)
 * @brief Fernsteuerung per Website
 * @version 0.1
 * @date 2021-01-20
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
#include "esp_http_server.h"
#include "mpack.h"


/**
 * @brief Typdeklarationen
 * 
 */

/**
 * @brief Struktur für mittels EMBED_FILES eigebundene Dateien.
 * 
 */
typedef struct {
    const uint8_t *start;
    const uint8_t *end;
    const char *url;
    const char *mimeType;
} remoteFile_t;

/**
 * @brief Component Open-Callback
 * Wird die Websocket-Verbindung zum Remote aufgebaut wird dieser
 * Callback einmal aufgerufen.
 * 
 * @return esp_err_t ESP_OK oder ESP_FAIL (momentan noch ignoriert)
 */
typedef esp_err_t (*remoteWsOpenCallback_t)(void);

/**
 * @brief Component Close-Callback
 * Wird die Websocket-Verbindung zum Remote geschlossen wird dieser
 * Callback einmal aufgerufen.
 * 
 * @return esp_err_t ESP_OK oder ESP_FAIL (momentan noch ignoriert)
 */
typedef esp_err_t (*remoteWsCloseCallback_t)(void);

/**
 * @brief Component Receive-Callback
 * Wird über den Websocket eine NAchricht für dieses Component erhalten
 * so wird diesee an diesen Callback weitergeleitet.
 * 
 * @param mpack_reader_t initialisierter Mpack-Reader
 * @return esp_err_t ESP_OK oder ESP_FAIL (momentan noch ignoriert)
 */
typedef esp_err_t (*remoteWsReceiveCallback_t)(mpack_reader_t*);

/**
 * @brief Registrierungsstruktur für Vue-Components
 * 
 */
typedef struct {
    char name[10]; // Name des Components, wird als Tabnamen verwendet
    remoteFile_t file; // Quellcode des Vue-Components (nur start und end anzugeben)
    remoteWsOpenCallback_t onOpen; // wird beim Öffnen der Verbindung aufgeführt
    remoteWsCloseCallback_t onClose; // wird beim Schliessen der Verbindung ausgeführt
    remoteWsReceiveCallback_t onReceive; // Callback der Daten erhält die vom Vue-Component gesendet wurden
} remoteComponentRegistration_t;


/**
 * @brief Variablendeklarationen
 * 
 */

#define REMOTE_NUM_COMPONENTS (10)  // Anzahl maximal registrierter components, technisches Limit bei UINT8_MAX


/**
 * @brief Öffentliche Funktionen
 * 
 */

/**
 * @brief Initialisiere WiFi und starte Remote-Task.
 * 
 * @return true - Fehlgeschlagen, false - Erfolgreich
 */
bool remoteStart();

/**
 * @brief Stoppe Remote-Task und deinitialisiere WiFi.
 * 
 */
bool remoteStop();

/**
 * @brief Registriere ein remoteComponent
 * Die Datei in der Registrierungsstruktur fungiert als gebastelte .vue-Datei mit HTML,
 * CSS und JS und wird als "tab"-component eingebunden. (In einem alleinstehenden Tab dargestellt.)
 * Dem Callback wird beim Empfang von Daten ein MessagePack-Reader übergeben.
 * Zum Senden von Daten wird entweder direkt mit WsSend geantwortet (als Antwort auf Clientanfrage)
 * oder per WsSendAsync (falls der Server ohne Anfrage senden möchte). Bei beiden Funktionen muss
 * das MessagePack-Packet mittels WsCreateMpackWriter / WsDestroyMPackWriter erstellt werden.
 * Diese stellen die äussere Struktur bereit der die Enummeration im WS-Stream zulässt. Die
 * Daten die der Nutzer dem MPack hinzufügt dürfen nur ein Root-Element haben: [handle, [*]]
 * 
 * @param[out] handle Handle / Enummeration in der Websocket-Verbindung
 * @param registation statische Registrierung mit [datei].js und Callback
 * @return true - Fehler, false - Erfolgreich registriert
 */
bool remoteRegisterComponent(uint8_t *handle, remoteComponentRegistration_t *registation);

/**
 * @brief Entferne eine frühere Registrierung
 * 
 * @param handle Handle der Verbindung
 */
void remoteUnregisterComponent(uint8_t handle);

/**
 * @brief Erstelle einen MPack-Writer
 * Initialisiert ein MPack-Writer mit einem Zweierarray, dessen erster Eintrag der
 * Verbindungshandle ist. Das Zweite Element kann der Nutzer frei erstellen. Dieses
 * darf aber nur ein einziges Root-Element besitzen.
 * 
 * @param handle Handle / Enummeration der Websocket-Verbindung
 * @param[out] writer MPack-Writer der initialisiert werden soll
 * @param[out] buffer Pointer zu einem Datenpuffer, ist nach Zerstörung des Writers immernoch gültig
 * @param[out] size Länge des Datenpuffers
 * @return true - Erstellung Fehlgeschlagen, false - erfolgreich Erstellt
 */
bool remoteWsCreateMpackWriter(uint8_t handle, mpack_writer_t *writer, uint8_t **buffer, size_t *size);

/**
 * @brief Schliesse den MPack-Writer
 * 
 * @param writer MPack-Writer der zuvor mit WsCreateMpackWriter erstellt wurde
 * @return mpack_error_t bei mpack_ok enthält der zuvor definierte Puffer den Messagepack
 */
mpack_error_t remoteWsDestroyMpackWriter(mpack_writer_t *writer);

/**
 * @brief Sende Daten Synchron als Antwort auf Anfrage
 * 
 * @param mpackBuffer Puffer der eine Messagepack-Nachricht enthält, wird gefree'd
 * @param length Länge der Nachricht
 * @return esp_err_t ESP_OK oder ESP_FAIL
 */
esp_err_t remoteWsSend(uint8_t *mpackBuffer, size_t length);

/**
 * @brief Sende Daten Asychron ohne Anfrage
 * 
 * @param mpackBuffer Puffer der eine Messagepack-Nachricht enthält, wird gefree'd
 * @param length Länge der Nachricht
 * @return esp_err_t ESP_OK oder ESP_FAIL
 */
esp_err_t remoteWsSendAsync(uint8_t *mpackBuffer, size_t length);
