/**
 * @file remote.c
 * @author NikLeberg (niklaus.leuenb@gmail.com)
 * @brief Fernsteuerung per Website
 * @version 0.1
 * @date 2021-01-20
 * 
 * @copyright Copyright (c) 2021 Niklaus Leuenberger
 * 
 * - Dieses Remote Modul stellt lediglich eine Zweiwegeverbindung und eine gesamthafte Darstellung
 *   einzelner Teilmodule bereit.
 * - Zur Kommunikation wird ein geteilter Websocket verwendet.
 *   Auf diesem werden per automatische Enummeration verschiedene Verbindungen unterstützt.
 * - Die einzelnen Module stellen bereit:
 *      - ein vue.js Template (oder Mehrere) für die Darstellung
 *      - eine open Funktion, der mitgeteilt wird wenn die Verbindung vorhanden ist
 *      - eine close Funktion, der mitgeteilt wird wenn die Verbindung unterbrochen wurde
 *      - eine receive Funktion, der beim Empfang eine ausgepackte Struktur übergeben wird
 */

/**
 * @brief Includes
 * 
 */

#include "esp_netif.h"
#include "esp_wifi_default.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "lwip/apps/netbiosns.h"
#include "esp_http_server.h"
#include "mpack.h"
#include "esp_log.h"

#include "remote.h"
#include "secrets.h"

/**
 * @brief Typdeklarationen
 * 
 */

typedef struct {
    uint8_t *payload;
    size_t length;
} remoteAsyncWs_t;


/**
 * @brief Variablendeklarationen
 * 
 */

/**
 * @brief Websitedateien, direkt in Binary eingebunden per EMBED_FILES.
 * 
 */
extern const uint8_t _binary_index_html_start[];
extern const uint8_t _binary_index_html_end[];
extern const uint8_t _binary_manifest_json_start[];
extern const uint8_t _binary_manifest_json_end[];
extern const uint8_t _binary_favicon_svg_gz_start[];
extern const uint8_t _binary_favicon_svg_gz_end[];
extern const uint8_t _binary_style_css_start[];
extern const uint8_t _binary_style_css_end[];
extern const uint8_t _binary_script_js_start[];
extern const uint8_t _binary_script_js_end[];
extern const uint8_t _binary_msgpack_min_js_gz_start[]; // https://github.com/ygoe/msgpack.js
extern const uint8_t _binary_msgpack_min_js_gz_end[];
static remoteFile_t files[] = {
    {_binary_index_html_start, _binary_index_html_end,          "/",                "text/html"},
    {_binary_manifest_json_start, _binary_manifest_json_end,    "/manifest.json",   "application/json"},
    {_binary_favicon_svg_gz_start, _binary_favicon_svg_gz_end,  "/favicon.svg",     "image/svg+xml"},
    {_binary_style_css_start, _binary_style_css_end,            "/style.css",       "text/css"},
    {_binary_script_js_start, _binary_script_js_end,            "/script.js",       "text/javascript"},
    {_binary_msgpack_min_js_gz_start, _binary_msgpack_min_js_gz_end, "/msgpack.min.js",  "text/javascript"}
};
#define REMOTE_FILE_NUM (sizeof(files) / sizeof(files[0]))

static struct {
    //esp_netif_t netif;
    httpd_handle_t server;
    int ws;

    remoteComponentRegistration_t *registeredComponents[REMOTE_NUM_COMPONENTS];

    struct {
        uint8_t handle;
        vprintf_like_t defaultLog;
    } home;
} remote;


/**
 * @brief Private Funktionsprototypen
 * 
 */

/**
 * @brief Genereller Url Handler für Dateien
 * req->user_ctx muss auf eine gültige remoteFile_t-Striktur zeigen
 * 
 * @param req request von httpd
 * @return esp_err_t ESP_OK oder ESP_FAIL
 */
static esp_err_t serveFile(httpd_req_t *req);

/**
 * @brief Url Handler für Websocket-Verbindung
 * 
 * @param req request von httpd
 * @return esp_err_t ESP_OK oder ESP_FAIL
 */
static esp_err_t wsReceive(httpd_req_t *req);

/**
 * @brief Worker für asynchrones Senden von WS-Frames
 * Wird durch remoteWsSendAsync() an httpd als Task übergeben.
 * 
 * @param arg Pointer auf remoteAsyncWs_t
 */
static void wsSendAsyncWorker(void *arg);

/**
 * @brief Callback für Socket-Schliessungen
 * 
 * @param hd httpd-Instanz
 * @param sockfd Filedeskriptor des Sockets
 */
static void socketClosed(httpd_handle_t hd, int sockfd);

/**
 * @brief Url Handler für Component Lade-Anfragen
 * 
 * @param req request von httpd
 * @return esp_err_t ESP_OK oder ESP_FAIL
 */
static esp_err_t sendComponents(httpd_req_t *req);

// Home Component

/**
 * @brief Callback für WS-Close
 * 
 * @return esp_err_t immer ESP_OK
 */
static esp_err_t homeOpen();

/**
 * @brief Callback für WS-Close
 * 
 * @return esp_err_t immer ESP_OK
 */
static esp_err_t homeClose();

/**
 * @brief Logumleitung von ESP_LOGx zum remote
 * 
 * @param format printf Format Prototyp
 * @param arguments printf argumente
 * @return int anzahl geschriebener Charakter
 */
static int homeForwardLog(const char *format, va_list arguments);

/**
 * @brief Callback für WS-Receive
 * 
 * @param reader initialisierter mpack-Reader
 * @return esp_err_t immer ESP_OK
 */
static esp_err_t homeReceive(mpack_reader_t *reader);

extern const uint8_t _binary_home_js_start[];
extern const uint8_t _binary_home_js_end[];
static remoteComponentRegistration_t homeComponent = {
    .name = "Home",
    .file = {
        .start = _binary_home_js_start,
        .end = _binary_home_js_end
    },
    .onOpen = homeOpen,
    .onClose = homeClose,
    .onReceive = homeReceive
};


/**
 * @brief Implementation Öffentlicher Funktionen
 * 
 */

bool remoteStart() {
    // Starte TCP/IP Stack und lwip
    // netif darf nur einmal initialisiert werden
    static bool netifInitialized = false;
    if (!netifInitialized) {
        if (esp_netif_init() != ESP_OK) return true;
        netifInitialized = true;
    }
    /*remote.netif =*/ esp_netif_create_default_wifi_sta();
    //if (!remote.netif) return true;
    // Event Loop
    if (esp_event_loop_create_default() != ESP_OK) return true;
    // WiFi
    wifi_init_config_t wifiConfig = WIFI_INIT_CONFIG_DEFAULT();
    if (esp_wifi_init(&wifiConfig) != ESP_OK) return true;
    if (esp_wifi_set_default_wifi_sta_handlers() != ESP_OK) return true;
    if (esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK) return true;
    if (esp_wifi_set_config(ESP_IF_WIFI_STA, &(wifi_config_t){
        .sta = {
            .ssid = REMOTE_SSID,
            .password = REMOTE_PASSWORD
        }
    }) != ESP_OK) return true;
    if (esp_wifi_set_ps(WIFI_PS_NONE) != ESP_OK) return true;
    if (esp_wifi_set_storage(WIFI_STORAGE_RAM) != ESP_OK) return true;
    if (esp_wifi_start() != ESP_OK) return true;
    if (esp_wifi_connect() != ESP_OK) return true;
    // NetBIOS
    netbiosns_init();
    netbiosns_set_name("quadro3");
    // Httpd Server
    httpd_config_t httpdConfig = HTTPD_DEFAULT_CONFIG();
    httpdConfig.close_fn = socketClosed;
    if (httpd_start(&remote.server, &httpdConfig) != ESP_OK) return true;
    // URIs
    for (size_t i = 0; i < REMOTE_FILE_NUM; ++i) {
        if (httpd_register_uri_handler(remote.server, &(httpd_uri_t){
            .handler = serveFile,
            .method = HTTP_GET,
            .uri = files[i].url,
            .user_ctx = &files[i]
        }) != ESP_OK) return true;
    }
    // Websockets
    if (httpd_register_uri_handler(remote.server, &(httpd_uri_t){
        .handler = wsReceive,
        .method = HTTP_GET,
        .uri = "/ws",
        .is_websocket = true
    }) != ESP_OK) return true;
    // Components
    if (httpd_register_uri_handler(remote.server, &(httpd_uri_t){
        .handler = sendComponents,
        .method = HTTP_GET,
        .uri = "/component"
    }) != ESP_OK) return true;
    // Home Component
    remoteRegisterComponent(&remote.home.handle, &homeComponent);
    return false;
}

bool remoteStop() {
    // Httpd Server
    if (httpd_stop(remote.server) != ESP_OK) return true;
    // NetBIOS
    netbiosns_stop();
    // WiFi
    if (esp_wifi_disconnect() != ESP_OK) return true;
    if (esp_wifi_stop() != ESP_OK) return true;
    if (esp_wifi_deinit() != ESP_OK) return true;
    // Event Loop
    if (esp_event_loop_delete_default() != ESP_OK) return true;
    // Netif
    // netif kann (noch) nicht deinitialisiert werden
    //if (esp_wifi_clear_default_wifi_driver_and_handlers(remote.netif) != ESP_OK) return true;
    //esp_netif_deinit();
    return false;
}

bool remoteRegisterComponent(uint8_t *handle, remoteComponentRegistration_t *registation) {
    if (!handle || !registation || !registation->file.start || !registation->file.end) {
        return true;
    }
    registation->file.mimeType = "text/javascript";
    *handle = 0;
    for (uint8_t i = 0; i < REMOTE_NUM_COMPONENTS; ++i) {
        if (remote.registeredComponents[i]) {
            continue;
        }
        remote.registeredComponents[i] = registation;
        *handle = i + 1;
        break;
    }
    return true;
}

void remoteUnregisterComponent(uint8_t handle) {
    if (handle > 0 && handle <= REMOTE_NUM_COMPONENTS) {
        remote.registeredComponents[handle - 1] = NULL;
    }
    return;
}

bool remoteWsCreateMpackWriter(uint8_t handle, mpack_writer_t *writer, uint8_t **buffer, size_t *size) {
    if (handle == 0 || handle > REMOTE_NUM_COMPONENTS || !writer || !buffer || !size) {
        return true;
    }
    // Format: [handle, [*]]
    mpack_writer_init_growable(writer, (char**)buffer, size);
    mpack_start_array(writer, 2);
    // Handle / Enummeration der Verbindung ist immer erstes Element
    mpack_write_u8(writer, handle);
    return false;
}

mpack_error_t remoteWsDestroyMpackWriter(mpack_writer_t *writer) {
    mpack_finish_array(writer);
    return mpack_writer_destroy(writer);
}

esp_err_t remoteWsSend(uint8_t *mpackBuffer, size_t length) {
    if (!remote.ws || !mpackBuffer) {
        return ESP_FAIL;
    }
    httpd_ws_frame_t frame = {.payload = mpackBuffer, .len = length, .type = HTTPD_WS_TYPE_BINARY};
    return httpd_ws_send_frame_async(remote.server, remote.ws, &frame);
}

esp_err_t remoteWsSendAsync(uint8_t *mpackBuffer, size_t length) {
    if (!remote.ws || !mpackBuffer) {
        return ESP_FAIL;
    }
    remoteAsyncWs_t *async = malloc(sizeof(remoteAsyncWs_t));
    if (!async) {
        return ESP_FAIL;
    }
    async->payload = mpackBuffer;
    async->length = length;
    return httpd_queue_work(remote.server, wsSendAsyncWorker, (void*)async);
}


/**
 * @brief Implementation Privater Funktionen
 * 
 */

static esp_err_t serveFile(httpd_req_t *req) {
    remoteFile_t *file = (remoteFile_t*) req->user_ctx;
    if (!file || !file->start || !file->end) return ESP_FAIL;
    // Prüfe auf gzip-DEFLATE Magic-Number
    if (*file->start == 0x1f && *(file->start + 1) == 0x8b && *(file->start + 2) == 0x08) { // Magic-Nummer von gzip-DEFLATE
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    }
    // Setze Datentyp
    if (file->mimeType) {
        httpd_resp_set_type(req, file->mimeType);
    }
    // Datei senden
    httpd_resp_send(req, (const char*)file->start, (size_t)file->end - (size_t)file->start);
    return ESP_OK;
}

static esp_err_t wsReceive(httpd_req_t *req) {
    // Modifiziertes httpd_uri.c::334 ruft URI-Callback bereits nach erfolgreichem Handshake auf.
    // Erkennbar ist dies nur dadurch, dass req->method ein HTTP_GET ist.
    if (req->method == HTTP_GET) {
        int fd = httpd_req_to_sockfd(req);
        // Durch den SocketClose Callback in httpd wird das Schliessen des aktiven WS-Sockets erkannt
        // und alle onClose-Callbacks aufgerufen. Wird durch z.B. einen zweiten Remote eine weitere
        // Verbindung geöffnet so übernimmt dieser als "aktive Verbindung". Hierzu werden zuerst alle
        // onClose und dann alle onOpen Callbacks ausgeführt. Ein Schliessen des nun inaktiven WS
        // triggert kein onClose aufruf.
        if (remote.ws) {
            ESP_LOGW("remote", "new WS connection fd:%i overwrites previous WS fd:%i", fd, remote.ws);
            for (uint8_t i = 0; i < REMOTE_NUM_COMPONENTS; ++i) {
                if (remote.registeredComponents[i] && remote.registeredComponents[i]->onClose) {
                    remote.registeredComponents[i]->onClose();
                }
            }
            // alte Verbindung schliessen
            httpd_sess_trigger_close(remote.server, remote.ws);
        }
        // Filedescriptor des zugrundeliegenden TCP Sockets speichern
        remote.ws = fd;
        // führe alle onOpen-Callbacks aus
        for (uint8_t i = 0; i < REMOTE_NUM_COMPONENTS; ++i) {
            if (remote.registeredComponents[i] && remote.registeredComponents[i]->onOpen) {
                remote.registeredComponents[i]->onOpen();
            }
        }
        // Callback wurde nur aufgerufen um mitzuteilen, dass der Handshake abgeschlossen ist,
        // es sind keine Frames zu empfangen.
        return ESP_OK;
    }
    // erfrage die Länge des Frames
    httpd_ws_frame_t frame = {0};
    if (httpd_ws_recv_frame(req, &frame, 0) != ESP_OK) {
        ESP_LOGE("remote", "could not get length of WS frame");
        return ESP_FAIL;
    }
    uint8_t *buffer = calloc(1, frame.len + 1);
    if (!buffer) {
        return ESP_ERR_NO_MEM;
    }
    frame.payload = buffer;
    // empfange vollen Frame
    if (httpd_ws_recv_frame(req, &frame, frame.len) != ESP_OK) {
        free(buffer);
        return ESP_FAIL;
    }
    // mpack reader erstellen, Handle lesen und an registrierten Callback weiterleiten
    mpack_reader_t reader;
    mpack_reader_init_data(&reader, (const char*)frame.payload, frame.len);
    mpack_expect_array_match(&reader, 2);
    uint8_t handle = mpack_expect_u8_max(&reader, REMOTE_NUM_COMPONENTS);
    ESP_LOGI("remote","WS rx handle: %u", handle);
    esp_err_t err = ESP_FAIL;
    uint8_t i = handle - 1;
    if (handle && remote.registeredComponents[i] && remote.registeredComponents[i]->onReceive) {
        err = remote.registeredComponents[i]->onReceive(&reader);
    }
    if (err != ESP_OK) {
        mpack_reader_flag_error(&reader, mpack_error_io);
    }
    mpack_done_array(&reader);
    mpack_reader_destroy(&reader);
    free(buffer);
    return ESP_OK;
}

static void wsSendAsyncWorker(void *arg) {
    remoteAsyncWs_t *async = (remoteAsyncWs_t*)arg;
    httpd_ws_frame_t frame = {.payload = async->payload, .len = async->length, .type = HTTPD_WS_TYPE_BINARY};
    httpd_ws_send_frame_async(remote.server, remote.ws, &frame);
    MPACK_FREE(async->payload);
    free(async);
}

static void socketClosed(httpd_handle_t hd, int sockfd) {
    if (remote.ws == sockfd) {
        remote.ws = 0;
        ESP_LOGE("remote", "active WS was closed");
        // unser aktueller Websocket wurde geschlossen
        // rufe alle onClose-Callbacks auf
        for (uint8_t i = 0; i < REMOTE_NUM_COMPONENTS; ++i) {
            if (remote.registeredComponents[i] && remote.registeredComponents[i]->onClose) {
                remote.registeredComponents[i]->onClose();
            }
        }
    }
    return;
}

static esp_err_t sendComponents(httpd_req_t *req) {
    // Wenn per Querry /component?XYZ nach einem einzelnen component gefragt
    // wird, suche nach ihm und sende es.
    size_t querryLength = httpd_req_get_url_query_len(req) + 1;
    if (querryLength > 1) {
        esp_err_t ret = ESP_FAIL;
        char *querry = malloc(querryLength);
        if (querry && httpd_req_get_url_query_str(req, querry, querryLength) == ESP_OK) {
            ESP_LOGI("remote", "component querry: %s", querry);
            for (uint8_t i = 0; i < REMOTE_NUM_COMPONENTS; ++i) {
                if (remote.registeredComponents[i] && strcmp(querry, remote.registeredComponents[i]->name) == 0) {
                    req->user_ctx = (void*)&remote.registeredComponents[i]->file;
                    ret = serveFile(req);
                    break;
                }
            }
            free(querry);
        }
        return ret;
    }
    // Anderenfalls, ohne Querry, sende ein Messagepack mit einem
    // Array an Strings der gültigen componenten-Namen.
    // ["name1", "name2", "", "name4"] Handle = Index + 1
    mpack_writer_t writer;
    uint8_t *buffer;
    size_t length;
    mpack_writer_init_growable(&writer, (char**)&buffer, &length);
    mpack_start_array(&writer, REMOTE_NUM_COMPONENTS);
    for (uint8_t i = 0; i < REMOTE_NUM_COMPONENTS; ++i) {
        if (remote.registeredComponents[i]) {
            mpack_write_cstr(&writer, remote.registeredComponents[i]->name);
        } else {
            mpack_write_nil(&writer);
        }
    }
    mpack_finish_array(&writer);
    bool success = mpack_writer_destroy(&writer) == mpack_ok;
    if (success) {
        httpd_resp_set_type(req, "application/octet-stream");
        httpd_resp_send(req, (const char*)buffer, length);
    }
    MPACK_FREE(buffer);
    return success ? ESP_OK : ESP_FAIL;
}


/**
 * @brief Home Vue-Component
 * 
 */

static esp_err_t homeOpen() {
    // Umleiten von ESP_LOG
    remote.home.defaultLog = esp_log_set_vprintf(&homeForwardLog);
    ESP_LOGI("remote-home", "open");
    // Infos senden
    // Format: ["gcc", "idf", "app"]
    mpack_writer_t writer;
    uint8_t *mpackBuffer;
    size_t mpackBufferLength;
    remoteWsCreateMpackWriter(remote.home.handle, &writer, &mpackBuffer, &mpackBufferLength);
    mpack_start_array(&writer, 3);
    // GCC Version
    char gccVersion[32] = {'\0'};
    size_t gccVersionLength = snprintf(gccVersion, 32, "GCC Version: %i.%i.%i", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__);
    mpack_write_utf8(&writer, gccVersion, gccVersionLength);
    // esp-idf Version
    char idfVersion[64] = {'\0'};
    size_t idfVersionLength = snprintf(idfVersion, 64, "IDF Version: %s", esp_get_idf_version());
    mpack_write_utf8(&writer, idfVersion, idfVersionLength);
    // app Version
    char appVersion[64] = {'\0'};
    size_t appVersionLength = snprintf(appVersion, 64, "APP Version: %s - %s", __DATE__, __TIME__);
    mpack_write_utf8(&writer, appVersion, appVersionLength);
    mpack_finish_array(&writer);
    if (remoteWsDestroyMpackWriter(&writer) == mpack_ok) {
        remoteWsSendAsync(mpackBuffer, mpackBufferLength);
    }
    return ESP_OK;
}

static esp_err_t homeClose() {
    // Umleitung von ESP_LOG rückgängig machen
    esp_log_set_vprintf(remote.home.defaultLog);
    remote.home.defaultLog = NULL;
    ESP_LOGI("remote-home", "close");
    return ESP_OK;
}

static int homeForwardLog(const char *format, va_list arguments) {
    size_t strLength = vsnprintf(NULL, 0, format, arguments);
    char *str = malloc(strLength + 1);
    if (str) {
        vsnprintf(str, strLength, format, arguments);
        // Format: "Loglinie"
        mpack_writer_t writer;
        uint8_t *mpackBuffer;
        size_t mpackBufferLength;
        remoteWsCreateMpackWriter(remote.home.handle, &writer, &mpackBuffer, &mpackBufferLength);
        mpack_write_utf8(&writer, str, strLength);
        free(str);
        if (remoteWsDestroyMpackWriter(&writer) == mpack_ok) {
            remoteWsSendAsync(mpackBuffer, mpackBufferLength);
        }
    }
    return remote.home.defaultLog(format, arguments);
}

static esp_err_t homeReceive(mpack_reader_t *reader) {
    int counter = mpack_expect_int(reader);
    if (mpack_reader_error(reader) != mpack_ok) {
        ESP_LOGE("remote-home", "mpack error");
    } else {
        ESP_LOGI("remote-home", "counter: %i", counter);
    }
    return ESP_FAIL;
}
