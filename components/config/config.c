/**
 * @file config.c
 * @author NikLeberg (niklaus.leuenb@gmail.com)
 * @brief Wrapper für nvs-component von esp-idf als Konfigurationsspeicher.
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

/**
 * @brief Includes
 * 
 */

#include "string.h"

#include "freertos/FreeRTOS.h"
#include "freertos/list.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "mpack.h"

#include "config.h"


/**
 * @brief Typdeklarationen
 * 
 */

typedef struct {
    configType_t type;
    void *pValue;
} configItem_t;


/**
 * @brief Variablendeklarationen
 * 
 */

static struct {
    List_t components;
} config;


/**
 * @brief Private Funktionsprototypen
 * 
 */

/**
 * @brief Erhalte gesuchtes Item aus der Liste der registrierten Einstellungen.
 * 
 * @param component Name des components
 * @param group Gruppenname
 * @param name Name der Variable
 * @param create erstelle das Item falls es nicht vorhanden ist
 * @return configItem_t* gefundenes resp. erstelltes Item, sonst NULL
 */
configItem_t *get(const char *component, const char *group, const char *name, bool create);

/**
 * @brief Suche Item in Liste
 * 
 * @param list Liste
 * @param name Suchstring
 * @param create erstelle Item falls nicht vorhanden
 * @return ListItem_t* gefundenes resp. erstelltes Item, sonst NULL
 */
ListItem_t *searchItem(List_t *list, const char *name, bool create);

/**
 * @brief Öffne NVS-Namespace "component group"
 * 
 * @param component Name des components
 * @param group Gruppenname
 * @return nvs_handle_t geöffneter Handle bei erfolg, sonst NULL
 */
nvs_handle_t openNamespace(const char *component, const char *group);

/**
 * @brief Aktualisiere Wert im NVS.
 * 
 * @param nvs Handle
 * @param name Keyname
 * @param pValue Pointer zur Variable
 * @param create falls Key-Value Paar nicht vorhande, so erstelle dieses mit intialem Wert in pValue
 * @return true - NVS error, false - erfolgreich
 */
bool update(nvs_handle_t nvs, const char *name, void *pValue, bool create);


/**
 * @brief Implementation Öffentlicher Funktionen
 * 
 */

bool configRegister(const char *component, const char *group, const char *name, configType_t type, void *pValue) {
    if (type >= CONFIG_TYPE_MAX || !pValue) return true;
    configItem_t *configItem = get(component, group, name, true);
    if (!configItem) return true;
    configItem->type = type;
    configItem->pValue = pValue;
    // Einstellung aus NVS laden
    nvs_handle_t nvs = openNamespace(component, group);
    if (!nvs) return true;
    bool failure = update(nvs, name, pValue, true);
    nvs_close(nvs);
    return failure;
}

bool configUnregister(const char *component, bool remove) {
    if (!component) return true;
    // Component
    ListItem_t *componentItem = searchItem(&config.components, component, false);
    if (!componentItem) return false;
    // jede Gruppe
    List_t *groupList = (List_t*)componentItem->xItemValue;
    if (groupList && !listLIST_IS_EMPTY(groupList)) {
        while (true) {
            ListItem_t *groupItem = listGET_HEAD_ENTRY(groupList);
            if (groupItem == listGET_END_MARKER(groupList)) break;
            // jeder Name
            List_t *nameList = (List_t*)groupItem->xItemValue;
            if (nameList && !listLIST_IS_EMPTY(nameList)) {
                while (true) {
                    ListItem_t *nameItem = listGET_HEAD_ENTRY(nameList);
                    if (nameItem == listGET_END_MARKER(nameList)) break;
                    configItem_t *configItem = (configItem_t*)nameItem->xItemValue;
                    if (configItem) {
                        if (remove) {
                            nvs_handle_t nvs = openNamespace(component, groupItem->pvOwner);
                            bool failure = (!nvs || nvs_erase_key(nvs, nameItem->pvOwner) || nvs_commit(nvs));
                            nvs_close(nvs);
                            if (failure) {
                                return true;
                            }
                        }
                        free(configItem);
                    }
                    uxListRemove(nameItem);
                    free(nameItem);
                }
                free(nameList);
            }
            uxListRemove(groupItem);
            free(groupItem);
        }
        free(groupList);
    }
    uxListRemove(componentItem);
    free(componentItem);
    return false;
}

bool configGet(const char *component, const char *group, const char *name, configType_t *type, void *pValue) {
    if (!type || !pValue) return true;
    configItem_t *configItem = get(component, group, name, false);
    if (!configItem) return true;
    *type = configItem->type;
    *(uint32_t*)pValue = *(uint32_t*)configItem->pValue;
    return false;
}

bool configSet(const char *component, const char *group, const char *name, void *pValue) {
    // aktualisiere den tatsächlichten Wert, falls Benutzer dies vergessen hat
    configItem_t *configItem = get(component, group, name, false);
    if (!configItem) return true;
    *(uint32_t*)configItem->pValue = *(uint32_t*)pValue;
    // aktualisiere in NVS permanent
    nvs_handle_t nvs = openNamespace(component, group);
    if (!nvs) return true;
    bool failure = update(nvs, name, pValue, false);
    nvs_close(nvs);
    return failure;
}

/**[
 *  {"component": "sensors", "groups": [
 *      {"group": "fusion", "values": {
 *          "doReset": 0,
 *          "doTest": 1
 *      }}
 *  ]},
 *  {"component":"flow","groups":[]}
 * ]
 *
*/
bool configSerialize(char **buffer, size_t *size) {
    mpack_writer_t writer;
    mpack_writer_init_growable(&writer, buffer, size);

    // jedes Component
    List_t *componentList = &config.components;
    mpack_start_array(&writer, listCURRENT_LIST_LENGTH(componentList));
    if (componentList && !listLIST_IS_EMPTY(componentList)) {
        ListItem_t *componentItem = listGET_HEAD_ENTRY(componentList);
        while (true) {
            mpack_start_map(&writer, 2);
            mpack_write_cstr(&writer, "component");
            mpack_write_cstr_or_nil(&writer, componentItem->pvOwner);
            mpack_write_cstr(&writer, "groups");
            // jede Gruppe
            List_t *groupList = (List_t*)componentItem->xItemValue;
            mpack_start_array(&writer, listCURRENT_LIST_LENGTH(groupList));
            if (groupList && !listLIST_IS_EMPTY(groupList)) {
                ListItem_t *groupItem = listGET_HEAD_ENTRY(groupList);
                while (true) {
                    mpack_start_map(&writer, 2);
                    mpack_write_cstr(&writer, "group");
                    mpack_write_cstr_or_nil(&writer, groupItem->pvOwner);
                    mpack_write_cstr(&writer, "values");
                    // jeder Name
                    List_t *nameList = (List_t*)groupItem->xItemValue;
                    if (nameList && !listLIST_IS_EMPTY(nameList)) {
                        mpack_start_map(&writer, listCURRENT_LIST_LENGTH(nameList));
                        ListItem_t *nameItem = listGET_HEAD_ENTRY(nameList);
                        while (true) {
                            // Key:Value Paar
                            mpack_write_cstr_or_nil(&writer, nameItem->pvOwner);
                            configItem_t *configItem = (configItem_t*)nameItem->xItemValue;
                            switch (configItem->type) {
                                case (CONFIG_TYPE_INT):
                                    mpack_write_i32(&writer, *(uint32_t*)configItem->pValue);
                                    break;
                                case (CONFIG_TYPE_FLOAT):
                                    mpack_write_float(&writer, *(float*)configItem->pValue);
                                    break;
                                default:
                                    assert(false);
                                    break;
                            }
                            nameItem = listGET_NEXT(nameItem);
                            if (nameItem == listGET_END_MARKER(nameList)) {
                                break;
                            }
                        }
                        mpack_finish_map(&writer);
                    }
                    mpack_finish_map(&writer);
                    groupItem = listGET_NEXT(groupItem);
                    if (groupItem == listGET_END_MARKER(groupList)) {
                        break;
                    }
                }
            }
            mpack_finish_array(&writer);
            mpack_finish_map(&writer);
            componentItem = listGET_NEXT(componentItem);
            if (componentItem == listGET_END_MARKER(componentList)) {
                break;
            }
        }
    }
    mpack_finish_array(&writer);

    return (mpack_writer_destroy(&writer) != mpack_ok);
}

bool configUpdate(char **buffer, size_t *size) {
    mpack_reader_t reader;
    mpack_reader_init_data(&reader, *buffer, *size);
    // Component
    const char component[16];
    mpack_expect_cstr(&reader, (char*)component, 16);
    // Group
    const char group[16];
    mpack_expect_cstr(&reader, (char*)group, 16);
    // Name
    const char name[16];
    mpack_expect_cstr(&reader, (char*)name, 16);
    // Erwarteten Typ erhalten
    configItem_t *configItem = get(component, group, name, false);
    if (!configItem) return true;
    int32_t i = 0;
    float f = 0.0;
    switch (configItem->type) {
        case (CONFIG_TYPE_INT):
            i = mpack_expect_i32(&reader);
            break;
        case (CONFIG_TYPE_FLOAT):
            f = mpack_expect_float(&reader);
            break;
        default:
            assert(false);
            break;
    }
    if (mpack_reader_destroy(&reader) != mpack_ok) return true;
    // Variabel aktualisieren
    if (configSet(component, group, name, configItem->type == CONFIG_TYPE_INT ? (void*)&i : (void*)&f)) return true;
    // Resultat zurückgeben, identisch dem empfangenen Buffer
    return false;
}


/**
 * @brief Implementation Privater Funktionen
 * 
 */

configItem_t *get(const char *component, const char *group, const char *name, bool create) {
    // Component
    ListItem_t *componentItem = searchItem(&config.components, component, create);
    if (!componentItem) return NULL;
    // Gruppe
    List_t *groupList = (List_t*)componentItem->xItemValue;
    if (!groupList) {
        groupList = calloc(1, sizeof(List_t));
        if (!groupList) return NULL;
        componentItem->xItemValue = (TickType_t)groupList;
    }
    ListItem_t *groupItem = searchItem(groupList, group, create);
    if (!groupItem) return NULL;
    // Name
    List_t *nameList = (List_t*)groupItem->xItemValue;
    if (!nameList) {
        nameList = calloc(1, sizeof(List_t));
        if (!nameList) return NULL;
        groupItem->xItemValue = (TickType_t)nameList;
    }
    ListItem_t *nameItem = searchItem(nameList, name, create);
    if (!nameItem) return NULL;
    // Config
    if (!nameItem->xItemValue && create) {
        nameItem->xItemValue = (TickType_t)calloc(1, sizeof(configItem_t));
        if (!nameItem->xItemValue) return NULL;
    }
    return (configItem_t*)nameItem->xItemValue;
}

ListItem_t *searchItem(List_t *list, const char *name, bool create) {
    if (!list || !name) return NULL;
    if (!listLIST_IS_INITIALISED(list)) {
        vListInitialise(list);
    }
    ListItem_t *item = NULL;
    // Wenn Liste nicht leer ist, so suche ob ein Eintrag mit dem gesuchten Namen schon existiert.
    if (!listLIST_IS_EMPTY(list)) {
        item = listGET_HEAD_ENTRY(list);
        while (true) {
            if (strncmp(item->pvOwner, name, 15) == 0) {
                break;
            }
            item = listGET_NEXT(item);
            if (item == listGET_END_MARKER(list)) {
                item = NULL;
                break;
            }
        }
    }
    // Liste war leer oder es wurde kein Eintrag gefunden
    if (!item && create) {
        item = calloc(1, sizeof(ListItem_t));
        if (item) {
            vListInitialiseItem(item);
            item->pvOwner = (char*)name;
            vListInsertEnd(list, item);
        }
    }
    return item;
}

nvs_handle_t openNamespace(const char *component, const char *group) {
    if (!component || !group || strnlen(component, 15) + strnlen(group, 15) > 15) return 0;
    nvs_handle_t nvs = 0;
    char namespace[16];
    strncpy(namespace, component, 15);
    strncat(namespace, group, 15 - strnlen(component, 15));
    if (nvs_open(namespace, NVS_READWRITE, &nvs) == ESP_ERR_NVS_NOT_INITIALIZED) { // lazy init
        if (nvs_flash_init()) {
            nvs_flash_erase();
            nvs_flash_init();
        }
        nvs_open(namespace, NVS_READWRITE, &nvs);
    }
    return nvs;
}

bool update(nvs_handle_t nvs, const char *name, void *pValue, bool create) {
    if (!nvs || !name || strnlen(name, 15) > 15 || !pValue) return true;
    esp_err_t err = nvs_get_u32(nvs, name, pValue);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        if (create) {
            err = nvs_set_u32(nvs, name, *(uint32_t*)pValue);
            if (err) return true;
            err = nvs_commit(nvs);
            if (err) return true;
        }
    } else if (err) return true;
    return false;
}
