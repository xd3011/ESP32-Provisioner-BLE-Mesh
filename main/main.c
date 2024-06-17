/* main.c - Application main entry point */

/*
 * SPDX-FileCopyrightText: 2017 Intel Corporation
 * SPDX-FileContributor: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "app_mqtt.h"
#include "ble_mesh_example_init.h"
#include "ble_mesh_example_nvs.h"
#include "cJSON.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "mqtt_client.h"
#include "nvs.h"
#include "nvs_flash.h"

// -------------WIFI AND MQTT-------------
/* The examples use WiFi configuration that you can set via project
   configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID "TFlash"
#define EXAMPLE_ESP_WIFI_PASS "30112002"
#define EXAMPLE_ESP_MAXIMUM_RETRY 5
#define CONFIG_ESP_WIFI_AUTH_OPEN 1

#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about
 * two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static int s_retry_num = 0;

// -------------WIFI AND MQTT-------------
#define MAX_DEVICES 100

static char found_devices[MAX_DEVICES][18];  // Array to store MAC addresses
static int num_devices = 0;                  // Number of devices found

int enable_provisioner = 0;
#define MQTT_ACTION_CREATE_GATEWAY 1
#define MQTT_ACTION_ENABLE_PROVISIONER 2
#define MQTT_ACTION_DISENABLE_PROVISIONER 3
#define MQTT_ACTION_PROVISION 4
#define MQTT_ACTION_CONTROL 5

#define TAG "EXAMPLE"

#define LED_PIN GPIO_NUM_2

#define LED_OFF 0x0
#define LED_ON 0x1

#define CID_ESP 0x02E5

#define PROV_OWN_ADDR 0x0001

#define MSG_SEND_TTL 16
#define MSG_TIMEOUT 0
#define MSG_ROLE ROLE_PROVISIONER

#define COMP_DATA_PAGE_0 0x00

#define APP_KEY_IDX 0x0000
#define APP_KEY_OCTET 0x12

static uint8_t dev_uuid[16];

typedef struct {
    uint8_t uuid[16];
    uint16_t unicast;
    uint8_t elem_num;
    uint8_t onoff;
} esp_ble_mesh_node_info_t;

static esp_ble_mesh_node_info_t nodes[CONFIG_BLE_MESH_MAX_PROV_NODES] = {0};

static struct esp_ble_mesh_key {
    uint16_t net_idx;
    uint16_t app_idx;
    uint8_t app_key[16];
} prov_key;

static esp_ble_mesh_client_t config_client;
static esp_ble_mesh_client_t onoff_client;

static esp_ble_mesh_cfg_srv_t config_server = {
    /* 3 transmissions with 20ms interval */
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay = ESP_BLE_MESH_RELAY_DISABLED,
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
#else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
    .default_ttl = 16,
};

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_CFG_CLI(&config_client),
    ESP_BLE_MESH_MODEL_GEN_ONOFF_CLI(NULL, &onoff_client),
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, ESP_BLE_MESH_MODEL_NONE),
};

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .element_count = ARRAY_SIZE(elements),
    .elements = elements,
};

static esp_ble_mesh_prov_t provision = {
    .prov_uuid = dev_uuid,
    .prov_unicast_addr = PROV_OWN_ADDR,
    .prov_start_address = 0x0005,
    .prov_attention = 0x00,
    .prov_algorithm = 0x00,
    .prov_pub_key_oob = 0x00,
    .prov_static_oob_val = NULL,
    .prov_static_oob_len = 0x00,
    .flags = 0x00,
    .iv_index = 0x00,
};

void send_control_led_switch_state(uint16_t addr, uint16_t onoff) {
    esp_ble_mesh_generic_client_set_state_t set = {0};
    esp_ble_mesh_client_common_param_t common = {0};
    esp_err_t err = ESP_OK;

    common.opcode = ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK;
    common.model = onoff_client.model;
    common.ctx.net_idx = prov_key.net_idx;
    common.ctx.app_idx = prov_key.app_idx;
    common.ctx.addr = addr; /* to all nodes */
    common.ctx.send_ttl = 16;
    common.msg_timeout =
        0; /* 0 indicates that timeout value from menuconfig will be used */
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 2, 0)
    common.msg_role = ROLE_NODE;
#endif

    set.onoff_set.op_en = false;
    set.onoff_set.onoff = onoff;

    err = esp_ble_mesh_generic_client_set_state(&common, &set);
    if (err) {
        ESP_LOGE(TAG, "Send Generic OnOff Set Unack failed");
        return;
    }
}

static esp_err_t example_ble_mesh_store_node_info(const uint8_t uuid[16],
                                                  uint16_t unicast,
                                                  uint8_t elem_num,
                                                  uint8_t onoff_state) {
    int i;

    if (!uuid || !ESP_BLE_MESH_ADDR_IS_UNICAST(unicast)) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Judge if the device has been provisioned before */
    for (i = 0; i < ARRAY_SIZE(nodes); i++) {
        if (!memcmp(nodes[i].uuid, uuid, 16)) {
            ESP_LOGW(TAG, "%s: reprovisioned device 0x%04x", __func__, unicast);
            nodes[i].unicast = unicast;
            nodes[i].elem_num = elem_num;
            nodes[i].onoff = onoff_state;
            return ESP_OK;
        }
    }

    for (i = 0; i < ARRAY_SIZE(nodes); i++) {
        if (nodes[i].unicast == ESP_BLE_MESH_ADDR_UNASSIGNED) {
            memcpy(nodes[i].uuid, uuid, 16);
            nodes[i].unicast = unicast;
            nodes[i].elem_num = elem_num;
            nodes[i].onoff = onoff_state;
            return ESP_OK;
        }
    }

    return ESP_FAIL;
}

static esp_ble_mesh_node_info_t *example_ble_mesh_get_node_info(
    uint16_t unicast) {
    int i;

    if (!ESP_BLE_MESH_ADDR_IS_UNICAST(unicast)) {
        return NULL;
    }

    for (i = 0; i < ARRAY_SIZE(nodes); i++) {
        ESP_LOGI(TAG, "Unicast address: 0x%04x, addr: 0x%04x\n",
                 nodes[i].unicast, unicast);
        if (nodes[i].unicast <= unicast &&
            nodes[i].unicast + nodes[i].elem_num > unicast) {
            return &nodes[i];
        }
    }

    return NULL;
}

static esp_err_t example_ble_mesh_set_msg_common(
    esp_ble_mesh_client_common_param_t *common, esp_ble_mesh_node_info_t *node,
    esp_ble_mesh_model_t *model, uint32_t opcode) {
    if (!common || !node || !model) {
        return ESP_ERR_INVALID_ARG;
    }

    common->opcode = opcode;
    common->model = model;
    common->ctx.net_idx = prov_key.net_idx;
    common->ctx.app_idx = prov_key.app_idx;
    common->ctx.addr = node->unicast;
    common->ctx.send_ttl = MSG_SEND_TTL;
    common->msg_timeout = MSG_TIMEOUT;
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 2, 0)
    common->msg_role = MSG_ROLE;
#endif

    return ESP_OK;
}

static esp_err_t prov_complete(char *addr_str, int node_idx,
                               const esp_ble_mesh_octet16_t uuid,
                               uint16_t unicast, uint8_t elem_num,
                               uint16_t net_idx) {
    esp_ble_mesh_client_common_param_t common = {0};
    esp_ble_mesh_cfg_client_get_state_t get_state = {0};
    esp_ble_mesh_node_info_t *node = NULL;
    char name[11] = {0};
    int err;

    ESP_LOGI(TAG,
             "node index: 0x%x, unicast address: 0x%02x, element num: %d, "
             "netkey index: 0x%02x",
             node_idx, unicast, elem_num, net_idx);
    ESP_LOGI(TAG, "device uuid: %s", bt_hex(uuid, 16));

    sprintf(name, "%s%d", "NODE-", node_idx);
    err = esp_ble_mesh_provisioner_set_node_name(node_idx, name);
    if (err) {
        ESP_LOGE(TAG, "%s: Set node name failed", __func__);
        return ESP_FAIL;
    }

    // ------------------------------
    // Publisher callback to server cloud
    char mqtt_message[128];
    snprintf(mqtt_message, sizeof(mqtt_message),
             "{\"unicast_addr\":%d, \"mac_address\":\"%s\"}", unicast,
             addr_str);
    // Publish the data via MQTT
    int msg_id = esp_mqtt_client_publish(get_mqtt_client(), "home_iot/create",
                                         mqtt_message, 0, 0, 0);
    ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
    // -------------------------------

    err = example_ble_mesh_store_node_info(uuid, unicast, elem_num, LED_OFF);
    if (err) {
        ESP_LOGE(TAG, "%s: Store node info failed", __func__);
        return ESP_FAIL;
    }

    node = example_ble_mesh_get_node_info(unicast);
    if (!node) {
        ESP_LOGE(TAG, "%s: Get node info failed", __func__);
        return ESP_FAIL;
    }

    example_ble_mesh_set_msg_common(&common, node, config_client.model,
                                    ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET);
    get_state.comp_data_get.page = COMP_DATA_PAGE_0;
    err = esp_ble_mesh_config_client_get_state(&common, &get_state);
    if (err) {
        ESP_LOGE(TAG, "%s: Send config comp data get failed", __func__);
        return ESP_FAIL;
    }

    return ESP_OK;
}

static void prov_link_open(esp_ble_mesh_prov_bearer_t bearer) {
    ESP_LOGI(TAG, "%s link open",
             bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
}

static void prov_link_close(esp_ble_mesh_prov_bearer_t bearer, uint8_t reason) {
    ESP_LOGI(TAG, "%s link close, reason 0x%02x",
             bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT", reason);
}

static bool device_found(const char *addr) {
    for (int i = 0; i < num_devices; i++) {
        if (strcmp(found_devices[i], addr) == 0) {
            return true;
        }
    }
    return false;
}

static void add_device(const char *addr) {
    if (num_devices < MAX_DEVICES) {
        strncpy(found_devices[num_devices], addr, 18);
        num_devices++;
    }
}

static void reset_devices() {
    for (int i = 0; i < num_devices; i++) {
        memset(found_devices[i], 0, sizeof(found_devices[i]));
    }
    num_devices = 0;
}

static void recv_unprov_adv_pkt(uint8_t dev_uuid[16], uint8_t addr[BD_ADDR_LEN],
                                esp_ble_mesh_addr_type_t addr_type,
                                uint16_t oob_info, uint8_t adv_type,
                                esp_ble_mesh_prov_bearer_t bearer) {
    /* Due to the API esp_ble_mesh_provisioner_set_dev_uuid_match, Provisioner
     * will only use this callback to report the devices, whose device UUID
     * starts with 0xdd & 0xdd, to the application layer.
     */
    // Check device stored
    char addr_str[18];
    snprintf(addr_str, sizeof(addr_str), "%02X:%02X:%02X:%02X:%02X:%02X",
             addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    if (!device_found(addr_str)) {
        add_device(addr_str);

        // Printf Device Unprovision
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_RECV_UNPROV_ADV_PKT_EVT");
        ESP_LOGI(TAG, "address: %s, address type: %d, adv type: %d",
                 bt_hex(addr, BD_ADDR_LEN), addr_type, adv_type);
        ESP_LOGI(TAG, "device uuid: %s", bt_hex(dev_uuid, 16));
        ESP_LOGI(TAG, "oob info: %d, bearer: %s", oob_info,
                 (bearer & ESP_BLE_MESH_PROV_ADV) ? "PB-ADV" : "PB-GATT");

        // ------------------------------

        // Get the MAC address of the provisioner
        uint8_t mac[6];
        esp_wifi_get_mac(ESP_IF_WIFI_STA, mac);
        // Convert the MAC address to a string
        char mac_str[32];
        snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

        // Publisher callback to server cloud
        char mqtt_message[256];
        snprintf(mqtt_message, sizeof(mqtt_message),
                 "{\"addr\":\"%s\", \"addr_type\":%d, \"dev_uuid\":\"%s\", "
                 "\"oob_info\":%d, \"bearer\":%d, \"gateway\":\"%s\"}",
                 addr_str, addr_type, bt_hex(dev_uuid, 16), oob_info, bearer,
                 mac_str);

        // Publish the data via MQTT
        int msg_id = esp_mqtt_client_publish(
            get_mqtt_client(), "home_iot/gatewayScan", mqtt_message, 0, 0, 0);
        ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
        // -------------------------------
    }
    return;
}

static void mqtt_action_provision(uint8_t dev_uuid[16],
                                  uint8_t addr[BD_ADDR_LEN],
                                  esp_ble_mesh_addr_type_t addr_type,
                                  uint16_t oob_info,
                                  esp_ble_mesh_prov_bearer_t bearer) {
    esp_ble_mesh_unprov_dev_add_t add_dev = {0};
    int err;

    memcpy(add_dev.addr, addr, BD_ADDR_LEN);
    add_dev.addr_type = (esp_ble_mesh_addr_type_t)addr_type;
    memcpy(add_dev.uuid, dev_uuid, 16);
    add_dev.oob_info = oob_info;
    add_dev.bearer = (esp_ble_mesh_prov_bearer_t)bearer;
    /* Note: If unprovisioned device adv packets have not been received, we
       should not add device with ADD_DEV_START_PROV_NOW_FLAG set. */
    err = esp_ble_mesh_provisioner_add_unprov_dev(
        &add_dev, (esp_ble_mesh_dev_add_flag_t)(ADD_DEV_RM_AFTER_PROV_FLAG |
                                                ADD_DEV_START_PROV_NOW_FLAG |
                                                ADD_DEV_FLUSHABLE_DEV_FLAG));
    if (err) {
        ESP_LOGE(TAG, "%s: Add unprovisioned device into queue failed",
                 __func__);
    }
    return;
}

static void example_ble_mesh_provisioning_cb(
    esp_ble_mesh_prov_cb_event_t event, esp_ble_mesh_prov_cb_param_t *param) {
    switch (event) {
        case ESP_BLE_MESH_PROVISIONER_PROV_ENABLE_COMP_EVT:
            ESP_LOGI(
                TAG,
                "ESP_BLE_MESH_PROVISIONER_PROV_ENABLE_COMP_EVT, err_code %d",
                param->provisioner_prov_enable_comp.err_code);
            break;
        case ESP_BLE_MESH_PROVISIONER_PROV_DISABLE_COMP_EVT:
            ESP_LOGI(
                TAG,
                "ESP_BLE_MESH_PROVISIONER_PROV_DISABLE_COMP_EVT, err_code %d",
                param->provisioner_prov_disable_comp.err_code);
            break;
        case ESP_BLE_MESH_PROVISIONER_RECV_UNPROV_ADV_PKT_EVT:
            if (enable_provisioner == 1) {
                recv_unprov_adv_pkt(
                    param->provisioner_recv_unprov_adv_pkt.dev_uuid,
                    param->provisioner_recv_unprov_adv_pkt.addr,
                    param->provisioner_recv_unprov_adv_pkt.addr_type,
                    param->provisioner_recv_unprov_adv_pkt.oob_info,
                    param->provisioner_recv_unprov_adv_pkt.adv_type,
                    param->provisioner_recv_unprov_adv_pkt.bearer);
            }
            break;
        case ESP_BLE_MESH_PROVISIONER_PROV_LINK_OPEN_EVT:
            prov_link_open(param->provisioner_prov_link_open.bearer);
            break;
        case ESP_BLE_MESH_PROVISIONER_PROV_LINK_CLOSE_EVT:
            prov_link_close(param->provisioner_prov_link_close.bearer,
                            param->provisioner_prov_link_close.reason);
            break;
        case ESP_BLE_MESH_PROVISIONER_PROV_COMPLETE_EVT:
            esp_ble_mesh_node_t *node =
                esp_ble_mesh_provisioner_get_node_with_uuid(
                    param->provisioner_prov_complete.device_uuid);
            char addr_str[18];
            if (node) {
                sprintf(addr_str, "%02X:%02X:%02X:%02X:%02X:%02X",
                        node->addr[0], node->addr[1], node->addr[2],
                        node->addr[3], node->addr[4], node->addr[5]);
            }
            prov_complete(addr_str, param->provisioner_prov_complete.node_idx,
                          param->provisioner_prov_complete.device_uuid,
                          param->provisioner_prov_complete.unicast_addr,
                          param->provisioner_prov_complete.element_num,
                          param->provisioner_prov_complete.netkey_idx);
            break;
        case ESP_BLE_MESH_PROVISIONER_ADD_UNPROV_DEV_COMP_EVT:
            ESP_LOGI(
                TAG,
                "ESP_BLE_MESH_PROVISIONER_ADD_UNPROV_DEV_COMP_EVT, err_code %d",
                param->provisioner_add_unprov_dev_comp.err_code);
            break;
        case ESP_BLE_MESH_PROVISIONER_SET_DEV_UUID_MATCH_COMP_EVT:
            ESP_LOGI(TAG,
                     "ESP_BLE_MESH_PROVISIONER_SET_DEV_UUID_MATCH_COMP_EVT, "
                     "err_code %d",
                     param->provisioner_set_dev_uuid_match_comp.err_code);
            break;
        case ESP_BLE_MESH_PROVISIONER_SET_NODE_NAME_COMP_EVT: {
            ESP_LOGI(
                TAG,
                "ESP_BLE_MESH_PROVISIONER_SET_NODE_NAME_COMP_EVT, err_code %d",
                param->provisioner_set_node_name_comp.err_code);
            if (param->provisioner_set_node_name_comp.err_code == ESP_OK) {
                const char *name = NULL;
                name = esp_ble_mesh_provisioner_get_node_name(
                    param->provisioner_set_node_name_comp.node_index);
                if (!name) {
                    ESP_LOGE(TAG, "Get node name failed");
                    return;
                }
                ESP_LOGI(TAG, "Node %d name is: %s",
                         param->provisioner_set_node_name_comp.node_index,
                         name);
            }
            break;
        }
        case ESP_BLE_MESH_PROVISIONER_ADD_LOCAL_APP_KEY_COMP_EVT: {
            ESP_LOGI(TAG,
                     "ESP_BLE_MESH_PROVISIONER_ADD_LOCAL_APP_KEY_COMP_EVT, "
                     "err_code %d",
                     param->provisioner_add_app_key_comp.err_code);
            if (param->provisioner_add_app_key_comp.err_code == ESP_OK) {
                esp_err_t err = 0;
                prov_key.app_idx = param->provisioner_add_app_key_comp.app_idx;
                err = esp_ble_mesh_provisioner_bind_app_key_to_local_model(
                    PROV_OWN_ADDR, prov_key.app_idx,
                    ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_CLI, ESP_BLE_MESH_CID_NVAL);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Provisioner bind local model appkey failed");
                    return;
                }
            }
            break;
        }
        case ESP_BLE_MESH_PROVISIONER_BIND_APP_KEY_TO_MODEL_COMP_EVT:
            ESP_LOGI(TAG,
                     "ESP_BLE_MESH_PROVISIONER_BIND_APP_KEY_TO_MODEL_COMP_EVT, "
                     "err_code %d",
                     param->provisioner_bind_app_key_to_model_comp.err_code);
            break;
        default:
            break;
    }

    return;
}

static void example_ble_mesh_config_client_cb(
    esp_ble_mesh_cfg_client_cb_event_t event,
    esp_ble_mesh_cfg_client_cb_param_t *param) {
    esp_ble_mesh_client_common_param_t common = {0};
    esp_ble_mesh_node_info_t *node = NULL;
    uint32_t opcode;
    uint16_t addr;
    int err;

    opcode = param->params->opcode;
    addr = param->params->ctx.addr;

    ESP_LOGI(TAG,
             "%s, error_code = 0x%02x, event = 0x%02x, addr: 0x%04x, opcode: "
             "0x%04" PRIx32,
             __func__, param->error_code, event, param->params->ctx.addr,
             opcode);

    if (param->error_code) {
        ESP_LOGE(TAG, "Send config client message failed, opcode 0x%04" PRIx32,
                 opcode);
        return;
    }

    node = example_ble_mesh_get_node_info(addr);
    if (!node) {
        ESP_LOGE(TAG, "%s: Get node info failed", __func__);
        return;
    }

    switch (event) {
        case ESP_BLE_MESH_CFG_CLIENT_GET_STATE_EVT:
            switch (opcode) {
                case ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET: {
                    ESP_LOGI(TAG, "composition data 1 %s",
                             bt_hex(param->status_cb.comp_data_status
                                        .composition_data->data,
                                    param->status_cb.comp_data_status
                                        .composition_data->len));
                    esp_ble_mesh_cfg_client_set_state_t set_state = {0};
                    example_ble_mesh_set_msg_common(
                        &common, node, config_client.model,
                        ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD);
                    set_state.app_key_add.net_idx = prov_key.net_idx;
                    set_state.app_key_add.app_idx = prov_key.app_idx;
                    memcpy(set_state.app_key_add.app_key, prov_key.app_key, 16);
                    err = esp_ble_mesh_config_client_set_state(&common,
                                                               &set_state);
                    if (err) {
                        ESP_LOGE(TAG, "%s: Config AppKey Add failed", __func__);
                        return;
                    }
                    break;
                }
                default:
                    break;
            }
            break;
        case ESP_BLE_MESH_CFG_CLIENT_SET_STATE_EVT:
            switch (opcode) {
                case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD: {
                    esp_ble_mesh_cfg_client_set_state_t set_state = {0};
                    example_ble_mesh_set_msg_common(
                        &common, node, config_client.model,
                        ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND);
                    set_state.model_app_bind.element_addr = node->unicast;
                    set_state.model_app_bind.model_app_idx = prov_key.app_idx;
                    set_state.model_app_bind.model_id =
                        ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV;
                    set_state.model_app_bind.company_id = ESP_BLE_MESH_CID_NVAL;
                    err = esp_ble_mesh_config_client_set_state(&common,
                                                               &set_state);
                    if (err) {
                        ESP_LOGE(TAG, "%s: Config Model App Bind failed",
                                 __func__);
                        return;
                    }
                    break;
                }
                case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND: {
                    esp_ble_mesh_generic_client_get_state_t get_state = {0};
                    example_ble_mesh_set_msg_common(
                        &common, node, onoff_client.model,
                        ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET);
                    err = esp_ble_mesh_generic_client_get_state(&common,
                                                                &get_state);
                    if (err) {
                        ESP_LOGE(TAG, "%s: Generic OnOff Get failed", __func__);
                        return;
                    }
                    break;
                }
                default:
                    break;
            }
            break;
        case ESP_BLE_MESH_CFG_CLIENT_PUBLISH_EVT:
            switch (opcode) {
                case ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_STATUS:
                    ESP_LOG_BUFFER_HEX("composition data 2 %s",
                                       param->status_cb.comp_data_status
                                           .composition_data->data,
                                       param->status_cb.comp_data_status
                                           .composition_data->len);
                    break;
                case ESP_BLE_MESH_MODEL_OP_APP_KEY_STATUS:
                    break;
                default:
                    break;
            }
            break;
        case ESP_BLE_MESH_CFG_CLIENT_TIMEOUT_EVT:
            switch (opcode) {
                case ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET: {
                    esp_ble_mesh_cfg_client_get_state_t get_state = {0};
                    example_ble_mesh_set_msg_common(
                        &common, node, config_client.model,
                        ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET);
                    get_state.comp_data_get.page = COMP_DATA_PAGE_0;
                    err = esp_ble_mesh_config_client_get_state(&common,
                                                               &get_state);
                    if (err) {
                        ESP_LOGE(TAG, "%s: Config Composition Data Get failed",
                                 __func__);
                        return;
                    }
                    break;
                }
                case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD: {
                    esp_ble_mesh_cfg_client_set_state_t set_state = {0};
                    example_ble_mesh_set_msg_common(
                        &common, node, config_client.model,
                        ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD);
                    set_state.app_key_add.net_idx = prov_key.net_idx;
                    set_state.app_key_add.app_idx = prov_key.app_idx;
                    memcpy(set_state.app_key_add.app_key, prov_key.app_key, 16);
                    err = esp_ble_mesh_config_client_set_state(&common,
                                                               &set_state);
                    if (err) {
                        ESP_LOGE(TAG, "%s: Config AppKey Add failed", __func__);
                        return;
                    }
                    break;
                }
                case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND: {
                    esp_ble_mesh_cfg_client_set_state_t set_state = {0};
                    example_ble_mesh_set_msg_common(
                        &common, node, config_client.model,
                        ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND);
                    set_state.model_app_bind.element_addr = node->unicast;
                    set_state.model_app_bind.model_app_idx = prov_key.app_idx;
                    set_state.model_app_bind.model_id =
                        ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV;
                    set_state.model_app_bind.company_id = ESP_BLE_MESH_CID_NVAL;
                    err = esp_ble_mesh_config_client_set_state(&common,
                                                               &set_state);
                    if (err) {
                        ESP_LOGE(TAG, "%s: Config Model App Bind failed",
                                 __func__);
                        return;
                    }
                    break;
                }
                default:
                    break;
            }
            break;
        default:
            ESP_LOGE(TAG, "Not a config client status message event");
            break;
    }
}

static void example_ble_mesh_generic_client_cb(
    esp_ble_mesh_generic_client_cb_event_t event,
    esp_ble_mesh_generic_client_cb_param_t *param) {
    esp_ble_mesh_client_common_param_t common = {0};
    esp_ble_mesh_node_info_t *node = NULL;
    uint32_t opcode;
    uint16_t addr;
    int err;

    opcode = param->params->opcode;
    addr = param->params->ctx.addr;

    ESP_LOGI(TAG,
             "%s, error_code = 0x%02x, event = 0x%02x, addr: 0x%04x, opcode: "
             "0x%04" PRIx32,
             __func__, param->error_code, event, param->params->ctx.addr,
             opcode);

    if (param->error_code) {
        ESP_LOGE(TAG, "Send generic client message failed, opcode 0x%04" PRIx32,
                 opcode);
        return;
    }
    node = example_ble_mesh_get_node_info(addr);
    if (!node) {
        ESP_LOGE(TAG, "%s: Get node info failed", __func__);
        return;
    }

    switch (event) {
        case ESP_BLE_MESH_GENERIC_CLIENT_GET_STATE_EVT:
            switch (opcode) {
                case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET: {
                    esp_ble_mesh_generic_client_set_state_t set_state = {0};
                    node->onoff = param->status_cb.onoff_status.present_onoff;
                    ESP_LOGI(
                        TAG,
                        "ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET onoff: 0x%02x",
                        node->onoff);
                    /* After Generic OnOff Status for Generic OnOff Get is
                     * received, Generic OnOff Set will be sent */
                    example_ble_mesh_set_msg_common(
                        &common, node, onoff_client.model,
                        ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET);
                    set_state.onoff_set.op_en = false;
                    set_state.onoff_set.onoff = !node->onoff;
                    set_state.onoff_set.tid = 0;
                    err = esp_ble_mesh_generic_client_set_state(&common,
                                                                &set_state);
                    if (err) {
                        ESP_LOGE(TAG, "%s: Generic OnOff Set failed", __func__);
                        return;
                    }
                    break;
                }
                default:
                    break;
            }
            break;
        case ESP_BLE_MESH_GENERIC_CLIENT_SET_STATE_EVT:
            switch (opcode) {
                case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET:
                    node->onoff = param->status_cb.onoff_status.present_onoff;
                    ESP_LOGI(
                        TAG,
                        "ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET onoff: 0x%02x",
                        node->onoff);
                    break;
                default:
                    break;
            }
            break;
        case ESP_BLE_MESH_GENERIC_CLIENT_PUBLISH_EVT:
            break;
        case ESP_BLE_MESH_GENERIC_CLIENT_TIMEOUT_EVT:
            /* If failed to receive the responses, these messages will be resend
             */
            switch (opcode) {
                case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET: {
                    esp_ble_mesh_generic_client_get_state_t get_state = {0};
                    example_ble_mesh_set_msg_common(
                        &common, node, onoff_client.model,
                        ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET);
                    err = esp_ble_mesh_generic_client_get_state(&common,
                                                                &get_state);
                    if (err) {
                        ESP_LOGE(TAG, "%s: Generic OnOff Get failed", __func__);
                        return;
                    }
                    break;
                }
                case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET: {
                    esp_ble_mesh_generic_client_set_state_t set_state = {0};
                    node->onoff = param->status_cb.onoff_status.present_onoff;
                    ESP_LOGI(
                        TAG,
                        "ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET onoff: 0x%02x",
                        node->onoff);
                    example_ble_mesh_set_msg_common(
                        &common, node, onoff_client.model,
                        ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET);
                    set_state.onoff_set.op_en = false;
                    set_state.onoff_set.onoff = !node->onoff;
                    set_state.onoff_set.tid = 0;
                    err = esp_ble_mesh_generic_client_set_state(&common,
                                                                &set_state);
                    if (err) {
                        ESP_LOGE(TAG, "%s: Generic OnOff Set failed", __func__);
                        return;
                    }
                    break;
                }
                default:
                    break;
            }
            break;
        default:
            ESP_LOGE(TAG, "Not a generic client status message event");
            break;
    }
}

static esp_err_t ble_mesh_init(void) {
    uint8_t match[2] = {0xdd, 0xdd};
    esp_err_t err = ESP_OK;

    prov_key.net_idx = ESP_BLE_MESH_KEY_PRIMARY;
    prov_key.app_idx = APP_KEY_IDX;
    memset(prov_key.app_key, APP_KEY_OCTET, sizeof(prov_key.app_key));

    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_client_callback(
        example_ble_mesh_config_client_cb);
    esp_ble_mesh_register_generic_client_callback(
        example_ble_mesh_generic_client_cb);

    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize mesh stack (err %d)", err);
        return err;
    }

    err = esp_ble_mesh_provisioner_set_dev_uuid_match(match, sizeof(match), 0x0,
                                                      false);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set matching device uuid (err %d)", err);
        return err;
    }
    err = esp_ble_mesh_provisioner_prov_enable(
        (esp_ble_mesh_prov_bearer_t)(ESP_BLE_MESH_PROV_ADV |
                                     ESP_BLE_MESH_PROV_GATT));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable mesh provisioner (err %d)", err);
        return err;
    }

    err = esp_ble_mesh_provisioner_add_local_app_key(
        prov_key.app_key, prov_key.net_idx, prov_key.app_idx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add local AppKey (err %d)", err);
        return err;
    }

    err = esp_ble_mesh_model_subscribe_group_addr(
        PROV_OWN_ADDR, ESP_BLE_MESH_CID_NVAL,
        ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_CLI, 0xC000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to subscribe to group address 0xC000");
        return err;
    }

    ESP_LOGI(TAG, "Successfully subscribed to group address 0xC000");

    ESP_LOGI(TAG, "BLE Mesh Provisioner initialized");

    return err;
}

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT &&
               event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta =
            {
                .ssid = EXAMPLE_ESP_WIFI_SSID,
                .password = EXAMPLE_ESP_WIFI_PASS,
                /* Authmode threshold resets to WPA2 as default if password
                 * matches WPA2 standards (pasword len => 8). If you want to
                 * connect the device to deprecated WEP/WPA networks, Please set
                 * the threshold value to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and
                 * set the password with length and format matching to
                 * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
                 */
                .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
                .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
            },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT)
     * or connection failed for the maximum number of re-tries (WIFI_FAIL_BIT).
     * The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE, portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we
     * can test which event actually happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
        mqtt_app_start();
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

void mqtt_data_callback(uint8_t *data, uint16_t lenght) {
    // {"action":1,"addr":"0x0008","state":0}
    uint16_t action = 0;
    // Parse JSON
    cJSON *root = cJSON_Parse((char *)data);
    if (root == NULL) {
        ESP_LOGE(TAG, "Failed to parse JSON");
        return;
    }

    // Get action
    cJSON *action_item = cJSON_GetObjectItem(root, "action");
    if (action_item != NULL && cJSON_IsNumber(action_item)) {
        action = (uint16_t)action_item->valueint;
    }
    printf("action: %d\n", action);

    switch (action) {
        case MQTT_ACTION_ENABLE_PROVISIONER:
            enable_provisioner = 1;
            break;
        case MQTT_ACTION_DISENABLE_PROVISIONER:
            enable_provisioner = 0;
            reset_devices();
            break;
        case MQTT_ACTION_PROVISION:
            uint8_t dev_uuid[16] = {0};
            // Get dev_uuid
            cJSON *dev_uuid_item = cJSON_GetObjectItem(root, "dev_uuid");
            if (dev_uuid_item != NULL && cJSON_IsString(dev_uuid_item)) {
                char *dev_uuid_str = dev_uuid_item->valuestring;
                for (int i = 0; i < 16; i++) {
                    sscanf(dev_uuid_str + 2 * i, "%2x",
                           (unsigned int *)&dev_uuid[i]);
                }
            }
            uint8_t addr[BD_ADDR_LEN] = {0};
            // Get addr
            cJSON *addr_item = cJSON_GetObjectItem(root, "addr");
            if (addr_item != NULL && cJSON_IsString(addr_item)) {
                char *addr_str = addr_item->valuestring;
                for (int i = 0; i < BD_ADDR_LEN; i++) {
                    sscanf(addr_str + 3 * i, "%2hhx", &addr[i]);
                }
            }
            esp_ble_mesh_addr_type_t addr_type = 0;
            // Get addr_type
            cJSON *addr_type_item = cJSON_GetObjectItem(root, "addr_type");
            if (addr_type_item != NULL && cJSON_IsNumber(addr_type_item)) {
                addr_type = (esp_ble_mesh_addr_type_t)addr_type_item->valueint;
            }
            uint16_t oob_info = 0;
            // Get oob_info
            cJSON *oob_info_item = cJSON_GetObjectItem(root, "oob_info");
            if (oob_info_item != NULL && cJSON_IsNumber(oob_info_item)) {
                oob_info = (uint16_t)oob_info_item->valueint;
            }
            esp_ble_mesh_prov_bearer_t bearer = 0;
            // Get bearer
            cJSON *bearer_item = cJSON_GetObjectItem(root, "bearer");
            if (bearer_item != NULL && cJSON_IsNumber(bearer_item)) {
                bearer = (esp_ble_mesh_prov_bearer_t)bearer_item->valueint;
            }

            printf("Send -> ");
            ESP_LOGI(TAG, "address: %s, address type: %d",
                     bt_hex(addr, BD_ADDR_LEN), addr_type);
            ESP_LOGI(TAG, "device uuid: %s", bt_hex(dev_uuid, 16));
            ESP_LOGI(TAG, "oob info: %d, bearer: %s", oob_info,
                     (bearer & ESP_BLE_MESH_PROV_ADV) ? "PB-ADV" : "PB-GATT");
            mqtt_action_provision(dev_uuid, addr, addr_type, oob_info, bearer);
            break;
        case MQTT_ACTION_CONTROL:
            uint16_t addr_control = 0x0000;
            uint16_t state = 9999;

            // Get addr
            cJSON *addr_item_control = cJSON_GetObjectItem(root, "addr");
            if (addr_item_control != NULL &&
                cJSON_IsString(addr_item_control)) {
                addr_control =
                    (uint16_t)strtol(addr_item_control->valuestring, NULL, 16);
            }
            // Get state
            cJSON *state_item = cJSON_GetObjectItem(root, "state");
            if (state_item != NULL && cJSON_IsNumber(state_item)) {
                state = (uint16_t)state_item->valueint;
            }
            printf("Send -> 0x%04x, 0x%04x\n", addr_control, state);
            send_control_led_switch_state(addr_control, state);
            break;
        default:
            break;
    }
}

esp_err_t test() {
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open("ble_mesh_node", NVS_READONLY, &my_handle);
    if (err != ESP_OK) return err;

    char index[3] = {0};  // Initialize index to zeros

    size_t required_size = sizeof(index);
    err = nvs_get_blob(my_handle, "index", index, &required_size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "Index blob not found, initializing to 0");
        index[0] = '0';
        index[1] = '\0';
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get index blob, err: %d", err);
        return err;
    }
    printf("index blob: %s\n", index);

    // Close
    nvs_close(my_handle);

    return ESP_OK;
}

void app_main(void) {
    esp_err_t err;

    ESP_LOGI(TAG, "Initializing...");

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    err = bluetooth_init();
    if (err) {
        ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }

    ble_mesh_get_dev_uuid(dev_uuid);
    test();

    /* Initialize the Bluetooth Mesh Subsystem */
    err = ble_mesh_init();
    if (err) {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
    }
    mqtt_data_pt_set_callback(mqtt_data_callback);
    wifi_init_sta();
}