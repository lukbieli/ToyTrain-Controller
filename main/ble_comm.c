
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#include "ble_comm.h"

#define GATTC_TAG "GATTC_DEMO"
#define REMOTE_SERVICE_UUID        0x00FF
#define REMOTE_CHAR_UUID    0xFF01
#define PROFILE_NUM      1
#define PROFILE_A_APP_ID 0
#define INVALID_HANDLE   0

// Battery Service UUIDs and Handles
#define GATTS_SERVICE_UUID_BATTERY       0x180F
#define GATTS_CHAR_UUID_BATTERY_LEVEL    0x2A19
#define GATTS_CHAR_UUID_BATTERY_VOLTAGE  0x2B18
#define GATTS_CHAR_NUM_BATTERY_LEVEL     0
#define GATTS_CHAR_NUM_BATTERY_VOLTAGE   1

// Motor Service UUIDs and Handles
#define GATTS_SERVICE_UUID_MOTOR         0x00DD
#define GATTS_CHAR_UUID_MOTOR_SPEED      0xDD01
#define GATTS_CHAR_UUID_MOTOR_DIRECTION  0xDD02
typedef enum {
    GATTS_CHAR_NUM_MOTOR_SPEED = 0,
    GATTS_CHAR_NUM_MOTOR_DIRECTION,
    GATTS_CHAR_NUM_MOTOR_MAX,
} gatts_motor_char_num_t;


// BLE Characteristic Structure
typedef struct {
    uint16_t handle;                     // Handle for the characteristic
    esp_bt_uuid_t uuid;                  // UUID of the characteristic
    esp_gatt_perm_t perm;                // Permissions for the characteristic
    esp_gatt_char_prop_t property;       // Properties of the characteristic
    esp_attr_value_t attr_val;           // Attribute value of the characteristic
    uint16_t descr_handle;               // Handle for the descriptor
    esp_bt_uuid_t descr_uuid;            // UUID of the descriptor
    // esp_attr_value_t cccd_val;           // CCCD value for notifications/indications
    bool is_ready;             // Flag to check if the characteristic is ready
    bool request_notify;           // Flag to check if notification is requested
} ble_characteristic_t;


typedef struct  {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    esp_bt_uuid_t service_uuid;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    ble_characteristic_t chars[GATTS_CHAR_NUM_MOTOR_MAX]; // Array of characteristics
    esp_bd_addr_t remote_bda;
    uint8_t currrentChar; // currently configured characteristic index
    uint8_t char_num; // Number of characteristics
}gattc_profile_inst_t;

volatile int readyFlag = 0;
volatile uint16_t dataRead = 0;

static char remote_device_name[ESP_BLE_ADV_NAME_LEN_MAX] = "ESP_GATTS_DEMO";
static bool connect    = false;
static bool get_server = false;
static esp_gattc_char_elem_t *char_elem_result   = NULL;
static esp_gattc_descr_elem_t *descr_elem_result = NULL;

/* Declare static functions */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static int8_t find_next_notif_to_enable(gattc_profile_inst_t* profile);


static esp_bt_uuid_t notify_descr_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,},
};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

/* One gatt-based profile one app_id and one gattc_if, this array will store the gattc_if returned by ESP_GATTS_REG_EVT */
static gattc_profile_inst_t gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
        .app_id = PROFILE_A_APP_ID,
        .conn_id = 0,
        .service_start_handle = 0,
        .service_end_handle = 0,
        .service_uuid = {
            .len = ESP_UUID_LEN_16,
            .uuid = {.uuid16 = GATTS_SERVICE_UUID_MOTOR,},
        },
        .currrentChar = 0,
        .char_num = GATTS_CHAR_NUM_MOTOR_MAX,
        .chars = {
            [GATTS_CHAR_NUM_MOTOR_SPEED] = {
                .handle = 0,
                .uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid = {.uuid16 = GATTS_CHAR_UUID_MOTOR_SPEED,},
                },
                .perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                .property = 0,
                .attr_val = {
                    .attr_max_len = 512,
                    .attr_len = 0,
                    .attr_value = NULL,
                },
                .descr_handle = 0,
                .request_notify = false,
                .is_ready = false,
            },
            [GATTS_CHAR_NUM_MOTOR_DIRECTION] = {
                .handle = 0,
                .uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid = {.uuid16 = GATTS_CHAR_UUID_MOTOR_DIRECTION,},
                },
                .perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                .property = 0,
                .attr_val = {
                    .attr_max_len = 512,
                    .attr_len = 0,
                    .attr_value = NULL,
                },
                .descr_handle = 0,
                .request_notify = false,
                .is_ready = false,
            }
        }
    },
};

static int8_t find_next_notif_to_enable(gattc_profile_inst_t* profile)
{
    for(; profile->currrentChar < profile->char_num; profile->currrentChar++){
        ble_characteristic_t *p_char = &profile->chars[profile->currrentChar];
        ESP_LOGI(GATTC_TAG, "char uuid16: %04x", p_char->uuid.uuid.uuid16);
        if ((p_char->request_notify == true) && (p_char->property & ESP_GATT_CHAR_PROP_BIT_NOTIFY))
        {
            ESP_LOGI(GATTC_TAG, "Notification requested for this characteristic");
            return profile->currrentChar;
        }
        else if ((p_char->request_notify == true) && ((p_char->property & ESP_GATT_CHAR_PROP_BIT_NOTIFY) == 0)){
            ESP_LOGI(GATTC_TAG, "Notification not allowed for this characteristic");
            p_char->is_ready = true; // Mark as ready even if notification is not allowed
        }
        else
        {
            ESP_LOGI(GATTC_TAG, "Notification not requested for this characteristic");
            p_char->is_ready = true; // Mark as ready
        }
    }

    return -1;
}

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    int i = 0;
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;
    esp_err_t scan_ret;
    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(GATTC_TAG, "GATT client register, status %d, app_id %d, gattc_if %d", param->reg.status, param->reg.app_id, gattc_if);
        scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (scan_ret){
            ESP_LOGE(GATTC_TAG, "set scan params error, error code = %x", scan_ret);
        }
        break;
    case ESP_GATTC_CONNECT_EVT:{
        ESP_LOGI(GATTC_TAG, "Connected, conn_id %d, remote "ESP_BD_ADDR_STR"", p_data->connect.conn_id,
                 ESP_BD_ADDR_HEX(p_data->connect.remote_bda));
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = p_data->connect.conn_id;
        memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        // Set connection parameters
        esp_ble_conn_update_params_t conn_params = {
            .latency = 0,
            .max_int = 0x30,  // 60ms
            .min_int = 0x20,  // 40ms
            .timeout = 400    // 4 seconds
        };
        memcpy(conn_params.bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        esp_ble_gap_update_conn_params(&conn_params);
        
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, p_data->connect.conn_id);
        if (mtu_ret){
            ESP_LOGE(GATTC_TAG, "Config MTU error, error code = %x", mtu_ret);
        }
        break;
    }
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Open failed, status %d", p_data->open.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Open successfully, MTU %u", p_data->open.mtu);
        break;
    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
        if (param->dis_srvc_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Service discover failed, status %d", param->dis_srvc_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Service discover complete, conn_id %d", param->dis_srvc_cmpl.conn_id);
        esp_ble_gattc_search_service(gattc_if, param->dis_srvc_cmpl.conn_id, &gl_profile_tab[PROFILE_A_APP_ID].service_uuid);
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        ESP_LOGI(GATTC_TAG, "MTU exchange, status %d, MTU %d", param->cfg_mtu.status, param->cfg_mtu.mtu);
        break;
    case ESP_GATTC_SEARCH_RES_EVT: {
        ESP_LOGI(GATTC_TAG, "Service search result, conn_id = %x, is primary service %d", p_data->search_res.conn_id, p_data->search_res.is_primary);
        ESP_LOGI(GATTC_TAG, "start handle %d, end handle %d, current handle value %d", p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.inst_id);
        for(i = 0; i < PROFILE_NUM; i++)
        {
            if (p_data->search_res.srvc_id.uuid.len == gl_profile_tab[PROFILE_A_APP_ID].service_uuid.len && p_data->search_res.srvc_id.uuid.uuid.uuid16 == gl_profile_tab[PROFILE_A_APP_ID].service_uuid.uuid.uuid16){ 
                ESP_LOGI(GATTC_TAG, "Service found");
                get_server = true;
                gl_profile_tab[PROFILE_A_APP_ID].service_start_handle = p_data->search_res.start_handle;
                gl_profile_tab[PROFILE_A_APP_ID].service_end_handle = p_data->search_res.end_handle;
                ESP_LOGI(GATTC_TAG, "UUID16: %04x", p_data->search_res.srvc_id.uuid.uuid.uuid16);
            }            
        }

        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (p_data->search_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Service search failed, status %x", p_data->search_cmpl.status);
            break;
        }
        if(p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_REMOTE_DEVICE) {
            ESP_LOGI(GATTC_TAG, "Get service information from remote device");
        } else if (p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_NVS_FLASH) {
            ESP_LOGI(GATTC_TAG, "Get service information from flash");
        } else {
            ESP_LOGI(GATTC_TAG, "Unknown service source");
        }
        ESP_LOGI(GATTC_TAG, "Service search complete");
        if (get_server){
            uint16_t count = 0;
            uint16_t countChar = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                     p_data->search_cmpl.conn_id,
                                                                     ESP_GATT_DB_CHARACTERISTIC,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                     INVALID_HANDLE,
                                                                     &count);
            if (status != ESP_GATT_OK){
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
                break;
            }

            if (count > 0){
                char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result){
                    ESP_LOGE(GATTC_TAG, "gattc no mem");
                    break;
                }else{
                    /* get char by uuid */
                    for(i = 0; i < gl_profile_tab[PROFILE_A_APP_ID].char_num; i++){
                        
                        status = esp_ble_gattc_get_char_by_uuid( gattc_if,
                                                                p_data->search_cmpl.conn_id,
                                                                gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                gl_profile_tab[PROFILE_A_APP_ID].chars[i].uuid,
                                                                char_elem_result,
                                                                &count);
                        if (status != ESP_GATT_OK){
                            ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_char_by_uuid error");
                            free(char_elem_result);
                            char_elem_result = NULL;
                            break;
                        }

                        if(count > 0)
                        {
                            ESP_LOGI(GATTC_TAG, "char uuid16: %04x", char_elem_result[i].uuid.uuid.uuid16);
                            ESP_LOGI(GATTC_TAG, "char handle: %d", char_elem_result[i].char_handle);
                            ESP_LOGI(GATTC_TAG, "char property: %d", char_elem_result[i].properties);
                            gl_profile_tab[PROFILE_A_APP_ID].chars[i].handle = char_elem_result[0].char_handle;
                            gl_profile_tab[PROFILE_A_APP_ID].chars[i].uuid = char_elem_result[0].uuid;
                            gl_profile_tab[PROFILE_A_APP_ID].chars[i].property = char_elem_result[0].properties;

                            // if(gl_profile_tab[PROFILE_A_APP_ID].chars[i].request_notify)
                            // {
                            //     if ((char_elem_result[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY)){
                            //         esp_ble_gattc_register_for_notify (gattc_if, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, char_elem_result[i].char_handle);
                            //     }
                            // }
                            // else
                            // {
                            //     ESP_LOGI(GATTC_TAG, "Notification not requested for this characteristic");
                            //     //char is ready
                            //     gl_profile_tab[PROFILE_A_APP_ID].chars[i].is_ready = true;
                            // }
                            if(gl_profile_tab[PROFILE_A_APP_ID].chars[i].request_notify == false)
                            {
                                //char is ready
                                gl_profile_tab[PROFILE_A_APP_ID].chars[i].is_ready = true;
                            }
                        }
                        else{
                            ESP_LOGE(GATTC_TAG, "char not found");
                        }
                        
                    }

                    /* register for notify if requested */
                    int8_t nextChar = find_next_notif_to_enable(&gl_profile_tab[PROFILE_A_APP_ID]);
                    if(nextChar != -1){
                        ESP_LOGI(GATTC_TAG, "Requesting enabling of notification for: %d", nextChar);
                        esp_ble_gattc_register_for_notify (gattc_if, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, gl_profile_tab[PROFILE_A_APP_ID].chars[i].handle);
                    }
                    else{
                        ESP_LOGI(GATTC_TAG, "No more characteristics to enable notification for");
                    }

                }
                /* free char_elem_result */
                free(char_elem_result);
            }else{
                ESP_LOGE(GATTC_TAG, "no char found");
            }
        }
         break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        if (p_data->reg_for_notify.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Notification register failed, status %d", p_data->reg_for_notify.status);
        }else{
            ESP_LOGI(GATTC_TAG, "Notification register successfully Handle %d", p_data->reg_for_notify.handle);
            uint16_t count = 0;
            uint16_t notify_en = 1;
            esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                         ESP_GATT_DB_DESCRIPTOR,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                         p_data->reg_for_notify.handle,
                                                                         &count);
            if (ret_status != ESP_GATT_OK){
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
                break;
            }
            if (count > 0){
                descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
                if (!descr_elem_result){
                    ESP_LOGE(GATTC_TAG, "malloc error, gattc no mem");
                    break;
                }else{
                    ret_status = esp_ble_gattc_get_descr_by_char_handle( gattc_if,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                         p_data->reg_for_notify.handle,
                                                                         notify_descr_uuid,
                                                                         descr_elem_result,
                                                                         &count);
                    if (ret_status != ESP_GATT_OK){
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                        free(descr_elem_result);
                        descr_elem_result = NULL;
                        break;
                    }
                    for(i = 0; i < count; i++){
                        ESP_LOGI(GATTC_TAG, "descr uuid16: %04x", descr_elem_result[i].uuid.uuid.uuid16);
                        ESP_LOGI(GATTC_TAG, "descr handle: %d", descr_elem_result[i].handle);
                        if (descr_elem_result[i].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[i].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG){
                            ble_characteristic_t *p_char = &gl_profile_tab[PROFILE_A_APP_ID].chars[gl_profile_tab[PROFILE_A_APP_ID].currrentChar];
                            p_char->descr_handle = descr_elem_result[i].handle;
                            p_char->descr_uuid = descr_elem_result[i].uuid;
                            ret_status = esp_ble_gattc_write_char_descr( gattc_if,
                                                                         gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                         descr_elem_result[i].handle,
                                                                         sizeof(notify_en),
                                                                         (uint8_t *)&notify_en,
                                                                         ESP_GATT_WRITE_TYPE_RSP,
                                                                         ESP_GATT_AUTH_REQ_NONE);
                        }
                    }
                    

                    if (ret_status != ESP_GATT_OK){
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_write_char_descr error");
                    }

                    /* free descr_elem_result */
                    free(descr_elem_result);
                }
            }
            else{
                ESP_LOGE(GATTC_TAG, "decsr not found");
            }

        }
        break;
    }
    case ESP_GATTC_NOTIFY_EVT:
        if (p_data->notify.is_notify){
            ESP_LOGI(GATTC_TAG, "Notification received");
        }else{
            ESP_LOGI(GATTC_TAG, "Indication received");
        }
        ESP_LOG_BUFFER_HEX(GATTC_TAG, p_data->notify.value, p_data->notify.value_len);
        dataRead = p_data->notify.value[0];
        // dataRead |= (p_data->notify.value[1] << 8);
        ESP_LOGI(GATTC_TAG, "Data received: %d", dataRead);
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Descriptor write failed, status %x", p_data->write.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Descriptor write successfully");
        if(gl_profile_tab[PROFILE_A_APP_ID].chars[i].request_notify)
        {
            ESP_LOGI(GATTC_TAG, "Notification enabled requested for this characteristic");
            //char is ready
            gl_profile_tab[PROFILE_A_APP_ID].chars[i].is_ready = true;
        }
        
        int8_t nextChar = find_next_notif_to_enable(&gl_profile_tab[PROFILE_A_APP_ID]);
        if(nextChar != -1){
            ESP_LOGI(GATTC_TAG, "Requesting enabling of notification for: %d", nextChar);
            esp_ble_gattc_register_for_notify (gattc_if, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, gl_profile_tab[PROFILE_A_APP_ID].chars[i].handle);
        }
        else{
            ESP_LOGI(GATTC_TAG, "No more characteristics to enable notification for");
        }

        break;
    case ESP_GATTC_SRVC_CHG_EVT: {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTC_TAG, "Service change from "ESP_BD_ADDR_STR"", ESP_BD_ADDR_HEX(bda));
        break;
    }
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "Characteristic write failed, status %x)", p_data->write.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Characteristic write successfully");
        // readyFlag = 1;
        break;
    case ESP_GATTC_DISCONNECT_EVT:
        connect = false;
        get_server = false;
        ESP_LOGI(GATTC_TAG, "Disconnected, remote "ESP_BD_ADDR_STR", reason 0x%02x",
                 ESP_BD_ADDR_HEX(p_data->disconnect.remote_bda), p_data->disconnect.reason);

        readyFlag = 0;
        ESP_LOGI(GATTC_TAG, "Start to scan again");
        //trigger scan again
        scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (scan_ret){
            ESP_LOGE(GATTC_TAG, "set scan params error, error code = %x", scan_ret);
        }
        break;
    default:
        break;
    }
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        //the unit of the duration is second
        uint32_t duration = 30;
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        //scan start complete event to indicate scan start successfully or failed
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTC_TAG, "Scanning start failed, status %x", param->scan_start_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Scanning start successfully");

        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            adv_name = esp_ble_resolve_adv_data_by_type(scan_result->scan_rst.ble_adv,
                                                        scan_result->scan_rst.adv_data_len + scan_result->scan_rst.scan_rsp_len,
                                                        ESP_BLE_AD_TYPE_NAME_CMPL,
                                                        &adv_name_len);
            ESP_LOGI(GATTC_TAG, "Scan result, device "ESP_BD_ADDR_STR", name len %u", ESP_BD_ADDR_HEX(scan_result->scan_rst.bda), adv_name_len);
            ESP_LOG_BUFFER_CHAR(GATTC_TAG, adv_name, adv_name_len);

#if CONFIG_EXAMPLE_DUMP_ADV_DATA_AND_SCAN_RESP
            if (scan_result->scan_rst.adv_data_len > 0) {
                ESP_LOGI(GATTC_TAG, "adv data:");
                ESP_LOG_BUFFER_HEX(GATTC_TAG, &scan_result->scan_rst.ble_adv[0], scan_result->scan_rst.adv_data_len);
            }
            if (scan_result->scan_rst.scan_rsp_len > 0) {
                ESP_LOGI(GATTC_TAG, "scan resp:");
                ESP_LOG_BUFFER_HEX(GATTC_TAG, &scan_result->scan_rst.ble_adv[scan_result->scan_rst.adv_data_len], scan_result->scan_rst.scan_rsp_len);
            }
#endif

            if (adv_name != NULL) {
                if (strlen(remote_device_name) == adv_name_len && strncmp((char *)adv_name, remote_device_name, adv_name_len) == 0) {
                    // Note: If there are multiple devices with the same device name, the device may connect to an unintended one.
                    // It is recommended to change the default device name to ensure it is unique.
                    ESP_LOGI(GATTC_TAG, "Device found %s", remote_device_name);
                    if (connect == false) {
                        connect = true;
                        ESP_LOGI(GATTC_TAG, "Connect to the remote device");
                        esp_ble_gap_stop_scanning();
                        esp_ble_gatt_creat_conn_params_t creat_conn_params = {0};
                        memcpy(&creat_conn_params.remote_bda, scan_result->scan_rst.bda, ESP_BD_ADDR_LEN);
                        creat_conn_params.remote_addr_type = scan_result->scan_rst.ble_addr_type;
                        creat_conn_params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
                        creat_conn_params.is_direct = true;
                        creat_conn_params.is_aux = false;
                        creat_conn_params.phy_mask = 0x0;
                        esp_ble_gattc_enh_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if,
                                            &creat_conn_params);
                    }
                }
            }
            break;
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            break;
        default:
            break;
        }
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GATTC_TAG, "Scanning stop failed, status %x", param->scan_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Scanning stop successfully");
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GATTC_TAG, "Advertising stop failed, status %x", param->adv_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Advertising stop successfully");
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(GATTC_TAG, "Connection params update, status %d, conn_int %d, latency %d, timeout %d",
                  param->update_conn_params.status,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
        ESP_LOGI(GATTC_TAG, "Packet length update, status %d, rx %d, tx %d",
                  param->pkt_data_length_cmpl.status,
                  param->pkt_data_length_cmpl.params.rx_len,
                  param->pkt_data_length_cmpl.params.tx_len);
        break;
    default:
        ESP_LOGI(GATTC_TAG, "GAP_EVENT %d", event);
        break;
    }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    /* If event is register event, store the gattc_if for each profile */
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        } else {
            ESP_LOGI(GATTC_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    /* If the gattc_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gattc_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gattc_if == gl_profile_tab[idx].gattc_if) {
                if (gl_profile_tab[idx].gattc_cb) {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}

void send_data_via_ble(const int16_t data)
{
    // Send data via BLE
    if (gl_profile_tab[PROFILE_A_APP_ID].chars[0].is_ready) {
        ESP_LOGI(GATTC_TAG, "Send data via BLE: %d", data);
        uint8_t write_pwm = (uint8_t)data;

        esp_err_t ret = esp_ble_gattc_write_char(gl_profile_tab[PROFILE_A_APP_ID].gattc_if,
                                                gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                gl_profile_tab[PROFILE_A_APP_ID].chars[0].handle,
                                                sizeof(write_pwm),
                                                &write_pwm,
                                                ESP_GATT_WRITE_TYPE_RSP,
                                                ESP_GATT_AUTH_REQ_NONE);

        if (ret){
            ESP_LOGE(GATTC_TAG, "gattc write char failed, error code = %x", ret);
        }
    }
}

uint16_t read_data_via_ble(void)
{
    return dataRead;
}

void ble_task(void)
{
    // Initialize NVS.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    #if CONFIG_EXAMPLE_CI_PIPELINE_ID
    memcpy(remote_device_name, esp_bluedroid_get_example_name(), sizeof(remote_device_name));
    #endif

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    //register the  callback function to the gap module
    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret){
        ESP_LOGE(GATTC_TAG, "%s gap register failed, error code = %x", __func__, ret);
        return;
    }

    //register the callback function to the gattc module
    ret = esp_ble_gattc_register_callback(esp_gattc_cb);
    if(ret){
        ESP_LOGE(GATTC_TAG, "%s gattc register failed, error code = %x", __func__, ret);
        return;
    }

    ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
    if (ret){
        ESP_LOGE(GATTC_TAG, "%s gattc app register failed, error code = %x", __func__, ret);
    }
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATTC_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    ESP_LOGI(GATTC_TAG, "GATT client demo initialized");
    // start periodic task
    // while (1) {
    //     vTaskDelay(5000 / portTICK_PERIOD_MS);
    //     if (readyFlag) {
    //         readyFlag = 0;
    //         ESP_LOGI(GATTC_TAG, "Send data");
    //         uint16_t write_pwm = 0x1234;

    //         esp_err_t ret = esp_ble_gattc_write_char(gl_profile_tab[PROFILE_A_APP_ID].gattc_if,
    //                                                  gl_profile_tab[PROFILE_A_APP_ID].conn_id,
    //                                                  gl_profile_tab[PROFILE_A_APP_ID].char_handle,
    //                                                  sizeof(write_pwm),
    //                                                  (uint8_t *)&write_pwm,
    //                                                  ESP_GATT_WRITE_TYPE_RSP,
    //                                                  ESP_GATT_AUTH_REQ_NONE);

    //         if (ret){
    //             ESP_LOGE(GATTC_TAG, "gattc write char failed, error code = %x", ret);
    //         }
    //     }
    // }

}
