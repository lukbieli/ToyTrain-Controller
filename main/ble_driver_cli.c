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

#include "ble_driver_cli.h"

/* DEFINES */
#define GATTC_TAG "GATTC_DEMO"

#define INVALID_HANDLE   0

#define ENABLE_LOGI 1
#define ENABLE_LOGD 0
#define ENABLE_LOGE 1
#define ENABLE_LOGBUF 0
#define MY_LOGI(...) do { if (ENABLE_LOGI) ESP_LOGI(__VA_ARGS__); } while(0)
#define MY_LOGD(...) do { if (ENABLE_LOGD) ESP_LOGD(__VA_ARGS__); } while(0)
#define MY_LOGE(...) do { if (ENABLE_LOGE)ESP_LOGE(__VA_ARGS__); } while(0)
#define MY_LOGBUF_CHAR(...) do { if (ENABLE_LOGBUF) ESP_LOG_BUFFER_CHAR(__VA_ARGS__); } while(0)
#define MY_LOGBUF_HEX(...) do { if (ENABLE_LOGBUF) ESP_LOG_BUFFER_HEX(__VA_ARGS__); } while(0)

// Battery Service UUIDs and Handles
#define GATTS_SERVICE_UUID_BATTERY       0x180F
#define GATTS_CHAR_UUID_BATTERY_LEVEL    0x2A19
#define GATTS_CHAR_UUID_BATTERY_VOLTAGE  0x2B18

// Motor Service UUIDs and Handles
#define GATTS_SERVICE_UUID_MOTOR         0x00DD
#define GATTS_CHAR_UUID_MOTOR_SPEED      0xDD01
#define GATTS_CHAR_UUID_MOTOR_DIRECTION  0xDD02

#define MAX_CHAT_COUNT  10
#define MAX_DESCR_COUNT 10

#define ATTR_VAL_MAX_LEN 20

/* Typedefs */
typedef enum {
    PROFILE_BATTERY_APP_ID = 0,
    PROFILE_MOTOR_APP_ID,
    PROFILE_NUM, // GATT client profile number
} profile_app_id_t;

typedef enum {
    GATTS_CHAR_NUM_BATTERY_LEVEL = 0,
    GATTS_CHAR_NUM_BATTERY_VOLTAGE,
    GATTS_CHAR_NUM_BATTERY_MAX,
} gatts_battery_char_num_t;

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
    volatile bool processing; // Flag to check if the characteristic is being processed
} ble_characteristic_t;


typedef struct  {
    profile_app_id_t app_id; // Application ID for the profile
    esp_bt_uuid_t service_uuid;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    ble_characteristic_t chars[GATTS_CHAR_NUM_MOTOR_MAX]; // Array of characteristics
    uint8_t currrentChar; // currently configured characteristic index
    uint8_t char_num; // Number of characteristics
    bool is_ready; // Flag to check if the profile is ready
    bool is_found; // Flag to check if the service is found
}gattc_profile_inst_t;


typedef struct{
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    esp_bd_addr_t remote_bda;
}gattc_connection_t;

/* Module scope variables variables */
static volatile int readyFlag = 0;
static volatile uint16_t dataRead = 0;

static char remote_device_name[ESP_BLE_ADV_NAME_LEN_MAX] = "TTLoc_001"; // Remote device name
static bool connect    = false; // Connection flag

static esp_ble_gatt_creat_conn_params_t creat_conn_params = {0};

static uint8_t attr_val_batt_lvl[ATTR_VAL_MAX_LEN] = {0};
static uint8_t attr_val_batt_volt[ATTR_VAL_MAX_LEN] = {0};
static uint8_t attr_val_motor_speed[ATTR_VAL_MAX_LEN] = {0};
static uint8_t attr_val_motor_direction[ATTR_VAL_MAX_LEN] = {0};

static esp_gattc_char_elem_t char_elem_result[MAX_CHAT_COUNT];
static esp_gattc_descr_elem_t descr_elem_result[MAX_DESCR_COUNT];

/* Declare static functions */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void espGattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static int8_t find_next_notif_to_enable(gattc_profile_inst_t* profile);
static void find_characteristics_and_descriptors(void);
static void convertFloatToUint8Arr(uint8_t *buf, float voltage);
static float convertUint8ArrToFloat(uint8_t *buf);
static gattc_profile_inst_t* findServiceByCharHandle(uint16_t char_handle, ble_characteristic_t** char_elem_result);
static gattc_profile_inst_t* findServiceByDescHandle(uint16_t desc_handle, ble_characteristic_t** char_elem_result);

static uint16_t notify_en = 1; // Notification enable flag

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

static gattc_connection_t gattc_conn = {   
    .gattc_if = ESP_GATT_IF_NONE,
    .app_id = PROFILE_BATTERY_APP_ID,
    .conn_id = 0,
    .remote_bda = {0},
};

/* One gatt-based profile one app_id and one gattc_if, this array will store the gattc_if returned by ESP_GATTS_REG_EVT */
static gattc_profile_inst_t gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_BATTERY_APP_ID] = {
        .app_id = PROFILE_BATTERY_APP_ID,
        .service_start_handle = 0,
        .service_end_handle = 0,
        .is_ready = false,
        .is_found = false,
        .service_uuid = {
            .len = ESP_UUID_LEN_16,
            .uuid = {.uuid16 = GATTS_SERVICE_UUID_BATTERY,},
        },
        .currrentChar = 0,
        .char_num = GATTS_CHAR_NUM_BATTERY_MAX,
        .chars = {
            [GATTS_CHAR_NUM_BATTERY_LEVEL] = {
                .handle = 0,
                .uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid = {.uuid16 = GATTS_CHAR_UUID_BATTERY_LEVEL,},
                },
                .perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                .property = 0,
                .attr_val = {
                    .attr_max_len = ATTR_VAL_MAX_LEN,
                    .attr_len = 0,
                    .attr_value = attr_val_batt_lvl,
                },
                .descr_handle = 0,
                .request_notify = true, // Request notification for battery level
                .is_ready = false,
            },
            [GATTS_CHAR_NUM_BATTERY_VOLTAGE] = {
                .handle = 0,
                .uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid = {.uuid16 = GATTS_CHAR_UUID_BATTERY_VOLTAGE,},
                },
                .perm = ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                .property = 0,
                .attr_val = {
                    .attr_max_len = ATTR_VAL_MAX_LEN,
                    .attr_len = 0,
                    .attr_value = attr_val_batt_volt,
                },
                .descr_handle = 0,
                .request_notify = true, // Request notification for battery voltage
                .is_ready = false,
            }
        }
    },
    [PROFILE_MOTOR_APP_ID] = {
        .app_id = PROFILE_MOTOR_APP_ID,
        .service_start_handle = 0,
        .service_end_handle = 0,
        .is_ready = false,
        .is_found = false,
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
                    .attr_max_len = ATTR_VAL_MAX_LEN,
                    .attr_len = 0,
                    .attr_value = attr_val_motor_speed,
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
                    .attr_max_len = ATTR_VAL_MAX_LEN,
                    .attr_len = 0,
                    .attr_value = attr_val_motor_direction,
                },
                .descr_handle = 0,
                .request_notify = false,
                .is_ready = false,
            }
        }
    }
};


/* Global functions */

bool BleDriverCli_IsReady(void)
{
    // Check if the connection is ready
    if (gl_profile_tab[PROFILE_BATTERY_APP_ID].is_ready && gl_profile_tab[PROFILE_MOTOR_APP_ID].is_ready) {
        return true; // Connection is ready
    }
    return false; // Connection is not ready
}

float BleDriverCli_GetBatteryVoltage(void)
{
    // Read battery voltage from the characteristic
    if (gl_profile_tab[PROFILE_BATTERY_APP_ID].chars[GATTS_CHAR_NUM_BATTERY_VOLTAGE].is_ready) {
        MY_LOGD(GATTC_TAG, "Read battery voltage via BLE");
        float battery_voltage = convertUint8ArrToFloat((uint8_t *)gl_profile_tab[PROFILE_BATTERY_APP_ID].chars[GATTS_CHAR_NUM_BATTERY_VOLTAGE].attr_val.attr_value);
        return battery_voltage; // Return battery voltage
    }
    return 0.0;
}

uint8_t BleDriverCli_GetBatteryLevel(void)
{
    // Read battery level from the characteristic
    if (gl_profile_tab[PROFILE_BATTERY_APP_ID].chars[GATTS_CHAR_NUM_BATTERY_LEVEL].is_ready) {
        MY_LOGD(GATTC_TAG, "Read battery level via BLE");
        uint8_t battery_level = *(uint8_t *)gl_profile_tab[PROFILE_BATTERY_APP_ID].chars[GATTS_CHAR_NUM_BATTERY_LEVEL].attr_val.attr_value;
        return battery_level; // Return battery level
    }
    return 0;
}

bool BleDriverCli_SetMotorSpeed(uint8_t speed)
{
    bool result = false;
    // Send motor speed via BLE
    if (gl_profile_tab[PROFILE_MOTOR_APP_ID].chars[GATTS_CHAR_NUM_MOTOR_SPEED].is_ready == true && 
        gl_profile_tab[PROFILE_MOTOR_APP_ID].chars[GATTS_CHAR_NUM_MOTOR_SPEED].processing == false) {

        gl_profile_tab[PROFILE_MOTOR_APP_ID].chars[GATTS_CHAR_NUM_MOTOR_SPEED].processing = true; // Set processing flag to true
        MY_LOGD(GATTC_TAG, "Send motor speed via BLE: %d", speed);
        esp_err_t ret = esp_ble_gattc_write_char(gattc_conn.gattc_if,
                                                gattc_conn.conn_id,
                                                gl_profile_tab[PROFILE_MOTOR_APP_ID].chars[GATTS_CHAR_NUM_MOTOR_SPEED].handle,
                                                sizeof(speed),
                                                &speed,
                                                ESP_GATT_WRITE_TYPE_RSP,
                                                ESP_GATT_AUTH_REQ_NONE);

        if (ret){
            MY_LOGE(GATTC_TAG, "gattc write char failed, error code = %x", ret);
            gl_profile_tab[PROFILE_MOTOR_APP_ID].chars[GATTS_CHAR_NUM_MOTOR_SPEED].processing = false; // Reset processing flag
        }
        else {
            result = true; // Set result to true if write was successful
        }
    }

    return result; // Return result of the operation
}

bool BleDriverCli_SetMotorDirection(uint8_t direction)
{
    bool result = false;
    // Send motor direction via BLE
    if (gl_profile_tab[PROFILE_MOTOR_APP_ID].chars[GATTS_CHAR_NUM_MOTOR_DIRECTION].is_ready == true &&
        gl_profile_tab[PROFILE_MOTOR_APP_ID].chars[GATTS_CHAR_NUM_MOTOR_DIRECTION].processing == false) {
        
        gl_profile_tab[PROFILE_MOTOR_APP_ID].chars[GATTS_CHAR_NUM_MOTOR_DIRECTION].processing = true; // Set processing flag to true
        MY_LOGD(GATTC_TAG, "Send motor direction via BLE: %d", direction);
        esp_err_t ret = esp_ble_gattc_write_char(gattc_conn.gattc_if,
                                                gattc_conn.conn_id,
                                                gl_profile_tab[PROFILE_MOTOR_APP_ID].chars[GATTS_CHAR_NUM_MOTOR_DIRECTION].handle,
                                                sizeof(direction),
                                                &direction,
                                                ESP_GATT_WRITE_TYPE_RSP,
                                                ESP_GATT_AUTH_REQ_NONE);


        if (ret){
            MY_LOGE(GATTC_TAG, "gattc write char failed, error code = %x", ret);
            gl_profile_tab[PROFILE_MOTOR_APP_ID].chars[GATTS_CHAR_NUM_MOTOR_DIRECTION].processing = false; // Reset processing flag
        }
        else {
            result = true; // Set result to true if write was successful
        }
    }

    return result; // Return result of the operation
}

void BleDriverCli_Setup(void)
{
    MY_LOGI(GATTC_TAG, "GATT client setup");
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
        MY_LOGE(GATTC_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        MY_LOGE(GATTC_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        MY_LOGE(GATTC_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        MY_LOGE(GATTC_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    //register the  callback function to the gap module
    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret){
        MY_LOGE(GATTC_TAG, "%s gap register failed, error code = %x", __func__, ret);
        return;
    }

    //register the callback function to the gattc module
    ret = esp_ble_gattc_register_callback(espGattc_cb);
    if(ret){
        MY_LOGE(GATTC_TAG, "%s gattc register failed, error code = %x", __func__, ret);
        return;
    }

    ret = esp_ble_gattc_app_register(PROFILE_BATTERY_APP_ID);
    if (ret){
        MY_LOGE(GATTC_TAG, "%s gattc app %d register failed, error code = %x", __func__, PROFILE_BATTERY_APP_ID, ret);
        return;
    }

    ret = esp_ble_gattc_app_register(PROFILE_MOTOR_APP_ID);
    if (ret){
        MY_LOGE(GATTC_TAG, "%s gattc app %d register failed, error code = %x", __func__, PROFILE_MOTOR_APP_ID, ret);
        return;
    }

    

    ret = esp_ble_gap_set_scan_params(&ble_scan_params);
    if (ret){
        MY_LOGE(GATTC_TAG, "%s  set scan params error, error code = %x",  __func__, ret);
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        MY_LOGE(GATTC_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }
    
    MY_LOGI(GATTC_TAG, "GATT client initialized");

}

/* Function Definitions */

/**
 * @brief Function to find the next characteristic to enable notifications for.
 * 
 * This function iterates through the characteristics of the given profile and checks if notifications are requested and allowed.
 * If a characteristic is found that requests notifications and allows them, it returns the index of that characteristic.
 * If no such characteristic is found, it returns -1.
 * 
 * @param profile Pointer to the GATT client profile instance.
 * @return Index of the next characteristic to enable notifications for, or -1 if none found.
 */
static int8_t find_next_notif_to_enable(gattc_profile_inst_t* profile)
{
    for(; profile->currrentChar < profile->char_num; profile->currrentChar++){
        ble_characteristic_t *p_char = &profile->chars[profile->currrentChar];
        MY_LOGD(GATTC_TAG, "char uuid16: %04x", p_char->uuid.uuid.uuid16);
        if ((p_char->request_notify == true) && (p_char->property & ESP_GATT_CHAR_PROP_BIT_NOTIFY))
        {
            MY_LOGD(GATTC_TAG, "Notification requested for this characteristic");
            return profile->currrentChar;
        }
        else if ((p_char->request_notify == true) && ((p_char->property & ESP_GATT_CHAR_PROP_BIT_NOTIFY) == 0)){
            MY_LOGD(GATTC_TAG, "Notification not allowed for this characteristic");
            p_char->is_ready = true; // Mark as ready even if notification is not allowed
        }
        else
        {
            MY_LOGD(GATTC_TAG, "Notification not requested for this characteristic");
            p_char->is_ready = true; // Mark as ready
        }
    }

    return -1;
}

/**
 * @brief Function to hangle GAP events.
 * 
 * This function handles various GAP events such as scan start, scan stop, and scan results.
 * It processes the scan results and initiates a connection to the remote device if the specified device name is found.
 * 
 * @param event GAP event type.
 * @param param Pointer to the event parameters.
 */
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
            MY_LOGE(GATTC_TAG, "Scanning start failed, status %x", param->scan_start_cmpl.status);
            break;
        }
        MY_LOGI(GATTC_TAG, "Scanning start successfully");

        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            adv_name = esp_ble_resolve_adv_data_by_type(scan_result->scan_rst.ble_adv,
                                                        scan_result->scan_rst.adv_data_len + scan_result->scan_rst.scan_rsp_len,
                                                        ESP_BLE_AD_TYPE_NAME_CMPL,
                                                        &adv_name_len);
            MY_LOGD(GATTC_TAG, "Scan result, device "ESP_BD_ADDR_STR", name len %u", ESP_BD_ADDR_HEX(scan_result->scan_rst.bda), adv_name_len);
            MY_LOGBUF_CHAR(GATTC_TAG, adv_name, adv_name_len);

#if CONFIG_EXAMPLE_DUMP_ADV_DATA_AND_SCAN_RESP
            if (scan_result->scan_rst.adv_data_len > 0) {
                MY_LOGD(GATTC_TAG, "adv data:");
                MY_LOGBUF_HEX(GATTC_TAG, &scan_result->scan_rst.ble_adv[0], scan_result->scan_rst.adv_data_len);
            }
            if (scan_result->scan_rst.scan_rsp_len > 0) {
                MY_LOGD(GATTC_TAG, "scan resp:");
                MY_LOGBUF_HEX(GATTC_TAG, &scan_result->scan_rst.ble_adv[scan_result->scan_rst.adv_data_len], scan_result->scan_rst.scan_rsp_len);
            }
#endif

            if (adv_name != NULL) {
                if (strlen(remote_device_name) == adv_name_len && strncmp((char *)adv_name, remote_device_name, adv_name_len) == 0) {
                    // Note: If there are multiple devices with the same device name, the device may connect to an unintended one.
                    // It is recommended to change the default device name to ensure it is unique.
                    MY_LOGD(GATTC_TAG, "Device found %s", remote_device_name);
                    if (connect == false) {
                        connect = true;
                        MY_LOGI(GATTC_TAG, "Connected to the remote device %s", remote_device_name);
                        esp_ble_gap_stop_scanning();                        
                        memcpy(&creat_conn_params.remote_bda, scan_result->scan_rst.bda, ESP_BD_ADDR_LEN);
                        creat_conn_params.remote_addr_type = scan_result->scan_rst.ble_addr_type;
                        creat_conn_params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
                        creat_conn_params.is_direct = true;
                        creat_conn_params.is_aux = false;
                        creat_conn_params.phy_mask = 0x0;
                        esp_ble_gattc_enh_open(gattc_conn.gattc_if,
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
            MY_LOGE(GATTC_TAG, "Scanning stop failed, status %x", param->scan_stop_cmpl.status);
            break;
        }
        MY_LOGD(GATTC_TAG, "Scanning stop successfully");
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            MY_LOGE(GATTC_TAG, "Advertising stop failed, status %x", param->adv_stop_cmpl.status);
            break;
        }
        MY_LOGD(GATTC_TAG, "Advertising stop successfully");
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         MY_LOGD(GATTC_TAG, "Connection params update, status %d, conn_int %d, latency %d, timeout %d",
                  param->update_conn_params.status,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
        MY_LOGD(GATTC_TAG, "Packet length update, status %d, rx %d, tx %d",
                  param->pkt_data_length_cmpl.status,
                  param->pkt_data_length_cmpl.params.rx_len,
                  param->pkt_data_length_cmpl.params.tx_len);
        break;
    default:
        MY_LOGD(GATTC_TAG, "GAP_EVENT %d", event);
        break;
    }
}

/**
 * @brief Function to find characteristics and descriptors in the GATT database.
 * 
 * This function iterates through the registered profiles and retrieves the characteristics and descriptors for each profile.
 * It also registers for notifications if requested by the application.
 */
void find_characteristics_and_descriptors(void)
{
    for(int i = 0; i < PROFILE_NUM; i++)
    {
        if(gl_profile_tab[i].is_found == false){
            MY_LOGD(GATTC_TAG, "Profile %d: Service not found", i);
            continue;
        }
        uint16_t count = 0;
        esp_gatt_status_t status = esp_ble_gattc_get_attr_count( gattc_conn.gattc_if,
                                                                 gattc_conn.conn_id,
                                                                 ESP_GATT_DB_CHARACTERISTIC,
                                                                 gl_profile_tab[i].service_start_handle,
                                                                 gl_profile_tab[i].service_end_handle,
                                                                 INVALID_HANDLE,
                                                                 &count);
        if (status != ESP_GATT_OK){
            MY_LOGE(GATTC_TAG, "Profile %d: esp_ble_gattc_get_attr_count error", i);
            break;
        }

        MY_LOGD(GATTC_TAG, "Profile %d: char attr count: %d", i, count);

        if (count > 0 && count <= MAX_CHAT_COUNT){

            /* get char by uuid */
            for(int j = 0; j < gl_profile_tab[i].char_num; j++){
                
                status = esp_ble_gattc_get_char_by_uuid( gattc_conn.gattc_if,
                                                        gattc_conn.conn_id,
                                                        gl_profile_tab[i].service_start_handle,
                                                        gl_profile_tab[i].service_end_handle,
                                                        gl_profile_tab[i].chars[j].uuid,
                                                        char_elem_result,
                                                        &count);
                if (status != ESP_GATT_OK){
                    MY_LOGE(GATTC_TAG, "Profile %d: esp_ble_gattc_get_char_by_uuid error", i);
                    break;
                }

                MY_LOGD(GATTC_TAG, "Profile %d: char count: %d", i, count);

                if(count > 0)
                {
                    MY_LOGD(GATTC_TAG, "Profile %d: char uuid16: %04x", i, char_elem_result[0].uuid.uuid.uuid16);
                    MY_LOGD(GATTC_TAG, "Profile %d: char handle: %d", i, char_elem_result[0].char_handle);
                    MY_LOGD(GATTC_TAG, "Profile %d: char property: %d", i, char_elem_result[0].properties);
                    gl_profile_tab[i].chars[j].handle = char_elem_result[0].char_handle;
                    gl_profile_tab[i].chars[j].uuid = char_elem_result[0].uuid;
                    gl_profile_tab[i].chars[j].property = char_elem_result[0].properties;

                    if(gl_profile_tab[i].chars[j].request_notify == false)
                    {
                        //char is ready
                        gl_profile_tab[i].chars[j].is_ready = true;
                    }
                }
                else{
                    MY_LOGE(GATTC_TAG, "Profile %d: char not found", i);
                }
                
            }

            /* register for notify if requested */
            int8_t nextChar = find_next_notif_to_enable(&gl_profile_tab[i]);
            if(nextChar != -1){
                MY_LOGD(GATTC_TAG, "Profile %d: Requesting enabling of notification for: %d", i, nextChar);
                esp_ble_gattc_register_for_notify (gattc_conn.gattc_if, gattc_conn.remote_bda, gl_profile_tab[i].chars[nextChar].handle);
                gl_profile_tab[i].currrentChar = nextChar;
            }
            else{
                MY_LOGD(GATTC_TAG, "Profile %d: No more characteristics to enable notification for", i);
                // mark profile as ready
                gl_profile_tab[i].is_ready = true;
                MY_LOGI(GATTC_TAG, "Profile %d: Profile is ready", i);
            }


        }else{
            MY_LOGE(GATTC_TAG, "Profile %d: no char found or count to high %d", i, count);
        }
    }
}

/**
 * @brief Function to find a service by its characteristic handle.
 * 
 * This function iterates through the registered profiles and checks if the given characteristic handle matches any of the characteristics in the profiles.
 * If a match is found, it returns the corresponding profile instance and optionally sets the char_elem_result to point to the matching characteristic.
 * 
 * @param char_handle Characteristic handle to search for.
 * @param char_elem_result Pointer to store the matching characteristic element (optional).
 * @return Pointer to the matching profile instance, or NULL if not found.
 */
static gattc_profile_inst_t* findServiceByCharHandle(uint16_t char_handle, ble_characteristic_t** char_elem_result)
{
    for(int i = 0; i < PROFILE_NUM; i++){
        for(int j = 0; j < gl_profile_tab[i].char_num; j++){
            if(gl_profile_tab[i].chars[j].handle == char_handle){
                if(char_elem_result != NULL)
                {
                    *char_elem_result = &gl_profile_tab[i].chars[j];
                }
                return &gl_profile_tab[i];
            }
        }
    }
    return NULL;
}

/**
 * @brief Function to find a service by its descriptor handle.
 * 
 * This function iterates through the registered profiles and checks if the given descriptor handle matches any of the descriptors in the profiles.
 * If a match is found, it returns the corresponding profile instance and optionally sets the char_elem_result to point to the matching characteristic.
 * 
 * @param desc_handle Descriptor handle to search for.
 * @param char_elem_result Pointer to store the matching characteristic element (optional).
 * @return Pointer to the matching profile instance, or NULL if not found.
 */
static gattc_profile_inst_t* findServiceByDescHandle(uint16_t desc_handle, ble_characteristic_t** char_elem_result)
{
    for(int i = 0; i < PROFILE_NUM; i++){
        for(int j = 0; j < gl_profile_tab[i].char_num; j++){
            MY_LOGD(GATTC_TAG, "[find_desc] desc_handle: %d", gl_profile_tab[i].chars[j].descr_handle);
            if(gl_profile_tab[i].chars[j].descr_handle == desc_handle){
                if(char_elem_result != NULL)
                {
                    *char_elem_result = &gl_profile_tab[i].chars[j];
                }
                return &gl_profile_tab[i];
            }
        }
    }
    return NULL;
}

/**
 * * @brief Function to handle GATT client events.
 * 
 * This function handles various GATT client events such as registration, connection, service discovery, and characteristic read/write operations.
 * It processes the events and updates the profile instances accordingly.
 * 
 * @param event GATT client event type.
 * @param gattc_if GATT client interface.
 * @param param Pointer to the event parameters.
 */
static void espGattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    MY_LOGD(GATTC_TAG, "GATT client event %d gattc_if %d", event, gattc_if);

    int i = 0;
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;
    esp_err_t scan_ret;
    ble_characteristic_t *p_char = NULL;
    gattc_profile_inst_t * p_srv = NULL;
    
    switch (event) {
            /* If event is register event, store the gattc_if for each profile */
    case ESP_GATTC_REG_EVT: 
        if (param->reg.status == ESP_GATT_OK) {
            gattc_conn.gattc_if = gattc_if;
            MY_LOGD(GATTC_TAG, "Register app_id %04x, gattc_if %d, status %d", param->reg.app_id, gattc_if, param->reg.status);
        } else {
            MY_LOGD(GATTC_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
        }
        break;

    case ESP_GATTC_CONNECT_EVT:
        MY_LOGD(GATTC_TAG, " Connected, conn_id %d, remote "ESP_BD_ADDR_STR"", p_data->connect.conn_id,
                 ESP_BD_ADDR_HEX(p_data->connect.remote_bda));
        gattc_conn.conn_id = p_data->connect.conn_id;
        memcpy(gattc_conn.remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));

        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, p_data->connect.conn_id);
        if (mtu_ret){
            MY_LOGE(GATTC_TAG, "Config MTU error, error code = %x", mtu_ret);
        }
        break;
    
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK){
            MY_LOGE(GATTC_TAG, "Open failed, status %d", p_data->open.status);
            break;
        }
        MY_LOGD(GATTC_TAG, "Open successfully, MTU %u", p_data->open.mtu);
        break;

    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
        if (param->dis_srvc_cmpl.status != ESP_GATT_OK){
            MY_LOGE(GATTC_TAG, "Service discover failed, status %d", param->dis_srvc_cmpl.status);
            break;
        }
        MY_LOGD(GATTC_TAG, "Service discover complete, conn_id %d", param->dis_srvc_cmpl.conn_id);
        esp_ble_gattc_search_service(gattc_if, param->dis_srvc_cmpl.conn_id, NULL);
        break;

    case ESP_GATTC_CFG_MTU_EVT:
        MY_LOGD(GATTC_TAG, "MTU exchange, status %d, MTU %d", param->cfg_mtu.status, param->cfg_mtu.mtu);
        break;

    case ESP_GATTC_SEARCH_RES_EVT: 
        MY_LOGD(GATTC_TAG, "Service search result, conn_id = %x, is primary service %d", p_data->search_res.conn_id, p_data->search_res.is_primary);
        MY_LOGD(GATTC_TAG, "start handle %d, end handle %d, current handle value %d", p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.inst_id);

        bool known_service = false;
        for(int i = 0; i < PROFILE_NUM; i++){
            
        
            if (param->search_res.srvc_id.uuid.len == gl_profile_tab[i].service_uuid.len && param->search_res.srvc_id.uuid.uuid.uuid16 == gl_profile_tab[i].service_uuid.uuid.uuid16)
            {           
                known_service = true;      
                gl_profile_tab[i].is_found = true;
                gl_profile_tab[i].service_start_handle = param->search_res.start_handle;
                gl_profile_tab[i].service_end_handle = param->search_res.end_handle;
                MY_LOGD(GATTC_TAG, "Service %d found. UUID16: %04x", i, param->search_res.srvc_id.uuid.uuid.uuid16);
            }      
        }

        if(!known_service){
            MY_LOGD(GATTC_TAG, "Service not found UUID16: %04x", param->search_res.srvc_id.uuid.uuid.uuid16);
        }
        
        break;
    
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (param->search_cmpl.status != ESP_GATT_OK){
            MY_LOGE(GATTC_TAG, "Service search failed, status %x", param->search_cmpl.status);
            return;
        }
        if(param->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_REMOTE_DEVICE) {
            MY_LOGD(GATTC_TAG, "Get service information from remote device");
        } else if (param->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_NVS_FLASH) {
            MY_LOGD(GATTC_TAG, "Get service information from flash");
        } else {
            MY_LOGD(GATTC_TAG, "Unknown service source");
        }
        MY_LOGD(GATTC_TAG, "Service search complete");
        find_characteristics_and_descriptors();
        break;

    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
        if (p_data->reg_for_notify.status != ESP_GATT_OK){
            MY_LOGE(GATTC_TAG, "Notification register failed, status %d", p_data->reg_for_notify.status);
        }else{
            MY_LOGD(GATTC_TAG, "Notification register successfully Handle %d", p_data->reg_for_notify.handle);
            uint16_t count = 0;

            p_char = NULL;
            p_srv = findServiceByCharHandle(p_data->reg_for_notify.handle, &p_char);
            if(p_srv == NULL){
                MY_LOGE(GATTC_TAG, "[REG_NOTIFY_EVT] Service not found by handle %d", p_data->reg_for_notify.handle);
                break;
            }
            
            esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                         gattc_conn.conn_id,
                                                                         ESP_GATT_DB_DESCRIPTOR,
                                                                         p_srv->service_start_handle,
                                                                         p_srv->service_end_handle,
                                                                         p_data->reg_for_notify.handle,
                                                                         &count);
            if (ret_status != ESP_GATT_OK){
                MY_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
                break;
            }
            if (count > 0 && count <= MAX_DESCR_COUNT){

                ret_status = esp_ble_gattc_get_descr_by_char_handle( gattc_if,
                                                                        gattc_conn.conn_id,
                                                                        p_data->reg_for_notify.handle,
                                                                        notify_descr_uuid,
                                                                        descr_elem_result,
                                                                        &count);
                if (ret_status != ESP_GATT_OK){
                    MY_LOGE(GATTC_TAG, "esp_ble_gattc_get_descr_by_char_handle error");
                    break;
                }
                for(i = 0; i < count; i++){
                    MY_LOGD(GATTC_TAG, "descr uuid16: %04x", descr_elem_result[i].uuid.uuid.uuid16);
                    MY_LOGD(GATTC_TAG, "descr handle: %d", descr_elem_result[i].handle);
                    if (descr_elem_result[i].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[i].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG){
                        p_char->descr_handle = descr_elem_result[i].handle;
                        p_char->descr_uuid = descr_elem_result[i].uuid;
                        MY_LOGD(GATTC_TAG, "descr found");
                        ret_status = esp_ble_gattc_write_char_descr( gattc_if,
                                                                        gattc_conn.conn_id,
                                                                        p_char->descr_handle,
                                                                        sizeof(notify_en),
                                                                        (uint8_t *)&notify_en,
                                                                        ESP_GATT_WRITE_TYPE_RSP,
                                                                        ESP_GATT_AUTH_REQ_NONE);
                        if (ret_status != ESP_GATT_OK){
                            MY_LOGE(GATTC_TAG, "esp_ble_gattc_write_char_descr error");
                        }
                        break;
                    }
                }
            }
        
            else{
                MY_LOGE(GATTC_TAG, "decsr not found or count to high %d", count);
            }

        }
        break;
    
    case ESP_GATTC_NOTIFY_EVT:
        if (p_data->notify.is_notify){
            MY_LOGD(GATTC_TAG, "Notification received");
        }else{
            MY_LOGD(GATTC_TAG, "Indication received");
        }
        MY_LOGBUF_HEX(GATTC_TAG, p_data->notify.value, p_data->notify.value_len);
        dataRead = p_data->notify.value[0];
        // dataRead |= (p_data->notify.value[1] << 8);
        MY_LOGD(GATTC_TAG, "Data received: %d", dataRead);
        // find characteristic 
        p_char = NULL;
        p_srv = findServiceByCharHandle(p_data->notify.handle, &p_char);
        if(p_srv == NULL){
            MY_LOGE(GATTC_TAG, "[NOTIFY_EVT] Service not found by handle %d", p_data->notify.handle);
            break;
        }
        else if(p_char == NULL){
            MY_LOGE(GATTC_TAG, "[NOTIFY_EVT] Characteristic not found by handle %d", p_data->notify.handle);
            break;
        }
        MY_LOGD(GATTC_TAG, "Profile %d: Characteristic handle: %d", p_srv->app_id, p_char->handle);
        // store attribute value
        if(p_data->notify.value_len > p_char->attr_val.attr_max_len){
            MY_LOGE(GATTC_TAG, "Attribute value length too long");
            p_char->attr_val.attr_len = p_char->attr_val.attr_max_len;
        }
        else
        {
            p_char->attr_val.attr_len = p_data->notify.value_len;
        }
        memcpy(p_char->attr_val.attr_value, p_data->notify.value, p_char->attr_val.attr_len);
        MY_LOGD(GATTC_TAG, "Attribute value length: %d", p_char->attr_val.attr_len);
        break;

    case ESP_GATTC_WRITE_DESCR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            MY_LOGE(GATTC_TAG, "Descriptor write failed, status %x", p_data->write.status);
            break;
        }
        MY_LOGD(GATTC_TAG, "Descriptor write successfully handle %d", p_data->write.handle);

        // find characteristic
        p_char = NULL;
        p_srv = findServiceByDescHandle(p_data->write.handle, &p_char);
        if(p_srv == NULL){
            MY_LOGE(GATTC_TAG, "[WRITE_DESCR_EVT] Service not found by handle %d", p_data->write.handle);
            break;
        }
        else if(p_char == NULL){
            MY_LOGE(GATTC_TAG, "[WRITE_DESCR_EVT] Characteristic not found by handle %d", p_data->write.handle);
            break;
        }
        MY_LOGD(GATTC_TAG, "Profile %d: Characteristic handle: %d", p_srv->app_id, p_char->handle);

        if(p_char->request_notify)
        {
            MY_LOGD(GATTC_TAG, "Notification enabled requested for this characteristic");
            //char is ready
            p_char->is_ready = true;
            p_srv->currrentChar++;
        }
        
        int8_t nextChar = find_next_notif_to_enable(p_srv);
        if(nextChar != -1){
            MY_LOGD(GATTC_TAG, "Requesting enabling of notification for: %d", nextChar);
            esp_ble_gattc_register_for_notify (gattc_if, gattc_conn.remote_bda, p_srv->chars[nextChar].handle);
            
        }
        else{
            MY_LOGD(GATTC_TAG, "No more characteristics to enable notification for srv %d", p_srv->app_id);
            // mark profile as ready
            p_srv->is_ready = true;
            MY_LOGI(GATTC_TAG, "Profile %d: Profile is ready", p_srv->app_id);
        }

        break;

    case ESP_GATTC_SRVC_CHG_EVT: 
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        MY_LOGD(GATTC_TAG, "Service change from "ESP_BD_ADDR_STR"", ESP_BD_ADDR_HEX(bda));
        break;
    
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            MY_LOGE(GATTC_TAG, "Characteristic write failed, status %d", p_data->write.status);
            break;
        }
        MY_LOGD(GATTC_TAG, "Characteristic write successfully");
        // find characteristic
        p_char = NULL;
        p_srv = findServiceByCharHandle(p_data->write.handle, &p_char);
        if(p_srv == NULL){
            MY_LOGE(GATTC_TAG, "[WRITE_CHAR_EVT] Service not found by handle %d", p_data->write.handle);
            break;
        }
        else if(p_char == NULL){
            MY_LOGE(GATTC_TAG, "[WRITE_CHAR_EVT] Characteristic not found by handle %d", p_data->write.handle);
            break;
        }
        MY_LOGD(GATTC_TAG, "Profile %d: Characteristic handle: %d", p_srv->app_id, p_char->handle);
        // end processing
        p_char->processing = false;
        break;

    case ESP_GATTC_DISCONNECT_EVT:
        connect = false;
        MY_LOGD(GATTC_TAG, "Disconnected, remote "ESP_BD_ADDR_STR", reason 0x%02x",
                 ESP_BD_ADDR_HEX(p_data->disconnect.remote_bda), p_data->disconnect.reason);

        // clear all profiles
        for(i = 0; i < PROFILE_NUM; i++){
            gl_profile_tab[i].is_found = false;
            gl_profile_tab[i].is_ready = false;
            gl_profile_tab[i].currrentChar = 0;
            for(int j = 0; j < gl_profile_tab[i].char_num; j++){
                gl_profile_tab[i].chars[j].is_ready = false;
                gl_profile_tab[i].chars[j].processing = false;
            }
        }
        //clear connection id
        gattc_conn.conn_id = 0xFFFF;
        // gattc_conn.gattc_if = 0xFF;
        //clear remote bd addr
        memset(gattc_conn.remote_bda, 0, sizeof(esp_bd_addr_t));
        

        MY_LOGD(GATTC_TAG, "Start to scan again");
        //trigger scan again
        scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (scan_ret){
            MY_LOGE(GATTC_TAG, "set scan params error, error code = %x", scan_ret);
        }
        break;

    default:
        MY_LOGD(GATTC_TAG, "GATTC_EVENT %d", event);
        break;
    }
}

/**
 * @brief Function to convert float voltage to uint8_t array and vice versa.
 * 
 * This function converts a float voltage value to a uint8_t array (4 bytes) and vice versa.
 * The conversion is done by multiplying the voltage by 1000 and storing it as an integer in the array.
 * 
 * @param buf Pointer to the uint8_t array to store the converted value.
 * @param voltage Float voltage value to convert.
 */
static void convertFloatToUint8Arr(uint8_t *buf, float voltage)
{
    // Convert float to uint8_t array (4 bytes)
    buf[0] = (uint8_t)((int)(voltage * 1000) & 0xFF);         // LSB
    buf[1] = (uint8_t)(((int)(voltage * 1000) >> 8) & 0xFF);
    buf[2] = (uint8_t)(((int)(voltage * 1000) >> 16) & 0xFF);
    buf[3] = (uint8_t)(((int)(voltage * 1000) >> 24) & 0xFF); // MSB
}

/**
 * @brief Function to convert uint8_t array to float voltage.
 * 
 * This function converts a uint8_t array (4 bytes) to a float voltage value.
 * The conversion is done by interpreting the array as an integer and dividing it by 1000.0.
 * 
 * @param buf Pointer to the uint8_t array to convert.
 * @return Converted float voltage value.
 */
static float convertUint8ArrToFloat(uint8_t *buf)
{
    // Convert uint8_t array (4 bytes) to float
    int32_t voltage_int = (int32_t)(buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24));
    return (float)voltage_int / 1000.0; // Convert to volts
}
