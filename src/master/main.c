#ifdef DEVICE_ROLE_MASTER
/* ============================================================
 * Plant Management System - Master
 * Step 3-2: BLE Central - Scan & Connect & GATT Read
 * ============================================================ */

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

/* BLE */
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_uuid.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"

/* 共通ライブラリ */
#include "ble_protocol.h"
/* SPIFFS */
#include "esp_spiffs.h"
#include "cJSON.h"

#define SPIFFS_QUEUE_PATH  "/spiffs/queue.json"
#define QUEUE_MAX_COUNT    864  /* 各子機144件×6台 */

static void spiffs_init(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE("spiffs", "Failed to mount: %d", ret);
    } else {
        ESP_LOGI("spiffs", "Mounted OK");
    }
}

/* キューにセンサーデータを追加 */
static void __attribute__((unused)) queue_push(sensor_data_t *data, bool ble_ok, const char *err_reason)
{
    /* 既存のJSONを読む */
    cJSON *arr = NULL;
    FILE *f = fopen(SPIFFS_QUEUE_PATH, "r");
    if (f) {
        fseek(f, 0, SEEK_END);
        long sz = ftell(f);
        fseek(f, 0, SEEK_SET);
        char *buf = malloc(sz + 1);
        if (buf) {
            fread(buf, 1, sz, f);
            buf[sz] = 0;
            arr = cJSON_Parse(buf);
            free(buf);
        }
        fclose(f);
    }
    if (!arr || !cJSON_IsArray(arr)) {
        if (arr) cJSON_Delete(arr);
        arr = cJSON_CreateArray();
    }

    /* 最大件数超えたら古いものを削除 */
    while (cJSON_GetArraySize(arr) >= QUEUE_MAX_COUNT) {
        cJSON_DeleteItemFromArray(arr, 0);
    }

    /* 新しいデータを追加 */
    cJSON *item = cJSON_CreateObject();
    cJSON_AddNumberToObject(item, "slave_id",     data->slave_id);
    cJSON_AddNumberToObject(item, "temperature",  data->temperature);
    cJSON_AddNumberToObject(item, "humidity",     data->humidity);
    cJSON_AddNumberToObject(item, "light",        data->light);
    cJSON_AddNumberToObject(item, "water_level",  data->water_tank_level);
    cJSON_AddNumberToObject(item, "battery_volt", data->battery_voltage);
    cJSON_AddNumberToObject(item, "pump_on_ms",   data->pump_on_duration_ms);
    cJSON_AddNumberToObject(item, "uptime_count", data->uptime_count);
    cJSON_AddNumberToObject(item, "ble_success",  ble_ok ? 1 : 0);
    if (err_reason) cJSON_AddStringToObject(item, "ble_err_reason", err_reason);
    cJSON_AddItemToArray(arr, item);

    /* 書き戻す */
    char *out = cJSON_PrintUnformatted(arr);
    if (out) {
        FILE *fw = fopen(SPIFFS_QUEUE_PATH, "w");
        if (fw) { fputs(out, fw); fclose(fw); }
        free(out);
    }
    cJSON_Delete(arr);
    ESP_LOGI("spiffs", "Queue push done, size=%d", cJSON_GetArraySize(arr));
}

/* キューを読んでJSON文字列を返す（呼び出し側でfree必要） */
static char __attribute__((unused)) *queue_read_all(int *count)
{
    *count = 0;
    FILE *f = fopen(SPIFFS_QUEUE_PATH, "r");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    char *buf = malloc(sz + 1);
    if (!buf) { fclose(f); return NULL; }
    fread(buf, 1, sz, f);
    buf[sz] = 0;
    fclose(f);
    cJSON *arr = cJSON_Parse(buf);
    if (arr && cJSON_IsArray(arr)) {
        *count = cJSON_GetArraySize(arr);
        cJSON_Delete(arr);
    }
    return buf;
}

/* キューをクリア */
static void __attribute__((unused)) queue_clear(void)
{
    FILE *f = fopen(SPIFFS_QUEUE_PATH, "w");
    if (f) { fputs("[]", f); fclose(f); }
    ESP_LOGI("spiffs", "Queue cleared");
}

/* NVS統計 */
#include "nvs.h"

#define NVS_NAMESPACE     "master_stats"
#define NVS_KEY_RESTART   "restart_cnt"
#define NVS_KEY_BLE_ERR   "ble_err_cnt"
#define NVS_KEY_WIFI_ERR  "wifi_err_cnt"
#define NVS_KEY_HTTP_ERR  "http_err_cnt"
#define RESTART_COUNT_MAX 65535

typedef struct {
    uint32_t restart_count;
    uint32_t ble_err_count;
    uint32_t wifi_err_count;
    uint32_t http_err_count;
} master_stats_t;

static master_stats_t g_stats = {0};

static void stats_load(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return;
    nvs_get_u32(h, NVS_KEY_RESTART,  &g_stats.restart_count);
    nvs_get_u32(h, NVS_KEY_BLE_ERR,  &g_stats.ble_err_count);
    nvs_get_u32(h, NVS_KEY_WIFI_ERR, &g_stats.wifi_err_count);
    nvs_get_u32(h, NVS_KEY_HTTP_ERR, &g_stats.http_err_count);
    nvs_close(h);
}

static void stats_save(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u32(h, NVS_KEY_RESTART,  g_stats.restart_count);
    nvs_set_u32(h, NVS_KEY_BLE_ERR,  g_stats.ble_err_count);
    nvs_set_u32(h, NVS_KEY_WIFI_ERR, g_stats.wifi_err_count);
    nvs_set_u32(h, NVS_KEY_HTTP_ERR, g_stats.http_err_count);
    nvs_commit(h);
    nvs_close(h);
}

static void stats_increment_restart(void)
{
    g_stats.restart_count++;
    if (g_stats.restart_count >= RESTART_COUNT_MAX) g_stats.restart_count = 0;
    stats_save();
}


static const char *TAG = "master";

/* ============================================================
 * 設定
 * ============================================================ */
#define SCAN_DURATION_MS        5000
#define CONNECT_TIMEOUT_MS      10000
#define DEBUG_WAIT_MS           10000   /* デバッグ用待機（本番では削除） */

/* ============================================================
 * グローバル変数
 * ============================================================ */
static uint16_t g_conn_handle    = BLE_HS_CONN_HANDLE_NONE;
static uint16_t g_sensor_val_handle = 0;
static SemaphoreHandle_t g_sem_read_done;
static sensor_data_t g_sensor_data;
static bool g_scan_done = false;
static bool g_slave_found = false;
static ble_addr_t g_slave_addr;
static char g_slave_name[32];

/* ============================================================
 * 終了処理（デバッグ待機→リセット）
 * ============================================================ */
static void finish_and_restart(const char *reason)
{
    ESP_LOGI(TAG, "Done: %s", reason);
    ESP_LOGI(TAG, "Waiting %d sec before restart...", DEBUG_WAIT_MS / 1000);
    vTaskDelay(pdMS_TO_TICKS(DEBUG_WAIT_MS));  /* デバッグ用（本番では削除） */
    ESP_LOGI(TAG, "Restarting...");
    esp_restart();
}

/* ============================================================
 * GATTリードコールバック
 * ============================================================ */
static int gatt_read_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                        struct ble_gatt_attr *attr, void *arg)
{
    if (error->status == 0 && attr != NULL) {
        uint16_t len = OS_MBUF_PKTLEN(attr->om);
        ESP_LOGI(TAG, "Received %d bytes", len);
        if (len >= sizeof(sensor_data_t)) {
            os_mbuf_copydata(attr->om, 0, sizeof(sensor_data_t), &g_sensor_data);
            ESP_LOGI(TAG, "=== Sensor Data ===");
            ESP_LOGI(TAG, "  Slave ID    : %d",   g_sensor_data.slave_id);
            ESP_LOGI(TAG, "  Temperature : %.1f C", g_sensor_data.temperature);
            ESP_LOGI(TAG, "  Humidity    : %.1f %%", g_sensor_data.humidity);
            ESP_LOGI(TAG, "  Light       : %.0f lux", g_sensor_data.light);
            ESP_LOGI(TAG, "  Tank        : %.0f %%", g_sensor_data.water_tank_level);
            ESP_LOGI(TAG, "  Battery     : %.2f V", g_sensor_data.battery_voltage);
            ESP_LOGI(TAG, "  Pump last   : %d ms", g_sensor_data.pump_on_duration_ms);
            ESP_LOGI(TAG, "  Boot count  : %lu", g_sensor_data.uptime_count);
            ESP_LOGI(TAG, "===================");
        } else {
            ESP_LOGW(TAG, "Data too short: %d bytes (expected %d)",
                     len, sizeof(sensor_data_t));
        }
    } else {
        ESP_LOGE(TAG, "GATT read error: %d", error->status);
    }
    xSemaphoreGive(g_sem_read_done);
    return 0;
}

/* ============================================================
 * キャラクタリスティック探索コールバック
 * ============================================================ */
static int gatt_chr_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_chr *chr, void *arg)
{
    if (error->status == BLE_HS_EDONE) {
        if (g_sensor_val_handle != 0) {
            ESP_LOGI(TAG, "Reading sensor characteristic...");
            ble_gattc_read(conn_handle, g_sensor_val_handle, gatt_read_cb, NULL);
        } else {
            ESP_LOGW(TAG, "Sensor characteristic not found");
            xSemaphoreGive(g_sem_read_done);
        }
        return 0;
    }
    if (error->status != 0) {
        ESP_LOGE(TAG, "CHR discovery error: %d", error->status);
        xSemaphoreGive(g_sem_read_done);
        return 0;
    }

    if (chr->uuid.u.type == BLE_UUID_TYPE_16) {
        ESP_LOGI(TAG, "  CHR UUID16: 0x%04X val_handle=%d",
                 chr->uuid.u16.value, chr->val_handle);
    } else if (chr->uuid.u.type == BLE_UUID_TYPE_128) {
        ESP_LOGI(TAG, "  CHR UUID128: %02x%02x val_handle=%d",
                 chr->uuid.u128.value[1], chr->uuid.u128.value[0],
                 chr->val_handle);
        /* DEE1（センサーREAD）*/
        if (chr->uuid.u128.value[0] == 0xe1 &&
            chr->uuid.u128.value[1] == 0xde) {
            g_sensor_val_handle = chr->val_handle;
            ESP_LOGI(TAG, "  -> Sensor CHR found!");
        }
    }
    return 0;
}

/* ============================================================
 * サービス探索コールバック
 * ============================================================ */
static int gatt_svc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                       const struct ble_gatt_svc *svc, void *arg)
{
    if (error->status == BLE_HS_EDONE) {
        ESP_LOGI(TAG, "Service discovery done");
        return 0;
    }
    if (error->status != 0) {
        ESP_LOGE(TAG, "SVC discovery error: %d", error->status);
        xSemaphoreGive(g_sem_read_done);
        return 0;
    }

    if (svc->uuid.u.type == BLE_UUID_TYPE_16) {
        ESP_LOGI(TAG, "Found service UUID16: 0x%04X", svc->uuid.u16.value);
    } else if (svc->uuid.u.type == BLE_UUID_TYPE_128) {
        ESP_LOGI(TAG, "Found service UUID128: %02x%02x",
                 svc->uuid.u128.value[1], svc->uuid.u128.value[0]);
        /* 128bitサービスのみキャラクタリスティック探索 */
        g_sensor_val_handle = 0;
        ble_gattc_disc_all_chrs(conn_handle,
                                svc->start_handle,
                                svc->end_handle,
                                gatt_chr_cb, NULL);
    }
    return 0;
}

/* ============================================================
 * GAPイベントコールバック
 * ============================================================ */
static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            g_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "Connected! conn_handle=%d", g_conn_handle);
            ble_gattc_exchange_mtu(g_conn_handle, NULL, NULL);  // 3引数が正しい
            ble_gattc_disc_all_svcs(g_conn_handle, gatt_svc_cb, NULL);
        } else {
            ESP_LOGE(TAG, "Connection failed: %d", event->connect.status);
            g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            xSemaphoreGive(g_sem_read_done);
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected, reason=%d", event->disconnect.reason);
        g_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        break;

    default:
        break;
    }
    return 0;
}

/* ============================================================
 * BLEスキャンコールバック
 * ============================================================ */
static int scan_event_cb(struct ble_gap_event *event, void *arg)
{
    if (event->type == BLE_GAP_EVENT_DISC_COMPLETE) {
        ESP_LOGI(TAG, "Scan complete");
        g_scan_done = true;
        return 0;
    }

    if (event->type != BLE_GAP_EVENT_DISC) return 0;
    if (g_slave_found) return 0;  /* すでに発見済み */

    struct ble_hs_adv_fields fields;
    if (ble_hs_adv_parse_fields(&fields,
            event->disc.data,
            event->disc.length_data) != 0) return 0;

    if (fields.name == NULL || fields.name_len == 0) return 0;

    char name[32] = {0};
    int len = fields.name_len < 31 ? fields.name_len : 31;
    memcpy(name, fields.name, len);

    if (strncmp(name, SLAVE_NAME_PREFIX, strlen(SLAVE_NAME_PREFIX)) != 0) return 0;

    ESP_LOGI(TAG, "Found slave: %s addr_type=%d", name, event->disc.addr.type);

    /* 発見 → スキャン停止して接続へ */
    g_slave_found = true;
    strncpy(g_slave_name, name, 31);
    g_slave_addr = event->disc.addr;
    ble_gap_disc_cancel();

    return 0;
}

/* ============================================================
 * マスタータスク
 * ============================================================ */
static void master_task(void *arg)
{
    /* スキャン開始 */
    struct ble_gap_disc_params disc_params = {0};
    disc_params.passive           = 0;
    disc_params.itvl              = 0x0010;
    disc_params.window            = 0x0010;
    disc_params.filter_duplicates = 1;

    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, SCAN_DURATION_MS,
                          &disc_params, scan_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to start scan: %d", rc);
        finish_and_restart("scan start failed");
    }
    ESP_LOGI(TAG, "BLE scan started (%dms)", SCAN_DURATION_MS);

    /* スキャン完了 or 発見まで待つ */
    while (!g_scan_done && !g_slave_found) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (!g_slave_found) {
        ESP_LOGW(TAG, "No slave found");
        finish_and_restart("no slave found");
    }

    /* 接続 */
    ESP_LOGI(TAG, "Connecting to %s ...", g_slave_name);
    struct ble_gap_conn_params conn_params = {0};
    conn_params.scan_itvl           = 0x0010;
    conn_params.scan_window         = 0x0010;
    conn_params.itvl_min            = BLE_GAP_INITIAL_CONN_ITVL_MIN;
    conn_params.itvl_max            = BLE_GAP_INITIAL_CONN_ITVL_MAX;
    conn_params.latency             = 0;
    conn_params.supervision_timeout = 400;
    conn_params.min_ce_len          = 0;
    conn_params.max_ce_len          = 0;

    rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &g_slave_addr,
                         CONNECT_TIMEOUT_MS, &conn_params,
                         gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_connect failed: %d", rc);
        finish_and_restart("connect failed");
    }

    /* GATTリード完了を待つ */
    if (xSemaphoreTake(g_sem_read_done, pdMS_TO_TICKS(15000)) != pdTRUE) {
        ESP_LOGW(TAG, "GATT read timeout");
        finish_and_restart("GATT timeout");
    }

    /* 切断 */
    if (g_conn_handle != BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_terminate(g_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    /* TODO: WiFi送信 */

    finish_and_restart("success");
}

/* ============================================================
 * NimBLE sync callback
 * ============================================================ */
static void on_sync(void)
{
    ESP_LOGI(TAG, "BLE sync OK");
    xTaskCreate(master_task, "master_task", 4096, NULL, 5, NULL);
}

static void on_reset(int reason)
{
    ESP_LOGE(TAG, "BLE reset: %d", reason);
}

static void nimble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* ============================================================
 * app_main
 * ============================================================ */
void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(5000));  /* デバッグ用待機（本番では削除） */
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " Plant Management System - Master");
    ESP_LOGI(TAG, "========================================");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    g_sem_read_done = xSemaphoreCreateBinary();

    /* SPIFFS初期化 */
    spiffs_init();

    /* NVS統計ロード */
    stats_load();
    stats_increment_restart();

    nimble_port_init();
    ble_svc_gap_init();
    ble_hs_cfg.sync_cb  = on_sync;
    ble_hs_cfg.reset_cb = on_reset;
    nimble_port_freertos_init(nimble_host_task);

    ESP_LOGI(TAG, "NimBLE initialized");
}
#endif // DEVICE_ROLE_MASTER