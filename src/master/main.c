#ifdef DEVICE_ROLE_MASTER
/* ============================================================
 * 植物管理システム - 親機 (Master) メインプログラム
 * ============================================================
 * 機能:
 *   1. BLEスキャンでPLANT_XXを発見
 *   2. 子機に接続してセンサーデータ読み取り
 *   3. 全子機のデータ収集後にWiFi送信（将来実装）
 *   4. ディープスリープ
 * ============================================================ */

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* BLE */
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"

/* 共通ライブラリ */
#include "ble_protocol.h"

static const char *TAG = "master";

/* ============================================================
 * 設定
 * ============================================================ */
#define MAX_SLAVES          6
#define SLAVE_NAME_PREFIX   "PLANT_"
#define SCAN_DURATION_MS    5000   // 5秒スキャン
#define CONNECT_TIMEOUT_MS  10000  // 10秒接続タイムアウト

/* ============================================================
 * グローバル変数
 * ============================================================ */
static sensor_data_t slave_data[MAX_SLAVES] __attribute__((unused));
static int slave_count __attribute__((unused)) = 0;static volatile bool scan_done = false;
static volatile bool data_received = false;

/* ============================================================
 * BLE スキャンコールバック
 * ============================================================ */
static int scan_event_cb(struct ble_gap_event *event, void *arg)
{
    if (event->type != BLE_GAP_EVENT_EXT_DISC &&
        event->type != BLE_GAP_EVENT_DISC) {
        return 0;
    }

    // アドバタイズデータからデバイス名を取得
    struct ble_hs_adv_fields fields;
    if (ble_hs_adv_parse_fields(&fields,
            event->disc.data,
            event->disc.length_data) != 0) {
        return 0;
    }

    if (fields.name == NULL || fields.name_len == 0) return 0;

    // "PLANT_"で始まる名前か確認
    char name[32] = {0};
    int len = fields.name_len < 31 ? fields.name_len : 31;
    memcpy(name, fields.name, len);

    if (strncmp(name, SLAVE_NAME_PREFIX, strlen(SLAVE_NAME_PREFIX)) == 0) {
        ESP_LOGI(TAG, "Found slave: %s", name);
        // TODO: 接続処理
    }

    return 0;
}

/* ============================================================
 * BLEスキャン開始
 * ============================================================ */
static void start_scan(void)
{
    struct ble_gap_disc_params disc_params = {0};
    disc_params.passive = 0;
    disc_params.itvl = 0x0010;
    disc_params.window = 0x0010;
    disc_params.filter_duplicates = 1;

    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, SCAN_DURATION_MS,
                          &disc_params, scan_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to start scan: %d", rc);
    } else {
        ESP_LOGI(TAG, "BLE scan started (%dms)", SCAN_DURATION_MS);
    }
}

/* ============================================================
 * NimBLE sync callback
 * ============================================================ */
static void on_sync(void)
{
    ESP_LOGI(TAG, "BLE sync OK, starting scan...");
    start_scan();
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
    vTaskDelay(pdMS_TO_TICKS(5000));  // 5秒待機（接続待ち）
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " Plant Management System - Master");
    ESP_LOGI(TAG, "========================================");

    // NVS初期化
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // NimBLE初期化
    nimble_port_init();
    ble_svc_gap_init();
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;
    nimble_port_freertos_init(nimble_host_task);

    ESP_LOGI(TAG, "NimBLE initialized");

    // メインループ（将来的にWiFi送信等を追加）
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
#endif // DEVICE_ROLE_MASTER