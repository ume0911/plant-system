/*
 * ============================================================
 * 子機 (Slave) メインプログラム
 * ============================================================
 *
 * 動作フロー (通常モード):
 *   1. ディープスリープから起床
 *   2. センサーデータ取得 (温度, 湿度, 光量, 水タンク, 電圧)
 *   3. 水やり判定 → 必要ならポンプON
 *   4. BLEアドバタイズ開始 (親機が接続してくるのを待つ)
 *   5. 親機と接続 → データ送信 & 設定受信
 *   6. 切断 → ディープスリープへ
 *
 * 動作フロー (メンテナンスモード):
 *   - ディープスリープ中にD2スイッチON → GPIO割り込みで起床
 *   - ディープスリープしない
 *   - スマホからBLE接続可能 (別のサービスUUID)
 *   - パラメータの表示・変更が可能
 *
 * ハードウェア構成 (想定):
 *   - 温湿度: DHT22 or SHT30 (I2C)
 *   - 光量:   BH1750 (I2C)
 *   - 水タンク: 超音波センサー or フロートスイッチ
 *   - 電圧:   ADC (分圧回路経由)
 *   - ポンプ:  MOSFET経由でGPIO制御
 *   - モードSW: GPIO7 (プルアップ、GNDに落とすとメンテナンスモード)
 */

#include <stdio.h>
#include <string.h>

/* ESP-IDF システム */
#include "esp_log.h"
#include "esp_system.h"
#include "esp_sleep.h"          // ディープスリープ

/* FreeRTOS */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* GPIO / ADC */
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

/* BLE */
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "host/ble_gatt.h"

/* 共通ライブラリ */
#include "ble_protocol.h"
#include "config_store.h"

/* ============================================================
 * 定数・GPIO定義
 * ============================================================ */
static const char *TAG = "slave";

// --- GPIO ピン定義 (実際の配線に合わせて変更) ---
#define GPIO_PUMP           GPIO_NUM_6      // D4
#define GPIO_MODE_SWITCH    GPIO_NUM_7      // D5 (RTC GPIO対応、ext1で使用)
#define GPIO_LED_STATUS     GPIO_NUM_15     // オンボードLED

// --- I2C設定 (センサー用) ---
#define I2C_MASTER_SDA      GPIO_NUM_21
#define I2C_MASTER_SCL      GPIO_NUM_22

// --- ADC (電圧測定) ---
#define ADC_BATTERY_CHANNEL ADC_CHANNEL_6   // GPIO34

// --- 子機ID (各子機ごとに変更) ---
#define MY_SLAVE_ID         1               // ← 子機ごとに 1-6 に変更

// --- BLEアドバタイズのタイムアウト ---
#define BLE_ADV_TIMEOUT_SEC 30              // 30秒待っても親機が来なければスリープ

/* ============================================================
 * グローバル変数
 * ============================================================ */

// 今回の測定データ
static sensor_data_t my_sensor_data;

// 親機から受け取った制御パラメータ
static control_params_t my_params;

// 起動回数 (RTC memoryに保存 → ディープスリープでも消えない)
static RTC_DATA_ATTR uint32_t boot_count = 0;

// 親機との通信完了フラグ
static volatile bool ble_data_exchanged = false;
static volatile bool ble_connected = false;
static void load_params(void);



/* ============================================================
 * センサー読み取り
 * ============================================================ */
static void read_sensors(sensor_data_t *data)
{
    data->slave_id = MY_SLAVE_ID;
    data->uptime_count = boot_count;

    // --- 温度・湿度 (DHT22 / SHT30) ---
    // TODO: 実際のI2Cドライバで読み取る
    data->temperature = 25.0f;  // ダミー
    data->humidity    = 45.0f;  // ダミー

    // --- 光量 (BH1750) ---
    // TODO: 実際のI2Cドライバで読み取る
    data->light = 500.0f;       // ダミー [lux]

    // --- 水タンク残量 ---
    // TODO: 超音波センサーまたはフロートスイッチ
    data->water_tank_level = 80.0f;  // ダミー [%]

    // --- 電源電圧 (ADC) ---
    // TODO: ADCから分圧回路経由で測定
    data->battery_voltage = 3.7f;     // ダミー [V]

    data->pump_on_duration_ms = 0;    // まだポンプ動かしてない

    ESP_LOGI(TAG, "Sensors: T=%.1f H=%.1f L=%.0f Tank=%.0f%% V=%.2f",
             data->temperature, data->humidity, data->light,
             data->water_tank_level, data->battery_voltage);
}

/* ============================================================
 * 水やり判定・ポンプ制御
 * ============================================================ */
static void check_and_water(sensor_data_t *data)
{
    bool should_water = false;

    if (data->humidity < my_params.water_threshold) {
        ESP_LOGI(TAG, "Humidity %.1f < threshold %.1f -> WATERING",
                 data->humidity, my_params.water_threshold);
        should_water = true;
    }

    if (my_params.flags & FLAG_FORCE_WATER) {
        ESP_LOGI(TAG, "Force water flag set -> WATERING");
        should_water = true;
    }

    if (data->water_tank_level < 5.0f) {
        ESP_LOGW(TAG, "Water tank nearly empty, skipping water");
        should_water = false;
    }

    if (should_water && my_params.pump_on_time_ms > 0) {
        ESP_LOGI(TAG, "Pump ON for %d ms", my_params.pump_on_time_ms);
        gpio_set_level(GPIO_PUMP, 1);
        vTaskDelay(pdMS_TO_TICKS(my_params.pump_on_time_ms));
        gpio_set_level(GPIO_PUMP, 0);
        data->pump_on_duration_ms = my_params.pump_on_time_ms;
        ESP_LOGI(TAG, "Pump OFF");
    }
}

/* ============================================================
 * BLE
 * ============================================================ */
static void ble_advertise(void);

static int gap_event_handler(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
	case BLE_GAP_EVENT_CONNECT:
    	   ESP_LOGI(TAG, "Master connected");
           ble_connected = true;   // ← 追加
        break;
        case BLE_GAP_EVENT_DISCONNECT:
           ESP_LOGI(TAG, "Master disconnected");
           ble_connected = false;  // ← 追加
           ble_data_exchanged = true;
           ble_advertise();
        break;
    }
    return 0;
}

static void ble_advertise(void)
{
    struct ble_gap_adv_params adv_params = {0};
    struct ble_hs_adv_fields fields = {0};
    char device_name[16];

    snprintf(device_name, sizeof(device_name), "%s%02d",
             SLAVE_NAME_PREFIX, MY_SLAVE_ID);

    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                      &adv_params, gap_event_handler, NULL);

    ESP_LOGI(TAG, "BLE advertising as '%s'", device_name);
}

static void nimble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void ble_init_and_advertise(void)
{
    nimble_port_init();
    ble_svc_gap_init();
    ble_svc_gatt_init();   // ← GATTサービス基盤を追加
    ble_hs_cfg.sync_cb = ble_advertise;
    nimble_port_freertos_init(nimble_host_task);
    ESP_LOGI(TAG, "NimBLE initialized");
}

static void ble_deinit(void)
{
    nimble_port_stop();
    nimble_port_deinit();
    ESP_LOGI(TAG, "BLE deinitialized");
}


/* ============================================================
 * GATT メンテナンスサービス実装
 * ============================================================ */

// コマンド定義
#define CMD_PUMP_ON         0x01
#define CMD_PUMP_OFF        0x02
#define CMD_SET_PARAMS      0x03
#define CMD_RESET           0x04

// 接続ハンドル
static uint16_t maint_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t maint_read_val_handle = 0;

// センサーデータをバイト列に変換して返す
static int gatt_maint_read_cb(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    // 最新センサーデータを読み取り
    read_sensors(&my_sensor_data);

    int rc = os_mbuf_append(ctxt->om,
                            &my_sensor_data,
                            sizeof(sensor_data_t));
    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

// スマホからの書き込み処理
static int gatt_maint_write_cb(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint8_t cmd = 0;
    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);

    if (len < 1) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

    os_mbuf_copydata(ctxt->om, 0, 1, &cmd);
    ESP_LOGI(TAG, "GATT Write: cmd=0x%02X len=%d", cmd, len);

    switch (cmd) {
        case CMD_PUMP_ON: {
            uint16_t on_ms = my_params.pump_on_time_ms;
            if (len >= 3) {
                os_mbuf_copydata(ctxt->om, 1, 2, &on_ms);
            }
            ESP_LOGI(TAG, "Pump ON for %d ms", on_ms);
            gpio_set_level(GPIO_PUMP, 1);
            vTaskDelay(pdMS_TO_TICKS(on_ms));
            gpio_set_level(GPIO_PUMP, 0);
            ESP_LOGI(TAG, "Pump OFF");
            break;
        }
        case CMD_PUMP_OFF:
            gpio_set_level(GPIO_PUMP, 0);
            ESP_LOGI(TAG, "Pump forced OFF");
            break;

        case CMD_SET_PARAMS: {
            if (len < 1 + sizeof(control_params_t)) {
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }
            control_params_t new_params;
            os_mbuf_copydata(ctxt->om, 1, sizeof(control_params_t), &new_params);
            // NVSに保存
            config_store_set_u16("pump_t",  new_params.pump_on_time_ms);
            config_store_set_float("wtr_th", new_params.water_threshold);
            config_store_set_u16("sleep",   new_params.sleep_duration_min);
            my_params = new_params;
            ESP_LOGI(TAG, "Params updated: pump=%dms threshold=%.1f sleep=%dmin",
                     my_params.pump_on_time_ms,
                     my_params.water_threshold,
                     my_params.sleep_duration_min);
            break;
        }
        case CMD_RESET:
            ESP_LOGI(TAG, "Reset command received");
            esp_restart();
            break;

        default:
            ESP_LOGW(TAG, "Unknown command: 0x%02X", cmd);
            break;
    }
    return 0;
}

// UUID定義（staticで永続化）
static ble_uuid128_t svc_uuid = {
    .u = { .type = BLE_UUID_TYPE_128 },
    .value = { 0xde, 0xf9, 0x89, 0x56, 0x34, 0x12,
               0x34, 0x12, 0x34, 0x12, 0x34, 0x12,
               0x78, 0x56, 0x34, 0x12 }
};
static ble_uuid128_t chr_read_uuid = {
    .u = { .type = BLE_UUID_TYPE_128 },
    .value = { 0xe1, 0xde, 0x89, 0x56, 0x34, 0x12,
               0x34, 0x12, 0x34, 0x12, 0x34, 0x12,
               0x78, 0x56, 0x34, 0x12 }
};
static ble_uuid128_t chr_write_uuid = {
    .u = { .type = BLE_UUID_TYPE_128 },
    .value = { 0xe2, 0xde, 0x89, 0x56, 0x34, 0x12,
               0x34, 0x12, 0x34, 0x12, 0x34, 0x12,
               0x78, 0x56, 0x34, 0x12 }
};

// Characteristics配列（staticで独立定義）
static const struct ble_gatt_chr_def gatt_maint_chrs[] = {
    {
        .uuid = &chr_read_uuid.u,
        .access_cb = gatt_maint_read_cb,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
        .val_handle = &maint_read_val_handle,
    },
    {
        .uuid = &chr_write_uuid.u,
        .access_cb = gatt_maint_write_cb,
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
    },
    { 0 }
};

static const struct ble_gatt_svc_def gatt_maint_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &svc_uuid.u,
        .characteristics = gatt_maint_chrs,
    },
    { 0 }
};

// GATTサービス初期化
static void gatt_maint_init(void)
{
    int rc = ble_gatts_count_cfg(gatt_maint_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_count_cfg failed: %d", rc);
        return;
    }
    rc = ble_gatts_add_svcs(gatt_maint_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gatts_add_svcs failed: %d", rc);
        return;
    }
    ESP_LOGI(TAG, "GATT maintenance service registered");
}

/* ============================================================
 * メンテナンスモード
 * ============================================================ */
static void maintenance_mode(void)
{
    ESP_LOGW(TAG, "=== MAINTENANCE MODE ===");

    // パラメータ読み込み
    load_params();
    int idle_count = 0;  // 未接続カウンター
    // GATTサービス登録
//    gatt_maint_init();

    ESP_LOGW(TAG, "Connect via BLE (PLANT_0%d) to configure", MY_SLAVE_ID);

    // LEDチカチカしながらスイッチOFF待ち
    while (1) {
        gpio_set_level(GPIO_LED_STATUS, 1);
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level(GPIO_LED_STATUS, 0);
        vTaskDelay(pdMS_TO_TICKS(200));

        if (ble_connected) {
            idle_count = 0;  // 接続中はリセット
        } else {
            idle_count++;
            // 30秒（400ms×75回）経過後にスイッチOFFでリブート
            if (idle_count > 75 && gpio_get_level(GPIO_MODE_SWITCH) == 1) {
                ESP_LOGI(TAG, "Mode switch OFF, rebooting...");
                gpio_set_level(GPIO_PUMP, 0);
                esp_restart();
            }
        }
    }
}


/* ============================================================
 * ディープスリープ  ★変更箇所1★
 * ============================================================
 *
 * 起床ソース:
 *   A) タイマー  : 通常の定期起床 (10分)
 *   B) ext1 GPIO : GPIO_MODE_SWITCH(GPIO7) がLOW → メンテナンスモード
 *
 * ESP32C6はext0非対応のため ext1 を使用。
 * GPIO7はRTC GPIO(0-7)なのでディープスリープ中も監視可能。
 */
static void enter_deep_sleep(void)
{
//    esp_sleep_config_gpio_isolate();  // ← この行を追加
    gpio_sleep_sel_dis(GPIO_MODE_SWITCH);  // ← この行を追加
    uint16_t sleep_min = my_params.sleep_duration_min;
    if (sleep_min == 0) sleep_min = DEFAULT_DEEP_SLEEP_MIN;

//    uint64_t sleep_us = (uint64_t)sleep_min * 60 * 1000000ULL;

//    ESP_LOGI(TAG, "Entering deep sleep for %d minutes...", sleep_min);
    // テスト用：30秒固定
    uint64_t sleep_us = 30ULL * 1000000ULL;

    ESP_LOGI(TAG, "Entering deep sleep for 30 seconds...");


    // タイマーウェイクアップ (通常起床)
    esp_sleep_enable_timer_wakeup(sleep_us);

    // ext1ウェイクアップ: GPIO7がLOW → メンテナンスモード起床
    // プルアップ構成のため、スイッチ押下でLOWになる
    uint64_t gpio_mask = (1ULL << GPIO_MODE_SWITCH);
    esp_sleep_enable_ext1_wakeup(gpio_mask, ESP_EXT1_WAKEUP_ANY_LOW);
//    gpio_hold_en(GPIO_MODE_SWITCH);
    esp_deep_sleep_start();
    // ↑ ここから先は実行されない
}

/* ============================================================
 * GPIO初期化
 * ============================================================ */
static void gpio_init(void)
{
    // ポンプ・LED (出力)
    gpio_config_t io_conf_out = {
        .pin_bit_mask = (1ULL << GPIO_PUMP) | (1ULL << GPIO_LED_STATUS),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf_out);
    gpio_set_level(GPIO_PUMP, 0);
    gpio_set_level(GPIO_LED_STATUS, 0);

    // モードスイッチ (入力、内蔵プルアップ)
    gpio_config_t io_conf_in = {
        .pin_bit_mask = (1ULL << GPIO_MODE_SWITCH),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf_in);
}

/* ============================================================
 * 制御パラメータ読み出し
 * ============================================================ */
static void load_params(void)
{
    my_params.slave_id           = MY_SLAVE_ID;
    my_params.pump_on_time_ms    = 3000;
    my_params.water_threshold    = DEFAULT_WATER_HUMIDITY_THRESHOLD;
    my_params.sleep_duration_min = DEFAULT_DEEP_SLEEP_MIN;
    my_params.flags              = 0;

    config_store_get_u16("pump_t",  &my_params.pump_on_time_ms);
    config_store_get_float("wtr_th", &my_params.water_threshold);
    config_store_get_u16("sleep",   &my_params.sleep_duration_min);

    ESP_LOGI(TAG, "Params: pump=%dms, threshold=%.1f, sleep=%dmin",
             my_params.pump_on_time_ms,
             my_params.water_threshold,
             my_params.sleep_duration_min);
}

/* ============================================================
 * app_main()  ★変更箇所2★
 * ============================================================ */
#include "esp_task_wdt.h"
void app_main(void)
{
    boot_count++;
    // シリアルモニタ再接続待ち (起床直後のログを取りこぼさないため)
    vTaskDelay(pdMS_TO_TICKS(3000));  // ← 追加

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " Plant Management System - Slave #%d", MY_SLAVE_ID);
    ESP_LOGI(TAG, " Boot count: %lu", boot_count);
    ESP_LOGI(TAG, " Free heap: %lu bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "========================================");

    // --- 1. 基本初期化 ---
    config_store_init();
    ESP_LOGI(TAG, ">>> config_store OK");
    gpio_init();
    ESP_LOGI(TAG, ">>> gpio_init OK");
    vTaskDelay(pdMS_TO_TICKS(50));  // GPIO安定待ち（50msで十分）
    ESP_LOGI(TAG, ">>> GPIO7 level: %d", gpio_get_level(GPIO_MODE_SWITCH));  // ← 追加
    // --- 2. 起床原因判定 ---
    esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
    ESP_LOGI(TAG, "Wakeup cause: %d", wakeup_cause);

//    if (gpio_get_level(GPIO_MODE_SWITCH) == 0) {
    if (wakeup_cause == ESP_SLEEP_WAKEUP_EXT1) {
       ESP_LOGW(TAG, "Wakeup by maintenance switch (GPIO%d)", GPIO_MODE_SWITCH);
       esp_task_wdt_deinit();
//       gatt_maint_init();         // 1. GATTサービス登録
       ble_init_and_advertise();  // 2. NimBLE起動
       maintenance_mode();
    }
    load_params();
    ESP_LOGI(TAG, ">>> load_params OK");

    // 通常モード (タイマー起床 or 初回起動)
    // 安全弁: スイッチが押されたままの状態でも対応
    if (gpio_get_level(GPIO_MODE_SWITCH) == 0) {
        ESP_LOGW(TAG, "Mode switch ON at boot (fallback)");
        esp_task_wdt_deinit();
        ble_init_and_advertise();
        maintenance_mode();
        // ↑ ここからは戻ってこない
    }

    // --- 3. 通常モード: センサー読み取り ---
    read_sensors(&my_sensor_data);

    // --- 4. 水やり判定 ---
    check_and_water(&my_sensor_data);

    // --- 5. BLE通信 (親機にデータ送信) ---
    esp_task_wdt_deinit();
    ble_init_and_advertise();
    ESP_LOGI(TAG, "Waiting for BLE connection...");

    int wait_count = 0;
    while (!ble_data_exchanged && wait_count < BLE_ADV_TIMEOUT_SEC * 10) {
        vTaskDelay(pdMS_TO_TICKS(100));
        wait_count++;
    }

    if (ble_data_exchanged) {
        ESP_LOGI(TAG, "Data exchange with master complete");
    } else {
        ESP_LOGW(TAG, "Master connection timeout");
    }

    // --- 6. BLE停止 & ディープスリープ ---
    ble_deinit();

    if (my_params.flags & FLAG_FORCE_WAKE) {
        ESP_LOGW(TAG, "Force wake flag set, not sleeping");
        esp_restart();
    }
    ESP_LOGI(TAG, ">>> GPIO7 level before sleep: %d", gpio_get_level(GPIO_MODE_SWITCH));
    enter_deep_sleep();
    // ↑ ここから先は実行されない
}