/*
 * BLE通信プロトコル定義
 * 親機・子機で共有するデータ構造とUUID
 *
 * ■ 通信の流れ:
 *   1. 子機がディープスリープから起床
 *   2. 子機がBLEアドバタイズ開始（サービスUUIDを含む）
 *   3. 親機がアドバタイズを検出し接続
 *   4. 子機→親機: センサーデータ送信 (sensor_data_t)
 *   5. 親機→子機: 制御パラメータ送信 (control_params_t)
 *   6. 切断 → 子機はディープスリープへ
 */

#ifndef BLE_PROTOCOL_H
#define BLE_PROTOCOL_H

#include <stdint.h>

/* ============================================================
 * BLE UUID定義
 * ============================================================ */

// 植物管理サービス (通常モード)
// カスタムUUID: 子機がアドバタイズ時にこのUUIDを含める
#define PLANT_SERVICE_UUID          "12345678-1234-1234-1234-123456789abc"

// Characteristic UUID
#define CHAR_SENSOR_DATA_UUID       "12345678-1234-1234-1234-123456789ab1"  // 子機→親機
#define CHAR_CONTROL_PARAMS_UUID    "12345678-1234-1234-1234-123456789ab2"  // 親機→子機
#define CHAR_DEVICE_STATUS_UUID     "12345678-1234-1234-1234-123456789ab3"  // 子機状態

// メンテナンスサービス (スマホ接続用)
#define MAINT_SERVICE_UUID          "12345678-1234-1234-1234-123456789def"
#define CHAR_MAINT_READ_UUID        "12345678-1234-1234-1234-123456789de1"  // 読み取り
#define CHAR_MAINT_WRITE_UUID       "12345678-1234-1234-1234-123456789de2"  // 書き込み

/* ============================================================
 * 子機ID定義 (最大6台)
 * ============================================================ */
#define MAX_SLAVES          6

// 子機のBLEデバイス名プレフィックス
// 実際の名前は "PLANT_01" ~ "PLANT_06"
#define SLAVE_NAME_PREFIX   "PLANT_"

/* ============================================================
 * データ構造体
 * ============================================================ */

// --- 子機 → 親機: センサーデータ ---
typedef struct __attribute__((packed)) {
    uint8_t  slave_id;              // 子機ID (1-6)
    float    temperature;           // 温度 [℃]
    float    humidity;              // 湿度 [%]
    float    light;                 // 光量 [lux]
    float    water_tank_level;      // 水タンク残量 [%]
    float    battery_voltage;       // 電源電圧 [V]
    uint16_t pump_on_duration_ms;   // 今回の水やりポンプON時間 [ms] (0=動かしてない)
    uint32_t uptime_count;          // 起動回数 (デバッグ用)
} sensor_data_t;

// --- 親機 → 子機: 制御パラメータ ---
typedef struct __attribute__((packed)) {
    uint8_t  slave_id;              // 子機ID (1-6)
    uint16_t pump_on_time_ms;       // 水やりポンプON時間設定 [ms]
    float    water_threshold;       // 水やり開始湿度閾値 [%]
    uint16_t sleep_duration_min;    // ディープスリープ時間 [分]
    uint8_t  flags;                 // ビットフラグ (下記参照)
} control_params_t;

// control_params_t.flags のビット定義
#define FLAG_FORCE_WATER    (1 << 0)  // 強制水やり指示
#define FLAG_FORCE_WAKE     (1 << 1)  // 次回起動時スリープしない
#define FLAG_UPDATE_CONFIG  (1 << 2)  // 設定変更あり→NVS保存せよ

// --- 子機ステータス ---
typedef struct __attribute__((packed)) {
    uint8_t  slave_id;
    uint8_t  mode;                  // 0=通常, 1=メンテナンス
    uint8_t  error_code;            // 0=正常, それ以外=エラー
    uint16_t free_heap_kb;          // 空きヒープ [KB]
} device_status_t;

/* ============================================================
 * エラーコード定義
 * ============================================================ */
#define ERR_NONE                0x00
#define ERR_SENSOR_TEMP         0x01  // 温度センサー異常
#define ERR_SENSOR_HUMIDITY     0x02  // 湿度センサー異常
#define ERR_SENSOR_LIGHT        0x04  // 光量センサー異常
#define ERR_WATER_TANK_EMPTY    0x08  // 水タンク空
#define ERR_LOW_BATTERY         0x10  // バッテリー低下
#define ERR_PUMP_FAULT          0x20  // ポンプ異常

/* ============================================================
 * Ambient設定 (親機で使用)
 * ============================================================ */
#define AMBIENT_CHANNEL_ID      0       // 要設定: AmbientのチャネルID
#define AMBIENT_WRITE_KEY       ""      // 要設定: Ambientのライトキー

// Ambientは1チャネルあたり8フィールド (d1-d8)
// 子機1台あたり: 温度, 湿度, 光量, 水タンク, 電圧 = 5フィールド
// → 6台分は1チャネルに収まらないので、複数チャネルまたはフィールドのローテーションが必要
// ここでは子機ごとにチャネルを分ける設計とする
typedef struct {
    uint32_t channel_id;
    char     write_key[20];
} ambient_config_t;

#endif // BLE_PROTOCOL_H
