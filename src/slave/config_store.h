/*
 * 設定値の保存・読み出し (NVS wrapper)
 *
 * ESP-IDFのNVS (Non-Volatile Storage) をラップして、
 * 設定値の保存・読み出しを簡単にする。
 * ArduinoのPreferencesライブラリに相当する。
 *
 * 使い方:
 *   config_store_init();                     // 初期化
 *   config_store_set_u16("pump_time", 3000); // 保存
 *   uint16_t val = 0;
 *   config_store_get_u16("pump_time", &val); // 読み出し
 */

#ifndef CONFIG_STORE_H
#define CONFIG_STORE_H

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * NVSの初期化
 * app_main()の最初に1回呼ぶ
 */
esp_err_t config_store_init(void);

/**
 * 値の保存・読み出し
 * key: 最大15文字のキー名
 */
esp_err_t config_store_set_u8(const char *key, uint8_t value);
esp_err_t config_store_get_u8(const char *key, uint8_t *value);

esp_err_t config_store_set_u16(const char *key, uint16_t value);
esp_err_t config_store_get_u16(const char *key, uint16_t *value);

esp_err_t config_store_set_u32(const char *key, uint32_t value);
esp_err_t config_store_get_u32(const char *key, uint32_t *value);

esp_err_t config_store_set_float(const char *key, float value);
esp_err_t config_store_get_float(const char *key, float *value);

esp_err_t config_store_set_str(const char *key, const char *value);
esp_err_t config_store_get_str(const char *key, char *value, size_t max_len);

/**
 * 全設定のリセット (工場出荷状態に戻す)
 */
esp_err_t config_store_reset(void);

#ifdef __cplusplus
}
#endif

#endif // CONFIG_STORE_H
