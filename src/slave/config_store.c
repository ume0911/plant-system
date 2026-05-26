/*
 * config_store.c - NVSラッパー実装
 */

#include "config_store.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "config_store";
static const char *NVS_NAMESPACE = "plant_cfg";

esp_err_t config_store_init(void)
{
    // NVSフラッシュの初期化
    // 初回やパーティション変更時は自動でフォーマット
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition truncated, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_LOGI(TAG, "NVS initialized");
    return ret;
}

/* --- uint8_t --- */
esp_err_t config_store_set_u8(const char *key, uint8_t value)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) return ret;

    ret = nvs_set_u8(handle, key, value);
    if (ret == ESP_OK) ret = nvs_commit(handle);
    nvs_close(handle);
    return ret;
}

esp_err_t config_store_get_u8(const char *key, uint8_t *value)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) return ret;

    ret = nvs_get_u8(handle, key, value);
    nvs_close(handle);
    return ret;
}

/* --- uint16_t --- */
esp_err_t config_store_set_u16(const char *key, uint16_t value)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) return ret;

    ret = nvs_set_u16(handle, key, value);
    if (ret == ESP_OK) ret = nvs_commit(handle);
    nvs_close(handle);
    return ret;
}

esp_err_t config_store_get_u16(const char *key, uint16_t *value)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) return ret;

    ret = nvs_get_u16(handle, key, value);
    nvs_close(handle);
    return ret;
}

/* --- uint32_t --- */
esp_err_t config_store_set_u32(const char *key, uint32_t value)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) return ret;

    ret = nvs_set_u32(handle, key, value);
    if (ret == ESP_OK) ret = nvs_commit(handle);
    nvs_close(handle);
    return ret;
}

esp_err_t config_store_get_u32(const char *key, uint32_t *value)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) return ret;

    ret = nvs_get_u32(handle, key, value);
    nvs_close(handle);
    return ret;
}

/* --- float (blobとして保存) --- */
esp_err_t config_store_set_float(const char *key, float value)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) return ret;

    ret = nvs_set_blob(handle, key, &value, sizeof(float));
    if (ret == ESP_OK) ret = nvs_commit(handle);
    nvs_close(handle);
    return ret;
}

esp_err_t config_store_get_float(const char *key, float *value)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) return ret;

    size_t len = sizeof(float);
    ret = nvs_get_blob(handle, key, value, &len);
    nvs_close(handle);
    return ret;
}

/* --- string --- */
esp_err_t config_store_set_str(const char *key, const char *value)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) return ret;

    ret = nvs_set_str(handle, key, value);
    if (ret == ESP_OK) ret = nvs_commit(handle);
    nvs_close(handle);
    return ret;
}

esp_err_t config_store_get_str(const char *key, char *value, size_t max_len)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (ret != ESP_OK) return ret;

    ret = nvs_get_str(handle, key, value, &max_len);
    nvs_close(handle);
    return ret;
}

/* --- リセット --- */
esp_err_t config_store_reset(void)
{
    nvs_handle_t handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (ret != ESP_OK) return ret;

    ret = nvs_erase_all(handle);
    if (ret == ESP_OK) ret = nvs_commit(handle);
    nvs_close(handle);
    ESP_LOGW(TAG, "All config reset to defaults");
    return ret;
}
