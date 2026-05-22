# 植物管理システム (Plant Management System)

ESP32ベースの植物自動管理システム。
親機・子機構成で、最大6台の鉢植えを自動監視・水やりする。

## システム構成

```
┌─────────┐     BLE      ┌─────────┐
│  子機1   │◄────────────►│         │
│ (ESP32)  │              │         │     WiFi      ┌──────────┐
├─────────┤              │  親機    │◄────────────►│  Ambient  │
│  子機2   │◄────────────►│(ESP32-S3│              │ (クラウド) │
│ (ESP32)  │              │  / C6)  │              └──────────┘
├─────────┤              │         │
│  ...     │◄────────────►│         │     (将来)     ┌──────────┐
├─────────┤              │         │◄─────────────►│  MATTER   │
│  子機6   │◄────────────►│         │              │ HomeKit等 │
│ (ESP32)  │              └─────────┘              └──────────┘
└─────────┘

┌─────────┐     BLE      ┌──────────┐
│  子機    │◄────────────►│ スマホ    │  ※メンテナンスモード時のみ
│(メンテ)  │              │(nRF Connect)│
└─────────┘              └──────────┘
```

## 子機の動作サイクル

```
ディープスリープ (10分)
    │
    ▼
  起床
    │
    ▼
センサー読み取り ─── 温度, 湿度, 光量, 水タンク, 電圧
    │
    ▼
水やり判定 ─── 湿度 < 閾値 → ポンプON (設定時間)
    │
    ▼
BLEアドバタイズ ─── 親機を待つ (最大30秒)
    │
    ├── 親機接続 → データ送信 & 設定受信
    │
    ▼
BLE停止 → ディープスリープへ
```

## ディレクトリ構成

```
plant-system/
├── platformio.ini              # ビルド設定 (親機/子機の環境定義)
├── partitions_master.csv       # 親機用パーティション
├── partitions_slave.csv        # 子機用パーティション
├── lib/
│   ├── ble_protocol/
│   │   └── ble_protocol.h      # ★ BLE通信プロトコル定義 (共通)
│   ├── config_store/
│   │   ├── config_store.h      # NVS設定保存 (共通)
│   │   └── config_store.c
│   └── sensor_common/          # (今後) センサードライバ
├── src/
│   ├── master/
│   │   └── main.c              # ★ 親機メインプログラム
│   └── slave/
│       └── main.c              # ★ 子機メインプログラム
└── README.md
```

## 開発環境セットアップ

### 必要なもの
- **VSCode** + **PlatformIO拡張**
- **ESP32ボード** (子機: ESP32 / 親機: ESP32-S3 or C6)
- USBケーブル

### PlatformIOインストール
1. VSCodeの拡張機能から「PlatformIO IDE」をインストール
2. このフォルダをVSCodeで開く
3. PlatformIOが自動でESP-IDFのツールチェインをダウンロード

### ビルド & 書き込み

```bash
# 親機をビルド
pio run -e master

# 子機をビルド
pio run -e slave

# 親機をESP32に書き込み
pio run -e master -t upload

# 子機をESP32に書き込み
pio run -e slave -t upload

# シリアルモニタ (ログ確認)
pio device monitor
```

## 段階的な実装ガイド

### Phase 1: 基礎通信
- [ ] 子機: Lチカ + ディープスリープの動作確認
- [ ] 子機: BLEアドバタイズの動作確認
- [ ] 親機: BLEスキャンで子機を発見
- [ ] 親機-子機間のBLE接続・データ交換

### Phase 2: センサー & 制御
- [ ] 子機: 実センサーの接続とドライバ実装
- [ ] 子機: 水やりポンプの制御
- [ ] 子機: NVSへのパラメータ保存

### Phase 3: クラウド連携
- [ ] 親機: WiFi接続
- [ ] 親機: AmbientへのHTTP POST
- [ ] 複数子機 (2台以上) の対応

### Phase 4: メンテナンス
- [ ] 子機: メンテナンスモードの実装
- [ ] スマホからのBLE接続とパラメータ変更

### Phase 5: MATTER対応 (オプション)
- [ ] 親機: ESP-MATTERの導入
- [ ] HomeKit/Google Homeからのセンサー値表示
- [ ] 水やりのリモート操作

## ESP-IDF vs Arduino 対応表

| Arduino            | ESP-IDF                          |
|-------------------|----------------------------------|
| `setup()`         | `app_main()`                     |
| `loop()`          | `while(1)` in task or app_main   |
| `delay(ms)`       | `vTaskDelay(pdMS_TO_TICKS(ms))`  |
| `Serial.println()`| `ESP_LOGI(TAG, "...")`           |
| `digitalWrite()`  | `gpio_set_level()`               |
| `digitalRead()`   | `gpio_get_level()`               |
| `analogRead()`    | `adc_oneshot_read()`             |
| `ESP.deepSleep()` | `esp_deep_sleep_start()`         |
| `Preferences`     | `nvs_get/set_xxx()`             |
| `WiFi.begin()`    | `esp_wifi_start()`               |

## BLE通信プロトコル

### データ構造 (ble_protocol.h参照)

**子機→親機 (sensor_data_t):**
温度, 湿度, 光量, 水タンク残量, 電源電圧, ポンプON時間, 起動回数

**親機→子機 (control_params_t):**
ポンプON時間設定, 水やり閾値, スリープ時間, フラグ(強制水やり等)
