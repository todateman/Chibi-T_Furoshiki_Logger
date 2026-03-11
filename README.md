# Chibi-T Furoshiki Logger (M5Stack Core2)

M5Stack Core2 上で動作するエコラン競技車両向けロガー兼リアルタイム表示システムです。以下を統合しています:

- ECU からの走行データ取得 (Serial1)
- GNSS 位置・時刻取得 (Serial2, TinyGPS++)
- BLE 経由のエンジン温度受信 (SoftwareSerial or HardwareSerial)
- SD カードへの CSV ロギング (SdFat)
- Wi-Fi (任意) + MQTT (AWS IoT Core) / Ambient 送信
- LCD (M5Unified + LovyanGFX) への複数モード表示

## 主な機能概要

| 機能 | 内容 |
| ---- | ---- |
| 周回/距離管理 | 位置範囲からサーキット判定 (鈴鹿/茂木/その他) と周回数・走行時間計算 |
| 表示モード切替 | Bボタン=`モード 0` 速度/残周回/走行時間(デフォルト)<BR>Aボタン=`モード 1` 回転数/噴射時間/進角(セッティング向け)<BR>Cボタン=`モード 2` 走行ルート標高グラフ + 現在地標高プロット |
| ウェイポイント標高グラフ | `sd`ディレクトリのウェイポイントCSV (`suzuka_waypoint.csv` / `motegi_waypoint.csv` / `toyota_waypoint.csv`) を読み込み、横軸=ウェイポイントID、縦軸=標高で表示。現在地の緯度経度を最も近いIDへ割り当てて標高を重ねて表示 |
| ログ保存 | `/LOG/LOGxxxx.CSV` (UTF-8 BOM付き, ヘッダ日本語) |
| 高度推定 | BMP280気圧から高度推定 (標準大気式) |
| 海面気圧補正 | Open-Meteoから`pressure_msl`を1回取得して高度基準を補正 |
| MQTT 送信 | JSON ペイロードを `mqtt_topic` へ (証明書による TLS) |
| Ambient 送信 | 10 フィールド + 位置情報文字列 |
| セーフ処理 | GNSS 異常値除外<BR>BLEタイムアウト時温度リセット<BR>ECU無信号時フェールセーフ |

## ハードウェア / 接続

- 基板: M5Stack Core2 (ESP32, PSRAM 使用)
- SD: SPI (GPIO4 / SHARED_SPI 設定)
- ECU: Serial1 115200 bps (RX=27, TX=19) ※コード参照
- GNSS: Serial2 38400 bps ([M5Stack GNSS Module](https://docs.m5stack.com/ja/module/GNSS%20Module) DIP-SW TX:1, RX:1)
- BMP280(気圧センサ): I2C 0x76 ([M5Stack GNSS Module](https://docs.m5stack.com/ja/module/GNSS%20Module)
- BLE 温度センサ: RX=32, TX=33 (SoftwareSerial 既定, `USE_HARDWARE_BLE=1` で UART2 を利用可)
- ボタン: A/B/C でモード選択 + 起動時設定

## ソフトウェア依存ライブラリ (platformio.ini より)

- [TinyGPSPlus](https://github.com/mikalhart/TinyGPSPlus)
- [Ambient ESP32 ESP8266 lib](https://github.com/AmbientDataInc/Ambient_ESP8266_lib)
- [M5Unified](https://github.com/m5stack/M5Unified)
- [TimeLib](https://github.com/derickr/timelib)
- [PubSubClient](https://github.com/knolleary/pubsubclient)
- [ArduinoJson v7](https://github.com/bblanchon/ArduinoJson)
- [EspSoftwareSerial](https://github.com/plerup/espsoftwareserial)
- [SdFat](https://github.com/greiman/SdFat)
- [WiFiManager](https://github.com/tzapu/WiFiManager) (同梱ライブラリ `lib/WiFiManager`)
- [Adafruit BMP280 Library](https://github.com/adafruit/Adafruit_BMP280_Library)
- [Adafruit Unified Sensor](https://github.com/adafruit/Adafruit_Sensor)

## ビルド & 実行 (PlatformIO)

1. VS Code + PlatformIO をインストール
2. 本リポジトリを開く
3. board: `m5stack-core2` を指定 (既に `platformio.ini` 設定済み)
4. Upload (書き込み) / Monitor (115200bps) で動作確認
5. 初回起動時 SD カードが挿入されていることを確認

### デバッグ

- シリアル出力: CSV形式 `lat,lon,alt,loc,Spd_GPS,rpm,Spd_PULSE,distance,gasml,dispergas,worktime,Temp`
- 例外デコード: `monitor_filters = esp32_exception_decoder`

## 高度推定 / Open-Meteo 補正

- 高度は GNSS ではなく BMP280 の気圧から算出
- 海面気圧は初期値 `101.325kPa` を使用
- Wi-Fi接続済みかつGNSS座標が有効なときのみ Open-Meteo API を低頻度で試行
- Open-Meteoの`pressure_msl`取得に成功したら1回だけ海面気圧を置換し、以後は再取得しない
- 通信遅延抑制のため Open-Meteo 取得は HTTP で実行

## 起動時の Wi-Fi 操作

| 操作 | 説明 |
| ---- | ---- |
| A ボタン | Wi-Fi 設定ポータル (AP モード, QR コードで簡易接続) |
| C ボタン | Wi-Fi 無効 (Ambient / MQTT も無効) |
| 無操作 (5秒) | 自動接続 (保存済 SSID) / 失敗時は接続先SSIDを設定するWebUIへのQRコードを表示 |

## 表示モード詳細

1. モード 0: 速度, 残り周回 (最後は G/FINISH), 走行時間進行バー
2. モード 1: 回転数, 噴射時間, 進角角度バー
3. モード 2: 走行ルートの標高グラフ（ウェイポイントID基準）と現在地標高を重ねて表示

### モード2(標高グラフ)の仕様

- サーキット判定 (`Loc`: su / mo / to) に応じて対応CSVを自動読み込み
- 現在地の緯度経度をもとに、ウェイポイントの最近傍IDを算出
- グラフ内に「ルート標高」と「現在地標高」を同時プロット
- 右端と左端は接続せず、先頭IDから末尾IDまでを表示

### モード2(標高グラフ)の画面例

https://github.com/user-attachments/assets/a7fdfc0b-4c62-4426-a9e1-d857ddd0c710

- 水色線: ウェイポイントファイルの標高プロファイル
- 黄色点: 最近傍ウェイポイントIDの標高
- 赤色点: 現在地標高
- 下部テキスト: `WP:現在ID/総ポイント数 route:ルート標高 now:現在地標高`

## ログファイル仕様 (SD)

ヘッダ: `記録日時,速度(km/h),ラップ数,走行時間,回転数,走行距離,積算燃料,燃費,lat,lon,alt,loc,温度`

- 記録日時: GNSS + JST補正 (`YYYY/M/D hh:mm:ss.cc`)
- 走行時間: ECU送信の積算秒
- 燃費: コード中 `dispergas` (km/L 指定の閾値 2000 スケールバー)
- タイムスタンプは `logFile.timestamp()` によりファイル更新時にも設定
- SD書き込みはファイルを開きっぱなしで運用し、定期 `sync()` で書き込み確定  
  （電源切では`sync()`実行済みのログを残す）

## MQTT 送信仕様

- 接続先:  
  **セキュリティ対応のため接続先設定はsecrets.hに記載し、secrets.hはGitのトラッキングの対象外とする**
  - AWS IoT Core (TLS)`mqtt_server` (ATS endpoint)
  - port `mqtt_port`
  - クライアント証明書 + 秘密鍵 + Amazon Root CA 1

- QoS: 0(接続先での欠損を許容する)

- ペイロード例:

```json
{
  "timestamp": "2025/11/09 12:34:56.12",
  "Spd_PULSE": 42,
  "Lapcount": 3,
  "worktime": 375,
  "tachoRpm": 5200,
  "distance": 1234,
  "gasml": 57.8,
  "dispergas": 21.3,
  "lat": 34.1234567,
  "lon": 136.1234567,
  "loc": "su",
  "temp": 92.5
}
```

- バッファサイズ: `MQTT_BUFFER_SIZE` (既定 512) → 変更時は `PubSubClient` の制約に注意
- 送信間隔: 走行前 `10s`, 走行中 `1s`

## Ambient 送信仕様

フィールド番号: 1=速度, 2=温度, 3=ラップ, 4=走行時間, 5=回転数, 6=距離, 7=積算燃料, 8=燃費, 9=lat文字列, 10=lon文字列

- `userKey` と `devKey` (MAC アドレスから生成) を利用して `channelId` / `writeKey` を動的に取得  
  (secrets.h に記載しての固定はしない)
- 送信タイムアウト 1000 ms

## secrets.h 管理

**認証情報は Git に含めないでください。**  
`.gitignore` には既に `secrets.h` が登録されています。  
以下の変数を `src/secrets.h` に定義します (実際の値は環境に合わせて置換):

```cpp
const char* mqtt_server;      // AWS IoT Core エンドポイント (xxxxxxxxx-ats.iot.<region>.amazonaws.com)
const int   mqtt_port;        // 8883 推奨 (TLS)
const char* mqtt_topic;       // 例: "Furoshiki/M5Logger"
const char* mqtt_deviceID;    // Thing Name / クライアントID
const char* userKey;          // Ambient ユーザーキー
static const char AWS_CERT_CA[] PROGMEM;       // Amazon Root CA 1
static const char AWS_CERT_CRT[] PROGMEM;      // デバイス証明書 (-----BEGIN CERTIFICATE-----)
static const char AWS_CERT_PRIVATE[] PROGMEM;  // デバイス秘密鍵 (-----BEGIN RSA PRIVATE KEY-----)
```

### テンプレート

`src/secrets.example.h` を参考に `src/secrets.h` を作成し、実値を記入してください。`secrets.example.h` は **公開可**、`secrets.h` は **非公開**。

### AWS IoT Core 設定手順 (概要)

1. AWS IoT Core で Thing 作成
2. 証明書 (CRT + Private Key + Amazon Root CA 1) を取得
3. ポリシーを証明書にアタッチ (iot:Connect / iot:Publish / iot:Subscribe / iot:Receive)
4. エンドポイントを `mqtt_server` に設定
5. Topic 名をコード/運用で統一 (`mqtt_topic`)

### セキュリティ注意事項

- 秘密鍵は絶対にコミットしない
- 公開環境ではデバッグ出力に秘密情報を含めない
- 証明書ローテーション時は再ビルド必須

## トラブルシュート

| 症状 | 対処 |
| ---- | ---- |
| SD init failed | FAT/exFAT フォーマット <BR> SPI 接続確認, 遅延を長くする検討 |
| MQTT connect失敗 | 証明書有効性/時刻同期 (GNSSで JST 変換) <BR> ポリシー権限確認 |
| Ambient failure | Wi-Fi RSSI / userKey / devKey/channelId 取得失敗再試行 |
| 温度 0.0 固定 | BLE センサ未送信 or タイムアウト <BR>  (>10s でリセット) |

## 既知の課題 (改善予定)

1. 電源断耐性: 開きっぱなし運用のため、電源断時は最後の `sync()` 以降の数秒分が欠損する可能性がある
2. BLE 温度欠損: SoftwareSerial 経由で温度値の欠損が発生。対策として FreeRTOS タスク分離 / HardwareSerial への移行 / リングバッファ導入を検討

## 次ステップ (改善案)

- 停止操作で `sync/close` を明示実行する安全停止フローの追加
- 速度(`Spd_PULSE`)を 0.1 km/h 単位で記録・送信 (必要ならスケール変更)
- GNSS 日付処理の簡素化 (標準ライブラリ活用)
- 証明書有効期限チェック機能

## ライセンス

MIT License (LICENSE 参照)

## 確認事項 / 追加要望の受付

追加の仕様変更があれば Issue に追記しREADME を随時更新する。
