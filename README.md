# Chibi-T Furoshiki Logger (M5Stack Core2 / Basic)

M5Stack Core2 / Basic 上で動作するエコラン競技車両向けロガー兼リアルタイム表示システムです。  
以下を統合しています:

- ECU からの走行データ取得 (Serial1)
- GNSS 位置・時刻取得 (Serial2, TinyGPS++)
- 高度推定 (BME280を用いた気圧→高度換算)
- インターネット接続時の国土地理院標高API利用 (HTTP優先、失敗時はHTTPS/GNSS高度へフォールバック)
- BLE 経由のエンジン温度・1次側/2次側空気圧・燃圧受信 (M5NanoC6 BLE中継機とPort A経由のI2C通信)
- BME280を使用した気温・湿度測定
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
| 高度推定 | BME280 気圧から高度推定 (標準大気式) |
| 標高オフセット補正 | 国土地理院APIから標高を1回取得し、BME280高度との差分でオフセット補正 |
| MQTT 送信 | JSON ペイロードを `mqtt_topic` へ (証明書による TLS) |
| Ambient 送信 | 10 フィールド + 位置情報文字列 |
| セーフ処理 | GNSS 異常値除外<BR>BLEタイムアウト時温度リセット<BR>ECU無信号時フェールセーフ |

## ハードウェア / 接続

- 基板: M5Stack Core2 (ESP32, PSRAM 使用) / M5Stack Basic (Gray)
- SD: SPI (GPIO4 / SHARED_SPI 設定)
- ECU: Serial1 115200 bps、RXピンはボード依存  
  (M5Stack Core2: RX=2, TX=0 / M5Stack Basic: RX=15, TX=0)  
  ※M5Stack Basicの起動不良対策としてGPIO12を使用しないため分岐 (コード参照)
- GNSS: Serial2 115200 bps (RX=13, TX=14)
- BME280(気圧/気温/湿度センサ): I2C 0x76、SDA/SCLは `M5.Ex_I2C.getSDA()/getSCL()` でボード既定値を自動取得
- BLE 中継機 [M5NanoC6_BLE_Central](https://github.com/todateman/M5NanoC6_BLE_Central) (M5NanoC6): Port A 経由の I2C で接続（NanoC6側がI2Cスレーブ, addr=`0x08`）
  - M5Stack Core2: SDA=32, SCL=33 (専用I2Cバス`Wire1`を使用。BME280用バスとはピンが異なるため)
  - M5Stack Basic: SDA=21, SCL=22 (BME280用`Wire`バスと共用。Port Aのピンが同一のため)
  - 200ms間隔で以下4コマンドを順に送信し、それぞれ32byte固定フレーム（`[0]`=データ長, `[1..]`=文字列データ）でデータを取得。いずれも2秒以上有効データを取得できない場合は該当値を`0.0`にリセットする

    | コマンド | データ | 単位 | 有効範囲(範囲外は破棄) |
    | ---- | ---- | ---- | ---- |
    | `CMD_ENGINE_TEMP` (`0x01`) | エンジン温度 ([Chibi-T_Furoshiki_Heater](https://github.com/todateman/Chibi-T_Furoshiki_Heater)経由) | ℃ | 10.0 〜 150.0 |
    | `CMD_PRI_PRE` (`0x02`) | 1次側空気圧 ([Chibi-T_Furoshiki_AutoAirAdjust](https://github.com/todateman/Chibi-T_Furoshiki_AutoAirAdjust)経由) | MPa | 0.01 〜 0.72 |
    | `CMD_SEC_PRE` (`0x03`) | 2次側空気圧 (同上) | MPa | 0.01 〜 0.72 |
    | `CMD_FUEL_PRE` (`0x04`) | 燃圧 (同上) | MPa | -0.02 〜 1.05 |

- ボタン: A/B/C でモード選択 + 起動時設定

## ソフトウェア依存ライブラリ (platformio.ini より)

- [TinyGPSPlus](https://github.com/mikalhart/TinyGPSPlus)
- [Ambient ESP32 ESP8266 lib](https://github.com/AmbientDataInc/Ambient_ESP8266_lib)
- [M5Unified](https://github.com/m5stack/M5Unified)
- [TimeLib](https://github.com/derickr/timelib)
- [PubSubClient](https://github.com/knolleary/pubsubclient)
- [ArduinoJson v7](https://github.com/bblanchon/ArduinoJson)
- [SdFat](https://github.com/greiman/SdFat)
- [WiFiManager](https://github.com/tzapu/WiFiManager) (同梱ライブラリ `lib/WiFiManager`)
- [Adafruit BME280 Library](https://github.com/adafruit/Adafruit_BME280_Library)
- [Adafruit BMP280 Library](https://github.com/adafruit/Adafruit_BMP280_Library)
- [Adafruit AHTX0](https://github.com/adafruit/Adafruit_AHTX0)
- [Adafruit Unified Sensor](https://github.com/adafruit/Adafruit_Sensor)

## ビルド & 実行 (PlatformIO)

1. VS Code + PlatformIO をインストール
2. 本リポジトリを開く
3. board: `m5stack-core2` (既定) または `m5stack-basic` を指定  
   (`platformio.ini` の `default_envs` で切替、`pio run -e <env>` でも指定可)
4. Upload (書き込み) / Monitor (115200bps) で動作確認
5. 初回起動時 SD カードが挿入されていることを確認

### デバッグ

- シリアル出力: CSV形式 `lat, lon, alt, loc, Spd_GPS, rpm, Spd_PULSE, distance, gasml, dispergas, worktime, EngTemp, Pressure, Temp, Humidity, PRI, SEC, FUEL, datetime`
- 例外デコード: `monitor_filters = esp32_exception_decoder`

## 高度推定 / 国土地理院API補正

- 高度は GNSS ではなく BME280 の気圧から算出
- 海面気圧は初期値 `101.325kPa` を使用
- Wi-Fi接続済みかつGNSS座標が有効なときのみ、国土地理院API (`getelevation.php`) で標高取得を低頻度で試行
- 標高取得成功時、その時点のBME280生高度との差分をオフセットとして1回だけ固定し、以後は再取得しない
- 以後の表示/ログ高度は `BME280生高度 + オフセット` で算出
- API取得は HTTP を優先し、失敗時は HTTPS フォールバック

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

<p align="center" width="100%">
<video src="https://github.com/user-attachments/assets/a7fdfc0b-4c62-4426-a9e1-d857ddd0c710" width="100%" controls></video>
</p>

- 水色線: ウェイポイントファイルの標高プロファイル
- 黄色点: 最近傍ウェイポイントIDの標高
- 赤色点: 現在地標高
- 下部テキスト: `WP:現在ID/総ポイント数 route:ルート標高 now:現在地標高`

## ログファイル仕様 (SD)

ヘッダ: `記録日時,速度(km/h),ラップ数,走行時間,回転数,走行距離,積算燃料,燃費,lat,lon,alt,loc,温度,気圧(kPa),気温(C),湿度(%),1次空気圧(MPa),2次空気圧(MPa),燃圧(MPa)`

- 記録日時: GNSS + JST補正 (`YYYY/M/D hh:mm:ss.cc`)
- `alt`: BME280高度 + 国土地理院APIで確定したオフセット
- `loc`: サーキット判定結果 (`su` / `mo` / `to`)
- 走行時間: ECU送信の積算秒
- 燃費: コード中 `dispergas` (km/L 指定の閾値 2000 スケールバー)
- タイムスタンプは `logFile.timestamp()` によりファイル更新時にも設定
- SD書き込みはファイルを開きっぱなしで運用し、複数行をバッファして定期 `sync()` で書き込み確定  
  （電源切では`sync()`実行済みのログを残す）

## 高度推定仕様

- 実装ファイル: `src/main.cpp`
- センサー構成:
  - 気圧: BME280
  - 気温/湿度: BME280
- 計算式: 標準大気式 `44330 * (1 - (P / P0)^0.1903)`
  - `P`: 実測気圧(kPa)
  - `P0`: 海面気圧初期値 `101.325kPa`
- オフセット補正:
  - Wi-Fi接続・位置有効時に国土地理院APIを試行
  - APIはHTTP優先、失敗時のみHTTPSフォールバック
  - 成功時に `GSI標高 - 生高度` をオフセットとして固定し、以後は再取得しない
- 補正失敗時の動作:
  - 高度は生高度ベースで継続
  - GNSS高度は高度補正計算には使用しない

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
  "alt": 123.4,
  "loc": "su",
  "temp": 92.5,
  "pri": 0.55,
  "sec": 0.52,
  "fuel": 0.35
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
| MQTT reconnect failed, state: -2 かつ `X509 - Allocation of memory failed` | TLS証明書検証時のヒープ不足。表示・バッファ確保量を下げて空きメモリを増やす (例: `MAX_WAYPOINTS` 削減, `lcd_s.setColorDepth(4)` など) <BR> 切り分け時は `MQTT_DIAGNOSTIC_LOG` を `1` にして `heap/minHeap/maxAlloc` を確認し、接続直前の `maxAlloc` を十分確保する |
| Ambient failure | Wi-Fi RSSI / userKey / devKey/channelId 取得失敗再試行 |
| 温度/PRI/SEC/FUEL が 0.0 固定 | NanoC6とのI2C通信失敗、またはNanoC6が対応するBLEペリフェラル(Heater/AutoAirAdjust)からNotifyをまだ受信していない <BR> (該当値ごとに有効データ未取得が2秒以上継続でリセット) |
| `[GSI] HTTPS GET failed: -1` が出る | 現在は `HTTP優先` 運用のため、HTTP成功時は実害なし。`[GSI] elevation=...` が継続していれば正常 |

### MQTT 診断ログ運用

- 本番運用では `src/main.cpp` の `MQTT_DIAGNOSTIC_LOG` を `0` のまま使用 (既定)
- AWS IoT接続トラブルの切り分け時のみ `1` に変更して再ビルド
- 診断で確認する主なログ: `MQTT TLS lastError`, `MQTT DNS`, `MQTT diag TCP/TLS(insecure)`, `heap/minHeap/maxAlloc`

## 既知の課題 (改善予定)

1. 電源断耐性: 開きっぱなし運用のため、電源断時は最後の `sync()` 以降の数秒分が欠損する可能性がある
2. BLE中継データの鮮度検知: NanoC6とのI2C通信自体が正常でも、NanoC6内部でBLE Notifyが途絶えた場合は検知できず、古い温度値を返し続ける可能性がある（NanoC6側にNotifyタイムアウト検知機能が無いため。詳細は [M5NanoC6_BLE_Central](https://github.com/todateman/M5NanoC6_BLE_Central) の今後の改善案を参照）

## 次ステップ (改善案)

- 停止操作で `sync/close` を明示実行する安全停止フローの追加
- 速度(`Spd_PULSE`)を 0.1 km/h 単位で記録・送信 (必要ならスケール変更)
- GNSS 日付処理の簡素化 (標準ライブラリ活用)
- 証明書有効期限チェック機能

## ライセンス

MIT License (LICENSE 参照)

## 確認事項 / 追加要望の受付

追加の仕様変更があれば Issue に追記しREADME を随時更新する。
