# Chibi-T Furoshiki Logger (M5Stack Core2 / Basic)

M5Stack Core2 / Basic 上で動作するエコラン競技車両向けロガー兼リアルタイム表示システムです。  
以下を統合しています:

- [ECU](https://github.com/todateman/UNOR4_Chibi-T_EFI) からの走行データ取得 (Serial1)
- GNSS 位置・時刻取得 (Serial2, TinyGPS++)
- 高度推定 (BME280を用いた気圧→高度換算)
- インターネット接続時の国土地理院標高API利用 (HTTP優先、失敗時はHTTPS/GNSS高度へフォールバック)
- BLE 経由の[エンジン温度](https://github.com/todateman/Chibi-T_Furoshiki_Heater)・[1次側/2次側空気圧・燃圧](https://github.com/todateman/Chibi-T_Furoshiki_AutoAirAdjust)受信 ([nF52840 BLE中継機](https://github.com/todateman/nRF52840_BLE_Central)とPort A経由のI2C通信)
- BME280を使用した気温・湿度測定
- SD カードへの CSV ロギング (SdFat)
- Wi-Fi (任意) + MQTT (AWS IoT Core) / Ambient 送信
- LCD (M5Unified + LovyanGFX) への複数モード表示

## 主な機能概要

| 機能 | 内容 |
| ---- | ---- |
| 周回/距離管理 | 位置範囲からサーキット判定 (鈴鹿/茂木/その他) 、コントロールライン通過検知による周回数カウント、走行時間計算 |
| 表示モード切替 | Bボタン=`モード 0` 速度/残周回/走行時間(デフォルト)。走行支援が有効な場所では走行支援画面<BR>Aボタン=`モード 1` 回転数/噴射時間/進角(セッティング向け)<BR>Cボタン=`モード 2` 走行ルート標高グラフ + 現在地標高プロット |
| 走行支援 | 加速開始位置・加速停止速度・通過タイムの遅れを画面の色と文字で合図する (SDに設定ファイル `motegi_strategy.json` があるときだけ有効。削除すれば無効)。詳細は「走行支援」を参照 |
| ウェイポイント標高グラフ | `sd`ディレクトリのウェイポイントCSV (`suzuka_waypoint.csv` / `motegi_waypoint.csv` / `toyota_waypoint.csv`) を読み込み、横軸=ウェイポイントID、縦軸=標高で表示。現在地の緯度経度を最も近いIDへ割り当てて標高を重ねて表示 |
| ログ保存 | `/LOG/LOGxxxx.CSV` (UTF-8 BOM付き, ヘッダ日本語) |
| 高度推定 | BME280 気圧から高度推定 (標準大気式) |
| 標高オフセット補正 | 国土地理院APIから標高を1回取得し、BME280高度との差分でオフセット補正 |
| MQTT 送信 | JSON ペイロードを `mqtt_topic` へ (証明書による TLS) |
| Ambient 送信 | 10 フィールド + 位置情報文字列 |
| セーフ処理 | GNSS 異常値・未固定除外<BR>周回カウントの多重検出防止 (デバウンス) / GPSロスト再測位時の誤検出防止 (テレポートガード)<BR>BLEタイムアウト時温度リセット<BR>ECU無信号時フェールセーフ |

## ハードウェア / 接続

- 基板: M5Stack Basic (Gray) / M5Stack Core2 (ESP32, PSRAM 使用)
- SD: SPI (GPIO4 / SHARED_SPI 設定)
- ECU: Serial1 115200 bps、RXピンはボード依存  
  (M5Stack Core2: RX=2, TX=0 / M5Stack Basic: RX=15, TX=0)  
  ※M5Stack Basicの起動不良対策としてGPIO12を使用しないため分岐 (コード参照)
  - 受信フォーマット (10Hz): `rpm,inj(ms),ign_ca,inj_end_ca,speed,distance,fuel(ml),km/L,worktime*XX` (`worktime` は0.1秒単位の小数秒。例 `267.3`。整数秒でも受理)  
    `XX` は先頭から `*` 直前までのXOR (16進2桁)。不一致・項目不足の行は破棄する。EFI側と同時に更新すること (旧形式とは非互換)  
    RXバッファは1024Bに拡大 (MQTT/HTTP中の取りこぼし対策)
- GNSS: Serial2 115200 bps (RX=13, TX=14)
- BME280(気圧/気温/湿度センサ): I2C 0x76、SDA/SCLは `M5.Ex_I2C.getSDA()/getSCL()` でボード既定値を自動取得
- BLE 中継機 [nRF52840_BLE_Central](https://github.com/todateman/nRF52840_BLE_Central) (XIAO nRF52840): Port A 経由の I2C で接続（XIAO nRF52840側がI2Cスレーブ, addr=`0x08`）
  - M5Stack Core2: SDA=32, SCL=33 (専用I2Cバス`Wire1`を使用。BME280用バスとはピンが異なるため)
  - M5Stack Basic: SDA=21, SCL=22 (BME280用`Wire`バスと共用。Port Aのピンが同一のため)
  - 200ms間隔で以下4コマンドを順に送信し、それぞれ32byte固定フレーム（`[0]`=データ長, `[1..]`=文字列データ）でデータを取得。いずれも2秒以上有効データを取得できない場合は該当値を`0.0`にリセットする

    | コマンド | データ | 単位 | 有効範囲(範囲外は破棄) |
    | ---- | ---- | ---- | ---- |
    | `CMD_ENGINE_TEMP` (`0x01`) | エンジン温度 ([Chibi-T_Furoshiki_Heater](https://github.com/todateman/Chibi-T_Furoshiki_Heater)経由) | ℃ | 10.0 〜 150.0 |
    | `CMD_PRI_PRE` (`0x02`) | 1次側空気圧 ([Chibi-T_Furoshiki_AutoAirAdjust](https://github.com/todateman/Chibi-T_Furoshiki_AutoAirAdjust)経由) | MPa | 0.01 〜 0.72 |
    | `CMD_SEC_PRE` (`0x03`) | 2次側空気圧 (同上) | MPa | 0.01 〜 0.72 |
    | `CMD_FUEL_PRE` (`0x04`) | 燃圧 (同上) | MPa | -0.02 〜 1.05 |

- ボタン: A/B/C でモード選択 + 起動時設定。B長押し(1秒)で走行支援の加速パターンを切替

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

その他の環境:

| 環境 | 用途 | コマンド |
| ---- | ---- | ---- |
| `native` | 走行支援の判定ロジック (`include/StrategyAssist.h`) のテストをPC上で実行 | `pio test -e native` |
| `m5stack-basic-sim` | 走行支援画面の机上確認。もてぎのウェイポイント上を8倍速で仮想走行する (`STRATEGY_SIM=8`)。SDに `motegi_waypoint.csv` と `motegi_strategy.json` が必要。SDログ・MQTT・Ambientは止まる。**本番には書き込まない** (状態行に `SIM` と出る) | `pio run -e m5stack-basic-sim -t upload` |

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

1. モード 0: 速度, 残り周回 (最後は G/FINISH), 走行時間進行バー。走行支援が有効な場所では走行支援画面に替わる
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

## 走行支援

もてぎ全国大会 2026 の走行戦略 ([ECU リポジトリ](https://github.com/todateman/UNOR4_Chibi-T_EFI) の `document/motegi_strategy_2026.md`) の判断を画面に出す。  
**SDカードにその場所の設定ファイル (もてぎは `motegi_strategy.json`) があるときだけ有効**になり、モード 0 がこの画面に替わる。  
走行パターンの値はファームには持っていない。**走行支援が不要なときは、設定ファイルを microSD から削除する** (従来のモード 0 に戻る)。

### 画面例

実機の写真ではなく、`src/main.cpp` の描画処理 (`drawStrategyMode()`) を PC 上で動かして出力した画像 (2 倍に拡大)。数値は例。

| 走り出すまで | 惰行中 | 加速開始位置の手前 |
| :----: | :----: | :----: |
| <img src="docs/strategy_screen_idle.png" width="260" alt="発進前の画面。スタート待ちと表示"> | <img src="docs/strategy_screen_coast.png" width="260" alt="惰行中の画面。加速まで400mと表示"> | <img src="docs/strategy_screen_approach.png" width="260" alt="加速開始位置の手前の画面。加速まで60mと黄色で表示"> |
| 発進前。右下に加速位置、右上に加速停止速度 | 2 周通過が 18 秒遅れ → 加速停止速度 33.5 (マゼンタ) | 100 m 手前から黄色 |

| 加速開始 | 加速中 | 加速停止速度に到達 |
| :----: | :----: | :----: |
| <img src="docs/strategy_screen_burn_now.png" width="260" alt="加速開始の画面。全面が緑で加速!と表示"> | <img src="docs/strategy_screen_burning.png" width="260" alt="加速中の画面。35.5で停止と表示"> | <img src="docs/strategy_screen_cut_now.png" width="260" alt="加速停止速度に到達した画面。全面が赤で停止!と表示"> |
| 全面が緑 | 38 秒遅れ → 加速停止速度 35.5 まで引っ張る | 全面が赤 |

| 最低限界速度 | 最終周 | 燃圧低下 |
| :----: | :----: | :----: |
| <img src="docs/strategy_screen_guard.png" width="260" alt="速度が16km/hまで落ちた画面。全面が緑で加速!速度低下と表示"> | <img src="docs/strategy_screen_coast_home.png" width="260" alt="最終周の画面。惰行でゴールと表示"> | <img src="docs/strategy_screen_fuel_low.png" width="260" alt="燃圧低下の画面。下端にオレンジの警告"> |
| 16 km/h まで落ちた | 3 つ目の加速位置は ▽ (使わない) | 35 秒早い → 加速停止速度 32.0 (水色) |

| 予備パターン |
| :----: |
| <img src="docs/strategy_screen_fallback.png" width="260" alt="予備パターンの画面。状態行に4回予備と表示"> |
| B 長押しで切替。状態行に `4回予備`、加速位置が 4 つ |

### 画面の見方

| 画面 | 意味 | ドライバーの操作 |
| ---- | ---- | ---- |
| **全面が緑**「加速!」 | 加速開始位置に来た | エンジンを始動して全開 |
| **全面が緑**「加速! 速度低下」 | 最低限界速度 (16 km/h) まで落ちた | 場所によらず加速 |
| 「32.5で停止」 | 加速中 | 右上の加速停止速度まで引っ張る |
| **全面が赤**「停止!」 | 加速停止速度に達した | エンジンを止める |
| 「加速まで 185m」 | 惰行中。100 m 手前から黄色になる | 準備 |
| 「惰行でゴール」 | 最終周の加速が終わった | そのまま惰行 (16 km/h まで落ちたら加速) |
| 「位置なし」 | GNSSが測位できていない | 目視の目印と最低限界速度の合図で走る |
| 下端にオレンジ「燃圧低下」 | 直近の加速で燃圧がしきい値を下回った | B長押しで予備パターンへ |

- 最も左上の表示は「場所 / 加速回数」
- 最も右上に「SDカードの有無 / GNSSの受信状態 / MQTTの送信有効・無効」
- 左上の7 セグメント風の数字が現在の速度（km/h）
- 右上の数字が**今の周の加速停止速度**。スタートライン通過のたびに走行時間を目標と比べ、遅れ 8 / 20 / 35 秒超で +1 / +2 / +3 km/h、30 秒超早ければ −0.5 km/h する (毎周計算し直す)。上げているあいだはマゼンタ、下げているあいだは水色
- 速度バーの黄色の線が最低限界速度、赤い線が加速停止速度
- 中央にドライバーへの指示を表示
- 位置バーは1周あたりのコース上の現在地を示す。
  - 位置バーの ▼ が加速開始位置、縦線が現在地。最終周に使わない位置は ▽
- 左下に現在の周回数を表示
- 中央下に走行開始からの経過時間を表示。遅れはマゼンタ
- 右下は直近の通過タイムと目標との差 (`+18s` は 18 秒遅れ)。目標より 2 分以上早い値は時間基準か周回数のずれとみなして使わない
- 下端の帯は規定時間までの残り時間 (従来のモード 0 の走行時間バーと同じ)。位置バーの現在地と同じ向きにそろえ、時間が進むと左から右へ短くなる。色は走行時間の数字と同じ (直近の通過が遅れていればマゼンタ)
  - 同じ帯に全周回の進み具合を重ねてある。縦の細い線が周の区切り、水色の縦線が走行距離の現在地 (ECU の走行距離 ÷ 規定距離)。**現在地が帯の左端より右にあれば、距離の進みが時間の進みを上回っている** (平均 25 km/h より速い)。最終周を惰行で終える走り方では、6 周通過の時点で現在地が帯の左端より帯の幅の 5% ほど (約 2 mm) 先行しているのが目標どおり。細かい遅れは右下の秒数で見る
- **B長押し (1秒)** で通常パターン ⇄ 予備パターンを切り替える (状態行に `予備` と出る)
- ブザーは M5Stack Basic のみ (加速=短く2回、停止=長く1回、続いている間は2秒ごと)。Core2 は内蔵スピーカが ECU 用 UART と同じピン (GPIO0/2) を使うため鳴らさない

### 判定の仕組み

- 周内位置: 現在地を最近傍ウェイポイントの前後の区間へ射影し、CSVの距離列から求める。公式の 1 周の距離 (`lap_m`) に合わせて伸縮し、測位からの経過時間ぶんを車速で進める。ウェイポイントを結んだ線から 60 m 以上離れている、または 3 秒以上測位が無い場合は「位置なし」
- 加速中の判定: ECUの噴射時間が 0 より大きい (ECUはエンジン停止中 0 を送る)
- 加速開始位置を通過した時点で速度が「加速停止速度 − 2 km/h」を超えていれば、その位置では合図を出さない
- 「加速!」は噴射が 1.5 秒続くまで出し続ける (再始動に失敗したら出たままになる)。200 m 過ぎるか次の位置の 100 m 手前で消える
- 最終周は `final_lap_marks` 個目までの位置だけ有効。最終周に入る通過タイムが `final_extra_if_split_over_s` を超えていれば全位置を有効にする
- 判定ロジックは `include/StrategyAssist.h` (Arduino非依存)。ルールは EFI 側の `tools/strategy_sim.py` と同じ

### 設定ファイル

SDカードのルートに置く (`sd/motegi_strategy.json`)。場所ごとに `motegi_strategy.json` / `suzuka_strategy.json` / `toyota_strategy.json`。

| SDカードの状態 | 動作 |
| ---- | ---- |
| 設定ファイルがあり、内容が正しい | 走行支援が有効。モード 0 が走行支援画面になる |
| 設定ファイルが無い | 走行支援は無効。従来のモード 0 |
| 設定ファイルはあるが内容に異常がある | 走行支援は無効。従来のモード 0 の左上に赤字で `支援設定異常` と出る |

- 読み込むのは場所を判定したときの 1 回だけ。ファイルを入れ替えたら再起動する
- 場所の判定には GNSS の測位が要る。起動後に一度も測位できていないあいだは、設定ファイルがあっても従来のモード 0 のまま
- **練習走行後に値を変えるときは、このファイルを書き換えて SD に入れ直すだけでよい** (再ビルド不要)。起動後、発進前の画面で右下の加速位置と右上の加速停止速度を確認する

| 項目 | 内容 | 省いた場合 | `sd/motegi_strategy.json` の値 |
| ---- | ---- | ---- | ---- |
| `lap_m` | 1 周の距離 [m] | **必須** | 2341.38 |
| `v_off` | 加速停止速度 [km/h] | **必須** | 32.5 |
| `marks_m` | 加速開始位置 [m] (スタートラインから、昇順、最大 6) | **必須** | 400, 800, 1400 |
| `final_lap_marks` | 最終周に有効な位置の数 (先頭から) | 全位置が有効 | 2 |
| `final_extra_if_split_over_s` | 最終周に入る通過タイムがこれを超えたら全位置を有効にする [s] | この判定をしない | 1965 |
| `guard_kmh` | 最低限界速度 [km/h] | 最低限界速度の合図を出さない | 16.0 |
| `splits_s` | n 周通過時の目標走行時間 [s] (最大 12) | 加速停止速度を補正しない | 309, 630, 951, 1272, 1593, 1914 |
| `fallback` | 予備パターン (`v_off` / `marks_m` / `final_lap_marks`) | 予備パターンなし (B長押しは無効) | 30.0 / 200, 600, 1000, 1400 / 3 |
| `fuel_warn_mpa` | 加速中の燃圧がこれを下回ったら警告 [MPa] | 燃圧の警告を出さない | 0.30 |
| `beep` | ブザーを鳴らすか | 鳴らす | true |

必須項目が無い、JSON として読めない、位置が昇順でない、位置が 1 周の距離を超えている、加速停止速度が最低限界速度以下、などの場合は設定全体を採用しない (`支援設定異常`)。

## 周回カウント仕様

- 実装ファイル: `src/main.cpp` (`updateGNSS()` / `updateLapCountByControlLineCrossing()`)
- 各サーキット (鈴鹿/茂木/豊田SENTAN) ごとに、コントロールラインを2点の緯度経度 (外側/内側) で定義済み
- GNSSの「前回測位位置 → 今回測位位置」を結ぶ線分が、サーキット判定結果 (`Loc`) に対応するコントロールライン (2点の線分) と交差したかを、外積 (クロス積) を用いた線分交差判定 (orientation/CCW法) で毎測位ごとに判定
- 交差を検知するたびに `Lapcount` を1加算 (旧実装の「走行距離 ÷ 1周あたり距離」による推定方式から変更)
- 誤検出防止:
  - デバウンス: 直前のカウントから `LAP_CROSS_DEBOUNCE_MS` (既定 10秒) 未満の再交差は無視 (スタート/フィニッシュ付近での低速走行・停止時のGPSジッタによる多重カウント防止)
  - テレポートガード: 前回位置からの概算移動距離が `LAP_CROSS_TELEPORT_GUARD_M` (既定 100m) を超える場合は判定をスキップ (GNSS受信断からの復帰など、位置が飛んだ際の誤検出防止)
  - GNSS未固定 (`gps.location.isValid()`が偽) の測位は前回位置の更新・交差判定の対象から除外
  - 走行距離ガード (鈴鹿・もてぎ、ECU受信中のみ): 前回カウントからの ECU 走行距離が 1 周の半分に満たない交差は数えない。グリッドがラインの手前にあっても発進直後の通過を 1 周と数えない。ECU の走行距離が途中で 0 に戻った場合は、次のカウントまでこの判定を行わない
  - 発進時のリセット: ECU の走行時間が 0 から動き出した時点で周回数を 0 に戻す (発進前にラインをまたいで数えた分を走行に持ち込まない)
- カウントした時点の走行時間を通過タイムとして走行支援へ渡す
- 残り周回は 0 で止まる (周回数が規定を超えても 255 にならない)

## ログファイル仕様 (SD)

ヘッダ: `記録日時,速度(km/h),ラップ数(周目),走行時間(秒),回転数(rpm),燃料噴射時間(ms),点火進角角度(CA),燃料噴射終了角度(CA),走行距離積算(m),積算燃料消費量(ml),燃費(km/l),lat(緯度),lng(経度),標高(m),場所,温度(C),気圧(kPa),気温(C),湿度(%),1次空気圧(MPa),2次空気圧(MPa),燃圧(MPa)` (22列)

- 記録日時: GNSS/NTP同期時刻を基準にmillis()で連続算出 + JST補正 (`YYYY/M/D hh:mm:ss.cc`)。SDログ行は10Hzでも重複しない (厳密単調増加)
- `alt`: BME280高度 + 国土地理院APIで確定したオフセット
- `loc`: サーキット判定結果 (`su` / `mo` / `to`)
- 走行時間: ECU送信の積算秒 (0.1秒単位、小数1桁。例 `267.3`)。LCD表示・MQTT/Ambient送信は整数秒
- 燃料噴射時間(ms) / 点火進角角度(CA) / 燃料噴射終了角度(CA): ECU送信値。ECU無信号時は0 (燃料噴射時間は小数1桁)
- 燃費: コード中 `dispergas` (km/L 指定の閾値 2000 スケールバー)
- タイムスタンプは `logFile.timestamp()` によりファイル更新時にも設定
- ログ周期は 10Hz (`SD_LOG_INTERVAL=100ms`)
- SD書き込みはファイルを開きっぱなしで運用し、複数行をバッファして定期 `sync()` で書き込み確定  
  （電源切では`sync()`実行済みのログを残す）
- SdFatの `preAllocate()` は使用しない。`O_APPEND` と併用すると予約領域ぶんCSV本体が後方へずれ、先頭にバイナリ/ゼロ領域が残るため

### 停止中の行の削除 (分析用)

走行前の待機中や走行後の停車中の行は、`tools/remove_stopped_rows.py` で取り除ける。PC上で実行する (Python 3.10 以上、追加ライブラリ不要)。

```sh
python3 tools/remove_stopped_rows.py LOG0651.CSV              # 同じ場所に LOG0651_trimmed.CSV を出力
python3 tools/remove_stopped_rows.py LOG0650.CSV LOG0651.CSV  # 複数まとめて処理
python3 tools/remove_stopped_rows.py LOG0651.CSV -o out.csv   # 出力先を指定 (入力が1件のときのみ)
```

`速度(km/h)` と `回転数(rpm)` の両方が 0 の行を「停止行」とし、停止行が続く区間ごとに次のように扱う。

| 行 | 扱い |
| --- | --- |
| 停止行ではない行 (惰行中、停車中のアイドリングなど) | 残す |
| 停止区間の最初の1行 (車体停止の瞬間) | 残す |
| 停止区間の最後の1行 (エンジン始動の瞬間) | 残す |
| 停止区間の途中の行 | 削除 |

- 元ファイルは上書きしない。出力先が入力と同じ場合はエラーで止まる
- ファイル先頭から始まる停止区間は最後の1行だけ、末尾まで続く停止区間は最初の1行だけが残る
- 残した行の中身は書き換えない (BOM・改行コード・数値の桁数は元のまま)。`走行時間(秒)` などの値も詰め直さない
- 列数が足りない行 (電源断で途切れた最終行など) は削除せずに残し、件数を表示する

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
  "Spd_PULSE": 42.5,
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
| CSVログの先頭が文字化け/バイナリになる | 旧実装で `preAllocate()` されたログの可能性。CSV本体が予約領域後に残っている場合は、該当オフセット以降を切り出して復旧する |
| MQTT connect失敗 | 証明書有効性/時刻同期 (GNSSで JST 変換) <BR> ポリシー権限確認 |
| MQTT reconnect failed, state: -2 かつ `X509 - Allocation of memory failed` | TLS証明書検証時のヒープ不足。表示・バッファ確保量を下げて空きメモリを増やす (例: `MAX_WAYPOINTS` 削減, `lcd_s.setColorDepth(4)` など) <BR> 切り分け時は `MQTT_DIAGNOSTIC_LOG` を `1` にして `heap/minHeap/maxAlloc` を確認し、接続直前の `maxAlloc` を十分確保する |
| Ambient failure | Wi-Fi RSSI / userKey / devKey/channelId 取得失敗再試行 |
| 温度/PRI/SEC/FUEL が 0.0 固定 | BLE中継機(XIAO nRF52840)とのI2C通信失敗、またはBLE中継機が対応するBLEペリフェラル(Heater/AutoAirAdjust)からNotifyを受信できていない <BR> (該当値ごとに有効データ未取得が2秒以上継続でリセット。BLE中継機は切断時や3秒以上更新のないデータを長さ0で返すため、ペリフェラル側が止まった場合もここに該当する) |
| 温度だけが同じ値のまま変化しない (例: LOG0645 の 79.50 固定) | BLE中継機の旧ファームウェアの不具合。AutoAirAdjust 接続中に Heater だけ切断されると再接続されず、切断前の値を返し続けていた <BR> [nRF52840_BLE_Central](https://github.com/todateman/nRF52840_BLE_Central) を最新版(2026-10-03 以降)へ更新する |
| `[GSI] HTTPS GET failed: -1` が出る | 現在は `HTTP優先` 運用のため、HTTP成功時は実害なし。`[GSI] elevation=...` が継続していれば正常 |

### MQTT 診断ログ運用

- 本番運用では `src/main.cpp` の `MQTT_DIAGNOSTIC_LOG` を `0` のまま使用 (既定)
- AWS IoT接続トラブルの切り分け時のみ `1` に変更して再ビルド
- 診断で確認する主なログ: `MQTT TLS lastError`, `MQTT DNS`, `MQTT diag TCP/TLS(insecure)`, `heap/minHeap/maxAlloc`

### BLE I2C 診断ログ運用

- 本番運用では `src/main.cpp` の `BLE_I2C_DIAGNOSTIC_LOG` を `0` のまま使用 (既定)
- BLE中継機との通信切り分け時のみ `1` に変更して再ビルド
- `1` にすると `[BLE I2C]` で始まる受信/範囲外/要求失敗ログをSerialへ出力するため、Serial CSVを保存する運用では混入に注意する

## 既知の課題 (改善予定)

1. 電源断耐性: 開きっぱなし運用のため、電源断時は最後の `sync()` 以降の数秒分が欠損する可能性がある

## 次ステップ (改善案)

- 停止操作で `sync/close` を明示実行する安全停止フローの追加
- GNSS 日付処理の簡素化 (標準ライブラリ活用)
- 証明書有効期限チェック機能

## ライセンス

MIT License (LICENSE 参照)

## 確認事項 / 追加要望の受付

追加の仕様変更があれば Issue に追記しREADME を随時更新する。
