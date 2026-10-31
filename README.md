# Chibi-T Furoshiki Logger (M5Stack Core2 / Basic)

M5Stack Core2 / Basic 上で動作するエコラン競技車両向けロガー兼リアルタイム表示システムです。  
以下を統合しています:

- [ECU](https://github.com/todateman/UNOR4_Chibi-T_EFI) からの走行データ取得 (Serial1)
- GNSS 位置・時刻取得 (Serial2, TinyGPS++)
- 高度推定 (Unit ENV-III または BME280 を用いた気圧→高度換算)
- インターネット接続時の国土地理院標高API利用 (HTTP優先、失敗時はHTTPS/GNSS高度へフォールバック)
- BLE 経由の[エンジン温度](https://github.com/todateman/Chibi-T_Furoshiki_Heater)・[1次側/2次側空気圧・燃圧](https://github.com/todateman/Chibi-T_Furoshiki_AutoAirAdjust)受信 ([nF52840 BLE中継機](https://github.com/todateman/nRF52840_BLE_Central)とPort A経由のI2C通信)
- [Unit ENV-III](https://docs.m5stack.com/en/unit/Unit_ENV-III) (見つからなければ BME280) を使用した気温・湿度測定
- SD カードへの CSV ロギング (SdFat)
- Wi-Fi (任意) + MQTT (AWS IoT Core) 送信
- LCD (M5Unified + LovyanGFX) への複数モード表示

## 主な機能概要

| 機能 | 内容 |
| ---- | ---- |
| 周回/距離管理 | 位置範囲からサーキット判定 (鈴鹿/茂木/その他) 、コントロールライン通過検知による周回数カウント、走行時間計算 |
| 表示モード切替 | Bボタン=`モード 0` 速度/残周回/走行時間(デフォルト)。走行支援が有効な場所では走行支援画面<BR>Aボタン=`モード 1` 回転数/噴射時間/進角(セッティング向け)<BR>Cボタン=`モード 2` コース図上の現在地表示 (加速開始位置・加速停止速度つき) |
| 走行支援 | 加速開始位置・加速停止速度・通過タイムの遅れを画面の色と文字で合図する (SDに設定ファイル `motegi_strategy.json` / `suzuka_strategy.json` があるときだけ有効。削除すれば無効)。詳細は「走行支援」を参照 |
| 現在地表示 (コース図) | `sd`ディレクトリのウェイポイントCSV (`suzuka_waypoint.csv` / `motegi_waypoint.csv` / `toyota_waypoint.csv`) を読み込み、北を上にしたコース図を描く。GNSSの現在地と、走行支援の加速開始位置・加速停止速度を重ねて表示。詳細は「モード2(現在地表示)」を参照 |
| ログ保存 | `/LOG/LOGxxxx.CSV` (UTF-8 BOM付き, ヘッダ日本語) |
| 高度推定 | Unit ENV-III (見つからなければ BME280) の気圧から高度推定 (標準大気式) |
| 標高オフセット補正 | 国土地理院APIから標高を1回取得し、気圧センサー高度との差分でオフセット補正 |
| MQTT 送信 | JSON ペイロードを `mqtt_topic` へ (証明書による TLS) |
| セーフ処理 | GNSS 異常値・未固定除外<BR>周回カウントの多重検出防止 (デバウンス) / GPSロスト再測位時の誤検出防止 (テレポートガード)<BR>BLEタイムアウト時温度リセット<BR>ECU無信号時フェールセーフ |

## ハードウェア / 接続

- 基板: M5Stack Basic (Gray) / M5Stack Core2 (ESP32, PSRAM 使用)
- SD: SPI (GPIO4 / SHARED_SPI 設定)
- ECU: Serial1 115200 bps、RXピンはボード依存  
  (M5Stack Basic: RX=15, TX=0 / M5Stack Core2: RX=2, TX=0)  
  ※M5Stack Basicの起動不良対策としてGPIO12を使用しないため分岐 (コード参照)
  - 受信フォーマット (10Hz): `rpm,inj(ms),ign_ca,inj_end_ca,speed,distance,fuel(ml),km/L,worktime*XX` (`worktime` は0.1秒単位の小数秒。例 `267.3`。整数秒でも受理)  
    `XX` は先頭から `*` 直前までのXOR (16進2桁)。不一致・項目不足の行は破棄する。EFI側と同時に更新すること (旧形式とは非互換)  
    RXバッファは1024Bに拡大 (MQTT/HTTP中の取りこぼし対策)
- GNSS(GY-NEO-6MV2): Serial2 115200 bps
  (M5Stack Basic: RX=16, TX=17 / M5Stack Core2: RX=13, TX=14)  
  RXバッファは1024Bに拡大 (MQTT/HTTP中の取りこぼし対策)
  - 受信が止まったときの自動復帰: 実走で位置が26分間止まり、電源を入れ直すまで戻らなかったことがあるため、次の順で立ち直らせる。行ったことはシリアルに `[GNSS] ...` と出し、SDログの回数の列に残す
    1. NMEA の文が `GNSS_STALL_MS` (3秒) 届かない: Serial2 を開き直す (ロガー側の UART が止まった場合はこれで戻る)
    2. 開き直しても 3 秒届かない: 開き直すたびに、モジュールへホットスタート `$PCAS10,0*1C` も送る (保存した設定と衛星の軌道情報は残る)
    3. 文は届くのに、一度測位したあと位置が `GNSS_FIX_LOST_RESTART_MS` (10秒) 更新されない: ホットスタートを送る (最短 30 秒おき)
    - モジュールが完全に固まって UART の入力を受け付けないときは戻らない (電源の入れ直しが要る)。ホットスタートは ATGM336H 用なので、ポータブル仕様 (M5Stack GNSS Module) では 1 の開き直しだけ行う
  - ATGM336H も使える。出荷状態は 9600 bps でそのままでは読めないので、先に `tools/setup_atgm336h.py` で設定を書き込んでおく (「ATGM336H を使う場合の設定」を参照)
- 環境センサ (気圧/気温/湿度): 起動時に下の順で探し、先に見つかった方を使う。SDA/SCLは `M5.Ex_I2C.getSDA()/getSCL()` でボード既定値を自動取得
  1. [Unit ENV-III](https://docs.m5stack.com/en/unit/Unit_ENV-III): I2C 0x44 (SHT30: 気温/湿度) + 0x70 (QMP6988: 気圧)。BME280 より精度が高い (気温 ±0.2℃、湿度 ±2%RH)。BME280 と同じ I2C バス (Port A) に繋ぐ (M5Stack Basic で BLE 中継機と同時に使うときは Grove ハブで分岐する)
  2. BME280: I2C 0x76 / 0x77 (Unit ENV-III が見つからないときだけ使う)
  - どちらを使っているかは起動時のシリアル出力 (`Unit ENV-III initialized` / `Unit ENV-III not found`) で分かる
- ポータブル仕様 (`m5stack-basic-portable`、M5Stack Basic のみ): 上の GNSS と BME280 の代わりに [M5Stack GNSS Module](https://docs.m5stack.com/ja/module/Module_GNSS) を重ねて使う
  - GNSS(NEO-M9N): Serial2 38400 bps (RX=16, TX=17)、測位は出荷時の 1Hz のまま。モジュールの DIP スイッチで NEO_TXD を G16 に繋ぐ (ロガーからモジュールへは何も送らない)  
    PPS は使わない。割り当てる場合は ECU 用の G15 / G0 と SD の G4 を避ける
  - BMP280(気圧/気温センサ): I2C 0x76 (SDA=21, SCL=22)。湿度は測れないので SD ログでは空欄、MQTT では `null` になる。気温はモジュール内の温度なので外気より高めに出る。Port A に Unit ENV-III を繋ぐと、BMP280 の代わりに Unit ENV-III から気圧/気温/湿度を取る
  - ECU・BLE 中継機の処理は車載と同じまま動く (繋がないので速度・回転数などは 0)
  - 周回数は走行開始を待たずに、コントロールラインを通過するたびに数える (車載では ECU の走行時間が動き出してから数える)
- BLE 中継機 [nRF52840_BLE_Central](https://github.com/todateman/nRF52840_BLE_Central) (XIAO nRF52840): Port A 経由の I2C で接続（XIAO nRF52840側がI2Cスレーブ, addr=`0x08`）
  - M5Stack Basic: SDA=21, SCL=22 (環境センサ用`Wire`バスと共用。Port Aのピンが同一のため)
  - M5Stack Core2: SDA=32, SCL=33 (専用I2Cバス`Wire1`を使用。環境センサ用バスとはピンが異なるため)
  - 200ms間隔で以下4コマンドを順に送信し、それぞれ32byte固定フレーム（`[0]`=データ長, `[1..]`=文字列データ）でデータを取得。いずれも2秒以上有効データを取得できない場合は該当値を`0.0`にリセットする

    | コマンド | データ | 単位 | 有効範囲(範囲外は破棄) |
    | ---- | ---- | ---- | ---- |
    | `CMD_ENGINE_TEMP` (`0x01`) | エンジン温度 ([Chibi-T_Furoshiki_Heater](https://github.com/todateman/Chibi-T_Furoshiki_Heater)経由) | ℃ | 10.0 〜 150.0 |
    | `CMD_PRI_PRE` (`0x02`) | 1次側空気圧 ([Chibi-T_Furoshiki_AutoAirAdjust](https://github.com/todateman/Chibi-T_Furoshiki_AutoAirAdjust)経由) | MPa | 0.01 〜 0.72 |
    | `CMD_SEC_PRE` (`0x03`) | 2次側空気圧 (同上) | MPa | 0.01 〜 0.72 |
    | `CMD_FUEL_PRE` (`0x04`) | 燃圧 (同上) | MPa | -0.02 〜 1.05 |

- ボタン: A/B/C でモード選択 + 起動時設定。B長押し(1秒)で走行支援の加速パターンを切替 (長押しでは画面は替わらない。モード 0 へは B を短く押して離す)

### ATGM336H を使う場合の設定

GNSS モジュールは GY-NEO-6MV2 を標準とするが、ATGM336H も代わりに使える。  
ロガーは Serial2 を 115200 bps で開いて NMEA を読むだけで、モジュールへ設定は送らない (受信が止まったときにホットスタートを送るだけ)。  
出荷状態の ATGM336H は 9600 bps・1 Hz なので、ロガーにつなぐ前に `tools/setup_atgm336h.py` で設定を書き込み、モジュールに保存しておく。  
PC上で実行する (Python 3.10 以上、`pyserial` が必要)。  
モジュールは USB シリアル変換でPCにつなぐ。

```sh
python3 tools/setup_atgm336h.py /dev/cu.usbmodemXXXX                       # 設定して保存し、実際の出力を確認
python3 tools/setup_atgm336h.py /dev/cu.usbmodemXXXX --verify-only         # 書き込まず、現在の出力を確認するだけ
python3 tools/setup_atgm336h.py /dev/cu.usbmodemXXXX --rate-hz 5           # 更新レートを変える (1 / 2 / 4 / 5 / 10)
python3 tools/setup_atgm336h.py /dev/cu.usbmodemXXXX --sentences gga-rmc   # GGA も出す (衛星数・HDOP・GNSS高度)
```

| 項目 | 出荷状態 | 設定後 (既定) | 送るコマンド |
| --- | --- | --- | --- |
| 出力センテンス | GGA / GLL / GSA / GSV / RMC / VTG / ZDA / TXT | RMC のみ | `$PCAS03,0,0,0,0,1,0,0,0,0,0,,,0,0*03` |
| ボーレート | 9600 bps | 115200 bps | `$PCAS01,5*19` |
| 更新レート | 1 Hz | 10 Hz | `$PCAS02,100*1E` |
| 保存 | - | モジュール内に保存 (電源を切っても残る) | `$PCAS00*01` |

- 現在のボーレートは受信して自動で判定するので、設定済みのモジュールにかけ直してもよい
- 書き込んだあと 5 秒間受信し、ボーレート・出力センテンス・更新レート・壊れた行の有無を `OK` / `NG` で表示する。`NG` があれば終了コードが 1 になる
- 設定後に一度電源を入れ直し、`--verify-only` で設定が残っていることを確かめる
- 更新レートは測位している状態で確かめる。測位前は 10 Hz で出ていても、測位すると下がる個体がある
- 測位中に 10 Hz を保てるかは個体によって違う (#28)。保てない個体は `--rate-hz 5` で保存する

  | 個体 | 10 Hz の設定で測位中の出力 | 5 Hz の設定で測位中の出力 |
  | --- | --- | --- |
  | Rhinozix ATGM336H GPS+BDS (2026 年全国大会の決勝で使用) | 約 6 Hz。回が抜ける (決勝のログでは約 3 Hz) | 5.00 Hz。回は抜けない |
  | M5Stack Unit GPS SMA (ATGM336H-6N) | 10 Hz を保つ | 未確認 |

- Rhinozix の個体は、4 Hz の設定 (`$PCAS02,250`) を送っても 5 Hz で出た
- ロガーが使うのは時刻・日付・緯度経度・速度で、すべて RMC に入っている。RMC のみにすると衛星数・HDOP・GNSS高度は出ないが、ファームでは使っていない
- ファーム側の変更は不要 (`src/main.cpp` の `gnss_bps` は 115200 のまま)。配線は GY-NEO-6MV2 と同じで、つないだあと屋外で画面右上の GNSS の表示が `O` になることを確認する (`O` は直近 3 秒以内に測位した位置があること。一度測位したあとも、位置が古くなれば `x` に戻る)

## ソフトウェア依存ライブラリ (platformio.ini より)

- [TinyGPSPlus](https://github.com/mikalhart/TinyGPSPlus)
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
| `native` | 走行支援の判定ロジック (`include/StrategyAssist.h`)・規定時間に対する余裕の計算 (`include/LimitMargin.h`)・コース図の投影 (`include/CourseMap.h`) のテストをPC上で実行 | `pio test -e native` |
| `m5stack-basic-sim` | 走行支援画面・現在地表示画面の机上確認。もてぎのウェイポイント上を8倍速で仮想走行する (`STRATEGY_SIM=8`)。設定ファイルの `start_m` (無ければウェイポイント ID 1 の 8 m 手前) から発進する。SDに `motegi_waypoint.csv` と `motegi_strategy.json` が必要。SDログ・MQTTは止まる。**本番には書き込まない** (状態行に `SIM` と出る) | `pio run -e m5stack-basic-sim -t upload` |
| `m5stack-basic-sim-suzuka` | `m5stack-basic-sim` の鈴鹿版 (`STRATEGY_SIM_SUZUKA=1`)。速度の倍率は `platformio.ini` のこの環境の `STRATEGY_SIM` で変える (もてぎの環境とは別に指定する。1 で実時間)。SDに `suzuka_waypoint.csv` と `suzuka_strategy.json` が必要。勾配はウェイポイントの標高から取る (もてぎも同じ)。発進は本コース上の `start_m` (112 m) から。最終周は分岐 (約 1,990 m) から 15 m 内側のピット入口の道へ移り、ゴールライン (2,135 m) で周回を数える。ゴール後はブレーキで止まる。仮想ドライバーは頂上の止め時の合図にも従う。走行抵抗を 2 割増しにしてあるので頂上の速度は目標より低く (S 字約 11、ダンロップ約 5 km/h)、ダンロップの頂上の手前で毎周 5 km/h を割って「加速! 速度低下」が出る (ゴールは約 32 分) | `pio run -e m5stack-basic-sim-suzuka -t upload` |
| `m5stack-basic-portable` | 車体に載せずコースウォークなどで使うポータブル仕様 (M5Stack Basic + M5Stack GNSS Module)。GNSS は NEO-M9N (38400 bps)、気圧は BMP280 から取り、SD への記録は 1Hz になる (`GNSS_MODULE_M5=1`)。車載用のハードでは GNSS も気圧も取れないので**車載には書き込まない** | `pio run -e m5stack-basic-portable -t upload` |

### 画面例の描き直し

README の画面例 (`docs/strategy_screen_*.png` / `docs/course_map_*.png`) は、`src/main.cpp` の描画関数を抜き出して PC 上で描いている (`tools/render_screens/`)。走行支援画面・現在地表示画面の描画を変えたら描き直す。

```sh
pio run -e m5stack-basic                    # 初回だけ: M5GFX / ArduinoJson を .pio/libdeps に取得する
tools/render_screens/render.sh              # .pio/render_screens/out/ に全場面の PNG を出す (確認用)
tools/render_screens/render.sh --install    # README で使う画像を docs/ へ上書きする
```

- 必要なもの: clang、SDL2 (macOS は `brew install sdl2`)、Python 3
- 場面 (周回数・位置・速度・計画からの遅れ) は `tools/render_screens/harness.cpp` の `main()`。設定は `sd/motegi_strategy.json` をファームと同じ関数で読む
- 描画に使う関数・変数が増えたら、`extract.py` の抜き出し対象と `common.h` の宣言を合わせる。M5GFX を更新したら `.pio/render_screens/obj` を消してから回す

#### 仮想走行の動画

`m5stack-basic-sim` / `m5stack-basic-sim-suzuka` と同じ仮想走行を PC 上で等倍で回し、走行支援画面 (左) と現在地画面 (右) を並べた動画 (MP4、1280×480、10 fps、ブザーの音つき) にする。実機に書き込まずに、8 周ぶんの合図と画面の動きを確かめられる。

```sh
tools/render_screens/render.sh --video suzuka   # .pio/render_screens/video/suzuka_sim.mp4 (約 32 分)
tools/render_screens/render.sh --video motegi   # .pio/render_screens/video/motegi_sim.mp4
```

- 必要なもの: 上と同じもの + ffmpeg (macOS は `brew install ffmpeg`)。設定は `sd/<コース>_strategy.json` と `sd/<コース>_waypoint.csv`
- 仮想走行 (`updateSimulatedRun()`)・周回の判定・走行支援の更新とブザー (`updateStrategyAssist()`)・描画は `src/main.cpp` から抜き出したものをそのまま使う (`tools/render_screens/video.cpp`)。`loop()` 1 回を 20 ms、表示を 100 ms ごととして時計を進める
- ブザーは `M5.Speaker.tone()` の代わりに矩形波を書く (加速は短く 2 回、停止は長く 1 回、続いている間は 2 秒ごと)
- ゴールして止まってから 15 秒で終わる。1 時間でゴールしなければ打ち切る

### デバッグ

- シリアル出力: CSV形式 `lat, lon, alt, loc, Spd_GPS, rpm, Spd_PULSE, distance, gasml, dispergas, worktime, EngTemp, Pressure, Temp, Humidity, PRI, SEC, FUEL, datetime`
- 例外デコード: `monitor_filters = esp32_exception_decoder`

## 高度推定 / 国土地理院API補正

- 高度は GNSS ではなく気圧センサー (Unit ENV-III、見つからなければ BME280) の気圧から算出
- 海面気圧は初期値 `101.325kPa` を使用
- Wi-Fi接続済みかつGNSS座標が有効なときのみ、国土地理院API (`getelevation.php`) で標高取得を低頻度で試行
- 標高取得成功時、その時点の気圧センサー生高度との差分をオフセットとして1回だけ固定し、以後は再取得しない
- 以後の表示/ログ高度は `気圧センサー生高度 + オフセット` で算出
- API取得は HTTP を優先し、失敗時は HTTPS フォールバック

## 起動時の Wi-Fi 操作

| 操作 | 説明 |
| ---- | ---- |
| A ボタン | Wi-Fi 設定ポータル (AP モード, QR コードで簡易接続) |
| C ボタン | Wi-Fi 無効 (MQTT も無効) |
| 無操作 (5秒) | 自動接続 (保存済 SSID) / 失敗時は接続先SSIDを設定するWebUIへのQRコードを表示 |

## 表示モード詳細

1. モード 0: 速度, 残り周回 (最後は G/FINISH), 走行時間進行バー。走行支援が有効な場所では走行支援画面に替わる
2. モード 1: 回転数, 噴射時間, 進角角度バー
3. モード 2: コース図の上に現在地・加速開始位置・規定時間の仮想車を重ね、四隅に速度・加速停止速度・残り周回・走行時間、左右に規定時間に対する余裕を表示

### モード2(現在地表示)

**試験的な機能。** コース図の上に GNSS の現在地と加速開始位置を重ね、加速停止速度を同じ画面に出す。「どこでエンジンを始動し、何 km/h で止めるか」をコース上の位置関係でつかむための画面。

#### 画面例

実機の写真ではなく、`src/main.cpp` の描画処理 (`drawCourseMapMode()`) を PC 上で動かして出力した画像 (2 倍に拡大)。数値は例 (設定は `sd/motegi_strategy.json`。走行時間は計画の時間に遅れ・進みを足した値)。描画を変えたら `tools/render_screens/render.sh --install` で描き直す ([画面例の描き直し](#画面例の描き直し))。

| 加速開始位置の手前 | 加速開始 | 加速停止速度に到達 |
| :----: | :----: | :----: |
| <img src="docs/course_map_approach.png" width="260" alt="加速開始位置の手前の画面。次の位置が黄色、下端の帯に加速まで59mと黄色で表示。後ろに規定時間の仮想車の菱形、左に余裕+69、右に予測+79"> | <img src="docs/course_map_burn_now.png" width="260" alt="加速開始の画面。下端の帯が緑で加速!と表示"> | <img src="docs/course_map_cut_now.png" width="260" alt="加速停止速度に到達した画面。現在地の矢印が緑、下端の帯が赤で停止!と表示"> |
| 次の加速開始位置が黄色。◆ (規定時間の仮想車) は約 470 m 後ろ、左に余裕 +69、右に予測 +79 | 下端の帯が緑 | 噴射中は矢印が緑。下端の帯が赤。38 秒遅れで余裕は +7 (黄) |

| 最終周 | 予備パターン・燃圧低下 | 設定ファイルが無い場所 |
| :----: | :----: | :----: |
| <img src="docs/course_map_coast_home.png" width="260" alt="最終周の画面。3つ目の加速開始位置が白抜き、下端の帯に惰行でゴールと表示"> | <img src="docs/course_map_fallback_fuel_low.png" width="260" alt="予備パターンの画面。加速開始位置が4つ、右上に燃圧低下と表示"> | <img src="docs/course_map_suzuka_no_plan.png" width="260" alt="設定ファイルが無い場所の画面。コース図と現在地だけが出て、下端の帯にプラン未設定と表示"> |
| 3 つ目の加速開始位置は ○ (使わない)。余裕は 3 桁 (+124) | 加速開始位置が 4 つ。右上に橙で `燃圧低下` | コース図と現在地だけ |

#### 画面の見方

- 中央がコース図 (北が上)。サーキット判定 (`Loc`: su / mo / to) に応じたウェイポイントCSVを読み込み、順に結んで描く
  - 白い短い線がコントロールライン (周回数が増える線。もてぎはコース上のコントロールラインで、ゴールラインと同じ並び)
  - 矢印が現在地。GNSS の測位位置をそのまま置く (コース線上へは寄せない)。向きはコースの進行方向。**噴射中は緑**、惰行中は水色、3 秒以上測位が無いときは灰色。起動後に一度も測位できていないあいだは出ない
  - ● と番号が加速開始位置。**次の位置は黄色**、最終周に使わない位置は ○ (モード 0 の位置バーの ▼ / ▽ と同じ)
  - マゼンタの ◆ が**規定時間の仮想車** (規定時間ちょうどにゴールする一定の速さで、走行時間ぶん進んだ位置。[規定時間に対する余裕](#規定時間に対する余裕))。**矢印が ◆ より前にいれば規定に対して余裕がある**。発進前は発進位置、規定時間を過ぎたらゴールで止まる。設定ファイルに `race_m` / `limit_s` が無いと出ない
  - コース図の下に「場所 / 加速回数」。測位が古いときは黄色で `GNSS:x` が付く
- 左上が現在の速度 (km/h)、右上が**今の周の加速停止速度**。色の意味はモード 0 と同じ (上げているあいだはマゼンタ、下げているあいだは水色)。直近の加速で燃圧がしきい値を下回ると、その下に橙で `燃圧低下` と出る
- 左下が残り周回、右下が走行時間
- 左の中ほど `余裕` が**いまの位置での規定時間に対する余裕** [秒]、右の中ほど `予測` が**計画の残りどおりに走ったときの、ゴール時の規定に対する余裕** [秒]。色はモード 0 と同じ (目標の帯より大きい=水色、帯の中=緑、帯より小さい=黄、マイナス=マゼンタ)。発進後、位置が一度取れてから出る
- 下端の帯がドライバーへの指示。文言はモード 0 と同じで、`加速!` のあいだは帯が緑、`停止!` のあいだは帯が赤になる (コース図の色は変わらない)
- ブザーと B 長押しでの予備パターン切替は、この画面でもモード 0 と同じように働く。B を長押ししても画面はこのままで、パターンだけが替わる (モード 0 へ移るのは B を 1 秒未満で離したとき)
- 走行支援の設定ファイルが無い場所では、コース図と現在地だけを出し、下端の帯は `プラン未設定` になる (設定ファイルの内容に異常があるときは赤字で `支援設定異常`)
- ウェイポイントCSVが読めないときは、コース図の代わりに `ウェイポイント未読込` と出る

## 走行支援

もてぎ全国大会 2026 の走行戦略 ([ECU リポジトリ](https://github.com/todateman/UNOR4_Chibi-T_EFI) の `document/motegi_strategy_2026.md`) の判断を画面に出す。  
**SDカードにその場所の設定ファイル (もてぎは `motegi_strategy.json`) があるときだけ有効**になり、モード 0 がこの画面に替わる。  
走行パターンの値はファームには持っていない。**走行支援が不要なときは、設定ファイルを microSD から削除する** (従来のモード 0 に戻る)。

### 走行支援の画面例

実機の写真ではなく、`src/main.cpp` の描画処理 (`drawStrategyMode()`) を PC 上で動かして出力した画像 (2 倍に拡大)。数値は例 (設定は `sd/motegi_strategy.json`。走行時間は計画の時間に遅れ・進みを足した値)。描画を変えたら `tools/render_screens/render.sh --install` で描き直す ([画面例の描き直し](#画面例の描き直し))。

| 走り出すまで | 惰行中 | 加速開始位置の手前 |
| :----: | :----: | :----: |
| <img src="docs/strategy_screen_idle.png" width="260" alt="発進前の画面。スタート待ちと表示"> | <img src="docs/strategy_screen_coast.png" width="260" alt="惰行中の画面。加速まで299mと表示。右下に余裕+26を緑で表示"> | <img src="docs/strategy_screen_approach.png" width="260" alt="加速開始位置の手前の画面。加速まで59mと黄色で表示"> |
| 発進前。右下に加速位置、右上に加速停止速度 | 2 周通過が 18 秒遅れ → 加速停止速度 33.5 (マゼンタ)。規定に対しては +26 秒 (目標の帯の中なので緑) | 100 m 手前から黄色。余裕 +69 (帯より大きいので水色) |

| 加速開始 | 加速中 | 加速停止速度に到達 |
| :----: | :----: | :----: |
| <img src="docs/strategy_screen_burn_now.png" width="260" alt="加速開始の画面。全面が緑で加速!と表示"> | <img src="docs/strategy_screen_burning.png" width="260" alt="加速中の画面。35.5で停止と表示"> | <img src="docs/strategy_screen_cut_now.png" width="260" alt="加速停止速度に到達した画面。全面が赤で停止!と表示"> |
| 全面が緑 | 38 秒遅れ → 加速停止速度 35.5 まで引っ張る。余裕 +7 (帯より小さいので黄) | 全面が赤 |

| 最低限界速度 | 最終周 | 燃圧低下 |
| :----: | :----: | :----: |
| <img src="docs/strategy_screen_guard.png" width="260" alt="速度が16km/hまで落ちた画面。全面が緑で加速!速度低下と表示"> | <img src="docs/strategy_screen_coast_home.png" width="260" alt="最終周の画面。惰行でゴールと表示"> | <img src="docs/strategy_screen_fuel_low.png" width="260" alt="燃圧低下の画面。下端にオレンジの警告"> |
| 16 km/h まで落ちた。余裕が 3 桁 (+105) なので数字を小さくし、`余裕` の文字は省く | 3 つ目の加速位置は ▽ (使わない) | 35 秒早い → 加速停止速度 32.0 (水色) |

| 予備パターン |
| :----: |
| <img src="docs/strategy_screen_fallback.png" width="260" alt="予備パターンの画面。状態行に4回予備と表示"> |
| B 長押しで切替。状態行に `4回予備`、加速位置が 4 つ |

鈴鹿の頂上の止め時 (設定は `sd/suzuka_strategy.json`。S 字の登り):

| 頂上の手前の登りで加速中 | 止める速度に到達 | 頂上へ惰行 |
| :----: | :----: | :----: |
| <img src="docs/strategy_screen_crest_burning.png" width="260" alt="S字の登りで加速中の画面。頂上34.1で停止と表示し、右上の停止速度も34.1"> | <img src="docs/strategy_screen_crest_cut_now.png" width="260" alt="止める速度に到達した画面。全面が赤で停止!と表示"> | <img src="docs/strategy_screen_crest_cresting.png" width="260" alt="頂上へ惰行している画面。速度16.2で頂上まで惰行と表示"> |
| 1,405 m・30.9 km/h。止める速度は位置ごとに下がり、ここでは 34.1 (加速停止速度の 29.0 では止めない) | 1,414 m・33 km/h で 0.5 秒先の表の速度に届いた | 最低限界速度 (21 km/h) を割っても、5 km/h までは「加速!」を出さない |

### 画面表示の見方

| 画面 | 意味 | ドライバーの操作 |
| ---- | ---- | ---- |
| **全面が緑**「加速!」 | 加速開始位置に来た | エンジンを始動して全開 |
| **全面が緑**「加速! 速度低下」 | 最低限界速度 (16 km/h) まで落ちた | 場所によらず加速 |
| 「32.5で停止」 | 加速中 | 右上の加速停止速度まで引っ張る |
| 「頂上34.1で停止」 | 頂上の止め時の区間 (鈴鹿の S 字・ダンロップの登り) で加速中。止める速度は位置ごとに変わる | 「停止!」が出るまで全開 |
| **全面が赤**「停止!」 | 加速停止速度に達した (頂上の止め時の区間では、惰行で頂上を目標の速度で越えられる速度に達した) | エンジンを止める |
| 「頂上まで惰行」 | 頂上の止め時で止めたあと、頂上まで | そのまま惰行 (5 km/h まで落ちたら「加速! 速度低下」が出る) |
| 「加速まで 185m」 | 惰行中。100 m 手前から黄色になる | 準備 |
| 「惰行でゴール」 | 最終周の加速が終わった | そのまま惰行 (16 km/h まで落ちたら加速) |
| 「位置なし」 | GNSSが測位できていない | 目視の目印と最低限界速度の合図で走る |
| 下端にオレンジ「燃圧低下」 | 直近の加速で燃圧がしきい値を下回った | B長押しで予備パターンへ |

- 最も左上の表示は「場所 / 加速回数」
- 最も右上に「SDカードの有無 / GNSSの受信状態 / MQTTの送信有効・無効」
- 左上の7 セグメント風の数字が現在の速度（km/h）
- 右上の数字が**今の周の加速停止速度**。コントロールライン通過のたびに走行時間を目標と比べ、遅れ 8 / 20 / 35 秒超で +1 / +2 / +3 km/h、30 秒超早ければ −0.5 km/h する (毎周計算し直す)。上げているあいだはマゼンタ、下げているあいだは水色
  - 頂上の止め時の区間で加速しているあいだは、位置ごとの止める速度 (白)
- 速度バーの黄色の線が最低限界速度、赤い線が加速停止速度 (頂上の止め時の区間で加速中は止める速度)
- 中央にドライバーへの指示を表示
- 位置バーは1周あたりのコース上の現在地を示す。
  - 位置バーの ▼ が加速開始位置、縦線が現在地。最終周に使わない位置は ▽
- 左下に現在の周回数を表示
- 中央下に走行開始からの経過時間を表示。遅れはマゼンタ
- 右下の大きい数字 (`余裕` の右) が**いまの位置での規定時間に対する余裕** [秒] (`+45` は規定時間ちょうどにゴールするペースより 45 秒先行、マイナスは遅れ)。周回の途中でも加速・惰行に合わせて増減する ([規定時間に対する余裕](#規定時間に対する余裕))
  - 色は余裕の目標の帯 (`margin_band_s`。既定 10〜30 秒) で分ける。**帯より大きい=水色** (余らせすぎ。燃料の無駄)、**帯の中=緑**、**帯より小さい=黄**、**マイナス=マゼンタ** (このままでは規定時間を超える)
  - その上の行の `通過+52` が直近のコントロールライン通過時の余裕、`予測+77` が**計画の残りどおりに走ったときの、ゴール時の余裕**。予測は「計画のゴールでの余裕 (もてぎは +77 秒) + いまの計画からの進み」なので、計画どおりならずっと同じ値になる
  - 発進前は加速位置の一覧。設定ファイルに `race_m` / `limit_s` が無い場合は、従来どおり直近の通過タイムと目標 (`splits_s`) との差 (`+18s` は 18 秒遅れ) を出す。目標より 2 分以上早い値は時間基準か周回数のずれとみなして使わない
- 下端の帯は規定時間までの残り時間 (従来のモード 0 の走行時間バーと同じ)。位置バーの現在地と同じ向きにそろえ、時間が進むと左から右へ短くなる。色は走行時間の数字と同じ (直近の通過が遅れていればマゼンタ)
  - 同じ帯に全周回の進み具合を重ねてある。縦の細い線が周の区切り、水色の縦線が走行距離の現在地 (ECU の走行距離 ÷ 規定距離)
  - 帯の上辺の ▼ の切り欠きが**計画上の現在地** (戦略どおりなら今いるはずの位置)。`splits_s` のあいだを直線で補間して求める。**水色の線が切り欠きに重なっていれば計画どおり、切り欠きより左なら遅れ、右なら先行**
  - 帯の左端と ▼ は比べる相手が違う。帯の左端は規定時間を均等なペース (平均 25 km/h) で走った場合の位置、▼ と右下の秒数は戦略の目標。戦略は目標時間に余裕を持たせ、最終周を惰行で終えるぶん前半で先行するので、▼ はいつも帯の左端より右にある (6 周通過の時点で帯の幅の 5% ほど、約 2 mm)。**水色の線が帯の左端より右にあっても、▼ より左なら目標には遅れている**
  - 帯の 1 ドットは約 8 秒 (約 55 m) にあたり、▼ の位置も周の途中では 2 ドットほどずれる。▼ で読めるのは 30 秒程度以上の大きなずれ。細かい遅れは右下の秒数で見る
  - 設定ファイルの `splits_s` にゴールの目標時間が無い場合 (規定周回数より 1 つ少ない場合)、最終周は ▼ が出ない
- **B長押し (1秒)** で通常パターン ⇄ 予備パターンを切り替える (状態行に `予備` と出る)
- ブザーは M5Stack Basic のみ (加速=短く2回、停止=長く1回、続いている間は2秒ごと)。Core2 は内蔵スピーカが ECU 用 UART と同じピン (GPIO0/2) を使うため鳴らさない

### 判定の仕組み

- 周内位置: 現在地を最近傍ウェイポイントの前後の区間へ射影し、CSVの距離列から求める。公式の 1 周の距離 (`lap_m`) に合わせて伸縮し、測位からの経過時間ぶんを車速で進める。ウェイポイントを結んだ線から 60 m 以上離れている、または 3 秒以上測位が無い場合は「位置なし」
- 加速中の判定: ECUの噴射時間が 0 より大きい (ECUはエンジン停止中 0 を送る)
- 加速開始位置を通過した時点で速度が「加速停止速度 − 2 km/h」を超えていれば、その位置では合図を出さない
- 「加速!」は噴射が 1.5 秒続くまで出し続ける (再始動に失敗したら出たままになる)。200 m 過ぎるか次の位置の 100 m 手前で消える
- 最終周は `final_lap_marks` 個目までの位置だけ有効。最終周に入る通過タイムが `final_extra_if_split_over_s` を超えていれば全位置を有効にする
- **頂上の止め時** (`crest_cut`。鈴鹿): 頂上の手前の登り (`from_m`〜`crest_m`) で加速しているあいだは、加速停止速度では止めない。止めてよい速度の表 (惰行で頂上を目標の速度で越えられる速度。ECU 側のシミュレータが名目の走行抵抗で頂上から逆算した値) を、**合図からエンジンが止まるまでの遅れ (`cut_lag_s`) のあいだに進む先の位置** (現在地 + 速度 × 0.5 秒) で引き、今の速度がそれ以上なら「停止!」を出す。シミュレータと違い、遅れのあいだの加速度は見込まない (合図はわずかに遅れ、頂上の速度は 0.5 km/h ほど高くなる)
  - 「停止!」はエンジンが止まるまで出し続ける。止まったら頂上を越えるまで「頂上まで惰行」で、最低限界速度を割っても `restart_kmh` (5 km/h) までは「加速!」を出さない。目印でも出さない
  - 表を使う前に自分で止めた場合は、今までどおり最低限界速度で「加速!」を出す
  - 位置なしのときは使わない (加速停止速度で止める)
- **加速しない区間** (`coast_only_m`。鈴鹿の S 字の頂上の先の下りと、ダンロップの頂上から 1・2 コーナーまで): 加速中に入ったら「停止!」を出し、区間の中では `restart_kmh` までは「加速!」を出さない
- 判定ロジックは `include/StrategyAssist.h` (Arduino非依存)。ルールは EFI 側の `tools/strategy_sim.py` と同じ
- 加速停止速度の補正は、いまも計画の通過タイム (`splits_s`) との差で決める。規定時間に対する余裕は表示だけで、補正には使っていない

### 規定時間に対する余裕

規定時間ちょうど (もてぎは 39:20.11) にゴールする**仮想車**と比べる。仮想車は、発進からゴールまでの道のり (`race_m`) を規定時間で割った一定の速さで進む。

- 道のりは、走行距離の積算 (ECU の `distance`) ではなく、**周回数とコース上の位置 (周内位置) から出す**。もてぎは発進 (160 m)・ゴール (2,224 m) とも周回路の内側の別レーンなので、ECU 側の `strategy_sim.py` の `race_span()` と同じく、発進位置から 1 回目のコントロールライン通過までを 2,064 m、以後 1 周ずつ足し、7 回目の通過でゴール (6 × 2,341.38 + 2,064 = 16,112 m)。仮想車は平均 24.58 km/h (コース上の道のりで見た速さ)
- コントロールラインの付近では周回数と位置が少しずれる (数えた直後に位置がまだ手前を指す・位置が先にラインを越える) ので、前回から半周以上飛んだ値は 1 周ずらして使う。周回数を取りこぼしても道のりは途切れない。位置なしのあいだは最後の道のりのままにする (時間が進むぶん余裕は減って見える)
- **余裕 = 仮想車がいまの位置に来る時刻 − 走行時間** (= 道のり × 規定時間 ÷ `race_m` − 走行時間)。ゴールでは「規定時間 − ゴールタイム」になり、そこで止まる
- **予測 = 規定時間 − (走行時間 + 計画のゴール時間 − 計画でいまの位置に着く時間)**。計画の時間は設定ファイルの表 (`plan_t_s`。道のり 100 m ごとの計画の累積時間) を直線で補間する
- 計算は `include/LimitMargin.h` (Arduino非依存。`pio test -e native` でテスト)

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
| `marks_m` | 加速開始位置 [m] (ウェイポイント ID 1 から、昇順、最大 6) | **必須** | 400, 800, 1300 |
| `final_lap_marks` | 最終周に有効な位置の数 (先頭から) | 全位置が有効 | 2 |
| `final_extra_if_split_over_s` | 最終周に入る通過タイムがこれを超えたら全位置を有効にする [s] | この判定をしない | 1934 |
| `guard_kmh` | 最低限界速度 [km/h] | 最低限界速度の合図を出さない | 16.0 |
| `splits_s` | n 周通過時の目標走行時間 [s] (最大 12)。最後はゴールの目標時間 | 加速停止速度を補正せず、計画上の現在地も出さない | 282, 603, 923, 1244, 1564, 1885, 2283 |
| `fallback` | 予備パターン (`v_off` / `marks_m` / `final_lap_marks`) | 予備パターンなし (B長押しは無効) | 30.0 / 200, 600, 1000, 1400 / 3 |
| `fuel_warn_mpa` | 加速中の燃圧がこれを下回ったら警告 [MPa] | 燃圧の警告を出さない | 0.30 |
| `beep` | ブザーを鳴らすか | 鳴らす | true |
| `start_m` / `control_m` | 発進位置・コントロールライン (周回を数える位置) [m] (ウェイポイント ID 1 から) | 0 | 160 / 2224 |
| `race_m` | 発進からゴールまでの道のり [m] | 規定時間に対する余裕・仮想車を出さない (`limit_s` も同じ) | 16112.3 |
| `limit_s` | 規定時間 [s] | 同上 | 2360.11 |
| `margin_band_s` | 規定時間に対する余裕の目標の帯 [下限, 上限] (s) | 10, 30 | 10, 30 |
| `plan_step_m` / `plan_t_s` | 計画の「道のり → 累積時間」表。道のり 0, `plan_step_m`, 2 × `plan_step_m`, … での累積時間 [s] (最大 200 個)。最後の 1 つはゴール (`race_m`) の時間 | ゴール時の予測を出さない | 100 / 0, 14.4, 26.2, … 2283.5 |
| `crest_cut` | 頂上の止め時の区間 (最大 2、昇順)。`from_m` / `crest_m` (区間 [m])・`step_m` (表の間隔 [m])・`kmh` (位置 `from_m`, `from_m` + `step_m`, … と最後に `crest_m` での、エンジンを止めてよい速度 [km/h]。最大 64 個) | 頂上の止め時を使わない | (もてぎは無し。鈴鹿は 1300〜1533 m と 1780〜1916 m、5 m ごと) |
| `cut_lag_s` | 止める合図からエンジンが止まるまでの遅れ [s] (0〜3)。頂上の止め時の表をこの時間ぶん先の位置で引く | 0 | (鈴鹿 0.5) |
| `restart_kmh` | 頂上の止め時で止めたあと・加速しない区間で「加速!」を出す速度 [km/h] | 5 | (鈴鹿 5.0) |
| `coast_only_m` | 加速しない区間 [[開始 m, 終了 m], …] (最大 4。開始 > 終了なら周の終端をまたぐ) | 加速しない区間なし | (鈴鹿 [1533, 1652], [1914, 740]) |

必須項目が無い、JSON として読めない、位置が昇順でない、位置が 1 周の距離を超えている、加速停止速度が最低限界速度以下、`plan_t_s` が昇順でない・個数が `race_m` と合わない、帯の下限が上限より大きい、`crest_cut` の表の個数が区間と `step_m` に合わない、などの場合は設定全体を採用しない (`支援設定異常`)。ファイルの大きさは 4,095 バイトまで (それより前のファームは 1,535 バイトまでなので、`plan_t_s` の入ったファイルは読めない)。

### 設定ファイルの生成

設定ファイルは、ECU リポジトリの戦略パラメータ (`tools/strategy/<コース>_<年>.json`。もてぎは `motegi_2026.json`) から `tools/make_strategy_json.py` で作る。PC上で実行する (Python 3.10 以上、追加ライブラリ不要)。  
ECU リポジトリの `tools/strategy_sim.py` を読み込んで `optimize` と同じ計算をするので、ECU リポジトリが手元に要る (既定はこのリポジトリの隣の `../UNOR4_Chibi-T_EFI`。別の場所なら `--sim-dir` で `tools` ディレクトリを指定する)。

```sh
python3 tools/make_strategy_json.py motegi --dry-run        # 書き込まずに結果と変更点だけ表示
python3 tools/make_strategy_json.py motegi                  # sd/motegi_strategy.json を書き換える
python3 tools/make_strategy_json.py motegi --target 38:30   # 目標時間を変える (既定はパラメータの target_s)
python3 tools/make_strategy_json.py motegi --margin-band 10 30   # 規定時間に対する余裕の目標の帯を変える

# 初めて作るコースは、パラメータから決まらない項目をオプションで与える (2027 鈴鹿。目印は下の phases の節)
python3 tools/make_strategy_json.py suzuka --marks 1350 1730 --fuel-warn-mpa 0.30 --beep --margin-band 60 900
```

- コースは `suzuka` / `motegi` / `toyota` で指定する (必須)。出力は `sd/<コース>_strategy.json`
- 入力は `tools/strategy/<コース>_*.json` のうち名前順で最後のもの (年が新しいもの)。別のファイルを使うときは `--params` で指定する

| 項目 | 決め方 |
| ---- | ---- |
| `lap_m` / `guard_kmh` | パラメータの `course` からそのまま (`total_m ÷ laps` / `guard_kmh`) |
| `v_off` / `marks_m` | `optimize` の推奨パターン |
| `splits_s` | 推奨パターンをずらさずに走ったときの各周の通過タイム。最後はゴールの時間 |
| `final_lap_marks` | 最終周に加速を始める回数。加速開始位置を ±30 m ずらした 3 通りの最大 |
| `final_extra_if_split_over_s` | 規定時間 − 15 秒 − 最終周の所要時間 (同じ 3 通りの最長) |
| `fallback` の `v_off` / `marks_m` | パラメータからは決まらない。**書き換える前のファイルの値を引き継ぐ** (`--fallback-v-off` / `--fallback-marks` で差し替え)。`final_lap_marks` は計算し直す |
| `fuel_warn_mpa` / `beep` | パラメータからは決まらない。**書き換える前のファイルの値を引き継ぐ** (`--fuel-warn-mpa` / `--beep` / `--no-beep` で差し替え) |
| `start_m` / `control_m` / `race_m` | `strategy_sim.py` の `race_span()` / `control_m()` (パラメータの `course.start_m` / `finish_m` / `control_m`)。`race_m` はゴールまでの通算距離 − 発進位置 |
| `limit_s` | パラメータの `course.limit_s` からそのまま |
| `margin_band_s` | パラメータからは決まらない。**書き換える前のファイルの値を引き継ぐ** (`--margin-band` で差し替え)。どちらも無ければ書き出さない (ファームの既定 10〜30 秒) |
| `plan_step_m` / `plan_t_s` | `splits_s` と同じ走行の速度トレース (`strategy_sim.py plan --trace` と同じもの) から、道のり 100 m ごとの時刻 (0.1 秒単位) と、最後にゴールの時刻 |
| `crest_cut` / `cut_lag_s` | パラメータに `course.crest_cut` があれば、`strategy_sim.py` の `crest_cut_table()` (1 m ごと) を 5 m ごとに引き (64 個を超える区間は間隔を広げる)、最後に頂上の目標速度 `target_kmh` を足す。遅れは `burn.cut_lag_s` |
| `coast_only_m` | パラメータの `course.coast_only_zones_m` からそのまま |
| `restart_kmh` | `crest_cut` か `coast_only_m` があるときだけ書く。**書き換える前のファイルの値を引き継ぐ** (`--restart-kmh` で差し替え)。どちらも無ければ 5 km/h (戦略書の「頂上の手前で 5 km/h を割りそうなら始動する」。シミュレータの `restart_kmh` 3 km/h は、ファームでは停車とみなす速度と同じで合図が出ないため使わない) |

- 引き継ぐ項目は、出力先にファイルが無く、オプションでも与えなければ書き出されない (予備パターンなし・燃圧の警告なしになる)
- 予備パターンが規定時間を超える場合は警告を表示する
- 書き出す前に、上の「設定全体を採用しない」条件と同じ検査をする。当てはまればファイルを書き換えずに終了する

#### 周ごとにパターンを替えるプラン (練習走行用)

パラメータ JSON に `phases` (何周目からどのパターンで走るか) があるときは、`optimize` の計算をせず、書かれたとおりに設定ファイルを作る。もてぎの練習走行用は ECU リポジトリの `tools/strategy/practice_motegi_2026.json`。

```sh
# 本番用の sd/motegi_strategy.json を上書きしないよう、出力先を分ける
python3 tools/make_strategy_json.py motegi \
    --params ../UNOR4_Chibi-T_EFI/tools/strategy/practice_motegi_2026.json \
    --fuel-warn-mpa 0.30 --beep -o sd/motegi_strategy_practice.json
```

| 項目 | 決め方 |
| ---- | ---- |
| `v_off` / `marks_m` | 1 つ目の段階。**通常パターン**になる (起動直後に有効) |
| `fallback` の `v_off` / `marks_m` | 2 つ目の段階。**予備パターン**になり、その周に入るところで B 長押しで切り替える |
| `splits_s` | 途中でパターンを切り替えて走ったときの各周の通過タイム。最後はゴールの時間 |
| `final_lap_marks` / `final_extra_if_split_over_s` | 最終周を走る側 (予備パターン) について計算する。通常パターンの `final_lap_marks` は全位置 |

- 段階は 2 つまで (ファームが持てるパターンは通常と予備の 2 つ)。3 つ以上はエラーで終了する
- `--marks` で 1 つ目の段階の `marks_m` を置き換える。鈴鹿 2027 の `phases` は `marks_m` が空 (登りで最低限界速度 21 km/h まで落ちたら加速する) だが、ファームは目印が 1 個以上要るので与える。目印は速度が `v_off` − 2 km/h 未満なら加速を出すので、21 km/h まで落ちる位置 (約 1,305・1,694 m) より手前に置くと、下りの勢いが残っている周に早く加速が出て、29 km/h で止めたあともう一度始動することになる。そこで先 (1,350・1,730 m) に置き、最低限界速度の加速が先に出るようにしている (シミュレータでは乾いた路面・雨とも目印なしの計画と同じ走り)
- `--target` と、段階が 2 つあるときの `--fallback-v-off` / `--fallback-marks` は使えない (エラーで終了する)
- 入力の既定 (`tools/strategy/<コース>_*.json` のうち名前順で最後) に練習用が選ばれないよう、練習用のファイル名は `<コース>_` で始めない。`--params` で指定する
- 出力先を分けた場合は `fuel_warn_mpa` / `beep` を引き継ぐファイルが無いので、オプションで与える
- microSD には `motegi_strategy.json` の名前で入れる。**練習走行のあとは本番用に入れ替える**
- 走行中にロガーが再起動すると通常パターン (1 つ目の段階) に戻る。切り替えたあとなら B 長押しをやり直す

#### ECU 側との照合

次のどれかに当たると、ファイルを書き換えずにエラーで終了する (`--dry-run` でも同じ)。

| 照合するもの | ECU 側 | このスクリプト側 |
| ---- | ---- | ---- |
| パラメータ JSON の形式 (現在 3。`course.start_m` / `course.finish_m` と `fuel_slack` を足した) | JSON の `"format_version"` | `PARAMS_FORMAT_VERSION` |
| シミュレータの仕様 (推奨パターンの選び方、関数の引数・戻り値。現在 3) | `strategy_sim.py` の `INTERFACE_VERSION` | `SIM_INTERFACE_VERSION` |
| 規定周回数 | JSON の `course.laps` | `src/main.cpp` の `totallaps_su` / `totallaps_mo` |

- 版数が違う・書かれていない場合は、ECU 側で何が変わったかを確かめ、スクリプトを合わせてから対応版数を上げる。スクリプトは `optimize` の選定手順を写して持っているので、ECU 側で選び方を変えたら `INTERFACE_VERSION` を上げること
- 新しいコースのパラメータ JSON にも `"format_version"` を書く
- ECU 側のウェイポイント CSV が、このリポジトリの `sd/<コース>_waypoint.csv` と違う場合は警告を表示する (加速開始位置はこの距離を基準にするため)。書き出しは止めない

## 周回カウント仕様

- 実装ファイル: `src/main.cpp` (`updateGNSS()` / `updateLapCountByControlLineCrossing()`)
- 各サーキット (鈴鹿/茂木/豊田SENTAN) ごとに、コントロールラインを2点の緯度経度 (外側/内側) で定義済み
- GNSSの「前回測位位置 → 今回測位位置」を結ぶ線分が、サーキット判定結果 (`Loc`) に対応するコントロールライン (2点の線分) と交差したかを、外積 (クロス積) を用いた線分交差判定 (orientation/CCW法) で毎測位ごとに判定
- 交差を検知するたびに `Lapcount` を1加算 (旧実装の「走行距離 ÷ 1周あたり距離」による推定方式から変更)
- 鈴鹿はゴールがコースから分かれるピット入口の道の上にあり、コントロールライン (本コース上) を通らずにゴールする。そのため最終周 (`Lapcount` が規定周回数 − 1) に限り、ゴールライン (`goalline_*_su`、`getGoalLineForLoc()`) を横切ったときも1加算する。競技規則のスタートの項から、コントロールラインはスタートライン (ピットレーン) の 2 m 先の計測ラインを延ばした本コース上とし、本コースだけを横切る。ゴールラインはピット入口の道だけを横切る。もてぎはゴールラインがコントロールラインと同じ並びにあり、コントロールラインをゴールレーンまで延ばしてあるので、ゴールラインは持たない
  - 鈴鹿の3本の線 (スタートライン `startline_*_su` は記録用で判定には使わない) は、2026 年の「競技会の流れ」の図・自チームのピットインの軌跡・他チームの車載動画から推定した仮の位置。2027 年の大会当日のコースウォークで確定させる (ECU リポジトリの `document/suzuka_strategy_2027.md`)。2024〜2026 年の自チームのログの軌跡に通すと、走行中のコントロールラインの通過は走った周回と、ゴールラインの通過は練習走行のピットインと一致した (発進前・発進直後の通過は、周回開始ガードと走行距離ガードで数えない)
- 誤検出防止:
  - デバウンス: 直前のカウントから `LAP_CROSS_DEBOUNCE_MS` (既定 10秒) 未満の再交差は無視 (スタート/フィニッシュ付近での低速走行・停止時のGPSジッタによる多重カウント防止)
  - テレポートガード: 前回位置からの概算移動距離が `LAP_CROSS_TELEPORT_GUARD_M` (既定 100m) を超える場合は判定をスキップ (GNSS受信断からの復帰など、位置が飛んだ際の誤検出防止)
  - GNSS未固定 (`gps.location.isValid()`が偽) の測位は前回位置の更新・交差判定の対象から除外
  - 走行距離ガード (鈴鹿・もてぎ、ECU受信中のみ): 前回カウントからの ECU 走行距離が 1 周の半分に満たない交差は数えない。グリッドがラインの手前にあっても発進直後の通過を 1 周と数えない。ECU の走行距離が途中で 0 に戻った場合は、次のカウントまでこの判定を行わない
  - 周回開始ガード: ECU の走行時間が 0 のあいだ (周回開始前) の交差は数えない。走行開始前の移動中にラインをまたいでも周回数は進まない。ECU からの受信が一度も無いまま走ると周回数は 0 のままになる。ポータブル仕様 (`m5stack-basic-portable`) は ECU を繋がず走行時間が動かないので、このガードを外して常時数える (`LAP_COUNT_BEFORE_START`)
  - 発進時のリセット: ECU の走行時間が 0 から動き出した時点で周回数を 0 に戻す (前の走行の周回数や、ポータブル仕様で発進前にラインをまたいで数えた分を走行に持ち込まない)
- カウントした時点の走行時間を通過タイムとして走行支援へ渡す
- 残り周回は 0 で止まる (周回数が規定を超えても 255 にならない)

## ログファイル仕様 (SD)

ヘッダ: `記録日時,速度(km/h),ラップ数(周目),走行時間(秒),回転数(rpm),燃料噴射時間(ms),点火進角角度(CA),燃料噴射終了角度(CA),走行距離積算(m),積算燃料消費量(ml),燃費(km/l),lat(緯度),lng(経度),標高(m),場所,温度(C),気圧(kPa),気温(C),湿度(%),1次空気圧(MPa),2次空気圧(MPa),燃圧(MPa),GNSS測位経過(ms),GNSS受信文字数,GNSS正常文数,GNSSエラー文数,GNSS UART再初期化(回),GNSS再起動(回)` (28列)

- 記録日時: GNSS/NTP同期時刻を基準にmillis()で連続算出 + JST補正 (`YYYY/M/D hh:mm:ss.cc`)。SDログ行は10Hzでも重複しない (厳密単調増加)
- `alt`: 気圧センサー高度 + 国土地理院APIで確定したオフセット
- `loc`: サーキット判定結果 (`su` / `mo` / `to`)
- ラップ数(周目): 1始まり (発進〜1回目のライン通過までが `1`)。内部の `Lapcount` (完了周回数) に1を足して記録する。MQTT の `Lapcount` は完了周回数のまま
- 走行時間: ECU送信の積算秒 (0.1秒単位、小数1桁。例 `267.3`)。LCD表示・MQTT送信は整数秒
- 燃料噴射時間(ms) / 点火進角角度(CA) / 燃料噴射終了角度(CA): ECU送信値。ECU無信号時は0 (燃料噴射時間は小数1桁)
- 燃費: コード中 `dispergas` (km/L 指定の閾値 2000 スケールバー)
- lat(緯度) / lng(経度): 最後に測位した位置。測位が途切れても前の値のまま書くので、新しさは `GNSS測位経過(ms)` で見る
- GNSS の診断列 (受信が止まったときに、どこで止まったかを見分ける):
  - `GNSS測位経過(ms)`: 最後に測位してからの時間 (一度も測位していなければ空欄)。測位が続いていれば数百 ms 以内に収まる
  - `GNSS受信文字数` / `GNSS正常文数` / `GNSSエラー文数`: 起動からの累計 (TinyGPS++ の `charsProcessed` / `passedChecksum` / `failedChecksum`)
  - `GNSS UART再初期化(回)` / `GNSS再起動(回)`: 自動復帰で Serial2 を開き直した回数 / モジュールへホットスタートを送った回数 (起動からの累計)
  - 読み方: 受信文字数が増えない → モジュールから何も届いていない。文字数は増えるが正常文数が増えない → ボーレート違いや配線のノイズ。正常文数は増えるが測位経過が伸びる → 文は届くが測位できていない
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
  - 気圧: Unit ENV-III の QMP6988 (見つからなければ BME280)
  - 気温/湿度: Unit ENV-III の SHT30 (見つからなければ BME280)
  - Unit ENV-III のドライバは `include/UnitEnvIII.h`。loop() を止めないよう、SHT30 は前回始めた測定の結果を読んで次の測定を始める (値は1秒前のもの)
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

## secrets.h 管理

**認証情報は Git に含めないでください。**  
`.gitignore` には既に `secrets.h` が登録されています。  
以下の変数を `src/secrets.h` に定義します (実際の値は環境に合わせて置換):

```cpp
const char* mqtt_server;      // AWS IoT Core エンドポイント (xxxxxxxxx-ats.iot.<region>.amazonaws.com)
const int   mqtt_port;        // 8883 推奨 (TLS)
const char* mqtt_topic;       // 例: "Furoshiki/M5Logger"
const char* mqtt_deviceID;    // Thing Name / クライアントID
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
