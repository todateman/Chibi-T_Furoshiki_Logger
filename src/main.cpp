#include <Arduino.h>
#include <TinyGPS++.h>
#include "SdFat.h"
#include "sdios.h"
#include <M5Unified.h>
#include "Ambient.h"
#include <WiFiManager.h>
#include <TimeLib.h>
#include <WiFiClientSecure.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <string.h>
#include <Adafruit_BME280.h>
#include "StrategyAssist.h"
#include "secrets.h"

//==================== 定数・マクロ ====================
#define pi 3.141592653589793
#define SD_SPI_SPEED SD_SCK_MHZ(25)
#define SD_CONFIG SdSpiConfig(GPIO_NUM_4, SHARED_SPI, SD_SPI_SPEED)

// ECU用ピンとボーレート（M5Stack Basicの起動不良対策でGPIO12の使用禁止）
#if defined(ARDUINO_M5STACK_Core2)
constexpr uint8_t ECU_RX_PIN = 2;
constexpr uint8_t ECU_TX_PIN = 0;
#elif defined(ARDUINO_M5Stack_Core_ESP32)
constexpr uint8_t ECU_RX_PIN = 15;
constexpr uint8_t ECU_TX_PIN = 0;
#else
#error "Unsupported board: ECU_UART_PIN is not defined for this target"
#endif
#define ECU_BPS 115200

// BLE中継機(M5NanoC6)とのI2C通信仕様（NanoC6側がI2Cスレーブ, addr=0x08で動作）
#if defined(ARDUINO_M5STACK_Core2)    // NanoC6(BLE中継機)接続用Port AのI2Cピン（M5Stack Core2）
constexpr uint8_t BLE_I2C_SDA = 32;
constexpr uint8_t BLE_I2C_SCL = 33;
#elif defined(ARDUINO_M5Stack_Core_ESP32)   // NanoC6(BLE中継機)接続用Port AのI2Cピン（M5Stack Basic 気圧センサー用Wireバスと同一ピン）
constexpr uint8_t BLE_I2C_SDA = 21;
constexpr uint8_t BLE_I2C_SCL = 22;
#else
#error "Unsupported board: BLE_I2C_SDA/SCL is not defined for this target"
#endif
#define BLE_I2C_SLAVE_ADDR 0x08
#define BLE_I2C_FRAME_SIZE 32
#define BLE_I2C_CMD_ENGINE_TEMP 0x01  // エンジン温度（Chibi-T_Furoshiki_Heater経由）
#define BLE_I2C_CMD_PRI_PRE  0x02  // 1次側空気圧（AutoAirAdjust経由）
#define BLE_I2C_CMD_SEC_PRE  0x03  // 2次側空気圧（AutoAirAdjust経由）
#define BLE_I2C_CMD_FUEL_PRE 0x04  // 燃圧（AutoAirAdjust経由）
#define BLE_I2C_POLL_INTERVAL 200  // NanoC6への要求間隔（ミリ秒）

// 各タスクの更新間隔（ミリ秒）
#define SERIAL_OUT_INTERVAL 100
#define SD_LOG_INTERVAL     100
#define MQTT_INTERVAL_PRE   10000  // 走行前
#define MQTT_INTERVAL_RUN   1000   // 走行中
#define MQTT_RECONNECT_BASE_INTERVAL 5000UL
#define MQTT_RECONNECT_MAX_INTERVAL  60000UL
#define AMBIENT_INTERVAL    10000
#define ALTITUDE_INTERVAL   500
#define ENV_SENSOR_INTERVAL 1000
#define DISPLAY_INTERVAL    100
#define ELEVATION_OFFSET_FETCH_RETRY_INTERVAL 15000UL
#define LAP_CROSS_DEBOUNCE_MS 10000UL       // ラップカウントの多重カウント防止用クールダウン時間
#define LAP_CROSS_TELEPORT_GUARD_M 100.0    // GPSロスト後の誤検出防止用の最大移動距離しきい値[m]

// GNSS受信バッファサイズ（1ループあたりの最大パース量。10Hzロギングでの取りこぼし防止のため広めに確保）
#define GNSS_PARSE_BUDGET_BYTES 512

// ECU/SDログ処理の高速化用バッファ・しきい値（10Hzロギング対応）
#define ECU_LINE_BUFFER_SIZE 128
#define SD_LINE_BUFFER_SIZE 256
#define SD_BATCH_BUFFER_SIZE 1024
#define SD_SYNC_LINE_THRESHOLD 40
#define SD_SYNC_TIME_MS 5000UL

// 走行支援（加速開始位置・加速停止速度・通過タイムの合図）
#ifndef STRATEGY_SIM
#define STRATEGY_SIM 0  // 机上確認用の仮想走行（0:無効, 1以上:時間の倍率）。本番は必ず0
#endif
#define STRATEGY_FILE_BUFFER_SIZE 1536        // 走行支援の設定ファイル(JSON)の最大サイズ
#define STRATEGY_POSITION_MAX_AGE_MS 3000UL   // GNSS測位がこれより古ければ位置不明として扱う
#define STRATEGY_OFF_COURSE_M 60.0            // ウェイポイントを結んだ線からこれ以上離れていればコース外として扱う
#define STRATEGY_BEEP_REPEAT_MS 2000UL        // 「加速」「停止」が続いている間のブザーの繰り返し間隔
// ブザーはM5Stack Basicのみ。Core2は内蔵スピーカのI2S(GPIO0/2)がECU用UARTと同じピンのため鳴らさない
#if defined(ARDUINO_M5Stack_Core_ESP32)
  #define STRATEGY_BEEP_AVAILABLE 1
#else
  #define STRATEGY_BEEP_AVAILABLE 0
#endif

// MQTT設定
#define MQTT_BUFFER_SIZE  512 // MQTT送受信のバッファサイズ
#define MQTT_DIAGNOSTIC_LOG 0  // 本番:0, 切り分け時のみ1
#define BLE_I2C_DIAGNOSTIC_LOG 0  // 本番:0, BLE I2C切り分け時のみ1

// PROGMEMに格納する定数文字列
const char MSG_WIFI_CONFIG[] PROGMEM = "このアクセスポイントに接続して\nWi-Fiの設定をしてください\nSSID: ";
const char MSG_WIFI_CONNECTING[] PROGMEM = "Wi-Fi接続中...";
const char MSG_NO_SD[] PROGMEM = "MicroSDが見つかりません";
const char MSG_LOG_DIR_CREATE[] PROGMEM = "ログディレクトリ作成中...";
const char MSG_LOG_FILE_CREATE[] PROGMEM = "ログファイル作成中...";
const char MSG_LOADING[] PROGMEM = "読み込み中...";

//==================== グローバル変数 =====================

// M5とLCD
static M5GFX lcd;
static LGFX_Sprite lcd_s(&lcd);

// SDカード用
SdFat sd;
#if SDFAT_FILE_TYPE == 0
  typedef File file_t;
#elif SDFAT_FILE_TYPE == 1
  typedef File32 file_t;
#elif SDFAT_FILE_TYPE == 2
  typedef ExFile file_t;
#elif SDFAT_FILE_TYPE == 3
  typedef FsFile file_t;
#else
  #error Invalid SDFAT_FILE_TYPE
#endif
file_t logFile;
bool LOGGING = true;
char fileName[20];  // ログファイル名（例: /LOG/LOG0000.CSV）
int fileNum = 0;    // ログファイル番号（例: LOG0000.CSVの0000部分）
bool logFileInitialized = false;  // ログファイルが初期化されているか（ヘッダ書き込み済みか）
const char NEXT_LOG_INDEX_FILE[] = "/LOG/NEXTID.TXT";  // 次回ログファイル番号を保存するファイル

// WiFi, MQTT, Ambient
WiFiManager wifiManager;
WiFiClient ambientClient;
WiFiClientSecure mqttTlsClient;
PubSubClient mqttclient(mqttTlsClient);
Ambient ambient;
bool isWifiConfigSucceeded = false;
bool ambientpush = false;   // Ambient送信有効（必要に応じてfalseに設定）
bool MQTTpush = true;       // MQTT送信有効
char devKey[20];
unsigned int channelId;
char writeKey[20];

// タイマー用
unsigned long t_Serial = 0;
unsigned long t_SD     = 0;
unsigned long t_MQTT   = 0;
unsigned long t_amb    = 0;
unsigned long t_alt    = 0;
unsigned long t_env    = 0;
unsigned long t_display = 0;
unsigned long lastSdSyncAt = 0;
uint16_t sdLinesSinceSync = 0;
char sdBatchBuffer[SD_BATCH_BUFFER_SIZE];
size_t sdBatchLen = 0;

// シリアル（ECU, GNSS）
unsigned long receiveECUtime = 0;

// BLE中継機(NanoC6)用I2Cバスの選択
// Port AのSDA/SCLが気圧センサー用Wireバスと同一ピンの機種（M5Stack Basic）は
// Wireバスを共用し、異なる機種（M5Stack Core2）は専用バスWire1を使う
#if defined(ARDUINO_M5Stack_Core_ESP32)
  #define BleI2C Wire
#else
  #define BleI2C Wire1
#endif

// ECU受信データ
uint16_t tachoRpm = 0;  // エンジン回転数 [rpm]
float INJ_timems = 0.0; // 燃料噴射時間 [ms]（燃料噴射量の指標として利用）
uint8_t IGN_CA = 0;     // 点火時期 [°CA]（クランク角度）
int16_t injEndCA = 0;    // 燃料噴射終了角 [°CA]（クランク角度）
float speed = 0.0;      // 車軸パルスから算出した車速 [km/h]
uint16_t distance = 0;  // 走行距離 [m]
float gasml = 0.0;      // 燃料消費量 [ml]（燃料噴射時間から推定）※あくまで目安で、実際の消費量とは異なる可能性が高い
float dispergas = 0.0;  // 燃料消費率 [ml/km]（燃料消費量 / 走行距離）※あくまで目安で、実際の消費率とは異なる可能性が高い
float worktime = 0.0f;  // 走行時間 [s]（エンジン始動以降の時間を累積。ECUから0.1秒単位で受信）
uint8_t Lapcount = 0;   // 現在の周回数
uint8_t totallaps = 3;  // 規定周回数（サーキットごとに設定値を上書き）
uint16_t goal = 1000;   // 走行距離 [m]（サーキットごとに設定値を上書き）
uint16_t limittime = 100; // 制限時間 [s]（サーキットごとに設定値を上書き）
float EngTemp = 0.0;    // エンジン温度 [°C]（Chibi-T_Furoshiki_Heater経由）
float PriPre  = 0.0;    // 1次側空気圧 [MPa]（AutoAirAdjust経由）
float SecPre  = 0.0;    // 2次側空気圧 [MPa]（AutoAirAdjust経由）
float FuelPre = 0.0;    // 燃圧 [MPa]（AutoAirAdjust経由）

// GPS用
TinyGPSPlus gps;
int gnss_bps = 115200;  // GNSSモジュールのボーレート（モジュールに応じて切り替える NEO-6M:115200, M5Stack GNSS Module:38400）
double la = 0.0, ln = 0.0;      // GPS緯度経度
// double la = 34.990768;    // KMMF2026の緯度経度初期値
// double ln = 137.010875;   // KMMF2026の緯度経度初期値
double alt = 0.0;   // GPS高度
double spd = 0.0;   // GPS速度
String Loc = "";    // ロケーション識別子（"su":鈴鹿, "mo":茂木, "to":豊田）

// BME280による高度推定用
Adafruit_BME280 bme280;
bool isBmx280Ready = false;     // BME280が正常に初期化されているかどうか
float envPressureKPa = NAN;                       // BME280から取得した環境気圧 [kPa]（利用できない場合はNAN）
float envTemperatureC = NAN;                      // BME280から取得した環境温度 [°C]（利用できない場合はNAN）
float envHumidityPct = NAN;                       // BME280から取得した環境湿度 [%]（利用できない場合はNAN）
float seaLevelPressureKPa = 101.325f;             // 海面上気圧の初期値（kPa単位）
float altitudeOffsetMeters = 0.0f;                // 国土地理院API標高とBME280生高度の差分（m）
bool altitudeOffsetFixed = false;                 // 標高オフセットが確定しているかどうか
unsigned long nextAltitudeOffsetFetchAt = 0;      // 次回の標高オフセット取得を試みる時刻（ミリ秒）

// サーキットごとの設定
const uint8_t totallaps_su = 8; // 鈴鹿サーキット東コースの規定周回数
const uint8_t totallaps_mo = 7; // ツインリンクもてぎオーバルコースの規定周回数
const float controlline_la1_su = 34.845093;  // 鈴鹿サーキット東コースのコントロールライン(外側)の緯度
const float controlline_ln1_su = 136.538735;  // 鈴鹿サーキット東コースのコントロールライン(外側)の経度
const float controlline_la2_su = 34.844847;  // 鈴鹿サーキット東コースのコントロールライン(内側)の緯度
const float controlline_ln2_su = 136.538379;  // 鈴鹿サーキット東コースのコントロールライン(内側)の経度
const float controlline_la1_mo = 36.532698;  // ツインリンクもてぎオーバルコースのコントロールライン(外側)の緯度
const float controlline_ln1_mo = 140.226047;  // ツインリンクもてぎオーバルコースのコントロールライン(外側)の経度
const float controlline_la2_mo = 36.533111;  // ツインリンクもてぎオーバルコースのコントロールライン(内側)の緯度
const float controlline_ln2_mo = 140.226982;  // ツインリンクもてぎオーバルコースのコントロールライン(内側)の経度
const float controlline_la1_to = 35.082069;  // 豊田市SENTANのコントロールライン(外側)の緯度
const float controlline_ln1_to = 137.159997;  // 豊田市SENTANのコントロールライン(外側)の経度
const float controlline_la2_to = 35.082060;  // 豊田市SENTANのコントロールライン(内側)の緯度
const float controlline_ln2_to = 137.160227;  // 豊田市SENTANのコントロールライン(内側)の経度
const uint16_t goal_su = 17616; // 鈴鹿サーキット東コースの走行距離 [m]
const uint16_t goal_mo = 16389; // ツインリンクもてぎオーバルコースの走行距離 [m]
const uint16_t limittime_su = 2536; // 鈴鹿サーキット東コースの制限時間 [s]
const uint16_t limittime_mo = 2360; // ツインリンクもてぎオーバルコースの制限時間 [s]
const int time_offset = 9;  // JST

// 時刻表示用バッファ
char datetime[23];
uint64_t lastDatetimeCentis = 0;  // CSV時刻の逆行防止（1/100秒単位）
uint64_t lastSdCentis = 0;        // SDログ行の時刻重複防止（1/100秒単位）

// 時刻の基準クロック: 壁時計[1/100秒] = millis()/10 + clockOffsetCentis
// GNSS/NTP同期時にアンカーし、以後はmillis()から連続的に算出する（呼び出し経路ごとの値のズレを防ぐ）
int64_t clockOffsetCentis = 0;
bool clockAnchored = false;
const int64_t CLOCK_REANCHOR_THRESHOLD_CENTIS = 100;  // 推定値とのずれがこの値(1秒)を超えたときのみ再アンカー

// NTP同期フラグ（GPS受信後は更新しない）
bool ntpSyncDone = false;
unsigned long nextNtpRetryAt = 0;

// 時刻同期で受け入れる年の下限（RTC初期値の1999年などを除外）
const int VALID_TIME_YEAR_MIN = 2024;
const unsigned long NTP_SYNC_TIMEOUT_MS = 30000UL;
const unsigned long NTP_RETRY_INTERVAL_MS = 60000UL;

// 時刻同期で受け入れる年の範囲内かどうかを判定する関数
bool isValidSyncYear(int year) {
  return year >= VALID_TIME_YEAR_MIN && year <= 2099;
}

// 基準クロックの現在値（1/100秒）を返す
uint64_t clockNowCentis() {
  return static_cast<uint64_t>(static_cast<int64_t>(millis() / 10UL) + clockOffsetCentis);
}

// 基準クロックを同期時刻(JST)にアンカーする
// 未アンカー、または推定値とのずれが1秒を超えたときのみ更新し、小さなずれでは動かさない
void anchorClock(time_t jstSec, uint8_t csec) {
  int64_t targetCentis = static_cast<int64_t>(jstSec) * 100LL + static_cast<int64_t>(csec);
  int64_t newOffset = targetCentis - static_cast<int64_t>(millis() / 10UL);
  int64_t diff = newOffset - clockOffsetCentis;
  if (!clockAnchored || diff > CLOCK_REANCHOR_THRESHOLD_CENTIS || diff < -CLOCK_REANCHOR_THRESHOLD_CENTIS) {
    clockOffsetCentis = newOffset;
    clockAnchored = true;
  }
}

// 日時バッファ更新関数の宣言
void refreshDatetime();

// NTPサーバーからの時刻同期を試みる関数（成功した場合はtrueを返す）
bool trySyncTimeFromNtp(unsigned long timeoutMs) {
  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }

  // JST固定で複数NTPサーバーを指定して取得成功率を上げる
  configTzTime("JST-9", "ntp.nict.jp", "time.google.com", "pool.ntp.org");
  Serial.println("NTP sync started");

  unsigned long ntpStart = millis();
  while (millis() - ntpStart < timeoutMs) {
    time_t epoch = time(nullptr);
    struct tm timeinfo;
    if (localtime_r(&epoch, &timeinfo) != nullptr) {
      int calendarYear = timeinfo.tm_year + 1900;
      if (isValidSyncYear(calendarYear)) {
        ntpSyncDone = true;
        // TimeLib側はJST基準で扱う（GNSS同期経路と同じ基準）
        time_t jstEpoch = static_cast<time_t>(epoch) + static_cast<time_t>(time_offset * SECS_PER_HOUR);
        setTime(jstEpoch);
        anchorClock(jstEpoch, 0);
        refreshDatetime();
        Serial.printf("NTP sync succeeded: %04d/%02d/%02d %02d:%02d:%02d\n",
                      calendarYear,
                      timeinfo.tm_mon + 1,
                      timeinfo.tm_mday,
                      timeinfo.tm_hour,
                      timeinfo.tm_min,
                      timeinfo.tm_sec);
        return true;
      }
    }

    M5.update();
    delay(100);
  }

  Serial.println("NTP sync timeout");
  return false;
}

// 1/100秒単位の時刻をdatetimeバッファへ整形する
void formatDatetime(uint64_t centis) {
  tmElements_t tm;
  breakTime(static_cast<time_t>(centis / 100ULL), tm);

  sprintf_P(datetime, PSTR("%d/%d/%d %02d:%02d:%02d.%02d"),
            tmYearToCalendar(tm.Year), tm.Month, tm.Day, tm.Hour, tm.Minute, tm.Second,
            static_cast<unsigned int>(centis % 100ULL));
}

// 日時バッファ更新
// 同期済みなら基準クロック(millis由来)から算出し、呼び出し元によらず同じ時間軸で値を返す
void refreshDatetime() {
  uint64_t currentCentis;
  if (clockAnchored) {
    currentCentis = clockNowCentis();
  } else {
    // 未同期時のフォールバック: 現在秒の先頭からの経過時間でセンチ秒を算出する
    static time_t centiBaseSecond = 0;
    static unsigned long centiBaseMillis = 0;
    time_t currentSecond = now();
    unsigned long nowMs = millis();
    uint8_t csec = 0;
    if (currentSecond != centiBaseSecond) {
      centiBaseSecond = currentSecond;
      centiBaseMillis = nowMs;
    } else {
      unsigned long elapsedMs = nowMs - centiBaseMillis;
      csec = static_cast<uint8_t>(elapsedMs > 990 ? 99 : elapsedMs / 10);
    }
    currentCentis = static_cast<uint64_t>(currentSecond) * 100ULL + csec;
  }

  // 再アンカー等で時刻が戻った場合でも、ログ時刻文字列は単調増加を維持する
  // NOTE: ここで setTime() は呼ばない。呼ぶと同一秒内の多重呼び出しで秒が人工的に進み、
  //       CSV時刻のバースト/空白を生むため。
  if (currentCentis < lastDatetimeCentis) {
    currentCentis = lastDatetimeCentis;
  }
  lastDatetimeCentis = currentCentis;

  formatDatetime(currentCentis);
}

// GNSS UTC日時をJSTへ変換したtime_tを作成する
bool buildGnssJstTime(time_t& outJstTime) {
  if (!gps.date.isValid() || !gps.time.isValid()) {
    return false;
  }

  int gpsYear = gps.date.year();
  if (!isValidSyncYear(gpsYear)) {
    return false;
  }

  tmElements_t tm;
  tm.Year = CalendarYrToTm(gpsYear);
  tm.Month = gps.date.month();
  tm.Day = gps.date.day();
  tm.Hour = gps.time.hour();
  tm.Minute = gps.time.minute();
  tm.Second = gps.time.second();

  time_t utc = makeTime(tm);
  outJstTime = utc + static_cast<time_t>(time_offset * SECS_PER_HOUR);
  return true;
}

// GNSSでの再同期はforward-onlyで行い、過剰な補正を抑制する
void updateSystemTimeFromGnss() {
  time_t gnssJst = 0;
  if (!buildGnssJstTime(gnssJst)) {
    return;
  }

  time_t current = now();
  if (current < 1577836800 || gnssJst > (current + 2)) {  // currentが未初期化または2秒以上先行時のみ補正
    setTime(gnssJst);
    ntpSyncDone = true;
  }
}

// ディスプレイ表示モード
uint8_t dispmode = 0;

// ウェイポイント（標高グラフ用）
struct Waypoint {
  uint16_t id;
  float lat;
  float lng;
  float alt;
  float dist;  // 先頭ウェイポイントからの距離 [m]（CSVに距離列が無い場合はNAN）
};

// ウェイポイントデータの最大数（必要に応じて増減させる）
const size_t MAX_WAYPOINTS = 300;
Waypoint waypoints[MAX_WAYPOINTS];
size_t waypointCount = 0;
float waypointMinAlt = 0.0f;
float waypointMaxAlt = 0.0f;
String loadedWaypointLoc = "";
int nearestWaypointIndex = -1;
float waypointRawLapMeters = 0.0f;  // ウェイポイントCSV上の1周の距離 [m]（距離列が無い場合は0）

// 走行支援
StrategyAssist::Assist strategy;
String loadedStrategyLoc = "";
enum StrategySource : uint8_t { STRATEGY_SRC_NONE, STRATEGY_SRC_SD, STRATEGY_SRC_SD_ERROR };
StrategySource strategySource = STRATEGY_SRC_NONE;  // 設定ファイルの読み込み結果（無し / 読み込み済み / 内容に異常）
bool positionFresh = false;         // 現在位置が新しい測位に基づいているか
uint32_t positionAgeMs = 0;         // 現在位置の測位からの経過時間 [ms]
uint16_t lapCountDistanceBase = 0;  // 前回の周回カウント時のECU走行距離 [m]
bool lapCountDistanceBaseValid = true;

// 2点間の概算距離 [m]（サーキット内の短距離用）
double approxDistanceMeters(double lat1, double lng1, double lat2, double lng2) {
  double dLat = (lat2 - lat1) * 111320.0;
  double dLng = (lng2 - lng1) * 111320.0 * cos(lat1 * pi / 180.0);
  return sqrt(dLat * dLat + dLng * dLng);
}

// ロケーション識別子に対応するウェイポイントCSVファイルのパスを返す関数
const char* getWaypointFilePathByLoc(const String& loc) {
  if (loc == "su") {
    return "/suzuka_waypoint.csv";
  }
  if (loc == "mo") {
    return "/motegi_waypoint.csv";
  }
  return "/toyota_waypoint.csv";
}

// CSVの1行をパースしてWaypoint構造体に変換する関数
bool parseWaypointCsvLine(const char* line, Waypoint& outPoint) {
  int id = 0;
  float latVal = 0.0f;
  float lngVal = 0.0f;
  float altVal = 0.0f;
  float distVal = 0.0f;
  int parsed = sscanf(line, "%d,%f,%f,%f,%f", &id, &latVal, &lngVal, &altVal, &distVal);
  if (parsed < 4 || id <= 0) {
    return false;
  }

  outPoint.id = static_cast<uint16_t>(id);
  outPoint.lat = latVal;
  outPoint.lng = lngVal;
  outPoint.alt = altVal;
  outPoint.dist = (parsed >= 5) ? distVal : NAN;
  return true;
}

// ロケーション識別子に対応するウェイポイントCSVファイルをSDカードから読み込む関数
bool loadWaypointFileForLoc(const String& loc) {
  waypointRawLapMeters = 0.0f;
  if (!LOGGING) {
    waypointCount = 0;
    nearestWaypointIndex = -1;
    loadedWaypointLoc = "";
    return false;
  }

  const char* path = getWaypointFilePathByLoc(loc);
  file_t waypointFile = sd.open(path, O_READ);
  if (!waypointFile) {
    Serial.printf("[Waypoint] open failed: %s\n", path);
    waypointCount = 0;
    nearestWaypointIndex = -1;
    loadedWaypointLoc = "";
    return false;
  }

  char line[128];
  size_t loadedCount = 0;
  float minAlt = 1000000.0f;
  float maxAlt = -1000000.0f;
  while (waypointFile.available() && loadedCount < MAX_WAYPOINTS) {
    int len = waypointFile.readBytesUntil('\n', line, sizeof(line) - 1);
    if (len <= 0) {
      continue;
    }
    line[len] = '\0';
    if (line[len - 1] == '\r') {
      line[len - 1] = '\0';
    }

    if (line[0] == '\0' || (line[0] >= 'A' && line[0] <= 'Z') || (line[0] >= 'a' && line[0] <= 'z')) {
      continue;  // ヘッダや空行を読み飛ばす
    }

    Waypoint point;
    if (!parseWaypointCsvLine(line, point)) {
      continue;
    }

    waypoints[loadedCount] = point;
    if (point.alt < minAlt) {
      minAlt = point.alt;
    }
    if (point.alt > maxAlt) {
      maxAlt = point.alt;
    }
    loadedCount++;
  }
  waypointFile.close();

  waypointCount = loadedCount;
  nearestWaypointIndex = -1;
  if (waypointCount == 0) {
    loadedWaypointLoc = "";
    return false;
  }

  if (fabsf(maxAlt - minAlt) < 0.1f) {
    minAlt -= 1.0f;
    maxAlt += 1.0f;
  }
  waypointMinAlt = minAlt;
  waypointMaxAlt = maxAlt;
  loadedWaypointLoc = loc;

  // 1周の距離 = 末尾までの距離 + 末尾から先頭へ戻る距離（走行支援の周内位置の算出に使う）
  const Waypoint& first = waypoints[0];
  const Waypoint& last = waypoints[waypointCount - 1];
  bool hasDistance = waypointCount >= 2 && last.dist > 0.0f;
  for (size_t i = 0; i < waypointCount && hasDistance; i++) {
    hasDistance = isfinite(waypoints[i].dist);
  }
  if (hasDistance) {
    waypointRawLapMeters = last.dist + static_cast<float>(approxDistanceMeters(last.lat, last.lng, first.lat, first.lng));
  }
  Serial.printf("[Waypoint] loaded %u points from %s\n", static_cast<unsigned int>(waypointCount), path);
  return true;
}

// 現在のロケーションに対応するウェイポイントがロードされていない場合にロードする関数
void ensureWaypointLoadedForCurrentLoc() {
  if (Loc.length() == 0) {
    return;
  }
  if (loadedWaypointLoc == Loc && waypointCount > 0) {
    return;
  }
  loadWaypointFileForLoc(Loc);
}

// 現在位置に最も近いウェイポイントのインデックスを返す関数
int findNearestWaypointIndex(double lat, double lng) {
  if (waypointCount == 0) {
    return -1;
  }

  int nearest = 0;
  double minDist2 = 1e18;
  for (size_t i = 0; i < waypointCount; i++) {
    double dLat = lat - static_cast<double>(waypoints[i].lat);
    double dLng = lng - static_cast<double>(waypoints[i].lng);
    double dist2 = dLat * dLat + dLng * dLng;
    if (dist2 < minDist2) {
      minDist2 = dist2;
      nearest = static_cast<int>(i);
    }
  }

  return nearest;
}

// 標高値をグラフのY座標に変換する関数
int altitudeToGraphY(float value, float minAlt, float maxAlt, int top, int height) {
  float normalized = (value - minAlt) / (maxAlt - minAlt);
  normalized = constrain(normalized, 0.0f, 1.0f);
  return top + height - 1 - static_cast<int>(normalized * static_cast<float>(height - 1));
}

// 標高グラフの描画
void drawAltitudeGraphMode() {
  lcd_s.setFont(&fonts::lgfxJapanGothicP_16);
  lcd_s.setTextSize(1);
  lcd_s.setTextDatum(TL_DATUM);
  lcd_s.setTextColor(TFT_WHITE);

  const int graphLeft = 10;
  const int graphTop = 20;
  const int graphWidth = 300;
  const int graphHeight = 180;

  lcd_s.drawRect(graphLeft, graphTop, graphWidth, graphHeight, TFT_WHITE);
  if (waypointCount < 2) {
    lcd_s.drawString("ウェイポイント未読込", 32, 108);
    return;
  }

  for (size_t i = 0; i + 1 < waypointCount; i++) {
    size_t next = i + 1;  // 右端と左端はつながない

    int x1 = graphLeft + static_cast<int>((static_cast<float>(i) / static_cast<float>(waypointCount - 1)) * static_cast<float>(graphWidth - 1));
    int y1 = altitudeToGraphY(waypoints[i].alt, waypointMinAlt, waypointMaxAlt, graphTop, graphHeight);
    int x2 = graphLeft + static_cast<int>((static_cast<float>(next) / static_cast<float>(waypointCount - 1)) * static_cast<float>(graphWidth - 1));
    int y2 = altitudeToGraphY(waypoints[next].alt, waypointMinAlt, waypointMaxAlt, graphTop, graphHeight);
    lcd_s.drawLine(x1, y1, x2, y2, TFT_CYAN);
  }

  int plotIndex = nearestWaypointIndex;
  if (plotIndex >= 0) {
    int x = graphLeft + static_cast<int>((static_cast<float>(plotIndex) / static_cast<float>(waypointCount - 1)) * static_cast<float>(graphWidth - 1));
    int yWaypoint = altitudeToGraphY(waypoints[plotIndex].alt, waypointMinAlt, waypointMaxAlt, graphTop, graphHeight);
    int yNow = altitudeToGraphY(static_cast<float>(alt), waypointMinAlt, waypointMaxAlt, graphTop, graphHeight);

    lcd_s.drawFastVLine(x, graphTop, graphHeight, TFT_DARKGREY);
    lcd_s.fillCircle(x, yWaypoint, 3, TFT_YELLOW);
    lcd_s.fillCircle(x, yNow, 4, TFT_RED);

    lcd_s.setFont(&fonts::lgfxJapanGothicP_12);
    lcd_s.setTextDatum(TL_DATUM);
    lcd_s.setTextColor(TFT_WHITE);
    lcd_s.setCursor(8, 225);
    lcd_s.printf("WP:%d/%u route:%.1fm now:%.1fm", waypoints[plotIndex].id,
                 static_cast<unsigned int>(waypointCount),
                 waypoints[plotIndex].alt,
                 static_cast<float>(alt));
  }

  lcd_s.setFont(&fonts::lgfxJapanGothicP_12);
  lcd_s.setTextDatum(TL_DATUM);
  lcd_s.setCursor(graphLeft, graphTop - 14);
  lcd_s.printf("max %.1fm", waypointMaxAlt);
  lcd_s.setCursor(graphLeft, graphTop + graphHeight + 2);
  lcd_s.printf("min %.1fm", waypointMinAlt);
}

//==================== 走行支援 =====================

// ロケーション識別子に対応する走行支援の設定ファイル(JSON)のパスを返す関数
const char* getStrategyFilePathByLoc(const String& loc) {
  if (loc == "su") {
    return "/suzuka_strategy.json";
  }
  if (loc == "mo") {
    return "/motegi_strategy.json";
  }
  return "/toyota_strategy.json";
}

// 加速パターン1つぶんをJSONから読み込む（指定された項目だけ上書きする）
void readStrategyPatternJson(JsonVariantConst src, StrategyAssist::Pattern& pattern) {
  if (src["v_off"].is<float>()) {
    pattern.vOffKmh = src["v_off"].as<float>();
  }
  JsonArrayConst marks = src["marks_m"];
  if (!marks.isNull()) {
    pattern.markCount = 0;
    for (JsonVariantConst mark : marks) {
      if (pattern.markCount >= StrategyAssist::MAX_MARKS) {
        break;
      }
      pattern.marksM[pattern.markCount++] = mark.as<float>();
    }
    // 目印を差し替えて最終周の指定が無い場合は、最終周も全目印を有効にする（加速が多い側＝完走側）
    pattern.finalLapMarks = pattern.markCount;
  }
  if (src["final_lap_marks"].is<int>()) {
    pattern.finalLapMarks = static_cast<uint8_t>(constrain(src["final_lap_marks"].as<int>(), 0, static_cast<int>(pattern.markCount)));
  }
}

// 加速パターンの値が妥当かどうかを判定する関数
bool isValidStrategyPattern(const StrategyAssist::Pattern& pattern, float lapM, float guardKmh) {
  if (pattern.markCount == 0 || pattern.vOffKmh <= guardKmh || pattern.vOffKmh > 60.0f) {
    return false;
  }
  for (uint8_t i = 0; i < pattern.markCount; i++) {
    if (!isfinite(pattern.marksM[i]) || pattern.marksM[i] < 0.0f || pattern.marksM[i] >= lapM) {
      return false;
    }
    if (i > 0 && pattern.marksM[i] <= pattern.marksM[i - 1]) {
      return false;  // 昇順でない
    }
  }
  return true;
}

// 走行支援の設定全体が妥当かどうかを判定する関数
bool isValidStrategyConfig(const StrategyAssist::Config& cfg) {
  if (cfg.lapM < 100.0f || cfg.guardKmh < 0.0f) {
    return false;
  }
  if (!isValidStrategyPattern(cfg.primary, cfg.lapM, cfg.guardKmh)) {
    return false;
  }
  if (cfg.fallback.markCount > 0 && !isValidStrategyPattern(cfg.fallback, cfg.lapM, cfg.guardKmh)) {
    return false;
  }
  for (uint8_t i = 0; i < cfg.splitCount; i++) {
    if (!isfinite(cfg.splitsS[i]) || cfg.splitsS[i] <= 0.0f || (i > 0 && cfg.splitsS[i] <= cfg.splitsS[i - 1])) {
      return false;
    }
  }
  return true;
}

// 走行支援の設定(JSON文字列)を解釈し、指定された項目でcfgを上書きする関数
bool parseStrategyConfigJson(const char* json, StrategyAssist::Config& cfg) {
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, json);
  if (err) {
    Serial.printf("[Strategy] JSON parse failed: %s\n", err.c_str());
    return false;
  }

  JsonVariantConst root = doc.as<JsonVariantConst>();
  if (root["lap_m"].is<float>()) { cfg.lapM = root["lap_m"].as<float>(); }
  if (root["guard_kmh"].is<float>()) { cfg.guardKmh = root["guard_kmh"].as<float>(); }
  if (root["final_extra_if_split_over_s"].is<float>()) { cfg.finalExtraIfSplitOverS = root["final_extra_if_split_over_s"].as<float>(); }
  if (root["fuel_warn_mpa"].is<float>()) { cfg.fuelWarnMpa = root["fuel_warn_mpa"].as<float>(); }
  if (root["beep"].is<bool>()) { cfg.beep = root["beep"].as<bool>(); }
  readStrategyPatternJson(root, cfg.primary);
  if (!root["fallback"].isNull()) {
    readStrategyPatternJson(root["fallback"], cfg.fallback);
  }
  JsonArrayConst splits = root["splits_s"];
  if (!splits.isNull()) {
    cfg.splitCount = 0;
    for (JsonVariantConst split : splits) {
      if (cfg.splitCount >= StrategyAssist::MAX_SPLITS) {
        break;
      }
      cfg.splitsS[cfg.splitCount++] = split.as<float>();
    }
  }
  return true;
}

// SDカードの設定ファイル(JSON)を読み込み、cfgを上書きする関数
// ファイルが無い場合はfalseを返す。ファイルはあるが解釈できなかった場合はoutParsedがfalseになる
bool readStrategyConfigFile(const char* path, StrategyAssist::Config& cfg, bool& outParsed) {
  outParsed = false;
  file_t strategyFile = sd.open(path, O_READ);
  if (!strategyFile) {
    return false;
  }

  static char buf[STRATEGY_FILE_BUFFER_SIZE];
  int len = strategyFile.read(buf, sizeof(buf) - 1);
  strategyFile.close();
  if (len > 0) {
    buf[len] = '\0';
    outParsed = parseStrategyConfigJson(buf, cfg);
  }
  return true;
}

// ロケーション識別子に対応する走行支援の設定をSDカードから読み込む関数
// 設定はファームに持たない。設定ファイルが無ければ走行支援は無効（SDから削除すれば無効にできる）
void loadStrategyConfigForLoc(const String& loc) {
  StrategyAssist::Config cfg;
  cfg.totalLaps = totallaps;
  strategySource = STRATEGY_SRC_NONE;

  if (LOGGING) {
    const char* path = getStrategyFilePathByLoc(loc);
    bool parsed = false;
    if (readStrategyConfigFile(path, cfg, parsed)) {
      if (parsed && isValidStrategyConfig(cfg)) {
        cfg.enabled = true;
        strategySource = STRATEGY_SRC_SD;
        Serial.printf("[Strategy] loaded %s\n", path);
      } else {
        // 設定ファイルが壊れている場合は走行支援を無効にし、モード0の画面に異常を出す
        cfg = StrategyAssist::Config();
        strategySource = STRATEGY_SRC_SD_ERROR;
        Serial.printf("[Strategy] invalid config: %s\n", path);
      }
    }
  }

  strategy.configure(cfg);
}

// 現在のロケーションに対応する走行支援の設定がロードされていない場合にロードする関数
void ensureStrategyLoadedForCurrentLoc() {
  if (Loc.length() == 0 || loadedStrategyLoc == Loc) {
    return;
  }
  loadStrategyConfigForLoc(Loc);
  loadedStrategyLoc = Loc;
}

// 周内位置 [m] を返す関数（スタートラインからの距離。公式の1周の距離に合わせて伸縮する。求められない場合はNAN）
// 最近傍ウェイポイントの前後の区間へ現在地を射影し、ウェイポイントの間隔（約8 m）より細かく求める
float currentLapPositionMeters() {
  const float lapM = strategy.config().lapM;
  if (!positionFresh || nearestWaypointIndex < 0 || waypointCount < 2 || waypointRawLapMeters <= 0.0f || lapM <= 0.0f) {
    return NAN;
  }

  const size_t nearest = static_cast<size_t>(nearestWaypointIndex);
  const double metersPerDegLat = 111320.0;
  const double metersPerDegLng = 111320.0 * cos(la * pi / 180.0);
  double bestOffset = 1e9;
  double bestRaw = 0.0;
  for (uint8_t k = 0; k < 2; k++) {
    // k=0: 手前の区間（nearest-1 → nearest）, k=1: 先の区間（nearest → nearest+1）
    const size_t a = (k == 0) ? (nearest + waypointCount - 1) % waypointCount : nearest;
    const size_t b = (a + 1) % waypointCount;
    const double bx = (waypoints[b].lng - waypoints[a].lng) * metersPerDegLng;
    const double by = (waypoints[b].lat - waypoints[a].lat) * metersPerDegLat;
    const double px = (ln - waypoints[a].lng) * metersPerDegLng;
    const double py = (la - waypoints[a].lat) * metersPerDegLat;
    const double len2 = bx * bx + by * by;
    const double t = (len2 > 0.0) ? constrain((px * bx + py * by) / len2, 0.0, 1.0) : 0.0;
    const double offset = sqrt(pow(px - t * bx, 2) + pow(py - t * by, 2));
    if (offset < bestOffset) {
      bestOffset = offset;
      const double segEnd = (b > a) ? waypoints[b].dist : waypointRawLapMeters;  // 末尾→先頭の区間は1周の距離まで
      bestRaw = waypoints[a].dist + t * (segEnd - waypoints[a].dist);
    }
  }
  if (bestOffset > STRATEGY_OFF_COURSE_M) {
    return NAN;  // ピット・パドックなどコース外
  }

  // 測位からの経過時間ぶんを車速で進める（GNSSの更新間隔による合図の遅れを補う）
  const float pos = static_cast<float>(bestRaw) * (lapM / waypointRawLapMeters) +
                    speed / 3.6f * static_cast<float>(positionAgeMs) / 1000.0f;
  return fmodf(pos, lapM);
}

// 発進（ECUの走行時間が0から動き出した）を検知して周回数を0に戻す関数
// 発進前にラインをまたいで数えてしまった周回を、走行に持ち込まないようにする
void resetLapCountOnLaunch() {
  static float prevWorktime = 0.0f;
  if (prevWorktime <= 0.0f && worktime > 0.0f) {
    Lapcount = 0;
    lapCountDistanceBase = 0;
    lapCountDistanceBaseValid = true;
  }
  prevWorktime = worktime;
}

// 走行支援の判定を更新し、合図が変わったときにブザーを鳴らす関数
void updateStrategyAssist() {
  if (!strategy.config().enabled) {
    return;
  }

  // Bボタン長押しで通常パターン ⇄ 予備パターンを切り替える
  if (M5.BtnB.wasHold()) {
    strategy.setFallback(!strategy.usingFallback());
  }

  StrategyAssist::Input in;
  in.nowMs = millis();
  in.lapPosM = currentLapPositionMeters();
  in.speedKmh = speed;
  in.engineOn = INJ_timems > 0.0f;  // ECUはエンジン停止中の噴射時間を0で送る
  in.worktimeS = worktime;
  in.lapsDone = Lapcount;
  in.fuelMpa = FuelPre;

  const StrategyAssist::Cue prevCue = strategy.output().cue;
  const StrategyAssist::Output& out = strategy.update(in);

#if STRATEGY_BEEP_AVAILABLE
  static unsigned long lastBeepAt = 0;
  const bool isActionCue = (out.cue == StrategyAssist::Cue::BurnNow || out.cue == StrategyAssist::Cue::CutNow);
  if (strategy.config().beep && isActionCue &&
      (out.cue != prevCue || millis() - lastBeepAt >= STRATEGY_BEEP_REPEAT_MS)) {
    lastBeepAt = millis();
    if (out.cue == StrategyAssist::Cue::BurnNow) {
      M5.Speaker.tone(2600, 120, 0, true);   // 加速: 短く2回
      M5.Speaker.tone(3200, 120, 0, false);
    } else {
      M5.Speaker.tone(1300, 500, 0, true);   // 停止: 長く1回
    }
  }
#else
  (void)prevCue;
  (void)out;
#endif
}

// 秒数を "mm:ss" に整形する
void formatMinSec(char* out, size_t outSize, float seconds) {
  const unsigned int total = static_cast<unsigned int>(seconds < 0.0f ? 0.0f : seconds);
  snprintf(out, outSize, "%02u:%02u", total / 60U, total % 60U);
}

// 走行支援画面（支援が有効なときのモード0）の描画
void drawStrategyMode() {
  using StrategyAssist::Cue;
  const StrategyAssist::Output& out = strategy.output();
  const StrategyAssist::Config& cfg = strategy.config();
  const StrategyAssist::Pattern& pattern = out.usingFallback ? cfg.fallback : cfg.primary;
  char buf[48];

  // 背景色で合図を伝える: 加速=緑, 停止=赤
  int bg = TFT_BLACK;
  int fg = TFT_WHITE;
  if (out.cue == Cue::BurnNow) {
    bg = TFT_GREEN;
    fg = TFT_BLACK;
  } else if (out.cue == Cue::CutNow) {
    bg = TFT_RED;
  }
  const bool plainBg = (bg == TFT_BLACK);
  lcd_s.fillScreen(bg);
  lcd_s.setTextColor(fg);
  lcd_s.setTextSize(1);

  // 状態行: 場所・加速パターン / 接続状況
  lcd_s.setFont(&fonts::lgfxJapanGothicP_16);
  lcd_s.setTextDatum(TL_DATUM);
  snprintf(buf, sizeof(buf), "%s %u回%s%s",
           Loc == "su" ? "鈴鹿" : (Loc == "mo" ? "茂木" : "豊田"),
           static_cast<unsigned int>(pattern.markCount),
           out.usingFallback ? "予備" : "",
           STRATEGY_SIM ? " SIM" : "");
  lcd_s.drawString(buf, 4, 0);
  lcd_s.setTextDatum(TR_DATUM);
  snprintf(buf, sizeof(buf), "SD:%c GNSS:%c MQ:%c", LOGGING ? 'O' : 'x', positionFresh ? 'O' : 'x', MQTTpush ? 'O' : 'x');
  lcd_s.drawString(buf, 318, 0);

  // 速度と、今の周の加速停止速度
  lcd_s.setFont(&fonts::Font7);
  lcd_s.setTextDatum(TR_DATUM);
  snprintf(buf, sizeof(buf), "%.1f", speed);
  lcd_s.drawString(buf, 150, 18);

  lcd_s.setFont(&fonts::lgfxJapanGothicP_16);
  lcd_s.setTextDatum(TL_DATUM);
  lcd_s.drawString("停止", 166, 22);
  lcd_s.setFont(&fonts::lgfxJapanGothicP_20);
  lcd_s.setTextSize(2);
  lcd_s.setTextDatum(TR_DATUM);
  if (plainBg && out.bumpKmh > 0.0f) {
    lcd_s.setTextColor(TFT_MAGENTA);  // 遅れを取り戻すために上げている
  } else if (plainBg && out.bumpKmh < 0.0f) {
    lcd_s.setTextColor(TFT_CYAN);
  }
  snprintf(buf, sizeof(buf), "%.1f", out.vOffKmh);
  lcd_s.drawString(buf, 316, 26);
  lcd_s.setTextColor(fg);
  lcd_s.setTextSize(1);

  // 速度バー（0〜45 km/h）と、最低限界速度・加速停止速度の目盛り
  const int barX = 10;
  const int barW = 300;
  const float barMaxKmh = 45.0f;
  lcd_s.drawRect(barX - 1, 70, barW + 2, 12, fg);
  lcd_s.fillRect(barX, 71, static_cast<int>(constrain(speed, 0.0f, barMaxKmh) / barMaxKmh * barW), 10, fg);
  const int guardX = barX + static_cast<int>(constrain(cfg.guardKmh, 0.0f, barMaxKmh) / barMaxKmh * barW);
  const int vOffX = barX + static_cast<int>(constrain(out.vOffKmh, 0.0f, barMaxKmh) / barMaxKmh * barW);
  if (cfg.guardKmh > 0.0f) {
    lcd_s.fillRect(guardX - 1, 67, 3, 18, plainBg ? TFT_YELLOW : fg);
  }
  lcd_s.fillRect(vOffX - 1, 67, 3, 18, plainBg ? TFT_RED : fg);

  // 合図
  int cueColor = fg;
  switch (out.cue) {
    case Cue::Idle:
      snprintf(buf, sizeof(buf), "スタート待ち");
      break;
    case Cue::Approach:
      cueColor = TFT_YELLOW;
      // fallthrough
    case Cue::Coast:
      snprintf(buf, sizeof(buf), "加速まで %dm", static_cast<int>(out.nextMarkDistM));
      break;
    case Cue::BurnNow:
      snprintf(buf, sizeof(buf), out.guard ? "加速! 速度低下" : "加速!");
      break;
    case Cue::Burning:
      snprintf(buf, sizeof(buf), "%.1fで停止", out.vOffKmh);
      break;
    case Cue::CutNow:
      snprintf(buf, sizeof(buf), "停止!");
      break;
    case Cue::CoastHome:
      snprintf(buf, sizeof(buf), "惰行でゴール");
      break;
    case Cue::NoPosition:
      cueColor = TFT_YELLOW;
      snprintf(buf, sizeof(buf), "位置なし");
      break;
    case Cue::Finished:
      snprintf(buf, sizeof(buf), "FINISH");
      break;
    default:
      buf[0] = '\0';
      break;
  }
  lcd_s.setFont(&fonts::lgfxJapanGothicP_20);
  lcd_s.setTextSize(2);
  lcd_s.setTextDatum(MC_DATUM);
  lcd_s.setTextColor(cueColor);
  lcd_s.drawString(buf, 160, 110);
  lcd_s.setTextColor(fg);
  lcd_s.setTextSize(1);

  // 1周の位置バー（▼=加速開始位置、縦線=現在地）。燃圧低下時は警告に差し替える
  if (out.fuelLow) {
    lcd_s.fillRect(0, 136, 320, 24, TFT_ORANGE);
    lcd_s.setFont(&fonts::lgfxJapanGothicP_16);
    lcd_s.setTextDatum(MC_DATUM);
    lcd_s.setTextColor(TFT_BLACK);
    if (!out.usingFallback && strategy.hasFallback()) {
      snprintf(buf, sizeof(buf), "燃圧低下 B長押しで%.0fkm/h・%u回",
               cfg.fallback.vOffKmh, static_cast<unsigned int>(cfg.fallback.markCount));
    } else {
      snprintf(buf, sizeof(buf), "燃圧低下");
    }
    lcd_s.drawString(buf, 160, 148);
    lcd_s.setTextColor(fg);
  } else {
    const int lapBarY = 148;
    lcd_s.drawRect(barX - 1, lapBarY, barW + 2, 8, fg);
    for (uint8_t i = 0; i < pattern.markCount; i++) {
      const int x = barX + static_cast<int>(pattern.marksM[i] / cfg.lapM * barW);
      if (i < out.activeMarks) {
        lcd_s.fillTriangle(x, lapBarY - 1, x - 5, lapBarY - 10, x + 5, lapBarY - 10,
                           (plainBg && i == out.nextMark) ? TFT_YELLOW : fg);
      } else {
        lcd_s.drawTriangle(x, lapBarY - 1, x - 5, lapBarY - 10, x + 5, lapBarY - 10, fg);  // 最終周で使わない目印
      }
    }
    const float lapPos = currentLapPositionMeters();
    if (isfinite(lapPos)) {
      const int x = barX + static_cast<int>(constrain(lapPos / cfg.lapM, 0.0f, 1.0f) * barW);
      lcd_s.fillRect(x - 2, lapBarY - 3, 5, 14, plainBg ? TFT_CYAN : fg);
    }
  }

  // 残り周回
  const int restlaps = max(0, static_cast<int>(totallaps) - static_cast<int>(Lapcount));
  lcd_s.setFont(&fonts::lgfxJapanGothicP_16);
  lcd_s.setTextDatum(TL_DATUM);
  lcd_s.drawString(restlaps == 1 ? "最終周" : "残り", 8, 162);
  lcd_s.drawString("走行時間", 84, 162);
  lcd_s.setFont(&fonts::lgfxJapanGothicP_20);
  lcd_s.setTextSize(2);
  snprintf(buf, sizeof(buf), "%d周", restlaps);
  lcd_s.drawString(buf, 8, 182);

  // 走行時間（直近の通過が遅れていればマゼンタ）
  const int timeColor = (plainBg && out.bumpKmh > 0.0f) ? TFT_MAGENTA : fg;
  formatMinSec(buf, sizeof(buf), worktime);
  lcd_s.setTextColor(timeColor);
  lcd_s.drawString(buf, 84, 182);
  lcd_s.setTextColor(fg);

  // 規定時間までの残り時間の帯。位置バーの現在地と同じ向きにそろえ、時間が進むと左から右へ短くなる
  const float elapsedRatio = (limittime > 0) ? constrain(worktime / static_cast<float>(limittime), 0.0f, 1.0f) : 0.0f;
  const int elapsedPx = static_cast<int>(elapsedRatio * barW);
  lcd_s.drawRect(barX - 1, 226, barW + 2, 12, fg);
  lcd_s.fillRect(barX + elapsedPx, 227, barW - elapsedPx, 10, timeColor);

  // 同じ帯に全周回の進み具合を重ねる: 周の区切りの目盛りと、走行距離の現在地
  // 現在地が帯の左端より右にあれば、距離の進みが時間の進みを上回っている（平均25 km/hより速い）
  for (uint8_t i = 1; i < totallaps; i++) {
    const int x = barX + barW * i / totallaps;
    lcd_s.drawFastVLine(x, 227, 10, (x >= barX + elapsedPx) ? bg : fg);  // 帯の上では背景色、帯の外では文字色で描く
  }
  if (goal > 0) {
    const int x = barX + static_cast<int>(constrain(static_cast<float>(distance) / static_cast<float>(goal), 0.0f, 1.0f) * barW);
    lcd_s.fillRect(x - 3, 224, 7, 16, bg);  // 帯と同じ色でも見えるよう背景色で縁取る
    lcd_s.fillRect(x - 2, 225, 5, 14, plainBg ? TFT_CYAN : fg);
  }

  // 直近の通過タイムと目標との差。まだ通過していなければ加速開始位置の一覧
  lcd_s.setTextDatum(TR_DATUM);
  if (out.hasSplit) {
    if (isfinite(out.delayS)) {
      snprintf(buf, sizeof(buf), "%+ds", static_cast<int>(constrain(lroundf(out.delayS), -99L, 99L)));
    } else {
      snprintf(buf, sizeof(buf), "--");
    }
    if (plainBg && out.bumpKmh > 0.0f) {
      lcd_s.setTextColor(TFT_MAGENTA);
    } else if (plainBg && out.bumpKmh < 0.0f) {
      lcd_s.setTextColor(TFT_CYAN);
    }
    lcd_s.drawString(buf, 316, 182);
    lcd_s.setTextColor(fg);
    lcd_s.setTextSize(1);
    lcd_s.setFont(&fonts::lgfxJapanGothicP_16);
    char splitStr[8];
    formatMinSec(splitStr, sizeof(splitStr), out.splitS);
    snprintf(buf, sizeof(buf), "%u周 %s", static_cast<unsigned int>(out.splitLap), splitStr);
    lcd_s.drawString(buf, 316, 162);
  } else {
    lcd_s.setTextSize(1);
    lcd_s.setFont(&fonts::lgfxJapanGothicP_16);
    lcd_s.drawString("加速位置(m)", 316, 162);
    lcd_s.setFont(&fonts::lgfxJapanGothicP_12);
    int len = 0;
    buf[0] = '\0';
    for (uint8_t i = 0; i < pattern.markCount && len < static_cast<int>(sizeof(buf)) - 8; i++) {
      len += snprintf(buf + len, sizeof(buf) - len, i == 0 ? "%d" : "/%d", static_cast<int>(pattern.marksM[i]));
    }
    lcd_s.drawString(buf, 316, 192);
  }
  lcd_s.setTextSize(1);
}

//==================== 各種関数 =====================

// LCD画面にメッセージを表示する
void showMessage(String msg)
{
  // lcd.setRotation(1);
  // lcd.fillScreen(BLACK);
  // lcd.setTextColor(WHITE);
  lcd.setCursor(0, 20);
  lcd.println(msg);
  lcd.setCursor(50, 230);
  lcd.print("A");
  lcd.setCursor(260, 230);
  lcd.print("C");
}

// BLEから受信した文字が数値データとして有効な文字かどうかを判定する
bool isBLEValueChar(char c)
{
  return (c >= '0' && c <= '9') || c == '.' || c == '+' || c == '-';
}

// BLEから受信した文字列が有効な数値データかどうかを判定し変換する（温度・PRI/SEC/FUEL共用）
bool tryParseBLEFloatValue(const String& raw, float& outValue) {
  if (raw.length() == 0) {                      // 空文字は無効
    return false;
  }

  bool hasDigit = false;
  for (size_t i = 0; i < raw.length(); i++) {   // 有効な文字以外が混入していないかチェック
    char c = raw[i];
    if (isBLEValueChar(c)) {                      // 有効な文字の場合は数字が含まれているかもチェック
      if (c >= '0' && c <= '9') {
        hasDigit = true;
      }
      continue;
    }
    if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {  // 空白文字は無視
      continue;
    }
    return false;
  }

  if (!hasDigit) {
    return false;
  }

  char* endPtr = nullptr;                       // 変換後の文字列の末尾を指すポインタ
  float parsed = strtof(raw.c_str(), &endPtr);  // 文字列をfloatに変換
  if (endPtr == raw.c_str()) {                  // 変換できなかった場合は無効
    return false;
  }
  while (*endPtr != '\0') {                     // 変換後の文字列の末尾以降に有効な文字が混入していないかチェック
    if (*endPtr != ' ' && *endPtr != '\t' && *endPtr != '\r' && *endPtr != '\n') {  // 空白文字以外が混入している場合は無効
      return false;
    }
    endPtr++;
  }

  outValue = parsed;
  return true;
}

// ECU 1行CSVをパースする
// 期待フォーマット: rpm,inj,ign,inj_end,speed,distance,gasml,dispergas,worktime*XX
// XX は先頭から'*'直前までのXOR（16進2桁）。不一致・項目不足の行は捨て、グローバルは更新しない。
bool tryParseEcuCsvLine(char* line) {
  if (line == nullptr || line[0] == '\0') {
    return false;
  }

  char* star = strrchr(line, '*');
  if (star == nullptr || strlen(star) != 3) {
    return false;
  }
  char* csEnd = nullptr;
  unsigned long expected = strtoul(star + 1, &csEnd, 16);
  if (*csEnd != '\0') {
    return false;
  }
  uint8_t cs = 0;
  for (const char* p = line; p < star; p++) {
    cs ^= static_cast<uint8_t>(*p);
  }
  if (cs != expected) {
    return false;
  }
  *star = '\0';

  // 欠けた行でグローバルが部分更新されないよう、全項目そろうまではローカルに溜める
  uint16_t rpm = 0, dist = 0;
  float inj = 0.0f, spd = 0.0f, gas = 0.0f, dispGas = 0.0f, work = 0.0f;
  uint8_t ign = 0;
  int16_t injEnd = 0;

  uint8_t index = 0;
  char* savePtr = nullptr;
  char* token = strtok_r(line, ",", &savePtr);
  while (token != nullptr && index < 9) {
    while (*token == ' ' || *token == '\t') {
      token++;
    }

    switch (index) {
      case 0: rpm = static_cast<uint16_t>(strtoul(token, nullptr, 10)); break;
      case 1: inj = strtof(token, nullptr); break;
      case 2: ign = static_cast<uint8_t>(strtoul(token, nullptr, 10)); break;
      case 3: injEnd = static_cast<int16_t>(strtol(token, nullptr, 10)); break;
      case 4: spd = strtof(token, nullptr); break;
      case 5: dist = static_cast<uint16_t>(strtoul(token, nullptr, 10)); break;
      case 6: gas = strtof(token, nullptr); break;
      case 7: dispGas = strtof(token, nullptr); break;
      case 8: work = strtof(token, nullptr); break;
      default: break;
    }

    index++;
    token = strtok_r(nullptr, ",", &savePtr);
  }

  if (index < 9 || !isfinite(work) || work < 0.0f) {
    return false;
  }

  tachoRpm = rpm;
  INJ_timems = inj;
  IGN_CA = ign;
  injEndCA = injEnd;
  speed = spd;
  distance = dist;
  gasml = gas;
  dispergas = dispGas;
  worktime = work;

  // Lapcountはコントロールライン通過検知（updateGNSS内のupdateLapCountByControlLineCrossing）で更新するため、
  // ここでは更新しない（旧: 走行距離ベースの周回数推定）
  return true;
}

// 次回ログ番号の読み込み（起動時の採番高速化用）
int loadNextLogIndex() {
  file_t indexFile = sd.open(NEXT_LOG_INDEX_FILE, O_READ);  // インデックスファイルを開く
  if (!indexFile) {
    return 0;
  }

  char buf[16] = {0};
  int len = indexFile.read(buf, sizeof(buf) - 1);  // インデックスファイルからデータを読み込む
  indexFile.close();
  if (len <= 0) {
    return 0;
  }

  int nextIndex = atoi(buf);          // 読み込んだ文字列を整数に変換
  if (nextIndex < 0) {
    return 0;
  }
  return nextIndex;
}

// 次回ログ番号の書き込み（起動時の採番高速化用）
void saveNextLogIndex(int nextIndex) {
  file_t indexFile = sd.open(NEXT_LOG_INDEX_FILE, O_WRITE | O_CREAT | O_TRUNC);  // インデックスファイルを開く
  if (!indexFile) {
    return;
  }
  indexFile.print(nextIndex);         // 次回ログ番号を書き込む
  indexFile.close();                  // インデックスファイルを閉じる
}

// BLE中継機(NanoC6)へコマンドを送信し、応答フレーム(32byte)を読み取る。
// NanoC6側は応答フレーム末尾1byte(frame[BLE_I2C_FRAME_SIZE-1])に要求されたコマンドを
// そのままエコーバックする仕様になっている。バスノイズ等により応答が別コマンドの
// ものとズレて配信されることがあるため、エコーが要求コマンドと一致するか確認し、
// 不一致なら読み直す
bool requestBleI2CFrame(uint8_t command, uint8_t (&frame)[BLE_I2C_FRAME_SIZE]) {
  // 初回 + リトライ3回。1回あたりのコストは僅か(待機2ms+実通信)で、
  // バスノイズによる偶発的なズレを吸収しやすくするため上限を広げてある
  const uint8_t maxAttempts = 4;
  for (uint8_t attempt = 0; attempt < maxAttempts; attempt++) {
    BleI2C.beginTransmission(BLE_I2C_SLAVE_ADDR);
    BleI2C.write(command);
    if (BleI2C.endTransmission() != 0) {
      return false;
    }

    // NanoC6側は応答フレームをタスクコンテキストで非同期に用意している（ISR内では
    // ESP-IDFのi2c_slave_transmit()を呼べないため）。書き込み直後すぐに読み出すと
    // 準備が間に合わず、応答フレームが1コマンド分ズレて配信されることがあるため、
    // 読み出し開始前にごく短い待ち時間を入れて余裕を持たせる
    delayMicroseconds(2000);

    if (BleI2C.requestFrom(static_cast<int>(BLE_I2C_SLAVE_ADDR), static_cast<int>(BLE_I2C_FRAME_SIZE)) != BLE_I2C_FRAME_SIZE) {
      return false;
    }
    for (uint8_t i = 0; i < BLE_I2C_FRAME_SIZE; i++) {
      frame[i] = BleI2C.available() ? static_cast<uint8_t>(BleI2C.read()) : 0;
    }

    if (frame[BLE_I2C_FRAME_SIZE - 1] == command) {
      return true;
    }
    // コマンドエコーが不一致 = 別コマンドの応答とズレて配信された。もう一度要求し直す
  }
  return false;  // リトライしても改善しなかった
}

// BLE中継機(NanoC6)経由で取得する1系統分のデータ定義（コマンド・格納先・有効範囲・最終有効受信時刻）
struct BleI2CChannel {
  uint8_t command;
  const char* label;
  float* value;
  float rangeMin;
  float rangeMax;
  unsigned long lastValidAt;
};
// BLE中継機(NanoC6)経由で取得するデータの定義（コマンド・格納先・有効範囲・最終有効受信時刻）
static BleI2CChannel bleChannels[] = {
  { BLE_I2C_CMD_ENGINE_TEMP, "ENGINE_TEMP", &EngTemp, 10.0f,  150.0f, 0 },
  { BLE_I2C_CMD_PRI_PRE,     "PRI",         &PriPre,   0.01f, 0.72f,  0 },
  { BLE_I2C_CMD_SEC_PRE,     "SEC",         &SecPre,   0.01f, 0.72f,  0 },
  { BLE_I2C_CMD_FUEL_PRE,    "FUEL",        &FuelPre, -0.02f, 1.05f,  0 },
};

// BLE中継機(NanoC6)からI2C経由でエンジン温度・PRI/SEC/FUELデータを取得する（一定間隔でポーリング＋タイムアウト処理）
void updateBLE() {
  static unsigned long lastPollAt = 0;

  if (millis() - lastPollAt >= BLE_I2C_POLL_INTERVAL) {
    lastPollAt = millis();

    for (BleI2CChannel& ch : bleChannels) {
      uint8_t frame[BLE_I2C_FRAME_SIZE];
      if (requestBleI2CFrame(ch.command, frame)) {
        uint8_t len = frame[0];
        // frame[BLE_I2C_FRAME_SIZE-1]はコマンドエコー用に予約されているため、
        // データ本体はそれを除いた BLE_I2C_FRAME_SIZE-2 byteまでが上限
        if (len > 0 && len <= BLE_I2C_FRAME_SIZE - 2) {
          String payload;
          payload.reserve(len);
          for (uint8_t i = 0; i < len; i++) {
            payload += static_cast<char>(frame[1 + i]);
          }

          float parsedValue = 0.0;
          if (tryParseBLEFloatValue(payload, parsedValue) && parsedValue >= ch.rangeMin && parsedValue <= ch.rangeMax) {
            *ch.value = parsedValue;
            ch.lastValidAt = millis();
#if BLE_I2C_DIAGNOSTIC_LOG
            Serial.printf("[BLE I2C] RX %s: %s -> %.2f\n", ch.label, payload.c_str(), parsedValue);
#endif
          } else {
#if BLE_I2C_DIAGNOSTIC_LOG
            Serial.printf("[BLE I2C] %s INVALID/OUT OF RANGE: %s\n", ch.label, payload.c_str());
#endif
          }
        }
      } else {
#if BLE_I2C_DIAGNOSTIC_LOG
        Serial.printf("[BLE I2C] %s request failed\n", ch.label);
#endif
      }
    }
  }

  // データ受信完全ロス時のリセット処理(2秒以上経過、チャンネルごとに判定)
  for (BleI2CChannel& ch : bleChannels) {
    if (millis() - ch.lastValidAt > 2000) {
      *ch.value = 0.0;
    }
  }
}

// ECUからのデータ読み取り
void updateECU() {
  static char ecuLine[ECU_LINE_BUFFER_SIZE] = {0};
  static uint8_t ecuLen = 0;

  while (Serial1.available() > 0) {
    char ch = Serial1.read();
    receiveECUtime = millis();

    if (ch == '\r') {
      continue;
    }

    if (ch == '\n') {
      ecuLine[ecuLen] = '\0';
      if (ecuLen > 0) {
        tryParseEcuCsvLine(ecuLine);
      }
      ecuLen = 0;
      ecuLine[0] = '\0';
      continue;
    }

    if (ecuLen < (ECU_LINE_BUFFER_SIZE - 1)) {
      ecuLine[ecuLen++] = ch;
      ecuLine[ecuLen] = '\0';
    } else {
      ecuLen = 0;
      ecuLine[0] = '\0';
    }
  }

  if (millis() - receiveECUtime > 2000) {
    tachoRpm = 0;
    INJ_timems = 0;
    IGN_CA = 0;
    injEndCA = 0;
    speed = 0.0;
  }
}

// コントロールライン通過判定（ラップカウント用）

// 3点の向き（時計回り/反時計回り/一直線）を判定する関数（外積の符号で判定）
int8_t orientation(double ax, double ay, double bx, double by, double cx, double cy) {
  double val = (by - ay) * (cx - bx) - (bx - ax) * (cy - by);
  if (fabs(val) < 1e-12) return 0;   // 一直線上（ほぼ0とみなす許容誤差）
  return (val > 0) ? 1 : 2;          // 1:時計回り, 2:反時計回り
}

// 点qが線分pr上にあるか判定する関数（orientationが0（一直線）の場合のみ呼び出す）
bool onSegment(double px, double py, double qx, double qy, double rx, double ry) {
  return (qx <= max(px, rx) && qx >= min(px, rx) &&
          qy <= max(py, ry) && qy >= min(py, ry));
}

// 線分p1-p2と線分q1-q2が交差しているか判定する関数（外積を用いた一般的な線分交差判定アルゴリズム）
bool segmentsIntersect(double p1x, double p1y, double p2x, double p2y,
                        double q1x, double q1y, double q2x, double q2y) {
  int8_t o1 = orientation(p1x, p1y, p2x, p2y, q1x, q1y);
  int8_t o2 = orientation(p1x, p1y, p2x, p2y, q2x, q2y);
  int8_t o3 = orientation(q1x, q1y, q2x, q2y, p1x, p1y);
  int8_t o4 = orientation(q1x, q1y, q2x, q2y, p2x, p2y);

  if (o1 != o2 && o3 != o4) return true;  // 一般ケース

  // 特殊ケース（端点が相手の線分上に重なる場合）
  if (o1 == 0 && onSegment(p1x, p1y, q1x, q1y, p2x, p2y)) return true;
  if (o2 == 0 && onSegment(p1x, p1y, q2x, q2y, p2x, p2y)) return true;
  if (o3 == 0 && onSegment(q1x, q1y, p1x, p1y, q2x, q2y)) return true;
  if (o4 == 0 && onSegment(q1x, q1y, p2x, p2y, q2x, q2y)) return true;

  return false;
}

// 現在のロケーションに対応するコントロールライン座標を取得する関数（未定義ロケーションではfalseを返す）
bool getControlLineForLoc(const String& loc, double& outLa1, double& outLn1, double& outLa2, double& outLn2) {
  if (loc == "su") {
    outLa1 = controlline_la1_su; outLn1 = controlline_ln1_su;
    outLa2 = controlline_la2_su; outLn2 = controlline_ln2_su;
    return true;
  }
  if (loc == "mo") {
    outLa1 = controlline_la1_mo; outLn1 = controlline_ln1_mo;
    outLa2 = controlline_la2_mo; outLn2 = controlline_ln2_mo;
    return true;
  }
  if (loc == "to") {
    outLa1 = controlline_la1_to; outLn1 = controlline_ln1_to;
    outLa2 = controlline_la2_to; outLn2 = controlline_ln2_to;
    return true;
  }
  return false;
}

// GPSの前回位置→今回位置を結ぶ線分がコントロールラインと交差したか判定し、
// 交差していればLapcountをインクリメントする関数（多重カウント防止のデバウンス・GPSロスト後の誤検出防止のテレポートガード付き）
void updateLapCountByControlLineCrossing(bool positionUpdated, double prevLat, double prevLng) {
  static bool hasPriorFix = false;
  static unsigned long lastLapCrossedAt = 0;

  if (!positionUpdated) {
    return;  // 今回GPSの新規測位がない場合は判定しない（誤検出防止）
  }

  if (!hasPriorFix) {
    hasPriorFix = true;  // 初回の有効な測位では前回位置が無いため判定をスキップする
    return;
  }

  double clLa1, clLn1, clLa2, clLn2;
  if (!getControlLineForLoc(Loc, clLa1, clLn1, clLa2, clLn2)) {
    return;  // コントロールライン座標が未定義のロケーションでは判定しない
  }

  if (millis() - lastLapCrossedAt < LAP_CROSS_DEBOUNCE_MS) {
    return;  // クールダウン時間内の連続検出は多重カウント防止のため無視する
  }

  // GPSロスト後の再測位による「テレポート」誤検出を防止（前回位置からの概算移動距離が閾値超なら判定しない）
  double dLat = la - prevLat;
  double dLng = ln - prevLng;
  double metersPerDegLat = 111320.0;
  double metersPerDegLng = 111320.0 * cos(la * pi / 180.0);
  double movedMeters = sqrt(pow(dLat * metersPerDegLat, 2) + pow(dLng * metersPerDegLng, 2));
  if (movedMeters > LAP_CROSS_TELEPORT_GUARD_M) {
    return;
  }

  if (segmentsIntersect(prevLat, prevLng, la, ln, clLa1, clLn1, clLa2, clLn2)) {
    // ECU受信中は、前回カウントからの走行距離が1周の半分に満たない交差を数えない
    // （グリッドがラインの手前にある場合の発進直後の通過や、ライン付近での二重カウントを防ぐ）
    if (distance < lapCountDistanceBase) {
      lapCountDistanceBaseValid = false;  // ECUが再起動して走行距離が0に戻った。次のカウントまで距離では判定しない
    }
    const bool ecuAlive = (millis() - receiveECUtime <= 2000);
    const float lapMeters = (Loc == "su" || Loc == "mo") ? static_cast<float>(goal) / static_cast<float>(totallaps) : 0.0f;
    if (ecuAlive && lapCountDistanceBaseValid && lapMeters > 0.0f &&
        !StrategyAssist::lapCrossingPlausible(static_cast<float>(distance - lapCountDistanceBase), lapMeters)) {
      return;
    }

    Lapcount++;
    lastLapCrossedAt = millis();
    lapCountDistanceBase = distance;
    lapCountDistanceBaseValid = true;
    strategy.onLapCrossed(Lapcount, worktime);  // 通過タイムを保持し、次の周の加速停止速度を決める
  }
}

// GNSSからの位置・時刻読み取り
void updateGNSS() {
  double prevLa = la;   // ラップカウント判定用に前回位置を退避
  double prevLn = ln;
  bool positionUpdated = false;

  uint16_t parsedBytes = 0;
  while (Serial2.available() > 0 && parsedBytes < GNSS_PARSE_BUDGET_BYTES) {
    char ch = Serial2.read();
    parsedBytes++;
    if (gps.encode(ch)) {
      if (gps.time.isUpdated()) {
        updateSystemTimeFromGnss();
        time_t gnssJst = 0;
        if (buildGnssJstTime(gnssJst)) {
          anchorClock(gnssJst, gps.time.centisecond());  // 基準クロックをGNSS時刻に同期
        }
        if (gps.location.lng() > 120 && gps.location.isValid()) {  // 異常値・未固定を除外
          la = gps.location.lat();
          ln = gps.location.lng();
          spd = gps.speed.kmph();
          positionUpdated = true;
        }
        break;
      }
    }
  }
  // ロケーション判定
  if (la >= 34.837989 && la <= 34.84828 && ln >= 136.522015 && ln <= 136.544450) {
    Loc = "su";
    totallaps = totallaps_su;
    goal = goal_su;
    limittime = limittime_su;
  } else if (la >= 36.528477 && la <= 36.538522 && ln >= 140.2192761 && ln <= 140.23853) {
    Loc = "mo";
    totallaps = totallaps_mo;
    goal = goal_mo;
    limittime = limittime_mo;
  } else {
    Loc = "to";
  }

  updateLapCountByControlLineCrossing(positionUpdated, prevLa, prevLn);
}

// 国土地理院APIを呼び出すために、現在の緯度経度が有効かどうかを判定する
bool hasValidLocationForGsiApi() {
  return isfinite(la) && isfinite(ln) && gps.location.isValid() && la >= -90.0 && la <= 90.0 && ln >= -180.0 && ln <= 180.0;
}

// 検出したセンサーから現在の気圧(Pa)を読み出す
bool readPressurePa(float& outPressurePa) {
  if (!isBmx280Ready) {
    return false;
  }
  outPressurePa = bme280.readPressure();
  return true;
}

// 利用中の環境センサーから気圧・気温・湿度を取得する
void updateEnvironmentSensors() {
  float pressurePa = NAN;
  if (readPressurePa(pressurePa) && isfinite(pressurePa) && pressurePa >= 30000.0f && pressurePa <= 120000.0f) {
    envPressureKPa = pressurePa / 1000.0f;
  } else {
    envPressureKPa = NAN;
  }

  if (isBmx280Ready) {
    float temperatureC = bme280.readTemperature();
    float humidityPct = bme280.readHumidity();
    envTemperatureC = (isfinite(temperatureC) && temperatureC > -40.0f && temperatureC < 100.0f) ? temperatureC : NAN;
    envHumidityPct = (isfinite(humidityPct) && humidityPct >= 0.0f && humidityPct <= 100.0f) ? humidityPct : NAN;
    return;
  }

  envTemperatureC = NAN;
  envHumidityPct = NAN;
}

// BME280から気圧を読み取って高度を更新する
void updateAltitudeFromBmx280() {
  if (!isBmx280Ready) {
    return;
  }

  // BME280から気圧を読み取る
  float pressurePa = NAN;
  if (!readPressurePa(pressurePa)) {
    return;
  }
  if (!isfinite(pressurePa) || pressurePa < 30000.0f || pressurePa > 120000.0f) {
    return;
  }

  // 気圧から高度を計算する（国際標準大気モデルを使用）
  float pressureKPa = pressurePa / 1000.0f;
  float rawAltitudeMeters = 44330.0f * (1.0f - powf(pressureKPa / seaLevelPressureKPa, 0.1903f));
  if (isfinite(rawAltitudeMeters) && rawAltitudeMeters > -1000.0f && rawAltitudeMeters < 12000.0f) {
    alt = rawAltitudeMeters + altitudeOffsetMeters;
  }
}

// 国土地理院APIのレスポンスをパースして標高(m)を取り出す
bool tryParseGsiElevationResponse(const String& payload, float& outElevationMeters) {
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    Serial.printf("[GSI] JSON parse failed: %s\n", err.c_str());
    return false;
  }

  float elevation = doc["elevation"] | NAN;
  if (!isfinite(elevation) || elevation < -500.0f || elevation > 10000.0f) {
    Serial.printf("[GSI] elevation out of range: %.2f\n", elevation);
    return false;
  }

  outElevationMeters = elevation;
  return true;
}

// 国土地理院APIから標高(m)を取得する（HTTP優先、失敗時はHTTPSフォールバック）
bool fetchGsiElevationMeters(float& outElevationMeters) {
  const char* gsiHost = "cyberjapandata2.gsi.go.jp";
  String query = String("/general/dem/scripts/getelevation.php?lon=") + String(ln, 6) +
                 String("&lat=") + String(la, 6) +
                 String("&outtype=JSON");
  String httpsUrl = String("https://") + gsiHost + query;
  String httpUrl = String("http://") + gsiHost + query;

  HTTPClient http;
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  http.setConnectTimeout(1200);
  http.setTimeout(1200);

  WiFiClient plainClient;
  if (http.begin(plainClient, httpUrl)) {
    int code = http.GET();
    if (code == HTTP_CODE_OK) {
      bool ok = tryParseGsiElevationResponse(http.getString(), outElevationMeters);
      http.end();
      if (ok) {
        return true;
      }
    } else {
      Serial.printf("[GSI] HTTP GET failed: %d (%s) RSSI=%d\n", code, http.errorToString(code).c_str(), WiFi.RSSI());
    }
    http.end();
  } else {
    Serial.println("[GSI] HTTP http.begin failed");
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[GSI] skip HTTPS fallback: WiFi disconnected");
    return false;
  }

  WiFiClientSecure secureClient;
  secureClient.setInsecure();
  if (http.begin(secureClient, httpsUrl)) {
    int code = http.GET();
    if (code == HTTP_CODE_OK) {
      bool ok = tryParseGsiElevationResponse(http.getString(), outElevationMeters);
      http.end();
      if (ok) {
        return true;
      }
    } else {
      Serial.printf("[GSI] HTTPS GET failed: %d (%s) RSSI=%d\n", code, http.errorToString(code).c_str(), WiFi.RSSI());
    }
    http.end();
  } else {
    Serial.println("[GSI] HTTPS http.begin failed");
  }

  return false;
}

// 国土地理院API標高とBME280生高度との差分を取得し、高度オフセットを固定する
void tryFetchAltitudeOffsetFromGsi() {
  // すでに正常に取得している場合は何もしない
  if (altitudeOffsetFixed) {
    return;
  }
  // 前回の取得から一定時間経過していない場合は何もしない
  if (millis() < nextAltitudeOffsetFetchAt) {
    return;
  }

  // 前提条件が未成立の間は待ち時刻を進めず、成立した瞬間に即実行できるようにする
  if (!isBmx280Ready || WiFi.status() != WL_CONNECTED || !hasValidLocationForGsiApi()) {
    return;
  }

  IPAddress resolvedIp;
  int dnsResult = WiFi.hostByName("cyberjapandata2.gsi.go.jp", resolvedIp);
  if (dnsResult != 1) {
    Serial.printf("[GSI] DNS failed: result=%d RSSI=%d\n", dnsResult, WiFi.RSSI());
    nextAltitudeOffsetFetchAt = millis() + ELEVATION_OFFSET_FETCH_RETRY_INTERVAL;
    return;
  }

  Serial.printf("[GSI] DNS ok: %s RSSI=%d\n", resolvedIp.toString().c_str(), WiFi.RSSI());

  float gsiElevationMeters = NAN;
  if (!fetchGsiElevationMeters(gsiElevationMeters)) {
    nextAltitudeOffsetFetchAt = millis() + ELEVATION_OFFSET_FETCH_RETRY_INTERVAL;
    return;
  }

  float pressurePa = NAN;
  if (!readPressurePa(pressurePa)) {
    Serial.println("[GSI] pressure sensor read failed while fixing offset");
    nextAltitudeOffsetFetchAt = millis() + ELEVATION_OFFSET_FETCH_RETRY_INTERVAL;
    return;
  }
  if (!isfinite(pressurePa) || pressurePa < 30000.0f || pressurePa > 120000.0f) {
    Serial.println("[GSI] BME280 pressure invalid while fixing offset");
    nextAltitudeOffsetFetchAt = millis() + ELEVATION_OFFSET_FETCH_RETRY_INTERVAL;
    return;
  }

  float pressureKPa = pressurePa / 1000.0f;
  float rawAltitudeMeters = 44330.0f * (1.0f - powf(pressureKPa / seaLevelPressureKPa, 0.1903f));
  if (!isfinite(rawAltitudeMeters) || rawAltitudeMeters < -1000.0f || rawAltitudeMeters > 12000.0f) {
    Serial.printf("[GSI] BME280 raw altitude out of range: %.2f\n", rawAltitudeMeters);
    nextAltitudeOffsetFetchAt = millis() + ELEVATION_OFFSET_FETCH_RETRY_INTERVAL;
    return;
  }

  float offset = gsiElevationMeters - rawAltitudeMeters;
  if (!isfinite(offset) || offset < -3000.0f || offset > 3000.0f) {
    Serial.printf("[GSI] offset out of range: %.2f (gsi=%.2f raw=%.2f)\n", offset, gsiElevationMeters, rawAltitudeMeters);
    nextAltitudeOffsetFetchAt = millis() + ELEVATION_OFFSET_FETCH_RETRY_INTERVAL;
    return;
  }

  altitudeOffsetMeters = offset;
  altitudeOffsetFixed = true;
  alt = rawAltitudeMeters + altitudeOffsetMeters;  // 次回ALT周期を待たずに現在値へ反映する
  Serial.printf("[GSI] altitude offset fixed: %.2fm (gsi=%.2fm raw=%.2fm)\n", altitudeOffsetMeters, gsiElevationMeters, rawAltitudeMeters);
}

// ディスプレイ更新（表示モードごとに分岐）
void updateDisplay() {
  lcd_s.fillScreen(TFT_BLACK);
  lcd_s.setTextColor(TFT_WHITE);
  
  // ボタン状態により表示モードを切り替え
  if (M5.BtnB.isPressed()) { dispmode = 0; }
  if (M5.BtnA.isPressed()) { dispmode = 1; }
  if (M5.BtnC.isPressed()) { dispmode = 2; }

  // 標高グラフモードの場合は専用の描画関数を呼び出して終了する
  if (dispmode == 2) {
    drawAltitudeGraphMode();      // 標高グラフモードの描画
    lcd.startWrite();
    lcd_s.pushSprite(0, 0);
    lcd.endWrite();
    return;
  }

  // 走行支援が有効な場合、モード0は走行支援画面に差し替える
  if (dispmode == 0 && strategy.config().enabled) {
    drawStrategyMode();
    lcd.startWrite();
    lcd_s.pushSprite(0, 0);
    lcd.endWrite();
    return;
  }
  
  // 接続状況表示
  if (dispmode == 0 || dispmode == 1) {
    lcd_s.setFont(&fonts::lgfxJapanGothicP_16);
    lcd_s.setTextSize(1);
    lcd_s.setTextDatum(TL_DATUM);
    lcd_s.drawString(Loc == "su" ? "鈴鹿" : (Loc == "mo" ? "茂木" : "豊田"), 10, 0);
    if (strategySource == STRATEGY_SRC_SD_ERROR) {
      // 走行支援の設定ファイルはあるが内容に異常があり、支援が無効になっている
      lcd_s.setTextColor(TFT_RED);
      lcd_s.drawString("支援設定異常", 50, 0);
      lcd_s.setTextColor(TFT_WHITE);
    }
    lcd_s.setTextDatum(top_right);
    lcd_s.drawString(LOGGING ? "SD: O" : "SD: x", 320, 0);
    lcd_s.drawString(gps.location.isValid() ? "GNSS: O" : "GNSS: x", 320, 15);
    if (MQTTpush) lcd_s.drawString("MQTT: O", 320, 30);
    if (ambientpush) lcd_s.drawString("Amb: O", 320, 30);
    if (!ambientpush && !MQTTpush) {
      lcd_s.drawString("MQTT: x", 320, 30);
      // lcd_s.drawString("Amb: x", 320, 45);
    }
  }
  
  // タイトル表示（表示モードごと）
  lcd_s.setFont(&fonts::lgfxJapanGothicP_20);
  lcd_s.setTextSize(1);
  lcd_s.setTextDatum(BL_DATUM);
  if (dispmode == 0) {
    lcd_s.drawString("速度(km/h):", 10, 50);
    lcd_s.drawString("残り周回数:", 10, 130);
    lcd_s.drawString("走行時間:", 10, 210);
  } else if (dispmode == 1) {
    lcd_s.drawString("回転数(rpm):", 10, 50);
    lcd_s.drawString("噴射時間(ms):", 10, 130);
    lcd_s.drawString("進角角度(CA):", 10, 210);
  }
  
  // 第１表示行
  lcd_s.setFont(&fonts::Font7);
  lcd_s.setTextSize(1);
  lcd_s.setTextDatum(BL_DATUM);
  lcd_s.setCursor(140, 50);
  if (dispmode == 0) {
    lcd_s.print(speed, 1);
    lcd_s.drawRect(9, 54, 302, 12, TFT_WHITE);
    int barLength = (int)((constrain(speed, 0.0, 45.0) / 45.0) * 300.0);
    lcd_s.fillRect(10, 55, barLength, 10, TFT_WHITE);
    lcd_s.fillRect(10 + barLength, 55, 300 - barLength, 10, TFT_BLACK);
  } else if (dispmode == 1) {
    lcd_s.print(tachoRpm);
    lcd_s.drawRect(9, 54, 302, 12, TFT_WHITE);
    int barLength = map(tachoRpm, 0, 6500, 0, 300);
    lcd_s.fillRect(10, 55, barLength, 10, TFT_WHITE);
    lcd_s.fillRect(map(tachoRpm, 0, 6500, 10, 310), 55, map(tachoRpm, 0, 6500, 300, 0), 10, TFT_BLACK);
  }
  
  // 第２表示行（周回数・噴射時間など）
  if (dispmode == 0) {
    // 周回数が規定を超えても負にならないようにする（符号なしだと255と表示される）
    int restlaps = max(0, static_cast<int>(totallaps) - static_cast<int>(Lapcount));
    lcd_s.setCursor(140, 130);
    if (restlaps > 1)
      lcd_s.print(restlaps);
    else if (restlaps == 1)
      lcd_s.print("G");
    else
      lcd_s.print("FINISH");
    lcd_s.drawRect(9, 134, 302, 12, TFT_WHITE);
    lcd_s.fillRect(10, 135, map(distance, 0, goal, 300, 0), 10, TFT_WHITE);
    lcd_s.fillRect(map(distance, 0, goal, 310, 10), 135, map(distance, 0, goal, 0, 300), 10, TFT_BLACK);
    for (uint8_t i = 0; i <= totallaps; i++) {
      lcd_s.setFont(&fonts::lgfxJapanGothicP_20);
      lcd_s.setTextSize(0.6);
      lcd_s.setTextDatum(TC_DATUM);
      lcd_s.setCursor((300 * i / totallaps) + 7, 147);
      lcd_s.print(i);
    }
  } else if (dispmode == 1) {
    lcd_s.setCursor(140, 130);
    lcd_s.print(INJ_timems, 1);
    lcd_s.drawRect(9, 134, 302, 12, TFT_WHITE);
    lcd_s.fillRect(10, 135, map(INJ_timems, 0, 10, 0, 300), 10, TFT_WHITE);
    lcd_s.fillRect(map(INJ_timems, 0, 10, 10, 310), 135, map(INJ_timems, 0, 10, 300, 0), 10, TFT_BLACK);
  }
  
  // 第３表示行（走行時間・進角・燃費）
  lcd_s.setFont(&fonts::Font7);
  lcd_s.setTextSize(1);
  lcd_s.setTextDatum(BL_DATUM);
  lcd_s.setCursor(140, 210);
  if (dispmode == 0) {
    const uint16_t worktimeSec = static_cast<uint16_t>(worktime);  // 表示は秒単位
    uint8_t workmin = worktimeSec / 60;
    uint8_t worksec = worktimeSec % 60;
    lcd_s.setTextColor((map(distance, 0, goal, 300, 0) <= map(worktimeSec, 0, limittime, 300, 0)) ? TFT_WHITE : TFT_MAGENTA);
    lcd_s.printf("%02d:%02d", workmin, worksec);
    lcd_s.drawRect(9, 214, 302, 12, TFT_WHITE);
    if (map(distance, 0, goal, 300, 0) <= map(worktimeSec, 0, limittime, 300, 0))
      lcd_s.fillRect(10, 215, map(worktimeSec, 0, limittime, 300, 0), 10, TFT_WHITE);
    else
      lcd_s.fillRect(10, 215, map(worktimeSec, 0, limittime, 300, 0), 10, TFT_MAGENTA);
    lcd_s.fillRect(map(worktimeSec, 0, limittime, 310, 10), 215, map(worktimeSec, 0, limittime, 0, 300), 10, TFT_BLACK);
  } else if (dispmode == 1) {
    lcd_s.print(IGN_CA);
    lcd_s.drawRect(9, 214, 302, 12, TFT_WHITE);
    lcd_s.fillRect(10, 215, map(IGN_CA, 0, 90, 0, 300), 10, TFT_WHITE);
    lcd_s.fillRect(map(IGN_CA, 0, 90, 10, 310), 215, map(IGN_CA, 0, 90, 300, 0), 10, TFT_BLACK);
  }
  
  // スプライトをLCDへ転送
  lcd.startWrite();
  lcd_s.pushSprite(0, 0);
  lcd.endWrite();
}

// デバッグ用Serial送信（CSV形式）
void updateSerialOutput() {
  refreshDatetime();  // 出力する日時を更新
  Serial.print(la, 7); Serial.print(",");
  Serial.print(ln, 7); Serial.print(",");
  Serial.print(alt, 1); Serial.print(",");
  Serial.print(Loc);   Serial.print(",");
  Serial.print(spd, 1); Serial.print(",");
  Serial.print(tachoRpm); Serial.print(",");
  Serial.print(speed, 1);    Serial.print(",");
  Serial.print(distance); Serial.print(",");
  Serial.print(gasml, 1); Serial.print(",");
  Serial.print(dispergas, 1); Serial.print(",");
  Serial.print(worktime, 1); Serial.print(",");
  Serial.print(EngTemp, 2); Serial.print(",");
  if (isfinite(envPressureKPa)) { Serial.print(envPressureKPa, 3); }
  Serial.print(",");
  if (isfinite(envTemperatureC)) { Serial.print(envTemperatureC, 2); }
  Serial.print(",");
  if (isfinite(envHumidityPct)) { Serial.print(envHumidityPct, 1); }
  Serial.print(",");
  Serial.print(PriPre, 2);  Serial.print(",");
  Serial.print(SecPre, 2);  Serial.print(",");
  Serial.print(FuelPre, 2); Serial.print(",");
  Serial.println(datetime);
}

// SDバッチをフラッシュする（必要時のみsync）
void flushSdBatch(bool doSync) {
  if (!logFile || sdBatchLen == 0) {
    return;
  }

  size_t written = logFile.write(reinterpret_cast<const uint8_t*>(sdBatchBuffer), sdBatchLen);
  if (written != sdBatchLen) {
    Serial.printf("SD batch write short: %u/%u\n", static_cast<unsigned int>(written), static_cast<unsigned int>(sdBatchLen));
  }
  sdBatchLen = 0;

  if (doSync) {
    logFile.timestamp(T_WRITE, year(), month(), day(), hour(), minute(), second());
    logFile.sync();
    sdLinesSinceSync = 0;
    lastSdSyncAt = millis();
  }
}

// SDカードへのログ書き出し
void updateSDLog() {
  refreshDatetime();  // 日時を更新

  // SD行の時刻は厳密単調増加にする（同一10ms内の連続呼び出しでも重複させない）
  if (lastDatetimeCentis <= lastSdCentis) {
    formatDatetime(lastSdCentis + 1ULL);
    lastSdCentis++;
  } else {
    lastSdCentis = lastDatetimeCentis;
  }

  bool isNewFile = false;
  if (!logFileInitialized) {
    isNewFile = !sd.exists(fileName);
  }

  // ログファイルが開いていない場合は開く（すでに開いている場合は再利用して追記）
  if (!logFile) {
    logFile = sd.open(fileName, O_WRITE | O_CREAT | O_APPEND);
  }
  if (logFile) {
    if (!logFileInitialized) {
      if (isNewFile) {
        logFile.timestamp(T_CREATE, 2024, 1, 31, 23, 59, 59);
        logFile.write(0xEF); logFile.write(0xBB); logFile.write(0xBF);
        logFile.println(F("記録日時,速度(km/h),ラップ数(周目),走行時間(秒),回転数(rpm),燃料噴射時間(ms),点火進角角度(CA),燃料噴射終了角度(CA),走行距離積算(m),積算燃料消費量(ml),燃費(km/l),lat(緯度),lng(経度),標高(m),場所,温度(C),気圧(kPa),気温(C),湿度(%),1次空気圧(MPa),2次空気圧(MPa),燃圧(MPa)"));
        saveNextLogIndex(fileNum + 1);
      }
      logFileInitialized = true;
    }

    char pressureStr[16] = {0};
    char tempStr[16] = {0};
    char humStr[16] = {0};
    if (isfinite(envPressureKPa)) { dtostrf(envPressureKPa, 0, 3, pressureStr); }
    if (isfinite(envTemperatureC)) { dtostrf(envTemperatureC, 0, 2, tempStr); }
    if (isfinite(envHumidityPct)) { dtostrf(envHumidityPct, 0, 1, humStr); }

    char logLine[SD_LINE_BUFFER_SIZE] = {0};
    int lineLen = snprintf(logLine,
                           sizeof(logLine),
                           "%s,%.1f,%u,%.1f,%u,%.1f,%u,%d,%u,%.1f,%.1f,%.7f,%.7f,%.1f,%s,%.2f,%s,%s,%s,%.3f,%.3f,%.3f\n",
                           datetime,
                           speed,
                           static_cast<unsigned int>(Lapcount),
                           worktime,
                           static_cast<unsigned int>(tachoRpm),
                           INJ_timems,
                           static_cast<unsigned int>(IGN_CA),
                           static_cast<int>(injEndCA),
                           static_cast<unsigned int>(distance),
                           gasml,
                           dispergas,
                           la,
                           ln,
                           alt,
                           Loc.c_str(),
                           EngTemp,
                           pressureStr,
                           tempStr,
                           humStr,
                           PriPre,
                           SecPre,
                           FuelPre);

    if (lineLen <= 0 || lineLen >= static_cast<int>(sizeof(logLine))) {
      Serial.println("SD log line build failed");
      return;
    }

    if (sdBatchLen + static_cast<size_t>(lineLen) > SD_BATCH_BUFFER_SIZE) {
      flushSdBatch(false);
    }
    memcpy(sdBatchBuffer + sdBatchLen, logLine, static_cast<size_t>(lineLen));
    sdBatchLen += static_cast<size_t>(lineLen);

    // 一定行数または一定時間でのみsyncして負荷を下げる
    sdLinesSinceSync++;
    if (sdLinesSinceSync >= SD_SYNC_LINE_THRESHOLD || (millis() - lastSdSyncAt) >= SD_SYNC_TIME_MS) {
      flushSdBatch(true);
    }
  } else {
    Serial.println("SD Log open failed!");
  }
}

// MQTT送信（非同期リトライ）
#if MQTT_DIAGNOSTIC_LOG
void logMqttTlsErrorDetails() {
  char errBuf[128] = {0};
  int lastErr = mqttTlsClient.lastError(errBuf, sizeof(errBuf));
  Serial.printf("MQTT TLS lastError=%d detail=%s\n", lastErr, errBuf[0] ? errBuf : "(none)");

  IPAddress mqttIp;
  int dnsResult = WiFi.hostByName(mqtt_server, mqttIp);
  if (dnsResult == 1) {
    Serial.printf("MQTT DNS ok: %s\n", mqttIp.toString().c_str());
  } else {
    Serial.printf("MQTT DNS failed: result=%d\n", dnsResult);
  }
}

void logMqttRuntimeStats(const char* phase) {
  Serial.printf("MQTT[%s] heap=%u minHeap=%u maxAlloc=%u RSSI=%d\n",
                phase,
                static_cast<unsigned int>(ESP.getFreeHeap()),
                static_cast<unsigned int>(ESP.getMinFreeHeap()),
                static_cast<unsigned int>(ESP.getMaxAllocHeap()),
                WiFi.RSSI());
}

void runMqttPathDiagnostics() {
  static unsigned long lastDiagAt = 0;
  if (millis() - lastDiagAt < 60000UL) {
    return;
  }
  lastDiagAt = millis();

  WiFiClient tcpClient;
  bool tcpOk = tcpClient.connect(mqtt_server, mqtt_port);
  Serial.printf("MQTT diag TCP %s:%d -> %s\n", mqtt_server, mqtt_port, tcpOk ? "ok" : "failed");
  tcpClient.stop();

  WiFiClientSecure insecureTls;
  insecureTls.setInsecure();
  insecureTls.setTimeout(15);
  bool tlsOk = insecureTls.connect(mqtt_server, mqtt_port);
  if (tlsOk) {
    Serial.println("MQTT diag TLS(insecure) -> ok");
  } else {
    char errBuf[128] = {0};
    int lastErr = insecureTls.lastError(errBuf, sizeof(errBuf));
    Serial.printf("MQTT diag TLS(insecure) -> failed, err=%d detail=%s\n", lastErr, errBuf[0] ? errBuf : "(none)");
  }
  insecureTls.stop();
}
#endif

void updateMQTT() {
  refreshDatetime();  // MQTT送信前に日時を更新

  // Wi-Fi未接続時は即時復帰し、メインループを止めない
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  if (!mqttclient.connected()) {
    static unsigned long lastReconnectAttempt = 0;
    static unsigned long reconnectInterval = MQTT_RECONNECT_BASE_INTERVAL;
    if (millis() - lastReconnectAttempt > reconnectInterval) {
      lastReconnectAttempt = millis();
#if MQTT_DIAGNOSTIC_LOG
      logMqttRuntimeStats("before-connect");
#endif
      if (mqttclient.connect(mqtt_deviceID)) {
        reconnectInterval = MQTT_RECONNECT_BASE_INTERVAL;
        Serial.println("MQTT connected");
      } else {
        reconnectInterval = min(reconnectInterval * 2UL, MQTT_RECONNECT_MAX_INTERVAL);
        Serial.print("MQTT reconnect failed, state: ");
        Serial.println(mqttclient.state());
#if MQTT_DIAGNOSTIC_LOG
        logMqttRuntimeStats("connect-failed");
        logMqttTlsErrorDetails();
        runMqttPathDiagnostics();
#endif
        mqttTlsClient.stop();
      }
    }
    return;
  }

  // 多重送信ガード: 前回送信から500ms未満の場合はスキップ
  static unsigned long lastPublishAt = 0;
  if (millis() - lastPublishAt < 500UL) {
    return;
  }

  StaticJsonDocument<512> doc;
  doc["timestamp"] = datetime;
  //doc["Spd_GPS"]   = (float)spd;
  doc["Spd_PULSE"] = (float)speed;
  doc["Lapcount"]  = (int)Lapcount;
  doc["worktime"]  = (int)worktime;
  doc["tachoRpm"]  = (int)tachoRpm;
  doc["distance"]  = (int)distance;
  doc["gasml"]     = (float)gasml;
  doc["dispergas"] = (float)dispergas;
  doc["lat"]       = (float)la;
  doc["lon"]       = (float)ln;
  doc["alt"]       = (float)alt;
  doc["loc"]       = Loc;
  doc["temp"]      = (float)EngTemp;
  doc["pri"]       = (float)PriPre;
  doc["sec"]       = (float)SecPre;
  doc["fuel"]      = (float)FuelPre;
  String jsonData;
  serializeJson(doc, jsonData);
  mqttclient.publish(mqtt_topic, jsonData.c_str());
  lastPublishAt = millis();
}

// Ambient送信
void updateAmbient() {
  if (WiFi.status() == WL_CONNECTED) {
    char buf[16];
    ambient.set(1, speed);
    ambient.set(2, EngTemp);
    ambient.set(3, Lapcount);
    ambient.set(4, static_cast<int>(worktime));
    ambient.set(5, tachoRpm);
    ambient.set(6, distance);
    ambient.set(7, gasml);
    ambient.set(8, dispergas);
    dtostrf(la, 12, 8, buf);
    ambient.set(9, buf);
    dtostrf(ln, 12, 8, buf);
    ambient.set(10, buf);
    // ambient.set(11, alt); // 高度追加 (フィールド11は使用上無いのでコメントアウト、文字列化は一旦せずに数値のまま送信、必要であれば文字列変換する)
    if (ambient.send(1000)) {
      Serial.println("Ambient: Success!");
    } else {
      Serial.println("Ambient: failure...");
    }
  } else {
    Serial.println("WiFi disconnected for Ambient");
    WiFi.reconnect();
  }
}

// I2CデバイスからチップIDレジスタ(0xD0)を読み取る
bool readPressureSensorChipId(uint8_t i2cAddress, uint8_t& outChipId) {
  Wire.beginTransmission(i2cAddress);
  Wire.write(0xD0);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(static_cast<int>(i2cAddress), 1) != 1) {
    return false;
  }
  outChipId = static_cast<uint8_t>(Wire.read());
  return true;
}

// BME280向けにI2CをG21/G22で初期化し、0x76/0x77で探索する
bool initializePressureSensorForGnssModule() {
  const uint8_t bme280ChipId = 0x60;

  Wire.begin(M5.Ex_I2C.getSDA(), M5.Ex_I2C.getSCL());

  const uint8_t addresses[] = {0x76, 0x77};
  for (uint8_t i = 0; i < sizeof(addresses); i++) {
    uint8_t chipId = 0;
    if (!readPressureSensorChipId(addresses[i], chipId)) {
      continue;
    }

    if (chipId == bme280ChipId) {
      if (bme280.begin(addresses[i], &Wire)) {
        Serial.printf("BME280 initialized (addr=0x%02X, id=0x%02X)\n", addresses[i], chipId);
        return true;
      }
      Serial.printf("BME280 begin failed (addr=0x%02X, id=0x%02X)\n", addresses[i], chipId);
      continue;
    }

    Serial.printf("Unknown pressure sensor id=0x%02X at addr=0x%02X\n", chipId, addresses[i]);
  }

  Serial.println("Pressure sensor init failed on addr 0x76/0x77");
  return false;
}

#if STRATEGY_SIM
// 机上確認用の仮想走行
// ウェイポイント上を走らせ、位置・速度・噴射・走行時間・燃圧を差し替えて走行支援画面を確認する。
// 仮想ドライバーは画面の合図どおりに加速・停止する。走行抵抗を2割増しにして、通過タイムの遅れと補正も出るようにしてある。
void updateSimulatedRun() {
  static unsigned long lastMs = millis();
  static float simS = -8.0f;   // スタートラインからの累積距離 [m]（グリッドはラインの8 m手前）
  static float simV = 0.0f;    // 速度 [m/s]
  static float simT = 0.0f;    // 走行時間 [s]
  static float reactS = 0.0f;  // 合図に気づいてから操作するまでの時間 [s]
  static bool launched = false;
  static bool engine = false;

  // もてぎとして扱う（ウェイポイントと走行支援の設定はloop()側で読み込まれる）
  Loc = "mo";
  totallaps = totallaps_mo;
  goal = goal_mo;
  limittime = limittime_mo;

  const unsigned long nowMs = millis();
  const float dt = static_cast<float>(nowMs - lastMs) / 1000.0f * static_cast<float>(STRATEGY_SIM);
  lastMs = nowMs;
  const float lapM = strategy.config().lapM;
  if (waypointCount < 2 || waypointRawLapMeters <= 0.0f || lapM <= 0.0f) {
    return;
  }

  // 仮想ドライバー
  const StrategyAssist::Cue cue = strategy.output().cue;
  if (!launched) {
    if (nowMs > 8000UL) {  // 起動後しばらくは「スタート待ち」を表示する
      launched = true;
      engine = true;
    }
  } else if (cue == StrategyAssist::Cue::Finished) {
    engine = false;
  } else if (!engine && cue == StrategyAssist::Cue::BurnNow) {
    reactS += dt;
    if (reactS >= 0.5f) { engine = true; reactS = 0.0f; }
  } else if (engine && cue == StrategyAssist::Cue::CutNow) {
    reactS += dt;
    if (reactS >= 0.3f) { engine = false; reactS = 0.0f; }
  } else {
    reactS = 0.0f;
  }

  // 運動（勾配は無視）
  const float kmh = simV * 3.6f;
  float accel = engine ? (1.6f - 0.03f * kmh) : 0.0f;
  accel -= 1.2f * (9.8f * 0.00164f + 0.000391f * simV * simV);
  simV = max(simV + accel * dt, 0.0f);
  simS += simV * dt;
  if (launched) {
    simT += dt;
  }

  // 累積距離をウェイポイント間の補間で緯度経度に直す
  const float rawPos = fmodf(simS + lapM, lapM) * (waypointRawLapMeters / lapM);
  size_t i = 0;
  while (i + 1 < waypointCount && waypoints[i + 1].dist <= rawPos) {
    i++;
  }
  const Waypoint& from = waypoints[i];
  const Waypoint& to = waypoints[(i + 1) % waypointCount];
  const float segEnd = (i + 1 < waypointCount) ? to.dist : waypointRawLapMeters;
  const float ratio = (segEnd > from.dist) ? (rawPos - from.dist) / (segEnd - from.dist) : 0.0f;
  const double prevLa = la;
  const double prevLn = ln;
  la = from.lat + (to.lat - from.lat) * ratio;
  ln = from.lng + (to.lng - from.lng) * ratio;

  // ECU・BLEの受信値を差し替える
  receiveECUtime = nowMs;
  speed = simV * 3.6f;
  spd = speed;
  tachoRpm = engine ? 3500 : 0;
  INJ_timems = engine ? 3.0f : 0.0f;
  worktime = simT;
  distance = launched ? static_cast<uint16_t>(simS + 8.0f) : 0;
  FuelPre = (engine && Lapcount == 2) ? 0.26f : 0.32f;  // 3周目の加速だけ燃圧低下を再現する

  updateLapCountByControlLineCrossing(true, prevLa, prevLn);
}
#endif

//==================== setup() =====================
void setup() {
  // M5初期化
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.update();
  M5.BtnB.setHoldThresh(1000);  // Bボタン長押し(1秒)で走行支援の加速パターンを切り替える
#if STRATEGY_BEEP_AVAILABLE
  M5.Speaker.setVolume(255);
#endif
  
  // デバッグ用Serial
  Serial.begin(115200);

  // 気圧センサー初期化（BME280）
  if (initializePressureSensorForGnssModule()) {
    bme280.setSampling(Adafruit_BME280::MODE_NORMAL,
                       Adafruit_BME280::SAMPLING_X2,
                       Adafruit_BME280::SAMPLING_X16,
                       Adafruit_BME280::SAMPLING_X1,
                       Adafruit_BME280::FILTER_X16,
                       Adafruit_BME280::STANDBY_MS_500);
    isBmx280Ready = true;
  }
  updateEnvironmentSensors();

  // ECU初期化
  // MQTT/HTTP等のブロッキング中もECUの行を取りこぼさないよう、begin()より前にRXバッファを拡大する
  Serial1.setRxBufferSize(1024);
  Serial1.begin(ECU_BPS, SERIAL_8N1, ECU_RX_PIN, ECU_TX_PIN);
  Serial1.setTimeout(5);  // readStringUntilのブロッキング待ちを最小化
  
  // GNSS初期化
  Serial2.begin(gnss_bps);

  // BLE中継機(NanoC6)用I2Cバスの初期化
  // Basicは気圧センサー用Wireバスと同一ピンのため、Wire.begin()の再呼び出しによる速度低下(400kHz→100kHz)を避けて共用する
  #if defined(ARDUINO_M5STACK_Core2)
    BleI2C.begin(BLE_I2C_SDA, BLE_I2C_SCL);
  #endif
  // NanoC6未接続/無応答時にI2Cタイムアウトの既定値(50ms)×4チャンネル分ブロックし、
  // loop()の実行間隔が伸びてボタン反応が悪化する（M5.update()の呼び出し頻度が落ちる）ことを防ぐ
  BleI2C.setTimeOut(20);
  // M5Unified側の初期化で400kHz(Fast Mode)になっている可能性が高い。Groveケーブル経由の
  // 配線ではFast Modeはノイズ・リンギングの影響を受けやすく、NanoC6側で本来存在しない
  // コマンドバイトを誤検出する一因になっていると考えられるため、Standard Mode(100kHz)に
  // 明示的に固定してノイズ耐性のマージンを広げる（共有バスのBME280も100kHzで問題なく動作する）
  BleI2C.setClock(100000);

  // LCD初期化
  lcd.init();
  lcd.setRotation(1);
  lcd.setBrightness(128);
  lcd.fillScreen(TFT_BLACK);
  // TLS接続時のヒープ確保余裕を増やすため、スプライトを4bitへ縮小
  lcd_s.setColorDepth(8);
  lcd_s.createSprite(lcd.width(), lcd.height());
  lcd.setFont(&fonts::lgfxJapanGothicP_20);
  lcd.setTextSize(1);
  lcd.setTextDatum(baseline_center);
  
  // SDカード初期化（最大3秒待機）
  bool sdInitialized = sd.begin(SD_CONFIG);
  if (!sdInitialized) {
    unsigned long sdStart = millis();
    while (!sdInitialized && (millis() - sdStart < 3000)) {
      sdInitialized = sd.begin(SD_CONFIG);
      if (sdInitialized) { break; }
      M5.update();

      Serial.println(F("SD Wait..."));
      lcd.fillScreen(TFT_RED);
      lcd.setTextColor(TFT_BLACK);
      showMessage(FPSTR(MSG_NO_SD));
      lcd.drawNumber(int((3000 - (millis() - sdStart)) / 1000), lcd.width()/2, lcd.height()/2+20);

      delay(1000);
    }
  }
  if (!sdInitialized) {
    LOGGING = false;
    Serial.println("SD init failed");  
  } else {
    if (!sd.exists("/LOG")) {
      sd.mkdir("/LOG");
      showMessage(FPSTR(MSG_LOG_DIR_CREATE));
    }

    // ログが消去されている場合はインデックスファイルを使わず先頭から採番
    if (!sd.exists("/LOG/LOG0000.CSV")) {
      fileNum = 0;
    } else {
      fileNum = loadNextLogIndex();
    }

    showMessage(FPSTR(MSG_LOG_FILE_CREATE));
    while (true) {
      snprintf(fileName, sizeof(fileName), "/LOG/LOG%04d.CSV", fileNum);
      if (!sd.exists(fileName)) {
        logFileInitialized = false;
        break;
      }
      fileNum++;
    }
  }
  
  // WiFi設定（WiFiManagerを利用）
  wifiManager.setAPCallback([](WiFiManager* mgr){
    Serial.println("Entered config mode");
    String portalSSID = mgr->getConfigPortalSSID();
    lcd.fillScreen(TFT_BLACK);
    lcd.setTextColor(TFT_WHITE);
    showMessage(String(FPSTR(MSG_WIFI_CONFIG)) + portalSSID);
    char ConfigSSID[40];
    sprintf(ConfigSSID, "WIFI:S:%s;T:nopass;R:1;;", portalSSID);
    lcd.qrcode(ConfigSSID, 105, 92, 135, 5);
  });
  
  bool doManualConfig = false;
  lcd.fillScreen(TFT_BLACK);
  showMessage("Aボタンを押してWi-Fi設定\nCボタンを押してWi-Fi無効化");
  unsigned long btnStart = millis();
  while (millis() - btnStart < 5000) {
    M5.update();
    if (M5.BtnA.isPressed()) { doManualConfig = true; break; }
    if (M5.BtnC.isPressed()) { ambientpush = false; MQTTpush = false; break; }
    delay(10);
  }
  if (doManualConfig) {
    if (wifiManager.startConfigPortal()) {
      isWifiConfigSucceeded = true;
    } else {
      isWifiConfigSucceeded = false;
    }
  } else {
    if (ambientpush || MQTTpush) {
      lcd.fillScreen(TFT_BLACK);
      showMessage(FPSTR(MSG_WIFI_CONNECTING));
      if (wifiManager.autoConnect()) { isWifiConfigSucceeded = true; }
      else { isWifiConfigSucceeded = false; ambientpush = false; MQTTpush = false; }
    } else {
      lcd.fillScreen(TFT_BLACK);
      lcd.drawString("Wi-Fi無効", lcd.width()/2, lcd.height()/2);
    }
  }

  if (isWifiConfigSucceeded) {
    // ESP32の省電力動作でTLS受信が不安定になることがあるため無効化する
    WiFi.setSleep(false);
    Serial.println("WiFi modem sleep disabled");
  }
  
  // Ambient設定
  if (ambientpush && isWifiConfigSucceeded) {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    sprintf(devKey, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
    if (!ambient.getchannel(userKey, devKey, channelId, writeKey, sizeof(writeKey), &ambientClient)) {
      Serial.printf("Cannot get channelId for device %s\n", devKey);
      while (!ambient.getchannel(userKey, devKey, channelId, writeKey, sizeof(writeKey), &ambientClient)) {
        M5.update();
        delay(500);
      }
    }
    ambient.begin(channelId, writeKey, &ambientClient);
  }
  
  // MQTT設定
  if (MQTTpush && isWifiConfigSucceeded) {
    mqttclient.setBufferSize(MQTT_BUFFER_SIZE);
    mqttclient.setSocketTimeout(15);
    mqttclient.setKeepAlive(60);
    mqttTlsClient.setCACert(AWS_CERT_CA);
    mqttTlsClient.setCertificate(AWS_CERT_CRT);
    mqttTlsClient.setPrivateKey(AWS_CERT_PRIVATE);
    mqttclient.setServer(mqtt_server, mqtt_port);
  }
  
  // NTP同期（Wi-Fi接続成功時、GPS時刻受信前）
  if (isWifiConfigSucceeded && !ntpSyncDone) {
    if (!trySyncTimeFromNtp(NTP_SYNC_TIMEOUT_MS)) {
      nextNtpRetryAt = millis() + NTP_RETRY_INTERVAL_MS;
    }
  }

  // 初回MQTT接続はNTP/GNSSで時刻が確定した後に実行する
  if (MQTTpush && isWifiConfigSucceeded) {
    delay(300);
    updateMQTT();
  }

#if STRATEGY_SIM
  // 仮想走行の値をクラウドへ送らない
  MQTTpush = false;
  ambientpush = false;
#endif

  lcd.fillScreen(TFT_BLACK);
  showMessage(FPSTR(MSG_LOADING));
  Serial.println(F("lat, lon, alt, loc, Spd_GPS, rpm, Spd_PULSE, distance, gasml, dispergas, worktime, Pressure, Temp, Humidity, PRI, SEC, FUEL, datetime"));
  
  t_Serial = millis();
  t_SD = millis();
  t_MQTT = millis();
  t_amb = millis();
  t_alt = millis();
  t_env = millis();
  t_display = millis();
  lastSdSyncAt = millis();
}

//==================== loop() =====================
void loop() {
  M5.update();

  // NTP初回失敗時の再試行（GNSS未受信環境でも時刻が確定するようにする）
  if (!ntpSyncDone && MQTTpush && isWifiConfigSucceeded && millis() >= nextNtpRetryAt) {
    if (trySyncTimeFromNtp(5000UL)) {
      nextNtpRetryAt = 0;
    } else {
      nextNtpRetryAt = millis() + NTP_RETRY_INTERVAL_MS;
    }
  }

  // MQTTコネクション維持（PubSubClient推奨: loop()を毎回呼ぶ）
  if (MQTTpush && mqttclient.connected()) {
    mqttclient.loop();
  }

#if STRATEGY_SIM
  updateSimulatedRun();
  const bool hasPosition = true;
  positionAgeMs = 0;
#else
  updateBLE();
  updateECU();
  updateGNSS();
  const bool hasPosition = gps.location.isValid();
  positionAgeMs = gps.location.age();
#endif
  positionFresh = hasPosition && positionAgeMs < STRATEGY_POSITION_MAX_AGE_MS;
  resetLapCountOnLaunch();

  ensureWaypointLoadedForCurrentLoc();
  ensureStrategyLoadedForCurrentLoc();
  if (hasPosition && waypointCount > 0) {
    nearestWaypointIndex = findNearestWaypointIndex(la, ln);
  } else {
    nearestWaypointIndex = -1;
  }
  updateStrategyAssist();

  if (millis() - t_display >= DISPLAY_INTERVAL) {
    updateDisplay();
    t_display += DISPLAY_INTERVAL;
    if (millis() - t_display >= DISPLAY_INTERVAL) {
      t_display = millis();
    }
  }
  
  // デバッグ用Serial出力
  if (millis() - t_Serial >= SERIAL_OUT_INTERVAL) {
    updateSerialOutput();
    t_Serial += SERIAL_OUT_INTERVAL;
    // 長時間ブロック後に連続実行（バースト）しないよう再同期する
    if (millis() - t_Serial >= SERIAL_OUT_INTERVAL) {
      t_Serial = millis();
    }
  }
  
  // SDカードへのログ書き出し（仮想走行の値は記録しない）
  if (LOGGING && !STRATEGY_SIM && (millis() - t_SD >= SD_LOG_INTERVAL)) {
    updateSDLog();
    t_SD += SD_LOG_INTERVAL;
    // 長時間ブロック後に連続実行（バースト）しないよう再同期する
    if (millis() - t_SD >= SD_LOG_INTERVAL) {
      t_SD = millis();
    }
  }
  
  // MQTT送信は頻度を変えて実行（周回開始前は低頻度、周回開始後は高頻度）
  // Wi-Fi接続が必要なため、接続成功している場合のみ更新する
  if (MQTTpush) {
    if (worktime <= 0.0f) {
      if (millis() - t_MQTT >= MQTT_INTERVAL_PRE) {
        updateMQTT();
        t_MQTT += MQTT_INTERVAL_PRE;
        // 長時間ブロック後に連続実行（バースト）しないよう再同期する
        if (millis() - t_MQTT >= MQTT_INTERVAL_PRE) {
          t_MQTT = millis();
        }
      }
    } else {
      if (millis() - t_MQTT >= MQTT_INTERVAL_RUN) {
        updateMQTT();
        t_MQTT += MQTT_INTERVAL_RUN;
        // 長時間ブロック後に連続実行（バースト）しないよう再同期する
        if (millis() - t_MQTT >= MQTT_INTERVAL_RUN) {
          t_MQTT = millis();
        }
      }
    }
  }
  
  // AmbientはWi-Fi接続が必要なため、接続成功している場合のみ更新する
  if (ambientpush && (millis() - t_amb >= AMBIENT_INTERVAL)) {
    updateAmbient();
    t_amb += AMBIENT_INTERVAL;
  }
  
  // 高度更新はBME280が正常に初期化されている場合のみ実行する
  if (millis() - t_alt >= ALTITUDE_INTERVAL) {
    updateAltitudeFromBmx280();
    t_alt += ALTITUDE_INTERVAL;
  }
  // 環境センサー更新はBME280が正常に初期化されている場合のみ実行する
  if (millis() - t_env >= ENV_SENSOR_INTERVAL) {
    updateEnvironmentSensors();
    t_env += ENV_SENSOR_INTERVAL;
    if (millis() - t_env >= ENV_SENSOR_INTERVAL) {
      t_env = millis();
    }
  }

  // 国土地理院API呼び出しはBME280接続時のみ低頻度で実行し、成功したら以後実行しない
  if (isBmx280Ready) tryFetchAltitudeOffsetFromGsi();

  delay(1);
}
