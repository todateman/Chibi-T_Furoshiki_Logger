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
#include <Adafruit_BMP280.h>
#include <Adafruit_BME280.h>
#include <Adafruit_AHTX0.h>
#include "secrets.h"

//==================== 定数・マクロ ====================
#define pi 3.141592653589793
#define SD_SPI_SPEED SD_SCK_MHZ(25)
#define SD_CONFIG SdSpiConfig(GPIO_NUM_4, SHARED_SPI, SD_SPI_SPEED)

// ECU用ピンとボーレート（M5Stack Basicの起動不良対策でGPIO12の使用禁止）
#if defined(ARDUINO_M5STACK_Core2)
constexpr uint8_t ECU_RX_PIN 2;
constexpr uint8_t ECU_TX_PIN 0;
#elif defined(ARDUINO_M5Stack_Core)
constexpr uint8_t ECU_RX_PIN 15;
constexpr uint8_t ECU_TX_PIN 0;
#else
#error "Unsupported board: ECU_UART_PIN is not defined for this target"
#endif
#define ECU_BPS 115200

// BLE用ピンと利用モード（1: ハードウェアUART使用; 0: SoftwareSerial使用）
#define BLE_RX_PIN 32
#define BLE_TX_PIN 33
#define BLE_BPS 115200
#define USE_HARDWARE_BLE 0

// 各タスクの更新間隔（ミリ秒）
#define SERIAL_OUT_INTERVAL 1000
#define SD_LOG_INTERVAL     1000
#define MQTT_INTERVAL_PRE   10000  // 走行前
#define MQTT_INTERVAL_RUN   1000   // 走行中
#define MQTT_RECONNECT_BASE_INTERVAL 5000UL
#define MQTT_RECONNECT_MAX_INTERVAL  60000UL
#define AMBIENT_INTERVAL    10000
#define ALTITUDE_INTERVAL   500
#define ENV_SENSOR_INTERVAL 1000
#define ELEVATION_OFFSET_FETCH_RETRY_INTERVAL 15000UL

// GNSS受信バッファサイズ（TinyGPS++の内部バッファサイズに合わせる）
#define GNSS_PARSE_BUDGET_BYTES 256

// MQTT設定
#define MQTT_BUFFER_SIZE  512 // MQTT送受信のバッファサイズ
#define MQTT_DIAGNOSTIC_LOG 0  // 本番:0, 切り分け時のみ1

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
unsigned long lastSdSyncAt = 0;
uint16_t sdLinesSinceSync = 0;

// シリアル（ECU, GNSS, BLE）
unsigned long receiveECUtime = 0;
#if USE_HARDWARE_BLE
// BLE通信にハードウェアUARTを使う場合（※UART0: Serialはデバッグ用なので避ける）
HardwareSerial SerialBLE(2);  // ※環境に応じてUART番号とピンを再設定
#else
#include <SoftwareSerial.h>
SoftwareSerial SerialBLE(BLE_RX_PIN, BLE_TX_PIN);
#endif

// ECU受信データ
uint16_t tachoRpm = 0;  // エンジン回転数 [rpm]
float INJ_timems = 0.0; // 燃料噴射時間 [ms]（燃料噴射量の指標として利用）
uint8_t IGN_CA = 0;     // 点火時期 [°CA]（クランク角度）
float speed = 0.0;      // 車軸パルスから算出した車速 [km/h]
uint16_t distance = 0;  // 走行距離 [m]
float gasml = 0.0;      // 燃料消費量 [ml]（燃料噴射時間から推定）※あくまで目安で、実際の消費量とは異なる可能性が高い
float dispergas = 0.0;  // 燃料消費率 [ml/km]（燃料消費量 / 走行距離）※あくまで目安で、実際の消費率とは異なる可能性が高い
uint16_t worktime = 0;  // 走行時間 [s]（エンジン始動以降の時間を累積）
uint16_t Lapcount = 0;  // 周回数
uint8_t totallaps = 3;  // 周回数（サーキットごとに設定値を上書き）
uint16_t goal = 1000;   // 走行距離 [m]（サーキットごとに設定値を上書き）
uint16_t limittime = 100; // 制限時間 [s]（サーキットごとに設定値を上書き）
float EngTemp = 0.0;    // エンジン温度 [°C]

// GPS用
TinyGPSPlus gps;
int gnss_bps = 115200;  // GNSSモジュールのボーレート（モジュールに応じて切り替える NEO-6M:115200, M5Stack GNSS Module:38400）
double la = 0.0, ln = 0.0;      // GPS緯度経度
// double la = 34.990768;    // KMMF2026の緯度経度初期値
// double ln = 137.010875;   // KMMF2026の緯度経度初期値
double alt = 0.0;   // GPS高度
double spd = 0.0;   // GPS速度
String Loc = "";    // ロケーション識別子（"su":鈴鹿, "mo":茂木, "to":豊田）

// BMP280/BME280による高度推定用
Adafruit_BMP280 bmp280;
Adafruit_BME280 bme280;
Adafruit_AHTX0 aht20;
bool isBmx280Ready = false;     // BMP280/BME280が正常に初期化されているかどうか
bool isAht20Ready = false;
enum PressureSensorType {
  PRESSURE_SENSOR_NONE = 0,
  PRESSURE_SENSOR_BMP280,
  PRESSURE_SENSOR_BME280,
};
PressureSensorType pressureSensorType = PRESSURE_SENSOR_NONE;
float envPressureKPa = NAN;                       // BMP280/BME280/AHT20から取得した環境気圧 [kPa]（センサーが複数ある場合はBME280 > AHT20 > BMP280の優先順位で選択。いずれも利用できない場合はNAN）
float envTemperatureC = NAN;                      // BMP280/BME280/AHT20から取得した環境温度 [°C]（センサーが複数ある場合はBME280 > AHT20 > BMP280の優先順位で選択。いずれも利用できない場合はNAN）
float envHumidityPct = NAN;                       // BME280またはAHT20から取得した環境湿度 [%]（BMP280は湿度センサーを搭載していないため、この値は利用できない）
float seaLevelPressureKPa = 101.325f;             // 海面上気圧の初期値（kPa単位）
float altitudeOffsetMeters = 0.0f;                // 国土地理院API標高とBMP280/BME280生高度の差分（m）
bool altitudeOffsetFixed = false;                 // 標高オフセットが確定しているかどうか
unsigned long nextAltitudeOffsetFetchAt = 0;      // 次回の標高オフセット取得を試みる時刻（ミリ秒）

// サーキットごとの設定
const uint8_t totallaps_su = 8; // 鈴鹿サーキット東コースの周回数
const uint8_t totallaps_mo = 7; // ツインリンクもてぎオーバルコースの周回数
const uint16_t goal_su = 17616; // 鈴鹿サーキット東コースの走行距離 [m]
const uint16_t goal_mo = 16389; // ツインリンクもてぎオーバルコースの走行距離 [m]
const uint16_t limittime_su = 2536; // 鈴鹿サーキット東コースの制限時間 [s]
const uint16_t limittime_mo = 2360; // ツインリンクもてぎオーバルコースの制限時間 [s]
const int time_offset = 9;  // JST

// 時刻表示用バッファ
char datetime[23];
uint64_t lastDatetimeCentis = 0;  // CSV時刻の逆行防止（1/100秒単位）

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

// 日時バッファ更新関数の宣言（必要に応じてセンチ秒を指定可能）
void refreshDatetime(uint8_t csec);

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
        setTime(static_cast<time_t>(epoch) + static_cast<time_t>(time_offset * SECS_PER_HOUR));
        refreshDatetime(0);
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

// 日時バッファ更新（必要に応じてセンチ秒を指定）
void refreshDatetime(uint8_t csec = 255) {
  time_t currentSecond = now();
  uint8_t displayCsec = csec;
  if (displayCsec > 99) {
    static time_t centiBaseSecond = 0;
    static unsigned long centiBaseMillis = 0;
    unsigned long nowMs = millis();

    // センチ秒が指定されていない場合は、現在秒の先頭からの経過時間で算出する
    // 1Hzロギング時に "毎秒+0.01" ずつ増える見え方を防ぐ
    if (currentSecond != centiBaseSecond) {
      centiBaseSecond = currentSecond;
      centiBaseMillis = nowMs;
      displayCsec = 0;
    } else {
      unsigned long elapsedMs = nowMs - centiBaseMillis;
      if (elapsedMs > 990) {
        elapsedMs = 990;
      }
      displayCsec = static_cast<uint8_t>(elapsedMs / 10);
    }
  }

  // GNSS/NTP再同期で秒が戻った場合でも、ログ時刻文字列は単調増加を維持する
  // NOTE: ここで setTime() は呼ばない。呼ぶと同一秒内の多重呼び出しで秒が人工的に進み、
  //       CSV時刻のバースト/空白を生むため。
  uint64_t currentCentis = static_cast<uint64_t>(currentSecond) * 100ULL + static_cast<uint64_t>(displayCsec);
  if (currentCentis < lastDatetimeCentis) {
    currentCentis = lastDatetimeCentis;
    displayCsec = static_cast<uint8_t>(lastDatetimeCentis % 100ULL);
  } else {
    displayCsec = static_cast<uint8_t>(currentCentis % 100ULL);
  }
  lastDatetimeCentis = currentCentis;

  tmElements_t tm;
  breakTime(static_cast<time_t>(currentCentis / 100ULL), tm);

  sprintf_P(datetime, PSTR("%d/%d/%d %02d:%02d:%02d.%02d"),
            tmYearToCalendar(tm.Year), tm.Month, tm.Day, tm.Hour, tm.Minute, tm.Second, displayCsec);
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
};

// ウェイポイントデータの最大数（必要に応じて増減させる）
const size_t MAX_WAYPOINTS = 300;
Waypoint waypoints[MAX_WAYPOINTS];
size_t waypointCount = 0;
float waypointMinAlt = 0.0f;
float waypointMaxAlt = 0.0f;
String loadedWaypointLoc = "";
int nearestWaypointIndex = -1;

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
  float distanceDummy = 0.0f;
  int parsed = sscanf(line, "%d,%f,%f,%f,%f", &id, &latVal, &lngVal, &altVal, &distanceDummy);
  if (parsed < 4 || id <= 0) {
    return false;
  }

  outPoint.id = static_cast<uint16_t>(id);
  outPoint.lat = latVal;
  outPoint.lng = lngVal;
  outPoint.alt = altVal;
  return true;
}

// ロケーション識別子に対応するウェイポイントCSVファイルをSDカードから読み込む関数
bool loadWaypointFileForLoc(const String& loc) {
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

// BLEから受信した文字が温度データとして有効な文字かどうかを判定する
bool isBLEValueChar(char c)
{
  return (c >= '0' && c <= '9') || c == '.' || c == '+' || c == '-';
}

// BLEから受信した文字列が有効な温度データかどうかを判定し変換する
bool tryParseBLETemperature(const String& raw, float& outTemp) {
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

  outTemp = parsed;
  return true;
}

// BLEから受信した文字列を処理してエンジン温度に変換する
void processBLEPayload(String& payload, bool timeoutPath) { 
  while (payload.length() > 0 && (payload[payload.length() - 1] == '\n' || payload[payload.length() - 1] == '\r')) {  // 末尾の改行コードを削除
    payload.remove(payload.length() - 1);   // これにより、改行コードが複数重なっている場合でもすべて削除される
  }
  payload.trim();                         // 前後の空白を削除
  if (payload.length() == 0) {
    return;
  }

  float tempValue = 0.0;                  // 変換後の温度値を格納する変数
  if (!tryParseBLETemperature(payload, tempValue)) {  // 文字列が有効な温度データでない場合はエラーとして処理
    payload = "";
    return;
  }

  if (tempValue >= 10.0 && tempValue <= 150.0) {  // 有効な温度範囲内かチェック
    EngTemp = tempValue;                          // グローバル変数にエンジン温度を保存
    if (timeoutPath) {                            // タイムアウト経路で受信したデータはログに残す
      Serial.printf("[BLE TIMEOUT] RX: %s -> %.2f°C\n", payload.c_str(), EngTemp);  // タイムアウト経路で受信したデータはログに残す（SDカードに記録されるため）
    } else {
      Serial.printf("[BLE] RX: %s -> %.2f°C\n", payload.c_str(), EngTemp);          // 通常経路で受信したデータはログに残さない（SDカードに記録されるため）
    }
  } else {
    if (timeoutPath) {
      Serial.printf("[BLE TIMEOUT] OUT OF RANGE: %s -> %.2f\n", payload.c_str(), tempValue);
    } else {
      Serial.printf("[BLE] OUT OF RANGE: %s -> %.2f\n", payload.c_str(), tempValue);
    }
  }

  payload = "";
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

// BLEからのデータ読み取り（バッファ＋タイムアウト処理）
void updateBLE() {
  static String bleBuffer = "";
  static unsigned long lastBLETime = 0;
  
  while (SerialBLE.available() > 0) {
    char c = SerialBLE.read();
    lastBLETime = millis();

    if (c == '\n') {
      processBLEPayload(bleBuffer, false);
      continue;
    }

    if (c == '\r') {
      continue;
    }

    if (isBLEValueChar(c)) {
      bleBuffer += c;
      if (bleBuffer.length() > 15) {
        bleBuffer = "";
      }
    }
  }

  // 不完全なデータの部分タイムアウト処理(100ミリ秒以上経過)
  if (millis() - lastBLETime > 100 && bleBuffer.length() > 0) {
    processBLEPayload(bleBuffer, true);
  }
  // データ受信完全ロス時のリセット処理(2秒以上経過)
  if (millis() - lastBLETime > 2000) {
    EngTemp = 0.0;  // エンジン温度をリセット
    bleBuffer = "";
  }
}

// ECUからのデータ読み取り
void updateECU() {
  if (Serial1.available()) {
    receiveECUtime = millis();
    String str = Serial1.readStringUntil('\n');
    str.trim();
    for (uint8_t i = 0; i < 8; i++) {
      int commaIndex = str.indexOf(",");
      String data = str.substring(0, commaIndex);
      data.trim();
      str = str.substring(commaIndex + 1);
      if (i == 0) { tachoRpm = data.toInt(); }
      if (i == 1) { INJ_timems = data.toFloat(); }
      if (i == 2) { IGN_CA = data.toInt(); }
      if (i == 3) { speed = data.toFloat(); }
      if (i == 4) { distance = data.toInt(); }
      if (i == 5) { gasml = data.toFloat(); }
      if (i == 6) { dispergas = data.toFloat(); }
      if (i == 7) { worktime = data.toInt(); }
    }
    Lapcount = distance / (goal / totallaps);
  } else {
    if (millis() - receiveECUtime > 2000) {
      tachoRpm = 0;
      INJ_timems = 0;
      IGN_CA = 0;
      speed = 0.0;
    }
  }
}

// GNSSからの位置・時刻読み取り
void updateGNSS() {
  uint16_t parsedBytes = 0;
  while (Serial2.available() > 0 && parsedBytes < GNSS_PARSE_BUDGET_BYTES) {
    char ch = Serial2.read();
    parsedBytes++;
    if (gps.encode(ch)) {
      if (gps.time.isUpdated()) {
        updateSystemTimeFromGnss();
        if (gps.location.lng() > 120) {  // 異常値除外
          la = gps.location.lat();
          ln = gps.location.lng();
          spd = gps.speed.kmph();
          uint8_t gnss_csec = gps.time.centisecond();
          refreshDatetime(gnss_csec);  // デバッグ用Serial出力
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
}

// 国土地理院APIを呼び出すために、現在の緯度経度が有効かどうかを判定する
bool hasValidLocationForGsiApi() {
  return isfinite(la) && isfinite(ln) && gps.location.isValid() && la >= -90.0 && la <= 90.0 && ln >= -180.0 && ln <= 180.0;
}

// 検出したセンサーから現在の気圧(Pa)を読み出す
bool readPressurePa(float& outPressurePa) {
  if (pressureSensorType == PRESSURE_SENSOR_BMP280) {
    outPressurePa = bmp280.readPressure();
    return true;
  }
  if (pressureSensorType == PRESSURE_SENSOR_BME280) {
    outPressurePa = bme280.readPressure();
    return true;
  }
  return false;
}

// 利用中の環境センサーから気圧・気温・湿度を取得する
void updateEnvironmentSensors() {
  float pressurePa = NAN;
  if (readPressurePa(pressurePa) && isfinite(pressurePa) && pressurePa >= 30000.0f && pressurePa <= 120000.0f) {
    envPressureKPa = pressurePa / 1000.0f;
  } else {
    envPressureKPa = NAN;
  }

  if (pressureSensorType == PRESSURE_SENSOR_BME280) {
    float temperatureC = bme280.readTemperature();
    float humidityPct = bme280.readHumidity();
    envTemperatureC = (isfinite(temperatureC) && temperatureC > -40.0f && temperatureC < 100.0f) ? temperatureC : NAN;
    envHumidityPct = (isfinite(humidityPct) && humidityPct >= 0.0f && humidityPct <= 100.0f) ? humidityPct : NAN;
    return;
  }

  if (pressureSensorType == PRESSURE_SENSOR_BMP280 && isAht20Ready) {
    sensors_event_t humidityEvent;
    sensors_event_t temperatureEvent;
    aht20.getEvent(&humidityEvent, &temperatureEvent);
    envTemperatureC = (isfinite(temperatureEvent.temperature) && temperatureEvent.temperature > -40.0f && temperatureEvent.temperature < 100.0f)
                        ? temperatureEvent.temperature : NAN;
    envHumidityPct = (isfinite(humidityEvent.relative_humidity) && humidityEvent.relative_humidity >= 0.0f && humidityEvent.relative_humidity <= 100.0f)
                        ? humidityEvent.relative_humidity : NAN;
    return;
  }

  envTemperatureC = NAN;
  envHumidityPct = NAN;
}

// BMP280/BME280から気圧を読み取って高度を更新する
void updateAltitudeFromBmx280() {
  if (!isBmx280Ready) {
    return;
  }
  
  // BMP280/BME280から気圧を読み取る
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

// 国土地理院API標高とBMP280/BME280生高度との差分を取得し、高度オフセットを固定する
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
    Serial.println("[GSI] BMP280/BME280 pressure invalid while fixing offset");
    nextAltitudeOffsetFetchAt = millis() + ELEVATION_OFFSET_FETCH_RETRY_INTERVAL;
    return;
  }

  float pressureKPa = pressurePa / 1000.0f;
  float rawAltitudeMeters = 44330.0f * (1.0f - powf(pressureKPa / seaLevelPressureKPa, 0.1903f));
  if (!isfinite(rawAltitudeMeters) || rawAltitudeMeters < -1000.0f || rawAltitudeMeters > 12000.0f) {
    Serial.printf("[GSI] BMP280/BME280 raw altitude out of range: %.2f\n", rawAltitudeMeters);
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
  
  // 接続状況表示
  if (dispmode == 0 || dispmode == 1) {
    lcd_s.setFont(&fonts::lgfxJapanGothicP_16);
    lcd_s.setTextSize(1);
    lcd_s.setTextDatum(TL_DATUM);
    lcd_s.drawString(Loc == "su" ? "鈴鹿" : (Loc == "mo" ? "茂木" : "豊田"), 10, 0);
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
    uint8_t restlaps = totallaps - Lapcount;
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
    uint8_t workmin = worktime / 60;
    uint8_t worksec = worktime % 60;
    lcd_s.setTextColor((map(distance, 0, goal, 300, 0) <= map(worktime, 0, limittime, 300, 0)) ? TFT_WHITE : TFT_MAGENTA);
    lcd_s.printf("%02d:%02d", workmin, worksec);
    lcd_s.drawRect(9, 214, 302, 12, TFT_WHITE);
    if (map(distance, 0, goal, 300, 0) <= map(worktime, 0, limittime, 300, 0))
      lcd_s.fillRect(10, 215, map(worktime, 0, limittime, 300, 0), 10, TFT_WHITE);
    else
      lcd_s.fillRect(10, 215, map(worktime, 0, limittime, 300, 0), 10, TFT_MAGENTA);
    lcd_s.fillRect(map(worktime, 0, limittime, 310, 10), 215, map(worktime, 0, limittime, 0, 300), 10, TFT_BLACK);
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
  Serial.print(worktime); Serial.print(",");
  Serial.print(EngTemp, 2); Serial.print(",");
  Serial.print(isfinite(envPressureKPa) ? String(envPressureKPa, 3) : ""); Serial.print(",");
  Serial.print(isfinite(envTemperatureC) ? String(envTemperatureC, 2) : ""); Serial.print(",");
  Serial.print(isfinite(envHumidityPct) ? String(envHumidityPct, 1) : ""); Serial.print(",");
  Serial.println(datetime);
}

// SDカードへのログ書き出し
void updateSDLog() {
  refreshDatetime();  // MQTT送信前に日時を更新

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
        logFile.println(F("記録日時,速度(km/h),ラップ数,走行時間,回転数,走行距離,積算燃料,燃費,lat,lon,alt,loc,温度,気圧(kPa),気温(C),湿度(%)"));
        saveNextLogIndex(fileNum + 1);
      }
      logFileInitialized = true;
    }

    logFile.timestamp(T_WRITE, year(), month(), day(), hour(), minute(), second());
    logFile.print(datetime); logFile.print(",");
    logFile.print(speed, 1);    logFile.print(",");
    logFile.print(Lapcount); logFile.print(",");
    logFile.print(worktime); logFile.print(",");
    logFile.print(tachoRpm); logFile.print(",");
    logFile.print(distance, 1); logFile.print(",");
    logFile.print(gasml, 1);   logFile.print(",");
    logFile.print(dispergas, 1); logFile.print(",");
    logFile.print(la, 7);      logFile.print(",");
    logFile.print(ln, 7);      logFile.print(",");
    logFile.print(alt, 1);     logFile.print(",");
    logFile.print(Loc);        logFile.print(",");
    logFile.print(EngTemp, 2); logFile.print(",");
    if (isfinite(envPressureKPa)) { logFile.print(envPressureKPa, 3); }
    logFile.print(",");
    if (isfinite(envTemperatureC)) { logFile.print(envTemperatureC, 2); }
    logFile.print(",");
    if (isfinite(envHumidityPct)) { logFile.print(envHumidityPct, 1); }
    logFile.println();

    // 毎回closeせずに一定間隔でsyncし、書き込み遅延と周期ばらつきを抑える
    sdLinesSinceSync++;
    if (sdLinesSinceSync >= 5 || (millis() - lastSdSyncAt) >= 5000UL) {
      logFile.sync();
      sdLinesSinceSync = 0;
      lastSdSyncAt = millis();
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
    ambient.set(4, worktime);
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

// BMP280/BME280向けにI2CをG21/G22で初期化し、0x76/0x77で探索する
bool initializePressureSensorForGnssModule() {
  const uint8_t bmp280ChipId = BMP280_CHIPID;  // 0x58
  const uint8_t bme280ChipId = 0x60;

  Wire.begin(M5.Ex_I2C.getSDA(), M5.Ex_I2C.getSCL());

  const uint8_t addresses[] = {0x76, 0x77};
  for (uint8_t i = 0; i < sizeof(addresses); i++) {
    uint8_t chipId = 0;
    if (!readPressureSensorChipId(addresses[i], chipId)) {
      continue;
    }

    // チップIDに応じてBMP280/BME280を初期化する
    if (chipId == bmp280ChipId) {
      if (bmp280.begin(addresses[i], BMP280_CHIPID)) {
        pressureSensorType = PRESSURE_SENSOR_BMP280;
        Serial.printf("BMP280 initialized (addr=0x%02X, id=0x%02X)\n", addresses[i], chipId);
        return true;
      }
      Serial.printf("BMP280 begin failed (addr=0x%02X, id=0x%02X)\n", addresses[i], chipId);
      continue;
    }

    if (chipId == bme280ChipId) {
      if (bme280.begin(addresses[i], &Wire)) {
        pressureSensorType = PRESSURE_SENSOR_BME280;
        Serial.printf("BME280 initialized (addr=0x%02X, id=0x%02X)\n", addresses[i], chipId);
        return true;
      }
      Serial.printf("BME280 begin failed (addr=0x%02X, id=0x%02X)\n", addresses[i], chipId);
      continue;
    }

    Serial.printf("Unknown pressure sensor id=0x%02X at addr=0x%02X\n", chipId, addresses[i]);
  }

  pressureSensorType = PRESSURE_SENSOR_NONE;
  Serial.println("Pressure sensor init failed on addr 0x76/0x77");
  return false;
}

// BMP280使用時の温湿度取得用にAHT20を初期化する
bool initializeAht20ForGnssModule() {
  if (aht20.begin(&Wire)) {
    Serial.println("AHT20 initialized");
    return true;
  }

  Serial.println("AHT20 init failed");
  return false;
}

//==================== setup() =====================
void setup() {
  // M5初期化
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.update();
  
  // デバッグ用Serial
  Serial.begin(115200);

  // 気圧センサー初期化（BMP280/BME280を自動判別）
  if (initializePressureSensorForGnssModule()) {
    if (pressureSensorType == PRESSURE_SENSOR_BMP280) {
      bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,
                         Adafruit_BMP280::SAMPLING_X2,
                         Adafruit_BMP280::SAMPLING_X16,
                         Adafruit_BMP280::FILTER_X16,
                         Adafruit_BMP280::STANDBY_MS_500);
    } else if (pressureSensorType == PRESSURE_SENSOR_BME280) {
      bme280.setSampling(Adafruit_BME280::MODE_NORMAL,
                         Adafruit_BME280::SAMPLING_X2,
                         Adafruit_BME280::SAMPLING_X16,
                         Adafruit_BME280::SAMPLING_X1,
                         Adafruit_BME280::FILTER_X16,
                         Adafruit_BME280::STANDBY_MS_500);
    }
    isBmx280Ready = true;
    if (pressureSensorType == PRESSURE_SENSOR_BMP280) {
      isAht20Ready = initializeAht20ForGnssModule();
    }
  }
  updateEnvironmentSensors();

  // ECU初期化
  Serial1.begin(ECU_BPS, SERIAL_8N1, ECU_RX_PIN, ECU_TX_PIN);
  Serial1.setTimeout(5);  // readStringUntilのブロッキング待ちを最小化
  
  // GNSS初期化
  Serial2.begin(gnss_bps);

  // BLE初期化
  #if USE_HARDWARE_BLE
    SerialBLE.begin(BLE_BPS, SERIAL_8N1, BLE_RX_PIN, BLE_TX_PIN);
  #else
    // バッファサイズを512バイトに拡大してデータロス対策
    SerialBLE.begin(BLE_BPS, SWSERIAL_8N1, BLE_RX_PIN, BLE_TX_PIN, false, 512);
  #endif
  
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

  lcd.fillScreen(TFT_BLACK);
  showMessage(FPSTR(MSG_LOADING));
  Serial.println(F("lat, lon, alt, loc, Spd_GPS, rpm, Spd_PULSE, distance, gasml, dispergas, worktime, Pressure, Temp, Humidity, datetime"));
  
  t_Serial = millis();
  t_SD = millis();
  t_MQTT = millis();
  t_amb = millis();
  t_alt = millis();
  t_env = millis();
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

  updateBLE();
  updateECU();
  updateGNSS();

  ensureWaypointLoadedForCurrentLoc();
  if (gps.location.isValid() && waypointCount > 0) {
    nearestWaypointIndex = findNearestWaypointIndex(la, ln);
  } else {
    nearestWaypointIndex = -1;
  }

  updateDisplay();
  
  // デバッグ用Serial出力
  if (millis() - t_Serial >= SERIAL_OUT_INTERVAL) {
    updateSerialOutput();
    t_Serial += SERIAL_OUT_INTERVAL;
    // 長時間ブロック後に連続実行（バースト）しないよう再同期する
    if (millis() - t_Serial >= SERIAL_OUT_INTERVAL) {
      t_Serial = millis();
    }
  }
  
  // SDカードへのログ書き出し
  if (LOGGING && (millis() - t_SD >= SD_LOG_INTERVAL)) {
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
    if (worktime == 0) {
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
  
  // 高度更新はBMP280/BME280が正常に初期化されている場合のみ実行する
  if (millis() - t_alt >= ALTITUDE_INTERVAL) {
    updateAltitudeFromBmx280();
    t_alt += ALTITUDE_INTERVAL;
  }
  // 環境センサー更新はBMP280/BME280のいずれかが正常に初期化されている場合のみ実行する
  if (millis() - t_env >= ENV_SENSOR_INTERVAL) {
    updateEnvironmentSensors();
    t_env += ENV_SENSOR_INTERVAL;
    if (millis() - t_env >= ENV_SENSOR_INTERVAL) {
      t_env = millis();
    }
  }

  // 国土地理院API呼び出しはBMP280/BME280接続時のみ低頻度で実行し、成功したら以後実行しない
  if (isBmx280Ready) tryFetchAltitudeOffsetFromGsi();

  delay(10);
}
