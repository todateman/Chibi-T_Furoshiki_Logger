
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
#include "AltitudeEKF.h"
#include "secrets.h"

// M5.In_I2C を使う BMP280 ミニドライバ
struct BMP280Driver {
  static constexpr uint8_t ADDR = 0x76;
  static constexpr uint32_t FREQ = 400000;
  uint16_t digT1 = 0;
  int16_t digT2 = 0;
  int16_t digT3 = 0;
  uint16_t digP1 = 0;
  int16_t digP2 = 0;
  int16_t digP3 = 0;
  int16_t digP4 = 0;
  int16_t digP5 = 0;
  int16_t digP6 = 0;
  int16_t digP7 = 0;
  int16_t digP8 = 0;
  int16_t digP9 = 0;
  int32_t tFine = 0;

  bool begin() {
    uint8_t id = 0;
    if (!M5.In_I2C.readRegister(ADDR, 0xD0, &id, 1, FREQ)) {
      return false;
    }
    if (id != 0x58) {
      return false;
    }
    uint8_t calib[24] = {0};
    if (!M5.In_I2C.readRegister(ADDR, 0x88, calib, sizeof(calib), FREQ)) {
      return false;
    }

    digT1 = (uint16_t)(calib[1] << 8 | calib[0]);
    digT2 = (int16_t)(calib[3] << 8 | calib[2]);
    digT3 = (int16_t)(calib[5] << 8 | calib[4]);
    digP1 = (uint16_t)(calib[7] << 8 | calib[6]);
    digP2 = (int16_t)(calib[9] << 8 | calib[8]);
    digP3 = (int16_t)(calib[11] << 8 | calib[10]);
    digP4 = (int16_t)(calib[13] << 8 | calib[12]);
    digP5 = (int16_t)(calib[15] << 8 | calib[14]);
    digP6 = (int16_t)(calib[17] << 8 | calib[16]);
    digP7 = (int16_t)(calib[19] << 8 | calib[18]);
    digP8 = (int16_t)(calib[21] << 8 | calib[20]);
    digP9 = (int16_t)(calib[23] << 8 | calib[22]);

    uint8_t ctrlMeas = 0x27;  // temp x1, press x1, normal mode
    uint8_t config = 0xA0;    // standby 1000ms, IIR x4
    if (!M5.In_I2C.writeRegister8(ADDR, 0xF4, ctrlMeas, FREQ)) {
      return false;
    }
    if (!M5.In_I2C.writeRegister8(ADDR, 0xF5, config, FREQ)) {
      return false;
    }
    return true;
  }

  bool readPressurePa(float& pressurePa) {
    uint8_t buf[6] = {0};
    if (!M5.In_I2C.readRegister(ADDR, 0xF7, buf, sizeof(buf), FREQ)) {
      return false;
    }

    int32_t adcP = (int32_t)((buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4));
    int32_t adcT = (int32_t)((buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4));
    if (adcP == 0x80000 || adcT == 0x80000) {
      return false;
    }

    int32_t var1 = ((((adcT >> 3) - ((int32_t)digT1 << 1))) * ((int32_t)digT2)) >> 11;
    int32_t var2 = (((((adcT >> 4) - ((int32_t)digT1)) * ((adcT >> 4) - ((int32_t)digT1))) >> 12) * ((int32_t)digT3)) >> 14;
    tFine = var1 + var2;

    int64_t pvar1 = ((int64_t)tFine) - 128000;
    int64_t pvar2 = pvar1 * pvar1 * (int64_t)digP6;
    pvar2 = pvar2 + ((pvar1 * (int64_t)digP5) << 17);
    pvar2 = pvar2 + (((int64_t)digP4) << 35);
    pvar1 = ((pvar1 * pvar1 * (int64_t)digP3) >> 8) + ((pvar1 * (int64_t)digP2) << 12);
    pvar1 = (((((int64_t)1) << 47) + pvar1) * ((int64_t)digP1)) >> 33;
    if (pvar1 == 0) {
      return false;
    }

    int64_t p = 1048576 - adcP;
    p = (((p << 31) - pvar2) * 3125) / pvar1;
    pvar1 = (((int64_t)digP9) * (p >> 13) * (p >> 13)) >> 25;
    pvar2 = (((int64_t)digP8) * p) >> 19;
    p = ((p + pvar1 + pvar2) >> 8) + (((int64_t)digP7) << 4);

    pressurePa = (float)p / 256.0f;
    return pressurePa > 0.0f;
  }

  bool readAltitude(float seaLevelHpa, float& altitudeM) {
    float pressurePa = 0.0f;
    if (!readPressurePa(pressurePa)) {
      return false;
    }
    altitudeM = AltitudeMath::pressureToAltitudeMeters(pressurePa, seaLevelHpa);
    return true;
  }
};

//==================== 定数・マクロ ====================
#define pi 3.141592653589793
#define SD_SPI_SPEED SD_SCK_MHZ(25)
#define SD_CONFIG SdSpiConfig(GPIO_NUM_4, SHARED_SPI, SD_SPI_SPEED)
  
// BLE用ピンと利用モード（1: ハードウェアUART使用; 0: SoftwareSerial使用）
#define BLE_RX_PIN 32
#define BLE_TX_PIN 33
#define USE_HARDWARE_BLE 0

// 各タスクの更新間隔（ミリ秒）
#define SERIAL_OUT_INTERVAL 1000
#define SD_LOG_INTERVAL     1000
#define MQTT_INTERVAL_PRE   10000  // 走行前
#define MQTT_INTERVAL_RUN   1000   // 走行中
#define AMBIENT_INTERVAL    10000

// MQTT設定
#define MQTT_BUFFER_SIZE  512 // MQTT送受信のバッファサイズ

// 高度推定設定
static constexpr float SEA_LEVEL_HPA = 1013.25f;
// 実走想定: 野外・水平移動・1-80km/h (路面振動/風圧の影響を見込んだ設定)
static constexpr float R_BARO = 0.36f;          // baro sigma ~0.6m
static constexpr float R_BARO_WITH_ABS = 4.0f;  // abs標高がある間はbaroを弱く使う
static constexpr float R_GSI = 4.0f;            // GSI sigma ~2.0m
static constexpr float R_GSI_HOLD = 9.0f;       // GSI更新間隔中の拘束用
static constexpr float R_GSI_HARD = 0.25f;      // GSIから大きく外れたときの再ロック用
static constexpr float R_GNSS = 36.0f;          // abs altitude sigma ~6.0m (GNSS fallback時)
static constexpr float EKF_PROCESS_SIGMA_A = 1.2f; // IMU vertical accel noise sigma [m/s^2]
static constexpr float ALT_ACCEL_DEADBAND = 0.35f; // 鉛直加速度デッドバンド [m/s^2]
static constexpr float GSI_HARD_GATE_M = 3.0f;     // GSI再ロック開始しきい値 [m]
static constexpr float LOW_SPEED_FREEZE_KMPH = 3.0f; // 低速時はIMU鉛直加速度を凍結
static constexpr uint32_t GNSS_BAUD = 38400;    // GNSSモジュールのシリアル通信速度
static constexpr unsigned long GSI_ELEVATION_INTERVAL = 10000;  // 国土地理院APIから標高を取得する間隔（ミリ秒）
static constexpr uint16_t GSI_HTTP_TIMEOUT_MS = 5000; // 国土地理院APIへのHTTPリクエストのタイムアウト時間（ミリ秒）

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
WiFiClientSecure client;
PubSubClient mqttclient(client);
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
BMP280Driver bmp;
AltitudeEKF altitudeEkf;
double la, ln;              // 緯度経度
// double la = 34.990768;    // KMMF2026の緯度経度初期値
// double ln = 137.010875;   // KMMF2026の緯度経度初期値
double alt = 0.0;           // GPS高度（GNSS）
double altGNSS = 0.0;       // GPS高度（GNSS）フィルタリング後
double altBaro = 0.0;       // 気圧高度（Baro）
double spd = 0.0;           // GPS速度
String Loc = "";            // 位置情報（緯度経度）文字列
bool gnssAltValid = false;  // GNSS高度が有効かどうか
bool gnssAltUpdated = false;  // GNSS高度が更新されたかどうか（EKFの更新に利用）
bool positionValid = false; // 位置情報が有効かどうか（GNSS高度の更新に利用）
double altGSI = 0.0;        // 国土地理院API(https://maps.gsi.go.jp/development/elevation_s.html)から取得した標高
bool gsiAltValid = false;   // 国土地理院APIから取得した標高が有効かどうか
bool gsiAltUpdated = false; // 国土地理院APIから取得した標高が更新されたかどうか（EKFの更新に利用）
unsigned long lastGsiRequestMs = 0; // 最後に国土地理院APIにリクエストを送った時刻（EKFの更新に利用）
bool bmpReady = false;      // BMP280が正常に初期化されているかどうか
bool baroValid = false;     // 気圧高度が有効かどうか（EKFの更新に利用）
bool baroCalibrated = false;  // 気圧高度がキャリブレーションされているかどうか（EKFの更新に利用）
float baroOffset = 0.0f;    // 気圧高度のオフセット値（キャリブレーションに利用）
unsigned long lastAltFusionMs = 0;  // 最後に高度融合を行った時刻（EKFの更新に利用）
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

// NTP同期フラグ（GPS受信後は更新しない）
bool ntpSyncDone = false;

// 日時バッファ更新（必要に応じてセンチ秒を指定）
void refreshDatetime(uint8_t csec = 255) {
  uint8_t displayCsec = csec;
  if (displayCsec > 99) {
    displayCsec = (millis() / 10) % 100;  // センチ秒が指定されていない場合は現在のミリ秒から算出して表示（00-99）
  }

  sprintf_P(datetime, PSTR("%d/%d/%d %02d:%02d:%02d.%02d"),
            year(), month(), day(), hour(), minute(), second(), displayCsec);
}

// ディスプレイ表示モード
uint8_t dispmode = 0;

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

// 国土地理院APIから標高を取得する（成功時 true）
bool parseGsiElevationPayload(const String& payload, double& outElevation) {
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload);
  if (!err) {
    JsonVariant elevation = doc["elevation"];
    if (elevation.is<float>() || elevation.is<double>() || elevation.is<int>() || elevation.is<long>()) {
      outElevation = elevation.as<double>();
      return true;
    }

    if (elevation.is<const char*>()) {
      const char* elevStr = elevation.as<const char*>();
      if (elevStr != nullptr && strcmp(elevStr, "-----") != 0) {
        char* endPtr = nullptr;
        double parsed = strtod(elevStr, &endPtr);
        if (endPtr != elevStr) {
          outElevation = parsed;
          return true;
        }
      }
    }
  }

  // 予期しないレスポンス形式に備え、最低限の文字列抽出も試す。
  int keyIndex = payload.indexOf("\"elevation\"");
  if (keyIndex < 0) {
    return false;
  }
  int colonIndex = payload.indexOf(':', keyIndex);
  if (colonIndex < 0) {
    return false;
  }

  int valueStart = colonIndex + 1;
  while (valueStart < (int)payload.length() && isspace((unsigned char)payload[valueStart])) {
    valueStart++;
  }
  if (valueStart >= (int)payload.length()) {
    return false;
  }

  bool quoted = payload[valueStart] == '"';
  if (quoted) {
    valueStart++;
  }

  int valueEnd = valueStart;
  while (valueEnd < (int)payload.length()) {
    char c = payload[valueEnd];
    if (quoted) {
      if (c == '"') {
        break;
      }
    } else if (c == ',' || c == '}' || isspace((unsigned char)c)) {
      break;
    }
    valueEnd++;
  }

  if (valueEnd <= valueStart) {
    return false;
  }

  String value = payload.substring(valueStart, valueEnd);
  value.trim();
  if (value == "-----") {
    return false;
  }

  char* endPtr = nullptr;
  double parsed = strtod(value.c_str(), &endPtr);
  if (endPtr == value.c_str()) {
    return false;
  }
  outElevation = parsed;
  return true;
}

bool fetchGsiElevation(double lat, double lon, double& outElevation) {
  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }

  WiFiClientSecure httpsClient;
  httpsClient.setInsecure();
  httpsClient.setTimeout(GSI_HTTP_TIMEOUT_MS);

  WiFiClient httpClient;

  char httpsUrl[192];
  char httpUrl[192];
  snprintf(httpsUrl, sizeof(httpsUrl),
           "https://cyberjapandata2.gsi.go.jp/general/dem/scripts/getelevation.php?lon=%.7f&lat=%.7f&outtype=JSON",
           lon, lat);
  snprintf(httpUrl, sizeof(httpUrl),
           "http://cyberjapandata2.gsi.go.jp/general/dem/scripts/getelevation.php?lon=%.7f&lat=%.7f&outtype=JSON",
           lon, lat);

  String payload;

  // HTTP優先。失敗時のみ HTTPS へフォールバック。
  {
    HTTPClient http;
    http.setConnectTimeout(GSI_HTTP_TIMEOUT_MS);
    http.setTimeout(GSI_HTTP_TIMEOUT_MS);
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    if (http.begin(httpClient, httpUrl)) {
      const int httpCode = http.GET();
      if (httpCode == HTTP_CODE_OK) {
        payload = http.getString();
        http.end();
        if (parseGsiElevationPayload(payload, outElevation)) {
          return true;
        }
        Serial.println("[GSI] HTTP parse failed");
      } else {
        Serial.printf("[GSI] HTTP GET failed: %d\n", httpCode);
        http.end();
      }
    } else {
      Serial.println("[GSI] HTTP begin failed");
    }
  }

  {
    HTTPClient https;
    https.setConnectTimeout(GSI_HTTP_TIMEOUT_MS);
    https.setTimeout(GSI_HTTP_TIMEOUT_MS);
    https.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    if (https.begin(httpsClient, httpsUrl)) {
      const int httpCode = https.GET();
      if (httpCode == HTTP_CODE_OK) {
        payload = https.getString();
        https.end();
        if (parseGsiElevationPayload(payload, outElevation)) {
          return true;
        }
        Serial.println("[GSI] HTTPS parse failed");
      } else {
        Serial.printf("[GSI] HTTPS GET failed: %d\n", httpCode);
        https.end();
      }
    } else {
      Serial.println("[GSI] HTTPS begin failed");
    }
  }

  return false;
}

// インターネット利用可能時はGSI標高を定期取得
void updateGsiElevation() {
  gsiAltUpdated = false;

  if (WiFi.status() != WL_CONNECTED || !positionValid) {
    gsiAltValid = false;
    return;
  }

  if (!isfinite(la) || !isfinite(ln) || la < -90.0 || la > 90.0 || ln < -180.0 || ln > 180.0) {
    gsiAltValid = false;
    return;
  }

  unsigned long now = millis();
  if (now - lastGsiRequestMs < GSI_ELEVATION_INTERVAL) {
    return;
  }
  lastGsiRequestMs = now;

  double gsiElevation = 0.0;
  if (fetchGsiElevation(la, ln, gsiElevation)) {
    if (gsiElevation > -500.0 && gsiElevation < 10000.0) {
      altGSI = gsiElevation;
      gsiAltValid = true;
      gsiAltUpdated = true;
      Serial.printf("[GSI] elevation=%.2f m (lat=%.7f, lon=%.7f)\n", gsiElevation, la, ln);
    } else {
      gsiAltValid = false;
      Serial.printf("[GSI] invalid elevation range: %.2f\n", gsiElevation);
    }
  } else {
    // API取得不可時はGNSSへフォールバックできるよう無効化
    gsiAltValid = false;
  }
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
  gnssAltUpdated = false;
  while (Serial2.available() > 0) {
    if (gps.encode(Serial2.read())) {
      if (gps.time.isUpdated()) {
        double rawAlt = gps.altitude.meters();
        if (rawAlt > -500 && rawAlt < 10000.0) { // 標高的に妥当な範囲内かを確認
          altGNSS = rawAlt;
          gnssAltValid = true;
          gnssAltUpdated = true;
        }
        if (gps.location.lng() > 120) {  // 異常値除外
          la = gps.location.lat();
          ln = gps.location.lng();
          positionValid = true;
          spd = gps.speed.kmph();
          uint8_t gnss_day = gps.date.day();
          uint8_t gnss_month = gps.date.month();
          uint8_t gnss_year = gps.date.year();
          uint8_t gnss_hour = gps.time.hour();
          uint8_t gnss_minute = gps.time.minute();
          uint8_t gnss_second = gps.time.second();
          uint8_t gnss_csec = gps.time.centisecond();
          // JST変換
          gnss_hour = gnss_hour + time_offset;
          if (gnss_hour > 23) {  // 時間が日付を超える場合
            gnss_hour -= 24;
            gnss_day++;
            if (gnss_month == 2){  // ２月の場合
              if ( (gnss_year % 4) == 0 ) {
                if(gnss_day > 28) {
                  gnss_day = 1;
                  gnss_month++;
                }
              } else {
                if(gnss_day > 29) {
                  gnss_day = 1;
                  gnss_month++;
                }              
              }
            }else if ((gnss_month % 2) == 0){ // ２月以外の偶数月の場合
              if ( gnss_day > 30 ){
                gnss_day = 1;
                gnss_month++;
                if ( gnss_month > 12 ){
                  gnss_year++;
                }
              }      
            }else{  //　奇数月の場合
              if ( gnss_day > 31 ){
                gnss_day = 1;
                gnss_month++;          
              }
            }
          }
          setTime(gnss_hour, gnss_minute, gnss_second, gnss_day, gnss_month, gnss_year);
          ntpSyncDone = true;  // GPS時刻受信後はNTP同期不要（GPS優先）
          refreshDatetime(gnss_csec);  // デバッグ用Serial出力
        }
        break;
      }
    }
  }
  // ロケーション判定
  if (positionValid && la >= 34.837989 && la <= 34.84828 && ln >= 136.522015 && ln <= 136.544450) {
    Loc = "su";
    totallaps = totallaps_su;
    goal = goal_su;
    limittime = limittime_su;
  } else if (positionValid && la >= 36.528477 && la <= 36.538522 && ln >= 140.2192761 && ln <= 140.23853) {
    Loc = "mo";
    totallaps = totallaps_mo;
    goal = goal_mo;
    limittime = limittime_mo;
  } else {
    Loc = "to";
  }
}

// GNSS + BMP280 + IMU で高度を融合
void updateAltitudeFusion() {
  unsigned long now = millis();
  if (lastAltFusionMs == 0) {
    lastAltFusionMs = now;
    return;
  }

  float dt = (now - lastAltFusionMs) * 0.001f;
  if (dt <= 0.0f) {
    return;
  }
  if (dt > 0.2f) {
    dt = 0.2f;
  }
  lastAltFusionMs = now;

  float ax = 0.0f;
  float ay = 0.0f;
  float az = 0.0f;
  M5.Imu.getAccel(&ax, &ay, &az);
  // Y軸加速度を用いて重力成分を除去
  float ayInertial = -(ay + 1.0f) * 9.80665f;
  if (fabsf(ayInertial) < ALT_ACCEL_DEADBAND || spd < LOW_SPEED_FREEZE_KMPH) {
    ayInertial = 0.0f;
  }

  bool baroUpdated = false;
  float rawBaroAlt = 0.0f;
  const bool absFromGsi = gsiAltValid;
  const bool absAltValid = gsiAltValid || gnssAltValid;
  const bool absAltUpdated = absFromGsi ? gsiAltUpdated : gnssAltUpdated;
  const float absAltVariance = absFromGsi ? R_GSI : R_GNSS;
  const double absAlt = gsiAltValid ? altGSI : altGNSS;

  if (bmpReady && bmp.readAltitude(SEA_LEVEL_HPA, rawBaroAlt)) {
    if (absAltValid) {
      const float targetOffset = (float)absAlt - rawBaroAlt;
      if (!baroCalibrated) {
        baroOffset = targetOffset;
        baroCalibrated = true;
      } else {
        float alpha = absAltUpdated ? 0.05f : 0.01f;
        if (fabsf(targetOffset - baroOffset) > 20.0f) {
          alpha = 0.2f;
        }
        baroOffset += alpha * (targetOffset - baroOffset);
      }
    }
    altBaro = rawBaroAlt + baroOffset;
    baroValid = true;
    baroUpdated = true;
  } else {
    baroValid = false;
  }

  if (!altitudeEkf.isInitialized()) {
    if (absAltValid) {
      altitudeEkf.init((float)absAlt);
      alt = absAlt;
    } else if (baroUpdated) {
      altitudeEkf.init((float)altBaro);
      alt = altBaro;
    }
    return;
  }

  altitudeEkf.predict(ayInertial, dt);
  if (baroUpdated) {
    altitudeEkf.update((float)altBaro, absAltValid ? R_BARO_WITH_ABS : R_BARO);
  }
  if (absAltValid) {
    altitudeEkf.update((float)absAlt, absAltUpdated ? absAltVariance : (absFromGsi ? R_GSI_HOLD : R_GNSS));
    if (absFromGsi && fabsf(altitudeEkf.altitude() - (float)absAlt) > GSI_HARD_GATE_M) {
      altitudeEkf.update((float)absAlt, R_GSI_HARD);
    }
  }
  alt = altitudeEkf.altitude();
}

// ディスプレイ更新（表示モードごとに分岐）
void updateDisplay() {
  lcd_s.fillScreen(TFT_BLACK);
  lcd_s.setTextColor(TFT_WHITE);
  
  // ボタン状態により表示モードを切り替え
  if (M5.BtnB.isPressed()) { dispmode = 0; }
  if (M5.BtnA.isPressed()) { dispmode = 1; }
  if (M5.BtnC.isPressed()) { dispmode = 2; }
  
  // 接続状況表示
  lcd_s.setFont(&fonts::lgfxJapanGothicP_16);
  lcd_s.setTextSize(1);
  lcd_s.setTextDatum(top_right);
  lcd_s.drawString(LOGGING ? "SD: O" : "SD: x", 320, 0);
  lcd_s.drawString(positionValid ? "GNSS: O" : "GNSS: x", 320, 15);
  // if (ambientpush) lcd_s.drawString("Amb: O", 320, 30);
  if (MQTTpush) lcd_s.drawString("MQTT: O", 320, 30);
  if (!ambientpush && !MQTTpush) {
    // lcd_s.drawString("Amb: x", 320, 30);
    lcd_s.drawString("MQTT: x", 320, 30);
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
  } else if (dispmode == 2) {
    lcd_s.drawString("速度(km/h):", 10, 50);
    lcd_s.drawString("回転数(rpm):", 10, 130);
    lcd_s.drawString("燃費(km/l):", 10, 210);
  }
  
  // 第１表示行
  lcd_s.setFont(&fonts::Font7);
  lcd_s.setTextSize(1);
  lcd_s.setTextDatum(BL_DATUM);
  lcd_s.setCursor(140, 50);
  if (dispmode == 0 || dispmode == 2) {
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
    if (restlaps > 1) {
      lcd_s.print(restlaps);
    } else if (restlaps == 1) {
      lcd_s.print("G");
    } else {
      lcd_s.print("FINISH");
    }
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
  } else if (dispmode == 2) {
    lcd_s.setCursor(140, 130);
    lcd_s.print(tachoRpm);
    lcd_s.drawRect(9, 134, 302, 12, TFT_WHITE);
    lcd_s.fillRect(10, 135, map(tachoRpm, 0, 6500, 0, 300), 10, TFT_WHITE);
    lcd_s.fillRect(map(tachoRpm, 0, 6500, 10, 310), 135, map(tachoRpm, 0, 6500, 300, 0), 10, TFT_BLACK);
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
    if (map(distance, 0, goal, 300, 0) <= map(worktime, 0, limittime, 300, 0)) {
      lcd_s.fillRect(10, 215, map(worktime, 0, limittime, 300, 0), 10, TFT_WHITE);
    } else {
      lcd_s.fillRect(10, 215, map(worktime, 0, limittime, 300, 0), 10, TFT_MAGENTA);
      lcd_s.fillRect(map(worktime, 0, limittime, 310, 10), 215, map(worktime, 0, limittime, 0, 300), 10, TFT_BLACK);
    }
  } else if (dispmode == 1) {
    lcd_s.print(IGN_CA);
    lcd_s.drawRect(9, 214, 302, 12, TFT_WHITE);
    lcd_s.fillRect(10, 215, map(IGN_CA, 0, 90, 0, 300), 10, TFT_WHITE);
    lcd_s.fillRect(map(IGN_CA, 0, 90, 10, 310), 215, map(IGN_CA, 0, 90, 300, 0), 10, TFT_BLACK);
  } else if (dispmode == 2) {
    lcd_s.print(dispergas, 1);
    lcd_s.drawRect(9, 214, 302, 12, TFT_WHITE);
    lcd_s.fillRect(10, 215, map(dispergas, 0, 2000, 0, 300), 10, TFT_WHITE);
    lcd_s.fillRect(map(dispergas, 0, 2000, 10, 310), 215, map(dispergas, 0, 2000, 300, 0), 10, TFT_BLACK);
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
  Serial.println(EngTemp, 2);
}

// SDカードへのログ書き出し
void updateSDLog() {
  refreshDatetime();  // MQTT送信前に日時を更新

  bool isNewFile = false;
  if (!logFileInitialized) {
    isNewFile = !sd.exists(fileName);
  }

  logFile = sd.open(fileName, O_WRITE | O_CREAT | O_APPEND);
  if (logFile) {
    if (!logFileInitialized) {
      if (isNewFile) {
        logFile.timestamp(T_CREATE, 2024, 1, 31, 23, 59, 59);
        logFile.write(0xEF); logFile.write(0xBB); logFile.write(0xBF);
        logFile.println(F("記録日時,速度(km/h),ラップ数,走行時間,回転数,走行距離,積算燃料,燃費,lat,lon,alt,loc,温度"));
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
    logFile.println(EngTemp, 2);
    logFile.close();
  } else {
    Serial.println("SD Log open failed!");
  }
}

// MQTT送信（非同期リトライ）
void updateMQTT() {
  refreshDatetime();  // MQTT送信前に日時を更新

  if (!mqttclient.connected()) {
    static unsigned long lastReconnectAttempt = 0;
    if (millis() - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = millis();
      if (mqttclient.connect(mqtt_deviceID)) {
        Serial.println("MQTT connected");
      } else {
        Serial.print("MQTT reconnect failed, state: ");
        Serial.println(mqttclient.state());
      }
    }
    return;
  }
  mqttclient.loop();
  JsonDocument doc;
  doc["timestamp"] = datetime;
  //doc["Spd_GPS"]   = spd;
  doc["Spd_PULSE"] = speed;
  doc["Lapcount"]  = Lapcount;
  doc["worktime"]  = worktime;
  doc["tachoRpm"]  = tachoRpm;
  doc["distance"]  = distance;
  doc["gasml"]     = gasml;
  doc["dispergas"] = dispergas;
  doc["lat"]       = la;
  doc["lon"]       = ln;
  doc["alt"]       = alt;
  doc["loc"]       = Loc;
  doc["temp"]      = EngTemp;
  String jsonData;
  serializeJson(doc, jsonData);
  mqttclient.publish(mqtt_topic, jsonData.c_str());
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

//==================== setup() =====================
void setup() {
  // M5初期化
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.update();
  
  // デバッグ用Serial
  Serial.begin(115200);
  // ECU, GNSS初期化
  Serial1.begin(115200, SERIAL_8N1, 27, 19);
  Serial2.begin(GNSS_BAUD);
  M5.Imu.init();
  altitudeEkf.setProcessAccelSigma(EKF_PROCESS_SIGMA_A);
  altitudeEkf.setInitialCovariance(100.0f, 10.0f);
  bmpReady = bmp.begin();
  baroValid = false;
  Serial.println(bmpReady ? "BMP280: OK" : "BMP280: NG");
  // BLE初期化
  #if USE_HARDWARE_BLE
    SerialBLE.begin(115200, SERIAL_8N1, BLE_RX_PIN, BLE_TX_PIN);
  #else
    // バッファサイズを512バイトに拡大してデータロス対策
    SerialBLE.begin(115200, SWSERIAL_8N1, BLE_RX_PIN, BLE_TX_PIN, false, 512);
  #endif
  
  // LCD初期化
  lcd.init();
  lcd.setRotation(1);
  lcd.setBrightness(128);
  lcd.fillScreen(TFT_BLACK);
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
  
  // Ambient設定
  if (ambientpush && isWifiConfigSucceeded) {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    sprintf(devKey, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
    if (!ambient.getchannel(userKey, devKey, channelId, writeKey, sizeof(writeKey), &client)) {
      Serial.printf("Cannot get channelId for device %s\n", devKey);
      while (!ambient.getchannel(userKey, devKey, channelId, writeKey, sizeof(writeKey), &client)) {
        M5.update();
        delay(500);
      }
    }
    ambient.begin(channelId, writeKey, &client);
  }
  
  // MQTT設定
  if (MQTTpush && isWifiConfigSucceeded) {
    mqttclient.setBufferSize(MQTT_BUFFER_SIZE);
    client.setCACert(AWS_CERT_CA);
    client.setCertificate(AWS_CERT_CRT);
    client.setPrivateKey(AWS_CERT_PRIVATE);
    mqttclient.setServer(mqtt_server, mqtt_port);
    updateMQTT();
  }
  
  // NTP同期（Wi-Fi接続成功時、GPS時刻受信前）
  if (isWifiConfigSucceeded && !ntpSyncDone) {
    configTime(9 * 3600, 0, "pool.ntp.org");  // JST (UTC+9)
    Serial.println("NTP sync started");
    // NTP同期待機（最大10秒）
    unsigned long ntpStart = millis();
    while (millis() - ntpStart < 10000) {
      time_t now = time(nullptr);
      struct tm* timeinfo = localtime(&now);
      if (timeinfo->tm_year > 70) {  // 1970年以降になったかチェック
        ntpSyncDone = true;
        // TimeLibの時刻を更新（datetime バッファ用）
        setTime(timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec,
                timeinfo->tm_mday, timeinfo->tm_mon + 1, timeinfo->tm_year + 1900);
        // datetime バッファを更新
        refreshDatetime(0);
        Serial.print("NTP sync succeeded: ");
        Serial.println(asctime(timeinfo));
        break;
      }
      M5.update();
      delay(100);
    }
    if (!ntpSyncDone) {
      Serial.println("NTP sync timeout");
    }
  }
  
  lcd.fillScreen(TFT_BLACK);
  showMessage(FPSTR(MSG_LOADING));
  Serial.println(F("lat, lon, alt, loc, Spd_GPS, rpm, Spd_PULSE, distance, gasml, dispergas, worktime, Temp"));
  
  t_Serial = millis();
  t_SD = millis();
  t_MQTT = millis();
  t_amb = millis();
}

//==================== loop() =====================
void loop() {
  M5.update();
  
  updateBLE();
  updateECU();
  updateGNSS();
  updateGsiElevation();
  updateAltitudeFusion();
  updateDisplay();
  
  if (millis() - t_Serial >= SERIAL_OUT_INTERVAL) {
    updateSerialOutput();
    t_Serial = millis();
  }
  
  if (LOGGING && (millis() - t_SD >= SD_LOG_INTERVAL)) {
    updateSDLog();
    t_SD = millis();
  }
  
  if (MQTTpush) {
    if (worktime == 0) {
      if (millis() - t_MQTT >= MQTT_INTERVAL_PRE) {
        updateMQTT();
        t_MQTT = millis();
      }
    } else {
      if (millis() - t_MQTT >= MQTT_INTERVAL_RUN) {
        updateMQTT();
        t_MQTT = millis();
      }
    }
  }
  
  if (ambientpush && (millis() - t_amb >= AMBIENT_INTERVAL)) {
    updateAmbient();
    t_amb = millis();
  }
  
  delay(10);
}
