
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
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "secrets.h"

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
char fileName[20];
int fileNum = 0;

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
uint16_t tachoRpm = 0;
float INJ_timems = 0.0;
uint8_t IGN_CA = 0;
uint8_t speed = 0;
uint16_t distance = 0;
float gasml = 0.0;
float dispergas = 0.0;
uint16_t worktime = 0;
uint16_t Lapcount = 0;
uint8_t totallaps = 3;
uint16_t goal = 1000;
uint16_t limittime = 100;
float EngTemp = 0.0;

// GPS用
TinyGPSPlus gps;
double la, ln;
// double la = 34.990768;    // KMMF2026の緯度経度初期値
// double ln = 137.010875;   // KMMF2026の緯度経度初期値
double spd = 0.0;
String Loc = "";
// サーキットごとの設定
const uint8_t totallaps_su = 8;
const uint8_t totallaps_mo = 7;
const uint16_t goal_su = 17616;
const uint16_t goal_mo = 16389;
const uint16_t limittime_su = 2536;
const uint16_t limittime_mo = 2360;
const int time_offset = 9;  // JST

// 時刻表示用バッファ
char datetime[23];

// NTP同期フラグ（GPS受信後は更新しない）
bool ntpSyncDone = false;

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

// BLEからのデータ読み取り（バッファ＋タイムアウト処理）
void updateBLE() {
  static String bleBuffer = "";
  static unsigned long lastBLETime = 0;
  
  while (SerialBLE.available() > 0) {
    char c = SerialBLE.read();
    bleBuffer += c;
    lastBLETime = millis();
    if (c == '\n') {
      bleBuffer.trim();
      if (bleBuffer.length() > 0) {
        // データ完整性チェック: 数値として有効かつ妥当な範囲内かを確認
        float tempValue = bleBuffer.toFloat();
        // 温度として妥当な範囲（20～150℃）かつ、toFloat()が有効な変換を行ったかチェック
        if ((tempValue != 0.0 || bleBuffer == "0" || bleBuffer == "0.0") && 
            tempValue >= 20.0 && tempValue <= 150.0) {
          EngTemp = tempValue;
        }
        // 不正なデータの場合は前回値を保持（更新しない）
      }
      bleBuffer = "";
    }
  }
  /*
  if (millis() - lastBLETime > 100 && bleBuffer.length() > 0) {   // タイムアウト処理(100ミリ秒以上経過)
    bleBuffer.trim();
    if (bleBuffer.length() > 0) {
      EngTemp = bleBuffer.toFloat();
    }
    bleBuffer = "";
  }
  */
  if (millis() - lastBLETime > 10000) {   // タイムアウト処理(10秒以上経過)
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
      if (i == 3) { speed = data.toInt(); }
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
      speed = 0;
    }
  }
}

// GNSSからの位置・時刻読み取り
void updateGNSS() {
  while (Serial2.available() > 0) {
    if (gps.encode(Serial2.read())) {
      if (gps.time.isUpdated()) {
        if (gps.location.lng() > 120) {  // 異常値除外
          la = gps.location.lat();
          ln = gps.location.lng();
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
          sprintf_P(datetime, PSTR("%d/%d/%d %02d:%02d:%02d.%02d"),
                    year(), month(), day(), hour(), minute(), second(), gnss_csec);
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
  if (ambientpush) lcd_s.drawString("Amb: O", 320, 15);
  if (MQTTpush) lcd_s.drawString("MQTT: O", 320, 15);
  if (!ambientpush && !MQTTpush) lcd_s.drawString("Amb･MQTT: x", 320, 15);
  
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
    lcd_s.print(speed);
    lcd_s.drawRect(9, 54, 302, 12, TFT_WHITE);
    int barLength = map(speed, 0, 45, 0, 300);
    lcd_s.fillRect(10, 55, barLength, 10, TFT_WHITE);
    lcd_s.fillRect(map(speed, 0, 45, 10, 310), 55, map(speed, 0, 45, 300, 0), 10, TFT_BLACK);
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
  Serial.print(Loc);   Serial.print(",");
  Serial.print(spd, 1); Serial.print(",");
  Serial.print(tachoRpm); Serial.print(",");
  Serial.print(speed);    Serial.print(",");
  Serial.print(distance); Serial.print(",");
  Serial.print(gasml, 1); Serial.print(",");
  Serial.print(dispergas, 1); Serial.print(",");
  Serial.print(worktime); Serial.print(",");
  Serial.println(EngTemp, 2);
}

// SDカードへのログ書き出し
void updateSDLog() {
  logFile = sd.open(fileName, O_WRITE | O_CREAT | O_APPEND);
  if (logFile) {
    logFile.timestamp(T_WRITE, year(), month(), day(), hour(), minute(), second());
    logFile.print(datetime); logFile.print(",");
    logFile.print(speed);    logFile.print(",");
    logFile.print(Lapcount); logFile.print(",");
    logFile.print(worktime); logFile.print(",");
    logFile.print(tachoRpm); logFile.print(",");
    logFile.print(distance, 1); logFile.print(",");
    logFile.print(gasml, 1);   logFile.print(",");
    logFile.print(dispergas, 1); logFile.print(",");
    logFile.print(la, 7);      logFile.print(",");
    logFile.print(ln, 7);      logFile.print(",");
    logFile.println(EngTemp, 2);
    logFile.close();
  } else {
    Serial.println("SD Log open failed!");
  }
}

// MQTT送信（非同期リトライ）
void updateMQTT() {
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
  StaticJsonDocument<512> doc;
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
    ambient.set(1, spd);
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
  Serial2.begin(115200);  
  // BLE初期化
  #if USE_HARDWARE_BLE
    SerialBLE.begin(115200, SERIAL_8N1, BLE_RX_PIN, BLE_TX_PIN);
  #else
    SerialBLE.begin(115200);
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
  unsigned long sdStart = millis();
  bool sdInitialized = false;
  while (!sdInitialized && (millis() - sdStart < 3000)) {
    if (sd.begin(SD_CONFIG)) { sdInitialized = true; break; }
    M5.update();

    Serial.println(F("SD Wait..."));
    lcd.fillScreen(TFT_RED);
    lcd.setTextColor(TFT_BLACK);
    showMessage(FPSTR(MSG_NO_SD));
    lcd.drawNumber(int((3000 - (millis() - sdStart)) / 1000), lcd.width()/2, lcd.height()/2+20);

    delay(1000);
  }
  if (!sdInitialized) {
    LOGGING = false;
    Serial.println("SD init failed");  
  } else {
    if (!sd.exists("/LOG")) {
      sd.mkdir("/LOG");
      showMessage(FPSTR(MSG_LOG_DIR_CREATE));
    }
    while (true) {
      snprintf(fileName, sizeof(fileName), "/LOG/LOG%04d.CSV", fileNum);
      showMessage(FPSTR(MSG_LOG_FILE_CREATE));
      if (!sd.exists(fileName)) {
        logFile = sd.open(fileName, O_WRITE | O_CREAT | O_APPEND);
        if (logFile) {
          logFile.timestamp(T_CREATE, 2024, 1, 31, 23, 59, 59);
          logFile.write(0xEF); logFile.write(0xBB); logFile.write(0xBF);
          logFile.println(F("記録日時,速度(km/h),ラップ数,走行時間,回転数,走行距離,積算燃料,燃費,lat,lon,温度"));
          logFile.close();
        }
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
        sprintf_P(datetime, PSTR("%d/%d/%d %02d:%02d:%02d.00"),
                  year(), month(), day(), hour(), minute(), second());
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
  Serial.println(F("lat, lon, loc, Spd_GPS, rpm, Spd_PULSE, distance, gasml, dispergas, worktime, Temp"));
  
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
