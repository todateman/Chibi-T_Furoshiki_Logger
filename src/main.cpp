// https://qiita.com/TanakaSU/items/2f414a7a46943e085471
// 計算式 https://www.trail-note.net/tech/coordinate/

#include <Arduino.h>
#include <TinyGPS++.h>
#include "SD.h"
#include <M5UnitLCD.h>
#include <M5Unified.h>
#include "Ambient.h"
#include <WiFiManager.h>
#include <TimeLib.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#define pi 3.141592653589793

static M5GFX lcd;
static LGFX_Sprite lcd_s(&lcd);

WiFiClient client;
Ambient ambient;
WiFiManager wifiManager;
PubSubClient mqttclient(client);

// 変数の定義
// 画面表示モード
enum dispmode {m_drive, m_engine, m_fuel};

// ECU
unsigned long receiveECUtime = 0; // ECUからデータを受信した時間
uint16_t tachoRpm = 0;            // エンジンの回転数(rpm)
float INJ_timems = 0.0;           // インジェクタ噴射時間(msec)
uint8_t IGN_CA = 0;               // 進角角度(CA)
uint8_t speed = 0;                // 速度(km/h)
uint16_t distance = 0;            // 走行距離積算(m)
float gasml = 0.0;                // 積算燃料消費量(ml)
float dispergas = 0.0;            // 燃費(km/l)
uint16_t Lapcount = 0;            // 現在の周回数
uint8_t totallaps = 3;            // トータル周回数
uint16_t goal = 1000;             // 総走行距離(m)
uint16_t limittime = 100;         // 規定時間(sec)
const uint8_t totallaps_su = 8;   // 鈴鹿のトータル周回数
const uint8_t totallaps_mo = 7;   // 茂木のトータル周回数
const uint16_t goal_su = 17616;   // 鈴鹿の総走行距離(m)
const uint16_t goal_mo = 16389;   // 鈴鹿の総走行距離(m)
const uint16_t limittime_su = 2536; // 鈴鹿の規定時間(42分16秒 = 2536sec)
const uint16_t limittime_mo = 2360; // 茂木の規定時間(39分20秒 = 2360sec)
uint16_t worktime = 0;            // 走行時間(sec)

// GPS
double goal_la_su = 34.842925;    // 鈴鹿サーキットスタートライン
double goal_ln_su = 136.540692;   // 鈴鹿サーキットスタートライン
//double goal_la_mo = 36.532770;  // ツインリンクもてぎオーバルコーススタートライン
//double goal_ln_mo = 140.226208; // ツインリンクもてぎオーバルコーススタートライン
double goal_la_mo = 36.533590;    // ツインリンクもてぎオーバルコーススタートライン
double goal_ln_mo = 140.225706;   // ツインリンクもてぎオーバルコーススタートライン
double goal_la_to = 35.082078;    // 豊田市SENTAN
double goal_ln_to = 137.160358;   // 豊田市SENTAN
//double goal_la_to = 35.066781;
//double goal_ln_to = 137.111857;
double la = goal_la_to;           // 初期設定で豊田市SENTANを設定
double ln = goal_ln_to;           // 初期設定で豊田市SENTANを設定
float spd=0.0;                    // 速度(km/h)
String Loc = "";
unsigned long t_Serial;           // Serialの送信時刻

// Wi-Fi
// const char* ssid = "****";  // Wi-Fi SSID
// const char* password = "****";  // Wi-Fi Password
bool isWifiConfigSucceeded = false;  // WiFi設定が成功したかどうかのフラグ

// ambient
bool ambientpush = false;    // ambientへの送信 有効(true)/無効(false)
//unsigned int channelId = 65530; // AmbientのチャネルID
//const char* writeKey = "050985c9530d8eb0"; // ライトキー
const char* userKey = "64bd5933d381952b59"; // ユーザーキー
char devKey[20];
unsigned int channelId;
char writeKey[20];

unsigned long t_amb;        // Ambientへの送信時刻

// MQTT
bool MQTTpush = true;      // MQTT送信 有効(true)/無効(false)
const char* mqtt_server = "furoshiki.asuscomm.com";
const int mqtt_port = 1883;
const char* mqtt_topic = "Furoshiki/M5Logger";
const char* mqtt_deviceID = "M5Core2";
const char* mqtt_user = "todateman";
const char* mqtt_password = "tomo8905";
unsigned long t_MQTT;       // MQTT送信時刻

// ログファイル
bool LOGGING = true;        // ロギング有効/無効
File logFile;
char fileName[20];          // ファイル名
int fileNum = 0;            // ファイル連番
unsigned long t_SD;         // SDの記録時刻
#define time_offset 32400   // UTC+9時間(60*60*9 秒）

TinyGPSPlus gps;


// LCD画面にメッセージを表示する
void showMessage(String msg)
{
  //lcd.setRotation(1);
  lcd.fillScreen(BLACK);
  lcd.setTextColor(WHITE);
  lcd.setCursor(0, 20);
  lcd.println(msg);
  lcd.setCursor(50, 230);
  lcd.print("A");
  lcd.setCursor(260, 230);
  lcd.print("C");
}

// WiFi接続モードに移行した時に呼ばれるコールバック
void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  Serial.println(myWiFiManager->getConfigPortalSSID());
  showMessage("このアクセスポイントに接続して\nWi-Fiの設定をしてください\nSSID: " + myWiFiManager->getConfigPortalSSID());
  char ConfigSSID[40];
  sprintf(ConfigSSID, "WIFI:S:%s;T:nopass;R:1;;", myWiFiManager->getConfigPortalSSID());
  lcd.qrcode(ConfigSSID, 105, 92, 135, 5);
}

// 起動後すぐにAボタンが押されたらWiFi設定モードに移行し、そうでなければ自動接続を行う
void setupWiFi ()
{
  wifiManager.setAPCallback(configModeCallback);

  // clicking power button at boot time to enter wifi config mode
  bool doManualConfig = false;
  showMessage("Aボタンを押してWi-Fi設定\nCボタンを押してWi-Fi無効化");
  for(int i=0 ; i<500 ; i++) {
    M5.update();
    if (M5.BtnA.isPressed()) {
      doManualConfig = true;
      break;
    }
    else if (M5.BtnC.isPressed()) {
      ambientpush = false;
      break;
    }
    delay(10);
  }

  if (doManualConfig) {
    Serial.println("wifiManager.startConfigPortal()");
    if (wifiManager.startConfigPortal()) {
      isWifiConfigSucceeded = true;
      Serial.println("startConfigPortal() connect success!");
    }
    else {
      Serial.println("startConfigPortal() connect failed!");
    }
  }
  else {
    if (ambientpush || MQTTpush){
      showMessage("Wi-Fi接続中...");

      Serial.println("wifiManager.autoConnect()");
      if (wifiManager.autoConnect()) {
        isWifiConfigSucceeded = true;
        Serial.println("autoConnect() connect success!");
        showMessage("Wi-Fi接続成功.");
      }
      else {
        Serial.println("autoConnect() connect failed!");
        showMessage("Wi-Fi接続失敗.");
      }
    }
    else {
      showMessage("Wi-Fi接続無効.");
    }
  }
}

// ECUからのデータを読み込み
void readSerialECU() {
  if (Serial1.available()){
    receiveECUtime = millis();                    // ECUからデータを受信した時間に現在時刻を代入
    String str = Serial1.readStringUntil('\n');   // Serial1から改行コード"CRLF"まで読み込む
    str.trim();                                   // Serial1から読み込んだデータの両端の空白、改行、タブなどを取り除く
    //Serial.println(str);
    for (uint8_t i = 0; i < 8; i++) {
      uint8_t check = (str.indexOf(","));         // ","の位置を探索する
      String data = str.substring(0, check);      // ","の位置で文字列を区切る
      data.trim();                                // 文字列の両端の空白、改行、タブなどを取り除く
      str = str.substring(check + 1);             // 読み込まなかった次の文字列を準備する
      if (i == 0) {tachoRpm   = data.toInt();}    // エンジンの回転数(rpm)
      if (i == 1) {INJ_timems = data.toFloat();}  // インジェクタ噴射時間(msec)
      if (i == 2) {IGN_CA     = data.toInt();}    // 進角角度(CA)
      if (i == 3) {speed      = data.toInt();}    // 速度(km/h)
      if (i == 4) {distance   = data.toInt();}    // 走行距離積算(m)
      if (i == 5) {gasml      = data.toFloat();}  // 積算燃料消費量(ml)
      if (i == 6) {dispergas  = data.toFloat();}  // 燃費(km/l)
      if (i == 7) {worktime   = data.toInt();}    // 走行時間(sec)
    }
    Lapcount = distance / (goal / totallaps);     // 現在の周回数
  }
  else {
    if ( millis() - receiveECUtime > 2000) {      // 2000msec間データの受信がなければ
      tachoRpm   = 0;                             // エンジンの回転数(rpm)
      INJ_timems = 0;                             // インジェクタ噴射時間(msec)
      IGN_CA     = 0;                             // 進角角度(CA)
      speed      = 0;                             // 速度(km/h)
    }
  }
}

// 位置情報を取得
void getGNSS() {
  while (Serial2.available() > 0) {
    if (gps.encode(Serial2.read())) {
      if (gps.location.isUpdated()) {
        // 経度緯度速度を取得
        if (gps.location.lng() > 120) {  // 東経120度の場合(異常値を除外するため)
          la         = gps.location.lat();
          ln         = gps.location.lng();
          spd        = gps.speed.kmph();
          uint8_t jst_day    = gps.date.day();
          uint8_t jst_month  = gps.date.month();
          uint8_t jst_year   = gps.date.year();
          uint8_t jst_hour   = gps.time.hour();
          uint8_t jst_minute = gps.time.minute();
          uint8_t jst_second = gps.time.second();
          setTime(jst_hour, jst_minute, jst_second, jst_day, jst_month, jst_year);
          // JST変換
          adjustTime(time_offset);
        }
        break;
      }
    }    
  }

  if ( la >= 34.839027 && la <= 34.84828 && ln >= 136.522015 && ln <= 136.543319 ) {
    Loc = "su";
    totallaps = totallaps_su;
    goal = goal_su;
    limittime = limittime_su;
  } else if ( la >= 36.528477 && la <= 36.53735 && ln >= 140.224726 && ln <= 140.23853 ) {
    Loc = "mo";
    totallaps = totallaps_mo;
    goal = goal_mo;
    limittime = limittime_mo;
  } else {
    Loc = "to";
  }
}

// ディスプレイに表示
void drawinfo(enum dispmode disp) {
  lcd_s.fillScreen(TFT_BLACK);

  // 記録ステータス表示
  lcd_s.setFont(&fonts::lgfxJapanGothicP_16);       // フォントを指定
  lcd_s.setTextSize(1.0);                           // フォントの拡大率
  lcd_s.setTextDatum(top_right);                    // データム(上・右)
  if (LOGGING) {                                    // SD
    lcd_s.drawString("SD: O", 320,  0);
  }
  else {
    lcd_s.drawString("SD: x", 320,  0);
  }
  if (ambientpush) {
    lcd_s.drawString("Amb: O", 320,  15);
  }
  if (MQTTpush) {
    lcd_s.drawString("MQTT: O", 320,  15);
  }
  if(!ambientpush && !MQTTpush) {
    lcd_s.drawString("Amb･MQTT: x", 320,  15);
  }

  // タイトル表示
  lcd_s.setFont(&fonts::lgfxJapanGothicP_20);       // フォントを指定
  lcd_s.setTextSize(1);                             // フォントの拡大率
  lcd_s.setTextDatum(BL_DATUM);                     // データム(下・左)
  //lcd_s.setTextColor(TFT_WHITE);                    // 文字色を指定
  if (disp == m_drive ) {                           // ディスプレイ表示モードが運転画面の場合
    lcd_s.drawString("速度(km/h):", 10,  50);         // タイトルを表示
    lcd_s.drawString("残り周回数:", 10, 130);         // タイトルを表示
    lcd_s.drawString("走行時間:", 10,  210);          // タイトルを表示
  } else if (disp == m_engine ) {                   // ディスプレイ表示モードがエンジン画面の場合
    lcd_s.drawString("回転数(rpm):", 10,  50);        // タイトルを表示
    lcd_s.drawString("噴射時間(ms):", 10, 130);       // タイトルを表示
    lcd_s.drawString("進角角度(CA):", 10,  210);      // タイトルを表示
  } else if (disp == m_fuel ) {                     // ディスプレイ表示モードが2の場合
    lcd_s.drawString("速度(km/h):", 10,  50);         // タイトルを表示
    lcd_s.drawString("回転数(rpm):", 10, 130);        // タイトルを表示
    lcd_s.drawString("燃費(km/l):", 10,  210);        // タイトルを表示
  }

  // 1番目の表示
  lcd_s.setFont(&fonts::Font7);                     // フォントを指定
  lcd_s.setTextSize(1);                             // フォントの拡大率
  lcd_s.setTextDatum(BL_DATUM);                     // データム(下・左)
  lcd_s.setCursor(140, 50);                         // 表示位置を指定
  if (disp == m_drive || disp == m_fuel ) {         // ディスプレイ表示モードが運転画面,燃費画面の場合
    lcd_s.print(speed);                               // 速度(km/h)
    lcd_s.drawRect(9, 54, 302, 12, TFT_WHITE);        // グラフの外枠を表示
    lcd_s.fillRect(10, 55, map(speed, 0, 45, 0, 300), 10, TFT_WHITE);
    lcd_s.fillRect(map(speed, 0, 45, 10, 310), 55, map(speed, 0, 45, 300, 0), 10, TFT_BLACK);
  } else if (disp == m_engine ) {                   // ディスプレイ表示モードがエンジン画面の場合
    lcd_s.print(tachoRpm);                            // 回転数(rpm)
    lcd_s.drawRect(9, 54, 302, 12, TFT_WHITE);        // グラフの外枠を表示
    lcd_s.fillRect(10, 55, map(tachoRpm, 0, 6500, 0, 300), 10, TFT_WHITE);
    lcd_s.fillRect(map(tachoRpm, 0, 6500, 10, 310), 55, map(tachoRpm, 0, 6500, 300, 0), 10, TFT_BLACK);
  } 

  // 2番目の表示
  if (disp == m_drive ) {                           // ディスプレイ表示モードが運転画面の場合
    uint8_t restlaps = totallaps - Lapcount;          // 残り周回数
    if (restlaps > 1){                                // 残り2周までの場合
      lcd_s.setFont(&fonts::Font7);                     // フォントを指定
      lcd_s.setTextSize(1);                             // フォントの拡大率
      lcd_s.setTextDatum(BL_DATUM);                     // データム(下・左)
      lcd_s.setCursor(140, 130);                        // 表示位置を指定
      lcd_s.print(restlaps);                            // 残り周回数
    }
    else if (restlaps == 1){                          // 残り1周の場合
      lcd_s.setFont(&fonts::lgfxJapanGothicP_20);       // フォントを指定
      lcd_s.setTextSize(2.5);                           // フォントの拡大率
      lcd_s.setTextDatum(BL_DATUM);                     // データム(下・左)
      lcd_s.setCursor(140, 135);                        // 表示位置を指定
      lcd_s.print(F("G"));
    }
    else if (restlaps < 1) {                          // 残り1周未満の場合
      lcd_s.setFont(&fonts::lgfxJapanGothicP_20);       // フォントを指定
      lcd_s.setTextSize(2.5);                           // フォントの拡大率
      lcd_s.setTextDatum(BL_DATUM);                     // データム(下・左)
      lcd_s.setCursor(140, 135);                        // 表示位置を指定
      lcd_s.print(F("FINISH"));
    }
    lcd_s.drawRect(9, 134, 302, 12, TFT_WHITE);         // グラフの外枠を表示
    lcd_s.fillRect(10, 135, map(distance, 0, goal, 300, 0), 10, TFT_WHITE);
    lcd_s.fillRect(map(distance, 0, goal, 310, 10), 135, map(distance, 0, goal, 0, 300), 10, TFT_BLACK);
    for (uint8_t i = 0; i <= totallaps; i++) {          // グラフに軸を表示
      lcd_s.setFont(&fonts::lgfxJapanGothicP_20);         // フォントを指定
      lcd_s.setTextSize(0.6);                             // フォントの拡大率
      lcd_s.setTextDatum(TC_DATUM);                       // データム(上・中央)
      lcd_s.setCursor((300 * i / totallaps) + 7, 147);    // 表示位置を指定
      lcd_s.print(i);                                     // 軸を表示
    }
  } else if (disp == m_engine ) {                   // ディスプレイ表示モードがエンジン画面の場合
    lcd_s.setFont(&fonts::Font7);                     // フォントを指定
    lcd_s.setTextSize(1);                             // フォントの拡大率
    lcd_s.setTextDatum(BL_DATUM);                     // データム(下・左)
    lcd_s.setCursor(140, 130);                        // 表示位置を指定
    lcd_s.print(INJ_timems, 1);                       // 燃料噴射時間(msec)
    lcd_s.drawRect(9, 134, 302, 12, TFT_WHITE);       // グラフの外枠を表示
    lcd_s.fillRect(10, 135, map(INJ_timems, 0, 10, 0, 300), 10, TFT_WHITE);
    lcd_s.fillRect(map(INJ_timems, 0, 10, 10, 310), 135, map(INJ_timems, 0, 10, 300, 0), 10, TFT_BLACK);
  } else if (disp == m_fuel ) {                     // ディスプレイ表示モードが燃費画面の場合
    lcd_s.setFont(&fonts::Font7);                     // フォントを指定
    lcd_s.setTextSize(1);                             // フォントの拡大率
    lcd_s.setTextDatum(BL_DATUM);                     // データム(下・左)
    lcd_s.setCursor(140, 130);                        // 表示位置を指定
    lcd_s.print(tachoRpm);                            // 回転数(rpm)
    lcd_s.drawRect(9, 134, 302, 12, TFT_WHITE);       // グラフの外枠を表示
    lcd_s.fillRect(10, 135, map(tachoRpm, 0, 6500, 0, 300), 10, TFT_WHITE);
    lcd_s.fillRect(map(tachoRpm, 0, 6500, 10, 310), 135, map(tachoRpm, 0, 6500, 300, 0), 10, TFT_BLACK);
  } 
  
  // 3番目の表示
  lcd_s.setFont(&fonts::Font7);                     // フォントを指定
  lcd_s.setTextSize(1);                             // フォントの拡大率
  lcd_s.setTextDatum(BL_DATUM);                     // データム(下・左)
  lcd_s.setCursor(140, 210);                        // 表示位置を指定
  if (disp == m_drive ) {                           // ディスプレイ表示モードが運転画面の場合
    uint8_t workmin = worktime / 60;                  // 走行時間を分の部分
    uint8_t worksec = worktime % 60;                  // 走行時間の秒の部分
    lcd_s.printf("%02d:%02d", workmin, worksec);      // 走行時間(mm:ss)
    lcd_s.drawRect(9, 214, 302, 12, TFT_WHITE);       // グラフの外枠を表示
    lcd_s.fillRect(10, 215, map(worktime, 0, limittime, 300, 0), 10, TFT_WHITE);
    lcd_s.fillRect(map(worktime, 0, limittime, 310, 10), 215, map(worktime, 0, limittime, 0, 300), 10, TFT_BLACK);
  } else if (disp == m_engine ) {                   // ディスプレイ表示モードが1の場合
    lcd_s.print(IGN_CA);                              // 進角角度(CA)
    lcd_s.drawRect(9, 214, 302, 12, TFT_WHITE);       // グラフの外枠を表示
    lcd_s.fillRect(10, 215, map(IGN_CA, 0, 90, 0, 300), 10, TFT_WHITE);
    lcd_s.fillRect(map(IGN_CA, 0, 90, 10, 310), 215, map(IGN_CA, 0, 90, 300, 0), 10, TFT_BLACK);
  } else if (disp == m_fuel ) {                     // ディスプレイ表示モードが2の場合
    lcd_s.print(dispergas, 1);                        // 燃費(km/l)
    lcd_s.drawRect(9, 214, 302, 12, TFT_WHITE);       // グラフの外枠を表示
    lcd_s.fillRect(10, 215, map(dispergas, 0, 2000, 0, 300), 10, TFT_WHITE);
    lcd_s.fillRect(map(dispergas, 0, 2000, 10, 310), 215, map(dispergas, 0, 2000, 300, 0), 10, TFT_BLACK);
  } 

  // スプライトを表示
  lcd.startWrite();
  lcd_s.pushSprite(0, 0);
  lcd.endWrite();
}

// Serial送信
void pushSerial() {
  Serial.print(la, 7);
  Serial.print(F(","));
  Serial.print(ln, 7);
  Serial.print(F(","));
  Serial.print(Loc);
  Serial.print(F(","));
  Serial.print(spd, 1);
  Serial.print(F(","));
  Serial.print(tachoRpm);
  Serial.print(F(","));
  Serial.print(speed);
  Serial.print(F(","));
  Serial.print(distance);
  Serial.print(F(","));
  Serial.print(gasml, 1);
  Serial.print(F(","));
  Serial.print(dispergas, 1);
  Serial.print(F(","));
  Serial.println(worktime);

  t_Serial = millis();
}

// Ambientへ送信
void pushAmbient() {
  if (WiFi.status() == WL_CONNECTED) {  //  Wi-Fi 接続できている場合
    Serial.println(F("WiFi:Connected"));

    char buf[16];
    ambient.set(1, spd);        // 1番目のデータとして速度をセット(GPS)
    ambient.set(2, speed);      // 2番目のデータとして速度をセット(車軸パルス)
    ambient.set(3, Lapcount);   // 3番目のデータとしてラップ数をセット
    ambient.set(4, worktime);   // 4番目のデータとして走行時間(sec)をセット
    ambient.set(5, tachoRpm);   // 5番目のデータとしてエンジン回転数(rpm)をセット
    ambient.set(6, distance);   // 6番目のデータとして走行距離積算(m)をセット
    ambient.set(7, gasml);      // 7番目のデータとして積算燃料消費量(ml)をセット
    ambient.set(8, dispergas);  // 8番目のデータとして燃費(km/l)をセット
    dtostrf(la, 12, 8, buf);
    ambient.set(9, buf);         // 9番目のデータとして緯度をセット
    dtostrf(ln, 12, 8, buf);
    ambient.set(10, buf);        // 10番目のデータとして経度をセット
    if (ambient.send(1000)) {
      Serial.println(F("Amb:Success!"));
    }
    else {
      Serial.println(F("Amb:failure..."));
    }
    t_amb = millis();
  }
  else {
    Serial.println(F("WiFi:Disconnected..."));
    WiFi.reconnect();           // 再接続
  }
}

// SDに保存
void WriteSD(){
  // ログファイルに書き込み
  logFile = SD.open(fileName, FILE_APPEND);
  if (logFile){
    logFile.printf("%d/%d/%d %02d:%02d:%02d", year(), month(), day(), hour(), minute(), second());  // 0.日時
    logFile.print(F(","));
    //logFile.print(spd, 1);    // 1.速度(GPS)
    logFile.print(speed);     // 1.速度(車軸パルス)
    logFile.print(F(","));
    logFile.print(Lapcount);  // 2.ラップ数
    logFile.print(F(","));
    logFile.print(worktime);  // 3.走行時間(sec)
    logFile.print(F(","));
    logFile.print(tachoRpm);  // 4.回転数(rpm)
    logFile.print(F(","));
    logFile.print(distance, 1);  // 5.走行距離積算(m)
    logFile.print(F(","));
    logFile.print(gasml, 1);     // 6.積算燃料消費量(ml)
    logFile.print(F(","));
    logFile.print(dispergas, 1); // 7.燃費(km/l)
    logFile.print(F(","));
    logFile.print(la, 7);     // 8.緯度
    logFile.print(F(","));
    logFile.println(ln, 7);   // 9.経度
  }
  logFile.close();

  t_SD = millis();
}

  // MQTT再接続
void MQTTreconnect() {
  uint8_t count = 0;
  while (!mqttclient.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (mqttclient.connect(mqtt_deviceID, mqtt_user, mqtt_password)) {
      Serial.println("connected");
      // 接続成功時にサブスクライブを設定
      // mqttclient.subscribe("your/subscribe/topic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttclient.state());
      Serial.println(" try again in 5 seconds");

      // 4回失敗したらMQTT送信を無効にする
      count++;
      if (count>=4) {
        MQTTpush = false;
        break;
      }
      delay(5000);
    }
  }
}

  // MQTT送信
void pushMQTT(){
  if (!mqttclient.connected()) {
    MQTTreconnect();
  }
  if (!MQTTpush) {
    return;
  }
  mqttclient.loop();

  StaticJsonDocument<512> doc;
  doc["Spd(GPS)"] = spd;
  doc["Spd(PULSE)"] = speed;
  doc["Lapcount"] = Lapcount;
  doc["worktime"] = worktime;
  doc["tachoRpm"] = tachoRpm;
  doc["distance"] = distance;
  doc["gasml"] = gasml;
  doc["dispergas"] = dispergas;
  doc["lat"] = la;
  doc["lon"] = ln;
  doc["loc"] = loc;

  // JSONオブジェクトを文字列にシリアライズ
  String jsonData;
  serializeJson(doc, jsonData);

  // MQTTでJSONデータを送信
  if (mqttclient.connected()) {
    mqttclient.publish(mqtt_topic, jsonData.c_str());
  }

  t_MQTT = millis();
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);

  Serial.begin(115200);                           // PCへのモニタリング用Serial
  Serial1.begin(115200, SERIAL_8N1, 27, 19);      // ECUとの通信用Serial(PortE TX=E2=G27, RX=E1=G19)
  Serial2.begin(115200);                          // GNSSからの受信用Serial(PortC TX=G14, RX=G13)
  
  lcd.init();                                     // TFTディスプレイの初期化
  lcd.setRotation(1);                             // 回転方向を 0～3 の4方向から設定します。(4～7を使用すると上下反転になります。)
  lcd.setBrightness(128);                         // バックライトの輝度を 0～255 の範囲で設定します。
  lcd.fillScreen(TFT_BLACK);                      // 背景色で塗りつぶし

  lcd_s.setColorDepth(1);                         // 2色モード
  lcd_s.createSprite(lcd.width(), lcd.height());  // スプライトの作成
  lcd_s.setPaletteColor(1, TFT_WHITE);

  lcd.setFont(&fonts::lgfxJapanGothicP_20);
  lcd.setTextSize(1);
  lcd.setTextDatum( baseline_center );

  // ロギング有効の場合
  if (LOGGING) {
    uint8_t i = 3;
    // SDカードマウント待ち
    while (false == SD.begin(GPIO_NUM_4, SPI, 15000000)) {
      if (i <= 0){
        LOGGING = false;
        break;
      }
      Serial.println(F("SD Wait..."));

      lcd.fillScreen(TFT_RED);
      //lcd_s.fillScreen(TFT_RED);
      lcd.setTextColor(TFT_BLACK);
      lcd.drawString("MicroSDが見つかりません", lcd.width()/2, lcd.height()/2-20);
      lcd.drawNumber(i, lcd.width()/2, lcd.height()/2+20);
      //lcd_s.pushSprite(0, 0);

      delay(1000);
      i--;
    }
  }

  // ロギング有効の場合
  if (LOGGING) {
    lcd.fillScreen(TFT_BLACK);
    //lcd_s.fillScreen(TFT_BLACK);
    lcd.setTextColor(TFT_WHITE);

    // SD内にLOGディレクトリがない場合はLOGディレクトリを作成する
    if(!SD.exists("/LOG")) {
      if(SD.mkdir("/LOG"));
      lcd.drawString("ログディレクトリ作成中...", lcd.width()/2, lcd.height()/2 - 10);
    }
    // microSD内のファイル名の連番を決定
    lcd.drawString("ログファイル作成中...", lcd.width()/2, lcd.height()/2 + 10);
    while(1){      
      sprintf(fileName, "/LOG/LOG%04d.CSV", fileNum);
      if(!SD.exists(fileName)) {
        Serial.println(fileName);
        logFile = SD.open(fileName, FILE_APPEND);
        if (logFile){
          logFile.write(0xEF);                                                  // BOMを書き込む
          logFile.write(0xBB);                                                  // BOMを書き込む
          logFile.write(0xBF);                                                  // BOMを書き込む
          logFile.println(F("記録日時,速度(km/h),ラップ数(周目),走行時間(秒),回転数(rpm),走行距離積算(m),積算燃料消費量(ml),燃費(km/l),lat(緯度),lng(経度),"));
          logFile.close();                                                      // ファイルを閉じる
        }
        break;
      }
      fileNum++;
    }
  }


  if (ambientpush || MQTTpush){
    setupWiFi();
    /*
    WiFi.begin(ssid, password);  //  Wi-Fi APに接続
    while (WiFi.status() != WL_CONNECTED) {  //  Wi-Fi AP接続待ち
      M5.update();
      if(M5.BtnA.isPressed() || M5.BtnB.isPressed() || M5.BtnC.isPressed() ) {  // いずれかのボタンを押すとambientへの送信無効化
        ambientpush = false;
        break;
      }
      lcd.clear(BLACK);
      //lcd_s.fillScreen(BLACK);
      lcd.setTextColor(WHITE);
      lcd.drawString("Wi-Fi接続待ち...", lcd.width()/2, lcd.height()/2);
      //lcd_s.pushSprite(0, 0);
      delay(100);
    }
    */
  }

  // ambientへの送信が有効の場合
  if (ambientpush) {
    if (isWifiConfigSucceeded){
      Serial.print("WiFi connected\r\nIP address: ");
      Serial.println(WiFi.localIP());

      //ambient.begin(channelId, writeKey, &client); // チャネルIDとライトキーを指定してAmbientの初期化
      uint8_t mac[6];
      esp_read_mac(mac, ESP_MAC_WIFI_STA);  // Wi-FiのMACアドレスを取得する
      sprintf(devKey, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      Serial.println(devKey);
      
      if (ambient.getchannel(userKey, devKey, channelId, writeKey, sizeof(writeKey), &client) == false) {
        Serial.printf("Cannot get channelId. Please set DeviceKey (%s) to Ambient.\r\n", devKey);
        lcd.clear(BLACK);
        lcd.setTextColor(WHITE);
        lcd.setCursor(0, 20);
        lcd.printf_P(PSTR("AmbientでチャネルIDに紐づける\nデバイスキーを登録してください\nデバイスキー: %s"), devKey);
        lcd.qrcode("https://ambidata.io/ch/devKey.html",85,87,150,5);
        while (ambient.getchannel(userKey, devKey, channelId, writeKey, sizeof(writeKey), &client) == false) {
          delay(500);
          if (ambient.getchannel(userKey, devKey, channelId, writeKey, sizeof(writeKey), &client) == true){
            break;
          }
        }
      }
      Serial.printf("channelId: %d, writeKey: %s\r\n", channelId, writeKey);
      
      ambient.begin(channelId, writeKey, &client); // 取得したチャネルIDとライトキーでAmbientの初期化
    }
    else {
      ambientpush = false;
    }
  }

  // MQTT送信が有効の場合
  if(MQTTpush) {
    if (isWifiConfigSucceeded){
      mqttclient.setServer(mqtt_server, mqtt_port);
    }
    else{
      MQTTpush = false;
    }
  }

  lcd.clear(TFT_BLACK);
  //lcd_s.fillScreen(TFT_BLACK);
  lcd.setTextColor(TFT_WHITE);
  lcd.drawString("読み込み中...", lcd.width()/2, lcd.height()/2);

  Serial.println(F("lat, lon, loc, km/h, rpm, km/h, km, ml, km/l, worktime"));

  t_Serial = millis();
  t_SD = millis();
  t_amb = millis();
  t_MQTT = millis();

  lcd.clear(TFT_BLACK);
}

void loop() {
  M5.update();

  // ECUからのデータを読み込み・表示
  readSerialECU();

  // 位置情報を取得
  getGNSS();

  // ディスプレイの表示モードを切り替えて表示
  if ( M5.BtnB.isPressed() ) { drawinfo(m_drive); }    // Bボタンを押した場合: 速度/周回数/走行時間
  if ( M5.BtnA.isPressed() ) { drawinfo(m_engine); }   // Aボタンを押した場合: 回転数/燃料噴射時間
  if ( M5.BtnC.isPressed() ) { drawinfo(m_fuel); }     // Cボタンを押した場合: 速度/回転数/燃費

  if (millis() - t_Serial >= 1 * 1000) {      // 1秒ごとにSerial送信
    if (Serial){
      pushSerial();
    }
  }

  if (MQTTpush) {   
    if (millis() - t_MQTT >= 1 * 1000) {     // 1秒ごとにMQTT送信
      pushMQTT();
    }
  }

  if (ambientpush) {                      // ambientへの送信が有効の場合
    if (millis() - t_amb >= 10 * 1000) {    // 10秒ごとにAmbientへ送信
      pushAmbient();
    }
  }

  if (LOGGING) {                          // ロギング有効の場合
    if (millis() - t_SD >= 1 * 1000) {      // 1秒ごとにSDへ記録
      WriteSD();
    }
  }

  delay(10);  // 10msec待機
}