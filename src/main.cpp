// https://qiita.com/TanakaSU/items/2f414a7a46943e085471
// 計算式 https://www.trail-note.net/tech/coordinate/

#include <Arduino.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include "SD.h"
#include <M5UnitLCD.h>
#include <M5Unified.h>
#include "Ambient.h"
#include <WiFiManager.h>
#include <TimeLib.h>
#define time_offset 32400    // UTC+9時間(60*60*9 秒）
#include <Adafruit_ADS1X15.h>
#define  IGPLS_PIN 35                 // 点火コイル.クランク1回転ごとにON

#define pi 3.141592653589793

static M5GFX lcd1;
static M5UnitLCD lcd2;
static M5Canvas lcd1_s(&lcd1);
static M5Canvas lcd2_s(&lcd2);
static M5Canvas lcd1_s_hb(&lcd1_s);

WiFiClient client;
Ambient ambient;
WiFiManager wifiManager;
Adafruit_ADS1115 ads;

//#define CFACTOR 0.18750 // GAIN_TWOTHIRDS: +/-6.144V range: Calibration factor (mV/bit) for ADS1115 
#define CFACTOR 0.12500 // GAIN_ONE: +/-4.096V range: Calibration factor (mV/bit) for ADS1115
//#define CFACTOR 0.06250 // GAIN_TWO: +/-2.048V range: Calibration factor (mV/bit) for ADS1115
//#define CFACTOR 0.03125 // GAIN_FOUR: +/-1.024V range: Calibration factor (mV/bit) for ADS1115
//#define CFACTOR 0.01563 // GAIN_EIGHT: +/-/0.512V range: Calibration factor (mV/bit) for ADS1115
//#define CFACTOR 0.00781 // GAIN_SIXTEEN: +/-0.256V range: Calibration factor (mV/bit) for ADS1115


// 変数の定義
// GPS
double goal_la, goal_ln;
double goal_la_su = 34.842925;   // 鈴鹿サーキットスタートライン
double goal_ln_su = 136.540692;  // 鈴鹿サーキットスタートライン
//double goal_la_mo = 36.532770;   // ツインリンクもてぎオーバルコーススタートライン
//double goal_ln_mo = 140.226208;  // ツインリンクもてぎオーバルコーススタートライン
double goal_la_mo = 36.533590;   // ツインリンクもてぎオーバルコーススタートライン
double goal_ln_mo = 140.225706;  // ツインリンクもてぎオーバルコーススタートライン
double goal_la_to = 35.082078;   // 豊田市SENTAN
double goal_ln_to = 137.160358;  // 豊田市SENTAN
//double goal_la_to = 35.066781;
//double goal_ln_to = 137.111857;
double la = goal_la_su;
double ln = goal_ln_su;
double spd=0.0, alt=0.0;  // 速度(km/h), 標高(m)
char labuf[12], lnbuf[12], spdbuf[6], altbuf[7], lobuf[7], O2buf[5];
String Loc = "";
float distanceTogoal = 0.0, before_distanceTogoal = 0.0;
unsigned long LAPtime, BeforeLAPtime = 0, Starttime = 0;
uint16_t Lapcount = 1, worktime, workmin, worksec, LAPRADchange = 0;
unsigned long t_Serial;           // Serialの送信時刻

// ECU
volatile bool tachopulse = false;         // 点火パルス(=クランクパルス)のトリガー
unsigned long tachoBefore = 0;            // 点火パルス(=クランクパルス)の前回の反応時の時間
unsigned long tachoAfter = 0;             // 点火パルス(=クランクパルス)の今回の反応時の時間
unsigned long tachoWidth = 0;             // 点火パルス(=クランクパルス)回転の時間　tachoAfter - tachoBefore
uint16_t RPM = 0;                         // エンジンの回転数(0-8500rpm)
uint16_t read_from_ads;                   // ADS1115から読んだ生値
uint8_t Value_THL = 0;                    // スロットル開度(0-100%)
float inputO2 = 0;                        // O2センサ出力電圧(0-1000mV)
float Value_O2 = 0;                       // O2センサから概算した空燃比(10.0-20.0)

// Wi-Fi
// const char* ssid = "****";  // Wi-Fi SSID
// const char* password = "****";  // Wi-Fi Password
bool isWifiConfigSucceeded = false;  // WiFi設定が成功したかどうかのフラグ

// ambient
// unsigned int channelId = 65530; // AmbientのチャネルID
// const char* writeKey = "050985c9530d8eb0"; // ライトキー
const char* userKey = "64bd5933d381952b59"; // ユーザーキー
char devKey[20];
unsigned int channelId;
char writeKey[20];
bool ambientpush = true;  // ambientへの送信 有効(true)/無効(false)
unsigned long t_amb;  // Ambientへの送信時刻

//ログファイル
bool LOGGING = true;     // ロギング有効/無効
File logFile;
char fileName[16];       // ファイル名
uint8_t fileNum = 0;     // ファイル連番
unsigned long t_SD;      // SDの記録時刻
uint8_t jst_year;
uint8_t jst_month;
uint8_t jst_day;
uint8_t jst_hour;
uint8_t jst_minute;
uint8_t jst_second;

HardwareSerial GPS_s(2);  // Serial2 = PortC(RX:13 TX:14)
TinyGPSPlus gps;


// タイマ割込み設定
/*
hw_timer_t * tim0 = NULL;                   //タイマー0の割り込みtim0で定義
volatile SemaphoreHandle_t timerSemaphore;  //セマフォの宣言（割込み発生の確認用）
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; //排他制御の利用を宣言

// Ambientへ送信
void IRAM_ATTR PushAmbient() {
  portENTER_CRITICAL_ISR(&timerMux);  //排他制御で以下を実行（割込み禁止）
  
  
  portEXIT_CRITICAL_ISR(&timerMux);   //排他制御終了（割り込み許可）
  xSemaphoreGiveFromISR(timerSemaphore, NULL);  //セマフォを開放
}
*/

// LCD画面にメッセージを表示する
void showMessage(String msg)
{
  //lcd1.setRotation(1);
  lcd1.fillScreen(BLACK);
  lcd1.setTextColor(WHITE);
  lcd1.setCursor(0, 20);
  lcd1.println(msg);
  lcd1.setCursor(50, 230);
  lcd1.print("A");
  lcd1.setCursor(260, 230);
  lcd1.print("C");
}

// WiFi接続モードに移行した時に呼ばれるコールバック
void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  Serial.println(myWiFiManager->getConfigPortalSSID());
  showMessage("このアクセスポイントに接続して\nWi-Fiの設定をしてください\nSSID: " + myWiFiManager->getConfigPortalSSID());
  char ConfigSSID[40];
  sprintf(ConfigSSID, "WIFI:S:%s;T:nopass;R:1;;", myWiFiManager->getConfigPortalSSID());
  lcd1.qrcode(ConfigSSID, 105, 92, 135, 5);
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
    if (ambientpush){
      showMessage("Wi-Fi接続中...");

      Serial.println("wifiManager.autoConnect()");
      if (wifiManager.autoConnect()) {
        isWifiConfigSucceeded = true;
        Serial.println("autoConnect() connect success!");
      }
      else {
        Serial.println("autoConnect() connect failed!");
      }
    }
  }

  if (isWifiConfigSucceeded) {
    showMessage("Wi-Fi接続.");
  } else {
    if (ambientpush){
      showMessage("Wi-Fi接続失敗.");
    }
    else {
      showMessage("Wi-Fi接続無効.");
    }
  }
}

// 位置情報を取得
void getGNSS() {
  while (GPS_s.available() > 0) {
    if (gps.encode(GPS_s.read())) {
      if (gps.location.isUpdated()) {
        // 経度緯度速度を取得
        if (gps.location.lng() > 120) {  // 東経120度の場合
          la         = gps.location.lat();
          ln         = gps.location.lng();
          spd        = gps.speed.kmph();
          alt        = gps.altitude.meters();
          jst_day    = gps.date.day();
          jst_month  = gps.date.month();
          jst_year   = gps.date.year();
          jst_hour   = gps.time.hour();
          jst_minute = gps.time.minute();
          jst_second = gps.time.second();
          setTime(jst_hour, jst_minute, jst_second, jst_day, jst_month, jst_year);
          // JST変換
          adjustTime(time_offset);
        }
        break;
      }
    }    
  }

  if ( la >= 34.839027 && la <= 34.84828 && ln >= 136.522015 && ln <= 136.543319 ) {
    Loc = "suzuka";
  } else if ( la >= 36.528477 && la <= 36.53735 && ln >= 140.224726 && ln <= 140.23853 ) {
    Loc = "motegi";
  } else {
    Loc = "toyota";
  }
  Loc.toCharArray( lobuf, Loc.length()+1 ); 

  dtostrf(spd, 5, 1, spdbuf);
  dtostrf(alt, 6, 1, altbuf);
  dtostrf(la, 11, 7, labuf);
  dtostrf(ln, 11, 7, lnbuf);

}

 // エンジン回転数を取得
void getRPM() {
  tachopulse = true;            // チャタリング防止のため、パルスONでtrue
}

// スロットル開度・空燃比を取得
void getTHL_O2(){
  while (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    lcd1.clear(RED);
    lcd1.setTextColor(BLACK);
    lcd1.drawString("ADS1115が見つかりません", lcd1.width()/2, lcd1.height()/2);
    delay(100);
  }

  uint16_t read_from_ads;                                   // ADS1115から読んだ生値
  read_from_ads = uint16_t(ads.readADC_Differential_0_1()); // チャネル0,1の差動(Differential)入力
  //Serial.println(read_from_ads);
  if (read_from_ads <= 2500) {
    Value_THL = 0;
  }
  else if (read_from_ads >= 19800) {
    Value_THL = 100;
  }
  else {
    Value_THL = map(read_from_ads, 2500, 19800, 0, 100);      // スロットル開度(0-100%)を算出
  }

  read_from_ads = uint16_t(ads.readADC_Differential_2_3()); // チャネル2,3の差動(Differential)入力
  inputO2 = read_from_ads * CFACTOR;                        // 校正係数を掛けてO2センサ出力電圧(mV)とする
  //Serial.println(read_from_ads);
  if (read_from_ads <= 0) {
    Value_O2 = 10.0;
  }
  else if (read_from_ads >= 4096) {
    Value_O2 = 20.0;
  }
  else {
    Value_O2 = map(read_from_ads, 0, 4096, 100, 200) * 0.1;   // 空燃比(10.0-20.0)を算出
  }
  dtostrf(Value_O2, -1, 1, O2buf);                          // 小数を含んだ数値を文字列に変換
}

// ラップタイム,周回数,走行時間を計測
void lap_count() {
  if (Loc == "suzuka") {
    goal_la = goal_la_su;
    goal_ln = goal_ln_su;
  }
  else if (Loc == "motegi") {
    goal_la = goal_la_mo;
    goal_ln = goal_ln_mo;
  }
  else if (Loc == "toyota") {
    goal_la = goal_la_to;
    goal_ln = goal_ln_to;
  }
  distanceTogoal = gps.distanceBetween(la, ln, goal_la, goal_ln);

  if (distanceTogoal > before_distanceTogoal && distanceTogoal < 15 && LAPRADchange==0) {  // 前回よりも基準点までの距離が遠い　& 基準点まで15m以内 & ラップ記録待機
    if (Lapcount >= 1 ) {
      LAPtime = (millis() - BeforeLAPtime) / 1000;
    }
    BeforeLAPtime = millis() / 1000;
    Lapcount++;
    if (Lapcount == 1){  // スタート時
      Starttime = BeforeLAPtime;  // スタート時間
    }
    LAPRADchange = 2;  // ラップ記録待機0 -> 2(リセット)
  }
  if (distanceTogoal > 15 && LAPRADchange == 2) {
    LAPRADchange = 1;  // ラップ記録待機2 -> 1(ゴール地点から離れた)
  }
  if (distanceTogoal < before_distanceTogoal && distanceTogoal < 15 && LAPRADchange == 1 ){  // 前回よりも基準点までの距離が近い　& 基準点まで15m以内
    LAPRADchange = 0;  // ラップ記録待機1 -> 0(準備)
  }
  if (Lapcount == 0){  // スタート前
    Starttime = millis() / 1000;  // 現在時間
  }
  worktime = millis() / 1000 - Starttime;
  workmin = worktime / 60;
  worksec = worktime % 60;

  before_distanceTogoal = distanceTogoal;
  
}

// 外部ディスプレイに概算空燃比を表示
void drawinfo() {
  lcd2_s.setFont(&fonts::lgfxJapanGothicP_20);
  lcd2_s.setTextSize(1.5);
  lcd2_s.fillScreen(BLACK);
  lcd2_s.setTextColor(WHITE);

  lcd2_s.setCursor(0, 0);
  lcd2_s.printf_P(PSTR("概算空燃比: %s%\n"), O2buf);

  // スプライトを表示
  lcd2.startWrite();
  lcd2_s.pushSprite(0, 0);
  lcd2.endWrite();
}

// M5Core2本体にエンジン回転数・スロットル開度・速度を表示
void drawinfo_cab() {
  lcd1_s.setFont(&fonts::lgfxJapanGothicP_20);
  lcd1_s.setTextSize(1.5);
  lcd1_s.fillScreen(BLACK);
  lcd1_s.setTextColor(WHITE);

  lcd1_s.setCursor(10, 10);
  lcd1_s.printf_P(PSTR("回転数: %drpm"), RPM);
  lcd1_s.drawRect(10, 50, 300, 20, WHITE);
  lcd1_s.fillRect(10, 50, map(RPM, 0, 8500, 0, 300), 20, WHITE);
  lcd1_s.setCursor(10, 90);
  lcd1_s.printf_P(PSTR("スロットル開度: %d%"), Value_THL);
  lcd1_s.drawRect(10, 130, 300, 20, WHITE);
  lcd1_s.fillRect(10, 130, map(Value_THL, 0, 100, 0, 300), 20, WHITE);
  lcd1_s.setCursor(10, 170);
  lcd1_s.printf_P(PSTR("速度: %skm/h"), spdbuf);
  lcd1_s.drawRect(10, 210, 300, 20, WHITE);
  if (spd <= 10) {
    lcd1_s.fillRect(10, 210, 0, 20, WHITE);
  }
  else if (spd >= 40) {
    lcd1_s.fillRect(10, 210, 300, 20, WHITE);
  }
  else {
    lcd1_s.fillRect(10, 210, map(spd, 10, 40, 0, 300), 20, WHITE);
  }

  lcd1_s_hb.fillScreen(BLACK);
  lcd1_s_hb.drawLine (0, 0, 10, 10, WHITE);

  // スプライトを表示
  lcd1.startWrite();
  //lcd1_s_hb.setPivot(315, 5);
  lcd1_s_hb.pushRotateZoom(315, 5, map(millis()*1000%60, 0, 60, 0, 359), 1, 1);	
  lcd1_s.pushSprite(0, 0);
  lcd1.endWrite();
}

// Serial送信
void pushSerial() {
  Serial.printf_P(PSTR("%s,%s,%s,%s,%s\n"), labuf, lnbuf, spdbuf, altbuf, lobuf);
    t_Serial = millis();
}

// Ambientへ送信
void pushAmbient() {
  if (WiFi.status() == WL_CONNECTED) {  //  Wi-Fi 接続できている場合
    ambient.set(1, spdbuf);     // 1番目のデータとして速度をセット
    ambient.set(2, altbuf);     // 2番目のデータとして標高をセット
    ambient.set(3, Lapcount);   // 3番目のデータとしてラップ数をセット
    ambient.set(4, worktime);   // 4番目のデータとして走行時間(sec)をセット
    ambient.set(5, RPM);        // 5番目のデータとして回転数(rpm)をセット
    ambient.set(6, Value_THL);  // 6番目のデータとしてスロットル開度(%)をセット
    ambient.set(7, O2buf);      // 7番目のデータとして概算空燃比をセット
    ambient.set(9, labuf);      // 9番目のデータとして緯度をセット
    ambient.set(10, lnbuf);     // 10番目のデータとして経度をセット

    if (ambient.send()) {
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
    logFile.printf_P(PSTR("%d/%d/%d %d:%d:%d"), year(), month(), day(), hour(), minute(), second());  // 0.日時
    logFile.print(F(","));
    logFile.print(spdbuf);    // 1.速度
    logFile.print(F(","));
    logFile.print(altbuf);    // 2.標高
    logFile.print(F(","));
    logFile.print(Lapcount);  // 3.ラップ数
    logFile.print(F(","));
    logFile.print(worktime);  // 4.走行時間(sec)
    logFile.print(F(","));
    logFile.print(RPM);       // 5.回転数(rpm)
    logFile.print(F(","));
    logFile.print(Value_THL); // 6.スロットル開度(%)
    logFile.print(F(","));
    logFile.print(O2buf);     // 7.概算空燃比
    logFile.print(F(","));
    logFile.print("");        // 8.
    logFile.print(F(","));
    logFile.print(labuf);     // 9.緯度
    logFile.print(F(","));
    logFile.println(lnbuf);   // 10.経度
  }
  logFile.close();

  t_SD = millis();
}

void setup()
{
  auto cfg = M5.config();
  M5.begin(cfg);
  Wire.begin(M5.Ex_I2C.getSDA(), M5.Ex_I2C.getSCL());         // PortA
  Wire1.begin(21, 22);                                        // M5Core2の内部I2C(M5Unifiedでは無効化？)

  pinMode(IGPLS_PIN, INPUT);

  lcd1.init();
  //lcd2.init();
  //lcd2.setRotation(3);
  lcd1_s.createSprite(lcd1.width(), lcd1.height());
  //lcd2_s.createSprite(lcd2.width(), lcd2.height());
  lcd1_s_hb.createSprite(10, 10);
  GPS_s.begin(115200);

  lcd1.setFont(&fonts::lgfxJapanGothicP_20);
  lcd1.setTextSize(1.0);
  lcd1.setTextDatum( baseline_center );

  delay(500);
  
  // アナログ電圧測定のため、ADS1115有効化
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  ads.setGain(GAIN_ONE);           // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    lcd1.clear(RED);
    //lcd1_s.fillScreen(RED);
    lcd1.setTextColor(BLACK);
    lcd1.drawString("ADS1115が見つかりません", lcd1.width()/2, lcd1.height()/2);
    //lcd1_s.pushSprite(0, 0);
    while (1);
  }

  // SDカードマウント待ち
  if (LOGGING) {                                  // ロギング有効の場合
    uint8_t i = 0;
    while (false == SD.begin(GPIO_NUM_4, SPI, 15000000)) {
      if (i > 6){
        LOGGING = false;
        break;
      }
      Serial.println("SD Wait...");

      lcd1.clear(RED);
      //lcd1_s.fillScreen(RED);
      lcd1.setTextColor(BLACK);
      lcd1.drawString("MicroSDが見つかりません", lcd1.width()/2, lcd1.height()/2);
      //lcd1_s.pushSprite(0, 0);

      delay(500);
      i++;
    }
  }

  if (LOGGING) {                                  // ロギング有効の場合
    lcd1.fillScreen(TFT_BLACK);
    //lcd_s.fillScreen(TFT_BLACK);
    lcd1.setTextColor(TFT_WHITE);

    // SD内にLOGディレクトリがない場合はLOGディレクトリを作成する
    if(!SD.exists("/LOG")) {
      if(SD.mkdir("/LOG"));
    }
    // microSD内のファイル名の連番を決定
    while(1){      
      sprintf_P(fileName, PSTR("/LOG/LOG%04d.CSV"), fileNum);
      if(!SD.exists(fileName)) {
        Serial.println(fileName);
        logFile = SD.open(fileName, FILE_APPEND);
        if (logFile){
          logFile.write(0xEF);                                                  // BOMを書き込む
          logFile.write(0xBB);                                                  // BOMを書き込む
          logFile.write(0xBF);                                                  // BOMを書き込む
          logFile.println(F("created,速度(km/h),標高(m),ラップ数(周目),走行時間(秒),回転数(rpm),スロットル開度(%),概算空燃比, ,lat,lng,"));
          logFile.close();                                                      // ファイルを閉じる
        }
        break;
      }
      fileNum++;
    }
  }

  if (ambientpush) {             // ambientへの送信が有効の場合
    setupWiFi();
    /*
    WiFi.begin(ssid, password);  //  Wi-Fi APに接続
    while (WiFi.status() != WL_CONNECTED) {  //  Wi-Fi AP接続待ち
      M5.update();
      if(M5.BtnA.isPressed() || M5.BtnB.isPressed() || M5.BtnC.isPressed() ) {  // いずれかのボタンを押すとambientへの送信無効化
        ambientpush = false;
        break;
      }
      lcd1.clear(BLACK);
      //lcd1_s.fillScreen(BLACK);
      lcd1.setTextColor(WHITE);
      lcd1.drawString("Wi-Fi接続待ち...", lcd1.width()/2, lcd1.height()/2);
      //lcd1_s.pushSprite(0, 0);
      delay(100);
    }
    */
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
        lcd1.clear(BLACK);
        lcd1.setTextColor(WHITE);
        lcd1.setCursor(0, 20);
        lcd1.printf_P(PSTR("AmbientでチャネルIDに紐づける\nデバイスキーを登録してください\nデバイスキー: %s"), devKey);
        lcd1.qrcode("https://ambidata.io/ch/devKey.html",85,87,150,5);
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
  }
  
  lcd1.clear(BLACK);
  //lcd1_s.fillScreen(BLACK);
  lcd1.setTextColor(WHITE);
  lcd1.drawString("読み込み中...", lcd1.width()/2, lcd1.height()/2);
  //lcd1_s.pushSprite(0, 0);

  //lcd2.setFont(&fonts::lgfxJapanGothicP_20);
  //lcd2.setTextSize(1.0);
  //lcd2.setTextDatum( baseline_center );
  //lcd2.clear(BLACK);
  //lcd2.setTextColor(WHITE);
  //lcd2.drawString("読み込み中...", lcd2.width()/2, lcd2.height()/2);

  Serial.println(F("lat,lon,spd,alt"));

  t_amb = millis();
  t_SD = millis();

  // タイマ割込み設定
    /*
  timerSemaphore = xSemaphoreCreateBinary();  //バイナリセマフォを作成(0か1のバイナリ)
  tim0 = timerBegin(0, 80, true);             //タイマー0を80MHz/80（1us）動作ｶｳﾝﾄｱｯﾌﾟでtim0に設定
  timerAttachInterrupt(tim0, &PushAmbient, true); //tim0割込みが発生した時に実行する処理を指定「GetGNSS」
  timerAlarmWrite(tim0, 10000000, true);         //tim0割込み発生周期を10s（1us × 1000(ms) x 1000(s) x 10）に設定
  timerAlarmEnable(tim0);                     //タイマー0割込みを有効化
  */

  // 回転数取得用の割り込みを登録 トリガはLOWになった時
	attachInterrupt((IGPLS_PIN), getRPM, FALLING);
}

void loop()
{
  M5.update();

  if(M5.BtnA.wasPressed()){
    la = goal_la_su;
    ln = goal_ln_su;
  }

  if(M5.BtnB.wasPressed()){
    la = goal_la_mo;
    ln = goal_ln_mo;
  }

  if(M5.BtnC.wasPressed()){
    LAPRADchange = 0;  // ラップ記録待機0(準備)
  }

  // 位置情報を取得
  getGNSS();

  // エンジン回転数を取得
  if (tachopulse){
    tachoAfter = micros();                                    // 現在の時刻を記録
    tachoWidth = tachoAfter - tachoBefore;                    // 前回と今回の時間の差(カムシャフト1回転当たりの時間)を計算
    RPM = 60000000 / tachoWidth;                              // クランクの回転数[rpm]を計算
    tachoBefore = tachoAfter;                                 // 今回の値を前回の値に代入する
    tachopulse = false;
  }
  if (micros() - tachoBefore > 60000000 * 0.01 ){RPM = 0;}   // 100rpm以下の時は0にする 

  // スロットル開度・空燃比を取得
  getTHL_O2();

  // ラップタイム,周回数,走行時間を計測
  lap_count();

  // 外部ディスプレイに速度と高度を表示
  drawinfo();
  
  if (millis() - t_Serial >= 1 * 1000) {      // 1秒ごとにSerial送信
    pushSerial();
  }

  // M5Core2本体にエンジン回転数・スロットル開度・空燃比を表示
  drawinfo_cab();

  if (ambientpush) {                      // ambientへの送信が有効の場合
    if (millis() - t_amb >= 10 * 1000) {  // 10秒ごとにAmbientへ送信
      pushAmbient();
    }
  }

  if (millis() - t_SD >= 1 * 1000) {      // 1秒ごとにSDへ記録
    WriteSD();
  }


  delay(10);                              // 6000rpm以上は無視する
}