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

#define pi 3.141592653589793

static M5GFX lcd1;
static M5UnitLCD lcd2;
static M5Canvas lcd1_s(&lcd1);
static M5Canvas lcd2_s(&lcd2);

WiFiClient client;
Ambient ambient;
WiFiManager wifiManager;

//変数の定義
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
char labuf[12], lnbuf[12], spdbuf[6], altbuf[7], lobuf[7];
String Loc = "";
double L = 85.05112878;
int z = 18;  //ズーム倍率
float distanceTogoal = 0.0, before_distanceTogoal = 0.0;
unsigned long LAPtime, BeforeLAPtime = 0, Starttime = 0;
int Lapcount = 1, worktime, workmin, worksec, LAPRADchange = 0;

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
bool ambientpush = true;  // // ambientへの送信 有効(true)/無効(false)
unsigned long t_amb;  // Ambientへの送信時刻

//ログファイル
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
#define time_offset 32400    // UTC+9時間(60*60*9 秒）

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
  lcd1.setCursor(0, 20);
  lcd1.fillScreen(BLACK);
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

  Serial.printf_P(PSTR("%s,%s,%s,%s,%s\n"), labuf, lnbuf, spdbuf, altbuf, lobuf);
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

// 地図を表示
void drawmap() {  
  // 緯度経度→ピクセル座標の変換計算
  double px = int(pow(2.0, z + 7.0) * ((ln / 180.0) + 1.0));
  double py = int(pow(2.0, z + 7.0) * (-1 * atanh(sin(pi * la / 180.0)) + atanh(sin(pi * L / 180.0))) / pi);
  double px_goal = int(pow(2.0, z + 7.0) * ((goal_ln / 180.0) + 1.0));
  double py_goal = int(pow(2.0, z + 7.0) * (-1 * atanh(sin(pi * goal_la / 180.0)) + atanh(sin(pi * L / 180.0))) / pi);
  
  // ピクセル座標→タイル座標の変換計算
  int tx = px / 256;
  int ty = py / 256;
  
  // タイル画像の中の座標を計算
  int x = int(px) % 256;
  int y = int(py) % 256;

  // 画像9枚のファイルアドレスを用意
  String filename[9];
  filename[0] = String("/pale/" + String(z) + "/" + String(tx - 1) + "/" + String(ty-1) + ".png");
  filename[1] = String("/pale/" + String(z) + "/" + String(tx) + "/" + String(ty-1) + ".png");
  filename[2] = String("/pale/" + String(z) + "/" + String(tx + 1) + "/" + String(ty-1) + ".png");
  filename[3] = String("/pale/" + String(z) + "/" + String(tx - 1) + "/" + String(ty) + ".png");
  filename[4] = String("/pale/" + String(z) + "/" + String(tx) + "/" + String(ty) + ".png");
  filename[5] = String("/pale/" + String(z) + "/" + String(tx + 1) + "/" + String(ty) + ".png");
  filename[6] = String("/pale/" + String(z) + "/" + String(tx - 1) + "/" + String(ty+1) + ".png");
  filename[7] = String("/pale/" + String(z) + "/" + String(tx) + "/" + String(ty+1) + ".png");
  filename[8] = String("/pale/" + String(z) + "/" + String(tx + 1) + "/" + String(ty+1) + ".png");

  /*
  画像9枚の並びはこのようになっている
  [0][1][2]
  [3][4][5]
  [6][7][8]
  */

  // Stringからchar配列に変換
  for (int i = 0; i < 9; i++)
  {
    int str_len = filename[i].length() + 1;
    char file[9][str_len];
    filename[i].toCharArray(file[i], str_len);
  }

  // filename[4]を中心として画像を描画
  int mainx = -1 * (x - lcd1.width()/2), mainy = -1 * (y - lcd1.height()/2);
  int mainx_goal = px_goal - px + lcd1.width()/2, mainy_goal = py_goal - py + lcd1.height()/2;  // 基準点の画面に対する座標

  lcd1_s.drawPngFile(SD, filename[4], mainx, mainy);
  // 他8枚の画像を描画
  if (mainx > 0 && mainy > 0) {
    lcd1_s.drawPngFile(SD, filename[0], mainx - 256, mainy-256);
  }
  if (mainy > 0) {
    lcd1_s.drawPngFile(SD, filename[1], mainx , mainy-256);
  }
  if (mainx + 256 < lcd1.width() && mainy >0) {
    lcd1_s.drawPngFile(SD, filename[2], mainx + 256, mainy - 256);
  }


  if (mainx > 0) {
    lcd1_s.drawPngFile(SD, filename[3], mainx - 256, mainy);
  }
  if (mainx + 256 < lcd1.width()) {
    lcd1_s.drawPngFile(SD, filename[5], mainx + 256, mainy);
  }


  if (mainx > 0 && mainy < -26) {
    lcd1_s.drawPngFile(SD, filename[6], mainx - 256, mainy + 256);
  }
  if (mainy < -26) {
    lcd1_s.drawPngFile(SD, filename[7], mainx, mainy + 256);
  }
  if (mainx + 256 < lcd1.width() && mainy < -26) {
    lcd1_s.drawPngFile(SD, filename[8], mainx + 256, mainy + 256);
  }

  // 中心に印をつける
  lcd1_s.fillCircle(lcd1.width()/2, lcd1.height()/2, 8, TFT_CYAN);
  lcd1_s.fillCircle(lcd1.width()/2, lcd1.height()/2, 5, TFT_BLUE);
  if (mainx_goal >= 0 && mainx_goal <= lcd1.width() && mainy_goal >= 0 && mainy_goal <= lcd1.height()) {  // 基準点が画面内にある場合
    lcd1_s.fillCircle(mainx_goal, mainy_goal, 2, TFT_RED);
  }

  // スプライトを表示
  lcd1.startWrite();
  lcd1_s.pushSprite(0, 0);
  lcd1.endWrite();
}

// 外部ディスプレイに速度,周回数,走行時間を表示
void drawinfo() {
  lcd2_s.setFont(&fonts::lgfxJapanGothicP_20);
  lcd2_s.setTextSize(1.5);
  lcd2_s.fillScreen(BLACK);
  lcd2_s.setTextColor(ORANGE);

  lcd2_s.setCursor(0, 0);
  lcd2_s.printf_P(PSTR("速度: %skm/h\n"), spdbuf);
  lcd2_s.printf_P(PSTR("ラップ: %d周目\n"), Lapcount);
  lcd2_s.printf_P(PSTR("時間: %d分%d秒\n"), workmin, worksec);

  // スプライトを表示
  lcd2.startWrite();
  lcd2_s.pushSprite(5, 5);
  lcd2.endWrite();
}

// Ambientへ送信
void pushAmbient() {
  if (WiFi.status() == WL_CONNECTED) {  //  Wi-Fi 接続できている場合
    ambient.set(1, spdbuf); // 1番目のデータとして速度をセット
    ambient.set(2, altbuf); // 2番目のデータとして標高をセット
    ambient.set(3, Lapcount); // 3番目のデータとしてラップ数をセット
    ambient.set(4, worktime); // 4番目のデータとして走行時間(sec)をセット
    ambient.set(9, labuf);  // 9番目のデータとして緯度をセット
    ambient.set(10, lnbuf); // 10番目のデータとして経度をセット

    ambient.send();
    t_amb = millis();
  }
}

// SDに保存
void WriteSD(){
  // ファイル名の連番を決定
  while(1){      
    sprintf_P(fileName, PSTR("LOG/LOG%04d.CSV"), fileNum);
    if(!SD.exists(fileName)) {
      Serial.println(fileName);
      break;
    }
    fileNum++;
  }

  // ログファイルが無かったらヘッダを書き込む
  if(!SD.exists(fileName)) {
    logFile = SD.open(fileName, FILE_WRITE);
    if (logFile){
      logFile.println(F("created,速度(km/h),標高(m),ラップ数(周目),走行時間(秒),	,	, , ,lat,lng,"));
    }
  }

  // ログファイルに書き込み
  logFile = SD.open(fileName, FILE_WRITE);
  if (logFile){
    logFile.printf_P(PSTR("%d/%d/%d %d:%d:%d"), jst_year, jst_month, jst_day, jst_hour, jst_minute, jst_second);  // 0.日時
    logFile.print(F(","));
    logFile.print(spdbuf);    // 1.速度
    logFile.print(F(","));
    logFile.print(altbuf);    // 2.標高
    logFile.print(F(","));
    logFile.print(Lapcount);  // 3.ラップ数
    logFile.print(F(","));
    logFile.print(worktime);  // 4.走行時間(sec)
    logFile.print(F(","));
    logFile.print("");        // 5.
    logFile.print(F(","));
    logFile.print("");        // 6.
    logFile.print(F(","));
    logFile.print("");        // 7.
    logFile.print(F(","));
    logFile.print("");        // 8.
    logFile.print(F(","));
    logFile.print(labuf);   // 9.緯度
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

  lcd1.init();
  lcd2.init();
  lcd2.setRotation(3);
  lcd1_s.createSprite(lcd1.width(), lcd1.height());
  lcd2_s.createSprite(lcd2.width(), lcd2.height());
  GPS_s.begin(115200);

  lcd1.setFont(&fonts::lgfxJapanGothicP_20);
  lcd1.setTextSize(1.0);
  lcd1.setTextDatum( baseline_center );

  delay(500);
  // SDカードマウント待ち
  while (false == SD.begin(GPIO_NUM_4, SPI, 15000000)) {
    Serial.println("SD Wait...");

    lcd1.clear(RED);
    //lcd1_s.fillScreen(RED);
    lcd1.setTextColor(BLACK);
    lcd1.drawString("MicroSDが見つかりません", lcd1.width()/2, lcd1.height()/2);
    //lcd1_s.pushSprite(0, 0);

    delay(500);
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

  lcd2.setFont(&fonts::lgfxJapanGothicP_20);
  lcd2.setTextSize(1.0);
  lcd2.setTextDatum( baseline_center );
  lcd2.clear(BLACK);
  //lcd2_s.fillScreen(BLACK);
  lcd2.setTextColor(WHITE);
  lcd2.drawString("読み込み中...", lcd2.width()/2, lcd2.height()/2);
  //lcd2_s.pushSprite(0, 0);

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

  // ラップタイム,周回数,走行時間を計測
  lap_count();

  // 地図を表示
  drawmap();

  // 外部ディスプレイに速度と高度を表示
  drawinfo();

  if (ambientpush) {                      // ambientへの送信が有効の場合
    if (millis() - t_amb >= 10 * 1000) {  // 10秒ごとにAmbientへ送信
      pushAmbient();
    }
  }

  if (millis() - t_SD >= 10 * 1000) {  // 1秒ごとにSDへ記録
    WriteSD();
  }


  delay(100);
}