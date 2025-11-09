// secrets.example.h
// 機微情報テンプレート: このファイルをコピーして secrets.h を作成し、実際の値を入力してください。
// secrets.h は .gitignore 済みのためリポジトリへコミットしないこと。

#pragma once

// MQTT / AWS IoT Core
// 例: "xxxxxxxxxx-ats.iot.ap-northeast-*.amazonaws.com"
extern const char* mqtt_server;     
extern const int   mqtt_port;        // 通常 8883 (TLS)
extern const char* mqtt_topic;       // 例: "Furoshiki/M5Logger"
extern const char* mqtt_deviceID;    // Thing Name / Client ID

// Ambient
extern const char* userKey;          // Ambient user key (channelId / writeKey は動的取得 or 追加定義可)

// Amazon Root CA 1
extern const char AWS_CERT_CA[] PROGMEM;    
// Device Certificate
extern const char AWS_CERT_CRT[] PROGMEM;   
// Device Private Key
extern const char AWS_CERT_PRIVATE[] PROGMEM; 

/*
実際の secrets.h の例: 
--------------------------------------------------
const char* mqtt_server = "xxxxxxxxxx-ats.iot.ap-northeast-*.amazonaws.com";
const int   mqtt_port   = 8883;
const char* mqtt_topic  = "Furoshiki/M5Logger";
const char* mqtt_deviceID = "M5Core2_Furoshiki";
const char* userKey = "your_ambient_user_key";

static const char AWS_CERT_CA[] PROGMEM = R"EOF(-----BEGIN CERTIFICATE-----\n...\n-----END CERTIFICATE-----\n)EOF";
static const char AWS_CERT_CRT[] PROGMEM = R"KEY(-----BEGIN CERTIFICATE-----\n...\n-----END CERTIFICATE-----\n)KEY";
static const char AWS_CERT_PRIVATE[] PROGMEM = R"KEY(-----BEGIN RSA PRIVATE KEY-----\n...\n-----END RSA PRIVATE KEY-----\n)KEY";
--------------------------------------------------
*/
