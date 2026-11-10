#pragma once

#include <stddef.h>
#include <stdint.h>

// u-blox M10（MAX-M10S）へ送る UBX メッセージの組み立てと、受信した NMEA の数え方（#28）
// Arduino非依存（PC上でテストできる）。Serial2 への送受信は main.cpp 側で行う。
//
// MAX-M10S にはフラッシュがなく、設定は RAM と BBR（V_BCKP の電池で保つ RAM）にしか残らない。
// 電池が切れると出荷時の設定（38400 bps・1 Hz・6 種類のセンテンス）に戻り、ロガーで読めなくなる。
// そこで起動時に出荷時の設定に戻っていないかを確かめ、戻っていれば
// tools/setup_max_m10s.py の既定と同じ設定（115200 bps・10 Hz・RMC のみ・GPS+Galileo+QZSS+SBAS）を送り直す。
namespace UbloxM10 {

constexpr uint32_t FACTORY_BAUD = 38400;  // MAX-M10S の出荷時のボーレート
constexpr uint32_t LOGGER_BAUD = 115200;  // src/main.cpp の gnss_bps と同じ

constexpr uint8_t LAYER_RAM = 0x01;  // UBX-CFG-VALSET の layers（ビットの和）
constexpr uint8_t LAYER_BBR = 0x02;

// 設定キー（u-blox M10 SPG 5.10 のインタフェース記述）。値の大きさはキーの 28〜30 ビットで決まる
constexpr uint32_t KEY_RATE_MEAS = 0x30210001;  // 測位間隔 [ms]
constexpr uint32_t KEY_RATE_NAV = 0x30210002;   // 何回の測定ごとに測位するか
constexpr uint32_t KEY_UART1_BAUD = 0x40520001;
constexpr uint32_t KEY_NMEA_GGA = 0x209100BB;  // UART1 へ何測位ごとに出すか（0 は出さない）
constexpr uint32_t KEY_NMEA_GLL = 0x209100CA;
constexpr uint32_t KEY_NMEA_GSA = 0x209100C0;
constexpr uint32_t KEY_NMEA_GSV = 0x209100C5;
constexpr uint32_t KEY_NMEA_RMC = 0x209100AC;
constexpr uint32_t KEY_NMEA_VTG = 0x209100B1;
constexpr uint32_t KEY_SIGNAL_GPS = 0x1031001F;
constexpr uint32_t KEY_SIGNAL_SBAS = 0x10310020;
constexpr uint32_t KEY_SIGNAL_GAL = 0x10310021;
constexpr uint32_t KEY_SIGNAL_BDS = 0x10310022;
constexpr uint32_t KEY_SIGNAL_QZSS = 0x10310024;
constexpr uint32_t KEY_SIGNAL_GLO = 0x10310025;

struct KeyValue {
  uint32_t key;
  uint32_t value;
};

// ロガー用の設定（ボーレート以外）。tools/setup_max_m10s.py の expected_values(10, "rmc", "gps-gal") と同じ順・同じ値
// 出荷時の GPS+Galileo+BeiDou では 10 Hz の計算が間に合わない（実測 約 9 Hz）ので、BeiDou を止める
constexpr KeyValue LOGGER_CONFIG[] = {
    {KEY_RATE_MEAS, 100},
    {KEY_RATE_NAV, 1},
    {KEY_NMEA_GGA, 0},
    {KEY_NMEA_GLL, 0},
    {KEY_NMEA_GSA, 0},
    {KEY_NMEA_GSV, 0},
    {KEY_NMEA_RMC, 1},
    {KEY_NMEA_VTG, 0},
    {KEY_SIGNAL_GPS, 1},
    {KEY_SIGNAL_SBAS, 1},
    {KEY_SIGNAL_GAL, 1},
    {KEY_SIGNAL_BDS, 0},
    {KEY_SIGNAL_QZSS, 1},
    {KEY_SIGNAL_GLO, 0},
};
constexpr size_t LOGGER_CONFIG_COUNT = sizeof(LOGGER_CONFIG) / sizeof(LOGGER_CONFIG[0]);

constexpr size_t MAX_FRAME_BYTES = 96;  // このファイルで組み立てるフレームの最大長（LOGGER_CONFIG で 84 バイト）

// キーの値のバイト数（1 ビットの値も 1 バイトで送る）。不明なら 0
inline size_t valueSize(uint32_t key) {
  switch ((key >> 28) & 0x7) {
    case 1:
    case 2:
      return 1;
    case 3:
      return 2;
    case 4:
      return 4;
    default:
      return 0;  // 8 バイトの値はこのファイルでは使わない
  }
}

// UBX フレーム（同期文字 + クラス + ID + 長さ + 中身 + チェックサム）を out に書き、長さを返す。入りきらなければ 0
inline size_t buildFrame(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t length, uint8_t* out,
                         size_t outSize) {
  const size_t total = static_cast<size_t>(length) + 8;
  if (total > outSize) {
    return 0;
  }
  out[0] = 0xB5;
  out[1] = 0x62;
  out[2] = cls;
  out[3] = id;
  out[4] = static_cast<uint8_t>(length & 0xFF);
  out[5] = static_cast<uint8_t>(length >> 8);
  for (uint16_t i = 0; i < length; i++) {
    out[6 + i] = payload[i];
  }
  uint8_t a = 0;
  uint8_t b = 0;
  for (size_t i = 2; i < total - 2; i++) {
    a = static_cast<uint8_t>(a + out[i]);
    b = static_cast<uint8_t>(b + a);
  }
  out[total - 2] = a;
  out[total - 1] = b;
  return total;
}

// UBX-CFG-VALSET（values を layers に書く）のフレームを out に書き、長さを返す。入りきらなければ 0
inline size_t buildValset(const KeyValue* values, size_t count, uint8_t layers, uint8_t* out, size_t outSize) {
  uint8_t payload[MAX_FRAME_BYTES];
  size_t length = 0;
  payload[length++] = 0;  // version
  payload[length++] = layers;
  payload[length++] = 0;  // reserved
  payload[length++] = 0;
  for (size_t i = 0; i < count; i++) {
    const size_t size = valueSize(values[i].key);
    if (size == 0 || length + 4 + size > sizeof(payload)) {
      return 0;
    }
    for (size_t j = 0; j < 4; j++) {
      payload[length++] = static_cast<uint8_t>(values[i].key >> (8 * j));
    }
    for (size_t j = 0; j < size; j++) {
      payload[length++] = static_cast<uint8_t>(values[i].value >> (8 * j));
    }
  }
  return buildFrame(0x06, 0x8A, payload, static_cast<uint16_t>(length), out, outSize);
}

// ロガー用の設定（ボーレート以外）を RAM と BBR に書くフレーム
inline size_t buildLoggerConfig(uint8_t* out, size_t outSize) {
  return buildValset(LOGGER_CONFIG, LOGGER_CONFIG_COUNT, LAYER_RAM | LAYER_BBR, out, outSize);
}

// UART1 のボーレートを RAM と BBR に書くフレーム。送るとすぐに切り替わるので、設定の最後に送る
inline size_t buildBaudrate(uint32_t baud, uint8_t* out, size_t outSize) {
  const KeyValue value = {KEY_UART1_BAUD, baud};
  return buildValset(&value, 1, LAYER_RAM | LAYER_BBR, out, outSize);
}

// UBX-CFG-RST のホットスタート（BBR の設定と衛星の軌道情報を残し、GNSS だけをソフトウェアで再起動する）
inline size_t buildHotStart(uint8_t* out, size_t outSize) {
  const uint8_t payload[] = {0x00, 0x00, 0x02, 0x00};  // navBbrMask=0（何も消さない）, resetMode=2, reserved
  return buildFrame(0x06, 0x04, payload, sizeof(payload), out, outSize);
}

// 受信した文字から、チェックサムの合う NMEA の文を数える
// 起動時に、今のボーレートでモジュールの出力が読めるかを見分けるのに使う
class NmeaCounter {
 public:
  void feed(char ch) {
    if (ch == '$') {
      length_ = 0;
      inSentence_ = true;
    }
    if (!inSentence_) {
      return;
    }
    if (ch == '\r' || ch == '\n') {
      finish();
      inSentence_ = false;
      return;
    }
    if (length_ >= sizeof(line_)) {
      inSentence_ = false;  // 長すぎるのは NMEA ではない（違うボーレートで化けた文字など）
      return;
    }
    line_[length_++] = ch;
  }

  uint16_t valid() const { return valid_; }  // チェックサムの合った文の数
  uint16_t rmc() const { return rmc_; }      // そのうち RMC の数

 private:
  static int hexValue(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
  }

  void finish() {
    // "$" + トーカー2文字 + 種類3文字 + ... + "*" + 16進2文字
    if (length_ < 10 || line_[length_ - 3] != '*') {
      return;
    }
    const int hi = hexValue(line_[length_ - 2]);
    const int lo = hexValue(line_[length_ - 1]);
    if (hi < 0 || lo < 0) {
      return;
    }
    uint8_t sum = 0;
    for (size_t i = 1; i < length_ - 3; i++) {
      sum ^= static_cast<uint8_t>(line_[i]);
    }
    if (sum != static_cast<uint8_t>(hi * 16 + lo)) {
      return;
    }
    valid_++;
    if (line_[3] == 'R' && line_[4] == 'M' && line_[5] == 'C') {
      rmc_++;
    }
  }

  char line_[96] = {};
  size_t length_ = 0;
  bool inSentence_ = false;
  uint16_t valid_ = 0;
  uint16_t rmc_ = 0;
};

}  // namespace UbloxM10
