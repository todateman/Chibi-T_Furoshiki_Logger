#pragma once

#include <Arduino.h>
#include <Wire.h>

// M5Stack Unit ENV-III（SHT30: 気温・湿度 / QMP6988: 気圧）の読み出し
// M5Unit-ENV の旧ドライバは1回の読み出しで250ms待つため、loop()を止めない形で書き直したもの。
// 気圧の補償計算は M5Unit-ENV (MIT License, M5Stack) の QMP6988 ドライバから移植している。
class UnitEnvIII {
 public:
  static constexpr uint8_t SHT30_ADDR = 0x44;
  static constexpr uint8_t QMP6988_ADDR = 0x70;

  // SHT30とQMP6988の両方が応答した場合のみtrue（Wire.begin()は呼び出し側で済ませておく）
  bool begin(TwoWire& wire) {
    wire_ = &wire;
    return beginSht30() && beginQmp6988();
  }

  // QMP6988から気圧[Pa]を読み出す
  bool readPressurePa(float& outPa) {
    uint8_t buf[6];
    if (!readRegisters(QMP6988_ADDR, QMP_DATA_REG, buf, sizeof(buf))) {
      return false;
    }
    const int32_t dp = static_cast<int32_t>(((uint32_t)buf[0] << 16 | (uint32_t)buf[1] << 8 | buf[2]) - RAW_OFFSET);
    const int32_t dt = static_cast<int32_t>(((uint32_t)buf[3] << 16 | (uint32_t)buf[4] << 8 | buf[5]) - RAW_OFFSET);
    const int16_t t256 = convertTemperature256(dt);
    outPa = static_cast<float>(convertPressure16(dp, t256)) / 16.0f;
    return true;
  }

  // SHT30から気温[°C]と湿度[%]を読み出す
  // 前回の呼び出しで始めた測定の結果を読み、次の測定を始めておく（測定に約15msかかるため、続けて呼ばないこと）
  bool readTemperatureHumidity(float& outC, float& outPct) {
    bool ok = false;
    uint8_t buf[6];
    if (wire_->requestFrom(static_cast<int>(SHT30_ADDR), static_cast<int>(sizeof(buf))) == static_cast<int>(sizeof(buf))) {
      for (uint8_t i = 0; i < sizeof(buf); i++) {
        buf[i] = static_cast<uint8_t>(wire_->read());
      }
      if (crc8(buf) == buf[2] && crc8(buf + 3) == buf[5]) {
        outC = -45.0f + 175.0f * static_cast<float>((uint16_t)buf[0] << 8 | buf[1]) / 65535.0f;
        outPct = 100.0f * static_cast<float>((uint16_t)buf[3] << 8 | buf[4]) / 65535.0f;
        ok = true;
      }
    }
    startSht30Measurement();
    return ok;
  }

 private:
  // SHT30のコマンド
  static constexpr uint16_t SHT_SOFT_RESET = 0x30A2;
  static constexpr uint16_t SHT_SINGLE_SHOT_HIGH = 0x2400;  // 高再現性・クロックストレッチなし

  // QMP6988のレジスタ
  static constexpr uint8_t QMP_CHIP_ID_REG = 0xD1;
  static constexpr uint8_t QMP_CHIP_ID = 0x5C;
  static constexpr uint8_t QMP_RESET_REG = 0xE0;
  static constexpr uint8_t QMP_CONFIG_REG = 0xF1;     // IIRフィルター
  static constexpr uint8_t QMP_CTRL_MEAS_REG = 0xF4;  // オーバーサンプリング・動作モード
  static constexpr uint8_t QMP_IO_SETUP_REG = 0xF5;   // 待機時間
  static constexpr uint8_t QMP_DATA_REG = 0xF7;       // 気圧3byte + 温度3byte
  static constexpr uint8_t QMP_CALIBRATION_REG = 0xA0;
  static constexpr uint32_t RAW_OFFSET = 8388608;     // 2^23

  // 補償計算用の係数（コメントは固定小数点の形式）
  struct Calibration {
    int32_t a0, b00;  // 20Q4
    int32_t a1, a2;
    int64_t bt1, bt2, bp1, b11, bp2, b12, b21, bp3;
  };

  TwoWire* wire_ = nullptr;
  Calibration cal_{};

  bool beginSht30() {
    if (!writeSht30Command(SHT_SOFT_RESET)) {
      return false;
    }
    delay(2);
    // 起動直後の1回目の読み出しに間に合うよう、最初の測定を待っておく
    if (!startSht30Measurement()) {
      return false;
    }
    delay(20);
    return true;
  }

  bool startSht30Measurement() { return writeSht30Command(SHT_SINGLE_SHOT_HIGH); }

  bool writeSht30Command(uint16_t command) {
    wire_->beginTransmission(SHT30_ADDR);
    wire_->write(static_cast<uint8_t>(command >> 8));
    wire_->write(static_cast<uint8_t>(command & 0xFF));
    return wire_->endTransmission() == 0;
  }

  bool beginQmp6988() {
    uint8_t id = 0;
    if (!readRegisters(QMP6988_ADDR, QMP_CHIP_ID_REG, &id, 1) || id != QMP_CHIP_ID) {
      return false;
    }

    // ソフトリセット（リセット中は応答しないので書き込み結果は見ない）
    writeRegister(QMP6988_ADDR, QMP_RESET_REG, 0xE6);
    delay(10);
    bool restarted = false;
    for (uint8_t i = 0; i < 100 && !restarted; i++) {
      restarted = readRegisters(QMP6988_ADDR, QMP_CHIP_ID_REG, &id, 1) && id == QMP_CHIP_ID;
      if (!restarted) {
        delay(1);
      }
    }
    if (!restarted || !readCalibration()) {
      return false;
    }

    uint8_t ioSetup = 0;
    if (!readRegisters(QMP6988_ADDR, QMP_IO_SETUP_REG, &ioSetup, 1)) {
      return false;
    }
    // 連続測定: 気圧16倍・温度2倍のオーバーサンプリング、IIRフィルター係数8、測定間の待機50ms
    if (!writeRegister(QMP6988_ADDR, QMP_CONFIG_REG, 0x03) ||
        !writeRegister(QMP6988_ADDR, QMP_IO_SETUP_REG, static_cast<uint8_t>((ioSetup & 0x1F) | (0x02 << 5))) ||
        !writeRegister(QMP6988_ADDR, QMP_CTRL_MEAS_REG, static_cast<uint8_t>((0x02 << 5) | (0x05 << 2) | 0x03))) {
      return false;
    }
    // 起動直後の1回目の読み出しに間に合うよう、最初の測定を待っておく
    delay(60);
    return true;
  }

  bool readCalibration() {
    uint8_t d[25];
    if (!readRegisters(QMP6988_ADDR, QMP_CALIBRATION_REG, d, sizeof(d))) {
      return false;
    }
    auto s16 = [&](uint8_t i) { return static_cast<int16_t>((uint16_t)d[i] << 8 | d[i + 1]); };
    // 20bitの符号付き値（上位16bit + 下位4bit）
    auto s20 = [](uint32_t v) { return static_cast<int32_t>(v << 12) >> 12; };

    cal_.a0 = s20((uint32_t)((uint16_t)d[18] << 8 | d[19]) << 4 | (d[24] & 0x0F));
    cal_.b00 = s20((uint32_t)((uint16_t)d[0] << 8 | d[1]) << 4 | (d[24] >> 4));
    cal_.a1 = 3608L * (int32_t)s16(20) - 1731677965L;              // 31Q23
    cal_.a2 = 16889L * (int32_t)s16(22) - 87619360L;               // 30Q47
    cal_.bt1 = 2982LL * (int64_t)s16(2) + 107370906LL;             // 28Q15
    cal_.bt2 = 329854LL * (int64_t)s16(4) + 108083093LL;           // 34Q38
    cal_.bp1 = 19923LL * (int64_t)s16(6) + 1133836764LL;           // 31Q20
    cal_.b11 = 2406LL * (int64_t)s16(8) + 118215883LL;             // 28Q34
    cal_.bp2 = 3079LL * (int64_t)s16(10) - 181579595LL;            // 29Q43
    cal_.b12 = 6846LL * (int64_t)s16(12) + 85590281LL;             // 29Q53
    cal_.b21 = 13836LL * (int64_t)s16(14) + 79333336LL;            // 29Q60
    cal_.bp3 = 2915LL * (int64_t)s16(16) + 157155561LL;            // 28Q65
    return true;
  }

  // 温度の生値から温度[1/256 °C]を求める
  int16_t convertTemperature256(int32_t dt) const {
    int64_t wk1 = (int64_t)cal_.a1 * dt;
    int64_t wk2 = ((int64_t)cal_.a2 * dt) >> 14;
    wk2 = (wk2 * dt) >> 10;
    wk2 = ((wk1 + wk2) / 32767) >> 19;
    return static_cast<int16_t>((cal_.a0 + wk2) >> 4);
  }

  // 気圧の生値と温度[1/256 °C]から気圧[1/16 Pa]を求める
  int32_t convertPressure16(int32_t dp, int16_t tx) const {
    int64_t wk1 = cal_.bt1 * tx;
    int64_t wk2 = (cal_.bp1 * dp) >> 5;
    wk1 += wk2;
    wk2 = (cal_.bt2 * tx) >> 1;
    wk2 = (wk2 * tx) >> 8;
    int64_t wk3 = wk2;
    wk2 = (cal_.b11 * tx) >> 4;
    wk2 = (wk2 * dp) >> 1;
    wk3 += wk2;
    wk2 = (cal_.bp2 * dp) >> 13;
    wk2 = (wk2 * dp) >> 1;
    wk3 += wk2;
    wk1 += wk3 >> 14;
    wk2 = cal_.b12 * tx;
    wk2 = (wk2 * tx) >> 22;
    wk2 = (wk2 * dp) >> 1;
    wk3 = wk2;
    wk2 = (cal_.b21 * tx) >> 6;
    wk2 = (wk2 * dp) >> 23;
    wk2 = (wk2 * dp) >> 1;
    wk3 += wk2;
    wk2 = (cal_.bp3 * dp) >> 12;
    wk2 = (wk2 * dp) >> 23;
    wk2 = wk2 * dp;
    wk3 += wk2;
    wk1 += wk3 >> 15;
    wk1 /= 32767L;
    wk1 >>= 11;
    wk1 += cal_.b00;
    return static_cast<int32_t>(wk1);
  }

  bool readRegisters(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len) {
    wire_->beginTransmission(addr);
    wire_->write(reg);
    if (wire_->endTransmission(false) != 0) {
      return false;
    }
    if (wire_->requestFrom(static_cast<int>(addr), static_cast<int>(len)) != len) {
      return false;
    }
    for (uint8_t i = 0; i < len; i++) {
      buf[i] = static_cast<uint8_t>(wire_->read());
    }
    return true;
  }

  bool writeRegister(uint8_t addr, uint8_t reg, uint8_t value) {
    wire_->beginTransmission(addr);
    wire_->write(reg);
    wire_->write(value);
    return wire_->endTransmission() == 0;
  }

  // SHT30のCRC-8（多項式0x31、初期値0xFF）。2byteのデータに対して計算する
  static uint8_t crc8(const uint8_t* data) {
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < 2; i++) {
      crc ^= data[i];
      for (uint8_t b = 0; b < 8; b++) {
        crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x31) : static_cast<uint8_t>(crc << 1);
      }
    }
    return crc;
  }
};
