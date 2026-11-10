// UbloxM10.h のテスト（PC上で実行: pio test -e native）
// 期待するバイト列は tools/setup_max_m10s.py で組み立てたもの（実機の MAX-M10S が ACK を返した設定と同じ）
#include <unity.h>

#include <string.h>

#include "UbloxM10.h"

namespace {

void assertBytes(const uint8_t* expected, size_t expectedLength, const uint8_t* actual, size_t actualLength) {
  TEST_ASSERT_EQUAL_UINT32(expectedLength, actualLength);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, actual, expectedLength);
}

void feedText(UbloxM10::NmeaCounter& counter, const char* text) {
  for (const char* p = text; *p; p++) {
    counter.feed(*p);
  }
}

}  // namespace

void setUp() {}
void tearDown() {}

void test_value_size() {
  TEST_ASSERT_EQUAL_UINT32(1, UbloxM10::valueSize(UbloxM10::KEY_SIGNAL_GPS));  // 1 ビット
  TEST_ASSERT_EQUAL_UINT32(1, UbloxM10::valueSize(UbloxM10::KEY_NMEA_RMC));    // 1 バイト
  TEST_ASSERT_EQUAL_UINT32(2, UbloxM10::valueSize(UbloxM10::KEY_RATE_MEAS));
  TEST_ASSERT_EQUAL_UINT32(4, UbloxM10::valueSize(UbloxM10::KEY_UART1_BAUD));
  TEST_ASSERT_EQUAL_UINT32(0, UbloxM10::valueSize(0x50000000));  // 8 バイトは扱わない
}

void test_logger_config_frame() {
  const uint8_t expected[] = {
      0xB5, 0x62, 0x06, 0x8A, 0x4C, 0x00, 0x00, 0x03, 0x00, 0x00, 0x01, 0x00, 0x21, 0x30, 0x64, 0x00, 0x02,
      0x00, 0x21, 0x30, 0x01, 0x00, 0xBB, 0x00, 0x91, 0x20, 0x00, 0xCA, 0x00, 0x91, 0x20, 0x00, 0xC0, 0x00,
      0x91, 0x20, 0x00, 0xC5, 0x00, 0x91, 0x20, 0x00, 0xAC, 0x00, 0x91, 0x20, 0x01, 0xB1, 0x00, 0x91, 0x20,
      0x00, 0x1F, 0x00, 0x31, 0x10, 0x01, 0x20, 0x00, 0x31, 0x10, 0x01, 0x21, 0x00, 0x31, 0x10, 0x01, 0x22,
      0x00, 0x31, 0x10, 0x00, 0x24, 0x00, 0x31, 0x10, 0x01, 0x25, 0x00, 0x31, 0x10, 0x00, 0xCC, 0xA9,
  };
  uint8_t frame[UbloxM10::MAX_FRAME_BYTES];
  assertBytes(expected, sizeof(expected), frame, UbloxM10::buildLoggerConfig(frame, sizeof(frame)));
}

void test_baudrate_frame() {
  const uint8_t expected[] = {0xB5, 0x62, 0x06, 0x8A, 0x0C, 0x00, 0x00, 0x03, 0x00, 0x00,
                              0x01, 0x00, 0x52, 0x40, 0x00, 0xC2, 0x01, 0x00, 0xF5, 0xBB};
  uint8_t frame[UbloxM10::MAX_FRAME_BYTES];
  assertBytes(expected, sizeof(expected), frame, UbloxM10::buildBaudrate(115200, frame, sizeof(frame)));
}

void test_hot_start_frame() {
  const uint8_t expected[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00, 0x00, 0x00, 0x02, 0x00, 0x10, 0x68};
  uint8_t frame[UbloxM10::MAX_FRAME_BYTES];
  assertBytes(expected, sizeof(expected), frame, UbloxM10::buildHotStart(frame, sizeof(frame)));
}

void test_frame_too_small() {
  uint8_t frame[40];
  TEST_ASSERT_EQUAL_UINT32(0, UbloxM10::buildLoggerConfig(frame, sizeof(frame)));
}

void test_nmea_counter_counts_valid_sentences() {
  UbloxM10::NmeaCounter counter;
  // MAX-M10S の出荷状態の出力（38400 bps）から
  feedText(counter,
           "$GNRMC,133013.00,A,3504.01675,N,13706.70168,E,0.060,,131126,,,D,V*1E\r\n"
           "$GNVTG,,T,,M,0.060,N,0.111,K,D*3F\r\n"
           "$GNGGA,,,,,,0,00,99.99,,,,,,*56\r\n");
  TEST_ASSERT_EQUAL_UINT16(3, counter.valid());
  TEST_ASSERT_EQUAL_UINT16(1, counter.rmc());
}

void test_nmea_counter_rejects_broken_lines() {
  UbloxM10::NmeaCounter counter;
  feedText(counter, "$GNRMC,,V,,,,,,,,,,N,V*38\r\n");  // チェックサムが違う（正しくは *37）
  feedText(counter, "GNVTG,,,,,,,,,N*2E\r\n");         // "$" がない
  feedText(counter, "$GNGGA,,,,,,0,00,99.99,,,,,,\r\n");  // チェックサムがない
  // 違うボーレートで受信したときのような、化けた文字
  const char garbage[] = {'\x11', 'U', '\xa5', '$', '\xc6', '%', '\x03', '\x9c', '*', '1', '2', '\r', '\n', 0};
  feedText(counter, garbage);
  TEST_ASSERT_EQUAL_UINT16(0, counter.valid());
  TEST_ASSERT_EQUAL_UINT16(0, counter.rmc());
}

void test_nmea_counter_resyncs_on_dollar() {
  UbloxM10::NmeaCounter counter;
  // 途中から受信し始めた行のあとに、正しい行が続く
  feedText(counter, "01675,N,13706.70168,E,0.060,,131126,,,D,V*1E\r\n$GNRMC,,V,,,,,,,,,,N,V*37\r\n");
  TEST_ASSERT_EQUAL_UINT16(1, counter.valid());
  TEST_ASSERT_EQUAL_UINT16(1, counter.rmc());
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_value_size);
  RUN_TEST(test_logger_config_frame);
  RUN_TEST(test_baudrate_frame);
  RUN_TEST(test_hot_start_frame);
  RUN_TEST(test_frame_too_small);
  RUN_TEST(test_nmea_counter_counts_valid_sentences);
  RUN_TEST(test_nmea_counter_rejects_broken_lines);
  RUN_TEST(test_nmea_counter_resyncs_on_dollar);
  return UNITY_END();
}
