// CourseMap.h のテスト（PC上で実行: pio test -e native）
#include <unity.h>

#include "CourseMap.h"

using CourseMap::Projection;

namespace {

// もてぎオーバルコースのウェイポイントの範囲（sd/motegi_waypoint.csv の最小・最大に近い値）
constexpr double MIN_LAT = 36.5290;
constexpr double MAX_LAT = 36.5369;
constexpr double MIN_LNG = 140.2236;
constexpr double MAX_LNG = 140.2314;

// コース図を収める矩形
constexpr int LEFT = 92;
constexpr int TOP = 22;
constexpr int WIDTH = 136;
constexpr int HEIGHT = 160;

Projection motegiProjection() {
  return CourseMap::fit(MIN_LAT, MAX_LAT, MIN_LNG, MAX_LNG, LEFT, TOP, WIDTH, HEIGHT);
}

}  // namespace

void setUp() {}
void tearDown() {}

// 北が上、東が右になる
void test_north_is_up_and_east_is_right() {
  const Projection p = motegiProjection();
  TEST_ASSERT_TRUE(p.valid);

  int xSW, ySW, xNE, yNE;
  p.toScreen(MIN_LAT, MIN_LNG, xSW, ySW);
  p.toScreen(MAX_LAT, MAX_LNG, xNE, yNE);
  TEST_ASSERT_TRUE(yNE < ySW);
  TEST_ASSERT_TRUE(xNE > xSW);
}

// 範囲は矩形に収まり、縦横のうち長い方が矩形の端まで届く（もてぎは南北に長い）
void test_bounds_fit_inside_rect() {
  const Projection p = motegiProjection();

  int xSW, ySW, xNE, yNE;
  p.toScreen(MIN_LAT, MIN_LNG, xSW, ySW);
  p.toScreen(MAX_LAT, MAX_LNG, xNE, yNE);
  TEST_ASSERT_EQUAL_INT(TOP, yNE);
  TEST_ASSERT_EQUAL_INT(TOP + HEIGHT - 1, ySW);
  TEST_ASSERT_TRUE(xSW > LEFT);
  TEST_ASSERT_TRUE(xNE < LEFT + WIDTH - 1);
  // 左右の余白は等しい（中央に置かれる）
  TEST_ASSERT_INT_WITHIN(1, xSW - LEFT, LEFT + WIDTH - 1 - xNE);
}

// 東西に長い範囲では、左右が矩形の端まで届く
void test_wide_bounds_fit_width() {
  const Projection p = CourseMap::fit(36.5320, 36.5330, 140.2200, 140.2300, LEFT, TOP, WIDTH, HEIGHT);
  TEST_ASSERT_TRUE(p.valid);

  int xSW, ySW, xNE, yNE;
  p.toScreen(36.5320, 140.2200, xSW, ySW);
  p.toScreen(36.5330, 140.2300, xNE, yNE);
  TEST_ASSERT_EQUAL_INT(LEFT, xSW);
  TEST_ASSERT_EQUAL_INT(LEFT + WIDTH - 1, xNE);
  TEST_ASSERT_TRUE(yNE > TOP);
  TEST_ASSERT_TRUE(ySW < TOP + HEIGHT - 1);
}

// 縦横の縮尺が等しい: 北へ100 mと東へ100 mは同じ画素数になる
void test_scale_is_uniform() {
  const Projection p = motegiProjection();
  const double dLat = 100.0 / CourseMap::METERS_PER_DEG;
  const double dLng = 100.0 / p.mPerDegLng;

  int x0, y0, xN, yN, xE, yE;
  p.toScreen(p.latC, p.lngC, x0, y0);
  p.toScreen(p.latC + dLat, p.lngC, xN, yN);
  p.toScreen(p.latC, p.lngC + dLng, xE, yE);
  TEST_ASSERT_EQUAL_INT(x0, xN);
  TEST_ASSERT_EQUAL_INT(y0, yE);
  TEST_ASSERT_INT_WITHIN(1, y0 - yN, xE - x0);
  TEST_ASSERT_INT_WITHIN(1, static_cast<int>(lroundf(100.0f * p.pxPerM)), xE - x0);
}

// 範囲の中心は矩形の中心に来る
void test_center_maps_to_rect_center() {
  const Projection p = motegiProjection();

  int x, y;
  p.toScreen((MIN_LAT + MAX_LAT) / 2.0, (MIN_LNG + MAX_LNG) / 2.0, x, y);
  TEST_ASSERT_INT_WITHIN(1, LEFT + WIDTH / 2, x);
  TEST_ASSERT_INT_WITHIN(1, TOP + HEIGHT / 2, y);
}

// 南北に一直線の範囲でも投影できる（東西の幅が0）
void test_straight_line_bounds() {
  const Projection p = CourseMap::fit(MIN_LAT, MAX_LAT, MIN_LNG, MIN_LNG, LEFT, TOP, WIDTH, HEIGHT);
  TEST_ASSERT_TRUE(p.valid);

  int xS, yS, xN, yN;
  p.toScreen(MIN_LAT, MIN_LNG, xS, yS);
  p.toScreen(MAX_LAT, MIN_LNG, xN, yN);
  TEST_ASSERT_EQUAL_INT(TOP + HEIGHT - 1, yS);
  TEST_ASSERT_EQUAL_INT(TOP, yN);
  TEST_ASSERT_EQUAL_INT(xS, xN);
}

// 範囲がつぶれている・矩形が小さすぎる場合は無効を返す
void test_degenerate_bounds_are_invalid() {
  TEST_ASSERT_FALSE(CourseMap::fit(MIN_LAT, MIN_LAT, MIN_LNG, MIN_LNG, LEFT, TOP, WIDTH, HEIGHT).valid);  // 点が1つ
  TEST_ASSERT_FALSE(CourseMap::fit(MAX_LAT, MIN_LAT, MAX_LNG, MIN_LNG, LEFT, TOP, WIDTH, HEIGHT).valid);  // 最小と最大が逆
  TEST_ASSERT_FALSE(CourseMap::fit(MIN_LAT, MAX_LAT, MIN_LNG, MAX_LNG, LEFT, TOP, 1, HEIGHT).valid);
  TEST_ASSERT_FALSE(CourseMap::fit(MIN_LAT, MAX_LAT, MIN_LNG, MAX_LNG, LEFT, TOP, WIDTH, 0).valid);
  TEST_ASSERT_FALSE(Projection().valid);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_north_is_up_and_east_is_right);
  RUN_TEST(test_bounds_fit_inside_rect);
  RUN_TEST(test_wide_bounds_fit_width);
  RUN_TEST(test_scale_is_uniform);
  RUN_TEST(test_center_maps_to_rect_center);
  RUN_TEST(test_straight_line_bounds);
  RUN_TEST(test_degenerate_bounds_are_invalid);
  return UNITY_END();
}
