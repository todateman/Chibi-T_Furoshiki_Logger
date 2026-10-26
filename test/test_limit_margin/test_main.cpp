// LimitMargin.h のテスト（PC上で実行: pio test -e native）
#include <unity.h>

#include "LimitMargin.h"

using LimitMargin::Band;
using LimitMargin::Config;
using LimitMargin::Output;
using LimitMargin::Tracker;

namespace {

constexpr float LAP_M = 2341.38f;
constexpr float RACE_M = 16112.28f;  // 6周 + 2224 - 160
constexpr float LIMIT_S = 2360.11f;
constexpr float PLAN_GOAL_S = 2283.0f;

// テスト用の設定（もてぎ: 発進 160 m, コントロールライン 2224 m）
// 計画は一定の速さで 2283 秒にゴールする表
Config motegiConfig() {
  Config c;
  c.enabled = true;
  c.lapM = LAP_M;
  c.startM = 160.0f;
  c.controlM = 2224.0f;
  c.raceM = RACE_M;
  c.limitS = LIMIT_S;
  c.planStepM = 100.0f;
  c.planCount = 0;
  for (float d = 0.0f; d < RACE_M; d += 100.0f) {
    c.planS[c.planCount++] = d * PLAN_GOAL_S / RACE_M;
  }
  c.planS[c.planCount++] = PLAN_GOAL_S;
  return c;
}

// 道のり d の周内位置
float lapPosOf(const Config& c, float d) {
  return LimitMargin::wrapM(c.startM + d, c.lapM);
}

void test_crossing_distance() {
  const Config c = motegiConfig();
  TEST_ASSERT_EQUAL_FLOAT(2064.0f, LimitMargin::firstCrossingM(c));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, LimitMargin::crossingM(c, 0));
  TEST_ASSERT_EQUAL_FLOAT(2064.0f, LimitMargin::crossingM(c, 1));
  TEST_ASSERT_FLOAT_WITHIN(0.05f, RACE_M, LimitMargin::crossingM(c, 7));  // ゴールはコントロールライン上

  // ライン上から発進するコースは、最初の通過が1周後
  Config onLine = c;
  onLine.startM = 0.0f;
  onLine.controlM = 0.0f;
  TEST_ASSERT_EQUAL_FLOAT(LAP_M, LimitMargin::firstCrossingM(onLine));
}

void test_planned_time() {
  const Config c = motegiConfig();
  TEST_ASSERT_EQUAL_UINT16(163, c.planCount);
  TEST_ASSERT_TRUE(LimitMargin::isValidConfig(c));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, LimitMargin::plannedTimeAt(c, -10.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f * PLAN_GOAL_S / RACE_M, LimitMargin::plannedTimeAt(c, 50.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 8000.0f * PLAN_GOAL_S / RACE_M, LimitMargin::plannedTimeAt(c, 8000.0f));
  // 最後の区間（16100 m → ゴール）
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 16106.0f * PLAN_GOAL_S / RACE_M, LimitMargin::plannedTimeAt(c, 16106.0f));
  TEST_ASSERT_EQUAL_FLOAT(PLAN_GOAL_S, LimitMargin::plannedTimeAt(c, RACE_M));
  TEST_ASSERT_EQUAL_FLOAT(PLAN_GOAL_S, LimitMargin::plannedTimeAt(c, RACE_M + 100.0f));

  Config noPlan = c;
  noPlan.planCount = 0;
  TEST_ASSERT_TRUE(isnan(LimitMargin::plannedTimeAt(noPlan, 100.0f)));
  TEST_ASSERT_TRUE(LimitMargin::isValidConfig(noPlan));
}

void test_invalid_config() {
  Config c = motegiConfig();
  c.planS[10] = c.planS[9];  // 昇順でない
  TEST_ASSERT_FALSE(LimitMargin::isValidConfig(c));

  c = motegiConfig();
  c.planCount -= 2;  // 表がゴールまで届かない
  TEST_ASSERT_FALSE(LimitMargin::isValidConfig(c));

  c = motegiConfig();
  c.limitS = 0.0f;
  TEST_ASSERT_FALSE(LimitMargin::isValidConfig(c));

  c = motegiConfig();
  c.startM = LAP_M;
  TEST_ASSERT_FALSE(LimitMargin::isValidConfig(c));

  c = motegiConfig();
  c.bandLoS = 40.0f;
  TEST_ASSERT_FALSE(LimitMargin::isValidConfig(c));
}

void test_band() {
  const Config c = motegiConfig();
  TEST_ASSERT_TRUE(LimitMargin::bandOf(c, -0.5f) == Band::Negative);
  TEST_ASSERT_TRUE(LimitMargin::bandOf(c, 0.0f) == Band::Under);
  TEST_ASSERT_TRUE(LimitMargin::bandOf(c, 9.9f) == Band::Under);
  TEST_ASSERT_TRUE(LimitMargin::bandOf(c, 10.0f) == Band::In);
  TEST_ASSERT_TRUE(LimitMargin::bandOf(c, 30.0f) == Band::In);
  TEST_ASSERT_TRUE(LimitMargin::bandOf(c, 30.1f) == Band::Over);
}

void test_idle_and_disabled() {
  Tracker t;
  t.configure(Config());
  TEST_ASSERT_FALSE(t.update(10.0f, 0, 500.0f).valid);
  TEST_ASSERT_TRUE(isnan(t.virtualLapPosM(10.0f)));

  t.configure(motegiConfig());
  TEST_ASSERT_FALSE(t.update(0.0f, 0, 160.0f).valid);  // 発進前
  TEST_ASSERT_FALSE(t.update(1.0f, 0, NAN).valid);     // 位置がまだ一度も取れていない
  const Output& o = t.update(2.0f, 0, 170.0f);
  TEST_ASSERT_TRUE(o.valid);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, o.distM);
  // 位置が取れなくなっても前回の道のりを保つ（時間が進むぶん余裕は減る）
  const float before = o.marginS;
  TEST_ASSERT_TRUE(t.update(3.0f, 0, NAN).valid);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, t.output().distM);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, before - 1.0f, t.output().marginS);
}

// 仮想車と同じ進みなら余裕は0、計画どおりなら予測は常に「規定 - 計画のゴール時間」
void test_margin_and_prediction() {
  const Config c = motegiConfig();
  Tracker t;
  t.configure(c);
  uint8_t laps = 0;
  for (float d = 0.0f; d < RACE_M - 50.0f; d += 37.0f) {
    if (d >= LimitMargin::crossingM(c, laps + 1)) {
      laps++;
      t.onLapCrossed(laps, d * LIMIT_S / RACE_M);
    }
    const float worktime = (d > 0.0f) ? d * LIMIT_S / RACE_M : 0.01f;
    const Output& o = t.update(worktime, laps, lapPosOf(c, d));
    TEST_ASSERT_TRUE(o.valid);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, d, o.distM);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, o.marginS);
  }
  TEST_ASSERT_EQUAL_UINT8(6, laps);

  t.configure(c);
  laps = 0;
  for (float d = 0.0f; d < RACE_M - 50.0f; d += 37.0f) {
    if (d >= LimitMargin::crossingM(c, laps + 1)) {
      laps++;
      t.onLapCrossed(laps, LimitMargin::crossingM(c, laps) * PLAN_GOAL_S / RACE_M);
    }
    const float worktime = (d > 0.0f) ? d * PLAN_GOAL_S / RACE_M : 0.01f;
    const Output& o = t.update(worktime, laps, lapPosOf(c, d));
    TEST_ASSERT_FLOAT_WITHIN(0.1f, LIMIT_S - PLAN_GOAL_S, o.predictedS);
  }
  // 周回通過時の余裕（1周目の通過: 2064 m を計画どおり）
  TEST_ASSERT_TRUE(t.output().hasLapMargin);
  TEST_ASSERT_EQUAL_UINT8(6, t.output().lap);
  const float d6 = LimitMargin::crossingM(c, 6);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, d6 * (LIMIT_S - PLAN_GOAL_S) / RACE_M, t.output().lapMarginS);
}

// 周回カウントの直後に位置がまだラインの手前を指す
void test_position_lags_lap_count() {
  const Config c = motegiConfig();
  Tracker t;
  t.configure(c);
  t.update(100.0f, 0, 2200.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 2040.0f, t.output().distM);
  t.onLapCrossed(1, 101.0f);
  t.update(101.0f, 1, 2220.0f);  // ラインの4 m手前
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 2060.0f, t.output().distM);
  t.update(102.0f, 1, 2230.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 2070.0f, t.output().distM);
}

// 周回カウントより先に位置がラインを越える
void test_position_leads_lap_count() {
  const Config c = motegiConfig();
  Tracker t;
  t.configure(c);
  t.update(100.0f, 0, 2200.0f);
  t.update(101.0f, 0, 2230.0f);  // まだ数えていないがラインの6 m先
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 2070.0f, t.output().distM);
  t.onLapCrossed(1, 101.5f);
  t.update(102.0f, 1, 2240.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 2080.0f, t.output().distM);
}

// 周回カウントを取りこぼしても道のりは連続する
void test_missed_lap_count() {
  const Config c = motegiConfig();
  Tracker t;
  t.configure(c);
  const float second = LimitMargin::crossingM(c, 2);  // 2回目の通過
  for (float d = 0.0f; d < second; d += 20.0f) {
    t.update(1.0f + d / 7.0f, 0, lapPosOf(c, d));
  }
  TEST_ASSERT_FLOAT_WITHIN(20.0f, second, t.output().distM);
  // 1回目を取りこぼし、2回目の通過が「1周目」として数えられた
  const float splitS = 1.0f + second / 7.0f;
  t.onLapCrossed(1, splitS);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, LimitMargin::marginAt(c, second, splitS), t.output().lapMarginS);
  t.update(splitS + 1.0f, 1, lapPosOf(c, second + 7.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.5f, second + 7.0f, t.output().distM);
}

// 発進位置の少し手前（グリッド）から動き出す
void test_launch_behind_start() {
  const Config c = motegiConfig();
  Tracker t;
  t.configure(c);
  t.update(0.5f, 0, 155.0f);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, t.output().distM);
  t.update(1.0f, 0, 165.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 5.0f, t.output().distM);
}

// ゴール後は、ゴール時の余裕で止める
void test_finish() {
  const Config c = motegiConfig();
  Tracker t;
  t.configure(c);
  t.update(2200.0f, 6, lapPosOf(c, RACE_M - 30.0f));
  TEST_ASSERT_FALSE(t.output().finished);
  t.onLapCrossed(7, 2300.0f);
  const Output& o = t.update(2301.0f, 7, 2230.0f);
  TEST_ASSERT_TRUE(o.finished);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, LIMIT_S - 2300.0f, o.marginS);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, LIMIT_S - 2300.0f, o.predictedS);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, LIMIT_S - 2300.0f, o.lapMarginS);
  t.update(2400.0f, 7, 2300.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, LIMIT_S - 2300.0f, t.output().marginS);

  // 次の走行（走行時間が0に戻る）では持ち越さない
  t.update(0.0f, 0, 160.0f);
  TEST_ASSERT_FALSE(t.output().valid);
  TEST_ASSERT_FALSE(t.output().hasLapMargin);
}

void test_virtual_car_position() {
  const Config c = motegiConfig();
  Tracker t;
  t.configure(c);
  TEST_ASSERT_EQUAL_FLOAT(160.0f, t.virtualLapPosM(0.0f));
  TEST_ASSERT_EQUAL_FLOAT(160.0f, t.virtualLapPosM(-5.0f));
  // 1周目の通過（2064 m）の時刻ではコントロールライン上
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 2224.0f, t.virtualLapPosM(2064.0f * LIMIT_S / RACE_M));
  // 規定時間を過ぎたらゴール（コントロールライン上）で止まる
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 2224.0f, t.virtualLapPosM(LIMIT_S));
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 2224.0f, t.virtualLapPosM(LIMIT_S + 100.0f));
}

}  // namespace

void setUp() {}
void tearDown() {}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_crossing_distance);
  RUN_TEST(test_planned_time);
  RUN_TEST(test_invalid_config);
  RUN_TEST(test_band);
  RUN_TEST(test_idle_and_disabled);
  RUN_TEST(test_margin_and_prediction);
  RUN_TEST(test_position_lags_lap_count);
  RUN_TEST(test_position_leads_lap_count);
  RUN_TEST(test_missed_lap_count);
  RUN_TEST(test_launch_behind_start);
  RUN_TEST(test_finish);
  RUN_TEST(test_virtual_car_position);
  return UNITY_END();
}
