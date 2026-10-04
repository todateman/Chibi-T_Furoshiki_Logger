// StrategyAssist.h のテスト（PC上で実行: pio test -e native）
#include <unity.h>

#include "StrategyAssist.h"

using StrategyAssist::Assist;
using StrategyAssist::Config;
using StrategyAssist::Cue;
using StrategyAssist::Input;
using StrategyAssist::Output;

namespace {

// テスト用の設定（sd/motegi_strategy.json と同じ値）
Config motegiConfig() {
  Config c;
  c.enabled = true;
  c.lapM = 2341.38f;
  c.totalLaps = 7;
  c.guardKmh = 16.0f;
  c.fuelWarnMpa = 0.30f;
  c.primary.vOffKmh = 32.5f;
  c.primary.markCount = 3;
  c.primary.marksM[0] = 400.0f;
  c.primary.marksM[1] = 800.0f;
  c.primary.marksM[2] = 1400.0f;
  c.primary.finalLapMarks = 2;
  c.fallback.vOffKmh = 30.0f;
  c.fallback.markCount = 4;
  c.fallback.marksM[0] = 200.0f;
  c.fallback.marksM[1] = 600.0f;
  c.fallback.marksM[2] = 1000.0f;
  c.fallback.marksM[3] = 1400.0f;
  c.fallback.finalLapMarks = 3;
  c.splitCount = 6;
  const float splits[6] = {309.0f, 630.0f, 951.0f, 1272.0f, 1593.0f, 1914.0f};
  for (uint8_t i = 0; i < 6; i++) {
    c.splitsS[i] = splits[i];
  }
  c.finalExtraIfSplitOverS = 1965.0f;
  return c;
}

// 0.1秒刻みで入力を進めるための補助
struct Runner {
  Assist assist;
  Input in;

  explicit Runner(const Config& cfg = motegiConfig()) {
    assist.configure(cfg);
    in.nowMs = 1000;
    in.worktimeS = 10.0f;
    in.speedKmh = 20.0f;
    in.lapPosM = 100.0f;
    in.fuelMpa = 0.32f;
  }

  const Output& step(float seconds = 0.1f) {
    const int steps = static_cast<int>(seconds * 10.0f + 0.5f);
    for (int i = 0; i < steps; i++) {
      in.nowMs += 100;
      in.worktimeS += 0.1f;
      assist.update(in);
    }
    return assist.output();
  }

  const Output& at(float posM, float speedKmh) {
    in.lapPosM = posM;
    in.speedKmh = speedKmh;
    return step();
  }

  // 加速して加速停止速度の手前でエンジンを止めるまでを進める
  void burn(float seconds = 2.0f) {
    in.engineOn = true;
    step(seconds);
    in.engineOn = false;
    step(0.5f);
  }
};

void test_pace_bump_thresholds() {
  TEST_ASSERT_EQUAL_FLOAT(0.0f, StrategyAssist::paceBumpKmh(0.0f));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, StrategyAssist::paceBumpKmh(8.0f));
  TEST_ASSERT_EQUAL_FLOAT(1.0f, StrategyAssist::paceBumpKmh(9.0f));
  TEST_ASSERT_EQUAL_FLOAT(1.0f, StrategyAssist::paceBumpKmh(20.0f));
  TEST_ASSERT_EQUAL_FLOAT(2.0f, StrategyAssist::paceBumpKmh(21.0f));
  TEST_ASSERT_EQUAL_FLOAT(2.0f, StrategyAssist::paceBumpKmh(35.0f));
  TEST_ASSERT_EQUAL_FLOAT(3.0f, StrategyAssist::paceBumpKmh(36.0f));
  TEST_ASSERT_EQUAL_FLOAT(0.0f, StrategyAssist::paceBumpKmh(-30.0f));
  TEST_ASSERT_EQUAL_FLOAT(-0.5f, StrategyAssist::paceBumpKmh(-31.0f));
}

void test_lap_crossing_plausible() {
  TEST_ASSERT_FALSE(StrategyAssist::lapCrossingPlausible(5.0f, 2341.38f));     // 発進直後の通過
  TEST_ASSERT_FALSE(StrategyAssist::lapCrossingPlausible(1100.0f, 2341.38f));
  TEST_ASSERT_TRUE(StrategyAssist::lapCrossingPlausible(2300.0f, 2341.38f));   // 距離計が短めに出ても数える
}

void test_off_and_idle() {
  Assist a;
  Input in;
  TEST_ASSERT_TRUE(a.update(in).cue == Cue::Off);

  a.configure(motegiConfig());
  in.lapPosM = 100.0f;
  in.speedKmh = 0.0f;
  in.worktimeS = 0.0f;
  TEST_ASSERT_TRUE(a.update(in).cue == Cue::Idle);
  TEST_ASSERT_EQUAL_FLOAT(32.5f, a.output().vOffKmh);
}

void test_mark_sequence() {
  Runner r;
  const Output& o = r.assist.output();

  r.at(250.0f, 20.0f);
  TEST_ASSERT_TRUE(o.cue == Cue::Coast);
  TEST_ASSERT_EQUAL_INT(0, o.nextMark);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 150.0f, o.nextMarkDistM);

  r.at(320.0f, 19.0f);
  TEST_ASSERT_TRUE(o.cue == Cue::Approach);

  r.at(400.0f, 18.0f);
  TEST_ASSERT_TRUE(o.cue == Cue::BurnNow);
  TEST_ASSERT_FALSE(o.guard);

  r.in.engineOn = true;
  r.at(405.0f, 19.0f);
  TEST_ASSERT_TRUE(o.cue == Cue::Burning);
  r.step(2.0f);

  r.at(435.0f, 32.5f);
  TEST_ASSERT_TRUE(o.cue == Cue::CutNow);

  r.in.engineOn = false;
  r.step(0.5f);
  TEST_ASSERT_TRUE(o.cue == Cue::Coast);
  TEST_ASSERT_EQUAL_INT(1, o.nextMark);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 365.0f, o.nextMarkDistM);
}

void test_failed_restart_keeps_cue() {
  Runner r;
  const Output& o = r.assist.output();
  r.at(400.0f, 18.0f);
  TEST_ASSERT_TRUE(o.cue == Cue::BurnNow);

  // 0.7秒クランキングして初爆せず
  r.in.engineOn = true;
  r.step(0.7f);
  TEST_ASSERT_TRUE(o.cue == Cue::Burning);
  r.in.engineOn = false;
  r.step(0.5f);
  TEST_ASSERT_TRUE(o.cue == Cue::BurnNow);
}

void test_skip_mark_when_fast() {
  Runner r;
  const Output& o = r.assist.output();
  r.at(390.0f, 31.0f);
  r.at(400.0f, 31.0f);  // 加速停止速度まで 2 km/h 未満
  TEST_ASSERT_TRUE(o.cue == Cue::Coast);
  TEST_ASSERT_EQUAL_INT(1, o.nextMark);
}

void test_missed_mark_expires() {
  Runner r;
  const Output& o = r.assist.output();
  r.at(400.0f, 20.0f);
  r.at(590.0f, 19.0f);
  TEST_ASSERT_TRUE(o.cue == Cue::BurnNow);
  r.at(600.0f, 19.0f);
  TEST_ASSERT_TRUE(o.cue == Cue::Coast);
}

void test_guard_speed() {
  Runner r;
  const Output& o = r.assist.output();
  r.at(1000.0f, 15.5f);
  TEST_ASSERT_TRUE(o.cue == Cue::BurnNow);
  TEST_ASSERT_TRUE(o.guard);

  r.at(1000.0f, 2.0f);  // 停車中は出さない
  TEST_ASSERT_FALSE(o.cue == Cue::BurnNow);
}

void test_no_position() {
  Runner r;
  const Output& o = r.assist.output();
  r.at(NAN, 25.0f);
  TEST_ASSERT_TRUE(o.cue == Cue::NoPosition);
  r.at(NAN, 15.0f);
  TEST_ASSERT_TRUE(o.cue == Cue::BurnNow);
  TEST_ASSERT_TRUE(o.guard);

  // 位置が戻ったとき、過ぎた目印では合図を出さない
  r.at(450.0f, 25.0f);
  TEST_ASSERT_TRUE(o.cue == Cue::Coast);
  TEST_ASSERT_EQUAL_INT(1, o.nextMark);
}

void test_pace_correction() {
  Runner r;
  const Output& o = r.assist.output();

  r.assist.onLapCrossed(1, 309.0f);
  r.at(5.0f, 27.0f);
  TEST_ASSERT_EQUAL_FLOAT(32.5f, o.vOffKmh);
  TEST_ASSERT_TRUE(o.hasSplit);
  TEST_ASSERT_EQUAL_UINT8(1, o.splitLap);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, o.delayS);

  const float delays[] = {9.0f, 21.0f, 36.0f, -31.0f};
  const float expected[] = {33.5f, 34.5f, 35.5f, 32.0f};
  for (int i = 0; i < 4; i++) {
    r.assist.onLapCrossed(2, 630.0f + delays[i]);
    r.at(5.0f, 27.0f);
    TEST_ASSERT_EQUAL_FLOAT(expected[i], o.vOffKmh);
  }

  // 補正は毎周やり直す
  r.assist.onLapCrossed(3, 951.0f);
  r.at(5.0f, 27.0f);
  TEST_ASSERT_EQUAL_FLOAT(32.5f, o.vOffKmh);
}

void test_implausible_split_is_ignored() {
  Runner r;
  const Output& o = r.assist.output();
  // 走行時間が途中で0に戻ると、目標より大幅に早い通過タイムになる
  r.assist.onLapCrossed(3, 200.0f);
  r.at(5.0f, 27.0f);
  TEST_ASSERT_EQUAL_FLOAT(32.5f, o.vOffKmh);
  TEST_ASSERT_TRUE(isnan(o.delayS));
}

void test_final_lap_coasts_home() {
  Runner r;
  const Output& o = r.assist.output();
  r.in.lapsDone = 6;
  r.assist.onLapCrossed(6, 1964.0f);
  r.at(5.0f, 27.0f);
  TEST_ASSERT_EQUAL_UINT8(2, o.activeMarks);

  r.at(400.0f, 18.0f);
  TEST_ASSERT_TRUE(o.cue == Cue::BurnNow);
  r.burn();
  r.at(800.0f, 18.0f);
  TEST_ASSERT_TRUE(o.cue == Cue::BurnNow);
  r.burn();

  r.at(1000.0f, 28.0f);
  TEST_ASSERT_TRUE(o.cue == Cue::CoastHome);
  r.at(1400.0f, 22.0f);
  TEST_ASSERT_TRUE(o.cue == Cue::CoastHome);

  r.in.lapsDone = 7;
  r.assist.onLapCrossed(7, 2290.0f);
  r.at(5.0f, 20.0f);
  TEST_ASSERT_TRUE(o.cue == Cue::Finished);
}

void test_final_lap_extra_burn_when_late() {
  Runner r;
  const Output& o = r.assist.output();
  r.in.lapsDone = 6;
  r.assist.onLapCrossed(6, 1966.0f);
  r.at(5.0f, 27.0f);
  TEST_ASSERT_EQUAL_UINT8(3, o.activeMarks);

  r.at(400.0f, 18.0f);
  r.burn();
  r.at(800.0f, 18.0f);
  r.burn();
  r.at(1400.0f, 20.0f);
  TEST_ASSERT_TRUE(o.cue == Cue::BurnNow);
}

void test_launch_from_grid_before_line() {
  Runner r;
  const Output& o = r.assist.output();

  // 発進前はグリッド（ラインの10 m手前）で待機
  r.in.worktimeS = -0.1f;
  r.in.lapPosM = 2331.0f;
  r.in.speedKmh = 0.0f;
  r.step();
  TEST_ASSERT_TRUE(o.cue == Cue::Idle);

  // 発進。ラインを越えても周回としては数えない（onLapCrossed は呼ばれない）
  r.in.engineOn = true;
  r.at(2335.0f, 5.0f);
  TEST_ASSERT_TRUE(o.cue == Cue::Burning);
  r.at(10.0f, 20.0f);
  r.step(3.0f);
  r.in.engineOn = false;
  r.at(60.0f, 32.5f);
  r.step(0.5f);
  TEST_ASSERT_EQUAL_INT(0, o.nextMark);

  r.at(400.0f, 25.0f);
  TEST_ASSERT_TRUE(o.cue == Cue::BurnNow);
}

void test_marks_restart_without_lap_count() {
  Runner r;
  const Output& o = r.assist.output();
  r.at(400.0f, 18.0f);
  r.burn();
  r.at(800.0f, 18.0f);
  r.burn();
  r.at(1400.0f, 20.0f);
  r.burn();

  // 周回カウントを取りこぼしても、次の周の目印は出る
  r.at(2330.0f, 27.0f);
  r.at(8.0f, 27.0f);
  r.at(400.0f, 18.0f);
  TEST_ASSERT_TRUE(o.cue == Cue::BurnNow);
}

void test_lap_count_before_position_wraps() {
  Runner r;
  const Output& o = r.assist.output();
  r.at(2335.0f, 27.0f);

  // カウントの時点で位置がまだ終端を指していても、誤って「加速」を出さない
  r.in.lapsDone = 1;
  r.assist.onLapCrossed(1, 309.0f);
  r.at(2338.0f, 27.0f);
  TEST_ASSERT_TRUE(o.cue == Cue::Coast);
  TEST_ASSERT_EQUAL_INT(0, o.nextMark);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 400.0f, o.nextMarkDistM);

  r.at(5.0f, 27.0f);
  TEST_ASSERT_TRUE(o.cue == Cue::Coast);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 395.0f, o.nextMarkDistM);
}

void test_fallback_pattern() {
  Runner r;
  const Output& o = r.assist.output();
  r.at(700.0f, 25.0f);
  r.assist.setFallback(true);
  r.at(705.0f, 25.0f);
  TEST_ASSERT_TRUE(o.usingFallback);
  TEST_ASSERT_EQUAL_FLOAT(30.0f, o.vOffKmh);
  TEST_ASSERT_TRUE(o.cue == Cue::Coast);  // 200 m・600 m では合図を出さない
  TEST_ASSERT_EQUAL_INT(2, o.nextMark);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 295.0f, o.nextMarkDistM);

  r.at(1000.0f, 20.0f);
  TEST_ASSERT_TRUE(o.cue == Cue::BurnNow);
}

void test_fuel_pressure_warning() {
  Runner r;
  const Output& o = r.assist.output();
  r.at(400.0f, 18.0f);

  // 未受信(0.0)は警告しない
  r.in.fuelMpa = 0.0f;
  r.in.engineOn = true;
  r.step(2.0f);
  TEST_ASSERT_FALSE(o.fuelLow);
  r.in.engineOn = false;
  r.step(0.5f);

  r.in.fuelMpa = 0.25f;
  r.in.engineOn = true;
  r.step(0.5f);
  TEST_ASSERT_FALSE(o.fuelLow);  // 加速開始直後は判定しない
  r.step(1.0f);
  TEST_ASSERT_TRUE(o.fuelLow);

  // 惰行中も表示を残し、次の加速で判定し直す
  r.in.engineOn = false;
  r.in.fuelMpa = 0.32f;
  r.step(1.0f);
  TEST_ASSERT_TRUE(o.fuelLow);
  r.in.engineOn = true;
  r.step(2.0f);
  TEST_ASSERT_FALSE(o.fuelLow);
}

void test_optional_features_off_when_not_configured() {
  // 最低限界速度・燃圧のしきい値・目標通過タイムを設定しなければ、それぞれの機能は働かない
  Config cfg;
  cfg.enabled = true;
  cfg.lapM = 2341.38f;
  cfg.totalLaps = 7;
  cfg.primary.vOffKmh = 32.5f;
  cfg.primary.markCount = 1;
  cfg.primary.marksM[0] = 400.0f;
  cfg.primary.finalLapMarks = 1;
  Runner r(cfg);
  const Output& o = r.assist.output();

  r.at(1000.0f, 10.0f);
  TEST_ASSERT_FALSE(o.cue == Cue::BurnNow);

  r.in.fuelMpa = 0.10f;
  r.in.engineOn = true;
  r.step(2.0f);
  TEST_ASSERT_FALSE(o.fuelLow);
  r.in.engineOn = false;
  r.step(0.5f);

  r.assist.onLapCrossed(1, 400.0f);
  r.at(5.0f, 27.0f);
  TEST_ASSERT_EQUAL_FLOAT(32.5f, o.vOffKmh);
  TEST_ASSERT_TRUE(isnan(o.delayS));
  TEST_ASSERT_FALSE(r.assist.hasFallback());
}

void test_reset_on_new_run() {
  Runner r;
  const Output& o = r.assist.output();
  r.assist.onLapCrossed(1, 345.0f);
  r.at(5.0f, 27.0f);
  TEST_ASSERT_EQUAL_FLOAT(35.5f, o.vOffKmh);

  // ECUがリセットされて走行時間が0に戻ったら、前回走行の補正を持ち越さない
  r.in.worktimeS = -0.1f;
  r.step();
  TEST_ASSERT_TRUE(o.cue == Cue::Idle);
  TEST_ASSERT_EQUAL_FLOAT(32.5f, o.vOffKmh);
  TEST_ASSERT_FALSE(o.hasSplit);
}

}  // namespace

void setUp() {}
void tearDown() {}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_pace_bump_thresholds);
  RUN_TEST(test_lap_crossing_plausible);
  RUN_TEST(test_off_and_idle);
  RUN_TEST(test_mark_sequence);
  RUN_TEST(test_failed_restart_keeps_cue);
  RUN_TEST(test_skip_mark_when_fast);
  RUN_TEST(test_missed_mark_expires);
  RUN_TEST(test_guard_speed);
  RUN_TEST(test_no_position);
  RUN_TEST(test_pace_correction);
  RUN_TEST(test_implausible_split_is_ignored);
  RUN_TEST(test_final_lap_coasts_home);
  RUN_TEST(test_final_lap_extra_burn_when_late);
  RUN_TEST(test_launch_from_grid_before_line);
  RUN_TEST(test_marks_restart_without_lap_count);
  RUN_TEST(test_lap_count_before_position_wraps);
  RUN_TEST(test_fallback_pattern);
  RUN_TEST(test_fuel_pressure_warning);
  RUN_TEST(test_optional_features_off_when_not_configured);
  RUN_TEST(test_reset_on_new_run);
  return UNITY_END();
}
