// drawStrategyMode() / drawCourseMapMode() をPC上でスプライトに描いてPPMに出す（README の画面例用。render.sh から使う）
// 描画関数は render.sh が src/main.cpp から抜き出した extracted.inc を取り込む。
// 設定は sd/motegi_strategy.json を main.cpp の読み込み関数でそのまま読む。出力は カレントディレクトリの out/<場面>.ppm
#define STRATEGY_SIM 0
#include "common.h"

static std::string moPath;  // もてぎのウェイポイント
static uint32_t nowMs = 1000;
// 位置を置いて判定を1回進める
static void step(float lapPos, float kmh, bool engine, uint32_t dtMs = 200) {
  nowMs += dtMs;
  speed = kmh;
  INJ_timems = engine ? 3.0f : 0.0f;
  if (std::isfinite(lapPos)) {
    if (!lapPositionToLatLng(lapPos, la, ln)) {  // 設定ファイルが無い場所: 距離列から直接置く
      size_t k = 0;
      while (k + 1 < waypointCount && waypoints[k + 1].dist <= lapPos) k++;
      la = waypoints[k].lat + 0.00003; ln = waypoints[k].lng + 0.00003;  // 測位のずれを少し入れる
    }
    double best = 1e18;
    for (size_t i = 0; i < waypointCount; i++) {
      double d = pow(la - waypoints[i].lat, 2) + pow(ln - waypoints[i].lng, 2);
      if (d < best) { best = d; nearestWaypointIndex = (int)i; }
    }
    positionFresh = true;
  }
  if (!strategy.config().enabled) return;
  StrategyAssist::Input in;
  in.nowMs = nowMs; in.lapPosM = positionFresh ? currentLapPositionMeters() : NAN; in.speedKmh = kmh; in.engineOn = engine;
  in.worktimeS = worktime; in.lapsDone = Lapcount; in.fuelMpa = FuelPre;
  strategy.update(in);
  limitMargin.update(worktime, Lapcount, in.lapPosM);
}

static void save(const char* name, bool strategyScreen) {
  lcd_s.fillScreen(TFT_BLACK);
  lcd_s.setTextColor(TFT_WHITE);
  if (strategyScreen) drawStrategyMode(); else drawCourseMapMode();
  char path[256];
  snprintf(path, sizeof(path), "out/%s.ppm", name);
  FILE* f = fopen(path, "wb");
  fprintf(f, "P6\n320 240\n255\n");
  for (int y = 0; y < 240; y++) for (int x = 0; x < 320; x++) {
    uint16_t v = lcd_s.readPixel(x, y);  // RGB565
    uint8_t rgb[3] = {(uint8_t)(((v >> 11) & 31) * 255 / 31), (uint8_t)(((v >> 5) & 63) * 255 / 63), (uint8_t)((v & 31) * 255 / 31)};
    fwrite(rgb, 1, 3, f);
  }
  fclose(f);
  const LimitMargin::Output& m = limitMargin.output();
  printf("%-24s cue=%d vOff=%.1f worktime=%.0f dist=%.0f margin=%+.0f lap=%+.0f pred=%+.0f\n", name, (int)strategy.output().cue,
         strategy.output().vOffKmh, worktime, m.distM, m.marginS, m.lapMarginS, m.predictedS);
}

static void reset(const char* loc, const char* csv, bool withPlan) {
  Loc = loc;
  if (csv) loadWaypoints(csv); else waypointCount = 0;
  StrategyAssist::Config c = withPlan ? planCfg : StrategyAssist::Config();
  c.totalLaps = totallaps;
  strategy.configure(c);
  limitMargin.configure(withPlan ? marginCfg : LimitMargin::Config());
  strategySource = withPlan ? STRATEGY_SRC_SD : STRATEGY_SRC_NONE;
  Lapcount = 0; worktime = 0; distance = 0; FuelPre = 0.32f; positionFresh = false; nearestWaypointIndex = -1; la = ln = 0;
}

// 発進からの道のり（周回数 laps を数えたあと、周内位置 pos にいるとき）
static float raceDist(uint8_t laps, float pos) {
  if (laps == 0) return LimitMargin::wrapM(pos - marginCfg.startM, marginCfg.lapM);
  return LimitMargin::crossingM(marginCfg, laps) + LimitMargin::wrapM(pos - marginCfg.controlM, marginCfg.lapM);
}

// 走行中の場面を作る: laps 周を数えた直後の通過が計画より delayS 遅れ（+ が遅れ）、いまも同じだけ遅れて、
// 位置 pos まで速度 kmh で来たところ。場面の手前150 mだけ場面の速度・噴射で進め、それより前は目印を飛ばす速さで進める
static void scene(uint8_t laps, float delayS, float pos, float kmh, bool engine, bool fallback = false, float fuel = 0.32f) {
  reset("mo", moPath.c_str(), true);
  Lapcount = laps;
  const float lap = marginCfg.lapM;
  const float d = raceDist(laps, pos);
  worktime = LimitMargin::plannedTimeAt(marginCfg, d) + delayS;
  distance = (uint16_t)d;
  const float splitS = LimitMargin::plannedTimeAt(marginCfg, LimitMargin::crossingM(marginCfg, laps)) + delayS;
  // ライン通過の直後から始める
  step(LimitMargin::wrapM(marginCfg.controlM + 5.0f, lap), 40.0f, false);
  strategy.onLapCrossed(laps, splitS);
  limitMargin.onLapCrossed(laps, splitS);
  const float from = LimitMargin::wrapM(pos - 150.0f, lap);
  for (float p = LimitMargin::wrapM(marginCfg.controlM + 25.0f, lap); ; p = LimitMargin::wrapM(p + 20.0f, lap)) {
    step(p, 40.0f, false);
    if (LimitMargin::wrapM(from - p, lap) < 20.0f) break;
  }
  if (fallback) strategy.setFallback(true);
  FuelPre = fuel;
  for (int i = 0; i <= 150; i++) step(LimitMargin::wrapM(from + i, lap), kmh, engine);
}

int main(int argc, char** argv) {
  const std::string sd = argv[1];
  lcd_s.setColorDepth(8);
  lcd_s.createSprite(320, 240);
  const std::string mo = sd + "/motegi_waypoint.csv", su = sd + "/suzuka_waypoint.csv";
  loadPlan((sd + "/motegi_strategy.json").c_str());
  moPath = mo;

  // 発進前（グリッドは発進位置）
  totallaps = 7; reset("mo", moPath.c_str(), true);
  step(marginCfg.startM, 0.0f, false);
  save("strategy_idle", true); save("map_idle", false);

  // 惰行中: 2周通過が18秒遅れ → 加速停止速度 +1.0
  scene(2, 18.0f, 1000.0f, 27.4f, false);
  save("strategy_coast", true); save("map_coast", false);

  // 加速開始位置の手前（3つ目の60 m手前）
  scene(3, -2.0f, 1240.0f, 21.3f, false);
  save("strategy_approach", true); save("map_approach", false);

  // 加速開始
  scene(3, -2.0f, 1302.0f, 20.1f, false);
  save("strategy_burn_now", true); save("map_burn_now", false);

  // 加速中: 38秒遅れ → +3.0
  scene(2, 38.0f, 1320.0f, 28.6f, true);
  save("strategy_burning", true); save("map_burning", false);

  // 加速停止速度に到達
  scene(2, 38.0f, 1336.0f, 35.6f, true);
  save("strategy_cut_now", true); save("map_cut_now", false);

  // 最低限界速度
  scene(4, 5.0f, 2200.0f, 15.8f, false);
  save("strategy_guard", true);

  // 最終周: 3つ目は使わない
  scene(6, 10.0f, 1300.0f, 26.0f, false);
  save("strategy_coast_home", true); save("map_coast_home", false);

  // 燃圧低下: 35秒早い → -0.5
  scene(1, -35.0f, 1500.0f, 30.2f, true, false, 0.25f);
  save("strategy_fuel_low", true);

  // 予備パターン
  scene(5, 12.0f, 100.0f, 24.0f, false, true);
  save("strategy_fallback", true);

  // 予備パターン + 燃圧低下（コース図）: 39秒遅れ → +3.0
  scene(3, 39.0f, 260.0f, 27.0f, true, true, 0.25f);
  for (int i = 0; i < 10; i++) step(260 + i * 6, 27, false);
  save("map_fallback_fuel_low", false);

  // 設定ファイルなし: 鈴鹿
  totallaps = 8; reset("su", su.c_str(), false); worktime = 754; Lapcount = 2;
  step(900, 25.2f, false);
  save("map_suzuka_no_plan", false);

  // 鈴鹿の頂上の止め時（sd/suzuka_strategy.json）: S 字の登りで 21 km/h から加速し、表の速度で止めて頂上へ惰行する
  loadPlan((sd + "/suzuka_strategy.json").c_str());
  reset("su", su.c_str(), true); worktime = 493; Lapcount = 2;
  strategy.onLapCrossed(2, 493.0f);
  limitMargin.onLapCrossed(2, 493.0f);
  for (float p = 1306.0f; p <= 1405.0f; p += 1.0f) {
    worktime += 0.2f;
    step(p, 21.0f + (p - 1306.0f) * 0.1f, true);
  }
  save("strategy_crest_burning", true); save("map_crest_burning", false);
  worktime += 0.2f; step(1414.0f, 33.0f, true);
  save("strategy_crest_cut_now", true);
  for (float p = 1416.0f, v = 33.0f; p <= 1500.0f; p += 4.0f, v -= 0.8f) {
    worktime += 0.4f;
    step(p, v, false);
  }
  save("strategy_crest_cresting", true); save("map_crest_cresting", false);
  return 0;
}
