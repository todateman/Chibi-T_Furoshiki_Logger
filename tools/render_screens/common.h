// harness.cpp（画面例）と video.cpp（仮想走行の動画）で共通の、src/main.cpp の代わりの定義と読み込み
// 描画・判定の関数は render.sh が src/main.cpp から抜き出した extracted.inc を取り込む。
// 取り込む側は STRATEGY_SIM（仮想走行の時間の倍率。画面例は 0）と STRATEGY_SIM_SUZUKA を決めてから include する
#pragma once
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <algorithm>
#include <M5GFX.h>
#include <ArduinoJson.h>
#include "CourseMap.h"
#include "LimitMargin.h"
#include "StrategyAssist.h"

using std::max;
using std::min;
#define pi 3.141592653589793
template <typename T, typename U, typename V> auto constrain(T v, U lo, V hi) -> decltype(v + lo + hi) { return v < lo ? lo : (v > hi ? hi : v); }

struct String {
  std::string s;
  String(const char* c = "") : s(c) {}
  bool operator==(const char* c) const { return s == c; }
  size_t length() const { return s.size(); }
};
struct {
  template <typename... A> void printf(const char* f, A... a) { ::printf(f, a...); }
  void println(const char* s) { ::puts(s); }
} Serial;

#ifndef STRATEGY_SIM_SUZUKA
#define STRATEGY_SIM_SUZUKA 0
#endif
#define STRATEGY_OFF_COURSE_M 60.0
#define STRATEGY_BEEP_AVAILABLE 1
#define GNSS_MODULE_M5 0

// millis() の代わり（動画では loop() 1 回ぶんずつ進める）
static uint32_t fakeMillis = 0;
static unsigned long millis() { return fakeMillis; }
// M5Unified のボタンとスピーカーの代わり。tone() は onTone があればそこへ渡す（動画の音声用）
static void (*onTone)(float hz, uint32_t ms, bool stopCurrent) = nullptr;
struct {
  struct { bool wasHold() { return false; } } BtnB;
  struct { void tone(float hz, uint32_t ms, int, bool stopCurrent) { if (onTone) onTone(hz, ms, stopCurrent); } } Speaker;
} M5;
unsigned long receiveECUtime = 0;
uint16_t tachoRpm = 0;
double spd = 0;
uint16_t lapCountDistanceBase = 0;
bool lapCountDistanceBaseValid = true;
static LGFX_Sprite lcd_s;
bool LOGGING = true, MQTTpush = true;
float INJ_timems = 0, speed = 0, worktime = 0, FuelPre = 0.32f;
uint16_t distance = 0, goal = 16389, limittime = 2360;
uint8_t Lapcount = 0, totallaps = 7;
double la = 0, ln = 0;
String Loc = "";
const size_t MAX_WAYPOINTS = 300;
StrategyAssist::Assist strategy;
LimitMargin::Tracker limitMargin;
enum StrategySource : uint8_t { STRATEGY_SRC_NONE, STRATEGY_SRC_SD, STRATEGY_SRC_SD_ERROR };
StrategySource strategySource = STRATEGY_SRC_NONE;
bool positionFresh = false;
uint32_t positionAgeMs = 0;
int nearestWaypointIndex = -1;
size_t waypointCount = 0;
float waypointRawLapMeters = 0;
double waypointMinLat, waypointMaxLat, waypointMinLng, waypointMaxLng;
struct Waypoint;
extern Waypoint waypoints[];

#include "extracted.inc"

Waypoint waypoints[MAX_WAYPOINTS];

static void loadWaypoints(const char* path) {
  FILE* f = fopen(path, "r");
  if (!f) { perror(path); exit(1); }
  char line[256];
  waypointCount = 0;
  waypointMinLat = waypointMinLng = 1000; waypointMaxLat = waypointMaxLng = -1000;
  while (fgets(line, sizeof(line), f) && waypointCount < MAX_WAYPOINTS) {
    const char* p = line;
    if ((unsigned char)p[0] == 0xEF) p += 3;  // BOM
    int id; float a, b, c, d;
    if (sscanf(p, "%d,%f,%f,%f,%f", &id, &a, &b, &c, &d) < 5) continue;
    Waypoint& w = waypoints[waypointCount++];
    w.id = id; w.lat = a; w.lng = b; w.alt = c; w.dist = d;
    waypointMinLat = min(waypointMinLat, (double)a); waypointMaxLat = max(waypointMaxLat, (double)a);
    waypointMinLng = min(waypointMinLng, (double)b); waypointMaxLng = max(waypointMaxLng, (double)b);
  }
  fclose(f);
  const Waypoint& first = waypoints[0]; const Waypoint& last = waypoints[waypointCount - 1];
  waypointRawLapMeters = last.dist + (float)approxDistanceMeters(last.lat, last.lng, first.lat, first.lng);
}

static StrategyAssist::Config planCfg;
static LimitMargin::Config marginCfg;

static void loadPlan(const char* path) {
  FILE* f = fopen(path, "rb");
  if (!f) { perror(path); exit(1); }
  static char buf[4096];
  const size_t n = fread(buf, 1, sizeof(buf) - 1, f);
  buf[n] = '\0';
  fclose(f);
  planCfg = StrategyAssist::Config();
  planCfg.totalLaps = 7;
  marginCfg = LimitMargin::Config();
  if (!parseStrategyConfigJson(buf, planCfg, marginCfg) || !isValidStrategyConfig(planCfg) || !LimitMargin::isValidConfig(marginCfg)) {
    fprintf(stderr, "invalid config: %s\n", path);
    exit(1);
  }
  planCfg.enabled = true;
}
