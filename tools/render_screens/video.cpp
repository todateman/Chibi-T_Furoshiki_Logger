// 仮想走行（src/main.cpp の updateSimulatedRun()。m5stack-basic-sim と同じ計算）を PC 上で等倍で回し、
// 走行支援画面（左）と現在地画面（右）を並べたコマを標準出力へ、ブザーの音を WAV に出す（render.sh --video から使う）。
// loop() の順（仮想走行 → 周回開始の検知 → 最近傍ウェイポイント → 走行支援の更新とブザー）と、表示の間隔 100 ms はファームと同じ。
// コースはコンパイル時の STRATEGY_SIM_SUZUKA（1: 鈴鹿, 0: もてぎ）で決まる。
// 使い方: video <sdディレクトリ> <WAVの出力先> > コマ（RGB24、640x240、10 fps）
#define STRATEGY_SIM 1
#include <vector>
#include "common.h"

namespace {

constexpr uint32_t LOOP_MS = 20;        // loop() 1 回の時間
constexpr uint32_t FRAME_MS = 100;      // 表示の間隔（src/main.cpp の DISPLAY_INTERVAL）
constexpr uint32_t AFTER_FINISH_MS = 15000;  // ゴールして止まってから映す時間
constexpr uint32_t MAX_MS = 3600000;    // これを超えたら打ち切る（ゴールしない設定など）
constexpr int SAMPLE_RATE = 22050;
constexpr int16_t AMPLITUDE = 6000;     // 机上確認用環境の音量（setVolume(100)）くらい

std::vector<int16_t> pcm;
double toneEndS = 0.0;  // 鳴らし終わる時刻（stopCurrent が false の音はこのあとに続ける）

// M5.Speaker.tone() の代わり。矩形波をWAVの波形に書き込む
void tone(float hz, uint32_t ms, bool stopCurrent) {
  const double nowS = fakeMillis / 1000.0;
  if (stopCurrent && toneEndS > nowS) {
    // 鳴っている音を止める
    for (size_t i = static_cast<size_t>(nowS * SAMPLE_RATE); i < pcm.size(); i++) {
      pcm[i] = 0;
    }
  }
  const double startS = stopCurrent ? nowS : std::max(nowS, toneEndS);
  const size_t from = static_cast<size_t>(startS * SAMPLE_RATE);
  const size_t to = static_cast<size_t>((startS + ms / 1000.0) * SAMPLE_RATE);
  if (pcm.size() < to) {
    pcm.resize(to, 0);
  }
  for (size_t i = from; i < to; i++) {
    pcm[i] = (fmod((i - from) * static_cast<double>(hz) / SAMPLE_RATE, 1.0) < 0.5) ? AMPLITUDE : -AMPLITUDE;
  }
  toneEndS = startS + ms / 1000.0;
}

void writeWav(const char* path, uint32_t totalMs) {
  pcm.resize(static_cast<size_t>(totalMs / 1000.0 * SAMPLE_RATE), 0);  // 動画と同じ長さにそろえる
  FILE* f = fopen(path, "wb");
  if (!f) { perror(path); exit(1); }
  const uint32_t dataBytes = static_cast<uint32_t>(pcm.size() * 2);
  auto u32 = [&](uint32_t v) { fwrite(&v, 4, 1, f); };
  auto u16 = [&](uint16_t v) { fwrite(&v, 2, 1, f); };
  fwrite("RIFF", 1, 4, f); u32(36 + dataBytes); fwrite("WAVEfmt ", 1, 8, f);
  u32(16); u16(1); u16(1); u32(SAMPLE_RATE); u32(SAMPLE_RATE * 2); u16(2); u16(16);
  fwrite("data", 1, 4, f); u32(dataBytes);
  fwrite(pcm.data(), 2, pcm.size(), f);
  fclose(f);
}

// 走行支援画面と現在地画面を並べた1コマを標準出力へ
void writeFrame() {
  static uint8_t rgb[240][640][3];
  for (int screen = 0; screen < 2; screen++) {
    lcd_s.fillScreen(TFT_BLACK);
    lcd_s.setTextColor(TFT_WHITE);
    if (screen == 0) drawStrategyMode(); else drawCourseMapMode();
    for (int y = 0; y < 240; y++) for (int x = 0; x < 320; x++) {
      const uint16_t v = lcd_s.readPixel(x, y);  // RGB565
      uint8_t* p = rgb[y][screen * 320 + x];
      p[0] = ((v >> 11) & 31) * 255 / 31; p[1] = ((v >> 5) & 63) * 255 / 63; p[2] = (v & 31) * 255 / 31;
    }
  }
  fwrite(rgb, 1, sizeof(rgb), stdout);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    fprintf(stderr, "使い方: %s <sdディレクトリ> <WAVの出力先> > frames.rgb\n", argv[0]);
    return 1;
  }
  const std::string sd = argv[1];
  const char* course = STRATEGY_SIM_SUZUKA ? "suzuka" : "motegi";
  lcd_s.setColorDepth(8);
  lcd_s.createSprite(320, 240);

  // 場所の判定と設定ファイルの読み込み（ファームでは測位後に loop() で行う）
  Loc = STRATEGY_SIM_SUZUKA ? "su" : "mo";
  totallaps = STRATEGY_SIM_SUZUKA ? totallaps_su : totallaps_mo;
  loadWaypoints((sd + "/" + course + "_waypoint.csv").c_str());
  loadPlan((sd + "/" + course + "_strategy.json").c_str());
  planCfg.totalLaps = totallaps;
  strategy.configure(planCfg);
  limitMargin.configure(marginCfg);
  strategySource = STRATEGY_SRC_SD;
  onTone = tone;

  uint32_t nextFrameMs = 0;
  uint32_t finishedMs = 0;
  for (fakeMillis = 0; fakeMillis < MAX_MS; fakeMillis += LOOP_MS) {
    updateSimulatedRun();
    positionAgeMs = 0;
    positionFresh = true;
    resetLapCountOnLaunch();
    nearestWaypointIndex = findNearestWaypointIndex(la, ln);
    updateStrategyAssist();
    if (fakeMillis >= nextFrameMs) {
      writeFrame();
      nextFrameMs += FRAME_MS;
    }
    if (finishedMs == 0 && Lapcount >= totallaps && speed <= 0.0f) {
      finishedMs = fakeMillis;
      fprintf(stderr, "ゴールして停止: 走行時間 %d:%04.1f\n", static_cast<int>(worktime) / 60, fmod(worktime, 60.0f));
    }
    if (finishedMs != 0 && fakeMillis - finishedMs >= AFTER_FINISH_MS) {
      break;
    }
  }
  if (finishedMs == 0) {
    fprintf(stderr, "警告: %u 分でゴールしなかったので打ち切りました（%u 周）\n", MAX_MS / 60000, Lapcount);
  }
  writeWav(argv[2], nextFrameMs);
  return 0;
}
