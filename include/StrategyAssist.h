#pragma once

#include <math.h>
#include <stdint.h>

// 走行戦略に基づくドライバー支援の判定ロジック
// Arduino非依存（PC上でテストできる）。画面・ブザー・SD読み込みは main.cpp 側で行う。
// 走行パターンの値はここには持たない。すべてSDカードの設定ファイルから与える。
// 判定ルールは UNOR4_Chibi-T_EFI/tools/strategy_sim.py と同じにしてある。
namespace StrategyAssist {

constexpr uint8_t MAX_MARKS = 6;    // 1周あたりの加速開始位置の最大数
constexpr uint8_t MAX_SPLITS = 12;  // 目標通過タイムの最大数
constexpr uint8_t MAX_CRESTS = 2;   // 頂上の止め時の区間の最大数
constexpr uint8_t MAX_CREST_POINTS = 64;  // 頂上の止め時の表の最大点数（1区間あたり）
constexpr uint8_t MAX_COAST_ZONES = 4;    // 加速しない区間の最大数

constexpr uint32_t ENGINE_HOLD_MS = 300;    // 噴射0が一瞬混じっても「加速中」を保つ時間
constexpr uint32_t BURN_CONFIRM_MS = 1500;  // これだけ噴射が続いたら始動成功とみなす（スタータのタイムアウト1秒より長く）
constexpr uint32_t FUEL_CHECK_DELAY_MS = 1000;  // 加速開始からこの時間は燃圧を判定しない
constexpr float FUEL_VALID_MIN_MPA = 0.05f; // これ以下の燃圧は「未受信」として扱う（BLE途絶時は0.0になる）
constexpr float STOPPED_KMH = 3.0f;         // これ未満は停車中とみなし、最低限界速度の合図を出さない
constexpr float MAX_EARLY_S = 120.0f;       // 目標よりこれ以上早い通過タイムは、時間基準か周回数のずれとみなして使わない

// 加速パターン（開始位置と加速停止速度）
struct Pattern {
  float vOffKmh = 0.0f;        // エンジンを止める速度 [km/h]
  uint8_t markCount = 0;       // 加速開始位置の数
  float marksM[MAX_MARKS] = {};  // 加速開始位置 [m]（スタートラインから、昇順）
  uint8_t finalLapMarks = 0;   // 最終周で有効にする目印の数（先頭から）
};

// 頂上の止め時: 頂上の手前の登り（fromM〜crestM）で加速しているとき、惰行で頂上を目標速度で越えられる速度に
// 達したらエンジンを止める。その速度の表（位置 fromM, fromM+stepM, … と最後に crestM）
struct CrestCut {
  float fromM = 0.0f;
  float crestM = 0.0f;
  float stepM = 0.0f;
  uint8_t count = 0;
  float kmh[MAX_CREST_POINTS] = {};
};

struct Config {
  bool enabled = false;
  float lapM = 0.0f;           // 1周の距離 [m]
  uint8_t totalLaps = 0;       // 規定周回数
  float guardKmh = 0.0f;       // 最低限界速度: この速度まで落ちたら場所によらず加速（0で無効）
  float skipKmh = 2.0f;        // 目印通過時に加速停止速度までこれ未満しか残っていなければ加速しない
  float approachM = 100.0f;    // 目印のこの距離手前から予告する
  float missWindowM = 200.0f;  // 目印を過ぎてもこの距離までは「加速」を出し続ける
  Pattern primary;             // 通常パターン
  Pattern fallback;            // 予備パターン（markCount == 0 なら無し）
  uint8_t splitCount = 0;
  float splitsS[MAX_SPLITS] = {};      // n周通過時の目標走行時間 [s]
  float finalExtraIfSplitOverS = 0.0f; // 最終周に入る通過タイムがこれを超えたら全目印を有効にする（0以下で無効）
  float fuelWarnMpa = 0.0f;    // 加速中の燃圧がこれを下回ったら警告（0で無効）
  bool beep = true;
  uint8_t crestCount = 0;
  CrestCut crests[MAX_CRESTS];  // 頂上の止め時（周内位置の昇順）
  float cutLagS = 0.0f;         // 止める合図からエンジンが止まるまでの遅れ [s]（頂上の止め時はこの先の位置で表を引く）
  float restartKmh = 5.0f;      // 頂上の止め時で止めたあと・加速しない区間では、この速度まで落ちるまで「加速」を出さない
  uint8_t coastZoneCount = 0;
  float coastZonesM[MAX_COAST_ZONES][2] = {};  // 加速しない区間 [開始 m, 終了 m]（開始 > 終了なら周の終端をまたぐ）
};

// 周内位置 pos が区間 [fromM, toM] の中か（fromM > toM なら周の終端をまたぐ区間）
inline bool inZone(float pos, float fromM, float toM) {
  return fromM <= toM ? (pos >= fromM && pos <= toM) : (pos >= fromM || pos <= toM);
}

// 頂上の止め時の表の i 番目の位置 [m]（最後の点は頂上）
inline float crestPointM(const CrestCut& cc, uint8_t i) {
  const float m = cc.fromM + cc.stepM * static_cast<float>(i);
  return m < cc.crestM ? m : cc.crestM;
}

// 位置 pos でエンジンを止めてよい速度 [km/h]（表を直線で補間）。区間の外はNAN
inline float crestCutKmh(const CrestCut& cc, float pos) {
  if (cc.count < 2 || cc.stepM <= 0.0f || !(pos >= cc.fromM && pos <= cc.crestM)) {
    return NAN;
  }
  uint8_t i = static_cast<uint8_t>((pos - cc.fromM) / cc.stepM);
  if (i > cc.count - 2) {
    i = cc.count - 2;
  }
  const float m0 = crestPointM(cc, i);
  const float m1 = crestPointM(cc, i + 1);
  const float t = (m1 > m0) ? (pos - m0) / (m1 - m0) : 0.0f;
  return cc.kmh[i] + (cc.kmh[i + 1] - cc.kmh[i]) * t;
}

// 周回通過時の遅れ（+ が遅れ）から、次の周の加速停止速度の上げ幅 [km/h] を決める
inline float paceBumpKmh(float delayS) {
  if (delayS > 35.0f) return 3.0f;
  if (delayS > 20.0f) return 2.0f;
  if (delayS > 8.0f) return 1.0f;
  if (delayS < -30.0f) return -0.5f;
  return 0.0f;
}

// 戦略どおりに走った場合の、走行時間 worktimeS での全周回の進み具合（0〜1）
// 目標通過タイムのあいだを直線で補間する。最後の目標を過ぎた場合や目標が無い場合はNAN
inline float plannedProgress(const Config& cfg, float worktimeS) {
  if (cfg.totalLaps == 0 || worktimeS <= 0.0f) {
    return NAN;
  }
  float prevS = 0.0f;
  for (uint8_t i = 0; i < cfg.splitCount && i < cfg.totalLaps; i++) {
    const float splitS = cfg.splitsS[i];
    if (worktimeS <= splitS) {
      return (static_cast<float>(i) + (worktimeS - prevS) / (splitS - prevS)) / static_cast<float>(cfg.totalLaps);
    }
    prevS = splitS;
  }
  return NAN;
}

// コントロールライン交差を1周として数えてよいか（前回カウントからの走行距離で判定）
// 発進直後の通過と、ライン付近での二重カウントを除外する
inline bool lapCrossingPlausible(float distSinceLastCountM, float lapM) {
  return distSinceLastCountM >= 0.5f * lapM;
}

enum class Cue : uint8_t {
  Off,         // 支援無効
  Idle,        // 発進前
  Coast,       // 惰行中（次の目印まで距離あり）
  Approach,    // 次の目印が近い
  BurnNow,     // 加速せよ
  Burning,     // 加速中
  CutNow,      // 停止せよ（加速停止速度に到達）
  CoastHome,   // 最終周: このまま惰行でゴール
  NoPosition,  // 位置不明（最低限界速度の合図と目視で走る）
  Finished,    // 規定周回を完了
};

struct Input {
  uint32_t nowMs = 0;      // 単調増加の時刻 [ms]
  float lapPosM = NAN;     // 周内位置 [m]（スタートラインから）。不明ならNAN
  float speedKmh = 0.0f;
  bool engineOn = false;   // 噴射中
  float worktimeS = 0.0f;  // 走行時間 [s]（発進前は0）
  uint8_t lapsDone = 0;    // 完了した周回数
  float fuelMpa = 0.0f;    // 燃圧 [MPa]
};

struct Output {
  Cue cue = Cue::Off;
  float vOffKmh = 0.0f;      // 今の周の加速停止速度（補正込み）
  float cutKmh = 0.0f;       // 今エンジンを止める速度（頂上の止め時の区間で加速中は表の値、それ以外は vOffKmh）
  bool crestZone = false;    // 頂上の止め時の区間で加速中（cutKmh は表の値）
  bool cresting = false;     // 頂上の止め時で止め、頂上へ惰行している
  float bumpKmh = 0.0f;      // 加速停止速度の補正量
  bool guard = false;        // 最低限界速度で「加速」を出している
  int8_t nextMark = -1;      // 次の目印の番号（無ければ -1）
  float nextMarkDistM = NAN; // 次の目印までの距離 [m]
  uint8_t activeMarks = 0;   // 今の周で有効な目印の数
  bool usingFallback = false;
  bool fuelLow = false;      // 直近の加速で燃圧が低下した
  bool hasSplit = false;     // 通過タイムを保持している
  uint8_t splitLap = 0;      // 何周目の通過か
  float splitS = 0.0f;       // 通過時の走行時間 [s]
  float delayS = NAN;        // 目標との差（+ が遅れ）。目標が無い周はNAN
};

class Assist {
public:
  void configure(const Config& cfg) {
    cfg_ = cfg;
    useFallback_ = false;
    resetRun();
  }

  const Config& config() const { return cfg_; }
  const Output& output() const { return out_; }
  bool hasFallback() const { return cfg_.fallback.markCount > 0; }
  bool usingFallback() const { return useFallback_; }

  // 通常パターン ⇄ 予備パターンの切替。通過済みの位置の目印では合図を出さない
  void setFallback(bool on) {
    on = on && hasFallback();
    if (on == useFallback_) {
      return;
    }
    useFallback_ = on;
    clearMarks();
    resync_ = true;
  }

  // コントロールライン通過を通知する（lapsDone は通過後の完了周回数、splitS は通過時の走行時間）
  void onLapCrossed(uint8_t lapsDone, float splitS) {
    clearMarks();
    awaitWrap_ = true;
    hasSplit_ = true;
    splitLap_ = lapsDone;
    splitS_ = splitS;
    delayS_ = NAN;
    bumpKmh_ = 0.0f;
    if (lapsDone >= 1 && lapsDone <= cfg_.splitCount) {
      const float delayS = splitS - cfg_.splitsS[lapsDone - 1];
      // ECUの再起動で走行時間が0に戻った場合などに、加速停止速度を誤って下げない
      if (delayS >= -MAX_EARLY_S) {
        delayS_ = delayS;
        bumpKmh_ = paceBumpKmh(delayS);
      }
    }
    finalAllMarks_ = cfg_.finalExtraIfSplitOverS > 0.0f && cfg_.totalLaps > 0 &&
                     lapsDone == cfg_.totalLaps - 1 && splitS > cfg_.finalExtraIfSplitOverS;
  }

  const Output& update(const Input& in) {
    Output o;
    const Pattern& pat = useFallback_ ? cfg_.fallback : cfg_.primary;
    o.usingFallback = useFallback_;
    o.bumpKmh = bumpKmh_;
    o.vOffKmh = pat.vOffKmh + bumpKmh_;
    o.cutKmh = o.vOffKmh;
    o.hasSplit = hasSplit_;
    o.splitLap = splitLap_;
    o.splitS = splitS_;
    o.delayS = delayS_;

    if (!cfg_.enabled) {
      o.cue = Cue::Off;
      out_ = o;
      return out_;
    }

    const bool finalLap = cfg_.totalLaps > 0 && in.lapsDone == cfg_.totalLaps - 1;
    uint8_t activeMarks = pat.markCount;
    if (finalLap && !finalAllMarks_ && pat.finalLapMarks < activeMarks) {
      activeMarks = pat.finalLapMarks;
    }
    o.activeMarks = activeMarks;

    // 噴射の有無から「加速中」を判定する（短い途切れは保持する）
    if (in.engineOn) {
      if (!engineOn_) {
        burnStartMs_ = in.nowMs;
        fuelLow_ = false;  // 加速のたびに判定し直す
      }
      engineOn_ = true;
      lastEngineOnMs_ = in.nowMs;
    } else if (engineOn_ && in.nowMs - lastEngineOnMs_ > ENGINE_HOLD_MS) {
      engineOn_ = false;
    }
    if (!engineOn_ && crestPending_ >= 0) {
      cresting_ = crestPending_;  // 頂上の止め時の合図でエンジンが止まった
      crestPending_ = -1;
    }
    const bool burnConfirmed = engineOn_ && in.nowMs - burnStartMs_ >= BURN_CONFIRM_MS;

    if (engineOn_ && in.nowMs - burnStartMs_ >= FUEL_CHECK_DELAY_MS &&
        in.fuelMpa > FUEL_VALID_MIN_MPA && in.fuelMpa < cfg_.fuelWarnMpa) {
      fuelLow_ = true;
    }
    o.fuelLow = fuelLow_;

    if (in.worktimeS <= 0.0f) {
      // 発進前（ECUが走行時間を0に保持している）。前回走行の状態を持ち越さない
      if (launched_) {
        resetRun();
        o.bumpKmh = 0.0f;
        o.vOffKmh = pat.vOffKmh;
        o.cutKmh = pat.vOffKmh;
        o.hasSplit = false;
        o.delayS = NAN;
      }
      o.cue = Cue::Idle;
      out_ = o;
      return out_;
    }

    launched_ = true;

    if (cfg_.totalLaps > 0 && in.lapsDone >= cfg_.totalLaps) {
      o.cue = Cue::Finished;
      out_ = o;
      return out_;
    }

    // 周内位置の更新
    const bool posValid = isfinite(in.lapPosM) && cfg_.lapM > 0.0f;
    float pos = NAN;
    if (posValid) {
      pos = in.lapPosM;
      // コントロールラインは周内位置の0 mより手前にある（もてぎは約117 m手前のゴールラインの横）。
      // 周回カウントの直後は位置がまだ周の終端付近を指すので、先頭へ戻るまでは負の位置（次の周の手前）として扱う
      if (awaitWrap_) {
        if (pos > 0.8f * cfg_.lapM) {
          pos -= cfg_.lapM;
        } else {
          awaitWrap_ = false;
        }
      }
      // 位置が終端から先頭へ戻ったら目印を新しい周として扱う
      // （周回カウントを取りこぼした場合や、グリッドがラインの手前にある発進時もここで切り替わる）
      if (isfinite(prevPos_) && prevPos_ > 0.8f * cfg_.lapM && pos < 0.2f * cfg_.lapM) {
        clearMarks();
      }
      // 発進直後・位置が取れるようになった直後・パターン切替直後は、すでに過ぎた目印で合図を出さない
      if (!posWasValid_ || resync_) {
        for (uint8_t i = 0; i < pat.markCount; i++) {
          if (pat.marksM[i] < pos) {
            seen_[i] = true;
            done_[i] = true;
          }
        }
        resync_ = false;
      }
      prevPos_ = pos;
    }
    posWasValid_ = posValid;

    // 頂上の止め時の区間と、加速しない区間（位置が分かるときだけ）
    int8_t crest = -1;
    bool coastOnly = false;
    if (posValid) {
      for (uint8_t i = 0; i < cfg_.crestCount; i++) {
        if (pos >= cfg_.crests[i].fromM && pos <= cfg_.crests[i].crestM) {
          crest = static_cast<int8_t>(i);
        }
      }
      for (uint8_t i = 0; i < cfg_.coastZoneCount; i++) {
        coastOnly = coastOnly || inZone(pos, cfg_.coastZonesM[i][0], cfg_.coastZonesM[i][1]);
      }
      if (cresting_ >= 0 && crest != cresting_) {
        cresting_ = -1;  // 頂上を越えた
      }
    }
    o.cresting = cresting_ >= 0;
    // 頂上へ惰行している・加速しない区間では、止まりそうになるまで「加速」を出さない
    const bool holdBurn = posValid && (o.cresting || coastOnly);

    // 目印の通過判定
    int8_t pending = -1;
    if (posValid) {
      for (uint8_t i = 0; i < activeMarks; i++) {
        if (!seen_[i] && pos >= pat.marksM[i]) {
          seen_[i] = true;
          if (in.speedKmh > o.vOffKmh - cfg_.skipKmh) {
            done_[i] = true;  // 十分速いので加速しない
          }
        }
        if (seen_[i] && !done_[i]) {
          const bool pastWindow = pos >= pat.marksM[i] + cfg_.missWindowM;
          const bool nextIsNear = i + 1 < activeMarks && pos >= pat.marksM[i + 1] - cfg_.approachM;
          if (burnConfirmed || pastWindow || nextIsNear || holdBurn) {
            done_[i] = true;
          } else if (pending < 0) {
            pending = static_cast<int8_t>(i);
          }
        }
      }

      for (uint8_t i = 0; i < activeMarks; i++) {
        if (!seen_[i]) {
          o.nextMark = static_cast<int8_t>(i);
          o.nextMarkDistM = pat.marksM[i] - pos;
          break;
        }
      }
      if (o.nextMark < 0 && !finalLap && pat.markCount > 0) {
        o.nextMark = 0;  // 次の周の最初の目印
        o.nextMarkDistM = cfg_.lapM - pos + pat.marksM[0];
      }
    }

    // 合図の決定
    const float guardKmh = (holdBurn && cfg_.restartKmh < cfg_.guardKmh) ? cfg_.restartKmh : cfg_.guardKmh;
    if (engineOn_ && coastOnly) {
      o.cue = Cue::CutNow;  // 下りなどに入ったら止める
    } else if (engineOn_ && crest >= 0) {
      // 頂上の止め時: 合図からエンジンが止まるまでの遅れのあいだに進む先の位置で、止めてよい速度を引く
      // （加速度の分は見込まない。合図はわずかに遅れ、頂上の速度は高めになる）
      const CrestCut& cc = cfg_.crests[crest];
      const float needKmh = crestCutKmh(cc, pos + in.speedKmh / 3.6f * cfg_.cutLagS);
      o.crestZone = true;
      o.cutKmh = isfinite(needKmh) ? needKmh : crestCutKmh(cc, pos);
      if (crestPending_ >= 0 || (isfinite(needKmh) && in.speedKmh >= needKmh)) {
        crestPending_ = crest;
        o.cue = Cue::CutNow;
      } else {
        o.cue = Cue::Burning;  // 頂上の手前の登りでは、加速停止速度で止めず頂上の止め時まで回す
      }
    } else if (engineOn_) {
      o.cue = (in.speedKmh >= o.vOffKmh) ? Cue::CutNow : Cue::Burning;
    } else if (in.speedKmh >= STOPPED_KMH && in.speedKmh <= guardKmh) {
      o.cue = Cue::BurnNow;
      o.guard = true;
    } else if (pending >= 0) {
      o.cue = Cue::BurnNow;
    } else if (!posValid) {
      o.cue = Cue::NoPosition;
    } else if (o.nextMark < 0) {
      o.cue = Cue::CoastHome;
    } else if (o.nextMarkDistM <= cfg_.approachM) {
      o.cue = Cue::Approach;
    } else {
      o.cue = Cue::Coast;
    }

    out_ = o;
    return out_;
  }

private:
  void clearMarks() {
    for (uint8_t i = 0; i < MAX_MARKS; i++) {
      seen_[i] = false;
      done_[i] = false;
    }
  }

  // 1回の走行ぶんの状態を初期化する（パターン選択は保持する）
  void resetRun() {
    clearMarks();
    launched_ = false;
    awaitWrap_ = false;
    resync_ = false;
    posWasValid_ = false;
    prevPos_ = NAN;
    engineOn_ = false;
    burnStartMs_ = 0;
    lastEngineOnMs_ = 0;
    fuelLow_ = false;
    hasSplit_ = false;
    splitLap_ = 0;
    splitS_ = 0.0f;
    delayS_ = NAN;
    bumpKmh_ = 0.0f;
    finalAllMarks_ = false;
    crestPending_ = -1;
    cresting_ = -1;
  }

  Config cfg_;
  Output out_;
  bool useFallback_ = false;
  bool seen_[MAX_MARKS] = {};  // 今の周でその目印を通過した
  bool done_[MAX_MARKS] = {};  // その目印の加速を済ませた（またはスキップ・期限切れ）
  bool launched_ = false;
  bool awaitWrap_ = false;     // ライン通過後、位置が周の先頭へ戻るのを待っている
  bool resync_ = false;
  bool posWasValid_ = false;
  float prevPos_ = NAN;
  bool engineOn_ = false;
  uint32_t burnStartMs_ = 0;
  uint32_t lastEngineOnMs_ = 0;
  bool fuelLow_ = false;
  bool hasSplit_ = false;
  uint8_t splitLap_ = 0;
  float splitS_ = 0.0f;
  float delayS_ = NAN;
  float bumpKmh_ = 0.0f;
  bool finalAllMarks_ = false;
  int8_t crestPending_ = -1;  // 頂上の止め時の「停止」を出した区間（エンジンが止まるまで保持）
  int8_t cresting_ = -1;      // 頂上の止め時で止め、頂上へ惰行している区間
};

}  // namespace StrategyAssist
