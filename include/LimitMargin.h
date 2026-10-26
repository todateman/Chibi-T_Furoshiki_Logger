#pragma once

#include <math.h>
#include <stdint.h>

// 規定時間に対する余裕の計算
// 規定時間ちょうどにゴールする仮想車（発進からゴールまでの道のりを規定時間で割った一定の速さで進む）と比べる。
// Arduino非依存（PC上でテストできる）。画面・SD読み込みは main.cpp 側で行う。
// 道のりの取り方は UNOR4_Chibi-T_EFI/tools/strategy_sim.py の race_span() と同じにしてある。
// 位置はすべて周内位置（ウェイポイント ID 1 からの距離を、公式の1周の距離に伸縮したもの）。
namespace LimitMargin {

constexpr uint16_t MAX_PLAN_POINTS = 200;  // 計画の「道のり → 累積時間」表の最大点数
constexpr float DEFAULT_BAND_LO_S = 10.0f;  // 余裕の目標の帯 [s]（設定ファイルに無い場合）
constexpr float DEFAULT_BAND_HI_S = 30.0f;

struct Config {
  bool enabled = false;
  float lapM = 0.0f;       // 1周の距離 [m]
  float startM = 0.0f;     // 発進位置 [m]（周内位置）
  float controlM = 0.0f;   // コントロールライン（周回を数える位置）[m]（周内位置）
  float raceM = 0.0f;      // 発進からゴールまでの道のり [m]
  float limitS = 0.0f;     // 規定時間 [s]
  float bandLoS = DEFAULT_BAND_LO_S;  // 余裕の目標の帯 [s]
  float bandHiS = DEFAULT_BAND_HI_S;
  float planStepM = 0.0f;  // 計画の表の間隔 [m]
  uint16_t planCount = 0;  // 計画の表の点数（0なら予測しない）
  float planS[MAX_PLAN_POINTS] = {};  // 道のり i×planStepM での計画の累積時間 [s]。最後の点だけは raceM（ゴール）
};

// [0, lap) に折り返す
inline float wrapM(float x, float lap) {
  float r = fmodf(x, lap);
  if (r < 0.0f) {
    r += lap;
  }
  return (r >= lap) ? r - lap : r;
}

// 発進から最初にコントロールラインを通過するまでの道のり [m]（ライン上から発進するなら1周）
inline float firstCrossingM(const Config& cfg) {
  const float d = wrapM(cfg.controlM - cfg.startM, cfg.lapM);
  return (d > 0.0f) ? d : cfg.lapM;
}

// n回目のコントロールライン通過時の道のり [m]
inline float crossingM(const Config& cfg, uint8_t lapsDone) {
  if (lapsDone == 0) {
    return 0.0f;
  }
  return firstCrossingM(cfg) + static_cast<float>(lapsDone - 1) * cfg.lapM;
}

// 道のり distM での規定への余裕 [s]（+ が余裕）。仮想車がその位置に来る時刻との差
inline float marginAt(const Config& cfg, float distM, float worktimeS) {
  return distM * cfg.limitS / cfg.raceM - worktimeS;
}

// 計画どおりに走った場合に、道のり distM へ着く累積時間 [s]。表が無ければNAN
inline float plannedTimeAt(const Config& cfg, float distM) {
  if (cfg.planCount < 2 || cfg.planStepM <= 0.0f) {
    return NAN;
  }
  const uint16_t last = cfg.planCount - 1;
  const float d = (distM < 0.0f) ? 0.0f : (distM > cfg.raceM ? cfg.raceM : distM);
  uint16_t i = static_cast<uint16_t>(d / cfg.planStepM);
  if (i >= last) {
    i = last - 1;
  }
  const float x0 = static_cast<float>(i) * cfg.planStepM;
  const float x1 = (i + 1 == last) ? cfg.raceM : static_cast<float>(i + 1) * cfg.planStepM;
  const float ratio = (x1 > x0) ? (d - x0) / (x1 - x0) : 0.0f;
  return cfg.planS[i] + (ratio > 1.0f ? 1.0f : ratio) * (cfg.planS[i + 1] - cfg.planS[i]);
}

// 設定の値が妥当かどうか
inline bool isValidConfig(const Config& cfg) {
  if (!(cfg.lapM > 0.0f) || !(cfg.raceM > 0.0f) || !(cfg.limitS > 0.0f)) {
    return false;
  }
  if (!(cfg.startM >= 0.0f && cfg.startM < cfg.lapM) || !(cfg.controlM >= 0.0f && cfg.controlM < cfg.lapM)) {
    return false;
  }
  if (!(cfg.bandLoS <= cfg.bandHiS)) {
    return false;
  }
  if (cfg.planCount == 0) {
    return true;
  }
  // 表は 0, step, 2·step, … と最後に raceM。最後の手前の点が raceM より前、最後の点が step 以内であること
  if (cfg.planCount < 2 || cfg.planCount > MAX_PLAN_POINTS || !(cfg.planStepM > 0.0f) ||
      !(static_cast<float>(cfg.planCount - 2) * cfg.planStepM < cfg.raceM) ||
      !(cfg.raceM <= static_cast<float>(cfg.planCount - 1) * cfg.planStepM + 0.5f)) {
    return false;
  }
  for (uint16_t i = 0; i < cfg.planCount; i++) {
    if (!isfinite(cfg.planS[i]) || cfg.planS[i] < 0.0f || (i > 0 && cfg.planS[i] <= cfg.planS[i - 1])) {
      return false;
    }
  }
  return true;
}

// 余裕が目標の帯に対してどこにあるか
enum class Band : uint8_t {
  Over,      // 帯より大きい（燃料の無駄）
  In,        // 帯の中
  Under,     // 帯より小さい（0以上）
  Negative,  // マイナス（このままでは規定時間を超える）
};

inline Band bandOf(const Config& cfg, float marginS) {
  if (marginS < 0.0f) {
    return Band::Negative;
  }
  if (marginS < cfg.bandLoS) {
    return Band::Under;
  }
  if (marginS <= cfg.bandHiS) {
    return Band::In;
  }
  return Band::Over;
}

struct Output {
  bool valid = false;        // 余裕を出せる（発進後で、位置が一度は取れた）
  float distM = NAN;         // 発進からの道のり [m]
  float marginS = NAN;       // いまの位置での規定への余裕 [s]
  float predictedS = NAN;    // 計画の残りどおりに走ったときの、ゴール時の規定への余裕 [s]。表が無ければNAN
  bool hasLapMargin = false; // 周回通過時の余裕を保持している
  uint8_t lap = 0;           // 何周目の通過か
  float lapMarginS = NAN;    // 通過時の規定への余裕 [s]
  bool finished = false;     // ゴールした（余裕はゴール時の値で止める）
};

class Tracker {
public:
  void configure(const Config& cfg) {
    cfg_ = cfg;
    reset();
  }

  const Config& config() const { return cfg_; }
  const Output& output() const { return out_; }

  // 走行時間 worktimeS で仮想車がいる周内位置 [m]（発進前は発進位置、規定時間を過ぎたらゴール）
  float virtualLapPosM(float worktimeS) const {
    if (!cfg_.enabled) {
      return NAN;
    }
    float ratio = worktimeS / cfg_.limitS;
    ratio = (ratio < 0.0f) ? 0.0f : (ratio > 1.0f ? 1.0f : ratio);
    return wrapM(cfg_.startM + cfg_.raceM * ratio, cfg_.lapM);
  }

  // コントロールライン通過を通知する（lapsDone は通過後の完了周回数、splitS は通過時の走行時間）
  void onLapCrossed(uint8_t lapsDone, float splitS) {
    if (!cfg_.enabled || !launched_) {
      return;
    }
    // 周回カウントを取りこぼしていると周回数が1つ少ないので、位置から追っている道のりに近い方の通過とみなす
    float d = nearest(crossingM(cfg_, lapsDone));
    if (d > cfg_.raceM) {
      d = cfg_.raceM;
    }
    out_.hasLapMargin = true;
    out_.lap = lapsDone;
    out_.lapMarginS = marginAt(cfg_, d, splitS);
    if (d >= cfg_.raceM - 1.0f) {
      finished_ = true;  // ゴールラインがコントロールライン上にある場合はここでゴール
      finishS_ = splitS;
      distM_ = cfg_.raceM;
      hasDist_ = true;
    }
  }

  // 走行時間・完了周回数・周内位置（不明ならNAN）から余裕を求める
  const Output& update(float worktimeS, uint8_t lapsDone, float lapPosM) {
    if (!cfg_.enabled) {
      out_ = Output();
      return out_;
    }
    if (worktimeS <= 0.0f) {
      reset();  // 発進前（ECUが走行時間を0に保持している）。前回走行の状態を持ち越さない
      return out_;
    }
    launched_ = true;

    if (isfinite(lapPosM) && !finished_) {
      // ライン付近では周回カウントと位置が少しずれる（カウント直後に位置がまだ手前・位置が先にラインを越える）。
      // 周回カウントの取りこぼしも含め、前回から半周以上飛んだら1周ずらした方を採る
      float d = nearest(rawDistM(lapsDone, lapPosM));
      d = (d < 0.0f) ? 0.0f : (d > cfg_.raceM ? cfg_.raceM : d);
      distM_ = d;
      hasDist_ = true;
      if (d >= cfg_.raceM) {
        finished_ = true;  // ゴールラインがコントロールラインと別の場所にある場合
        finishS_ = worktimeS;
      }
    }

    out_.finished = finished_;
    if (!hasDist_) {
      out_.valid = false;  // 位置がまだ一度も取れていない
      return out_;
    }
    const float t = finished_ ? finishS_ : worktimeS;
    out_.valid = true;
    out_.distM = distM_;
    out_.marginS = marginAt(cfg_, distM_, t);
    const float planHere = plannedTimeAt(cfg_, distM_);
    out_.predictedS = isfinite(planHere) ? cfg_.limitS - (t + plannedTimeAt(cfg_, cfg_.raceM) - planHere) : NAN;
    return out_;
  }

private:
  // 道のり d と1周ずらした値のうち、前回の道のりに近いものを返す（前回が無ければそのまま）
  float nearest(float d) const {
    if (hasDist_) {
      if (d - distM_ > 0.5f * cfg_.lapM) {
        return d - cfg_.lapM;
      }
      if (distM_ - d > 0.5f * cfg_.lapM) {
        return d + cfg_.lapM;
      }
    }
    return d;
  }

  // 周回数と周内位置から出す道のり（連続性の補正前）
  float rawDistM(uint8_t lapsDone, float lapPosM) const {
    if (lapsDone == 0) {
      // 発進から最初の通過までの区間の外（ラインと発進位置のあいだ）で、発進位置に近い側は発進位置の手前とみなす
      const float c0 = firstCrossingM(cfg_);
      const float d = wrapM(lapPosM - cfg_.startM, cfg_.lapM);
      return (d > c0 + 0.5f * (cfg_.lapM - c0)) ? d - cfg_.lapM : d;
    }
    return crossingM(cfg_, lapsDone) + wrapM(lapPosM - cfg_.controlM, cfg_.lapM);
  }

  void reset() {
    out_ = Output();
    launched_ = false;
    hasDist_ = false;
    distM_ = 0.0f;
    finished_ = false;
    finishS_ = 0.0f;
  }

  Config cfg_;
  Output out_;
  bool launched_ = false;
  bool hasDist_ = false;
  float distM_ = 0.0f;
  bool finished_ = false;
  float finishS_ = 0.0f;
};

}  // namespace LimitMargin
