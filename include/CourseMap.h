#pragma once

#include <math.h>

// コース図の投影（緯度経度 → 画面座標）
// Arduino非依存（PC上でテストできる）。描画は main.cpp 側で行う。
namespace CourseMap {

constexpr double METERS_PER_DEG = 111320.0;              // 緯度1度あたりの距離 [m]
constexpr double RAD_PER_DEG = 0.017453292519943295;

// 緯度経度の範囲を画面上の矩形へ収める投影（北が上、縦横は同じ縮尺。サーキット内の短距離用）
struct Projection {
  bool valid = false;
  double latC = 0.0;         // 範囲の中心の緯度経度
  double lngC = 0.0;
  double mPerDegLng = 0.0;   // 経度1度あたりの距離 [m]
  float pxPerM = 0.0f;       // 縮尺 [px/m]
  float cx = 0.0f;           // 矩形の中心 [px]
  float cy = 0.0f;

  // 中心から東へ何m離れているか
  float eastMeters(double lng) const { return static_cast<float>((lng - lngC) * mPerDegLng); }
  // 中心から北へ何m離れているか
  float northMeters(double lat) const { return static_cast<float>((lat - latC) * METERS_PER_DEG); }

  void toScreen(double lat, double lng, int& x, int& y) const {
    x = static_cast<int>(lroundf(cx + eastMeters(lng) * pxPerM));
    y = static_cast<int>(lroundf(cy - northMeters(lat) * pxPerM));
  }
};

// 緯度経度の範囲が、左上(left, top)・幅width・高さheight [px] の矩形にちょうど収まる投影を返す
// 範囲がつぶれている（点が1つだけ等）場合は valid が false になる
inline Projection fit(double minLat, double maxLat, double minLng, double maxLng,
                      int left, int top, int width, int height) {
  Projection p;
  p.latC = (minLat + maxLat) / 2.0;
  p.lngC = (minLng + maxLng) / 2.0;
  p.mPerDegLng = METERS_PER_DEG * cos(p.latC * RAD_PER_DEG);
  p.cx = static_cast<float>(left) + static_cast<float>(width - 1) / 2.0f;
  p.cy = static_cast<float>(top) + static_cast<float>(height - 1) / 2.0f;

  const float spanEastM = static_cast<float>((maxLng - minLng) * p.mPerDegLng);
  const float spanNorthM = static_cast<float>((maxLat - minLat) * METERS_PER_DEG);
  if (width < 2 || height < 2 || !(spanEastM >= 0.0f) || !(spanNorthM >= 0.0f) ||
      (spanEastM <= 0.0f && spanNorthM <= 0.0f)) {
    return p;
  }

  // 縦横のうち、先に矩形の端へ届く方に縮尺を合わせる
  const float scaleEast = (spanEastM > 0.0f) ? static_cast<float>(width - 1) / spanEastM : INFINITY;
  const float scaleNorth = (spanNorthM > 0.0f) ? static_cast<float>(height - 1) / spanNorthM : INFINITY;
  p.pxPerM = (scaleEast < scaleNorth) ? scaleEast : scaleNorth;
  p.valid = true;
  return p;
}

}  // namespace CourseMap
