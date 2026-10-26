#!/bin/bash
# README の画面例（docs/strategy_screen_*.png / docs/course_map_*.png）を PC 上で描き直す。
# src/main.cpp の描画関数（drawStrategyMode / drawCourseMapMode など）をそのまま抜き出し、
# M5GFX の SDL 版でスプライトに描いて 2 倍に拡大した PNG にする。設定は sd/motegi_strategy.json を読む。
#
# 必要なもの: macOS か Linux の clang、SDL2（macOS は brew install sdl2）、Python 3、
#            一度ビルドして取得したライブラリ（pio run -e m5stack-basic で .pio/libdeps/m5stack-basic に入る）
#
# 使い方:
#   tools/render_screens/render.sh             # .pio/render_screens/out/ に全場面の PNG を出す（確認用）
#   tools/render_screens/render.sh --install   # さらに README で使う画像を docs/ へ上書きする
#
# 場面（位置・速度・遅れなど）は harness.cpp の main() にある。描画関数の名前や、描画に使う関数・変数が
# 変わったら extract.py の抜き出し対象と harness.cpp の宣言を合わせる。
# M5GFX を更新したら .pio/render_screens/obj を消してから回す（ライブラリのオブジェクトを使い回しているため）。
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
R=$(cd "$HERE/../.." && pwd)
G=$R/.pio/libdeps/m5stack-basic/M5GFX/src
J=$R/.pio/libdeps/m5stack-basic/ArduinoJson/src
B=$R/.pio/render_screens

if [ ! -d "$G" ] || [ ! -d "$J" ]; then
  echo "M5GFX / ArduinoJson が見つかりません。先に pio run -e m5stack-basic を実行してください" >&2
  exit 1
fi
SDL2_CONFIG=$(command -v sdl2-config || echo /opt/homebrew/bin/sdl2-config)
if [ ! -x "$SDL2_CONFIG" ]; then
  echo "sdl2-config が見つかりません。SDL2 を入れてください（macOS: brew install sdl2）" >&2
  exit 1
fi

CF="-O1 -w -DSDL_MAIN_HANDLED -I$G $("$SDL2_CONFIG" --cflags)"
mkdir -p "$B/obj" "$B/out"
cd "$B"

# ライブラリは初回だけビルドする
for f in "$G"/lgfx/v1/*.cpp "$G"/lgfx/v1/misc/*.cpp "$G"/lgfx/v1/panel/Panel_Device.cpp "$G"/lgfx/v1/panel/Panel_FrameBufferBase.cpp \
         "$G"/lgfx/v1/platforms/sdl/*.cpp "$G"/M5GFX.cpp; do
  o=obj/$(basename "$f").o; [ -f "$o" ] || clang++ -std=gnu++17 $CF -c "$f" -o "$o"
done
for f in "$G"/lgfx/utility/*.c "$G"/lgfx/Fonts/efont/*.c "$G"/lgfx/Fonts/IPA/*.c; do
  o=obj/$(basename "$f").o; [ -f "$o" ] || clang $CF -c "$f" -o "$o"
done

python3 "$HERE/extract.py" "$R/src/main.cpp" extracted.inc
clang++ -std=gnu++17 $CF -I"$B" -I"$R/include" -I"$J" -c "$HERE/harness.cpp" -o harness.o
clang++ harness.o obj/*.o $("$SDL2_CONFIG" --libs) -o harness
rm -f out/*
./harness "$R/sd"
python3 "$HERE/topng.py" out/*.ppm
rm -f out/*.ppm
echo "出力: $B/out"

if [ "${1:-}" = "--install" ]; then
  for n in idle coast approach burn_now burning cut_now guard coast_home fuel_low fallback; do
    cp "out/strategy_$n.png" "$R/docs/strategy_screen_$n.png"
  done
  for n in approach burn_now cut_now coast_home fallback_fuel_low suzuka_no_plan; do
    cp "out/map_$n.png" "$R/docs/course_map_$n.png"
  done
  echo "docs/ の画面例を書き換えました"
fi
