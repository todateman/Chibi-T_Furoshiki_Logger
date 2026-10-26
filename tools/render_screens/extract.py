#!/usr/bin/env python3
"""src/main.cpp から描画に要る関数・定義を抜き出して extracted.inc に書く"""
import re, sys
src = open(sys.argv[1], encoding="utf-8").read()

def block(start_pat):
    m = re.search(start_pat, src, re.M)
    assert m, start_pat
    i = src.index("{", m.start())
    depth = 0
    for j in range(i, len(src)):
        if src[j] == "{": depth += 1
        elif src[j] == "}":
            depth -= 1
            if depth == 0:
                end = j + 1
                if src[end:end+1] == ";": end += 1
                return src[m.start():end] + "\n\n"
    raise SystemExit("unbalanced")

out = []
out.append("\n".join(l for l in src.splitlines() if re.match(r"const (float|uint8_t|uint16_t) (controlline_|totallaps_|goal_|limittime_)", l)) + "\n\n")
for pat in [r"^struct Waypoint \{", r"^double approxDistanceMeters\(", r"^bool lapPositionToLatLng\(",
            r"^void readStrategyPatternJson\(", r"^bool isValidStrategyPattern\(", r"^bool isValidStrategyConfig\(", r"^bool readCrestCutJson\(",
            r"^bool readLimitMarginJson\(", r"^bool parseStrategyConfigJson\(", r"^float currentLapPositionMeters\(",
            r"^void formatMinSec\(", r"^bool formatStrategyCue\(", r"^int limitMarginColor\(", r"^int limitMarginSeconds\(",
            r"^void drawStrategyMode\(", r"^bool getControlLineForLoc\(.*\{$", r"^void drawCourseMapMode\("]:
    out.append(block(pat))
open(sys.argv[2], "w", encoding="utf-8").write("".join(out))
