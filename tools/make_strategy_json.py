#!/usr/bin/env python3
"""走行支援の設定ファイル（sd/<コース>_strategy.json）を、ECU リポジトリの戦略パラメータから生成する。

ECU リポジトリ（UNOR4_Chibi-T_EFI）の tools/strategy_sim.py を読み込み、optimize と同じ手順で
推奨パターンを求めて、ロガーが読む形式に書き出す。練習走行後は ECU 側の
tools/strategy/<コース>_<年>.json を書き換えてから、このスクリプトを回し直す。

計算で決まる項目:
  - lap_m, guard_kmh                 パラメータの course からそのまま
  - v_off, marks_m                   optimize の推奨パターン
  - splits_s                         推奨パターンをずらさずに走ったときの各周の通過タイム（最後はゴール）
  - final_lap_marks                  最終周に加速を始める回数
  - final_extra_if_split_over_s      規定時間 - 余裕 - 最終周の所要時間。これより遅く最終周に入ると
                                     惰行で終えたときの余裕が足りない

  - start_m, control_m, race_m      発進位置・コントロールライン（周内位置）と、発進からゴールまでの道のり
                                     （race_span と同じ。規定時間の仮想車と、規定時間に対する余裕に使う）
  - limit_s                          パラメータの course からそのまま
  - plan_step_m, plan_t_s            計画の「道のり → 累積時間」表。splits_s と同じ走行（ずらさない）の速度トレースから、
                                     道のり 0, 100, 200 m, … の時刻と、最後にゴールの時刻を取る（ゴール時の余裕の予測に使う）

最終周の 2 項目は、加速開始位置を ±30 m ずらした 3 通りのうち完走側（回数は最大、所要時間は最長）を採る。

パラメータに "phases"（周ごとのパターン。練習走行用）があるときは探索をせず、書かれたとおりに作る:
  - 1 つ目の段階 → 通常パターン（起動直後に有効）、2 つ目の段階 → 予備パターン（B 長押しで切り替える）
  - splits_s は途中でパターンを切り替えて走ったときの通過タイム
  - 最終周の 2 項目は最終周を走る側（予備パターン）について計算する
  - 段階は 2 つまで（ファームが持てるパターンが 2 つのため）

パラメータからは決まらない項目は、出力先に既にあるファイルの値を引き継ぐ。初めて作るコースでは
オプションで与える:
  - fallback の v_off / marks_m（--fallback-v-off / --fallback-marks。final_lap_marks は計算し直す）
  - fuel_warn_mpa（--fuel-warn-mpa）, beep（--beep / --no-beep）
  - margin_band_s（--margin-band。規定時間に対する余裕の目標の帯 [下限, 上限] 秒。無ければファームの既定 10〜30 秒）

ECU 側の形式が変わったことに気づけるよう、次の 2 つの版数を照合する。違えば何も書き出さない:
  - パラメータ JSON の "format_version"        ↔ PARAMS_FORMAT_VERSION
  - strategy_sim.py の INTERFACE_VERSION       ↔ SIM_INTERFACE_VERSION
止まったら ECU 側で何が変わったかを確かめ、このスクリプトを合わせてから下の対応版数を上げる。

使い方:
  python3 tools/make_strategy_json.py motegi --dry-run    # 書き込まずに結果と変更点だけ表示
  python3 tools/make_strategy_json.py motegi              # → sd/motegi_strategy.json
  python3 tools/make_strategy_json.py suzuka --target 38:30
  python3 tools/make_strategy_json.py suzuka --params ../UNOR4_Chibi-T_EFI/tools/strategy/suzuka_2027.json
  python3 tools/make_strategy_json.py motegi --margin-band 10 30   # 余裕の目標の帯を変える

  # 練習走行用（本番用の sd/motegi_strategy.json を上書きしないよう出力先を分ける）
  python3 tools/make_strategy_json.py motegi --params ../UNOR4_Chibi-T_EFI/tools/strategy/practice_motegi_2026.json \
      --fuel-warn-mpa 0.30 --beep -o sd/motegi_strategy_practice.json
"""

import argparse
import json
import math
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
DEFAULT_SIM_DIR = REPO.parent / "UNOR4_Chibi-T_EFI" / "tools"
FIRMWARE = REPO / "src" / "main.cpp"

# コース名 → ファームのロケーション識別子（src/main.cpp の getStrategyFilePathByLoc と同じ対応）
COURSES = {"suzuka": "su", "motegi": "mo", "toyota": "to"}

# 対応している ECU 側の版数
PARAMS_FORMAT_VERSION = 4  # パラメータ JSON の format_version
SIM_INTERFACE_VERSION = 4  # strategy_sim.py の INTERFACE_VERSION

# ファーム側の上限（include/StrategyAssist.h, src/main.cpp）
MAX_PATTERNS = 2  # 通常パターンと予備パターン
MAX_MARKS = 6
MAX_SPLITS = 12
MAX_PLAN_POINTS = 200  # include/LimitMargin.h
MAX_FILE_BYTES = 4095  # STRATEGY_FILE_BUFFER_SIZE - 1
MAX_V_OFF_KMH = 60.0

PLAN_STEP_M = 100  # 計画の「道のり → 累積時間」表の間隔


class Literal(str):
    """既存ファイルの数値を、書かれていた表記のまま持ち回る（0.30 を 0.3 に変えない）。"""


def check_version(name: str, found, supported: int) -> None:
    if found != supported:
        raise ValueError(
            f"{name} が対応していない版です（見つかった版: {'なし' if found is None else found}、"
            f"このスクリプトが対応する版: {supported}）\n"
            "  ECU リポジトリ側で何が変わったかを確かめ、このスクリプトを合わせてから対応版数を上げてください")


def load_simulator(sim_dir: Path):
    if not (sim_dir / "strategy_sim.py").is_file():
        raise ValueError(f"strategy_sim.py が見つかりません: {sim_dir}（--sim-dir で指定してください）")
    sys.dont_write_bytecode = True  # ECU リポジトリに __pycache__ を作らない
    sys.path.insert(0, str(sim_dir))
    import strategy_sim

    check_version("strategy_sim.py の INTERFACE_VERSION", getattr(strategy_sim, "INTERFACE_VERSION", None),
                  SIM_INTERFACE_VERSION)
    return strategy_sim


def find_params(sim_dir: Path, course: str) -> Path:
    """コースのパラメータ JSON のうち、名前順で最後のもの（年が新しいもの）を返す。"""
    found = sorted((sim_dir / "strategy").glob(f"{course}_*.json"))
    if not found:
        raise ValueError(f"パラメータ JSON が見つかりません: {sim_dir / 'strategy'}/{course}_*.json（--params で指定してください）")
    return found[-1]


def firmware_laps(course: str) -> int | None:
    """ファームが持つ規定周回数（src/main.cpp の totallaps_su など）。見つからなければ None。"""
    try:
        source = FIRMWARE.read_text(encoding="utf-8")
    except OSError:
        return None
    m = re.search(rf"\btotallaps_{COURSES[course]}\s*=\s*(\d+)", source)
    return int(m.group(1)) if m else None


def check_course(course: str, p: dict) -> list[str]:
    """コースの取り違えを調べ、注意書きを返す。周回数がファームと違えば例外にする。"""
    notes = []
    laps = firmware_laps(course)
    if laps is None:
        notes.append(f"周回数: ファームに {course} の規定周回数が無いので照合していません")
    elif laps != p["course"]["laps"]:
        raise ValueError(f"周回数がファームと違います（パラメータ {p['course']['laps']} 周、ファーム {laps} 周）。"
                         f"コースの取り違えか、{FIRMWARE.name} の totallaps_{COURSES[course]} を確かめてください")
    ours = REPO / "sd" / f"{course}_waypoint.csv"
    theirs = Path(p["_dir"]) / p["course"]["waypoints"]
    if not ours.is_file() or ours.read_bytes() != theirs.read_bytes():
        notes.append(f"警告: ウェイポイントがロガーの {ours} と一致しません（加速開始位置がずれます）: {theirs}")
    return notes


def recommend(sim, p: dict, course, target_s: float, n_min: int, n_max: int, fuel_slack: float) -> dict | None:
    """推奨パターンの評価結果を返す。目標に収まる組が無ければ None。

    strategy_sim.py の cmd_optimize と同じ手順（そちらは表示専用で結果を返さないので写してある）。
    """
    c = p["course"]
    guard = c.get("guard_kmh", 16.0)
    lap_target = sim.steady_lap_target(c, course, target_s)
    results = []
    for n in range(n_min, n_max + 1):
        placed = sim.place_marks(p, course, n, lap_target, guard + 1.0)
        if placed is None:
            continue
        marks, v_off = placed
        v_off = math.ceil(v_off * 2) / 2 - 1.0
        for _ in range(12):
            r = sim.evaluate(p, course, {"v_off": v_off, "marks": marks, "v_on": guard})
            if r["time_s"] <= target_s:
                break
            v_off += 0.5
        results.append(r)
    _, rec = sim.pick(results, target_s, fuel_slack)
    return rec


def plan(sim, p: dict, course, args) -> dict | None:
    """パラメータの phases に書かれたプランの評価結果を返す。phases が無ければ None。"""
    pol = sim.plan_from_params(p)
    if pol is None:
        return None
    if 1 + len(pol["switches"]) > MAX_PATTERNS:
        raise ValueError(f"phases は {MAX_PATTERNS} つまでです（ロガーが持てるのは通常パターンと予備パターンだけ）")
    if args.target:
        raise ValueError("phases のあるパラメータでは探索をしないので、--target は使えません")
    if pol["switches"] and (args.fallback_v_off is not None or args.fallback_marks is not None):
        raise ValueError("phases の 2 つ目の段階が予備パターンになるので、--fallback-v-off / --fallback-marks は使えません")
    return sim.evaluate(p, course, pol)


def final_lap(sim, p: dict, course, pol: dict) -> tuple[int, float]:
    """(最終周に加速を始める回数の最大, 最終周の所要時間の最長 s) を返す。"""
    laps = p["course"]["laps"]
    counts, times = [], []
    for v_off, jitter in sim.jittered(pol):
        r = sim.simulate(p, course, v_off, **jitter)
        counts.append(sum(1 for lap, _, _ in r["starts"] if lap == laps))
        times.append(r["time_s"] - r["crossings"][laps - 2][0])
    last_marks = pol["switches"][-1][2] if pol.get("switches") else pol["marks"]
    return min(max(counts), len(last_marks)), max(times)


def number(value: float):
    """400.0 を 400 と書く。"""
    return int(value) if float(value).is_integer() else value


def plan_table(trace: list, s0: float, race_m: float, goal_s: float) -> list:
    """計画の「道のり → 累積時間」表。道のり 0, PLAN_STEP_M, 2·PLAN_STEP_M, … の時刻と、最後にゴールの時刻。

    trace は simulate(trace=..., trace_every=1) の (累積時間, コース距離, …) の並び。
    """
    pts = [(0.0, 0.0)] + [(s - s0, t) for t, s, *_ in trace]
    table, i, d = [], 0, 0.0
    while d < race_m:
        while i + 2 < len(pts) and pts[i + 1][0] < d:
            i += 1
        (d0, t0), (d1, t1) = pts[i], pts[i + 1]
        t = t0 + (d - d0) / (d1 - d0) * (t1 - t0) if d1 > d0 else t0  # 発進直後は道のりが 0 のままの行が続く
        table.append(number(round(t, 1)))
        d += PLAN_STEP_M
    table.append(number(round(goal_s, 1)))
    return table


def build(sim, p: dict, course, rec: dict, existing: dict, args) -> tuple[dict, list[str]]:
    """(設定ファイルの内容, 注意書き) を返す。"""
    c = p["course"]
    laps, limit = c["laps"], c["limit_s"]
    guard = c.get("guard_kmh", 16.0)
    notes = []

    pol = rec["pol"]
    v_off, marks, switches = pol["v_off"], pol["marks"], pol.get("switches", [])
    # 途中で予備パターンへ切り替えるプランでは、最終周を走るのは予備パターンの側
    count, longest = final_lap(sim, p, course, pol)
    final_marks = switches[-1][2] if switches else marks
    cfg = {
        "lap_m": round(c["total_m"] / laps, 2),
        "v_off": v_off,
        "marks_m": [number(m) for m in marks],
        "final_lap_marks": len(marks) if switches else count,
    }
    if count < len(final_marks):
        cfg["final_extra_if_split_over_s"] = math.floor(limit - sim.HOME_MARGIN_S - longest)
    cfg["guard_kmh"] = guard
    trace = []
    nominal = sim.simulate(p, course, v_off, marks=marks, v_on=guard, switches=switches, trace=trace, trace_every=1)
    cfg["splits_s"] = [round(t) for t, _, _ in nominal["crossings"][:laps]]  # 最後はゴールの目標時間

    kept = existing.get("fallback") or {}
    fb_v_off = args.fallback_v_off if args.fallback_v_off is not None else kept.get("v_off")
    fb_marks = args.fallback_marks if args.fallback_marks is not None else kept.get("marks_m")
    if switches:
        from_lap, sw_v_off, sw_marks = switches[0]
        cfg["fallback"] = {"v_off": sw_v_off, "marks_m": [number(m) for m in sw_marks], "final_lap_marks": count}
        notes.append(f"1〜{from_lap - 1} 周目は通常パターン、{from_lap} 周目から予備パターン"
                     f"（{from_lap - 1} 周通過後に B 長押しで切り替える）")
    elif fb_v_off is not None and fb_marks:
        fb_marks = sorted(float(m) for m in fb_marks)
        fb_count, _ = final_lap(sim, p, course, {"v_off": float(fb_v_off), "marks": fb_marks, "v_on": guard})
        cfg["fallback"] = {"v_off": fb_v_off, "marks_m": [number(m) for m in fb_marks], "final_lap_marks": fb_count}
        r = sim.evaluate(p, course, {"v_off": float(fb_v_off), "marks": fb_marks, "v_on": guard})
        notes.append(f"予備パターン: {sim.mmss(r['time_min_s'])}〜{sim.mmss(r['time_s'])}  加速 {r['burns']} 回")
        if r["time_s"] > limit:
            notes.append(f"警告: 予備パターンは規定時間 {sim.mmss(limit)} を超えます")
    else:
        notes.append("fallback: 既存の値も指定も無いので書き出しません")

    for key, given in (("fuel_warn_mpa", args.fuel_warn_mpa), ("beep", args.beep)):
        if given is not None:
            cfg[key] = given
        elif key in existing:
            cfg[key] = existing[key]
        else:
            notes.append(f"{key}: 既存の値も指定も無いので書き出しません")

    # 規定時間の仮想車と、ゴール時の余裕の予測
    s0, total, _ = sim.race_span(c, course.lap_m)
    race_m = total - s0
    cfg["start_m"] = number(s0)
    cfg["control_m"] = number(sim.control_m(c))
    cfg["race_m"] = round(race_m, 2)
    cfg["limit_s"] = limit
    if args.margin_band is not None:
        cfg["margin_band_s"] = [number(v) for v in args.margin_band]
    elif "margin_band_s" in existing:
        cfg["margin_band_s"] = existing["margin_band_s"]
    else:
        notes.append("margin_band_s: 既存の値も指定も無いので書き出しません（ファームの既定 10〜30 秒）")
    cfg["plan_step_m"] = PLAN_STEP_M
    cfg["plan_t_s"] = plan_table(trace, s0, race_m, nominal["time_s"])
    notes.append(f"規定時間の仮想車: 道のり {race_m:.0f} m ÷ 規定 {sim.mmss(limit)} = 平均 {race_m / limit * 3.6:.2f} km/h"
                 f"、計画のゴールで規定まで {limit - nominal['time_s']:+.0f} 秒")
    return cfg, notes


def check_pattern(name: str, pattern: dict, lap_m: float, guard: float) -> list[str]:
    problems = []
    marks = [float(m) for m in pattern["marks_m"]]
    if not 1 <= len(marks) <= MAX_MARKS:
        problems.append(f"{name}: 加速開始位置は 1〜{MAX_MARKS} 個（{len(marks)} 個）")
    if not guard < float(pattern["v_off"]) <= MAX_V_OFF_KMH:
        problems.append(f"{name}: v_off が最低限界速度 {guard} km/h 以下か {MAX_V_OFF_KMH} km/h 超")
    if any(not 0 <= m < lap_m for m in marks):
        problems.append(f"{name}: 加速開始位置が 1 周の距離 {lap_m} m の外")
    if any(b <= a for a, b in zip(marks, marks[1:])):
        problems.append(f"{name}: 加速開始位置が昇順でない")
    return problems


def check_margin(cfg: dict, lap_m: float) -> list[str]:
    """規定時間に対する余裕の項目で、ファームが採用しない条件（include/LimitMargin.h の isValidConfig）。"""
    problems = []
    race_m, limit_s = float(cfg["race_m"]), float(cfg["limit_s"])
    if race_m <= 0 or limit_s <= 0:
        problems.append("race_m か limit_s が 0 以下")
    if any(not 0 <= float(cfg[k]) < lap_m for k in ("start_m", "control_m")):
        problems.append(f"start_m か control_m が 1 周の距離 {lap_m} m の外")
    band = cfg.get("margin_band_s")
    if band is not None and (len(band) != 2 or float(band[0]) > float(band[1])):
        problems.append("margin_band_s は [下限, 上限] で下限 ≦ 上限")
    plan, step = [float(t) for t in cfg["plan_t_s"]], float(cfg["plan_step_m"])
    if not 2 <= len(plan) <= MAX_PLAN_POINTS:
        problems.append(f"plan_t_s は 2〜{MAX_PLAN_POINTS} 個（{len(plan)} 個）")
    elif not (len(plan) - 2) * step < race_m <= (len(plan) - 1) * step + 0.5:
        problems.append("plan_t_s の個数が race_m と plan_step_m に合わない")
    if plan[0] < 0 or any(b <= a for a, b in zip(plan, plan[1:])):
        problems.append("plan_t_s が 0 以上の昇順でない")
    return problems


def check(cfg: dict, text: str) -> list[str]:
    """ファームが設定を採用しない条件（src/main.cpp の isValidStrategyConfig）に当たるものを返す。"""
    lap_m, guard = float(cfg["lap_m"]), float(cfg["guard_kmh"])
    problems = []
    if lap_m < 100 or guard < 0:
        problems.append("lap_m が 100 m 未満か guard_kmh が負")
    problems += check_pattern("通常パターン", cfg, lap_m, guard)
    if "fallback" in cfg:
        problems += check_pattern("予備パターン", cfg["fallback"], lap_m, guard)
    splits = cfg["splits_s"]
    if len(splits) > MAX_SPLITS:
        problems.append(f"splits_s は {MAX_SPLITS} 個まで（{len(splits)} 個）")
    if any(s <= 0 for s in splits) or any(b <= a for a, b in zip(splits, splits[1:])):
        problems.append("splits_s が正の昇順でない")
    problems += check_margin(cfg, lap_m)
    size = len(text.encode("utf-8"))
    if size > MAX_FILE_BYTES:
        problems.append(f"ファイルが大きすぎる（{size} バイト、上限 {MAX_FILE_BYTES}）")
    return problems


def dump(value, depth: int = 0) -> str:
    """配列を 1 行に収めた JSON 文字列にする。"""
    if isinstance(value, dict):
        pad = "  " * (depth + 1)
        body = ",\n".join(f"{pad}{json.dumps(k)}: {dump(v, depth + 1)}" for k, v in value.items())
        return "{\n" + body + "\n" + "  " * depth + "}"
    if isinstance(value, list):
        return "[" + ", ".join(dump(v) for v in value) + "]"
    if isinstance(value, Literal):
        return str(value)
    return json.dumps(value)


def changes(old: dict, new: dict) -> list[str]:
    def flat(d: dict, key: str) -> str:
        return " ".join(dump(d[key]).split()) if key in d else "（なし）"

    keys = list(new) + [k for k in old if k not in new]
    return [f"{k}: {flat(old, k)} → {flat(new, k)}" for k in keys if flat(old, k) != flat(new, k)]


def main() -> int:
    parser = argparse.ArgumentParser(description="走行支援の設定ファイルを ECU リポジトリの戦略パラメータから生成する")
    parser.add_argument("course", choices=list(COURSES), help="コース（出力は sd/<コース>_strategy.json）")
    parser.add_argument("--params", type=Path, metavar="JSON",
                        help="戦略パラメータ（省略時は <sim-dir>/strategy/<コース>_*.json のうち名前順で最後のもの）")
    parser.add_argument("-o", "--output", type=Path, help="出力先（省略時は sd/<コース>_strategy.json）")
    parser.add_argument("--sim-dir", type=Path, default=DEFAULT_SIM_DIR,
                        help="strategy_sim.py のあるディレクトリ（既定: %(default)s）")
    parser.add_argument("--target", help="目標時間 MM:SS（既定はパラメータの target_s）")
    parser.add_argument("--n-min", type=int, default=2, help="1 周あたり加速回数の下限")
    parser.add_argument("--n-max", type=int, default=5, help="1 周あたり加速回数の上限")
    parser.add_argument("--fuel-slack", type=float,
                        help="推奨を選ぶとき、燃料最小からこの割合までは加速回数の少なさを優先する"
                             "（既定はパラメータの fuel_slack、無ければ 0.05）")
    parser.add_argument("--fallback-v-off", type=float, help="予備パターンの加速停止速度 km/h（既定は既存ファイルの値）")
    parser.add_argument("--fallback-marks", type=float, nargs="+", metavar="M",
                        help="予備パターンの加速開始位置 m（既定は既存ファイルの値）")
    parser.add_argument("--fuel-warn-mpa", type=float, help="燃圧の警告しきい値 MPa（既定は既存ファイルの値）")
    parser.add_argument("--beep", action=argparse.BooleanOptionalAction, help="ブザーを鳴らすか（既定は既存ファイルの値）")
    parser.add_argument("--margin-band", type=float, nargs=2, metavar=("LOW", "HIGH"),
                        help="規定時間に対する余裕の目標の帯 秒（既定は既存ファイルの値、無ければファームの既定 10〜30）")
    parser.add_argument("--dry-run", action="store_true", help="書き込まずに結果だけ表示する")
    args = parser.parse_args()
    output = args.output or REPO / "sd" / f"{args.course}_strategy.json"

    try:
        sim = load_simulator(args.sim_dir)
        params = args.params or find_params(args.sim_dir, args.course)
        p = sim.load_params(str(params))
        check_version(f"{params.name} の format_version", p.get("format_version"), PARAMS_FORMAT_VERSION)
        notes = check_course(args.course, p)
        course = sim.course_from_params(p)
        existing = {}
        if output.exists():
            existing = json.loads(output.read_text(encoding="utf-8"), parse_float=Literal)

        limit = p["course"]["limit_s"]
        rec = plan(sim, p, course, args)
        if rec is not None:
            summary = [f"phases のプラン（探索なし） / 規定 {sim.mmss(limit)} / {p['course']['laps']} 周",
                       f"プランの通し: {sim.mmss(rec['time_min_s'])}〜{sim.mmss(rec['time_s'])}  加速 {rec['burns']} 回"]
            if rec["time_s"] > limit:
                summary.append(f"警告: プランは規定時間 {sim.mmss(limit)} を超えます")
        else:
            target = sim.parse_mmss(args.target) if args.target else p["target_s"]
            rec = recommend(sim, p, course, target, args.n_min, args.n_max, sim.fuel_slack(p, args.fuel_slack))
            if rec is None:
                raise ValueError(f"目標 {sim.mmss(target)} に収まるパターンがありません")
            summary = [f"目標 {sim.mmss(target)} / 規定 {sim.mmss(limit)} / {p['course']['laps']} 周",
                       f"通常パターン: {sim.mmss(rec['time_min_s'])}〜{sim.mmss(rec['time_s'])}  加速 {rec['burns']} 回"]
        cfg, build_notes = build(sim, p, course, rec, existing, args)
    except (OSError, ValueError) as e:
        print(f"エラー: {e}", file=sys.stderr)
        return 1
    except KeyError as e:
        print(f"エラー: パラメータに項目がありません: {e}", file=sys.stderr)
        return 1

    text = dump(cfg) + "\n"
    print(f"入力 {params}")
    for note in summary + build_notes + notes:
        print(note)
    print()
    print(text, end="")
    print()

    problems = check(cfg, text)
    if problems:
        for problem in problems:
            print(f"エラー: {problem}", file=sys.stderr)
        return 1

    if existing:
        diff = changes(existing, cfg)
        print("既存ファイルからの変更:" if diff else "既存ファイルからの変更はありません")
        for line in diff:
            print(f"  {line}")
    if args.dry_run:
        print(f"--dry-run のため {output} には書き込んでいません")
        return 0
    try:
        output.write_text(text, encoding="utf-8")
    except OSError as e:
        print(f"エラー: {e}", file=sys.stderr)
        return 1
    print(f"書き込みました: {output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
