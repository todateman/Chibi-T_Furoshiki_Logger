#!/usr/bin/env python3
"""ログCSVから停止状態の行を削除する。

エンジンと車体がともに停止している行（回転数(rpm) と 速度(km/h) の両方が 0）を
「停止行」とし、停止行が連続する区間の途中の行を削除する。
状態が切り替わる瞬間は分析の起点・終点になるので、次の行は残す。

  - 停止区間の最初の1行（車体停止の瞬間: 速度が 0 に落ちた行）
  - 停止区間の最後の1行（エンジン始動の瞬間: 回転数が上がる直前の行）

ファイル先頭から始まる停止区間は最後の1行だけ、ファイル末尾まで続く停止区間は
最初の1行だけが残る。残した行は元のバイト列のまま書き出すので、BOM・改行コード・
数値の桁数は変わらない。

使い方:
  python3 tools/remove_stopped_rows.py LOG0651.CSV              # → LOG0651_trimmed.CSV
  python3 tools/remove_stopped_rows.py LOG0650.CSV LOG0651.CSV  # 複数まとめて処理
  python3 tools/remove_stopped_rows.py LOG0651.CSV -o out.csv   # 出力先を指定
"""

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

BOM = b"\xef\xbb\xbf"
SPEED_COLUMN = "速度(km/h)"
RPM_COLUMN = "回転数(rpm)"
OUTPUT_SUFFIX = "_trimmed"


@dataclass
class Stats:
    total: int  # データ行数（ヘッダーを除く）
    removed: int  # 削除した行数
    unparsed: int  # 解釈できずそのまま残した行数

    @property
    def kept(self) -> int:
        return self.total - self.removed


def find_columns(header_line: bytes) -> tuple[int, int, int]:
    """ヘッダー行から (速度の列位置, 回転数の列位置, 列数) を返す。"""
    if header_line.startswith(BOM):
        header_line = header_line[len(BOM):]
    names = [name.strip() for name in header_line.decode("utf-8").split(",")]
    missing = [name for name in (SPEED_COLUMN, RPM_COLUMN) if name not in names]
    if missing:
        raise ValueError(f"ヘッダーに列がありません: {', '.join(missing)}")
    return names.index(SPEED_COLUMN), names.index(RPM_COLUMN), len(names)


def classify(line: bytes, speed_idx: int, rpm_idx: int, column_count: int) -> bool | None:
    """停止行なら True、それ以外は False、解釈できない行は None を返す。"""
    fields = line.split(b",")
    # 電源断で途切れた行は列数が足りない。値が欠けている可能性があるので判定しない
    if len(fields) != column_count:
        return None
    try:
        return float(fields[speed_idx]) == 0 and float(fields[rpm_idx]) == 0
    except ValueError:
        return None


def select_rows(states: list[bool | None]) -> list[bool]:
    """各行を残すかどうかを返す。停止区間は最初と最後の1行だけ残す。"""
    keep = [True] * len(states)
    # 解釈できない行は前後関係の判定から外す（隣の停止行を余分に残さないため）
    known = [i for i, state in enumerate(states) if state is not None]
    for pos, i in enumerate(known):
        if not states[i]:
            continue
        starts_section = pos > 0 and not states[known[pos - 1]]
        ends_section = pos + 1 < len(known) and not states[known[pos + 1]]
        keep[i] = starts_section or ends_section
    return keep


def filter_file(src: Path, dst: Path) -> Stats:
    lines = src.read_bytes().splitlines(keepends=True)
    if not lines:
        raise ValueError("ファイルが空です")
    speed_idx, rpm_idx, column_count = find_columns(lines[0])

    rows = lines[1:]
    states = [classify(row, speed_idx, rpm_idx, column_count) for row in rows]
    keep = select_rows(states)

    with dst.open("wb") as out:
        out.write(lines[0])
        out.writelines(row for row, kept in zip(rows, keep) if kept)

    return Stats(total=len(rows), removed=keep.count(False), unparsed=states.count(None))


def main() -> int:
    parser = argparse.ArgumentParser(description="ログCSVから停止状態（速度と回転数がともに0）の行を削除する")
    parser.add_argument("inputs", nargs="+", type=Path, metavar="CSV", help="入力するログCSV")
    parser.add_argument("-o", "--output", type=Path, help=f"出力先（省略時は <入力名>{OUTPUT_SUFFIX}.<拡張子>。入力が1件のときのみ指定可）")
    args = parser.parse_args()

    if args.output and len(args.inputs) > 1:
        parser.error("-o は入力が1件のときだけ指定できます")

    failed = False
    for src in args.inputs:
        dst = args.output or src.with_name(f"{src.stem}{OUTPUT_SUFFIX}{src.suffix}")
        try:
            if dst.resolve() == src.resolve():
                raise ValueError("出力先が入力と同じです（元ファイルは上書きしません）")
            stats = filter_file(src, dst)
        except (OSError, ValueError) as e:
            print(f"{src}: エラー: {e}", file=sys.stderr)
            failed = True
            continue

        print(f"{src} -> {dst}")
        print(f"  入力 {stats.total} 行 / 削除 {stats.removed} 行 / 出力 {stats.kept} 行")
        if stats.unparsed:
            print(f"  解釈できない行を {stats.unparsed} 行そのまま残しました")

    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
