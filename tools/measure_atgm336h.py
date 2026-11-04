#!/usr/bin/env python3
"""ATGM336H が測位中に設定どおりの更新レートで RMC を出しているかを測る（#28）。

setup_atgm336h.py の確認は 5 秒しか受信しないので、測位して数分たってから回が抜ける個体を見逃す。
このスクリプトは指定した時間だけ受信し、次を集計する。

  - 平均の更新レートと、一定時間ごとの更新レート（時間とともに落ちるか）
  - RMC の時刻の間隔の内訳（10 Hz なら 0.1 秒だけのはず。0.2 秒があればモジュールが回を飛ばしている）
  - PC に届いた間隔の最大と、0.7 秒以上止まった回数
  - 測位の状態（A / V）、使用衛星数（GGA を出しているときだけ）、壊れた行の数

--rate-hz と --sentences は、測る前に送るだけで保存しない（$PCAS00 を送らない）。
電源を入れ直せば、モジュールに保存してある設定に戻る。
--query は $PCAS06 でファームウェアなどを問い合わせる。設定は変えない。

使い方:
  python3 tools/measure_atgm336h.py /dev/cu.usbmodemXXXX --query                        # ファームウェアを問い合わせる
  python3 tools/measure_atgm336h.py /dev/cu.usbmodemXXXX --seconds 600                  # 今の設定のまま 10 分測る
  python3 tools/measure_atgm336h.py /dev/cu.usbmodemXXXX --seconds 120 --sentences gga-rmc  # GGA を足して衛星数も見る
  python3 tools/measure_atgm336h.py /dev/cu.usbmodemXXXX --seconds 240 --rate-hz 10      # 保存せずに 10 Hz にして測る
  python3 tools/measure_atgm336h.py /dev/cu.usbmodemXXXX --seconds 600 --log run.log     # 受信した行を到着時刻つきで残す
"""

import argparse
import sys
import time
from collections import Counter
from collections.abc import Iterator
from dataclasses import dataclass, field

try:
    import serial
except ImportError:
    sys.exit("pyserial が必要です: python3 -m pip install pyserial")

from setup_atgm336h import (
    BAUD_CANDIDATES,
    RATE_INTERVAL_MS,
    SENTENCE_PATTERN,
    SENTENCE_SETS,
    build_command,
    checksum,
    detect_baud,
)

# $PCAS06 の引数と、応答の意味
QUERIES = (("SW", "ファームウェア"), ("HW", "ハードウェア"), ("MO", "衛星システム"), ("CI", "顧客番号"))
QUERY_SECONDS = 2.0
SETTLE_SECONDS = 1.0  # 一時的な設定を送ってから、出力が切り替わるまで待つ時間
STALL_SECONDS = 0.7  # これ以上 RMC が来なければ「止まった」と数える（決勝では 0.7〜0.9 秒の無音が出た）
SECONDS_PER_DAY = 86400


@dataclass
class Result:
    rmc_arrivals: list[float] = field(default_factory=list)  # RMC が届いた時刻（受信開始からの秒）
    rmc_times: list[float] = field(default_factory=list)  # RMC に入っている UTC 時刻（0 時からの秒）
    rmc_lengths: list[int] = field(default_factory=list)  # 改行を含むバイト数
    status: Counter = field(default_factory=Counter)  # RMC の測位状態（A / V）ごとの行数
    satellites: list[tuple[float, int]] = field(default_factory=list)  # (届いた時刻, GGA の使用衛星数)
    broken: int = 0


def is_valid(line: bytes) -> bool:
    matched = SENTENCE_PATTERN.match(line)
    return bool(matched) and checksum(line[1:-3].decode("ascii")) == int(matched.group(3), 16)


def read_lines(port: serial.Serial, seconds: float, skip_first: bool = True) -> Iterator[tuple[float, bytes]]:
    """受信開始からの秒と行を返す。

    skip_first のときは、受信開始時点で途中から始まっているかもしれない最初の行を捨てる。
    """
    started = time.monotonic()
    buffer = b""
    first = skip_first
    while (now := time.monotonic()) - started < seconds:
        buffer += port.read(4096)
        while b"\n" in buffer:
            line, buffer = buffer.split(b"\n", 1)
            if first:
                first = False
                continue
            yield now - started, line.rstrip(b"\r")


def query(port: serial.Serial) -> None:
    for index, (name, meaning) in enumerate(QUERIES):
        port.write(build_command(f"PCAS06,{index}"))
        port.flush()
        # 応答は $GPTXT で、送ってから 0.1 秒以内に返る。最初の行が応答のことがあるので捨てない
        # （途中から始まった行はチェックサムが合わないので、is_valid で落ちる）。受信バッファも捨てない
        replies = [
            line.decode("ascii")
            for _, line in read_lines(port, QUERY_SECONDS, skip_first=False)
            if is_valid(line) and line[3:6] == b"TXT" and b"ANTENNA" not in line
        ]
        print(f"  {name}（{meaning}）: {' | '.join(dict.fromkeys(replies)) or '応答なし'}")


def utc_seconds(text: str) -> float | None:
    try:
        return int(text[0:2]) * 3600 + int(text[2:4]) * 60 + float(text[4:])
    except (ValueError, IndexError):
        return None


def capture(port: serial.Serial, seconds: float, log_path: str | None) -> Result:
    result = Result()
    log = open(log_path, "w", encoding="ascii", errors="replace") if log_path else None
    try:
        for arrived, line in read_lines(port, seconds):
            if log:
                log.write(f"{arrived:.4f}\t{line.decode('ascii', 'replace')}\n")
            if not line:
                continue
            if not is_valid(line):
                result.broken += 1
                continue
            fields = line.decode("ascii").split(",")
            kind = line[3:6]
            if kind == b"RMC":
                result.rmc_arrivals.append(arrived)
                result.rmc_lengths.append(len(line) + 2)
                if (utc := utc_seconds(fields[1])) is not None:
                    result.rmc_times.append(utc)
                result.status[fields[2] if len(fields) > 2 else "?"] += 1
            elif kind == b"GGA" and len(fields) > 7 and fields[7].isdigit():
                result.satellites.append((arrived, int(fields[7])))
    finally:
        if log:
            log.close()
    return result


def report(result: Result, seconds: float, window: float) -> None:
    arrivals = result.rmc_arrivals
    print(f"  RMC {len(arrivals)} 行、壊れた行 {result.broken} 行")
    if len(arrivals) < 2:
        print("  RMC が 2 行未満なので集計できません")
        return

    print(f"  平均の更新レート: {(len(arrivals) - 1) / (arrivals[-1] - arrivals[0]):.2f} Hz")

    gaps = [later - earlier for earlier, later in zip(arrivals, arrivals[1:])]
    stalls = sum(gap >= STALL_SECONDS for gap in gaps)
    print(f"  届いた間隔: 最大 {max(gaps) * 1000:.0f} ms、{STALL_SECONDS} 秒以上の停止 {stalls} 回")

    steps = Counter()
    for earlier, later in zip(result.rmc_times, result.rmc_times[1:]):
        step = later - earlier
        if step < -SECONDS_PER_DAY / 2:  # UTC の 0 時をまたいだ
            step += SECONDS_PER_DAY
        steps[round(step, 1)] += 1
    print("  RMC の時刻の間隔: " + "、".join(f"{step:.1f} 秒 {count} 回" for step, count in sorted(steps.items())))

    print("  測位の状態: " + "、".join(f"{state} {count} 行" for state, count in sorted(result.status.items())))
    if result.satellites:
        counts = sorted(count for _, count in result.satellites)
        print(f"  使用衛星数: {counts[0]}〜{counts[-1]}（中央値 {counts[len(counts) // 2]}）")
    print(f"  RMC の長さ: {min(result.rmc_lengths)}〜{max(result.rmc_lengths)} バイト")

    print(f"  {window:.0f} 秒ごとの更新レート:")
    start = 0.0
    while start < seconds:
        end = start + window
        rate = sum(start <= t < end for t in arrivals) / min(window, seconds - start)
        in_window = sorted(count for t, count in result.satellites if start <= t < end)
        satellites = f"、衛星 {in_window[0]}〜{in_window[-1]}" if in_window else ""
        print(f"    {start:5.0f}〜{min(end, seconds):5.0f} 秒: {rate:5.2f} Hz{satellites}")
        start = end


def main() -> int:
    parser = argparse.ArgumentParser(description="ATGM336H の測位中の更新レートを測る（設定は保存しない）")
    parser.add_argument("port", help="モジュールをつないだシリアルポート（例: /dev/cu.usbmodemXXXX）")
    parser.add_argument("--query", action="store_true", help="$PCAS06 でファームウェアなどを問い合わせて終わる")
    parser.add_argument("--seconds", type=float, default=240, help="受信する時間 [秒]（既定 240）")
    parser.add_argument("--window", type=float, default=20, help="更新レートを区切って表示する間隔 [秒]（既定 20）")
    parser.add_argument("--rate-hz", type=int, choices=sorted(RATE_INTERVAL_MS), help="測る前に送る更新レート（保存しない）")
    parser.add_argument("--sentences", choices=sorted(SENTENCE_SETS), help="測る前に送る出力センテンス（保存しない）")
    parser.add_argument("--log", help="受信した行を到着時刻つきで書き出すファイル")
    args = parser.parse_args()

    try:
        with serial.Serial(args.port, BAUD_CANDIDATES[0], timeout=0.05) as port:
            print("ボーレートを判定中...")
            baud = detect_baud(port)
            if baud is None:
                raise RuntimeError("NMEA を受信できません（配線と電源、ポート名を確認する）")
            port.baudrate = baud
            print(f"  現在 {baud} bps")
            port.reset_input_buffer()

            if args.query:
                print("問い合わせ中...")
                query(port)
                return 0

            if args.sentences or args.rate_hz:
                print("一時的な設定を送信中（保存しない。電源を入れ直すと元に戻る）...")
                if args.sentences:
                    port.write(build_command(f"PCAS03,{SENTENCE_SETS[args.sentences][0]}"))
                    print(f"  出力センテンス: {args.sentences}")
                if args.rate_hz:
                    port.write(build_command(f"PCAS02,{RATE_INTERVAL_MS[args.rate_hz]}"))
                    print(f"  更新レート: {args.rate_hz} Hz")
                port.flush()
                time.sleep(SETTLE_SECONDS)
                port.reset_input_buffer()

            print(f"受信中（{args.seconds:.0f} 秒）...")
            result = capture(port, args.seconds, args.log)
    except (OSError, RuntimeError) as e:
        print(f"{args.port}: エラー: {e}", file=sys.stderr)
        return 1

    report(result, args.seconds, args.window)
    return 0


if __name__ == "__main__":
    sys.exit(main())
