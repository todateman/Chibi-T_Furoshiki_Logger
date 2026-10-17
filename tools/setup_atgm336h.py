#!/usr/bin/env python3
"""GNSSモジュール ATGM336H を、ロガーがそのまま読める設定にしてモジュールへ保存する。

ロガーは Serial2 を 115200 bps で開いて NMEA を読むだけで、モジュールへ設定は送らない。
出荷状態の ATGM336H は 9600 bps・1 Hz・全センテンス出力なので、そのままでは読めない。
モジュールを USB シリアル変換でPCにつなぎ、次の設定を書き込む。

  - ボーレート 115200 bps（src/main.cpp の gnss_bps と同じ）
  - 更新レート 10 Hz
  - 出力センテンスは RMC のみ（ロガーが使う時刻・日付・緯度経度・速度はすべて RMC に入っている）

現在のボーレートは受信して自動で判定するので、設定済みのモジュールにもそのまま使える。
書き込んだあとは実際の出力を数秒受信し、設定どおりになっているかを確かめる。

使い方:
  python3 tools/setup_atgm336h.py /dev/cu.usbmodemXXXX                       # 設定して保存、結果を確認
  python3 tools/setup_atgm336h.py /dev/cu.usbmodemXXXX --verify-only         # 現在の出力を確認するだけ
  python3 tools/setup_atgm336h.py /dev/cu.usbmodemXXXX --rate-hz 5           # 更新レートを変える
  python3 tools/setup_atgm336h.py /dev/cu.usbmodemXXXX --sentences gga-rmc   # GGA も出す（衛星数・HDOP・高度）
"""

import argparse
import re
import sys
import time
from collections import Counter
from dataclasses import dataclass

try:
    import serial
except ImportError:
    sys.exit("pyserial が必要です: python3 -m pip install pyserial")

LOGGER_BAUD = 115200  # src/main.cpp の gnss_bps
LOGGER_BAUD_CODE = 5  # $PCAS01 の引数（0:4800 1:9600 2:19200 3:38400 4:57600 5:115200）
# 判定で試す順。出荷状態(9600)と設定済み(115200)を先に見る
BAUD_CANDIDATES = (9600, 115200, 38400, 57600, 19200, 4800)

# 更新レート[Hz] → $PCAS02 の測位間隔[ms]
RATE_INTERVAL_MS = {1: 1000, 2: 500, 4: 250, 5: 200, 10: 100}

# $PCAS03 の引数: GGA,GLL,GSA,GSV,RMC,VTG,ZDA,ANT,DHV,LPS,(予約),(予約),UTC,GST
# 値は「何測位ごとに出すか」で、0 は出力しない
SENTENCE_SETS = {
    "rmc": ("0,0,0,0,1,0,0,0,0,0,,,0,0", {"RMC"}),
    "gga-rmc": ("1,0,0,0,1,0,0,0,0,0,,,0,0", {"GGA", "RMC"}),
}

# 1 Hz・RMC のみの出力でも複数行を受信できる長さにする（先頭と末尾の行は数えないため）
DETECT_SECONDS = 3.5
VERIFY_SECONDS = 5.0
COMMAND_WAIT_SECONDS = 0.3
RATE_TOLERANCE = 0.15  # 行数から求めた更新レートの許容誤差（割合）

SENTENCE_PATTERN = re.compile(rb"^\$([A-Z]{2}[A-Z0-9]{3}),?([\x20-\x7e]*)\*([0-9A-Fa-f]{2})$")


@dataclass
class Capture:
    seconds: float
    sentences: list[str]  # チェックサムの合った行（"$" と "*XX" を含む）
    broken: int  # 行の形はしているがチェックサムが合わない、または形が崩れている行数

    def counts(self) -> Counter:
        """センテンスの種類（トーカーIDを除いた3文字）ごとの行数を返す。"""
        return Counter(line[3:6] for line in self.sentences)


def checksum(body: str) -> int:
    """"$" と "*" のあいだの文字列のXORを返す。"""
    value = 0
    for byte in body.encode("ascii"):
        value ^= byte
    return value


def build_command(body: str) -> bytes:
    return f"${body}*{checksum(body):02X}\r\n".encode("ascii")


def parse(buffer: bytes, seconds: float) -> Capture:
    pieces = buffer.split(b"\n")
    sentences: list[str] = []
    broken = 0
    # 先頭は受信開始時点で途中から始まり、末尾は受信終了時点で途切れているので数えない
    for piece in pieces[1:-1]:
        line = piece.rstrip(b"\r")
        if not line:
            continue
        matched = SENTENCE_PATTERN.match(line)
        if matched and checksum(line[1:-3].decode("ascii")) == int(matched.group(3), 16):
            sentences.append(line.decode("ascii"))
        else:
            broken += 1
    return Capture(seconds=seconds, sentences=sentences, broken=broken)


def listen(port: serial.Serial, seconds: float) -> Capture:
    port.reset_input_buffer()
    started = time.monotonic()
    buffer = bytearray()
    while time.monotonic() - started < seconds:
        buffer += port.read(4096)
    return parse(bytes(buffer), time.monotonic() - started)


def detect_baud(port: serial.Serial) -> int | None:
    """各ボーレートで受信し、チェックサムの合う行が来るボーレートを返す。"""
    for baud in BAUD_CANDIDATES:
        port.baudrate = baud
        capture = listen(port, DETECT_SECONDS)
        if len(capture.sentences) >= 2 and len(capture.sentences) > capture.broken:
            return baud
    return None


def send(port: serial.Serial, body: str, note: str) -> None:
    command = build_command(body)
    print(f"  {command.decode('ascii').strip():<40} {note}")
    port.write(command)
    port.flush()
    time.sleep(COMMAND_WAIT_SECONDS)


def configure(port: serial.Serial, current_baud: int, rate_hz: int, sentences: str) -> None:
    # 9600 bps のまま更新レートを上げると全センテンスを送りきれないので、先に出力を絞る
    send(port, f"PCAS03,{SENTENCE_SETS[sentences][0]}", f"出力センテンス: {sentences}")
    if current_baud != LOGGER_BAUD:
        send(port, f"PCAS01,{LOGGER_BAUD_CODE}", f"ボーレート: {LOGGER_BAUD} bps")
        port.baudrate = LOGGER_BAUD
        if not listen(port, DETECT_SECONDS).sentences:
            raise RuntimeError(f"{LOGGER_BAUD} bps へ切り替えたあと、NMEA を受信できません")
    send(port, f"PCAS02,{RATE_INTERVAL_MS[rate_hz]}", f"更新レート: {rate_hz} Hz")
    send(port, "PCAS00", "設定をモジュールへ保存")


def verify(port: serial.Serial, baud: int, rate_hz: int, sentences: str) -> bool:
    """実際の出力を受信し、ロガーが読める設定になっているかを表示して返す。"""
    capture = listen(port, VERIFY_SECONDS)
    counts = capture.counts()
    expected = SENTENCE_SETS[sentences][1]
    # TXT は起動直後の通知（バージョン、アンテナ状態）で、設定の対象外
    actual = set(counts) - {"TXT"}
    measured_hz = counts["RMC"] / capture.seconds

    checks = [
        ("ボーレート", f"{baud} bps", baud == LOGGER_BAUD),
        ("出力センテンス", ", ".join(sorted(actual)) or "なし", actual == expected),
        ("更新レート", f"{measured_hz:.1f} Hz", abs(measured_hz - rate_hz) <= max(0.5, rate_hz * RATE_TOLERANCE)),
        ("壊れた行", f"{capture.broken} 行", capture.broken == 0),
    ]
    for name, value, ok in checks:
        print(f"  {'OK' if ok else 'NG'}  {name}: {value}")

    rmc = [line.split(",") for line in capture.sentences if line[3:6] == "RMC"]
    if rmc:
        fixed = sum(1 for fields in rmc if len(fields) > 2 and fields[2] == "A")
        print(f"  --  測位: {'あり' if fixed else 'なし（屋外でアンテナをつないで確認する）'}")
    for line in dict.fromkeys(line for line in capture.sentences if line[3:6] == "TXT"):
        print(f"  --  {line}")
    return all(ok for _, _, ok in checks)


def main() -> int:
    parser = argparse.ArgumentParser(description="ATGM336H をロガー用（115200 bps）に設定してモジュールへ保存する")
    parser.add_argument("port", help="モジュールをつないだシリアルポート（例: /dev/cu.usbmodemXXXX）")
    parser.add_argument("--rate-hz", type=int, choices=sorted(RATE_INTERVAL_MS), default=10, help="更新レート [Hz]（既定 10）")
    parser.add_argument("--sentences", choices=sorted(SENTENCE_SETS), default="rmc", help="出力センテンス（既定 rmc）")
    parser.add_argument("--verify-only", action="store_true", help="設定を書き込まず、現在の出力が設定どおりかを確認する")
    args = parser.parse_args()

    try:
        with serial.Serial(args.port, BAUD_CANDIDATES[0], timeout=0.1) as port:
            print("ボーレートを判定中...")
            baud = detect_baud(port)
            if baud is None:
                raise RuntimeError("NMEA を受信できません（配線と電源、ポート名を確認する）")
            print(f"  現在 {baud} bps")

            if not args.verify_only:
                print("設定を書き込み中...")
                configure(port, baud, args.rate_hz, args.sentences)
                baud = LOGGER_BAUD

            print(f"出力を確認中（{VERIFY_SECONDS:.0f} 秒）...")
            port.baudrate = baud
            ok = verify(port, baud, args.rate_hz, args.sentences)
    except (OSError, RuntimeError) as e:
        print(f"{args.port}: エラー: {e}", file=sys.stderr)
        return 1

    if not ok:
        print("設定どおりになっていません", file=sys.stderr)
        return 1
    if not args.verify_only:
        print("完了。電源を入れ直してから --verify-only で設定が残っていることを確認してください")
    return 0


if __name__ == "__main__":
    sys.exit(main())
