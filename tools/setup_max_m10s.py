#!/usr/bin/env python3
"""GNSSモジュール MAX-M10S（u-blox M10）を、ロガーがそのまま読める設定にしてモジュールへ保存する（#28）。

ロガーは Serial2 を 115200 bps で開いて NMEA を読むだけで、モジュールへ設定は送らない。
出荷状態の MAX-M10S は 38400 bps・1 Hz・6 種類のセンテンス出力なので、そのままでは読めない。
モジュールを USB シリアル変換でPCにつなぎ、UBX の UBX-CFG-VALSET で次の設定を書き込む。

  - ボーレート 115200 bps（src/main.cpp の gnss_bps と同じ）
  - 更新レート 10 Hz
  - 出力センテンスは RMC のみ
  - 衛星システムは GPS+Galileo（QZSS と SBAS も使う。BeiDou と GLONASS は使わない）

出荷時の GPS+Galileo+BeiDou では 10 Hz の計算が間に合わず、ベランダで約 9 Hz に落ちた。
データシートの最大は GPS+Galileo+BeiDou が 3 Hz、GPS+Galileo が 10 Hz、GPS のみが 18 Hz。
これより上げる「高性能モード」は OTP（1 回しか書き込めない）への書き込みが要るので、このスクリプトでは扱わない。

MAX-M10S にはフラッシュがない。設定は RAM と BBR（V_BCKP の電池で保持する RAM）に書く。
電池の載った基板なら電源を入れ直しても残るが、電池が切れると出荷時の設定（38400 bps・1 Hz）に戻る。

現在のボーレートは受信して自動で判定するので、設定済みのモジュールにもそのまま使える。
書き込んだあとは実際の出力を数秒受信し、RAM と BBR の値も読み出して、設定どおりかを確かめる。

使い方:
  python3 tools/setup_max_m10s.py /dev/cu.usbmodemXXXX                       # 設定して保存、結果を確認
  python3 tools/setup_max_m10s.py /dev/cu.usbmodemXXXX --verify-only         # 現在の出力と保存した設定を確認するだけ
  python3 tools/setup_max_m10s.py /dev/cu.usbmodemXXXX --query               # ファームウェアと RAM・BBR の設定を表示する
  python3 tools/setup_max_m10s.py /dev/cu.usbmodemXXXX --sentences gga-rmc   # GGA も出す（衛星数・HDOP・高度）
  python3 tools/setup_max_m10s.py /dev/cu.usbmodemXXXX --gnss gps            # 衛星システムを GPS のみにする
"""

import argparse
import struct
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial が必要です: python3 -m pip install pyserial")

from setup_atgm336h import (
    DETECT_SECONDS,
    LOGGER_BAUD,
    RATE_INTERVAL_MS,
    RATE_TOLERANCE,
    VERIFY_SECONDS,
    detect_baud,
    listen,
)

# 設定キー（u-blox M10 SPG 5.10 のインタフェース記述）。値の大きさはキーの 28〜30 ビットで決まる
KEY_RATE_MEAS = 0x30210001  # 測位間隔 [ms]
KEY_RATE_NAV = 0x30210002  # 何回の測定ごとに測位するか
KEY_UART1_BAUD = 0x40520001
KEY_GPS = 0x1031001F
KEY_SBAS = 0x10310020
KEY_GAL = 0x10310021
KEY_BDS = 0x10310022
KEY_QZSS = 0x10310024
KEY_GLO = 0x10310025
KEY_NMEA = {  # UART1 へ何測位ごとに出すか（0 は出さない）
    "GGA": 0x209100BB,
    "GLL": 0x209100CA,
    "GSA": 0x209100C0,
    "GSV": 0x209100C5,
    "RMC": 0x209100AC,
    "VTG": 0x209100B1,
}
VALUE_SIZE = {1: 1, 2: 1, 3: 2, 4: 4, 5: 8}  # キーの 28〜30 ビット → 値のバイト数

LAYER_RAM = 0x01  # UBX-CFG-VALSET の layers（ビットの和）
LAYER_BBR = 0x02
GET_LAYER_RAM = 0  # UBX-CFG-VALGET の layer
GET_LAYER_BBR = 1

SENTENCE_SETS = {
    "rmc": {"RMC"},
    "gga-rmc": {"GGA", "RMC"},
}

# 衛星システムの名前 → (GPS, SBAS, Galileo, BeiDou, QZSS, GLONASS)
# SBAS と QZSS は GPS を有効にしたときだけ使える
GNSS_SETS = {
    "gps-gal": (1, 1, 1, 0, 1, 0),
    "gps": (1, 1, 0, 0, 1, 0),
    "gps-gal-bds": (1, 1, 1, 1, 1, 0),  # 出荷時と同じ。10 Hz を保てない
}
GNSS_KEYS = (KEY_GPS, KEY_SBAS, KEY_GAL, KEY_BDS, KEY_QZSS, KEY_GLO)

UBX_SECONDS = 1.0  # UBX の応答を待つ時間（10 Hz の NMEA に混ざって届く）
GNSS_RESTART_SECONDS = 3.0  # 衛星システムを変えると GNSS を再起動するので、出力が戻るまで待つ


def ubx_frame(cls: int, msg_id: int, payload: bytes = b"") -> bytes:
    body = bytes([cls, msg_id]) + struct.pack("<H", len(payload)) + payload
    a = b = 0
    for byte in body:
        a = (a + byte) & 0xFF
        b = (b + a) & 0xFF
    return b"\xb5\x62" + body + bytes([a, b])


def ubx_messages(buffer: bytes) -> list[tuple[int, int, bytes]]:
    """受信したバイト列から、チェックサムの合う UBX メッセージ（クラス, ID, 中身）を取り出す。"""
    found = []
    start = buffer.find(b"\xb5\x62")
    while start >= 0 and start + 6 <= len(buffer):
        length = struct.unpack("<H", buffer[start + 4 : start + 6])[0]
        end = start + 8 + length
        if end <= len(buffer):
            frame = buffer[start:end]
            if ubx_frame(frame[2], frame[3], frame[6:-2]) == frame:
                found.append((frame[2], frame[3], frame[6:-2]))
                start = buffer.find(b"\xb5\x62", end)
                continue
        start = buffer.find(b"\xb5\x62", start + 2)
    return found


def ubx_request(port: serial.Serial, frame: bytes, wanted: set[tuple[int, int]]) -> list[tuple[int, int, bytes]]:
    """frame を送り、wanted のクラスと ID の応答が届くまで（最大 UBX_SECONDS）受信して返す。"""
    port.reset_input_buffer()
    port.write(frame)
    port.flush()
    started = time.monotonic()
    buffer = bytearray()
    while time.monotonic() - started < UBX_SECONDS:
        buffer += port.read(4096)
        messages = ubx_messages(bytes(buffer))
        if any((cls, msg_id) in wanted for cls, msg_id, _ in messages):
            return messages
    return ubx_messages(bytes(buffer))


def value_size(key: int) -> int:
    return VALUE_SIZE[(key >> 28) & 0x7]


def valset(port: serial.Serial, values: dict[int, int], layers: int) -> bool:
    """UBX-CFG-VALSET で values を layers に書き込む。ACK が返れば True。"""
    payload = struct.pack("<BBH", 0, layers, 0)
    for key, value in values.items():
        payload += struct.pack("<I", key) + value.to_bytes(value_size(key), "little")
    for cls, msg_id, body in ubx_request(port, ubx_frame(0x06, 0x8A, payload), {(0x05, 0x01), (0x05, 0x00)}):
        if cls == 0x05 and body[:2] == b"\x06\x8a":
            return msg_id == 0x01
    return False


def valget(port: serial.Serial, keys: list[int], layer: int) -> dict[int, int] | None:
    """UBX-CFG-VALGET で keys の値を読む。応答がなければ None、層に値がなければ空の dict。"""
    payload = struct.pack("<BBH", 0, layer, 0) + b"".join(struct.pack("<I", key) for key in keys)
    for cls, msg_id, body in ubx_request(port, ubx_frame(0x06, 0x8B, payload), {(0x06, 0x8B), (0x05, 0x00)}):
        if (cls, msg_id) == (0x06, 0x8B):
            values = {}
            data = body[4:]
            while len(data) >= 4:
                key = struct.unpack("<I", data[:4])[0]
                size = value_size(key)
                values[key] = int.from_bytes(data[4 : 4 + size], "little")
                data = data[4 + size :]
            return values
        if cls == 0x05 and msg_id == 0x00 and body[:2] == b"\x06\x8b":
            return {}  # BBR に何も保存していないと NAK が返る
    return None


def expected_values(rate_hz: int, sentences: str, gnss: str) -> dict[int, int]:
    values = {KEY_RATE_MEAS: RATE_INTERVAL_MS[rate_hz], KEY_RATE_NAV: 1}
    values.update({key: int(name in SENTENCE_SETS[sentences]) for name, key in KEY_NMEA.items()})
    values.update(dict(zip(GNSS_KEYS, GNSS_SETS[gnss])))
    return values


def describe(values: dict[int, int]) -> str:
    """読み出した値を、人が読める形にする。"""
    parts = []
    if KEY_UART1_BAUD in values:
        parts.append(f"{values[KEY_UART1_BAUD]} bps")
    if KEY_RATE_MEAS in values:
        parts.append(f"{1000 / values[KEY_RATE_MEAS] / max(values.get(KEY_RATE_NAV, 1), 1):g} Hz")
    nmea = [name for name, key in KEY_NMEA.items() if values.get(key)]
    if any(key in values for key in KEY_NMEA.values()):
        parts.append("出力: " + (", ".join(nmea) or "なし"))
    names = dict(zip(GNSS_KEYS, ("GPS", "SBAS", "Galileo", "BeiDou", "QZSS", "GLONASS")))
    if any(key in values for key in GNSS_KEYS):
        parts.append("衛星システム: " + ("+".join(name for key, name in names.items() if values.get(key)) or "なし"))
    return "、".join(parts) or "値なし"


ALL_KEYS = [KEY_UART1_BAUD, KEY_RATE_MEAS, KEY_RATE_NAV, *KEY_NMEA.values(), *GNSS_KEYS]


def c_string(chunk: bytes) -> str:
    return chunk.rstrip(b"\0").decode("ascii", "replace")


def query(port: serial.Serial) -> None:
    """UBX-MON-VER でファームウェアを、UBX-CFG-VALGET で RAM と BBR の設定を表示する。設定は変えない。"""
    for cls, msg_id, body in ubx_request(port, ubx_frame(0x0A, 0x04), {(0x0A, 0x04)}):
        if (cls, msg_id) == (0x0A, 0x04):
            print(f"  SW（ファームウェア）: {c_string(body[:30])}")
            print(f"  HW: {c_string(body[30:40])}")
            for start in range(40, len(body), 30):
                print(f"    {c_string(body[start : start + 30])}")
            break
    else:
        print("  UBX-MON-VER: 応答なし")
    for layer, name in ((GET_LAYER_RAM, "RAM（今の設定）"), (GET_LAYER_BBR, "BBR（保存した設定）")):
        values = valget(port, ALL_KEYS, layer)
        print(f"  {name}: " + ("応答なし" if values is None else describe(values) if values else "保存なし"))


def configure(port: serial.Serial, current_baud: int, rate_hz: int, sentences: str, gnss: str) -> None:
    layers = LAYER_RAM | LAYER_BBR
    # 38400 bps のまま更新レートを上げると全センテンスを送りきれないので、出力の変更と一緒に 1 回で送る
    ok = valset(port, expected_values(rate_hz, sentences, gnss), layers)
    print(f"  {'ACK' if ok else 'NAK'}  更新レート {rate_hz} Hz・出力 {sentences}・衛星システム {gnss}（RAM と BBR）")
    if not ok:
        raise RuntimeError("設定を受け付けませんでした")
    time.sleep(GNSS_RESTART_SECONDS)

    # ボーレートは最後に変える。すでに 115200 bps でも、BBR に残すために書く
    if current_baud == LOGGER_BAUD:
        ok = valset(port, {KEY_UART1_BAUD: LOGGER_BAUD}, layers)
        print(f"  {'ACK' if ok else 'NAK'}  ボーレート {LOGGER_BAUD} bps（RAM と BBR）")
    else:
        # 新しいボーレートに切り替わるので、ACK は受け取れないことがある。読み出して確かめる
        payload = struct.pack("<BBH", 0, layers, 0) + struct.pack("<II", KEY_UART1_BAUD, LOGGER_BAUD)
        port.write(ubx_frame(0x06, 0x8A, payload))
        port.flush()
        time.sleep(0.3)
        port.baudrate = LOGGER_BAUD
        if not listen(port, DETECT_SECONDS).sentences:
            raise RuntimeError(f"{LOGGER_BAUD} bps へ切り替えたあと、NMEA を受信できません")
        print(f"  --   ボーレート {LOGGER_BAUD} bps（RAM と BBR）")


def verify(port: serial.Serial, baud: int, rate_hz: int, sentences: str, gnss: str) -> bool:
    """実際の出力と、RAM・BBR に入っている値を確かめて表示する。"""
    capture = listen(port, VERIFY_SECONDS)
    counts = capture.counts()
    actual = set(counts) - {"TXT"}
    measured_hz = counts["RMC"] / capture.seconds
    expected = expected_values(rate_hz, sentences, gnss) | {KEY_UART1_BAUD: LOGGER_BAUD}
    saved = valget(port, ALL_KEYS, GET_LAYER_BBR)

    checks = [
        ("ボーレート", f"{baud} bps", baud == LOGGER_BAUD),
        ("出力センテンス", ", ".join(sorted(actual)) or "なし", actual == SENTENCE_SETS[sentences]),
        ("更新レート", f"{measured_hz:.1f} Hz", abs(measured_hz - rate_hz) <= max(0.5, rate_hz * RATE_TOLERANCE)),
        ("壊れた行", f"{capture.broken} 行", capture.broken == 0),
        ("保存した設定（BBR）", "応答なし" if saved is None else describe(saved) if saved else "保存なし",
         saved is not None and all(saved.get(key) == value for key, value in expected.items())),
    ]
    current = valget(port, list(GNSS_KEYS), GET_LAYER_RAM)
    checks.append(("衛星システム（RAM）", "応答なし" if current is None else describe(current),
                   current is not None and all(current.get(key) == expected[key] for key in GNSS_KEYS)))
    for name, value, ok in checks:
        print(f"  {'OK' if ok else 'NG'}  {name}: {value}")

    rmc = [line.split(",") for line in capture.sentences if line[3:6] == "RMC"]
    if rmc:
        fixed = sum(1 for fields in rmc if len(fields) > 2 and fields[2] == "A")
        print(f"  --  測位: {'あり' if fixed else 'なし（屋外でアンテナをつないで確認する）'}")
    return all(ok for _, _, ok in checks)


def main() -> int:
    parser = argparse.ArgumentParser(description="MAX-M10S をロガー用（115200 bps）に設定して RAM と BBR へ保存する")
    parser.add_argument("port", help="モジュールをつないだシリアルポート（例: /dev/cu.usbmodemXXXX）")
    parser.add_argument("--rate-hz", type=int, choices=sorted(RATE_INTERVAL_MS), default=10, help="更新レート [Hz]（既定 10）")
    parser.add_argument("--sentences", choices=sorted(SENTENCE_SETS), default="rmc", help="出力センテンス（既定 rmc）")
    parser.add_argument(
        "--gnss",
        choices=sorted(GNSS_SETS),
        default="gps-gal",
        help="使う衛星システム（既定 gps-gal。gps-gal-bds は出荷時と同じで、10 Hz を保てない）",
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--verify-only", action="store_true", help="設定を書き込まず、現在の出力と保存した設定を確認する")
    mode.add_argument("--query", action="store_true", help="ファームウェアと RAM・BBR の設定を表示して終わる（設定は変えない）")
    args = parser.parse_args()

    try:
        with serial.Serial(args.port, LOGGER_BAUD, timeout=0.1) as port:
            print("ボーレートを判定中...")
            baud = detect_baud(port)
            if baud is None:
                raise RuntimeError("NMEA を受信できません（配線と電源、ポート名を確認する）")
            print(f"  現在 {baud} bps")
            port.baudrate = baud

            if args.query:
                print("問い合わせ中...")
                query(port)
                return 0

            if not args.verify_only:
                print("設定を書き込み中...")
                configure(port, baud, args.rate_hz, args.sentences, args.gnss)
                baud = LOGGER_BAUD
                port.baudrate = baud

            print(f"出力を確認中（{VERIFY_SECONDS:.0f} 秒）...")
            ok = verify(port, baud, args.rate_hz, args.sentences, args.gnss)
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
