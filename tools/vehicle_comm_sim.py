#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
STM32 <-> vehicle UART protocol V1.0 simulator (PC acts as vehicle).

Wiring (USB-TTL to STM32 UART4):
  PC TX  -> STM32 PC11 (UART4_RX)
  PC RX  -> STM32 PC10 (UART4_TX)
  GND    -> GND

Requires: pip install pyserial

Examples:
  python tools/vehicle_comm_sim.py -p COM13
  python tools/vehicle_comm_sim.py -p COM13 --speed 800
  python tools/vehicle_comm_sim.py -p COM13 --alternate 10 15 --tx-hz 25
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from typing import List, Tuple

try:
    import serial
except ImportError:
    print("Install pyserial: pip install pyserial", file=sys.stderr)
    sys.exit(1)

FRAME_HEAD = bytes((0xAA, 0x55))
PAYLOAD_LEN = 8

WARN_NAMES = {
    0: "clear (0)",
    1: "warn  (1)",
    2: "stop  (2)",
}


def _setup_console_utf8() -> None:
    if sys.platform == "win32":
        try:
            sys.stdout.reconfigure(encoding="utf-8")
            sys.stderr.reconfigure(encoding="utf-8")
        except (AttributeError, OSError):
            pass


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_frame(payload: bytes) -> bytes:
    if len(payload) != PAYLOAD_LEN:
        raise ValueError(f"payload length must be {PAYLOAD_LEN}")
    body = bytes((PAYLOAD_LEN,)) + payload
    crc = crc16_modbus(body)
    return FRAME_HEAD + body + struct.pack("<H", crc)


def build_speed_frame(vx_mm_s: int, heartbeat: int = 0x05) -> bytes:
    """Vehicle -> STM32: vx_mm_s (int16 LE) + HEARTBEAT at byte 7."""
    payload = bytearray(PAYLOAD_LEN)
    struct.pack_into("<h", payload, 0, vx_mm_s)
    payload[7] = heartbeat & 0xFF
    return build_frame(bytes(payload))


def parse_stm32_payload(payload: bytes) -> Tuple[int, int, int]:
    """STM32 -> vehicle: warn_level, dist_mm, heartbeat."""
    warn = payload[0]
    dist = payload[1] | (payload[2] << 8)
    hb = payload[7]
    return warn, dist, hb


class FrameParser:
    """Byte stream parser (same logic as firmware comm_rx_feed_byte)."""

    def __init__(self) -> None:
        self._state = "H1"
        self._data_len = 0
        self._buf = bytearray()

    def feed(self, chunk: bytes) -> List[bytes]:
        out: List[bytes] = []
        for byte in chunk:
            if self._state == "H1":
                if byte == 0xAA:
                    self._state = "H2"
            elif self._state == "H2":
                if byte == 0x55:
                    self._state = "LEN"
                elif byte == 0xAA:
                    self._state = "H2"
                else:
                    self._state = "H1"
            elif self._state == "LEN":
                if byte > 16:
                    self._state = "H1"
                    continue
                self._data_len = byte
                self._buf = bytearray()
                self._state = "DATA"
            elif self._state == "DATA":
                self._buf.append(byte)
                need = self._data_len + 2
                if len(self._buf) < need:
                    continue
                crc_body = bytes((self._data_len,)) + self._buf[: self._data_len]
                calc = crc16_modbus(crc_body)
                recv = self._buf[self._data_len] | (self._buf[self._data_len + 1] << 8)
                if calc == recv and self._data_len == PAYLOAD_LEN:
                    out.append(bytes(self._buf[: self._data_len]))
                self._state = "H1"
        return out


def format_dist(dist_mm: int) -> str:
    if dist_mm == 0xFFFF:
        return "no obstacle (0xFFFF)"
    return f"{dist_mm} mm ({dist_mm / 1000.0:.2f} m)"


def main() -> int:
    _setup_console_utf8()

    ap = argparse.ArgumentParser(description="STM32 vehicle UART protocol simulator")
    ap.add_argument("-p", "--port", required=True, help="Serial port, e.g. COM13")
    ap.add_argument("-b", "--baud", type=int, default=115200, help="Baud rate (default 115200)")
    ap.add_argument(
        "--speed",
        type=int,
        default=None,
        help="Fixed vx_mm_s in mm/s (overrides --alternate)",
    )
    ap.add_argument(
        "--alternate",
        type=int,
        nargs=2,
        metavar=("V1", "V2"),
        default=(10, 15),
        help="Toggle between two speeds in mm/s (default 10 15)",
    )
    ap.add_argument(
        "--switch-sec",
        type=float,
        default=3.0,
        help="Seconds between speed toggle when using --alternate (default 3)",
    )
    ap.add_argument(
        "--tx-hz",
        type=float,
        default=20.0,
        help="TX rate of speed frames to STM32 in Hz (default 20)",
    )
    ap.add_argument(
        "--heartbeat",
        type=lambda x: int(x, 0),
        default=0x05,
        help="HEARTBEAT byte in vehicle->STM32 frame (default 0x05)",
    )
    args = ap.parse_args()

    use_fixed = args.speed is not None
    speeds = (args.speed, args.speed) if use_fixed else tuple(args.alternate)
    tx_interval = 1.0 / args.tx_hz

    print("=" * 60)
    print("STM32 <-> vehicle protocol sim (PC = vehicle)")
    print(f"  Port: {args.port} @ {args.baud} bps")
    if use_fixed:
        print(f"  TX speed: fixed {speeds[0]} mm/s @ {args.tx_hz} Hz")
    else:
        print(
            f"  TX speed: alternate {speeds[0]} / {speeds[1]} mm/s, "
            f"switch every {args.switch_sec}s @ {args.tx_hz} Hz"
        )
    print("  RX: warn_level, dist_mm from STM32")
    print("  Ctrl+C to quit")
    print("=" * 60)

    parser = FrameParser()
    speed_idx = 0
    last_switch = time.monotonic()
    last_tx = 0.0
    rx_count = 0
    last_line = ""

    try:
        with serial.Serial(args.port, args.baud, timeout=0.05) as ser:
            while True:
                now = time.monotonic()

                if not use_fixed and (now - last_switch) >= args.switch_sec:
                    speed_idx ^= 1
                    last_switch = now
                    print(f"\n>>> Switch TX speed to {speeds[speed_idx]} mm/s\n")

                if now - last_tx >= tx_interval:
                    vx = speeds[speed_idx] if not use_fixed else speeds[0]
                    ser.write(build_speed_frame(vx, args.heartbeat))
                    last_tx = now

                chunk = ser.read(256)
                if chunk:
                    for payload in parser.feed(chunk):
                        warn, dist, hb = parse_stm32_payload(payload)
                        rx_count += 1
                        wname = WARN_NAMES.get(warn, f"unknown({warn})")
                        line = (
                            f"[RX #{rx_count:04d}] warn={warn} {wname}  "
                            f"dist={format_dist(dist)}  heartbeat=0x{hb:02X}"
                        )
                        if line != last_line:
                            print(line)
                            last_line = line

                time.sleep(0.001)

    except serial.SerialException as e:
        print(f"Serial error: {e}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\nExit.")
        return 0


if __name__ == "__main__":
    sys.exit(main())
