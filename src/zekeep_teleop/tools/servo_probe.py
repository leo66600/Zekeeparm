#!/usr/bin/env python3
"""Read-only probe for Zhongling serial servo IDs."""

import argparse
import time

import serial


def send_command(port: serial.Serial, command: str) -> str:
    port.reset_input_buffer()
    port.write(f"{command}\r\n".encode("ascii"))
    port.flush()
    time.sleep(0.03)
    return port.read_until(b"!").decode("ascii", errors="ignore").strip()


def main() -> None:
    parser = argparse.ArgumentParser(description="Probe Zhongling servo IDs without changing them.")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--start", type=int, default=1)
    parser.add_argument("--end", type=int, default=7)
    args = parser.parse_args()
    if not 0 <= args.start <= args.end <= 999:
        parser.error("require 0 <= --start <= --end <= 999")

    found: list[int] = []
    with serial.Serial(args.port, args.baudrate, timeout=0.1) as port:
        print(f"Opened {args.port} @ {args.baudrate}")
        for servo_id in range(args.start, args.end + 1):
            response = send_command(port, f"#{servo_id:03d}PRAD!")
            if response:
                found.append(servo_id)
                print(f"ID {servo_id:03d}: {response}")
    if found:
        print("Found IDs:", ", ".join(f"{servo_id:03d}" for servo_id in found))
    else:
        print("No servo response found.")


if __name__ == "__main__":
    main()

