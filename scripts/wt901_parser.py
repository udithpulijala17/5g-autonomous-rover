from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path


@dataclass
class WT901Data:
    ax_g: float
    ay_g: float
    az_g: float

    gx_dps: float
    gy_dps: float
    gz_dps: float

    roll_deg: float
    pitch_deg: float
    yaw_deg: float


def int16_le(lo: int, hi: int) -> int:
    value = lo | (hi << 8)
    return value - 0x10000 if value & 0x8000 else value


def parse_0x61(packet: bytes) -> WT901Data:
    if len(packet) != 20:
        raise ValueError(f"Expected 20 bytes, got {len(packet)}")

    if packet[0] != 0x55 or packet[1] != 0x61:
        raise ValueError(
            f"Invalid frame header: {packet[0]:02X} {packet[1]:02X}"
        )

    raw = [
        int16_le(packet[i], packet[i + 1])
        for i in range(2, 20, 2)
    ]

    ax, ay, az, gx, gy, gz, roll, pitch, yaw = raw

    return WT901Data(
        ax_g=ax / 32768.0 * 16.0,
        ay_g=ay / 32768.0 * 16.0,
        az_g=az / 32768.0 * 16.0,
        gx_dps=gx / 32768.0 * 2000.0,
        gy_dps=gy / 32768.0 * 2000.0,
        gz_dps=gz / 32768.0 * 2000.0,
        roll_deg=roll / 32768.0 * 180.0,
        pitch_deg=pitch / 32768.0 * 180.0,
        yaw_deg=yaw / 32768.0 * 180.0,
    )


def parse_hex_line(line: str) -> WT901Data:
    parts = line.strip().split()
    packet = bytes(int(x, 16) for x in parts)
    return parse_0x61(packet)


def main() -> None:
    csv_path = Path("logs/imu_ble_raw.csv")

    with csv_path.open(newline="") as file:
        reader = csv.reader(file)

        for row in reader:
            if len(row) < 2:
                continue

            timestamp = row[0]
            hex_data = row[1]

            try:
                data = parse_hex_line(hex_data)
            except ValueError:
                continue

            print(
                f"{timestamp}\n"
                f"  accel : "
                f"{data.ax_g:+.4f} g, "
                f"{data.ay_g:+.4f} g, "
                f"{data.az_g:+.4f} g\n"
                f"  gyro  : "
                f"{data.gx_dps:+.3f} °/s, "
                f"{data.gy_dps:+.3f} °/s, "
                f"{data.gz_dps:+.3f} °/s\n"
                f"  angle : "
                f"{data.roll_deg:+.3f}°, "
                f"{data.pitch_deg:+.3f}°, "
                f"{data.yaw_deg:+.3f}°"
            )


if __name__ == "__main__":
    main()
    
