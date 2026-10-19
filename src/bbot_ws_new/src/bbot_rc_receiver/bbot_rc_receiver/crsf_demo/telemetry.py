"""CRSF 遥测数据源。
全部编码进 CRSF GPS 0x02 帧：
    target_speed        → GPS ground_speed (km/h)
    battery_soc         → GPS satellites (0-100)
    battery_temperature → GPS altitude (°C，利用 +1000 偏移)
遥控器显示：
    Gspd = 目标车速 (km/h)
    Sats = 电池SOC (%)
    Alt  = 电池温度 (°C)
"""
from __future__ import annotations
import logging
import math
import time
from dataclasses import dataclass
from .frames import GpsData, build_gps_frame

logger = logging.getLogger(__name__)


@dataclass
class TelemetryState:
    target_speed_mps: float = 0.0
    battery_soc: float = 0.0
    battery_temperature: float = 0.0


class TelemetrySource:
    def __init__(self, interval: float = 0.1) -> None:
        self._interval = max(0.01, float(interval))
        self._next_due = 0.0
        self.state = TelemetryState()

    def set_target_speed(self, speed_mps: float) -> None:
        try:
            speed = float(speed_mps)
            if not math.isfinite(speed):
                speed = 0.0
            self.state.target_speed_mps = speed
        except (TypeError, ValueError):
            logger.warning("Invalid target speed: %r", speed_mps)
            self.state.target_speed_mps = 0.0

    def set_battery_soc(self, soc: float) -> None:
        try:
            val = float(soc)
            if not math.isfinite(val):
                val = 0.0
            self.state.battery_soc = max(0.0, min(100.0, val))
        except (TypeError, ValueError):
            logger.warning("Invalid battery SOC: %r", soc)
            self.state.battery_soc = 0.0

    def set_battery_temperature(self, temp: float) -> None:
        try:
            val = float(temp)
            if not math.isfinite(val):
                val = 0.0
            self.state.battery_temperature = val
        except (TypeError, ValueError):
            logger.warning("Invalid battery temperature: %r", temp)
            self.state.battery_temperature = 0.0

    def update(self, now: float | None = None) -> None:
        return

    def due_frames(self, now: float | None = None) -> list[bytes]:
        if now is None:
            now = time.monotonic()
        frames: list[bytes] = []
        if now < self._next_due:
            return frames
        self._next_due = now + self._interval

        target_speed_kmh = abs(self.state.target_speed_mps) * 3.6
        gps = GpsData(
            latitude_deg=0.0,
            longitude_deg=0.0,
            ground_speed_kmh=target_speed_kmh,
            heading_deg=0.0,
            altitude_m=int(round(self.state.battery_temperature)),
            satellites=int(round(self.state.battery_soc)),
        )
        try:
            frames.append(build_gps_frame(gps))
        except Exception as exc:
            logger.warning("Failed to build GPS telemetry frame: %s", exc)
        return frames


__all__ = ["TelemetryState", "TelemetrySource"]
