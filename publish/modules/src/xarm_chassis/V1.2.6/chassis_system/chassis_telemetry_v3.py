#!/usr/bin/env python3
"""Pure-Python parser for the K1 omni-chassis telemetry protocol V3.

The STM32 sends eight Classic CAN frames in this normal order::

    0x510 -> 0x511 -> 0x512 -> 0x513 -> 0x514 -> 0x515 -> 0x516 -> 0x517

Only ``0x514`` carries a sequence number.  It is metadata rather than a strict
commit frame, so a ``0x517`` arrival publishes a clearly labelled best-effort
snapshot only when the cached ``0x510``..``0x516`` frames are all fresh.
This module deliberately imports neither ROS nor python-can.
"""

from __future__ import annotations

import math
import struct
import time
from dataclasses import dataclass, field
from typing import Optional, Sequence, Tuple


TELEMETRY_V3_START_CAN_ID = 0x510
TELEMETRY_V3_SNAPSHOT_CAN_ID = 0x517
TELEMETRY_V3_METADATA_CAN_ID = 0x514
TELEMETRY_V3_PROTOCOL_VERSION = 3
TELEMETRY_V3_FRAME_IDS: Tuple[int, ...] = tuple(range(0x510, 0x518))
WHEEL_ORDER: Tuple[str, ...] = ("RF", "LF", "LR", "RR")

DEFAULT_FRESHNESS_SEC = 0.050
DEFAULT_OFFLINE_SEC = 0.100

# Integer fields intentionally travel in Float64MultiArray.  uint32 and uint16
# values remain exact IEEE-754 float64 integers; epoch nanoseconds never do, so
# the host stamp is represented as separate second/nanosecond fields.
TELEMETRY_V3_FLOAT64_FIELDS: Tuple[str, ...] = (
    "protocol_version",
    "valid_flags",
    "seq",
    "telemetry_drop_count",
    "device_timestamp_ms",
    "host_receive_sec",
    "host_receive_nanosec",
    "x_m",
    "y_m",
    "fused_yaw_rad",
    "wheel_yaw_rad",
    "mahony_yaw_rad",
    "actual_vx_m_s",
    "actual_vy_m_s",
    "actual_wz_rad_s",
    "target_vx_m_s",
    "target_vy_m_s",
    "target_wz_rad_s",
    "target_wheel_rf_rad_s",
    "target_wheel_lf_rad_s",
    "target_wheel_lr_rad_s",
    "target_wheel_rr_rad_s",
    "actual_wheel_rf_rad_s",
    "actual_wheel_lf_rad_s",
    "actual_wheel_lr_rad_s",
    "actual_wheel_rr_rad_s",
    "wheel_error_rf_rad_s",
    "wheel_error_lf_rad_s",
    "wheel_error_lr_rad_s",
    "wheel_error_rr_rad_s",
    "body_error_vx_m_s",
    "body_error_vy_m_s",
    "body_error_wz_rad_s",
    "slip_state",
    "velocity_valid_flags",
    "frame_age_510_ms",
    "frame_age_511_ms",
    "frame_age_512_ms",
    "frame_age_513_ms",
    "frame_age_514_ms",
    "frame_age_515_ms",
    "frame_age_516_ms",
    "frame_age_517_ms",
)

TELEMETRY_V3_STATUS_FLOAT64_FIELDS: Tuple[str, ...] = (
    "online",
    "latest_snapshot_complete",
    "last_protocol_version",
    "last_valid_flags",
    "last_seq",
    "last_telemetry_drop_count",
    "last_slip_state",
    "metadata_age_ms",
    "frame_age_510_ms",
    "frame_age_511_ms",
    "frame_age_512_ms",
    "frame_age_513_ms",
    "frame_age_514_ms",
    "frame_age_515_ms",
    "frame_age_516_ms",
    "frame_age_517_ms",
    "invalid_frames",
    "invalid_dlc",
    "protocol_version_mismatches",
    "incomplete_snapshots",
    "duplicate_sequences",
    "dropped_sequences",
    "sequence_regressions",
    "timestamp_regressions",
    "device_resets",
    "drop_count_growth_events",
)


@dataclass
class TelemetryV3Diagnostics:
    invalid_frames: int = 0
    invalid_dlc: int = 0
    protocol_version_mismatches: int = 0
    incomplete_snapshots: int = 0
    duplicate_sequences: int = 0
    dropped_sequences: int = 0
    sequence_regressions: int = 0
    timestamp_regressions: int = 0
    device_resets: int = 0
    drop_count_growth_events: int = 0


@dataclass(frozen=True)
class ChassisTelemetryV3:
    """One valid, fresh best-effort V3 snapshot triggered by ``0x517``."""

    host_receive_wall_time_ns: int
    host_receive_monotonic: float
    device_timestamp_ms: int
    seq: int
    protocol_version: int
    valid_flags: int
    telemetry_drop_count: int
    x_m: float
    y_m: float
    fused_yaw_rad: float
    wheel_yaw_rad: float
    mahony_yaw_rad: float
    actual_vx_m_s: float
    actual_vy_m_s: float
    actual_wz_rad_s: float
    target_vx_m_s: float
    target_vy_m_s: float
    target_wz_rad_s: float
    target_wheel_rad_s: Tuple[float, float, float, float]
    actual_wheel_rad_s: Tuple[float, float, float, float]
    wheel_speed_error_rad_s: Tuple[float, float, float, float]
    body_velocity_error: Tuple[float, float, float]
    slip_state: int
    velocity_valid_flags: int
    frame_ages_sec: Tuple[float, float, float, float, float, float, float, float]

    @property
    def host_receive_sec_nanosec(self) -> Tuple[int, int]:
        return divmod(self.host_receive_wall_time_ns, 1_000_000_000)

    def to_float64_values(self) -> list[float]:
        """Return values in :data:`TELEMETRY_V3_FLOAT64_FIELDS` order."""

        sec, nanosec = self.host_receive_sec_nanosec
        return [
            float(self.protocol_version),
            float(self.valid_flags),
            float(self.seq),
            float(self.telemetry_drop_count),
            float(self.device_timestamp_ms),
            float(sec),
            float(nanosec),
            self.x_m,
            self.y_m,
            self.fused_yaw_rad,
            self.wheel_yaw_rad,
            self.mahony_yaw_rad,
            self.actual_vx_m_s,
            self.actual_vy_m_s,
            self.actual_wz_rad_s,
            self.target_vx_m_s,
            self.target_vy_m_s,
            self.target_wz_rad_s,
            *self.target_wheel_rad_s,
            *self.actual_wheel_rad_s,
            *self.wheel_speed_error_rad_s,
            *self.body_velocity_error,
            float(self.slip_state),
            float(self.velocity_valid_flags),
            *(age * 1000.0 for age in self.frame_ages_sec),
        ]


@dataclass(frozen=True)
class TelemetryV3Status:
    """Quality/status view that remains available even while V3 is offline."""

    online: bool
    latest_snapshot_complete: bool
    last_protocol_version: Optional[int]
    last_valid_flags: Optional[int]
    last_seq: Optional[int]
    last_telemetry_drop_count: Optional[int]
    last_slip_state: Optional[int]
    metadata_age_sec: float
    frame_ages_sec: Tuple[float, float, float, float, float, float, float, float]
    diagnostics: TelemetryV3Diagnostics

    def to_float64_values(self) -> list[float]:
        def age_ms(age: float) -> float:
            # A missing frame has no finite age.  Use -1 as the documented
            # status sentinel instead of publishing Inf into PlotJuggler.
            return age * 1000.0 if math.isfinite(age) else -1.0

        return [
            float(self.online),
            float(self.latest_snapshot_complete),
            float(-1 if self.last_protocol_version is None else self.last_protocol_version),
            float(-1 if self.last_valid_flags is None else self.last_valid_flags),
            float(-1 if self.last_seq is None else self.last_seq),
            float(-1 if self.last_telemetry_drop_count is None else self.last_telemetry_drop_count),
            float(-1 if self.last_slip_state is None else self.last_slip_state),
            age_ms(self.metadata_age_sec),
            *(age_ms(age) for age in self.frame_ages_sec),
            float(self.diagnostics.invalid_frames),
            float(self.diagnostics.invalid_dlc),
            float(self.diagnostics.protocol_version_mismatches),
            float(self.diagnostics.incomplete_snapshots),
            float(self.diagnostics.duplicate_sequences),
            float(self.diagnostics.dropped_sequences),
            float(self.diagnostics.sequence_regressions),
            float(self.diagnostics.timestamp_regressions),
            float(self.diagnostics.device_resets),
            float(self.diagnostics.drop_count_growth_events),
        ]


@dataclass
class _CachedFrame:
    payload: bytes
    received_monotonic: float


class ChassisTelemetryV3Assembler:
    """Cache V3 frames and emit one fresh best-effort snapshot on ``0x517``."""

    def __init__(
        self,
        freshness_sec: float = DEFAULT_FRESHNESS_SEC,
        offline_sec: float = DEFAULT_OFFLINE_SEC,
    ) -> None:
        if not math.isfinite(freshness_sec) or freshness_sec <= 0.0:
            raise ValueError("freshness_sec must be positive and finite")
        if not math.isfinite(offline_sec) or offline_sec < freshness_sec:
            raise ValueError("offline_sec must be finite and at least freshness_sec")
        self.freshness_sec = freshness_sec
        self.offline_sec = offline_sec
        self.diagnostics = TelemetryV3Diagnostics()
        self._frames: dict[int, _CachedFrame] = {}
        self._last_seq: Optional[int] = None
        self._last_timestamp_ms: Optional[int] = None
        self._last_drop_count: Optional[int] = None
        self._last_snapshot_complete = False

    def reset(self) -> None:
        self._frames.clear()
        self._last_seq = None
        self._last_timestamp_ms = None
        self._last_drop_count = None
        self._last_snapshot_complete = False

    def ingest(
        self,
        arbitration_id: int,
        data: Sequence[int] | bytes | bytearray,
        *,
        dlc: Optional[int] = None,
        is_extended_id: bool = False,
        is_error_frame: bool = False,
        host_receive_monotonic: Optional[float] = None,
        host_receive_wall_time_ns: Optional[int] = None,
    ) -> Optional[ChassisTelemetryV3]:
        """Accept one CAN frame; only a valid fresh ``0x517`` returns a snapshot."""

        now = time.monotonic() if host_receive_monotonic is None else host_receive_monotonic
        wall_ns = time.time_ns() if host_receive_wall_time_ns is None else host_receive_wall_time_ns
        if not math.isfinite(now) or wall_ns < 0:
            raise ValueError("host receive timestamps must be valid")
        if arbitration_id not in TELEMETRY_V3_FRAME_IDS:
            return None
        if is_extended_id or is_error_frame:
            self.diagnostics.invalid_frames += 1
            return None
        payload = bytes(data)
        effective_dlc = len(payload) if dlc is None else dlc
        if effective_dlc != 8 or len(payload) != 8:
            self.diagnostics.invalid_dlc += 1
            return None

        self._frames[arbitration_id] = _CachedFrame(payload, now)
        if arbitration_id != TELEMETRY_V3_SNAPSHOT_CAN_ID:
            return None
        return self._snapshot_on_517(now, wall_ns)

    def status(self, now_monotonic: Optional[float] = None) -> TelemetryV3Status:
        now = time.monotonic() if now_monotonic is None else now_monotonic
        if not math.isfinite(now):
            raise ValueError("now_monotonic must be finite")
        ages = self._frame_ages(now)
        metadata_age = ages[TELEMETRY_V3_METADATA_CAN_ID - TELEMETRY_V3_START_CAN_ID]
        metadata = self._frames.get(TELEMETRY_V3_METADATA_CAN_ID)
        version = flags = seq = drop_count = None
        if metadata is not None:
            seq, version, flags, drop_count = struct.unpack("<HBBI", metadata.payload)
        latest_512 = self._frames.get(0x512)
        slip_state = latest_512.payload[6] if latest_512 is not None else None
        fresh_complete = self._last_snapshot_complete and all(age <= self.freshness_sec for age in ages)
        return TelemetryV3Status(
            online=metadata is not None and metadata_age <= self.offline_sec,
            latest_snapshot_complete=fresh_complete,
            last_protocol_version=version,
            last_valid_flags=flags,
            last_seq=seq,
            last_telemetry_drop_count=drop_count,
            last_slip_state=slip_state,
            metadata_age_sec=metadata_age,
            frame_ages_sec=ages,
            diagnostics=self.diagnostics,
        )

    def _snapshot_on_517(self, now: float, wall_ns: int) -> Optional[ChassisTelemetryV3]:
        self._last_snapshot_complete = False
        if any(can_id not in self._frames for can_id in TELEMETRY_V3_FRAME_IDS):
            self.diagnostics.incomplete_snapshots += 1
            return None
        ages = self._frame_ages(now)
        # 0x517 is the trigger frame.  Every preceding source must be fresh.
        if any(age > self.freshness_sec for age in ages[:-1]):
            self.diagnostics.incomplete_snapshots += 1
            return None

        metadata = self._frames[0x514].payload
        seq, protocol_version, valid_flags, drop_count = struct.unpack("<HBBI", metadata)
        if protocol_version != TELEMETRY_V3_PROTOCOL_VERSION:
            self.diagnostics.protocol_version_mismatches += 1
            return None

        x_mm, y_mm = struct.unpack("<ii", self._frames[0x510].payload)
        fused_yaw_mrad, device_timestamp_ms = struct.unpack("<iI", self._frames[0x511].payload)
        actual_vx, actual_vy, actual_wz, slip_state, flags_512 = struct.unpack(
            "<hhhBB", self._frames[0x512].payload
        )
        # 0x512 and 0x514 are transmitted in different frames.  Retain both
        # values for diagnostics; 0x514 remains the group metadata used for
        # state-validity decisions, so a transient difference is not treated as
        # a false protocol error.
        target_wheels_raw = struct.unpack("<hhhh", self._frames[0x513].payload)
        wheel_yaw_mrad, mahony_yaw_mrad = struct.unpack("<ii", self._frames[0x515].payload)
        target_vx, target_vy, target_wz, _reserved = struct.unpack("<hhhH", self._frames[0x516].payload)
        actual_wheels_raw = struct.unpack("<hhhh", self._frames[0x517].payload)

        if not self._accept_sequence_timestamp_and_drop(seq, device_timestamp_ms, drop_count):
            return None

        target_wheels = tuple(value / 1000.0 for value in target_wheels_raw)
        actual_wheels = tuple(value / 1000.0 for value in actual_wheels_raw)
        errors = tuple(target - actual for target, actual in zip(target_wheels, actual_wheels))
        sample = ChassisTelemetryV3(
            host_receive_wall_time_ns=wall_ns,
            host_receive_monotonic=now,
            device_timestamp_ms=device_timestamp_ms,
            seq=seq,
            protocol_version=protocol_version,
            valid_flags=valid_flags,
            telemetry_drop_count=drop_count,
            x_m=x_mm / 1000.0,
            y_m=y_mm / 1000.0,
            fused_yaw_rad=fused_yaw_mrad / 1000.0,
            wheel_yaw_rad=wheel_yaw_mrad / 1000.0,
            mahony_yaw_rad=mahony_yaw_mrad / 1000.0,
            actual_vx_m_s=actual_vx / 1000.0,
            actual_vy_m_s=actual_vy / 1000.0,
            actual_wz_rad_s=actual_wz / 1000.0,
            target_vx_m_s=target_vx / 1000.0,
            target_vy_m_s=target_vy / 1000.0,
            target_wz_rad_s=target_wz / 1000.0,
            target_wheel_rad_s=target_wheels,
            actual_wheel_rad_s=actual_wheels,
            wheel_speed_error_rad_s=errors,
            body_velocity_error=(
                target_vx / 1000.0 - actual_vx / 1000.0,
                target_vy / 1000.0 - actual_vy / 1000.0,
                target_wz / 1000.0 - actual_wz / 1000.0,
            ),
            slip_state=slip_state,
            velocity_valid_flags=flags_512,
            frame_ages_sec=ages,
        )
        if not all(math.isfinite(value) for value in sample.to_float64_values()):
            self.diagnostics.invalid_frames += 1
            return None
        self._last_snapshot_complete = True
        return sample

    def _frame_ages(self, now: float) -> Tuple[float, float, float, float, float, float, float, float]:
        ages = []
        for can_id in TELEMETRY_V3_FRAME_IDS:
            cached = self._frames.get(can_id)
            ages.append(float("inf") if cached is None else max(0.0, now - cached.received_monotonic))
        return tuple(ages)  # type: ignore[return-value]

    def _accept_sequence_timestamp_and_drop(self, seq: int, timestamp_ms: int, drop_count: int) -> bool:
        if self._last_seq is not None:
            seq_delta = (seq - self._last_seq) & 0xFFFF
            time_delta = (timestamp_ms - int(self._last_timestamp_ms)) & 0xFFFFFFFF
            if seq_delta == 0:
                self.diagnostics.duplicate_sequences += 1
                return False
            if seq_delta >= 0x8000:
                # A genuine MCU restart moves both counters backward.  A simple
                # seq regression with time still moving forward is rejected.
                if time_delta >= 0x80000000:
                    self.diagnostics.device_resets += 1
                else:
                    self.diagnostics.sequence_regressions += 1
                    return False
            elif seq_delta > 1:
                self.diagnostics.dropped_sequences += seq_delta - 1

            if time_delta >= 0x80000000 and seq_delta < 0x8000:
                self.diagnostics.timestamp_regressions += 1
                return False
            if self._last_drop_count is not None:
                drop_delta = (drop_count - self._last_drop_count) & 0xFFFFFFFF
                if 0 < drop_delta < 0x80000000:
                    self.diagnostics.drop_count_growth_events += 1

        self._last_seq = seq
        self._last_timestamp_ms = timestamp_ms
        self._last_drop_count = drop_count
        return True
