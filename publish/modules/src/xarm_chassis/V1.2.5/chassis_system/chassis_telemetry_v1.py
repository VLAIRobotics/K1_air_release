#!/usr/bin/env python3
"""Pure-Python parser for the STM32 physical chassis telemetry protocol v1.

This module deliberately has no python-can or ROS dependency. It accepts
individual CAN frame fields and emits exactly one :class:`ChassisTelemetryV1`
only when a valid 0x510 -> 0x514 group is committed by 0x514.
"""

from __future__ import annotations

import math
import struct
import time
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple


TELEMETRY_START_CAN_ID = 0x510
TELEMETRY_COMMIT_CAN_ID = 0x514
TELEMETRY_PROTOCOL_VERSION = 1
TELEMETRY_REQUIRED_VALID_FLAGS = 0x3F

# A 200 Hz sender should normally complete a five-frame group in a few ms.
# This value only detects incomplete host-side assemblies; it is not a device
# timestamp timeout and is configurable for integration tests.
DEFAULT_ASSEMBLY_TIMEOUT_SEC = 0.100

# Temporary std_msgs/Float64MultiArray schema used because this workspace has
# no existing ROS interface package in which to define a generated message.
# Integers are emitted as float64 (not float32); every uint32 value is exactly
# representable by IEEE-754 float64. Host wall time is split into sec/nsec so
# a nanosecond epoch value is never lossy-converted to float64.
TELEMETRY_V1_FLOAT64_FIELDS: Tuple[str, ...] = (
    "protocol_version",
    "valid_flags",
    "seq",
    "telemetry_drop_count",
    "device_timestamp_ms",
    "host_receive_sec",
    "host_receive_nanosec",
    "x_m",
    "y_m",
    "yaw_total_rad",
    "actual_vx_m_s",
    "actual_wz_rad_s",
    "left_actual_rpm",
    "right_actual_rpm",
    "target_vx_m_s",
    "target_wz_rad_s",
    "left_target_rpm",
    "right_target_rpm",
)


@dataclass(frozen=True)
class ChassisTelemetryV1:
    """One validated physical telemetry sample committed by CAN ID 0x514."""

    host_receive_wall_time_ns: int
    host_receive_monotonic: float
    device_timestamp_ms: int
    seq: int
    protocol_version: int
    valid_flags: int
    telemetry_drop_count: int
    x_m: float
    y_m: float
    yaw_total_rad: float
    actual_vx_m_s: float
    actual_wz_rad_s: float
    left_actual_rpm: float
    right_actual_rpm: float
    target_vx_m_s: float
    target_wz_rad_s: float
    left_target_rpm: float
    right_target_rpm: float

    @property
    def host_receive_sec_nanosec(self) -> Tuple[int, int]:
        return divmod(self.host_receive_wall_time_ns, 1_000_000_000)

    def to_float64_values(self) -> list[float]:
        """Return values in :data:`TELEMETRY_V1_FLOAT64_FIELDS` order."""
        host_sec, host_nanosec = self.host_receive_sec_nanosec
        return [
            float(self.protocol_version),
            float(self.valid_flags),
            float(self.seq),
            float(self.telemetry_drop_count),
            float(self.device_timestamp_ms),
            float(host_sec),
            float(host_nanosec),
            self.x_m,
            self.y_m,
            self.yaw_total_rad,
            self.actual_vx_m_s,
            self.actual_wz_rad_s,
            self.left_actual_rpm,
            self.right_actual_rpm,
            self.target_vx_m_s,
            self.target_wz_rad_s,
            self.left_target_rpm,
            self.right_target_rpm,
        ]


@dataclass
class TelemetryDiagnostics:
    """Counters for rejected traffic and device-side discontinuities."""

    invalid_frames: int = 0
    invalid_dlc: int = 0
    incomplete_groups: int = 0
    out_of_order_frames: int = 0
    invalid_protocol_versions: int = 0
    invalid_valid_flags: int = 0
    duplicate_sequences: int = 0
    dropped_sequences: int = 0
    sequence_regressions: int = 0
    timestamp_regressions: int = 0
    device_resets: int = 0


@dataclass
class _PartialTelemetryGroup:
    started_monotonic: float
    x_m: float
    y_m: float
    yaw_total_rad: Optional[float] = None
    device_timestamp_ms: Optional[int] = None
    actual_vx_m_s: Optional[float] = None
    actual_wz_rad_s: Optional[float] = None
    left_actual_rpm: Optional[float] = None
    right_actual_rpm: Optional[float] = None
    target_vx_m_s: Optional[float] = None
    target_wz_rad_s: Optional[float] = None
    left_target_rpm: Optional[float] = None
    right_target_rpm: Optional[float] = None
    next_can_id: int = TELEMETRY_START_CAN_ID + 1


def differential_drive_from_wheel_rpm(
    left_rpm: float,
    right_rpm: float,
    wheel_radius_m: float,
    wheel_track_m: float,
) -> Tuple[float, float]:
    """Recompute physical ``(vx, wz)`` for a supplied wheel geometry.

    The telemetry protocol does not define wheel radius or track width, so the
    listener does not use this helper to alter or reject STM32-provided values.
    It exists for offline validation once the vehicle geometry is known.
    """
    values = (left_rpm, right_rpm, wheel_radius_m, wheel_track_m)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("wheel kinematics values must be finite")
    if wheel_radius_m <= 0.0 or wheel_track_m <= 0.0:
        raise ValueError("wheel radius and track must be positive")

    left_m_s = left_rpm * (2.0 * math.pi * wheel_radius_m / 60.0)
    right_m_s = right_rpm * (2.0 * math.pi * wheel_radius_m / 60.0)
    return ((left_m_s + right_m_s) / 2.0, (right_m_s - left_m_s) / wheel_track_m)


class ChassisTelemetryAssembler:
    """Assemble and validate one ordered five-frame telemetry group at a time."""

    def __init__(self, assembly_timeout_sec: float = DEFAULT_ASSEMBLY_TIMEOUT_SEC) -> None:
        if not math.isfinite(assembly_timeout_sec) or assembly_timeout_sec <= 0.0:
            raise ValueError("assembly_timeout_sec must be positive and finite")
        self.assembly_timeout_sec = assembly_timeout_sec
        self.diagnostics = TelemetryDiagnostics()
        self._partial: Optional[_PartialTelemetryGroup] = None
        self._last_seq: Optional[int] = None
        self._last_timestamp_ms: Optional[int] = None

    def reset(self) -> None:
        """Clear incomplete and device sequence/time baselines."""
        self._partial = None
        self._last_seq = None
        self._last_timestamp_ms = None

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
    ) -> Optional[ChassisTelemetryV1]:
        """Process one CAN frame and return a sample only on valid 0x514 commit."""
        monotonic_now = time.monotonic() if host_receive_monotonic is None else host_receive_monotonic
        wall_now_ns = time.time_ns() if host_receive_wall_time_ns is None else host_receive_wall_time_ns
        if not math.isfinite(monotonic_now) or wall_now_ns < 0:
            raise ValueError("host receive timestamps must be valid")

        self._expire_partial(monotonic_now)
        if arbitration_id < TELEMETRY_START_CAN_ID or arbitration_id > TELEMETRY_COMMIT_CAN_ID:
            return None
        if is_extended_id or is_error_frame:
            self.diagnostics.invalid_frames += 1
            return None

        payload = bytes(data)
        effective_dlc = len(payload) if dlc is None else dlc
        if effective_dlc != 8 or len(payload) != 8:
            self.diagnostics.invalid_dlc += 1
            return None

        if arbitration_id == TELEMETRY_START_CAN_ID:
            return self._start_group(payload, monotonic_now)
        if self._partial is None:
            self.diagnostics.out_of_order_frames += 1
            return None
        if arbitration_id != self._partial.next_can_id:
            self.diagnostics.out_of_order_frames += 1
            self._partial = None
            return None

        if arbitration_id == 0x511:
            self._partial.yaw_total_rad, self._partial.device_timestamp_ms = self._parse_0x511(payload)
            self._partial.next_can_id = 0x512
            return None
        if arbitration_id == 0x512:
            (
                self._partial.actual_vx_m_s,
                self._partial.actual_wz_rad_s,
                self._partial.left_actual_rpm,
                self._partial.right_actual_rpm,
            ) = self._parse_0x512(payload)
            self._partial.next_can_id = 0x513
            return None
        if arbitration_id == 0x513:
            (
                self._partial.target_vx_m_s,
                self._partial.target_wz_rad_s,
                self._partial.left_target_rpm,
                self._partial.right_target_rpm,
            ) = self._parse_0x513(payload)
            self._partial.next_can_id = 0x514
            return None
        return self._commit_group(payload, monotonic_now, wall_now_ns)

    def _expire_partial(self, monotonic_now: float) -> None:
        if self._partial is None:
            return
        if monotonic_now - self._partial.started_monotonic > self.assembly_timeout_sec:
            self.diagnostics.incomplete_groups += 1
            self._partial = None

    def _start_group(self, payload: bytes, monotonic_now: float) -> Optional[ChassisTelemetryV1]:
        if self._partial is not None:
            self.diagnostics.incomplete_groups += 1
        x_mm, y_mm = struct.unpack("<ii", payload)
        self._partial = _PartialTelemetryGroup(
            started_monotonic=monotonic_now,
            x_m=x_mm / 1000.0,
            y_m=y_mm / 1000.0,
        )
        return None

    @staticmethod
    def _parse_0x511(payload: bytes) -> Tuple[float, int]:
        yaw_total_mrad, timestamp_ms = struct.unpack("<iI", payload)
        return yaw_total_mrad / 1000.0, timestamp_ms

    @staticmethod
    def _parse_0x512(payload: bytes) -> Tuple[float, float, float, float]:
        actual_vx_mm_s, actual_wz_mrad_s, left_rpm_x10, right_rpm_x10 = struct.unpack(
            "<hhhh", payload
        )
        return (
            actual_vx_mm_s / 1000.0,
            actual_wz_mrad_s / 1000.0,
            left_rpm_x10 / 10.0,
            right_rpm_x10 / 10.0,
        )

    @staticmethod
    def _parse_0x513(payload: bytes) -> Tuple[float, float, float, float]:
        target_vx_mm_s, target_wz_mrad_s, left_rpm, right_rpm = struct.unpack("<hhhh", payload)
        return (
            target_vx_mm_s / 1000.0,
            target_wz_mrad_s / 1000.0,
            float(left_rpm),
            float(right_rpm),
        )

    def _commit_group(
        self,
        payload: bytes,
        monotonic_now: float,
        wall_now_ns: int,
    ) -> Optional[ChassisTelemetryV1]:
        partial = self._partial
        self._partial = None
        if partial is None:
            self.diagnostics.out_of_order_frames += 1
            return None

        seq, protocol_version, valid_flags, telemetry_drop_count = struct.unpack("<HBBI", payload)
        if protocol_version != TELEMETRY_PROTOCOL_VERSION:
            self.diagnostics.invalid_protocol_versions += 1
            return None
        if valid_flags != TELEMETRY_REQUIRED_VALID_FLAGS:
            self.diagnostics.invalid_valid_flags += 1
            return None
        if not self._accept_sequence_and_timestamp(seq, partial.device_timestamp_ms):
            return None

        required_values = (
            partial.yaw_total_rad,
            partial.device_timestamp_ms,
            partial.actual_vx_m_s,
            partial.actual_wz_rad_s,
            partial.left_actual_rpm,
            partial.right_actual_rpm,
            partial.target_vx_m_s,
            partial.target_wz_rad_s,
            partial.left_target_rpm,
            partial.right_target_rpm,
        )
        if any(value is None for value in required_values):
            self.diagnostics.incomplete_groups += 1
            return None

        return ChassisTelemetryV1(
            host_receive_wall_time_ns=wall_now_ns,
            host_receive_monotonic=monotonic_now,
            device_timestamp_ms=int(partial.device_timestamp_ms),
            seq=seq,
            protocol_version=protocol_version,
            valid_flags=valid_flags,
            telemetry_drop_count=telemetry_drop_count,
            x_m=partial.x_m,
            y_m=partial.y_m,
            yaw_total_rad=float(partial.yaw_total_rad),
            actual_vx_m_s=float(partial.actual_vx_m_s),
            actual_wz_rad_s=float(partial.actual_wz_rad_s),
            left_actual_rpm=float(partial.left_actual_rpm),
            right_actual_rpm=float(partial.right_actual_rpm),
            target_vx_m_s=float(partial.target_vx_m_s),
            target_wz_rad_s=float(partial.target_wz_rad_s),
            left_target_rpm=float(partial.left_target_rpm),
            right_target_rpm=float(partial.right_target_rpm),
        )

    def _accept_sequence_and_timestamp(self, seq: int, timestamp_ms: Optional[int]) -> bool:
        if timestamp_ms is None:
            self.diagnostics.incomplete_groups += 1
            return False
        if self._last_seq is None or self._last_timestamp_ms is None:
            self._last_seq = seq
            self._last_timestamp_ms = timestamp_ms
            return True

        seq_delta = (seq - self._last_seq) & 0xFFFF
        timestamp_delta = (timestamp_ms - self._last_timestamp_ms) & 0xFFFFFFFF
        timestamp_forward = timestamp_delta < 0x80000000

        if seq_delta == 0:
            self.diagnostics.duplicate_sequences += 1
            return False

        seq_forward = seq_delta < 0x8000
        if not timestamp_forward and not seq_forward:
            # Both counters regressed rather than wrapped forward: STM32 reboot.
            self.diagnostics.device_resets += 1
            self._last_seq = seq
            self._last_timestamp_ms = timestamp_ms
            return True
        if not timestamp_forward:
            self.diagnostics.timestamp_regressions += 1
            return False
        if not seq_forward:
            self.diagnostics.sequence_regressions += 1
            return False

        self.diagnostics.dropped_sequences += seq_delta - 1
        self._last_seq = seq
        self._last_timestamp_ms = timestamp_ms
        return True
