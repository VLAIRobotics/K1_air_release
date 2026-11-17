#!/usr/bin/env python3
"""Bounded SI-velocity sender for the K1 omni chassis CAN ``0x050`` command.

This program is intentionally independent from the receive-only V1.2.6
telemetry listener. It is the only process that may send CAN 0x050 while it
is running: it refuses known competing senders and holds the shared advisory
lock in ``/tmp``. The module has no module-level python-can import, so
``--dry-run`` performs only deterministic conversion and safety checks.

The conversion is derived from the current K1 STM32 source:

* ``can_spd_input.{vx,vy,vw} = int16_le(0x050)``;
* AUTO mode applies ``0.01`` to each raw value;
* the four wheel targets use the mecanum mixing matrix; and
* telemetry ``0x516`` inverts the final wheel targets using R=0.075 m,
  K=0.18844+0.18844 m and sqrt(2) roller projection.

SI requests and coordinate-corrected telemetry use X forward, Y left and
counter-clockwise yaw positive. The legacy 0x050 lateral command remains
right-positive, so only the outgoing Y request is negated. This sender must
be paired with the coordinate-corrected STM32 firmware; V3 does not identify
the old/new coordinate convention automatically.

Byte 7 is the STM32 ``updown`` GPIO-control field, not a sequence counter.
It is consequently fixed to its safe neutral value, zero, in every frame.
"""

from __future__ import annotations

import argparse
import fcntl
import math
import os
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


CAN_INTERFACE = "can2"
CAN_ID = 0x050
DEFAULT_RATE_HZ = 50.0
MIN_RATE_HZ = 20.0
MAX_RATE_HZ = 100.0
STOP_SEND_SECONDS = 0.5
LOCK_PATH = Path("/tmp/xair-can2-0x050.lock")

# Values read from VLAI_CHASSIS/onmi_car/User/app/chassis_task.c.
COMMAND_SCALE = 0.01
WHEEL_RADIUS_M = 0.075
HALF_LENGTH_M = 0.18844
HALF_WIDTH_M = 0.18844
CHASSIS_K_M = HALF_LENGTH_M + HALF_WIDTH_M
ROLLER_SCALE = math.sqrt(2.0)
WHEEL_SPEED_MAX_RAD_S = 8.0
RAW_LIMIT = 800

# Axis-wise user limits, doubled with the STM32 wheel limit from 4 to 8 rad/s.
# This does not change the raw-to-SI mapping. A separate four-wheel check below
# rejects any mixed vector that would otherwise trigger STM32 normalization.
MAX_VX_M_S = 0.80
MAX_VY_M_S = 0.80
MAX_WZ_RAD_S = 2.00

# Conversion magnitudes before normalization / ramp. The legacy raw Y axis
# is right-positive; _raw_from_si converts the left-positive SI request once.
LINEAR_M_S_PER_RAW = COMMAND_SCALE * ROLLER_SCALE * WHEEL_RADIUS_M
ANGULAR_RAD_S_PER_RAW = COMMAND_SCALE * ROLLER_SCALE * WHEEL_RADIUS_M / CHASSIS_K_M
RAW_PER_LINEAR_M_S = 1.0 / LINEAR_M_S_PER_RAW
RAW_PER_ANGULAR_RAD_S = -1.0 / ANGULAR_RAD_S_PER_RAW


class OmniVelocityCommandError(RuntimeError):
    """Raised for an unsafe or invalid local K1 velocity request."""


@dataclass(frozen=True)
class CanOmniVelocityCommand:
    """One SI request and its fixed eight-byte K1 0x050 command."""

    vx_m_s: float
    vy_m_s: float
    wz_rad_s: float
    vx_raw: int
    vy_raw: int
    vw_raw: int
    payload: bytes


def round_half_away_from_zero(value: float) -> int:
    """Round a finite float with ties away from zero, independent of ``round``."""

    if not math.isfinite(value):
        raise OmniVelocityCommandError("conversion input must be finite")
    return math.floor(value + 0.5) if value >= 0.0 else math.ceil(value - 0.5)


def clamp_raw(value: int) -> int:
    """Apply the STM32 signed raw receive envelope."""

    return max(-RAW_LIMIT, min(RAW_LIMIT, value))


def _raw_from_si(vx_m_s: float, vy_m_s: float, wz_rad_s: float) -> tuple[int, int, int]:
    if not all(math.isfinite(value) for value in (vx_m_s, vy_m_s, wz_rad_s)):
        raise OmniVelocityCommandError("--vx, --vy and --wz must be finite")
    return (
        clamp_raw(round_half_away_from_zero(RAW_PER_LINEAR_M_S * vx_m_s)),
        clamp_raw(round_half_away_from_zero(-RAW_PER_LINEAR_M_S * vy_m_s)),
        clamp_raw(round_half_away_from_zero(RAW_PER_ANGULAR_RAD_S * wz_rad_s)),
    )


def wheel_speed_demands(raw_vx: int, raw_vy: int, raw_vw: int) -> tuple[float, float, float, float]:
    """Return pre-normalization wheel demands using the STM32 mixing matrix."""

    vx = raw_vx * COMMAND_SCALE
    vy = raw_vy * COMMAND_SCALE
    vw = raw_vw * COMMAND_SCALE
    return (-vx + vy + vw, vx + vy + vw, vx - vy + vw, -vx - vy + vw)


def build_command(vx_m_s: float, vy_m_s: float, wz_rad_s: float) -> CanOmniVelocityCommand:
    """Convert an SI body-velocity request into the fixed little-endian payload."""

    vx_raw, vy_raw, vw_raw = _raw_from_si(vx_m_s, vy_m_s, wz_rad_s)
    # byte6 is unused by the current STM32 receiver; byte7 is updown and must
    # stay neutral. Neither byte is a host sequence counter.
    payload = struct.pack("<hhhBB", vx_raw, vy_raw, vw_raw, 0, 0)
    return CanOmniVelocityCommand(vx_m_s, vy_m_s, wz_rad_s, vx_raw, vy_raw, vw_raw, payload)


def _validate_wheel_vector(vx_m_s: float, vy_m_s: float, wz_rad_s: float) -> None:
    raw = _raw_from_si(vx_m_s, vy_m_s, wz_rad_s)
    max_wheel_speed = max(abs(value) for value in wheel_speed_demands(*raw))
    if max_wheel_speed > WHEEL_SPEED_MAX_RAD_S:
        raise OmniVelocityCommandError(
            "requested velocity vector requires a wheel target of "
            f"{max_wheel_speed:.3f} rad/s, above the STM32 {WHEEL_SPEED_MAX_RAD_S:.1f} rad/s limit; "
            "refusing to silently normalize the requested motion"
        )


def validate_request(args: argparse.Namespace) -> None:
    """Reject unsafe requests instead of silently reducing their SI velocity."""

    for name in ("vx", "vy", "wz", "duration", "rate_hz"):
        if not math.isfinite(getattr(args, name)):
            raise OmniVelocityCommandError(f"--{name.replace('_', '-')} must be finite")
    for name, limit, unit in (
        ("vx", MAX_VX_M_S, "m/s"),
        ("vy", MAX_VY_M_S, "m/s"),
        ("wz", MAX_WZ_RAD_S, "rad/s"),
    ):
        value = getattr(args, name)
        if abs(value) > limit:
            raise OmniVelocityCommandError(
                f"--{name} must be within ±{limit:.2f} {unit} (received {value:g})"
            )
    if args.duration < 0.0:
        raise OmniVelocityCommandError("--duration must not be negative")
    if not MIN_RATE_HZ <= args.rate_hz <= MAX_RATE_HZ:
        raise OmniVelocityCommandError(f"--rate-hz must be within {MIN_RATE_HZ:g}..{MAX_RATE_HZ:g} Hz")
    if any(getattr(args, name) != 0.0 for name in ("vx", "vy", "wz")) and args.duration <= 0.0:
        raise OmniVelocityCommandError("a non-zero --vx, --vy or --wz requires a positive --duration")
    _validate_wheel_vector(args.vx, args.vy, args.wz)


def _is_known_sender_argv(argv: Sequence[str]) -> bool:
    if not argv:
        return False
    if Path(argv[0]).name == "cansend":
        return True
    script_names = {"chassis_control.py", "chassis_velocity_command.py", "chassis_omni_velocity_command.py"}
    if any(Path(argument).name in script_names for argument in argv[1:]):
        return True
    # V1.2.5 listener has a joystick-driven 0x050 path. The V1.2.6 listener
    # is deliberately receive-only and is safe to run with this K1 CLI.
    return any(
        Path(argument).name == "chassis_listener.py" and "V1.2.6/chassis_system" not in argument
        for argument in argv[1:]
    )


def find_known_sender_processes(proc_root: Path = Path("/proc")) -> list[str]:
    conflicts: list[str] = []
    for entry in proc_root.iterdir():
        if not entry.name.isdecimal() or int(entry.name) == os.getpid():
            continue
        try:
            argv = [item.decode(errors="replace") for item in (entry / "cmdline").read_bytes().split(b"\0") if item]
        except OSError:
            continue
        if _is_known_sender_argv(argv):
            conflicts.append(f"pid {entry.name}: {' '.join(argv)}")
    return conflicts


class AdvisoryLock:
    def __init__(self, path: Path = LOCK_PATH) -> None:
        self.path = path
        self._file = None

    def acquire(self) -> None:
        self._file = self.path.open("a+", encoding="utf-8")
        try:
            fcntl.flock(self._file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            self._file.close()
            self._file = None
            raise OmniVelocityCommandError(f"another 0x050 velocity CLI holds {self.path}") from error
        self._file.seek(0)
        self._file.truncate()
        self._file.write(f"pid={os.getpid()}\n")
        self._file.flush()

    def release(self) -> None:
        if self._file is not None:
            fcntl.flock(self._file.fileno(), fcntl.LOCK_UN)
            self._file.close()
            self._file = None

    def __enter__(self):
        self.acquire()
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.release()


def _open_can_bus():
    try:
        import can  # type: ignore[import-not-found]
    except ImportError as error:
        raise OmniVelocityCommandError("python-can is required for real transmission; use --dry-run") from error
    return can.interface.Bus(channel=CAN_INTERFACE, interface="socketcan"), can


def _send_payload(bus, can_module, payload: bytes) -> None:
    bus.send(can_module.Message(arbitration_id=CAN_ID, is_extended_id=False, is_fd=False, data=payload))


def send_for_duration(bus, can_module, payload: bytes, duration: float, rate_hz: float) -> int:
    period = 1.0 / rate_hz
    deadline = time.monotonic() + duration
    next_send = time.monotonic()
    sent = 0
    while time.monotonic() < deadline:
        _send_payload(bus, can_module, payload)
        sent += 1
        next_send += period
        if (remaining := next_send - time.monotonic()) > 0.0:
            time.sleep(remaining)
        else:
            next_send = time.monotonic()
    return sent


def send_zero_sequence(bus, can_module, rate_hz: float) -> int:
    frames = max(1, math.ceil(STOP_SEND_SECONDS * rate_hz) + 1)
    period = 1.0 / rate_hz
    next_send = time.monotonic()
    zero_payload = build_command(0.0, 0.0, 0.0).payload
    for index in range(frames):
        _send_payload(bus, can_module, zero_payload)
        if index + 1 < frames:
            next_send += period
            if (remaining := next_send - time.monotonic()) > 0.0:
                time.sleep(remaining)
            else:
                next_send = time.monotonic()
    return frames


def print_command(command: CanOmniVelocityCommand, rate_hz: float, duration: float, dry_run: bool) -> None:
    print(f"CAN interface: {CAN_INTERFACE}")
    print(f"CAN ID: 0x{CAN_ID:03X} (standard, classic CAN, DLC={len(command.payload)})")
    print(
        "request: "
        f"vx={command.vx_m_s:.6f} m/s, vy={command.vy_m_s:.6f} m/s, wz={command.wz_rad_s:.6f} rad/s"
    )
    print(
        f"raw: vx={command.vx_raw}, vy={command.vy_raw}, vw={command.vw_raw}, "
        "byte6=0, updown=0"
    )
    print(f"payload: {command.payload.hex(' ')}")
    print(f"rate: {rate_hz:g} Hz; requested duration: {duration:g} s")
    if dry_run:
        print("dry-run: python-can was not imported and SocketCAN was not opened")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vx", type=float, default=0.0, help=f"body x velocity in m/s (±{MAX_VX_M_S:.2f}); forward positive")
    parser.add_argument("--vy", type=float, default=0.0, help=f"body y velocity in m/s (±{MAX_VY_M_S:.2f}); left positive; legacy CAN Y is negated")
    parser.add_argument("--wz", type=float, default=0.0, help=f"counter-clockwise yaw rate in rad/s (±{MAX_WZ_RAD_S:.2f})")
    parser.add_argument("--duration", type=float, default=0.0, help="bounded non-zero command duration in seconds")
    parser.add_argument("--rate-hz", type=float, default=DEFAULT_RATE_HZ, help="send rate in Hz (20..100; default: 50)")
    parser.add_argument("--dry-run", action="store_true", help="print conversion only; do not import python-can or access SocketCAN")
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    args = build_parser().parse_args(list(argv) if argv is not None else None)
    try:
        validate_request(args)
        command = build_command(args.vx, args.vy, args.wz)
        print_command(command, args.rate_hz, args.duration, args.dry_run)
        if args.dry_run:
            return 0
        conflicts = find_known_sender_processes()
        if conflicts:
            raise OmniVelocityCommandError("refusing CAN 0x050 transmission because known sender processes are running:\n" + "\n".join(conflicts))
        bus = can_module = None
        interrupted = False
        try:
            with AdvisoryLock():
                bus, can_module = _open_can_bus()
                if any(value != 0.0 for value in (args.vx, args.vy, args.wz)):
                    send_for_duration(bus, can_module, command.payload, args.duration, args.rate_hz)
        except KeyboardInterrupt:
            interrupted = True
            print("interrupted; entering zero-command shutdown sequence", file=sys.stderr)
        finally:
            if bus is not None and can_module is not None:
                try:
                    send_zero_sequence(bus, can_module, args.rate_hz)
                except Exception as error:
                    print(f"warning: could not complete zero-command sequence: {error}", file=sys.stderr)
                try:
                    bus.shutdown()
                except Exception as error:
                    print(f"warning: SocketCAN shutdown failed: {error}", file=sys.stderr)
        return 130 if interrupted else 0
    except OmniVelocityCommandError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("interrupted before SocketCAN was opened", file=sys.stderr)
        return 130
    except Exception as error:
        print(f"error: transmission failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
