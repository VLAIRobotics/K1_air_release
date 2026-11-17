#!/usr/bin/env python3
"""Bounded SI-velocity command sender for the V1.2.5 differential chassis.

This program is intentionally independent from the telemetry listener.  It is
the only process that may send CAN 0x050 while it is running; it refuses known
competing sender processes and holds a process-lifetime advisory lock in
``/tmp``.  The module has no module-level python-can import so ``--dry-run``
is a pure conversion check and never opens SocketCAN.
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
DEFAULT_RATE_HZ = 30.0
MIN_RATE_HZ = 10.0
MAX_RATE_HZ = 50.0
MAX_VX_M_S = 0.50
MAX_WZ_RAD_S = 1.60
RAW_LIMIT = 800
VY_RAW_PER_M_S = 1224.27
VW_RAW_PER_RAD_S = -459.10
STOP_SEND_SECONDS = 0.5
LOCK_PATH = Path("/tmp/xair-can2-0x050.lock")


class VelocityCommandError(RuntimeError):
    """Raised for an unsafe or invalid local command request."""


@dataclass(frozen=True)
class CanVelocityCommand:
    """The one V1.2.5 0x050 command generated from SI velocity input."""

    vx_m_s: float
    wz_rad_s: float
    vy_raw: int
    vw_raw: int
    payload: bytes


def round_half_away_from_zero(value: float) -> int:
    """Round a finite float with ties away from zero, independent of Python's round."""

    if not math.isfinite(value):
        raise VelocityCommandError("conversion input must be finite")
    if value >= 0.0:
        return math.floor(value + 0.5)
    return math.ceil(value - 0.5)


def clamp_raw(value: int) -> int:
    """Apply the lower-controller's signed raw command envelope."""

    return max(-RAW_LIMIT, min(RAW_LIMIT, value))


def build_command(vx_m_s: float, wz_rad_s: float) -> CanVelocityCommand:
    """Convert SI request values into the fixed little-endian 0x050 payload."""

    if not (math.isfinite(vx_m_s) and math.isfinite(wz_rad_s)):
        raise VelocityCommandError("--vx and --wz must be finite")

    vy_raw = clamp_raw(round_half_away_from_zero(VY_RAW_PER_M_S * vx_m_s))
    vw_raw = clamp_raw(round_half_away_from_zero(VW_RAW_PER_RAD_S * wz_rad_s))
    payload = struct.pack("<hhhBB", 0, vy_raw, vw_raw, 0, 0)
    return CanVelocityCommand(vx_m_s, wz_rad_s, vy_raw, vw_raw, payload)


def validate_request(args: argparse.Namespace) -> None:
    """Reject unsafe requests instead of silently reducing their requested speed."""

    for name in ("vx", "wz", "duration", "rate_hz"):
        if not math.isfinite(getattr(args, name)):
            raise VelocityCommandError(f"--{name.replace('_', '-')} must be finite")
    if abs(args.vx) > MAX_VX_M_S:
        raise VelocityCommandError(
            f"--vx must be within ±{MAX_VX_M_S:.2f} m/s (received {args.vx:g})"
        )
    if abs(args.wz) > MAX_WZ_RAD_S:
        raise VelocityCommandError(
            f"--wz must be within ±{MAX_WZ_RAD_S:.2f} rad/s (received {args.wz:g})"
        )
    if args.duration < 0.0:
        raise VelocityCommandError("--duration must not be negative")
    if not MIN_RATE_HZ <= args.rate_hz <= MAX_RATE_HZ:
        raise VelocityCommandError(
            f"--rate-hz must be within {MIN_RATE_HZ:g}..{MAX_RATE_HZ:g} Hz"
        )
    if (args.vx != 0.0 or args.wz != 0.0) and args.duration <= 0.0:
        raise VelocityCommandError("a non-zero --vx or --wz requires a positive --duration")


def _is_known_sender_argv(argv: Sequence[str]) -> bool:
    """Identify direct process invocations that can contend for CAN 0x050."""

    if not argv:
        return False
    if Path(argv[0]).name == "cansend":
        return True
    sender_scripts = {
        "chassis_listener.py",
        "chassis_control.py",
        "chassis_velocity_command.py",
    }
    return any(Path(argument).name in sender_scripts for argument in argv[1:])


def find_known_sender_processes(proc_root: Path = Path("/proc")) -> list[str]:
    """Return known competing sender command lines, excluding this process.

    This is intentionally a conservative check for the repository's known
    entry points.  SocketCAN cannot prove that an unrelated process is not
    sending the same CAN ID, so an operator must still perform passive 0x050
    monitoring before an actual vehicle test.
    """

    try:
        entries = list(proc_root.iterdir())
    except OSError as exc:
        raise VelocityCommandError(f"cannot inspect {proc_root} for competing senders: {exc}") from exc

    current_pid = os.getpid()
    conflicts: list[str] = []
    for entry in entries:
        if not entry.name.isdecimal() or int(entry.name) == current_pid:
            continue
        try:
            raw = (entry / "cmdline").read_bytes()
        except OSError:
            continue
        argv = [item.decode(errors="replace") for item in raw.split(b"\0") if item]
        if _is_known_sender_argv(argv):
            conflicts.append(f"pid {entry.name}: {' '.join(argv)}")
    return conflicts


class AdvisoryLock:
    """A non-blocking, process-held lock for this CLI's CAN 0x050 authority."""

    def __init__(self, path: Path = LOCK_PATH):
        self.path = path
        self._file = None

    def acquire(self) -> None:
        self._file = self.path.open("a+", encoding="utf-8")
        try:
            fcntl.flock(self._file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            self._file.close()
            self._file = None
            raise VelocityCommandError(
                f"another chassis_velocity_command instance holds {self.path}"
            ) from exc
        self._file.seek(0)
        self._file.truncate()
        self._file.write(f"pid={os.getpid()}\n")
        self._file.flush()

    def release(self) -> None:
        if self._file is not None:
            fcntl.flock(self._file.fileno(), fcntl.LOCK_UN)
            self._file.close()
            self._file = None

    def __enter__(self) -> "AdvisoryLock":
        self.acquire()
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.release()


def _open_can_bus():
    """Import python-can only for an explicitly requested real send path."""

    try:
        import can  # type: ignore[import-not-found]
    except ImportError as exc:
        raise VelocityCommandError(
            "python-can is required for real transmission; use --dry-run for conversion only"
        ) from exc
    return can.interface.Bus(channel=CAN_INTERFACE, interface="socketcan"), can


def _send_payload(bus, can_module, payload: bytes) -> None:
    """Send one standard, classic CAN frame with the fixed eight-byte layout."""

    message = can_module.Message(
        arbitration_id=CAN_ID,
        is_extended_id=False,
        is_fd=False,
        data=payload,
    )
    bus.send(message)


def send_for_duration(bus, can_module, payload: bytes, duration_sec: float, rate_hz: float) -> int:
    """Transmit at a monotonic fixed-rate schedule for a bounded duration."""

    period = 1.0 / rate_hz
    deadline = time.monotonic() + duration_sec
    next_send = time.monotonic()
    sent = 0
    while True:
        now = time.monotonic()
        if now >= deadline:
            break
        _send_payload(bus, can_module, payload)
        sent += 1
        next_send += period
        remaining = next_send - time.monotonic()
        if remaining > 0.0:
            time.sleep(remaining)
        else:
            # Do not accumulate scheduling debt: the next frame is scheduled
            # from the current monotonic time rather than burst-sending old ones.
            next_send = time.monotonic()
    return sent


def send_zero_sequence(
    bus,
    can_module,
    rate_hz: float,
    stop_seconds: float = STOP_SEND_SECONDS,
) -> int:
    """Send zero frames through and including the requested stop interval."""

    zero_payload = build_command(0.0, 0.0).payload
    period = 1.0 / rate_hz
    frame_count = max(1, math.ceil(stop_seconds * rate_hz) + 1)
    next_send = time.monotonic()
    for index in range(frame_count):
        _send_payload(bus, can_module, zero_payload)
        if index + 1 == frame_count:
            continue
        next_send += period
        remaining = next_send - time.monotonic()
        if remaining > 0.0:
            time.sleep(remaining)
        else:
            next_send = time.monotonic()
    return frame_count


def print_command(command: CanVelocityCommand, rate_hz: float, duration: float, dry_run: bool) -> None:
    """Print a complete auditable command conversion without sending it."""

    print(f"CAN interface: {CAN_INTERFACE}")
    print(f"CAN ID: 0x{CAN_ID:03X} (standard, classic CAN, DLC={len(command.payload)})")
    print(f"request: vx={command.vx_m_s:.6f} m/s, wz={command.wz_rad_s:.6f} rad/s")
    print(f"raw: vx=0, vy={command.vy_raw}, vw={command.vw_raw}, byte6=0, updown=0")
    print(f"payload: {command.payload.hex(' ')}")
    print(f"rate: {rate_hz:g} Hz; requested duration: {duration:g} s")
    if dry_run:
        print("dry-run: python-can was not imported and SocketCAN was not opened")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vx", type=float, default=0.0, help="forward velocity in m/s (±0.50)")
    parser.add_argument("--wz", type=float, default=0.0, help="counter-clockwise yaw rate in rad/s (±1.60)")
    parser.add_argument("--duration", type=float, default=0.0, help="bounded non-zero command duration in seconds")
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=DEFAULT_RATE_HZ,
        help="send rate in Hz (10..50; default: 30)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print conversion only; do not import python-can or access SocketCAN",
    )
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    args = build_parser().parse_args(list(argv) if argv is not None else None)
    try:
        validate_request(args)
        command = build_command(args.vx, args.wz)
        print_command(command, args.rate_hz, args.duration, args.dry_run)
        if args.dry_run:
            return 0

        conflicts = find_known_sender_processes()
        if conflicts:
            raise VelocityCommandError(
                "refusing CAN 0x050 transmission because known sender processes are running:\n"
                + "\n".join(conflicts)
            )

        bus = None
        can_module = None
        interrupted = False
        try:
            with AdvisoryLock():
                bus, can_module = _open_can_bus()
                if args.vx != 0.0 or args.wz != 0.0:
                    send_for_duration(bus, can_module, command.payload, args.duration, args.rate_hz)
                # Zero request is the entire normal operation when vx=wz=0.
        except KeyboardInterrupt:
            interrupted = True
            print("interrupted; entering zero-speed shutdown sequence", file=sys.stderr)
        finally:
            if bus is not None and can_module is not None:
                try:
                    send_zero_sequence(bus, can_module, args.rate_hz)
                except Exception as exc:  # Zero-send failure must not skip socket cleanup.
                    print(f"warning: could not complete zero-speed sequence: {exc}", file=sys.stderr)
                try:
                    bus.shutdown()
                except Exception as exc:
                    print(f"warning: SocketCAN shutdown failed: {exc}", file=sys.stderr)
        return 130 if interrupted else 0
    except VelocityCommandError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("interrupted before SocketCAN was opened", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"error: transmission failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
