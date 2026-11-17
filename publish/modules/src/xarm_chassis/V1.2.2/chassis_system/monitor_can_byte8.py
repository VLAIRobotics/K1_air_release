#!/usr/bin/env python3
"""只读显示 can2 / 0x051 帧的第 8 个数据字节及其 bit 变化。"""

import argparse
from datetime import datetime

import can


def describe_bits(value: int) -> str:
    """按 bit7 到 bit0 的顺序显示一个字节。"""
    return " ".join(f"bit{index}={(value >> index) & 1}" for index in range(7, -1, -1))


def changed_bits(previous: int | None, current: int) -> str:
    if previous is None:
        return "首次收到"
    changed = [f"bit{index}" for index in range(7, -1, -1)
               if ((previous ^ current) & (1 << index))]
    return ", ".join(changed) if changed else "无变化"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="只读监听 CAN 0x051，并显示第 8 个数据字节的 bit 变化。"
    )
    parser.add_argument("--channel", default="can2", help="SocketCAN 接口，默认 can2")
    parser.add_argument("--id", default="0x051", help="要监听的 CAN ID，默认 0x051")
    parser.add_argument(
        "--all", action="store_true", help="即使第 8 字节未变化也打印每一帧"
    )
    args = parser.parse_args()
    can_id = int(args.id, 0)

    try:
        bus = can.interface.Bus(channel=args.channel, interface="socketcan")
    except (can.CanError, OSError) as error:
        print(f"[ERROR] 无法打开 {args.channel}: {error}")
        print("请确认 CAN 接口存在、已启用，且已安装 python-can。")
        return 2

    previous_byte8: int | None = None

    print(f"只读监听 {args.channel} / CAN 0x{can_id:03X}；按 Ctrl+C 退出。")
    print("输出顺序：bit7 bit6 bit5 bit4 bit3 bit2 bit1 bit0")

    try:
        while True:
            message = bus.recv(timeout=1.0)
            if message is None:
                continue
            if message.arbitration_id != can_id:
                continue

            frame_timestamp = message.timestamp

            if len(message.data) < 8:
                print(f"[WARN] 收到长度仅 {len(message.data)} 的 0x{can_id:03X} 帧，忽略。")
                continue

            byte8 = message.data[7]
            if not args.all and previous_byte8 == byte8:
                continue

            raw = " ".join(f"{value:02X}" for value in message.data)
            timestamp = datetime.fromtimestamp(frame_timestamp).strftime("%H:%M:%S.%f")[:-3]
            print(
                f"[{timestamp}] 原始帧: {raw}\n"
                f"  第8字节: 0x{byte8:02X} = {byte8:3d} = {byte8:08b}\n"
                f"  位状态: {describe_bits(byte8)}\n"
                f"  本次变化: {changed_bits(previous_byte8, byte8)}"
            )
            previous_byte8 = byte8
    except KeyboardInterrupt:
        print("\n已停止监听。")
    finally:
        bus.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
