import argparse
import sys
import time
from typing import Iterable, List

import gvar_can
import ota


def parse_can_id(raw: str) -> int:
    """Parse a CAN standard ID in decimal or hex form (e.g., 258 or 0x102)."""
    value = int(raw.strip(), 0)
    if value < 0 or value > 0x7FF:
        raise argparse.ArgumentTypeError(
            f"Invalid CAN ID '{raw}': must be in [0, 0x7FF] for standard IDs"
        )
    return value


def parse_can_id_list(raw_values: Iterable[str]) -> List[int]:
    """Parse one or more CLI tokens into a CAN ID list.

    Supports both:
    - space-separated tokens: --ids 0x102 0x103
    - comma-separated values: --ids 0x102,0x103
    """
    parsed: List[int] = []
    for token in raw_values:
        for part in token.split(","):
            stripped = part.strip()
            if not stripped:
                continue
            parsed.append(parse_can_id(stripped))

    if not parsed:
        raise argparse.ArgumentTypeError("No valid actuator IDs were provided")

    # Keep order while removing duplicates.
    return list(dict.fromkeys(parsed))


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Batch CAN OTA uploader for multiple actuator IDs"
    )
    parser.add_argument(
        "--firmware",
        default="Valve_not_embedded_XL.bin",
        help="Path to firmware .bin file",
    )
    parser.add_argument(
        "--port",
        default=ota.COMPORT,
        help=f"Serial CAN port for slcan adapter (default: {ota.COMPORT})",
    )
    parser.add_argument(
        "--bitrate",
        type=int,
        default=gvar_can.CAN_BITRATE,
        help=f"CAN bitrate (default: {gvar_can.CAN_BITRATE})",
    )
    parser.add_argument(
        "--tty-baudrate",
        type=int,
        default=gvar_can.SLCAN_TTY_BAUDRATE,
        help=f"SLCAN serial baudrate (default: {gvar_can.SLCAN_TTY_BAUDRATE})",
    )
    parser.add_argument(
        "--delay",
        type=float,
        default=0.5,
        help="Delay in seconds between actuators (default: 0.5)",
    )
    parser.add_argument(
        "--continue-on-fail",
        action="store_true",
        help="Continue updating remaining actuators if one fails",
    )
    return parser


def main(actuator_ids: List[int]) -> int:
    parser = build_arg_parser()
    args = parser.parse_args()

    if not actuator_ids:
        parser.error("No actuator IDs provided to main()")

    print("=== CAN OTA Batch Uploader ===")
    print(f"Firmware: {args.firmware}")
    print(f"Port: {args.port}, Bitrate: {args.bitrate}")
    print("Targets:", ", ".join(hex(aid) for aid in actuator_ids))

    try:
        bus = gvar_can.open_slcan_bus(
            args.port,
            bitrate=args.bitrate,
            tty_baudrate=args.tty_baudrate,
        )
    except Exception as exc:
        print(f"[ERROR] Could not open CAN bus on {args.port}: {exc}")
        return 2

    failures: List[int] = []
    try:
        for index, actuator_id in enumerate(actuator_ids, start=1):
            print(f"\n[{index}/{len(actuator_ids)}] OTA -> {hex(actuator_id)}")
            success = ota.perform_can_ota_on_bus(bus, actuator_id, args.firmware, log_fn=print)

            if success:
                print(f"[OK] {hex(actuator_id)} updated successfully")
            else:
                print(f"[FAIL] {hex(actuator_id)} update failed")
                failures.append(actuator_id)
                if not args.continue_on_fail:
                    print("Stopping after first failure. Use --continue-on-fail to keep going.")
                    break

            if index < len(actuator_ids) and args.delay > 0:
                time.sleep(args.delay)
    finally:
        bus.shutdown()

    print("\n=== Batch OTA Result ===")
    if failures:
        print("Failed IDs:", ", ".join(hex(aid) for aid in failures))
        print("Succeeded IDs:", ", ".join(hex(aid) for aid in actuator_ids if aid not in failures))
        return 1

    print("All target actuators updated successfully")
    return 0


if __name__ == "__main__":
    # Edit this list for your target actuators.
    # target_ids = parse_can_id_list(["0x101", "0x102", "0x103", "0x104","0x105", "0x106", "0x107", "0x108"])
    target_ids = parse_can_id_list(["0x104"])
    sys.exit(main(target_ids))
