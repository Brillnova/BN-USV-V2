import argparse
import time
from contextlib import suppress

from esp_link_v2 import EspMainLink


def main() -> None:
    parser = argparse.ArgumentParser(description="BN-USV V2 Pi-to-Main command sender test")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--throttle", type=float, default=0.0)
    parser.add_argument("--steering", type=float, default=0.0)
    args = parser.parse_args()

    period = 1.0 / args.rate
    esp = EspMainLink(args.port, args.baud)

    print(f"Connected to Main ESP32-S3 on {args.port} at {args.baud} baud")
    print("Press Ctrl+C to stop. A neutral command is sent on exit.")

    last_print = 0.0
    try:
        while True:
            loop_start = time.time()

            esp.send_manual_command(args.throttle, args.steering)
            telemetry = esp.poll()

            now = time.time()
            if now - last_print >= 1.0:
                last_print = now
                print(
                    "main=",
                    telemetry.main_status,
                    "left=",
                    telemetry.left_sub_status,
                    "right=",
                    telemetry.right_sub_status,
                )

            elapsed = time.time() - loop_start
            if elapsed < period:
                time.sleep(period - elapsed)

    except KeyboardInterrupt:
        print("\nStopping command sender...")
    finally:
        with suppress(Exception):
            for _ in range(5):
                esp.send_neutral()
                time.sleep(0.05)
        esp.close()
        print("Neutral sent. Serial closed.")


if __name__ == "__main__":
    main()
