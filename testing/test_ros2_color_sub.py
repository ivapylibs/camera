#!/usr/bin/env python3
import time
import argparse

import camera.rostopic_latest as roscam


def _stamp_to_tuple(hdr):
    if hdr is None or getattr(hdr, "stamp", None) is None:
        return None
    st = hdr.stamp
    return (int(st.sec), int(st.nanosec))


def main():
    parser = argparse.ArgumentParser(description="Stream-test ROS2 Color subscriber (latest-only mailbox).")
    parser.add_argument("--topic-path", default="/camera/camera/color", help="ROS2 topic path (default: /camera/camera/color)")
    parser.add_argument("--topic-name", default="image_raw", help="Topic name (default: image_raw)")
    parser.add_argument("--timeout", type=float, default=3.0, help="Seconds to wait for first frame (default: 3.0)")
    parser.add_argument("--seconds", type=float, default=5.0, help="Seconds to run after first frame (default: 5.0)")
    parser.add_argument("--print-hz", type=float, default=5.0, help="How often to print status (default: 5.0)")
    args = parser.parse_args()

    cfg = roscam.CfgROSCam()
    cfg.topicPath = args.topic_path
    cfg.topicName = args.topic_name

    cam = roscam.Color(cfg)

    try:
        cam.start()

        # ---- Wait for first frame ----
        t0 = time.time()
        while not cam.has_frames():
            if time.time() - t0 > args.timeout:
                print(f"FAIL: No color frames within {args.timeout:.2f}s on {cfg.topicPath}/{cfg.topicName}")
                print(f"  stats={cam.get_stats()}")
                return 1
            time.sleep(0.05)

        print("First frame received ✅")

        # ---- Stream loop ----
        end_time = time.time() + args.seconds
        print_period = 1.0 / args.print_hz if args.print_hz > 0 else 0.0
        next_print = 0.0

        last_stamp = None

        while time.time() < end_time:
            frame = cam.get_frames()
            hdr = cam.get_header()
            stats = cam.get_stats()

            if frame is None:
                print("FAIL: Color get_frames() returned None after initialization.")
                print(f"  stats={stats}")
                return 1

            if hdr is None:
                print("FAIL: Color header is None after initialization.")
                print(f"  stats={stats}")
                return 1

            # Sanity checks
            if frame.ndim < 2:
                print(f"FAIL: Color frame unexpected ndim={frame.ndim}, shape={frame.shape}")
                return 1

            if str(frame.dtype) != "uint8":
                print(f"FAIL: Color dtype expected uint8, got {frame.dtype}")
                return 1

            # Timestamp monotonic (non-decreasing)
            stamp = _stamp_to_tuple(hdr)
            if stamp is not None and last_stamp is not None and stamp < last_stamp:
                print(f"FAIL: Color header stamp went backwards: {stamp} < {last_stamp}")
                return 1
            if stamp is not None:
                last_stamp = stamp

            now = time.time()
            if print_period == 0.0 or now >= next_print:
                next_print = now + print_period
                st = hdr.stamp
                print("---- COLOR ----")
                print(f"shape={frame.shape} dtype={frame.dtype}")
                print(f"frame_id={hdr.frame_id} stamp=({st.sec}.{st.nanosec:09d})")
                print(f"stats {stats}")

            time.sleep(0.01)

        print("PASS: Color subscriber produced valid frames and stopped cleanly ✅")
        return 0

    finally:
        try:
            cam.stop()
        except Exception as e:
            print(f"WARNING: cam.stop() raised: {e}")


if __name__ == "__main__":
    raise SystemExit(main())