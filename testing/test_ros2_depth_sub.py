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
    parser = argparse.ArgumentParser(description="Stream-test ROS2 Depth subscriber (latest-only mailbox).")
    parser.add_argument("--topic-path", default="/camera/camera/depth", help="ROS2 topic path (default: /camera/camera/depth)")
    parser.add_argument("--topic-name", default="image_rect_raw", help="Topic name (default: image_rect_raw)")
    parser.add_argument("--timeout", type=float, default=3.0, help="Seconds to wait for first frame (default: 3.0)")
    parser.add_argument("--seconds", type=float, default=5.0, help="Seconds to run after first frame (default: 5.0)")
    parser.add_argument("--print-hz", type=float, default=5.0, help="How often to print status (default: 5.0)")
    args = parser.parse_args()

    cfg = roscam.CfgROSCam()
    cfg.topicPath = args.topic_path
    cfg.topicName = args.topic_name

    cam = roscam.Depth(cfg)

    try:
        cam.start()

        # ---- Wait for first frame ----
        t0 = time.time()
        while not cam.has_frames():
            if time.time() - t0 > args.timeout:
                print(f"FAIL: No depth frames within {args.timeout:.2f}s on {cfg.topicPath}/{cfg.topicName}")
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
            depth = cam.get_frames()
            hdr = cam.get_header()
            stats = cam.get_stats()
            enc = stats.get("encoding", None)  # Depth.get_stats() provides "encoding"

            if depth is None:
                print("FAIL: Depth get_frames() returned None after initialization.")
                print(f"  stats={stats}")
                return 1

            if hdr is None:
                print("FAIL: Depth header is None after initialization.")
                print(f"  stats={stats}")
                return 1

            # Sanity checks: accept (H,W) or (H,W,1)
            if depth.ndim not in (2, 3):
                print(f"FAIL: Depth frame unexpected ndim={depth.ndim}, shape={depth.shape}")
                return 1

            # Dtype expectation based on encoding (when known)
            if enc in ("16UC1", "mono16", "", None):
                if str(depth.dtype) != "uint16":
                    print(f"FAIL: Depth dtype expected uint16 for encoding={enc}, got {depth.dtype}")
                    return 1
            elif enc == "32FC1":
                if str(depth.dtype) != "float32":
                    print(f"FAIL: Depth dtype expected float32 for encoding=32FC1, got {depth.dtype}")
                    return 1
            else:
                # Unknown encoding: don’t hard fail; just report it.
                pass

            # Timestamp monotonic (non-decreasing)
            stamp = _stamp_to_tuple(hdr)
            if stamp is not None and last_stamp is not None and stamp < last_stamp:
                print(f"FAIL: Depth header stamp went backwards: {stamp} < {last_stamp}")
                return 1
            if stamp is not None:
                last_stamp = stamp

            now = time.time()
            if print_period == 0.0 or now >= next_print:
                next_print = now + print_period
                st = hdr.stamp
                print("---- DEPTH ----")
                print(f"shape={depth.shape} dtype={depth.dtype} encoding={enc}")
                print(f"frame_id={hdr.frame_id} stamp=({st.sec}.{st.nanosec:09d})")
                print(f"stats {stats}")

            time.sleep(0.01)

        print("PASS: Depth subscriber produced valid frames and stopped cleanly ✅")
        return 0

    finally:
        try:
            cam.stop()
        except Exception as e:
            print(f"WARNING: cam.stop() raised: {e}")


if __name__ == "__main__":
    raise SystemExit(main())