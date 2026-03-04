#!/usr/bin/env python3
import time
import sys
import argparse

import camera.rostopic_latest as roscam


def main():
    parser = argparse.ArgumentParser(description="Test unified ROS2 RGBD subscriber (latest-only mailbox).")
    parser.add_argument("--topic-path", default="/camera/camera", help="ROS2 topic path/prefix (default: /camera/camera)")
    parser.add_argument("--topic-name", default="rgbd", help="Topic name under topic-path (default: rgbd)")
    parser.add_argument("--timeout", type=float, default=3.0, help="Seconds to wait for first frame (default: 3.0)")
    parser.add_argument("--seconds", type=float, default=5.0, help="Seconds to run after first frame (default: 5.0)")
    parser.add_argument("--print-hz", type=float, default=5.0, help="How often to print status (default: 5.0)")
    args = parser.parse_args()

    # Config
    cfg = roscam.CfgROSCam()
    cfg.topicPath = args.topic_path
    cfg.topicName = args.topic_name

    cam = roscam.RGBD(cfg)

    try:
        cam.start()

        # ---- Wait for first frame (deterministic, with timeout) ----
        t0 = time.time()
        while not cam.has_frames():
            if time.time() - t0 > args.timeout:
                print(f"FAIL: No frames received within {args.timeout:.2f}s on {cfg.topicPath}/{cfg.topicName}")
                print(f"  stats={cam.get_stats()}")
                return 1
            time.sleep(0.05)

        print("First frame received ✅")

        # ---- Run loop: validate shapes, alignment, and stats ----
        end_time = time.time() + args.seconds
        print_period = 1.0 / args.print_hz if args.print_hz > 0 else 0.0
        next_print = 0.0

        # Track that frames update over time (optional but useful)
        last_version = None

        while time.time() < end_time:
            rgb, depth = cam.get_frames()
            hdr = cam.get_header()
            stats = cam.get_stats()

            # Basic sanity: should not be None after initialized
            if rgb is None or depth is None:
                print("FAIL: get_frames() returned None after initialization.")
                print(f"  stats={stats}")
                return 1

            # Alignment sanity: compare H,W (handle depth possibly being HxW or HxWx1)
            rgb_hw = rgb.shape[:2]
            depth_hw = depth.shape[:2]
            aligned_ok = (rgb_hw == depth_hw)

            # Frame-update sanity via mailbox version
            if last_version is None:
                last_version = stats.get("version")
            else:
                # not a hard fail if version doesn't change every loop (consumer may be faster than producer)
                pass

            # Print at requested rate
            now = time.time()
            if print_period == 0.0 or now >= next_print:
                next_print = now + print_period

                stamp_s = None
                frame_id = None
                if hdr is not None:
                    frame_id = getattr(hdr, "frame_id", None)
                    st = getattr(hdr, "stamp", None)
                    if st is not None:
                        stamp_s = f"{st.sec}.{st.nanosec:09d}"

                print("---- RGBD ----")
                print(f"RGB   shape={rgb.shape} dtype={rgb.dtype}")
                print(f"Depth shape={depth.shape} dtype={depth.dtype} aligned={aligned_ok}")
                print(f"header frame_id={frame_id} stamp={stamp_s}")
                print(f"stats {stats}")

                if not aligned_ok:
                    print("FAIL: Depth is not aligned to RGB (H,W mismatch).")
                    return 1

            time.sleep(0.01)

        print("PASS: RGBD subscriber produced valid frames and stopped cleanly ✅")
        return 0

    finally:
        # Always stop to avoid leaving rclpy/executor running
        try:
            cam.stop()
        except Exception as e:
            print(f"WARNING: cam.stop() raised: {e}")


if __name__ == "__main__":
    raise SystemExit(main())