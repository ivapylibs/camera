#!/usr/bin/env python3
import time
import sys

import camera.rostopic_latest as roscam


def main():
    cfg = roscam.CfgROSCam()
    cfg.topicPath = "/camera/camera/color"
    cfg.topicName = "image_raw"

    cam = roscam.Color(cfg)
    cam.start()

    # Give subscriptions time to receive at least one message
    time.sleep(0.5)

    frame = cam.get_frames()
    hdr = cam.get_header()
    stats = cam.get_stats()

    cam.stop()

    # Minimal assertions / checks
    if frame is None:
        print("FAIL: Color get_frames() returned None (no frames received).")
        sys.exit(1)

    if hdr is None:
        print("FAIL: Color header is None (metadata not stored).")
        sys.exit(1)

    # Basic sanity (don’t over-assume channels)
    if frame.ndim < 2:
        print(f"FAIL: Color frame has unexpected ndim={frame.ndim}, shape={frame.shape}.")
        sys.exit(1)

    if str(frame.dtype) != "uint8":
        print(f"FAIL: Color dtype expected uint8, got {frame.dtype}.")
        sys.exit(1)

    print("PASS: Color subscriber OK")
    print(f"  shape={frame.shape} dtype={frame.dtype}")
    print(f"  frame_id={hdr.frame_id} stamp=({hdr.stamp.sec}.{hdr.stamp.nanosec:09d})")
    print(f"  stats={stats}")


if __name__ == "__main__":
    main()