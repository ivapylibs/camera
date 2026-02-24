#!/usr/bin/env python3
import time
import sys

import camera.rostopic_latest as roscam


def main():
    cfg = roscam.CfgROSCam()
    cfg.topicPath = "/camera/camera/depth"
    cfg.topicName = "image_rect_raw"

    cam = roscam.Depth(cfg)
    cam.start()

    time.sleep(0.5)

    depth = cam.get_frames()
    hdr = cam.get_header()
    stats = cam.get_stats()

    cam.stop()

    if depth is None:
        print("FAIL: Depth get_frames() returned None (no frames received).")
        sys.exit(1)

    if hdr is None:
        print("FAIL: Depth header is None (metadata not stored).")
        sys.exit(1)

    # Depth is often (H,W,1) in your conversion; accept either (H,W) or (H,W,1)
    if depth.ndim not in (2, 3):
        print(f"FAIL: Depth frame has unexpected ndim={depth.ndim}, shape={depth.shape}.")
        sys.exit(1)

    # Encoding is 16UC1 on your system; dtype should therefore be uint16.
    if str(depth.dtype) != "uint16":
        print(f"FAIL: Depth dtype expected uint16 (16UC1), got {depth.dtype}.")
        sys.exit(1)

    print("PASS: Depth subscriber OK")
    print(f"  shape={depth.shape} dtype={depth.dtype}")
    print(f"  frame_id={hdr.frame_id} stamp=({hdr.stamp.sec}.{hdr.stamp.nanosec:09d})")
    print(f"  stats={stats}")


if __name__ == "__main__":
    main()