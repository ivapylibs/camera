#!/usr/bin/env python3

from __future__ import annotations

import argparse
import sys
import time

import cv2
import numpy as np


def ok(name: str, msg: str = "") -> bool:
    print(f"[PASS] {name}: {msg}")
    return True


def fail(name: str, msg: str = "") -> bool:
    print(f"[FAIL] {name}: {msg}")
    return False


def check_color(name: str, img, expected_hw=(480, 640)) -> bool:
    if img is None:
        return fail(name, "frame is None")
    if not isinstance(img, np.ndarray):
        return fail(name, f"not ndarray: {type(img)}")
    if img.ndim != 3 or img.shape[2] != 3:
        return fail(name, f"expected HxWx3 color image, got {img.shape}")
    if img.shape[:2] != expected_hw:
        return fail(name, f"expected {expected_hw}, got {img.shape[:2]}")
    if not np.isfinite(img).all():
        return fail(name, "contains NaN or inf")
    return ok(name, f"shape={img.shape}, dtype={img.dtype}")


def check_depth_meters(name: str, depth, expected_hw=(480, 640)) -> bool:
    if depth is None:
        return fail(name, "depth is None")
    if not isinstance(depth, np.ndarray):
        return fail(name, f"not ndarray: {type(depth)}")
    if depth.ndim != 2:
        return fail(name, f"expected 2D depth, got {depth.shape}")
    if depth.shape != expected_hw:
        return fail(name, f"expected {expected_hw}, got {depth.shape}")
    if not np.isfinite(depth).all():
        return fail(name, "contains NaN or inf")

    valid = depth[depth > 0]
    if valid.size == 0:
        return fail(name, "no positive depth values")

    median = float(np.median(valid))
    valid_ratio = valid.size / depth.size

    if not (0.1 <= median <= 10.0):
        return fail(name, f"suspicious median depth: {median:.3f} m")
    if valid_ratio < 0.05:
        return fail(name, f"too few valid pixels: {valid_ratio:.3f}")

    return ok(
        name,
        f"shape={depth.shape}, dtype={depth.dtype}, "
        f"valid={valid_ratio:.2f}, median={median:.3f} m",
    )


def check_depth_normalized(name: str, depth, expected_hw=(480, 640)) -> bool:
    if depth is None:
        return fail(name, "depth is None")
    if not isinstance(depth, np.ndarray):
        return fail(name, f"not ndarray: {type(depth)}")
    if depth.ndim != 2:
        return fail(name, f"expected 2D depth, got {depth.shape}")
    if depth.shape != expected_hw:
        return fail(name, f"expected {expected_hw}, got {depth.shape}")
    if depth.dtype != np.uint8:
        return fail(name, f"expected uint8 normalized depth, got {depth.dtype}")

    return ok(name, f"shape={depth.shape}, dtype={depth.dtype}, min={depth.min()}, max={depth.max()}")


def check_intrinsics(name: str, K, dist) -> bool:
    K = np.asarray(K)
    dist = np.asarray(dist)

    if K.shape != (3, 3):
        return fail(name, f"K should be 3x3, got {K.shape}")
    if dist.ndim != 1 or dist.size < 4:
        return fail(name, f"dist should be 1D with >=4 coeffs, got {dist.shape}")
    if not np.isfinite(K).all() or not np.isfinite(dist).all():
        return fail(name, "K or dist contains NaN/inf")
    if K[0, 0] <= 0 or K[1, 1] <= 0:
        return fail(name, f"invalid fx/fy: {K[0,0]}, {K[1,1]}")

    return ok(name, f"fx={K[0,0]:.2f}, fy={K[1,1]:.2f}, cx={K[0,2]:.2f}, cy={K[1,2]:.2f}")


def test_direct_freenect() -> list[bool]:
    print("\n=== Direct libfreenect ===")
    results = []

    try:
        import freenect
        results.append(ok("import freenect"))
    except Exception as exc:
        results.append(fail("import freenect", repr(exc)))
        return results

    try:
        color, _ = freenect.sync_get_video()
        depth, _ = freenect.sync_get_depth()

        results.append(check_color("freenect.sync_get_video()", np.asarray(color)))
        if depth is None:
            results.append(fail("freenect.sync_get_depth()", "depth is None"))
        else:
            depth = np.asarray(depth)
            good = depth.ndim == 2 and depth.shape == (480, 640)
            results.append(ok("freenect.sync_get_depth()", f"shape={depth.shape}, dtype={depth.dtype}") if good
                           else fail("freenect.sync_get_depth()", f"bad shape={depth.shape}"))

        try:
            freenect.sync_stop()
        except Exception:
            pass

    except Exception as exc:
        results.append(fail("direct freenect capture", repr(exc)))

    return results


def test_color_api(display: bool, seconds: float) -> list[bool]:
    print("\n=== Color API ===")
    results = []

    try:
        from camera.kinect import Color
        cam = Color()
        results.append(ok("Color() constructor"))

        results.append(ok("Color.start()") if cam.start() else fail("Color.start()"))

        w, h = cam.getImagePixelWidthAndHeight()
        expected_hw = (h, w)
        results.append(ok("Color.getImagePixelWidthAndHeight()", f"width={w}, height={h}"))

        K, dist = cam.getCameraIntrinsics()
        results.append(check_intrinsics("Color.getCameraIntrinsics()", K, dist))

        results.append(check_color("Color.get_frames()", cam.get_frames(), expected_hw))
        results.append(check_color("Color.capture()", cam.capture(), expected_hw))

        if display:
            t0 = time.time()
            while time.time() - t0 < seconds:
                frame = cam.get_frames()
                cv2.imshow("Color", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            cv2.destroyWindow("Color")

        cam.stop()
        results.append(ok("Color.stop()"))

    except Exception as exc:
        results.append(fail("Color API", repr(exc)))

    return results


def test_depth_api(display: bool, seconds: float) -> list[bool]:
    print("\n=== Depth API ===")
    results = []

    try:
        from camera.kinect import Depth
        cam = Depth()
        results.append(ok("Depth() constructor"))

        results.append(ok("Depth.start()") if cam.start() else fail("Depth.start()"))

        w, h = cam.getImagePixelWidthAndHeight()
        expected_hw = (h, w)
        results.append(ok("Depth.getImagePixelWidthAndHeight()", f"width={w}, height={h}"))

        K, dist = cam.getCameraIntrinsics()
        results.append(check_intrinsics("Depth.getCameraIntrinsics()", K, dist))

        if hasattr(cam, "getDepthCameraIntrinsics"):
            depth_K, depth_dist = cam.getDepthCameraIntrinsics()
            results.append(check_intrinsics("Depth.getDepthCameraIntrinsics()", depth_K, depth_dist))

        results.append(check_depth_meters("Depth.get_frames(False)", cam.get_frames(normalization=False), expected_hw))
        results.append(check_depth_normalized("Depth.get_frames(True)", cam.get_frames(normalization=True), expected_hw))
        results.append(check_depth_meters("Depth.capture()", cam.capture(), expected_hw))

        if display:
            t0 = time.time()
            while time.time() - t0 < seconds:
                depth_vis = cam.get_frames(normalization=True)
                cv2.imshow("Depth", depth_vis)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            cv2.destroyWindow("Depth")

        cam.stop()
        results.append(ok("Depth.stop()"))

    except Exception as exc:
        results.append(fail("Depth API", repr(exc)))

    return results


def test_rgbd_api(display: bool, seconds: float) -> list[bool]:
    print("\n=== RGBD API ===")
    results = []

    try:
        from camera.kinect import RGBD
        cam = RGBD()
        results.append(ok("RGBD() constructor"))

        results.append(ok("RGBD.start()") if cam.start() else fail("RGBD.start()"))

        w, h = cam.getImagePixelWidthAndHeight()
        expected_hw = (h, w)
        results.append(ok("RGBD.getImagePixelWidthAndHeight()", f"width={w}, height={h}"))

        K, dist = cam.getCameraIntrinsics()
        results.append(check_intrinsics("RGBD.getCameraIntrinsics()", K, dist))

        color, depth_m = cam.get_frames(normalization=False)
        results.append(check_color("RGBD.get_frames(False) color", color, expected_hw))
        results.append(check_depth_meters("RGBD.get_frames(False) depth", depth_m, expected_hw))

        color, depth_vis = cam.get_frames(normalization=True)
        results.append(check_color("RGBD.get_frames(True) color", color, expected_hw))
        results.append(check_depth_normalized("RGBD.get_frames(True) depth", depth_vis, expected_hw))

        images = cam.capture(normalization=True)
        results.append(ok("RGBD.capture() has .color/.depth") if hasattr(images, "color") and hasattr(images, "depth")
                       else fail("RGBD.capture() has .color/.depth"))
        results.append(check_color("RGBD.capture().color", images.color, expected_hw))
        results.append(check_depth_normalized("RGBD.capture().depth", images.depth, expected_hw))

        count = 0
        t0 = time.time()
        while time.time() - t0 < max(seconds, 2.0):
            color, depth = cam.get_frames(normalization=False)
            if color is None or depth is None:
                results.append(fail("RGBD FPS", "None frame during FPS test"))
                break
            count += 1
        else:
            elapsed = time.time() - t0
            results.append(ok("RGBD FPS", f"{count / elapsed:.2f} fps over {elapsed:.2f}s"))

        if display:
            t0 = time.time()
            while time.time() - t0 < seconds:
                color, depth_vis = cam.get_frames(normalization=True)
                cv2.imshow("RGBD Color", color)
                cv2.imshow("RGBD Depth", depth_vis)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            cv2.destroyWindow("RGBD Color")
            cv2.destroyWindow("RGBD Depth")

        cam.stop()
        results.append(ok("RGBD.stop()"))

    except Exception as exc:
        results.append(fail("RGBD API", repr(exc)))

    return results


def test_set_configs(yaml_path: str | None) -> list[bool]:
    print("\n=== Optional set_configs() ===")

    if yaml_path is None:
        return [ok("set_configs skipped", "no --yaml path provided")]

    results = []
    try:
        from camera.kinect import RGBD
        cam = RGBD()
        cam.set_configs(yaml_path)
        results.append(ok("RGBD.set_configs()", yaml_path))

        K, dist = cam.getCameraIntrinsics()
        results.append(check_intrinsics("intrinsics after set_configs()", K, dist))

        cam.stop()

    except Exception as exc:
        results.append(fail("RGBD.set_configs()", repr(exc)))

    return results


def main() -> int:
    parser = argparse.ArgumentParser(description="Concise Kinect v1 API test")
    parser.add_argument("--display", action="store_true", help="show OpenCV preview windows")
    parser.add_argument("--seconds", type=float, default=3.0, help="display/FPS duration")
    parser.add_argument("--skip-direct", action="store_true", help="skip direct freenect test")
    parser.add_argument("--yaml", type=str, default=None, help="optional kinect.yaml path for set_configs()")
    args = parser.parse_args()

    results = []

    if not args.skip_direct:
        results += test_direct_freenect()

    results += test_color_api(args.display, args.seconds)
    results += test_depth_api(args.display, args.seconds)
    results += test_rgbd_api(args.display, args.seconds)
    results += test_set_configs(args.yaml)

    passed = sum(results)
    total = len(results)

    print("\n=== Summary ===")
    print(f"Passed {passed}/{total} tests.")

    if passed == total:
        print("All tests passed.")
        return 0

    print("Some tests failed.")
    return 1


if __name__ == "__main__":
    sys.exit(main())