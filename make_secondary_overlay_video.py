import argparse
from pathlib import Path
import cv2
import numpy as np


def read_xyzi_text_file(filename):
    """
    Reads plain-text XYZI data like:
        FIELDS x y z intensity
        x y z intensity
        ...
    Returns:
        points: Nx3 float32
        intensities: N float32
    """
    points = []
    intensities = []

    with open(filename, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            upper = line.upper()
            if (
                upper.startswith("FIELDS")
                or upper.startswith("SIZE")
                or upper.startswith("TYPE")
                or upper.startswith("COUNT")
                or upper.startswith("WIDTH")
                or upper.startswith("HEIGHT")
                or upper.startswith("VIEWPOINT")
                or upper.startswith("POINTS")
                or upper.startswith("DATA")
            ):
                continue

            parts = line.split()
            if len(parts) < 3:
                continue

            try:
                x = float(parts[0])
                y = float(parts[1])
                z = float(parts[2])
                points.append([x, y, z])

                if len(parts) >= 4:
                    intensities.append(float(parts[3]))
                else:
                    intensities.append(0.5)
            except Exception:
                continue

    points = np.array(points, dtype=np.float32)
    intensities = np.array(intensities, dtype=np.float32)

    if len(points) == 0:
        raise ValueError(f"Could not read any points from {filename}")

    return points, intensities


def load_calibration(calib_file):
    lines = Path(calib_file).read_text(encoding="utf-8").splitlines()

    def find_idx(prefix):
        for i, line in enumerate(lines):
            if line.strip().startswith(prefix):
                return i
        return -1

    k0 = find_idx("# Camera Intrinsics")
    r0 = find_idx("# Rotation Matrix")
    t0 = find_idx("# Translation Vector")

    if k0 == -1 or r0 == -1 or t0 == -1:
        raise ValueError(f"Could not parse calibration file: {calib_file}")

    K = np.array(
        [
            [float(x) for x in lines[k0 + 1].split()],
            [float(x) for x in lines[k0 + 2].split()],
            [float(x) for x in lines[k0 + 3].split()],
        ],
        dtype=np.float64,
    )

    R = np.array(
        [
            [float(x) for x in lines[r0 + 1].split()],
            [float(x) for x in lines[r0 + 2].split()],
            [float(x) for x in lines[r0 + 3].split()],
        ],
        dtype=np.float64,
    )

    t = np.array([float(x) for x in lines[t0 + 1].split()], dtype=np.float64)

    return K, R, t


def depth_to_bgr(depth_values):
    """
    Near -> red, mid -> green/yellow, far -> blue
    Returns Nx3 uint8 BGR colors.
    """
    z = np.asarray(depth_values, dtype=np.float32)
    if len(z) == 0:
        return np.zeros((0, 3), dtype=np.uint8)

    z_norm = (z - z.min()) / (z.max() - z.min() + 1e-8)

    b = (255 * z_norm).astype(np.uint8)
    r = (255 * (1.0 - z_norm)).astype(np.uint8)
    g = (255 * (1.0 - np.abs(z_norm - 0.5) * 2.0)).clip(0, 255).astype(np.uint8)

    return np.stack([b, g, r], axis=1)


def find_image_for_stem(img_dir, stem):
    for ext in [".png", ".jpg", ".jpeg"]:
        p = img_dir / f"{stem}{ext}"
        if p.exists():
            return p
    return None


def overlay_points_on_image(
    img,
    pts_lidar,
    K,
    R,
    t,
    point_radius=2,
    max_depth=50.0,
):
    """
    Project LiDAR points into the image and draw them.
    """
    overlay = img.copy()

    # LiDAR -> camera
    pts_cam = (R @ pts_lidar.T).T + t.reshape(1, 3)

    # Only keep points in front of camera and within depth range
    valid = (pts_cam[:, 2] > 0.1) & (pts_cam[:, 2] < max_depth)
    pts_cam = pts_cam[valid]

    if len(pts_cam) == 0:
        return overlay, 0

    uvw = (K @ pts_cam.T).T
    uv = uvw[:, :2] / uvw[:, 2:3]

    h, w = img.shape[:2]
    colors = depth_to_bgr(pts_cam[:, 2])

    count_drawn = 0
    for i in range(len(uv)):
        u = int(round(uv[i, 0]))
        v = int(round(uv[i, 1]))

        if 0 <= u < w and 0 <= v < h:
            cv2.circle(
                overlay,
                (u, v),
                point_radius,
                tuple(int(c) for c in colors[i]),
                -1,
            )
            count_drawn += 1

    return overlay, count_drawn


def main():
    parser = argparse.ArgumentParser(
        description="Overlay LiDAR on secondary camera images and export a video."
    )
    parser.add_argument("--data_dir", required=True, help="Path to prepared dataset folder")
    parser.add_argument(
        "--calib_file",
        default="multiframe_secondary_calibration.txt",
        help="Calibration txt file for secondary camera",
    )
    parser.add_argument(
        "--output_video",
        default="secondary_overlay_video.mp4",
        help="Output video filename",
    )
    parser.add_argument(
        "--frames_dir",
        default="secondary_overlay_frames",
        help="Folder to save individual overlay frames",
    )
    parser.add_argument("--fps", type=float, default=10.0, help="Video FPS")
    parser.add_argument("--point_radius", type=int, default=2, help="Projected point radius")
    parser.add_argument("--max_depth", type=float, default=50.0, help="Max depth to draw")
    parser.add_argument(
        "--display_scale",
        type=float,
        default=1.0,
        help="Resize output frames, e.g. 0.5 for half-size video",
    )
    args = parser.parse_args()

    data_dir = Path(args.data_dir)
    pc_dir = data_dir / "PointCloudsIntensity"
    img_dir = data_dir / "secondaryImages"

    if not pc_dir.exists():
        raise FileNotFoundError(f"Missing folder: {pc_dir}")
    if not img_dir.exists():
        raise FileNotFoundError(f"Missing folder: {img_dir}")

    K, R, t = load_calibration(args.calib_file)

    pc_files = sorted(pc_dir.glob("*.pcd"))
    if len(pc_files) == 0:
        raise ValueError(f"No point cloud files found in {pc_dir}")

    frames_dir = Path(args.frames_dir)
    frames_dir.mkdir(parents=True, exist_ok=True)

    writer = None
    written_frames = 0

    for pc_file in pc_files:
        stem = pc_file.stem
        img_file = find_image_for_stem(img_dir, stem)

        if img_file is None:
            print(f"Skipping {stem}: no matching image")
            continue

        try:
            pts_lidar, _ = read_xyzi_text_file(pc_file)
        except Exception as e:
            print(f"Skipping {stem}: failed to read point cloud ({e})")
            continue

        img = cv2.imread(str(img_file))
        if img is None:
            print(f"Skipping {stem}: failed to read image")
            continue

        overlay, count_drawn = overlay_points_on_image(
            img,
            pts_lidar,
            K,
            R,
            t,
            point_radius=args.point_radius,
            max_depth=args.max_depth,
        )

        cv2.putText(
            overlay,
            f"Frame {stem} | projected points: {count_drawn}",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (255, 255, 255),
            2,
        )

        if args.display_scale != 1.0:
            overlay = cv2.resize(
                overlay,
                (0, 0),
                fx=args.display_scale,
                fy=args.display_scale,
            )

        frame_out = frames_dir / f"{stem}.png"
        cv2.imwrite(str(frame_out), overlay)

        if writer is None:
            h, w = overlay.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            writer = cv2.VideoWriter(str(args.output_video), fourcc, args.fps, (w, h))

        writer.write(overlay)
        written_frames += 1

        if written_frames % 25 == 0:
            print(f"Wrote {written_frames} frames...")

    if writer is not None:
        writer.release()

    print(f"\nDone.")
    print(f"Saved frames to: {frames_dir}")
    print(f"Saved video to: {args.output_video}")
    print(f"Frames written: {written_frames}")


if __name__ == "__main__":
    main()