import argparse
from pathlib import Path
import numpy as np
import cv2


def read_xyzi_text_file(filename):
    points = []

    with open(filename, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            upper = line.upper()
            if upper.startswith("FIELDS") or upper.startswith("SIZE") or upper.startswith("TYPE") \
               or upper.startswith("COUNT") or upper.startswith("WIDTH") or upper.startswith("HEIGHT") \
               or upper.startswith("VIEWPOINT") or upper.startswith("POINTS") or upper.startswith("DATA"):
                continue

            parts = line.split()
            if len(parts) < 3:
                continue

            try:
                x = float(parts[0])
                y = float(parts[1])
                z = float(parts[2])
                points.append([x, y, z])
            except Exception:
                continue

    points = np.array(points, dtype=np.float32)
    if len(points) == 0:
        raise ValueError(f"Could not read any points from {filename}")
    return points


def load_calibration(calib_file):
    lines = Path(calib_file).read_text().splitlines()

    def find_idx(prefix):
        for i, l in enumerate(lines):
            if l.strip().startswith(prefix):
                return i
        return -1

    k0 = find_idx("# Camera Intrinsics")
    r0 = find_idx("# Rotation Matrix")
    t0 = find_idx("# Translation Vector")

    if k0 == -1 or r0 == -1 or t0 == -1:
        raise ValueError(f"Could not parse calibration file: {calib_file}")

    K = np.array([
        [float(x) for x in lines[k0 + 1].split()],
        [float(x) for x in lines[k0 + 2].split()],
        [float(x) for x in lines[k0 + 3].split()],
    ], dtype=np.float64)

    R = np.array([
        [float(x) for x in lines[r0 + 1].split()],
        [float(x) for x in lines[r0 + 2].split()],
        [float(x) for x in lines[r0 + 3].split()],
    ], dtype=np.float64)

    t = np.array([float(x) for x in lines[t0 + 1].split()], dtype=np.float64)

    return K, R, t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data_dir", required=True)
    ap.add_argument("--camera", required=True, choices=["primary", "secondary"])
    ap.add_argument("--frame", type=int, default=0)
    ap.add_argument("--calib_file", required=True)
    ap.add_argument("--point_radius", type=int, default=2)
    ap.add_argument("--max_depth", type=float, default=50.0)
    ap.add_argument("--display_scale", type=float, default=0.6)
    args = ap.parse_args()

    data_dir = Path(args.data_dir)
    pc_file = data_dir / "PointCloudsIntensity" / f"{args.frame:06d}.pcd"

    img_dir = data_dir / f"{args.camera}Images"
    img_file = None
    for ext in [".png", ".jpg", ".jpeg"]:
        candidate = img_dir / f"{args.frame:06d}{ext}"
        if candidate.exists():
            img_file = candidate
            break

    if img_file is None:
        raise FileNotFoundError(f"No image found for frame {args.frame:06d} in {img_dir}")

    pts = read_xyzi_text_file(pc_file)
    img = cv2.imread(str(img_file))
    if img is None:
        raise ValueError(f"Could not read image: {img_file}")

    K, R, t = load_calibration(args.calib_file)

    # LiDAR -> camera
    pts_cam = (R @ pts.T).T + t.reshape(1, 3)

    # keep only points in front of camera
    mask = (pts_cam[:, 2] > 0.1) & (pts_cam[:, 2] < args.max_depth)
    pts_cam = pts_cam[mask]

    uvw = (K @ pts_cam.T).T
    uv = uvw[:, :2] / uvw[:, 2:3]

    h, w = img.shape[:2]

    # simple depth coloring: near=red, far=blue
    z = pts_cam[:, 2]
    z_norm = (z - z.min()) / (z.max() - z.min() + 1e-8)

    overlay = img.copy()

    for i in range(len(uv)):
        u = int(round(uv[i, 0]))
        v = int(round(uv[i, 1]))

        if 0 <= u < w and 0 <= v < h:
            r = int(255 * (1.0 - z_norm[i]))
            g = int(255 * (1.0 - abs(z_norm[i] - 0.5) * 2.0))
            b = int(255 * z_norm[i])
            cv2.circle(overlay, (u, v), args.point_radius, (b, g, r), -1)

    display = cv2.resize(
        overlay,
        (0, 0),
        fx=args.display_scale,
        fy=args.display_scale
    )

    cv2.imshow("Projected LiDAR on Image", display)
    print("Press any key in the image window to close.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()