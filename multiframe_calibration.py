import argparse
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation


def read_xyzi_text_file(filename):
    """
    Read plain-text XYZI data such as:

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


def intensity_to_color(intensities):
    """
    Map intensity values to RGB colors.
    """
    intensities = np.array(intensities, dtype=np.float32)

    if len(intensities) == 0:
        return np.zeros((0, 3), dtype=np.float32)

    if intensities.max() > intensities.min():
        norm = (intensities - intensities.min()) / (intensities.max() - intensities.min())
    else:
        norm = np.ones_like(intensities) * 0.5

    hue = (1.0 - norm) * 0.75
    sat = np.ones_like(norm)
    val = np.ones_like(norm)

    colors = np.zeros((len(intensities), 3), dtype=np.float32)

    for i in range(len(intensities)):
        h = hue[i]
        s = sat[i]
        v = val[i]

        c = v * s
        x = c * (1 - abs((h * 6) % 2 - 1))
        m = v - c

        if h < 1 / 6:
            r, g, b = c, x, 0
        elif h < 2 / 6:
            r, g, b = x, c, 0
        elif h < 3 / 6:
            r, g, b = 0, c, x
        elif h < 4 / 6:
            r, g, b = 0, x, c
        elif h < 5 / 6:
            r, g, b = x, 0, c
        else:
            r, g, b = c, 0, x

        colors[i] = [r + m, g + m, b + m]

    return colors


def save_calibration(K, R, t, filename, camera, used_frames, reproj_mean, reproj_median):
    """
    Save final calibration to a text file.
    """
    rot = Rotation.from_matrix(R)
    euler = rot.as_euler("xyz", degrees=True)

    with open(filename, "w", encoding="utf-8") as f:
        f.write(f"# Multi-frame Camera-LiDAR Calibration - {camera} camera\n")
        f.write(f"# Frames used: {used_frames}\n")
        f.write(f"# Mean reprojection error: {reproj_mean:.6f} px\n")
        f.write(f"# Median reprojection error: {reproj_median:.6f} px\n\n")

        f.write("# Camera Intrinsics (K)\n")
        for row in K:
            f.write(f"{row[0]} {row[1]} {row[2]}\n")
        f.write("\n")

        f.write("# Rotation Matrix (R)\n")
        for row in R:
            f.write(f"{row[0]} {row[1]} {row[2]}\n")
        f.write("\n")

        f.write("# Translation Vector (t)\n")
        f.write(f"{t[0]} {t[1]} {t[2]}\n\n")

        f.write("# Euler Angles (degrees)\n")
        f.write(f"Roll: {euler[0]}\n")
        f.write(f"Pitch: {euler[1]}\n")
        f.write(f"Yaw: {euler[2]}\n")

    print(f"\nSaved calibration to: {filename}")


class MultiFrameCalibrator:
    def __init__(
        self,
        data_dir,
        camera,
        n_frames=10,
        points_per_frame=4,
        fx=2018.1,
        fy=2018.1,
        cx=960.0,
        cy=600.0,
        display_scale=0.5,
    ):
        self.data_path = Path(data_dir)
        self.camera = camera
        self.n_frames = n_frames
        self.points_per_frame = points_per_frame
        self.display_scale = display_scale

        self.K = np.array(
            [
                [fx, 0.0, cx],
                [0.0, fy, cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

        self.pc_dir = self.data_path / "PointCloudsIntensity"
        self.img_dir = self.data_path / f"{camera}Images"

        if not self.pc_dir.exists():
            raise FileNotFoundError(f"Missing folder: {self.pc_dir}")
        if not self.img_dir.exists():
            raise FileNotFoundError(f"Missing folder: {self.img_dir}")

        self.pc_files = sorted(self.pc_dir.glob("*.pcd"))
        self.img_files = sorted(
            list(self.img_dir.glob("*.png"))
            + list(self.img_dir.glob("*.jpg"))
            + list(self.img_dir.glob("*.jpeg"))
        )

        if len(self.pc_files) == 0:
            raise ValueError(f"No point cloud files found in {self.pc_dir}")
        if len(self.img_files) == 0:
            raise ValueError(f"No image files found in {self.img_dir}")

        pc_stems = {p.stem for p in self.pc_files}
        img_stems = {p.stem for p in self.img_files}
        common_stems = sorted(pc_stems & img_stems)

        if len(common_stems) == 0:
            raise ValueError("No matching frame stems between point clouds and images.")

        if len(common_stems) <= self.n_frames:
            self.selected_stems = common_stems
        else:
            idxs = np.linspace(0, len(common_stems) - 1, self.n_frames, dtype=int)
            self.selected_stems = [common_stems[i] for i in idxs]

        self.all_pc_points = []
        self.all_img_points = []
        self.used_frames = []

        self.frame_pc_points = []
        self.frame_img_points = []
        self.waiting_for = "pointcloud"
        self.current_image = None

        self.colors = [
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1],
            [1, 1, 0],
            [1, 0, 1],
            [0, 1, 1],
            [1, 0.5, 0],
            [0.5, 0, 1],
            [0, 1, 0.5],
            [1, 0, 0.5],
        ]

    def get_color_for_pair(self, idx):
        return self.colors[idx % len(self.colors)]

    def find_image_for_stem(self, stem):
        for ext in [".png", ".jpg", ".jpeg"]:
            candidate = self.img_dir / f"{stem}{ext}"
            if candidate.exists():
                return candidate
        return None

    def update_image_display(self, frame_stem):
        """
        Show a resized image for easier viewing, while keeping original coordinates
        for calibration.
        """
        display = cv2.resize(
            self.current_image,
            (0, 0),
            fx=self.display_scale,
            fy=self.display_scale,
        )

        for i, pt in enumerate(self.frame_img_points):
            color_bgr = (np.array(self.get_color_for_pair(i)[::-1]) * 255).astype(int).tolist()

            pt_scaled = (
                int(pt[0] * self.display_scale),
                int(pt[1] * self.display_scale),
            )

            cv2.circle(display, pt_scaled, 8, color_bgr, -1)
            cv2.circle(display, pt_scaled, 10, (255, 255, 255), 2)
            cv2.putText(
                display,
                str(i + 1),
                (pt_scaled[0] + 10, pt_scaled[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color_bgr,
                2,
            )

        if self.waiting_for == "pointcloud":
            status = f"Frame {frame_stem}: press P to pick LiDAR point"
            color = (0, 255, 255)
        else:
            status = f"Frame {frame_stem}: click matching IMAGE point"
            color = (0, 255, 0)

        cv2.putText(
            display,
            status,
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
        )

        cv2.putText(
            display,
            f"Pairs: {len(self.frame_img_points)}/{self.points_per_frame}",
            (20, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )

        cv2.putText(
            display,
            "P: pick LiDAR  U: undo  R: reset  N: next frame  Q: skip frame",
            (20, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
        )

        cv2.imshow("Image - Pick corresponding points", display)

    def image_click_callback(self, event, x, y, flags, frame_stem):
        """
        Convert click coordinates from resized display back to original image coords.
        """
        if event == cv2.EVENT_LBUTTONDOWN and self.waiting_for == "image":
            orig_x = x / self.display_scale
            orig_y = y / self.display_scale

            image_point = np.array([orig_x, orig_y], dtype=np.float32)
            self.frame_img_points.append(image_point)

            print(f"Picked image point {len(self.frame_img_points)}: ({orig_x:.1f}, {orig_y:.1f})")

            self.waiting_for = "pointcloud"
            self.update_image_display(frame_stem)

    def create_pointcloud_with_markers(self, points_3d, intensities):
        """
        Show previously selected points by recoloring them instead of adding giant spheres.
        """
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_3d)

        colors = intensity_to_color(intensities)

        for i, pt in enumerate(self.frame_pc_points):
            dists = np.linalg.norm(points_3d - pt, axis=1)
            idx = int(np.argmin(dists))
            colors[idx] = np.array(self.get_color_for_pair(i), dtype=np.float32)

        pcd.colors = o3d.utility.Vector3dVector(colors)
        return [pcd]

    def pick_pointcloud_point(self, points_3d, intensities, frame_stem):
        """
        Open point cloud picker for exactly one point.
        """
        print("\nPoint cloud window opened")
        print("SHIFT + left click ONE point, then press Q to close")

        geometries = self.create_pointcloud_with_markers(points_3d, intensities)

        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window(
            window_name=f"LiDAR frame {frame_stem} - pick one point",
            width=1280,
            height=720,
        )

        for geom in geometries:
            vis.add_geometry(geom)

        opt = vis.get_render_option()
        opt.point_size = 2.0
        opt.background_color = np.array([0.1, 0.1, 0.1])

        vis.run()
        picked_indices = vis.get_picked_points()
        vis.destroy_window()

        if len(picked_indices) == 0:
            print("No LiDAR point picked.")
            return False

        picked_idx = picked_indices[0]
        picked_point = points_3d[picked_idx]
        self.frame_pc_points.append(picked_point)

        print(f"Picked LiDAR point {len(self.frame_pc_points)}: {picked_point}")
        print("Now click the matching point in the image window.")

        self.waiting_for = "image"
        return True

    def collect_points_for_frame(self, stem):
        pc_file = self.pc_dir / f"{stem}.pcd"
        img_file = self.find_image_for_stem(stem)

        if not pc_file.exists():
            raise FileNotFoundError(pc_file)
        if img_file is None:
            raise FileNotFoundError(f"No image found for stem {stem}")

        points_3d, intensities = read_xyzi_text_file(pc_file)
        image = cv2.imread(str(img_file))
        if image is None:
            raise ValueError(f"Could not read image: {img_file}")

        self.current_image = image
        self.frame_pc_points = []
        self.frame_img_points = []
        self.waiting_for = "pointcloud"

        print("\n" + "=" * 70)
        print(f"FRAME {stem} | CAMERA={self.camera}")
        print("=" * 70)
        print("Instructions:")
        print("  P = pick one LiDAR point")
        print("  then click matching image point")
        print("  repeat until 4 pairs")
        print("  U = undo last pair")
        print("  R = reset this frame")
        print("  N = accept frame when 4 pairs are done")
        print("  Q = skip frame")

        cv2.namedWindow("Image - Pick corresponding points", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("Image - Pick corresponding points", self.image_click_callback, stem)
        self.update_image_display(stem)

        while True:
            key = cv2.waitKey(20) & 0xFF

            if key == ord("q"):
                cv2.destroyAllWindows()
                raise ValueError("Frame skipped by user")

            elif key == ord("p") and self.waiting_for == "pointcloud":
                self.pick_pointcloud_point(points_3d, intensities, stem)
                self.update_image_display(stem)

            elif key == ord("u"):
                if len(self.frame_pc_points) > 0 and len(self.frame_img_points) > 0:
                    self.frame_pc_points.pop()
                    self.frame_img_points.pop()
                elif len(self.frame_pc_points) > len(self.frame_img_points):
                    self.frame_pc_points.pop()

                self.waiting_for = "pointcloud"
                print(f"Undo. Remaining pairs: {len(self.frame_img_points)}")
                self.update_image_display(stem)

            elif key == ord("r"):
                self.frame_pc_points = []
                self.frame_img_points = []
                self.waiting_for = "pointcloud"
                print("Reset this frame.")
                self.update_image_display(stem)

            elif key == ord("n"):
                if (
                    len(self.frame_pc_points) == self.points_per_frame
                    and len(self.frame_img_points) == self.points_per_frame
                ):
                    break
                else:
                    print(f"Need exactly {self.points_per_frame} pairs before continuing.")

        cv2.destroyAllWindows()

        self.all_pc_points.append(np.array(self.frame_pc_points, dtype=np.float32))
        self.all_img_points.append(np.array(self.frame_img_points, dtype=np.float32))
        self.used_frames.append(stem)

        print(f"Collected {self.points_per_frame} pairs from frame {stem}")

    def collect_all_points(self):
        print(f"Selected frames: {self.selected_stems}")

        for stem in self.selected_stems:
            try:
                self.collect_points_for_frame(stem)
            except Exception as e:
                print(f"Skipping frame {stem}: {e}")

        if len(self.all_pc_points) == 0:
            raise ValueError("No valid frames were collected.")

        self.all_pc_points = np.vstack(self.all_pc_points).astype(np.float32)
        self.all_img_points = np.vstack(self.all_img_points).astype(np.float32)

        print("\n" + "=" * 70)
        print(f"Collected total correspondences: {len(self.all_pc_points)}")
        print(f"Frames used: {self.used_frames}")
        print("=" * 70)

    def solve(self):
        if len(self.all_pc_points) < 4:
            raise ValueError("Need at least 4 total correspondences.")

        success, rvec, tvec, inliers = cv2.solvePnPRansac(
            self.all_pc_points,
            self.all_img_points,
            self.K,
            None,
            flags=cv2.SOLVEPNP_ITERATIVE,
            reprojectionError=8.0,
            confidence=0.99,
            iterationsCount=1000,
        )

        if not success:
            raise ValueError("solvePnPRansac failed.")

        R, _ = cv2.Rodrigues(rvec)
        t = tvec.flatten()

        projected, _ = cv2.projectPoints(self.all_pc_points, rvec, tvec, self.K, None)
        projected = projected.reshape(-1, 2)
        errors = np.linalg.norm(projected - self.all_img_points, axis=1)

        print("\n" + "=" * 70)
        print("FINAL MULTI-FRAME CALIBRATION")
        print("=" * 70)
        print(f"Total point pairs: {len(self.all_pc_points)}")
        print(f"Inliers: {0 if inliers is None else len(inliers)}")
        print(f"Mean reprojection error: {errors.mean():.3f} px")
        print(f"Median reprojection error: {np.median(errors):.3f} px")
        print(f"Max reprojection error: {errors.max():.3f} px")

        rot = Rotation.from_matrix(R)
        euler = rot.as_euler("xyz", degrees=True)

        print("\nRotation Matrix R:")
        print(R)
        print("\nTranslation t:")
        print(t)
        print("\nEuler Angles (degrees):")
        print(f"Roll={euler[0]:.3f}, Pitch={euler[1]:.3f}, Yaw={euler[2]:.3f}")

        return R, t, errors


def main():
    parser = argparse.ArgumentParser(description="Multi-frame Camera-LiDAR Calibration")
    parser.add_argument("--data_dir", required=True, type=str)
    parser.add_argument("--camera", required=True, choices=["primary", "secondary"])
    parser.add_argument("--n_frames", type=int, default=10)
    parser.add_argument("--points_per_frame", type=int, default=4)
    parser.add_argument("--fx", type=float, default=2018.1)
    parser.add_argument("--fy", type=float, default=2018.1)
    parser.add_argument("--cx", type=float, default=960.0)
    parser.add_argument("--cy", type=float, default=600.0)
    parser.add_argument("--display_scale", type=float, default=0.5)
    parser.add_argument("--output", type=str, default=None)

    args = parser.parse_args()

    calib = MultiFrameCalibrator(
        data_dir=args.data_dir,
        camera=args.camera,
        n_frames=args.n_frames,
        points_per_frame=args.points_per_frame,
        fx=args.fx,
        fy=args.fy,
        cx=args.cx,
        cy=args.cy,
        display_scale=args.display_scale,
    )

    calib.collect_all_points()
    R, t, errors = calib.solve()

    output = args.output or f"multiframe_{args.camera}_calibration.txt"
    save_calibration(
        calib.K,
        R,
        t,
        output,
        args.camera,
        calib.used_frames,
        reproj_mean=float(errors.mean()),
        reproj_median=float(np.median(errors)),
    )


if __name__ == "__main__":
    main()