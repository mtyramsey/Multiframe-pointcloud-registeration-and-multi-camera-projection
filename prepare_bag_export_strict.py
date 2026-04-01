from pathlib import Path
import shutil
import re

SRC = Path(r"C:\Users\Michael\Desktop\bag_export")
DST = Path(r"C:\Users\Michael\Desktop\bag_export_strict")

LIDAR_DIR = SRC / "lidar"
PRIMARY_DIR = SRC / "primary_camera"
SECONDARY_DIR = SRC / "secondary_camera"

DST_LIDAR = DST / "PointCloudsIntensity"
DST_PRIMARY = DST / "primaryImages"
DST_SECONDARY = DST / "secondaryImages"

MAX_DT = 0.05  # seconds; try 0.02 if you want stricter matching

for d in [DST_LIDAR, DST_PRIMARY, DST_SECONDARY]:
    d.mkdir(parents=True, exist_ok=True)

def read_ts_file(ts_path: Path):
    try:
        return float(ts_path.read_text().strip())
    except Exception:
        return None

def stem_index(name: str):
    m = re.search(r"frame_(\d+)", name)
    return int(m.group(1)) if m else None

def build_items(folder: Path, allow_pcd=False):
    items = []
    for p in folder.iterdir():
        if not p.is_file():
            continue
        if "_timestamp" in p.name:
            continue

        if allow_pcd:
            if p.suffix.lower() != ".pcd":
                continue

        idx = stem_index(p.name)
        if idx is None:
            continue

        ts_file = folder / f"frame_{idx:06d}_timestamp.txt"
        if not ts_file.exists():
            continue

        ts = read_ts_file(ts_file)
        if ts is None:
            continue

        items.append((ts, idx, p))

    items.sort(key=lambda x: x[0])
    return items

def nearest_by_timestamp(ts, items):
    if not items:
        return None
    return min(items, key=lambda x: abs(x[0] - ts))

lidar_items = build_items(LIDAR_DIR, allow_pcd=True)
primary_items = build_items(PRIMARY_DIR)
secondary_items = build_items(SECONDARY_DIR)

used_primary = set()
used_secondary = set()

count = 0
skipped = 0

for lidar_ts, lidar_idx, lidar_file in lidar_items:
    p_match = nearest_by_timestamp(lidar_ts, primary_items)
    s_match = nearest_by_timestamp(lidar_ts, secondary_items)

    if p_match is None or s_match is None:
        skipped += 1
        continue

    p_ts, p_idx, p_file = p_match
    s_ts, s_idx, s_file = s_match

    dt_p = abs(lidar_ts - p_ts)
    dt_s = abs(lidar_ts - s_ts)

    # Keep only close timestamp matches
    if dt_p > MAX_DT or dt_s > MAX_DT:
        skipped += 1
        continue

    # Optional: prevent reusing the same camera frames multiple times
    if p_idx in used_primary or s_idx in used_secondary:
        skipped += 1
        continue

    used_primary.add(p_idx)
    used_secondary.add(s_idx)

    out_name = f"{count:06d}"

    shutil.copy2(lidar_file, DST_LIDAR / f"{out_name}.pcd")
    shutil.copy2(p_file, DST_PRIMARY / f"{out_name}{p_file.suffix}")
    shutil.copy2(s_file, DST_SECONDARY / f"{out_name}{s_file.suffix}")

    print(
        f"KEPT  lidar frame_{lidar_idx:06d} "
        f"| primary frame_{p_idx:06d} dt={dt_p:.6f}s "
        f"| secondary frame_{s_idx:06d} dt={dt_s:.6f}s"
    )

    count += 1

print(f"\nDone. Kept {count} synchronized triplets.")
print(f"Skipped {skipped} lidar frames.")
print(f"Saved to: {DST}")