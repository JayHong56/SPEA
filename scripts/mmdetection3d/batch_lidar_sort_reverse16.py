import argparse
from pathlib import Path
import numpy as np


def load_kitti_bin(bin_path: Path) -> np.ndarray:
    """
    Load KITTI-style point cloud bin: float32 [N, 4] = x, y, z, intensity.
    """
    data = np.fromfile(str(bin_path), dtype=np.float32)
    if data.size % 4 != 0:
        raise ValueError(f"{bin_path}: float32 count {data.size} is not divisible by 4")
    return data.reshape(-1, 4)


def estimate_ring_id_by_elevation(points_xyz: np.ndarray, num_rings: int = 64, num_iter: int = 30):
    """
    Estimate approximate LiDAR ring id by clustering elevation angle.
    This is not the true Velodyne laser id unless calibration is available.
    """
    x = points_xyz[:, 0]
    y = points_xyz[:, 1]
    z = points_xyz[:, 2]

    horizontal_dist = np.sqrt(x * x + y * y)
    elevation = np.arctan2(z, horizontal_dist)

    valid = np.isfinite(elevation)
    elev_valid = elevation[valid]
    if elev_valid.size == 0:
        raise ValueError("no finite elevation angles")

    percentiles = np.linspace(0, 100, num_rings)
    centers = np.percentile(elev_valid, percentiles)

    for _ in range(num_iter):
        dist = np.abs(elev_valid[:, None] - centers[None, :])
        labels_valid = np.argmin(dist, axis=1)

        new_centers = centers.copy()
        for k in range(num_rings):
            mask = labels_valid == k
            if np.any(mask):
                new_centers[k] = np.mean(elev_valid[mask])

        if np.max(np.abs(new_centers - centers)) < 1e-8:
            break
        centers = new_centers

    sort_idx = np.argsort(centers)
    remap = np.zeros(num_rings, dtype=np.int64)
    for new_id, old_id in enumerate(sort_idx):
        remap[old_id] = new_id

    dist_all = np.abs(elevation[:, None] - centers[None, :])
    labels_all = np.argmin(dist_all, axis=1)
    ring_id = remap[labels_all]
    centers_sorted = centers[sort_idx]

    return ring_id, centers_sorted


def circular_azimuth_key(azimuth: np.ndarray, start_angle: float = 0.0, direction: str = "ccw") -> np.ndarray:
    two_pi = 2.0 * np.pi
    if direction == "ccw":
        return (azimuth - start_angle) % two_pi
    if direction == "cw":
        return (start_angle - azimuth) % two_pi
    raise ValueError("direction must be 'ccw' or 'cw'")


def reorder_kitti_to_rotating_scan(
    pts: np.ndarray,
    num_rings: int = 64,
    start_angle_deg: float = 0.0,
    direction: str = "ccw",
):
    """
    Reorder points approximately by rotating-scan order.
    Primary key: azimuth scanning order.
    Secondary key: estimated ring id.
    """
    xyz = pts[:, :3]
    ring_id, elev_centers = estimate_ring_id_by_elevation(xyz, num_rings=num_rings)

    azimuth = np.arctan2(xyz[:, 1], xyz[:, 0])
    azimuth = np.mod(azimuth, 2.0 * np.pi)
    start_angle = np.deg2rad(start_angle_deg)
    az_key = circular_azimuth_key(azimuth, start_angle=start_angle, direction=direction)

    order = np.lexsort((ring_id, az_key))
    return pts[order], order, ring_id, elev_centers


def reverse_16bytes_per_point(sorted_pts: np.ndarray) -> bytes:
    """
    Match the behavior of 2941bffb-...py:
    for every 16-byte point record, reverse the whole 16-byte order.

    Input memory order before reverse is little-endian KITTI float32:
        x_le(4), y_le(4), z_le(4), i_le(4)
    Output per point becomes:
        reverse_bytes(i_le), reverse_bytes(z_le), reverse_bytes(y_le), reverse_bytes(x_le)
    This is exactly whole-record byte reversal, not normal KITTI bin format.
    """
    pts32 = np.ascontiguousarray(sorted_pts.astype(np.float32, copy=False))
    raw = pts32.view(np.uint8).reshape(-1, 16)
    reversed_raw = raw[:, ::-1]
    return reversed_raw.tobytes()


def process_one_file(in_path: Path, out_path: Path, args) -> dict:
    pts = load_kitti_bin(in_path)
    sorted_pts, order, ring_id, elev_centers = reorder_kitti_to_rotating_scan(
        pts,
        num_rings=args.num_rings,
        start_angle_deg=args.start_angle_deg,
        direction=args.direction,
    )

    out_path.parent.mkdir(parents=True, exist_ok=True)
    # payload = reverse_16bytes_per_point(sorted_pts)
    out_path.write_bytes(sorted_pts)

    if args.save_sorted_bin:
        sorted_path = out_path.with_suffix(".sorted_float32.bin")
        sorted_pts.astype(np.float32).tofile(str(sorted_path))
    else:
        sorted_path = ""

    if args.save_order:
        order_path = out_path.with_suffix(".order.npy")
        np.save(str(order_path), order)
    else:
        order_path = ""

    return {
        "input": str(in_path),
        "output": str(out_path),
        "sorted_float32": str(sorted_path),
        "order": str(order_path),
        "points": int(pts.shape[0]),
        "bytes": int(len(sorted_pts)),
        "elev_min_deg": float(np.rad2deg(elev_centers[0])),
        "elev_max_deg": float(np.rad2deg(elev_centers[-1])),
    }


def main():
    parser = argparse.ArgumentParser(
        description="Batch apply lidar_sort-style rotating-scan sort, then reverse each 16-byte point record."
    )
    parser.add_argument("--input-dir", required=True, help="Folder containing source .bin files")
    parser.add_argument("--output-dir", required=True, help="Folder for processed .bin files")
    parser.add_argument("--pattern", default="*.bin", help="Input glob pattern, default: *.bin")
    parser.add_argument("--suffix", default="_sorted_rev16.bin", help="Output filename suffix")
    parser.add_argument("--num-rings", type=int, default=64)
    parser.add_argument("--start-angle-deg", type=float, default=0.0)
    parser.add_argument("--direction", choices=["ccw", "cw"], default="ccw")
    parser.add_argument("--save-sorted-bin", action="store_true", help="Also save sorted normal float32 KITTI-format bin")
    parser.add_argument("--save-order", action="store_true", help="Also save original-to-sorted index order as .npy")
    args = parser.parse_args()

    input_dir = Path(args.input_dir)
    output_dir = Path(args.output_dir)
    files = sorted(input_dir.glob(args.pattern))

    if not files:
        raise FileNotFoundError(f"no files matched {input_dir / args.pattern}")

    print(f"[INFO] found {len(files)} files")
    summary_rows = []

    for idx, in_path in enumerate(files):
        out_name = in_path.stem + args.suffix
        out_path = output_dir / out_name
        try:
            info = process_one_file(in_path, out_path, args)
            summary_rows.append(info)
            print(
                f"[{idx + 1}/{len(files)}] {in_path.name}: "
                f"points={info['points']}, bytes={info['bytes']}, out={out_path.name}"
            )
        except Exception as e:
            print(f"[ERROR] {in_path}: {e}")

    summary_path = output_dir / "summary.csv"
    output_dir.mkdir(parents=True, exist_ok=True)
    with summary_path.open("w", encoding="utf-8") as f:
        f.write("input,output,sorted_float32,order,points,bytes,elev_min_deg,elev_max_deg\n")
        for r in summary_rows:
            f.write(
                f"{r['input']},{r['output']},{r['sorted_float32']},{r['order']},"
                f"{r['points']},{r['bytes']},{r['elev_min_deg']:.8f},{r['elev_max_deg']:.8f}\n"
            )

    print(f"[INFO] summary saved to {summary_path}")


if __name__ == "__main__":
    main()


"""
python E:\mmdetection3d\my_script\batch_lidar_sort_reverse16.py ^
  --input-dir E:\mmdetection3d\data\kitti\training\velodyne_val ^
  --output-dir E:\mmdetection3d\data\kitti\training\velodyne_val_sorted


"""