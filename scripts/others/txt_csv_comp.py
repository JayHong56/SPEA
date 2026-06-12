import re
import csv
import argparse
from collections import defaultdict


def parse_txt_voxels(txt_path: str, coord_order: str = "mmdet"):
    """
    解析类似：
      --- Voxel Index: 0 | Coordinates: [0, 0, 302, 34] | Valid Points: 20 ---

    coord_order:
      mmdet : Coordinates = [batch, z, y, x]
      xy    : Coordinates = [*, *, x, y]
    """
    pattern = re.compile(
        r"Voxel Index:\s*(\d+)\s*\|\s*Coordinates:\s*\[([^\]]+)\]\s*\|\s*Valid Points:\s*(\d+)"
    )

    voxels = {}
    duplicates = defaultdict(list)

    with open(txt_path, "r", encoding="utf-8", errors="ignore") as f:
        for line_no, line in enumerate(f, start=1):
            m = pattern.search(line)
            if not m:
                continue

            voxel_index = int(m.group(1))
            coords = [int(x.strip()) for x in m.group(2).split(",")]
            valid_points = int(m.group(3))

            if len(coords) < 4:
                continue

            if coord_order == "mmdet":
                # [batch, z, y, x]
                y = coords[2]
                x = coords[3]
            elif coord_order == "xy":
                # [*, *, x, y]
                x = coords[2]
                y = coords[3]
            else:
                raise ValueError(f"Unsupported coord_order: {coord_order}")

            key = (x, y)

            if key in voxels:
                duplicates[key].append(voxels[key])

            voxels[key] = {
                "x": x,
                "y": y,
                "valid_points": valid_points,
                "voxel_index": voxel_index,
                "coords_raw": coords,
                "line_no": line_no,
            }

    return voxels, duplicates


def parse_kill_log(kill_path: str):
    """
    解析类似：
      KILL, global_timer:    202, addr: 100, ts_reg:   200, r_pn:     1, x:   42, y:  247
    """
    pattern = re.compile(
        r"KILL,\s*global_timer:\s*(\d+),\s*addr:\s*(\d+),\s*ts_reg:\s*(\d+),\s*r_pn:\s*(\d+),\s*x:\s*(-?\d+),\s*y:\s*(-?\d+)"
    )

    voxels = {}
    duplicates = defaultdict(list)

    with open(kill_path, "r", encoding="utf-8", errors="ignore") as f:
        for line_no, line in enumerate(f, start=1):
            m = pattern.search(line)
            if not m:
                continue

            global_timer = int(m.group(1))
            addr = int(m.group(2))
            ts_reg = int(m.group(3))
            r_pn = int(m.group(4))
            x = int(m.group(5))
            y = int(m.group(6))

            key = (x, y)

            item = {
                "x": x,
                "y": y,
                "r_pn": r_pn,
                "global_timer": global_timer,
                "addr": addr,
                "ts_reg": ts_reg,
                "line_no": line_no,
            }

            if key in voxels:
                duplicates[key].append(voxels[key])

            # 如果同一个 x,y 出现多次，默认保留最后一次
            voxels[key] = item

    return voxels, duplicates


def compare_voxel_point_numbers(txt_voxels, kill_voxels):
    all_keys = sorted(set(txt_voxels.keys()) | set(kill_voxels.keys()))

    rows = []
    for x, y in all_keys:
        txt_item = txt_voxels.get((x, y))
        kill_item = kill_voxels.get((x, y))

        valid_points = txt_item["valid_points"] if txt_item else None
        r_pn = kill_item["r_pn"] if kill_item else None

        if txt_item and kill_item:
            diff = valid_points - r_pn
            status = "MATCH" if diff == 0 else "DIFF"
        elif txt_item and not kill_item:
            diff = None
            status = "MISSING_IN_KILL"
        else:
            diff = None
            status = "MISSING_IN_TXT"

        rows.append({
            "status": status,
            "x": x,
            "y": y,
            "valid_points_txt": valid_points,
            "r_pn_kill": r_pn,
            "diff_txt_minus_kill": diff,
            "txt_voxel_index": txt_item["voxel_index"] if txt_item else "",
            "txt_line_no": txt_item["line_no"] if txt_item else "",
            "kill_global_timer": kill_item["global_timer"] if kill_item else "",
            "kill_addr": kill_item["addr"] if kill_item else "",
            "kill_ts_reg": kill_item["ts_reg"] if kill_item else "",
            "kill_line_no": kill_item["line_no"] if kill_item else "",
        })

    return rows


def write_report(rows, out_csv: str):
    fieldnames = [
        "status",
        "x",
        "y",
        "valid_points_txt",
        "r_pn_kill",
        "diff_txt_minus_kill",
        "txt_voxel_index",
        "txt_line_no",
        "kill_global_timer",
        "kill_addr",
        "kill_ts_reg",
        "kill_line_no",
    ]

    with open(out_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def print_summary(rows, txt_duplicates, kill_duplicates):
    total = len(rows)
    match = sum(1 for r in rows if r["status"] == "MATCH")
    diff = sum(1 for r in rows if r["status"] == "DIFF")
    missing_in_kill = sum(1 for r in rows if r["status"] == "MISSING_IN_KILL")
    missing_in_txt = sum(1 for r in rows if r["status"] == "MISSING_IN_TXT")

    print("=" * 80)
    print("Voxel point number compare summary")
    print("=" * 80)
    print(f"Total unique (x,y): {total}")
    print(f"MATCH          : {match}")
    print(f"DIFF           : {diff}")
    print(f"MISSING_IN_KILL: {missing_in_kill}")
    print(f"MISSING_IN_TXT : {missing_in_txt}")
    print(f"TXT duplicate coordinates : {len(txt_duplicates)}")
    print(f"KILL duplicate coordinates: {len(kill_duplicates)}")
    print("=" * 80)

    if diff > 0:
        print("\nFirst 1000 DIFF rows:")
        shown = 0
        for r in rows:
            if r["status"] == "DIFF":
                print(
                    f"x={r['x']}, y={r['y']}, "
                    f"Valid Points={r['valid_points_txt']}, "
                    f"r_pn={r['r_pn_kill']}, "
                    f"diff={r['diff_txt_minus_kill']}"
                )
                shown += 1
                if shown >= 1000:
                    break


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--txt", required=True, help="txt file with Voxel Index / Coordinates / Valid Points")
    parser.add_argument("--kill", required=True, help="kill log/csv file with r_pn, x, y")
    parser.add_argument("--out", default="voxel_point_compare_report.csv", help="output csv path")
    parser.add_argument(
        "--coord-order",
        default="mmdet",
        choices=["mmdet", "xy"],
        help="mmdet means Coordinates=[batch,z,y,x]; xy means Coordinates=[*,*,x,y]",
    )

    args = parser.parse_args()

    txt_voxels, txt_duplicates = parse_txt_voxels(args.txt, args.coord_order)
    kill_voxels, kill_duplicates = parse_kill_log(args.kill)

    rows = compare_voxel_point_numbers(txt_voxels, kill_voxels)
    write_report(rows, args.out)
    print_summary(rows, txt_duplicates, kill_duplicates)

    print(f"\nReport saved to: {args.out}")


if __name__ == "__main__":
    main()




# python txt_csv_comp.py --txt E:\mmdetection3d\sample_output.txt --kill E:\verilog\pillarnest\modelsim_pointpillars\ssimulation_result_killed.csv --out compare_report.csv
# python txt_csv_comp.py --txt E:\mmdetection3d\sample_output_nuscenes.txt --kill E:\verilog\pillarnest\temp\modelsim\ssimulation_result_killed.csv --out compare_report_nuscenes.csv