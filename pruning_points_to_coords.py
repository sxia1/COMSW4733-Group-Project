# pruning_points_to_coords.py

import csv
import numpy as np
from pathlib import Path

# ==== ここを後で「本物のボクセルサイズ」に書き換える ====
# shape = (Z, Y, X) という前提で書いています
GRID_SHAPE = (200, 200, 100)  # TODO: Sophia / データセットから正しい値に差し替え

# ボクセル中心を world 座標に変換する時の原点
# とりあえず (0,0,0) を原点にしておき、必要なら後でオフセットを足す
VOXEL_ORIGIN = (0.0, 0.0, 0.0)


def index_to_zyx(idx: int):
    """flatten されたインデックス -> (z, y, x) の3Dインデックス"""
    z, y, x = np.unravel_index(idx, GRID_SHAPE)
    return int(z), int(y), int(x)


def voxel_to_world(z: int, y: int, x: int, voxel_size: float):
    """(z,y,x) とボクセルサイズから world 座標を計算（ボクセル中心）"""
    ox, oy, oz = VOXEL_ORIGIN
    wx = ox + (x + 0.5) * voxel_size
    wy = oy + (y + 0.5) * voxel_size
    wz = oz + (z + 0.5) * voxel_size
    return wx, wy, wz


def main():
    repo_root = Path(__file__).parent
    in_path = repo_root / "data" / "pruning_points_indices.csv"
    out_path = repo_root / "data" / "pruning_points_coords.csv"

    if not in_path.exists():
        raise FileNotFoundError(f"入力CSVが見つかりません: {in_path}")

    rows_out = []

    with in_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            tree_id = row["tree_id"]
            voxel_size = float(row["voxel_size"])
            voxel_index = int(row["voxel_index"])

            # 1) インデックス -> (z,y,x)
            z, y, x = index_to_zyx(voxel_index)

            # 2) (z,y,x) -> world座標
            wx, wy, wz = voxel_to_world(z, y, x, voxel_size)

            rows_out.append(
                {
                    "tree_id": tree_id,
                    "voxel_size": voxel_size,
                    "voxel_index": voxel_index,
                    "z": z,
                    "y": y,
                    "x": x,
                    "world_x": wx,
                    "world_y": wy,
                    "world_z": wz,
                }
            )

    # 出力
    with out_path.open("w", newline="") as f:
        fieldnames = [
            "tree_id",
            "voxel_size",
            "voxel_index",
            "z",
            "y",
            "x",
            "world_x",
            "world_y",
            "world_z",
        ]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows_out)

    print(f"Saved pruning-point coordinates to: {out_path}")


if __name__ == "__main__":
    main()