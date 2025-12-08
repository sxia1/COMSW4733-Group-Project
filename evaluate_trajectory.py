import csv
import math
import os
import argparse


PRUNING_CSV = "data/pruning_points_coords.csv"


def load_pruning_targets(pruning_csv_path):
    """
    pruning_points_coords.csv を読んで
    tree_id -> (world_x, world_y, world_z) の dict を返す
    """
    targets = {}
    with open(pruning_csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            tree_id = row["tree_id"]
            wx = float(row["world_x"])
            wy = float(row["world_y"])
            wz = float(row["world_z"])
            targets[tree_id] = (wx, wy, wz)
    return targets


def load_trajectory(csv_path):
    """
    trajectory_xxx.csv を読んで
    times: [t0, t1, ...]
    positions: [(x,y,z), ...]
    を返す
    """
    times = []
    positions = []
    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            t = float(row["sim_time"])
            x = float(row["world_x"])
            y = float(row["world_y"])
            z = float(row["world_z"])
            times.append(t)
            positions.append((x, y, z))
    return times, positions


def euclidean(a, b):
    return math.sqrt(
        (a[0] - b[0]) ** 2 +
        (a[1] - b[1]) ** 2 +
        (a[2] - b[2]) ** 2
    )


def evaluate_trajectory(traj_path, target_pos):
    """
    1 つの軌道に対して評価指標を計算
    """
    times, poses = load_trajectory(traj_path)
    if len(times) < 2:
        raise ValueError("Trajectory has fewer than 2 samples.")

    # 実行時間
    duration = times[-1] - times[0]

    # 経路長（連続サンプル間の距離の合計）
    path_length = 0.0
    for p0, p1 in zip(poses[:-1], poses[1:]):
        path_length += euclidean(p0, p1)

    # 終点誤差
    final_pos = poses[-1]
    final_error = euclidean(final_pos, target_pos)

    # 最小距離とその時刻
    min_error = float("inf")
    time_at_min = None
    for t, p in zip(times, poses):
        d = euclidean(p, target_pos)
        if d < min_error:
            min_error = d
            time_at_min = t

    return {
        "duration": duration,
        "path_length": path_length,
        "final_error": final_error,
        "min_error": min_error,
        "time_at_min_error": time_at_min,
    }


def infer_tree_id_from_filename(traj_path):
    """
    ファイル名: trajectory_tree_1_V_0001.csv
    -> tree_id: tree_1_V_0001
    """
    base = os.path.basename(traj_path)
    if not base.startswith("trajectory_") or not base.endswith(".csv"):
        raise ValueError(f"Unexpected trajectory filename: {base}")
    return base[len("trajectory_"):-len(".csv")]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--trajectory",
        type=str,
        default="data/trajectory_tree_1_V_0001.csv",
        help="Path to trajectory CSV file",
    )
    parser.add_argument(
        "--pruning_csv",
        type=str,
        default=PRUNING_CSV,
        help="Path to pruning_points_coords.csv",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="data/eval_metrics.csv",
        help="Output CSV to append metrics to",
    )
    parser.add_argument(
        "--tree_id",
        type=str,
        default=None,
        help="Tree ID (if omitted, inferred from trajectory filename)",
    )
    args = parser.parse_args()

    traj_path = args.trajectory
    tree_id = args.tree_id or infer_tree_id_from_filename(traj_path)

    print(f"[eval] trajectory: {traj_path}")
    print(f"[eval] inferred tree_id: {tree_id}")

    # pruning point 読み込み
    targets = load_pruning_targets(args.pruning_csv)
    if tree_id not in targets:
        raise KeyError(
            f"Tree ID {tree_id} not found in {args.pruning_csv}"
        )
    target_pos = targets[tree_id]
    print(f"[eval] pruning target (world): {target_pos}")

    metrics = evaluate_trajectory(traj_path, target_pos)

    print("\n=== Metrics ===")
    for k, v in metrics.items():
        print(f"{k}: {v}")

    # 結果を CSV に追記（append）
    header = [
        "tree_id",
        "trajectory_csv",
        "target_x",
        "target_y",
        "target_z",
        "duration",
        "path_length",
        "final_error",
        "min_error",
        "time_at_min_error",
    ]
    file_exists = os.path.exists(args.output)

    with open(args.output, "a", newline="") as f:
        writer = csv.writer(f)
        if not file_exists:
            writer.writerow(header)
        writer.writerow([
            tree_id,
            traj_path,
            target_pos[0],
            target_pos[1],
            target_pos[2],
            metrics["duration"],
            metrics["path_length"],
            metrics["final_error"],
            metrics["min_error"],
            metrics["time_at_min_error"],
        ])

    print(f"\n[eval] Metrics appended to {args.output}")


if __name__ == "__main__":
    main()