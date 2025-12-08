import csv
import sys
import ast
from pathlib import Path
from typing import List, Tuple, Optional

# フィールドサイズ上限を拡張（labels 列が非常に長いため）
csv.field_size_limit(sys.maxsize)

DATA_DIR = Path("data")
LABELS_CSV = DATA_DIR / "labels_teaser.csv"
OUT_CSV = DATA_DIR / "pruning_points_indices.csv"


def parse_labels(labels_str: str) -> List[int]:
    """
    labels_teaser.csv の 'labels' 列（例: "[0, 0, 1, ...]"）を
    Python の list[int] に変換する。
    """
    try:
        labels_list = ast.literal_eval(labels_str)
    except (SyntaxError, ValueError) as e:
        raise ValueError(f"Failed to parse labels string: {e}") from e

    # 念のため int にキャストしておく
    try:
        return [int(x) for x in labels_list]
    except Exception as e:
        raise ValueError(f"Labels list contains non-integer values: {e}") from e


def choose_representative_index(labels: List[int]) -> Optional[int]:
    """
    label == 1 の voxel インデックスの中から代表点を 1 つ選ぶ。
    ここでは一番単純に「中央のインデックス」を返す。

    1 が一つもない場合は None を返す。
    """
    pruned_indices = [i for i, v in enumerate(labels) if v == 1]
    if not pruned_indices:
        return None

    mid = len(pruned_indices) // 2
    return pruned_indices[mid]


def compute_all_pruning_indices() -> List[Tuple[str, float, int]]:
    """
    labels_teaser.csv の全行に対して pruning voxel index を計算する。

    Returns:
        List of (tree_id, voxel_size, voxel_index)
    """
    results: List[Tuple[str, float, int]] = []

    if not LABELS_CSV.exists():
        raise FileNotFoundError(f"{LABELS_CSV} not found. Make sure labels_teaser.csv is placed in data/")

    with LABELS_CSV.open() as f:
        reader = csv.DictReader(f)

        for row_idx, row in enumerate(reader):
            tree_id = row.get("tree")
            voxel_size_str = row.get("voxel_size")
            labels_str = row.get("labels")

            if tree_id is None or voxel_size_str is None or labels_str is None:
                print(f"[warning] Row {row_idx} is missing required fields; skipping.")
                continue

            try:
                voxel_size = float(voxel_size_str)
            except ValueError:
                print(f"[warning] Invalid voxel_size '{voxel_size_str}' at row {row_idx}; skipping.")
                continue

            try:
                labels = parse_labels(labels_str)
            except ValueError as e:
                print(f"[warning] Failed to parse labels for tree {tree_id} at row {row_idx}: {e}")
                continue

            idx = choose_representative_index(labels)
            if idx is None:
                print(f"[info] No pruned voxels (label==1) for tree {tree_id}; skipping.")
                continue

            results.append((tree_id, voxel_size, idx))

    return results


def save_all_pruning_indices(results: List[Tuple[str, float, int]]) -> None:
    """
    計算した pruning indices を CSV に保存する。
    """
    OUT_CSV.parent.mkdir(parents=True, exist_ok=True)

    with OUT_CSV.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["tree_id", "voxel_size", "voxel_index"])
        for tree_id, voxel_size, idx in results:
            writer.writerow([tree_id, voxel_size, idx])


def main() -> None:
    print(f"Reading labels from: {LABELS_CSV}")
    results = compute_all_pruning_indices()
    print(f"Computed pruning indices for {len(results)} trees.")

    save_all_pruning_indices(results)
    print(f"Saved pruning indices to: {OUT_CSV}")


if __name__ == "__main__":
    main()