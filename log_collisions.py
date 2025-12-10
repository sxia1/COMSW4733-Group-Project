import csv
import os

import carb
import omni
import omni.physx as physx

# この run で使う tree ID（ファイル名に埋め込み）
TREE_ID = "tree_1_V_0001"

# いつものプロジェクトディレクトリに合わせる
OUTPUT_DIR = "/root/COMSW4733-Group-Project/data"
CSV_PATH = os.path.join(OUTPUT_DIR, f"collisions_{TREE_ID}.csv")

# Isaac Sim のインターフェースを取得
_timeline = omni.timeline.get_timeline_interface()
_physx = physx.acquire_physx_interface()

_subscription = None
_rows = []  # (sim_time, num_contacts) のリスト


def _on_contact(headers, data):
    """
    PhysX の contact report コールバック。

    headers: ContactEventHeaderVector
    data   : ContactDataVector
    """
    global _rows

    # このステップのシミュレーション時間
    t = _timeline.get_current_time()

    # このステップで発生した接触点の数
    num_contacts = len(data)

    # 衝突がなければ何も記録しない
    if num_contacts == 0:
        return

    _rows.append((t, num_contacts))


def start_collision_logging():
    """
    衝突ログの記録を開始。
    再生ボタンを押す「前」にこれを一度呼びます。
    """
    global _subscription, _rows

    if _subscription is not None:
        carb.log_warn("[collision] logging is already running")
        return

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    _rows = []

    _subscription = _physx.subscribe_contact_report_events(_on_contact)
    carb.log_info(f"[collision] started logging, will save to: {CSV_PATH}")


def stop_collision_logging():
    """
    衝突ログの記録を停止し、CSV に書き出し。
    シミュレーションが終わったら呼びます。
    """
    global _subscription

    if _subscription is None:
        carb.log_warn("[collision] logging is not running")
        return

    # サブスクリプション解除
    try:
        _subscription.unsubscribe()
    except Exception as e:
        carb.log_warn(f"[collision] failed to unsubscribe: {e}")
    _subscription = None

    # CSV に書き出し
    with open(CSV_PATH, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["sim_time", "collision_count"])
        writer.writerows(_rows)

    carb.log_info(f"[collision] saved {len(_rows)} rows to {CSV_PATH}")