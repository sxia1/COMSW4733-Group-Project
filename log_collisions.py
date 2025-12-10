import csv

import omni.timeline
from omni.physx import get_physx_simulation_interface
from omni.physx.scripts.physicsUtils import PhysicsSchemaTools

# ---- 設定（必要ならここだけ書き換える） ----
ROBOT_PREFIX = "/World/ur5e_cutter"
TREE_PREFIX = "/World/tree"
CSV_PATH = "/root/collision_log.csv"
# -------------------------------------------

_timeline = omni.timeline.get_timeline_interface()
_physx_sim = get_physx_simulation_interface()

_log_file = None
_csv_writer = None
_sub_handle = None
_started = False


def _on_contact_report(contact_headers, contact_data):
    """PhysX から衝突イベントが飛んでくるコールバック"""
    global _csv_writer, _started
    if not _started or _csv_writer is None:
        return

    for header in contact_headers:
        # actor0/1 は int なので SdfPath に直す
        prim0 = str(PhysicsSchemaTools.intToSdfPath(header.actor0))
        prim1 = str(PhysicsSchemaTools.intToSdfPath(header.actor1))

        # ロボット vs 木 の衝突だけを見る
        pair = {prim0, prim1}
        has_robot = any(p.startswith(ROBOT_PREFIX) for p in pair)
        has_tree = any(p.startswith(TREE_PREFIX) for p in pair)

        if has_robot and has_tree:
            t = _timeline.get_current_time()
            _csv_writer.writerow([t, prim0, prim1])


def start_collision_logging():
    """Script Editor から呼ぶ入口"""
    global _log_file, _csv_writer, _sub_handle, _started

    if _started:
        print("[collision_logger] Already running")
        return

    # CSV を開く
    _log_file = open(CSV_PATH, "w", newline="")
    _csv_writer = csv.writer(_log_file)
    _csv_writer.writerow(["time", "actor0", "actor1"])

    # PhysX 衝突イベントを購読
    _sub_handle = _physx_sim.subscribe_contact_report_events(
        _on_contact_report
    )

    _started = True
    print(f"[collision_logger] Logging collisions to {CSV_PATH}")


def stop_collision_logging():
    """必要なら手動で停止する用"""
    global _log_file, _csv_writer, _sub_handle, _started

    if not _started:
        return

    if _sub_handle is not None:
        _sub_handle.unsubscribe()
        _sub_handle = None

    if _log_file is not None:
        _log_file.close()
        _log_file = None

    _csv_writer = None
    _started = False
    print("[collision_logger] Stopped collision logging")