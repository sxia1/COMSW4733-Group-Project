import omni
import omni.kit.app
from pxr import Usd
import omni.physx as physx
import csv
import time

# ====== 設定 ======
CSV_PATH = "/root/collision_log.csv"      # ログの保存先
ROBOT_ROOT = "/World/ur5e_cutter"         # ロボットのルート Prim
TREE_ROOT = "/World/tree"                 # 木のルート Prim

_stage = omni.usd.get_context().get_stage()
_physx = physx.get_physx_interface()

_update_sub = None
_started = False
_robot_prims = set()
_tree_prims = set()


def _get_descendants(path):
    """指定した Prim 以下のすべての Prim パスを列挙"""
    prim = _stage.GetPrimAtPath(path)
    if not prim.IsValid():
        print(f"[collision_logger] WARNING: Prim {path} not found.")
        return []
    return [str(p.GetPath()) for p in Usd.PrimRange(prim)]


def start_collision_logging():
    """毎フレームの衝突情報を CSV に記録し始める"""
    global _update_sub, _started, _robot_prims, _tree_prims

    if _started:
        print("[collision_logger] Already running.")
        return

    _robot_prims = set(_get_descendants(ROBOT_ROOT))
    _tree_prims = set(_get_descendants(TREE_ROOT))

    print("[collision_logger] Starting collision logging...")
    print(f"[collision_logger] Robot prim count = {len(_robot_prims)}")
    print(f"[collision_logger] Tree  prim count = {len(_tree_prims)}")
    print(f"[collision_logger] Logging CSV -> {CSV_PATH}")

    # まずヘッダだけ書いておく
    with open(CSV_PATH, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            ["time", "body0", "body1",
             "normal_x", "normal_y", "normal_z",
             "impulse"]
        )

    def on_update(dt: float):
        """毎フレーム呼ばれて衝突をチェック"""
        report = _physx.get_contact_report()
        if report is None:
            return

        now = time.time()
        rows = []

        for c in report.contacts:
            a = c.rigid_body0
            b = c.rigid_body1

            # 片方がロボット群、もう片方が木群ならログ対象
            if (a in _robot_prims and b in _tree_prims) or (a in _tree_prims and b in _robot_prims):
                n = c.contact_normal
                rows.append([
                    now,
                    a,
                    b,
                    n[0], n[1], n[2],
                    c.impulse
                ])
                print(f"[collision_logger] Collision: {a} <-> {b}")

        if rows:
            # ここで追記モードで一気に書く
            with open(CSV_PATH, "a", newline="") as f:
                writer = csv.writer(f)
                writer.writerows(rows)

    # ★ ここが修正ポイント：timeline ではなく kit.app の update イベントを使う
    update_stream = omni.kit.app.get_app().get_update_event_stream()
    _update_sub = update_stream.create_subscription_to_pop(
        on_update, name="collision_logger_update"
    )

    _started = True
    print("[collision_logger] Subscribed to update events.")


def stop_collision_logging():
    """購読を止める（必要なら）"""
    global _update_sub, _started
    if _update_sub is not None:
        _update_sub = None
    _started = False
    print("[collision_logger] Stopped collision logging.")