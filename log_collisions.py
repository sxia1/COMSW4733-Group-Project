import omni
from pxr import PhysxSchema, UsdPhysics, Usd, Gf
import csv
import time

# ログファイルの保存先
CSV_PATH = "/root/collision_log.csv"

# どの Prim 同士の衝突を見るか（必要なら変更）
ROBOT_PRIM = "/World/ur5e_cutter"
TREE_PRIM = "/World/tree"

stage = omni.usd.get_context().get_stage()


def get_all_collisions():
    """シーン内のすべての衝突イベントを返す"""
    physxIFace = omni.physx.acquire_physx_interface()
    contact_report = physxIFace.get_contact_report()
    return contact_report


def prim_involved(contact, prim_paths):
    """衝突の当事者に特定の Prim が含まれているか確認"""
    return (contact.rigid_body0 in prim_paths) or (contact.rigid_body1 in prim_paths)


def start_collision_logging():
    print("[collision_logger] Starting collision logging...")

    # 監視 Prim の全子孫 Prim を収集
    def get_descendants(path):
        prim = stage.GetPrimAtPath(path)
        return [str(p.GetPath()) for p in Usd.PrimRange(prim)]

    robot_prims = set(get_descendants(ROBOT_PRIM))
    tree_prims = set(get_descendants(TREE_PRIM))

    print(f"[collision_logger] Robot prim count = {len(robot_prims)}")
    print(f"[collision_logger] Tree prim count = {len(tree_prims)}")
    print(f"[collision_logger] Logging CSV → {CSV_PATH}")

    # CSV 書き込み開始
    with open(CSV_PATH, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time", "robot_prim", "tree_prim", "normal", "impulse"])

        # ハンドラ内で毎フレーム実行
        def on_update(dt):
            report = get_all_collisions()
            now = time.time()

            for contact in report.contacts:
                a = contact.rigid_body0
                b = contact.rigid_body1

                # 片方がロボット、もう片方が木の場合のみログ
                if (a in robot_prims and b in tree_prims) or (a in tree_prims and b in robot_prims):
                    writer.writerow([
                        now,
                        a,
                        b,
                        list(contact.contact_normal),
                        contact.impulse
                    ])
                    print(f"[collision_logger] Collision: {a} <-> {b}")

        # USD Update イベント購読
        update_sub = omni.timeline.get_timeline().get_update_event_stream().create_subscription_to_pop(on_update)

    print("[collision_logger] Logging active.")