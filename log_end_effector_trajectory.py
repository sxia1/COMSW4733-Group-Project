import csv
import omni.kit.app
import omni.usd
from pxr import UsdGeom

# ==== ここを書き換える ===============================

# 例: "/World/TreeScene/UR5e_cutter/Tool0/cutter/cut_point"
CUT_POINT_PATH = "/World/ur5e_cutter/Tool0/cutter/cut_point"  # TODO: 実際のパスに合わせる

# 例: Git リポジトリを /root にマウントしている場合
OUTPUT_CSV_PATH = "/root/COMSW4733-Group-Project/data/trajectory_tree_1_V_0001.csv"

# ====================================================


# グローバル状態
_log_file = None
_csv_writer = None
_subscription = None
_sim_time = 0.0


def _init_logger():
    """CSV ログの初期化 & cut_point Prim の取得"""
    global _log_file, _csv_writer, _cutpoint_xform

    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(CUT_POINT_PATH)
    if not prim.IsValid():
        raise RuntimeError(f"[log_end_effector_trajectory] Prim not found at path: {CUT_POINT_PATH}")

    _cutpoint_xform = UsdGeom.Xformable(prim)

    # CSV を開く（上書き）
    _log_file = open(OUTPUT_CSV_PATH, "w", newline="")
    _csv_writer = csv.writer(_log_file)
    _csv_writer.writerow(["sim_time", "world_x", "world_y", "world_z"])


def _get_cutpoint_world_position():
    """cut_point の world 座標 (x, y, z) を取得"""
    # Isaac Sim では、物理更新ごとに Prim の変換行列が更新されるので、
    # Usd.TimeCode.Default() で現在の transform を取得できる。
    m = _cutpoint_xform.ComputeLocalToWorldTransform(0.0)
    t = m.ExtractTranslation()
    return float(t[0]), float(t[1]), float(t[2])


def _on_update(update_event):
    """毎フレーム呼ばれるコールバック"""
    global _sim_time

    # update_event.payload に dt が入っている
    dt = update_event.payload["dt"]
    _sim_time += float(dt)

    # 座標取得
    x, y, z = _get_cutpoint_world_position()

    # CSV に書き出し
    _csv_writer.writerow([_sim_time, x, y, z])
    _log_file.flush()


def start_logging():
    """ロギング開始（Script Editor から呼ぶエントリーポイント）"""
    global _subscription

    print("[log_end_effector_trajectory] Initializing logger...")
    _init_logger()

    print(f"[log_end_effector_trajectory] Logging to: {OUTPUT_CSV_PATH}")
    print(f"[log_end_effector_trajectory] Tracking cut point at: {CUT_POINT_PATH}")

    # update イベントストリームを購読して毎フレーム _on_update を呼ぶ
    update_stream = omni.kit.app.get_app().get_update_event_stream()
    _subscription = update_stream.create_subscription_to_pop(_on_update)
    print("[log_end_effector_trajectory] Subscribed to update events.")


def stop_logging():
    """ロギング停止 & リソース解放（必要なら Script Editor から呼ぶ）"""
    global _subscription, _log_file

    if _subscription is not None:
        _subscription = None
        print("[log_end_effector_trajectory] Unsubscribed from update events.")

    if _log_file is not None:
        _log_file.close()
        _log_file = None
        print("[log_end_effector_trajectory] Log file closed.")


# Script Editor でこのファイルを「Import」して使う場合は、
# その後 Python タブで以下のように呼び出せばログが始まります：
#
#   import log_end_effector_trajectory as leet
#   leet.start_logging()
#
# 止めたいときは:
#
#   leet.stop_logging()
#
# としてください。