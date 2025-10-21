from omni.isaac.core import World
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.franka import Franka
from omni.isaac.motion_generation import ArticulationKinematicsSolver
from pxr import UsdGeom, Gf
import omni.usd, numpy as np

# --- create or get World (robust) ---
world = None
try:
    world = World.instance()
except Exception:
    pass
if world is None:
    world = World(stage_units_in_meters=1.0)
else:
    world.scene.clear()   # only clears registered objects, not USD prims

# --- add or pick Franka at /World/franka ---
if is_prim_path_valid("/World/franka"):
    franka = Franka(prim_path="/World/franka", name="franka")
    world.scene.add(franka)
else:
    franka = world.scene.add(Franka(prim_path="/World/franka", name="franka"))

# --- read target (/World/branch) position safely ---
stage = omni.usd.get_context().get_stage()
branch_prim = stage.GetPrimAtPath("/World/branch")
target_pos = Gf.Vec3f(0.55, 0.0, 0.30)
if branch_prim and branch_prim.IsValid():
    xformable = UsdGeom.Xformable(branch_prim)
    result = xformable.GetLocalTransformation()       # 1要素 or (行列, 何か)
    M = result[0] if isinstance(result, tuple) else result
    t = M.ExtractTranslation()                         # Gf.Vec3d
    target_pos = Gf.Vec3f(float(t[0]), float(t[1]), float(t[2]))

# --- simulate & move EE to target via IK ---
world.reset()

ik = ArticulationKinematicsSolver(franka)
q0 = franka.get_joint_positions()
q_sol, success = ik.compute_inverse_kinematics(
    np.array([target_pos[0], target_pos[1], target_pos[2]]),
    np.array([0, 1, 0, 0])   # wxyz orientation (simple)
)
print("IK success:", success)

if success:
    steps = 600
    for i in range(steps):
        a = (i + 1) / steps
        q = q0 * (1 - a) + q_sol * a
        franka.apply_action(ArticulationAction(joint_positions=q))
        world.step(render=True)
else:
    for _ in range(120):
        world.step(render=True)

print("Done")
