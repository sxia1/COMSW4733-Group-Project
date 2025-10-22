import omni.usd
from pxr import Usd, UsdGeom, Sdf, Gf
import numpy as np
from omni.isaac.core import World
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.franka import Franka
from omni.isaac.motion_generation import ArticulationKinematicsSolver
from omni.isaac.core.utils.types import ArticulationAction

stage = omni.usd.get_context().get_stage()
if stage.GetDefaultPrim() is None:
    world_xf = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world_xf.GetPrim())

branch_path = Sdf.Path("/World/branch")
if not stage.GetPrimAtPath(branch_path):
    cube = UsdGeom.Cube.Define(stage, branch_path)
else:
    cube = UsdGeom.Cube.Get(stage, branch_path)

cube.CreateSizeAttr(0.08)
xf = UsdGeom.Xformable(cube.GetPrim())
xf.SetXformOpOrder([])
xf.AddTranslateOp().Set(Gf.Vec3f(0.55, 0.0, 0.30))
UsdGeom.Gprim(cube.GetPrim()).CreateDisplayColorAttr([(1.0, 0.1, 0.1)])

try:
    world = World.instance()
except Exception:
    world = None
if world is None:
    world = World(stage_units_in_meters=1.0)
else:
    world.scene.clear()

if is_prim_path_valid("/World/franka"):
    franka = Franka(prim_path="/World/franka", name="franka")
    world.scene.add(franka)
else:
    franka = world.scene.add(Franka(prim_path="/World/franka", name="franka"))

branch_prim = stage.GetPrimAtPath("/World/branch")
target_pos = Gf.Vec3f(0.55, 0.0, 0.30)
if branch_prim and branch_prim.IsValid():
    xfm = UsdGeom.Xformable(branch_prim)
    res = xfm.GetLocalTransformation()
    M = res[0] if isinstance(res, tuple) else res
    t = M.ExtractTranslation()
    target_pos = Gf.Vec3f(float(t[0]), float(t[1]), float(t[2]))

ik = ArticulationKinematicsSolver(franka, None, end_effector_frame_name="tool_center")
target_quat_wxyz = np.array([1.0, 0.0, 0.0, 0.0])
target_xyz = np.array([target_pos[0], target_pos[1], target_pos[2]])

world.reset()
q0 = franka.get_joint_positions()
q_sol, success = ik.compute_inverse_kinematics(target_xyz, target_quat_wxyz)

steps = 600 if success else 120
for i in range(steps):
    if success:
        a = (i + 1) / steps
        q = q0 * (1 - a) + q_sol * a
        franka.apply_action(ArticulationAction(joint_positions=q))
    world.step(render=True)