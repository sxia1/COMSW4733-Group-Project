import os
import omni.usd
from pxr import Usd, UsdGeom, Sdf, Gf
import numpy as np
from omni.isaac.core import World
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.franka import Franka
from omni.isaac.motion_generation import ArticulationKinematicsSolver
from omni.isaac.core.utils.types import ArticulationAction
from isaacsim.core.utils.extensions import get_extension_path_from_name
from isaacsim.core.prims import Articulation
from isaacsim.robot_motion.motion_generation.lula import RRT

def create_world(stage):
    if stage.GetDefaultPrim() is None:
        world_xf = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
        stage.SetDefaultPrim(world_xf.GetPrim())
    try:
        world = World.instance()
    except Exception:
        world = None
    if world is None:
        world = World(stage_units_in_meters=1.0)
    else:
        world.scene.clear()
    return world

def create_branch(stage):
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
    return stage.GetPrimAtPath("/World/branch")

def create_robot(stage, world):
    if is_prim_path_valid("/World/franka"):
        return stage.GetPrimAtPath("/World/franka")
    return world.scene.add(Franka(prim_path="/World/franka", name="franka"))

def create_rrt():
    mg_extension_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
    rmp_config_dir = os.path.join(mg_extension_path, "motion_policy_configs")
    rrt_config_dir = os.path.join(mg_extension_path, "path_planner_configs")
    rrt = RRT(
        robot_description_path = rmp_config_dir + "/franka/rmpflow/robot_descriptor.yaml",
        urdf_path = rmp_config_dir + "/franka/lula_franka_gen.urdf",
        rrt_config_path = rrt_config_dir + "/franka/rrt/franka_planner_config.yaml",
        end_effector_frame_name = "tool_center"
    )
    rrt.set_max_iterations(5000)
    return rrt

stage = omni.usd.get_context().get_stage()
world = create_world(stage)
branch_prim = create_branch(stage)
franka = create_robot(stage, world)

target_pos = Gf.Vec3f(0.55, 0.0, 0.30)
if branch_prim and branch_prim.IsValid():
    xfm = UsdGeom.Xformable(branch_prim)
    res = xfm.GetLocalTransformation()
    M = res[0] if isinstance(res, tuple) else res
    t = M.ExtractTranslation()
    target_pos = Gf.Vec3f(float(t[0]), float(t[1]), float(t[2]))

articulation = Articulation("/World/franka")
rrt = create_rrt()
path_planner_visualizer = PathPlannerVisualizer(articulation, rrt)

plan = path_planner_visualizer.compute_plan_as_articulation_actions(max_cspace_dist=.01)
current_target_orientation = np.array([1.0, 0.0, 0.0, 0.0])
current_target_translation = np.array([target_pos[0], target_pos[1], target_pos[2]])
current_target_rotation = quats_to_rot_matrices(current_target_orientation)
translation_distance = np.linalg.norm(target_translation - current_target_translation)
rotation_distance = rotational_distance_angle(current_target_rotation, target_rotation)

steps = 600 if success else 120
for i in range(steps):
    if plan:
        action = plan.pop(0)
        articulation.apply_action(action)
