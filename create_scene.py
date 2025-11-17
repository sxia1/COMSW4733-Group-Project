import asyncio
import os
import numpy as np
import omni.usd
import omni.kit.app
from pxr import Usd, UsdGeom, Sdf, Gf, UsdPhysics, PhysxSchema
from omni.isaac.core import World
from omni.isaac.core.utils.prims import is_prim_path_valid
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.franka import Franka
from omni.isaac.motion_generation import ArticulationKinematicsSolver
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.objects.ground_plane import GroundPlane
from omni.physx.scripts import utils
from isaacsim.util.debug_draw import _debug_draw
from isaacsim.core.utils.extensions import get_extension_path_from_name
from isaacsim.core.prims import Articulation
from isaacsim.robot_motion.motion_generation.lula import RRT
from isaacsim.core.api.robots import Robot

def create_world(stage):
    if World.instance():
        World.instance().clear_instance()
    world = World()
    return world

def create_ground(stage):
    return GroundPlane(prim_path="/World/groundPlane", size=10, color=np.array([0,153,23]))

def create_tree(stage):
    tree_prim = add_reference_to_stage(
        usd_path="/root/denoised_mesh.usd",
        prim_path="/World/tree",
    )
    xf = UsdGeom.Xformable(tree_prim)
    xf = xf.GetTranslateOp()
    xf.Set(Gf.Vec3d(0.0, -0.15, 0.6))
    #UsdGeom.Gprim(tree_prim).CreateDisplayColorAttr([(164,116,73)]) 
    return tree_prim

# https://docs.isaacsim.omniverse.nvidia.com/latest/core_api_tutorials/tutorial_core_hello_robot.html
def create_robot(stage, world):
    robot = add_reference_to_stage(
        usd_path="/root/robots/ur5e_cutter_new_calibrated_precise_level/ur5e_cutter_new_calibrated_precise_level.usd",
        prim_path="/World/ur5e_cutter",
    )
    return robot

def create_scene(stage):
    print("creating world")
    world = create_world(stage)
    print("creating ground")
    ground = create_ground(stage)
    print("creating tree")
    tree = create_tree(stage)
    print("creating ur5e cutter")
    ur5e_cutter = create_robot(stage, world)

    # Fix the Tree to the Ground
    omni.physx.scripts.physicsUtils.add_joint_fixed(
        stage,
        "/World/tree/node_/FixedJoint",
        "/World/groundPlane",
        "/World/tree",
        localPos0 = Gf.Vec3f(0.0, -0.15, 0.6),
        #localPos0 = Gf.Vec3f(0.0, 0.0, 0.0),
        localRot0 = Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)),
        localPos1 = Gf.Vec3f(0.0, 0.0, 0.0),
        localRot1 = Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)),
        breakForce = float("inf"),
        breakTorque = float("inf")
    )

def create_debug_marker(point_list):
    draw = _debug_draw.acquire_debug_draw_interface()
    N = len(point_list)
    print("drawing points")
    draw.draw_points(point_list, [(1, 0, 0, 1)] * N, [20] * N)

def get_world_transform(prim_path):
    timeline = omni.timeline.get_timeline_interface()
    timecode = timeline.get_current_time() * timeline.get_time_codes_per_seconds()
    prim = stage.GetPrimAtPath(prim_path)
    pose = omni.usd.utils.get_world_transform_matrix(prim, timecode)
    print("Matrix Form:", pose)

def create_rrt():
    mg_extension_path = get_extension_path_from_name("isaacsim.robot_motion.motion_generation")
    rmp_config_dir = os.path.join(mg_extension_path, "motion_policy_configs")
    rrt_config_dir = os.path.join(mg_extension_path, "path_planner_configs")
    print(mg_extension_path, rmp_config_dir, rrt_config_dir)
    """
    rrt = RRT(
        robot_description_path = rmp_config_dir + "/franka/rmpflow/robot_descriptor.yaml",
        urdf_path = rmp_config_dir + "/franka/lula_franka_gen.urdf",
        rrt_config_path = rrt_config_dir + "/franka/rrt/franka_planner_config.yaml",
        end_effector_frame_name = "tool_center"
    )
    rrt.set_max_iterations(5000)
    return rrt
    """

stage = omni.usd.get_context().get_stage()
create_scene(stage)
get_world_transform("/World/tree")
get_world_transform("/World/ur5e_cutter")

point_list = [(-0.1, 1.0, 0.92)]
create_debug_marker(point_list)

target_pos = Gf.Vec3f(-0.1, 1.0, 0.92)
articulation = Articulation("/World/ur5e_cutter")
rrt = create_rrt()



"""
target_pos = Gf.Vec3f(0.55, 0.0, 0.30)
if branch_prim and branch_prim.IsValid():
    xfm = UsdGeom.Xformable(branch_prim)
    res = xfm.GetLocalTransformation()
    M = res[0] if isinstance(res, tuple) else res
    t = M.ExtractTranslation()
    target_pos = Gf.Vec3f(float(t[0]), float(t[1]), float(t[2]))

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
"""
