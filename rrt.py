import asyncio
import os
import numpy as np
import omni.usd
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
from isaacsim.core.utils.extensions import get_extension_path_from_name
from isaacsim.core.prims import Articulation
from isaacsim.robot_motion.motion_generation.lula import RRT
from isaacsim.core.api.robots import Robot

def create_world(stage):
    if World.instance():
        World.instance().clear_instance()
    world = World()
    """
    if stage.GetDefaultPrim() is None:
        world_xf = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
        stage.SetDefaultPrim(world_xf.GetPrim())
    try:
        world = World.instance()
    except Exception:
        world = None
    if world is None:
        world = World(stage_units_in_meters=1.0)
    #    world.scene.clear()
    """
    return world

def create_ground(stage):
    return GroundPlane(prim_path="/World/groundPlane", size=10, color=np.array([0,153,23]))

# https://docs.isaacsim.omniverse.nvidia.com/5.0.0/python_scripting/environment_setup.html#convert-asset-to-usd
async def convert_asset_to_usd(input_obj: str, output_usd: str):
    import omni.kit.asset_converter
    def progress_callback(progress, total_steps):
        pass
    converter_context = omni.kit.asset_converter.AssetConverterContext()
    # setup converter and flags
    # converter_context.ignore_material = False
    # converter_context.ignore_animation = False
    # converter_context.ignore_cameras = True
    # converter_context.single_mesh = True
    # converter_context.smooth_normals = True
    # converter_context.preview_surface = False
    # converter_context.support_point_instancer = False
    # converter_context.embed_mdl_in_usd = False
    converter_context.use_meter_as_world_unit = True
    # converter_context.create_world_as_default_root_prim = False
    instance = omni.kit.asset_converter.get_instance()
    task = instance.create_converter_task(input_obj, output_usd, progress_callback, converter_context)
    success = await task.wait_until_finished()
    if not success:
        carb.log_error(task.get_status(), task.get_detailed_error())
    print("converting done")

"""
asyncio.ensure_future(
    convert_asset_to_usd(
        "/root/denoised_mesh.stl",
        "/root/denoised_mesh.usd",
    )
)
"""

def create_tree(stage):
    tree_prim = add_reference_to_stage(
        usd_path="/root/data/denoised_mesh.usd",
        prim_path="/World/tree"
    )
    xf = UsdGeom.Xformable(tree_prim)
    xf.SetXformOpOrder([])
    xf.AddTranslateOp().Set(Gf.Vec3d(0.0,-0.15,0.6))
    UsdGeom.Gprim(tree_prim).CreateDisplayColorAttr([(164,116,73)]) 
    utils.setRigidBody(tree_prim, "convexDecomposition", False)
    return tree_prim
    
def get_world_transform(prim_path):
    timeline = omni.timeline.get_timeline_interface()
    timecode = timeline.get_current_time() * timeline.get_time_codes_per_seconds()
    prim = stage.GetPrimAtPath(prim_path)
    pose = omni.usd.utils.get_world_transform_matrix(prim, timecode)
    print("Matrix Form:", pose)

# https://docs.isaacsim.omniverse.nvidia.com/latest/core_api_tutorials/tutorial_core_hello_robot.html
def create_robot(stage, world):
    return add_reference_to_stage(
        usd_path="/root/robots/ur5e_cutter_new_calibrated_precise_level/ur5e_cutter_new_calibrated_precise_level.usd",
        prim_path="/World/ur5e_cutter")
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
    print("Setting Physics")
    scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(981.0)
    PhysxSchema.PhysxSceneAPI.apply(stage.GetPrimAtPath("/World/physicsScene"))


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
create_scene(stage)
get_world_transform("/World/tree")
get_world_transform("/World/ur5e_cutter")

"""
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
"""
