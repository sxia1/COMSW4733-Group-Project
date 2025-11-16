import asyncio
import os
from pathlib import Path
import omni.kit.asset_converter

def convert_point_cloud_to_mesh(point_cloud_file, save_dir, smooth=True):
    mesh_filename = os.path.join(
        save_dir,
        f"{os.path.basename(point_cloud_file)[:-3]}stl"
    )
    return mesh_filename

# https://docs.isaacsim.omniverse.nvidia.com/5.0.0/python_scripting/environment_setup.html#convert-asset-to-usd
async def convert_asset_to_usd(input_obj: str, output_usd: str):
    mesh_filename = os.path.join(
        output_usd,
        f"{os.path.basename(input_obj)[:-3]}usd"
    )
    print(mesh_filename)
    def progress_callback(progress, total_steps):
        pass
    converter_context = omni.kit.asset_converter.AssetConverterContext()
    # setup converter and flags
    # converter_context.ignore_material = False
    # converter_context.ignore_animation = False
    # converter_context.ignore_cameras = True
    # converter_context.single_mesh = True
    converter_context.smooth_normals = True
    # converter_context.preview_surface = False
    # converter_context.support_point_instancer = False
    # converter_context.embed_mdl_in_usd = False
    converter_context.use_meter_as_world_unit = True
    # converter_context.create_world_as_default_root_prim = False
    instance = omni.kit.asset_converter.get_instance()
    task = instance.create_converter_task(input_obj, mesh_filename, progress_callback, converter_context)
    success = await task.wait_until_finished()
    if not success:
        carb.log_error(task.get_status(), task.get_detailed_error())
    

raw_data_dir = "/root/data/raw/"
for name in os.listdir(raw_data_dir):
    filename = os.path.join(raw_data_dir, name)
    print("Converting: ", filename)
    mesh_name = convert_point_cloud_to_mesh(filename, "/root/data/mesh")
    asyncio.ensure_future(
        convert_asset_to_usd(
            mesh_name,
            "/root/data/mesh"
        )
    )
    print("DONE Converting: ", filename)
    break

"""
# Load mesh
mesh = mm.loadMesh("denoised_mesh.stl")
 
# Setup parameters
params = mm.OffsetParameters()
params.voxelSize = mesh.computeBoundingBox().diagonal() * 5e-3  # offset grid precision (algorithm is voxel based)
if mm.findRightBoundary(mesh.topology).empty():
    params.signDetectionMode = mm.SignDetectionMode.HoleWindingRule  # use if you have holes in mesh
 
# Make offset mesh
offset = mesh.computeBoundingBox().diagonal() * 0.05
result_mesh = mm.offsetMesh(mesh, offset, params)
 
 
mv.launch() # a window will be opened
mv.addMeshToScene(mesh, "Mesh 1") # show initial mesh
mv.Viewer().preciseFitDataViewport() # fit viewer to the mesh
mv.selectByName("Mesh 1")
 
mv.Viewer().preciseFitDataViewport() # fit viewer to the mesh
mv.Viewer().showSceneTree(True) # enables Scene Tree in Viewer window
# user can manipulate with viewer window while this python is on pause
os.system("pause")
 
# remove all objects from scene
mv.clearScene()
 
# add offset mesh to scene
mv.addMeshToScene(result_mesh, "Mesh Offset")
mv.selectByName("Mesh Offset")
mv.Viewer().showSceneTree(False) # disables Scene Tree in Viewer window
# user can manipulate with viewer window while this python is on pause
os.system("pause")
 
# close viewer window nicely
mv.Viewer().shutdown()
os.system("pause")
"""
