import omni.usd
from pxr import Usd, UsdGeom, Sdf, Gf

stage = omni.usd.get_context().get_stage()

# Ensure /World exists
if stage.GetDefaultPrim() is None:
    world = UsdGeom.Xform.Define(stage, Sdf.Path("/World"))
    stage.SetDefaultPrim(world.GetPrim())

path = Sdf.Path("/World/branch")

# Create or get cube
if not stage.GetPrimAtPath(path):
    cube = UsdGeom.Cube.Define(stage, path)
else:
    cube = UsdGeom.Cube.Get(stage, path)

# Size, pose, color
cube.CreateSizeAttr(0.08)
xform = UsdGeom.Xformable(cube.GetPrim())
for op in xform.GetOrderedXformOps():
    xform.RemoveXformOp(op)
xform.AddTranslateOp().Set(Gf.Vec3f(0.55, 0.0, 0.30))
UsdGeom.Gprim(cube.GetPrim()).CreateDisplayColorAttr([(1.0, 0.1, 0.1)])

print("Cube created:", stage.GetPrimAtPath(path).IsValid())
