from meshlib import mrmeshpy as mm
from pathlib import Path

wdir = Path(__file__).parent
pc = mm.loadPoints(wdir / "tree_1_V_0011_merged_teaser.ply")
#samplingSettings = mm.UniformSamplingSettings()
#samplingSettings.distance = 1e-3
#pc.validPoints = mm.pointUniformSampling(pc, samplingSettings)
#pc.invalidateCaches()
test_mesh = mm.triangulatePointCloud(pc)

mm.saveMesh(test_mesh,"noisy_mesh.stl")

# Create parameters for adding noise
nSettings = mm.NoiseSettings()
nSettings.sigma = test_mesh.computeBoundingBox().diagonal()*0.0001
# Add noise to mesh
mm.addNoise(test_mesh.points,test_mesh.topology.getValidVerts(),nSettings)
# Save noised mesh
mm.saveMesh(test_mesh,"noised_mesh.stl")
# Denoise mesh with sharpening for sharp edges
# see the article "Mesh Denoising via a Novel Mumford-Shah Framework"
dnSettings = mm.DenoiseViaNormalsSettings()
dnSettings.beta = 0.1
dnSettings.gamma = 1.0
mm.meshDenoiseViaNormals( test_mesh, dnSettings)
# Save denoised mesh
mm.saveMesh(test_mesh,"denoised_mesh.stl")
