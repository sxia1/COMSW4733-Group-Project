import json
import sys
import csv
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

LABELS_CSV = "./data/labels_teaser.csv"

csv.field_size_limit(sys.maxsize)
pruning_labels = []
with open(LABELS_CSV) as f:
    pruning_labels = list(csv.DictReader(f))


for item in pruning_labels:
    tree = item["tree"]
    labels = np.asarray(json.loads(item["labels"]))

    pcd = o3d.io.read_point_cloud(f"./data/raw/{tree}_teaser.ply")
    points = np.asarray(pcd.points)
    pruned_pts = points[labels == 1]
    unpruned_pts = points[labels == 0]

    tree = cKDTree(unpruned_pts)

    # Radius to consider "bordering"
    R = 0.05

    neighbor_idx = tree.query_ball_point(pruned_pts, r=R)

    # Cut points = pruned points that have at least one unpruned neighbor
    pruning_points = pruned_pts[np.array([len(n) > 0 for n in neighbor_idx])]
    print(pruning_points)
    print(len(pruning_points))

    pts_unpruned = points[labels == 0]
    pts_pruned   = points[labels == 1]

    pcd_unpruned = o3d.geometry.PointCloud()
    pcd_unpruned.points = o3d.utility.Vector3dVector(pts_unpruned)
    pcd_unpruned.paint_uniform_color([0, 1, 0])

    pcd_pruned = o3d.geometry.PointCloud()
    pcd_pruned.points = o3d.utility.Vector3dVector(pts_pruned)
    pcd_pruned.paint_uniform_color([1, 0, 0])

    cut_points = o3d.geometry.PointCloud()
    cut_points.points = o3d.utility.Vector3dVector(pruning_points)
    cut_points.paint_uniform_color([0, 0, 1])

    o3d.visualization.draw_geometries(
        [pcd_unpruned, pcd_pruned, cut_points],
        window_name="Pruning Point Visualization",
        width=1280,
        height=720
    )
