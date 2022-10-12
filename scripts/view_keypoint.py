#!/usr/bin/env python3
import os
import open3d as o3d

def keypoints_to_spheres(keypoints):
    spheres = o3d.geometry.TriangleMesh()
    for keypoint in keypoints.points:
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.015)
        sphere.translate(keypoint)
        spheres += sphere
    spheres.paint_uniform_color([1.0, 0.0, 0.0])
    return spheres

if __name__ == "__main__":
    if 'SVGA_VGPU10' in os.environ:    # this environment variable may exist in VMware virtual machines
        del os.environ['SVGA_VGPU10']  # remove it to launch Open3D visualizer properly

    os.system('../build/test_keypoint ../config/config.yaml')
    
    cloud = o3d.io.read_point_cloud("../results/cloud.pcd")
    keypoints = o3d.io.read_point_cloud("../results/keypoint.pcd")

    o3d.visualization.draw_geometries([cloud, keypoints_to_spheres(keypoints)])
