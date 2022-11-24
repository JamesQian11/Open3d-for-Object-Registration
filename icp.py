import open3d as o3d
import numpy as np
import copy
import time
import os
import sys

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)
    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(pcd_down,o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def load_point_clouds(voxel_size=0.0):
    pcds_down = []
    fpfh = []
    for i in range(46,50):
        pcd = o3d.io.read_point_cloud("Test_data/255/depth_novoxel_%d.ply" %i)
        #pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
        pcd_down, pcd_fpfh = preprocess_point_cloud(pcd, voxel_size)
        pcds_down.append(pcd_down)
        fpfh.append(pcd_fpfh)
    return pcds_down, fpfh

voxel_size = 3
pcds_down,pcds_fpfh = load_point_clouds(voxel_size)
o3d.visualization.draw_geometries(pcds_down)
o3d.visualization.draw_geometries(pcds_fpfh)