# Filename : 3D reconstruction.py
# Author   : James Qian
# Date     : 2020 01 19
import open3d as o3d
import numpy as np
import copy


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
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=10))
    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def load_point_clouds(voxel_size=0.0):
    pcds_down = []
    pcds_fpfh = []
    for i in range(5):
        pcd = o3d.io.read_point_cloud("../test_data/p40pro/p40pro2/depth_%d.ply" %i)
        pcd_down, pcd_fpfh = preprocess_point_cloud(pcd, voxel_size)
        pcds_down.append(pcd_down)
        pcds_fpfh.append(pcd_fpfh)
    return pcds_down, pcds_fpfh

def execute_global_registration(source_down, target_down, source_fpfh,target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.10f," % voxel_size)
    print("   we use a liberal distance threshold %.10f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,distance_threshold,
           o3d.pipelines.registration.TransformationEstimationPointToPoint(False),3,
         [ o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.5),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
           #o3d.pipelines.registration.CorrespondenceCheckerBasedOnNormal(0.1)
         ], o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 0.9))
    return result



def pairwise_registration(source, target,source_fpfh, target_fpfh, voxel_size):
    print("Apply point-to-point ICP")
    result_ransac = execute_global_registration(source, target, source_fpfh, target_fpfh, voxel_size)

    icp_fine = o3d.pipelines.registration.registration_icp(
        source, target, max_correspondence_distance_fine,
        result_ransac.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())

    transformation_icp = icp_fine.transformation
    information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
        source, target, max_correspondence_distance_fine,
        icp_fine.transformation)
    return transformation_icp, information_icp


def full_registration(pcds,fpfh,max_correspondence_distance_coarse,
                                   max_correspondence_distance_fine,voxel_size):

    pose_graph = o3d.pipelines.registration.PoseGraph()
    odometry = np.identity(4)
    pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
    n_pcds = len(pcds)
    for source_id in range(n_pcds):
        for target_id in range(source_id + 1, n_pcds):
            transformation_icp, information_icp = pairwise_registration(
                pcds[source_id], pcds[target_id],
                fpfh[source_id], fpfh[target_id],
                voxel_size)
            print("Build o3d.pipelines.registration.PoseGraph")
            if target_id == source_id + 1:  # odometry case
                odometry = np.dot(transformation_icp, odometry)
                pose_graph.nodes.append(
                    o3d.pipelines.registration.PoseGraphNode(
                        np.linalg.inv(odometry)))
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                             target_id,
                                                             transformation_icp,
                                                             information_icp,
                                                             uncertain=False))
            else:  # loop closure case
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id,
                                                             target_id,
                                                             transformation_icp,
                                                             information_icp,
                                                             uncertain=True))
    return pose_graph




voxel_size =2
max_correspondence_distance_coarse = voxel_size * 15
max_correspondence_distance_fine = voxel_size * 1.5
pcds_down,pcds_fpfh = load_point_clouds(voxel_size)
print("Full registration ...")
pose_graph = full_registration(pcds_down,pcds_fpfh,
                                   max_correspondence_distance_coarse,
                                   max_correspondence_distance_fine,voxel_size)
print("Optimizing PoseGraph ...")
option = o3d.pipelines.registration.GlobalOptimizationOption(
    max_correspondence_distance=max_correspondence_distance_fine,
    edge_prune_threshold=0.25,
    reference_node=0)
o3d.pipelines.registration.global_optimization(
        pose_graph,
        o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
        o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
        option)
print("Transform points and display")
pcd_combined = o3d.geometry.PointCloud()
for point_id in range(len(pcds_down)):
    print(pose_graph.nodes[point_id].pose)
    #print(pose_graph.edges[point_id].pose)
    pcds_down[point_id].transform(pose_graph.nodes[point_id].pose)
    pcd_combined += pcds_down[point_id]
o3d.io.write_point_cloud("../test_data/PLY_full/full_registration.ply", pcd_combined)
o3d.visualization.draw_geometries(pcds_down)
