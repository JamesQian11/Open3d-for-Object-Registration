import open3d as o3d
import numpy as np

#pcd = o3d.io.read_point_cloud("../test_data/p40pro/p40pro2/raw_7.ply")
pcd = o3d.io.read_point_cloud("depth_20210315144025596.xyz")
o3d.visualization.draw_geometries([pcd])
cl, ind = pcd.remove_statistical_outlier(nb_neighbors=10, std_ratio=5)
inlier_cloud = pcd.select_by_index(ind)
o3d.visualization.draw_geometries([inlier_cloud])
#o3d.io.write_point_cloud("1.ply", pcd)

'''
voxel_down_pcd = pcd.voxel_down_sample(voxel_size=0.001)
o3d.visualization.draw_geometries([voxel_down_pcd])
uni_down_pcd = pcd.uniform_down_sample(every_k_points=10)
o3d.visualization.draw_geometries([uni_down_pcd])
cl, ind = uni_down_pcd.remove_statistical_outlier(nb_neighbors=10, std_ratio=5)
inlier_cloud = pcd.select_by_index(ind)
#o3d.io.write_point_cloud("Test_data/Ply_data/20_1.ply", inlier_cloud)
    #pcd_1=inlier_cloud.voxel_down_sample(voxel_size=0.02)
#o3d.visualization.draw_geometries([inlier_cloud])


def load_point_clouds(voxel_size=0.0):
    pcds = []
    for i in range(1,100):
        pcd = o3d.io.read_point_cloud("Test_data/1/depth_%d.ply" %i)
        pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
        pcds.append(pcd_down)
    return pcds

#%%

voxel_size = 0.02
pcds_down = load_point_clouds(voxel_size)
o3d.visualization.draw_geometries(pcds_down)
'''