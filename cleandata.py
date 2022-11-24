import open3d as o3d
import numpy as np

for i in range(91):
        pcd = o3d.io.read_point_cloud("../test_data/p40pro/p40pro2/raw_%d.ply"%(i))

        cl, ind = pcd.remove_statistical_outlier(nb_neighbors=30, std_ratio=2)
        inlier_cloud = pcd.select_by_index(ind)
        #pcd_down = inlier_cloud.voxel_down_sample(voxel_size=5)

        o3d.io.write_point_cloud("../test_data/p40pro/p40pro2/depth_%d.ply"%(i), inlier_cloud)





