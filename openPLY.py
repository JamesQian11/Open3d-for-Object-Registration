import open3d as o3d

#pcd = o3d.io.read_point_cloud("Test_data/0.ply")
#o3d.visualization.draw_geometries([pcd])

for i in range(10):
    for j in range(i + 1, 10):
       #print(i,j)
       if j==i+1:
        print(i,j)