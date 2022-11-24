
import open3d as o3d
import numpy as np
import PIL.Image
import IPython.display
import os
import urllib.request
import tarfile
import gzip
import zipfile
import shutil



pcd = o3d.io.read_point_cloud("Test_data/012503/1.ply")
#o3d.visualization.draw_geometries([pcd])
pcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=0.06, max_nn=30))
print('run Poisson surface reconstruction')
mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=9)
print(mesh)
o3d.visualization.draw_geometries([mesh])