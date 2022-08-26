import open3d as o3d

if __name__ == "__main__":
    # visualization of point clouds.
    pcd = o3d.io.read_point_cloud('test.ply')
    o3d.visualization.draw_geometries([pcd])