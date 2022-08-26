import os
import open3d as o3d
import matplotlib.pyplot as plt

if __name__ == "__main__":
    path = "/simple_operations/"
    files = os.listdir(path)
    length = int(len(files)/2)
    n = 0
    for idx in range(length):
        color_raw = o3d.io.read_image(path + str(idx) + ".jpg")
        depth_raw = o3d.io.read_image(path + str(idx) + ".png")
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw, depth_raw)
        # print(rgbd_image)
        # plt.subplot(1, 2, 1)
        # plt.title('grayscale image')
        # plt.imshow(rgbd_image.color)
        # plt.subplot(1, 2, 2)
        # plt.title('depth image')
        # plt.imshow(rgbd_image.depth)
        # plt.show()
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd_image,
            o3d.camera.PinholeCameraIntrinsic(
                o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
        # Flip it, otherwise the pointcloud will be upside down
        pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        print("Saveing img ...")
        name = str(n)
        n += 1
        o3d.io.write_point_cloud(name + ".pcd", pcd)
        o3d.visualization.draw_geometries([pcd])

    # color_raw = o3d.io.read_image("0.jpg")
    # depth_raw = o3d.io.read_image("0.png")
    # rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw, depth_raw)
    # print(rgbd_image)
    # plt.subplot(1, 2, 1)
    # plt.title('grayscale image')
    # plt.imshow(rgbd_image.color)
    # plt.subplot(1, 2, 2)
    # plt.title('depth image')
    # plt.imshow(rgbd_image.depth)
    # plt.show()
    # pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
    #     rgbd_image,
    #     o3d.camera.PinholeCameraIntrinsic(
    #         o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
    # # Flip it, otherwise the pointcloud will be upside down
    # pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    # o3d.visualization.draw_geometries([pcd])
    # o3d.io.write_point_cloud("0.pcd", pcd)