from glob import glob
import numpy as np
import open3d as o3d
import os
import re


def create_point_clouds():
    file_dir = os.path.dirname(os.path.abspath(__file__))
    config = ""
    for pos_file in glob(os.path.join(file_dir, "positions", "*.txt")):
        with open(pos_file, "r") as f:
            data = f.read().replace("\t", "    ").replace("    ", " ").replace("  ", " ").splitlines()
        skin_name = os.path.basename(pos_file).split(".")[0]
        parent_link_name = re.findall("name +(.*)", data[0])[0]
        pc_points = []
        for line in data[4:]:
            xyz = list(map(float, line.split(" ")[:3]))
            if np.sum(xyz) == 0:
                continue
            pc_points.append(xyz)
        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pc_points))
        pc.estimate_normals()
        if "foot" not in skin_name and "hand" not in skin_name:
            pc.orient_normals_consistent_tangent_plane(10)
        else:
            pc.orient_normals_towards_camera_location()
            pc.normals = o3d.utility.Vector3dVector(np.asarray(pc.normals) * -1)
        pc.normalize_normals()
        pc.paint_uniform_color([1, 0, 0])
        #o3d.visualization.draw_geometries([pc], point_show_normal=True)
        o3d.io.write_point_cloud(os.path.join(file_dir, "point_clouds", f"{skin_name}.pcd"), pc)
        config += f"{skin_name};{parent_link_name}\n"

    with open(os.path.join(file_dir, "point_clouds", "config.txt"), "w") as f:
        f.write(config)


if __name__ == "__main__":
    create_point_clouds()
