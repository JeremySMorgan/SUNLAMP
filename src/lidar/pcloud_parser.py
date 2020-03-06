
if __name__ == "__main__":
    import open3d as o3d
    import pcl
    import pcl.pcl_visualization

import time
import numpy as np
import matplotlib.pyplot as plt
from src.utils.math_utils import MathUtils
from klampt import vis
from klampt import WorldModel
from klampt.model import trajectory
from klampt.robotsim import PointCloud
from klampt.robotsim import TriangleMesh
from klampt.robotsim import Geometry3D
from scipy.optimize import minimize
import klampt
from klampt.model import ik,coordinates,config,trajectory,collide
from scipy.ndimage.filters import maximum_filter as maxf2D
from src.utils.data_objects.height_map import HeightMap
from src.utils.logger import Logger as Lg


class PCloudParser:

    HM_IDXS_PER_CM = 1
    PCLOUD_COLOR = (0,0,0,1)

    def __init__(self):
        pass

    @staticmethod
    def get_ground_z(pc_xyz_np):

        pc_zs = np.array(pc_xyz_np[:, 2])
        # print "point cloud Z's shape:", pc_zs.shape

        # Find estimate of floor's Z
        n_bins = 300
        ballpark_hist, ballpark_bin_edges = np.histogram(pc_zs, bins=n_bins)
        ballpark_floor_z_idx = np.argmax(ballpark_hist)
        ballpark_floor_z = ballpark_bin_edges[ballpark_floor_z_idx]
        ballpark_zs = pc_zs[np.abs(pc_zs[:] - ballpark_floor_z) < .025]
        # print "ballpark z's shape:", ballpark_zs.shape

        # Fine search centered at floor's Z
        n_bins = 300
        hist, bin_edges = np.histogram(ballpark_zs, bins=n_bins)
        floor_z_idx = np.argmax(hist)
        floor_z = bin_edges[floor_z_idx]
        return floor_z

    @staticmethod
    def best_fitting_plane(xyzs, debug=False):

        if not type(xyzs) == np.array:
            xyzs = np.array(xyzs)

        downsample_factor = 1
        downsampled = xyzs[0:len(xyzs): downsample_factor]

        x_min = np.min(downsampled[:, 0])
        y_min = np.min(downsampled[:, 1])

        x0 = x_min
        y0 = y_min

        downsampled_xs = downsampled[:, 0]
        downsampled_ys = downsampled[:, 1]
        downsampled_zs = downsampled[:, 2]

        round_amt = 6

        def mse_(p):

            a = p[0]
            b = p[1]
            c = p[2]
            z0 = p[3]
            plane_specified_zs = (c*z0 - a*(downsampled_xs - x0) - b*(downsampled_ys - y0)) / c
            error = np.sum(np.abs(plane_specified_zs - downsampled_zs))

            if debug:
                print("a:", round(a, round_amt)," b:",round(b, round_amt),"c:", round(c, round_amt)," z0:",round(z0, round_amt), "error :", error)
            return error

        bounds = ((-1, 1), (-1, 1), (-1, 1), (-10, 10))
        p0 = np.array([0, 0, 1, np.average(xyzs[:,2])])

        res = minimize(mse_, p0, bounds=bounds)
        p_opt = res.x
        a, b, c, z0 = p_opt[0], p_opt[1], p_opt[2], p_opt[3]

        if debug:
            print("returning [a, b, c, zo] a:", round(a, round_amt), " b:", round(b, round_amt), "c:", round(c, round_amt), " z0:",
              round(z0, round_amt), "error :", mse_(p_opt))

        return a, b, c, x0, y0, z0

    def planar_point_cloud(self, a, b, c, x0, y0, z0, x_width, nb_x_vals_p_width, y_width, nb_y_vals_p_width):

        x_vals = np.linspace(x0, x0+x_width, nb_x_vals_p_width)
        y_vals = np.linspace(y0, y0+y_width, nb_y_vals_p_width)

        xx, yy = np.meshgrid(x_vals, y_vals)
        z = (c*z0 - a*xx + a*x0 - b*yy + b*y0) / c
        xyz = np.zeros((xx.shape[0]*xx.shape[1], 3))

        xx_flattened = xx.flatten()
        yy_flattened = yy.flatten()
        z_flattened = z.flatten()

        xyz[:, 0] = xx_flattened
        xyz[:, 1] = yy_flattened
        xyz[:, 2] = z_flattened

        return xyz

    def visualize_np_list(self, xyz_np):
        pcd = self.o3d_pcloud_from_np_xyz(xyz_np)
        o3d.visualization.draw_geometries([pcd])

    def o3d_pcloud_from_np_xyz(self, xyz_np):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz_np)
        return pcd

    @staticmethod
    def get_transformation(pcd_fname):

        if "pc_1" in pcd_fname or "pc_2" in pcd_fname or "pc_3" in pcd_fname:
            theta_y = np.deg2rad(-5)
            T_y = np.array([[np.cos(theta_y), 0, np.sin(theta_y), 0],
                            [0, 1, 0, 0],
                            [-np.sin(theta_y), 0, np.cos(theta_y), 0],
                            [0, 0, 0, 0]])

            theta_x = np.deg2rad(0)
            T_x = np.array([[1, 0, 0, 0],
                            [0, np.cos(theta_x), -np.sin(theta_x), 0],
                            [0, np.sin(theta_x), np.cos(theta_x), 0],
                            [0, 0, 0, 0]])
            T = np.matmul(T_y, T_x)

        else:
            T = np.array([[1, 0, 0, 0],
                          [0, 1, 0, 0],
                          [0, 0, 1, 0],
                          [0, 0, 0, 0]])
        return T


    def parse_hm(self, pcd_fname, visualize=False, debug=False):

        coordinate_frame_mesh = o3d.geometry.create_mesh_coordinate_frame(size=0.6, origin=[0, 0, 0])

        voxel_size = .01

        # pcd: o3d.geometry.PointCloud = o3d.io.read_point_cloud(pcd_fname)
        pcd: o3d.geometry.PointCloud = o3d.io.read_point_cloud(pcd_fname)
        pcd: o3d.geometry.PointCloud = o3d.geometry.voxel_down_sample(pcd, voxel_size=voxel_size)
        pc_xyzs = np.asarray(pcd.points)

        x_width_rob_mask = 1.7
        y_width_rob_mask = 1
        robot_mask = np.logical_and(np.abs(pc_xyzs[:, 0]) < x_width_rob_mask, np.abs(pc_xyzs[:, 1]) < y_width_rob_mask)

        pc_xyzs_norobot = pc_xyzs[np.logical_not(robot_mask)]
        no_robot_pcd = o3d.geometry.PointCloud()
        no_robot_pcd.points = o3d.utility.Vector3dVector(pc_xyzs_norobot)
        pcd, _ = o3d.geometry.statistical_outlier_removal(no_robot_pcd, nb_neighbors=15, std_ratio=1.5)

        T = self.get_transformation(pcd_fname)
        pcd.transform(T)

        pc_xyzs = np.asarray(pcd.points)
        floor_z = PCloudParser.get_ground_z(pc_xyzs)
        if debug:
            print(f"ground z: {floor_z}")

        x_min = np.min(pc_xyzs[:, 0])
        y_min = np.min(pc_xyzs[:, 1])
        pcd.translate([-x_min, -y_min, -floor_z])
        pc_xyzs = np.asarray(pcd.points)

        if debug:
            print(f" pc pre cieling mask filter has {len(pc_xyzs)} points")

        # Filter all points above k meters
        cieling_mask = pc_xyzs[:, 2] < 2.5
        pc_xyzs = pc_xyzs[cieling_mask]

        if debug:
            print(f" pc post cieling mask filter has {len(pc_xyzs)} points")
            print("new ground z: ", PCloudParser.get_ground_z(pc_xyzs))

        floor_threshold = .04
        floor_mask = np.abs(pc_xyzs[:, 2]) < floor_threshold

        floor_points_xyz = pc_xyzs[floor_mask]
        floor_pcd = o3d.geometry.PointCloud()
        floor_pcd.points = o3d.utility.Vector3dVector(floor_points_xyz)

        # non_floor_mask = np.logical_not(floor_mask)
        non_floor_mask = pc_xyzs[:, 2] > floor_threshold + .025
        non_floor_points_xyz = pc_xyzs[non_floor_mask]
        non_floor_pcd = o3d.geometry.PointCloud()
        non_floor_pcd.points = o3d.utility.Vector3dVector(non_floor_points_xyz)

        a0, b0, c0, x0, y0, z0 = PCloudParser.best_fitting_plane(floor_points_xyz, debug=False)
        ang_to_zaxis_rad = MathUtils.angle_between_two_3d_vectors([a0, b0, c0], [0, 0, 1])
        ang_to_zaxis_deg = np.rad2deg(ang_to_zaxis_rad)
        round_amt = 6
        if debug:
            print("a0, b0, c0 :", round(a0, round_amt), round(b0, round_amt), round(c0, round_amt), " angle to <0, 0, 1>:",
              round(ang_to_zaxis_deg, 3), " degrees")

        planar_pc = o3d.geometry.PointCloud()
        planar_pc.points = o3d.utility.Vector3dVector(self.planar_point_cloud(0, 0, 1, 5, 5, 0, 10, 25, 10, 25))

        if visualize:
            planar_pc.paint_uniform_color([1, 0, 0])
            floor_pcd.paint_uniform_color([1, 0.706, 0])
            o3d.visualization.draw_geometries([floor_pcd, non_floor_pcd, planar_pc, coordinate_frame_mesh])

        return pc_xyzs, non_floor_points_xyz

    def add_hm_to_klampt_vis(self, pcd_fname):

        pc_xyzs, non_floor_points_xyz = self.parse_hm(pcd_fname)
        pc_xyzs_as_list = pc_xyzs.flatten().astype("float")
        pcloud = PointCloud()
        pcloud.setPoints(int(len(pc_xyzs_as_list)/3), pc_xyzs_as_list)
        vis.add("pcloud", pcloud)
        r,g,b,a = PCloudParser.PCLOUD_COLOR
        vis.setColor("pcloud", r, g, b, a)

    def build_hm_obj(self, pcd_fname, visualize=False, inbound_xrange=None, inbound_yrange=None, debug=False):

        pc_xyzs, non_floor_points_xyz = self.parse_hm(pcd_fname, visualize=visualize, debug=debug)

        # -- Build Height Map
        if inbound_xrange:
            x_min = inbound_xrange[0]
            x_max = inbound_xrange[1]
        else:
            x_min = np.min(pc_xyzs[:, 0])
            x_max = np.max(pc_xyzs[:, 0])
        x_range = x_max - x_min

        if debug:
            print(f"x_min: {x_min}")
            print(f"x_max: {x_max}")

        if inbound_yrange:
            y_min = inbound_yrange[0]
            y_max = inbound_yrange[1]
        else:
            y_min = np.min(pc_xyzs[:, 1])
            y_max = np.max(pc_xyzs[:, 1])
        y_range = y_max - y_min

        x_vals_p_meter = PCloudParser.HM_IDXS_PER_CM * 100
        y_vals_p_meter = PCloudParser.HM_IDXS_PER_CM * 100

        nb_x_hm_idxs = int(x_range * x_vals_p_meter)
        nb_y_hm_idxs = int(y_range * y_vals_p_meter)

        height_map = np.zeros((nb_x_hm_idxs, nb_y_hm_idxs))

        surrounding_fill_width_idxs = 2

        # Proccess non floor points
        start_t = time.time()

        for xyz in non_floor_points_xyz:

            x = xyz[0]
            y = xyz[1]

            if x_min < x < x_max and y_min < y < y_max:

                z = xyz[2]

                hm_x_idx = int(((x - x_min) / x_range) * nb_x_hm_idxs)
                hm_y_idx = int(((y - y_min) / y_range) * nb_y_hm_idxs)

                idx_x_min = max(hm_x_idx - surrounding_fill_width_idxs, 0)
                idx_x_max = min(hm_x_idx + surrounding_fill_width_idxs, nb_x_hm_idxs-1)

                idx_y_min = max(hm_y_idx - surrounding_fill_width_idxs, 0)
                idx_y_max = min(hm_y_idx + surrounding_fill_width_idxs, nb_y_hm_idxs - 1)

                height_map[idx_x_min:idx_x_max, idx_y_min:idx_y_max] = z

        N, M = 2, 2
        P, Q = height_map.shape

        # Use 2D max filter and slice out elements not affected by boundary conditions
        maxed_hm = maxf2D(height_map, size=(M, N))

        # plt.imshow(maxed_hm[2180:2300, 1400:1500].transpose())
        # plt.imshow(maxed_hm[1400:1500, 2180:2300].transpose())
        # plt.imshow(maxed_hm.transpose())
        # plt.show()

        xvars = [0, x_max, 1/x_vals_p_meter]
        yvars = [0, y_max, 1/y_vals_p_meter]
        hm_obj = HeightMap(xvars, yvars)
        hm_obj.np_arr = maxed_hm
        runtime = round(time.time() - start_t, 2)
        if debug:
            Lg.log(f"done building height map in {runtime} seconds","")

        return hm_obj, runtime

    def get_xy_vars_from_pcloud(self, pcloud_fname: str):

        pc_xyzs, non_floor_points_xyz = self.parse_hm(pcloud_fname, visualize=False)
        x_max = np.max(pc_xyzs[:, 0])
        y_max = np.max(pc_xyzs[:, 1])

        x_vals_p_meter = PCloudParser.HM_IDXS_PER_CM * 100
        y_vals_p_meter = PCloudParser.HM_IDXS_PER_CM * 100

        xvars = [0, x_max, 1/x_vals_p_meter]
        yvars = [0, y_max, 1/y_vals_p_meter]

        return xvars, yvars

    def visualize_pc_in_klampt_vis(self, pcloud_fname):
        title = pcloud_fname+" klampt world"
        vis_window_id = vis.createWindow(title)
        vis.setWindowTitle(title)

        world = WorldModel()
        vis.add("world", world)

        pcd = o3d.io.read_point_cloud(pcloud_fname)
        print(pcd)
        pc_xyzs = np.asarray(pcd.points)
        pc_xyzs_as_list = pc_xyzs.flatten().astype("float")
        # pc_xyzs_as_list = np.array([1,0,0, 1.1, 0, 0, 0, 1, 0])
        pcloud = PointCloud()
        pcloud.setPoints(int(len(pc_xyzs_as_list)/3), pc_xyzs_as_list)
        print(pcloud.numPoints())

        vis.add("pcloud", pcloud)
        # vis.setColor("pcloud", 0, 0, 1, a=1)

        # vis.setAttribute("p1", "size", 5.0)

        box = klampt.GeometricPrimitive()
        box.setAABB([-1, -1, 0], [-0.9, -0.9, 0.2])
        g = klampt.Geometry3D(box)
        vis.add("box", g)
        vis.setColor("box", 0, 0, 1, 0.5)

        coordinates.setWorldModel(world)
        vis.add("coordinates", coordinates.manager())

        vis.show()
        while vis.shown():
            vis.lock()
            vis.unlock()
            time.sleep(0.01)
        vis.kill()

    def plot_1d_histogram(self, x, n_bins):
        plt.hist(x, bins=n_bins)
        plt.ylabel('Count')
        plt.show()
