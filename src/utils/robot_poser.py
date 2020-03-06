import numpy as np
from klampt.math import so3
from klampt.model import ik
from klampt.robotsim import IKSolver
from src.utils.logger import Logger


class RobotPoser:

    def __init__(self, ProjectConstants, world, height_map, fs_scatter=None):

        self.ProjectConstants = ProjectConstants
        self.height_map = height_map
        self.world = world
        self.robosimian = world.robot(0)
        self.f_r = self.robosimian.link(13)
        self.f_l = self.robosimian.link(37)
        self.b_r = self.robosimian.link(21)
        self.b_l = self.robosimian.link(29)
        self.torso = self.robosimian.link(5)

        self.fs_scatter = fs_scatter
        if fs_scatter: self.scatter_list = fs_scatter.get_scatter_list()

    def reset_robot(self):
        self.robosimian.setConfig(self.ProjectConstants.NOMINAL_CONFIG)

    def set_end_effectors_xyz_torso_xyz_yaw(self,fl,fr,br,bl, torso_x,torso_y, torso_z, yaw_rads):

        global_torso_xyz = [torso_x, torso_y, torso_z]
        des_torso_rotation = self.get_torso_R_from_yaw(yaw_rads)
        torso_obj = ik.objective(self.torso, R=des_torso_rotation, t=global_torso_xyz)

        f_l_r_const = ik.objective(self.f_l, R=self.get_desired_end_effector_rotation(1, yaw_rads), t=fl)
        f_r_r_const = ik.objective(self.f_r, R=self.get_desired_end_effector_rotation(2, yaw_rads), t=fr)
        b_l_r_const = ik.objective(self.b_l, R=self.get_desired_end_effector_rotation(3, yaw_rads), t=bl)
        b_r_r_const = ik.objective(self.b_r, R=self.get_desired_end_effector_rotation(4, yaw_rads), t=br)

        bias_config = self.ProjectConstants.NOMINAL_CONFIG
        goals = [f_l_r_const, f_r_r_const, b_l_r_const, b_r_r_const,torso_obj]

        s = IKSolver(self.robosimian)
        for goal in goals: s.add(goal)
        s.setBiasConfig(bias_config)
        res = s.solve()
        if not res:
            print("robot_poser IK Error")
            return False
        return True

    def set_q(self, q, debug=False):

        torso_x, torso_y, yaw_rads = self.get_torso_xy_yaw(q)
        if not torso_x:
            return
        torso_z = self.ProjectConstants.TORSO_Z_DESIRED

        fl_global = self.replace_end_effector_z(self.scatter_list[q[0]][0:3])
        fr_global = self.replace_end_effector_z(self.scatter_list[q[1]][0:3])
        br_global = self.replace_end_effector_z(self.scatter_list[q[2]][0:3])
        bl_global = self.replace_end_effector_z(self.scatter_list[q[3]][0:3])

        if debug:
            print("    fl_global:",Logger.pp_list(fl_global))
            print("    fr_global:",Logger.pp_list(fr_global))
            print("    br_global:",Logger.pp_list(br_global))
            print("    bl_global:",Logger.pp_list(bl_global))

        f_l_r_const = ik.objective(self.f_l, R=self.get_desired_end_effector_rotation(1, yaw_rads), t=fl_global)
        f_r_r_const = ik.objective(self.f_r, R=self.get_desired_end_effector_rotation(2, yaw_rads), t=fr_global)
        b_l_r_const = ik.objective(self.b_l, R=self.get_desired_end_effector_rotation(3, yaw_rads), t=bl_global)
        b_r_r_const = ik.objective(self.b_r, R=self.get_desired_end_effector_rotation(4, yaw_rads), t=br_global)

        global_torso_xyz = [torso_x, torso_y, torso_z]
        torso_obj = ik.objective(self.torso, local=[0,0,0],world=global_torso_xyz)
        bias_config = self.ProjectConstants.NOMINAL_CONFIG
        goals = [f_l_r_const, f_r_r_const, b_l_r_const, b_r_r_const, torso_obj]

        s = IKSolver(self.robosimian)
        for goal in goals:
            s.add(goal)
        s.setBiasConfig(bias_config)
        res = s.solve()
        if not res:
            return False
        return True

    def replace_end_effector_z(self,xyz):
            xyz_new = [xyz[0],xyz[1],xyz[2]+self.ProjectConstants.END_EFFECTOR_HEIGHT]
            return xyz_new

    def always_true_func(self):
        return True

    def get_torso_R_from_yaw(self, yaw_rad):
        axis_angle = ([0, 0, 1], yaw_rad)
        desired_r = so3.from_axis_angle(axis_angle)
        return desired_r

    def get_desired_end_effector_rotation(self, leg_number, yaw_rads):
        if leg_number == 1 or leg_number == 3:
            r = [0, 0, -1, 0, -1, 0, -1, 0, 0]
        else:
            r = [0, 0, -1, 0, 1, 0, 1, 0, 0]
        yaw_rotation_aa = ([0, 0, 1], yaw_rads)
        yaw_rotation_R = so3.from_axis_angle(yaw_rotation_aa)
        return so3.mul(yaw_rotation_R, r)

    def set_xyz_yaw(self,x,y,z,yaw_rads):
        q = self.robosimian.getConfig()
        q[0] = x
        q[1] = y
        q[2] = z
        q[3] = yaw_rads
        self.robosimian.setConfig(q)

    def get_torso_xy_yaw(self, q):
        if q[4] == -1:
            return False, False, False
        fl_xyzc, fr_xyzc, br_xyzc, bl_xyzc = self.get_end_affectors_xyzc(q)
        x_ave = (fl_xyzc[0] + fr_xyzc[0] + bl_xyzc[0] + br_xyzc[0]) / 4.0
        y_ave = (fl_xyzc[1] + fr_xyzc[1] + bl_xyzc[1] + br_xyzc[1]) / 4.0
        right_side_v = np.array([fr_xyzc[0] - br_xyzc[0], fr_xyzc[1] - br_xyzc[1]])
        left_side_v = np.array([fl_xyzc[0] - bl_xyzc[0], fl_xyzc[1] - bl_xyzc[1]])
        right_side_normalized_v = right_side_v / np.linalg.norm(right_side_v)
        left_side_normalized_v = left_side_v / np.linalg.norm(left_side_v)
        ave_v = left_side_normalized_v + right_side_normalized_v
        yaw_rads = np.arctan2(ave_v[1], ave_v[0])
        return x_ave, y_ave, yaw_rads

    def get_end_affectors_xyzc(self, q):
        fl_xyzc = self.scatter_list[int(q[0])]
        fr_xyzc = self.scatter_list[int(q[1])]
        br_xyzc = self.scatter_list[int(q[2])]
        bl_xyzc = self.scatter_list[int(q[3])]
        return fl_xyzc, fr_xyzc, br_xyzc, bl_xyzc