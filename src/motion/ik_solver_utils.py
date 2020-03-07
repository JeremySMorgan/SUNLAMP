import numpy as np
from klampt.model import ik
from klampt.robotsim import IKSolver
from src.utils.logger import Logger
from src.motion.motion_utils import MotionUtils
from src.utils import project_constants

class IKSolverUtils(MotionUtils):

    def __init__(self, world, height_map,scatter_list, state_path, gradient_map):
        MotionUtils.__init__(self, world, height_map, scatter_list, state_path, gradient_map)

    def set_pose_w_R(self, r_pose):

        torso_xyz_yawdeg = r_pose.torso_xyz_yawdeg
        fl_xyz, fr_xyz, br_xyz, bl_xyz = r_pose.fl, r_pose.fr, r_pose.br, r_pose.bl
        fl_obj = ik.objective(self.fl_end_effector, R=fl_xyz[3], t=fl_xyz[0:3])
        fr_obj = ik.objective(self.fr_end_effector, R=fr_xyz[3], t=fr_xyz[0:3])
        bl_obj = ik.objective(self.bl_end_effector, R=bl_xyz[3], t=bl_xyz[0:3])
        br_obj = ik.objective(self.br_end_effector, R=br_xyz[3], t=br_xyz[0:3])

        des_torso_rotation = self.get_torso_R_from_yaw_rad(np.deg2rad(torso_xyz_yawdeg[3]))
        torso_obj = ik.objective(self.torso, R=des_torso_rotation, t=torso_xyz_yawdeg[0:3])
        bias_config = project_constants.NOMINAL_CONFIG
        bias_config[0] = torso_xyz_yawdeg[0]
        bias_config[1] = torso_xyz_yawdeg[1]
        bias_config[2] = torso_xyz_yawdeg[2]
        bias_config[3] = np.deg2rad(torso_xyz_yawdeg[3])
        self.robosimian.setConfig(bias_config)
        return ik.solve_global([torso_obj, fl_obj, fr_obj, bl_obj, br_obj])

    def set_pose(self, r_pose):

        torso_xyz_yawdeg = r_pose.torso_xyz_yawdeg
        torso_x, torso_y, torso_z, torso_yaw_deg = torso_xyz_yawdeg[0], torso_xyz_yawdeg[1], torso_xyz_yawdeg[2],torso_xyz_yawdeg[3]
        fl_xyz, fr_xyz, br_xyz, bl_xyz = r_pose.fl, r_pose.fr, r_pose.br, r_pose.bl
        yaw_rads = np.deg2rad(torso_yaw_deg)

        f_l_rotation = self.get_end_effector_rotation_matrix(1)
        f_r_rotation = self.get_end_effector_rotation_matrix(2)
        b_r_rotation = self.get_end_effector_rotation_matrix(3)
        b_l_rotation = self.get_end_effector_rotation_matrix(4)

        f_l_r_const = ik.objective(self.fl_end_effector, R=f_l_rotation, t=fl_xyz[0:3])
        f_r_r_const = ik.objective(self.fr_end_effector, R=f_r_rotation, t=fr_xyz[0:3])
        b_l_r_const = ik.objective(self.bl_end_effector, R=b_r_rotation, t=bl_xyz[0:3])
        b_r_r_const = ik.objective(self.br_end_effector, R=b_l_rotation, t=br_xyz[0:3])

        global_torso_xyz = [torso_x, torso_y, torso_z]
        des_torso_rotation = self.get_torso_R_from_yaw_rad(yaw_rads)
        torso_obj = ik.objective(self.torso, R=des_torso_rotation, t=global_torso_xyz)
        goals = [f_l_r_const, f_r_r_const, b_l_r_const, b_r_r_const, torso_obj]
        s = IKSolver(self.robosimian)
        for goal in goals:
            s.add(goal)
        # s.setTolerance(ik_max_deviation)
        s.setBiasConfig(project_constants.NOMINAL_CONFIG)
        res = s.solve()
        if not res:
            print("set_pose IK error")
            return False
        return True

    def set_q(self, q, debug=False):

        torso_x, torso_y, yaw_rads = self.estimate_torso_xy_yaw_rads_from_stance(q)
        if not torso_x:
            Logger.log(("stance: end config"), "OKGREEN")
            return
        torso_z = project_constants.TORSO_Z_DESIRED

        fl_global = self.adjust_endeff_z(self.scatter_list[q[0]][0:3])
        fr_global = self.adjust_endeff_z(self.scatter_list[q[1]][0:3])
        br_global = self.adjust_endeff_z(self.scatter_list[q[2]][0:3])
        bl_global = self.adjust_endeff_z(self.scatter_list[q[3]][0:3])

        if debug:
            print("    fl_global:",Logger.pp_list(fl_global))
            print("    fr_global:",Logger.pp_list(fr_global))
            print("    br_global:",Logger.pp_list(br_global))
            print("    bl_global:",Logger.pp_list(bl_global))

        f_l_r_const = ik.objective(self.fl_end_effector, R=self.get_end_effector_rotation_matrix(1, yaw_rad=yaw_rads), t=fl_global)
        f_r_r_const = ik.objective(self.fr_end_effector, R=self.get_end_effector_rotation_matrix(2, yaw_rad=yaw_rads), t=fr_global)
        b_l_r_const = ik.objective(self.bl_end_effector, R=self.get_end_effector_rotation_matrix(3, yaw_rad=yaw_rads), t=bl_global)
        b_r_r_const = ik.objective(self.br_end_effector, R=self.get_end_effector_rotation_matrix(4, yaw_rad=yaw_rads), t=br_global)

        global_torso_xyz = [torso_x, torso_y, torso_z]
        des_torso_rotation = self.get_torso_R_from_yaw_rad(yaw_rads)
        torso_obj = ik.objective(self.torso, R=des_torso_rotation, t=global_torso_xyz)
        bias_config = project_constants.NOMINAL_CONFIG
        goals = [f_l_r_const, f_r_r_const, b_l_r_const, b_r_r_const, torso_obj]

        s = IKSolver(self.robosimian)
        for goal in goals:
            s.add(goal)
        s.setBiasConfig(bias_config)
        res = s.solve()
        if not res:
            print("robot_poser IK Error")
            return False
        return True













