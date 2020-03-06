import numpy as np
from klampt import vis
from klampt.model import trajectory
from src.utils.logger import Logger


from src.utils.math_utils import MathUtils


class GeneratorSuperclass:

    def __init__(self, ProjectConstants, height_map):
        self.height_map = height_map
        self.ProjectConstants = ProjectConstants
        self.x_start = self.height_map.x_vars[0]
        self.x_end = self.height_map.x_vars[1]
        self.y_start = self.height_map.y_vars[0]
        self.y_end = self.height_map.y_vars[1]

    def get_end_affector_xyz_coords_from_xy_yaw_deg_at_base_state(self, xy_yaw_deg, debug=False, visualize=False, with_x_margin=None, with_y_margin=None):
        yaw_rad = np.deg2rad(xy_yaw_deg[2])
        x = xy_yaw_deg[0]
        y = xy_yaw_deg[1]
        base = (self.ProjectConstants.BASE_STATE_END_EFF_DX_FROM_TORSO, self.ProjectConstants.BASE_STATE_END_EFF_DY_FROM_TORSO)
        fl = MathUtils._2d_rotation_transformation(base[0], base[1], yaw_rad)
        fr = MathUtils._2d_rotation_transformation(base[0], -base[1], yaw_rad)
        bl = MathUtils._2d_rotation_transformation(-base[0], base[1], yaw_rad)
        br = MathUtils._2d_rotation_transformation(-base[0], -base[1], yaw_rad)
        fl_x = x + fl[0]
        fl_y = y + fl[1]
        fr_x = x + fr[0]
        fr_y = y + fr[1]
        bl_x = x + bl[0]
        bl_y = y + bl[1]
        br_x = x + br[0]
        br_y = y + br[1]
        if (not self.xy_inbound(fl_x, fl_y, with_x_margin=with_x_margin, with_y_margin=with_y_margin)) or \
                (not self.xy_inbound(fr_x, fr_y, with_x_margin=with_x_margin, with_y_margin=with_y_margin)) or \
                (not self.xy_inbound(br_x, br_y, with_x_margin=with_x_margin, with_y_margin=with_y_margin)) or \
                (not self.xy_inbound(bl_x, bl_y, with_x_margin=with_x_margin, with_y_margin=with_y_margin)):
            if debug:
                print(" value error in get end effector xyz coords at base state!")
            return False, False, False, False
        if visualize:
            self.visualize_cost(fl_x, fl_y, 1, "FRONT LEFT")
            self.visualize_cost(fr_x, fr_y, 1, "FRONT RIGHT")
            self.visualize_cost(bl_x, bl_y, 1, "BACK LEFT")
            self.visualize_cost(br_x, br_y, 1, "BACK RIGHT")
        if debug:
            print("\nin get_end_affector_xy_coords_from_xy_yaw_at_base_state()")
            print("\n    input x:",x)
            print("          y:",y)
            print("          yaw:",xy_yaw_deg[2])
            print("\n  fl:",Logger.pp_list([fl_x, fl_y]))
            print("  fr:",Logger.pp_list([fr_x, fr_y]))
            print("  br:",Logger.pp_list([br_x, br_y]))
            print("  bl:",Logger.pp_list([bl_x, bl_y]))
        fl_z = self.height_map.height_at_xy(fl_x, fl_y)
        fr_z = self.height_map.height_at_xy(fr_x, fr_y)
        bl_z = self.height_map.height_at_xy(bl_x, bl_y)
        br_z = self.height_map.height_at_xy(br_x, br_y)
        return (fl_x, fl_y, fl_z), (fr_x, fr_y, fr_z), (br_x, br_y, br_z), (bl_x, bl_y, bl_z)

    def xy_inbound(self,x,y, with_x_margin=None, with_y_margin=None):

        x_margin = 0
        y_margin = 0
        if with_x_margin:
            x_margin = with_x_margin
        if with_y_margin:
            y_margin = with_y_margin

        if x > self.x_start+x_margin and x < self.x_end-x_margin and y > self.y_start+y_margin and y < self.y_end-y_margin:
            return True
        return False

    def visualize_cost(self, x, y, cost, name, with_z=0):
        traj = trajectory.Trajectory(milestones=[[x, y, with_z+.005], [x, y,with_z+cost]])
        vis.add(name, traj)

    def add_line_to_vis(self,name, p1, p2):
        traj = trajectory.Trajectory(milestones=[p2, p1])
        vis.add(name, traj)
