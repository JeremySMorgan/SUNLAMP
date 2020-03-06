import time

import numpy as np
from src.utils.data_objects.footstep_scatter import FootstepScatter
from src.utils.logger import Logger


from src.utils.math_utils import MathUtils
from src.utils import config

class ScatterListGenerator:

    def __init__(self, fs_height_map, footstep_fs_cost_map, xy_yaw0):

        self.fs_cost_map = footstep_fs_cost_map
        self.height_map = fs_height_map
        self.xy_yaw0 = xy_yaw0

        self.fs_scatter_obj = FootstepScatter()

        self.x_granularity = config.X_STEP_SIZE_FS_SCATTER
        self.x_start = self.height_map.x_start
        self.x_end = self.height_map.x_end
        self.y_granularity = config.Y_STEP_SIZE_FS_SCATTER
        self.y_start = self.height_map.y_start
        self.y_end = self.height_map.y_end

    def build_list(self, debug=False):
        '''
            returns runtime
        '''
        start_t = time.time()
        # scatter = self.fs_scatter_obj.get_scatter_list()

        world_x = self.x_start
        world_y = self.y_start

        while world_x <= self.x_end:
            while world_y < self.y_end:

                try:
                    cost = self.fs_cost_map.cost_at_xy(world_x, world_y)
                    z = self.height_map.height_at_xy(world_x, world_y)
                    # scatter.append((world_x, world_y, z, cost))
                    self.fs_scatter_obj.append_to_scatter((world_x, world_y, z, cost))
                except ValueError:
                    print("value error. world x,y: ", world_x, world_y)


                world_y += config.Y_STEP_SIZE_FS_SCATTER
            world_x += config.X_STEP_SIZE_FS_SCATTER
            world_y = self.y_start

        # Add Starting Leg Positions to scatter
        try:
            fl_xyz, fr_xyz, br_xyz, bl_xyz = self.get_end_eff_xy_basestate_coords_from_xy_yaw(self.xy_yaw0)
        except ValueError:
            Logger.log("Error: robot is outside of search area", "FAIL"); return

        fl_xyzc = (fl_xyz[0], fl_xyz[1], fl_xyz[2], self.fs_cost_map.cost_at_xy(fl_xyz[0],fl_xyz[1]))
        fr_xyzc = (fr_xyz[0], fr_xyz[1], fr_xyz[2], self.fs_cost_map.cost_at_xy(fr_xyz[0],fr_xyz[1]))
        br_xyzc = (br_xyz[0], br_xyz[1], br_xyz[2], self.fs_cost_map.cost_at_xy(br_xyz[0],br_xyz[1]))
        bl_xyzc = (bl_xyz[0], bl_xyz[1], bl_xyz[2], self.fs_cost_map.cost_at_xy(bl_xyz[0],bl_xyz[1]))

        self.fs_scatter_obj.append_to_scatter(fl_xyzc, add_to_idx_tree=False)
        self.fs_scatter_obj.append_to_scatter(fr_xyzc, add_to_idx_tree=False)
        self.fs_scatter_obj.append_to_scatter(br_xyzc, add_to_idx_tree=False)
        self.fs_scatter_obj.append_to_scatter(bl_xyzc, add_to_idx_tree=False)

        if debug:
            print(f"fs scatter built in: {round(time.time() - start_t,2)} seconds with: {len(self.fs_scatter_obj.nb_points())} elements")

        return time.time() - start_t

    def save_list(self,file_name):
        self.fs_scatter_obj.save(file_name)

    def print_list(self):
        self.fs_scatter_obj.print_scatter()

    def return_scatter(self):
        return self.fs_scatter_obj

    def get_end_eff_xy_basestate_coords_from_xy_yaw(self, xy_yaw_deg):

        '''
        :param xy_yaw_deg:
        :return:
        '''

        yaw_rad = np.deg2rad(xy_yaw_deg[2])
        x = xy_yaw_deg[0]
        y = xy_yaw_deg[1]
        base = (config.BASE_STATE_END_EFF_DX_FROM_TORSO, config.BASE_STATE_END_EFF_DY_FROM_TORSO)

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

        fl_z = self.height_map.height_at_xy(fl_x, fl_y)
        fr_z = self.height_map.height_at_xy(fr_x, fr_y)
        bl_z = self.height_map.height_at_xy(bl_x, bl_y)
        br_z = self.height_map.height_at_xy(br_x, br_y)
        return (fl_x, fl_y, fl_z), (fr_x, fr_y, fr_z), (br_x, br_y, br_z), (bl_x, bl_y, bl_z)