import random
import time
import numpy as np
import signal
from src.utils.data_objects.high_level_trajectory import HLTrajectory
from src.utils.graph_search_utils.astar_fibheap import AStar
from src.utils.logger import Logger

from src.generators.generator_superclass import GeneratorSuperclass
from src.utils.math_utils import MathUtils
from src.utils import project_constants


class HighLevelTrajectoryPlanner(GeneratorSuperclass, AStar):

    '''State = (x, y, yaw, (x_prev, y_prev, yaw_prev), (x_2ndprev, y_2ndprev, yaw_2ndprev))
    '''

    def __init__(self,  height_map, fs_cost_map, xy_yaw0, xy_yawf, rposer=None):

        self.start_state = (xy_yaw0[0],xy_yaw0[1],xy_yaw0[2],(None,None,None),(None,None,None))
        self.hl_traj_astar_search = None
        self.xy_yawf = xy_yawf
        self.xy_yaw0 = xy_yaw0
        self.straight_path_yaw_rad = np.arctan2(self.xy_yawf[1]-self.xy_yaw0[1],self.xy_yawf[0]-self.xy_yaw0[0])
        self.fs_cost_map = fs_cost_map
        self.height_map = height_map

        self.vis_successor = project_constants.STEPSEQ_VISUALIZE_SUCCESSOR
        self.rposer = rposer

        self.directions = [
            [1.0, 1.0, 1.0],
            [1.0, 1.0, 0.0],
            [1.0, 1.0, -1.0],
            [1.0, 0, 1.0],
            [1.0, 0, 0.0],
            [1.0, 0, -1.0],
            [1.0, -1.0, 1.0],
            [1.0, -1.0, 0.0],
            [1.0, -1.0, -1.0],
            [0, 1.0, 1.0],
            [0, 1.0, 0.0],
            [0, 1.0, -1.0],
            [0, 0, 1.0],
            # [0, 0, 0.0],
            [0, 0, -1.0],
            [0, -1.0, 1.0],
            [0, -1.0, 0.0],
            [0, -1.0, -1.0],
            [-1.0, 1.0, 1.0],
            [-1.0, 1.0, 0.0],
            [-1.0, 1.0, -1.0],
            [-1.0, 0, 1.0],
            [-1.0, 0, 0.0],
            [-1.0, 0, -1.0],
            [-1.0, -1.0, 1.0],
            [-1.0, -1.0, 0.0],
            [-1.0, -1, -1]
        ]

        self.end_effector_x = project_constants.BASE_STATE_END_EFF_DX_FROM_TORSO
        self.end_effector_y = project_constants.BASE_STATE_END_EFF_DY_FROM_TORSO
        self.base_theta = np.arctan(self.end_effector_y / self.end_effector_x)
        self.xy_yaw_costs = {}
        self.decimal_round = 5
        GeneratorSuperclass.__init__(self, height_map)

        # Astar Search variables
        self.visited_state_nodes = {}
        self.g_weight = None
        self.h_weight = None
        # self.iterations = 0
        self.t_start = -1
        self.save_search_weights()
        self.xy_yaw_path = []
        self.smoothed_path = None

        AStar.__init__(self,state=self.start_state)

        self.inbounds_error = False
        if self.get_cost(xy_yawf[0], xy_yawf[1], xy_yawf[2]) > 10:
            Logger.log("Error: robot xy yaw_f is out of bounds", color="FAIL", class_id=1, msg_type=1)
            self.inbounds_error = True

    def save_search_weights(self, debug=False):
        self.g_weight = project_constants.HLTRAJ_G_WEIGHT
        self.h_weight = project_constants.HLTRAJ_H_WEIGHT

        if project_constants.STEPSEQ_VERBOSITY >= 3:
            print("g weight:",Logger.pp_double(self.g_weight),", h_weight:",Logger.pp_double(self.h_weight))

    def build_trajectory(self, suspend_after=None, save_high_density_xy_yaw_path=True, save_smoothed_path=True):
        '''
            returns runtime, -1 if doesn't complete after suspend_after seconds, -2 if inbounds error
        '''

        hl_traj = HLTrajectory()
        hl_traj.xy_yaw0 = self.xy_yaw0[:]
        hl_traj.xy_yawf = self.xy_yawf[:]

        if self.inbounds_error:
            Logger.log("Error: robot xy yaw_f is out of bounds", "FAIL", class_id=1, msg_type=1)
            return -2

        t_start = time.time()
        if project_constants.STEPSEQ_VERBOSITY >= 2:
            print("Starting high level trajectory search")

        if suspend_after:

            def handler(signum, frame):
                raise Exception("End of time")

            signal.signal(signal.SIGALRM, handler)
            signal.alarm(suspend_after)

            self.t_start = time.time()
            self.search()
            try:
                self.search()
            except Exception as e:
                if project_constants.STEPSEQ_VERBOSITY >= 1:
                    Logger.log(f"Failed to build a path after {round(suspend_after), 2} seconds", "FAIL")
                return -1

        else:
            self.search()

        if suspend_after: signal.alarm(0)

        path = self.result_path()

        if project_constants.STEPSEQ_VERBOSITY >= 2:
            print("High level trajectory search finished in:",time.time() - t_start, "s")

        state_path = []
        for node in path:
            state_path.append(node.state)
        state_path = state_path[:-1]
        for xy_yaw in state_path:
            self.xy_yaw_path.append([xy_yaw[0], xy_yaw[1], xy_yaw[2]])

        if save_smoothed_path:
            self.save_smoothed_path()

        if save_high_density_xy_yaw_path:
            self.save_high_density_xy_yaw_path()

        # Save results
        hl_traj.xy_yaw_path = self.xy_yaw_path[:]
        hl_traj.higher_density_xy_yaw_path = self.higher_density_xy_yaw_path[:]
        hl_traj.ave_higher_density_xy_yaw_path_distance_change = self.ave_higher_density_xy_yaw_path_distance_change
        hl_traj.ave_smooth_path_distance_change = self.ave_smooth_path_distance_change
        hl_traj.failed = False
        hl_traj.runtime = time.time() - t_start

        return hl_traj

    def return_trajectory(self):
        hl_traj = HLTrajectory()
        hl_traj.xy_yaw0 = self.xy_yaw0[:]
        hl_traj.xy_yawf = self.xy_yawf[:]
        hl_traj.xy_yaw_path = self.xy_yaw_path[:]
        hl_traj.higher_density_xy_yaw_path = self.higher_density_xy_yaw_path[:]
        hl_traj.ave_higher_density_xy_yaw_path_distance_change = self.ave_higher_density_xy_yaw_path_distance_change
        # hl_traj.smoothed_path = self.smoothed_path[:]
        hl_traj.ave_smooth_path_distance_change = self.ave_smooth_path_distance_change
        return hl_traj

    def save_high_density_xy_yaw_path(self):
        num_to_add_between_points = project_constants.HLTRAJ_NB_POINTS_BTWN_PATH_NODES
        extended_xy_yaw_path = MathUtils.list_extender(self.xy_yaw_path, num_to_add_between_points)
        self.higher_density_xy_yaw_path = extended_xy_yaw_path
        ave_dist = 0
        for i in range(0, len(extended_xy_yaw_path) - 2):
            xy_yaw1 = extended_xy_yaw_path[i]
            xy_yaw2 = extended_xy_yaw_path[i + 1]
            ave_dist += MathUtils._2d_euclidian_distance(xy_yaw1, xy_yaw2)
        ave_dist /= len(extended_xy_yaw_path)
        self.ave_higher_density_xy_yaw_path_distance_change = ave_dist

    def get_higher_density_xy_yaw_path(self):
        return self.higher_density_xy_yaw_path

    # --------------------- A* Functions
    def is_goal(self, state, debug=False):
        err_threshold = project_constants.HLTRAJ_GOAL_THRESHOLD
        dist = np.fabs(state[0]-self.xy_yawf[0])+ np.fabs(state[1]-self.xy_yawf[1])
        # if project_constants.STEPSEQ_VERBOSITY >= 3:
        #     if dist < 1:
        #         print("Dist for state:", Logger.pp_list(state), " to goal: ", Logger.pp_double(dist))
        return dist < err_threshold

    def successors(self, state):

        # stance = (x, y, yaw, (x_prev, y_prev, yaw_prev))
        # Aprox 1000 iterations/ 60s

        # self.iterations += 1
        successors, successor_costs = [],[]
        for direction in self.directions:
            x_new = self.round(state[0] + direction[0] * project_constants.HLTRAJ_DELTAX * np.cos(self.straight_path_yaw_rad))
            y_new = self.round(state[1] + direction[1] * project_constants.HLTRAJ_DELTAY * np.sin(self.straight_path_yaw_rad))
            yaw_new = self.round(state[2] + direction[2] * project_constants.HLTRAJ_YAW_OFFSET)

            # if state[3] is not None:
            #     new_state = (x_new, y_new, yaw_new, (state[0], state[1], state[2]), (state[3][0], state[3][1], state[3][2]))
            # else:
            #     new_state = (x_new, y_new, yaw_new, (state[0], state[1], state[2]), (None, None, None))

            new_state = (x_new, y_new, yaw_new, (state[0], state[1], state[2]))

            new_state_cost = self.g_weight * self.get_cost(new_state[0], new_state[1], new_state[2])

            successors.append(new_state)
            successor_costs.append(new_state_cost)

        # if self.debug:
        #     if random.randint(0,1000)> 950:
        #         node = self.visited_state_node(state)
        #         # self.is_goal(state, debug=True)
        #         # print "\n\n  current state:",logger.pp_list(state),"\t cost to node (node.g):",logger.pp_double(node.g)," node.h weight:",logger.pp_double(node.h),"  \t total cost (f):",logger.pp_double(node.f)
        #
        #         # print "  parent state:",logger.pp_list(parent_state
        #         t_elapsed = time.time() - self.t_start
        #         print "\n  current state:",logger.pp_list(state)
        #         print "  iterations:",self.iterations," t_elapsed:",t_elapsed, " hweight:",logger.pp_double(self.h_weight)
        #         # print "  path_yaw_est:",logger.pp_double(path_yaw_est)
        #         # print "  dist traveled (out of 1):",i
        #         # print
        #         # for i in range(len(successors)):
        #         #     print "  cost:",successor_costs[i],"\th:",logger.pp_double(self.heuristic(successors[i])),"\tq:",logger.pp_list(successors[i])

        if self.vis_successor:
            if random.randint(0,1000)>950:
                self.rposer.set_xyz_yaw(state[0], state[1], project_constants.TORSO_Z_DESIRED, np.deg2rad(state[2]))

        if project_constants.STEPSEQ_VERBOSITY >= 3:
            if random.randint(0,1000)> 950:
                node = self.visited_state_node(state)
                # self.is_goal(state, debug=True)
                print(f"\t q_xy: {Logger.pp_list(state[0:3])} \t dist to goal: {round(np.sqrt( (state[0] - self.xy_yawf[0])**2 + (state[1]-self.xy_yawf[1])**2 ), 2)} \t n.g: {Logger.pp_double(node.g)} \t n.h: {Logger.pp_double(node.h) }\tn.f: {Logger.pp_double(node.f)}")

                # print "  parent state:",logger.pp_list(parent_state
                # print "\n\n  current state:",logger.pp_list(state)
                # print "  path_yaw_est:",logger.pp_double(path_yaw_est)
                # print "  dist traveled (out of 1):",i
                # print
                # for i in range(len(successors)):  # type: int
                #     print "  cost:",successor_costs[i],"\th:",logger.pp_double(self.heuristic(successors[i])),"\tq:",logger.pp_list(successors[i])
                # time.sleep(2)

        return successors,successor_costs

    def clear_visited(self):
        self.visited_state_nodes.clear()

    def visit(self, state, node):
        self.visited_state_nodes[state] = node

    def visited_state_node(self, state):
        if state in self.visited_state_nodes:
            return self.visited_state_nodes[state]
        return None

    def heuristic(self, state):

        parent_state = state[3]
        # _2nd_parent_state = state[4]

        if parent_state == (None, None, None):
            path_yaw_est_deg = state[2] # just sets the yaw heuristic to 0
        else:
            path_yaw_est_deg = np.rad2deg(np.arctan2(state[1] - parent_state[1], state[0] - parent_state[0]))

        # total_dist = np.sqrt( (self.xy_yaw0[0] - self.xy_yawf[0])**2 + (self.xy_yaw0[1]-self.xy_yawf[1])**2 )
        xy_dist_to_goal = np.sqrt( (state[0] - self.xy_yawf[0])**2 + (state[1]-self.xy_yawf[1])**2 )
        dist_weight = 1.0
        # yaw_weight = 0.0
        yaw_weight = .001
        #
        # if xy_dist_to_goal / total_dist > .8:
        #     yaw_weight = 0

        return self.h_weight * (dist_weight*xy_dist_to_goal + yaw_weight*np.abs(state[2]-path_yaw_est_deg))
        # return self.h_weight * (dist_weight*xy_dist_to_goal)

    # --------------------- Helper Functions
    
    def save_smoothed_path(self):

        if project_constants.STEPSEQ_VERBOSITY >= 3:
            for xy_yaw in self.xy_yaw_path:
                print("  ",Logger.pp_list(xy_yaw))
        try:
            smoothed_path = MathUtils._3d_pointlist_cubic_interprolation(self.xy_yaw_path)
        except TypeError:
            Logger.log("Error: 'm > k must hold'", "FAIL")
            smoothed_path = self.xy_yaw_path

        self.smoothed_path = smoothed_path
        ave_dist = 0
        for i in range(0, len(smoothed_path) - 2):
            xy_yaw1 = smoothed_path[i]
            xy_yaw2 = smoothed_path[i + 1]
            ave_dist += MathUtils._2d_euclidian_distance(xy_yaw1, xy_yaw2)
        ave_dist /= len(smoothed_path)
        self.ave_smooth_path_distance_change = ave_dist

    def get_cost(self, x, y, yaw, debug=False):

        x_margin = project_constants.SEARCH_SPACE_X_MARGIN
        y_margin = project_constants.SEARCH_SPACE_Y_MARGIN
        fl_base, fr_base, br_base, bl_base = self.get_end_affector_xyz_coords_from_xy_yaw_deg_at_base_state([x,y,yaw],debug=debug, with_x_margin=x_margin, with_y_margin=y_margin)

        # out of search area
        if not fl_base:
            return np.inf

        fl_x, fl_y = fl_base[0], fl_base[1]
        fr_x, fr_y = fr_base[0], fr_base[1]
        bl_x, bl_y = bl_base[0], bl_base[1]
        br_x, br_y = br_base[0], br_base[1]

        r = .1

        # fl_cost = self.fs_cost_map.cost_at_xy(fl_x, fl_y)
        # fr_cost = self.fs_cost_map.cost_at_xy(fr_x, fr_y)
        # bl_cost = self.fs_cost_map.cost_at_xy(bl_x, bl_y)
        # br_cost = self.fs_cost_map.cost_at_xy(br_x, br_y)
        fl_cost = self.fs_cost_map.min_cost_in_circle_about_xy(fl_x, fl_y, r)
        fr_cost = self.fs_cost_map.min_cost_in_circle_about_xy(fr_x, fr_y, r)
        bl_cost = self.fs_cost_map.min_cost_in_circle_about_xy(bl_x, bl_y, r)
        br_cost = self.fs_cost_map.min_cost_in_circle_about_xy(br_x, br_y, r)
        cost = fl_cost + fr_cost + bl_cost + br_cost

        return cost

    def dist_between_xy_yaws(self, q1, q2, yaw_err_coeff=None):
        x_scaler = 1
        y_scaler = 1
        yaw_scaler = 1.0/25.0
        if yaw_err_coeff:
            yaw_scaler *= yaw_err_coeff
        return np.sqrt( x_scaler*(q1[0] - q2[0])**2 + y_scaler*(q1[1] - q2[1])**2 + yaw_scaler*(q1[2] - q2[2])**2 )

    def round(self,n):
        return np.round(n,decimals=self.decimal_round)