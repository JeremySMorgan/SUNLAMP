import random
import time
import numpy as np
from klampt import vis
import signal
from shapely.geometry import Point, LineString
from src.utils.data_objects.footstep_sequence import FootstepSequence
from src.utils.data_objects.height_map import HeightMap
from src.utils.data_objects.footstep_scatter import FootstepScatter
from src.utils.graph_search_utils.astar_fibheap import AStar
from src.utils.logger import Logger
from src.generators.generator_superclass import GeneratorSuperclass
from src.utils.math_utils import MathUtils
from src.utils.vis_utils import VisUtils
from src.utils.py_utils import PyUtils
from src.utils import project_constants

class StepSequencePlanner(GeneratorSuperclass, AStar):

    goal_state = (-1, -1, -1, -1, -1)
    OUT_OF_BOUND_COST = np.inf

    # : (Front Left, Front Right, Back Right, Back Left)
    # legs to move:
    #   1 - front left
    #   2 - front right
    #   3 - back right
    #   4 - back left

    # x in fs_scatter_obj: [x,y,z,cost] - 'Footstep Scatter Point'
    # stance := (x_i, x_j, x_k, x_l, cost), where x_i refers to fs_scatter_obj[i] - 'Stance Space Configuration'

    def __init__(self, height_map: HeightMap, scatter_obj: FootstepScatter, hl_traj_obj, fs_cost_map,
            xy_yaw0, xy_yawf,
            r_poser=None,
            vis_successors=False,
            deep_debug_visualization=False):

        self.r_poser = r_poser
        self.hl_traj = hl_traj_obj
        self.fs_cost_map = fs_cost_map
        self.height_map = height_map

        self.fs_scatter_obj = scatter_obj
        self.fs_scatter_list = scatter_obj.get_scatter_list()

        self.vis_successors= True if r_poser is not None and vis_successors else False
        self.detailed_visualization = deep_debug_visualization

        self.xy_yaw0 = xy_yaw0
        self.xy_yawf = xy_yawf
        self.total_xy_dist = MathUtils._2d_euclidian_distance(xy_yawf, xy_yaw0)

        # --- Save calculations
        self.q_ideal_fs_locations = {}

        # --- AStar Vars
        self.start_state = None
        self.state_path = None
        self.start_state = (len(self.fs_scatter_list) - 4, len(self.fs_scatter_list) - 3, len(self.fs_scatter_list) - 2,
                            len(self.fs_scatter_list) - 1, project_constants.STEP_ORDER[0])

        # High Level Trajectory
        self.high_density_xy_yaw_list = self.hl_traj.get_higher_density_xy_yaw_path()

        # Astar Search variables
        self.visited_state_nodes = {}

        self.g_weight = None
        self.h_weight = None
        self.save_search_weights()

        GeneratorSuperclass.__init__(self, height_map)
        AStar.__init__(self,state=self.start_state,testGoalOnGeneration=True)

        # debugging vars
        self.successor_calls = 0
        self.t_start = time.time()
        self.plotted_costcurve = None
        self.t_kinematically_valid_elapsed = 0
        self.kinematically_valid_calls = 0
        self.t_get_cost_elapsed = 0
        self.get_cost_calls = 0
        self.kinematically_valid_q_plotted = None
        self.support_tris_not_plotted = False
        self.search_start_t = -1
        self.print_stat_every_k = 25
        self.print_stat_i = 0

    def save_search_weights(self):
        self.g_weight = project_constants.STEPSEQ_G_WEIGHT
        self.h_weight = project_constants.STEPSEQ_H_WEIGHT
        if project_constants.STEPSEQ_VERBOSITY >= 3:
            print("g weight:",Logger.pp_double(self.g_weight),", h_weight:",Logger.pp_double(self.h_weight))

    def get_start_state(self):
        return self.start_state

    def get_state_path(self):
        return self.state_path

    def build_sequence(self, print_seq=False, suspend_after=None):

        fs_sequence = FootstepSequence()
        fs_sequence.xy_yaw0 = self.xy_yaw0
        fs_sequence.xy_yawf = self.xy_yawf

        if project_constants.STEPSEQ_VERBOSITY >= 2:
            print("Starting step sequence search")

        t_start = time.time()
        self.search_start_t = time.time()
        if suspend_after:

            def handler(signum, frame):
                raise Exception("end of time")

            res = self.search()
            # signal.signal(signal.SIGALRM, handler)
            # signal.alarm(suspend_after)
            # try:
            #     res = self.search()
            # except Exception as e:
            #     if project_constants.STEPSEQ_VERBOSITY >= 2:
            #         print(f"error: {e}")
            #         Logger.log(f"Failed to build a path after {round(suspend_after, 2)} seconds", "FAIL")
            #     return -1
        else:
            res = self.search()
        if suspend_after:
            signal.alarm(0)

        path = self.result_path()
        if res:
            if project_constants.STEPSEQ_VERBOSITY >= 2:
                print(f"Step sequence search finished in: {round(time.time() - t_start, 2)} seconds, with {len(path)} states")
            fs_sequence.failed = False
        else:
            if project_constants.STEPSEQ_VERBOSITY >= 1:
                Logger.log(f"Step sequence search failed after: {round(time.time() - t_start, 2)} seconds", "")
            fs_sequence.failed = True

        state_path = []
        for node in path:
            state_path.append(node.state)
            if print_seq: print(node.state,"cost:",node.f)
        state_path = state_path[:-1]
        self.state_path = state_path

        fs_sequence.state_path = self.state_path
        fs_sequence.runtime = time.time() - t_start

        return fs_sequence



    # -------- A* Functions
    def is_goal(self, state, debug=False):
        x_ave, y_ave, yaw_rads = self.get_estimated_torso_xy_yaw_rads(state)
        C_xy = 1.0
        # C_yaw = 1.0/5.0
        dif = C_xy*(np.abs(x_ave-self.xy_yawf[0]) + np.abs(y_ave-self.xy_yawf[1])) # + C_yaw*np.abs(np.rad2deg(yaw_rads)-self.xy_yawf[2])
        ret = dif < project_constants.STEPSEQ_GOAL_THRESHOLD
        if project_constants.STEPSEQ_VERBOSITY >= 3:
            if ret:
                print(Logger.styled_text(" Goal State Found","OKGREEN"), "\txy:",Logger.pp_list([x_ave,y_ave]))
        return ret

    def successors(self, state, klampt_vis_enabled=False):

        c_leg_to_move = state[4]
        next_leg_to_move = self.get_next_leg_to_move(c_leg_to_move)
        successors = []
        successor_costs = []

        if self.vis_successors:
            self.r_poser.set_q(state)

        t_start = time.time()

        fl_xyzc, fr_xyzc, br_xyzc, bl_xyzc = self.get_end_affectors_xyzc(state)
        if c_leg_to_move == 1:
            idx_range = self.fs_scatter_obj.get_idxs_near_xy(fl_xyzc[0], fl_xyzc[1], 1.25)
        elif c_leg_to_move == 2:
            idx_range = self.fs_scatter_obj.get_idxs_near_xy(fr_xyzc[0], fr_xyzc[1], 1.25)
        elif c_leg_to_move == 3:
            idx_range = self.fs_scatter_obj.get_idxs_near_xy(br_xyzc[0], br_xyzc[1], 1.25)
        elif c_leg_to_move == 4:
            idx_range = self.fs_scatter_obj.get_idxs_near_xy(bl_xyzc[0], bl_xyzc[1], 1.25)
        else:
            Logger.log("Error: leg to move is not identified, exiting", "FAIL", class_id=2, msg_type=1)
            return [], []

        for i in idx_range:
            if c_leg_to_move == 1:
                possible_state = (i, state[1], state[2], state[3], next_leg_to_move)
            elif c_leg_to_move == 2:
                possible_state = (state[0], i, state[2], state[3], next_leg_to_move)
            elif c_leg_to_move == 3:
                possible_state = (state[0], state[1], i, state[3], next_leg_to_move)
            else: # c_leg_to_move == 4:
                possible_state = (state[0], state[1], state[2], i, next_leg_to_move)

            if not possible_state[0:4] == state[0:4]:

                self.kinematically_valid_calls += 1
                kin_valid_stime = time.time()

                kin_valid = self.state_is_kinematically_valid(
                    state, possible_state, visualize=self.detailed_visualization, verbose_debug=False)
                self.t_kinematically_valid_elapsed += time.time() - kin_valid_stime

                if kin_valid:
                    successors.append(possible_state)
                    self.get_cost_calls += 1
                    get_cost_stime = time.time()
                    possible_state_cost = self.get_cost(state, possible_state)
                    self.t_get_cost_elapsed += time.time() - get_cost_stime
                    successor_costs.append(possible_state_cost)

        if project_constants.STEPSEQ_VERBOSITY >= 3:
            node = self.visited_state_node(state)
            torso_xy = self.get_estimated_torso_xy_yaw_rads(state)[0:2]
            dist_traveled = MathUtils._2d_euclidian_distance(torso_xy, self.xy_yaw0)
            pct_traveled = round(100*dist_traveled / self.total_xy_dist, 2)

            print(f"{PyUtils.format_time(time.time() - self.search_start_t)} | {pct_traveled}% done | "
                  f"{len(successors)} scrs | xy: {Logger.pp_list(torso_xy)} \t", Logger.log_boolean(self.is_goal(state), msg="isgoal"),
                  f" n.g: {round(node.g, 3)}\tn.h: {round(node.h, 3)}\tn.f: {round(node.f, 3)}")

        if project_constants.STEPSEQ_VERBOSITY >= 3:
            if self.print_stat_i == self.print_stat_every_k:
                self.print_stats(t_start)
                self.print_stat_i = 0
            self.print_stat_i += 1

        return successors, successor_costs

    def print_stats(self, t_start):
        t_elapsed = time.time() - t_start
        print("\nt elapsed:", round(t_elapsed,2), "(from last successors() call)"
              "\n  time/kinvalid call", round(self.t_kinematically_valid_elapsed / self.kinematically_valid_calls, 7),
              "\n  ttime/get cost call", round(self.t_get_cost_elapsed / self.get_cost_calls, 7),
              "\n  t_get_cost_elapsed", round(self.t_get_cost_elapsed, 2),
              "\n  t_kinematically_valid_elapsed", round(self.t_kinematically_valid_elapsed, 2),
              "\n  kinematically_valid_calls", self.kinematically_valid_calls,
              "\n  self.get_cost_calls", self.get_cost_calls
        )
        print()

    def clear_visited(self):
        self.visited_state_nodes.clear()

    def visit(self, state, node):
        self.visited_state_nodes[state] = node

    def visited_state_node(self, state):
        if state in self.visited_state_nodes:
            return self.visited_state_nodes[state]
        return None

    def heuristic(self, state):

        if self.is_goal(state):
            return 0

        dist_to_goal_scalar = 1.4
        trajectory_tangency_scalar = .0025
        torso_x, torso_y, yaw_rads = self.get_estimated_torso_xy_yaw_rads(state)
        yaw_deg = np.rad2deg(yaw_rads)
        best_fitting_xyyaw_along_smoothed_path_idx = self.get_best_fitting_smoothed_traj_idx(state, torso_x, torso_y, yaw_deg)
        est_trajectory_yaw_deg = self.high_density_xy_yaw_list[best_fitting_xyyaw_along_smoothed_path_idx][2]
        xy_dist_to_goal = self.get_xy_yaw_error_from_goal(state, ignore_yaw=True)
        # yaw_err = np.abs(yaw_deg - est_trajectory_yaw_deg)
        h = self.h_weight * (dist_to_goal_scalar*xy_dist_to_goal + trajectory_tangency_scalar*np.abs(yaw_deg - est_trajectory_yaw_deg))
        return h

    def get_xy_yaw_error_from_goal(self, q, ignore_yaw=False):
        if self.is_goal(q,debug=False):
            return 0
        x_ave, y_ave, yaw_rads = self.get_estimated_torso_xy_yaw_rads(q)
        position_scale_factor = 1
        rotation_scale_factor = .01
        error = position_scale_factor * np.sqrt((self.xy_yawf[0] - x_ave) ** 2 + (self.xy_yawf[1] - y_ave) ** 2)
        if not ignore_yaw:
            error += rotation_scale_factor * np.abs(np.rad2deg(yaw_rads) - self.xy_yawf[2])
        # self.xy_yaw_error_from_goal_dict[stance] = error
        return error

    def get_estimated_torso_xy_yaw_rads(self, q):
        if q[0] == -1:
            return self.xy_yawf[0],self.xy_yawf[1],self.xy_yawf[2]
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

    def get_next_leg_to_move(self, c_leg):

        idx = project_constants.STEP_ORDER.index(c_leg)
        if idx == len(project_constants.STEP_ORDER) - 1:
            return project_constants.STEP_ORDER[0]
        return project_constants.STEP_ORDER[project_constants.STEP_ORDER.index(c_leg) + 1]

    def get_end_affectors_xyzc(self, q):
        fl_xyzc = self.fs_scatter_list[int(q[0])]
        fr_xyzc = self.fs_scatter_list[int(q[1])]
        br_xyzc = self.fs_scatter_list[int(q[2])]
        bl_xyzc = self.fs_scatter_list[int(q[3])]
        return fl_xyzc, fr_xyzc, br_xyzc, bl_xyzc

    def state_is_kinematically_valid(self, q_curr, q_next, visualize=False, verbose_debug=False):

        '''
            - returns wether q_state is kinematically valid.
            tests:
                parrallel distance: The moving leg and its diagnol leg must be within STEPSEQ_MAX_DIAGNOL_DIST distance
                ...

        :param q_curr: current state
        :param q_next:  next state
        :return:  Bool
        '''

        fl_xyzc, fr_xyzc, br_xyzc, bl_xyzc = self.get_end_affectors_xyzc(q_curr)
        fl_xyz_next, fr_xyz_next, br_xyz_next, bl_xyz_next = self.get_end_affectors_xyzc(q_next)

        moving_leg = q_curr[4]
        next_moving_leg = q_next[4]
        internal_vis_enabled = False
        return_false = False

        ### ____________ reject obviously invalid states
        r = project_constants.STEPSEQ_MAX_FEASIBLE_LEG_TRANSLATION
        if visualize:
            if not q_curr == self.kinematically_valid_q_plotted:
                internal_vis_enabled = True
                self.support_tris_not_plotted = True
                self.kinematically_valid_q_plotted = q_curr

        if moving_leg == 1:
            if visualize and internal_vis_enabled:
                VisUtils.visualize_circle(fl_xyzc[0], fl_xyzc[1], r, name="obviousrejection circle")

            if MathUtils._3d_euclidian_distance(fl_xyzc, fl_xyz_next) > r:
                return_false = True
                if not internal_vis_enabled:
                    return False

        elif moving_leg == 2:
            if visualize and internal_vis_enabled:
                VisUtils.visualize_circle(fr_xyzc[0], fr_xyzc[1], r, name="obviousrejection circle")
            if MathUtils._3d_euclidian_distance(fr_xyzc, fr_xyz_next) > r:
                return_false = True
                if not internal_vis_enabled:
                    return False

        elif moving_leg == 3:
            if visualize and internal_vis_enabled:
                VisUtils.visualize_circle(br_xyzc[0], br_xyzc[1], r, name="obviousrejection circle")

            if MathUtils._3d_euclidian_distance(br_xyzc, br_xyz_next) > r:
                return_false = True
                if not internal_vis_enabled:
                    return False
        elif moving_leg == 4:
            if visualize and internal_vis_enabled:
                VisUtils.visualize_circle(bl_xyzc[0], bl_xyzc[1], r, name="obviousrejection circle")
            if MathUtils._3d_euclidian_distance(bl_xyzc, bl_xyz_next) > r:
                return_false = True
                if not internal_vis_enabled:
                    return False
        else:
            Logger.log(("Unrecognized leg to move:" + str(moving_leg)), "FAIL")

        ### ____________ Calculate diaganol distance
        diaganol_dist = 0
        if moving_leg == 1:
            diaganol_dist = MathUtils._3d_euclidian_distance(fl_xyz_next, br_xyzc)
            if visualize and internal_vis_enabled:
                VisUtils.visualize_circle(br_xyzc[0], br_xyzc[1], project_constants.STEPSEQ_MAX_DIAGNOL_DIST, name="max diaganol distance circle")
        elif moving_leg == 2:
            diaganol_dist = MathUtils._3d_euclidian_distance(fr_xyz_next, bl_xyzc)
            if visualize and internal_vis_enabled:
                VisUtils.visualize_circle(bl_xyzc[0], bl_xyzc[1], project_constants.STEPSEQ_MAX_DIAGNOL_DIST, name="max diaganol distance circle")
        elif moving_leg == 3:
            diaganol_dist = MathUtils._3d_euclidian_distance(br_xyz_next, fl_xyzc)
            if visualize and internal_vis_enabled:
                VisUtils.visualize_circle(fl_xyzc[0], fl_xyzc[1], project_constants.STEPSEQ_MAX_DIAGNOL_DIST, name="max diaganol distance circle")
        elif moving_leg == 4:
            diaganol_dist = MathUtils._3d_euclidian_distance(bl_xyz_next, fr_xyzc)
            if visualize and internal_vis_enabled:
                VisUtils.visualize_circle(fr_xyzc[0], fr_xyzc[1], project_constants.STEPSEQ_MAX_DIAGNOL_DIST, name="max diaganol distance circle")
        else:
            Logger.log(("Unrecognized leg to move:" + str(moving_leg)), "FAIL")

        if diaganol_dist > project_constants.STEPSEQ_MAX_DIAGNOL_DIST:
            return_false = True
            if not internal_vis_enabled:
                return False

        if return_false:
            return False

        ### ____________ Must be in correct region

        X_wingspan = 2 * project_constants.BASE_STATE_END_EFF_DX_FROM_TORSO
        Y_wingspan = 2 * project_constants.BASE_STATE_END_EFF_DY_FROM_TORSO

        if moving_leg == 1:

            # diag constraint
            if MathUtils.points_on_same_side_of_line(bl_xyzc, fr_xyzc, br_xyzc, fl_xyz_next):
                return False

            # right side line constraint
            if not MathUtils.points_on_same_side_of_line(br_xyzc, fr_xyzc, bl_xyzc, fl_xyz_next):
                return False

            # back legs line constraint
            if not MathUtils.points_on_same_side_of_line(bl_xyzc, br_xyzc, fr_xyzc, fl_xyz_next):
                return False

            fl_nextP = Point(fl_xyz_next[0], fl_xyz_next[1])

            # if fl next is in circle of radius STEPSEQ_KINEMATIC_INFEASIBILITY_CIRC_RADIUS_CENTERED_AT_ADJACENT_ENDEFFECTORS
            #  centered at bl_next -> infeasible

            r = project_constants.STEPSEQ_KINEMATIC_INFEASIBILITY_CIRC_RADIUS_CENTERED_AT_ADJACENT_ENDEFFECTORS

            if Point(bl_xyzc[0], bl_xyzc[1]).buffer(r).contains(fl_nextP):
                return False

            if Point(fr_xyzc[0], fr_xyzc[1]).buffer(
                    project_constants.STEPSEQ_KINEMATIC_INFEASIBILITY_CIRC_RADIUS_CENTERED_AT_ADJACENT_ENDEFFECTORS).contains(
                    fl_nextP):
                return False

            diag_midpointX, diag_midpointY = (bl_xyzc[0] + fr_xyzc[0]) / 2.0, (bl_xyzc[1] + fr_xyzc[1]) / 2.0
            if Point(diag_midpointX, diag_midpointY).buffer(
                    project_constants.STEPSEQ_KINEMATIC_INFEASIBILITY_CIRC_RADIUS_CENTERED_AT_COMTRI_DIAG_MIDPOINT).contains(
                    fl_nextP):
                return False

            theta_rightside_circ = np.arctan2(fr_xyzc[1] - br_xyzc[1], fr_xyzc[0] - br_xyzc[0])
            x_rside_circ_local, y_rside_circ_local = MathUtils._2d_rotation_transformation(X_wingspan, Y_wingspan, theta_rightside_circ)
            rside_circX = x_rside_circ_local + br_xyzc[0]
            rside_circY = y_rside_circ_local + br_xyzc[1]
            right_side_circ = Point(rside_circX, rside_circY).buffer(project_constants.STEPSEQ_KINEMATIC_FEASIBILITY_CIRC_RADIUS)

            theta_backside_circ = np.arctan2(bl_xyzc[1] - br_xyzc[1], bl_xyzc[0] - br_xyzc[0]) - np.pi / 2.0
            x_backside_circ_local, y_backside_circ_local = MathUtils._2d_rotation_transformation(X_wingspan, Y_wingspan, theta_backside_circ)
            backside_circX = x_backside_circ_local + br_xyzc[0]
            backside_circY = y_backside_circ_local + br_xyzc[1]
            backside_circ = Point(backside_circX, backside_circY).buffer(project_constants.STEPSEQ_KINEMATIC_FEASIBILITY_CIRC_RADIUS)

            if not (right_side_circ.contains(fl_nextP) or backside_circ.contains(fl_nextP)):
                return False

        elif moving_leg == 2:

            # diag constraint
            if MathUtils.points_on_same_side_of_line(br_xyzc, fl_xyzc, bl_xyzc, fr_xyz_next):
                return False

            #  left side constraint
            if not MathUtils.points_on_same_side_of_line(bl_xyzc, fl_xyzc, br_xyzc, fr_xyz_next):
                return False

            # back legs line constraint
            if not MathUtils.points_on_same_side_of_line(bl_xyzc, br_xyzc, fl_xyzc, fr_xyz_next):
                return False

            fr_nextP = Point(fr_xyz_next[0], fr_xyz_next[1])

            if Point(fl_xyzc[0], fl_xyzc[1]).buffer(
                    project_constants.STEPSEQ_KINEMATIC_INFEASIBILITY_CIRC_RADIUS_CENTERED_AT_ADJACENT_ENDEFFECTORS).contains(
                    fr_nextP):
                return False

            if Point(br_xyzc[0], br_xyzc[1]).buffer(
                    project_constants.STEPSEQ_KINEMATIC_INFEASIBILITY_CIRC_RADIUS_CENTERED_AT_ADJACENT_ENDEFFECTORS).contains(
                    fr_nextP):
                return False

            diag_midpointX, diag_midpointY = (fl_xyzc[0] + br_xyzc[0]) / 2.0, (fl_xyzc[1] + br_xyzc[1]) / 2.0
            if Point(diag_midpointX, diag_midpointY).buffer(
                    project_constants.STEPSEQ_KINEMATIC_INFEASIBILITY_CIRC_RADIUS_CENTERED_AT_COMTRI_DIAG_MIDPOINT).contains(fr_nextP):
                return False

            theta_leftside_circ = np.arctan2(fl_xyzc[1] - bl_xyzc[1], fl_xyzc[0] - bl_xyzc[0])
            x_lside_circ_local, y_lside_circ_local = MathUtils._2d_rotation_transformation(X_wingspan, -Y_wingspan, theta_leftside_circ)
            lside_circX = x_lside_circ_local + bl_xyzc[0]
            lside_circY = y_lside_circ_local + bl_xyzc[1]
            lside_circ = Point(lside_circX, lside_circY).buffer(project_constants.STEPSEQ_KINEMATIC_FEASIBILITY_CIRC_RADIUS)

            theta_backside_circ = np.arctan2(bl_xyzc[1] - br_xyzc[1], bl_xyzc[0] - br_xyzc[0]) - (np.pi) / 2.0
            x_backside_circ_local, y_backside_circ_local = MathUtils._2d_rotation_transformation(X_wingspan, 0.0, theta_backside_circ)
            backside_circX = x_backside_circ_local + br_xyzc[0]
            backside_circY = y_backside_circ_local + br_xyzc[1]
            backside_circ = Point(backside_circX, backside_circY).buffer(project_constants.STEPSEQ_KINEMATIC_FEASIBILITY_CIRC_RADIUS)

            if not (lside_circ.contains(fr_nextP) or backside_circ.contains(fr_nextP)):
                return False

        elif moving_leg == 3:

            # Put computationally less expensive operations before circle intersection testing
            # diag constraint
            if MathUtils.points_on_same_side_of_line(bl_xyzc, fr_xyzc, fl_xyzc, br_xyz_next):
                return False

            # left left side constraint
            if not MathUtils.points_on_same_side_of_line(bl_xyzc, fl_xyzc, fr_xyzc, br_xyz_next):
                return False

            # front legs line constraint
            if not MathUtils.points_on_same_side_of_line(fl_xyzc, fr_xyzc, bl_xyzc, br_xyz_next):
                return False

            br_nextP = Point(br_xyz_next[0], br_xyz_next[1])

            if Point(fr_xyzc[0], fr_xyzc[1]).buffer(
                    project_constants.STEPSEQ_KINEMATIC_INFEASIBILITY_CIRC_RADIUS_CENTERED_AT_ADJACENT_ENDEFFECTORS).contains(br_nextP):
                return False

            if Point(bl_xyzc[0], bl_xyzc[1]).buffer(
                    project_constants.STEPSEQ_KINEMATIC_INFEASIBILITY_CIRC_RADIUS_CENTERED_AT_ADJACENT_ENDEFFECTORS).contains(br_nextP):
                return False

            diag_midpointX, diag_midpointY = (bl_xyzc[0] + fr_xyzc[0]) / 2.0, (bl_xyzc[1] + fr_xyzc[1]) / 2.0
            if Point(diag_midpointX, diag_midpointY).buffer(
                    project_constants.STEPSEQ_KINEMATIC_INFEASIBILITY_CIRC_RADIUS_CENTERED_AT_COMTRI_DIAG_MIDPOINT).contains(br_nextP):
                return False

            theta_leftside_circ = np.arctan2(fl_xyzc[1] - bl_xyzc[1], fl_xyzc[0] - bl_xyzc[0])
            x_lside_circ_local, y_lside_circ_local = MathUtils._2d_rotation_transformation(0.0, -Y_wingspan, theta_leftside_circ)
            lside_circX = x_lside_circ_local + bl_xyzc[0]
            lside_circY = y_lside_circ_local + bl_xyzc[1]
            lside_circ = Point(lside_circX, lside_circY).buffer(project_constants.STEPSEQ_KINEMATIC_FEASIBILITY_CIRC_RADIUS)

            theta_frontside_circ = np.arctan2(fl_xyzc[1] - fr_xyzc[1], fl_xyzc[0] - fr_xyzc[0]) - (np.pi) / 2.0
            x_frontside_circ_local, y_backside_circ_local = MathUtils._2d_rotation_transformation(-X_wingspan, 0.0,
                                                                                                  theta_frontside_circ)
            frontside_circX = x_frontside_circ_local + fr_xyzc[0]
            frontside_circY = y_backside_circ_local + fr_xyzc[1]
            frontside_circ = Point(frontside_circX, frontside_circY).buffer(
                project_constants.STEPSEQ_KINEMATIC_FEASIBILITY_CIRC_RADIUS)

            if not (lside_circ.contains(br_nextP) or frontside_circ.contains(br_nextP)):
                return False

        elif moving_leg == 4:

            # diag constraint
            if MathUtils.points_on_same_side_of_line(br_xyzc, fl_xyzc, fr_xyzc, bl_xyz_next):
                return False

            # right side constraint
            if not MathUtils.points_on_same_side_of_line(br_xyzc, fr_xyzc, fl_xyzc, bl_xyz_next):
                return False

            # front legs line constraint
            if not MathUtils.points_on_same_side_of_line(fl_xyzc, fr_xyzc, br_xyzc, bl_xyz_next):
                return False

            bl_nextP = Point(bl_xyz_next[0], bl_xyz_next[1])

            if Point(fl_xyzc[0], fl_xyzc[1]).buffer(
                    project_constants.STEPSEQ_KINEMATIC_INFEASIBILITY_CIRC_RADIUS_CENTERED_AT_ADJACENT_ENDEFFECTORS).contains(bl_nextP):
                return False

            if Point(br_xyzc[0], br_xyzc[1]).buffer(
                    project_constants.STEPSEQ_KINEMATIC_INFEASIBILITY_CIRC_RADIUS_CENTERED_AT_ADJACENT_ENDEFFECTORS).contains(
                    bl_nextP):
                return False

            diag_midpointX, diag_midpointY = (fl_xyzc[0] + br_xyzc[0]) / 2.0, (fl_xyzc[1] + br_xyzc[1]) / 2.0
            if Point(diag_midpointX, diag_midpointY).buffer(
                    project_constants.STEPSEQ_KINEMATIC_INFEASIBILITY_CIRC_RADIUS_CENTERED_AT_COMTRI_DIAG_MIDPOINT).contains(
                    bl_nextP):
                return False

            theta_rightside_circ = np.arctan2(fr_xyzc[1] - br_xyzc[1], fr_xyzc[0] - br_xyzc[0])
            x_rside_circ_local, y_rside_circ_local = MathUtils._2d_rotation_transformation(
                0, Y_wingspan, theta_rightside_circ)
            rside_circX = x_rside_circ_local + br_xyzc[0]
            rside_circY = y_rside_circ_local + br_xyzc[1]
            right_side_circ = Point(rside_circX, rside_circY).buffer(
                project_constants.STEPSEQ_KINEMATIC_FEASIBILITY_CIRC_RADIUS)

            theta_frontside_circ = np.arctan2(fl_xyzc[1] - fr_xyzc[1], fl_xyzc[0] - fr_xyzc[0]) - (np.pi) / 2.0
            x_frontside_circ_local, y_backside_circ_local = MathUtils._2d_rotation_transformation(
                -X_wingspan, Y_wingspan, theta_frontside_circ)
            frontside_circX = x_frontside_circ_local + fr_xyzc[0]
            frontside_circY = y_backside_circ_local + fr_xyzc[1]
            frontside_circ = Point(frontside_circX, frontside_circY).buffer(
                project_constants.STEPSEQ_KINEMATIC_FEASIBILITY_CIRC_RADIUS)

            if not (right_side_circ.contains(bl_nextP) or frontside_circ.contains(bl_nextP)):
                return False

        else:
            Logger.log(("Unrecognized leg to move:" + str(moving_leg)), "FAIL")

        # ### ---------------- Calculate q_next moving leg to q_next support region distance, measured along
        #                       (qnext moving_leg_xyz -> qnext support region triangle incenter)

        # ### ---------------- Calculate stance moving leg to q_next support region distance, measured along
        #                       (qnext moving_leg_xyz -> qnext support region triangle incenter)

        #   d_1: distance from stance's moving leg at q_next (called ml_qnext), to the intersection between the line
        #        between defined the moving leg's ajacent legs, where the intersection is along (ml_qnext -> q_support_tri_incenter)
        #        where q_support_tri_incenter is the incenter circle center of the support triangle for stance
        #
        q_support_tri = [fl_xyzc, fr_xyzc, br_xyzc, bl_xyzc]

        if moving_leg == 1:
            q_support_tri.remove(fl_xyzc)
        elif moving_leg == 2:
            q_support_tri.remove(fr_xyzc)
        elif moving_leg == 3:
            q_support_tri.remove(br_xyzc)
        elif moving_leg == 4:
            q_support_tri.remove(bl_xyzc)
        else:
            Logger.log(("Unrecognized leg to move:" + str(moving_leg)), "FAIL")

        q_incenter_circX, q_incenter_circY, q_incenter_circR = MathUtils.incenter_circle_xy_R_fromT(q_support_tri)

        d_1_2d = -1
        d_1_3d = -1

        if visualize and self.support_tris_not_plotted:
            self.support_tris_not_plotted = False
            VisUtils.visualize_xyz_list(q_support_tri, height_map=self.height_map, loop=True, name="stance curr support tri", color=VisUtils.WHITE)
            VisUtils.visualize_circle(q_incenter_circX, q_incenter_circY, q_incenter_circR, hm=self.height_map, name="qcurr incenter", color=VisUtils.WHITE)

        if moving_leg == 1:

            # calculate d1
            support_tri_diag_line = LineString([bl_xyzc[0:2], fr_xyzc[0:2]])
            moving_leg_to_incenter_line = LineString([fl_xyz_next[0:2], [q_incenter_circX, q_incenter_circY]])
            intersection_pt = support_tri_diag_line.intersection(moving_leg_to_incenter_line)
            if not intersection_pt.is_empty:
                intersection_pt_xyz = [intersection_pt.x,intersection_pt.y,
                                       self.height_map.height_at_xy(intersection_pt.x,intersection_pt.y)]
                d_1_2d = MathUtils._2d_euclidian_distance(fl_xyz_next, [intersection_pt.x,intersection_pt.y])
                d_1_3d = MathUtils._3d_euclidian_distance(fl_xyz_next, intersection_pt_xyz)
                #
                # if verbose_debug:
                #     print("1 ml1: lines intersect")
                if visualize:
                    VisUtils.visualize_line(fl_xyz_next, [intersection_pt.x,intersection_pt.y], name="d_1", add_z=.005)
            else:
                if verbose_debug:
                    print("1 ml1: lines DONT intersect")
                return False

        elif moving_leg == 2:
            support_tri_diag_line = LineString([fl_xyzc[0:2], br_xyzc[0:2]])
            moving_leg_to_incenter_line = LineString([fr_xyz_next[0:2], [q_incenter_circX, q_incenter_circY]])
            intersection_pt = support_tri_diag_line.intersection(moving_leg_to_incenter_line)
            if not intersection_pt.is_empty:
                intersection_pt_xyz = [intersection_pt.x,intersection_pt.y,
                                       self.height_map.height_at_xy(intersection_pt.x,intersection_pt.y)]
                d_1_2d = MathUtils._2d_euclidian_distance(fr_xyz_next, [intersection_pt.x,intersection_pt.y])
                d_1_3d = MathUtils._3d_euclidian_distance(fr_xyz_next, intersection_pt_xyz)
                # if verbose_debug:
                #     print("stance ml2: lines intersect")
                if visualize:
                    VisUtils.visualize_line(fr_xyz_next, [intersection_pt.x,intersection_pt.y], name="daig_com_dist_c", add_z=.005)
            else:
                if verbose_debug:
                    print("stance ml2: lines DONT intersect")
                return False

        elif moving_leg == 3:
            support_tri_diag_line = LineString([bl_xyzc[0:2], fr_xyzc[0:2]])
            moving_leg_to_incenter_line = LineString([br_xyz_next[0:2], [q_incenter_circX, q_incenter_circY]])
            intersection_pt = support_tri_diag_line.intersection(moving_leg_to_incenter_line)
            if not intersection_pt.is_empty:
                intersection_ptxy = [intersection_pt.x,intersection_pt.y]
                intersection_pt_xyz = [intersection_pt.x,intersection_pt.y,
                                       self.height_map.height_at_xy(intersection_pt.x,intersection_pt.y)]
                d_1_2d = MathUtils._2d_euclidian_distance(br_xyz_next, intersection_ptxy)
                d_1_3d = MathUtils._3d_euclidian_distance(br_xyz_next, intersection_pt_xyz)
                # if verbose_debug:
                #     print("stance ml3: lines intersect")

                if visualize:
                    VisUtils.visualize_line(br_xyz_next, intersection_ptxy, name="daig_com_dist_c", add_z=.005)
            else:
                if verbose_debug:
                    print("stance ml3: lines DONT intersect for nextmoving leg:3")
                return False

        elif moving_leg == 4:
            support_tri_diag_line = LineString([fl_xyzc[0:2], br_xyzc[0:2]])
            moving_leg_to_incenter_line = LineString([bl_xyz_next[0:2], [q_incenter_circX, q_incenter_circY]])
            intersection_pt = support_tri_diag_line.intersection(moving_leg_to_incenter_line)
            if not intersection_pt.is_empty:
                intersection_ptxy = [intersection_pt.x, intersection_pt.y]
                intersection_pt_xyz = [intersection_pt.x,intersection_pt.y,
                                       self.height_map.height_at_xy(intersection_pt.x,intersection_pt.y)]
                d_1_3d = MathUtils._3d_euclidian_distance(bl_xyz_next, intersection_pt_xyz)
                d_1_2d = MathUtils._2d_euclidian_distance(bl_xyz_next, intersection_ptxy)

                # if verbose_debug:
                #     print("stance ml1: intersect")

                if visualize:
                    VisUtils.visualize_line(bl_xyz_next, intersection_ptxy, name="daig_com_dist_c", add_z=.005)
            else:
                if verbose_debug:
                    print("stance ml4: lines DONT intersect - returning FALSE")
                # note: the false return is new here
                return False

        if project_constants.STEPSEQ_USE_Z_IN_MAX_DIST_CALCS:
            if d_1_3d > project_constants.STEPSEQ_D12_3D_MULTIPLIER * project_constants.STEPSEQ_MAX_FOOTHOLD_TO_COM_ALONG_FOOTHOLD_INCENTER_LINE_DIST:
                if verbose_debug:
                    print("d_1_3d:", d_1_3d, "Returning FALSE")
                return False

        if d_1_2d > project_constants.STEPSEQ_MAX_FOOTHOLD_TO_COM_ALONG_FOOTHOLD_INCENTER_LINE_DIST:
            if verbose_debug:
                print("d_1_3d:", d_1_3d, "Returning FALSE")
            return False

        # if verbose_debug:
        #     print("stance d_1_2d:", d_1_2d)

        # ----------------------------------------------------------
        # Calculate d2
        #
        q_next_support_tri = [fl_xyz_next, fr_xyz_next, br_xyz_next, bl_xyz_next]
        if next_moving_leg == 1:
            q_next_support_tri.remove(fl_xyz_next)
        elif next_moving_leg == 2:
            q_next_support_tri.remove(fr_xyz_next)
        elif next_moving_leg == 3:
            q_next_support_tri.remove(br_xyz_next)
        elif next_moving_leg == 4:
            q_next_support_tri.remove(bl_xyz_next)
        else:
            Logger.log(("Unrecognized leg to move:" + str(moving_leg)), "FAIL")

        qnext_incenter_circX, qnext_incenter_circY, qnext_incenter_circR = MathUtils.incenter_circle_xy_R_fromT(q_next_support_tri)

        d_2 = -1
        d_2_3d = -1
        d_2_3d_vis_point = None

        if visualize:
            try:
                VisUtils.visualize_xyz_list(q_next_support_tri, height_map=self.height_map, loop=True, name="Q_next support tri", color=VisUtils.GREEN)
                VisUtils.visualize_circle(qnext_incenter_circX, qnext_incenter_circY, qnext_incenter_circR, hm=self.height_map, name="qnext support region incenter")
            except ValueError:
                print(
                    f"Value error in step_sequence_planner.state_is_kinematically_valid(), qnext_incenter_circX:"
                    f"{qnext_incenter_circX}, qnext_incenter_circY:{qnext_incenter_circY}, q_next_support_tri: {q_next_support_tri}")

        try:
            if moving_leg == 1:

                # calculate d2
                if next_moving_leg == 2:
                    q_next_moving_end_eff_to_incenter_line = LineString([fr_xyzc[0:2], [qnext_incenter_circX, qnext_incenter_circY]])
                    q_next_support_tri_diag_line = LineString([fl_xyz_next[0:2], br_xyzc[0:2]])
                    q_next_intersection_pt = q_next_moving_end_eff_to_incenter_line.intersection(q_next_support_tri_diag_line)
                    q_next_intersection_pt_xy = [q_next_intersection_pt.x, q_next_intersection_pt.y]
                    q_next_intersection_pt_xyz = [q_next_intersection_pt.x, q_next_intersection_pt.y,
                                                  self.height_map.height_at_xy(q_next_intersection_pt.x, q_next_intersection_pt.y)]
                    d_2 = MathUtils._2d_euclidian_distance(fr_xyzc, q_next_intersection_pt_xy)
                    d_2_3d = MathUtils._3d_euclidian_distance(fr_xyzc, q_next_intersection_pt_xyz)

                    if visualize:
                        VisUtils.visualize_line(fr_xyzc, q_next_intersection_pt_xy, name="next moving", add_z=.005)
                        d_2_3d_vis_point = fr_xyzc

                elif next_moving_leg == 3:
                    q_next_moving_end_eff_to_incenter_line = LineString([br_xyzc[0:2], [qnext_incenter_circX, qnext_incenter_circY]])
                    q_next_support_tri_diag_line = LineString([bl_xyz_next[0:2], fr_xyzc[0:2]])
                    q_next_intersection_pt = q_next_moving_end_eff_to_incenter_line.intersection(q_next_support_tri_diag_line)
                    q_next_intersection_pt_xy =  [q_next_intersection_pt.x,q_next_intersection_pt.y]
                    q_next_intersection_pt_xyz = [q_next_intersection_pt.x, q_next_intersection_pt.y,
                                                  self.height_map.height_at_xy(q_next_intersection_pt.x, q_next_intersection_pt.y)]
                    d_2 = MathUtils._2d_euclidian_distance(br_xyz_next, q_next_intersection_pt_xy)
                    d_2_3d = MathUtils._3d_euclidian_distance(br_xyz_next, q_next_intersection_pt_xyz)
                    if visualize:
                        VisUtils.visualize_line(br_xyz_next, q_next_intersection_pt_xy, name="next moving", add_z=.005)
                        d_2_3d_vis_point = br_xyz_next

                elif next_moving_leg == 4:
                    q_next_moving_end_eff_to_incenter_line = LineString([bl_xyzc[0:2], [qnext_incenter_circX, qnext_incenter_circY]])
                    q_next_support_tri_diag_line = LineString([fl_xyz_next[0:2], br_xyzc[0:2]])
                    q_next_intersection_pt = q_next_moving_end_eff_to_incenter_line.intersection(q_next_support_tri_diag_line)
                    q_next_intersection_pt_xy =  [q_next_intersection_pt.x,q_next_intersection_pt.y]
                    q_next_intersection_pt_xyz = [q_next_intersection_pt.x, q_next_intersection_pt.y,
                                                  self.height_map.height_at_xy(q_next_intersection_pt.x, q_next_intersection_pt.y)]
                    d_2 = MathUtils._2d_euclidian_distance(bl_xyzc, q_next_intersection_pt_xy)
                    d_2_3d = MathUtils._3d_euclidian_distance(bl_xyzc, q_next_intersection_pt_xyz)
                    if visualize:
                        VisUtils.visualize_line(bl_xyzc, q_next_intersection_pt_xy, name="next moving", add_z=.005)
                        VisUtils.visualize_line(bl_xyzc, q_next_intersection_pt_xyz, name="d_2_3d", add_z=.005)
                else:
                    raise RuntimeError("invalid leg number")

            elif moving_leg == 2:

                # calculate d2
                if next_moving_leg == 1:
                    q_next_moving_end_eff_to_incenter_line = LineString([fl_xyzc[0:2], [qnext_incenter_circX, qnext_incenter_circY]])
                    q_next_support_tri_diag_line = LineString([bl_xyz_next[0:2], br_xyz_next[0:2]])
                    q_next_intersection_pt = q_next_moving_end_eff_to_incenter_line.intersection( q_next_support_tri_diag_line)
                    q_next_intersection_pt_xy = [q_next_intersection_pt.x, q_next_intersection_pt.y]
                    q_next_intersection_pt_xyz = [q_next_intersection_pt.x, q_next_intersection_pt.y,
                                                  self.height_map.height_at_xy(q_next_intersection_pt.x, q_next_intersection_pt.y)]
                    d_2 = MathUtils._2d_euclidian_distance(fl_xyzc, q_next_intersection_pt_xy)
                    d_2_3d = MathUtils._3d_euclidian_distance(fl_xyzc, q_next_intersection_pt_xyz)

                elif next_moving_leg == 3:
                    q_next_moving_end_eff_to_incenter_line = LineString( [br_xyzc[0:2], [qnext_incenter_circX, qnext_incenter_circY]])
                    q_next_support_tri_diag_line = LineString([bl_xyzc[0:2], fr_xyz_next[0:2]])
                    q_next_intersection_pt = q_next_moving_end_eff_to_incenter_line.intersection(q_next_support_tri_diag_line)
                    q_next_intersection_pt_xy = [q_next_intersection_pt.x, q_next_intersection_pt.y]
                    q_next_intersection_pt_xyz = [q_next_intersection_pt.x, q_next_intersection_pt.y,
                                                  self.height_map.height_at_xy(q_next_intersection_pt.x, q_next_intersection_pt.y)]
                    d_2 = MathUtils._2d_euclidian_distance(br_xyzc, q_next_intersection_pt_xy)
                    d_2_3d = MathUtils._3d_euclidian_distance(br_xyzc, q_next_intersection_pt_xyz)

                elif next_moving_leg == 4:
                    q_next_moving_end_eff_to_incenter_line = LineString( [ bl_xyzc[0:2], [qnext_incenter_circX, qnext_incenter_circY] ])
                    q_next_support_tri_diag_line = LineString( [fl_xyzc[0:2], br_xyzc[0:2]])
                    q_next_intersection_pt = q_next_moving_end_eff_to_incenter_line.intersection(q_next_support_tri_diag_line)
                    q_next_intersection_pt_xy = [q_next_intersection_pt.x, q_next_intersection_pt.y]
                    q_next_intersection_pt_xyz = [q_next_intersection_pt.x, q_next_intersection_pt.y,
                                                  self.height_map.height_at_xy(q_next_intersection_pt.x, q_next_intersection_pt.y)]
                    d_2 = MathUtils._2d_euclidian_distance(bl_xyzc, q_next_intersection_pt_xy)
                    d_2_3d = MathUtils._3d_euclidian_distance(bl_xyzc, q_next_intersection_pt_xyz)
                else:
                    raise RuntimeError("invalid leg number")

            elif moving_leg == 3:

                # calculate d2
                if next_moving_leg == 1:
                    q_next_moving_end_eff_to_incenter_line = LineString([fl_xyzc[0:2], [qnext_incenter_circX, qnext_incenter_circY]])
                    q_next_support_tri_diag_line = LineString([bl_xyzc[0:2], fr_xyzc[0:2]])
                    q_next_intersection_pt = q_next_moving_end_eff_to_incenter_line.intersection( q_next_support_tri_diag_line)
                    q_next_intersection_pt_xy = [q_next_intersection_pt.x, q_next_intersection_pt.y]
                    q_next_intersection_pt_xyz = [q_next_intersection_pt.x, q_next_intersection_pt.y,
                                                  self.height_map.height_at_xy(q_next_intersection_pt.x, q_next_intersection_pt.y)]
                    d_2 = MathUtils._2d_euclidian_distance(fl_xyzc, q_next_intersection_pt_xy)
                    d_2_3d = MathUtils._3d_euclidian_distance(fl_xyzc, q_next_intersection_pt_xyz)

                elif next_moving_leg == 2:
                    q_next_moving_end_eff_to_incenter_line = LineString([fr_xyzc[0:2], [qnext_incenter_circX, qnext_incenter_circY]])
                    q_next_support_tri_diag_line = LineString([fl_xyzc[0:2], br_xyz_next[0:2]])
                    q_next_intersection_pt = q_next_moving_end_eff_to_incenter_line.intersection(q_next_support_tri_diag_line)
                    q_next_intersection_pt_xy = [q_next_intersection_pt.x, q_next_intersection_pt.y]
                    q_next_intersection_pt_xyz = [q_next_intersection_pt.x, q_next_intersection_pt.y,
                                                  self.height_map.height_at_xy(q_next_intersection_pt.x, q_next_intersection_pt.y)]
                    d_2 = MathUtils._2d_euclidian_distance(fr_xyzc, q_next_intersection_pt_xy)
                    d_2_3d = MathUtils._3d_euclidian_distance(fr_xyzc, q_next_intersection_pt_xyz)

                elif next_moving_leg == 4:
                    q_next_moving_end_eff_to_incenter_line = LineString( [bl_xyzc[0:2], [qnext_incenter_circX, qnext_incenter_circY]])
                    q_next_support_tri_diag_line = LineString([fl_xyzc[0:2], br_xyz_next[0:2]])
                    q_next_intersection_pt = q_next_moving_end_eff_to_incenter_line.intersection(q_next_support_tri_diag_line)
                    q_next_intersection_pt_xy = [q_next_intersection_pt.x, q_next_intersection_pt.y]
                    q_next_intersection_pt_xyz = [q_next_intersection_pt.x, q_next_intersection_pt.y,
                                                  self.height_map.height_at_xy(q_next_intersection_pt.x, q_next_intersection_pt.y)]
                    d_2 = MathUtils._2d_euclidian_distance(bl_xyzc, q_next_intersection_pt_xy)
                    d_2_3d = MathUtils._3d_euclidian_distance(bl_xyzc, q_next_intersection_pt_xyz)
                else:
                    raise RuntimeError("invalid leg number")

            elif moving_leg == 4:

                # calculate d2
                if next_moving_leg == 1:
                    q_next_moving_end_eff_to_incenter_line = LineString([fl_xyzc[0:2], [qnext_incenter_circX, qnext_incenter_circY]])
                    q_next_support_tri_diag_line = LineString([bl_xyz_next[0:2], fr_xyzc[0:2]])
                    q_next_intersection_pt = q_next_moving_end_eff_to_incenter_line.intersection( q_next_support_tri_diag_line)
                    q_next_intersection_pt_xy = [q_next_intersection_pt.x, q_next_intersection_pt.y]
                    q_next_intersection_pt_xyz = [q_next_intersection_pt.x, q_next_intersection_pt.y,
                                                  self.height_map.height_at_xy(q_next_intersection_pt.x, q_next_intersection_pt.y)]
                    d_2 = MathUtils._2d_euclidian_distance(fl_xyzc, q_next_intersection_pt_xy)
                    d_2_3d = MathUtils._3d_euclidian_distance(fl_xyzc, q_next_intersection_pt_xyz)

                elif next_moving_leg == 2:
                    q_next_moving_end_eff_to_incenter_line = LineString( [fr_xyzc[0:2], [qnext_incenter_circX, qnext_incenter_circY]])
                    q_next_support_tri_diag_line = LineString([fl_xyzc[0:2], br_xyzc[0:2]])
                    q_next_intersection_pt = q_next_moving_end_eff_to_incenter_line.intersection( q_next_support_tri_diag_line)
                    q_next_intersection_pt_xy = [q_next_intersection_pt.x, q_next_intersection_pt.y]
                    q_next_intersection_pt_xyz = [q_next_intersection_pt.x, q_next_intersection_pt.y,
                                                  self.height_map.height_at_xy(q_next_intersection_pt.x, q_next_intersection_pt.y)]
                    d_2 = MathUtils._2d_euclidian_distance(fr_xyzc, q_next_intersection_pt_xy)
                    d_2_3d = MathUtils._3d_euclidian_distance(fr_xyzc, q_next_intersection_pt_xyz)

                elif next_moving_leg == 3:
                    q_next_moving_end_eff_to_incenter_line = LineString( [br_xyzc[0:2], [qnext_incenter_circX, qnext_incenter_circY]])
                    q_next_support_tri_diag_line = LineString([bl_xyz_next[0:2], fr_xyzc[0:2]])
                    q_next_intersection_pt = q_next_moving_end_eff_to_incenter_line.intersection( q_next_support_tri_diag_line)
                    q_next_intersection_pt_xy = [q_next_intersection_pt.x, q_next_intersection_pt.y]
                    q_next_intersection_pt_xyz = [q_next_intersection_pt.x, q_next_intersection_pt.y,
                                                  self.height_map.height_at_xy(q_next_intersection_pt.x, q_next_intersection_pt.y)]
                    d_2 = MathUtils._2d_euclidian_distance(br_xyzc, q_next_intersection_pt_xy)
                    d_2_3d = MathUtils._3d_euclidian_distance(br_xyzc, q_next_intersection_pt_xyz)
                else:
                    raise RuntimeError("invalid leg number")

        except AttributeError as e:
            print("AttributeError: 'GeometryCollection' object has no attribute 'x'")
            return False

        if project_constants.STEPSEQ_USE_Z_IN_MAX_DIST_CALCS:
            if d_2_3d > project_constants.STEPSEQ_D12_3D_MULTIPLIER * \
                    project_constants.STEPSEQ_MAX_FOOTHOLD_TO_COM_ALONG_FOOTHOLD_INCENTER_LINE_DIST_d2:
                if verbose_debug:
                    print("d_2_3d:", d_2, "Returning FALSE")
                return False

        if d_2 > project_constants.STEPSEQ_MAX_FOOTHOLD_TO_COM_ALONG_FOOTHOLD_INCENTER_LINE_DIST_d2:
            if verbose_debug: print("stance: for stance d_2:",d_2,"Returning FALSE")
            return False

        if verbose_debug:
            print("stance d_2:", d_2)

        return True

    def get_best_fitting_smoothed_traj_idx(self, q, torso_x, torso_y, torso_yaw):
        rotation_scale = .005
        translation_scale = 1
        min_error = 10000
        min_error_idx = -1
        for i in range(len(self.high_density_xy_yaw_list)):
            xy_yaw = self.high_density_xy_yaw_list[i]
            translation_err = np.sqrt((xy_yaw[0]-torso_x)**2 + (xy_yaw[1]-torso_y)**2  )
            rotation_err = np.abs(xy_yaw[2]-torso_yaw)
            error = translation_scale * translation_err + rotation_scale*rotation_err
            if error < min_error:
                min_error = error
                min_error_idx = i
        return min_error_idx

    def get_hookpoints(self, torso_yaw_est, fl, fr, br, bl):
        fl_hookpoint_x_local, fl_hookpoint_y_local = MathUtils._2d_rotation_transformation(0, project_constants.HOOK_LENGTH, torso_yaw_est)
        fr_hookpoint_x_local, fr_hookpoint_y_local= MathUtils._2d_rotation_transformation(0, -project_constants.HOOK_LENGTH, torso_yaw_est)
        br_hookpoint_x_local, br_hookpoint_y_local = MathUtils._2d_rotation_transformation(0, -project_constants.HOOK_LENGTH, torso_yaw_est)
        bl_hookpoint_x_local, bl_hookpoint_y_local = MathUtils._2d_rotation_transformation(0, project_constants.HOOK_LENGTH, torso_yaw_est)
        fl_hookx, fl_hooky, fr_hookx, fr_hooky, br_hookx, br_hooky, bl_hookx, bl_hooky = fl[0] + fl_hookpoint_x_local, fl[1] + fl_hookpoint_y_local, \
               fr[0] + fr_hookpoint_x_local, fr[1] + fr_hookpoint_y_local,\
               br[0] + br_hookpoint_x_local, br[1] + br_hookpoint_y_local, \
               bl[0] + bl_hookpoint_x_local, bl[1] + bl_hookpoint_y_local
        return fl_hookx, fl_hooky, fr_hookx, fr_hooky, br_hookx, br_hooky, bl_hookx, bl_hooky

    def get_xy_hookpoints(self, leg_to_move, q_next_x, q_next_y, yaw_est ):
        x_local, y_local = -1, -1
        if leg_to_move == 1:
            x_local, y_local = MathUtils._2d_rotation_transformation(0, project_constants.HOOK_LENGTH, yaw_est)
        elif leg_to_move == 2:
            x_local, y_local = MathUtils._2d_rotation_transformation(0, -project_constants.HOOK_LENGTH, yaw_est)
        elif leg_to_move == 3:
            x_local, y_local = MathUtils._2d_rotation_transformation(0, -project_constants.HOOK_LENGTH, yaw_est)
        elif leg_to_move == 4:
            x_local, y_local = MathUtils._2d_rotation_transformation(0, project_constants.HOOK_LENGTH, yaw_est)
        else:
            Logger.log(("Unrecognized leg to move:" + str(leg_to_move)), "FAIL")
        return q_next_x + x_local, q_next_y + y_local

    def dist_from_hl_traj_costfn(self, xy_endeff, xy_hltraj):
        dist = MathUtils._2d_euclidian_distance(xy_endeff, xy_hltraj)
        if np.abs(dist - project_constants.BASE_STATE_END_EFF_DY_FROM_TORSO) < project_constants.STEPSEQ_NONPENALIZED_WIDTH_FROM_BASESTATE_Y:
            return 0
        return (dist - project_constants.BASE_STATE_END_EFF_DY_FROM_TORSO) ** 2

    def terrain_costfn(self, q_next_x, q_next_y):
        r = project_constants.END_AFFECTOR_RADIUS
        r_sqrt2 = r/np.sqrt(2)
        try:
            terrain_cost = (1/9) * (self.fs_cost_map.cost_at_xy(q_next_x, q_next_y) +
                         self.fs_cost_map.cost_at_xy(q_next_x+r, q_next_y) + self.fs_cost_map.cost_at_xy(q_next_x+r_sqrt2, q_next_y+r_sqrt2) +
                         self.fs_cost_map.cost_at_xy(q_next_x, q_next_y+r) + self.fs_cost_map.cost_at_xy(q_next_x-r_sqrt2, q_next_y+r_sqrt2) +
                         self.fs_cost_map.cost_at_xy(q_next_x-r, q_next_y) + self.fs_cost_map.cost_at_xy(q_next_x-r_sqrt2, q_next_y-r_sqrt2) +
                         self.fs_cost_map.cost_at_xy(q_next_x, q_next_y-r) + self.fs_cost_map.cost_at_xy(q_next_x+r_sqrt2, q_next_y-r_sqrt2))
        except ValueError:
            return self.fs_cost_map.cost_at_xy(q_next_x, q_next_y)
        return terrain_cost

    def ideal_location_costfn(self, x_des, y_des, q_next_x, q_next_y):
        if np.sqrt((q_next_x - x_des) ** 2 + ( q_next_y - y_des) ** 2) < project_constants.STEPSEQ_ZERO_COST_RADIUS:
            return 0
        return (q_next_x - x_des) ** 2 + (q_next_y - y_des) ** 2

    def hook_point_cost(self, q_next_x, q_next_y, hookpoint_next_x, hookpoint_next_y):
        q_next_h = self.height_map.height_at_xy(q_next_x, q_next_y)
        hook_point_h = self.height_map.height_at_xy(hookpoint_next_x, hookpoint_next_y)
        if hook_point_h > q_next_h + project_constants.HOOK_DIST_TO_GROUND - project_constants.STEPSEQ_HOOK_SAFETY_MARGIN:
            return project_constants.STEPSEQ_PREDICTED_HOOK_COLLISION_COST
        return 0

    def height_around_endeff_costfn(self, x, y, debug=False):
        r = project_constants.STEPSEQ_HEIGHT_AROUND_ENDEFF_R
        height_at_xy = self.height_map.height_at_xy(x, y)
        max_height_in_circle_r = self.height_map.max_in_radius_r_centered_at_xy(x, y, r)
        ret = 0
        if max_height_in_circle_r - height_at_xy > .3:
            ret = 1
        if debug:
            print(f"  height at {round(x, 2)} {round(y, 2)}: {round(height_at_xy, 2)} \t max in circle r ({round(r, 2)}):"
                  f" {round(max_height_in_circle_r, 2)}\t\t returning: {Logger.log_boolean(ret, invert=True)}")

        return ret

    def get_cost(self, curr_state, next_state, debug=False, force_vis=False):

        if next_state == StepSequencePlanner.goal_state:
            return self.h_weight * self.get_xy_yaw_error_from_goal(curr_state)
        else:
            # Compare to q_next footstep location
            leg_to_move_qnext_idx = int(curr_state[4] - 1)
            q_next_end_effector_xyzc = self.fs_scatter_list[int(next_state[leg_to_move_qnext_idx])]
            q_next_x = q_next_end_effector_xyzc[0]
            q_next_y = q_next_end_effector_xyzc[1]

            # Calculate end effector to move's ideal next (x,y,z) position. Idea is we find the robots estimated torso xyz,
            #   then the associated closest point along the hl trajectory to the torso. then move along the hl trajectory
            #   by some set distance (STEPSEQ_TRANSLATION_DISTANCE), and find the location where the moving end effector
            #   would be if the robot were in its 'base state', at the shifted point along trajectory. This is the
            #   'ideal fs location' cost, distance from the ideal point, outside of a zero cost circle of radius STEPSEQ_ZERO_COST_RADIUS
            #   is parabolically weighted,
            leg_to_move = curr_state[4]

            if curr_state in self.q_ideal_fs_locations:
                data = self.q_ideal_fs_locations[curr_state]
                leg_to_move_x_desired, leg_to_move_y_desired, torso_yaw_estimated = data[0], data[1], data[2]
                pt_at_torso_est_shifted_by_translation_dist_plus_base_state_x = data[3]
                ignore_dist_from_hl_traj = data[4]
            else:
                torso_x, torso_y, torso_yaw_estimated = self.get_estimated_torso_xy_yaw_rads(curr_state)
                torso_yaw_deg_est = np.rad2deg(torso_yaw_estimated)

                indexes_to_shift_along_hl_trajectory = int(project_constants.STEPSEQ_TRANSLATION_DISTANCE /
                                                           self.hl_traj.ave_higher_density_xy_yaw_path_distance_change)
                index_of_best_fitting_xy_yaw_along_smoothed_path = self.get_best_fitting_smoothed_traj_idx(curr_state, torso_x, torso_y, torso_yaw_deg_est)
                idx_shifted_torso = index_of_best_fitting_xy_yaw_along_smoothed_path + indexes_to_shift_along_hl_trajectory
                try:
                    torso_xy_yaw_when_shifted_down_smoothed_traj = self.high_density_xy_yaw_list[idx_shifted_torso]
                except IndexError:
                    torso_xy_yaw_when_shifted_down_smoothed_traj = self.high_density_xy_yaw_list[len(self.high_density_xy_yaw_list)-1]

                torso_dx = torso_xy_yaw_when_shifted_down_smoothed_traj[0] - torso_x
                torso_dy = torso_xy_yaw_when_shifted_down_smoothed_traj[1] - torso_y
                i = 1
                torso_dl = np.sqrt(torso_dy**2 + torso_dx**2)
                while torso_dl < .90 *project_constants.STEPSEQ_TRANSLATION_DISTANCE:
                    try:
                        idx_shifted_torso += i
                        torso_xy_yaw_when_shifted_down_smoothed_traj = self.high_density_xy_yaw_list[idx_shifted_torso]

                    # End of path
                    except IndexError:
                        torso_xy_yaw_when_shifted_down_smoothed_traj = self.high_density_xy_yaw_list[len(self.high_density_xy_yaw_list) - 1]
                        break
                    i += 1

                    torso_dx = torso_xy_yaw_when_shifted_down_smoothed_traj[0] - torso_x
                    torso_dy = torso_xy_yaw_when_shifted_down_smoothed_traj[1] - torso_y
                    torso_dl = np.sqrt(torso_dy ** 2 + torso_dx ** 2)

                fl_base, fr_base, br_base, bl_base = self.get_end_affector_xyz_coords_from_xy_yaw_deg_at_base_state(
                    torso_xy_yaw_when_shifted_down_smoothed_traj, debug=debug)

                # one or more legs are outside of the search area
                if not fl_base:
                    return np.inf

                base_statex_num_idxs_along_hltraj = int(project_constants.BASE_STATE_END_EFF_DX_FROM_TORSO /
                                                        self.hl_traj.ave_higher_density_xy_yaw_path_distance_change)

                idx_torso_shifted_plus_basestate_x = -1
                leg_to_move_y_desired, leg_to_move_x_desired = -1,-1
                if leg_to_move == 1:
                    leg_to_move_x_desired, leg_to_move_y_desired = fl_base[0], fl_base[1]
                    idx_torso_shifted_plus_basestate_x = idx_shifted_torso + base_statex_num_idxs_along_hltraj
                elif leg_to_move == 2:
                    idx_torso_shifted_plus_basestate_x = idx_shifted_torso + base_statex_num_idxs_along_hltraj
                    leg_to_move_x_desired, leg_to_move_y_desired = fr_base[0], fr_base[1]
                elif leg_to_move == 3:
                    idx_torso_shifted_plus_basestate_x = idx_shifted_torso - base_statex_num_idxs_along_hltraj
                    leg_to_move_x_desired, leg_to_move_y_desired = br_base[0], br_base[1]
                elif leg_to_move == 4:
                    idx_torso_shifted_plus_basestate_x = idx_shifted_torso - base_statex_num_idxs_along_hltraj
                    leg_to_move_x_desired, leg_to_move_y_desired = bl_base[0], bl_base[1]
                else:
                    Logger.log(("Unrecognized leg to move:" + str(leg_to_move)), "FAIL")

                ignore_dist_from_hl_traj = False
                if idx_torso_shifted_plus_basestate_x > (len(self.high_density_xy_yaw_list) - 1) or idx_torso_shifted_plus_basestate_x <= 0:
                    idx_torso_shifted_plus_basestate_x = 0
                    ignore_dist_from_hl_traj = True

                pt_at_torso_est_shifted_by_translation_dist_plus_base_state_x = self.high_density_xy_yaw_list[idx_torso_shifted_plus_basestate_x]

                # saving calculations reduces runtime
                data = [0,0,0,0,0]
                data[0] = leg_to_move_x_desired
                data[1] = leg_to_move_y_desired
                data[2] = torso_yaw_estimated
                data[3] = pt_at_torso_est_shifted_by_translation_dist_plus_base_state_x
                data[4] = ignore_dist_from_hl_traj
                self.q_ideal_fs_locations[curr_state] = data

            hookpoint_next_x, hookpoint_next_y = self.get_xy_hookpoints(leg_to_move, q_next_x, q_next_y, torso_yaw_estimated)

            # Terrain cost: combined costs of end effector + hook. simple approach to prevent hook from hitting things. 9 points taken at end end effector

            dist_from_hl_traj_cost = 0
            if not ignore_dist_from_hl_traj:
                dist_from_hl_traj_cost = project_constants.STEPSEQ_DIST_FROM_HL_TRAJ_COST_COEFF * \
                                         self.dist_from_hl_traj_costfn( [q_next_x, q_next_y],
                                             pt_at_torso_est_shifted_by_translation_dist_plus_base_state_x)

            if not self.fs_cost_map.xy_inbound(q_next_x, q_next_y) or not self.fs_cost_map.xy_inbound(hookpoint_next_x, hookpoint_next_y):
                return StepSequencePlanner.OUT_OF_BOUND_COST

            hook_point_cost = self.hook_point_cost(q_next_x, q_next_y, hookpoint_next_x, hookpoint_next_y)
            terrain_cost = project_constants.STEPSEQ_TERRAIN_COST_COEFF * self.terrain_costfn(q_next_x, q_next_y)
            ideal_location_cost = project_constants.STEPSEQ_IDEAL_LOCATION_COST_COEFF * \
                                  self.ideal_location_costfn(
                                      leg_to_move_x_desired, leg_to_move_y_desired, q_next_x, q_next_y
                                  )

            height_around_endeff_cost = project_constants.STEPSEQ_HEIGHT_NEAR_ENDEFF_COST_COEFF * \
                                        self.height_around_endeff_costfn(q_next_x, q_next_y)

            c = terrain_cost + ideal_location_cost + dist_from_hl_traj_cost + hook_point_cost + height_around_endeff_cost

            if self.detailed_visualization or force_vis:

                if self.plotted_costcurve == None:
                    self.plotted_costcurve = curr_state

                name = "considered_point"
                h = self.height_map.height_at_xy(q_next_x, q_next_y)
                VisUtils.visualize_cost(q_next_x, q_next_y, c, name, with_z=h, color=VisUtils.RED, hide_label=True)

                if curr_state != self.plotted_costcurve or force_vis:
                    # if False:
                    step = 5
                    self.plotted_costcurve = curr_state

                    fl_xyzc, fr_xyzc, br_xyzc, bl_xyzc = self.get_end_affectors_xyzc(curr_state)
                    tri = [fl_xyzc, fr_xyzc, br_xyzc, bl_xyzc]
                    # print "\n  considering stance:",logger.pp_list(stance)
                    if leg_to_move == 1:
                        tri.remove(fl_xyzc)
                    elif leg_to_move == 2:
                        tri.remove(fr_xyzc)
                    elif leg_to_move == 3:
                        tri.remove(br_xyzc)
                    elif leg_to_move == 4:
                        tri.remove(bl_xyzc)

                    VisUtils.visualize_xyz_list(tri,name="support_triangle",height_map=self.height_map, loop=True)
                    inscribed_circX,inscribed_circY,inscribed_circR = MathUtils.incenter_circle_xy_R_fromT(tri)
                    VisUtils.visualize_circle(inscribed_circX,inscribed_circY,inscribed_circR,"inscribed_circle", hm=self.height_map)
                    # print inscribed_circR

                    name = "fl"
                    h = self.height_map.height_at_xy(fl_xyzc[0], fl_xyzc[1])
                    p1 = [fl_xyzc[0], fl_xyzc[1], h]
                    p2 = [fl_xyzc[0], fl_xyzc[1], h + 1]
                    self.add_line_to_vis(name, p1, p2)
                    vis.setColor(name, 0, 1, 0, a=1.0)
                    vis.hideLabel(name)

                    name = "fr"
                    h = self.height_map.height_at_xy(fr_xyzc[0], fr_xyzc[1])
                    p1 = [fr_xyzc[0], fr_xyzc[1], h]
                    p2 = [fr_xyzc[0], fr_xyzc[1], h + 1]
                    self.add_line_to_vis(name, p1, p2)
                    vis.setColor(name, 0, 1, 0, a=1.0)
                    vis.hideLabel(name)

                    name = "br"
                    h = self.height_map.height_at_xy(br_xyzc[0], br_xyzc[1])
                    p1 = [br_xyzc[0], br_xyzc[1], h]
                    p2 = [br_xyzc[0], br_xyzc[1], h + 1]
                    self.add_line_to_vis(name, p1, p2)
                    vis.setColor(name, 0, 1, 0, a=1.0)
                    vis.hideLabel(name)

                    name = "bl"
                    h = self.height_map.height_at_xy(bl_xyzc[0], bl_xyzc[1])
                    p1 = [bl_xyzc[0], bl_xyzc[1], h]
                    p2 = [bl_xyzc[0], bl_xyzc[1], h + 1]
                    self.add_line_to_vis(name, p1, p2)
                    vis.setColor(name, 0, 1, 0, a=1.0)
                    vis.hideLabel(name)

                    fl_hookx, fl_hooky, fr_hookx, fr_hooky, br_hookx, br_hooky, bl_hookx, bl_hooky = \
                        self.get_hookpoints(torso_yaw_estimated,fl_xyzc, fr_xyzc, br_xyzc, bl_xyzc)

                    name = "fl_hook"
                    VisUtils.visualize_cost(fl_hookx, fl_hooky, .5, name)
                    vis.setColor(name, 0, 1, 1, a=1.0)
                    vis.hideLabel(name)

                    name = "fr_hook"
                    VisUtils.visualize_cost(fr_hookx, fr_hooky, .5, name)
                    vis.setColor(name, 0, 1, 1, a=1.0)
                    vis.hideLabel(name)

                    name = "br_hook"
                    VisUtils.visualize_cost(br_hookx, br_hooky, .5, name)
                    vis.setColor(name, 0, 1, 1, a=1.0)
                    vis.hideLabel(name)

                    name = "bl_hook"
                    VisUtils.visualize_cost(bl_hookx, bl_hooky, .5, name)
                    vis.setColor(name, 0, 1, 1, a=1.0)
                    vis.hideLabel(name)

                    for x in range(-50, 50, step):
                        for y in range(-50, 50, step):
                            world_x = leg_to_move_x_desired + x / 100.0
                            world_y = leg_to_move_y_desired + y / 100.0
                            try:

                                hookpoint_next_x, hookpoint_next_y = self.get_xy_hookpoints(leg_to_move, world_x, world_y, torso_yaw_estimated)

                                world_cost = project_constants.STEPSEQ_TERRAIN_COST_COEFF * self.terrain_costfn(world_x, world_y)

                                hook_point_cost = self.hook_point_cost(world_x, world_y, hookpoint_next_x, hookpoint_next_y)

                                porabala_cost = project_constants.STEPSEQ_IDEAL_LOCATION_COST_COEFF * \
                                                self.ideal_location_costfn(leg_to_move_x_desired, leg_to_move_y_desired, world_x, world_y)

                                dist_from_hl_traj_cost = \
                                    project_constants.STEPSEQ_DIST_FROM_HL_TRAJ_COST_COEFF * \
                                    self.dist_from_hl_traj_costfn(
                                        [world_x, world_y],
                                        pt_at_torso_est_shifted_by_translation_dist_plus_base_state_x
                                    )

                                if ignore_dist_from_hl_traj:
                                    dist_from_hl_traj_cost = 0

                                cost = world_cost + porabala_cost + dist_from_hl_traj_cost + hook_point_cost
                            except ValueError:
                                cost = 0
                            name = str(x) + "," + str(y)
                            try:
                                h = self.height_map.height_at_xy(world_x, world_y)
                            except ValueError:
                                h = 0
                            p1 = [world_x,world_y,h]
                            p2 = [world_x,world_y,h+cost]
                            self.add_line_to_vis(name,p1,p2)
                            vis.hideLabel(name)
                    # time.sleep(1)
                    # print "Leg to move:",leg_to_move,"qnext:",q_next, "desired xy:",
                    # logger.pp_list([leg_to_move_x_desired, leg_to_move_y_desired]), "leg xy:",logger.pp_list([q_next_x, q_next_y])

            return self.g_weight * c


