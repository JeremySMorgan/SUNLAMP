import random
import time
import numpy as np
from klampt.math import so3
from heapq import heappop
from heapq import heappush
from shapely.geometry import LineString
from klampt.math import vectorops
from klampt.model import ik
from klampt.model.collide import WorldCollider
from klampt.plan import cspace
from klampt.plan import robotcspace
from klampt.robotsim import IKSolver
from klampt import vis
from src.utils.geometry_objects._2d_triangle_geometry import SupportTriangle
from src.motion.motion_utils import MotionUtils, Constraints
from src.utils.logger import Logger as Lg
from src.utils.math_utils import MathUtils
from src.utils.vis_utils import VisUtils
from src.utils import project_constants


def end_config_exhaustive_search(
        x0: float, y0: float, xf: float, yf: float, x_delta: float, y_delta: float,
        z_offsets: list, curr_state, torso_xy_inbound, self, config_valid,  break_early_min_cost_ratio,
        end_config_iktargets, get_config_cost,
        bias_config=None, break_on_first_found_q=False, with_rotation_R=None, debug=False,
        ignore_dist_to_nominal_z_cost=False):

    t_start = time.time()
    search_cnt = 0
    x = x0
    y = y0
    y_min = y0
    break_early = False
    initial_min_cost = None
    lowest_cost_config_c = np.inf
    lowest_cost_config = None
    start_t = time.time()

    possible_configs_found = 0
    torso_outbount_cnt = 0
    z_offset_end = None

    while x < xf:
        while y < yf:

            if torso_xy_inbound(x, y):

                z_des = self.get_torso_z_des_from_xy([x, y], curr_state, debug=debug)

                for offset in z_offsets:

                    z = z_des + offset

                    torso_xyz = [x, y, z]
                    if with_rotation_R:
                        offset_torso_obj = ik.objective(self.torso, R=with_rotation_R, t=torso_xyz)
                    else:
                        offset_torso_obj = ik.objective(self.torso, local=[0, 0, 0], world=torso_xyz)

                    ik_solver = IKSolver(self.robosimian)
                    for goal in end_config_iktargets + [offset_torso_obj]:
                        ik_solver.add(goal)

                    if bias_config:
                        # bias_config[0] = torso_xyz[0]
                        # bias_config[1] = torso_xyz[1]
                        # bias_config[2] = torso_xyz[2]
                        ik_solver.setBiasConfig(bias_config)

                    res = ik_solver.solve()
                    config = self.robosimian.getConfig()

                    search_cnt += 1

                    if config_valid(config, debug=False) and res:

                        if break_on_first_found_q:
                            z_offset_end = offset
                            break_early = True
                            break

                        possible_configs_found += 1
                        cost = get_config_cost(config, ignore_dist_to_nominal_z=ignore_dist_to_nominal_z_cost)[0]
                        if initial_min_cost is None:
                            initial_min_cost = cost

                        ratio = cost / initial_min_cost

                        if cost < lowest_cost_config_c:

                            lowest_cost_config_c = cost
                            lowest_cost_config = config
                            z_offset_end = offset

                            if project_constants.MPLANNER_VERBOSITY >= 4:
                                print(
                                    f"end_config_exhaustive_search(): lowest cost config found ({possible_configs_found} total), cost: {round(cost, 3)}\t"
                                    f" ratio: {round(ratio, 3)}, break early ratio: {round(break_early_min_cost_ratio, 3)}")

                        if ratio < break_early_min_cost_ratio:
                            if project_constants.MPLANNER_VERBOSITY >= 3:
                                print(
                                    f"ratio {round(ratio, 3)} < break early ratio: {round(break_early_min_cost_ratio, 3)}, exiting")
                            break_early = True
                            break

            else:
                torso_outbount_cnt += 1
            if break_early:
                break

            y += y_delta

        if break_early:
            break

        y = y_min
        x += x_delta

    if project_constants.MPLANNER_VERBOSITY >= 3:
        print(f"end_config_exhaustive_search() finished in {Lg.log_bold(round(time.time() - t_start, 2))} seconds, {possible_configs_found} end configs found")

    return lowest_cost_config, lowest_cost_config_c, search_cnt, time.time() - start_t, x, y, z_offset_end, break_early


class MPlannerResults:

    UNFILLED = -1

    def __init__(self):

        self.stance_idx = MPlannerResults.UNFILLED
        self.leg_step = None
        self.success = None
        self.resulting_config_path = []

        self.initial_rpose = None
        self.final_rpose = None

        self.nb_preset_loc_endconfigs_found = 0
        self.nb_randomized_search_endconfigs_found = 0
        self.nb_exaustive_search_endconfigs_found = 0

        self.preset_loc_endconfig_search_time = 0
        self.randomized_search_endconfig_search_time = 0
        self.exaustive_search_search_time = 0

        self.plan_time_on_preset_loc_gen_configs = 0
        self.trajectories_run_on_preset_loc_gen_configs = 0

        self.plan_time_on_randomized_search_gen_configs = 0
        self.trajectories_run_on_randomized_search_gen_configs = 0

        self.plan_time_on_exaustive_search_gen_configs = 0
        self.trajectories_run_on_exaustive_search_gen_configs = 0

class EndConfigGenerator(MotionUtils):

    def __init__(self, world, fs_scatter_obj, fs_seq, height_map, gradient_map):

        MotionUtils.__init__(self, world, height_map, fs_scatter_obj.get_scatter_list(), fs_seq.get_state_path(), gradient_map)
        


class ConfigSpacePlanner(MotionUtils):

    max_joint_angle_change_deg = 2.5
    max_allowed_joint_angle_change = np.deg2rad(max_joint_angle_change_deg)

    if project_constants.MPLANNER_VERBOSITY >= 3:
        print("\nmax_allowed_joint_angle_change:", round(max_allowed_joint_angle_change, 4))

    max_torso_translation_dist = .015
    max_torso_rotation_deg = 1.5

    qminmax_delta_arr = max_allowed_joint_angle_change * np.ones(38)
    qminmax_delta_arr[0] = max_torso_translation_dist
    qminmax_delta_arr[1] = max_torso_translation_dist
    qminmax_delta_arr[2] = max_torso_translation_dist
    qminmax_delta_arr[3] = np.deg2rad(max_torso_rotation_deg)
    qminmax_delta_arr[4] = np.deg2rad(max_torso_rotation_deg)
    qminmax_delta_arr[5] = np.deg2rad(max_torso_rotation_deg)

    leg1_j1_limits = (-1.8, 2.1)  # j 31
    leg2_j1_limits = (-2.1, 1.8)  # j7
    leg3_j1_limits = (-1.8, 2.1)  # j15
    leg4_j1_limits = (-2.1, 1.8)  # j23

    def __init__(self, world, fs_scatter_obj, fs_seq, height_map, gradient_map, u_input=None, lidar_mode=False):

        MotionUtils.__init__(
            self, world, height_map, fs_scatter_obj.get_scatter_list(), fs_seq.get_state_path(),
            gradient_map, u_input=u_input, lidar_mode=lidar_mode)

        self.collider = WorldCollider(self.world)

    @staticmethod
    def configs_j1s(q: list):
        return f"q_j1s = [{round(q[7], 3)}, {round(q[15], 3)}, {round(q[23], 3)}, {round(q[31], 3)}]"

    @staticmethod
    def path_valid(_path, q_valid, _debug=False):
        if _debug:
            errors = 0
            for i in range(len(_path)):
                _valid = q_valid(_path[i])
                print(f"{i}: {Lg.log_boolean(_valid)}")
                if not _valid:
                    errors += 1
            print(f" path_valid(): {Lg.log_boolean(errors == 0, msg=str(errors))} invalid qs")
            return errors == 0
        else:
            for i in range(len(_path)):
                _valid = q_valid(_path[i])
                if i >= 1:
                    joint_ang_change = MathUtils.max_joint_angle_distance(_path[i], _path[i-1])
                if not _valid:
                    return False
            return True

    @staticmethod
    def joint_limits(q):
        qmin = np.array(q) - ConfigSpacePlanner.qminmax_delta_arr
        qmax = np.array(q) + ConfigSpacePlanner.qminmax_delta_arr
        # qmin[31] = max(qmin[31], MotionPlanner.leg1_j1_limits[0])
        # qmax[31] = min(qmax[31], MotionPlanner.leg1_j1_limits[1])
        # qmin[7] = max(qmin[7], MotionPlanner.leg2_j1_limits[0])
        # qmax[7] = min(qmax[7], MotionPlanner.leg2_j1_limits[1])
        # qmin[15] = max(qmin[15], MotionPlanner.leg3_j1_limits[0])
        # qmax[15] = min(qmax[15], MotionPlanner.leg3_j1_limits[1])
        # qmin[23] = max(qmin[23], MotionPlanner.leg4_j1_limits[0])
        # qmax[23] = min(qmax[23], MotionPlanner.leg4_j1_limits[1])
        return qmin, qmax

    @staticmethod
    def joint_angles_valid(q, debug=False):
        l1 = ConfigSpacePlanner.leg1_j1_limits[0] < q[31] < ConfigSpacePlanner.leg1_j1_limits[1]
        l2 = ConfigSpacePlanner.leg2_j1_limits[0] < q[7] < ConfigSpacePlanner.leg2_j1_limits[1]
        l3 = ConfigSpacePlanner.leg3_j1_limits[0] < q[14] < ConfigSpacePlanner.leg3_j1_limits[1]
        l4 = ConfigSpacePlanner.leg4_j1_limits[0] < q[23] < ConfigSpacePlanner.leg4_j1_limits[1]
        if debug:
            print(f"joint_angles_valid(stance): {l1 and l2 and l3 and l4}")
            print(f"   l1: {l1} \t {round(q[31], 3)}")
            print(f"   l2: {l2} \t {round(q[7], 3)}")
            print(f"   l3: {l3} \t {round(q[14], 3)}")
            print(f"   l4: {l4} \t {round(q[23], 3)}")
        return l1 and l2 and l3 and l4

    def ik_solve(self, q_for_joint_limits, ik_targets, config_valid, bias_config=None):
        ik_solver = IKSolver(self.robosimian)
        qmin, qmax = ConfigSpacePlanner.joint_limits(q_for_joint_limits)
        ik_solver.setJointLimits(qmin, qmax)
        for goal in ik_targets:
            ik_solver.add(goal)
        if bias_config:
            ik_solver.setBiasConfig(bias_config)
        res = ik_solver.solve()
        q = self.robosimian.getConfig()
        if res and config_valid(q):
            return q
        return None

    def get_endconfigs(
            self, stance_idx, constraint_obj, fl_obj, fr_obj, bl_obj, br_obj, _config_valid, start_config, step,
            debug=False, visualize=False, run_preset_location_search=True, run_probabilistic_search=True,
            run_exaustive_grid_search=True):

        '''
            if calling with step=True
                get_endconfig() finds a configuration in which the torso is in the support region, and the moving end effector is
                at its next hold.

            if calling with step=False:
                This fn is called after a step is made. at this point, there will be a new support triangle, composed of
                all foot holds which wont be moving in the next step.
                returns config in which the torso is in the new support region.
        '''

        # fn wide variables
        return_configs = []
        curr_stance = self.stance_path[stance_idx]
        moving_leg = self.get_moving_leg_from_stance_idx(stance_idx)
        fl_xyz, fr_xyz, br_xyz, bl_xyz = self.get_end_affector_xyzs_from_curr_stance(curr_stance)
        xyzR_next = self.get_moving_leg_xyzR_f(moving_leg, stance_idx, visualize_normal=visualize)

        range_circles = constraint_obj.get_range_circles()
        support_tri: SupportTriangle = constraint_obj.get_support_triangles()[0]
        torso_x_est, torso_y_est, yaw_rads_est_at_q = self.estimate_torso_xy_yaw_rads_from_stance(self.stance_path[stance_idx])
        nominal_q_joint_angles = project_constants.NOMINAL_CONFIG[6:]
        support_tri_incenter_xy = [support_tri.incenterx, support_tri.incentery]
        visitems_to_clear = []

        if step:
            if moving_leg == 1:
                fl_obj = ik.objective(self.fl_end_effector, R=xyzR_next[3], t=xyzR_next[0:3])
            if moving_leg == 2:
                fr_obj = ik.objective(self.fr_end_effector, R=xyzR_next[3], t=xyzR_next[0:3])
            if moving_leg == 3:
                br_obj = ik.objective(self.br_end_effector, R=xyzR_next[3], t=xyzR_next[0:3])
            if moving_leg == 4:
                bl_obj = ik.objective(self.bl_end_effector, R=xyzR_next[3], t=xyzR_next[0:3])

        end_config_iktargets = [fl_obj, fr_obj, bl_obj, br_obj]

        nominal_config_cp = project_constants.NOMINAL_CONFIG[:]
        nominal_config_cp[0], nominal_config_cp[1] = torso_x_est, torso_y_est
        nominal_config_cp[5] = yaw_rads_est_at_q

        solver = IKSolver(self.robosimian)
        for obj in end_config_iktargets:
            solver.add(obj)
        solver.setBiasConfig(nominal_config_cp)
        res = solver.solve()
        if not res:
            Lg.log("ERROR: Unable to solve for bias config, res=False", "WARNING")
            bias_config = nominal_config_cp
        else:
            bias_config = self.robosimian.getConfig()

        def torso_xy_inbound(x, y, ignore_range_tris=True):
            return constraint_obj.xy_in_support_area(x, y, ignore_range_tris=ignore_range_tris)

        # # TODO: Test that fn works as expected
        # def torso_com_validator(q):
        #     self.robosimian.setConfig(q)
        #     com = self.robosimian.getCom()
        #     return constraint_obj.xy_in_support_area(com[0], com[1])

        def config_valid(q, debug=False):
            if not torso_xy_inbound(q[0], q[1]):
                if debug:
                    print("!torso_xy_inbound(x,y)")
                return False
            # if not torso_com_validator(q):
            #     if debug:
            #         print("!torso_xy_inbound(x,y)")
            #     return False
            return _config_valid(q, debug=debug)

        def randomized_centered_search( nb_samples, torso_xyz, max_dx, max_dy, max_dz, bias_config=None, debug=False):

            function_heap = []
            initial_lowestcost = None
            min_costs = []

            num_min_costs = 0
            mincost_break_threshold = .7
            broke_early = False

            i = 0
            while i < nb_samples:

                u = float(i) / float(num_samples)
                rx = random.uniform(-u*max_dx, u*max_dx)
                ry = random.uniform(-u*max_dy, u*max_dy)
                rz = random.uniform(-u*max_dz, u*max_dz)
                t = [torso_xyz[0] + rx, torso_xyz[1] + ry, torso_xyz[2] + rz]
                i += .1

                if torso_xy_inbound(t[0], t[1]):

                    i += .9

                    offset_torso_obj = ik.objective(self.torso, local=[0, 0, 0], world=t)
                    offset_ik_targets = end_config_iktargets[:]
                    offset_ik_targets.append(offset_torso_obj)

                    ik_solver = IKSolver(self.robosimian)

                    for goal in offset_ik_targets:
                        ik_solver.add(goal)

                    if bias_config:
                        ik_solver.setBiasConfig(bias_config)
                        # if debug:
                        #     print(f"\nrandomized_centered_search() Setting bias config, hash: {hash(tuple(ik_solver.getBiasConfig()))}")

                    res = ik_solver.solve()
                    q = self.robosimian.getConfig()

                    if config_valid(q, debug=False) and res is not False:

                        cost = get_config_cost(q)
                        if len(min_costs) == 0:
                            if project_constants.MPLANNER_VERBOSITY >= 3:
                                print("randomized_centered_search() Found valid end config, with heuristic cost:", Lg.pp_double(cost[0]))
                            initial_lowestcost = cost[0]
                            heappush(function_heap, cost)
                            min_costs.append(initial_lowestcost)
                        else:
                            if cost[0] < min(min_costs):
                                heappush(function_heap, cost)
                                num_min_costs += 1
                                min_costs.append(cost[0])
                                ratio = cost[0] / initial_lowestcost

                                if ratio < mincost_break_threshold:
                                    if project_constants.MPLANNER_VERBOSITY >= 3:
                                        print(f"randomized_centered_search() sample {int(i)}: new lowest cost: {round(cost[0], 3)} ratio: "
                                              f"{Lg.pp_double(ratio)} -> breaking")
                                    broke_early = True
                                    break
                                else:
                                    if project_constants.MPLANNER_VERBOSITY >= 3:
                                        print(f"randomized_centered_search() sample {int(i)}: new lowest cost: {round(cost[0], 3)} ratio: {Lg.pp_double(ratio)}")
            if debug and broke_early:
                print("Breaking early from end config sampling. ", len(function_heap), "valid end configs found")
            try:
                cqbest = heappop(function_heap)
                end_config = cqbest[1]
                if debug:
                    get_config_cost(end_config, debug=False)
                return end_config
            except IndexError:
                return None

        def get_config_cost(config, debug=False, ignore_dist_to_nominal_z=False):

            dist_from_nominal_q_joint_angles = np.sum(np.power(MathUtils.joint_angle_distances(nominal_q_joint_angles, config[6:]), 2))
            dist_to_incenter = vectorops.distance(config[0:2], support_tri_incenter_xy)

            # TODO: Is this correct?
            angle_to_rotate_to_upright = so3.distance(so3.from_rpy(config[3:6]), torso_R_des)
            dist_to_nominal_z = np.abs(self.get_torso_z_des_from_xy(config[0:2], config) - config[2])

            c_joint_dist_from_nominal_ = .75 * dist_from_nominal_q_joint_angles
            c_dist_to_incenter = dist_to_incenter * .25
            c_angle_to_rotate = angle_to_rotate_to_upright * 25
            c_dist_to_nominal_z = dist_to_nominal_z * 5.0

            c = c_joint_dist_from_nominal_ + c_dist_to_incenter + c_angle_to_rotate + c_dist_to_nominal_z

            if ignore_dist_to_nominal_z:
                c -= c_dist_to_nominal_z

            if project_constants.MPLANNER_VERBOSITY >= 3:
                print(f"get_config_cost(stance) hash(stance): {str(hash(tuple(config)))[0:8]}")
                print(f"  dist_from_nominal_q: {round(float(dist_from_nominal_q_joint_angles), 3)}\t cost: {round(c_joint_dist_from_nominal_,3)}")
                print(f"  dist_to_incenter: {round(dist_to_incenter, 3)}\t cost: {round(c_dist_to_incenter,3)}")
                print(f"  angle_to_upright: {round(angle_to_rotate_to_upright, 3)}\t cost: {round(c_angle_to_rotate, 3)}")
                if ignore_dist_to_nominal_z:
                    print(f"  dist_to_nominal_z: 0 (ignored)")
                else:
                    print(f"  dist_to_nominal_z: {round(dist_to_nominal_z, 3)}\t cost: {round(c_dist_to_nominal_z,3)}")
                print(f"  cost: {round(c, 3)}\n")

            return c, config

        if visualize:
            support_tri.visualize()

        # ctdol: Closest to diaganol opposite leg (from moving leg)
        # ctml: Closest to moving leg
        # ctsfobl: Closest to same front or back leg (from moving leg) Ex: moving leg is FL, ctsfobl is FR
        # ctofobl: Closest to Opposite front or back leg (from moving leg) Ex: moving leg is FL, ctsfobl is BL

        d = .05
        if visualize:
            visitems_to_clear.append("daig_com_dist")

        if moving_leg == 1:

            support_tri_diag_line = support_tri.get_diag_linestring()
            if visualize:
                VisUtils.visualize_shapely_line(support_tri_diag_line, add_z=.025, color=VisUtils.WHITE)

            moving_leg_to_incenter_line = LineString([fl_xyz[0:2], [support_tri.incenterx, support_tri.incentery]])
            torso_xy_pt = support_tri_diag_line.intersection(moving_leg_to_incenter_line)
            vec_towards_opp_leg = MathUtils.normalize([br_xyz[0] - fl_xyz[0], br_xyz[1] - fl_xyz[1] ])

            try:
                support_tri_intersection_ptxy = MathUtils.add_scaled_vector_to_pt([torso_xy_pt.x, torso_xy_pt.y], vec_towards_opp_leg, d)
            except AttributeError:
                support_tri_intersection_ptxy = None

            intersection_centroid_xy_ctdol = support_tri.xy_centroid_from_o_3dgeoms(range_circles, closest_to=br_xyz)
            intersection_centroid_xy_ctml = support_tri.xy_centroid_from_o_3dgeoms(range_circles, closest_to=fl_xyz)
            intersection_centroid_xy_ctsfobl = support_tri.xy_centroid_from_o_3dgeoms(range_circles, closest_to=fr_xyz)
            intersection_centroid_xy_ctofobl = support_tri.xy_centroid_from_o_3dgeoms(range_circles, closest_to=bl_xyz)

            if visualize:
                VisUtils.visualize_line(fl_xyz, support_tri_intersection_ptxy, name="daig_com_dist")

        elif moving_leg == 2:

            support_tri_diag_line = support_tri.get_diag_linestring()
            if visualize:
                VisUtils.visualize_shapely_line(support_tri_diag_line, add_z=.025, color=VisUtils.WHITE)

            moving_leg_to_incenter_line = LineString([fr_xyz[0:2], [support_tri.incenterx, support_tri.incentery]])
            torso_xy_pt = support_tri_diag_line.intersection(moving_leg_to_incenter_line)
            vec_towards_opp_leg = MathUtils.normalize([bl_xyz[0] - fr_xyz[0], bl_xyz[1] - fr_xyz[1]])

            try:
                support_tri_intersection_ptxy = MathUtils.add_scaled_vector_to_pt([torso_xy_pt.x, torso_xy_pt.y], vec_towards_opp_leg, d)
            except AttributeError:
                support_tri_intersection_ptxy = None

            intersection_centroid_xy_ctdol = support_tri.xy_centroid_from_o_3dgeoms(range_circles, closest_to=bl_xyz)
            intersection_centroid_xy_ctml = support_tri.xy_centroid_from_o_3dgeoms(range_circles, closest_to=fr_xyz)
            intersection_centroid_xy_ctsfobl = support_tri.xy_centroid_from_o_3dgeoms(range_circles, closest_to=fl_xyz)
            intersection_centroid_xy_ctofobl = support_tri.xy_centroid_from_o_3dgeoms(range_circles, closest_to=br_xyz)

            if visualize:
                VisUtils.visualize_line(fr_xyz, support_tri_intersection_ptxy, name="daig_com_dist")

        elif moving_leg == 3:

            support_tri_diag_line = support_tri.get_diag_linestring()
            if visualize:
                VisUtils.visualize_shapely_line(support_tri_diag_line, add_z=.025, color=VisUtils.WHITE)

            moving_leg_to_incenter_line = LineString([br_xyz[0:2], [support_tri.incenterx, support_tri.incentery]])
            torso_xy_pt = support_tri_diag_line.intersection(moving_leg_to_incenter_line)
            vec_towards_opp_leg = MathUtils.normalize([fl_xyz[0] - br_xyz[0], fl_xyz[1] - br_xyz[1]])

            try:
                support_tri_intersection_ptxy = MathUtils.add_scaled_vector_to_pt([torso_xy_pt.x, torso_xy_pt.y], vec_towards_opp_leg, d)
            except AttributeError:
                support_tri_intersection_ptxy = None

            intersection_centroid_xy_ctdol = support_tri.xy_centroid_from_o_3dgeoms(range_circles, closest_to=fl_xyz)
            intersection_centroid_xy_ctml = support_tri.xy_centroid_from_o_3dgeoms(range_circles, closest_to=br_xyz)
            intersection_centroid_xy_ctsfobl = support_tri.xy_centroid_from_o_3dgeoms(range_circles, closest_to=bl_xyz)
            intersection_centroid_xy_ctofobl = support_tri.xy_centroid_from_o_3dgeoms(range_circles, closest_to=fr_xyz)

            if visualize:
                VisUtils.visualize_line(br_xyz, support_tri_intersection_ptxy, name="daig_com_dist")

        elif moving_leg == 4:
            support_tri_diag_line = support_tri.get_diag_linestring()
            if visualize:
                VisUtils.visualize_shapely_line(support_tri_diag_line, add_z=.025, color=VisUtils.WHITE)

            moving_leg_to_incenter_line = LineString([bl_xyz[0:2], [support_tri.incenterx, support_tri.incentery]])
            torso_xy_pt = support_tri_diag_line.intersection(moving_leg_to_incenter_line)
            vec_towards_opp_leg = MathUtils.normalize([fr_xyz[0] - bl_xyz[0], fr_xyz[1] - bl_xyz[1] ])

            try:
                support_tri_intersection_ptxy = MathUtils.add_scaled_vector_to_pt([torso_xy_pt.x, torso_xy_pt.y], vec_towards_opp_leg, d)
            except AttributeError:
                support_tri_intersection_ptxy = None

            intersection_centroid_xy_ctdol = support_tri.xy_centroid_from_o_3dgeoms(range_circles, closest_to=fr_xyz)
            intersection_centroid_xy_ctml = support_tri.xy_centroid_from_o_3dgeoms(range_circles, closest_to=bl_xyz)
            intersection_centroid_xy_ctsfobl = support_tri.xy_centroid_from_o_3dgeoms(range_circles, closest_to=br_xyz)
            intersection_centroid_xy_ctofobl = support_tri.xy_centroid_from_o_3dgeoms(range_circles, closest_to=fl_xyz)

            if visualize:
                VisUtils.visualize_line(bl_xyz, support_tri_intersection_ptxy, name="daig_com_dist")
        else:
            Lg.log("invalid leg to move #", "FAIL")
            return None

        dL = 0
        intersection_centroid_xy_ctdol = MathUtils.add_scaled_vector_to_pt(
            intersection_centroid_xy_ctdol,vec_towards_opp_leg, dL)

        intersection_centroid_xy_ctml = MathUtils.add_scaled_vector_to_pt(
            intersection_centroid_xy_ctml,vec_towards_opp_leg, dL)

        intersection_centroid_xy_ctsfobl = MathUtils.add_scaled_vector_to_pt(
            intersection_centroid_xy_ctsfobl,vec_towards_opp_leg, dL)

        intersection_centroid_xy_ctofobl = MathUtils.add_scaled_vector_to_pt(
            intersection_centroid_xy_ctofobl, vec_towards_opp_leg, dL)

        intersection_centroid_xy = support_tri.xy_centroid_from_o_3dgeoms(range_circles)
        torso_R = self.get_torso_R_from_yaw_rad(yaw_rads_est_at_q)
        z_delta = .065

        endconfig_xys = [
            start_config[0:2],
            intersection_centroid_xy,
            intersection_centroid_xy_ctdol,
            intersection_centroid_xy_ctml,
            intersection_centroid_xy_ctsfobl,
            intersection_centroid_xy_ctofobl
        ]
        if support_tri_intersection_ptxy:
            endconfig_xys.append(support_tri_intersection_ptxy)


        # _____________________________________ Try solving ik problem at each intersection point

        torso_R_des = self.get_torso_R_from_yaw_rad(yaw_rads_est_at_q)
        search_debug = False

        if run_preset_location_search:

            for i in range(len(endconfig_xys)):

                torso_xy_pt = endconfig_xys[i]
                z_des = self.get_torso_z_des_from_xy(torso_xy_pt, curr_stance, debug=debug)
                self.robosimian.setConfig(start_config)
                ik_solver = IKSolver(self.robosimian)

                for goal in end_config_iktargets:
                    ik_solver.add(goal)

                torso_xyz = [torso_xy_pt[0], torso_xy_pt[1], z_des]
                torso_obj = ik.objective(self.torso, R=torso_R, t=torso_xyz)
                ik_solver.add(torso_obj)
                ik_solver.setBiasConfig(bias_config)
                res = ik_solver.solve()

                q = self.robosimian.getConfig()
                q_valid = config_valid(q, debug=False)

                # if debug:
                #     dist = MathUtils._2d_euclidian_distance(q, torso_xy_pt)
                #     print(f"____\nget_endconfigs() preset_loc_{i} IKSolver()\tres: {Lg.log_boolean(res)}, "
                #           f"config valid: {Lg.log_boolean(q_valid)}\t dist from specified torso_xy: {round(dist, 3)}")
                #     config_valid(q, debug=True)

                # end_config_iktargets += [torso_obj]
                # if not res or not q_valid:
                #     self.robosimian.setConfig(bias_config)
                #     res = ik.solve(end_config_iktargets, activeDofs=project_constants.ACTIVE_DOFS)
                #     q = self.robosimian.getConfig()
                #     q_valid = config_valid(q, project_constants.MPLANNER_VERBOSITY >= 3=False)
                #     # if debug:
                #     #     dist = MathUtils._2d_euclidian_distance(q, torso_xy_pt)
                #     #     print(f"get_endconfigs() preset_loc_{i} ik.solve()\tres: {Lg.log_boolean(res)}, "
                #     #           f"config valid: {Lg.log_boolean(q_valid)}\t dist from specified torso_xy: {round(dist, 3)}")
                #     #     config_valid(q, debug=True)
                #     if res and q_valid:
                #         print("\n\n\n\n\n___________\n\n\n\n\n")
                #         Lg.log("ik.solve FOUND A SOLUTION", "OKGREEN")
                #         print("\n\n\n\n\n___________\n\n\n\n\n")
                #
                # if not res or not q_valid:
                #     dev = .5
                #     self.robosimian.setConfig(bias_config)
                #     res = ik.solve_nearby(end_config_iktargets, dev, activeDofs=project_constants.ACTIVE_DOFS)
                #     q = self.robosimian.getConfig()
                #     q_valid = config_valid(q, debug=False)
                #
                #     # if debug:
                #     #     dist = MathUtils._2d_euclidian_distance(q, torso_xy_pt)
                #     #     print(f"get_endconfigs() preset_loc_{i} ik.solve_nearby()\tres: {Lg.log_boolean(res)}, "
                #     #           f"config valid: {Lg.log_boolean(q_valid)}\t dist from specified torso_xy: {round(dist, 3)}")
                #     #     config_valid(q, debug=True)
                #
                #     if res and q_valid:
                #         print("\n\n\n\n\n___________\n\n\n\n\n")
                #         Lg.log("ik.solve FOUND A SOLUTION", "OKGREEN")
                #         print("\n\n\n\n\n___________\n\n\n\n\n")
                # end_config_iktargets.pop()  # NOTE: Important to pop() (to remove torso obj), as variable is reused for each EndcCnfig solve

                if visualize:
                    if not q_valid:
                        VisUtils.visualize_xy_point(q, f"q_{i}", color=VisUtils.RED, hm=self.height_map, height=.25)

                    else:
                        VisUtils.visualize_xy_point(q, f"q_{i}", color=VisUtils.GREEN, hm=self.height_map, height=.25)
                    visitems_to_clear.append(f"q_{i}")

                if res and q_valid:
                    return_configs.append(q)

            #  Grid search
            search_margin = .2
            x_min, y_min = intersection_centroid_xy[0] - search_margin, intersection_centroid_xy[1] - search_margin
            x_max, y_max = intersection_centroid_xy[0] + search_margin, intersection_centroid_xy[1] + search_margin
            break_early_min_cost_ratio = .65
            x_delta = .075
            y_delta = .075
            z_offsets = [0]

            lowest_cost_config, _, _, runtime, _, _, _, _ = end_config_exhaustive_search(
                x_min, y_min, x_max, y_max, x_delta, y_delta, z_offsets, curr_stance, torso_xy_inbound, self,
                config_valid, break_early_min_cost_ratio, end_config_iktargets, get_config_cost,
                with_rotation_R=torso_R_des, debug=search_debug, bias_config=bias_config)

            if lowest_cost_config:
                return_configs = [lowest_cost_config] + return_configs

                if visualize:
                    VisUtils.visualize_xy_point(lowest_cost_config, f"grid_search_xy", height=.35, color=VisUtils.GREEN)
                    visitems_to_clear.append(f"grid_search_xy")

        num_samples = project_constants.END_CONFIG_SAMPLE_COUNT

        if run_probabilistic_search:

            if project_constants.MPLANNER_VERBOSITY >= 3:
                print(f"get_endconfigs(): {len(return_configs)} configs saved, running randomized search centered at intersection centroid")
            max_dx = .3
            max_dy = .3
            max_dz = .2
            z_des = self.get_torso_z_des_from_xy(intersection_centroid_xy, curr_stance, debug=debug)
            intersection_centroid_xyz = [intersection_centroid_xy[0], intersection_centroid_xy[1], z_des]
            centered_search_res = randomized_centered_search(
                num_samples, intersection_centroid_xyz, max_dx, max_dy, max_dz, debug=search_debug, bias_config=bias_config)
            if centered_search_res:
                return_configs += [centered_search_res]

            if project_constants.MPLANNER_VERBOSITY >= 3:
                print(f"get_endconfigs(): {len(return_configs)} configs saved, running randomized search centered at "
                      f"support tri, line from incenter to moving leg intersection")
            z_des = self.get_torso_z_des_from_xy(support_tri_intersection_ptxy, curr_stance, debug=debug)
            support_tri_intersection_ptxyz = [support_tri_intersection_ptxy[0], support_tri_intersection_ptxy[1], z_des]
            centered_search_res = randomized_centered_search(
                num_samples, support_tri_intersection_ptxyz, max_dx, max_dy, max_dz, debug=search_debug, bias_config=bias_config)
            if centered_search_res:
                return_configs += [centered_search_res]

            if project_constants.MPLANNER_VERBOSITY >= 3:
                print(f"get_endconfigs(): {len(return_configs)} configs saved, rerunning previous search at lower z")
            support_tri_intersection_ptxyz[2] -= .15
            centered_search_res = randomized_centered_search(
                num_samples, support_tri_intersection_ptxyz, max_dx, max_dy, max_dz, debug=search_debug, bias_config=bias_config)
            if centered_search_res:
                return_configs += [centered_search_res]

            if project_constants.MPLANNER_VERBOSITY >= 3:
                print(f"get_endconfigs(): {len(return_configs)} configs saved, rerunning previous search at higher z")
            support_tri_intersection_ptxyz[2] += .3
            centered_search_res = randomized_centered_search(
            num_samples, support_tri_intersection_ptxyz, max_dx, max_dy, max_dz, debug=search_debug, bias_config=bias_config)
            if centered_search_res:
                return_configs += [centered_search_res]

        if run_exaustive_grid_search:
            if project_constants.MPLANNER_VERBOSITY >= 3:
                print(f"get_endconfigs(): {len(return_configs)} configs saved, running exhaustive grid search")

            p1 = support_tri.points[0]
            p2 = support_tri.points[1]
            p3 = support_tri.points[2]

            margin = .15
            x_min = margin + min(p1[0], p2[0], p3[0])
            x_max = -margin + max(p1[0], p2[0], p3[0])
            y_min = margin + min(p1[1], p2[1], p3[1])
            y_max = -margin + max(p1[1], p2[1], p3[1])

            VisUtils.visualize_line([x_min, y_min, .4], [x_max, y_min, .4], "l1", color=VisUtils.RED)
            VisUtils.visualize_line([x_max, y_min, .4], [x_max, y_max, .4], "l2", color=VisUtils.RED)
            VisUtils.visualize_line([x_max, y_max, .4], [x_min, y_max, .4], "l3", color=VisUtils.RED)
            VisUtils.visualize_line([x_min, y_max, .4], [x_min, y_min, .4], "l4", color=VisUtils.RED)

            break_early_min_cost_ratio = .65

            x_delta = .015
            y_delta = .015

            z_offsets = [0, 2*z_delta, -2*z_delta]

            lowest_cost_config, _, _, _, x_search_end, y_search_end, z_offset_end, broke_early = end_config_exhaustive_search(
                x_min, y_min, x_max, y_max, x_delta, y_delta, z_offsets, curr_stance,
                torso_xy_inbound, self, config_valid, break_early_min_cost_ratio, end_config_iktargets, get_config_cost,
                break_on_first_found_q=True, debug=search_debug, ignore_dist_to_nominal_z_cost=True
            )

            if lowest_cost_config:
                return_configs += [lowest_cost_config]


            if broke_early:

                if project_constants.MPLANNER_VERBOSITY >= 3:
                    print(f"Valid config found through exaustive_grid_search with x, y deltas: {round(x_delta, 3)}, {round(y_delta, 3)}")

                x_delta = .01
                y_delta = .01
                fine_search_w = .1
                z_offsets = [z_offset_end, z_offset_end + z_delta, z_offset_end - z_delta]

            else:
                x_delta = .02
                y_delta = .02
                fine_search_w = .25
                x_search_end = (x_max + x_min) / 2
                y_search_end = (y_max + y_min) / 2
                z_offsets = [-2*z_delta, -3*z_delta, z_delta, 2*z_delta,  3*z_delta]

            for z_offset in z_offsets:
                if project_constants.MPLANNER_VERBOSITY >= 3:
                    print(f"  searching at z_des + {round(z_offset, 3)}")

                lowest_cost_config,  _, _, _, _, _, _, _ = end_config_exhaustive_search(
                    x_search_end - fine_search_w, y_search_end - fine_search_w,
                    x_search_end + fine_search_w, y_search_end + fine_search_w,
                    x_delta, y_delta, [z_offset], curr_stance, torso_xy_inbound, self, config_valid,
                    break_early_min_cost_ratio, end_config_iktargets,
                    get_config_cost, debug=search_debug, ignore_dist_to_nominal_z_cost=True)
                if lowest_cost_config:
                    return_configs += [lowest_cost_config]
                    if project_constants.MPLANNER_VERBOSITY >= 3:
                        print(f"found valid config, {len(return_configs)} total return configs")

                # With Specified Torso Rotation
                lowest_cost_config,  _, _, _, _, _, _, _ = end_config_exhaustive_search(
                    x_search_end - fine_search_w, y_search_end - fine_search_w,
                    x_search_end + fine_search_w, y_search_end + fine_search_w,
                    x_delta, y_delta, [z_offset], curr_stance, torso_xy_inbound, self, config_valid,
                    break_early_min_cost_ratio, end_config_iktargets, get_config_cost, debug=search_debug,
                    with_rotation_R=torso_R_des, ignore_dist_to_nominal_z_cost=True)
                if lowest_cost_config:
                    return_configs += [lowest_cost_config]
                    if project_constants.MPLANNER_VERBOSITY >= 3:
                        print(f"found valid config, {len(return_configs)} total return configs")

            else:
                if project_constants.MPLANNER_VERBOSITY >= 3:
                    print("0 valid configs with coarse exaustive search")

        return_confg_costs = []
        for config in return_configs:
            return_confg_costs.append(get_config_cost(config)[0])
        sorted_return_configs = [x for _, x in sorted(zip(return_confg_costs, return_configs))]

        # print()
        # for config in sorted_return_configs:
        #     print("config =", config)
        #     print("  ", MotionPlanner.configs_j1s(config))
        #     get_config_cost(config, debug=True)

        return sorted_return_configs, visitems_to_clear

    def ik_arc(
            self, imax, q_valid_fn, arc_ik_targets, start_config, torso_motion_fn,
            leg_motion_fn=None, moving_leg_xyzRf=None, moving_leg_xyzR0=None, moving_end_effector=None,
            debug=False, step_height=None, sleep_t=0.0):

        start_t = time.time()
        arc_failed = False

        q_arc = [start_config]
        last_added_q = q_arc[0]
        max_j_ang_changes = []

        np.set_printoptions(suppress=True)

        def get_i_inc(x, c3):
            c1 = .95
            c2 = .09
            inc = 1 - c1*np.power(np.e, -c2*(x-c3))
            return inc

        exp_mu = None
        counter = 0
        i = 0

        # Note: planning fails when using NOMINALCONFIG/end_config as bias, too jittery

        while i < imax:

            self.robosimian.setConfig(last_added_q)
            ik_solver = IKSolver(self.robosimian)

            for goal in arc_ik_targets:
                ik_solver.add(goal)

            if leg_motion_fn:
                if step_height:
                    xyzR_i = leg_motion_fn(moving_leg_xyzR0, moving_leg_xyzRf, i, imax, step_height)
                else:
                    xyzR_i = leg_motion_fn(moving_leg_xyzR0, moving_leg_xyzRf, i, imax)
                ik_solver.add(ik.objective(moving_end_effector, R=xyzR_i[3], t=xyzR_i[0:3]))

            torso_xyz_i, torso_R_i = torso_motion_fn(i, imax)

            torso_obj = ik.objective(self.torso, R=torso_R_i, t=torso_xyz_i)

            ik_solver.add(torso_obj)
            ik_solver.setBiasConfig(last_added_q)
            # ik_solver.setBiasConfig(bias_config)

            # Get Joint Limits
            qmin, qmax = ConfigSpacePlanner.joint_limits(last_added_q)
            ik_solver.setJointLimits(qmin, qmax)

            res = ik_solver.solve()
            q = self.robosimian.getConfig()

            max_joint_dist = MathUtils.max_joint_angle_distance(q[7:], last_added_q[7:])
            config_valid = q_valid_fn(q)

            if debug:
                inc = get_i_inc(counter, exp_mu) if exp_mu else 1
                if debug:
                    print(f"{round(i, 2)}/{imax}, inc:{round(inc,3)}\t| ik res {Lg.log_boolean(res)}\t| q_valid "
                      f"{Lg.log_boolean(config_valid)}\t| max_joint_dist {round(max_joint_dist,4)}\t| stance == last addedq: ",q == last_added_q)
            time.sleep(sleep_t)

            if res and config_valid:
                if not q == last_added_q:
                    q_arc.append(q)
                    last_added_q = q
                    max_j_ang_changes.append(max_joint_dist)

            elif not res and config_valid and np.abs(max_joint_dist - ConfigSpacePlanner.max_allowed_joint_angle_change) < .1:

                if debug:
                    print("res: False, config_valid: True, max_joint_dist close to max allowable. rewinding a bit and lowering increment")

                rewind = False
                threshold = .5

                if exp_mu is None:
                    rewind = True
                    inc = 1.0
                else:
                    inc = get_i_inc(counter, exp_mu)
                    if inc > threshold:
                        rewind = True

                if rewind:
                    if debug:
                        print(f" inc: {round(inc,4)}, rewinding setting exp_mu to previous counter, subtracting from i by inc_current")
                    i -= inc
                    exp_mu = counter - 1

                else:
                    if debug:
                        print(f" current inc:{round(inc,4)} < {threshold}, failing")
                    arc_failed = True

            else:
                arc_failed = True

            # Catch rapidly changing joint angles
            if max_joint_dist > .45*ConfigSpacePlanner.max_allowed_joint_angle_change:
                exp_mu = counter
                if debug:
                    print(f"\tmax_joint_dist: {round(max_joint_dist, 4)} -> setting new exp_mu {exp_mu}")

            # Slow down task space translation if rapidly changing joint angles
            if exp_mu is None:
                i += 1
            else:
                inc = get_i_inc(counter, exp_mu)
                i += inc
                if i + .001 < 1:
                    if debug:
                        print(f" counter:{counter}, exp_mu: {exp_mu}, inc: {round(inc, 3)}")

            counter += 1
            if arc_failed:
                break

        return q_arc, arc_failed, time.time() - start_t

    def plan_legstep(self, r_pose, constraint_obj, stance_idx: int, visualize=False, only_use_sbl=False):

        debug = project_constants.MPLANNER_VERBOSITY >= 3

        return_configs = []

        fl_xyz, fr_xyz, br_xyz, bl_xyz = r_pose.fl, r_pose.fr, r_pose.br, r_pose.bl
        end_effectors = [self.fl_end_effector, self.fr_end_effector, self.br_end_effector, self.bl_end_effector]
        fl_obj = ik.objective(self.fl_end_effector, R=fl_xyz[3], t=fl_xyz[0:3])
        fr_obj = ik.objective(self.fr_end_effector, R=fr_xyz[3], t=fr_xyz[0:3])
        bl_obj = ik.objective(self.bl_end_effector, R=bl_xyz[3], t=bl_xyz[0:3])
        br_obj = ik.objective(self.br_end_effector, R=br_xyz[3], t=br_xyz[0:3])
        moving_leg = self.get_moving_leg_from_stance_idx(stance_idx)
        start_config = self.robosimian.getConfig()

        if debug:
            print("start_config=", start_config)
            print("  ", ConfigSpacePlanner.configs_j1s(start_config))

        self.debug_visualize_stance(stance_idx, constraint_obj, visualize=visualize, debug=debug)

        def torso_com_validator(q, ignore_range_tris=True):
            # self.robosimian.setConfig(q)
            # com = self.robosimian.getCom()
            # return constraint_obj.xy_in_support_area(com[0], com[1], ignore_range_tris=ignore_range_tris)
            return constraint_obj.xy_in_support_area(q[0], q[1], ignore_range_tris=ignore_range_tris)

        # Get MPlanner IK Constrains
        all_end_eff_constraints = [fl_obj, fr_obj, br_obj, bl_obj]
        m_planner_ik_constraints = [fl_obj, fr_obj, br_obj, bl_obj]
        m_planner_ik_constraints.pop(moving_leg - 1)

        # Create CSpace
        space = robotcspace.ClosedLoopRobotCSpace(self.robosimian, m_planner_ik_constraints, self.collider)
        # space.addConstraint(torso_com_validator)
        space.addConstraint(ConfigSpacePlanner.joint_angles_valid)
        space.setup()

        if space is None:
            Lg.log("Error: cspace setup failed.", "FAIL")
            return False

        def config_valid(q, debug=False):
            if not ConfigSpacePlanner.joint_angles_valid(q):
                if debug:
                    # print("!joint_angles_valid(stance)")
                    ConfigSpacePlanner.joint_angles_valid(q, debug=True)
                return False
            if not torso_com_validator(q):
                if debug:
                    print("!torso_com_validator(q)")
                return False
            if not space.inBounds(q):
                if debug: print("!space.inBounds(q)")
                return False
            if space.selfCollision(q):
                if debug: print("space.selfCollision(q)")
                return False
            if not space.closedLoop(q):
                if debug: print("!space.closedLoop(q)")
                return False
            if not space.isFeasible(q):
                if debug: print("!space.isFeasible(q)")
                return False
            return True

        if not space.closedLoop(start_config):

            max_torso_translation = ConfigSpacePlanner.max_torso_translation_dist
            if debug:
                print("space.closedLoop(start_config) is False. Entering hacky-fix function")

            torso_xyz = self.get_current_torso_xyz_yaw_deg()
            i = 0
            target_config = None

            while i < 300:
                rx = random.uniform(-max_torso_translation, max_torso_translation)
                ry = random.uniform(-max_torso_translation, max_torso_translation)
                rz = random.uniform(-max_torso_translation, max_torso_translation)
                t = [torso_xyz[0] + rx, torso_xyz[1] + ry, torso_xyz[2] + rz]
                offset_torso_obj = ik.objective(self.torso, local=[0, 0, 0], world=t)
                offset_ik_targets = [fl_obj, fr_obj, br_obj, bl_obj, offset_torso_obj]
                q = self.ik_solve(start_config, offset_ik_targets, config_valid, start_config)
                if q:
                    target_config = q
                    break
                i += 1
            if target_config is None:
                if debug:
                    Lg.log("Start config was invalid, unable to find nearby valid project_constants. Exiting", "FAIL")
                return False
            if debug:
                Lg.log(f"Warning: start config was infeasible, found new config with torso translated < {max_torso_translation}"
                           "to new position", "WARNING")
            print("MathUtils.max_joint_angle_distance(start_config, target_config):",
                  MathUtils.max_joint_angle_distance(start_config, target_config))
            start_config = target_config
            return_configs += [target_config]
            if debug:
                print("new_start_config=", start_config)

        pre_sleep_t = 0.0
        q_arc_sleep = 0.0
        ik_arcdebug = project_constants.MPLANNER_VERBOSITY >= 4

        if only_use_sbl:
            Lg.log("Warning: Only use SBL enabled", "WARNING")

        t0 = time.time()
        t_start = time.time()

        # -- Generate end configs from
        end_configs_1, visitems_to_clear = self.get_endconfigs(
            stance_idx, constraint_obj, fl_obj, fr_obj, bl_obj, br_obj, config_valid, start_config, True,
            run_preset_location_search=True,
            run_probabilistic_search=False,
            run_exaustive_grid_search=False,
            visualize=visualize, debug=debug)
        if debug:
            print(f"\n{len(end_configs_1)} end configs found in {Lg.log_bold(round(time.time() - t_start, 2))} seconds from 'preset_location_search'")

        t_start1 = time.time()
        ret = self.plan_legstep_on_end_configs_w_ikarc(
            start_config, end_effectors, moving_leg, r_pose,torso_com_validator, m_planner_ik_constraints,
            all_end_eff_constraints, end_configs_1, config_valid, debug=debug,ik_arcdebug=ik_arcdebug,
            q_arc_sleep=q_arc_sleep, pre_sleep_t=pre_sleep_t) if not only_use_sbl else []

        if ret:
            if debug:
                print(f"\nplan_legstep(): total plantime on endconfigs1: {Lg.log_bold(Lg.pp_double(time.time() - t_start1))} seconds")
            VisUtils.clear_visitems(visitems_to_clear)
            space.close()
            return ret
        if debug and len(end_configs_1) > 0:
            print(f"\nplan_legstep(): Failed to build a configuration path using end_configs_1 in "
                  f"{Lg.log_bold(Lg.pp_double(time.time() - t_start1))} seconds, querying new end configs and replanning\n")

        t_start = time.time()
        end_configs_2, visitems_to_clear2 = self.get_endconfigs(
            stance_idx, constraint_obj, fl_obj, fr_obj, bl_obj, br_obj, config_valid, start_config, True,
            visualize=visualize, debug=debug,
            run_preset_location_search=False,
            run_probabilistic_search=True,
            run_exaustive_grid_search=False)
        visitems_to_clear += visitems_to_clear2

        if debug:
            print(f"\nplan_legstep(): {len(end_configs_2)} end configs found in:",
                  Lg.log_bold(round(time.time() - t_start, 2)), "seconds from 'probabilistic_searches'")
        t_start1 = time.time()
        ret = self.plan_legstep_on_end_configs_w_ikarc(
            start_config, end_effectors, moving_leg, r_pose, torso_com_validator, m_planner_ik_constraints,
            all_end_eff_constraints, end_configs_2, config_valid, debug=debug, ik_arcdebug=ik_arcdebug,
            q_arc_sleep=q_arc_sleep, pre_sleep_t=pre_sleep_t)  if not only_use_sbl else []
        if ret:
            if debug:
                print(f"\nplan_legstep(): total plantime on endconfigs2: {Lg.log_bold(Lg.pp_double(time.time() - t_start1))} seconds")
            VisUtils.clear_visitems(visitems_to_clear)
            space.close()
            return ret

        t_start1 = time.time()
        ret = self.plan_motion_on_end_configs_w_sbl(start_config, end_configs_1 + end_configs_2, space, config_valid, debug=debug)
        if ret:
            if debug:
                print(f"\nplan_legstep(): total sbl plantime: {Lg.log_bold(Lg.pp_double(time.time() - t_start1))} seconds, returning")
            VisUtils.clear_visitems(visitems_to_clear)
            space.close()
            return ret
        if debug:
            print(f"plan_legstep(): Failed to build a configuration path with end_configs_(1|2) in "
                  f"{Lg.log_bold(Lg.pp_double(time.time() - t0))} seconds, generating more end configs w/ exhaustive search, then running sbl")

        t_start = time.time()
        end_configs_3, visitems_to_clear3 = self.get_endconfigs(
            stance_idx, constraint_obj, fl_obj, fr_obj, bl_obj, br_obj, config_valid, start_config, True,
            visualize=visualize, debug=debug,
            run_preset_location_search=False,
            run_probabilistic_search=False,
            run_exaustive_grid_search=True)
        visitems_to_clear += visitems_to_clear3
        if debug:
            print(f"\nplan_legstep(): {len(end_configs_3)} end configs found in:", Lg.log_bold(Lg.pp_double(time.time() - t_start)), "seconds from 'exaustive_grid_search'")
        t_start1 = time.time()
        ret = self.plan_legstep_on_end_configs_w_ikarc(
            start_config, end_effectors, moving_leg, r_pose, torso_com_validator, m_planner_ik_constraints,
            all_end_eff_constraints, end_configs_3, config_valid, debug=debug, ik_arcdebug=ik_arcdebug,
            q_arc_sleep=q_arc_sleep, pre_sleep_t=pre_sleep_t) if not only_use_sbl else []

        if ret:
            if debug:
                print(f"\nplan_legstep(): total plantime on endconfigs3: {Lg.log_bold(Lg.pp_double(time.time() - t_start1))} seconds")
            VisUtils.clear_visitems(visitems_to_clear)
            space.close()
            return ret

        if debug:
            print(f"plan_legstep(): Failed to build a configuration path with end_configs_(1|2|3) in {Lg.log_bold(Lg.pp_double(time.time() - t0))} seconds, running sbl")

        t_start1 = time.time()
        ret = self.plan_motion_on_end_configs_w_sbl(start_config, end_configs_3, space, config_valid, debug=debug)
        if ret:
            if debug:
                print(f"\nplan_legstep(): total sbl plantime: {Lg.log_bold(Lg.pp_double(time.time() - t_start1))} seconds, returning")
            VisUtils.clear_visitems(visitems_to_clear)
            space.close()
            return ret
        if debug:
            print(
                f"plan_legstep(): Failed to build a configuration path with sbl on end_configs_3 in {Lg.log_bold(Lg.pp_double(time.time() - t0))} seconds, exiting")

        VisUtils.clear_visitems(visitems_to_clear)
        return False

    # TODO: Investigate whether this is working as intended. Returned list is only ~8 long for leg steps
    def plan_motion_on_end_configs_w_sbl(self, start_config, end_configs, space, config_valid, debug=False):

        '''
            "To plan, call planMore(iters) until getPath(0,1) returns non-NULL. The return
            value is a list of configurations."

            SBL is a tree-based motion planner that attempts to grow two trees at once: one grows from the starting state
            and the other from the goal state. The tree expansion strategy is the same as for EST. Attempts are made to
            connect these trees at every step of the expansion. If they are connected, a solution path is obtained.
            However, this solution path is not certain to be valid (the lazy part of the algorithm) so it is checked for
            validity. If invalid parts are found, they are removed from the tree and exploration of the state space
            continues until a solution is found. To guide the exploration, an additional grid data structure is maintained.
            Grid cells contain states that have been previously visited. When deciding which state to use for further expansion,
            this grid is used; least-filled grid cells have most chances of being selected. The grid is usually imposed on
             a projection of the state space. This projection needs to be set before using the planner (setProjectionEvaluator() function).
             Connection of states in different trees is attempted if they fall in the same grid cell. If no projection is set,
             the planner will attempt to use the default projection associated to the state space. An exception is thrown
             if no default projection is available either.
        '''

        for end_config_i in range(len(end_configs)):
            end_config = end_configs[end_config_i]
            t_start = time.time()
            try:
                algo = project_constants.KLAMPT_MPLANNER_ALGO
                settings = {'type': algo, 'perturbationRadius': 0.5, 'bidirectional': 1, 'shortcut': 0, 'restart': 0,
                            'restartTermCond': "{foundSolution:1, maxIters:1000}"}
                plan = cspace.MotionPlan(space)
                plan.setEndpoints(start_config, end_config)
                plan.setOptions(**settings)
            except RuntimeError as e:
                if debug:
                    Lg.log(f"Error: planner setup failed. Caught runtime exception: '{e}'", "FAIL")
                continue

            if plan is None:
                if debug:
                    Lg.log("Error: planner setup failed.", "FAIL")
                continue

            if not algo == "sbl":
                Lg.log("search algorithm not implemented, exiting", "FAIL")
                plan.space.close()
                plan.close()
                return

            path = None
            i = 0
            while path is None and i < 100:
                plan.planMore(25)
                path = plan.getPath()

                i += 1
                # print(f"\n{i}| path is none:{path is None}")
                # stats = plan.getStats()
                # for stat in stats:
                #     print(f"{stat}: {stats[stat]}")

            if path:
                discretized_path = space.discretizePath(path)
                discretized_path_valid = ConfigSpacePlanner.path_valid(discretized_path, config_valid, _debug=False)
                if discretized_path_valid:
                    plan.close()
                    if debug:
                        print(f"Built valid path with {project_constants.KLAMPT_MPLANNER_ALGO} "
                              f"for end-config {end_config_i + 1}/{len(end_configs)} in {Lg.bold_txt(round(time.time() - t_start, 3))} seconds")
                    return discretized_path

                if debug:
                    print(f"discretizedPath for path built by {project_constants.KLAMPT_MPLANNER_ALGO} "
                          f"for end-config {end_config_i + 1}/{len(end_configs)} is invalid. search and discretization completed "
                          f"in {Lg.bold_txt(round(time.time() - t_start, 3))} seconds")
            else:
                if debug:
                    print(f"Failed to build a path with {project_constants.KLAMPT_MPLANNER_ALGO} "
                          f"for end-config {end_config_i + 1}/{len(end_configs)} in {Lg.bold_txt(round(time.time() - t_start, 3))} seconds")
            plan.close()
            if debug:
                print(f"{project_constants.KLAMPT_MPLANNER_ALGO} search failed for end-config {end_config_i + 1}/"
                      f"{len(end_configs)} in {Lg.bold_txt(round(time.time()-t_start,2))} seconds")
        return False

    def plan_legstep_on_end_configs_w_ikarc(
            self, start_config, end_effectors, moving_leg, r_pose, torso_com_validator, m_planner_ik_constraints,
            all_end_eff_constraints, end_configs, config_valid, debug=False, ik_arcdebug=False, q_arc_sleep=0.0,
            pre_sleep_t=0.0):

        if len(end_configs) == 0:
            return False

        moving_end_effector = end_effectors[moving_leg-1]
        self.robosimian.setConfig(start_config)
        torso_R0 = self.torso.getTransform()[0]
        torso_xyz0 = start_config[0:3]

        # Get moving leg, torso starting position, rotations
        self.robosimian.setConfig(end_configs[0])
        moving_endeff_xyzRf = self.get_end_effector_current_xyzRs()[moving_leg - 1]
        moving_endeff_xyzR0 = r_pose.get_moving_leg_xyzR(moving_leg)
        max_obst_height_in_path = -np.inf
        r_world = project_constants.HOOK_LENGTH + project_constants.END_AFFECTOR_RADIUS
        for i in range(50):
            xy_i = self.get_parabolic_mid_motion_xyzR(moving_endeff_xyzR0, moving_endeff_xyzRf, i, 50, project_constants.STEP_HEIGHT)[0:2]
            tallest_obs_height = self.height_map.max_in_radius_r_centered_at_xy(xy_i[0], xy_i[1], r_world)
            max_obst_height_in_path = max(max_obst_height_in_path, tallest_obs_height)

        margin = .2
        translation_height = max_obst_height_in_path + margin
        upward_pct_time = .2
        translation_pct_time = 1 - 2 * upward_pct_time

        def rectangular_path_end_eff_xyzR(_endeff_xyzR_0, _endeff_xyzR_f, _i, _imax):
            _endeff_xyzR_i = MathUtils.mid_motion_rotation_matrix(_endeff_xyzR_0[3], _endeff_xyzR_f[3], _i, _imax)
            if _i / _imax <= upward_pct_time:
                starting_z = _endeff_xyzR_0[2]
                pct_raised = (_i / _imax) / upward_pct_time
                delta_z = (translation_height - starting_z) * pct_raised
                ret = [_endeff_xyzR_0[0], _endeff_xyzR_0[1], _endeff_xyzR_0[2] + delta_z, _endeff_xyzR_i]
            elif upward_pct_time < _i / _imax < 1 - upward_pct_time:
                start = [_endeff_xyzR_0[0], _endeff_xyzR_0[1], translation_height]
                end = [_endeff_xyzR_f[0], _endeff_xyzR_f[1], translation_height]
                k = _i - upward_pct_time * _imax
                kmax = _imax - 2 * _imax * upward_pct_time
                xyz = self.get_linear_3d_mid_motion_vals(start, end, k, kmax)
                ret = xyz + [_endeff_xyzR_i]
            else:
                starting_z = translation_height
                pct_lowered = ((_i / _imax) - upward_pct_time - translation_pct_time) / upward_pct_time
                delta_z = (_endeff_xyzR_f[2] - starting_z) * pct_lowered
                z_des = starting_z + delta_z
                ret = [_endeff_xyzR_f[0], _endeff_xyzR_f[1], z_des, _endeff_xyzR_i]
            return ret

        for end_config_i in range(len(end_configs)):

            end_config = end_configs[end_config_i]

            if debug:
                print(f"\n -- Planning with end config {end_config_i+1}/{len(end_configs)}")

            if not torso_com_validator(end_config):
                Lg.log("Error: end config not in torso com", "FAIL")
                return

            # Get moving leg, torso final position, rotations
            self.robosimian.setConfig(end_config)
            moving_endeff_xyzRf = self.get_end_effector_current_xyzRs()[moving_leg-1]
            torso_xyz_f = end_config[0:3]
            torso_Rf = self.torso.getTransform()[0]

            arc_ik_targets = m_planner_ik_constraints[:]

            q_arc_debug = ik_arcdebug
            imax = 125
            # imax = 100

            def linear_torso_motion(i, imax):
                torso_xyz_i = self.get_linear_3d_mid_motion_vals(torso_xyz0, torso_xyz_f, i, imax)
                torso_R_i = MathUtils.mid_motion_rotation_matrix(torso_R0, torso_Rf, i, imax)
                return torso_xyz_i, torso_R_i

            # starting_torso_xyz, starting_torso_R, torso_xyz_f, torso_R_f
            # torso_xyz0, torso_R0, torso_xyz_f, torso_Rf

            # ______________________________________________ Path 1: Torso Shift with End Eff. Parabolic Arc

            start_t = time.time()
            self.robosimian.setConfig(start_config)
            time.sleep(pre_sleep_t)
            q_arc, arc_failed, runtime = self.ik_arc(
                imax, config_valid, arc_ik_targets, start_config, linear_torso_motion,
                leg_motion_fn=self.get_parabolic_mid_motion_xyzR, moving_leg_xyzR0=moving_endeff_xyzR0,
                moving_leg_xyzRf=moving_endeff_xyzRf, moving_end_effector=moving_end_effector,
                step_height=project_constants.STEP_HEIGHT,
                debug=q_arc_debug, sleep_t=q_arc_sleep
            )

            path = q_arc
            if not arc_failed:
                if debug:
                    print(f"Parabolic arc generated in {Lg.styled_text(str( round(time.time() - start_t, 2)), 'bold')} seconds, returning")
                return path
            if debug:
                print(f"Parabolic arc {Lg.fail('failed')} in {Lg.styled_text(str( round(time.time() - start_t, 2)), 'bold')} seconds")

            # ______________________________________________ Path 2: Torso Shift with Rectangular Arc

            time.sleep(pre_sleep_t)
            start_t = time.time()
            # imax = 100
            self.robosimian.setConfig(start_config)
            q_arc, arc_failed, runtime = self.ik_arc(
                imax, config_valid, arc_ik_targets, start_config, linear_torso_motion,
                leg_motion_fn=rectangular_path_end_eff_xyzR, moving_leg_xyzR0=moving_endeff_xyzR0, moving_leg_xyzRf=moving_endeff_xyzRf,
                moving_end_effector=moving_end_effector,
                debug=q_arc_debug, sleep_t=q_arc_sleep
            )

            path = q_arc
            if not arc_failed:
                if debug:
                    print(f"Rectangular arc generated in {Lg.styled_text(str( round(time.time() - start_t, 2)), 'bold')} seconds, returning")
                return path
            if debug:
                print(f"Rectangular arc {Lg.fail('failed')} in {Lg.styled_text(str(round(time.time() - start_t, 2)), 'bold')} seconds")

            # ______________________________________________ Path 3: Rectangular Arc, then Torso Shift

            time.sleep(pre_sleep_t)
            start_t = time.time()
            # imax = 100
            self.robosimian.setConfig(start_config)
            q_arc1, arc_failed, runtime = self.ik_arc(
                imax, config_valid, arc_ik_targets, start_config, linear_torso_motion,
                leg_motion_fn=rectangular_path_end_eff_xyzR, moving_leg_xyzR0=moving_endeff_xyzR0, moving_leg_xyzRf=moving_endeff_xyzRf,
                moving_end_effector=moving_end_effector, debug=q_arc_debug,  sleep_t=q_arc_sleep)

            if not arc_failed:
                ending_q_from_leg_step = q_arc1[len(q_arc1)-1]
                self.robosimian.setConfig(ending_q_from_leg_step)
                moving_leg_Tt = moving_end_effector.getTransform()
                moving_leg_R = moving_leg_Tt[0]
                moving_leg_t = moving_leg_Tt[1]
                moving_end_eff_ik_constraint = ik.objective(end_effectors[moving_leg - 1], R=moving_leg_R, t=moving_leg_t)
                ik_constrains = arc_ik_targets + [moving_end_eff_ik_constraint]
                q_arc2, arc_failed, runtime = self.ik_arc(
                    imax, config_valid, ik_constrains, ending_q_from_leg_step, linear_torso_motion,
                    debug=q_arc_debug
                )

                if not arc_failed:
                    if debug:
                        print(f"End eff., then torso motion path built in {Lg.styled_text(str( round(time.time() - start_t, 2)), 'bold')} seconds, returning")
                        return q_arc1 + q_arc2
            if debug:
                print(f"End eff., then torso motion path {Lg.fail('failed')} in {Lg.styled_text(str(round(time.time() - start_t, 2)), 'bold')} seconds")


            # ______________________________________________ Path 4: Torso Shift then Rectangular Arc
            # imax = 200
            time.sleep(pre_sleep_t)
            start_t = time.time()
            self.robosimian.setConfig(start_config)
            q_arc1, arc_failed, runtime = self.ik_arc(
                imax, config_valid, all_end_eff_constraints, start_config, linear_torso_motion,
                debug=q_arc_debug, sleep_t=q_arc_sleep)
            if not arc_failed:
                ending_q_from_leg_step = q_arc1[len(q_arc1)-1]
                self.robosimian.setConfig(ending_q_from_leg_step)
                q_arc2, arc_failed, runtime = self.ik_arc(
                    imax, config_valid, arc_ik_targets, ending_q_from_leg_step, linear_torso_motion,
                    leg_motion_fn=rectangular_path_end_eff_xyzR, moving_leg_xyzR0=moving_endeff_xyzR0,
                    moving_leg_xyzRf=moving_endeff_xyzRf,
                    moving_end_effector=moving_end_effector, debug=q_arc_debug, sleep_t=q_arc_sleep)

                if not arc_failed:
                    if debug:
                        print(f"Torso motion, then rectangular arc built in {Lg.styled_text(str( round(time.time() - start_t, 2)), 'bold')} seconds, returning")
                    return q_arc1 + q_arc2
            if debug:
                print(f"Torso motion, then rectangular arc {Lg.fail('failed')} in {Lg.styled_text(str(round(time.time() - start_t, 2)), 'bold')} seconds")

        return False

    def plan_torsoshift(self, r_pose, constraint_obj: Constraints, stance_idx, visualize=False, only_use_sbl=False):

        debug = project_constants.MPLANNER_VERBOSITY >= 3

        # Function wide variables
        fl_xyz, fr_xyz, br_xyz, bl_xyz = r_pose.fl, r_pose.fr, r_pose.br, r_pose.bl
        fl_obj = ik.objective(self.fl_end_effector, R=fl_xyz[3], t=fl_xyz[0:3])
        fr_obj = ik.objective(self.fr_end_effector, R=fr_xyz[3], t=fr_xyz[0:3])
        bl_obj = ik.objective(self.bl_end_effector, R=bl_xyz[3], t=bl_xyz[0:3])
        br_obj = ik.objective(self.br_end_effector, R=br_xyz[3], t=br_xyz[0:3])
        start_config = self.robosimian.getConfig()

        self.debug_visualize_stance(stance_idx, constraint_obj, visualize=visualize, debug=debug)

        if debug:
            print("start_config =",start_config)
            print("  ", ConfigSpacePlanner.configs_j1s(start_config))

        # Get MPlanner IK Constrains
        m_planner_ik_constraints = [fl_obj, fr_obj, br_obj, bl_obj]

        # Create CSpace
        space = robotcspace.ClosedLoopRobotCSpace(self.robosimian, m_planner_ik_constraints, self.collider)
        space.addConstraint(ConfigSpacePlanner.joint_angles_valid)
        space.setup()
        if space is None:
            Lg.log("Error: cspace setup failed.", "FAIL")
            return False

        def config_valid(q, debug=False):
            if not ConfigSpacePlanner.joint_angles_valid(q): # repeated, included for debugging
                if debug:
                    print("!joint_angles_valid(q)")
                    ConfigSpacePlanner.joint_angles_valid(q, debug=True)
                return False
            if not space.inBounds(q):
                if debug: print("!space.inBounds(q)")
                return False
            if space.selfCollision(q):
                if debug: print("space.selfCollision(q)")
                return False
            if not space.closedLoop(q):
                if debug: print("!space.closedLoop(q)")
                return False
            if not space.isFeasible(q):
                if debug: print("!space.isFeasible(q)")
                return False
            return True

        if not config_valid(start_config):
            if debug:
                print("Error, start config is not valid")
                config_valid(start_config, debug=True)
            return False

        if not space.closedLoop(start_config):
            if debug:
                print("space.closedLoop(start_config) is  false. Entering hacky-fix function")
            torso_xyz = self.get_current_torso_xyz_yaw_deg()
            i = 0
            max_torso_translation = ConfigSpacePlanner.max_torso_translation_dist
            target_config = None
            while i < 200:
                rx = random.uniform(-max_torso_translation, max_torso_translation)
                ry = random.uniform(-max_torso_translation, max_torso_translation)
                rz = random.uniform(-max_torso_translation, max_torso_translation)
                t = [torso_xyz[0] + rx, torso_xyz[1] + ry, torso_xyz[2] + rz]
                offset_torso_obj = ik.objective(self.torso, local=[0, 0, 0], world=t)
                offset_ik_targets = [fl_obj, fr_obj, br_obj, bl_obj, offset_torso_obj]
                q = self.ik_solve(start_config, offset_ik_targets, config_valid, start_config)
                if q:
                    target_config = q
                    break
                i += 1

            if target_config is None:
                if debug:
                    Lg.log("Unable to find a valid start project_constants. Exiting", "FAIL")
                return False

            if debug:
                Lg.log(f"Warning: start config was infeasible, found a new torso position within old {max_torso_translation} of the "
                           "old one that is valid. May produce jump", "WARNING")
            start_config = target_config

        def plan_given_end_configs(end_configs):

            if len(end_configs) == 0:
                return

            # Try discritizePath(q0, qf)
            for end_config_i in range(len(end_configs)):
                end_config = end_configs[end_config_i]

                if debug:
                    print(f"\n -- Planning config path given end config {end_config_i + 1}/{len(end_configs)}")

                t_start = time.time()
                path = space.discretizePath([start_config, end_config])
                if path is not None and len(path) > 2:
                    valid = ConfigSpacePlanner.path_valid(path, config_valid, _debug=False)
                    if valid:
                        if debug:
                            print(f"Valid torso motion built between qo, qf with discretizedPath(q0,qf) in:",
                                  round(time.time() - t_start, 2), f"seconds")
                        return path
                if debug:
                    print(f"{Lg.fail('Failed')} to build path between q0, qf with discretizePath() for end config"
                          f"{end_config_i+1}/{len(end_configs)} in ({round(time.time() - t_start, 2)} seconds)")

            # def linear_torso_motion(i, imax):/
                # torso_xyzR0[0:3], torso_xyzR0[3], torso_xyzRf[0:3], torso_xyzRf[3],

            def linear_torso_motion(i, imax):
                torso_xyz_i = self.get_linear_3d_mid_motion_vals(torso_xyzR0[0:3], torso_xyzRf[3], i, imax)
                torso_R_i = MathUtils.mid_motion_rotation_matrix(torso_xyzR0[3], torso_xyzRf[3], i, imax)
                return torso_xyz_i, torso_R_i

            for end_config_i in range(len(end_configs)):
                end_config = end_configs[end_config_i]

                t_start = time.time()

                imax = 150
                torso_xyzR0 = self.torso.getTransform()[1] + [self.torso.getTransform()[0]]
                self.robosimian.setConfig(end_config)
                torso_xyzRf = self.torso.getTransform()[1] + [self.torso.getTransform()[0]]
                self.robosimian.setConfig(start_config)
                arc_ik_targets = m_planner_ik_constraints[:]

                q_arc, arc_failed, runtime = self.ik_arc(
                    imax, config_valid, arc_ik_targets, start_config, linear_torso_motion, debug=False
                )
                path = q_arc

                if not arc_failed:
                    # valid = MotionPlanner.path_valid(path, config_valid, _debug=False)
                    valid = True
                    if valid:
                        if debug:
                            print(f"Valid torso motion found between qo, qf with ik arc for end config "
                                  f"{end_config_i+1}/{len(end_configs)} in:", round(time.time() - t_start, 2),f"seconds, returning")

                        return path
                    else:
                        if debug:
                            print(f"{Lg.fail('Failed')} to build path between q0, qf with ik arc for {end_config_i + 1}/"
                                  f"{len(end_configs)} in ({round(time.time() - t_start, 2)} seconds)")

        if only_use_sbl:
            Lg.log("Warning: Only use SBL enabled", "WARNING")

        # _____________________ Generate, plan on End Configs1
        visitems_to_clear = []
        t0 = time.time()
        ec1_t0 = time.time()
        end_configs_1, visitems_to_clear = self.get_endconfigs(
            stance_idx, constraint_obj, fl_obj, fr_obj, bl_obj, br_obj, config_valid, start_config, False,
            run_preset_location_search=True,
            run_probabilistic_search=False,
            run_exaustive_grid_search=False,
            visualize=visualize, debug=debug)
        if debug:
            print(f"\nplan_torsomotion(): {len(end_configs_1)} configs found with 'run_preset_location_search' "
                  f"in {Lg.log_bold(round(time.time() - ec1_t0, 2))} seconds")

        ec1p_t0 = time.time()
        ret = plan_given_end_configs(end_configs_1) if not only_use_sbl else []
        if ret:
            if debug:
                print(f"\nplan_torsomotion(): total plantime on endconfigs1: {Lg.log_bold(Lg.pp_double(time.time() - ec1p_t0))} seconds")
            VisUtils.clear_visitems(visitems_to_clear); space.close()
            return ret
        if debug and len(end_configs_1) > 0:
            print(f"\nplan_torsomotion(): Failed to build a configuration path using end_configs_1 in ",
                  Lg.pp_double(time.time() - ec1p_t0), "seconds")

        # _____________________ Generate, plan on End Configs2
        ec2_t0 = time.time()
        # end_configs_2, visitems_to_clear2 = [], []
        end_configs_2, visitems_to_clear2 = self.get_endconfigs(
            stance_idx, constraint_obj, fl_obj, fr_obj, bl_obj, br_obj, config_valid, start_config, False,
            visualize=visualize, debug=debug,
            run_preset_location_search=False,
            run_probabilistic_search=True,
            run_exaustive_grid_search=False,)
        visitems_to_clear += visitems_to_clear2

        if debug:
            print(f"\nplan_torsomotion(): {len(end_configs_2)} configs found with 'run_probabilistic_searches' in:",
                  Lg.log_bold(Lg.pp_double(time.time() - ec2_t0)), "seconds")
        ec1p_t0 = time.time()

        if len(end_configs_2) > 0:
            ret = plan_given_end_configs(end_configs_2) if not only_use_sbl else []
            if ret:
                if debug:
                    print(f"\nplan_torsomotion(): total plantime on endconfigs2: {Lg.log_bold(Lg.pp_double(time.time() - ec1p_t0))} seconds")
                VisUtils.clear_visitems(visitems_to_clear); space.close()
                return ret

        # -- Run sbl on end configs 1,2
        if len(end_configs_1 + end_configs_2) > 0:
            sbls1_t0 = time.time()
            ret = self.plan_motion_on_end_configs_w_sbl(start_config, end_configs_1 + end_configs_2, space, config_valid, debug=debug)
            if ret:
                if debug:
                    print(f"\nplan_torsomotion(): total sbl plantime: {Lg.log_bold(Lg.pp_double(time.time()-sbls1_t0))} seconds, returning")
                VisUtils.clear_visitems(visitems_to_clear); space.close()
                return ret
            if debug:
                print(f"plan_torsomotion(): Failed to build a configuration path with end_configs_(1|2) in "
                      f"{Lg.log_bold(Lg.pp_double(time.time() - t0))} seconds, running sbl")

        # _____________________ Generate, plan on End Configs3
        t_start = time.time()
        end_configs_3, visitems_to_clear3 = self.get_endconfigs(
            stance_idx, constraint_obj, fl_obj, fr_obj, bl_obj, br_obj, config_valid, start_config, False,
            visualize=visualize, debug=debug, run_preset_location_search=False, run_probabilistic_search=False,
            run_exaustive_grid_search=True)
        visitems_to_clear += visitems_to_clear3
        if debug:
            if len(end_configs_3) > 0:
                print(f"\nplan_torsomotion(): {len(end_configs_3)} configs found with 'run_grid_searches' in:", Lg.log_bold(Lg.pp_double(time.time() - t_start)), "seconds")
            else:
                print(f"\nplan_torsomotion(): 0 configs found with 'run_grid_searches' in:", Lg.log_bold(Lg.pp_double(time.time() - t_start)), "seconds, returning Failure")

        if len(end_configs_3) > 0:
            # t_start1 = time.time()
            ec3_t0 = time.time()
            ret = plan_given_end_configs(end_configs_3)  if not only_use_sbl else []
            if ret:
                if debug:
                    print(f"\nplan_torsomotion(): total plantime on endconfigs3: {Lg.log_bold(Lg.pp_double(time.time() - ec3_t0))} seconds")
                VisUtils.clear_visitems(visitems_to_clear); space.close()
                return ret
            if debug:
                print(f"plan_torsomotion(): Failed to build a configuration path with end_configs_(1|2|3) in "
                      f"{Lg.log_bold(Lg.pp_double(time.time() - ec3_t0))} seconds, running sbl")

            sbls3_t0 = time.time()
            ret = self.plan_motion_on_end_configs_w_sbl(start_config, end_configs_3, space, config_valid, debug=debug)
            if ret:
                if debug:
                    print(f"\nplan_torsomotion(): total sbl plantime: {Lg.log_bold(Lg.pp_double(time.time()- sbls3_t0))} seconds, returning")
                VisUtils.clear_visitems(visitems_to_clear); space.close()
                return ret
            if debug:
                print(f"plan_torsomotion(): Failed to build a configuration path with end_configs_3 in "
                      f"{Lg.log_bold(Lg.pp_double(time.time() - sbls3_t0))} seconds, returning Failure")

        return False