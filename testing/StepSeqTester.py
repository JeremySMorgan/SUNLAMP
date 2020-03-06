import time
#from src.generators.StepSequencePlanner.FootStepSequenceGenerator import FootStepSequenceGenerator
from src.generators.step_sequence_planner import StepSequencePlanner
import klampt
from klampt import vis
from src.utils import config

class StepSequencePlannerTester:

    def __init__(self, height_map, scatter_obj, hl_traj_obj, fs_cost_map, xy_yaw0, xy_yawf, r_poser):
        self.r_poser = r_poser
        self.hl_traj = hl_traj_obj
        self.fs_cost_map = fs_cost_map
        self.height_map = height_map

        self.fs_scatter_obj = scatter_obj
        self.fs_scatter = scatter_obj.get_scatter_list()

        self.xy_yawf = xy_yawf
        self.start_state = (len(self.fs_scatter) - 4, len(self.fs_scatter) - 3, len(self.fs_scatter) - 2, len(self.fs_scatter) - 1, 1)

        self.step_seq_planner = StepSequencePlanner(
            config, height_map, scatter_obj, hl_traj_obj, fs_cost_map, xy_yaw0, xy_yawf,
            r_poser=self.r_poser, debug=True, vis_successors=True, deep_debug_visualization=True)

    def test_height_around_endeff_costfn(self):
        br_xyz = (8.759999999999984, 0.48, 0.0)
        ret = self.step_seq_planner.height_around_endeff_costfn(br_xyz[0], br_xyz[1], debug=True)

        sphere = klampt.GeometricPrimitive()
        sphere.setSphere([br_xyz[0], br_xyz[1], .1], config.STEPSEQ_HEIGHT_AROUND_ENDEFF_R)
        vis.add("sphere", sphere)
        vis.setColor("sphere", 0, 0, 1, 0.5)

        print(ret)

    def state_is_kinematically_valid(self):

        # q_prev = (2862, 2982, 2171, 2412, 1)
        q = (6931, 6932, 209, 6934, 2)
        q_next = (6931, 643, 209, 6934, 4)

        cost = self.step_seq_planner.get_cost(q, q_next, force_vis=True)
        # time.sleep(5)
        #
        # self.r_poser.set_q(stance)
        # ret =  self.step_seq_planner.state_is_kinematically_valid( q_prev, stance, visualize=True, verbose_debug=True)
        # time.sleep(3)

        # for i in [620]:
        #     for j in range(25):
        #         for n in [1, -1]:
        #             _q = (6931, i + n*j, 209, 6934, 4)
        #             print(_q)
        #             self.r_poser.set_q(_q)
        #             time.sleep(.7)

        for i in range(2):
            self.r_poser.set_q(q)
            time.sleep(3)
            self.r_poser.set_q(q_next)
            time.sleep(3)

        self.r_poser.set_q(q)
        ret = self.step_seq_planner.state_is_kinematically_valid(q, q_next, visualize=True, verbose_debug=True)

        cost = self.step_seq_planner.get_cost(q, q_next, force_vis=True)
        print("stance is kinematically valid:", ret)

    def test_is_goal(self):
        goals = [(0,1,2,3,1), (-1,3,4,53,5), (-1.0,23,53,53,4)]
        for goal in goals:
            is_goal = self.step_seq_planner.is_goal(goal)
            print("for stance:",goal,"isgoal:",is_goal)

    def test_get_cost_from_q_to_qnext(self, q, qnext):
        print("\nin test_get_cost() - setting current stance to:",q)
        self.r_poser.set_q(q, debug=False)
        cost = self.step_seq_planner.get_cost(q, qnext, debug=False)
        print("cost to qnext:",qnext,":",cost)

    def test_successors(self):

        # sleep_t = 3
        # r = 20
        # print("\nin Test Successors. Starting stance:", self.start_state)

        q0 = (6931, 6932, 208, 6934, 2)

        successors, successor_costs = self.step_seq_planner.successors(q0)

        # for _ in range(r):
        #     if len(successors) ==0: break
        #     lowest_cost_successor = successors[successor_costs.index(min(successor_costs))]
        #     q1 = lowest_cost_successor
        #     self.r_poser.set_q(lowest_cost_successor)
        #     print("lowest cost successor:", lowest_cost_successor, " cost:", min(successor_costs))
        #     self.step_seq_planner.get_cost(q0, q1, klampt_vis_enabled=klampt_vis_enabled,debug=False)
        #
        #     time.sleep(sleep_t)
        #     successors, successor_costs = self.step_seq_planner.successors(lowest_cost_successor)
        #     q0 = q1

        print("\ntest_successors done \n")

    def test_clear_visited(self):
        pass

    def test_visit(self):
        pass

    def test_visited_state_node(self):
        pass

    def test_heuristic(self):
        pass

    # ------------- Helper Functions

    def test_get_xy_yaw_error_from_goal(self):
        pass

    def test_get_torso_xy_yaw(self):
        pass

    def test_get_next_leg_to_move(self):
        pass

    def test_any_two_equal(self):
        pass

    def test_get_end_affectors_xyzc(self):
        pass

    def test_state_is_kinematically_valid(self):
        pass

    def tset_get_best_fitting_smoothed_traj_idx(self):
        pass

    def test_get_cost(self):

        sleep_t = 3
        steps = 30
        t_start = time.time()
        next_lowest_qs = self.get_n_lowest_cost_next_qs(steps)
        print("found:",steps,"in",time.time() - t_start,"s")

        self.r_poser.set_q(self.start_state)
        for i in range( len(next_lowest_qs)-1 ):
            q_0 =  next_lowest_qs[i]
            q_f = next_lowest_qs[i+1]
            print("Setting Q(",i,"):",q_f)
            self.step_seq_planner.get_cost(q_0, q_f, klampt_vis_enabled=True)
            self.r_poser.set_q(q_f)
            time.sleep(sleep_t)

    def get_n_lowest_cost_next_qs(self,n,start_q=None):
        if start_q:
            ret = [start_q]
            successors, successor_costs = self.step_seq_planner.successors(start_q)
        else:
            ret = [self.start_state]
            successors, successor_costs = self.step_seq_planner.successors(self.start_state)
        for _ in range(n):
            lowest_cost_successor = successors[successor_costs.index(min(successor_costs))]
            ret.append(lowest_cost_successor)
            successors, successor_costs = self.step_seq_planner.successors(lowest_cost_successor)
        return ret