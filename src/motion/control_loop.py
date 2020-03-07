import time
from src.utils.logger import Logger
from src.utils.math_utils import MathUtils
from .motion_planner import ConfigSpacePlanner
from src.motion.motion_utils import MotionUtils
from src.utils.data_objects.controlloop_output import ControlLoopOutput
from src.utils.vis_utils import VisUtils
from src.utils.py_utils import PyUtils
from src.motion.motion_planner import MPlannerResults
from klampt import Simulator
import _thread
from src.utils import project_constants

class ControlLoop(MotionUtils):

    ERROR = -1
    DONE = -2
    CONTINUE_LOOP = 3

    def __init__(
            self, world, fs_scatter_obj, fs_seq, height_map, gradient_map,
            u_input=None,
            debug=False,
            visualize=False,
            lidar_mode=False,
            physics_sim_enabled=False,
            execution_world_vis_id=None,
            execution_world=None,
            disable_sleep=False,
            save_qs=True
    ):

        MotionUtils.__init__(
            self, world, height_map, fs_scatter_obj.get_scatter_list(), fs_seq.get_state_path(),
            gradient_map, u_input=u_input, include_ik_solver=True, lidar_mode=lidar_mode)

        self.MotionPlanner = ConfigSpacePlanner(world, fs_scatter_obj, fs_seq, height_map, gradient_map, u_input=u_input)

        self.curr_stance_idx = 0
        self.curr_stance = self.stance_path[self.curr_stance_idx]

        if self.curr_stance_idx != 0:
            print(Logger.styled_text("\n|/////////////////////////////////////////////////////////////////////////////////////|", "BOLD"))
            print(Logger.styled_text("|/////////////////////////////////////////////////////////////////////////////////////|", "BOLD"))
            print(Logger.styled_text(
                "|///"+ Logger.styled_text(f" Warning: Current stance state idx != 0, set to: {self.curr_stance_idx}", "WARNING"), "BOLD"))
            print(Logger.styled_text("|/////////////////////////////////////////////////////////////////////////////////////|", "BOLD"))
            print(Logger.styled_text("|/////////////////////////////////////////////////////////////////////////////////////|", "BOLD"),"\n")

        self.torso_COM_constraints = None
        self.robot_pose = None
        self.torso_inside_new_support_tri = False
        self.mid_torso_shift_qs = None
        self.i = 0
        self.mid_step_qs = None
        self.j = 0

        # Testing settings + telemetry
        self.disable_sleep = disable_sleep or execution_world is None
        self.debug = debug
        self.start_time = None
        self.visualize = visualize
        self.save_qs = save_qs
        self.configurations = []

        self.execution_world = execution_world
        self.execution_world_vis_id = execution_world_vis_id
        self.physics_sim_enabled = physics_sim_enabled
        if self.physics_sim_enabled:

            self.thread_alive = True
            self.physics_sim_world = execution_world
            self.sim = Simulator(execution_world)
            self.controller = self.sim.controller(0)
            self.controller.setRate(project_constants.PHYSICS_SIM_CONTROLLER_DT)

            self.queue_counter = 0
            self.queue_every_kth = 5
            _thread.start_new_thread(self.physics_sim_loop, ())

    def physics_sim_loop(self):
        print("\n starting physics simulation loop \n")
        i = 0
        last_t = time.time()
        with open("measured_configs2.txt", "w") as f:
            while self.thread_alive:
                i += 1
                print(i, round(time.time() - last_t, 2))
                last_t = time.time()
                self.sim.simulate(project_constants.PHYSICS_SIM_CONTROLLER_DT)
                self.sim.updateWorld()
                f.writelines([str(self.sim.getTime()) + " " + str(self.controller.getSensedConfig())+"\n"])
                time.sleep(project_constants.PHYSICS_SIM_CONTROLLER_DT)
                if i == 20000:
                    break
        # self.sim_logger.close()

    def shutdown(self):
        self.thread_alive = False
        time.sleep(project_constants.CONTROLLER_DT + .01)

    def run(self):
        '''
            returns ControlLoopOutput object        '''
        # start_config = None
        # start_config = [8.983191813821893, 1.1387765913321155, 0.7110453548780796, -0.010607350121714439,
        #                 0.00318897585241481, -0.3782106726822103, 0.0, -0.36470022126774526, -0.42983708546072674,
        #                 -0.9656154843877591, -1.162764144835825, -1.1227687685326153, 1.2067709831981772,
        #                 2.012370669697552, 0.0, -2.855325976487946, 0.9675704008929811, 2.4039312001695445,
        #                 -1.3898375748390006, 2.3472823755818117, -1.2296154637618235, -2.3572032148269435, 0.0,
        #                 -1.387671812439424, 2.311986003640131, -1.85581655132715, -1.9382148305703715,
        #                 -1.230689538058792, -1.1043240925722673, 0.05175799094462954, 0.0, 0.40142659350852167,
        #                 0.27949971163500337, 1.2587274957276247, 2.171646993438606, 1.0540536194598196,
        #                 -1.1198153421419772, -1.2892958345343375]

        # start_config = [9.204843361615229, 1.0683417867501261, 0.5547397104925256, -1.5836487712828977e-05,
        #                 -8.878067632178796e-05, -3.984110984635196e-05, 0.0, -1.958527736371666, 2.055438605867218,
        #                 -0.5489179760341032, 1.4642684187682993, 0.25777658278394505, 1.264973614244851,
        #                 1.393247086960612, 0.0, 0.4953918768519218, -0.0073456253698971265, 0.6156951717025214,
        #                 2.797169700126785, 0.6460614641456953, -1.2838089842745606, -0.6897902290565635, 0.0,
        #                 -0.12156021912886919, 1.1101398279800254, -0.09330049302835725, -2.4004457258724634,
        #                 0.14757744961626185, -0.2849972241081419, -0.10373154592846655, 0.0, 0.14301345230721588,
        #                 0.1109278517604882, 0.8996531060588081, 2.540623351362262, 0.9371750613636076,
        #                 -1.3091759575838973, -0.618654287338747]

        # 41
        start_config = [9.049024593591819, 1.1851522339542324, 0.5692177749144672, 0.5758130419631018,
                        -0.11980322429956233, 0.08870166881329453, 0.0, -0.6133105302994748, 3.0300043195235458,
                        -0.23112829263252888, 0.001476833424789456, 0.2330298831195669, 1.531984938995741,
                        1.2047660006406191, 0.0, 0.9140563251616718, 0.1775375288086226, 1.3066756694515782,
                        2.6018142906672392, 1.122659480989548, -1.5205381103307465, -0.8401173066130498, 0.0,
                        -1.6252433045257002, 2.7316826795290026, 2.5693174738150977, -1.6322805852417177,
                        -2.2077853919498347, 0.801023298495912, 1.127272749101234, 0.0, 0.3644646362777182,
                        0.7295326245090006, 0.4805191970237466, -0.8141541775600726, 2.6443308580970446,
                        -1.6263948522654204, 3.0087282552769548]

        time.sleep(20)
        start_q = None if self.curr_stance_idx == 0 else start_config

        if start_q:
            self.robosimian.setConfig(start_q)
            self.robot_pose = self.get_robot_pose_from_stance(self.curr_stance, with_end_effector_Rs=True, visualize_normal=False)
            self.update_rpose_from_current_stance(self.robot_pose)
            time.sleep(15)

        else:
            self.robot_pose = self.get_robot_pose_from_stance(self.curr_stance, with_end_effector_Rs=True, visualize_normal=False)
            if not self.IKSolverUtil.set_pose_w_R(self.robot_pose):
                if project_constants.CLOOP_VERBOSITY >= 2:
                    Logger.log("Initial IK solver failed.", "FAIL")

        t_start = time.time()
        self.start_time = t_start
        run_loop = True

        if project_constants.CLOOP_VERBOSITY >= 2:
            print(f"Starting control loop. {len(self.stance_path)} total states")

        while 1:

            if self.u_input:
                if self.u_input.skip_one_config_forward():
                    self.skip_to_stance(self.curr_stance_idx + 1)

                if self.u_input.skip_one_config_backward():
                    self.skip_to_stance(self.curr_stance_idx - 1)

                if self.u_input.pause_control_loop():
                    run_loop = False

                if self.u_input.print_robot_config():
                    print(self.robosimian.getConfig())

                if self.u_input.enter_manual_control_mode():
                    self.manual_torso_control()
                    self.skip_to_stance(self.curr_stance_idx)

                if self.u_input.exit():
                    break

            if run_loop:
                cloop_res = self.control_loop()

                if cloop_res == ControlLoop.ERROR:

                    if project_constants.CLOOP_VERBOSITY >= 1:
                        Logger.log("Control loop error", "FAIL")

                    current_torso_xyz = self.get_current_torso_xyz_yaw_deg()
                    xy_yaw_rads0 = self.estimate_torso_xy_yaw_rads_from_stance(self.stance_path[0])
                    xy_yaw_radsf = self.estimate_torso_xy_yaw_rads_from_stance(self.stance_path[len(self.stance_path) - 1])
                    output_obj = ControlLoopOutput()
                    output_obj.configs = self.configurations[:]
                    output_obj.end_stance_idx = self.curr_stance_idx
                    output_obj.failed = True
                    output_obj.dist_to_end = MathUtils._2d_euclidian_distance(current_torso_xyz, xy_yaw_radsf)
                    output_obj.dist_from_start_to_end = MathUtils._2d_euclidian_distance(xy_yaw_rads0, xy_yaw_radsf)
                    output_obj.runtime = time.time()-t_start
                    return output_obj

                elif cloop_res == ControlLoop.DONE:
                    if project_constants.CLOOP_VERBOSITY >= 2:
                        Logger.log("Done", "OKGREEN")

                    current_torso_xyz = self.get_current_torso_xyz_yaw_deg()
                    xy_yaw_rads0 = self.estimate_torso_xy_yaw_rads_from_stance(self.stance_path[0])
                    xy_yaw_radsf = self.estimate_torso_xy_yaw_rads_from_stance(self.stance_path[len(self.stance_path) - 1])
                    output_obj = ControlLoopOutput()
                    output_obj.configs = self.configurations[:]
                    output_obj.end_stance_idx = self.curr_stance_idx
                    output_obj.failed = False
                    output_obj.dist_to_end = MathUtils._2d_euclidian_distance(current_torso_xyz, xy_yaw_radsf)
                    output_obj.dist_from_start_to_end = MathUtils._2d_euclidian_distance(xy_yaw_rads0, xy_yaw_radsf)
                    output_obj.runtime = time.time()-t_start
                    return output_obj

                elif cloop_res == ControlLoop.CONTINUE_LOOP:
                    pass

                else:
                    Logger.log("Error: control loop return value unrecognized", "FAIL")

            run_loop = True

            if not self.disable_sleep:
                time.sleep(project_constants.CONTROLLER_DT)

    def write_config_to_execution_world_robot(self, config: list):
        if self.execution_world:
            if self.physics_sim_enabled:
                if self.queue_counter == self.queue_every_kth:
                    print("calling controller.addMilestone(), remaining time:", self.controller.remainingTime())
                    # self.controller.setLinear(config, project_constants.PHYSICS_SIM_CONTROLLER_DT)
                    self.controller.addMilestone(config)
                    self.queue_counter = 0
                self.queue_counter += 1
            else:
                self.execution_world.robot(0).setConfig(config)

    def control_loop(self):

        if self.curr_stance_idx + 1 == len(self.stance_path):
            return ControlLoop.DONE

        only_use_sbl = False

        # self.update_torso_com_line()

        # Shift Torso to inside support triangle
        if not self.torso_inside_new_support_tri:

            # Start of torso shift
            if self.mid_torso_shift_qs is None:

                if project_constants.CLOOP_VERBOSITY >= 3:
                    print(f"\n\n____________________________\n{PyUtils.format_time(time.time() - self.start_time)}:  Starting torso shift\n ")

                # Update com constraints
                if self.torso_COM_constraints:
                    self.torso_COM_constraints.remove_all_visualizations()
                try:
                    self.torso_COM_constraints = self.get_constraint_obj(self.curr_stance_idx, visualize=1)
                except IndexError:
                    return ControlLoop.DONE # Finished

                # Update robot pose
                self.update_rpose_from_current_stance(self.robot_pose)

                self.mid_torso_shift_qs = self.MotionPlanner.plan_torsoshift(
                    self.robot_pose, self.torso_COM_constraints, self.curr_stance_idx,
                    visualize=self.visualize, only_use_sbl=only_use_sbl)

                if self.mid_torso_shift_qs is False:
                    return ControlLoop.ERROR
                else:
                    if self.save_qs:
                        self.configurations += self.mid_torso_shift_qs

            else:
                if self.disable_sleep:
                    self.robosimian.setConfig(self.mid_torso_shift_qs[len(self.mid_torso_shift_qs)-1])
                    self.i = len(self.mid_torso_shift_qs)

                if self.i == len(self.mid_torso_shift_qs):
                    self.torso_inside_new_support_tri = True
                    self.i = 0
                    self.mid_torso_shift_qs = None
                    self.update_rpose_from_current_stance(self.robot_pose)
                else:
                    self.write_config_to_execution_world_robot(self.mid_torso_shift_qs[self.i])
                    # self.robosimian.setConfig(self.mid_torso_shift_qs[self.i])
                    self.i += 1

        # Move leg
        else:

            if self.mid_step_qs is None:

                if project_constants.CLOOP_VERBOSITY >= 3:
                    print("\n\n___________________\n  Starting leg step\n")

                self.mid_step_qs = self.MotionPlanner.plan_legstep(
                    self.robot_pose, self.torso_COM_constraints, self.curr_stance_idx,
                    visualize=self.visualize, only_use_sbl=only_use_sbl)

                if not self.mid_step_qs:
                    return ControlLoop.ERROR
                else:
                    if self.save_qs:
                        self.configurations += self.mid_step_qs
            else:
                if self.disable_sleep:
                    self.robosimian.setConfig(self.mid_step_qs[len(self.mid_step_qs)-1])
                    self.j = len(self.mid_step_qs)

                if self.j == len(self.mid_step_qs):
                    self.mid_step_qs = None
                    self.torso_inside_new_support_tri = False
                    self.j = 0
                    self.curr_stance_idx += 1
                    self.curr_stance = self.stance_path[self.curr_stance_idx]
                    self.update_rpose_from_current_stance(self.robot_pose)
                else:
                    self.write_config_to_execution_world_robot(self.mid_step_qs[self.j])
                    # self.robosimian.setConfig(self.mid_step_qs[self.j])
                    self.j += 1

        return ControlLoop.CONTINUE_LOOP
