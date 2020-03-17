from klampt import WorldModel
from klampt import vis
import numpy as np
import pickle as pickle
import time
import os
import sys
from src.utils import project_constants
from src.generators.footstep_costmap_generator import FootstepCostMapGenerator
from src.generators.conv_costmap_generator import ConvolutionCostMapGenerator
from src.generators.step_sequence_planner import  StepSequencePlanner
from src.generators.height_map_generator import  HeightMapGenerator
from src.generators.high_level_trajectory_generator import HighLevelTrajectoryPlanner
from src.generators.scatter_list_generator import ScatterListGenerator
from src.utils.data_objects.system_runner_results import SystemRunnerResults
from src.utils.robot_poser import RobotPoser
from src.utils.vis_utils import VisUtils
from src.motion.control_loop import ControlLoop
from src.utils.logger import Logger
from src.utils.math_utils import MathUtils
from src.utils.data_objects.controlloop_output import ControlLoopOutput
from testing.StepSeqTester import StepSequencePlannerTester
from src.lidar.pcloud_parser import PCloudParser
from src.utils.data_objects.height_map import HeightMap
from src.utils.data_objects.gradient_map import GradientMap


class SystemRunner:

    robot_file = "data/robot_model/robosimian_caesar_new.rob"

    hm_type = "_hm"
    fs_costmap_type = "_fs_costmap"
    fs_convcostmap_type = "_conv_fs_costmap"
    gradient_map_type = "_fs_gradient_map"
    hl_traj_type = "_high_level_traj"
    fs_scatter_type = "_scatter_list"
    fs_seq_type = "_fs_seq_type"
    control_loop_type = "_control_loop_config_outputs"

    def __init__(self):

        self.HM_X_START = None
        self.HM_X_END = None
        self.HM_Y_START = None
        self.HM_Y_END = None
        self.xy_yaw0 = None
        self.xy_yawf = None

        self.world_name = None
        self.save_root = None
        self.world_file = None
        self.pcloud_fname = None

        self.results = SystemRunnerResults()
        self.results.world_name = None
        self.results.project_constants = project_constants

        self.planning_world = WorldModel()
        self.rposer = None

        # self.settings = SystemRunner.default_settings

        # saved data objects
        self.hm_obj = None
        self.fs_cost_map_obj = None
        self.fs_convcostmap_obj = None
        self.gradient_map_obj = None
        self.hl_traj_obj = None
        self.fs_scatter_obj = None
        self.fs_seq_obj = None
        self.control_loop_output_obj = None

        self.hl_traj_max_runtime = project_constants.HL_TRAJ_MAX_RUNTIME
        self.step_seq_max_runtime =  project_constants.STEPSEQ_MAX_RUNTIME

        self.physics_sim_enabled = False
        self.execution_world_enabled = False
        self.execution_world_window_id = None
        self.execution_world = None

        self.cloop_run_name = None
        self.active_control_loop = None

        self.vis_window_id = None

    def initialize_sim_world(
            self, x_inbound_range, y_inbound_range, xy_yaw0, xy_yawf, world_name,
            execution_world_enabled=True, physics_sim_enabled=False, visualize=True, cloop_run_name=None):

        self.HM_X_START = x_inbound_range[0]
        self.HM_X_END = x_inbound_range[1]
        self.HM_Y_START = y_inbound_range[0]
        self.HM_Y_END = y_inbound_range[1]
        self.xy_yaw0 = xy_yaw0
        self.xy_yawf = xy_yawf

        self.cloop_run_name = cloop_run_name

        if not MathUtils._2dpoint_inbounds(xy_yaw0, x_inbound_range, y_inbound_range):
            Logger.log("robot xy0 outside of search area", color="FAIL", class_id=0, msg_type=1)

        if not MathUtils._2dpoint_inbounds(xy_yawf, x_inbound_range, y_inbound_range):
            Logger.log("robot xyf outside of search area", color="FAIL", class_id=0, msg_type=1)

        self.world_name = world_name
        self.save_root = "data/stored_world_data/" + world_name + "/"
        self.world_file = SystemRunner.get_world_file_from_world_name(world_name)

        self.results.world_name = world_name

        if not self.planning_world.readFile(self.world_file):
            raise RuntimeError("Unable to load terrain model")

        if not self.planning_world.readFile(SystemRunner.robot_file):
            raise RuntimeError("Unable to load robot model")

        q = project_constants.NOMINAL_CONFIG
        q[0] = self.xy_yaw0[0]
        q[1] = self.xy_yaw0[1]
        q[5] = self.xy_yaw0[2]
        self.planning_world.robot(0).setConfig(q)

        self.physics_sim_enabled = physics_sim_enabled
        if execution_world_enabled:

            self.execution_world_enabled = True
            self.execution_world = self.planning_world.copy()

            if visualize:
                self.execution_world_window_id = vis.createWindow(self.world_name + " execution world")
                vis.setWindow(self.execution_world_window_id)
                vis.add("world", self.execution_world)
                self.visualize_hm_bounds()
                vp = vis.getViewport()
                vp.w, vp.h = 800, 800
                vis.setViewport(vp)
                vis.autoFitCamera()
                vis.show()

        if visualize:
            self.visualize(visualize_hltraj=False, visualize_stepseq=False, visualize_mplanner=False, load_first=False)

    def initialize_lidar_world(
            self, world_name, pcloud_fname, xy_yaw0, xy_yawf, visualize=True,
            inbound_xrange=None, inbound_yrange=None):

        self.world_name = world_name
        self.save_root = "data/stored_world_data/"+world_name+"/"
        self.pcloud_fname = pcloud_fname

        self.xy_yaw0 = xy_yaw0
        self.xy_yawf = xy_yawf

        self.results.world_name = world_name

        if not self.planning_world.readFile(SystemRunner.robot_file):
            raise RuntimeError("Unable to load robot model")

        pcloud_parser = PCloudParser()
        x_vars, y_vars = pcloud_parser.get_xy_vars_from_pcloud(self.pcloud_fname)

        self.HM_X_START = inbound_xrange[0]
        self.HM_X_END = inbound_xrange[1]
        self.HM_Y_START = inbound_yrange[0]
        self.HM_Y_END = inbound_yrange[1]
        project_constants.HM_Y_GRANULARITY = y_vars[2]
        project_constants.HM_X_GRANULARITY = x_vars[2]

        self.hm_obj = self.get_saved_object(self.hm_type)
        if self.hm_obj is None:
            self.hm_obj, runtime = pcloud_parser.build_hm_obj(self.pcloud_fname, visualize=False, debug=False)
            self.results.hm_runtime = runtime

        if visualize:
            self.visualize(
                visualize_hltraj=False, visualize_stepseq=False, visualize_mplanner=False, load_first=False,
                visualize_hm_bounds=False
            )

            pcloud_parser.add_hm_to_klampt_vis(self.pcloud_fname)
            q = project_constants.NOMINAL_CONFIG
            q[0] = self.xy_yaw0[0]
            q[1] = self.xy_yaw0[1]
            q[2] = self.xy_yaw0[2]
            self.planning_world.robot(0).setConfig(q)

            if inbound_xrange and inbound_yrange:
                self.hm_obj.visualize_in_klampt(xmin=inbound_xrange[0], xmax=inbound_xrange[1], ymin=inbound_yrange[0], ymax=inbound_yrange[1])
            else:
                self.hm_obj.visualize_in_klampt()

    @staticmethod
    def get_world_file_from_world_name(world_name):
        return "data/simulation_test_worlds/" + world_name + ".xml"

    # ___________________________________________________________________ Object Building Functions

    def build_costmap(self, save=False, overwrite=False, print_saved=True, debug=False, exlude_slope=False, exlude_roughness=False, exlude_step=False):

        if self.fs_cost_map_obj is not None:
            Logger.log("Warning: cost map is already built and loaded", "WARNING")
            return

        if self.hm_obj is None:
            self.hm_obj = self.get_saved_object(SystemRunner.hm_type, print_loaded=False)

        if self.hm_obj is None:
            Logger.log("Height map is not build, exiting", color="FAIL", class_id=0, msg_type=1)

        if self.gradient_map_obj is None:
            self.gradient_map_obj = self.get_saved_object(SystemRunner.gradient_map_type, print_loaded=False)
        if self.gradient_map_obj is None:
            Logger.log("Gradient map is not build, exiting", color="FAIL", class_id=0, msg_type=1)

        cmap_generator = FootstepCostMapGenerator(self.hm_obj, self.gradient_map_obj)

        self.fs_cost_map_obj = cmap_generator.build_costmap(
            debug=debug, exlude_slope=exlude_slope, exlude_roughness=exlude_roughness, exlude_step=exlude_step, normalize_cost_arr=True)

        Logger.log(f"Built cost map in {Logger.pp_double(self.fs_cost_map_obj.runtime)} seconds", class_id=0, msg_type=2)
        if save:
            print("saving from build_costmap()")
            self.save_obj(SystemRunner.fs_costmap_type, print_saved=print_saved, overwrite=overwrite)
        return self.fs_cost_map_obj

    def build_heightmap(self, save=False, overwrite=True, print_saved = True):

        if self.hm_obj is not None:
            Logger.log("Warning: height map is already built and loaded", "WARNING")

        hm_generator = HeightMapGenerator(self.planning_world, [self.HM_X_START, self.HM_X_END], [self.HM_Y_START, self.HM_Y_END])
        self.hm_obj = hm_generator.build_height_map()

        Logger.log(f"Built height map in {Logger.pp_double(self.hm_obj.runtime)} s", class_id=0, msg_type=2)
        if save:
            self.save_obj(SystemRunner.hm_type, print_saved=print_saved, overwrite=overwrite)
        return self.hm_obj

    def build_gradientmap(self, save=False, overwrite=True, print_saved = True):

        if self.gradient_map_obj is not None:
            Logger.log("Warning: gradient map is already built and loaded", "WARNING")

        self.gradient_map_obj: GradientMap = self.hm_obj.get_gradient_map_obj()
        Logger.log("Built gradient map in " + str(Logger.pp_double(self.gradient_map_obj.runtime)) + "s", class_id=0, msg_type=2)
        if save:
            self.save_obj(SystemRunner.gradient_map_type, print_saved=print_saved, overwrite=overwrite)
        return self.gradient_map_obj

    def build_convcostmap(self, save=False, overwrite=True, print_saved = True):

        if self.fs_cost_map_obj is None:
            self.fs_cost_map_obj = self.get_saved_object(SystemRunner.fs_costmap_type, print_loaded=True)

        if self.fs_cost_map_obj is None:
            Logger.log("Cost map is not build, exiting", color="FAIL", class_id=0, msg_type=1)
            return

        convmap_gen = ConvolutionCostMapGenerator(self.fs_cost_map_obj)
        self.fs_convcostmap_obj = convmap_gen.build_conv_arr(normalize=True)
        # convmap_gen.normalize_cost_arr()

        self.fs_convcostmap_obj = convmap_gen.return_cost_arr()

        Logger.log(f"Built conv cost map in {Logger.pp_double(self.fs_convcostmap_obj.runtime)} s", class_id=0, msg_type=2)
        if save:
            self.save_obj(SystemRunner.fs_convcostmap_type, overwrite=overwrite, print_saved=print_saved)
        return self.fs_convcostmap_obj

    # ___________________________________________________________________ Retreive Objects

    def get_stepseq_tester(self):

        hm = self.get_heightmap()
        if hm is None: Logger.log("Height map does not exist. Exiting", color="FAIL", class_id=0, msg_type=1); return None

        fs_scatter = self.get_saved_object(SystemRunner.fs_scatter_type, print_dne=True)
        if fs_scatter is None: Logger.log("fs scatter does not exist. Exiting", color="FAIL", class_id=0, msg_type=1); return None

        hl_traj = self.get_hl_traj()
        if hl_traj is None: Logger.log("hl traj does not exist. Exiting", color="FAIL", class_id=0, msg_type=1); return None
        self.visualize_hl_traj_route()

        cost_map = self.get_costmap()
        if cost_map is None:
            Logger.log("cost map does not exist. Exiting", color="FAIL", class_id=0, msg_type=1); return None

        rposer = RobotPoser(self.planning_world, hm, fs_scatter=fs_scatter)
        return StepSequencePlannerTester( hm, fs_scatter, hl_traj, cost_map, self.xy_yaw0, self.xy_yawf, rposer)

    def get_hl_traj(self):
        if self.hl_traj_obj is None:
            self.hl_traj_obj = self.get_saved_object(SystemRunner.hl_traj_type)
            if self.hl_traj_obj is None:
                Logger.log("hl traj does not exist. Exiting", color="FAIL", class_id=0, msg_type=1)
                return None
        return self.hl_traj_obj

    def get_fs_seq(self):
        if self.fs_seq_obj is None:
            self.fs_seq_obj = self.get_saved_object(SystemRunner.fs_seq_type)
            if self.fs_seq_obj is None:
                Logger.log("fs seq does not exist. Exiting", color="FAIL", class_id=0, msg_type=1)
                return None
        return self.fs_seq_obj

    def get_fs_scatter(self, print_dne=True):

        if self.fs_scatter_obj is None:
            self.fs_scatter_obj = self.get_saved_object(SystemRunner.fs_scatter_type, print_dne=print_dne)
            if self.fs_scatter_obj is None:
                return None
        return self.fs_scatter_obj

    def get_costmap(self):
        if not self.fs_cost_map_obj:
            self.fs_cost_map_obj = self.get_saved_object(SystemRunner.fs_costmap_type)
            if not self.fs_cost_map_obj:
                Logger.log("Cost map does not exist. Exiting", color="FAIL", class_id=0, msg_type=1)
                return None
        return self.fs_cost_map_obj

    def get_heightmap(self):
        if not self.hm_obj:
            self.hm_obj = self.get_saved_object(SystemRunner.hm_type)
            if not self.hm_obj:
                Logger.log("Height map does not exist. Exiting", color="FAIL", class_id=0, msg_type=1)
                return None
        return self.hm_obj


    # __________________________________________________________________________________________________________________
    # -- Run

    def run_physics_sim(self, save_qs_to_file=None):

        if not self.execution_world_enabled:
            print("execution world must be initialized first")
            return

        self.control_loop_output_obj: ControlLoopOutput = self.get_saved_object(SystemRunner.control_loop_type)

        from klampt import Simulator
        self.execution_world.robot(0).setConfig(self.control_loop_output_obj.configs[0])
        sim = Simulator(self.execution_world)

        # nb_configs = len(self.control_loop_output_obj.configs)
        nb_configs = min(7500, len(self.control_loop_output_obj.configs))

        controller_dt = .001
        simulation_dt = .001

        controller = sim.controller(0)
        controller.setRate(controller_dt)

        f = None
        if save_qs_to_file:
            f = open(save_qs_to_file, "w")

        dt = .1
        M = 7
        j = 0
        import progressbar
        with progressbar.ProgressBar(max_value=nb_configs+M) as bar:
            while j < nb_configs:
                if controller.remainingTime() < 3:
                    config = self.control_loop_output_obj.configs[j]

                    # controller.addMilestone(config)
                    controller.addLinear(config, dt)

                    # print("appending milestone, remaining itme:", controller.remainingTime())
                    j += M
                    bar.update(j)

                if f:
                    f.writelines([str(sim.getTime()) + " " + str(controller.getSensedConfig())+"\n"])

                sim.simulate(simulation_dt)
                # sim.updateWorld()
                # time.sleep(simulation_dt)

        f.close()

    def run(
            self,
            print_results=True,
            only_mapping=False,
            run_hltplanner=True,
            run_splanner=True,
            run_mplanner=True,
            lidar_mode=False,
            conv_override=False,
            ignore_saved_cloop=False,
            ignore_saved_splan=False):


        # _____________________________________________________________ Height map
        if self.hm_obj is None:
            self.hm_obj: HeightMap = self.get_saved_object(SystemRunner.hm_type, print_loaded=False, print_dne=False)
        if self.hm_obj is None:
            self.build_heightmap()
            self.results.hm_runtime = self.hm_obj.runtime
        else:
            self.results.hm_runtime = SystemRunnerResults.LOADED_FROM_PICKLE


        # _____________________________________________________________ Gradient map
        if self.gradient_map_obj is None:
            self.gradient_map_obj = self.get_saved_object(SystemRunner.gradient_map_type, print_loaded=False, print_dne=False)
        if self.gradient_map_obj is None:
            self.build_gradientmap()
            self.results.gradient_map_runtime = self.gradient_map_obj.runtime
        else:
            self.results.gradient_map_runtime = SystemRunnerResults.LOADED_FROM_PICKLE


        # _____________________________________________________________ fs Cost Map
        self.results.fs_cost_map_runtime = SystemRunnerResults.LOADED_FROM_PICKLE
        if self.fs_cost_map_obj is None:
            self.fs_cost_map_obj = self.get_saved_object(SystemRunner.fs_costmap_type, print_loaded=False, print_dne=False)
        if self.fs_cost_map_obj is None:
            self.build_costmap(exlude_slope=True)
            self.results.fs_cost_map_runtime = self.fs_cost_map_obj.runtime
        else:
            self.results.fs_cost_map_runtime = SystemRunnerResults.LOADED_FROM_PICKLE
        cost_map = self.fs_cost_map_obj


        # _____________________________________________________________ fs conv. cost Map
        if project_constants.USE_CONVMAP or conv_override:
            if self.fs_convcostmap_obj is None:
                self.fs_convcostmap_obj = self.get_saved_object(SystemRunner.fs_convcostmap_type, print_loaded=False, print_dne=False)
            if self.fs_convcostmap_obj is None:
                self.build_convcostmap()
                self.results.fs_convcost_map_runtime = self.fs_convcostmap_obj.runtime
            else:
                self.results.fs_convcost_map_runtime = SystemRunnerResults.LOADED_FROM_PICKLE

            if not conv_override:
                cost_map = self.fs_convcostmap_obj

        if only_mapping:
            if print_results:
                self.results.print_results()
            return

        # _____________________________________________________________ fs scatter
        fs_scatter_runtime = SystemRunnerResults.LOADED_FROM_PICKLE
        if self.fs_scatter_obj is None:
            self.fs_scatter_obj = self.get_saved_object(SystemRunner.fs_scatter_type, print_loaded=False, print_dne=False)
            fs_scatter_runtime = SystemRunnerResults.LOADED_FROM_PICKLE

        if self.fs_scatter_obj is None:
            Logger.log("Building Footstep Scatter", class_id=0, msg_type=2)
            scatter_generator = ScatterListGenerator(self.hm_obj, cost_map, self.xy_yaw0)
            self.fs_scatter_obj = scatter_generator.build_list()
            fs_scatter_runtime = self.fs_scatter_obj.runtime
        self.results.fs_scatter_runtime = fs_scatter_runtime

        if not run_hltplanner:
            if print_results:
                self.results.print_results()
            return


        # _____________________________________________________________ HL Trajectory
        self.rposer = RobotPoser(self.planning_world, self.hm_obj, fs_scatter=self.fs_scatter_obj)

        if self.hl_traj_obj is None:
            self.hl_traj_obj = self.get_saved_object(SystemRunner.hl_traj_type, print_loaded=False, print_dne=False)
            if self.hl_traj_obj is not None:
                self.results.hl_traj_runtime = SystemRunnerResults.LOADED_FROM_PICKLE

        if self.hl_traj_obj is None:
            Logger.log("Building HL Trajectory", class_id=0, msg_type=2)
            hl_trajectory_generator = HighLevelTrajectoryPlanner(self.hm_obj, cost_map, self.xy_yaw0, self.xy_yawf, rposer=self.rposer)
            self.hl_traj_obj = hl_trajectory_generator.build_trajectory(suspend_after=self.hl_traj_max_runtime)
            self.results.hl_traj_runtime = self.hl_traj_obj.runtime

        if self.vis_window_id is not None:
            if project_constants.HL_TRAJ_VISUALIZE_ROUTE:
                smoothed_path = self.hl_traj_obj.get_higher_density_xy_yaw_path()
                VisUtils.visualize_xyz_list(smoothed_path, name="hl traj", height_map=self.hm_obj)

        if self.hl_traj_obj.failed:
            self.results.hl_traj_runtime = SystemRunnerResults.FAILED
            if print_results:
                self.results.print_results()
            return

        if conv_override:
            cost_map = self.fs_convcostmap_obj

        if not run_splanner:
            if print_results:
                self.results.print_results()
            return


        # _____________________________________________________________ Fs Sequence
        if self.fs_seq_obj is None:
            self.fs_seq_obj = self.get_saved_object(SystemRunner.fs_seq_type, print_loaded=False, print_dne=False)
            if self.fs_seq_obj is not None:
                self.results.fs_seq_runtime = SystemRunnerResults.LOADED_FROM_PICKLE

        if self.fs_seq_obj is None or ignore_saved_splan:
            Logger.log("Building FS Sequence", class_id=0, msg_type=2)
            if self.vis_window_id is not None:
                fstep_seq_generator = StepSequencePlanner(
                    self.hm_obj, self.fs_scatter_obj, self.hl_traj_obj, cost_map, self.xy_yaw0, self.xy_yawf,
                    vis_successors=project_constants.STEPSEQ_VISUALIZE_SUCCESSOR,
                    deep_debug_visualization=project_constants.STEPSEQ_VISUALIZE_INDIVIDUAL_FS_PLACEMENT_SEARCH,
                    r_poser=self.rposer)
            else:
                fstep_seq_generator = StepSequencePlanner(self.hm_obj, self.fs_scatter_obj, self.hl_traj_obj, cost_map,
                    self.xy_yaw0, self.xy_yawf,
                    vis_successors=False, deep_debug_visualization=False, r_poser=None)

            self.fs_seq_obj = fstep_seq_generator.build_sequence(suspend_after=self.step_seq_max_runtime)
            self.results.fs_seq_runtime = self.fs_seq_obj.runtime

        # fs sequence failed
        if self.fs_seq_obj.failed:
            self.results.fs_seq_runtime = SystemRunnerResults.FAILED
            if print_results:
                self.results.print_results()
            return



        # _____________________________________________________________  Motion Planner
        if run_mplanner:
            if self.control_loop_output_obj is None and not ignore_saved_cloop:
                self.control_loop_output_obj = self.get_saved_object(
                    SystemRunner.control_loop_type, print_loaded=False, print_dne=False)
                if self.control_loop_output_obj is not None:
                    self.results.control_loop_output_obj = SystemRunnerResults.LOADED_FROM_PICKLE

            if self.control_loop_output_obj is None or ignore_saved_cloop:

                disable_sleep = True if self.vis_window_id is None else project_constants.CLOOP_ENABLE_SLEEP
                visualize = self.vis_window_id is not None and project_constants.MPLANNER_VIS_ENABLED

                self.active_control_loop = ControlLoop(
                    self.planning_world, self.fs_scatter_obj, self.fs_seq_obj, self.hm_obj, self.gradient_map_obj,
                    physics_sim_enabled = self.physics_sim_enabled,
                    execution_world_vis_id= self.execution_world_window_id,
                    execution_world=self.execution_world,
                    lidar_mode=lidar_mode,
                    visualize=visualize,
                    disable_sleep=disable_sleep,
                    save_qs=project_constants.CLOOP_SAVE_Qs
                )

                Logger.log("Running full body motion planner", class_id=0, msg_type=2)
                self.control_loop_output_obj = self.active_control_loop.run()

                self.results.control_loop_output_obj = self.control_loop_output_obj

        if print_results:
            self.results.print_results()

    def get_results(self):
        return self.results

    # __________________________________________________________________________________________________________________
    #  Get stats

    def print_stats(self, load=True):
        
        print(f"\nResults for: {self.world_name}")        

        if load: self.load()

        if self.hm_obj:
            self.hm_obj.print_stats()
        else: 
            print("<HeightMap>: None")

        if self.fs_cost_map_obj:
            self.fs_cost_map_obj.print_stats()
        else: print("<CostMap>: None")

        if self.fs_scatter_obj:
            self.fs_scatter_obj.print_stats()
        else: print("<ScatterList>: None")

        if self.hl_traj_obj:
            self.hl_traj_obj.print_stats()
        else:  print("<HLTraj>: None")

        if self.fs_seq_obj:
            self.fs_seq_obj.print_stats()
        else: print("<FsSeq>: None")

        if self.control_loop_output_obj:
            self.control_loop_output_obj.print_stats()
        else:  print("<CLoopObj>: None")


    def print_cloop_output_stats(self):
        self.control_loop_output_obj: ControlLoopOutput = self.get_saved_object(SystemRunner.control_loop_type)
        self.control_loop_output_obj.print_stats()

    # __________________________________________________________________________________________________________________
    # Visualization functions

    def visualize_hl_traj_route(self):
        hl_traj = self.get_hl_traj()
        if hl_traj is None: Logger.log("hl does not exist. Exiting", color="FAIL", class_id=0, msg_type=1); return
        hm = self.get_heightmap()
        if hm is None: Logger.log("hm does not exist. Exiting", color="FAIL", class_id=0, msg_type=1); return
        smoothed_path = self.hl_traj_obj.get_higher_density_xy_yaw_path()
        VisUtils.visualize_xyz_list(smoothed_path, name="hl traj", height_map=self.hm_obj)

    def visualize_conv_cmap_in_klampt_vis(self, step=5):

        if not self.hm_obj:
            self.hm_obj = self.get_saved_object(SystemRunner.hm_type)
            if not self.hm_obj:
                Logger.log("Height map does not exist. Exiting", color="FAIL", class_id=0, msg_type=1)
                return

        if not self.fs_convcostmap_obj:
            self.fs_convcostmap_obj = self.get_saved_object(SystemRunner.fs_convcostmap_type)
            if not self.fs_convcostmap_obj:
                Logger.log("Cost map does not exist. Exiting", color="FAIL", class_id=0, msg_type=1)
                return

        self.init_klampt_vis()
        self.fs_convcostmap_obj.visualize_in_klampt(self.hm_obj, step=step)

    def visualize_fs_scatter(self):
        self.fs_scatter_obj = self.get_saved_object(SystemRunner.fs_scatter_type, print_dne=True)
        if self.fs_scatter_obj:
            self.init_klampt_vis()
            self.fs_scatter_obj.visualize_in_klampt()

    def visualize_cost_map(self):
        if not self.fs_cost_map_obj:
            self.fs_cost_map_obj = self.get_saved_object(SystemRunner.fs_costmap_type)
            if not self.fs_cost_map_obj:
                Logger.log("FS cost map does not exist. Exiting", color="FAIL", class_id=0, msg_type=1)
        self.fs_cost_map_obj.visualize()

    def visualize_height_map(self):
        if not self.hm_obj:
            self.hm_obj: HeightMap = self.get_saved_object(SystemRunner.hm_type)
            if not self.hm_obj:
                Logger.log("Height map does not exist. Exiting", color="FAIL", class_id=0, msg_type=1)
        self.hm_obj.visualize()

    def visualize_convcost_map(self):
        if not self.fs_convcostmap_obj:
            self.fs_convcostmap_obj = self.get_saved_object(SystemRunner.fs_convcostmap_type)
            if not self.fs_convcostmap_obj:
                Logger.log("Height map does not exist. Exiting", color="FAIL", class_id=0, msg_type=1)
        self.fs_convcostmap_obj.visualize()

    def visualize_gradient_map(self):
        if not self.gradient_map_obj:
            self.gradient_map_obj = self.get_saved_object(SystemRunner.gradient_map_type)
            if not self.gradient_map_obj:
                Logger.log("Gradient map does not exist. Exiting", color="FAIL", class_id=0, msg_type=1)
        self.gradient_map_obj.visualize()

    def visualize_cmap_in_klampt_vis(self, step=5):

        if not self.hm_obj:
            self.hm_obj = self.get_saved_object(SystemRunner.hm_type, print_dne=False, print_loaded=True)
            if not self.hm_obj:
                Logger.log("Height map does not exist. Exiting", color="FAIL", class_id=0, msg_type=1)
                return

        if not self.fs_cost_map_obj:
            self.fs_cost_map_obj = self.get_saved_object(SystemRunner.fs_costmap_type)
            if not self.fs_cost_map_obj:

                # Logger.log("Cost map does not exist. Exiting", color="FAIL", class_id=0, msg_type=1)
                print("building cost map")
                start_t = time.time()

                # self.build_costmap(exlude_slope=True, exlude_step=True)
                self.build_costmap(exlude_roughness=True, exlude_step=True)
                # self.build_costmap(exlude_slope=True)

                print(f"built cost map in {time.time() - start_t} seconds")
                # _ seconds, no vis, skipping if min == max

        self.init_klampt_vis()
        self.fs_cost_map_obj.visualize_in_klampt(self.hm_obj, step=step)

    def visualize_hm_bounds(self):
        y0 = self.HM_Y_START
        yf = self.HM_Y_END
        x0 = self.HM_X_START
        xf = self.HM_X_END

        VisUtils.visualize_line(
            [self.xy_yaw0[0], self.xy_yaw0[1], .001], [self.xy_yaw0[0], self.xy_yaw0[1], .001], "Start")

        VisUtils.visualize_line(
            [self.xy_yawf[0], self.xy_yawf[1], .001], [self.xy_yawf[0], self.xy_yawf[1], .001], "End")

        VisUtils.visualize_line([xf, y0, .001], [x0, y0, .001], " -- X -- ")
        VisUtils.visualize_line([x0, yf, .001], [x0, y0, .001], " -- Y -- ")
        for x in range(int(x0), int(xf + 1)):
            for y in range(int(y0), int(yf + 1)):
                text = "(" + str(x) + ", " + str(y) + ")"
                vis.addText(str(x) + str(y), text, pos=[x, y, .01])

    def init_klampt_vis(self, visualize_hm_bounds=True):

        if self.vis_window_id is None:
            self.vis_window_id = vis.createWindow(self.world_name)
            vis.setWindowTitle(self.world_name + " planning world")

            if visualize_hm_bounds:
                self.visualize_hm_bounds()

        else:
            vis.setWindow(self.vis_window_id)

        try:
            vis.add("world", self.planning_world)
            vp = vis.getViewport()
            vp.w, vp.h = 800, 800
            vis.setViewport(vp)
            vis.autoFitCamera()
            vis.show()

        except AttributeError:
            print("Attribute error in init_klampt_vis()")
            pass

    def visualize(
            self, visualize_hltraj=True, visualize_stepseq=True, visualize_mplanner=True, load_first=True,
            visualize_hm_bounds=True, skip_to_pct_through_cloop_output=None):

        self.init_klampt_vis(visualize_hm_bounds=visualize_hm_bounds)

        if visualize_hltraj:

            if load_first:
                if self.hl_traj_obj is None:
                    self.hl_traj_obj = self.get_saved_object(SystemRunner.hl_traj_type, print_dne=False)

            if self.hl_traj_obj:

                if self.hm_obj is None:
                    self.hm_obj = self.get_saved_object(SystemRunner.hm_type, print_dne=False)

                name = "_"
                smoothed_path = self.hl_traj_obj.get_higher_density_xy_yaw_path()
                VisUtils.visualize_xyz_list(smoothed_path, name=name, height_map=self.hm_obj)

                torso_z = .62
                robot = self.planning_world.robot(0)

                xy_yaw_traj = self.hl_traj_obj.get_xy_yaw_path()
                if not xy_yaw_traj:
                    Logger.log("Error: trajectory doesn't exist", color="FAIL", class_id=0, msg_type=1)
                    return

                base_config = project_constants.NOMINAL_CONFIG[:]
                base_config[2] = project_constants.TORSO_Z_DESIRED

                start_config = base_config
                start_config[0] = self.hl_traj_obj.xy_yaw0[0]
                start_config[1] = self.hl_traj_obj.xy_yaw0[1]
                start_config[3] = np.deg2rad(self.hl_traj_obj.xy_yaw0[2])
                vis.add("start config", start_config)

                end_config = base_config
                end_config[0] = self.hl_traj_obj.xy_yawf[0]
                end_config[1] = self.hl_traj_obj.xy_yawf[1]
                end_config[3] = np.deg2rad(self.hl_traj_obj.xy_yawf[2])
                vis.add("end config", end_config)

                # smoothed_path = math_utils._3d_pointlist_cubic_interprolation(xy_yaw_traj)
                extended_path = self.hl_traj_obj.get_higher_density_xy_yaw_path()
                in_t = 15
                dt = float(in_t) / len(extended_path)

                for xy_yaw in extended_path:
                    q = base_config
                    q[0] = xy_yaw[0]
                    q[1] = xy_yaw[1]
                    q[3] = np.deg2rad(xy_yaw[2])
                    ground_height = self.hm_obj.height_at_xy(xy_yaw[0], xy_yaw[1])
                    q[2] = torso_z + ground_height
                    VisUtils.visualize_line([xy_yaw[0], xy_yaw[1], ground_height], [xy_yaw[0], xy_yaw[1], torso_z],
                                      " -- Torso COM -- ")
                    robot.setConfig(q)
                    time.sleep(dt)
                vis.remove("end config")
                vis.remove("start config")

        if visualize_stepseq:
            if load_first:
                if self.fs_seq_obj is None:
                    self.fs_seq_obj = self.get_saved_object(SystemRunner.fs_seq_type, print_dne=True, print_loaded=True)

            self.hm = self.get_heightmap()
            self.fs_scatter_obj = self.get_saved_object(SystemRunner.fs_scatter_type, print_dne=False)
            if self.fs_seq_obj and self.fs_scatter_obj:
                run_fsviz = True
                if self.rposer is None:
                    if self.hm_obj is not None and self.fs_scatter_obj is not None and self.planning_world is not None:
                        self.rposer = RobotPoser( self.planning_world, self.hm_obj, fs_scatter=self.fs_scatter_obj)
                        run_fsviz = True
                    else:
                        run_fsviz = False
                if run_fsviz:
                    self.fs_seq_obj.visualize(self.fs_scatter_obj.get_scatter_list(), self.rposer, sleep_t_p_step=.25)

        if visualize_mplanner:

            if load_first:
                if self.control_loop_output_obj is None:
                    self.control_loop_output_obj = self.get_saved_object(SystemRunner.control_loop_type, print_dne=True, print_loaded=True)


            if self.control_loop_output_obj:
                if len(self.control_loop_output_obj.configs) > 0:

                    start_config_idx = 0
                    if skip_to_pct_through_cloop_output:
                        start_config_idx = int(skip_to_pct_through_cloop_output*len(self.control_loop_output_obj.configs))

                    robot = self.planning_world.robot(0)
                    sleep_t = .00125
                    robot.setConfig(self.control_loop_output_obj.configs[start_config_idx])

                    time.sleep(10)
                    for i in range(start_config_idx, len(self.control_loop_output_obj.configs)):
                        robot.setConfig(self.control_loop_output_obj.configs[i])
                        time.sleep(sleep_t)
                    # print(self.control_loop_output_obj.configs[len(self.control_loop_output_obj.configs)-1])
                    # robot.setConfig(self.control_loop_output_obj.configs[len(self.control_loop_output_obj.configs)-1])

        if self.hm_obj is not None:
            Logger.log(("Done visualizing " + self.world_name), "STANDARD")

    # __________________________________________________________________________________________________________________
    # -- File Saving/Loading/Deleting Functions

    def delete_maps(self):
        for obj_type in [SystemRunner.hm_type, SystemRunner.fs_costmap_type, SystemRunner.gradient_map_type,
                         SystemRunner.fs_convcostmap_type]:
            self.delete_saved_obj(obj_type)

    def delete_all_saved_data(self):
        for obj_type in [SystemRunner.hm_type, SystemRunner.fs_costmap_type, SystemRunner.gradient_map_type,
                         SystemRunner.fs_convcostmap_type, SystemRunner.hl_traj_type, SystemRunner.fs_scatter_type,
                         SystemRunner.fs_seq_type, SystemRunner.control_loop_type]:
            self.delete_saved_obj(obj_type)

    def delete_conv_cost_map(self):
        self.delete_saved_obj(SystemRunner.fs_convcostmap_type)

    def delete_scatter(self):
        self.delete_saved_obj(SystemRunner.fs_scatter_type)

    def delete_hl_traj(self):
        self.delete_saved_obj(SystemRunner.hl_traj_type)

    def delete_fs_seq(self):
        self.delete_saved_obj(SystemRunner.fs_seq_type)

    def delete_cloop_output(self):
        self.delete_saved_obj(SystemRunner.control_loop_type)

    def delete_cost_map(self):
        self.delete_saved_obj(SystemRunner.fs_costmap_type)

    def delete_saved_obj(self, type):
        file_name = self.save_root + self.get_hash(type) + self.world_name + type + ".pickle"
        if self.file_exists(file_name):
            os.remove(file_name)
            Logger.log("Deleted saved object of type "+type+" for "+self.world_name, color="FAIL", class_id=0, msg_type=2)
        else:
            Logger.log( type + " not saved for " + self.world_name, color="WARNING", class_id=0, msg_type=2)

    def get_hash(self, obj_type):

        hm_vars = self.HM_X_START + self.HM_X_END + project_constants.HM_X_GRANULARITY + self.HM_Y_START + \
                  self.HM_Y_END + project_constants.HM_Y_GRANULARITY

        start_end_vars = .23*self.xy_yaw0[0] + .93*self.xy_yaw0[2] + .9323*self.xy_yaw0[1] + 1.0323*self.xy_yawf[0] + \
                         1.0323*self.xy_yawf[1] + self.xy_yawf[2]**2

        hl_traj_vars = project_constants.HLTRAJ_G_WEIGHT + project_constants.HLTRAJ_H_WEIGHT + \
                       project_constants.HLTRAJ_YAW_OFFSET + project_constants.HLTRAJ_DELTAX + \
                       project_constants.HLTRAJ_DELTAY + project_constants.SEARCH_SPACE_X_MARGIN + \
                       project_constants.SEARCH_SPACE_Y_MARGIN + start_end_vars + \
                       12.23 * project_constants.BASE_STATE_END_EFF_DX_FROM_TORSO + \
                       project_constants.BASE_STATE_END_EFF_DY_FROM_TORSO ** 2

        fs_seq_vars = project_constants.STEPSEQ_G_WEIGHT + project_constants.STEPSEQ_H_WEIGHT + \
                      project_constants.STEPSEQ_TRANSLATION_DISTANCE + project_constants.STEP_ORDER[0] - \
                      project_constants.STEP_ORDER[3] + project_constants.STEPSEQ_IDEAL_LOCATION_COST_COEFF + \
                      project_constants.STEPSEQ_MAX_DIAGNOL_DIST + \
                      project_constants.STEPSEQ_MAX_FOOTHOLD_TO_COM_ALONG_FOOTHOLD_INCENTER_LINE_DIST + \
                      start_end_vars + project_constants.X_STEP_SIZE_FS_SCATTER + project_constants.Y_STEP_SIZE_FS_SCATTER + \
                      project_constants.STEPSEQ_KINEMATIC_INFEASIBILITY_CIRC_RADIUS_CENTERED_AT_ADJACENT_ENDEFFECTORS + \
                      project_constants.STEPSEQ_MAX_FOOTHOLD_TO_COM_ALONG_FOOTHOLD_INCENTER_LINE_DIST_d2 + \
                      project_constants.HOOK_LENGTH + project_constants.HOOK_DIST_TO_GROUND + \
                      project_constants.STEPSEQ_HOOK_SAFETY_MARGIN + \
                      project_constants.STEPSEQ_MAX_FOOTHOLD_TO_COM_ALONG_FOOTHOLD_INCENTER_LINE_DIST + \
                      project_constants.STEPSEQ_MAX_FOOTHOLD_TO_COM_ALONG_FOOTHOLD_INCENTER_LINE_DIST_d2 + \
                      project_constants.STEPSEQ_ZERO_COST_RADIUS ** 2 * 3 + \
                      project_constants.STEPSEQ_HEIGHT_NEAR_ENDEFF_COST_COEFF * .83 + \
                      project_constants.STEPSEQ_USE_Z_IN_MAX_DIST_CALCS + project_constants.STEPSEQ_HEIGHT_AROUND_ENDEFF_R

        cmap_vars = project_constants.CMAP_STEP_SIZEX + project_constants.CMAP_STEP_SIZEY + \
                    project_constants.CMAP_NORMALIZED_MAX_VALUE + project_constants.CMAP_SLOPE_HEURISTIC_COEFF + \
                    project_constants.CMAP_ROUGHNESS_HEURISTIC_COEFF + project_constants.CMAP_MAX_NONPENALIZED_SLOPE + \
                    project_constants.CMAP_SLOPE_HEURISTIC_MAX_CONSIDERED_DEGREE + \
                    project_constants.END_AFFECTOR_RADIUS + project_constants.COST_MAP_SEARCH_MARGIN_WIDTH + \
                    project_constants.CMAP_STEP_COSTFN_DIF_SLOPE_COEFF + project_constants.CMAP_STEP_COSTFN_SEARCH_SIZE + \
                    project_constants.CMAP_STEP_COSTFN_MIN_HEIGHT_DIF + project_constants.CMAP_STEP_COSTFN_BASELINE_COST

        if project_constants.USE_CONVMAP:
            fs_seq_vars += .1866

        if obj_type == SystemRunner.hm_type:
            hash_input = hm_vars

        elif obj_type == SystemRunner.fs_convcostmap_type:
            hash_input = cmap_vars + project_constants.CONV_X_WIDTH + project_constants.CONV_Y_WIDTH

        elif obj_type == SystemRunner.fs_costmap_type:
            hash_input = hm_vars + cmap_vars

        elif obj_type == SystemRunner.gradient_map_type:
            hash_input = hm_vars

        elif obj_type == SystemRunner.hl_traj_type:
            hash_input = hm_vars + hl_traj_vars

        elif obj_type == SystemRunner.fs_scatter_type:
            hash_input = hm_vars + start_end_vars + project_constants.X_STEP_SIZE_FS_SCATTER + \
                         project_constants.Y_STEP_SIZE_FS_SCATTER

        elif obj_type == SystemRunner.fs_seq_type:
            hash_input = hm_vars + start_end_vars + hl_traj_vars + fs_seq_vars

        elif obj_type == SystemRunner.control_loop_type:
            hash_input = hm_vars + start_end_vars + hl_traj_vars + fs_seq_vars + project_constants.END_RANGE_MULTIPLIER + \
                         project_constants.TORSO_Z_DESIRED
            if self.cloop_run_name:
                for letter in self.cloop_run_name:
                    hash_input += ord(letter)

        else:
            Logger.log(("Error, obj type:" + obj_type + " unrecognized"), color="FAIL", class_id=0, msg_type=1)
            return

        for i in self.world_name:
            hash_input += ord(i)

        return str(np.round(hash_input, 5)).replace(".", "-") + "_"

    def file_exists(self, file_name):
        if not os.path.isdir(self.save_root):
            Logger.log(f"Creating {self.world_name} directory", color="WARNING", class_id=0, msg_type=2)
            os.makedirs(self.save_root)
        return os.path.isfile(file_name)

    def save_obj(self, type, print_saved=True, print_not_saved=True, overwrite=False):
        '''
            input is object type, not object for overwriting purposes
        '''
        types = [SystemRunner.hm_type, SystemRunner.fs_costmap_type, SystemRunner.gradient_map_type, SystemRunner.fs_convcostmap_type,
                 SystemRunner.hl_traj_type, SystemRunner.fs_scatter_type, SystemRunner.fs_seq_type, SystemRunner.control_loop_type]
        objs = [self.hm_obj,  self.fs_cost_map_obj,  self.gradient_map_obj,   self.fs_convcostmap_obj, self.hl_traj_obj,
                self.fs_scatter_obj, self.fs_seq_obj, self.control_loop_output_obj]
        obj = objs[types.index(type)]
        file_name = self.save_root + self.get_hash(type) + self.world_name + type
        if obj is not None:

            if self.file_exists(file_name+".pickle"):
                if overwrite:
                    Logger.log("Overwriting previous height map", class_id=0, msg_type=2)
                    os.remove(file_name+".pickle")
                    obj.save(file_name, print_=print_saved)
                else:
                    if print_not_saved:
                        Logger.log(f"{type} already exists. aborting (set overwrite to True to ... overwrite object)", class_id=0, msg_type=2)
            else:
                obj.save(file_name, print_=print_saved)
        else:
            if print_not_saved:
                Logger.log( type+" is not build/loaded, nothing to save", color="WARNING", class_id=0, msg_type=2)

    def save_all(
            self, save_hm=True, save_cmap=True, save_gradmap=True, save_fs_convcostmap=True, save_fsscatter=True,
            savehl_traj=True, save_fsseq=True, save_cloopcfgs=True):

        if not os.path.isdir(self.save_root):
            os.makedirs(self.save_root)

        types = [SystemRunner.hm_type, SystemRunner.fs_costmap_type, SystemRunner.gradient_map_type,
                 SystemRunner.fs_convcostmap_type, SystemRunner.hl_traj_type, SystemRunner.fs_scatter_type,
                 SystemRunner.fs_seq_type, SystemRunner.control_loop_type]

        if not save_hm: types.remove(SystemRunner.hm_type)
        if not save_cmap: types.remove(SystemRunner.fs_costmap_type)
        if not save_gradmap: types.remove(SystemRunner.gradient_map_type)
        if not save_fs_convcostmap: types.remove(SystemRunner.fs_convcostmap_type)
        if not save_fsscatter: types.remove(SystemRunner.fs_scatter_type)
        if not savehl_traj: types.remove(SystemRunner.hl_traj_type)
        if not save_fsseq: types.remove(SystemRunner.fs_seq_type)
        if not save_cloopcfgs: types.remove(SystemRunner.control_loop_type)

        for type in types:
            self.save_obj(type, print_not_saved=False)

    def load(self, print_=True):
        self.hm_obj = self.get_saved_object(SystemRunner.hm_type, print_loaded=print_)
        self.fs_cost_map_obj = self.get_saved_object(SystemRunner.fs_costmap_type, print_loaded=print_)
        self.gradient_map_obj = self.get_saved_object(SystemRunner.gradient_map_type, print_loaded=print_)
        self.hl_traj_obj = self.get_saved_object(SystemRunner.hl_traj_type, print_loaded=print_)
        self.fs_seq_obj = self.get_saved_object(SystemRunner.fs_seq_type, print_loaded=print_)
        self.fs_scatter_obj = self.get_saved_object(SystemRunner.fs_scatter_type, print_loaded=print_)
        self.control_loop_output_obj = self.get_saved_object(SystemRunner.control_loop_type, print_loaded=print_)

    def get_saved_object(self, obj_type, print_loaded=True, print_dne=True):

        file_name = self.save_root + self.get_hash(obj_type) +self.world_name+ obj_type + ".pickle"

        t_start = time.time()

        if self.file_exists(file_name):
            try:
                ret_ = pickle.load(open(file_name, "r+b"))
            except EOFError as e:
                Logger.log("Error: pickle file" + file_name + " is corrupted - please delete it", color="FAIL", class_id=0, msg_type=1)
                return None
            if print_loaded:
                Logger.log(f"loaded {obj_type} for {self.world_name} world in { round( time.time() - t_start, 3)} (s)", class_id=0, msg_type=2)
            return ret_

        if print_dne:
            Logger.log(f"Could not find {obj_type} for {self.world_name}", color="WARNING", class_id=0, msg_type=2)
        return None
